"""Tests for the isolated phase-aware progressive moving-wire audit."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import phase_aware_progressive_wire_audit as audit
from traj import Timeline, load_events


class PhaseAwareProgressiveWireAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.events = load_events(audit.CAPTURE)
        cls.timeline = Timeline(cls.events)
        cls.loci, cls.passes = audit.extract_raw_loci(
            cls.events, cls.timeline,
        )

    def test_canonical_raw_coverage_and_signs(self) -> None:
        self.assertEqual(len(self.passes), 24)
        self.assertEqual(len(self.loci), 2400)
        self.assertEqual(
            sum(row.motion_sign < 0 for row in self.loci), 1200,
        )
        self.assertEqual(
            sum(row.motion_sign > 0 for row in self.loci), 1200,
        )
        self.assertLessEqual(
            max(abs(row.m1_alignment_error_rad) for row in self.loci),
            audit.M1_ALIGNMENT_TOL_RAD,
        )

    def test_progressive_other_tooth_population_is_pass_ordered(self) -> None:
        self.assertEqual(
            self.passes[0]["completed_other_teeth_before"], [],
        )
        for index, row in enumerate(self.passes):
            self.assertEqual(
                len(row["completed_other_teeth_before"]), index,
            )
            self.assertNotIn(
                row["tooth_index"], row["completed_other_teeth_before"],
            )
        self.assertEqual(
            len(self.passes[-1]["completed_other_teeth_before"]), 23,
        )

    def test_loci_bind_raw_pose_to_turn_and_half_turn(self) -> None:
        for pass_index in range(24):
            rows = self.loci[
                pass_index * 100:(pass_index + 1) * 100
            ]
            self.assertEqual([row.state_index for row in rows], list(range(100)))
            self.assertEqual([row.turn_index for row in rows], [
                index // 2 for index in range(100)
            ])
            self.assertEqual([row.half_turn_index for row in rows], [
                index & 1 for index in range(100)
            ])

    def test_allowed_contact_does_not_erase_independent_collision(self) -> None:
        allowed = audit.classify_copper_probe({
            "minimum_centerline_distance_mm": 0.22352,
            "minimum_obstacle_id": "active-support",
        }, 0.22352)
        forbidden = audit.classify_copper_probe({
            "minimum_centerline_distance_mm": 0.0,
            "minimum_obstacle_id": "second-obstacle",
        }, 0.22352)
        self.assertEqual(
            allowed["classification"],
            "NONPENETRATING_SUPPORT_OR_GLIDE_CONTACT",
        )
        self.assertFalse(allowed["interpenetration"])
        self.assertTrue(forbidden["interpenetration"])
        self.assertEqual(
            forbidden["classification"],
            "CENTERLINE_INTERPENETRATION_OR_THROUGH_CROSSING",
        )

    def test_empty_progressive_class_is_explicit(self) -> None:
        row = audit.classify_copper_probe({
            "minimum_centerline_distance_mm": None,
            "minimum_obstacle_id": None,
        }, 0.22352)
        self.assertEqual(row["classification"], "NO_PRIOR_COPPER_IN_CLASS")
        self.assertFalse(row["interpenetration"])

    def test_input_contract_validates_elastic_3d_integrity(self) -> None:
        manifest = audit._load_json(audit.MANIFEST)
        packing = audit._load_json(audit.PACKING)
        insulation = audit._load_json(audit.INSULATION)
        elastic = audit._load_json(audit.ELASTIC_3D)
        with patch.object(
            audit.elastic_3d_study, "validate_report_integrity",
        ) as validate:
            contract = audit._input_contract(
                self.events, manifest, packing, insulation, elastic,
            )
        validate.assert_called_once_with(elastic)
        self.assertTrue(contract["checks"][
            "elastic_3d_integrity_current"
        ])

    def test_input_contract_rejects_invalid_elastic_3d_report(self) -> None:
        manifest = audit._load_json(audit.MANIFEST)
        packing = audit._load_json(audit.PACKING)
        insulation = audit._load_json(audit.INSULATION)
        elastic = audit._load_json(audit.ELASTIC_3D)
        elastic["status"] = "PASS"
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            audit._input_contract(
                self.events, manifest, packing, insulation, elastic,
            )

    def test_checked_in_report_is_hash_bound_and_fail_closed(self) -> None:
        self.assertTrue(audit.OUTPUT_JSON.is_file())
        report = json.loads(audit.OUTPUT_JSON.read_text(encoding="utf-8"))
        audit.validate_report_integrity(report)
        self.assertEqual(report["scope"]["locus_count"], 2400)
        self.assertEqual(report["status"], "FAIL")
        self.assertFalse(report["production_authorized"])
        self.assertFalse(report["assembly_integration_authorized"])
        self.assertGreater(
            report["per_class_summary"]["nominal_ellipse_tangent"]
            ["core_and_liner"]["collision_locus_count"],
            0,
        )
        self.assertEqual(
            report["related_R3_elastic_evidence"]["file_sha256"],
            audit._sha256(audit.ELASTIC_3D),
        )

    def test_exact_packing_match_is_not_a_gate(self) -> None:
        report = json.loads(audit.OUTPUT_JSON.read_text(encoding="utf-8"))
        gate_names = set(report["semantic_gates"])
        self.assertFalse(any("schedule" in name for name in gate_names))
        self.assertFalse(report["release_flags"][
            "exact_layer_neatness_required"
        ])


if __name__ == "__main__":
    unittest.main()
