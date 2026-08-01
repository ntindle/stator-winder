"""Focused tests for the fail-closed follower integration audit."""

from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_integration_audit as audit


class AggregateBoundaryFollowerIntegrationAuditTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.report = audit.analyze()

    def test_exact_carriage_transform_and_selection_map_absence(self):
        transform = self.report["exact_transform"]
        self.assertEqual(transform["owner"], "carriage")
        self.assertEqual(
            transform["reference_formula"],
            "machine(x,y,z)=(-local_y, local_z, 95-local_x)",
        )
        self.assertEqual(
            transform["carriage_owned_pose_formula"],
            "world=(-local_y, local_z, stator_axis_z(M0)-local_x)",
        )
        self.assertAlmostEqual(transform["m0_mm_per_rad"], 8 / (2 * math.pi))
        selection = self.report["selection_owner_map"]
        self.assertEqual(selection["M1_law_count"], 3)
        self.assertEqual(selection["M2_identity_count"], 4)
        self.assertTrue(selection["physical_occurrence_owner_map_absent"])
        self.assertFalse(self.report["gates"][
            "physical_M1_M2_selected_occurrence_owner_map_defined"])

    def test_21_reference_pairs_and_decisive_live_OCC_witnesses(self):
        scan = self.report["reference_pose_OCC_scan"]
        self.assertEqual(scan["positive_pair_count"], 21)
        self.assertEqual(len(scan["positive_pairs"]), 21)
        self.assertAlmostEqual(
            scan["principal"]["carrier_vs_existing_yoke_mm3"],
            1782.698995,
        )
        self.assertAlmostEqual(
            scan["principal"]["carrier_vs_revised_tower_mm3"], 1268.0,
        )
        live = self.report["live_decisive_OCC_witnesses"]
        self.assertAlmostEqual(
            live["carrier_vs_existing_yoke_common_mm3"],
            1782.6989950592267,
            places=6,
        )
        self.assertAlmostEqual(
            live["carrier_vs_revised_tower_common_mm3"], 1268.0,
            places=6,
        )
        self.assertAlmostEqual(
            live["spine_vs_revised_tower_common_mm3"], 740.0,
            places=6,
        )
        self.assertAlmostEqual(
            live["adapter_vs_revised_tower_common_mm3"], 544.0,
            places=6,
        )
        self.assertTrue(live["full_STEP_extent_is_not_spine_length"])

    def test_coarse_M2_witnesses_are_not_promoted_to_sweep_clearance(self):
        diagnostic = self.report["coarse_M2_diagnostic"]
        counts = {
            row["M2_deg"]: row["positive_pair_count"]
            for row in diagnostic["samples"]
        }
        self.assertEqual(counts, {0.0: 3, 90.0: 0, 180.0: 10, 270.0: 0})
        self.assertFalse(diagnostic["clearance_claimed"])
        self.assertGreaterEqual(len(diagnostic["limitations"]), 5)
        self.assertFalse(self.report["gates"][
            "coarse_M2_diagnostic_is_continuous_clearance_sweep"])
        existing = self.report["existing_collision_contract"]
        self.assertEqual(existing["existing_yoke_full_raw_sample_count"], 225775)
        self.assertEqual(existing["existing_yoke_full_raw_status"], "PASS")
        self.assertFalse(existing["transferable_to_follower"])

    def test_additive_integration_fails_closed_without_authority(self):
        report = self.report
        self.assertEqual(report["status"], "FAIL")
        self.assertFalse(report["additive_integration_feasible"])
        self.assertFalse(report["clearance_claimed"])
        self.assertFalse(report["assembly_integration_authorized"])
        self.assertFalse(report["collision_authorized"])
        self.assertFalse(report["wire_route_authorized"])
        self.assertFalse(report["production_authorized"])
        self.assertFalse(report["selected_release_modified"])
        self.assertFalse(report["gates"]["additive_integration_feasible"])
        self.assertGreaterEqual(len(report["minimum_implementation_plan"]), 6)

    def test_hash_binding_tamper_rejection_and_written_outputs(self):
        current = deepcopy(self.report)
        audit.validate_report_integrity(current)
        current["additive_integration_feasible"] = True
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            audit.validate_report_integrity(current)

        generated = audit.write_outputs(self.report)
        written = json.loads(audit.OUTPUT_JSON.read_text(encoding="utf-8"))
        self.assertEqual(generated["report_sha256"], written["report_sha256"])
        audit.validate_report_integrity(written)
        artifacts = written["artifacts"]
        self.assertEqual(len(artifacts["follower_STEP"]["sha256"]), 64)
        self.assertEqual(
            artifacts["follower_STEP"]["sha256"],
            "092db9a20b404af4a54f1df700f9c7831c1784edd28f9385bbf232b9b7eec6d0",
        )
        self.assertEqual(len(
            artifacts["integrated_release_candidate_STEP"]["sha256"]
        ), 64)


if __name__ == "__main__":
    unittest.main(verbosity=2)
