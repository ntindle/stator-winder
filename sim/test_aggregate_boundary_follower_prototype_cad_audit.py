"""Focused tests for the consolidated isolated-prototype CAD audit."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_prototype_cad_audit as audit


class AggregateBoundaryFollowerPrototypeCadAuditTests(unittest.TestCase):

    def test_g0_step_manifest_trade_and_local_geometry_are_bound(self):
        result = audit.g0_cap_shelf_evidence()
        geometry = result["established_geometry"]

        self.assertEqual(result["artifact"]["sha256"], audit._sha256(audit.G0_STEP))
        self.assertEqual(result["artifact"]["byte_count"], audit.G0_STEP.stat().st_size)
        self.assertEqual(geometry["cap_solid_counts"], {"front": 1, "rear": 1})
        self.assertEqual(geometry["lane_clear_width_mm"], 0.65)
        self.assertEqual(geometry["wire_diameter_cases_mm"], [0.2, 0.5])
        self.assertEqual(len(geometry["diameter_cases"]), 4)
        for row in geometry["diameter_cases"]:
            self.assertAlmostEqual(
                row["endpoint_to_cap_distance_mm"],
                row["wire_diameter_mm"] / 2.0,
                places=8,
            )
            self.assertEqual(row["cap_to_wire_positive_overlap_mm3"], 0.0)
            self.assertEqual(row["cap_to_R0p36_gauge_positive_overlap_mm3"], 0.0)
        self.assertTrue(all(result["evidence_gates"].values()))
        self.assertFalse(result["selection_authority"])
        self.assertFalse(result["clearance_authority"])
        self.assertFalse(result["complete_2400_route_authority"])

    def test_return_step_manifest_screen_dimensions_and_states_are_bound(self):
        result = audit.custom_return_package_evidence()
        geometry = result["established_geometry"]

        self.assertEqual(
            result["artifact"]["sha256"], audit._sha256(audit.RETURN_STEP)
        )
        self.assertEqual(geometry["STEP_leaf_body_count"], 15)
        self.assertEqual(geometry["shaft"], {
            "diameter_mm": 3.0, "length_mm": 16.0, "axis": "+Y",
        })
        self.assertEqual(geometry["igus_WPFFM_0304_05"], {
            "ID_mm": 3.0,
            "body_OD_mm": 4.5,
            "body_length_mm": 5.0,
            "flange_OD_mm": 7.5,
            "flange_thickness_mm": 0.75,
        })
        self.assertEqual(geometry["McMaster_9293K122_envelope"], {
            "coil_OD_mm": 15.75, "coil_width_mm": 6.35,
        })
        rows = geometry["same_state_overlap_rows"]
        self.assertEqual(
            {row["tangential_q_mm"] for row in rows}, {-0.6, 0.0, 0.6}
        )
        self.assertTrue(all(row["all_part_leaf_count"] == 15 for row in rows))
        self.assertTrue(all(row["all_part_positive_overlap_count"] == 0 for row in rows))
        self.assertTrue(all(row["custom_body_positive_overlap_count"] == 0 for row in rows))
        self.assertTrue(all(result["evidence_gates"].values()))
        self.assertIsNone(
            result["analytical_screen_context"]["production_selection"]
        )

    def test_report_binds_both_artifact_hashes_and_never_imports_acceptance(self):
        report = audit.build_report()
        source_text = Path(audit.__file__).read_text(encoding="utf-8")

        self.assertEqual(report["prototype_count"], 2)
        self.assertEqual(
            report["artifacts"]["aggregate_boundary_g0_cap_shelf_STEP"]["sha256"],
            audit._sha256(audit.G0_STEP),
        )
        self.assertEqual(
            report["artifacts"]
            ["aggregate_boundary_follower_custom_return_packaging_STEP"]
            ["sha256"],
            audit._sha256(audit.RETURN_STEP),
        )
        self.assertNotIn("import aggregate_boundary_follower_acceptance", source_text)
        self.assertNotIn("from aggregate_boundary_follower_acceptance", source_text)
        self.assertTrue(report["evidence_gates"]["no_acceptance_or_release_input_imported"])

    def test_all_broader_authorities_remain_false(self):
        report = audit.build_report()
        self.assertTrue(all(report["evidence_gates"].values()))
        self.assertTrue(
            all(value is False for value in report["fail_closed_gates"].values())
        )
        for key in (
            "selection_authority",
            "load_authority",
            "spring_rate_authority",
            "fatigue_authority",
            "clearance_authority",
            "complete_2400_route_authority",
            "integration_authority",
            "procurement_authority",
            "BOM_change_authorized",
            "order_authorized",
            "production_authority",
            "release_authority",
        ):
            self.assertFalse(report[key], key)

    def test_report_integrity_and_markdown_hashes(self):
        report = audit.build_report()
        audit.validate_report_integrity(report)
        self.assertEqual(report["report_sha256"], audit._canonical_hash(report))
        markdown = audit._markdown(report)
        self.assertIn(audit._sha256(audit.G0_STEP), markdown)
        self.assertIn(audit._sha256(audit.RETURN_STEP), markdown)
        self.assertIn("Local zero positive-volume overlap is not general clearance", markdown)


if __name__ == "__main__":
    unittest.main()
