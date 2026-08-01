"""Focused tests for the robust g=0 landing redesign trade."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_g0_landing_trade as trade


class AggregateBoundaryFollowerG0LandingTradeTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.report = trade.analyze()
        cls.by_id = {
            row["id"]: row for row in cls.report["candidates"]
        }

    def test_PEEK_shelf_dimensions_and_diameter_rebound_contract(self):
        peek = self.by_id["diameter_rebound_integral_PEEK_cap_side_shelf"]
        self.assertTrue(peek["selected"])
        self.assertEqual(peek["status"], "SELECTED_FOR_DETAILED_REDESIGN")
        self.assertEqual(peek["contact_normal_active_local"], [0.0, 1.0, 0.0])
        self.assertEqual(peek["shelf_dimensions_mm"], {
            "radial_cap_side_length": 1.5,
            "axial_width": 0.75,
            "minimum_stock_behind_contact_face": 0.3,
            "ends_at_seam_X": 12.687039228440508,
        })
        mouth = peek["mouth_dimensions_mm"]
        self.assertEqual(mouth["radial_length"], 2.4)
        self.assertEqual(mouth["tangential_width"], 1.0)
        self.assertEqual(mouth["axial_span"], 0.9)
        self.assertAlmostEqual(mouth["center_y"], 0.9436710365709817)

        rows = peek["diameter_endpoint_contract"]
        self.assertEqual([row["wire_diameter_mm"] for row in rows], [0.2, 0.5])
        self.assertAlmostEqual(rows[0]["endpoint_active_local_mm"][1],
                               0.8686710365709818)
        self.assertAlmostEqual(rows[1]["endpoint_active_local_mm"][1],
                               1.0186710365709817)
        self.assertAlmostEqual(rows[0]["tangential_rebind_from_current_endpoint_mm"],
                               -0.03425624516932224)
        self.assertAlmostEqual(rows[1]["tangential_rebind_from_current_endpoint_mm"],
                               0.11574375483067767)
        self.assertAlmostEqual(rows[0]["minimum_R3p5_two_arc_X_run_mm"],
                               0.6916747371687147)
        self.assertAlmostEqual(rows[1]["minimum_R3p5_two_arc_X_run_mm"],
                               1.267681328586638)

    def test_modified_front_rear_BREP_witnesses_cover_both_diameters(self):
        peek = self.by_id["diameter_rebound_integral_PEEK_cap_side_shelf"]
        cases = peek["exact_modified_BREP_cases"]
        self.assertEqual(len(cases), 2)
        for case in cases:
            self.assertEqual(case["modified_cap_solid_count"], 1)
            self.assertTrue(case["shelf_positive_fusion"])
            self.assertGreater(
                case["shelf_to_before_cap_positive_overlap_mm3"], 0.0)
            self.assertEqual(len(case["diameter_cases"]), 2)
            for row in case["diameter_cases"]:
                self.assertTrue(row["endpoint_distance_equals_wire_radius"])
                self.assertTrue(row["wire_tangent_without_positive_overlap"])
                self.assertTrue(row["R0p36_gauge_clear"])
                self.assertEqual(
                    row["shelf_to_cap_side_wire_positive_overlap_mm3"], 0.0)
                self.assertEqual(
                    row["modified_cap_to_R0p36_gauge_positive_overlap_mm3"], 0.0)
            self.assertEqual(case["status"], "PASS")
        self.assertTrue(all(self.report["geometry_gates"].values()))

    def test_Nomex_and_shifted_cut_are_rejected_with_exact_bounds(self):
        nomex = self.by_id[
            "installed_0p127mm_Nomex_BREP_with_route_rebind"
        ]
        self.assertFalse(nomex["selected"])
        self.assertEqual(nomex["sheet_thickness_mm"], 0.127)
        rows = nomex["diameter_endpoint_contract"]
        self.assertAlmostEqual(rows[0]["endpoint_active_local_mm"][1],
                               0.9956710365709818)
        self.assertAlmostEqual(rows[1]["endpoint_active_local_mm"][1],
                               1.145671036570982)
        self.assertAlmostEqual(rows[0]["minimum_R3p5_two_arc_X_run_mm"],
                               1.135698535514331)
        self.assertAlmostEqual(rows[1]["minimum_R3p5_two_arc_X_run_mm"],
                               1.8274266160697656)

        shifted = self.by_id["right_seam_mouth_cut_shift_plus_0p30mm"]
        self.assertFalse(shifted["selected"])
        self.assertFalse(shifted["all_cases_pass"])
        for case in shifted["exact_cases"]:
            self.assertEqual(case["candidate_solid_count"], 1)
            self.assertAlmostEqual(
                case["original_endpoint_to_candidate_surface_mm"],
                0.11176,
                places=9,
            )
            self.assertGreater(
                case["candidate_to_R0p36_gauge_positive_overlap_mm3"], 0.03)
            self.assertEqual(case["status"], "FAIL")

    def test_balance_lane_width_and_force_gates_remain_fail_closed(self):
        balance = self.report["mass_and_balance_estimate"]
        self.assertEqual(balance["physical_shelf_count_if_24fold_front_and_rear"], 48)
        self.assertTrue(balance[
            "first_moment_cancels_by_24fold_front_rear_symmetry"])
        self.assertLess(
            balance["maximum_total_added_PEEK_mass_g_before_mouth_accounting"],
            0.014,
        )
        inputs = self.report["inputs"]
        self.assertEqual(inputs["current_cap_lane_clear_width_mm"], 0.47752)
        self.assertTrue(inputs["current_cap_lane_is_narrower_than_0p5mm_wire"])
        self.assertFalse(self.report["force_normal_gate"][
            "incident_tension_resultant_compressive_at_all_48"])
        self.assertEqual(self.report["status"], "FAIL")
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["wire_route_authorized"])
        self.assertFalse(self.report["collision_authorized"])
        self.assertFalse(self.report["assembly_integration_authorized"])
        self.assertFalse(self.report["selected_release_modified"])
        self.assertFalse(any(self.report["release_gates"].values()))

    def test_hash_binding_tamper_rejection_and_written_outputs(self):
        current = deepcopy(self.report)
        trade.validate_report_integrity(current)
        current["wire_route_authorized"] = True
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            trade.validate_report_integrity(current)

        generated = trade.write_outputs(self.report)
        written = json.loads(trade.OUTPUT_JSON.read_text(encoding="utf-8"))
        self.assertEqual(generated["report_sha256"], written["report_sha256"])
        trade.validate_report_integrity(written)
        self.assertEqual(len(
            written["artifacts"]["g0_normal_audit"]["sha256"]
        ), 64)
        self.assertEqual(len(written["source_hashes"]), 4)


if __name__ == "__main__":
    unittest.main(verbosity=2)
