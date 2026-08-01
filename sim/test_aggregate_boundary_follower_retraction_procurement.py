"""Focused tests for follower retraction procurement evidence."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_retraction_procurement as procurement


class AggregateBoundaryFollowerRetractionProcurementTests(unittest.TestCase):

    def test_shaft_requires_exact_cut_and_bushing_requires_new_pocket(self):
        shaft = procurement.shaft_selection()
        bushing = procurement.bushing_selection()

        self.assertEqual(
            shaft["selected_purchase_candidate"]["catalog_number"], "5033N11"
        )
        self.assertEqual(shaft["fit_calculation"]["length_excess_to_remove_mm"], 9.0)
        self.assertFalse(shaft["exact_drop_in_fit"])
        self.assertEqual(
            bushing["selected_candidate"]["catalog_number"], "WPFFM-0304-05"
        )
        self.assertEqual(bushing["selected_candidate"]["body_OD_d2_mm"], 4.5)
        self.assertEqual(bushing["selected_candidate"]["flange_OD_d3_mm"], 7.5)
        self.assertEqual(bushing["fit_calculation"]["bearing_length_delta_from_target_mm"], -1.0)
        self.assertFalse(bushing["exact_drop_in_fit"])

    def test_both_stock_tangential_springs_remain_rejected(self):
        result = procurement.tangential_spring_selection()
        rows = {row["catalog_number"]: row for row in result["rejected_candidates"]}

        self.assertIsNone(result["selected_stock_candidate"])
        self.assertEqual(rows["S-1576CS"]["shaft_interference_nominal_mm"], 0.23)
        self.assertEqual(
            rows["S-1576CS"]["force_calculation"]["net_centering_stiffness_N_per_mm"],
            0.22,
        )
        self.assertAlmostEqual(
            rows["S-1576CS"]["force_calculation"]["center_preload_each_N"],
            0.1199,
            places=8,
        )
        self.assertEqual(
            rows["B-50CS"]["fit_calculation"]["deflection_beyond_suggested_maximum_mm"],
            13.02,
        )
        self.assertAlmostEqual(
            rows["B-50CS"]["fit_calculation"]["center_preload_each_N"],
            4.0278,
            places=8,
        )
        self.assertTrue(all(row["status"].startswith("REJECT") for row in rows.values()))

    def test_independent_return_stock_candidate_exceeds_force_ceiling(self):
        result = procurement.independent_return_selection()
        budget = result["force_budget"]
        candidate = result["rejected_stock_candidate"]

        self.assertAlmostEqual(budget["remaining_force_at_hard_extension_N"], 0.303291)
        self.assertAlmostEqual(
            budget["maximum_effective_rate_N_per_mm"], 0.008326719, places=9
        )
        self.assertGreater(
            candidate["minimum_tolerance_load_N"],
            budget["remaining_force_at_hard_extension_N"],
        )
        self.assertIsNone(result["selected_stock_candidate"])

    def test_D4F_is_remote_only_and_not_direct_follower_fit(self):
        result = procurement.safety_switch_selection()

        self.assertEqual(result["candidate"]["catalog_number"], "D4F-120-1R")
        self.assertEqual(result["candidate"]["quantity"], 2)
        self.assertTrue(result["candidate"]["direct_opening"])
        self.assertEqual(
            result["force_calculation"]["two_switch_normal_operating_reaction_max_N"],
            10.0,
        )
        self.assertEqual(
            result["force_calculation"]["two_switch_direct_opening_reaction_min_N"],
            40.0,
        )
        self.assertFalse(result["direct_slide_actuation_feasible"])
        self.assertIsNone(result["selected_direct_fit_candidate"])

    def test_report_is_evidence_complete_but_authority_false(self):
        report = procurement.build_report()
        procurement.validate_report_integrity(report)
        self.assertEqual(report["report_sha256"], procurement._canonical_hash(report))

        self.assertEqual(report["status"], "PROCUREMENT_NO_GO_CUSTOM_HARDWARE_REQUIRED")
        self.assertTrue(all(report["evidence_gates"].values()))
        self.assertTrue(all(value is False for value in report["fail_closed_gates"].values()))
        for gate in (
            "physical_procurement_authority",
            "BOM_change_authorized",
            "order_authorized",
            "CAD_change_authorized",
            "assembly_integration_authorized",
            "release_authorized",
        ):
            self.assertFalse(report[gate], gate)

        markdown = procurement._markdown(report)
        self.assertIn("Remove 9.0 mm and restore the chamfer", markdown)
        self.assertIn("Both rejected", markdown)
        self.assertIn("Remote-only", markdown)
        self.assertIn("Do not add these lines to the BOM", markdown)


if __name__ == "__main__":
    unittest.main()
