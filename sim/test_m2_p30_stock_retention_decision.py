"""Regression tests for the fail-closed P30 motor-shaft decision."""

from __future__ import annotations

import math
import unittest

import m2_p30_stock_retention_decision as decision


class P30StockRetentionDecisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = decision.analyze()

    def test_exact_vendor_artifacts_and_stock_configuration_are_bound(self) -> None:
        cad = self.report["exact_stock_CAD_and_drawing"]
        self.assertEqual(cad["part_number"], "P30-3GT-BLP-6C-5")
        self.assertEqual(cad["STEP_solid_count"], 1)
        self.assertEqual(cad["STEP_face_count"], 49)
        self.assertFalse(cad["contains_BNS_or_BNW"])
        self.assertFalse(cad["contains_keyway_or_taper_lock"])
        self.assertTrue(
            self.report["release_gates"][
                "exact_stock_NBK_STEP_and_drawing_hash_bound"
            ]
        )

    def test_route_torque_and_two_x_target_are_self_consistent(self) -> None:
        load = self.report["load_contract"]
        route = load["route"]
        self.assertTrue(
            math.isclose(
                route["wire_torque_at_10N_nm"],
                decision.WIRE_TENSION_N
                * route["max_projected_moment_arm_mm"]
                / 1000.0,
                rel_tol=0.0,
                abs_tol=1.0e-12,
            )
        )
        expected = decision.REQUIRED_RETENTION_MULTIPLE * (
            route["wire_torque_at_10N_nm"]
            + decision.FRICTION_ALLOWANCE_NM
            + decision.ANGULAR_ACCELERATION_RAD_S2
            * route["final_full_output_inertia_kg_m2"]
        )
        self.assertAlmostEqual(load["required_2x_reversing_retention_nm"], expected)
        self.assertGreater(
            load["required_2x_reversing_retention_nm"],
            load["wire_plus_friction_2x_floor_excluding_inertia_nm"],
        )

    def test_provisional_route_never_claims_final_binding(self) -> None:
        route = self.report["load_contract"]["route"]
        if route["authority"].startswith("provisional"):
            self.assertFalse(route["canonical_route_and_final_inertia_bound"])
            self.assertIsNone(route["canonical_locus_sha256"])
            self.assertEqual(
                route["provisional_row_sha256"], decision.PROVISIONAL_ROUTE_ROW_HASH
            )

    def test_belt_capacity_and_bolt_torque_are_not_retention_ratings(self) -> None:
        separation = self.report["interface_separation"]
        belt = separation["belt_and_tooth_transmission"]
        self.assertTrue(belt["passes_belt_capacity_math"])
        self.assertFalse(belt["proves_pulley_to_shaft_retention"])
        clamp = separation["stock_clamp_fastener"]
        self.assertEqual(clamp["installation_tightening_torque_nm"], 0.5)
        self.assertFalse(clamp["is_output_retention_rating"])
        self.assertTrue(clamp["must_not_be_compared_numerically_to_output_torque"])

    def test_all_compared_retention_options_remain_fail_closed(self) -> None:
        by_id = {row["id"]: row for row in self.report["options"]}
        self.assertEqual(
            set(by_id),
            {
                "stock_split_clamp",
                "stock_plus_BNS",
                "stock_plus_BNW",
                "keyed_or_taper_lock_P30",
            },
        )
        for row in by_id.values():
            self.assertIsNone(row["published_P30_shaft_slip_rating_nm"])
            self.assertFalse(row["proves_required_2x_retention"])
            self.assertFalse(row["production_release"])
        self.assertEqual(
            self.report["decision"]["recommended_configuration"], "stock_plus_BNW"
        )

    def test_no_stock_keyed_or_taper_lock_drop_in_is_invented(self) -> None:
        option = next(
            row
            for row in self.report["options"]
            if row["id"] == "keyed_or_taper_lock_P30"
        )
        self.assertEqual(option["orderability"], "no exact stock drop-in configuration found")
        self.assertIn("tapered keyway", option["additional_machining"])
        self.assertFalse(self.report["Leadshine_shaft"]["keyway_present"])
        self.assertFalse(self.report["decision"]["keyed_or_taper_lock_drop_in_exists"])

    def test_reference_chart_cannot_release_BNS_or_BNW(self) -> None:
        chart = self.report["NBK_reference_slip_chart"]
        self.assertTrue(chart["reference_only_not_guaranteed"])
        self.assertTrue(chart["NBK_requires_actual_use_testing"])
        self.assertFalse(chart["P30_BNS_delivered_screw_size_known"])
        self.assertFalse(chart["P30_BNW_delivered_screw_size_known"])
        self.assertFalse(chart["Leadshine_material_and_hardness_match_known"])
        self.assertFalse(chart["can_prove_any_compared_configuration"])

    def test_overall_decision_is_not_procurement_or_production_authorization(self) -> None:
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["production_procurement_authorized"])
        self.assertTrue(self.report["prototype_coupon_purchase_recommended"])
        self.assertFalse(
            self.report["decision"]["stock_split_clamp_can_be_released_now"]
        )
        self.assertFalse(
            self.report["decision"]["lower_live_line_torque_changes_release_answer"]
        )


if __name__ == "__main__":
    unittest.main()
