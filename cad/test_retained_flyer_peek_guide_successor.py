"""Regression gates for the physical retained-flyer PEEK guide."""

from __future__ import annotations

import json
import unittest

import retained_flyer_peek_guide_successor as guide


class RetainedFlyerPeekGuideSuccessorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = json.loads(guide.JSON_OUT.read_text(encoding="utf-8"))

    def test_source_level_integration_api_is_explicit(self) -> None:
        api = self.report["integration_api"]
        self.assertEqual(api["revised_arm"], "revised_retained_arm()")
        self.assertEqual(api["guide_insert"], "peek_guide_insert()")
        self.assertEqual(api["guide_hardware"], "guide_retention_hardware()")
        self.assertEqual(api["exit_bell"], "bell_fairlead()")
        self.assertEqual(api["root_sleeve"], "successor_root_sleeve()")
        self.assertIn(
            "solve_successor_balance_with_base_rows",
            api["merged_balance_solver"],
        )
        self.assertIn("guide_bore_centerline_samples", api["bore_centerline_samples"])
        self.assertEqual(api["frame"], "retained flyer local; M2 axis +Z")

    def test_guide_and_revised_arm_are_each_one_solid(self) -> None:
        self.assertEqual(self.report["guide"]["solid_count"], 1)
        self.assertEqual(self.report["revised_printed_arm"]["solid_count"], 1)
        self.assertTrue(
            self.report["geometry_gates"]["PEEK_guide_exactly_one_solid"]
        )
        self.assertTrue(
            self.report["geometry_gates"]
            ["revised_printed_arm_exactly_one_solid"]
        )

    def test_bore_wander_still_meets_R3_for_full_wire_range(self) -> None:
        row = self.report["guide"]
        self.assertEqual(row["wire_diameter_range_mm"], [0.2, 0.5])
        self.assertAlmostEqual(row["centerline_elbow_radius_mm"], 3.25)
        self.assertAlmostEqual(
            row["minimum_supported_wire_center_radius_after_bore_wander_mm"],
            3.05,
        )
        self.assertTrue(
            self.report["geometry_gates"]
            ["all_supported_wire_positions_remain_R_ge_3mm"]
        )

    def test_job_and_max_wire_have_zero_petg_or_peek_intrusion(self) -> None:
        checks = self.report["exact_BREP_checks"]
        for name in (
            "job_wire_to_revised_arm_intersection_mm3",
            "max_wire_to_revised_arm_intersection_mm3",
            "job_wire_to_PEEK_insert_material_intersection_mm3",
            "max_wire_to_PEEK_insert_material_intersection_mm3",
        ):
            with self.subTest(name=name):
                self.assertLessEqual(checks[name], 1.0e-8)

    def test_open_seat_preserves_load_bearing_spoke(self) -> None:
        arm = self.report["revised_printed_arm"]
        self.assertGreaterEqual(arm["remaining_spoke_floor_mm"], 6.0)
        self.assertGreaterEqual(arm["remaining_side_web_each_mm"], 5.0)
        self.assertGreater(arm["seat_and_pilot_removed_volume_mm3"], 0.0)
        self.assertTrue(
            self.report["geometry_gates"]
            ["guide_seat_does_not_cut_counterweight_stacks"]
        )
        self.assertEqual(
            self.report["exact_BREP_checks"]
            ["guide_seat_to_counterweight_stack_intersection_mm3"],
            0.0,
        )

    def test_actual_caps_clear_isolated_guide_successor(self) -> None:
        checks = self.report["exact_BREP_checks"]
        self.assertGreater(
            checks["actual_PEEK_caps_to_revised_arm_distance_mm"], 0.0
        )
        self.assertGreater(
            checks["actual_PEEK_caps_to_guide_insert_distance_mm"], 0.0
        )

    def test_bell_is_physical_one_piece_and_not_torus_metadata(self) -> None:
        bell = self.report["exit_bell"]
        self.assertEqual(bell["geometry"],
                         "one-piece axisymmetric exposed PEEK fairlead")
        self.assertGreaterEqual(
            bell["minimum_wire_center_radius_over_0p20_to_0p50mm_wire_mm"],
            3.25,
        )
        self.assertGreaterEqual(bell["minimum_finished_wall_mm"], 1.8)
        self.assertTrue(bell["externally_accessible_for_polish_and_gauge"])
        self.assertFalse(bell["hidden_curved_bore"])
        self.assertNotIn("torus", json.dumps(bell).lower())

    def test_shaft_bore_and_counterrail_bypass_are_real(self) -> None:
        self.assertEqual(
            self.report["exact_BREP_checks"]
            ["OD12_shaft_envelope_to_revised_arm_intersection_mm3"],
            0.0,
        )
        self.assertTrue(
            self.report["geometry_gates"]
            ["counterrail_has_two_3mm_shaft_bypass_cheeks"]
        )

    def test_root_sleeve_is_positive_load_path_not_open_air(self) -> None:
        root = self.report["root_sleeve_load_path"]
        self.assertEqual(root["root_sleeve_solid_count"], 1)
        self.assertEqual(root["final_arm_solid_count"], 1)
        self.assertAlmostEqual(root["radial_ligament_mm"], 2.95)
        self.assertGreater(
            root["sleeve_to_existing_collar_overlap_mm3"], 0.0
        )
        self.assertGreater(root["sleeve_to_main_spoke_overlap_mm3"], 0.0)
        self.assertGreater(
            root["sleeve_to_existing_rear_counterrail_overlap_mm3"], 0.0
        )
        self.assertTrue(all(
            value > 0.0
            for value in root["sleeve_to_bypass_cheek_overlap_mm3"]
        ))
        load = root["conservative_combined_root_load_case"]
        self.assertEqual(load["review_safety_factor"], 3.0)
        self.assertTrue(load["passes_review_allowable"])
        self.assertFalse(
            load["orientation_matched_physical_coupon_complete"]
        )

    def test_isolated_balance_is_not_final_drive_authority(self) -> None:
        self.assertIn(
            "LEGACY_DRIVE_CONTEXT_ONLY",
            self.report["six_slug_balance"]["authority"],
        )

    def test_six_slug_balance_and_front_trim_retention(self) -> None:
        balance = self.report["six_slug_balance"]
        trim = self.report["front_balance_trim"]
        self.assertEqual(guide.FRONT_TRIM_PILOT_BOTTOM_Z_MM, -19.75)
        self.assertLessEqual(balance["scaled_balance_residual_norm"], 1.0e-6)
        self.assertEqual(len(balance["rear_slug_lengths_mm"]), 4)
        self.assertGreaterEqual(
            balance["minimum_rear_slug_margin_to_0p35mm_mm"], 0.25
        )
        self.assertGreaterEqual(trim["screw_tip_clearance_behind_insert_mm"],
                                0.5)
        self.assertGreaterEqual(trim["blind_printed_material_behind_pilot_mm"],
                                2.4)
        self.assertGreaterEqual(trim["minimum_outer_radial_printed_wall_mm"],
                                1.5)
        self.assertTrue(all(
            value <= 1.0e-8
            for value in trim["screw_to_arm_intersection_mm3"]
        ))
        self.assertFalse(trim["pull_coupon_complete"])
        self.assertFalse(trim["300rpm_endurance_complete"])

    def test_positive_retention_is_modeled_but_physical_proof_is_open(self) -> None:
        retention = self.report["retention"]
        self.assertEqual(retention["ear_count"], 3)
        self.assertIn("M2x6", retention["hardware"])
        self.assertFalse(retention["physical_pull_and_endurance_complete"])
        self.assertTrue(
            self.report["geometry_gates"]
            ["three_positive_retention_screw_and_insert_stacks"]
        )

    def test_release_remains_fail_closed_on_terminal_route(self) -> None:
        self.assertEqual(
            self.report["status"],
            "GEOMETRY_PASS_REVIEW_ONLY__TERMINAL_ROUTE_FAIL",
        )
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["assembly_integration_authorized"])
        self.assertTrue(all(self.report["geometry_gates"].values()))
        self.assertFalse(
            self.report["release_gates"]
            ["full_2400_locus_terminal_route_pass"]
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
