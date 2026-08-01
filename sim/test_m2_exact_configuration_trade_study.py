"""Focused regression tests for the review-only M2 trade study."""

from __future__ import annotations

import json
from pathlib import Path
import unittest

import m2_exact_configuration_trade_study as study


class M2ExactConfigurationTradeStudyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = study.analyze()

    def test_report_is_review_only_and_cannot_change_authority(self) -> None:
        self.assertEqual(
            self.report["status"],
            "REVIEW_ONLY__NO_ELIGIBLE_EXACT_PRODUCTION_CONFIGURATION",
        )
        self.assertTrue(self.report["review_only"])
        for key in (
            "normal_GOAL_modified",
            "selected_CAD_modified",
            "selected_BOM_modified",
            "selected_load_authority_modified",
            "controller_or_firmware_modified",
            "integration_authorized",
            "procurement_authorized",
            "production_authorized",
        ):
            self.assertFalse(self.report[key], key)
        self.assertIsNone(self.report["decision"]["eligible_alternative"])
        self.assertFalse(
            self.report["decision"]["selected_configuration_changed"]
        )

    def test_exact_driver_current_has_no_released_integer_setting(self) -> None:
        lane = self.report["alternatives"]["exact_CS_D508_current"]
        pr35 = lane["CS_D508_PR2"]["PR2_35"]
        pr36 = lane["CS_D508_PR2"]["PR2_36"]
        self.assertFalse(
            lane["CS_D508_PR2"]["exact_curve_equivalent_is_programmable"]
        )
        self.assertAlmostEqual(
            lane["curve_condition"]["sinusoidal_equivalent_peak_A"],
            2.5 * 2.0 ** 0.5,
        )
        self.assertAlmostEqual(
            pr35["linear_current_scaled_torque_nm"],
            0.7276128778409573,
        )
        self.assertAlmostEqual(
            pr35["margin_at_200rad_s2_multiple"],
            1.9975199406360504,
        )
        self.assertFalse(pr35["gate_at_200rad_s2_ge_2x"])
        self.assertFalse(pr35["linear_scaling_is_manufacturer_verified"])
        self.assertAlmostEqual(
            pr35["maximum_alpha_for_exact_2x_rad_s2"],
            195.3090887095785,
        )
        self.assertGreater(pr35["margin_at_190rad_s2_multiple"], 2.0)
        self.assertFalse(
            pr35["acceleration_limit_bound_in_upstream_configuration"]
        )
        self.assertGreater(pr36["percent_above_curve_RMS_current"], 0.0)
        self.assertFalse(pr36["manufacturer_authorized_for_CS_M21708"])
        self.assertFalse(lane["eligible_as_exact_production_configuration"])

    def test_p28_fails_torque_and_fixed_ratio_contract(self) -> None:
        lane = self.report["alternatives"]["NBK_P28_reduction"]
        self.assertAlmostEqual(
            lane["ratio_motor_speed_over_flyer_speed"], 30.0 / 28.0
        )
        self.assertAlmostEqual(
            lane["belt_geometry"]["center_distance_mm"],
            61.49258536092255,
        )
        self.assertAlmostEqual(
            lane["belt_geometry"]["small_pulley_engaged_teeth"],
            13.861587840364061,
        )
        self.assertAlmostEqual(
            lane["available_to_required_multiple"],
            1.9102136554890239,
        )
        self.assertFalse(lane["torque_screen_ge_2x"])
        self.assertFalse(lane["controller_contract_gate"])
        self.assertFalse(
            lane["upstream_absolute_radians_and_readback_preserved"]
        )
        self.assertFalse(lane["eligible_for_normal_GOAL_selection"])

    def test_p26_math_pass_does_not_override_fixed_ratio_contract(self) -> None:
        lane = self.report["alternatives"]["NBK_P26_reduction"]
        self.assertAlmostEqual(
            lane["ratio_motor_speed_over_flyer_speed"], 30.0 / 26.0
        )
        self.assertAlmostEqual(
            lane["belt_geometry"]["center_distance_mm"],
            62.97103777593675,
        )
        self.assertAlmostEqual(
            lane["belt_geometry"]["small_pulley_engaged_teeth"],
            12.748955560520107,
        )
        self.assertAlmostEqual(
            lane["full_output_inertia_kgm2"],
            9.91054846957379e-5,
        )
        self.assertAlmostEqual(
            lane["available_to_required_multiple"],
            2.0210116643300258,
        )
        self.assertAlmostEqual(
            lane["reserve_above_2x_nm"],
            0.007665498429397122,
        )
        self.assertTrue(lane["torque_screen_ge_2x"])
        self.assertFalse(lane["controller_contract_gate"])
        self.assertFalse(lane["electronic_gearing_or_firmware_change_verified"])
        self.assertFalse(lane["eligible_for_normal_GOAL_selection"])

    def test_nema23_torque_pass_is_rejected_by_goal_and_packaging(self) -> None:
        lane = self.report["alternatives"]["Leadshine_CS_M22313_NEMA23"]
        torque = lane["torque_screen"]
        fit = lane["current_machine_placement_screen"]
        self.assertTrue(torque["math_screen_ge_2x"])
        self.assertAlmostEqual(
            torque["available_to_required_multiple"],
            2.9343190993979595,
        )
        self.assertFalse(torque["production_torque_released"])
        self.assertTrue(lane["normal_GOAL_requires_NEMA17"])
        self.assertFalse(lane["candidate_is_NEMA17"])
        self.assertFalse(lane["normal_GOAL_frame_gate"])
        self.assertLess(
            fit["felt_tensioner_clearance_mm"],
            fit["minimum_required_clearance_mm"],
        )
        self.assertFalse(fit["felt_clearance_gate"])
        self.assertGreater(fit["existing_NEMA17_mount_overlap_mm3"], 0.0)
        self.assertFalse(fit["existing_mount_collision_gate"])
        self.assertTrue(lane["mount_interface"]["new_mount_required"])
        self.assertFalse(lane["eligible_for_normal_GOAL_selection"])

    def test_official_nema23_product_files_are_hash_pinned(self) -> None:
        observed = self.report["source_evidence"][
            "pinned_product_artifact_sha256"
        ]
        self.assertEqual(observed, study.EXPECTED_PRODUCT_HASHES)
        self.assertEqual(
            observed["tmp/CS-M22313_3D/CS-M22313_3D.STEP"],
            "01813231d4bd0c1de12f966f8c7352467a757157c81ae43ee2031cb774a4b5e5",
        )
        self.assertEqual(
            observed["tmp/CS-M22313_MS31.pdf"],
            "5670b96517feefcd81a284ef419da998d4c84325055247342661216b6c7f15e7",
        )
        self.assertEqual(
            observed["tmp/Leadshine_CS-M22313_torque_curve.png"],
            "d6dc3b643d0f17546922f7d96ad8a2824fc3cb784902d2deb112e6ecbe711358",
        )
        self.assertEqual(
            len(
                self.report["source_evidence"][
                    "canonical_review_input_sha256"
                ]
            ),
            64,
        )

    def test_write_outputs_is_reproducible_and_explicit(self) -> None:
        report = study.write_outputs()
        loaded = json.loads(study.OUTPUT_JSON.read_text(encoding="utf-8"))
        self.assertEqual(loaded, report)
        markdown = study.OUTPUT_MD.read_text(encoding="utf-8")
        self.assertIn("review only; no eligible exact production configuration", markdown)
        self.assertIn("Reject; absolute-radians contract is fixed 1:1", markdown)
        self.assertIn("GOAL requires NEMA17", markdown)
        self.assertIn("authorizes no CAD/BOM/load change", markdown)


if __name__ == "__main__":
    unittest.main()
