from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
for path in (HERE, CAD):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import permanent_cap_offset_spoke_wire_force_torque as study


class OffsetSpokeWireForceTorqueTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = study.analyze()

    def test_complete_raw_coverage_is_hash_bound(self) -> None:
        raw = self.report["canonical_raw_diagnostic"]
        self.assertEqual(raw["pass_count"], 24)
        self.assertEqual(raw["half_turn_locus_count"], 2400)
        self.assertEqual(raw["motion_sign_locus_counts"], {
            "-1": 1200, "1": 1200,
        })
        self.assertEqual(raw["unique_exact_path_templates"], 720)
        self.assertEqual(raw["all_interval_angle_evaluations"], 432000)
        self.assertTrue(self.report["gates"][
            "canonical_raw_capture_sha256_exact"])

    def test_whole_flyer_external_moment_uses_exact_exit_tangent(self) -> None:
        witness = self.report["canonical_raw_diagnostic"][
            "continuous_one_degree_worst"]["path_force_witness"]
        exit_point = witness["toroid_exit_mm"]
        tangent = witness["outgoing_unit_tangent"]
        cross = abs(exit_point[0] * tangent[1]
                    - exit_point[1] * tangent[0])
        self.assertAlmostEqual(
            cross, witness["effective_line_of_action_distance_mm"], places=10,
        )
        self.assertAlmostEqual(
            sum(value * value for value in tangent), 1.0, places=10,
        )
        self.assertAlmostEqual(witness["incoming_boundary_radius_mm"], 0.0)
        self.assertAlmostEqual(
            witness["wire_torque_at_10N_nm"], 10.0 * cross / 1000.0,
        )

    def test_centered_target_bounds_have_exact_planar_tangent_witnesses(self) -> None:
        cases = self.report["duty_cases"]
        for name, radius in (
            ("canonical_default_OD46", 22.88824),
            ("GOAL_launch_OD65", 32.5),
            ("parametric_OD90_advisory", 45.0),
        ):
            witness = cases[name]["force_vector"]
            self.assertAlmostEqual(
                witness["target_radius_bound_mm"], radius, places=8,
            )
            self.assertAlmostEqual(
                witness["effective_line_of_action_distance_mm"], radius,
                places=8,
            )
            self.assertLessEqual(
                abs(witness["tangent_perpendicularity_residual_mm"]), 1.0e-9,
            )

    def test_default_numeric_margin_passes_but_goal_launch_fails(self) -> None:
        cases = self.report["duty_cases"]
        default = cases["canonical_default_OD46"]
        launch = cases["GOAL_launch_OD65"]
        advisory = cases["parametric_OD90_advisory"]
        direct = cases["unconstrained_direct_R64"]
        self.assertTrue(default["raw_capture_supports_this_stator"])
        self.assertTrue(default["known_load_motor_margin_ge_2"])
        self.assertTrue(default["known_load_pulley_margin_ge_2"])
        self.assertFalse(launch["raw_capture_supports_this_stator"])
        self.assertFalse(launch["known_load_motor_margin_ge_2"])
        self.assertFalse(launch["known_load_pulley_margin_ge_2"])
        self.assertLess(advisory["known_load_motor_margin"], 2.0)
        self.assertLess(direct["known_load_motor_margin"], 1.0)
        self.assertAlmostEqual(
            launch["known_load_required_torque_nm"], 0.36253132, places=8,
        )

    def test_current_drive_geometry_limits_are_explicit(self) -> None:
        limits = self.report[
            "line_of_action_limits_for_current_1_to_1_drive"]
        self.assertAlmostEqual(
            limits["motor_controlling_maximum_mm_for_2x_known_load"],
            27.746868, places=6,
        )
        self.assertAlmostEqual(
            limits["pulley_maximum_mm_for_2x_known_load"],
            30.646868, places=6,
        )
        self.assertLess(
            limits["motor_controlling_maximum_mm_for_2x_known_load"], 32.5,
        )

    def test_ratio_change_is_rejected_by_unmodified_upstream_contract(self) -> None:
        drive = self.report["drive_recommendation"]
        self.assertFalse(drive["48T_flyer_40T_motor_selected"])
        self.assertIn("50/50", drive["48T_40T_reason_rejected"])
        self.assertTrue(self.report["gates"][
            "frozen_upstream_exact_1_to_1_ratio_preserved"])
        candidate = drive["unapproved_exact_1_to_1_NEMA23_candidate"]
        self.assertFalse(candidate["selected"])
        self.assertFalse(candidate["motor_curve_gate"])
        self.assertEqual(candidate["exact_STEP_sha256"],
                         "08218506695fb01b1e37b551084824e9fadc9c05a86557c7594a8a49e75ec0d6")
        self.assertEqual(candidate["transmission"]["ratio"], 1.0)
        self.assertEqual(candidate["transmission"]["belt"], "210-3GT-6")
        self.assertAlmostEqual(
            candidate["transmission"]["official_P30_base_T_Tr_at_300rpm_nm"],
            2.06,
        )
        self.assertAlmostEqual(
            candidate["transmission"]["allowable_torque_at_300rpm_nm"],
            1.854,
        )
        self.assertTrue(candidate["transmission"]
                        ["pulley_capacity_ge_required_2x"])
        self.assertFalse(self.report["gates"]
                         ["NEMA23_candidate_dynamic_curve_at_300rpm_verified"])

    def test_missing_physical_terms_and_od65_fail_closed(self) -> None:
        gates = self.report["gates"]
        for name in (
            "GOAL_OD65_known_load_motor_margin_ge_2",
            "GOAL_OD65_known_load_pulley_margin_ge_2",
            "actual_successor_outgoing_path_and_transition_guide_defined",
            "transition_guide_adhesive_and_motor_pulley_set_screw_mass_bound",
            "selected_motor_rotor_inertia_bound",
            "installed_M2_friction_measured_le_0p020Nm",
        ):
            self.assertFalse(gates[name])
            self.assertIn(name, self.report["controlling_blockers"])
        self.assertEqual(self.report["status"], "FAIL_CLOSED")
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["assembly_integration_authorized"])

    def test_report_hash_round_trip(self) -> None:
        study.validate_report_integrity(self.report)
        written = study.write_reports(self.report)
        study.validate_report_integrity(written)
        self.assertEqual(written["report_sha256"], self.report["report_sha256"])
        self.assertTrue(study.JSON_OUT.exists())
        self.assertTrue(study.MD_OUT.exists())


if __name__ == "__main__":
    unittest.main()
