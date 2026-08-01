from __future__ import annotations

from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
for path in (HERE, CAD):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import permanent_cap_offset_spoke_balance_retention as study


class OffsetSpokeBalanceRetentionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = study.analyze()

    def test_stable_review_counterweights_are_explicitly_unattached(self) -> None:
        truth = self.report["current_review_truth"]
        self.assertFalse(truth["tungsten_slugs_attached"])
        self.assertIn("no screw", truth["finding"])
        self.assertTrue(self.report["gates"][
            "current_review_slugs_correctly_classified_unretained"])

    def test_m3x8_proposal_is_rejected_by_exact_geometry(self) -> None:
        retention = self.report["retention"]
        collision = self.report["collision"]
        self.assertEqual(retention["status"], "REJECTED_PROPOSED_STACK")
        self.assertFalse(
            retention["all_four_retainers_stay_ahead_of_their_floors"])
        self.assertAlmostEqual(
            retention["maximum_retainer_projection_behind_floor_mm"],
            2.74092326360476,
        )
        self.assertAlmostEqual(retention["front_boss_exterior_wall_mm"], 0.65)
        self.assertAlmostEqual(retention["front_pair_center_septum_mm"], 1.0)
        exact = collision["exact_controlling_pose_mm"]
        self.assertAlmostEqual(
            exact["correction_packages_to_block_mm"],
            0.7069465194080351,
        )
        certificate = collision["continuous_360_certificate"]
        self.assertFalse(certificate["certificate_valid"])
        self.assertAlmostEqual(
            certificate["minimum_certified_clearance_mm"],
            exact["correction_packages_to_block_mm"],
        )

    def test_old_balance_solution_is_only_a_rejected_math_witness(self) -> None:
        balance = self.report["two_plane_balance"]
        self.assertEqual(
            balance["status"],
            "REJECTED_NOMINAL_SOLUTION__RECOMPUTE_REQUIRED",
        )
        self.assertEqual(set(balance["rejected_slug_length_solution_mm"]), {
            "rear_left", "rear_right", "front_left", "front_right",
        })
        self.assertLessEqual(balance["nominal_residual_static_g_mm"], 1.0e-5)
        self.assertLessEqual(balance["nominal_residual_couple_g_mm2"], 1.0e-4)
        self.assertEqual(
            balance["centered_package_targets_for_corrected_recompute_g"],
            {
                "rear_left": 5.776218,
                "rear_right": 4.929977,
                "front_left": 2.018484,
                "front_right": 2.516098,
            },
        )
        self.assertTrue(balance["physical_balance_requirement"]
                        ["maximum_G2p5_residual_g_mm"] > 0.0)

    def test_corrected_m3x6_contract_stays_fail_closed_until_regenerated(self) -> None:
        corrected = self.report["corrected_successor_retention_contract"]
        self.assertEqual(corrected["pocket_body_axial_depth_mm"], 8.0)
        self.assertIn("M3x6", corrected["screw"]["selection"])
        self.assertEqual(corrected["screw"]["tip_from_pocket_rear_mm"], 6.0)
        self.assertEqual(corrected["insert"]["full_engagement_mm"], 4.3)
        self.assertEqual(
            corrected["printed_retainer_boss"]["rear_projection_allowed_mm"],
            0.0,
        )
        self.assertEqual(
            corrected["front_pair"]["turned_slug_diameter_mm"], 11.0,
        )
        self.assertGreaterEqual(corrected["front_pair"]["center_septum_mm"],
                                2.4)
        self.assertAlmostEqual(
            corrected["analytic_successor_clearance_witness_mm"]["minimum"],
            2.88,
        )
        self.assertTrue(corrected["successor_balance_recompute_required"])
        self.assertFalse(corrected["focused_CAD_authorized"])

    def test_real_force_vector_od65_duty_fails_current_motor_and_pulley(self) -> None:
        duty = self.report["tolerance_and_M2_duty"]
        force = self.report["wire_force_vector_torque"]
        launch = force["GOAL_launch_OD65"]
        self.assertAlmostEqual(
            launch["force_vector"]["effective_line_of_action_distance_mm"],
            32.5,
        )
        self.assertAlmostEqual(
            duty["required_torque_at_300rpm_10N_nm"], 0.36253132,
            places=8,
        )
        self.assertLess(duty["selected_motor_margin"], 2.0)
        self.assertLess(duty["selected_pulley_margin"], 2.0)
        self.assertTrue(duty["motor_rotor_inertia_missing"])
        self.assertAlmostEqual(
            force["current_1_to_1_line_of_action_limits"]
            ["motor_controlling_maximum_mm_for_2x_known_load"],
            27.746868,
            places=6,
        )
        self.assertFalse(force["drive_recommendation"]
                         ["48T_flyer_40T_motor_selected"])

    def test_material_missing_terms_and_physical_balance_are_blockers(self) -> None:
        gates = self.report["gates"]
        for name in (
            "proposed_retainer_exact_package_to_block_clearance_ge_2p2mm",
            "all_proposed_retainers_stay_ahead_of_pocket_floors",
            "front_boss_minimum_exterior_wall_ge_2p4mm",
            "front_pocket_center_septum_ge_2p4mm",
            "corrected_M3x6_successor_geometry_modeled_in_CAD",
            "corrected_successor_hardware_spacers_and_adhesive_mass_bound",
            "corrected_successor_two_plane_balance_recomputed",
            "actual_transition_guide_and_adhesive_mass_bound",
            "bearing_inner_elements_and_selected_motor_rotor_inertia_bound",
            "motor_pulley_set_screws_modeled",
            "physical_two_plane_balance_to_G2p5_or_better_complete",
            "GOAL_OD65_force_vector_motor_margin_ge_2",
            "GOAL_OD65_force_vector_pulley_margin_ge_2",
            "installed_M2_friction_measured_within_allowance",
            "NEMA23_candidate_dynamic_curve_at_300rpm_verified",
            "NEMA23_candidate_installed_geometry_and_inertia_verified",
        ):
            self.assertFalse(gates[name])
            self.assertIn(name, self.report["controlling_blockers"])

    def test_corrected_hardware_and_source_hashes_are_bound(self) -> None:
        sourcing = self.report["sourcing"]
        self.assertIn("5995N71", sourcing["dense_stock"]["selection"])
        self.assertEqual(
            sourcing["rejected_retention_screw"]["step_parts_id"],
            "countersunk_socket_screw_m3_l0008_simple",
        )
        self.assertIn(
            "92125A126",
            sourcing["corrected_successor_retention_screw"]["selection"],
        )
        self.assertIn("94459A130", sourcing["insert"]["selection"])
        contracts = self.report["source_contracts"]
        for key in (
            "frozen_review_source_sha256",
            "frozen_review_STEP_sha256",
            "frozen_review_report_sha256",
            "loads_source_sha256",
            "loads_report_sha256",
            "hardware_source_sha256",
            "hardware_audit_sha256",
            "wire_force_source_sha256",
            "wire_force_report_sha256",
        ):
            self.assertEqual(len(contracts[key]), 64)

    def test_no_go_boundary_and_report_hash_round_trip(self) -> None:
        self.assertEqual(self.report["status"], "DESIGN_NO_GO")
        self.assertFalse(self.report["focused_counterweight_CAD_authorized"])
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["assembly_integration_authorized"])
        self.assertGreater(len(self.report["controlling_blockers"]), 0)
        study.validate_report_integrity(self.report)
        written = study.write_reports(self.report)
        study.validate_report_integrity(written)
        self.assertEqual(written["report_sha256"], self.report["report_sha256"])
        self.assertTrue(study.JSON_OUT.exists())
        self.assertTrue(study.MD_OUT.exists())


if __name__ == "__main__":
    unittest.main()
