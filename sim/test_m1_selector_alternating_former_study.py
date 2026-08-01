"""Fail-closed report tests for the M1-selected former successor."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import m1_selector_alternating_former_study as study


class M1SelectorAlternatingFormerStudyTests(unittest.TestCase):

    def test_selector_and_signed_cam_cover_every_raw_required_pulse(self):
        result = study.selector_and_cam_contract()
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["raw_winding_pass_count"], 24)
        self.assertEqual(result["raw_turns_per_pass"], 50)
        self.assertEqual(result["both_M2_directions"], [-1, 1])
        self.assertEqual(result["unique_cam_law_count"], 3)
        self.assertEqual(result["evaluated_required_pulses"], 96)
        self.assertTrue(result["all_24_indices_select_exact_capture_law"])
        self.assertTrue(result["all_required_phase_pulses_select_exact_finger"])

    def test_m0_gate_retracts_before_every_index_and_shaft_wrap(self):
        result = study.m0_fail_safe_contract()
        self.assertEqual(result["status"], "PASS")
        self.assertTrue(result["all_raw_M1_moves_all_retracted"])
        self.assertTrue(result["all_raw_shaft_wraps_all_retracted"])
        self.assertTrue(result["all_raw_winding_range_commands_engaged"])
        self.assertGreater(
            result["minimum_forced_retracted_margin_at_M1_move_mm"], 5.99)
        self.assertGreater(
            result["tongue_receiver_gap_at_nearest_M1_move_mm"], 2.99)

    def test_analytical_load_and_interlock_margin_remain_above_two(self):
        result = study.loads_balance_and_interlocks()
        loads = study._load(study.LOADS)
        current_screen = loads["m2"]["current_geometry_supporting_screen"]
        self.assertEqual(result["status"], "PASS_ANALYTICAL")
        self.assertGreaterEqual(result["revised_M2_margin"], 2.0)
        self.assertEqual(
            result["baseline_M2_required_torque_Nm"],
            current_screen["energy_model_required_nm"],
        )
        self.assertEqual(
            result["available_M2_transmission_capacity_Nm"],
            loads["m2"]["pulley"]["allowable_torque_nm"],
        )
        inferred_accel = (
            result["added_M2_acceleration_torque_Nm"]
            / result["cam_rotor_polar_inertia_kg_m2"]
        )
        self.assertAlmostEqual(
            inferred_accel,
            current_screen["t_accel_nm"] / loads["flyer"]["izz_kgm2"],
        )
        self.assertEqual(
            result["balance"]["status"], "PASS_ANALYTICAL_SYMMETRY")
        self.assertTrue(
            result["hardwired_interlock"]["shaft_wrap_M2_motion_allowed"])
        self.assertFalse(
            result["hardwired_interlock"]["raw_protocol_changed"])

    def test_current_turn45_diagnostics_keep_R3_gates_fail_closed(self):
        report = study.build_report()
        route = report["exact_progressive_wire_route"]
        self.assertEqual(report["status"], "DESIGN_NO_GO")
        self.assertFalse(report["release_authorized"])
        self.assertFalse(report["assembly_integration_authorized"])
        self.assertEqual(route["stored_route_status"], "FAIL")
        self.assertEqual(route["stored_route_geometry_status"], "PASS")
        self.assertEqual(route["inherited_existing_pass_count"], 100)
        self.assertEqual(route["existing_crossing_route_count"], 100)
        self.assertEqual(
            set(route["stored_route_release_blockers"]),
            set(study.ROUTE_RELEASE_BLOCKERS),
        )
        self.assertEqual(
            route["selected_turn45_diagnostic_cases"], [[45, 0], [45, 1]])
        self.assertGreater(route["turn45_parent_prefix_margin_mm"], 0.0)
        self.assertEqual(route["raw_half_turn_states_bound"], 2400)
        self.assertEqual(route["evaluated_tail_candidates"], 6240)
        self.assertEqual(route["curvature_R3_tail_candidates"], 244)
        self.assertEqual(route["copper_clear_R3_tail_candidates"], 0)
        self.assertEqual(route["elastic_contact_status"], "FAIL")
        self.assertEqual(route["elastic_curvature_pass_count"], 0)
        self.assertIn(
            "contact_routes_meet_3mm_bend_contract",
            route["elastic_release_blockers"],
        )
        self.assertTrue(all(row["status"] == "FAIL"
                            for row in route["rows"]))
        self.assertTrue(all(
            "support_normal_approach_point_local_mm" in row
            for row in route["rows"]))
        self.assertTrue(all(
            row["best_joint_centerline_clearance_mm"]
            < row["required_centerline_clearance_mm"]
            for row in route["rows"]))
        self.assertTrue(
            report["gates"]["stored_route_rigid_geometry_100_of_100"])
        self.assertFalse(report["gates"]["stored_route_release_proof"])
        self.assertFalse(
            report["gates"]["both_turn45_one_stage_R3_tail_routes"])
        self.assertFalse(report["gates"]["elastic_contact_routes_meet_R3"])
        self.assertIn("not based on two rigid failures", report["decision"])
        markdown = study._markdown(report)
        self.assertIn("Existing rigid route geometries: 100 / 100 pass", markdown)
        self.assertIn("diagnostic targets, not failed rigid rows", markdown)
        self.assertNotIn("unsupported shortfall", markdown)

    def test_single_valued_nonlinear_m0_limit_is_narrowly_scoped(self):
        result = study.single_valued_nonlinear_m0_limit()
        self.assertEqual(result["status"], "NO_GO_SINGLE_VALUED_ONLY")
        self.assertEqual(result["raw_states_matching_certified_schedule"], 6)
        self.assertEqual(result["raw_state_count"], 2400)
        self.assertFalse(
            result["single_valued_monotone_mapping_can_reproduce_plan"])
        self.assertIn("two-track", result["not_evaluated_here"])


if __name__ == "__main__":
    unittest.main()
