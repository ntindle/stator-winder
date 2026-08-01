"""Regression tests for the standalone moving-half-turn authority audit."""

from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import unittest

import moving_half_turn_conductor_audit as audit
from params import DEFAULT_STATOR
from slot_route import PackingSupportGraph, solve_safe_mouth_crossover


class MovingHalfTurnConductorAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = json.loads(
            audit.OUTPUT_JSON.read_text(encoding="utf-8")
        )
        cls.plan = json.loads(audit.PLAN.read_text(encoding="utf-8"))

    def test_report_is_current_hash_bound_and_fail_closed(self) -> None:
        audit.validate_report_integrity(self.report)
        self.assertEqual(self.report["status"], "FAIL")
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["assembly_integration_authorized"])
        self.assertEqual(
            self.report["coverage"]
            ["physically_authorized_moving_half_turn_interval_count"],
            0,
        )

    def test_raw_clocks_bind_but_raw_m0_never_matches_policy_target(self) -> None:
        raw = self.report["raw_timing_and_target_binding"]
        self.assertEqual(raw["raw_pass_count"], 24)
        self.assertEqual(raw["raw_half_turn_interval_count"], 2400)
        self.assertLessEqual(raw["maximum_locus_time_error_s"], 2.0e-6)
        self.assertLessEqual(raw["maximum_locus_axis_error_rad"], 2.0e-8)
        self.assertLessEqual(
            raw["maximum_M2_logical_phase_error_rad"], 2.0e-8
        )
        self.assertEqual(raw["raw_M0_exact_sequential_target_count"], 0)
        self.assertEqual(raw["raw_M0_target_mismatch_count"], 2400)
        self.assertAlmostEqual(
            raw["maximum_absolute_raw_M0_target_error_mm"],
            5.26469278034466,
            places=9,
        )
        self.assertFalse(
            raw["gates"]
            ["every_raw_M0_pose_matches_sequential_radial_target"]
        )

    def test_named_guide_radii_and_all_cap_seams_pass_static_scope(
        self,
    ) -> None:
        guide = self.report["physical_active_guide_chain"]
        self.assertEqual(guide["status"], "PASS")
        self.assertGreaterEqual(
            guide["minimum_named_wire_center_radius_mm"], 3.0
        )
        self.assertTrue(
            guide["gates"]["all_named_guide_bend_radii_ge_3mm"]
        )
        self.assertTrue(
            guide["gates"]
            ["all_2400_short_leadin_endpoints_join_actual_cap_lane"]
        )
        seam = guide["exact_short_leadin_to_cap_lane_seam"]
        self.assertEqual(seam["connected_locus_count"], 2400)
        self.assertEqual(seam["disconnected_locus_count"], 0)
        self.assertLessEqual(seam["maximum_exact_distance_mm"], 1.0e-6)
        self.assertTrue(
            self.report["release_gates"]
            ["current_physical_active_guide_chain_authorized"]
        )
        self.assertFalse(
            self.report["release_gates"]
            ["guide_to_rounded_loop_continuation_explicit"]
        )
        raw = self.report["raw_timing_and_target_binding"]
        self.assertGreater(
            raw["minimum_unmodeled_cap_endpoint_to_rounded_target_gap_mm"],
            0.0,
        )

    def test_default_guard_stops_at_21_to_22(self) -> None:
        cross = self.report["sequential_crossover_model"]
        self.assertEqual(
            cross["default_guard_pass_count_before_first_failure"], 21
        )
        failure = cross["first_default_guard_failure"]
        self.assertEqual(
            (failure["start_turn_index"], failure["end_turn_index"]),
            (21, 22),
        )
        self.assertIn("no exact-clear endpoint portal", failure["reason"])
        diagnostic = cross["zero_guard_counterexample_diagnostic"]
        self.assertEqual(
            diagnostic["status"],
            "DIAGNOSTIC_PATH_FOUND_NOT_AUTHORITY",
        )
        self.assertAlmostEqual(
            diagnostic["minimum_prior_center_distance_mm"],
            diagnostic["finished_wire_diameter_mm"],
            places=12,
        )

    def test_existing_solver_reproduces_guarded_failure_and_zero_guard_path(
        self,
    ) -> None:
        graph = PackingSupportGraph.from_report(
            self.plan, spec=DEFAULT_STATOR
        )
        with self.assertRaisesRegex(
            RuntimeError, "21->22 has no exact-clear endpoint portal"
        ):
            solve_safe_mouth_crossover(graph, 21)
        diagnostic = solve_safe_mouth_crossover(
            graph, 21, planner_guard_mm=0.0
        )
        self.assertAlmostEqual(
            diagnostic.minimum_prior_center_distance_mm,
            graph.wire_diameter_mm,
            places=12,
        )
        self.assertEqual(diagnostic.planner_guard_mm, 0.0)

    def test_all_rounded_loop_corner_targets_are_sub_r3(self) -> None:
        bend = self.report["rounded_loop_bend_model"]
        self.assertEqual(bend["sub_R3_placement_count"], 50)
        self.assertFalse(bend["all_50_placement_corner_radii_ge_3mm"])
        self.assertLess(
            bend["maximum_corner_wire_center_radius_mm"], 3.0
        )
        self.assertAlmostEqual(
            bend["minimum_corner_wire_center_radius_mm"],
            0.238760000000001,
            places=12,
        )

    def test_report_tampering_is_rejected(self) -> None:
        tampered = deepcopy(self.report)
        tampered["status"] = "PASS"
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            audit.validate_report_integrity(tampered)

    def test_missing_input_fails_before_any_authority(self) -> None:
        report = audit.analyze(capture_path=Path("missing-raw-capture.jsonl"))
        self.assertEqual(report["status"], "FAIL")
        self.assertFalse(report["production_authorized"])
        self.assertEqual(report["missing_inputs"], ["raw_capture"])
        self.assertEqual(
            report["report_sha256"],
            audit._canonical_hash(report, "report_sha256"),
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
