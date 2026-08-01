"""Focused gates for the bounded moving-half-turn C0 proof."""

from __future__ import annotations

import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

import moving_half_turn_segment_continuity_audit as audit


class MovingHalfTurnSegmentContinuityAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = audit.analyze()

    def test_all_available_adjacent_loci_have_analytic_C0_homotopy(self) -> None:
        report = self.report
        coverage = report["coverage"]
        gates = report["gates"]
        self.assertEqual(report["schema"], audit.SCHEMA)
        self.assertEqual(report["status"], "PASS")
        self.assertEqual(report["structural_status"], "PASS")
        self.assertEqual(
            report["decision"],
            "C0_ADJACENT_LOCUS_HOMOTOPY_PROVEN__"
            "MOVING_PHYSICAL_ROUTE_NOT_PROVEN",
        )
        self.assertEqual(coverage["available_point_state_count"], 2400)
        self.assertEqual(coverage["proved_adjacent_C0_interval_count"], 2376)
        self.assertAlmostEqual(
            coverage["proved_adjacent_C0_duration_s"],
            380.0813848800001,
        )
        self.assertLessEqual(
            coverage["maximum_affine_seam_bound_for_all_u_mm"],
            audit.SEAM_TOL_MM,
        )
        self.assertTrue(
            gates["all_2376_available_adjacent_intervals_have_C0_homotopy"]
        )
        self.assertTrue(
            gates["all_affine_segment_seams_below_tolerance_for_every_u"]
        )
        self.assertTrue(gates["all_endpoint_axes_match_raw_timeline"])
        self.assertEqual(len(report["intervals"]), 2376)
        self.assertTrue(all(
            row["proof"]["C0_segment_chain_homotopy_proven"]
            for row in report["intervals"]
        ))

    def test_proof_is_explicitly_not_moving_physical_authority(self) -> None:
        report = self.report
        gates = report["gates"]
        self.assertEqual(report["physical_authority_status"], "NOT_PROVEN")
        self.assertFalse(report["production_authorized"])
        self.assertFalse(report["controller_modified"])
        self.assertFalse(report["CAD_modified"])
        self.assertEqual(
            report["physical_authority_boundary"][
                "moving_physical_interval_count"
            ],
            0,
        )
        for key in (
            "named_guide_surface_adherence_through_motion_proven",
            "moving_rigid_and_prior_copper_clearance_proven",
            "moving_bend_radius_proven",
            "moving_contact_tail_ownership_proven",
            "physical_quasistatic_moving_interval_authorized",
        ):
            self.assertFalse(gates[key], key)
        self.assertTrue(all(
            row["physical_quasistatic_interval_authorized"] is False
            for row in report["intervals"]
        ))

    def test_24_closing_intervals_and_contact_observations_remain_missing(
        self,
    ) -> None:
        coverage = self.report["coverage"]
        capture = self.report["capture_schema_evidence"]
        self.assertEqual(coverage["required_half_turn_intervals"], 2400)
        self.assertEqual(coverage["unpaired_final_half_turn_interval_count"], 24)
        self.assertFalse(
            self.report["gates"]
            ["all_2400_half_turn_intervals_have_paired_route_endpoints"]
        )
        self.assertEqual(capture["capture_schema"], 4)
        self.assertEqual(capture["controller_mode"], "upstream")
        self.assertIsNone(capture["controller_adapter_sha256"])
        self.assertEqual(capture["wire_contact_observation_count"], 0)
        self.assertEqual(capture["intermediate_wire_route_event_count"], 0)
        issue_codes = {row["code"] for row in self.report["issues"]}
        self.assertIn("closing_half_turn_endpoint_loci_missing", issue_codes)
        self.assertIn("capture_has_no_wire_contact_observations", issue_codes)

    def test_existing_sequential_policy_binds_timing_but_not_physical_route(
        self,
    ) -> None:
        binding = self.report["declared_slot_route_policy_binding"]
        timing = binding["raw_timing_binding"]
        crossover = binding["mouth_crossover_table"]
        probe = crossover["blocking_required_member_probe"]
        crossing = binding["existing_crossing_certificate"]
        endpoints = binding["active_guide_endpoint_binding"]

        self.assertTrue(timing["all_stored_starts_match_k_pi"])
        self.assertLess(timing["maximum_directed_phase_error_rad"], 2.0e-10)
        self.assertFalse(crossover["complete_table_constructible"])
        self.assertFalse(crossover["sequential_lay_samples_full_table_callable"])
        self.assertEqual(probe["status"], "FAIL")
        self.assertEqual(probe["start_turn_index"], 21)
        self.assertEqual(probe["end_turn_index"], 22)
        self.assertEqual(probe["commanded_half_turn_interval_index"], 43)
        self.assertEqual(
            probe["reason"],
            "crossover 21->22 has no exact-clear endpoint portal",
        )

        self.assertEqual(crossing["status"], "FAIL")
        self.assertEqual(crossing["passed_geometry_cases"], 100)
        self.assertEqual(crossing["expected_geometry_cases"], 100)
        self.assertEqual(crossing["covered_direction_cases"], 0)
        self.assertEqual(crossing["expected_direction_cases"], 200)
        self.assertTrue(crossing["progressive_support_validated"])
        self.assertEqual(crossing["failing_crossings"], [])

        self.assertEqual(endpoints["comparison_count"], 2400)
        self.assertGreater(
            endpoints["minimum_direct_endpoint_difference_mm"], 6.0
        )
        self.assertGreater(
            endpoints["maximum_direct_endpoint_difference_mm"], 8.0
        )
        self.assertFalse(endpoints["endpoints_identical"])
        self.assertFalse(
            endpoints["hash_bound_moving_continuation_between_endpoints_exists"]
        )
        self.assertFalse(
            binding["bindable_to_current_raw_timing_and_active_guide"]
        )

    def test_every_paired_interval_changes_named_terminal_lane(self) -> None:
        coverage = self.report["coverage"]
        self.assertEqual(coverage["terminal_lane_change_interval_count"], 2376)
        self.assertTrue(coverage["all_paired_intervals_change_terminal_lane"])
        self.assertTrue(all(
            row["start_terminal_lane_id"] != row["end_terminal_lane_id"]
            for row in self.report["intervals"]
        ))

    def test_arclength_reparameterization_preserves_endpoint_polyline(self) -> None:
        points = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 2.0, 0.0],
        ])
        sampled = audit._resample_polyline(points, 9)
        np.testing.assert_array_equal(sampled[0], points[0])
        np.testing.assert_array_equal(sampled[-1], points[-1])
        self.assertTrue(np.allclose(sampled[:, 2], 0.0))
        self.assertTrue(all(
            math.isclose(float(point[0]), 1.0, abs_tol=1e-12)
            or math.isclose(float(point[1]), 0.0, abs_tol=1e-12)
            for point in sampled
        ))

    def test_report_and_markdown_writers_preserve_fail_closed_scope(self) -> None:
        self.assertEqual(
            self.report["report_sha256"],
            audit._canonical_hash(self.report, "report_sha256"),
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            json_path = root / "proof.json"
            md_path = root / "proof.md"
            audit.write_report(self.report, json_path)
            audit.write_markdown(self.report, md_path)
            loaded = json.loads(json_path.read_text(encoding="utf-8"))
            markdown = md_path.read_text(encoding="utf-8")
        self.assertEqual(loaded["report_sha256"], self.report["report_sha256"])
        self.assertIn("Physical moving-route authority: **NOT PROVEN**", markdown)
        self.assertIn("Physically authorized moving intervals | 0", markdown)
        self.assertIn("crossover 21->22 has no exact-clear endpoint portal", markdown)


if __name__ == "__main__":
    unittest.main(verbosity=2)
