"""Regression tests for the fail-closed two-lane raster study."""

import math
import unittest

import two_lane_raster_study as study


class TwoLaneRasterStudyTests(unittest.TestCase):
    def test_constant_tangential_far_lane_cannot_hold_25(self):
        result = study.straight_lane_capacity(0.22352, 0.127)
        self.assertEqual(result["status"], "FAIL")
        self.assertGreaterEqual(result["near_lane_maximum_center_count"], 25)
        self.assertEqual(result["far_lane_maximum_center_count"], 18)

    def test_variable_partition_is_only_static_successor_geometry(self):
        report = study.analyze()
        partition = report["variable_tangential_partition"]
        self.assertEqual(partition["status"], "PASS")
        self.assertTrue(partition["checks"]["25_centers_per_branch"])
        self.assertTrue(partition["checks"]["both_branch_centerlines_simple"])
        self.assertTrue(partition["checks"]["branch_centerlines_disjoint"])
        self.assertTrue(
            partition["checks"]["full_neighbor_history_mouth_connected"])
        self.assertTrue(math.isclose(
            partition["reversal_connector_mm"], 0.22352, abs_tol=1e-9))

    def test_raw_capture_has_one_exact_direction_m0_alias_per_pass(self):
        report = study.analyze()
        raw = report["raw_single_value_mapping"]
        self.assertEqual(raw["status"], "FAIL")
        self.assertEqual(raw["conflict_count"], 24)
        self.assertEqual(raw["affected_pass_count"], 24)
        self.assertEqual(
            {tuple(row["turn_indices"]) for row in raw["witnesses"]},
            {(0, 1)},
        )
        self.assertEqual(
            {row["m0_position_rad"] for row in raw["witnesses"]},
            {-61.918},
        )
        self.assertEqual(
            {row["motion_sign"] for row in raw["witnesses"]}, {-1, 1})

    def test_r3_and_error_budget_fail_closed(self):
        report = study.analyze()
        route = report["R3_sequential_route"]
        self.assertEqual(report["status"], "FAIL")
        self.assertEqual(
            report["decision"], "REJECT_DIRECTION_ONLY_TWO_LANE_RASTER")
        self.assertTrue(route["stored_100_case_geometry_coverage"])
        self.assertEqual(route["minimum_proved_contact_bend_radius_mm"], 0.22352)
        self.assertLess(route["minimum_proved_contact_bend_radius_mm"], 3.0)
        self.assertFalse(
            route["all_neighbor_histories_and_both_signs_release_proved"])
        self.assertEqual(report["tolerance_error_budget"][
            "allowable_unmodeled_independent_cam_error_mm"], 0.0)

    def test_report_hash_and_capture_binding(self):
        report = study.analyze()
        payload = dict(report)
        expected = payload.pop("report_sha256")
        self.assertEqual(study._canonical_hash(payload), expected)
        self.assertEqual(
            report["capture"]["sha256"], study.EXPECTED_CAPTURE_SHA256)


if __name__ == "__main__":
    unittest.main()
