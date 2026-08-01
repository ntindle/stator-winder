"""Regression tests for raw-upstream fixed-delay motor timing evidence."""

import math
import unittest

import loads


class MotionTimeTest(unittest.TestCase):
    def test_triangular_profile(self):
        # Distance is below v^2/a, so the velocity limit is never reached.
        self.assertAlmostEqual(
            loads.minimum_trapezoid_time(2.0, 20.0, 50.0),
            2.0 * math.sqrt(2.0 / 50.0),
            places=12,
        )

    def test_trapezoidal_profile(self):
        self.assertAlmostEqual(
            loads.minimum_trapezoid_time(17.540558983, 20.0, 50.0),
            1.27702794915,
            places=9,
        )

    def test_invalid_motion_limits_fail(self):
        with self.assertRaises(ValueError):
            loads.minimum_trapezoid_time(1.0, 0.0, 50.0)
        with self.assertRaises(ValueError):
            loads.minimum_trapezoid_time(1.0, 20.0, 0.0)


class RawUpstreamTimingTest(unittest.TestCase):
    def test_current_raw_capture_arrives_before_fixed_sleeps(self):
        report = loads.raw_upstream_timing_evidence()
        self.assertEqual(report["status"], "PASS")
        self.assertEqual(report["capture_schema"], 4)
        self.assertEqual(len(report["shaft_wrap_moves"]), 2)
        self.assertTrue(all(report["checks"].values()))
        self.assertTrue(all(
            row["time_margin_s"] > 0.0
            and row["arrived_in_raw_timeline"]
            for row in report["shaft_wrap_moves"]
        ))


if __name__ == "__main__":
    unittest.main()
