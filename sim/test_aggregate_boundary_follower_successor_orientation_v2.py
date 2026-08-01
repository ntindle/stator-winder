"""Focused tests for the successor guide V2 orientation law."""

from __future__ import annotations

import unittest
from pathlib import Path
import sys


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_successor_orientation_v2 as orientation


class SuccessorOrientationV2Tests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.result = orientation.analyze()

    def test_current_evidence_and_all_case_upstream_binding(self):
        result = self.result
        self.assertEqual(result["case_count"], 4704)
        self.assertEqual(
            result["identity_counts"],
            {"0": 1176, "1": 1176, "2": 1176, "3": 1176},
        )
        upstream = result["upstream_C1_binding"]
        self.assertEqual(upstream["matched_case_count"], 4704)
        self.assertLessEqual(upstream["maximum_tangent_vector_residual"], 1e-15)
        self.assertLessEqual(upstream["maximum_normal_vector_residual"], 1e-14)

    def test_single_Rot_failure_is_reproduced_exactly(self):
        legacy = self.result["legacy_single_Rot"]
        self.assertEqual(legacy["matched_case_count"], 0)
        self.assertAlmostEqual(
            legacy["minimum_error_deg"], 22.80930999066286, places=11
        )
        self.assertAlmostEqual(
            legacy["maximum_error_deg"], 107.24423103927084, places=11
        )
        self.assertAlmostEqual(
            legacy["mean_error_deg"], 48.88086161808727, places=11
        )
        self.assertEqual(
            legacy["matrix_order"], "Ry(-elevation) @ Rz(yaw)"
        )

    def test_split_euler_maps_local_X_to_all_requested_tangents(self):
        split = self.result["split_yaw_pitch"]
        self.assertEqual(split["matched_case_count"], 4704)
        self.assertLessEqual(split["maximum_angle_residual_deg"], 1e-12)
        self.assertLessEqual(split["maximum_vector_residual"], 1e-14)
        self.assertEqual(split["matrix_order"], "Rz(yaw) @ Ry(-elevation)")

    def test_full_plane_frame_maps_tangent_normal_and_roll(self):
        frame = self.result["full_tangent_normal_frame"]
        self.assertEqual(frame["matched_tangent_case_count"], 4704)
        self.assertEqual(frame["matched_normal_case_count"], 4704)
        self.assertLessEqual(frame["maximum_tangent_vector_residual"], 1e-15)
        self.assertLessEqual(frame["maximum_normal_vector_residual"], 1e-14)
        self.assertLessEqual(frame["maximum_orthonormality_residual"], 1e-14)
        self.assertLessEqual(frame["maximum_determinant_residual"], 1e-14)
        self.assertIn("Plane(origin=center", frame["build123d"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
