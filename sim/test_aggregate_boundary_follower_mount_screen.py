"""Focused tests for the follower eccentric mount screen."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_mount_screen as screen


class AggregateBoundaryFollowerMountScreenTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.report = screen.analyze()

    def test_exact_mount_lever_and_pattern(self):
        mount = self.report["mount_geometry"]
        self.assertEqual(mount["tower_interface_centroid_local_mm"],
                         [32.0, 0.0, -114.0])
        self.assertEqual(mount["nose_axis_center_local_mm"],
                         [33.0, 0.0, 24.0])
        self.assertEqual(mount["lever_vector_local_mm"], [1.0, 0.0, 138.0])
        self.assertEqual(mount["M4_x_span_mm"], 6.0)
        self.assertEqual(mount["M4_y_span_mm"], 42.0)
        self.assertEqual(mount["key_x_span_mm"], 0.0)
        self.assertEqual(mount["key_y_span_mm"], 20.0)

    def test_40N_moment_invalidates_equal_share_only_screen(self):
        radial = self.report["load_cases"]["radial_X_40N"]
        tangential = self.report["load_cases"]["tangential_Y_40N"]
        self.assertEqual(radial["moment_about_Y_Nmm"], 5520.0)
        self.assertEqual(radial["ideal_M4_row_couple_N"], 920.0)
        self.assertEqual(
            radial["ideal_differential_reaction_per_screw_N"], 460.0)
        self.assertAlmostEqual(
            tangential["ideal_M4_row_couple_N"], 131.42857142857142)
        self.assertAlmostEqual(
            tangential["ideal_differential_reaction_per_screw_N"],
            65.71428571428571,
        )
        self.assertFalse(self.report["release_gates"][
            "equal_10N_per_M4_is_sufficient_mount_proof"])

    def test_mass_is_positive_but_explicitly_incomplete(self):
        mass = self.report["custom_body_mass"]
        self.assertGreater(mass["total_g"], 25.0)
        self.assertLess(mass["total_g"], 35.0)
        self.assertTrue(mass[
            "hardware_springs_and_unattached_M0_gate_excluded"])
        self.assertFalse(self.report["release_gates"][
            "hardware_spring_and_linkage_mass_included"])

    def test_fail_closed_integrity_and_written_output(self):
        report = deepcopy(self.report)
        screen.validate_report_integrity(report)
        report["status"] = "PASS"
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            screen.validate_report_integrity(report)
        generated = screen.write_outputs(self.report)
        written = json.loads(screen.OUTPUT_JSON.read_text(encoding="utf-8"))
        self.assertEqual(generated["report_sha256"], written["report_sha256"])
        screen.validate_report_integrity(written)


if __name__ == "__main__":
    unittest.main(verbosity=2)
