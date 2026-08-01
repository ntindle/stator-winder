"""Regression gates for the filtered Leadshine CS-M21708 vendor CAD."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import leadshine_cs_m21708_cableless as motor  # noqa: E402


class LeadshineCablelessCadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = motor.audit()

    def test_vendor_source_and_solid_partition_are_hash_bound(self) -> None:
        self.assertEqual(
            self.report["source"]["step_sha256"], motor.SOURCE_SHA256
        )
        self.assertEqual(self.report["source"]["solid_count"], 27)
        self.assertEqual(self.report["filter"]["retained_solid_count"], 18)
        self.assertEqual(
            self.report["filter"]["dropped_cable_connector_solid_indices_1_based"],
            [4, 5, 9, 10, 11, 18, 19, 20, 21],
        )
        self.assertFalse(self.report["filter"]["retained_geometry_remodeled"])

    def test_mount_frame_is_exactly_rebased(self) -> None:
        frame = self.report["mount_frame"]
        self.assertEqual(frame["mount_face_z_mm"], 0.0)
        self.assertEqual(frame["shaft_direction"], "+Z")
        self.assertAlmostEqual(frame["shaft_tip_z_mm"], 24.0, places=6)
        self.assertAlmostEqual(frame["nominal_body_rear_z_mm"], -83.0, places=6)
        self.assertAlmostEqual(frame["exact_feature_rear_z_mm"], -83.2, places=6)
        self.assertAlmostEqual(frame["bounds"]["size_mm"][0], 42.3, places=3)
        self.assertAlmostEqual(frame["bounds"]["size_mm"][1], 42.3, places=3)

    def test_exposed_shaft_is_d_profile_not_round(self) -> None:
        shaft = self.report["shaft_interface"]
        self.assertEqual(shaft["profile"], "D")
        self.assertAlmostEqual(shaft["diameter_mm"], 5.0)
        self.assertAlmostEqual(shaft["across_flat_mm"], 4.5)
        self.assertAlmostEqual(
            shaft["internal_round_section_area_mm2"], math.pi * 2.5**2, places=3
        )
        self.assertAlmostEqual(
            shaft["exposed_d_section_area_mm2"], 18.6131, places=3
        )
        self.assertFalse(shaft["stock_round_bore_split_clamp_authorized"])

    def test_all_derivative_geometry_gates_pass(self) -> None:
        self.assertTrue(all(self.report["gates"].values()))


if __name__ == "__main__":
    unittest.main()
