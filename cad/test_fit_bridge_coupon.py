"""Exact geometry tests for the isolated print-qualification coupon."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

from build123d import Align, Box, Cylinder, Pos


sys.path.insert(0, str(Path(__file__).resolve().parent))
import fit_bridge_coupon as coupon  # noqa: E402


CYLINDER_ZMIN = (Align.CENTER, Align.CENTER, Align.MIN)


class FitBridgeCouponTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.part = coupon.gen_step()

    def test_single_solid_and_exact_envelope(self):
        self.assertEqual(len(self.part.solids()), 1)
        bb = self.part.bounding_box()
        self.assertAlmostEqual(bb.min.X, -90.0, places=6)
        self.assertAlmostEqual(bb.max.X, 90.0, places=6)
        self.assertAlmostEqual(bb.min.Y, -45.0, places=6)
        self.assertAlmostEqual(bb.max.Y, 45.0, places=6)
        self.assertAlmostEqual(bb.min.Z, 0.0, places=6)
        self.assertAlmostEqual(bb.max.Z, 24.0, places=6)

    def test_a1_slicer_variant_is_only_translated_inside_bed(self):
        placed = coupon.gen_a1_plate_part()
        bb = placed.bounding_box()
        self.assertEqual(len(placed.solids()), 1)
        self.assertAlmostEqual(placed.volume, self.part.volume, places=5)
        self.assertGreaterEqual(bb.min.X, 0.0)
        self.assertGreaterEqual(bb.min.Y, 0.0)
        self.assertLessEqual(bb.max.X, 256.0)
        self.assertLessEqual(bb.max.Y, 256.0)
        self.assertAlmostEqual(bb.min.Z, 0.0, places=6)
        self.assertAlmostEqual(bb.max.Z, 24.0, places=6)

    def test_every_production_fit_diameter_is_present(self):
        radii = [face.radius for face in self.part.faces()
                 if isinstance(getattr(face, "radius", None), (int, float))]
        required = [row["diameter_mm"] / 2.0
                    for row in coupon.BEARING_GAUGES]
        required.extend((coupon.PULLEY_BORE_DIAMETER / 2.0,
                         coupon.ELBOW_SLEEVE_DIAMETER / 2.0,
                         coupon.INSERT_PILOT_DIAMETER / 2.0))
        for radius in required:
            self.assertTrue(any(math.isclose(radius, actual, abs_tol=1e-7)
                                for actual in radii), radius)

    def test_bearing_and_pulley_gauges_are_open_through(self):
        rows = [*coupon.BEARING_GAUGES, {
            "diameter_mm": coupon.PULLEY_BORE_DIAMETER,
            "center": coupon.PULLEY_CENTER,
        }]
        for row in rows:
            x, y = row["center"]
            probe = Pos(x, y, -0.5) * Cylinder(
                row["diameter_mm"] / 2.0 - 0.05, 17.0,
                align=CYLINDER_ZMIN)
            self.assertLess((self.part & probe).volume, 1e-7,
                            str(row))

    def test_elbow_male_gauge_has_full_height(self):
        x, y = coupon.ELBOW_CENTER
        probe = Pos(x, y, coupon.BASE_T + 0.01) * Cylinder(
            coupon.ELBOW_SLEEVE_DIAMETER / 2.0 - 0.05,
            coupon.ELBOW_PEG_H - 0.02, align=CYLINDER_ZMIN)
        self.assertAlmostEqual((self.part & probe).volume,
                               probe.volume, places=5)

    def test_heat_set_pilot_depths_are_exact_and_blind(self):
        for row in coupon.INSERT_PILOTS:
            x, y = row["center"]
            top = coupon.BASE_T + coupon.INSERT_BOSS_H
            void = Pos(x, y, top - row["depth_mm"] + 0.01) * Cylinder(
                coupon.INSERT_PILOT_DIAMETER / 2.0 - 0.05,
                row["depth_mm"] + 0.5, align=CYLINDER_ZMIN)
            self.assertLess((self.part & void).volume, 1e-7, str(row))
            floor = Pos(x, y, top - row["depth_mm"] - 0.25) * Cylinder(
                coupon.INSERT_PILOT_DIAMETER / 2.0 - 0.1,
                0.20, align=CYLINDER_ZMIN)
            self.assertAlmostEqual((self.part & floor).volume,
                                   floor.volume, places=5)

    def test_bridge_has_exact_clear_gap_and_roof(self):
        inner_left = coupon.BRIDGE_CENTER_X - coupon.BRIDGE_GAP / 2.0
        y0 = coupon.BRIDGE_CENTER_Y - coupon.BRIDGE_WIDTH / 2.0
        gap = Pos(inner_left + 0.01, y0 + 0.01, coupon.BASE_T + 0.01) * Box(
            coupon.BRIDGE_GAP - 0.02,
            coupon.BRIDGE_WIDTH - 0.02,
            coupon.BRIDGE_UNDERSIDE_Z - coupon.BASE_T - 0.02,
            align=Align.MIN)
        self.assertLess((self.part & gap).volume, 1e-7)
        roof = Pos(inner_left + 0.01, y0 + 0.01,
                   coupon.BRIDGE_UNDERSIDE_Z + 0.01) * Box(
            coupon.BRIDGE_GAP - 0.02,
            coupon.BRIDGE_WIDTH - 0.02,
            coupon.BRIDGE_ROOF_T - 0.02,
            align=Align.MIN)
        self.assertAlmostEqual((self.part & roof).volume,
                               roof.volume, places=5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
