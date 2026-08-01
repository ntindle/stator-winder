"""Deterministic interface checks for the selected Omron home switch."""

from pathlib import Path
import math
import sys
import unittest

from build123d import Align, Cylinder, Pos, Rot


sys.path.insert(0, str(Path(__file__).resolve().parent))

import cots  # noqa: E402
import carriage_endstop_flag  # noqa: E402
from params import PARAMS as P  # noqa: E402


class OmronEndstopTests(unittest.TestCase):
    def test_controlled_envelope_is_single_solid(self):
        switch = cots.endstop()
        self.assertTrue(switch.is_valid)
        self.assertEqual(len(switch.solids()), 1)
        bb = switch.bounding_box()
        self.assertTrue(math.isclose(bb.size.X, 12.8, abs_tol=1e-6))
        self.assertTrue(math.isclose(bb.size.Y, 5.8, abs_tol=1e-6))
        self.assertTrue(math.isclose(bb.min.Z, -3.4, abs_tol=1e-6))
        self.assertTrue(math.isclose(bb.max.Z, 18.9, abs_tol=1e-6))

    def test_both_controlled_mount_holes_are_open(self):
        switch = cots.endstop()
        for x in P.endstop_switch_hole_x:
            probe = Pos(x, 0.0, 3.15) * (
                Rot(90, 0, 0) * Cylinder(
                    1.0, 8.0,
                    align=(Align.CENTER, Align.CENTER, Align.CENTER),
                )
            )
            self.assertAlmostEqual((switch & probe).volume, 0.0, places=7)

    def test_home_gap_matches_revised_flag(self):
        roller_near_z = P.endstop_switch_origin_z - 18.9
        flag_rear_z = carriage_endstop_flag.TRIGGER_Z[1]
        self.assertAlmostEqual(roller_near_z, 144.6, places=6)
        self.assertAlmostEqual(flag_rear_z, 142.0, places=6)
        self.assertAlmostEqual(roller_near_z - flag_rear_z, 2.6, places=6)

    def test_hole_datum_matches_assembly_rotation(self):
        self.assertEqual(P.endstop_switch_hole_x, (-3.25, 3.25))
        self.assertAlmostEqual(
            P.endstop_switch_origin_z - 3.15,
            P.endstop_switch_hole_z,
            places=6,
        )


if __name__ == "__main__":
    unittest.main()
