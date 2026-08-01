"""Tests for the released Rev-D D10/ID6-to-ID9 flyer shaft geometry."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import flyer_shaft_d10 as shaft


class FlyerShaftD10Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.part = shaft.flyer_shaft()

    def test_single_solid_and_primary_dimensions(self):
        self.assertEqual(len(self.part.solids()), 1)
        self.assertTrue(self.part.is_valid)
        box = self.part.bounding_box()
        self.assertAlmostEqual(box.size.X, 12.0, places=5)
        self.assertAlmostEqual(box.size.Y, 12.0, places=5)
        self.assertAlmostEqual(box.size.Z, 79.0, places=5)
        self.assertAlmostEqual(box.min.Z, shaft.LOCAL_REAR_Z_MM, places=5)
        self.assertAlmostEqual(box.max.Z, shaft.LOCAL_FRONT_Z_MM, places=5)

    def test_d10_seat_and_wire_transition_contract(self):
        self.assertEqual(shaft.NECK_LENGTH_MM, 18.5)
        self.assertEqual(shaft.MIN_NECK_RADIAL_WALL_MM, 2.0)
        self.assertEqual(shaft.NECK_OD_MM, 10.0)
        self.assertEqual(shaft.NECK_ID_MM, 6.0)
        self.assertEqual(shaft.NECK_OD_H6_LIMITS_MM, (9.991, 10.0))
        self.assertEqual(shaft.NECK_ID_LIMITS_MM, (6.0, 6.03))
        self.assertAlmostEqual(shaft.MIN_NECK_RADIAL_WALL_AT_LIMITS_MM, 1.9805)
        self.assertEqual(shaft.MAIN_ID_MM, 9.0)
        self.assertEqual(shaft.MAIN_ID_LIMITS_MM, (9.0, 9.05))
        self.assertEqual(shaft.TRANSITION_LENGTH_MM, 3.0)
        self.assertEqual(shaft.TRANSITION_END_WORLD_Z_MM, -89.25)
        self.assertEqual(shaft.ARM_FLAT_WORLD_Z_MM, -46.0)
        self.assertEqual(shaft.ARM_FLAT_STATION_FROM_REAR_MM, 64.75)
        self.assertEqual(shaft.WORLD_REAR_Z_MM, -110.75)
        self.assertEqual(shaft.WORLD_FRONT_Z_MM, -31.75)
        self.assertEqual(shaft.LABEL, "flyer_shaft_d10_id6_to_id9_l79")


if __name__ == "__main__":
    unittest.main(verbosity=2)
