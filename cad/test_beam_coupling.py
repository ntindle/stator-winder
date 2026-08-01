"""Regression checks for the selected M0/M1 shaft coupling."""

import unittest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import cots
from params import PARAMS as P


class BeamCouplingTests(unittest.TestCase):
    def test_selected_supplier_body_fits_conservative_collision_envelope(self):
        box = cots.beam_coupling_5x8().bounding_box().size
        dimensions = sorted((box.X, box.Y, box.Z))
        self.assertGreaterEqual(dimensions[0], P.coupling_5x8_od - 1e-6)
        self.assertGreaterEqual(dimensions[1], P.coupling_5x8_od - 1e-6)
        self.assertGreaterEqual(dimensions[2], P.coupling_5x8_length - 1e-6)

    def test_installed_shaft_engagement_is_physical(self):
        engagement = (P.coupling_5x8_length - 2.0) / 2.0
        self.assertAlmostEqual(engagement, 12.5)
        self.assertLessEqual(engagement, P.coupling_5x8_shaft_penetration)

    def test_reversing_capacity_has_two_x_margin_on_both_axes(self):
        # Same conservative simulation duties emitted by loads.py.
        self.assertGreaterEqual(
            P.coupling_5x8_dynamic_reversing_nm / 0.0266, 2.0)
        self.assertGreaterEqual(
            P.coupling_5x8_dynamic_reversing_nm / 0.0625, 2.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
