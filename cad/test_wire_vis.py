"""Regression tests for honest wire presentation geometry."""

import unittest

from build123d import Pos

import hardware
from params import DEFAULT_STATOR
import wire_vis


class WireVisualTests(unittest.TestCase):
    def test_visual_radius_is_physical_default_job_radius(self):
        self.assertAlmostEqual(
            wire_vis.R_VIS, DEFAULT_STATOR.wire_d / 2.0, places=12)

    def test_unloaded_felt_pads_do_not_intersect_visual_wire(self):
        wire = wire_vis.wire_static()
        fixed = Pos(-45.0, -40.0, -160.25) * hardware.felt_washer(
            20.0, 4.5, 3.0, label="fixed")
        moving = Pos(-45.0, -40.0, -156.75) * hardware.felt_washer(
            20.0, 4.5, 3.0, label="moving")
        fixed_overlap = wire.intersect(fixed)
        moving_overlap = wire.intersect(moving)
        self.assertLessEqual(
            0.0 if fixed_overlap is None else fixed_overlap.volume, 1e-9)
        self.assertLessEqual(
            0.0 if moving_overlap is None else moving_overlap.volume, 1e-9)
        # The 0.5 mm rigid gap is an unloaded envelope. In use the spring and
        # compliant felt close onto this physical wire; that deformation is
        # intentionally not represented as a rigid-solid overlap.
        self.assertAlmostEqual(fixed.distance_to(moving), 0.5, places=9)


if __name__ == "__main__":
    unittest.main(verbosity=2)
