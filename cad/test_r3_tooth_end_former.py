"""Unit checks for the isolated retained R3 former review CAD."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

import numpy as np


HERE = Path(__file__).resolve().parent
SIM = HERE.parent / "sim"
for path in (HERE, SIM):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import r3_bend_scope_feasibility as scope  # noqa: E402
import r3_tooth_end_former as former  # noqa: E402


class R3ToothEndFormerCadTest(unittest.TestCase):
    def test_constants_match_advisory_exactly(self):
        params = scope.lrl_parameters()
        self.assertAlmostEqual(
            former.BASE_WIRE_RADIUS_MM, params["base_radius_mm"], places=12)
        self.assertAlmostEqual(
            former.PACKING_Q_STEP_MM, params["offset_step_mm"], places=12)
        self.assertAlmostEqual(
            former.BASE_FIRST_ARC_RAD, params["alpha_rad"], places=12)
        self.assertAlmostEqual(
            former.BASE_WIRE_RADIUS_MM - former.PACKING_Q_MAX_MM,
            3.0, places=12)

    def test_exact_50_row_witness_matches_scope(self):
        rows = former.packing_rows()
        advisory = scope.square_row_centres()
        self.assertEqual(len(rows), 50)
        self.assertEqual(
            [row.tangential_layer for row in rows],
            [int(row["layer_index"]) for row in advisory])
        for actual, expected in zip(rows, advisory):
            self.assertAlmostEqual(
                actual.radial_mm, float(expected["tooth_x_mm"]), places=11)
            self.assertAlmostEqual(
                former.FIRST_WIRE_HALF_SPAN_MM + actual.q_mm,
                float(expected["tooth_half_span_mm"]), places=11)

    def test_front_rear_wire_caps_are_exact_mirrors(self):
        row = former.packing_rows()[-1]
        front = former.wire_cap_points(row, +1, lane_mm=12.0)
        rear = former.wire_cap_points(row, -1, lane_mm=12.0)
        expected = front * np.array((1.0, 1.0, -1.0))
        self.assertTrue(np.allclose(rear, expected, atol=1e-11, rtol=0.0))
        self.assertAlmostEqual(front[0, 2], 7.5, places=12)
        self.assertAlmostEqual(front[-1, 2], 7.5, places=12)

    def test_paddle_is_one_valid_od_bounded_solid(self):
        paddle = former.tooth_paddle(+1, lane_mm=12.0)
        self.assertTrue(paddle.is_valid)
        self.assertEqual(len(paddle.solids()), 1)
        box = paddle.bounding_box()
        self.assertGreaterEqual(box.min.X, former.RADIAL_SURFACE_MIN_MM - 1e-9)
        self.assertLessEqual(box.max.X, former.RADIAL_SURFACE_MAX_MM + 1e-9)
        self.assertLess(former.RADIAL_SURFACE_MAX_MM, 23.0)

    def test_tooth_face_strap_does_not_widen_the_neck(self):
        self.assertLessEqual(
            former.TOOTH_FACE_STRAP_WIDTH_MM, 2.0 * former.HALF_NECK_MM)
        self.assertGreater(
            former.RADIAL_SURFACE_MAX_MM,
            max(row.radial_mm for row in former.packing_rows()))


if __name__ == "__main__":
    unittest.main()
