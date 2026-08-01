"""Focused checks for the isolated permanent guide-cap review CAD."""

import math
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import stator_winding_guide_cap as cap


class StatorWindingGuideCapCadTests(unittest.TestCase):
    def test_max_launch_wire_keeps_R3_centerline_contact(self):
        self.assertTrue(math.isclose(
            cap.HORN_CONTACT_SURFACE_RADIUS_MM, 2.75))
        self.assertTrue(math.isclose(
            cap.HORN_CONTACT_SURFACE_RADIUS_MM
            + cap.MAXIMUM_LAUNCH_WIRE_RADIUS_MM,
            3.0,
        ))

    def test_planar_seed_span_has_negative_R3_bridge(self):
        seed_profile = 0.23876
        span = 2.0 * (cap.tooth_half_width_mm() + seed_profile)
        self.assertTrue(math.isclose(span, 3.69752, abs_tol=1e-9))
        self.assertTrue(math.isclose(span - 6.0, -2.30248, abs_tol=1e-9))

    def test_each_review_cap_has_exact_face_plus_24_open_ribs_and_pads(self):
        parts = cap.guide_cap_parts(1)
        self.assertEqual(len(parts), 1 + 2 * 24)
        self.assertGreaterEqual(len(parts[0].solids()), 1)
        self.assertTrue(all(len(part.solids()) == 1 for part in parts[1:]))


if __name__ == "__main__":
    unittest.main()
