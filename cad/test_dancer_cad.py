"""BREP regression tests for the integrated moving dancer geometry."""

from pathlib import Path
import sys
import unittest

from build123d import Pos, Rot

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hardware_placements
from params import PARAMS as P
import printed


def moved_arm(offset_deg):
    return (Pos(P.rear_post_x, P.dancer_y, 0) * Rot(0, 0, offset_deg) *
            Pos(-P.rear_post_x, -P.dancer_y, 0) * printed.dancer_arm())


class DancerCadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hardware = {
            occurrence.label: occurrence.build()
            for occurrence in hardware_placements.static_occurrences()
        }
        cls.base = printed.dancer_base()
        cls.entry = printed.entry_bracket()
        cls.spring = cls.hardware["dancer_extension_spring"]
        cls.stops = [cls.hardware[f"dancer_stop_{i}_od5_sleeve"]
                     for i in (1, 2)]

    def test_nominal_axial_separations_match_audit(self):
        arm = moved_arm(0.0)
        self.assertAlmostEqual(arm.distance_to(self.base), 1.0, places=6)
        self.assertAlmostEqual(arm.distance_to(self.entry), 1.0, places=6)
        self.assertAlmostEqual(arm.distance_to(self.spring), 4.5, places=6)

    def test_full_stop_to_stop_sweep_has_no_positive_volume_overlap(self):
        offset = -3.0
        while offset <= 5.500001:
            arm = moved_arm(offset)
            for obstacle in (self.base, self.entry, self.spring, *self.stops):
                self.assertLess((arm & obstacle).volume, 1e-6,
                                f"offset={offset}, obstacle={obstacle.label}")
            offset += 0.25

    def test_only_selected_sleeve_contacts_at_each_stop(self):
        lower = moved_arm(-3.0)
        upper = moved_arm(5.5)
        self.assertLess(lower.distance_to(self.stops[0]), 1e-6)
        self.assertGreater(lower.distance_to(self.stops[1]), 2.0)
        self.assertGreater(upper.distance_to(self.stops[0]), 2.0)
        self.assertLess(upper.distance_to(self.stops[1]), 1e-6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
