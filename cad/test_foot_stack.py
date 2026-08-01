"""Exact installed-geometry checks for the selected 35 mm foot stack."""

from pathlib import Path
import math
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parent))

import assembly  # noqa: E402
import cots  # noqa: E402
import hardware  # noqa: E402
import hardware_placements as placements  # noqa: E402
from params import PARAMS as P  # noqa: E402


class FootStackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        occurrences = placements.hardware_occurrences_by_link(P)["static"]
        cls.by_label = {row.label: row for row in occurrences}

    def _part(self, label):
        return self.by_label[label].build()

    def test_cots_solids_match_controlled_dimensions(self):
        foot = hardware.machine_foot_m5_17()
        standoff = hardware.foot_standoff_m5_ff_18()
        self.assertEqual(len(foot.solids()), 1)
        self.assertEqual(len(standoff.solids()), 1)
        foot_bb = foot.bounding_box()
        stand_bb = standoff.bounding_box()
        self.assertAlmostEqual(foot_bb.size.X, 20.0, places=6)
        self.assertAlmostEqual(foot_bb.min.Z, -17.0, places=6)
        self.assertAlmostEqual(foot_bb.max.Z, 6.0, places=6)
        self.assertAlmostEqual(stand_bb.size.Y, 8.0, places=6)
        self.assertAlmostEqual(stand_bb.size.Z, 18.0, places=6)

    def test_each_stack_has_all_four_hardware_occurrences(self):
        for index in range(4):
            for suffix in ("", "_standoff", "_set_screw", "_tnut"):
                self.assertIn(f"foot_{index}{suffix}", self.by_label)

    def test_installed_vertical_datums_preserve_bench_clearance(self):
        foot = self._part("foot_0").bounding_box()
        standoff = self._part("foot_0_standoff").bounding_box()
        screw = self._part("foot_0_set_screw").bounding_box()
        tnut = self._part("foot_0_tnut").bounding_box()

        self.assertAlmostEqual(foot.min.Y, -260.0, places=6)
        self.assertAlmostEqual(foot.max.Y, -237.0, places=6)
        self.assertAlmostEqual(standoff.min.Y, -243.0, places=6)
        self.assertAlmostEqual(standoff.max.Y, -225.0, places=6)
        self.assertAlmostEqual(screw.min.Y, -232.2, places=6)
        self.assertAlmostEqual(screw.max.Y, -220.2, places=6)
        self.assertAlmostEqual(tnut.min.Y, -223.5, places=6)
        self.assertAlmostEqual(tnut.max.Y, -220.3, places=6)
        self.assertAlmostEqual(-251.65 - foot.min.Y, 8.35, places=6)

    def test_rail_slot_clears_projected_set_screw(self):
        rail = assembly._at(
            cots.extrusion_2020(450),
            -P.base_rail_x, -215.0, P.frame_z0,
            label="test_base_rail",
        )
        screw = self._part("foot_0_set_screw")
        standoff = self._part("foot_0_standoff")
        tnut = self._part("foot_0_tnut")
        self.assertTrue(math.isclose((rail & screw).volume, 0.0,
                                     abs_tol=1e-7))
        self.assertTrue(math.isclose((rail & standoff).volume, 0.0,
                                     abs_tol=1e-7))
        self.assertTrue(math.isclose((rail & tnut).volume, 0.0,
                                     abs_tol=1e-7))


if __name__ == "__main__":
    unittest.main()
