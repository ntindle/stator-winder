"""Source-level checks for the explicit M1 workholding options."""

import unittest

from build123d import Align, Cylinder

import assembly
from params import PARAMS, SPINDLE_OPTIONS, StatorSpec, spindle_option


CTR_MAX = (Align.CENTER, Align.CENTER, Align.MAX)


class SpindleOptionTests(unittest.TestCase):
    def test_option_capacity_is_explicit_and_complete(self):
        self.assertEqual(set(SPINDLE_OPTIONS), {"er11", "shaft8"})
        er11 = spindle_option("er11")
        shaft8 = spindle_option("shaft8")
        self.assertEqual((er11.shaft_d_min, er11.shaft_d_max), (3.0, 7.0))
        self.assertEqual((shaft8.shaft_d_min, shaft8.shaft_d_max), (8.0, 8.0))
        self.assertEqual(er11.shank_d, 8.0)
        self.assertEqual(shaft8.shank_d, 8.0)
        self.assertEqual(
            er11.manifest_record()["changeover_interface_id"],
            shaft8.manifest_record()["changeover_interface_id"],
        )

    def test_custom_shaft8_holder_is_one_solid_with_clear_socket(self):
        holder = assembly.shaft8_socket_holder()
        bbox = holder.bounding_box()
        self.assertEqual(len(holder.solids()), 1)
        self.assertAlmostEqual(bbox.size.X, 16.0, places=5)
        self.assertAlmostEqual(bbox.size.Y, 16.0, places=5)
        self.assertAlmostEqual(bbox.min.Z, -116.0, places=5)
        self.assertAlmostEqual(bbox.max.Z, 0.0, places=5)

        # The 8.00 mm shaft has 0.05 mm radial clearance throughout its
        # required 12 mm insertion; positive common volume would mean the
        # drawing does not actually accept the endpoint shaft.
        nominal_shaft = Cylinder(4.0, 12.0, align=CTR_MAX)
        common = holder & nominal_shaft
        self.assertLessEqual(common.volume, 1e-7)

    def test_neck_profiles_match_the_selected_physical_holder(self):
        spec = StatorSpec(od=28.0, stack=8.4, shaft_d=8.0,
                          wire_d=0.20, turns=1)
        profile = PARAMS.chuck_neck_profile(spec, "shaft8")
        # exposed shaft, OD16x16 clamp body, then shared OD8 shank
        self.assertEqual([round(2.0 * row[0], 6) for row in profile],
                         [8.0, 16.0, 8.0])
        self.assertAlmostEqual(profile[1][1] - profile[1][2], 16.0)

    def test_assembly_uses_one_stable_holder_label_for_both_options(self):
        for option, shaft_d in (("er11", 4.0), ("shaft8", 8.0)):
            spec = StatorSpec(od=28.0, stack=8.4, shaft_d=shaft_d,
                              wire_d=0.20, turns=1)
            parts = assembly.spindle_link(spec, spindle=option)
            labels = [part.label for part in parts]
            self.assertEqual(labels.count("spindle_holder"), 1)
            self.assertNotIn("er11_chuck", labels)


if __name__ == "__main__":
    unittest.main(verbosity=2)
