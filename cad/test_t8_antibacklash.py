"""Geometry and purchasing regressions for the complete T8x8 nut set."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest

from build123d import Pos

sys.path.insert(0, str(Path(__file__).resolve().parent))
import assembly
import cots
from params import PARAMS as P


class T8AntiBacklashTests(unittest.TestCase):
    def test_supplier_drawing_envelopes(self):
        main = cots.t8_nut().bounding_box()
        secondary = cots.t8_nut_secondary().bounding_box()
        spring = cots.t8_nut_spring_envelope().bounding_box()

        self.assertAlmostEqual(main.size.X, 22.0, places=6)
        self.assertAlmostEqual(main.size.Y, 22.0, places=6)
        self.assertAlmostEqual(main.min.Z, 0.0, places=6)
        self.assertAlmostEqual(main.max.Z, 15.0, places=6)
        self.assertAlmostEqual(secondary.size.X, 14.0, places=6)
        self.assertAlmostEqual(secondary.size.Y, 14.0, places=6)
        self.assertAlmostEqual(secondary.max.Z, 22.4, places=6)
        self.assertAlmostEqual(spring.size.X, 14.0, places=6)
        self.assertAlmostEqual(spring.min.Z, 4.0, places=6)
        self.assertAlmostEqual(spring.max.Z, 18.4, places=6)

    def test_all_three_supplied_components_are_in_the_carriage_link(self):
        labels = {part.label for part in assembly.carriage_link()}
        self.assertTrue({
            "t8_nut_main", "t8_nut_spring", "t8_nut_secondary",
        }.issubset(labels))

    def test_complete_set_remains_on_screw_at_hard_stop(self):
        # At a carriage-axis position z, the main-nut flange datum is z-18.
        lowest_nut_face = (P.m0_axis_z_min - 18.0
                           - cots.T8_AB_INSTALLED_LENGTH)
        self.assertLessEqual(P.screw_z0, lowest_nut_face)
        self.assertGreaterEqual(lowest_nut_face - P.screw_z0, 0.5)
        self.assertAlmostEqual(
            P.m0_travel,
            P.m0_home_standoff - P.m0_axis_z_min,
            places=9,
        )

    def test_new_components_do_not_intersect_static_machine_over_travel(self):
        links = assembly.build_links()
        nuts = [part for part in links["carriage"] if part.label in {
            "t8_nut_main", "t8_nut_spring", "t8_nut_secondary",
        }]
        self.assertEqual(len(nuts), 3)

        def boxes_overlap(a, b):
            aa, bb = a.bounding_box(), b.bounding_box()
            return all((getattr(aa.min, axis) <= getattr(bb.max, axis) + 1e-9
                        and getattr(bb.min, axis) <= getattr(aa.max, axis) + 1e-9)
                       for axis in ("X", "Y", "Z"))

        # The lead screw passes through the declared bores and is checked
        # separately below.  No other static solid may occupy the new swept
        # nut/spring volumes at home or the hard stop.
        obstacles = [part for part in links["static"]
                     if part.label != "t8_screw"]
        for axis_z in (P.m0_home_standoff, P.m0_axis_z_min):
            dz = axis_z - P.m0_home_standoff
            for source in nuts:
                moved = Pos(0, 0, dz) * source
                for obstacle in obstacles:
                    if not boxes_overlap(moved, obstacle):
                        continue
                    self.assertLess(
                        (moved & obstacle).volume,
                        1e-6,
                        f"axis_z={axis_z} {source.label} vs {obstacle.label}",
                    )

        screw = next(part for part in links["static"]
                     if part.label == "t8_screw")
        for axis_z in (P.m0_home_standoff, P.m0_axis_z_min):
            dz = axis_z - P.m0_home_standoff
            for source in nuts:
                moved = Pos(0, 0, dz) * source
                self.assertLess((moved & screw).volume, 1e-6)
                self.assertGreaterEqual(moved.distance_to(screw), 0.049)

    def test_release_line_is_exact_and_has_receiving_gate(self):
        catalog = json.loads(
            (Path(__file__).with_name("release_catalog.json"))
            .read_text(encoding="utf-8")
        )
        item = next(row for row in catalog["items"]
                    if row["id"] == "t8-antibacklash-flange-nut")
        self.assertEqual(item["purchase_status"], "cart_ready")
        self.assertEqual(item["selection"]["supplier_sku"],
                         "HW-SC-SMALLFL-P / T8x8 option")
        self.assertEqual(
            item["receiving_contract"]["configured_installed_length_max_mm"],
            cots.T8_AB_INSTALLED_LENGTH,
        )
        self.assertIn("4-start", item["receiving_contract"]["required_option"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
