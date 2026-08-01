"""Regression tests for the isolated aggregate-boundary follower CAD."""

from __future__ import annotations

import itertools
from pathlib import Path
import sys
import unittest

from build123d import Align, Cylinder, Pos, Rot, Vector

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import aggregate_boundary_floating_follower as follower


CTR = (Align.CENTER, Align.CENTER, Align.CENTER)


class AggregateBoundaryFloatingFollowerCadTests(unittest.TestCase):

    def test_strokes_include_required_usable_range_and_overtravel_stops(self):
        contract = follower.geometry_contract()["stroke_contract"]
        self.assertEqual(contract["radial_stroke_mm"], 6.0)
        self.assertEqual(contract["tangential_stroke_mm"], 1.0)
        self.assertEqual(contract["radial_hard_center_travel_mm"], 6.4)
        self.assertEqual(
            contract["tangential_hard_center_stops_mm"], [-0.6, 0.6])
        self.assertTrue(contract["all_endpoint_tongues_captured"])

    def test_all_endpoint_custom_bodies_are_single_solids_without_overlap(self):
        for radial, tangential in itertools.product(
            ("retracted", "mid", "extended"),
            ("negative", "center", "positive"),
        ):
            with self.subTest(radial=radial, tangential=tangential):
                parts = follower.custom_bodies(radial, tangential)
                self.assertTrue(all(part.volume > 0.0 for part in parts))
                self.assertTrue(all(len(part.solids()) == 1 for part in parts))
                audit = follower.same_state_overlap_audit(radial, tangential)
                self.assertEqual(audit["status"], "PASS")
                self.assertEqual(audit["positive_overlap_count"], 0)

    def test_nose_has_z_axis_r3_floor_open_groove_and_no_peek_thread(self):
        nose = follower.nose_insert("mid", "center")
        x, y, z = follower._gimbal_center("mid", "center")
        x += 8.0
        self.assertFalse(nose.is_inside(Vector(x, y, z)))
        self.assertTrue(nose.is_inside(Vector(x + 2.99, y, z)))
        self.assertFalse(nose.is_inside(Vector(x + 3.10, y, z)))
        self.assertTrue(nose.is_inside(Vector(x + 3.40, y, z + 1.0)))
        contract = follower.geometry_contract()["nose_contract"]
        self.assertEqual(contract["contact_surface_radius_mm"], 3.0)
        self.assertEqual(contract["nose_cylinder_axis"], "+Z_stator_axis")
        self.assertFalse(
            follower.geometry_contract()["fastener_contract"]
            ["direct_PEEK_threads"]
        )

    def test_od5_outer_pivot_clears_both_yokes_and_uses_din988_shim(self):
        x, y, z = follower._gimbal_center("mid", "center")
        pin = Pos(x, y, z) * (
            Rot(90.0, 0.0, 0.0)
            * Cylinder(2.5, 10.0, align=CTR)
        )
        self.assertAlmostEqual(
            follower._common_volume(pin, follower.outer_gimbal_yoke()),
            0.0, places=7,
        )
        self.assertAlmostEqual(
            follower._common_volume(pin, follower.inner_gimbal_yoke()),
            0.0, places=7,
        )
        labels = [part.label for part in follower.gimbal_pin_hardware()]
        self.assertIn("outer_pivot_DIN988_5x10x0p5_shim", labels)
        self.assertIn("outer_pivot_DIN988_5x10x0p5_far_shim", labels)
        self.assertIn(
            "inner_pivot_McMaster_90265A115_OD3x10_M2", labels)
        self.assertEqual(sum("inner_pivot_DIN988_3x6x0p5" in x
                             for x in labels), 4)
        self.assertIn("inner_pivot_M2_nyloc", labels)
        self.assertFalse(any("90265A420" in x for x in labels))
        contract = follower.geometry_contract()["fastener_contract"]
        self.assertEqual(contract["qualified_shoulder_pin_stack_count"], 2)
        self.assertEqual(contract["inner_pivot"]["sku"],
                         "McMaster 90265A115")

        nose_x = x + 8.0
        inner_pin = Pos(nose_x, y, z) * Cylinder(1.5, 10.0, align=CTR)
        self.assertAlmostEqual(
            follower._common_volume(inner_pin, follower.inner_gimbal_yoke()),
            0.0, places=7,
        )
        self.assertAlmostEqual(
            follower._common_volume(inner_pin, follower.nose_insert()),
            0.0, places=7,
        )

    def test_primary_tower_mount_has_four_complete_m4_stacks(self):
        hardware = follower.tower_m4_hardware()
        labels = [part.label for part in hardware]
        self.assertEqual(len(hardware), 12)
        self.assertEqual(sum("M4x10" in x for x in labels), 4)
        self.assertEqual(sum("M4_washer" in x for x in labels), 4)
        self.assertEqual(sum("M4_short_heat_insert" in x for x in labels), 4)
        self.assertTrue(all(part.volume > 0.0 for part in hardware))
        contract = follower.geometry_contract()["fastener_contract"]
        self.assertEqual(contract["primary_load_case_N"], 40.0)
        self.assertEqual(contract["primary_load_per_M4_N"], 10.0)
        self.assertFalse(contract["secondary_M3_structural_proof_claimed"])

    def test_tangential_slide_and_outer_yoke_are_one_positive_cartridge(self):
        cartridge = follower.tangential_slide_outer_gimbal_cartridge()
        self.assertEqual(len(cartridge.solids()), 1)
        self.assertGreater(cartridge.volume, 0.0)
        contract = follower.geometry_contract()["monolithic_cartridge_contract"]
        self.assertEqual(contract["modeled_positive_throat_mm"],
                         [5.0, 4.0, 1.0])
        self.assertFalse(contract["separate_slide_to_yoke_fasteners_required"])
        self.assertTrue(contract["root_blend_geometry_modeled"])

    def test_retraction_is_not_authorized_by_unattached_dock_concept(self):
        contract = follower.geometry_contract()
        selection = contract["selection_and_retraction"]
        self.assertEqual(selection["M0_positive_dock"],
                         "UNATTACHED_CONCEPT_ONLY")
        self.assertFalse(selection["M0_dock_attached_to_actuator"])
        self.assertFalse(contract["authority"]["fail_retraction_authorized"])
        self.assertIn(
            "UNMODELED_M0_positive_retraction_linkage",
            contract["procurement_blockers"],
        )


if __name__ == "__main__":
    unittest.main()
