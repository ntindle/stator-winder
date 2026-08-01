"""Focused geometry and drawing-source tests for custom shop artifacts."""

import math
import unittest

from ezdxf import units
from build123d import GeomType

import custom_parts
import flyer_shaft_d10
import felt_backing_disc
import felt_pad
from shop_artifacts import (
    EXTRUSION_CUTS,
    SLEEVE_SPECS,
    extrusion_total_mm,
    sleeve_part,
)


def _bbox_dimensions(part):
    box = part.bounding_box()
    return (
        box.max.X - box.min.X,
        box.max.Y - box.min.Y,
        box.max.Z - box.min.Z,
    )


class ShopArtifactTests(unittest.TestCase):
    def _assert_annular_dxf(self, document):
        self.assertEqual(document.units, units.MM)
        entities = list(document.modelspace().query('CIRCLE[layer=="CUT"]'))
        self.assertEqual(len(entities), 2)
        self.assertEqual(
            sorted(round(float(entity.dxf.radius), 6) for entity in entities),
            [2.25, 10.0],
        )

    def test_felt_backing_disc_dxf_is_exact_1_to_1_annulus(self):
        self._assert_annular_dxf(felt_backing_disc.gen_dxf())

    def test_felt_pad_dxf_is_exact_1_to_1_annulus(self):
        self._assert_annular_dxf(felt_pad.gen_dxf())

    def test_every_dancer_sleeve_is_one_solid_with_exact_bbox(self):
        for spec in SLEEVE_SPECS.values():
            with self.subTest(spec=spec.part_id):
                part = sleeve_part(spec)
                self.assertEqual(len(part.solids()), 1)
                dimensions = _bbox_dimensions(part)
                expected = (
                    spec.outer_diameter_mm,
                    spec.outer_diameter_mm,
                    spec.length_mm,
                )
                for actual, target in zip(dimensions, expected):
                    self.assertAlmostEqual(actual, target, places=6)

    def test_extrusion_cut_list_has_ten_members_and_2775_mm(self):
        self.assertEqual(sum(row.quantity for row in EXTRUSION_CUTS), 10)
        self.assertAlmostEqual(extrusion_total_mm(), 2775.0, places=6)

    def test_existing_custom_release_parts_are_single_solids(self):
        parts = custom_parts.release_parts()
        self.assertGreaterEqual(len(parts), 13)
        self.assertNotIn("tip_toroid_guide", parts)
        for name, part in parts.items():
            with self.subTest(part=name):
                self.assertEqual(len(part.solids()), 1)

    def test_existing_custom_release_critical_bounding_dimensions(self):
        parts = custom_parts.release_parts()
        expected = {
            "fixed_eyelet_id4_od9_t3": (9.0, 9.0, 3.0),
            "shaft_wrap_sleeve_d4": (8.0, 8.0, 6.0),
            "t8x8_leadscrew_188_journal30": (8.0, 8.0, 188.0),
            "flyer_shaft_d10_id6_to_id9_l79": (12.0, 12.0, 79.0),
            "m0_inner_shim": (12.0, 12.0, 1.0),
            "m1_outer_race_spacer": (21.8, 21.8, 16.0),
            "m2_outer_race_spacer": (27.8, 27.8, 11.0),
            "m2_center_inner_spacer": (17.8, 17.8, 11.0),
        }
        for name, target in expected.items():
            with self.subTest(part=name):
                for actual, wanted in zip(_bbox_dimensions(parts[name]), target):
                    self.assertTrue(math.isclose(actual, wanted, abs_tol=1e-5))

    def test_released_stock_d10_shaft_has_two_arm_flats_only(self):
        part = custom_parts.flyer_tube_with_flats()
        flat_rows = []
        for face in part.faces():
            if face.geom_type != GeomType.PLANE:
                continue
            center = face.center()
            normal = face.normal_at()
            box = face.bounding_box()
            if math.isclose(center.X, 5.7, abs_tol=1e-6) and math.isclose(
                normal.X, 1.0, abs_tol=1e-6
            ):
                flat_rows.append(("plus_x", center.Z - flyer_shaft_d10.LOCAL_REAR_Z_MM,
                                  box.max.Z - box.min.Z))
            if math.isclose(center.Y, -5.7, abs_tol=1e-6) and math.isclose(
                normal.Y, -1.0, abs_tol=1e-6
            ):
                flat_rows.append(("minus_y", center.Z - flyer_shaft_d10.LOCAL_REAR_Z_MM,
                                  box.max.Z - box.min.Z))
        self.assertEqual(
            sorted((axis, round(station, 6), round(length, 6))
                   for axis, station, length in flat_rows),
            [
                ("minus_y", 64.75, 5.0),
                ("plus_x", 64.75, 5.0),
            ],
        )
        self.assertAlmostEqual(6.0 - 5.7, 0.30, places=9)
        mouth_fillets = [
            face for face in part.faces()
            if face.geom_type == GeomType.TORUS
        ]
        self.assertEqual(len(mouth_fillets), 2)
        self.assertEqual(
            sorted(round(abs(face.center().Z), 6) for face in mouth_fillets),
            [39.353553, 39.353553],
        )


if __name__ == "__main__":
    unittest.main()
