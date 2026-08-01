"""Deterministic tests for the isolated flat carriage candidate."""

from __future__ import annotations

from pathlib import Path
import math
import sys
import unittest

import ezdxf
from ezdxf import bbox as dxf_bbox
from ezdxf import units

sys.path.insert(0, str(Path(__file__).resolve().parent))
import carriage_endstop_flag as flag  # noqa: E402
import fabricated_carriage as plate  # noqa: E402


class FabricatedCarriageStepTests(unittest.TestCase):
    def test_plate_is_one_valid_solid_at_exact_stock_thickness(self):
        part = plate.gen_step()
        bbox = part.bounding_box()
        self.assertTrue(part.is_valid)
        self.assertEqual(len(part.solids()), 1)
        self.assertAlmostEqual(bbox.min.X, -88.0, places=6)
        self.assertAlmostEqual(bbox.max.X, 60.0, places=6)
        self.assertAlmostEqual(bbox.min.Y, plate.PLATE_BOTTOM_Y, places=6)
        self.assertAlmostEqual(bbox.max.Y, plate.PLATE_TOP_Y, places=6)
        self.assertAlmostEqual(bbox.size.Y, 6.35, places=6)
        self.assertAlmostEqual(bbox.min.Z, plate.PLATE_Z_MIN, places=6)
        self.assertAlmostEqual(bbox.max.Z, 140.0, places=6)

    def test_inherited_hole_centers_and_counts(self):
        self.assertEqual(len(plate.mgn12h_holes()), 8)
        self.assertEqual(len(plate.tower_holes()), 4)
        self.assertEqual(len(plate.nut_bracket_holes()), 2)
        self.assertEqual(len(plate.mounting_holes()), 14)
        self.assertEqual(
            {(hole.x, hole.z) for hole in plate.tower_holes()},
            {(-31.0, 64.0), (-31.0, 126.0),
             (31.0, 64.0), (31.0, 126.0)},
        )
        self.assertEqual(
            {(hole.x, hole.z) for hole in plate.nut_bracket_holes()},
            {(-78.0, 97.0), (-78.0, 107.0)},
        )

    def test_notch_preserves_rear_tower_hole_as_closed_cut(self):
        rear_left = next(
            hole for hole in plate.tower_holes()
            if hole.x == -31.0 and hole.z == 126.0
        )
        ligament = rear_left.x - rear_left.diameter / 2.0 - plate.NOTCH_RIGHT_X
        self.assertAlmostEqual(ligament, 2.75, places=6)
        self.assertGreater(ligament, 0.0)

    def test_plate_volume_matches_planar_cut_area(self):
        outer_area = (
            (plate.PLATE_X_MAX - plate.PLATE_X_MIN)
            * (plate.NOTCH_Z_MIN - plate.PLATE_Z_MIN)
            + (plate.PLATE_X_MAX - plate.NOTCH_RIGHT_X)
            * (plate.PLATE_Z_MAX - plate.NOTCH_Z_MIN)
        )
        rectangular_cuts = (
            (plate.MOTOR_WINDOW[1] - plate.MOTOR_WINDOW[0])
            * (plate.MOTOR_WINDOW[3] - plate.MOTOR_WINDOW[2])
            + (plate.T8_RELIEF[1] - plate.T8_RELIEF[0])
            * (plate.T8_RELIEF[3] - plate.T8_RELIEF[2])
        )
        round_cuts = sum(
            math.pi * (hole.diameter / 2.0) ** 2
            for hole in plate.mounting_holes()
        )
        expected = (outer_area - rectangular_cuts - round_cuts) * 6.35
        self.assertAlmostEqual(plate.gen_step().volume, expected, places=3)


class FabricatedCarriageDxfTests(unittest.TestCase):
    def test_dxf_units_entities_closed_contours_and_extents(self):
        document = plate.gen_dxf()
        modelspace = document.modelspace()
        self.assertEqual(document.units, units.MM)
        self.assertEqual(len(modelspace.query("LWPOLYLINE")), 2)
        self.assertEqual(len(modelspace.query("CIRCLE")), 14)
        self.assertTrue(all(entity.closed for entity in
                            modelspace.query("LWPOLYLINE")))
        self.assertTrue(all(entity.dxf.layer == "CUT" for entity in modelspace))

        extents = dxf_bbox.extents(modelspace)
        self.assertAlmostEqual(extents.extmin.x, -88.0, places=6)
        self.assertAlmostEqual(extents.extmax.x, 60.0, places=6)
        self.assertAlmostEqual(extents.extmin.y, plate.PLATE_Z_MIN, places=6)
        self.assertAlmostEqual(extents.extmax.y, 140.0, places=6)

        outer = modelspace.query("LWPOLYLINE").first
        vertices = {(round(x, 6), round(y, 6))
                    for x, y in outer.get_points("xy")}
        self.assertTrue({
            (plate.T8_RELIEF[0], plate.PLATE_Z_MIN),
            (plate.T8_RELIEF[0], plate.T8_RELIEF[3]),
            (plate.T8_RELIEF[1], plate.T8_RELIEF[3]),
            (plate.T8_RELIEF[1], plate.PLATE_Z_MIN),
        }.issubset(vertices))

    def test_generated_document_round_trips(self):
        output = Path(__file__).with_name("_test_fabricated_carriage.dxf")
        try:
            plate.gen_dxf().saveas(output)
            loaded = ezdxf.readfile(output)
            self.assertEqual(loaded.units, units.MM)
            self.assertEqual(len(loaded.modelspace().query("CIRCLE")), 14)
        finally:
            output.unlink(missing_ok=True)


class EndstopFlagTests(unittest.TestCase):
    def test_flag_is_one_valid_solid_and_restores_trigger_envelope(self):
        part = flag.gen_step()
        bbox = part.bounding_box()
        self.assertTrue(part.is_valid)
        self.assertEqual(len(part.solids()), 1)
        self.assertAlmostEqual(bbox.min.X, -36.0, places=6)
        self.assertAlmostEqual(bbox.max.X, 36.0, places=6)
        self.assertAlmostEqual(bbox.size.Y, 6.0, places=6)
        self.assertAlmostEqual(bbox.min.Z, 121.0, places=6)
        self.assertAlmostEqual(bbox.max.Z, 142.0, places=6)
        self.assertEqual(flag.TRIGGER_X, (-8.0, 8.0))
        self.assertEqual(flag.TRIGGER_Z, (134.0, 142.0))

    def test_flag_reuses_rear_tower_holes_and_has_exact_fasteners(self):
        self.assertEqual(flag.ATTACHMENT_HOLES_XZ,
                         ((-31.0, 126.0), (31.0, 126.0)))
        rear_tower = {
            (hole.x, hole.z) for hole in plate.tower_holes()
            if hole.z == 126.0
        }
        self.assertEqual(set(flag.ATTACHMENT_HOLES_XZ), rear_tower)
        self.assertEqual(flag.FASTENER_RECOMMENDATION["quantity"], 2)
        self.assertIn("M4x25", flag.FASTENER_RECOMMENDATION["screw"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
