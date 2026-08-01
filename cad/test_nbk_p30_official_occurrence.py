"""Tests for immutable NBK stock occurrence placement and BNW witnesses."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

from build123d import GeomType, import_step
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import nbk_p30_official_occurrence as nbk


CENTER = (37.25, -60.0, -121.75)


def _cylinders(part):
    rows = []
    for face in part.faces():
        if face.geom_type != GeomType.CYLINDER:
            continue
        adaptor = BRepAdaptor_Surface(face.wrapped)
        if adaptor.GetType() != GeomAbs_Cylinder:
            continue
        cylinder = adaptor.Cylinder()
        direction = cylinder.Axis().Direction()
        location = cylinder.Axis().Location()
        rows.append(
            {
                "radius": cylinder.Radius(),
                "direction": (direction.X(), direction.Y(), direction.Z()),
                "location": (location.X(), location.Y(), location.Z()),
            }
        )
    return rows


def _near(value: float, target: float, tolerance: float = 1e-6) -> bool:
    return abs(value - target) <= tolerance


def _main_axis_rows(part, radius: float):
    return [
        row
        for row in _cylinders(part)
        if _near(row["radius"], radius)
        and abs(row["direction"][2]) >= 0.999999
        and abs(row["direction"][0]) <= 1e-6
        and abs(row["direction"][1]) <= 1e-6
    ]


def _witness_axis(part, radius: float):
    rows = [row for row in _cylinders(part) if _near(row["radius"], radius)]
    if len(rows) != 1:
        raise AssertionError(f"expected one cylindrical witness face, got {len(rows)}")
    return rows[0]


class OfficialOccurrenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hash_before = nbk.source_sha256()
        cls.mtime_before = nbk.SOURCE_STEP.stat().st_mtime_ns
        cls.placed = nbk.place_for_m2(
            CENTER,
            stock_roll_deg=23.0,
            bnw_first_azimuth_deg=17.0,
        )
        cls.hash_after = nbk.source_sha256()
        cls.mtime_after = nbk.SOURCE_STEP.stat().st_mtime_ns

    def test_vendor_source_stays_byte_identical_and_unwritten(self):
        self.assertEqual(self.hash_before, nbk.SOURCE_STEP_SHA256)
        self.assertEqual(self.placed.source_sha256_before, nbk.SOURCE_STEP_SHA256)
        self.assertEqual(self.placed.source_sha256_after, nbk.SOURCE_STEP_SHA256)
        self.assertEqual(self.hash_after, nbk.SOURCE_STEP_SHA256)
        self.assertEqual(self.mtime_before, self.mtime_after)

        source = import_step(str(nbk.SOURCE_STEP))
        self.assertEqual(len(source.solids()), 1)
        self.assertAlmostEqual(source.volume, self.placed.stock_occurrence.volume, places=5)

    def test_local_plus_x_maps_to_machine_plus_z_at_supplied_center(self):
        bore = _main_axis_rows(self.placed.stock_occurrence, 2.5)
        self.assertGreaterEqual(len(bore), 2)
        for row in bore:
            self.assertGreater(row["direction"][2], 0.999999)
            self.assertAlmostEqual(row["location"][0], CENTER[0], places=5)
            self.assertAlmostEqual(row["location"][1], CENTER[1], places=5)
        self.assertEqual(nbk.MACHINE_SHAFT_AXIS, (0.0, 0.0, 1.0))

    def test_transformed_bore_flange_and_axial_envelope(self):
        stock = self.placed.stock_occurrence
        box = stock.bounding_box()
        self.assertEqual(len(stock.solids()), 1)
        self.assertEqual(stock.label, nbk.STOCK_LABEL)
        self.assertAlmostEqual(box.min.X, CENTER[0] - 16.0, places=5)
        self.assertAlmostEqual(box.max.X, CENTER[0] + 16.0, places=5)
        self.assertAlmostEqual(box.min.Y, CENTER[1] - 16.0, places=5)
        self.assertAlmostEqual(box.max.Y, CENTER[1] + 16.0, places=5)
        self.assertAlmostEqual(box.min.Z, CENTER[2] - 5.5, places=5)
        self.assertAlmostEqual(box.max.Z, CENTER[2] + 13.0, places=5)
        self.assertAlmostEqual(box.size.Z, 18.5, places=5)
        self.assertTrue(_main_axis_rows(stock, 2.5))
        self.assertTrue(_main_axis_rows(stock, 16.0))

    def test_bnw_holes_and_screws_are_separate_orthogonal_witnesses(self):
        placed = self.placed
        self.assertEqual(len(placed.bnw_hole_witnesses), 2)
        self.assertEqual(len(placed.bnw_set_screw_witnesses), 2)
        self.assertTrue(all("hole_path_witness" in part.label for part in placed.bnw_hole_witnesses))
        self.assertTrue(all("set_screw_envelope_witness" in part.label for part in placed.bnw_set_screw_witnesses))

        hole_axes = [
            _witness_axis(part, nbk.BNW_WITNESS_HOLE_DIAMETER_MM / 2.0)
            for part in placed.bnw_hole_witnesses
        ]
        screw_axes = [
            _witness_axis(part, nbk.BNW_WITNESS_SCREW_DIAMETER_MM / 2.0)
            for part in placed.bnw_set_screw_witnesses
        ]
        for row in (*hole_axes, *screw_axes):
            self.assertAlmostEqual(row["direction"][2], 0.0, places=6)

        dot = sum(
            hole_axes[0]["direction"][index] * hole_axes[1]["direction"][index]
            for index in range(3)
        )
        self.assertAlmostEqual(dot, 0.0, places=6)
        for hole, screw in zip(hole_axes, screw_axes):
            coaxial = abs(sum(
                hole["direction"][index] * screw["direction"][index]
                for index in range(3)
            ))
            self.assertAlmostEqual(coaxial, 1.0, places=6)

        witness_z = CENTER[2] + nbk.BNW_WITNESS_DEFAULT_LOCAL_X_MM
        for part in (*placed.bnw_hole_witnesses, *placed.bnw_set_screw_witnesses):
            box = part.bounding_box()
            self.assertAlmostEqual((box.min.Z + box.max.Z) / 2.0, witness_z, places=5)

        # Positive witness children coexist with the untouched stock solid;
        # they were not boolean-subtracted from it.
        assembly = placed.review_assembly()
        self.assertEqual(assembly.label, nbk.REVIEW_ASSEMBLY_LABEL)
        self.assertEqual(len(assembly.children), 5)
        self.assertEqual(len(assembly.solids()), 5)
        self.assertEqual(len(placed.parts_by_role()), 5)

    def test_official_mass_and_axial_inertia_are_exposed_without_recalculation(self):
        properties = self.placed.official_mass_properties
        self.assertEqual(properties.mass_g, 28.0)
        self.assertEqual(properties.mass_kg, 0.028)
        self.assertEqual(properties.axial_moment_of_inertia_kg_m2, 3.0e-6)
        self.assertIn("NBK", properties.authority)
        self.assertEqual(nbk.OFFICIAL_MASS_KG, 0.028)
        self.assertEqual(nbk.OFFICIAL_AXIAL_MOMENT_OF_INERTIA_KG_M2, 3.0e-6)

    def test_each_M3x12_witness_can_thread_inward_without_changing_length(self):
        placed = nbk.place_for_m2(
            CENTER,
            bnw_first_azimuth_deg=0.0,
            bnw_screw_inward_adjustments_mm=(0.0, 0.5),
        )
        self.assertEqual(
            placed.bnw_screw_inward_adjustments_mm,
            (0.0, 0.5),
        )
        round_box = placed.bnw_set_screw_witnesses[0].bounding_box()
        flat_box = placed.bnw_set_screw_witnesses[1].bounding_box()
        self.assertAlmostEqual(round_box.min.X, CENTER[0] + 2.5, places=6)
        self.assertAlmostEqual(round_box.max.X, CENTER[0] + 14.5, places=6)
        self.assertAlmostEqual(flat_box.min.Y, CENTER[1] + 2.0, places=6)
        self.assertAlmostEqual(flat_box.max.Y, CENTER[1] + 14.0, places=6)
        self.assertAlmostEqual(round_box.size.X, 12.0, places=6)
        self.assertAlmostEqual(flat_box.size.Y, 12.0, places=6)
        self.assertEqual(nbk.source_sha256(), nbk.SOURCE_STEP_SHA256)

    def test_invalid_placement_inputs_fail_closed(self):
        with self.assertRaises(ValueError):
            nbk.place_for_m2((0.0, 0.0))
        with self.assertRaises(ValueError):
            nbk.place_for_m2((0.0, 0.0, math.nan))
        with self.assertRaises(ValueError):
            nbk.place_for_m2((0.0, 0.0, 0.0), bnw_local_x_mm=30.0)
        with self.assertRaises(ValueError):
            nbk.place_for_m2(
                (0.0, 0.0, 0.0),
                bnw_screw_inward_adjustments_mm=(0.0,),
            )
        with self.assertRaises(ValueError):
            nbk.place_for_m2(
                (0.0, 0.0, 0.0),
                bnw_screw_inward_adjustments_mm=(0.0, -0.1),
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
