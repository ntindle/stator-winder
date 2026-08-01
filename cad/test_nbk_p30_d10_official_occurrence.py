"""Tests for the immutable official stock D10 flyer occurrence."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

from build123d import GeomType

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import nbk_p30_d10_official_occurrence as d10


class OfficialD10Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hash_before = d10.source_sha256()
        cls.mtime_before = d10.SOURCE_STEP.stat().st_mtime_ns
        cls.source = d10.import_official()
        cls.placed = d10.place_hub_rear((0.0, 0.0, -97.75), stock_roll_deg=45.0)

    def test_source_is_pinned_and_immutable(self):
        self.assertEqual(self.hash_before, d10.SOURCE_STEP_SHA256)
        self.assertEqual(d10.SOURCE_STEP.stat().st_size, 57130)
        self.assertEqual(d10.source_sha256(), self.hash_before)
        self.assertEqual(d10.SOURCE_STEP.stat().st_mtime_ns, self.mtime_before)

    def test_exact_source_geometry(self):
        self.assertEqual(len(self.source.solids()), 1)
        self.assertTrue(self.source.is_valid)
        self.assertEqual(len(self.source.faces()), 49)
        self.assertAlmostEqual(self.source.volume, 7834.785240560267, places=5)
        box = self.source.bounding_box()
        self.assertAlmostEqual(box.min.X, -5.5, places=5)
        self.assertAlmostEqual(box.max.X, 13.0, places=5)
        bore_faces = [
            face for face in self.source.faces().filter_by(GeomType.CYLINDER)
            if abs(face.bounding_box().size.X) > 5.0
        ]
        self.assertTrue(bore_faces)

    def test_hub_rear_placement_and_mass_authority(self):
        stock = self.placed.stock_occurrence
        box = stock.bounding_box()
        self.assertAlmostEqual(box.min.Z, -110.75, places=5)
        self.assertAlmostEqual(box.max.Z, -92.25, places=5)
        self.assertEqual(len(stock.solids()), 1)
        props = self.placed.official_mass_properties
        self.assertEqual(props.mass_g, 28.0)
        self.assertEqual(props.axial_moment_of_inertia_kg_m2, 3.0e-6)
        self.assertEqual(self.placed.source_sha256_before, d10.SOURCE_STEP_SHA256)
        self.assertEqual(self.placed.source_sha256_after, d10.SOURCE_STEP_SHA256)


if __name__ == "__main__":
    unittest.main(verbosity=2)
