"""Regression checks for the official NBK P30-3GT-BLP-6C-5 CAD artifact."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import unittest

from build123d import GeomType, import_step
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder


HERE = Path(__file__).resolve().parent
UPGRADES = HERE / "models" / "upgrades"
STEP = UPGRADES / "NBK_P30-3GT-BLP-6C-5_AP214.step"
DRAWING = UPGRADES / "NBK_3GT-BLP-6C_official_drawing.pdf"
MANIFEST = UPGRADES / "NBK_P30-3GT-BLP-6C-5_CADENAS_download.xml"
TERMS = UPGRADES / "NBK_PARTcommunity_terms.txt"
SOURCE = UPGRADES / "NBK_P30-3GT-BLP-6C-5.source.json"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _cylinders(part):
    rows = []
    for face in part.faces():
        if face.geom_type != GeomType.CYLINDER:
            continue
        adaptor = BRepAdaptor_Surface(face.wrapped)
        if adaptor.GetType() != GeomAbs_Cylinder:
            continue
        cylinder = adaptor.Cylinder()
        axis = cylinder.Axis()
        direction = axis.Direction()
        location = axis.Location()
        rows.append(
            {
                "radius": cylinder.Radius(),
                "direction": (direction.X(), direction.Y(), direction.Z()),
                "location": (location.X(), location.Y(), location.Z()),
                "bbox": face.bounding_box(),
            }
        )
    return rows


def _near(value: float, target: float, tolerance: float = 1e-5) -> bool:
    return abs(value - target) <= tolerance


def _axial(rows, radius: float):
    return [
        row
        for row in rows
        if _near(row["radius"], radius)
        and abs(row["direction"][0]) >= 0.999999
        and abs(row["direction"][1]) <= 1e-6
        and abs(row["direction"][2]) <= 1e-6
    ]


class NBKP30OfficialCadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = json.loads(SOURCE.read_text(encoding="utf-8"))
        cls.part = import_step(str(STEP))
        cls.box = cls.part.bounding_box()
        cls.cylinders = _cylinders(cls.part)

    def test_provenance_artifacts_are_pinned(self):
        self.assertEqual(
            _sha256(STEP),
            "996449b7d9ec7703e7b38c6f75eff00a1174e3e1f088c05f0f1460b205169df9",
        )
        self.assertEqual(
            _sha256(DRAWING),
            "a6559b594f927fd3e7e4878ec341f6877f628e14cd9fc46a79cac7dc8e1dde87",
        )
        self.assertEqual(
            _sha256(MANIFEST),
            "dc08ed45bdb3833831004822d7db7902fdde36972c01924e7f53e66bb9c1db2d",
        )
        self.assertEqual(
            _sha256(TERMS),
            "91e9e661965585ded6ef16c986873e79516256ff059e7d8904f999941e701f46",
        )

        header = STEP.read_text(encoding="utf-8", errors="strict")[:1200]
        self.assertIn("'STEP AP214'", header)
        self.assertIn("'P30-3GT-BLP-6C-5'", header)
        self.assertIn("'CADENAS'", header)
        self.assertIn("'PARTsolutions'", header)
        self.assertIn("'License CC BY-ND 4.0'", header)

        manifest = MANIFEST.read_text(encoding="utf-8")
        self.assertIn("<NB>P30-3GT-BLP-6C-5</NB>", manifest)
        self.assertIn("<CADFORMAT>STEP</CADFORMAT>", manifest)
        self.assertIn("<CADVERSION>214</CADVERSION>", manifest)
        self.assertIn("<FILENAME>P30-3GT-BLP-6C-5.stp</FILENAME>", manifest)

    def test_topology_and_primary_axis(self):
        self.assertEqual(len(self.part.solids()), 1)
        self.assertEqual(len(self.part.faces()), 49)
        self.assertAlmostEqual(self.part.volume, 8844.456907625592, places=5)

        bore = _axial(self.cylinders, 2.5)
        self.assertGreaterEqual(len(bore), 3)
        for row in bore:
            self.assertAlmostEqual(row["location"][1], 0.0, places=6)
            self.assertAlmostEqual(row["location"][2], 0.0, places=6)

        for radius in (10.0, 11.5, 13.95, 16.0):
            self.assertTrue(_axial(self.cylinders, radius), radius)

        self.assertEqual(self.source["measured_step"]["shaft_axis_local"], "+X")
        self.assertEqual(self.source["measured_step"]["shaft_axis_center_yz_mm"], [0.0, 0.0])

    def test_official_diameters_and_envelope(self):
        self.assertAlmostEqual(self.box.min.X, -5.5, places=5)
        self.assertAlmostEqual(self.box.max.X, 13.0, places=5)
        self.assertAlmostEqual(self.box.size.X, 18.5, places=5)
        self.assertAlmostEqual(self.box.size.Y, 32.0, places=5)
        self.assertAlmostEqual(self.box.size.Z, 32.0, places=5)

        expected_diameters = {
            2.5: 5.0,
            10.0: 20.0,
            11.5: 23.0,
            13.95: 27.9,
            16.0: 32.0,
        }
        for radius, diameter in expected_diameters.items():
            self.assertTrue(_axial(self.cylinders, radius), diameter)
            self.assertAlmostEqual(2.0 * radius, diameter, places=6)

    def test_official_axial_dimensions(self):
        tooth_faces = _axial(self.cylinders, 13.95)
        self.assertEqual(len(tooth_faces), 1)
        self.assertAlmostEqual(tooth_faces[0]["bbox"].size.X, 7.3, places=5)

        flange_faces = _axial(self.cylinders, 16.0)
        self.assertEqual(len(flange_faces), 2)
        self.assertAlmostEqual(
            sum(row["bbox"].size.X for row in flange_faces), 1.85, places=5
        )
        self.assertAlmostEqual(self.box.max.X - 5.5, 7.5, places=5)
        self.assertAlmostEqual(5.5 - self.box.min.X, 11.0, places=5)

        clamp_radial_faces = [
            row
            for row in self.cylinders
            if row["radius"] >= 1.0
            and abs(row["direction"][2]) >= 0.999999
            and _near(row["location"][0], 10.25)
        ]
        self.assertTrue(clamp_radial_faces)
        self.assertAlmostEqual(self.box.max.X - 10.25, 2.75, places=5)

    def test_mass_and_configuration_authority_are_explicit(self):
        table = self.source["official_table"]
        self.assertEqual(table["declared_mass_g"], 28.0)
        self.assertEqual(table["moment_of_inertia_kg_m2"], 0.000003)
        self.assertEqual(table["body_material"], "A2017 aluminum alloy")
        self.assertEqual(table["clamp_bolt_material"], "SCM435, black oxide")
        self.assertEqual(self.source["mass_authority"]["authority"], "NBK product table")
        self.assertEqual(
            self.source["mass_authority"]["step_mass_recalculation"],
            "not authoritative",
        )

        configuration = self.source["configuration"]
        self.assertIn("split-clamp", configuration["stock_attachment"])
        self.assertIsNone(configuration["additional_machining"])
        self.assertFalse(configuration["bnw_two_set_screw_machining_present"])

        license_handling = self.source["license_handling"]
        self.assertEqual(license_handling["license_from_step_header"], "CC BY-ND 4.0")
        self.assertTrue(license_handling["vendor_step_must_remain_byte_for_byte_unmodified"])
        self.assertIn("separate assembly occurrences", license_handling["bnw_representation"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
