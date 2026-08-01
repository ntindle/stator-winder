"""Deterministic checks for the Nomex 410 stator-insulation cut set."""

from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

import ezdxf
from shapely.geometry import Polygon

import stator_insulation_nomex410 as insulation


class Nomex410MaterialTests(unittest.TestCase):
    def test_exact_selected_bae_stock(self):
        self.assertEqual(insulation.MATERIAL_PART_NUMBER,
                         "Nomex Type 410, 5 mil")
        self.assertEqual(insulation.MATERIAL_SUPPLIER_SKU,
                         "INNMX410005S")
        self.assertAlmostEqual(insulation.MATERIAL_NOMINAL_THICKNESS_MM,
                               0.127, places=12)
        self.assertAlmostEqual(insulation.MATERIAL_RECEIVING_MIN_MM,
                               0.120, places=12)
        self.assertAlmostEqual(insulation.MATERIAL_RECEIVING_MAX_MM,
                               0.140, places=12)

    def test_active_job_uses_receiving_max_and_alternate_fails_closed(self):
        report = insulation.build_report()
        self.assertEqual(report["fabrication_package_status"], "PASS")
        self.assertEqual(report["active_winding_job_compatibility"],
                         "PASS")
        self.assertLessEqual(insulation.MATERIAL_RECEIVING_MAX_MM,
                             insulation.ACTIVE_WINDING_PLAN_LINER_MAX_MM)
        self.assertAlmostEqual(
            report["integration_contract"][
                "accepted_wire_finished_diameter_max_mm"
            ],
            0.235,
            places=12,
        )
        self.assertGreater(insulation.DMD180_ALTERNATE_THICKNESS_MM,
                           insulation.ACTIVE_WINDING_PLAN_LINER_MAX_MM)
        self.assertEqual(
            report["incompatible_alternate"]["status"],
            "REJECT_AS_DROP_IN_SUBSTITUTE",
        )


class SlotCellGeometryTests(unittest.TestCase):
    def setUp(self):
        self.cell = insulation.developed_slot_cell()

    def test_blank_has_exact_stack_plus_two_flares(self):
        self.assertAlmostEqual(
            self.cell.blank_length_mm,
            self.cell.stack_mm + 2.0 * self.cell.axial_end_flare_mm,
            places=12,
        )
        self.assertAlmostEqual(self.cell.blank_length_mm, 18.0, places=12)

    def test_development_is_symmetric_and_root_is_central(self):
        self.assertAlmostEqual(self.cell.blank_width_mm,
                               2.0 * self.cell.root_fold_x_mm, places=12)
        stations = self.cell.fold_stations_x_mm
        for left, right in zip(stations, reversed(stations)):
            self.assertAlmostEqual(left + right,
                                   self.cell.blank_width_mm, places=12)

    def test_outline_is_one_valid_relief_profile(self):
        polygon = Polygon(insulation.slot_cell_outline())
        self.assertTrue(polygon.is_valid)
        self.assertGreater(polygon.area, 0.0)
        min_x, min_y, max_x, max_y = polygon.bounds
        self.assertAlmostEqual(min_x, 0.0, places=12)
        self.assertAlmostEqual(min_y, 0.0, places=12)
        self.assertAlmostEqual(max_x, self.cell.blank_width_mm, places=12)
        self.assertAlmostEqual(max_y, self.cell.blank_length_mm, places=12)


class EndCapGeometryTests(unittest.TestCase):
    def test_cap_is_one_polygon_with_one_central_cutout(self):
        cap = insulation.end_cap_geometry()
        self.assertIsInstance(cap, Polygon)
        self.assertTrue(cap.is_valid)
        self.assertEqual(len(cap.interiors), 1)
        min_x, min_y, max_x, max_y = cap.bounds
        self.assertAlmostEqual(max_x - min_x, 46.0, places=3)
        self.assertAlmostEqual(max_y - min_y, 46.0, places=3)

    def test_front_and_rear_are_identical_non_handed_cut_geometry(self):
        front = insulation.end_cap_dxf("front")
        rear = insulation.end_cap_dxf("rear")
        front_cut = [
            tuple((round(point[0], 9), round(point[1], 9))
                  for point in entity.get_points("xy"))
            for entity in front.modelspace().query(
                'LWPOLYLINE[layer=="CUT"]')
        ]
        rear_cut = [
            tuple((round(point[0], 9), round(point[1], 9))
                  for point in entity.get_points("xy"))
            for entity in rear.modelspace().query(
                'LWPOLYLINE[layer=="CUT"]')
        ]
        self.assertEqual(front_cut, rear_cut)

    def test_cap_overlap_leaves_positive_single_wire_opening(self):
        summary = insulation.geometry_summary()["clearance_indicators"]
        self.assertGreater(
            summary["single_active_wire_static_margin_at_cap_mouth_mm"],
            0.0,
        )
        self.assertAlmostEqual(
            summary["end_cap_mouth_after_edge_overlap_mm"],
            summary["bare_slot_mouth_mm"]
            - 2.0 * insulation.CAP_EDGE_OVERLAP_MM,
            places=12,
        )

    def test_one_selected_sheet_has_a_constructive_simple_nest(self):
        sheet = insulation.geometry_summary()["sheet_yield"]
        self.assertTrue(sheet["fits_selected_sheet"])
        self.assertLessEqual(sheet["layout_bounds_mm"][0],
                             sheet["sheet_size_mm"][0])
        self.assertLessEqual(sheet["layout_bounds_mm"][1],
                             sheet["sheet_size_mm"][1])


class DxfAndReportTests(unittest.TestCase):
    def test_slot_dxf_layers_and_entities(self):
        document = insulation.slot_cell_dxf()
        modelspace = document.modelspace()
        cut = list(modelspace.query('LWPOLYLINE[layer=="CUT"]'))
        folds = list(modelspace.query('LINE[layer=="FOLD_REFERENCE"]'))
        datums = list(modelspace.query('LINE[layer=="DATUM_REFERENCE"]'))
        self.assertEqual(len(cut), 1)
        self.assertTrue(cut[0].closed)
        self.assertEqual(len(folds), 5)
        self.assertEqual(len(datums), 2)
        self.assertEqual(document.units, ezdxf.units.MM)

    def test_cap_dxf_has_two_closed_cut_contours(self):
        document = insulation.end_cap_dxf("front")
        cut = list(document.modelspace().query(
            'LWPOLYLINE[layer=="CUT"]'))
        self.assertEqual(len(cut), 2)
        self.assertTrue(all(entity.closed for entity in cut))
        self.assertEqual(document.units, ezdxf.units.MM)

    def test_report_and_pdf_write_to_explicit_targets(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pdf = root / "package.pdf"
            json_path = root / "report.json"
            markdown_path = root / "report.md"
            insulation.write_pdf(pdf)
            report = insulation.write_reports(json_path, markdown_path)
            self.assertGreater(pdf.stat().st_size, 10_000)
            self.assertEqual(report["active_winding_job_compatibility"],
                             "PASS")
            markdown = markdown_path.read_text(encoding="utf-8")
            self.assertIn("Integration contract", markdown)
            self.assertIn("Incompatible alternate", markdown)


if __name__ == "__main__":
    unittest.main()
