"""Regression checks for the human BOM's release-critical quantities."""

from __future__ import annotations

import csv
from pathlib import Path
import unittest


BOM = Path(__file__).resolve().parent.parent / "bom.csv"


class BomReconciliationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with BOM.open(newline="", encoding="utf-8-sig") as stream:
            cls.rows = list(csv.DictReader(stream))
        cls.items = {
            row["item"]: row for row in cls.rows if (row.get("item") or "").strip()
        }

    def test_csv_shape_and_declared_total(self):
        self.assertTrue(all(None not in row for row in self.rows))
        estimated = sum(
            float(row["ext_usd"])
            for row in self.items.values()
            if (row.get("ext_usd") or "").strip()
        )
        total = next(
            float(row["ext_usd"])
            for row in self.rows
            if (row.get("unit_usd") or "").strip() == "TOTAL:"
        )
        self.assertAlmostEqual(estimated, total, places=2)
        self.assertAlmostEqual(total, 1573.78, places=2)

    def test_current_frame_and_consumable_quantities(self):
        self.assertEqual(self.items["MISUMI frame brackets"]["qty"], "15")
        self.assertEqual(self.items["35 mm machine-foot stack"]["qty"], "4")
        self.assertIn("970180581", self.items["35 mm machine-foot stack"]["spec"])
        self.assertIn("4.8 +/-0.1", self.items["35 mm machine-foot stack"]["cad_model_source"])
        self.assertEqual(self.items["3GT closed belt 210-3GT-6"]["qty"], "2")
        self.assertIn("1 installed + 1 spare",
                      self.items["3GT closed belt 210-3GT-6"]["spec"])
        self.assertEqual(
            self.items["3GT closed belt 210-3GT-6"]["unit_usd"], "11.14"
        )
        self.assertEqual(self.items["Felt washers Ø20"]["qty"], "6")
        self.assertIn("2775 mm total",
                      self.items["2020 T-slot extrusion"]["spec"])
        counterweight = self.items["Counterweight hardware"]
        self.assertIn("92125A126", counterweight["spec"])
        self.assertIn("94459A130", counterweight["spec"])
        self.assertIn("M2x8", counterweight["spec"])
        self.assertEqual(counterweight["unit_usd"], "")
        self.assertEqual(self.items["Fixed ceramic eyelet"]["qty"], "2")
        self.assertIn("1 installed + 1 spare",
                      self.items["Fixed ceramic eyelet"]["spec"])

        carriage = self.items["0.250 inch MIC6 carriage plate"]
        self.assertEqual(carriage["qty"], "1")
        self.assertEqual(carriage["unit_usd"], "")
        self.assertEqual(carriage["ext_usd"], "")
        self.assertIn("ALUMIC6-250", carriage["spec"])
        self.assertIn("UPLOAD_READY_UNPRICED", carriage["sourcing"])
        self.assertIn("cad/fabricated_carriage.dxf", carriage["cad_model_source"])

        total_row = next(
            row for row in self.rows
            if (row.get("unit_usd") or "").strip() == "TOTAL:"
        )
        self.assertIn(
            "unquoted SendCutSend MIC6 carriage plate",
            total_row["sourcing"],
        )

    def test_36v_condition_is_required_but_no_psu_is_order_released(self):
        condition = self.items["Required regulated 36 V M2 supply condition"]
        self.assertEqual(
            condition["category"],
            "electronics_required_condition",
        )
        self.assertEqual(condition["qty"], "1")
        self.assertEqual(condition["unit_usd"], "199.00")
        self.assertEqual(condition["ext_usd"], "199.00")
        self.assertIn("Leadshine LSP-360-36", condition["spec"])
        self.assertIn("10 A continuous", condition["spec"])
        self.assertIn("18 A peak", condition["spec"])
        self.assertIn("CONDITIONAL CART CANDIDATE ONLY", condition["sourcing"])
        self.assertIn("no PSU is order-authorized", condition["sourcing"])
        self.assertIn("24 V must not be used", condition["cad_model_source"])
        self.assertEqual(
            self.items["Optional controller integration"]["category"],
            "electronics_optional",
        )

    def test_exact_m2_release_delta_is_explicit_and_fail_closed(self):
        motor = self.items["NEMA17 motor/encoder (M2)"]
        self.assertIn("Leadshine CS-M21708", motor["spec"])
        self.assertEqual(motor["unit_usd"], "199.00")
        self.assertIn("CS-M21708.STEP", motor["cad_model_source"])

        driver = self.items[
            "Conditional closed-loop stepper driver candidate (M2)"
        ]
        self.assertIn("Leadshine CS-D508", driver["spec"])
        self.assertIn("3.5 versus 3.6 A peak", driver["spec"])
        self.assertIn("CONDITIONAL ONLY", driver["sourcing"])
        self.assertIn("0faaf40e", driver["cad_model_source"])
        self.assertNotIn("CL42T", driver["spec"])
        self.assertIn("not valid substitutes", driver["cad_model_source"])
        self.assertNotIn("CS1-D503S", driver["spec"])

        pulley = self.items["30T 3GT motor pulley with conditional BNW machining"]
        self.assertIn("P30-3GT-BLP-6C-5", pulley["spec"])
        self.assertIn("CONDITIONAL RFQ", pulley["sourcing"])
        self.assertIn("M2-P30-D5-BNW-RFQ-A", pulley["sourcing"])
        self.assertIn("planning allowance", pulley["sourcing"])
        self.assertIn("996449b7", pulley["cad_model_source"])
        self.assertIn("0.471456 N m", pulley["cad_model_source"])

    def test_stale_substitution_language_is_absent(self):
        text = "\n".join(
            " ".join(str(value or "") for value in row.values())
            for row in self.rows
        )
        for stale in (
            "LRS-240-24", "M8 x 15", "(~2.5 m)", "any 42.3mm",
            "McMaster 6627T421", "P40-2GT-BLP-6C-5", "200-2GT-6RF",
            "PTFE elbow guide", "Optional 24 V PSU integration",
        ):
            self.assertNotIn(stale, text)
        self.assertIn("Remington Industries 32SNSP.125", text)
        self.assertIn("INNMX410005S", text)
        self.assertIn("0.22352 mm supplier nominal finished OD", text)
        self.assertIn("measured job input 0.220-0.235 mm", text)
        self.assertIn("0.127 mm supplier nominal", text)
        self.assertIn("measured installed job input 0.120-0.140 mm", text)
        self.assertNotIn("model 0.240 mm", text)
        self.assertNotIn("reject above 0.240 mm", text)
        self.assertGreaterEqual(text.count("regenerate every job artifact"), 2)
        self.assertIn("1-CL42T-S05-V41", text)
        self.assertIn("17HS19-2004D-E1K.step", text)
        self.assertIn("P30-3GT-BLP-6C-5", text)
        self.assertIn("210-3GT-6", text)
        self.assertNotIn("discontinued 17HS19-2004D-E1000", text)

    def test_release_procurement_lines_are_explicit(self):
        self.assertIn(
            "DigiKey Z4701-ND",
            self.items["Omron D2F-01L2-D3 home microswitch"]["sourcing"],
        )
        self.assertIn(
            "exact active stocked order line",
            self.items["Omron D2F-01L2-D3 home microswitch"]["sourcing"],
        )
        self.assertIn(
            "Farnell 2884560",
            self.items["35 mm machine-foot stack"]["sourcing"],
        )
        self.assertIn(
            "McMaster 8341K31",
            self.items["Felt washers Ø20"]["sourcing"],
        )
        self.assertIn(
            "EIS LOC21425",
            self.items["Fixed-eyelet and shaft-wrap-sleeve adhesive"]["sourcing"],
        )
        adhesive = self.items["Fixed-eyelet and shaft-wrap-sleeve adhesive"]
        self.assertIn("fixed ceramic eyelets", adhesive["cad_model_source"])
        self.assertIn("shaft-wrap sleeve", adhesive["cad_model_source"])
        self.assertNotIn("torus", " ".join(adhesive.values()).lower())
        self.assertIn(
            "Micro Center SKU 151506",
            self.items["PETG filament"]["sourcing"],
        )

    def test_frozen_successor_custom_parts_are_explicit_and_blocked(self):
        expected = {
            "One-piece PEEK flyer guide and exit bell": "1",
            "Short-leadin PEEK stator caps": "2",
            "Active-sector PEEK terminal guides": "2",
            "Active-sector aluminum yoke": "1",
            "Six serialized ASTM-B777 balance trims": "6",
        }
        for item, quantity in expected.items():
            with self.subTest(item=item):
                row = self.items[item]
                self.assertEqual(row["qty"], quantity)
                self.assertEqual(row["unit_usd"], "")
                self.assertEqual(row["ext_usd"], "")
                self.assertIn("RFQ_READY_FOR_QUOTE_ONLY", row["sourcing"])
                self.assertIn("PRODUCTION BLOCKED", row["sourcing"])
                self.assertIn("BLOCKED", row["sourcing"])
        self.assertIn(
            "default-job rigid geometry and the complete raw rigid sweep pass",
            self.items["Active-sector aluminum yoke"]["sourcing"],
        )
        self.assertIn(
            "rev6-front-plane-outboard-coil-bypass-yoke",
            self.items["Active-sector aluminum yoke"]["cad_model_source"],
        )
        pulley = self.items["Stock NBK 30T 3GT flyer pulley"]
        self.assertEqual(pulley["qty"], "1")
        self.assertIn("P30-3GT-BLP-6C-10", pulley["spec"])
        self.assertIn("CART_READY_REFERENCE_ONLY", pulley["sourcing"])
        self.assertIn("JPY 5060", pulley["sourcing"])
        self.assertIn("BLOCKED", pulley["sourcing"])
        balance = self.items["Six serialized ASTM-B777 balance trims"]
        self.assertIn("18.49 g/cm3", balance["spec"])
        self.assertIn("5995N71", balance["sourcing"])
        self.assertIn("new balance solve", balance["cad_model_source"])
        self.assertEqual(self.items["PETG filament"]["qty"], "1")
        self.assertIn("21 printed parts", self.items["PETG filament"]["spec"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
