"""Regression tests for one-row-per-obligation order generation."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import procurement


class FullOrderFulfillmentTests(unittest.TestCase):
    def test_canonical_bearing_and_carriage_order_obligations(self):
        result = procurement.audit()
        rows = {
            row["id"]: row
            for row in procurement._full_order_rows(result["release_catalog"])
        }

        bearing = rows["bearing-608zz"]
        self.assertEqual(bearing["quantity"]["required_qty"], 2)
        self.assertEqual(bearing["quantity"]["spare_qty"], 1)
        self.assertEqual(bearing["quantity"]["order_qty"], 3)

        carriage = rows["carriage-plate-mic6-0p250"]
        self.assertEqual(carriage["purchase_status"], "upload_ready")
        self.assertEqual(carriage["quantity"]["order_qty"], 1)
        self.assertEqual(
            carriage["selection"]["supplier_sku"], "ALUMIC6-250"
        )

    def test_catalog_mapped_hardware_is_not_ordered_twice(self):
        catalog = {
            "items": [
                {"id": "felt-set"},
                {"id": "dancer-sleeve-set"},
            ],
            "hardware": [
                {
                    "id": "hardware:FELT-PAD",
                    "fulfilled_by_catalog_item": "felt-set",
                },
                {
                    "id": "hardware:SLEEVE-A",
                    "fulfilled_by_catalog_item": "dancer-sleeve-set",
                },
                {
                    "id": "hardware:SLEEVE-B",
                    "fulfilled_by_catalog_item": "dancer-sleeve-set",
                },
                {"id": "hardware:ISO4762-M3x8"},
            ],
        }

        rows = procurement._full_order_rows(catalog)

        self.assertEqual(
            [row["id"] for row in rows],
            ["felt-set", "dancer-sleeve-set", "hardware:ISO4762-M3x8"],
        )

    def test_exact_cart_ready_hardware_keeps_checkout_caveat(self):
        result = procurement.audit()
        rows = {row["sku"]: row for row in result["hardware_order"]}
        for sku in (
            "ISO10642-M3x6", "ISO4762-M2x6", "ISO4762-M2x8",
            "ISO4762-M2x20", "ISO4762-M3x14", "ISO4762-M4x10",
            "MCMASTER-94459A120", "MCMASTER-94459A150",
        ):
            with self.subTest(sku=sku):
                row = rows[sku]
                self.assertEqual(row["purchase_status"], "cart_ready")
                self.assertIsNone(row["mapping_blocker"])
                self.assertIn(
                    "Confirm current pack price", row["checkout_condition"]
                )
                self.assertIn("not asserted", row["note"])

    def test_rfq_ready_custom_rows_remain_order_and_production_blocked(self):
        result = procurement.audit()
        rows = {
            row["id"]: row
            for row in procurement._full_order_rows(result["release_catalog"])
        }
        for item_id in (
            "flyer-guide-peek-one-piece",
            "stator-short-leadin-peek-cap-pair",
            "active-sector-peek-guide-pair",
            "active-sector-aluminum-yoke",
            "flyer-balance-b777-six-trim-set",
        ):
            with self.subTest(item=item_id):
                row = rows[item_id]
                self.assertEqual(row["purchase_status"], "rfq_ready")
                self.assertEqual(row["authorization_status"], "blocked")
                contract = row["manufacturing"]["rfq_contract"]
                self.assertEqual(contract["rfq_submission_status"], "ready")
                self.assertFalse(contract["order_authorized"])
                self.assertFalse(contract["production_authorized"])

    def test_absent_catalog_fulfillment_target_fails_closed(self):
        catalog = {
            "items": [],
            "hardware": [{
                "id": "hardware:FELT-PAD",
                "fulfilled_by_catalog_item": "missing-felt-set",
            }],
        }

        with self.assertRaisesRegex(ValueError, "missing-felt-set"):
            procurement._full_order_rows(catalog)


if __name__ == "__main__":
    unittest.main(verbosity=2)
