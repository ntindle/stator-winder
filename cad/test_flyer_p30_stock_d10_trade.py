"""Regression tests for the fail-closed stock-D10 flyer trade."""

from __future__ import annotations

import json
import math
import unittest

import flyer_p30_stock_d10_trade as trade


class StockD10TradeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hash_before = trade._sha256(trade.SOURCE_D10_STEP)
        cls.mtime_before = trade.SOURCE_D10_STEP.stat().st_mtime_ns
        cls.official = trade.official_d10()
        cls.shaft = trade.proposed_necked_shaft()

    def test_official_d10_is_exact_immutable_one_solid(self):
        self.assertTrue(trade.HISTORICAL_NON_GOVERNING)
        self.assertEqual(trade.SUPERSEDED_BY, "M2-001 Rev D L79.00")
        self.assertEqual(self.hash_before, trade.SOURCE_D10_SHA256)
        self.assertEqual(trade.SOURCE_D10_STEP.stat().st_size, 57130)
        self.assertEqual(len(self.official.solids()), 1)
        self.assertTrue(self.official.is_valid)
        self.assertAlmostEqual(self.official.volume, 7834.785240560267, places=5)
        self.assertEqual(trade._sha256(trade.SOURCE_D10_STEP), self.hash_before)
        self.assertEqual(
            trade.SOURCE_D10_STEP.stat().st_mtime_ns, self.mtime_before
        )

    def test_necked_shaft_is_one_solid_with_full_through_bore_seat(self):
        self.assertEqual(len(self.shaft.solids()), 1)
        self.assertTrue(self.shaft.is_valid)
        box = self.shaft.bounding_box()
        self.assertAlmostEqual(box.min.Z, -110.75, places=5)
        self.assertAlmostEqual(box.max.Z, -30.0, places=5)
        self.assertAlmostEqual(box.size.Z, 80.75, places=5)
        self.assertEqual(trade.NECK_LENGTH_MM, 18.5)
        self.assertEqual(
            (trade.NECK_OD_MM - trade.NECK_ID_MM) / 2.0, 2.0
        )
        self.assertGreaterEqual(
            trade.REAR_BEARING_START_Z_MM - trade.INTERNAL_TRANSITION_END_Z_MM,
            3.0,
        )

    def test_report_is_conditional_and_invalidates_old_balance(self):
        self.assertTrue(trade.REPORT.is_file())
        report = json.loads(trade.REPORT.read_text(encoding="utf-8"))
        self.assertEqual(
            report["status"], "CONDITIONAL_GEOMETRY_PASS_RELEASE_BLOCKED"
        )
        self.assertFalse(report["production_authorized"])
        self.assertFalse(report["candidate_integration_authorized"])
        self.assertTrue(report["official_product"]["standard_stock_d10"])
        self.assertEqual(report["official_product"]["mass_g"], 28.0)
        self.assertFalse(
            report["mass_and_balance_delta"]["exact_balance_solution_still_valid"]
        )
        current = report["placement"]["current_no_extension"]
        self.assertLess(
            current["pulley_to_entry_bracket_mm"],
            current["release_clearance_target_mm"],
        )
        conditional = report["placement"]["conditional_full_engagement"]
        self.assertTrue(conditional["geometry_screen_passes"])
        self.assertFalse(conditional["release_authorized"])
        self.assertGreaterEqual(
            conditional["pulley_to_shifted_entry_bracket_mm"], 2.2
        )
        self.assertTrue(math.isfinite(
            report["shaft_load_screen"][
                "combined_von_mises_mpa_without_Kt_or_pretension"
            ]
        ))


if __name__ == "__main__":
    unittest.main(verbosity=2)
