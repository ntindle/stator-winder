"""Exact-current flyer mass/inertia contract used by loads.py."""

from __future__ import annotations

from collections import Counter
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import loads  # noqa: E402
import nbk_p30_d10_official_occurrence as official_pulley  # noqa: E402


class CurrentFlyerMassModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = loads.current_flyer_mass_model()
        cls.rows = cls.model["rows"]
        cls.by_name = {str(row["name"]): row for row in cls.rows}

    def test_all_six_serialized_counterweight_stacks_are_present_once(self):
        contract = self.model["counterweight_contract"]
        self.assertEqual(contract["rear_stack_count"], 4)
        self.assertEqual(contract["front_stack_count"], 2)
        self.assertEqual(contract["stack_count"], 6)
        self.assertEqual(contract["serialized_mass_occurrence_count"], 24)
        self.assertEqual(
            set(contract["rear_occurrence_counts_by_suffix"].values()), {4}
        )

        rear_slugs = [
            row for row in self.rows
            if str(row["name"]).endswith("_tungsten_slug")
        ]
        front_slugs = [
            row for row in self.rows
            if str(row["name"]).startswith("front_trim_B777_")
        ]
        self.assertEqual((len(rear_slugs), len(front_slugs)), (4, 2))
        self.assertEqual(
            {str(row["material"]) for row in rear_slugs + front_slugs},
            {"ASTM-B777 tungsten alloy"},
        )
        front_hardware = [
            row for row in self.rows
            if str(row["name"]).startswith("front_trim_hardware_")
        ]
        self.assertEqual(
            Counter(str(row["material"]) for row in front_hardware),
            Counter({"steel": 4, "brass": 2}),
        )

    def test_official_stock_d10_pulley_mass_and_axial_J_are_used(self):
        pulley = self.by_name["flyer_pulley"]
        self.assertEqual(pulley["mass_g"], official_pulley.OFFICIAL_MASS_G)
        self.assertEqual(
            pulley["izz_about_M2_axis_g_mm2"],
            official_pulley.OFFICIAL_AXIAL_MOMENT_OF_INERTIA_KG_M2 * 1.0e9,
        )
        self.assertEqual(
            pulley["source_step_sha256"], official_pulley.SOURCE_STEP_SHA256
        )
        self.assertIn("NBK", str(pulley["mass_and_J_authority"]))

    def test_exact_merged_rows_are_unique_balanced_and_sum_to_total(self):
        total = self.model["total"]
        self.assertEqual(len(self.by_name), len(self.rows))
        self.assertEqual(total["part_count"], len(self.rows))
        self.assertLess(total["static_imbalance_g_mm"], 1.0e-6)
        self.assertLess(total["couple_imbalance_g_mm2"], 1.0e-6)
        self.assertAlmostEqual(
            sum(float(row["mass_g"]) for row in self.rows),
            float(total["mass_g"]), places=9,
        )
        self.assertAlmostEqual(
            sum(float(row["izz_about_M2_axis_g_mm2"])
                for row in self.rows),
            float(total["izz_about_M2_axis_g_mm2"]), places=6,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
