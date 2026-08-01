"""Regression tests for the fail-closed felt preload audit."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import felt_loads as F


class FeltSpringSelectionTests(unittest.TestCase):
    def test_exact_mcmaster_catalog_row_and_units(self):
        spring = F.SELECTED_SPRING
        self.assertEqual(spring.sku, "94125K614")
        self.assertEqual(spring.free_length_mm, 22.0)
        self.assertEqual(spring.od_mm, 9.25)
        self.assertEqual(spring.id_mm, 6.75)
        self.assertEqual(spring.compressed_length_at_max_load_mm, 14.1)
        self.assertAlmostEqual(spring.rate_n_per_mm, 8.896443230521, places=10)
        self.assertAlmostEqual(spring.max_load_n, 71.171545844168, places=10)

    def test_selected_spring_fits_and_retains_catalog_margin(self):
        checks = F.selected_spring_checks()
        self.assertTrue(all(row["pass"] for row in checks), checks)
        band = F.design_preload_band()
        self.assertGreaterEqual(
            band["minimum_installed_length_mm"]
            - F.SELECTED_SPRING.compressed_length_at_max_load_mm,
            F.MIN_COIL_BIND_MARGIN_MM,
        )

    def test_design_box_covers_one_to_ten_newtons(self):
        band = F.design_preload_band()
        low = F.drag_from_preload(band["minimum_normal_force_n"],
                                  F.MU_DESIGN_MAX)
        high = F.drag_from_preload(band["maximum_normal_force_n"],
                                   F.MU_DESIGN_MIN)
        self.assertAlmostEqual(low, F.DRAG_MIN_N, places=10)
        self.assertAlmostEqual(high, F.DRAG_MAX_N, places=10)
        self.assertGreater(band["wingnut_turns"], 5.0)
        self.assertLess(band["wingnut_turns"], 5.3)


class FeltCurrentIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = F.audit()

    def test_authoritative_wire_and_stack_are_read_from_sources(self):
        stack = self.report["current_stack"]
        self.assertAlmostEqual(stack["wire_contact_xyz_mm"][2], -157.0)
        self.assertAlmostEqual(stack["wire_radial_offset_from_stud_mm"], 6.0)
        self.assertAlmostEqual(stack["unloaded_pad_gap_mm"], 0.5)

    def test_status_fails_closed_and_matches_every_check(self):
        current_ok = all(row["pass"] for row
                         in self.report["current_integration_checks"])
        spring_ok = all(row["pass"] for row
                        in self.report["selected_spring_checks"])
        self.assertEqual(self.report["current_integration_ready"], current_ok)
        self.assertEqual(self.report["selected_spring_sizing_ready"], spring_ok)
        self.assertEqual(self.report["status"],
                         "PASS" if current_ok and spring_ok else "FAIL")

    def test_geometry_issues_are_explicit_not_silently_accepted(self):
        checks = {row["name"]: row for row
                  in self.report["current_integration_checks"]}
        expected = {
            "wire centered between unloaded felt faces",
            "fixed backing is seated against printed boss",
            "metal backing supports wire contact",
            "selected spring is not beyond catalog compression in modeled pose",
            "hardware schedule selects exact spring SKU",
            "separate spring thrust washer is placed under wingnut",
        }
        self.assertTrue(expected.issubset(checks))
        # While the placeholder remains, all associated checks must fail.  Once
        # root integrates the recommendation this branch naturally disappears
        # and the same regression test remains valid.
        schedule = self.report["current_stack"]["hardware_schedule"]
        if schedule["sku"] == "SPRING-TBD-FELT":
            for name in expected:
                self.assertFalse(checks[name]["pass"], name)

    def test_recommended_m4x50_stud_restores_full_engagement(self):
        geometry = self.report["recommended_geometry"]
        self.assertEqual(geometry["recommended_standard_stud_length_mm"], 50.0)
        self.assertGreaterEqual(
            geometry["minimum_thread_engagement_over_adjustment_mm"],
            F.MIN_THREAD_ENGAGEMENT_MM)

    def test_report_writer_preserves_failed_state(self):
        json_path, md_path = F.write_reports(self.report)
        self.assertTrue(json_path.is_file())
        self.assertTrue(md_path.is_file())
        self.assertIn(f'"status": "{self.report["status"]}"',
                      json_path.read_text(encoding="utf-8"))
        self.assertIn(
            "current CAD/BOM integration: "
            f"{'PASS' if self.report['current_integration_ready'] else 'FAIL'}",
                      md_path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
