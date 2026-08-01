"""Deterministic contract checks for the permanent-cap DFM report."""

from __future__ import annotations

import unittest

import permanent_cap_material_dfm as dfm


class PermanentCapMaterialDfmTests(unittest.TestCase):
    def setUp(self) -> None:
        self.report = dfm.build_report()

    def test_frozen_contract(self) -> None:
        contract = self.report["contract"]
        self.assertEqual(contract["lane_id"], "cap-r3-sector-lane-v1")
        self.assertEqual(contract["nominal_wall_mm"], 1.0)
        self.assertEqual(contract["minimum_contact_surface_radius_mm"], 2.88824)
        self.assertEqual(contract["minimum_clear_polished_groove_width_mm"], 0.47752)
        self.assertEqual(contract["nominal_wire"]["finished_diameter_mm"], 0.22352)
        self.assertEqual(contract["maximum_open_access_wire_diameter_mm"], 0.5)

    def test_only_fit_prototype_is_in_house(self) -> None:
        route = self.report["routes"]["in_house_petg_fit_prototype"]
        self.assertEqual(route["status"], "SELECTED_FOR_FIT_ONLY")
        self.assertFalse(route["wire_winding_authorized"])
        self.assertIn("0.2 mm", route["tooling"])

    def test_unfilled_peek_is_conditional_selection(self) -> None:
        decision = self.report["decision"]
        self.assertIn("unfilled PEEK", decision["selected_material_family"])
        self.assertIn("CNC", decision["selected_low_volume_process"])
        self.assertIn("450G", decision["selected_volume_process"])
        self.assertFalse(decision["production_authorized"])
        self.assertFalse(decision["purchasing_authorized"])
        self.assertFalse(decision["bom_or_release_catalog_edited"])

    def test_manufacturing_reserves_exceed_contract(self) -> None:
        dfm_values = self.report["drawing_and_supplier_dfm"]
        cnc = dfm_values["low_volume_cnc_planning_values"]
        mold = dfm_values["volume_molding_planning_values"]
        self.assertGreater(cnc["worst_radius_after_negative_tolerance_mm"], 2.88824)
        self.assertGreater(cnc["worst_groove_after_negative_tolerance_mm"], 0.47752)
        self.assertGreater(mold["worst_radius_after_negative_tolerance_mm"], 2.88824)
        self.assertGreater(mold["worst_groove_after_negative_tolerance_mm"], 0.47752)

    def test_abrasive_filled_routes_are_prohibited(self) -> None:
        route = self.report["routes"]["fiber_or_mineral_filled_polymers"]
        self.assertIn("PROHIBITED", route["status"])
        self.assertTrue(any("PEEK GF/CF" in item for item in route["excluded"]))
        self.assertTrue(any("blasted" in item for item in route["excluded"]))

    def test_coupon_plan_covers_every_physical_gate(self) -> None:
        plan = self.report["physical_coupon_plan"]
        for key in (
            "material_receipt",
            "dimensional_and_finish",
            "retention_and_fit",
            "thermal_and_varnish",
            "enamel_abrasion",
            "dielectric",
        ):
            self.assertIn(key, plan)
        self.assertIn("10,000", plan["enamel_abrasion"]["procedure"][0])
        self.assertEqual(
            plan["dimensional_and_finish"]["acceptance"]["minimum_local_contact_radius_mm"],
            2.88824,
        )
        self.assertEqual(
            plan["dimensional_and_finish"]["acceptance"]["minimum_clear_groove_width_mm"],
            0.47752,
        )

    def test_all_release_gates_fail_closed(self) -> None:
        self.assertTrue(self.report["release_gates"])
        self.assertTrue(all(value is False for value in self.report["release_gates"].values()))

    def test_source_set_is_dated_and_primary_or_current_vendor(self) -> None:
        sources = self.report["sources"]
        self.assertGreaterEqual(len(sources), 15)
        self.assertTrue(all(source["accessed"] == "2026-07-11" for source in sources))
        self.assertTrue(all(source["url"].startswith("https://") for source in sources))
        ids = {source["id"] for source in sources}
        for required in (
            "bambu-a1-spec",
            "victrex-450g-tds",
            "mcmaster-peek-sheet",
            "xometry-peek-cnc",
            "fictiv-spi-finish",
            "remington-32sns",
        ):
            self.assertIn(required, ids)


if __name__ == "__main__":
    unittest.main()
