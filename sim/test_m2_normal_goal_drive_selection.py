"""Regression gates for the normal-GOAL M2 drive selection."""

from __future__ import annotations

import hashlib
from pathlib import Path
import unittest

import m2_normal_goal_drive_selection as selection


class M2NormalGoalDriveSelectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = selection.analyze()

    def test_full_inertia_and_required_torque_are_reproducible(self) -> None:
        duty = self.report["OD65_10N_full_inertia_torque"]
        self.assertAlmostEqual(duty["full_output_inertia_kgm2"], 9.629064906673928e-5)
        self.assertAlmostEqual(duty["acceleration_torque_nm"], 0.0192581298133479)
        self.assertAlmostEqual(duty["required_output_torque_nm"], 0.3642581298133478)
        self.assertAlmostEqual(duty["required_2x_running_torque_nm"], 0.7285162596266956)

    def test_36v_curve_narrowly_passes_but_24v_fails(self) -> None:
        duty = self.report["OD65_10N_full_inertia_torque"]
        self.assertTrue(duty["manufacturer_curve_motor_gate_ge_2x"])
        self.assertGreaterEqual(duty["available_to_required_multiple"], 2.0)
        self.assertLess(duty["percent_above_2x_threshold"], 1.0)
        self.assertFalse(duty["24V_gate_ge_2x"])

    def test_exact_driver_and_36v_condition_are_pinned(self) -> None:
        binding = self.report["manufacturer_evidence"]["driver_binding"]
        self.assertEqual(binding["model"], "CS-D508")
        self.assertEqual(
            binding["manual_appendix_A_explicitly_tested_motor"],
            "CS-M21708",
        )
        self.assertEqual(binding["manual_motor_encoder_requirement_lines"], 1000)
        self.assertEqual(binding["input_voltage_range_vdc"], [20.0, 50.0])
        self.assertGreaterEqual(binding["maximum_output_current_A_peak"], 8.0)
        self.assertEqual(binding["selected_supply_vdc"], 36.0)
        self.assertEqual(binding["curve_condition_A_RMS"], 2.5)
        self.assertAlmostEqual(
            binding["curve_condition_equivalent_A_peak_if_sinusoidal"],
            2.5 * 2.0 ** 0.5,
        )
        self.assertEqual(binding["nearest_0p1A_peak_settings"], [3.5, 3.6])
        self.assertEqual(binding["software_peak_current_parameter"], "PR 2")
        self.assertEqual(binding["software_peak_current_increment_A"], 0.1)
        self.assertAlmostEqual(
            binding["lower_setting_equivalent_RMS_A"],
            3.5 / 2.0 ** 0.5,
        )
        self.assertAlmostEqual(
            binding["upper_setting_equivalent_RMS_A"],
            3.6 / 2.0 ** 0.5,
        )
        self.assertTrue(binding["prior_CS1_D503S_3A_peak_rejected"])
        self.assertFalse(
            binding["exact_peak_setting_reproducing_curve_confirmed"]
        )
        self.assertFalse(
            binding["motor_product_page_current_rating_is_self_consistent"]
        )
        self.assertTrue(binding["current_catalog_and_curve_RMS_current_match"])
        gates = self.report["release_gates"]
        self.assertTrue(
            gates["exact_CS_D508_driver_officially_tested_with_CS_M21708"]
        )
        self.assertTrue(
            gates["driver_peak_capacity_exceeds_RMS_2p5A_equivalent"]
        )
        self.assertFalse(
            gates["driver_configuration_reproduces_RMS_2p5A_curve"]
        )
        self.assertTrue(gates["regulated_36V_supply_condition_defined"])
        self.assertTrue(gates["exact_36V_supply_SKU_pinned"])
        self.assertTrue(
            gates["exact_36V_supply_capacity_and_input_windows_pinned"]
        )
        self.assertFalse(
            gates["36V_supply_mains_safety_integration_pinned"]
        )
        supply = self.report["manufacturer_evidence"]["supply_binding"]
        self.assertEqual(supply["model"], "LSP-360-36")
        self.assertEqual(supply["product_page_status"], "active_add_to_cart")
        self.assertEqual(supply["official_online_price_usd"], 199.0)
        self.assertEqual(supply["output_voltage_vdc"], 36.0)
        self.assertEqual(supply["continuous_output_current_A"], 10.0)
        self.assertEqual(supply["peak_output_current_A"], 18.0)
        self.assertEqual(supply["rated_power_W"], 360.0)
        self.assertEqual(
            supply["input_voltage_windows_vac"],
            [[92.0, 138.0], [184.0, 276.0]],
        )
        self.assertTrue(supply["exact_SKU_and_electrical_capacity_pinned"])
        self.assertTrue(supply["candidate_cart_ready"])
        self.assertFalse(supply["mains_safety_integration_pinned"])
        self.assertFalse(supply["production_order_authorized"])
        self.assertEqual(
            self.report["manufacturer_evidence"]["source_hashes"]
            ["cad/models/upgrades/Leadshine_CS-D508_manual_v1.0.pdf"],
            "0faaf40eebe24203511b50b3e3658bec9fc298b13221031759b84d3eb9bdba60",
        )
        selected = self.report["selection"]
        self.assertIn("CS-D508", selected["driver"])
        self.assertNotIn("CL42T", selected["driver"])
        self.assertNotIn("CS1-D503S", selected["driver"])
        self.assertIn("36 VDC", selected["supply_condition"])
        self.assertIn("LSP-360-36", selected["supply_condition"])
        self.assertNotIn("24 V", selected["supply_condition"])

    def test_exact_1_to_1_and_p30_belt_capacity_pass(self) -> None:
        drive = self.report["transmission"]
        self.assertEqual(drive["motor_teeth"], 30)
        self.assertEqual(drive["flyer_teeth"], 30)
        self.assertEqual(drive["belt_pitch_length_mm"], 210.0)
        self.assertEqual(drive["exact_ratio"], 1.0)
        self.assertTrue(drive["upstream_radians_contract_preserved"])
        self.assertAlmostEqual(drive["allowable_transmission_torque_nm"], 1.854)
        self.assertTrue(drive["transmission_capacity_gate_ge_2x"])

    def test_D_shaft_uses_split_clamp_plus_BNW_but_retention_stays_open(self) -> None:
        retention = self.report["motor_shaft_retention"]
        self.assertFalse(retention["shaft_is_round"])
        self.assertIn("D-profile", retention["exact_profile"])
        self.assertFalse(retention["stock_split_clamp_round_h6_h7_interface_authorized"])
        self.assertIn("BNW", retention["selected_method"])
        self.assertTrue(retention["stock_split_clamp_plus_BNW_configuration_is_orderable"])
        self.assertEqual(retention["BNW_set_screw_count"], 2)
        self.assertEqual(retention["BNW_set_screw_spacing_deg"], 90.0)
        self.assertFalse(retention["exact_BNW_set_screw_size_known"])
        self.assertFalse(retention["modified_pulley_retention_torque_published"])
        self.assertFalse(retention["retention_release_gate"])

    def test_BNW_inertia_is_bounded_but_reference_slip_chart_does_not_release_it(self) -> None:
        pulley = self.report["motor_pulley_geometry_and_inertia"]
        self.assertEqual(pulley["selected_additional_machining"], "BNW")
        self.assertEqual(pulley["stock_clamp_bolt"]["size"], "M2")
        self.assertAlmostEqual(pulley["stock_clamp_bolt"]["tightening_torque_nm"], 0.5)
        self.assertEqual(pulley["BNW"]["set_screw_count"], 2)
        self.assertGreater(
            pulley["full_motor_pulley_inertia_upper_kgm2"],
            pulley["published_stock_inertia_kgm2"],
        )
        chart = self.report["motor_shaft_retention"]["reference_slip_chart"]
        self.assertTrue(chart["reference_only_not_guaranteed"])
        self.assertTrue(chart["actual_use_testing_required_by_NBK"])
        self.assertFalse(chart["proves_required_slip_torque"])
        stock = pulley["official_stock_CAD"]
        self.assertTrue(stock["local_exact_STEP_acquired"])
        self.assertTrue(stock["exact_stock_CAD_gate"])
        self.assertTrue(stock["immutable_stock_only_BNW_absent"])
        self.assertFalse(stock["configured_BNW_drawing_received"])
        self.assertEqual(
            stock["observed_sha256"],
            "996449b7d9ec7703e7b38c6f75eff00a1174e3e1f088c05f0f1460b205169df9",
        )
        self.assertTrue(
            self.report["release_gates"]["motor_pulley_exact_stock_CAD_acquired"]
        )
        self.assertNotIn(
            "motor_pulley_exact_stock_CAD_acquired",
            self.report["controlling_open_gates"],
        )
        self.assertIn(
            "motor_pulley_BNW_set_screw_size_and_drawing_known",
            self.report["controlling_open_gates"],
        )

    def test_ratio_trade_reflects_motor_speed_and_full_inertia(self) -> None:
        trade = self.report["ratio_trade_1p0_to_3p0"]
        self.assertEqual(trade["selected_ratio"], 1.0)
        rows = trade["rows"]
        self.assertEqual([row["ratio_motor_speed_over_flyer_speed"] for row in rows],
                         [1.0, 1.1, 1.25, 1.5, 2.0, 2.5, 3.0])
        for row in rows:
            self.assertAlmostEqual(
                row["motor_rpm_at_300_flyer_rpm"],
                300.0 * row["ratio_motor_speed_over_flyer_speed"],
            )
            self.assertTrue(row["motor_math_ge_2x"])
        self.assertGreater(rows[-1]["full_output_inertia_kgm2"], rows[0]["full_output_inertia_kgm2"])

    def test_dual_motor_is_unnecessary_architecture_fallback(self) -> None:
        dual = self.report["dual_NEMA17_fallback"]
        self.assertTrue(dual["static_motor_math_ge_2x"])
        self.assertFalse(dual["architecture_authorized"])

    def test_reference_selection_is_not_a_production_release(self) -> None:
        source = Path(selection.__file__).resolve()
        self.assertEqual(
            self.report["analysis_source"],
            "sim/m2_normal_goal_drive_selection.py",
        )
        self.assertEqual(
            self.report["analysis_source_sha256"],
            hashlib.sha256(source.read_bytes()).hexdigest(),
        )
        self.assertTrue(self.report["reference_CAD_integration_authorized"])
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["procurement_authorized"])
        self.assertFalse(self.report["release_gates"]["production_authorized"])
        self.assertGreater(len(self.report["controlling_open_gates"]), 0)


if __name__ == "__main__":
    unittest.main()
