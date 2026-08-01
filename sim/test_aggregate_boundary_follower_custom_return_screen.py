"""Focused tests for the custom follower return design screen."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_custom_return_screen as screen


class AggregateBoundaryFollowerCustomReturnScreenTests(unittest.TestCase):

    def test_torsion_pair_hits_rate_force_and_shaft_envelope(self):
        result = screen.torsion_wire_pair()
        geometry = result["geometry"]
        force = result["force_rate_screen"]

        self.assertAlmostEqual(force["rotational_rate_N_mm_per_rad"], 2.4)
        self.assertAlmostEqual(force["linear_rate_each_N_per_mm"], 0.15)
        self.assertAlmostEqual(force["opposed_net_rate_N_per_mm"], 0.30)
        self.assertAlmostEqual(force["center_prewind_deg"], 14.323944878)
        self.assertEqual(force["individual_force_range_N"], [0.06, 0.24])
        self.assertAlmostEqual(
            force["net_restoring_force_at_hard_travel_N"], 0.18
        )
        self.assertGreater(
            geometry["coil_inside_diameter_mm"], geometry["shaft_diameter_mm"]
        )
        self.assertGreater(
            geometry["remaining_axial_span_after_bushing_and_two_coils_mm"], 0
        )

    def test_torsion_stress_tolerance_and_fatigue_are_screened_not_proven(self):
        result = screen.torsion_wire_pair()
        fatigue = result["stress_fatigue_screen"]
        tolerance = result["tolerance_screen"]

        self.assertAlmostEqual(fatigue["stress_range_MPa"][0], 95.909731952)
        self.assertAlmostEqual(fatigue["stress_range_MPa"][1], 383.638927809)
        self.assertGreater(fatigue["modified_goodman_screening_factor"], 2.8)
        self.assertGreater(fatigue["static_yield_screening_factor"], 4.6)
        self.assertFalse(fatigue["fatigue_life_qualified"])
        self.assertAlmostEqual(tolerance["estimated_rate_RSS_fraction"], 0.073178879)
        self.assertAlmostEqual(
            tolerance["estimated_rate_worst_case_fraction"], 0.115544444
        )

    def test_etched_plate_hits_rate_but_keeps_endurance_open(self):
        result = screen.etched_flat_flexure()
        geometry = result["geometry_each_leaf"]
        fatigue = result["stress_fatigue_screen"]

        self.assertAlmostEqual(geometry["width_mm"], 0.99825)
        self.assertAlmostEqual(
            result["force_rate_screen"]["combined_rate_N_per_mm"], 0.30
        )
        self.assertAlmostEqual(fatigue["maximum_stress_MPa"], 297.520661157)
        self.assertGreater(fatigue["static_yield_screening_factor"], 6.0)
        self.assertGreater(fatigue["zero_mean_fatigue_screening_factor"], 2.1)
        self.assertFalse(fatigue["fatigue_life_qualified"])
        self.assertTrue(result["manufacturing"]["preferred_route_credible"])
        self.assertEqual(result["manufacturing"]["laser_cut_route"], "prototype-only")

    def test_nylon12_is_bounded_to_replaceable_tangential_prototype(self):
        result = screen.nylon12_prototype_flexure()
        rates = result["force_rate_screen"]
        manufacturing = result["manufacturing"]

        self.assertAlmostEqual(result["geometry_each_leaf"]["thickness_mm"], 0.55)
        self.assertAlmostEqual(rates["nominal_combined_rate_N_per_mm"], 0.30)
        self.assertLess(
            rates["combined_rate_range_for_E_and_thickness_screen_N_per_mm"][0],
            0.22,
        )
        self.assertGreater(
            rates["combined_rate_range_for_E_and_thickness_screen_N_per_mm"][1],
            0.41,
        )
        self.assertTrue(manufacturing["tangential_prototype_credible"])
        self.assertFalse(manufacturing["radial_continuous_bias_credible"])
        self.assertFalse(result["stress_screen"]["creep_qualified"])

    def test_reduced_cartridge_ratio_calibration_and_envelope_are_bounded(self):
        result = screen.reduced_constant_force_cartridge()
        force = result["force_budget"]
        mechanism = result["reduction_mechanism"]
        package = result["package_screen"]
        ratios = mechanism["required_ratios"]

        self.assertAlmostEqual(force["maximum_effective_rate_N_per_mm"], 0.008326719)
        self.assertEqual(force["calibrated_acceptance_range_N"], [0.266, 0.286])
        self.assertGreaterEqual(min(ratios.values()), 0.235)
        self.assertLessEqual(max(ratios.values()), 0.315)
        self.assertEqual(
            package["proposed_fixed_cartridge_local_envelope"]["y_mm"],
            [-27.875, -12.125],
        )
        self.assertTrue(package["analytically_envelope_plausible"])
        self.assertFalse(package["full_carrier_gimbal_collision_sweep_complete"])
        self.assertFalse(force["full_stroke_force_and_rate_qualified"])

    def test_rejected_concepts_remain_explicit_and_fail_closed(self):
        rows = {row["concept"]: row for row in screen.rejected_concepts()}

        self.assertTrue(all(row["status"].startswith("REJECT") for row in rows.values()))
        self.assertAlmostEqual(
            rows["preloaded linear leaf radial return"]["calculation"]
            ["minimum_preload_deflection_mm"],
            30.023831416,
        )
        self.assertAlmostEqual(
            rows["post-buckled steel plateau flexure"]["calculation"]
            ["ideal_pinned_beam_thickness_mm"],
            0.061431436,
        )
        compact = rows["hand-made compact raw constant-force strip"]["calculation"]
        self.assertGreaterEqual(compact["examples"][1]["idealized_straightening_stress_MPa"], 965)

    def test_report_integrity_sources_and_authority_false(self):
        report = screen.build_report()
        screen.validate_report_integrity(report)

        self.assertEqual(report["report_sha256"], screen._canonical_hash(report))
        self.assertTrue(all(report["evidence_gates"].values()))
        self.assertTrue(
            all(value is False for value in report["fail_closed_gates"].values())
        )
        for gate in (
            "physical_authority",
            "CAD_authority",
            "procurement_authority",
            "BOM_change_authorized",
            "order_authorized",
            "release_authority",
        ):
            self.assertFalse(report[gate], gate)
        self.assertTrue(
            all(source["url"].startswith("https://") for source in report["sources"].values())
        )

        markdown = screen._markdown(report)
        self.assertIn("Physical/CAD/procurement/BOM/order/release authority", markdown)
        self.assertIn("Photochemical etching", markdown)
        self.assertIn("Replaceable prototype only", markdown)
        self.assertIn("0.235–0.315 adjustment", markdown)


if __name__ == "__main__":
    unittest.main()
