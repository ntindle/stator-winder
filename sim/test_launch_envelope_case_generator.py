"""Focused tests for deterministic launch-corner evidence generation."""

from __future__ import annotations

import math
from pathlib import Path
import tempfile
import unittest

import yaml


import launch_envelope_case_generator as generator


class LaunchEnvelopeCaseGeneratorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.plans = generator.derive_all_case_plans()

    def test_all_24_cases_receive_one_deterministic_plan(self) -> None:
        self.assertEqual(len(self.plans), 24)
        self.assertEqual(len({plan.case.case_id for plan in self.plans}), 24)
        self.assertTrue(all(plan.turns_per_tooth > 0 for plan in self.plans))

    def test_representative_topologies_make_all_24_corner_plans_feasible(self) -> None:
        feasible = [plan for plan in self.plans if plan.feasible]
        rejected = [plan for plan in self.plans if not plan.feasible]
        self.assertEqual(len(feasible), 24)
        self.assertEqual(rejected, [])
        self.assertTrue(all(
            plan.selected_analysis["representative_placement_band"]
            ["workholder_reach_margin_mm"]
            + 1.0e-9 >= plan.case.reach_reserve_mm
            for plan in feasible
        ))

    def test_capacity_stress_turn_selection_is_stable(self) -> None:
        by_geometry = {
            (plan.case.od_mm, plan.case.wire_finished_d_mm,
             plan.case.spindle_id):
                plan.turns_per_tooth
            for plan in self.plans
        }
        self.assertEqual(by_geometry[(28.0, 0.2, "er11")], 61)
        self.assertEqual(by_geometry[(28.0, 0.2, "shaft8")], 96)
        self.assertEqual(by_geometry[(28.0, 0.5, "er11")], 9)
        self.assertEqual(by_geometry[(28.0, 0.5, "shaft8")], 15)
        self.assertEqual(by_geometry[(65.0, 0.2, "er11")], 159)
        self.assertEqual(by_geometry[(65.0, 0.5, "shaft8")], 24)

    def test_one_turn_uses_one_wire_pitch_not_the_full_slot_span(self) -> None:
        case = next(
            plan.case for plan in self.plans
            if plan.case.od_mm == 28.0
            and plan.case.wire_finished_d_mm == 0.5
            and plan.case.spindle_id == "er11"
        )
        spec = generator.StatorSpec(
            slots=case.slots,
            od=case.od_mm,
            stack=case.stack_mm,
            shaft_d=case.shaft_d_mm,
            wire_d=case.wire_finished_d_mm,
            turns=1,
            hub_od_ratio=case.hub_od_ratio,
            winding_config=case.winding_config,
        )
        analysis = generator.coil_growth.analyze_job(spec)
        band = generator._placement_band(spec, analysis, case.spindle_id)
        full = band["full_accessible_radial_span_mm"]
        occupied = band["occupied_radial_span_mm"]
        self.assertAlmostEqual(band["occupied_span_mm"], 0.5, places=9)
        self.assertLess(occupied[1] - occupied[0], full[1] - full[0])
        self.assertGreaterEqual(
            band["workholder_reach_margin_mm"], case.reach_reserve_mm,
        )

    def test_job_identity_binds_representative_topology_and_reach(self) -> None:
        plan = next(
            plan for plan in self.plans
            if plan.case.od_mm == 28.0
            and plan.case.wire_finished_d_mm == 0.5
            and plan.case.spindle_id == "er11"
        )
        self.assertEqual(plan.job["slots"], 12)
        self.assertEqual(plan.job["hub_od_mm"], 19.5)
        self.assertEqual(
            plan.job["upstream_config_id"],
            generator.authority.UPSTREAM_12N14P_CONFIG_ID,
        )
        self.assertEqual(plan.job["reach_reserve_mm"], 0.25)

    def test_settings_yaml_carries_the_complete_topology_identity(self) -> None:
        plan = next(
            plan for plan in self.plans
            if plan.case.od_mm == 28.0
            and plan.case.stack_mm == 5.0
            and plan.case.wire_finished_d_mm == 0.5
            and plan.case.spindle_id == "er11"
            and plan.case.shaft_d_mm == 3.0
        )
        with tempfile.TemporaryDirectory() as temporary:
            path, cfg, verdict = generator._settings_evidence(
                plan, Path(temporary),
            )
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
        self.assertIsNotNone(cfg)
        self.assertEqual(verdict["status"], "PASS")
        job = document["job"]
        for key in (
            "slots", "hub_od_mm", "hub_od_ratio", "winding_config",
            "upstream_config_id", "topology_basis", "reach_reserve_mm",
        ):
            self.assertEqual(job[key], plan.job[key])
        self.assertEqual(
            job["radial_winding_span_mm"],
            verdict["representative_placement_band"]
            ["occupied_radial_span_mm"],
        )

    def test_nonfinite_analysis_values_are_standard_json_safe(self) -> None:
        safe = generator._json_safe({"value": math.inf})
        self.assertEqual(safe, {"value": "Infinity"})

    def test_superseded_24_slot_throat_contradiction_is_dimensioned(self) -> None:
        proof = generator._superseded_od28_24n22p_throat_proof()
        self.assertAlmostEqual(
            proof["maximum_inscribed_center_clearance_mm"],
            0.3259763339095709,
            places=12,
        )
        self.assertAlmostEqual(
            proof["maximum_compatible_finished_wire_mm"],
            0.39795266781914185,
            places=12,
        )
        self.assertAlmostEqual(
            proof["finished_wire_diameter_deficit_mm"],
            0.10204733218085815,
            places=12,
        )
        self.assertLess(proof["coil_growth_access_span_mm"], 0.0)
        self.assertEqual(
            proof["coil_growth_max_turns_at_design_fill"], 0,
        )

    def test_step_header_timestamp_normalization_is_deterministic(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "probe.step"
            path.write_text(
                "ISO-10303-21;\nHEADER;\n"
                "FILE_NAME('probe','2026-07-11T12:34:56',('Author'),"
                "('Open CASCADE'),'processor','build123d','Unknown');\n"
                "ENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n",
                encoding="utf-8",
            )
            generator._normalize_step_header(path)
            first = path.read_bytes()
            generator._normalize_step_header(path)
            second = path.read_bytes()
            self.assertEqual(first, second)
            self.assertIn(b"1970-01-01T00:00:00", first)

    def test_capture_wall_clock_logs_do_not_enter_evidence_hashes(self) -> None:
        output = (
            "2026-07-11 18:16:37,451 - Wind - \x1b[92mINFO\x1b[0m done\n"
            "cycle complete: 629 motor commands\n"
        )
        self.assertEqual(
            generator._stable_subprocess_output(output),
            "cycle complete: 629 motor commands",
        )


if __name__ == "__main__":
    unittest.main()
