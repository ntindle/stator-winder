"""Regression tests for the constructive plan/controller conversion."""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

import yaml

from winding_plan import load_slot_winding_plan


ROOT = Path(__file__).resolve().parents[1]
PLAN_PATH = ROOT / "out" / "reports" / "slot_winding_plan.json"
SETTINGS_PATH = ROOT / "out" / "settings.yml"


class SlotWindingPlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan = load_slot_winding_plan(PLAN_PATH)
        cls.settings = yaml.safe_load(SETTINGS_PATH.read_text())

    def test_live_plan_is_exact_default_job_and_controller_ready(self):
        plan = self.plan
        self.assertTrue(plan.controller_ready)
        self.assertEqual(plan.slots, 24)
        self.assertEqual(plan.turns_per_tooth, 50)
        self.assertEqual(plan.wire_finished_d_mm, 0.22352)
        self.assertEqual(plan.model_wire_envelope_mm, 0.22352)
        self.assertEqual(plan.receiving_sensitivity_wire_envelope_mm, 0.235)
        self.assertEqual(plan.receiving_sensitivity_status, "PASS")
        self.assertEqual(len(plan.placements), 50)
        self.assertEqual(len(plan.half_turn_centers), 100)

    def test_every_turn_has_two_identical_radial_crossing_centers(self):
        for turn in range(self.plan.turns_per_tooth):
            left, right = self.plan.half_turn_centers[2 * turn:2 * turn + 2]
            self.assertEqual(left.placement_index, turn)
            self.assertEqual(right.placement_index, turn)
            self.assertAlmostEqual(left.radial_mm, right.radial_mm, places=12)
            self.assertAlmostEqual(
                left.m0_target_rad, right.m0_target_rad, places=12)

    def test_default_settings_identity_and_plan_path_match(self):
        self.plan.validate_settings(self.settings)
        ref = self.settings["job"]["winding_plan"]
        self.assertEqual(
            (SETTINGS_PATH.parent / ref).resolve(), PLAN_PATH.resolve())

    def test_nominal_and_half_turn_leadout_waypoints_are_complete(self):
        span = self.settings["job"]["radial_winding_span_mm"]
        m0 = self.settings["motor"]["M0"]
        m0_range = [m0["wind_range_start"], m0["wind_range_end"]]
        nominal = 50 * 2.0 * math.pi
        base = self.plan.controller_waypoints(span, m0_range, nominal)
        lead = self.plan.controller_waypoints(span, m0_range,
                                               nominal + math.pi)
        self.assertEqual(len(base), 101)
        self.assertEqual(len(lead), 102)
        self.assertAlmostEqual(base[0]["m2_phase_rad"], 0.0)
        self.assertAlmostEqual(base[-1]["m2_phase_rad"], nominal)
        self.assertAlmostEqual(lead[-1]["m2_phase_rad"], nominal + math.pi)
        self.assertAlmostEqual(
            lead[-1]["m0_target_rad"], lead[-2]["m0_target_rad"])
        self.assertTrue(all(
            a["m2_phase_rad"] < b["m2_phase_rad"]
            for a, b in zip(lead, lead[1:])))

    def test_zero_and_offset_phase_origins_each_keep_100_centers(self):
        span = self.settings["job"]["radial_winding_span_mm"]
        motor = self.settings["motor"]["M0"]
        m0_range = [motor["wind_range_start"], motor["wind_range_end"]]
        nominal = 50 * 2.0 * math.pi
        cases = (
            (0.0, nominal, 0.0),
            (math.pi, nominal + math.pi, math.pi),
        )
        for origin, target, first_expected in cases:
            with self.subTest(origin=origin):
                points = self.plan.controller_waypoints(
                    span, m0_range, target, origin)
                centers = [p for p in points
                           if p["kind"] == "placement_center"]
                holds = [p for p in points if p["kind"] == "final_hold"]
                self.assertEqual(len(centers), 100)
                self.assertEqual(centers[0]["m2_phase_rad"], first_expected)
                self.assertEqual(holds[-1]["m2_phase_rad"], target)
                for placement in range(50):
                    self.assertEqual(sum(
                        p["placement_index"] == placement for p in centers), 2)

    def test_offset_origin_rejects_target_before_closure_crossing(self):
        span = self.settings["job"]["radial_winding_span_mm"]
        motor = self.settings["motor"]["M0"]
        m0_range = [motor["wind_range_start"], motor["wind_range_end"]]
        nominal = self.plan.turns_per_tooth * 2.0 * math.pi
        with self.assertRaisesRegex(
                ValueError, "before the required post-deposition closure"):
            self.plan.controller_waypoints(
                span, m0_range, nominal, math.pi)

    def test_plan_radial_step_can_settle_with_configured_axis_speeds(self):
        radial = [point.active_tooth_radial_mm
                  for point in self.plan.placements]
        max_step = max(abs(b - a) for a, b in zip(radial, radial[1:]))
        available = math.pi / self.settings["motor"]["M2"]["velocity"]
        required = max_step / (
            self.settings["motor"]["M0"]["velocity"]
            * (8.0 / (2.0 * math.pi)))
        self.assertLessEqual(max_step, 0.22352 + 1e-9)
        self.assertGreater(available - required - 0.02, 0.10)

    def test_nominal_wire_mismatch_fails_closed(self):
        bad = copy.deepcopy(self.settings)
        bad["job"]["wire_finished_d_mm"] = 0.24
        with self.assertRaisesRegex(ValueError, "wire_finished_d_mm mismatch"):
            self.plan.validate_settings(bad)

    def test_mutated_half_turn_pair_is_rejected(self):
        raw = json.loads(PLAN_PATH.read_text())
        raw["half_turn_centers"][1]["placement_index"] = 1
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text(json.dumps(raw))
            with self.assertRaisesRegex(ValueError, "adjacent half-turn pair"):
                load_slot_winding_plan(path)


if __name__ == "__main__":
    unittest.main(verbosity=2)
