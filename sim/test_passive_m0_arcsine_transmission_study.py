"""Regression tests for the passive M0 inverse-sine transmission study."""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import passive_m0_arcsine_transmission_study as study  # noqa: E402
from traj import load_events  # noqa: E402


PRODUCTION_SETTINGS_SHA256 = (
    "6c4dbd8287c14dfaf98203a3733b743dd1ea04a39abe8f36f7be502d641cf4d1"
)
CANONICAL_CAPTURE_SHA256 = (
    "350210c36550ae2c22e5352675aee6bd2eebb18325f3b5f45c7cb35d6a314958"
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class PassiveM0ArcsineStudyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = json.loads(study.OUTPUT_JSON.read_text(encoding="utf-8"))

    def test_production_settings_and_canonical_capture_are_untouched(self):
        self.assertEqual(sha256(study.PRODUCTION_SETTINGS),
                         PRODUCTION_SETTINGS_SHA256)
        self.assertEqual(sha256(study.CANONICAL_CAPTURE),
                         CANONICAL_CAPTURE_SHA256)
        self.assertEqual(self.report["role"],
                         "analytical_study_only_no_production_mutation")
        self.assertIsNone(self.report["cad_brief"]["geometry_artifact"])

    def test_n6_capture_is_unmodified_upstream_at_requested_velocity(self):
        candidate = next(item for item in study.candidates()
                         if item.label == "n6_exact")
        events = load_events(candidate.capture_path)
        meta = next(row for row in events if row["e"] == "meta")
        self.assertEqual(meta["controller_mode"], "upstream")
        self.assertIsNone(meta["controller_adapter_sha256"])
        self.assertIsNone(meta["winding_plan"])
        self.assertAlmostEqual(meta["velocities"][2],
                               math.pi / (6.0 * 0.03), places=12)
        self.assertEqual(meta["winder_commit"],
                         "6039b33c8f15a20086c2195c3f2d02b3a833e8ca")

    def test_n8_is_retained_as_a_real_upstream_cycle_failure(self):
        candidate = next(item for item in study.candidates()
                         if item.label == "n8_exact")
        self.assertFalse(candidate.capture_path.exists())
        text = candidate.failure_path.read_text(encoding="utf-8")
        self.assertIn("AssertionError", text)
        self.assertIn("motor2_pos: 2.9269908169872307", text)
        sweep = next(row for row in self.report["sweep"]
                     if row["label"] == "n8_exact")
        self.assertEqual(sweep["capture"]["status"], "FAIL")

    def test_static_inverse_is_exact_but_real_24x100_pitch_is_not(self):
        self.assertLessEqual(
            self.report["static_mapping_proof"]
            ["maximum_static_inversion_error_mm"], 1.0e-9)
        selected = self.report["selected_candidate"]
        self.assertEqual(selected["half_turn_interval_count"], 2400)
        pitch = selected["serial_coordinate_pitch"]
        self.assertEqual(pitch["interval_count"], 2400)
        self.assertFalse(pitch["all_intervals_linear_within_1um"])
        self.assertGreater(pitch["zero_pitch_interval_count"], 0)
        with study.OUTPUT_CSV.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(len(rows), 24 * 100)
        self.assertEqual(
            sorted({int(row["pass_index"]) for row in rows}), list(range(24)))

    def test_n6_poll_sample_and_command_issue_are_distinguished(self):
        timing = self.report["selected_candidate"]["poll_alignment"]
        self.assertTrue(timing["stale_poll_samples_half_turn_aligned"])
        self.assertFalse(timing["commands_physically_issued_at_half_turns"])
        self.assertAlmostEqual(timing["n6_exact_sleep_motion_half_turn"],
                               1.0 / 6.0, places=12)
        self.assertTrue(timing["all_updates_settle_before_next_half_turn"])

    def test_optimistic_straight_direction_memory_raster_fails_exact_fit(self):
        variant = self.report["direction_memory_two_track_variant"]
        fit = variant["exact_slot_fit"]
        self.assertEqual(variant["status"], "FAIL")
        self.assertTrue(fit["pair_clearance_ok"])
        self.assertFalse(fit["core_liner_clearance_ok"])
        self.assertFalse(fit["radial_cap_ok"])
        self.assertLess(fit["center_core_margin_mm"], 0.0)
        capacity = variant["straight_lane_capacity_bound"]
        self.assertEqual(capacity["far_lane_max_centers_at_wire_pitch"], 18)
        self.assertFalse(capacity["far_lane_can_hold_25"])
        route = variant["sequential_mouth_and_r3"]
        self.assertFalse(route["r3_route_exists_both_signs"])
        for sign in ("M2_positive", "M2_negative"):
            self.assertEqual(route["both_flyer_signs"][sign]["status"],
                             "FAIL")
            self.assertEqual(route["both_flyer_signs"][sign]
                             ["first_prefilled_neighbor_failure"], 0)

    def test_honeycomb_successor_partitions_but_cam_and_r3_still_fail(self):
        successor = self.report["serpentine_honeycomb_successor"]
        partition = successor["packing_partition"]
        self.assertEqual(partition["status"], "PASS")
        self.assertEqual(len(partition["branches"]), 2)
        self.assertTrue(all(row["center_count"] == 25
                            for row in partition["branches"]))
        self.assertTrue(all(row["all_steps_one_wire_diameter"]
                            for row in partition["branches"]))
        self.assertTrue(partition["selector_transition_is_one_wire_diameter"])
        slope = successor["cam_track_slope"]
        self.assertFalse(slope["reasonable_slope_pass"])
        self.assertGreater(slope["maximum_average_pressure_angle_deg"], 80.0)
        route = successor["r3_sequential_route"]
        self.assertEqual(route["status"], "FAIL")
        self.assertEqual(route["covered_direction_cases"], 0)
        self.assertFalse(route["c1_bend_continuity"])
        self.assertTrue(successor["successor_concept_survives_static_geometry"])
        self.assertFalse(successor["production_integration_authorized"])

    def test_endpoint_load_backlash_and_retract_fail_closed(self):
        mechanics = self.report["cam_mechanics"]
        self.assertEqual(mechanics["geometry"]["endpoint_pressure_angle_deg"],
                         90.0)
        bound = mechanics["last_half_turn_quasistatic_bound"]
        self.assertLess(bound["selected_motor_margin"], 1.0)
        self.assertLess(bound["coupling_margin"], 1.0)
        self.assertEqual(bound["continuous_endpoint_force_torque_and_speed"],
                         "unbounded")
        backlash = mechanics["backlash_and_quantization"]
        self.assertFalse(backlash["physical_m0_error_is_measured"])
        retract = self.report["retract_and_index_extension"]
        self.assertFalse(retract["all_required_poses_inside_inverse_sine_domain"])
        self.assertEqual(retract["monotone_extension_status"], "FAIL")

    def test_report_hash_and_release_decision_fail_closed(self):
        self.assertEqual(self.report["schema"], study.SCHEMA)
        self.assertEqual(self.report["status"], "FAIL")
        self.assertFalse(self.report["release_authorized"])
        self.assertFalse(self.report["production_integration"]["authorized"])
        self.assertIsNone(
            self.report["production_integration"]["specification"])
        self.assertEqual(study._canonical_hash(self.report),
                         self.report["report_sha256"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
