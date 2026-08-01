"""Fail-closed regressions for the M2-cammed alternating-former trade."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import unittest
from unittest import mock


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import m2_cammed_alternating_former_study as study


REPORT = (
    Path(__file__).resolve().parents[1]
    / "out" / "reports" / "m2_cammed_alternating_former.json"
)


def _report():
    return json.loads(REPORT.read_text(encoding="utf-8"))


class M2CammedAlternatingFormerStudyTests(unittest.TestCase):

    def test_report_is_hashed_and_fails_closed(self):
        report = _report()
        expected = report.pop("report_sha256")
        canonical = json.dumps(
            report, sort_keys=True, separators=(",", ":"), allow_nan=False)
        self.assertEqual(hashlib.sha256(canonical.encode()).hexdigest(), expected)
        self.assertEqual(report["schema"], study.SCHEMA)
        self.assertEqual(report["status"], "DESIGN_NO_GO")
        self.assertIs(report["release_authorized"], False)
        self.assertIs(report["assembly_integration_authorized"], False)

    def test_raw_upstream_capture_is_complete_and_needs_three_laws(self):
        capture = _report()["capture_contract"]
        self.assertEqual(capture["controller_mode"], "upstream")
        self.assertEqual(capture["axis_velocities_rad_s"], [20.0, 20.0, 20.0, 5.0])
        self.assertEqual(capture["winding_pass_count"], 24)
        self.assertEqual(capture["turns_per_tooth"], 50)
        self.assertIs(capture["packing_waypoint_events_present"], False)
        self.assertIs(capture["packing_plan_used_as_motion_authority"], False)
        self.assertEqual(capture["M2_winding_commands_per_pass"], 13)
        self.assertEqual(capture["total_M2_winding_commands"], 24 * 13)
        self.assertEqual(capture["directions"], [-1, 1])
        self.assertEqual(
            capture["physical_crossing_origin_classes_deg"],
            [0, 180],
        )
        self.assertEqual(capture["unique_cam_law_count"], 3)
        self.assertIs(capture["m1_index_uniquely_selects_cam_law"], True)
        self.assertEqual(len(capture["m1_index_to_law"]), 24)

    def test_exact_packed_loops_require_four_mutually_exclusive_R3_states(self):
        geometry = _report()["necessary_support_geometry"]
        self.assertEqual(len(geometry["necessary_former_demands"]), 4)
        self.assertTrue(all(
            row["all_50_turns"]
            for row in geometry["necessary_former_demands"]))
        self.assertGreaterEqual(
            geometry["nominal_wire_center_contact_radius_mm"], 3.0)
        overlap = geometry["simultaneous_pair_boundary"]
        self.assertIs(overlap["all_50_turns_overlap"], True)
        self.assertGreater(overlap["overlap_range_mm"][0], 1.0)
        self.assertGreater(overlap["overlap_range_mm"][1], 2.0)

    def test_one_stationary_ring_aliases_same_direction_passes(self):
        alias = _report()["stationary_cam_phase_alias"]
        self.assertEqual(alias["status"], "FAIL")
        self.assertGreaterEqual(alias["conflicting_physical_angle_count"], 4)
        witness = alias["decisive_same_direction_witness"]
        self.assertEqual(witness["physical_m2_angle_deg"], 150)
        self.assertEqual(witness["first"]["pass_index"], 0)
        self.assertEqual(witness["second"]["pass_index"], 2)
        self.assertEqual(witness["first"]["direction"], -1)
        self.assertEqual(witness["second"]["direction"], -1)
        self.assertNotEqual(
            witness["first"]["required_finger"],
            witness["second"]["required_finger"],
        )
        self.assertIs(alias["one_stationary_cam_law_satisfies_capture"], False)

    def test_recessed_ring_envelope_passes_only_necessary_rigid_clearance(self):
        rigid = _report()["candidate_ring_rigid_envelope"]
        self.assertEqual(rigid["status"], "PASS_NECESSARY_ENVELOPE_ONLY")
        self.assertGreaterEqual(rigid["minimum_flyer_clearance_mm"], 2.0)
        self.assertGreaterEqual(
            rigid["minimum_final_wound_chuck_clearance_mm"], 2.0)
        self.assertGreaterEqual(rigid["minimum_other_static_clearance_mm"], 2.0)
        self.assertGreater(len(rigid["not_included"]), 0)

    def test_contact_policy_does_not_invent_a_frozen_access_gate(self):
        route = _report()["exact_wire_route_context"]
        self.assertEqual(route["current_route_status"], "FAIL")
        self.assertEqual(route["evaluated_crossing_routes"], 100)
        self.assertEqual(route["current_rigid_geometry_status"], "PASS")
        self.assertEqual(route["current_rigid_geometry_pass_count"], 100)
        self.assertEqual(route["current_failure_cases"], [])
        self.assertEqual(
            set(route["report_level_release_blockers"]),
            {
                "current_half_sign_specific",
                "c1_bend_continuity",
                "physical_error_budget",
            },
        )
        self.assertEqual(route["current_free_approach_shortfall_mm"], 0.0)
        self.assertGreater(route["current_turn_45_parent_prefix_margin_mm"], 0.0)
        self.assertIn("intentional support/contact", route["contact_policy"])
        self.assertEqual(
            route["former_supported_route"],
            "NOT_EVALUATED_AFTER_CAM_PHASE_FAILURE",
        )
        self.assertEqual(
            _report()["minimum_mechanical_escape"]["status"],
            "CONCEPT_ONLY_NOT_PROVEN",
        )

    def test_route_context_rejects_a_missing_false_proof_flag(self):
        routes = json.loads(study.ROUTES.read_text(encoding="utf-8"))
        broken = deepcopy(routes)
        del broken["validation"]["release_proof_flags"]["c1_bend_continuity"]
        with mock.patch.object(study, "_load", return_value=broken):
            with self.assertRaisesRegex(
                    ValueError, "geometry/proof split drifted"):
                study.route_context()

    def test_all_bound_sources_are_current(self):
        root = Path(__file__).resolve().parents[1]
        for relative, expected in _report()["source_hashes"].items():
            actual = hashlib.sha256((root / relative).read_bytes()).hexdigest()
            self.assertEqual(actual, expected, relative)


if __name__ == "__main__":
    unittest.main()
