"""Fail-closed contract tests for the passive cam slot-guide study."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import passive_cam_slot_guide_study as study


REPORT = (
    Path(__file__).resolve().parents[1]
    / "out" / "reports" / "passive_cam_slot_guide.json"
)


def _report():
    return json.loads(REPORT.read_text(encoding="utf-8"))


class PassiveCamSlotGuideStudyTests(unittest.TestCase):

    def test_report_hash_and_fail_closed_decision(self):
        report = _report()
        expected = report.pop("report_sha256")
        canonical = json.dumps(
            report, sort_keys=True, separators=(",", ":"), allow_nan=False
        )
        self.assertEqual(
            hashlib.sha256(canonical.encode()).hexdigest(), expected)
        self.assertEqual(report["schema"], study.SCHEMA)
        self.assertEqual(report["status"], "DESIGN_NO_GO")
        self.assertIs(report["release_authorized"], False)
        self.assertIs(report["assembly_integration_authorized"], False)

    def test_existing_axis_motion_is_unambiguous_and_retracted(self):
        motion = _report()["motion_synchronization"]
        self.assertEqual(motion["status"], "PASS")
        self.assertEqual(
            motion["selected_input"], "M0 only; M2 deliberately unused")
        self.assertIs(motion["M0_state_mapping_single_valued"], True)
        self.assertEqual(motion["state_at_M1_index"], "RETRACTED")
        self.assertEqual(motion["state_at_shaft_wrap"], "RETRACTED")
        self.assertGreaterEqual(
            motion["index_pose_radial_clearance_to_final_wound_radius_mm"],
            2.0,
        )
        self.assertGreaterEqual(
            motion["shaft_wrap_pose_radial_clearance_to_final_wound_radius_mm"],
            2.0,
        )
        self.assertEqual(
            motion["M2_memoryless_cam_rejected"][
                "repetitions_per_tooth_pass"
            ],
            50,
        )

    def test_mouth_and_manufacturing_contracts_pass_before_route_gate(self):
        report = _report()
        self.assertEqual(report["mouth_corridor"]["status"], "PASS")
        self.assertIs(
            report["mouth_corridor"]["source_matches_selected"], True)
        self.assertIs(report["horn_contract"]["mouth_only"], True)
        self.assertIs(report["horn_contract"]["wire_center_R3"], True)
        self.assertEqual(
            report["manufacturing_error_budget"]["status"], "PASS")
        self.assertGreaterEqual(
            report["manufacturing_error_budget"]["residual_each_side_mm"],
            0.05,
        )
        self.assertEqual(report["force_budget"]["status"], "PASS")

    def test_all_turns_both_signs_and_current_half_are_covered(self):
        route = _report()["progressive_current_half_route"]
        self.assertEqual(route["turns"], 50)
        self.assertEqual(route["motion_signs"], [-1, 1])
        self.assertEqual(route["phases_per_turn"], 72)
        self.assertEqual(route["expected_case_count"], 50 * 2 * 72)
        self.assertEqual(
            route["evaluated_case_count"], route["expected_case_count"])
        self.assertIs(route["coverage_complete"], True)
        self.assertIs(route["both_motion_signs_covered"], True)
        self.assertIs(route["progressive_turns_covered"], True)
        self.assertEqual(
            route["current_prefix_applicable_case_count"], 50 * 2 * 71)
        self.assertGreater(route["core_liner_failure_count"], 0)
        self.assertGreater(
            route["prior_and_neighbor_copper_failure_count"], 0)
        self.assertGreater(
            route["already_laid_current_half_failure_count"], 0)
        self.assertLess(
            route["complete_passing_case_count"],
            route["evaluated_case_count"],
        )
        self.assertEqual(route["status"], "FAIL")


if __name__ == "__main__":
    unittest.main()
