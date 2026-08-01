"""Release-contract tests for the packed-wire route table."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import unittest

import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

from slot_route import route_packing_turn
from slot_wire_routes import (
    OUTPUT_PATH,
    PACKING_PATH,
    SCHEMA,
    validate_report,
    validate_report_integrity,
)


class SlotWireRouteReportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.packing = json.loads(PACKING_PATH.read_text())
        cls.report = json.loads(OUTPUT_PATH.read_text())

    def test_checked_geometry_is_complete_but_release_stays_blocked(self):
        validate_report_integrity(self.report, self.packing)
        self.assertEqual(self.report["schema"], SCHEMA)
        self.assertEqual(self.report["status"], "FAIL")
        validation = self.report["validation"]
        self.assertEqual(validation["generated_geometry_cases"], 100)
        self.assertEqual(validation["passed_geometry_cases"], 100)
        self.assertEqual(validation["labeled_direction_cases"], 200)
        self.assertEqual(validation["covered_direction_cases"], 0)
        self.assertFalse(validation["both_motion_signs_covered"])
        self.assertFalse(validation["release_proof_flags"][
            "current_half_sign_specific"])
        self.assertFalse(validation["release_proof_flags"][
            "c1_bend_continuity"])
        self.assertTrue(validation["release_proof_flags"][
            "exact_core_or_bound"])
        self.assertFalse(validation["release_proof_flags"][
            "physical_error_budget"])
        with self.assertRaisesRegex(ValueError, "not PASS"):
            validate_report(self.report, self.packing)

    def test_turn45_uses_exactly_postchecked_refined_support_cone(self):
        rows = [
            row for row in self.report["routes"]
            if row["turn_index"] == 45
        ]
        self.assertEqual(len(rows), 2)
        expected = {0: 23.0, 1: 337.0}
        for row in rows:
            self.assertEqual(row["status"], "PASS")
            self.assertTrue(row["progressive_support_validated"])
            attempts = row["planner_metadata"][
                "core_route_copper_fallback"
            ]["approach_attempts"]
            passed = [attempt for attempt in attempts
                      if attempt["status"] == "PASS"]
            self.assertEqual(len(passed), 1)
            self.assertEqual(
                passed[0]["direction_deg"],
                expected[row["half_turn_index"]],
            )
            self.assertEqual(passed[0]["distance_mm"], 0.5)
            self.assertTrue(all(
                row["planner_metadata"]["exact_release_postcheck"][
                    "checks"
                ].values()
            ))

    def test_every_turn_has_two_pose_rows_and_requested_sign_labels(self):
        coverage = {}
        for row in self.report["routes"]:
            key = (row["turn_index"], row["half_turn_index"])
            self.assertNotIn(key, coverage)
            coverage[key] = row
            self.assertEqual(row["validated_motion_signs"], [-1, 1])
            if row["status"] == "PASS":
                self.assertTrue(row["progressive_support_validated"])
                self.assertTrue(all(
                    row["planner_metadata"]["exact_release_postcheck"][
                        "checks"].values()))
            self.assertTrue(np.allclose(
                row["route"]["points_local_mm"][-1],
                row["target_local_mm"],
                rtol=0.0,
                atol=1e-12,
            ))
        self.assertEqual(
            set(coverage),
            {(turn, half) for turn in range(50) for half in (0, 1)},
        )

    def test_report_tampering_and_stale_packing_fail_closed(self):
        tampered = deepcopy(self.report)
        tampered["routes"][0]["target_local_mm"][0] += 0.001
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            validate_report_integrity(tampered, self.packing)

        stale = deepcopy(self.packing)
        stale["report_sha256"] = "0" * 64
        with self.assertRaises(ValueError):
            validate_report_integrity(self.report, stale)

    def test_invalid_half_turn_is_rejected_before_geometry(self):
        with self.assertRaisesRegex(ValueError, "half_turn_index"):
            route_packing_turn(None, None, None, 0, 2)  # type: ignore[arg-type]


if __name__ == "__main__":
    unittest.main()
