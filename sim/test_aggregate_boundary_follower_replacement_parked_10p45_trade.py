"""Focused report tests for the isolated parked-|Y|=10.45 trade."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_replacement_parked_10p45_trade as trade


class AggregateBoundaryFollowerReplacementParked10p45TradeTests(
    unittest.TestCase,
):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = json.loads(trade.OUTPUT_JSON.read_text(encoding="utf-8"))

    def test_report_is_current_self_hashed_and_dense_enough(self):
        trade.validate_report_integrity(self.report)
        candidate = self.report["trade_candidate"]
        sampling = self.report["sampling"]
        self.assertEqual(candidate["candidate_parked_base_abs_y_mm"], 10.45)
        self.assertEqual(candidate["active_base_abs_y_mm"], 2.05)
        self.assertAlmostEqual(candidate["candidate_coarse_stroke_mm"], 8.40)
        self.assertEqual(candidate["coarse_subdivisions"], 17)
        self.assertLessEqual(
            sampling["maximum_independent_translation_step_mm"], 0.5,
        )
        self.assertEqual(sampling["sample_count_per_identity"], 57)
        self.assertEqual(sampling["total_pose_count"], 228)
        self.assertFalse(candidate["release_modified"])

    def test_no_authority_or_integration_is_promoted(self):
        self.assertTrue(all(not value for value in self.report["authority"].values()))
        self.assertFalse(self.report["trade_candidate"]["carrier_CAD_modified"])
        self.assertFalse(self.report["trade_candidate"]["assembly_modified"])
        self.assertFalse(self.report["trade_candidate"]["release_modified"])
        self.assertFalse(self.report["integration"]["BOM_modified"])

    def test_trade_decision_matches_exact_collision_and_clearance_result(self):
        collision = self.report["collision_audit"]
        clearance = self.report["clearance_audit"]
        result = self.report["trade_result"]
        expected = (
            collision["positive_failure_count"] == 0
            and clearance["passes_2p00mm_gate"] is True
        )
        self.assertEqual(result["restores_full_2p00mm_gate"], expected)
        if expected:
            self.assertGreaterEqual(
                clearance["minimum_sampled_exact_clearance_mm"],
                2.0 - clearance["tolerance_mm"],
            )
            self.assertFalse(result["meets_selected_nominal_reserve"])
            self.assertFalse(result["selected_for_redesign"])
            self.assertIn("SELECT_10P95", result["recommendation"])
        else:
            self.assertEqual(self.report["status"], "FAIL_CLOSED")


if __name__ == "__main__":
    unittest.main(verbosity=2)
