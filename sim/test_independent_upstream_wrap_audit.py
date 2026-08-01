"""Regression tests for the independent untouched-upstream wrap audit."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import independent_upstream_wrap_audit as audit


class IndependentUpstreamWrapAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = audit.analyze(
            audit.DEFAULT_CAPTURE,
            audit.DEFAULT_SETTINGS,
            audit.DEFAULT_WINDER,
        )

    def test_capture_is_clean_untouched_expected_upstream(self) -> None:
        self.assertTrue(self.report["gates"]["real_untouched_upstream_capture"])
        self.assertEqual(
            self.report["authority"]["winder_commit"],
            audit.EXPECTED_COMMIT,
        )

    def test_completed_raw_targets_are_not_two_turns(self) -> None:
        wraps = self.report["capture"]["wraps"]
        self.assertTrue(self.report["gates"][
            "both_raw_targets_complete_before_next_target"])
        self.assertEqual(len(wraps), 2)
        self.assertAlmostEqual(wraps[0]["completed_motor_turns"], 11 / 8, places=9)
        self.assertAlmostEqual(wraps[1]["completed_motor_turns"], 67 / 24, places=9)
        self.assertFalse(self.report["gates"][
            "both_raw_moves_exactly_two_physical_turns_direct_drive"])

    def test_one_fixed_ratio_cannot_map_both_to_two(self) -> None:
        ratios = self.report["fixed_mechanical_transmission_audit"][
            "required_abs_k_for_each_wrap"]
        self.assertAlmostEqual(ratios[0], 16 / 11, places=9)
        self.assertAlmostEqual(ratios[1], 48 / 67, places=9)
        self.assertFalse(math.isclose(ratios[0], ratios[1], abs_tol=1e-12))
        self.assertFalse(self.report["gates"][
            "fixed_affine_transmission_solution"])

    def test_no_settings_or_equivalent_pattern_solution(self) -> None:
        settings = self.report["settings_only_audit"]
        self.assertFalse(self.report["gates"]["settings_only_solution"])
        self.assertFalse(settings["balanced_winding_config"][
            "direct_drive_exact_two_possible"])
        self.assertEqual(settings["canonical_equivalence_search"][
            "variant_count"], 48)
        self.assertEqual(settings["canonical_equivalence_search"][
            "variants_with_equal_wrap_magnitudes"], 0)

    def test_documented_relative_fix_is_not_applied_and_has_timing_margin(self) -> None:
        fix = self.report["smallest_upstream_correction_suggestion"]
        self.assertFalse(fix["applied"])
        self.assertEqual(fix["predicted_turns"], [2.0, 2.0])
        self.assertGreater(fix["slack_before_m2_recenter_s"], 0.8)
        source = audit.DEFAULT_WINDER / "src" / "winding.py"
        self.assertNotIn(
            "self.move_motor(1, motor1_pos - motor1_rotation)",
            source.read_text(encoding="utf-8"),
        )


if __name__ == "__main__":
    unittest.main()
