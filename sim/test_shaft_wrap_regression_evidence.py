"""Regression tests for the standalone shaft-wrap evidence bundle."""

from __future__ import annotations

import hashlib
import json
import math
import subprocess
import sys
import unittest
from pathlib import Path


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import shaft_wrap_regression_evidence as evidence  # noqa: E402


class ShaftWrapRegressionEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.before_source_sha = hashlib.sha256(
            (evidence.DEFAULT_WINDER / "src" / "winding.py").read_bytes()
        ).hexdigest()
        cls.before_status = subprocess.check_output(
            ["git", "-C", str(evidence.DEFAULT_WINDER), "status", "--porcelain"],
            text=True,
        )
        cls.report, cls.patch = evidence.analyze()

    @classmethod
    def tearDownClass(cls) -> None:
        after_source_sha = hashlib.sha256(
            (evidence.DEFAULT_WINDER / "src" / "winding.py").read_bytes()
        ).hexdigest()
        after_status = subprocess.check_output(
            ["git", "-C", str(evidence.DEFAULT_WINDER), "status", "--porcelain"],
            text=True,
        )
        if after_source_sha != cls.before_source_sha or after_status != cls.before_status:
            raise AssertionError("standalone evidence audit modified upstream")

    def test_current_commit_source_and_raw_capture_are_bound(self) -> None:
        current = self.report["current_upstream"]
        raw = self.report["current_raw_capture"]
        self.assertEqual(current["commit"], evidence.CURRENT_COMMIT)
        self.assertTrue(current["worktree_clean"])
        self.assertTrue(current["uses_bookkeeping_zero_targets"])
        self.assertFalse(current["queries_live_m1_inside_wrap"])
        self.assertEqual(raw["winder_commit"], evidence.CURRENT_COMMIT)
        self.assertTrue(raw["matches_expected_regression"])
        self.assertAlmostEqual(raw["observed_turns"][0], 1.375, places=9)
        self.assertAlmostEqual(
            raw["observed_turns"][1], 2.7916666666666665, places=9,
        )

    def test_independent_serial_position_evidence_is_hash_bound(self) -> None:
        serial = self.report["independent_serial_position_evidence"]
        self.assertEqual(serial["transport"], "serial_position_digital_twin")
        self.assertFalse(serial["upstream_source_subclassed"])
        self.assertFalse(serial["upstream_source_modified_by_harness"])
        self.assertEqual(
            serial["winding_source_sha256"],
            self.report["current_upstream"]["source_sha256"],
        )
        self.assertEqual(
            serial["capture_harness_sha256"], serial["harness_sha256"],
        )
        self.assertTrue(serial["evidence_bound_to_current_source_and_harness"])
        self.assertTrue(serial["matches_expected_regression"])
        self.assertAlmostEqual(
            serial["observed_turns"][0], evidence.SERIAL_EXPECTED_TURNS[0],
            places=12,
        )
        self.assertAlmostEqual(
            serial["observed_turns"][1], evidence.SERIAL_EXPECTED_TURNS[1],
            places=12,
        )

    def test_pre_regression_parent_source_proves_two_turn_request(self) -> None:
        historical = self.report["pre_regression_two_turn_source_evidence"]
        boundary = self.report["regression_boundary"]
        self.assertEqual(boundary["first_bad_parent"], evidence.PRE_REGRESSION_COMMIT)
        self.assertTrue(boundary["parent_is_pre_regression_commit"])
        self.assertTrue(historical["queries_live_m1"])
        self.assertTrue(historical["forms_four_pi"])
        self.assertTrue(historical["commands_from_live_position"])
        self.assertTrue(math.isclose(
            historical["requested_delta_turns"], 2.0,
            rel_tol=0.0, abs_tol=1.0e-15,
        ))
        self.assertLess(historical["serial_command_turn_error_max"], 0.00008)

    def test_review_patch_is_minimal_applicable_and_not_applied(self) -> None:
        patch = self.report["review_only_patch"]
        self.assertFalse(patch["applied"])
        self.assertTrue(patch["git_apply_check_pass"])
        self.assertTrue(patch["restores_live_position_query"])
        self.assertTrue(patch["restores_live_position_minus_four_pi"])
        self.assertTrue(patch["restores_live_position_plus_four_pi"])
        self.assertTrue(patch["adds_two_start_angle_regression_cases"])
        self.assertEqual(
            patch["changed_files"], ["src/winding.py", "tests/test_winding.py"],
        )
        self.assertEqual(self.patch.count("motor1_pos = self.get_motor_position(1)"), 1)
        self.assertEqual(self.patch.count("motor1_pos - motor1_rotation"), 1)
        self.assertEqual(self.patch.count("motor1_pos + motor1_rotation"), 1)
        self.assertIn(
            "def test_shaft_wrap_is_two_turns_from_live_m1(", self.patch,
        )
        self.assertIn("-3.926990817", self.patch)
        self.assertIn("-17.540558983", self.patch)
        source = (evidence.DEFAULT_WINDER / "src" / "winding.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn(
            "self.move_motor(1, motor1_pos - motor1_rotation)", source,
        )
        self.assertNotIn(
            "self.move_motor(1, motor1_pos + motor1_rotation)", source,
        )

    def test_bundle_is_complete_but_release_stays_fail_closed(self) -> None:
        self.assertTrue(self.report["evidence_bundle_complete"])
        self.assertEqual(self.report["status"], "FAIL_CLOSED")
        self.assertFalse(self.report["release_authority"])
        self.assertFalse(self.report["gates"][
            "current_upstream_satisfies_two_turn_requirement"
        ])
        self.assertTrue(self.report["gates"][
            "review_patch_contains_focused_regression_test"
        ])
        self.assertFalse(self.report["gates"]["release_authorized"])

    def test_generated_report_and_patch_match_live_analysis(self) -> None:
        generated = json.loads(evidence.DEFAULT_JSON.read_text(encoding="utf-8"))
        generated_patch = evidence.DEFAULT_PATCH.read_text(encoding="utf-8")
        self.assertEqual(generated["schema"], self.report["schema"])
        self.assertEqual(
            generated["current_upstream"]["source_sha256"],
            self.report["current_upstream"]["source_sha256"],
        )
        self.assertEqual(
            generated["current_raw_capture"]["sha256"],
            self.report["current_raw_capture"]["sha256"],
        )
        self.assertEqual(
            generated["independent_serial_position_evidence"]["sha256"],
            self.report["independent_serial_position_evidence"]["sha256"],
        )
        self.assertEqual(generated_patch, self.patch)
        self.assertEqual(
            hashlib.sha256(generated_patch.encode("utf-8")).hexdigest(),
            generated["review_only_patch"]["sha256"],
        )


if __name__ == "__main__":
    unittest.main()
