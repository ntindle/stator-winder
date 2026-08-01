"""Regression tests for the bounded M0-following full-shroud verdict."""

from __future__ import annotations

import json
import math
from pathlib import Path
import unittest

import m0_following_full_shroud_study as study


class M0FollowingFullShroudStudyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = study.write_reports()

    def test_raw_capture_identity_and_complete_phase_coverage(self) -> None:
        raw = self.report["motion"]["raw_capture"]
        self.assertEqual(raw["sha256"], study.EXPECTED_CAPTURE_SHA256)
        self.assertEqual(raw["controller_mode"], "upstream")
        self.assertIsNone(raw["adapter_sha256"])
        self.assertEqual(raw["pass_count"], 24)
        self.assertEqual(raw["motion_sign_counts"], {-1: 12, 1: 12})
        self.assertEqual(raw["shaft_wrap_count"], 2)

    def test_m0_tracks_but_lost_motion_cannot_clear_completed_coil(self) -> None:
        motion = self.report["motion"]
        self.assertEqual(motion["status"], "FAIL")
        # The rounded raw M0 command endpoints give 6.516440 mm; the job's
        # independent contact-depth metadata spans 6.516099 mm.  Both bind the
        # same nominal 6.516 mm traverse and the study reports the actual
        # command-derived motion here.
        self.assertGreater(motion["tracking_stroke_mm"], 6.516)
        self.assertLess(motion["tracking_stroke_mm"], 6.517)
        self.assertTrue(motion["all_winding_starts_deployed"])
        self.assertFalse(motion["all_M1_moves_retracted"])
        self.assertAlmostEqual(
            motion["conservative_completed_coil_radius_mm"], 26.0,
            places=9,
        )
        self.assertAlmostEqual(
            motion["required_relative_extraction_mm"], 14.086099494243948,
            places=9,
        )
        self.assertAlmostEqual(
            motion["available_relative_extraction_mm"], 12.31986583485737,
            places=9,
        )
        self.assertAlmostEqual(
            motion["extraction_stroke_shortfall_mm"], 1.766233659386578,
            places=9,
        )
        self.assertGreater(motion["extraction_time_shortfall_s"], 0.069)
        self.assertLess(
            motion["minimum_tolerance_reserved_clearance_at_M1_motion_mm"],
            study.RIGID_CLEARANCE_MM,
        )

    def test_capacity_and_load_pass_but_rigid_envelope_fails(self) -> None:
        support = self.report["capacity_rigid_load_build"]
        self.assertEqual(support["status"], "FAIL")
        self.assertEqual(support["slot_capacity"]["status"], "PASS")
        self.assertLessEqual(
            support["slot_capacity"]["gross_slot_fill"],
            support["slot_capacity"]["hard_fill_limit"],
        )
        self.assertEqual(support["rigid_clearance"]["status"], "FAIL")
        self.assertLess(
            support["rigid_clearance"]["minimum_rigid_clearance_mm"], 2.0)
        self.assertGreaterEqual(
            support["loads_and_timing"]["revised_M0_force_margin"], 2.0)

    def test_standard_convex_R3_crown_has_exact_spacing_shortfall(self) -> None:
        check = self.report["wire_route_and_contact"]["checks"][
            "standard convex crown fits lined tooth side spacing"]
        self.assertFalse(check["ok"])
        self.assertGreater(check["shortfall_mm"], 2.5)
        self.assertLess(
            check["available_lined_side_spacing_mm"],
            check["required_for_two_R3_quarter_turns_mm"],
        )

    def test_optimistic_nonconvex_escape_crosses_completed_neighbor_sector(self) -> None:
        checks = self.report["wire_route_and_contact"]["checks"]
        sector = checks[
            "optimistic nonconvex R3 escape remains in tooth sector"]
        aggregate = checks[
            "previous aggregate can be independently cleared"]
        self.assertFalse(sector["ok"])
        self.assertGreater(sector["sector_intrusion_mm"], 2.3)
        self.assertFalse(aggregate["ok"])
        self.assertEqual(
            aggregate["passes_with_at_least_one_completed_neighbor"], 22)
        self.assertEqual(aggregate["passes_with_both_completed_neighbors"], 2)

    def test_first_turn_loses_R3_support_after_extraction(self) -> None:
        check = self.report["wire_route_and_contact"]["checks"][
            "withdrawn first turn retains physical R3 support"]
        self.assertFalse(check["ok"])
        self.assertAlmostEqual(
            check["supported_corner_after_full_extraction_mm"], 0.23876,
            places=8,
        )
        self.assertGreater(check["shortfall_mm"], 2.76)

    def test_fail_closed_verdict_prohibits_CAD_and_integration(self) -> None:
        self.assertEqual(self.report["schema"], study.SCHEMA)
        self.assertEqual(self.report["status"], "DESIGN_NO_GO")
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["integration_authorized"])
        self.assertFalse(self.report["isolated_CAD_authorized"])
        self.assertFalse(self.report["scope"]["CAD_generated"])
        self.assertIn(
            "all_raw_M1_and_shaft_wrap_motion_shroud_retracted",
            self.report["controlling_failures"],
        )
        self.assertIn("M0_tracking_extraction_timing_and_clearance",
                      self.report["controlling_failures"])
        self.assertIn("two_millimetre_rigid_clearance",
                      self.report["controlling_failures"])
        self.assertIn("standard_full_shroud_R3_path",
                      self.report["controlling_failures"])

    def test_checked_in_report_matches_fresh_canonical_hash(self) -> None:
        stored = json.loads(study.JSON_OUT.read_text(encoding="utf-8"))
        self.assertEqual(stored["report_sha256"], self.report["report_sha256"])
        self.assertEqual(
            self.report["report_sha256"], study._canonical_hash(self.report))
        self.assertTrue(all(
            isinstance(value, str) and len(value) == 64
            for value in self.report["source_hashes"].values()
        ))


if __name__ == "__main__":
    unittest.main()
