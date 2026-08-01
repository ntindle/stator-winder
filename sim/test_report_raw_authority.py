"""Regression tests for selected Definition-of-Done evidence authority."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest
from unittest import mock


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import report  # noqa: E402


def by_label(checks, label):
    return next(row for row in checks if row["label"] == label)


class SelectedAuthorityReportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cycle, _ = report._read_json(report.RAW_CYCLE)
        cls.wrap, _ = report._read_json(report.SHAFT_WRAP_REGRESSION)
        cls.manifest, _ = report._read_json(report.SELECTED_MANIFEST)
        cls.player_render, _ = report._read_json(report.SELECTED_PLAYER_RENDER)
        cls.candidate, _ = report._read_json(report.INTEGRATED_CANDIDATE)
        cls.rigid, _ = report._read_json(report.ACTIVE_RIGID_AUDIT)
        cls.conductor, _ = report._read_json(
            report.FULL_CONDUCTOR_AUTHORITY,
        )
        cls.launch, _ = report._read_json(report.LAUNCH_AUTHORITY)
        cls.catalog, _ = report._read_json(report.RELEASE_CATALOG)
        cls.events, cls.meta, cls.capture_error = report._read_capture()
        cls.player_bundle, cls.player_error = report._read_player_bundle()

    def test_raw_cycle_remains_required_and_rejects_wrap_regression(self):
        checks = report._cycle_gate(
            self.cycle, None, self.capture_error, self.meta,
        )
        self.assertTrue(by_label(
            checks, "capture is unmodified upstream authority",
        )["ok"])
        self.assertFalse(by_label(
            checks, "raw cycle declares the supported PASS schema",
        )["ok"])
        self.assertTrue(by_label(
            checks, "raw cycle is bound to the canonical capture",
        )["ok"])
        self.assertFalse(by_label(
            checks, "captured-cycle verifier passes every explicit check",
        )["ok"])

        tampered = deepcopy(self.cycle)
        tampered["capture"]["sha256"] = "0" * 64
        checks = report._cycle_gate(
            tampered, None, self.capture_error, self.meta,
        )
        self.assertFalse(by_label(
            checks, "raw cycle is bound to the canonical capture",
        )["ok"])

    def test_separate_wrap_bundle_is_current_but_release_fails_closed(self):
        checks = report._shaft_wrap_compatibility_gate(
            self.wrap, None, self.meta,
        )
        self.assertTrue(by_label(
            checks, "shaft-wrap regression integrity gates pass",
        )["ok"])
        self.assertFalse(by_label(
            checks,
            "untouched upstream executes exactly two turns in both shaft wraps",
        )["ok"])

        tampered = deepcopy(self.wrap)
        tampered["current_raw_capture"]["sha256"] = "0" * 64
        checks = report._shaft_wrap_compatibility_gate(
            tampered, None, self.meta,
        )
        self.assertFalse(by_label(
            checks, "shaft-wrap evidence is bound to the canonical raw capture",
        )["ok"])

    def test_integrated_v3_player_is_the_current_animation_authority(self):
        self.assertIsNone(self.player_error)
        with mock.patch.object(
            report, "_read_player_bundle",
            return_value=(self.player_bundle, None),
        ):
            checks = report._integrated_player_gate(
                self.manifest, None, self.player_render, None,
                self.catalog, None,
            )
        self.assertTrue(report._gate(checks))
        self.assertTrue(by_label(
            checks,
            "selected integrated player contains the complete four-axis cycle",
        )["ok"])
        self.assertEqual(self.player_bundle["state"]["schema"], "winder-player/v3")
        self.assertEqual(len(self.player_bundle["state"]["coilStarts"]), 24)
        self.assertEqual(len(self.player_bundle["state"]["halfTurns"]), 2400)

        tampered = deepcopy(self.player_render)
        tampered["capture_sha256"] = "0" * 64
        with mock.patch.object(
            report, "_read_player_bundle",
            return_value=(self.player_bundle, None),
        ):
            checks = report._integrated_player_gate(
                self.manifest, None, tampered, None, self.catalog, None,
            )
        self.assertFalse(by_label(
            checks, "selected integrated player render hash closure is current",
        )["ok"])

    def test_selected_rigid_geometry_is_gating_not_legacy_clearance(self):
        checks = report._interference_gate(
            self.candidate, None, self.rigid, None, self.catalog, None,
        )
        self.assertTrue(by_label(
            checks,
            "selected integrated candidate passes every geometry check",
        )["ok"])
        self.assertTrue(by_label(
            checks,
            "full raw selected-rigid motion covers every required class",
        )["ok"])
        self.assertTrue(by_label(
            checks, "all selected rigid clearance pairs meet the 2 mm target",
        )["ok"])
        labels = "\n".join(row["label"] for row in checks).lower()
        self.assertNotIn("canonical raw dynamic-clearance", labels)
        self.assertNotIn("same-link static audit", labels)
        self.assertNotIn("focused m2 belt audit", labels)

        tampered = deepcopy(self.rigid)
        tampered["full_raw_rigid_motion"][
            "required_motion_classes_present"
        ]["shaft_wrap"] = False
        checks = report._interference_gate(
            self.candidate, None, tampered, None, self.catalog, None,
        )
        self.assertFalse(by_label(
            checks,
            "full raw selected-rigid motion covers every required class",
        )["ok"])

    def test_launch_matrix_is_current_but_zero_of_24_fails_dod2(self):
        checks = report._launch_coverage_gate(
            self.launch, None, self.catalog, None,
        )
        self.assertTrue(by_label(
            checks,
            "all 24 launch certificates have current source/artifact closure",
        )["ok"])
        self.assertFalse(by_label(
            checks, "all required OD28--65 launch corners are authorized",
        )["ok"])

        tampered = deepcopy(self.launch)
        tampered["required_certificates"][0]["certificate_sha256"] = "0" * 64
        checks = report._launch_coverage_gate(
            tampered, None, self.catalog, None,
        )
        self.assertFalse(by_label(
            checks,
            "all 24 launch certificates have current source/artifact closure",
        )["ok"])

    def test_full_cycle_conductor_is_bound_and_remains_fail_closed(self):
        checks = report._wire_gate(
            self.conductor, None, self.catalog, None,
        )
        self.assertTrue(by_label(
            checks, "continuous-conductor report hash is valid",
        )["ok"])
        self.assertTrue(by_label(
            checks,
            "continuous-conductor authority binds selected integrated inputs",
        )["ok"])
        self.assertFalse(by_label(
            checks, "full-cycle flexible conductor is production-authorized",
        )["ok"])
        labels = "\n".join(row["label"] for row in checks).lower()
        self.assertNotIn("canonical raw wire-path", labels)
        self.assertNotIn("winding-tooling authority", labels)

        tampered = deepcopy(self.conductor)
        tampered["input_file_sha256"]["loci"] = "0" * 64
        checks = report._wire_gate(
            tampered, None, self.catalog, None,
        )
        self.assertFalse(by_label(
            checks,
            "continuous-conductor authority input hash closure is current",
        )["ok"])
        self.assertFalse(by_label(
            checks,
            "continuous-conductor authority binds selected integrated inputs",
        )["ok"])

    def test_validation_manifest_separates_governing_and_baseline_evidence(self):
        validation = json.loads(
            (report.REPORTS / "validation.json").read_text(encoding="utf-8")
        )
        self.assertEqual(validation["schema"], report.VALIDATION_SCHEMA)
        evidence = validation["evidence"]
        authority = validation["evidence_authority"]
        self.assertEqual(
            evidence["selected_animation_player"],
            "out/review/integrated_adapter/play_integrated_candidate_raw.html",
        )
        self.assertEqual(
            evidence["selected_active_rigid_audit"],
            "out/reports/carriage_active_sector_terminal_guide_audit.json",
        )
        self.assertEqual(
            evidence["full_cycle_conductor_authority"],
            "out/reports/full_cycle_continuous_conductor_authority_audit.json",
        )
        self.assertEqual(
            evidence["launch_envelope_authority"],
            "out/reports/launch_envelope_authority.json",
        )
        self.assertEqual(authority["selected_animation_player"], "GOVERNING")
        self.assertEqual(
            authority["legacy_raw_clearance"], "BASELINE_DIAGNOSTIC_ONLY",
        )
        self.assertEqual(
            authority["legacy_raw_wirepath"], "BASELINE_DIAGNOSTIC_ONLY",
        )
        self.assertEqual(
            authority["legacy_raw_animation_player"],
            "BASELINE_DIAGNOSTIC_ONLY",
        )
        source_hashes = validation["source_hashes"]
        self.assertNotIn(evidence["legacy_raw_clearance"], source_hashes)
        self.assertNotIn(evidence["legacy_raw_wirepath"], source_hashes)
        self.assertNotIn(evidence["legacy_raw_animation_player"], source_hashes)
        self.assertEqual(
            validation["baseline_diagnostics"]["authority"], "NON_GOVERNING",
        )
        self.assertNotIn("clearance", evidence)
        self.assertNotIn("wirepath", evidence)
        self.assertNotIn("animation_player", evidence)

    def test_report_source_does_not_gate_dod1_to_3_on_legacy_outputs(self):
        source = Path(report.__file__).read_text(encoding="utf-8")
        self.assertNotIn("_interference_gate(clearance", source)
        self.assertNotIn("_wire_gate(wire, wire_error", source)
        self.assertNotIn("matches current raw capture and CAD", source)
        self.assertIn('"legacy_raw_clearance": RAW_CLEARANCE', source)
        self.assertIn('"legacy_raw_wirepath": RAW_WIREPATH', source)
        self.assertIn('"legacy_raw_animation_player": RAW_PLAYER', source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
