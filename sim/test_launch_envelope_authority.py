"""Focused tests for the fail-closed launch-envelope authority matrix."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import launch_envelope_authority as authority  # noqa: E402


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class LaunchEnvelopeAuthorityTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name) / "machine"
        self.root.mkdir(parents=True)
        for relative in authority.REQUIRED_SOURCE_PATHS:
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(f"source fixture: {relative}\n", encoding="utf-8")

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _write_json(self, path: Path, value: dict) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(value, sort_keys=True) + "\n", encoding="utf-8",
        )

    def _artifact_row(self, path: Path, job: dict) -> dict:
        return {
            "path": path.relative_to(self.root).as_posix(),
            "sha256": _sha(path),
            "job_identity": deepcopy(job),
        }

    def _create_valid_corner(self, case: authority.LaunchCase) -> Path:
        bundle = self.root / "out" / "launch_certificates" / case.case_id
        bundle.mkdir(parents=True, exist_ok=True)
        job = case.expected_job()
        certificate_job = {**job, "turns_per_tooth": 1}

        settings_path = bundle / "settings.yml"
        self._write_json(settings_path, {
            "job": {**job, "hardware_motion_authorized": False},
            "winding": {"turns": 1},
        })
        capture_path = bundle / "capture.jsonl"
        capture_meta = {
            "t": 0.0,
            "e": "meta",
            "capture_schema": 4,
            "controller_mode": "upstream",
            "winder_commit": "0123456789abcdef",
            "winder_dirty": False,
            "upstream_source_modified_by_harness": False,
            "settings_sha256": _sha(settings_path),
            "job": certificate_job,
        }
        capture_path.write_text(
            json.dumps(capture_meta, sort_keys=True) + "\n"
            + json.dumps({"t": 1.0, "e": "cycle_complete"}) + "\n",
            encoding="utf-8",
        )

        step_path = bundle / "assembly.step"
        step_path.write_text(
            "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n",
            encoding="ascii",
        )
        packing_path = bundle / "packing.json"
        self._write_json(packing_path, {
            "status": "PASS",
            "config": job,
        })
        report_paths: dict[str, Path] = {}
        for key in ("collision", "wire", "load", "buildability"):
            path = bundle / f"{key}.json"
            self._write_json(path, {
                "status": "PASS",
                "job": job,
                "certificate_case_id": case.case_id,
            })
            report_paths[key] = path

        paths = {
            "capture": capture_path,
            "step": step_path,
            "settings": settings_path,
            "packing": packing_path,
            **report_paths,
        }
        certificate = {
            "schema": authority.CORNER_CERTIFICATE_SCHEMA,
            "case_id": case.case_id,
            "status": "PASS",
            "corner_authorized": True,
            "production_authorized": False,
            "source_dependency_closure_complete": True,
            "job": certificate_job,
            "sources": {
                relative: _sha(self.root / relative)
                for relative in authority.REQUIRED_SOURCE_PATHS
            },
            "artifacts": {
                key: self._artifact_row(path, job)
                for key, path in paths.items()
            },
            "verdicts": deepcopy(authority.REQUIRED_VERDICT_CONTRACT),
        }
        certificate["certificate_payload_sha256"] = (
            authority._canonical_hash(certificate)
        )
        certificate_path = bundle / "certificate.json"
        self._write_json(certificate_path, certificate)
        return certificate_path

    def _create_current_blocked_evidence(
        self, case: authority.LaunchCase,
    ) -> Path:
        bundle = self.root / "out" / "launch_certificates" / case.case_id
        bundle.mkdir(parents=True, exist_ok=True)
        job = {**case.expected_job(), "turns_per_tooth": 1}
        paths = {}
        step = bundle / "job_geometry.step"
        step.write_text(
            "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n",
            encoding="ascii",
        )
        paths["step"] = step
        for key, status in (
            ("capture", "BLOCKED"),
            ("settings", "FAIL"),
            ("packing", "FAIL"),
            ("collision", "BLOCKED"),
            ("wire", "BLOCKED"),
            ("load", "BLOCKED"),
            ("buildability", "PASS"),
        ):
            path = bundle / f"{key}.json"
            report = {"status": status, "job": job}
            report["report_sha256"] = authority._canonical_hash(report)
            self._write_json(path, report)
            paths[key] = path
        verdicts = {
            key: {"status": status}
            for key, status in (
                ("capture", "BLOCKED"),
                ("step", "FAIL"),
                ("settings", "FAIL"),
                ("packing", "FAIL"),
                ("collision", "BLOCKED"),
                ("wire", "BLOCKED"),
                ("load", "BLOCKED"),
                ("buildability", "PASS"),
            )
        }
        blockers = sorted(
            key for key, row in verdicts.items()
            if row["status"] != "PASS"
        )
        certificate = {
            "schema": authority.CORNER_EVIDENCE_SCHEMA,
            "case_id": case.case_id,
            "status": "FAIL_CLOSED",
            "corner_authorized": False,
            "production_authorized": False,
            "source_dependency_closure_complete": True,
            "job": job,
            "sources": {
                relative: _sha(self.root / relative)
                for relative in authority.REQUIRED_SOURCE_PATHS
            },
            "artifacts": {
                key: self._artifact_row(path, job)
                for key, path in paths.items()
            },
            "verdicts": verdicts,
            "blocking_gates": blockers,
        }
        certificate["certificate_payload_sha256"] = (
            authority._canonical_hash(certificate)
        )
        certificate_path = bundle / "certificate.json"
        self._write_json(certificate_path, certificate)
        return certificate_path

    def test_required_matrix_is_four_by_two_by_three(self) -> None:
        cases = authority.required_launch_cases()
        self.assertEqual(len(cases), 24)
        self.assertEqual(len({case.case_id for case in cases}), 24)
        self.assertEqual({case.od_mm for case in cases}, {28.0, 65.0})
        self.assertEqual({case.stack_mm for case in cases}, {5.0, 20.0})
        self.assertEqual(
            {case.wire_finished_d_mm for case in cases}, {0.2, 0.5},
        )
        self.assertEqual(
            {(case.spindle_id, case.shaft_d_mm) for case in cases},
            {("er11", 3.0), ("er11", 7.0), ("shaft8", 8.0)},
        )
        self.assertEqual(
            {case.slots for case in cases if case.od_mm == 28.0}, {12},
        )
        self.assertEqual(
            {case.slots for case in cases if case.od_mm == 65.0}, {24},
        )
        self.assertTrue(all(
            len(case.winding_config) == case.slots
            and len(case.winding_config) % 3 == 0
            for case in cases
        ))
        self.assertEqual(
            {case.upstream_config_id for case in cases if case.od_mm == 28.0},
            {authority.UPSTREAM_12N14P_CONFIG_ID},
        )
        self.assertEqual(
            {
                (case.spindle_id, case.od_mm * case.hub_od_ratio)
                for case in cases if case.od_mm == 28.0
            },
            {("er11", 19.5), ("shaft8", 16.0)},
        )
        self.assertTrue(all(
            case.reach_reserve_mm
            == authority.REPRESENTATIVE_REACH_RESERVE_MM
            for case in cases
        ))
        self.assertNotIn(46.0, {case.od_mm for case in cases})

    def test_absent_certificates_fail_closed_with_actionable_matrix(self) -> None:
        report = authority.analyze(root=self.root)
        self.assertEqual(report["status"], "FAIL")
        self.assertFalse(report["production_authorized"])
        self.assertEqual(report["summary"], {
            "required": 24,
            "passing": 0,
            "missing": 24,
            "invalid_or_stale": 0,
        })
        self.assertEqual(len(report["actionable_missing_matrix"]), 24)
        self.assertTrue(all(
            row["status"] == "MISSING_CERTIFICATE"
            for row in report["required_certificates"]
        ))
        self.assertFalse(
            report["existing_OD46_evidence"]["counts_toward_launch_matrix"]
        )

    def test_one_current_exact_bundle_closes_only_its_own_row(self) -> None:
        case = authority.required_launch_cases()[0]
        self._create_valid_corner(case)
        report = authority.analyze(root=self.root)
        rows = {row["case_id"]: row for row in report["required_certificates"]}
        self.assertEqual(rows[case.case_id]["status"], "PASS")
        self.assertTrue(rows[case.case_id]["certificate_current"])
        self.assertEqual(report["summary"]["passing"], 1)
        self.assertEqual(report["summary"]["missing"], 23)
        self.assertFalse(report["production_authorized"])

    def test_current_blocked_evidence_is_progress_not_authority(self) -> None:
        case = authority.required_launch_cases()[0]
        self._create_current_blocked_evidence(case)
        report = authority.analyze(root=self.root)
        row = report["required_certificates"][0]
        self.assertEqual(row["status"], "EVIDENCE_BUNDLE_BLOCKED")
        self.assertTrue(row["certificate_current"])
        self.assertFalse(row["corner_authorized"])
        self.assertEqual(report["summary"], {
            "required": 24,
            "passing": 0,
            "missing": 23,
            "invalid_or_stale": 0,
        })
        self.assertEqual(
            report["evidence_progress"]["current_fail_closed_evidence_bundles"],
            1,
        )
        self.assertFalse(report["production_authorized"])

    def test_artifact_hash_drift_invalidates_certificate(self) -> None:
        case = authority.required_launch_cases()[0]
        certificate_path = self._create_valid_corner(case)
        collision = certificate_path.parent / "collision.json"
        collision.write_text("tampered\n", encoding="utf-8")
        report = authority.analyze(root=self.root)
        row = report["required_certificates"][0]
        self.assertEqual(row["status"], "INVALID_CERTIFICATE")
        self.assertTrue(any(
            "artifacts.collision hash mismatch" in error
            for error in row["errors"]
        ))

    def test_current_source_drift_invalidates_certificate(self) -> None:
        case = authority.required_launch_cases()[0]
        self._create_valid_corner(case)
        source = self.root / authority.REQUIRED_SOURCE_PATHS[0]
        source.write_text("changed current source\n", encoding="utf-8")
        report = authority.analyze(root=self.root)
        row = report["required_certificates"][0]
        self.assertEqual(row["status"], "INVALID_CERTIFICATE")
        self.assertTrue(any(
            "source hash mismatch" in error for error in row["errors"]
        ))

    def test_bundle_cannot_reuse_artifact_outside_case_directory(self) -> None:
        case = authority.required_launch_cases()[0]
        certificate_path = self._create_valid_corner(case)
        outside = self.root / "out" / "review" / "default.step"
        outside.parent.mkdir(parents=True, exist_ok=True)
        outside.write_text("default OD46 STEP\n", encoding="utf-8")
        certificate = json.loads(certificate_path.read_text(encoding="utf-8"))
        certificate["artifacts"]["step"] = self._artifact_row(
            outside, case.expected_job(),
        )
        certificate["certificate_payload_sha256"] = authority._canonical_hash(
            certificate, omit=("certificate_payload_sha256",),
        )
        self._write_json(certificate_path, certificate)
        report = authority.analyze(root=self.root)
        row = report["required_certificates"][0]
        self.assertEqual(row["status"], "INVALID_CERTIFICATE")
        self.assertTrue(any(
            "inside its case certificate bundle" in error
            for error in row["errors"]
        ))

    def test_all_24_exact_bundles_can_close_launch_gate(self) -> None:
        for case in authority.required_launch_cases():
            self._create_valid_corner(case)
        report = authority.analyze(root=self.root)
        self.assertEqual(report["status"], "PASS")
        self.assertTrue(report["production_authorized"])
        self.assertEqual(report["summary"]["passing"], 24)
        # OD90 generation is explicitly advisory and cannot block OD28..65.
        self.assertEqual(
            report["OD90_advisory_generation"]["status"],
            "MISSING_ADVISORY_GENERATION",
        )
        self.assertFalse(
            report["OD90_advisory_generation"]["counts_toward_launch_matrix"]
        )

    def test_existing_od46_capture_is_context_only(self) -> None:
        capture = self.root / "out" / "capture" / "upstream_current_raw.jsonl"
        capture.parent.mkdir(parents=True, exist_ok=True)
        capture.write_text(
            json.dumps({
                "e": "meta",
                "controller_mode": "upstream",
                "job": {
                    "od_mm": 46.0,
                    "stack_mm": 15.0,
                    "wire_finished_d_mm": 0.22352,
                    "shaft_d_mm": 4.0,
                    "spindle_id": "er11",
                    "slots": 24,
                    "turns": 50,
                },
            }) + "\n" + json.dumps({"e": "cycle_complete"}) + "\n",
            encoding="utf-8",
        )
        report = authority.analyze(root=self.root)
        od46 = report["existing_OD46_evidence"]
        self.assertEqual(od46["capture_inventory"]["by_od_mm"], {"46.0": 1})
        self.assertFalse(od46["counts_toward_launch_matrix"])
        self.assertEqual(report["summary"]["passing"], 0)

    def test_report_writer_emits_json_and_markdown(self) -> None:
        report = authority.analyze(root=self.root)
        json_out = self.root / "reports" / "authority.json"
        md_out = self.root / "reports" / "authority.md"
        authority.write_reports(report, json_out, md_out)
        loaded = json.loads(json_out.read_text(encoding="utf-8"))
        self.assertEqual(loaded["report_payload_sha256"],
                         report["report_payload_sha256"])
        markdown = md_out.read_text(encoding="utf-8")
        self.assertIn("Status: **FAIL**", markdown)
        self.assertIn("Existing OD46 evidence (non-gating)", markdown)
        self.assertIn("OD90 advisory generation", markdown)


if __name__ == "__main__":
    unittest.main()
