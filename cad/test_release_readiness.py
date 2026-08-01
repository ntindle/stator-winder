"""Tests for the fail-closed supplemental release-readiness gate."""

from __future__ import annotations

from contextlib import redirect_stdout
import hashlib
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parent))
import release_readiness as release  # noqa: E402


BELT_SOURCE_NAMES = (
    "sim/belt_audit.py",
    "cad/integrated_release_candidate.py",
    "cad/assembly.py",
    "cad/params.py",
    "cad/printed.py",
    "cad/cots.py",
    "cad/hardware.py",
    "cad/hardware_placements.py",
    "cad/wire_geometry.py",
    "cad/m2_drive_successor_review.py",
    "cad/permanent_cap_offset_spoke_retained_review.py",
    "cad/retained_flyer_peek_guide_successor.py",
    "cad/flyer_shaft_d10.py",
    "cad/nbk_p30_official_occurrence.py",
    "cad/models/upgrades/NBK_P30-3GT-BLP-6C-5_AP214.step",
    "cad/nbk_p30_d10_official_occurrence.py",
    "cad/models/upgrades/NBK_P30_D10_download/P30-3GT-BLP-6C-10.stp",
    "cad/leadshine_cs_m21708_cableless.py",
    "cad/models/upgrades/CS-M21708.STEP",
    "cad/models/upgrades/CS-M21708_cableless.step",
    "out/reports/integrated_release_candidate.json",
)


class Fixture:
    def __init__(self, root: Path):
        self.root = root
        self.source_time = time.time() - 20.0
        self.report_time = self.source_time + 10.0

    def path(self, relative: str) -> Path:
        return self.root / relative

    def text(self, relative: str, value: str = "source\n", *, report: bool = False) -> Path:
        path = self.path(relative)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(value, encoding="utf-8")
        stamp = self.report_time if report else self.source_time
        os.utime(path, (stamp, stamp))
        return path

    def binary(self, relative: str, value: bytes, *, report: bool = False) -> Path:
        path = self.path(relative)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(value)
        stamp = self.report_time if report else self.source_time
        os.utime(path, (stamp, stamp))
        return path

    def json(self, relative: str, value: dict, *, report: bool = True) -> Path:
        return self.text(relative, json.dumps(value) + "\n", report=report)

    def sha(self, relative: str) -> str:
        return hashlib.sha256(self.path(relative).read_bytes()).hexdigest()


def passing_belt_report(fixture: Fixture) -> dict:
    rotating = [
        {
            "part_key": f"rotating_part_{index}",
            "label": f"rotating part {index}",
            "ok": True,
            "sample_count": 360,
            "collision_count": 0,
            "collision_angles_deg": [],
            "minimum_clearance_mm": 3.0 + index / 100.0,
            "minimum_angle_deg": float(index),
        }
        for index in range(41)
    ]
    static_keys = {
        "successor_drive": (
            "mount", "motor",
            "motor_pulley_BNW_hole_path_0", "motor_pulley_BNW_hole_path_1",
            "motor_pulley_BNW_set_screw_0", "motor_pulley_BNW_set_screw_1",
            "motor_screw_0", "motor_screw_1", "motor_screw_2", "motor_screw_3",
        ),
        "shifted_support": (
            "flyer_block", "flyer_6001_front", "flyer_6001_rear",
            "m2_outer_race_spacer", "m2_din472_28",
            "flyer_block_L_low_m5x16", "flyer_block_L_low_tnut",
            "flyer_block_L_high_m5x16", "flyer_block_L_high_tnut",
            "flyer_block_R_low_m5x16", "flyer_block_R_low_tnut",
            "flyer_block_R_high_m5x16", "flyer_block_R_high_tnut",
            "m2_mount_L_low_m5x12", "m2_mount_L_low_tnut",
            "m2_mount_L_high_m5x12", "m2_mount_L_high_tnut",
            "m2_mount_R_low_m5x12", "m2_mount_R_low_tnut",
            "m2_mount_R_high_m5x12", "m2_mount_R_high_tnut",
        ),
        "shifted_entry": (
            "entry_bracket", "entry_eyelet", "entry_base_m5x12_1",
            "entry_base_tnut_1", "entry_base_m5x12_2", "entry_base_tnut_2",
        ),
        "configured_wire": ("configured_static_supply_wire",),
    }
    static = [
        {
            "group": group,
            "part_key": key,
            "label": key,
            "ok": True,
            "positive_overlap": False,
            "overlap_mm3": 0.0,
            "clearance_mm": 2.25 + index / 100.0,
            "BREP": {
                "valid": True,
                "method": "exact_OCC_distance_to_and_common_volume",
            },
        }
        for group, keys in static_keys.items()
        for index, key in enumerate(keys)
    ]
    engagements = [
        {
            "pair": "belt_to_motor_P30_D5_tooth_band",
            "exact_overlap_mm3": 340.0,
            "minimum_radius_from_pulley_axis_mm": 12.8,
            "maximum_radius_from_pulley_axis_mm": 13.95,
            "expected_tooth_engagement_radial_band_mm": [12.7, 14.0],
            "tooth_band_only": True,
            "contact_required": True,
        },
        {
            "pair": "belt_to_flyer_P30_D10_tooth_band",
            "exact_overlap_mm3": 340.0,
            "minimum_radius_from_pulley_axis_mm": 12.8,
            "maximum_radius_from_pulley_axis_mm": 13.95,
            "expected_tooth_engagement_radial_band_mm": [12.7, 14.0],
            "tooth_band_only": True,
            "sample_count": 360,
            "contact_count": 360,
            "contact_at_every_sample": True,
            "missing_contact_angles_deg": [],
        },
    ]
    report = {
        "schema": "selected-m2-belt-audit/v2",
        "status": "PASS",
        "passed": True,
        "geometry_authorized": True,
        "production_authorized": False,
        "sampling": {
            "start_deg_inclusive": 0.0,
            "stop_deg_exclusive": 360.0,
            "step_deg": 1.0,
            "sample_count": 360,
            "complete_revolution": True,
        },
        "lane": {
            "motor_teeth": 30,
            "flyer_teeth": 30,
            "pitch_mm": 3.0,
            "belt_model": "210-3GT-6",
            "belt_pitch_length_mm": 210.0,
            "belt_width_mm": 6.0,
            "center_distance_mm": 60.0,
            "motor_pulley_label": (
                "NBK_P30_3GT_BLP_6C_5_stock_split_clamp_vendor_occurrence"
            ),
            "flyer_pulley_label": (
                "NBK_P30_3GT_BLP_6C_10_stock_hub_rear_vendor_occurrence"
            ),
            "belt_label": "m2_successor_210_3gt_6_belt",
        },
        "exemption_policy": {
            "allowed_positive_contact_pairs": [
                "belt_to_motor_P30_D5_tooth_band",
                "belt_to_flyer_P30_D10_tooth_band",
            ],
            "all_other_belt_contacts_forbidden": True,
            "generic_collision_gate_modified": False,
        },
        "intended_engagements": engagements,
        "rotating_non_engagement_parts": rotating,
        "static_non_engagement_parts": static,
        "summary": {
            "rotating_part_count_total": 42,
            "rotating_non_engagement_part_count": 41,
            "rotating_query_count": 14760,
            "static_part_count": 38,
            "rotating_failure_count": 0,
            "static_failure_count": 0,
            "minimum_rotating_clearance_mm": 3.0,
            "minimum_static_clearance_mm": 2.25,
        },
        "checks": {"all_required_belt_checks_pass": True},
        "unexpected": [],
        "source_hashes": {
            name: fixture.sha(name) for name in BELT_SOURCE_NAMES
        },
    }
    report["report_sha256"] = release._canonical_hash(report)
    return report


def make_passing_fixture(root: Path) -> Fixture:
    fixture = Fixture(root)

    for name in ("params.py", "stator_model.py", "coil_growth.py"):
        fixture.text(f"cad/{name}", f"# {name}\n")
    fixture.json("out/reports/coil_growth.json", {
        "schema": 1,
        "current_default": {"status": "PASS"},
        "source_sha256": {
            name: fixture.sha(f"cad/{name}")
            for name in ("params.py", "stator_model.py", "coil_growth.py")
        },
    })

    for name in ("dancer_loads.py", "wire_geometry.py"):
        fixture.text(f"cad/{name}", f"# {name}\n")
    fixture.json("out/reports/dancer_loads.json", {
        "fail": [],
        "checks": {"stable": True, "rated": True},
    })

    for name in (
        "felt_loads.py", "hardware.py", "hardware_placements.py", "printed.py",
    ):
        fixture.text(f"cad/{name}", f"# {name}\n")
    fixture.json("out/reports/felt_loads.json", {
        "status": "PASS",
        "current_integration_ready": True,
        "selected_spring_sizing_ready": True,
        "selected_spring_checks": [{"name": "spring", "pass": True}],
        "current_integration_checks": [{"name": "integration", "pass": True}],
    })

    fixture.text("cad/sendcutsend_preflight.py")
    fixture.text("cad/fabricated_carriage.py")
    fixture.binary("cad/fabricated_carriage.dxf", b"dxf")
    fixture.binary("cad/fabricated_carriage.step", b"step")
    fixture.json("out/reports/sendcutsend-catalog.json", {"materials": []}, report=False)
    fixture.json("out/reports/sendcutsend-specs.json", {"materials": []}, report=False)
    fixture.text("out/reports/sendcutsend-ordering-guide.md", "guide\n")
    fixture.json("out/reports/sendcutsend_carriage.json", {
        "file": "cad/fabricated_carriage.dxf",
        "step_reference": "cad/fabricated_carriage.step",
        "ready_to_upload_for_assumed_context": True,
        "checks": [{"name": "geometry", "ok": True}],
    })

    fixture.text("cad/procurement.py")
    fixture.text("cad/release_catalog.py")
    fixture.text("cad/release_catalog.json", "{}\n")
    fixture.text("cad/buildability.py")
    for name in (
        "successor_manufacturing.py", "integrated_release_candidate.py",
        "retained_flyer_peek_guide_successor.py",
        "carriage_active_sector_terminal_guide.py",
        "m2_drive_successor_review.py",
        "permanent_cap_offset_spoke_retained_review.py",
    ):
        fixture.text(f"cad/{name}")
    for relative in BELT_SOURCE_NAMES:
        if relative == "out/reports/integrated_release_candidate.json":
            continue
        if fixture.path(relative).exists():
            continue
        if Path(relative).suffix.lower() in {".step", ".stp"}:
            fixture.binary(relative, b"STEP fixture")
        else:
            fixture.text(relative)
    fixture.json(
        "out/reports/integrated_release_candidate.json",
        {"schema": "integrated-release-candidate/v1", "status": "PASS"},
        report=False,
    )
    fixture.json(
        "out/reports/belt_audit.json", passing_belt_report(fixture),
    )
    fixture.json("out/custom/successor/manifest.json", {
        "schema": "successor-manufacturing-packet/v1",
        "production_authorized": False,
        "order_authorized": False,
    }, report=False)
    fixture.text("out/custom/successor/successor_rfq.csv", "part_id\nfixture\n")
    fixture.binary("output/pdf/successor_custom_parts_rfq.pdf", b"%PDF-fixture")
    fixture.text("bom.csv", "category,item\nprint,part\n")
    stl = fixture.binary("out/stl/fixture.stl", b"solid fixture\nendsolid\n")
    fixture.json("out/reports/buildability.json", {
        "single_solid_check": "pass",
        "mesh_check": "pass",
        "parts": [{"part": "fixture", "bed_fit": True,
                   "mesh": {"ok": True}}],
        "wall_checks": [{"feature": "wall", "ok": True}],
        "machining": [{"part": "fixture", "operation": "none"}],
    }, report=False)
    fixture.json(
        "out/reports/live_release_evidence.json",
        {"schema": "fixture/v1", "status": "PASS"},
        report=False,
    )
    live_evidence = fixture.path("out/reports/live_release_evidence.json")
    fixture.json("out/reports/procurement.json", {
        "ready_to_order_and_print": True,
        "bom": {"path": "bom.csv", "sha256": fixture.sha("bom.csv")},
        "hardware_order": [{
            "sku": "FIXTURE",
            "status": "selected",
            "design_status": "selected",
            "purchase_status": "cart_ready",
        }],
        "release_catalog": {
            "ready": True,
            "blockers": [],
            "release_artifacts": [
                {
                    "id": "release-live-evidence",
                    "path": "out/reports/live_release_evidence.json",
                    "exists": True,
                    "generated": False,
                    "bytes": live_evidence.stat().st_size,
                    "sha256": fixture.sha(
                        "out/reports/live_release_evidence.json"
                    ),
                },
                {
                    # Self/sibling outputs cannot carry their own final hash
                    # inside the procurement snapshot and are skipped.
                    "id": "release-procurement",
                    "path": "out/reports/procurement.json",
                    "exists": False,
                    "generated": True,
                },
            ],
        },
        "print_manifest": [{
            "part": "fixture",
            "file": "out/stl/fixture.stl",
            "bytes": stl.stat().st_size,
            "sha256": fixture.sha("out/stl/fixture.stl"),
        }],
        "blockers": [],
    })

    for name in (
        "carriage_hardware_audit.py", "carriage_endstop_flag.py", "cots.py",
        "assembly.py", "frame_hardware_audit.py", "m2_m3_hardware_audit.py",
        "wire_vis.py",
    ):
        if not fixture.path(f"cad/{name}").exists():
            fixture.text(f"cad/{name}")
    fixture.json("out/reports/carriage_hardware_audit.json", {
        "passed": True,
        "checks": [{"name": "carriage", "passed": True}],
    })
    fixture.json("cad/frame_hardware_audit.report.json", {
        "layouts": [{
            "name": "frame",
            "positive_volume_pairs": 0,
            "allowed_positive_volume_pairs": 0,
            "forbidden_positive_volume_pairs": 0,
            "findings": [],
        }],
    })
    fixture.json("out/reports/m2_m3_hardware_audit.json", {
        "passed": True,
        "checks": [{"name": "retention", "passed": True}],
    })
    return fixture


def gate(result: dict, gate_id: str) -> dict:
    return next(row for row in result["gates"] if row["id"] == gate_id)


class ReleaseReadinessTests(unittest.TestCase):
    def test_complete_current_fixture_passes_with_required_audits_present(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = make_passing_fixture(Path(temporary))
            result = release.evaluate(fixture.root)

        self.assertTrue(result["passed"], result["blockers"])
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["required_gate_count"], 9)
        self.assertEqual(result["passed_gate_count"], 9)
        self.assertTrue(gate(result, "selected_m2_belt_audit")["passed"])
        for audit_id in (
            "carriage_hardware_audit",
            "frame_hardware_audit",
            "m2_m3_hardware_audit",
        ):
            audit = gate(result, audit_id)
            self.assertTrue(audit["required"])
            self.assertTrue(audit["passed"])

    def test_missing_hardware_audit_is_required_and_fails_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = make_passing_fixture(Path(temporary))
            fixture.path("out/reports/carriage_hardware_audit.json").unlink()
            result = release.evaluate(fixture.root)

        audit = gate(result, "carriage_hardware_audit")
        self.assertTrue(audit["required"])
        self.assertFalse(audit["present"])
        self.assertEqual(audit["status"], "FAIL")
        self.assertFalse(result["passed"])

    def test_nonmanifold_print_mesh_fails_procurement_gate(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = make_passing_fixture(Path(temporary))
            path = fixture.path("out/reports/buildability.json")
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["mesh_check"] = [
                {"part": "fixture", "nonmanifold_edges": 1}
            ]
            payload["parts"][0]["mesh"]["ok"] = False
            fixture.json(
                "out/reports/buildability.json", payload, report=False,
            )
            result = release.evaluate(fixture.root)

        procurement = gate(result, "procurement")
        self.assertFalse(procurement["passed"])
        check = next(row for row in procurement["checks"]
                     if row["name"] == "buildability evidence is print-ready")
        self.assertFalse(check["passed"])
        self.assertIn("fixture", check["detail"])

    def test_missing_or_malformed_evidence_fails_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = make_passing_fixture(Path(temporary))
            fixture.path("out/reports/dancer_loads.json").unlink()
            fixture.text(
                "out/reports/felt_loads.json", "{not json\n", report=True,
            )
            result = release.evaluate(fixture.root)

        self.assertFalse(result["passed"])
        self.assertEqual(gate(result, "dancer_loads")["status"], "FAIL")
        self.assertEqual(gate(result, "felt_loads")["status"], "FAIL")
        failed_names = {
            (row["gate"], row["check"]) for row in result["blockers"]
        }
        self.assertIn(("dancer_loads", "report is readable JSON"), failed_names)
        self.assertIn(("felt_loads", "report is readable JSON"), failed_names)

    def test_source_artifact_freshness_and_embedded_hashes_are_enforced(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = make_passing_fixture(Path(temporary))
            dxf = fixture.path("cad/fabricated_carriage.dxf")
            dxf.write_bytes(b"new dxf")
            newer = fixture.report_time + 20.0
            os.utime(dxf, (newer, newer))
            fixture.path("bom.csv").write_text(
                "category,item\nprint,changed\n", encoding="utf-8",
            )
            os.utime(fixture.path("bom.csv"), (newer, newer))
            result = release.evaluate(fixture.root)

        send = gate(result, "sendcutsend_carriage")
        procurement = gate(result, "procurement")
        self.assertFalse(send["passed"])
        self.assertFalse(procurement["passed"])
        self.assertTrue(any(
            check["name"] == "evidence is current" and not check["passed"]
            for check in send["checks"]
        ))
        self.assertTrue(any(
            check["name"] == "embedded BOM hash matches current bom.csv"
            and not check["passed"]
            for check in procurement["checks"]
        ))

    def test_nested_release_catalog_artifact_hash_drift_fails_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = make_passing_fixture(Path(temporary))
            evidence = fixture.path("out/reports/live_release_evidence.json")
            evidence.write_text(
                '{"schema":"fixture/v1","status":"CHANGED"}\n',
                encoding="utf-8",
            )
            # Keep the old timestamp so this specifically exercises the
            # embedded hash contract rather than the coarse freshness gate.
            os.utime(evidence, (fixture.source_time, fixture.source_time))
            result = release.evaluate(fixture.root)

        procurement = gate(result, "procurement")
        check = next(
            row for row in procurement["checks"]
            if row["name"] == (
                "embedded non-generated release artifact hashes match current files"
            )
        )
        self.assertFalse(check["passed"])
        self.assertIn("release-live-evidence", check["detail"])
        self.assertIn("SHA-256 drift", check["detail"])

    def test_each_declared_business_verdict_is_required(self):
        mutations = (
            ("out/reports/coil_growth.json", "current_default", {"status": "FAIL"}, "coil_growth"),
            ("out/reports/dancer_loads.json", "fail", ["unstable"], "dancer_loads"),
            ("out/reports/felt_loads.json", "status", "FAIL", "felt_loads"),
            (
                "out/reports/belt_audit.json", "passed", False,
                "selected_m2_belt_audit",
            ),
            (
                "out/reports/sendcutsend_carriage.json",
                "ready_to_upload_for_assumed_context", False,
                "sendcutsend_carriage",
            ),
            (
                "out/reports/procurement.json",
                "ready_to_order_and_print", False,
                "procurement",
            ),
        )
        for relative, key, value, gate_id in mutations:
            with self.subTest(gate=gate_id), tempfile.TemporaryDirectory() as temporary:
                fixture = make_passing_fixture(Path(temporary))
                payload = json.loads(fixture.path(relative).read_text(encoding="utf-8"))
                payload[key] = value
                fixture.json(relative, payload)
                result = release.evaluate(fixture.root)
                self.assertFalse(gate(result, gate_id)["passed"])
                self.assertFalse(result["passed"])

    def test_belt_lane_and_exemption_tampering_fails_even_with_new_self_hash(self):
        mutations = (
            ("lane", "belt_model", "200-2GT"),
            (
                "exemption_policy", "allowed_positive_contact_pairs",
                [
                    "belt_to_motor_P30_D5_tooth_band",
                    "belt_to_flyer_P30_D10_tooth_band",
                    "belt_to_motor_body",
                ],
            ),
        )
        for section, key, value in mutations:
            with self.subTest(section=section, key=key), tempfile.TemporaryDirectory() as temporary:
                fixture = make_passing_fixture(Path(temporary))
                path = fixture.path("out/reports/belt_audit.json")
                payload = json.loads(path.read_text(encoding="utf-8"))
                payload[section][key] = value
                payload["report_sha256"] = release._canonical_hash(payload)
                fixture.json("out/reports/belt_audit.json", payload)
                result = release.evaluate(fixture.root)

                belt = gate(result, "selected_m2_belt_audit")
                self.assertFalse(belt["passed"])
                self.assertFalse(result["passed"])
                self.assertTrue(any(
                    not check["passed"] and (
                        "selected lane" in check["name"]
                        or "only the two P30" in check["name"]
                    )
                    for check in belt["checks"]
                ))

    def test_belt_source_hash_closure_cannot_be_omitted(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = make_passing_fixture(Path(temporary))
            path = fixture.path("out/reports/belt_audit.json")
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["source_hashes"].pop("cad/assembly.py")
            payload["report_sha256"] = release._canonical_hash(payload)
            fixture.json("out/reports/belt_audit.json", payload)
            result = release.evaluate(fixture.root)

        belt = gate(result, "selected_m2_belt_audit")
        check = next(
            row for row in belt["checks"]
            if row["name"] == (
                "all embedded belt-audit source hashes match current files"
            )
        )
        self.assertFalse(check["passed"])
        self.assertIn("cad/assembly.py", check["detail"])
        self.assertFalse(result["passed"])

    def test_present_hardware_audits_are_required_and_schema_checked(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = make_passing_fixture(Path(temporary))
            for name in (
                "carriage_hardware_audit.py", "carriage_endstop_flag.py", "cots.py",
                "fabricated_carriage.py", "assembly.py", "frame_hardware_audit.py",
                "m2_m3_hardware_audit.py", "wire_vis.py",
            ):
                if not fixture.path(f"cad/{name}").exists():
                    fixture.text(f"cad/{name}")
            fixture.json("out/reports/carriage_hardware_audit.json", {
                "passed": True,
                "checks": [{"name": "carriage", "passed": True}],
            })
            fixture.json("cad/frame_hardware_audit.report.json", {
                "layouts": [{
                    "name": "frame",
                    "positive_volume_pairs": 1,
                    "allowed_positive_volume_pairs": 0,
                    "forbidden_positive_volume_pairs": 1,
                    "findings": [{"status": "forbidden"}],
                }],
            })
            fixture.json("out/reports/m2_m3_hardware_audit.json", {
                "passed": False,
                "checks": [{"name": "retention", "passed": False}],
            })
            result = release.evaluate(fixture.root)

        self.assertEqual(result["required_gate_count"], 9)
        self.assertTrue(gate(result, "carriage_hardware_audit")["passed"])
        self.assertFalse(gate(result, "frame_hardware_audit")["passed"])
        self.assertFalse(gate(result, "m2_m3_hardware_audit")["passed"])
        self.assertFalse(result["passed"])

    def test_cli_writes_both_reports_and_returns_gate_status(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = make_passing_fixture(Path(temporary))
            with redirect_stdout(io.StringIO()):
                exit_code = release.main(["--root", str(fixture.root), "--quiet"])
            json_path = fixture.path("out/reports/release_readiness.json")
            markdown_path = fixture.path("out/reports/release_readiness.md")
            written = json.loads(json_path.read_text(encoding="utf-8"))
            rendered = markdown_path.read_text(encoding="utf-8")

        self.assertEqual(exit_code, 0)
        self.assertTrue(written["passed"])
        self.assertIn("Supplemental release readiness", rendered)

    def test_current_workspace_result_is_structured_without_assuming_green(self):
        result = release.evaluate()
        self.assertIn(result["status"], {"PASS", "BLOCKED"})
        self.assertEqual(
            {row["id"] for row in result["gates"]},
            {
                "coil_growth", "dancer_loads", "felt_loads",
                "selected_m2_belt_audit", "sendcutsend_carriage", "procurement",
                "carriage_hardware_audit", "frame_hardware_audit",
                "m2_m3_hardware_audit",
            },
        )
        self.assertEqual(result["passed"], not bool(result["blockers"]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
