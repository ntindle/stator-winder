"""Fail-closed supplemental release gate for ordering and printing.

This report complements, but does not replace, the six GOAL Definition of
Done gates assembled by ``sim/report.py``.  It consumes already-generated
JSON evidence and verifies that each report is newer than the source/artifact
inputs that produced it.  No CAD module is imported and no upstream evidence
is regenerated here.

Run from the machine root::

    .\.venv\Scripts\python.exe cad\release_readiness.py

The command writes ``out/reports/release_readiness.json`` and
``out/reports/release_readiness.md``.  It exits zero only when every required
supplemental gate, plus every hardware audit that has a machine-readable JSON
report, passes.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any, Iterable, Sequence


MACHINE_ROOT = Path(__file__).resolve().parents[1]
REPORTS_REL = Path("out") / "reports"
SCHEMA = 1
FRESHNESS_TOLERANCE_SECONDS = 1.0


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_hash(value: dict[str, Any]) -> str:
    payload = dict(value)
    payload.pop("report_sha256", None)
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _root_relative(path: Path, root: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except (OSError, ValueError):
        return str(path)


def _timestamp(seconds: float) -> str:
    return datetime.fromtimestamp(seconds, timezone.utc).isoformat()


def _file_record(path: Path, root: Path) -> dict[str, Any]:
    record: dict[str, Any] = {
        "path": _root_relative(path, root),
        "exists": False,
    }
    try:
        stat = path.stat()
        if not path.is_file():
            record["error"] = "not a regular file"
            return record
        record.update({
            "exists": True,
            "bytes": stat.st_size,
            "modified_utc": _timestamp(stat.st_mtime),
            "mtime_ns": stat.st_mtime_ns,
            "sha256": _sha256(path),
        })
    except OSError as exc:
        record["error"] = str(exc)
    return record


def _read_json(path: Path, root: Path) -> tuple[dict[str, Any] | None, str | None, dict[str, Any]]:
    record = _file_record(path, root)
    if not record["exists"]:
        return None, f"missing {record['path']}", record
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        return None, f"cannot parse {record['path']}: {exc}", record
    if not isinstance(value, dict):
        return None, f"{record['path']} must contain a JSON object", record
    return value, None, record


def _check(name: str, passed: Any, detail: str) -> dict[str, Any]:
    return {
        "name": str(name),
        "passed": passed is True,
        "detail": str(detail),
    }


def _dedupe_paths(paths: Iterable[Path]) -> list[Path]:
    result: list[Path] = []
    seen: set[str] = set()
    for path in paths:
        key = os.path.normcase(str(path.resolve(strict=False)))
        if key not in seen:
            seen.add(key)
            result.append(path)
    return result


def _freshness(
    report_path: Path,
    dependencies: Sequence[Path],
    root: Path,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    dependency_records = [
        _file_record(path, root) for path in _dedupe_paths(dependencies)
    ]
    report_record = _file_record(report_path, root)
    missing = [row["path"] for row in dependency_records if not row["exists"]]
    if not report_record["exists"]:
        return _check(
            "evidence is current",
            False,
            f"missing {report_record['path']}",
        ), dependency_records
    if missing:
        return _check(
            "evidence is current",
            False,
            "missing dependencies: " + ", ".join(missing),
        ), dependency_records
    if not dependency_records:
        return _check(
            "evidence is current",
            False,
            "no dependencies declared; freshness cannot be established",
        ), dependency_records

    newest = max(dependency_records, key=lambda row: int(row["mtime_ns"]))
    lag_seconds = (
        int(report_record["mtime_ns"]) - int(newest["mtime_ns"])
    ) / 1_000_000_000.0
    current = lag_seconds >= -FRESHNESS_TOLERANCE_SECONDS
    state = "current" if current else f"stale by {-lag_seconds:.3f} s"
    return _check(
        "evidence is current",
        current,
        f"{state}; newest input is {newest['path']}",
    ), dependency_records


def _finish_gate(
    gate_id: str,
    title: str,
    checks: list[dict[str, Any]],
    report: dict[str, Any],
    dependencies: list[dict[str, Any]],
    *,
    summary: dict[str, Any] | None = None,
    required: bool = True,
    present: bool = True,
) -> dict[str, Any]:
    passed = bool(checks) and all(row["passed"] for row in checks)
    return {
        "id": gate_id,
        "title": title,
        "required": required,
        "present": present,
        "status": "PASS" if passed else "FAIL",
        "passed": passed,
        "report": report,
        "dependencies": dependencies,
        "checks": checks,
        "summary": summary or {},
    }


def _unreadable_check(error: str | None, path: Path, root: Path) -> dict[str, Any]:
    label = _root_relative(path, root)
    return _check(
        "report is readable JSON",
        error is None,
        f"parsed {label}" if error is None else str(error),
    )


def _coil_growth_gate(root: Path) -> dict[str, Any]:
    report_path = root / REPORTS_REL / "coil_growth.json"
    dependency_names = ("params.py", "stator_model.py", "coil_growth.py")
    dependency_paths = [root / "cad" / name for name in dependency_names]
    data, error, report_record = _read_json(report_path, root)
    fresh, dependency_records = _freshness(report_path, dependency_paths, root)
    checks = [_unreadable_check(error, report_path, root), fresh]
    data = data or {}
    checks.append(_check(
        "schema is supported",
        data.get("schema") == 1,
        f"schema={data.get('schema')!r}; expected 1",
    ))
    current_default = data.get("current_default")
    default_status = (
        current_default.get("status")
        if isinstance(current_default, dict) else None
    )
    checks.append(_check(
        "current default winding passes",
        default_status == "PASS",
        f"current_default.status={default_status!r}; expected 'PASS'",
    ))

    embedded = data.get("source_sha256")
    embedded = embedded if isinstance(embedded, dict) else {}
    for name, dependency in zip(dependency_names, dependency_paths):
        actual = next(
            (row.get("sha256") for row in dependency_records
             if row["path"] == _root_relative(dependency, root)),
            None,
        )
        expected = embedded.get(name)
        matches = (
            isinstance(expected, str)
            and len(expected) == 64
            and expected.lower() == actual
        )
        checks.append(_check(
            f"embedded source hash matches {name}",
            matches,
            f"embedded={expected!r}; current={actual!r}",
        ))
    return _finish_gate(
        "coil_growth",
        "Current-default coil growth",
        checks,
        report_record,
        dependency_records,
        summary={"current_default_status": default_status},
    )


def _dancer_loads_gate(root: Path) -> dict[str, Any]:
    report_path = root / REPORTS_REL / "dancer_loads.json"
    dependencies = [
        root / "cad" / "dancer_loads.py",
        root / "cad" / "params.py",
        root / "cad" / "wire_geometry.py",
    ]
    data, error, report_record = _read_json(report_path, root)
    fresh, dependency_records = _freshness(report_path, dependencies, root)
    checks = [_unreadable_check(error, report_path, root), fresh]
    data = data or {}
    failures = data.get("fail")
    checks.append(_check(
        "failure list is explicitly empty",
        isinstance(failures, list) and len(failures) == 0,
        f"fail={failures!r}",
    ))
    report_checks = data.get("checks")
    well_formed = (
        isinstance(report_checks, dict)
        and bool(report_checks)
        and all(type(value) is bool for value in report_checks.values())
    )
    failed_names = (
        [name for name, value in report_checks.items() if value is not True]
        if isinstance(report_checks, dict) else []
    )
    checks.append(_check(
        "all named dancer checks pass",
        well_formed and not failed_names,
        (
            f"{len(report_checks)} explicit checks; failed={failed_names}"
            if isinstance(report_checks, dict)
            else "checks must be a non-empty object of booleans"
        ),
    ))
    return _finish_gate(
        "dancer_loads",
        "Dancer load and spring audit",
        checks,
        report_record,
        dependency_records,
        summary={
            "failure_count": len(failures) if isinstance(failures, list) else None,
            "check_count": len(report_checks) if isinstance(report_checks, dict) else None,
        },
    )


def _explicit_record_checks(
    value: Any,
    flag: str,
) -> tuple[bool, list[str], str]:
    if not isinstance(value, list) or not value:
        return False, [], "expected a non-empty check list"
    malformed: list[str] = []
    failed: list[str] = []
    for index, row in enumerate(value):
        if not isinstance(row, dict) or type(row.get(flag)) is not bool:
            malformed.append(str(index))
            continue
        if row[flag] is not True:
            failed.append(str(row.get("name", index)))
    detail = (
        f"{len(value)} checks; failed={failed}; malformed indexes={malformed}"
    )
    return not malformed and not failed, failed, detail


def _felt_loads_gate(root: Path) -> dict[str, Any]:
    report_path = root / REPORTS_REL / "felt_loads.json"
    dependencies = [
        root / "cad" / name for name in (
            "felt_loads.py",
            "hardware.py",
            "hardware_placements.py",
            "params.py",
            "printed.py",
            "wire_geometry.py",
        )
    ]
    data, error, report_record = _read_json(report_path, root)
    fresh, dependency_records = _freshness(report_path, dependencies, root)
    checks = [_unreadable_check(error, report_path, root), fresh]
    data = data or {}
    checks.extend([
        _check(
            "felt report status is PASS",
            data.get("status") == "PASS",
            f"status={data.get('status')!r}; expected 'PASS'",
        ),
        _check(
            "current felt integration is ready",
            data.get("current_integration_ready") is True,
            f"current_integration_ready={data.get('current_integration_ready')!r}",
        ),
        _check(
            "selected spring sizing is ready",
            data.get("selected_spring_sizing_ready") is True,
            f"selected_spring_sizing_ready={data.get('selected_spring_sizing_ready')!r}",
        ),
    ])
    spring_ok, _, spring_detail = _explicit_record_checks(
        data.get("selected_spring_checks"), "pass",
    )
    integration_ok, _, integration_detail = _explicit_record_checks(
        data.get("current_integration_checks"), "pass",
    )
    checks.extend([
        _check("all selected-spring checks pass", spring_ok, spring_detail),
        _check("all felt integration checks pass", integration_ok, integration_detail),
    ])
    return _finish_gate(
        "felt_loads",
        "Felt preload and spring integration",
        checks,
        report_record,
        dependency_records,
        summary={
            "status": data.get("status"),
            "current_integration_ready": data.get("current_integration_ready"),
            "selected_spring_sizing_ready": data.get("selected_spring_sizing_ready"),
        },
    )


def _sendcutsend_gate(root: Path) -> dict[str, Any]:
    report_path = root / REPORTS_REL / "sendcutsend_carriage.json"
    expected_dxf = Path("cad/fabricated_carriage.dxf")
    expected_step = Path("cad/fabricated_carriage.step")
    dependencies = [
        root / "cad" / "sendcutsend_preflight.py",
        root / "cad" / "fabricated_carriage.py",
        root / expected_dxf,
        root / expected_step,
        root / REPORTS_REL / "sendcutsend-catalog.json",
        root / REPORTS_REL / "sendcutsend-specs.json",
        root / REPORTS_REL / "sendcutsend-ordering-guide.md",
    ]
    data, error, report_record = _read_json(report_path, root)
    fresh, dependency_records = _freshness(report_path, dependencies, root)
    checks = [_unreadable_check(error, report_path, root), fresh]
    data = data or {}
    checks.extend([
        _check(
            "report targets the release DXF",
            data.get("file") == expected_dxf.as_posix(),
            f"file={data.get('file')!r}; expected {expected_dxf.as_posix()!r}",
        ),
        _check(
            "report targets the release STEP reference",
            data.get("step_reference") == expected_step.as_posix(),
            (
                f"step_reference={data.get('step_reference')!r}; "
                f"expected {expected_step.as_posix()!r}"
            ),
        ),
        _check(
            "carriage is ready for the assumed upload context",
            data.get("ready_to_upload_for_assumed_context") is True,
            (
                "ready_to_upload_for_assumed_context="
                f"{data.get('ready_to_upload_for_assumed_context')!r}"
            ),
        ),
    ])
    report_checks = data.get("checks")
    all_ok, _, detail = _explicit_record_checks(report_checks, "ok")
    checks.append(_check("all SendCutSend checks pass", all_ok, detail))
    return _finish_gate(
        "sendcutsend_carriage",
        "SendCutSend carriage preflight",
        checks,
        report_record,
        dependency_records,
        summary={
            "ready_to_upload_for_assumed_context": data.get(
                "ready_to_upload_for_assumed_context"
            ),
            "check_count": len(report_checks) if isinstance(report_checks, list) else None,
        },
    )


def _safe_declared_path(root: Path, value: Any) -> Path | None:
    if not isinstance(value, str) or not value.strip():
        return None
    relative = Path(value)
    if relative.is_absolute():
        return None
    try:
        candidate = (root / relative).resolve(strict=False)
        candidate.relative_to(root.resolve())
    except (OSError, ValueError):
        return None
    return candidate


def _belt_audit_gate(root: Path) -> dict[str, Any]:
    """Require current full-revolution evidence for the selected M2 belt lane."""

    report_path = root / REPORTS_REL / "belt_audit.json"
    data, error, report_record = _read_json(report_path, root)
    data = data or {}
    source_hashes = data.get("source_hashes")
    source_hashes = source_hashes if isinstance(source_hashes, dict) else {}

    required_sources = (
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
    declared_dependencies: list[Path] = []
    source_hash_errors: list[str] = []
    for relative, expected in source_hashes.items():
        path = _safe_declared_path(root, relative)
        canonical_relative = (
            _root_relative(path, root) if path is not None else None
        )
        if path is None or canonical_relative != relative:
            source_hash_errors.append(f"unsafe or noncanonical source path {relative!r}")
            continue
        declared_dependencies.append(path)
        record = _file_record(path, root)
        if not record["exists"]:
            source_hash_errors.append(f"missing source {relative}")
        elif (
            not isinstance(expected, str)
            or len(expected) != 64
            or expected.lower() != record.get("sha256")
        ):
            source_hash_errors.append(
                f"SHA-256 drift for {relative}: embedded={expected!r}; "
                f"current={record.get('sha256')!r}"
            )
    missing_required_sources = sorted(set(required_sources) - set(source_hashes))
    if missing_required_sources:
        source_hash_errors.append(
            "missing required source hashes: " + ", ".join(missing_required_sources)
        )
    dependencies = [root / relative for relative in required_sources]
    dependencies.extend(declared_dependencies)
    fresh, dependency_records = _freshness(report_path, dependencies, root)

    checks = [_unreadable_check(error, report_path, root), fresh]
    checks.append(_check(
        "schema is selected M2 belt audit v2",
        data.get("schema") == "selected-m2-belt-audit/v2",
        f"schema={data.get('schema')!r}",
    ))
    report_checks = data.get("checks")
    declared_pass = (
        data.get("status") == "PASS"
        and data.get("passed") is True
        and data.get("geometry_authorized") is True
        and data.get("production_authorized") is False
        and isinstance(data.get("unexpected"), list)
        and not data.get("unexpected")
        and isinstance(report_checks, dict)
        and bool(report_checks)
        and all(type(value) is bool and value for value in report_checks.values())
    )
    checks.append(_check(
        "belt audit explicitly passes every internal check",
        declared_pass,
        (
            f"status={data.get('status')!r}; passed={data.get('passed')!r}; "
            f"geometry_authorized={data.get('geometry_authorized')!r}; "
            f"unexpected={data.get('unexpected')!r}"
        ),
    ))
    embedded_self_hash = data.get("report_sha256")
    actual_self_hash = _canonical_hash(data) if data else None
    checks.append(_check(
        "belt audit self-hash is valid",
        (
            isinstance(embedded_self_hash, str)
            and len(embedded_self_hash) == 64
            and embedded_self_hash == actual_self_hash
        ),
        f"embedded={embedded_self_hash!r}; canonical={actual_self_hash!r}",
    ))
    checks.append(_check(
        "all embedded belt-audit source hashes match current files",
        bool(source_hashes) and not source_hash_errors,
        (
            f"verified={len(source_hashes)}"
            if not source_hash_errors
            else "; ".join(source_hash_errors[:12])
        ),
    ))

    def finite_number(value: Any) -> bool:
        return type(value) in (int, float) and math.isfinite(float(value))

    lane = data.get("lane")
    lane = lane if isinstance(lane, dict) else {}
    expected_lane = {
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
    }
    lane_errors = [
        f"{name}={lane.get(name)!r}; expected {expected!r}"
        for name, expected in expected_lane.items()
        if lane.get(name) != expected
    ]
    checks.append(_check(
        "selected lane is exact NBK P30 30T:30T with 210-3GT-6 belt",
        not lane_errors,
        "exact selected lane" if not lane_errors else "; ".join(lane_errors),
    ))

    sampling = data.get("sampling")
    sampling = sampling if isinstance(sampling, dict) else {}
    sampling_ok = (
        sampling.get("start_deg_inclusive") == 0.0
        and sampling.get("stop_deg_exclusive") == 360.0
        and sampling.get("step_deg") == 1.0
        and sampling.get("sample_count") == 360
        and sampling.get("complete_revolution") is True
    )
    checks.append(_check(
        "complete flyer revolution is sampled at one-degree increments",
        sampling_ok,
        (
            f"start={sampling.get('start_deg_inclusive')!r}; "
            f"stop={sampling.get('stop_deg_exclusive')!r}; "
            f"step={sampling.get('step_deg')!r}; "
            f"samples={sampling.get('sample_count')!r}; "
            f"complete={sampling.get('complete_revolution')!r}"
        ),
    ))

    engagement_pairs = (
        "belt_to_motor_P30_D5_tooth_band",
        "belt_to_flyer_P30_D10_tooth_band",
    )
    policy = data.get("exemption_policy")
    policy = policy if isinstance(policy, dict) else {}
    exemption_ok = (
        policy.get("allowed_positive_contact_pairs") == list(engagement_pairs)
        and policy.get("all_other_belt_contacts_forbidden") is True
        and policy.get("generic_collision_gate_modified") is False
    )
    checks.append(_check(
        "only the two P30 tooth engagements are exempt",
        exemption_ok,
        (
            f"allowed={policy.get('allowed_positive_contact_pairs')!r}; "
            f"all_other_forbidden={policy.get('all_other_belt_contacts_forbidden')!r}; "
            f"generic_gate_modified={policy.get('generic_collision_gate_modified')!r}"
        ),
    ))

    engagements = data.get("intended_engagements")
    engagement_errors: list[str] = []
    engagement_by_pair: dict[str, dict[str, Any]] = {}
    if not isinstance(engagements, list) or len(engagements) != 2:
        engagement_errors.append("intended_engagements must contain exactly two rows")
    else:
        for index, row in enumerate(engagements):
            if not isinstance(row, dict) or not isinstance(row.get("pair"), str):
                engagement_errors.append(f"engagement {index} is malformed")
                continue
            pair = row["pair"]
            if pair in engagement_by_pair:
                engagement_errors.append(f"duplicate engagement {pair}")
            engagement_by_pair[pair] = row
    if set(engagement_by_pair) != set(engagement_pairs):
        engagement_errors.append(
            f"engagement pairs={sorted(engagement_by_pair)!r}"
        )
    for pair, row in engagement_by_pair.items():
        expected_band = row.get("expected_tooth_engagement_radial_band_mm")
        band_ok = (
            isinstance(expected_band, list)
            and len(expected_band) == 2
            and all(finite_number(value) for value in expected_band)
            and finite_number(row.get("minimum_radius_from_pulley_axis_mm"))
            and finite_number(row.get("maximum_radius_from_pulley_axis_mm"))
            and row["minimum_radius_from_pulley_axis_mm"] >= expected_band[0]
            and row["maximum_radius_from_pulley_axis_mm"] <= expected_band[1]
        )
        if not (
            finite_number(row.get("exact_overlap_mm3"))
            and row["exact_overlap_mm3"] > 1.0e-5
            and row.get("tooth_band_only") is True
            and band_ok
        ):
            engagement_errors.append(f"{pair}: missing or out-of-band overlap")
    motor_engagement = engagement_by_pair.get(engagement_pairs[0], {})
    if motor_engagement.get("contact_required") is not True:
        engagement_errors.append("motor P30 contact is not required")
    flyer_engagement = engagement_by_pair.get(engagement_pairs[1], {})
    if not (
        flyer_engagement.get("sample_count") == 360
        and flyer_engagement.get("contact_count") == 360
        and flyer_engagement.get("contact_at_every_sample") is True
        and flyer_engagement.get("missing_contact_angles_deg") == []
    ):
        engagement_errors.append("flyer P30 contact is not continuous for 360 samples")
    checks.append(_check(
        "both P30 tooth-band engagements are positively established",
        not engagement_errors,
        "two bounded engagements verified" if not engagement_errors
        else "; ".join(engagement_errors),
    ))

    rotating = data.get("rotating_non_engagement_parts")
    rotating_errors: list[str] = []
    rotating_keys: list[str] = []
    if not isinstance(rotating, list) or len(rotating) != 41:
        rotating_errors.append("rotating_non_engagement_parts must contain 41 rows")
    else:
        for index, row in enumerate(rotating):
            if not isinstance(row, dict):
                rotating_errors.append(f"rotating row {index} is malformed")
                continue
            key = row.get("part_key")
            if not isinstance(key, str) or not key:
                rotating_errors.append(f"rotating row {index} has no part_key")
            else:
                rotating_keys.append(key)
            if not (
                row.get("ok") is True
                and row.get("sample_count") == 360
                and row.get("collision_count") == 0
                and row.get("collision_angles_deg") == []
                and finite_number(row.get("minimum_clearance_mm"))
                and row["minimum_clearance_mm"] >= 2.2 - 1.0e-6
            ):
                rotating_errors.append(f"{key or index}: collision or sub-2.2 mm clearance")
    if len(rotating_keys) != len(set(rotating_keys)):
        rotating_errors.append("duplicate rotating part keys")
    if "flyer_pulley" in rotating_keys:
        rotating_errors.append("engagement pulley appears in non-engagement rows")
    checks.append(_check(
        "all 41 non-pulley rotating flyer parts clear through 360 degrees",
        not rotating_errors,
        "41 parts x 360 samples verified" if not rotating_errors
        else "; ".join(rotating_errors[:12]),
    ))

    expected_static_keys = {
        "successor_drive": {
            "mount", "motor",
            "motor_pulley_BNW_hole_path_0", "motor_pulley_BNW_hole_path_1",
            "motor_pulley_BNW_set_screw_0", "motor_pulley_BNW_set_screw_1",
            "motor_screw_0", "motor_screw_1", "motor_screw_2", "motor_screw_3",
        },
        "shifted_support": {
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
        },
        "shifted_entry": {
            "entry_bracket", "entry_eyelet", "entry_base_m5x12_1",
            "entry_base_tnut_1", "entry_base_m5x12_2", "entry_base_tnut_2",
        },
        "configured_wire": {"configured_static_supply_wire"},
    }
    static = data.get("static_non_engagement_parts")
    static_errors: list[str] = []
    actual_static_keys: dict[str, set[str]] = {}
    static_clearances: list[float] = []
    if not isinstance(static, list) or len(static) != 38:
        static_errors.append("static_non_engagement_parts must contain 38 rows")
    else:
        for index, row in enumerate(static):
            if not isinstance(row, dict):
                static_errors.append(f"static row {index} is malformed")
                continue
            group = row.get("group")
            key = row.get("part_key")
            if not isinstance(group, str) or not isinstance(key, str):
                static_errors.append(f"static row {index} lacks group or part_key")
                continue
            actual_static_keys.setdefault(group, set()).add(key)
            brep = row.get("BREP")
            brep = brep if isinstance(brep, dict) else {}
            clearance = row.get("clearance_mm")
            if finite_number(clearance):
                static_clearances.append(float(clearance))
            if not (
                row.get("ok") is True
                and row.get("positive_overlap") is False
                and finite_number(row.get("overlap_mm3"))
                and row["overlap_mm3"] <= 1.0e-5
                and finite_number(clearance)
                and clearance >= 2.2 - 1.0e-6
                and brep.get("valid") is True
                and brep.get("method") == "exact_OCC_distance_to_and_common_volume"
            ):
                static_errors.append(f"{group}:{key}: overlap or sub-2.2 mm clearance")
    if actual_static_keys != expected_static_keys:
        static_errors.append(
            f"static key coverage mismatch: {actual_static_keys!r}"
        )
    checks.append(_check(
        "all selected motor mount BNW support entry and wire hardware clears",
        not static_errors,
        "38 exact-BREP static checks verified" if not static_errors
        else "; ".join(static_errors[:12]),
    ))

    summary = data.get("summary")
    summary = summary if isinstance(summary, dict) else {}
    rotating_rows = rotating if isinstance(rotating, list) else []
    rotating_clearances = [
        float(row["minimum_clearance_mm"])
        for row in rotating_rows if isinstance(row, dict)
        and finite_number(row.get("minimum_clearance_mm"))
    ]
    minima_ok = (
        summary.get("rotating_part_count_total") == 42
        and summary.get("rotating_non_engagement_part_count") == 41
        and summary.get("rotating_query_count") == 14760
        and summary.get("static_part_count") == 38
        and summary.get("rotating_failure_count") == 0
        and summary.get("static_failure_count") == 0
        and rotating_clearances
        and static_clearances
        and finite_number(summary.get("minimum_rotating_clearance_mm"))
        and finite_number(summary.get("minimum_static_clearance_mm"))
        and math.isclose(
            summary["minimum_rotating_clearance_mm"], min(rotating_clearances),
            abs_tol=1.0e-9,
        )
        and math.isclose(
            summary["minimum_static_clearance_mm"], min(static_clearances),
            abs_tol=1.0e-9,
        )
        and summary["minimum_rotating_clearance_mm"] >= 2.2 - 1.0e-6
        and summary["minimum_static_clearance_mm"] >= 2.2 - 1.0e-6
    )
    checks.append(_check(
        "belt clearance summary matches all detailed rows",
        bool(minima_ok),
        (
            f"rotating_min={summary.get('minimum_rotating_clearance_mm')!r}; "
            f"static_min={summary.get('minimum_static_clearance_mm')!r}; "
            f"rotating_queries={summary.get('rotating_query_count')!r}"
        ),
    ))

    return _finish_gate(
        "selected_m2_belt_audit",
        "Selected M2 belt full-revolution audit",
        checks,
        report_record,
        dependency_records,
        summary={
            "lane": "NBK P30 30T:30T / 210-3GT-6",
            "rotating_query_count": summary.get("rotating_query_count"),
            "minimum_rotating_clearance_mm": summary.get(
                "minimum_rotating_clearance_mm"
            ),
            "minimum_static_clearance_mm": summary.get(
                "minimum_static_clearance_mm"
            ),
            "allowed_positive_contact_pairs": policy.get(
                "allowed_positive_contact_pairs"
            ),
        },
    )


def _release_catalog_artifact_hash_integrity(
    value: Any,
    root: Path,
) -> tuple[bool, str]:
    """Verify the procurement snapshot's non-generated artifact records.

    Outputs generated by ``procurement.py`` are deliberately skipped: the
    procurement JSON cannot contain its own final file hash, and its sibling
    CSV/JSON outputs are written after the catalog snapshot is constructed.
    Every external/non-generated release artifact must otherwise retain the
    exact existence, byte-count, and SHA-256 state embedded by the catalog.
    """

    if not isinstance(value, list) or not value:
        return False, "release_catalog.release_artifacts must be a non-empty list"

    errors: list[str] = []
    seen_ids: set[str] = set()
    verified = 0
    consistent_missing = 0
    skipped_generated = 0
    for index, row in enumerate(value):
        if not isinstance(row, dict):
            errors.append(f"entry {index} is not an object")
            continue
        artifact_id = row.get("id")
        if not isinstance(artifact_id, str) or not artifact_id:
            errors.append(f"entry {index} has no artifact id")
            artifact_id = str(index)
        elif artifact_id in seen_ids:
            errors.append(f"duplicate artifact id {artifact_id}")
        seen_ids.add(str(artifact_id))

        generated = row.get("generated", False)
        if type(generated) is not bool:
            errors.append(f"{artifact_id}: generated must be boolean")
            continue
        path = _safe_declared_path(root, row.get("path"))
        if path is None:
            errors.append(f"{artifact_id}: unsafe or missing path")
            continue
        if generated:
            skipped_generated += 1
            continue
        current = _file_record(path, root)
        declared_exists = row.get("exists")
        if type(declared_exists) is not bool:
            errors.append(f"{artifact_id}: exists must be boolean")
            continue
        if current["exists"] is not declared_exists:
            errors.append(
                f"{artifact_id}: existence drift; embedded={declared_exists}, "
                f"current={current['exists']}"
            )
            continue
        if not declared_exists:
            consistent_missing += 1
            continue

        embedded_bytes = row.get("bytes")
        embedded_hash = row.get("sha256")
        if type(embedded_bytes) is not int or embedded_bytes != current.get("bytes"):
            errors.append(
                f"{artifact_id}: byte-count drift; embedded={embedded_bytes!r}, "
                f"current={current.get('bytes')!r}"
            )
        if (
            not isinstance(embedded_hash, str)
            or len(embedded_hash) != 64
            or embedded_hash.lower() != current.get("sha256")
        ):
            errors.append(
                f"{artifact_id}: SHA-256 drift; embedded={embedded_hash!r}, "
                f"current={current.get('sha256')!r}"
            )
        verified += 1

    detail = (
        f"verified={verified}; consistently missing={consistent_missing}; "
        f"generated outputs skipped={skipped_generated}"
    )
    if errors:
        detail += "; " + "; ".join(errors[:12])
        if len(errors) > 12:
            detail += f"; and {len(errors) - 12} more"
    return not errors and verified > 0, detail


def _procurement_gate(root: Path) -> dict[str, Any]:
    report_path = root / REPORTS_REL / "procurement.json"
    buildability_path = root / REPORTS_REL / "buildability.json"
    data, error, report_record = _read_json(report_path, root)
    buildability, build_error, _ = _read_json(buildability_path, root)
    data = data or {}
    buildability = buildability or {}

    parts = buildability.get("parts")
    expected_parts: list[str] = []
    if isinstance(parts, list):
        for row in parts:
            name = row.get("part") if isinstance(row, dict) else None
            if isinstance(name, str) and name and Path(name).name == name:
                expected_parts.append(name)

    manifest = data.get("print_manifest")
    declared_print_paths: list[Path] = []
    if isinstance(manifest, list):
        for row in manifest:
            if isinstance(row, dict):
                path = _safe_declared_path(root, row.get("file"))
                if path is not None:
                    declared_print_paths.append(path)
    expected_print_paths = [root / "out" / "stl" / f"{name}.stl" for name in expected_parts]
    dependencies = [
        root / "cad" / "procurement.py",
        root / "cad" / "release_catalog.py",
        root / "cad" / "release_catalog.json",
        root / "bom.csv",
        root / "cad" / "hardware.py",
        root / "cad" / "hardware_placements.py",
        root / "cad" / "buildability.py",
        root / "cad" / "successor_manufacturing.py",
        root / "cad" / "integrated_release_candidate.py",
        root / "cad" / "retained_flyer_peek_guide_successor.py",
        root / "cad" / "carriage_active_sector_terminal_guide.py",
        root / "cad" / "m2_drive_successor_review.py",
        root / "cad" / "permanent_cap_offset_spoke_retained_review.py",
        root / "out" / "custom" / "successor" / "manifest.json",
        root / "out" / "custom" / "successor" / "successor_rfq.csv",
        root / "output" / "pdf" / "successor_custom_parts_rfq.pdf",
        buildability_path,
        *expected_print_paths,
        *declared_print_paths,
    ]
    fresh, dependency_records = _freshness(report_path, dependencies, root)
    checks = [
        _unreadable_check(error, report_path, root),
        _check(
            "buildability report is readable JSON",
            build_error is None,
            (
                f"parsed {_root_relative(buildability_path, root)}"
                if build_error is None else str(build_error)
            ),
        ),
        fresh,
    ]

    checks.extend([
        _check(
            "procurement declares order/print readiness",
            data.get("ready_to_order_and_print") is True,
            f"ready_to_order_and_print={data.get('ready_to_order_and_print')!r}",
        ),
        _check(
            "procurement blocker list is explicitly empty",
            isinstance(data.get("blockers"), list) and not data.get("blockers"),
            f"blockers={data.get('blockers')!r}",
        ),
    ])

    hardware_order = data.get("hardware_order")
    bad_hardware: list[str] = []
    hardware_well_formed = isinstance(hardware_order, list) and bool(hardware_order)
    if isinstance(hardware_order, list):
        for index, row in enumerate(hardware_order):
            if (
                not isinstance(row, dict)
                or row.get("design_status") != "selected"
                or row.get("purchase_status") not in {
                    "cart_ready", "rfq_ready", "upload_ready", "print_ready",
                }
            ):
                bad_hardware.append(
                    str(row.get("sku", index)) if isinstance(row, dict) else str(index)
                )
    checks.append(_check(
        "all hardware lines have selected designs and ready purchase routes",
        hardware_well_formed and not bad_hardware,
        (
            f"{len(hardware_order)} lines; unresolved={bad_hardware}"
            if isinstance(hardware_order, list)
            else "hardware_order must be a non-empty list"
        ),
    ))

    catalog = data.get("release_catalog")
    catalog_blockers = catalog.get("blockers") if isinstance(catalog, dict) else None
    checks.append(_check(
        "canonical release catalog is ready",
        (
            isinstance(catalog, dict)
            and catalog.get("ready") is True
            and isinstance(catalog_blockers, list)
            and not catalog_blockers
        ),
        (
            f"ready={catalog.get('ready')!r}; blockers={catalog_blockers!r}"
            if isinstance(catalog, dict)
            else "release_catalog must be an object"
        ),
    ))
    catalog_hashes_ok, catalog_hashes_detail = (
        _release_catalog_artifact_hash_integrity(
            catalog.get("release_artifacts") if isinstance(catalog, dict) else None,
            root,
        )
    )
    checks.append(_check(
        "embedded non-generated release artifact hashes match current files",
        catalog_hashes_ok,
        catalog_hashes_detail,
    ))

    build_parts_well_formed = (
        isinstance(parts, list)
        and bool(parts)
        and len(expected_parts) == len(parts)
        and len(expected_parts) == len(set(expected_parts))
    )
    build_failures: list[str] = []
    mesh_failures: list[str] = []
    if isinstance(parts, list):
        for index, row in enumerate(parts):
            if not isinstance(row, dict) or row.get("bed_fit") is not True:
                build_failures.append(
                    str(row.get("part", index)) if isinstance(row, dict) else str(index)
                )
            if (
                not isinstance(row, dict)
                or not isinstance(row.get("mesh"), dict)
                or row["mesh"].get("ok") is not True
            ):
                mesh_failures.append(
                    str(row.get("part", index)) if isinstance(row, dict) else str(index)
                )
    wall_ok, _, wall_detail = _explicit_record_checks(
        buildability.get("wall_checks"), "ok",
    )
    build_ok = (
        build_parts_well_formed
        and not build_failures
        and buildability.get("single_solid_check") == "pass"
        and buildability.get("mesh_check") == "pass"
        and not mesh_failures
        and wall_ok
        and isinstance(buildability.get("machining"), list)
        and bool(buildability.get("machining"))
    )
    checks.append(_check(
        "buildability evidence is print-ready",
        build_ok,
        (
            f"parts={len(parts) if isinstance(parts, list) else 'invalid'}; "
            f"bed failures={build_failures}; single_solid_check="
            f"{buildability.get('single_solid_check')!r}; mesh_check="
            f"{buildability.get('mesh_check')!r}; mesh failures="
            f"{mesh_failures}; {wall_detail}; "
            f"machining entries="
            f"{len(buildability.get('machining', [])) if isinstance(buildability.get('machining'), list) else 'invalid'}"
        ),
    ))

    manifest_errors: list[str] = []
    manifest_parts: list[str] = []
    manifest_files: list[str] = []
    if not isinstance(manifest, list) or not manifest:
        manifest_errors.append("print_manifest must be a non-empty list")
    else:
        for index, row in enumerate(manifest):
            if not isinstance(row, dict):
                manifest_errors.append(f"entry {index} is not an object")
                continue
            part = row.get("part")
            path = _safe_declared_path(root, row.get("file"))
            if not isinstance(part, str) or not part:
                manifest_errors.append(f"entry {index} has no part")
            else:
                manifest_parts.append(part)
            if path is None:
                manifest_errors.append(f"{part or index}: unsafe or missing file path")
                continue
            manifest_files.append(_root_relative(path, root))
            record = _file_record(path, root)
            if not record["exists"] or int(record.get("bytes", 0)) <= 0:
                manifest_errors.append(f"{part or index}: missing or empty {record['path']}")
                continue
            expected_hash = row.get("sha256")
            if not isinstance(expected_hash, str) or expected_hash.lower() != record.get("sha256"):
                manifest_errors.append(f"{part or index}: SHA-256 mismatch")
            declared_bytes = row.get("bytes")
            if type(declared_bytes) is not int or declared_bytes != record.get("bytes"):
                manifest_errors.append(f"{part or index}: byte count mismatch")
    if len(manifest_parts) != len(set(manifest_parts)):
        manifest_errors.append("duplicate part names in print_manifest")
    if len(manifest_files) != len(set(manifest_files)):
        manifest_errors.append("duplicate file paths in print_manifest")
    if sorted(manifest_parts) != sorted(expected_parts):
        manifest_errors.append(
            f"manifest parts {sorted(manifest_parts)!r} do not match "
            f"buildability parts {sorted(expected_parts)!r}"
        )
    checks.append(_check(
        "print manifest is complete and every STL hash matches",
        not manifest_errors and bool(expected_parts),
        "verified" if not manifest_errors else "; ".join(manifest_errors),
    ))

    bom = data.get("bom")
    bom_path = _safe_declared_path(root, bom.get("path")) if isinstance(bom, dict) else None
    actual_bom = _file_record(bom_path, root) if bom_path is not None else {
        "path": "invalid", "exists": False,
    }
    expected_bom_hash = bom.get("sha256") if isinstance(bom, dict) else None
    bom_ok = (
        bom_path is not None
        and _root_relative(bom_path, root) == "bom.csv"
        and actual_bom["exists"]
        and isinstance(expected_bom_hash, str)
        and expected_bom_hash.lower() == actual_bom.get("sha256")
    )
    checks.append(_check(
        "embedded BOM hash matches current bom.csv",
        bom_ok,
        f"embedded={expected_bom_hash!r}; current={actual_bom.get('sha256')!r}",
    ))

    return _finish_gate(
        "procurement",
        "Procurement and print handoff",
        checks,
        report_record,
        dependency_records,
        summary={
            "ready_to_order_and_print": data.get("ready_to_order_and_print"),
            "blocker_count": (
                len(data.get("blockers")) if isinstance(data.get("blockers"), list) else None
            ),
            "hardware_line_count": (
                len(hardware_order) if isinstance(hardware_order, list) else None
            ),
            "print_count": len(manifest) if isinstance(manifest, list) else None,
        },
    )


HARDWARE_AUDITS: tuple[dict[str, Any], ...] = (
    {
        "id": "carriage_hardware_audit",
        "title": "Carriage hardware audit",
        "kind": "checks",
        "json_candidates": (
            "out/reports/carriage_hardware_audit.json",
            "cad/carriage_hardware_audit.report.json",
        ),
        "non_json_candidates": ("cad/carriage_hardware_audit.report.md",),
        "dependencies": (
            "cad/carriage_hardware_audit.py",
            "cad/carriage_endstop_flag.py",
            "cad/cots.py",
            "cad/fabricated_carriage.py",
            "cad/hardware.py",
            "cad/hardware_placements.py",
            "cad/params.py",
            "cad/printed.py",
        ),
    },
    {
        "id": "frame_hardware_audit",
        "title": "Static frame hardware audit",
        "kind": "layouts",
        "json_candidates": (
            "out/reports/frame_hardware_audit.json",
            "cad/frame_hardware_audit.report.json",
        ),
        "non_json_candidates": ("cad/frame_hardware_audit.report.md",),
        "dependencies": (
            "cad/frame_hardware_audit.py",
            "cad/assembly.py",
            "cad/cots.py",
            "cad/hardware_placements.py",
            "cad/params.py",
            "cad/printed.py",
        ),
    },
    {
        "id": "m2_m3_hardware_audit",
        "title": "M2/M3 hardware audit",
        "kind": "checks",
        "json_candidates": (
            "out/reports/m2_m3_hardware_audit.json",
            "cad/m2_m3_hardware_audit.report.json",
        ),
        "non_json_candidates": (),
        "dependencies": (
            "cad/m2_m3_hardware_audit.py",
            "cad/cots.py",
            "cad/hardware.py",
            "cad/hardware_placements.py",
            "cad/params.py",
            "cad/printed.py",
            "cad/wire_vis.py",
        ),
    },
)


def _absent_hardware_gate(root: Path, spec: dict[str, Any]) -> dict[str, Any]:
    companion = [
        _file_record(root / item, root)
        for item in spec["non_json_candidates"]
        if (root / item).is_file()
    ]
    return {
        "id": spec["id"],
        "title": spec["title"],
        "required": True,
        "present": False,
        "status": "FAIL",
        "passed": False,
        "report": {
            "path": None,
            "exists": False,
            "json_candidates": list(spec["json_candidates"]),
            "non_json_companions": companion,
        },
        "dependencies": [
            _file_record(root / item, root) for item in spec["dependencies"]
        ],
        "checks": [_check(
            "machine-readable hardware audit is present",
            False,
            "none of the required JSON candidates exists: " +
            ", ".join(spec["json_candidates"]),
        )],
        "summary": {
            "note": "required audit has no machine-readable JSON report",
        },
    }


def _hardware_audit_gate(root: Path, spec: dict[str, Any]) -> dict[str, Any]:
    present_paths = [
        root / item for item in spec["json_candidates"]
        if (root / item).is_file()
    ]
    if not present_paths:
        return _absent_hardware_gate(root, spec)
    report_path = max(present_paths, key=lambda path: path.stat().st_mtime_ns)
    dependencies = [root / item for item in spec["dependencies"]]
    data, error, report_record = _read_json(report_path, root)
    fresh, dependency_records = _freshness(report_path, dependencies, root)
    checks = [_unreadable_check(error, report_path, root), fresh]
    data = data or {}

    if spec["kind"] == "checks":
        checks.append(_check(
            "audit declares pass",
            data.get("passed") is True,
            f"passed={data.get('passed')!r}",
        ))
        all_ok, _, detail = _explicit_record_checks(data.get("checks"), "passed")
        checks.append(_check("all hardware checks pass", all_ok, detail))
        summary = {
            "declared_passed": data.get("passed"),
            "check_count": (
                len(data.get("checks")) if isinstance(data.get("checks"), list) else None
            ),
        }
    else:
        layouts = data.get("layouts")
        failures: list[str] = []
        malformed: list[str] = []
        if not isinstance(layouts, list) or not layouts:
            malformed.append("layouts must be a non-empty list")
        else:
            for index, layout in enumerate(layouts):
                if not isinstance(layout, dict):
                    malformed.append(f"layout {index} is not an object")
                    continue
                name = str(layout.get("name", index))
                forbidden = layout.get("forbidden_positive_volume_pairs")
                allowed = layout.get("allowed_positive_volume_pairs")
                positive = layout.get("positive_volume_pairs")
                if any(type(value) is not int or value < 0
                       for value in (forbidden, allowed, positive)):
                    malformed.append(f"{name}: invalid pair counts")
                elif positive != allowed + forbidden:
                    malformed.append(f"{name}: pair counts do not add up")
                if forbidden != 0:
                    failures.append(f"{name}: {forbidden!r} forbidden pairs")
                findings = layout.get("findings")
                if not isinstance(findings, list):
                    malformed.append(f"{name}: findings is not a list")
                elif any(
                    not isinstance(row, dict) or row.get("status") == "forbidden"
                    for row in findings
                ):
                    failures.append(f"{name}: forbidden or malformed finding")
        checks.append(_check(
            "all frame layouts have zero forbidden positive-volume pairs",
            not malformed and not failures,
            f"failures={failures}; malformed={malformed}",
        ))
        summary = {
            "layout_count": len(layouts) if isinstance(layouts, list) else None,
            "failed_layouts": failures,
        }
    result = _finish_gate(
        spec["id"],
        spec["title"],
        checks,
        report_record,
        dependency_records,
        summary=summary,
    )
    result["report"]["other_json_candidates"] = [
        _root_relative(path, root) for path in present_paths if path != report_path
    ]
    return result


def evaluate(root: str | Path = MACHINE_ROOT) -> dict[str, Any]:
    """Evaluate the supplemental gate without writing or regenerating inputs."""

    root = Path(root).resolve()
    gates = [
        _coil_growth_gate(root),
        _dancer_loads_gate(root),
        _felt_loads_gate(root),
        _belt_audit_gate(root),
        _sendcutsend_gate(root),
        _procurement_gate(root),
    ]
    gates.extend(_hardware_audit_gate(root, spec) for spec in HARDWARE_AUDITS)

    required = [gate for gate in gates if gate["required"]]
    passed = bool(required) and all(gate["passed"] is True for gate in required)
    blockers = [
        {
            "gate": gate["id"],
            "check": check["name"],
            "detail": check["detail"],
        }
        for gate in required
        for check in gate["checks"]
        if not check["passed"]
    ]
    return {
        "schema": SCHEMA,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "scope": (
            "Supplemental order/print readiness beyond the six GOAL DoD gates; "
            "a release requires both this gate and the independent DoD report."
        ),
        "policy": {
            "fail_closed": True,
            "freshness_tolerance_seconds": FRESHNESS_TOLERANCE_SECONDS,
            "optional_hardware_audits": (
                "A hardware audit becomes required when a machine-readable JSON "
                "report is present. Markdown-only notes are recorded but cannot pass "
                "a machine gate."
            ),
        },
        "generator": {
            "path": "cad/release_readiness.py",
            "sha256": _sha256(Path(__file__)),
        },
        "status": "PASS" if passed else "BLOCKED",
        "passed": passed,
        "release_ready": passed,
        "ready_to_order_and_print": passed,
        "required_gate_count": len(required),
        "passed_gate_count": sum(gate["passed"] is True for gate in required),
        "gates": gates,
        "blockers": blockers,
    }


def _md(value: Any) -> str:
    return str(value).replace("|", "\\|").replace("\r", " ").replace("\n", " ")


def markdown(report: dict[str, Any]) -> str:
    status = report["status"]
    lines = [
        "# Supplemental release readiness",
        "",
        f"**{status}** — {report['passed_gate_count']}/{report['required_gate_count']} required gates pass.",
        "",
        report["scope"],
        "",
        "| Gate | Required | Result | Evidence |",
        "|---|---:|---:|---|",
    ]
    for gate in report["gates"]:
        report_path = gate.get("report", {}).get("path") or "no JSON report"
        lines.append(
            f"| {_md(gate['title'])} | {'yes' if gate['required'] else 'no'} | "
            f"{gate['status']} | `{_md(report_path)}` |"
        )

    for gate in report["gates"]:
        lines.extend(["", f"## {gate['title']} — {gate['status']}", ""])
        if not gate["present"]:
            lines.append(
                "No machine-readable JSON report is present; this required audit fails closed."
            )
            for check in gate["checks"]:
                label = "PASS" if check["passed"] else "FAIL"
                lines.append(
                    f"- [{label}] {_md(check['name'])}: {_md(check['detail'])}"
                )
            companions = gate.get("report", {}).get("non_json_companions", [])
            for row in companions:
                lines.append(f"- Markdown companion recorded: `{_md(row['path'])}`")
            continue
        for check in gate["checks"]:
            label = "PASS" if check["passed"] else "FAIL"
            lines.append(
                f"- [{label}] {_md(check['name'])}: {_md(check['detail'])}"
            )
        evidence = [gate["report"], *gate.get("dependencies", [])]
        lines.extend([
            "",
            "| Evidence file | SHA-256 | Modified UTC |",
            "|---|---|---|",
        ])
        for row in evidence:
            digest = row.get("sha256", "missing")
            shown = digest[:16] if isinstance(digest, str) else "missing"
            lines.append(
                f"| `{_md(row.get('path', 'unknown'))}` | `{shown}` | "
                f"{_md(row.get('modified_utc', 'missing'))} |"
            )

    lines.extend(["", "## Blockers", ""])
    if report["blockers"]:
        for blocker in report["blockers"]:
            lines.append(
                f"- `{_md(blocker['gate'])}` / {_md(blocker['check'])}: "
                f"{_md(blocker['detail'])}"
            )
    else:
        lines.append("None in the supplemental evidence.")
    lines.extend([
        "",
        "This result does not waive hardware pull-gauge, wear, thermal, fatigue, or running-machine tests documented by the individual audits.",
        "",
    ])
    return "\n".join(lines)


def write_reports(
    report: dict[str, Any],
    root: str | Path = MACHINE_ROOT,
    json_path: str | Path | None = None,
    markdown_path: str | Path | None = None,
) -> tuple[Path, Path]:
    root = Path(root).resolve()
    json_target = Path(json_path) if json_path is not None else root / REPORTS_REL / "release_readiness.json"
    markdown_target = Path(markdown_path) if markdown_path is not None else root / REPORTS_REL / "release_readiness.md"
    if not json_target.is_absolute():
        json_target = root / json_target
    if not markdown_target.is_absolute():
        markdown_target = root / markdown_target
    json_target.parent.mkdir(parents=True, exist_ok=True)
    markdown_target.parent.mkdir(parents=True, exist_ok=True)
    json_target.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    markdown_target.write_text(markdown(report), encoding="utf-8")
    return json_target, markdown_target


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=MACHINE_ROOT,
        help="machine project root (default: parent of this cad directory)",
    )
    parser.add_argument("--json-out", type=Path, help="override JSON output path")
    parser.add_argument("--markdown-out", type=Path, help="override Markdown output path")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    result = evaluate(args.root)
    json_path, markdown_path = write_reports(
        result,
        args.root,
        args.json_out,
        args.markdown_out,
    )
    if not args.quiet:
        print(
            f"supplemental release readiness: {result['status']} "
            f"({result['passed_gate_count']}/{result['required_gate_count']} gates)"
        )
        for blocker in result["blockers"]:
            print(
                f"FAIL {blocker['gate']} / {blocker['check']}: "
                f"{blocker['detail']}"
            )
        print(json_path)
        print(markdown_path)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
