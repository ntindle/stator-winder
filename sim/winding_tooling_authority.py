"""Build the fail-closed winding-tooling release authority.

This report does not choose a design by preference.  It inventories every
generated shoe/guide/former/selector architecture study, hash-binds those
files and the elastic-contact study, and authorizes a production candidate
only when the fixed-flyer study itself or exactly one superseding architecture
explicitly declares both ``status=PASS`` and ``production_authorized=true``
against the canonical raw-upstream capture.  A current fixed-flyer FAIL is
valid evidence; it must not veto a later active-tooling PASS that resolves it.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parent.parent
REPORTS = ROOT / "out" / "reports"
CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
ELASTIC = REPORTS / "elastic_wire_contact_study.json"
OUTPUT = REPORTS / "winding_tooling_authority.json"
SCHEMA = "winding-tooling-authority/v1"

ARCHITECTURE_TOKENS = (
    "shoe", "guide", "former", "selector", "transmission", "raster",
    # Route-family studies below are architecture decisions even when they
    # intentionally stop before emitting a solid.  Keep them in the
    # fail-closed inventory so a later report cannot appear to be the only
    # surviving option merely because an earlier R3 topology used a
    # different noun.
    "shroud", "dogleg", "basket", "sector-chord", "staircase",
    "flyer-recovery", "aggregate-wire-route", "progressive-wire-corridor",
    "cap-aggregate", "offset-spoke",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    return hashlib.sha256(json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def _safe_path(root: Path, relative: Any) -> Path | None:
    if not isinstance(relative, str) or not relative:
        return None
    try:
        path = (root / relative).resolve(strict=False)
        path.relative_to(root.resolve())
    except (OSError, ValueError):
        return None
    return path


def _read_object(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        return None, str(exc)
    if not isinstance(value, dict):
        return None, "JSON root is not an object"
    return value, None


def _capture_meta(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            event = json.loads(line)
            if isinstance(event, dict) and event.get("e") == "meta":
                return event, None
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        return None, str(exc)
    return None, "capture has no meta event"


def _self_hash_valid(payload: Mapping[str, Any]) -> bool:
    expected = payload.get("report_sha256")
    if not isinstance(expected, str) or len(expected) != 64:
        return False
    body = deepcopy(dict(payload))
    body.pop("report_sha256", None)
    return _canonical_hash(body) == expected.lower()


def _path_hash_bindings(payload: Mapping[str, Any]) -> Mapping[str, Any]:
    bindings = payload.get("source_hashes")
    if not isinstance(bindings, Mapping):
        bindings = payload.get("input_hashes")
    return bindings if isinstance(bindings, Mapping) else {}


def _verify_path_bindings(
    root: Path, bindings: Mapping[str, Any],
) -> tuple[bool, list[str]]:
    mismatches: list[str] = []
    if not bindings:
        return False, ["source/input hash map missing"]
    for relative, expected in bindings.items():
        # GOAL.md intentionally lives one level above the nested ``machine``
        # repository.  It is the only approved binding outside ``root``;
        # every other report-controlled path must remain inside the machine
        # tree.
        if str(relative).replace("\\", "/") == "GOAL.md":
            path = (root.parent / "GOAL.md").resolve(strict=False)
        else:
            path = _safe_path(root, relative)
        if path is None:
            mismatches.append(f"{relative}: unsafe path")
            continue
        actual = _sha256(path) if path.is_file() else None
        if not isinstance(expected, str) or expected.lower() != actual:
            mismatches.append(
                f"{relative}: embedded={expected!r}; current={actual!r}"
            )
    return not mismatches, mismatches


def _raw_capture_bound(
    payload: Mapping[str, Any], capture_sha256: str,
) -> bool:
    for field in ("capture_contract", "raw_capture", "capture"):
        contract = payload.get(field)
        if not isinstance(contract, Mapping):
            continue
        embedded_sha = contract.get("capture_sha256", contract.get("sha256"))
        mode = contract.get("controller_mode")
        if embedded_sha == capture_sha256 and mode == "upstream":
            return True
    for relative, embedded in _path_hash_bindings(payload).items():
        normalized = str(relative).replace("\\", "/")
        if (normalized == "out/capture/upstream_current_raw.jsonl"
                and embedded == capture_sha256):
            return True
    return False


def _is_architecture_study(path: Path, payload: Mapping[str, Any]) -> bool:
    if path.name == OUTPUT.name or path == ELASTIC:
        return False
    schema = payload.get("schema")
    haystack = f"{path.stem} {schema}".lower()
    return isinstance(schema, str) and any(
        token in haystack for token in ARCHITECTURE_TOKENS
    )


def _elastic_record(
    root: Path, path: Path, capture_sha256: str,
) -> tuple[dict[str, Any], list[str]]:
    payload, error = _read_object(path)
    if payload is None:
        record = {
            "path": path.relative_to(root).as_posix(),
            "exists": path.is_file(),
            "readable": False,
            "error": error,
        }
        return record, [f"elastic contact evidence unreadable: {error}"]

    expected_sources = {
        "raw_capture_sha256": root / "out" / "capture" / "upstream_current_raw.jsonl",
        "packing_file_sha256": root / "out" / "reports" / "slot_packing.json",
        "slot_wire_routes_file_sha256": root / "out" / "reports" / "slot_wire_routes.json",
        "settings_sha256": root / "out" / "settings.yml",
        "goal_sha256": root.parent / "GOAL.md",
        "traj_source_sha256": root / "sim" / "traj.py",
        "study_source_sha256": root / "sim" / "elastic_wire_contact_study.py",
    }
    source_hashes = payload.get("source_hashes")
    source_hashes = source_hashes if isinstance(source_hashes, Mapping) else {}
    mismatches: list[str] = []
    for field, source in expected_sources.items():
        actual = _sha256(source) if source.is_file() else None
        if source_hashes.get(field) != actual:
            mismatches.append(
                f"{field}: embedded={source_hashes.get(field)!r}; current={actual!r}"
            )
    self_hash_valid = _self_hash_valid(payload)
    raw_bound = source_hashes.get("raw_capture_sha256") == capture_sha256
    evidence_current = (
        payload.get("schema") == "elastic-wire-contact-study/v1"
        and self_hash_valid
        and raw_bound
        and not mismatches
    )
    passed = evidence_current and payload.get("status") == "PASS"
    record = {
        "path": path.relative_to(root).as_posix(),
        "sha256": _sha256(path),
        "schema": payload.get("schema"),
        "status": payload.get("status"),
        "decision": payload.get("decision"),
        "production_authorized": payload.get("production_authorized") is True,
        "report_sha256": payload.get("report_sha256"),
        "report_sha256_valid": self_hash_valid,
        "canonical_raw_capture_bound": raw_bound,
        "source_hashes_current": not mismatches,
        "source_hash_mismatches": mismatches,
        "evidence_current": evidence_current,
        "passed": passed,
    }
    blockers: list[str] = []
    if not evidence_current:
        blockers.append(
            "elastic wire-contact study is stale, unbound, or malformed"
        )
    return record, blockers


def _architecture_record(
    root: Path, path: Path, payload: Mapping[str, Any], capture_sha256: str,
) -> dict[str, Any]:
    bindings = _path_hash_bindings(payload)
    sources_current, source_mismatches = _verify_path_bindings(root, bindings)
    raw_bound = _raw_capture_bound(payload, capture_sha256)
    self_hash_valid = _self_hash_valid(payload)
    production_authorized = payload.get("production_authorized") is True
    eligible = (
        payload.get("status") == "PASS"
        and production_authorized
        and payload.get("release_authorized") is not False
        and self_hash_valid
        and sources_current
        and raw_bound
    )
    return {
        "path": path.relative_to(root).as_posix(),
        "sha256": _sha256(path),
        "schema": payload.get("schema"),
        "status": payload.get("status"),
        "release_authorized": payload.get("release_authorized"),
        "production_authorized": payload.get("production_authorized"),
        "assembly_integration_authorized": payload.get(
            "assembly_integration_authorized"
        ),
        "decision": payload.get("decision"),
        "report_sha256": payload.get("report_sha256"),
        "report_sha256_valid": self_hash_valid,
        "canonical_raw_capture_bound": raw_bound,
        "source_hashes_current": sources_current,
        "source_hash_mismatches": source_mismatches,
        "eligible_production_candidate": eligible,
    }


def evaluate(root: str | Path = ROOT) -> dict[str, Any]:
    root = Path(root).resolve()
    reports = root / "out" / "reports"
    capture = root / "out" / "capture" / "upstream_current_raw.jsonl"
    elastic_path = reports / "elastic_wire_contact_study.json"
    output_path = reports / OUTPUT.name

    meta, capture_error = _capture_meta(capture)
    capture_sha256 = _sha256(capture) if capture.is_file() else ""
    canonical_capture = {
        "path": capture.relative_to(root).as_posix(),
        "sha256": capture_sha256 or None,
        "schema": (meta or {}).get("capture_schema"),
        "controller_mode": (meta or {}).get("controller_mode"),
        "controller_adapter_sha256": (meta or {}).get(
            "controller_adapter_sha256"
        ),
        "winder_commit": (meta or {}).get("winder_commit"),
        "valid": (
            capture_error is None
            and (meta or {}).get("capture_schema") == 4
            and (meta or {}).get("controller_mode") == "upstream"
            and (meta or {}).get("controller_adapter_sha256") is None
            and (meta or {}).get("winding_plan") is None
        ),
        "error": capture_error,
    }

    elastic, blockers = _elastic_record(root, elastic_path, capture_sha256)
    studies: list[dict[str, Any]] = []
    if reports.is_dir():
        for path in sorted(reports.glob("*.json")):
            if path == output_path or path == elastic_path:
                continue
            payload, error = _read_object(path)
            if payload is None or not _is_architecture_study(path, payload):
                continue
            studies.append(_architecture_record(
                root, path, payload, capture_sha256,
            ))

    eligible = [row for row in studies if row["eligible_production_candidate"]]
    eligible_paths = [row["path"] for row in eligible]
    if (elastic.get("passed") is True
            and elastic.get("production_authorized") is True):
        eligible_paths.insert(0, elastic["path"])
    selected_path = eligible_paths[0] if len(eligible_paths) == 1 else None
    if not canonical_capture["valid"]:
        blockers.append("canonical raw-upstream capture contract is invalid")
    if len(eligible_paths) == 0:
        blockers.append(
            "neither fixed-flyer contact nor an architecture study is both "
            "PASS and production_authorized"
        )
    elif len(eligible_paths) > 1:
        blockers.append(
            "multiple winding solutions claim production authority; "
            "selection is ambiguous"
        )

    # This field is advisory progress context only.  Do not keep naming an
    # architecture after its report has become a fail-closed no-go.  Prefer
    # the newest R3/contact-basket evidence, then fall back to the newest
    # unreleased architecture record so the wording cannot imply survival.
    development = None
    # Explicitly prefer the latest permanent-support/flyer recovery lane over
    # older bounded topology failures.  ``studies`` is path-sorted, not
    # chronological, so a bare ``reversed(studies)`` was never a valid
    # implementation of "newest development evidence".
    for token in (
        "offset_spoke", "offset-spoke", "offset_flyer", "offset-flyer",
        "permanent_cap_flyer", "permanent-cap-flyer",
        "permanent_cap_aggregate", "permanent-cap-aggregate",
        "aggregate_progressive", "aggregate-progressive",
        "m0_following_full_shroud", "m0-following-full-shroud",
        "sector_chord", "sector-chord",
        "dogleg", "r3_tooth", "r3-tooth",
    ):
        candidates = [
            row for row in studies if token in row["path"].lower()
        ]
        if candidates:
            development = candidates[-1]
            break
    if development is None and studies:
        development = studies[-1]
    surviving_lane = {
        "id": "goal-bound-r3-contact-corridor",
        "report": development["path"] if development else None,
        "status": (
            development["status"] if development
            else "IN_PROGRESS_NOT_YET_EVIDENCED"
        ),
        "statement": (
            "This is the newest bounded development evidence for the "
            "phase-aware Nomex-offset and all-tooth R3 contact corridor. "
            "Its presence is not a survival or release claim; only an "
            "eligible PASS plus production_authorized report can be selected."
        ),
    }

    production_authorized = (
        canonical_capture["valid"] is True
        and elastic.get("evidence_current") is True
        and selected_path is not None
        and not blockers
    )
    source_hashes: dict[str, str] = {
        "sim/winding_tooling_authority.py": _sha256(Path(__file__)),
    }
    if capture.is_file():
        source_hashes[capture.relative_to(root).as_posix()] = capture_sha256
    for row in [elastic, *studies]:
        path_value = row.get("path")
        source = _safe_path(root, path_value)
        if source is not None and source.is_file():
            source_hashes[str(path_value)] = _sha256(source)

    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "PASS" if production_authorized else "FAIL",
        "production_authorized": production_authorized,
        "canonical_capture": canonical_capture,
        "elastic_contact": elastic,
        "architecture_studies": studies,
        "selected_production_candidate": (
            selected_path
        ),
        "surviving_lane": surviving_lane,
        "release_blockers": blockers,
        "source_hashes": source_hashes,
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def write_report(root: str | Path = ROOT) -> Path:
    root = Path(root).resolve()
    path = root / "out" / "reports" / OUTPUT.name
    path.parent.mkdir(parents=True, exist_ok=True)
    report = evaluate(root)
    path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=ROOT)
    args = parser.parse_args(argv)
    path = write_report(args.root)
    report = json.loads(path.read_text(encoding="utf-8"))
    print(f"wrote {path}")
    print(
        f"status={report['status']}; "
        f"studies={len(report['architecture_studies'])}; "
        f"selected={report['selected_production_candidate']}"
    )
    for blocker in report["release_blockers"]:
        print(f" - {blocker}")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
