"""Assemble a truthful validation report from machine-generated evidence.

Run after the selected integrated-candidate adapter/player, active-sector
rigid audit, full-cycle conductor authority, launch-envelope authority,
cad/loads.py, and cad/buildability.py.  The untouched upstream capture and
cycle remain the controller-compatibility contract.  Legacy ``out/links``
clearance, wire-path and player outputs are diagnostics only and never decide
Definition of Done gates 1--3.
"""

from __future__ import annotations

import base64
import binascii
import hashlib
import json
import math
import re
import sys
from datetime import datetime
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "out"
REPORTS = OUT / "reports"
CAPTURE = OUT / "capture" / "upstream_current_raw.jsonl"
RAW_CYCLE = REPORTS / "upstream_current_raw_cycle.json"
RAW_CLEARANCE = REPORTS / "clearance_upstream_raw.json"
RAW_WIREPATH = REPORTS / "wirepath_upstream_raw.json"
RAW_GLB = OUT / "winding_cycle_upstream_raw.glb"
RAW_PLAYER = OUT / "play_animation_upstream_raw.html"
TOOLING_AUTHORITY = REPORTS / "winding_tooling_authority.json"
LINKS = OUT / "links"
ADAPTER_ROOT = OUT / "review" / "integrated_adapter"
SELECTED_MANIFEST = ADAPTER_ROOT / "links" / "manifest.json"
SELECTED_PLAYER_RENDER = ADAPTER_ROOT / "player_render.json"
SELECTED_GLB = ADAPTER_ROOT / "winding_cycle_integrated_candidate_raw.glb"
SELECTED_PLAYER = ADAPTER_ROOT / "play_integrated_candidate_raw.html"
SELECTED_ROUTE = ADAPTER_ROOT / "reports" / "continuous_conductor_route.json"
ACTIVE_RIGID_AUDIT = REPORTS / "carriage_active_sector_terminal_guide_audit.json"
ACTIVE_LOCI = REPORTS / "carriage_active_sector_terminal_guide_loci.json"
INTEGRATED_CANDIDATE = REPORTS / "integrated_release_candidate.json"
FULL_CONDUCTOR_AUTHORITY = (
    REPORTS / "full_cycle_continuous_conductor_authority_audit.json"
)
SHAFT_WRAP_REGRESSION = REPORTS / "shaft_wrap_regression_evidence.json"
LAUNCH_AUTHORITY = REPORTS / "launch_envelope_authority.json"
RELEASE_CATALOG = ROOT / "cad" / "release_catalog.json"
VALIDATION_SCHEMA = "machine-validation/v2"


def _read_json(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8")), None
    except FileNotFoundError:
        return None, f"missing {path.relative_to(ROOT)}"
    except (OSError, json.JSONDecodeError) as exc:
        return None, f"cannot read {path.relative_to(ROOT)}: {exc}"


def _sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_capture(path: Path = CAPTURE):
    events = []
    number = 0
    try:
        for number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            if line.strip():
                events.append(json.loads(line))
    except FileNotFoundError:
        return [], None, f"missing {path.relative_to(ROOT)}"
    except (OSError, json.JSONDecodeError) as exc:
        return [], None, f"cannot read capture at line {number}: {exc}"
    meta = next((event for event in events if event.get("e") == "meta"), None)
    return events, meta, None if meta else "capture has no meta event"


def _read_player_bundle(player_path: Path = SELECTED_PLAYER,
                        glb_path: Path = SELECTED_GLB):
    """Read a player state and hash the exact embedded/on-disk GLB."""
    try:
        html = player_path.read_text(encoding="utf-8")
        state_match = re.search(
            r'const stateB64 = "([A-Za-z0-9+/=]+)";', html)
        glb_match = re.search(
            r'const glbB64 = "([A-Za-z0-9+/=]+)";', html)
        if state_match is None or glb_match is None:
            raise ValueError("player has no embedded state/GLB payload")
        state_bytes = base64.b64decode(state_match.group(1), validate=True)
        state = json.loads(state_bytes.decode("utf-8"))
        if not isinstance(state, dict):
            raise ValueError("player state is not a JSON object")
        embedded_glb = base64.b64decode(glb_match.group(1), validate=True)
        disk_glb_sha256 = _sha256(glb_path)
        return {
            "state": state,
            "embedded_glb_sha256": hashlib.sha256(embedded_glb).hexdigest(),
            "disk_glb_sha256": disk_glb_sha256,
            "embedded_glb_bytes": len(embedded_glb),
            "disk_glb_bytes": glb_path.stat().st_size if glb_path.is_file() else None,
        }, None
    except FileNotFoundError as exc:
        return None, f"missing {Path(exc.filename).relative_to(ROOT)}"
    except (OSError, UnicodeError, ValueError, binascii.Error,
            json.JSONDecodeError) as exc:
        return None, f"cannot verify player bundle: {exc}"


def _relative_source_hash_checks(label: str, embedded):
    """Validate a report's relative-path -> SHA-256 map fail-closed."""
    if not isinstance(embedded, dict) or not embedded:
        return [_check(f"{label} source hashes are explicit", False,
                       "missing or empty source_hashes")]
    checks = []
    for relative, expected in sorted(embedded.items()):
        try:
            path = (ROOT / relative).resolve(strict=False)
            path.relative_to(ROOT.resolve())
        except (OSError, ValueError, TypeError):
            checks.append(_check(
                f"{label} source path is safe", False,
                f"unsafe path {relative!r}",
            ))
            continue
        actual = _sha256(path)
        checks.append(_check(
            f"{label} hash matches {relative}",
            isinstance(expected, str) and expected == actual,
            f"embedded={expected}; current={actual}",
        ))
    return checks


def _canonical_field_hash_valid(payload, field="report_sha256") -> bool:
    if not isinstance(payload, dict):
        return False
    expected = payload.get(field)
    if not isinstance(expected, str) or len(expected) != 64:
        return False
    body = dict(payload)
    body.pop(field, None)
    actual = hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()
    return actual == expected


def _canonical_report_hash_valid(payload) -> bool:
    return _canonical_field_hash_valid(payload, "report_sha256")


def _source_hash_closure_check(label: str, embedded):
    rows = _relative_source_hash_checks(label, embedded)
    failed = [
        f"{row['label']}: {row['detail']}" for row in rows if not row["ok"]
    ]
    return _check(
        f"{label} source hash closure is current",
        bool(rows) and not failed,
        "; ".join(failed) if failed else f"{len(rows)} bound files match",
    )


def _safe_project_path(value) -> Path | None:
    if not isinstance(value, str) or not value:
        return None
    try:
        candidate = Path(value)
        if not candidate.is_absolute():
            candidate = ROOT / candidate
        candidate = candidate.resolve(strict=False)
        candidate.relative_to(ROOT.resolve())
        return candidate
    except (OSError, ValueError, TypeError):
        return None


def _path_matches(value, expected: Path) -> bool:
    try:
        candidate = Path(value)
        if not candidate.is_absolute():
            candidate = ROOT / candidate
        return candidate.resolve(strict=False) == expected.resolve(strict=False)
    except (OSError, TypeError, ValueError):
        return False


def _release_catalog_binding_checks(catalog, catalog_error, expected):
    checks = [_check(
        "release catalog readable",
        catalog_error is None and isinstance(catalog, dict),
        catalog_error or f"{RELEASE_CATALOG.relative_to(ROOT)} parsed",
    )]
    if not isinstance(catalog, dict):
        return checks
    rows = catalog.get("release_artifacts")
    rows = rows if isinstance(rows, list) else []
    by_id = {
        row.get("id"): row for row in rows
        if isinstance(row, dict) and isinstance(row.get("id"), str)
    }
    checks.append(_check(
        "release catalog declares supported artifact schema",
        catalog.get("schema") == 1 and bool(rows),
        f"schema={catalog.get('schema')!r}; artifacts={len(rows)}",
    ))
    for artifact_id, (role, path) in expected.items():
        row = by_id.get(artifact_id, {})
        relative = path.relative_to(ROOT).as_posix()
        checks.append(_check(
            f"release catalog binds {artifact_id}",
            row.get("role") == role and row.get("path") == relative,
            f"role={row.get('role')!r}; path={row.get('path')!r}; "
            f"expected={role!r}, {relative!r}",
        ))
    return checks


def _bound_input_hash_check(label: str, report):
    files = report.get("input_files") if isinstance(report, dict) else None
    hashes = report.get("input_file_sha256") if isinstance(report, dict) else None
    if not isinstance(files, dict) or not isinstance(hashes, dict) or not files:
        return _check(
            f"{label} input hash closure is current", False,
            "input_files/input_file_sha256 missing",
        )
    failures = []
    for key, relative in files.items():
        path = _safe_project_path(relative)
        expected = hashes.get(key)
        actual = _sha256(path) if path is not None else None
        if path is None or not isinstance(expected, str) or expected != actual:
            failures.append(
                f"{key}: path={relative!r}; embedded={expected}; current={actual}"
            )
    extra = sorted(set(hashes) - set(files))
    if extra:
        failures.append(f"unmapped hash keys={extra}")
    return _check(
        f"{label} input hash closure is current",
        not failures,
        "; ".join(failures) if failures else f"{len(files)} inputs match",
    )


def _stamp(path: Path) -> str:
    if not path.exists():
        return "missing"
    return datetime.fromtimestamp(path.stat().st_mtime).isoformat(
        sep=" ", timespec="seconds")


def _freshness(report_path: Path, dependencies: list[Path]):
    """Return (is_fresh, detail) for an evidence file and its real inputs."""
    missing = [path for path in dependencies if not path.exists()]
    if not report_path.exists():
        return False, f"missing {report_path.relative_to(ROOT)}"
    if missing:
        shown = ", ".join(str(path.relative_to(ROOT)) for path in missing)
        return False, f"missing dependencies: {shown}"
    newest = max(dependencies, key=lambda path: path.stat().st_mtime)
    lag = report_path.stat().st_mtime - newest.stat().st_mtime
    ok = lag >= -1.0  # tolerate coarse filesystem timestamp resolution
    state = "current" if ok else f"stale by {-lag:.1f}s"
    return ok, (f"{state}; report {_stamp(report_path)}, latest input "
                f"{newest.relative_to(ROOT)} {_stamp(newest)}")


def _check(label: str, ok: bool, detail: str):
    return {"label": label, "ok": bool(ok), "detail": str(detail)}


def _gate(checks):
    return bool(checks) and all(item["ok"] for item in checks)


def _fmt(value, places=3):
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        return "missing"
    return f"{value:.{places}f}"


def _cycle_gate(cycle, cycle_error, capture_error, meta):
    checks = [
        _check("canonical raw cycle report readable", cycle_error is None,
               cycle_error or f"{RAW_CYCLE.relative_to(ROOT)} parsed"),
        _check("canonical raw capture readable", capture_error is None,
               capture_error or f"{CAPTURE.relative_to(ROOT)} parsed"),
    ]
    checks.append(_check(
        "capture is unmodified upstream authority",
        bool(meta) and meta.get("capture_schema") == 4
        and meta.get("controller_mode") == "upstream"
        and meta.get("controller_adapter_sha256") is None
        and meta.get("winding_plan") is None,
        (f"schema={meta.get('capture_schema')}; "
         f"mode={meta.get('controller_mode')}; adapter="
         f"{meta.get('controller_adapter_sha256')}; winding_plan="
         f"{meta.get('winding_plan')}" if meta else
         "capture metadata missing")))

    if cycle is not None:
        cycle_checks = cycle.get("checks", {})
        failed = [name for name, result in cycle_checks.items()
                  if not isinstance(result, dict) or result.get("ok") is not True]
        declared = cycle.get("fail", [])
        declared_items = declared if isinstance(declared, list) else [
            "fail field is not a list"
        ]
        combined = list(dict.fromkeys([*failed, *declared_items]))
        capture = cycle.get("capture")
        capture = capture if isinstance(capture, dict) else {}
        actual_capture_sha = _sha256(CAPTURE)
        checks.extend([
            _check(
                "raw cycle declares the supported PASS schema",
                cycle.get("schema") == "captured-cycle-verification/v2"
                and cycle.get("status") == "PASS"
                and cycle.get("passed") is True,
                f"schema={cycle.get('schema')!r}; status="
                f"{cycle.get('status')!r}; passed={cycle.get('passed')!r}",
            ),
            _check(
                "raw cycle is bound to the canonical capture",
                capture.get("sha256") == actual_capture_sha
                and capture.get("capture_schema") == 4
                and capture.get("controller_mode") == "upstream",
                f"embedded={capture}; current_sha256={actual_capture_sha}",
            ),
            _check(
                "captured-cycle verifier passes every explicit check",
                bool(cycle_checks) and isinstance(declared, list)
                and not combined,
                ", ".join(map(str, combined)) if combined else
                f"{len(cycle_checks)} checks passed",
            ),
        ])
        checks.extend(_relative_source_hash_checks(
            "raw cycle", cycle.get("source_hashes")))
    return checks


def _shaft_wrap_compatibility_gate(wrap, wrap_error, meta):
    checks = [_check(
        "shaft-wrap regression evidence readable",
        wrap_error is None,
        wrap_error or f"{SHAFT_WRAP_REGRESSION.relative_to(ROOT)} parsed",
    )]
    if not isinstance(wrap, dict):
        return checks

    canonical = wrap.get("current_raw_capture")
    canonical = canonical if isinstance(canonical, dict) else {}
    upstream = wrap.get("current_upstream")
    upstream = upstream if isinstance(upstream, dict) else {}
    serial = wrap.get("independent_serial_position_evidence")
    serial = serial if isinstance(serial, dict) else {}
    patch = wrap.get("review_only_patch")
    patch = patch if isinstance(patch, dict) else {}
    gates = wrap.get("gates")
    gates = gates if isinstance(gates, dict) else {}

    expected_upstream_source = ROOT.parent / "winder" / "src" / "winding.py"
    expected_independent_capture = (
        OUT / "capture" / "independent_upstream_wrap_6039.jsonl"
    )
    expected_serial_capture = OUT / "capture" / "upstream_serial_twin_raw.jsonl"
    expected_serial_harness = ROOT / "sim" / "capture_upstream_serial_twin.py"
    expected_patch = REPORTS / "shaft_wrap_live_position_review_only.patch"
    current_commit = (meta or {}).get("winder_commit")
    evidence_gates = (
        "current_commit_and_clean_source_bound",
        "raw_capture_bound_to_current_commit",
        "raw_capture_reproduces_1p375_and_2p7916667_turns",
        "serial_position_evidence_bound",
        "serial_position_evidence_reproduces_regression",
        "pre_regression_commit_is_first_bad_parent",
        "pre_regression_source_proves_live_position_two_turn_request",
        "review_patch_applies_cleanly_without_being_applied",
    )
    checks.extend([
        _check(
            "shaft-wrap evidence declares the supported schema",
            wrap.get("schema") == "shaft-wrap-regression-evidence/v1"
            and wrap.get("evidence_bundle_complete") is True,
            f"schema={wrap.get('schema')!r}; complete="
            f"{wrap.get('evidence_bundle_complete')!r}",
        ),
        _check(
            "shaft-wrap evidence is bound to the canonical raw capture",
            _path_matches(canonical.get("path"), expected_independent_capture)
            and canonical.get("sha256") == _sha256(expected_independent_capture)
            and canonical.get("sha256") == _sha256(CAPTURE)
            and canonical.get("capture_schema") == 4
            and canonical.get("controller_mode") == "upstream"
            and canonical.get("winder_commit") == current_commit,
            f"embedded_sha={canonical.get('sha256')}; "
            f"current_sha={_sha256(CAPTURE)}; embedded_commit="
            f"{canonical.get('winder_commit')}; current_commit={current_commit}",
        ),
        _check(
            "shaft-wrap current upstream source binding is current",
            _path_matches(upstream.get("source_path"), expected_upstream_source)
            and upstream.get("source_sha256") == _sha256(expected_upstream_source)
            and upstream.get("commit") == current_commit
            and upstream.get("expected_commit") == current_commit
            and upstream.get("worktree_clean") is True,
            f"path={upstream.get('source_path')!r}; commit="
            f"{upstream.get('commit')!r}; sha={upstream.get('source_sha256')!r}",
        ),
        _check(
            "shaft-wrap independent serial-position evidence is current",
            _path_matches(serial.get("path"), expected_serial_capture)
            and serial.get("sha256") == _sha256(expected_serial_capture)
            and _path_matches(serial.get("harness_path"), expected_serial_harness)
            and serial.get("harness_sha256") == _sha256(expected_serial_harness)
            and serial.get("winder_commit") == current_commit,
            f"capture={serial.get('path')!r}; harness="
            f"{serial.get('harness_path')!r}; commit={serial.get('winder_commit')!r}",
        ),
        _check(
            "shaft-wrap review-only patch evidence is current and unapplied",
            _path_matches(patch.get("path"), expected_patch)
            and patch.get("sha256") == _sha256(expected_patch)
            and patch.get("applied") is False
            and patch.get("git_apply_check_pass") is True,
            f"path={patch.get('path')!r}; applied={patch.get('applied')!r}; "
            f"apply_check={patch.get('git_apply_check_pass')!r}",
        ),
        _check(
            "shaft-wrap regression integrity gates pass",
            all(gates.get(name) is True for name in evidence_gates),
            ", ".join(
                name for name in evidence_gates if gates.get(name) is not True
            ) or f"{len(evidence_gates)} evidence gates pass",
        ),
        _check(
            "untouched upstream executes exactly two turns in both shaft wraps",
            wrap.get("status") == "PASS"
            and wrap.get("release_authority") is True
            and gates.get("current_upstream_satisfies_two_turn_requirement") is True
            and gates.get("release_authorized") is True,
            f"status={wrap.get('status')!r}; release_authority="
            f"{wrap.get('release_authority')!r}; current_upstream_two_turn="
            f"{gates.get('current_upstream_satisfies_two_turn_requirement')!r}; "
            f"release_authorized={gates.get('release_authorized')!r}",
        ),
    ])
    return checks


def _integrated_player_gate(manifest, manifest_error, player_render,
                            player_render_error, catalog, catalog_error):
    checks = [
        _check(
            "selected integrated adapter manifest readable",
            manifest_error is None,
            manifest_error or f"{SELECTED_MANIFEST.relative_to(ROOT)} parsed",
        ),
        _check(
            "selected integrated player-render manifest readable",
            player_render_error is None,
            player_render_error
            or f"{SELECTED_PLAYER_RENDER.relative_to(ROOT)} parsed",
        ),
    ]
    checks.extend(_release_catalog_binding_checks(catalog, catalog_error, {
        "release-upstream-raw-capture": ("capture_authority", CAPTURE),
        "release-upstream-raw-cycle": ("evidence", RAW_CYCLE),
        "release-shaft-wrap-regression": (
            "fail_closed_regression_evidence", SHAFT_WRAP_REGRESSION,
        ),
        "release-integrated-adapter-manifest": (
            "selected_review_adapter", SELECTED_MANIFEST,
        ),
        "release-integrated-player-render": (
            "review_player_manifest", SELECTED_PLAYER_RENDER,
        ),
        "release-integrated-animation-glb": (
            "selected_review_animation", SELECTED_GLB,
        ),
        "release-integrated-player": (
            "selected_review_player", SELECTED_PLAYER,
        ),
    }))

    if isinstance(manifest, dict):
        checks.extend([
            _check(
                "selected integrated adapter declares the supported review schema",
                manifest.get("schema") == "integrated-candidate-player-adapter/v1"
                and manifest.get("status")
                == "REVIEW_ASSETS_READY_RELEASE_GATES_OPEN"
                and manifest.get("production_authorized") is False
                and manifest.get("canonical_promotion_authorized") is False,
                f"schema={manifest.get('schema')!r}; status="
                f"{manifest.get('status')!r}; production_authorized="
                f"{manifest.get('production_authorized')!r}",
            ),
            _check(
                "selected integrated adapter contract hash is valid",
                _canonical_field_hash_valid(manifest, "contract_sha256"),
                f"contract_sha256={manifest.get('contract_sha256')!r}",
            ),
            _source_hash_closure_check(
                "selected integrated adapter", manifest.get("source_hashes"),
            ),
        ])

    if isinstance(player_render, dict):
        route_sha = _sha256(SELECTED_ROUTE)
        glb_sha = _sha256(SELECTED_GLB)
        html_sha = _sha256(SELECTED_PLAYER)
        manifest_contract = (
            manifest.get("contract_sha256") if isinstance(manifest, dict)
            else None
        )
        checks.extend([
            _check(
                "selected integrated player render declares review-only schema",
                player_render.get("schema")
                == "integrated-candidate-player-render/v1"
                and player_render.get("review_only") is True
                and player_render.get("canonical_assets_modified") is False,
                f"schema={player_render.get('schema')!r}; review_only="
                f"{player_render.get('review_only')!r}; canonical_modified="
                f"{player_render.get('canonical_assets_modified')!r}",
            ),
            _check(
                "selected integrated player render binds exact input paths",
                _path_matches(
                    player_render.get("adapter_manifest"), SELECTED_MANIFEST,
                )
                and _path_matches(player_render.get("capture"), CAPTURE)
                and _path_matches(
                    player_render.get("conductor_route"), SELECTED_ROUTE,
                )
                and _path_matches(player_render.get("glb"), SELECTED_GLB)
                and _path_matches(player_render.get("html"), SELECTED_PLAYER),
                "adapter/capture/route/GLB/HTML must be the selected paths",
            ),
            _check(
                "selected integrated player render hash closure is current",
                player_render.get("adapter_contract_sha256") == manifest_contract
                and player_render.get("capture_sha256") == _sha256(CAPTURE)
                and player_render.get("conductor_route_artifact_sha256")
                == route_sha
                and player_render.get("glb_sha256") == glb_sha
                and player_render.get("html_sha256") == html_sha,
                f"manifest={manifest_contract}; capture={_sha256(CAPTURE)}; "
                f"route={route_sha}; glb={glb_sha}; html={html_sha}",
            ),
        ])

    player, player_error = _read_player_bundle()
    checks.append(_check(
        "selected integrated player bundle is readable and self-contained",
        player_error is None,
        player_error or "embedded state and GLB decoded",
    ))
    if player is not None:
        state = player["state"]
        capture_sha = _sha256(CAPTURE)
        glb_sha = player["disk_glb_sha256"]
        command_counts = state.get("commandCountsByAxis")
        command_counts = command_counts if isinstance(command_counts, dict) else {}
        all_axes = all(
            isinstance(command_counts.get(str(axis)), int)
            and command_counts[str(axis)] > 0
            for axis in range(4)
        )
        checks.extend([
            _check(
                "selected integrated player declares supported upstream schema",
                state.get("schema") == "winder-player/v3"
                and state.get("captureMode") == "upstream_raw",
                f"schema={state.get('schema')!r}; "
                f"captureMode={state.get('captureMode')!r}",
            ),
            _check(
                "selected integrated player is bound to canonical raw capture",
                state.get("captureSha256") == capture_sha,
                f"embedded={state.get('captureSha256')}; current={capture_sha}",
            ),
            _check(
                "selected integrated player embeds the exact selected GLB",
                glb_sha is not None
                and state.get("glbSha256") == glb_sha
                and player["embedded_glb_sha256"] == glb_sha
                and player["embedded_glb_bytes"] == player["disk_glb_bytes"],
                f"state={state.get('glbSha256')}; disk={glb_sha}; "
                f"embedded={player['embedded_glb_sha256']}",
            ),
            _check(
                "selected integrated player contains the complete four-axis cycle",
                all_axes
                and state.get("phaseCount") == 3
                and state.get("teethCount") == 24
                and state.get("turnsPerTooth") == 50
                and isinstance(state.get("coilStarts"), list)
                and len(state["coilStarts"]) == 24
                and isinstance(state.get("depositions"), list)
                and len(state["depositions"]) == 1200
                and isinstance(state.get("halfTurns"), list)
                and len(state["halfTurns"]) == 2400
                and isinstance(state.get("wraps"), list)
                and len(state["wraps"]) == 2,
                f"axis_counts={command_counts}; phases={state.get('phaseCount')}; "
                f"teeth={state.get('teethCount')}; turns="
                f"{state.get('turnsPerTooth')}",
            ),
            _check(
                "selected integrated player binds current route and terminal loci",
                state.get("conductorRouteArtifactSha256")
                == _sha256(SELECTED_ROUTE)
                and state.get("activeTerminalLociArtifactSha256")
                == _sha256(ACTIVE_LOCI),
                f"route={state.get('conductorRouteArtifactSha256')}; "
                f"current_route={_sha256(SELECTED_ROUTE)}; loci="
                f"{state.get('activeTerminalLociArtifactSha256')}; "
                f"current_loci={_sha256(ACTIVE_LOCI)}",
            ),
        ])
    return checks


def _rigid_pair_rows(report):
    rows = []
    for section in (
        "deposition_rigid_collision",
        "arbitrary_M1_and_progressive_copper_clearance",
    ):
        pairs = report.get(section, {}).get("pairs", {})
        if isinstance(pairs, dict):
            rows.extend((f"{section}.{name}", row)
                        for name, row in pairs.items()
                        if isinstance(row, dict))
    return rows


def _interference_gate(candidate, candidate_error, rigid, rigid_error,
                       catalog, catalog_error):
    checks = [
        _check(
            "selected integrated candidate report readable",
            candidate_error is None,
            candidate_error or f"{INTEGRATED_CANDIDATE.relative_to(ROOT)} parsed",
        ),
        _check(
            "selected active-sector rigid-motion audit readable",
            rigid_error is None,
            rigid_error or f"{ACTIVE_RIGID_AUDIT.relative_to(ROOT)} parsed",
        ),
    ]
    checks.extend(_release_catalog_binding_checks(catalog, catalog_error, {
        "release-integrated-candidate-report": (
            "selected_geometry_authority", INTEGRATED_CANDIDATE,
        ),
        "release-active-sector-audit": (
            "selected_rigid_motion_authority", ACTIVE_RIGID_AUDIT,
        ),
        "release-active-sector-loci": (
            "selected_terminal_locus_api", ACTIVE_LOCI,
        ),
        "release-launch-envelope-authority": (
            "launch_matrix_authority", LAUNCH_AUTHORITY,
        ),
    }))

    if isinstance(candidate, dict):
        geometry = candidate.get("geometry")
        geometry = geometry if isinstance(geometry, dict) else {}
        geometry_checks = geometry.get("checks")
        geometry_checks = (
            geometry_checks if isinstance(geometry_checks, dict) else {}
        )
        overlaps = geometry.get("unintended_overlaps_mm3")
        overlaps = overlaps if isinstance(overlaps, dict) else {}
        overlap_bad = {
            name: value for name, value in overlaps.items()
            if not isinstance(value, (int, float)) or value > 1.0e-6
        }
        release = candidate.get("release_gates")
        release = release if isinstance(release, dict) else {}
        checks.extend([
            _check(
                "selected integrated candidate declares supported geometry schema",
                candidate.get("schema") == "integrated-release-candidate/v1"
                and candidate.get("status")
                == "REFERENCE_GEOMETRY_PASS_RELEASE_GATES_OPEN",
                f"schema={candidate.get('schema')!r}; "
                f"status={candidate.get('status')!r}",
            ),
            _check(
                "selected integrated candidate report hash is valid",
                _canonical_report_hash_valid(candidate),
                f"report_sha256={candidate.get('report_sha256')!r}",
            ),
            _source_hash_closure_check(
                "selected integrated candidate", candidate.get("source_hashes"),
            ),
            _check(
                "selected integrated candidate passes every geometry check",
                bool(geometry_checks)
                and all(value is True for value in geometry_checks.values()),
                ", ".join(
                    name for name, value in geometry_checks.items()
                    if value is not True
                ) or f"{len(geometry_checks)} geometry checks pass",
            ),
            _check(
                "selected integrated candidate has zero unintended overlap",
                bool(overlaps) and not overlap_bad,
                ", ".join(f"{k}={v}" for k, v in overlap_bad.items())
                if overlap_bad else f"{len(overlaps)} overlap pairs clear",
            ),
            _check(
                "selected candidate consumes current rigid-sweep results",
                release.get("targeted_reference_pose_geometry") is True
                and release.get("full_raw_cycle_collision_regenerated") is True
                and release.get(
                    "all_2400_deposition_terminal_routes_exact_and_clear"
                ) is True,
                "targeted reference, full raw rigid sweep and 2400 loci "
                "must all pass",
            ),
        ])

    if isinstance(rigid, dict):
        canonical = rigid.get("canonical_run")
        canonical = canonical if isinstance(canonical, dict) else {}
        full = rigid.get("full_raw_rigid_motion")
        full = full if isinstance(full, dict) else {}
        required_classes = full.get("required_motion_classes_present")
        required_classes = (
            required_classes if isinstance(required_classes, dict) else {}
        )
        full_pairs = full.get("pairs")
        full_pairs = full_pairs if isinstance(full_pairs, dict) else {}
        pair_rows = _rigid_pair_rows(rigid)
        bad_pairs = []
        minima = []
        for name, row in pair_rows:
            clearance = row.get("minimum_clearance_mm")
            target = row.get("clearance_target_mm")
            if isinstance(clearance, (int, float)):
                minima.append(clearance)
            if (row.get("status") != "PASS"
                    or row.get("collision_count") != 0
                    or not isinstance(clearance, (int, float))
                    or not isinstance(target, (int, float))
                    or clearance < target):
                bad_pairs.append(name)
        outboard = rigid.get("outboard_yoke_packaging")
        outboard = outboard if isinstance(outboard, dict) else {}
        yoke = rigid.get("front_plane_yoke_full_M2_clearance")
        yoke = yoke if isinstance(yoke, dict) else {}
        if isinstance(yoke.get("minimum_clearance_mm"), (int, float)):
            minima.append(yoke["minimum_clearance_mm"])
        checks.extend([
            _check(
                "active-sector audit declares supported selected-rigid schema",
                rigid.get("schema")
                == "carriage-active-sector-terminal-guide-audit/v1"
                and rigid.get("assembly_geometry_integration_authorized") is True,
                f"schema={rigid.get('schema')!r}; geometry_authorized="
                f"{rigid.get('assembly_geometry_integration_authorized')!r}",
            ),
            _check(
                "active-sector audit report hash is valid",
                _canonical_report_hash_valid(rigid),
                f"report_sha256={rigid.get('report_sha256')!r}",
            ),
            _source_hash_closure_check(
                "active-sector rigid audit", rigid.get("source_hashes"),
            ),
            _check(
                "active-sector rigid audit binds the canonical raw capture",
                canonical.get("capture_sha256") == _sha256(CAPTURE)
                and canonical.get("pass_count") == 24
                and canonical.get("locus_count") == 2400,
                f"capture={canonical.get('capture_sha256')}; passes="
                f"{canonical.get('pass_count')}; loci={canonical.get('locus_count')}",
            ),
            _check(
                "all selected rigid clearance pairs meet the 2 mm target",
                bool(pair_rows) and not bad_pairs,
                ", ".join(bad_pairs) if bad_pairs else
                f"{len(pair_rows)} pairs; worst {_fmt(min(minima) if minima else None)} mm",
            ),
            _check(
                "full raw selected-rigid motion covers every required class",
                full.get("status") == "PASS"
                and full.get("sample_count", 0) > 0
                and bool(required_classes)
                and all(value is True for value in required_classes.values())
                and bool(full_pairs)
                and len([row for row in full_pairs.values()
                         if isinstance(row, dict)]) == len(full_pairs)
                and all(
                    row.get("status") == "PASS"
                    and row.get("collision_count") == 0
                    for row in full_pairs.values() if isinstance(row, dict)
                ),
                f"samples={full.get('sample_count')}; classes="
                f"{required_classes}; pairs={len(full_pairs)}",
            ),
            _check(
                "integrated yoke and hardware packaging sweeps pass",
                outboard.get("status") == "PASS"
                and yoke.get("status") == "PASS"
                and yoke.get("collision_count") == 0
                and isinstance(yoke.get("minimum_clearance_mm"), (int, float))
                and isinstance(yoke.get("clearance_target_mm"), (int, float))
                and yoke.get("minimum_clearance_mm")
                >= yoke.get("clearance_target_mm"),
                f"outboard={outboard.get('status')!r}; yoke="
                f"{yoke.get('status')!r}; clearance="
                f"{yoke.get('minimum_clearance_mm')}",
            ),
        ])
    return checks


def _launch_certificate_closure_check(launch):
    rows = launch.get("required_certificates") if isinstance(launch, dict) else None
    rows = rows if isinstance(rows, list) else []
    failures = []
    ids = []
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            failures.append(f"row {index} is not an object")
            continue
        case_id = row.get("case_id")
        ids.append(case_id)
        path = _safe_project_path(row.get("certificate_path"))
        actual = _sha256(path) if path is not None else None
        if path is None or actual != row.get("certificate_sha256"):
            failures.append(f"{case_id}: certificate file/hash mismatch")
            continue
        certificate, error = _read_json(path)
        if error or not isinstance(certificate, dict):
            failures.append(f"{case_id}: {error or 'certificate is not an object'}")
            continue
        if not _canonical_field_hash_valid(
                certificate, "certificate_payload_sha256"):
            failures.append(f"{case_id}: certificate payload hash invalid")
        sources = certificate.get("sources")
        if not isinstance(sources, dict) or not sources:
            failures.append(f"{case_id}: source closure missing")
        else:
            for relative, expected in sources.items():
                source = _safe_project_path(relative)
                if source is None or _sha256(source) != expected:
                    failures.append(f"{case_id}: source drift {relative}")
        artifacts = certificate.get("artifacts")
        if not isinstance(artifacts, dict) or not artifacts:
            failures.append(f"{case_id}: artifact closure missing")
        else:
            for key, artifact in artifacts.items():
                artifact = artifact if isinstance(artifact, dict) else {}
                artifact_path = _safe_project_path(artifact.get("path"))
                if (artifact_path is None
                        or _sha256(artifact_path) != artifact.get("sha256")):
                    failures.append(f"{case_id}: artifact drift {key}")
    if len(rows) != 24:
        failures.append(f"required rows={len(rows)}, expected=24")
    if len(set(ids)) != len(ids):
        failures.append("duplicate launch case ids")
    return _check(
        "all 24 launch certificates have current source/artifact closure",
        not failures,
        "; ".join(failures[:20]) if failures else "24 certificate bundles current",
    )


def _launch_coverage_gate(launch, launch_error, catalog, catalog_error):
    checks = [_check(
        "launch-envelope authority readable",
        launch_error is None,
        launch_error or f"{LAUNCH_AUTHORITY.relative_to(ROOT)} parsed",
    )]
    checks.extend(_release_catalog_binding_checks(catalog, catalog_error, {
        "release-launch-envelope-authority": (
            "launch_matrix_authority", LAUNCH_AUTHORITY,
        ),
    }))
    if not isinstance(launch, dict):
        return checks
    audit = launch.get("audit_source")
    audit = audit if isinstance(audit, dict) else {}
    audit_path = _safe_project_path(audit.get("path"))
    summary = launch.get("summary")
    summary = summary if isinstance(summary, dict) else {}
    release = launch.get("release_gates")
    release = release if isinstance(release, dict) else {}
    checks.extend([
        _check(
            "launch authority declares supported schema and valid payload hash",
            launch.get("schema") == "launch-envelope-authority/v1"
            and _canonical_field_hash_valid(launch, "report_payload_sha256"),
            f"schema={launch.get('schema')!r}; payload_sha="
            f"{launch.get('report_payload_sha256')!r}",
        ),
        _check(
            "launch authority source binding is current",
            audit_path == (ROOT / "sim" / "launch_envelope_authority.py").resolve()
            and audit.get("sha256") == _sha256(audit_path),
            f"path={audit.get('path')!r}; embedded={audit.get('sha256')!r}; "
            f"current={_sha256(audit_path) if audit_path else None}",
        ),
        _launch_certificate_closure_check(launch),
        _check(
            "all required OD28--65 launch corners are authorized",
            launch.get("status") == "PASS"
            and launch.get("production_authorized") is True
            and summary.get("required") == 24
            and summary.get("passing") == 24
            and summary.get("missing") == 0
            and summary.get("invalid_or_stale") == 0
            and release.get("all_required_launch_corner_certificates_pass")
            is True
            and release.get("every_corner_binds_current_required_sources")
            is True
            and release.get("production_authorized") is True,
            f"status={launch.get('status')!r}; production_authorized="
            f"{launch.get('production_authorized')!r}; summary={summary}",
        ),
    ])
    return checks


def _wire_gate(conductor, conductor_error, catalog, catalog_error):
    checks = [_check(
        "full-cycle continuous-conductor authority readable",
        conductor_error is None,
        conductor_error
        or f"{FULL_CONDUCTOR_AUTHORITY.relative_to(ROOT)} parsed",
    )]
    checks.extend(_release_catalog_binding_checks(catalog, catalog_error, {
        "release-full-cycle-conductor-authority": (
            "selected_flexible_conductor_authority", FULL_CONDUCTOR_AUTHORITY,
        ),
    }))
    if not isinstance(conductor, dict):
        return checks
    capture = conductor.get("capture_evidence")
    capture = capture if isinstance(capture, dict) else {}
    capture_gates = capture.get("gates")
    capture_gates = capture_gates if isinstance(capture_gates, dict) else {}
    coverage = conductor.get("coverage_result")
    coverage = coverage if isinstance(coverage, dict) else {}
    matrix = conductor.get("required_state_matrix")
    matrix = matrix if isinstance(matrix, dict) else {}
    proven = []

    def collect_proven(value, prefix=""):
        if isinstance(value, dict):
            for key, child in value.items():
                child_prefix = f"{prefix}.{key}" if prefix else key
                if key == "proven":
                    proven.append((prefix or "matrix", child))
                else:
                    collect_proven(child, child_prefix)
        elif isinstance(value, list):
            for index, child in enumerate(value):
                collect_proven(child, f"{prefix}[{index}]")

    collect_proven(matrix)
    input_hashes = conductor.get("input_file_sha256")
    input_hashes = input_hashes if isinstance(input_hashes, dict) else {}
    issues = conductor.get("issues")
    checks.extend([
        _check(
            "continuous-conductor report declares supported schema",
            conductor.get("schema")
            == "full-cycle-continuous-conductor-authority-audit/v1",
            f"schema={conductor.get('schema')!r}",
        ),
        _check(
            "continuous-conductor report hash is valid",
            _canonical_report_hash_valid(conductor),
            f"report_sha256={conductor.get('report_sha256')!r}",
        ),
        _source_hash_closure_check(
            "continuous-conductor authority", conductor.get("source_hashes"),
        ),
        _bound_input_hash_check("continuous-conductor authority", conductor),
        _check(
            "continuous-conductor authority binds selected integrated inputs",
            input_hashes.get("capture") == _sha256(CAPTURE)
            and input_hashes.get("guide_audit") == _sha256(ACTIVE_RIGID_AUDIT)
            and input_hashes.get("loci") == _sha256(ACTIVE_LOCI)
            and input_hashes.get("presentation") == _sha256(SELECTED_ROUTE),
            f"capture={input_hashes.get('capture')}; guide="
            f"{input_hashes.get('guide_audit')}; loci="
            f"{input_hashes.get('loci')}; presentation="
            f"{input_hashes.get('presentation')}",
        ),
        _check(
            "continuous-conductor capture includes the exact required raw cycle",
            capture.get("capture_schema") == 4
            and capture.get("controller_mode") == "upstream"
            and capture.get("controller_adapter_sha256") is None
            and capture.get("winding_pass_count") == 24
            and bool(capture_gates)
            and all(value is True for value in capture_gates.values()),
            ", ".join(
                name for name, value in capture_gates.items()
                if value is not True
            ) or f"{len(capture_gates)} raw-cycle gates pass",
        ),
        _check(
            "every required flexible-conductor state family is proven",
            bool(proven) and all(value is True for _, value in proven),
            ", ".join(name for name, value in proven if value is not True)
            or f"{len(proven)} state families proven",
        ),
        _check(
            "full-cycle flexible conductor is production-authorized",
            conductor.get("audit_integrity_status") == "PASS"
            and conductor.get("status") == "PASS"
            and conductor.get("production_authorized") is True
            and coverage.get("presentation_timeline_fully_classified") is True
            and coverage.get("full_cycle_physical_quasistatic_proven") is True
            and coverage.get("physically_authorized_timeline_fraction") == 1.0
            and isinstance(issues, list) and not issues,
            f"audit_integrity={conductor.get('audit_integrity_status')!r}; "
            f"status={conductor.get('status')!r}; production_authorized="
            f"{conductor.get('production_authorized')!r}; coverage={coverage}; "
            f"issues={len(issues) if isinstance(issues, list) else 'missing'}",
        ),
    ])
    return checks


def _mapping_gate(cycle, events, meta, capture_error, manifest,
                  manifest_error):
    settings = OUT / "settings.yml"
    checks = [
        _check("generated settings exists", settings.exists() and
               settings.stat().st_size > 0,
               f"{settings.stat().st_size} bytes" if settings.exists()
               else "missing out/settings.yml"),
        _check("capture metadata readable", capture_error is None and
               meta is not None, capture_error or "meta event found"),
        _check("CAD manifest readable", manifest_error is None,
               manifest_error or "manifest.json parsed"),
    ]

    if settings.exists() and meta:
        digest = hashlib.sha256(settings.read_bytes()).hexdigest()
        captured = meta.get("settings_sha256")
        cycle_hash = (cycle or {}).get("checks", {}).get(
            "settings hash current", {}).get("value")
        checks.append(_check("capture used the current generated settings",
                             digest == captured == cycle_hash,
                             f"current={digest}, captured={captured}, "
                             f"cycle={cycle_hash}"))
    if cycle is not None:
        completed = cycle.get("checks", {}).get(
            "cycle completion marker", {}).get("ok") is True
        checks.append(_check("software completed the captured cycle",
                             completed,
                              str(cycle.get("checks", {}).get(
                                  "cycle completion marker", {}).get("value"))))
    checks.append(_check(
        "mapping evidence uses canonical upstream controller mode",
        bool(meta) and meta.get("capture_schema") == 4
        and meta.get("controller_mode") == "upstream"
        and meta.get("controller_adapter_sha256") is None,
        (f"schema={meta.get('capture_schema')}; mode="
         f"{meta.get('controller_mode')}; adapter="
         f"{meta.get('controller_adapter_sha256')}" if meta else
         "capture metadata missing"),
    ))

    m0_targets = [event.get("model_target") for event in events
                  if event.get("e") == "cmd" and event.get("m") == 0]
    m0_targets = [float(value) for value in m0_targets
                  if isinstance(value, (int, float))]
    try:
        sys.path.insert(0, str(ROOT / "cad"))
        from params import PARAMS as params
        if not manifest:
            raise ValueError("manifest unavailable")
        mapping_match = (
            math.isclose(manifest.get("mm_per_rad_m0", math.nan),
                         params.mm_per_rad, rel_tol=0, abs_tol=1e-12) and
            math.isclose(manifest.get("m0_home_standoff", math.nan),
                         params.m0_home_standoff, rel_tol=0, abs_tol=1e-12))
        checks.append(_check("manifest transmission matches CAD parameters",
                             mapping_match,
                             f"{manifest.get('mm_per_rad_m0')} mm/rad; "
                             f"home {manifest.get('m0_home_standoff')} mm"))
        z_values = [params.stator_axis_z(target) for target in m0_targets]
        travel_ok = bool(z_values) and min(z_values) >= params.m0_axis_z_min and \
            max(z_values) <= params.m0_home_standoff and all(
                abs(target) * params.mm_per_rad <= params.m0_travel + 1e-9
                for target in m0_targets)
        checks.append(_check("all captured M0 targets fit physical travel",
                             travel_ok,
                             (f"axis Z {_fmt(min(z_values))}.."
                              f"{_fmt(max(z_values))} mm; limits "
                              f"{_fmt(params.m0_axis_z_min)}.."
                              f"{_fmt(params.m0_home_standoff)} mm")
                             if z_values else "no M0 command targets"))
    except Exception as exc:  # report the missing proof; never assume success
        checks.append(_check("CAD transmission/travel evaluation", False,
                             f"could not evaluate: {exc}"))
    return checks


def _loads_gate(loads, loads_error):
    source_inputs = [ROOT / "cad" / name for name in
                     ("loads.py", "assembly.py", "printed.py", "cots.py",
                      "params.py")]
    source_inputs.extend((
        ROOT / "sim" / "m2_normal_goal_drive_selection.py",
        REPORTS / "m2_normal_goal_drive_selection.json",
    ))
    fresh, detail = _freshness(REPORTS / "loads.json", source_inputs)
    checks = [
        _check("loads report readable", loads_error is None,
               loads_error or "loads.json parsed"),
        _check("loads evidence current", fresh, detail),
    ]
    if loads is not None:
        checks.append(_check(
            "loads report uses selected-authority schema",
            loads.get("schema") == "machine-loads/v2",
            str(loads.get("schema", "missing")),
        ))
        m2 = loads.get("m2", {})
        selected = m2.get("governing_selected_authority", {})
        margins = {
            "M0": loads.get("m0", {}).get("margin"),
            "M1": loads.get("m1", {}).get("margin"),
            "M2 selected 36 V curve at 300 RPM": selected.get(
                "available_to_required_multiple"
            ),
            "M2 selected P30/210-3GT transmission": selected.get(
                "transmission", {}
            ).get("allowable_to_required_multiple"),
        }
        bad = {name: value for name, value in margins.items()
               if not isinstance(value, (int, float)) or value < 2.0}
        details = ", ".join(f"{name}={_fmt(value, 2)}x"
                            for name, value in margins.items())
        checks.append(_check("selected motor and transmission load margins are at least 2x",
                             not bad, details))
        m2_name = str(loads.get("motors", {}).get("m2", ""))
        nema17 = (
            m2_name == "Leadshine CS-M21708 closed-loop NEMA17"
            and selected.get("motor") == m2_name
        )
        checks.append(_check("M2 selection satisfies GOAL NEMA17 constraint",
                             nema17, m2_name or "motor selection missing"))
        selected_stack = (
            selected.get("motor") == "Leadshine CS-M21708 closed-loop NEMA17"
            and selected.get("driver") == "Leadshine CS-D508"
            and selected.get("supply") == "Leadshine LSP-360-36"
            and selected.get("ratio") == 1.0
            and selected.get("curve_condition") == "36 VDC, RMS 2.5 A"
        )
        checks.append(_check(
            "DoD #5 uses the selected Leadshine M2 stack",
            selected_stack,
            (f"motor={selected.get('motor')}; driver={selected.get('driver')}; "
             f"supply={selected.get('supply')}; "
             f"condition={selected.get('curve_condition')}")
        ))
        legacy = m2.get("legacy_baseline", {})
        legacy_is_non_governing = (
            legacy.get("non_governing") is True
            and legacy.get("role") == "historical_non_governing_baseline"
            and "McMaster 6627T421" in str(legacy.get("motor", ""))
            and "McMaster 6627T421" not in m2_name
        )
        checks.append(_check(
            "retired McMaster M2 curve is non-governing",
            legacy_is_non_governing,
             str(legacy.get("reason_non_governing", "missing legacy boundary")),
        ))
        release = m2.get("release_gates", {})
        dod5 = loads.get("definition_of_done_5", {})
        dod5_pass = (
            loads.get("static_sizing_pass") is True
            and loads.get("analytical_order_input_ready") is True
            and dod5.get("status") == "PASS"
            and dod5.get("pass") is True
            and dod5.get("requires_post_purchase_hardware") is False
            and release.get("definition_of_done_5_pass") is True
        )
        checks.append(_check(
            "DoD #5 analytical loads authority passes",
            dod5_pass,
            (f"static_sizing_pass={loads.get('static_sizing_pass')}; "
             f"analytical_order_input_ready="
             f"{loads.get('analytical_order_input_ready')}; "
             f"dod5={dod5.get('status')!r}"),
        ))
        production_proof = (
            release.get("driver_current_configuration_verified") is True
            and release.get("installed_hot_dyno_verified") is True
            and release.get("production_authorized") is True
            and loads.get("production_authorized") is True
        )
        qualification = loads.get("post_purchase_motion_qualification", {})
        qualification_boundary = (
            qualification.get("required_for_definition_of_done_5") is False
            and qualification.get("required_before_energized_motion") is True
            and qualification.get("status")
            == ("PASS" if production_proof else "BLOCKED")
            and loads.get("production_authorized") is production_proof
        )
        checks.append(_check(
            "post-purchase M2 production qualification is separate and fail-closed",
            qualification_boundary,
            ("driver_configured="
             f"{release.get('driver_current_configuration_verified')}; "
             f"hot_dyno={release.get('installed_hot_dyno_verified')}; "
             f"production_authorized={loads.get('production_authorized')}; "
             f"qualification={qualification.get('status')!r}")
        ))
        flyer = loads.get("flyer", {})
        inertia_ok = all(isinstance(flyer.get(key), (int, float))
                         for key in ("mass_g", "izz_kgm2"))
        checks.append(_check("flyer mass and inertia are reported", inertia_ok,
                             f"mass {_fmt(flyer.get('mass_g'), 1)} g; "
                             f"Izz {flyer.get('izz_kgm2', 'missing')} kg m^2"))
    return checks


def _buildability_gate(build, build_error):
    fresh, detail = _freshness(
        REPORTS / "buildability.json",
        [ROOT / "cad" / name for name in (
            "buildability.py", "printed.py", "params.py",
            "integrated_release_candidate.py",
            "retained_flyer_peek_guide_successor.py",
            "carriage_active_sector_terminal_guide.py",
            "m2_drive_successor_review.py",
            "permanent_cap_offset_spoke_retained_review.py",
        )])
    checks = [
        _check("buildability report readable", build_error is None,
               build_error or "buildability.json parsed"),
        _check("buildability evidence current", fresh, detail),
    ]
    if build is not None:
        parts = build.get("parts")
        parts = parts if isinstance(parts, list) else []
        disconnected = build.get("single_solid_check") != "pass"
        checks.append(_check("every printed part is one solid",
                             bool(parts) and not disconnected,
                             str(build.get("single_solid_check", "missing"))))
        mesh_bad = [part.get("part", "unnamed") for part in parts
                    if not isinstance(part.get("mesh"), dict)
                    or part["mesh"].get("ok") is not True]
        checks.append(_check(
            "every serialized STL is watertight and manifold",
            bool(parts) and build.get("mesh_check") == "pass" and not mesh_bad,
            (", ".join(mesh_bad) if mesh_bad else
             f"mesh_check={build.get('mesh_check', 'missing')!r}"),
        ))
        bed_bad = [part.get("part", "unnamed") for part in parts
                   if part.get("bed_fit") is not True]
        checks.append(_check("every printed part fits the bed",
                             bool(parts) and not bed_bad,
                             ", ".join(bed_bad) if bed_bad else
                             f"{len(parts)} parts fit"))
        supports_complete = bool(parts) and all(
            part.get("supports") in {"yes", "no"} for part in parts)
        support_names = [part.get("part", "unnamed") for part in parts
                         if part.get("supports") == "yes"]
        checks.append(_check("support needs are flagged per part",
                             supports_complete,
                             ", ".join(support_names) if support_names else
                             "none flagged"))
        orientation_complete = bool(parts) and all(part.get("note") for part in parts)
        checks.append(_check("print orientation/note exists per part",
                             orientation_complete,
                             f"{sum(bool(part.get('note')) for part in parts)}/"
                             f"{len(parts)} documented"))
        wall_note = str(build.get("wall_note", ""))
        wall_flagged = bool(re.search(r"\b(?:flagged|fail(?:ed)?|below)\b",
                                      wall_note, re.IGNORECASE))
        checks.append(_check("wall-thickness audit has no flagged violation",
                             bool(wall_note) and not wall_flagged,
                             wall_note or "wall_note missing"))
        machining = build.get("machining")
        per_part_machining = bool(parts) and all("machining" in part
                                                for part in parts)
        checks.append(_check("machining needs are explicitly audited",
                             machining is not None or per_part_machining,
                             "top-level machining field" if machining is not None
                             else ("per-part fields" if per_part_machining else
                                   "no machining field in report")))
    return checks


def _section(lines, number, title, checks, extra=None):
    passed = _gate(checks)
    lines.extend([f"## DoD #{number} — {title}: "
                  f"{'PASS' if passed else 'FAIL'}", ""])
    for item in checks:
        lines.append(f"- [{'PASS' if item['ok'] else 'FAIL'}] "
                     f"{item['label']}: {item['detail']}")
    if extra:
        lines.extend(["", *extra])
    lines.append("")
    return passed


def main():
    REPORTS.mkdir(parents=True, exist_ok=True)
    cycle, cycle_error = _read_json(RAW_CYCLE)
    wrap, wrap_error = _read_json(SHAFT_WRAP_REGRESSION)
    selected_manifest, selected_manifest_error = _read_json(SELECTED_MANIFEST)
    player_render, player_render_error = _read_json(SELECTED_PLAYER_RENDER)
    candidate, candidate_error = _read_json(INTEGRATED_CANDIDATE)
    rigid, rigid_error = _read_json(ACTIVE_RIGID_AUDIT)
    conductor, conductor_error = _read_json(FULL_CONDUCTOR_AUTHORITY)
    launch, launch_error = _read_json(LAUNCH_AUTHORITY)
    catalog, catalog_error = _read_json(RELEASE_CATALOG)
    loads, loads_error = _read_json(REPORTS / "loads.json")
    build, build_error = _read_json(REPORTS / "buildability.json")
    manifest, manifest_error = _read_json(LINKS / "manifest.json")
    events, meta, capture_error = _read_capture()

    gates = {
        1: [
            *_cycle_gate(cycle, cycle_error, capture_error, meta),
            *_shaft_wrap_compatibility_gate(wrap, wrap_error, meta),
            *_integrated_player_gate(
                selected_manifest, selected_manifest_error,
                player_render, player_render_error,
                catalog, catalog_error,
            ),
        ],
        2: [
            *_interference_gate(
                candidate, candidate_error, rigid, rigid_error,
                catalog, catalog_error,
            ),
            *_launch_coverage_gate(
                launch, launch_error, catalog, catalog_error,
            ),
        ],
        3: _wire_gate(
            conductor, conductor_error, catalog, catalog_error,
        ),
        4: _mapping_gate(cycle, events, meta, capture_error, manifest,
                         manifest_error),
        5: _loads_gate(loads, loads_error),
        6: _buildability_gate(build, build_error),
    }

    stator = (selected_manifest or {}).get("stator", {})
    commit = ((cycle or {}).get("checks", {}).get(
        "upstream commit recorded", {}).get("value") or
        (meta or {}).get("winder_commit") or "unavailable")
    lines = [
        "# Validation Report — 4-Axis Stator Flyer Winder",
        "",
        "This report is generated from the selected, hash-bound JSON evidence. "
        "A gate cannot pass when governing evidence is missing, stale by "
        "content hash, or contains a failed check. Legacy controller-baseline "
        "artifacts are listed separately and do not decide DoD #1--3.",
        "",
        f"Stator under test: OD {stator.get('od', 'unknown')} mm, "
        f"{stator.get('slots', 'unknown')} slots, stack "
        f"{stator.get('stack', 'unknown')} mm. Captured upstream commit: "
        f"`{commit}`.",
        "",
        "| Definition of Done gate | result |",
        "|---|---|",
    ]
    titles = {
        1: "Full-cycle digital twin run",
        2: "Interference proof",
        3: "Wire-path verification",
        4: "Geometry-to-config mapping",
        5: "Loads sanity check",
        6: "Buildability audit",
    }
    for number in range(1, 7):
        lines.append(f"| #{number} {titles[number]} | "
                     f"{'PASS' if _gate(gates[number]) else 'FAIL'} |")
    lines.append("")

    wrap_extra = []
    if cycle and cycle.get("shaft_wraps"):
        wrap_extra = ["Shaft-wrap measurements:", "",
                      "| wrap | physical M1 turns | result |",
                      "|---|---:|---|"]
        for wrap in cycle["shaft_wraps"]:
            wrap_extra.append(
                f"| {wrap.get('index')} | {_fmt(wrap.get('turns'), 6)} | "
                f"{'PASS' if wrap.get('ok') else 'FAIL'} |")
    results = []
    results.append(_section(lines, 1, titles[1], gates[1], wrap_extra))

    clearance_extra = []
    rigid_rows = _rigid_pair_rows(rigid or {})
    if rigid_rows:
        clearance_extra = ["Selected integrated rigid-clearance minima:", "",
                           "| pair | clearance (mm) | target (mm) |",
                           "|---|---:|---:|"]
        for name, item in rigid_rows:
            clearance_extra.append(
                f"| {name} | {_fmt(item.get('minimum_clearance_mm'))} | "
                f"{_fmt(item.get('clearance_target_mm'))} |")
    results.append(_section(lines, 2, titles[2], gates[2], clearance_extra))

    wire_extra = []
    if conductor:
        coverage = conductor.get("coverage_result", {})
        wire_extra = [
            "Full-cycle physical/quasi-static conductor coverage: "
            f"{_fmt(coverage.get('physically_authorized_timeline_fraction'), 6)}; "
            f"authorized intervals: "
            f"{coverage.get('physically_authorized_continuous_interval_count', 'missing')}.",
        ]
    results.append(_section(lines, 3, titles[3], gates[3], wire_extra))
    results.append(_section(lines, 4, titles[4], gates[4]))
    results.append(_section(lines, 5, titles[5], gates[5]))
    results.append(_section(lines, 6, titles[6], gates[6]))

    failed = []
    for number, checks in gates.items():
        for item in checks:
            if not item["ok"]:
                failed.append(f"DoD #{number}: {item['label']} — {item['detail']}")
    lines.extend(["## Remaining blockers", ""])
    if failed:
        lines.extend(f"- {item}" for item in failed)
    else:
        lines.append("None in the currently generated evidence.")
    lines.extend([
        "",
        "## Non-governing baseline diagnostics",
        "",
        "The legacy `out/links` player, raw clearance/wirepath, static-link "
        "and focused belt reports remain useful for controller-baseline "
        "comparison. Their timestamps and hashes do not decide DoD #1--3; "
        "the selected integrated authorities above do.",
        "",
        "## What this simulation does not prove",
        "",
        "Real wire-tension dynamics, sag and snagging, layering neatness, "
        "enamel abrasion, belt resonance, printed-part creep/fatigue, and "
        "thermal behavior still require hardware validation.",
        "",
    ])

    limitations = [
        "Real wire-tension dynamics, sag and snagging, layering neatness, "
        "enamel abrasion, belt resonance, printed-part creep/fatigue, and "
        "thermal behavior still require hardware validation."
    ]
    validation_passed = all(results)
    governing_evidence_paths = {
        "capture": CAPTURE,
        "cycle": RAW_CYCLE,
        "shaft_wrap_regression": SHAFT_WRAP_REGRESSION,
        "selected_adapter_manifest": SELECTED_MANIFEST,
        "selected_player_render": SELECTED_PLAYER_RENDER,
        "selected_animation_glb": SELECTED_GLB,
        "selected_animation_player": SELECTED_PLAYER,
        "selected_conductor_route": SELECTED_ROUTE,
        "selected_integrated_candidate": INTEGRATED_CANDIDATE,
        "selected_active_rigid_audit": ACTIVE_RIGID_AUDIT,
        "selected_active_loci": ACTIVE_LOCI,
        "full_cycle_conductor_authority": FULL_CONDUCTOR_AUTHORITY,
        "launch_envelope_authority": LAUNCH_AUTHORITY,
        "release_catalog": RELEASE_CATALOG,
        "loads": REPORTS / "loads.json",
        "buildability": REPORTS / "buildability.json",
    }
    baseline_evidence_paths = {
        "legacy_links_manifest": LINKS / "manifest.json",
        "legacy_raw_animation_glb": RAW_GLB,
        "legacy_raw_animation_player": RAW_PLAYER,
        "legacy_raw_clearance": RAW_CLEARANCE,
        "legacy_static_audit": REPORTS / "static_audit.json",
        "legacy_belt_audit": REPORTS / "belt_audit.json",
        "legacy_raw_wirepath": RAW_WIREPATH,
        "legacy_winding_tooling_authority": TOOLING_AUTHORITY,
    }
    evidence_paths = {**governing_evidence_paths, **baseline_evidence_paths}
    machine_report = {
        "schema": VALIDATION_SCHEMA,
        "status": "PASS" if validation_passed else "FAIL",
        "passed": validation_passed,
        "upstream_commit": commit,
        "canonical_capture": {
            "path": CAPTURE.relative_to(ROOT).as_posix(),
            "sha256": _sha256(CAPTURE),
            "schema": (meta or {}).get("capture_schema"),
            "controller_mode": (meta or {}).get("controller_mode"),
            "controller_adapter_sha256": (meta or {}).get(
                "controller_adapter_sha256"),
        },
        "stator": stator,
        "dod": {
            str(number): {
                "title": titles[number],
                "passed": _gate(gates[number]),
                "checks": gates[number],
            }
            for number in range(1, 7)
        },
        "remaining_blockers": failed,
        "limitations": limitations,
        "evidence": {
            name: path.relative_to(ROOT).as_posix()
            for name, path in evidence_paths.items()
        },
        "evidence_authority": {
            **{name: "GOVERNING" for name in governing_evidence_paths},
            **{name: "BASELINE_DIAGNOSTIC_ONLY"
               for name in baseline_evidence_paths},
        },
        "source_hashes": {
            "sim/report.py": _sha256(Path(__file__)),
            **{
                path.relative_to(ROOT).as_posix(): _sha256(path)
                for path in governing_evidence_paths.values()
            },
        },
        "baseline_diagnostics": {
            "authority": "NON_GOVERNING",
            "artifacts": {
                name: {
                    "path": path.relative_to(ROOT).as_posix(),
                    "present": path.is_file(),
                }
                for name, path in baseline_evidence_paths.items()
            },
            "note": (
                "Presence is informational only; timestamps and hashes do not "
                "decide DoD #1--3 or validation source freshness."
            ),
        },
    }
    json_output = REPORTS / "validation.json"
    json_output.write_text(
        json.dumps(machine_report, indent=2) + "\n", encoding="utf-8")
    output = REPORTS / "validation.md"
    output.write_text("\n".join(lines), encoding="utf-8")
    print("wrote", json_output)
    print("wrote", output)
    print("DoD:", ", ".join(
        f"{number} {'PASS' if _gate(gates[number]) else 'FAIL'}"
        for number in range(1, 7)))
    if failed:
        print("remaining blockers:")
        for item in failed:
            print(" -", item)
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
