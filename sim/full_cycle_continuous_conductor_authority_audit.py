"""Fail-closed authority audit for the complete live-conductor cycle.

This module does not invent a flexible-wire shape.  It binds the captured
axis timeline to the active-sector quasi-static route loci and to the existing
connected presentation route, then reports the exact difference between:

* a physical/quasi-static route proved at one sampled pose;
* a presentation polyline which keeps the conductor visible; and
* a physical route proved continuously through a motion interval.

The current normal-GOAL artifacts provide the first two plus a small number
of positive-duration raw-axis holds whose pose is exactly one of the proved
guide loci.  A constant physical route at a constant raw pose is valid over
that hold; moving spans do not inherit that authority.  Load, park, index,
moving M0/M2 spans, shaft-transition continuity, and unload therefore remain
explicitly unproved.  The input paths are parameters so a future raw capture
(including a serial-position digital twin) can be audited without weakening
the bindings: its guide-locus and presentation artifacts must be regenerated
against that exact capture before they can contribute any authority.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

from traj import Timeline, load_events, winding_windows


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAPTURE_PATH = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
GUIDE_AUDIT_PATH = (
    ROOT / "out" / "reports"
    / "carriage_active_sector_terminal_guide_audit.json"
)
LOCUS_PATH = (
    ROOT / "out" / "reports"
    / "carriage_active_sector_terminal_guide_loci.json"
)
PRESENTATION_PATH = (
    ROOT / "out" / "reports" / "continuous_conductor_route.json"
)
OUTPUT_PATH = (
    ROOT / "out" / "reports"
    / "full_cycle_continuous_conductor_authority_audit.json"
)
OUTPUT_MD = (
    ROOT / "out" / "reports"
    / "full_cycle_continuous_conductor_authority_audit.md"
)

SCHEMA = "full-cycle-continuous-conductor-authority-audit/v1"
GUIDE_SCHEMA = "carriage-active-sector-terminal-guide-audit/v1"
LOCUS_SCHEMA = "carriage-active-sector-terminal-guide-loci/v1"
PRESENTATION_SCHEMA = "continuous-conductor-route/v1"

EXPECTED_PASSES = 24
EXPECTED_STATES_PER_PASS = 100
EXPECTED_LOCI = EXPECTED_PASSES * EXPECTED_STATES_PER_PASS
EXPECTED_SHA256_LENGTH = 64
TIME_TOL_S = 2.0e-6
AXIS_TOL_RAD = 2.0e-8
SEAM_TOL_MM = 1.0e-6

REQUIRED_SEGMENT_ORDER = (
    "flyer_tensioned_bore_handoff",
    "flyer_bell_meridian_arc",
    "dynamic_free_span_to_capture",
    "carriage_capture_fillet",
    "carriage_fixed_free_gap",
    "carriage_selection_bowl",
    "dynamic_handoff_gap",
    "spindle_short_leadin",
)

UNPROVEN_TRANSITION_KINDS = {
    "cap_transition",
    "inter_turn_advance",
    "tooth_transition",
    "to_shaft_wrap",
    "shaft_wrap",
    "from_shaft_wrap",
}

CURRENT_TERMINAL_SEGMENTS = {
    "carriage_selection_bowl",
    "dynamic_handoff_gap",
    "spindle_short_leadin",
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(value: Mapping[str, Any], field: str) -> str:
    body = deepcopy(dict(value))
    body.pop(field, None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load_object(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def _finite(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _distance(left: Sequence[Any], right: Sequence[Any]) -> float:
    if len(left) != 3 or len(right) != 3:
        return math.inf
    if not all(_finite(value) for value in (*left, *right)):
        return math.inf
    return math.sqrt(sum(
        (float(a) - float(b)) ** 2 for a, b in zip(left, right)
    ))


def _expected_cap_endpoint_name(lane_id: object) -> str | None:
    """Return the current side-specific active-guide cap endpoint."""

    value = str(lane_id)
    if "_left_" in value:
        return "_lane_points()['riser_top']"
    if "_right_" in value:
        return "_lane_points()['waypoint']"
    return None


def _relative(path: Path, root: Path) -> str:
    try:
        result = Path(path).resolve().relative_to(Path(root).resolve())
    except ValueError:
        result = Path(path).resolve()
    return str(result).replace("\\", "/")


def _under_root(root: Path, raw_name: Any) -> Path:
    parts = [part for part in str(raw_name).replace("\\", "/").split("/")
             if part]
    return Path(root).joinpath(*parts)


def _issue(severity: str, code: str, message: str,
           **detail: Any) -> dict[str, Any]:
    row: dict[str, Any] = {
        "severity": severity,
        "code": code,
        "message": message,
    }
    if detail:
        row["detail"] = detail
    return row


def _check_self_hash(
    label: str,
    value: Mapping[str, Any],
    field: str,
    issues: list[dict[str, Any]],
) -> bool:
    actual = value.get(field)
    expected = _canonical_hash(value, field)
    passed = isinstance(actual, str) and actual == expected
    if not passed:
        issues.append(_issue(
            "INTEGRITY_FAIL", f"{label}_self_hash_mismatch",
            f"{label} canonical self-hash is missing or stale",
            expected=expected, actual=actual,
        ))
    return passed


def _source_freshness(
    label: str,
    hashes: Any,
    source_root: Path,
    issues: list[dict[str, Any]],
) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    if not isinstance(hashes, dict) or not hashes:
        issues.append(_issue(
            "INTEGRITY_FAIL", f"{label}_source_hashes_missing",
            f"{label} has no source-hash table",
        ))
        return {"all_current": False, "rows": rows}
    for raw_name, expected in hashes.items():
        path = _under_root(source_root, raw_name)
        exists = path.is_file()
        actual = _sha256(path) if exists else None
        current = (
            isinstance(expected, str)
            and len(expected) == EXPECTED_SHA256_LENGTH
            and actual == expected
        )
        rows.append({
            "path": str(raw_name).replace("\\", "/"),
            "exists": exists,
            "expected_sha256": expected,
            "actual_sha256": actual,
            "current": current,
        })
        if not current:
            issues.append(_issue(
                "INTEGRITY_FAIL", f"{label}_source_stale",
                f"{label} source/input hash is stale: {raw_name}",
                path=str(raw_name), expected=expected, actual=actual,
            ))
    return {
        "all_current": bool(rows) and all(row["current"] for row in rows),
        "rows": rows,
    }


def _infer_shaft_wraps(
    events: Sequence[Mapping[str, Any]], timeline: Timeline,
) -> list[dict[str, Any]]:
    """Infer commanded wrap motion from the actual M1 position at command."""

    starts = [
        index for index, event in enumerate(events)
        if event.get("e") == "wind_wire_around_shaft"
    ]
    result: list[dict[str, Any]] = []
    velocity = float(timeline.meta["velocities"][1])
    for number, start_index in enumerate(starts, start=1):
        done_index = next((
            index for index in range(start_index + 1, len(events))
            if events[index].get("e") == "wind_wire_around_shaft_done"
        ), None)
        commands = [] if done_index is None else [
            (index, events[index])
            for index in range(start_index + 1, done_index)
            if events[index].get("e") == "cmd"
            and events[index].get("m") == 1
        ]
        row: dict[str, Any] = {
            "number": number,
            "marker_start_time_s": float(events[start_index]["t"]),
            "marker_done_time_s": (
                float(events[done_index]["t"])
                if done_index is not None else None
            ),
            "m1_command_count": len(commands),
            "valid_marker_and_command_contract": False,
        }
        if done_index is None or len(commands) != 1:
            result.append(row)
            continue
        command_index, command = commands[0]
        command_t = float(command["t"])
        start_m1 = float(timeline.axes[1].pos_at(command_t))
        target = float(command.get("model_target", command.get("a")))
        delta = target - start_m1
        arrival = command_t + abs(delta) / velocity
        row.update({
            "source_command_index": command_index,
            "command_time_s": command_t,
            "actual_start_m1_rad": start_m1,
            "target_m1_rad": target,
            "delta_m1_rad": delta,
            "turns": abs(delta) / (2.0 * math.pi),
            "arrival_time_s": arrival,
            "arrives_before_done_marker": (
                arrival <= float(events[done_index]["t"]) + TIME_TOL_S
            ),
            "valid_marker_and_command_contract": True,
        })
        result.append(row)
    return result


def _capture_evidence(
    events: list[dict[str, Any]], timeline: Timeline,
    issues: list[dict[str, Any]],
) -> dict[str, Any]:
    metas = [event for event in events if event.get("e") == "meta"]
    if len(metas) != 1:
        issues.append(_issue(
            "INTEGRITY_FAIL", "capture_meta_count",
            "capture must have exactly one meta row", count=len(metas),
        ))
    meta = metas[0] if len(metas) == 1 else {}
    completions = [
        event for event in events if event.get("e") == "cycle_complete"
    ]
    if len(completions) != 1:
        issues.append(_issue(
            "INTEGRITY_FAIL", "capture_cycle_complete_count",
            "capture must have exactly one cycle_complete row",
            count=len(completions),
        ))
    try:
        passes = winding_windows(events)
    except (KeyError, TypeError, ValueError) as exc:
        passes = []
        issues.append(_issue(
            "INTEGRITY_FAIL", "capture_winding_windows_invalid", str(exc),
        ))
    wraps = _infer_shaft_wraps(events, timeline)
    wrap_marker_contract = (
        len(wraps) == 2
        and all(row["valid_marker_and_command_contract"] for row in wraps)
    )
    wraps_exactly_two = (
        wrap_marker_contract
        and all(
            math.isclose(float(row["turns"]), 2.0, abs_tol=1.0e-9)
            for row in wraps
        )
    )
    if not wrap_marker_contract:
        issues.append(_issue(
            "INTEGRITY_FAIL", "shaft_wrap_marker_contract_invalid",
            "capture does not contain two one-command M1 shaft-wrap blocks",
        ))
    if not wraps_exactly_two:
        issues.append(_issue(
            "AUTHORITY_GAP", "shaft_wraps_not_exactly_two_turns",
            "actual M1 feedback-to-target motion is not two turns per wrap",
            observed_turns=[row.get("turns") for row in wraps],
        ))
    nontrivial_m0_commands = 0
    for event in events:
        if event.get("e") != "cmd" or event.get("m") != 0:
            continue
        command_t = float(event["t"])
        target = float(event.get("model_target", event.get("a")))
        if abs(target - timeline.axes[0].pos_at(command_t)) > 1.0e-9:
            nontrivial_m0_commands += 1
    end_time = (
        float(completions[0]["t"])
        if len(completions) == 1
        else max(float(event.get("t", 0.0)) for event in events)
    )
    return {
        "capture_schema": meta.get("capture_schema"),
        "controller_mode": meta.get("controller_mode"),
        "controller_adapter_sha256": meta.get("controller_adapter_sha256"),
        "shaft_wrap_contract": meta.get("shaft_wrap_contract"),
        "event_count": len(events),
        "timeline_start_time_s": 0.0,
        "timeline_end_time_s": end_time,
        "timeline_axis_motion_end_time_s": float(timeline.t_end),
        "winding_pass_count": len(passes),
        "shaft_wrap_phase_event_count": sum(
            event.get("e") == "shaft_wrap_phase" for event in events
        ),
        "nontrivial_m0_command_count": nontrivial_m0_commands,
        "shaft_wraps": wraps,
        "gates": {
            "one_meta": len(metas) == 1,
            "one_cycle_complete": len(completions) == 1,
            "capture_schema_at_least_4": (
                isinstance(meta.get("capture_schema"), int)
                and int(meta["capture_schema"]) >= 4
            ),
            "all_24_passes_present": len(passes) == EXPECTED_PASSES,
            "two_well_formed_shaft_wrap_markers": wrap_marker_contract,
            "both_shaft_wraps_exactly_two_turns": wraps_exactly_two,
        },
    }


def _locus_evidence(
    loci_document: Mapping[str, Any],
    guide_report: Mapping[str, Any],
    timeline: Timeline,
    capture_sha256: str,
    locus_file_sha256: str,
    guide_sources_current: bool,
    issues: list[dict[str, Any]],
) -> dict[str, Any]:
    run = loci_document.get("run", {})
    rows = loci_document.get("loci", [])
    if not isinstance(rows, list):
        rows = []
        issues.append(_issue(
            "INTEGRITY_FAIL", "locus_rows_invalid",
            "locus artifact has no list-valued loci field",
        ))
    schema_ok = loci_document.get("schema") == LOCUS_SCHEMA
    if not schema_ok:
        issues.append(_issue(
            "INTEGRITY_FAIL", "locus_schema_mismatch",
            f"locus artifact must be {LOCUS_SCHEMA}",
            actual=loci_document.get("schema"),
        ))
    locus_hash_ok = _check_self_hash(
        "locus_payload", loci_document, "locus_payload_sha256", issues,
    )
    capture_bound = run.get("capture_sha256") == capture_sha256
    if not capture_bound:
        issues.append(_issue(
            "INTEGRITY_FAIL", "locus_capture_binding_mismatch",
            "locus artifact is not bound to the supplied capture",
            expected=capture_sha256, actual=run.get("capture_sha256"),
        ))

    api = guide_report.get("player_route_api", {})
    guide_file_bound = api.get("compact_file_sha256") == locus_file_sha256
    guide_payload_bound = (
        api.get("canonical_payload_sha256")
        == loci_document.get("locus_payload_sha256")
    )
    guide_count_bound = api.get("locus_count") == len(rows)
    for passed, code, message in (
        (guide_file_bound, "guide_locus_file_binding_mismatch",
         "guide audit compact-file SHA does not match the locus artifact"),
        (guide_payload_bound, "guide_locus_payload_binding_mismatch",
         "guide audit canonical payload SHA does not match the locus artifact"),
        (guide_count_bound, "guide_locus_count_binding_mismatch",
         "guide audit locus count does not match the locus artifact"),
    ):
        if not passed:
            issues.append(_issue("INTEGRITY_FAIL", code, message))

    indices_ok = all(
        isinstance(row, dict) and row.get("locus_index") == index
        for index, row in enumerate(rows)
    )
    state_keys = {
        (row.get("pass_index"), row.get("state_index"))
        for row in rows if isinstance(row, dict)
    }
    expected_state_keys = {
        (pass_index, state_index)
        for pass_index in range(EXPECTED_PASSES)
        for state_index in range(EXPECTED_STATES_PER_PASS)
    }
    state_coverage_ok = state_keys == expected_state_keys
    times_monotonic = all(
        _finite(left.get("time_s"))
        and _finite(right.get("time_s"))
        and float(left["time_s"]) < float(right["time_s"])
        for left, right in zip(rows, rows[1:])
        if isinstance(left, dict) and isinstance(right, dict)
    ) and len(rows) > 0

    maximum_axis_error = 0.0
    maximum_seam_error = 0.0
    axis_rows_valid = True
    segment_rows_valid = True
    terminal_rows_valid = True
    path_hash_rows_valid = True
    for row in rows:
        if not isinstance(row, dict) or not _finite(row.get("time_s")):
            axis_rows_valid = False
            segment_rows_valid = False
            terminal_rows_valid = False
            path_hash_rows_valid = False
            continue
        axes = row.get("axes", {})
        actual_axes = (
            axes.get("M0_raw_rad"),
            axes.get("M1_spindle_rad"),
            axes.get("M2_flyer_rad"),
        )
        if not all(_finite(value) for value in actual_axes):
            axis_rows_valid = False
        else:
            expected_axes = timeline.pose_at(float(row["time_s"]))
            error = max(abs(float(a) - float(b))
                        for a, b in zip(actual_axes, expected_axes))
            maximum_axis_error = max(maximum_axis_error, error)
            if error > AXIS_TOL_RAD:
                axis_rows_valid = False

        segments = row.get("segments", [])
        names = [segment.get("name") for segment in segments
                 if isinstance(segment, dict)]
        if tuple(names) != REQUIRED_SEGMENT_ORDER:
            segment_rows_valid = False
        for left, right in zip(segments, segments[1:]):
            left_points = left.get("machine_world_samples_mm", [])
            right_points = right.get("machine_world_samples_mm", [])
            if not left_points or not right_points:
                segment_rows_valid = False
                continue
            seam = _distance(left_points[-1], right_points[0])
            maximum_seam_error = max(maximum_seam_error, seam)
            if seam > SEAM_TOL_MM:
                segment_rows_valid = False

        terminal = row.get("terminal_binding", {})
        expected_endpoint_name = _expected_cap_endpoint_name(
            terminal.get("lane_id")
        )
        if not (
            expected_endpoint_name is not None
            and terminal.get("cap_endpoint_name") == expected_endpoint_name
            and terminal.get("exact_strand_settling_and_neatness_authorized")
            is False
        ):
            terminal_rows_valid = False
        path_hash = row.get("path_sha256")
        if not (
            isinstance(path_hash, str)
            and len(path_hash) == EXPECTED_SHA256_LENGTH
        ):
            path_hash_rows_valid = False

    if not axis_rows_valid:
        issues.append(_issue(
            "INTEGRITY_FAIL", "locus_axis_binding_mismatch",
            "one or more locus axes do not match the supplied raw Timeline",
            maximum_axis_error_rad=maximum_axis_error,
        ))
    if not segment_rows_valid:
        issues.append(_issue(
            "INTEGRITY_FAIL", "locus_segment_contract_invalid",
            "one or more locus route segment chains are incomplete/disconnected",
            maximum_seam_error_mm=maximum_seam_error,
        ))

    guide_gates = guide_report.get("release_gates", {})
    guide_authority = guide_report.get("authority_boundary", {})
    point_state_proven = all((
        schema_ok,
        locus_hash_ok,
        capture_bound,
        guide_file_bound,
        guide_payload_bound,
        guide_count_bound,
        guide_sources_current,
        len(rows) == EXPECTED_LOCI,
        indices_ok,
        state_coverage_ok,
        times_monotonic,
        axis_rows_valid,
        segment_rows_valid,
        terminal_rows_valid,
        path_hash_rows_valid,
        loci_document.get("flyer_reference_validation", {}).get("status")
        == "PASS",
        guide_gates.get("all_2400_physical_bell_terminal_routes_pass")
        is True,
        guide_authority.get("raw_capture_and_2400_loci") == "authoritative",
    ))
    if not point_state_proven:
        issues.append(_issue(
            "AUTHORITY_GAP", "deposition_point_state_authority_not_current",
            "the 2400 quasi-static deposition point routes are not all current and bound",
        ))
    return {
        "locus_count": len(rows),
        "expected_locus_count": EXPECTED_LOCI,
        "first_locus_time_s": (
            float(rows[0]["time_s"]) if rows and _finite(rows[0].get("time_s"))
            else None
        ),
        "last_locus_time_s": (
            float(rows[-1]["time_s"]) if rows and _finite(rows[-1].get("time_s"))
            else None
        ),
        "maximum_axis_binding_error_rad": maximum_axis_error,
        "maximum_segment_seam_error_mm": maximum_seam_error,
        "authority_kind": "QUASI_STATIC_POINT_STATES_ONLY",
        "continuous_interpolation_authorized": False,
        "gates": {
            "schema": schema_ok,
            "self_hash": locus_hash_ok,
            "capture_bound": capture_bound,
            "guide_file_bound": guide_file_bound,
            "guide_payload_bound": guide_payload_bound,
            "guide_count_bound": guide_count_bound,
            "all_2400_pass_state_keys": state_coverage_ok,
            "indices_ordered": indices_ok,
            "times_strictly_monotonic": times_monotonic,
            "axes_match_raw_timeline": axis_rows_valid,
            "segment_order_and_seams": segment_rows_valid,
            "terminal_lanes_named": terminal_rows_valid,
            "path_hashes_present": path_hash_rows_valid,
            "deposition_point_states_physically_quasistatically_proven": (
                point_state_proven
            ),
            "between_locus_motion_physically_quasistatically_proven": False,
        },
    }


def _current_cap_entry_evidence(
    loci_document: Mapping[str, Any],
    guide_report: Mapping[str, Any],
) -> dict[str, Any]:
    """Bind cap-entry truth to the current physical active-sector route.

    ``integrated_phase_aware_wire_path`` predates the active-sector successor.
    Its free chord terminates at an old cap endpoint and then compares that
    chord with global +Y, producing 2,400 real kinks *for that retired route*.
    The current compact artifact instead reaches a side-specific cap-lane
    point (left ``riser_top``; right ``waypoint``) through the named selection
    bowl, open handoff, and physical R3.50 short lead-in.
    Applying the retired chord test to this different topology is invalid.
    """

    rows = loci_document.get("loci", [])
    contracts = loci_document.get("segment_contract", {})
    segment_names = set(contracts) if isinstance(contracts, dict) else set()
    terminal = guide_report.get("terminal_deposition_route", {})
    guide_gates = guide_report.get("release_gates", {})
    endpoints_current = bool(rows) and all(
        isinstance(row, Mapping)
        and _expected_cap_endpoint_name(
            row.get("terminal_binding", {}).get("lane_id")
        ) is not None
        and row.get("terminal_binding", {}).get("cap_endpoint_name")
        == _expected_cap_endpoint_name(
            row.get("terminal_binding", {}).get("lane_id")
        )
        for row in rows
    )
    current_topology = CURRENT_TERMINAL_SEGMENTS <= segment_names
    maximum_tangent_error = terminal.get(
        "maximum_bell_exit_tangent_error_deg"
    )
    minimum_radius = terminal.get("minimum_bell_wire_center_radius_mm")
    current_route_pass = bool(
        len(rows) == EXPECTED_LOCI
        and endpoints_current
        and current_topology
        and guide_gates.get("all_2400_physical_bell_terminal_routes_pass")
        is True
        and _finite(maximum_tangent_error)
        and float(maximum_tangent_error) <= 1.0e-5
        and _finite(minimum_radius)
        and float(minimum_radius) >= 3.0
    )
    return {
        "governing_route": (
            "physical PEEK bell -> carriage selection bowl -> open handoff "
            "-> spindle R3.50 short lead-in -> side-specific cap lane point"
        ),
        "retired_direct_chord_cap_endpoint_check_applicable": False,
        "retired_check_reason": (
            "the old chord/global-Y test has a different endpoint and omits "
            "the current physical selection-bowl and short-leadin segments"
        ),
        "current_route_locus_count": len(rows),
        "current_route_cap_entry_kink_count": 0 if current_route_pass else None,
        "maximum_physical_bell_exit_tangent_error_deg": (
            float(maximum_tangent_error)
            if _finite(maximum_tangent_error) else None
        ),
        "minimum_named_guide_wire_center_radius_mm": (
            min(3.25, 3.50, float(minimum_radius))
            if _finite(minimum_radius) else None
        ),
        "gates": {
            "all_terminal_endpoints_are_current_side_specific_cap_lane_points": (
                endpoints_current
            ),
            "physical_selection_handoff_leadin_chain_present": current_topology,
            "all_2400_current_physical_routes_pass": current_route_pass,
            "obsolete_cap_endpoint_check_excluded_from_governing_decision": True,
        },
        "status": "PASS" if current_route_pass else "FAIL",
    }


def _stationary_locus_interval_evidence(
    loci_document: Mapping[str, Any],
    guide_report: Mapping[str, Any],
    presentation: Mapping[str, Any],
    timeline: Timeline,
    capture_end_time_s: float,
    point_states_proven: bool,
) -> dict[str, Any]:
    """Find positive-duration raw holds at exact proved deposition loci.

    This is deliberately narrower than interpolation.  Boundaries include
    every raw axis knot, presentation item boundary, and locus time.  An
    interval is admitted only when all three reconstructed raw axes are
    constant, the whole interval lies inside one winding-half-turn item, and
    its exact pose matches a locus from that same pass.
    """

    rows = loci_document.get("loci", [])
    items = presentation.get("items", [])
    if not isinstance(rows, list):
        rows = []
    if not isinstance(items, list):
        items = []

    boundaries = {0.0, float(capture_end_time_s)}
    boundaries.update(float(value) for value in timeline.knot_times())
    for row in rows:
        if isinstance(row, Mapping) and _finite(row.get("time_s")):
            boundaries.add(float(row["time_s"]))
    for item in items:
        if not isinstance(item, Mapping):
            continue
        if _finite(item.get("start_time_s")):
            boundaries.add(float(item["start_time_s"]))
        if _finite(item.get("end_time_s")):
            boundaries.add(float(item["end_time_s"]))
    ordered = sorted(
        value for value in boundaries
        if -TIME_TOL_S <= value <= capture_end_time_s + TIME_TOL_S
    )

    loci_by_pass: defaultdict[int, list[tuple[int, Mapping[str, Any]]]] = (
        defaultdict(list)
    )
    for index, row in enumerate(rows):
        if isinstance(row, Mapping) and isinstance(row.get("pass_index"), int):
            loci_by_pass[int(row["pass_index"])].append((index, row))

    guide_gates = guide_report.get("release_gates", {})
    point_contact_contract = all((
        point_states_proven,
        guide_gates.get("all_2400_physical_bell_terminal_routes_pass") is True,
        guide_gates.get("deposition_exact_rigid_pairs_clear_ge_2mm") is True,
        guide_gates.get("arbitrary_M1_caps_and_copper_clear_ge_2mm") is True,
    ))
    admitted: list[dict[str, Any]] = []
    stationary_candidate_count = 0
    for start, end in zip(ordered, ordered[1:]):
        duration = end - start
        if duration <= TIME_TOL_S:
            continue
        start_pose = tuple(map(float, timeline.pose_at(start)))
        end_pose = tuple(map(float, timeline.pose_at(end)))
        if max(abs(a - b) for a, b in zip(start_pose, end_pose)) \
                > AXIS_TOL_RAD:
            continue
        midpoint = (start + end) / 2.0
        owner = next((
            item for item in items
            if isinstance(item, Mapping)
            and item.get("kind") == "winding_half_turn"
            and float(item.get("start_time_s", math.inf)) - TIME_TOL_S
            <= midpoint
            <= float(item.get("end_time_s", -math.inf)) + TIME_TOL_S
        ), None)
        if owner is None or not isinstance(owner.get("pass_index"), int):
            continue
        stationary_candidate_count += 1
        matched = None
        maximum_error = math.inf
        for locus_index, row in loci_by_pass[int(owner["pass_index"])]:
            axes = row.get("axes", {})
            candidate = (
                axes.get("M0_raw_rad"),
                axes.get("M1_spindle_rad"),
                axes.get("M2_flyer_rad"),
            )
            if not all(_finite(value) for value in candidate):
                continue
            error = max(
                abs(float(a) - float(b))
                for a, b in zip(start_pose, candidate)
            )
            if error < maximum_error:
                maximum_error = error
                matched = (locus_index, row)
        if matched is None or maximum_error > AXIS_TOL_RAD:
            continue
        locus_index, row = matched
        if not point_contact_contract:
            continue
        admitted.append({
            "interval_index": len(admitted),
            "start_time_s": start,
            "end_time_s": end,
            "duration_s": duration,
            "presentation_item_index": owner.get("index"),
            "locus_index": locus_index,
            "pass_index": row.get("pass_index"),
            "state_index": row.get("state_index"),
            "raw_pose_rad": list(start_pose),
            "maximum_locus_pose_error_rad": maximum_error,
            "authority_scope": (
                "constant spool-to-active-cap quasi-static route at one "
                "physically proved guide locus; exact strand settling and "
                "tension dynamics excluded"
            ),
        })
    duration = sum(row["duration_s"] for row in admitted)
    return {
        "partition_boundary_count": len(ordered),
        "stationary_winding_candidate_count": stationary_candidate_count,
        "authorized_interval_count": len(admitted),
        "authorized_duration_s": duration,
        "timeline_fraction": (
            duration / capture_end_time_s if capture_end_time_s > 0.0 else 0.0
        ),
        "maximum_axis_motion_inside_authorized_interval_rad": 0.0,
        "authority_scope": (
            "positive-duration constant-route holds only; no moving route "
            "interpolation authority"
        ),
        "gates": {
            "deposition_point_states_current": point_states_proven,
            "point_route_named_contact_and_rigid_clearance_contract": (
                point_contact_contract
            ),
            "every_admitted_interval_has_positive_duration": bool(admitted)
            and all(row["duration_s"] > TIME_TOL_S for row in admitted),
            "every_admitted_interval_matches_exact_same_pass_locus": (
                bool(admitted)
                and all(
                    row["maximum_locus_pose_error_rad"] <= AXIS_TOL_RAD
                    for row in admitted
                )
            ),
            "moving_between_locus_route_family_proven": False,
        },
        "intervals": admitted,
    }


def _presentation_evidence(
    presentation: Mapping[str, Any],
    capture_sha256: str,
    capture_end_time_s: float,
    source_root: Path,
    issues: list[dict[str, Any]],
) -> dict[str, Any]:
    schema_ok = presentation.get("schema") == PRESENTATION_SCHEMA
    if not schema_ok:
        issues.append(_issue(
            "INTEGRITY_FAIL", "presentation_schema_mismatch",
            f"presentation route must be {PRESENTATION_SCHEMA}",
            actual=presentation.get("schema"),
        ))
    self_hash_ok = _check_self_hash(
        "presentation", presentation, "report_sha256", issues,
    )
    capture_bound = (
        presentation.get("source_hashes", {}).get("raw_capture_sha256")
        == capture_sha256
    )
    if not capture_bound:
        issues.append(_issue(
            "INTEGRITY_FAIL", "presentation_capture_binding_mismatch",
            "presentation route is not bound to the supplied capture",
        ))

    source_rows = []
    source_bindings = presentation.get("source_hashes", {})
    source_map = {
        "generator_source_sha256": "sim/continuous_conductor_route.py",
        "traj_source_sha256": "sim/traj.py",
    }
    for hash_key, relative in source_map.items():
        path = _under_root(source_root, relative)
        actual = _sha256(path) if path.is_file() else None
        expected = source_bindings.get(hash_key)
        current = actual is not None and actual == expected
        source_rows.append({
            "path": relative,
            "expected_sha256": expected,
            "actual_sha256": actual,
            "current": current,
        })
        if not current:
            issues.append(_issue(
                "INTEGRITY_FAIL", "presentation_source_stale",
                f"presentation route source is stale: {relative}",
            ))

    items = presentation.get("items", [])
    if not isinstance(items, list):
        items = []
    index_order = all(
        isinstance(item, dict) and item.get("index") == index
        for index, item in enumerate(items)
    )
    time_continuity = bool(items)
    point_continuity = bool(items)
    maximum_time_gap = 0.0
    maximum_point_gap = 0.0
    if items:
        time_continuity &= abs(float(items[0]["start_time_s"])) <= TIME_TOL_S
        time_continuity &= (
            abs(float(items[-1]["end_time_s"]) - capture_end_time_s)
            <= TIME_TOL_S
        )
    for left, right in zip(items, items[1:]):
        time_gap = abs(
            float(right["start_time_s"]) - float(left["end_time_s"])
        )
        maximum_time_gap = max(maximum_time_gap, time_gap)
        if time_gap > TIME_TOL_S:
            time_continuity = False
        point_gap = _distance(
            left.get("end_point_mm", []), right.get("start_point_mm", []),
        )
        maximum_point_gap = max(maximum_point_gap, point_gap)
        if point_gap > SEAM_TOL_MM:
            point_continuity = False

    counts: Counter[str] = Counter()
    durations: defaultdict[str, float] = defaultdict(float)
    unproven_runs: Counter[str] = Counter()
    for item in items:
        kind = str(item.get("kind"))
        counts[kind] += 1
        durations[kind] += (
            float(item.get("end_time_s", 0.0))
            - float(item.get("start_time_s", 0.0))
        )
        if item.get("authorization") == "UNPROVEN_FAIL_CLOSED":
            unproven_runs[kind] += 1
        for run in item.get("runs", []):
            run_kind = str(run.get("kind"))
            if (
                run_kind in UNPROVEN_TRANSITION_KINDS
                and run.get("authorization") == "UNPROVEN_FAIL_CLOSED"
            ):
                unproven_runs[run_kind] += 1

    explicit_unproven = all(
        unproven_runs[kind] > 0 for kind in UNPROVEN_TRANSITION_KINDS
    )
    structural_gates = presentation.get("structural_gates", {})
    wire_handoff = presentation.get("wire_handoff_contract")
    wire_handoff_valid = bool(
        isinstance(wire_handoff, Mapping)
        and wire_handoff.get("status") == "PASS"
        and float(wire_handoff.get("maximum_gap_mm", math.inf))
        <= float(wire_handoff.get("tolerance_mm", -math.inf))
        and wire_handoff.get(
            "static_owner_continues_through_shaft_to_guide_root"
        ) is True
        and wire_handoff.get(
            "static_to_flyer_handoff_is_M2_axis_invariant"
        ) is True
        and wire_handoff.get(
            "unsupported_flexible_intervals_authorized"
        ) is False
        and structural_gates.get(
            "static_supply_to_flyer_bore_seam_exact"
        ) is True
    )
    if not wire_handoff_valid:
        issues.append(_issue(
            "INTEGRITY_FAIL", "presentation_wire_handoff_invalid",
            "static supply does not bind through the shaft to the flyer bore",
        ))
    presentation_partition_valid = all((
        schema_ok,
        self_hash_ok,
        capture_bound,
        all(row["current"] for row in source_rows),
        index_order,
        time_continuity,
        point_continuity,
        presentation.get("structural_status") == "PASS",
        structural_gates.get("full_virtual_timeline_has_live_endpoint")
        is True,
        structural_gates.get("ordered_conductor_graph_connected") is True,
        structural_gates.get("every_unproven_transition_is_dashed_red")
        is True,
        wire_handoff_valid,
        explicit_unproven,
    ))
    if not presentation_partition_valid:
        issues.append(_issue(
            "INTEGRITY_FAIL", "presentation_partition_invalid",
            "connected presentation route is stale, discontinuous, or hides authority",
            maximum_time_gap_s=maximum_time_gap,
            maximum_point_gap_mm=maximum_point_gap,
        ))
    classifications = []
    for kind in sorted(counts):
        classifications.append({
            "kind": kind,
            "item_count": counts[kind],
            "duration_s": durations[kind],
            "evidence_role": (
                "POINT_SAMPLED_PRESENTATION_NOT_INTERVAL_AUTHORITY"
                if kind == "winding_half_turn"
                else "UNPROVEN_FAIL_CLOSED_PRESENTATION"
            ),
            "physical_quasistatic_interval_proven": False,
        })
    return {
        "item_count": len(items),
        "maximum_time_gap_s": maximum_time_gap,
        "maximum_point_gap_mm": maximum_point_gap,
        "classifications": classifications,
        "kind_counts": dict(sorted(counts.items())),
        "kind_durations_s": dict(sorted(durations.items())),
        "explicit_unproven_run_counts": dict(sorted(unproven_runs.items())),
        "wire_handoff_contract": wire_handoff,
        "source_freshness": source_rows,
        "gates": {
            "schema": schema_ok,
            "self_hash": self_hash_ok,
            "capture_bound": capture_bound,
            "source_hashes_current": all(row["current"] for row in source_rows),
            "item_indices_ordered": index_order,
            "timeline_has_no_gaps_or_overlaps": time_continuity,
            "presentation_graph_connected": point_continuity,
            "all_transition_kinds_explicitly_fail_closed": explicit_unproven,
            "static_supply_to_flyer_bore_seam_exact": wire_handoff_valid,
            "presentation_partition_valid": presentation_partition_valid,
            "presentation_is_physical_interval_authority": False,
        },
    }


def _state_matrix(
    capture: Mapping[str, Any],
    loci: Mapping[str, Any],
    presentation: Mapping[str, Any],
    stationary_intervals: Mapping[str, Any],
) -> dict[str, Any]:
    counts = presentation.get("kind_counts", {})
    point_states = bool(
        loci.get("gates", {}).get(
            "deposition_point_states_physically_quasistatically_proven"
        )
    )
    wraps_exact = bool(
        capture.get("gates", {}).get("both_shaft_wraps_exactly_two_turns")
    )
    rows = {
        "load_and_initial_lead_capture": {
            "required_occurrences": 1,
            "observed_presentation_occurrences": counts.get("initial_hold", 0),
            "authority": "UNPROVEN",
            "proven": False,
            "blocker": (
                "the hash-bound fixed supply reaches the flyer guide root, "
                "but no initial lead anchor/load state or continuous flexible "
                "guide-to-first-deposition route is authorized"
            ),
            "fixed_supply_to_flyer_bore_bound": bool(
                presentation.get("gates", {}).get(
                    "static_supply_to_flyer_bore_seam_exact"
                )
            ),
        },
        "deposition_locus_pose": {
            "required_occurrences": EXPECTED_LOCI,
            "observed_occurrences": loci.get("locus_count", 0),
            "authority": "QUASI_STATIC_POINT_STATE",
            "proven": point_states,
            "blocker": None if point_states else (
                "locus source/bindings/geometry are not all current"
            ),
        },
        "between_locus_and_m0_motion": {
            "required_half_turn_intervals": EXPECTED_LOCI,
            "observed_nontrivial_m0_commands": capture.get(
                "nontrivial_m0_command_count", 0
            ),
            "authority": (
                "COMPUTED_CONSTANT_ROUTE_SUBINTERVALS_PLUS_"
                "UNPROVEN_MOVING_SPANS"
            ),
            "proven": False,
            "blocker": (
                "constant raw-pose holds are now admitted from exact locus "
                "matches, but moving M0/M2 spans still lack a conservative "
                "flexible-route/contact sweep"
            ),
            "authorized_constant_route_interval_count": (
                stationary_intervals.get("authorized_interval_count", 0)
            ),
            "authorized_constant_route_duration_s": (
                stationary_intervals.get("authorized_duration_s", 0.0)
            ),
            "moving_route_family_proven": bool(
                stationary_intervals.get("gates", {}).get(
                    "moving_between_locus_route_family_proven"
                )
            ),
        },
        "park_for_shaft_wrap": {
            "required_occurrences": 2,
            "observed_presentation_occurrences": counts.get("to_shaft_wrap", 0),
            "authority": "UNPROVEN",
            "proven": False,
            "blocker": (
                "no physical/quasi-static conductor family through M0/M2 park"
            ),
        },
        "tooth_and_phase_index": {
            "required_occurrences": EXPECTED_PASSES - 1,
            "observed_presentation_occurrences": (
                counts.get("tooth_transition", 0)
                + counts.get("from_shaft_wrap", 0)
            ),
            "authority": "UNPROVEN",
            "proven": False,
            "blocker": (
                "active terminal/deposited-tail ownership and swept flexible "
                "route are undefined between passes"
            ),
        },
        "shaft_wrap": {
            "required_occurrences": 2,
            "observed_occurrences": len(capture.get("shaft_wraps", [])),
            "raw_turn_count_gate": wraps_exact,
            "authority": "UNPROVEN",
            "proven": False,
            "blocker": (
                "raw turns must equal two and the full bell/shaft/tail route "
                "must be swept; current diagnostic helix is not clearance authority"
            ),
        },
        "unload_and_final_lead_state": {
            "required_occurrences": 1,
            "observed_presentation_occurrences": counts.get("final_hold", 0),
            "authority": "UNPROVEN",
            "proven": False,
            "blocker": "no terminal anchor/cut/unload conductor state or route",
        },
    }
    return rows


def _empty_report(
    input_paths: Mapping[str, Path], issues: list[dict[str, Any]],
) -> dict[str, Any]:
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "FAIL",
        "audit_integrity_status": "FAIL",
        "decision": "FULL_CYCLE_CONDUCTOR_NOT_PROVEN_FAIL_CLOSED",
        "production_authorized": False,
        "input_files": {name: str(path) for name, path in input_paths.items()},
        "input_file_sha256": {
            name: _sha256(path) if path.is_file() else None
            for name, path in input_paths.items()
        },
        "issues": issues,
        "source_hashes": {
            "sim/full_cycle_continuous_conductor_authority_audit.py": (
                _sha256(Path(__file__))
            ),
            "sim/traj.py": _sha256(HERE / "traj.py"),
        },
    }
    report["report_sha256"] = _canonical_hash(report, "report_sha256")
    return report


def analyze(
    capture_path: Path = CAPTURE_PATH,
    guide_audit_path: Path = GUIDE_AUDIT_PATH,
    locus_path: Path = LOCUS_PATH,
    presentation_path: Path = PRESENTATION_PATH,
    *,
    source_root: Path = ROOT,
) -> dict[str, Any]:
    """Return a hash-bound, fail-closed full-cycle authority report."""

    capture_path = Path(capture_path)
    guide_audit_path = Path(guide_audit_path)
    locus_path = Path(locus_path)
    presentation_path = Path(presentation_path)
    source_root = Path(source_root)
    input_paths = {
        "capture": capture_path,
        "guide_audit": guide_audit_path,
        "loci": locus_path,
        "presentation": presentation_path,
    }
    issues: list[dict[str, Any]] = []
    missing = [name for name, path in input_paths.items() if not path.is_file()]
    if missing:
        for name in missing:
            issues.append(_issue(
                "INTEGRITY_FAIL", f"{name}_missing",
                f"required input is missing: {input_paths[name]}",
            ))
        return _empty_report(input_paths, issues)

    try:
        events = load_events(capture_path)
        guide_report = _load_object(guide_audit_path)
        loci_document = _load_object(locus_path)
        presentation = _load_object(presentation_path)
        timeline = Timeline(events)
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
        issues.append(_issue(
            "INTEGRITY_FAIL", "input_parse_or_timeline_error", str(exc),
        ))
        return _empty_report(input_paths, issues)

    capture_sha = _sha256(capture_path)
    guide_sha = _sha256(guide_audit_path)
    locus_sha = _sha256(locus_path)
    presentation_sha = _sha256(presentation_path)

    guide_schema_ok = guide_report.get("schema") == GUIDE_SCHEMA
    if not guide_schema_ok:
        issues.append(_issue(
            "INTEGRITY_FAIL", "guide_schema_mismatch",
            f"guide audit must be {GUIDE_SCHEMA}",
            actual=guide_report.get("schema"),
        ))
    guide_self_hash_ok = _check_self_hash(
        "guide_audit", guide_report, "report_sha256", issues,
    )
    guide_sources = _source_freshness(
        "guide_audit", guide_report.get("source_hashes"), source_root, issues,
    )

    capture_evidence = _capture_evidence(events, timeline, issues)
    locus_evidence = _locus_evidence(
        loci_document, guide_report, timeline, capture_sha, locus_sha,
        bool(guide_sources["all_current"]), issues,
    )
    presentation_evidence = _presentation_evidence(
        presentation, capture_sha,
        float(capture_evidence["timeline_end_time_s"]),
        source_root, issues,
    )
    cap_entry_evidence = _current_cap_entry_evidence(
        loci_document, guide_report,
    )
    point_states_proven = bool(
        locus_evidence.get("gates", {}).get(
            "deposition_point_states_physically_quasistatically_proven"
        )
    )
    stationary_intervals = _stationary_locus_interval_evidence(
        loci_document, guide_report, presentation, timeline,
        float(capture_evidence["timeline_end_time_s"]),
        point_states_proven,
    )
    state_matrix = _state_matrix(
        capture_evidence, locus_evidence, presentation_evidence,
        stationary_intervals,
    )

    required_states_proven = all(
        bool(row.get("proven")) for row in state_matrix.values()
    )
    # Exact-locus stationary holds have positive duration and may carry their
    # constant route authority.  They do not prove any moving route family.
    full_cycle_proven = required_states_proven
    if not full_cycle_proven:
        issues.append(_issue(
            "AUTHORITY_GAP", "full_cycle_continuous_conductor_not_proven",
            "one or more required live-conductor state families lack continuous physical/quasi-static authority",
        ))

    integrity_ok = not any(
        issue["severity"] == "INTEGRITY_FAIL" for issue in issues
    )
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "PASS" if full_cycle_proven else "FAIL",
        "audit_integrity_status": "PASS" if integrity_ok else "FAIL",
        "decision": (
            "FULL_CYCLE_CONDUCTOR_PROVEN"
            if full_cycle_proven
            else "FULL_CYCLE_CONDUCTOR_NOT_PROVEN_FAIL_CLOSED"
        ),
        "production_authorized": full_cycle_proven,
        "authority_boundary": {
            "raw_timeline": "authoritative for supplied capture semantics",
            "active_sector_loci": (
                "quasi-static point states plus exactly matched constant-pose "
                "holds when every binding/source gate passes"
            ),
            "connected_conductor_route": (
                "presentation and coverage inventory only; never interval authority"
            ),
            "flexible_wire_between_moving_states": "unmodeled fail-closed",
            "sag_snag_friction_tension_dynamics": "hardware-only unless separately evidenced",
        },
        "input_files": {
            name: _relative(path, source_root)
            for name, path in input_paths.items()
        },
        "input_file_sha256": {
            "capture": capture_sha,
            "guide_audit": guide_sha,
            "loci": locus_sha,
            "presentation": presentation_sha,
        },
        "bound_artifact_ids": {
            "guide_report_sha256": guide_report.get("report_sha256"),
            "locus_payload_sha256": loci_document.get("locus_payload_sha256"),
            "presentation_report_sha256": presentation.get("report_sha256"),
        },
        "guide_artifact_integrity": {
            "schema": guide_schema_ok,
            "self_hash": guide_self_hash_ok,
            "source_freshness": guide_sources,
            "production_authorized_by_source": guide_report.get(
                "production_authorized"
            ),
            "declared_continuous_park_index_load_unload": (
                guide_report.get("authority_boundary", {}).get(
                    "continuous_park_index_load_unload"
                )
            ),
        },
        "capture_evidence": capture_evidence,
        "deposition_locus_evidence": locus_evidence,
        "current_cap_entry_evidence": cap_entry_evidence,
        "stationary_locus_interval_evidence": stationary_intervals,
        "presentation_timeline_evidence": presentation_evidence,
        "required_state_matrix": state_matrix,
        "coverage_result": {
            "presentation_timeline_fully_classified": bool(
                presentation_evidence.get("gates", {}).get(
                    "timeline_has_no_gaps_or_overlaps"
                )
            ),
            "quasi_static_point_state_count": (
                locus_evidence["locus_count"]
                if locus_evidence.get("gates", {}).get(
                    "deposition_point_states_physically_quasistatically_proven"
                ) else 0
            ),
            "physically_authorized_continuous_interval_count": (
                stationary_intervals["authorized_interval_count"]
            ),
            "physically_authorized_continuous_interval_duration_s": (
                stationary_intervals["authorized_duration_s"]
            ),
            "physically_authorized_timeline_fraction": (
                stationary_intervals["timeline_fraction"]
            ),
            "continuous_interval_authority_scope": (
                stationary_intervals["authority_scope"]
            ),
            "moving_route_interval_count": 0,
            "full_cycle_physical_quasistatic_proven": full_cycle_proven,
        },
        "minimum_evidence_to_close": [
            (
                "Regenerate a raw capture with actual feedback semantics and "
                "two exact M1 turns per shaft wrap; do not patch recorded targets."
            ),
            (
                "Regenerate the active route artifact against that exact capture "
                "and add hash-bound route states at every command, arrival, axis "
                "extremum, park, index, wrap, load, and unload boundary."
            ),
            (
                "Retain the hash-bound spool/felt/dancer/entry/shaft/flyer "
                "path and define initial and terminal lead-anchor states plus "
                "the flexible guide-to-workpiece continuation."
            ),
            (
                "For every adjacent state, provide a conservative continuous "
                "flexible-route family/corridor (not a single straight connector), "
                "including active terminal lane and deposited-tail ownership."
            ),
            (
                "Sweep those route families against final rigid parts and prior "
                "copper with bend-radius, clearance, and tension bounds."
            ),
            (
                "Keep snag, abrasion, strand settling, sag, and tension transients "
                "behind physical pull-through/endurance coupons."
            ),
        ],
        "issues": issues,
        "source_hashes": {
            "sim/full_cycle_continuous_conductor_authority_audit.py": (
                _sha256(Path(__file__))
            ),
            "sim/traj.py": _sha256(HERE / "traj.py"),
        },
    }
    report["report_sha256"] = _canonical_hash(report, "report_sha256")
    return report


def write_report(report: Mapping[str, Any], path: Path = OUTPUT_PATH) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def render_markdown(report: Mapping[str, Any]) -> str:
    coverage = report.get("coverage_result", {})
    capture = report.get("capture_evidence", {})
    lines = [
        "# Full-cycle continuous-conductor authority audit",
        "",
        f"**Release truth: {report.get('status', 'FAIL')}**  ",
        f"Decision: `{report.get('decision', 'UNKNOWN')}`  ",
        f"Audit integrity: `{report.get('audit_integrity_status', 'FAIL')}`  ",
        f"Production authorized: `{str(bool(report.get('production_authorized'))).lower()}`",
        "",
        "This is a fail-closed authority audit. A connected presentation route "
        "does not substitute for a physical or conservative quasi-static "
        "flexible-wire route through a motion interval.",
        "",
        "## Coverage",
        "",
        "| Measure | Result |",
        "|---|---:|",
        (
            "| Presentation timeline fully classified | "
            f"`{str(bool(coverage.get('presentation_timeline_fully_classified'))).lower()}` |"
        ),
        (
            "| Quasi-static point states currently authorized | "
            f"{coverage.get('quasi_static_point_state_count', 0)} |"
        ),
        (
            "| Physically authorized continuous intervals | "
            f"{coverage.get('physically_authorized_continuous_interval_count', 0)} |"
        ),
        (
            "| Authorized constant-route duration | "
            f"{coverage.get('physically_authorized_continuous_interval_duration_s', 0.0):.9f} s |"
        ),
        (
            "| Physically authorized timeline fraction | "
            f"{coverage.get('physically_authorized_timeline_fraction', 0.0):.6f} |"
        ),
        (
            "| Captured live-cycle duration | "
            f"{capture.get('timeline_end_time_s', 0.0):.6f} s |"
        ),
        "",
        "Interval scope: "
        f"`{coverage.get('continuous_interval_authority_scope', 'none')}`",
        "",
        "## Current cap-entry topology",
        "",
        (
            "The governing route uses the physical PEEK bell, carriage "
            "selection bowl, open handoff, and R3.50 spindle lead-in. The "
            "retired direct-chord/global-Y cap-mouth check is not applicable "
            "to this topology."
        ),
        "",
        (
            "- Current-route cap-entry kink count: **"
            f"{report.get('current_cap_entry_evidence', {}).get('current_route_cap_entry_kink_count')}**"
        ),
        (
            "- Minimum named-guide wire-center radius: **"
            f"{report.get('current_cap_entry_evidence', {}).get('minimum_named_guide_wire_center_radius_mm')} mm**"
        ),
        "",
        "## Required state families",
        "",
        "| State family | Authority | Proven | Blocker |",
        "|---|---|---:|---|",
    ]
    for name, row in report.get("required_state_matrix", {}).items():
        blocker = str(row.get("blocker") or "-").replace("|", "\\|")
        lines.append(
            f"| `{name}` | `{row.get('authority', 'UNKNOWN')}` | "
            f"`{str(bool(row.get('proven'))).lower()}` | {blocker} |"
        )
    lines.extend([
        "",
        "## Raw shaft-wrap motion",
        "",
        "| Wrap | Actual M1 start (rad) | Target (rad) | Turns | Exact two-turn gate |",
        "|---:|---:|---:|---:|---:|",
    ])
    exact_gate = bool(
        capture.get("gates", {}).get("both_shaft_wraps_exactly_two_turns")
    )
    for row in capture.get("shaft_wraps", []):
        lines.append(
            f"| {row.get('number')} | "
            f"{float(row.get('actual_start_m1_rad', 0.0)):.9f} | "
            f"{float(row.get('target_m1_rad', 0.0)):.9f} | "
            f"{float(row.get('turns', 0.0)):.12f} | "
            f"`{str(exact_gate).lower()}` |"
        )
    lines.extend([
        "",
        "## Minimum evidence to close",
        "",
    ])
    for index, item in enumerate(report.get("minimum_evidence_to_close", []), 1):
        lines.append(f"{index}. {item}")
    lines.extend([
        "",
        "## Fail-closed findings",
        "",
    ])
    for issue in report.get("issues", []):
        lines.append(
            f"- **{issue.get('severity', 'UNKNOWN')} / "
            f"`{issue.get('code', 'unknown')}`:** {issue.get('message', '')}"
        )
    lines.extend([
        "",
        f"Report SHA-256: `{report.get('report_sha256', '')}`",
        "",
    ])
    return "\n".join(lines)


def write_markdown(report: Mapping[str, Any], path: Path = OUTPUT_MD) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(render_markdown(report), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", type=Path, default=CAPTURE_PATH)
    parser.add_argument("--guide-audit", type=Path, default=GUIDE_AUDIT_PATH)
    parser.add_argument("--loci", type=Path, default=LOCUS_PATH)
    parser.add_argument("--presentation", type=Path, default=PRESENTATION_PATH)
    parser.add_argument("--source-root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path, default=OUTPUT_PATH)
    parser.add_argument("--markdown", type=Path, default=OUTPUT_MD)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    report = analyze(
        args.capture, args.guide_audit, args.loci, args.presentation,
        source_root=args.source_root,
    )
    if not args.check:
        write_report(report, args.output)
        write_markdown(report, args.markdown)
        print(f"wrote {args.output}")
        print(f"wrote {args.markdown}")
    coverage = report.get("coverage_result", {})
    print(
        f"full-cycle conductor {report['status']}: "
        f"integrity={report['audit_integrity_status']}; "
        f"point_states={coverage.get('quasi_static_point_state_count', 0)}; "
        f"continuous_intervals="
        f"{coverage.get('physically_authorized_continuous_interval_count', 0)}"
    )
    if report["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
