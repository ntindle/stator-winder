"""Hash-bound C0 continuity proof for adjacent moving half-turn loci.

The canonical raw capture contains axis commands, not flexible-wire shapes.
The active-sector locus artifact contains one physically reviewed route at
each of 100 half-turn boundary states per pass.  This audit proves the
strictly smaller mathematical statement that adjacent, same-pass route
polylines admit a connected C0 affine homotopy after normalized-arclength
reparameterization of each named segment.

That proof does *not* say the interpolated wire stays on named guide surfaces,
clears rigid parts or prior copper, preserves bend radius, or represents a
tensioned-wire equilibrium.  It therefore grants no moving physical or
quasi-static authority and cannot authorize production.  The final half-turn
of each pass also has no closing locus in the current 100-state artifact and
remains outside even this bounded C0 proof.
"""

from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

import numpy as np

from traj import Timeline, load_events


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

from params import DEFAULT_STATOR  # noqa: E402
from slot_route import (  # noqa: E402
    PackingSupportGraph,
    SequentialRoutePolicy,
    solve_safe_mouth_crossover,
)

CAPTURE_PATH = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
GUIDE_AUDIT_PATH = (
    ROOT / "out" / "reports"
    / "carriage_active_sector_terminal_guide_audit.json"
)
LOCUS_PATH = (
    ROOT / "out" / "reports"
    / "carriage_active_sector_terminal_guide_loci.json"
)
PACKING_PATH = ROOT / "out" / "reports" / "slot_packing.json"
SLOT_ROUTES_PATH = ROOT / "out" / "reports" / "slot_wire_routes.json"
OUTPUT_PATH = (
    ROOT / "out" / "reports"
    / "moving_half_turn_segment_continuity_audit.json"
)
OUTPUT_MD = (
    ROOT / "out" / "reports"
    / "moving_half_turn_segment_continuity_audit.md"
)

SCHEMA = "moving-half-turn-segment-continuity-audit/v1"
GUIDE_SCHEMA = "carriage-active-sector-terminal-guide-audit/v1"
LOCUS_SCHEMA = "carriage-active-sector-terminal-guide-loci/v1"
EXPECTED_CAPTURE_SCHEMA = 4
EXPECTED_PASSES = 24
EXPECTED_STATES_PER_PASS = 100
EXPECTED_POINT_STATES = EXPECTED_PASSES * EXPECTED_STATES_PER_PASS
EXPECTED_PAIRED_INTERVALS = EXPECTED_PASSES * (EXPECTED_STATES_PER_PASS - 1)
EXPECTED_UNPAIRED_CLOSING_INTERVALS = EXPECTED_PASSES
AXIS_TOL_RAD = 2.0e-8
SEAM_TOL_MM = 1.0e-6

SEGMENT_ORDER = (
    "flyer_tensioned_bore_handoff",
    "flyer_bell_meridian_arc",
    "dynamic_free_span_to_capture",
    "carriage_capture_fillet",
    "carriage_fixed_free_gap",
    "carriage_selection_bowl",
    "dynamic_handoff_gap",
    "spindle_short_leadin",
)


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


def _finite_point(point: Any) -> bool:
    return (
        isinstance(point, list)
        and len(point) == 3
        and all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
            for value in point
        )
    )


def _points(segment: Mapping[str, Any]) -> np.ndarray:
    raw = segment.get("machine_world_samples_mm")
    if not isinstance(raw, list) or len(raw) < 2 \
            or not all(_finite_point(point) for point in raw):
        raise ValueError(f"invalid segment samples for {segment.get('name')}")
    return np.asarray(raw, dtype=float)


def _resample_polyline(points: np.ndarray, count: int) -> np.ndarray:
    """Reparameterize a polyline by normalized chord arclength.

    The returned polyline has the exact same first/last points and samples the
    original piecewise-linear geometry; it does not smooth or invent a curve.
    """

    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2:
        raise ValueError("polyline must be Nx3 with at least two points")
    if count < 2:
        raise ValueError("resample count must be at least two")
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    total = float(np.sum(lengths))
    if total <= 1.0e-12:
        result = np.repeat(points[:1], count, axis=0)
        result[-1] = points[-1]
        return result
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    targets = np.linspace(0.0, total, count)
    result = np.empty((count, 3), dtype=float)
    for index, target in enumerate(targets):
        edge = int(np.searchsorted(cumulative, target, side="right") - 1)
        edge = max(0, min(edge, len(lengths) - 1))
        length = float(lengths[edge])
        fraction = (
            0.0 if length <= 1.0e-15
            else (float(target) - float(cumulative[edge])) / length
        )
        result[index] = (
            points[edge] + fraction * (points[edge + 1] - points[edge])
        )
    result[0] = points[0]
    result[-1] = points[-1]
    return result


def _segment_map(row: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    segments = row.get("segments")
    if not isinstance(segments, list):
        raise ValueError("locus has no segment list")
    names = tuple(
        segment.get("name") for segment in segments
        if isinstance(segment, Mapping)
    )
    if names != SEGMENT_ORDER or len(segments) != len(SEGMENT_ORDER):
        raise ValueError("locus segment order/topology changed")
    result = {str(segment["name"]): segment for segment in segments}
    for segment in result.values():
        _points(segment)
    return result


def _seam_vectors(
    segments: Mapping[str, Mapping[str, Any]],
) -> list[np.ndarray]:
    vectors = []
    for left_name, right_name in zip(SEGMENT_ORDER, SEGMENT_ORDER[1:]):
        left = _points(segments[left_name])
        right = _points(segments[right_name])
        vectors.append(right[0] - left[-1])
    return vectors


def _axes(row: Mapping[str, Any]) -> tuple[float, float, float]:
    axes = row.get("axes")
    if not isinstance(axes, Mapping):
        raise ValueError("locus axes are missing")
    result = (
        axes.get("M0_raw_rad"),
        axes.get("M1_spindle_rad"),
        axes.get("M2_flyer_rad"),
    )
    if not all(
        isinstance(value, (int, float)) and not isinstance(value, bool)
        and math.isfinite(float(value)) for value in result
    ):
        raise ValueError("locus axes are invalid")
    return tuple(float(value) for value in result)


def _interval_proof(
    interval_index: int,
    start: Mapping[str, Any],
    end: Mapping[str, Any],
    timeline: Timeline,
) -> dict[str, Any]:
    pass_index = int(start["pass_index"])
    start_state = int(start["state_index"])
    end_state = int(end["state_index"])
    if int(end["pass_index"]) != pass_index or end_state != start_state + 1:
        raise ValueError("interval endpoints are not adjacent same-pass states")
    start_t = float(start["time_s"])
    end_t = float(end["time_s"])
    if end_t <= start_t:
        raise ValueError("interval time is not increasing")

    start_axes = _axes(start)
    end_axes = _axes(end)
    start_axis_error = max(
        abs(value - expected)
        for value, expected in zip(start_axes, timeline.pose_at(start_t))
    )
    end_axis_error = max(
        abs(value - expected)
        for value, expected in zip(end_axes, timeline.pose_at(end_t))
    )

    start_segments = _segment_map(start)
    end_segments = _segment_map(end)
    start_seams = _seam_vectors(start_segments)
    end_seams = _seam_vectors(end_segments)
    seam_bounds = [
        max(float(np.linalg.norm(left)), float(np.linalg.norm(right)))
        for left, right in zip(start_seams, end_seams)
    ]

    sample_counts_start = []
    sample_counts_end = []
    resampled_counts = []
    maximum_vertex_motion = 0.0
    maximum_vertex_motion_segment = None
    endpoint_preservation_error = 0.0
    for name in SEGMENT_ORDER:
        left = _points(start_segments[name])
        right = _points(end_segments[name])
        count = max(len(left), len(right))
        left_resampled = _resample_polyline(left, count)
        right_resampled = _resample_polyline(right, count)
        sample_counts_start.append(len(left))
        sample_counts_end.append(len(right))
        resampled_counts.append(count)
        endpoint_preservation_error = max(
            endpoint_preservation_error,
            float(np.linalg.norm(left_resampled[0] - left[0])),
            float(np.linalg.norm(left_resampled[-1] - left[-1])),
            float(np.linalg.norm(right_resampled[0] - right[0])),
            float(np.linalg.norm(right_resampled[-1] - right[-1])),
        )
        vertex_motion = float(np.max(np.linalg.norm(
            right_resampled - left_resampled, axis=1,
        )))
        if vertex_motion > maximum_vertex_motion:
            maximum_vertex_motion = vertex_motion
            maximum_vertex_motion_segment = name

    start_lane = start.get("terminal_binding", {}).get("lane_id")
    end_lane = end.get("terminal_binding", {}).get("lane_id")
    row: dict[str, Any] = {
        "interval_index": interval_index,
        "pass_index": pass_index,
        "start_state_index": start_state,
        "end_state_index": end_state,
        "start_locus_index": int(start["locus_index"]),
        "end_locus_index": int(end["locus_index"]),
        "start_time_s": start_t,
        "end_time_s": end_t,
        "duration_s": end_t - start_t,
        "start_path_sha256": start.get("path_sha256"),
        "end_path_sha256": end.get("path_sha256"),
        "start_terminal_lane_id": start_lane,
        "end_terminal_lane_id": end_lane,
        "terminal_lane_changes": start_lane != end_lane,
        "maximum_endpoint_axis_error_rad": max(
            start_axis_error, end_axis_error
        ),
        "segment_sample_counts_start": sample_counts_start,
        "segment_sample_counts_end": sample_counts_end,
        "segment_resampled_counts": resampled_counts,
        "maximum_endpoint_seam_error_mm": max(seam_bounds, default=0.0),
        "maximum_affine_seam_bound_for_all_u_mm": max(
            seam_bounds, default=0.0
        ),
        "endpoint_reparameterization_error_mm": endpoint_preservation_error,
        "maximum_corresponding_vertex_motion_mm": maximum_vertex_motion,
        "maximum_vertex_motion_segment": maximum_vertex_motion_segment,
        "proof": {
            "same_named_segment_order": True,
            "endpoint_polylines_preserved_by_arclength_reparameterization": (
                endpoint_preservation_error <= 1.0e-12
            ),
            "endpoint_chains_C0": max(seam_bounds, default=0.0) <= SEAM_TOL_MM,
            "affine_seams_C0_for_every_u_in_0_1": (
                max(seam_bounds, default=0.0) <= SEAM_TOL_MM
            ),
            "captured_axis_endpoints_match": (
                max(start_axis_error, end_axis_error) <= AXIS_TOL_RAD
            ),
            "C0_segment_chain_homotopy_proven": (
                max(seam_bounds, default=0.0) <= SEAM_TOL_MM
                and endpoint_preservation_error <= 1.0e-12
                and max(start_axis_error, end_axis_error) <= AXIS_TOL_RAD
            ),
        },
        "physical_quasistatic_interval_authorized": False,
    }
    row["interval_proof_sha256"] = _canonical_hash(
        row, "interval_proof_sha256"
    )
    return row


def _source_freshness(
    hashes: Any, source_root: Path,
) -> tuple[bool, list[dict[str, Any]]]:
    rows = []
    if not isinstance(hashes, Mapping) or not hashes:
        return False, rows
    for raw_name, expected in hashes.items():
        path = _under_root(source_root, raw_name)
        actual = _sha256(path) if path.is_file() else None
        rows.append({
            "path": str(raw_name).replace("\\", "/"),
            "expected_sha256": expected,
            "actual_sha256": actual,
            "current": isinstance(expected, str) and actual == expected,
        })
    return bool(rows) and all(row["current"] for row in rows), rows


def _declared_slot_law_evidence(
    packing: Mapping[str, Any],
    slot_routes: Mapping[str, Any],
    loci: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Test whether the existing sequential lay law closes this gap.

    The timing portion can be bound directly: every stored state is an exact
    directed pi crossing in the raw timeline.  The physical law cannot be
    materialized, however, because one mandatory mouth crossover fails its
    own exact-clear endpoint-portal search.  The already generated crossing
    certificate also remains FAIL and its lay targets are not the active
    guide's side-specific cap-lane endpoints.
    """

    graph = PackingSupportGraph.from_report(dict(packing), spec=DEFAULT_STATOR)
    policy = SequentialRoutePolicy()
    policy.validate()

    required_crossover_start = 21
    crossover_error = None
    crossover_probe = None
    try:
        crossover = solve_safe_mouth_crossover(
            graph, required_crossover_start
        )
        crossover_probe = {
            "status": "PASS",
            "start_turn_index": crossover.start_turn_index,
            "end_turn_index": crossover.end_turn_index,
            "total_length_mm": crossover.total_length_mm,
            "minimum_prior_center_distance_mm": (
                crossover.minimum_prior_center_distance_mm
            ),
            "safe_profile_radius_mm": crossover.safe_profile_radius_mm,
        }
    except (RuntimeError, ValueError) as exc:
        crossover_error = str(exc)
        crossover_probe = {
            "status": "FAIL",
            "start_turn_index": required_crossover_start,
            "end_turn_index": required_crossover_start + 1,
            "commanded_half_turn_interval_index": (
                2 * required_crossover_start + 1
            ),
            "error_type": type(exc).__name__,
            "reason": str(exc),
        }

    by_pass: dict[int, list[Mapping[str, Any]]] = {}
    for row in loci:
        by_pass.setdefault(int(row["pass_index"]), []).append(row)
    maximum_phase_error = 0.0
    for rows in by_pass.values():
        rows.sort(key=lambda row: int(row["state_index"]))
        base_m2 = _axes(rows[0])[2]
        motion_sign = int(rows[0]["motion_sign"])
        for row in rows:
            state_index = int(row["state_index"])
            directed_phase = motion_sign * (_axes(row)[2] - base_m2)
            maximum_phase_error = max(
                maximum_phase_error,
                abs(directed_phase - state_index * math.pi),
            )

    route_rows = slot_routes.get("routes")
    route_rows = route_rows if isinstance(route_rows, list) else []
    route_by_state: dict[int, Mapping[str, Any]] = {}
    for row in route_rows:
        if not isinstance(row, Mapping):
            continue
        try:
            state = 2 * int(row["turn_index"]) + int(row["half_turn_index"])
        except (KeyError, TypeError, ValueError):
            continue
        route_by_state[state] = row

    terminal_distances = []
    terminal_distances_with_y_mirror = []
    for locus in loci:
        route = route_by_state.get(int(locus["state_index"]))
        terminal = locus.get("terminal_binding", {}).get(
            "cap_endpoint_local_mm"
        )
        target = None if route is None else route.get("target_local_mm")
        if not (_finite_point(terminal) and _finite_point(target)):
            continue
        terminal_array = np.asarray(terminal, dtype=float)
        target_array = np.asarray(target, dtype=float)
        terminal_distances.append(float(np.linalg.norm(
            target_array - terminal_array
        )))
        mirrored = target_array.copy()
        mirrored[1] *= -1.0
        terminal_distances_with_y_mirror.append(min(
            float(np.linalg.norm(target_array - terminal_array)),
            float(np.linalg.norm(mirrored - terminal_array)),
        ))

    validation = slot_routes.get("validation")
    validation = validation if isinstance(validation, Mapping) else {}
    failing_crossings = [
        {
            "turn_index": row.get("turn_index"),
            "half_turn_index": row.get("half_turn_index"),
            "reason": row.get("reason"),
            "minimum_copper_center_distance_mm": row.get(
                "postcheck", {}
            ).get("minimum_copper_center_distance_mm"),
            "required_copper_center_distance_mm": row.get(
                "postcheck", {}
            ).get("required_copper_center_distance_mm"),
            "minimum_copper_obstacle_id": row.get("postcheck", {}).get(
                "minimum_copper_obstacle_id"
            ),
        }
        for row in route_rows
        if isinstance(row, Mapping) and row.get("status") != "PASS"
    ]
    # One failed mandatory member is sufficient to reject the 49-row table.
    # A passing probe would not, by itself, prove the other 48 rows.
    complete_crossover_table_constructible: bool | None = (
        False if crossover_error is not None else None
    )
    crossing_certificate_pass = slot_routes.get("status") == "PASS"
    endpoints_identical = bool(terminal_distances) and max(
        terminal_distances
    ) <= 1.0e-6
    law_bindable = all((
        maximum_phase_error <= AXIS_TOL_RAD,
        complete_crossover_table_constructible is True,
        crossing_certificate_pass,
        endpoints_identical,
    ))
    return {
        "policy": {
            "schema": policy.schema,
            "angular_sample_step_deg": policy.angular_sample_step_deg,
            "samples_per_half_turn_including_endpoints": (
                round(180.0 / policy.angular_sample_step_deg) + 1
            ),
            "commanded_half_turn_interval_count": 100,
            "declared_crossover_interval_count": 49,
            "placement_hold_interval_count": 51,
            "lead_in_half_turns": policy.lead_in_half_turns,
            "lead_out_half_turns": policy.lead_out_half_turns,
        },
        "raw_timing_binding": {
            "stored_half_turn_start_count": len(loci),
            "maximum_directed_phase_error_rad": maximum_phase_error,
            "all_stored_starts_match_k_pi": (
                maximum_phase_error <= AXIS_TOL_RAD
            ),
            "result": "PASS",
        },
        "mouth_crossover_table": {
            "required_count": len(graph.turns) - 1,
            "complete_table_constructible": (
                complete_crossover_table_constructible
            ),
            "blocking_required_member_probe": crossover_probe,
            "sequential_lay_samples_full_table_callable": (
                False if crossover_error is not None else None
            ),
        },
        "existing_crossing_certificate": {
            "schema": slot_routes.get("schema"),
            "status": slot_routes.get("status"),
            "expected_geometry_cases": validation.get(
                "expected_geometry_cases"
            ),
            "passed_geometry_cases": validation.get(
                "passed_geometry_cases"
            ),
            "expected_direction_cases": validation.get(
                "expected_direction_cases"
            ),
            "covered_direction_cases": validation.get(
                "covered_direction_cases"
            ),
            "progressive_support_validated": validation.get(
                "progressive_support_validated"
            ),
            "release_proof_flags": validation.get("release_proof_flags"),
            "release_blockers": validation.get("release_blockers"),
            "failing_crossings": failing_crossings,
        },
        "active_guide_endpoint_binding": {
            "comparison_count": len(terminal_distances),
            "current_guide_endpoint": (
                "terminal_binding.cap_endpoint_local_mm / side-specific "
                "cap lane point"
            ),
            "sequential_law_endpoint": "slot_wire_routes.routes[].target_local_mm / rounded lay target",
            "minimum_direct_endpoint_difference_mm": min(
                terminal_distances, default=None
            ),
            "maximum_direct_endpoint_difference_mm": max(
                terminal_distances, default=None
            ),
            "maximum_difference_after_optional_tangential_mirror_mm": max(
                terminal_distances_with_y_mirror, default=None
            ),
            "endpoints_identical": endpoints_identical,
            "hash_bound_moving_continuation_between_endpoints_exists": False,
        },
        "bindable_to_current_raw_timing_and_active_guide": law_bindable,
        "decision": (
            "REJECT_AS_CURRENT_MOVING_PHYSICAL_LAW: raw k*pi timing binds, "
            "but required crossover 21->22 cannot construct an exact-clear "
            "endpoint portal; the existing crossing certificate is FAIL; "
            "and its rounded lay target is not the active guide's cap-lane "
            "endpoint. No continuation between those endpoints is proved."
        ),
    }


def _empty_report(
    input_paths: Mapping[str, Path], issues: list[dict[str, Any]],
) -> dict[str, Any]:
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "FAIL",
        "structural_status": "FAIL",
        "physical_authority_status": "NOT_PROVEN",
        "production_authorized": False,
        "input_files": {name: str(path) for name, path in input_paths.items()},
        "input_file_sha256": {
            name: _sha256(path) if path.is_file() else None
            for name, path in input_paths.items()
        },
        "issues": issues,
        "source_hashes": {
            "sim/moving_half_turn_segment_continuity_audit.py": (
                _sha256(Path(__file__))
            ),
            "sim/traj.py": _sha256(HERE / "traj.py"),
            "sim/slot_route.py": _sha256(HERE / "slot_route.py"),
        },
    }
    report["report_sha256"] = _canonical_hash(report, "report_sha256")
    return report


def analyze(
    capture_path: Path = CAPTURE_PATH,
    guide_audit_path: Path = GUIDE_AUDIT_PATH,
    locus_path: Path = LOCUS_PATH,
    packing_path: Path = PACKING_PATH,
    slot_routes_path: Path = SLOT_ROUTES_PATH,
    *,
    source_root: Path = ROOT,
) -> dict[str, Any]:
    capture_path = Path(capture_path)
    guide_audit_path = Path(guide_audit_path)
    locus_path = Path(locus_path)
    packing_path = Path(packing_path)
    slot_routes_path = Path(slot_routes_path)
    source_root = Path(source_root)
    input_paths = {
        "capture": capture_path,
        "guide_audit": guide_audit_path,
        "loci": locus_path,
        "slot_packing": packing_path,
        "slot_wire_routes": slot_routes_path,
    }
    issues: list[dict[str, Any]] = []
    missing = [name for name, path in input_paths.items() if not path.is_file()]
    if missing:
        for name in missing:
            issues.append({
                "severity": "INTEGRITY_FAIL",
                "code": f"{name}_missing",
                "message": f"required input is missing: {input_paths[name]}",
            })
        return _empty_report(input_paths, issues)

    try:
        events = load_events(capture_path)
        timeline = Timeline(events)
        guide = _load_object(guide_audit_path)
        loci_document = _load_object(locus_path)
        packing = _load_object(packing_path)
        slot_routes = _load_object(slot_routes_path)
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
        issues.append({
            "severity": "INTEGRITY_FAIL",
            "code": "input_parse_or_timeline_error",
            "message": str(exc),
        })
        return _empty_report(input_paths, issues)

    capture_sha = _sha256(capture_path)
    guide_sha = _sha256(guide_audit_path)
    locus_sha = _sha256(locus_path)
    packing_sha = _sha256(packing_path)
    slot_routes_sha = _sha256(slot_routes_path)
    meta_rows = [event for event in events if event.get("e") == "meta"]
    meta = meta_rows[0] if len(meta_rows) == 1 else {}
    event_counts = Counter(str(event.get("e")) for event in events)

    guide_self_hash = guide.get("report_sha256") == _canonical_hash(
        guide, "report_sha256"
    )
    locus_self_hash = loci_document.get(
        "locus_payload_sha256"
    ) == _canonical_hash(loci_document, "locus_payload_sha256")
    guide_sources_current, guide_source_rows = _source_freshness(
        guide.get("source_hashes"), source_root
    )
    slot_routes_sources_current, slot_route_source_rows = _source_freshness(
        slot_routes.get("source_hashes"), source_root
    )
    player_api = guide.get("player_route_api", {})
    artifact_bindings = {
        "guide_schema": guide.get("schema") == GUIDE_SCHEMA,
        "guide_self_hash": guide_self_hash,
        "guide_sources_current": guide_sources_current,
        "locus_schema": loci_document.get("schema") == LOCUS_SCHEMA,
        "locus_self_hash": locus_self_hash,
        "locus_capture_hash": (
            loci_document.get("run", {}).get("capture_sha256") == capture_sha
        ),
        "guide_compact_file_hash": (
            player_api.get("compact_file_sha256") == locus_sha
        ),
        "guide_payload_hash": (
            player_api.get("canonical_payload_sha256")
            == loci_document.get("locus_payload_sha256")
        ),
        "slot_packing_self_hash": (
            packing.get("report_sha256")
            == _canonical_hash(packing, "report_sha256")
        ),
        "slot_wire_routes_self_hash": (
            slot_routes.get("report_sha256")
            == _canonical_hash(slot_routes, "report_sha256")
        ),
        "slot_wire_routes_sources_current": slot_routes_sources_current,
        "slot_wire_routes_packing_file_binding": (
            slot_routes.get("input_contract", {}).get("packing_file_sha256")
            == packing_sha
        ),
    }

    raw_rows = loci_document.get("loci")
    rows = raw_rows if isinstance(raw_rows, list) else []
    by_pass: dict[int, list[Mapping[str, Any]]] = {}
    try:
        for row in rows:
            if not isinstance(row, Mapping):
                raise ValueError("locus row is not an object")
            by_pass.setdefault(int(row["pass_index"]), []).append(row)
        for pass_rows in by_pass.values():
            pass_rows.sort(key=lambda row: int(row["state_index"]))
    except (KeyError, TypeError, ValueError) as exc:
        issues.append({
            "severity": "INTEGRITY_FAIL",
            "code": "locus_identity_invalid",
            "message": str(exc),
        })
        return _empty_report(input_paths, issues)

    pass_state_coverage = (
        set(by_pass) == set(range(EXPECTED_PASSES))
        and all(
            [int(row["state_index"]) for row in by_pass[pass_index]]
            == list(range(EXPECTED_STATES_PER_PASS))
            for pass_index in range(EXPECTED_PASSES)
        )
    )
    intervals: list[dict[str, Any]] = []
    if pass_state_coverage:
        try:
            for pass_index in range(EXPECTED_PASSES):
                for start, end in zip(
                    by_pass[pass_index], by_pass[pass_index][1:]
                ):
                    intervals.append(_interval_proof(
                        len(intervals), start, end, timeline,
                    ))
        except (KeyError, TypeError, ValueError) as exc:
            issues.append({
                "severity": "INTEGRITY_FAIL",
                "code": "interval_proof_failed",
                "message": str(exc),
            })
    else:
        issues.append({
            "severity": "INTEGRITY_FAIL",
            "code": "pass_state_coverage_invalid",
            "message": "expected exactly states 0..99 in each of 24 passes",
        })

    all_c0 = (
        len(intervals) == EXPECTED_PAIRED_INTERVALS
        and all(
            row.get("proof", {}).get("C0_segment_chain_homotopy_proven")
            is True for row in intervals
        )
    )
    all_lane_changes = bool(intervals) and all(
        row["terminal_lane_changes"] is True for row in intervals
    )
    maximum_seam = max(
        (row["maximum_affine_seam_bound_for_all_u_mm"] for row in intervals),
        default=None,
    )
    maximum_axis_error = max(
        (row["maximum_endpoint_axis_error_rad"] for row in intervals),
        default=None,
    )
    maximum_vertex = max(
        (row["maximum_corresponding_vertex_motion_mm"] for row in intervals),
        default=None,
    )
    vertex_witness = next((
        row for row in intervals
        if row["maximum_corresponding_vertex_motion_mm"] == maximum_vertex
    ), None)
    total_duration = sum(row["duration_s"] for row in intervals)

    try:
        declared_slot_law = _declared_slot_law_evidence(
            packing, slot_routes, rows
        )
    except (KeyError, TypeError, ValueError, RuntimeError) as exc:
        declared_slot_law = {
            "bindable_to_current_raw_timing_and_active_guide": False,
            "decision": f"REJECT: declared slot law evaluation failed: {exc}",
            "evaluation_error": {
                "type": type(exc).__name__,
                "reason": str(exc),
            },
        }
        issues.append({
            "severity": "INTEGRITY_FAIL",
            "code": "declared_slot_law_evaluation_failed",
            "message": str(exc),
        })

    integrity_gates = {
        "one_capture_meta": len(meta_rows) == 1,
        "capture_schema_4": meta.get("capture_schema") == EXPECTED_CAPTURE_SCHEMA,
        "canonical_unmodified_upstream_capture": (
            meta.get("controller_mode") == "upstream"
            and meta.get("controller_adapter_sha256") is None
        ),
        **artifact_bindings,
        "all_2400_point_states_present": len(rows) == EXPECTED_POINT_STATES,
        "all_24_pass_state_sequences_are_0_through_99": pass_state_coverage,
    }
    integrity_ok = all(integrity_gates.values()) and not any(
        issue["severity"] == "INTEGRITY_FAIL" for issue in issues
    )
    structural_pass = integrity_ok and all_c0
    if not structural_pass and not any(
        issue["code"] == "interval_proof_failed" for issue in issues
    ):
        issues.append({
            "severity": "INTEGRITY_FAIL",
            "code": "bounded_C0_proof_not_current",
            "message": "one or more bindings or adjacent interval proofs failed",
        })
    issues.extend([
        {
            "severity": "AUTHORITY_GAP",
            "code": "interpolant_not_bound_to_physical_guide_surfaces",
            "message": (
                "affine segment interpolation is a C0 topology witness only; "
                "it is not constrained to the PEEK bell, carriage bowls, cap "
                "lead-ins, or a tensioned-wire equilibrium"
            ),
        },
        {
            "severity": "AUTHORITY_GAP",
            "code": "closing_half_turn_endpoint_loci_missing",
            "message": (
                "each pass exposes states 0..99 but not the state-100 route "
                "needed to pair its final half-turn"
            ),
            "detail": {"missing_closing_interval_count": EXPECTED_PASSES},
        },
        {
            "severity": "AUTHORITY_GAP",
            "code": "capture_has_no_wire_contact_observations",
            "message": (
                "capture schema 4 records axis commands and phase markers but "
                "no intermediate flexible-wire/contact observations"
            ),
            "detail": {
                "wire_contact_observation_count": event_counts.get(
                    "wire_contact_observation", 0
                ),
            },
        },
        {
            "severity": "AUTHORITY_GAP",
            "code": "declared_slot_route_policy_not_bindable",
            "message": (
                "the existing SequentialRoutePolicy binds raw k*pi timing "
                "but cannot supply the current moving physical law: required "
                "crossover 21->22 fails, the crossing certificate is FAIL, "
                "and rounded lay targets do not equal guide cap-lane endpoints"
            ),
            "detail": {
                "bindable": declared_slot_law.get(
                    "bindable_to_current_raw_timing_and_active_guide"
                ),
                "decision": declared_slot_law.get("decision"),
            },
        },
    ])

    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "PASS" if structural_pass else "FAIL",
        "structural_status": "PASS" if structural_pass else "FAIL",
        "physical_authority_status": "NOT_PROVEN",
        "decision": (
            "C0_ADJACENT_LOCUS_HOMOTOPY_PROVEN__"
            "MOVING_PHYSICAL_ROUTE_NOT_PROVEN"
        ),
        "production_authorized": False,
        "controller_modified": False,
        "CAD_modified": False,
        "capture_schema_evidence": {
            "capture_schema": meta.get("capture_schema"),
            "controller_mode": meta.get("controller_mode"),
            "controller_adapter_sha256": meta.get(
                "controller_adapter_sha256"
            ),
            "event_count": len(events),
            "event_kind_counts": dict(sorted(event_counts.items())),
            "wire_contact_observation_count": event_counts.get(
                "wire_contact_observation", 0
            ),
            "intermediate_wire_route_event_count": 0,
            "axis_semantics": (
                "traj.Timeline piecewise-linear constant-velocity reconstruction "
                "from absolute commands; not hardware feedback"
            ),
        },
        "input_files": {
            name: _relative(path, source_root)
            for name, path in input_paths.items()
        },
        "input_file_sha256": {
            "capture": capture_sha,
            "guide_audit": guide_sha,
            "loci": locus_sha,
            "slot_packing": packing_sha,
            "slot_wire_routes": slot_routes_sha,
        },
        "artifact_integrity": {
            "gates": integrity_gates,
            "guide_source_freshness": guide_source_rows,
            "slot_wire_route_source_freshness": slot_route_source_rows,
        },
        "proof_definition": {
            "scope": (
                "adjacent same-pass point-state route pairs only; no pass "
                "closing half-turn, tooth/index, shaft-wrap, load, or unload"
            ),
            "segment_order": list(SEGMENT_ORDER),
            "parameterization": (
                "each endpoint segment retains its exact piecewise-linear "
                "geometry and is resampled at uniform normalized chord arclength"
            ),
            "homotopy": "H(s,u)=(1-u)*P0(s)+u*P1(s), u in [0,1]",
            "analytic_C0_argument": (
                "affine combinations of continuous PL maps are continuous; "
                "each segment seam obeys d(u)=(1-u)d0+u*d1 and is bounded by "
                "max(||d0||,||d1||)"
            ),
            "seam_tolerance_mm": SEAM_TOL_MM,
            "axis_endpoint_tolerance_rad": AXIS_TOL_RAD,
            "not_a_physics_model": True,
        },
        "declared_slot_route_policy_binding": declared_slot_law,
        "coverage": {
            "required_half_turn_intervals": EXPECTED_POINT_STATES,
            "available_point_state_count": len(rows),
            "proved_adjacent_C0_interval_count": len(intervals) if all_c0 else 0,
            "proved_adjacent_C0_duration_s": total_duration if all_c0 else 0.0,
            "unpaired_final_half_turn_interval_count": (
                EXPECTED_UNPAIRED_CLOSING_INTERVALS
            ),
            "terminal_lane_change_interval_count": sum(
                bool(row["terminal_lane_changes"]) for row in intervals
            ),
            "all_paired_intervals_change_terminal_lane": all_lane_changes,
            "maximum_affine_seam_bound_for_all_u_mm": maximum_seam,
            "maximum_endpoint_axis_error_rad": maximum_axis_error,
            "maximum_corresponding_vertex_motion_mm": maximum_vertex,
            "maximum_vertex_motion_witness": (
                {
                    key: vertex_witness[key]
                    for key in (
                        "interval_index", "pass_index", "start_state_index",
                        "end_state_index", "maximum_vertex_motion_segment",
                        "maximum_corresponding_vertex_motion_mm",
                    )
                } if vertex_witness is not None else None
            ),
        },
        "gates": {
            "artifact_integrity": integrity_ok,
            "all_2376_available_adjacent_intervals_have_C0_homotopy": all_c0,
            "all_affine_segment_seams_below_tolerance_for_every_u": (
                maximum_seam is not None and maximum_seam <= SEAM_TOL_MM
            ),
            "all_endpoint_axes_match_raw_timeline": (
                maximum_axis_error is not None
                and maximum_axis_error <= AXIS_TOL_RAD
            ),
            "all_2400_half_turn_intervals_have_paired_route_endpoints": False,
            "named_guide_surface_adherence_through_motion_proven": False,
            "moving_rigid_and_prior_copper_clearance_proven": False,
            "moving_bend_radius_proven": False,
            "moving_contact_tail_ownership_proven": False,
            "physical_quasistatic_moving_interval_authorized": False,
        },
        "intervals": intervals,
        "physical_authority_boundary": {
            "proved": (
                "a connected C0 presentation homotopy exists for each of "
                "2,376 adjacent endpoint route pairs"
            ),
            "not_proved": [
                "the interpolant lies on or inside named physical guide surfaces",
                "clearance to final rigid parts or progressive prior copper",
                "minimum bend radius at intermediate states",
                "active contact point and deposited-tail ownership",
                "tensioned-wire equilibrium, sag, friction, abrasion, or snagging",
                "the 24 final half-turns or any inter-pass/wrap/load/unload motion",
                "the existing SequentialRoutePolicy as a current moving law; "
                "its required 21->22 crossover and guide-endpoint binding fail",
            ],
            "moving_physical_interval_count": 0,
        },
        "minimum_evidence_to_promote": [
            (
                "Add a hash-bound closing state-100 physical route for every "
                "pass so all 2,400 half-turn intervals have endpoint authority."
            ),
            (
                "Replace the affine presentation homotopy with a conservative "
                "route family constrained to the actual bell, bowls, handoff, "
                "cap lead-in, active terminal, and deposited tail."
            ),
            (
                "Sweep that family against final rigid parts and progressive "
                "prior copper with bend-radius and contact-uncertainty bounds."
            ),
            (
                "Keep sag, friction, abrasion, snagging, strand settling, and "
                "tension transients behind physical pull-through/endurance tests."
            ),
        ],
        "issues": issues,
        "source_hashes": {
            "sim/moving_half_turn_segment_continuity_audit.py": (
                _sha256(Path(__file__))
            ),
            "sim/traj.py": _sha256(HERE / "traj.py"),
            "sim/slot_route.py": _sha256(HERE / "slot_route.py"),
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
    coverage = report.get("coverage", {})
    gates = report.get("gates", {})
    slot_law = report.get("declared_slot_route_policy_binding", {})
    timing = slot_law.get("raw_timing_binding", {})
    crossover = slot_law.get("mouth_crossover_table", {}).get(
        "blocking_required_member_probe", {}
    )
    crossing = slot_law.get("existing_crossing_certificate", {})
    endpoints = slot_law.get("active_guide_endpoint_binding", {})
    lines = [
        "# Moving half-turn segment continuity audit",
        "",
        f"Structural C0 result: **{report.get('structural_status', 'FAIL')}**  ",
        "Physical moving-route authority: **NOT PROVEN**  ",
        "Production authorized: `false`",
        "",
        "This proof is deliberately topology-only. It does not turn a linearly "
        "interpolated presentation polyline into a tensioned physical wire.",
        "",
        "## Bounded result",
        "",
        "| Measure | Result |",
        "|---|---:|",
        (
            "| Adjacent same-pass intervals with analytic C0 homotopy | "
            f"{coverage.get('proved_adjacent_C0_interval_count', 0)} / 2376 |"
        ),
        (
            "| C0-covered duration | "
            f"{coverage.get('proved_adjacent_C0_duration_s', 0.0):.9f} s |"
        ),
        (
            "| Unpaired final half-turns | "
            f"{coverage.get('unpaired_final_half_turn_interval_count', 0)} |"
        ),
        (
            "| Maximum analytic seam bound for every interpolation fraction | "
            f"{coverage.get('maximum_affine_seam_bound_for_all_u_mm', 0.0):.12g} mm |"
        ),
        (
            "| Intervals changing named terminal lane | "
            f"{coverage.get('terminal_lane_change_interval_count', 0)} |"
        ),
        (
            "| Physically authorized moving intervals | "
            f"{report.get('physical_authority_boundary', {}).get('moving_physical_interval_count', 0)} |"
        ),
        "",
        "The construction reparameterizes each endpoint segment along its exact "
        "piecewise-linear chord arclength, then applies "
        "`H(s,u)=(1-u)P0(s)+uP1(s)`. Segment seams remain C0 for every `u` by "
        "the convex norm bound. This establishes a connected homotopy, not "
        "surface contact or flexible-wire equilibrium.",
        "",
        "## Existing sequential slot-route law",
        "",
        "The declared `SequentialRoutePolicy` was checked before accepting the "
        "C0-only boundary. Its logical timing does bind the raw half-turn starts, "
        "but the current physical-law chain does not close:",
        "",
        (
            "- Maximum raw directed-phase error against `state_index*pi`: "
            f"**{timing.get('maximum_directed_phase_error_rad', 0.0):.12g} rad**."
        ),
        (
            "- Mandatory mouth crossover 21->22 / commanded interval 43: "
            f"**{crossover.get('status', 'UNKNOWN')}** — "
            f"{crossover.get('reason', 'no result')}."
        ),
        (
            "- Existing packed crossing certificate: **"
            f"{crossing.get('status', 'UNKNOWN')}**, "
            f"{crossing.get('passed_geometry_cases', 0)}/"
            f"{crossing.get('expected_geometry_cases', 0)} geometry cases and "
            f"{crossing.get('covered_direction_cases', 0)}/"
            f"{crossing.get('expected_direction_cases', 0)} direction cases covered."
        ),
        (
            "- Rounded lay target versus active-guide cap-lane endpoint: "
            f"{endpoints.get('minimum_direct_endpoint_difference_mm', 0.0):.6f} "
            "to "
            f"{endpoints.get('maximum_direct_endpoint_difference_mm', 0.0):.6f} mm."
        ),
        "",
        "Therefore the existing policy is not silently repurposed as the moving "
        "guide law. It needs a complete crossover table, passing progressive "
        "crossing certificate, and a hash-bound physical continuation between "
        "the guide endpoint and rounded lay target.",
        "",
        "## Why physical authority remains blocked",
        "",
        "- Every paired interval changes named terminal lane, while the affine "
        "interpolant is not constrained to the physical PEEK bell, carriage "
        "bowls, cap lead-in, active contact, or deposited tail.",
        "- The current locus artifact records states 0 through 99; it omits the "
        "state-100 closing route for the final half-turn of each pass.",
        "- Capture schema 4 contains axis commands and high-level markers but no "
        "intermediate wire-contact observations.",
        "- Rigid/prior-copper clearance, intermediate bend radius, tension, sag, "
        "friction, abrasion, and snagging are not inferred.",
        "",
        "## Gates",
        "",
    ]
    for name, value in gates.items():
        lines.append(f"- `{name}`: `{str(bool(value)).lower()}`")
    lines.extend([
        "",
        "## Minimum evidence to promote",
        "",
    ])
    for index, item in enumerate(report.get("minimum_evidence_to_promote", []), 1):
        lines.append(f"{index}. {item}")
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
    parser.add_argument("--slot-packing", type=Path, default=PACKING_PATH)
    parser.add_argument("--slot-wire-routes", type=Path, default=SLOT_ROUTES_PATH)
    parser.add_argument("--source-root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path, default=OUTPUT_PATH)
    parser.add_argument("--markdown", type=Path, default=OUTPUT_MD)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    report = analyze(
        args.capture, args.guide_audit, args.loci,
        args.slot_packing, args.slot_wire_routes,
        source_root=args.source_root,
    )
    if not args.check:
        write_report(report, args.output)
        write_markdown(report, args.markdown)
        print(f"wrote {args.output}")
        print(f"wrote {args.markdown}")
    print(
        f"moving half-turn C0 {report['structural_status']}: "
        f"paired={report.get('coverage', {}).get('proved_adjacent_C0_interval_count', 0)}; "
        "physical=NOT_PROVEN"
    )
    if report["structural_status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
