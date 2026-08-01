"""Fail-closed moving-half-turn conductor integration audit.

CAD brief
---------
* Task: source-level geometry/trajectory authority audit; no CAD or STEP is
  generated or edited.
* Units/frames: millimetres in normalized active-tooth stator-local XYZ;
  raw M0/M1/M2 are radians in the canonical upstream Timeline.
* Inputs: the unmodified raw capture, constructive slot-winding plan,
  ``slot_route`` sequential policy/crossover model, and the current physical
  PEEK bell/selection-bowl/short-leadin compact locus artifact.
* Required proof: all 2,400 raw half-turn clocks must agree with the planned
  rounded-loop lay state; all 49 turn-to-turn crossovers must solve; the
  physical guide-to-rounded-loop continuation must be explicit; every named
  contact and free bend must be R >= 3 mm; and the complete moving family must
  receive exact core/prior-copper/unintended-contact postchecks.
* Output: ``out/reports/moving_half_turn_conductor_audit.{json,md}``.

The audit does not convert state-space paths into physical wire by assertion.
``MouthCrossover`` lives in radial/profile state space, while the active guide
artifact ends at a physical cap ``riser_top``.  They contribute authority only
after a real 3D continuation and all postchecks exist.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

import numpy as np
from build123d import Vertex


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
PLAN = REPORTS / "slot_winding_plan.json"
GUIDE_REPORT = REPORTS / "carriage_active_sector_terminal_guide_audit.json"
GUIDE_LOCI = REPORTS / "carriage_active_sector_terminal_guide_loci.json"
AGGREGATE = REPORTS / "permanent_cap_aggregate_authorization.json"
OUTPUT_JSON = REPORTS / "moving_half_turn_conductor_audit.json"
OUTPUT_MD = REPORTS / "moving_half_turn_conductor_audit.md"

for folder in (HERE, CAD):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

from params import DEFAULT_STATOR, PARAMS  # noqa: E402
import permanent_cap_production_review as cap  # noqa: E402
from phase_aware_progressive_wire_audit import (  # noqa: E402
    _directed_crossing_times,
)
from slot_route import (  # noqa: E402
    PackingSupportGraph,
    SequentialRoutePolicy,
    _rounded_loop_yz,
    dependency_versions,
    sequential_lay_samples,
    solve_safe_mouth_crossover,
)
from traj import Timeline, load_events, winding_windows  # noqa: E402
import wire_geometry  # noqa: E402


SCHEMA = "moving-half-turn-conductor-audit/v1"
EXPECTED_PASSES = 24
EXPECTED_HALF_TURNS_PER_PASS = 100
EXPECTED_RAW_INTERVALS = EXPECTED_PASSES * EXPECTED_HALF_TURNS_PER_PASS
EXPECTED_CROSSOVERS = 49
REQUIRED_BEND_RADIUS_MM = 3.0
RAW_TIME_TOL_S = 2.0e-6
RAW_AXIS_TOL_RAD = 2.0e-8
RADIAL_TARGET_TOL_MM = 1.0e-6
CURRENT_TERMINAL_SEGMENTS = {
    "carriage_selection_bowl",
    "dynamic_handoff_gap",
    "spindle_short_leadin",
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(value: Any, field: str | None = None) -> str:
    body = deepcopy(value)
    if field is not None and isinstance(body, dict):
        body.pop(field, None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load(path: Path) -> dict[str, Any]:
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


def _current_source_rows(hashes: Mapping[str, Any]) -> list[dict[str, Any]]:
    rows = []
    for relative, expected in hashes.items():
        path = ROOT / str(relative).replace("/", "\\")
        actual = _sha256(path) if path.is_file() else None
        rows.append({
            "path": str(relative).replace("\\", "/"),
            "expected_sha256": expected,
            "actual_sha256": actual,
            "current": isinstance(expected, str) and actual == expected,
        })
    return rows


def _guide_evidence(
    guide_report: Mapping[str, Any],
    loci_document: Mapping[str, Any],
    aggregate: Mapping[str, Any],
    guide_loci_path: Path,
) -> dict[str, Any]:
    guide_hash_ok = (
        guide_report.get("report_sha256")
        == _canonical_hash(guide_report, "report_sha256")
    )
    loci_hash_ok = (
        loci_document.get("locus_payload_sha256")
        == _canonical_hash(loci_document, "locus_payload_sha256")
    )
    aggregate_hash_ok = (
        aggregate.get("report_sha256")
        == _canonical_hash(aggregate, "report_sha256")
    )
    source_rows = _current_source_rows(
        guide_report.get("source_hashes", {})
        if isinstance(guide_report.get("source_hashes"), Mapping) else {}
    )
    player_api = guide_report.get("player_route_api", {})
    rows = loci_document.get("loci", [])
    segment_contract = loci_document.get("segment_contract", {})
    segment_names = (
        set(segment_contract)
        if isinstance(segment_contract, Mapping) else set()
    )
    def expected_endpoint_name(row: Mapping[str, Any]) -> str | None:
        binding = row.get("terminal_binding", {})
        lane_id = str(binding.get("lane_id", ""))
        if "_left_" in lane_id:
            return "_lane_points()['riser_top']"
        if "_right_" in lane_id:
            return "_lane_points()['waypoint']"
        return None

    endpoints_current = (
        isinstance(rows, list)
        and len(rows) == EXPECTED_RAW_INTERVALS
        and all(
            isinstance(row, Mapping)
            and expected_endpoint_name(row) is not None
            and row.get("terminal_binding", {}).get("cap_endpoint_name")
            == expected_endpoint_name(row)
            for row in rows
        )
    )
    cap_lane_wires = {
        -1: cap.lane_wire(-1),
        1: cap.lane_wire(1),
    }
    endpoint_seam_rows = []
    for index, row in enumerate(rows if isinstance(rows, list) else []):
        terminal_binding = row.get("terminal_binding", {})
        point = terminal_binding.get("cap_endpoint_local_mm")
        lane_id = str(terminal_binding.get("lane_id", ""))
        if not (
            isinstance(point, Sequence) and len(point) == 3
            and all(_finite(value) for value in point)
            and ("_front" in lane_id or "_rear" in lane_id)
        ):
            distance = math.inf
            sign = None
        else:
            sign = 1 if "_front" in lane_id else -1
            distance = float(
                cap_lane_wires[sign].distance_to(
                    Vertex(*map(float, point))
                )
            )
        endpoint_seam_rows.append({
            "locus_index": index,
            "lane_id": lane_id,
            "axial_sign": sign,
            "cap_endpoint_local_mm": (
                list(map(float, point))
                if isinstance(point, Sequence) and len(point) == 3 else None
            ),
            "exact_distance_to_actual_cap_lane_wire_mm": distance,
            "connected": distance <= 1.0e-6,
        })
    connected_endpoint_count = sum(
        row["connected"] for row in endpoint_seam_rows
    )
    worst_endpoint = max(
        endpoint_seam_rows,
        key=lambda row: row["exact_distance_to_actual_cap_lane_wire_mm"],
        default=None,
    )
    terminal = guide_report.get("terminal_deposition_route", {})
    guide_gates = guide_report.get("release_gates", {})
    lane = aggregate.get("cap_support_lane", {})
    radii = {
        "physical_bell_wire_center_mm": terminal.get(
            "minimum_bell_wire_center_radius_mm"
        ),
        "bell_straight_bore_handoff_mm": terminal.get(
            "minimum_straight_bore_handoff_radius_over_0p20_to_0p50mm_wire_mm"
        ),
        "carriage_selection_bowl_mm": 3.25,
        "spindle_short_leadin_mm": 3.50,
        "permanent_cap_lane_mm": lane.get(
            "minimum_lane_wire_center_bend_radius_mm"
        ),
    }
    minimum_radius = min(
        (float(value) for value in radii.values() if _finite(value)),
        default=None,
    )
    gates = {
        "guide_report_self_hash": guide_hash_ok,
        "guide_report_sources_current": bool(source_rows) and all(
            row["current"] for row in source_rows
        ),
        "compact_loci_self_hash": loci_hash_ok,
        "compact_loci_file_bound": (
            player_api.get("compact_file_sha256") == _sha256(guide_loci_path)
        ),
        "aggregate_self_hash_and_pass": (
            aggregate_hash_ok and aggregate.get("status") == "PASS"
        ),
        "all_2400_physical_terminal_routes_pass": (
            guide_gates.get("all_2400_physical_bell_terminal_routes_pass")
            is True
        ),
        "current_selection_handoff_leadin_topology": (
            CURRENT_TERMINAL_SEGMENTS <= segment_names and endpoints_current
        ),
        "all_2400_short_leadin_endpoints_join_actual_cap_lane": (
            len(endpoint_seam_rows) == EXPECTED_RAW_INTERVALS
            and connected_endpoint_count == EXPECTED_RAW_INTERVALS
        ),
        "all_named_guide_bend_radii_ge_3mm": (
            minimum_radius is not None
            and minimum_radius >= REQUIRED_BEND_RADIUS_MM
        ),
    }
    return {
        "status": "PASS" if all(gates.values()) else "FAIL",
        "authority_scope": (
            "static supply through current physical bell, carriage selection "
            "bowl, open handoff, short lead-in and permanent-cap lane"
        ),
        "terminal_route_locus_count": len(rows) if isinstance(rows, list) else 0,
        "terminal_endpoint_by_side": {
            "left": "_lane_points()['riser_top']",
            "right": "_lane_points()['waypoint']",
        },
        "exact_short_leadin_to_cap_lane_seam": {
            "evaluated_locus_count": len(endpoint_seam_rows),
            "connected_locus_count": connected_endpoint_count,
            "disconnected_locus_count": (
                len(endpoint_seam_rows) - connected_endpoint_count
            ),
            "tolerance_mm": 1.0e-6,
            "maximum_exact_distance_mm": (
                None if worst_endpoint is None else worst_endpoint[
                    "exact_distance_to_actual_cap_lane_wire_mm"
                ]
            ),
            "worst_witness": worst_endpoint,
            "meaning": (
                "the compact endpoint label is checked against the actual "
                "cad/permanent_cap_production_review.py lane_wire BREP; a "
                "label alone cannot create a C0 conductor seam"
            ),
        },
        "named_wire_center_radii_mm": radii,
        "minimum_named_wire_center_radius_mm": minimum_radius,
        "source_freshness": source_rows,
        "gates": gates,
    }


def _raw_timing_and_target_evidence(
    events: list[dict[str, Any]],
    timeline: Timeline,
    loci_document: Mapping[str, Any],
    graph: PackingSupportGraph,
) -> dict[str, Any]:
    windows = winding_windows(events)
    loci = loci_document.get("loci", [])
    if not isinstance(loci, list):
        loci = []
    by_key = {
        (int(row["pass_index"]), int(row["state_index"])): row
        for row in loci
        if isinstance(row, Mapping)
        and isinstance(row.get("pass_index"), int)
        and isinstance(row.get("state_index"), int)
    }
    timing_error = 0.0
    axis_error = 0.0
    phase_error = 0.0
    radial_errors: list[float] = []
    endpoint_gaps: list[float] = []
    duration_rows: list[float] = []
    worst_radial = None
    target_digest_rows = []
    cap_points = cap._lane_points()

    for pass_index, window in enumerate(windows):
        direction = 1 if bool(window["clockwise"]) else -1
        start_m2 = float(timeline.axes[2].pos_at(float(window["motionStart"])))
        crossings = _directed_crossing_times(
            timeline.axes[2], float(window["motionStart"]), start_m2,
            direction, EXPECTED_HALF_TURNS_PER_PASS + 1,
        )
        if len(crossings) != EXPECTED_HALF_TURNS_PER_PASS + 1:
            raise RuntimeError(
                f"raw pass {pass_index} has {len(crossings)} of 101 crossings"
            )
        intervals = tuple(zip(crossings, crossings[1:]))
        for state_index, (start, end) in enumerate(intervals):
            duration_rows.append(float(end - start))
            row = by_key.get((pass_index, state_index))
            if row is None:
                timing_error = math.inf
                axis_error = math.inf
                phase_error = math.inf
                continue
            timing_error = max(
                timing_error, abs(float(row["time_s"]) - float(start))
            )
            axes = row.get("axes", {})
            raw_pose = tuple(map(float, timeline.pose_at(float(row["time_s"]))))
            artifact_pose = (
                float(axes["M0_raw_rad"]),
                float(axes["M1_spindle_rad"]),
                float(axes["M2_flyer_rad"]),
            )
            axis_error = max(
                axis_error,
                max(abs(a - b) for a, b in zip(raw_pose, artifact_pose)),
            )
            expected_m2 = start_m2 + direction * state_index * math.pi
            phase_error = max(
                phase_error, abs(raw_pose[2] - expected_m2)
            )

            turn_index = state_index // 2
            half_turn_index = state_index & 1
            turn = graph.turn(turn_index)
            actual_radial = (
                float(PARAMS.stator_axis_z(raw_pose[0]))
                - float(wire_geometry.TOOTH_CONTACT_Z)
            )
            radial_error = actual_radial - float(turn.radial_mm)
            radial_errors.append(radial_error)
            if worst_radial is None or abs(radial_error) > abs(
                    worst_radial["signed_error_mm"]):
                worst_radial = {
                    "pass_index": pass_index,
                    "state_index": state_index,
                    "turn_index": turn_index,
                    "time_s": float(start),
                    "actual_raw_M0_radial_target_mm": actual_radial,
                    "sequential_policy_radial_target_mm": float(
                        turn.radial_mm
                    ),
                    "signed_error_mm": radial_error,
                }

            logical_phase = state_index * math.pi
            yz = _rounded_loop_yz(
                float(turn.profile_radius_mm), logical_phase, DEFAULT_STATOR
            )
            target = np.array((float(turn.radial_mm), *map(float, yz)))
            side = str(row.get("terminal_binding", {}).get("lane_id", ""))
            endpoint_name = "start" if "_left_" in side else "end"
            endpoint = np.asarray(cap_points[endpoint_name], dtype=float)
            if "_rear" in side:
                endpoint[2] *= -1.0
            endpoint_gaps.append(float(np.linalg.norm(endpoint - target)))
            target_digest_rows.append([
                pass_index, state_index, *np.round(target, 9).tolist(),
                round(actual_radial, 9),
            ])

    exact_radial = sum(
        abs(error) <= RADIAL_TARGET_TOL_MM for error in radial_errors
    )
    gates = {
        "all_24_raw_passes_present": len(windows) == EXPECTED_PASSES,
        "all_2400_half_turn_start_loci_present": (
            len(by_key) == EXPECTED_RAW_INTERVALS
        ),
        "locus_times_match_raw_half_turn_starts": (
            timing_error <= RAW_TIME_TOL_S
        ),
        "locus_axes_match_raw_timeline": axis_error <= RAW_AXIS_TOL_RAD,
        "raw_M2_phase_matches_half_turn_index": phase_error <= RAW_AXIS_TOL_RAD,
        "every_raw_M0_pose_matches_sequential_radial_target": (
            len(radial_errors) == EXPECTED_RAW_INTERVALS
            and exact_radial == EXPECTED_RAW_INTERVALS
        ),
        "physical_cap_endpoint_to_rounded_target_continuation_explicit": False,
    }
    return {
        "status": "PASS" if all(gates.values()) else "FAIL",
        "raw_pass_count": len(windows),
        "raw_half_turn_interval_count": len(radial_errors),
        "maximum_locus_time_error_s": timing_error,
        "maximum_locus_axis_error_rad": axis_error,
        "maximum_M2_logical_phase_error_rad": phase_error,
        "raw_M0_exact_sequential_target_count": exact_radial,
        "raw_M0_target_mismatch_count": len(radial_errors) - exact_radial,
        "maximum_absolute_raw_M0_target_error_mm": max(
            (abs(value) for value in radial_errors), default=None
        ),
        "mean_absolute_raw_M0_target_error_mm": (
            sum(abs(value) for value in radial_errors) / len(radial_errors)
            if radial_errors else None
        ),
        "worst_raw_M0_target_witness": worst_radial,
        "minimum_unmodeled_cap_endpoint_to_rounded_target_gap_mm": min(
            endpoint_gaps, default=None
        ),
        "maximum_unmodeled_cap_endpoint_to_rounded_target_gap_mm": max(
            endpoint_gaps, default=None
        ),
        "minimum_raw_half_turn_duration_s": min(duration_rows, default=None),
        "maximum_raw_half_turn_duration_s": max(duration_rows, default=None),
        "target_binding_sha256": _canonical_hash(target_digest_rows),
        "gates": gates,
    }


def _crossover_evidence(graph: PackingSupportGraph) -> dict[str, Any]:
    rows = []
    first_failure = None
    for index in range(EXPECTED_CROSSOVERS):
        try:
            path = solve_safe_mouth_crossover(graph, index)
        except Exception as exc:  # exact fail-closed counterexample
            first_failure = {
                "start_turn_index": index,
                "end_turn_index": index + 1,
                "exception_type": type(exc).__name__,
                "reason": str(exc),
            }
            break
        minimum = float(path.minimum_prior_center_distance_mm)
        rows.append({
            "start_turn_index": index,
            "end_turn_index": index + 1,
            "safe_profile_radius_mm": float(path.safe_profile_radius_mm),
            "waypoint_count": len(path.waypoints_radial_profile_mm),
            "total_state_space_length_mm": float(path.total_length_mm),
            "minimum_prior_center_distance_mm": (
                None if not math.isfinite(minimum) else minimum
            ),
            "planner_guard_mm": float(path.planner_guard_mm),
            "waypoint_sha256": _canonical_hash(
                [list(point) for point in path.waypoints_radial_profile_mm]
            ),
        })

    zero_guard = None
    if first_failure is not None:
        index = int(first_failure["start_turn_index"])
        try:
            path = solve_safe_mouth_crossover(
                graph, index, planner_guard_mm=0.0
            )
            minimum = float(path.minimum_prior_center_distance_mm)
            zero_guard = {
                "status": "DIAGNOSTIC_PATH_FOUND_NOT_AUTHORITY",
                "start_turn_index": index,
                "end_turn_index": index + 1,
                "planner_guard_mm": 0.0,
                "minimum_prior_center_distance_mm": minimum,
                "finished_wire_diameter_mm": graph.wire_diameter_mm,
                "clearance_above_one_wire_diameter_mm": (
                    minimum - graph.wire_diameter_mm
                ),
                "reason_not_authority": (
                    "removing the declared guard yields an exact tangent but "
                    "does not classify that contact as an intended physical "
                    "support or supply a 3D moving-route postcheck"
                ),
            }
        except Exception as exc:
            zero_guard = {
                "status": "NO_DIAGNOSTIC_PATH",
                "exception_type": type(exc).__name__,
                "reason": str(exc),
            }

    complete = len(rows) == EXPECTED_CROSSOVERS and first_failure is None
    sequential_count = 0
    sequential_error = None
    if complete:
        try:
            crossovers = tuple(
                solve_safe_mouth_crossover(graph, index)
                for index in range(EXPECTED_CROSSOVERS)
            )
            sequential_count = len(sequential_lay_samples(
                graph, DEFAULT_STATOR, SequentialRoutePolicy(), crossovers
            ))
        except Exception as exc:
            sequential_error = f"{type(exc).__name__}: {exc}"
    gates = {
        "all_49_default_guard_crossovers_solve": complete,
        "sequential_lay_samples_materialized": (
            complete and sequential_count > 0 and sequential_error is None
        ),
        "zero_guard_diagnostic_not_promoted": (
            zero_guard is None
            or zero_guard.get("status") != "AUTHORIZED"
        ),
    }
    return {
        "status": "PASS" if all(gates.values()) else "FAIL",
        "policy": SequentialRoutePolicy().__dict__,
        "required_crossover_count": EXPECTED_CROSSOVERS,
        "default_guard_pass_count_before_first_failure": len(rows),
        "first_default_guard_failure": first_failure,
        "passing_prefix": rows,
        "zero_guard_counterexample_diagnostic": zero_guard,
        "sequential_sample_count": sequential_count,
        "sequential_materialization_error": sequential_error,
        "gates": gates,
    }


def analyze(
    capture_path: Path = CAPTURE,
    plan_path: Path = PLAN,
    guide_report_path: Path = GUIDE_REPORT,
    guide_loci_path: Path = GUIDE_LOCI,
    aggregate_path: Path = AGGREGATE,
) -> dict[str, Any]:
    capture_path = Path(capture_path)
    plan_path = Path(plan_path)
    guide_report_path = Path(guide_report_path)
    guide_loci_path = Path(guide_loci_path)
    aggregate_path = Path(aggregate_path)
    required = {
        "raw_capture": capture_path,
        "slot_winding_plan": plan_path,
        "active_guide_report": guide_report_path,
        "active_guide_loci": guide_loci_path,
        "aggregate_authority": aggregate_path,
    }
    missing = [name for name, path in required.items() if not path.is_file()]
    if missing:
        report: dict[str, Any] = {
            "schema": SCHEMA,
            "status": "FAIL",
            "decision": "MOVING_HALF_TURN_CONDUCTOR_NOT_PROVEN",
            "production_authorized": False,
            "assembly_integration_authorized": False,
            "missing_inputs": missing,
            "input_files": {name: str(path) for name, path in required.items()},
            "source_hashes": {
                "sim/moving_half_turn_conductor_audit.py": _sha256(
                    Path(__file__)
                ),
                "sim/slot_route.py": _sha256(HERE / "slot_route.py"),
            },
        }
        report["report_sha256"] = _canonical_hash(report, "report_sha256")
        return report

    events = load_events(capture_path)
    timeline = Timeline(events)
    plan = _load(plan_path)
    guide_report = _load(guide_report_path)
    loci_document = _load(guide_loci_path)
    aggregate = _load(aggregate_path)
    graph = PackingSupportGraph.from_report(plan, spec=DEFAULT_STATOR)

    guide = _guide_evidence(
        guide_report, loci_document, aggregate, guide_loci_path
    )
    raw = _raw_timing_and_target_evidence(
        events, timeline, loci_document, graph
    )
    crossovers = _crossover_evidence(graph)
    rounded_radii = [float(turn.profile_radius_mm) for turn in graph.turns]
    rounded_loop = {
        "model": (
            "Shapely buffer of the tooth-neck/stack rectangle; corner "
            "wire-center curvature equals each placement profile radius"
        ),
        "minimum_corner_wire_center_radius_mm": min(rounded_radii),
        "maximum_corner_wire_center_radius_mm": max(rounded_radii),
        "required_free_or_named_contact_radius_mm": REQUIRED_BEND_RADIUS_MM,
        "all_50_placement_corner_radii_ge_3mm": all(
            radius >= REQUIRED_BEND_RADIUS_MM for radius in rounded_radii
        ),
        "sub_R3_placement_count": sum(
            radius < REQUIRED_BEND_RADIUS_MM for radius in rounded_radii
        ),
    }
    moving_postcheck = {
        "state_space_crossover_is_not_a_3D_wire_route": True,
        "physical_cap_endpoint_to_target_polyline_serialized": False,
        "every_moving_sample_exact_core_clearance_checked": False,
        "every_moving_sample_prior_copper_clearance_checked": False,
        "every_moving_sample_unintended_rigid_contact_checked": False,
        "every_tangent_contact_has_named_surface_owner": False,
        "status": "FAIL",
    }
    release_gates = {
        "canonical_raw_timing_and_axes_bound": all(
            raw["gates"][name] for name in (
                "all_24_raw_passes_present",
                "all_2400_half_turn_start_loci_present",
                "locus_times_match_raw_half_turn_starts",
                "locus_axes_match_raw_timeline",
                "raw_M2_phase_matches_half_turn_index",
            )
        ),
        "current_physical_active_guide_chain_authorized": (
            guide["status"] == "PASS"
        ),
        "all_raw_M0_targets_match_sequential_policy": raw["gates"][
            "every_raw_M0_pose_matches_sequential_radial_target"
        ],
        "all_49_sequential_crossovers_solve": crossovers["gates"][
            "all_49_default_guard_crossovers_solve"
        ],
        "guide_to_rounded_loop_continuation_explicit": raw["gates"][
            "physical_cap_endpoint_to_rounded_target_continuation_explicit"
        ],
        "all_free_and_named_contact_bends_ge_3mm": (
            guide["gates"]["all_named_guide_bend_radii_ge_3mm"]
            and rounded_loop["all_50_placement_corner_radii_ge_3mm"]
        ),
        "complete_moving_core_copper_rigid_contact_postcheck": (
            moving_postcheck["status"] == "PASS"
        ),
    }
    passed = all(release_gates.values())
    report = {
        "schema": SCHEMA,
        "status": "PASS" if passed else "FAIL",
        "decision": (
            "MOVING_HALF_TURN_CONDUCTOR_PROVEN"
            if passed else "MOVING_HALF_TURN_CONDUCTOR_NOT_PROVEN"
        ),
        "production_authorized": passed,
        "assembly_integration_authorized": passed,
        "authority_boundary": {
            "raw_timing_and_axes": "authoritative supplied capture",
            "sequential_policy": (
                "candidate radial/profile state family only until 3D route "
                "and contact postchecks pass"
            ),
            "active_guide": (
                "current physical route to side-specific cap lane endpoint"
            ),
            "rounded_loop_targets": (
                "candidate deposition targets; not a physical continuation "
                "from the side-specific cap lane endpoint"
            ),
            "sag_snag_friction_tension_dynamics": "not modeled",
        },
        "input_files": {
            name: str(path.relative_to(ROOT)).replace("\\", "/")
            for name, path in required.items()
        },
        "input_sha256": {name: _sha256(path) for name, path in required.items()},
        "job": {
            "slots": int(DEFAULT_STATOR.slots),
            "turns_per_tooth": len(graph.turns),
            "wire_finished_diameter_mm": graph.wire_diameter_mm,
            "center_core_access_mm": graph.center_core_access_mm,
            "raw_pass_count": EXPECTED_PASSES,
            "raw_half_turn_interval_count": EXPECTED_RAW_INTERVALS,
        },
        "raw_timing_and_target_binding": raw,
        "physical_active_guide_chain": guide,
        "sequential_crossover_model": crossovers,
        "rounded_loop_bend_model": rounded_loop,
        "moving_3D_postcheck_coverage": moving_postcheck,
        "coverage": {
            "required_raw_moving_half_turn_intervals": EXPECTED_RAW_INTERVALS,
            "raw_timing_bound_interval_count": raw[
                "raw_half_turn_interval_count"
            ],
            "locally_solved_default_guard_crossover_prefix_count": crossovers[
                "default_guard_pass_count_before_first_failure"
            ],
            "physically_authorized_moving_half_turn_interval_count": (
                EXPECTED_RAW_INTERVALS if passed else 0
            ),
        },
        "release_gates": release_gates,
        "minimum_evidence_to_close": [
            (
                "Generate a raw capture whose M0 at every half-turn follows "
                "the same 50-turn radial/profile schedule consumed here."
            ),
            (
                "Resolve the default guarded 21->22 crossover without "
                "removing clearance or reclassifying an unnamed tangent."
            ),
            (
                "Replace the sub-R3 rounded-loop corner construction with a "
                "physical R3-supported end-turn family."
            ),
            (
                "Serialize the cap-lane-endpoint-to-live-target conductor "
                "continuation with C0/C1 seams and named surface owners."
            ),
            (
                "Postcheck every moving 3D route against exact core, all "
                "prior/neighbor copper, and every unintended rigid part."
            ),
        ],
        "dependency_versions": dependency_versions(),
        "source_hashes": {
            "sim/moving_half_turn_conductor_audit.py": _sha256(Path(__file__)),
            "sim/slot_route.py": _sha256(HERE / "slot_route.py"),
            "sim/phase_aware_progressive_wire_audit.py": _sha256(
                HERE / "phase_aware_progressive_wire_audit.py"
            ),
            "sim/traj.py": _sha256(HERE / "traj.py"),
            "cad/params.py": _sha256(CAD / "params.py"),
            "cad/wire_geometry.py": _sha256(CAD / "wire_geometry.py"),
            "cad/permanent_cap_production_review.py": _sha256(
                CAD / "permanent_cap_production_review.py"
            ),
        },
    }
    report["report_sha256"] = _canonical_hash(report, "report_sha256")
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported moving-half-turn audit schema")
    if report.get("report_sha256") != _canonical_hash(
            report, "report_sha256"):
        raise ValueError("moving-half-turn audit hash mismatch")
    rows = _current_source_rows(
        report.get("source_hashes", {})
        if isinstance(report.get("source_hashes"), Mapping) else {}
    )
    stale = [row["path"] for row in rows if not row["current"]]
    if stale:
        raise ValueError(
            "moving-half-turn audit has stale sources: " + ", ".join(stale)
        )
    input_files = report.get("input_files", {})
    input_hashes = report.get("input_sha256", {})
    if not isinstance(input_files, Mapping) or not isinstance(
            input_hashes, Mapping):
        raise ValueError("moving-half-turn audit input bindings are absent")
    stale_inputs = []
    for name, relative in input_files.items():
        path = ROOT / str(relative).replace("/", "\\")
        actual = _sha256(path) if path.is_file() else None
        if input_hashes.get(name) != actual:
            stale_inputs.append(str(name))
    if stale_inputs:
        raise ValueError(
            "moving-half-turn audit has stale inputs: "
            + ", ".join(stale_inputs)
        )
    expected_status = (
        "PASS" if all(report.get("release_gates", {}).values()) else "FAIL"
    )
    if report.get("status") != expected_status:
        raise ValueError("moving-half-turn status/gate mismatch")
    if report.get("production_authorized") is not (
            expected_status == "PASS"):
        raise ValueError("moving-half-turn production authority drifted")


def render_markdown(report: Mapping[str, Any]) -> str:
    raw = report.get("raw_timing_and_target_binding", {})
    guide = report.get("physical_active_guide_chain", {})
    cross = report.get("sequential_crossover_model", {})
    bend = report.get("rounded_loop_bend_model", {})
    coverage = report.get("coverage", {})
    failure = cross.get("first_default_guard_failure") or {}
    lines = [
        "# Moving half-turn conductor audit",
        "",
        f"**Status: {report.get('status', 'FAIL')}**  ",
        f"Decision: `{report.get('decision', 'UNKNOWN')}`  ",
        f"Production authorized: `{str(bool(report.get('production_authorized'))).lower()}`",
        "",
        "This audit binds the existing sequential slot-route model to the raw "
        "half-turn clocks and the current physical active-sector guide. A "
        "radial/profile state-space path is not treated as physical wire.",
        "",
        "## Coverage",
        "",
        f"- Raw half-turn intervals bound: {coverage.get('raw_timing_bound_interval_count', 0)} / {coverage.get('required_raw_moving_half_turn_intervals', EXPECTED_RAW_INTERVALS)}",
        f"- Physically authorized moving intervals: **{coverage.get('physically_authorized_moving_half_turn_interval_count', 0)}**",
        f"- Default-guard crossover prefix solved: {cross.get('default_guard_pass_count_before_first_failure', 0)} / {cross.get('required_crossover_count', EXPECTED_CROSSOVERS)}",
        "",
        "## Concrete counterexamples",
        "",
        f"- Raw M0 target matches: **{raw.get('raw_M0_exact_sequential_target_count', 0)} / {raw.get('raw_half_turn_interval_count', 0)}**; maximum error `{raw.get('maximum_absolute_raw_M0_target_error_mm')} mm`.",
        f"- First guarded crossover failure: `{failure.get('start_turn_index')}->{failure.get('end_turn_index')}` — {failure.get('reason', 'none')}.",
        f"- Rounded-loop minimum corner radius: `{bend.get('minimum_corner_wire_center_radius_mm')} mm`; required `{bend.get('required_free_or_named_contact_radius_mm', REQUIRED_BEND_RADIUS_MM)} mm`.",
        f"- Physical active-guide chain: `{guide.get('status', 'FAIL')}` with minimum named-guide radius `{guide.get('minimum_named_wire_center_radius_mm')} mm`.",
        f"- Exact short-leadin/cap-lane seams: `{guide.get('exact_short_leadin_to_cap_lane_seam', {}).get('connected_locus_count', 0)} / {guide.get('exact_short_leadin_to_cap_lane_seam', {}).get('evaluated_locus_count', 0)}`; maximum gap `{guide.get('exact_short_leadin_to_cap_lane_seam', {}).get('maximum_exact_distance_mm')} mm`.",
        f"- Unmodeled cap-endpoint/rounded-target gap range: `{raw.get('minimum_unmodeled_cap_endpoint_to_rounded_target_gap_mm')}..{raw.get('maximum_unmodeled_cap_endpoint_to_rounded_target_gap_mm')} mm`.",
        "",
        "## Release gates",
        "",
    ]
    lines.extend(
        f"- {'PASS' if passed else 'FAIL'} - `{name}`"
        for name, passed in report.get("release_gates", {}).items()
    )
    lines.extend(["", "## Minimum evidence to close", ""])
    lines.extend(
        f"{index}. {item}"
        for index, item in enumerate(
            report.get("minimum_evidence_to_close", []), 1
        )
    )
    lines.extend([
        "",
        f"Report SHA-256: `{report.get('report_sha256', '')}`",
        "",
    ])
    return "\n".join(lines)


def write_outputs(
    report: Mapping[str, Any],
    json_path: Path = OUTPUT_JSON,
    markdown_path: Path = OUTPUT_MD,
) -> None:
    json_path = Path(json_path)
    markdown_path = Path(markdown_path)
    json_path.parent.mkdir(parents=True, exist_ok=True)
    markdown_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    markdown_path.write_text(
        render_markdown(report), encoding="utf-8", newline="\n"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", type=Path, default=CAPTURE)
    parser.add_argument("--plan", type=Path, default=PLAN)
    parser.add_argument("--guide-report", type=Path, default=GUIDE_REPORT)
    parser.add_argument("--guide-loci", type=Path, default=GUIDE_LOCI)
    parser.add_argument("--aggregate", type=Path, default=AGGREGATE)
    parser.add_argument("--json", type=Path, default=OUTPUT_JSON)
    parser.add_argument("--markdown", type=Path, default=OUTPUT_MD)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    report = analyze(
        args.capture, args.plan, args.guide_report, args.guide_loci,
        args.aggregate,
    )
    if not args.check:
        write_outputs(report, args.json, args.markdown)
        print(f"wrote {args.json}")
        print(f"wrote {args.markdown}")
    print(
        f"moving half-turn conductor {report['status']}: "
        f"raw={report.get('coverage', {}).get('raw_timing_bound_interval_count', 0)}; "
        f"authorized={report.get('coverage', {}).get('physically_authorized_moving_half_turn_interval_count', 0)}; "
        f"crossovers={report.get('sequential_crossover_model', {}).get('default_guard_pass_count_before_first_failure', 0)}/{EXPECTED_CROSSOVERS}"
    )
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
