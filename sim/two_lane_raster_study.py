"""Fail-closed raw-compatible two-branch raster feasibility study.

This study tests a deliberately narrow mechanism contract: the lay point is
a single-valued function of ``(sign(dM0), M0)``.  Outbound M0 selects one
25-point branch and inbound M0 selects the other.  No turn counter, flyer
phase input, elastic wire overlap, or software change is credited.

The exact current 50-center packing is also partitioned into two continuous
25-point branches.  That is useful successor geometry, but it cannot rescue
a non-injective raw input and it inherits the current route's 3 mm curvature
failure.  The report therefore distinguishes static packing feasibility from
release feasibility.
"""

from __future__ import annotations

from functools import lru_cache
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np
from shapely.geometry import LineString


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
PACKING = REPORTS / "slot_packing.json"
ELASTIC = REPORTS / "elastic_wire_contact_study.json"
OUTPUT_JSON = REPORTS / "two_lane_raster_study.json"
OUTPUT_MD = REPORTS / "two_lane_raster_study.md"

if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

from params import DEFAULT_STATOR  # noqa: E402
import slot_packing_audit  # noqa: E402
import elastic_wire_contact_study as elastic  # noqa: E402
from slot_route import PackingSupportGraph  # noqa: E402
from traj import Timeline  # noqa: E402


SCHEMA = "two-lane-raster-study/v1"
EXPECTED_CAPTURE_SHA256 = (
    "350210c36550ae2c22e5352675aee6bd2eebb18325f3b5f45c7cb35d6a314958"
)
WIRE_D_MM = 0.22352
LINER_T_MM = 0.127
TURNS = 50
TURNS_PER_BRANCH = 25
MIN_BEND_RADIUS_MM = 3.0
EPS = 1.0e-9


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: dict[str, Any]) -> str:
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _validate_hashed_report(report: dict[str, Any]) -> None:
    payload = dict(report)
    expected = payload.pop("report_sha256", None)
    if not isinstance(expected, str) or _canonical_hash(payload) != expected:
        raise ValueError("input report hash mismatch")


def _line_interval(domain: Any, tangential_mm: float) -> tuple[float, float]:
    cut = domain.intersection(LineString((
        (0.0, float(tangential_mm)),
        (2.0 * float(DEFAULT_STATOR.od), float(tangential_mm)),
    )))
    if cut.is_empty:
        return (0.0, 0.0)
    return (float(cut.length), float(cut.bounds[2] - cut.bounds[0]))


def straight_lane_capacity(wire_d_mm: float, liner_t_mm: float) -> dict[str, Any]:
    """Upper-bound two constant-tangential-row capacity.

    The mirrored neighbor requires the near-row center to be at least one
    wire radius from the slot bisector.  A distinct far row must then be at
    least one full wire diameter farther away.  Slot width decreases
    monotonically in that direction, so this is the most favorable possible
    far-row ordinate.
    """

    job = slot_packing_audit.PackingInput(wire_d_mm, liner_t_mm)
    domain = slot_packing_audit._positive_slot_center_domain(job)
    near_v = job.wire_radius_mm
    far_v = near_v + job.wire_d_mm
    near_length, near_span = _line_interval(domain, near_v)
    far_length, far_span = _line_interval(domain, far_v)

    def capacity(length: float) -> int:
        return int(math.floor((length + 1e-12) / wire_d_mm)) + 1

    near_capacity = capacity(near_length)
    far_capacity = capacity(far_length)
    return {
        "status": "PASS" if min(near_capacity, far_capacity) >= 25 else "FAIL",
        "wire_finished_diameter_mm": wire_d_mm,
        "liner_thickness_mm": liner_t_mm,
        "near_lane_tangential_mm": near_v,
        "far_lane_most_favorable_tangential_mm": far_v,
        "near_lane_available_length_mm": near_length,
        "far_lane_available_length_mm": far_length,
        "near_lane_radial_span_mm": near_span,
        "far_lane_radial_span_mm": far_span,
        "near_lane_maximum_center_count": near_capacity,
        "far_lane_maximum_center_count": far_capacity,
        "required_centers_per_lane": 25,
        "method": (
            "exact intersection of the generated liner-offset slot-center "
            "polygon with constant-tangential lines"
        ),
    }


def _branch_metrics(points: np.ndarray) -> dict[str, Any]:
    path = LineString(points)
    steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    return {
        "center_count": int(len(points)),
        "simple": bool(path.is_simple),
        "length_mm": float(path.length),
        "minimum_step_mm": float(np.min(steps)),
        "maximum_step_mm": float(np.max(steps)),
        "radial_range_mm": [float(np.min(points[:, 0])),
                            float(np.max(points[:, 0]))],
        "tangential_range_mm": [float(np.min(points[:, 1])),
                                float(np.max(points[:, 1]))],
        "centers_slot_frame_uv_mm": points.tolist(),
    }


def variable_branch_partition(packing: dict[str, Any]) -> dict[str, Any]:
    """Partition the current exact Hamiltonian centers into 25 + 25."""

    selected = packing["selected_schedule"]
    rows = selected["side_positive"]
    mirror = selected["side_negative"]
    if len(rows) != 50 or len(mirror) != 50:
        raise ValueError("packing must contain 50 centers on each slot side")
    points = np.asarray([row["slot_frame_uv_mm"] for row in rows], dtype=float)
    outbound = points[:25]
    inbound = points[25:]
    out_line = LineString(outbound)
    in_line = LineString(inbound)
    intersection = out_line.intersection(in_line)
    paired = outbound - inbound[::-1]
    paired_norm = np.linalg.norm(paired, axis=1)
    connector = float(np.linalg.norm(inbound[0] - outbound[-1]))

    job = slot_packing_audit.PackingInput(
        float(packing["config"]["wire_finished_diameter_mm"]),
        float(packing["config"]["liner_thickness_mm"]),
    )
    mouth = slot_packing_audit._sequential_mouth_audit(job, rows, mirror)
    validation = packing["validation"]
    checks = {
        "25_centers_per_branch": len(outbound) == len(inbound) == 25,
        "both_branch_centerlines_simple": out_line.is_simple and in_line.is_simple,
        "branch_centerlines_disjoint": intersection.is_empty,
        "reversal_connector_one_wire_pitch": abs(connector - job.wire_d_mm) <= EPS,
        "full_100_center_pair_nonoverlap": (
            float(validation["minimum_pair_center_distance_mm"])
            + EPS >= job.wire_d_mm
        ),
        "exact_core_and_liner_access": bool(validation["core_access_ok"]),
        "all_empty_neighbor_histories_mouth_connected": bool(
            mouth["all_empty_neighbor_side_connected"]),
        "full_neighbor_history_mouth_connected": bool(
            mouth["all_prefilled_neighbor_side_connected"]),
    }
    status = "PASS" if all(checks.values()) else "FAIL"
    return {
        "status": status,
        "interpretation": (
            "Static successor geometry only; it is not a raw-compatible cam "
            "or an R3 moving-wire certificate."
        ),
        "partition_rule": "packing turns 0..24 outbound; 25..49 inbound",
        "outbound_branch": _branch_metrics(outbound),
        "inbound_branch": _branch_metrics(inbound),
        "branch_intersection": intersection.wkt,
        "reversal_connector_mm": connector,
        "selector_profile_separation_at_equal_m0": {
            "pairing": (
                "outbound index i against inbound index 24-i because the "
                "raw M0 coordinate reverses direction"
            ),
            "minimum_vector_travel_mm": float(np.min(paired_norm)),
            "maximum_vector_travel_mm": float(np.max(paired_norm)),
            "maximum_required_radial_output_difference_mm": float(
                np.max(np.abs(paired[:, 0]))),
            "maximum_required_tangential_output_difference_mm": float(
                np.max(np.abs(paired[:, 1]))),
            "travel_at_direction_reversal_mm": connector,
        },
        "packing_clearance": {
            "minimum_center_distance_mm": float(
                validation["minimum_pair_center_distance_mm"]),
            "pair_margin_mm": float(
                validation["minimum_pair_center_distance_mm"] - job.wire_d_mm),
            "minimum_center_core_distance_mm": float(
                validation["minimum_center_core_distance_mm"]),
            "required_center_core_distance_mm": (
                job.wire_radius_mm + job.liner_t_mm),
            "core_margin_mm": float(
                validation["minimum_center_core_distance_mm"]
                - job.wire_radius_mm - job.liner_t_mm),
            "radial_outer_margin_mm": float(
                validation["radial_outer_margin_mm"]),
        },
        "sequential_mouth_access": mouth,
        "checks": checks,
    }


def raw_single_value_conflicts(
    states: list[dict[str, Any]], radial_error_bound_mm: float,
) -> dict[str, Any]:
    """Find same-branch, same-side equal-M0 states in each raw pass."""

    witnesses = []
    for pass_index in range(24):
        lane = sorted((
            row for row in states
            if int(row["pass_index"]) == pass_index
            and int(row["half_turn_index"]) == 0
            and int(row["turn_index"]) < 25
        ), key=lambda row: int(row["turn_index"]))
        for left, right in zip(lane, lane[1:]):
            delta = abs(float(right["m0_position_rad"])
                        - float(left["m0_position_rad"]))
            if delta <= 1e-12:
                witnesses.append({
                    "pass_index": pass_index,
                    "tooth_index": int(left["tooth_index"]),
                    "motion_sign": int(left["motion_sign"]),
                    "half_turn_index": 0,
                    "turn_indices": [int(left["turn_index"]),
                                     int(right["turn_index"])],
                    "m0_position_rad": float(left["m0_position_rad"]),
                    "m0_input_delta_rad": delta,
                    "raw_radial_contact_delta_mm": abs(
                        float(right["radial_contact_mm"])
                        - float(left["radial_contact_mm"])),
                    "minimum_required_distinct_center_separation_mm": WIRE_D_MM,
                })
    checks = {
        "all_24_passes_examined": len({
            int(row["pass_index"]) for row in states
        }) == 24,
        "both_m2_signs_present": {
            int(row["motion_sign"]) for row in states
        } == {-1, 1},
        "no_same_input_distinct_turn_conflicts": not witnesses,
    }
    return {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "mechanism_contract": (
            "lay_point = F(direction_of_M0, M0); therefore equal M0 on "
            "the same direction branch must produce one equal lay point"
        ),
        "conflict_count": len(witnesses),
        "affected_pass_count": len({row["pass_index"] for row in witnesses}),
        "minimum_resulting_center_distance_mm": 0.0 if witnesses else None,
        "required_finished_wire_center_distance_mm": WIRE_D_MM,
        "capture_radial_quantization_error_bound_mm": radial_error_bound_mm,
        "witnesses": witnesses,
        "checks": checks,
    }


@lru_cache(maxsize=1)
def analyze() -> dict[str, Any]:
    if _sha256(CAPTURE) != EXPECTED_CAPTURE_SHA256:
        raise ValueError("canonical raw capture SHA-256 drifted")
    packing = json.loads(PACKING.read_text(encoding="utf-8"))
    graph = PackingSupportGraph.from_report(packing, spec=DEFAULT_STATOR)
    elastic_report = json.loads(ELASTIC.read_text(encoding="utf-8"))
    _validate_hashed_report(elastic_report)

    events = elastic._load_jsonl(CAPTURE)
    capture_contract = elastic.validate_capture_contract(events, CAPTURE)
    timeline = Timeline(events)
    replay = elastic.replay_raw_winding_states(events, timeline, graph)
    raw_conflicts = raw_single_value_conflicts(
        replay["states"], float(replay["radial_position_error_bound_mm"]))

    straight_nominal = straight_lane_capacity(WIRE_D_MM, LINER_T_MM)
    straight_receiving_max = straight_lane_capacity(0.235, 0.140)
    partition = variable_branch_partition(packing)
    contact = elastic_report["elastic_contact_reanalysis"]
    route = {
        "status": "PASS" if (
            int(contact["contact_geometric_pass_count"]) == 2
            and int(contact["elastic_curvature_pass_count"]) == 2
        ) else "FAIL",
        "same_center_set_and_order_as_partition": True,
        "stored_100_case_geometry_coverage": bool(
            elastic_report["release_flags"][
                "all_100_rigid_or_contact_routes_geometrically_nonpenetrating"]),
        "both_raw_m2_signs_present": bool(
            replay["both_motion_signs_covered"]),
        "rigid_failure_cases": int(contact["rigid_failure_case_count"]),
        "contact_geometric_pass_cases": int(
            contact["contact_geometric_pass_count"]),
        "contact_cases_meeting_3mm_bend": int(
            contact["elastic_curvature_pass_count"]),
        "minimum_required_bend_radius_mm": MIN_BEND_RADIUS_MM,
        "minimum_proved_contact_bend_radius_mm": min(
            float(row["analytic_local_bend_radius_mm"])
            for row in contact["cases"]),
        "minimum_contact_route_core_center_distance_mm": min(
            float(row["minimum_core_center_distance_mm"])
            for row in contact["cases"]),
        "minimum_contact_route_nonparent_copper_lower_bound_mm": min(
            float(row["minimum_nonparent_copper_lower_bound_mm"])
            for row in contact["cases"]),
        "all_contact_routes_simple": all(
            bool(row["local_contact_path_simple"])
            for row in contact["cases"]),
        "all_neighbor_histories_and_both_signs_release_proved": False,
        "reason": (
            "Two turn-45 half-routes are nonpenetrating and topologically "
            "simple only by following a 0.22352 mm-radius parent-wire arc; "
            "that is below the required 3 mm, and the stored sign-specific "
            "current-half history gate is not a release proof."
        ),
        "input_report_sha256": elastic_report["report_sha256"],
    }

    error_budget = {
        "status": "FAIL",
        "wire_and_liner_receiving_policy": (
            "measure and regenerate geometry for 0.220..0.235 mm finished "
            "wire and 0.120..0.140 mm liner"
        ),
        "nominal_pair_margin_mm": partition["packing_clearance"][
            "pair_margin_mm"],
        "nominal_core_margin_mm": partition["packing_clearance"][
            "core_margin_mm"],
        "raw_capture_radial_quantization_bound_mm": float(
            replay["radial_position_error_bound_mm"]),
        "allowable_unmodeled_independent_cam_error_mm": 0.0,
        "reason": (
            "The constructive centers intentionally use tangent copper and "
            "tangent liner support, leaving no positive independent placement "
            "error allowance. Receiving regeneration is not a cam-following "
            "tolerance budget."
        ),
    }

    release_flags = {
        "canonical_unmodified_raw_capture_bound": True,
        "constant_tangential_two_lane_capacity_25_each": (
            straight_nominal["status"] == "PASS"),
        "variable_25_plus_25_static_partition": partition["status"] == "PASS",
        "single_valued_direction_m0_mapping_injective": (
            raw_conflicts["status"] == "PASS"),
        "sequential_static_mouth_access_with_full_neighbor": bool(
            partition["checks"]["full_neighbor_history_mouth_connected"]),
        "complete_R3_route_both_signs_all_histories": route["status"] == "PASS",
        "minimum_3mm_bend_radius": (
            route["minimum_proved_contact_bend_radius_mm"]
            + EPS >= MIN_BEND_RADIUS_MM),
        "positive_physical_error_budget": error_budget["status"] == "PASS",
    }
    status = "PASS" if all(release_flags.values()) else "FAIL"
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": status,
        "decision": "REJECT_DIRECTION_ONLY_TWO_LANE_RASTER",
        "job": {
            "stator_od_mm": 46.0,
            "stack_mm": 15.0,
            "slots": 24,
            "turns_per_tooth": 50,
            "wire_finished_diameter_mm": WIRE_D_MM,
            "liner_thickness_mm": LINER_T_MM,
        },
        "capture": capture_contract,
        "straight_constant_tangential_lanes": {
            "nominal": straight_nominal,
            "receiving_max": straight_receiving_max,
        },
        "variable_tangential_partition": partition,
        "raw_single_value_mapping": raw_conflicts,
        "R3_sequential_route": route,
        "tolerance_error_budget": error_budget,
        "release_flags": release_flags,
        "release_blockers": [
            name for name, passed in release_flags.items() if not passed
        ],
        "next_feasible_mechanism": {
            "minimum_added_state": (
                "one flyer-phase or turn-count state in addition to M0 "
                "direction and position; the first two same-side outbound "
                "turns have exactly identical raw M0"
            ),
            "candidate": (
                "M2-synchronized positive lay guide or indexed 50-step "
                "ratchet/cam with a >=3 mm-radius dielectric former"
            ),
            "required_outputs_from_static_successor": {
                "maximum_radial_profile_difference_mm": partition[
                    "selector_profile_separation_at_equal_m0"
                ]["maximum_required_radial_output_difference_mm"],
                "maximum_tangential_profile_difference_mm": partition[
                    "selector_profile_separation_at_equal_m0"
                ]["maximum_required_tangential_output_difference_mm"],
                "direction_reversal_transition_mm": partition[
                    "selector_profile_separation_at_equal_m0"
                ]["travel_at_direction_reversal_mm"],
                "minimum_dielectric_former_bend_radius_mm": 3.0,
            },
            "integration_status": "SUCCESSOR_STUDY_ONLY",
        },
        "source_hashes": {
            "sim/two_lane_raster_study.py": _sha256(Path(__file__)),
            "out/capture/upstream_current_raw.jsonl": _sha256(CAPTURE),
            "out/reports/slot_packing.json": _sha256(PACKING),
            "out/reports/elastic_wire_contact_study.json": _sha256(ELASTIC),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def render_markdown(report: dict[str, Any]) -> str:
    nominal = report["straight_constant_tangential_lanes"]["nominal"]
    part = report["variable_tangential_partition"]
    raw = report["raw_single_value_mapping"]
    route = report["R3_sequential_route"]
    selector = part["selector_profile_separation_at_equal_m0"]
    return f"""# Raw-compatible two-lane raster study

**Overall: {report['status']}**  
**Decision: {report['decision']}**

The direction-only mechanism is rejected. Every one of the 24 raw winding
passes commands the first two same-side outbound turns at exactly
`M0 = -61.918 rad`. A single-valued function of `(direction, M0)` must send
both turns to the same point, giving 0 mm center separation for
{report['job']['wire_finished_diameter_mm']:.5f} mm finished wire.

## Geometry results

- Straight constant-t rows: far row holds at most
  {nominal['far_lane_maximum_center_count']} centers, not 25.
- Variable 25 + 25 partition: {part['status']} as static successor geometry.
- Branches are simple and disjoint: {part['checks']['both_branch_centerlines_simple']} /
  {part['checks']['branch_centerlines_disjoint']}.
- Full-neighbor sequential mouth connectivity: {part['checks']['full_neighbor_history_mouth_connected']}.
- Raw equal-input conflicts: {raw['conflict_count']} across
  {raw['affected_pass_count']} passes.
- Selector profile envelope: radial {selector['maximum_required_radial_output_difference_mm']:.6f} mm,
  tangential {selector['maximum_required_tangential_output_difference_mm']:.6f} mm;
  reversal transition {selector['travel_at_direction_reversal_mm']:.6f} mm.

## R3 and tolerance gates

- Nonpenetrating rigid/contact geometry coverage: {route['stored_100_case_geometry_coverage']}.
- Smallest proved contact bend radius: {route['minimum_proved_contact_bend_radius_mm']:.5f} mm
  versus 3.0 mm required.
- Complete both-sign/all-history R3 proof: {route['all_neighbor_histories_and_both_signs_release_proved']}.
- Nominal pair and core margins are tangent (effectively 0 mm), so there is
  no positive cam/selector following-error budget.

## Required successor

Add flyer phase or a turn counter: an M2-synchronized positive guide or
indexed 50-step ratchet/cam, plus a dielectric former whose supported bends
are at least 3 mm radius. The variable two-branch center table may seed that
successor, but is not production-authorized.

Report SHA-256: `{report['report_sha256']}`
"""


def write_outputs(report: dict[str, Any], json_path: Path,
                  markdown_path: Path) -> None:
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    markdown_path.write_text(render_markdown(report), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", type=Path, default=OUTPUT_JSON)
    parser.add_argument("--markdown", type=Path, default=OUTPUT_MD)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    report = analyze()
    if not args.check:
        write_outputs(report, args.json, args.markdown)
        print(f"wrote {args.json} and {args.markdown}")
    print(
        f"two-lane raster {report['status']}: "
        f"{report['raw_single_value_mapping']['conflict_count']} raw "
        "same-input conflicts"
    )
    if report["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
