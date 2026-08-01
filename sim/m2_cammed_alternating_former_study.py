"""Fail-closed study of an M2-cammed alternating-former mechanism.

This is an isolated architecture trade.  It does not modify the production
assembly or controller.  The proposed mechanism is deliberately narrow:

* one stationary, machine-fixed cam ring is coaxial with M2;
* spring-return polished fingers provide four mutually exclusive R3 support
  states at the two axial ends and two tangential sides of the active tooth;
* the cam is allowed to be direction sensitive, but has no pass counter,
  controller signal, or additional commanded axis; and
* M0 must retract the fingers before M1 indexing and shaft wrapping.

The raw upstream captured cycle does not present one reusable M2 phase law.
Across its 24 winding passes it contains three distinct (physical crossing
origin, direction) laws.  Worse, two negative-direction passes require opposite
axial/tangential fingers at the same physical M2 angle.  Directional clutches
therefore cannot rescue one stationary cam profile.

Intentional sliding contact on Nomex and previously deposited copper is
physical and is not rejected here.  The route gate is steel penetration,
topology, curvature/strain, and a buildable positively selected mechanism.
The current exact route report is included to bind the 100 rigid route
geometries and their separate report-level proof blockers; it is not used as a
frozen-clearance substitute for a former-supported path.

Outputs:
  out/reports/m2_cammed_alternating_former.json
  out/reports/m2_cammed_alternating_former.md
"""

from __future__ import annotations

from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np
from build123d import Compound, Cylinder, Pos


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
PLAN = REPORTS / "slot_winding_plan.json"
ROUTES = REPORTS / "slot_wire_routes.json"
PERMANENT_CAP = REPORTS / "stator_winding_guide_cap.json"

for path in (str(CAD), str(HERE)):
    if path not in sys.path:
        sys.path.insert(0, path)

import assembly  # noqa: E402
import hardware_placements  # noqa: E402
from params import DEFAULT_STATOR, PARAMS  # noqa: E402
import slot_route  # noqa: E402


SCHEMA = "m2-cammed-alternating-former-study/v1"
MINIMUM_WIRE_CENTER_BEND_RADIUS_MM = 3.0
CAM_RING_INNER_RADIUS_MM = 17.0
CAM_RING_OUTER_RADIUS_MM = 45.0
CAM_RING_Z_CENTER_MM = -32.0
CAM_RING_AXIAL_THICKNESS_MM = 2.0
RIGID_CLEARANCE_TARGET_MM = 2.0

# These are interior points of the four rounded end transitions, not guessed
# engagement-window endpoints.  On every one of the 50 exact packed loops,
# 15/30 degrees lie on opposite tangential halves of the +Z end and 195/210
# degrees lie on opposite tangential halves of the -Z end.
FORMER_DEMANDS = {
    15: "axial_positive_tangential_negative",
    30: "axial_positive_tangential_positive",
    195: "axial_negative_tangential_positive",
    210: "axial_negative_tangential_negative",
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _canonical_hash(value: dict[str, Any]) -> str:
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode()).hexdigest()


def _snap_deg(value_rad: float) -> int:
    degrees = math.degrees(float(value_rad)) % 360.0
    nearest = int(round(degrees)) % 360
    if abs(((degrees - nearest + 180.0) % 360.0) - 180.0) > 1e-5:
        raise ValueError(f"captured phase {degrees:.9f} deg is not integral")
    return nearest


def _capture_rows() -> list[dict[str, Any]]:
    return [json.loads(line) for line in CAPTURE.read_text(
        encoding="utf-8").splitlines() if line.strip()]


def capture_contract() -> dict[str, Any]:
    rows = _capture_rows()
    if not rows or rows[0].get("e") != "meta":
        raise ValueError("capture does not begin with a meta record")
    meta = rows[0]
    if int(meta.get("capture_schema", -1)) != 4:
        raise ValueError("M2 former study requires capture schema 4")
    if meta.get("controller_mode") != "upstream":
        raise ValueError("cam timing must come from a raw upstream capture")
    if meta.get("controller_adapter_sha256") is not None:
        raise ValueError("raw upstream capture unexpectedly names an adapter")
    if int(meta.get("turns", -1)) != 50:
        raise ValueError("M2 former study is bound to the 50-turn job")

    positions = {0: 0.0, 1: 0.0, 2: 0.0}
    passes: list[dict[str, Any]] = []
    index = 0
    collision_offset = float(meta["angle_to_prevent_collision"])
    while index < len(rows):
        row = rows[index]
        if row.get("e") == "cmd" and row.get("m") in positions:
            positions[int(row["m"])] = float(row["model_target"])
        if row.get("e") != "wind_wire":
            index += 1
            continue

        end_index = next(
            cursor for cursor in range(index + 1, len(rows))
            if rows[cursor].get("e") == "wind_wire_done"
        )
        local_positions = dict(positions)
        set_index = None
        prevent_index = None
        start_state = None
        physical_start = None
        m1_at_winding = None
        for cursor in range(index + 1, end_index + 1):
            event = rows[cursor]
            if event.get("e") == "cmd" and event.get("m") in local_positions:
                local_positions[int(event["m"])] = float(event["model_target"])
            if event.get("e") == "set_motor2_wire_position_done":
                set_index = cursor
                start_state = str(event["m2state"])
                physical_start = float(local_positions[2])
                m1_at_winding = float(local_positions[1])
            if event.get("e") == "prevent_collision" and prevent_index is None:
                prevent_index = cursor
        if set_index is None or physical_start is None or m1_at_winding is None:
            raise ValueError("raw pass has no set_motor2_wire_position seam")
        winding_end = prevent_index if prevent_index is not None else end_index
        m2_commands = [
            event for event in rows[set_index + 1:winding_end]
            if event.get("e") == "cmd" and event.get("m") == 2
        ]
        m0_commands = [
            event for event in rows[set_index + 1:winding_end]
            if event.get("e") == "cmd" and event.get("m") == 0
        ]
        if len(m2_commands) != 13:
            raise ValueError("raw upstream pass lost its 12-step plus target law")
        first_target = float(m2_commands[0]["model_target"])
        direction = 1 if first_target > physical_start else -1
        expected_direction = 1 if bool(row["args"][1]) else -1
        if direction != expected_direction:
            raise ValueError("raw M2 command sign disagrees with wind_wire")
        if start_state == "TOP_RIGHT":
            physical_crossing_origin = physical_start - collision_offset
        elif start_state == "BOTTOM_RIGHT":
            physical_crossing_origin = physical_start + collision_offset
        elif start_state in ("TOP", "BOTTOM"):
            physical_crossing_origin = physical_start
        else:
            raise ValueError(f"unsupported upstream M2 start state {start_state}")
        target = float(m2_commands[-1]["model_target"])
        actual_travel = abs(target - physical_crossing_origin)
        if actual_travel + 1e-9 < 50.0 * 2.0 * math.pi:
            raise ValueError("raw upstream target contains fewer than 50 turns")

        origin_deg = _snap_deg(physical_crossing_origin)
        m1_index_deg = _snap_deg(m1_at_winding)
        law_id = f"origin_{origin_deg:03d}_direction_{direction:+d}"
        passes.append({
            "pass_index": len(passes),
            "teeth_index": int(row["args"][0]),
            "wind_index": int(row["args"][2]),
            "direction": direction,
            "physical_crossing_origin_deg": origin_deg,
            "m1_index_deg": m1_index_deg,
            "physical_start_deg": math.degrees(physical_start % (2.0 * math.pi)),
            "upstream_motor2_state": start_state,
            "actual_travel_rad": actual_travel,
            "actual_target_revolutions": actual_travel / (2.0 * math.pi),
            "M2_winding_command_count": len(m2_commands),
            "captured_M0_winding_command_count": len(m0_commands),
            "law_id": law_id,
        })
        positions = local_positions
        index = end_index + 1

    if len(passes) != 24 or [row["pass_index"] for row in passes] != list(range(24)):
        raise ValueError("capture does not contain 24 ordered winding passes")
    if {row["direction"] for row in passes} != {-1, 1}:
        raise ValueError("capture does not contain both M2 directions")

    laws: dict[str, dict[str, Any]] = {}
    for row in passes:
        law = laws.setdefault(row["law_id"], {
            "physical_crossing_origin_deg": row[
                "physical_crossing_origin_deg"],
            "direction": row["direction"],
            "pass_indices": [],
            "m1_index_positions_deg": [],
        })
        law["pass_indices"].append(row["pass_index"])
        law["m1_index_positions_deg"].append(row["m1_index_deg"])

    m1_to_law: dict[int, str] = {}
    for row in passes:
        existing = m1_to_law.setdefault(row["m1_index_deg"], row["law_id"])
        if existing != row["law_id"]:
            raise ValueError("one physical M1 index maps to multiple cam laws")
    if len(m1_to_law) != 24:
        raise ValueError("physical M1 index is not unique for all 24 passes")

    return {
        "capture_schema": int(meta["capture_schema"]),
        "controller_mode": str(meta["controller_mode"]),
        "winder_commit": str(meta["winder_commit"]),
        "axis_velocities_rad_s": list(map(float, meta["velocities"])),
        "turns_per_tooth": int(meta["turns"]),
        "winding_pass_count": len(passes),
        "packing_waypoint_events_present": False,
        "packing_plan_used_as_motion_authority": False,
        "M2_winding_commands_per_pass": 13,
        "total_M2_winding_commands": sum(
            row["M2_winding_command_count"] for row in passes),
        "captured_M0_winding_command_count_range": [
            min(row["captured_M0_winding_command_count"] for row in passes),
            max(row["captured_M0_winding_command_count"] for row in passes),
        ],
        "directions": sorted({row["direction"] for row in passes}),
        "physical_crossing_origin_classes_deg": sorted({
            row["physical_crossing_origin_deg"] for row in passes
        }),
        "unique_cam_law_count": len(laws),
        "cam_laws": [dict(law_id=key, **value)
                      for key, value in sorted(laws.items())],
        "passes": passes,
        "m1_index_uniquely_selects_cam_law": True,
        "m1_index_to_law": {
            str(key): value for key, value in sorted(m1_to_law.items())
        },
        "status": "PASS",
    }


def _graph() -> slot_route.PackingSupportGraph:
    report = _load(PLAN)
    graph = slot_route.PackingSupportGraph.from_report(
        report, spec=DEFAULT_STATOR)
    if len(graph.turns) != 50:
        raise ValueError("winding plan does not contain 50 turns")
    if not math.isclose(
        graph.wire_diameter_mm, float(DEFAULT_STATOR.wire_d),
        rel_tol=0.0, abs_tol=1e-12,
    ):
        raise ValueError("winding plan wire diameter drift")
    return graph


def support_geometry(graph: slot_route.PackingSupportGraph) -> dict[str, Any]:
    half_stack = float(DEFAULT_STATOR.stack) / 2.0
    demand_rows = []
    for phase_deg, finger in FORMER_DEMANDS.items():
        points = np.asarray([
            slot_route._rounded_loop_yz(
                turn.profile_radius_mm,
                math.radians(phase_deg),
                DEFAULT_STATOR,
            )
            for turn in graph.turns
        ], dtype=float)
        y = points[:, 0]
        z = points[:, 1]
        expected_axial = 1 if "axial_positive" in finger else -1
        expected_tangential = 1 if "tangential_positive" in finger else -1
        if np.any(expected_axial * z <= half_stack):
            raise ValueError(
                f"phase {phase_deg} is not outboard of the axial face")
        if np.any(expected_tangential * y <= 0.0):
            raise ValueError(
                f"phase {phase_deg} is not on the declared tangential side")
        demand_rows.append({
            "logical_phase_deg": phase_deg,
            "required_finger": finger,
            "all_50_turns": True,
            "tangential_range_mm": [float(np.min(y)), float(np.max(y))],
            "axial_range_mm": [float(np.min(z)), float(np.max(z))],
        })

    cap = _load(PERMANENT_CAP)
    boundary = cap["geometry_boundaries"]["planar_horn_boundary"]
    if float(boundary["required_center_span_mm"]) < 6.0 - 1e-12:
        raise ValueError("permanent-cap R3 boundary drift")
    if int(boundary["failing_turn_count"]) != 50:
        raise ValueError("R3 overlap boundary no longer covers all turns")

    nominal_wire_radius = float(DEFAULT_STATOR.wire_d) / 2.0
    selected_surface_radius = 3.0
    return {
        "coordinate_frame": (
            "stator local: +X radial through active tooth, +Y tangential, "
            "+Z axial"
        ),
        "exact_wire_finished_diameter_mm": float(DEFAULT_STATOR.wire_d),
        "selected_polished_former_surface_radius_mm": selected_surface_radius,
        "nominal_wire_center_contact_radius_mm": (
            selected_surface_radius + nominal_wire_radius
        ),
        "minimum_required_wire_center_radius_mm": (
            MINIMUM_WIRE_CENTER_BEND_RADIUS_MM
        ),
        "R3_contact_pass": (
            selected_surface_radius + nominal_wire_radius
            >= MINIMUM_WIRE_CENTER_BEND_RADIUS_MM
        ),
        "necessary_former_demands": demand_rows,
        "simultaneous_pair_boundary": {
            "required_two_R3_center_span_mm": float(
                boundary["required_center_span_mm"]),
            "actual_span_range_mm": [
                float(boundary["actual_minimum_span_mm"]),
                float(boundary["actual_maximum_span_mm"]),
            ],
            "overlap_range_mm": [
                -float(boundary["maximum_bridge_mm"]),
                -float(boundary["minimum_bridge_mm"]),
            ],
            "all_50_turns_overlap": True,
            "consequence": (
                "opposed same-end R3 fingers are mutually exclusive; a "
                "single broad dwell cannot deploy both"
            ),
        },
        "status": "PASS_NECESSARY_GEOMETRY_ONLY",
    }


def phase_alias(capture: dict[str, Any]) -> dict[str, Any]:
    demands_by_angle: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for winding_pass in capture["passes"]:
        origin = int(winding_pass["physical_crossing_origin_deg"])
        direction = int(winding_pass["direction"])
        for logical_phase, finger in FORMER_DEMANDS.items():
            physical = (origin + direction * logical_phase) % 360
            demands_by_angle[physical].append({
                "pass_index": int(winding_pass["pass_index"]),
                "direction": direction,
                "physical_crossing_origin_deg": origin,
                "logical_phase_deg": logical_phase,
                "required_finger": finger,
                "m1_index_deg": int(winding_pass["m1_index_deg"]),
            })

    conflicts = []
    for physical, rows in sorted(demands_by_angle.items()):
        fingers = sorted({row["required_finger"] for row in rows})
        if len(fingers) <= 1:
            continue
        conflicts.append({
            "physical_m2_angle_deg": physical,
            "required_fingers": fingers,
            "demand_count": len(rows),
            "witnesses": rows,
        })

    angle_150 = next(row for row in conflicts
                     if row["physical_m2_angle_deg"] == 150)
    pass_0 = next(row for row in angle_150["witnesses"]
                  if row["pass_index"] == 0
                  and row["logical_phase_deg"] == 210)
    pass_2 = next(row for row in angle_150["witnesses"]
                  if row["pass_index"] == 2
                  and row["logical_phase_deg"] == 30)
    if pass_0["direction"] != -1 or pass_2["direction"] != -1:
        raise ValueError("same-direction phase-alias witness drifted")
    if pass_0["required_finger"] == pass_2["required_finger"]:
        raise ValueError("phase-alias witness no longer conflicts")

    return {
        "stationary_ring_model": (
            "one machine-fixed memoryless cam state for each physical M2 "
            "angle; direction-sensitive clutch allowed"
        ),
        "evaluated_necessary_demands": (
            len(capture["passes"]) * len(FORMER_DEMANDS)
        ),
        "conflicting_physical_angle_count": len(conflicts),
        "conflicts": conflicts,
        "decisive_same_direction_witness": {
            "physical_m2_angle_deg": 150,
            "first": pass_0,
            "second": pass_2,
            "why_directional_clutch_does_not_help": (
                "both passes traverse M2 in the negative direction but "
                "require different axial/tangential fingers"
            ),
        },
        "one_stationary_cam_law_satisfies_capture": False,
        "status": "FAIL",
    }


def _cam_ring_envelope():
    ring = Cylinder(
        CAM_RING_OUTER_RADIUS_MM,
        CAM_RING_AXIAL_THICKNESS_MM,
    ) - Cylinder(
        CAM_RING_INNER_RADIUS_MM,
        CAM_RING_AXIAL_THICKNESS_MM + 0.2,
    )
    return Pos(0.0, 0.0, CAM_RING_Z_CENTER_MM) * ring


def rigid_envelope() -> dict[str, Any]:
    """Necessary BREP check for a recessed face-cam envelope.

    The ring is seated against the flyer block and that intended mount contact
    is excluded.  No follower, selector, finger linkage, or fastener is
    invented; their absence is why this is not a complete collision proof.
    """

    ring = _cam_ring_envelope()
    flyer_parts = list(assembly.flyer_link())
    flyer_parts.extend(hardware_placements.hardware_parts_by_link(
        PARAMS, include_flagged=False).get("flyer", ()))
    flyer_distances = [float(part.distance_to(ring)) for part in flyer_parts]
    flyer_min = min(flyer_distances)
    flyer_label = flyer_parts[int(np.argmin(flyer_distances))].label

    deepest_m0 = min(
        float(row["model_target"])
        for row in _capture_rows()
        if row.get("e") == "cmd" and row.get("m") == 0
    )
    spindle_parts = assembly.spindle_link(
        DEFAULT_STATOR, final_wound_collision=True)
    spindle_location = assembly.link_location(
        "spindle", m0=deepest_m0, m1=0.0)
    spindle = Compound(children=[spindle_location * part
                                 for part in spindle_parts])
    chuck_min = float(spindle.distance_to(ring))

    static_parts = [part for part in assembly.static_link()
                    if part.label != "flyer_block"]
    static = Compound(children=static_parts)
    static_min = float(static.distance_to(ring))
    status = "PASS_NECESSARY_ENVELOPE_ONLY" if min(
        flyer_min, chuck_min, static_min
    ) >= RIGID_CLEARANCE_TARGET_MM - 1e-9 else "FAIL"
    bounds = ring.bounding_box()
    return {
        "candidate": (
            "recessed annular face-cam envelope; no grooves/follower/selector"
        ),
        "inner_radius_mm": CAM_RING_INNER_RADIUS_MM,
        "outer_radius_mm": CAM_RING_OUTER_RADIUS_MM,
        "axial_bounds_mm": [float(bounds.min.Z), float(bounds.max.Z)],
        "intended_mount_contact": "stationary flyer_block front face",
        "clearance_target_mm": RIGID_CLEARANCE_TARGET_MM,
        "minimum_flyer_clearance_mm": flyer_min,
        "minimum_flyer_part": flyer_label,
        "minimum_final_wound_chuck_clearance_mm": chuck_min,
        "deepest_captured_m0_rad": deepest_m0,
        "minimum_other_static_clearance_mm": static_min,
        "axisymmetry_covers_all_m2_angles": True,
        "not_included": [
            "cam grooves and follower rollers",
            "M1-indexed law selector",
            "former sliders, return springs, links, fasteners, and guards",
        ],
        "status": status,
    }


def route_context() -> dict[str, Any]:
    report = _load(ROUTES)
    if report.get("status") != "FAIL" or len(report.get("routes", ())) != 100:
        raise ValueError("current exact slot route report drifted")
    failures = [row for row in report["routes"] if row.get("status") != "PASS"]
    validation = report.get("validation", {})
    proof_flags = validation.get("release_proof_flags", {})
    blockers = list(validation.get("release_blockers", ()))
    if (failures
            or int(validation.get("passed_geometry_cases", -1)) != 100
            or set(blockers) != {
                "current_half_sign_specific",
                "c1_bend_continuity",
                "physical_error_budget",
            }
            or any(
                name not in proof_flags or proof_flags[name] is not False
                for name in blockers
            )):
        raise ValueError("current slot-route geometry/proof split drifted")
    turn_45 = [
        row for row in report["routes"]
        if int(row["turn_index"]) == 45
    ]
    if {(int(row["turn_index"]), int(row["half_turn_index"]))
            for row in turn_45} != {(45, 0), (45, 1)}:
        raise ValueError("current turn-45 diagnostic coverage drifted")
    lower_bounds = [
        float(row["planner_metadata"]["exact_release_postcheck"]
              ["parent_prefix_centerline_lower_bound_mm"])
        for row in turn_45
    ]
    required = float(DEFAULT_STATOR.wire_d)
    return {
        "current_route_status": "FAIL",
        "evaluated_crossing_routes": 100,
        "current_rigid_geometry_status": "PASS",
        "current_rigid_geometry_pass_count": 100,
        "current_failure_cases": [],
        "report_level_release_blockers": blockers,
        "turn_45_parent_prefix_lower_bound_mm": min(lower_bounds),
        "wire_centerline_contact_distance_mm": required,
        "current_free_approach_shortfall_mm": max(
            0.0, required - min(lower_bounds)
        ),
        "current_turn_45_parent_prefix_margin_mm": min(lower_bounds) - required,
        "contact_policy": (
            "Nomex and earlier copper are intentional support/contact "
            "surfaces; a successor route may slide on them.  Only steel "
            "penetration, invalid topology, sub-R3 guide curvature/strain, "
            "or an unbuildable mechanism is rejected."
        ),
        "former_supported_route": "NOT_EVALUATED_AFTER_CAM_PHASE_FAILURE",
        "interpretation": (
            "All 100 current rigid route geometries pass, including both "
            "turn-45 rows.  The route report remains fail-closed for sign-"
            "specific history, C1 bend continuity, and physical error budget; "
            "none of those report-level blockers proves this cam architecture."
        ),
        "status": "NOT_PROVEN",
    }


def build_report() -> dict[str, Any]:
    graph = _graph()
    capture = capture_contract()
    geometry = support_geometry(graph)
    alias = phase_alias(capture)
    rigid = rigid_envelope()
    routes = route_context()
    required_laws = int(capture["unique_cam_law_count"])
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "status": "DESIGN_NO_GO",
        "release_authorized": False,
        "assembly_integration_authorized": False,
        "scope": {
            "architecture": (
                "single stationary M2 cam ring driving mutually exclusive "
                "spring-return alternating polished formers"
            ),
            "job": "raw upstream captured 24-slot OD46 x stack15, 50 turns/tooth",
            "production_files_modified": False,
            "upstream_serial_protocol_changed": False,
            "new_commanded_axis": False,
        },
        "capture_contract": capture,
        "necessary_support_geometry": geometry,
        "stationary_cam_phase_alias": alias,
        "candidate_ring_rigid_envelope": rigid,
        "exact_wire_route_context": routes,
        "minimum_mechanical_escape": {
            "required_independent_cam_laws": required_laws,
            "why": (
                "physical crossing origins 0/180 degrees combined with both "
                "directions produce three distinct phase maps"
            ),
            "M1_index_can_select_law_without_new_serial_command": bool(
                capture["m1_index_uniquely_selects_cam_law"]),
            "candidate": (
                "positive M1-indexed three-law selector feeding an M2 cam "
                "follower, with an M0-gated all-fingers-retracted state"
            ),
            "required_features": [
                "positive three-position selector with no ambiguous neutral",
                "selector locked during each 50-turn M2 pass",
                "all fingers mechanically retracted before every M1 move",
                "spring-return failure state and retracted-position sensing",
                "complete follower/link/fastener/load and balance design",
                "new exact former-supported route against steel, liner, and progressive copper",
            ],
            "status": "CONCEPT_ONLY_NOT_PROVEN",
            "outside_this_trade": (
                "it is no longer one stationary M2-only cam law"
            ),
        },
        "gates": {
            "raw_upstream_capture_24_passes_50_turns": capture["status"] == "PASS",
            "both_M2_directions": capture["directions"] == [-1, 1],
            "exact_packing_and_R3_necessary_geometry": (
                geometry["status"] == "PASS_NECESSARY_GEOMETRY_ONLY"
                and geometry["R3_contact_pass"]
            ),
            "one_stationary_cam_law": False,
            "directional_clutch_resolves_alias": False,
            "candidate_ring_envelope_only": rigid["status"].startswith("PASS"),
            "complete_cam_follower_former_collision_sweep": False,
            "all_50_turns_both_signs_former_supported_route": False,
            "loads_balance_wear_and_enamel_coupon": False,
        },
        "decision": (
            "Do not integrate a single stationary M2 cam ring.  The exact "
            "capture requires multiple former identities at the same "
            "physical M2 angle, including two negative-direction passes, "
            "so neither a widened dwell nor a direction clutch can select "
            "the correct finger.  The recessed ring envelope itself clears "
            "the current rigid machine, but that is only a placement bound. "
            "A successor must positively select one of three cam laws from "
            "the existing M1 index (and retract through M0) before any exact "
            "former-supported wire route or production CAD is meaningful."
        ),
        "source_hashes": {
            "sim/m2_cammed_alternating_former_study.py": _sha256(
                HERE / "m2_cammed_alternating_former_study.py"),
            "out/capture/upstream_current_raw.jsonl": _sha256(CAPTURE),
            "out/reports/slot_winding_plan.json": _sha256(PLAN),
            "out/reports/slot_wire_routes.json": _sha256(ROUTES),
            "out/reports/stator_winding_guide_cap.json": _sha256(PERMANENT_CAP),
            "cad/params.py": _sha256(CAD / "params.py"),
            "cad/stator_model.py": _sha256(CAD / "stator_model.py"),
            "cad/stator_insulation_nomex410.py": _sha256(
                CAD / "stator_insulation_nomex410.py"),
            "cad/assembly.py": _sha256(CAD / "assembly.py"),
            "sim/slot_route.py": _sha256(HERE / "slot_route.py"),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def _markdown(report: dict[str, Any]) -> str:
    capture = report["capture_contract"]
    alias = report["stationary_cam_phase_alias"]
    witness = alias["decisive_same_direction_witness"]
    rigid = report["candidate_ring_rigid_envelope"]
    route = report["exact_wire_route_context"]
    return "\n".join((
        "# M2-cammed alternating-former study",
        "",
        f"**Status: {report['status']} — do not integrate the single-ring concept.**",
        "",
        report["decision"],
        "",
        "## Exact capture result",
        "",
        f"- {capture['winding_pass_count']} winding passes, 50 turns/tooth, "
        f"{capture['total_M2_winding_commands']} raw upstream M2 winding commands.",
        f"- Physical crossing origins: {capture['physical_crossing_origin_classes_deg']} degrees.",
        f"- Distinct origin/direction cam laws: {capture['unique_cam_law_count']}.",
        f"- Conflicting physical M2 angles: {alias['conflicting_physical_angle_count']}.",
        f"- Decisive witness at {witness['physical_m2_angle_deg']} degrees: "
        f"pass {witness['first']['pass_index']} and pass "
        f"{witness['second']['pass_index']} both rotate negative but require "
        "different formers.",
        "",
        "## What is and is not ruled out",
        "",
        "- The exact 0.22352 mm wire has a 3.11176 mm centerline radius on the selected polished R3 surface.",
        "- Intentional sliding on Nomex and earlier copper is allowed; it is not treated as a clearance failure.",
        f"- All {route['current_rigid_geometry_pass_count']} rigid route geometries pass; "
        f"the turn-45 parent-prefix margin is "
        f"{route['current_turn_45_parent_prefix_margin_mm']:.6f} mm.",
        "- The route artifact still fails its separate sign-specific history, C1 continuity, and physical-error-budget gates.",
        f"- A recessed ring envelope has {rigid['minimum_flyer_clearance_mm']:.3f} mm to the flyer, "
        f"{rigid['minimum_final_wound_chuck_clearance_mm']:.3f} mm to the deepest final-wound chuck, and "
        f"{rigid['minimum_other_static_clearance_mm']:.3f} mm to other static parts.",
        "- Those envelope numbers do not include grooves, followers, selectors, links, springs, fasteners, or finger sweeps.",
        "",
        "## Smallest compatible successor",
        "",
        "Use the existing M1 tooth index to positively select one of three cam laws, and use M0 to gate a mechanically retracted state before indexing. This preserves the serial command set, but it is a different, multi-input mechanism and still needs complete CAD, loads, balance, sensing, and exact progressive-wire proof.",
        "",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ))


def write_report() -> dict[str, Any]:
    report = build_report()
    REPORTS.mkdir(parents=True, exist_ok=True)
    (REPORTS / "m2_cammed_alternating_former.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    (REPORTS / "m2_cammed_alternating_former.md").write_text(
        _markdown(report), encoding="utf-8")
    return report


if __name__ == "__main__":
    result = write_report()
    print(
        f"{result['status']}: "
        f"{result['capture_contract']['unique_cam_law_count']} cam laws, "
        f"{result['stationary_cam_phase_alias']['conflicting_physical_angle_count']} "
        "conflicting physical angles"
    )
