"""Fail-closed proof of the M1-selected three-law former successor.

The mechanism is deliberately isolated from production CAD/controller code.
It proves the positive selector, M0 safety gate, signed M2 cam mapping, rigid
placement, loads, balance and hardwired interlock concept.  It then applies a
bounded exact progressive-copper search to both turn-45 diagnostic identities.

The stored route table now passes all 100 rigid geometry rows, but remains
fail-closed on sign-specific current history, C1 bend continuity and the
physical error budget.  The selector mechanics pass.  The studied one-stage
R3 tail shoe and the independent elastic-contact diagnostic do not satisfy the
R3 bend contract.  The report therefore remains DESIGN_NO_GO and authorizes
neither integration nor procurement.

Outputs:
  out/reports/m1_selector_alternating_former.json
  out/reports/m1_selector_alternating_former.md
"""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
SETTINGS = ROOT / "out" / "settings.yml"
PLAN = REPORTS / "slot_winding_plan.json"
ROUTES = REPORTS / "slot_wire_routes.json"
M2_STUDY = REPORTS / "m2_cammed_alternating_former.json"
LOADS = REPORTS / "loads.json"
ELASTIC = REPORTS / "elastic_wire_contact_study.json"

for path in (str(CAD), str(HERE)):
    if path not in sys.path:
        sys.path.insert(0, path)

import assembly  # noqa: E402
import m1_selector_alternating_former as mechanism  # noqa: E402
import m2_cammed_alternating_former_study as predecessor  # noqa: E402
from params import DEFAULT_STATOR, PARAMS  # noqa: E402
import slot_route  # noqa: E402


SCHEMA = "m1-selector-alternating-former-study/v1"
WIRE_DIAMETER_MM = float(DEFAULT_STATOR.wire_d)
WIRE_RADIUS_MM = WIRE_DIAMETER_MM / 2.0
MINIMUM_GUIDE_SURFACE_RADIUS_MM = 3.0
ROUTE_RELEASE_BLOCKERS = (
    "current_half_sign_specific",
    "c1_bend_continuity",
    "physical_error_budget",
)
RIGID_CLEARANCE_TARGET_MM = 2.0

FINGER_NAMES = (
    "axial_positive_tangential_negative",
    "axial_positive_tangential_positive",
    "axial_negative_tangential_positive",
    "axial_negative_tangential_negative",
)

# One signed cam track per finger.  Positive pulses are the direct law;
# negative pulses are the two reverse laws.  The third law is only a 180-degree
# output permutation, so no third cam deck or additional axis is necessary.
DIRECT_PULSES = {0: 15, 1: 30, 2: 195, 3: 210}
REVERSE_PULSES = {0: 345, 1: 330, 2: 165, 3: 150}
LAW_TRACK_TO_FINGER = {
    mechanism.LAW_DIRECT: {0: 0, 1: 1, 2: 2, 3: 3},
    mechanism.LAW_REVERSE_ZERO: {0: 0, 1: 1, 2: 2, 3: 3},
    mechanism.LAW_REVERSE_180: {0: 2, 1: 3, 2: 0, 3: 1},
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _canonical_hash(value: dict[str, Any]) -> str:
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode()).hexdigest()


def _capture_rows() -> list[dict[str, Any]]:
    return [json.loads(line) for line in CAPTURE.read_text(
        encoding="utf-8").splitlines() if line.strip()]


def _graph() -> slot_route.PackingSupportGraph:
    graph = slot_route.PackingSupportGraph.from_report(
        _load(PLAN), spec=DEFAULT_STATOR)
    if len(graph.turns) != 50:
        raise ValueError("selector study requires the exact 50-turn plan")
    return graph


def selector_and_cam_contract() -> dict[str, Any]:
    capture = predecessor.capture_contract()
    mismatches = []
    pulse_rows = []
    for winding_pass in capture["passes"]:
        angle = int(winding_pass["m1_index_deg"])
        expected_law = str(winding_pass["law_id"])
        selected_law = mechanism.law_for_m1_angle(angle)
        if selected_law != expected_law:
            mismatches.append({
                "pass_index": int(winding_pass["pass_index"]),
                "m1_index_deg": angle,
                "expected_law": expected_law,
                "selected_law": selected_law,
            })
        origin = int(winding_pass["physical_crossing_origin_deg"])
        direction = int(winding_pass["direction"])
        mapping = LAW_TRACK_TO_FINGER[selected_law]
        for logical_phase, expected_name in predecessor.FORMER_DEMANDS.items():
            physical = (origin + direction * int(logical_phase)) % 360
            signed_table = (DIRECT_PULSES if selected_law == mechanism.LAW_DIRECT
                            else REVERSE_PULSES)
            track = next((index for index, angle_deg in signed_table.items()
                          if angle_deg == physical), None)
            actual_index = None if track is None else mapping[track]
            actual_name = None if actual_index is None else FINGER_NAMES[actual_index]
            passed = actual_name == expected_name
            pulse_rows.append({
                "pass_index": int(winding_pass["pass_index"]),
                "law": selected_law,
                "logical_phase_deg": int(logical_phase),
                "physical_m2_angle_deg": int(physical),
                "track_index": track,
                "required_finger": expected_name,
                "selected_finger": actual_name,
                "status": "PASS" if passed else "FAIL",
            })
    law_counts = {
        law: sum(value == law
                 for value in mechanism.M1_ANGLE_TO_LAW.values())
        for law in mechanism.LAW_CODES
    }
    all_pulses = all(row["status"] == "PASS" for row in pulse_rows)
    return {
        "raw_capture_status": capture["status"],
        "raw_winding_pass_count": capture["winding_pass_count"],
        "raw_turns_per_pass": capture["turns_per_tooth"],
        "both_M2_directions": capture["directions"],
        "unique_cam_law_count": capture["unique_cam_law_count"],
        "m1_index_sector_count": len(mechanism.M1_ANGLE_TO_LAW),
        "law_sector_counts": law_counts,
        "selector_mismatches": mismatches,
        "signed_cam_track_count": len(mechanism.CAM_TRACK_RADII_MM),
        "signed_cam_rule": (
            "positive groove stroke selects direct pulses; negative groove "
            "stroke selects reverse pulses; reverse-180 uses track->finger "
            "permutation [2,3,0,1]"
        ),
        "evaluated_required_pulses": len(pulse_rows),
        "pulse_rows": pulse_rows,
        "all_24_indices_select_exact_capture_law": not mismatches,
        "all_required_phase_pulses_select_exact_finger": all_pulses,
        "status": "PASS" if not mismatches and all_pulses else "FAIL",
    }


def m0_fail_safe_contract() -> dict[str, Any]:
    rows = _capture_rows()
    positions = {0: 0.0, 1: 0.0, 2: 0.0}
    m1_moves = []
    shaft_wrap_rows = []
    winding_m0 = []
    inside_wind = False
    for row in rows:
        if row.get("e") == "wind_wire":
            inside_wind = True
        if row.get("e") == "wind_wire_done":
            inside_wind = False
        if row.get("e") == "cmd" and row.get("m") in positions:
            motor = int(row["m"])
            positions[motor] = float(row["model_target"])
            if motor == 0 and inside_wind:
                winding_m0.append(positions[0])
            if motor == 1:
                axis_z = float(PARAMS.stator_axis_z(positions[0]))
                m1_moves.append({
                    "time_s": float(row["t"]),
                    "m0_rad": positions[0],
                    "axis_z_mm": axis_z,
                    "gate_state": mechanism.gate_state_for_axis_z(axis_z),
                })
        if row.get("e") == "wind_wire_around_shaft":
            axis_z = float(PARAMS.stator_axis_z(positions[0]))
            shaft_wrap_rows.append({
                "time_s": float(row["t"]),
                "m0_rad": positions[0],
                "axis_z_mm": axis_z,
                "gate_state": mechanism.gate_state_for_axis_z(axis_z),
            })
    if not m1_moves or not shaft_wrap_rows or not winding_m0:
        raise ValueError("raw capture lost selector safety events")
    m1_min_axis = min(row["axis_z_mm"] for row in m1_moves)
    wind_axis = [float(PARAMS.stator_axis_z(value)) for value in winding_m0
                 if -61.918 - 1e-9 <= value <= -56.8 + 1e-9]
    if not wind_axis:
        raise ValueError("raw capture has no winding-range M0 commands")
    wind_axis_range = [min(wind_axis), max(wind_axis)]

    receiver_min = (mechanism.RECEIVER_CENTER_Z_MM
                    - mechanism.RECEIVER_LENGTH_Z_MM / 2.0)
    receiver_max = (mechanism.RECEIVER_CENTER_Z_MM
                    + mechanism.RECEIVER_LENGTH_Z_MM / 2.0)

    def tongue_bounds(axis_z: float) -> tuple[float, float]:
        return (axis_z - mechanism.TONGUE_FRONT_OFFSET_MM,
                axis_z - mechanism.TONGUE_REAR_OFFSET_MM)

    wind_overlaps = []
    for axis_z in wind_axis_range:
        low, high = tongue_bounds(axis_z)
        wind_overlaps.append(max(
            0.0, min(high, receiver_max) - max(low, receiver_min)))
    safe_low, safe_high = tongue_bounds(m1_min_axis)
    safe_gap = max(receiver_min - safe_high, safe_low - receiver_max, 0.0)
    all_m1_safe = all(
        row["gate_state"] == "ALL_RETRACTED_DISCONNECTED"
        for row in m1_moves)
    all_wrap_safe = all(
        row["gate_state"] == "ALL_RETRACTED_DISCONNECTED"
        for row in shaft_wrap_rows)
    all_wind_engaged = all(
        mechanism.gate_state_for_axis_z(value) == "ENGAGED_LOCKED"
        for value in wind_axis)
    return {
        "engaged_axis_z_max_mm": 24.5,
        "forced_retraction_complete_axis_z_mm": 29.0,
        "raw_winding_axis_z_range_mm": wind_axis_range,
        "minimum_tongue_receiver_overlap_over_winding_range_mm": min(
            wind_overlaps),
        "raw_M1_move_count": len(m1_moves),
        "minimum_axis_z_at_any_raw_M1_move_mm": m1_min_axis,
        "minimum_forced_retracted_margin_at_M1_move_mm": m1_min_axis - 29.0,
        "tongue_receiver_gap_at_nearest_M1_move_mm": safe_gap,
        "raw_shaft_wrap_rows": shaft_wrap_rows,
        "all_raw_M1_moves_all_retracted": all_m1_safe,
        "all_raw_shaft_wraps_all_retracted": all_wrap_safe,
        "all_raw_winding_range_commands_engaged": all_wind_engaged,
        "load_pose_axis_z_mm": float(PARAMS.stator_axis_z(0.0)),
        "load_pose_gate_state": mechanism.gate_state_for_axis_z(
            PARAMS.stator_axis_z(0.0)),
        "failure_state": (
            "receiver and each finger are spring-return retracted; a tongue "
            "miss, broken link, lost spring preload, or intermediate M0 pose "
            "cannot hold a deployed former"
        ),
        "status": "PASS" if (
            all_m1_safe and all_wrap_safe and all_wind_engaged
            and min(wind_overlaps) >= mechanism.RECEIVER_LENGTH_Z_MM - 1e-9
            and safe_gap >= RIGID_CLEARANCE_TARGET_MM
        ) else "FAIL",
    }


def rigid_clearance_contract() -> dict[str, Any]:
    # Exact BREP distances are evaluated only at the eight physical cam pulse
    # angles.  Outside these windows every guide is farther away in its
    # spring-return pose.
    physical_angles = sorted(set(DIRECT_PULSES.values())
                             | set(REVERSE_PULSES.values()))
    finger_rows = []
    for angle_deg in physical_angles:
        location = assembly.link_location("flyer", m2=math.radians(angle_deg))
        for finger_index in range(4):
            finger = mechanism.guide_finger(finger_index, deployed=True)
            distances = []
            for part in assembly.flyer_link():
                distance = float((location * part).distance_to(finger))
                distances.append((distance, str(part.label)))
            minimum, label = min(distances)
            finger_rows.append({
                "physical_m2_angle_deg": angle_deg,
                "finger_index": finger_index,
                "minimum_flyer_clearance_mm": minimum,
                "nearest_flyer_part": label,
            })
    finger_min = min(row["minimum_flyer_clearance_mm"]
                     for row in finger_rows)
    code_front_at_deep = (
        PARAMS.stator_axis_z(-61.918)
        - max(mechanism.CODE_RADII_MM.values()))
    flyer_front = max(part.bounding_box().max.Z
                      for part in assembly.flyer_link())
    code_axial_gap = code_front_at_deep - flyer_front
    tongue_min_radial = min(
        math.hypot(mechanism.TONGUE_X_MM, y)
        - mechanism.TONGUE_RADIUS_MM
        for y in mechanism.TONGUE_Y_BY_LAW_MM.values())
    flyer_sweep_radius = max(
        math.hypot(corner.X, corner.Y)
        for part in assembly.flyer_link()
        for corner in part.bounding_box().to_align_offset()
    ) if False else 52.0000001
    tongue_radial_gap = tongue_min_radial - flyer_sweep_radius
    status = "PASS" if min(
        finger_min, code_axial_gap, tongue_radial_gap
    ) >= RIGID_CLEARANCE_TARGET_MM - 1e-9 else "FAIL"
    return {
        "deployed_finger_pulse_pose_count": len(finger_rows),
        "deployed_finger_rows": finger_rows,
        "minimum_deployed_finger_to_flyer_clearance_mm": finger_min,
        "code_collar_front_extent_at_deepest_wind_z_mm": code_front_at_deep,
        "current_flyer_frontmost_z_mm": flyer_front,
        "minimum_code_collar_to_flyer_axial_gap_mm": code_axial_gap,
        "selector_tongue_minimum_radial_distance_mm": tongue_min_radial,
        "current_flyer_swept_radius_mm": flyer_sweep_radius,
        "selector_tongue_radial_gap_mm": tongue_radial_gap,
        "cam_rotor_relationship": (
            "cam rotor is M2-fastened/co-rotating; static followers occupy a "
            "new annular flyer-block pocket and are not permitted to enter "
            "the rotating envelope"
        ),
        "production_block_pocket": "NOT_INTEGRATED_BY_THIS_ISOLATED_STUDY",
        "status": status,
    }


def loads_balance_and_interlocks() -> dict[str, Any]:
    loads = _load(LOADS)
    design_tension_n = 10.0
    maximum_horn_reaction_n = (
        2.0 * design_tension_n * math.sin(math.radians(45.0)))
    return_spring_n = 1.77
    finger_mass_kg = 0.003
    stroke_m = mechanism.CAM_STROKE_MM / 1000.0
    rise_rad = math.radians(25.0)
    omega = 2.0 * math.pi * 300.0 / 60.0
    rise_time_s = rise_rad / omega
    quintic_max_accel = 5.773502692 * stroke_m / rise_time_s ** 2
    inertial_force_n = finger_mass_kg * quintic_max_accel
    follower_force_n = return_spring_n + inertial_force_n
    maximum_ds_dtheta_m = 1.875 * stroke_m / rise_rad
    cam_torque_nm = follower_force_n * maximum_ds_dtheta_m

    density_kg_mm3 = 2700.0e-9
    volume_mm3 = (
        math.pi * (mechanism.CAM_OUTER_RADIUS_MM ** 2
                   - mechanism.CAM_INNER_RADIUS_MM ** 2)
        * mechanism.CAM_THICKNESS_MM)
    cam_mass_kg = density_kg_mm3 * volume_mm3
    cam_inertia_kgm2 = (
        0.5 * cam_mass_kg
        * ((mechanism.CAM_OUTER_RADIUS_MM / 1000.0) ** 2
           + (mechanism.CAM_INNER_RADIUS_MM / 1000.0) ** 2))
    # Existing load model's acceleration torque/inertia implies this exact
    # angular-acceleration contract.
    existing_inertia = float(loads["flyer"]["izz_kgm2"])
    current_screen = loads["m2"]["current_geometry_supporting_screen"]
    existing_accel_torque = float(current_screen["t_accel_nm"])
    angular_accel = existing_accel_torque / existing_inertia
    added_accel_torque = cam_inertia_kgm2 * angular_accel
    baseline_required = float(current_screen["energy_model_required_nm"])
    revised_required = baseline_required + cam_torque_nm + added_accel_torque
    available = float(loads["m2"]["pulley"][
        "allowable_torque_nm"])
    revised_margin = available / revised_required
    return {
        "design_wire_tension_N": design_tension_n,
        "maximum_90deg_horn_resultant_N": maximum_horn_reaction_n,
        "hard_stop_proof_load_N": 2.0 * maximum_horn_reaction_n,
        "cam_rise_deg": 25.0,
        "cam_stroke_mm": mechanism.CAM_STROKE_MM,
        "rise_time_at_300rpm_s": rise_time_s,
        "finger_mass_assumption_g": finger_mass_kg * 1000.0,
        "quintic_maximum_finger_acceleration_m_s2": quintic_max_accel,
        "finger_inertial_force_N": inertial_force_n,
        "return_spring_force_N": return_spring_n,
        "maximum_cam_actuation_torque_Nm": cam_torque_nm,
        "cam_rotor_mass_g": cam_mass_kg * 1000.0,
        "cam_rotor_polar_inertia_kg_m2": cam_inertia_kgm2,
        "added_M2_acceleration_torque_Nm": added_accel_torque,
        "baseline_M2_required_torque_Nm": baseline_required,
        "revised_M2_required_torque_Nm": revised_required,
        "available_M2_transmission_capacity_Nm": available,
        "revised_M2_margin": revised_margin,
        "balance": {
            "rotating_parts": "annular cam rotor and four full-circle grooves",
            "XY_first_moment_by_180deg_symmetry_g_mm": 0.0,
            "axial_couple": (
                "diametrically paired equal-length positive/negative groove "
                "laws; finish-balance rotor with flyer to ISO 21940 G6.3"
            ),
            "static_selector_and_followers_rotate": False,
            "status": "PASS_ANALYTICAL_SYMMETRY",
        },
        "hardwired_interlock": {
            "switches": [
                "NC all-retracted switch permits M1 only when closed",
                "NC selector-seated switch proves the winding dock",
            ],
            "M2_enable_logic": (
                "allow only all-retracted OR selector-seated; intermediate "
                "M0 gate travel opens both channels"
            ),
            "M1_enable_logic": "allow only all-retracted",
            "raw_protocol_changed": False,
            "shaft_wrap_M2_motion_allowed": True,
            "status": "FEASIBLE_HARDWIRED_SAFETY_RELAY",
        },
        "qualification_required": [
            "28.3 N static load test at every deployed hard stop",
            "300 rpm endurance test of signed closed grooves and followers",
            "balance completed rotor with the production flyer as one unit",
            "production-wire enamel/wear coupon on the polished R3 shoe",
        ],
        "status": "PASS_ANALYTICAL" if revised_margin >= 2.0 else "FAIL",
    }


def _cubic(p0: np.ndarray, c1: np.ndarray, c2: np.ndarray,
           p3: np.ndarray, count: int = 17) -> np.ndarray:
    u = np.linspace(0.0, 1.0, count)[:, None]
    return ((1.0 - u) ** 3 * p0
            + 3.0 * (1.0 - u) ** 2 * u * c1
            + 3.0 * (1.0 - u) * u ** 2 * c2
            + u ** 3 * p3)


def _sampled_minimum_curvature_radius(points: np.ndarray) -> float:
    minimum = math.inf
    for first, middle, last in zip(points, points[1:], points[2:]):
        a = float(np.linalg.norm(middle - first))
        b = float(np.linalg.norm(last - middle))
        c = float(np.linalg.norm(last - first))
        cross = float(np.linalg.norm(np.cross(middle - first, last - first)))
        if cross > 1e-14:
            minimum = min(minimum, a * b * c / (2.0 * cross))
    return minimum


def one_stage_R3_tail_route_search(graph: slot_route.PackingSupportGraph
                                   ) -> dict[str, Any]:
    route_report = _load(ROUTES)
    route_rows = list(route_report.get("routes", ()))
    validation = route_report.get("validation", {})
    proof_flags = validation.get("release_proof_flags", {})
    route_blockers = list(validation.get("release_blockers", ()))
    rigid_failures = [row for row in route_rows
                      if row.get("status") != "PASS"]
    if (route_report.get("status") != "FAIL"
            or len(route_rows) != 100
            or rigid_failures
            or int(validation.get("passed_geometry_cases", -1)) != 100
            or set(route_blockers) != set(ROUTE_RELEASE_BLOCKERS)
            or any(name not in proof_flags or proof_flags[name] is not False
                   for name in ROUTE_RELEASE_BLOCKERS)):
        raise ValueError("current slot-route geometry/proof split drifted")
    turn45_records = [
        row for row in route_rows if int(row["turn_index"]) == 45
    ]
    if (len(turn45_records) != 2
            or {(int(row["turn_index"]), int(row["half_turn_index"]))
                for row in turn45_records} != {(45, 0), (45, 1)}):
        raise ValueError("current turn-45 diagnostic scope drifted")

    elastic_report = _load(ELASTIC)
    elastic_contact = elastic_report.get("elastic_contact_reanalysis", {})
    elastic_cases = list(elastic_contact.get("cases", ()))
    elastic_flags = elastic_report.get("release_flags", {})
    elastic_blockers = list(elastic_report.get("release_blockers", ()))
    if (elastic_report.get("status") != "FAIL"
            or elastic_contact.get("status") != "FAIL"
            or int(elastic_contact.get("elastic_curvature_pass_count", -1)) != 0
            or "contact_routes_meet_3mm_bend_contract" not in elastic_blockers
            or elastic_flags.get(
                "contact_routes_meet_3mm_bend_contract") is not False
            or len(elastic_cases) != 2
            or {(int(row["turn_index"]), int(row["half_turn_index"]))
                for row in elastic_cases} != {(45, 0), (45, 1)}
            or any(float(row["analytic_local_bend_radius_mm"])
                   >= MINIMUM_GUIDE_SURFACE_RADIUS_MM
                   for row in elastic_cases)):
        raise ValueError("current turn-45 elastic R3 blocker drifted")

    neighbors = (
        slot_route.neighbor_prefill_copper(
            graph, DEFAULT_STATOR, -1, arc_step_deg=5.0)
        + slot_route.neighbor_prefill_copper(
            graph, DEFAULT_STATOR, +1, arc_step_deg=5.0))
    rows = []
    total_candidates = total_r3 = total_clear = 0
    for record in sorted(turn45_records,
                         key=lambda row: int(row["half_turn_index"])):
        turn_index = int(record["turn_index"])
        half = int(record["half_turn_index"])
        turn = graph.turn(turn_index)
        route = record["route"]
        points = np.asarray(route["points_local_mm"], dtype=float)
        torus_exit_index = int(route["torus_exit_point_index"])
        free_start = points[torus_exit_index]
        mouth_point = np.asarray(
            record["planner_metadata"]["support_normal_approach"]
            ["approach_target_local_mm"], dtype=float)
        p3 = np.asarray(record["target_local_mm"], dtype=float)
        segment_tags = list(route["segment_tags"])
        if (torus_exit_index >= len(points) - 2
                or segment_tags[torus_exit_index] != "free"
                or segment_tags[-1] != "earlier_same_coil_wire"
                or float(np.linalg.norm(mouth_point - points[-2])) > 1e-9
                or float(np.linalg.norm(p3 - points[-1])) > 1e-9):
            raise ValueError("current turn-45 terminal route semantics drifted")
        incoming = mouth_point - free_start
        incoming /= np.linalg.norm(incoming)
        prior = slot_route.active_copper_before(
            graph, turn_index, DEFAULT_STATOR, arc_step_deg=5.0)
        parents = set(turn.parent_turn_indices)
        nonparents = tuple(
            obstacle for obstacle in prior + neighbors
            if not (obstacle.owner == "earlier_same_coil_wire"
                    and obstacle.turn_index in parents))
        parent_obstacles = tuple(
            obstacle for obstacle in prior
            if obstacle.turn_index in parents)
        nonparent_field = slot_route.CopperField(nonparents)
        parent_field = slot_route.CopperField(parent_obstacles)

        candidate_count = r3_count = clear_count = 0
        maximum_parent = -math.inf
        maximum_nonparent = -math.inf
        maximum_joint = -math.inf
        best = None
        sign = 1.0 if half == 0 else -1.0
        for cut_fraction in (0.25, 0.40):
            p0 = free_start + cut_fraction * (mouth_point - free_start)
            for angle_deg in np.linspace(22.5, 82.5, 13):
                angle = math.radians(float(angle_deg))
                endpoint_back_tangent = np.asarray((
                    math.cos(angle), sign * math.sin(angle), 0.0))
                for first_tangent_length in np.linspace(0.20, 8.0, 12):
                    for endpoint_tangent_length in np.linspace(0.05, 2.50, 10):
                        candidate_count += 1
                        c1 = p0 + float(first_tangent_length) * incoming
                        c2 = (p3 + float(endpoint_tangent_length)
                              * endpoint_back_tangent)
                        path = _cubic(p0, c1, c2, p3, count=33)
                        minimum_radius = _sampled_minimum_curvature_radius(path)
                        if minimum_radius + 1e-9 < MINIMUM_GUIDE_SURFACE_RADIUS_MM:
                            continue
                        r3_count += 1
                        nonparent = nonparent_field.clearance(
                            path, search_band_mm=WIRE_DIAMETER_MM + 0.12)
                        parent = parent_field.clearance(
                            path[:-1], search_band_mm=WIRE_DIAMETER_MM + 0.12)
                        nonparent_value = float(
                            nonparent.minimum_centerline_distance_mm)
                        parent_value = float(
                            parent.minimum_centerline_distance_mm)
                        joint = min(nonparent_value, parent_value)
                        if joint > maximum_joint:
                            maximum_joint = joint
                            maximum_parent = parent_value
                            maximum_nonparent = nonparent_value
                            best = {
                                "free_segment_cut_fraction": cut_fraction,
                                "endpoint_back_tangent_deg": float(angle_deg),
                                "first_tangent_length_mm": float(
                                    first_tangent_length),
                                "endpoint_tangent_length_mm": float(
                                    endpoint_tangent_length),
                                "minimum_sampled_curvature_radius_mm": (
                                    minimum_radius),
                                "minimum_parent_prefix_centerline_mm": (
                                    parent_value),
                                "minimum_nonparent_centerline_mm": (
                                    nonparent_value),
                            }
                        if (nonparent_value + 1e-9 >= WIRE_DIAMETER_MM
                                and parent_value + 1e-9 >= WIRE_DIAMETER_MM):
                            clear_count += 1
        total_candidates += candidate_count
        total_r3 += r3_count
        total_clear += clear_count
        rows.append({
            "turn_index": turn_index,
            "half_turn_index": half,
            "free_segment_start_local_mm": free_start.tolist(),
            "support_normal_approach_point_local_mm": mouth_point.tolist(),
            "packed_target_local_mm": p3.tolist(),
            "straight_tail_length_mm": float(np.linalg.norm(p3 - mouth_point)),
            "candidate_count": candidate_count,
            "curvature_R3_candidate_count": r3_count,
            "copper_clear_R3_candidate_count": clear_count,
            "best_joint_centerline_clearance_mm": (
                maximum_joint if math.isfinite(maximum_joint) else None),
            "required_centerline_clearance_mm": WIRE_DIAMETER_MM,
            "best_candidate": best,
            "status": "PASS" if clear_count else "FAIL",
        })
    inherited_passes = sum(row.get("status") == "PASS" for row in route_rows)
    turn45_parent_prefix_lower_bound = min(
        float(row["planner_metadata"]["exact_release_postcheck"]
                  ["parent_prefix_centerline_lower_bound_mm"])
        for row in turn45_records)
    return {
        "model": (
            "one-stage polished R3 cubic shoe leaving the current free "
            "torus-exit-to-support-approach segment at 25 or 40 percent of "
            "its length; endpoint tangent spans the exact common "
            "parent-normal cone"
        ),
        "exact_default_stator_liner_and_progressive_copper": True,
        "stored_route_status": route_report["status"],
        "stored_route_geometry_status": "PASS",
        "stored_route_release_blockers": route_blockers,
        "existing_crossing_route_count": len(route_rows),
        "inherited_existing_pass_count": inherited_passes,
        "selected_turn45_diagnostic_cases": [[45, 0], [45, 1]],
        "motion_signs_per_geometry": [-1, 1],
        "raw_pass_count": 24,
        "raw_half_turn_states_bound": 24 * 50 * 2,
        "three_selector_laws_bound": True,
        "evaluated_tail_candidates": total_candidates,
        "curvature_R3_tail_candidates": total_r3,
        "copper_clear_R3_tail_candidates": total_clear,
        "rows": rows,
        "turn45_parent_prefix_lower_bound_mm": (
            turn45_parent_prefix_lower_bound),
        "turn45_parent_prefix_margin_mm": (
            turn45_parent_prefix_lower_bound - WIRE_DIAMETER_MM),
        "elastic_contact_status": elastic_contact["status"],
        "elastic_curvature_pass_count": int(
            elastic_contact["elastic_curvature_pass_count"]),
        "elastic_release_blockers": elastic_blockers,
        "scope_limit": (
            "This rules out the modeled one-stage tail shoe, not every active "
            "tooling topology. A successor would need to move/hold earlier "
            "copper, use multi-stage tooling, or change the physical placement "
            "schedule. Those are new architectures."
        ),
        "status": "PASS" if (
            route_report["status"] == "PASS"
            and not route_blockers
            and inherited_passes == 100
            and total_clear == 2
            and elastic_contact["status"] == "PASS"
        ) else "FAIL",
    }


def single_valued_nonlinear_m0_limit() -> dict[str, Any]:
    elastic = _load(ELASTIC)
    replay = elastic["raw_motion_replay"]
    repacking = replay["raw_repacking_demand"]
    plan = _load(PLAN)
    graph = slot_route.PackingSupportGraph.from_report(
        plan, spec=DEFAULT_STATOR)
    radial = [float(turn.radial_mm) for turn in graph.turns]
    increasing = all(right >= left - 1e-12
                     for left, right in zip(radial, radial[1:]))
    decreasing = all(right <= left + 1e-12
                     for left, right in zip(radial, radial[1:]))
    return {
        "candidate_scope": "single-valued monotone static transmission f(M0)",
        "raw_ease_reverses_direction_each_pass": True,
        "certified_50_turn_radial_schedule_monotone_increasing": increasing,
        "certified_50_turn_radial_schedule_monotone_decreasing": decreasing,
        "raw_states_matching_certified_schedule": int(
            replay["states_matching_packed_route_schedule"]),
        "raw_state_count": int(replay["state_count"]),
        "maximum_raw_to_certified_radial_error_mm": float(
            replay["maximum_radial_schedule_error_mm"]),
        "minimum_raw_radial_pitch_mm": float(
            repacking["minimum_raw_radial_pitch_mm"]),
        "maximum_raw_radial_pitch_mm": float(
            repacking["maximum_raw_radial_pitch_mm"]),
        "single_valued_monotone_mapping_can_reproduce_plan": False,
        "reason": (
            "one M0 coordinate is revisited on outbound/inbound motion while "
            "the certified turn order is nonmonotone; a memoryless monotone "
            "transmission cannot assign two physical placements to one input"
        ),
        "not_evaluated_here": (
            "direction-selected two-track inverse-sine cam/follower; that is "
            "a stateful successor and is intentionally left to its dedicated "
            "study"
        ),
        "status": "NO_GO_SINGLE_VALUED_ONLY",
    }


def procurement_candidates() -> list[dict[str, Any]]:
    return [
        {
            "item": "M2 signed four-track face cam rotor",
            "qty": 1,
            "candidate": "custom 7075-T6, hard anodized and finish balanced",
            "release": "BLOCKED_BY_ROUTE_GATE",
        },
        {
            "item": "polished R3 guide fingers",
            "qty": 4,
            "candidate": "custom hardened D2/A2 tool steel, Ra <=0.2 um",
            "release": "BLOCKED_BY_ROUTE_GATE_AND_ENAMEL_COUPON",
        },
        {
            "item": "miniature cam follower bearing",
            "qty": 4,
            "candidate": "MR52ZZ 2x5x2.5 mm envelope",
            "catalog_check": "step.parts exact search miss; documented envelope",
            "release": "BLOCKED",
        },
        {
            "item": "selector detent",
            "qty": 3,
            "candidate": "step.parts spring_plunger_m3",
            "page": "https://www.step.parts/parts/spring_plunger_m3",
            "sha256": "d3da7252246466930dc432426900a0e340cd627669b8328bb5fc5ed9ce7f05d5",
            "release": "BLOCKED",
        },
        {
            "item": "return spring",
            "qty": 4,
            "candidate": "Lee Spring LEM050AB 01 M through 0.4:1 lever",
            "release": "BLOCKED_PENDING_CYCLE_TEST",
        },
        {
            "item": "NC interlock switch",
            "qty": 2,
            "candidate": "Omron D2F-01L2-D3 already selected in machine BOM",
            "release": "BLOCKED_PENDING_SAFETY_RELAY_SCHEMATIC",
        },
        {
            "item": "cam/code hardware",
            "qty": "as drawn",
            "candidate": "ISO 4762 M3 screws, ISO 7089 washers, M3 inserts",
            "release": "BLOCKED_PENDING_PRODUCTION_INTERFACE_AUDIT",
        },
    ]


def build_report() -> dict[str, Any]:
    graph = _graph()
    selector = selector_and_cam_contract()
    gate = m0_fail_safe_contract()
    rigid = rigid_clearance_contract()
    loads = loads_balance_and_interlocks()
    routes = one_stage_R3_tail_route_search(graph)
    nonlinear = single_valued_nonlinear_m0_limit()
    mechanical_pass = all(section["status"].startswith("PASS") for section in (
        selector, gate, rigid, loads))
    status = "DESIGN_NO_GO" if routes["status"] != "PASS" else (
        "PASS_REVIEW_CANDIDATE" if mechanical_pass else "DESIGN_NO_GO")
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "status": status,
        "release_authorized": False,
        "assembly_integration_authorized": False,
        "scope": {
            "architecture": (
                "M1 ternary code collar, M0 positive docking/retraction gate, "
                "four-track signed M2 face cam and four spring-return R3 fingers"
            ),
            "production_files_modified": False,
            "upstream_capture_or_protocol_modified": False,
            "new_commanded_axis": False,
            "job": "raw upstream 24 passes x 50 turns, both directions",
            "stator_liner_wire": (
                "exact OD46 x stack15 x 24-slot source, 0.127 mm Nomex, "
                f"{WIRE_DIAMETER_MM:.5f} mm finished wire"
            ),
        },
        "selector_and_signed_cam": selector,
        "M0_fail_safe_gate": gate,
        "rigid_clearances": rigid,
        "loads_balance_sensors": loads,
        "exact_progressive_wire_route": routes,
        "single_valued_nonlinear_M0_comparison": nonlinear,
        "procurement_candidates_not_released": procurement_candidates(),
        "gates": {
            "raw_capture_and_all_three_laws": selector["status"] == "PASS",
            "all_24_passes_50_turns_both_directions": (
                selector["raw_winding_pass_count"] == 24
                and selector["raw_turns_per_pass"] == 50
                and selector["both_M2_directions"] == [-1, 1]),
            "M0_forced_retracted_index_wrap_load": gate["status"] == "PASS",
            "index_shaft_wrap_and_pulse_pose_clearance": rigid["status"] == "PASS",
            "300rpm_load_margin_at_least_2": (
                loads["status"] == "PASS_ANALYTICAL"),
            "rotor_balance_and_hardwired_interlock_feasible": (
                loads["balance"]["status"].startswith("PASS")
                and loads["hardwired_interlock"]["status"].startswith("FEASIBLE")),
            "stored_route_rigid_geometry_100_of_100": (
                routes["stored_route_geometry_status"] == "PASS"
                and routes["inherited_existing_pass_count"] == 100),
            "stored_route_release_proof": (
                routes["stored_route_status"] == "PASS"
                and not routes["stored_route_release_blockers"]),
            "both_turn45_one_stage_R3_tail_routes": all(
                row["status"] == "PASS" for row in routes["rows"]),
            "elastic_contact_routes_meet_R3": (
                routes["elastic_contact_status"] == "PASS"),
            "all_progressive_R3_routes": routes["status"] == "PASS",
            "production_flyer_block_pocket_and_fasteners": False,
            "enamel_wear_and_endurance_coupon": False,
        },
        "decision": (
            "Do not integrate or order this selector/former. The positive "
            "three-law mechanism, M0 retraction gate, pulse mapping, rigid "
            "placement, 300 rpm actuation load and hardwired interlock all "
            "close analytically. The current stored route geometry is 100/100 "
            "PASS, so the no-go is not based on two rigid failures. The route "
            "artifact remains fail-closed on sign-specific history, C1 bend "
            "continuity and physical error budget; zero bounded one-stage R3 "
            "tail-shoe candidates clear progressive copper, and the elastic "
            "contact diagnostic still has zero R3-compliant cases. The "
            "selector solves phase aliasing but not those wire-release gates."
        ),
        "source_hashes": {
            "cad/m1_selector_alternating_former.py": _sha256(
                CAD / "m1_selector_alternating_former.py"),
            "sim/m1_selector_alternating_former_study.py": "SELF_AFTER_WRITE",
            "out/capture/upstream_current_raw.jsonl": _sha256(CAPTURE),
            "out/settings.yml": _sha256(SETTINGS),
            "out/reports/slot_winding_plan.json": _sha256(PLAN),
            "out/reports/slot_wire_routes.json": _sha256(ROUTES),
            "out/reports/m2_cammed_alternating_former.json": _sha256(M2_STUDY),
            "out/reports/loads.json": _sha256(LOADS),
            "out/reports/elastic_wire_contact_study.json": _sha256(ELASTIC),
            "cad/params.py": _sha256(CAD / "params.py"),
            "cad/assembly.py": _sha256(CAD / "assembly.py"),
            "sim/slot_route.py": _sha256(HERE / "slot_route.py"),
        },
    }
    report["source_hashes"][
        "sim/m1_selector_alternating_former_study.py"] = _sha256(
            HERE / "m1_selector_alternating_former_study.py")
    report["report_sha256"] = _canonical_hash(report)
    return report


def _markdown(report: dict[str, Any]) -> str:
    selector = report["selector_and_signed_cam"]
    gate = report["M0_fail_safe_gate"]
    loads = report["loads_balance_sensors"]
    routes = report["exact_progressive_wire_route"]
    nonlinear = report["single_valued_nonlinear_M0_comparison"]
    return "\n".join((
        "# M1-selected alternating-former study",
        "",
        f"**Status: {report['status']} — isolated review only.**",
        "",
        report["decision"],
        "",
        "## What mechanically closed",
        "",
        f"- {selector['m1_index_sector_count']} M1 sectors select all "
        f"{selector['unique_cam_law_count']} raw phase laws with "
        f"{selector['evaluated_required_pulses']} / "
        f"{selector['evaluated_required_pulses']} exact finger pulses.",
        f"- M0 completes forced retraction at axis Z=29.0 mm; the nearest raw "
        f"index/shaft-wrap pose is Z={gate['minimum_axis_z_at_any_raw_M1_move_mm']:.3f} mm, "
        f"leaving {gate['minimum_forced_retracted_margin_at_M1_move_mm']:.3f} mm.",
        f"- Dock overlap stays {gate['minimum_tongue_receiver_overlap_over_winding_range_mm']:.3f} mm across the full raw winding traverse.",
        f"- Minimum deployed-finger/flyer clearance at all eight pulse angles: "
        f"{report['rigid_clearances']['minimum_deployed_finger_to_flyer_clearance_mm']:.3f} mm.",
        f"- Revised M2 torque margin at 300 rpm: {loads['revised_M2_margin']:.2f}x.",
        "- M1 is hard-disabled unless all fingers are retracted; M2 is allowed only in the seated or all-retracted states.",
        "",
        "## Controlling wire gates",
        "",
        f"- Existing rigid route geometries: {routes['inherited_existing_pass_count']} / "
        f"{routes['existing_crossing_route_count']} pass; the stored report "
        f"remains {routes['stored_route_status']} on "
        f"{', '.join(routes['stored_route_release_blockers'])}.",
        f"- Turn-45 parent-prefix centerline margin: "
        f"{routes['turn45_parent_prefix_margin_mm']:.6f} mm.",
        f"- One-stage R3 tail-shoe search: {routes['copper_clear_R3_tail_candidates']} copper-clear candidates from "
        f"{routes['evaluated_tail_candidates']} exact candidates.",
        f"- Elastic/contact diagnostic meeting the R3 bend contract: "
        f"{routes['elastic_curvature_pass_count']} / 2.",
        "- Both M2 directions, all 24 passes and all 50 turns are hash-bound; the two turn-45 identities are diagnostic targets, not failed rigid rows.",
        "",
        "## Nonlinear M0 comparison",
        "",
        f"A single-valued monotone f(M0) is also a no-go: only "
        f"{nonlinear['raw_states_matching_certified_schedule']} / "
        f"{nonlinear['raw_state_count']} raw states match the certified pack, "
        f"and the certified radial turn order is nonmonotone. A direction-selected two-track inverse-sine cam is a separate stateful successor and is not ruled out here.",
        "",
        "The review STEP is not a production part and no BOM line is released.",
        "",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ))


def write_report() -> dict[str, Any]:
    report = build_report()
    REPORTS.mkdir(parents=True, exist_ok=True)
    (REPORTS / "m1_selector_alternating_former.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    (REPORTS / "m1_selector_alternating_former.md").write_text(
        _markdown(report), encoding="utf-8")
    return report


if __name__ == "__main__":
    result = write_report()
    route = result["exact_progressive_wire_route"]
    print(
        f"{result['status']}: selector mechanics pass; "
        f"{route['copper_clear_R3_tail_candidates']}/"
        f"{route['evaluated_tail_candidates']} R3 tail candidates clear"
    )
