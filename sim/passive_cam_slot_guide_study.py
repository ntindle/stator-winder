"""Fail-closed study of a passive M0-cammed, mouth-only winding guide.

The selected concept has no commanded actuator and changes no serial command:

* the guide head follows the M0 carriage one-for-one only over the winding
  range, keeping four polished quarter-horns at the stator mouth;
* after the final winding M0 position, lost motion leaves the guide behind
  while the stator extracts radially;
* only after the complete final-wound OD has cleared does a return spring move
  each finger 0.8 mm tangentially to its park position;
* M2 is not used as a cam input because its physical angle repeats on every
  one of the 50 turns.

The decisive route model is intentionally simple and physical: between the
last mouth-horn contact and the instantaneous lay point, a tensioned conductor
is a straight segment unless another support is introduced.  Introducing an
inner support would no longer be the requested mouth-only pair.  Every segment
is checked against the exact default source stator mesh with the exact default
liner/wire centre offset, all previously laid active-tooth turns, both fully
wound adjacent teeth, and the already-laid prefix of the current turn.  The
current prefix is direction-specific and is trimmed by one wire diameter at
the live end to exclude intentional same-conductor continuity.

Outputs:
  out/reports/passive_cam_slot_guide.json
  out/reports/passive_cam_slot_guide.md

This module never edits production CAD, controller code, packing, or the
upstream serial protocol.
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
import trimesh


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
CAPTURE = ROOT / "out" / "capture" / "commands.jsonl"
PLAN = REPORTS / "slot_winding_plan.json"
PACKING = REPORTS / "slot_packing.json"
EXISTING_ROUTE = REPORTS / "slot_wire_routes.json"

for path in (str(CAD), str(HERE)):
    if path not in sys.path:
        sys.path.insert(0, path)

from params import DEFAULT_STATOR, PARAMS  # noqa: E402
import coil_growth  # noqa: E402
import passive_cam_slot_guide as guide  # noqa: E402
import slot_route  # noqa: E402
import stator_model  # noqa: E402


SCHEMA = "passive-cam-slot-guide-study/v1"
PHASE_STEP_DEG = 5
PHASES_PER_TURN = 360 // PHASE_STEP_DEG
MINIMUM_GUIDE_WIRE_CENTER_RADIUS_MM = 3.0
LINER_NOMINAL_MM = 0.127
LINER_RECEIVING_MAX_MM = 0.140
RECEIVING_WIRE_DIAMETER_MAX_MM = 0.235
TANGENTIAL_PARK_STROKE_MM = 0.800
FINAL_WOUND_EXTRA_RADIUS_MM = 3.0
RIGID_CLEARANCE_TARGET_MM = 2.0
DESIGN_WIRE_TENSION_N = 10.0
RETURN_SPRING_FORCE_PER_FINGER_N = 1.0


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _capture_meta() -> dict[str, Any]:
    with CAPTURE.open(encoding="utf-8") as stream:
        meta = json.loads(next(stream))
    if meta.get("e") != "meta":
        raise ValueError("capture does not begin with a meta record")
    return meta


def _m0_command_range() -> tuple[float, float]:
    targets = []
    with CAPTURE.open(encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            if row.get("e") == "cmd" and row.get("m") == 0:
                targets.append(float(row["model_target"]))
    if not targets:
        raise ValueError("capture contains no M0 commands")
    return min(targets), max(targets)


def _graph() -> slot_route.PackingSupportGraph:
    return slot_route.PackingSupportGraph.from_report(
        _read_json(PLAN), spec=DEFAULT_STATOR
    )


def _part_mesh(part: Any, linear: float = 0.04,
               angular: float = 0.06) -> trimesh.Trimesh:
    meshes = []
    for solid in list(part.solids()) or [part]:
        vertices, faces = solid.tessellate(linear, angular)
        mesh = trimesh.Trimesh(
            vertices=np.asarray(
                [(vertex.X, vertex.Y, vertex.Z) for vertex in vertices],
                dtype=float,
            ),
            faces=np.asarray(faces),
            process=True,
        )
        if not mesh.is_watertight:
            raise RuntimeError("source stator tessellation is not watertight")
        meshes.append(mesh)
    return trimesh.util.concatenate(meshes)


def _slot_boundaries(radius_mm: float) -> tuple[float, float]:
    """Positive slot steel boundaries from the exact stator source formula."""

    spec = DEFAULT_STATOR
    pitch = 2.0 * math.pi / int(spec.slots)
    shoe_half_angle = 0.36 * pitch
    half_neck = max(2.5, float(spec.od) * 0.07) / 2.0
    shoe_inner = float(spec.od) / 2.0 - max(1.6, float(spec.od) * 0.045)
    active = half_neck
    neighbor = (
        -half_neck + math.sin(pitch) * float(radius_mm)
    ) / math.cos(pitch)
    if radius_mm >= shoe_inner:
        active = max(active, radius_mm * math.tan(shoe_half_angle))
        neighbor = min(
            neighbor,
            radius_mm * math.tan(pitch - shoe_half_angle),
        )
    return float(active), float(neighbor)


def _r3_tangential_half_window(radial_run_mm: float,
                               radius_mm: float) -> float:
    """Max lateral offset of the <=90-degree circular R>=radius connector."""

    run = float(radial_run_mm)
    radius = float(radius_mm)
    if run <= 0.0:
        return -math.inf
    if run >= radius:
        return radius
    return radius - math.sqrt(max(0.0, radius * radius - run * run))


def mouth_corridor(graph: slot_route.PackingSupportGraph) -> dict[str, Any]:
    """Common horn-centre interval at the fixed mouth exit.

    The R3 interval is a necessary planar entry condition, not a sufficient
    complete route proof.  It is intersected with the exact lined steel gap
    for the complete 0.4 mm finger body.
    """

    half_neck = max(2.5, float(DEFAULT_STATOR.od) * 0.07) / 2.0
    r3_intervals = []
    for turn in graph.turns:
        target_y = half_neck + float(turn.profile_radius_mm)
        half_window = _r3_tangential_half_window(
            guide.MOUTH_EXIT_RADIUS_MM - float(turn.radial_mm),
            guide.HORN_WIRE_CENTER_RADIUS_MM,
        )
        r3_intervals.append((
            target_y - half_window,
            target_y + half_window,
            turn.turn_index,
        ))
    steel_lower, steel_upper = _slot_boundaries(
        guide.MOUTH_EXIT_RADIUS_MM
    )
    body_lower = (
        steel_lower + LINER_RECEIVING_MAX_MM
        + guide.GUIDE_TANGENTIAL_THICKNESS_MM / 2.0
    )
    body_upper = (
        steel_upper - LINER_RECEIVING_MAX_MM
        - guide.GUIDE_TANGENTIAL_THICKNESS_MM / 2.0
    )
    lower_row = max(r3_intervals, key=lambda row: row[0])
    upper_row = min(r3_intervals, key=lambda row: row[1])
    lower = max(body_lower, lower_row[0])
    upper = min(body_upper, upper_row[1])
    selected = (lower + upper) / 2.0
    return {
        "mouth_exit_radius_mm": guide.MOUTH_EXIT_RADIUS_MM,
        "exact_steel_boundaries_positive_slot_mm": [
            steel_lower, steel_upper,
        ],
        "liner_receiving_max_mm": LINER_RECEIVING_MAX_MM,
        "finger_thickness_mm": guide.GUIDE_TANGENTIAL_THICKNESS_MM,
        "body_center_interval_mm": [body_lower, body_upper],
        "R3_common_interval_mm": [lower_row[0], upper_row[1]],
        "R3_lower_witness_turn": lower_row[2],
        "R3_upper_witness_turn": upper_row[2],
        "combined_center_interval_mm": [lower, upper],
        "combined_width_mm": upper - lower,
        "selected_center_mm": selected,
        "source_center_mm": guide.GUIDE_TANGENTIAL_CENTER_MM,
        "source_matches_selected": abs(
            selected - guide.GUIDE_TANGENTIAL_CENTER_MM
        ) <= 1e-9,
        "status": "PASS" if upper > lower else "FAIL",
        "scope": (
            "necessary crossing/R3 and lined-body corridor; complete "
            "progressive current-half route is a separate gate"
        ),
    }


def error_budget(corridor: dict[str, Any]) -> dict[str, Any]:
    """Worst-case tangential stack required to manufacture the cam concept."""

    existing_m1_settle_rad = 0.010
    terms = {
        "existing_M1_settle_at_mouth_mm": (
            existing_m1_settle_rad * guide.MOUTH_EXIT_RADIUS_MM
        ),
        "received_stator_tooth_presentation_TIR_limit_mm": 0.030,
        "ground_cam_form_limit_mm": 0.025,
        "preloaded_slide_lateral_play_limit_mm": 0.025,
        "horn_center_grind_limit_mm": 0.020,
        "mount_datum_stack_limit_mm": 0.030,
        "formed_liner_position_limit_mm": 0.030,
        "finger_elastic_deflection_limit_mm": 0.040,
    }
    required_each_side = sum(terms.values())
    available_each_side = float(corridor["combined_width_mm"]) / 2.0
    residual = available_each_side - required_each_side
    return {
        "method": "worst-case arithmetic stack, not RSS",
        "terms_mm": terms,
        "required_each_side_mm": required_each_side,
        "available_each_side_mm": available_each_side,
        "residual_each_side_mm": residual,
        "status": "PASS" if residual >= 0.05 else "FAIL",
        "manufacturing_contract": [
            "ground metal cam or equivalent form-controlled insert",
            "preloaded miniature slide; no printed sliding datum",
            "CMM or optical inspection of horn centre and cam dwell",
            "measure tooth-presentation TIR and formed-liner position before use",
            "load-test finger deflection at the declared horn force",
        ],
    }


def motion_study(meta: dict[str, Any]) -> dict[str, Any]:
    wind_start, wind_end = map(float, meta["m0_wind_range"])
    rotating = float(meta["m1_rotating_position"])
    shaft_park = float(meta["shaft_wrap_contract"]["m0_park_rad"])
    mm_per_rad = float(PARAMS.mm_per_rad)
    final_radius = float(DEFAULT_STATOR.od) / 2.0 + FINAL_WOUND_EXTRA_RADIUS_MM
    extraction_needed = (
        final_radius + RIGID_CLEARANCE_TARGET_MM
        - guide.MOUTH_EXIT_RADIUS_MM
    )
    clear_m0 = wind_end + extraction_needed / mm_per_rad
    tangential_ramp_mm = (rotating - clear_m0) * mm_per_rad
    max_smoothstep_slope = (
        1.5 * TANGENTIAL_PARK_STROKE_MM / tangential_ramp_mm
    )
    pressure_angle = math.degrees(math.atan(max_smoothstep_slope))
    cam_reaction = (
        2.0 * RETURN_SPRING_FORCE_PER_FINGER_N * max_smoothstep_slope
    )
    m0_min, m0_max = _m0_command_range()

    def state(value: float) -> str:
        if wind_start - 1e-9 <= value <= wind_end + 1e-9:
            return "ENGAGED_TRACKING"
        if wind_end < value < clear_m0:
            return "RADIAL_EXTRACTION_LOST_MOTION"
        if clear_m0 <= value < rotating:
            return "TANGENTIAL_RETRACTION"
        return "RETRACTED"

    samples = np.linspace(m0_min, m0_max, 2049)
    states = [state(float(value)) for value in samples]
    index_radial_clearance = (
        guide.MOUTH_EXIT_RADIUS_MM
        + (rotating - wind_end) * mm_per_rad
        - final_radius
    )
    shaft_radial_clearance = (
        guide.MOUTH_EXIT_RADIUS_MM
        + (shaft_park - wind_end) * mm_per_rad
        - final_radius
    )
    return {
        "selected_input": "M0 only; M2 deliberately unused",
        "wind_range_rad": [wind_start, wind_end],
        "full_captured_M0_command_range_rad": [m0_min, m0_max],
        "M1_index_pose_rad": rotating,
        "shaft_wrap_park_rad": shaft_park,
        "radial_tracking_stroke_mm": (
            (wind_end - wind_start) * mm_per_rad
        ),
        "radial_extraction_clear_pose_rad": clear_m0,
        "radial_extraction_before_tangential_motion_mm": extraction_needed,
        "tangential_park_stroke_each_mm": TANGENTIAL_PARK_STROKE_MM,
        "tangential_cam_ramp_available_mm": tangential_ramp_mm,
        "maximum_cam_pressure_angle_deg": pressure_angle,
        "two_finger_return_spring_M0_reaction_N": cam_reaction,
        "index_pose_radial_clearance_to_final_wound_radius_mm": (
            index_radial_clearance
        ),
        "shaft_wrap_pose_radial_clearance_to_final_wound_radius_mm": (
            shaft_radial_clearance
        ),
        "full_range_sample_count": len(samples),
        "full_range_states_seen": sorted(set(states)),
        "state_at_M1_index": state(rotating),
        "state_at_shaft_wrap": state(shaft_park),
        "M0_state_mapping_single_valued": True,
        "M2_memoryless_cam_rejected": {
            "reason": (
                "physical flyer angle repeats on every revolution and cannot "
                "encode turn number or pass phase without a separate geared "
                "counter/latch state"
            ),
            "repetitions_per_tooth_pass": int(DEFAULT_STATOR.turns),
            "selected_architecture_requires_M2": False,
        },
        "status": (
            "PASS" if state(rotating) == "RETRACTED"
            and state(shaft_park) == "RETRACTED"
            and index_radial_clearance >= RIGID_CLEARANCE_TARGET_MM
            and clear_m0 < rotating else "FAIL"
        ),
    }


def force_budget(motion: dict[str, Any]) -> dict[str, Any]:
    horn_resultant = (
        2.0 * DESIGN_WIRE_TENSION_N
        * math.sin(math.radians(90.0) / 2.0)
    )
    return {
        "design_wire_tension_N": DESIGN_WIRE_TENSION_N,
        "maximum_90deg_horn_resultant_N": horn_resultant,
        "required_static_horn_load_test_N": 2.0 * horn_resultant,
        "return_spring_force_per_finger_N": RETURN_SPRING_FORCE_PER_FINGER_N,
        "maximum_cam_pressure_angle_deg": motion[
            "maximum_cam_pressure_angle_deg"
        ],
        "additional_M0_reaction_during_clear_retraction_N": motion[
            "two_finger_return_spring_M0_reaction_N"
        ],
        "current_M0_modeled_axial_force_N": 10.4,
        "status": (
            "PASS" if horn_resultant <= 15.0
            and motion["maximum_cam_pressure_angle_deg"] <= 15.0
            and motion["two_finger_return_spring_M0_reaction_N"] <= 1.0
            else "FAIL"
        ),
        "qualification_required": (
            "cycle a production-wire/horn coupon at 10 N, then measure enamel "
            "damage, horn wear, and loaded finger deflection"
        ),
    }


def _trim_polyline_end(points: np.ndarray, distance_mm: float) -> np.ndarray:
    result = [np.asarray(point, dtype=float).copy() for point in points]
    remaining = float(distance_mm)
    while len(result) >= 2:
        length = float(np.linalg.norm(result[-1] - result[-2]))
        if length > remaining + 1e-12:
            result[-1] += (result[-2] - result[-1]) * (remaining / length)
            return np.asarray(result)
        remaining -= length
        result.pop()
    return np.asarray(result)


def _guide_exit(target: np.ndarray) -> np.ndarray:
    side = 1.0 if float(target[1]) >= 0.0 else -1.0
    face = 1.0 if float(target[2]) >= 0.0 else -1.0
    return np.asarray((
        guide.MOUTH_EXIT_RADIUS_MM,
        side * guide.GUIDE_TANGENTIAL_CENTER_MM,
        face * float(DEFAULT_STATOR.stack) / 2.0,
    ), dtype=float)


def _current_prefix(turn: slot_route.PackingTurn, sign: int,
                    phase_index: int,
                    wire_diameter_mm: float) -> np.ndarray:
    if phase_index <= 0:
        return np.empty((0, 3), dtype=float)
    phases = [
        sign * math.radians(PHASE_STEP_DEG * index)
        for index in range(phase_index + 1)
    ]
    points = np.asarray([
        (
            float(turn.radial_mm),
            *slot_route._rounded_loop_yz(
                turn.profile_radius_mm, phase, DEFAULT_STATOR
            ),
        )
        for phase in phases
    ], dtype=float)
    return _trim_polyline_end(points, wire_diameter_mm)


def _witness(turn: int, sign: int, phase_deg: int, clearance: float,
             start: np.ndarray, target: np.ndarray,
             obstacle: str | None = None) -> dict[str, Any]:
    return {
        "turn_index": int(turn),
        "motion_sign": int(sign),
        "phase_deg": int(phase_deg),
        "clearance_mm": float(clearance),
        "guide_exit_local_mm": list(map(float, start)),
        "target_local_mm": list(map(float, target)),
        "obstacle_id": obstacle,
    }


def route_study(graph: slot_route.PackingSupportGraph) -> dict[str, Any]:
    core = stator_model.stator(DEFAULT_STATOR, label="exact_default_stator")
    core_mesh = _part_mesh(core)
    core_required = LINER_NOMINAL_MM + float(DEFAULT_STATOR.wire_d) / 2.0
    copper_required = float(graph.wire_diameter_mm)
    neighbor_obstacles = (
        slot_route.neighbor_prefill_copper(
            graph, DEFAULT_STATOR, -1, arc_step_deg=PHASE_STEP_DEG
        )
        + slot_route.neighbor_prefill_copper(
            graph, DEFAULT_STATOR, +1, arc_step_deg=PHASE_STEP_DEG
        )
    )

    case_count = core_failures = prior_failures = current_failures = 0
    complete_passes = 0
    current_prefix_applicable = 0
    worst_core = worst_prior = worst_current = None
    minimum_core = minimum_prior = minimum_current = math.inf
    worst_core_route = None

    for turn in graph.turns:
        prior = slot_route.active_copper_before(
            graph, turn.turn_index, DEFAULT_STATOR,
            arc_step_deg=PHASE_STEP_DEG,
        ) + neighbor_obstacles
        prior_field = slot_route.CopperField(prior)
        for sign in (-1, 1):
            for phase_index in range(PHASES_PER_TURN):
                phase_deg = phase_index * PHASE_STEP_DEG
                phase = sign * math.radians(phase_deg)
                yz = slot_route._rounded_loop_yz(
                    turn.profile_radius_mm, phase, DEFAULT_STATOR
                )
                target = np.asarray((turn.radial_mm, *yz), dtype=float)
                start = _guide_exit(target)
                route = np.asarray((start, target), dtype=float)
                case_count += 1

                core_distance, _, _ = slot_route.exact_polyline_mesh_clearance(
                    route, core_mesh, search_band_mm=core_required + 0.05
                )
                core_ok = core_distance >= core_required - 1e-8
                if not core_ok:
                    core_failures += 1
                if core_distance < minimum_core:
                    minimum_core = float(core_distance)
                    worst_core_route = route.copy()
                    worst_core = _witness(
                        turn.turn_index, sign, phase_deg, core_distance,
                        start, target,
                    )

                prior_clearance = prior_field.clearance(
                    route, search_band_mm=copper_required + 0.05
                )
                prior_distance = prior_clearance.minimum_centerline_distance_mm
                prior_ok = prior_distance >= copper_required - 1e-8
                if not prior_ok:
                    prior_failures += 1
                if prior_distance < minimum_prior:
                    minimum_prior = float(prior_distance)
                    worst_prior = _witness(
                        turn.turn_index, sign, phase_deg, prior_distance,
                        start, target, prior_clearance.obstacle_id,
                    )

                current_ok = True
                prefix = _current_prefix(
                    turn, sign, phase_index, graph.wire_diameter_mm
                )
                if len(prefix) >= 2:
                    current_prefix_applicable += 1
                    obstacle = slot_route.CopperPolyline(
                        obstacle_id="current-half-prefix",
                        owner="already_laid_current_half",
                        turn_index=turn.turn_index,
                        centerline_local_mm=tuple(
                            tuple(map(float, point)) for point in prefix
                        ),
                    )
                    current_clearance = slot_route.CopperField(
                        (obstacle,)
                    ).clearance(
                        route, search_band_mm=copper_required + 0.05
                    )
                    current_distance = (
                        current_clearance.minimum_centerline_distance_mm
                    )
                    current_ok = current_distance >= copper_required - 1e-8
                    if not current_ok:
                        current_failures += 1
                    if current_distance < minimum_current:
                        minimum_current = float(current_distance)
                        worst_current = _witness(
                            turn.turn_index, sign, phase_deg,
                            current_distance, start, target,
                            current_clearance.obstacle_id,
                        )

                if core_ok and prior_ok and current_ok:
                    complete_passes += 1

    if worst_core_route is None:
        raise RuntimeError("route sweep produced no core witness")
    exact_brep_worst = slot_route.exact_polyline_part_clearance(
        worst_core_route, core
    )
    expected = int(DEFAULT_STATOR.turns) * 2 * PHASES_PER_TURN
    status = "PASS" if complete_passes == expected else "FAIL"
    return {
        "model": (
            "taut straight free segment from selected mouth horn to the "
            "instantaneous lay point; no hidden inner guide"
        ),
        "failure_scope": (
            "necessary downstream horn-to-lay subpath; flyer-torus-to-horn "
            "coupling is not claimed or evaluated after this subpath fails"
        ),
        "turns": int(DEFAULT_STATOR.turns),
        "motion_signs": [-1, 1],
        "phase_step_deg": PHASE_STEP_DEG,
        "phases_per_turn": PHASES_PER_TURN,
        "expected_case_count": expected,
        "evaluated_case_count": case_count,
        "current_prefix_applicable_case_count": current_prefix_applicable,
        "complete_passing_case_count": complete_passes,
        "core_liner_failure_count": core_failures,
        "prior_and_neighbor_copper_failure_count": prior_failures,
        "already_laid_current_half_failure_count": current_failures,
        "required_core_center_clearance_mm": core_required,
        "required_copper_centerline_clearance_mm": copper_required,
        "minimum_core_center_clearance_mm": minimum_core,
        "exact_BREP_recheck_of_mesh_worst_mm": exact_brep_worst,
        "minimum_prior_copper_centerline_clearance_mm": minimum_prior,
        "minimum_current_prefix_centerline_clearance_mm": minimum_current,
        "worst_core": worst_core,
        "worst_prior_or_neighbor_copper": worst_prior,
        "worst_current_half": worst_current,
        "coverage_complete": case_count == expected,
        "both_motion_signs_covered": True,
        "progressive_turns_covered": len(graph.turns) == 50,
        "status": status,
        "interpretation": (
            "Any rescue needs an additional intentional support/contact path "
            "inside the mouth or a changed winding law.  That is outside the "
            "studied pair of mouth-only passive fingers."
        ),
    }


def build_report() -> dict[str, Any]:
    graph = _graph()
    meta = _capture_meta()
    corridor = mouth_corridor(graph)
    motion = motion_study(meta)
    errors = error_budget(corridor)
    forces = force_budget(motion)
    routes = route_study(graph)
    existing_route = _read_json(EXISTING_ROUTE) if EXISTING_ROUTE.is_file() else {}
    all_motion = (
        corridor["status"] == "PASS"
        and motion["status"] == "PASS"
        and errors["status"] == "PASS"
        and forces["status"] == "PASS"
    )
    status = "PASS" if all_motion and routes["status"] == "PASS" else "DESIGN_NO_GO"
    report = {
        "schema": SCHEMA,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "status": status,
        "release_authorized": False,
        "assembly_integration_authorized": False,
        "scope": {
            "architecture": (
                "M0-tracked mouth head, lost-motion radial extraction, then "
                "spring tangential retraction; no commanded axis"
            ),
            "production_files_modified": False,
            "upstream_serial_protocol_changed": False,
            "stator": "exact default OD46 x stack15 x 24-slot source BREP",
            "liner": "exact default 0.127 mm Nomex 410 analytical offset",
            "wire": f"exact default {DEFAULT_STATOR.wire_d:.5f} mm finished OD",
        },
        "horn_contract": guide.horn_contract(),
        "mouth_corridor": corridor,
        "motion_synchronization": motion,
        "manufacturing_error_budget": errors,
        "force_budget": forces,
        "progressive_current_half_route": routes,
        "existing_route_report_context": {
            "path": str(EXISTING_ROUTE.relative_to(ROOT)),
            "status": existing_route.get("status"),
            "sha256": _sha256(EXISTING_ROUTE) if EXISTING_ROUTE.is_file() else None,
            "not_used_as_substitute_for_this_study": True,
        },
        "gates": {
            "existing_axis_state_unambiguous": motion["status"] == "PASS",
            "mouth_only_geometry": guide.horn_contract()["mouth_only"],
            "guide_R3": guide.horn_contract()["wire_center_R3"],
            "full_M0_range_and_retraction": motion["status"] == "PASS",
            "manufacturable_error_budget": errors["status"] == "PASS",
            "manufacturable_force_budget": forces["status"] == "PASS",
            "all_50_turns_both_signs_current_half": routes["status"] == "PASS",
        },
        "decision": (
            "Do not integrate this passive guide.  M0 alone provides an "
            "unambiguous and mechanically plausible engage/extract/retract "
            "sequence, but the mouth-only pair leaves a taut guide-to-lay "
            "segment that fails the exact lined core and progressive copper "
            "sweeps.  A successor must add a proved intentional inner support "
            "path or change the winding law; merely camming the mouth fingers "
            "cannot close the wire-route gate."
        ),
        "source_hashes": {
            "cad/passive_cam_slot_guide.py": _sha256(
                CAD / "passive_cam_slot_guide.py"
            ),
            "cad/stator_model.py": _sha256(CAD / "stator_model.py"),
            "cad/stator_insulation_nomex410.py": _sha256(
                CAD / "stator_insulation_nomex410.py"
            ),
            "sim/slot_route.py": _sha256(HERE / "slot_route.py"),
            "out/reports/slot_winding_plan.json": _sha256(PLAN),
            "out/reports/slot_packing.json": _sha256(PACKING),
            "out/capture/commands.jsonl": _sha256(CAPTURE),
        },
    }
    canonical = json.dumps(
        report, sort_keys=True, separators=(",", ":"), allow_nan=False
    )
    report["report_sha256"] = hashlib.sha256(canonical.encode()).hexdigest()
    return report


def _markdown(report: dict[str, Any]) -> str:
    route = report["progressive_current_half_route"]
    motion = report["motion_synchronization"]
    error = report["manufacturing_error_budget"]
    return "\n".join((
        "# Passive cam slot guide study",
        "",
        f"**Status: {report['status']}**",
        "",
        report["decision"],
        "",
        "## What passed",
        "",
        "- Existing-axis synchronization: M0-only; no new command or serial change.",
        f"- Radial tracking stroke: {motion['radial_tracking_stroke_mm']:.3f} mm.",
        f"- M1-index radial clearance after extraction: {motion['index_pose_radial_clearance_to_final_wound_radius_mm']:.3f} mm.",
        f"- Horn wire-centre bend radius: {report['horn_contract']['horn_wire_center_radius_mm']:.5f} mm.",
        f"- Worst-case tangential error residual: {error['residual_each_side_mm']:.3f} mm.",
        "",
        "## Decisive failed route gate",
        "",
        f"- Coverage: {route['evaluated_case_count']} / {route['expected_case_count']} cases (50 turns x 2 signs x 72 phases).",
        f"- Lined-core failures: {route['core_liner_failure_count']}.",
        f"- Prior/adjacent-copper failures: {route['prior_and_neighbor_copper_failure_count']}.",
        f"- Already-laid current-half failures: {route['already_laid_current_half_failure_count']}.",
        f"- Complete passing cases: {route['complete_passing_case_count']}.",
        "",
        "The review STEP is intentionally isolated and must not be merged into the production assembly.",
        "",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ))


def write_report() -> dict[str, Any]:
    report = build_report()
    REPORTS.mkdir(parents=True, exist_ok=True)
    json_path = REPORTS / "passive_cam_slot_guide.json"
    md_path = REPORTS / "passive_cam_slot_guide.md"
    json_path.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(_markdown(report), encoding="utf-8")
    return report


if __name__ == "__main__":
    result = write_report()
    route = result["progressive_current_half_route"]
    print(
        f"{result['status']}: {route['complete_passing_case_count']}/"
        f"{route['evaluated_case_count']} complete route cases pass"
    )
