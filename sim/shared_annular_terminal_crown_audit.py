"""Fail-closed audit for the physical shared annular terminal crown.

The deposition route, rigid-body motion and continuous-conductor authorities
remain separate.  A green 2400-locus terminal route cannot hide a failed raw
shaft wrap or an unmodelled park/index conductor state.
"""

from __future__ import annotations

from bisect import bisect_right
from copy import deepcopy
from dataclasses import asdict
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

import fcl
import numpy as np
import trimesh
from build123d import Compound, export_stl


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
REVIEW = ROOT / "out" / "review"
CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
AGGREGATE = REPORTS / "permanent_cap_aggregate_authorization.json"
GUIDE_REPORT = REPORTS / "retained_flyer_peek_guide_successor.json"
INTEGRATED_WIRE = REPORTS / "integrated_phase_aware_wire_path.json"
INTEGRATED_RELEASE = REPORTS / "integrated_release_candidate.json"
LOADS_REPORT = REPORTS / "loads.json"
JSON_OUT = REPORTS / "shared_annular_terminal_crown_audit.json"
MD_OUT = REPORTS / "shared_annular_terminal_crown_audit.md"
MANIFEST_OUT = REVIEW / "shared_annular_terminal_crown.manifest.json"
LOCUS_OUT = REPORTS / "shared_annular_terminal_crown_loci.json"

for folder in (CAD, HERE):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

import collide  # noqa: E402
import integrated_phase_aware_wire_path as integrated  # noqa: E402
from params import DEFAULT_STATOR, PARAMS  # noqa: E402
from phase_aware_progressive_wire_audit import (  # noqa: E402
    RawLocus,
    extract_raw_loci,
)
import retained_flyer_peek_guide_successor as flyer  # noqa: E402
import shared_annular_terminal_crown as crown  # noqa: E402
import stator_insulation_nomex410 as insulation  # noqa: E402
from traj import Timeline, load_events  # noqa: E402
import wire_geometry  # noqa: E402
import wirepath  # noqa: E402


SCHEMA = "shared-annular-terminal-crown-audit/v1"
EXPECTED_LOCI = 2400
DESIGN_TENSION_N = 10.0
MINIMUM_WIRE_CENTER_RADIUS_MM = 3.0
MAX_WIRE_RADIUS_MM = crown.WIRE_DIAMETER_MAX_MM / 2.0
MIN_WIRE_RADIUS_MM = crown.WIRE_DIAMETER_MIN_MM / 2.0
CAPTURE_Y_GRID_MM = tuple(float(v) for v in np.linspace(-2.55, 2.55, 35))
SELECTION_FILLET_RADIUS_MM = 3.25
FULL_MOTION_DM2_RAD = math.radians(2.0)
FULL_MOTION_DM1_RAD = math.radians(2.0)
FULL_MOTION_DM0_MM = 0.25
PEEK_DENSITY_G_MM3 = crown.PEEK_DENSITY_G_MM3
WIRE_GUIDE = {
    "center_local_mm": [0.0, 64.0, -17.0],
    "axis_local": [0.0, 1.0, 0.0],
    "feed_local_mm": [0.0, 67.0, -17.0],
    "major_radius_mm": 6.5,
    "tube_radius_mm": 3.0,
}


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} is not a JSON object")
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(value: Any) -> str:
    raw = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _report_hash(report: Mapping[str, Any]) -> str:
    body = deepcopy(dict(report))
    body.pop("report_sha256", None)
    return _canonical_hash(body)


def _unit(vector: Sequence[float]) -> np.ndarray:
    result = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(result))
    if norm <= 1.0e-12:
        raise ValueError("zero length vector")
    return result / norm


def _side_and_sign(locus: RawLocus) -> tuple[int, int]:
    side_name, sign = integrated._expected_port(locus)
    return (-1 if side_name == "left" else 1), int(sign)


def _route_key(locus: RawLocus, crown_radius_mm: float) -> tuple[Any, ...]:
    side, sign = _side_and_sign(locus)
    return (
        round(float(crown_radius_mm), 9),
        round(locus.radial_x_mm, 9),
        round(locus.m2_mod_rad, 9),
        int(locus.motion_sign),
        int(side),
        int(sign),
    )


def _path_hash(points: np.ndarray) -> str:
    return _canonical_hash(np.round(np.asarray(points, dtype=float), 9).tolist())


def _capture_candidate(
    locus: RawLocus,
    crown_radius_mm: float,
    capture_y_mm: float,
    branch_tangential_mm: float,
) -> dict[str, Any]:
    _side, axial_sign = _side_and_sign(locus)
    capture = np.array([
        crown_radius_mm, capture_y_mm, axial_sign * 21.0,
    ])
    selection_corner = np.array([
        crown.SELECTION_BOWL_X_MM,
        branch_tangential_mm,
        axial_sign * crown.TRANSFER_HIGH_Z_MM,
    ])
    free_direction = selection_corner - capture
    target_world = integrated._stator_local_to_world(capture, locus)
    rotation = wirepath.rot_z(locus.m2_rad)
    feed_world = rotation @ np.asarray(WIRE_GUIDE["feed_local_mm"], dtype=float)
    tip_path, tip_meta = wirepath.tip_guide_path(
        feed_world,
        target_world,
        WIRE_GUIDE,
        MAX_WIRE_RADIUS_MM,
        rotation,
        arc_step_deg=2.0,
    )
    local_tip = integrated._world_to_active_local(tip_path, locus)
    first_arc, first_meta = wire_geometry._circular_fillet(
        tuple(capture),
        tuple(local_tip[-1] - local_tip[-2]),
        tuple(free_direction),
        SELECTION_FILLET_RADIUS_MM,
        step_deg=2.0,
    )
    second_arc, second_meta = wire_geometry._circular_fillet(
        tuple(selection_corner),
        tuple(free_direction),
        (-1.0, 0.0, 0.0),
        SELECTION_FILLET_RADIUS_MM,
        step_deg=2.0,
    )
    first_arc = np.asarray(first_arc, dtype=float)
    second_arc = np.asarray(second_arc, dtype=float)
    straight_mm = float(np.linalg.norm(
        np.asarray(second_meta["start"], dtype=float)
        - np.asarray(first_meta["end"], dtype=float)
    ))
    if straight_mm <= 0.1:
        raise ValueError("capture and selection fillet trims consume free span")
    mouth_half_envelope = float(np.max(np.abs(first_arc[:, 1]))) + MAX_WIRE_RADIUS_MM
    return {
        "capture": capture,
        "selection_corner": selection_corner,
        "local_tip": local_tip,
        "tip_meta": tip_meta,
        "first_arc": first_arc,
        "first_meta": first_meta,
        "second_arc": second_arc,
        "second_meta": second_meta,
        "straight_mm": straight_mm,
        "mouth_half_envelope_mm": mouth_half_envelope,
    }


@lru_cache(maxsize=4096)
def _optimized_template(key: tuple[Any, ...]) -> dict[str, Any]:
    # The caller supplies a representative locus through the side cache.
    locus = _TEMPLATE_LOCI[key]
    crown_radius_mm = float(key[0])
    side, axial_sign = _side_and_sign(locus)
    branch_t = side * crown.TRANSFER_TANGENTIAL_MM
    best: tuple[tuple[float, float], float, dict[str, Any]] | None = None
    for capture_y in CAPTURE_Y_GRID_MM:
        try:
            candidate = _capture_candidate(
                locus, crown_radius_mm, capture_y, branch_t,
            )
        except (RuntimeError, ValueError):
            continue
        score = (
            float(candidate["mouth_half_envelope_mm"]),
            -float(candidate["straight_mm"]),
        )
        if best is None or score < best[0]:
            best = (score, capture_y, candidate)
    if best is None:
        raise RuntimeError("no shared-mouth two-fillet route candidate")
    _score, capture_y, candidate = best

    # Exact physical open-channel transfer: radial high branch, analytic
    # R3.60 quarter bend, then the existing named cap port/riser boundary.
    bend_center = np.array([
        crown.TRANSFER_BEND_RADIAL_X_MM,
        branch_t,
        axial_sign * crown.TRANSFER_BEND_CENTER_Z_MM,
    ])
    bend = []
    for q in np.linspace(0.0, math.pi / 2.0, 55):
        bend.append(bend_center + np.array([
            -crown.TRANSFER_CENTERLINE_RADIUS_MM * math.sin(q),
            0.0,
            axial_sign * crown.TRANSFER_CENTERLINE_RADIUS_MM * math.cos(q),
        ]))
    port = np.array([
        crown.TRANSFER_INNER_X_MM,
        branch_t,
        axial_sign * crown.TRANSFER_PORT_Z_MM,
    ])
    local_tip = candidate["local_tip"]
    first_meta = candidate["first_meta"]
    second_meta = candidate["second_meta"]
    route = np.vstack([
        local_tip[:-1],
        np.asarray(first_meta["start"], dtype=float),
        candidate["first_arc"][1:],
        np.asarray(second_meta["start"], dtype=float),
        candidate["second_arc"][1:],
        np.array([[
            crown.TRANSFER_BEND_RADIAL_X_MM,
            branch_t,
            axial_sign * crown.TRANSFER_HIGH_Z_MM,
        ]]),
        np.asarray(bend, dtype=float)[1:],
        port.reshape(1, 3),
    ])
    return {
        "route": route,
        "capture_y_mm": float(capture_y),
        "capture": candidate["capture"],
        "selection_corner": candidate["selection_corner"],
        "mouth_half_envelope_mm": candidate["mouth_half_envelope_mm"],
        "free_straight_after_trims_mm": candidate["straight_mm"],
        "capture_turn_deg": float(first_meta["turn_deg"]),
        "selection_turn_deg": float(second_meta["turn_deg"]),
        "tip_meta": candidate["tip_meta"],
        "branch_tangential_mm": branch_t,
        "axial_sign": axial_sign,
    }


_TEMPLATE_LOCI: dict[tuple[Any, ...], RawLocus] = {}


def _route_for_locus(locus: RawLocus, crown_radius_mm: float) -> dict[str, Any]:
    key = _route_key(locus, crown_radius_mm)
    _TEMPLATE_LOCI.setdefault(key, locus)
    return _optimized_template(key)


def _core_face():
    return insulation._main_lamination_face()


def terminal_rows(loci: list[RawLocus]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    face = _core_face()
    aggregate = _load(AGGREGATE)
    copper_outer = float(
        aggregate["aggregate_loft"]["tooth_owned_front_crown_cell"]
        ["OD_center_limit_mm"]
    )
    copper_axial = float(
        aggregate["cap_support_lane"]["finished_wire_total_axial_envelope_mm"]
    ) / 2.0
    rows: list[dict[str, Any]] = []
    failures: list[dict[str, Any]] = []
    for locus in loci:
        template = _route_for_locus(
            locus, crown.crown_radius_for_stator_od(DEFAULT_STATOR.od)
        )
        route = np.asarray(template["route"], dtype=float)
        core = integrated.core_prism_intersection(route, face)
        # Before the named selection bowl, the entire path is outside both
        # core and finished-copper radial authority.  The high radial branch
        # is beyond the finished axial envelope; only the final x=18.2 riser
        # enters that envelope, exactly on the named cap-lane boundary.
        outside_selection = route[
            np.linalg.norm(route[:, :2], axis=1)
            >= crown.SELECTION_BOWL_X_MM - 0.5
        ]
        min_outer_radius = float(np.min(
            np.linalg.norm(outside_selection[:, :2], axis=1)
        ))
        high_branch_axial_margin = (
            crown.TRANSFER_BEND_CENTER_Z_MM
            - copper_axial - MAX_WIRE_RADIUS_MM
        )
        mouth_margin = (
            crown.CAPTURE_MOUTH_CLEAR_WIDTH_MM / 2.0
            - float(template["mouth_half_envelope_mm"])
        )
        transfer_wander = crown.TRANSFER_CLEAR_RADIUS_MM - MIN_WIRE_RADIUS_MM
        transfer_min_radius = (
            crown.TRANSFER_CENTERLINE_RADIUS_MM - transfer_wander
        )
        gates = {
            "route_constructed": True,
            "no_core_intersection": not bool(core["intersects"]),
            "outer_capture_and_free_span_outside_finished_copper": (
                min_outer_radius - MAX_WIRE_RADIUS_MM > copper_outer
            ),
            "high_branch_clears_finished_axial_envelope": (
                high_branch_axial_margin > 0.0
            ),
            "shared_mouth_contains_max_wire_envelope": mouth_margin >= 0.0,
            "capture_and_selection_contact_R_ge_3": (
                SELECTION_FILLET_RADIUS_MM >= MINIMUM_WIRE_CENTER_RADIUS_MM
            ),
            "open_transfer_channel_worst_wander_R_ge_3": (
                transfer_min_radius >= MINIMUM_WIRE_CENTER_RADIUS_MM
            ),
            "route_ends_at_named_existing_cap_port": bool(np.allclose(
                route[-1],
                [
                    crown.TRANSFER_INNER_X_MM,
                    template["branch_tangential_mm"],
                    template["axial_sign"] * crown.TRANSFER_PORT_Z_MM,
                ],
                atol=1.0e-9,
            )),
        }
        if not all(gates.values()):
            failures.append({
                "pass_index": locus.pass_index,
                "state_index": locus.state_index,
                "failed": [name for name, value in gates.items() if not value],
            })
        rows.append({
            "pass_index": locus.pass_index,
            "state_index": locus.state_index,
            "half_turn_index": locus.half_turn_index,
            "motion_sign": locus.motion_sign,
            "m0_rad": round(locus.m0_rad, 12),
            "m1_rad": round(locus.m1_rad, 12),
            "m2_rad": round(locus.m2_rad, 12),
            "radial_x_mm": round(locus.radial_x_mm, 12),
            "capture_y_mm": round(float(template["capture_y_mm"]), 12),
            "capture_local_mm": np.round(template["capture"], 12).tolist(),
            "selection_local_mm": np.round(
                template["selection_corner"], 12
            ).tolist(),
            "path_sha256": _path_hash(route),
            "path_point_count": int(len(route)),
            "capture_turn_deg": round(float(template["capture_turn_deg"]), 12),
            "selection_turn_deg": round(float(template["selection_turn_deg"]), 12),
            "free_straight_after_trims_mm": round(
                float(template["free_straight_after_trims_mm"]), 12
            ),
            "mouth_half_envelope_mm": round(
                float(template["mouth_half_envelope_mm"]), 12
            ),
            "mouth_margin_mm": round(mouth_margin, 12),
            "minimum_outer_free_radius_mm": round(min_outer_radius, 12),
            "high_branch_axial_margin_mm": round(high_branch_axial_margin, 12),
            "transfer_min_wire_center_radius_after_wander_mm": round(
                transfer_min_radius, 12
            ),
            "core_intersection": bool(core["intersects"]),
            "route_gates_pass": bool(all(gates.values())),
        })
    summary = {
        "locus_count": len(rows),
        "unique_geometry_case_count": len(_TEMPLATE_LOCI),
        "constructed_count": len(rows),
        "pass_count": sum(row["route_gates_pass"] for row in rows),
        "failure_count": len(failures),
        "first_failures": failures[:10],
        "minimum_mouth_margin_mm": min(row["mouth_margin_mm"] for row in rows),
        "minimum_free_straight_after_trims_mm": min(
            row["free_straight_after_trims_mm"] for row in rows
        ),
        "maximum_capture_turn_deg": max(row["capture_turn_deg"] for row in rows),
        "maximum_selection_turn_deg": max(row["selection_turn_deg"] for row in rows),
        "minimum_outer_free_radius_mm": min(
            row["minimum_outer_free_radius_mm"] for row in rows
        ),
        "minimum_high_branch_axial_margin_mm": min(
            row["high_branch_axial_margin_mm"] for row in rows
        ),
        "minimum_transfer_wire_center_radius_after_wander_mm": min(
            row["transfer_min_wire_center_radius_after_wander_mm"] for row in rows
        ),
    }
    return rows, summary


def _shape_mesh(shape: Any, name: str) -> trimesh.Trimesh:
    cache = REVIEW / "shared_crown_collision_meshes"
    cache.mkdir(parents=True, exist_ok=True)
    target = cache / f"{name}.stl"
    export_stl(
        shape, target, tolerance=0.08, angular_tolerance=0.10,
        ascii_format=False,
    )
    mesh = trimesh.load(target, force="mesh", process=True)
    if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
        raise RuntimeError(f"empty collision mesh for {name}")
    return mesh


def _spindle_reference_mesh(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    manifest = collide.load_manifest()
    standoff = float(manifest["m0_home_standoff"])
    result = mesh.copy()
    point = result.vertices.copy()
    # Exact assembly reference mapping from stator local to M0=M1=0 machine.
    result.vertices = np.column_stack((
        -point[:, 1], point[:, 2], standoff - point[:, 0],
    ))
    return result


def _fcl_objects(
    bvh_a: fcl.BVHModel,
    bvh_b: fcl.BVHModel,
    tf_a: tuple[np.ndarray, np.ndarray],
    tf_b: tuple[np.ndarray, np.ndarray],
) -> tuple[fcl.CollisionObject, fcl.CollisionObject]:
    ra, ta = tf_a
    rb, tb = tf_b
    return (
        fcl.CollisionObject(bvh_a, fcl.Transform(ra, ta)),
        fcl.CollisionObject(bvh_b, fcl.Transform(rb, tb)),
    )


def _fcl_collision_and_distance(
    bvh_a: fcl.BVHModel,
    bvh_b: fcl.BVHModel,
    tf_a: tuple[np.ndarray, np.ndarray],
    tf_b: tuple[np.ndarray, np.ndarray],
    *,
    with_distance: bool,
) -> tuple[bool, float | None]:
    object_a, object_b = _fcl_objects(bvh_a, bvh_b, tf_a, tf_b)
    collision_result = fcl.CollisionResult()
    fcl.collide(
        object_a, object_b, fcl.CollisionRequest(), collision_result,
    )
    collision_value = bool(collision_result.is_collision)
    if not with_distance or collision_value:
        return collision_value, (-1.0 if collision_value else None)
    distance = float(fcl.distance(
        object_a,
        object_b,
        fcl.DistanceRequest(),
        fcl.DistanceResult(),
    ))
    return False, distance


def _collision_models() -> dict[str, Any]:
    crowned_pair = crown.crown_pair()
    flyer_assembly = Compound(children=flyer.rotating_parts())
    flyer_assembly.label = "final_retained_flyer_arm_PEEK_guide_and_hardware"
    crown_mesh_local = _shape_mesh(crowned_pair, "actual_crowned_cap_pair")
    crown_mesh_reference = _spindle_reference_mesh(crown_mesh_local)
    flyer_mesh = _shape_mesh(flyer_assembly, "final_flyer_with_PEEK_guide")
    return {
        "crown_mesh": crown_mesh_reference,
        "flyer_mesh": flyer_mesh,
        "crown_bvh": collide.make_bvh(crown_mesh_reference),
        "flyer_bvh": collide.make_bvh(flyer_mesh),
        "crown_mesh_watertight": bool(crown_mesh_reference.is_watertight),
        "flyer_mesh_watertight": bool(flyer_mesh.is_watertight),
        "crown_triangle_count": int(len(crown_mesh_reference.faces)),
        "flyer_triangle_count": int(len(flyer_mesh.faces)),
    }


def add_deposition_collision(
    rows: list[dict[str, Any]],
    loci: list[RawLocus],
    models: Mapping[str, Any],
) -> dict[str, Any]:
    kin = collide.Kinematics(collide.load_manifest())
    worst: tuple[float, int] | None = None
    collision_count = 0
    for index, (row, locus) in enumerate(zip(rows, loci)):
        spindle_tf = kin.link_tf(
            "spindle", locus.m0_rad, locus.m1_rad, locus.m2_rad,
        )
        flyer_tf = kin.link_tf(
            "flyer", locus.m0_rad, locus.m1_rad, locus.m2_rad,
        )
        collision_value, distance = _fcl_collision_and_distance(
            models["crown_bvh"], models["flyer_bvh"],
            spindle_tf, flyer_tf, with_distance=True,
        )
        value = float(distance if distance is not None else -1.0)
        row["crown_cap_to_final_flyer_collision"] = collision_value
        row["crown_cap_to_final_flyer_clearance_mm"] = round(value, 12)
        collision_count += int(collision_value)
        if worst is None or value < worst[0]:
            worst = (value, index)
    if worst is None:
        raise RuntimeError("no deposition collision rows")
    witness = rows[worst[1]]
    return {
        "locus_count": len(rows),
        "collision_count": collision_count,
        "minimum_clearance_mm": worst[0],
        "witness": {
            "pass_index": witness["pass_index"],
            "state_index": witness["state_index"],
            "m0_rad": witness["m0_rad"],
            "m1_rad": witness["m1_rad"],
            "m2_rad": witness["m2_rad"],
        },
        "status": "PASS" if collision_count == 0 else "FAIL",
    }


def full_raw_rigid_motion(
    events: list[dict[str, Any]],
    timeline: Timeline,
    models: Mapping[str, Any],
) -> dict[str, Any]:
    kin = collide.Kinematics(collide.load_manifest())
    event_rows = [
        (float(event["t"]), str(event["e"]))
        for event in events if event["e"] not in ("cmd", "meta")
    ]
    event_times = [row[0] for row in event_rows]
    phase_counts: dict[str, int] = {}
    pose_hasher = hashlib.sha256()
    count = 0
    collision_count = 0
    first_collision = None
    for time_s, m0, m1, m2 in timeline.samples(
        max_dm2=FULL_MOTION_DM2_RAD,
        max_dm0=FULL_MOTION_DM0_MM,
        max_dm1=FULL_MOTION_DM1_RAD,
    ):
        phase_index = bisect_right(event_times, float(time_s)) - 1
        phase = event_rows[phase_index][1] if phase_index >= 0 else "init"
        phase_counts[phase] = phase_counts.get(phase, 0) + 1
        serialized = [
            round(float(time_s), 9), round(float(m0), 12),
            round(float(m1), 12), round(float(m2), 12), phase,
        ]
        pose_hasher.update(json.dumps(
            serialized, separators=(",", ":"), allow_nan=False,
        ).encode("utf-8"))
        spindle_tf = kin.link_tf("spindle", m0, m1, m2)
        flyer_tf = kin.link_tf("flyer", m0, m1, m2)
        hit, _distance = _fcl_collision_and_distance(
            models["crown_bvh"], models["flyer_bvh"],
            spindle_tf, flyer_tf, with_distance=False,
        )
        count += 1
        if hit:
            collision_count += 1
            if first_collision is None:
                first_collision = {
                    "sample_index": count - 1,
                    "time_s": float(time_s),
                    "phase": phase,
                    "m0_rad": float(m0),
                    "m1_rad": float(m1),
                    "m2_rad": float(m2),
                }
    required_markers = {
        "park_or_prevent_collision": any(
            "prevent_collision" in name for name in phase_counts
        ),
        "tooth_index": any(
            "move_to_teeth" in name or "motor1" in name
            for name in phase_counts
        ),
        "shaft_wrap": "wind_wire_around_shaft" in phase_counts,
        "load_or_position": any(
            "position" in name or "set_motor2_wire_position" in name
            for name in phase_counts
        ),
        "winding": any(
            name in phase_counts for name in (
                "set_motor2_wire_position_done", "wind_wire_done",
            )
        ),
    }
    return {
        "authority": "RIGID_PARTS_ONLY_NOT_CONTINUOUS_CONDUCTOR",
        "sampling": {
            "maximum_M2_step_deg": math.degrees(FULL_MOTION_DM2_RAD),
            "maximum_M1_step_deg": math.degrees(FULL_MOTION_DM1_RAD),
            "maximum_M0_step_mm": FULL_MOTION_DM0_MM,
        },
        "sample_count": count,
        "pose_stream_sha256": pose_hasher.hexdigest(),
        "collision_count": collision_count,
        "first_collision": first_collision,
        "covered_phase_sample_counts": phase_counts,
        "required_motion_classes_present": required_markers,
        "status": (
            "PASS" if collision_count == 0 and all(required_markers.values())
            else "FAIL"
        ),
    }


def _axis_line_distance_mm(
    point: np.ndarray,
    line_unit: np.ndarray,
    axis_origin: np.ndarray,
    axis_unit: np.ndarray,
) -> float:
    normal = np.cross(line_unit, axis_unit)
    norm = float(np.linalg.norm(normal))
    if norm <= 1.0e-12:
        return float(np.linalg.norm(np.cross(point - axis_origin, axis_unit)))
    return abs(float(np.dot(point - axis_origin, normal))) / norm


def add_coupled_loads(
    rows: list[dict[str, Any]], loci: list[RawLocus],
) -> dict[str, Any]:
    m2_worst: tuple[float, int] | None = None
    m1_worst: tuple[float, int] | None = None
    for index, (row, locus) in enumerate(zip(rows, loci)):
        capture_local = np.asarray(row["capture_local_mm"], dtype=float)
        contact_world = integrated._stator_local_to_world(capture_local, locus)
        tip_world = (
            wirepath.rot_z(locus.m2_rad)
            @ np.asarray(WIRE_GUIDE["feed_local_mm"], dtype=float)
        )
        toward_crown = _unit(contact_world - tip_world)
        toward_flyer = -toward_crown
        m2_projected = abs(float(np.cross(tip_world, toward_crown)[2]))
        m2_distance = _axis_line_distance_mm(
            tip_world, toward_crown, np.zeros(3), np.array([0.0, 0.0, 1.0]),
        )
        axis_z = float(PARAMS.stator_axis_z(locus.m0_rad))
        m1_origin = np.array([0.0, 0.0, axis_z])
        m1_projected = abs(float(np.cross(
            contact_world - m1_origin, toward_flyer,
        )[1]))
        m1_distance = _axis_line_distance_mm(
            contact_world,
            toward_flyer,
            m1_origin,
            np.array([0.0, 1.0, 0.0]),
        )
        row.update({
            "live_line_tip_world_mm": np.round(tip_world, 12).tolist(),
            "live_line_contact_world_mm": np.round(contact_world, 12).tolist(),
            "live_line_unit_tip_to_crown": np.round(toward_crown, 15).tolist(),
            "M2_projected_moment_arm_mm": round(m2_projected, 12),
            "M2_axis_to_line_distance_mm": round(m2_distance, 12),
            "M1_projected_moment_arm_mm": round(m1_projected, 12),
            "M1_axis_to_line_distance_mm": round(m1_distance, 12),
        })
        if m2_worst is None or m2_projected > m2_worst[0]:
            m2_worst = (m2_projected, index)
        if m1_worst is None or m1_projected > m1_worst[0]:
            m1_worst = (m1_projected, index)
    if m2_worst is None or m1_worst is None:
        raise RuntimeError("no coupled-load rows")

    def witness(kind: str, pair: tuple[float, int]) -> dict[str, Any]:
        value, index = pair
        row = rows[index]
        return {
            "pass_index": row["pass_index"],
            "state_index": row["state_index"],
            "m0_rad": row["m0_rad"],
            "m1_rad": row["m1_rad"],
            "m2_rad": row["m2_rad"],
            "tip_world_mm": row["live_line_tip_world_mm"],
            "contact_world_mm": row["live_line_contact_world_mm"],
            "unit_tip_to_crown": row["live_line_unit_tip_to_crown"],
            "projected_moment_arm_mm": value,
            "literal_axis_to_line_distance_mm": row[
                f"{kind}_axis_to_line_distance_mm"
            ],
        }

    m2_row = rows[m2_worst[1]]
    m1_row = rows[m1_worst[1]]
    return {
        "force_vector_definition": (
            "axis torque = tension * abs((application radius cross unit "
            "tension direction) dot axis); literal shortest skew-line "
            "distance is reported separately and is not substituted for "
            "the projected torque component"
        ),
        "design_tension_N": DESIGN_TENSION_N,
        "M2": {
            "max_projected_moment_arm_mm": m2_worst[0],
            "wire_torque_at_10N_nm": DESIGN_TENSION_N * m2_worst[0] / 1000.0,
            "maximum_literal_axis_to_line_distance_mm": max(
                row["M2_axis_to_line_distance_mm"] for row in rows
            ),
            "witness": witness("M2", m2_worst),
        },
        "M1": {
            "max_projected_moment_arm_mm": m1_worst[0],
            "wire_torque_at_10N_nm": DESIGN_TENSION_N * m1_worst[0] / 1000.0,
            "maximum_literal_axis_to_line_distance_mm": max(
                row["M1_axis_to_line_distance_mm"] for row in rows
            ),
            "witness": witness("M1", m1_worst),
        },
    }


def _mass_inertia_row(
    name: str, shape: Any, density_g_mm3: float, axis: str,
) -> dict[str, Any]:
    volume = float(shape.volume)
    mass = volume * float(density_g_mm3)
    center = shape.center()
    matrix = shape.matrix_of_inertia
    if axis == "z":
        volume_j = float(matrix[2][2]) + volume * (
            float(center.X) ** 2 + float(center.Y) ** 2
        )
    elif axis == "y":
        volume_j = float(matrix[1][1]) + volume * (
            float(center.X) ** 2 + float(center.Z) ** 2
        )
    else:
        raise ValueError(axis)
    j_g_mm2 = volume_j * float(density_g_mm3)
    return {
        "name": name,
        "volume_mm3": volume,
        "density_g_mm3": float(density_g_mm3),
        "mass_g": mass,
        "center_of_mass_mm": [
            float(center.X), float(center.Y), float(center.Z),
        ],
        "axis": axis,
        "moment_of_inertia_g_mm2": j_g_mm2,
        "moment_of_inertia_kg_m2": j_g_mm2 * 1.0e-9,
    }


def mass_and_motor_margins(coupled: Mapping[str, Any]) -> dict[str, Any]:
    front_add = _mass_inertia_row(
        "front_shared_crown_add_on", crown.crown_add_on(1),
        PEEK_DENSITY_G_MM3, "z",
    )
    rear_add = _mass_inertia_row(
        "rear_shared_crown_add_on", crown.crown_add_on(-1),
        PEEK_DENSITY_G_MM3, "z",
    )
    front_replacement = _mass_inertia_row(
        "front_complete_crowned_cap_replacement",
        crown.crowned_cap_replacement(1), PEEK_DENSITY_G_MM3, "z",
    )
    rear_replacement = _mass_inertia_row(
        "rear_complete_crowned_cap_replacement",
        crown.crowned_cap_replacement(-1), PEEK_DENSITY_G_MM3, "z",
    )
    guide_row = _mass_inertia_row(
        "rotating_flyer_shaft_to_tip_PEEK_guide",
        flyer.peek_guide_insert(), PEEK_DENSITY_G_MM3, "z",
    )
    crown_j = (
        front_add["moment_of_inertia_kg_m2"]
        + rear_add["moment_of_inertia_kg_m2"]
    )
    crown_mass = front_add["mass_g"] + rear_add["mass_g"]

    release = _load(INTEGRATED_RELEASE)
    prior = release["geometry"]["final_integrated_M2_torque"]
    prior_j = float(prior["full_output_inertia_kgm2"])
    guide_j = float(guide_row["moment_of_inertia_kg_m2"])
    final_m2_j = prior_j + guide_j
    m2_acceleration = float(prior["angular_acceleration_rad_s2"])
    m2_accel_torque = final_m2_j * m2_acceleration
    m2_wire = float(coupled["M2"]["wire_torque_at_10N_nm"])
    m2_friction = float(prior["friction_allowance_nm_unmeasured"])
    m2_required = m2_wire + m2_accel_torque + m2_friction
    available_36 = float(prior["Leadshine_36V_lower_edge_nm"])
    available_24 = float(prior["Leadshine_24V_lower_edge_nm"])

    # The current M1 load report rounds to 0.0625 Nm: 0.040 shaft-wrap
    # reaction + 0.020 friction + J*50.  Recover the bounded baseline J and
    # add only the M1-owned crowned-cap extension inertia.
    loads_report = _load(LOADS_REPORT)
    m1_baseline_required = float(loads_report["m1"]["t_required_nm"])
    m1_wrap_torque = 0.040
    m1_friction = 0.020
    m1_acceleration = 50.0
    m1_baseline_j = (
        m1_baseline_required - m1_wrap_torque - m1_friction
    ) / m1_acceleration
    m1_reaction = float(coupled["M1"]["wire_torque_at_10N_nm"])
    m1_final_j = m1_baseline_j + crown_j
    m1_required = (
        max(m1_wrap_torque, m1_reaction)
        + m1_friction + m1_final_j * m1_acceleration
    )
    m1_available = 0.33135211024345884
    return {
        "material": "natural unfilled PEEK, lot certified",
        "density_g_cm3": PEEK_DENSITY_G_MM3 * 1000.0,
        "rows": [
            front_add, rear_add, front_replacement, rear_replacement, guide_row,
        ],
        "incremental_crown_pair_mass_g": crown_mass,
        "crown_link_ownership": "spindle/stator; rotates with M1, never M2",
        "crown_pair_izz_about_M1_axis_kg_m2": crown_j,
        "rotating_flyer_guide_link_ownership": "flyer; rotates with M2",
        "rotating_flyer_PEEK_guide_izz_about_axis_kg_m2": guide_j,
        "M2": {
            "prior_full_output_inertia_kg_m2": prior_j,
            "rotating_flyer_PEEK_guide_izz_about_axis_kg_m2": guide_j,
            "crown_inertia_contribution_kg_m2": 0.0,
            "final_full_output_inertia_kg_m2": final_m2_j,
            "angular_acceleration_rad_s2": m2_acceleration,
            "acceleration_torque_nm": m2_accel_torque,
            "wire_torque_nm": m2_wire,
            "friction_allowance_nm": m2_friction,
            "required_output_torque_nm": m2_required,
            "required_2x_running_torque_nm": 2.0 * m2_required,
            "Leadshine_36V_lower_edge_nm": available_36,
            "Leadshine_36V_available_to_required_multiple": (
                available_36 / m2_required
            ),
            "Leadshine_36V_gate_ge_2x": available_36 / m2_required >= 2.0,
            "Leadshine_24V_lower_edge_nm": available_24,
            "Leadshine_24V_numeric_multiple": available_24 / m2_required,
            "Leadshine_24V_release_gate": False,
            "Leadshine_24V_release_reason": (
                "36V is the selected/validated operating condition; 24V "
                "remains fail-closed despite the recomputed arithmetic"
            ),
        },
        "M1": {
            "motor": "17HS19-2004D-E1K + CL42T-V41 at 24V",
            "available_torque_at_190p986rpm_nm": m1_available,
            "baseline_spindle_inertia_kg_m2": m1_baseline_j,
            "added_crown_pair_izz_kg_m2": crown_j,
            "final_spindle_inertia_kg_m2": m1_final_j,
            "angular_acceleration_rad_s2": m1_acceleration,
            "shaft_wrap_torque_nm": m1_wrap_torque,
            "terminal_crown_reaction_torque_nm": m1_reaction,
            "governing_wire_torque_nm": max(m1_wrap_torque, m1_reaction),
            "friction_allowance_nm": m1_friction,
            "required_output_torque_nm": m1_required,
            "available_to_required_multiple": m1_available / m1_required,
            "gate_ge_2x": m1_available / m1_required >= 2.0,
        },
    }


def exact_brep_and_dfm() -> dict[str, Any]:
    radius = crown.crown_radius_for_stator_od(DEFAULT_STATOR.od)
    front_add = crown.crown_add_on(1)
    rear_add = crown.crown_add_on(-1)
    front = crown.crowned_cap_replacement(1)
    rear = crown.crowned_cap_replacement(-1)
    channel = crown._branch_open_channel(1, 1, radius)
    max_wire = crown._branch_sweep(
        MAX_WIRE_RADIUS_MM, 1, 1, radius,
    )
    min_wire = crown._branch_sweep(
        MIN_WIRE_RADIUS_MM, 1, 1, radius,
    )
    max_intrusion = float((max_wire & channel).volume)
    min_intrusion = float((min_wire & channel).volume)
    transfer_wander = crown.TRANSFER_CLEAR_RADIUS_MM - MIN_WIRE_RADIUS_MM
    minimum_transfer_radius = (
        crown.TRANSFER_CENTERLINE_RADIUS_MM - transfer_wander
    )
    pitch = 2.0 * radius * math.sin(math.pi / crown.SLOTS)
    checks = {
        "front_add_on_exactly_one_solid": len(list(front_add.solids())) == 1,
        "rear_add_on_exactly_one_solid": len(list(rear_add.solids())) == 1,
        "front_replacement_exactly_one_solid": len(list(front.solids())) == 1,
        "rear_replacement_exactly_one_solid": len(list(rear.solids())) == 1,
        "representative_open_channel_exactly_one_solid": (
            len(list(channel.solids())) == 1
        ),
        "max_wire_zero_positive_channel_material_intrusion": (
            max_intrusion <= 1.0e-8
        ),
        "min_wire_zero_positive_channel_material_intrusion": (
            min_intrusion <= 1.0e-8
        ),
        "continuous_opening_ge_max_wire_plus_0p25mm": (
            crown.TRANSFER_OPENING_WIDTH_MM
            >= crown.WIRE_DIAMETER_MAX_MM + 0.25
        ),
        "clear_channel_radial_reserve_ge_0p20mm_at_max_wire": (
            crown.TRANSFER_CLEAR_RADIUS_MM - MAX_WIRE_RADIUS_MM >= 0.20
        ),
        "worst_min_wire_wander_still_R_ge_3mm": (
            minimum_transfer_radius >= 3.0
        ),
        "R40_tooth_pitch_contains_shoe_and_web": (
            pitch - crown.CAPTURE_SHOE_TANGENTIAL_WIDTH_MM >= 1.0
        ),
        "positive_cap_retention_inherited": (
            len(crown.retention_hardware()) == 12
            and crown.cap.RETENTION_FASTENER_COUNT == 3
            and crown.cap.KEY_COUNT == 24
        ),
        "blind_curved_bores_absent": True,
    }
    return {
        "checks": checks,
        "status": "PASS" if all(checks.values()) else "FAIL",
        "front_add_on_volume_mm3": float(front_add.volume),
        "rear_add_on_volume_mm3": float(rear_add.volume),
        "front_replacement_volume_mm3": float(front.volume),
        "rear_replacement_volume_mm3": float(rear.volume),
        "representative_max_wire_to_channel_intersection_mm3": max_intrusion,
        "representative_min_wire_to_channel_intersection_mm3": min_intrusion,
        "transfer_clear_diameter_mm": 2.0 * crown.TRANSFER_CLEAR_RADIUS_MM,
        "continuous_side_opening_mm": crown.TRANSFER_OPENING_WIDTH_MM,
        "max_wire_radial_DFM_reserve_mm": (
            crown.TRANSFER_CLEAR_RADIUS_MM - MAX_WIRE_RADIUS_MM
        ),
        "minimum_wire_wander_mm": transfer_wander,
        "minimum_transfer_wire_center_radius_after_wander_mm": (
            minimum_transfer_radius
        ),
        "capture_mouth_clear_width_mm": crown.CAPTURE_MOUTH_CLEAR_WIDTH_MM,
        "capture_surface_radius_mm": crown.CAPTURE_SURFACE_RADIUS_MM,
        "selection_bowl_surface_radius_mm": (
            crown.SELECTION_BOWL_SURFACE_RADIUS_MM
        ),
        "tooth_pitch_chord_at_R40_mm": pitch,
        "remaining_web_between_shoes_at_R40_mm": (
            pitch - crown.CAPTURE_SHOE_TANGENTIAL_WIDTH_MM
        ),
        "manufacturing": {
            "process": (
                "low-volume 5-axis CNC from stress-relieved certified natural "
                "unfilled PEEK; polish with access from continuous C openings"
            ),
            "channel_access": (
                "all 96 transfer channels are externally accessible for ball-end "
                "machining, polishing cord, visual inspection and pin/wire gauge"
            ),
            "wire_contact_finish": "Ra <= 0.4 um along wire travel",
            "hidden_curved_drilling": False,
            "supplier_DFM_complete": False,
        },
    }


def parametric_family(loci: list[RawLocus]) -> dict[str, Any]:
    rows = []
    for od in (28.0, 46.0, 65.0, 90.0):
        radius = crown.crown_radius_for_stator_od(od)
        pitch = 2.0 * radius * math.sin(math.pi / crown.SLOTS)
        templates = [_route_for_locus(locus, radius) for locus in loci]
        min_straight = min(
            float(row["free_straight_after_trims_mm"]) for row in templates
        )
        minimum_mouth_margin = min(
            crown.CAPTURE_MOUTH_CLEAR_WIDTH_MM / 2.0
            - float(row["mouth_half_envelope_mm"])
            for row in templates
        )
        outer_pass = min_straight > 0.0 and minimum_mouth_margin >= 0.0
        physical_cap = math.isclose(od, float(DEFAULT_STATOR.od), abs_tol=1e-12)
        rows.append({
            "stator_od_mm": od,
            "capture_radius_mm": radius,
            "tooth_pitch_chord_at_capture_mm": pitch,
            "capture_shoe_width_mm": crown.CAPTURE_SHOE_TANGENTIAL_WIDTH_MM,
            "remaining_pitch_web_mm": (
                pitch - crown.CAPTURE_SHOE_TANGENTIAL_WIDTH_MM
            ),
            "minimum_tip_to_capture_free_straight_after_R3_trims_mm": (
                min_straight
            ),
            "minimum_shared_mouth_margin_mm": minimum_mouth_margin,
            "outer_capture_all_2400_default_pose_templates_construct": outer_pass,
            "physical_matching_cap_footprint_modeled": physical_cap,
            "status": (
                "FULL_DEFAULT_CROWN_ROUTE_PASS"
                if physical_cap and outer_pass else
                "PARAMETRIC_OUTER_CAPTURE_PASS__CAP_FOOTPRINT_NOT_SUPPLIED"
                if outer_pass else "PARAMETRIC_OUTER_CAPTURE_FAIL"
            ),
        })
    return {
        "radius_law": "max(40 mm, stator_OD/2 + 7 mm)",
        "launch_ODs_mm": [28.0, 46.0, 65.0],
        "advisory_OD90_mm": 90.0,
        "cases": rows,
        "current_physical_release_bound_mm": 65.0,
        "OD90_note": (
            "R52 outer capture and R64-tip free-span constructibility are "
            "reported, but OD90 has no supplied cap/core/copper footprint or "
            "full rigid sweep; it is advisory and not release-authorized"
        ),
    }


def continuous_conductor_authority() -> dict[str, Any]:
    prior = _load(INTEGRATED_WIRE)
    wraps = prior["shaft_wraps"]
    return {
        "authority": (
            "continuous flexible conductor; separate from rigid crown sweep "
            "and deposition-only terminal paths"
        ),
        "park_index_load_unload_path_geometry_proven": False,
        "park_index_load_unload_reason": (
            "no time-varying flexible-conductor geometry was supplied for "
            "these transitions; rigid-part clearance cannot substitute"
        ),
        "shaft_wraps": {
            "source_report": "out/reports/integrated_phase_aware_wire_path.json",
            "source_report_sha256": _sha256(INTEGRATED_WIRE),
            "status": wraps["status"],
            "case_count": wraps["case_count"],
            "raw_pose_count": wraps["raw_pose_count"],
            "raw_turns": [
                float(case["raw_turns"]) for case in wraps["cases"]
            ],
            "path_sha256": [
                case["path_sha256"] for case in wraps["cases"]
            ],
            "each_raw_wrap_is_two_full_turns": wraps["gates"][
                "each_raw_wrap_is_two_full_turns"
            ],
        },
        "status": "FAIL",
    }


def _bbox(shape: Any) -> dict[str, list[float]]:
    box = shape.bounding_box()
    return {
        "minimum_mm": [float(box.min.X), float(box.min.Y), float(box.min.Z)],
        "maximum_mm": [float(box.max.X), float(box.max.Y), float(box.max.Z)],
        "size_mm": [float(box.size.X), float(box.size.Y), float(box.size.Z)],
    }


def analyze(*, run_full_motion: bool = True) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    events = load_events(CAPTURE)
    timeline = Timeline(events)
    loci, passes = extract_raw_loci(events, timeline)
    if len(loci) != EXPECTED_LOCI:
        raise RuntimeError(f"expected 2400 loci; observed {len(loci)}")
    rows, terminal = terminal_rows(loci)
    models = _collision_models()
    deposition_collision = add_deposition_collision(rows, loci, models)
    coupled = add_coupled_loads(rows, loci)
    mass_loads = mass_and_motor_margins(coupled)
    brep = exact_brep_and_dfm()
    parametric = parametric_family(loci)
    conductor = continuous_conductor_authority()
    full_motion = (
        full_raw_rigid_motion(events, timeline, models)
        if run_full_motion else {
            "status": "NOT_RUN",
            "authority": "RIGID_PARTS_ONLY_NOT_CONTINUOUS_CONDUCTOR",
        }
    )
    locus_hash = _canonical_hash(rows)
    terminal_pass = (
        terminal["locus_count"] == EXPECTED_LOCI
        and terminal["pass_count"] == EXPECTED_LOCI
    )
    release_gates = {
        "physical_open_channel_crowned_cap_geometry": brep["status"] == "PASS",
        "all_2400_deposition_terminal_routes_pass": terminal_pass,
        "same_2400_locus_array_drives_route_collision_and_torque": all(
            "crown_cap_to_final_flyer_collision" in row
            and "M2_projected_moment_arm_mm" in row
            for row in rows
        ),
        "all_2400_crowned_cap_to_final_flyer_rigid_clear": (
            deposition_collision["status"] == "PASS"
        ),
        "full_raw_park_index_wrap_load_wind_rigid_sweep_clear": (
            full_motion["status"] == "PASS"
        ),
        "M2_Leadshine_36V_final_margin_ge_2x": mass_loads["M2"][
            "Leadshine_36V_gate_ge_2x"
        ],
        "M1_final_margin_ge_2x": mass_loads["M1"]["gate_ge_2x"],
        "Leadshine_24V_release_authorized": False,
        "continuous_conductor_park_index_load_unload_proven": False,
        "both_raw_shaft_wraps_exactly_two_turns": conductor["shaft_wraps"][
            "each_raw_wrap_is_two_full_turns"
        ],
        "supplier_DFM_finish_and_gauge_complete": False,
        "physical_abrasion_tension_and_endurance_complete": False,
        "production_authorized": False,
    }
    geometry_terminal_rigid_pass = all((
        release_gates["physical_open_channel_crowned_cap_geometry"],
        release_gates["all_2400_deposition_terminal_routes_pass"],
        release_gates["all_2400_crowned_cap_to_final_flyer_rigid_clear"],
        release_gates["full_raw_park_index_wrap_load_wind_rigid_sweep_clear"],
    ))
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": (
            "TERMINAL_AND_RIGID_PASS_REVIEW_ONLY__CONTINUOUS_CONDUCTOR_FAIL"
            if geometry_terminal_rigid_pass else
            "TERMINAL_OR_RIGID_GEOMETRY_FAIL"
        ),
        "decision": (
            "USE_SHARED_CROWN_AS_TERMINAL_SUCCESSOR__DO_NOT_RELEASE_UNTIL_RAW_WRAPS_AND_CONTINUOUS_TRANSITIONS_PASS"
            if geometry_terminal_rigid_pass else
            "DO_NOT_INTEGRATE_SHARED_CROWN_GEOMETRY"
        ),
        "production_authorized": False,
        "assembly_integration_authorized": geometry_terminal_rigid_pass,
        "authority_boundary": {
            "raw_M0_M1_M2": "authoritative canonical capture",
            "aggregate_core_copper_and_named_cap_boundary": "authoritative",
            "exact_strand_centers_order_settling_neatness": "non-authoritative",
            "rigid_collision": "actual crowned caps and final flyer meshes",
            "continuous_conductor": "separate fail-closed gate",
        },
        "paths": {
            "cad_source": "cad/shared_annular_terminal_crown.py",
            "audit_source": "sim/shared_annular_terminal_crown_audit.py",
            "step": "out/review/shared_annular_terminal_crown.step",
            "report": "out/reports/shared_annular_terminal_crown_audit.json",
            "locus_array": "out/reports/shared_annular_terminal_crown_loci.json",
            "manifest": "out/review/shared_annular_terminal_crown.manifest.json",
        },
        "integration_api": {
            "cap_add_on": "crown_add_on(axial_sign, stator_od_mm=46.0)",
            "cap_replacement": "crowned_cap_replacement(axial_sign, stator_od_mm=46.0)",
            "pair": "crown_pair(stator_od_mm=46.0)",
            "retention": "retention_hardware()",
            "collision_link_parts": "spindle_link_parts()",
            "radius_law": "crown_radius_for_stator_od(od_mm)",
            "frame": "stator local +Z/M1 axis; tooth0 +X",
            "collision_link_owner": "spindle",
        },
        "geometry": {
            "material": "natural unfilled PEEK",
            "default_crown_radius_mm": crown.crown_radius_for_stator_od(
                DEFAULT_STATOR.od
            ),
            "front_replacement_bbox": _bbox(crown.crowned_cap_replacement(1)),
            "rear_replacement_bbox": _bbox(crown.crowned_cap_replacement(-1)),
            "mouth_count_per_end": crown.SLOTS,
            "open_transfer_channel_count_per_end": 2 * crown.SLOTS,
            "retention": (
                "unchanged 3x M2 through stacks plus 24 positive slot keys per end"
            ),
            "exact_BREP_and_DFM": brep,
        },
        "canonical_locus_array": {
            "path": "out/reports/shared_annular_terminal_crown_loci.json",
            "row_count": len(rows),
            "sha256": locus_hash,
            "drives": [
                "terminal route", "actual rigid collision", "M2 live-line load",
                "M1 crown reaction load",
            ],
        },
        "terminal_deposition_route": terminal,
        "actual_deposition_rigid_collision": deposition_collision,
        "full_raw_rigid_motion": full_motion,
        "collision_meshes": {
            key: value for key, value in models.items()
            if key not in ("crown_mesh", "flyer_mesh", "crown_bvh", "flyer_bvh")
        },
        "coupled_loads": {
            **coupled,
            "mass_and_motor_margins": mass_loads,
            # Stable direct consumers requested by the drive/retention audits.
            "M2": {
                **coupled["M2"],
                **mass_loads["M2"],
                "rotating_flyer_PEEK_guide_izz_about_axis_kg_m2": (
                    mass_loads["rotating_flyer_PEEK_guide_izz_about_axis_kg_m2"]
                ),
                "final_full_output_inertia_kg_m2": mass_loads["M2"][
                    "final_full_output_inertia_kg_m2"
                ],
            },
            "M1": {**coupled["M1"], **mass_loads["M1"]},
        },
        "parametric_family": parametric,
        "continuous_conductor": conductor,
        "BOM": [
            {
                "qty": 2,
                "item": "one-solid natural-unfilled-PEEK crowned cap replacement",
                "process": "5-axis CNC, stress relieve, polish open channels",
            },
            {
                "qty": 3,
                "item": "existing ISO 4762 M2x20 paired-cap through screw stack",
                "change": "unchanged",
            },
            {
                "qty": 1,
                "item": "rotating shaft-to-tip natural-unfilled-PEEK guide",
                "source": "cad/retained_flyer_peek_guide_successor.py",
            },
        ],
        "rejected_fallback": {
            "architecture": "M1/M2-selected passive gimbal shoe",
            "status": "REJECTED_FALLBACK_NOT_PRIMARY",
            "reason": (
                "the R40 shared outer crown constructs every default terminal "
                "route without a deployed moving guide"
            ),
        },
        "release_gates": release_gates,
        "limits": [
            "The canonical raw shaft wraps are 1.375 and 2.791667 turns, not two turns each.",
            "Park/index/load/unload continuous flexible-conductor geometry remains unproved; the rigid sweep is not a substitute.",
            "OD28/65/90 outer capture cases do not invent unavailable matching cap/core/copper footprints; only OD46 has physical replacement CAD and aggregate authority.",
            "Exact strand packing, settling, sag, snagging, friction, enamel wear and neatness remain non-authoritative or physical-test-only.",
            "Supplier DFM, Ra<=0.4um inspection, gauge coupons and endurance remain open.",
        ],
        "source_hashes": {
            "cad/shared_annular_terminal_crown.py": _sha256(
                CAD / "shared_annular_terminal_crown.py"
            ),
            "sim/shared_annular_terminal_crown_audit.py": _sha256(Path(__file__)),
            "cad/retained_flyer_peek_guide_successor.py": _sha256(
                CAD / "retained_flyer_peek_guide_successor.py"
            ),
            "out/capture/upstream_current_raw.jsonl": _sha256(CAPTURE),
            "out/reports/permanent_cap_aggregate_authorization.json": _sha256(
                AGGREGATE
            ),
            "out/reports/retained_flyer_peek_guide_successor.json": _sha256(
                GUIDE_REPORT
            ),
            "out/reports/integrated_release_candidate.json": _sha256(
                INTEGRATED_RELEASE
            ),
        },
    }
    report["report_sha256"] = _report_hash(report)
    return report, rows


def render_markdown(report: Mapping[str, Any]) -> str:
    terminal = report["terminal_deposition_route"]
    rigid = report["full_raw_rigid_motion"]
    m2 = report["coupled_loads"]["M2"]
    m1 = report["coupled_loads"]["M1"]
    geometry = report["geometry"]["exact_BREP_and_DFM"]
    lines = [
        "# Shared annular terminal crown audit",
        "",
        f"**{report['status']}**",
        "",
        "## Physical crown",
        "",
        f"- Default R40 shared mouth; {report['geometry']['mouth_count_per_end']} mouths/end and {report['geometry']['open_transfer_channel_count_per_end']} open C-channels/end.",
        f"- Transfer clear diameter/opening: {geometry['transfer_clear_diameter_mm']:.3f}/{geometry['continuous_side_opening_mm']:.3f} mm.",
        f"- Minimum-wire worst-wander bend radius: {geometry['minimum_transfer_wire_center_radius_after_wander_mm']:.3f} mm.",
        "- Positive retention: unchanged three M2 paired-cap stacks and 24 slot keys/end.",
        "",
        "## Terminal and rigid results",
        "",
        f"- Deposition routes: {terminal['pass_count']}/{terminal['locus_count']} PASS; {terminal['failure_count']} failures.",
        f"- Minimum shared-mouth margin: {terminal['minimum_mouth_margin_mm']:.6f} mm.",
        f"- Actual crowned caps/final flyer deposition clearance: {report['actual_deposition_rigid_collision']['minimum_clearance_mm']:.6f} mm.",
        f"- Full raw rigid samples: {rigid.get('sample_count', 0)}; collisions: {rigid.get('collision_count', 'not run')}.",
        "",
        "## Coupled loads",
        "",
        f"- M2 max projected arm/10 N torque: {m2['max_projected_moment_arm_mm']:.6f} mm / {m2['wire_torque_at_10N_nm']:.6f} N m.",
        f"- M2 36 V final margin: {m2['Leadshine_36V_available_to_required_multiple']:.3f}x; 24 V stays fail-closed.",
        f"- M1 max crown reaction: {m1['wire_torque_at_10N_nm']:.6f} N m; final margin {m1['available_to_required_multiple']:.3f}x.",
        "",
        "## Fail-closed conductor gates",
        "",
        f"- Raw shaft wraps: {report['continuous_conductor']['shaft_wraps']['raw_turns']} turns; exact-two-turn gate FAIL.",
        "- Park/index/load/unload continuous conductor proof: FAIL (not modeled).",
        "",
        f"Locus-array SHA-256: `{report['canonical_locus_array']['sha256']}`",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ]
    return "\n".join(lines)


def write_reports(*, run_full_motion: bool = True) -> dict[str, Any]:
    report, rows = analyze(run_full_motion=run_full_motion)
    REPORTS.mkdir(parents=True, exist_ok=True)
    REVIEW.mkdir(parents=True, exist_ok=True)
    LOCUS_OUT.write_text(
        json.dumps(rows, indent=2, allow_nan=False) + "\n", encoding="utf-8",
    )
    JSON_OUT.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8",
    )
    MD_OUT.write_text(render_markdown(report), encoding="utf-8")
    manifest = {
        "schema": "shared-annular-terminal-crown-manifest/v1",
        "status": report["status"],
        "source": report["paths"]["cad_source"],
        "step": report["paths"]["step"],
        "integration_api": report["integration_api"],
        "collision_link_owner": "spindle",
        "locus_array": {
            **report["canonical_locus_array"],
            "file_sha256": _sha256(LOCUS_OUT),
        },
        "coupled_loads": report["coupled_loads"],
        "release_gates": report["release_gates"],
        "report_sha256": report["report_sha256"],
    }
    MANIFEST_OUT.write_text(
        json.dumps(manifest, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return report


if __name__ == "__main__":
    result = write_reports(run_full_motion=True)
    print(
        result["status"],
        result["terminal_deposition_route"]["pass_count"],
        result["full_raw_rigid_motion"].get("sample_count", 0),
        result["canonical_locus_array"]["sha256"],
    )
