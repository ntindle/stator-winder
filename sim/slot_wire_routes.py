"""Generate the hash-bound packed-wire half-turn route certificate.

The packing report proves where each completed loop belongs.  This module
adds the missing moving-span proof at both 180-degree crossings of every
turn.  A deterministic guarded-core candidate is accepted only when its
independent prior-copper postcheck is clear; obstructed candidates are
re-planned against a projected copper visibility graph.  Every selected
polyline is then postchecked against the complete 3D stator mesh and exact
segment/capsule copper distances.

The physical crossing pose is independent of angular-velocity sign.  Every
serialized route therefore binds both M2 signs (-1 and +1) to one polyline,
giving 100 unique geometry cases and 200 direction cases for 50 turns.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import heapq
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np
import trimesh
import shapely
from shapely.geometry import LineString, Point
from shapely.ops import unary_union


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

from params import DEFAULT_STATOR  # noqa: E402
import stator_model  # noqa: E402
import wire_geometry  # noqa: E402
import slot_packing_audit  # noqa: E402

from slot_route import (  # noqa: E402
    CopperField,
    PackingSupportGraph,
    PackingTurnRouteAudit,
    RouteResult,
    SlotRoutePlanner,
    active_copper_before,
    dependency_versions,
    neighbor_prefill_copper,
    route_packing_turn,
)


SCHEMA = "slot-wire-routes/v1"
PACKING_PATH = REPORTS / "slot_packing.json"
OUTPUT_PATH = REPORTS / "slot_wire_routes.json"
PLANNER_NUMERICAL_SHELL_MM = 0.0001
LOOP_ARC_STEP_DEG = 5.0
NEIGHBOR_PREFILL_SIDES = (-1, 1)


def _slot_mouth_path_local_xy(
    packing_report: dict[str, Any],
    turn_index: int,
    half_turn_index: int,
    *,
    grid_step_mm: float = 0.02,
) -> tuple[tuple[float, float], ...]:
    """Materialize the packing audit's connected mouth component by A*."""

    config = packing_report["config"]
    selected = packing_report["selected_schedule"]
    job = slot_packing_audit.PackingInput(
        float(config["wire_finished_diameter_mm"]),
        float(config["liner_thickness_mm"]),
    )
    domain = slot_packing_audit._positive_slot_center_domain(job)
    positive = [row["slot_frame_uv_mm"]
                for row in selected["side_positive"]]
    negative = [row["slot_frame_uv_mm"]
                for row in selected["side_negative"]]
    target = np.asarray(positive[int(turn_index)], dtype=float)
    record = selected["side_positive"][int(turn_index)]
    obstacle_radius = job.wire_d_mm - 2.0e-5
    obstacles = [*negative, *positive[:int(turn_index)]]
    disks = [Point(*map(float, center)).buffer(
        obstacle_radius, quad_segs=64) for center in obstacles]
    free = domain.difference(unary_union(disks)) if disks else domain
    components = list(free.geoms) if hasattr(free, "geoms") else [free]
    component = next((item for item in components
                      if not item.is_empty
                      and item.buffer(3.0e-5).covers(
                          Point(*map(float, target)))), None)
    if component is None:
        raise RuntimeError("packed target has no mouth-connected component")
    # Grid/A* is only a candidate finder.  Dilate by less than one grid cell
    # so a mathematically connected tangent corridor is represented on the
    # raster; the returned polyline still must pass the undilated exact 3D
    # core/copper checks in route_packing_turn.
    navigable = component.buffer(max(1.0e-5, grid_step_mm * 0.75))
    minx, miny, maxx, maxy = navigable.bounds
    xs = np.arange(minx, maxx + grid_step_mm * 0.5, grid_step_mm)
    ys = np.arange(miny, maxy + grid_step_mm * 0.5, grid_step_mm)
    xx, yy = np.meshgrid(xs, ys, indexing="ij")
    open_mask = shapely.contains_xy(navigable, xx, yy)
    open_indices = np.argwhere(open_mask)
    if not len(open_indices):
        raise RuntimeError("mouth component has no navigation cells")
    coordinates = np.column_stack((
        xs[open_indices[:, 0]], ys[open_indices[:, 1]]))
    start_order = np.argsort(np.linalg.norm(coordinates - target, axis=1))
    start = None
    angle = math.radians(float(config["slot_bisector_deg"]))
    c, s = math.cos(angle), math.sin(angle)

    def active_xy(slot_point: np.ndarray) -> np.ndarray:
        u, v = map(float, slot_point)
        value = np.array((u * c + v * s, -u * s + v * c))
        if half_turn_index == 1:
            value[1] *= -1.0
        return value

    active_target = active_xy(target)
    support_normals = []
    for parent_index in record["parent_turn_indices"]:
        parent_slot = np.asarray(
            positive[int(parent_index)], dtype=float)
        vector = active_target - active_xy(parent_slot)
        support_normals.append(vector / np.linalg.norm(vector))
    for candidate_index in start_order[:256]:
        candidate = open_indices[candidate_index]
        point = coordinates[candidate_index]
        approach = active_xy(point) - active_target
        if (support_normals
                and min(float(approach @ normal)
                        for normal in support_normals) < -1e-9):
            continue
        if navigable.covers(LineString((target, point))):
            start = tuple(map(int, candidate))
            break
    if start is None:
        raise RuntimeError("mouth target has no reachable navigation cell")

    cap = float(config["radial_center_cap_mm"])
    goal_mask = open_mask & (np.hypot(xx, yy) >= cap - 0.03)
    if not np.any(goal_mask):
        raise RuntimeError("mouth component does not reach radial cap")
    neighbor_steps = (
        (-1, -1), (-1, 0), (-1, 1), (0, -1),
        (0, 1), (1, -1), (1, 0), (1, 1),
    )
    queue = [(0.0, 0.0, start)]
    costs = {start: 0.0}
    previous: dict[tuple[int, int], tuple[int, int]] = {}
    goal = None
    while queue:
        _, cost, current = heapq.heappop(queue)
        if cost > costs.get(current, math.inf) + 1e-12:
            continue
        if goal_mask[current]:
            goal = current
            break
        for di, dj in neighbor_steps:
            nxt = (current[0] + di, current[1] + dj)
            if (not 0 <= nxt[0] < len(xs)
                    or not 0 <= nxt[1] < len(ys)
                    or not open_mask[nxt]):
                continue
            step = grid_step_mm * math.hypot(di, dj)
            new_cost = cost + step
            if new_cost + 1e-12 >= costs.get(nxt, math.inf):
                continue
            costs[nxt] = new_cost
            previous[nxt] = current
            radius = math.hypot(xs[nxt[0]], ys[nxt[1]])
            heuristic = max(0.0, cap - 0.03 - radius)
            heapq.heappush(queue, (new_cost + heuristic, new_cost, nxt))
    if goal is None:
        raise RuntimeError("A* could not traverse mouth-connected component")
    cells = [goal]
    while cells[-1] != start:
        cells.append(previous[cells[-1]])
    cells.reverse()
    path = [target, *[np.array((xs[i], ys[j])) for i, j in cells]]
    # Greedy line-of-sight compression retains the exact component contract.
    compact = [path[0]]
    cursor = 0
    while cursor < len(path) - 1:
        chosen = cursor + 1
        if cursor != 0:
            for index in range(len(path) - 1, cursor, -1):
                if navigable.covers(LineString((
                        path[cursor], path[index]))):
                    chosen = index
                    break
        compact.append(path[chosen])
        cursor = chosen
    compact.reverse()  # mouth -> target
    active = []
    for u, v in compact:
        x = float(u * c + v * s)
        y = float(-u * s + v * c)
        if half_turn_index == 1:
            y = -y
        active.append((x, y))
    return tuple(active)


def _end_turn_boundary_path_local_xy(
    packing_report: dict[str, Any],
    graph: PackingSupportGraph,
    turn_index: int,
    half_turn_index: int,
    *,
    arc_step_deg: float = 0.5,
) -> tuple[tuple[float, float], ...]:
    """Follow an exact support-parent boundary into the end-turn arc.

    A zero-clearance tangent corridor is not representable by an ordinary
    occupancy raster.  Start at the mouth point found by the packing audit,
    take a tangent to a slightly guarded circle around one declared parent,
    follow that circle, then approach the packed target radially.  The path
    is evaluated analytically in ``(active radial, profile radius)`` space;
    mapping it onto a fixed end-turn normal is an isometry.
    """

    if not 0.0 < arc_step_deg <= 2.0:
        raise ValueError("end-turn boundary arc step must be in (0, 2] deg")
    turn = graph.turn(turn_index)
    if not turn.parent_turn_indices:
        raise ValueError("end-turn boundary path requires support parents")
    fast_path = _slot_mouth_path_local_xy(
        packing_report, turn_index, half_turn_index)
    config = packing_report["config"]
    half_neck = max(2.5, float(config["od_mm"]) * 0.07) / 2.0

    def to_state(point: tuple[float, float]) -> np.ndarray:
        radial, tangential = map(float, point)
        profile = (
            -tangential - half_neck
            if half_turn_index == 0
            else tangential - half_neck)
        return np.array((radial, profile), dtype=float)

    start = to_state(fast_path[0])
    target = np.array((
        turn.radial_mm, turn.profile_radius_mm), dtype=float)
    prior = tuple(graph.turn(index) for index in range(turn.turn_index))
    maximum_profile = max(item.profile_radius_mm for item in graph.turns)
    obstacle_chord_guard = (
        maximum_profile
        * (1.0 - math.cos(math.radians(LOOP_ARC_STEP_DEG) / 2.0))
        + 1.0e-6)
    boundary_guard = max(0.0015, obstacle_chord_guard + 0.0005)

    def point_polyline_distance(
            point: np.ndarray, path: np.ndarray) -> float:
        starts = path[:-1]
        vectors = path[1:] - starts
        denominators = np.einsum("ij,ij->i", vectors, vectors)
        fractions = np.divide(
            np.einsum("ij,ij->i", point - starts, vectors),
            denominators,
            out=np.zeros(len(starts), dtype=float),
            where=denominators > 1e-18,
        )
        fractions = np.clip(fractions, 0.0, 1.0)
        nearest = starts + fractions[:, None] * vectors
        return float(np.min(np.linalg.norm(nearest - point, axis=1)))

    candidates: list[tuple[float, np.ndarray]] = []
    parent_set = set(turn.parent_turn_indices)
    for support_index in turn.parent_turn_indices:
        support = graph.turn(support_index)
        center = np.array((
            support.radial_mm, support.profile_radius_mm), dtype=float)
        target_radius = float(np.linalg.norm(target - center))
        if abs(target_radius - graph.wire_diameter_mm) > 1e-9:
            continue
        guarded_radius = graph.wire_diameter_mm + boundary_guard
        start_vector = start - center
        start_radius = float(np.linalg.norm(start_vector))
        if start_radius <= guarded_radius:
            continue
        start_angle = math.atan2(start_vector[1], start_vector[0])
        tangent_offset = math.acos(guarded_radius / start_radius)
        target_vector = target - center
        target_angle = math.atan2(target_vector[1], target_vector[0])
        for tangent_angle in (
                start_angle - tangent_offset,
                start_angle + tangent_offset):
            for direction in (-1, 1):
                if direction > 0:
                    delta = (target_angle - tangent_angle) % (2.0 * math.pi)
                else:
                    delta = -(
                        (tangent_angle - target_angle) % (2.0 * math.pi))
                divisions = max(
                    1, math.ceil(
                        abs(math.degrees(delta)) / arc_step_deg))
                angles = np.linspace(
                    tangent_angle, tangent_angle + delta, divisions + 1)
                guarded_arc = center + guarded_radius * np.column_stack((
                    np.cos(angles), np.sin(angles)))
                candidate = np.vstack((start, guarded_arc, target))
                clear = True
                for obstacle in prior:
                    obstacle_center = np.array((
                        obstacle.radial_mm,
                        obstacle.profile_radius_mm), dtype=float)
                    required = (
                        graph.wire_diameter_mm
                        if obstacle.turn_index in parent_set
                        else (graph.wire_diameter_mm
                              + obstacle_chord_guard))
                    if (point_polyline_distance(
                            obstacle_center, candidate) + 1e-9
                            < required):
                        clear = False
                        break
                if clear:
                    length = float(np.sum(np.linalg.norm(
                        candidate[1:] - candidate[:-1], axis=1)))
                    candidates.append((length, candidate))
    if not candidates:
        raise RuntimeError(
            "no analytically clear support-boundary end-turn path")
    _, selected_path = min(candidates, key=lambda item: item[0])
    active_path = []
    for radial, profile in selected_path:
        tangential = (
            -half_neck - profile
            if half_turn_index == 0
            else half_neck + profile)
        active_path.append((float(radial), float(tangential)))
    return tuple(active_path)


def _support_cone_boundary_directions_deg(
    parent_normals: list[np.ndarray],
) -> tuple[float, ...]:
    """Return one-degree common-cone directions, boundary first.

    A coarse 15-degree search can miss a real narrow route beside a packed
    support cluster.  The support cone itself is analytic; this helper only
    refines candidate direction selection.  Every returned candidate still
    goes through the exact core/copper postcheck in ``route_packing_turn``.
    """

    if not parent_normals:
        return ()
    eligible = []
    for angle_deg in range(360):
        direction = np.array((
            math.cos(math.radians(angle_deg)),
            math.sin(math.radians(angle_deg)),
        ))
        eligible.append(all(
            float(direction @ normal) >= -1.0e-9
            for normal in parent_normals
        ))
    if not any(eligible):
        return ()

    starts = [
        index for index, allowed in enumerate(eligible)
        if allowed and not eligible[(index - 1) % 360]
    ]
    if not starts:  # all 360 degrees are eligible
        starts = [0]
    runs: list[list[int]] = []
    seen: set[int] = set()
    for start in starts:
        run = []
        cursor = start
        while eligible[cursor] and cursor not in seen:
            run.append(cursor)
            seen.add(cursor)
            cursor = (cursor + 1) % 360
        if run:
            runs.append(run)

    result: list[float] = []
    for run in runs:
        # Probe both exact boundaries, then a small coarse-to-fine set of
        # inward offsets before walking the remainder.  Narrow physical
        # corridors often sit a few degrees inside a common-cone boundary;
        # this order reaches them without changing the candidate set.
        preferred_offsets = (0, 4, 2, 1, 3)
        offsets = [
            *preferred_offsets,
            *(index for index in range(len(run))
              if index not in preferred_offsets),
        ]
        used: set[int] = set()
        for offset in offsets:
            for index in (offset, len(run) - 1 - offset):
                if not 0 <= index < len(run) or index in used:
                    continue
                used.add(index)
                result.append(float(run[index]))
    return tuple(result)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _source_hashes() -> dict[str, str]:
    paths = {
        "sim/slot_route.py": HERE / "slot_route.py",
        "sim/slot_wire_routes.py": HERE / "slot_wire_routes.py",
        "cad/params.py": CAD / "params.py",
        "cad/stator_model.py": CAD / "stator_model.py",
        "cad/wire_geometry.py": CAD / "wire_geometry.py",
        "cad/slot_packing_audit.py": CAD / "slot_packing_audit.py",
    }
    return {name: _sha256(path) for name, path in paths.items()}


def _canonical_hash(payload: dict[str, Any]) -> str:
    canonical = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(canonical.encode()).hexdigest()


def _finite_or_none(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _finite_or_none(item)
                for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_finite_or_none(item) for item in value]
    return value


def _route_record(result: RouteResult,
                  audit: PackingTurnRouteAudit) -> dict[str, Any]:
    record = {
        "turn_index": audit.turn_index,
        "half_turn_index": audit.half_turn_index,
        "phase_index": audit.phase_index,
        "logical_phase_rad": audit.logical_phase_rad,
        "validated_motion_signs": list(audit.validated_motion_signs),
        "status": "PASS" if result.ok and audit.ok else "FAIL",
        "reason": result.reason if not result.ok else audit.reason,
        "target_local_mm": list(audit.target_local_mm),
        "endpoint_support": result.endpoint_support,
        "progressive_support_validated": (
            result.progressive_support_validated),
        "route": {
            "points_local_mm": [list(point)
                                for point in result.points_local_mm],
            "segment_tags": list(result.segment_tags),
            "torus_exit_point_index": result.torus_exit_point_index,
            "total_length_mm": result.total_length_mm,
            "free_length_mm": result.free_length_mm,
            "torus_continuity_error_deg": (
                result.torus_continuity_error_deg),
        },
        "postcheck": asdict(audit),
        "planner_metadata": result.metadata,
    }
    return _finite_or_none(record)


def build_planner(graph: PackingSupportGraph, spec: Any) -> SlotRoutePlanner:
    """Build the release planner from current source CAD, not a mesh cache."""

    if not math.isclose(
            float(spec.wire_d), graph.wire_diameter_mm,
            rel_tol=0.0, abs_tol=1e-12):
        raise ValueError(
            "route-job wire diameter does not match slot_packing.json")
    if int(spec.turns) != len(graph.turns):
        raise ValueError(
            "route-job turn count does not match slot_packing.json")
    part = stator_model.stator(spec)
    vertices, faces = part.tessellate(0.01, 0.03)
    mesh = trimesh.Trimesh(
        vertices=np.array([(vertex.X, vertex.Y, vertex.Z)
                           for vertex in vertices]),
        faces=np.asarray(faces),
        process=True,
    )
    if not mesh.is_watertight:
        raise ValueError("source stator tessellation is not watertight")
    return SlotRoutePlanner(
        spec=spec,
        stator_part=part,
        stator_mesh_local=mesh,
        guide=wire_geometry.tip_guide_spec(),
        # Explicit packed routes only consume the shared contact plane.
        contact={"z_mm": wire_geometry.TOOTH_CONTACT_Z},
        guide_wire_radius_mm=graph.wire_diameter_mm / 2.0,
        access_radius_mm=graph.center_core_access_mm,
        planner_offset_mm=(graph.center_core_access_mm
                           + PLANNER_NUMERICAL_SHELL_MM),
        clamp_goal_to_stack=False,
        # Candidate visibility graph only.  The selected polyline is checked
        # against the unsimplified 3D core/copper geometry below.
        visibility_chord_mm=0.20,
    )


def _validate_packing_contract(report: dict[str, Any]) -> Any:
    """Validate one nominal/measured release job and return its local spec."""

    if report.get("schema") != slot_packing_audit.SCHEMA:
        raise ValueError("route generation requires slot-packing/v2")
    if report.get("status") != "PASS":
        raise ValueError("slot-packing report is not PASS")
    config = report.get("config", {})
    selected = report.get("selected_schedule", {})
    receiving = report.get("receiving_contract", {})
    try:
        wire = float(config["wire_finished_diameter_mm"])
        liner = float(config["liner_thickness_mm"])
        wire_range = tuple(map(
            float, receiving["wire_finished_diameter_range_mm"]))
        liner_range = tuple(map(
            float, receiving["liner_thickness_range_mm"]))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            "slot-packing report has no numeric receiving contract") from exc
    if (len(wire_range) != 2 or not wire_range[0] <= wire <= wire_range[1]
            or len(liner_range) != 2
            or not liner_range[0] <= liner <= liner_range[1]):
        raise ValueError("slot-packing job is outside its receiving interval")
    if receiving.get("topology_sensitivity_status") != "PASS":
        raise ValueError("slot-packing receiving topology is not PASS")

    turns = int(selected.get("turns_per_tooth", -1))
    checks = {
        "slots": (config.get("slots"), int(DEFAULT_STATOR.slots)),
        "od_mm": (config.get("od_mm"), float(DEFAULT_STATOR.od)),
        "stack_mm": (config.get("stack_mm"), float(DEFAULT_STATOR.stack)),
        "turns_per_tooth": (turns, int(DEFAULT_STATOR.turns)),
    }
    mismatches = []
    for name, (actual, expected) in checks.items():
        if actual is None or not math.isclose(
                float(actual), float(expected), rel_tol=0.0, abs_tol=1e-9):
            mismatches.append(f"{name}: report={actual}, required={expected}")
    if mismatches:
        raise ValueError(
            "slot-packing release contract mismatch: " + "; ".join(mismatches))
    nominal_values = (
        math.isclose(
            wire, slot_packing_audit.SUPPLIER_NOMINAL_WIRE_MM,
            rel_tol=0.0, abs_tol=1e-12)
        and math.isclose(
            liner, slot_packing_audit.SUPPLIER_NOMINAL_LINER_MM,
            rel_tol=0.0, abs_tol=1e-12))
    role = report.get("role")
    provenance = config.get("input_provenance")
    allowed_role_provenance = {
        ("authoritative_release_default",
         "supplier_nominal_simulation_default"),
        ("authoritative_measured_release_job", "measured_receiving_input"),
    }
    if (role, provenance) not in allowed_role_provenance:
        raise ValueError(
            "slot-packing role/provenance pair is not authoritative")
    if role == "authoritative_release_default" and not nominal_values:
        raise ValueError("default release role requires supplier nominal inputs")
    expected_access = wire / 2.0 + liner
    if not math.isclose(
            float(config.get("center_core_access_mm", math.nan)),
            expected_access, rel_tol=0.0, abs_tol=1e-12):
        raise ValueError("slot-packing center/core contract is inconsistent")
    mouth = selected.get("sequential_mouth_access", {})
    if (mouth.get("status") != "PASS"
            or not mouth.get("all_empty_neighbor_side_connected")
            or not mouth.get("all_prefilled_neighbor_side_connected")):
        raise ValueError("slot-packing sequential mouth access is not PASS")
    distances = report.get("validation", {}).get(
        "all_consecutive_schedule_distances_mm")
    if (not isinstance(distances, list) or len(distances) != 49
            or any(abs(float(distance) - wire) > 1e-9
                   for distance in distances)):
        raise ValueError("slot-packing schedule is not a 49-edge tangent path")
    return replace(DEFAULT_STATOR, wire_d=wire, turns=turns)


def analyze(packing_path: Path = PACKING_PATH,
            *, progress: bool = False) -> dict[str, Any]:
    packing_path = Path(packing_path)
    packing_report = json.loads(packing_path.read_text())
    spec = _validate_packing_contract(packing_report)
    graph = PackingSupportGraph.from_report(
        packing_report, spec=spec)
    planner = build_planner(graph, spec)

    neighbor_obstacles = {
        side: neighbor_prefill_copper(
            graph, spec, side,
            arc_step_deg=LOOP_ARC_STEP_DEG)
        for side in NEIGHBOR_PREFILL_SIDES
    }
    all_neighbor_obstacles = tuple(
        obstacle for side in NEIGHBOR_PREFILL_SIDES
        for obstacle in neighbor_obstacles[side])
    records: list[dict[str, Any]] = []
    minimum_core = math.inf
    minimum_copper = math.inf
    for turn in graph.turns:
        prior = active_copper_before(
            graph, turn.turn_index, spec,
            arc_step_deg=LOOP_ARC_STEP_DEG)
        field = CopperField(prior + all_neighbor_obstacles)
        for half_turn_index in (0, 1):
            # The crossing lies on one tooth side.  Search against that
            # adjacent prefilled coil plus every prior active loop; retain
            # both neighbors in the independent exact 3D postcheck.
            approach_neighbor_side = -1 if half_turn_index == 0 else 1
            parent_indices = set(turn.parent_turn_indices)
            planning_prior = tuple(
                obstacle for obstacle in prior
                if obstacle.turn_index not in parent_indices)
            planning_field = CopperField(
                planning_prior
                + neighbor_obstacles[approach_neighbor_side])
            # Most poses clear on the deterministic core route.  Only invoke
            # the much larger projected-copper visibility graph when the
            # exact first postcheck finds an obstruction.
            result, audit = route_packing_turn(
                planner,
                graph,
                spec,
                turn.turn_index,
                half_turn_index,
                neighbor_sides=NEIGHBOR_PREFILL_SIDES,
                copper_field=field,
                planning_copper_field=planning_field,
                arc_step_deg=LOOP_ARC_STEP_DEG,
                plan_with_copper_projection=False,
            )
            if not (result.ok and audit.ok):
                first_reason = result.reason
                attempts = []
                strategies: list[tuple[float, bool, float | None]] = (
                    [(0.5, True, None), (0.25, True, None),
                     (0.1, True, None), (0.05, True, None),
                     (0.02, True, None), (0.1, False, None),
                     (0.05, False, None), (0.02, False, None),
                     (0.01, False, None), (0.25, False, None),
                     (1.0, True, None)]
                    if turn.parent_turn_indices else [(0.0, True, None)])
                if turn.parent_turn_indices:
                    phase_y_sign = -1.0 if half_turn_index == 0 else 1.0
                    parent_normals = []
                    for parent_index in turn.parent_turn_indices:
                        parent = graph.turn(parent_index)
                        vector = np.array((
                            turn.radial_mm - parent.radial_mm,
                            phase_y_sign * (
                                turn.profile_radius_mm
                                - parent.profile_radius_mm),
                        ))
                        parent_normals.append(
                            vector / np.linalg.norm(vector))
                    for angle_deg in range(0, 360, 15):
                        direction = np.array((
                            math.cos(math.radians(angle_deg)),
                            math.sin(math.radians(angle_deg)),
                        ))
                        if min(float(direction @ normal)
                               for normal in parent_normals) < -1e-9:
                            continue
                        for distance in (0.05, 0.1, 0.25, 0.5, 1.0):
                            strategies.append(
                                (distance, False, float(angle_deg)))
                    coarse_directions = {
                        direction_deg for _distance, _core, direction_deg
                        in strategies if direction_deg is not None
                    }
                    refined_directions = [
                        angle_deg
                        for angle_deg in _support_cone_boundary_directions_deg(
                            parent_normals
                        )
                        if angle_deg not in coarse_directions
                    ]
                    refined_strategies = []
                    # Try one promising stand-off across the whole cone before
                    # multiplying directions by every fallback distance.  Put
                    # these analytically admissible boundary refinements ahead
                    # of the legacy coarse/global guesses; every candidate is
                    # still accepted only by the same exact postchecks.
                    for distance in (
                            0.5, 0.25, 1.0, 0.1, 0.05, 0.02, 0.01):
                        for angle_deg in refined_directions:
                            refined_strategies.append(
                                (distance, False, angle_deg))
                    strategies = refined_strategies + strategies
                for distance, core_outward, direction_deg in strategies:
                    try:
                        candidate_result, candidate_audit = route_packing_turn(
                            planner,
                            graph,
                            spec,
                            turn.turn_index,
                            half_turn_index,
                            neighbor_sides=NEIGHBOR_PREFILL_SIDES,
                            copper_field=field,
                            planning_copper_field=planning_field,
                            arc_step_deg=LOOP_ARC_STEP_DEG,
                            plan_with_copper_projection=False,
                            support_normal_approach_mm=distance,
                            enforce_core_outward_approach=core_outward,
                            support_approach_direction_deg=direction_deg,
                        )
                    except Exception as exc:
                        attempts.append({
                            "distance_mm": distance,
                            "core_outward_constraint": core_outward,
                            "direction_deg": direction_deg,
                            "status": "ERROR",
                            "reason": str(exc),
                        })
                        continue
                    result, audit = candidate_result, candidate_audit
                    attempts.append({
                        "distance_mm": distance,
                        "core_outward_constraint": core_outward,
                        "direction_deg": direction_deg,
                        "status": (
                            "PASS" if result.ok and audit.ok else "FAIL"),
                        "reason": result.reason,
                    })
                    if result.ok and audit.ok:
                        break
                if not (result.ok and audit.ok):
                    try:
                        mouth_path = _slot_mouth_path_local_xy(
                            packing_report, turn.turn_index,
                            half_turn_index)
                        candidate_result, candidate_audit = route_packing_turn(
                            planner,
                            graph,
                            spec,
                            turn.turn_index,
                            half_turn_index,
                            neighbor_sides=NEIGHBOR_PREFILL_SIDES,
                            copper_field=field,
                            planning_copper_field=planning_field,
                            arc_step_deg=LOOP_ARC_STEP_DEG,
                            mouth_path_local_xy_mm=mouth_path,
                        )
                        result, audit = candidate_result, candidate_audit
                        attempts.append({
                            "strategy": "slot_mouth_component_astar",
                            "point_count": len(mouth_path),
                            "status": (
                                "PASS" if result.ok and audit.ok else "FAIL"),
                            "reason": result.reason,
                        })
                    except Exception as exc:
                        attempts.append({
                            "strategy": "slot_mouth_component_astar",
                            "status": "ERROR",
                            "reason": str(exc),
                        })
                if not (result.ok and audit.ok):
                    try:
                        end_turn_path = _end_turn_boundary_path_local_xy(
                            packing_report, graph, turn.turn_index,
                            half_turn_index)
                        candidate_result, candidate_audit = route_packing_turn(
                            planner,
                            graph,
                            spec,
                            turn.turn_index,
                            half_turn_index,
                            neighbor_sides=NEIGHBOR_PREFILL_SIDES,
                            copper_field=field,
                            planning_copper_field=planning_field,
                            arc_step_deg=LOOP_ARC_STEP_DEG,
                            mouth_path_local_xy_mm=end_turn_path,
                            end_turn_arc_approach=True,
                        )
                        result, audit = candidate_result, candidate_audit
                        attempts.append({
                            "strategy": (
                                "support_boundary_mapped_end_turn_arc"),
                            "point_count": len(end_turn_path),
                            "status": (
                                "PASS" if result.ok and audit.ok else
                                "FAIL"),
                            "reason": result.reason,
                        })
                    except Exception as exc:
                        attempts.append({
                            "strategy": (
                                "support_boundary_mapped_end_turn_arc"),
                            "status": "ERROR",
                            "reason": str(exc),
                        })
                metadata = dict(result.metadata)
                metadata["core_route_copper_fallback"] = {
                    "attempted": True,
                    "initial_rejection": first_reason,
                    "approach_attempts": attempts,
                }
                result = replace(result, metadata=metadata)
            records.append(_route_record(result, audit))
            if math.isfinite(audit.minimum_core_center_distance_mm):
                minimum_core = min(
                    minimum_core,
                    audit.minimum_core_center_distance_mm)
            if math.isfinite(audit.minimum_copper_center_distance_mm):
                minimum_copper = min(
                    minimum_copper,
                    audit.minimum_copper_center_distance_mm)
            if progress:
                print(
                    f"turn {turn.turn_index:02d} half {half_turn_index}: "
                    f"{'PASS' if result.ok and audit.ok else 'FAIL'}",
                    flush=True,
                )

    expected_geometry_cases = len(graph.turns) * 2
    passed = sum(record["status"] == "PASS" for record in records)
    coverage = {
        (int(record["turn_index"]), int(record["half_turn_index"]))
        for record in records
    }
    expected_coverage = {
        (turn_index, half_turn_index)
        for turn_index in range(len(graph.turns))
        for half_turn_index in (0, 1)
    }
    signs_ok = all(
        record["validated_motion_signs"] == [-1, 1]
        for record in records)
    progressive_ok = all(
        bool(record["progressive_support_validated"])
        for record in records)
    release_proof_flags = {
        # Sign-specific deposition history changes which half of the current
        # turn is already an obstacle at the second crossing.  The current
        # table intentionally does not claim that proof from pose equality.
        "current_half_sign_specific": False,
        # Visibility/A* and support-boundary candidates contain polyline
        # corners that have not been replaced by tangent C1 curves or proved
        # as continuously supported contact.
        "c1_bend_continuity": False,
        # Selected routes are now checked against the exact OCC source part;
        # the tessellation is candidate-search/diagnostic geometry only.
        "exact_core_or_bound": True,
        # The smallest nominal geometric margin has no released allowance
        # yet for wire/liner tolerance, placement following error, elastic
        # deformation, or numerical copper-loop chord error.
        "physical_error_budget": False,
    }
    status = "PASS" if all((
        len(records) == expected_geometry_cases,
        coverage == expected_coverage,
        passed == expected_geometry_cases,
        signs_ok,
        progressive_ok,
        minimum_core + 1e-9 >= graph.center_core_access_mm,
        minimum_copper + 1e-9 >= graph.wire_diameter_mm,
        all(release_proof_flags.values()),
    )) else "FAIL"

    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": status,
        "scope": {
            "packing_input": str(packing_path.relative_to(ROOT)).replace(
                "\\", "/"),
            "unique_geometry_cases": expected_geometry_cases,
            "motion_direction_cases": expected_geometry_cases * 2,
            "turns": len(graph.turns),
            "half_turn_crossings_per_turn": 2,
            "validated_motion_signs": [-1, 1],
            "direction_invariance": (
                "The rigid pose is sign-invariant, but deposited-current-"
                "half obstacles are not. The [-1,+1] labels remain requested "
                "coverage only until sign-specific history is modeled."),
            "neighbor_prefill_sides": list(NEIGHBOR_PREFILL_SIDES),
            "prior_copper_rule": (
                "all completed active-tooth turns plus both fully wound "
                "adjacent teeth; current turn excluded as the same conductor"),
            "not_proved_here": (
                "continuous between-crossing M0/M2 motion, lead-in/out, "
                "dynamic tension, wire sag, and capture synchronization are "
                "separate release gates"),
        },
        "input_contract": {
            "packing_schema": graph.schema,
            "packing_role": packing_report["role"],
            "input_provenance": packing_report["config"].get(
                "input_provenance"),
            "packing_report_sha256": graph.report_sha256,
            "packing_file_sha256": _sha256(packing_path),
            "wire_finished_diameter_mm": graph.wire_diameter_mm,
            "liner_thickness_mm": float(
                packing_report["config"]["liner_thickness_mm"]),
            "required_core_center_distance_mm": (
                graph.center_core_access_mm),
            "required_copper_center_distance_mm": (
                graph.wire_diameter_mm),
            "receiving_contract": {
                "wire_finished_diameter_range_mm": packing_report[
                    "receiving_contract"][
                        "wire_finished_diameter_range_mm"],
                "liner_thickness_range_mm": packing_report[
                    "receiving_contract"]["liner_thickness_range_mm"],
                "topology_sensitivity_status": packing_report[
                    "receiving_contract"]["topology_sensitivity_status"],
            },
            "stator": {
                "slots": int(spec.slots),
                "od_mm": float(spec.od),
                "stack_mm": float(spec.stack),
                "turns_per_tooth": int(spec.turns),
            },
            "sequential_mouth_access": packing_report[
                "selected_schedule"]["sequential_mouth_access"],
        },
        "planner_contract": {
            "core_geometry": (
                "source OpenCascade stator; selected polyline converted to "
                "OCC edges and checked with Part.distance_to; watertight "
                "0.01/0.03 tessellation is candidate-search/diagnostic only"),
            "physical_core_access_mm": graph.center_core_access_mm,
            "numerical_planner_shell_mm": (
                graph.center_core_access_mm + PLANNER_NUMERICAL_SHELL_MM),
            "guide_wire_radius_mm": graph.wire_diameter_mm / 2.0,
            "deposited_loop_arc_step_deg": LOOP_ARC_STEP_DEG,
            "copper_postcheck": (
                "exact 3D segment-to-segment centerline distance against "
                "every prior/neighbor polyline capsule"),
            "copper_route_search": (
                "2D projection of every relevant 3D copper capsule into the "
                "active radial section; declared endpoint-support parents "
                "are omitted only from this candidate graph so their tangent "
                "endpoint is not sealed by the numerical guard; the exact "
                "3D postcheck includes them and remains authority"),
            "dependencies": dependency_versions(),
        },
        "validation": {
            "expected_geometry_cases": expected_geometry_cases,
            "generated_geometry_cases": len(records),
            "passed_geometry_cases": passed,
            "expected_direction_cases": expected_geometry_cases * 2,
            "covered_direction_cases": (
                0 if not release_proof_flags[
                    "current_half_sign_specific"] else
                sum(len(record["validated_motion_signs"])
                    for record in records)),
            "labeled_direction_cases": sum(
                len(record["validated_motion_signs"])
                for record in records),
            "coverage_complete": coverage == expected_coverage,
            "both_motion_signs_labeled": signs_ok,
            "both_motion_signs_covered": bool(
                signs_ok and release_proof_flags[
                    "current_half_sign_specific"]),
            "progressive_support_validated": progressive_ok,
            "release_proof_flags": release_proof_flags,
            "release_blockers": [
                name for name, passed_flag in release_proof_flags.items()
                if not passed_flag
            ],
            "minimum_core_center_distance_mm": _finite_or_none(minimum_core),
            "minimum_core_margin_mm": _finite_or_none(
                minimum_core - graph.center_core_access_mm),
            "minimum_copper_center_distance_mm": _finite_or_none(
                minimum_copper),
            "minimum_copper_margin_mm": _finite_or_none(
                minimum_copper - graph.wire_diameter_mm),
        },
        "source_hashes": _source_hashes(),
        "routes": records,
    }
    report = _finite_or_none(report)
    report["report_sha256"] = _canonical_hash(report)
    return report


def validate_report_integrity(report: dict[str, Any],
                              packing_report: dict[str, Any]) -> None:
    """Validate diagnostic table integrity without authorizing hardware."""

    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported slot-wire route report schema")
    payload = dict(report)
    expected_hash = payload.pop("report_sha256", None)
    if not isinstance(expected_hash, str):
        raise ValueError("slot-wire route report has no report_sha256")
    if _canonical_hash(payload) != expected_hash:
        raise ValueError("slot-wire route report hash mismatch")
    packing_hash = packing_report.get("report_sha256")
    if report.get("input_contract", {}).get(
            "packing_report_sha256") != packing_hash:
        raise ValueError("slot-wire route report is stale for packing input")
    spec = _validate_packing_contract(packing_report)
    PackingSupportGraph.from_report(
        packing_report, spec=spec)
    if report.get("source_hashes") != _source_hashes():
        raise ValueError("slot-wire route report source hashes are stale")
    routes = report.get("routes")
    if not isinstance(routes, list) or len(routes) != 100:
        raise ValueError("slot-wire route table must contain 100 rows")
    coverage = {
        (int(route["turn_index"]), int(route["half_turn_index"]))
        for route in routes
    }
    expected_coverage = {
        (turn_index, half_turn_index)
        for turn_index in range(50)
        for half_turn_index in (0, 1)
    }
    if coverage != expected_coverage:
        raise ValueError("slot-wire route table has duplicate/missing poses")
    for route in routes:
        geometry = route.get("route", {})
        points = geometry.get("points_local_mm")
        tags = geometry.get("segment_tags")
        if (not isinstance(points, list) or len(points) < 2
                or not isinstance(tags, list)
                or len(tags) != len(points) - 1):
            raise ValueError("slot-wire route polyline/tag structure is invalid")


def validate_report(report: dict[str, Any],
                    packing_report: dict[str, Any]) -> None:
    """Fail closed on a stale, incomplete, or non-release route table."""

    validate_report_integrity(report, packing_report)
    if report.get("status") != "PASS":
        raise ValueError("slot-wire route report is not PASS")
    validation = report.get("validation", {})
    required = {
        "generated_geometry_cases": 100,
        "passed_geometry_cases": 100,
        "expected_direction_cases": 200,
        "covered_direction_cases": 200,
    }
    for name, expected in required.items():
        if int(validation.get(name, -1)) != expected:
            raise ValueError(f"route coverage mismatch: {name}")
    if not all(bool(validation.get(name)) for name in (
            "coverage_complete", "both_motion_signs_covered",
            "progressive_support_validated")):
        raise ValueError("slot-wire route coverage/support gate is false")
    proof_flags = validation.get("release_proof_flags")
    if (not isinstance(proof_flags, dict)
            or set(proof_flags) != {
                "current_half_sign_specific", "c1_bend_continuity",
                "exact_core_or_bound", "physical_error_budget",
            }
            or not all(bool(value) for value in proof_flags.values())):
        raise ValueError("slot-wire route release proof is incomplete")
    routes = report.get("routes")
    if not isinstance(routes, list) or len(routes) != 100:
        raise ValueError("slot-wire route table must contain 100 rows")
    coverage = {
        (int(route["turn_index"]), int(route["half_turn_index"]))
        for route in routes
    }
    expected_coverage = {
        (turn_index, half_turn_index)
        for turn_index in range(50)
        for half_turn_index in (0, 1)
    }
    if coverage != expected_coverage:
        raise ValueError("slot-wire route table has duplicate/missing poses")
    if any(route.get("status") != "PASS"
           or route.get("validated_motion_signs") != [-1, 1]
           or not route.get("progressive_support_validated")
           for route in routes):
        raise ValueError("slot-wire route row failed its release contract")
    for route in routes:
        geometry = route.get("route", {})
        points = geometry.get("points_local_mm")
        tags = geometry.get("segment_tags")
        if (not isinstance(points, list) or len(points) < 2
                or not isinstance(tags, list)
                or len(tags) != len(points) - 1):
            raise ValueError("slot-wire route polyline/tag structure is invalid")
        if (geometry.get("torus_continuity_error_deg") is None
                or float(geometry["torus_continuity_error_deg"]) > 1e-8):
            raise ValueError("slot-wire route torus tangency regressed")
        checks = route.get("planner_metadata", {}).get(
            "exact_release_postcheck", {}).get("checks")
        if not isinstance(checks, dict) or not checks or not all(checks.values()):
            raise ValueError("slot-wire route exact postcheck is incomplete")
    contract = report["input_contract"]
    if (float(validation["minimum_core_center_distance_mm"]) + 1e-9
            < float(contract["required_core_center_distance_mm"])):
        raise ValueError("slot-wire route core clearance regressed")
    if (float(validation["minimum_copper_center_distance_mm"]) + 1e-9
            < float(contract["required_copper_center_distance_mm"])):
        raise ValueError("slot-wire route copper clearance regressed")


def write_report(report: dict[str, Any], path: Path = OUTPUT_PATH) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--packing", type=Path, default=PACKING_PATH)
    parser.add_argument("--output", type=Path, default=OUTPUT_PATH)
    parser.add_argument("--check", action="store_true",
                        help="regenerate in memory and require PASS")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()
    report = analyze(args.packing, progress=not args.quiet)
    if not args.check:
        write_report(report, args.output)
        print(f"wrote {args.output}")
    if report["status"] != "PASS":
        failures = sum(
            route["status"] != "PASS" for route in report["routes"])
        raise SystemExit(f"slot-wire routes FAIL: {failures} cases")
    print(
        "slot-wire routes PASS: "
        f"{report['validation']['passed_geometry_cases']}/100 poses, "
        f"{report['validation']['covered_direction_cases']}/200 directions, "
        f"core={report['validation']['minimum_core_center_distance_mm']:.9f} "
        f"mm, copper="
        f"{report['validation']['minimum_copper_center_distance_mm']:.9f} mm"
    )


if __name__ == "__main__":
    main()
