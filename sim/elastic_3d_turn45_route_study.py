"""Bounded, fail-closed 3D route study for the two turn-45 loci.

This file is intentionally isolated from production CAD, settings, and the
captured controller stream.  It asks a narrow question: can a smooth wire
centreline connect the existing guarded slot-mouth source to the exact packed
turn-45 target while preserving the real incoming tangent, a physically
meaningful terminal tangent, R >= 3 mm, bare-core clearance, progressive
copper clearance, and both M2 motion signs?

The search is constructive, not an existence proof over all possible curves.
It covers the shortest parent-contact route, the two standard biarc branches,
an analytically optimal shallow parabolic bow, a bounded clamped-quintic grid,
and an explicit line/circular-arc detour whose curvature is known exactly.
Every claimed passing clearance is lower-bounded for route and obstacle chord
error.  Missing equilibrium, material, or guide evidence fails closed.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable

import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
PACKING = REPORTS / "slot_packing.json"
ROUTES = REPORTS / "slot_wire_routes.json"
MANIFEST = ROOT / "out" / "links" / "manifest.json"
GOAL = ROOT.parent / "GOAL.md"
OUTPUT_JSON = REPORTS / "elastic_3d_turn45_route_study.json"
OUTPUT_MD = REPORTS / "elastic_3d_turn45_route_study.md"

if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

from params import DEFAULT_STATOR, PARAMS  # noqa: E402
from slot_route import (  # noqa: E402
    CopperField,
    PackingSupportGraph,
    SlotRoutePlanner,
    _loop_centerline,
    active_copper_before,
    exact_polyline_part_clearance,
    neighbor_prefill_copper,
)
from crown_routes import (  # noqa: E402
    CurrentHalfObstacle,
    _closed_polyline_phase_subpath,
    adjacent_self_clearance,
)
import elastic_wire_contact_study as elastic_contact  # noqa: E402


ELASTIC_CONTACT = elastic_contact.OUTPUT_JSON


SCHEMA = "elastic-3d-turn45-route-study/v1"
TURN_INDEX = 45
PARENT_TURN_INDEX = 44
WIRE_ROUTE_STEP_DEG = 0.125
OBSTACLE_STEP_DEG = 0.5
MINIMUM_BEND_RADIUS_MM = 3.0
R3_DETOUR_FIRST_DIRECTION_DEG = 345.0
QUINTIC_GRID_VALUES_MM = tuple(float(value) for value in np.geomspace(
    0.05, 50.0, 32))
QUINTIC_SAMPLE_COUNT = 513
CURVATURE_EPS = 1.0e-12


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: dict[str, Any]) -> str:
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _unit(vector: np.ndarray) -> np.ndarray:
    value = np.asarray(vector, dtype=float)
    length = float(np.linalg.norm(value))
    if length <= CURVATURE_EPS:
        raise ValueError("cannot normalize a zero vector")
    return value / length


def _angle_deg(left: np.ndarray, right: np.ndarray) -> float:
    return math.degrees(math.acos(float(np.clip(
        _unit(left) @ _unit(right), -1.0, 1.0))))


def _turn45_cases(route_report: dict[str, Any]) -> list[dict[str, Any]]:
    """Return the two current turn-45 rows independent of stored status.

    ``slot_wire_routes.json`` now passes the rigid geometry postcheck at all
    100 loci while remaining fail closed on sign-specific history, C1 bend
    continuity, and its physical error budget.  This R3 study still owns the
    same two turn-45 endpoint/obstacle families, so selecting report failures
    would incorrectly erase its input as soon as the rigid planner improved.
    """

    routes = route_report.get("routes")
    if not isinstance(routes, list):
        raise ValueError("slot-wire route table is missing")
    cases = [
        row for row in routes
        if int(row.get("turn_index", -1)) == TURN_INDEX
        and int(row.get("half_turn_index", -1)) in (0, 1)
    ]
    keys = {(int(row["turn_index"]), int(row["half_turn_index"]))
            for row in cases}
    if (len(cases) != 2
            or keys != {(TURN_INDEX, 0), (TURN_INDEX, 1)}):
        raise ValueError(f"turn-45 route coverage changed: {sorted(keys)}")
    return sorted(cases, key=lambda row: int(row["half_turn_index"]))


def _source_target_tangent(row: dict[str, Any]) \
        -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return the terminal end-plane source, target, and incoming tangent.

    The production route may contain any number of guide and torus samples
    before its final guarded end-plane approach.  Consequently, a fixed
    negative point index is not a geometric contract.  The source owned by
    this study is the first point in the trailing run coplanar with the packed
    target; the preceding point defines the tangent that the replacement R3
    curve must preserve.
    """

    half = int(row.get("half_turn_index", -1))
    if half not in (0, 1):
        raise ValueError("half-turn index must be 0 or 1")
    points = np.asarray(row["route"]["points_local_mm"], dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 3
            or not np.all(np.isfinite(points))):
        raise ValueError("stored route has no guarded mouth approach")
    target = points[-1]
    declared_target = np.asarray(row.get("target_local_mm"), dtype=float)
    if (declared_target.shape != (3,)
            or not np.allclose(target, declared_target, atol=1e-9,
                               rtol=0.0)):
        raise ValueError("stored route target changed")
    end_plane_z = (
        float(DEFAULT_STATOR.stack) / 2.0
        if half == 0 else -float(DEFAULT_STATOR.stack) / 2.0
    )
    if abs(float(target[2]) - end_plane_z) > 1e-9:
        raise ValueError("stored route target is not on the winding end plane")

    source_index = len(points) - 1
    while (source_index > 0
           and abs(float(points[source_index - 1, 2])
                   - end_plane_z) <= 1e-9):
        source_index -= 1
    if source_index <= 0 or source_index >= len(points) - 1:
        raise ValueError("stored route has no bounded end-plane approach")
    source = points[source_index]
    if float(np.linalg.norm(target[:2] - source[:2])) <= CURVATURE_EPS:
        raise ValueError("stored route end-plane approach has zero xy length")
    incoming = _unit(source - points[source_index - 1])
    return source, target, incoming


def _parent_endpoint_centers(graph: PackingSupportGraph,
                             half_turn_index: int) -> dict[int, np.ndarray]:
    side = -1.0 if int(half_turn_index) == 0 else 1.0
    axial = float(DEFAULT_STATOR.stack) / 2.0
    axial *= 1.0 if int(half_turn_index) == 0 else -1.0
    half_neck = max(2.5, float(DEFAULT_STATOR.od) * 0.07) / 2.0
    return {
        index: np.array((
            graph.turn(index).radial_mm,
            side * (half_neck + graph.turn(index).profile_radius_mm),
            axial,
        ), dtype=float)
        for index in graph.turn(TURN_INDEX).parent_turn_indices
    }


def _support_direction(graph: PackingSupportGraph, target: np.ndarray,
                       half_turn_index: int) -> tuple[np.ndarray, float]:
    centers = _parent_endpoint_centers(graph, half_turn_index)
    normals = np.asarray([
        _unit(target - center) for center in centers.values()
    ])
    # A half-degree deterministic search contains the exact 52.5 degree
    # solution for this three-parent hexagonal packing locus.
    candidates = np.asarray([
        (math.cos(math.radians(value)), math.sin(math.radians(value)), 0.0)
        for value in np.arange(0.0, 360.0, 0.5)
    ])
    scores = np.min(candidates @ normals.T, axis=1)
    index = int(np.argmax(scores))
    return candidates[index], float(scores[index])


def _arc_basis(start_tangent: np.ndarray, end_tangent: np.ndarray,
               *, long_way: bool = False) -> tuple[float, np.ndarray]:
    start = _unit(start_tangent)
    end = _unit(end_tangent)
    cosine = float(np.clip(start @ end, -1.0, 1.0))
    short = math.acos(cosine)
    if short <= 1e-10 or abs(short - math.pi) <= 1e-10:
        raise ValueError("arc endpoints require an explicit plane")
    angle = 2.0 * math.pi - short if long_way else short
    plane = _unit((end - math.cos(angle) * start) / math.sin(angle))
    return angle, plane


def _sample_arc(start: np.ndarray, start_tangent: np.ndarray,
                end_tangent: np.ndarray, radius_mm: float, step_deg: float,
                *, long_way: bool = False) -> np.ndarray:
    angle, plane = _arc_basis(
        start_tangent, end_tangent, long_way=long_way)
    count = max(2, math.ceil(math.degrees(angle) / float(step_deg)))
    values = np.linspace(0.0, angle, count + 1)
    return np.asarray(start, dtype=float) + float(radius_mm) * (
        np.sin(values)[:, None] * _unit(start_tangent)
        + (1.0 - np.cos(values))[:, None] * plane
    )


def _sample_semicircle(start: np.ndarray, start_tangent: np.ndarray,
                       displacement_direction: np.ndarray, radius_mm: float,
                       step_deg: float) -> np.ndarray:
    tangent = _unit(start_tangent)
    plane = _unit(displacement_direction)
    if abs(float(tangent @ plane)) > 1e-10:
        raise ValueError("semicircle plane is not normal to its tangent")
    count = max(2, math.ceil(180.0 / float(step_deg)))
    values = np.linspace(0.0, math.pi, count + 1)
    return np.asarray(start, dtype=float) + float(radius_mm) * (
        np.sin(values)[:, None] * tangent
        + (1.0 - np.cos(values))[:, None] * plane
    )


def _append(parts: list[np.ndarray], points: np.ndarray) -> np.ndarray:
    value = np.asarray(points, dtype=float)
    if value.ndim != 2 or value.shape[1] != 3 or len(value) < 2:
        raise ValueError("route part must be a 3D polyline")
    if parts and np.linalg.norm(parts[-1][-1] - value[0]) > 1e-8:
        raise ValueError("route parts do not share an endpoint")
    parts.append(value if not parts else value[1:])
    return value[-1]


def r3_multiarc_route(row: dict[str, Any], graph: PackingSupportGraph,
                      *, step_deg: float = WIRE_ROUTE_STEP_DEG
                      ) -> tuple[np.ndarray, dict[str, Any]]:
    """Explicit C1 arc/line detour with exact R >= 3 mm pieces.

    Half 1 is the exact (x,-y,-z) mirror of half 0 so the two audited loci
    cannot silently receive different topology.
    """

    half = int(row["half_turn_index"])
    if half == 1:
        route_report = _load(ROUTES)
        base = next(item for item in _turn45_cases(route_report)
                    if int(item["half_turn_index"]) == 0)
        route, meta = r3_multiarc_route(base, graph, step_deg=step_deg)
        mirror = np.array((1.0, -1.0, -1.0))
        mirrored = route * mirror
        result = dict(meta)
        result.update({
            "half_turn_index": 1,
            "source_local_mm": (np.asarray(meta["source_local_mm"])
                                * mirror).tolist(),
            "target_local_mm": (np.asarray(meta["target_local_mm"])
                                * mirror).tolist(),
            "incoming_tangent_local": (
                np.asarray(meta["incoming_tangent_local"]) * mirror).tolist(),
            "terminal_tangent_local": (
                np.asarray(meta["terminal_tangent_local"]) * mirror).tolist(),
            "support_direction_local": (
                np.asarray(meta["support_direction_local"]) * mirror).tolist(),
        })
        return mirrored, result

    if half != 0:
        raise ValueError("half-turn index must be 0 or 1")
    source, target, incoming = _source_target_tangent(row)
    radius = MINIMUM_BEND_RADIUS_MM
    up = np.array((0.0, 0.0, 1.0))
    down = -up
    first_direction = np.array((
        math.cos(math.radians(R3_DETOUR_FIRST_DIRECTION_DEG)),
        math.sin(math.radians(R3_DETOUR_FIRST_DIRECTION_DEG)),
        0.0,
    ))
    support, support_score = _support_direction(graph, target, half)
    terminal_entry = -support
    terminal_arc_start = target + radius * support + radius * up
    # Start of down -> terminal_entry quarter arc.
    preterminal_start = (
        terminal_arc_start - radius * (down + terminal_entry))

    parts: list[np.ndarray] = []
    point = source
    point = _append(parts, _sample_arc(
        point, incoming, first_direction, radius, step_deg))
    point = _append(parts, _sample_arc(
        point, first_direction, up, radius, step_deg))
    vertical_length = float(preterminal_start[2] - point[2])
    if vertical_length <= 0.0:
        raise RuntimeError("R3 detour lost its outward axial bridge")
    point = _append(parts, np.asarray((point, point + vertical_length * up)))
    horizontal = preterminal_start - point
    if abs(float(horizontal[2])) > 1e-8:
        raise RuntimeError("semicircle endpoints are not coplanar")
    middle_radius = float(np.linalg.norm(horizontal)) / 2.0
    if middle_radius + 1e-12 < radius:
        raise RuntimeError("middle detour arc violates R3")
    point = _append(parts, _sample_semicircle(
        point, up, horizontal, middle_radius, step_deg))
    point = _append(parts, _sample_arc(
        point, down, terminal_entry, radius, step_deg))
    point = _append(parts, _sample_arc(
        point, terminal_entry, down, radius, step_deg))
    route = np.vstack(parts)
    endpoint_error = float(np.linalg.norm(point - target))
    if endpoint_error > 1e-8:
        raise RuntimeError(f"R3 detour endpoint drifted {endpoint_error:.3g} mm")
    lengths = np.linalg.norm(route[1:] - route[:-1], axis=1)
    chord_error = max(radius, middle_radius) * (
        1.0 - math.cos(math.radians(step_deg) / 2.0))
    return route, {
        "family": "analytic_line_and_constant_radius_arcs",
        "half_turn_index": half,
        "source_local_mm": source.tolist(),
        "target_local_mm": target.tolist(),
        "incoming_tangent_local": incoming.tolist(),
        "terminal_tangent_local": down.tolist(),
        "support_direction_local": support.tolist(),
        "support_cone_minimum_dot": support_score,
        "piece_radii_mm": [radius, radius, middle_radius, radius, radius],
        "minimum_bend_radius_mm": min(radius, middle_radius),
        "vertical_bridge_length_mm": vertical_length,
        "length_mm": float(np.sum(lengths)),
        "source_target_chord_mm": float(np.linalg.norm(target - source)),
        "sample_step_deg": float(step_deg),
        "route_chord_error_bound_mm": chord_error,
        "endpoint_error_mm": endpoint_error,
        "analytic_source_tangent_error_deg": 0.0,
        "analytic_terminal_tangent_error_deg": 0.0,
        "point_count": len(route),
        "bounds_min_local_mm": np.min(route, axis=0).tolist(),
        "bounds_max_local_mm": np.max(route, axis=0).tolist(),
    }


def _standard_biarc_branches(source: np.ndarray, target: np.ndarray,
                             source_tangent: np.ndarray,
                             target_tangent: np.ndarray
                             ) -> list[dict[str, Any]]:
    """Return both standard equal-tangent-distance biarc solutions."""

    t0, t1 = _unit(source_tangent), _unit(target_tangent)
    chord = target - source
    a = 2.0 * (1.0 - float(t0 @ t1))
    b = 2.0 * float(chord @ (t0 + t1))
    c = -float(chord @ chord)
    discriminant = b * b - 4.0 * a * c
    if a <= CURVATURE_EPS or discriminant < 0.0:
        raise RuntimeError("standard biarc quadratic is degenerate")

    def radius(start: np.ndarray, tangent: np.ndarray,
               end: np.ndarray) -> float:
        local = end - start
        normal = local - float(local @ tangent) * tangent
        denominator = 2.0 * float(np.linalg.norm(normal))
        return math.inf if denominator <= CURVATURE_EPS else float(
            local @ local) / denominator

    rows = []
    for label, sign in (("principal", 1.0), ("alternate", -1.0)):
        distance = (-b + sign * math.sqrt(discriminant)) / (2.0 * a)
        join = 0.5 * (
            source + target + distance * (t0 - t1))
        first = radius(source, t0, join)
        second = radius(target, -t1, join)
        rows.append({
            "branch": label,
            "equal_tangent_distance_parameter_mm": distance,
            "join_local_mm": join.tolist(),
            "first_radius_mm": first,
            "second_radius_mm": second,
            "minimum_radius_mm": min(first, second),
            "meets_R3": min(first, second) + 1e-12 >= MINIMUM_BEND_RADIUS_MM,
        })
    return rows


def _shallow_bow_bounds(source: np.ndarray, target: np.ndarray,
                        blocking_parent_center: np.ndarray) -> dict[str, Any]:
    """Analytic best-case parabolic normal bow threshold.

    p(u) = source + u D + 4 A u(1-u) n.  This family maximizes displacement
    for a given quadratic second derivative and lets its terminal tangent
    rotate away from the blocking parent.  The terminal one-sided clearance
    condition is n dot p'(1) <= 0.
    """

    chord = target - source
    length = float(np.linalg.norm(chord))
    normal = _unit(target - blocking_parent_center)
    normal_chord = float(normal @ chord)
    cross = float(np.linalg.norm(np.cross(chord, normal)))
    terminal_safe_amplitude = max(0.0, normal_chord / 4.0)
    sine = cross / length
    r3_max_amplitude = (
        math.inf if sine <= CURVATURE_EPS else
        length * length / (8.0 * MINIMUM_BEND_RADIUS_MM * sine)
    )

    def midpoint_radius(amplitude: float) -> float:
        curvature = 8.0 * float(amplitude) * cross / (length ** 3)
        return math.inf if curvature <= CURVATURE_EPS else 1.0 / curvature

    radius_at_terminal_safe = midpoint_radius(terminal_safe_amplitude)
    r3_amplitude_unbounded = not math.isfinite(r3_max_amplitude)
    radius_at_terminal_safe_unbounded = not math.isfinite(
        radius_at_terminal_safe)
    terminal_safe_at_r3 = (
        True if r3_amplitude_unbounded else
        r3_max_amplitude + 1e-12 >= terminal_safe_amplitude
    )
    return {
        "family": "optimal_quadratic_normal_bow",
        "normal_local": normal.tolist(),
        "source_target_chord_mm": length,
        "terminal_safe_minimum_amplitude_mm": terminal_safe_amplitude,
        "R3_maximum_amplitude_mm": (
            None if r3_amplitude_unbounded else r3_max_amplitude),
        "R3_maximum_amplitude_unbounded": r3_amplitude_unbounded,
        "amplitude_gap_mm": (
            None if r3_amplitude_unbounded else
            terminal_safe_amplitude - r3_max_amplitude),
        "radius_at_terminal_safe_amplitude_mm": (
            None if radius_at_terminal_safe_unbounded else
            radius_at_terminal_safe),
        "radius_at_terminal_safe_amplitude_unbounded": (
            radius_at_terminal_safe_unbounded),
        "terminal_safe_at_R3_amplitude": terminal_safe_at_r3,
        "meets_terminal_clearance_and_R3": terminal_safe_at_r3,
        "endpoint_tangent_contract_evaluated": False,
        "qualifies_as_full_C1_route": False,
        "derivation": (
            "terminal safety requires n dot (D-4 A n)<=0; midpoint "
            "curvature is 8 A |D x n| / |D|^3. This bound does not "
            "enforce the required incoming and axial terminal tangents."),
    }


def _quintic_curve(source: np.ndarray, target: np.ndarray,
                    source_tangent: np.ndarray, target_tangent: np.ndarray,
                    source_handle_mm: float, target_handle_mm: float,
                    samples: int = QUINTIC_SAMPLE_COUNT
                    ) -> tuple[np.ndarray, float]:
    t0, t1 = _unit(source_tangent), _unit(target_tangent)
    a, b = float(source_handle_mm), float(target_handle_mm)
    controls = np.asarray((
        source, source + a * t0, source + 2.0 * a * t0,
        target - 2.0 * b * t1, target - b * t1, target,
    ))
    u = np.linspace(0.0, 1.0, int(samples))
    om = 1.0 - u
    curve = sum(
        math.comb(5, index)
        * (u ** index * om ** (5 - index))[:, None]
        * controls[index]
        for index in range(6)
    )
    first_controls = 5.0 * (controls[1:] - controls[:-1])
    first = sum(
        math.comb(4, index)
        * (u ** index * om ** (4 - index))[:, None]
        * first_controls[index]
        for index in range(5)
    )
    second_controls = 4.0 * (
        first_controls[1:] - first_controls[:-1])
    second = sum(
        math.comb(3, index)
        * (u ** index * om ** (3 - index))[:, None]
        * second_controls[index]
        for index in range(4)
    )
    speed = np.linalg.norm(first, axis=1)
    curvature = np.linalg.norm(np.cross(first, second), axis=1) / np.maximum(
        speed ** 3, 1e-30)
    maximum = float(np.max(curvature))
    return curve, math.inf if maximum <= CURVATURE_EPS else 1.0 / maximum


def _bounded_quintic_search(source: np.ndarray, target: np.ndarray,
                             incoming: np.ndarray,
                             terminal: np.ndarray,
                             parent_field: CopperField) -> dict[str, Any]:
    best: dict[str, Any] | None = None
    r3_count = 0
    for source_handle in QUINTIC_GRID_VALUES_MM:
        for target_handle in QUINTIC_GRID_VALUES_MM:
            curve, radius = _quintic_curve(
                source, target, incoming, terminal,
                source_handle, target_handle)
            if radius + 1e-9 < MINIMUM_BEND_RADIUS_MM:
                continue
            r3_count += 1
            clearance = parent_field.clearance(curve, 0.7)
            row = {
                "source_handle_mm": source_handle,
                "target_handle_mm": target_handle,
                "sampled_minimum_radius_mm": radius,
                "parent_raw_centerline_distance_mm": (
                    clearance.minimum_centerline_distance_mm),
                "parent_obstacle_id": clearance.obstacle_id,
                "bounds_min_local_mm": np.min(curve, axis=0).tolist(),
                "bounds_max_local_mm": np.max(curve, axis=0).tolist(),
            }
            if (best is None or row["parent_raw_centerline_distance_mm"]
                    > best["parent_raw_centerline_distance_mm"]):
                best = row
    return {
        "family": "zero_endpoint_curvature_clamped_quintic",
        "source_handle_grid_mm": list(QUINTIC_GRID_VALUES_MM),
        "target_handle_grid_mm": list(QUINTIC_GRID_VALUES_MM),
        "grid_case_count": len(QUINTIC_GRID_VALUES_MM) ** 2,
        "sample_count_per_curve": QUINTIC_SAMPLE_COUNT,
        "R3_candidate_count": r3_count,
        "best_R3_candidate": best,
        "required_parent_centerline_distance_mm": float(
            DEFAULT_STATOR.wire_d),
        "found_parent_clear_R3_candidate": bool(
            best is not None
            and best["parent_raw_centerline_distance_mm"] + 1e-9
            >= float(DEFAULT_STATOR.wire_d)),
        "exhaustiveness": (
            "bounded logarithmic handle grid; not a proof over arbitrary "
            "spline control polygons"),
    }


def _flat_current_half(graph: PackingSupportGraph, half: int, sign: int,
                       target: np.ndarray) -> CurrentHalfObstacle:
    if half not in (0, 1) or sign not in (-1, 1):
        raise ValueError("invalid current-half key")
    turn = graph.turn(TURN_INDEX)
    end_phase = float(sign * half) * math.pi
    start_phase = end_phase - float(sign) * math.pi
    loop = _loop_centerline(
        turn, DEFAULT_STATOR, arc_step_deg=OBSTACLE_STEP_DEG)
    phase_zero = np.array((
        turn.radial_mm,
        -max(2.5, float(DEFAULT_STATOR.od) * 0.07) / 2.0
        - turn.profile_radius_mm,
        float(DEFAULT_STATOR.stack) / 2.0,
    ))
    points = _closed_polyline_phase_subpath(
        loop, start_phase, end_phase, phase_zero)
    if np.linalg.norm(points[-1] - target) > 2e-5:
        raise RuntimeError("flat current-half endpoint does not match target")
    points[-1] = target
    lengths = np.linalg.norm(points[1:] - points[:-1], axis=1)
    return CurrentHalfObstacle(
        turn_index=TURN_INDEX,
        physical_half_index=half,
        motion_sign=sign,
        start_phase_rad=start_phase,
        end_phase_rad=end_phase,
        points_local_mm=tuple(tuple(map(float, point)) for point in points),
        length_mm=float(np.sum(lengths)),
        sha256=hashlib.sha256(np.asarray(
            points, dtype="<f8").tobytes()).hexdigest(),
    )


def _audit_multiarc(route: np.ndarray, metadata: dict[str, Any],
                    planner: SlotRoutePlanner, graph: PackingSupportGraph,
                    nonparent_field: CopperField,
                    parent_field: CopperField) -> dict[str, Any]:
    route_error = float(metadata["route_chord_error_bound_mm"])
    obstacle_radius = max(turn.profile_radius_mm for turn in graph.turns)
    obstacle_error = obstacle_radius * (
        1.0 - math.cos(math.radians(OBSTACLE_STEP_DEG) / 2.0))
    core_raw = exact_polyline_part_clearance(route, planner.stator_part)
    nonparent = nonparent_field.clearance(route, 0.7)
    parent = parent_field.clearance(route, 0.7)
    core_lower = core_raw - route_error
    nonparent_lower = (
        nonparent.minimum_centerline_distance_mm
        - route_error - obstacle_error)
    parent_lower = (
        parent.minimum_centerline_distance_mm
        - route_error - obstacle_error)
    target = np.asarray(metadata["target_local_mm"], dtype=float)
    motion_rows = []
    for sign in (-1, 1):
        current = _flat_current_half(
            graph, int(metadata["half_turn_index"]), sign, target)
        clearance = adjacent_self_clearance(
            route, current, graph.wire_diameter_mm,
            search_band_mm=0.7)
        lower = (
            clearance.minimum_centerline_distance_mm
            - route_error - obstacle_error)
        motion_rows.append({
            "motion_sign": sign,
            "current_half_sha256": current.sha256,
            "current_half_length_mm": current.length_mm,
            "raw_centerline_distance_mm": (
                clearance.minimum_centerline_distance_mm),
            "continuous_lower_bound_mm": lower,
            "required_centerline_distance_mm": graph.wire_diameter_mm,
            "adjacency_limit_mm": clearance.adjacency_limit_mm,
            "combined_geodesic_to_endpoint_mm": (
                clearance.combined_geodesic_to_endpoint_mm),
            "status": "PASS" if lower + 1e-9 >= graph.wire_diameter_mm
                      else "FAIL",
        })
    checks = {
        "exact_endpoint": float(metadata["endpoint_error_mm"]) <= 1e-8,
        "source_C1_tangent": (
            float(metadata["analytic_source_tangent_error_deg"]) <= 1e-9),
        "terminal_C1_axial_tangent": (
            float(metadata["analytic_terminal_tangent_error_deg"]) <= 1e-9),
        "minimum_bend_radius_R3": (
            float(metadata["minimum_bend_radius_mm"]) + 1e-12
            >= MINIMUM_BEND_RADIUS_MM),
        "exact_OCC_core_lower_bound": (
            core_lower + 1e-9 >= graph.center_core_access_mm),
        "all_prior_nonparent_copper_lower_bound": (
            nonparent_lower + 1e-9 >= graph.wire_diameter_mm),
        "declared_parent_prefix_no_penetration": (
            parent_lower + 1e-9 >= graph.wire_diameter_mm),
        "both_motion_sign_current_half_clearance": all(
            row["status"] == "PASS" for row in motion_rows),
    }
    return {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "core": {
            "exact_OCC_raw_mm": core_raw,
            "continuous_lower_bound_mm": core_lower,
            "required_center_distance_mm": graph.center_core_access_mm,
        },
        "nonparent_copper": {
            "raw_chordal_centerline_distance_mm": (
                nonparent.minimum_centerline_distance_mm),
            "continuous_lower_bound_mm": nonparent_lower,
            "required_centerline_distance_mm": graph.wire_diameter_mm,
            "obstacle_id": nonparent.obstacle_id,
            "route_segment_index": nonparent.route_segment_index,
        },
        "declared_parent_prefix": {
            "raw_chordal_centerline_distance_mm": (
                parent.minimum_centerline_distance_mm),
            "continuous_lower_bound_mm": parent_lower,
            "required_centerline_distance_mm": graph.wire_diameter_mm,
            "obstacle_id": parent.obstacle_id,
            "route_segment_index": parent.route_segment_index,
            "endpoint_contact_is_intentional": True,
            "interpretation": (
                "the minimum occurs before the shared endpoint; endpoint "
                "contact does not excuse prefix penetration"),
        },
        "motion_sign_cases": motion_rows,
        "numerical_error_budget_mm": {
            "route_chord": route_error,
            "obstacle_chord": obstacle_error,
            "sum": route_error + obstacle_error,
        },
    }


def _convergence(row: dict[str, Any], graph: PackingSupportGraph
                 ) -> list[dict[str, Any]]:
    result = []
    prior_length = None
    for step in (1.0, 0.5, 0.25, 0.125):
        route, meta = r3_multiarc_route(row, graph, step_deg=step)
        length = float(np.sum(np.linalg.norm(
            route[1:] - route[:-1], axis=1)))
        result.append({
            "step_deg": step,
            "point_count": len(route),
            "polyline_length_mm": length,
            "change_from_previous_length_mm": (
                None if prior_length is None else length - prior_length),
            "analytic_curve_length_mm": meta["length_mm"],
            "route_chord_error_bound_mm": (
                meta["route_chord_error_bound_mm"]),
            "endpoint_error_mm": meta["endpoint_error_mm"],
        })
        prior_length = length
    return result


def analyze() -> dict[str, Any]:
    contact_report = _load(ELASTIC_CONTACT)
    elastic_contact.validate_report_integrity(contact_report)
    packing = _load(PACKING)
    route_report = _load(ROUTES)
    graph = PackingSupportGraph.from_report(packing, spec=DEFAULT_STATOR)
    turn45_cases = _turn45_cases(route_report)
    manifest = _load(MANIFEST)
    planner = SlotRoutePlanner.from_project(
        manifest, spec=DEFAULT_STATOR,
        access_radius_mm=graph.center_core_access_mm,
        planner_offset_mm=graph.center_core_access_mm)

    active = active_copper_before(
        graph, TURN_INDEX, DEFAULT_STATOR,
        arc_step_deg=OBSTACLE_STEP_DEG)
    neighbors = (
        *neighbor_prefill_copper(
            graph, DEFAULT_STATOR, -1,
            arc_step_deg=OBSTACLE_STEP_DEG),
        *neighbor_prefill_copper(
            graph, DEFAULT_STATOR, 1,
            arc_step_deg=OBSTACLE_STEP_DEG),
    )
    parent_ids = {
        f"active-turn-{index:02d}"
        for index in graph.turn(TURN_INDEX).parent_turn_indices
    }
    parent_field = CopperField(tuple(
        obstacle for obstacle in active
        if obstacle.obstacle_id in parent_ids))
    nonparent_field = CopperField(tuple(
        obstacle for obstacle in (*active, *neighbors)
        if obstacle.obstacle_id not in parent_ids))

    cases = []
    finest_routes: list[np.ndarray] = []
    for row in turn45_cases:
        half = int(row["half_turn_index"])
        source, target, incoming = _source_target_tangent(row)
        axial_sign = 1.0 if half == 0 else -1.0
        terminal_axial = np.array((0.0, 0.0, -axial_sign))
        support, support_score = _support_direction(graph, target, half)
        centers = _parent_endpoint_centers(graph, half)
        contact_route, contact_meta = elastic_contact.contact_detour(
            row, graph)
        multiarc, multiarc_meta = r3_multiarc_route(row, graph)
        audit = _audit_multiarc(
            multiarc, multiarc_meta, planner, graph,
            nonparent_field, parent_field)
        finest_routes.append(multiarc)
        shallow = _shallow_bow_bounds(
            source, target, centers[PARENT_TURN_INDEX])
        biarcs = _standard_biarc_branches(
            source, target, incoming, terminal_axial)
        quintic = _bounded_quintic_search(
            source, target, incoming, terminal_axial, parent_field)
        cases.append({
            "turn_index": TURN_INDEX,
            "half_turn_index": half,
            "stored_route_status": row.get("status"),
            "source_local_mm": source.tolist(),
            "target_local_mm": target.tolist(),
            "source_target_chord_mm": float(np.linalg.norm(target - source)),
            "incoming_tangent_local": incoming.tolist(),
            "required_terminal_axial_tangent_local": terminal_axial.tolist(),
            "support_direction_local": support.tolist(),
            "support_cone_minimum_dot": support_score,
            "declared_parent_turn_indices": list(
                graph.turn(TURN_INDEX).parent_turn_indices),
            "declared_parent_endpoint_centers_local_mm": {
                str(index): center.tolist()
                for index, center in centers.items()
            },
            "shortest_parent_contact_route": {
                "status": "FAIL",
                "geometrically_nonpenetrating": True,
                "analytic_minimum_radius_mm": contact_meta[
                    "analytic_local_bend_radius_mm"],
                "required_radius_mm": MINIMUM_BEND_RADIUS_MM,
                "point_count": len(contact_route),
                "terminal_tangent_is_deposited_loop_C1": False,
            },
            "standard_biarc": {
                "status": "FAIL" if not all(
                    branch["meets_R3"] for branch in biarcs) else "PASS",
                "terminal_tangent": "required axial deposited-loop tangent",
                "branches": biarcs,
            },
            "shallow_normal_bow": {
                "status": "DIAGNOSTIC_ONLY",
                **shallow,
            },
            "bounded_quintic_search": {
                "status": "PASS" if quintic[
                    "found_parent_clear_R3_candidate"] else "FAIL",
                **quintic,
            },
            "analytic_R3_multiarc": {
                **multiarc_meta,
                "route_points_local_mm": multiarc.tolist(),
                "audit": audit,
            },
            "convergence": _convergence(row, graph),
        })

    mirror = np.array((1.0, -1.0, -1.0))
    mirror_error = float(np.max(np.abs(
        finest_routes[1] - finest_routes[0] * mirror)))
    all_audits_pass = all(
        case["analytic_R3_multiarc"]["audit"]["status"] == "PASS"
        for case in cases)
    all_bounded_family_pass = all(any((
        case["standard_biarc"]["status"] == "PASS",
        case["bounded_quintic_search"]["status"] == "PASS",
        case["analytic_R3_multiarc"]["audit"]["status"] == "PASS",
    )) for case in cases)
    longest_constructed_route_mm = max(
        case["analytic_R3_multiarc"]["length_mm"] for case in cases)
    longest_source_target_chord_mm = max(
        case["source_target_chord_mm"] for case in cases)
    free_equilibrium = {
        "status": "FAIL",
        "plausible_free_elastic_equilibrium": False,
        "guide_or_former_required": True,
        "reason": (
            f"The explicit R3 curve is {longest_constructed_route_mm:.3f} "
            f"mm long across a {longest_source_target_chord_mm:.3f} mm "
            "chord and contains five prescribed constant-curvature arcs. "
            "A moment-free wire span under the machine's tensile preload "
            "shortcuts toward the taut/contact branch; no distributed guide "
            "reaction, endpoint bending clamp, or stable elastica branch is "
            "present to hold this detour."),
        "unclosed_physical_gates": [
            "no guide/contact surface matching the constructed arc chain",
            "no measured EI, yield, residual-set, or enamel abrasion curve",
            "no friction/contact stability proof through M2 reversal",
            "no dynamic rod solve bound to the raw tension history",
        ],
    }
    release_flags = {
        "two_exact_turn45_loci_covered": len(cases) == 2,
        "exact_half_turn_mirror": mirror_error <= 1e-10,
        "both_motion_signs_explicit": all(
            {row["motion_sign"] for row in case[
                "analytic_R3_multiarc"]["audit"]["motion_sign_cases"]}
            == {-1, 1} for case in cases),
        "bounded_family_found_R3_clear_route": all_bounded_family_pass,
        "explicit_R3_multiarc_all_physical_clearances": all_audits_pass,
        "plausible_free_elastic_equilibrium": False,
        "physical_former_geometry_bound": False,
        "complete_material_and_dynamic_error_budget": False,
    }
    status = "PASS" if all(release_flags.values()) else "FAIL"
    payload: dict[str, Any] = {
        "schema": SCHEMA,
        "status": status,
        "decision": "NO_PROVEN_FREE_R3_ROUTE__GUIDE_OR_FORMER_REQUIRED",
        "production_authorized": False,
        "scope": {
            "question": (
                "Can a bounded explicit C1 curve provide an independent R3 "
                "route for the two current turn-45 endpoint/obstacle "
                "families while clearing source CAD and the complete "
                "progressive copper field for both signs?"),
            "answer": (
                "No within the tested constructive families. The shortest "
                "nonpenetrating contact arc is R0.22352. Standard biarcs and "
                "the axial-tangent clamped-quintic grid find no parent-clear "
                "R3 route. The shallow-bow calculation is only a curvature/"
                "clearance bound and does not enforce the endpoint tangents. "
                "An explicit all-R3 arc chain clears steel but intersects "
                "turn-43 and the turn-44 parent prefix. It is also not a "
                "plausible un-guided tensile equilibrium."),
            "production_sources_modified": False,
            "search_is_global_nonexistence_proof": False,
            "alternate_raw_compatible_wire_distribution_rejected": False,
            "scope_boundary": (
                "This result audits the current hash-bound turn-45 packed "
                "endpoint/obstacle families independently of whether their "
                "stored rigid-route rows pass. It does not reject a different "
                "constructive wire distribution that is proved against the "
                "authoritative raw motion."),
            "bounded_families": [
                "analytic parent tangent/contact arc",
                "both standard biarc branches",
                "diagnostic shallow quadratic normal-bow bound",
                "32x32 axial-terminal zero-curvature clamped-quintic grid",
                "explicit line plus five constant-radius arcs",
            ],
        },
        "release_flags": release_flags,
        "release_blockers": [
            name for name, passed in release_flags.items() if not passed
        ],
        "geometry_contract": {
            "minimum_bend_radius_mm": MINIMUM_BEND_RADIUS_MM,
            "wire_finished_diameter_mm": graph.wire_diameter_mm,
            "required_core_center_distance_mm": graph.center_core_access_mm,
            "route_step_deg": WIRE_ROUTE_STEP_DEG,
            "obstacle_step_deg": OBSTACLE_STEP_DEG,
            "mirror_transform": [1.0, -1.0, -1.0],
            "mirror_max_abs_error_mm": mirror_error,
        },
        "cases": cases,
        "free_equilibrium_assessment": free_equilibrium,
        "model_dependency_warning": {
            "status": "UNRESOLVED",
            "current_obstacle_family": (
                "slot_route rounded box using packing profile radii"),
            "current_profile_radius_range_mm": [
                min(turn.profile_radius_mm for turn in graph.turns),
                max(turn.profile_radius_mm for turn in graph.turns),
            ],
            "physical_former_hypothesis_mm": 2.75,
            "effect": (
                "The exact result is authoritative only for the current "
                "hash-bound packed-loop field. A literal R2.75 former plus "
                "wire-radius buffered rectangle/crown changes end-turn "
                "obstacles and must be regenerated and re-audited; it is not "
                "credited as extra clearance here."),
        },
        "source_hashes": {
            "elastic_contact_file_sha256": _sha256(ELASTIC_CONTACT),
            "elastic_contact_report_sha256": contact_report[
                "report_sha256"],
            "elastic_contact_source_sha256": _sha256(
                Path(elastic_contact.__file__)),
            "packing_file_sha256": _sha256(PACKING),
            "packing_report_sha256": packing["report_sha256"],
            "slot_wire_routes_file_sha256": _sha256(ROUTES),
            "slot_wire_routes_report_sha256": route_report["report_sha256"],
            "manifest_sha256": _sha256(MANIFEST),
            "goal_sha256": _sha256(GOAL),
            "slot_route_source_sha256": _sha256(HERE / "slot_route.py"),
            "crown_routes_source_sha256": _sha256(HERE / "crown_routes.py"),
            "study_source_sha256": _sha256(Path(__file__)),
        },
        "limitations": [
            "The spline grid is bounded and cannot prove global curve "
            "nonexistence in unbounded free space.",
            "A different raw-compatible packing/contact distribution remains "
            "an open architecture, not a result disproved by this report.",
            "Exact OCC checks cover the source stator CAD; the wire and "
            "progressive copper lower bounds include route and obstacle "
            "polyline chord error.",
            "The study is quasi-static geometry plus equilibrium screening, "
            "not nonlinear 3D rod FEA.",
            "No material deformation, enamel compression, plastic hinge, or "
            "frictional branch selection is credited.",
        ],
    }
    payload["report_sha256"] = _canonical_hash(payload)
    return payload


def validate_report_integrity(report: dict[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported 3D turn-45 route study schema")
    payload = dict(report)
    expected = payload.pop("report_sha256", None)
    if not isinstance(expected, str) or _canonical_hash(payload) != expected:
        raise ValueError("3D turn-45 route report hash mismatch")
    contact_report = _load(ELASTIC_CONTACT)
    elastic_contact.validate_report_integrity(contact_report)
    current = {
        "elastic_contact_file_sha256": _sha256(ELASTIC_CONTACT),
        "elastic_contact_report_sha256": contact_report.get(
            "report_sha256"),
        "elastic_contact_source_sha256": _sha256(
            Path(elastic_contact.__file__)),
        "packing_file_sha256": _sha256(PACKING),
        "slot_wire_routes_file_sha256": _sha256(ROUTES),
        "manifest_sha256": _sha256(MANIFEST),
        "goal_sha256": _sha256(GOAL),
        "slot_route_source_sha256": _sha256(HERE / "slot_route.py"),
        "crown_routes_source_sha256": _sha256(HERE / "crown_routes.py"),
        "study_source_sha256": _sha256(Path(__file__)),
    }
    actual = report.get("source_hashes", {})
    stale = [name for name, value in current.items()
             if actual.get(name) != value]
    if stale:
        raise ValueError("3D turn-45 route report has stale sources: "
                         + ", ".join(stale))


def render_markdown(report: dict[str, Any]) -> str:
    lines = [
        "# Bounded 3D elastic route study — turn 45",
        "",
        f"**Status: {report['status']}**  ",
        f"**Decision: {report['decision']}**  ",
        "**Production authorized: no**",
        "",
        report["scope"]["answer"],
        "",
        "## Constructive results",
        "",
    ]
    for case in report["cases"]:
        arc = case["analytic_R3_multiarc"]
        audit = arc["audit"]
        shallow_r3_maximum = case["shallow_normal_bow"][
            "R3_maximum_amplitude_mm"]
        shallow_r3_maximum_text = (
            "unbounded" if shallow_r3_maximum is None else
            f"{shallow_r3_maximum:.6f} mm"
        )
        lines.extend((
            f"### Turn 45 / half {case['half_turn_index']}",
            "",
            f"- Mouth-to-target chord: {case['source_target_chord_mm']:.6f} mm",
            f"- Contact-arc radius: {case['shortest_parent_contact_route']['analytic_minimum_radius_mm']:.6f} mm (FAIL R3)",
            f"- Best standard-biarc minimum radius: {max(branch['minimum_radius_mm'] for branch in case['standard_biarc']['branches']):.6f} mm",
            f"- Shallow-bow terminal-safe amplitude: {case['shallow_normal_bow']['terminal_safe_minimum_amplitude_mm']:.6f} mm",
            "- Shallow-bow R3 maximum amplitude: "
            f"{shallow_r3_maximum_text}",
            f"- Explicit multiarc minimum radius: {arc['minimum_bend_radius_mm']:.6f} mm",
            f"- Exact OCC core lower bound: {audit['core']['continuous_lower_bound_mm']:.6f} mm (required {audit['core']['required_center_distance_mm']:.6f})",
            f"- Nonparent copper lower bound: {audit['nonparent_copper']['continuous_lower_bound_mm']:.6f} mm at `{audit['nonparent_copper']['obstacle_id']}` (required {audit['nonparent_copper']['required_centerline_distance_mm']:.6f})",
            f"- Parent-prefix lower bound: {audit['declared_parent_prefix']['continuous_lower_bound_mm']:.6f} mm at `{audit['declared_parent_prefix']['obstacle_id']}`",
            f"- Motion signs: {', '.join(str(row['motion_sign']) + ':' + row['status'] for row in audit['motion_sign_cases'])}",
            "",
        ))
    lines.extend((
        "## Equilibrium decision",
        "",
        report["free_equilibrium_assessment"]["reason"],
        "",
        "A guide or former is therefore required before this branch can be "
        "treated as a physical route. No CAD, settings, capture, BOM, or "
        "production authority was changed by this study.",
        "",
        "## Model warning",
        "",
        report["model_dependency_warning"]["effect"],
        "",
    ))
    return "\n".join(lines)


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", type=Path, default=OUTPUT_JSON)
    parser.add_argument("--markdown", type=Path, default=OUTPUT_MD)
    args = parser.parse_args(list(argv) if argv is not None else None)
    report = analyze()
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8")
    args.markdown.parent.mkdir(parents=True, exist_ok=True)
    args.markdown.write_text(render_markdown(report), encoding="utf-8")
    print(
        f"3D turn-45 route study {report['status']}: "
        f"production_authorized={report['production_authorized']}")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
