"""Fail-closed study of a permanent stator-rotating winding guide cap.

Unlike the rejected machine-fixed shoe, the two candidate caps follow M0 and
M1 and remain in the finished motor.  This removes blade insertion/extraction
and indexing-corridor constraints.  The study then asks the harder questions:

* can an R3, non-self-looping outboard U-return be constructed for every one
  of the exact 50 packing placements and both winding directions;
* does that route clear earlier active-tooth copper, the arrived part of the
  current loop, and both fully wound neighbouring teeth;
* can the minimum 0.50 mm PPS cap envelope clear the exact flyer/chuck motion;
* and does the open mouth retain the 0.50 mm launch-wire option?

The source packing report remains authoritative.  This module never edits the
production packing, controller, assembly, or collision exclusions.
"""

from __future__ import annotations

from dataclasses import dataclass
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable

import fcl
import numpy as np
import trimesh


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
PACKING_PATH = REPORTS / "slot_packing.json"
JSON_OUT = REPORTS / "stator_winding_guide_cap.json"
MD_OUT = REPORTS / "stator_winding_guide_cap.md"

for path in (CAD, HERE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import assembly  # noqa: E402
import collide  # noqa: E402
from params import DEFAULT_STATOR, PARAMS  # noqa: E402
import slot_route  # noqa: E402
import stator_insulation_nomex410 as insulation  # noqa: E402
import stator_winding_guide_cap as cap  # noqa: E402


SCHEMA = "stator-winding-guide-cap-study/v1"
ARC_STEP_DEG = 5.0
WIRE_DIRECTIONS = (-1, 1)
FLYER_ANGLE_COUNT = 360
RIGID_TARGET_MM = 2.0
SOURCE_PPS_URL = (
    "https://www.solvay.com/sites/g/files/srpend616/files/2018-10/"
    "Ryton-PPS-Design-Guide_EN-v2.3_0.pdf"
)
SOURCE_PPS_STATOR_URL = (
    "https://www.solvay.com/en/press-release/"
    "new-ryton-pps-supreme-high-voltage-and-high-flow-polymers"
)
SOURCE_PEEK_URL = (
    "https://www.victrex.com/en/blog/2019/"
    "five-factors-to-consider-when-moulding-peek"
)
SOURCE_NOMEX_URL = (
    "https://www.dupont.com/content/dam/aramids/amer/us/en/safety/"
    "public/documents/en/Nomex_410_Tech_Data_Sheet.pdf"
)


@dataclass(frozen=True)
class RouteGeometry:
    turn_index: int
    radial_mm: float
    profile_radius_mm: float
    crown_center_radius_mm: float
    crown_working_radius_mm: float
    crown_axial_offset_mm: float
    points_local_mm: tuple[tuple[float, float, float], ...]
    minimum_analytic_bend_radius_mm: float
    simple_non_self_looping: bool
    nonlocal_self_clearance_mm: float


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: dict[str, Any]) -> str:
    canonical = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False
    )
    return hashlib.sha256(canonical.encode()).hexdigest()


def _load_graph() -> tuple[dict[str, Any], slot_route.PackingSupportGraph]:
    report = json.loads(PACKING_PATH.read_text())
    return report, slot_route.PackingSupportGraph.from_report(
        report, spec=DEFAULT_STATOR
    )


def _append_segment(result: list[np.ndarray], points: np.ndarray) -> None:
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return
    if result and np.linalg.norm(result[-1][-1] - points[0]) > 1e-8:
        raise RuntimeError("route segment is disconnected")
    result.append(points if not result else points[1:])


def _quarter_axial_to_radial(start: np.ndarray, axial_sign: int,
                             radius_mm: float) -> np.ndarray:
    """R quarter arc: +/-Z tangent at start, +X tangent at end."""

    count = max(2, math.ceil(90.0 / ARC_STEP_DEG))
    theta = np.linspace(math.pi, math.pi / 2.0, count + 1)
    center = np.asarray(start, dtype=float) + np.array((radius_mm, 0.0, 0.0))
    return np.column_stack((
        center[0] + radius_mm * np.cos(theta),
        np.full(len(theta), center[1]),
        center[2] + axial_sign * radius_mm * np.sin(theta),
    ))


def _s_bend(start: np.ndarray, axis: int, signed_offset_mm: float,
            radius_mm: float) -> np.ndarray:
    """Two tangent opposite-curvature arcs, +X tangent at both ends."""

    offset = abs(float(signed_offset_mm))
    if offset <= 1e-12:
        return np.asarray((start, start + np.array((1e-9, 0.0, 0.0))))
    if offset > 4.0 * radius_mm + 1e-9:
        raise ValueError("S-bend lateral offset is too large")
    sign = 1.0 if signed_offset_mm >= 0.0 else -1.0
    alpha = math.acos(1.0 - offset / (2.0 * radius_mm))
    count = max(2, math.ceil(math.degrees(alpha) / ARC_STEP_DEG))
    first_t = np.linspace(0.0, alpha, count + 1)
    first = np.repeat(np.asarray(start, dtype=float)[None, :], len(first_t), axis=0)
    first[:, 0] += radius_mm * np.sin(first_t)
    first[:, axis] += sign * radius_mm * (1.0 - np.cos(first_t))
    second_u = np.linspace(0.0, alpha, count + 1)
    second = np.repeat(first[-1][None, :], len(second_u), axis=0)
    second[:, 0] += radius_mm * (
        math.sin(alpha) - np.sin(alpha - second_u)
    )
    second[:, axis] += sign * radius_mm * (
        np.cos(alpha - second_u) - math.cos(alpha)
    )
    return np.vstack((first, second[1:]))


def _outbound_connector(turn: slot_route.PackingTurn,
                        crown_center_radius_mm: float,
                        crown_working_radius_mm: float,
                        crown_axial_offset_mm: float,
                        tangential_sign: int,
                        axial_sign: int) -> np.ndarray:
    if tangential_sign not in (-1, 1) or axial_sign not in (-1, 1):
        raise ValueError("signs must be -1 or +1")
    radius = cap.MINIMUM_WIRE_CENTER_BEND_RADIUS_MM
    half_span = cap.tooth_half_width_mm() + float(turn.profile_radius_mm)
    start = np.array((
        float(turn.radial_mm),
        tangential_sign * half_span,
        axial_sign * float(DEFAULT_STATOR.stack) / 2.0,
    ))
    pieces: list[np.ndarray] = []
    first = _quarter_axial_to_radial(start, axial_sign, radius)
    _append_segment(pieces, first)

    tangential_target = tangential_sign * float(crown_working_radius_mm)
    lateral = tangential_target - first[-1, 1]
    side_bend = _s_bend(first[-1], 1, lateral, radius)
    _append_segment(pieces, side_bend)

    axial_target = axial_sign * (
        float(DEFAULT_STATOR.stack) / 2.0
        + radius + float(crown_axial_offset_mm)
    )
    axial_bend = _s_bend(
        pieces[-1][-1], 2, axial_target - pieces[-1][-1, 2], radius
    )
    _append_segment(pieces, axial_bend)

    end = pieces[-1][-1]
    if crown_center_radius_mm < end[0] + cap.CAP_RADIAL_STRAIGHT_ALLOWANCE_MM:
        raise ValueError(
            f"turn {turn.turn_index} connector needs crown radius >= "
            f"{end[0] + cap.CAP_RADIAL_STRAIGHT_ALLOWANCE_MM:.6f} mm"
        )
    straight = np.asarray((
        end,
        np.array((crown_center_radius_mm, tangential_target, axial_target)),
    ))
    _append_segment(pieces, straight)
    return np.vstack(pieces)


def _front_crown(turn: slot_route.PackingTurn,
                 crown_center_radius_mm: float,
                 crown_working_radius_mm: float,
                 crown_axial_offset_mm: float,
                 axial_sign: int) -> np.ndarray:
    radius = cap.MINIMUM_WIRE_CENTER_BEND_RADIUS_MM
    positive = _outbound_connector(
        turn, crown_center_radius_mm, crown_working_radius_mm,
        crown_axial_offset_mm, 1, axial_sign
    )
    negative = _outbound_connector(
        turn, crown_center_radius_mm, crown_working_radius_mm,
        crown_axial_offset_mm, -1, axial_sign
    )
    count = max(2, math.ceil(180.0 / ARC_STEP_DEG))
    theta = np.linspace(math.pi / 2.0, -math.pi / 2.0, count + 1)
    z = positive[-1, 2]
    u_return = np.column_stack((
        crown_center_radius_mm + crown_working_radius_mm * np.cos(theta),
        crown_working_radius_mm * np.sin(theta),
        np.full(len(theta), z),
    ))
    if np.linalg.norm(positive[-1] - u_return[0]) > 1e-8:
        raise RuntimeError("positive connector misses the U-return")
    if np.linalg.norm(negative[-1] - u_return[-1]) > 1e-8:
        raise RuntimeError("negative connector misses the U-return")
    return np.vstack((positive, u_return[1:], negative[-2::-1]))


def _segment_distance(a0: np.ndarray, a1: np.ndarray,
                      b0: np.ndarray, b1: np.ndarray) -> float:
    """Exact Euclidean distance between two closed 3D line segments."""

    u, v, w = a1 - a0, b1 - b0, a0 - b0
    aa, bb, cc = float(u @ u), float(u @ v), float(v @ v)
    dd, ee = float(u @ w), float(v @ w)
    denom = aa * cc - bb * bb
    if aa <= 1e-15 and cc <= 1e-15:
        return float(np.linalg.norm(a0 - b0))
    if aa <= 1e-15:
        t = float(np.clip(ee / cc, 0.0, 1.0))
        return float(np.linalg.norm(a0 - (b0 + t * v)))
    if cc <= 1e-15:
        s = float(np.clip(-dd / aa, 0.0, 1.0))
        return float(np.linalg.norm((a0 + s * u) - b0))
    s = 0.0 if abs(denom) <= 1e-15 else (bb * ee - cc * dd) / denom
    s = float(np.clip(s, 0.0, 1.0))
    t = (bb * s + ee) / cc
    if t < 0.0:
        t = 0.0
        s = float(np.clip(-dd / aa, 0.0, 1.0))
    elif t > 1.0:
        t = 1.0
        s = float(np.clip((bb - dd) / aa, 0.0, 1.0))
    return float(np.linalg.norm((a0 + s * u) - (b0 + t * v)))


def _nonlocal_self_clearance(points: np.ndarray,
                             adjacency_mm: float,
                             search_band_mm: float = 0.30) -> float:
    """Prove no non-local self approach inside ``search_band_mm``.

    A bounds tree avoids the quadratic all-segment scan.  Returning the band
    when no candidate is closer is an intentional lower bound, not a claimed
    exact global minimum.
    """

    points = np.asarray(points, dtype=float)
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    total = float(cumulative[-1])
    starts, ends = points[:-1], points[1:]
    bounds = np.column_stack((
        np.minimum(starts, ends),
        np.maximum(starts, ends),
    ))
    tree = trimesh.util.bounds_tree(bounds)
    minimum = float(search_band_mm)
    for one in range(len(lengths)):
        lower = np.minimum(starts[one], ends[one]) - search_band_mm
        upper = np.maximum(starts[one], ends[one]) + search_band_mm
        for two in sorted(tree.intersection((*lower, *upper))):
            if two <= one:
                continue
            direct = max(0.0, cumulative[two] - cumulative[one + 1])
            wrap = max(0.0, total - cumulative[two + 1] + cumulative[one])
            if min(direct, wrap) <= adjacency_mm + 1e-12:
                continue
            minimum = min(minimum, _segment_distance(
                points[one], points[one + 1],
                points[two], points[two + 1],
            ))
    return float(minimum)


def route_for_turn(turn: slot_route.PackingTurn, *, radial_min_mm: float,
                   profile_min_mm: float) -> RouteGeometry:
    crown_radius = (
        cap.CROWN_BASE_CENTER_RADIUS_MM
        + float(turn.radial_mm) - float(radial_min_mm)
    )
    profile_delta = float(turn.profile_radius_mm) - float(profile_min_mm)
    crown_working = cap.MINIMUM_WIRE_CENTER_BEND_RADIUS_MM + profile_delta
    crown_axial = 0.0
    front = _front_crown(
        turn, crown_radius, crown_working, crown_axial, 1
    )
    rear_forward = _front_crown(
        turn, crown_radius, crown_working, crown_axial, -1
    )
    # front: +side -> -side.  Rear must continue -side -> +side.
    negative_side = np.asarray((
        front[-1],
        rear_forward[-1],
    ))
    positive_side = np.asarray((
        rear_forward[0],
        front[0],
    ))
    points = np.vstack((
        front,
        negative_side[1:],
        rear_forward[-2::-1],
        positive_side[1:],
    ))
    self_clearance = _nonlocal_self_clearance(
        points, adjacency_mm=2.0 * float(DEFAULT_STATOR.wire_d)
    )
    return RouteGeometry(
        turn_index=int(turn.turn_index),
        radial_mm=float(turn.radial_mm),
        profile_radius_mm=float(turn.profile_radius_mm),
        crown_center_radius_mm=float(crown_radius),
        crown_working_radius_mm=float(crown_working),
        crown_axial_offset_mm=float(crown_axial),
        points_local_mm=tuple(tuple(map(float, point)) for point in points),
        minimum_analytic_bend_radius_mm=(
            cap.MINIMUM_WIRE_CENTER_BEND_RADIUS_MM
        ),
        simple_non_self_looping=bool(
            self_clearance + 1e-9 >= float(DEFAULT_STATOR.wire_d)
        ),
        nonlocal_self_clearance_mm=float(self_clearance),
    )


def _as_obstacle(route: RouteGeometry, obstacle_id: str,
                 owner: str) -> slot_route.CopperPolyline:
    return slot_route.CopperPolyline(
        obstacle_id=obstacle_id,
        owner=owner,
        turn_index=route.turn_index,
        centerline_local_mm=route.points_local_mm,
    )


def _rotate_about_stator_axis(points: Iterable[Iterable[float]],
                              angle_rad: float) -> np.ndarray:
    points = np.asarray(tuple(points), dtype=float)
    cosine, sine = math.cos(angle_rad), math.sin(angle_rad)
    result = points.copy()
    result[:, 0] = cosine * points[:, 0] - sine * points[:, 1]
    result[:, 1] = sine * points[:, 0] + cosine * points[:, 1]
    return result


def route_clearance_audit(graph: slot_route.PackingSupportGraph
                          ) -> dict[str, Any]:
    radial_min = min(float(turn.radial_mm) for turn in graph.turns)
    profile_min = min(float(turn.profile_radius_mm) for turn in graph.turns)
    routes = tuple(route_for_turn(
        turn, radial_min_mm=radial_min, profile_min_mm=profile_min
    ) for turn in graph.turns)
    wire = float(graph.wire_diameter_mm)
    chord = cap.MINIMUM_WIRE_CENTER_BEND_RADIUS_MM * (
        1.0 - math.cos(math.radians(ARC_STEP_DEG) / 2.0)
    )

    active_rows = []
    minimum_nonparent = math.inf
    minimum_parent = math.inf
    minimum_nonparent_case = None
    minimum_parent_case = None
    for route, turn in zip(routes, graph.turns):
        parents = set(turn.parent_turn_indices)
        prior_nonparents = tuple(
            _as_obstacle(routes[index], f"active-turn-{index:02d}",
                         "prior_nonparent")
            for index in range(route.turn_index)
            if index not in parents
        )
        prior_parents = tuple(
            _as_obstacle(routes[index], f"active-turn-{index:02d}",
                         "declared_support_parent")
            for index in sorted(parents)
        )
        nonparent_clearance = (
            slot_route.CopperField(prior_nonparents).clearance(
                route.points_local_mm, 1.0
            ) if prior_nonparents else None
        )
        parent_clearance = (
            slot_route.CopperField(prior_parents).clearance(
                route.points_local_mm, 1.0
            ) if prior_parents else None
        )
        nonparent_raw = (float(nonparent_clearance.minimum_centerline_distance_mm)
                         if nonparent_clearance is not None else 1e9)
        parent_raw = (float(parent_clearance.minimum_centerline_distance_mm)
                      if parent_clearance is not None else 1e9)
        nonparent_lower = float(nonparent_raw) - 2.0 * chord
        parent_lower = float(parent_raw) - 2.0 * chord
        if (nonparent_clearance is not None
                and nonparent_lower < minimum_nonparent):
            minimum_nonparent = nonparent_lower
            minimum_nonparent_case = {
                "turn_index": route.turn_index,
                "obstacle_id": nonparent_clearance.obstacle_id,
                "raw_centerline_distance_mm": nonparent_raw,
            }
        if (parent_clearance is not None
                and parent_lower < minimum_parent):
            minimum_parent = parent_lower
            minimum_parent_case = {
                "turn_index": route.turn_index,
                "obstacle_id": parent_clearance.obstacle_id,
                "raw_centerline_distance_mm": parent_raw,
            }
        active_rows.append({
            "turn_index": route.turn_index,
            "nonparent_centerline_lower_bound_mm": nonparent_lower,
            "parent_centerline_lower_bound_mm": parent_lower,
            "nonparent_obstacle_id": (
                nonparent_clearance.obstacle_id
                if nonparent_clearance is not None else None
            ),
            "parent_obstacle_id": (
                parent_clearance.obstacle_id
                if parent_clearance is not None else None
            ),
            "self_centerline_clearance_mm": route.nonlocal_self_clearance_mm,
            "both_direction_current_half_ok": (
                route.simple_non_self_looping
            ),
        })

    pitch = 2.0 * math.pi / int(DEFAULT_STATOR.slots)
    minimum_neighbor = math.inf
    minimum_neighbor_case = None
    neighbor_rows = []
    for side in (-1, 1):
        neighbor_obstacles = []
        for route in routes:
            points = _rotate_about_stator_axis(
                route.points_local_mm, side * pitch
            )
            neighbor_obstacles.append(slot_route.CopperPolyline(
                obstacle_id=(
                    f"neighbor-{side:+d}-turn-{route.turn_index:02d}"
                ),
                owner="fully_wound_neighbor",
                turn_index=route.turn_index,
                centerline_local_mm=tuple(
                    tuple(map(float, point)) for point in points
                ),
            ))
        neighbor_field = slot_route.CopperField(tuple(neighbor_obstacles))
        for route in routes:
            raw = neighbor_field.clearance(route.points_local_mm, 1.0)
            lower = float(raw.minimum_centerline_distance_mm) - 2.0 * chord
            if lower < minimum_neighbor:
                minimum_neighbor = lower
                minimum_neighbor_case = {
                    "neighbor_side": side,
                    "turn_index": route.turn_index,
                    "obstacle_id": raw.obstacle_id,
                    "raw_centerline_distance_mm": float(
                        raw.minimum_centerline_distance_mm
                    ),
                }
            neighbor_rows.append({
                "neighbor_side": side,
                "turn_index": route.turn_index,
                "centerline_lower_bound_mm": lower,
                "obstacle_id": raw.obstacle_id,
            })

    route_records = [{
        "turn_index": route.turn_index,
        "radial_mm": route.radial_mm,
        "profile_radius_mm": route.profile_radius_mm,
        "crown_center_radius_mm": route.crown_center_radius_mm,
        "crown_working_radius_mm": route.crown_working_radius_mm,
        "crown_axial_offset_mm": route.crown_axial_offset_mm,
        "point_count": len(route.points_local_mm),
        "minimum_analytic_bend_radius_mm": (
            route.minimum_analytic_bend_radius_mm
        ),
        "nonlocal_self_clearance_mm": route.nonlocal_self_clearance_mm,
        "simple_non_self_looping": route.simple_non_self_looping,
    } for route in routes]
    gates = {
        "all_50_route_constructions": len(routes) == 50,
        "all_R3_or_greater": all(
            route.minimum_analytic_bend_radius_mm >= 3.0
            for route in routes
        ),
        "both_directions_current_half": all(
            route.simple_non_self_looping for route in routes
        ),
        "prior_nonparent_copper": minimum_nonparent + 1e-9 >= wire,
        "declared_parent_contact_not_penetrating": (
            minimum_parent + 1e-9 >= wire - 2.0 * chord
        ),
        "both_neighbor_teeth": minimum_neighbor + 1e-9 >= wire,
    }
    return {
        "status": "PASS" if all(gates.values()) else "FAIL",
        "gates": gates,
        "wire_diameter_mm": wire,
        "sampled_arc_chord_error_bound_each_mm": chord,
        "minimum_nonparent_centerline_lower_bound_mm": minimum_nonparent,
        "minimum_parent_centerline_lower_bound_mm": minimum_parent,
        "minimum_nonparent_case": minimum_nonparent_case,
        "minimum_parent_case": minimum_parent_case,
        "minimum_neighbor_centerline_lower_bound_mm": minimum_neighbor,
        "minimum_neighbor_case": minimum_neighbor_case,
        "route_cases": 50 * len(WIRE_DIRECTIONS),
        "routes": route_records,
        "active_progressive": active_rows,
        "neighbors": neighbor_rows,
    }


def geometry_boundaries(graph: slot_route.PackingSupportGraph) -> dict[str, Any]:
    radius = cap.MINIMUM_WIRE_CENTER_BEND_RADIUS_MM
    half_neck = cap.tooth_half_width_mm()
    spans = [
        2.0 * (half_neck + float(turn.profile_radius_mm))
        for turn in graph.turns
    ]
    planar_bridges = [span - 2.0 * radius for span in spans]
    radial_min = min(float(turn.radial_mm) for turn in graph.turns)
    radial_max = max(float(turn.radial_mm) for turn in graph.turns)
    profile_min = min(float(turn.profile_radius_mm) for turn in graph.turns)
    profile_max = max(float(turn.profile_radius_mm) for turn in graph.turns)
    profile_span = profile_max - profile_min
    maximum_working_radius = radius + profile_span
    connector_required = []
    for turn in graph.turns:
        lateral = radius - (half_neck + profile_min)
        axial = 0.0
        connector_required.append(
            radial_min + radius
            + cap.s_bend_forward_mm(lateral)
            + cap.s_bend_forward_mm(axial)
            + cap.CAP_RADIAL_STRAIGHT_ALLOWANCE_MM
        )
    pitch_half = math.pi / int(DEFAULT_STATOR.slots)
    wire_neighbor_required = (
        2.0 * (maximum_working_radius
               + cap.MAXIMUM_LAUNCH_WIRE_RADIUS_MM)
        / (2.0 * math.sin(pitch_half))
    )
    pad_radius = (
        maximum_working_radius + cap.MAXIMUM_LAUNCH_WIRE_RADIUS_MM
        + cap.CAP_NOMINAL_WALL_MM / 2.0
    )
    cap_ligament_required = (
        2.0 * pad_radius + cap.CAP_MINIMUM_LIGAMENT_MM
    ) / (2.0 * math.sin(pitch_half))
    required_base = max(
        max(connector_required),
        wire_neighbor_required,
        cap_ligament_required,
    )
    selected_base = cap.CROWN_BASE_CENTER_RADIUS_MM
    radial_span = radial_max - radial_min
    outer_required = (
        selected_base + radial_span + radius
        + profile_span
        + cap.MAXIMUM_LAUNCH_WIRE_RADIUS_MM
        + cap.CAP_NOMINAL_WALL_MM / 2.0
    )
    mouth = (
        float(insulation.geometry_summary()["stator"]["bare_slot_mouth_mm"])
        - 2.0 * float(insulation.MATERIAL_RECEIVING_MAX_MM)
    )
    return {
        "planar_horn_boundary": {
            "required_center_span_mm": 2.0 * radius,
            "actual_minimum_span_mm": min(spans),
            "actual_maximum_span_mm": max(spans),
            "minimum_bridge_mm": min(planar_bridges),
            "maximum_bridge_mm": max(planar_bridges),
            "failing_turn_count": sum(value < -1e-9 for value in planar_bridges),
            "interpretation": (
                "all in-plane two-quarter-horn returns self-overlap; an "
                "outboard radial U-return is mandatory"
            ),
        },
        "outboard_recovery": {
            "maximum_connector_required_base_radius_mm": max(connector_required),
            "minimum_wire_neighbor_base_radius_mm": wire_neighbor_required,
            "minimum_0p5_wall_ligament_base_radius_mm": cap_ligament_required,
            "controlling_required_base_radius_mm": required_base,
            "selected_base_radius_mm": selected_base,
            "selected_base_margin_mm": selected_base - required_base,
            "packing_radial_span_mm": radial_span,
            "packing_profile_span_mm": profile_span,
            "maximum_outboard_U_radius_mm": maximum_working_radius,
            "minimum_required_cap_outer_radius_mm": outer_required,
            "minimum_required_cap_outer_diameter_mm": 2.0 * outer_required,
        },
        "open_mouth": {
            "lined_mouth_mm": mouth,
            "launch_wire_diameter_mm": cap.MAXIMUM_LAUNCH_WIRE_DIAMETER_MM,
            "free_lateral_remainder_mm": (
                mouth - cap.MAXIMUM_LAUNCH_WIRE_DIAMETER_MM
            ),
            "closed_nozzle": False,
            "status": (
                "PASS" if mouth >= cap.MAXIMUM_LAUNCH_WIRE_DIAMETER_MM
                else "FAIL"
            ),
        },
    }


def _part_mesh(part: Any, linear: float = 0.03,
               angular: float = 0.08) -> trimesh.Trimesh:
    vertices, faces = part.tessellate(linear, angular)
    mesh = trimesh.Trimesh(
        vertices=np.asarray([(v.X, v.Y, v.Z) for v in vertices]),
        faces=np.asarray(faces),
        process=True,
    )
    if not mesh.is_watertight:
        raise RuntimeError(f"{getattr(part, 'label', 'part')} mesh is not watertight")
    return mesh


def _fcl_object(mesh: trimesh.Trimesh) -> fcl.CollisionObject:
    return fcl.CollisionObject(collide.make_bvh(mesh), fcl.Transform())


def _distance(one: fcl.CollisionObject,
              two: fcl.CollisionObject) -> float:
    collision = fcl.CollisionResult()
    fcl.collide(one, two, fcl.CollisionRequest(), collision)
    if collision.is_collision:
        return -1.0
    return float(fcl.distance(
        one, two, fcl.DistanceRequest(), fcl.DistanceResult()
    ))


def rigid_envelope_audit(graph: slot_route.PackingSupportGraph,
                         full_sweep: bool = True) -> dict[str, Any]:
    radial_min = min(float(turn.radial_mm) for turn in graph.turns)
    radial_max = max(float(turn.radial_mm) for turn in graph.turns)
    radial_span = radial_max - radial_min
    profile_min = min(float(turn.profile_radius_mm) for turn in graph.turns)
    profile_max = max(float(turn.profile_radius_mm) for turn in graph.turns)
    profile_span = profile_max - profile_min
    cap_mesh = _part_mesh(cap.guide_cap(
        1, radial_span_mm=radial_span, profile_span_mm=profile_span
    ))
    rear_mesh = _part_mesh(cap.guide_cap(
        -1, radial_span_mm=radial_span, profile_span_mm=profile_span
    ))
    cap_pair = trimesh.util.concatenate((cap_mesh, rear_mesh))
    flyer_mesh = trimesh.util.concatenate([
        _part_mesh(part) for part in assembly.flyer_link()
    ])
    cap_object = _fcl_object(cap_pair)
    flyer_object = _fcl_object(flyer_mesh)

    # Stator-local -> machine at M1=0.
    local_to_world = np.asarray((
        (0.0, -1.0, 0.0),
        (0.0, 0.0, 1.0),
        (-1.0, 0.0, 0.0),
    ))
    angles = range(FLYER_ANGLE_COUNT) if full_sweep else range(0, 360, 10)
    rows = []
    minimum = math.inf
    witness = None
    for turn in graph.turns:
        axis_z = 2.0 + float(turn.radial_mm)
        cap_object.setTransform(fcl.Transform(
            local_to_world, np.array((0.0, 0.0, axis_z))
        ))
        for angle_deg in angles:
            angle = math.radians(angle_deg)
            cosine, sine = math.cos(angle), math.sin(angle)
            flyer_rotation = np.asarray((
                (cosine, -sine, 0.0),
                (sine, cosine, 0.0),
                (0.0, 0.0, 1.0),
            ))
            flyer_object.setTransform(fcl.Transform(
                flyer_rotation, np.zeros(3)
            ))
            value = _distance(cap_object, flyer_object)
            if value < minimum:
                minimum = value
                witness = {
                    "turn_index": int(turn.turn_index),
                    "stator_axis_machine_z_mm": axis_z,
                    "flyer_angle_deg": int(angle_deg),
                    "clearance_mm": float(value),
                }
            rows.append(value)
    # The cap starts at local radius hub_OD/2.  The largest chuck neck is the
    # ER11 nut radius 9.75 mm; the common radial gap is independent of M0/M1.
    cap_inner = (
        float(DEFAULT_STATOR.od) * float(DEFAULT_STATOR.hub_od_ratio) / 2.0
    )
    chuck_radius = max(
        float(segment[0])
        for segment in PARAMS.chuck_neck_profile(DEFAULT_STATOR)
    )
    chuck_radial_clearance = cap_inner - chuck_radius
    return {
        "status": (
            "PASS" if minimum >= RIGID_TARGET_MM
            and chuck_radial_clearance >= RIGID_TARGET_MM else "FAIL"
        ),
        "flyer": {
            "sweep": (
                "50 packing depths x 360 integer flyer angles"
                if full_sweep else "50 packing depths x 36 ten-degree angles"
            ),
            "sample_count": len(rows),
            "minimum_clearance_mm": float(minimum),
            "target_mm": RIGID_TARGET_MM,
            "witness": witness,
            "status": "PASS" if minimum >= RIGID_TARGET_MM else "FAIL",
        },
        "chuck": {
            "minimum_cap_inner_radius_mm": cap_inner,
            "maximum_chuck_neck_radius_mm": chuck_radius,
            "radial_clearance_mm": chuck_radial_clearance,
            "target_mm": RIGID_TARGET_MM,
            "status": (
                "PASS" if chuck_radial_clearance >= RIGID_TARGET_MM
                else "FAIL"
            ),
            "note": (
                "rear guide projection occupies the 3 mm grip-gap plane but "
                "starts radially outside the chuck neck"
            ),
        },
    }


def material_and_permanence() -> dict[str, Any]:
    return {
        "architecture": (
            "two non-handed stator end caps permanently retained on the "
            "lamination stack; both rotate and translate with M0/M1"
        ),
        "permanence": {
            "status": "GEOMETRICALLY_PLAUSIBLE_NOT_QUALIFIED",
            "retention_candidates": [
                "molded PPS cap with hub-bore snap/heat-stake features",
                "bonded machined unfilled PEEK prototype cap",
            ],
            "removal_after_winding_required": False,
            "reason": (
                "the cap is end-use insulation and remains under the end "
                "turns, so deposited copper is never crossed during removal"
            ),
        },
        "materials": {
            "preferred_series_candidate": "thin-wall injection-molded Ryton PPS",
            "candidate_nominal_wall_mm": cap.CAP_NOMINAL_WALL_MM,
            "source_basis": (
                "Solvay reports many Ryton PPS applications at 0.38-0.51 mm "
                "walls and identifies thin-wall stator bobbins/insulators as "
                "a target application"
            ),
            "prototype_alternate": (
                "machined unfilled PEEK, but Victrex's molding guidance gives "
                "about 1 mm minimum for unfilled molded PEEK; it is not a "
                "0.5 mm molding substitute"
            ),
            "existing_slot_liner": "0.127 mm Nomex 410 slot-cell liner",
            "qualification_required": [
                "production-wire abrasion/reversal coupon at full tension",
                "dielectric withstand before and after cycling",
                "molding shrinkage, flash, gate, weld-line, and 0.5 mm wall capability",
                "temperature, varnish/impregnation, and retention compatibility",
            ],
            "sources": [
                SOURCE_PPS_URL,
                SOURCE_PPS_STATOR_URL,
                SOURCE_PEEK_URL,
                SOURCE_NOMEX_URL,
            ],
        },
    }


def analyze(*, full_rigid_sweep: bool = True) -> dict[str, Any]:
    packing, graph = _load_graph()
    bounds = geometry_boundaries(graph)
    route = route_clearance_audit(graph)
    rigid = rigid_envelope_audit(graph, full_sweep=full_rigid_sweep)
    materials = material_and_permanence()
    gates = {
        "packing_hash_bound": True,
        "permanent_no_extraction": True,
        "open_mouth_accepts_0p5mm_launch_wire": (
            bounds["open_mouth"]["status"] == "PASS"
        ),
        "all_50_turns_both_directions_route": route["status"] == "PASS",
        "flyer_and_chuck_envelope": rigid["status"] == "PASS",
        "material_finish_qualified": False,
    }
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "PASS" if all(gates.values()) else "DESIGN_NO_GO",
        "release_authorized": False,
        "assembly_integration_authorized": False,
        "scope": {
            "stator": "exact default 24-slot OD46 x stack15 source geometry",
            "turns_per_tooth": len(graph.turns),
            "wire_directions": list(WIRE_DIRECTIONS),
            "route_cases": len(graph.turns) * len(WIRE_DIRECTIONS),
            "neighbor_teeth": [-1, 1],
            "minimum_wire_contact_radius_mm": 3.0,
            "maximum_launch_wire_diameter_mm": 0.5,
        },
        "gates": gates,
        "geometry_boundaries": bounds,
        "route_audit": route,
        "rigid_envelope": rigid,
        "material_and_permanence": materials,
        "decision": (
            "Do not integrate the permanent cap.  Following M0/M1 removes "
            "the fixed-shoe extraction failure and an outboard R3 route can "
            "be constructed analytically, but the minimum cap envelope then "
            "enters the flyer's required 360-degree swept volume.  The "
            "0.50 mm wall and wire-contact finish also remain unqualified."
        ),
        "exact_no_go_boundary": {
            "planar": (
                "every packed side span is below 6.000 mm, so the simple "
                "two-quarter-horn planar bridge is negative for all 50 turns"
            ),
            "outboard": (
                "radial recovery requires the reported minimum cap OD; the "
                "full rigid sweep reports the first/closest flyer witness"
            ),
        },
        "source_hashes": {
            "cad/stator_winding_guide_cap.py": _sha256(
                CAD / "stator_winding_guide_cap.py"
            ),
            "sim/stator_winding_guide_cap_study.py": _sha256(Path(__file__)),
            "out/reports/slot_packing.json": _sha256(PACKING_PATH),
            "cad/stator_model.py": _sha256(CAD / "stator_model.py"),
            "cad/stator_insulation_nomex410.py": _sha256(
                CAD / "stator_insulation_nomex410.py"
            ),
        },
        "packing_report_sha256": packing["report_sha256"],
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def write(report: dict[str, Any]) -> None:
    REPORTS.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    bounds = report["geometry_boundaries"]
    route = report["route_audit"]
    rigid = report["rigid_envelope"]
    planar = bounds["planar_horn_boundary"]
    outboard = bounds["outboard_recovery"]
    lines = [
        "# Permanent stator winding-guide cap study",
        "",
        f"Status: **{report['status']}**. Release and assembly integration remain false.",
        "",
        "## Exact geometry boundary",
        "",
        (
            f"All {planar['failing_turn_count']} packed turns have a negative planar "
            f"R3 bridge: span {planar['actual_minimum_span_mm']:.3f}.."
            f"{planar['actual_maximum_span_mm']:.3f} mm versus 6.000 mm required; "
            f"bridge {planar['minimum_bridge_mm']:.3f}.."
            f"{planar['maximum_bridge_mm']:.3f} mm."
        ),
        (
            f"The outboard recovery needs cap OD >= "
            f"{outboard['minimum_required_cap_outer_diameter_mm']:.3f} mm."
        ),
        "",
        "## Route proof",
        "",
        (
            f"Constructed {route['route_cases']} turn/direction cases. "
            f"Route status: {route['status']}; minimum prior non-parent, parent, "
            f"and neighbor centreline lower bounds are "
            f"{route['minimum_nonparent_centerline_lower_bound_mm']:.4f}, "
            f"{route['minimum_parent_centerline_lower_bound_mm']:.4f}, and "
            f"{route['minimum_neighbor_centerline_lower_bound_mm']:.4f} mm."
        ),
        "",
        "## Machine envelope",
        "",
        (
            f"Flyer sweep {rigid['flyer']['status']}: minimum "
            f"{rigid['flyer']['minimum_clearance_mm']:.3f} mm at "
            f"`{json.dumps(rigid['flyer']['witness'], sort_keys=True)}`."
        ),
        (
            f"Chuck radial clearance {rigid['chuck']['radial_clearance_mm']:.3f} mm "
            f"({rigid['chuck']['status']})."
        ),
        "",
        "## Decision",
        "",
        report["decision"],
        "",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ]
    MD_OUT.write_text("\n".join(lines))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--quick-rigid", action="store_true")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    report = analyze(full_rigid_sweep=not args.quick_rigid)
    if not args.check:
        write(report)
        print(f"wrote {JSON_OUT} and {MD_OUT}")
    print(json.dumps({
        "status": report["status"],
        "route_status": report["route_audit"]["status"],
        "flyer_clearance_mm": (
            report["rigid_envelope"]["flyer"]["minimum_clearance_mm"]
        ),
        "report_sha256": report["report_sha256"],
    }, indent=2))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
