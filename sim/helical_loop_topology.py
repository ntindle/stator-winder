"""Fail-closed study of a fixed-M0, curvature-bounded winding topology.

This file is deliberately isolated from the production route modules.  It
tests a different topological idea against the *captured* controller
kinematics before that idea is allowed anywhere near the release path.

The important kinematic constraint is easy to miss: the two M2 half-turns
which make one deposited turn carry the same M0 target.  A deposited turn is
therefore confined to one stator-local radial plane.  A radial hairpin would
look attractive in CAD, but would require uncommanded intra-turn M0 motion.

The candidate below instead uses one symmetric L-R-L bounded-curvature cap
in the tangential/axial plane.  The complete 50-wire bundle is made from
parallel normal offsets of that cap.  This retains the exact measured slot
packing metric through the end turn rather than collapsing the tangential
packing coordinate at an apex.  Adjacent teeth use a two-colour axial lane:
even teeth use lane zero and odd teeth use a 12 mm extension.  The 24-tooth
cycle is bipartite, so both neighbours of every tooth occupy the other lane.

Nothing here grants release.  The report remains FAIL unless the exact M0
contract, curvature, self-clearance, prior copper, both neighbours, both M2
signs, core clearance, and flyer envelope all pass.  Even a geometric PASS
is only a candidate schedule until controller/capture integration and the
physical receiving/coupon gates are completed.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np
from scipy.optimize import brentq
from shapely.geometry import LineString


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

from params import DEFAULT_STATOR, PARAMS  # noqa: E402
import slot_wire_routes  # noqa: E402
from crown_route_study import known_physical_lower_bound_mm  # noqa: E402
from crown_routes import (  # noqa: E402
    CrownPolicy,
    CurrentHalfObstacle,
    _angle_error_deg,
    _closed_polyline_phase_subpath,
    _sample_tangent_arc,
    _unit,
    adjacent_self_clearance,
    common_support_direction,
)
from slot_route import (  # noqa: E402
    CopperField,
    CopperPolyline,
    PackingSupportGraph,
    _rounded_loop_yz,
    _tip_path,
    exact_polyline_part_clearance,
    rot_z,
)


SCHEMA = "helical-loop-topology-study/v1"
PACKING_PATH = REPORTS / "slot_packing.json"
PLAN_PATH = REPORTS / "slot_winding_plan.json"
OUTPUT_JSON = REPORTS / "helical_loop_topology.json"
OUTPUT_MD = REPORTS / "helical_loop_topology.md"
_EPS = 1.0e-12


@dataclass(frozen=True)
class HelicalLoopPolicy:
    """Parameters of the isolated fixed-M0 topology candidate."""

    base_bend_radius_mm: float = 4.0
    neighbor_axial_lane_mm: float = 12.0
    obstacle_arc_step_deg: float = 1.0
    moving_route_arc_step_deg: float = 1.0
    moving_route_radius_mm: float = 3.0
    moving_route_bridge_mm: float = 5.0
    adjacent_self_limit_diameters: float = 2.0

    def validate(self, maximum_profile_mm: float) -> None:
        if self.base_bend_radius_mm - maximum_profile_mm < 3.0:
            raise ValueError(
                "base LRL radius does not leave 3 mm on the inward offset")
        if self.neighbor_axial_lane_mm <= 0.0:
            raise ValueError("neighbour axial lane must be positive")
        if not 0.0 < self.obstacle_arc_step_deg <= 2.0:
            raise ValueError("obstacle arc step must be in (0, 2] degrees")
        if not 0.0 < self.moving_route_arc_step_deg <= 2.0:
            raise ValueError("route arc step must be in (0, 2] degrees")
        if self.moving_route_radius_mm < 3.0:
            raise ValueError("moving route radius must be at least 3 mm")
        if self.moving_route_bridge_mm <= 0.0:
            raise ValueError("moving route bridge must be positive")
        if not 0.0 < self.adjacent_self_limit_diameters <= 2.0:
            raise ValueError("adjacent-self exemption cannot exceed 2d")


DEFAULT_POLICY = HelicalLoopPolicy()


@dataclass(frozen=True)
class LoopComponents:
    points_local_mm: np.ndarray
    front_cap_local_mm: np.ndarray
    positive_side_local_mm: np.ndarray
    rear_cap_local_mm: np.ndarray
    negative_side_local_mm: np.ndarray
    axial_lane_mm: float


@dataclass(frozen=True)
class SignAwareRoute:
    points_local_mm: tuple[tuple[float, float, float], ...]
    target_local_mm: tuple[float, float, float]
    terminal_tangent_error_deg: float
    join_tangent_error_deg: float
    tip_exit_tangent_error_deg: float
    sampled_arc_chord_error_bound_mm: float


def _canonical_hash(payload: dict[str, Any]) -> str:
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _half_neck_mm(spec: Any) -> float:
    return max(2.5, float(spec.od) * 0.07) / 2.0


def _advance_arc_yz(
    state: np.ndarray,
    curvature_sign: int,
    angle_rad: float,
    radius_mm: float,
) -> np.ndarray:
    """Integrate one constant-curvature planar arc exactly."""

    y, z, heading = map(float, state)
    sign = int(curvature_sign)
    end_heading = heading + sign * float(angle_rad)
    return np.array((
        y + radius_mm / sign
        * (math.sin(end_heading) - math.sin(heading)),
        z + radius_mm / sign
        * (-math.cos(end_heading) + math.cos(heading)),
        end_heading,
    ))


def symmetric_lrl_angles(spec: Any, policy: HelicalLoopPolicy
                         ) -> tuple[float, float, float]:
    """Return the unique short symmetric L-R-L cap for the tooth neck."""

    half_neck = _half_neck_mm(spec)
    radius = float(policy.base_bend_radius_mm)

    def residual(first: float) -> float:
        state = np.array((-half_neck, 0.0, math.pi / 2.0))
        for sign, angle in (
                (1, first), (-1, 2.0 * first + math.pi), (1, first)):
            state = _advance_arc_yz(state, sign, angle, radius)
        return float(state[0] - half_neck)

    first = float(brentq(residual, 1.0e-10, math.pi / 2.0 - 1.0e-8))
    return first, 2.0 * first + math.pi, first


def _sample_offset_front_cap_yz(
    profile_radius_mm: float,
    axial_lane_mm: float,
    spec: Any,
    policy: HelicalLoopPolicy,
) -> np.ndarray:
    """Sample one exact normal offset of the common L-R-L base cap."""

    radius = float(policy.base_bend_radius_mm)
    half_neck = _half_neck_mm(spec)
    half_stack = float(spec.stack) / 2.0
    state = np.array((
        -half_neck, half_stack + float(axial_lane_mm), math.pi / 2.0,
    ))
    points: list[np.ndarray] = []
    for arc_index, (sign, angle) in enumerate(zip(
            (1, -1, 1), symmetric_lrl_angles(spec, policy))):
        count = max(
            2, math.ceil(math.degrees(angle)
                         / policy.obstacle_arc_step_deg))
        samples = np.linspace(0.0, angle, count + 1)
        if arc_index:
            samples = samples[1:]
        y, z, heading = map(float, state)
        for value in samples:
            theta = heading + sign * float(value)
            base = np.array((
                y + radius / sign
                * (math.sin(theta) - math.sin(heading)),
                z + radius / sign
                * (-math.cos(theta) + math.cos(heading)),
            ))
            # Continuous left normal.  It is -Y at the first side and +Y at
            # the second side, exactly the packing profile convention.
            normal = np.array((-math.sin(theta), math.cos(theta)))
            points.append(base + float(profile_radius_mm) * normal)
        state = _advance_arc_yz(state, sign, angle, radius)
    result = np.asarray(points, dtype=float)
    expected = np.asarray((
        (-half_neck - profile_radius_mm, half_stack + axial_lane_mm),
        (half_neck + profile_radius_mm, half_stack + axial_lane_mm),
    ))
    if (np.linalg.norm(result[0] - expected[0]) > 1e-8
            or np.linalg.norm(result[-1] - expected[1]) > 1e-8):
        raise RuntimeError("normal-offset cap changed a packed endpoint")
    return result


def axial_lane_for_tooth(tooth_index: int,
                         policy: HelicalLoopPolicy = DEFAULT_POLICY) -> float:
    """Two-colour lane; a 24-cycle gives every tooth opposite neighbours."""

    return float((int(tooth_index) & 1) * policy.neighbor_axial_lane_mm)


def loop_components(
    turn: Any,
    spec: Any,
    *,
    tooth_index: int = 0,
    policy: HelicalLoopPolicy = DEFAULT_POLICY,
) -> LoopComponents:
    """Build one closed, phase-anchored, fixed-radial deposited loop."""

    lane = axial_lane_for_tooth(tooth_index, policy)
    half_stack = float(spec.stack) / 2.0
    side = _half_neck_mm(spec) + float(turn.profile_radius_mm)
    front_yz = _sample_offset_front_cap_yz(
        float(turn.profile_radius_mm), lane, spec, policy)
    front = np.column_stack((
        np.full(len(front_yz), float(turn.radial_mm)), front_yz,
    ))
    start = np.array((turn.radial_mm, -side, half_stack))
    cap_start = np.array((turn.radial_mm, -side, half_stack + lane))
    cap_end = np.array((turn.radial_mm, side, half_stack + lane))
    end = np.array((turn.radial_mm, side, half_stack))
    positive_side = np.asarray((
        end, (turn.radial_mm, side, -half_stack),
    ), dtype=float)
    rear = front * np.array((1.0, -1.0, -1.0))
    negative_side = np.asarray((
        rear[-1], start,
    ), dtype=float)

    pieces = [
        np.asarray((start, cap_start)), front,
        np.asarray((cap_end, end)), positive_side,
        rear, negative_side,
    ]
    raw: list[np.ndarray] = []
    for piece in pieces:
        for point in piece:
            if not raw or np.linalg.norm(point - raw[-1]) > 1e-10:
                raw.append(np.asarray(point, dtype=float))
    if np.linalg.norm(raw[-1] - raw[0]) > 1e-8:
        raw.append(raw[0].copy())
    points = np.asarray(raw, dtype=float)
    if np.ptp(points[:, 0]) > 1e-10:
        raise RuntimeError("fixed-M0 candidate changed radial x within a turn")
    return LoopComponents(
        points_local_mm=points,
        front_cap_local_mm=np.vstack((start, cap_start, front[1:])),
        positive_side_local_mm=positive_side,
        rear_cap_local_mm=np.vstack((positive_side[-1], rear[0], rear[1:])),
        negative_side_local_mm=negative_side,
        axial_lane_mm=lane,
    )


def _validate_captured_m0_contract(plan: dict[str, Any]) -> dict[str, Any]:
    """Prove M0 is held for both halves of every deposited turn."""

    rows = plan.get("half_turn_centers")
    if not isinstance(rows, list) or len(rows) != 100:
        raise ValueError("winding plan does not contain 100 half-turn centres")
    failures = []
    pairs = []
    for turn in range(50):
        first, second = rows[2 * turn:2 * turn + 2]
        row = {
            "turn_index": turn,
            "half_turn_indices": [
                int(first["half_turn_index"]), int(second["half_turn_index"]),
            ],
            "placement_indices": [
                int(first["placement_index"]), int(second["placement_index"]),
            ],
            "m0_targets_rad": [
                float(first["m0_target_rad"]),
                float(second["m0_target_rad"]),
            ],
            "radial_targets_mm": [
                float(first["radial_mm"]), float(second["radial_mm"]),
            ],
        }
        row["same_placement"] = (
            row["placement_indices"] == [turn, turn])
        row["same_m0_target"] = math.isclose(
            *row["m0_targets_rad"], rel_tol=0.0, abs_tol=1e-12)
        row["same_radial_target"] = math.isclose(
            *row["radial_targets_mm"], rel_tol=0.0, abs_tol=1e-12)
        if not (row["same_placement"] and row["same_m0_target"]
                and row["same_radial_target"]):
            failures.append(turn)
        pairs.append(row)
    return {
        "status": "PASS" if not failures else "FAIL",
        "checked_turns": 50,
        "failed_turn_indices": failures,
        "radial_motion_within_turn_allowed": False,
        "rejected_topology": (
            "radial hairpin requiring intra-turn M0 modulation"),
        "pairs": pairs,
    }


def _as_obstacle(identifier: str, points: np.ndarray,
                 *, owner: str, turn_index: int | None) -> CopperPolyline:
    return CopperPolyline(
        obstacle_id=identifier,
        owner=owner,
        turn_index=turn_index,
        centerline_local_mm=tuple(tuple(map(float, point)) for point in points),
    )


def _rotate_tooth(points: np.ndarray, tooth_delta: int, slots: int
                  ) -> np.ndarray:
    angle = int(tooth_delta) * 2.0 * math.pi / int(slots)
    cosine, sine = math.cos(angle), math.sin(angle)
    result = np.asarray(points, dtype=float).copy()
    x, y = result[:, 0].copy(), result[:, 1].copy()
    result[:, 0] = cosine * x - sine * y
    result[:, 1] = sine * x + cosine * y
    return result


def _polyline_pair_clearance(
    left: np.ndarray,
    right: np.ndarray,
    search_band_mm: float,
) -> tuple[float, int | None, int | None]:
    field = CopperField((_as_obstacle(
        "right", right, owner="pair", turn_index=None),))
    result = field.clearance(left, search_band_mm)
    return (
        float(result.minimum_centerline_distance_mm),
        result.route_segment_index,
        result.obstacle_segment_index,
    )


def _minimum_same_tooth_clearance(
    loops: list[LoopComponents], search_band_mm: float,
) -> dict[str, Any]:
    minimum = math.inf
    pair = None
    segments = None
    failures = []
    for left_index, left in enumerate(loops):
        for right_index in range(left_index):
            right = loops[right_index]
            distance, left_segment, right_segment = _polyline_pair_clearance(
                left.points_local_mm, right.points_local_mm, search_band_mm)
            if distance < minimum:
                minimum = distance
                pair = [left_index, right_index]
                segments = [left_segment, right_segment]
            if distance < search_band_mm:
                failures.append((left_index, right_index, distance))
    return {
        "minimum_centerline_distance_mm": float(minimum),
        "minimum_pair": pair,
        "minimum_segments": segments,
        # The caller replaces this diagnostic threshold with the wire policy.
        "pairs_inside_search_band": len(failures),
    }


def _minimum_cross_field_clearance(
    active: list[LoopComponents],
    obstacles: tuple[CopperPolyline, ...],
    search_band_mm: float,
) -> dict[str, Any]:
    field = CopperField(obstacles)
    minimum = math.inf
    record = None
    for turn_index, loop in enumerate(active):
        result = field.clearance(loop.points_local_mm, search_band_mm)
        if result.minimum_centerline_distance_mm < minimum:
            minimum = float(result.minimum_centerline_distance_mm)
            record = {
                "active_turn_index": turn_index,
                "active_segment_index": result.route_segment_index,
                "obstacle_id": result.obstacle_id,
                "obstacle_segment_index": result.obstacle_segment_index,
            }
    return {
        "minimum_centerline_distance_mm": minimum,
        "minimum_case": record,
    }


def _parity_radius_sweep(
    graph: PackingSupportGraph,
    spec: Any,
    active: list[LoopComponents],
    policy: HelicalLoopPolicy,
) -> list[dict[str, Any]]:
    """Try immediate-divergence odd-tooth caps without a straight riser.

    This is the other fundamentally simple two-colour construction.  Odd
    teeth get a different LRL radius but leave the stack face immediately;
    therefore no vertical lane segment exists for a neighbour cap to cross.
    A small, deterministic radius sweep records whether that family changes
    the topological result.  Two-degree chords are used only to *find* a
    collision; a sub-wire exact segment distance is a valid fail witness.
    """

    rows = []
    for radius in (4.25, 6.0, 10.0, 12.0, 14.0):
        odd_policy = replace(
            policy,
            base_bend_radius_mm=radius,
            neighbor_axial_lane_mm=policy.neighbor_axial_lane_mm,
            obstacle_arc_step_deg=2.0,
        )
        odd_policy.validate(max(
            float(turn.profile_radius_mm) for turn in graph.turns))
        obstacles = []
        odd_loops = [
            loop_components(
                turn, spec, tooth_index=0, policy=odd_policy)
            for turn in graph.turns
        ]
        for side in (-1, 1):
            for turn_index, loop in enumerate(odd_loops):
                transformed = _rotate_tooth(
                    loop.points_local_mm, side, int(spec.slots))
                obstacles.append(_as_obstacle(
                    f"radius-{radius:.2f}-neighbor-{side:+d}-"
                    f"turn-{turn_index:02d}", transformed,
                    owner="parity_radius_probe", turn_index=turn_index))
        minimum = _minimum_cross_field_clearance(
            active, tuple(obstacles), search_band_mm=0.5)
        maximum_radius = max(float(np.max(np.linalg.norm(
            loop.points_local_mm[:, 1:3], axis=1)))
            for loop in odd_loops)
        rows.append({
            "odd_tooth_base_radius_mm": radius,
            "minimum_neighbor_centerline_mm": minimum[
                "minimum_centerline_distance_mm"],
            "minimum_case": minimum["minimum_case"],
            "maximum_flyer_radius_mm": maximum_radius,
            "inside_flyer_tip_circle": (
                maximum_radius + graph.wire_diameter_mm / 2.0
                <= float(PARAMS.flyer_tip_r) + 1e-9),
            "status": (
                "PASS" if minimum["minimum_centerline_distance_mm"]
                + 1e-9 >= graph.wire_diameter_mm else "FAIL"),
        })
    return rows


def _build_sign_aware_route(
    planner: Any,
    graph: PackingSupportGraph,
    spec: Any,
    turn_index: int,
    half_turn_index: int,
    motion_sign: int,
    policy: HelicalLoopPolicy,
) -> SignAwareRoute:
    """Free-span branch whose terminal tangent follows the actual M2 sign.

    This is not a second fixed nozzle.  Both branches start at the same flyer
    guide/feed and end at the same packed point.  Only the free span bows to
    the side from which the just-deposited half arrives.
    """

    if half_turn_index not in (0, 1) or motion_sign not in (-1, 1):
        raise ValueError("half/sign must be 0|1 and -1|+1")
    turn = graph.turn(turn_index)
    phase = float(half_turn_index) * math.pi
    base_axial_sign = 1.0 if half_turn_index == 0 else -1.0
    axial_sign = base_axial_sign * float(motion_sign)
    target = np.array((
        float(turn.radial_mm),
        *_rounded_loop_yz(turn.profile_radius_mm, phase, spec),
    ))
    support, _ = common_support_direction(
        graph, turn, half_turn_index, spec, CrownPolicy())
    radius = float(policy.moving_route_radius_mm)
    terminal_start = (
        target + radius * support
        + np.array((0.0, 0.0, axial_sign * radius)))
    virtual_corner = (
        terminal_start + policy.moving_route_bridge_mm * support)
    bridge_tangent = -support
    rotation = rot_z(phase)
    feed = rotation @ np.asarray(
        planner.guide["feed_local_mm"], dtype=float)
    axis_z = float(turn.radial_mm) + float(planner.contact["z_mm"])
    incoming = _unit(
        bridge_tangent + np.array((0.0, 0.0, -0.2 * axial_sign)))
    local_tip = None
    tip_meta = None
    approach_start = None
    for _ in range(100):
        angle = math.acos(float(np.clip(
            incoming @ bridge_tangent, -1.0, 1.0)))
        setback = radius * math.tan(angle / 2.0)
        approach_start = virtual_corner - setback * incoming
        target_world = np.array((
            -approach_start[1], approach_start[2],
            axis_z - approach_start[0],
        ))
        tip_path, tip_meta = _tip_path(
            feed, target_world, planner.guide,
            planner.guide_wire_radius_mm, rotation,
            arc_step_deg=policy.moving_route_arc_step_deg)
        local_tip = np.column_stack((
            axis_z - tip_path[:, 2], -tip_path[:, 0], tip_path[:, 1],
        ))
        observed = _unit(approach_start - local_tip[-2])
        updated = _unit(incoming + observed)
        if np.linalg.norm(updated - incoming) <= 1e-12:
            incoming = observed
            break
        incoming = updated
    else:
        raise RuntimeError("sign-aware tip tangent did not converge")

    angle = math.acos(float(np.clip(
        incoming @ bridge_tangent, -1.0, 1.0)))
    setback = radius * math.tan(angle / 2.0)
    approach_start = virtual_corner - setback * incoming
    bridge_start = virtual_corner + setback * bridge_tangent
    bridge_vector = terminal_start - bridge_start
    if (float(bridge_vector @ bridge_tangent) <= 1e-9
            or np.linalg.norm(np.cross(
                bridge_vector, bridge_tangent)) > 1e-8):
        raise RuntimeError("sign-aware bridge is too short")
    target_world = np.array((
        -approach_start[1], approach_start[2],
        axis_z - approach_start[0],
    ))
    tip_path, tip_meta = _tip_path(
        feed, target_world, planner.guide,
        planner.guide_wire_radius_mm, rotation,
        arc_step_deg=policy.moving_route_arc_step_deg)
    local_tip = np.column_stack((
        axis_z - tip_path[:, 2], -tip_path[:, 0], tip_path[:, 1],
    ))
    incoming = _unit(approach_start - local_tip[-2])
    join = _sample_tangent_arc(
        approach_start, incoming, bridge_tangent, radius,
        policy.moving_route_arc_step_deg)
    terminal_tangent = np.array((0.0, 0.0, -axial_sign))
    terminal = _sample_tangent_arc(
        terminal_start, bridge_tangent, terminal_tangent, radius,
        policy.moving_route_arc_step_deg)
    raw = np.vstack((
        local_tip[:-1], join, bridge_start, terminal_start, terminal, target,
    ))
    keep = [0]
    for index in range(1, len(raw)):
        if np.linalg.norm(raw[index] - raw[keep[-1]]) > 1e-9:
            keep.append(index)
    points = raw[keep]
    join_error = max(
        _angle_error_deg(
            approach_start - local_tip[-2], join[1] - join[0]),
        _angle_error_deg(
            join[-1] - join[-2], terminal_start - bridge_start),
        _angle_error_deg(
            terminal_start - bridge_start,
            terminal[1] - terminal[0]),
    )
    terminal_error = _angle_error_deg(
        terminal[-1] - terminal[-2], terminal_tangent)
    chord = radius * (
        1.0 - math.cos(math.radians(
            policy.moving_route_arc_step_deg) / 2.0))
    return SignAwareRoute(
        points_local_mm=tuple(tuple(map(float, point)) for point in points),
        target_local_mm=tuple(map(float, target)),
        terminal_tangent_error_deg=float(terminal_error),
        join_tangent_error_deg=float(join_error),
        tip_exit_tangent_error_deg=float(tip_meta.exit_tangent_error_deg),
        sampled_arc_chord_error_bound_mm=float(chord),
    )


def _current_half(
    loop: LoopComponents,
    graph: PackingSupportGraph,
    spec: Any,
    turn_index: int,
    half_turn_index: int,
    motion_sign: int,
) -> CurrentHalfObstacle:
    turn = graph.turn(turn_index)
    end_phase = float(motion_sign * half_turn_index) * math.pi
    start_phase = end_phase - float(motion_sign) * math.pi
    phase_zero = np.array((
        float(turn.radial_mm),
        -_half_neck_mm(spec) - float(turn.profile_radius_mm),
        float(spec.stack) / 2.0,
    ))
    points = _closed_polyline_phase_subpath(
        loop.points_local_mm, start_phase, end_phase, phase_zero)
    length = float(np.sum(np.linalg.norm(points[1:] - points[:-1], axis=1)))
    return CurrentHalfObstacle(
        turn_index=turn_index,
        physical_half_index=half_turn_index,
        motion_sign=motion_sign,
        start_phase_rad=start_phase,
        end_phase_rad=end_phase,
        points_local_mm=tuple(tuple(map(float, point)) for point in points),
        length_mm=length,
        sha256=hashlib.sha256(
            np.asarray(points, dtype="<f8").tobytes()).hexdigest(),
    )


def _self_simple(loop: LoopComponents) -> bool:
    yz = np.asarray(loop.points_local_mm[:, 1:], dtype=float)
    return bool(LineString(yz).is_simple)


def _arc_chord_error_bound(graph: PackingSupportGraph,
                           policy: HelicalLoopPolicy) -> float:
    maximum_radius = (
        policy.base_bend_radius_mm
        + max(float(turn.profile_radius_mm) for turn in graph.turns))
    return float(maximum_radius * (
        1.0 - math.cos(math.radians(
            policy.obstacle_arc_step_deg) / 2.0)) + 1.0e-9)


def analyze(
    packing_path: Path = PACKING_PATH,
    plan_path: Path = PLAN_PATH,
    *,
    policy: HelicalLoopPolicy = DEFAULT_POLICY,
    progress: bool = False,
) -> dict[str, Any]:
    packing_path, plan_path = Path(packing_path), Path(plan_path)
    packing = json.loads(packing_path.read_text(encoding="utf-8"))
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    spec = slot_wire_routes._validate_packing_contract(packing)
    graph = PackingSupportGraph.from_report(packing, spec=spec)
    maximum_profile = max(float(turn.profile_radius_mm)
                          for turn in graph.turns)
    policy.validate(maximum_profile)
    planner = slot_wire_routes.build_planner(graph, spec)
    m0_contract = _validate_captured_m0_contract(plan)
    known_budget = known_physical_lower_bound_mm()
    obstacle_chord = _arc_chord_error_bound(graph, policy)
    wire = float(graph.wire_diameter_mm)

    active = [loop_components(
        turn, spec, tooth_index=0, policy=policy) for turn in graph.turns]
    neighbours_by_side: dict[int, list[LoopComponents]] = {}
    neighbour_obstacles = []
    for side in (-1, 1):
        raw = [loop_components(
            turn, spec, tooth_index=side, policy=policy)
               for turn in graph.turns]
        neighbours_by_side[side] = raw
        for turn_index, loop in enumerate(raw):
            transformed = _rotate_tooth(
                loop.points_local_mm, side, int(spec.slots))
            neighbour_obstacles.append(_as_obstacle(
                f"neighbor-{side:+d}-turn-{turn_index:02d}", transformed,
                owner="opposite_axial_lane_neighbor",
                turn_index=turn_index))

    same = _minimum_same_tooth_clearance(
        active, search_band_mm=max(0.5, wire + known_budget + 0.1))
    neighbour = _minimum_cross_field_clearance(
        active, tuple(neighbour_obstacles),
        search_band_mm=max(0.5, wire + known_budget + 0.1))
    parity_radius_sweep = _parity_radius_sweep(
        graph, spec, active, policy)

    # Exact OCC core distances and planar self checks for every deposited loop.
    minimum_core = math.inf
    minimum_core_case = None
    self_failures = []
    for tooth in (0, 1):
        for turn in graph.turns:
            loop = loop_components(
                turn, spec, tooth_index=tooth, policy=policy)
            if not _self_simple(loop):
                self_failures.append([tooth, turn.turn_index])
            core = exact_polyline_part_clearance(
                loop.points_local_mm, planner.stator_part) - obstacle_chord
            if core < minimum_core:
                minimum_core = float(core)
                minimum_core_case = [tooth, turn.turn_index]

    base_radius_min = policy.base_bend_radius_mm - maximum_profile
    envelope_points = np.vstack([
        loop.points_local_mm for loop in active
    ] + [
        loop.points_local_mm for loop in neighbours_by_side[1]
    ])
    flyer_radii = np.linalg.norm(envelope_points[:, 1:3], axis=1)
    maximum_flyer_radius = float(np.max(flyer_radii))

    # The route sweep is intentionally exhaustive: 50 turns x two physical
    # halves x both M2 signs.  Earlier active wire and both full neighbours
    # are present in every relevant case.
    sign_rows = []
    minimum_current = math.inf
    minimum_current_case = None
    minimum_route_core = math.inf
    minimum_route_core_case = None
    minimum_prior_nonparent = math.inf
    minimum_prior_case = None
    minimum_neighbor_route = math.inf
    minimum_neighbor_route_case = None
    active_obstacles = [
        _as_obstacle(
            f"active-turn-{index:02d}", loop.points_local_mm,
            owner="earlier_same_tooth", turn_index=index)
        for index, loop in enumerate(active)
    ]
    neighbor_field = CopperField(tuple(neighbour_obstacles))
    for turn in graph.turns:
        prior_all = tuple(active_obstacles[:turn.turn_index])
        parent_ids = {
            f"active-turn-{index:02d}" for index in turn.parent_turn_indices
        }
        prior_nonparent = CopperField(tuple(
            obstacle for obstacle in prior_all
            if obstacle.obstacle_id not in parent_ids))
        parent_field = CopperField(tuple(
            obstacle for obstacle in prior_all
            if obstacle.obstacle_id in parent_ids))
        for half in (0, 1):
            for sign in (-1, 1):
                route = _build_sign_aware_route(
                    planner, graph, spec, turn.turn_index, half, sign, policy)
                points = np.asarray(route.points_local_mm, dtype=float)
                route_core = exact_polyline_part_clearance(
                    points, planner.stator_part)
                route_core_lower = (
                    route_core - route.sampled_arc_chord_error_bound_mm)
                if route_core_lower < minimum_route_core:
                    minimum_route_core = route_core_lower
                    minimum_route_core_case = [turn.turn_index, half, sign]

                current = _current_half(
                    active[turn.turn_index], graph, spec,
                    turn.turn_index, half, sign)
                adjacent = adjacent_self_clearance(
                    points, current, wire,
                    CrownPolicy(
                        adjacent_self_limit_diameters=(
                            policy.adjacent_self_limit_diameters)),
                    search_band_mm=max(0.5, wire + known_budget + 0.1))
                current_lower = (
                    adjacent.minimum_centerline_distance_mm
                    - route.sampled_arc_chord_error_bound_mm
                    - obstacle_chord)
                if current_lower < minimum_current:
                    minimum_current = current_lower
                    minimum_current_case = [turn.turn_index, half, sign]

                nonparent = prior_nonparent.clearance(
                    points, max(0.5, wire + known_budget + 0.1))
                nonparent_lower = (
                    nonparent.minimum_centerline_distance_mm
                    - route.sampled_arc_chord_error_bound_mm
                    - obstacle_chord)
                if nonparent_lower < minimum_prior_nonparent:
                    minimum_prior_nonparent = nonparent_lower
                    minimum_prior_case = [
                        turn.turn_index, half, sign, nonparent.obstacle_id]

                # Parents are allowed to meet only at the packed endpoint.
                parent_prefix = parent_field.clearance(
                    points[:-1], max(0.5, wire + 0.1))
                parent_prefix_lower = (
                    parent_prefix.minimum_centerline_distance_mm
                    - route.sampled_arc_chord_error_bound_mm
                    - obstacle_chord)

                neighbor_hit = neighbor_field.clearance(
                    points, max(0.5, wire + known_budget + 0.1))
                neighbor_lower = (
                    neighbor_hit.minimum_centerline_distance_mm
                    - route.sampled_arc_chord_error_bound_mm
                    - obstacle_chord)
                if neighbor_lower < minimum_neighbor_route:
                    minimum_neighbor_route = neighbor_lower
                    minimum_neighbor_route_case = [
                        turn.turn_index, half, sign,
                        neighbor_hit.obstacle_id]

                endpoint_parent = all(math.isclose(
                    math.hypot(
                        turn.radial_mm - graph.turn(index).radial_mm,
                        turn.profile_radius_mm
                        - graph.turn(index).profile_radius_mm),
                    wire, rel_tol=0.0, abs_tol=1e-8)
                    for index in turn.parent_turn_indices)
                checks = {
                    "endpoint_identity": np.linalg.norm(
                        points[-1] - np.asarray(route.target_local_mm)) <= 1e-8,
                    "core_clearance": (
                        route_core_lower + 1e-9
                        >= graph.center_core_access_mm),
                    "prior_nonparent_clearance": (
                        nonparent_lower > wire + known_budget),
                    "parent_prefix_clearance": (
                        not parent_ids or parent_prefix_lower + 1e-9 >= wire),
                    "endpoint_parent_tangency": endpoint_parent,
                    "neighbor_clearance": (
                        neighbor_lower + 1e-9 >= wire),
                    "current_half_both_sign_branch": (
                        current_lower > wire + known_budget),
                    "analytic_route_radius": (
                        policy.moving_route_radius_mm >= 3.0),
                    "sampled_c1": (
                        route.tip_exit_tangent_error_deg <= 1e-6
                        and route.join_tangent_error_deg
                        <= policy.moving_route_arc_step_deg / 2.0 + 1e-6
                        and route.terminal_tangent_error_deg
                        <= policy.moving_route_arc_step_deg / 2.0 + 1e-6),
                }
                checks = {name: bool(value)
                          for name, value in checks.items()}
                sign_rows.append({
                    "turn_index": turn.turn_index,
                    "half_turn_index": half,
                    "motion_sign": sign,
                    "status": "PASS" if all(checks.values()) else "FAIL",
                    "checks": checks,
                    "route_sha256": hashlib.sha256(
                        np.asarray(points, dtype="<f8").tobytes()).hexdigest(),
                    "current_half_sha256": current.sha256,
                    "current_half_continuous_lower_bound_mm": current_lower,
                    "prior_nonparent_continuous_lower_bound_mm": (
                        nonparent_lower),
                    "parent_prefix_continuous_lower_bound_mm": (
                        None if not parent_ids else parent_prefix_lower),
                    "neighbor_continuous_lower_bound_mm": neighbor_lower,
                    "route_core_continuous_lower_bound_mm": route_core_lower,
                })
        if progress:
            passed = sum(
                row["status"] == "PASS" for row in sign_rows
                if row["turn_index"] == turn.turn_index)
            print(f"turn {turn.turn_index:02d}: {passed}/4", flush=True)

    route_failure_counts = {
        name: sum(not row["checks"][name] for row in sign_rows)
        for name in sign_rows[0]["checks"]
    }
    route_passed = sum(row["status"] == "PASS" for row in sign_rows)
    deposited_checks = {
        "captured_fixed_m0_contract": m0_contract["status"] == "PASS",
        "all_loops_planar_and_simple": not self_failures,
        "analytic_minimum_bend_radius_3mm": base_radius_min >= 3.0,
        "same_tooth_50_loop_clearance": (
            same["minimum_centerline_distance_mm"] + 1e-9 >= wire),
        "both_adjacent_teeth_present": len(neighbour_obstacles) == 100,
        "neighbor_two_colour_lane_clearance": (
            neighbour["minimum_centerline_distance_mm"] + 1e-9 >= wire),
        "exact_occ_core_clearance": (
            minimum_core + 1e-9 >= graph.center_core_access_mm),
        "inside_flyer_tip_circle": (
            maximum_flyer_radius + wire / 2.0
            <= float(PARAMS.flyer_tip_r) + 1e-9),
    }
    deposited_checks = {
        name: bool(value) for name, value in deposited_checks.items()
    }
    release_flags = {
        **deposited_checks,
        "all_200_sign_specific_live_routes": route_passed == 200,
        "controller_schedule_integrated": False,
        "continuous_capture_regenerated": False,
        "physical_error_budget_complete": False,
    }
    status = "PASS" if all(release_flags.values()) else "FAIL"
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": status,
        "candidate_geometry_status": (
            "PASS" if all(deposited_checks.values())
            and route_passed == 200 else "FAIL"),
        "inputs": {
            "study_source_sha256": _sha256(Path(__file__)),
            "packing_path": str(packing_path.relative_to(ROOT)).replace(
                "\\", "/"),
            "packing_file_sha256": _sha256(packing_path),
            "packing_report_sha256": graph.report_sha256,
            "winding_plan_path": str(plan_path.relative_to(ROOT)).replace(
                "\\", "/"),
            "winding_plan_file_sha256": _sha256(plan_path),
            "wire_finished_diameter_mm": wire,
            "required_core_centerline_mm": graph.center_core_access_mm,
        },
        "policy": asdict(policy),
        "kinematic_contract": m0_contract,
        "topology": {
            "name": "fixed-M0 parallel-offset LRL bundle",
            "single_turn_structure": [
                "front normal-offset LRL cap",
                "positive packed axial stack passage",
                "mirrored rear normal-offset LRL cap",
                "negative packed axial stack passage",
            ],
            "m2_phase_anchors": {
                "0_rad": "negative-side front stack end",
                "pi_rad": "positive-side rear stack end",
                "2pi_rad": "closed at negative-side front stack end",
            },
            "base_lrl_arc_angles_rad": list(
                symmetric_lrl_angles(spec, policy)),
            "minimum_offset_bend_radius_mm": base_radius_min,
            "c1_join_rule": (
                "constant-curvature arcs and axial sides share exact tangents"),
        },
        "candidate_schedule": {
            "electrical_job": {
                "slots": int(spec.slots),
                "turns_per_tooth": int(spec.turns),
                "winding_config": str(spec.winding_config),
            },
            "within_tooth_turn_order": list(range(50)),
            "tooth_order": list(range(0, 24, 2)) + list(range(1, 24, 2)),
            "axial_lane_rule": (
                "even tooth: 0 mm; odd tooth: +12 mm at front and -12 mm "
                "at rear; both neighbours always occupy the other lane"),
            "integration_status": "CANDIDATE_ONLY_NOT_IN_PRODUCTION",
        },
        "alternative_topology_checks": {
            "radial_hairpin": {
                "status": "FAIL",
                "reason": (
                    "requires intra-turn radial x motion while captured M0 "
                    "is identical for both halves of all 50 turns"),
            },
            "straight_riser_two_colour_lane": {
                "status": (
                    "PASS" if neighbour[
                        "minimum_centerline_distance_mm"] + 1e-9 >= wire
                    else "FAIL"),
                "minimum_neighbor_centerline_mm": neighbour[
                    "minimum_centerline_distance_mm"],
                "reason": (
                    "continuous cap segments cross the opposite lane riser"),
            },
            "immediate_divergence_parity_radius_sweep": {
                "status": (
                    "PASS" if any(row["status"] == "PASS"
                                  for row in parity_radius_sweep)
                    else "FAIL"),
                "rows": parity_radius_sweep,
            },
        },
        "deposited_loop_audit": {
            "status": (
                "PASS" if all(deposited_checks.values()) else "FAIL"),
            "checks": deposited_checks,
            "self_failure_cases": self_failures,
            "same_tooth": same,
            "neighbors": neighbour,
            "minimum_occ_core_centerline_mm": minimum_core,
            "minimum_occ_core_case": minimum_core_case,
            "maximum_flyer_radius_mm": maximum_flyer_radius,
            "flyer_tip_radius_mm": float(PARAMS.flyer_tip_r),
            "obstacle_chord_error_bound_mm": obstacle_chord,
        },
        "moving_route_audit": {
            "status": "PASS" if route_passed == 200 else "FAIL",
            "passed_cases": route_passed,
            "expected_cases": 200,
            "failure_counts_by_check": route_failure_counts,
            "minimum_current_half_continuous_lower_bound_mm": minimum_current,
            "minimum_current_half_case": minimum_current_case,
            "minimum_route_core_continuous_lower_bound_mm": minimum_route_core,
            "minimum_route_core_case": minimum_route_core_case,
            "minimum_prior_nonparent_continuous_lower_bound_mm": (
                minimum_prior_nonparent),
            "minimum_prior_nonparent_case": minimum_prior_case,
            "minimum_neighbor_route_continuous_lower_bound_mm": (
                minimum_neighbor_route),
            "minimum_neighbor_route_case": minimum_neighbor_route_case,
            "cases": sign_rows,
        },
        "known_physical_error_lower_bound_mm": known_budget,
        "release_flags": release_flags,
        "limitations": [
            "The two-colour tooth order/lane is a report-only candidate and "
            "has not changed controller or capture production sources.",
            "The sign-aware branches are quasi-static free-span solutions "
            "from the same guide, not proof of real wire dynamics.",
            "Unknown TIR, contact-position, receipt-instrument, tension, sag, "
            "and enamel-abrasion terms forbid release authorization.",
            "A physical coupon and receiving inspection remain mandatory.",
        ],
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def render_markdown(report: dict[str, Any]) -> str:
    deposited = report["deposited_loop_audit"]
    moving = report["moving_route_audit"]
    return f"""# Fixed-M0 helical-loop topology study

**Overall release status: {report['status']}**  
**Candidate geometry: {report['candidate_geometry_status']}**

The captured motion holds one M0 target through both halves of every turn.
That rejects radial-hairpin ideas which require hidden intra-turn M0 motion.
The tested candidate is a parallel-offset L-R-L bundle in one radial plane,
with a two-colour 0/12 mm axial lane assigned by tooth parity.

- Minimum analytic wire bend radius: {report['topology']['minimum_offset_bend_radius_mm']:.6f} mm
- Same-tooth minimum centerline distance: {deposited['same_tooth']['minimum_centerline_distance_mm']:.9f} mm
- Neighbor minimum centerline distance: {deposited['neighbors']['minimum_centerline_distance_mm']:.9f} mm
- Minimum deposited-loop OCC core distance: {deposited['minimum_occ_core_centerline_mm']:.9f} mm
- Maximum flyer-plane radius: {deposited['maximum_flyer_radius_mm']:.6f} / {deposited['flyer_tip_radius_mm']:.6f} mm
- Sign-specific moving routes: {moving['passed_cases']} / {moving['expected_cases']}

This remains candidate-only until the controller schedule is integrated, a
fresh full capture/continuous audit passes, and physical receiving/coupon
evidence closes the deliberately unknown error terms.

Report SHA-256: `{report['report_sha256']}`
"""


def write_reports(
    report: dict[str, Any],
    json_path: Path = OUTPUT_JSON,
    markdown_path: Path = OUTPUT_MD,
) -> None:
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8")
    markdown_path.write_text(render_markdown(report), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--packing", type=Path, default=PACKING_PATH)
    parser.add_argument("--plan", type=Path, default=PLAN_PATH)
    parser.add_argument("--json", type=Path, default=OUTPUT_JSON)
    parser.add_argument("--markdown", type=Path, default=OUTPUT_MD)
    parser.add_argument("--progress", action="store_true")
    args = parser.parse_args()
    report = analyze(
        args.packing, args.plan, progress=args.progress)
    write_reports(report, args.json, args.markdown)
    moving = report["moving_route_audit"]
    print(
        f"helical-loop topology {report['status']}; candidate geometry "
        f"{report['candidate_geometry_status']}; sign routes "
        f"{moving['passed_cases']}/{moving['expected_cases']}")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
