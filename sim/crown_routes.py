"""Fail-closed layer-staggered crown routes for packed stator winding.

The two-dimensional slot packing remains authoritative.  This module changes
only the deposited end-turns outside the stack: each packed loop keeps its
exact side coordinates while its rounded crown is shifted axially according
to the packed layer.  A moving span leaves the side axially, turns through an
analytic radius-controlled arc in the common outward support cone, and joins
the flyer-tip torus through a second tangent fillet.

Nothing in this module grants release by construction.  Callers must still
check the sampled candidate against exact OCC core geometry, every prior and
neighbor copper loop, both sign-specific arriving current halves, and the
physical error budget.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import hashlib
import math
from typing import Any, Iterable

import numpy as np
import trimesh
from shapely.geometry import box

from slot_route import (
    CopperField,
    CopperPolyline,
    PackingSupportGraph,
    PackingTurn,
    SlotRoutePlanner,
    _rounded_loop_yz,
    _tip_path,
    rot_z,
)


_EPS = 1.0e-12


@dataclass(frozen=True)
class CrownPolicy:
    """Measured-job crown and route policy in millimetres/degrees."""

    layer_step_mm: float = 0.85
    maximum_layer_index: int = 3
    minimum_bend_radius_mm: float = 3.0
    bridge_length_mm: float = 5.0
    route_arc_step_deg: float = 1.0
    obstacle_arc_step_deg: float = 1.0
    direction_step_deg: float = 0.5
    adjacent_self_limit_diameters: float = 2.0
    geometry_family: str = "layer_tier_box"
    half_twist_base_radius_mm: float = 3.95
    half_twist_profile_scale: float = 3.5
    half_twist_bridge_step_mm: float = 0.025
    half_twist_reference_radial_mm: float | None = None
    half_twist_reference_profile_mm: float | None = None
    radial_axial_radius_mm: float = 10.0
    radial_axial_outward_bias_mm: float = 10.0
    radial_axial_profile_scale: float = 3.5

    def validate(self) -> None:
        if self.layer_step_mm <= 0.0:
            raise ValueError("crown layer step must be positive")
        if self.maximum_layer_index < 0:
            raise ValueError("maximum crown layer index cannot be negative")
        if self.minimum_bend_radius_mm < 3.0:
            raise ValueError("crown route bend radius must be at least 3 mm")
        if self.bridge_length_mm <= 0.0:
            raise ValueError("crown bridge length must be positive")
        if not 0.0 < self.route_arc_step_deg <= 2.0:
            raise ValueError("route arc step must be in (0, 2] degrees")
        if not 0.0 < self.obstacle_arc_step_deg <= 2.0:
            raise ValueError("obstacle arc step must be in (0, 2] degrees")
        if not 0.0 < self.direction_step_deg <= 2.0:
            raise ValueError("direction step must be in (0, 2] degrees")
        if not 0.0 < self.adjacent_self_limit_diameters <= 2.0:
            raise ValueError("adjacent self exclusion cannot exceed 2d")
        if self.geometry_family not in {
                "layer_tier_box", "packing_frame_half_twist",
                "radial_axial_dubins"}:
            raise ValueError("unknown crown geometry family")
        if self.half_twist_base_radius_mm <= 0.0:
            raise ValueError("half-twist base radius must be positive")
        if self.half_twist_profile_scale < 1.0:
            raise ValueError("half-twist profile scale cannot contract")
        if self.half_twist_bridge_step_mm <= 0.0:
            raise ValueError("half-twist bridge step must be positive")
        if self.geometry_family == "packing_frame_half_twist" and (
                self.half_twist_reference_radial_mm is None
                or self.half_twist_reference_profile_mm is None):
            raise ValueError(
                "half-twist family requires packing reference coordinates")
        if self.radial_axial_radius_mm < 3.0:
            raise ValueError("radial/axial Dubins radius must be at least 3 mm")
        if self.radial_axial_outward_bias_mm < 0.0:
            raise ValueError("radial/axial outward bias cannot be negative")
        if self.radial_axial_profile_scale < 1.0:
            raise ValueError("radial/axial profile scale cannot contract")

    def crown_extension_mm(self, turn: PackingTurn) -> float:
        layer = int(turn.layer_index)
        if not 0 <= layer <= self.maximum_layer_index:
            raise ValueError("packing layer is outside crown policy")
        # Later packed layers rise farther outside the stack.  Reversing this
        # order puts an early crown above a later incoming span (the original
        # turn-45/46 failure) even though the 2D packed sides remain valid.
        return float(layer) * self.layer_step_mm

    def canonical_dict(self) -> dict[str, float | int | str | None]:
        return {
            "layer_step_mm": self.layer_step_mm,
            "maximum_layer_index": self.maximum_layer_index,
            "minimum_bend_radius_mm": self.minimum_bend_radius_mm,
            "bridge_length_mm": self.bridge_length_mm,
            "route_arc_step_deg": self.route_arc_step_deg,
            "obstacle_arc_step_deg": self.obstacle_arc_step_deg,
            "direction_step_deg": self.direction_step_deg,
            "adjacent_self_limit_diameters": (
                self.adjacent_self_limit_diameters),
            "geometry_family": self.geometry_family,
            "half_twist_base_radius_mm": self.half_twist_base_radius_mm,
            "half_twist_profile_scale": self.half_twist_profile_scale,
            "half_twist_bridge_step_mm": self.half_twist_bridge_step_mm,
            "half_twist_reference_radial_mm": (
                self.half_twist_reference_radial_mm),
            "half_twist_reference_profile_mm": (
                self.half_twist_reference_profile_mm),
            "radial_axial_radius_mm": self.radial_axial_radius_mm,
            "radial_axial_outward_bias_mm": (
                self.radial_axial_outward_bias_mm),
            "radial_axial_profile_scale": self.radial_axial_profile_scale,
        }


DEFAULT_CROWN_POLICY = CrownPolicy()


def packing_frame_half_twist_policy(
    graph: PackingSupportGraph,
    *,
    base_radius_mm: float = 3.95,
    profile_scale: float = 3.5,
    bridge_step_mm: float = 0.025,
) -> CrownPolicy:
    """Bind the reversible half-twist to this packing coordinate frame."""

    radial = [float(turn.radial_mm) for turn in graph.turns]
    profile = [float(turn.profile_radius_mm) for turn in graph.turns]
    policy = replace(
        DEFAULT_CROWN_POLICY,
        geometry_family="packing_frame_half_twist",
        half_twist_base_radius_mm=float(base_radius_mm),
        half_twist_profile_scale=float(profile_scale),
        half_twist_bridge_step_mm=float(bridge_step_mm),
        half_twist_reference_radial_mm=(min(radial) + max(radial)) / 2.0,
        half_twist_reference_profile_mm=(
            min(profile) + max(profile)) / 2.0,
    )
    policy.validate()
    return policy


def radial_axial_dubins_policy(
    *,
    radius_mm: float = 10.0,
    outward_bias_mm: float = 10.0,
    profile_scale: float = 3.5,
    arc_step_deg: float = 0.5,
) -> CrownPolicy:
    """Radial/axial RLR reversal with monotone tangential crossover."""

    policy = replace(
        DEFAULT_CROWN_POLICY,
        geometry_family="radial_axial_dubins",
        radial_axial_radius_mm=float(radius_mm),
        radial_axial_outward_bias_mm=float(outward_bias_mm),
        radial_axial_profile_scale=float(profile_scale),
        obstacle_arc_step_deg=float(arc_step_deg),
    )
    policy.validate()
    return policy


@dataclass(frozen=True)
class CrownRoute:
    points_local_mm: tuple[tuple[float, float, float], ...]
    target_local_mm: tuple[float, float, float]
    half_turn_index: int
    support_direction_local: tuple[float, float, float]
    support_cone_minimum_dot: float
    minimum_bend_radius_mm: float
    bridge_length_mm: float
    tip_exit_tangent_error_deg: float
    join_tangent_error_deg: float
    terminal_tangent_error_deg: float
    sampled_arc_chord_error_bound_mm: float

    @property
    def sha256(self) -> str:
        array = np.asarray(self.points_local_mm, dtype="<f8")
        return hashlib.sha256(array.tobytes()).hexdigest()


@dataclass(frozen=True)
class CurrentHalfObstacle:
    turn_index: int
    physical_half_index: int
    motion_sign: int
    start_phase_rad: float
    end_phase_rad: float
    points_local_mm: tuple[tuple[float, float, float], ...]
    length_mm: float
    sha256: str


@dataclass(frozen=True)
class AdjacentSelfClearance:
    minimum_centerline_distance_mm: float
    route_segment_index: int | None
    current_segment_index: int | None
    route_fraction: float | None
    current_fraction: float | None
    combined_geodesic_to_endpoint_mm: float | None
    adjacency_limit_mm: float
    candidate_pair_count: int


def _half_neck_mm(spec: Any) -> float:
    return max(2.5, float(spec.od) * 0.07) / 2.0


def _left_normal_yz(tangent_yz: np.ndarray) -> np.ndarray:
    tangent = _unit(np.asarray(tangent_yz, dtype=float))
    return np.array((-tangent[1], tangent[0]), dtype=float)


def _quarter_arc_offset_yz(
    start_yz: np.ndarray,
    start_tangent_yz: np.ndarray,
    end_tangent_yz: np.ndarray,
    base_radius_mm: float,
    profile_radius_mm: float,
    step_deg: float,
) -> np.ndarray:
    """Exact normal-offset quarter arc sampled at a bounded angle."""

    start = np.asarray(start_yz, dtype=float)
    u = _unit(np.asarray(start_tangent_yz, dtype=float))
    v = _unit(np.asarray(end_tangent_yz, dtype=float))
    cross = float(u[0] * v[1] - u[1] * v[0])
    if abs(abs(cross) - 1.0) > 1e-9 or abs(float(u @ v)) > 1e-9:
        raise ValueError("hairpin arc tangents must be perpendicular")
    signed_angle = math.copysign(math.pi / 2.0, cross)
    center = (
        start
        + math.copysign(1.0, signed_angle)
        * float(base_radius_mm) * _left_normal_yz(u)
    )
    radial = start - center
    count = max(2, math.ceil(90.0 / float(step_deg)))
    angles = np.linspace(0.0, signed_angle, count + 1)
    cosine, sine = np.cos(angles), np.sin(angles)
    base = center + np.column_stack((
        cosine * radial[0] - sine * radial[1],
        sine * radial[0] + cosine * radial[1],
    ))
    tangents = np.column_stack((
        cosine * u[0] - sine * u[1],
        sine * u[0] + cosine * u[1],
    ))
    normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    return base + float(profile_radius_mm) * normals


def _twist_bump(u: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """C2 endpoint-flat unit bump and its first two derivatives."""

    value = np.asarray(u, dtype=float)
    bump = 64.0 * value ** 3 * (1.0 - value) ** 3
    first = 64.0 * (
        3.0 * value ** 2 - 12.0 * value ** 3
        + 15.0 * value ** 4 - 6.0 * value ** 5)
    second = 64.0 * (
        6.0 * value - 36.0 * value ** 2
        + 60.0 * value ** 3 - 30.0 * value ** 4)
    return bump, first, second


def _half_twist_bridge_points(
    turn: PackingTurn,
    start_y_mm: float,
    axial_z_mm: float,
    length_mm: float,
    policy: CrownPolicy,
) -> np.ndarray:
    radial_reference = float(policy.half_twist_reference_radial_mm)
    profile_reference = float(policy.half_twist_reference_profile_mm)
    count = max(
        16, math.ceil(float(length_mm) / policy.half_twist_bridge_step_mm))
    u = np.linspace(0.0, 1.0, count + 1)
    bump, _, _ = _twist_bump(u)
    phi = math.pi * bump
    scale = 1.0 + (policy.half_twist_profile_scale - 1.0) * bump
    cosine, sine = np.cos(phi), np.sin(phi)
    radial_delta = float(turn.radial_mm) - radial_reference
    profile_delta = float(turn.profile_radius_mm) - profile_reference
    x = (
        radial_reference
        + radial_delta * cosine
        - profile_delta * scale * sine)
    y = float(start_y_mm) + float(length_mm) * u
    z = (
        float(axial_z_mm) + profile_reference
        + radial_delta * sine
        + profile_delta * scale * cosine)
    return np.column_stack((x, y, z))


def _packing_frame_half_twist_loop_centerline(
    turn: PackingTurn,
    spec: Any,
    policy: CrownPolicy,
) -> np.ndarray:
    """Closed six-arc crown with a reversible bridge-only half twist."""

    half_neck = _half_neck_mm(spec)
    half_stack = float(spec.stack) / 2.0
    radius = float(policy.half_twist_base_radius_mm)
    profile = float(turn.profile_radius_mm)
    if radius - profile < 3.0:
        raise ValueError("half-twist inward offset bend radius is below 3 mm")
    headings = (
        np.array((0.0, 1.0)),
        np.array((-1.0, 0.0)),
        np.array((0.0, 1.0)),
        np.array((1.0, 0.0)),
        np.array((0.0, -1.0)),
        np.array((-1.0, 0.0)),
        np.array((0.0, -1.0)),
    )
    base_start = np.array((-half_neck, 0.0), dtype=float)
    top_parts: list[np.ndarray] = []
    start = base_start
    for arc_index in range(3):
        offset_arc = _quarter_arc_offset_yz(
            start, headings[arc_index], headings[arc_index + 1],
            radius, profile, policy.obstacle_arc_step_deg)
        top_parts.append(offset_arc if not top_parts else offset_arc[1:])
        start = start + radius * (
            headings[arc_index] + headings[arc_index + 1])
    bridge_length = 2.0 * half_neck + 2.0 * radius
    bridge = _half_twist_bridge_points(
        turn, float(start[0]), half_stack + float(start[1]),
        bridge_length, policy)
    top_yz = np.vstack(top_parts)
    top = np.column_stack((
        np.full(len(top_yz), float(turn.radial_mm)),
        top_yz[:, 0], half_stack + top_yz[:, 1],
    ))
    if np.linalg.norm(top[-1] - bridge[0]) > 1e-8:
        raise RuntimeError("half-twist bridge does not meet the entry arcs")
    top_parts_3d = [top, bridge[1:]]
    start = start + np.array((bridge_length, 0.0))
    for arc_index in range(3, 6):
        offset_arc = _quarter_arc_offset_yz(
            start, headings[arc_index], headings[arc_index + 1],
            radius, profile, policy.obstacle_arc_step_deg)
        points = np.column_stack((
            np.full(len(offset_arc), float(turn.radial_mm)),
            offset_arc[:, 0], half_stack + offset_arc[:, 1],
        ))
        top_parts_3d.append(points[1:])
        start = start + radius * (
            headings[arc_index] + headings[arc_index + 1])
    positive = np.vstack(top_parts_3d)
    expected_start = np.array((
        turn.radial_mm, -half_neck - profile, half_stack))
    expected_end = np.array((
        turn.radial_mm, half_neck + profile, half_stack))
    if (np.linalg.norm(positive[0] - expected_start) > 1e-8
            or np.linalg.norm(positive[-1] - expected_end) > 1e-8):
        raise RuntimeError("half-twist crown changed a packed side endpoint")

    negative = positive * np.array((1.0, -1.0, -1.0))
    result = np.vstack((
        positive,
        expected_end + np.array((0.0, 0.0, -2.0 * half_stack)),
        negative[1:],
        expected_start,
    ))
    # Replace the two long chords above with explicit stack-side segments;
    # their endpoints are sufficient because both sides are exactly straight.
    if np.linalg.norm(result[0] - result[-1]) > 1e-8:
        raise RuntimeError("half-twist loop did not close")
    return result


def _smoothstep5(u: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    value = np.asarray(u, dtype=float)
    smooth = 10.0 * value ** 3 - 15.0 * value ** 4 + 6.0 * value ** 5
    first = 30.0 * value ** 2 * (1.0 - value) ** 2
    second = 60.0 * value * (1.0 - value) * (1.0 - 2.0 * value)
    return smooth, first, second


def _radial_axial_base_samples(
    policy: CrownPolicy,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Sample exact RLR base x/z/u and one-sided tangent headings."""

    radius = float(policy.radial_axial_radius_mm)
    total_angle = 7.0 * math.pi / 3.0
    x = z = 0.0
    heading = math.pi / 2.0
    cumulative_angle = 0.0
    parts = []
    for sign, angle in (
            (1.0, math.pi / 3.0),
            (-1.0, 5.0 * math.pi / 3.0),
            (1.0, math.pi / 3.0)):
        count = max(
            2, math.ceil(math.degrees(angle)
                         / policy.obstacle_arc_step_deg))
        local = np.linspace(0.0, angle, count + 1)
        theta = heading + sign * local
        radial = (
            x + radius / sign * (np.sin(theta) - math.sin(heading)))
        axial = (
            z + radius / sign * (-np.cos(theta) + math.cos(heading)))
        u = (cumulative_angle + local) / total_angle
        part = np.column_stack((radial, axial, u, theta))
        parts.append(part if not parts else part[1:])
        x, z = float(radial[-1]), float(axial[-1])
        heading += sign * angle
        cumulative_angle += angle
    result = np.vstack(parts)
    u = result[:, 2]
    result[:, 0] += (
        policy.radial_axial_outward_bias_mm * np.sin(math.pi * u) ** 2)
    return result[:, 0], result[:, 1], u, result[:, 3]


def _radial_axial_dubins_loop_centerline(
    turn: PackingTurn,
    spec: Any,
    policy: CrownPolicy,
) -> np.ndarray:
    half_neck = _half_neck_mm(spec)
    half_stack = float(spec.stack) / 2.0
    base_x, base_z, u, _ = _radial_axial_base_samples(policy)
    smooth, _, _ = _smoothstep5(u)
    frame_angle = math.pi * smooth
    sine, cosine = np.sin(frame_angle), np.cos(frame_angle)
    scale = (
        1.0 + (policy.radial_axial_profile_scale - 1.0) * sine ** 2)
    profile_y = -cosine
    profile_z = -scale * sine
    profile = float(turn.profile_radius_mm)
    positive = np.column_stack((
        float(turn.radial_mm) + base_x,
        half_neck * (2.0 * smooth - 1.0) + profile * profile_y,
        half_stack + base_z + profile * profile_z,
    ))
    expected_start = np.array((
        turn.radial_mm, -half_neck - profile, half_stack))
    expected_end = np.array((
        turn.radial_mm, half_neck + profile, half_stack))
    if (np.linalg.norm(positive[0] - expected_start) > 1e-8
            or np.linalg.norm(positive[-1] - expected_end) > 1e-8):
        raise RuntimeError("radial/axial crown changed a packed endpoint")
    negative = positive * np.array((1.0, -1.0, -1.0))
    result = np.vstack((
        positive,
        expected_end + np.array((0.0, 0.0, -2.0 * half_stack)),
        negative[1:],
        expected_start,
    ))
    if np.linalg.norm(result[0] - result[-1]) > 1e-8:
        raise RuntimeError("radial/axial crown did not close")
    return result


def radial_axial_curvature_study(
    graph: PackingSupportGraph,
    spec: Any,
    policy: CrownPolicy,
    *,
    samples_per_arc_degree: int = 40,
) -> dict[str, Any]:
    """Analytic-derivative dense bracket for the RLR crown curvature."""

    policy.validate()
    if policy.geometry_family != "radial_axial_dubins":
        raise ValueError("curvature study requires radial/axial family")
    radius = float(policy.radial_axial_radius_mm)
    bias = float(policy.radial_axial_outward_bias_mm)
    scale_max = float(policy.radial_axial_profile_scale)
    total_angle = 7.0 * math.pi / 3.0
    total_length = radius * total_angle
    boundaries = np.array((0.0, 1.0 / 7.0, 6.0 / 7.0, 1.0))
    signs = (1.0, -1.0, 1.0)
    heading_starts = (
        math.pi / 2.0,
        math.pi / 2.0 + math.pi / 3.0,
        math.pi / 2.0 + math.pi / 3.0 - 5.0 * math.pi / 3.0,
    )
    minimum = math.inf
    minimum_turn = None
    minimum_u = None
    rows = []
    for turn in graph.turns:
        turn_minimum = math.inf
        turn_u = None
        profile = float(turn.profile_radius_mm)
        for segment, sign in enumerate(signs):
            count = max(
                1000,
                round((boundaries[segment + 1] - boundaries[segment])
                      * 420.0 * samples_per_arc_degree))
            u = np.linspace(
                boundaries[segment], boundaries[segment + 1], count + 1)
            local_angle = total_angle * (u - boundaries[segment])
            theta = heading_starts[segment] + sign * local_angle
            smooth, smooth_first, smooth_second = _smoothstep5(u)
            frame = math.pi * smooth
            frame_first = math.pi * smooth_first
            frame_second = math.pi * smooth_second
            sine, cosine = np.sin(frame), np.cos(frame)
            bias_first = bias * math.pi * np.sin(2.0 * math.pi * u)
            bias_second = (
                2.0 * bias * math.pi ** 2 * np.cos(2.0 * math.pi * u))
            x_first = total_length * np.cos(theta) + bias_first
            z_first = total_length * np.sin(theta)
            x_second = (
                -sign * total_length * total_angle * np.sin(theta)
                + bias_second)
            z_second = (
                sign * total_length * total_angle * np.cos(theta))
            profile_y_first = sine * frame_first
            profile_y_second = (
                cosine * frame_first ** 2 + sine * frame_second)
            multiplier = scale_max - 1.0
            f_first = -cosine * (1.0 + 3.0 * multiplier * sine ** 2)
            f_second = sine * (
                1.0 + 3.0 * multiplier * (3.0 * sine ** 2 - 2.0))
            profile_z_first = f_first * frame_first
            profile_z_second = (
                f_second * frame_first ** 2 + f_first * frame_second)
            y_first = (
                2.0 * _half_neck_mm(spec) * smooth_first
                + profile * profile_y_first)
            y_second = (
                2.0 * _half_neck_mm(spec) * smooth_second
                + profile * profile_y_second)
            first = np.column_stack((
                x_first, y_first, z_first + profile * profile_z_first))
            second = np.column_stack((
                x_second, y_second,
                z_second + profile * profile_z_second))
            speed = np.linalg.norm(first, axis=1)
            curvature = (
                np.linalg.norm(np.cross(first, second), axis=1)
                / speed ** 3)
            bend_radius = np.divide(
                1.0, curvature,
                out=np.full_like(curvature, math.inf),
                where=curvature > _EPS)
            index = int(np.argmin(bend_radius))
            if bend_radius[index] < turn_minimum:
                turn_minimum = float(bend_radius[index])
                turn_u = float(u[index])
        rows.append({
            "turn_index": int(turn.turn_index),
            "sampled_minimum_bend_radius_mm": turn_minimum,
            "minimum_u": turn_u,
        })
        if turn_minimum < minimum:
            minimum = turn_minimum
            minimum_turn = int(turn.turn_index)
            minimum_u = turn_u
    # Dense analytic-derivative evaluation is evidence, but without an
    # interval remainder it is intentionally not labelled a formal proof.
    return {
        "status": "NOT_PROVEN" if minimum >= 3.0 else "FAIL",
        "study_kind": "analytic derivatives with dense piecewise bracket",
        "sampled_minimum_bend_radius_mm": minimum,
        "minimum_turn_index": minimum_turn,
        "minimum_u": minimum_u,
        "required_bend_radius_mm": 3.0,
        "interval_remainder_proven": False,
        "turns": rows,
    }


def half_twist_curvature_proof(
    graph: PackingSupportGraph,
    spec: Any,
    policy: CrownPolicy,
) -> dict[str, Any]:
    """Analytic lower bound for every arc and twisted bridge bend radius.

    The bridge has C'=(w'_x,L,w'_z), so |C'|>=L.  Therefore
    kappa<=|w''|/L^2.  The reported |w''| bound follows directly from the
    rotating-frame second derivative and triangle inequalities, using the
    exact global derivative maxima of g=64*u^3*(1-u)^3:
    max|g'|=g'((5-sqrt(5))/10), max|g''|=24.
    """

    policy.validate()
    if policy.geometry_family != "packing_frame_half_twist":
        raise ValueError("curvature proof requires the half-twist family")
    radius = float(policy.half_twist_base_radius_mm)
    maximum_profile = max(
        float(turn.profile_radius_mm) for turn in graph.turns)
    exact_arc_lower = radius - maximum_profile
    half_neck = _half_neck_mm(spec)
    bridge_length = 2.0 * half_neck + 2.0 * radius
    scale = float(policy.half_twist_profile_scale)
    u_critical = (5.0 - math.sqrt(5.0)) / 10.0
    bump_prime_max = (
        192.0 * u_critical ** 2 * (1.0 - u_critical) ** 2
        * (1.0 - 2.0 * u_critical))
    bump_second_max = 24.0
    phi_prime_max = math.pi * bump_prime_max
    phi_second_max = math.pi * bump_second_max
    radial_reference = float(policy.half_twist_reference_radial_mm)
    profile_reference = float(policy.half_twist_reference_profile_mm)
    rows = []
    minimum_bridge_lower = math.inf
    minimum_bridge_turn = None
    for turn in graph.turns:
        radial_delta = abs(float(turn.radial_mm) - radial_reference)
        profile_delta = abs(
            float(turn.profile_radius_mm) - profile_reference)
        value_max = math.hypot(
            radial_delta, profile_delta * scale)
        value_prime_max = (
            profile_delta * (scale - 1.0) * bump_prime_max)
        value_second_max = (
            profile_delta * (scale - 1.0) * bump_second_max)
        second_derivative_upper = (
            (phi_prime_max ** 2 + phi_second_max) * value_max
            + 2.0 * phi_prime_max * value_prime_max
            + value_second_max)
        bridge_radius_lower = (
            math.inf if second_derivative_upper <= _EPS
            else bridge_length ** 2 / second_derivative_upper)
        if bridge_radius_lower < minimum_bridge_lower:
            minimum_bridge_lower = bridge_radius_lower
            minimum_bridge_turn = int(turn.turn_index)
        rows.append({
            "turn_index": int(turn.turn_index),
            "radial_delta_abs_mm": radial_delta,
            "profile_delta_abs_mm": profile_delta,
            "bridge_second_derivative_upper_mm_per_u2": (
                second_derivative_upper),
            "bridge_bend_radius_lower_bound_mm": bridge_radius_lower,
        })
    overall = min(exact_arc_lower, minimum_bridge_lower)
    return {
        "status": "PASS" if overall + 1e-12 >= 3.0 else "FAIL",
        "proof_kind": (
            "analytic rotating-frame derivative norm inequality"),
        "base_radius_mm": radius,
        "maximum_profile_offset_mm": maximum_profile,
        "exact_arc_radius_lower_bound_mm": exact_arc_lower,
        "bridge_length_mm": bridge_length,
        "bump_prime_global_max": bump_prime_max,
        "bump_second_derivative_global_max": bump_second_max,
        "minimum_bridge_radius_lower_bound_mm": minimum_bridge_lower,
        "minimum_bridge_turn_index": minimum_bridge_turn,
        "overall_bend_radius_lower_bound_mm": overall,
        "required_bend_radius_mm": 3.0,
        "turns": rows,
    }


def half_twist_bridge_midpoint_local_z_mm(
    turn: PackingTurn,
    spec: Any,
    policy: CrownPolicy,
) -> float:
    """Exact positive-crown bridge height at phi=pi, g=1."""

    policy.validate()
    if policy.geometry_family != "packing_frame_half_twist":
        raise ValueError("midpoint height requires the half-twist family")
    half_stack = float(spec.stack) / 2.0
    radius = float(policy.half_twist_base_radius_mm)
    reference = float(policy.half_twist_reference_profile_mm)
    delta = float(turn.profile_radius_mm) - reference
    return float(
        half_stack + 3.0 * radius + reference
        - policy.half_twist_profile_scale * delta)


def crowned_loop_centerline(
    turn: PackingTurn,
    spec: Any,
    policy: CrownPolicy = DEFAULT_CROWN_POLICY,
) -> np.ndarray:
    """Closed crowned loop with unchanged packed side coordinates."""

    policy.validate()
    if policy.geometry_family == "packing_frame_half_twist":
        return _packing_frame_half_twist_loop_centerline(
            turn, spec, policy)
    if policy.geometry_family == "radial_axial_dubins":
        return _radial_axial_dubins_loop_centerline(
            turn, spec, policy)
    half_neck = _half_neck_mm(spec)
    half_stack = float(spec.stack) / 2.0
    extension = policy.crown_extension_mm(turn)
    resolution = max(
        2, math.ceil(90.0 / float(policy.obstacle_arc_step_deg)))
    profile = box(
        -half_neck, -half_stack - extension,
        half_neck, half_stack + extension,
    ).buffer(float(turn.profile_radius_mm), quad_segs=resolution)
    yz = np.asarray(profile.exterior.coords, dtype=float)
    result = np.column_stack((
        np.full(len(yz), float(turn.radial_mm)), yz,
    ))
    if len(result) < 8 or not np.all(np.isfinite(result)):
        raise RuntimeError("crowned loop construction is invalid")
    return result


def crowned_loop_point(
    turn: PackingTurn,
    phase_rad: float,
    spec: Any,
    policy: CrownPolicy = DEFAULT_CROWN_POLICY,
) -> np.ndarray:
    """Arc-length point on a crowned loop, phase-zero at the packed side."""

    points = crowned_loop_centerline(turn, spec, policy)
    phase_zero = np.array((
        float(turn.radial_mm),
        -_half_neck_mm(spec) - float(turn.profile_radius_mm),
        float(spec.stack) / 2.0,
    ))
    return _closed_polyline_phase_points(
        points, np.asarray((float(phase_rad),)), phase_zero)[0]


def _polyline_projection_distance_3d(
    points: np.ndarray,
    target: np.ndarray,
) -> tuple[float, float]:
    """Arc distance and Euclidean residual for a 3D polyline projection."""

    vectors = points[1:] - points[:-1]
    lengths = np.linalg.norm(vectors, axis=1)
    squared = np.einsum("ij,ij->i", vectors, vectors)
    fractions = np.divide(
        np.einsum("ij,ij->i", target - points[:-1], vectors),
        squared,
        out=np.zeros(len(vectors), dtype=float),
        where=squared > _EPS,
    )
    fractions = np.clip(fractions, 0.0, 1.0)
    projections = points[:-1] + fractions[:, None] * vectors
    residuals = np.linalg.norm(projections - target, axis=1)
    index = int(np.argmin(residuals))
    cumulative = np.concatenate(((0.0,), np.cumsum(lengths)))
    return (
        float(cumulative[index] + fractions[index] * lengths[index]),
        float(residuals[index]),
    )


def _closed_polyline_phase_points(
    points: np.ndarray,
    phases_rad: np.ndarray,
    phase_zero_point: np.ndarray,
) -> np.ndarray:
    """Vectorized full-3D arc-length interpolation of a closed centerline."""

    loop = np.asarray(points, dtype=float)
    phases = np.asarray(phases_rad, dtype=float)
    vectors = loop[1:] - loop[:-1]
    lengths = np.linalg.norm(vectors, axis=1)
    cumulative = np.concatenate(((0.0,), np.cumsum(lengths)))
    total = float(cumulative[-1])
    if total <= _EPS:
        raise ValueError("closed crown centerline has zero length")
    origin, residual = _polyline_projection_distance_3d(
        loop, np.asarray(phase_zero_point, dtype=float))
    if residual > 1e-7:
        raise RuntimeError("phase-zero packed side is absent from crown")
    distances = (
        origin
        + np.mod(phases, 2.0 * math.pi)
        / (2.0 * math.pi) * total
    ) % total
    indices = np.searchsorted(cumulative, distances, side="right") - 1
    indices = np.clip(indices, 0, len(lengths) - 1)
    local = distances - cumulative[indices]
    fractions = np.divide(
        local,
        lengths[indices],
        out=np.zeros_like(local),
        where=lengths[indices] > _EPS,
    )
    return loop[indices] + fractions[:, None] * vectors[indices]


def _closed_polyline_phase_subpath(
    points: np.ndarray,
    start_phase_rad: float,
    end_phase_rad: float,
    phase_zero_point: np.ndarray,
) -> np.ndarray:
    """Half-loop subpath preserving every source-polyline vertex."""

    loop = np.asarray(points, dtype=float)
    vectors = loop[1:] - loop[:-1]
    lengths = np.linalg.norm(vectors, axis=1)
    cumulative = np.concatenate(((0.0,), np.cumsum(lengths)))
    total = float(cumulative[-1])
    origin, residual = _polyline_projection_distance_3d(
        loop, np.asarray(phase_zero_point, dtype=float))
    if residual > 1e-7:
        raise RuntimeError("phase-zero packed side is absent from crown")
    direction = math.copysign(1.0, end_phase_rad - start_phase_rad)
    start_fraction = (
        (float(start_phase_rad) % (2.0 * math.pi))
        / (2.0 * math.pi))
    start_distance = origin + start_fraction * total
    span = abs(float(end_phase_rad - start_phase_rad)) / (2.0 * math.pi) * total
    end_distance = start_distance + direction * span
    lower, upper = sorted((start_distance, end_distance))
    candidates: list[tuple[float, np.ndarray]] = []
    for wrap in range(-2, 4):
        offset = wrap * total
        for vertex_index, distance in enumerate(cumulative[:-1]):
            unwrapped = float(distance + offset)
            if lower + 1e-10 < unwrapped < upper - 1e-10:
                candidates.append((unwrapped, loop[vertex_index]))
    candidates.sort(key=lambda item: item[0], reverse=direction < 0.0)
    endpoints = _closed_polyline_phase_points(
        loop,
        np.asarray((start_phase_rad, end_phase_rad)),
        phase_zero_point,
    )
    result = np.vstack((
        endpoints[0],
        *[point for _, point in candidates],
        endpoints[1],
    ))
    keep = [0]
    for index in range(1, len(result)):
        if np.linalg.norm(result[index] - result[keep[-1]]) > 1e-10:
            keep.append(index)
    return result[keep]


def crowned_active_copper_before(
    graph: PackingSupportGraph,
    turn_index: int,
    spec: Any,
    policy: CrownPolicy = DEFAULT_CROWN_POLICY,
) -> tuple[CopperPolyline, ...]:
    graph.turn(turn_index)
    return tuple(
        CopperPolyline(
            obstacle_id=f"active-turn-{turn.turn_index:02d}",
            owner="earlier_same_coil_wire_crowned",
            turn_index=turn.turn_index,
            centerline_local_mm=tuple(
                tuple(map(float, point))
                for point in crowned_loop_centerline(turn, spec, policy)),
        )
        for turn in graph.turns[:int(turn_index)]
    )


def crowned_neighbor_prefill_copper(
    graph: PackingSupportGraph,
    spec: Any,
    neighbor_side: int,
    policy: CrownPolicy = DEFAULT_CROWN_POLICY,
) -> tuple[CopperPolyline, ...]:
    if neighbor_side not in (-1, 1):
        raise ValueError("neighbor side must be -1 or +1")
    angle = neighbor_side * 2.0 * math.pi / int(spec.slots)
    cosine, sine = math.cos(angle), math.sin(angle)
    result = []
    for turn in graph.turns:
        local = crowned_loop_centerline(turn, spec, policy)
        transformed = local.copy()
        transformed[:, 0] = cosine * local[:, 0] - sine * local[:, 1]
        transformed[:, 1] = sine * local[:, 0] + cosine * local[:, 1]
        result.append(CopperPolyline(
            obstacle_id=(
                f"neighbor-{neighbor_side:+d}-turn-{turn.turn_index:02d}"),
            owner="neighbor_side_prefill_crowned",
            turn_index=turn.turn_index,
            centerline_local_mm=tuple(
                tuple(map(float, point)) for point in transformed),
        ))
    return tuple(result)


def crown_policy_geometry_sha256(
    graph: PackingSupportGraph,
    spec: Any,
    policy: CrownPolicy = DEFAULT_CROWN_POLICY,
) -> str:
    digest = hashlib.sha256()
    for turn in graph.turns:
        digest.update(np.asarray(
            crowned_loop_centerline(turn, spec, policy),
            dtype="<f8").tobytes())
    return digest.hexdigest()


def _unit(vector: np.ndarray) -> np.ndarray:
    value = np.asarray(vector, dtype=float)
    length = float(np.linalg.norm(value))
    if length <= _EPS:
        raise ValueError("cannot normalize a zero vector")
    return value / length


def common_support_direction(
    graph: PackingSupportGraph,
    turn: PackingTurn,
    half_turn_index: int,
    spec: Any,
    policy: CrownPolicy = DEFAULT_CROWN_POLICY,
) -> tuple[np.ndarray, float]:
    """Planar center direction of the declared-parent outward cone."""

    if half_turn_index not in (0, 1):
        raise ValueError("half turn must be 0 or 1")
    if not turn.parent_turn_indices:
        return np.array((1.0, 0.0, 0.0)), 1.0
    phase = float(half_turn_index) * math.pi
    target = np.array((
        float(turn.radial_mm),
        *_rounded_loop_yz(turn.profile_radius_mm, phase, spec),
    ))
    normals = []
    for parent_index in turn.parent_turn_indices:
        parent = graph.turn(parent_index)
        parent_target = np.array((
            float(parent.radial_mm),
            *_rounded_loop_yz(parent.profile_radius_mm, phase, spec),
        ))
        normal = _unit(target - parent_target)
        if abs(float(normal[2])) > 1e-9:
            raise RuntimeError("packed parent normal is not in the side plane")
        normals.append(normal)
    count = round(360.0 / policy.direction_step_deg)
    candidates = np.array([
        (math.cos(2.0 * math.pi * index / count),
         math.sin(2.0 * math.pi * index / count), 0.0)
        for index in range(count)
    ])
    scores = np.min(candidates @ np.asarray(normals).T, axis=1)
    best = int(np.argmax(scores))
    score = float(scores[best])
    if score < -1e-10:
        raise RuntimeError("declared parents have no planar outward cone")
    return candidates[best], score


def _sample_tangent_arc(
    start: np.ndarray,
    start_tangent: np.ndarray,
    end_tangent: np.ndarray,
    radius_mm: float,
    step_deg: float,
) -> np.ndarray:
    u = _unit(start_tangent)
    v = _unit(end_tangent)
    cosine = float(np.clip(u @ v, -1.0, 1.0))
    angle = math.acos(cosine)
    if angle <= 1e-10:
        return np.asarray((start,), dtype=float)
    e2 = _unit(v - cosine * u)
    count = max(2, math.ceil(math.degrees(angle) / float(step_deg)))
    values = np.linspace(0.0, angle, count + 1)
    return np.asarray(start, dtype=float) + float(radius_mm) * (
        np.sin(values)[:, None] * u
        + (1.0 - np.cos(values))[:, None] * e2)


def _angle_error_deg(left: np.ndarray, right: np.ndarray) -> float:
    cosine = float(np.clip(_unit(left) @ _unit(right), -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def build_c1_crown_route(
    planner: SlotRoutePlanner,
    graph: PackingSupportGraph,
    spec: Any,
    turn_index: int,
    half_turn_index: int,
    policy: CrownPolicy = DEFAULT_CROWN_POLICY,
) -> CrownRoute:
    """Construct one analytic torus/fillet/bridge/terminal-arc route."""

    policy.validate()
    if half_turn_index not in (0, 1):
        raise ValueError("half turn must be 0 or 1")
    turn = graph.turn(turn_index)
    phase = float(half_turn_index) * math.pi
    axial_sign = 1.0 if half_turn_index == 0 else -1.0
    target = np.array((
        float(turn.radial_mm),
        *_rounded_loop_yz(turn.profile_radius_mm, phase, spec),
    ))
    crown_target = crowned_loop_point(turn, phase, spec, policy)
    if np.linalg.norm(crown_target - target) > 1e-7:
        raise RuntimeError("crown changed the authoritative packed side point")

    support_direction, cone_score = common_support_direction(
        graph, turn, half_turn_index, spec, policy)
    radius = float(policy.minimum_bend_radius_mm)
    terminal_arc_start = (
        target + radius * support_direction
        + np.array((0.0, 0.0, axial_sign * radius)))
    virtual_corner = (
        terminal_arc_start
        + float(policy.bridge_length_mm) * support_direction)
    bridge_tangent = -support_direction

    rotation = rot_z(phase)
    feed = rotation @ np.asarray(planner.guide["feed_local_mm"], dtype=float)
    axis_z = float(turn.radial_mm) + float(planner.contact["z_mm"])
    incoming = _unit(
        bridge_tangent + np.array((0.0, 0.0, -0.2 * axial_sign)))
    local_tip = None
    tip_meta = None
    approach_start = None
    for _ in range(100):
        angle = math.acos(float(np.clip(incoming @ bridge_tangent, -1, 1)))
        setback = radius * math.tan(angle / 2.0)
        approach_start = virtual_corner - setback * incoming
        target_world = np.array((
            -approach_start[1], approach_start[2],
            axis_z - approach_start[0],
        ))
        tip_path, tip_meta = _tip_path(
            feed, target_world, planner.guide,
            planner.guide_wire_radius_mm, rotation,
            arc_step_deg=policy.route_arc_step_deg)
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
        raise RuntimeError("tip/fillet tangent fixed point did not converge")

    angle = math.acos(float(np.clip(incoming @ bridge_tangent, -1, 1)))
    setback = radius * math.tan(angle / 2.0)
    approach_start = virtual_corner - setback * incoming
    bridge_start = virtual_corner + setback * bridge_tangent
    bridge_vector = terminal_arc_start - bridge_start
    if (float(bridge_vector @ bridge_tangent) <= 1e-9
            or np.linalg.norm(np.cross(
                bridge_vector, bridge_tangent)) > 1e-8):
        raise RuntimeError(
            "crown bridge is too short for the radius-controlled join fillet")
    target_world = np.array((
        -approach_start[1], approach_start[2],
        axis_z - approach_start[0],
    ))
    tip_path, tip_meta = _tip_path(
        feed, target_world, planner.guide,
        planner.guide_wire_radius_mm, rotation,
        arc_step_deg=policy.route_arc_step_deg)
    local_tip = np.column_stack((
        axis_z - tip_path[:, 2], -tip_path[:, 0], tip_path[:, 1],
    ))
    incoming = _unit(approach_start - local_tip[-2])
    join_arc = _sample_tangent_arc(
        approach_start, incoming, bridge_tangent,
        radius, policy.route_arc_step_deg)
    terminal_tangent = np.array((0.0, 0.0, -axial_sign))
    terminal_arc = _sample_tangent_arc(
        terminal_arc_start, bridge_tangent, terminal_tangent,
        radius, policy.route_arc_step_deg)

    raw = np.vstack((
        local_tip[:-1], join_arc,
        bridge_start, terminal_arc_start,
        terminal_arc, target,
    ))
    keep = [0]
    for index in range(1, len(raw)):
        if np.linalg.norm(raw[index] - raw[keep[-1]]) > 1e-9:
            keep.append(index)
    points = raw[keep]
    if np.linalg.norm(points[-1] - target) > 1e-8:
        raise RuntimeError("crown route endpoint changed")

    join_error = max(
        _angle_error_deg(
            approach_start - local_tip[-2], join_arc[1] - join_arc[0]),
        _angle_error_deg(
            join_arc[-1] - join_arc[-2],
            terminal_arc_start - bridge_start),
        _angle_error_deg(
            terminal_arc_start - bridge_start,
            terminal_arc[1] - terminal_arc[0]),
    )
    terminal_error = _angle_error_deg(
        terminal_arc[-1] - terminal_arc[-2], terminal_tangent)
    route_chord = radius * (
        1.0 - math.cos(math.radians(policy.route_arc_step_deg) / 2.0))
    return CrownRoute(
        points_local_mm=tuple(tuple(map(float, point)) for point in points),
        target_local_mm=tuple(map(float, target)),
        half_turn_index=int(half_turn_index),
        support_direction_local=tuple(map(float, support_direction)),
        support_cone_minimum_dot=cone_score,
        minimum_bend_radius_mm=radius,
        bridge_length_mm=float(policy.bridge_length_mm),
        tip_exit_tangent_error_deg=float(tip_meta.exit_tangent_error_deg),
        join_tangent_error_deg=float(join_error),
        terminal_tangent_error_deg=float(terminal_error),
        sampled_arc_chord_error_bound_mm=float(route_chord),
    )


def build_current_half_obstacle(
    graph: PackingSupportGraph,
    spec: Any,
    turn_index: int,
    physical_half_index: int,
    motion_sign: int,
    policy: CrownPolicy = DEFAULT_CROWN_POLICY,
) -> CurrentHalfObstacle:
    """Full arriving half-loop for one physical crossing and M2 sign."""

    if physical_half_index not in (0, 1):
        raise ValueError("physical half must be 0 or 1")
    if motion_sign not in (-1, 1):
        raise ValueError("motion sign must be -1 or +1")
    turn = graph.turn(turn_index)
    end_phase = float(motion_sign * physical_half_index) * math.pi
    start_phase = end_phase - float(motion_sign) * math.pi
    loop = crowned_loop_centerline(turn, spec, policy)
    phase_zero = np.array((
        float(turn.radial_mm),
        -_half_neck_mm(spec) - float(turn.profile_radius_mm),
        float(spec.stack) / 2.0,
    ))
    points = _closed_polyline_phase_subpath(
        loop, start_phase, end_phase, phase_zero)
    length = float(np.sum(np.linalg.norm(points[1:] - points[:-1], axis=1)))
    digest = hashlib.sha256(np.asarray(points, dtype="<f8").tobytes()).hexdigest()
    return CurrentHalfObstacle(
        turn_index=int(turn_index),
        physical_half_index=int(physical_half_index),
        motion_sign=int(motion_sign),
        start_phase_rad=float(start_phase),
        end_phase_rad=float(end_phase),
        points_local_mm=tuple(tuple(map(float, point)) for point in points),
        length_mm=length,
        sha256=digest,
    )


def _segment_closest_fractions(
    p0: np.ndarray, p1: np.ndarray,
    q0: np.ndarray, q1: np.ndarray,
) -> tuple[float, float, float]:
    """Closest fractions and distance for two finite 3D segments."""

    u, v, w = p1 - p0, q1 - q0, p0 - q0
    a, b, c = float(u @ u), float(u @ v), float(v @ v)
    d, e = float(u @ w), float(v @ w)
    denominator = a * c - b * b
    s_denominator = t_denominator = denominator
    if denominator < _EPS:
        s_numerator, s_denominator = 0.0, 1.0
        t_numerator, t_denominator = e, c
    else:
        s_numerator = b * e - c * d
        t_numerator = a * e - b * d
        if s_numerator < 0.0:
            s_numerator, t_numerator, t_denominator = 0.0, e, c
        elif s_numerator > s_denominator:
            s_numerator, t_numerator, t_denominator = (
                s_denominator, e + b, c)
    if t_numerator < 0.0:
        t_numerator = 0.0
        if -d < 0.0:
            s_numerator = 0.0
        elif -d > a:
            s_numerator = s_denominator
        else:
            s_numerator, s_denominator = -d, a
    elif t_numerator > t_denominator:
        t_numerator = t_denominator
        if -d + b < 0.0:
            s_numerator = 0.0
        elif -d + b > a:
            s_numerator = s_denominator
        else:
            s_numerator, s_denominator = -d + b, a
    s = 0.0 if abs(s_numerator) < _EPS else s_numerator / s_denominator
    t = 0.0 if abs(t_numerator) < _EPS else t_numerator / t_denominator
    delta = p0 + s * u - q0 - t * v
    return float(s), float(t), float(np.linalg.norm(delta))


def _boundary_constrained_pair(
    p0: np.ndarray, p1: np.ndarray,
    q0: np.ndarray, q1: np.ndarray,
    route_remaining_start: float,
    current_remaining_start: float,
    adjacency_limit: float,
) -> tuple[float, float, float] | None:
    """Minimum segment-pair distance on geodesic-sum == adjacency limit."""

    u, v = p1 - p0, q1 - q0
    route_length = float(np.linalg.norm(u))
    current_length = float(np.linalg.norm(v))
    if route_length <= _EPS or current_length <= _EPS:
        return None
    constant = (
        route_remaining_start + current_remaining_start - adjacency_limit)
    # route_length*s + current_length*t == constant
    low = max(0.0, (constant - current_length) / route_length)
    high = min(1.0, constant / route_length)
    if low > high + 1e-12:
        return None

    def value(s: float) -> tuple[float, float, float]:
        t = (constant - route_length * s) / current_length
        t = float(np.clip(t, 0.0, 1.0))
        delta = p0 + s * u - q0 - t * v
        return float(delta @ delta), float(s), t

    # Substituting t(s) makes the squared distance one quadratic.
    direction = u + (route_length / current_length) * v
    origin = p0 - q0 - (constant / current_length) * v
    denominator = float(direction @ direction)
    optimum = (low if denominator <= _EPS else float(np.clip(
        -float(origin @ direction) / denominator, low, high)))
    candidates = [value(low), value(high), value(optimum)]
    squared, s, t = min(candidates, key=lambda item: item[0])
    return s, t, math.sqrt(max(0.0, squared))


def adjacent_self_clearance(
    route_points_local_mm: Iterable[Iterable[float]],
    current_half: CurrentHalfObstacle,
    wire_diameter_mm: float,
    policy: CrownPolicy = DEFAULT_CROWN_POLICY,
    *,
    search_band_mm: float = 0.5,
) -> AdjacentSelfClearance:
    """Clearance with only <=2d combined endpoint adjacency exempted."""

    policy.validate()
    route = np.asarray(tuple(route_points_local_mm), dtype=float)
    current = np.asarray(current_half.points_local_mm, dtype=float)
    if (route.ndim != 2 or current.ndim != 2
            or route.shape[1:] != (3,) or current.shape[1:] != (3,)
            or len(route) < 2 or len(current) < 2
            or not np.all(np.isfinite(route))
            or not np.all(np.isfinite(current))):
        raise ValueError("route and current half must be finite 3D polylines")
    if np.linalg.norm(route[-1] - current[-1]) > 1e-7:
        raise ValueError("route/current half do not share their endpoint")
    limit = float(
        wire_diameter_mm * policy.adjacent_self_limit_diameters)
    route_lengths = np.linalg.norm(route[1:] - route[:-1], axis=1)
    current_lengths = np.linalg.norm(current[1:] - current[:-1], axis=1)
    route_remaining = np.concatenate((
        np.cumsum(route_lengths[::-1])[::-1], (0.0,)))
    current_remaining = np.concatenate((
        np.cumsum(current_lengths[::-1])[::-1], (0.0,)))

    starts = current[:-1]
    ends = current[1:]
    bounds = np.column_stack((np.minimum(starts, ends), np.maximum(starts, ends)))
    tree = trimesh.util.bounds_tree(bounds)
    best = float(search_band_mm)
    best_record: tuple[int, int, float, float, float] | None = None
    pair_count = 0
    for route_index, (p0, p1) in enumerate(zip(route, route[1:])):
        lower = np.minimum(p0, p1) - search_band_mm
        upper = np.maximum(p0, p1) + search_band_mm
        for current_index in sorted(tree.intersection((*lower, *upper))):
            pair_count += 1
            q0, q1 = starts[current_index], ends[current_index]
            s, t, distance = _segment_closest_fractions(p0, p1, q0, q1)
            route_geodesic = (
                route_remaining[route_index] - s * route_lengths[route_index])
            current_geodesic = (
                current_remaining[current_index]
                - t * current_lengths[current_index])
            geodesic = route_geodesic + current_geodesic
            if geodesic < limit - 1e-12:
                constrained = _boundary_constrained_pair(
                    p0, p1, q0, q1,
                    route_remaining[route_index],
                    current_remaining[current_index], limit)
                if constrained is None:
                    continue
                s, t, distance = constrained
                route_geodesic = (
                    route_remaining[route_index]
                    - s * route_lengths[route_index])
                current_geodesic = (
                    current_remaining[current_index]
                    - t * current_lengths[current_index])
                geodesic = route_geodesic + current_geodesic
            if distance < best:
                best = distance
                best_record = (
                    route_index, int(current_index), s, t, geodesic)
    if best_record is None:
        return AdjacentSelfClearance(
            best, None, None, None, None, None, limit, pair_count)
    return AdjacentSelfClearance(
        minimum_centerline_distance_mm=float(best),
        route_segment_index=best_record[0],
        current_segment_index=best_record[1],
        route_fraction=float(best_record[2]),
        current_fraction=float(best_record[3]),
        combined_geodesic_to_endpoint_mm=float(best_record[4]),
        adjacency_limit_mm=limit,
        candidate_pair_count=pair_count,
    )
