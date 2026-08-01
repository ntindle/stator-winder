"""Quasi-static load, travel, and spring audit for the passive dancer.

This module deliberately owns no CAD solids.  It consumes the authoritative
wire and machine parameters and answers the mechanical question that a static
wire rendering cannot: where does the spring-loaded dancer settle for each
wire tension from 1 N through 10 N?

The calculation is planar in the wire plane (XY, millimetres).  At every arm
angle it reconstructs both *exact external tangencies* to the moving pulley,
the clockwise pulley wrap, the wire moment about the arm pivot, and the length
of wire stored by the dancer.  A catalog extension spring is then evaluated at
its real free length, initial load, rate, and maximum extension.  Stable moment
equilibria are found numerically rather than assumed.

This is a quasi-static sizing audit, not a claim about transient damping,
spring fatigue, felt friction, or wire sag.  Those remain hardware tests.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from functools import lru_cache
import json
import math
from pathlib import Path
from typing import Iterable

from params import PARAMS as P
import wire_geometry


Vec2 = tuple[float, float]

HERE = Path(__file__).resolve().parent
REPORT = HERE.parent / "out" / "reports" / "dancer_loads.json"
MARKDOWN_REPORT = HERE.parent / "out" / "reports" / "dancer_loads.md"

# Shared source geometry.  The downstream endpoint is the tangent mouth of
# the fixed R4 entry elbow; using the virtual sharp corner would conceal the
# angular mismatch that appears when the dancer moves.
_STATIC = wire_geometry.static_path_spec()
_LM = _STATIC["landmarks"]
PIVOT: Vec2 = (P.rear_post_x, P.dancer_y)
NOMINAL_CENTER: Vec2 = tuple(_LM["dancer_center"][:2])  # type: ignore[assignment]
UPSTREAM: Vec2 = tuple(_LM["felt_contact"][:2])  # type: ignore[assignment]
DOWNSTREAM: Vec2 = tuple(_STATIC["entry_bend"]["start"][:2])  # type: ignore[assignment]
ENTRY_DIRECTION: Vec2 = tuple(
    _STATIC["entry_bend"]["incoming_direction"][:2]
)  # type: ignore[assignment]
PATH_RADIUS = float(_STATIC["dancer"]["path_radius"])
WIRE_RADIUS = wire_geometry.WIRE_RADIUS_MAX
ARM_LENGTH = math.dist(PIVOT, NOMINAL_CENTER)
NOMINAL_ANGLE_DEG = math.degrees(math.atan2(
    NOMINAL_CENTER[1] - PIVOT[1], NOMINAL_CENTER[0] - PIVOT[0]))

# The curved entry channel extends 7 mm outboard of its tangent point and has
# a 1.6 mm modeled radius in printed.py.  A moving straight segment must stay
# inside that mouth after the real maximum wire radius is subtracted.
ENTRY_CHANNEL_OPEN_LENGTH = 7.0
ENTRY_CHANNEL_RADIUS = 1.6
FELT_STUD_CENTER: Vec2 = (P.rear_post_x, P.felt_y)
FELT_STUD_RADIUS = 2.25  # M4 clearance envelope, including thread crest.

# The moving arm occupies z=-163..-160.5.  The extension spring must run in
# front of it; putting the spring or a fixed boss in the arm plane would make
# the nominal arm collide.  The selected OD3.5 spring is centered at z=-154.25
# in front of both the arm and a 3 mm-thick bridge while retaining ample XY
# clearance to the z=-157 wire.  A small overpass carries the fixed hook: its riser crosses
# the arm slab at a swept-clear XY root, then a front bridge reaches the
# analytically selected anchor without placing a post through the arm.
WIRE_PLANE_Z = float(_LM["dancer_center"][2])
ARM_Z = (P.rear_post_z + 17.0, P.rear_post_z + 19.5)
FIXED_BOSS_Z = (P.rear_post_z + 10.0, P.rear_post_z + 16.0)
SPRING_RISER_ROOT: Vec2 = (-22.0, 11.5)
SPRING_RISER_RADIUS = 4.5
SPRING_RISER_Z = (FIXED_BOSS_Z[0], -156.5)
SPRING_BRIDGE_Z = (-159.5, -156.5)
SPRING_BRIDGE_HALF_WIDTH = 3.0
SPRING_PLANE_Z = -154.25
FIXED_PRINT_TOP_Z = P.rear_post_z + 16.0
FIXED_PRINT_AXIAL_CLEARANCE = (WIRE_PLANE_Z - FIXED_PRINT_TOP_Z
                               - WIRE_RADIUS)

ANGLE_LIMIT_DEG = 12.0
FELT_DEFLECTION_LIMIT_DEG = 12.0
WRAP_LIMIT_DEG = (60.0, 120.0)
MIN_ENTRY_CHANNEL_CLEARANCE = 0.25
MIN_FELT_STUD_CLEARANCE = 1.0
MIN_SPRING_WIRE_CLEARANCE = 0.5
SPRING_RATED_USE_LIMIT = 0.87
STOP_CONTACT_RADIUS = 15.0
ARM_HALF_WIDTH = 5.0
STOP_PIN_RADIUS = 2.5
STOP_PIN_Z = (FIXED_BOSS_Z[1], -160.0)
STOP_WASHER_Z = (-160.0, -159.5)


@dataclass(frozen=True)
class SpringSpec:
    """Catalog extension-spring data in millimetres and newtons."""

    sku: str
    material: str
    free_length_mm: float
    max_length_mm: float
    initial_load_n: float
    max_load_n: float
    rate_n_per_mm: float
    od_mm: float
    source_url: str

    def force(self, length_mm: float) -> float | None:
        """Return catalog force, or ``None`` outside rated travel."""
        if length_mm < self.free_length_mm - 1e-9:
            return None
        if length_mm > self.max_length_mm + 1e-9:
            return None
        value = self.initial_load_n + self.rate_n_per_mm * (
            length_mm - self.free_length_mm)
        if value > self.max_load_n + 0.1:
            return None
        return value


# Rows are copied from current manufacturer/catalog tables, with exact unit
# conversion where the source is inch-based.  The search evaluates the stock
# options; the final report records which one fits rather than silently
# inventing a placeholder spring.  Lee Spring's compact instrument-series
# parts are included because the two longer McMaster springs do not generate
# enough rate in this short arm swing.
CATALOG_SPRINGS = (
    SpringSpec(
        sku="LE 020A 002",
        material="Lee Spring instrument series, music wire, random loops",
        free_length_mm=12.70,
        max_length_mm=17.53,
        initial_load_n=1.78,
        max_load_n=12.90,
        rate_n_per_mm=2.343,
        od_mm=3.18,
        source_url=("https://www.leespring.co.uk/sites/default/files/2020-01/"
                    "Extension_Springs_Lee_Spring_UK_Eng.pdf"),
    ),
    SpringSpec(
        sku="LE 022A 01",
        material="Lee Spring instrument series, music wire, random loops",
        free_length_mm=15.88,
        max_length_mm=21.21,
        initial_load_n=2.00,
        max_load_n=17.35,
        rate_n_per_mm=2.820,
        od_mm=3.18,
        source_url=("https://www.leespring.co.uk/sites/default/files/2020-01/"
                    "Extension_Springs_Lee_Spring_UK_Eng.pdf"),
    ),
    SpringSpec(
        sku="LEM050AB 01",
        material="Lee Spring metric instrument series, inline loops",
        free_length_mm=9.50,
        max_length_mm=13.82,
        initial_load_n=1.77,
        max_load_n=12.00,
        rate_n_per_mm=2.350,
        od_mm=3.50,
        source_url=("https://www.leespring.co.uk/sites/default/files/2020-01/"
                    "Extension_Springs_Lee_Spring_UK_Eng.pdf"),
    ),
    SpringSpec(
        sku="9433K398",
        material="302 stainless steel, loop ends",
        free_length_mm=1.25 * 25.4,
        max_length_mm=2.32 * 25.4,
        initial_load_n=0.33 * 4.4482216153,
        max_load_n=3.75 * 4.4482216153,
        rate_n_per_mm=3.17 * 4.4482216153 / 25.4,
        od_mm=0.24 * 25.4,
        source_url=("https://www.mcmaster.com/products/extension-springs/"
                    "material~302-stainless-steel/"),
    ),
    SpringSpec(
        sku="4992N311",
        material="316 stainless steel, loop ends",
        free_length_mm=1.25 * 25.4,
        max_length_mm=1.94 * 25.4,
        initial_load_n=0.32 * 4.4482216153,
        max_load_n=2.04 * 4.4482216153,
        rate_n_per_mm=2.5 * 4.4482216153 / 25.4,
        od_mm=0.30 * 25.4,
        source_url=("https://www.mcmaster.com/products/extension-springs/"
                    "material~316-stainless-steel/"),
    ),
)


@dataclass(frozen=True)
class WirePose:
    angle_deg: float
    center: Vec2
    tangent_in: Vec2
    tangent_out: Vec2
    wrap_deg: float
    path_length_mm: float
    moment_per_tension_mm: float
    felt_deflection_deg: float
    entry_direction_error_deg: float
    entry_channel_clearance_mm: float
    felt_stud_clearance_mm: float


@dataclass(frozen=True)
class SpringPose:
    moving_anchor: Vec2
    length_mm: float
    force_n: float
    lever_arm_mm: float
    moment_n_mm: float
    wire_clearance_mm: float


@dataclass(frozen=True)
class Design:
    spring: SpringSpec
    fixed_anchor: Vec2
    moving_anchor_radius_mm: float


def _add(a: Vec2, b: Vec2) -> Vec2:
    return (a[0] + b[0], a[1] + b[1])


def _sub(a: Vec2, b: Vec2) -> Vec2:
    return (a[0] - b[0], a[1] - b[1])


def _mul(a: Vec2, scalar: float) -> Vec2:
    return (a[0] * scalar, a[1] * scalar)


def _dot(a: Vec2, b: Vec2) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _cross(a: Vec2, b: Vec2) -> float:
    return a[0] * b[1] - a[1] * b[0]


def _norm(a: Vec2) -> float:
    return math.hypot(a[0], a[1])


def _unit(a: Vec2) -> Vec2:
    length = _norm(a)
    if length < 1e-12:
        raise ValueError("zero-length vector")
    return (a[0] / length, a[1] / length)


def _angle_deg(a: Vec2, b: Vec2) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0,
        _dot(_unit(a), _unit(b))))))


def _point_segment_distance(point: Vec2, a: Vec2, b: Vec2) -> float:
    ab = _sub(b, a)
    denominator = _dot(ab, ab)
    if denominator < 1e-12:
        return math.dist(point, a)
    t = max(0.0, min(1.0, _dot(_sub(point, a), ab) / denominator))
    return math.dist(point, _add(a, _mul(ab, t)))


def _segment_distance(a0: Vec2, a1: Vec2, b0: Vec2, b1: Vec2) -> float:
    """Minimum distance between two closed 2D line segments."""
    def orient(a: Vec2, b: Vec2, c: Vec2) -> float:
        return _cross(_sub(b, a), _sub(c, a))

    o1, o2 = orient(a0, a1, b0), orient(a0, a1, b1)
    o3, o4 = orient(b0, b1, a0), orient(b0, b1, a1)
    if o1 * o2 <= 0.0 and o3 * o4 <= 0.0:
        return 0.0
    return min(_point_segment_distance(a0, b0, b1),
               _point_segment_distance(a1, b0, b1),
               _point_segment_distance(b0, a0, a1),
               _point_segment_distance(b1, a0, a1))


def _circle_arm_clearance(center: Vec2, radius: float,
                          angle_deg: float) -> float:
    """Signed XY clearance from a circle to the finite 10 mm-wide arm.

    Negative means planar penetration.  This is used to prove why a Ø9 boss
    cannot share the arm's Z slab even though the intended Ø5 stop sleeve can.
    """
    angle = math.radians(angle_deg)
    along = (math.cos(angle), math.sin(angle))
    normal = (-math.sin(angle), math.cos(angle))
    delta = _sub(center, PIVOT)
    local_x = _dot(delta, along)
    local_y = _dot(delta, normal)
    dx = max(0.0, -local_x, local_x - ARM_LENGTH)
    dy = max(0.0, abs(local_y) - ARM_HALF_WIDTH)
    return math.hypot(dx, dy) - radius


def _z_gap(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Signed separation between closed axial intervals; <0 means overlap."""
    if a[1] < b[0]:
        return b[0] - a[1]
    if b[1] < a[0]:
        return a[0] - b[1]
    return -min(a[1], b[1]) + max(a[0], b[0])


def _external_tangencies(center: Vec2, point: Vec2) -> tuple[Vec2, Vec2]:
    delta = _sub(point, center)
    distance_sq = _dot(delta, delta)
    if distance_sq <= PATH_RADIUS * PATH_RADIUS:
        raise ValueError("wire endpoint lies on or inside dancer pulley")
    perpendicular = (-delta[1], delta[0])
    root = math.sqrt(distance_sq - PATH_RADIUS * PATH_RADIUS)
    radial = _mul(delta, PATH_RADIUS * PATH_RADIUS / distance_sq)
    offset = _mul(perpendicular, PATH_RADIUS * root / distance_sq)
    return (_add(center, _add(radial, offset)),
            _add(center, _sub(radial, offset)))


def _clockwise_tangent(center: Vec2, point: Vec2, *, incoming: bool) -> Vec2:
    """Select the unique external tangent traversed clockwise on the pulley."""
    ranked: list[tuple[float, Vec2]] = []
    for tangent in _external_tangencies(center, point):
        radial = _sub(tangent, center)
        theta = math.atan2(radial[1], radial[0])
        clockwise = (math.sin(theta), -math.cos(theta))
        path_direction = (_unit(_sub(tangent, point)) if incoming
                          else _unit(_sub(point, tangent)))
        ranked.append((_dot(clockwise, path_direction), tangent))
    score, result = max(ranked, key=lambda item: item[0])
    if score < 1.0 - 1e-9:
        raise AssertionError(f"failed to select exact tangent: dot={score}")
    return result


@lru_cache(maxsize=2048)
def wire_pose(angle_deg: float) -> WirePose:
    angle = math.radians(angle_deg)
    arm = (math.cos(angle), math.sin(angle))
    center = _add(PIVOT, _mul(arm, ARM_LENGTH))
    tangent_in = _clockwise_tangent(center, UPSTREAM, incoming=True)
    tangent_out = _clockwise_tangent(center, DOWNSTREAM, incoming=False)
    theta_in = math.atan2(tangent_in[1] - center[1],
                          tangent_in[0] - center[0])
    theta_out = math.atan2(tangent_out[1] - center[1],
                           tangent_out[0] - center[0])
    wrap = (theta_in - theta_out) % (2.0 * math.pi)

    force_in = _unit(_sub(UPSTREAM, tangent_in))
    force_out = _unit(_sub(DOWNSTREAM, tangent_out))
    moment_per_tension = (
        _cross(_sub(tangent_in, PIVOT), force_in)
        + _cross(_sub(tangent_out, PIVOT), force_out))
    path_length = (math.dist(UPSTREAM, tangent_in) + PATH_RADIUS * wrap
                   + math.dist(tangent_out, DOWNSTREAM))

    incoming_direction = _unit(_sub(tangent_in, UPSTREAM))
    outgoing_direction = _unit(_sub(DOWNSTREAM, tangent_out))
    felt_deflection = _angle_deg((0.0, 1.0), incoming_direction)
    entry_error = _angle_deg(ENTRY_DIRECTION, outgoing_direction)
    mouth_deviation = ENTRY_CHANNEL_OPEN_LENGTH * math.sin(
        math.radians(entry_error))
    entry_clearance = (ENTRY_CHANNEL_RADIUS - WIRE_RADIUS
                       - mouth_deviation)
    stud_clearance = (_point_segment_distance(
        FELT_STUD_CENTER, UPSTREAM, tangent_in)
        - FELT_STUD_RADIUS - WIRE_RADIUS)
    return WirePose(
        angle_deg=angle_deg,
        center=center,
        tangent_in=tangent_in,
        tangent_out=tangent_out,
        wrap_deg=math.degrees(wrap),
        path_length_mm=path_length,
        moment_per_tension_mm=moment_per_tension,
        felt_deflection_deg=felt_deflection,
        entry_direction_error_deg=entry_error,
        entry_channel_clearance_mm=entry_clearance,
        felt_stud_clearance_mm=stud_clearance,
    )


def spring_pose(design: Design, angle_deg: float,
                pose: WirePose | None = None) -> SpringPose | None:
    angle = math.radians(angle_deg)
    moving = _add(PIVOT, _mul((math.cos(angle), math.sin(angle)),
                              design.moving_anchor_radius_mm))
    spring_vector = _sub(design.fixed_anchor, moving)
    length = _norm(spring_vector)
    force = design.spring.force(length)
    if force is None:
        return None
    direction = _unit(spring_vector)
    lever = _cross(_sub(moving, PIVOT), direction)
    moment = force * lever
    pose = pose or wire_pose(angle_deg)
    xy_clearance = min(
        _segment_distance(moving, design.fixed_anchor,
                          UPSTREAM, pose.tangent_in),
        _segment_distance(moving, design.fixed_anchor,
                          pose.tangent_out, DOWNSTREAM),
    )
    spatial_distance = math.hypot(xy_clearance,
                                  SPRING_PLANE_Z - WIRE_PLANE_Z)
    wire_clearance = (spatial_distance - design.spring.od_mm / 2.0
                      - WIRE_RADIUS)
    return SpringPose(moving, length, force, lever, moment, wire_clearance)


def net_moment(design: Design, tension_n: float, angle_deg: float) -> float | None:
    pose = wire_pose(angle_deg)
    angle = math.radians(angle_deg)
    moving = _add(PIVOT, _mul((math.cos(angle), math.sin(angle)),
                              design.moving_anchor_radius_mm))
    spring_vector = _sub(design.fixed_anchor, moving)
    length = _norm(spring_vector)
    force = design.spring.force(length)
    if force is None:
        return None
    spring_moment = force * _cross(_sub(moving, PIVOT),
                                   _unit(spring_vector))
    return tension_n * pose.moment_per_tension_mm + spring_moment


def _stable_derivative(design: Design, tension_n: float,
                       angle_deg: float) -> float:
    delta = 0.01
    lo = net_moment(design, tension_n, angle_deg - delta)
    hi = net_moment(design, tension_n, angle_deg + delta)
    if lo is None or hi is None:
        return math.inf
    # N*mm per degree.  Negative slope is a restoring equilibrium.
    return (hi - lo) / (2.0 * delta)


def equilibrium(design: Design, tension_n: float,
                lower_offset_deg: float = -ANGLE_LIMIT_DEG,
                upper_offset_deg: float = ANGLE_LIMIT_DEG) -> float | None:
    """Find the unique stable equilibrium inside the allowed arm swing."""
    lo_angle = NOMINAL_ANGLE_DEG + lower_offset_deg
    hi_angle = NOMINAL_ANGLE_DEG + upper_offset_deg
    step = 0.25
    samples: list[tuple[float, float]] = []
    angle = lo_angle
    while angle <= hi_angle + 1e-9:
        value = net_moment(design, tension_n, angle)
        if value is not None:
            samples.append((angle, value))
        angle += step

    roots: list[float] = []
    for (a0, v0), (a1, v1) in zip(samples, samples[1:]):
        if v0 == 0.0:
            roots.append(a0)
            continue
        if v0 * v1 > 0.0:
            continue
        lo, hi = a0, a1
        vlo = v0
        for _ in range(50):
            mid = (lo + hi) / 2.0
            vmid = net_moment(design, tension_n, mid)
            if vmid is None:
                break
            if abs(vmid) < 1e-8:
                lo = hi = mid
                break
            if vlo * vmid <= 0.0:
                hi = mid
            else:
                lo, vlo = mid, vmid
        root = (lo + hi) / 2.0
        if _stable_derivative(design, tension_n, root) < 0.0:
            roots.append(root)
    if not roots:
        return None
    return min(roots, key=lambda value: abs(value - NOMINAL_ANGLE_DEG))


def _equilibrium_row(design: Design, tension: float,
                     nominal_length: float) -> dict | None:
    angle = equilibrium(design, tension)
    if angle is None:
        return None
    wire = wire_pose(angle)
    spring = spring_pose(design, angle, wire)
    assert spring is not None
    residual = tension * wire.moment_per_tension_mm + spring.moment_n_mm
    return {
        "tension_n": tension,
        "angle_deg": angle,
        "offset_deg": angle - NOMINAL_ANGLE_DEG,
        "pulley_center": list(wire.center),
        "tangent_in": list(wire.tangent_in),
        "tangent_out": list(wire.tangent_out),
        "wrap_deg": wire.wrap_deg,
        "wire_path_length_mm": wire.path_length_mm,
        "wire_takeup_from_nominal_mm": wire.path_length_mm - nominal_length,
        "felt_deflection_deg": wire.felt_deflection_deg,
        "entry_direction_error_deg": wire.entry_direction_error_deg,
        "entry_channel_clearance_mm": wire.entry_channel_clearance_mm,
        "felt_stud_clearance_mm": wire.felt_stud_clearance_mm,
        "wire_moment_n_mm": tension * wire.moment_per_tension_mm,
        "spring_length_mm": spring.length_mm,
        "spring_force_n": spring.force_n,
        "spring_lever_mm": spring.lever_arm_mm,
        "spring_moment_n_mm": spring.moment_n_mm,
        "spring_wire_clearance_mm": spring.wire_clearance_mm,
        "segment_clearances_mm": {
            "incoming_to_felt_stud": wire.felt_stud_clearance_mm,
            "incoming_to_felt_printed_base_axial": FIXED_PRINT_AXIAL_CLEARANCE,
            "outgoing_in_entry_channel_mouth": wire.entry_channel_clearance_mm,
            "outgoing_to_entry_support_axial": FIXED_PRINT_AXIAL_CLEARANCE,
            "wire_to_extension_spring": spring.wire_clearance_mm,
        },
        "moment_residual_n_mm": residual,
        "stability_n_mm_per_deg": _stable_derivative(design, tension, angle),
    }


def evaluate_design(design: Design) -> dict:
    nominal_length = wire_pose(NOMINAL_ANGLE_DEG).path_length_mm
    rows = [_equilibrium_row(design, float(tension), nominal_length)
            for tension in range(1, 11)]
    failures: list[str] = []
    if any(row is None for row in rows):
        failures.append("no stable rated equilibrium for every integer tension 1-10 N")
        return {"ok": False, "failures": failures, "sweep": rows}
    sweep: list[dict] = [row for row in rows if row is not None]
    offsets = [row["offset_deg"] for row in sweep]
    if any(abs(value) > ANGLE_LIMIT_DEG + 1e-6 for value in offsets):
        failures.append("equilibrium exceeds allowed arm swing")
    if any(b <= a for a, b in zip(offsets, offsets[1:])):
        failures.append("equilibrium angle is not monotonic with tension")
    if min(row["wrap_deg"] for row in sweep) < WRAP_LIMIT_DEG[0] or \
            max(row["wrap_deg"] for row in sweep) > WRAP_LIMIT_DEG[1]:
        failures.append("pulley wrap exits 60-120 degree envelope")
    if max(row["felt_deflection_deg"] for row in sweep) > FELT_DEFLECTION_LIMIT_DEG:
        failures.append("felt exit deflection exceeds limit")
    if min(row["entry_channel_clearance_mm"] for row in sweep) \
            < MIN_ENTRY_CHANNEL_CLEARANCE:
        failures.append("moving tangent does not fit fixed entry channel mouth")
    if min(row["felt_stud_clearance_mm"] for row in sweep) \
            < MIN_FELT_STUD_CLEARANCE:
        failures.append("wire approaches felt stud")
    if min(row["spring_wire_clearance_mm"] for row in sweep) \
            < MIN_SPRING_WIRE_CLEARANCE:
        failures.append("spring envelope approaches tangential wire")
    if max(abs(row["moment_residual_n_mm"]) for row in sweep) > 1e-5:
        failures.append("moment solver residual exceeds tolerance")
    if any(row["stability_n_mm_per_deg"] >= 0.0 for row in sweep):
        failures.append("non-restoring equilibrium")

    # Leave catalog travel/load headroom for normal spring tolerances and
    # printed-hole placement error.  This is a sizing derate, not a fatigue
    # certification.
    derated_max_length = (design.spring.free_length_mm
                          + SPRING_RATED_USE_LIMIT
                          * (design.spring.max_length_mm
                             - design.spring.free_length_mm))
    derated_max_force = SPRING_RATED_USE_LIMIT * design.spring.max_load_n
    if max(row["spring_length_mm"] for row in sweep) > derated_max_length:
        failures.append("spring exceeds derated catalog travel")
    if max(row["spring_force_n"] for row in sweep) > derated_max_force:
        failures.append("spring exceeds derated catalog load")

    # Mechanical stops give at least 0.5 degree beyond the 1 N and 10 N
    # equilibria, rounded outward to printable half-degree datums.  Sweep the
    # complete stop-to-stop envelope independently of the equilibrium solver.
    stop_lo = math.floor((min(offsets) - 0.5) * 2.0) / 2.0
    stop_hi = math.ceil((max(offsets) + 0.5) * 2.0) / 2.0
    angle_sweep: list[dict] = []
    offset = float(stop_lo)
    while offset <= stop_hi + 1e-9:
        angle = NOMINAL_ANGLE_DEG + offset
        wire = wire_pose(angle)
        spring = spring_pose(design, angle, wire)
        if spring is None:
            failures.append("spring leaves rated travel inside hard stops")
            break
        equivalent_tension = (-spring.moment_n_mm
                              / wire.moment_per_tension_mm)
        angle_sweep.append({
            "offset_deg": offset,
            "pulley_center": list(wire.center),
            "tangent_in": list(wire.tangent_in),
            "tangent_out": list(wire.tangent_out),
            "wrap_deg": wire.wrap_deg,
            "wire_path_length_mm": wire.path_length_mm,
            "wire_takeup_from_nominal_mm": wire.path_length_mm - nominal_length,
            "felt_deflection_deg": wire.felt_deflection_deg,
            "entry_direction_error_deg": wire.entry_direction_error_deg,
            "entry_channel_clearance_mm": wire.entry_channel_clearance_mm,
            "felt_stud_clearance_mm": wire.felt_stud_clearance_mm,
            "spring_length_mm": spring.length_mm,
            "spring_force_n": spring.force_n,
            "spring_lever_mm": spring.lever_arm_mm,
            "spring_moment_n_mm": spring.moment_n_mm,
            "spring_wire_clearance_mm": spring.wire_clearance_mm,
            "segment_clearances_mm": {
                "incoming_to_felt_stud": wire.felt_stud_clearance_mm,
                "incoming_to_felt_printed_base_axial":
                    FIXED_PRINT_AXIAL_CLEARANCE,
                "outgoing_in_entry_channel_mouth":
                    wire.entry_channel_clearance_mm,
                "outgoing_to_entry_support_axial":
                    FIXED_PRINT_AXIAL_CLEARANCE,
                "wire_to_extension_spring": spring.wire_clearance_mm,
            },
            "equivalent_equilibrium_tension_n": equivalent_tension,
        })
        offset += 0.25
    if angle_sweep:
        if min(row["wrap_deg"] for row in angle_sweep) < WRAP_LIMIT_DEG[0] or \
                max(row["wrap_deg"] for row in angle_sweep) > WRAP_LIMIT_DEG[1]:
            failures.append("hard-stop sweep exits wrap envelope")
        if max(row["felt_deflection_deg"] for row in angle_sweep) \
                > FELT_DEFLECTION_LIMIT_DEG:
            failures.append("hard-stop sweep exceeds felt deflection")
        if min(row["entry_channel_clearance_mm"] for row in angle_sweep) \
                < MIN_ENTRY_CHANNEL_CLEARANCE:
            failures.append("hard-stop sweep approaches entry channel")
        if min(row["felt_stud_clearance_mm"] for row in angle_sweep) \
                < MIN_FELT_STUD_CLEARANCE:
            failures.append("hard-stop sweep approaches felt stud")
        if min(row["spring_wire_clearance_mm"] for row in angle_sweep) \
                < MIN_SPRING_WIRE_CLEARANCE:
            failures.append("hard-stop sweep approaches wire with spring")
        # The continuously occupied 1-10 N equilibria obey the 87% derate.
        # A hard-stop excursion need only remain inside the catalog's absolute
        # rated length/load; spring_pose() has already enforced those limits.
        if angle_sweep[0]["equivalent_equilibrium_tension_n"] >= 1.0 or \
                angle_sweep[-1]["equivalent_equilibrium_tension_n"] <= 10.0:
            failures.append("hard stops do not bracket 1-10 N equilibrium range")

    return {
        "ok": not failures,
        "failures": failures,
        "sweep": sweep,
        "angle_sweep": angle_sweep,
        "metrics": {
            "offset_range_deg": [min(offsets), max(offsets)],
            "swing_span_deg": max(offsets) - min(offsets),
            "max_abs_offset_deg": max(abs(value) for value in offsets),
            "wrap_range_deg": [min(row["wrap_deg"] for row in sweep),
                               max(row["wrap_deg"] for row in sweep)],
            "wire_takeup_range_mm": [
                min(row["wire_takeup_from_nominal_mm"] for row in sweep),
                max(row["wire_takeup_from_nominal_mm"] for row in sweep)],
            "spring_length_range_mm": [min(row["spring_length_mm"] for row in sweep),
                                       max(row["spring_length_mm"] for row in sweep)],
            "spring_force_range_n": [min(row["spring_force_n"] for row in sweep),
                                     max(row["spring_force_n"] for row in sweep)],
            "minimum_entry_channel_clearance_mm": min(
                row["entry_channel_clearance_mm"] for row in sweep),
            "minimum_felt_stud_clearance_mm": min(
                row["felt_stud_clearance_mm"] for row in sweep),
            "minimum_spring_wire_clearance_mm": min(
                row["spring_wire_clearance_mm"] for row in sweep),
            "fixed_print_axial_clearance_mm": FIXED_PRINT_AXIAL_CLEARANCE,
            "hard_stop_offsets_deg": [stop_lo, stop_hi],
            "derated_spring_max_length_mm": derated_max_length,
            "derated_spring_max_force_n": derated_max_force,
            "spring_rated_extension_usage_fraction": (
                (max(row["spring_length_mm"] for row in sweep)
                 - design.spring.free_length_mm)
                / (design.spring.max_length_mm
                   - design.spring.free_length_mm)),
            "spring_rated_load_usage_fraction": (
                max(row["spring_force_n"] for row in sweep)
                / design.spring.max_load_n),
        },
    }


def search_design() -> tuple[Design, dict, list[dict]]:
    """Search anchors mountable on the existing entry-bracket support.

    The +Y entry support spans x=-45..6 and y=6..16.  Keeping the spring
    anchor in its rear 10 mm avoids a new cantilever from the dancer base and
    allows a short axial boss to place the hook in the arm plane.
    """
    candidates: list[tuple[tuple[float, ...], Design, dict]] = []
    summaries: list[dict] = []
    # Fixed anchors remain on or immediately beside the rear-post plate.
    # Half-millimetre resolution is smaller than practical printed-hole
    # placement tolerance and makes the result deterministic.
    for spring in CATALOG_SPRINGS:
        for moving_radius in (40.0, 42.0, 44.0, 45.0, 46.0, 47.0, 48.0):
            for ix in range(-42, -37):  # robust overlap with entry support
                x = float(ix)
                for iy in range(9, 13):
                    y = float(iy)
                    design = Design(spring, (x, y), moving_radius)
                    result = evaluate_design(design)
                    if not result["ok"]:
                        continue
                    metrics = result["metrics"]
                    # Prefer little total swing and good centering, then
                    # generous guide/spring clearances and a compact anchor.
                    score = (
                        metrics["max_abs_offset_deg"],
                        metrics["swing_span_deg"],
                        -metrics["minimum_entry_channel_clearance_mm"],
                        -metrics["minimum_spring_wire_clearance_mm"],
                        abs(x - P.rear_post_x) + abs(y - 10.0),
                    )
                    candidates.append((score, design, result))
        count = sum(1 for _, design, _ in candidates if design.spring == spring)
        summaries.append({"sku": spring.sku, "feasible_candidates": count})
    if not candidates:
        raise RuntimeError("no spring/anchor design satisfies the audit envelope")
    _, design, result = min(candidates, key=lambda item: item[0])
    return design, result, summaries


def _rounded(value, digits: int = 6):
    if isinstance(value, float):
        return round(value, digits)
    if isinstance(value, list):
        return [_rounded(item, digits) for item in value]
    if isinstance(value, tuple):
        return [_rounded(item, digits) for item in value]
    if isinstance(value, dict):
        return {key: _rounded(item, digits) for key, item in value.items()}
    return value


def hard_stop_pin_centers(offsets: Iterable[float]) -> list[dict]:
    """Return exact fixed pin centers for the two arm-angle limits.

    A Ø5 fixed pin contacts the 10 mm-wide arm at radius 15 mm.  The lower
    pin sits on the clockwise side; the upper pin sits on the counterclockwise
    side.  Removing the unused 15 mm spring hole keeps the contact section
    solid.
    """
    values = list(offsets)
    if len(values) != 2:
        raise ValueError("hard stops require lower and upper offsets")
    rows = []
    for name, offset, side in (("clockwise_lower", values[0], -1.0),
                               ("counterclockwise_upper", values[1], 1.0)):
        angle = math.radians(NOMINAL_ANGLE_DEG + offset)
        arm = (math.cos(angle), math.sin(angle))
        normal_ccw = (-math.sin(angle), math.cos(angle))
        center = _add(
            _add(PIVOT, _mul(arm, STOP_CONTACT_RADIUS)),
            _mul(normal_ccw, side * (ARM_HALF_WIDTH + STOP_PIN_RADIUS)),
        )
        rows.append({"name": name, "offset_deg": offset,
                     "center_xy": list(center)})
    return rows


def build_report() -> dict:
    design, result, search_summary = search_design()
    fixed_x, fixed_y = design.fixed_anchor
    stop_offsets = result["metrics"]["hard_stop_offsets_deg"]
    stop_pins = hard_stop_pin_centers(stop_offsets)
    nominal_stop_checks = []
    for row in stop_pins:
        center = tuple(row["center_xy"])
        nominal_stop_checks.append({
            "name": row["name"],
            "flawed_r4p5_boss_xy_clearance_at_nominal_mm":
                _circle_arm_clearance(center, 4.5, NOMINAL_ANGLE_DEG),
            "correct_r2p5_pin_xy_clearance_at_nominal_mm":
                _circle_arm_clearance(center, STOP_PIN_RADIUS,
                                      NOMINAL_ANGLE_DEG),
            "correct_pin_clearance_at_target_stop_mm":
                _circle_arm_clearance(center, STOP_PIN_RADIUS,
                                      NOMINAL_ANGLE_DEG + row["offset_deg"]),
        })
    spring_radius = design.spring.od_mm / 2.0
    spring_body_z = (SPRING_PLANE_Z - spring_radius,
                     SPRING_PLANE_Z + spring_radius)
    fixed_spring_boss_xy_nominal = _circle_arm_clearance(
        design.fixed_anchor, 4.5, NOMINAL_ANGLE_DEG)
    riser_arm_clearances = []
    bridge_wire_clearances = []
    moving_pin_bridge_clearances = []
    for row in result["angle_sweep"]:
        angle = NOMINAL_ANGLE_DEG + row["offset_deg"]
        wire = wire_pose(angle)
        riser_arm_clearances.append(_circle_arm_clearance(
            SPRING_RISER_ROOT, SPRING_RISER_RADIUS, angle))
        bridge_distance = min(
            _segment_distance(SPRING_RISER_ROOT, design.fixed_anchor,
                              UPSTREAM, wire.tangent_in),
            _segment_distance(SPRING_RISER_ROOT, design.fixed_anchor,
                              wire.tangent_out, DOWNSTREAM),
        )
        bridge_wire_clearances.append(
            bridge_distance - SPRING_BRIDGE_HALF_WIDTH - WIRE_RADIUS)
        angle_rad = math.radians(angle)
        moving_anchor = _add(PIVOT, _mul(
            (math.cos(angle_rad), math.sin(angle_rad)),
            design.moving_anchor_radius_mm))
        moving_pin_bridge_clearances.append(
            _point_segment_distance(moving_anchor, SPRING_RISER_ROOT,
                                    design.fixed_anchor)
            - 2.0 - SPRING_BRIDGE_HALF_WIDTH)
    report = {
        "method": {
            "type": "quasi-static planar exact-tangent moment balance",
            "units": "mm, N, degrees",
            "tension_range_n": [1.0, 10.0],
            "equilibrium_samples_n": list(range(1, 11)),
            "angle_limit_deg_from_nominal": ANGLE_LIMIT_DEG,
            "limitations": [
                "does not model transient damping or flyer acceleration",
                "does not certify spring fatigue or hook life",
                "does not model felt friction hysteresis, wire sag, or snagging",
            ],
        },
        "source_geometry": {
            "pivot": list(PIVOT),
            "nominal_pulley_center": list(NOMINAL_CENTER),
            "arm_length_mm": ARM_LENGTH,
            "nominal_arm_angle_deg": NOMINAL_ANGLE_DEG,
            "path_radius_mm": PATH_RADIUS,
            "wire_radius_mm": WIRE_RADIUS,
            "upstream_felt_contact": list(UPSTREAM),
            "downstream_entry_tangent_mouth": list(DOWNSTREAM),
            "entry_nominal_direction": list(ENTRY_DIRECTION),
            "fixed_print_top_z_mm": FIXED_PRINT_TOP_Z,
            "fixed_print_axial_clearance_mm": FIXED_PRINT_AXIAL_CLEARANCE,
            "dancer_arm_z_mm": list(ARM_Z),
        },
        "requirements": {
            "wrap_deg": list(WRAP_LIMIT_DEG),
            "felt_deflection_max_deg": FELT_DEFLECTION_LIMIT_DEG,
            "entry_channel_clearance_min_mm": MIN_ENTRY_CHANNEL_CLEARANCE,
            "felt_stud_clearance_min_mm": MIN_FELT_STUD_CLEARANCE,
            "spring_wire_clearance_min_mm": MIN_SPRING_WIRE_CLEARANCE,
            "spring_catalog_load_and_travel_use_max_fraction":
                SPRING_RATED_USE_LIMIT,
        },
        "catalog_springs": [asdict(spring) for spring in CATALOG_SPRINGS],
        "search": {
            "fixed_anchor_domain": {"x_mm": [-42.0, -38.0],
                                    "y_mm": [9.0, 12.0],
                                    "grid_mm": 1.0},
            "moving_anchor_radii_mm": [40.0, 42.0, 44.0, 45.0, 46.0,
                                        47.0, 48.0],
            "summary": search_summary,
        },
        "recommended": {
            "spring": asdict(design.spring),
            "fixed_anchor": list(design.fixed_anchor),
            "moving_anchor_radius_mm": design.moving_anchor_radius_mm,
            "result": result,
            "axial_interference_audit": {
                "flaw_confirmed": True,
                "finding": (
                    "A radius-4.5 boss in z=-164..-159.5 overlaps the arm "
                    "axially and both stop bosses penetrate the arm in XY "
                    "at the nominal angle. The arm would be locked."),
                "nominal_stop_xy_checks": nominal_stop_checks,
                "fixed_spring_boss_if_in_arm_plane_clearance_mm":
                    fixed_spring_boss_xy_nominal,
                "corrected_fixed_boss_z_mm": list(FIXED_BOSS_Z),
                "fixed_boss_to_arm_axial_gap_mm":
                    _z_gap(FIXED_BOSS_Z, ARM_Z),
                "stop_pin_sleeve_z_mm": list(STOP_PIN_Z),
                "stop_pin_to_arm_axial_overlap_mm":
                    -_z_gap(STOP_PIN_Z, ARM_Z),
                "stop_front_washer_z_mm": list(STOP_WASHER_Z),
                "stop_washer_to_arm_axial_gap_mm":
                    _z_gap(STOP_WASHER_Z, ARM_Z),
                "spring_center_plane_z_mm": SPRING_PLANE_Z,
                "spring_body_z_mm": list(spring_body_z),
                "spring_body_to_arm_axial_gap_mm":
                    _z_gap(spring_body_z, ARM_Z),
                "spring_bridge_z_mm": list(SPRING_BRIDGE_Z),
                "spring_bridge_to_arm_axial_gap_mm":
                    _z_gap(SPRING_BRIDGE_Z, ARM_Z),
                "spring_body_to_bridge_axial_gap_mm":
                    _z_gap(spring_body_z, SPRING_BRIDGE_Z),
                "spring_riser_root_xy": list(SPRING_RISER_ROOT),
                "spring_riser_min_arm_clearance_mm":
                    min(riser_arm_clearances),
                "spring_bridge_min_wire_clearance_mm":
                    min(bridge_wire_clearances),
                "moving_spring_pin_to_bridge_min_clearance_mm":
                    min(moving_pin_bridge_clearances),
            },
            "printed_coordinate_changes": {
                "remove_dancer_base_fixed_spring_eye": [
                    P.rear_post_x - 10.0, P.dancer_y],
                "entry_bracket_spring_overpass": {
                    "riser_root_xy": list(SPRING_RISER_ROOT),
                    "riser_radius_mm": SPRING_RISER_RADIUS,
                    "riser_z_range_mm": list(SPRING_RISER_Z),
                    "front_bridge_start_xy": list(SPRING_RISER_ROOT),
                    "front_bridge_end_xy": [fixed_x, fixed_y],
                    "front_bridge_width_mm": 2.0 * SPRING_BRIDGE_HALF_WIDTH,
                    "front_bridge_z_range_mm": list(SPRING_BRIDGE_Z),
                    "anchor_pad_radius_mm": 4.5,
                    "anchor_hole_radius_mm": 1.2,
                },
                "dancer_arm_selected_spring_hole_distance_mm":
                    design.moving_anchor_radius_mm,
                "dancer_arm_spring_hole_radius_mm": 1.2,
                "remove_unused_arm_spring_holes_mm": [15.0, 25.0, 35.0],
                "add_hard_stops_at_offset_deg":
                    stop_offsets,
                "dancer_base_y_bounds_mm": [
                    P.dancer_y + min(P.dancer_base_mount_offsets) - 5.0,
                    P.dancer_y + max(P.dancer_base_mount_offsets) + 5.0,
                ],
                "hard_stop_pins": {
                    "contact_radius_from_pivot_mm": STOP_CONTACT_RADIUS,
                    "pin_radius_mm": STOP_PIN_RADIUS,
                    "boss_radius_mm": 4.5,
                    "hole_radius_mm": 1.7,
                    "boss_z_range_mm": list(FIXED_BOSS_Z),
                    "pin_sleeve_z_range_mm": list(STOP_PIN_Z),
                    "front_washer_z_range_mm": list(STOP_WASHER_Z),
                    "centers": stop_pins,
                },
                "spring_center_plane_z_mm": SPRING_PLANE_Z,
                "pin_hardware_stack": {
                    "hard_stops_qty_2": [
                        "M3x16 socket-head screw inserted from rear",
                        "OD5 x ID3.2 x 4.0 mm steel spacer sleeve, z=-164..-160",
                        "M3 washer, z=-160..-159.5",
                        "M3 nyloc nut entirely forward of z=-159.5",
                    ],
                    "fixed_spring_anchor": [
                        "M2x12 screw through the front overpass anchor pad",
                        "OD4 x ID2.2 x 1.5 mm spacer forward of bridge",
                        "M2 washer, spring loop centered at z=-154.25, M2 washer",
                        "M2 nyloc nut",
                    ],
                    "moving_spring_anchor": [
                        "M2x16 screw inserted from arm rear",
                        "OD4 x ID2.2 x 4.0 mm spacer forward of arm",
                        "M2 washer, spring loop centered at z=-154.25, M2 washer",
                        "M2 nyloc nut",
                    ],
                },
            },
        },
        "checks": {
            "all_tensions_have_stable_equilibrium": result["ok"],
            "exact_nominal_wrap_80_deg": abs(
                wire_pose(NOMINAL_ANGLE_DEG).wrap_deg - 80.0) < 1e-6,
            "catalog_spring_stays_in_rated_travel": result["ok"],
            "wire_segments_clear_hardware_envelopes": result["ok"],
        },
    }
    report["fail"] = [name for name, ok in report["checks"].items() if not ok]
    return _rounded(report)


def _markdown(report: dict) -> str:
    rec = report["recommended"]
    result = rec["result"]
    metrics = result["metrics"]
    spring = rec["spring"]
    changes = rec["printed_coordinate_changes"]
    axial = rec["axial_interference_audit"]
    lines = [
        "# Dancer quasi-static load and travel audit",
        "",
        f"Result: **{'PASS' if not report['fail'] else 'FAIL'}**",
        "",
        ("Exact external wire tangencies and moment balance were solved at "
         "every integer tension from 1 N through 10 N. The full mechanical "
         "stop-to-stop angle range was then swept in 0.25 degree steps."),
        "",
        "## Recommended spring and anchors",
        "",
        f"- Spring: Lee Spring `{spring['sku']}` ({spring['material']})",
        (f"- Catalog data: free {spring['free_length_mm']:.2f} mm, rate "
         f"{spring['rate_n_per_mm']:.3f} N/mm, initial load "
         f"{spring['initial_load_n']:.2f} N, max load "
         f"{spring['max_load_n']:.2f} N, max length "
         f"{spring['max_length_mm']:.2f} mm, OD {spring['od_mm']:.2f} mm"),
        f"- Catalog source: {spring['source_url']}",
        f"- Fixed anchor XY: {rec['fixed_anchor']} mm",
        (f"- Moving anchor: {rec['moving_anchor_radius_mm']:.1f} mm from "
         "the dancer pivot along the arm"),
        (f"- Hard stops: {metrics['hard_stop_offsets_deg'][0]:.2f} to "
         f"{metrics['hard_stop_offsets_deg'][1]:.2f} degrees from nominal"),
        "",
        "## Envelope",
        "",
        (f"- Equilibrium swing: {metrics['offset_range_deg'][0]:.3f} to "
         f"{metrics['offset_range_deg'][1]:.3f} degrees"),
        (f"- Pulley wrap: {metrics['wrap_range_deg'][0]:.3f} to "
         f"{metrics['wrap_range_deg'][1]:.3f} degrees"),
        (f"- Wire storage relative to nominal: "
         f"{metrics['wire_takeup_range_mm'][0]:.3f} to "
         f"{metrics['wire_takeup_range_mm'][1]:.3f} mm"),
        (f"- Spring force: {metrics['spring_force_range_n'][0]:.3f} to "
         f"{metrics['spring_force_range_n'][1]:.3f} N"),
        (f"- Spring length: {metrics['spring_length_range_mm'][0]:.3f} to "
         f"{metrics['spring_length_range_mm'][1]:.3f} mm"),
        (f"- Rated spring use at 10 N: "
         f"{100 * metrics['spring_rated_extension_usage_fraction']:.1f}% "
         f"of extension and {100 * metrics['spring_rated_load_usage_fraction']:.1f}% "
         "of load"),
        (f"- Minimum entry-channel mouth clearance: "
         f"{metrics['minimum_entry_channel_clearance_mm']:.3f} mm"),
        (f"- Minimum felt-stud clearance: "
         f"{metrics['minimum_felt_stud_clearance_mm']:.3f} mm"),
        (f"- Minimum spring-to-wire clearance: "
         f"{metrics['minimum_spring_wire_clearance_mm']:.3f} mm"),
        "",
        "## Equilibrium sweep",
        "",
        "| Tension N | Offset deg | Wrap deg | Spring N | Spring mm | Entry clr mm | Felt defl deg |",
        "|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in result["sweep"]:
        lines.append(
            f"| {row['tension_n']:.0f} | {row['offset_deg']:.3f} | "
            f"{row['wrap_deg']:.3f} | {row['spring_force_n']:.3f} | "
            f"{row['spring_length_mm']:.3f} | "
            f"{row['entry_channel_clearance_mm']:.3f} | "
            f"{row['felt_deflection_deg']:.3f} |")
    lines.extend([
        "",
        "## Exact `printed.py` changes recommended",
        "",
        (f"- Remove old dancer-base fixed eye at "
         f"{changes['remove_dancer_base_fixed_spring_eye']} mm."),
        (f"- Add a spring overpass to `entry_bracket()`: radius-"
         f"{changes['entry_bracket_spring_overpass']['riser_radius_mm']} mm "
         f"riser at {changes['entry_bracket_spring_overpass']['riser_root_xy']} mm, "
         f"then a {changes['entry_bracket_spring_overpass']['front_bridge_width_mm']} mm-wide "
         f"front bridge to {changes['entry_bracket_spring_overpass']['front_bridge_end_xy']} mm "
         f"at Z {changes['entry_bracket_spring_overpass']['front_bridge_z_range_mm']} mm."),
        (f"- Add/select the arm spring hole at "
         f"{changes['dancer_arm_selected_spring_hole_distance_mm']} mm from the pivot."),
        (f"- Remove the unused arm holes at "
         f"{changes['remove_unused_arm_spring_holes_mm']} mm so the lower stop "
         "contacts solid arm material."),
        (f"- Add hard stops at offsets "
         f"{changes['add_hard_stops_at_offset_deg']} degrees."),
        (f"- Extend the dancer-base Y bounds to "
         f"{changes['dancer_base_y_bounds_mm']} mm and add Ø5 stop pins at "
         f"{[row['center_xy'] for row in changes['hard_stop_pins']['centers']]} mm."),
        "",
        "## Axial interference correction",
        "",
        ("The earlier suggestion to extend each radius-4.5 stop boss through "
         "z=-164..-159.5 was invalid. Both bosses would overlap the "
         "z=-163..-160.5 arm even at the nominal angle and lock it."),
        (f"- Keep every fixed radius-4.5 boss behind the arm at Z "
         f"{axial['corrected_fixed_boss_z_mm']} mm; axial gap to the arm is "
         f"{axial['fixed_boss_to_arm_axial_gap_mm']:.3f} mm."),
        (f"- Only the radius-{changes['hard_stop_pins']['pin_radius_mm']:.1f} "
         f"stop sleeve crosses the arm plane, Z "
         f"{changes['hard_stop_pins']['pin_sleeve_z_range_mm']} mm."),
        (f"- Put the extension spring in the front plane Z "
         f"{changes['spring_center_plane_z_mm']:.3f} mm; its body-to-arm "
         f"axial gap is {axial['spring_body_to_arm_axial_gap_mm']:.3f} mm."),
        (f"- The overpass riser stays {axial['spring_riser_min_arm_clearance_mm']:.3f} mm "
         f"from the swept arm and the front bridge stays "
         f"{axial['spring_bridge_min_wire_clearance_mm']:.3f} mm from the wire."),
        (f"- The moving spring pin stays "
         f"{axial['moving_spring_pin_to_bridge_min_clearance_mm']:.3f} mm "
         "from the fixed bridge."),
        "- Stop hardware, each: M3x16 rear-entry screw, OD5 x ID3.2 x 4 mm sleeve, M3 washer, and M3 nyloc entirely in front of the arm.",
        "",
        "## Limits",
        "",
        "This is a quasi-static sizing result. It does not validate transient damping, spring fatigue, felt-friction hysteresis, sag, or snagging.",
        "",
    ])
    return "\n".join(lines)


def write_report(path: Path = REPORT) -> dict:
    report = build_report()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    MARKDOWN_REPORT.write_text(_markdown(report), encoding="utf-8")
    return report


def main() -> int:
    report = write_report()
    recommendation = report["recommended"]
    result = recommendation["result"]
    spring = recommendation["spring"]
    print("=== dancer quasi-static audit ===")
    print(f"spring {spring['sku']}: free {spring['free_length_mm']:.2f} mm, "
          f"rate {spring['rate_n_per_mm']:.4f} N/mm")
    print(f"fixed anchor {recommendation['fixed_anchor']}; moving anchor "
          f"r={recommendation['moving_anchor_radius_mm']:.1f} mm")
    metrics = result["metrics"]
    print(f"equilibrium offset {metrics['offset_range_deg'][0]:.3f} .. "
          f"{metrics['offset_range_deg'][1]:.3f} deg")
    print(f"spring force {metrics['spring_force_range_n'][0]:.3f} .. "
          f"{metrics['spring_force_range_n'][1]:.3f} N")
    print(f"wrap {metrics['wrap_range_deg'][0]:.3f} .. "
          f"{metrics['wrap_range_deg'][1]:.3f} deg")
    print(f"entry clearance min "
          f"{metrics['minimum_entry_channel_clearance_mm']:.3f} mm")
    print(f"RESULT: {'PASS' if not report['fail'] else 'FAIL'}")
    print(f"report: {REPORT}")
    print(f"summary: {MARKDOWN_REPORT}")
    return 0 if not report["fail"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
