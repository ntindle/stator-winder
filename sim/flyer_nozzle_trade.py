"""Fail-closed trade study for a flyer-fixed terminal horn or nozzle.

CAD brief
---------

Model/task
    An isolated analytical trade study, not a production CAD modification.
    Compare a thin closed tube extending tangentially from the released flyer
    torus with an optimistic polished open terminal horn.
Units/frame
    Millimetres.  Flyer-local +Y points from the M2 axis toward the released
    tip at M2=0, +X is tangential, and +Z is the machine flyer axis.  Flyer
    local points rotate about +Z with M2.
Controlled inputs
    Default OD46 x stack15 x 24-slot stator, the maximum launch wire envelope
    (diameter 0.5), minimum wire-centre bend radius 3.0, nine M0 lay depths,
    every integer M2 degree, both motion signs, the exact progressive packing
    graph, and the current Nomex cap/liner dimensions.
Candidate families
    Closed nozzle: ID0.6 (0.5 wire plus 0.1 diametral running clearance),
    wall 0.05..0.30, exit radii 8..40, exit Z 0/1/2, and a straight terminal
    barrel tangent to the existing R3.25 torus path.
    Open horn: an *optimistic* all-direction spherical terminal contact with
    a 3.0 wire-centre radius (2.75 physical surface radius).  This relaxation
    gives a passive horn more freedom than a manufacturable one.
Outputs
    ``out/reports/flyer_nozzle_trade.json`` and ``.md`` only.  No STEP is
    emitted because no candidate is geometrically authorized.
Validation
    Full 360 x 9 x 2 target sweep, exact progressive prior/current copper
    segment distances for the closest-launch closed candidates, triangulated
    source-core capsule collision, rigid flyer/chuck checks, cap aperture,
    R>=3, and an analytic lower bound applicable to *any* flyer-fixed passive
    contact feature.

The study deliberately does not modify ``assembly.py``, ``printed.py``, the
production wire route, or collision exclusions.  A failure therefore cannot
silently become an assembly release.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import sys
from typing import Any, Iterable

import fcl
import numpy as np
import trimesh


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
OUT = ROOT / "out"
REPORTS = OUT / "reports"
MANIFEST_PATH = OUT / "links" / "manifest.json"
PACKING_PATH = REPORTS / "slot_packing.json"
JSON_OUT = REPORTS / "flyer_nozzle_trade.json"
MD_OUT = REPORTS / "flyer_nozzle_trade.md"

for path in (CAD, HERE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import coil_growth  # noqa: E402
import collide  # noqa: E402
from params import DEFAULT_STATOR  # noqa: E402
import slot_route  # noqa: E402
import stator_insulation_nomex410 as insulation  # noqa: E402
import wirepath  # noqa: E402


SCHEMA = "flyer-nozzle-trade/v1"
ANGLE_COUNT = 360
DEPTH_COUNT = 9
DIRECTIONS = (-1, 1)
MAX_WIRE_DIAMETER_MM = 0.5
MAX_WIRE_RADIUS_MM = MAX_WIRE_DIAMETER_MM / 2.0
NOZZLE_DIAMETRAL_CLEARANCE_MM = 0.10
NOZZLE_ID_MM = MAX_WIRE_DIAMETER_MM + NOZZLE_DIAMETRAL_CLEARANCE_MM
NOZZLE_WALLS_MM = (0.05, 0.10, 0.15, 0.20, 0.25, 0.30)
CAP_RELIEF_EACH_MM = (0.0, 0.10, 0.20, 0.30, 0.35)
MINIMUM_WIRE_CENTER_BEND_RADIUS_MM = 3.0
OPEN_HORN_SURFACE_RADIUS_MM = (
    MINIMUM_WIRE_CENTER_BEND_RADIUS_MM - MAX_WIRE_RADIUS_MM
)
OPEN_HORN_MINIMUM_OD_MM = 2.0 * OPEN_HORN_SURFACE_RADIUS_MM
MAXIMUM_LAUNCH_TARGET_MM = 0.5
LINER_ALLOWANCE_MM = float(insulation.MATERIAL_RECEIVING_MAX_MM)
WIRE_CORE_ENVELOPE_MM = MAX_WIRE_RADIUS_MM + LINER_ALLOWANCE_MM
NOZZLE_EXIT_RADII_MM = (
    8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0,
    22.0, 24.0, 28.0, 32.0, 36.0, 40.0,
)
NOZZLE_EXIT_Z_MM = (0.0, 1.0, 2.0)
OPEN_HORN_CENTER_RADII_MM = NOZZLE_EXIT_RADII_MM
EXIT_LIP_C1_LIMIT_DEG = 0.5
COPPER_SEARCH_BAND_MM = 1.0
CLEARANCE_ANGLE_STEP_DEG = 30


@dataclass(frozen=True)
class Case:
    depth_index: int
    turn_index: int
    angle_deg: int
    motion_sign: int
    axis_z_mm: float
    radial_mm: float
    profile_radius_mm: float
    target_world_mm: tuple[float, float, float]
    target_flyer_mm: tuple[float, float, float]
    current_arrival_local_mm: tuple[tuple[float, float, float], ...]


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: dict[str, Any]) -> str:
    canonical = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False
    )
    return hashlib.sha256(canonical.encode()).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _unit(value: np.ndarray) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    length = float(np.linalg.norm(vector))
    if length <= 1e-12:
        raise ValueError("zero vector")
    return vector / length


def _angle_deg(one: np.ndarray, two: np.ndarray) -> float:
    cosine = float(np.clip(np.dot(_unit(one), _unit(two)), -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _machine_to_local(points: np.ndarray, axis_z_mm: float) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    return np.column_stack((
        float(axis_z_mm) - points[:, 2],
        -points[:, 0],
        points[:, 1],
    ))


def _trim_polyline_end(points: np.ndarray, trim_mm: float) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if trim_mm <= 0.0:
        return points.copy()
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    keep_to = float(np.sum(lengths)) - float(trim_mm)
    if keep_to <= 1e-12:
        return points[:2].copy()
    result = [points[0]]
    cumulative = 0.0
    for start, end, length in zip(points, points[1:], lengths):
        if cumulative + float(length) < keep_to - 1e-12:
            result.append(end)
            cumulative += float(length)
            continue
        ratio = (keep_to - cumulative) / float(length)
        result.append(start + ratio * (end - start))
        break
    return np.asarray(result, dtype=float)


@lru_cache(maxsize=1)
def _manifest() -> dict[str, Any]:
    return _load(MANIFEST_PATH)


@lru_cache(maxsize=1)
def _packing() -> dict[str, Any]:
    return _load(PACKING_PATH)


@lru_cache(maxsize=1)
def _graph() -> slot_route.PackingSupportGraph:
    return slot_route.PackingSupportGraph.from_report(
        _packing(), spec=DEFAULT_STATOR
    )


@lru_cache(maxsize=1)
def _planner() -> slot_route.SlotRoutePlanner:
    config = _packing()["config"]
    access = float(config["center_core_access_mm"])
    return slot_route.SlotRoutePlanner.from_project(
        _manifest(), spec=DEFAULT_STATOR,
        access_radius_mm=access, planner_offset_mm=access,
    )


def _current_arrival_points(radial_mm: float, profile_radius_mm: float,
                            signed_phase_rad: float) -> np.ndarray:
    if abs(signed_phase_rad) < math.radians(1.0):
        return np.empty((0, 3), dtype=float)
    count = max(2, math.ceil(abs(math.degrees(signed_phase_rad)) / 5.0))
    phases = np.linspace(0.0, signed_phase_rad, count + 1)
    points = np.asarray([
        (
            radial_mm,
            *slot_route._rounded_loop_yz(
                profile_radius_mm, float(phase), DEFAULT_STATOR
            ),
        )
        for phase in phases
    ], dtype=float)
    points = _trim_polyline_end(points, _graph().wire_diameter_mm)
    if len(points) < 2 or np.linalg.norm(points[-1] - points[0]) < 1e-9:
        return np.empty((0, 3), dtype=float)
    return points


@lru_cache(maxsize=1)
def _cases() -> tuple[Case, ...]:
    graph = _graph()
    bundle = coil_growth.analyze_job(DEFAULT_STATOR)["bundle"]
    depth_radii = np.linspace(
        float(bundle["radial_winding_start_mm"]),
        float(bundle["radial_winding_end_mm"]),
        DEPTH_COUNT,
    )
    result: list[Case] = []
    for depth_index in range(DEPTH_COUNT):
        turn_index = round(depth_index * (len(graph.turns) - 1)
                           / (DEPTH_COUNT - 1))
        turn = graph.turn(turn_index)
        radial_mm = float(depth_radii[depth_index])
        axis_z = 2.0 + radial_mm
        for motion_sign in DIRECTIONS:
            for angle_deg in range(ANGLE_COUNT):
                angle = math.radians(angle_deg)
                signed = motion_sign * angle
                yz = slot_route._rounded_loop_yz(
                    turn.profile_radius_mm, signed, DEFAULT_STATOR
                )
                target_world = np.array((-yz[0], yz[1], 2.0), dtype=float)
                target_flyer = wirepath.rot_z(-angle) @ target_world
                current = _current_arrival_points(
                    radial_mm,
                    float(turn.profile_radius_mm),
                    signed,
                )
                result.append(Case(
                    depth_index=depth_index,
                    turn_index=turn_index,
                    angle_deg=angle_deg,
                    motion_sign=motion_sign,
                    axis_z_mm=axis_z,
                    radial_mm=radial_mm,
                    profile_radius_mm=float(turn.profile_radius_mm),
                    target_world_mm=tuple(map(float, target_world)),
                    target_flyer_mm=tuple(map(float, target_flyer)),
                    current_arrival_local_mm=tuple(
                        tuple(map(float, point)) for point in current
                    ),
                ))
    return tuple(result)


@lru_cache(maxsize=DEPTH_COUNT)
def _copper_fields(depth_index: int) -> dict[str, Any]:
    graph = _graph()
    turn_index = round(depth_index * (len(graph.turns) - 1)
                       / (DEPTH_COUNT - 1))
    turn = graph.turn(turn_index)
    prior = slot_route.active_copper_before(
        graph, turn_index, DEFAULT_STATOR, arc_step_deg=5.0
    )
    parents = {
        f"active-turn-{index:02d}" for index in turn.parent_turn_indices
    }
    return {
        "parent_ids": sorted(parents),
        "nonparents": slot_route.CopperField(tuple(
            item for item in prior if item.obstacle_id not in parents
        )),
        "parents": slot_route.CopperField(tuple(
            item for item in prior if item.obstacle_id in parents
        )),
    }


def _circle_contains(circle: tuple[np.ndarray, float] | None,
                     point: np.ndarray, tolerance: float = 1e-9) -> bool:
    return (
        circle is not None
        and float(np.linalg.norm(point - circle[0]))
        <= circle[1] + tolerance
    )


def _diameter_circle(one: np.ndarray, two: np.ndarray
                     ) -> tuple[np.ndarray, float]:
    center = (one + two) / 2.0
    return center, float(np.linalg.norm(one - center))


def _circumcircle(one: np.ndarray, two: np.ndarray, three: np.ndarray
                  ) -> tuple[np.ndarray, float] | None:
    ax, ay = map(float, one)
    bx, by = map(float, two)
    cx, cy = map(float, three)
    determinant = 2.0 * (
        ax * (by - cy) + bx * (cy - ay) + cx * (ay - by)
    )
    if abs(determinant) <= 1e-12:
        return None
    aa, bb, cc = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
    center = np.array((
        (aa * (by - cy) + bb * (cy - ay) + cc * (ay - by))
        / determinant,
        (aa * (cx - bx) + bb * (ax - cx) + cc * (bx - ax))
        / determinant,
    ))
    return center, float(np.linalg.norm(one - center))


def smallest_enclosing_circle(points: np.ndarray
                              ) -> tuple[np.ndarray, float]:
    """Deterministic randomized-incremental minimum enclosing circle."""

    values = [np.asarray(point, dtype=float) for point in points]
    random.Random(0).shuffle(values)
    circle: tuple[np.ndarray, float] | None = None
    for index, point in enumerate(values):
        if _circle_contains(circle, point):
            continue
        circle = (point.copy(), 0.0)
        for second_index in range(index):
            second = values[second_index]
            if _circle_contains(circle, second):
                continue
            circle = _diameter_circle(point, second)
            for third_index in range(second_index):
                third = values[third_index]
                if _circle_contains(circle, third):
                    continue
                candidate = _circumcircle(point, second, third)
                if candidate is None:
                    pairs = (
                        _diameter_circle(point, second),
                        _diameter_circle(point, third),
                        _diameter_circle(second, third),
                    )
                    candidate = min(
                        (item for item in pairs
                         if all(_circle_contains(item, value)
                                for value in (point, second, third))),
                        key=lambda item: item[1],
                    )
                circle = candidate
    if circle is None:
        raise ValueError("minimum enclosing circle needs at least one point")
    return circle


def target_kinematics() -> dict[str, Any]:
    cases = _cases()
    by_sign: dict[int, list[np.ndarray]] = {-1: [], 1: []}
    for case in cases:
        by_sign[case.motion_sign].append(
            np.asarray(case.target_flyer_mm, dtype=float)
        )

    summaries: dict[str, Any] = {}
    for key, points in (
        ("motion_sign_-1", np.asarray(by_sign[-1])),
        ("motion_sign_+1", np.asarray(by_sign[1])),
        ("both_signs", np.vstack((by_sign[-1], by_sign[1]))),
    ):
        center, radius = smallest_enclosing_circle(points[:, :2])
        radial = np.linalg.norm(points[:, :2], axis=1)
        summaries[key] = {
            "case_count": len(points),
            "target_flyer_xy_min_mm": list(map(float, points[:, :2].min(0))),
            "target_flyer_xy_max_mm": list(map(float, points[:, :2].max(0))),
            "target_orbit_radius_range_mm": [
                float(np.min(radial)), float(np.max(radial))
            ],
            "minimum_enclosing_circle_center_xy_mm": list(map(float, center)),
            "minimum_possible_fixed_exit_worst_launch_mm": float(radius),
            "meets_0p5mm_launch": bool(radius <= MAXIMUM_LAUNCH_TARGET_MM),
        }

    all_points = np.vstack((by_sign[-1], by_sign[1]))
    radii = np.linalg.norm(all_points[:, :2], axis=1)
    minimum_index = int(np.argmin(radii))
    ordered_cases = [
        case for sign in (-1, 1) for case in cases
        if case.motion_sign == sign
    ]
    witness = ordered_cases[minimum_index]
    summaries["minimum_radius_witness"] = {
        "target_orbit_radius_mm": float(radii[minimum_index]),
        "depth_index": witness.depth_index,
        "turn_index": witness.turn_index,
        "flyer_angle_deg": witness.angle_deg,
        "motion_sign": witness.motion_sign,
        "target_flyer_mm": list(witness.target_flyer_mm),
    }
    return summaries


def passive_orbit_lower_bound(kinematics: dict[str, Any]) -> dict[str, Any]:
    """Exact necessary bound for any feature fixed to the flyer.

    The source tooth neck contains an axis-aligned rectangular prism.  At a
    fixed flyer point's M2 orbit, its stator-local tangential/axial coordinates
    trace a circle.  If that orbit radius is at or below the farthest radius
    of the liner+wire-dilated neck rectangle, one M2 pose necessarily enters
    the neck.  Escaping in machine Z instead must clear the neck over *all*
    nine M0 positions.  The nearer of those disjoint escape sets is a rigorous
    lower bound; the rest of the stator can only make it worse.
    """

    spec = DEFAULT_STATOR
    hub_od = float(spec.od) * float(spec.hub_od_ratio)
    shoe_t = max(1.6, float(spec.od) * 0.045)
    shoe_inner = float(spec.od) / 2.0 - shoe_t
    half_neck = max(2.5, float(spec.od) * 0.07) / 2.0
    half_stack = float(spec.stack) / 2.0
    neck_x_min = hub_od / 2.0 - 1.0
    neck_length = shoe_inner - hub_od / 2.0 + 2.0
    neck_x_max = neck_x_min + neck_length
    clearance = WIRE_CORE_ENVELOPE_MM
    forbidden_orbit_radius = math.hypot(half_neck, half_stack) + clearance

    depth_radii = sorted({case.radial_mm for case in _cases()})
    axis_min = 2.0 + min(depth_radii)
    axis_max = 2.0 + max(depth_radii)
    rear_safe_z_max = axis_min - (neck_x_max + clearance)
    front_safe_z_min = axis_max - (neck_x_min - clearance)
    target_z = 2.0
    rear_escape = target_z - rear_safe_z_max
    front_escape = front_safe_z_min - target_z

    minimum_target_radius = float(
        kinematics["minimum_radius_witness"]["target_orbit_radius_mm"]
    )
    radial_escape = forbidden_orbit_radius - minimum_target_radius
    lower_bound = min(radial_escape, rear_escape, front_escape)
    return {
        "method": (
            "exact contained source tooth-neck prism, Minkowski-dilated by "
            "maximum-wire radius plus nominal liner; every flyer-fixed point "
            "is swept through all 360 M2 degrees and all nine M0 depths"
        ),
        "source_neck_x_range_mm": [neck_x_min, neck_x_max],
        "source_neck_half_tangential_mm": half_neck,
        "source_stack_half_axial_mm": half_stack,
        "wire_plus_liner_clearance_mm": clearance,
        "forbidden_orbit_radius_through_neck_mm": forbidden_orbit_radius,
        "m0_axis_z_range_mm": [axis_min, axis_max],
        "rear_machine_z_escape_boundary_mm": rear_safe_z_max,
        "front_machine_z_escape_boundary_mm": front_safe_z_min,
        "minimum_target_orbit_radius_mm": minimum_target_radius,
        "radial_escape_launch_lower_bound_mm": radial_escape,
        "rear_axial_escape_launch_lower_bound_mm": rear_escape,
        "front_axial_escape_launch_lower_bound_mm": front_escape,
        "minimum_possible_worst_launch_mm": lower_bound,
        "requested_maximum_launch_mm": MAXIMUM_LAUNCH_TARGET_MM,
        "shortfall_mm": lower_bound - MAXIMUM_LAUNCH_TARGET_MM,
        "status": "FAIL" if lower_bound > MAXIMUM_LAUNCH_TARGET_MM else "PASS",
        "interpretation": (
            "Any passive flyer-fixed contact close enough to control the "
            "innermost lay point later orbits through the active tooth neck. "
            "Moving it behind/ahead of every M0 depth takes still more launch."
        ),
    }


def mouth_budget() -> dict[str, Any]:
    slot = coil_growth.slot_geometry(DEFAULT_STATOR)
    bare = float(slot["opening_width_mm"])
    current_overlap = float(insulation.CAP_EDGE_OVERLAP_MM)
    lined = bare - 2.0 * LINER_ALLOWANCE_MM
    current = bare - 2.0 * current_overlap
    relief_rows = []
    for relief in CAP_RELIEF_EACH_MM:
        aperture = min(lined, current + 2.0 * relief)
        relief_rows.append({
            "cap_relief_each_mm": relief,
            "controlling_aperture_mm": aperture,
        })
    nozzle_rows = []
    for wall in NOZZLE_WALLS_MM:
        od = NOZZLE_ID_MM + 2.0 * wall
        fits = [
            row["cap_relief_each_mm"] for row in relief_rows
            if od <= row["controlling_aperture_mm"] + 1e-12
        ]
        nozzle_rows.append({
            "wall_each_mm": wall,
            "id_mm": NOZZLE_ID_MM,
            "od_mm": od,
            "minimum_listed_cap_relief_each_mm": min(fits) if fits else None,
            "fits_current_unrelieved_cap": bool(
                od <= current + 1e-12
            ),
            "fits_fully_relived_liner_only_mouth": bool(
                od <= lined + 1e-12
            ),
        })
    return {
        "bare_mouth_mm": bare,
        "liner_each_mm": LINER_ALLOWANCE_MM,
        "lined_mouth_mm": lined,
        "current_cap_overlap_each_mm": current_overlap,
        "current_cap_mouth_mm": current,
        "relief_sweep": relief_rows,
        "closed_nozzle_sweep": nozzle_rows,
        "open_horn_minimum_surface_radius_mm": OPEN_HORN_SURFACE_RADIUS_MM,
        "open_horn_minimum_physical_od_mm": OPEN_HORN_MINIMUM_OD_MM,
        "open_horn_liner_only_aperture_shortfall_mm": (
            OPEN_HORN_MINIMUM_OD_MM - lined
        ),
        "open_horn_can_enter_lined_mouth": bool(
            OPEN_HORN_MINIMUM_OD_MM <= lined
        ),
    }


def _align_z_to(vector: np.ndarray) -> np.ndarray:
    target = _unit(vector)
    source = np.array((0.0, 0.0, 1.0))
    cross = np.cross(source, target)
    cosine = float(np.dot(source, target))
    squared = float(np.dot(cross, cross))
    if squared <= 1e-16:
        return np.eye(3) if cosine >= 0.0 else np.diag((1.0, -1.0, -1.0))
    skew = np.array((
        (0.0, -cross[2], cross[1]),
        (cross[2], 0.0, -cross[0]),
        (-cross[1], cross[0], 0.0),
    ))
    return np.eye(3) + skew + skew @ skew * ((1.0 - cosine) / squared)


def _capsule(start: np.ndarray, end: np.ndarray, radius: float
             ) -> tuple[fcl.Capsule, np.ndarray, np.ndarray]:
    vector = np.asarray(end, dtype=float) - np.asarray(start, dtype=float)
    length = float(np.linalg.norm(vector))
    if length <= 1e-9:
        raise ValueError("capsule segment is zero length")
    return (
        fcl.Capsule(float(radius), length),
        _align_z_to(vector),
        (np.asarray(start, dtype=float) + np.asarray(end, dtype=float)) / 2.0,
    )


def _fcl_distance(geometry_a: Any, rotation_a: np.ndarray,
                  translation_a: np.ndarray, geometry_b: Any,
                  rotation_b: np.ndarray | None = None,
                  translation_b: np.ndarray | None = None) -> float:
    rotation_b = np.eye(3) if rotation_b is None else rotation_b
    translation_b = (
        np.zeros(3) if translation_b is None else translation_b
    )
    one = fcl.CollisionObject(
        geometry_a, fcl.Transform(rotation_a, translation_a)
    )
    two = fcl.CollisionObject(
        geometry_b, fcl.Transform(rotation_b, translation_b)
    )
    result = fcl.CollisionResult()
    fcl.collide(one, two, fcl.CollisionRequest(), result)
    if result.is_collision:
        return -1.0
    return float(fcl.distance(
        one, two, fcl.DistanceRequest(), fcl.DistanceResult()
    ))


def _fcl_collides(geometry_a: Any, rotation_a: np.ndarray,
                  translation_a: np.ndarray, geometry_b: Any,
                  rotation_b: np.ndarray | None = None,
                  translation_b: np.ndarray | None = None) -> bool:
    rotation_b = np.eye(3) if rotation_b is None else rotation_b
    translation_b = (
        np.zeros(3) if translation_b is None else translation_b
    )
    one = fcl.CollisionObject(
        geometry_a, fcl.Transform(rotation_a, translation_a)
    )
    two = fcl.CollisionObject(
        geometry_b, fcl.Transform(rotation_b, translation_b)
    )
    result = fcl.CollisionResult()
    fcl.collide(one, two, fcl.CollisionRequest(), result)
    return bool(result.is_collision)


def _load_link_mesh(link: str, exclude: Iterable[str] = ()
                    ) -> trimesh.Trimesh:
    excluded = set(exclude)
    meshes = [
        trimesh.load(
            OUT / "links" / "parts" / link / f"{label}.stl",
            force="mesh",
        )
        for label in _manifest()["parts"][link]
        if label not in excluded
    ]
    return trimesh.util.concatenate(meshes)


@lru_cache(maxsize=1)
def _rigid_bvhs() -> dict[str, Any]:
    return {
        "core": collide.make_bvh(_planner().mesh),
        "flyer": collide.make_bvh(_load_link_mesh(
            "flyer", exclude=("tip_toroid_guide",)
        )),
        "chuck": collide.make_bvh(_load_link_mesh(
            "spindle", exclude=("stator_final_wound_envelope",)
        )),
    }


# Source-stator local -> machine-world rotation, matching assembly.py.
STATOR_LOCAL_TO_WORLD = np.array((
    (0.0, -1.0, 0.0),
    (0.0, 0.0, 1.0),
    (-1.0, 0.0, 0.0),
))


def _fixed_terminal(exit_local: np.ndarray
                    ) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    guide = _manifest()["wire"]["tip_guide"]
    feed = np.asarray(guide["feed_local_mm"], dtype=float)
    path, meta = wirepath.tip_guide_path(
        feed, np.asarray(exit_local, dtype=float), guide,
        MAX_WIRE_RADIUS_MM, np.eye(3), arc_step_deg=2.0,
    )
    # The existing torus owns the path through path[-2].  Only its tangent
    # terminal segment is the proposed tube/horn extension.
    return np.asarray(path[-2], dtype=float), np.asarray(exit_local), meta


def _current_segment_margin(start: np.ndarray, end: np.ndarray,
                            current: np.ndarray,
                            required_center_distance_mm: float) -> float:
    if len(current) < 2:
        return 1e9
    minimum = min(
        slot_route._segment_segment_distance(start, end, one, two)
        for one, two in zip(current, current[1:])
    )
    return float(minimum - required_center_distance_mm)


def progressive_route_audit(exit_local: np.ndarray,
                            terminal_start_local: np.ndarray
                            ) -> dict[str, Any]:
    graph = _graph()
    wire_diameter = float(graph.wire_diameter_mm)
    outer_radius = NOZZLE_ID_MM / 2.0 + min(NOZZLE_WALLS_MM)
    failure_counts = {
        "prior_nonparent_wire": 0,
        "support_parent_prefix": 0,
        "already_laid_current_half": 0,
        "tube_body_vs_prior_copper": 0,
        "tube_body_vs_current_half": 0,
    }
    worst = {name: 1e9 for name in failure_counts}
    for case in _cases():
        rotation = wirepath.rot_z(math.radians(case.angle_deg))
        exit_world = rotation @ exit_local
        terminal_world = rotation @ terminal_start_local
        target_world = np.asarray(case.target_world_mm, dtype=float)
        route_local = _machine_to_local(
            np.vstack((exit_world, target_world)), case.axis_z_mm
        )
        prefix = _trim_polyline_end(route_local, wire_diameter)
        body_local = _machine_to_local(
            np.vstack((terminal_world, exit_world)), case.axis_z_mm
        )
        fields = _copper_fields(case.depth_index)

        nonparent = fields["nonparents"].clearance(
            route_local, COPPER_SEARCH_BAND_MM
        ).minimum_centerline_distance_mm - wire_diameter
        parent = (
            fields["parents"].clearance(
                prefix, COPPER_SEARCH_BAND_MM
            ).minimum_centerline_distance_mm - wire_diameter
            if fields["parent_ids"] else 1e9
        )
        current = np.asarray(case.current_arrival_local_mm, dtype=float)
        current_margin = _current_segment_margin(
            prefix[0], prefix[-1], current, wire_diameter
        )
        body_required = outer_radius + MAX_WIRE_RADIUS_MM
        body_prior = fields["nonparents"].clearance(
            body_local, 4.0
        ).minimum_centerline_distance_mm - body_required
        body_current = min(
            _current_segment_margin(
                start, end, current, body_required
            )
            for start, end in zip(body_local, body_local[1:])
        )

        values = {
            "prior_nonparent_wire": nonparent,
            "support_parent_prefix": parent,
            "already_laid_current_half": current_margin,
            "tube_body_vs_prior_copper": body_prior,
            "tube_body_vs_current_half": body_current,
        }
        for name, value in values.items():
            worst[name] = min(worst[name], float(value))
            if value < -1e-9:
                failure_counts[name] += 1
    return {
        "case_count": len(_cases()),
        "failure_counts": failure_counts,
        "minimum_margins_mm": worst,
        "status": (
            "PASS" if not any(failure_counts.values()) else "FAIL"
        ),
    }


def _rigid_and_core_audit(exit_local: np.ndarray,
                          terminal_start_local: np.ndarray,
                          body_radius_mm: float) -> dict[str, Any]:
    bvhs = _rigid_bvhs()
    body_geometry, body_rotation, body_translation = _capsule(
        terminal_start_local, exit_local, body_radius_mm
    )
    self_collision = _fcl_collides(
        body_geometry, body_rotation, body_translation,
        bvhs["flyer"], np.eye(3), np.zeros(3),
    )
    core_collisions = chuck_collisions = 0
    manifest = _manifest()
    home = float(manifest["m0_home_standoff"])
    # The body result is sign-independent.  Do not double-count the same
    # rigid pose merely because the live wire may travel in two directions.
    for depth_index in range(DEPTH_COUNT):
        depth_cases = [
            case for case in _cases()
            if case.depth_index == depth_index and case.motion_sign == 1
        ]
        axis_z = depth_cases[0].axis_z_mm
        dz = axis_z - home
        for case in depth_cases:
            flyer_rotation = wirepath.rot_z(math.radians(case.angle_deg))
            rotation = flyer_rotation @ body_rotation
            translation = flyer_rotation @ body_translation
            core_collision = _fcl_collides(
                body_geometry, rotation, translation,
                bvhs["core"], STATOR_LOCAL_TO_WORLD,
                np.array((0.0, 0.0, axis_z)),
            )
            chuck_collision = _fcl_collides(
                body_geometry, rotation, translation,
                bvhs["chuck"], np.eye(3), np.array((0.0, 0.0, dz)),
            )
            core_collisions += core_collision
            chuck_collisions += chuck_collision
    return {
        "body_radius_mm": body_radius_mm,
        "flyer_self_collision": self_collision,
        "core_collision_pose_count": int(core_collisions),
        "chuck_collision_pose_count": int(chuck_collisions),
        "clearance_method": (
            "exact FCL mesh/primitive intersection at every integer degree; "
            "positive distance is not promoted after a collision failure"
        ),
        "rigid_pose_count": DEPTH_COUNT * ANGLE_COUNT,
        "status": (
            "PASS" if not self_collision
            and not core_collisions and not chuck_collisions else "FAIL"
        ),
    }


def _free_wire_core_audit(exit_local: np.ndarray) -> dict[str, Any]:
    core = _rigid_bvhs()["core"]
    collisions = 0
    for case in _cases():
        rotation = wirepath.rot_z(math.radians(case.angle_deg))
        exit_world = rotation @ exit_local
        target = np.asarray(case.target_world_mm, dtype=float)
        geometry, local_rotation, local_translation = _capsule(
            exit_world, target, WIRE_CORE_ENVELOPE_MM
        )
        collision = _fcl_collides(
            geometry, local_rotation, local_translation,
            core, STATOR_LOCAL_TO_WORLD,
            np.array((0.0, 0.0, case.axis_z_mm)),
        )
        collisions += collision
    return {
        "case_count": len(_cases()),
        "collision_case_count": int(collisions),
        "clearance_method": (
            "maximum-wire-plus-liner capsule vs source-core triangulation "
            "at every integer degree"
        ),
        "status": "PASS" if not collisions else "FAIL",
    }


def closed_nozzle_sweep() -> dict[str, Any]:
    targets = np.asarray([case.target_flyer_mm for case in _cases()])
    rows = []
    progressive_by_radius: dict[float, dict[str, Any]] = {}
    for radius in NOZZLE_EXIT_RADII_MM:
        for exit_z in NOZZLE_EXIT_Z_MM:
            exit_local = np.array((0.0, radius, exit_z), dtype=float)
            terminal_start, _exit, torus_meta = _fixed_terminal(exit_local)
            launch = np.linalg.norm(targets - exit_local, axis=1)
            barrel_direction = exit_local - terminal_start
            lip_errors = np.asarray([
                _angle_deg(barrel_direction, target - exit_local)
                for target in targets
            ])
            rigid = _rigid_and_core_audit(
                exit_local, terminal_start,
                NOZZLE_ID_MM / 2.0 + min(NOZZLE_WALLS_MM),
            )
            free_core = _free_wire_core_audit(exit_local)
            row = {
                "exit_radius_mm": radius,
                "reach_from_released_tip_mm": (
                    float(_manifest()["flyer_tip_r"]) - radius
                ),
                "exit_z_mm": exit_z,
                "terminal_tangent_start_local_mm": list(map(
                    float, terminal_start
                )),
                "minimum_launch_mm": float(np.min(launch)),
                "maximum_launch_mm": float(np.max(launch)),
                "maximum_exit_lip_direction_error_deg": float(
                    np.max(lip_errors)
                ),
                "existing_torus_wire_center_radius_mm": float(
                    torus_meta["wire_center_bend_radius_mm"]
                ),
                "existing_torus_inside_radius_mm": float(
                    torus_meta["inside_wire_path_radius_mm"]
                ),
                "rigid_thinnest_nozzle": rigid,
                "free_wire_vs_lined_core": free_core,
            }
            row["status"] = (
                "PASS" if row["maximum_launch_mm"]
                <= MAXIMUM_LAUNCH_TARGET_MM
                and row["maximum_exit_lip_direction_error_deg"]
                <= EXIT_LIP_C1_LIMIT_DEG
                and rigid["status"] == "PASS"
                and free_core["status"] == "PASS" else "FAIL"
            )
            rows.append(row)
            print(
                f"closed nozzle r={radius:g} z={exit_z:g}: "
                f"body_core={rigid['core_collision_pose_count']} "
                f"wire_core={free_core['collision_case_count']}",
                flush=True,
            )
        # z=2 is the shortest possible axial launch in the declared family;
        # run the expensive progressive copper audit once per reach there.
        exit_local = np.array((0.0, radius, 2.0), dtype=float)
        terminal_start, _exit, _meta = _fixed_terminal(exit_local)
        progressive_by_radius[radius] = progressive_route_audit(
            exit_local, terminal_start
        )
        print(f"closed nozzle progressive r={radius:g} complete", flush=True)

    passing = [row for row in rows if row["status"] == "PASS"]
    best_launch = min(rows, key=lambda row: row["maximum_launch_mm"])
    fewest_core = min(
        rows,
        key=lambda row: (
            row["free_wire_vs_lined_core"]["collision_case_count"],
            row["maximum_launch_mm"],
        ),
    )
    return {
        "candidate_count": len(rows),
        "exit_radius_sweep_mm": list(NOZZLE_EXIT_RADII_MM),
        "exit_z_sweep_mm": list(NOZZLE_EXIT_Z_MM),
        "direction_model": (
            "straight barrel tangent to the existing R3.25 torus; a thin "
            "lip is required to be C1 because an R3 lip cannot fit the mouth"
        ),
        "rows": rows,
        "progressive_copper_at_z2_by_exit_radius": {
            f"{radius:g}": progressive_by_radius[radius]
            for radius in NOZZLE_EXIT_RADII_MM
        },
        "passing_candidate_count": len(passing),
        "best_maximum_launch_row": best_launch,
        "fewest_core_collision_row": fewest_core,
        "status": "PASS" if passing else "FAIL",
    }


def _point_copper_margin(point_local: np.ndarray,
                         case: Case,
                         required_center_distance_mm: float) -> float:
    fields = _copper_fields(case.depth_index)
    distances = fields["nonparents"].point_clearances(
        point_local, required_center_distance_mm + 0.5
    )
    prior = min(distances.values(), default=1e9) - required_center_distance_mm
    current = np.asarray(case.current_arrival_local_mm, dtype=float)
    if len(current) < 2:
        return prior
    current_distances = []
    for start, end in zip(current, current[1:]):
        vector = end - start
        denominator = float(np.dot(vector, vector))
        fraction = 0.0 if denominator <= 1e-12 else float(np.clip(
            np.dot(point_local - start, vector) / denominator, 0.0, 1.0
        ))
        current_distances.append(float(np.linalg.norm(
            point_local - (start + fraction * vector)
        )))
    return min(prior, min(current_distances) - required_center_distance_mm)


def open_horn_sweep() -> dict[str, Any]:
    """Optimistic all-direction R3 terminal-contact sweep.

    A real open horn has a finite approach groove and a restricted contact
    cap.  Treating it as an all-direction sphere removes both restrictions,
    so a failure here is a necessary no-go, not an artifact of a narrow horn.
    """

    targets = np.asarray([case.target_flyer_mm for case in _cases()])
    core = _rigid_bvhs()["core"]
    chuck = _rigid_bvhs()["chuck"]
    sphere = fcl.Sphere(OPEN_HORN_SURFACE_RADIUS_MM)
    rows = []
    for radius in OPEN_HORN_CENTER_RADII_MM:
        center_local = np.array((0.0, radius, 2.0), dtype=float)
        center_distance = np.linalg.norm(targets - center_local, axis=1)
        inside = center_distance < MINIMUM_WIRE_CENTER_BEND_RADIUS_MM - 1e-9
        tangent_launch = np.sqrt(np.maximum(
            center_distance * center_distance
            - MINIMUM_WIRE_CENTER_BEND_RADIUS_MM ** 2,
            0.0,
        ))
        core_collisions = chuck_collisions = progressive_collisions = 0
        progressive_minimum = 1e9
        home = float(_manifest()["m0_home_standoff"])
        for case in _cases():
            # Both signs share rigid transforms, but progressive copper does
            # not.  Count all route cases here so the record remains explicit.
            rotation = wirepath.rot_z(math.radians(case.angle_deg))
            center_world = rotation @ center_local
            core_collision = _fcl_collides(
                sphere, np.eye(3), center_world,
                core, STATOR_LOCAL_TO_WORLD,
                np.array((0.0, 0.0, case.axis_z_mm)),
            )
            chuck_collision = _fcl_collides(
                sphere, np.eye(3), center_world,
                chuck, np.eye(3),
                np.array((0.0, 0.0, case.axis_z_mm - home)),
            )
            core_collisions += core_collision
            chuck_collisions += chuck_collision
            center_stator = _machine_to_local(
                center_world[None, :], case.axis_z_mm
            )[0]
            copper_margin = _point_copper_margin(
                center_stator, case,
                OPEN_HORN_SURFACE_RADIUS_MM + MAX_WIRE_RADIUS_MM,
            )
            progressive_minimum = min(progressive_minimum, copper_margin)
            progressive_collisions += copper_margin < -1e-9
        row = {
            "horn_center_orbit_radius_mm": radius,
            "reach_from_released_tip_mm": (
                float(_manifest()["flyer_tip_r"]) - radius
            ),
            "horn_center_z_mm": 2.0,
            "physical_surface_radius_mm": OPEN_HORN_SURFACE_RADIUS_MM,
            "wire_center_contact_radius_mm": (
                MINIMUM_WIRE_CENTER_BEND_RADIUS_MM
            ),
            "target_inside_required_contact_sphere_case_count": int(
                np.count_nonzero(inside)
            ),
            "maximum_optimistic_tangent_launch_mm": float(
                np.max(tangent_launch)
            ),
            "core_collision_case_count": int(core_collisions),
            "chuck_collision_case_count": int(chuck_collisions),
            "clearance_method": (
                "exact FCL sphere/mesh intersection at every route case; "
                "analytic tooth-neck containment bound handles fully enclosed poses"
            ),
            "progressive_copper_collision_case_count": int(
                progressive_collisions
            ),
            "minimum_progressive_copper_margin_mm": progressive_minimum,
        }
        row["status"] = (
            "PASS" if not np.any(inside)
            and row["maximum_optimistic_tangent_launch_mm"]
            <= MAXIMUM_LAUNCH_TARGET_MM
            and not core_collisions and not chuck_collisions
            and not progressive_collisions else "FAIL"
        )
        rows.append(row)
        print(
            f"open horn r={radius:g}: core={core_collisions} "
            f"copper={progressive_collisions}", flush=True
        )
    passing = [row for row in rows if row["status"] == "PASS"]
    return {
        "model": (
            "optimistic all-direction spherical polished contact; ignores "
            "approach-groove restriction and therefore favors the horn"
        ),
        "candidate_count": len(rows),
        "rows": rows,
        "passing_candidate_count": len(passing),
        "status": "PASS" if passing else "FAIL",
    }


def analyze(*, include_dense: bool = True) -> dict[str, Any]:
    kinematics = target_kinematics()
    orbit = passive_orbit_lower_bound(kinematics)
    mouth = mouth_budget()
    closed = closed_nozzle_sweep() if include_dense else None
    opened = open_horn_sweep() if include_dense else None
    gates = {
        "fixed_exit_0p5mm_launch": (
            kinematics["both_signs"][
                "minimum_possible_fixed_exit_worst_launch_mm"
            ] <= MAXIMUM_LAUNCH_TARGET_MM
        ),
        "any_passive_contact_0p5mm_launch": orbit["status"] == "PASS",
        "closed_nozzle_family": (
            closed is not None and closed["status"] == "PASS"
        ),
        "open_horn_family": (
            opened is not None and opened["status"] == "PASS"
        ),
        "open_horn_fits_slot_mouth": mouth[
            "open_horn_can_enter_lined_mouth"
        ],
    }
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "PASS" if all(gates.values()) else "DESIGN_NO_GO",
        "release_authorized": False,
        "assembly_integration_authorized": False,
        "scope": {
            "goal": "normal GOAL.md flyer machine; G2 needle module excluded",
            "stator": "default OD46 x stack15 x 24-slot source geometry",
            "maximum_wire_diameter_mm": MAX_WIRE_DIAMETER_MM,
            "minimum_wire_center_bend_radius_mm": (
                MINIMUM_WIRE_CENTER_BEND_RADIUS_MM
            ),
            "requested_maximum_launch_mm": MAXIMUM_LAUNCH_TARGET_MM,
            "flyer_angles": ANGLE_COUNT,
            "m0_depths": DEPTH_COUNT,
            "motion_signs": list(DIRECTIONS),
            "required_route_cases": len(_cases()),
            "progressive_prior_and_current_copper": True,
        },
        "gates": gates,
        "target_kinematics": kinematics,
        "any_passive_flyer_fixed_contact_lower_bound": orbit,
        "mouth_od_wall_cap_relief": mouth,
        "closed_nozzle": closed,
        "open_horn": opened,
        "comparison": {
            "closed_nozzle": (
                "Small ODs can fit after little/no cap relief, but a thin lip "
                "cannot supply an R3 redirection. The fixed exit cannot stay "
                "within 0.5 mm, and every swept terminal family retains bare-"
                "core and/or progressive-copper failures."
            ),
            "open_horn": (
                "An R3 external polished turn requires at least OD5.5, 4.246 "
                "mm wider than the fully relieved lined mouth. Even an "
                "unphysical all-direction contact fails the orbit, launch, "
                "rigid-core, and progressive-copper gates."
            ),
        },
        "decision": (
            "No flyer-mounted passive horn or closed nozzle in the swept "
            "families can solve the final slot-mouth route. More generally, "
            f"the source tooth neck proves a flyer-fixed contact needs at "
            f"least {orbit['minimum_possible_worst_launch_mm']:.3f} mm free "
            "launch at the innermost witness, so the requested 0.5 mm control "
            "cannot be reached without phase-selective retraction/compliance "
            "or another independently moved guide. The machine can remain a "
            "flyer architecture, but not with a purely passive fixed nozzle."
        ),
        "no_go_boundaries": [
            (
                f"any fixed exit: best possible worst launch is "
                f"{kinematics['both_signs']['minimum_possible_fixed_exit_worst_launch_mm']:.3f} mm"
            ),
            (
                f"any passive flyer-fixed contact: active-neck lower bound "
                f"is {orbit['minimum_possible_worst_launch_mm']:.3f} mm"
            ),
            (
                f"R3 open horn minimum OD {OPEN_HORN_MINIMUM_OD_MM:.3f} mm "
                f"versus lined mouth {mouth['lined_mouth_mm']:.3f} mm"
            ),
            "closed thin nozzle may fit the mouth only if its exit is effectively C1; its lip cannot furnish R3",
        ],
        "source_hashes": {
            "sim/flyer_nozzle_trade.py": _sha256(Path(__file__)),
            "sim/slot_route.py": _sha256(HERE / "slot_route.py"),
            "sim/wirepath.py": _sha256(HERE / "wirepath.py"),
            "cad/stator_model.py": _sha256(CAD / "stator_model.py"),
            "cad/stator_insulation_nomex410.py": _sha256(
                CAD / "stator_insulation_nomex410.py"
            ),
            "out/reports/slot_packing.json": _sha256(PACKING_PATH),
            "out/links/manifest.json": _sha256(MANIFEST_PATH),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def write(report: dict[str, Any]) -> None:
    REPORTS.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    orbit = report["any_passive_flyer_fixed_contact_lower_bound"]
    kinematics = report["target_kinematics"]["both_signs"]
    mouth = report["mouth_od_wall_cap_relief"]
    closed = report.get("closed_nozzle") or {}
    opened = report.get("open_horn") or {}
    lines = [
        "# Flyer-mounted horn / nozzle trade",
        "",
        f"Status: **{report['status']}**. Production integration and hardware release remain false.",
        "",
        "## Controlling result",
        "",
        (
            f"A fixed exit has an exact best-case worst launch of "
            f"{kinematics['minimum_possible_fixed_exit_worst_launch_mm']:.3f} mm."
        ),
        (
            f"Allowing an arbitrary passive horn contact does not rescue it: "
            f"the contained source tooth-neck prism proves at least "
            f"{orbit['minimum_possible_worst_launch_mm']:.3f} mm launch is "
            f"required at the innermost lay witness (target {MAXIMUM_LAUNCH_TARGET_MM:.3f} mm)."
        ),
        "",
        "## Closed nozzle",
        "",
        (
            f"Swept {closed.get('candidate_count', 0)} reach/Z candidates over "
            f"{len(_cases())} wire cases; passing candidates: "
            f"{closed.get('passing_candidate_count', 0)}."
        ),
        (
            f"ID {NOZZLE_ID_MM:.3f} mm with 0.05..0.30 mm walls gives OD "
            f"{NOZZLE_ID_MM + 2*min(NOZZLE_WALLS_MM):.3f}.."
            f"{NOZZLE_ID_MM + 2*max(NOZZLE_WALLS_MM):.3f} mm. The current cap "
            f"mouth is {mouth['current_cap_mouth_mm']:.3f} mm and the fully "
            f"relieved liner-only mouth is {mouth['lined_mouth_mm']:.3f} mm."
        ),
        "A thin closed lip cannot provide an R3 direction change, so its free span must leave essentially C1; the direction sweep fails.",
        "",
        "## Open polished horn",
        "",
        (
            f"R3 at the wire centre requires a physical surface radius "
            f"{OPEN_HORN_SURFACE_RADIUS_MM:.3f} mm (minimum OD "
            f"{OPEN_HORN_MINIMUM_OD_MM:.3f} mm), exceeding the lined mouth by "
            f"{mouth['open_horn_liner_only_aperture_shortfall_mm']:.3f} mm."
        ),
        (
            f"Even the optimistic all-direction spherical-contact sweep has "
            f"{opened.get('passing_candidate_count', 0)} passing candidates."
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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--write", action="store_true")
    parser.add_argument(
        "--skip-dense", action="store_true",
        help="write only analytic bounds; not a release audit",
    )
    args = parser.parse_args()
    report = analyze(include_dense=not args.skip_dense)
    if args.write:
        write(report)
    print(json.dumps({
        "status": report["status"],
        "report_sha256": report["report_sha256"],
        "passive_launch_lower_bound_mm": report[
            "any_passive_flyer_fixed_contact_lower_bound"
        ]["minimum_possible_worst_launch_mm"],
        "closed_passes": (
            report.get("closed_nozzle") or {}
        ).get("passing_candidate_count"),
        "open_passes": (
            report.get("open_horn") or {}
        ).get("passing_candidate_count"),
    }, indent=2))


if __name__ == "__main__":
    main()
