"""Fail-closed route and motion audit for ``cad/active_tooth_shoe.py``.

The audit is intentionally isolated from the production route modules.  It
evaluates the proposed machine-fixed split shoe without changing collision
exclusions or assembly membership.

Required sweep:

* 360 integer flyer angles;
* nine M0 lay depths;
* both winding directions;
* exact source-core clearance, nominal installed liner allowance, the shoe
  itself, all prior non-parent copper, declared support parents, and the
  already-arrived portion of the current loop;
* insertion/extraction and rigid flyer/chuck/indexing motion.

The source stator BREP is authoritative.  Dense route sweeps use its
watertight 0.01/0.03 tessellation with <=0.10 mm centreline spacing; every
rigid insertion witness is independently intersected as BREP common volume.
Because the candidate fails, this module never promotes that mesh sweep into
an exact release proof.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from functools import lru_cache
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import fcl
import numpy as np
import trimesh


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
OUT = ROOT / "out"
REPORTS = OUT / "reports"
REVIEW = OUT / "review"
MANIFEST_PATH = OUT / "links" / "manifest.json"
PACKING_PATH = REPORTS / "slot_packing.json"
JSON_OUT = REPORTS / "active_tooth_shoe.json"
MD_OUT = REPORTS / "active_tooth_shoe.md"

for path in (CAD, HERE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import active_tooth_shoe as shoe  # noqa: E402
import collide  # noqa: E402
from params import DEFAULT_STATOR  # noqa: E402
import slot_route  # noqa: E402
import stator_insulation_nomex410 as insulation  # noqa: E402
import stator_model  # noqa: E402
import wirepath  # noqa: E402


SCHEMA = "active-tooth-shoe-audit/v1"
ANGLE_COUNT = 360
DEPTH_COUNT = 9
DIRECTIONS = (-1, 1)
ROUTE_SAMPLE_SPACING_MM = 0.10
EXTRACTION_SAMPLE_SPACING_MM = 0.05
HORN_ARC_STEP_DEG = 2.0
TANGENCY_LIMIT_DEG = 0.5
# ``tip_guide_path`` is analytically tangent.  Its reported error compares a
# true tangent with the last 2-degree chord, whose expected half-step error is
# one degree.  This bound accepts that documented chord error only.
TORUS_CHORD_TANGENCY_LIMIT_DEG = 1.01
RIGID_CLEARANCE_TARGET_MM = 2.0


@dataclass(frozen=True)
class RouteCase:
    depth_index: int
    turn_index: int
    flyer_angle_deg: int
    motion_sign: int
    route_ok: bool
    selected_side: int
    selected_axial_sign: int
    horn_contact_angle_deg: float
    torus_exit_tangent_error_deg: float
    horn_entry_tangent_error_deg: float
    mouth_tangent_error_deg: float
    minimum_core_surface_distance_mm: float
    minimum_lined_mouth_margin_mm: float
    minimum_shoe_free_clearance_mm: float
    minimum_nonparent_copper_margin_mm: float
    minimum_parent_prefix_margin_mm: float
    minimum_current_arrival_margin_mm: float
    core_inside_sample_count: int
    failures: tuple[str, ...]


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: dict[str, Any]) -> str:
    canonical = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False
    )
    return hashlib.sha256(canonical.encode()).hexdigest()


def _finite(value: float, fallback: float = 1e9) -> float:
    return float(value) if math.isfinite(float(value)) else float(fallback)


def _unit(vector: np.ndarray) -> np.ndarray:
    value = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(value))
    if norm <= 1e-12:
        raise ValueError("zero vector")
    return value / norm


def _angle_deg(one: np.ndarray, two: np.ndarray) -> float:
    cosine = float(np.clip(np.dot(_unit(one), _unit(two)), -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _sample_polyline(points: np.ndarray,
                     spacing_mm: float = ROUTE_SAMPLE_SPACING_MM
                     ) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2:
        raise ValueError("polyline must be Nx3 with N>=2")
    result = []
    for index, (start, end) in enumerate(zip(points, points[1:])):
        distance = float(np.linalg.norm(end - start))
        count = max(2, math.ceil(distance / spacing_mm) + 1)
        segment = np.linspace(start, end, count)
        if index:
            segment = segment[1:]
        result.append(segment)
    return np.vstack(result)


def _trim_polyline_end(points: np.ndarray, trim_mm: float) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if trim_mm <= 0.0:
        return points.copy()
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    total = float(np.sum(lengths))
    if total <= trim_mm + 1e-12:
        return points[:2].copy()
    keep_to = total - trim_mm
    cumulative = 0.0
    result = [points[0]]
    for start, end, length in zip(points, points[1:], lengths):
        if cumulative + length < keep_to - 1e-12:
            result.append(end)
            cumulative += float(length)
            continue
        ratio = (keep_to - cumulative) / float(length)
        result.append(start + ratio * (end - start))
        break
    return np.asarray(result, dtype=float)


def _trim_polyline_start(points: np.ndarray, trim_mm: float) -> np.ndarray:
    return _trim_polyline_end(np.asarray(points, dtype=float)[::-1], trim_mm)[::-1]


def _machine_to_local(points: np.ndarray, axis_z_mm: float) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    return np.column_stack((
        float(axis_z_mm) - points[:, 2],
        -points[:, 0],
        points[:, 1],
    ))


def _local_to_machine(points: np.ndarray, axis_z_mm: float) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    return np.column_stack((
        -points[:, 1],
        points[:, 2],
        float(axis_z_mm) - points[:, 0],
    ))


def _part_mesh(part: Any, linear: float = 0.01,
               angular: float = 0.03) -> trimesh.Trimesh:
    meshes = []
    solids = list(part.solids()) or [part]
    for solid in solids:
        vertices, faces = solid.tessellate(linear, angular)
        mesh = trimesh.Trimesh(
            vertices=np.array([(v.X, v.Y, v.Z) for v in vertices]),
            faces=np.asarray(faces),
            process=True,
        )
        if not mesh.is_watertight:
            raise RuntimeError(
                f"{getattr(part, 'label', 'part')} contains a non-watertight solid"
            )
        meshes.append(mesh)
    return trimesh.util.concatenate(meshes)


@lru_cache(maxsize=1)
def _manifest() -> dict[str, Any]:
    return json.loads(MANIFEST_PATH.read_text())


@lru_cache(maxsize=1)
def _packing() -> dict[str, Any]:
    return json.loads(PACKING_PATH.read_text())


@lru_cache(maxsize=1)
def _graph() -> slot_route.PackingSupportGraph:
    return slot_route.PackingSupportGraph.from_report(
        _packing(), spec=DEFAULT_STATOR
    )


@lru_cache(maxsize=1)
def _planner() -> slot_route.SlotRoutePlanner:
    config = _packing()["config"]
    return slot_route.SlotRoutePlanner.from_project(
        _manifest(),
        spec=DEFAULT_STATOR,
        access_radius_mm=float(config["center_core_access_mm"]),
        planner_offset_mm=float(config["center_core_access_mm"]),
    )


@lru_cache(maxsize=1)
def _shoe_mesh() -> trimesh.Trimesh:
    meshes = [_part_mesh(part) for part in shoe.shoe_parts()]
    return trimesh.util.concatenate(meshes)


@lru_cache(maxsize=1)
def _shoe_query() -> trimesh.proximity.ProximityQuery:
    return trimesh.proximity.ProximityQuery(_shoe_mesh())


def _horn_direction_toward_mouth(side: int, axial_sign: int,
                                 theta: float) -> np.ndarray:
    _origin, u, _normal = shoe.nose_frame_vectors(side)
    axial = np.array((0.0, 1.0, 0.0))
    derivative_increasing = (
        shoe.HORN_CENTERLINE_RADIUS_MM * math.sin(theta) * u
        + axial_sign
        * shoe.HORN_CENTERLINE_RADIUS_MM * math.cos(theta) * axial
    )
    return -_unit(derivative_increasing)


def _ideal_shoe_center_distance(points: np.ndarray) -> np.ndarray:
    """Distance from points to the controlled blade/horn ideal solids.

    This is the analytical primitive set used to generate the CAD: a finite
    blade slab in the nose frame and two finite circular horn disks per side.
    The small curve variation beyond the nose is conservatively covered by
    the corridor/BREP insertion gate rather than hidden in route exclusions.
    """

    points = np.asarray(points, dtype=float)
    best = np.full(len(points), math.inf, dtype=float)
    half_stack = float(DEFAULT_STATOR.stack) / 2.0
    blade_length = float(sum(
        math.hypot(two[0] - one[0], two[1] - one[1])
        for one, two in zip(
            shoe.blade_center_samples(33),
            shoe.blade_center_samples(33)[1:],
        )
    ))
    half_thickness = shoe.BLADE_THICKNESS_MM / 2.0
    for side in (-1, 1):
        origin, u, normal = shoe.nose_frame_vectors(side)
        rel = points - origin
        local_u = rel @ u
        local_v = points[:, 1]
        local_n = rel @ normal

        # Exact point-to-axis-aligned box distance in the blade frame.
        du = np.maximum(np.maximum(-local_u, local_u - blade_length), 0.0)
        dv = np.maximum(np.abs(local_v) - half_stack, 0.0)
        dn = np.maximum(np.abs(local_n) - half_thickness, 0.0)
        outside = np.sqrt(du * du + dv * dv + dn * dn)
        inside = (
            (local_u >= 0.0) & (local_u <= blade_length)
            & (np.abs(local_v) <= half_stack)
            & (np.abs(local_n) <= half_thickness)
        )
        outside[inside] = 0.0
        best = np.minimum(best, outside)

        for axial_sign in (-1, 1):
            center_u = shoe.HORN_SURFACE_RADIUS_MM
            center_v = axial_sign * (
                half_stack + shoe.HORN_AXIAL_CENTER_OFFSET_MM
            )
            radial = np.hypot(local_u - center_u, local_v - center_v)
            radial_gap = np.maximum(radial - shoe.HORN_SURFACE_RADIUS_MM, 0.0)
            normal_gap = np.maximum(np.abs(local_n) - half_thickness, 0.0)
            distance = np.hypot(radial_gap, normal_gap)
            cylinder_inside = (
                (radial <= shoe.HORN_SURFACE_RADIUS_MM)
                & (np.abs(local_n) <= half_thickness)
            )
            distance[cylinder_inside] = 0.0
            best = np.minimum(best, distance)
    return best


def _route_to_shoe_margin(route_world: np.ndarray,
                           horn_first_index: int,
                           wire_radius_mm: float) -> float:
    chunks = []
    if horn_first_index >= 1:
        chunks.append(_trim_polyline_end(
            route_world[:horn_first_index + 1], 0.30
        ))
    # The horn mouth itself is an intentional contact.  Start the independent
    # mouth-to-lay clearance after a 0.30 mm controlled contact allowance.
    chunks.append(_trim_polyline_start(route_world[-2:], 0.30))
    minimum = math.inf
    spacing = 0.05
    for chunk in chunks:
        samples = _sample_polyline(chunk, spacing_mm=spacing)
        distance = float(np.min(_ideal_shoe_center_distance(samples)))
        # A half-sample Lipschitz bound keeps the point sweep conservative for
        # continuous segment distance.
        minimum = min(
            minimum, distance - float(wire_radius_mm) - spacing / 2.0
        )
    return float(minimum)


@lru_cache(maxsize=4096)
def _torus_to_horn(flyer_angle_deg: int, side: int,
                   axial_sign: int) -> dict[str, Any]:
    """Numerically couple the flyer torus to one exposed horn quadrant."""

    manifest = _manifest()
    guide = manifest["wire"]["tip_guide"]
    angle = math.radians(int(flyer_angle_deg) % 360)
    rotation = wirepath.rot_z(angle)
    feed = rotation @ np.asarray(guide["feed_local_mm"], dtype=float)
    best: tuple[float, float, np.ndarray, dict[str, Any]] | None = None
    # A non-zero contact arc is mandatory; zero would merely rename the old
    # unsupported straight endpoint.
    for theta in np.linspace(math.radians(8.0), math.pi / 2.0, 42):
        target = shoe.horn_wire_center_point(side, axial_sign, float(theta))
        try:
            path, meta = wirepath.tip_guide_path(
                feed,
                target,
                guide,
                shoe.MAX_WIRE_RADIUS_MM,
                rotation,
                arc_step_deg=2.0,
            )
        except (RuntimeError, ValueError):
            continue
        continuation = _horn_direction_toward_mouth(
            side, axial_sign, float(theta)
        )
        error = _angle_deg(target - path[-2], continuation)
        if best is None or error < best[0]:
            best = (error, float(theta), path, meta)
    if best is None:
        raise RuntimeError("no torus-to-horn candidate exists")

    # Deterministic local refinement around the best coarse angle.
    error, theta, path, meta = best
    for span_deg in (2.0, 0.4, 0.08):
        candidates = np.linspace(
            max(math.radians(8.0), theta - math.radians(span_deg)),
            min(math.pi / 2.0, theta + math.radians(span_deg)),
            17,
        )
        for candidate in candidates:
            target = shoe.horn_wire_center_point(
                side, axial_sign, float(candidate)
            )
            try:
                candidate_path, candidate_meta = wirepath.tip_guide_path(
                    feed,
                    target,
                    guide,
                    shoe.MAX_WIRE_RADIUS_MM,
                    rotation,
                    arc_step_deg=2.0,
                )
            except (RuntimeError, ValueError):
                continue
            candidate_error = _angle_deg(
                target - candidate_path[-2],
                _horn_direction_toward_mouth(
                    side, axial_sign, float(candidate)
                ),
            )
            if candidate_error < error:
                error, theta = candidate_error, float(candidate)
                path, meta = candidate_path, candidate_meta
    return {
        "error_deg": float(error),
        "theta_rad": float(theta),
        "path": path,
        "meta": meta,
    }


def _route_geometry(depth_index: int, flyer_angle_deg: int,
                    motion_sign: int) -> dict[str, Any]:
    graph = _graph()
    turn_index = round(int(depth_index) * (len(graph.turns) - 1)
                       / (DEPTH_COUNT - 1))
    turn = graph.turn(turn_index)
    radial = float(shoe.DEPTH_RADII_MM[int(depth_index)])
    signed_phase = motion_sign * math.radians(int(flyer_angle_deg) % 360)
    yz = slot_route._rounded_loop_yz(
        turn.profile_radius_mm, signed_phase, DEFAULT_STATOR
    )
    target_local = np.array((radial, yz[0], yz[1]), dtype=float)
    axis_z = shoe.WIRE_LAY_DATUM_Z_MM + radial
    target_world = _local_to_machine(target_local[None, :], axis_z)[0]
    side = 1 if target_world[0] < 0.0 else -1
    axial_sign = 1 if target_world[1] >= 0.0 else -1
    coupled = _torus_to_horn(flyer_angle_deg, side, axial_sign)
    theta = float(coupled["theta_rad"])
    torus_path = np.asarray(coupled["path"], dtype=float)
    arc_count = max(
        2, math.ceil(math.degrees(theta) / HORN_ARC_STEP_DEG)
    )
    horn_arc = np.asarray([
        shoe.horn_wire_center_point(
            side, axial_sign, theta * (1.0 - index / arc_count)
        )
        for index in range(arc_count + 1)
    ])
    mouth = horn_arc[-1]
    if np.linalg.norm(target_world - mouth) <= 1e-9:
        route = np.vstack((torus_path, horn_arc[1:]))
        mouth_error = 0.0
    else:
        route = np.vstack((torus_path, horn_arc[1:], target_world))
        mouth_error = _angle_deg(
            horn_arc[-1] - horn_arc[-2], target_world - mouth
        )
    horn_first = len(torus_path) - 1
    horn_last = horn_first + len(horn_arc) - 1
    return {
        "turn_index": int(turn_index),
        "radial_mm": radial,
        "profile_radius_mm": float(turn.profile_radius_mm),
        "axis_z_mm": axis_z,
        "signed_phase_rad": float(signed_phase),
        "target_local_mm": target_local,
        "target_world_mm": target_world,
        "side": side,
        "axial_sign": axial_sign,
        "route_world_mm": route,
        "horn_first_index": horn_first,
        "horn_last_index": horn_last,
        "horn_contact_angle_deg": math.degrees(theta),
        "horn_entry_tangent_error_deg": float(coupled["error_deg"]),
        "torus_exit_tangent_error_deg": float(
            coupled["meta"]["exit_tangent_error"]
        ),
        "mouth_tangent_error_deg": float(mouth_error),
    }


@lru_cache(maxsize=DEPTH_COUNT)
def _copper_fields(depth_index: int) -> dict[str, Any]:
    graph = _graph()
    turn_index = round(int(depth_index) * (len(graph.turns) - 1)
                       / (DEPTH_COUNT - 1))
    turn = graph.turn(turn_index)
    prior = slot_route.active_copper_before(
        graph, turn_index, DEFAULT_STATOR, arc_step_deg=5.0
    )
    parents = {
        f"active-turn-{index:02d}" for index in turn.parent_turn_indices
    }
    nonparents = tuple(
        obstacle for obstacle in prior if obstacle.obstacle_id not in parents
    )
    parent_obstacles = tuple(
        obstacle for obstacle in prior if obstacle.obstacle_id in parents
    )
    return {
        "turn_index": turn_index,
        "all": slot_route.CopperField(tuple(prior)),
        "nonparents": slot_route.CopperField(nonparents),
        "parents": slot_route.CopperField(parent_obstacles),
        "parent_ids": sorted(parents),
    }


def _current_arrival_field(radial_mm: float, profile_radius_mm: float,
                           signed_phase_rad: float
                           ) -> slot_route.CopperField | None:
    if abs(signed_phase_rad) < math.radians(1.0):
        return None
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
    # Remove one wire diameter at the live end.  That is the same continuous
    # conductor joining the route and is not an obstacle to itself.
    points = _trim_polyline_end(points, _graph().wire_diameter_mm)
    if len(points) < 2 or np.linalg.norm(points[-1] - points[0]) < 1e-9:
        return None
    obstacle = slot_route.CopperPolyline(
        obstacle_id="current-arrived-segment",
        owner="already_laid_current_half",
        turn_index=None,
        centerline_local_mm=tuple(tuple(map(float, point)) for point in points),
    )
    return slot_route.CopperField((obstacle,))


def _evaluate_case(depth_index: int, flyer_angle_deg: int,
                   motion_sign: int) -> RouteCase:
    geometry = _route_geometry(depth_index, flyer_angle_deg, motion_sign)
    graph = _graph()
    config = _packing()["config"]
    wire_radius = float(config["wire_radius_mm"])
    liner = float(config["liner_thickness_mm"])
    route_world = np.asarray(geometry["route_world_mm"], dtype=float)
    route_local = _machine_to_local(route_world, geometry["axis_z_mm"])
    planner = _planner()
    horn_first = int(geometry["horn_first_index"])
    critical_local = route_local[max(0, horn_first - 1):]
    core_min, _core_segment, _core_triangle = (
        slot_route.exact_polyline_mesh_clearance(
            critical_local, planner.mesh, planner.mesh_search_band_mm
        )
    )
    core_margin = float(core_min) - wire_radius

    # The supported mouth/lay segment is the final route segment.  Its core
    # clearance must include the installed nominal liner as well as wire.
    mouth_distance, _mouth_segment, _mouth_triangle = (
        slot_route.exact_polyline_mesh_clearance(
            route_local[-2:], planner.mesh, planner.mesh_search_band_mm
        )
    )
    mouth_margin = float(mouth_distance) - wire_radius - liner

    # Exclude the declared horn-contact arc from shoe clearance.  The free
    # torus path is trimmed 0.30 mm before the contact and the mouth-to-lay
    # segment remains a checked free/supported segment.
    shoe_clearance = _route_to_shoe_margin(
        route_world, horn_first, wire_radius
    )

    fields = _copper_fields(depth_index)
    route_prefix = _trim_polyline_end(route_local, graph.wire_diameter_mm)
    nonparent = fields["nonparents"].clearance(route_local, 1.0)
    nonparent_margin = (
        float(nonparent.minimum_centerline_distance_mm)
        - graph.wire_diameter_mm
    )
    parent = fields["parents"].clearance(route_prefix, 1.0)
    parent_margin = (
        float(parent.minimum_centerline_distance_mm)
        - graph.wire_diameter_mm
        if fields["parent_ids"] else 1e9
    )
    current = _current_arrival_field(
        geometry["radial_mm"],
        geometry["profile_radius_mm"],
        geometry["signed_phase_rad"],
    )
    if current is None:
        current_margin = 1e9
    else:
        current_clearance = current.clearance(route_prefix, 1.0)
        current_margin = (
            float(current_clearance.minimum_centerline_distance_mm)
            - graph.wire_diameter_mm
        )

    failures = []
    if (geometry["torus_exit_tangent_error_deg"]
            > TORUS_CHORD_TANGENCY_LIMIT_DEG):
        failures.append("torus_exit_not_C1")
    if geometry["horn_entry_tangent_error_deg"] > TANGENCY_LIMIT_DEG:
        failures.append("horn_entry_not_C1")
    if geometry["mouth_tangent_error_deg"] > TANGENCY_LIMIT_DEG:
        failures.append("mouth_to_lay_not_C1")
    if core_margin < -1e-9:
        failures.append("bare_core_clearance")
    if mouth_margin < -1e-9:
        failures.append("lined_mouth_clearance")
    if shoe_clearance < -1e-9:
        failures.append("shoe_free_segment_clearance")
    if nonparent_margin < -1e-9:
        failures.append("prior_nonparent_copper_clearance")
    if parent_margin < -1e-9:
        failures.append("support_parent_prefix_clearance")
    if current_margin < -1e-9:
        failures.append("already_laid_current_half_clearance")

    return RouteCase(
        depth_index=int(depth_index),
        turn_index=int(geometry["turn_index"]),
        flyer_angle_deg=int(flyer_angle_deg),
        motion_sign=int(motion_sign),
        route_ok=not failures,
        selected_side=int(geometry["side"]),
        selected_axial_sign=int(geometry["axial_sign"]),
        horn_contact_angle_deg=float(geometry["horn_contact_angle_deg"]),
        torus_exit_tangent_error_deg=float(
            geometry["torus_exit_tangent_error_deg"]
        ),
        horn_entry_tangent_error_deg=float(
            geometry["horn_entry_tangent_error_deg"]
        ),
        mouth_tangent_error_deg=float(geometry["mouth_tangent_error_deg"]),
        minimum_core_surface_distance_mm=core_min,
        minimum_lined_mouth_margin_mm=float(mouth_margin),
        minimum_shoe_free_clearance_mm=float(shoe_clearance),
        minimum_nonparent_copper_margin_mm=float(nonparent_margin),
        minimum_parent_prefix_margin_mm=float(parent_margin),
        minimum_current_arrival_margin_mm=float(current_margin),
        core_inside_sample_count=int(core_min <= 1e-9),
        failures=tuple(failures),
    )


def _fcl_distance(bvh_a: fcl.BVHModel, bvh_b: fcl.BVHModel,
                  rotation: np.ndarray | None = None,
                  translation: np.ndarray | None = None) -> float:
    rotation = np.eye(3) if rotation is None else np.asarray(rotation, dtype=float)
    translation = (np.zeros(3) if translation is None
                   else np.asarray(translation, dtype=float))
    object_a = fcl.CollisionObject(bvh_a, fcl.Transform())
    object_b = fcl.CollisionObject(
        bvh_b, fcl.Transform(rotation, translation)
    )
    result = fcl.CollisionResult()
    fcl.collide(object_a, object_b, fcl.CollisionRequest(), result)
    if result.is_collision:
        return -1.0
    return float(fcl.distance(
        object_a,
        object_b,
        fcl.DistanceRequest(),
        fcl.DistanceResult(),
    ))


def _load_link_mesh(link: str, exclude: set[str] | None = None
                    ) -> trimesh.Trimesh:
    exclude = set() if exclude is None else set(exclude)
    manifest = _manifest()
    meshes = [
        trimesh.load(
            OUT / "links" / "parts" / link / f"{label}.stl", force="mesh"
        )
        for label in manifest["parts"][link]
        if label not in exclude
    ]
    return trimesh.util.concatenate(meshes)


def rigid_motion_audit() -> dict[str, Any]:
    manifest = _manifest()
    kin = collide.Kinematics(manifest)
    shoe_mesh = _shoe_mesh()
    flyer = _load_link_mesh("flyer")
    chuck = _load_link_mesh(
        "spindle", exclude={"stator_final_wound_envelope"}
    )
    bare_stator = _part_mesh(
        shoe._installed_stator_at_radius(
            float(manifest["m0_home_standoff"]) - shoe.WIRE_LAY_DATUM_Z_MM
        )
    )
    shoe_bvh = collide.make_bvh(shoe_mesh)
    flyer_bvh = collide.make_bvh(flyer)
    chuck_bvh = collide.make_bvh(chuck)
    stator_bvh = collide.make_bvh(bare_stator)

    flyer_rows = []
    for angle in range(ANGLE_COUNT):
        rotation = wirepath.rot_z(math.radians(angle))
        flyer_rows.append((angle, _fcl_distance(
            shoe_bvh, flyer_bvh, rotation=rotation
        )))

    chuck_rows = []
    for index, radial in enumerate(shoe.DEPTH_RADII_MM):
        axis_z = shoe.WIRE_LAY_DATUM_Z_MM + float(radial)
        m0 = (
            axis_z - float(manifest["m0_home_standoff"])
        ) / float(manifest["mm_per_rad_m0"])
        rotation, translation = kin.link_tf("spindle", m0, 0.0, 0.0)
        chuck_rows.append((index, _fcl_distance(
            shoe_bvh, chuck_bvh,
            rotation=rotation, translation=translation
        )))

    chuck_witness = min(chuck_rows, key=lambda row: row[1])
    witness_radial = float(shoe.DEPTH_RADII_MM[int(chuck_witness[0])])
    witness_axis = shoe.WIRE_LAY_DATUM_Z_MM + witness_radial
    witness_m0 = (
        witness_axis - float(manifest["m0_home_standoff"])
    ) / float(manifest["mm_per_rad_m0"])
    witness_rotation, witness_translation = kin.link_tf(
        "spindle", witness_m0, 0.0, 0.0
    )
    chuck_part_clearances = {}
    for label in manifest["parts"]["spindle"]:
        if label == "stator_final_wound_envelope":
            continue
        mesh = trimesh.load(
            OUT / "links" / "parts" / "spindle" / f"{label}.stl",
            force="mesh",
        )
        chuck_part_clearances[label] = _fcl_distance(
            shoe_bvh,
            collide.make_bvh(mesh),
            rotation=witness_rotation,
            translation=witness_translation,
        )

    index_rows = []
    pitch = 2.0 * math.pi / int(DEFAULT_STATOR.slots)
    for angle in np.linspace(0.0, pitch, 16):
        rotation = wirepath.rot_y(float(angle))
        axis = np.array((0.0, 0.0, float(manifest["m0_home_standoff"])))
        translation = axis - rotation @ axis
        index_rows.append((math.degrees(float(angle)), _fcl_distance(
            shoe_bvh,
            stator_bvh,
            rotation=rotation,
            translation=translation,
        )))

    def summarize(rows: list[tuple[float | int, float]]) -> dict[str, Any]:
        witness = min(rows, key=lambda row: row[1])
        return {
            "sample_count": len(rows),
            "minimum_clearance_mm": float(witness[1]),
            "witness": {"sample": float(witness[0]),
                        "clearance_mm": float(witness[1])},
            "target_mm": RIGID_CLEARANCE_TARGET_MM,
            "status": (
                "PASS" if witness[1] + 1e-9 >= RIGID_CLEARANCE_TARGET_MM
                else "FAIL"
            ),
        }

    return {
        "flyer_360deg": summarize(flyer_rows),
        "chuck_9_depths": {
            **summarize(chuck_rows),
            "witness_part_clearances_mm": chuck_part_clearances,
            "colliding_part_labels": sorted(
                label for label, value in chuck_part_clearances.items()
                if value < 0.0
            ),
        },
        "m1_index_at_full_retraction": summarize(index_rows),
    }


def insertion_brep_audit() -> dict[str, Any]:
    rows = []
    blades = (shoe.dielectric_blade(1), shoe.dielectric_blade(-1))
    for depth_index, radial in enumerate(shoe.DEPTH_RADII_MM):
        stator = shoe._installed_stator_at_radius(float(radial))
        volumes = []
        for blade in blades:
            common = stator & blade
            volumes.append(float(sum(solid.volume for solid in common.solids())))
        rows.append({
            "depth_index": depth_index,
            "radial_mm": float(radial),
            "left_common_volume_mm3": volumes[0],
            "right_common_volume_mm3": volumes[1],
            "status": "PASS" if max(volumes) <= 1e-9 else "FAIL",
        })
    worst = max(rows, key=lambda row: max(
        row["left_common_volume_mm3"], row["right_common_volume_mm3"]
    ))
    return {
        "method": "exact OpenCascade BREP common volume",
        "depth_count": len(rows),
        "rows": rows,
        "maximum_common_volume_mm3": max(
            worst["left_common_volume_mm3"],
            worst["right_common_volume_mm3"],
        ),
        "worst_depth_index": int(worst["depth_index"]),
        "status": "PASS" if all(row["status"] == "PASS" for row in rows)
        else "FAIL",
    }


def extraction_audit() -> dict[str, Any]:
    graph = _graph()
    config = _packing()["config"]
    wire_radius = float(config["wire_radius_mm"])
    loops = np.vstack([
        _sample_polyline(
            slot_route._loop_centerline(turn, DEFAULT_STATOR, arc_step_deg=2.0),
            spacing_mm=EXTRACTION_SAMPLE_SPACING_MM,
        )
        for turn in graph.turns
    ])
    start_axis = shoe.WIRE_LAY_DATUM_Z_MM + float(
        shoe.RADIAL_WORKING_END_MM
    )
    # Once the stator axis is beyond shoe.maxZ + OD/2 + 2 mm, monotonic M0
    # retraction can only increase separation; the remainder to home is
    # therefore bounded analytically.
    checked_end_axis = max(
        start_axis,
        float(_shoe_mesh().bounds[1, 2]) + float(DEFAULT_STATOR.od) / 2.0 + 2.0,
    )
    axes = np.arange(start_axis, checked_end_axis + 0.025, 0.05)
    worst = (math.inf, None, 0)
    collision_samples = 0
    for axis in axes:
        world = _local_to_machine(loops, float(axis))
        distances = _ideal_shoe_center_distance(world)
        margin = (
            float(np.min(distances))
            - wire_radius
            - EXTRACTION_SAMPLE_SPACING_MM / 2.0
        )
        contained = distances <= 1e-12
        if margin < worst[0]:
            worst = (margin, float(axis), int(np.count_nonzero(contained)))
        if margin < -1e-9:
            collision_samples += 1
    return {
        "method": (
            "all 50 exact packed-loop polylines, <=0.05 mm centre spacing, "
            "0.05 mm M0 translation spacing, analytical source blade/horn solids"
        ),
        "axis_range_checked_mm": [float(start_axis), float(checked_end_axis)],
        "axis_sample_count": int(len(axes)),
        "minimum_center_to_shoe_margin_mm": float(worst[0]),
        "worst_axis_z_mm": worst[1],
        "inside_center_sample_count_at_worst": int(worst[2]),
        "colliding_axis_sample_count": int(collision_samples),
        "remainder_to_home_proof": (
            "after checked_end the complete OD46 workpiece is at least 2 mm "
            "beyond the shoe maximum-Z plane and separation grows monotonically"
        ),
        "status": "PASS" if worst[0] >= -1e-9 else "FAIL",
    }


def _case_summary(cases: list[RouteCase]) -> dict[str, Any]:
    failure_counts: dict[str, int] = {}
    for case in cases:
        for failure in case.failures:
            failure_counts[failure] = failure_counts.get(failure, 0) + 1

    def minimum(field: str) -> dict[str, Any]:
        case = min(cases, key=lambda item: getattr(item, field))
        return {
            "value": float(getattr(case, field)),
            "depth_index": case.depth_index,
            "turn_index": case.turn_index,
            "flyer_angle_deg": case.flyer_angle_deg,
            "motion_sign": case.motion_sign,
        }

    def maximum(field: str) -> dict[str, Any]:
        case = max(cases, key=lambda item: getattr(item, field))
        return {
            "value": float(getattr(case, field)),
            "depth_index": case.depth_index,
            "turn_index": case.turn_index,
            "flyer_angle_deg": case.flyer_angle_deg,
            "motion_sign": case.motion_sign,
        }

    return {
        "required_case_count": ANGLE_COUNT * DEPTH_COUNT * len(DIRECTIONS),
        "evaluated_case_count": len(cases),
        "passing_case_count": sum(case.route_ok for case in cases),
        "failing_case_count": sum(not case.route_ok for case in cases),
        "failure_counts": dict(sorted(failure_counts.items())),
        "worst": {
            "maximum_horn_entry_tangent_error_deg": maximum(
                "horn_entry_tangent_error_deg"
            ),
            "maximum_mouth_tangent_error_deg": maximum(
                "mouth_tangent_error_deg"
            ),
            "minimum_lined_mouth_margin_mm": minimum(
                "minimum_lined_mouth_margin_mm"
            ),
            "minimum_shoe_free_clearance_mm": minimum(
                "minimum_shoe_free_clearance_mm"
            ),
            "minimum_nonparent_copper_margin_mm": minimum(
                "minimum_nonparent_copper_margin_mm"
            ),
            "minimum_parent_prefix_margin_mm": minimum(
                "minimum_parent_prefix_margin_mm"
            ),
            "minimum_current_arrival_margin_mm": minimum(
                "minimum_current_arrival_margin_mm"
            ),
        },
        "status": "PASS" if all(case.route_ok for case in cases) else "FAIL",
    }


def material_rfq_contract() -> dict[str, Any]:
    return {
        "status": "BLOCKED_UNQUALIFIED",
        "custom_part": "split active-tooth dielectric insert, quantity 2",
        "drawing_inputs_mm": {
            "blade_thickness_nominal": shoe.BLADE_THICKNESS_MM,
            "blade_thickness_maximum": shoe.MAXIMUM_BLADE_THICKNESS_MM,
            "projected_radial_working_span": shoe.RADIAL_WORKING_SPAN_MM,
            "horn_surface_radius": shoe.HORN_SURFACE_RADIUS_MM,
            "minimum_axial_projection": shoe.MINIMUM_AXIAL_PROJECTION_MM,
            "mount_datum_A_machine_Z": shoe.MOUNT_DATUM_A_Z_MM,
            "mount_hole_diameter": shoe.MOUNT_HOLE_DIAMETER_MM,
        },
        "required_supplier_declarations": [
            "electrically insulating and non-magnetic finished insert",
            "material grade, lot traceability, dielectric data, and service-temperature data",
            "wire-contact surface Ra <= 0.20 um after all edge finishing",
            "no chips, burrs, coating holidays, or sharp contact edges",
            "dimensional capability for the blade and horn drawing",
        ],
        "qualification_coupon": [
            "route the received production wire over a representative horn/blade coupon at the production tension envelope",
            "cycle through the production reversal count and inspect enamel microscopically",
            "perform insulation withstand testing before and after cycling",
            "measure horn wear and blade thickness; feed the measured maxima back into this audit",
        ],
        "blockers": [
            "no supplier or manufacturable 0.10 mm rigid dielectric insert has been selected",
            "no abrasion, reversal-life, dielectric, heat, or wear coupon has been run",
            "the geometry fails before material qualification can authorize release",
        ],
    }


def cap_relief_contract() -> dict[str, Any]:
    geometry = shoe._JOB["slot"]
    bare = float(geometry["opening_width_mm"])
    lined = bare - 2.0 * float(insulation.MATERIAL_RECEIVING_MAX_MM)
    current = bare - 2.0 * float(insulation.CAP_EDGE_OVERLAP_MM)
    residual = (
        lined
        - 2.0 * shoe.MAX_WIRE_RADIUS_MM
        - shoe.BLADE_THICKNESS_MM
    )
    return {
        "status": "NOT_RELEASED",
        "active_mouth_count": 2,
        "bare_mouth_mm": bare,
        "current_cap_mouth_mm": current,
        "target_liner_only_mouth_mm": lined,
        "blade_thickness_mm": shoe.BLADE_THICKNESS_MM,
        "maximum_wire_diameter_mm": 2.0 * shoe.MAX_WIRE_RADIUS_MM,
        "residual_lateral_budget_before_motion_error_mm": residual,
        "continuous_insulation_rule": (
            "locally remove only the inward star-cap encroachment at the two "
            "active mouths; the blade must overlap the formed slot-cell liner "
            "and remain between every steel edge and the wire"
        ),
        "release_blocker": (
            "the rigid blade has no common liner-preserving M0 corridor, so "
            "a fabrication DXF would encode a known-unbuildable interface"
        ),
    }


def analyze() -> dict[str, Any]:
    cases: list[RouteCase] = []
    for depth_index in range(DEPTH_COUNT):
        for motion_sign in DIRECTIONS:
            for angle in range(ANGLE_COUNT):
                cases.append(_evaluate_case(depth_index, angle, motion_sign))
        print(f"active shoe route depth {depth_index + 1}/{DEPTH_COUNT}")

    corridor = shoe.corridor_summary()
    insertion = insertion_brep_audit()
    extraction = extraction_audit()
    rigid = rigid_motion_audit()
    route = _case_summary(cases)
    cap = cap_relief_contract()
    material = material_rfq_contract()
    gates = {
        "common_rigid_blade_corridor": corridor["status"] == "PASS",
        "exact_brep_insertion": insertion["status"] == "PASS",
        "route_360x9x2": route["status"] == "PASS",
        "radial_extraction": extraction["status"] == "PASS",
        "flyer_rigid_clearance": rigid["flyer_360deg"]["status"] == "PASS",
        "chuck_rigid_clearance": rigid["chuck_9_depths"]["status"] == "PASS",
        "indexing_at_retraction": (
            rigid["m1_index_at_full_retraction"]["status"] == "PASS"
        ),
        "cap_relief_fabrication": cap["status"] == "PASS",
        "material_and_finish_qualified": material["status"] == "PASS",
    }
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "PASS" if all(gates.values()) else "DESIGN_NO_GO",
        "release_authorized": False,
        "assembly_integration_authorized": False,
        "scope": {
            "flyer_angles": ANGLE_COUNT,
            "depths": DEPTH_COUNT,
            "directions": list(DIRECTIONS),
            "required_route_cases": ANGLE_COUNT * DEPTH_COUNT * len(DIRECTIONS),
            "minimum_horn_centerline_radius_mm": 3.0,
            "route_sample_spacing_mm": ROUTE_SAMPLE_SPACING_MM,
            "torus_chord_tangency_limit_deg": (
                TORUS_CHORD_TANGENCY_LIMIT_DEG
            ),
            "stator": "default OD46 x stack15 x 24-slot source BREP",
        },
        "mount_and_retraction_datum": {
            "architecture": "two-part machine-fixed split shoe",
            "datum_A": "machine plane Z=8.000 mm",
            "mount_hole_axes": [
                {"x_side": side, "y_mm": axial * shoe.MOUNT_HOLE_Y_MM,
                 "axis": "+/- machine Z", "diameter_mm": shoe.MOUNT_HOLE_DIAMETER_MM}
                for side in (-1, 1) for axial in (-1, 1)
            ],
            "working_lay_plane_Z_mm": shoe.WIRE_LAY_DATUM_Z_MM,
            "retraction": (
                "M0 translates the workpiece +Z away from the fixed shoe; "
                "M1 indexing is permitted only at the existing home standoff"
            ),
        },
        "gates": gates,
        "corridor": corridor,
        "insertion": insertion,
        "route_sweep": route,
        "extraction": extraction,
        "rigid_motion": rigid,
        "cap_relief": cap,
        "material_finish_rfq": material,
        "decision": (
            "Do not integrate this shoe. The fixed rigid blade has no common "
            "liner-preserving corridor over the required M0 depth span, and "
            "the remaining route, extraction, rigid, cap, and material gates "
            "remain fail-closed. A viable successor needs an independent "
            "tangential/compliant blade motion or a new packing/depth window."
        ),
        "source_hashes": {
            "cad/active_tooth_shoe.py": _sha256(CAD / "active_tooth_shoe.py"),
            "sim/active_tooth_shoe_route.py": _sha256(Path(__file__)),
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
    route = report["route_sweep"]
    corridor = report["corridor"]
    insertion = report["insertion"]
    rigid = report["rigid_motion"]
    lines = [
        "# Active-tooth split shoe audit",
        "",
        f"Status: **{report['status']}**. Assembly integration and hardware release are false.",
        "",
        "## Controlling no-go",
        "",
        (
            f"The common fixed-blade corridor reaches {corridor['minimum_common_corridor_margin_mm']:.6f} mm "
            f"({corridor['failing_station_count']} of {corridor['sample_count']} stations fail)."
        ),
        (
            f"Exact BREP insertion reaches {insertion['maximum_common_volume_mm3']:.6f} mm^3 common volume "
            f"at depth {insertion['worst_depth_index']}."
        ),
        "",
        "## Full route sweep",
        "",
        (
            f"Evaluated {route['evaluated_case_count']} / {route['required_case_count']} required "
            f"360 deg x 9 depth x 2 direction cases; {route['passing_case_count']} pass."
        ),
        f"Failure counts: `{json.dumps(route['failure_counts'], sort_keys=True)}`",
        "",
        "## Rigid motion",
        "",
        (
            f"Flyer minimum {rigid['flyer_360deg']['minimum_clearance_mm']:.3f} mm; "
            f"chuck minimum {rigid['chuck_9_depths']['minimum_clearance_mm']:.3f} mm; "
            f"retracted indexing minimum {rigid['m1_index_at_full_retraction']['minimum_clearance_mm']:.3f} mm."
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
    args = parser.parse_args()
    report = analyze()
    if args.write:
        write(report)
    print(json.dumps({
        "status": report["status"],
        "report_sha256": report["report_sha256"],
        "route_cases": report["route_sweep"]["evaluated_case_count"],
        "route_passes": report["route_sweep"]["passing_case_count"],
        "corridor_margin_mm": report["corridor"]["minimum_common_corridor_margin_mm"],
    }, indent=2))


if __name__ == "__main__":
    main()
