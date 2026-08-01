"""Coupled, slot-aware wire routing against the bare stator core.

The free span leaving the flyer torus may not acquire an unsupported kink.
For every visible offset-liner vertex this module therefore solves a new
tangent/torus/tangent path, checks the resulting exit-to-vertex segment
against the complete bare-stator mesh, and only then follows the shortest
visibility path whose bends are supported by the configured core offset.

Coordinates are stator-local: ``x`` is radial along the presented tooth,
``y`` is tangential, and ``z`` is axial.  The flyer/manifest frame maps that
to ``(-y, z, stator_axis_z - x)``.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any
from importlib.metadata import PackageNotFoundError, version

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import shortest_path
try:
    import shapely
    from shapely.geometry import LineString, Point, Polygon, box
    from shapely.ops import nearest_points, unary_union
except ModuleNotFoundError as exc:  # pragma: no cover - dependency bootstrap
    raise RuntimeError(
        "slot_route requires shapely==2.1.2; install the pinned README "
        "reproduction dependencies"
    ) from exc
import trimesh
from build123d import Compound, Edge, Kind, Plane, Vector, section


_EPS = 1e-12


@dataclass(frozen=True)
class RouteResult:
    """Fail-closed result for one radial/angle/direction route."""

    ok: bool
    reason: str
    points_local: tuple[tuple[float, float, float], ...] = ()
    segment_tags: tuple[str, ...] = ()
    torus_exit_point_index: int | None = None
    center_core_min_mm: float | None = None
    access_margin_mm: float | None = None
    torus_continuity_error_deg: float | None = None
    total_length_mm: float | None = None
    free_length_mm: float | None = None
    supported_bends: int | None = None
    obstruction_triangle: int | None = None
    endpoint_family: str | None = None
    endpoint_support: str | None = None
    boundary_source: str | None = None
    progressive_support_validated: bool = False
    metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def points_local_mm(self) -> tuple[tuple[float, float, float], ...]:
        """Unit-explicit alias used by player/table serializers."""

        return self.points_local


@lru_cache(maxsize=1)
def dependency_versions() -> dict[str, str]:
    """Versions that make a serialized route result reproducible."""

    result: dict[str, str] = {}
    for package in ("build123d", "numpy", "scipy", "shapely", "trimesh"):
        try:
            result[package] = version(package)
        except PackageNotFoundError:
            result[package] = "missing"
    return result


@dataclass(frozen=True)
class PackingTurn:
    """One deposited loop from the authoritative slot-packing graph."""

    turn_index: int
    layer_index: int
    radial_mm: float
    profile_radius_mm: float
    support_kind: str
    parent_turn_indices: tuple[int, ...]
    prior_contact_turn_indices: tuple[int, ...]


@dataclass(frozen=True)
class PackingSupportGraph:
    """Validated, hash-bound progressive support graph."""

    schema: str
    report_sha256: str
    wire_diameter_mm: float
    center_core_access_mm: float
    turns: tuple[PackingTurn, ...]

    @classmethod
    def from_report(cls, report: dict[str, Any], spec: Any = None
                    ) -> "PackingSupportGraph":
        if report.get("schema") == "slot-winding-plan/v1":
            return cls._from_winding_plan(report, spec=spec)
        packing_schema = report.get("schema")
        if packing_schema not in ("slot-packing/v1", "slot-packing/v2"):
            raise ValueError("unsupported slot-packing schema")
        if report.get("status") != "PASS":
            raise ValueError("slot-packing report is not PASS")
        payload = dict(report)
        expected_hash = payload.pop("report_sha256", None)
        if not isinstance(expected_hash, str):
            raise ValueError("slot-packing report has no stable hash")
        canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        actual_hash = hashlib.sha256(canonical.encode()).hexdigest()
        if actual_hash != expected_hash:
            raise ValueError("slot-packing report hash mismatch")
        config = report.get("config", {})
        selected = report.get("selected_schedule", {})
        rows = selected.get("side_positive")
        mirror = selected.get("side_negative")
        if not isinstance(rows, list) or not isinstance(mirror, list):
            raise ValueError("slot-packing report is missing both slot sides")
        if len(rows) != 50 or len(mirror) != 50:
            raise ValueError("slot-packing report must contain 50 turns/side")
        wire_d = float(config["wire_finished_diameter_mm"])
        access = float(config["center_core_access_mm"])
        if spec is not None:
            checks = {
                "slots": (int(config["slots"]), int(spec.slots)),
                "wire_d": (wire_d, float(spec.wire_d)),
                "turns": (len(rows), int(spec.turns)),
            }
            mismatches = [
                f"{name}: report={actual}, spec={expected}"
                for name, (actual, expected) in checks.items()
                if not math.isclose(float(actual), float(expected),
                                    rel_tol=0.0, abs_tol=1e-9)
            ]
            if mismatches:
                raise ValueError("packing report/spec mismatch: "
                                 + "; ".join(mismatches))

        turns: list[PackingTurn] = []
        centers = [np.asarray(row["center_xy_mm"], dtype=float)
                   for row in rows]
        for index, row in enumerate(rows):
            if int(row["turn_index"]) != index:
                raise ValueError("packing turn indices are not contiguous")
            parents = tuple(map(int, row["parent_turn_indices"]))
            contacts = tuple(map(int,
                                 row["prior_contact_turn_indices"]))
            layer = int(row["layer_index"])
            if any(parent >= index for parent in parents + contacts):
                raise ValueError("packing graph contains a forward contact")
            for contact in contacts:
                distance = float(np.linalg.norm(
                    centers[index] - centers[contact]))
                if abs(distance - wire_d) > 1e-9:
                    raise ValueError("declared packing contact is not tangent")
            if any(parent not in contacts for parent in parents):
                raise ValueError("packing support parent is not a tangent contact")
            support_kind = str(row["support_kind"])
            if index == 0:
                if support_kind != "slot_liner" or parents:
                    raise ValueError(
                        "packing seed must be liner-supported without parents")
            elif support_kind != "deposited_wire" or not parents:
                raise ValueError(
                    "every post-seed packing turn needs earlier wire support")
            raw_profile = float(row["normal_profile_radius_mm"])
            # v1 stored the offset from the tooth surface.  v2 stores the
            # active-tooth center coordinate, which includes half the tooth
            # neck.  Normalize both schemas to the rounded-loop buffer radius
            # consumed by _loop_centerline; subtracting a common constant
            # preserves every exact center/contact distance.
            if packing_schema == "slot-packing/v2":
                od = float(spec.od if spec is not None
                           else config.get("od_mm", 46.0))
                raw_profile -= max(2.5, od * 0.07) / 2.0
                if raw_profile <= 0.0:
                    raise ValueError("v2 packing profile is inside tooth neck")
            turns.append(PackingTurn(
                turn_index=index,
                layer_index=layer,
                radial_mm=float(row["radial_parameter_mm"]),
                profile_radius_mm=raw_profile,
                support_kind=support_kind,
                parent_turn_indices=parents,
                prior_contact_turn_indices=contacts,
            ))
        return cls(
            schema=str(report["schema"]),
            report_sha256=expected_hash,
            wire_diameter_mm=wire_d,
            center_core_access_mm=access,
            turns=tuple(turns),
        )

    @classmethod
    def _from_winding_plan(cls, report: dict[str, Any], spec: Any = None
                           ) -> "PackingSupportGraph":
        """Consume the project's richer constructive winding-plan schema."""

        payload = dict(report)
        expected_hash = payload.pop("proof_sha256", None)
        if not isinstance(expected_hash, str):
            raise ValueError("slot winding plan has no proof hash")
        canonical = json.dumps(
            payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
        if hashlib.sha256(canonical.encode()).hexdigest() != expected_hash:
            raise ValueError("slot winding plan proof hash mismatch")
        job = report.get("job", {})
        selected = report.get("selected_case", {})
        transition = selected.get("transition_proof", {})
        if selected.get("status") != "PASS" or transition.get("status") != "PASS":
            raise ValueError("slot winding plan is not constructively PASS")
        placements = report.get("placements")
        support_rows = transition.get("first_side_insertion")
        if not isinstance(placements, list) or not isinstance(support_rows, list):
            raise ValueError("slot winding plan is missing placements/support")
        turns_expected = int(job["turns_per_tooth"])
        if len(placements) != turns_expected or len(support_rows) != turns_expected:
            raise ValueError("slot winding plan placement count mismatch")
        wire_d = float(job["model_wire_envelope_mm"])
        access = float(selected["required_center_core_clearance_mm"])
        if spec is not None:
            checks = {
                "slots": (int(job["slots"]), int(spec.slots)),
                "od": (float(job["od_mm"]), float(spec.od)),
                "stack": (float(job["stack_mm"]), float(spec.stack)),
                "wire_d": (float(job["wire_finished_d_mm"]),
                           float(spec.wire_d)),
                "turns": (turns_expected, int(spec.turns)),
            }
            mismatches = [
                f"{name}: plan={actual}, spec={expected}"
                for name, (actual, expected) in checks.items()
                if not math.isclose(float(actual), float(expected),
                                    rel_tol=0.0, abs_tol=1e-9)
            ]
            if mismatches:
                raise ValueError("winding plan/spec mismatch: "
                                 + "; ".join(mismatches))

        support_by_index = {
            int(row["placement_index"]): row for row in support_rows
        }
        if set(support_by_index) != set(range(turns_expected)):
            raise ValueError("winding plan support indices are incomplete")
        centers = [np.array((
            float(row["active_tooth_radial_mm"]),
            float(row["active_tooth_tangential_mm"]),
        )) for row in placements]
        half_neck = max(
            2.5, float(job.get("od_mm", getattr(spec, "od", 46.0))) * 0.07
        ) / 2.0
        turns = []
        for index, row in enumerate(placements):
            if int(row["turn_index"]) != index:
                raise ValueError("winding plan turn indices are not contiguous")
            support = support_by_index[index]
            parents = tuple(map(
                int, support.get("support_predecessor_indices", ())))
            if any(parent >= index for parent in parents):
                raise ValueError("winding plan has a forward support edge")
            contacts = []
            for prior in range(index):
                distance = float(np.linalg.norm(centers[index] - centers[prior]))
                if abs(distance - wire_d) <= 1e-8:
                    contacts.append(prior)
            if index and support.get("support") != "slot_liner" and not parents:
                raise ValueError("winding placement has no support predecessor")
            if any(parent not in contacts for parent in parents):
                raise ValueError("winding support predecessor is not tangent")
            if "active_tooth_neck_normal_center_offset_mm" in row:
                profile_radius = float(
                    row["active_tooth_neck_normal_center_offset_mm"])
            else:
                profile_radius = abs(float(
                    row["active_tooth_tangential_mm"])) - half_neck
            if profile_radius <= 0.0:
                raise ValueError("winding placement is inside the tooth neck")
            turns.append(PackingTurn(
                turn_index=index,
                layer_index=int(row["layer"]),
                radial_mm=float(row["active_tooth_radial_mm"]),
                profile_radius_mm=profile_radius,
                support_kind=str(support["support"]),
                parent_turn_indices=parents,
                prior_contact_turn_indices=tuple(contacts),
            ))
        return cls(
            schema=str(report["schema"]),
            report_sha256=expected_hash,
            wire_diameter_mm=wire_d,
            center_core_access_mm=access,
            turns=tuple(turns),
        )

    @classmethod
    def load(cls, path: str | Path, spec: Any = None
             ) -> "PackingSupportGraph":
        return cls.from_report(json.loads(Path(path).read_text()), spec=spec)

    def turn(self, turn_index: int) -> PackingTurn:
        if not 0 <= int(turn_index) < len(self.turns):
            raise IndexError("packing turn index is outside the support graph")
        return self.turns[int(turn_index)]


@dataclass(frozen=True)
class DepositedLoopProfile:
    turn_index: int
    layer_index: int
    radial_mm: float
    profile_radius_mm: float
    parent_turn_indices: tuple[int, ...]
    centerline_local_mm: tuple[tuple[float, float, float], ...]


@dataclass(frozen=True)
class CopperPolyline:
    """One explicit deposited-wire centerline obstacle."""

    obstacle_id: str
    owner: str
    turn_index: int | None
    centerline_local_mm: tuple[tuple[float, float, float], ...]


@dataclass(frozen=True)
class CopperClearance:
    minimum_centerline_distance_mm: float
    route_segment_index: int | None
    obstacle_id: str | None
    obstacle_segment_index: int | None


class CopperField:
    """Spatially indexed exact segment/capsule centerline obstacle field."""

    def __init__(self, obstacles: tuple[CopperPolyline, ...]):
        self.obstacles = tuple(obstacles)
        starts: list[np.ndarray] = []
        ends: list[np.ndarray] = []
        identities: list[tuple[str, int]] = []
        for obstacle in self.obstacles:
            points = np.asarray(obstacle.centerline_local_mm, dtype=float)
            if (points.ndim != 2 or points.shape[1] != 3
                    or len(points) < 2 or not np.all(np.isfinite(points))):
                raise ValueError(
                    f"copper obstacle {obstacle.obstacle_id!r} has an "
                    "invalid centerline")
            for index, (start, end) in enumerate(zip(points, points[1:])):
                if np.linalg.norm(end - start) <= _EPS:
                    continue
                starts.append(start)
                ends.append(end)
                identities.append((obstacle.obstacle_id, index))
        self.starts = (np.asarray(starts, dtype=float)
                       if starts else np.empty((0, 3), dtype=float))
        self.ends = (np.asarray(ends, dtype=float)
                     if ends else np.empty((0, 3), dtype=float))
        self.identities = tuple(identities)
        if len(self.starts):
            bounds = np.column_stack((
                np.minimum(self.starts, self.ends),
                np.maximum(self.starts, self.ends),
            ))
            self._tree = trimesh.util.bounds_tree(bounds)
        else:
            self._tree = None

    def clearance(self, route_points: np.ndarray,
                  search_band_mm: float) -> CopperClearance:
        """Exact polyline-to-polyline centerline clearance."""

        route = np.asarray(route_points, dtype=float)
        if (route.ndim != 2 or route.shape[1] != 3 or len(route) < 2
                or not np.all(np.isfinite(route))):
            raise ValueError("route_points must be a finite 3D polyline")
        if search_band_mm <= 0.0:
            raise ValueError("copper search band must be positive")
        if self._tree is None:
            return CopperClearance(
                float(search_band_mm), None, None, None)
        best = float(search_band_mm)
        best_route = best_obstacle_segment = None
        best_id = None
        for route_index, (start, end) in enumerate(zip(route, route[1:])):
            lower = np.minimum(start, end) - search_band_mm
            upper = np.maximum(start, end) + search_band_mm
            candidates = sorted(self._tree.intersection((*lower, *upper)))
            for candidate in candidates:
                distance = _segment_segment_distance(
                    start, end, self.starts[candidate], self.ends[candidate])
                if distance < best:
                    best = distance
                    best_route = route_index
                    best_id, best_obstacle_segment = self.identities[candidate]
                    if best <= _EPS:
                        return CopperClearance(
                            best, best_route, best_id,
                            best_obstacle_segment)
        return CopperClearance(
            float(best), best_route, best_id, best_obstacle_segment)

    def point_clearances(self, point: np.ndarray,
                         search_band_mm: float
                         ) -> dict[str, float]:
        """Minimum point-to-centerline distance for nearby obstacles."""

        value = np.asarray(point, dtype=float)
        if value.shape != (3,) or not np.all(np.isfinite(value)):
            raise ValueError("point must be one finite 3D coordinate")
        if self._tree is None:
            return {}
        lower = value - search_band_mm
        upper = value + search_band_mm
        result: dict[str, float] = {}
        for candidate in sorted(self._tree.intersection((*lower, *upper))):
            start, end = self.starts[candidate], self.ends[candidate]
            delta = end - start
            denominator = float(delta @ delta)
            fraction = (0.0 if denominator <= _EPS else float(np.clip(
                ((value - start) @ delta) / denominator, 0.0, 1.0)))
            distance = float(np.linalg.norm(
                value - (start + fraction * delta)))
            obstacle_id, _ = self.identities[candidate]
            result[obstacle_id] = min(
                distance, result.get(obstacle_id, math.inf))
        return result


def projected_copper_obstacle(field: CopperField, radial_x_mm: float,
                              center_clearance_mm: float,
                              buffer_resolution: int = 16) -> Any:
    """Conservative yz projection of 3D capsule obstacles at one x plane."""

    if center_clearance_mm <= 0.0:
        raise ValueError("center_clearance_mm must be positive")
    radial = float(radial_x_mm)
    shapes = []
    for start, end in zip(field.starts, field.ends):
        x0, x1 = float(start[0]), float(end[0])
        if min(x0, x1) <= radial <= max(x0, x1):
            radial_distance = 0.0
        else:
            radial_distance = min(abs(radial - x0), abs(radial - x1))
        if radial_distance >= center_clearance_mm:
            continue
        radius = math.sqrt(max(
            0.0, center_clearance_mm ** 2 - radial_distance ** 2))
        yz = LineString(((float(start[1]), float(start[2])),
                         (float(end[1]), float(end[2]))))
        shapes.append(yz.buffer(radius, quad_segs=buffer_resolution))
    return unary_union(shapes).buffer(0) if shapes else None


@dataclass(frozen=True)
class PackingLoopContact:
    prior_turn_index: int
    centerline_distance_mm: float
    classification: str


@dataclass(frozen=True)
class PackingContactAudit:
    ok: bool
    progressive_support_validated: bool
    minimum_prior_centerline_distance_mm: float | None
    contacts: tuple[PackingLoopContact, ...]
    reason: str


@dataclass(frozen=True)
class PackingTurnRouteAudit:
    """Independent release postcheck for one packed half-turn crossing.

    ``route_explicit_core_target`` performs the same geometric checks while
    selecting a route.  This second record deliberately repeats them against
    the returned polyline so a serialized route table cannot mistake planner
    acceptance for a release proof.
    """

    ok: bool
    turn_index: int
    half_turn_index: int
    phase_index: int
    logical_phase_rad: float
    validated_motion_signs: tuple[int, int]
    target_local_mm: tuple[float, float, float]
    endpoint_error_mm: float
    minimum_core_center_distance_mm: float
    minimum_copper_center_distance_mm: float
    minimum_copper_obstacle_id: str | None
    support_contract_ok: bool
    required_core_center_distance_mm: float
    required_copper_center_distance_mm: float
    reason: str


def _loop_centerline(turn: PackingTurn, spec: Any,
                     arc_step_deg: float = 5.0) -> np.ndarray:
    """Rounded-rectangle deposited loop in the active-tooth frame."""

    if arc_step_deg <= 0.0:
        raise ValueError("arc_step_deg must be positive")
    half_neck = max(2.5, float(spec.od) * 0.07) / 2.0
    half_stack = float(spec.stack) / 2.0
    resolution = max(2, math.ceil(90.0 / float(arc_step_deg)))
    profile = box(-half_neck, -half_stack,
                  half_neck, half_stack).buffer(
                      turn.profile_radius_mm, quad_segs=resolution)
    yz = np.asarray(profile.exterior.coords, dtype=float)
    return np.column_stack((
        np.full(len(yz), turn.radial_mm), yz[:, 0], yz[:, 1],
    ))


def build_deposited_profiles(graph: PackingSupportGraph,
                             through_turn_index: int, spec: Any,
                             arc_step_deg: float = 5.0
                             ) -> tuple[DepositedLoopProfile, ...]:
    """Build active-tooth loop profiles through an inclusive turn index."""

    graph.turn(through_turn_index)
    result = []
    for turn in graph.turns[:int(through_turn_index) + 1]:
        points = _loop_centerline(turn, spec, arc_step_deg=arc_step_deg)
        result.append(DepositedLoopProfile(
            turn_index=turn.turn_index,
            layer_index=turn.layer_index,
            radial_mm=turn.radial_mm,
            profile_radius_mm=turn.profile_radius_mm,
            parent_turn_indices=turn.parent_turn_indices,
            centerline_local_mm=tuple(tuple(map(float, point))
                                      for point in points),
        ))
    return tuple(result)


def active_copper_before(graph: PackingSupportGraph, turn_index: int,
                         spec: Any, arc_step_deg: float = 5.0
                         ) -> tuple[CopperPolyline, ...]:
    """Completed active-tooth loops strictly before ``turn_index``."""

    graph.turn(turn_index)
    result = []
    for turn in graph.turns[:int(turn_index)]:
        points = _loop_centerline(turn, spec, arc_step_deg=arc_step_deg)
        result.append(CopperPolyline(
            obstacle_id=f"active-turn-{turn.turn_index:02d}",
            owner="earlier_same_coil_wire",
            turn_index=turn.turn_index,
            centerline_local_mm=tuple(
                tuple(map(float, point)) for point in points),
        ))
    return tuple(result)


def neighbor_prefill_copper(graph: PackingSupportGraph, spec: Any,
                            neighbor_side: int,
                            arc_step_deg: float = 5.0
                            ) -> tuple[CopperPolyline, ...]:
    """Transform the declared full neighbor coil into the active-tooth frame."""

    if neighbor_side not in (-1, 1):
        raise ValueError("neighbor_side must be -1 or +1")
    angle = neighbor_side * 2.0 * math.pi / int(spec.slots)
    c, s = math.cos(angle), math.sin(angle)
    result = []
    for turn in graph.turns:
        local = _loop_centerline(turn, spec, arc_step_deg=arc_step_deg)
        transformed = local.copy()
        transformed[:, 0] = c * local[:, 0] - s * local[:, 1]
        transformed[:, 1] = s * local[:, 0] + c * local[:, 1]
        result.append(CopperPolyline(
            obstacle_id=(
                f"neighbor-{neighbor_side:+d}-turn-{turn.turn_index:02d}"),
            owner="neighbor_side_prefill",
            turn_index=turn.turn_index,
            centerline_local_mm=tuple(
                tuple(map(float, point)) for point in transformed),
        ))
    return tuple(result)


@dataclass(frozen=True)
class SequentialRoutePolicy:
    """Explicit interpolation absent from the crossing-only source plan."""

    schema: str = "slot-sequential-route-policy/v1"
    angular_sample_step_deg: float = 5.0
    lead_in_half_turns: int = 1
    lead_out_half_turns: int = 1
    first_half_rule: str = "hold current placement profile"
    second_half_rule: str = (
        "shortest guarded withdraw-to-mouth/reposition/re-enter crossover")
    lead_in_rule: str = "hold placement 0"
    lead_out_rule: str = "hold placement 49"

    def validate(self) -> None:
        if self.schema != "slot-sequential-route-policy/v1":
            raise ValueError("unsupported sequential route policy")
        if (self.angular_sample_step_deg <= 0.0
                or 180.0 % self.angular_sample_step_deg > 1e-9):
            raise ValueError(
                "angular sample step must divide one 180-degree half-turn")
        if self.lead_in_half_turns != 1 or self.lead_out_half_turns != 1:
            raise ValueError("release policy requires one lead-in and lead-out")


@dataclass(frozen=True)
class SequentialLaySample:
    interval_index: int
    sample_index: int
    logical_phase_rad: float
    start_turn_index: int
    end_turn_index: int
    interpolation_fraction: float
    radial_mm: float
    profile_radius_mm: float
    target_local_mm: tuple[float, float, float]
    crossing_kind: str


@dataclass(frozen=True)
class MouthCrossover:
    start_turn_index: int
    end_turn_index: int
    safe_profile_radius_mm: float
    waypoints_radial_profile_mm: tuple[tuple[float, float], ...]
    total_length_mm: float
    minimum_prior_center_distance_mm: float
    planner_guard_mm: float


def _point_to_segments_2d(point: np.ndarray, starts: np.ndarray,
                           ends: np.ndarray) -> np.ndarray:
    vectors = ends - starts
    denominator = np.einsum("ij,ij->i", vectors, vectors)
    numerator = np.einsum("ij,ij->i", point - starts, vectors)
    fraction = np.divide(
        numerator, denominator, out=np.zeros_like(numerator),
        where=denominator > _EPS)
    fraction = np.clip(fraction, 0.0, 1.0)
    nearest = starts + fraction[:, None] * vectors
    return np.linalg.norm(nearest - point, axis=1)


def solve_safe_mouth_crossover(
    graph: PackingSupportGraph,
    start_turn_index: int,
    *,
    radial_min_mm: float | None = None,
    radial_max_mm: float = 20.68,
    maximum_profile_radius_mm: float = 3.0,
    planner_guard_mm: float = 0.0001,
) -> MouthCrossover:
    """Shortest guarded mouth-return path between consecutive placements.

    The start turn is the same continuous conductor and is therefore not an
    obstacle.  Every earlier turn is a closed deposited loop whose separation
    in this radial/profile state space is exactly its 3D nested-loop
    centerline separation.
    """

    start_turn = graph.turn(start_turn_index)
    end_turn = graph.turn(start_turn_index + 1)
    wire_d = float(graph.wire_diameter_mm)
    guard_radius = wire_d + float(planner_guard_mm)
    radial_min = (min(turn.radial_mm for turn in graph.turns) - 0.25
                  if radial_min_mm is None else float(radial_min_mm))
    domain = box(
        radial_min, graph.center_core_access_mm,
        float(radial_max_mm), float(maximum_profile_radius_mm))
    start = np.array((start_turn.radial_mm,
                      start_turn.profile_radius_mm), dtype=float)
    end = np.array((end_turn.radial_mm,
                    end_turn.profile_radius_mm), dtype=float)
    centers = np.array([
        (turn.radial_mm, turn.profile_radius_mm)
        for turn in graph.turns[:start_turn.turn_index]
    ], dtype=float)
    if not len(centers):
        return MouthCrossover(
            start_turn.turn_index, end_turn.turn_index,
            max(start[1], end[1]) + guard_radius,
            (tuple(map(float, start)), tuple(map(float, end))),
            float(np.linalg.norm(end - start)), math.inf,
            float(planner_guard_mm))

    obstacle = unary_union([
        Point(*map(float, center)).buffer(guard_radius, quad_segs=32)
        for center in centers
    ]).buffer(0)

    def endpoint_portals(endpoint: np.ndarray) -> list[np.ndarray]:
        point = Point(*map(float, endpoint))
        if not obstacle.covers(point):
            return [endpoint.copy()]
        polygons = (list(obstacle.geoms)
                    if obstacle.geom_type == "MultiPolygon" else [obstacle])
        component = max(
            (polygon for polygon in polygons if polygon.covers(point)),
            key=lambda polygon: polygon.area)
        coordinates = np.asarray(component.exterior.coords[:-1], dtype=float)
        representative = np.array(
            component.representative_point().coords[0], dtype=float)
        order = np.argsort(np.linalg.norm(coordinates - endpoint, axis=1))
        result = []
        for index in order:
            boundary = coordinates[index]
            outward = boundary - representative
            if np.linalg.norm(outward) <= _EPS:
                continue
            candidate = boundary + 1e-8 * _unit(outward)
            if (not domain.covers(Point(*map(float, candidate)))
                    or obstacle.covers(Point(*map(float, candidate)))):
                continue
            distances = _point_to_segments_2d(
                centers,
                np.repeat(endpoint[None, :], len(centers), axis=0),
                np.repeat(candidate[None, :], len(centers), axis=0),
            )
            if float(np.min(distances)) + 1e-9 < wire_d:
                continue
            result.append(candidate)
            if len(result) >= 32:
                break
        return result

    start_portals = endpoint_portals(start)
    end_portals = endpoint_portals(end)
    if not start_portals or not end_portals:
        raise RuntimeError(
            f"crossover {start_turn.turn_index}->{end_turn.turn_index} has "
            "no exact-clear endpoint portal")

    safe_height = max(
        float(np.max(centers[:, 1])), float(start[1]), float(end[1]))
    safe_height += guard_radius + 0.001
    while safe_height <= maximum_profile_radius_mm + 1e-9:
        proxy = obstacle.simplify(
            0.01, preserve_topology=True).buffer(
                0.010001, quad_segs=16)
        polygons = (list(proxy.geoms)
                    if proxy.geom_type == "MultiPolygon" else [proxy])
        boundary_nodes = np.array([
            coordinate for polygon in polygons
            for ring in (polygon.exterior, *polygon.interiors)
            for coordinate in list(ring.coords)[:-1]
        ], dtype=float)
        safe_nodes = np.column_stack((
            np.linspace(radial_min, radial_max_mm, 96),
            np.full(96, safe_height),
        ))
        nodes = np.vstack((
            start_portals, end_portals, safe_nodes, boundary_nodes))
        start_count, end_count = len(start_portals), len(end_portals)
        safe_first = start_count + end_count
        safe_last = safe_first + len(safe_nodes)
        ii, jj = np.triu_indices(len(nodes), 1)
        lines = shapely.linestrings(
            np.stack((nodes[ii], nodes[jj]), axis=1))
        visible = (
            shapely.relate_pattern(lines, obstacle, "F********")
            & shapely.covers(domain, lines)
        )
        left, right = ii[visible], jj[visible]
        weights = np.linalg.norm(nodes[left] - nodes[right], axis=1)
        adjacency = csr_matrix((
            np.concatenate((weights, weights)),
            (np.concatenate((left, right)),
             np.concatenate((right, left))),
        ), shape=(len(nodes), len(nodes)))
        start_distances, start_predecessors = shortest_path(
            adjacency, directed=False,
            indices=list(range(start_count)), return_predecessors=True)
        end_sources = list(range(start_count, start_count + end_count))
        end_distances, end_predecessors = shortest_path(
            adjacency, directed=False, indices=end_sources,
            return_predecessors=True)

        best = None
        for safe_node in range(safe_first, safe_last):
            totals = (start_distances[:, safe_node, None]
                      + end_distances[:, safe_node][None, :])
            start_choice, end_choice = np.unravel_index(
                int(np.argmin(totals)), totals.shape)
            value = float(totals[start_choice, end_choice])
            if math.isfinite(value) and (best is None or value < best[0]):
                best = (value, start_choice, end_choice, safe_node)
        if best is None:
            safe_height += 0.01
            continue

        _, start_choice, end_choice, safe_node = best

        def reconstruct(predecessors: np.ndarray, source: int,
                        target: int) -> list[int]:
            path = [target]
            cursor = target
            while cursor != source:
                cursor = int(predecessors[cursor])
                if cursor < 0:
                    raise RuntimeError("crossover predecessor chain broke")
                path.append(cursor)
            path.reverse()
            return path

        left_path = reconstruct(
            start_predecessors[start_choice], start_choice, safe_node)
        end_source = start_count + end_choice
        right_from_end = reconstruct(
            end_predecessors[end_choice], end_source, safe_node)
        right_path = list(reversed(right_from_end))
        path_indices = left_path + right_path[1:]
        points = [start, *[nodes[index] for index in path_indices], end]
        compact = [points[0]]
        for point in points[1:]:
            if np.linalg.norm(point - compact[-1]) > 1e-10:
                compact.append(point)

        minimum = math.inf
        for one, two in zip(compact, compact[1:]):
            distances = _point_to_segments_2d(
                centers,
                np.repeat(one[None, :], len(centers), axis=0),
                np.repeat(two[None, :], len(centers), axis=0),
            )
            minimum = min(minimum, float(np.min(distances)))
        if minimum + 1e-9 < wire_d:
            safe_height += 0.01
            continue
        return MouthCrossover(
            start_turn.turn_index, end_turn.turn_index,
            float(safe_height),
            tuple(tuple(map(float, point)) for point in compact),
            float(sum(np.linalg.norm(two - one)
                      for one, two in zip(compact, compact[1:]))),
            float(minimum), float(planner_guard_mm))

    raise RuntimeError(
        f"crossover {start_turn.turn_index}->{end_turn.turn_index} cannot "
        f"reach the {maximum_profile_radius_mm:.3f} mm flyer envelope")


def build_safe_mouth_crossovers(
    graph: PackingSupportGraph,
) -> tuple[MouthCrossover, ...]:
    """Solve every consecutive transition; fail before route serialization."""

    return tuple(
        solve_safe_mouth_crossover(graph, turn_index)
        for turn_index in range(len(graph.turns) - 1)
    )


def _interpolate_crossover(path: MouthCrossover,
                           fraction: float) -> np.ndarray:
    points = np.asarray(path.waypoints_radial_profile_mm, dtype=float)
    lengths = np.linalg.norm(points[1:] - points[:-1], axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    target = float(np.clip(fraction, 0.0, 1.0)) * cumulative[-1]
    segment = min(len(lengths) - 1,
                  int(np.searchsorted(cumulative, target, side="right") - 1))
    local = target - cumulative[segment]
    ratio = 0.0 if lengths[segment] <= _EPS else local / lengths[segment]
    return points[segment] + ratio * (points[segment + 1] - points[segment])


def _rounded_loop_yz(profile_radius_mm: float, phase_rad: float,
                     spec: Any, resolution: int = 36) -> np.ndarray:
    """Arc-length synchronized point on the rounded tooth-offset loop."""

    half_neck = max(2.5, float(spec.od) * 0.07) / 2.0
    half_stack = float(spec.stack) / 2.0
    profile = box(-half_neck, -half_stack, half_neck, half_stack).buffer(
        float(profile_radius_mm), quad_segs=int(resolution))
    line = LineString(profile.exterior.coords)
    phase_zero = Point(-half_neck - float(profile_radius_mm), half_stack)
    origin = float(line.project(phase_zero))
    distance = (origin
                + (float(phase_rad) % (2.0 * math.pi))
                / (2.0 * math.pi) * float(line.length)) % float(line.length)
    return np.asarray(line.interpolate(distance).coords[0], dtype=float)


def sequential_lay_samples(graph: PackingSupportGraph, spec: Any,
                           policy: SequentialRoutePolicy,
                           crossovers: tuple[MouthCrossover, ...] | None = None,
                           ) -> tuple[SequentialLaySample, ...]:
    """Materialize 100 intervals plus explicit lead-in and lead-out."""

    policy.validate()
    if len(graph.turns) != 50:
        raise ValueError("release sequential policy requires 50 turns")
    if crossovers is None:
        crossovers = build_safe_mouth_crossovers(graph)
    if (len(crossovers) != len(graph.turns) - 1
            or any(path.start_turn_index != index
                   or path.end_turn_index != index + 1
                   for index, path in enumerate(crossovers))):
        raise ValueError("safe-mouth crossover table is incomplete or unordered")
    divisions = round(180.0 / policy.angular_sample_step_deg)
    result = []
    # -1 is lead-in, 0..99 are the commanded half-turn intervals, and 100 is
    # the final lead-out hold.  Duplicate boundary samples are intentional:
    # they prove that adjacent interval policies meet at exactly one pose.
    for interval in range(-1, 101):
        if interval < 0:
            start_turn = end_turn = graph.turn(0)
            phase0 = -math.pi
            kind = "lead_in"
        elif interval >= 100:
            start_turn = end_turn = graph.turn(49)
            phase0 = 100.0 * math.pi
            kind = "lead_out"
        else:
            start_turn = graph.turn(interval // 2)
            end_turn = graph.turn(min(49, (interval + 1) // 2))
            phase0 = interval * math.pi
            kind = ("placement_hold" if start_turn.turn_index
                    == end_turn.turn_index else "declared_crossover")
        for sample_index in range(divisions + 1):
            fraction = sample_index / divisions
            phase = phase0 + fraction * math.pi
            if start_turn.turn_index == end_turn.turn_index:
                radial = start_turn.radial_mm
                profile = start_turn.profile_radius_mm
            else:
                state = _interpolate_crossover(
                    crossovers[start_turn.turn_index], fraction)
                radial, profile = map(float, state)
            yz = _rounded_loop_yz(profile, phase, spec)
            result.append(SequentialLaySample(
                interval_index=interval,
                sample_index=sample_index,
                logical_phase_rad=float(phase),
                start_turn_index=start_turn.turn_index,
                end_turn_index=end_turn.turn_index,
                interpolation_fraction=float(fraction),
                radial_mm=float(radial),
                profile_radius_mm=float(profile),
                target_local_mm=(float(radial), float(yz[0]), float(yz[1])),
                crossing_kind=kind,
            ))
    return tuple(result)


def classify_active_loop_contacts(graph: PackingSupportGraph,
                                  turn_index: int,
                                  tolerance_mm: float = 1e-8
                                  ) -> PackingContactAudit:
    """Classify exact nested-loop distances to every prior deposited turn."""

    active = graph.turn(turn_index)
    contacts = []
    minimum = math.inf
    ok = True
    for prior in graph.turns[:active.turn_index]:
        distance = math.hypot(
            active.radial_mm - prior.radial_mm,
            active.profile_radius_mm - prior.profile_radius_mm,
        )
        minimum = min(minimum, distance)
        declared = prior.turn_index in active.prior_contact_turn_indices
        parent = prior.turn_index in active.parent_turn_indices
        if distance < graph.wire_diameter_mm - tolerance_mm:
            classification = "overlap"
            ok = False
        elif abs(distance - graph.wire_diameter_mm) <= tolerance_mm:
            if parent:
                classification = "intended_parent_tangent"
            elif (declared and
                  prior.layer_index == active.layer_index):
                classification = "same_layer_tangent"
            elif declared:
                classification = "incidental_tangent"
            else:
                classification = "undeclared_tangent"
                ok = False
        else:
            classification = "clear"
            if declared:
                ok = False
        contacts.append(PackingLoopContact(
            prior_turn_index=prior.turn_index,
            centerline_distance_mm=distance,
            classification=classification,
        ))
    parent_hits = {
        contact.prior_turn_index for contact in contacts
        if contact.classification == "intended_parent_tangent"
    }
    progressive = (
        (active.support_kind == "slot_liner"
         and active.turn_index == 0
         and not active.parent_turn_indices)
        or (active.support_kind == "deposited_wire"
            and bool(active.parent_turn_indices)
            and set(active.parent_turn_indices).issubset(parent_hits))
    )
    ok = ok and progressive
    return PackingContactAudit(
        ok=ok,
        progressive_support_validated=progressive,
        minimum_prior_centerline_distance_mm=(
            None if not contacts else float(minimum)),
        contacts=tuple(contacts),
        reason="ok" if ok else "deposited-loop contact contract failed",
    )


@dataclass(frozen=True)
class _TipMeta:
    analytic_length_mm: float
    exit_tangent_error_deg: float
    arc_turn_deg: float


@dataclass
class _SectionPlan:
    obstacle: Any
    nodes: np.ndarray
    adjacency: csr_matrix
    distances: np.ndarray
    predecessors: np.ndarray

    def path(self, source: int, target: int) -> list[int]:
        if source == target:
            return [source]
        result = [target]
        cursor = target
        while cursor != source:
            cursor = int(self.predecessors[source, cursor])
            if cursor < 0:
                raise RuntimeError("visibility graph path is disconnected")
            result.append(cursor)
        result.reverse()
        return result


def _visibility_plan(obstacle: Any, *, buffer_resolution: int,
                     visibility_chord_mm: float) -> _SectionPlan:
    """Build the reusable visibility graph for one exact 2D obstacle."""

    node_boundary = obstacle
    if visibility_chord_mm > 0.0:
        node_boundary = obstacle.simplify(
            visibility_chord_mm, preserve_topology=True).buffer(
                visibility_chord_mm + 1e-6,
                quad_segs=buffer_resolution)
    polygons = (list(node_boundary.geoms)
                if node_boundary.geom_type == "MultiPolygon"
                else [node_boundary])
    nodes = np.array([
        coordinate
        for polygon in polygons
        for ring in (polygon.exterior, *polygon.interiors)
        for coordinate in list(ring.coords)[:-1]
    ], dtype=float)
    if not len(nodes):
        raise RuntimeError("visibility obstacle has no boundary nodes")
    ii, jj = np.triu_indices(len(nodes), 1)
    lines = shapely.linestrings(np.stack((nodes[ii], nodes[jj]), axis=1))
    visible = shapely.relate_pattern(lines, obstacle, "F********")
    left, right = ii[visible], jj[visible]
    weights = np.linalg.norm(nodes[left] - nodes[right], axis=1)
    adjacency = csr_matrix((
        np.concatenate((weights, weights)),
        (np.concatenate((left, right)), np.concatenate((right, left))),
    ), shape=(len(nodes), len(nodes)))
    distances, predecessors = shortest_path(
        adjacency, directed=False, return_predecessors=True)
    return _SectionPlan(
        obstacle, nodes, adjacency, np.asarray(distances),
        np.asarray(predecessors))


@dataclass(frozen=True)
class _ActiveGoal:
    """Planner portal and independently checked physical support endpoint."""

    portal: np.ndarray
    terminal: np.ndarray
    endpoint_support: str
    raw_portal: np.ndarray
    portal_nudge_mm: float


def _unit(vector: np.ndarray) -> np.ndarray:
    length = float(np.linalg.norm(vector))
    if length < _EPS:
        raise ValueError("zero-length vector")
    return np.asarray(vector, dtype=float) / length


def rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))


def _circle_tangents(point: np.ndarray, center: np.ndarray,
                     radius: float) -> tuple[np.ndarray, np.ndarray]:
    delta = np.asarray(point, dtype=float) - np.asarray(center, dtype=float)
    distance2 = float(np.dot(delta, delta))
    if distance2 <= radius * radius:
        raise ValueError("tangent source lies inside contact circle")
    base = center + delta * (radius * radius / distance2)
    perpendicular = np.array((-delta[1], delta[0]))
    offset = radius * math.sqrt(distance2 - radius * radius) / distance2
    return base + perpendicular * offset, base - perpendicular * offset


def _tip_path(feed: np.ndarray, target: np.ndarray, guide: dict[str, Any],
              guide_wire_radius: float, rotation: np.ndarray,
              arc_step_deg: float = 2.0) -> tuple[np.ndarray, _TipMeta]:
    """Analytically tangent torus path; arc points are display geometry."""

    center = rotation @ np.asarray(guide["center_local_mm"], dtype=float)
    axis = _unit(rotation @ np.asarray(guide["axis_local"], dtype=float))
    feed = np.asarray(feed, dtype=float)
    target = np.asarray(target, dtype=float)
    feed_rel = feed - center
    feed_axial = float(np.dot(feed_rel, axis))
    if np.linalg.norm(feed_rel - feed_axial * axis) > 1e-6:
        raise ValueError("tip-guide feed is not on the torus axis")

    target_rel = target - center
    target_axial = float(np.dot(target_rel, axis))
    transverse = target_rel - target_axial * axis
    target_rho = float(np.linalg.norm(transverse))
    if target_rho < 1e-9:
        seed = np.array((1.0, 0.0, 0.0))
        if abs(float(np.dot(seed, axis))) > 0.9:
            seed = np.array((0.0, 0.0, 1.0))
        meridian = _unit(seed - np.dot(seed, axis) * axis)
    else:
        meridian = transverse / target_rho

    major = float(guide["major_radius_mm"])
    radius = float(guide["tube_radius_mm"]) + guide_wire_radius
    circle = np.array((0.0, major))
    entry = max(
        (point for point in _circle_tangents(
            np.array((feed_axial, 0.0)), circle, radius)
         if point[1] < major),
        key=lambda point: point[0],
    )
    theta_entry = math.atan2(entry[1] - major, entry[0])

    def front_delta(point: np.ndarray) -> float:
        theta = math.atan2(point[1] - major, point[0])
        ccw = (theta - theta_entry) % (2.0 * math.pi)
        contains_front = ((-theta_entry) % (2.0 * math.pi)) <= ccw + 1e-12
        return ccw if contains_front else ccw - 2.0 * math.pi

    exits = [point for point in _circle_tangents(
        np.array((target_axial, target_rho)), circle, radius)
             if point[1] > major]
    if not exits:
        raise ValueError("tip torus has no outer exit tangent")
    exit_point = min(exits, key=lambda point: abs(front_delta(point)))
    turn = front_delta(exit_point)
    count = max(2, math.ceil(abs(math.degrees(turn)) / arc_step_deg))
    arc = np.array([
        circle + radius * np.array((
            math.cos(theta_entry + turn * index / count),
            math.sin(theta_entry + turn * index / count),
        ))
        for index in range(count + 1)
    ])

    def world(point: np.ndarray) -> np.ndarray:
        return center + point[0] * axis + point[1] * meridian

    points = np.vstack((feed, world(entry),
                        np.array([world(point) for point in arc[1:]]),
                        target))
    radial = exit_point - circle
    span = np.array((target_axial, target_rho)) - exit_point
    tangent_error = math.degrees(math.asin(min(
        1.0, abs(float(np.dot(_unit(radial), _unit(span))))
    )))
    analytic_length = (
        float(np.linalg.norm(np.array((feed_axial, 0.0)) - entry))
        + abs(turn) * radius
        + float(np.linalg.norm(span))
    )
    return points, _TipMeta(
        analytic_length_mm=analytic_length,
        exit_tangent_error_deg=tangent_error,
        arc_turn_deg=abs(math.degrees(turn)),
    )


def _coupled_tip_arrays(targets: np.ndarray, guide: dict[str, Any],
                        guide_wire_radius: float,
                        rotation: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Vectorized analytic torus exits and exact path lengths."""

    targets = np.asarray(targets, dtype=float)
    center = rotation @ np.asarray(guide["center_local_mm"], dtype=float)
    axis = _unit(rotation @ np.asarray(guide["axis_local"], dtype=float))
    feed = rotation @ np.asarray(guide["feed_local_mm"], dtype=float)
    feed_axial = float(np.dot(feed - center, axis))
    major = float(guide["major_radius_mm"])
    radius = float(guide["tube_radius_mm"]) + guide_wire_radius
    circle = np.array((0.0, major))
    entry = max(
        (point for point in _circle_tangents(
            np.array((feed_axial, 0.0)), circle, radius)
         if point[1] < major),
        key=lambda point: point[0],
    )
    theta_entry = math.atan2(entry[1] - major, entry[0])

    relative = targets - center
    axial = relative @ axis
    transverse = relative - axial[:, None] * axis
    rho = np.linalg.norm(transverse, axis=1)
    meridian = transverse / rho[:, None]
    delta = np.column_stack((axial, rho)) - circle
    distance2 = np.einsum("ij,ij->i", delta, delta)
    base = circle + delta * (radius * radius / distance2)[:, None]
    perpendicular = np.column_stack((-delta[:, 1], delta[:, 0]))
    offset = radius * np.sqrt(distance2 - radius * radius) / distance2
    options = np.stack((
        base + perpendicular * offset[:, None],
        base - perpendicular * offset[:, None],
    ), axis=1)
    scores = np.full((len(targets), 2), np.inf)
    turns = np.zeros_like(scores)
    front_zero = (-theta_entry) % (2.0 * math.pi)
    for option in range(2):
        theta = np.arctan2(options[:, option, 1] - major,
                           options[:, option, 0])
        ccw = (theta - theta_entry) % (2.0 * math.pi)
        turn = np.where(front_zero <= ccw + 1e-12,
                        ccw, ccw - 2.0 * math.pi)
        valid = options[:, option, 1] > major
        scores[:, option] = np.where(valid, np.abs(turn), np.inf)
        turns[:, option] = turn
    choice = np.argmin(scores, axis=1)
    rows = np.arange(len(targets))
    exits_2d = options[rows, choice]
    turn = turns[rows, choice]
    exits = (center + exits_2d[:, 0, None] * axis
             + exits_2d[:, 1, None] * meridian)
    lengths = (
        np.linalg.norm(np.array((feed_axial, 0.0)) - entry)
        + radius * np.abs(turn)
        + np.linalg.norm(np.column_stack((axial, rho)) - exits_2d, axis=1)
    )
    return exits, lengths


def _tooth_tangents(tip: np.ndarray, contact: dict[str, Any]
                    ) -> dict[int, np.ndarray]:
    point = np.asarray(tip, dtype=float)[:2]
    a = float(contact["physical_tangential_radius_mm"])
    b = float(contact["physical_axial_radius_mm"])
    offset = float(contact["wire_offset_radius_mm"])

    def value(theta: float) -> float:
        c, s = math.cos(theta), math.sin(theta)
        body = np.array((a * c, b * s))
        normal = _unit(np.array((c / a, s / b)))
        return float(np.dot(normal, point - body) - offset)

    roots: list[float] = []
    grid = np.linspace(0.0, 2.0 * math.pi, 721)
    values = [value(theta) for theta in grid]
    for lo0, hi0, flo0, fhi0 in zip(grid, grid[1:], values, values[1:]):
        if flo0 * fhi0 > 0.0:
            continue
        lo, hi, flo = float(lo0), float(hi0), float(flo0)
        for _ in range(45):
            mid = (lo + hi) / 2.0
            fmid = value(mid)
            if flo * fmid <= 0.0:
                hi = mid
            else:
                lo, flo = mid, fmid
        roots.append((lo + hi) / 2.0)

    result: dict[int, np.ndarray] = {}
    for theta in roots:
        c, s = math.cos(theta), math.sin(theta)
        body = np.array((a * c, b * s))
        tangent = body + offset * _unit(np.array((c / a, s / b)))
        line = tangent - point
        cross = line[0] * (-point[1]) - line[1] * (-point[0])
        side = 1 if cross >= 0.0 else -1
        result[side] = np.array((tangent[0], tangent[1],
                                 float(contact["z_mm"])))
    if set(result) != {-1, 1}:
        raise RuntimeError("failed to construct both tooth support tangents")
    return result


def _tooth_target(tip: np.ndarray, contact: dict[str, Any],
                  motion_sign: int) -> np.ndarray:
    return _tooth_tangents(tip, contact)[-1 if motion_sign >= 0 else 1]


def _point_triangle_distance(point: np.ndarray, a: np.ndarray,
                             b: np.ndarray, c: np.ndarray) -> float:
    ab, ac, ap = b - a, c - a, point - a
    d1, d2 = float(np.dot(ab, ap)), float(np.dot(ac, ap))
    if d1 <= 0.0 and d2 <= 0.0:
        return float(np.linalg.norm(ap))
    bp = point - b
    d3, d4 = float(np.dot(ab, bp)), float(np.dot(ac, bp))
    if d3 >= 0.0 and d4 <= d3:
        return float(np.linalg.norm(bp))
    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        return float(np.linalg.norm(point - (a + d1 / (d1 - d3) * ab)))
    cp = point - c
    d5, d6 = float(np.dot(ab, cp)), float(np.dot(ac, cp))
    if d6 >= 0.0 and d5 <= d6:
        return float(np.linalg.norm(cp))
    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        return float(np.linalg.norm(point - (a + d2 / (d2 - d6) * ac)))
    va = d3 * d6 - d5 * d4
    if va <= 0.0 and d4 - d3 >= 0.0 and d5 - d6 >= 0.0:
        ratio = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return float(np.linalg.norm(point - (b + ratio * (c - b))))
    inverse = 1.0 / (va + vb + vc)
    closest = a + vb * inverse * ab + vc * inverse * ac
    return float(np.linalg.norm(point - closest))


def _segment_segment_distance(p1: np.ndarray, q1: np.ndarray,
                              p2: np.ndarray, q2: np.ndarray) -> float:
    u, v, w = q1 - p1, q2 - p2, p1 - p2
    a, b, c = float(u @ u), float(u @ v), float(v @ v)
    d, e = float(u @ w), float(v @ w)
    denominator = a * c - b * b
    s_denominator = t_denominator = denominator
    if denominator < _EPS:
        s_numerator, s_denominator = 0.0, 1.0
        t_numerator, t_denominator = e, c
    else:
        s_numerator, t_numerator = b * e - c * d, a * e - b * d
        if s_numerator < 0.0:
            s_numerator, t_numerator, t_denominator = 0.0, e, c
        elif s_numerator > s_denominator:
            s_numerator, t_numerator, t_denominator = s_denominator, e + b, c
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
    sc = 0.0 if abs(s_numerator) < _EPS else s_numerator / s_denominator
    tc = 0.0 if abs(t_numerator) < _EPS else t_numerator / t_denominator
    return float(np.linalg.norm(w + sc * u - tc * v))


def segment_triangle_distance(start: np.ndarray, end: np.ndarray,
                              triangle: np.ndarray) -> float:
    """Exact Euclidean distance between a finite segment and triangle."""

    a, b, c = triangle
    direction, edge1, edge2 = end - start, b - a, c - a
    h = np.cross(direction, edge2)
    determinant = float(np.dot(edge1, h))
    if abs(determinant) > _EPS:
        inverse = 1.0 / determinant
        delta = start - a
        u = inverse * float(np.dot(delta, h))
        cross = np.cross(delta, edge1)
        v = inverse * float(np.dot(direction, cross))
        distance = inverse * float(np.dot(edge2, cross))
        if (-_EPS <= u <= 1.0 + _EPS and v >= -_EPS
                and u + v <= 1.0 + _EPS
                and -_EPS <= distance <= 1.0 + _EPS):
            return 0.0
    return min(
        _point_triangle_distance(start, a, b, c),
        _point_triangle_distance(end, a, b, c),
        _segment_segment_distance(start, end, a, b),
        _segment_segment_distance(start, end, b, c),
        _segment_segment_distance(start, end, c, a),
    )


def exact_polyline_mesh_clearance(points: np.ndarray, mesh: trimesh.Trimesh,
                                  search_band_mm: float = 0.5
                                  ) -> tuple[float, int | None, int | None]:
    """Exact segment/triangle minimum with a proved AABB omission bound."""

    triangles = mesh.triangles
    tree = mesh.triangles_tree
    best, best_segment, best_triangle = search_band_mm, None, None
    for index, (start, end) in enumerate(zip(points, points[1:])):
        lower = np.minimum(start, end) - search_band_mm
        upper = np.maximum(start, end) + search_band_mm
        triangle_ids = sorted(tree.intersection((*lower, *upper)))
        for triangle_id in triangle_ids:
            distance = segment_triangle_distance(
                start, end, triangles[triangle_id])
            if distance < best:
                best = distance
                best_segment = index
                best_triangle = int(triangle_id)
                if best <= _EPS:
                    return best, best_segment, best_triangle
    return best, best_segment, best_triangle


def exact_polyline_part_clearance(points: np.ndarray, part: Any) -> float:
    """Exact OpenCascade distance from a polyline to source CAD."""

    route = np.asarray(points, dtype=float)
    if (route.ndim != 2 or route.shape[1] != 3 or len(route) < 2
            or not np.all(np.isfinite(route))):
        raise ValueError("points must be a finite 3D polyline")
    edges = [
        Edge.make_line(
            Vector(*map(float, start)), Vector(*map(float, end)))
        for start, end in zip(route, route[1:])
        if np.linalg.norm(end - start) > _EPS
    ]
    if not edges:
        raise ValueError("polyline has no nonzero segments")
    return float(part.distance_to(Compound(children=edges)))


def machine_stator_mesh_to_local(mesh: trimesh.Trimesh,
                                 home_standoff_mm: float) -> trimesh.Trimesh:
    """Copy a reference-pose exported stator mesh into stator-local axes."""

    result = mesh.copy()
    vertices = result.vertices.copy()
    result.vertices = np.column_stack((
        home_standoff_mm - vertices[:, 2], -vertices[:, 0], vertices[:, 1],
    ))
    return result


class SlotRoutePlanner:
    """Reusable coupled planner for one stator and access contract."""

    def __init__(self, *, spec: Any, stator_part: Any,
                 stator_mesh_local: trimesh.Trimesh,
                 guide: dict[str, Any], contact: dict[str, Any],
                 guide_wire_radius_mm: float = 0.25,
                 access_radius_mm: float,
                 planner_offset_mm: float,
                 buffer_resolution: int = 16,
                 mesh_search_band_mm: float = 0.5,
                 clamp_goal_to_stack: bool = False,
                 visibility_chord_mm: float = 0.01):
        if planner_offset_mm < access_radius_mm:
            raise ValueError("planner offset must cover the access radius")
        if mesh_search_band_mm <= planner_offset_mm:
            raise ValueError("mesh search band must exceed planner offset")
        self.spec = spec
        self.stator_part = stator_part
        self.mesh = stator_mesh_local
        self._mesh_query = trimesh.proximity.ProximityQuery(self.mesh)
        self.guide = guide
        self.contact = contact
        self.guide_wire_radius_mm = float(guide_wire_radius_mm)
        self.access_radius_mm = float(access_radius_mm)
        self.planner_offset_mm = float(planner_offset_mm)
        self.buffer_resolution = int(buffer_resolution)
        self.mesh_search_band_mm = float(mesh_search_band_mm)
        self.clamp_goal_to_stack = bool(clamp_goal_to_stack)
        self.visibility_chord_mm = float(visibility_chord_mm)
        self.neck_half_mm = max(2.5, float(spec.od) * 0.07) / 2.0
        self._sections: dict[float, _SectionPlan] = {}
        self._dilated_solids: tuple[Any, ...] | None = None
        self._active_neck_seed: _SectionPlan | None = None

    @classmethod
    def from_project(cls, manifest: dict[str, Any], *, spec: Any = None,
                     coil: dict[str, Any] | None = None,
                     access_radius_mm: float | None = None,
                     planner_offset_mm: float | None = None,
                     buffer_resolution: int = 16,
                     clamp_goal_to_stack: bool = False,
                     visibility_chord_mm: float = 0.01) -> "SlotRoutePlanner":
        """Construct from the checked project manifest and source stator CAD."""

        machine_root = Path(__file__).resolve().parent.parent
        cad_root = machine_root / "cad"
        if str(cad_root) not in sys.path:
            sys.path.insert(0, str(cad_root))
        from params import DEFAULT_STATOR  # local project import
        import coil_growth
        import stator_model
        import wire_geometry

        spec = DEFAULT_STATOR if spec is None else spec
        coil = coil_growth.analyze_job(spec) if coil is None else coil
        manifest_stator = manifest.get("stator")
        if not isinstance(manifest_stator, dict):
            raise ValueError("manifest is missing its stator contract")
        comparisons = {
            "od": float(spec.od), "stack": float(spec.stack),
            "slots": int(spec.slots), "shaft_d": float(spec.shaft_d),
            "wire_d": float(spec.wire_d), "turns": int(spec.turns),
        }
        mismatches = []
        for name, expected in comparisons.items():
            actual = manifest_stator.get(name)
            if actual is None or not math.isclose(
                    float(actual), float(expected), rel_tol=0.0, abs_tol=1e-9):
                mismatches.append(f"{name}: manifest={actual}, source={expected}")
        if mismatches:
            raise ValueError("manifest/spec mismatch: " + "; ".join(mismatches))

        stator_part = stator_model.stator(spec)
        vertices, faces = stator_part.tessellate(0.01, 0.03)
        mesh = trimesh.Trimesh(
            vertices=np.array([(v.X, v.Y, v.Z) for v in vertices]),
            faces=np.asarray(faces), process=True)
        if not mesh.is_watertight:
            raise ValueError("source stator tessellation is not watertight")
        job_radius = float(spec.wire_d) / 2.0
        required_gap = float(
            coil["slot_access"]["required_neck_gap_mm"])
        edge_allowance = (required_gap - float(spec.wire_d)) / 2.0
        if edge_allowance < -1e-9:
            raise ValueError("coil report has a negative edge allowance")
        access = (job_radius + edge_allowance
                  if access_radius_mm is None else float(access_radius_mm))
        offset = (access if planner_offset_mm is None
                  else float(planner_offset_mm))
        return cls(
            spec=spec, stator_part=stator_part,
            stator_mesh_local=mesh, guide=wire_geometry.tip_guide_spec(),
            contact=wire_geometry.tooth_contact_spec(spec, coil),
            access_radius_mm=access, planner_offset_mm=offset,
            buffer_resolution=buffer_resolution,
            clamp_goal_to_stack=clamp_goal_to_stack,
            visibility_chord_mm=visibility_chord_mm,
        )

    def _section_plan(self, radial_x_mm: float) -> _SectionPlan:
        key = round(float(radial_x_mm), 9)
        cached = self._sections.get(key)
        if cached is not None:
            return cached
        if self._dilated_solids is None:
            self._dilated_solids = tuple(
                solid.offset_3d(
                    [], self.planner_offset_mm, kind=Kind.ARC)
                for solid in self.stator_part.solids()
            )
        sliced_faces = []
        for solid in self._dilated_solids:
            sliced_faces.extend(section(
                solid, section_by=Plane.YZ.offset(
                    float(radial_x_mm))).faces())
        triangles_2d: list[Polygon] = []
        for face in sliced_faces:
            vertices, triangles = face.tessellate(0.01, 0.05)
            coordinates = np.array([(v.Y, v.Z) for v in vertices])
            triangles_2d.extend(
                Polygon(coordinates[list(index)]) for index in triangles)
        if not triangles_2d:
            raise RuntimeError("dilated stator has no radial section")
        # offset_3d is the exact solid/sphere configuration-space boundary;
        # the tiny cleanup is topological only, not an added design margin.
        obstacle = unary_union(triangles_2d).buffer(0)
        result = _visibility_plan(
            obstacle, buffer_resolution=self.buffer_resolution,
            visibility_chord_mm=self.visibility_chord_mm)
        self._sections[key] = result
        return result

    def _active_neck_seed_plan(self) -> _SectionPlan:
        """Fast seed graph; the full 3D core remains the pass/fail authority."""

        if self._active_neck_seed is None:
            half_neck = max(2.5, float(self.spec.od) * 0.07) / 2.0
            half_stack = float(self.spec.stack) / 2.0
            obstacle = box(
                -half_neck, -half_stack, half_neck, half_stack,
            ).buffer(
                self.planner_offset_mm,
                quad_segs=self.buffer_resolution,
            )
            self._active_neck_seed = _visibility_plan(
                obstacle, buffer_resolution=self.buffer_resolution,
                visibility_chord_mm=self.visibility_chord_mm)
        return self._active_neck_seed

    def _point_core_distance(self, point: np.ndarray) -> float:
        return float(self._mesh_query.on_surface(
            np.asarray(point, dtype=float)[None, :])[1][0])

    def _active_goal(self, plan: _SectionPlan, radial: float,
                     desired: np.ndarray, side: int,
                     support_radius_mm: float
                     ) -> _ActiveGoal:
        """Return planner portal plus an independent physical support goal."""

        polygons = (list(plan.obstacle.geoms)
                    if plan.obstacle.geom_type == "MultiPolygon"
                    else [plan.obstacle])
        # At winding radii the presented tooth is the unique dilated section
        # containing the local y/z origin.  This remains true around its
        # rounded axial end even when the requested final-pack tangent lies
        # outside the steel stack.
        origin = Point(0.0, 0.0)
        active = next((polygon for polygon in polygons
                       if polygon.covers(origin)), None)
        if active is None:
            raise RuntimeError(
                "expanded active-tooth component does not contain y=0")
        _, boundary = nearest_points(Point(*map(float, desired)),
                                     active.boundary)
        raw_portal = np.array(boundary.coords[0], dtype=float)

        # GEOS can classify the coordinate returned by ``nearest_points`` as
        # infinitesimally *inside* its source polygon.  A line beginning at
        # that coordinate then has no visibility edges even though its
        # boundary distance is at floating-point zero.  Move the planning
        # portal a scale-aware number of ULPs toward the requested exterior
        # point.  The original boundary remains explicit in metadata and the
        # complete 3D route still has to pass the exact mesh postcheck below.
        bounds = np.asarray(plan.obstacle.bounds, dtype=float)
        coordinate_scale = max(
            1.0,
            abs(float(radial)),
            float(np.linalg.norm(desired)),
            float(np.max(np.abs(bounds))),
        )
        base_nudge = max(
            1e-12,
            256.0 * float(np.spacing(coordinate_scale)),
        )
        directions: list[np.ndarray] = []
        for vector in (
            np.asarray(desired, dtype=float) - raw_portal,
            raw_portal - np.array(active.representative_point().coords[0]),
        ):
            if np.linalg.norm(vector) > _EPS:
                unit = _unit(vector)
                if not any(np.linalg.norm(unit - prior) < 1e-9
                           for prior in directions):
                    directions.append(unit)
        portal = None
        portal_nudge = None
        max_nudge = max(1e-8, coordinate_scale * 1e-8)
        for direction in directions:
            nudge = base_nudge
            while nudge <= max_nudge:
                candidate = raw_portal + nudge * direction
                if not plan.obstacle.covers(
                        Point(*map(float, candidate))):
                    portal = candidate
                    portal_nudge = float(np.linalg.norm(
                        candidate - raw_portal))
                    break
                nudge *= 2.0
            if portal is not None:
                break
        if portal is None or portal_nudge is None:
            raise RuntimeError(
                "could not place active support portal strictly outside "
                "the planner obstacle")
        if side * portal[0] < -1e-7:
            raise RuntimeError("nearest active support has wrong trailing side")
        if support_radius_mm > self.planner_offset_mm + 1e-9:
            raise RuntimeError(
                "support profile outside the cached planner obstacle is "
                "unsupported without an explicit deposited-layer profile")
        if abs(support_radius_mm - self.planner_offset_mm) <= 1e-9:
            return _ActiveGoal(
                portal, portal.copy(), "slot_liner_glide",
                raw_portal, portal_nudge)

        portal_3d = np.array((radial, portal[0], portal[1]))
        nearest = self._mesh_query.on_surface(portal_3d[None, :])[0][0]
        direction = portal - nearest[1:]
        if np.linalg.norm(direction) < 1e-9:
            direction = portal - np.array(active.representative_point().coords[0])
        direction = _unit(direction)
        # Find an inside bracket along the same local support normal, then
        # solve the exact mesh distance for the requested independent radius.
        low = portal.copy()
        for distance in np.linspace(0.0, self.planner_offset_mm * 2.0, 65):
            candidate = portal - distance * direction
            if (self._point_core_distance(
                    (radial, candidate[0], candidate[1]))
                    <= support_radius_mm):
                low = candidate
                break
        else:
            raise RuntimeError("could not bracket the physical support profile")
        high = portal.copy()
        for _ in range(55):
            mid = (low + high) / 2.0
            value = self._point_core_distance((radial, mid[0], mid[1]))
            if value < support_radius_mm:
                low = mid
            else:
                high = mid
        terminal = high
        if side * terminal[0] < -1e-7:
            raise RuntimeError("physical support goal changed trailing side")
        return _ActiveGoal(
            portal, terminal, "slot_liner_glide",
            raw_portal, portal_nudge)

    def _explicit_core_goal(
        self,
        plan: _SectionPlan,
        terminal_yz_mm: np.ndarray,
        *,
        allow_liner_glide: bool,
    ) -> _ActiveGoal:
        """Resolve an explicit lay point against the guarded core section.

        A physical liner-supported point is allowed to sit between the exact
        access radius and the slightly larger numerical planning shell.  No
        other explicit target may be hidden inside that shell: deposited-wire
        support is handled by the separate copper obstacle contract.
        """

        terminal = np.asarray(terminal_yz_mm, dtype=float)
        if terminal.shape != (2,) or not np.all(np.isfinite(terminal)):
            raise RuntimeError("explicit terminal must be one finite yz point")
        point = Point(*map(float, terminal))
        if not plan.obstacle.covers(point):
            return _ActiveGoal(
                terminal.copy(), terminal.copy(), "free_explicit_target",
                terminal.copy(), 0.0)
        if not allow_liner_glide:
            raise RuntimeError(
                "explicit deposited-wire target lies inside the guarded "
                "bare-core obstacle")

        polygons = (list(plan.obstacle.geoms)
                    if plan.obstacle.geom_type == "MultiPolygon"
                    else [plan.obstacle])
        active = next((polygon for polygon in polygons
                       if polygon.covers(Point(0.0, 0.0))), None)
        if active is None:
            raise RuntimeError(
                "guarded active-tooth component does not contain yz origin")
        _, boundary = nearest_points(point, active.boundary)
        raw = np.array(boundary.coords[0], dtype=float)
        outward = raw - np.array(active.representative_point().coords[0])
        if np.linalg.norm(outward) <= _EPS:
            raise RuntimeError("explicit liner portal has no outward normal")
        outward = _unit(outward)
        bounds = np.asarray(plan.obstacle.bounds, dtype=float)
        scale = max(1.0, float(np.max(np.abs(bounds))))
        nudge = max(1e-12, 256.0 * float(np.spacing(scale)))
        maximum = max(1e-8, scale * 1e-8)
        portal = raw.copy()
        while nudge <= maximum:
            candidate = raw + nudge * outward
            if not plan.obstacle.covers(
                    Point(*map(float, candidate))):
                portal = candidate
                break
            nudge *= 2.0
        else:
            raise RuntimeError(
                "could not place explicit liner portal outside core shell")
        return _ActiveGoal(
            portal, terminal.copy(), "slot_liner_glide", raw,
            float(np.linalg.norm(portal - raw)))

    def _explicit_combined_goal(
        self,
        plan: _SectionPlan,
        terminal_yz_mm: np.ndarray,
        endpoint_support: str,
    ) -> _ActiveGoal:
        """Place a guarded portal outside a core-plus-copper obstacle union."""

        terminal = np.asarray(terminal_yz_mm, dtype=float)
        point = Point(*map(float, terminal))
        if not plan.obstacle.covers(point):
            return _ActiveGoal(
                terminal.copy(), terminal.copy(), endpoint_support,
                terminal.copy(), 0.0)
        polygons = (list(plan.obstacle.geoms)
                    if plan.obstacle.geom_type == "MultiPolygon"
                    else [plan.obstacle])
        containing = [polygon for polygon in polygons if polygon.covers(point)]
        if not containing:
            raise RuntimeError(
                "combined obstacle covers target without a polygon component")
        component = max(containing, key=lambda polygon: polygon.area)
        _, boundary = nearest_points(point, component.boundary)
        raw = np.array(boundary.coords[0], dtype=float)
        outward = raw - np.array(component.representative_point().coords[0])
        if np.linalg.norm(outward) <= _EPS:
            raise RuntimeError("combined support portal has no outward normal")
        outward = _unit(outward)
        bounds = np.asarray(plan.obstacle.bounds, dtype=float)
        scale = max(1.0, float(np.max(np.abs(bounds))))
        nudge = max(1e-12, 256.0 * float(np.spacing(scale)))
        maximum = max(1e-8, scale * 1e-8)
        portal = raw.copy()
        while nudge <= maximum:
            candidate = raw + nudge * outward
            if not plan.obstacle.covers(
                    Point(*map(float, candidate))):
                portal = candidate
                break
            nudge *= 2.0
        else:
            raise RuntimeError(
                "could not place combined portal outside obstacle union")
        return _ActiveGoal(
            portal, terminal.copy(), endpoint_support, raw,
            float(np.linalg.norm(portal - raw)))

    def route_explicit_core_target(
        self,
        radial_x_mm: float,
        flyer_angle_rad: float,
        terminal_yz_mm: tuple[float, float] | np.ndarray,
        *,
        endpoint_support: str,
        support_predecessor_indices: tuple[int, ...] = (),
        seed_with_active_neck: bool = False,
        copper_field: CopperField | None = None,
        support_copper_field: CopperField | None = None,
        copper_center_clearance_mm: float | None = None,
        declared_support_centerline_distances_mm: tuple[float, ...] = (),
        stator_reference_radial_x_mm: float | None = None,
    ) -> RouteResult:
        """Route to one declared lay point while checking the complete core.

        This is the bare-core half of progressive routing.  It deliberately
        refuses deposited-wire support until a caller supplies and validates
        the corresponding copper field; returning a plausible core-only path
        with ``progressive_support_validated=True`` would be unsafe.
        """

        if endpoint_support not in (
                "slot_liner", "earlier_same_coil_wire", "free_space"):
            return RouteResult(
                False, "unmodeled explicit endpoint support",
                endpoint_support=endpoint_support,
                progressive_support_validated=False)
        if (endpoint_support == "slot_liner"
                and support_predecessor_indices):
            return RouteResult(
                False, "liner support cannot name wire predecessors",
                endpoint_support=endpoint_support,
                progressive_support_validated=False)
        if (endpoint_support == "earlier_same_coil_wire"
                and not support_predecessor_indices):
            return RouteResult(
                False, "deposited-wire support has no predecessor",
                endpoint_support=endpoint_support,
                progressive_support_validated=False)
        if endpoint_support == "free_space" and support_predecessor_indices:
            return RouteResult(
                False, "free-space endpoint cannot name wire predecessors",
                endpoint_support=endpoint_support,
                progressive_support_validated=False)

        radial = float(radial_x_mm)
        try:
            terminal_3d = np.array((
                radial, float(terminal_yz_mm[0]),
                float(terminal_yz_mm[1])), dtype=float)
            base_plan = (self._active_neck_seed_plan()
                         if seed_with_active_neck
                         else self._section_plan(radial))
            support_field = (copper_field if support_copper_field is None
                             else support_copper_field)
            copper_required = (None if copper_field is None else float(
                copper_center_clearance_mm
                if copper_center_clearance_mm is not None else 0.0))
            if copper_field is not None and copper_required <= 0.0:
                raise RuntimeError(
                    "copper field requires a positive center clearance")

            support_validated = False
            terminal_copper_distances: dict[str, float] = {}
            if copper_field is not None:
                if support_field is None:
                    raise RuntimeError("copper route has no support field")
                terminal_copper_distances = support_field.point_clearances(
                    terminal_3d, max(0.5, copper_required + 0.05))
                expected_support_ids = {
                    f"active-turn-{index:02d}"
                    for index in support_predecessor_indices
                }
                overlapping = {
                    name: distance
                    for name, distance in terminal_copper_distances.items()
                    if distance + 1e-9 < copper_required
                    and name not in expected_support_ids
                }
                if overlapping:
                    raise RuntimeError(
                        "explicit terminal overlaps prior copper: "
                        + ", ".join(
                            f"{name}={distance:.9f}"
                            for name, distance in sorted(overlapping.items())))
                if endpoint_support == "earlier_same_coil_wire":
                    expected = expected_support_ids
                    if (len(declared_support_centerline_distances_mm)
                            != len(support_predecessor_indices)
                            or any(abs(float(distance) - copper_required)
                                   > 1e-9
                                   for distance in
                                   declared_support_centerline_distances_mm)):
                        raise RuntimeError(
                            "declared support lacks an exact analytical "
                            "centerline-distance proof")
                    missing = [
                        name for name in sorted(expected)
                        if name not in terminal_copper_distances
                        or abs(terminal_copper_distances[name]
                               - copper_required) > 1e-3
                    ]
                    if missing:
                        raise RuntimeError(
                            "declared support predecessor is not tangent at "
                            "the explicit terminal: " + ", ".join(missing))
                    support_validated = bool(expected)
                elif endpoint_support == "slot_liner":
                    core_at_terminal = self._point_core_distance(terminal_3d)
                    if abs(core_at_terminal - self.access_radius_mm) > 1e-7:
                        raise RuntimeError(
                            "liner terminal is not tangent to the exact core "
                            f"access surface: {core_at_terminal:.9f} mm")
                    support_validated = True
                else:
                    support_validated = True

                copper_obstacle = projected_copper_obstacle(
                    copper_field, radial,
                    copper_required + 0.0001,
                    buffer_resolution=self.buffer_resolution)
                obstacle = (base_plan.obstacle if copper_obstacle is None
                            else unary_union((base_plan.obstacle,
                                              copper_obstacle)).buffer(0))
                plan = _visibility_plan(
                    obstacle, buffer_resolution=self.buffer_resolution,
                    visibility_chord_mm=self.visibility_chord_mm)
                active_goal = self._explicit_combined_goal(
                    plan, np.asarray(terminal_yz_mm, dtype=float),
                    endpoint_support)
            else:
                plan = base_plan
                active_goal = self._explicit_core_goal(
                    plan, np.asarray(terminal_yz_mm, dtype=float),
                    allow_liner_glide=endpoint_support == "slot_liner")
        except Exception as exc:
            return RouteResult(
                False, f"explicit core route setup failed: {exc}",
                endpoint_support=endpoint_support,
                boundary_source="explicit_winding_plan_target",
                progressive_support_validated=False)

        goal = active_goal.portal
        terminal = active_goal.terminal
        goal_lines = shapely.linestrings(np.stack((
            np.repeat(goal[None, :], len(plan.nodes), axis=0), plan.nodes,
        ), axis=1))
        goal_visible = shapely.relate_pattern(
            goal_lines, plan.obstacle, "F********")
        goal_nodes = np.where(goal_visible)[0]
        if not len(goal_nodes):
            return RouteResult(
                False, "explicit core target is disconnected",
                endpoint_support=endpoint_support,
                boundary_source="explicit_winding_plan_target",
                progressive_support_validated=False)
        goal_distances = np.linalg.norm(plan.nodes - goal, axis=1)
        matrix = (plan.distances[:, goal_nodes]
                  + goal_distances[goal_nodes][None, :])
        choices = np.argmin(matrix, axis=1)
        tail_distances = matrix[np.arange(len(plan.nodes)), choices]
        tail_goal_nodes = goal_nodes[choices]

        rotation = rot_z(float(flyer_angle_rad))
        feed = rotation @ np.asarray(self.guide["feed_local_mm"], dtype=float)
        reference_radial = (
            radial if stator_reference_radial_x_mm is None
            else float(stator_reference_radial_x_mm))
        target_z = (float(self.contact["z_mm"])
                    + reference_radial - radial)
        targets = np.column_stack((
            -plan.nodes[:, 0], plan.nodes[:, 1],
            np.full(len(plan.nodes), target_z),
        ))
        exits, tip_lengths = _coupled_tip_arrays(
            targets, self.guide, self.guide_wire_radius_mm, rotation)
        starts_2d = np.column_stack((-exits[:, 0], exits[:, 1]))
        coupled_lines = shapely.linestrings(
            np.stack((starts_2d, plan.nodes), axis=1))
        coupled_visible = shapely.relate_pattern(
            coupled_lines, plan.obstacle, "F********")
        valid = coupled_visible & np.isfinite(tail_distances)
        candidates: list[tuple[float, int, int]] = [
            (float(tip_lengths[index] + tail_distances[index]),
             int(index), int(tail_goal_nodes[index]))
            for index in np.where(valid)[0]
        ]

        direct_path, direct_meta = _tip_path(
            feed, np.array((-goal[0], goal[1], target_z)), self.guide,
            self.guide_wire_radius_mm, rotation, arc_step_deg=360.0)
        direct_exit = direct_path[-2]
        if LineString(((-direct_exit[0], direct_exit[1]), goal)).relate_pattern(
                plan.obstacle, "F********"):
            candidates.append((direct_meta.analytic_length_mm, -1, -1))
        candidates.sort(key=lambda item: (item[0], item[1], item[2]))

        rejected = 0
        best = (-math.inf, None)
        for total_length, candidate_index, final_node in candidates:
            candidate = (goal if candidate_index < 0
                         else plan.nodes[candidate_index])
            tip_path, tip_meta = _tip_path(
                feed, np.array((-candidate[0], candidate[1], target_z)),
                self.guide, self.guide_wire_radius_mm, rotation,
                arc_step_deg=2.0)
            axis_z = reference_radial + float(self.contact["z_mm"])
            local_tip = np.column_stack((
                axis_z - tip_path[:, 2], -tip_path[:, 0], tip_path[:, 1],
            ))
            if candidate_index < 0:
                route_2d = np.array((goal,))
                tail = np.empty((0, 3))
            else:
                base = plan.path(candidate_index, final_node)
                route_2d = np.vstack((plan.nodes[base], goal))
                tail = np.column_stack((
                    np.full(len(route_2d) - 1, radial), route_2d[1:]))
            polyline = np.vstack((local_tip, tail))
            if np.linalg.norm(terminal - goal) > 1e-12:
                polyline = np.vstack((
                    polyline,
                    np.array((radial, terminal[0], terminal[1])),
                ))
                total_length += float(np.linalg.norm(terminal - goal))
            clearance, segment_index, triangle_index = (
                exact_polyline_mesh_clearance(
                    polyline, self.mesh, self.mesh_search_band_mm))
            if clearance > best[0]:
                best = (clearance, triangle_index)
            if clearance + 1e-9 < self.access_radius_mm:
                rejected += 1
                continue
            copper_clearance = None
            if copper_field is not None:
                copper_clearance = copper_field.clearance(
                    polyline, max(0.5, copper_required + 0.05))
                if (copper_clearance.minimum_centerline_distance_mm + 1e-9
                        < copper_required):
                    rejected += 1
                    continue
            free_start = len(tip_path) - 2
            tags = []
            for segment in range(len(polyline) - 1):
                if segment == 0:
                    tags.append("guide_free")
                elif segment < free_start:
                    tags.append("tip_guide_contact")
                elif segment == free_start:
                    tags.append("free")
                elif segment == len(polyline) - 2:
                    tags.append(active_goal.endpoint_support)
                else:
                    tags.append("core_visibility_glide")
            return RouteResult(
                True, ("progressive explicit route"
                       if copper_field is not None
                       else "core-only explicit route; copper proof required"),
                tuple(tuple(map(float, point)) for point in polyline),
                segment_tags=tuple(tags),
                torus_exit_point_index=free_start,
                center_core_min_mm=float(clearance),
                access_margin_mm=float(clearance - self.access_radius_mm),
                torus_continuity_error_deg=tip_meta.exit_tangent_error_deg,
                total_length_mm=float(total_length),
                free_length_mm=float(np.linalg.norm(
                    polyline[free_start + 1] - polyline[free_start])),
                obstruction_triangle=triangle_index,
                endpoint_family="explicit_winding_plan_target",
                endpoint_support=endpoint_support,
                boundary_source="explicit_winding_plan_target",
                progressive_support_validated=(
                    bool(support_validated) if copper_field is not None
                    else False),
                metadata={
                    "radial_x_mm": radial,
                    "stator_reference_radial_x_mm": reference_radial,
                    "flyer_angle_deg": (
                        math.degrees(flyer_angle_rad) % 360.0),
                    "terminal_goal_local_yz_mm": terminal.tolist(),
                    "goal_local_yz_mm": goal.tolist(),
                    "raw_goal_local_yz_mm": active_goal.raw_portal.tolist(),
                    "planner_portal_nudge_mm": active_goal.portal_nudge_mm,
                    "support_predecessor_indices": list(
                        support_predecessor_indices),
                    "candidate_count": len(candidates),
                    "rejected_shorter_candidates": rejected,
                    "minimum_segment_index": segment_index,
                    "copper_postcheck": "required_before_release",
                    "minimum_copper_centerline_distance_mm": (
                        None if copper_clearance is None else
                        copper_clearance.minimum_centerline_distance_mm),
                    "minimum_copper_obstacle_id": (
                        None if copper_clearance is None else
                        copper_clearance.obstacle_id),
                    "terminal_copper_distances_mm": (
                        terminal_copper_distances),
                    "planning_obstacle_scope": (
                        "active_tooth_neck_seed_with_complete_3d_postcheck"
                        if seed_with_active_neck
                        else "complete_brep_offset_section"),
                })

        return RouteResult(
            False,
            "all explicit core routes violate bare-core access",
            center_core_min_mm=(None if not math.isfinite(best[0])
                                else float(best[0])),
            access_margin_mm=(None if not math.isfinite(best[0])
                              else float(best[0] - self.access_radius_mm)),
            obstruction_triangle=best[1],
            endpoint_family="explicit_winding_plan_target",
            endpoint_support=endpoint_support,
            boundary_source="explicit_winding_plan_target",
            progressive_support_validated=False,
            metadata={
                "radial_x_mm": radial,
                "flyer_angle_deg": math.degrees(flyer_angle_rad) % 360.0,
                "candidate_count": len(candidates),
                "rejected_candidates": rejected,
                "support_predecessor_indices": list(
                    support_predecessor_indices),
            })

    def route(self, radial_x_mm: float, flyer_angle_rad: float,
              motion_sign: int, *, endpoint_family: str = "liner_outbound",
              support_profile_radius_mm: float | None = None) -> RouteResult:
        """Return the shortest exact-clear coupled route, or fail closed."""

        if motion_sign not in (-1, 1):
            return RouteResult(False, "motion_sign must be -1 or +1")
        radial = float(radial_x_mm)
        try:
            plan = self._section_plan(radial)
        except Exception as exc:
            return RouteResult(
                False, f"core configuration-space construction failed: {exc}",
                endpoint_family=endpoint_family,
                progressive_support_validated=False,
                metadata={
                    "radial_x_mm": radial,
                    "flyer_angle_deg": math.degrees(flyer_angle_rad) % 360.0,
                    "motion_sign": motion_sign,
                    "support_profile_radius_mm": support_profile_radius_mm,
                },
            )
        rotation = rot_z(float(flyer_angle_rad))
        feed = rotation @ np.asarray(self.guide["feed_local_mm"], dtype=float)
        guide_center = rotation @ np.asarray(
            self.guide["center_local_mm"], dtype=float)
        desired_world = _tooth_target(
            guide_center, self.contact, motion_sign)
        desired_local = np.array((-desired_world[0], desired_world[1]))
        side = 1 if desired_local[0] >= 0.0 else -1
        if self.clamp_goal_to_stack:
            desired_local[1] = float(np.clip(
                desired_local[1], -self.spec.stack / 2.0,
                self.spec.stack / 2.0))
        support_radius = (self.access_radius_mm
                          if support_profile_radius_mm is None
                          else float(support_profile_radius_mm))
        if support_radius <= 0.0:
            return RouteResult(False, "support_profile_radius must be positive")
        try:
            active_goal = self._active_goal(
                plan, radial, desired_local, side, support_radius)
        except RuntimeError as exc:
            return RouteResult(
                False, str(exc), endpoint_family=endpoint_family,
                boundary_source="bare_core_offset_profile",
                progressive_support_validated=False,
                metadata={
                    "radial_x_mm": radial,
                    "flyer_angle_deg": math.degrees(flyer_angle_rad) % 360.0,
                    "motion_sign": motion_sign,
                    "desired_local_yz_mm": desired_local.tolist(),
                    "support_profile_radius_mm": support_radius,
                    "progressive_support_validated": False,
                })
        goal = active_goal.portal
        terminal_goal = active_goal.terminal
        endpoint_support = active_goal.endpoint_support

        portal_metadata = {
            "raw_goal_local_yz_mm": active_goal.raw_portal.tolist(),
            "goal_local_yz_mm": goal.tolist(),
            "planner_portal_nudge_mm": active_goal.portal_nudge_mm,
            "planner_offset_mm": self.planner_offset_mm,
        }

        goal_lines = shapely.linestrings(np.stack((
            np.repeat(goal[None, :], len(plan.nodes), axis=0), plan.nodes,
        ), axis=1))
        goal_visible = shapely.relate_pattern(
            goal_lines, plan.obstacle, "F********")
        goal_nodes = np.where(goal_visible)[0]
        if not len(goal_nodes):
            return RouteResult(
                False, "active target is disconnected",
                endpoint_family=endpoint_family,
                endpoint_support=endpoint_support,
                boundary_source="bare_core_offset_profile",
                progressive_support_validated=False,
                metadata={
                    "radial_x_mm": radial,
                    "flyer_angle_deg": (
                        math.degrees(flyer_angle_rad) % 360.0),
                    "motion_sign": motion_sign,
                    "desired_local_yz_mm": desired_local.tolist(),
                    "terminal_goal_local_yz_mm": terminal_goal.tolist(),
                    "support_profile_radius_mm": support_radius,
                    **portal_metadata,
                },
            )
        goal_distances = np.linalg.norm(plan.nodes - goal, axis=1)
        matrix = (plan.distances[:, goal_nodes]
                  + goal_distances[goal_nodes][None, :])
        choices = np.argmin(matrix, axis=1)
        tail_distances = matrix[np.arange(len(plan.nodes)), choices]
        tail_goal_nodes = goal_nodes[choices]

        target_z = float(self.contact["z_mm"])
        targets = np.column_stack((
            -plan.nodes[:, 0], plan.nodes[:, 1],
            np.full(len(plan.nodes), target_z),
        ))
        exits, tip_lengths = _coupled_tip_arrays(
            targets, self.guide, self.guide_wire_radius_mm, rotation)
        starts_2d = np.column_stack((-exits[:, 0], exits[:, 1]))
        coupled_lines = shapely.linestrings(
            np.stack((starts_2d, plan.nodes), axis=1))
        coupled_visible = shapely.relate_pattern(
            coupled_lines, plan.obstacle, "F********")
        valid = coupled_visible & np.isfinite(tail_distances)
        candidates: list[tuple[float, int, int]] = [
            (float(tip_lengths[index] + tail_distances[index]),
             int(index), int(tail_goal_nodes[index]))
            for index in np.where(valid)[0]
        ]

        direct_path, direct_meta = _tip_path(
            feed, np.array((-goal[0], goal[1], target_z)), self.guide,
            self.guide_wire_radius_mm, rotation, arc_step_deg=360.0)
        direct_exit = direct_path[-2]
        if LineString(((-direct_exit[0], direct_exit[1]), goal)).relate_pattern(
                plan.obstacle, "F********"):
            candidates.append((direct_meta.analytic_length_mm, -1, -1))
        candidates.sort(key=lambda item: (item[0], item[1], item[2]))

        best_failed: tuple[float, int | None] = (-math.inf, None)
        rejected = 0
        for total_length, candidate_index, final_node in candidates:
            candidate = goal if candidate_index < 0 else plan.nodes[candidate_index]
            tip_path, tip_meta = _tip_path(
                feed, np.array((-candidate[0], candidate[1], target_z)),
                self.guide, self.guide_wire_radius_mm, rotation,
                arc_step_deg=360.0)
            axis_z = radial + target_z
            local_tip = np.column_stack((
                axis_z - tip_path[:, 2], -tip_path[:, 0], tip_path[:, 1],
            ))
            if candidate_index < 0:
                route_2d = np.array((goal,))
                tail = np.empty((0, 3))
            else:
                base = plan.path(candidate_index, final_node)
                route_2d = np.vstack((plan.nodes[base], goal))
                tail = np.column_stack((
                    np.full(len(route_2d) - 1, radial), route_2d[1:],
                ))
            polyline = np.vstack((local_tip, tail))
            if np.linalg.norm(terminal_goal - goal) > 1e-9:
                polyline = np.vstack((
                    polyline,
                    np.array((radial, terminal_goal[0], terminal_goal[1])),
                ))
                total_length += float(np.linalg.norm(terminal_goal - goal))
            clearance, segment_index, triangle_index = (
                exact_polyline_mesh_clearance(
                    polyline, self.mesh, self.mesh_search_band_mm))
            if clearance > best_failed[0]:
                best_failed = (clearance, triangle_index)
            if clearance + 1e-9 < self.access_radius_mm:
                rejected += 1
                continue

            # Candidate enumeration deliberately uses a two-chord torus arc
            # for speed.  Regenerate the selected route at the normal 2-degree
            # display resolution and repeat the exact core check before it can
            # reach a player or serialized table.
            smooth_tip, smooth_meta = _tip_path(
                feed, np.array((-candidate[0], candidate[1], target_z)),
                self.guide, self.guide_wire_radius_mm, rotation,
                arc_step_deg=2.0)
            smooth_local_tip = np.column_stack((
                axis_z - smooth_tip[:, 2], -smooth_tip[:, 0],
                smooth_tip[:, 1],
            ))
            smooth_polyline = np.vstack((smooth_local_tip, tail))
            if np.linalg.norm(terminal_goal - goal) > 1e-9:
                smooth_polyline = np.vstack((
                    smooth_polyline,
                    np.array((radial, terminal_goal[0], terminal_goal[1])),
                ))
            smooth_clearance, smooth_segment, smooth_triangle = (
                exact_polyline_mesh_clearance(
                    smooth_polyline, self.mesh, self.mesh_search_band_mm))
            if smooth_clearance + 1e-9 < self.access_radius_mm:
                rejected += 1
                continue
            tip_path, tip_meta = smooth_tip, smooth_meta
            polyline = smooth_polyline
            clearance, segment_index, triangle_index = (
                smooth_clearance, smooth_segment, smooth_triangle)

            exit_2d = np.array((-tip_path[-2, 0], tip_path[-2, 1]))
            route_with_exit = np.vstack((exit_2d, route_2d))
            bends = 0
            for a, b, c in zip(route_with_exit, route_with_exit[1:],
                               route_with_exit[2:]):
                u, v = b - a, c - b
                cross = abs(float(u[0] * v[1] - u[1] * v[0]))
                if cross > 1e-7 * np.linalg.norm(u) * np.linalg.norm(v):
                    bends += 1
            free_start = len(tip_path) - 2
            free_length = float(np.linalg.norm(
                polyline[free_start + 1] - polyline[free_start]))
            tags = []
            for segment in range(len(polyline) - 1):
                if segment == 0:
                    tags.append("guide_free")
                elif segment < free_start:
                    tags.append("tip_guide_contact")
                elif segment == free_start:
                    tags.append("free")
                else:
                    tags.append("slot_liner_glide")
            if tags and endpoint_support == "workpiece_contact":
                tags[-1] = "workpiece_contact"
            return RouteResult(
                True, "ok",
                tuple(tuple(map(float, point)) for point in polyline),
                segment_tags=tuple(tags),
                torus_exit_point_index=free_start,
                center_core_min_mm=float(clearance),
                access_margin_mm=float(clearance - self.access_radius_mm),
                torus_continuity_error_deg=tip_meta.exit_tangent_error_deg,
                total_length_mm=float(total_length),
                free_length_mm=free_length,
                supported_bends=bends,
                obstruction_triangle=triangle_index,
                endpoint_family=endpoint_family,
                endpoint_support=endpoint_support,
                boundary_source="bare_core_offset_profile",
                progressive_support_validated=False,
                metadata={
                    "radial_x_mm": radial,
                    "flyer_angle_deg": math.degrees(flyer_angle_rad) % 360.0,
                    "motion_sign": motion_sign,
                    "candidate_node": candidate_index,
                    "rejected_shorter_candidates": rejected,
                    "desired_local_yz_mm": desired_local.tolist(),
                    "terminal_goal_local_yz_mm": terminal_goal.tolist(),
                    "endpoint_support": endpoint_support,
                    "endpoint_family": endpoint_family,
                    "support_profile_radius_mm": (
                        support_radius),
                    "boundary_source": "bare_core_offset_profile",
                    "progressive_support_validated": False,
                    "minimum_segment_index": segment_index,
                    "candidate_count": len(candidates),
                    "support_overlap_checked": True,
                    "dependency_versions": dependency_versions(),
                    **portal_metadata,
                },
            )

        reason = ("no coupled visible route" if not candidates else
                  "all coupled routes violate core access")
        best_clearance = (None if not math.isfinite(best_failed[0])
                          else float(best_failed[0]))
        return RouteResult(
            False, reason,
            center_core_min_mm=best_clearance,
            access_margin_mm=(None if best_clearance is None else
                              best_clearance - self.access_radius_mm),
            obstruction_triangle=best_failed[1],
            endpoint_family=endpoint_family,
            endpoint_support=endpoint_support,
            boundary_source="bare_core_offset_profile",
            progressive_support_validated=False,
            metadata={
                "radial_x_mm": radial,
                "flyer_angle_deg": math.degrees(flyer_angle_rad) % 360.0,
                "motion_sign": motion_sign,
                "candidate_count": len(candidates),
                "rejected_candidates": rejected,
                "desired_local_yz_mm": desired_local.tolist(),
                "terminal_goal_local_yz_mm": terminal_goal.tolist(),
                "endpoint_support": endpoint_support,
                "endpoint_family": endpoint_family,
                "support_profile_radius_mm": (
                    support_radius),
                "boundary_source": "bare_core_offset_profile",
                "progressive_support_validated": False,
                "support_overlap_checked": True,
                "dependency_versions": dependency_versions(),
                **portal_metadata,
            },
        )


def route_packing_turn(
    planner: SlotRoutePlanner,
    graph: PackingSupportGraph,
    spec: Any,
    turn_index: int,
    half_turn_index: int,
    *,
    neighbor_sides: tuple[int, ...] = (-1, 1),
    copper_field: CopperField | None = None,
    planning_copper_field: CopperField | None = None,
    arc_step_deg: float = 5.0,
    plan_with_copper_projection: bool = False,
    axial_approach_mm: float = 0.0,
    support_normal_approach_mm: float = 0.0,
    enforce_core_outward_approach: bool = True,
    support_approach_direction_deg: float | None = None,
    mouth_path_local_xy_mm: tuple[tuple[float, float], ...] | None = None,
    end_turn_arc_approach: bool = False,
) -> tuple[RouteResult, PackingTurnRouteAudit]:
    """Route and independently postcheck one packed half-turn crossing.

    The physical pose at each 180-degree crossing is invariant to the sign of
    M2 velocity.  One exact route therefore validates both ``-1`` and ``+1``
    motion signs, which are recorded explicitly rather than duplicated as two
    numerically identical polylines.  The obstacle field contains every
    completed active-tooth turn plus the declared fully wound neighboring
    teeth.  The currently deposited turn is intentionally absent because it
    is the same continuous conductor as the moving span.

    This wrapper fails closed.  A planner result is not released until a
    second exact 3D core-distance pass, a second exact capsule-to-polyline
    copper-distance pass, the endpoint identity check, and the progressive
    support-graph check all agree.
    """

    if half_turn_index not in (0, 1):
        raise ValueError("half_turn_index must be 0 or 1")
    if not neighbor_sides:
        raise ValueError("at least one neighbor prefill side is required")
    normalized_sides = tuple(sorted(set(map(int, neighbor_sides))))
    if any(side not in (-1, 1) for side in normalized_sides):
        raise ValueError("neighbor_sides may contain only -1 and +1")
    if abs(float(planner.access_radius_mm)
           - float(graph.center_core_access_mm)) > 1e-9:
        raise ValueError("planner/core access does not match packing graph")

    turn = graph.turn(turn_index)
    phase_index = 2 * int(turn_index) + int(half_turn_index)
    logical_phase = float(phase_index) * math.pi
    # Reducing the angle keeps the analytic torus solver well conditioned;
    # the exact logical phase remains in the audit and serialized table.
    flyer_phase = float(half_turn_index) * math.pi
    terminal_yz = _rounded_loop_yz(
        turn.profile_radius_mm, flyer_phase, spec)
    target = np.array((turn.radial_mm, *terminal_yz), dtype=float)
    if axial_approach_mm < 0.0:
        raise ValueError("axial_approach_mm cannot be negative")
    if support_normal_approach_mm < 0.0:
        raise ValueError("support_normal_approach_mm cannot be negative")
    if axial_approach_mm > 0.0 and support_normal_approach_mm > 0.0:
        raise ValueError("choose axial or support-normal approach, not both")
    if mouth_path_local_xy_mm is not None and (
            axial_approach_mm > 0.0 or support_normal_approach_mm > 0.0):
        raise ValueError("mouth path cannot be combined with another approach")
    if end_turn_arc_approach and (
            axial_approach_mm > 0.0
            or support_normal_approach_mm > 0.0):
        raise ValueError(
            "end-turn arc cannot combine with axial/support-normal approach")
    planner_terminal_yz = np.asarray(terminal_yz, dtype=float).copy()
    if axial_approach_mm > 0.0:
        planner_terminal_yz[1] += (
            float(axial_approach_mm)
            if half_turn_index == 0 else -float(axial_approach_mm))

    if copper_field is None:
        obstacles = list(active_copper_before(
            graph, turn.turn_index, spec, arc_step_deg=arc_step_deg))
        for neighbor_side in normalized_sides:
            obstacles.extend(neighbor_prefill_copper(
                graph, spec, neighbor_side,
                arc_step_deg=arc_step_deg))
        copper_field = CopperField(tuple(obstacles))
    if planning_copper_field is None:
        planning_copper_field = copper_field

    support_audit = classify_active_loop_contacts(graph, turn.turn_index)
    endpoint_support = (
        "slot_liner" if turn.support_kind == "slot_liner"
        else "earlier_same_coil_wire")
    # The complete copper field is always an independent 3D release
    # postcheck.  Projecting thousands of rounded-loop segments into the 2D
    # visibility graph is optional because that graph is only a route search
    # accelerator, not pass/fail authority.  The release generator uses the
    # deterministic core route and fails closed if its exact copper postcheck
    # is not clear; callers may request a slower copper-aware search.
    planner_endpoint_support = (
        "free_space" if axial_approach_mm > 0.0 else endpoint_support)
    planner_predecessors = (
        () if axial_approach_mm > 0.0 else turn.parent_turn_indices)
    if end_turn_arc_approach:
        half_neck = max(2.5, float(spec.od) * 0.07) / 2.0
        half_stack = float(spec.stack) / 2.0
        if half_turn_index == 0:
            center_yz = np.array((-half_neck, half_stack))
            angles = np.linspace(math.pi / 2.0, math.pi, 37)
        else:
            center_yz = np.array((half_neck, -half_stack))
            angles = np.linspace(-math.pi / 2.0, 0.0, 37)
        arc_yz = np.array([
            center_yz + turn.profile_radius_mm * np.array((
                math.cos(angle), math.sin(angle)))
            for angle in angles
        ])
        rotation = rot_z(flyer_phase)
        feed = rotation @ np.asarray(
            planner.guide["feed_local_mm"], dtype=float)
        axis_z = turn.radial_mm + float(planner.contact["z_mm"])
        arc_start = np.array((turn.radial_mm, *arc_yz[0]))
        end_plane_path = np.asarray((arc_start,), dtype=float)
        source_mouth_path: np.ndarray | None = None
        if mouth_path_local_xy_mm is not None:
            source_mouth_path = np.asarray(
                mouth_path_local_xy_mm, dtype=float)
            if (source_mouth_path.ndim != 2
                    or source_mouth_path.shape[1] != 2
                    or len(source_mouth_path) < 2
                    or not np.all(np.isfinite(source_mouth_path))):
                raise ValueError(
                    "end-turn mouth path must contain finite local xy points")
            if np.linalg.norm(source_mouth_path[-1] - target[:2]) > 1e-8:
                raise ValueError(
                    "end-turn mouth path does not terminate at packed target")
            if half_turn_index == 0:
                profiles = -source_mouth_path[:, 1] - half_neck
                end_plane_path = np.column_stack((
                    source_mouth_path[:, 0],
                    np.full(len(source_mouth_path), -half_neck),
                    half_stack + profiles,
                ))
            else:
                profiles = source_mouth_path[:, 1] - half_neck
                end_plane_path = np.column_stack((
                    source_mouth_path[:, 0],
                    np.full(len(source_mouth_path), half_neck),
                    -half_stack - profiles,
                ))
            if np.linalg.norm(end_plane_path[-1] - arc_start) > 1e-8:
                raise RuntimeError(
                    "mapped slot-mouth path does not meet end-turn arc")
        start_world = np.array((
            -end_plane_path[0, 1], end_plane_path[0, 2],
            axis_z - end_plane_path[0, 0]))
        tip_path, tip_meta = _tip_path(
            feed, start_world, planner.guide,
            planner.guide_wire_radius_mm, rotation, arc_step_deg=2.0)
        local_tip = np.column_stack((
            axis_z - tip_path[:, 2], -tip_path[:, 0], tip_path[:, 1]))
        end_plane_tail = end_plane_path[1:]
        arc_tail = np.column_stack((
            np.full(len(arc_yz) - 1, turn.radial_mm), arc_yz[1:]))
        polyline = np.vstack((local_tip, end_plane_tail, arc_tail))
        core_distance, core_segment, core_triangle = (
            exact_polyline_mesh_clearance(
                polyline, planner.mesh, planner.mesh_search_band_mm))
        free_start = len(tip_path) - 2
        mouth_first_segment = len(tip_path) - 1
        arc_first_segment = len(tip_path) + len(end_plane_tail) - 1
        tags = []
        for segment in range(len(polyline) - 1):
            if segment == 0:
                tags.append("guide_free")
            elif segment < free_start:
                tags.append("tip_guide_contact")
            elif segment == free_start:
                tags.append("free")
            elif segment < arc_first_segment:
                tags.append("end_turn_mouth_glide")
            elif segment == len(polyline) - 2:
                tags.append(endpoint_support)
            else:
                tags.append("corresponding_end_turn_arc")
        result = RouteResult(
            True, "corresponding rounded end-turn arc candidate",
            tuple(tuple(map(float, point)) for point in polyline),
            segment_tags=tuple(tags),
            torus_exit_point_index=free_start,
            center_core_min_mm=float(core_distance),
            access_margin_mm=float(
                core_distance - graph.center_core_access_mm),
            torus_continuity_error_deg=tip_meta.exit_tangent_error_deg,
            total_length_mm=float(
                tip_meta.analytic_length_mm
                + sum(np.linalg.norm(two - one)
                      for one, two in zip(end_plane_path,
                                         end_plane_path[1:]))
                + turn.profile_radius_mm * math.pi / 2.0),
            free_length_mm=float(np.linalg.norm(
                polyline[free_start + 1] - polyline[free_start])),
            obstruction_triangle=core_triangle,
            endpoint_family="explicit_winding_plan_target",
            endpoint_support=endpoint_support,
            boundary_source="corresponding_end_turn_arc",
            progressive_support_validated=False,
            metadata={
                "radial_x_mm": turn.radial_mm,
                "flyer_angle_deg": math.degrees(flyer_phase) % 360.0,
                "support_predecessor_indices": list(
                    turn.parent_turn_indices),
                "minimum_segment_index": core_segment,
                "end_turn_arc_approach": {
                    "arc_first_segment_index": arc_first_segment,
                    "arc_last_segment_index": len(polyline) - 2,
                    "radius_mm": turn.profile_radius_mm,
                    "sweep_deg": 90.0,
                    "slot_mouth_component_mapped": bool(
                        source_mouth_path is not None),
                    "mouth_first_segment_index": mouth_first_segment,
                    "mouth_path_local_xy_mm": (
                        None if source_mouth_path is None
                        else source_mouth_path.tolist()),
                    "mapped_end_plane_points_local_mm": (
                        end_plane_path.tolist()),
                },
            },
        )
    elif mouth_path_local_xy_mm is not None:
        mouth_xy = np.asarray(mouth_path_local_xy_mm, dtype=float)
        if (mouth_xy.ndim != 2 or mouth_xy.shape[1] != 2
                or len(mouth_xy) < 2 or not np.all(np.isfinite(mouth_xy))):
            raise ValueError("mouth path must contain finite local xy points")
        if np.linalg.norm(mouth_xy[-1] - target[:2]) > 1e-8:
            raise ValueError("mouth path does not terminate at packed target")
        rotation = rot_z(flyer_phase)
        feed = rotation @ np.asarray(
            planner.guide["feed_local_mm"], dtype=float)
        axis_z = turn.radial_mm + float(planner.contact["z_mm"])
        mouth = np.array((mouth_xy[0, 0], mouth_xy[0, 1], target[2]))
        mouth_world = np.array((
            -mouth[1], mouth[2], axis_z - mouth[0]))
        tip_path, tip_meta = _tip_path(
            feed, mouth_world, planner.guide,
            planner.guide_wire_radius_mm, rotation, arc_step_deg=2.0)
        local_tip = np.column_stack((
            axis_z - tip_path[:, 2], -tip_path[:, 0], tip_path[:, 1]))
        mouth_tail = np.column_stack((
            mouth_xy[1:, 0], mouth_xy[1:, 1],
            np.full(len(mouth_xy) - 1, target[2])))
        polyline = np.vstack((local_tip, mouth_tail))
        core_distance, core_segment, core_triangle = (
            exact_polyline_mesh_clearance(
                polyline, planner.mesh, planner.mesh_search_band_mm))
        free_start = len(tip_path) - 2
        tags = []
        for segment in range(len(polyline) - 1):
            if segment == 0:
                tags.append("guide_free")
            elif segment < free_start:
                tags.append("tip_guide_contact")
            elif segment == free_start:
                tags.append("free")
            elif segment == len(polyline) - 2:
                tags.append(endpoint_support)
            else:
                tags.append("slot_mouth_glide")
        result = RouteResult(
            True, "explicit slot-mouth component path candidate",
            tuple(tuple(map(float, point)) for point in polyline),
            segment_tags=tuple(tags),
            torus_exit_point_index=free_start,
            center_core_min_mm=float(core_distance),
            access_margin_mm=float(
                core_distance - graph.center_core_access_mm),
            torus_continuity_error_deg=tip_meta.exit_tangent_error_deg,
            total_length_mm=float(
                tip_meta.analytic_length_mm
                + sum(np.linalg.norm(two - one)
                      for one, two in zip(mouth_xy, mouth_xy[1:]))),
            free_length_mm=float(np.linalg.norm(
                polyline[free_start + 1] - polyline[free_start])),
            obstruction_triangle=core_triangle,
            endpoint_family="explicit_winding_plan_target",
            endpoint_support=endpoint_support,
            boundary_source="slot_mouth_connected_component",
            progressive_support_validated=False,
            metadata={
                "radial_x_mm": turn.radial_mm,
                "flyer_angle_deg": math.degrees(flyer_phase) % 360.0,
                "support_predecessor_indices": list(
                    turn.parent_turn_indices),
                "minimum_segment_index": core_segment,
                "slot_mouth_path": {
                    "point_count": len(mouth_xy),
                    "points_local_xy_mm": mouth_xy.tolist(),
                },
            },
        )
    elif support_normal_approach_mm > 0.0:
        if not turn.parent_turn_indices:
            raise ValueError(
                "support-normal approach requires tangent support parents")
        normals = []
        for parent_index in turn.parent_turn_indices:
            parent = graph.turn(parent_index)
            parent_yz = _rounded_loop_yz(
                parent.profile_radius_mm, flyer_phase, spec)
            vector = target - np.array((
                parent.radial_mm, float(parent_yz[0]),
                float(parent_yz[1])))
            length = float(np.linalg.norm(vector))
            if abs(length - graph.wire_diameter_mm) > 1e-8:
                raise ValueError(
                    "support-normal endpoint is not analytically tangent")
            normals.append(vector / length)
        # Find the center of the common outward normal cone.  Parent contact
        # normals have zero axial component at the two side crossings, so a
        # deterministic angular search in stator x/y is exact for this cone.
        candidates = np.array([
            (math.cos(math.radians(degree / 2.0)),
             math.sin(math.radians(degree / 2.0)), 0.0)
            for degree in range(720)
        ] + [(0.0, 0.0, 1.0 if half_turn_index == 0 else -1.0)],
            dtype=float)
        core_outward = np.array((
            0.0, -1.0 if half_turn_index == 0 else 1.0, 0.0))
        cone_normals = np.asarray(normals)
        if enforce_core_outward_approach:
            cone_normals = np.vstack((cone_normals, core_outward))
        scores = np.min(candidates @ cone_normals.T, axis=1)
        if support_approach_direction_deg is None:
            planar_best = int(np.argmax(scores[:-1]))
            # Prefer a physical slot-plane approach when it satisfies the
            # cone to floating-point tolerance.  The axial candidate is a
            # genuine fallback, not a tie-breaker for an exact zero-width
            # planar corridor.
            best_index = (planar_best if scores[planar_best] >= -1e-10
                          else len(scores) - 1)
            direction = candidates[best_index]
            direction_score = float(scores[best_index])
        else:
            angle = math.radians(float(support_approach_direction_deg))
            direction = np.array((math.cos(angle), math.sin(angle), 0.0))
            direction_score = float(np.min(cone_normals @ direction))
        if direction_score < -1e-12:
            raise RuntimeError("support parents have no common outward cone")
        approach = target + float(support_normal_approach_mm) * direction
        result = planner.route_explicit_core_target(
            float(approach[0]),
            flyer_phase,
            approach[1:],
            endpoint_support="free_space",
            seed_with_active_neck=True,
            copper_field=planning_copper_field,
            support_copper_field=planning_copper_field,
            copper_center_clearance_mm=graph.wire_diameter_mm,
            stator_reference_radial_x_mm=turn.radial_mm,
        )
        if result.ok:
            metadata = dict(result.metadata)
            metadata["support_normal_approach"] = {
                "distance_mm": float(support_normal_approach_mm),
                "direction_local": direction.tolist(),
                    "minimum_parent_normal_dot": direction_score,
                    "requested_direction_deg": (
                        None if support_approach_direction_deg is None else
                        float(support_approach_direction_deg) % 360.0),
                    "core_outward_normal_local": core_outward.tolist(),
                    "core_outward_constraint_enabled": bool(
                        enforce_core_outward_approach),
                "approach_target_local_mm": approach.tolist(),
            }
            result = replace(
                result,
                points_local=(*result.points_local_mm,
                              tuple(map(float, target))),
                segment_tags=(*result.segment_tags, endpoint_support),
                endpoint_support=endpoint_support,
                boundary_source="support_normal_approach",
                total_length_mm=(
                    None if result.total_length_mm is None else
                    float(result.total_length_mm)
                    + float(support_normal_approach_mm)),
                metadata=metadata,
            )
    else:
        result = planner.route_explicit_core_target(
            turn.radial_mm,
            flyer_phase,
            planner_terminal_yz,
            endpoint_support=planner_endpoint_support,
            support_predecessor_indices=planner_predecessors,
            seed_with_active_neck=True,
            copper_field=(planning_copper_field
                          if plan_with_copper_projection else None),
            support_copper_field=(copper_field
                                  if plan_with_copper_projection else None),
            copper_center_clearance_mm=(
                graph.wire_diameter_mm
                if plan_with_copper_projection else None),
            declared_support_centerline_distances_mm=tuple(
                graph.wire_diameter_mm for _ in planner_predecessors),
        )

    if result.ok and axial_approach_mm > 0.0:
        points_with_target = (*result.points_local_mm,
                              tuple(map(float, target)))
        tags_with_target = (*result.segment_tags, endpoint_support)
        metadata = dict(result.metadata)
        metadata["axial_approach"] = {
            "distance_mm": float(axial_approach_mm),
            "approach_target_local_mm": list(result.points_local_mm[-1]),
            "supported_terminal_local_mm": target.tolist(),
            "rule": (
                "route to free axial point, then approach the declared "
                "tangent support parallel to the end-turn side segment"),
        }
        result = replace(
            result,
            points_local=points_with_target,
            segment_tags=tags_with_target,
            endpoint_support=endpoint_support,
            total_length_mm=(None if result.total_length_mm is None else
                             float(result.total_length_mm)
                             + float(axial_approach_mm)),
            metadata=metadata,
        )

    if not result.ok or not result.points_local_mm:
        audit = PackingTurnRouteAudit(
            ok=False,
            turn_index=turn.turn_index,
            half_turn_index=int(half_turn_index),
            phase_index=phase_index,
            logical_phase_rad=logical_phase,
            validated_motion_signs=(-1, 1),
            target_local_mm=tuple(map(float, target)),
            endpoint_error_mm=math.inf,
            minimum_core_center_distance_mm=(
                -math.inf if result.center_core_min_mm is None
                else float(result.center_core_min_mm)),
            minimum_copper_center_distance_mm=-math.inf,
            minimum_copper_obstacle_id=None,
            support_contract_ok=bool(support_audit.ok),
            required_core_center_distance_mm=float(
                graph.center_core_access_mm),
            required_copper_center_distance_mm=float(
                graph.wire_diameter_mm),
            reason=f"planner rejected packed route: {result.reason}",
        )
        return result, audit

    points = np.asarray(result.points_local_mm, dtype=float)
    endpoint_error = float(np.linalg.norm(points[-1] - target))
    mesh_core_distance, core_segment, core_triangle = (
        exact_polyline_mesh_clearance(
            points, planner.mesh, planner.mesh_search_band_mm))
    core_distance = exact_polyline_part_clearance(
        points, planner.stator_part)
    copper = copper_field.clearance(
        points, max(0.5, graph.wire_diameter_mm + 0.05))
    parent_ids = {
        f"active-turn-{index:02d}" for index in turn.parent_turn_indices
    }
    nonparent_field = CopperField(tuple(
        obstacle for obstacle in copper_field.obstacles
        if obstacle.obstacle_id not in parent_ids))
    nonparent_copper = nonparent_field.clearance(
        points, max(0.5, graph.wire_diameter_mm + 0.05))
    # Rounded-loop obstacles are chordalizations of exact quarter circles.
    # Bound their Hausdorff error instead of pretending the sampled polyline
    # is the continuous copper centerline.
    maximum_profile = max(item.profile_radius_mm for item in graph.turns)
    chord_error_bound = (
        maximum_profile
        * (1.0 - math.cos(math.radians(arc_step_deg) / 2.0))
        + 1e-9)
    nonparent_lower_bound = (
        nonparent_copper.minimum_centerline_distance_mm
        - chord_error_bound)
    intended_support_contact_ok = not parent_ids
    support_normal_dots: dict[str, float] = {}
    corresponding_arc_chord_error_bound = 0.0
    parent_prefix_lower_bound = math.inf
    parent_prefix_obstacle_id: str | None = None
    mapped_mouth_parent_lower_bound = math.inf
    mapped_mouth_parent_distances: dict[str, float] = {}
    if parent_ids:
        arc_metadata = result.metadata.get("end_turn_arc_approach")
        if isinstance(arc_metadata, dict):
            arc_first = int(arc_metadata["arc_first_segment_index"])
            arc_last = int(arc_metadata["arc_last_segment_index"])
            if not (0 < arc_first <= arc_last < len(points) - 1):
                raise RuntimeError("invalid corresponding end-turn arc range")

            # Everything before the final free segment into the arc must be
            # conservatively clear of the support parents.  The final free
            # segment itself terminates at the exact tangent point and is
            # proved by its common outward-support cone below.
            parent_field = CopperField(tuple(
                obstacle for obstacle in copper_field.obstacles
                if obstacle.obstacle_id in parent_ids))
            mapped_mouth = bool(
                arc_metadata.get("slot_mouth_component_mapped"))
            mouth_first = int(arc_metadata.get(
                "mouth_first_segment_index", arc_first - 1))
            prefix_points = (
                points[:mouth_first + 1]
                if mapped_mouth else points[:arc_first])
            if len(prefix_points) >= 2:
                parent_prefix = parent_field.clearance(
                    prefix_points,
                    max(0.5, graph.wire_diameter_mm + 0.05),
                )
                parent_prefix_lower_bound = (
                    parent_prefix.minimum_centerline_distance_mm
                    - chord_error_bound)
                parent_prefix_obstacle_id = parent_prefix.obstacle_id

            if mapped_mouth:
                mapped_points = np.asarray(
                    arc_metadata["mapped_end_plane_points_local_mm"],
                    dtype=float)
                if (mapped_points.ndim != 2
                        or mapped_points.shape[1] != 3
                        or len(mapped_points) < 2):
                    raise RuntimeError(
                        "invalid mapped end-turn mouth path")
                half_stack = float(spec.stack) / 2.0
                mapped_state = np.column_stack((
                    mapped_points[:, 0],
                    (mapped_points[:, 2] - half_stack
                     if half_turn_index == 0
                     else -half_stack - mapped_points[:, 2]),
                ))
                for parent_index in turn.parent_turn_indices:
                    parent = graph.turn(parent_index)
                    distances = _point_to_segments_2d(
                        np.array((parent.radial_mm,
                                  parent.profile_radius_mm)),
                        mapped_state[:-1], mapped_state[1:])
                    value = float(np.min(distances))
                    mapped_mouth_parent_distances[
                        f"active-turn-{parent_index:02d}"] = value
                    mapped_mouth_parent_lower_bound = min(
                        mapped_mouth_parent_lower_bound, value)

            half_neck = max(2.5, float(spec.od) * 0.07) / 2.0
            half_stack = float(spec.stack) / 2.0
            if half_turn_index == 0:
                corner_center_yz = np.array((-half_neck, half_stack))
                arc_start_angle = math.pi / 2.0
            else:
                corner_center_yz = np.array((half_neck, -half_stack))
                arc_start_angle = -math.pi / 2.0
            arc_start = points[arc_first]
            approach_vector = points[arc_first - 1] - arc_start
            tangent_graph_ok = True
            for parent_index in turn.parent_turn_indices:
                parent = graph.turn(parent_index)
                center_separation = math.hypot(
                    turn.radial_mm - parent.radial_mm,
                    turn.profile_radius_mm - parent.profile_radius_mm,
                )
                tangent_graph_ok = bool(
                    tangent_graph_ok
                    and abs(center_separation - graph.wire_diameter_mm)
                    <= 1e-9)
                parent_arc_start_yz = (
                    corner_center_yz
                    + parent.profile_radius_mm * np.array((
                        math.cos(arc_start_angle),
                        math.sin(arc_start_angle),
                    )))
                contact_normal = _unit(
                    arc_start - np.array((
                        parent.radial_mm,
                        float(parent_arc_start_yz[0]),
                        float(parent_arc_start_yz[1]),
                    )))
                support_normal_dots[
                    f"active-turn-{parent_index:02d}"] = float(
                        np.dot(approach_vector, contact_normal))

            arc_segment_count = arc_last - arc_first + 1
            route_arc_step = math.radians(90.0 / arc_segment_count)
            route_arc_chord_error = (
                turn.profile_radius_mm
                * (1.0 - math.cos(route_arc_step / 2.0)))
            corresponding_arc_chord_error_bound = (
                chord_error_bound + route_arc_chord_error + 1e-9)
            # The rounded loops are a common Minkowski-offset family.  For
            # every point on this quarter arc, the corresponding point on a
            # declared support parent is exactly one graph edge away in the
            # (radial, profile-radius) plane.  The two chord-error bounds
            # explain only the finite sampling of those exact curves.
            intended_support_contact_ok = bool(
                tangent_graph_ok
                and copper.obstacle_id in parent_ids
                and arc_first - 1 <= copper.route_segment_index <= arc_last
                and (copper.minimum_centerline_distance_mm
                     + corresponding_arc_chord_error_bound + 1e-9
                     >= graph.wire_diameter_mm)
                and parent_prefix_lower_bound + 1e-9
                >= graph.wire_diameter_mm
                and mapped_mouth_parent_lower_bound + 1e-9
                >= graph.wire_diameter_mm
                and all(value >= -1e-10
                        for value in support_normal_dots.values()))
        else:
            parent_field = CopperField(tuple(
                obstacle for obstacle in copper_field.obstacles
                if obstacle.obstacle_id in parent_ids))
            prefix_points = points[:-1]
            if len(prefix_points) >= 2:
                parent_prefix = parent_field.clearance(
                    prefix_points,
                    max(0.5, graph.wire_diameter_mm + 0.05),
                )
                parent_prefix_lower_bound = (
                    parent_prefix.minimum_centerline_distance_mm
                    - chord_error_bound)
                parent_prefix_obstacle_id = parent_prefix.obstacle_id
            approach_vector = points[-2] - target
            for parent_index in turn.parent_turn_indices:
                parent = graph.turn(parent_index)
                parent_yz = _rounded_loop_yz(
                    parent.profile_radius_mm, flyer_phase, spec)
                contact_normal = target - np.array((
                    parent.radial_mm, float(parent_yz[0]),
                    float(parent_yz[1])))
                contact_normal = _unit(contact_normal)
                support_normal_dots[
                    f"active-turn-{parent_index:02d}"] = float(
                        np.dot(approach_vector, contact_normal))
            intended_support_contact_ok = bool(
                copper.obstacle_id in parent_ids
                and copper.route_segment_index == len(points) - 2
                and (copper.minimum_centerline_distance_mm
                     + chord_error_bound + 1e-9
                     >= graph.wire_diameter_mm)
                and parent_prefix_lower_bound + 1e-9
                >= graph.wire_diameter_mm
                and all(value >= -1e-10
                        for value in support_normal_dots.values()))
    effective_copper_distance = min(
        float(graph.wire_diameter_mm) if parent_ids else math.inf,
        float(nonparent_lower_bound),
    )
    toler = 1e-9
    appended_approach = bool(
        "support_normal_approach" in result.metadata
        or "axial_approach" in result.metadata)
    checks = {
        "endpoint_identity": endpoint_error <= toler,
        "core_center_clearance": (
            core_distance + toler >= graph.center_core_access_mm),
        "copper_center_clearance": (
            intended_support_contact_ok
            and effective_copper_distance + toler
            >= graph.wire_diameter_mm),
        "progressive_support_graph": (
            support_audit.ok
            and support_audit.progressive_support_validated),
        "planner_core_agreement": (
            result.center_core_min_mm is not None
            and ((appended_approach
                  and float(result.center_core_min_mm) + toler
                  >= graph.center_core_access_mm)
                 or (not appended_approach
                     and abs(float(result.center_core_min_mm)
                             - mesh_core_distance)
                     <= 1e-9))),
        "planner_copper_projection_clear": (
            not plan_with_copper_projection
            or (result.metadata.get(
                "minimum_copper_centerline_distance_mm") is not None
                and float(result.metadata[
                    "minimum_copper_centerline_distance_mm"]) + toler
                    >= graph.wire_diameter_mm)),
    }
    ok = all(checks.values())
    reason = "ok" if ok else "; ".join(
        name for name, passed in checks.items() if not passed)
    audit = PackingTurnRouteAudit(
        ok=ok,
        turn_index=turn.turn_index,
        half_turn_index=int(half_turn_index),
        phase_index=phase_index,
        logical_phase_rad=logical_phase,
        validated_motion_signs=(-1, 1),
        target_local_mm=tuple(map(float, target)),
        endpoint_error_mm=endpoint_error,
        minimum_core_center_distance_mm=float(core_distance),
        minimum_copper_center_distance_mm=float(effective_copper_distance),
        minimum_copper_obstacle_id=copper.obstacle_id,
        support_contract_ok=bool(
            support_audit.ok and support_audit.progressive_support_validated),
        required_core_center_distance_mm=float(
            graph.center_core_access_mm),
        required_copper_center_distance_mm=float(
            graph.wire_diameter_mm),
        reason=reason,
    )
    metadata = dict(result.metadata)
    planner_predecessor_metadata = list(
        metadata.get("support_predecessor_indices", ()))
    metadata.update({
        "packing_report_schema": graph.schema,
        "packing_report_sha256": graph.report_sha256,
        "packing_turn_index": turn.turn_index,
        "packing_half_turn_index": int(half_turn_index),
        "packing_phase_index": phase_index,
        "packing_logical_phase_rad": logical_phase,
        "validated_motion_signs": [-1, 1],
        "motion_direction_invariant": (
            "crossing pose and tangent/arc/tangent geometry depend on flyer "
            "angle, not the sign of angular velocity"),
        "neighbor_prefill_sides": list(normalized_sides),
        "prior_active_turn_count": turn.turn_index,
        "planner_endpoint_support_predecessor_indices": (
            planner_predecessor_metadata),
        "support_predecessor_indices": list(turn.parent_turn_indices),
        "copper_search_scope": (
            "2d_projected_search_plus_exact_3d_postcheck"
            if plan_with_copper_projection else
            "exact_3d_postcheck_of_deterministic_core_route"),
        "planning_copper_obstacle_count": len(
            planning_copper_field.obstacles),
        "postcheck_copper_obstacle_count": len(copper_field.obstacles),
        "exact_release_postcheck": {
            "checks": checks,
            "endpoint_error_mm": endpoint_error,
            "minimum_core_center_distance_mm": float(core_distance),
            "mesh_diagnostic_core_center_distance_mm": float(
                mesh_core_distance),
            "minimum_core_segment_index": core_segment,
            "minimum_core_triangle_index": core_triangle,
            "minimum_copper_center_distance_mm": float(
                effective_copper_distance),
            "raw_chordal_copper_center_distance_mm": float(
                copper.minimum_centerline_distance_mm),
            "nonparent_raw_chordal_distance_mm": float(
                nonparent_copper.minimum_centerline_distance_mm),
            "loop_chord_error_bound_mm": float(chord_error_bound),
            "corresponding_arc_chord_error_bound_mm": float(
                corresponding_arc_chord_error_bound),
            "parent_prefix_centerline_lower_bound_mm": float(
                parent_prefix_lower_bound),
            "parent_prefix_minimum_obstacle_id": (
                parent_prefix_obstacle_id),
            "mapped_mouth_parent_centerline_lower_bound_mm": float(
                mapped_mouth_parent_lower_bound),
            "mapped_mouth_parent_centerline_distances_mm": (
                mapped_mouth_parent_distances),
            "intended_support_contact_analytical_proof": bool(
                intended_support_contact_ok),
            "support_approach_normal_dots": support_normal_dots,
            "minimum_copper_route_segment_index": (
                copper.route_segment_index),
            "minimum_copper_obstacle_id": copper.obstacle_id,
            "minimum_copper_obstacle_segment_index": (
                copper.obstacle_segment_index),
        },
    })
    if not ok:
        result = replace(
            result,
            ok=False,
            reason=f"packed route release postcheck failed: {reason}",
            progressive_support_validated=False,
            metadata=metadata,
        )
    else:
        result = replace(
            result, progressive_support_validated=True,
            reason="packed route passed independent core/copper/support proof",
            metadata=metadata)
    return result, audit
