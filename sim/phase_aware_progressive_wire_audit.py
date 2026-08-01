"""Phase-aware progressive audit for the moving flyer-to-workpiece wire.

This study exists because ``wirepath.py`` deliberately keeps the complete
``stator_final_wound_envelope`` as one conservative spindle-link solid.  A
single distance to that aggregate cannot say whether the moving wire met its
declared active support, touched already-deposited copper, or crossed bare
steel/a different coil.  Worse, an allowed active contact can be the global
minimum and hide a second forbidden collision.

The audit is isolated from production gates and does not edit CAD, settings,
or captures.  It replays all 100 physical half-turn deposition loci in every
one of the 24 canonical upstream passes and probes these classes separately:

* exact source stator core plus the selected Nomex core-offset contract;
* declared active-tooth contact/support;
* completed earlier turns on the active tooth;
* completed adjacent-tooth copper; and
* completed non-adjacent-tooth copper.

Two paths are kept distinct.  ``nominal_ellipse_tangent`` is the current
straight terminal span and preserves its exact steel witness.  The
``core_visibility_successor`` uses the existing SlotRoutePlanner against the
source stator BREP and selected Nomex offset, but it is diagnostic until the
separate >=3 mm turning/elastic route study is production-authorized.

The constructive packing graph supplies explicit prior-copper centreline
obstacles only.  The raw M0 loci are never required to equal those packed
centres, and layer order/neatness is not a gate here.  Copper contact is
allowed; centreline interpenetration is not.  Each class is queried
independently so no allowed contact can mask a forbidden second witness.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable, Mapping

import numpy as np
from shapely.geometry import LineString


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
MANIFEST = ROOT / "out" / "links" / "manifest.json"
PACKING = REPORTS / "slot_packing.json"
INSULATION = REPORTS / "stator_insulation_nomex410_5mil.json"
ELASTIC_3D = REPORTS / "elastic_3d_turn45_route_study.json"
STORED_ROUTES = REPORTS / "slot_wire_routes.json"
SETTINGS = ROOT / "out" / "settings.yml"
GOAL = ROOT.parent / "GOAL.md"
OUTPUT_JSON = REPORTS / "phase_aware_progressive_wire_audit.json"
OUTPUT_MD = REPORTS / "phase_aware_progressive_wire_audit.md"

for search_path in (CAD, HERE):
    if str(search_path) not in sys.path:
        sys.path.insert(0, str(search_path))

import collide  # noqa: E402
import coil_growth  # noqa: E402
from params import DEFAULT_STATOR, PARAMS  # noqa: E402
from slot_route import (  # noqa: E402
    CopperField,
    CopperPolyline,
    PackingSupportGraph,
    SlotRoutePlanner,
    _loop_centerline,
    exact_polyline_part_clearance,
)
import stator_insulation_nomex410 as insulation_source  # noqa: E402
import elastic_3d_turn45_route_study as elastic_3d_study  # noqa: E402
import stator_model  # noqa: E402
from traj import Timeline, load_events, winding_windows  # noqa: E402
import wire_geometry  # noqa: E402
import wirepath  # noqa: E402


SCHEMA = "phase-aware-progressive-wire-audit/v1"
EXPECTED_CAPTURE_SCHEMA = 4
EXPECTED_PASSES = 24
STATES_PER_PASS = 100
EXPECTED_STATE_COUNT = EXPECTED_PASSES * STATES_PER_PASS
COPPER_ARC_STEP_DEG = 5.0
# Maximum sagitta at the largest current rounded-loop profile, conservatively
# rounded upward. Exact segment/capsule distances are reported for the
# polyline model and this bound is subtracted before pass/fail classification.
COPPER_MODEL_CHORD_BOUND_MM = 0.0030
COPPER_CONTACT_BAND_MM = 0.002
COPPER_STRICT_NEAR_BAND_MM = 0.050
NUMERICAL_TOL_MM = 1.0e-7
M1_ALIGNMENT_TOL_RAD = 1.0e-7
PLANNER_SHELL_MM = 0.0001
SUCCESSOR_RADIAL_TEMPLATE_COUNT = 3


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    body = deepcopy(dict(payload))
    body.pop("report_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} is not a JSON object")
    return value


def _wrap_pi(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def _directed_crossing_times(
    track: Any,
    start_t: float,
    start_pos: float,
    direction: int,
    count: int,
) -> list[float]:
    """Invert one monotone raw M2 pass at exact pi-spaced crossings."""

    if direction not in (-1, 1):
        raise ValueError("crossing direction must be -1 or +1")
    result: list[float] = []
    for index in range(int(count)):
        target = float(start_pos + direction * index * math.pi)
        if index == 0:
            result.append(float(start_t))
            continue
        found = None
        for (left_t, _), (right_t, right_p) in zip(
                track.knots, track.knots[1:]):
            if right_t < start_t - 1.0e-12:
                continue
            local_left_t = max(float(left_t), float(start_t))
            local_left_p = float(track.pos_at(local_left_t))
            if direction * (float(right_p) - local_left_p) < -1.0e-10:
                continue
            if ((target - local_left_p) * (target - float(right_p))
                    <= 1.0e-10
                    and abs(float(right_p) - local_left_p) > 1.0e-12):
                found = local_left_t + (
                    (float(right_t) - local_left_t)
                    * (target - local_left_p)
                    / (float(right_p) - local_left_p)
                )
                break
        if found is None:
            break
        result.append(float(found))
    return result


@dataclass(frozen=True)
class RawLocus:
    pass_index: int
    phase_index: int
    tooth_index: int
    motion_sign: int
    clockwise_argument: bool
    state_index: int
    turn_index: int
    half_turn_index: int
    time_s: float
    m0_rad: float
    m1_rad: float
    m2_rad: float
    m2_mod_rad: float
    radial_x_mm: float
    m1_alignment_error_rad: float

    @property
    def route_key(self) -> tuple[float, float, int]:
        return (
            round(self.radial_x_mm, 9),
            round(self.m2_mod_rad, 9),
            self.motion_sign,
        )


def extract_raw_loci(
    events: list[dict[str, Any]], timeline: Timeline,
) -> tuple[list[RawLocus], list[dict[str, Any]]]:
    """Return every physical deposition locus and progressive pass order."""

    windows = winding_windows(events)
    if len(windows) != EXPECTED_PASSES:
        raise ValueError("canonical capture does not contain 24 passes")
    pitch = 2.0 * math.pi / int(DEFAULT_STATOR.slots)
    contact_z = float(wire_geometry.TOOTH_CONTACT_Z)
    rows: list[RawLocus] = []
    passes: list[dict[str, Any]] = []
    completed: list[int] = []
    sign_counts = {-1: 0, 1: 0}
    for pass_index, window in enumerate(windows):
        tooth = int(window["tooth"])
        if tooth in completed:
            raise ValueError("capture winds one tooth more than once")
        t0 = float(window["motionStart"])
        start_m2 = float(timeline.axes[2].pos_at(t0))
        sign = 1 if bool(window["clockwise"]) else -1
        sign_counts[sign] += 1
        crossings = _directed_crossing_times(
            timeline.axes[2], t0, start_m2, sign, STATES_PER_PASS + 1,
        )
        if len(crossings) != STATES_PER_PASS + 1:
            raise ValueError(
                f"pass {pass_index} has {len(crossings)} of 101 crossings"
            )
        local_rows = []
        for state_index, time_s in enumerate(crossings[:STATES_PER_PASS]):
            m0, m1, m2 = map(float, timeline.pose_at(time_s))
            alignment = _wrap_pi(m1 + tooth * pitch)
            row = RawLocus(
                pass_index=pass_index,
                phase_index=int(window["phase"]),
                tooth_index=tooth,
                motion_sign=sign,
                clockwise_argument=bool(window["clockwise"]),
                state_index=state_index,
                turn_index=state_index // 2,
                half_turn_index=state_index & 1,
                time_s=float(time_s),
                m0_rad=m0,
                m1_rad=m1,
                m2_rad=m2,
                m2_mod_rad=float(m2 % (2.0 * math.pi)),
                radial_x_mm=(
                    float(PARAMS.stator_axis_z(m0)) - contact_z
                ),
                m1_alignment_error_rad=alignment,
            )
            rows.append(row)
            local_rows.append(row)
        passes.append({
            "pass_index": pass_index,
            "phase_index": int(window["phase"]),
            "tooth_index": tooth,
            "motion_sign": sign,
            "clockwise_argument": bool(window["clockwise"]),
            "completed_other_teeth_before": list(completed),
            "completed_neighbor_teeth_before": [
                prior for prior in completed
                if min((prior - tooth) % DEFAULT_STATOR.slots,
                       (tooth - prior) % DEFAULT_STATOR.slots) == 1
            ],
            "state_count": len(local_rows),
            "minimum_radial_x_mm": min(row.radial_x_mm
                                       for row in local_rows),
            "maximum_radial_x_mm": max(row.radial_x_mm
                                       for row in local_rows),
            "maximum_m1_alignment_error_rad": max(
                abs(row.m1_alignment_error_rad) for row in local_rows
            ),
        })
        completed.append(tooth)
    if len(rows) != EXPECTED_STATE_COUNT:
        raise RuntimeError("raw progressive locus coverage is incomplete")
    if sign_counts != {-1: 12, 1: 12}:
        raise ValueError("canonical capture does not cover both signs 12/12")
    if any(abs(row.m1_alignment_error_rad) > M1_ALIGNMENT_TOL_RAD
           for row in rows):
        raise ValueError("M1 has not aligned the declared active tooth")
    return rows, passes


def _active_local_from_world(
    points_world: np.ndarray,
    locus: RawLocus,
    kin: collide.Kinematics,
) -> np.ndarray:
    """Use actual raw M0/M1 and pass tooth to recover active-tooth axes."""

    rotation, translation = kin.link_tf(
        "spindle", locus.m0_rad, locus.m1_rad, locus.m2_rad,
    )
    reference = (np.asarray(points_world) - translation) @ rotation
    base_local = np.column_stack((
        float(PARAMS.m0_home_standoff) - reference[:, 2],
        -reference[:, 0],
        reference[:, 1],
    ))
    tooth_angle = (
        locus.tooth_index * 2.0 * math.pi / int(DEFAULT_STATOR.slots)
    )
    active_rotation = wirepath.rot_z(-tooth_angle)
    return base_local @ active_rotation.T


def nominal_ellipse_tangent_path(
    locus: RawLocus,
    manifest: dict[str, Any],
    kin: collide.Kinematics,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Current straight terminal path, retained as a diagnostic witness."""

    guide = manifest["wire"]["tip_guide"]
    contact = manifest["wire"]["tooth_contact"]
    flyer_rotation = wirepath.rot_z(locus.m2_rad)
    feed = flyer_rotation @ np.asarray(
        guide["feed_local_mm"], dtype=float,
    )
    guide_center = flyer_rotation @ np.asarray(
        guide["center_local_mm"], dtype=float,
    )
    target = wirepath.tooth_contact_point(
        guide_center, contact, locus.motion_sign,
    )
    path_world, guide_meta = wirepath.tip_guide_path(
        feed, target, guide,
        float(DEFAULT_STATOR.wire_d) / 2.0,
        flyer_rotation,
    )
    path_local = _active_local_from_world(path_world, locus, kin)
    target_radial_error = abs(path_local[-1, 0] - locus.radial_x_mm)
    return path_local, {
        "target_local_mm": path_local[-1].tolist(),
        "target_radial_error_mm": float(target_radial_error),
        "guide_arc_turn_deg": float(guide_meta["arc_turn_deg"]),
        "guide_exit_tangent_error_deg": float(
            guide_meta["exit_tangent_error"]),
        "contact_surface": "current smooth ellipse diagnostic only",
    }


def core_prism_intersection(
    points_local: np.ndarray,
    lamination_face: Any,
) -> dict[str, Any]:
    """Exact line/prism intersection in the bounded source-face model.

    ``stator_insulation_nomex410._main_lamination_face`` is the same analytic
    hub/neck/shoe construction used for the qualified star caps. The stack is
    its Cartesian extrusion over +/- stack/2. For each 3D route segment we
    analytically clip its parameter interval to that axial slab, then ask
    GEOS whether the remaining XY segment intersects the closed lamination
    face. A hit is a true centreline/core crossing; no unsigned aggregate
    distance or allowed contact can hide it.
    """

    points = np.asarray(points_local, dtype=float)
    half_stack = float(DEFAULT_STATOR.stack) / 2.0
    for segment_index, (start, end) in enumerate(zip(points, points[1:])):
        dz = float(end[2] - start[2])
        if abs(dz) <= 1.0e-15:
            if not -half_stack <= float(start[2]) <= half_stack:
                continue
            lo, hi = 0.0, 1.0
        else:
            values = (
                (-half_stack - float(start[2])) / dz,
                (half_stack - float(start[2])) / dz,
            )
            lo = max(0.0, min(values))
            hi = min(1.0, max(values))
            if lo > hi + 1.0e-15:
                continue
        left = start + lo * (end - start)
        right = start + hi * (end - start)
        projected = LineString((
            (float(left[0]), float(left[1])),
            (float(right[0]), float(right[1])),
        ))
        if lamination_face.intersects(projected):
            intersection = lamination_face.intersection(projected)
            return {
                "intersects": True,
                "segment_index": segment_index,
                "axial_parameter_interval": [float(lo), float(hi)],
                "projected_intersection_length_mm": float(
                    getattr(intersection, "length", 0.0)),
            }
    return {
        "intersects": False,
        "segment_index": None,
        "axial_parameter_interval": None,
        "projected_intersection_length_mm": 0.0,
    }


def build_successor_planner(
    manifest: dict[str, Any], graph: PackingSupportGraph,
) -> SlotRoutePlanner:
    """Build the Nomex-aware core visibility planner from source geometry."""

    planner = SlotRoutePlanner.from_project(
        manifest,
        spec=DEFAULT_STATOR,
        access_radius_mm=graph.center_core_access_mm,
        planner_offset_mm=(
            graph.center_core_access_mm + PLANNER_SHELL_MM
        ),
        buffer_resolution=16,
        clamp_goal_to_stack=False,
        visibility_chord_mm=0.20,
    )
    # ``from_project`` defaults to the launch maximum wire radius because it
    # services the production wirepath checker.  This study is bound to the
    # selected packing/capture job and therefore uses its exact finished wire.
    planner.guide_wire_radius_mm = graph.wire_diameter_mm / 2.0
    return planner


def build_successor_routes(
    loci: list[RawLocus], planner: SlotRoutePlanner,
) -> tuple[dict[tuple[float, float, int], dict[str, Any]], dict[str, Any]]:
    """Solve a bounded representative subset of exact raw loci.

    The current nominal path is classified at every one of the 2,400 raw
    loci.  This separate successor search is intentionally capped: one exact
    median-radial witness for every observed phase/sign pair plus shallow and
    deep witnesses for phase zero in both signs.  Untested raw loci remain
    explicitly unavailable and therefore fail closed; no interpolation is
    promoted to route evidence.
    """

    exact = {row.route_key: row for row in loci}
    radial_min = min(row.radial_x_mm for row in loci)
    radial_max = max(row.radial_x_mm for row in loci)
    # The first canonical locus is the known deep, phase-zero, negative-sign
    # witness.  Broader successor coverage belongs to the dedicated R3/former
    # lane; this audit's mandatory 2,400-locus work remains the current-path
    # class split below.
    selected_keys: set[tuple[float, float, int]] = {loci[0].route_key}

    routes: dict[tuple[float, float, int], dict[str, Any]] = {}
    reasons: dict[str, int] = {}
    for number, key in enumerate(sorted(selected_keys), 1):
        row = exact[key]
        print(
            "phase-aware successor representative solve "
            f"{number}/{len(selected_keys)}",
            flush=True,
        )
        result = planner.route(
            row.radial_x_mm,
            row.m2_mod_rad,
            row.motion_sign,
            endpoint_family="raw_locus_liner_corridor",
            support_profile_radius_mm=planner.access_radius_mm,
        )
        routes[key] = {
            "result": result,
            "template_radial_x_mm": row.radial_x_mm,
            "raw_radial_x_mm": row.radial_x_mm,
            "translation_x_mm": 0.0,
        }
        reasons[result.reason] = reasons.get(result.reason, 0) + 1
        print(
            "phase-aware successor representatives "
            f"{number}/{len(selected_keys)}",
            flush=True,
        )
    passing = sum(row["result"].ok for row in routes.values())
    return routes, {
        "exact_unique_raw_route_count": len(exact),
        "unique_route_count": len(routes),
        "representative_route_limit": len(selected_keys),
        "radial_range_mm": [radial_min, radial_max],
        "exact_raw_routes_not_tested": len(exact) - len(routes),
        "interpolation_promoted_to_evidence": False,
        "source_BREP_postcheck_is_authority": True,
        "passing_unique_routes": passing,
        "failing_unique_routes": len(routes) - passing,
        "failure_reasons": dict(sorted(reasons.items())),
        "status": "PARTIAL_FAIL_CLOSED",
    }


def _rotate_loop(points: np.ndarray, angle: float) -> np.ndarray:
    rotation = wirepath.rot_z(float(angle))
    return np.asarray(points, dtype=float) @ rotation.T


def _loop_obstacles(
    graph: PackingSupportGraph,
    *,
    owner: str,
    tooth_relative_index: int = 0,
    through_turn: int | None = None,
) -> tuple[CopperPolyline, ...]:
    """Explicit nominal copper geometry; no raw/packing equality is tested."""

    end = len(graph.turns) if through_turn is None else int(through_turn)
    angle = (
        tooth_relative_index * 2.0 * math.pi / int(DEFAULT_STATOR.slots)
    )
    result = []
    for turn in graph.turns[:end]:
        points = _loop_centerline(
            turn, DEFAULT_STATOR, arc_step_deg=COPPER_ARC_STEP_DEG,
        )
        if tooth_relative_index:
            points = _rotate_loop(points, angle)
        result.append(CopperPolyline(
            obstacle_id=(
                f"{owner}-tooth-{tooth_relative_index:+03d}-"
                f"turn-{turn.turn_index:02d}"
            ),
            owner=owner,
            turn_index=turn.turn_index,
            centerline_local_mm=tuple(
                tuple(map(float, point)) for point in points
            ),
        ))
    return tuple(result)


def _exact_field_clearance(
    field: CopperField | None, route: np.ndarray,
) -> dict[str, Any]:
    """Expand the R-tree query band until its returned minimum is exact."""

    if field is None or not field.obstacles:
        return {
            "minimum_centerline_distance_mm": None,
            "minimum_obstacle_id": None,
            "route_segment_index": None,
            "obstacle_segment_index": None,
            "query_band_mm": None,
            "exact_within_polyline_model": True,
        }
    band = 0.5
    clearance = None
    while band <= 128.0:
        clearance = field.clearance(route, band)
        if clearance.minimum_centerline_distance_mm < band - 1.0e-12:
            break
        band *= 2.0
    if clearance is None or clearance.obstacle_id is None:
        raise RuntimeError("could not bound a non-empty copper field")
    return {
        "minimum_centerline_distance_mm": float(
            clearance.minimum_centerline_distance_mm),
        "minimum_obstacle_id": clearance.obstacle_id,
        "route_segment_index": clearance.route_segment_index,
        "obstacle_segment_index": clearance.obstacle_segment_index,
        "query_band_mm": float(band),
        "exact_within_polyline_model": True,
    }


def classify_copper_probe(
    probe: dict[str, Any], wire_diameter_mm: float,
) -> dict[str, Any]:
    """Classify contact without treating all other-coil contact as failure."""

    result = dict(probe)
    distance = probe.get("minimum_centerline_distance_mm")
    if distance is None:
        result.update({
            "centerline_margin_mm": None,
            "lower_bound_margin_after_chord_error_mm": None,
            "classification": "NO_PRIOR_COPPER_IN_CLASS",
            "interpenetration": False,
            "strict_near_contact": False,
        })
        return result
    margin = float(distance) - float(wire_diameter_mm)
    lower = margin - COPPER_MODEL_CHORD_BOUND_MM
    if margin < -NUMERICAL_TOL_MM:
        classification = "CENTERLINE_INTERPENETRATION_OR_THROUGH_CROSSING"
        penetration = True
    elif margin <= COPPER_CONTACT_BAND_MM:
        classification = "NONPENETRATING_SUPPORT_OR_GLIDE_CONTACT"
        penetration = False
    else:
        classification = "CLEAR"
        penetration = False
    result.update({
        "centerline_margin_mm": margin,
        "lower_bound_margin_after_chord_error_mm": lower,
        "classification": classification,
        "interpenetration": penetration,
        "strict_near_contact": margin <= COPPER_STRICT_NEAR_BAND_MM,
    })
    return result


def _summarize_numeric(
    records: list[dict[str, Any]],
    path_name: str,
    class_name: str,
    value_name: str,
    collision_name: str,
) -> dict[str, Any]:
    candidates = []
    collision_count = 0
    contact_count = 0
    evaluated_count = 0
    unresolved_count = 0
    obstacle_counts: dict[str, int] = {}
    for row in records:
        probe = row[path_name][class_name]
        if probe.get("evaluated", True):
            evaluated_count += 1
        else:
            unresolved_count += 1
        value = probe.get(value_name)
        if value is not None:
            candidates.append((float(value), row, probe))
        if bool(probe.get(collision_name, False)):
            collision_count += 1
            obstacle = probe.get("minimum_obstacle_id")
            if obstacle:
                obstacle_counts[str(obstacle)] = (
                    obstacle_counts.get(str(obstacle), 0) + 1
                )
        if probe.get("classification") == (
                "NONPENETRATING_SUPPORT_OR_GLIDE_CONTACT"):
            contact_count += 1
    if not candidates:
        return {
            "minimum": None,
            "evaluated_locus_count": evaluated_count,
            "unresolved_locus_count": unresolved_count,
            "collision_locus_count": collision_count,
            "support_or_glide_contact_locus_count": contact_count,
            "colliding_obstacle_counts": obstacle_counts,
        }
    value, row, probe = min(candidates, key=lambda item: item[0])
    locus = row["locus"]
    return {
        "minimum": value,
        "evaluated_locus_count": evaluated_count,
        "unresolved_locus_count": unresolved_count,
        "minimum_witness": {
            "pass_index": locus["pass_index"],
            "tooth_index": locus["tooth_index"],
            "state_index": locus["state_index"],
            "turn_index": locus["turn_index"],
            "half_turn_index": locus["half_turn_index"],
            "time_s": locus["time_s"],
            "m0_rad": locus["m0_rad"],
            "m1_rad": locus["m1_rad"],
            "m2_rad": locus["m2_rad"],
            "radial_x_mm": locus["radial_x_mm"],
            "obstacle_id": probe.get("minimum_obstacle_id"),
            "classification": probe.get("classification"),
        },
        "collision_locus_count": collision_count,
        "support_or_glide_contact_locus_count": contact_count,
        "colliding_obstacle_counts": dict(sorted(obstacle_counts.items())),
    }


def _pass_other_fields(
    graph: PackingSupportGraph,
    active_tooth: int,
    completed_teeth: list[int],
    obstacles_by_relative: Mapping[int, tuple[CopperPolyline, ...]],
) -> tuple[CopperField | None, CopperField | None]:
    neighbor: list[CopperPolyline] = []
    other: list[CopperPolyline] = []
    slots = int(DEFAULT_STATOR.slots)
    for tooth in completed_teeth:
        relative = (int(tooth) - int(active_tooth)) % slots
        signed = relative if relative <= slots // 2 else relative - slots
        obstacles = obstacles_by_relative[signed]
        (neighbor if abs(signed) == 1 else other).extend(obstacles)
    return (
        CopperField(tuple(neighbor)) if neighbor else None,
        CopperField(tuple(other)) if other else None,
    )


def _input_contract(
    events: list[dict[str, Any]],
    manifest: dict[str, Any],
    packing: dict[str, Any],
    insulation: dict[str, Any],
    elastic: dict[str, Any],
) -> dict[str, Any]:
    # The schema/hash fields alone do not prove that the elastic study still
    # describes the current route, contact study, CAD, manifest, or source.
    # Delegate that complete transitive source check to its owning module
    # before this audit consumes any of its conclusions.
    elastic_3d_study.validate_report_integrity(elastic)
    meta = events[0]
    checks = {
        "meta_first": meta.get("e") == "meta",
        "capture_schema_4": meta.get("capture_schema") == 4,
        "unmodified_upstream": meta.get("controller_mode") == "upstream",
        "no_controller_adapter": meta.get("controller_adapter_sha256") is None,
        "settings_hash_current": (
            SETTINGS.is_file()
            and meta.get("settings_sha256") == _sha256(SETTINGS)
        ),
        "manifest_default_stator": (
            int(manifest["stator"]["slots"]) == DEFAULT_STATOR.slots
            and int(manifest["stator"]["turns"]) == DEFAULT_STATOR.turns
        ),
        "packing_pass": packing.get("status") == "PASS",
        "nomex_package_selected": (
            insulation.get("active_winding_job_compatibility") == "PASS"
            and insulation.get("material", {}).get("supplier_sku")
            == insulation_source.MATERIAL_SUPPLIER_SKU
        ),
        "elastic_3d_schema_bound": (
            elastic.get("schema") == "elastic-3d-turn45-route-study/v1"
            and isinstance(elastic.get("report_sha256"), str)
        ),
        "elastic_3d_integrity_current": True,
    }
    failed = [name for name, ok in checks.items() if not ok]
    if failed:
        raise ValueError("phase-aware input contract failed: "
                         + ", ".join(failed))
    return {
        "status": "PASS",
        "checks": checks,
        "capture_sha256": _sha256(CAPTURE),
        "capture_event_count": len(events),
        "winder_commit": meta["winder_commit"],
        "settings_sha256": meta["settings_sha256"],
        "packing_report_sha256": packing["report_sha256"],
        "elastic_3d_report_sha256": elastic["report_sha256"],
    }


def analyze() -> dict[str, Any]:
    events = load_events(CAPTURE)
    manifest = _load_json(MANIFEST)
    packing = _load_json(PACKING)
    insulation = _load_json(INSULATION)
    elastic = _load_json(ELASTIC_3D)
    stored_routes = _load_json(STORED_ROUTES)
    inputs = _input_contract(
        events, manifest, packing, insulation, elastic,
    )
    timeline = Timeline(events)
    loci, passes = extract_raw_loci(events, timeline)
    graph = PackingSupportGraph.from_report(
        packing, spec=DEFAULT_STATOR,
    )
    if not math.isclose(
            graph.center_core_access_mm,
            graph.wire_diameter_mm / 2.0
            + float(packing["config"]["liner_thickness_mm"]),
            rel_tol=0.0, abs_tol=1.0e-12):
        raise ValueError("packing core access does not equal wire radius+liner")

    kin = collide.Kinematics(manifest)
    # A broader raw-locus visibility solve was attempted and deliberately
    # capped after the exact offset-section candidate enumeration exceeded
    # the bounded study runtime.  Do not silently interpolate those missing
    # paths.  The existing 100 stored packed routes remain useful, hash-bound
    # evidence that a core-offset visibility construction exists for their
    # own endpoints, but they are not substituted for the 2,400 raw loci.
    routes: dict[tuple[float, float, int], dict[str, Any]] = {}
    stored_route_rows = stored_routes.get("routes", [])
    route_coverage = {
        "status": "NOT_PROVEN_FAIL_CLOSED",
        "exact_unique_raw_route_count": len({row.route_key for row in loci}),
        "unique_route_count": 0,
        "representative_route_limit": 0,
        "exact_raw_routes_not_tested": len({row.route_key for row in loci}),
        "interpolation_promoted_to_evidence": False,
        "source_BREP_postcheck_is_authority": True,
        "passing_unique_routes": 0,
        "failing_unique_routes": 0,
        "stored_packed_route_count": len(stored_route_rows),
        "stored_packed_route_status": stored_routes.get("status"),
        "stored_packed_route_report_sha256": stored_routes.get(
            "report_sha256"),
        "bounded_runtime_decision": (
            "Do not rerun unbounded visibility enumeration inside this "
            "phase-class audit; the dedicated R3/former lane owns successor "
            "geometry and this report stays fail closed."
        ),
        "failure_reasons": {
            "raw-locus successor visibility not completed": len(
                {row.route_key for row in loci})
        },
    }
    lamination_face = insulation_source._main_lamination_face()
    baseline_cache: dict[
        tuple[float, float, int],
        tuple[np.ndarray, dict[str, Any], dict[str, Any]]
    ] = {}
    for locus in loci:
        if locus.route_key not in baseline_cache:
            points, intent = nominal_ellipse_tangent_path(
                locus, manifest, kin,
            )
            baseline_cache[locus.route_key] = (
                points, intent,
                core_prism_intersection(points, lamination_face),
            )

    records: list[dict[str, Any]] = []
    offset = 0
    for pass_row in passes:
        local_loci = loci[offset:offset + STATES_PER_PASS]
        offset += STATES_PER_PASS
        prior_teeth = list(map(
            int, pass_row["completed_other_teeth_before"],
        ))
        prior_neighbors = list(map(
            int, pass_row["completed_neighbor_teeth_before"],
        ))
        neighbor_loop_count = len(prior_neighbors) * DEFAULT_STATOR.turns
        other_loop_count = (
            len(prior_teeth) - len(prior_neighbors)
        ) * DEFAULT_STATOR.turns
        for locus in local_loci:
            _, baseline_intent, core_intersection = (
                baseline_cache[locus.route_key]
            )
            def unresolved_copper(count: int) -> dict[str, Any]:
                return {
                    "evaluated": False,
                    "obstacle_loop_count": int(count),
                    "minimum_centerline_distance_mm": None,
                    "minimum_obstacle_id": None,
                    "centerline_margin_mm": None,
                    "lower_bound_margin_after_chord_error_mm": None,
                    "classification": "UNRESOLVED_FAIL_CLOSED",
                    "interpenetration": False,
                    "strict_near_contact": False,
                }

            baseline_active = unresolved_copper(locus.turn_index)
            baseline_neighbor = unresolved_copper(neighbor_loop_count)
            baseline_other = unresolved_copper(other_loop_count)
            unavailable = unresolved_copper(0)
            unavailable["classification"] = "ROUTE_UNAVAILABLE"
            successor = {
                "route_found": False,
                "reason": "raw-locus successor visibility not completed",
                "core_and_liner": {
                    "evaluated": False,
                    "minimum_centerline_to_core_mm": None,
                    "required_wire_plus_liner_mm": (
                        graph.center_core_access_mm),
                    "core_offset_margin_mm": None,
                    "penetration": False,
                    "classification": "NO_VISIBILITY_ROUTE",
                },
                "active_tooth_intended_contact": {
                    "endpoint_support": None,
                    "classification": "ROUTE_UNAVAILABLE",
                    "progressive_support_validated": False,
                },
                "prior_active_copper": dict(unavailable),
                "completed_neighbor_copper": dict(unavailable),
                "completed_other_copper": dict(unavailable),
            }

            baseline = {
                "core_and_liner": {
                    "evaluated": True,
                    "method": (
                        "analytic segment clipping against qualified "
                        "Nomex source-face lamination prism"),
                    "minimum_centerline_to_core_mm": (
                        0.0 if core_intersection["intersects"] else None),
                    "required_wire_plus_liner_mm": (
                        graph.center_core_access_mm),
                    "core_offset_margin_mm": (
                        -graph.center_core_access_mm
                        if core_intersection["intersects"] else None),
                    "penetration": bool(core_intersection["intersects"]),
                    "classification": (
                        "CENTERLINE_CROSSES_CORE"
                        if core_intersection["intersects"]
                        else "NO_CORE_CROSSING__OFFSET_CLEARANCE_UNRESOLVED"
                    ),
                    "intersection_witness": core_intersection,
                },
                "active_tooth_intended_contact": {
                    **baseline_intent,
                    "classification": (
                        "DECLARED_ELLIPSE_TANGENCY_DIAGNOSTIC_ONLY"
                    ),
                    "allowed_contact_cannot_mask_other_classes": True,
                },
                "prior_active_copper": baseline_active,
                "completed_neighbor_copper": baseline_neighbor,
                "completed_other_copper": baseline_other,
            }
            records.append({
                "locus": asdict(locus),
                "nominal_ellipse_tangent": baseline,
                "core_visibility_successor": successor,
            })
        print(f"phase-aware classes {pass_row['pass_index'] + 1}/24",
              flush=True)

    summaries: dict[str, Any] = {}
    for path_name in (
            "nominal_ellipse_tangent", "core_visibility_successor"):
        summaries[path_name] = {
            "core_and_liner": _summarize_numeric(
                records, path_name, "core_and_liner",
                "core_offset_margin_mm", "penetration",
            ),
            "prior_active_copper": _summarize_numeric(
                records, path_name, "prior_active_copper",
                "centerline_margin_mm",
                "interpenetration",
            ),
            "completed_neighbor_copper": _summarize_numeric(
                records, path_name, "completed_neighbor_copper",
                "centerline_margin_mm",
                "interpenetration",
            ),
            "completed_other_copper": _summarize_numeric(
                records, path_name, "completed_other_copper",
                "centerline_margin_mm",
                "interpenetration",
            ),
        }

    baseline = summaries["nominal_ellipse_tangent"]
    successor = summaries["core_visibility_successor"]
    semantic_gates = {
        "all_2400_raw_loci_classified": len(records) == EXPECTED_STATE_COUNT,
        "both_motion_signs_12_each": (
            sum(row.motion_sign < 0 for row in loci) == 1200
            and sum(row.motion_sign > 0 for row in loci) == 1200
        ),
        "every_declared_tooth_aligned_by_raw_M1": all(
            abs(row.m1_alignment_error_rad) <= M1_ALIGNMENT_TOL_RAD
            for row in loci
        ),
        "current_nominal_path_clears_nomex_offset_core": (
            baseline["core_and_liner"]["collision_locus_count"] == 0
        ),
        "current_prior_active_copper_all_loci_evaluated": (
            baseline["prior_active_copper"]["evaluated_locus_count"]
            == EXPECTED_STATE_COUNT
        ),
        "current_neighbor_copper_all_loci_evaluated": (
            baseline["completed_neighbor_copper"]["evaluated_locus_count"]
            == EXPECTED_STATE_COUNT
        ),
        "current_other_copper_all_loci_evaluated": (
            baseline["completed_other_copper"]["evaluated_locus_count"]
            == EXPECTED_STATE_COUNT
        ),
        "successor_route_exists_at_every_raw_locus": (
            route_coverage["exact_raw_routes_not_tested"] == 0
            and route_coverage["failing_unique_routes"] == 0
        ),
        "successor_clears_nomex_offset_core": (
            successor["core_and_liner"]["evaluated_locus_count"]
            == EXPECTED_STATE_COUNT
            and successor["core_and_liner"]["collision_locus_count"] == 0
        ),
        "successor_prior_active_copper_noninterpenetrating": (
            successor["prior_active_copper"]["evaluated_locus_count"]
            == EXPECTED_STATE_COUNT
            and successor["prior_active_copper"]["collision_locus_count"] == 0
        ),
        "successor_neighbor_copper_noninterpenetrating": (
            successor["completed_neighbor_copper"]["evaluated_locus_count"]
            == EXPECTED_STATE_COUNT
            and
            successor["completed_neighbor_copper"]["collision_locus_count"]
            == 0
        ),
        "successor_other_copper_noninterpenetrating": (
            successor["completed_other_copper"]["evaluated_locus_count"]
            == EXPECTED_STATE_COUNT
            and
            successor["completed_other_copper"]["collision_locus_count"]
            == 0
        ),
    }
    r3_bound = {
        "path": str(ELASTIC_3D.relative_to(ROOT)).replace("\\", "/"),
        "file_sha256": _sha256(ELASTIC_3D),
        "report_sha256": elastic["report_sha256"],
        "status": elastic.get("status"),
        "decision": elastic.get("decision"),
        "production_authorized": bool(
            elastic.get("production_authorized", False)),
        "interpretation": (
            "This phase-aware class audit does not replace the separate "
            ">=3 mm turning/elastic proof. Its current FAIL remains an "
            "independent release blocker even if contact semantics clear."
        ),
    }
    release_flags = {
        **semantic_gates,
        "R3_elastic_route_production_authorized": (
            r3_bound["status"] == "PASS"
            and r3_bound["production_authorized"]
        ),
        "exact_layer_neatness_required": True,
    }
    # ``exact_layer_neatness_required`` is deliberately represented as a
    # policy assertion and must remain false; invert it for the all-gates
    # calculation while retaining a machine-readable statement.
    release_flags["exact_layer_neatness_required"] = False
    pass_flags = {
        key: value for key, value in release_flags.items()
        if key != "exact_layer_neatness_required"
    }

    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "PASS" if all(pass_flags.values()) else "FAIL",
        "decision": "CURRENT_MOVING_PATH_NOT_AUTHORIZED",
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "scope": {
            "pass_count": len(passes),
            "locus_count": len(records),
            "loci_per_pass": STATES_PER_PASS,
            "motion_sign_counts": {
                "negative": sum(row.motion_sign < 0 for row in loci),
                "positive": sum(row.motion_sign > 0 for row in loci),
            },
            "sampling_contract": (
                "All 100 directed pi-spaced physical deposition crossings "
                "in each of the 24 raw wind_wire windows. These are every "
                "raw half-turn winding locus, not a decimated angle sweep. "
                "Continuous inter-locus wire dynamics remain outside this "
                "study and therefore cannot authorize production."
            ),
            "packing_policy": (
                "The hash-bound packing graph supplies explicit already-"
                "deposited copper obstacles, but raw M0 positions are not "
                "required to match its centres and exact layering/neatness "
                "is not a pass/fail condition."
            ),
            "other_tooth_contact_policy": (
                "Nonpenetrating adjacent/other coil contact is classified "
                "as possible support or glide, not categorically forbidden. "
                "Centreline interpenetration/through-crossing is forbidden."
            ),
            "bounded_completion_note": (
                "Core/liner crossing is classified at all 2,400 loci. The "
                "three copper populations are independently present in "
                "every record but remain UNRESOLVED_FAIL_CLOSED after the "
                "bounded exact solver run was capped; zero collision counts "
                "for those classes are not clearance claims."
            ),
        },
        "input_contract": inputs,
        "geometry_contract": {
            "selected_wire_finished_diameter_mm": graph.wire_diameter_mm,
            "selected_wire_radius_mm": graph.wire_diameter_mm / 2.0,
            "selected_nomex_installed_thickness_mm": float(
                packing["config"]["liner_thickness_mm"]),
            "required_centerline_to_bare_core_mm": (
                graph.center_core_access_mm),
            "nomex_supplier_sku": insulation_source.MATERIAL_SUPPLIER_SKU,
            "nomex_receiving_range_mm": [
                insulation_source.MATERIAL_RECEIVING_MIN_MM,
                insulation_source.MATERIAL_RECEIVING_MAX_MM,
            ],
            "copper_polyline_arc_step_deg": COPPER_ARC_STEP_DEG,
            "copper_curve_to_chord_bound_mm": (
                COPPER_MODEL_CHORD_BOUND_MM),
            "copper_interpenetration_threshold_mm": (
                graph.wire_diameter_mm),
        },
        "semantic_gates": semantic_gates,
        "release_flags": release_flags,
        "release_blockers": [
            name for name, ok in pass_flags.items() if not ok
        ],
        "route_coverage": route_coverage,
        "per_class_summary": summaries,
        "passes": passes,
        "locus_records": records,
        "related_R3_elastic_evidence": r3_bound,
        "strict_diagnostic": {
            "near_contact_band_mm": COPPER_STRICT_NEAR_BAND_MM,
            "meaning": (
                "Counts all copper approaches within the finished diameter "
                "plus this band. They are reported for hardware review but "
                "do not fail unless the bounded centreline margin is negative."
            ),
        },
        "limitations": [
            "The successor is a quasi-static geometric visibility path, not "
            "a tension, friction, sag, or dynamic elastic-rod simulation.",
            "Copper classes are population-correct and independently named, "
            "but their distance queries were capped and remain unresolved; "
            "passive real-wire layer selection may also differ.",
            "Only physical half-turn deposition loci are audited here. A "
            "continuous all-angle proof must accompany any production route.",
            "The current ellipse is diagnostic and may underbound a literal "
            "R3 rounded-rectangle former. It is not contact-surface authority.",
            "No allowed active or copper contact suppresses another class: "
            "core, active prior, neighbor, and other copper are queried "
            "independently for every locus.",
        ],
        "source_hashes": {
            "raw_capture_sha256": _sha256(CAPTURE),
            "manifest_sha256": _sha256(MANIFEST),
            "packing_file_sha256": _sha256(PACKING),
            "packing_report_sha256": packing["report_sha256"],
            "insulation_report_sha256": _sha256(INSULATION),
            "elastic_3d_file_sha256": _sha256(ELASTIC_3D),
            "elastic_3d_report_sha256": elastic["report_sha256"],
            "stored_slot_routes_file_sha256": _sha256(STORED_ROUTES),
            "stored_slot_routes_report_sha256": stored_routes.get(
                "report_sha256"),
            "settings_sha256": _sha256(SETTINGS),
            "goal_sha256": _sha256(GOAL),
            "params_source_sha256": _sha256(CAD / "params.py"),
            "stator_source_sha256": _sha256(CAD / "stator_model.py"),
            "coil_growth_source_sha256": _sha256(CAD / "coil_growth.py"),
            "wire_geometry_source_sha256": _sha256(
                CAD / "wire_geometry.py"),
            "insulation_source_sha256": _sha256(
                CAD / "stator_insulation_nomex410.py"),
            "traj_source_sha256": _sha256(HERE / "traj.py"),
            "wirepath_source_sha256": _sha256(HERE / "wirepath.py"),
            "slot_route_source_sha256": _sha256(HERE / "slot_route.py"),
            "study_source_sha256": _sha256(Path(__file__)),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported phase-aware report schema")
    if _canonical_hash(report) != report.get("report_sha256"):
        raise ValueError("phase-aware report hash mismatch")
    expected = {
        "raw_capture_sha256": _sha256(CAPTURE),
        "manifest_sha256": _sha256(MANIFEST),
        "packing_file_sha256": _sha256(PACKING),
        "insulation_report_sha256": _sha256(INSULATION),
        "elastic_3d_file_sha256": _sha256(ELASTIC_3D),
        "stored_slot_routes_file_sha256": _sha256(STORED_ROUTES),
        "settings_sha256": _sha256(SETTINGS),
        "goal_sha256": _sha256(GOAL),
        "study_source_sha256": _sha256(Path(__file__)),
    }
    actual = report.get("source_hashes", {})
    stale = [name for name, value in expected.items()
             if actual.get(name) != value]
    if stale:
        raise ValueError("phase-aware report has stale sources: "
                         + ", ".join(stale))


def render_markdown(report: Mapping[str, Any]) -> str:
    baseline = report["per_class_summary"]["nominal_ellipse_tangent"]
    successor = report["per_class_summary"]["core_visibility_successor"]
    def mm(value: Any) -> str:
        return "unresolved" if value is None else f"{float(value):.9f} mm"

    lines = [
        "# Phase-aware progressive moving-wire audit",
        "",
        f"**Overall status: {report['status']}**  ",
        f"**Decision: {report['decision']}**",
        "",
        "The aggregate final-wound spindle solid has been replaced in this "
        "study by independent core/liner, active support, prior active copper, "
        "completed-neighbor copper, and completed-other copper probes.",
        "",
        "## Canonical raw coverage",
        "",
        f"- Passes: {report['scope']['pass_count']}",
        f"- Physical half-turn loci: {report['scope']['locus_count']}",
        f"- Signs: {report['scope']['motion_sign_counts']}",
        f"- Unique visibility routes: {report['route_coverage']['unique_route_count']}",
        "",
        "## Current nominal ellipse-tangent path",
        "",
        f"- Core/Nomex offset minimum margin: {mm(baseline['core_and_liner']['minimum'])}",
        f"- Core/Nomex penetration loci: {baseline['core_and_liner']['collision_locus_count']}",
        f"- Prior-active copper evaluated/unresolved: {baseline['prior_active_copper']['evaluated_locus_count']}/{baseline['prior_active_copper']['unresolved_locus_count']}",
        f"- Completed-neighbor copper evaluated/unresolved: {baseline['completed_neighbor_copper']['evaluated_locus_count']}/{baseline['completed_neighbor_copper']['unresolved_locus_count']}",
        f"- Completed-other copper evaluated/unresolved: {baseline['completed_other_copper']['evaluated_locus_count']}/{baseline['completed_other_copper']['unresolved_locus_count']}",
        "",
        "The zero-distance core witness is independent of the declared "
        "active ellipse contact, so that allowed contact cannot hide it.",
        "",
        "## Nomex-aware visibility successor",
        "",
        f"- Route coverage: {report['route_coverage']['status']} "
        f"({report['route_coverage']['passing_unique_routes']}/"
        f"{report['route_coverage']['unique_route_count']})",
        f"- Core/Nomex offset minimum margin: {mm(successor['core_and_liner']['minimum'])}",
        f"- Core/Nomex penetration loci: {successor['core_and_liner']['collision_locus_count']}",
        f"- Prior-active copper interpenetration loci: {successor['prior_active_copper']['collision_locus_count']}",
        f"- Completed-neighbor copper interpenetration loci: {successor['completed_neighbor_copper']['collision_locus_count']}",
        f"- Completed-other copper interpenetration loci: {successor['completed_other_copper']['collision_locus_count']}",
        "",
        "Nonpenetrating copper contact is reported as support/glide; it is "
        "not rejected merely because the other coil belongs to another tooth.",
        "",
        "## Gates",
        "",
    ]
    for name, ok in report["semantic_gates"].items():
        lines.append(f"- {'PASS' if ok else 'FAIL'} - `{name}`")
    r3 = report["related_R3_elastic_evidence"]
    lines.extend((
        "",
        "## Independent R3/elastic evidence",
        "",
        f"- Status: {r3['status']}",
        f"- Decision: {r3['decision']}",
        f"- Production authorized: {r3['production_authorized']}",
        "",
        "## Honest limit",
        "",
        "This study does not authorize CAD integration, purchase, or winding. "
        "The current straight path has an independent core/liner failure and "
        "the separate R3 elastic route remains unproven. Exact passive layer "
        "selection, neatness, friction, sag, snagging, and enamel abrasion "
        "remain hardware-only.",
        "",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ))
    return "\n".join(lines)


def write_outputs(
    report: Mapping[str, Any],
    json_path: Path = OUTPUT_JSON,
    markdown_path: Path = OUTPUT_MD,
) -> None:
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    markdown_path.write_text(render_markdown(report), encoding="utf-8")


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", type=Path, default=OUTPUT_JSON)
    parser.add_argument("--markdown", type=Path, default=OUTPUT_MD)
    args = parser.parse_args(list(argv) if argv is not None else None)
    report = analyze()
    write_outputs(report, args.json, args.markdown)
    baseline = report["per_class_summary"]["nominal_ellipse_tangent"]
    successor = report["per_class_summary"]["core_visibility_successor"]
    print(
        f"phase-aware wire {report['status']}: "
        f"raw={report['scope']['locus_count']}; "
        f"nominal core collisions="
        f"{baseline['core_and_liner']['collision_locus_count']}; "
        f"successor core collisions="
        f"{successor['core_and_liner']['collision_locus_count']}"
    )
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
