"""Audit the smallest M0-following, M1-static active-sector terminal guide."""

from __future__ import annotations

from bisect import bisect_right
from copy import deepcopy
from dataclasses import asdict
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

import fcl
import numpy as np
import trimesh
from build123d import (
    Align,
    BuildSketch,
    Circle,
    Compound,
    Cylinder,
    Plane,
    Pos,
    Rot,
    Vertex,
    export_stl,
    sweep,
)


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
REVIEW = ROOT / "out" / "review"
CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
AGGREGATE = REPORTS / "permanent_cap_aggregate_authorization.json"
INTEGRATED_WIRE = REPORTS / "integrated_phase_aware_wire_path.json"
INTEGRATED_RELEASE = REPORTS / "integrated_release_candidate.json"
LOADS_REPORT = REPORTS / "loads.json"
M2_SELECTION_REPORT = REPORTS / "m2_normal_goal_drive_selection.json"
JSON_OUT = REPORTS / "carriage_active_sector_terminal_guide_audit.json"
MD_OUT = REPORTS / "carriage_active_sector_terminal_guide_audit.md"
LOCUS_OUT = REPORTS / "carriage_active_sector_terminal_guide_loci.json"
MANIFEST_OUT = REVIEW / "carriage_active_sector_terminal_guide.manifest.json"
AUDIT_CACHE = REVIEW / "carriage_active_sector_audit_cache"
FINAL_WOUND_MESH = (
    ROOT / "out" / "links" / "parts" / "spindle"
    / "stator_final_wound_envelope.stl"
)

for folder in (CAD, HERE):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

import carriage_active_sector_terminal_guide as guide  # noqa: E402
import collide  # noqa: E402
from continuous_conductor_route import _raw_shaft_wraps  # noqa: E402
import integrated_phase_aware_wire_path as integrated  # noqa: E402
from params import DEFAULT_STATOR, PARAMS  # noqa: E402
import permanent_cap_production_review as cap  # noqa: E402
from phase_aware_progressive_wire_audit import (  # noqa: E402
    RawLocus,
    extract_raw_loci,
)
import retained_flyer_peek_guide_successor as flyer  # noqa: E402
import integrated_release_candidate as release_candidate  # noqa: E402
import shared_annular_terminal_crown as predecessor  # noqa: E402
import shared_annular_terminal_crown_audit as old_audit  # noqa: E402
import stator_insulation_nomex410 as insulation  # noqa: E402
from traj import Timeline, load_events  # noqa: E402
import wire_geometry  # noqa: E402
import wirepath  # noqa: E402


SCHEMA = "carriage-active-sector-terminal-guide-audit/v1"
EXPECTED_LOCI = 2400
TENSION_N = 10.0
MAX_WIRE_RADIUS_MM = 0.25
MIN_WIRE_RADIUS_MM = 0.10
MINIMUM_CLEARANCE_MM = 2.0
FULL_MOTION_DM2_RAD = math.radians(2.0)
FULL_MOTION_DM1_RAD = math.radians(2.0)
FULL_MOTION_DM0_MM = 0.25
ARBITRARY_M1_STEP_DEG = 0.5
FULL_M2_YOKE_STEP_DEG = 0.25
PEEK_DENSITY_G_MM3 = 1.30e-3
ALUMINUM_DENSITY_G_MM3 = 2.70e-3
STEEL_DENSITY_G_MM3 = 7.85e-3
BRASS_DENSITY_G_MM3 = 8.50e-3
PEEK_PROFILE_TOLERANCE_MM = 0.10
ADJACENT_ISOLATION_RESERVE_MM = 0.05
RIGHT_SEAM_INSERTION_GAUGE_RADIUS_MM = 0.36
RIGHT_SEAM_RAY_SAMPLE_COUNT = 25
CONSERVATIVE_COIL_RADIUS_MM = 26.0
CONSERVATIVE_COIL_AXIAL_HALF_MM = 10.5
CAPTURE_GRID_MM = tuple(float(v) for v in np.linspace(-2.55, 2.55, 35))
CAPTURE_FILLET_RADIUS_MM = 3.25
BELL_ARC_STEP_DEG = 2.0
COLLISION_GEOMETRY_REVISION = (
    "active-sector-r39p2__physical-bell-root-sleeve-six-slug__"
    "L79-stock-D10-P30__short-cap-leadins__"
    "rev6-front-plane-outboard-coil-bypass-yoke"
)
if (
    release_candidate.COLLISION_GEOMETRY_REVISION
    != COLLISION_GEOMETRY_REVISION
):
    raise RuntimeError("candidate/audit collision geometry revision drift")
_CACHE_PROVENANCE: dict[str, dict[str, Any]] = {}


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(path)
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def _report_hash(report: Mapping[str, Any]) -> str:
    body = deepcopy(dict(report))
    body.pop("report_sha256", None)
    return _canonical_hash(body)


def _artifact_row(path: Path) -> dict[str, Any]:
    return {
        "path": str(path.relative_to(ROOT)).replace("\\", "/"),
        "exists": path.is_file(),
        "size_bytes": path.stat().st_size if path.is_file() else None,
        "file_sha256": _sha256(path) if path.is_file() else None,
    }


def _collision_cache_key(kind: str) -> str:
    mesh_folder = REVIEW / "carriage_active_sector_collision_meshes"
    prefixes = (
        "actual_fixed_active_sector_yoke_tower_hardware_",
        "actual_production_caps_short_leadins_hardware_",
        "incremental_short_leadins_only_",
        "final_integrated_L79_stock_D10_P30_PEEK_bell_six_slug_flyer_",
        "conservative_R26_axial21_coil_growth",
        "packaging_active_",
        "packaging_preexisting_carriage_",
        "packaging_static_",
        "front_plane_full_m2_",
    )
    mesh_hashes = {
        path.name: _sha256(path)
        for path in sorted(mesh_folder.glob("*.processed.stl"))
        if path.name.startswith(prefixes)
    }
    payload = {
        "kind": kind,
        "algorithm": "per-solid-FCL-relative-transform-v3",
        "collision_geometry_revision": COLLISION_GEOMETRY_REVISION,
        "guide_source": _sha256(
            CAD / "carriage_active_sector_terminal_guide.py"
        ),
        "flyer_source": _sha256(
            CAD / "retained_flyer_peek_guide_successor.py"
        ),
        "integrated_candidate_source": _sha256(
            CAD / "integrated_release_candidate.py"
        ),
        "generated_per_occurrence_mesh_sha256": mesh_hashes,
        "kinematics_manifest": _sha256(
            ROOT / "out" / "links" / "manifest.json"
        ),
        "capture": _sha256(CAPTURE),
        "final_wound": _sha256(FINAL_WOUND_MESH),
        "minimum_clearance_mm": MINIMUM_CLEARANCE_MM,
    }
    return _canonical_hash(payload)


def _read_collision_cache(kind: str) -> dict[str, Any] | None:
    path = AUDIT_CACHE / f"{kind}.json"
    key = _collision_cache_key(kind)
    if not path.exists():
        _CACHE_PROVENANCE[kind] = {
            "path": str(path.relative_to(ROOT)).replace("\\", "/"),
            "cache_key": key,
            "hit": False,
            "file_sha256": None,
        }
        return None
    value = _load(path)
    if value.get("cache_key") != key:
        _CACHE_PROVENANCE[kind] = {
            "path": str(path.relative_to(ROOT)).replace("\\", "/"),
            "cache_key": key,
            "hit": False,
            "file_sha256": _sha256(path),
            "stale_cache_key": value.get("cache_key"),
        }
        return None
    _CACHE_PROVENANCE[kind] = {
        "path": str(path.relative_to(ROOT)).replace("\\", "/"),
        "cache_key": key,
        "hit": True,
        "file_sha256": _sha256(path),
    }
    return value


def _write_collision_cache(kind: str, payload: Mapping[str, Any]) -> None:
    AUDIT_CACHE.mkdir(parents=True, exist_ok=True)
    value = {"cache_key": _collision_cache_key(kind), **dict(payload)}
    (AUDIT_CACHE / f"{kind}.json").write_text(
        json.dumps(value, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    path = AUDIT_CACHE / f"{kind}.json"
    _CACHE_PROVENANCE[kind] = {
        "path": str(path.relative_to(ROOT)).replace("\\", "/"),
        "cache_key": value["cache_key"],
        "hit": False,
        "written": True,
        "file_sha256": _sha256(path),
    }


def _unit(value: Sequence[float]) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    length = float(np.linalg.norm(result))
    if length <= 1.0e-12:
        raise ValueError("zero vector")
    return result / length


def _side_sign(locus: RawLocus) -> tuple[int, int]:
    side, sign = integrated._expected_port(locus)
    return (-1 if side == "left" else 1), int(sign)


def bell_fairlead_path(
    target_world: Sequence[float],
    locus: RawLocus,
    wire_radius_mm: float,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Exact selectable-meridian path on the physical PEEK exit bell."""

    wire_radius = float(wire_radius_mm)
    if not MIN_WIRE_RADIUS_MM <= wire_radius <= MAX_WIRE_RADIUS_MM:
        raise ValueError("wire radius is outside the audited range")
    rotation = wirepath.rot_z(locus.m2_rad)
    axis = _unit(rotation @ np.array([0.0, 1.0, 0.0]))
    throat = rotation @ np.array([
        0.0,
        flyer.BELL_THROAT_Y_MM,
        float(flyer.base.TIP_GUIDE_CENTER_Z_MM),
    ])
    target = np.asarray(target_world, dtype=float)
    relative = target - throat
    target_axial = float(np.dot(relative, axis))
    transverse = relative - target_axial * axis
    target_rho = float(np.linalg.norm(transverse))
    if target_rho <= 1.0e-9:
        raise ValueError("bell target lies on the outlet axis")
    meridian = transverse / target_rho

    path_radius = (
        flyer.BELL_CONTACT_SURFACE_RADIUS_MM + wire_radius
    )
    circle = np.array([0.0, flyer.BELL_CENTER_RADIAL_MM], dtype=float)
    entry = np.array([
        0.0, flyer.BELL_CENTER_RADIAL_MM - path_radius
    ])
    offset = float(entry[1])
    candidates = wirepath._circle_tangent_points_2d(
        np.array([target_axial, target_rho]), circle, path_radius
    )
    theta_entry = -math.pi / 2.0
    allowed = []
    for point in candidates:
        theta = math.atan2(point[1] - circle[1], point[0] - circle[0])
        turn = (theta - theta_entry) % (2.0 * math.pi)
        if (
            1.0e-9 < turn
            <= math.radians(flyer.BELL_SWEEP_DEG) + 1.0e-9
            and point[1] >= circle[1] - 1.0e-9
        ):
            allowed.append((turn, point))
    if not allowed:
        raise RuntimeError("physical PEEK bell has no tangent exit to target")
    turn, exit_point = min(allowed, key=lambda row: row[0])
    count = max(2, int(math.ceil(
        math.degrees(turn) / BELL_ARC_STEP_DEG
    )))

    def world(point: np.ndarray) -> np.ndarray:
        return throat + point[0] * axis + point[1] * meridian

    arc = [
        circle + path_radius * np.array([
            math.cos(theta_entry + turn * index / count),
            math.sin(theta_entry + turn * index / count),
        ])
        for index in range(count + 1)
    ]
    arc_world = np.asarray([world(point) for point in arc], dtype=float)
    # Bounded straight-bore handoff: the wire is centered at the end of the
    # last R3.25 elbow (Y=64), then a zero-slope cubic moves it to the
    # selected-meridian bore offset at the bell throat.  The complete curve
    # stays inside the straight ID0.60 bore and is tangent to both neighbors.
    handoff_start = rotation @ np.array([
        0.0,
        float(flyer.GUIDE_SECOND_BEND_CENTER_Y_MM),
        float(flyer.base.TIP_GUIDE_CENTER_Z_MM),
    ])
    handoff_length = (
        flyer.BELL_THROAT_Y_MM - flyer.GUIDE_SECOND_BEND_CENTER_Y_MM
    )
    handoff_count = max(8, int(math.ceil(handoff_length / 0.20)) + 1)
    handoff = []
    for index in range(handoff_count):
        q = index / (handoff_count - 1)
        smooth = 3.0 * q * q - 2.0 * q * q * q
        handoff.append(
            handoff_start
            + axis * (handoff_length * q)
            + meridian * (offset * smooth)
        )
    handoff = np.asarray(handoff, dtype=float)
    if float(np.linalg.norm(handoff[-1] - arc_world[0])) > 1.0e-8:
        raise RuntimeError("straight-bore handoff does not join bell entry")
    result = np.vstack([handoff[:-1], arc_world, target.reshape(1, 3)])
    handoff_min_radius = (
        math.inf if abs(offset) <= 1.0e-12
        else handoff_length * handoff_length / (6.0 * abs(offset))
    )
    theta_exit = theta_entry + turn
    analytic_exit_tangent = (
        -math.sin(theta_exit) * axis + math.cos(theta_exit) * meridian
    )
    return result, {
        "owner": "flyer_M2",
        "fairlead": "one_piece_axisymmetric_polished_PEEK_exit_bell",
        "surface_generatrix_radius_mm": (
            flyer.BELL_CONTACT_SURFACE_RADIUS_MM
        ),
        "wire_center_bend_radius_mm": path_radius,
        "turn_deg": math.degrees(turn),
        "physical_sweep_limit_deg": flyer.BELL_SWEEP_DEG,
        "entry_bore_center_offset_mm": offset,
        "straight_bore_handoff_length_mm": handoff_length,
        "straight_bore_handoff_min_radius_mm": handoff_min_radius,
        "straight_bore_handoff_point_count": len(handoff),
        "bell_arc_point_count": len(arc_world),
        "bore_to_bell_join_error_mm": float(
            np.linalg.norm(handoff[-1] - arc_world[0])
        ),
        "exit_tangent_error_deg": wirepath._angle_deg(
            target - arc_world[-1], analytic_exit_tangent
        ),
        "sampled_exit_chord_tangent_error_deg": wirepath._angle_deg(
            target - arc_world[-1], arc_world[-1] - arc_world[-2]
        ),
    }


def carriage_world(point: Sequence[float], locus: RawLocus) -> np.ndarray:
    """Actual M0-only transform for the fixed active sector."""

    local = np.asarray(point, dtype=float)
    return np.array([
        -local[1], local[2],
        float(PARAMS.stator_axis_z(locus.m0_rad)) - local[0],
    ])


def periodic_equivalence(loci: list[RawLocus]) -> dict[str, Any]:
    probes = (
        np.array([guide.CAPTURE_RADIUS_MM, 0.0, guide.CAPTURE_POINT_AXIAL_MM]),
        np.array([
            guide.FIXED_BOWL_X_MM,
            guide.PORT_TANGENTIAL_MM,
            guide.FIXED_BOWL_AXIAL_MM,
        ]),
        np.array([
            guide.HANDOFF_X_MM,
            -guide.PORT_TANGENTIAL_MM,
            -guide.LEADIN_HIGH_AXIAL_MM,
        ]),
    )
    maximum = 0.0
    witness = None
    errors = []
    for locus in loci:
        for probe_index, point in enumerate(probes):
            rotating = integrated._stator_local_to_world(point, locus)
            fixed = carriage_world(point, locus)
            error = float(np.linalg.norm(rotating - fixed))
            errors.append(round(error, 15))
            if error > maximum:
                maximum = error
                witness = {
                    "pass_index": locus.pass_index,
                    "state_index": locus.state_index,
                    "tooth_index": locus.tooth_index,
                    "probe_index": probe_index,
                    "m1_alignment_error_rad": locus.m1_alignment_error_rad,
                    "error_mm": error,
                }
    return {
        "proof": (
            "raw M1 + tooth_index*15deg cancels at every deposition locus; "
            "the active tooth maps to the M0-only carriage frame"
        ),
        "locus_count": len(loci),
        "probe_count_per_locus": len(probes),
        "comparison_count": len(errors),
        "maximum_machine_space_error_mm": maximum,
        "witness": witness,
        "error_array_sha256": _canonical_hash(errors),
        "tolerance_mm": 1.0e-6,
        "status": "PASS" if maximum <= 1.0e-6 else "FAIL",
    }


def _route_key(locus: RawLocus) -> tuple[Any, ...]:
    side, sign = _side_sign(locus)
    return (
        round(locus.radial_x_mm, 9), round(locus.m2_mod_rad, 9),
        locus.motion_sign, side, sign,
    )


@lru_cache(maxsize=4)
def _short_leadin_samples(side: int, sign: int) -> np.ndarray:
    """Sample the actual source BREP centerline from cap lane to handoff."""

    wire = guide._leadin_centerline(int(side), int(sign))
    count = 241 if int(side) > 0 else 121
    result = []
    for position in np.linspace(0.0, 1.0, count):
        point = wire.position_at(float(position))
        result.append((float(point.X), float(point.Y), float(point.Z)))
    return np.asarray(result, dtype=float)


@lru_cache(maxsize=1)
def _short_leadin_cap_binding_cases() -> dict[str, Any]:
    """Exact BREP seam and C1-axis proof for all side/end variants."""

    cases = []
    for sign in (-1, 1):
        lane = cap.lane_wire(sign)
        for side in (-1, 1):
            endpoint_name = guide.cap_lane_endpoint_name(side)
            endpoint = np.asarray(
                guide.cap_lane_endpoint(side, sign), dtype=float,
            )
            handoff = np.asarray(
                guide.leadin_handoff(side, sign), dtype=float,
            )
            centerline = guide._leadin_centerline(side, sign)
            start = centerline.position_at(0.0)
            end = centerline.position_at(1.0)
            start_vector = np.asarray(
                (float(start.X), float(start.Y), float(start.Z)), dtype=float,
            )
            end_vector = np.asarray(
                (float(end.X), float(end.Y), float(end.Z)), dtype=float,
            )
            start_tangent_value = centerline.tangent_at(0.0)
            end_tangent_value = centerline.tangent_at(1.0)
            start_tangent = np.asarray((
                float(start_tangent_value.X),
                float(start_tangent_value.Y),
                float(start_tangent_value.Z),
            ))
            end_tangent = np.asarray((
                float(end_tangent_value.X),
                float(end_tangent_value.Y),
                float(end_tangent_value.Z),
            ))
            centerline_edges = list(centerline.edges())
            actual_circle_radii = [
                float(edge.radius) for edge in centerline_edges
                if edge.geom_type.name == "CIRCLE"
            ]
            internal_position_errors = []
            internal_tangent_errors_deg = []
            for first_edge, second_edge in zip(
                centerline_edges, centerline_edges[1:],
            ):
                first_end_value = first_edge.position_at(1.0)
                second_start_value = second_edge.position_at(0.0)
                first_end = np.asarray((
                    float(first_end_value.X), float(first_end_value.Y),
                    float(first_end_value.Z),
                ))
                second_start = np.asarray((
                    float(second_start_value.X),
                    float(second_start_value.Y),
                    float(second_start_value.Z),
                ))
                internal_position_errors.append(float(
                    np.linalg.norm(first_end - second_start)
                ))
                tangent_dot = float(first_edge.tangent_at(1.0).dot(
                    second_edge.tangent_at(0.0)
                ))
                internal_tangent_errors_deg.append(math.degrees(math.acos(
                    max(-1.0, min(1.0, tangent_dot))
                )))
            incident_tangents = []
            for edge in lane.edges():
                for position in (0.0, 1.0):
                    edge_point_value = edge.position_at(position)
                    edge_point = np.asarray((
                        float(edge_point_value.X),
                        float(edge_point_value.Y),
                        float(edge_point_value.Z),
                    ))
                    if float(np.linalg.norm(edge_point - endpoint)) <= 1.0e-8:
                        tangent_value = edge.tangent_at(position)
                        tangent = np.asarray((
                            float(tangent_value.X),
                            float(tangent_value.Y),
                            float(tangent_value.Z),
                        ))
                        incident_tangents.append(tangent)
            tangent_axis_errors = [
                1.0 - abs(float(np.dot(start_tangent, tangent)))
                for tangent in incident_tangents
            ]
            radii = [float(guide.LEADIN_CENTERLINE_RADIUS_MM)]
            if side > 0:
                radii.extend([
                    float(guide.RIGHT_S_BEND_RADIUS_MM),
                    float(guide.RIGHT_S_BEND_RADIUS_MM),
                ])
            checks = {
                "endpoint_is_on_actual_cap_lane_BREP": (
                    float(lane.distance_to(Vertex(tuple(endpoint))))
                    <= 1.0e-8
                ),
                "centerline_starts_at_named_cap_point": (
                    float(np.linalg.norm(start_vector - endpoint)) <= 1.0e-8
                ),
                "centerline_ends_at_unchanged_fixed_handoff": (
                    float(np.linalg.norm(end_vector - handoff)) <= 1.0e-8
                ),
                "cap_and_leadin_tangent_axes_are_C1_collinear": (
                    len(incident_tangents) == 2
                    and max(tangent_axis_errors, default=math.inf) <= 1.0e-8
                ),
                "handoff_tangent_is_machine_local_plus_X": (
                    float(np.linalg.norm(
                        end_tangent - np.asarray((1.0, 0.0, 0.0))
                    )) <= 1.0e-8
                ),
                "all_named_centerline_radii_ge_R3p50": min(radii) >= 3.5,
                "actual_BREP_circle_radii_match_named_contract": (
                    len(actual_circle_radii) == len(radii)
                    and max((
                        abs(actual - named) for actual, named
                        in zip(actual_circle_radii, radii)
                    ), default=math.inf) <= 1.0e-8
                ),
                "every_internal_edge_seam_is_C0": max(
                    internal_position_errors, default=math.inf,
                ) <= 1.0e-8,
                "every_internal_edge_seam_is_C1": max(
                    internal_tangent_errors_deg, default=math.inf,
                ) <= 1.0e-7,
            }
            cases.append({
                "side": "left" if side < 0 else "right",
                "side_sign": side,
                "axial_end": "rear" if sign < 0 else "front",
                "axial_sign": sign,
                "cap_endpoint_name": f"_lane_points()['{endpoint_name}']",
                "cap_endpoint_local_mm": endpoint.tolist(),
                "fixed_handoff_local_mm": handoff.tolist(),
                "actual_cap_lane_BREP_gap_mm": float(
                    lane.distance_to(Vertex(tuple(endpoint)))
                ),
                "start_position_error_mm": float(
                    np.linalg.norm(start_vector - endpoint)
                ),
                "handoff_position_error_mm": float(
                    np.linalg.norm(end_vector - handoff)
                ),
                "incident_cap_edge_count": len(incident_tangents),
                "maximum_C1_tangent_axis_error": max(
                    tangent_axis_errors, default=math.inf,
                ),
                "named_centerline_radii_mm": radii,
                "actual_BREP_circle_radii_mm": actual_circle_radii,
                "minimum_named_centerline_radius_mm": min(radii),
                "maximum_internal_edge_position_error_mm": max(
                    internal_position_errors, default=math.inf,
                ),
                "maximum_internal_edge_tangent_error_deg": max(
                    internal_tangent_errors_deg, default=math.inf,
                ),
                "checks": checks,
                "status": "PASS" if all(checks.values()) else "FAIL",
            })
    return {
        "authority": (
            "actual permanent_cap_production_review.lane_wire BREP and "
            "actual carriage_active_sector_terminal_guide centerline BREP"
        ),
        "case_count": len(cases),
        "cases": cases,
        "status": "PASS" if all(
            row["status"] == "PASS" for row in cases
        ) else "FAIL",
    }


@lru_cache(maxsize=4)
def _short_leadin_max_wire_envelope(side: int, sign: int):
    """Exact R0.25 sweep on one actual short-leadin centerline."""

    start = guide.cap_lane_endpoint(side, sign)
    with BuildSketch(Plane(
        origin=start,
        x_dir=(1.0, 0.0, 0.0),
        z_dir=(0.0, 0.0, float(sign)),
    )) as profile:
        Circle(MAX_WIRE_RADIUS_MM)
    return sweep(
        profile.sketch, guide._leadin_centerline(side, sign),
    )


@lru_cache(maxsize=1)
def adjacent_short_leadin_isolation() -> dict[str, Any]:
    """Prove global groove cuts preserve wire paths and separator webs."""

    cases = []
    for sign in (-1, 1):
        negative_parts = {
            (tooth, side): guide._channel_negative_for_tooth(
                tooth, side, sign,
            )
            for tooth in range(guide.SLOTS) for side in (-1, 1)
        }
        comparisons = []
        # Exact rotational reduction: R0 versus every R/L offset plus L0
        # versus every nonzero L offset covers all unordered tool classes.
        reference_right = negative_parts[(0, 1)]
        reference_left = negative_parts[(0, -1)]
        for tooth in range(1, guide.SLOTS):
            comparisons.append((
                "right_right", tooth,
                float(reference_right.distance_to(
                    negative_parts[(tooth, 1)]
                )),
            ))
            comparisons.append((
                "left_left", tooth,
                float(reference_left.distance_to(
                    negative_parts[(tooth, -1)]
                )),
            ))
        for tooth in range(guide.SLOTS):
            comparisons.append((
                "right_left", tooth,
                float(reference_right.distance_to(
                    negative_parts[(tooth, -1)]
                )),
            ))
        minimum_negative_gap = min(row[2] for row in comparisons)
        gap_witness = min(comparisons, key=lambda row: row[2])

        right_outer = guide._outer_channel_for_tooth(0, 1, sign)
        left_outer = guide._outer_channel_for_tooth(1, -1, sign)
        right_negative = negative_parts[(0, 1)]
        left_negative = negative_parts[(1, -1)]
        positive_common = right_outer & left_outer
        positive_overlap = (
            0.0 if positive_common is None
            else float(positive_common.volume)
        )
        globally_cut_pair = right_outer.fuse(left_outer).cut(
            right_negative, left_negative,
        )
        right_wire = _short_leadin_max_wire_envelope(1, sign)
        left_wire = (
            Rot(0.0, 0.0, guide.PITCH_DEG)
            * _short_leadin_max_wire_envelope(-1, sign)
        )
        right_wire_intrusion = float(
            (right_wire & globally_cut_pair).volume
        )
        left_wire_intrusion = float(
            (left_wire & globally_cut_pair).volume
        )
        intended_wire_reserve = (
            guide.LEADIN_CLEAR_RADIUS_MM
            - MAX_WIRE_RADIUS_MM - PEEK_PROFILE_TOLERANCE_MM
        )
        separator_reserve = (
            minimum_negative_gap - 2.0 * PEEK_PROFILE_TOLERANCE_MM
        )
        checks = {
            "all_distinct_groove_and_access_negatives_are_disjoint": (
                minimum_negative_gap > 0.0
            ),
            "adjacent_outer_shells_form_positive_separator_web": (
                positive_overlap > 1.0e-8
            ),
            "globally_cut_representative_pair_is_one_solid": (
                len(list(globally_cut_pair.solids())) == 1
            ),
            "right_max_wire_envelope_clears_globally_cut_pair": (
                right_wire_intrusion <= 1.0e-8
            ),
            "left_max_wire_envelope_clears_globally_cut_pair": (
                left_wire_intrusion <= 1.0e-8
            ),
            "profile_tolerance_retains_intended_wire_reserve": (
                intended_wire_reserve
                >= ADJACENT_ISOLATION_RESERVE_MM - 1.0e-12
            ),
            "two_sided_profile_tolerance_retains_separator_web": (
                separator_reserve
                >= ADJACENT_ISOLATION_RESERVE_MM - 1.0e-12
            ),
            "access_slot_is_wider_than_max_supported_wire": (
                guide.LEADIN_OPENING_WIDTH_MM >= 2.0 * MAX_WIRE_RADIUS_MM
            ),
            "adjacent_intended_max_wire_envelopes_do_not_touch": (
                float(right_wire.distance_to(left_wire)) > 0.0
            ),
        }
        cases.append({
            "axial_end": "rear" if sign < 0 else "front",
            "axial_sign": sign,
            "representative_pair": "tooth_00_right_to_tooth_01_left",
            "all_24_transform": (
                "exact relative tooth offsets cover all 24 rotations"
            ),
            "boolean_order": (
                "fuse cap and all R1.20 positive shells; subtract every "
                "R0.45 groove plus radial-outward access negative globally"
            ),
            "access_orientation": (
                "local +X radial-outward before exact tooth rotation"
            ),
            "open_channel_outer_radius_mm": guide.LEADIN_OUTER_RADIUS_MM,
            "open_channel_clear_radius_mm": guide.LEADIN_CLEAR_RADIUS_MM,
            "maximum_supported_wire_radius_mm": MAX_WIRE_RADIUS_MM,
            "profile_tolerance_mm": PEEK_PROFILE_TOLERANCE_MM,
            "required_post_tolerance_reserve_mm": (
                ADJACENT_ISOLATION_RESERVE_MM
            ),
            "negative_tool_pair_count_checked": len(comparisons),
            "minimum_distinct_negative_tool_gap_mm": minimum_negative_gap,
            "minimum_negative_tool_gap_witness": {
                "pair_class": gap_witness[0],
                "tooth_offset": gap_witness[1],
            },
            "separator_outer_shell_positive_overlap_mm3": positive_overlap,
            "globally_cut_representative_pair_solid_count": len(list(
                globally_cut_pair.solids()
            )),
            "intended_wire_reserve_after_profile_tolerance_mm": (
                intended_wire_reserve
            ),
            "separator_web_after_two_sided_profile_tolerance_mm": (
                separator_reserve
            ),
            "right_max_wire_to_globally_cut_pair_intrusion_mm3": (
                right_wire_intrusion
            ),
            "left_max_wire_to_globally_cut_pair_intrusion_mm3": (
                left_wire_intrusion
            ),
            "adjacent_max_wire_envelope_distance_mm": float(
                right_wire.distance_to(left_wire)
            ),
            "checks": checks,
            "status": "PASS" if all(checks.values()) else "FAIL",
        })
    return {
        "authority": (
            "exact globally ordered positive/negative BREP booleans plus "
            "exact R0.25 swept max-wire envelopes"
        ),
        "representative_case_count": len(cases),
        "physical_adjacent_pair_count": 2 * guide.SLOTS,
        "cases": cases,
        "status": "PASS" if all(
            row["status"] == "PASS" for row in cases
        ) else "FAIL",
    }


def _right_seam_insertion_gauge(tooth: int, sign: int):
    """R0.36 flat-ended radial cylinder through one finished mouth."""

    x, y, z = guide.cap_lane_endpoint(1, sign)
    local = (
        Pos(x, y, z)
        * Rot(0.0, 90.0, 0.0)
        * Cylinder(
            RIGHT_SEAM_INSERTION_GAUGE_RADIUS_MM,
            guide.RIGHT_SEAM_MOUTH_RADIAL_LENGTH_MM,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
    )
    return Rot(0.0, 0.0, tooth * guide.PITCH_DEG) * local


@lru_cache(maxsize=1)
def right_seam_final_brep_accessibility() -> dict[str, Any]:
    """Test all 24x2 finished-cap right mouths, not just source tools."""

    seam_rows = []
    cap_rows = []
    for sign in (-1, 1):
        before = guide._cap_with_short_leadins_before_right_seam_mouth(sign)
        final = guide.cap_with_short_leadins(sign)
        removed = float(before.volume - final.volume)
        cap_solid_count = len(list(final.solids()))
        cap_rows.append({
            "axial_end": "rear" if sign < 0 else "front",
            "axial_sign": sign,
            "right_mouth_count": guide.SLOTS,
            "left_mouth_count": 0,
            "solid_count_after_all_24_right_mouth_cuts": cap_solid_count,
            "total_new_right_mouth_removed_volume_mm3": removed,
            "removed_volume_per_right_seam_mm3": removed / guide.SLOTS,
            "status": "PASS" if (
                cap_solid_count == 1 and removed > 0.0
            ) else "FAIL",
        })
        local_x, local_y, local_z = guide.cap_lane_endpoint(1, sign)
        for tooth in range(guide.SLOTS):
            gauge = _right_seam_insertion_gauge(tooth, sign)
            common = final & gauge
            gauge_intrusion = (
                0.0 if common is None else float(common.volume)
            )
            angle = math.radians(tooth * guide.PITCH_DEG)
            cosine = math.cos(angle)
            sine = math.sin(angle)
            ray_inside_count = 0
            for distance in np.linspace(
                0.01,
                guide.RIGHT_SEAM_MOUTH_RADIAL_LENGTH_MM - 0.01,
                RIGHT_SEAM_RAY_SAMPLE_COUNT,
            ):
                x = local_x + float(distance)
                point = (
                    x * cosine - local_y * sine,
                    x * sine + local_y * cosine,
                    local_z,
                )
                ray_inside_count += int(final.is_inside(point, 1.0e-7))
            checks = {
                "R0p36_radial_insertion_gauge_zero_positive_overlap": (
                    gauge_intrusion <= 1.0e-8
                ),
                "radial_center_ray_cross_section_is_open": (
                    ray_inside_count == 0
                ),
                "mouth_tangential_half_width_exceeds_R0p36": (
                    guide.RIGHT_SEAM_MOUTH_TANGENTIAL_WIDTH_MM / 2.0
                    > RIGHT_SEAM_INSERTION_GAUGE_RADIUS_MM
                ),
                "mouth_axial_half_span_exceeds_R0p36": (
                    guide.RIGHT_SEAM_MOUTH_AXIAL_SPAN_MM / 2.0
                    > RIGHT_SEAM_INSERTION_GAUGE_RADIUS_MM
                ),
                "final_cap_is_one_solid": cap_solid_count == 1,
            }
            seam_rows.append({
                "tooth_index": tooth,
                "axial_end": "rear" if sign < 0 else "front",
                "axial_sign": sign,
                "cap_endpoint_name": "_lane_points()['waypoint']",
                "insertion_direction": (
                    "local +X radial-outward after tooth rotation"
                ),
                "gauge_radius_mm": RIGHT_SEAM_INSERTION_GAUGE_RADIUS_MM,
                "gauge_radial_length_mm": (
                    guide.RIGHT_SEAM_MOUTH_RADIAL_LENGTH_MM
                ),
                "gauge_to_final_cap_positive_overlap_mm3": gauge_intrusion,
                "open_ray_sample_count": RIGHT_SEAM_RAY_SAMPLE_COUNT,
                "open_ray_inside_sample_count": ray_inside_count,
                "checks": checks,
                "status": "PASS" if all(checks.values()) else "FAIL",
            })
    all_pass = (
        len(seam_rows) == 2 * guide.SLOTS
        and all(row["status"] == "PASS" for row in seam_rows)
        and all(row["status"] == "PASS" for row in cap_rows)
    )
    return {
        "authority": (
            "finished cap_with_short_leadins BREP after all global groove "
            "and right seam mouth cuts"
        ),
        "mouth_tool": {
            "radial_length_mm": guide.RIGHT_SEAM_MOUTH_RADIAL_LENGTH_MM,
            "tangential_width_mm": (
                guide.RIGHT_SEAM_MOUTH_TANGENTIAL_WIDTH_MM
            ),
            "axial_span_mm": guide.RIGHT_SEAM_MOUTH_AXIAL_SPAN_MM,
            "cap_side_tangent_overlap_mm": (
                guide.RIGHT_SEAM_MOUTH_CAP_SIDE_OVERLAP_MM
            ),
            "right_only": True,
        },
        "seam_count": len(seam_rows),
        "cap_rows": cap_rows,
        "seams": seam_rows,
        "maximum_gauge_positive_overlap_mm3": max(
            row["gauge_to_final_cap_positive_overlap_mm3"]
            for row in seam_rows
        ),
        "maximum_open_ray_inside_sample_count": max(
            row["open_ray_inside_sample_count"] for row in seam_rows
        ),
        "status": "PASS" if all_pass else "FAIL",
    }


_TEMPLATE_LOCI: dict[tuple[Any, ...], RawLocus] = {}


@lru_cache(maxsize=1)
def _flyer_bore_to_tensioned_handoff_local() -> np.ndarray:
    """Exact geometric bore centerline through the last R3.25 elbow."""

    points = np.asarray(
        flyer.guide_bore_centerline_samples(0.50), dtype=float
    )
    keep = points[:, 1] <= (
        flyer.GUIDE_SECOND_BEND_CENTER_Y_MM + 1.0e-9
    )
    result = points[keep]
    expected = np.array([
        0.0,
        flyer.GUIDE_SECOND_BEND_CENTER_Y_MM,
        float(flyer.base.TIP_GUIDE_CENTER_Z_MM),
    ])
    if float(np.linalg.norm(result[-1] - expected)) > 1.0e-8:
        raise RuntimeError("flyer geometric bore/handoff seam drifted")
    return result


def _candidate(locus: RawLocus, capture_y: float) -> dict[str, Any]:
    side, sign = _side_sign(locus)
    capture = np.array([
        guide.CAPTURE_RADIUS_MM, capture_y,
        sign * guide.CAPTURE_POINT_AXIAL_MM,
    ])
    selection = np.array([
        guide.FIXED_BOWL_X_MM, side * guide.PORT_TANGENTIAL_MM,
        sign * guide.FIXED_BOWL_AXIAL_MM,
    ])
    direction = selection - capture
    target = carriage_world(capture, locus)
    tip_path, tip_meta = bell_fairlead_path(
        target, locus, MAX_WIRE_RADIUS_MM,
    )
    # M0-only fixed frame has the same inverse coordinate map as normalized
    # active stator local after periodicity cancellation.
    local_tip = integrated._world_to_active_local(tip_path, locus)
    first, meta1 = wire_geometry._circular_fillet(
        tuple(capture), tuple(local_tip[-1] - local_tip[-2]),
        tuple(direction), CAPTURE_FILLET_RADIUS_MM, step_deg=2.0,
    )
    second, meta2 = wire_geometry._circular_fillet(
        tuple(selection), tuple(direction), (-1.0, 0.0, 0.0),
        CAPTURE_FILLET_RADIUS_MM, step_deg=2.0,
    )
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    straight = float(np.linalg.norm(
        np.asarray(meta2["start"]) - np.asarray(meta1["end"])
    ))
    if straight <= 0.1:
        raise ValueError("fillet trims consume free segment")
    return {
        "capture": capture,
        "selection": selection,
        "local_tip": local_tip,
        "tip_path_world": tip_path,
        "tip_meta": tip_meta,
        "first": first,
        "meta1": meta1,
        "second": second,
        "meta2": meta2,
        "straight_mm": straight,
        "mouth_half_mm": float(np.max(np.abs(first[:, 1]))) + MAX_WIRE_RADIUS_MM,
    }


@lru_cache(maxsize=4096)
def _template(key: tuple[Any, ...]) -> dict[str, Any]:
    locus = _TEMPLATE_LOCI[key]
    best = None
    for capture_y in CAPTURE_GRID_MM:
        try:
            candidate = _candidate(locus, capture_y)
        except (RuntimeError, ValueError):
            continue
        score = (candidate["mouth_half_mm"], -candidate["straight_mm"])
        if best is None or score < best[0]:
            best = (score, capture_y, candidate)
    if best is None:
        raise RuntimeError("no active-sector route")
    _score, capture_y, candidate = best
    side, sign = _side_sign(locus)
    branch_y = side * guide.PORT_TANGENTIAL_MM
    handoff_count = int(
        candidate["tip_meta"]["straight_bore_handoff_point_count"]
    )
    bell_count = int(candidate["tip_meta"]["bell_arc_point_count"])
    local_tip = np.asarray(candidate["local_tip"], dtype=float)
    tensioned_handoff = local_tip[:handoff_count]
    bell_arc = local_tip[
        handoff_count - 1:handoff_count - 1 + bell_count
    ]
    bore_seam_world = (
        wirepath.rot_z(locus.m2_rad)
        @ _flyer_bore_to_tensioned_handoff_local()[-1]
    )
    if float(np.linalg.norm(
        bore_seam_world
        - np.asarray(candidate["tip_path_world"], dtype=float)[0]
    )) > 1.0e-8:
        raise RuntimeError("geometric bore/tensioned handoff seam drifted")
    first = np.asarray(candidate["first"], dtype=float)
    second = np.asarray(candidate["second"], dtype=float)
    handoff = np.asarray(guide.leadin_handoff(side, sign), dtype=float)
    if side < 0:
        bend_center = np.array([
            guide.LEADIN_BEND_X_MM, branch_y,
            sign * guide.LEADIN_BEND_CENTER_AXIAL_MM,
        ])
        bend = []
        for q in np.linspace(0.0, math.pi / 2.0, 55):
            bend.append(bend_center + np.array([
                -guide.LEADIN_CENTERLINE_RADIUS_MM * math.sin(q), 0.0,
                sign * guide.LEADIN_CENTERLINE_RADIUS_MM * math.cos(q),
            ]))
        port = np.asarray(
            guide.cap_lane_endpoint(side, sign), dtype=float,
        )
        leadin_high = np.array([
            guide.LEADIN_BEND_X_MM, branch_y,
            sign * guide.LEADIN_HIGH_AXIAL_MM,
        ])
        bend_array = np.asarray(bend, dtype=float)
        dynamic_handoff = np.vstack([
            second[-1], handoff, leadin_high,
        ])
        spindle_leadin = np.vstack([
            leadin_high, bend_array[1:], port,
        ])
        route_tail = np.vstack([
            handoff, leadin_high, bend_array[1:], port,
        ])
    else:
        # Reverse the actual source BREP centerline so the route continues
        # from fixed bowl -> spindle handoff -> named cap waypoint.
        spindle_leadin = _short_leadin_samples(side, sign)[::-1]
        port = spindle_leadin[-1]
        dynamic_handoff = np.vstack([second[-1], handoff])
        route_tail = spindle_leadin
    route = np.vstack([
        candidate["local_tip"][:-1],
        np.asarray(candidate["meta1"]["start"]),
        candidate["first"][1:],
        np.asarray(candidate["meta2"]["start"]),
        candidate["second"][1:],
        route_tail,
    ])
    return {
        "route": route,
        "capture": candidate["capture"],
        "selection": candidate["selection"],
        "capture_y_mm": float(capture_y),
        "mouth_half_mm": float(candidate["mouth_half_mm"]),
        "straight_mm": float(candidate["straight_mm"]),
        "capture_turn_deg": float(candidate["meta1"]["turn_deg"]),
        "selection_turn_deg": float(candidate["meta2"]["turn_deg"]),
        "bell_turn_deg": float(candidate["tip_meta"]["turn_deg"]),
        "bell_wire_center_radius_mm": float(
            candidate["tip_meta"]["wire_center_bend_radius_mm"]
        ),
        "bell_exit_tangent_error_deg": float(
            candidate["tip_meta"]["exit_tangent_error_deg"]
        ),
        "bell_sampled_exit_chord_error_deg": float(
            candidate["tip_meta"]["sampled_exit_chord_tangent_error_deg"]
        ),
        "bore_to_bell_join_error_mm": float(
            candidate["tip_meta"]["bore_to_bell_join_error_mm"]
        ),
        "straight_bore_handoff_min_radius_mm": float(
            candidate["tip_meta"]["straight_bore_handoff_min_radius_mm"]
        ),
        "segments_local": {
            "flyer_geometric_bore": (
                _flyer_bore_to_tensioned_handoff_local()
            ),
            "flyer_tensioned_bore_handoff": tensioned_handoff,
            "flyer_bell_meridian_arc": bell_arc,
            "dynamic_free_span_to_capture": np.vstack([
                bell_arc[-1], np.asarray(candidate["meta1"]["start"]),
            ]),
            "carriage_capture_fillet": first,
            "carriage_fixed_free_gap": np.vstack([
                first[-1], np.asarray(candidate["meta2"]["start"]),
            ]),
            "carriage_selection_bowl": second,
            "dynamic_handoff_gap": dynamic_handoff,
            "spindle_short_leadin": spindle_leadin,
        },
        "bell_exit_local": bell_arc[-1],
        "port_local": port,
        "side": side,
        "sign": sign,
    }


def route_for_locus(locus: RawLocus) -> dict[str, Any]:
    key = _route_key(locus)
    _TEMPLATE_LOCI.setdefault(key, locus)
    return _template(key)


def terminal_rows(loci: list[RawLocus]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    face = insulation._main_lamination_face()
    aggregate = _load(AGGREGATE)
    copper_axial = float(
        aggregate["cap_support_lane"]["finished_wire_total_axial_envelope_mm"]
    ) / 2.0
    binding = _short_leadin_cap_binding_cases()
    binding_by_sign = {
        (int(row["side_sign"]), int(row["axial_sign"])): row
        for row in binding["cases"]
    }
    rows = []
    for locus in loci:
        template = route_for_locus(locus)
        binding_case = binding_by_sign[(
            int(template["side"]), int(template["sign"]),
        )]
        route = np.asarray(template["route"], dtype=float)
        target_world = carriage_world(template["capture"], locus)
        _min_wire_path, min_wire_bell = bell_fairlead_path(
            target_world, locus, MIN_WIRE_RADIUS_MM,
        )
        core = integrated.core_prism_intersection(route, face)
        mouth_margin = (
            predecessor.CAPTURE_MOUTH_CLEAR_WIDTH_MM / 2.0
            - template["mouth_half_mm"]
        )
        axial_margin = (
            guide.LEADIN_BEND_CENTER_AXIAL_MM
            - copper_axial - MAX_WIRE_RADIUS_MM
        )
        transfer_min_radius = (
            guide.LEADIN_CENTERLINE_RADIUS_MM
            - (guide.LEADIN_CLEAR_RADIUS_MM - MIN_WIRE_RADIUS_MM)
        )
        gates = {
            "no_core": not core["intersects"],
            "mouth": mouth_margin >= 0.0,
            "straight": template["straight_mm"] > 0.0,
            "high_branch": axial_margin > 0.0,
            "transfer_R": transfer_min_radius >= 3.0,
            "cap_lane_BREP_seam": binding_case["status"] == "PASS",
            "M1_rigid_gap": guide.ARBITRARY_M1_RADIAL_CLEARANCE_MM >= 2.0,
            "physical_bell_R": (
                min(
                    template["bell_wire_center_radius_mm"],
                    min_wire_bell["wire_center_bend_radius_mm"],
                ) >= 3.25
            ),
            "physical_bell_sweep": (
                max(
                    template["bell_turn_deg"],
                    min_wire_bell["turn_deg"],
                ) <= flyer.BELL_SWEEP_DEG
            ),
            "bell_exit_tangent": (
                max(
                    template["bell_exit_tangent_error_deg"],
                    min_wire_bell["exit_tangent_error_deg"],
                ) <= 1.0e-5
            ),
            "bore_to_bell_continuity": (
                max(
                    template["bore_to_bell_join_error_mm"],
                    min_wire_bell["bore_to_bell_join_error_mm"],
                ) <= 1.0e-6
            ),
            "straight_bore_handoff_R": (
                min(
                    template["straight_bore_handoff_min_radius_mm"],
                    min_wire_bell[
                        "straight_bore_handoff_min_radius_mm"
                    ],
                ) >= 3.0
            ),
        }
        rows.append({
            "pass_index": locus.pass_index,
            "phase_index": locus.phase_index,
            "state_index": locus.state_index,
            "turn_index": locus.turn_index,
            "half_turn_index": locus.half_turn_index,
            "tooth_index": locus.tooth_index,
            "motion_sign": locus.motion_sign,
            "clockwise_argument": locus.clockwise_argument,
            "time_s": round(locus.time_s, 9),
            "m0_rad": round(locus.m0_rad, 12),
            "m1_rad": round(locus.m1_rad, 12),
            "m2_rad": round(locus.m2_rad, 12),
            "radial_x_mm": round(locus.radial_x_mm, 12),
            "capture_y_mm": round(template["capture_y_mm"], 12),
            "capture_local_mm": np.round(template["capture"], 12).tolist(),
            "selection_local_mm": np.round(template["selection"], 12).tolist(),
            "path_sha256": old_audit._path_hash(route),
            "path_point_count": len(route),
            "mouth_margin_mm": round(mouth_margin, 12),
            "free_straight_after_trims_mm": round(template["straight_mm"], 12),
            "capture_turn_deg": round(template["capture_turn_deg"], 12),
            "selection_turn_deg": round(template["selection_turn_deg"], 12),
            "bell_turn_deg": round(template["bell_turn_deg"], 12),
            "minimum_wire_bell_turn_deg": round(
                min_wire_bell["turn_deg"], 12
            ),
            "bell_wire_center_radius_mm": round(
                template["bell_wire_center_radius_mm"], 12
            ),
            "minimum_wire_bell_center_radius_mm": round(
                min_wire_bell["wire_center_bend_radius_mm"], 12
            ),
            "bell_exit_tangent_error_deg": round(
                template["bell_exit_tangent_error_deg"], 12
            ),
            "minimum_wire_bell_exit_tangent_error_deg": round(
                min_wire_bell["exit_tangent_error_deg"], 12
            ),
            "bore_to_bell_join_error_mm": round(
                template["bore_to_bell_join_error_mm"], 15
            ),
            "minimum_wire_bore_to_bell_join_error_mm": round(
                min_wire_bell["bore_to_bell_join_error_mm"], 15
            ),
            "straight_bore_handoff_min_radius_mm": round(
                template["straight_bore_handoff_min_radius_mm"], 12
            ),
            "minimum_wire_straight_bore_handoff_radius_mm": round(
                min_wire_bell["straight_bore_handoff_min_radius_mm"], 12
            ),
            "bell_exit_local_mm": np.round(
                template["bell_exit_local"], 12
            ).tolist(),
            "port_local_mm": np.round(
                template["port_local"], 12
            ).tolist(),
            "cap_endpoint_name": binding_case["cap_endpoint_name"],
            "cap_lane_BREP_seam_gap_mm": round(
                binding_case["actual_cap_lane_BREP_gap_mm"], 15,
            ),
            "short_leadin_minimum_named_centerline_radius_mm": round(
                binding_case["minimum_named_centerline_radius_mm"], 12,
            ),
            "high_branch_axial_margin_mm": round(axial_margin, 12),
            "transfer_min_radius_after_wander_mm": round(transfer_min_radius, 12),
            "core_intersection": bool(core["intersects"]),
            "route_gates_pass": all(gates.values()),
        })
    return rows, {
        "locus_count": len(rows),
        "unique_geometry_case_count": len(_TEMPLATE_LOCI),
        "pass_count": sum(row["route_gates_pass"] for row in rows),
        "failure_count": sum(not row["route_gates_pass"] for row in rows),
        "minimum_mouth_margin_mm": min(row["mouth_margin_mm"] for row in rows),
        "minimum_free_straight_after_trims_mm": min(
            row["free_straight_after_trims_mm"] for row in rows
        ),
        "maximum_capture_turn_deg": max(row["capture_turn_deg"] for row in rows),
        "maximum_selection_turn_deg": max(row["selection_turn_deg"] for row in rows),
        "minimum_bell_turn_deg": min(row["bell_turn_deg"] for row in rows),
        "maximum_bell_turn_deg": max(
            max(row["bell_turn_deg"], row["minimum_wire_bell_turn_deg"])
            for row in rows
        ),
        "minimum_bell_sweep_reserve_deg": min(
            flyer.BELL_SWEEP_DEG - max(
                row["bell_turn_deg"], row["minimum_wire_bell_turn_deg"]
            ) for row in rows
        ),
        "minimum_bell_wire_center_radius_mm": min(
            min(
                row["bell_wire_center_radius_mm"],
                row["minimum_wire_bell_center_radius_mm"],
            ) for row in rows
        ),
        "maximum_bell_exit_tangent_error_deg": max(
            max(
                row["bell_exit_tangent_error_deg"],
                row["minimum_wire_bell_exit_tangent_error_deg"],
            ) for row in rows
        ),
        "maximum_bore_to_bell_join_error_mm": max(
            max(
                row["bore_to_bell_join_error_mm"],
                row["minimum_wire_bore_to_bell_join_error_mm"],
            ) for row in rows
        ),
        "minimum_straight_bore_handoff_radius_over_0p20_to_0p50mm_wire_mm": min(
            min(
                row["straight_bore_handoff_min_radius_mm"],
                row["minimum_wire_straight_bore_handoff_radius_mm"],
            ) for row in rows
        ),
        "minimum_high_branch_axial_margin_mm": min(
            row["high_branch_axial_margin_mm"] for row in rows
        ),
        "maximum_cap_lane_BREP_seam_gap_mm": max(
            row["cap_lane_BREP_seam_gap_mm"] for row in rows
        ),
        "minimum_short_leadin_named_centerline_radius_mm": min(
            row["short_leadin_minimum_named_centerline_radius_mm"]
            for row in rows
        ),
        "cap_endpoint_counts": {
            name: sum(row["cap_endpoint_name"] == name for row in rows)
            for name in sorted({row["cap_endpoint_name"] for row in rows})
        },
        "arbitrary_M1_radial_rigid_clearance_mm": (
            guide.ARBITRARY_M1_RADIAL_CLEARANCE_MM
        ),
    }


def _shape_mesh(shape: Any, name: str) -> trimesh.Trimesh:
    """Tessellate and validate an actual BREP without filling holes."""

    folder = REVIEW / "carriage_active_sector_collision_meshes"
    folder.mkdir(parents=True, exist_ok=True)
    raw = folder / f"{name}.stl"
    processed = folder / f"{name}.processed.stl"
    sidecar = folder / f"{name}.geometry.json"
    geometry_sources = (
        CAD / "carriage_active_sector_terminal_guide.py",
        CAD / "retained_flyer_peek_guide_successor.py",
    )
    newest_source = max(path.stat().st_mtime for path in geometry_sources)
    box = shape.bounding_box()
    shape_signature = {
        "collision_geometry_revision": COLLISION_GEOMETRY_REVISION,
        "governing_geometry_source_sha256": {
            str(path.relative_to(ROOT)).replace("\\", "/"): _sha256(path)
            for path in geometry_sources
        },
        "label": str(getattr(shape, "label", "part")),
        "solid_count": len(list(shape.solids())),
        "volume_mm3": round(float(shape.volume), 9),
        "bbox_mm": [
            round(float(value), 9) for value in (
                box.min.X, box.min.Y, box.min.Z,
                box.max.X, box.max.Y, box.max.Z,
            )
        ],
    }
    sidecar_match = False
    if sidecar.exists():
        try:
            sidecar_match = _load(sidecar).get("shape_signature") \
                == shape_signature
        except (OSError, ValueError, json.JSONDecodeError):
            sidecar_match = False
    if (
        raw.exists() and processed.exists()
        and (
            sidecar_match
            or (
                not sidecar.exists()
                and min(raw.stat().st_mtime, processed.stat().st_mtime)
                >= newest_source
            )
        )
    ):
        mesh = trimesh.load(processed, force="mesh", process=False)
        if isinstance(mesh, trimesh.Trimesh) and len(mesh.faces) > 0:
            mesh.process(validate=True)
            mesh.update_faces(mesh.unique_faces())
            mesh.remove_unreferenced_vertices()
            sidecar.write_text(json.dumps({
                "shape_signature": shape_signature,
                "raw_sha256": _sha256(raw),
                "processed_sha256": _sha256(processed),
            }, indent=2, allow_nan=False) + "\n", encoding="utf-8")
            return mesh
    export_stl(
        shape, raw, tolerance=0.06, angular_tolerance=0.08,
        ascii_format=False,
    )
    mesh = trimesh.load(raw, force="mesh", process=False)
    if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
        raise RuntimeError(f"empty collision mesh for {name}")
    mesh.process(validate=True)
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()
    mesh.export(processed)
    sidecar.write_text(json.dumps({
        "shape_signature": shape_signature,
        "raw_sha256": _sha256(raw),
        "processed_sha256": _sha256(processed),
    }, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return mesh


def _parts_mesh(parts: Sequence[Any], name: str) -> trimesh.Trimesh:
    """Tessellate top-level solids separately and preserve closed shells.

    Exporting an assembly compound in one STL can create coincident internal
    seams that look non-manifold after vertex merging.  Collision authority is
    instead one validated mesh per physical occurrence, concatenated without
    welding vertices between occurrences.
    """

    meshes = []
    component_rows = []
    for index, shape in enumerate(parts):
        label = str(getattr(shape, "label", "part")) or "part"
        safe = "".join(
            character if character.isalnum() else "_"
            for character in label
        )[:80]
        mesh = _shape_mesh(shape, f"{name}_{index:03d}_{safe}")
        topology = _mesh_topology(mesh)
        if (
            not topology["watertight"]
            or topology["boundary_edges"] != 0
            or topology["nonmanifold_edges"] != 0
        ):
            raise RuntimeError(
                f"collision component {name}/{index}/{label} is not a "
                f"closed authority: {topology}"
            )
        meshes.append(mesh)
        component_rows.append({
            "index": index,
            "label": label,
            **topology,
        })
    if not meshes:
        raise ValueError(f"no collision parts for {name}")
    result = trimesh.util.concatenate(meshes)
    result.metadata["component_topology"] = component_rows
    return result


def _file_mesh(path: Path) -> trimesh.Trimesh:
    mesh = trimesh.load(path, force="mesh", process=False)
    if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
        raise RuntimeError(f"empty collision mesh {path}")
    mesh.process(validate=True)
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()
    return mesh


def _incremental_leadins() -> Compound:
    parts = [
        guide.leadin_for_tooth(tooth, side, axial_sign)
        for axial_sign in (-1, 1)
        for tooth in range(guide.SLOTS)
        for side in (-1, 1)
    ]
    result = Compound(children=parts)
    result.label = "96_incremental_short_open_cap_leadins"
    return result


def _reference(shape: Any) -> Any:
    return guide.to_machine_reference(shape)


def _mesh_topology(mesh: trimesh.Trimesh) -> dict[str, Any]:
    edge_use = np.bincount(mesh.edges_unique_inverse)
    return {
        "vertices": int(len(mesh.vertices)),
        "triangles": int(len(mesh.faces)),
        "watertight": bool(mesh.is_watertight),
        "boundary_edges": int(np.count_nonzero(edge_use == 1)),
        "nonmanifold_edges": int(np.count_nonzero(edge_use > 2)),
    }


@lru_cache(maxsize=1)
def _collision_models() -> dict[str, Any]:
    fixed_parts = guide.carriage_link_reference_parts()
    spindle_parts = release_candidate.cap_module_parts()
    incremental_parts = [
        _reference(guide.leadin_for_tooth(tooth, side, axial_sign))
        for axial_sign in (-1, 1)
        for tooth in range(guide.SLOTS)
        for side in (-1, 1)
    ]
    flyer_parts = list(
        release_candidate.retained_rotating_parts().values()
    )
    coil_local = Cylinder(
        CONSERVATIVE_COIL_RADIUS_MM,
        2.0 * CONSERVATIVE_COIL_AXIAL_HALF_MM,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    coil = _reference(coil_local)
    meshes = {
        "fixed": _parts_mesh(
            fixed_parts, "actual_fixed_active_sector_yoke_tower_hardware"
        ),
        "spindle": _parts_mesh(
            spindle_parts, "actual_production_caps_short_leadins_hardware"
        ),
        "incremental": _parts_mesh(
            incremental_parts, "incremental_short_leadins_only"
        ),
        "flyer": _parts_mesh(
            flyer_parts,
            "final_integrated_L79_stock_D10_P30_PEEK_bell_six_slug_flyer",
        ),
        "coil": _shape_mesh(coil, "conservative_R26_axial21_coil_growth"),
        "final_wound": _file_mesh(FINAL_WOUND_MESH),
    }
    result: dict[str, Any] = {}
    for name, mesh in meshes.items():
        result[f"{name}_mesh"] = mesh
        result[f"{name}_bvh"] = collide.make_bvh(mesh)
        result[f"{name}_topology"] = _mesh_topology(mesh)
        if "component_topology" in mesh.metadata:
            result[f"{name}_components"] = mesh.metadata[
                "component_topology"
            ]
    return result


def _bounds(shape: Any) -> np.ndarray:
    box = shape.bounding_box()
    return np.asarray([
        [float(box.min.X), float(box.min.Y), float(box.min.Z)],
        [float(box.max.X), float(box.max.Y), float(box.max.Z)],
    ])


def _aabb_distance(left: np.ndarray, right: np.ndarray) -> float:
    gap = np.maximum(
        np.maximum(left[0] - right[1], right[0] - left[1]), 0.0
    )
    return float(np.linalg.norm(gap))


def outboard_yoke_packaging_audit(
    events: list[dict[str, Any]], timeline: Timeline,
) -> dict[str, Any]:
    """Prove the final fixed structure packages on carriage and frame."""

    cached = _read_collision_cache("outboard_yoke_packaging")
    if cached is not None:
        return dict(cached["result"])

    active_parts = [
        shape for shape in guide.carriage_link_reference_parts()
        if str(getattr(shape, "label", ""))
        != "spindle_tower_with_active_sector_M4_insert_pilots"
    ]
    preexisting_carriage = [
        shape for shape in release_candidate._main_links()["carriage"]
        if str(getattr(shape, "label", "")) != "spindle_tower"
    ]
    static_groups = release_candidate.main_static_groups()
    drive_parts = release_candidate.successor_drive_parts()
    static_parts = [
        *static_groups["unchanged"],
        *static_groups["shifted_support"],
        *static_groups["shifted_entry"],
        *(shape for key, shape in drive_parts.items()
          if key != "flyer_pulley"
          and not key.startswith("motor_pulley_BNW_hole_path_")),
    ]

    # Same-link exact BREP: only parts whose AABBs come within the packaging
    # review band need an expensive distance/common computation.
    same_link_rows = []
    same_link_overlap_count = 0
    same_link_minimum = math.inf
    same_link_pair_bounds = []
    for active_index, active in enumerate(active_parts):
        active_bounds = _bounds(active)
        for existing_index, existing in enumerate(preexisting_carriage):
            existing_bounds = _bounds(existing)
            lower = _aabb_distance(active_bounds, existing_bounds)
            same_link_pair_bounds.append((
                lower, active_index, existing_index,
            ))
    same_link_pair_bounds.sort()
    for lower, active_index, existing_index in same_link_pair_bounds:
        if lower >= same_link_minimum:
            break
        active = active_parts[active_index]
        existing = preexisting_carriage[existing_index]
        distance = float(active.distance_to(existing))
        common = active & existing if distance <= 1.0e-6 else None
        overlap = 0.0 if common is None else float(common.volume)
        same_link_overlap_count += int(overlap > 1.0e-5)
        same_link_minimum = min(same_link_minimum, distance)
        same_link_rows.append({
            "active_index": active_index,
            "active_label": str(getattr(
                active, "label", f"active_{active_index}"
            )),
            "preexisting_index": existing_index,
            "preexisting_label": str(getattr(
                existing, "label", f"carriage_{existing_index}"
            )),
            "AABB_lower_bound_mm": lower,
            "exact_distance_mm": distance,
            "positive_overlap_mm3": overlap,
            "unintended_overlap": overlap > 1.0e-5,
        })
    same_link_rows.sort(key=lambda row: row["exact_distance_mm"])

    # The active structure moves only with M0 relative to the static frame.
    # Collapse 225,775 raw samples to exact unique relative transforms.
    kin = collide.Kinematics(collide.load_manifest())
    unique_transforms: dict[tuple[float, ...], dict[str, Any]] = {}
    raw_sample_count = 0
    for time_s, m0, m1, m2 in timeline.samples(
        max_dm2=FULL_MOTION_DM2_RAD,
        max_dm0=FULL_MOTION_DM0_MM,
        max_dm1=FULL_MOTION_DM1_RAD,
    ):
        transform = kin.link_tf("carriage", m0, m1, m2)
        key = _transform_key(transform)
        unique_transforms.setdefault(key, {
            "time_s": float(time_s),
            "m0_rad": float(m0),
            "transform": transform,
        })
        raw_sample_count += 1
    transforms = list(unique_transforms.values())
    translations_z = [
        float(row["transform"][1][2]) for row in transforms
    ]
    minimum_translation = min(translations_z)
    maximum_translation = max(translations_z)

    pair_bounds = []
    active_bounds_rows = [_bounds(shape) for shape in active_parts]
    static_bounds_rows = [_bounds(shape) for shape in static_parts]
    for active_index, active_bounds in enumerate(active_bounds_rows):
        swept = active_bounds.copy()
        swept[0, 2] += minimum_translation
        swept[1, 2] += maximum_translation
        for static_index, static_bounds in enumerate(static_bounds_rows):
            pair_bounds.append((
                _aabb_distance(swept, static_bounds),
                active_index, static_index,
            ))
    pair_bounds.sort()

    active_mesh_cache: dict[int, fcl.BVHModel] = {}
    static_mesh_cache: dict[int, fcl.BVHModel] = {}

    def active_bvh(index: int) -> fcl.BVHModel:
        if index not in active_mesh_cache:
            shape = active_parts[index]
            label = str(getattr(shape, "label", f"active_{index}"))
            safe = "".join(
                character if character.isalnum() else "_"
                for character in label
            )[:80]
            mesh = _shape_mesh(
                shape, f"packaging_active_{index:03d}_{safe}"
            )
            active_mesh_cache[index] = collide.make_bvh(mesh)
        return active_mesh_cache[index]

    def static_bvh(index: int) -> fcl.BVHModel:
        if index not in static_mesh_cache:
            shape = static_parts[index]
            label = str(getattr(shape, "label", f"static_{index}"))
            safe = "".join(
                character if character.isalnum() else "_"
                for character in label
            )[:80]
            mesh = _shape_mesh(
                shape, f"packaging_static_{index:03d}_{safe}"
            )
            static_mesh_cache[index] = collide.make_bvh(mesh)
        return static_mesh_cache[index]

    static_identity = kin.link_tf("static", 0.0, 0.0, 0.0)
    minimum_clearance = math.inf
    witness = None
    collision_count = 0
    evaluated_pair_count = 0
    for lower_bound, active_index, static_index in pair_bounds:
        if lower_bound >= minimum_clearance:
            break
        reusable = _ReusableFCLPair(
            active_bvh(active_index), static_bvh(static_index)
        )
        evaluated_pair_count += 1
        pair_collision = False
        for row in transforms:
            hit, value = reusable.query(
                row["transform"], static_identity, distance=True,
            )
            clearance = float(value if value is not None else -1.0)
            if hit:
                collision_count += 1
                pair_collision = True
            if clearance < minimum_clearance:
                minimum_clearance = clearance
                witness = {
                    "active_index": active_index,
                    "active_label": str(getattr(
                        active_parts[active_index], "label",
                        f"active_{active_index}",
                    )),
                    "static_index": static_index,
                    "static_label": str(getattr(
                        static_parts[static_index], "label",
                        f"static_{static_index}",
                    )),
                    "time_s": row["time_s"],
                    "m0_rad": row["m0_rad"],
                    "carriage_translation_mm": np.asarray(
                        row["transform"][1], dtype=float
                    ).tolist(),
                    "clearance_mm": clearance,
                }
            if pair_collision:
                break
        if pair_collision:
            break
    if witness is None:
        raise RuntimeError("static packaging search produced no witness")

    result = {
        "authority": (
            "exact incremental active yoke/guides/M3/M4 hardware versus "
            "every retained carriage occurrence and exact final static/frame "
            "occurrences across every unique M0 transform in the full raw sweep"
        ),
        "same_link_carriage": {
            "active_occurrence_count": len(active_parts),
            "preexisting_occurrence_count": len(preexisting_carriage),
            "exact_near_row_count": len(same_link_rows),
            "unintended_positive_overlap_count": same_link_overlap_count,
            "minimum_exact_distance_mm": same_link_minimum,
            "near_rows": same_link_rows,
            "status": "PASS" if same_link_overlap_count == 0 else "FAIL",
        },
        "static_frame_full_M0": {
            "raw_sample_count": raw_sample_count,
            "unique_M0_transform_count": len(transforms),
            "active_occurrence_count": len(active_parts),
            "static_occurrence_count": len(static_parts),
            "candidate_pair_count": len(pair_bounds),
            "exact_evaluated_pair_count": evaluated_pair_count,
            "collision_count": collision_count,
            "minimum_clearance_mm": minimum_clearance,
            "clearance_target_mm": MINIMUM_CLEARANCE_MM,
            "witness": witness,
            "status": "PASS" if (
                collision_count == 0
                and minimum_clearance >= MINIMUM_CLEARANCE_MM
            ) else "FAIL",
        },
    }
    result["status"] = "PASS" if (
        result["same_link_carriage"]["status"] == "PASS"
        and result["static_frame_full_M0"]["status"] == "PASS"
    ) else "FAIL"
    _write_collision_cache("outboard_yoke_packaging", {"result": result})
    return result


def front_plane_yoke_full_m2_clearance(
    timeline: Timeline,
) -> dict[str, Any]:
    """Sweep the exact yoke against every selected flyer occurrence."""

    cached = _read_collision_cache("front_plane_yoke_full_m2")
    if cached is not None:
        return dict(cached["result"])

    kin = collide.Kinematics(collide.load_manifest())
    deepest = min(
        (
            {
                "time_s": float(time_s),
                "m0_rad": float(m0),
                "translation_z_mm": float(
                    kin.link_tf("carriage", m0, m1, m2)[1][2]
                ),
            }
            for time_s, m0, m1, m2 in timeline.samples(
                max_dm2=FULL_MOTION_DM2_RAD,
                max_dm0=FULL_MOTION_DM0_MM,
                max_dm1=FULL_MOTION_DM1_RAD,
            )
        ),
        key=lambda row: row["translation_z_mm"],
    )
    m0 = float(deepest["m0_rad"])
    yoke_shape = guide.to_machine_reference(guide.carriage_yoke())
    rotating_parts = release_candidate.retained_rotating_parts()
    yoke_bvh = collide.make_bvh(_shape_mesh(
        yoke_shape, "front_plane_full_m2_yoke",
    ))
    flyer_bvh = collide.make_bvh(_parts_mesh(
        list(rotating_parts.values()), "front_plane_full_m2_selected_flyer",
    ))
    pair = _ReusableFCLPair(yoke_bvh, flyer_bvh)
    yoke_tf = kin.link_tf("carriage", m0, 0.0, 0.0)
    sample_count = int(round(360.0 / FULL_M2_YOKE_STEP_DEG))
    collision_count = 0
    minimum = math.inf
    witness_deg = None
    for index in range(sample_count):
        angle_deg = index * FULL_M2_YOKE_STEP_DEG
        flyer_tf = kin.link_tf(
            "flyer", m0, 0.0, math.radians(angle_deg)
        )
        hit, value = pair.query(yoke_tf, flyer_tf, distance=True)
        collision_count += int(hit)
        if not hit and value is not None and float(value) < minimum:
            minimum = float(value)
            witness_deg = angle_deg

    witness = None
    if witness_deg is not None:
        flyer_tf = kin.link_tf(
            "flyer", m0, 0.0, math.radians(witness_deg)
        )
        for index, (name, shape) in enumerate(rotating_parts.items()):
            label = str(getattr(shape, "label", name)) or name
            safe = "".join(
                character if character.isalnum() else "_"
                for character in label
            )[:80]
            part_bvh = collide.make_bvh(_shape_mesh(
                shape,
                f"front_plane_full_m2_flyer_{index:03d}_{safe}",
            ))
            hit, value = _fcl_query(
                yoke_bvh, part_bvh, yoke_tf, flyer_tf, distance=True,
            )
            clearance = -1.0 if hit else float(value)
            if witness is None or clearance < witness["clearance_mm"]:
                witness = {
                    "flyer_occurrence_name": name,
                    "flyer_occurrence_label": label,
                    "m2_deg": witness_deg,
                    "clearance_mm": clearance,
                }

    result = {
        "authority": (
            "exact fused/cut yoke versus every selected final rotating "
            "occurrence over a complete M2 revolution at deepest raw M0"
        ),
        "deepest_raw_M0": deepest,
        "M2_step_deg": FULL_M2_YOKE_STEP_DEG,
        "M2_sample_count": sample_count,
        "rotating_occurrence_count": len(rotating_parts),
        "collision_count": collision_count,
        "minimum_clearance_mm": minimum,
        "clearance_target_mm": MINIMUM_CLEARANCE_MM,
        "witness": witness,
        "status": "PASS" if (
            collision_count == 0 and minimum >= MINIMUM_CLEARANCE_MM
        ) else "FAIL",
    }
    _write_collision_cache("front_plane_yoke_full_m2", {"result": result})
    return result


def _fcl_query(
    left: fcl.BVHModel,
    right: fcl.BVHModel,
    left_tf: tuple[np.ndarray, np.ndarray],
    right_tf: tuple[np.ndarray, np.ndarray],
    *,
    distance: bool,
) -> tuple[bool, float | None]:
    lr, lt = left_tf
    rr, rt = right_tf
    left_object = fcl.CollisionObject(left, fcl.Transform(lr, lt))
    right_object = fcl.CollisionObject(right, fcl.Transform(rr, rt))
    collision_result = fcl.CollisionResult()
    fcl.collide(
        left_object, right_object, fcl.CollisionRequest(), collision_result,
    )
    hit = bool(collision_result.is_collision)
    if hit:
        return True, -1.0
    if not distance:
        return False, None
    value = float(fcl.distance(
        left_object, right_object, fcl.DistanceRequest(), fcl.DistanceResult(),
    ))
    return False, value


class _ReusableFCLPair:
    """Reuse two collision objects so long sweeps do not retain native BVHs."""

    def __init__(self, left: fcl.BVHModel, right: fcl.BVHModel) -> None:
        identity = fcl.Transform(np.eye(3), np.zeros(3))
        self.left = fcl.CollisionObject(left, identity)
        self.right = fcl.CollisionObject(right, identity)

    def query(
        self,
        left_tf: tuple[np.ndarray, np.ndarray],
        right_tf: tuple[np.ndarray, np.ndarray],
        *,
        distance: bool,
    ) -> tuple[bool, float | None]:
        left_r, left_t = left_tf
        right_r, right_t = right_tf
        self.left.setTransform(fcl.Transform(left_r, left_t))
        self.right.setTransform(fcl.Transform(right_r, right_t))
        collision_result = fcl.CollisionResult()
        fcl.collide(
            self.left, self.right, fcl.CollisionRequest(),
            collision_result,
        )
        hit = bool(collision_result.is_collision)
        if hit:
            return True, -1.0
        if not distance:
            return False, None
        value = float(fcl.distance(
            self.left, self.right, fcl.DistanceRequest(),
            fcl.DistanceResult(),
        ))
        return False, value


def _relative_transform(
    left_tf: tuple[np.ndarray, np.ndarray],
    right_tf: tuple[np.ndarray, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    """Express ``right`` in ``left`` coordinates for cacheable queries."""

    left_r, left_t = left_tf
    right_r, right_t = right_tf
    relative_r = left_r.T @ right_r
    relative_t = left_r.T @ (right_t - left_t)
    return relative_r, relative_t


def _transform_key(transform: tuple[np.ndarray, np.ndarray]) -> tuple[float, ...]:
    rotation, translation = transform
    return tuple(np.round(np.concatenate([
        rotation.reshape(-1), translation.reshape(-1),
    ]), 10).tolist())


def deposition_rigid_collision(
    rows: list[dict[str, Any]],
    loci: list[RawLocus],
    models: Mapping[str, Any],
) -> dict[str, Any]:
    """Bind three actual rigid-part pairs to the same 2,400 route loci."""

    cached = _read_collision_cache("deposition_rigid_collision")
    if cached is not None:
        fields = cached["row_fields"]
        if len(fields) != len(rows):
            raise RuntimeError("deposition collision cache row count drift")
        for row, values in zip(rows, fields):
            row.update(values)
        return dict(cached["result"])

    kin = collide.Kinematics(collide.load_manifest())
    pairs = {
        "fixed_to_flyer": ("fixed", "flyer", "carriage", "flyer"),
        "caps_to_flyer": ("spindle", "flyer", "spindle", "flyer"),
        "fixed_to_caps": ("fixed", "spindle", "carriage", "spindle"),
    }
    summaries: dict[str, Any] = {}
    for key in pairs:
        summaries[key] = {
            "collision_count": 0,
            "minimum_clearance_mm": math.inf,
            "witness_index": None,
        }
    caches: dict[str, dict[tuple[float, ...], tuple[bool, float | None]]] = {
        key: {} for key in pairs
    }
    identity = (np.eye(3), np.zeros(3))
    for index, (row, locus) in enumerate(zip(rows, loci)):
        transforms = {
            owner: kin.link_tf(
                owner, locus.m0_rad, locus.m1_rad, locus.m2_rad
            )
            for owner in ("carriage", "spindle", "flyer")
        }
        for key, (left, right, left_owner, right_owner) in pairs.items():
            relative = _relative_transform(
                transforms[left_owner], transforms[right_owner]
            )
            cache_key = _transform_key(relative)
            if cache_key not in caches[key]:
                caches[key][cache_key] = _fcl_query(
                    models[f"{left}_bvh"], models[f"{right}_bvh"],
                    identity, relative, distance=True,
                )
            hit, value = caches[key][cache_key]
            clearance = float(value if value is not None else -1.0)
            row[f"rigid_{key}_collision"] = hit
            row[f"rigid_{key}_clearance_mm"] = round(clearance, 12)
            summary = summaries[key]
            summary["collision_count"] += int(hit)
            if clearance < summary["minimum_clearance_mm"]:
                summary["minimum_clearance_mm"] = clearance
                summary["witness_index"] = index
    for key, summary in summaries.items():
        witness = rows[int(summary.pop("witness_index"))]
        summary["witness"] = {
            field: witness[field]
            for field in (
                "pass_index", "state_index", "tooth_index",
                "m0_rad", "m1_rad", "m2_rad",
            )
        }
        summary["clearance_target_mm"] = MINIMUM_CLEARANCE_MM
        summary["unique_relative_pose_query_count"] = len(caches[key])
        summary["status"] = (
            "PASS" if summary["collision_count"] == 0
            and summary["minimum_clearance_mm"] >= MINIMUM_CLEARANCE_MM
            else "FAIL"
        )
    result = {
        "locus_count": len(rows),
        "pairs": summaries,
        "same_locus_array_as_routes": len(rows) == EXPECTED_LOCI,
        "status": "PASS" if all(
            value["status"] == "PASS" for value in summaries.values()
        ) else "FAIL",
    }
    collision_fields = [
        {
            key: value for key, value in row.items()
            if key.startswith("rigid_")
        }
        for row in rows
    ]
    _write_collision_cache(
        "deposition_rigid_collision",
        {"result": result, "row_fields": collision_fields},
    )
    return result


def arbitrary_m1_and_coil_clearance(models: Mapping[str, Any]) -> dict[str, Any]:
    """Sweep M1 independently of the deposition-only index positions."""

    cached = _read_collision_cache("arbitrary_m1_clearance")
    if cached is not None:
        return dict(cached["result"])

    kin = collide.Kinematics(collide.load_manifest())
    pairs = {
        "fixed_to_caps_all_M1": ("fixed", "spindle", "carriage", "spindle"),
        "fixed_to_conservative_coil_all_M1": (
            "fixed", "coil", "carriage", "spindle"
        ),
        "fixed_to_final_wound_all_M1": (
            "fixed", "final_wound", "carriage", "spindle"
        ),
    }
    rows = {
        key: {"collision_count": 0, "minimum_clearance_mm": math.inf,
              "witness_M1_deg": None}
        for key in pairs
    }
    full_angles = np.arange(
        0.0, 360.0 + ARBITRARY_M1_STEP_DEG / 2.0,
        ARBITRARY_M1_STEP_DEG,
    )
    # The complete 24-tooth cap/lead-in set is exactly 15-degree periodic;
    # the conservative cylindrical coil is axisymmetric.  The final-wound
    # imported mesh is not granted symmetry and still receives all 721 poses.
    angle_sets = {
        "fixed_to_caps_all_M1": np.arange(
            0.0, 15.0, ARBITRARY_M1_STEP_DEG,
        ),
        "fixed_to_conservative_coil_all_M1": np.asarray([0.0]),
        "fixed_to_final_wound_all_M1": full_angles,
    }
    carriage_tf = kin.link_tf("carriage", 0.0, 0.0, 0.0)
    for key, (left, right, left_owner, right_owner) in pairs.items():
        reusable = _ReusableFCLPair(
            models[f"{left}_bvh"], models[f"{right}_bvh"]
        )
        for degrees in angle_sets[key]:
            spindle_tf = kin.link_tf(
                "spindle", 0.0, math.radians(float(degrees)), 0.0
            )
            transforms = {"carriage": carriage_tf, "spindle": spindle_tf}
            hit, value = reusable.query(
                transforms[left_owner], transforms[right_owner],
                distance=True,
            )
            clearance = float(value if value is not None else -1.0)
            row = rows[key]
            row["collision_count"] += int(hit)
            if clearance < row["minimum_clearance_mm"]:
                row["minimum_clearance_mm"] = clearance
                row["witness_M1_deg"] = float(degrees)
    identity = (np.eye(3), np.zeros(3))
    for key, left, right in (
        ("incremental_leadins_to_conservative_coil", "incremental", "coil"),
        ("incremental_leadins_to_final_wound", "incremental", "final_wound"),
    ):
        hit, value = _fcl_query(
            models[f"{left}_bvh"], models[f"{right}_bvh"],
            identity, identity, distance=True,
        )
        rows[key] = {
            "collision_count": int(hit),
            "minimum_clearance_mm": float(value if value is not None else -1.0),
            "witness_M1_deg": "same_spindle_link_relative_pose_invariant",
        }
    for row in rows.values():
        row["clearance_target_mm"] = MINIMUM_CLEARANCE_MM
        row["status"] = (
            "PASS" if row["collision_count"] == 0
            and row["minimum_clearance_mm"] >= MINIMUM_CLEARANCE_MM
            else "FAIL"
        )
    result = {
        "M1_step_deg": ARBITRARY_M1_STEP_DEG,
        "M1_effective_full_circle_sample_count": len(full_angles),
        "actual_query_counts": {
            key: len(values) for key, values in angle_sets.items()
        },
        "periodicity_authority": {
            "caps": "24 identical tooth-indexed cap/lead-in bodies; 30 residues x 24 sectors = full 721-point closure including duplicate 360deg",
            "conservative_coil": "axisymmetric R26 x axial21 cylinder",
            "final_wound": "no symmetry assumed; all 721 M1 angles queried",
        },
        "progressive_body": (
            "conservative nested completed-coil bound: R26 x axial 21 mm"
        ),
        "final_body": str(FINAL_WOUND_MESH.relative_to(ROOT)).replace("\\", "/"),
        "pairs": rows,
        "status": "PASS" if all(
            value["status"] == "PASS" for value in rows.values()
        ) else "FAIL",
    }
    _write_collision_cache("arbitrary_m1_clearance", {"result": result})
    return result


def full_raw_rigid_motion(
    events: list[dict[str, Any]],
    timeline: Timeline,
    models: Mapping[str, Any],
) -> dict[str, Any]:
    """Sweep the exact final rigid candidate through every raw motion class."""

    cached = _read_collision_cache("full_raw_rigid_motion")
    if cached is not None:
        return dict(cached["result"])

    kin = collide.Kinematics(collide.load_manifest())
    event_rows = [
        (float(event["t"]), str(event["e"]))
        for event in events if event["e"] not in ("cmd", "meta")
    ]
    event_times = [row[0] for row in event_rows]
    pair_specs = {
        "fixed_guide_yoke_tower_to_final_flyer": (
            "fixed", "flyer", "carriage", "flyer"
        ),
        "production_caps_leadins_to_final_flyer": (
            "spindle", "flyer", "spindle", "flyer"
        ),
    }
    pairs = {
        key: {"collision_count": 0, "first_collision": None}
        for key in pair_specs
    }
    caches: dict[str, dict[tuple[float, ...], bool]] = {
        key: {} for key in pair_specs
    }
    reusable_pairs = {
        key: _ReusableFCLPair(
            models[f"{left}_bvh"], models[f"{right}_bvh"]
        )
        for key, (left, right, _left_owner, _right_owner)
        in pair_specs.items()
    }
    identity = (np.eye(3), np.zeros(3))
    phase_counts: dict[str, int] = {}
    pose_hasher = hashlib.sha256()
    count = 0
    for time_s, m0, m1, m2 in timeline.samples(
        max_dm2=FULL_MOTION_DM2_RAD,
        max_dm0=FULL_MOTION_DM0_MM,
        max_dm1=FULL_MOTION_DM1_RAD,
    ):
        phase_index = bisect_right(event_times, float(time_s)) - 1
        phase = event_rows[phase_index][1] if phase_index >= 0 else "init"
        phase_counts[phase] = phase_counts.get(phase, 0) + 1
        serialized = [
            round(float(time_s), 9), round(float(m0), 12),
            round(float(m1), 12), round(float(m2), 12), phase,
        ]
        pose_hasher.update(json.dumps(
            serialized, separators=(",", ":"), allow_nan=False,
        ).encode("utf-8"))
        transforms = {
            owner: kin.link_tf(owner, m0, m1, m2)
            for owner in ("carriage", "spindle", "flyer")
        }
        for key, (left, right, left_owner, right_owner) in pair_specs.items():
            relative = _relative_transform(
                transforms[left_owner], transforms[right_owner]
            )
            cache_key = _transform_key(relative)
            if cache_key not in caches[key]:
                hit, _ = reusable_pairs[key].query(
                    identity, relative, distance=False,
                )
                caches[key][cache_key] = hit
            hit = caches[key][cache_key]
            if hit:
                pairs[key]["collision_count"] += 1
                if pairs[key]["first_collision"] is None:
                    pairs[key]["first_collision"] = {
                        "sample_index": count,
                        "time_s": float(time_s),
                        "phase": phase,
                        "m0_rad": float(m0),
                        "m1_rad": float(m1),
                        "m2_rad": float(m2),
                    }
        count += 1
    required = {
        "park_or_prevent_collision": any(
            "prevent_collision" in name for name in phase_counts
        ),
        "tooth_index": any(
            "move_to_teeth" in name or "motor1" in name
            for name in phase_counts
        ),
        "shaft_wrap": "wind_wire_around_shaft" in phase_counts,
        "load_or_position": any(
            "position" in name or "set_motor2_wire_position" in name
            for name in phase_counts
        ),
        "winding": any(
            name in phase_counts for name in (
                "set_motor2_wire_position_done", "wind_wire_done",
            )
        ),
    }
    for row in pairs.values():
        row["status"] = (
            "PASS" if row["collision_count"] == 0 else "FAIL"
        )
    for key, row in pairs.items():
        row["unique_relative_pose_query_count"] = len(caches[key])
    result = {
        "authority": "EXACT_FINAL_RIGID_PARTS_NOT_FLEXIBLE_CONDUCTOR",
        "sampling": {
            "maximum_M2_step_deg": math.degrees(FULL_MOTION_DM2_RAD),
            "maximum_M1_step_deg": math.degrees(FULL_MOTION_DM1_RAD),
            "maximum_M0_step_mm": FULL_MOTION_DM0_MM,
        },
        "sample_count": count,
        "pose_stream_sha256": pose_hasher.hexdigest(),
        "covered_phase_sample_counts": phase_counts,
        "required_motion_classes_present": required,
        "pairs": pairs,
        "status": "PASS" if (
            all(row["status"] == "PASS" for row in pairs.values())
            and all(required.values())
        ) else "FAIL",
    }
    _write_collision_cache("full_raw_rigid_motion", {"result": result})
    return result


def coupled_live_line_loads(
    rows: list[dict[str, Any]], loci: list[RawLocus],
) -> dict[str, Any]:
    """Bind the physical bell-exit/free-span force line to M2, M1 and M0."""

    m2_worst: tuple[float, int] | None = None
    m0_worst: tuple[float, int] | None = None
    for index, (row, locus) in enumerate(zip(rows, loci)):
        rotation = wirepath.rot_z(locus.m2_rad)
        bell_exit = rotation @ np.asarray(
            row["bell_exit_local_mm"], dtype=float
        )
        capture = carriage_world(row["capture_local_mm"], locus)
        direction = _unit(capture - bell_exit)
        m2_arm = abs(float(np.cross(bell_exit, direction)[2]))
        m0_component = abs(float(direction[2]))
        row["live_line_bell_exit_world_mm"] = np.round(
            bell_exit, 12
        ).tolist()
        row["live_line_capture_world_mm"] = np.round(
            capture, 12
        ).tolist()
        row["live_line_unit_bell_to_capture"] = np.round(
            direction, 15
        ).tolist()
        row["M2_perpendicular_live_line_lever_mm"] = round(m2_arm, 12)
        row["M0_force_component_N_at_10N"] = round(
            TENSION_N * m0_component, 12
        )
        if m2_worst is None or m2_arm > m2_worst[0]:
            m2_worst = (m2_arm, index)
        if m0_worst is None or m0_component > m0_worst[0]:
            m0_worst = (m0_component, index)
    if m2_worst is None or m0_worst is None:
        raise RuntimeError("no live-line load loci")

    mass_rows, mass = release_candidate.rotating_mass_rows()
    rotating_j = float(mass["izz_about_M2_axis_kg_m2"])
    selection = _load(M2_SELECTION_REPORT)
    prior_drive = selection["OD65_10N_full_inertia_torque"]
    drive_components = prior_drive["components_kgm2"]
    pulley_authority = selection["motor_pulley_geometry_and_inertia"]
    added_output_j = {
        "official_NBK_P30_stock_complete_assembly": float(
            pulley_authority["published_stock_inertia_kgm2"]
        ),
        "unreleased_BNW_two_M3x12_screw_upper_bound": float(
            pulley_authority["BNW_inertia_bound"]
            ["two_bound_screws_inertia_kgm2"]
        ),
        "210_3GT_6_belt": float(drive_components["210_3GT_6_belt"]),
        "Leadshine_rotor": float(drive_components["Leadshine_rotor"]),
        "two_complete_6001_bearings_upper_bound": float(
            drive_components["two_complete_6001_bearings_upper_bound"]
        ),
    }
    m2_wire = TENSION_N * m2_worst[0] / 1000.0
    full_j = rotating_j + sum(added_output_j.values())
    acceleration = float(prior_drive["angular_acceleration_rad_s2"])
    friction = float(prior_drive["friction_allowance_nm_unmeasured"])
    required = m2_wire + full_j * acceleration + friction
    available_36 = float(prior_drive["available_36V_lower_edge_nm"])
    available_24 = float(selection["motor"]["24V_lower_edge_300rpm_nm"])
    transmission = float(
        selection["transmission"]["allowable_transmission_torque_nm"]
    )

    loads_report = _load(LOADS_REPORT)
    m1_baseline_required = float(loads_report["m1"]["t_required_nm"])
    m1_wrap = 0.040
    m1_friction = 0.020
    m1_acceleration = 50.0
    m1_baseline_j = (
        m1_baseline_required - m1_wrap - m1_friction
    ) / m1_acceleration
    leadins = _reference(_incremental_leadins())
    leadin_volume = float(leadins.volume)
    leadin_center = leadins.center()
    leadin_matrix = leadins.matrix_of_inertia
    leadin_volume_j = float(leadin_matrix[1][1]) + leadin_volume * (
        float(leadin_center.X) ** 2 + float(leadin_center.Z) ** 2
    )
    cap_j = leadin_volume_j * PEEK_DENSITY_G_MM3 * 1.0e-9
    m1_reaction = TENSION_N * abs(guide.PORT_TANGENTIAL_MM) / 1000.0
    m1_required = (
        max(m1_wrap, m1_reaction) + m1_friction
        + (m1_baseline_j + cap_j) * m1_acceleration
    )
    m1_available = 0.33135211024345884

    m0_base_force = float(loads_report["m0"]["axial_force_n"])
    m0_base_required = float(loads_report["m0"]["t_required_nm"])
    m0_available = m0_base_required * float(loads_report["m0"]["margin"])
    m0_added_mass_g = (
        float(guide.carriage_yoke().volume) * ALUMINUM_DENSITY_G_MM3
        + sum(
            float(guide.active_sector_guide(sign).volume)
            for sign in (-1, 1)
        ) * PEEK_DENSITY_G_MM3
        + sum(
            float(shape.volume) for shape in guide.guide_retention_hardware()
        ) * STEEL_DENSITY_G_MM3
        + sum(
            float(shape.volume)
            for shape in guide.tower_adapter_hardware_reference()
        ) * STEEL_DENSITY_G_MM3
    )
    m0_added_gravity = m0_added_mass_g / 1000.0 * 9.80665
    m0_terminal_component = TENSION_N * m0_worst[0]
    m0_final_force = (
        m0_base_force + m0_terminal_component + m0_added_gravity
    )
    m0_required = m0_base_required * m0_final_force / m0_base_force

    def witness(pair: tuple[float, int]) -> dict[str, Any]:
        value, index = pair
        row = rows[index]
        return {
            "pass_index": row["pass_index"],
            "state_index": row["state_index"],
            "tooth_index": row["tooth_index"],
            "m0_rad": row["m0_rad"],
            "m1_rad": row["m1_rad"],
            "m2_rad": row["m2_rad"],
            "bell_exit_world_mm": row["live_line_bell_exit_world_mm"],
            "capture_world_mm": row["live_line_capture_world_mm"],
            "unit_bell_to_capture": row["live_line_unit_bell_to_capture"],
            "value": value,
        }

    return {
        "force_line_definition": (
            "10 N along the exact physical bell tangent from the exposed "
            "bell exit to the M0-following capture; torque uses the "
            "perpendicular projected lever, not a radial guess"
        ),
        "M2": {
            "maximum_perpendicular_live_line_lever_mm": m2_worst[0],
            "wire_torque_at_10N_nm": m2_wire,
            "integrated_rotating_occurrence_count": len(mass_rows),
            "integrated_rotating_izz_kg_m2": rotating_j,
            "full_output_inertia_kg_m2": full_j,
            "added_output_referred_components_kg_m2": added_output_j,
            "pre_terminal_drive_report": str(
                M2_SELECTION_REPORT.relative_to(ROOT)
            ).replace("\\", "/"),
            "pre_terminal_drive_report_sha256": _sha256(
                M2_SELECTION_REPORT
            ),
            "angular_acceleration_rad_s2": acceleration,
            "acceleration_torque_nm": full_j * acceleration,
            "friction_allowance_nm": friction,
            "required_output_torque_nm": required,
            "required_2x_running_torque_nm": 2.0 * required,
            "Leadshine_36V_lower_edge_nm": available_36,
            "Leadshine_36V_available_to_required_multiple": (
                available_36 / required
            ),
            "Leadshine_36V_gate_ge_2x": available_36 / required >= 2.0,
            "theoretical_curve_gate_scope": (
                "published 36 V lower-edge curve only"
            ),
            "driver_36V_current_microstep_limits_configured_and_verified": False,
            "installed_hot_dyno_verified": False,
            "Leadshine_24V_lower_edge_nm": available_24,
            "Leadshine_24V_available_to_required_multiple": (
                available_24 / required
            ),
            "Leadshine_24V_numeric_gate_ge_2x": (
                available_24 / required >= 2.0
            ),
            "Leadshine_24V_gate_ge_2x": (
                available_24 / required >= 2.0
            ),
            "Leadshine_24V_release_authorized": False,
            "Leadshine_24V_release_reason": (
                "exact-load arithmetic passes 2x, but normal release remains "
                "the selected 36 V curve condition and no configured-driver "
                "or hot-dyno evidence reauthorizes 24 V"
            ),
            "P30_210_3GT_allowable_transmission_torque_nm": transmission,
            "P30_210_3GT_available_to_required_multiple": (
                transmission / required
            ),
            "P30_210_3GT_gate_ge_2x": transmission / required >= 2.0,
            "witness": witness(m2_worst),
        },
        "M1": {
            "short_handoff_tangential_offset_mm": abs(
                guide.PORT_TANGENTIAL_MM
            ),
            "terminal_reaction_torque_at_10N_nm": m1_reaction,
            "shaft_wrap_torque_nm": m1_wrap,
            "governing_wire_torque_nm": max(m1_wrap, m1_reaction),
            "added_short_leadins_Iyy_kg_m2": cap_j,
            "final_spindle_inertia_kg_m2": m1_baseline_j + cap_j,
            "required_output_torque_nm": m1_required,
            "available_torque_nm": m1_available,
            "available_to_required_multiple": m1_available / m1_required,
            "gate_ge_2x": m1_available / m1_required >= 2.0,
            "commanded_hold_contract": (
                "closed-loop M1 remains enabled and commanded at the indexed "
                "angle throughout winding; no passive detent is required"
            ),
            "drive_fault_safe_behavior_verified": False,
        },
        "M0": {
            "wire_force_resultant_N": TENSION_N,
            "maximum_axis_force_component_N": TENSION_N * m0_worst[0],
            "baseline_axial_force_N": m0_base_force,
            "baseline_required_motor_torque_Nm": m0_base_required,
            "added_fixed_guide_yoke_hardware_mass_g": m0_added_mass_g,
            "added_fixed_guide_yoke_gravity_N": m0_added_gravity,
            "final_conservative_axial_force_N": m0_final_force,
            "final_required_motor_torque_Nm": m0_required,
            "available_motor_torque_Nm": m0_available,
            "available_to_required_multiple": m0_available / m0_required,
            "gate_ge_2x": m0_available / m0_required >= 2.0,
            "witness": witness(m0_worst),
        },
    }


def guide_structure_dfm_and_attachments() -> dict[str, Any]:
    """Analytical load screen plus exact attachment-chain contacts."""

    local_guides = [guide.active_sector_guide(sign) for sign in (-1, 1)]
    local_yoke = guide.carriage_yoke()
    machine_yoke = guide.to_machine_reference(local_yoke)
    tower = guide.revised_spindle_tower()
    m3 = guide.guide_retention_hardware()
    m4 = guide.tower_adapter_hardware_reference()
    guide_to_yoke = []
    for shape in local_guides:
        common = shape & local_yoke
        guide_to_yoke.append({
            "distance_mm": float(shape.distance_to(local_yoke)),
            "positive_overlap_mm3": (
                0.0 if common is None else float(common.volume)
            ),
        })
    yoke_tower_common = machine_yoke & tower
    yoke_tower_overlap = (
        0.0 if yoke_tower_common is None
        else float(yoke_tower_common.volume)
    )
    yoke_tower_distance = float(machine_yoke.distance_to(tower))

    fixed_mass_g = (
        float(local_yoke.volume) * ALUMINUM_DENSITY_G_MM3
        + sum(float(shape.volume) for shape in local_guides)
        * PEEK_DENSITY_G_MM3
        + sum(float(shape.volume) for shape in m3) * STEEL_DENSITY_G_MM3
        + sum(float(shape.volume) for shape in m4) * STEEL_DENSITY_G_MM3
    )
    gravity_n = fixed_mass_g / 1000.0 * 9.80665
    span = guide.YOKE_AXIAL_SPAN_MM
    radial = guide.YOKE_BAR_RADIAL_LENGTH_MM
    tangent = guide.YOKE_BAR_WIDTH_MM
    i_radial = tangent * radial**3 / 12.0
    i_tangent = radial * tangent**3 / 12.0
    minimum_i = min(i_radial, i_tangent)
    c = tangent / 2.0 if minimum_i == i_tangent else radial / 2.0
    load_n = TENSION_N + gravity_n
    moment_nmm = load_n * span
    stress_mpa = moment_nmm * c / minimum_i
    deflection_mm = load_n * span**3 / (
        3.0 * 69000.0 * minimum_i
    )

    profile_tolerance = PEEK_PROFILE_TOLERANCE_MM
    finished_wall = (
        guide.LEADIN_OUTER_RADIUS_MM
        - guide.LEADIN_CLEAR_RADIUS_MM - profile_tolerance
    )
    lip_width = 2.0
    combined_section_modulus = (
        2.0 * lip_width * finished_wall**2 / 6.0
    )
    lip_lever = guide.LEADIN_OUTER_RADIUS_MM
    c_stress = TENSION_N * lip_lever / combined_section_modulus
    return {
        "mass_and_yoke_load": {
            "fixed_guide_yoke_hardware_mass_g": fixed_mass_g,
            "gravity_load_N": gravity_n,
            "wire_plus_gravity_screen_load_N": load_n,
            "minimum_full_section_mm": [radial, tangent],
            "cantilever_span_mm": span,
            "minimum_second_moment_mm4": minimum_i,
            "maximum_bending_stress_MPa": stress_mpa,
            "maximum_tip_deflection_mm": deflection_mm,
            "6061_T6_screen_allowable_MPa": 55.0,
            "stress_gate": stress_mpa <= 55.0,
            "M3_per_screw_static_load_N": TENSION_N / 2.0,
            "M3_per_screw_3x_proof_load_N": 3.0 * TENSION_N / 2.0,
            "M4_per_screw_static_load_N": (
                (2.0 * TENSION_N + gravity_n) / 4.0
            ),
            "M4_per_screw_3x_proof_load_N": (
                3.0 * (2.0 * TENSION_N + gravity_n) / 4.0
            ),
        },
        "short_leadin_C_section": {
            "nominal_wall_mm": (
                guide.LEADIN_OUTER_RADIUS_MM
                - guide.LEADIN_CLEAR_RADIUS_MM
            ),
            "continuous_opening_mm": guide.LEADIN_OPENING_WIDTH_MM,
            "profile_tolerance_mm": profile_tolerance,
            "minimum_finished_wall_mm": finished_wall,
            "two_lip_effective_width_each_mm": lip_width,
            "10N_screen_stress_MPa": c_stress,
            "PEEK_screen_allowable_MPa": 60.0,
            "analytical_gate": c_stress <= 60.0,
            "physical_forming_gauge_and_abrasion_coupon_complete": False,
        },
        "attachments": {
            "guide_to_aluminum_yoke_face_contacts": guide_to_yoke,
            "yoke_to_revised_tower_distance_mm": yoke_tower_distance,
            "yoke_to_revised_tower_positive_overlap_mm3": yoke_tower_overlap,
            "M3_stack_count": len(m3) // 3,
            "M3_hardware": "4x M3x14 + washer + short M3 insert",
            "M3_insert_engagement_mm": guide.M3_SHORT_INSERT_LENGTH_MM,
            "M3_location_control": "two positive guide datum keys; screw clearance does not locate",
            "M4_stack_count": len(m4) // 3,
            "M4_hardware": "4x M4x10 + washer + short M4 insert",
            "M4_insert_pilot_depth_mm": guide.TOWER_M4_INSERT_DEPTH_MM,
            "M4_location_control": "two positive tower datum keys; screw clearance does not locate",
            "no_floating_screw_chain_gate": (
                len(m3) == 12 and len(m4) == 12
                and all(row["distance_mm"] <= 1.0e-8 for row in guide_to_yoke)
                and all(
                    row["positive_overlap_mm3"] <= 1.0e-8
                    for row in guide_to_yoke
                )
                and yoke_tower_distance <= 1.0e-8
                and yoke_tower_overlap <= 1.0e-8
            ),
        },
    }


def tolerance_budget(
    deposition: Mapping[str, Any], arbitrary: Mapping[str, Any],
    structure: Mapping[str, Any], full_m2_yoke: Mapping[str, Any],
) -> dict[str, Any]:
    components = {
        "PEEK_profile_mm": 0.10,
        "aluminum_yoke_machining_mm": 0.08,
        "positive_datum_key_fit_mm": 0.05,
        "M3_M4_clamp_stack_residual_after_keys_mm": 0.05,
        "thermal_differential_mm": 0.03,
        "10N_yoke_deflection_mm": float(
            structure["mass_and_yoke_load"]["maximum_tip_deflection_mm"]
        ),
        "FCL_tessellation_reserve_mm": 0.06,
    }
    total = sum(components.values())
    controls = {
        "analytic_M1_handoff_gap": guide.ARBITRARY_M1_RADIAL_CLEARANCE_MM,
        "deposition_fixed_to_final_flyer": float(
            deposition["pairs"]["fixed_to_flyer"]["minimum_clearance_mm"]
        ),
        "deposition_fixed_to_caps": float(
            deposition["pairs"]["fixed_to_caps"]["minimum_clearance_mm"]
        ),
        "full_M2_yoke_to_final_flyer": float(
            full_m2_yoke["minimum_clearance_mm"]
        ),
        "arbitrary_M1_fixed_to_caps": float(
            arbitrary["pairs"]["fixed_to_caps_all_M1"]
            ["minimum_clearance_mm"]
        ),
        "arbitrary_M1_fixed_to_conservative_coil": float(
            arbitrary["pairs"]["fixed_to_conservative_coil_all_M1"]
            ["minimum_clearance_mm"]
        ),
    }
    rows = {
        name: {
            "nominal_clearance_mm": value,
            "summed_one_sided_adverse_budget_mm": total,
            "worst_case_clearance_mm": value - total,
            "gate_ge_2mm": value - total >= MINIMUM_CLEARANCE_MM,
        }
        for name, value in controls.items()
    }
    return {
        "method": "linear sum of independent one-sided adverse contributors",
        "components": components,
        "summed_one_sided_adverse_budget_mm": total,
        "controls": rows,
        "status": "PASS" if all(
            row["gate_ge_2mm"] for row in rows.values()
        ) else "FAIL",
    }


def _polyline_capsule_mesh(
    points: np.ndarray, radius_mm: float,
) -> trimesh.Trimesh:
    pieces: list[trimesh.Trimesh] = []
    for point in points:
        sphere = trimesh.creation.icosphere(subdivisions=2, radius=radius_mm)
        sphere.apply_translation(point)
        pieces.append(sphere)
    for start, end in zip(points, points[1:]):
        delta = end - start
        length = float(np.linalg.norm(delta))
        if length <= 1.0e-9:
            continue
        transform = trimesh.geometry.align_vectors(
            np.array([0.0, 0.0, 1.0]), delta / length
        )
        transform[:3, 3] = (start + end) / 2.0
        pieces.append(trimesh.creation.cylinder(
            radius=radius_mm, height=length, sections=16,
            transform=transform,
        ))
    return trimesh.util.concatenate(pieces)


def shaft_wrap_guide_bypass(
    events: list[dict[str, Any]], timeline: Timeline,
    models: Mapping[str, Any],
) -> dict[str, Any]:
    """Prove the physical bell-to-shaft wire bypasses the fixed guide/yoke."""

    wraps = _raw_shaft_wraps(events, timeline)
    contact = wire_geometry.shaft_contact_spec(DEFAULT_STATOR)
    kin = collide.Kinematics(collide.load_manifest())
    cases = []
    for wrap in wraps:
        start = float(wrap["start"])
        m0, m1, m2 = map(float, timeline.pose_at(start))
        rotation = wirepath.rot_z(m2)
        bell_throat = rotation @ np.array([
            0.0, flyer.BELL_THROAT_Y_MM,
            float(flyer.base.TIP_GUIDE_CENTER_Z_MM),
        ])
        side = -1 if float(wrap["delta_m1_rad"]) > 0.0 else 1
        target = wirepath.shaft_tangent_point(
            bell_throat, float(PARAMS.stator_axis_z(m0)), contact, side,
        )
        locus = RawLocus(
            pass_index=-1, phase_index=-1, tooth_index=0,
            motion_sign=side, clockwise_argument=side > 0,
            state_index=0, turn_index=0, half_turn_index=0,
            time_s=start, m0_rad=m0, m1_rad=m1, m2_rad=m2,
            m2_mod_rad=float(m2 % (2.0 * math.pi)),
            radial_x_mm=float(PARAMS.stator_axis_z(m0)),
            m1_alignment_error_rad=0.0,
        )
        path, meta = bell_fairlead_path(
            target, locus, MAX_WIRE_RADIUS_MM,
        )
        wire_mesh = _polyline_capsule_mesh(path, MAX_WIRE_RADIUS_MM)
        wire_bvh = collide.make_bvh(wire_mesh)
        fixed_tf = kin.link_tf("carriage", m0, m1, m2)
        hit, distance = _fcl_query(
            models["fixed_bvh"], wire_bvh,
            fixed_tf, (np.eye(3), np.zeros(3)), distance=True,
        )
        cases.append({
            "wrap_number": int(wrap["number"]),
            "raw_start_time_s": start,
            "raw_end_time_s": float(wrap["end"]),
            "raw_delta_m1_rad": float(wrap["delta_m1_rad"]),
            "raw_turns": float(wrap["turns"]),
            "raw_M1_pose_count_at_le_0p5deg": int(math.ceil(
                abs(float(wrap["delta_m1_rad"])) / math.radians(0.5)
            )) + 1,
            "physical_bell_wire_center_radius_mm": float(
                meta["wire_center_bend_radius_mm"]
            ),
            "straight_bore_handoff_min_radius_mm": float(
                meta["straight_bore_handoff_min_radius_mm"]
            ),
            "shaft_sleeve_wire_center_radius_mm": float(
                contact["radius_to_wire_center_mm"]
            ),
            "wire_to_fixed_guide_yoke_collision": hit,
            "wire_to_fixed_guide_yoke_clearance_mm": float(
                distance if distance is not None else -1.0
            ),
            "path_sha256": hashlib.sha256(
                np.round(path, decimals=9).tobytes()
            ).hexdigest(),
        })
    guide_bypass = all(
        not row["wire_to_fixed_guide_yoke_collision"]
        and row["wire_to_fixed_guide_yoke_clearance_mm"] >= 0.0
        for row in cases
    )
    turns_exact = all(
        math.isclose(row["raw_turns"], 2.0, abs_tol=1.0e-9)
        for row in cases
    )
    return {
        "authority": (
            "physical PEEK bell meridian plus finite shaft tangent; fixed "
            "guide/yoke collision is independent of M1 rotation during each "
            "axisymmetric sleeve wrap"
        ),
        "cases": cases,
        "gates": {
            "two_raw_wraps_present": len(cases) == 2,
            "physical_bell_and_shaft_contact_radii_ge_3mm": all(
                min(
                    row["physical_bell_wire_center_radius_mm"],
                    row["straight_bore_handoff_min_radius_mm"],
                    row["shaft_sleeve_wire_center_radius_mm"],
                ) >= 3.0 for row in cases
            ),
            "wire_bypasses_fixed_guide_yoke_for_both_wraps": guide_bypass,
            "each_raw_wrap_is_two_full_turns": turns_exact,
            "park_index_load_unload_continuous_conductor_proven": False,
        },
        "status": "FAIL",
    }


def player_locus_payload(
    loci: list[RawLocus], rows: list[dict[str, Any]],
) -> dict[str, Any]:
    """Serialize only the analytically proved wire route for the player."""

    segment_contract = {
        "flyer_geometric_bore": {
            "surface_owner": "flyer",
            "local_frame": "flyer_reference_M2_axis_plus_Z",
            "authority": "exact geometric PEEK bore centerline through last R3.25 elbow",
        },
        "flyer_tensioned_bore_handoff": {
            "surface_owner": "flyer",
            "local_frame": "normalized_active_stator_local_at_locus",
            "authority": "bounded cubic tensioned path inside straight ID0.60 bore",
        },
        "flyer_bell_meridian_arc": {
            "surface_owner": "flyer",
            "local_frame": "normalized_active_stator_local_at_locus",
            "authority": "selected physical polished PEEK bell meridian",
        },
        "dynamic_free_span_to_capture": {
            "surface_owner": "none_flexible_free_span",
            "local_frame": "normalized_active_stator_local_at_locus",
            "authority": "analytic tangent plus R3.25 capture trim",
        },
        "carriage_capture_fillet": {
            "surface_owner": "carriage_M0_following_M1_static",
            "local_frame": "active_sector_carriage_local",
            "authority": "physical fixed capture bowl contact",
        },
        "carriage_fixed_free_gap": {
            "surface_owner": "none_between_fixed_bowl_contacts",
            "local_frame": "active_sector_carriage_local",
            "authority": "analytic straight free segment",
        },
        "carriage_selection_bowl": {
            "surface_owner": "carriage_M0_following_M1_static",
            "local_frame": "active_sector_carriage_local",
            "authority": "physical fixed selection bowl contact",
        },
        "dynamic_handoff_gap": {
            "surface_owner": "none_carriage_to_spindle_gap",
            "local_frame": "normalized_active_stator_local_at_locus",
            "authority": "analytic 2.50 mm arbitrary-M1 rigid handoff gap",
        },
        "spindle_short_leadin": {
            "surface_owner": "spindle_M1",
            "local_frame": "active_tooth_cap_local",
            "authority": (
                "physical short open cap lead-in: left R3.50 to named "
                "riser_top; right R3.50 quarter plus R7.03144 symmetric "
                "S-bend to named waypoint"
            ),
        },
    }
    locus_payload = []
    for locus, row in zip(loci, rows):
        template = route_for_locus(locus)
        segments = []
        for name, points_value in template["segments_local"].items():
            points = np.asarray(points_value, dtype=float)
            if name == "flyer_geometric_bore":
                # Shared exactly in ``flyer_reference``.  The player expands
                # it by the locus M2 angle; duplicating 175 points 2,400
                # times would add no information and materially hurts load.
                continue
            elif name.startswith("carriage_"):
                world = np.asarray([
                    carriage_world(point, locus) for point in points
                ])
            else:
                world = np.asarray([
                    integrated._stator_local_to_world(point, locus)
                    for point in points
                ])
            segments.append({
                "name": name,
                "machine_world_samples_mm": np.round(world, 9).tolist(),
            })
        side_name = "left" if template["side"] < 0 else "right"
        axial_name = "rear" if template["sign"] < 0 else "front"
        locus_payload.append({
            "locus_index": len(locus_payload),
            "time_s": row["time_s"],
            "pass_index": row["pass_index"],
            "phase_index": row["phase_index"],
            "state_index": row["state_index"],
            "turn_index": row["turn_index"],
            "half_turn_index": row["half_turn_index"],
            "tooth_index": row["tooth_index"],
            "motion_sign": row["motion_sign"],
            "axes": {
                "M0_raw_rad": row["m0_rad"],
                "M1_spindle_rad": row["m1_rad"],
                "M2_flyer_rad": row["m2_rad"],
            },
            "path_sha256": row["path_sha256"],
            "route_template_key": [
                value if not isinstance(value, np.generic) else value.item()
                for value in _route_key(locus)
            ],
            "segments": segments,
            "terminal_binding": {
                "lane_id": (
                    f"tooth_{row['tooth_index']:02d}_{side_name}_{axial_name}"
                ),
                "cap_endpoint_name": row["cap_endpoint_name"],
                "cap_endpoint_local_mm": row["port_local_mm"],
                "short_leadin_centerline_radius_mm": (
                    guide.LEADIN_CENTERLINE_RADIUS_MM
                ),
                "right_S_bend_centerline_radius_mm": (
                    guide.RIGHT_S_BEND_RADIUS_MM
                    if template["side"] > 0 else None
                ),
                "source": "cad/permanent_cap_production_review.py",
                "progressive_downstream_authority": (
                    "out/reports/permanent_cap_aggregate_authorization.json"
                ),
                "exact_strand_settling_and_neatness_authorized": False,
            },
        })
    full_bore_reference = np.asarray(
        flyer.guide_bore_centerline_samples(0.50), dtype=float
    )
    conductor_prefix = _flyer_bore_to_tensioned_handoff_local()
    payload: dict[str, Any] = {
        "schema": "carriage-active-sector-terminal-guide-loci/v1",
        "run": {
            "capture": str(CAPTURE.relative_to(ROOT)).replace("\\", "/"),
            "capture_sha256": _sha256(CAPTURE),
            "goal_contract": "GOAL.md normal goal",
            "tags": [
                "canonical_raw_capture", "2400_deposition_loci",
                "active_sector_terminal_guide", "exact_physical_bell",
            ],
            "locus_count": len(locus_payload),
        },
        "axes_mapping": {
            "M0": "raw M0 command; carriage translation through PARAMS.stator_axis_z",
            "M1": "spindle/stator rotation about machine +Y",
            "M2": "flyer rotation about machine +Z",
        },
        "segment_contract": segment_contract,
        "flyer_reference": {
            "frame": "flyer_reference_M2_axis_plus_Z",
            "full_geometric_bore_local_samples_mm": np.round(
                full_bore_reference, 9
            ).tolist(),
            "full_geometric_bore_point_count": len(full_bore_reference),
            "conductor_prefix_point_count": len(conductor_prefix),
            "geometric_bore_to_tensioned_handoff_local_samples_mm": (
                np.round(conductor_prefix, 9).tolist()
            ),
            "guide_only_suffix_reason": (
                "points 175..180 describe the straight geometric bore from "
                "Y64.5..67; the tensioned conductor instead follows the "
                "proved cubic offset beginning at Y64, so those six points "
                "must not be drawn as a second conductor segment"
            ),
            "world_expansion": (
                "Rz(locus.axes.M2_flyer_rad) * local; no translation"
            ),
            "source_api": (
                "retained_flyer_peek_guide_successor."
                "guide_bore_centerline_samples(0.50)"
            ),
        },
        "loci": locus_payload,
    }
    validation_rows = []
    for locus_index in (0, len(loci) // 2, len(loci) - 1):
        locus = loci[locus_index]
        rotation = wirepath.rot_z(locus.m2_rad)
        expanded = np.asarray([
            rotation @ point for point in conductor_prefix
        ])
        direct_source = np.asarray([
            rotation @ point
            for point in full_bore_reference[:len(conductor_prefix)]
        ])
        tensioned = next(
            segment for segment in locus_payload[locus_index]["segments"]
            if segment["name"] == "flyer_tensioned_bore_handoff"
        )
        tensioned_start = np.asarray(
            tensioned["machine_world_samples_mm"][0], dtype=float
        )
        max_error = float(np.max(np.abs(expanded - direct_source)))
        seam_error = float(np.linalg.norm(expanded[-1] - tensioned_start))
        validation_rows.append({
            "locus_index": locus_index,
            "expanded_sha256": hashlib.sha256(
                np.round(expanded, decimals=9).tobytes()
            ).hexdigest(),
            "direct_source_sha256": hashlib.sha256(
                np.round(direct_source, decimals=9).tobytes()
            ).hexdigest(),
            "maximum_point_error_mm": max_error,
            "continuity_to_tensioned_handoff_mm": seam_error,
            "first_world_mm": np.round(expanded[0], 9).tolist(),
            "last_world_mm": np.round(expanded[-1], 9).tolist(),
        })
    payload["flyer_reference_validation"] = {
        "sampled_locus_indices": [0, len(loci) // 2, len(loci) - 1],
        "rows": validation_rows,
        "status": "PASS" if all(
            row["maximum_point_error_mm"] <= 1.0e-12
            and row["continuity_to_tensioned_handoff_mm"] <= 5.0e-8
            and row["expanded_sha256"] == row["direct_source_sha256"]
            for row in validation_rows
        ) else "FAIL",
    }
    if payload["flyer_reference_validation"]["status"] != "PASS":
        raise RuntimeError("flyer reference expansion validation failed")
    payload["locus_payload_sha256"] = _canonical_hash(payload)
    if "torus" in json.dumps(payload).lower():
        raise RuntimeError("obsolete torus metadata leaked into player route")
    return payload


def analyze(
    *, run_full_motion: bool = True,
) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, Any]]:
    events = load_events(CAPTURE)
    timeline = Timeline(events)
    loci, passes = extract_raw_loci(events, timeline)
    if len(loci) != EXPECTED_LOCI:
        raise RuntimeError(f"expected {EXPECTED_LOCI} loci; observed {len(loci)}")
    periodic = periodic_equivalence(loci)
    rows, terminal = terminal_rows(loci)
    cap_binding = deepcopy(_short_leadin_cap_binding_cases())
    cap_binding["locus_count"] = len(rows)
    cap_binding["locus_endpoint_counts"] = terminal["cap_endpoint_counts"]
    cap_binding["maximum_locus_BREP_seam_gap_mm"] = terminal[
        "maximum_cap_lane_BREP_seam_gap_mm"
    ]
    cap_binding["all_2400_loci_bind_actual_cap_lane_BREP"] = (
        len(rows) == EXPECTED_LOCI
        and cap_binding["status"] == "PASS"
        and terminal["maximum_cap_lane_BREP_seam_gap_mm"] <= 1.0e-8
        and sum(terminal["cap_endpoint_counts"].values()) == EXPECTED_LOCI
    )
    cap_binding["status"] = (
        "PASS" if cap_binding[
            "all_2400_loci_bind_actual_cap_lane_BREP"
        ] else "FAIL"
    )
    adjacent_isolation = adjacent_short_leadin_isolation()
    right_seam_access = right_seam_final_brep_accessibility()
    models = _collision_models()
    deposition = deposition_rigid_collision(rows, loci, models)
    arbitrary = arbitrary_m1_and_coil_clearance(models)
    loads = coupled_live_line_loads(rows, loci)
    structure = guide_structure_dfm_and_attachments()
    packaging = outboard_yoke_packaging_audit(events, timeline)
    full_m2_yoke = front_plane_yoke_full_m2_clearance(timeline)
    tolerances = tolerance_budget(
        deposition, arbitrary, structure, full_m2_yoke,
    )
    wraps = shaft_wrap_guide_bypass(events, timeline, models)
    full_motion = (
        full_raw_rigid_motion(events, timeline, models)
        if run_full_motion else {
            "status": "NOT_RUN",
            "authority": "EXACT_FINAL_RIGID_PARTS_NOT_FLEXIBLE_CONDUCTOR",
        }
    )
    locus_payload = player_locus_payload(loci, rows)
    terminal_pass = (
        terminal["locus_count"] == EXPECTED_LOCI
        and terminal["pass_count"] == EXPECTED_LOCI
    )
    topology = {
        name: {
            "aggregate": models[f"{name}_topology"],
            "per_occurrence": models.get(f"{name}_components"),
        }
        for name in (
            "fixed", "spindle", "incremental", "flyer", "coil",
            "final_wound",
        )
    }
    per_solid_topology_pass = all(
        all(
            item["watertight"]
            and item["boundary_edges"] == 0
            and item["nonmanifold_edges"] == 0
            for item in (row["per_occurrence"] or [row["aggregate"]])
        )
        for row in topology.values()
    )
    release_gates = {
        "M0_only_periodic_equivalence_all_2400": periodic["status"] == "PASS",
        "all_2400_physical_bell_terminal_routes_pass": terminal_pass,
        "all_2400_short_leadin_endpoints_join_actual_cap_lane_BREP": (
            cap_binding["status"] == "PASS"
        ),
        "all_48_adjacent_short_leadin_pairs_isolate_R0p25_wire": (
            adjacent_isolation["status"] == "PASS"
        ),
        "all_48_right_seams_accept_R0p36_radial_insertion_gauge": (
            right_seam_access["status"] == "PASS"
        ),
        "minimum_and_maximum_wire_bell_extremes_pass": (
            terminal["minimum_bell_wire_center_radius_mm"] >= 3.25
            and terminal[
                "minimum_straight_bore_handoff_radius_over_0p20_to_0p50mm_wire_mm"
            ] >= 3.0
            and terminal["maximum_bell_turn_deg"] <= flyer.BELL_SWEEP_DEG
        ),
        "per_occurrence_collision_meshes_closed": per_solid_topology_pass,
        "deposition_exact_rigid_pairs_clear_ge_2mm": (
            deposition["status"] == "PASS"
        ),
        "arbitrary_M1_caps_and_copper_clear_ge_2mm": (
            arbitrary["status"] == "PASS"
        ),
        "summed_tolerance_and_10N_deflection_clearance_ge_2mm": (
            tolerances["status"] == "PASS"
        ),
        "guide_yoke_tower_attachment_chain": structure["attachments"]
        ["no_floating_screw_chain_gate"],
        "outboard_yoke_full_M0_carriage_and_static_packaging_clear": (
            packaging["status"] == "PASS"
        ),
        "front_plane_yoke_full_M2_final_flyer_clear_ge_2mm": (
            full_m2_yoke["status"] == "PASS"
        ),
        "yoke_full_section_10N_stress_screen": structure[
            "mass_and_yoke_load"
        ]["stress_gate"],
        "short_leadin_C_section_10N_screen": structure[
            "short_leadin_C_section"
        ]["analytical_gate"],
        "M2_exact_live_line_Leadshine_36V_margin_ge_2x": loads["M2"]
        ["Leadshine_36V_gate_ge_2x"],
        "M2_P30_210_3GT_capacity_ge_2x": loads["M2"]
        ["P30_210_3GT_gate_ge_2x"],
        "M1_wrap_governed_margin_ge_2x": loads["M1"]["gate_ge_2x"],
        "M0_terminal_force_and_added_mass_margin_ge_2x": loads["M0"]
        ["gate_ge_2x"],
        "M2_36V_driver_configuration_verified": loads["M2"]
        ["driver_36V_current_microstep_limits_configured_and_verified"],
        "M2_installed_hot_dyno_verified": loads["M2"]
        ["installed_hot_dyno_verified"],
        "M2_exact_24V_numeric_margin_ge_2x": loads["M2"]
        ["Leadshine_24V_numeric_gate_ge_2x"],
        "M2_24V_release_authorized": loads["M2"]
        ["Leadshine_24V_release_authorized"],
        "M1_closed_loop_drive_fault_safe_behavior_verified": loads["M1"]
        ["drive_fault_safe_behavior_verified"],
        "both_raw_wrap_wire_paths_bypass_fixed_guide_yoke": wraps["gates"]
        ["wire_bypasses_fixed_guide_yoke_for_both_wraps"],
        "full_raw_rigid_sweep_clear": full_motion["status"] == "PASS",
        "both_raw_shaft_wraps_exactly_two_turns": wraps["gates"]
        ["each_raw_wrap_is_two_full_turns"],
        "park_index_load_unload_continuous_conductor_proven": False,
        "PEEK_forming_gauge_polish_abrasion_coupon": False,
        "M3_M4_insert_pull_and_endurance_coupon": False,
        "production_authorized": False,
    }
    geometry_pass = all((
        release_gates["M0_only_periodic_equivalence_all_2400"],
        release_gates["all_2400_physical_bell_terminal_routes_pass"],
        release_gates[
            "all_2400_short_leadin_endpoints_join_actual_cap_lane_BREP"
        ],
        release_gates[
            "all_48_adjacent_short_leadin_pairs_isolate_R0p25_wire"
        ],
        release_gates[
            "all_48_right_seams_accept_R0p36_radial_insertion_gauge"
        ],
        release_gates["per_occurrence_collision_meshes_closed"],
        release_gates["deposition_exact_rigid_pairs_clear_ge_2mm"],
        release_gates["arbitrary_M1_caps_and_copper_clear_ge_2mm"],
        release_gates["summed_tolerance_and_10N_deflection_clearance_ge_2mm"],
        release_gates["guide_yoke_tower_attachment_chain"],
        release_gates[
            "outboard_yoke_full_M0_carriage_and_static_packaging_clear"
        ],
        release_gates[
            "front_plane_yoke_full_M2_final_flyer_clear_ge_2mm"
        ],
        release_gates["yoke_full_section_10N_stress_screen"],
        release_gates["short_leadin_C_section_10N_screen"],
    ))
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": (
            "GEOMETRY_TERMINAL_TOLERANCE_PASS_REVIEW_ONLY__CONTINUOUS_CONDUCTOR_AND_PHYSICAL_GATES_FAIL"
            if geometry_pass else "ACTIVE_SECTOR_GEOMETRY_OR_TOLERANCE_FAIL"
        ),
        "decision": (
            "USE_M0_FOLLOWING_M1_STATIC_ACTIVE_SECTOR__DO_NOT_PRODUCTION_RELEASE"
            if geometry_pass else "DO_NOT_INTEGRATE_ACTIVE_SECTOR"
        ),
        "production_authorized": False,
        "assembly_geometry_integration_authorized": bool(
            geometry_pass and full_motion["status"] == "PASS"
        ),
        "authority_boundary": {
            "raw_capture_and_2400_loci": "authoritative",
            "rigid_geometry": "current integrated_release_candidate source APIs",
            "wire_route": "analytic physical-bell quasi-static tensioned centerline",
            "exact_strand_settling_sag_snags_friction": "not authoritative",
            "continuous_park_index_load_unload": "unmodeled fail-closed",
        },
        "paths": {
            "cad_source": "cad/carriage_active_sector_terminal_guide.py",
            "audit_source": "sim/carriage_active_sector_terminal_guide_audit.py",
            "step": "out/review/carriage_active_sector_terminal_guide.step",
            "report": "out/reports/carriage_active_sector_terminal_guide_audit.json",
            "locus_api": "out/reports/carriage_active_sector_terminal_guide_loci.json",
            "manifest": "out/review/carriage_active_sector_terminal_guide.manifest.json",
        },
        "artifacts": {
            "step": _artifact_row(
                REVIEW / "carriage_active_sector_terminal_guide.step"
            ),
        },
        "collision_authority": {
            "geometry_revision": COLLISION_GEOMETRY_REVISION,
            "mesh_method": (
                "one closed processed mesh per exact physical occurrence; "
                "no compound vertex welding"
            ),
        },
        "collision_cache_provenance": deepcopy(_CACHE_PROVENANCE),
        "integration_api": {
            "fixed_carriage_parts": "carriage_active_sector_terminal_guide.carriage_link_reference_parts()",
            "production_caps": "integrated_release_candidate.cap_module_parts()",
            "final_balanced_flyer": "integrated_release_candidate.retained_rotating_parts()",
            "final_rotating_mass_rows": "integrated_release_candidate.rotating_mass_rows()",
            "route_for_locus": "route_for_locus(RawLocus)",
            "player_locus_payload": "player_locus_payload(loci, rows)",
            "frames": "M0 carriage / M1 spindle / M2 flyer; millimetres and radians",
        },
        "canonical_run": {
            "capture_path": str(CAPTURE.relative_to(ROOT)).replace("\\", "/"),
            "capture_sha256": _sha256(CAPTURE),
            "pass_count": len(passes),
            "locus_count": len(loci),
            "locus_payload_sha256": locus_payload["locus_payload_sha256"],
        },
        "periodic_equivalence": periodic,
        "terminal_deposition_route": terminal,
        "physical_cap_lane_binding": cap_binding,
        "adjacent_short_leadin_isolation": adjacent_isolation,
        "right_seam_final_BREP_accessibility": right_seam_access,
        "deposition_rigid_collision": deposition,
        "arbitrary_M1_and_progressive_copper_clearance": arbitrary,
        "full_raw_rigid_motion": full_motion,
        "collision_mesh_topology": topology,
        "coupled_live_line_loads": loads,
        "guide_structure_DFM_and_attachments": structure,
        "outboard_yoke_packaging": packaging,
        "front_plane_yoke_full_M2_clearance": full_m2_yoke,
        "tolerance_budget": tolerances,
        "shaft_wrap_guide_bypass": wraps,
        "M1_commanded_hold": {
            "architecture": "closed-loop motor hold at commanded index",
            "passive_detent_or_lock_required": False,
            "computed_margin_gate": loads["M1"]["gate_ge_2x"],
            "drive_fault_safe_behavior_verified": loads["M1"]
            ["drive_fault_safe_behavior_verified"],
        },
        "player_route_api": {
            "schema": locus_payload["schema"],
            "path": "out/reports/carriage_active_sector_terminal_guide_loci.json",
            "locus_count": len(locus_payload["loci"]),
            "canonical_payload_sha256": locus_payload[
                "locus_payload_sha256"
            ],
            "compact_file_sha256": None,
            "compact_size_bytes": None,
        },
        "release_gates": release_gates,
        "limits": [
            "Raw shaft wraps remain 1.375 and 2.791667 turns rather than two turns each.",
            "Park, index, load and unload flexible-conductor geometry remains unproved even when rigid parts and both wrap free spans clear.",
            "The wire tubes are quasi-static geometry for path review; tension dynamics, sag, snagging, friction, enamel wear and neatness require hardware.",
            "PEEK finish/gauge/abrasion, insert pull, printed-root and 300 rpm endurance coupons remain mandatory.",
            "M1 relies on closed-loop commanded hold; drive-fault safe behavior still requires control/safety verification.",
        ],
        "source_hashes": {
            "cad/carriage_active_sector_terminal_guide.py": _sha256(
                CAD / "carriage_active_sector_terminal_guide.py"
            ),
            "cad/permanent_cap_production_review.py": _sha256(
                CAD / "permanent_cap_production_review.py"
            ),
            "cad/retained_flyer_peek_guide_successor.py": _sha256(
                CAD / "retained_flyer_peek_guide_successor.py"
            ),
            "cad/integrated_release_candidate.py": _sha256(
                CAD / "integrated_release_candidate.py"
            ),
            "sim/carriage_active_sector_terminal_guide_audit.py": _sha256(
                Path(__file__)
            ),
            "out/capture/upstream_current_raw.jsonl": _sha256(CAPTURE),
            "out/reports/permanent_cap_aggregate_authorization.json": _sha256(
                AGGREGATE
            ),
        },
    }
    report["report_sha256"] = _report_hash(report)
    return report, rows, locus_payload


def render_markdown(report: Mapping[str, Any]) -> str:
    terminal = report["terminal_deposition_route"]
    binding = report["physical_cap_lane_binding"]
    isolation = report["adjacent_short_leadin_isolation"]
    seam_access = report["right_seam_final_BREP_accessibility"]
    deposition = report["deposition_rigid_collision"]["pairs"]
    arbitrary = report["arbitrary_M1_and_progressive_copper_clearance"]["pairs"]
    m2 = report["coupled_live_line_loads"]["M2"]
    tol = report["tolerance_budget"]
    wraps = report["shaft_wrap_guide_bypass"]
    lines = [
        "# Carriage active-sector terminal guide audit",
        "",
        f"**{report['status']}**",
        "",
        "## Route and clearance",
        "",
        f"- Physical bell routes: {terminal['pass_count']}/{terminal['locus_count']} PASS.",
        f"- Exact cap-lane BREP seams: {binding['status']}; maximum gap {binding['maximum_locus_BREP_seam_gap_mm']:.12g} mm.",
        f"- Named short-leadin centerline radii: minimum {terminal['minimum_short_leadin_named_centerline_radius_mm']:.6f} mm.",
        f"- Adjacent open-channel isolation: {isolation['status']}; intended-wire reserve {min(row['intended_wire_reserve_after_profile_tolerance_mm'] for row in isolation['cases']):.6f} mm; separator-web reserve {min(row['separator_web_after_two_sided_profile_tolerance_mm'] for row in isolation['cases']):.6f} mm.",
        f"- Finished right-seam accessibility: {seam_access['status']}; {seam_access['seam_count']} R0.36 gauges, maximum positive overlap {seam_access['maximum_gauge_positive_overlap_mm3']:.12g} mm3.",
        f"- Fixed guide to final flyer: {deposition['fixed_to_flyer']['minimum_clearance_mm']:.6f} mm nominal.",
        f"- Fixed guide to production caps: {arbitrary['fixed_to_caps_all_M1']['minimum_clearance_mm']:.6f} mm over arbitrary M1.",
        f"- Fixed guide to conservative coil: {arbitrary['fixed_to_conservative_coil_all_M1']['minimum_clearance_mm']:.6f} mm.",
        f"- Summed adverse tolerance/deflection budget: {tol['summed_one_sided_adverse_budget_mm']:.6f} mm; status {tol['status']}.",
        "",
        "## Live-line loads",
        "",
        f"- M2 perpendicular lever: {m2['maximum_perpendicular_live_line_lever_mm']:.6f} mm; 10 N torque {m2['wire_torque_at_10N_nm']:.6f} N m.",
        f"- Exact final flyer J: {m2['integrated_rotating_izz_kg_m2']:.12g} kg m2; full output J {m2['full_output_inertia_kg_m2']:.12g} kg m2.",
        f"- Leadshine 36 V available/required: {m2['Leadshine_36V_available_to_required_multiple']:.3f}x.",
        "",
        "## Fail-closed items",
        "",
        f"- Raw wrap turns: {[row['raw_turns'] for row in wraps['cases']]} (not two each).",
        "- Continuous park/index/load/unload path: unproved.",
        "- Physical finish, abrasion, pull and endurance coupons: open.",
        "",
        f"Locus API canonical SHA-256: `{report['player_route_api']['canonical_payload_sha256']}`",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ]
    return "\n".join(lines)


def write_reports(*, run_full_motion: bool = True) -> dict[str, Any]:
    report, rows, locus_payload = analyze(run_full_motion=run_full_motion)
    REPORTS.mkdir(parents=True, exist_ok=True)
    REVIEW.mkdir(parents=True, exist_ok=True)
    LOCUS_OUT.write_text(
        json.dumps(
            locus_payload, separators=(",", ":"), allow_nan=False
        ) + "\n",
        encoding="utf-8",
    )
    report["player_route_api"]["compact_file_sha256"] = _sha256(LOCUS_OUT)
    report["player_route_api"]["compact_size_bytes"] = (
        LOCUS_OUT.stat().st_size
    )
    report["artifacts"]["step"] = _artifact_row(
        REVIEW / "carriage_active_sector_terminal_guide.step"
    )
    report["collision_cache_provenance"] = deepcopy(_CACHE_PROVENANCE)
    report["report_sha256"] = _report_hash(report)
    JSON_OUT.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    MD_OUT.write_text(render_markdown(report), encoding="utf-8")
    manifest = {
        "schema": "carriage-active-sector-terminal-guide-manifest/v1",
        "status": report["status"],
        "source": report["paths"]["cad_source"],
        "step": report["paths"]["step"],
        "artifacts": report["artifacts"],
        "collision_authority": report["collision_authority"],
        "collision_cache_provenance": report[
            "collision_cache_provenance"
        ],
        "integration_api": report["integration_api"],
        "locus_api": {
            **report["player_route_api"],
        },
        "coupled_live_line_loads": report["coupled_live_line_loads"],
        "tolerance_budget": report["tolerance_budget"],
        "release_gates": report["release_gates"],
        "report_sha256": report["report_sha256"],
    }
    MANIFEST_OUT.write_text(
        json.dumps(manifest, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return report


if __name__ == "__main__":
    result = write_reports(run_full_motion=True)
    print(
        result["status"],
        result["terminal_deposition_route"]["pass_count"],
        result["full_raw_rigid_motion"].get("sample_count", 0),
        result["player_route_api"]["canonical_payload_sha256"],
    )
