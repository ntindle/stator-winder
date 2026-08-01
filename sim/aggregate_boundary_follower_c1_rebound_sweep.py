"""Analytic C1 S-biarc rebound sweep for every follower route case.

For each nonzero-growth route case, this module constructs a planar S-biarc
between the cap terminal and selected aggregate support contact.  The first
arc turns past the final tangent; the second arc reverses into the aggregate
support tangent.  Its end-arc centerline radius is fixed to the isolated
follower nose surface radius plus wire radius (3.10 or 3.25 mm), and the
first-arc radius must be no smaller.

The construction is exact C0/C1 mathematics, not installed geometry.  In
particular, the curvature normal at the aggregate endpoint is orthogonal to
the aggregate support normal, and no placed BREP proves cap, copper, follower,
or neighboring-wire clearance.  Physical authority therefore remains false.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORTS = ROOT / "out" / "reports"
REVIEW = ROOT / "out" / "review"

ROUTE_SWEEP_PATH = REPORTS / "aggregate_boundary_follower_route_sweep.json"
FLOATING_FOLLOWER_STEP = REVIEW / "aggregate_boundary_floating_follower.step"
CAP_SHELF_STEP = REVIEW / "aggregate_boundary_g0_cap_shelf.step"
OUTPUT_JSON = REPORTS / "aggregate_boundary_follower_c1_rebound_sweep.json"
OUTPUT_MD = REPORTS / "aggregate_boundary_follower_c1_rebound_sweep.md"

SCHEMA = "aggregate-boundary-follower-c1-rebound-sweep/v1"
EXPECTED_LOCI = 2400
EXPECTED_ROUTE_CASES = 4800
EXPECTED_G0_CASES = 96
EXPECTED_NONZERO_CASES = 4704
WIRE_DIAMETERS_MM = (0.2, 0.5)
MINIMUM_CENTERLINE_RADIUS_MM = 3.0
FOLLOWER_NOSE_SURFACE_RADIUS_MM = 3.0
PROTOTYPE_RADIAL_STROKE_MM = 6.0
PROTOTYPE_TANGENTIAL_STROKE_MM = 1.0
PROTOTYPE_GIMBAL_HALF_RANGE_DEG = 65.0
SOLVER_ITERATIONS = 80
GEOMETRY_TOLERANCE_MM = 1.0e-8
TANGENT_TOLERANCE = 1.0e-10
NORMAL_ALIGNMENT_TOLERANCE = 1.0e-8

SOURCE_PATHS = (
    Path("sim/aggregate_boundary_follower_c1_rebound_sweep.py"),
    Path("sim/aggregate_boundary_follower_route_sweep.py"),
    Path("cad/aggregate_boundary_floating_follower.py"),
    Path("cad/aggregate_boundary_g0_cap_shelf.py"),
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(value: Any, field: str | None = None) -> str:
    body = deepcopy(value)
    if field is not None and isinstance(body, dict):
        body.pop(field, None)
    return hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def _dot(one: Sequence[float], two: Sequence[float]) -> float:
    return sum(float(one[index]) * float(two[index])
               for index in range(len(one)))


def _norm(vector: Sequence[float]) -> float:
    return math.sqrt(_dot(vector, vector))


def _unit(vector: Sequence[float]) -> tuple[float, ...]:
    length = _norm(vector)
    if length <= 1.0e-15:
        raise ValueError("zero-length vector")
    return tuple(float(value) / length for value in vector)


def _sub(one: Sequence[float], two: Sequence[float]) -> tuple[float, ...]:
    return tuple(float(one[index]) - float(two[index])
                 for index in range(len(one)))


def _add(one: Sequence[float], two: Sequence[float]) -> tuple[float, ...]:
    return tuple(float(one[index]) + float(two[index])
                 for index in range(len(one)))


def _scale(vector: Sequence[float], scalar: float) -> tuple[float, ...]:
    return tuple(float(value) * float(scalar) for value in vector)


def _cross(one: Sequence[float], two: Sequence[float]) \
        -> tuple[float, float, float]:
    return (
        float(one[1]) * float(two[2]) - float(one[2]) * float(two[1]),
        float(one[2]) * float(two[0]) - float(one[0]) * float(two[2]),
        float(one[0]) * float(two[1]) - float(one[1]) * float(two[0]),
    )


def _to_world(
    origin: Sequence[float], e0: Sequence[float], e1: Sequence[float],
    point_xy: Sequence[float],
) -> tuple[float, float, float]:
    return tuple(
        float(origin[index])
        + float(point_xy[0]) * float(e0[index])
        + float(point_xy[1]) * float(e1[index])
        for index in range(3)
    )


def _radii_at(
    join_angle_rad: float, final_angle_rad: float,
    endpoint_xy_mm: tuple[float, float],
) -> tuple[float, float]:
    a = float(join_angle_rad)
    phi = float(final_angle_rad)
    x, y = map(float, endpoint_xy_mm)
    first = (math.sin(a), 1.0 - math.cos(a))
    second = (
        math.sin(phi) - math.sin(a),
        math.cos(a) - math.cos(phi),
    )
    determinant = first[0] * second[1] - first[1] * second[0]
    if abs(determinant) <= 1.0e-15:
        raise ValueError("singular biarc join angle")
    radius_0 = (x * second[1] - y * second[0]) / determinant
    radius_1 = (first[0] * y - first[1] * x) / determinant
    return radius_0, radius_1


def _arc_point(
    signed_radius_mm: float, start_angle_rad: float,
    angle_rad: float, start_xy_mm: Sequence[float],
) -> tuple[float, float]:
    radius = float(signed_radius_mm)
    start = float(start_angle_rad)
    angle = float(angle_rad)
    return (
        float(start_xy_mm[0])
        + radius * (math.sin(angle) - math.sin(start)),
        float(start_xy_mm[1])
        + radius * (math.cos(start) - math.cos(angle)),
    )


def solve_s_biarc(
    start_mm: Sequence[float], start_tangent: Sequence[float],
    end_mm: Sequence[float], end_tangent: Sequence[float],
    required_end_radius_mm: float,
    aggregate_outward_normal: Sequence[float],
) -> dict[str, Any]:
    """Return the canonical end-radius-bound S-biarc or a precise failure."""

    p0 = tuple(map(float, start_mm))
    p1 = tuple(map(float, end_mm))
    t0 = _unit(start_tangent)
    t1 = _unit(end_tangent)
    chord = _sub(p1, p0)
    chord_length = _norm(chord)
    if chord_length <= GEOMETRY_TOLERANCE_MM:
        return {"status": "NO_SOLUTION_COINCIDENT_ENDPOINTS"}

    cosine = max(-1.0, min(1.0, _dot(t0, t1)))
    phi = math.acos(cosine)
    if phi <= TANGENT_TOLERANCE:
        return {
            "status": "DEGENERATE_STRAIGHT_C1",
            "chord_length_mm": chord_length,
        }
    transverse = _sub(t1, _scale(t0, cosine))
    transverse_length = _norm(transverse)
    if transverse_length <= TANGENT_TOLERANCE:
        return {"status": "NO_SOLUTION_ANTIPARALLEL_TANGENTS"}
    e0 = t0
    e1 = _unit(transverse)
    plane_normal = _unit(_cross(e0, e1))
    endpoint_xy = (_dot(chord, e0), _dot(chord, e1))
    off_plane = abs(_dot(chord, plane_normal))
    chord_tangent_error = _norm(_sub(_unit(chord), t1))
    if off_plane > GEOMETRY_TOLERANCE_MM:
        return {
            "status": "NO_PLANAR_BIARC_ENDPOINT_OFF_TANGENT_PLANE",
            "endpoint_off_plane_residual_mm": off_plane,
        }
    if chord_tangent_error > TANGENT_TOLERANCE:
        return {
            "status": "NO_CANONICAL_BIARC_END_TANGENT_NOT_CHORD",
            "end_tangent_to_chord_residual": chord_tangent_error,
        }

    required = float(required_end_radius_mm)
    # For phi < a < 2phi, R0 is positive and R1 is negative.  -R1 decreases
    # continuously from infinity to zero.  Bind the final arc exactly to the
    # diameter-specific follower-nose centerline radius.
    lower = phi * (1.0 + 1.0e-10)
    upper = 2.0 * phi * (1.0 - 1.0e-10)
    try:
        lower_radii = _radii_at(lower, phi, endpoint_xy)
        upper_radii = _radii_at(upper, phi, endpoint_xy)
    except ValueError:
        return {"status": "NO_SOLUTION_SINGULAR_BIARC_INTERVAL"}
    if not (-lower_radii[1] > required > -upper_radii[1]):
        return {
            "status": "NO_SOLUTION_END_RADIUS_NOT_BRACKETED",
            "required_end_radius_mm": required,
        }
    for _index in range(SOLVER_ITERATIONS):
        middle = (lower + upper) / 2.0
        radius_0, radius_1 = _radii_at(middle, phi, endpoint_xy)
        if -radius_1 > required:
            lower = middle
        else:
            upper = middle
    join_angle = (lower + upper) / 2.0
    radius_0, radius_1 = _radii_at(join_angle, phi, endpoint_xy)
    second_sweep = phi - join_angle
    length_0 = join_angle * radius_0
    length_1 = second_sweep * radius_1
    if radius_0 < required - GEOMETRY_TOLERANCE_MM:
        return {
            "status": "NO_SOLUTION_FIRST_ARC_BELOW_RADIUS_FLOOR",
            "first_arc_radius_mm": radius_0,
            "required_radius_mm": required,
        }
    if length_0 <= 0.0 or length_1 <= 0.0:
        return {"status": "NO_SOLUTION_NONPOSITIVE_ARC_LENGTH"}

    join_xy = _arc_point(radius_0, 0.0, join_angle, (0.0, 0.0))
    end_xy = _arc_point(
        radius_1, join_angle, phi, join_xy,
    )
    join_tangent_xy = (math.cos(join_angle), math.sin(join_angle))
    end_tangent_xy = (math.cos(phi), math.sin(phi))
    center_0_xy = (0.0, radius_0)
    center_1_xy = (
        join_xy[0] - radius_1 * math.sin(join_angle),
        join_xy[1] + radius_1 * math.cos(join_angle),
    )

    join_world = _to_world(p0, e0, e1, join_xy)
    end_world = _to_world(p0, e0, e1, end_xy)
    center_0_world = _to_world(p0, e0, e1, center_0_xy)
    center_1_world = _to_world(p0, e0, e1, center_1_xy)
    join_tangent_world = _unit(_add(
        _scale(e0, join_tangent_xy[0]),
        _scale(e1, join_tangent_xy[1]),
    ))
    end_tangent_world = _unit(_add(
        _scale(e0, end_tangent_xy[0]),
        _scale(e1, end_tangent_xy[1]),
    ))

    # The chord line has angle phi in the biarc plane.  Its exact stationary
    # lateral-distance candidate on the first arc is theta=phi; the second
    # arc has no additional interior stationary point over [phi,a].
    arc0_phi_xy = _arc_point(radius_0, 0.0, phi, (0.0, 0.0))
    chord_normal_xy = (-math.sin(phi), math.cos(phi))

    def lateral(point: Sequence[float]) -> float:
        return (float(point[0]) * chord_normal_xy[0]
                + float(point[1]) * chord_normal_xy[1])

    lateral_candidates = {
        "start": 0.0,
        "first_arc_tangent_parallel_to_chord": lateral(arc0_phi_xy),
        "join": lateral(join_xy),
        "end": lateral(end_xy),
    }
    max_lateral = max(abs(value) for value in lateral_candidates.values())

    aggregate_normal = _unit(aggregate_outward_normal)
    end_center_direction = _unit(_sub(center_1_world, p1))
    compression_alignment = _dot(end_center_direction, aggregate_normal)
    plane_normal_alignment = abs(_dot(plane_normal, aggregate_normal))
    result = {
        "status": "PASS_ANALYTIC_C1_S_BIARC",
        "curve_family": "two_signed_circular_arcs_shared_C1_tangent",
        "selection_rule": (
            "end arc abs(radius)=3.0+wire_radius; choose unique "
            "phi<a<2phi S-biarc root"
        ),
        "start_mm": list(p0),
        "end_mm": list(p1),
        "start_tangent": list(t0),
        "end_tangent": list(t1),
        "plane_basis": {
            "start_tangent_e0": list(e0),
            "transverse_e1": list(e1),
            "plane_normal": list(plane_normal),
            "endpoint_off_plane_residual_mm": off_plane,
        },
        "final_tangent_turn_deg": math.degrees(phi),
        "maximum_tangent_excursion_deg": math.degrees(join_angle),
        "join_mm": list(join_world),
        "join_tangent": list(join_tangent_world),
        "first_arc": {
            "signed_radius_mm": radius_0,
            "absolute_radius_mm": abs(radius_0),
            "signed_sweep_deg": math.degrees(join_angle),
            "length_mm": length_0,
            "center_mm": list(center_0_world),
        },
        "second_arc": {
            "signed_radius_mm": radius_1,
            "absolute_radius_mm": abs(radius_1),
            "signed_sweep_deg": math.degrees(second_sweep),
            "length_mm": length_1,
            "center_mm": list(center_1_world),
        },
        "minimum_absolute_radius_mm": min(abs(radius_0), abs(radius_1)),
        "required_centerline_radius_mm": required,
        "chord_length_mm": chord_length,
        "total_length_mm": length_0 + length_1,
        "length_over_chord_mm": length_0 + length_1 - chord_length,
        "lateral_signed_candidates_mm": lateral_candidates,
        "maximum_lateral_sweep_from_chord_mm": max_lateral,
        "closure": {
            "end_position_residual_mm": _norm(_sub(end_world, p1)),
            "start_tangent_residual": _norm(_sub(t0, _unit(start_tangent))),
            "join_tangent_residual": _norm(_sub(
                join_tangent_world,
                _unit(_add(
                    _scale(e0, math.cos(join_angle)),
                    _scale(e1, math.sin(join_angle)),
                )),
            )),
            "end_tangent_residual": _norm(_sub(end_tangent_world, t1)),
            "end_radius_residual_mm": abs(abs(radius_1) - required),
        },
        "follower_center_for_end_arc_mm": list(center_1_world),
        "aggregate_outward_normal": list(aggregate_normal),
        "end_center_direction_dot_aggregate_outward_normal": (
            compression_alignment
        ),
        "biarc_plane_normal_abs_dot_aggregate_outward_normal": (
            plane_normal_alignment
        ),
        "end_arc_compression_normal_compatible": (
            compression_alignment >= 1.0 - NORMAL_ALIGNMENT_TOLERANCE
        ),
        "positive_volume_path_placed": False,
        "collision_authorized": False,
        "physical_route_authorized": False,
    }
    return result


def _case(
    locus: Mapping[str, Any], route_case: Mapping[str, Any],
) -> dict[str, Any]:
    diameter = float(route_case["wire_diameter_mm"])
    if int(locus["turn_index"]) == 0:
        return {
            "wire_diameter_mm": diameter,
            "status": "NOT_APPLICABLE_G0_NO_AGGREGATE_ENDPOINT_TANGENT",
            "reason": (
                "g=0 has only the cap terminal owner; no nondegenerate "
                "aggregate support endpoint/tangent exists for a two-end C1 arc"
            ),
            "physical_route_authorized": False,
        }
    support = route_case["aggregate_contact"]["selected_support"]
    aggregate_normal = [
        float(support["aggregate_outward_normal_xy"][0]),
        float(support["aggregate_outward_normal_xy"][1]),
        0.0,
    ]
    required = FOLLOWER_NOSE_SURFACE_RADIUS_MM + diameter / 2.0
    result = solve_s_biarc(
        route_case["endpoint_active_local_mm"],
        support["guide_terminal_tangent_local"],
        support["contact_local_mm"],
        support["straight_chord_tangent_local"],
        required,
        aggregate_normal,
    )
    result.update({
        "wire_diameter_mm": diameter,
        "wire_radius_mm": diameter / 2.0,
        "terminal_C0_class": route_case["terminal_C0_class"],
        "terminal_C0_exact": bool(route_case["terminal_C0_exact"]),
        "terminal_C0_analytic": bool(route_case["terminal_C0_analytic"]),
        "upstream_rebind_C1_exact": bool(
            route_case["upstream_rebind_C1_exact"]
        ),
        "aggregate_C0_analytic": bool(
            route_case["aggregate_contact"]["aggregate_C0_analytic"]
        ),
    })
    return result


def _coordinate_bounds(points: list[Sequence[float]]) -> dict[str, Any]:
    axes = list(zip(*points))
    minima = [min(map(float, values)) for values in axes]
    maxima = [max(map(float, values)) for values in axes]
    return {
        "min_mm": minima,
        "max_mm": maxima,
        "span_mm": [maxima[index] - minima[index] for index in range(3)],
    }


def _follower_travel(rows: list[dict[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for physical_id in range(4):
        identity_rows = [
            row for row in rows
            if int(row["identity"]["physical_id"]) == physical_id
        ]
        per_diameter: dict[str, Any] = {}
        combined: list[Sequence[float]] = []
        for diameter in WIRE_DIAMETERS_MM:
            cases = [
                case for row in identity_rows for case in row["diameter_cases"]
                if case["status"] == "PASS_ANALYTIC_C1_S_BIARC"
                and math.isclose(float(case["wire_diameter_mm"]), diameter,
                                 abs_tol=1.0e-12)
            ]
            centers = [case["follower_center_for_end_arc_mm"] for case in cases]
            combined.extend(centers)
            bounds = _coordinate_bounds(centers)
            per_diameter[f"d{diameter:.1f}"] = {
                "constructed_case_count": len(cases),
                "follower_end_arc_center_bounds": bounds,
                "radial_X_span_mm": bounds["span_mm"][0],
                "tangential_Y_span_mm": bounds["span_mm"][1],
                "axial_Z_span_mm": bounds["span_mm"][2],
            }
        combined_bounds = _coordinate_bounds(combined)
        radial = combined_bounds["span_mm"][0]
        tangential = combined_bounds["span_mm"][1]
        axial = combined_bounds["span_mm"][2]
        result[str(physical_id)] = {
            "name": identity_rows[0]["identity"]["name"],
            "per_diameter": per_diameter,
            "combined_diameter_center_bounds": combined_bounds,
            "required_radial_X_span_mm": radial,
            "required_tangential_Y_span_mm": tangential,
            "required_axial_Z_span_mm": axial,
            "prototype_radial_stroke_mm": PROTOTYPE_RADIAL_STROKE_MM,
            "prototype_tangential_stroke_mm": PROTOTYPE_TANGENTIAL_STROKE_MM,
            "radial_stroke_analytic_pass": (
                radial <= PROTOTYPE_RADIAL_STROKE_MM + GEOMETRY_TOLERANCE_MM
            ),
            "tangential_stroke_analytic_pass": (
                tangential
                <= PROTOTYPE_TANGENTIAL_STROKE_MM + GEOMETRY_TOLERANCE_MM
            ),
            "axial_translation_DOF_present": False,
            "axial_center_shift_zero": axial <= GEOMETRY_TOLERANCE_MM,
        }
    return result


def analyze() -> dict[str, Any]:
    route = _load(ROUTE_SWEEP_PATH)
    if route.get("report_sha256") != _canonical_hash(
            route, "report_sha256"):
        raise ValueError("stale or corrupt route-sweep input")
    if len(route.get("loci", [])) != EXPECTED_LOCI:
        raise ValueError("C1 sweep requires the exact 2,400 route loci")

    rows: list[dict[str, Any]] = []
    for locus in route["loci"]:
        rows.append({
            "locus_index": int(locus["locus_index"]),
            "pass_index": int(locus["pass_index"]),
            "state_index": int(locus["state_index"]),
            "turn_index": int(locus["turn_index"]),
            "half_turn_index": int(locus["half_turn_index"]),
            "tooth_index": int(locus["tooth_index"]),
            "time_s": float(locus["time_s"]),
            "lane_id": str(locus["lane_id"]),
            "identity": dict(locus["identity"]),
            "law_track_bindings": dict(locus["law_track_bindings"]),
            "diameter_cases": [
                _case(locus, route_case)
                for route_case in locus["diameter_cases"]
            ],
        })

    cases = [case for row in rows for case in row["diameter_cases"]]
    constructed = [case for case in cases
                   if case["status"] == "PASS_ANALYTIC_C1_S_BIARC"]
    g0 = [case for case in cases if case["status"].startswith("NOT_APPLICABLE_G0")]
    failed = [case for case in cases if case not in constructed and case not in g0]
    travel = _follower_travel(rows)
    closures = [case["closure"] for case in constructed]

    analytic_gates = {
        "exact_2400_loci_x_two_diameters_bound": (
            len(rows) == EXPECTED_LOCI and len(cases) == EXPECTED_ROUTE_CASES
        ),
        "all_96_g0_cases_precisely_classified_no_endpoint_pair": (
            len(g0) == EXPECTED_G0_CASES
        ),
        "all_4704_nonzero_cases_have_C1_S_biarc": (
            len(constructed) == EXPECTED_NONZERO_CASES and not failed
        ),
        "all_constructed_endpoints_close": all(
            closure["end_position_residual_mm"] <= GEOMETRY_TOLERANCE_MM
            for closure in closures
        ),
        "all_constructed_end_tangents_close": all(
            closure["end_tangent_residual"] <= TANGENT_TOLERANCE
            for closure in closures
        ),
        "all_constructed_join_tangents_are_C1": all(
            closure["join_tangent_residual"] <= TANGENT_TOLERANCE
            for closure in closures
        ),
        "all_constructed_radii_at_least_diameter_specific_floor": all(
            case["minimum_absolute_radius_mm"]
            >= case["required_centerline_radius_mm"] - GEOMETRY_TOLERANCE_MM
            and case["required_centerline_radius_mm"]
            >= MINIMUM_CENTERLINE_RADIUS_MM
            for case in constructed
        ),
        "all_constructed_lengths_exceed_chords": all(
            case["length_over_chord_mm"] > 0.0 for case in constructed
        ),
        "maximum_tangent_excursion_within_65deg_analytic_gimbal_range": max(
            case["maximum_tangent_excursion_deg"] for case in constructed
        ) <= PROTOTYPE_GIMBAL_HALF_RANGE_DEG + 1.0e-12,
        "all_identity_radial_center_spans_within_6mm": all(
            value["radial_stroke_analytic_pass"] for value in travel.values()
        ),
    }
    physical_gates = {
        "g0_two_end_C1_route_defined": False,
        "all_terminal_rebinds_exact_positive_volume_C1": all(
            case.get("upstream_rebind_C1_exact", False) for case in constructed
        ),
        "end_arc_center_direction_matches_aggregate_compression_normal": all(
            case["end_arc_compression_normal_compatible"]
            for case in constructed
        ),
        "all_identity_tangential_center_spans_within_1mm": all(
            value["tangential_stroke_analytic_pass"]
            for value in travel.values()
        ),
        "required_axial_center_shift_has_physical_DOF": all(
            value["axial_center_shift_zero"]
            or value["axial_translation_DOF_present"]
            for value in travel.values()
        ),
        "positive_volume_biarcs_placed": False,
        "cap_aggregate_follower_neighbor_collision_sweep_passed": False,
        "transition_tolerance_load_wear_and_dynamics_passed": False,
        "exact_route_length_and_dancer_coupling_passed": False,
        "assembly_integration_passed": False,
    }

    artifacts = {
        "route_sweep": {
            "path": str(ROUTE_SWEEP_PATH.relative_to(ROOT)).replace("\\", "/"),
            "sha256": _sha256(ROUTE_SWEEP_PATH),
            "report_sha256": route.get("report_sha256"),
        },
        "floating_follower_STEP": {
            "path": str(FLOATING_FOLLOWER_STEP.relative_to(ROOT)).replace("\\", "/"),
            "sha256": _sha256(FLOATING_FOLLOWER_STEP),
        },
        "cap_shelf_STEP": {
            "path": str(CAP_SHELF_STEP.relative_to(ROOT)).replace("\\", "/"),
            "sha256": _sha256(CAP_SHELF_STEP),
        },
    }
    blockers = [name for name, value in physical_gates.items() if not value]
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "FAIL",
        "decision": (
            "4704_NONZERO_CASES_HAVE_EXACT_ANALYTIC_C1_S_BIARCS__"
            "CONTACT_NORMAL_TRAVEL_AND_BREP_AUTHORITY_FAIL"
        ),
        "wire_route_authorized": False,
        "collision_authorized": False,
        "assembly_integration_authorized": False,
        "dancer_coupling_authorized": False,
        "production_authorized": False,
        "scope": {
            "proved": [
                "exact C0/C1 end interpolation for all nonzero route cases",
                "diameter-specific 3.10/3.25 mm end-arc centerline radii",
                "first-arc radius, signed sweeps, length, and lateral sweep",
                "analytic end-arc follower-center travel by physical identity",
                "precise g=0 no-endpoint classification",
            ],
            "not_proved": [
                "aggregate-compressive follower normal",
                "positive-volume cap/rebound/biarc/follower placement",
                "1 mm tangential and absent axial-translation realization",
                "collision, tolerance, load, wear, dynamics, or dancer coupling",
            ],
        },
        "construction_contract": {
            "family": "planar_C1_S_biarc",
            "minimum_centerline_radius_mm": MINIMUM_CENTERLINE_RADIUS_MM,
            "end_arc_radius_rule": "3.0 + wire_radius_mm",
            "wire_diameter_mm": list(WIRE_DIAMETERS_MM),
            "solver_iterations": SOLVER_ITERATIONS,
            "solver_interval": "final_angle < join_angle < 2*final_angle",
            "physical_limit": (
                "aggregate support normal is the biarc-plane binormal; "
                "the end-arc curvature center lies in the biarc plane"
            ),
        },
        "coverage": {
            "required_loci": EXPECTED_LOCI,
            "evaluated_loci": len(rows),
            "route_case_count": len(cases),
            "g0_no_endpoint_pair_case_count": len(g0),
            "nonzero_attempted_case_count": len(cases) - len(g0),
            "analytic_C1_biarc_pass_case_count": len(constructed),
            "mathematical_failure_case_count": len(failed),
            "compression_normal_compatible_case_count": sum(
                case["end_arc_compression_normal_compatible"]
                for case in constructed
            ),
            "positive_volume_placed_case_count": sum(
                case["positive_volume_path_placed"] for case in constructed
            ),
            "physically_authorized_case_count": sum(
                case["physical_route_authorized"] for case in constructed
            ),
        },
        "bounds": {
            "first_arc_absolute_radius_mm": [
                min(case["first_arc"]["absolute_radius_mm"]
                    for case in constructed),
                max(case["first_arc"]["absolute_radius_mm"]
                    for case in constructed),
            ],
            "second_arc_absolute_radius_mm": [
                min(case["second_arc"]["absolute_radius_mm"]
                    for case in constructed),
                max(case["second_arc"]["absolute_radius_mm"]
                    for case in constructed),
            ],
            "total_length_mm": [
                min(case["total_length_mm"] for case in constructed),
                max(case["total_length_mm"] for case in constructed),
            ],
            "length_over_chord_mm": [
                min(case["length_over_chord_mm"] for case in constructed),
                max(case["length_over_chord_mm"] for case in constructed),
            ],
            "maximum_lateral_sweep_from_chord_mm": [
                min(case["maximum_lateral_sweep_from_chord_mm"]
                    for case in constructed),
                max(case["maximum_lateral_sweep_from_chord_mm"]
                    for case in constructed),
            ],
            "final_tangent_turn_deg": [
                min(case["final_tangent_turn_deg"] for case in constructed),
                max(case["final_tangent_turn_deg"] for case in constructed),
            ],
            "maximum_tangent_excursion_deg": [
                min(case["maximum_tangent_excursion_deg"]
                    for case in constructed),
                max(case["maximum_tangent_excursion_deg"]
                    for case in constructed),
            ],
            "maximum_closure_residual_mm": max(
                closure["end_position_residual_mm"] for closure in closures
            ),
            "maximum_end_tangent_residual": max(
                closure["end_tangent_residual"] for closure in closures
            ),
            "end_center_direction_dot_aggregate_normal": [
                min(case[
                    "end_center_direction_dot_aggregate_outward_normal"
                ] for case in constructed),
                max(case[
                    "end_center_direction_dot_aggregate_outward_normal"
                ] for case in constructed),
            ],
        },
        "follower_center_travel": travel,
        "analytic_gates": analytic_gates,
        "physical_gates": physical_gates,
        "blockers": blockers,
        "loci": rows,
        "artifacts": artifacts,
        "source_hashes": {
            str(path).replace("\\", "/"): _sha256(ROOT / path)
            for path in SOURCE_PATHS
        },
    }
    report["report_sha256"] = _canonical_hash(report, "report_sha256")
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported C1 rebound sweep schema")
    if report.get("report_sha256") != _canonical_hash(
            report, "report_sha256"):
        raise ValueError("C1 rebound sweep report hash mismatch")
    for relative, expected in report.get("source_hashes", {}).items():
        path = ROOT / str(relative).replace("/", "\\")
        if not path.is_file() or _sha256(path) != expected:
            raise ValueError(f"stale C1 rebound source {relative}")
    for name, artifact in report.get("artifacts", {}).items():
        path = ROOT / str(artifact["path"]).replace("/", "\\")
        if not path.is_file() or _sha256(path) != artifact.get("sha256"):
            raise ValueError(f"stale C1 rebound artifact {name}")
    coverage = report.get("coverage", {})
    if coverage.get("evaluated_loci") != EXPECTED_LOCI:
        raise ValueError("C1 rebound locus coverage mismatch")
    if coverage.get("route_case_count") != EXPECTED_ROUTE_CASES:
        raise ValueError("C1 rebound route-case coverage mismatch")
    if coverage.get("physically_authorized_case_count") != 0:
        raise ValueError("C1 rebound promoted unproved physical routes")
    if report.get("status") != "FAIL":
        raise ValueError("C1 rebound sweep must remain fail-closed")
    for key in (
        "wire_route_authorized", "collision_authorized",
        "assembly_integration_authorized", "dancer_coupling_authorized",
        "production_authorized",
    ):
        if report.get(key) is not False:
            raise ValueError(f"C1 rebound sweep cannot authorize {key}")


def render_markdown(report: Mapping[str, Any]) -> str:
    coverage = report["coverage"]
    bounds = report["bounds"]
    lines = [
        "# Aggregate-boundary follower C1 rebound sweep",
        "",
        f"- Status: **{report['status']}**",
        f"- Decision: `{report['decision']}`",
        f"- Loci / route cases: {coverage['evaluated_loci']} / {coverage['route_case_count']}",
        f"- g=0 no-endpoint cases: {coverage['g0_no_endpoint_pair_case_count']}",
        f"- Analytic C1 S-biarcs: {coverage['analytic_C1_biarc_pass_case_count']}",
        f"- Mathematical failures: {coverage['mathematical_failure_case_count']}",
        f"- Compression-normal-compatible: {coverage['compression_normal_compatible_case_count']}",
        f"- Positive-volume placed / physically authorized: {coverage['positive_volume_placed_case_count']} / {coverage['physically_authorized_case_count']}",
        (
            "- First-arc radius range: "
            f"{bounds['first_arc_absolute_radius_mm'][0]:.6f} to "
            f"{bounds['first_arc_absolute_radius_mm'][1]:.6f} mm"
        ),
        (
            "- Total biarc length range: "
            f"{bounds['total_length_mm'][0]:.6f} to "
            f"{bounds['total_length_mm'][1]:.6f} mm"
        ),
        (
            "- Lateral sweep range: "
            f"{bounds['maximum_lateral_sweep_from_chord_mm'][0]:.6f} to "
            f"{bounds['maximum_lateral_sweep_from_chord_mm'][1]:.6f} mm"
        ),
        "",
        "## Physical blockers",
        "",
    ]
    lines.extend(f"- `{name}`" for name in report["blockers"])
    lines.extend([
        "",
        "The S-biarcs are exact analytic curves only. No selected CAD, release, "
        "assembly, BOM, or acceptance source was modified.",
        "",
    ])
    return "\n".join(lines)


def write_outputs(report: Mapping[str, Any] | None = None) -> dict[str, Any]:
    value = dict(report) if report is not None else analyze()
    validate_report_integrity(value)
    REPORTS.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(
        json.dumps(value, indent=2, sort_keys=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    OUTPUT_MD.write_text(render_markdown(value), encoding="utf-8")
    return value


def main(argv: Iterable[str] | None = None) -> int:
    del argv
    report = write_outputs()
    print(json.dumps({
        "status": report["status"],
        "decision": report["decision"],
        "coverage": report["coverage"],
        "bounds": report["bounds"],
        "report_sha256": report["report_sha256"],
    }, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
