"""Fail-closed 2,400-locus / two-diameter follower-route sweep.

The sweep joins the canonical terminal-locus stream to the isolated robust
cap-shelf and floating-follower evidence.  It deliberately distinguishes:

* exact positive-volume terminal ownership;
* analytic contact/support constructions;
* straight-chord and minimum-radius length proxies; and
* a physically continuous, swept route (which is not yet available).

No analytic chord or radius construction is promoted to collision, dancer,
transition, assembly, or production authority.
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

LOCI_STUDY_PATH = REPORTS / "aggregate_boundary_follower_locus_study.json"
TERMINAL_LOCI_PATH = REPORTS / "carriage_active_sector_terminal_guide_loci.json"
G0_NORMAL_PATH = REPORTS / "aggregate_boundary_follower_g0_normal_audit.json"
G0_TRADE_PATH = REPORTS / "aggregate_boundary_follower_g0_landing_trade.json"
ARCHITECTURE_PATH = REPORTS / (
    "aggregate_boundary_follower_replacement_architecture.json"
)
CAP_MANIFEST_PATH = REVIEW / "aggregate_boundary_g0_cap_shelf.manifest.json"
CAP_STEP_PATH = REVIEW / "aggregate_boundary_g0_cap_shelf.step"

OUTPUT_JSON = REPORTS / "aggregate_boundary_follower_route_sweep.json"
OUTPUT_MD = REPORTS / "aggregate_boundary_follower_route_sweep.md"

SCHEMA = "aggregate-boundary-follower-route-sweep/v1"
EXPECTED_LOCI = 2400
WIRE_DIAMETERS_MM = (0.2, 0.5)
MINIMUM_CENTERLINE_RADIUS_MM = 3.0
FOLLOWER_NOSE_SURFACE_RADIUS_MM = 3.0
FOLLOWER_GROOVE_CLEAR_WIDTH_MM = 0.65
CAP_LANE_CLEAR_WIDTH_MM = 0.65
INSERTION_GAUGE_RADIUS_MM = 0.36
CAP_REBOUND_RADIUS_MM = 3.5
SUPPORT_TOLERANCE_MM = 1.0e-9
C1_TOLERANCE_DEG = 1.0e-4

# The widened lane is swept about the selected predecessor centreline with a
# 0.25 mm inward cavity extent.  The isolated manifest tests the right shelf
# endpoints explicitly.  The left rebound below remains analytic, not an
# exact-manifest claim.
LEFT_CANONICAL_FLOOR_OFFSET_MM = 0.25

IDENTITIES = (
    {"physical_id": 0, "name": "front_left", "axial_sign": 1,
     "tangent_sign": -1},
    {"physical_id": 1, "name": "front_right", "axial_sign": 1,
     "tangent_sign": 1},
    {"physical_id": 2, "name": "rear_right", "axial_sign": -1,
     "tangent_sign": 1},
    {"physical_id": 3, "name": "rear_left", "axial_sign": -1,
     "tangent_sign": -1},
)
LAWS = (
    "origin_000_direction_+1",
    "origin_000_direction_-1",
    "origin_180_direction_-1",
)

SOURCE_PATHS = (
    Path("sim/aggregate_boundary_follower_route_sweep.py"),
    Path("sim/aggregate_boundary_follower_locus_study.py"),
    Path("sim/aggregate_boundary_follower_g0_normal_audit.py"),
    Path("sim/aggregate_boundary_follower_g0_landing_trade.py"),
    Path("sim/aggregate_boundary_follower_replacement_architecture.py"),
    Path("cad/aggregate_boundary_g0_cap_shelf.py"),
    Path("cad/aggregate_boundary_floating_follower.py"),
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


def _unit(vector: Sequence[float]) -> tuple[float, ...]:
    length = math.sqrt(sum(float(value) ** 2 for value in vector))
    if length <= 1.0e-15:
        raise ValueError("zero-length vector")
    return tuple(float(value) / length for value in vector)


def _distance(one: Sequence[float], two: Sequence[float]) -> float:
    return math.sqrt(sum(
        (float(one[index]) - float(two[index])) ** 2
        for index in range(len(one))
    ))


def _polyline_length(points: Sequence[Sequence[float]]) -> float:
    return sum(
        _distance(points[index - 1], points[index])
        for index in range(1, len(points))
    )


def _sampled_upstream_length(locus: Mapping[str, Any]) -> float:
    """Length the named sampled segments without double-counting joins."""

    return sum(
        _polyline_length(segment.get("machine_world_samples_mm", []))
        for segment in locus.get("segments", [])
    )


def _minimum_two_arc_run(delta_mm: float, radius_mm: float) -> float:
    delta = abs(float(delta_mm))
    radius = float(radius_mm)
    if delta > 4.0 * radius:
        return math.inf
    return 2.0 * math.sqrt(radius * delta - delta * delta / 4.0)


def _two_arc_sweep_deg(delta_mm: float, radius_mm: float) -> float:
    delta = abs(float(delta_mm))
    return math.degrees(math.acos(1.0 - delta / (2.0 * float(radius_mm))))


def _identity(side_sign: int, axial_sign: int) -> dict[str, Any]:
    for row in IDENTITIES:
        if (int(row["tangent_sign"]) == int(side_sign)
                and int(row["axial_sign"]) == int(axial_sign)):
            return dict(row)
    raise ValueError("unbound handed follower identity")


def _law_track_bindings(physical_id: int) -> dict[str, int]:
    if physical_id not in range(4):
        raise ValueError("physical_id must be 0..3")
    return {
        LAWS[0]: physical_id,
        LAWS[1]: physical_id,
        LAWS[2]: (physical_id + 2) % 4,
    }


def _support_candidates(
    point_xy: tuple[float, float],
    triangle: list[tuple[float, float, str]],
) -> list[dict[str, Any]]:
    px, py = point_xy
    result: list[dict[str, Any]] = []
    for qx, qy, vertex in triangle:
        dx, dy = px - qx, py - qy
        length = math.hypot(dx, dy)
        if length <= 1.0e-15:
            continue
        for sign in (-1.0, 1.0):
            nx = sign * (-dy / length)
            ny = sign * (dx / length)
            signed = [
                nx * (x - qx) + ny * (y - qy)
                for x, y, _name in triangle
            ]
            endpoint_side = nx * (px - qx) + ny * (py - qy)
            if max(signed) <= SUPPORT_TOLERANCE_MM:
                result.append({
                    "vertex": vertex,
                    "contact_xy_mm": [qx, qy],
                    "aggregate_outward_normal_xy": [nx, ny],
                    "maximum_triangle_support_residual_mm": max(signed),
                    "endpoint_signed_normal_distance_mm": endpoint_side,
                    "endpoint_lies_on_support_line": (
                        abs(endpoint_side) <= SUPPORT_TOLERANCE_MM
                    ),
                    "aggregate_outward_normal_places_triangle_nonpositive": (
                        max(signed) <= SUPPORT_TOLERANCE_MM
                    ),
                })
    return result


def _right_endpoint_contract(
    manifest: Mapping[str, Any], axial_sign: int, diameter: float,
) -> tuple[list[float], dict[str, Any]]:
    key = f"d{diameter:.1f}"
    end = "front" if axial_sign > 0 else "rear"
    endpoint = list(map(float, manifest["geometry_contract"]
                        ["endpoint_contract"][key]
                        [f"{end}_endpoint_active_local_mm"]))
    diameter_case = next(
        row for row in manifest["validation"]["diameter_cases"]
        if int(row["axial_sign"]) == axial_sign
        and math.isclose(float(row["wire_diameter_mm"]), diameter,
                         abs_tol=1.0e-12)
    )
    return endpoint, diameter_case


def _terminal_contract(
    row: Mapping[str, Any], raw_locus: Mapping[str, Any], diameter: float,
    manifest: Mapping[str, Any], trade: Mapping[str, Any],
) -> dict[str, Any]:
    old = list(map(
        float, raw_locus["terminal_binding"]["cap_endpoint_local_mm"]
    ))
    side = int(row["side_sign"])
    axial = int(row["axial_sign"])
    radius = diameter / 2.0
    if side > 0:
        endpoint, exact = _right_endpoint_contract(
            manifest, axial, diameter,
        )
        selected = next(
            candidate for candidate in trade["candidates"]
            if candidate.get("selected") is True
        )
        diameter_row = next(
            item for item in selected["diameter_endpoint_contract"]
            if math.isclose(float(item["wire_diameter_mm"]), diameter,
                            abs_tol=1.0e-12)
        )
        rebind = [endpoint[index] - old[index] for index in range(3)]
        return {
            "endpoint_active_local_mm": endpoint,
            "terminal_owner": "integral_PEEK_right_cap_side_shelf",
            "terminal_contact_normal_active_local": [0.0, 1.0, 0.0],
            "terminal_C0_class": "EXACT_POSITIVE_BREP",
            "terminal_C0_exact": bool(
                exact["distance_equals_wire_radius"]
                and exact["wire_zero_positive_overlap"]
            ),
            "terminal_C0_analytic": True,
            "R0p36_insertion_gauge_applicable": True,
            "R0p36_insertion_gauge_exact_clear": bool(
                exact["gauge_zero_positive_overlap"]
            ),
            "endpoint_rebind_from_current_mm": rebind,
            "endpoint_rebind_magnitude_mm": _distance(endpoint, old),
            "upstream_predecessor_centerline_reused": False,
            "upstream_rebind_transition_class": (
                "ANALYTIC_R3P5_TWO_ARC__NO_POSITIVE_VOLUME_PATH"
            ),
            "upstream_rebind_minimum_two_arc_run_mm": float(
                diameter_row["minimum_R3p5_two_arc_X_run_mm"]
            ),
            "upstream_rebind_per_arc_sweep_deg": float(
                diameter_row["per_arc_sweep_deg"]
            ),
            "upstream_rebind_C1_exact": False,
        }

    # The left widened-lane floor is analytically at max-wire-radius behind
    # the predecessor centreline.  Unlike the right shelf, this diameter
    # rebound is not one of the manifest's exact endpoint cases.
    contact_x = old[0] - LEFT_CANONICAL_FLOOR_OFFSET_MM
    endpoint = [contact_x + radius, old[1], old[2]]
    delta = endpoint[0] - old[0]
    reused = math.isclose(delta, 0.0, abs_tol=1.0e-12)
    return {
        "endpoint_active_local_mm": endpoint,
        "terminal_owner": "widened_left_PEEK_cap_lane_floor",
        "terminal_contact_normal_active_local": [1.0, 0.0, 0.0],
        "terminal_C0_class": "ANALYTIC_SOURCE_PROFILE_NOT_MANIFEST_BREP",
        "terminal_C0_exact": False,
        "terminal_C0_analytic": True,
        "R0p36_insertion_gauge_applicable": False,
        "R0p36_insertion_gauge_exact_clear": None,
        "endpoint_rebind_from_current_mm": [delta, 0.0, 0.0],
        "endpoint_rebind_magnitude_mm": abs(delta),
        "upstream_predecessor_centerline_reused": reused,
        "upstream_rebind_transition_class": (
            "PREDECESSOR_CENTERLINE_REUSED"
            if reused else
            "ANALYTIC_R3P5_TWO_ARC__NO_POSITIVE_VOLUME_PATH"
        ),
        "upstream_rebind_minimum_two_arc_run_mm": (
            0.0 if reused else _minimum_two_arc_run(delta, CAP_REBOUND_RADIUS_MM)
        ),
        "upstream_rebind_per_arc_sweep_deg": (
            0.0 if reused else _two_arc_sweep_deg(delta, CAP_REBOUND_RADIUS_MM)
        ),
        "upstream_rebind_C1_exact": False,
    }


def _diameter_case(
    row: Mapping[str, Any], raw_locus: Mapping[str, Any], diameter: float,
    manifest: Mapping[str, Any], trade: Mapping[str, Any],
) -> dict[str, Any]:
    terminal = _terminal_contract(row, raw_locus, diameter, manifest, trade)
    endpoint = terminal["endpoint_active_local_mm"]
    turn = int(row["turn_index"])
    wire_radius = diameter / 2.0
    lane_margin = CAP_LANE_CLEAR_WIDTH_MM - diameter
    nose_margin = FOLLOWER_GROOVE_CLEAR_WIDTH_MM - diameter
    centerline_radius = FOLLOWER_NOSE_SURFACE_RADIUS_MM + wire_radius
    upstream_length = _sampled_upstream_length(raw_locus)

    aggregate: dict[str, Any] | None = None
    direct_c1 = False
    chord = None
    provisional = upstream_length
    if turn > 0:
        triangle = [
            (float(point[0]), float(point[1]), name)
            for point, name in zip(
                row["triangle_vertices_uv_mm"], ("A", "B", "C")
            )
        ]
        candidates = _support_candidates(
            (float(endpoint[0]), float(endpoint[1])), triangle,
        )
        if not candidates:
            aggregate = {
                "support_candidate_count": 0,
                "aggregate_C0_analytic": False,
            }
        else:
            qz = int(row["axial_sign"]) * float(
                row["selected_support"]["contact_local_mm"][2]
                / int(row["axial_sign"])
            )
            for candidate in candidates:
                qx, qy = candidate["contact_xy_mm"]
                contact = [qx, qy, qz]
                span = [contact[index] - endpoint[index]
                        for index in range(3)]
                tangent = _unit(span)
                guide = (0.0, 0.0, -float(row["axial_sign"]))
                cosine = max(-1.0, min(1.0, sum(
                    tangent[index] * guide[index] for index in range(3)
                )))
                error = math.degrees(math.acos(cosine))
                candidate.update({
                    "contact_local_mm": contact,
                    "straight_chord_length_mm": math.sqrt(sum(
                        value * value for value in span
                    )),
                    "straight_chord_tangent_local": list(tangent),
                    "guide_terminal_tangent_local": list(guide),
                    "direct_C1_error_deg": error,
                    "nose_centerline_radius_mm": centerline_radius,
                    "minimum_radius_arc_length_mm": (
                        centerline_radius * math.radians(error)
                    ),
                    "minimum_radius_lateral_sweep_mm": (
                        centerline_radius
                        * (1.0 - math.cos(math.radians(error)))
                    ),
                })
            selected = min(candidates, key=lambda item: (
                item["straight_chord_length_mm"], item["vertex"],
            ))
            direct_c1 = selected["direct_C1_error_deg"] <= C1_TOLERANCE_DEG
            chord = float(selected["straight_chord_length_mm"])
            provisional += chord
            aggregate = {
                "support_candidate_count": len(candidates),
                "selected_support": selected,
                "aggregate_C0_analytic": True,
                "aggregate_support_normal_sign_valid": bool(
                    selected[
                        "aggregate_outward_normal_places_triangle_nonpositive"
                    ]
                ),
            }

    complete_c0 = bool(
        terminal["terminal_C0_analytic"]
        and (turn == 0 or aggregate and aggregate["aggregate_C0_analytic"])
    )
    physically_continuous = bool(
        terminal["terminal_C0_exact"]
        and terminal["upstream_rebind_C1_exact"]
        and (turn == 0 or direct_c1)
    )
    result = {
        "wire_diameter_mm": diameter,
        "wire_radius_mm": wire_radius,
        "cap_lane_diametral_margin_mm": lane_margin,
        "follower_nose_groove_diametral_margin_mm": nose_margin,
        "wire_fits_0p65_cap_lane": lane_margin >= -1.0e-12,
        "wire_fits_0p65_follower_groove": nose_margin >= -1.0e-12,
        "follower_nose_centerline_radius_mm": centerline_radius,
        "minimum_centerline_radius_requirement_mm": (
            MINIMUM_CENTERLINE_RADIUS_MM
        ),
        "minimum_centerline_radius_analytic_pass": (
            centerline_radius >= MINIMUM_CENTERLINE_RADIUS_MM
        ),
        **terminal,
        "aggregate_contact": aggregate,
        "complete_endpoint_C0_analytic": complete_c0,
        "direct_cap_to_aggregate_C1": direct_c1,
        "positive_volume_locus_arc_placed": False,
        "sampled_upstream_polyline_length_mm": upstream_length,
        "selected_support_chord_length_mm": chord,
        "provisional_path_without_C1_arc_mm": provisional,
        "provisional_length_is_dancer_authority": False,
        "physical_continuous_route_authorized": physically_continuous,
    }
    return result


def _length_proxy_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for diameter in WIRE_DIAMETERS_MM:
        timed_cases = [
            (float(row["time_s"]), case)
            for row in rows for case in row["diameter_cases"]
            if math.isclose(float(case["wire_diameter_mm"]), diameter,
                            abs_tol=1.0e-12)
        ]
        cases = [case for _time, case in timed_cases]
        lengths = [float(case["provisional_path_without_C1_arc_mm"])
                   for case in cases]
        consecutive = [abs(lengths[index] - lengths[index - 1])
                       for index in range(1, len(lengths))]
        consecutive_rates = [
            abs(lengths[index] - lengths[index - 1])
            / (timed_cases[index][0] - timed_cases[index - 1][0])
            for index in range(1, len(lengths))
            if timed_cases[index][0] > timed_cases[index - 1][0]
        ]
        per_identity: dict[str, Any] = {}
        for identity in IDENTITIES:
            identity_rows = [
                row for row in rows
                if int(row["identity"]["physical_id"])
                == int(identity["physical_id"])
            ]
            identity_lengths = [
                next(case["provisional_path_without_C1_arc_mm"]
                     for case in row["diameter_cases"]
                     if math.isclose(float(case["wire_diameter_mm"]), diameter,
                                     abs_tol=1.0e-12))
                for row in identity_rows
            ]
            per_identity[str(identity["physical_id"])] = {
                "name": identity["name"],
                "case_count": len(identity_lengths),
                "minimum_proxy_length_mm": min(identity_lengths),
                "maximum_proxy_length_mm": max(identity_lengths),
            }
        result[f"d{diameter:.1f}"] = {
            "case_count": len(cases),
            "minimum_proxy_length_mm": min(lengths),
            "maximum_proxy_length_mm": max(lengths),
            "maximum_consecutive_locus_proxy_delta_mm": max(consecutive),
            "maximum_consecutive_locus_proxy_rate_mm_s": max(
                consecutive_rates
            ),
            "per_identity": per_identity,
        }
    return result


def analyze() -> dict[str, Any]:
    loci_study = _load(LOCI_STUDY_PATH)
    terminal_loci = _load(TERMINAL_LOCI_PATH)
    g0_normal = _load(G0_NORMAL_PATH)
    trade = _load(G0_TRADE_PATH)
    architecture = _load(ARCHITECTURE_PATH)
    manifest = _load(CAP_MANIFEST_PATH)

    study_rows = loci_study.get("loci", [])
    raw_rows = terminal_loci.get("loci", [])
    if len(study_rows) != EXPECTED_LOCI or len(raw_rows) != EXPECTED_LOCI:
        raise ValueError("route sweep requires matching 2,400-locus inputs")

    rows: list[dict[str, Any]] = []
    for row, raw in zip(study_rows, raw_rows):
        if int(row["locus_index"]) != int(raw["locus_index"]):
            raise ValueError("locus-study/raw-locus index drift")
        identity = _identity(int(row["side_sign"]), int(row["axial_sign"]))
        rows.append({
            "locus_index": int(row["locus_index"]),
            "pass_index": int(row["pass_index"]),
            "state_index": int(row["state_index"]),
            "turn_index": int(row["turn_index"]),
            "half_turn_index": int(row["half_turn_index"]),
            "tooth_index": int(row["tooth_index"]),
            "time_s": float(row["time_s"]),
            "lane_id": str(row["lane_id"]),
            "side_sign": int(row["side_sign"]),
            "axial_sign": int(row["axial_sign"]),
            "identity": identity,
            "law_track_bindings": _law_track_bindings(
                int(identity["physical_id"])
            ),
            "diameter_cases": [
                _diameter_case(row, raw, diameter, manifest, trade)
                for diameter in WIRE_DIAMETERS_MM
            ],
        })

    cases = [case for row in rows for case in row["diameter_cases"]]
    nonzero = [
        case for row in rows if row["turn_index"] > 0
        for case in row["diameter_cases"]
    ]
    g0_cases = [
        case for row in rows if row["turn_index"] == 0
        for case in row["diameter_cases"]
    ]
    right_cases = [
        case for row in rows if row["side_sign"] > 0
        for case in row["diameter_cases"]
    ]
    selected_supports = [
        case["aggregate_contact"]["selected_support"]
        for case in nonzero
        if case["aggregate_contact"]
        and case["aggregate_contact"].get("selected_support")
    ]

    analytic_gates = {
        "exact_2400_loci_bound": len(rows) == EXPECTED_LOCI,
        "two_diameter_cases_per_locus": len(cases) == 2 * EXPECTED_LOCI,
        "four_handed_identities_exercised": (
            {row["identity"]["physical_id"] for row in rows}
            == set(range(4))
        ),
        "three_law_track_bindings_per_locus": all(
            set(row["law_track_bindings"]) == set(LAWS) for row in rows
        ),
        "all_cases_fit_0p65_cap_lane_and_follower_groove": all(
            case["wire_fits_0p65_cap_lane"]
            and case["wire_fits_0p65_follower_groove"]
            for case in cases
        ),
        "all_right_terminal_cases_have_exact_PEEK_C0": all(
            case["terminal_C0_exact"] for case in right_cases
        ),
        "all_right_terminal_R0p36_gauges_clear": all(
            case["R0p36_insertion_gauge_exact_clear"]
            for case in right_cases
        ),
        "all_nonzero_cases_have_analytic_aggregate_C0": (
            len(selected_supports) == len(nonzero)
        ),
        "all_selected_aggregate_normals_are_outward_supports": all(
            support[
                "aggregate_outward_normal_places_triangle_nonpositive"
            ]
            for support in selected_supports
        ),
        "all_nose_centerline_radii_at_least_R3": all(
            case["minimum_centerline_radius_analytic_pass"] for case in cases
        ),
    }
    physical_gates = {
        "left_diameter_endpoints_exact_BREP_validated": False,
        "all_upstream_diameter_rebinds_positive_volume_C1": False,
        "positive_volume_follower_arc_placed_at_all_nonzero_loci": False,
        "direct_span_C1_at_all_nonzero_loci": all(
            case["direct_cap_to_aggregate_C1"] for case in nonzero
        ),
        "continuous_intra_locus_transition_sweep_proven": False,
        "follower_force_resultant_compressive_at_all_loci": False,
        "assembly_collision_and_tolerance_sweep_proven": False,
        "exact_route_length_history_proven": False,
        "dancer_feed_length_coupling_proven": False,
        "dynamic_tension_friction_wear_and_settling_proven": False,
    }

    artifacts = {
        "locus_study": {
            "path": str(LOCI_STUDY_PATH.relative_to(ROOT)).replace("\\", "/"),
            "sha256": _sha256(LOCI_STUDY_PATH),
            "report_sha256": loci_study.get("report_sha256"),
        },
        "terminal_loci": {
            "path": str(TERMINAL_LOCI_PATH.relative_to(ROOT)).replace("\\", "/"),
            "sha256": _sha256(TERMINAL_LOCI_PATH),
            "payload_sha256": terminal_loci.get("locus_payload_sha256"),
        },
        "g0_normal_audit": {
            "path": str(G0_NORMAL_PATH.relative_to(ROOT)).replace("\\", "/"),
            "sha256": _sha256(G0_NORMAL_PATH),
            "report_sha256": g0_normal.get("report_sha256"),
        },
        "g0_landing_trade": {
            "path": str(G0_TRADE_PATH.relative_to(ROOT)).replace("\\", "/"),
            "sha256": _sha256(G0_TRADE_PATH),
            "report_sha256": trade.get("report_sha256"),
        },
        "replacement_architecture": {
            "path": str(ARCHITECTURE_PATH.relative_to(ROOT)).replace("\\", "/"),
            "sha256": _sha256(ARCHITECTURE_PATH),
            "report_sha256": architecture.get("report_sha256"),
        },
        "cap_shelf_manifest": {
            "path": str(CAP_MANIFEST_PATH.relative_to(ROOT)).replace("\\", "/"),
            "sha256": _sha256(CAP_MANIFEST_PATH),
            "manifest_sha256": manifest.get("manifest_sha256"),
        },
        "cap_shelf_STEP": {
            "path": str(CAP_STEP_PATH.relative_to(ROOT)).replace("\\", "/"),
            "sha256": _sha256(CAP_STEP_PATH),
        },
    }
    blockers = [name for name, passed in physical_gates.items() if not passed]
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "FAIL",
        "decision": (
            "ALL_2400_LOCI_X_TWO_DIAMETERS_CLASSIFIED__"
            "CONTINUOUS_PHYSICAL_ROUTE_NOT_PROVEN"
        ),
        "production_authorized": False,
        "wire_route_authorized": False,
        "collision_authorized": False,
        "assembly_integration_authorized": False,
        "dancer_coupling_authorized": False,
        "scope": {
            "proved": [
                "2,400 locus identity and three-law track binding",
                "0.2/0.5 mm lane and follower-groove diameter fit",
                "exact right-shelf terminal C0 and R0.36 gauge clearance",
                "analytic left terminal rebound and nonzero aggregate support C0",
                "aggregate support-normal sign and R3-or-greater nose radius",
                "sampled-upstream plus straight-chord length proxy envelope",
            ],
            "not_proved": [
                "left diameter endpoint exact BREP ownership",
                "positive-volume C1 rebound and per-locus follower arc",
                "continuous transition, collision, tolerance, load, or wear",
                "exact route-length history or spool/dancer dynamic coupling",
            ],
        },
        "diameter_and_contact_contract": {
            "wire_diameters_mm": list(WIRE_DIAMETERS_MM),
            "cap_lane_clear_width_mm": CAP_LANE_CLEAR_WIDTH_MM,
            "follower_groove_clear_width_mm": FOLLOWER_GROOVE_CLEAR_WIDTH_MM,
            "follower_nose_surface_radius_mm": FOLLOWER_NOSE_SURFACE_RADIUS_MM,
            "wire_centerline_radius_by_diameter_mm": {
                f"d{diameter:.1f}": FOLLOWER_NOSE_SURFACE_RADIUS_MM + diameter / 2.0
                for diameter in WIRE_DIAMETERS_MM
            },
            "minimum_required_centerline_radius_mm": (
                MINIMUM_CENTERLINE_RADIUS_MM
            ),
            "right_insertion_gauge_radius_mm": INSERTION_GAUGE_RADIUS_MM,
        },
        "identity_and_law_contract": {
            "identities": list(IDENTITIES),
            "laws": list(LAWS),
            "reverse_180_track_to_physical_id": [2, 3, 0, 1],
            "binding_rule": (
                "direct/reverse-zero track=physical_id; "
                "reverse-180 track=(physical_id+2)%4"
            ),
        },
        "coverage": {
            "required_loci": EXPECTED_LOCI,
            "evaluated_loci": len(rows),
            "diameter_route_case_count": len(cases),
            "g0_case_count": len(g0_cases),
            "nonzero_growth_case_count": len(nonzero),
            "exact_right_terminal_C0_case_count": sum(
                case["terminal_C0_exact"] for case in right_cases
            ),
            "analytic_left_terminal_C0_case_count": sum(
                case["terminal_C0_analytic"] and not case["terminal_C0_exact"]
                for row in rows if row["side_sign"] < 0
                for case in row["diameter_cases"]
            ),
            "nonzero_analytic_aggregate_C0_case_count": len(selected_supports),
            "nonzero_direct_C1_case_count": sum(
                case["direct_cap_to_aggregate_C1"] for case in nonzero
            ),
            "positive_volume_locus_arc_case_count": sum(
                case["positive_volume_locus_arc_placed"] for case in nonzero
            ),
            "physically_authorized_route_case_count": sum(
                case["physical_continuous_route_authorized"] for case in cases
            ),
        },
        "turn_and_length_bounds": {
            "minimum_direct_C1_error_deg": min(
                support["direct_C1_error_deg"] for support in selected_supports
            ),
            "maximum_direct_C1_error_deg": max(
                support["direct_C1_error_deg"] for support in selected_supports
            ),
            "maximum_minimum_radius_arc_length_mm": max(
                support["minimum_radius_arc_length_mm"]
                for support in selected_supports
            ),
            "maximum_minimum_radius_lateral_sweep_mm": max(
                support["minimum_radius_lateral_sweep_mm"]
                for support in selected_supports
            ),
            "length_proxy": _length_proxy_summary(rows),
            "length_proxy_definition": (
                "sampled canonical upstream polyline plus selected analytic "
                "support chord; excludes all missing rebound/C1 arc length"
            ),
            "exact_route_length_available": False,
            "dancer_coupling_available": False,
        },
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
        raise ValueError("unsupported follower-route sweep schema")
    if report.get("report_sha256") != _canonical_hash(
            report, "report_sha256"):
        raise ValueError("follower-route sweep report hash mismatch")
    for relative, expected in report.get("source_hashes", {}).items():
        path = ROOT / str(relative).replace("/", "\\")
        if not path.is_file() or _sha256(path) != expected:
            raise ValueError(f"stale follower-route source {relative}")
    for name, artifact in report.get("artifacts", {}).items():
        path = ROOT / str(artifact["path"]).replace("/", "\\")
        if not path.is_file() or _sha256(path) != artifact.get("sha256"):
            raise ValueError(f"stale follower-route artifact {name}")
    if report.get("status") != "FAIL":
        raise ValueError("route sweep must remain fail-closed")
    for key in (
        "production_authorized", "wire_route_authorized",
        "collision_authorized", "assembly_integration_authorized",
        "dancer_coupling_authorized",
    ):
        if report.get(key) is not False:
            raise ValueError(f"route sweep cannot authorize {key}")
    coverage = report.get("coverage", {})
    if coverage.get("evaluated_loci") != EXPECTED_LOCI:
        raise ValueError("route sweep locus coverage mismatch")
    if coverage.get("diameter_route_case_count") != 2 * EXPECTED_LOCI:
        raise ValueError("route sweep diameter coverage mismatch")
    if coverage.get("physically_authorized_route_case_count") != 0:
        raise ValueError("route sweep promoted an unproved physical route")


def render_markdown(report: Mapping[str, Any]) -> str:
    coverage = report["coverage"]
    bounds = report["turn_and_length_bounds"]
    lines = [
        "# Aggregate-boundary follower route sweep",
        "",
        f"- Status: **{report['status']}**",
        f"- Decision: `{report['decision']}`",
        f"- Loci: {coverage['evaluated_loci']} / {coverage['required_loci']}",
        f"- Diameter route cases: {coverage['diameter_route_case_count']}",
        (
            "- Exact right-shelf terminal C0 cases: "
            f"{coverage['exact_right_terminal_C0_case_count']}"
        ),
        (
            "- Analytic left terminal C0 cases: "
            f"{coverage['analytic_left_terminal_C0_case_count']}"
        ),
        (
            "- Nonzero aggregate-support C0 cases: "
            f"{coverage['nonzero_analytic_aggregate_C0_case_count']}"
        ),
        (
            "- Direct C1 / placed positive-volume arc / physically authorized: "
            f"{coverage['nonzero_direct_C1_case_count']} / "
            f"{coverage['positive_volume_locus_arc_case_count']} / "
            f"{coverage['physically_authorized_route_case_count']}"
        ),
        (
            "- Direct C1 error range: "
            f"{bounds['minimum_direct_C1_error_deg']:.6f} to "
            f"{bounds['maximum_direct_C1_error_deg']:.6f} deg"
        ),
        (
            "- Maximum minimum-radius arc demand: "
            f"{bounds['maximum_minimum_radius_arc_length_mm']:.6f} mm"
        ),
        "",
        "## What the numeric length means",
        "",
        bounds["length_proxy_definition"],
        "It is not an exact route length and grants no dancer authority.",
        "",
        "## Blocking physical gates",
        "",
    ]
    lines.extend(f"- `{name}`" for name in report["blockers"])
    lines.extend([
        "",
        "The isolated cap and follower remain review-only; no selected release "
        "or assembly was modified.",
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
        "report_sha256": report["report_sha256"],
    }, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
