"""Fail-closed robust trade for the aggregate follower g=0 landing.

The trade rebuilds bounded front/rear PEEK shelf and widened-mouth witnesses
without modifying cap CAD.  It compares that topology with an installed
Nomex sheet and a shifted seam cut.  Geometry can select a successor here;
route, force, collision, tolerance, release, and production cannot.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable, Mapping

from build123d import Align, Box, Cylinder, Pos, Rot, Vertex


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

import carriage_active_sector_terminal_guide as guide
import permanent_cap_production_review as cap
import stator_insulation_nomex410 as nomex


SCHEMA = "aggregate-boundary-follower-g0-landing-trade/v1"
OUTPUT_JSON = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_g0_landing_trade.json"
)
OUTPUT_MD = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_g0_landing_trade.md"
)
G0_AUDIT_PATH = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_g0_normal_audit.json"
)
LOCI_PATH = ROOT / "out" / "reports" / (
    "carriage_active_sector_terminal_guide_loci.json"
)
CAP_STEP_PATH = ROOT / "out" / "review" / (
    "permanent_cap_production_review.step"
)
INTEGRATED_STEP_PATH = ROOT / "out" / "review" / (
    "integrated_release_candidate.step"
)

SOURCE_PATHS = (
    Path("cad/permanent_cap_production_review.py"),
    Path("cad/carriage_active_sector_terminal_guide.py"),
    Path("cad/stator_insulation_nomex410.py"),
    Path("sim/aggregate_boundary_follower_g0_normal_audit.py"),
)

WIRE_DIAMETERS_MM = (0.2, 0.5)
CONTACT_RADIUS_MM = guide.LEADIN_CENTERLINE_RADIUS_MM
INSERTION_GAUGE_RADIUS_MM = 0.36

PEEK_SHELF_RADIAL_LENGTH_MM = 1.50
PEEK_SHELF_AXIAL_WIDTH_MM = 0.75
PEEK_SHELF_STOCK_MM = 0.30
MOUTH_RADIAL_LENGTH_MM = 2.40
MOUTH_TANGENTIAL_WIDTH_MM = 1.00
MOUTH_AXIAL_SPAN_MM = 0.90
PEEK_DENSITY_G_MM3 = guide.PEEK_DENSITY_G_MM3

NOMEX_THICKNESS_MM = nomex.MATERIAL_NOMINAL_THICKNESS_MM
NOMEX_RADIAL_LENGTH_MM = 2.00
NOMEX_AXIAL_WIDTH_MM = 0.75
CUT_SHIFT_MM = 0.30


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(value: Mapping[str, Any]) -> str:
    body = deepcopy(dict(value))
    body.pop("report_sha256", None)
    return hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def _common_volume(one, two) -> float:
    common = one & two
    return 0.0 if common is None else float(common.volume)


def _minimum_two_arc_run(delta_mm: float, radius_mm: float) -> float:
    delta = abs(float(delta_mm))
    radius = float(radius_mm)
    if delta > 4.0 * radius:
        return math.inf
    return 2.0 * math.sqrt(radius * delta - delta * delta / 4.0)


def _two_arc_sweep_deg(delta_mm: float, radius_mm: float) -> float:
    delta = abs(float(delta_mm))
    return math.degrees(math.acos(1.0 - delta / (2.0 * radius_mm)))


def _x_cylinder(
    radius_mm: float, length_mm: float, origin: tuple[float, float, float],
):
    return (
        Pos(*origin)
        * Rot(0.0, 90.0, 0.0)
        * Cylinder(
            radius_mm, length_mm,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
    )


def _selected_contact_datum() -> dict[str, Any]:
    endpoint = tuple(map(float, cap._lane_points()["waypoint"]))
    selected = guide.cap_with_short_leadins(1)
    point, _ = selected.closest_points(Vertex(endpoint))
    surface = tuple(float(value) for value in point)
    return {
        "canonical_right_endpoint_active_local_mm": list(endpoint),
        "selected_cap_contact_surface_active_local_mm": list(surface),
        "contact_surface_y_mm": surface[1],
        "current_endpoint_surface_distance_mm": float(
            selected.distance_to(Vertex(endpoint))
        ),
    }


def _endpoint_rows(surface_y_mm: float, *, sheet_mm: float = 0.0) \
        -> list[dict[str, Any]]:
    endpoint = cap._lane_points()["waypoint"]
    contact_y = float(surface_y_mm) + float(sheet_mm)
    result = []
    for diameter in WIRE_DIAMETERS_MM:
        rebound_y = contact_y + diameter / 2.0
        offset = rebound_y - float(endpoint[1])
        result.append({
            "wire_diameter_mm": diameter,
            "wire_radius_mm": diameter / 2.0,
            "endpoint_active_local_mm": [
                float(endpoint[0]), rebound_y, float(endpoint[2])
            ],
            "tangential_rebind_from_current_endpoint_mm": offset,
            "minimum_R3p5_two_arc_X_run_mm": _minimum_two_arc_run(
                offset, CONTACT_RADIUS_MM
            ),
            "per_arc_sweep_deg": _two_arc_sweep_deg(
                offset, CONTACT_RADIUS_MM
            ),
        })
    return result


def _modified_peek_cases(
    surface_y_mm: float, endpoint_rows: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], dict[str, float]]:
    canonical = cap._lane_points()["waypoint"]
    x, _old_y, z_abs = map(float, canonical)
    rebound_ys = [row["endpoint_active_local_mm"][1]
                  for row in endpoint_rows]
    mouth_center_y = (min(rebound_ys) + max(rebound_ys)) / 2.0
    shelf_box_volume = (
        PEEK_SHELF_RADIAL_LENGTH_MM
        * PEEK_SHELF_STOCK_MM
        * PEEK_SHELF_AXIAL_WIDTH_MM
    )
    cases = []
    cap_overlaps = []
    for sign in (1, -1):
        z = sign * z_abs
        before = guide._cap_with_short_leadins_before_right_seam_mouth(sign)
        mouth = Pos(x, mouth_center_y, z) * Box(
            MOUTH_RADIAL_LENGTH_MM,
            MOUTH_TANGENTIAL_WIDTH_MM,
            MOUTH_AXIAL_SPAN_MM,
            align=(Align.MIN, Align.CENTER, Align.CENTER),
        )
        shelf = Pos(
            x - PEEK_SHELF_RADIAL_LENGTH_MM / 2.0,
            surface_y_mm - PEEK_SHELF_STOCK_MM / 2.0,
            z,
        ) * Box(
            PEEK_SHELF_RADIAL_LENGTH_MM,
            PEEK_SHELF_STOCK_MM,
            PEEK_SHELF_AXIAL_WIDTH_MM,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
        shelf_overlap = _common_volume(shelf, before)
        modified = before.cut(mouth).fuse(shelf)
        diameter_rows = []
        for endpoint_row in endpoint_rows:
            diameter = endpoint_row["wire_diameter_mm"]
            radius = diameter / 2.0
            rebound_y = endpoint_row["endpoint_active_local_mm"][1]
            endpoint = (x, rebound_y, z)
            gauge = _x_cylinder(
                INSERTION_GAUGE_RADIUS_MM,
                MOUTH_RADIAL_LENGTH_MM,
                endpoint,
            )
            cap_side_wire = _x_cylinder(
                radius,
                PEEK_SHELF_RADIAL_LENGTH_MM,
                (x - PEEK_SHELF_RADIAL_LENGTH_MM, rebound_y, z),
            )
            distance = float(modified.distance_to(Vertex(endpoint)))
            wire_overlap = _common_volume(shelf, cap_side_wire)
            gauge_overlap = _common_volume(modified, gauge)
            diameter_rows.append({
                "wire_diameter_mm": diameter,
                "endpoint_active_local_mm": list(endpoint),
                "endpoint_to_modified_cap_distance_mm": distance,
                "expected_wire_radius_mm": radius,
                "endpoint_distance_equals_wire_radius": math.isclose(
                    distance, radius, rel_tol=0.0, abs_tol=1.0e-9
                ),
                "shelf_to_cap_side_wire_positive_overlap_mm3": wire_overlap,
                "shelf_to_cap_side_wire_distance_mm": float(
                    shelf.distance_to(cap_side_wire)
                ),
                "modified_cap_to_R0p36_gauge_positive_overlap_mm3": (
                    gauge_overlap
                ),
                "wire_tangent_without_positive_overlap": (
                    wire_overlap <= 1.0e-8
                    and float(shelf.distance_to(cap_side_wire)) <= 1.0e-8
                ),
                "R0p36_gauge_clear": gauge_overlap <= 1.0e-8,
            })
        case = {
            "axial_end": "front" if sign > 0 else "rear",
            "axial_sign": sign,
            "modified_cap_solid_count": len(modified.solids()),
            "shelf_to_before_cap_positive_overlap_mm3": shelf_overlap,
            "shelf_positive_fusion": shelf_overlap > 1.0e-8,
            "diameter_cases": diameter_rows,
        }
        case["status"] = "PASS" if (
            case["modified_cap_solid_count"] == 1
            and case["shelf_positive_fusion"]
            and all(
                row["endpoint_distance_equals_wire_radius"]
                and row["wire_tangent_without_positive_overlap"]
                and row["R0p36_gauge_clear"]
                for row in diameter_rows
            )
        ) else "FAIL"
        cap_overlaps.append(shelf_overlap)
        cases.append(case)
    balance = {
        "shelf_box_volume_each_mm3": shelf_box_volume,
        "maximum_net_new_volume_each_before_mouth_accounting_mm3": (
            shelf_box_volume - min(cap_overlaps)
        ),
        "physical_shelf_count_if_24fold_front_and_rear": 48,
        "maximum_total_added_PEEK_volume_before_mouth_accounting_mm3": (
            48.0 * (shelf_box_volume - min(cap_overlaps))
        ),
        "maximum_total_added_PEEK_mass_g_before_mouth_accounting": (
            48.0 * (shelf_box_volume - min(cap_overlaps))
            * PEEK_DENSITY_G_MM3
        ),
        "first_moment_cancels_by_24fold_front_rear_symmetry": True,
        "polar_inertia_recalculation_required": True,
        "mouth_removal_mass_not_netted": True,
    }
    return cases, balance


def _shifted_cut_witness() -> list[dict[str, Any]]:
    x, y, z_abs = map(float, cap._lane_points()["waypoint"])
    rows = []
    for sign in (1, -1):
        z = sign * z_abs
        before = guide._cap_with_short_leadins_before_right_seam_mouth(sign)
        shifted_length = (
            guide.RIGHT_SEAM_MOUTH_RADIAL_LENGTH_MM - CUT_SHIFT_MM
        )
        shifted_cut = Pos(x + CUT_SHIFT_MM, y, z) * Box(
            shifted_length,
            guide.RIGHT_SEAM_MOUTH_TANGENTIAL_WIDTH_MM,
            guide.RIGHT_SEAM_MOUTH_AXIAL_SPAN_MM,
            align=(Align.MIN, Align.CENTER, Align.CENTER),
        )
        candidate = before.cut(shifted_cut)
        gauge = _x_cylinder(
            INSERTION_GAUGE_RADIUS_MM,
            guide.RIGHT_SEAM_MOUTH_RADIAL_LENGTH_MM,
            (x, y, z),
        )
        overlap = _common_volume(candidate, gauge)
        rows.append({
            "axial_end": "front" if sign > 0 else "rear",
            "axial_sign": sign,
            "cut_start_shift_plus_X_mm": CUT_SHIFT_MM,
            "candidate_solid_count": len(candidate.solids()),
            "original_endpoint_to_candidate_surface_mm": float(
                candidate.distance_to(Vertex((x, y, z)))
            ),
            "candidate_to_R0p36_gauge_positive_overlap_mm3": overlap,
            "status": "FAIL" if overlap > 1.0e-8 else "PASS",
        })
    return rows


def analyze() -> dict[str, Any]:
    g0_audit = _load(G0_AUDIT_PATH)
    if g0_audit.get("report_sha256") != _canonical_hash(g0_audit):
        raise ValueError("bound g0 normal audit self-hash mismatch")
    datum = _selected_contact_datum()
    surface_y = datum["contact_surface_y_mm"]
    peek_endpoints = _endpoint_rows(surface_y)
    peek_cases, balance = _modified_peek_cases(surface_y, peek_endpoints)
    nomex_endpoints = _endpoint_rows(
        surface_y, sheet_mm=NOMEX_THICKNESS_MM
    )
    shifted = _shifted_cut_witness()

    mouth_center_y = (
        peek_endpoints[0]["endpoint_active_local_mm"][1]
        + peek_endpoints[1]["endpoint_active_local_mm"][1]
    ) / 2.0
    peek_pass = all(row["status"] == "PASS" for row in peek_cases)
    shifted_pass = all(row["status"] == "PASS" for row in shifted)
    cap_lane_width_blocked = cap.GROOVE_CLEAR_WIDTH_MM < max(
        WIRE_DIAMETERS_MM
    )

    candidates = [
        {
            "id": "diameter_rebound_integral_PEEK_cap_side_shelf",
            "status": "SELECTED_FOR_DETAILED_REDESIGN" if peek_pass else "FAIL",
            "selected": peek_pass,
            "contact_surface_y_mm": surface_y,
            "contact_normal_active_local": [0.0, 1.0, 0.0],
            "shelf_dimensions_mm": {
                "radial_cap_side_length": PEEK_SHELF_RADIAL_LENGTH_MM,
                "axial_width": PEEK_SHELF_AXIAL_WIDTH_MM,
                "minimum_stock_behind_contact_face": PEEK_SHELF_STOCK_MM,
                "ends_at_seam_X": datum[
                    "canonical_right_endpoint_active_local_mm"
                ][0],
            },
            "mouth_dimensions_mm": {
                "radial_length": MOUTH_RADIAL_LENGTH_MM,
                "tangential_width": MOUTH_TANGENTIAL_WIDTH_MM,
                "axial_span": MOUTH_AXIAL_SPAN_MM,
                "center_y": mouth_center_y,
            },
            "diameter_endpoint_contract": peek_endpoints,
            "exact_modified_BREP_cases": peek_cases,
            "controlling_open_items": [
                "full cap lane width for 0.5 mm wire",
                "force-resultant compression sign",
                "full route/collision/tolerance/wear/endurance",
            ],
        },
        {
            "id": "installed_0p127mm_Nomex_BREP_with_route_rebind",
            "status": "REJECTED_NOT_SMALLEST_ROBUST_TOPOLOGY",
            "selected": False,
            "sheet_thickness_mm": NOMEX_THICKNESS_MM,
            "required_radial_length_mm": NOMEX_RADIAL_LENGTH_MM,
            "required_axial_width_mm": NOMEX_AXIAL_WIDTH_MM,
            "diameter_endpoint_contract": nomex_endpoints,
            "reasons": [
                "requires a longer maximum R3.5 transition than integral PEEK",
                "no selected pocket, clamp, adhesive, or installed 3D BREP",
                "received thickness changes endpoint and contact preload",
                "exposed paper edge snag, polish, wear, varnish, and life are open",
            ],
        },
        {
            "id": "right_seam_mouth_cut_shift_plus_0p30mm",
            "status": "REJECTED_R0P36_INSERTION_COLLISION",
            "selected": False,
            "exact_cases": shifted,
            "all_cases_pass": shifted_pass,
            "reason": (
                "restores the old floor but retains positive volume inside "
                "the required R0.36 insertion cylinder"
            ),
        },
    ]

    geometry_gates = {
        "PEEK_front_and_rear_modified_BREP_one_solid": all(
            row["modified_cap_solid_count"] == 1 for row in peek_cases
        ),
        "PEEK_0p2_and_0p5_endpoint_distance_equals_radius": all(
            case["endpoint_distance_equals_wire_radius"]
            for row in peek_cases for case in row["diameter_cases"]
        ),
        "PEEK_0p2_and_0p5_shelf_wire_tangent_no_overlap": all(
            case["wire_tangent_without_positive_overlap"]
            for row in peek_cases for case in row["diameter_cases"]
        ),
        "PEEK_0p2_and_0p5_R0p36_gauge_clear": all(
            case["R0p36_gauge_clear"]
            for row in peek_cases for case in row["diameter_cases"]
        ),
        "PEEK_minimum_section_ge_0p30mm": PEEK_SHELF_STOCK_MM >= 0.30,
        "PEEK_1p50mm_transition_covers_both_R3p5_runs": max(
            row["minimum_R3p5_two_arc_X_run_mm"]
            for row in peek_endpoints
        ) <= PEEK_SHELF_RADIAL_LENGTH_MM,
        "24fold_front_rear_first_moment_balance": balance[
            "first_moment_cancels_by_24fold_front_rear_symmetry"
        ],
    }
    release_gates = {
        "selected_PEEK_topology_integrated_into_cap_CAD": False,
        "complete_cap_lane_width_ge_0p65mm": False,
        "0p5mm_wire_clears_current_0p47752mm_cap_lane": not cap_lane_width_blocked,
        "all_48_force_resultants_compressive_into_owner_normal": False,
        "all_2400_routes_rebound_and_revalidated": False,
        "full_raw_rigid_copper_self_collision_revalidated": False,
        "cap_mass_inertia_and_retention_loads_revalidated": False,
        "received_tolerance_polish_enamel_wear_endurance_qualified": False,
    }

    source_hashes = {
        str(path).replace("\\", "/"): _sha256(ROOT / path)
        for path in SOURCE_PATHS
    }
    artifacts = {
        "g0_normal_audit": {
            "path": "out/reports/aggregate_boundary_follower_g0_normal_audit.json",
            "byte_count": G0_AUDIT_PATH.stat().st_size,
            "sha256": _sha256(G0_AUDIT_PATH),
            "report_sha256": g0_audit["report_sha256"],
        },
        "terminal_loci": {
            "path": "out/reports/carriage_active_sector_terminal_guide_loci.json",
            "byte_count": LOCI_PATH.stat().st_size,
            "sha256": _sha256(LOCI_PATH),
        },
        "permanent_cap_STEP": {
            "path": "out/review/permanent_cap_production_review.step",
            "byte_count": CAP_STEP_PATH.stat().st_size,
            "sha256": _sha256(CAP_STEP_PATH),
        },
        "integrated_release_candidate_STEP": {
            "path": "out/review/integrated_release_candidate.step",
            "byte_count": INTEGRATED_STEP_PATH.stat().st_size,
            "sha256": _sha256(INTEGRATED_STEP_PATH),
        },
    }

    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "FAIL",
        "decision": (
            "PEEK_SHELF_TOPOLOGY_SELECTED_FOR_REDESIGN__RANGE_ROUTE_AND_RELEASE_OPEN"
        ),
        "selected_candidate_id": (
            "diameter_rebound_integral_PEEK_cap_side_shelf"
        ),
        "production_authorized": False,
        "wire_route_authorized": False,
        "collision_authorized": False,
        "assembly_integration_authorized": False,
        "selected_release_modified": False,
        "inputs": {
            "wire_diameter_range_mm": list(WIRE_DIAMETERS_MM),
            "minimum_contact_radius_mm": CONTACT_RADIUS_MM,
            "insertion_gauge_radius_mm": INSERTION_GAUGE_RADIUS_MM,
            "current_cap_lane_clear_width_mm": cap.GROOVE_CLEAR_WIDTH_MM,
            "required_range_cap_lane_clear_width_mm": 0.65,
            "current_cap_lane_is_narrower_than_0p5mm_wire": (
                cap_lane_width_blocked
            ),
        },
        "selected_contact_datum": datum,
        "candidates": candidates,
        "mass_and_balance_estimate": balance,
        "geometry_gates": geometry_gates,
        "release_gates": release_gates,
        "force_normal_gate": {
            "selected_contact_normal_active_local": [0.0, 1.0, 0.0],
            "physical_normal_mapping": "Rz(tooth_index*15deg)",
            "surface_normal_geometry_available": True,
            "incident_tension_resultant_compressive_at_all_48": False,
            "reason": (
                "the current cap/short-leadin seam is C1 and geometry alone "
                "does not prove preload or the sign of the downstream reaction"
            ),
        },
        "authority_limit": (
            "The PEEK topology is the smallest reviewed geometry that meets "
            "the 0.30 mm section, endpoint-radius, and R0.36 witness checks. "
            "It does not authorize 0.5 mm passage through the narrower current "
            "cap lane or any force, route, collision, tolerance, or release claim."
        ),
        "validation_plan": [
            "Parameterize the right cap-side endpoint by received wire diameter and create the two-arc R3.5 transition inside the 1.50 mm shelf length.",
            "Widen the complete permanent-cap groove to at least 0.65 mm before any 0.5 mm route claim.",
            "Generate all 24 right shelves on front and rear caps after global groove cuts; preserve one solid and 24fold/front-rear symmetry.",
            "Run all 48 rebound endpoint and R0.36 insertion witnesses at both wire diameters and tolerance extremes.",
            "Prove the incident tension resultant is compressive into Rz(tooth*15deg)*(0,+1,0) for every g=0 locus.",
            "Rebind all 2400 routes and rerun full raw rigid/copper/self collision, extraction, cap balance/inertia, retention, polish, enamel, wear, and endurance checks.",
        ],
        "source_hashes": source_hashes,
        "artifacts": artifacts,
    }
    report["report_sha256"] = _canonical_hash(report)
    validate_report_integrity(report)
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported g0 landing trade schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("g0 landing trade report hash mismatch")
    if report.get("status") != "FAIL":
        raise ValueError("g0 landing trade must remain fail closed")
    if report.get("selected_candidate_id") != (
            "diameter_rebound_integral_PEEK_cap_side_shelf"):
        raise ValueError("g0 landing selected topology drift")
    for key in (
        "production_authorized", "wire_route_authorized",
        "collision_authorized", "assembly_integration_authorized",
        "selected_release_modified",
    ):
        if report.get(key) is not False:
            raise ValueError(f"g0 landing trade invented {key}")
    if not all(report.get("geometry_gates", {}).values()):
        raise ValueError("selected PEEK geometry witness failed")
    if any(report.get("release_gates", {}).values()):
        raise ValueError("open g0 landing release gate was promoted")
    candidates = report.get("candidates", [])
    if not isinstance(candidates, list) or len(candidates) != 3:
        raise ValueError("g0 landing candidate partition incomplete")
    by_id = {row["id"]: row for row in candidates}
    if by_id["right_seam_mouth_cut_shift_plus_0p30mm"][
            "all_cases_pass"] is not False:
        raise ValueError("colliding seam-cut shift was promoted")
    if by_id["installed_0p127mm_Nomex_BREP_with_route_rebind"][
            "selected"] is not False:
        raise ValueError("unretained Nomex topology was promoted")
    for relative, expected_hash in report.get("source_hashes", {}).items():
        path = ROOT / Path(str(relative).replace("\\", "/"))
        if not path.is_file() or _sha256(path) != expected_hash:
            raise ValueError(f"g0 landing source hash drift: {relative}")
    for row in report.get("artifacts", {}).values():
        path = ROOT / Path(str(row["path"]).replace("\\", "/"))
        if (not path.is_file()
                or path.stat().st_size != row["byte_count"]
                or _sha256(path) != row["sha256"]):
            raise ValueError(f"g0 landing artifact hash drift: {row['path']}")


def render_markdown(report: Mapping[str, Any]) -> str:
    selected = next(
        row for row in report["candidates"] if row["selected"]
    )
    inputs = report["inputs"]
    balance = report["mass_and_balance_estimate"]
    lines = [
        "# Robust g=0 landing redesign trade", "",
        f"**{report['status']} — {report['decision']}**", "",
        "The smallest reviewed tolerance-plausible topology is a diameter-"
        "rebound integral PEEK cap-side shelf. It is selected for redesign, "
        "not integrated or released.", "",
        "## Selected PEEK topology", "",
        f"- Shelf: {selected['shelf_dimensions_mm']['radial_cap_side_length']:.2f} mm radial x "
        f"{selected['shelf_dimensions_mm']['axial_width']:.2f} mm axial x "
        f">={selected['shelf_dimensions_mm']['minimum_stock_behind_contact_face']:.2f} mm stock.",
        f"- Mouth: {selected['mouth_dimensions_mm']['radial_length']:.2f} x "
        f"{selected['mouth_dimensions_mm']['tangential_width']:.2f} x "
        f"{selected['mouth_dimensions_mm']['axial_span']:.2f} mm, center Y="
        f"{selected['mouth_dimensions_mm']['center_y']:.12f} mm.",
        "- Front/rear modified witnesses stay one solid, place both wire "
        "centers exactly one radius from the cap, and clear both R0.36 gauges.", "",
        "## Rejected alternatives", "",
        "- 0.127 mm Nomex: longer route rebind, no selected retention or installed BREP, and open edge/wear/tolerance behavior.",
        "- +0.30 mm seam-cut shift: restores the old floor but positively intersects the R0.36 insertion gauge.", "",
        "## Range blocker", "",
        f"The current cap lane is only {inputs['current_cap_lane_clear_width_mm']:.5f} mm clear, "
        "which is narrower than 0.5 mm wire. The complete lane must be widened "
        f"to at least {inputs['required_range_cap_lane_clear_width_mm']:.2f} mm and revalidated.", "",
        "## Balance and authority", "",
        f"The conservative pre-mouth mass estimate is <="
        f"{balance['maximum_total_added_PEEK_mass_g_before_mouth_accounting']:.6f} g. "
        "Twenty-fourfold front/rear replication cancels first moments, but "
        "polar inertia and retention loads remain open.", "",
        "Force direction, 2400 routes, full collision, tolerance, polish, "
        "enamel, wear, endurance, assembly, release, and production remain unauthorized.", "",
        f"Bound g=0 audit SHA-256: `{report['artifacts']['g0_normal_audit']['sha256']}`", "",
        f"Report SHA-256: `{report['report_sha256']}`", "",
    ]
    return "\n".join(lines)


def write_outputs(report: Mapping[str, Any] | None = None) -> dict[str, Any]:
    value = dict(report or analyze())
    validate_report_integrity(value)
    OUTPUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(
        json.dumps(value, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    OUTPUT_MD.write_text(render_markdown(value), encoding="utf-8")
    return value


def main(argv: Iterable[str] | None = None) -> int:
    del argv
    report = write_outputs()
    print(
        f"g0 landing trade {report['status']}: "
        f"selected={report['selected_candidate_id']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
