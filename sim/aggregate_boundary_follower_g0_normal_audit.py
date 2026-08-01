"""Exact fail-closed g=0 physical-normal audit for the follower endpoint set.

The audit queries all 48 g=0 terminal bindings against the finished selected
PEEK cap BREP.  It also constructs, without modifying cap CAD, the smallest
bounded cap-side landing witness found for the unsupported right seam.  Route,
force, collision, tolerance, release, and production authority remain false.
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


SCHEMA = "aggregate-boundary-follower-g0-normal-audit/v1"
LOCI_PATH = ROOT / "out" / "reports" / (
    "carriage_active_sector_terminal_guide_loci.json"
)
CAP_REPORT_PATH = ROOT / "out" / "reports" / (
    "permanent_cap_production_review.json"
)
CAP_STEP_PATH = ROOT / "out" / "review" / (
    "permanent_cap_production_review.step"
)
INTEGRATED_STEP_PATH = ROOT / "out" / "review" / (
    "integrated_release_candidate.step"
)
OUTPUT_JSON = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_g0_normal_audit.json"
)
OUTPUT_MD = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_g0_normal_audit.md"
)

SOURCE_PATHS = (
    Path("cad/permanent_cap_production_review.py"),
    Path("cad/carriage_active_sector_terminal_guide.py"),
    Path("cad/stator_insulation_nomex410.py"),
    Path("sim/carriage_active_sector_terminal_guide_audit.py"),
)

EXPECTED_G0_LOCI = 48
EXPECTED_EXISTING_OWNER_COUNT = 24
EXPECTED_UNSUPPORTED_COUNT = 24
DISTANCE_TOLERANCE_MM = 1.0e-9
RIGHT_SEAM_INSERTION_GAUGE_RADIUS_MM = 0.36

# Constructive witness dimensions.  The 0.50 mm cap-side length is a bounded
# finite landing, not a claimed manufacturing minimum.  The exact minimum
# normal protrusion is recomputed from the selected cap BREP.
LANDING_RADIAL_LENGTH_MM = 0.50
LANDING_CAP_EMBED_MM = 0.05
LANDING_AXIAL_WIDTH_MM = cap.GROOVE_CLEAR_WIDTH_MM


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


def _rotate_xy(point: tuple[float, float, float], tooth: int) \
        -> tuple[float, float, float]:
    angle = math.radians(int(tooth) * guide.PITCH_DEG)
    cosine, sine = math.cos(angle), math.sin(angle)
    x, y, z = map(float, point)
    return (
        x * cosine - y * sine,
        x * sine + y * cosine,
        z,
    )


def _unrotate_vector(vector: tuple[float, float, float], tooth: int) \
        -> list[float]:
    angle = -math.radians(int(tooth) * guide.PITCH_DEG)
    cosine, sine = math.cos(angle), math.sin(angle)
    x, y, z = map(float, vector)
    return [
        x * cosine - y * sine,
        x * sine + y * cosine,
        z,
    ]


def _unit(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(sum(float(value) ** 2 for value in vector))
    if length <= 1.0e-15:
        raise ValueError("zero-length surface normal witness")
    return tuple(float(value) / length for value in vector)


def _common_volume(one, two) -> float:
    common = one & two
    return 0.0 if common is None else float(common.volume)


def _bbox(shape) -> list[float]:
    bounds = shape.bounding_box()
    return [float(value) for value in (
        bounds.min.X, bounds.max.X,
        bounds.min.Y, bounds.max.Y,
        bounds.min.Z, bounds.max.Z,
    )]


def _g0_rows(loci_document: Mapping[str, Any]) -> list[dict[str, Any]]:
    rows = loci_document.get("loci", [])
    if not isinstance(rows, list):
        raise ValueError("terminal locus array is missing")
    result = []
    for row in rows:
        if int(row.get("turn_index", -1)) != 0:
            continue
        binding = row.get("terminal_binding", {})
        lane_id = str(binding.get("lane_id", ""))
        side = "left" if "_left_" in lane_id else (
            "right" if "_right_" in lane_id else "unknown"
        )
        result.append({
            "locus_index": int(row["locus_index"]),
            "pass_index": int(row["pass_index"]),
            "state_index": int(row["state_index"]),
            "turn_index": int(row["turn_index"]),
            "half_turn_index": int(row["half_turn_index"]),
            "tooth_index": int(row["tooth_index"]),
            "time_s": float(row["time_s"]),
            "lane_id": lane_id,
            "side": side,
            "axial_end": "front" if lane_id.endswith("_front") else "rear",
            "axial_sign": 1 if lane_id.endswith("_front") else -1,
            "cap_endpoint_name": binding.get("cap_endpoint_name"),
            "cap_endpoint_active_local_mm": list(map(
                float, binding["cap_endpoint_local_mm"]
            )),
        })
    if len(result) != EXPECTED_G0_LOCI:
        raise ValueError(f"expected 48 g=0 loci, found {len(result)}")
    return result


def _classify_endpoints(
    g0_rows: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], dict[int, Any]]:
    caps = {
        1: guide.cap_with_short_leadins(1),
        -1: guide.cap_with_short_leadins(-1),
    }
    result = []
    for row in g0_rows:
        tooth = row["tooth_index"]
        endpoint = _rotate_xy(
            tuple(row["cap_endpoint_active_local_mm"]), tooth
        )
        endpoint_vertex = Vertex(endpoint)
        selected_cap = caps[row["axial_sign"]]
        cap_point, _endpoint_point = selected_cap.closest_points(
            endpoint_vertex
        )
        surface_point = tuple(float(value) for value in cap_point)
        separation_vector = tuple(
            endpoint[index] - surface_point[index] for index in range(3)
        )
        distance = math.sqrt(sum(value * value for value in separation_vector))
        normal_stator = _unit(separation_vector)
        normal_active = _unrotate_vector(normal_stator, tooth)
        owned = math.isclose(
            distance, cap.WIRE_RADIUS_MM,
            rel_tol=0.0, abs_tol=DISTANCE_TOLERANCE_MM,
        )
        result.append({
            **row,
            "endpoint_stator_local_mm": list(endpoint),
            "nearest_selected_cap_surface_stator_local_mm": list(surface_point),
            "selected_cap_surface_distance_mm": distance,
            "current_job_wire_radius_mm": cap.WIRE_RADIUS_MM,
            "surface_to_wire_normal_stator_local": list(normal_stator),
            "surface_to_wire_normal_active_local": normal_active,
            "existing_positive_BREP_owner": owned,
            "unsupported_normal_gap_mm": max(
                0.0, distance - cap.WIRE_RADIUS_MM
            ),
            "owner": (
                "finished_selected_PEEK_cap_floor"
                if owned else None
            ),
            "status": "OWNED" if owned else "UNSUPPORTED",
        })
    return result, caps


def _right_seam_gauge(axial_sign: int):
    x, y, z = guide.cap_lane_endpoint(1, axial_sign)
    return (
        Pos(x, y, z)
        * Rot(0.0, 90.0, 0.0)
        * Cylinder(
            RIGHT_SEAM_INSERTION_GAUGE_RADIUS_MM,
            guide.RIGHT_SEAM_MOUTH_RADIAL_LENGTH_MM,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
    )


def _constructive_landing_witness(caps: Mapping[int, Any]) -> dict[str, Any]:
    endpoint_front = tuple(map(float, cap._lane_points()["waypoint"]))
    endpoint_vertex = Vertex(endpoint_front)
    current_point, _ = caps[1].closest_points(endpoint_vertex)
    current_surface = tuple(float(value) for value in current_point)
    contact_y = endpoint_front[1] - cap.WIRE_RADIUS_MM
    protrusion = contact_y - current_surface[1]
    if protrusion <= 0.0:
        raise ValueError("right seam no longer needs a positive landing")

    total_tangential = LANDING_CAP_EMBED_MM + protrusion
    total_box_volume = (
        LANDING_RADIAL_LENGTH_MM
        * total_tangential
        * LANDING_AXIAL_WIDTH_MM
    )
    cases = []
    for sign in (1, -1):
        endpoint = (
            endpoint_front[0], endpoint_front[1], sign * endpoint_front[2]
        )
        center = (
            endpoint[0] - LANDING_RADIAL_LENGTH_MM / 2.0,
            (
                current_surface[1] - LANDING_CAP_EMBED_MM + contact_y
            ) / 2.0,
            endpoint[2],
        )
        landing = Pos(*center) * Box(
            LANDING_RADIAL_LENGTH_MM,
            total_tangential,
            LANDING_AXIAL_WIDTH_MM,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
        selected_cap = caps[sign]
        wire = cap.nominal_wire_witness(0, sign)
        gauge = _right_seam_gauge(sign)
        cap_overlap = _common_volume(landing, selected_cap)
        wire_overlap = _common_volume(landing, wire)
        gauge_overlap = _common_volume(landing, gauge)
        net_new_volume = total_box_volume - cap_overlap
        case = {
            "axial_end": "front" if sign > 0 else "rear",
            "axial_sign": sign,
            "landing_bbox_xyz_minmax_mm": _bbox(landing),
            "landing_total_box_volume_mm3": total_box_volume,
            "landing_to_selected_cap_positive_overlap_mm3": cap_overlap,
            "landing_net_new_volume_mm3": net_new_volume,
            "landing_to_nominal_wire_positive_overlap_mm3": wire_overlap,
            "landing_to_nominal_wire_distance_mm": float(
                landing.distance_to(wire)
            ),
            "landing_to_R0p36_insertion_gauge_positive_overlap_mm3": (
                gauge_overlap
            ),
            "positive_fusion_to_cap": cap_overlap > 1.0e-8,
            "nominal_wire_exact_tangent_without_positive_overlap": (
                wire_overlap <= 1.0e-8
                and float(landing.distance_to(wire)) <= 1.0e-8
            ),
            "R0p36_plus_X_gauge_zero_positive_overlap": (
                gauge_overlap <= 1.0e-8
            ),
        }
        case["status"] = "PASS" if all((
            case["positive_fusion_to_cap"],
            case["nominal_wire_exact_tangent_without_positive_overlap"],
            case["R0p36_plus_X_gauge_zero_positive_overlap"],
        )) else "FAIL"
        cases.append(case)
    return {
        "kind": "integral_cap_side_PEEK_landing_constructive_witness",
        "not_integrated_into_cap_CAD": True,
        "canonical_right_endpoint_active_local_mm": list(endpoint_front),
        "current_nearest_selected_cap_surface_active_local_mm": list(
            current_surface
        ),
        "contact_surface_active_local_mm": [
            endpoint_front[0], contact_y, endpoint_front[2]
        ],
        "contact_normal_active_local": [0.0, 1.0, 0.0],
        "required_normal_protrusion_mm": protrusion,
        "required_normal_protrusion_um": 1000.0 * protrusion,
        "cap_embed_mm": LANDING_CAP_EMBED_MM,
        "radial_length_mm": LANDING_RADIAL_LENGTH_MM,
        "axial_contact_width_mm": LANDING_AXIAL_WIDTH_MM,
        "radial_extent_rule": "X in [endpoint_X-0.50, endpoint_X]",
        "gauge_avoidance_rule": (
            "landing ends at seam plane; insertion gauge occupies X>=endpoint_X"
        ),
        "cases": cases,
        "status": "PASS" if all(row["status"] == "PASS" for row in cases)
        else "FAIL",
        "constructive_positive_volume_geometry_proven": all(
            row["status"] == "PASS" for row in cases
        ),
    }


def analyze() -> dict[str, Any]:
    loci_document = _load(LOCI_PATH)
    g0 = _g0_rows(loci_document)
    classified, caps = _classify_endpoints(g0)
    owned = [row for row in classified if row["existing_positive_BREP_owner"]]
    unsupported = [
        row for row in classified if not row["existing_positive_BREP_owner"]
    ]
    landing = _constructive_landing_witness(caps)

    left_distances = [
        row["selected_cap_surface_distance_mm"] for row in classified
        if row["side"] == "left"
    ]
    right_distances = [
        row["selected_cap_surface_distance_mm"] for row in classified
        if row["side"] == "right"
    ]
    right_gap = max(
        row["unsupported_normal_gap_mm"] for row in unsupported
    )

    geometric_gates = {
        "exact_48_g0_loci_classified": len(classified) == EXPECTED_G0_LOCI,
        "24_existing_PEEK_floor_owners": (
            len(owned) == EXPECTED_EXISTING_OWNER_COUNT
        ),
        "24_right_seams_unsupported": (
            len(unsupported) == EXPECTED_UNSUPPORTED_COUNT
        ),
        "all_left_distances_equal_current_wire_radius": all(
            math.isclose(value, cap.WIRE_RADIUS_MM, rel_tol=0.0,
                         abs_tol=DISTANCE_TOLERANCE_MM)
            for value in left_distances
        ),
        "all_right_distances_share_one_gap": (
            max(right_distances) - min(right_distances) <= 1.0e-9
        ),
        "constructive_PEEK_landing_front_and_rear_OCC_pass": (
            landing["status"] == "PASS"
        ),
    }
    release_gates = {
        "landing_integrated_into_cap_CAD": False,
        "22p496um_protrusion_manufacturing_tolerance_qualified": False,
        "all_48_contact_force_resultants_compressive": False,
        "all_2400_routes_revalidated_after_feature": False,
        "full_raw_rigid_copper_self_collision_revalidated": False,
        "wire_range_0p2_to_0p5mm_revalidated": False,
        "enamel_wear_polish_and_endurance_qualified": False,
    }

    source_hashes = {
        str(path).replace("\\", "/"): _sha256(ROOT / path)
        for path in SOURCE_PATHS
    }
    artifacts = {
        "terminal_loci": {
            "path": "out/reports/carriage_active_sector_terminal_guide_loci.json",
            "byte_count": LOCI_PATH.stat().st_size,
            "sha256": _sha256(LOCI_PATH),
            "payload_sha256": loci_document.get("locus_payload_sha256"),
        },
        "permanent_cap_report": {
            "path": "out/reports/permanent_cap_production_review.json",
            "byte_count": CAP_REPORT_PATH.stat().st_size,
            "sha256": _sha256(CAP_REPORT_PATH),
        },
        "permanent_cap_STEP": {
            "path": "out/review/permanent_cap_production_review.step",
            "byte_count": CAP_STEP_PATH.stat().st_size,
            "sha256": _sha256(CAP_STEP_PATH),
            "note": "base permanent cap; selected short-leadin cuts are source-bound",
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
            "24_G0_NORMALS_OWNED__24_REQUIRE_UNQUALIFIED_RIGHT_SEAM_LANDING"
        ),
        "production_authorized": False,
        "wire_route_authorized": False,
        "collision_authorized": False,
        "assembly_integration_authorized": False,
        "selected_release_modified": False,
        "scope": {
            "current_job_wire_diameter_mm": cap.WIRE_DIAMETER_MM,
            "current_job_wire_radius_mm": cap.WIRE_RADIUS_MM,
            "wire_range_0p2_to_0p5mm_authorized": False,
            "source_owned_positive_surface_normal_only": True,
            "contact_force_direction_proven": False,
        },
        "coverage": {
            "required_g0_locus_count": EXPECTED_G0_LOCI,
            "classified_g0_locus_count": len(classified),
            "existing_positive_BREP_owner_count": len(owned),
            "unsupported_count": len(unsupported),
            "left_surface_distance_range_mm": [
                min(left_distances), max(left_distances)
            ],
            "right_surface_distance_range_mm": [
                min(right_distances), max(right_distances)
            ],
            "right_unsupported_gap_mm": right_gap,
            "right_unsupported_gap_um": 1000.0 * right_gap,
            "loci": classified,
        },
        "existing_owner_contract": {
            "owner": "finished selected natural-unfilled PEEK cap floor",
            "owned_side": "left",
            "canonical_endpoint_active_local_mm": [
                18.2, -2.05, 13.961655295982
            ],
            "canonical_nearest_surface_active_local_mm": [
                18.08824, -2.05, 13.961655295982
            ],
            "canonical_surface_to_wire_normal_active_local": [1.0, 0.0, 0.0],
            "physical_tooth_mapping": "Rz(tooth_index*15deg)",
        },
        "constructive_PEEK_landing_witness": landing,
        "nomex_assessment": {
            "current_3D_BREP_owner_available": False,
            "current_source_kind": "2D Shapely/DXF cut geometry",
            "selected_stock_nominal_thickness_mm": (
                nomex.MATERIAL_NOMINAL_THICKNESS_MM
            ),
            "selected_stock_receiving_min_mm": nomex.MATERIAL_RECEIVING_MIN_MM,
            "right_gap_mm": right_gap,
            "nominal_stock_to_gap_ratio": (
                nomex.MATERIAL_NOMINAL_THICKNESS_MM / right_gap
            ),
            "drop_in_flat_insert_valid": False,
            "reason": (
                "0.127 mm stock would overclose the 0.022496 mm gap and is "
                "not represented by an installed source-owned BREP"
            ),
        },
        "geometric_gates": geometric_gates,
        "release_gates": release_gates,
        "authority_limit": (
            "A positive surface normal is only geometric contact eligibility. "
            "The incident tension resultant must still be compressive, and the "
            "landing must pass tolerance, route, insertion, collision, wear, "
            "and endurance validation."
        ),
        "implementation_and_validation_plan": [
            "Add one integral cap-side right landing ending at the seam X plane; fuse it after all right-seam mouth cuts.",
            "Replicate the landing at all 24 right seams on both caps to preserve periodic and front/rear symmetry.",
            "Emit a source-owned face identifier, closest point, and outward normal for every g=0 terminal binding.",
            "Require all 48 nominal endpoint distances to equal the current-job wire radius without positive wire overlap.",
            "Prove the two incident tension vectors produce a compressive resultant for every claimed owner normal.",
            "Rerun all 48 R0.36 insertion gauges, cap one-solid checks, 2400 route bindings, full raw rigid/copper collision, balance, tolerance, polish, wear, and endurance gates.",
            "Bind this report into follower acceptance only after every release gate passes; do not delete current blockers from geometry alone.",
        ],
        "source_hashes": source_hashes,
        "artifacts": artifacts,
    }
    report["report_sha256"] = _canonical_hash(report)
    validate_report_integrity(report)
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported g0 normal audit schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("g0 normal audit report hash mismatch")
    if report.get("status") != "FAIL":
        raise ValueError("unqualified g0 normal audit must fail closed")
    for key in (
        "production_authorized", "wire_route_authorized",
        "collision_authorized", "assembly_integration_authorized",
        "selected_release_modified",
    ):
        if report.get(key) is not False:
            raise ValueError(f"g0 normal audit invented {key}")
    coverage = report.get("coverage", {})
    if coverage.get("classified_g0_locus_count") != 48:
        raise ValueError("g0 locus count drift")
    if coverage.get("existing_positive_BREP_owner_count") != 24:
        raise ValueError("existing g0 owner count drift")
    if coverage.get("unsupported_count") != 24:
        raise ValueError("unsupported g0 count drift")
    gap = float(coverage.get("right_unsupported_gap_mm", math.nan))
    if not math.isclose(gap, 0.02249624516932,
                        rel_tol=0.0, abs_tol=1.0e-9):
        raise ValueError("right seam normal gap drift")
    landing = report.get("constructive_PEEK_landing_witness", {})
    if landing.get("status") != "PASS":
        raise ValueError("constructive PEEK landing witness failed")
    if landing.get("not_integrated_into_cap_CAD") is not True:
        raise ValueError("landing witness was promoted to integrated CAD")
    if report.get("nomex_assessment", {}).get(
            "current_3D_BREP_owner_available") is not False:
        raise ValueError("2D Nomex geometry was promoted to a BREP owner")
    if any(report.get("release_gates", {}).values()):
        raise ValueError("open g0 release gate was promoted")
    for relative, expected_hash in report.get("source_hashes", {}).items():
        path = ROOT / Path(str(relative).replace("\\", "/"))
        if not path.is_file() or _sha256(path) != expected_hash:
            raise ValueError(f"g0 source hash drift: {relative}")
    for row in report.get("artifacts", {}).values():
        path = ROOT / Path(str(row["path"]).replace("\\", "/"))
        if (not path.is_file()
                or path.stat().st_size != row["byte_count"]
                or _sha256(path) != row["sha256"]):
            raise ValueError(f"g0 artifact hash drift: {row['path']}")


def render_markdown(report: Mapping[str, Any]) -> str:
    coverage = report["coverage"]
    landing = report["constructive_PEEK_landing_witness"]
    lines = [
        "# Aggregate-boundary follower g=0 physical-normal audit", "",
        f"**{report['status']} — {report['decision']}**", "",
        "The finished selected PEEK cap owns 24 left-side g=0 surface "
        "normals. Twenty-four right-side loci retain an unsupported seam gap.", "",
        "## Exact classification", "",
        f"- Classified: {coverage['classified_g0_locus_count']} / 48 loci.",
        f"- Existing PEEK owners: {coverage['existing_positive_BREP_owner_count']}.",
        f"- Unsupported right seams: {coverage['unsupported_count']}.",
        f"- Left distance: {coverage['left_surface_distance_range_mm'][0]:.12f} mm, equal to the current-job wire radius.",
        f"- Right distance: {coverage['right_surface_distance_range_mm'][0]:.12f} mm.",
        f"- Right unsupported gap: {coverage['right_unsupported_gap_um']:.6f} um.", "",
        "## Constructive PEEK witness", "",
        f"A {landing['radial_length_mm']:.3f} mm cap-side landing with "
        f"{landing['cap_embed_mm']:.3f} mm embed and "
        f"{landing['axial_contact_width_mm']:.5f} mm axial contact width "
        f"closes the {landing['required_normal_protrusion_um']:.6f} um gap.",
        "It has positive cap overlap, exact nominal-wire tangent contact, and "
        "zero positive overlap with the existing R0.36 +X insertion gauge in "
        "both front and rear witnesses.", "",
        "The landing is not integrated CAD. Its 22.496 um correction is below "
        "ordinary cap manufacturing tolerance and is not production-qualified.", "",
        "## Authority boundary", "",
        "Force direction, all-wire-range behavior, 2400 routes, full raw "
        "collision, insertion, tolerance, enamel, wear, and endurance remain "
        "open. Production, route, collision, and assembly authority are false.", "",
        f"Terminal loci SHA-256: `{report['artifacts']['terminal_loci']['sha256']}`", "",
        f"Cap STEP SHA-256: `{report['artifacts']['permanent_cap_STEP']['sha256']}`", "",
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
        f"g0 normal audit {report['status']}: "
        f"owned={report['coverage']['existing_positive_BREP_owner_count']}; "
        f"unsupported={report['coverage']['unsupported_count']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
