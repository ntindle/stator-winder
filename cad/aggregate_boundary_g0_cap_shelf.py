"""Isolated robust g=0 cap-shelf prototype; never selected assembly CAD.

The prototype derives front/rear review caps from the current short-leadin
source, opens a 0.65 mm complete cap lane, replaces every right seam mouth
with a diameter-range opening, and fuses a 1.50 x 0.75 x 0.30 mm integral
PEEK cap-side shelf at all 24 right seams on each cap.

No selected cap, assembly, player, BOM, release, or controller source imports
this module.  Force, route, collision, integration, tolerance, wear, and
production authority remain false.
"""

from __future__ import annotations

from copy import deepcopy
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

from build123d import (
    Align,
    Box,
    BuildSketch,
    Compound,
    Cylinder,
    Locations,
    Plane,
    Pos,
    RectangleRounded,
    Rot,
    Transition,
    Vertex,
    export_step,
    sweep,
)

import carriage_active_sector_terminal_guide as guide
import permanent_cap_production_review as cap


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REVIEW = ROOT / "out" / "review"
STEP_OUT = REVIEW / "aggregate_boundary_g0_cap_shelf.step"
MANIFEST_OUT = REVIEW / "aggregate_boundary_g0_cap_shelf.manifest.json"

CTR = (Align.CENTER, Align.CENTER, Align.CENTER)
MIN = (Align.CENTER, Align.CENTER, Align.MIN)

WIRE_DIAMETERS_MM = (0.2, 0.5)
LANE_CLEAR_WIDTH_MM = 0.65
LANE_MAX_WIRE_RADIUS_MM = max(WIRE_DIAMETERS_MM) / 2.0
LANE_CAVITY_OUTWARD_MM = 0.50
LANE_PROFILE_CORNER_RADIUS_MM = 0.10

CONTACT_SURFACE_Y_MM = 0.7686710365709818
SHELF_RADIAL_LENGTH_MM = 1.50
SHELF_AXIAL_WIDTH_MM = 0.75
SHELF_STOCK_MM = 0.30

MOUTH_RADIAL_LENGTH_MM = 2.40
MOUTH_TANGENTIAL_WIDTH_MM = 1.00
MOUTH_AXIAL_SPAN_MM = 0.90
INSERTION_GAUGE_RADIUS_MM = 0.36

PEEK_DENSITY_G_MM3 = guide.PEEK_DENSITY_G_MM3
SLOTS = guide.SLOTS
PITCH_DEG = guide.PITCH_DEG

SOURCE_PATHS = (
    Path("cad/aggregate_boundary_g0_cap_shelf.py"),
    Path("cad/aggregate_boundary_g0_cap_shelf_brief.md"),
    Path("cad/permanent_cap_production_review.py"),
    Path("cad/carriage_active_sector_terminal_guide.py"),
    Path("out/reports/aggregate_boundary_follower_g0_landing_trade.json"),
)

AUTHORITY = {
    "isolated_review_only": True,
    "selected_cap_modified": False,
    "assembly_integration_authorized": False,
    "collision_authorized": False,
    "wire_route_authorized": False,
    "force_normal_authorized": False,
    "production_authorized": False,
    "release_authorized": False,
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(value: Mapping[str, Any]) -> str:
    body = deepcopy(dict(value))
    body.pop("manifest_sha256", None)
    return hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def endpoint_for_diameter(
    wire_diameter_mm: float, axial_sign: int = 1,
) -> tuple[float, float, float]:
    diameter = float(wire_diameter_mm)
    if not any(math.isclose(diameter, value, abs_tol=1.0e-12)
               for value in WIRE_DIAMETERS_MM):
        raise ValueError("review wire diameter must be 0.2 or 0.5 mm")
    sign = guide._validate_sign(axial_sign)
    x, _y, z = cap._lane_points()["waypoint"]
    return (
        float(x),
        CONTACT_SURFACE_Y_MM + diameter / 2.0,
        sign * float(z),
    )


def mouth_center_y() -> float:
    values = [endpoint_for_diameter(value)[1]
              for value in WIRE_DIAMETERS_MM]
    return (min(values) + max(values)) / 2.0


def _widened_lane_negative(axial_sign: int):
    sign = guide._validate_sign(axial_sign)
    start = cap._lane_points()["start"]
    start = (float(start[0]), float(start[1]), sign * float(start[2]))
    cavity_left = -LANE_MAX_WIRE_RADIUS_MM
    cavity_right = LANE_CAVITY_OUTWARD_MM
    cavity_width = cavity_right - cavity_left
    cavity_center = (cavity_right + cavity_left) / 2.0
    with BuildSketch(Plane(
        origin=start,
        x_dir=(1.0, 0.0, 0.0),
        z_dir=(0.0, 0.0, float(sign)),
    )) as profile:
        with Locations((cavity_center, 0.0)):
            RectangleRounded(
                cavity_width,
                LANE_CLEAR_WIDTH_MM,
                LANE_PROFILE_CORNER_RADIUS_MM,
            )
    result = sweep(
        profile.sketch,
        cap.lane_wire(sign),
        transition=Transition.ROUND,
    )
    result.label = (
        f"{'front' if sign > 0 else 'rear'}_tooth00_"
        "0p65_clear_maxR0p25_cap_lane_negative"
    )
    return result


def widened_lane_negative_for_tooth(tooth: int, axial_sign: int):
    if int(tooth) not in range(SLOTS):
        raise ValueError("tooth outside 0..23")
    result = Rot(0.0, 0.0, int(tooth) * PITCH_DEG) * (
        _widened_lane_negative(axial_sign)
    )
    result.label = (
        f"tooth_{int(tooth):02d}_"
        f"{'front' if axial_sign > 0 else 'rear'}_widened_lane_negative"
    )
    return result


def right_mouth_negative(axial_sign: int):
    sign = guide._validate_sign(axial_sign)
    x, _y, z = cap._lane_points()["waypoint"]
    result = Pos(float(x), mouth_center_y(), sign * float(z)) * Box(
        MOUTH_RADIAL_LENGTH_MM,
        MOUTH_TANGENTIAL_WIDTH_MM,
        MOUTH_AXIAL_SPAN_MM,
        align=(Align.MIN, Align.CENTER, Align.CENTER),
    )
    result.label = (
        f"{'front' if sign > 0 else 'rear'}_right_recentered_"
        "2p40x1p00x0p90_mouth_negative"
    )
    return result


def right_mouth_negative_for_tooth(tooth: int, axial_sign: int):
    if int(tooth) not in range(SLOTS):
        raise ValueError("tooth outside 0..23")
    result = Rot(0.0, 0.0, int(tooth) * PITCH_DEG) * (
        right_mouth_negative(axial_sign)
    )
    result.label = (
        f"tooth_{int(tooth):02d}_"
        f"{'front' if axial_sign > 0 else 'rear'}_right_mouth_negative"
    )
    return result


def integral_shelf(axial_sign: int):
    sign = guide._validate_sign(axial_sign)
    x, _y, z = cap._lane_points()["waypoint"]
    result = Pos(
        float(x) - SHELF_RADIAL_LENGTH_MM / 2.0,
        CONTACT_SURFACE_Y_MM - SHELF_STOCK_MM / 2.0,
        sign * float(z),
    ) * Box(
        SHELF_RADIAL_LENGTH_MM,
        SHELF_STOCK_MM,
        SHELF_AXIAL_WIDTH_MM,
        align=CTR,
    )
    result.label = (
        f"{'front' if sign > 0 else 'rear'}_integral_PEEK_"
        "1p50x0p75x0p30_g0_shelf"
    )
    return result


def integral_shelf_for_tooth(tooth: int, axial_sign: int):
    if int(tooth) not in range(SLOTS):
        raise ValueError("tooth outside 0..23")
    result = Rot(0.0, 0.0, int(tooth) * PITCH_DEG) * (
        integral_shelf(axial_sign)
    )
    result.label = (
        f"tooth_{int(tooth):02d}_"
        f"{'front' if axial_sign > 0 else 'rear'}_integral_g0_shelf"
    )
    return result


@lru_cache(maxsize=2)
def finished_cap(axial_sign: int):
    """Return one isolated one-solid cap-shelf prototype."""

    sign = guide._validate_sign(axial_sign)
    body = guide._cap_with_short_leadins_before_right_seam_mouth(sign)
    lane_cuts = [
        widened_lane_negative_for_tooth(tooth, sign)
        for tooth in range(SLOTS)
    ]
    mouth_cuts = [
        right_mouth_negative_for_tooth(tooth, sign)
        for tooth in range(SLOTS)
    ]
    # Diameter-rebound cap-side corridors are explicit review negatives.  The
    # inherited cap lane follows the current-job centreline; merely widening
    # its cross-section would still intersect the shifted 0.2/0.5 mm endpoint
    # witnesses.  Cutting both tangent cylinders provides the finite 1.50 mm
    # transition envelope while the shelf, fused afterward, restores only the
    # intended contact plane.
    endpoint_corridor_cuts = [
        wire_witness(tooth, sign, diameter)
        for tooth in range(SLOTS)
        for diameter in WIRE_DIAMETERS_MM
    ]
    shelves = [
        integral_shelf_for_tooth(tooth, sign)
        for tooth in range(SLOTS)
    ]
    body = body.cut(
        *lane_cuts, *mouth_cuts, *endpoint_corridor_cuts
    ).fuse(*shelves)
    solids = list(body.solids())
    if len(solids) != 1:
        raise RuntimeError(
            f"robust cap shelf must be one solid; got {len(solids)}"
        )
    body.label = (
        f"{'front' if sign > 0 else 'rear'}_isolated_robust_"
        "PEEK_cap_shelf_0p65_lane"
    )
    return body


def _x_cylinder(
    radius_mm: float,
    length_mm: float,
    origin: tuple[float, float, float],
):
    return (
        Pos(*origin)
        * Rot(0.0, 90.0, 0.0)
        * Cylinder(radius_mm, length_mm, align=MIN)
    )


def wire_witness(
    tooth: int, axial_sign: int, wire_diameter_mm: float,
):
    endpoint = endpoint_for_diameter(wire_diameter_mm, axial_sign)
    local = _x_cylinder(
        float(wire_diameter_mm) / 2.0,
        SHELF_RADIAL_LENGTH_MM,
        (
            endpoint[0] - SHELF_RADIAL_LENGTH_MM,
            endpoint[1],
            endpoint[2],
        ),
    )
    result = Rot(0.0, 0.0, int(tooth) * PITCH_DEG) * local
    result.label = (
        f"tooth_{int(tooth):02d}_"
        f"{'front' if axial_sign > 0 else 'rear'}_"
        f"wire_d{float(wire_diameter_mm):.1f}_tangent_witness"
    )
    return result


def insertion_gauge(
    tooth: int, axial_sign: int, wire_diameter_mm: float,
):
    endpoint = endpoint_for_diameter(wire_diameter_mm, axial_sign)
    local = _x_cylinder(
        INSERTION_GAUGE_RADIUS_MM,
        MOUTH_RADIAL_LENGTH_MM,
        endpoint,
    )
    result = Rot(0.0, 0.0, int(tooth) * PITCH_DEG) * local
    result.label = (
        f"tooth_{int(tooth):02d}_"
        f"{'front' if axial_sign > 0 else 'rear'}_"
        f"R0p36_gauge_for_d{float(wire_diameter_mm):.1f}"
    )
    return result


def _common_volume(one, two) -> float:
    common = one & two
    return 0.0 if common is None else float(common.volume)


def validation_report() -> dict[str, Any]:
    caps = {sign: finished_cap(sign) for sign in (1, -1)}
    cases = []
    for sign in (1, -1):
        for diameter in WIRE_DIAMETERS_MM:
            endpoint = endpoint_for_diameter(diameter, sign)
            wire = wire_witness(0, sign, diameter)
            gauge = insertion_gauge(0, sign, diameter)
            distance = float(caps[sign].distance_to(Vertex(endpoint)))
            wire_overlap = _common_volume(caps[sign], wire)
            gauge_overlap = _common_volume(caps[sign], gauge)
            cases.append({
                "axial_end": "front" if sign > 0 else "rear",
                "axial_sign": sign,
                "wire_diameter_mm": diameter,
                "endpoint_active_local_mm": list(endpoint),
                "endpoint_to_cap_distance_mm": distance,
                "expected_wire_radius_mm": diameter / 2.0,
                "cap_to_wire_positive_overlap_mm3": wire_overlap,
                "cap_to_R0p36_gauge_positive_overlap_mm3": gauge_overlap,
                "distance_equals_wire_radius": math.isclose(
                    distance, diameter / 2.0,
                    rel_tol=0.0, abs_tol=1.0e-8,
                ),
                "wire_zero_positive_overlap": wire_overlap <= 1.0e-8,
                "gauge_zero_positive_overlap": gauge_overlap <= 1.0e-8,
            })
    predecessor = {
        sign: guide.cap_with_short_leadins(sign) for sign in (1, -1)
    }
    volume_rows = []
    for sign in (1, -1):
        volume_rows.append({
            "axial_end": "front" if sign > 0 else "rear",
            "prototype_volume_mm3": float(caps[sign].volume),
            "selected_predecessor_volume_mm3": float(predecessor[sign].volume),
            "net_volume_delta_mm3": (
                float(caps[sign].volume) - float(predecessor[sign].volume)
            ),
            "prototype_mass_g": float(caps[sign].volume) * PEEK_DENSITY_G_MM3,
            "center_of_mass_mm": list(map(float, caps[sign].center())),
        })
    all_pass = all(
        row["distance_equals_wire_radius"]
        and row["wire_zero_positive_overlap"]
        and row["gauge_zero_positive_overlap"]
        for row in cases
    )
    return {
        "status": "PASS_GEOMETRY_ONLY" if all_pass else "FAIL",
        "cap_solid_counts": {
            "front": len(caps[1].solids()),
            "rear": len(caps[-1].solids()),
        },
        "diameter_cases": cases,
        "volume_mass_balance": volume_rows,
        "physical_shelf_count": 2 * SLOTS,
        "24fold_front_rear_symmetry_authored": True,
        "first_moment_expected_to_cancel": True,
        "exact_mass_inertia_release_update_complete": False,
        "authority": dict(AUTHORITY),
    }


def geometry_contract() -> dict[str, Any]:
    endpoint_rows = {
        f"d{diameter:.1f}": {
            "front_endpoint_active_local_mm": list(
                endpoint_for_diameter(diameter, 1)
            ),
            "rear_endpoint_active_local_mm": list(
                endpoint_for_diameter(diameter, -1)
            ),
        }
        for diameter in WIRE_DIAMETERS_MM
    }
    return {
        "schema": "aggregate-boundary-g0-cap-shelf-geometry/v1",
        "material": "natural unfilled PEEK review geometry",
        "wire_diameter_review_mm": list(WIRE_DIAMETERS_MM),
        "lane_clear_width_mm": LANE_CLEAR_WIDTH_MM,
        "lane_max_wire_radius_mm": LANE_MAX_WIRE_RADIUS_MM,
        "shelf_dimensions_mm": [
            SHELF_RADIAL_LENGTH_MM,
            SHELF_AXIAL_WIDTH_MM,
            SHELF_STOCK_MM,
        ],
        "contact_surface_y_mm": CONTACT_SURFACE_Y_MM,
        "mouth_dimensions_mm": [
            MOUTH_RADIAL_LENGTH_MM,
            MOUTH_TANGENTIAL_WIDTH_MM,
            MOUTH_AXIAL_SPAN_MM,
        ],
        "mouth_center_y_mm": mouth_center_y(),
        "insertion_gauge_radius_mm": INSERTION_GAUGE_RADIUS_MM,
        "shelf_count_front_rear": 2 * SLOTS,
        "endpoint_contract": endpoint_rows,
        "output_target": "out/review/aggregate_boundary_g0_cap_shelf.step",
        "authority": dict(AUTHORITY),
    }


def gen_step() -> Compound:
    children = [finished_cap(1), finished_cap(-1)]
    # Place the two diameter review sets on different teeth so their visible
    # wire/gauge envelopes do not hide each other.
    for sign in (1, -1):
        for tooth, diameter in enumerate(WIRE_DIAMETERS_MM):
            children.extend((
                wire_witness(tooth, sign, diameter),
                insertion_gauge(tooth, sign, diameter),
            ))
    result = Compound(children=children)
    result.label = "isolated_robust_g0_cap_shelf_review_only"
    return result


def write_outputs() -> dict[str, Any]:
    REVIEW.mkdir(parents=True, exist_ok=True)
    shape = gen_step()
    export_step(shape, STEP_OUT)
    validation = validation_report()
    manifest: dict[str, Any] = {
        "schema": "aggregate-boundary-g0-cap-shelf-manifest/v1",
        "artifact": str(STEP_OUT.relative_to(ROOT)).replace("\\", "/"),
        "artifact_byte_count": STEP_OUT.stat().st_size,
        "artifact_sha256": _sha256(STEP_OUT),
        "geometry_contract": geometry_contract(),
        "validation": validation,
        "source_hashes": {
            str(path).replace("\\", "/"): _sha256(ROOT / path)
            for path in SOURCE_PATHS
        },
        "selected_cap_or_release_modified": False,
    }
    manifest["manifest_sha256"] = _canonical_hash(manifest)
    MANIFEST_OUT.write_text(
        json.dumps(manifest, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return manifest


if __name__ == "__main__":
    value = write_outputs()
    print(
        f"robust g0 cap shelf: {value['validation']['status']}; "
        f"sha256={value['artifact_sha256']}"
    )
