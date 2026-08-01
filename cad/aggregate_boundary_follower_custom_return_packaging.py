"""Isolated custom-return packaging prototype for one follower occurrence.

The STEP shows a nominal OD3 x 16 tangential shaft, an igus
WPFFM-0304-05 envelope and provisional pocket, two unselected custom 17-7PH
torsion-spring envelopes with indexed anchors, and a contained 9293K122
constant-force cartridge envelope driving a review-only reduction lever.

This source is deliberately not imported by the main follower, assembly,
player, BOM, procurement, or release paths.  Geometry is packaging evidence
only and grants no physical, load, fatigue, procurement, integration, or
release authority.
"""

from __future__ import annotations

import itertools
import json
import math
from pathlib import Path
from typing import Any

from build123d import Align, Box, Compound, Cylinder, Pos, Rot, Torus

import aggregate_boundary_floating_follower as current_follower


CTR = (Align.CENTER, Align.CENTER, Align.CENTER)
HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REVIEW = ROOT / "out" / "review"
MANIFEST = REVIEW / "aggregate_boundary_follower_custom_return_packaging.json"

SCHEMA = "aggregate-boundary-follower-custom-return-packaging/v1"
REFERENCE_RADIAL_STATE = "mid"
REFERENCE_TANGENTIAL_STATE = "center"
TANGENTIAL_OFFSETS_MM = {
    "negative_hard": -0.60,
    "center": 0.0,
    "positive_hard": 0.60,
}

SHAFT_AXIS_X_MM = 17.0
SHAFT_AXIS_Z_MM = 20.0
SHAFT_DIAMETER_MM = 3.0
SHAFT_LENGTH_MM = 16.0

IGUS_CATALOG_NUMBER = "WPFFM-0304-05"
IGUS_ID_MM = 3.0
IGUS_BODY_OD_MM = 4.5
IGUS_BODY_LENGTH_MM = 5.0
IGUS_FLANGE_OD_MM = 7.5
IGUS_FLANGE_THICKNESS_MM = 0.75
IGUS_BODY_POCKET_DIAMETER_MM = 4.54
IGUS_FLANGE_COUNTERFACE_DIAMETER_MM = 7.54

TORSION_WIRE_DIAMETER_MM = 0.30
TORSION_MEAN_COIL_DIAMETER_MM = 4.00
TORSION_ACTIVE_COILS = 2.63671875
TORSION_REVIEW_RING_COUNT = 3
TORSION_REVIEW_RING_PITCH_MM = 0.28
TORSION_COIL_Y_MM = 4.75
TORSION_MOVING_LEG_BOTTOM_Z_MM = 17.10
TORSION_FIXED_LEG_TOP_Z_MM = 22.50

CARTRIDGE_CATALOG_NUMBER = "9293K122"
CARTRIDGE_CENTER = (25.0, -19.75, 20.0)
CARTRIDGE_COIL_OD_MM = 15.75
CARTRIDGE_COIL_WIDTH_MM = 6.35
CARTRIDGE_STRIP_WIDTH_MM = 6.35
CONTAINMENT_SIZE_MM = (17.5, 16.5, 8.0)
CONTAINMENT_BOUNDS_MM = {
    "x": [16.25, 33.75],
    "y": [-28.0, -11.5],
    "z": [16.0, 24.0],
}
SERVICE_POCKET_BOUNDS_MM = {
    "x": [16.0, 34.0],
    "y": [-28.0, -11.0],
    "z": [15.5, 24.5],
}
GLOBAL_REVIEW_BOUNDS_MM = {
    "x": [8.0, 34.0],
    "y": [-28.0, 8.0],
    "z": [5.5, 26.0],
}

LEVER_PIVOT_XY_MM = (25.0, -9.5)
LEVER_OUTPUT_XY_MM = (17.0, -6.0)
LEVER_CENTER_Z_MM = 11.80
LEVER_THICKNESS_MM = 1.0
LEVER_WIDTH_MM = 2.0
LEVER_RATIO_HOLES = (0.235, 0.270, 0.315)

AUTHORITY = {
    "review_only": True,
    "custom_parts_selected": False,
    "physical_fit_authority": False,
    "load_authority": False,
    "spring_rate_authority": False,
    "fatigue_authority": False,
    "procurement_authority": False,
    "assembly_integration_authority": False,
    "collision_release_authority": False,
    "BOM_change_authorized": False,
    "order_authorized": False,
    "production_authority": False,
    "release_authority": False,
}


def _label(part, label: str):
    part.label = label
    return part


def _cylinder_y(radius: float, length: float):
    return Rot(90.0, 0.0, 0.0) * Cylinder(radius, length, align=CTR)


def _bar_xy(
    one: tuple[float, float],
    two: tuple[float, float],
    width_mm: float,
    thickness_mm: float,
    z_mm: float,
):
    dx = two[0] - one[0]
    dy = two[1] - one[1]
    length = math.hypot(dx, dy)
    angle_deg = math.degrees(math.atan2(dy, dx))
    midpoint = ((one[0] + two[0]) / 2.0, (one[1] + two[1]) / 2.0)
    bar = Pos(midpoint[0], midpoint[1], z_mm) * (
        Rot(0.0, 0.0, angle_deg)
        * Box(length, width_mm, thickness_mm, align=CTR)
    )
    for x, y in (one, two):
        bar += Pos(x, y, z_mm) * Cylinder(
            width_mm / 2.0, thickness_mm, align=CTR
        )
    return bar


def tangential_offset(state: str) -> float:
    try:
        return TANGENTIAL_OFFSETS_MM[state]
    except KeyError as exc:
        raise ValueError(
            "state must be negative_hard, center, or positive_hard"
        ) from exc


def current_radial_slide_context():
    """Current midpoint radial slide, copied only as placement context."""

    part = current_follower.radial_slide(REFERENCE_RADIAL_STATE)
    return _label(part, "CONTEXT_current_radial_slide_mid_not_modified")


def fixed_shaft_support_yoke():
    """Unselected yoke with two bored ears and a Z=16 contact datum."""

    crossbar = Pos(10.50, 0.0, 18.0) * Box(5.0, 16.0, 4.0, align=CTR)
    ear_parts = []
    for side in (-1.0, 1.0):
        ear = Pos(16.75, side * 7.25, 20.5) * Box(
            6.5, 1.5, 9.0, align=CTR
        )
        bridge = Pos(13.25, side * 7.25, 18.0) * Box(
            1.5, 1.5, 4.0, align=CTR
        )
        ear_parts.extend((ear, bridge))
    body = crossbar
    for part in ear_parts:
        body += part
    body -= Pos(SHAFT_AXIS_X_MM, 0.0, SHAFT_AXIS_Z_MM) * _cylinder_y(
        SHAFT_DIAMETER_MM / 2.0 + 0.02, 18.0
    )
    return _label(
        body,
        "UNSELECTED_custom_fixed_OD3_shaft_support_yoke_review_only",
    )


def moving_bushing_carriage(state: str = REFERENCE_TANGENTIAL_STATE):
    """Unselected moving carrier with the provisional igus pocket."""

    y = tangential_offset(state)
    body = Pos(SHAFT_AXIS_X_MM, y, SHAFT_AXIS_Z_MM) * Box(
        7.0, 5.5, 6.0, align=CTR
    )
    beam = Pos(23.0, y, 18.0) * Box(5.0, 4.0, 2.0, align=CTR)
    neck = Pos(20.75, y, 18.0) * Box(1.5, 4.0, 2.0, align=CTR)
    body += beam
    body += neck

    # Moving spring-anchor tabs reach toward the two fixed coil planes.  Their
    # Z=17.10 top faces meet the wire-leg envelope without positive overlap.
    for side in (-1.0, 1.0):
        tab_center_y = y + side * 3.875
        tab = Pos(15.0, tab_center_y, 16.55) * Box(
            2.0, 2.25, 1.10, align=CTR
        )
        body += tab

    body -= Pos(SHAFT_AXIS_X_MM, y, SHAFT_AXIS_Z_MM) * _cylinder_y(
        IGUS_BODY_POCKET_DIAMETER_MM / 2.0, IGUS_BODY_LENGTH_MM + 0.20
    )
    body -= Pos(SHAFT_AXIS_X_MM, y, SHAFT_AXIS_Z_MM) * _cylinder_y(
        SHAFT_DIAMETER_MM / 2.0 + 0.02, 7.0
    )
    flange_center_y = y - IGUS_BODY_LENGTH_MM / 2.0 - 0.425
    body -= Pos(SHAFT_AXIS_X_MM, flange_center_y, SHAFT_AXIS_Z_MM) * _cylinder_y(
        IGUS_FLANGE_COUNTERFACE_DIAMETER_MM / 2.0,
        IGUS_FLANGE_THICKNESS_MM + 0.10,
    )
    return _label(
        body,
        f"UNSELECTED_custom_moving_igus_pocket_carriage:{state}",
    )


def nominal_shaft():
    """Nominal OD3 x 16 shaft envelope; purchase/cut is not authorized."""

    return _label(
        Pos(SHAFT_AXIS_X_MM, 0.0, SHAFT_AXIS_Z_MM)
        * _cylinder_y(SHAFT_DIAMETER_MM / 2.0, SHAFT_LENGTH_MM),
        "UNSELECTED_COTS_envelope_ground_shaft_OD3x16_cut_required",
    )


def igus_bushing_envelope(state: str = REFERENCE_TANGENTIAL_STATE):
    """WPFFM-0304-05 catalog envelope with a true nominal ID3 bore."""

    y = tangential_offset(state)
    body = Pos(SHAFT_AXIS_X_MM, y, SHAFT_AXIS_Z_MM) * _cylinder_y(
        IGUS_BODY_OD_MM / 2.0, IGUS_BODY_LENGTH_MM
    )
    flange_center_y = y - IGUS_BODY_LENGTH_MM / 2.0 - (
        IGUS_FLANGE_THICKNESS_MM / 2.0
    )
    body += Pos(SHAFT_AXIS_X_MM, flange_center_y, SHAFT_AXIS_Z_MM) * _cylinder_y(
        IGUS_FLANGE_OD_MM / 2.0, IGUS_FLANGE_THICKNESS_MM
    )
    body -= Pos(SHAFT_AXIS_X_MM, y - 0.375, SHAFT_AXIS_Z_MM) * _cylinder_y(
        IGUS_ID_MM / 2.0, IGUS_BODY_LENGTH_MM + IGUS_FLANGE_THICKNESS_MM + 0.2
    )
    return _label(
        body,
        f"UNSELECTED_COTS_envelope_igus_{IGUS_CATALOG_NUMBER}:{state}",
    )


def indexed_fixed_anchor(side: int):
    """Review-only fixed plate with six visible prewind-index holes."""

    if side not in (-1, 1):
        raise ValueError("side must be -1 or +1")
    center_y = side * 5.25
    plate = Pos(SHAFT_AXIS_X_MM, center_y, 24.0) * Box(
        7.0, 2.5, 3.0, align=CTR
    )
    for angle_deg in (60.0, 72.0, 84.0, 96.0, 108.0, 120.0):
        angle = math.radians(angle_deg)
        x = SHAFT_AXIS_X_MM + 3.0 * math.cos(angle)
        z = SHAFT_AXIS_Z_MM + 3.0 * math.sin(angle)
        plate -= Pos(x, center_y, z) * _cylinder_y(0.25, 3.0)
    return _label(
        plate,
        "UNSELECTED_custom_indexed_17_7PH_fixed_anchor_"
        + ("negative_Y" if side < 0 else "positive_Y"),
    )


def torsion_spring_envelope(side: int):
    """Visible three-ring envelope for the screened 2.64-turn wire spring."""

    if side not in (-1, 1):
        raise ValueError("side must be -1 or +1")
    y = side * TORSION_COIL_Y_MM
    body = None
    for index in range(TORSION_REVIEW_RING_COUNT):
        offset = (index - 1) * TORSION_REVIEW_RING_PITCH_MM
        ring = Pos(SHAFT_AXIS_X_MM, y + offset, SHAFT_AXIS_Z_MM) * (
            Rot(90.0, 0.0, 0.0)
            * Torus(
                TORSION_MEAN_COIL_DIAMETER_MM / 2.0,
                TORSION_WIRE_DIAMETER_MM / 2.0,
                align=CTR,
            )
        )
        body = ring if body is None else body + ring
    moving_leg = Pos(
        SHAFT_AXIS_X_MM - TORSION_MEAN_COIL_DIAMETER_MM / 2.0,
        y,
        (TORSION_MOVING_LEG_BOTTOM_Z_MM + SHAFT_AXIS_Z_MM) / 2.0,
    ) * Cylinder(
        TORSION_WIRE_DIAMETER_MM / 2.0,
        SHAFT_AXIS_Z_MM - TORSION_MOVING_LEG_BOTTOM_Z_MM,
        align=CTR,
    )
    fixed_leg = Pos(
        SHAFT_AXIS_X_MM + TORSION_MEAN_COIL_DIAMETER_MM / 2.0,
        y,
        (SHAFT_AXIS_Z_MM + TORSION_FIXED_LEG_TOP_Z_MM) / 2.0,
    ) * Cylinder(
        TORSION_WIRE_DIAMETER_MM / 2.0,
        TORSION_FIXED_LEG_TOP_Z_MM - SHAFT_AXIS_Z_MM,
        align=CTR,
    )
    body += moving_leg
    body += fixed_leg
    return _label(
        body,
        "UNSELECTED_custom_17_7PH_torsion_pair_"
        + ("negative_Y" if side < 0 else "positive_Y")
        + "_N2p64_review_envelope",
    )


def cartridge_containment():
    """Fixed box containment with coil cavity, floor/roof and strip exit."""

    x, y, z = CARTRIDGE_CENTER
    body = Pos(x, y, z) * Box(*CONTAINMENT_SIZE_MM, align=CTR)
    body -= Pos(x, y, z) * Cylinder(
        CARTRIDGE_COIL_OD_MM / 2.0 + 0.05,
        CARTRIDGE_COIL_WIDTH_MM + 0.10,
        align=CTR,
    )
    # Positive-Y strip mouth opens through the wall while retaining the roof,
    # floor, negative wall and both lateral side walls.
    body -= Pos(x, -11.65, z) * Box(1.0, 3.0, 8.20, align=CTR)
    return _label(
        body,
        "UNSELECTED_custom_9293K122_fragment_containment_negative_Y",
    )


def cartridge_envelope():
    """Solid collision envelope for the unpurchased 9293K122 coil and strip."""

    x, y, z = CARTRIDGE_CENTER
    body = Pos(x, y, z) * Cylinder(
        CARTRIDGE_COIL_OD_MM / 2.0,
        CARTRIDGE_COIL_WIDTH_MM,
        align=CTR,
    )
    strip = Pos(x, -11.65, z) * Box(
        0.30, 1.20, CARTRIDGE_STRIP_WIDTH_MM, align=CTR
    )
    body += strip
    return _label(
        body,
        f"UNSELECTED_COTS_envelope_McMaster_{CARTRIDGE_CATALOG_NUMBER}_coil_and_strip",
    )


def reduction_lever():
    """Review lever with three input holes spanning the screened ratio range."""

    pivot = LEVER_PIVOT_XY_MM
    output = LEVER_OUTPUT_XY_MM
    output_radius = math.dist(pivot, output)
    max_input_radius = max(LEVER_RATIO_HOLES) * output_radius
    input_end = (pivot[0], pivot[1] - max_input_radius - 0.75)
    body = _bar_xy(
        pivot, output, LEVER_WIDTH_MM, LEVER_THICKNESS_MM,
        LEVER_CENTER_Z_MM,
    )
    body += _bar_xy(
        pivot, input_end, LEVER_WIDTH_MM, LEVER_THICKNESS_MM,
        LEVER_CENTER_Z_MM,
    )
    body -= Pos(*pivot, LEVER_CENTER_Z_MM) * Cylinder(
        1.05, LEVER_THICKNESS_MM + 0.2, align=CTR
    )
    body -= Pos(*output, LEVER_CENTER_Z_MM) * Cylinder(
        0.85, LEVER_THICKNESS_MM + 0.2, align=CTR
    )
    for ratio in LEVER_RATIO_HOLES:
        body -= Pos(
            pivot[0], pivot[1] - ratio * output_radius, LEVER_CENTER_Z_MM
        ) * Cylinder(0.35, LEVER_THICKNESS_MM + 0.2, align=CTR)
    return _label(
        body,
        "UNSELECTED_custom_9293K122_reduction_lever_ratio_0p235_to_0p315",
    )


def lever_pivot_bracket():
    """Fixed review bracket terminating at the lever lower face."""

    x, y = LEVER_PIVOT_XY_MM
    body = Pos(x, y, 9.80) * Box(5.0, 3.0, 3.0, align=CTR)
    body -= Pos(x, y, 9.80) * Cylinder(1.10, 3.2, align=CTR)
    return _label(
        body,
        "UNSELECTED_custom_fixed_reduction_lever_pivot_bracket",
    )


def radial_slide_output_anchor():
    """Moving review pad touching the current slide's negative-Y face."""

    x, y = LEVER_OUTPUT_XY_MM
    body = Pos(x, y, 14.0) * Box(4.0, 2.0, 3.0, align=CTR)
    body -= Pos(x, y, 14.0) * Cylinder(0.90, 3.2, align=CTR)
    return _label(
        body,
        "UNSELECTED_custom_radial_slide_reduction_lever_output_anchor",
    )


def strip_transfer_envelope():
    """Unselected vertical transfer envelope between lever and strip mouth."""

    bottom = LEVER_CENTER_Z_MM + LEVER_THICKNESS_MM / 2.0
    top = CARTRIDGE_CENTER[2] - CARTRIDGE_COIL_WIDTH_MM / 2.0
    return _label(
        Pos(25.0, -11.65, (bottom + top) / 2.0)
        * Box(0.30, 2.0, top - bottom, align=CTR),
        "UNSELECTED_custom_constant_force_strip_transfer_envelope",
    )


def custom_bodies(state: str = REFERENCE_TANGENTIAL_STATE) -> tuple:
    """Current context plus separately manufactured custom package bodies."""

    return (
        current_radial_slide_context(),
        fixed_shaft_support_yoke(),
        moving_bushing_carriage(state),
        indexed_fixed_anchor(-1),
        indexed_fixed_anchor(1),
        torsion_spring_envelope(-1),
        torsion_spring_envelope(1),
        cartridge_containment(),
        reduction_lever(),
        lever_pivot_bracket(),
        radial_slide_output_anchor(),
        strip_transfer_envelope(),
    )


def all_parts(state: str = REFERENCE_TANGENTIAL_STATE) -> tuple:
    """All custom bodies and unselected catalog envelopes in one state."""

    return (
        *custom_bodies(state),
        nominal_shaft(),
        igus_bushing_envelope(state),
        cartridge_envelope(),
    )


def _common_volume(one, two) -> float:
    try:
        common = one & two
        return float(common.volume) if common is not None else 0.0
    except Exception:
        return 0.0


def same_state_overlap_audit(
    state: str = REFERENCE_TANGENTIAL_STATE,
    *, include_catalog_envelopes: bool = False,
) -> dict[str, Any]:
    """Exact positive-volume audit; never compares geometry across states."""

    parts = all_parts(state) if include_catalog_envelopes else custom_bodies(state)
    rows = []
    for one, two in itertools.combinations(parts, 2):
        volume = _common_volume(one, two)
        if volume > 1.0e-7:
            rows.append({
                "one": one.label,
                "two": two.label,
                "common_volume_mm3": volume,
            })
    return {
        "state": state,
        "scope": "all_parts" if include_catalog_envelopes else "custom_bodies",
        "body_count": len(parts),
        "positive_overlap_count": len(rows),
        "positive_overlaps": rows,
        "status": "PASS" if not rows else "FAIL",
    }


def _bounds(shape) -> dict[str, list[float]]:
    box = shape.bounding_box()
    return {
        "x": [float(box.min.X), float(box.max.X)],
        "y": [float(box.min.Y), float(box.max.Y)],
        "z": [float(box.min.Z), float(box.max.Z)],
    }


def _inside(
    actual: dict[str, list[float]],
    allowed: dict[str, list[float]],
    tolerance: float = 1.0e-7,
) -> bool:
    return all(
        actual[axis][0] >= allowed[axis][0] - tolerance
        and actual[axis][1] <= allowed[axis][1] + tolerance
        for axis in ("x", "y", "z")
    )


def bounds_audit(state: str = REFERENCE_TANGENTIAL_STATE) -> dict[str, Any]:
    containment_bounds = _bounds(cartridge_containment())
    cartridge_bounds = _bounds(cartridge_envelope())
    assembly_bounds = _bounds(Compound(children=list(all_parts(state))))
    return {
        "state": state,
        "containment_bounds_mm": containment_bounds,
        "declared_containment_bounds_mm": CONTAINMENT_BOUNDS_MM,
        "containment_matches_declared_bounds": all(
            math.isclose(
                containment_bounds[axis][index],
                CONTAINMENT_BOUNDS_MM[axis][index],
                abs_tol=1.0e-7,
            )
            for axis in ("x", "y", "z")
            for index in (0, 1)
        ),
        "cartridge_envelope_bounds_mm": cartridge_bounds,
        "cartridge_inside_service_pocket": _inside(
            cartridge_bounds, SERVICE_POCKET_BOUNDS_MM
        ),
        "containment_inside_service_pocket": _inside(
            containment_bounds, SERVICE_POCKET_BOUNDS_MM
        ),
        "assembly_bounds_mm": assembly_bounds,
        "global_review_bounds_mm": GLOBAL_REVIEW_BOUNDS_MM,
        "assembly_inside_global_review_bounds": _inside(
            assembly_bounds, GLOBAL_REVIEW_BOUNDS_MM
        ),
    }


def geometry_contract() -> dict[str, Any]:
    overlap_by_state = {
        state: {
            "custom": same_state_overlap_audit(state),
            "all_parts": same_state_overlap_audit(
                state, include_catalog_envelopes=True
            ),
            "bounds": bounds_audit(state),
        }
        for state in TANGENTIAL_OFFSETS_MM
    }
    custom_center = custom_bodies(REFERENCE_TANGENTIAL_STATE)
    all_center = all_parts(REFERENCE_TANGENTIAL_STATE)
    output_radius = math.dist(LEVER_PIVOT_XY_MM, LEVER_OUTPUT_XY_MM)
    return {
        "schema": SCHEMA,
        "status": "REVIEW_ONLY_CUSTOM_RETURN_PACKAGING_NO_AUTHORITY",
        "artifacts": {
            "source": "cad/aggregate_boundary_follower_custom_return_packaging.py",
            "brief": (
                "cad/aggregate_boundary_follower_custom_return_packaging_brief.md"
            ),
            "focused_tests": (
                "cad/test_aggregate_boundary_follower_custom_return_packaging.py"
            ),
            "step": (
                "out/review/aggregate_boundary_follower_custom_return_packaging.step"
            ),
            "manifest": (
                "out/review/aggregate_boundary_follower_custom_return_packaging.json"
            ),
            "snapshot_job": (
                "cad/aggregate_boundary_follower_custom_return_packaging.snapshots.json"
            ),
        },
        "units": "mm",
        "frame": {
            "+X": "radial outward",
            "+Y": "tangential and shaft axis",
            "+Z": "stator axis",
        },
        "reference_state": {
            "radial": REFERENCE_RADIAL_STATE,
            "tangential": REFERENCE_TANGENTIAL_STATE,
        },
        "shaft": {
            "diameter_mm": SHAFT_DIAMETER_MM,
            "length_mm": SHAFT_LENGTH_MM,
            "axis": "+Y",
            "center": [SHAFT_AXIS_X_MM, 0.0, SHAFT_AXIS_Z_MM],
            "selected": False,
            "cut_and_metrology_complete": False,
        },
        "igus_bushing_and_pocket": {
            "catalog_number": IGUS_CATALOG_NUMBER,
            "ID_mm": IGUS_ID_MM,
            "body_OD_mm": IGUS_BODY_OD_MM,
            "body_length_mm": IGUS_BODY_LENGTH_MM,
            "flange_OD_mm": IGUS_FLANGE_OD_MM,
            "flange_thickness_mm": IGUS_FLANGE_THICKNESS_MM,
            "provisional_body_pocket_diameter_mm": (
                IGUS_BODY_POCKET_DIAMETER_MM
            ),
            "provisional_flange_counterface_diameter_mm": (
                IGUS_FLANGE_COUNTERFACE_DIAMETER_MM
            ),
            "vendor_installed_tolerance_bound": False,
            "selected": False,
        },
        "torsion_pair": {
            "material": "17-7PH CH900 envelope; unselected",
            "wire_diameter_mm": TORSION_WIRE_DIAMETER_MM,
            "mean_coil_diameter_mm": TORSION_MEAN_COIL_DIAMETER_MM,
            "active_coils_analytical": TORSION_ACTIVE_COILS,
            "visible_review_ring_count": TORSION_REVIEW_RING_COUNT,
            "coil_centers_y_mm": [-TORSION_COIL_Y_MM, TORSION_COIL_Y_MM],
            "indexed_holes_per_fixed_anchor": 6,
            "indexed_anchor_selected": False,
            "manufacturing_helix_authority": False,
        },
        "radial_cartridge": {
            "catalog_number": CARTRIDGE_CATALOG_NUMBER,
            "coil_OD_mm": CARTRIDGE_COIL_OD_MM,
            "coil_width_mm": CARTRIDGE_COIL_WIDTH_MM,
            "fixed_center": list(CARTRIDGE_CENTER),
            "containment_bounds_mm": CONTAINMENT_BOUNDS_MM,
            "reduction_output_radius_mm": output_radius,
            "ratio_hole_targets": list(LEVER_RATIO_HOLES),
            "input_hole_radii_mm": [
                ratio * output_radius for ratio in LEVER_RATIO_HOLES
            ],
            "fragment_containment_qualified": False,
            "selected": False,
        },
        "body_contract": {
            "custom_center_body_count": len(custom_center),
            "all_center_body_count": len(all_center),
            "all_custom_center_bodies_single_solid": all(
                len(part.solids()) == 1 and float(part.volume) > 0.0
                for part in custom_center
            ),
            "all_center_parts_positive_volume": all(
                float(part.volume) > 0.0 for part in all_center
            ),
            "overlap_and_bounds_by_state": overlap_by_state,
            "all_same_state_custom_overlap_checks_pass": all(
                row["custom"]["status"] == "PASS"
                for row in overlap_by_state.values()
            ),
            "all_same_state_all_part_overlap_checks_pass": all(
                row["all_parts"]["status"] == "PASS"
                for row in overlap_by_state.values()
            ),
            "all_bounds_checks_pass": all(
                row["bounds"]["containment_matches_declared_bounds"]
                and row["bounds"]["cartridge_inside_service_pocket"]
                and row["bounds"]["containment_inside_service_pocket"]
                and row["bounds"]["assembly_inside_global_review_bounds"]
                for row in overlap_by_state.values()
            ),
        },
        "source_evidence": {
            "analytical_screen": (
                "sim/aggregate_boundary_follower_custom_return_screen.py"
            ),
            "procurement_no_go": (
                "sim/aggregate_boundary_follower_retraction_procurement.py"
            ),
            "main_follower_context": (
                "cad/aggregate_boundary_floating_follower.py"
            ),
            "main_sources_edited_by_this_prototype": False,
        },
        "catalog_sources": {
            "igus_WPFFM_0304_05": (
                "https://www.igus.com/iglide-ibh/flange-bearings/"
                "product-details/iglide-w300pf-m"
            ),
            "McMaster_9293K122": (
                "https://www.mcmaster.com/products/constant-force-springs/"
            ),
        },
        "inspection_refs": {
            "shaft_occurrence": "#o1.13",
            "shaft_cylindrical_face": "#o1.13.f1",
            "shaft_negative_end_face": "#o1.13.f2",
            "shaft_positive_end_face": "#o1.13.f3",
            "igus_occurrence": "#o1.14",
            "igus_ID_cylindrical_face": "#o1.14.f4",
            "igus_body_OD_cylindrical_face": "#o1.14.f5",
            "cartridge_containment_occurrence": "#o1.8",
            "reduction_lever_occurrence": "#o1.9",
            "cartridge_envelope_occurrence": "#o1.15",
        },
        "authority": dict(AUTHORITY),
        "decision": (
            "The package is suitable only for visual and deterministic CAD "
            "review. Every custom/COTS label remains unselected and all load, "
            "fatigue, procurement, integration, BOM, order, production and "
            "release authorities remain false."
        ),
    }


def gen_step() -> Compound:
    """Return the centered one-occurrence review assembly."""

    assembly = Compound(children=list(all_parts(REFERENCE_TANGENTIAL_STATE)))
    assembly.label = "aggregate_boundary_follower_custom_return_packaging_review_only"
    return assembly


def write_manifest() -> Path:
    REVIEW.mkdir(parents=True, exist_ok=True)
    MANIFEST.write_text(
        json.dumps(geometry_contract(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return MANIFEST


if __name__ == "__main__":
    print(write_manifest())
