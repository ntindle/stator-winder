"""Historical fail-closed trade for a stock NBK P30 D10 flyer pulley.

This source never modifies or re-exports either CC-BY-ND NBK STEP.  The exact
official D10 file is imported byte-for-byte as a read-only assembly occurrence
for interface checks.  The only STEP written by this module is the proposed
custom 6061-T6 hollow shaft, clearly labeled review-only.

The trade exists because the former flyer-side P30 solid was only a smooth
collision envelope.  The safer tooth authority is the purchased stock
P30-3GT-BLP-6C-10 product.  It requires a local OD10 shaft seat.  Since its
D10 bore passes through the complete 18.5 mm pulley envelope, an OD10 neck
limited to the nominal 7.5 mm clamp hub is physically impossible.

The conditional L80.75 shaft proposal is retained as study history only.  It
is non-governing and superseded by released M2-001 Rev D L79.00.
"""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from build123d import (
    Align,
    Box,
    CenterOf,
    Cone,
    Cylinder,
    GeomType,
    Part,
    Pos,
    Rot,
    export_step,
    fillet,
    import_step,
)
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder

import custom_parts
import flyer_shaft_d10
import integrated_release_candidate as candidate


HISTORICAL_NON_GOVERNING = True
SUPERSEDED_BY = "M2-001 Rev D L79.00"


ROOT = Path(__file__).resolve().parents[1]
HERE = Path(__file__).resolve().parent
SOURCE_D10_STEP = (
    HERE / "models" / "upgrades" / "NBK_P30_D10_download" /
    "P30-3GT-BLP-6C-10.stp"
)
SOURCE_D10_SHA256 = (
    "780110e1d59a988661f5ae80e9ebbe5d2eb324b9037d33a481809c939fa4c9f1"
)
SOURCE_D10_BYTES = 57130
SOURCE_D10_ORDER_ID = "22026071121383341311079d0b6156e"
SOURCE_D10_EXPRESSION = "{CN=P30-3GT-BLP-6C},{D=10}"
SOURCE_PRODUCT_URL = (
    "https://www.nbk1560.com/products/pulley/timingpulley/"
    "3GT-BLP-6C/P30-3GT-BLP-6C/"
)

REPORT = ROOT / "out" / "reports" / "flyer_p30_stock_d10_trade.json"
REPORT_MD = ROOT / "out" / "reports" / "flyer_p30_stock_d10_trade.md"
SHAFT_STEP = ROOT / "out" / "review" / "flyer_shaft_stock_d10_review_only.step"

# Frozen placement facts from the current integrated candidate.
TOOTH_MIDPLANE_Z_MM = -97.75
PULLEY_REAR_Z_MM = -110.75
PULLEY_FRONT_Z_MM = -92.25
PULLEY_AXIAL_LENGTH_MM = 18.5
STOCK_CLAMP_LENGTH_MM = 7.5
CURRENT_SHAFT_REAR_Z_MM = -110.0
PROPOSED_SHAFT_REAR_Z_MM = PULLEY_REAR_Z_MM
SHAFT_FRONT_Z_MM = -30.0
PROPOSED_SHAFT_LENGTH_MM = SHAFT_FRONT_Z_MM - PROPOSED_SHAFT_REAR_Z_MM
NECK_FRONT_Z_MM = PULLEY_FRONT_Z_MM
NECK_LENGTH_MM = NECK_FRONT_Z_MM - PROPOSED_SHAFT_REAR_Z_MM
ENTRY_PRIOR_REAR_SHIFT_MM = 3.5
ENTRY_ADDITIONAL_REAR_SHIFT_MM = 0.75
ENTRY_TOTAL_REAR_SHIFT_MM = ENTRY_PRIOR_REAR_SHIFT_MM + ENTRY_ADDITIONAL_REAR_SHIFT_MM

NECK_OD_MM = 10.0
NECK_ID_MM = 6.0
MAIN_OD_MM = 12.0
MAIN_ID_MM = 9.0
INTERNAL_TRANSITION_LENGTH_MM = 3.0
INTERNAL_TRANSITION_END_Z_MM = NECK_FRONT_Z_MM + INTERNAL_TRANSITION_LENGTH_MM
REAR_BEARING_START_Z_MM = -85.0
WIRE_DIAMETER_MAX_MM = 0.5
SHAFT_DENSITY_G_MM3 = 2.70 / 1000.0

# Current provisional M2 load contract.  The differential belt force below
# does not include installed belt pretension, which remains a physical gate.
REQUIRED_OUTPUT_TORQUE_NM = 0.3641973829292742
REQUIRED_2X_OUTPUT_TORQUE_NM = 0.7283947658585485
PITCH_DIAMETER_MM = 28.7
NEAREST_BEARING_FACE_Z_MM = -85.0

OFFICIAL_MASS_G = 28.0
OFFICIAL_AXIAL_J_KGM2 = 3.0e-6
OFFICIAL_CLAMP_BOLT = "M2 socket-head clamp bolt"
OFFICIAL_CLAMP_TORQUE_NM = 0.5
OFFICIAL_SHAFT_TOLERANCE = "h6 or h7 recommended by NBK"

CTR = (Align.CENTER, Align.CENTER, Align.CENTER)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def official_d10() -> Part:
    """Import the exact stock D10 occurrence without modifying it."""

    if _sha256(SOURCE_D10_STEP) != SOURCE_D10_SHA256:
        raise RuntimeError("official NBK D10 STEP hash drift")
    if SOURCE_D10_STEP.stat().st_size != SOURCE_D10_BYTES:
        raise RuntimeError("official NBK D10 STEP byte count drift")
    part = import_step(str(SOURCE_D10_STEP))
    if len(part.solids()) != 1 or not part.is_valid:
        raise RuntimeError("official NBK D10 must import as one valid solid")
    return part


def placed_official_d10_hub_rear() -> Part:
    """Place source +X toward machine -Z at the frozen tooth midplane."""

    source = official_d10()
    placed = Pos(0.0, 0.0, TOOTH_MIDPLANE_Z_MM) * (
        Rot(0.0, 0.0, candidate.NBK_P30_STOCK_ROLL_DEG)
        * (Rot(0.0, 90.0, 0.0) * source)
    )
    placed.label = "official_NBK_P30_3GT_BLP_6C_10_hub_rear_review_occurrence"
    return placed


def proposed_necked_shaft() -> Part:
    """Return the conditional L80.75 OD10/ID6-to-OD12/ID9 shaft.

    The D10 seat spans the complete stock through-bore.  A coaxial three-mm
    cone opens ID6 to ID9 before the rear bearing.  The wire centerline stays
    straight through this transition, so its analytical bend radius is
    infinite; the surface still requires a polished supplier inspection.
    """

    neck = Pos(0.0, 0.0, PROPOSED_SHAFT_REAR_Z_MM) * Cylinder(
        NECK_OD_MM / 2.0,
        NECK_LENGTH_MM,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    main = Pos(0.0, 0.0, NECK_FRONT_Z_MM) * Cylinder(
        MAIN_OD_MM / 2.0,
        SHAFT_FRONT_Z_MM - NECK_FRONT_Z_MM,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    outer = neck + main

    neck_bore = Pos(0.0, 0.0, PROPOSED_SHAFT_REAR_Z_MM - 1.0) * Cylinder(
        NECK_ID_MM / 2.0,
        NECK_LENGTH_MM + 1.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    transition = Pos(0.0, 0.0, NECK_FRONT_Z_MM) * Cone(
        NECK_ID_MM / 2.0,
        MAIN_ID_MM / 2.0,
        INTERNAL_TRANSITION_LENGTH_MM,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    main_bore = Pos(0.0, 0.0, INTERNAL_TRANSITION_END_Z_MM) * Cylinder(
        MAIN_ID_MM / 2.0,
        SHAFT_FRONT_Z_MM - INTERNAL_TRANSITION_END_Z_MM + 1.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    part = outer - neck_bore - transition - main_bore

    # Retain only the two existing arm-clamp flats at 64 mm from the old rear
    # datum (-110 -> world z=-46).  The stock split clamp requires a round D10
    # seat, so the two obsolete flyer-P30 set-screw flats are deliberately gone.
    arm_station_z = -46.0
    minus_y = Pos(0.0, -8.2, arm_station_z) * Box(20.0, 5.0, 5.0, align=CTR)
    plus_x = Pos(8.2, 0.0, arm_station_z) * Box(5.0, 20.0, 5.0, align=CTR)
    part = part - minus_y - plus_x

    rear_inner_edges = [
        edge for edge in part.edges().filter_by(GeomType.CIRCLE)
        if math.isclose(edge.length, math.pi * NECK_ID_MM, abs_tol=1.0e-5)
        and math.isclose(
            edge.center().Z, PROPOSED_SHAFT_REAR_Z_MM, abs_tol=1.0e-6
        )
    ]
    if len(rear_inner_edges) != 1:
        raise RuntimeError(
            f"expected one rear ID6 mouth edge, got {len(rear_inner_edges)}"
        )
    part = fillet(rear_inner_edges, 0.50)
    part.label = "REVIEW_ONLY_flyer_shaft_L80p75_OD10_ID6_to_OD12_ID9"
    if len(part.solids()) != 1 or not part.is_valid:
        raise RuntimeError("proposed D10 shaft must remain one valid solid")
    return part


def _axial_volume_moment(shape: Part, axis: int) -> float:
    center = shape.center(CenterOf.MASS)
    matrix = shape.matrix_of_inertia
    if axis == 0:
        offset_sq = center.Y ** 2 + center.Z ** 2
    elif axis == 2:
        offset_sq = center.X ** 2 + center.Y ** 2
    else:  # pragma: no cover - only principal shaft axes are used
        raise ValueError(axis)
    return float(matrix[axis][axis]) + float(shape.volume) * offset_sq


def _bbox(shape: Part) -> dict[str, list[float]]:
    box = shape.bounding_box()
    return {
        "min_mm": [float(box.min.X), float(box.min.Y), float(box.min.Z)],
        "max_mm": [float(box.max.X), float(box.max.Y), float(box.max.Z)],
        "size_mm": [float(box.size.X), float(box.size.Y), float(box.size.Z)],
    }


def _cylindrical_faces(part: Part) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for face in part.faces():
        adaptor = BRepAdaptor_Surface(face.wrapped)
        if adaptor.GetType() != GeomAbs_Cylinder:
            continue
        cylinder = adaptor.Cylinder()
        direction = cylinder.Axis().Direction()
        location = cylinder.Axis().Location()
        box = face.bounding_box()
        rows.append({
            "radius_mm": float(cylinder.Radius()),
            "axis": [direction.X(), direction.Y(), direction.Z()],
            "location_mm": [location.X(), location.Y(), location.Z()],
            "bbox_min_mm": [box.min.X, box.min.Y, box.min.Z],
            "bbox_max_mm": [box.max.X, box.max.Y, box.max.Z],
        })
    return rows


def _distance(a: Part, b: Part) -> float:
    return float(a.distance_to(b))


def _overlap(a: Part, b: Part) -> float:
    try:
        return float((a & b).volume)
    except Exception:
        return 0.0


def evaluate() -> dict[str, Any]:
    source_mtime = SOURCE_D10_STEP.stat().st_mtime_ns
    source_hash_before = _sha256(SOURCE_D10_STEP)
    official = official_d10()
    pulley = placed_official_d10_hub_rear()
    shaft = proposed_necked_shaft()
    current_shaft = candidate.shaft_with_integrated_p30_flats()
    current_pulley = candidate.integrated_base_rotating_parts()["flyer_pulley"]

    static = candidate.main_static_groups()
    block = candidate._find(static["shifted_support"], "flyer_block")
    rear_bearing = candidate._find(static["shifted_support"], "flyer_6001_rear")
    current_entry = candidate._find(static["shifted_entry"], "entry_bracket")
    current_eyelet = candidate._find(static["shifted_entry"], "entry_eyelet")
    shifted_entry = Pos(0.0, 0.0, -ENTRY_ADDITIONAL_REAR_SHIFT_MM) * current_entry
    shifted_eyelet = Pos(0.0, 0.0, -ENTRY_ADDITIONAL_REAR_SHIFT_MM) * current_eyelet

    wire = Pos(0.0, 0.0, PROPOSED_SHAFT_REAR_Z_MM - 2.0) * Cylinder(
        WIRE_DIAMETER_MAX_MM / 2.0,
        SHAFT_FRONT_Z_MM - PROPOSED_SHAFT_REAR_Z_MM + 4.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )

    official_moment = _axial_volume_moment(official, 0)
    current_pulley_moment = _axial_volume_moment(current_pulley, 2)
    current_shaft_moment = _axial_volume_moment(current_shaft, 2)
    proposed_shaft_moment = _axial_volume_moment(shaft, 2)

    stress_od_mm = flyer_shaft_d10.NECK_OD_H6_LIMITS_MM[0]
    stress_id_mm = flyer_shaft_d10.NECK_ID_LIMITS_MM[1]
    neck_j_mm4 = math.pi / 32.0 * (stress_od_mm ** 4 - stress_id_mm ** 4)
    neck_i_mm4 = neck_j_mm4 / 2.0
    pitch_radius_mm = PITCH_DIAMETER_MM / 2.0
    differential_belt_force_n = (
        REQUIRED_2X_OUTPUT_TORQUE_NM * 1000.0 / pitch_radius_mm
    )
    bearing_lever_mm = NEAREST_BEARING_FACE_Z_MM - TOOTH_MIDPLANE_Z_MM
    bending_moment_nmm = differential_belt_force_n * bearing_lever_mm
    torsion_nmm = REQUIRED_2X_OUTPUT_TORQUE_NM * 1000.0
    torsion_mpa = torsion_nmm * (stress_od_mm / 2.0) / neck_j_mm4
    bending_mpa = bending_moment_nmm * (stress_od_mm / 2.0) / neck_i_mm4
    von_mises_mpa = math.sqrt(bending_mpa ** 2 + 3.0 * torsion_mpa ** 2)

    cylinders = _cylindrical_faces(official)
    bore_faces = [
        row for row in cylinders
        if math.isclose(row["radius_mm"], 5.0, abs_tol=1.0e-6)
        and abs(row["axis"][0]) >= 0.999999
    ]
    tooth_envelope_faces = [
        row for row in cylinders
        if math.isclose(row["radius_mm"], 13.95, abs_tol=1.0e-6)
        and abs(row["axis"][0]) >= 0.999999
    ]

    current_pulley_mass_g = current_pulley.volume * SHAFT_DENSITY_G_MM3
    current_pulley_j = current_pulley_moment * SHAFT_DENSITY_G_MM3 * 1.0e-9
    current_shaft_mass_g = current_shaft.volume * SHAFT_DENSITY_G_MM3
    proposed_shaft_mass_g = shaft.volume * SHAFT_DENSITY_G_MM3
    current_shaft_j = current_shaft_moment * SHAFT_DENSITY_G_MM3 * 1.0e-9
    proposed_shaft_j = proposed_shaft_moment * SHAFT_DENSITY_G_MM3 * 1.0e-9

    current_no_extension = {
        "hub_insertion_mm": STOCK_CLAMP_LENGTH_MM - 0.75,
        "hub_insertion_fraction": (STOCK_CLAMP_LENGTH_MM - 0.75) /
        STOCK_CLAMP_LENGTH_MM,
        "pulley_to_entry_bracket_mm": _distance(pulley, current_entry),
        "pulley_to_entry_eyelet_mm": _distance(pulley, current_eyelet),
        "release_clearance_target_mm": candidate.MIN_REVIEW_CLEARANCE_MM,
        "passes": False,
    }
    conditional = {
        "shaft_span_z_mm": [PROPOSED_SHAFT_REAR_Z_MM, SHAFT_FRONT_Z_MM],
        "shaft_length_mm": PROPOSED_SHAFT_LENGTH_MM,
        "hub_insertion_mm": STOCK_CLAMP_LENGTH_MM,
        "hub_insertion_fraction": 1.0,
        "entry_total_rear_shift_mm": ENTRY_TOTAL_REAR_SHIFT_MM,
        "pulley_to_shifted_entry_bracket_mm": _distance(pulley, shifted_entry),
        "pulley_to_shifted_entry_eyelet_mm": _distance(pulley, shifted_eyelet),
        "shaft_to_shifted_entry_bracket_mm": _distance(shaft, shifted_entry),
        "shaft_to_shifted_entry_eyelet_mm": _distance(shaft, shifted_eyelet),
        "pulley_to_flyer_block_mm": _distance(pulley, block),
        "pulley_to_rear_bearing_mm": _distance(pulley, rear_bearing),
        "pulley_vs_flyer_block_mm3": _overlap(pulley, block),
        "pulley_vs_rear_bearing_mm3": _overlap(pulley, rear_bearing),
        "shaft_vs_pulley_mm3": _overlap(shaft, pulley),
        "shaft_to_pulley_distance_mm": _distance(shaft, pulley),
        "transition_end_to_rear_bearing_margin_mm": (
            REAR_BEARING_START_Z_MM - INTERNAL_TRANSITION_END_Z_MM
        ),
        "geometry_screen_passes": all((
            _distance(pulley, shifted_entry) >= candidate.MIN_REVIEW_CLEARANCE_MM,
            _distance(pulley, block) >= candidate.MIN_REVIEW_CLEARANCE_MM,
            _overlap(pulley, rear_bearing) <= candidate.BOOLEAN_TOL_MM3,
            _overlap(pulley, block) <= candidate.BOOLEAN_TOL_MM3,
            REAR_BEARING_START_Z_MM - INTERNAL_TRANSITION_END_Z_MM >= 3.0,
        )),
        "release_authorized": False,
    }

    source_hash_after = _sha256(SOURCE_D10_STEP)
    if source_hash_after != source_hash_before:
        raise RuntimeError("official D10 STEP changed during evaluation")
    if SOURCE_D10_STEP.stat().st_mtime_ns != source_mtime:
        raise RuntimeError("official D10 STEP mtime changed during evaluation")

    result: dict[str, Any] = {
        "schema": "flyer-p30-stock-d10-trade/v1",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "status": "CONDITIONAL_GEOMETRY_PASS_RELEASE_BLOCKED",
        "production_authorized": False,
        "candidate_integration_authorized": False,
        "historical_non_governing": HISTORICAL_NON_GOVERNING,
        "superseded_by": SUPERSEDED_BY,
        "decision": (
            "Historical conditional proposal: prefer the genuine stock NBK "
            "D10 pulley over a custom-tooth pulley with the review-only "
            "L80.75 shaft and a 0.75 mm rear entry-module shift. Do not "
            "integrate this shaft; M2-001 Rev D L79.00 supersedes it."
        ),
        "official_product": {
            "part_number": "P30-3GT-BLP-6C-10",
            "url": SOURCE_PRODUCT_URL,
            "standard_stock_d10": True,
            "teeth": 30,
            "pitch_mm": 3.0,
            "belt_width_mm": 6.0,
            "bore_mm": 10.0,
            "shaft_tolerance": OFFICIAL_SHAFT_TOLERANCE,
            "clamp_bolt": OFFICIAL_CLAMP_BOLT,
            "clamp_torque_nm": OFFICIAL_CLAMP_TORQUE_NM,
            "mass_g": OFFICIAL_MASS_G,
            "axial_j_kgm2": OFFICIAL_AXIAL_J_KGM2,
            "body_material": "A2017",
            "bolt_material": "SCM435 black oxide",
            "source_step": SOURCE_D10_STEP.relative_to(ROOT).as_posix(),
            "source_step_sha256": SOURCE_D10_SHA256,
            "source_step_bytes": SOURCE_D10_BYTES,
            "cadenas_order_id": SOURCE_D10_ORDER_ID,
            "cadenas_expression": SOURCE_D10_EXPRESSION,
            "source_step_remained_byte_identical": source_hash_after == SOURCE_D10_SHA256,
            "step_single_solid": len(official.solids()) == 1,
            "step_face_count": len(official.faces()),
            "step_volume_mm3": float(official.volume),
            "step_axial_volume_moment_mm5": official_moment,
            "step_bbox": _bbox(official),
            "exact_bore_face_count": len(bore_faces),
            "cad_tooth_topology_explicit": False,
            "cad_tooth_envelope": {
                "cylindrical_face_count": len(tooth_envelope_faces),
                "radius_mm": 13.95,
                "axial_width_mm": 7.3,
                "note": (
                    "The downloaded interface STEP uses one smooth cylindrical "
                    "tooth-band envelope. Purchased-product identity, not the "
                    "STEP surface, is tooth-form manufacturing authority."
                ),
            },
        },
        "placement": {
            "tooth_midplane_z_mm": TOOTH_MIDPLANE_Z_MM,
            "orientation": "official source +X mapped to machine -Z (hub rear)",
            "exact_pulley_bbox": _bbox(pulley),
            "current_no_extension": current_no_extension,
            "conditional_full_engagement": conditional,
        },
        "shaft": {
            "artifact": SHAFT_STEP.relative_to(ROOT).as_posix(),
            "artifact_role": "review_only_not_release",
            "material": "6061-T6 custom machined hollow shaft",
            "single_solid": len(shaft.solids()) == 1,
            "valid": bool(shaft.is_valid),
            "bbox": _bbox(shaft),
            "od10_id6_neck_span_z_mm": [
                PROPOSED_SHAFT_REAR_Z_MM, NECK_FRONT_Z_MM
            ],
            "neck_length_mm": NECK_LENGTH_MM,
            "why_not_7p5mm_only": (
                "The stock D10 bore is through the complete 18.5 mm pulley; "
                "OD12 cannot pass through the toothed/flanged portion."
            ),
            "nominal_neck_radial_wall_mm": (NECK_OD_MM - NECK_ID_MM) / 2.0,
            "drawing_limit_OD_h6_mm": list(
                flyer_shaft_d10.NECK_OD_H6_LIMITS_MM
            ),
            "drawing_limit_ID_mm": list(flyer_shaft_d10.NECK_ID_LIMITS_MM),
            "minimum_neck_radial_wall_at_limits_mm": (
                flyer_shaft_d10.MIN_NECK_RADIAL_WALL_AT_LIMITS_MM
            ),
            "id6_to_id9_transition_span_z_mm": [
                NECK_FRONT_Z_MM, INTERNAL_TRANSITION_END_Z_MM
            ],
            "wire_centerline_bend_radius_mm": "infinite_straight_coaxial",
            "minimum_wire_to_shaft_wall_clearance_mm": _distance(wire, shaft),
            "rear_id6_mouth_fillet_mm": 0.5,
            "arm_flats_retained": 2,
            "obsolete_p30_set_screw_flats_removed": 2,
            "current_mass_g": current_shaft_mass_g,
            "proposed_mass_g": proposed_shaft_mass_g,
            "mass_delta_g": proposed_shaft_mass_g - current_shaft_mass_g,
            "current_axial_j_kgm2": current_shaft_j,
            "proposed_axial_j_kgm2": proposed_shaft_j,
            "axial_j_delta_kgm2": proposed_shaft_j - current_shaft_j,
        },
        "shaft_load_screen": {
            "basis": (
                "2x required output torque at minimum OD h6 and maximum ID "
                "drawing limits; belt differential force only. "
                "Installed pretension, shoulder Kt and fatigue remain unmeasured."
            ),
            "section_OD_mm": stress_od_mm,
            "section_ID_mm": stress_id_mm,
            "neck_polar_second_moment_mm4": neck_j_mm4,
            "neck_planar_second_moment_mm4": neck_i_mm4,
            "two_x_torque_nm": REQUIRED_2X_OUTPUT_TORQUE_NM,
            "pitch_radius_mm": pitch_radius_mm,
            "belt_differential_force_n": differential_belt_force_n,
            "tooth_midplane_to_nearest_bearing_face_mm": bearing_lever_mm,
            "bending_moment_nmm": bending_moment_nmm,
            "torsional_shear_mpa": torsion_mpa,
            "bending_stress_mpa": bending_mpa,
            "combined_von_mises_mpa_without_Kt_or_pretension": von_mises_mpa,
            "screen_result": "LOW_NOMINAL_STRESS_BUT_PHYSICAL_GATE_REQUIRED",
        },
        "mass_and_balance_delta": {
            "current_smooth_flyer_pulley_mass_g_at_2p70": current_pulley_mass_g,
            "official_stock_d10_mass_g": OFFICIAL_MASS_G,
            "pulley_mass_delta_g": OFFICIAL_MASS_G - current_pulley_mass_g,
            "current_smooth_flyer_pulley_axial_j_kgm2": current_pulley_j,
            "official_stock_d10_axial_j_kgm2": OFFICIAL_AXIAL_J_KGM2,
            "pulley_axial_j_delta_kgm2": OFFICIAL_AXIAL_J_KGM2 - current_pulley_j,
            "exact_balance_solution_still_valid": False,
            "reason": (
                "Stock split-clamp and supplied M2 bolt replace the custom "
                "two-M3 hub. Delivered radial COM and bolt clocking must be "
                "measured before the six trims are re-solved."
            ),
        },
        "release_blockers": [
            "Entry bracket/eyelet and integral dancer-anchor keeper are not yet redesigned at the -4.25 mm datum.",
            "Full raw-cycle collision, phase-aware wire, attachment and continuous-conductor gates have not been rerun.",
            "At the time of this study, the L80.75 OD10/ID6 necked shaft had no released drawing, supplier quote, dimensional inspection or polished-wire coupon; it is now retired and non-governing.",
            "Installed stock-clamp reversing slip, belt pretension, shoulder fatigue/stress concentration and hot endurance are unmeasured.",
            "The frozen six-trim balance solution predates the official 28 g D10 pulley and supplied M2 clamp bolt.",
        ],
    }
    return result


def _write_markdown(report: dict[str, Any]) -> None:
    no_extension = report["placement"]["current_no_extension"]
    conditional = report["placement"]["conditional_full_engagement"]
    load = report["shaft_load_screen"]
    delta = report["mass_and_balance_delta"]
    lines = [
        "# Stock NBK P30 D10 flyer-pulley trade",
        "",
        f"Status: **{report['status']}**. Production and candidate integration remain blocked.",
        "",
        "The exact stock `P30-3GT-BLP-6C-10` is the preferred tooth authority. "
        "The official D10 STEP remains byte-identical and is never re-exported.",
        "",
        "## Current placement fails",
        "",
        f"- Hub insertion is {no_extension['hub_insertion_mm']:.2f}/7.50 mm.",
        f"- Pulley-to-entry clearance is {no_extension['pulley_to_entry_bracket_mm']:.2f} mm versus the {no_extension['release_clearance_target_mm']:.2f} mm gate.",
        "",
        "## Conditional geometry",
        "",
        f"- Extend the rear of the shaft 0.75 mm: z={PROPOSED_SHAFT_REAR_Z_MM:.2f}..{SHAFT_FRONT_Z_MM:.2f}, L={PROPOSED_SHAFT_LENGTH_MM:.2f} mm.",
        f"- OD10/ID6 must span the full {NECK_LENGTH_MM:.2f} mm pulley bore; shoulder to OD12 at z={NECK_FRONT_Z_MM:.2f}.",
        f"- Transition ID6 to ID9 over 3.00 mm; it ends {conditional['transition_end_to_rear_bearing_margin_mm']:.2f} mm before the rear bearing.",
        f"- Shift the complete entry module another 0.75 mm rearward to total -{ENTRY_TOTAL_REAR_SHIFT_MM:.2f} mm.",
        f"- Resulting pulley-entry clearance {conditional['pulley_to_shifted_entry_bracket_mm']:.2f} mm; block {conditional['pulley_to_flyer_block_mm']:.2f} mm; bearing {conditional['pulley_to_rear_bearing_mm']:.2f} mm.",
        "",
        "## Load and balance screen",
        "",
        f"- OD10/ID6 nominal stress at the 2x torque screen: bending {load['bending_stress_mpa']:.3f} MPa, torsion {load['torsional_shear_mpa']:.3f} MPa, von Mises {load['combined_von_mises_mpa_without_Kt_or_pretension']:.3f} MPa.",
        f"- Official pulley mass delta versus the smooth candidate is {delta['pulley_mass_delta_g']:+.3f} g and axial-J delta {delta['pulley_axial_j_delta_kgm2']:+.6e} kg m^2.",
        "- These values invalidate the frozen balance trim solution; delivered COM and clamp-bolt clocking must be measured.",
        "",
        "## Blockers",
        "",
        *[f"- {item}" for item in report["release_blockers"]],
        "",
    ]
    REPORT_MD.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    SHAFT_STEP.parent.mkdir(parents=True, exist_ok=True)
    report = evaluate()
    shaft = proposed_necked_shaft()
    export_step(shaft, str(SHAFT_STEP))
    report["shaft"]["artifact_sha256"] = _sha256(SHAFT_STEP)
    report["shaft"]["artifact_bytes"] = SHAFT_STEP.stat().st_size
    REPORT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    _write_markdown(report)
    print(REPORT)
    print(REPORT_MD)
    print(SHAFT_STEP)
    print(report["status"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
