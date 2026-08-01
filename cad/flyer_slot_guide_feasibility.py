"""Feasibility gate for replacing the invalid flyer-to-tooth free span.

The current moving-wire model uses one straight segment from the flyer torus
to a support tangent on a smooth coil ellipse.  That ellipse is a conservative
radially extruded collision envelope, not necessarily an exposed surface in
the real 24-tooth lamination.  At a captured witness pose its chosen endpoint
is inside bare steel.

This module does not invent a new route.  It:

* reproduces the exact OpenCascade core/coil penetration witness;
* calculates the available wire-centre corridor through the bare, lined, and
  currently capped 1.5339 mm slot mouth for the maximum Ø0.5 mm wire;
* derives minimum dimensions for a >=3 mm bend-radius guide; and
* records the smallest credible changeover concept and every proof it still
  lacks.

No CAD is emitted because the candidate is not yet geometrically authorized.
The report is deliberately ``DESIGN_CHANGE_REQUIRED``, never a release PASS.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np
from build123d import Pos, Rot, Vector


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SIM = ROOT / "sim"
OUT = ROOT / "out"
REPORTS = OUT / "reports"
MANIFEST = OUT / "links" / "manifest.json"
WIREPATH_REPORT = REPORTS / "wirepath.json"
ROUTE_REPORT = REPORTS / "slot_wire_routes.json"
JSON_OUT = REPORTS / "flyer_slot_guide_feasibility.json"
MD_OUT = REPORTS / "flyer_slot_guide_feasibility.md"

if str(SIM) not in sys.path:
    sys.path.insert(0, str(SIM))

import coil_growth  # noqa: E402
from params import DEFAULT_STATOR, PARAMS  # noqa: E402
import stator_insulation_nomex410 as insulation  # noqa: E402
import stator_model  # noqa: E402
import wirepath  # noqa: E402


SCHEMA = "flyer-slot-guide-feasibility/v1"
MAX_WIRE_RADIUS_MM = 0.25
MAX_WIRE_DIAMETER_MM = 2.0 * MAX_WIRE_RADIUS_MM
MIN_WIRE_CENTER_BEND_RADIUS_MM = 3.0
# This is a required design allowance, not a claim about a selected nozzle.
NOZZLE_DIAMETRAL_RUNNING_CLEARANCE_MM = 0.10
ACTIVE_SHOE_BLADE_TARGET_MM = 0.25
WITNESS_ANGLE_DEG = 26.0
WITNESS_MOTION_SIGN = -1
WITNESS_DEPTH_INDEX = 7


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: dict[str, Any]) -> str:
    canonical = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(canonical.encode()).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _stator_local_from_spindle(point: np.ndarray) -> list[float]:
    """Invert assembly.py's installed M1=0 stator transform."""

    # assembly.py maps local +X -> machine -Z, local +Y -> machine -X,
    # and local +Z -> machine +Y, centered at world/spindle Z=home_standoff.
    return [
        float(PARAMS.m0_home_standoff - point[2]),
        float(-point[0]),
        float(point[1]),
    ]


def _installed(shape):
    return Pos(0.0, 0.0, PARAMS.m0_home_standoff) * (
        Rot(0.0, 90.0, 0.0) * (Rot(-90.0, 0.0, 0.0) * shape)
    )


def penetration_witness(manifest: dict[str, Any]) -> dict[str, Any]:
    """Reproduce the exact core/coil witness with OCC point classification."""

    wire = manifest["wire"]
    contact = wire["tooth_contact"]
    guide = wire["tip_guide"]
    depths = np.linspace(
        *map(float, contact["insertion_depth_range_mm"]), 9)
    depth = float(depths[WITNESS_DEPTH_INDEX])
    angle = math.radians(WITNESS_ANGLE_DEG)
    flyer_rotation = wirepath.rot_z(angle)
    feed = flyer_rotation @ np.asarray(guide["feed_local_mm"], dtype=float)
    guide_center = flyer_rotation @ np.asarray(
        guide["center_local_mm"], dtype=float)
    lay = wirepath.tooth_contact_point(
        guide_center, contact, WITNESS_MOTION_SIGN)
    path, guide_meta = wirepath.tip_guide_path(
        feed, lay, guide, MAX_WIRE_RADIUS_MM, flyer_rotation)
    samples = wirepath._trim_sampled_polyline(
        path, start_mm=0.5, end_mm=0.75, spacing=0.25)

    dz = (DEFAULT_STATOR.od / 2.0 - depth
          - float(manifest["m0_home_standoff"]))
    spindle_points = samples - np.array((0.0, 0.0, dz))
    spindle_endpoint = lay - np.array((0.0, 0.0, dz))

    core = _installed(stator_model.stator(DEFAULT_STATOR))
    coils = coil_growth.coil_collision_envelopes(DEFAULT_STATOR)
    # The first obstructing neighboring envelope in the reported witness is
    # tooth 2.  It is checked separately so the combined spindle label cannot
    # hide whether the bare core itself is already fatal.
    coil_2 = _installed(coils[2])

    def inside_indices(shape) -> list[int]:
        return [
            index for index, point in enumerate(spindle_points)
            if shape.is_inside(Vector(*map(float, point)), tolerance=1e-7)
        ]

    core_inside = inside_indices(core)
    coil_inside = inside_indices(coil_2)
    endpoint_inside = core.is_inside(
        Vector(*map(float, spindle_endpoint)), tolerance=1e-7)

    first_index = core_inside[0] if core_inside else None
    last_index = core_inside[-1] if core_inside else None
    first_point = (
        _stator_local_from_spindle(spindle_points[first_index])
        if first_index is not None else None)
    last_point = (
        _stator_local_from_spindle(spindle_points[last_index])
        if last_index is not None else None)

    return {
        "depth_index": WITNESS_DEPTH_INDEX,
        "depth_mm": depth,
        "flyer_angle_deg": WITNESS_ANGLE_DEG,
        "motion_sign": WITNESS_MOTION_SIGN,
        "wire_radius_mm": MAX_WIRE_RADIUS_MM,
        "sample_spacing_max_mm": 0.25,
        "sample_count": len(spindle_points),
        "core_inside_sample_count": len(core_inside),
        "core_inside_index_span": [first_index, last_index],
        "neighbor_tooth_2_envelope_inside_sample_count": len(coil_inside),
        "neighbor_tooth_2_inside_index_span": [
            coil_inside[0] if coil_inside else None,
            coil_inside[-1] if coil_inside else None,
        ],
        "constructed_lay_endpoint_inside_bare_core": bool(endpoint_inside),
        "first_core_inside_stator_local_mm": first_point,
        "last_core_inside_stator_local_mm": last_point,
        "constructed_endpoint_stator_local_mm": (
            _stator_local_from_spindle(spindle_endpoint)),
        "tip_guide_arc_turn_deg": float(guide_meta["arc_turn_deg"]),
        "classification": (
            "TRUE_CORE_PENETRATION" if core_inside or endpoint_inside
            else "NO_CORE_PENETRATION"),
        "interpretation": (
            "The conservative final-copper envelope is not the sole cause: "
            "the current straight support route and its endpoint enter bare "
            "steel, which is present in every progressive winding state."
        ),
    }


def mouth_and_guide_budget() -> dict[str, Any]:
    geometry = coil_growth.slot_geometry(DEFAULT_STATOR)
    bare = float(geometry["opening_width_mm"])
    liner_max = float(insulation.MATERIAL_RECEIVING_MAX_MM)
    cap_overlap = float(insulation.CAP_EDGE_OVERLAP_MM)
    lined = bare - 2.0 * liner_max
    capped = bare - 2.0 * cap_overlap
    controlling = min(lined, capped)

    current_cap_center_corridor = controlling - MAX_WIRE_DIAMETER_MM
    liner_only_center_corridor = lined - MAX_WIRE_DIAMETER_MM
    current_cap_wall_each = (
        controlling - MAX_WIRE_DIAMETER_MM
        - NOZZLE_DIAMETRAL_RUNNING_CLEARANCE_MM
    ) / 2.0
    liner_only_wall_each = (
        lined - MAX_WIRE_DIAMETER_MM
        - NOZZLE_DIAMETRAL_RUNNING_CLEARANCE_MM
    ) / 2.0

    pitch = 2.0 * math.pi / DEFAULT_STATOR.slots
    inner_radius = float(geometry["shoe_inner_radius_mm"])
    outer_radius = float(geometry["outer_radius_mm"])
    external_surface_radius = (
        MIN_WIRE_CENTER_BEND_RADIUS_MM - MAX_WIRE_RADIUS_MM)
    external_pin_diameter = 2.0 * external_surface_radius
    concave_groove_root_radius = (
        MIN_WIRE_CENTER_BEND_RADIUS_MM + MAX_WIRE_RADIUS_MM)
    inner_pitch_chord = 2.0 * inner_radius * math.sin(pitch / 2.0)
    outer_pitch_chord = 2.0 * outer_radius * math.sin(pitch / 2.0)
    minimum_pin_center_radius = (
        external_pin_diameter / (2.0 * math.sin(pitch / 2.0)))

    contact = coil_growth.analyze_job(DEFAULT_STATOR)["bundle"]
    radial_span = (
        float(contact["radial_winding_end_mm"])
        - float(contact["radial_winding_start_mm"])
    )
    blade_residual = (
        lined - MAX_WIRE_DIAMETER_MM
        - NOZZLE_DIAMETRAL_RUNNING_CLEARANCE_MM
        - ACTIVE_SHOE_BLADE_TARGET_MM)

    return {
        "slot": {
            "bare_mouth_mm": bare,
            "liner_receiving_max_each_mm": liner_max,
            "lined_mouth_mm": lined,
            "existing_cap_overlap_each_mm": cap_overlap,
            "existing_cap_mouth_mm": capped,
            "controlling_current_mouth_mm": controlling,
        },
        "maximum_wire": {
            "radius_mm": MAX_WIRE_RADIUS_MM,
            "diameter_mm": MAX_WIRE_DIAMETER_MM,
            "diametral_running_clearance_design_input_mm": (
                NOZZLE_DIAMETRAL_RUNNING_CLEARANCE_MM),
        },
        "wire_center_corridor": {
            "with_current_cap_mm": current_cap_center_corridor,
            "with_liner_only_mm": liner_only_center_corridor,
        },
        "enclosed_nozzle_wall_budget": {
            "maximum_symmetric_wall_each_with_current_cap_mm": (
                current_cap_wall_each),
            "maximum_symmetric_wall_each_with_liner_only_mm": (
                liner_only_wall_each),
            "current_cap_release_status": (
                "NO_GO_UNQUALIFIED_ULTRATHIN_WALL"
                if current_cap_wall_each < 0.15 else "GEOMETRIC_ROOM_ONLY"),
            "reason": (
                "The current cap leaves only the stated wall budget after "
                "the maximum wire and 0.10 mm running allowance. No material, "
                "supplier, tolerance, or durability qualification exists for "
                "that ultrathin closed nozzle."
            ),
        },
        "three_mm_bend_geometry": {
            "minimum_external_guide_surface_radius_mm": (
                external_surface_radius),
            "minimum_external_pin_diameter_mm": external_pin_diameter,
            "minimum_concave_groove_root_radius_mm": (
                concave_groove_root_radius),
            "shoe_inner_pitch_chord_mm": inner_pitch_chord,
            "stator_od_pitch_chord_mm": outer_pitch_chord,
            "pin_gap_at_shoe_inner_radius_mm": (
                inner_pitch_chord - external_pin_diameter),
            "pin_gap_at_stator_od_mm": (
                outer_pitch_chord - external_pin_diameter),
            "minimum_24_fold_pin_center_radius_mm": (
                minimum_pin_center_radius),
            "existing_nomex_axial_flare_mm": (
                insulation.AXIAL_END_FLARE_MM),
            "minimum_quarter_turn_axial_projection_mm": (
                MIN_WIRE_CENTER_BEND_RADIUS_MM),
            "existing_flare_projection_shortfall_mm": (
                MIN_WIRE_CENTER_BEND_RADIUS_MM
                - insulation.AXIAL_END_FLARE_MM),
        },
        "active_shoe_candidate": {
            "architecture": (
                "machine-fixed split active-tooth shoe; existing M0 retracts "
                "the stator before M1 indexes the next tooth"),
            "one_blade_per_adjacent_slot_target_thickness_mm": (
                ACTIVE_SHOE_BLADE_TARGET_MM),
            "liner_only_residual_lateral_budget_mm": blade_residual,
            "maximum_combined_lateral_error_for_centered_budget_mm": (
                blade_residual / 2.0),
            "minimum_polished_radial_working_span_mm": radial_span,
            "minimum_external_horn_surface_radius_mm": (
                external_surface_radius),
            "minimum_concave_horn_root_radius_mm": (
                concave_groove_root_radius),
            "minimum_axial_horn_projection_mm": (
                MIN_WIRE_CENTER_BEND_RADIUS_MM),
            "cap_change": (
                "locally relieve/replace the 0.35 mm inward star-cap overlap "
                "at the two active mouths; the qualified shoe/liner must "
                "retain continuous steel-edge insulation"),
            "status": "CONCEPT_ONLY_NOT_PROVEN",
        },
    }


def analyze(*, include_occ_witness: bool = True) -> dict[str, Any]:
    manifest = _load(MANIFEST)
    wirepath_report = _load(WIREPATH_REPORT)
    route_report = _load(ROUTE_REPORT)
    budget = mouth_and_guide_budget()
    witness = (
        penetration_witness(manifest) if include_occ_witness else None)

    failures = [
        "current torus-to-lay endpoint is inside the exact bare stator core",
        "current capped mouth cannot accept a qualified conventional closed nozzle for the maximum wire",
        "no exact exposed multi-contact path exists for the proposed active shoe",
        "proposed shoe insertion/extraction has not been checked against progressive copper",
        "proposed shoe body has not been swept against the flyer, chuck, or indexing motion",
        "physical error budget and contact material/finish remain unqualified",
    ]
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "DESIGN_CHANGE_REQUIRED",
        "release_authorized": False,
        "scope": {
            "stator": "default 24-slot OD46 x stack15 simplified stator",
            "wire_radius_mm": MAX_WIRE_RADIUS_MM,
            "flyer_angles_required_deg": [0.0, 360.0],
            "lay_depth_count_required": 9,
            "progressive_copper_required": True,
            "minimum_free_guide_bend_radius_mm": (
                MIN_WIRE_CENTER_BEND_RADIUS_MM),
        },
        "current_artifacts": {
            "manifest_sha256": _sha256(MANIFEST),
            "wirepath_report_sha256": _sha256(WIREPATH_REPORT),
            "wirepath_status": "FAIL" if wirepath_report.get("fail") else "PASS",
            "wirepath_failure": wirepath_report.get("fail", []),
            "crossing_route_file_sha256": _sha256(ROUTE_REPORT),
            "crossing_route_report_sha256": route_report.get("report_sha256"),
            "crossing_route_status": route_report.get("status"),
        },
        "exact_penetration_witness": witness,
        "mouth_and_guide_budget": budget,
        "candidate": budget["active_shoe_candidate"],
        "required_minimal_cad_changes": [
            "Add a separately replaceable, machine-fixed split winding shoe on a rigid datum; do not merge it into printed.py.",
            "Use one thin polished dielectric blade in each slot flanking the active tooth and >=3 mm center-radius front/rear horns outside the lamination faces.",
            "Create a local active-mouth relief variant of the Nomex star caps while preserving continuous overlap with the slot-cell liners.",
            "Place the shoe so M0 fully withdraws both blades before every M1 indexing or shaft-wrap move using the existing controller motion.",
            "Export the shoe as its own collision part and replace the single support ellipse with torus-to-horn, horn contact, lined-mouth, and progressive-placement route segments.",
        ],
        "proofs_required_before_cad_release": [
            "Exact 360 degree x 9 depth x both direction path proof against bare core, liners, shoe, all prior turns, and the already-laid current half-turn.",
            "Positive swept clearance/error budget for M0/M1/M2 tracking, runout, measured wire, liner, guide placement, and guide wear.",
            "Insertion and radial extraction proof showing the shoe never crosses or traps deposited copper.",
            ">=3 mm centerline curvature at every horn/groove contact with continuous tangent transitions.",
            "Flyer/chuck/indexing collision sweep with the complete guide body and mount.",
            "Material, surface finish, dielectric, wear, heat, and enamel-abrasion coupon qualification.",
        ],
        "failures": failures,
        "decision": (
            "Do not edit the current collision exclusions. A passive exposed "
            "multi-contact route may be feasible only with a new active-tooth "
            "changeover shoe; the present geometry cannot prove it."
        ),
        "source_hashes": {
            "cad/flyer_slot_guide_feasibility.py": _sha256(Path(__file__)),
            "cad/coil_growth.py": _sha256(HERE / "coil_growth.py"),
            "cad/stator_model.py": _sha256(HERE / "stator_model.py"),
            "cad/stator_insulation_nomex410.py": _sha256(
                HERE / "stator_insulation_nomex410.py"),
            "sim/wirepath.py": _sha256(SIM / "wirepath.py"),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def write(report: dict[str, Any], json_path: Path = JSON_OUT,
          md_path: Path = MD_OUT) -> None:
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    budget = report["mouth_and_guide_budget"]
    slot = budget["slot"]
    nozzle = budget["enclosed_nozzle_wall_budget"]
    bend = budget["three_mm_bend_geometry"]
    witness = report.get("exact_penetration_witness") or {}
    lines = [
        "# Flyer slot-guide feasibility",
        "",
        f"Status: **{report['status']}**. Hardware authorization remains false.",
        "",
        "## Exact blocker",
        "",
        (f"At depth {witness.get('depth_mm', 'not run')} mm, flyer "
         f"{witness.get('flyer_angle_deg', 'not run')} deg, the current path "
         f"has {witness.get('core_inside_sample_count', 'not run')} sampled "
         "centres inside bare steel. The constructed lay endpoint is also "
         f"inside: {witness.get('constructed_lay_endpoint_inside_bare_core', 'not run')}."),
        "",
        "## Mouth budget for maximum wire",
        "",
        f"- Bare mouth: {slot['bare_mouth_mm']:.6f} mm.",
        f"- Lined mouth at 0.140 mm per wall: {slot['lined_mouth_mm']:.6f} mm.",
        f"- Current star-cap mouth: {slot['existing_cap_mouth_mm']:.6f} mm.",
        (f"- Maximum symmetric enclosed-nozzle wall with current cap, 0.5 mm diameter "
         f"wire, and 0.10 mm running allowance: "
         f"{nozzle['maximum_symmetric_wall_each_with_current_cap_mm']:.6f} mm."),
        "",
        "## Minimum guide geometry",
        "",
        f"- External polished guide radius: >= {bend['minimum_external_guide_surface_radius_mm']:.3f} mm (OD >= {bend['minimum_external_pin_diameter_mm']:.3f} mm).",
        f"- Concave groove root radius: >= {bend['minimum_concave_groove_root_radius_mm']:.3f} mm.",
        f"- Axial horn projection: >= {bend['minimum_quarter_turn_axial_projection_mm']:.3f} mm; existing Nomex flare is only {bend['existing_nomex_axial_flare_mm']:.3f} mm.",
        f"- Polished radial working span: >= {report['candidate']['minimum_polished_radial_working_span_mm']:.6f} mm.",
        "",
        "## Decision",
        "",
        report["decision"],
        "",
        "## Required CAD changes",
        "",
    ]
    lines.extend(f"- {item}" for item in report[
        "required_minimal_cad_changes"])
    lines.extend(("", "## Proofs still required", ""))
    lines.extend(f"- {item}" for item in report[
        "proofs_required_before_cad_release"])
    lines.extend(("", f"Report SHA-256: `{report['report_sha256']}`", ""))
    md_path.write_text("\n".join(lines))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", type=Path, default=JSON_OUT)
    parser.add_argument("--markdown", type=Path, default=MD_OUT)
    parser.add_argument("--skip-occ-witness", action="store_true")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    report = analyze(include_occ_witness=not args.skip_occ_witness)
    if not args.check:
        write(report, args.json, args.markdown)
        print(f"wrote {args.json} and {args.markdown}")
    print(
        f"slot guide {report['status']}: current cap nozzle wall budget "
        f"{report['mouth_and_guide_budget']['enclosed_nozzle_wall_budget']['maximum_symmetric_wall_each_with_current_cap_mm']:.6f} mm/side"
    )
    if report["release_authorized"]:
        raise SystemExit("unexpected release authorization")


if __name__ == "__main__":
    main()
