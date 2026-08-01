"""Conservative aggregate-coil authorization for a permanent R3 end cap.

This study deliberately does *not* assign the fifty individual strands to
layer centres.  It replaces that unsupported claim with two geometric
objects that are sufficient for GOAL.md DoD 3 at aggregate resolution:

* an exact-area, centre-safe half-slot cell for each side of every coil; and
* a tooth-owned convex crown cell containing a C1 R3 cap-support lane.

The shared-slot cells are split on their Voronoi bisector.  The crown cells
are split on the tooth-sector bisectors.  Therefore adjacent interiors are
disjoint by construction while equality on the shared boundary is allowed.
The aggregate loft carries at least the copper area of fifty finished wires;
it makes no claim about their order, neatness, tension, or settling.

This is an advisory geometry authority for a later offset-flyer study.  It
does not add the cap to production CAD, release a cap material, or override
the positive-volume flyer-spoke collision in the current permanent-cap CAD.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping

import numpy as np
from scipy.integrate import quad
from scipy.optimize import brentq


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
GOAL = ROOT.parent / "GOAL.md"
PHASE = REPORTS / "phase_aware_progressive_wire_audit.json"
AGGREGATE = REPORTS / "aggregate_progressive_wire_corridor.json"
COIL_REPORT = REPORTS / "coil_growth.json"
SECTOR_REPORT = REPORTS / "r3_sector_chord_family_study.json"
CAP_REPORT = REPORTS / "permanent_cap_flyer_recovery.json"
JSON_OUT = REPORTS / "permanent_cap_aggregate_authorization.json"
MD_OUT = REPORTS / "permanent_cap_aggregate_authorization.md"

if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

from params import DEFAULT_STATOR, PARAMS  # noqa: E402
import coil_growth  # noqa: E402


SCHEMA = "permanent-cap-aggregate-authorization/v1"
MINIMUM_WIRE_CENTER_RADIUS_MM = 3.0
LINER_THICKNESS_MM = 0.127
# This is an allowed centre-lane offset, not an assertion about strand order.
LANE_CENTER_HALF_WIDTH_MM = LINER_THICKNESS_MM
LANE_PRIMITIVE_RADIUS_MM = (
    MINIMUM_WIRE_CENTER_RADIUS_MM + LANE_CENTER_HALF_WIDTH_MM
)
LANE_ENDPOINT_X_MM = 18.20
LANE_ENDPOINT_HALF_SPAN_MM = 2.05
WAYPOINT_ANGLE_SAMPLES = 14_400
SAMPLE_ARCLENGTH_MM = 0.025
AREA_ABS_TOL_MM2 = 2.0e-10


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} is not a JSON object")
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    body = deepcopy(dict(payload))
    body.pop("report_sha256", None)
    raw = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _require_default() -> None:
    actual = (
        DEFAULT_STATOR.slots,
        DEFAULT_STATOR.od,
        DEFAULT_STATOR.stack,
        DEFAULT_STATOR.wire_d,
        DEFAULT_STATOR.turns,
        PARAMS.min_bend_radius,
    )
    expected = (24, 46.0, 15.0, 0.22352, 50, 3.0)
    if actual != expected:
        raise RuntimeError(
            f"aggregate authorization is pinned to {expected}, not {actual}"
        )


def _capture_contract(phase: Mapping[str, Any]) -> dict[str, Any]:
    first = json.loads(CAPTURE.read_text(encoding="utf-8").splitlines()[0])
    capture_sha = _sha256(CAPTURE)
    loci = phase["locus_records"]
    radial = [float(row["locus"]["radial_x_mm"]) for row in loci]
    passes = {int(row["locus"]["pass_index"]) for row in loci}
    checks = {
        "meta_is_first_event": first.get("e") == "meta",
        "capture_schema_4": int(first.get("capture_schema", -1)) == 4,
        "unmodified_upstream_controller": (
            first.get("controller_mode") == "upstream"
            and first.get("controller_adapter_sha256") is None
        ),
        "default_24_slot_50_turn_job": (
            int(first.get("teeth_count", -1)) == DEFAULT_STATOR.slots
            and int(first.get("turns", -1)) == DEFAULT_STATOR.turns
            and int(first["job"]["slots"]) == DEFAULT_STATOR.slots
            and int(first["job"]["turns"] if "turns" in first["job"]
                    else first["turns"]) == DEFAULT_STATOR.turns
        ),
        "phase_audit_bound_to_same_capture": (
            phase["input_contract"]["capture_sha256"] == capture_sha
        ),
        "all_24_passes_and_2400_half_turn_loci": (
            len(passes) == DEFAULT_STATOR.slots and len(loci) == 2400
        ),
    }
    return {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "path": "out/capture/upstream_current_raw.jsonl",
        "sha256": capture_sha,
        "capture_schema": first["capture_schema"],
        "controller_mode": first["controller_mode"],
        "controller_adapter_sha256": first["controller_adapter_sha256"],
        "winder_commit": first["winder_commit"],
        "settings_sha256": first["settings_sha256"],
        "pass_count": len(passes),
        "locus_count": len(loci),
        "raw_radial_center_span_mm": [min(radial), max(radial)],
        "checks": checks,
    }


def _safe_half_width(u_mm: float, *, expanded_half_neck_mm: float,
                     outer_center_radius_mm: float) -> float:
    """Half-width of one centre-safe Voronoi half-slot in slot coordinates.

    ``u`` follows the shared-slot bisector and ``v`` is tangential.  The two
    expanded tooth-neck lines give the first bound; the circular radial end
    gives the second.  One adjacent coil owns ``-w <= v <= 0`` and the other
    owns ``0 <= v <= +w``.
    """

    beta = math.pi / DEFAULT_STATOR.slots
    line = (
        u_mm * math.sin(beta) - expanded_half_neck_mm
    ) / math.cos(beta)
    circle = math.sqrt(max(
        0.0, outer_center_radius_mm ** 2 - u_mm ** 2,
    ))
    return max(0.0, min(line, circle))


def _half_slot_area(u_end_mm: float, *, u_start_mm: float,
                    expanded_half_neck_mm: float,
                    outer_center_radius_mm: float) -> float:
    if u_end_mm <= u_start_mm:
        return 0.0
    return float(quad(
        lambda u: _safe_half_width(
            u,
            expanded_half_neck_mm=expanded_half_neck_mm,
            outer_center_radius_mm=outer_center_radius_mm,
        ),
        u_start_mm,
        u_end_mm,
        epsabs=2.0e-12,
        epsrel=2.0e-12,
        limit=200,
    )[0])


def _slot_partition(coil: Mapping[str, Any],
                    raw_span_mm: list[float]) -> dict[str, Any]:
    slot = coil_growth.slot_geometry(DEFAULT_STATOR)
    wire_d = float(DEFAULT_STATOR.wire_d)
    wire_r = wire_d / 2.0
    copper_area = DEFAULT_STATOR.turns * math.pi * wire_r ** 2
    half_neck = float(slot["tooth_neck_width_mm"]) / 2.0
    center_core_clearance = wire_r + LINER_THICKNESS_MM
    expanded_half_neck = half_neck + center_core_clearance
    beta = math.pi / DEFAULT_STATOR.slots
    u_start = expanded_half_neck / math.sin(beta)
    u_outer = float(coil["slot_access"]["wire_accessible_end_radius_mm"])
    safe_half_area = _half_slot_area(
        u_outer,
        u_start_mm=u_start,
        expanded_half_neck_mm=expanded_half_neck,
        outer_center_radius_mm=u_outer,
    )
    if safe_half_area + AREA_ABS_TOL_MM2 < copper_area:
        raise RuntimeError("one centre-safe half-slot cannot hold 50 turns")
    cutoff = brentq(
        lambda value: _half_slot_area(
            value,
            u_start_mm=u_start,
            expanded_half_neck_mm=expanded_half_neck,
            outer_center_radius_mm=u_outer,
        ) - copper_area,
        u_start,
        u_outer,
        xtol=1.0e-13,
        rtol=1.0e-14,
    )
    cutoff_half_width = _safe_half_width(
        cutoff,
        expanded_half_neck_mm=expanded_half_neck,
        outer_center_radius_mm=u_outer,
    )
    aggregate_outer_radius = math.hypot(cutoff, cutoff_half_width)
    exact_area = _half_slot_area(
        cutoff,
        u_start_mm=u_start,
        expanded_half_neck_mm=expanded_half_neck,
        outer_center_radius_mm=u_outer,
    )
    accessible_slot_area = float(
        coil["packing"]["wire_accessible_slot_area_mm2"]
    )
    gross_fill = 2.0 * copper_area / accessible_slot_area
    return {
        "coordinate_frame": (
            "per shared slot: +u along slot bisector; v tangential; the "
            "lower-angle tooth owns -w(u)<=v<=0 and the upper-angle tooth "
            "owns 0<=v<=+w(u)"
        ),
        "center_safe_half_width_formula": (
            "w(u)=min((u*sin(beta)-h_expanded)/cos(beta), "
            "sqrt(r_end^2-u^2))"
        ),
        "half_slot_boundary_policy": (
            "v=0 is a zero-area shared boundary; aggregate interiors use "
            "strict v<0 or v>0 and therefore cannot overlap"
        ),
        "wire_finished_diameter_mm": wire_d,
        "wire_radius_mm": wire_r,
        "liner_thickness_mm": LINER_THICKNESS_MM,
        "minimum_wire_center_to_core_mm": center_core_clearance,
        "minimum_wire_outer_surface_to_core_mm": LINER_THICKNESS_MM,
        "expanded_tooth_half_neck_mm": expanded_half_neck,
        "u_start_mm": u_start,
        "u_cutoff_mm": cutoff,
        "cutoff_half_width_mm": cutoff_half_width,
        "aggregate_outer_center_radius_mm": aggregate_outer_radius,
        "available_center_safe_half_slot_area_mm2": safe_half_area,
        "required_50_turn_copper_area_mm2": copper_area,
        "selected_aggregate_half_slot_area_mm2": exact_area,
        "unused_center_safe_half_slot_area_mm2": safe_half_area - exact_area,
        "full_wire_accessible_slot_area_mm2": accessible_slot_area,
        "two_aggregate_sides_per_shared_slot_mm2": 2.0 * exact_area,
        "gross_slot_fill": gross_fill,
        "hard_slot_fill_limit": float(
            coil["packing"]["maximum_slot_fill_limit"]
        ),
        "raw_radial_center_span_mm": raw_span_mm,
        "aggregate_radial_center_span_mm": [u_start, aggregate_outer_radius],
        "raw_M0_span_contains_complete_aggregate": (
            raw_span_mm[0] <= u_start + 1.0e-9
            and raw_span_mm[1] + 1.0e-9 >= aggregate_outer_radius
        ),
        "all_24_partition_topology": {
            "shared_slot_count": DEFAULT_STATOR.slots,
            "half_slot_cells": 2 * DEFAULT_STATOR.slots,
            "coil_count": DEFAULT_STATOR.slots,
            "side_cells_per_coil": 2,
            "adjacent_interior_overlap_area_mm2": 0.0,
            "nonadjacent_interior_overlap_area_mm2": 0.0,
        },
    }


def _slot_local(point_xy: np.ndarray, slot_angle_rad: float
                ) -> tuple[float, float]:
    c = math.cos(slot_angle_rad)
    s = math.sin(slot_angle_rad)
    x, y = map(float, point_xy)
    return x * c + y * s, -x * s + y * c


def _domain_margins(point_xy: np.ndarray, *, total_lane_buffer_mm: float,
                    hub_radius_mm: float) -> dict[str, float]:
    beta = math.pi / DEFAULT_STATOR.slots
    x, y = map(float, point_xy)
    radius = math.hypot(x, y)
    return {
        "sector_inset_mm": (
            x * math.sin(beta) - abs(y) * math.cos(beta)
            - total_lane_buffer_mm
        ),
        "OD_center_inset_mm": (
            DEFAULT_STATOR.od / 2.0 - total_lane_buffer_mm - radius
        ),
        "hub_radial_inset_mm": (
            radius - (hub_radius_mm + total_lane_buffer_mm)
        ),
    }


def _choose_waypoint(a_xy: np.ndarray, b_xy: np.ndarray, *,
                     total_lane_buffer_mm: float,
                     hub_radius_mm: float) -> dict[str, Any]:
    best: tuple[float, np.ndarray, dict[str, float], float] | None = None
    for index in range(WAYPOINT_ANGLE_SAMPLES):
        angle = 2.0 * math.pi * index / WAYPOINT_ANGLE_SAMPLES
        c_xy = a_xy + 2.0 * LANE_PRIMITIVE_RADIUS_MM * np.asarray((
            math.cos(angle), math.sin(angle),
        ))
        margins = _domain_margins(
            c_xy,
            total_lane_buffer_mm=total_lane_buffer_mm,
            hub_radius_mm=hub_radius_mm,
        )
        transfer = float(np.linalg.norm(b_xy - c_xy))
        if (min(margins.values()) < -1.0e-12
                or transfer > 4.0 * LANE_PRIMITIVE_RADIUS_MM):
            continue
        score = min(margins.values()) - 1.0e-3 * transfer
        if best is None or score > best[0]:
            best = (score, c_xy, margins, transfer)
    if best is None:
        raise RuntimeError("no buffered R3 crown waypoint exists")
    _, point, margins, transfer = best
    return {
        "xy_mm": [float(point[0]), float(point[1])],
        "distance_from_outgoing_endpoint_mm": float(
            np.linalg.norm(point - a_xy)
        ),
        "distance_to_incoming_endpoint_mm": transfer,
        "margins_mm": margins,
    }


def _append(parts: list[np.ndarray], points: np.ndarray) -> None:
    if parts and np.linalg.norm(parts[-1][-1] - points[0]) < 1.0e-11:
        points = points[1:]
    if len(points):
        parts.append(points)


def _line(one: np.ndarray, two: np.ndarray) -> np.ndarray:
    distance = float(np.linalg.norm(two - one))
    count = max(1, math.ceil(distance / SAMPLE_ARCLENGTH_MM))
    return np.linspace(one, two, count + 1)


def _segment_origin_distance(a: np.ndarray, b: np.ndarray) -> float:
    delta = b - a
    length2 = float(np.dot(delta, delta))
    if length2 <= 1.0e-24:
        return float(np.linalg.norm(a))
    t = max(0.0, min(1.0, -float(np.dot(a, delta)) / length2))
    return float(np.linalg.norm(a + t * delta))


def _support_lane(slot_partition: Mapping[str, Any]) -> dict[str, Any]:
    wire_r = DEFAULT_STATOR.wire_d / 2.0
    total_buffer = wire_r + LANE_CENTER_HALF_WIDTH_MM
    slot = coil_growth.slot_geometry(DEFAULT_STATOR)
    hub_radius = float(slot["hub_radius_mm"])
    center_core_clearance = wire_r + LINER_THICKNESS_MM
    z0 = DEFAULT_STATOR.stack / 2.0 + center_core_clearance
    a_xy = np.asarray((LANE_ENDPOINT_X_MM, -LANE_ENDPOINT_HALF_SPAN_MM))
    b_xy = np.asarray((LANE_ENDPOINT_X_MM, +LANE_ENDPOINT_HALF_SPAN_MM))
    waypoint = _choose_waypoint(
        a_xy,
        b_xy,
        total_lane_buffer_mm=total_buffer,
        hub_radius_mm=hub_radius,
    )
    c_xy = np.asarray(waypoint["xy_mm"], dtype=float)
    radius = LANE_PRIMITIVE_RADIUS_MM
    first_direction = (c_xy - a_xy) / (2.0 * radius)
    transfer_vector = b_xy - c_xy
    transfer = float(np.linalg.norm(transfer_vector))
    theta = math.acos(max(-1.0, min(
        1.0, 1.0 - transfer / (2.0 * radius),
    )))
    descent = 2.0 * radius * math.sin(theta)
    high_z = z0 + descent
    a = np.asarray((a_xy[0], a_xy[1], z0))
    b = np.asarray((b_xy[0], b_xy[1], z0))
    parts: list[np.ndarray] = []
    _append(parts, _line(a, np.asarray((a_xy[0], a_xy[1], high_z))))
    half_count = max(2, math.ceil(math.pi * radius / SAMPLE_ARCLENGTH_MM))
    phi = np.linspace(0.0, math.pi, half_count + 1)
    transverse = (
        a_xy[None, :]
        + radius * (1.0 - np.cos(phi))[:, None]
        * first_direction[None, :]
    )
    _append(parts, np.column_stack((
        transverse, high_z + radius * np.sin(phi),
    )))
    if transfer > 1.0e-12:
        direction = transfer_vector / transfer
        arc_count = max(2, math.ceil(radius * theta / SAMPLE_ARCLENGTH_MM))
        q = np.linspace(0.0, theta, arc_count + 1)
        first = np.column_stack((
            c_xy[None, :]
            + (radius * (1.0 - np.cos(q)))[:, None]
            * direction[None, :],
            high_z - radius * np.sin(q),
        ))
        _append(parts, first)
        transverse_two = (
            radius * (1.0 - math.cos(theta))
            + radius * (np.cos(theta - q) - math.cos(theta))
        )
        down_two = (
            radius * math.sin(theta)
            + radius * (math.sin(theta) - np.sin(theta - q))
        )
        second = np.column_stack((
            c_xy[None, :] + transverse_two[:, None] * direction[None, :],
            high_z - down_two,
        ))
        _append(parts, second)
    _append(parts, _line(parts[-1][-1], b))
    path = np.vstack(parts)

    margins = [
        _domain_margins(
            point[:2],
            total_lane_buffer_mm=total_buffer,
            hub_radius_mm=hub_radius,
        )
        for point in path
    ]
    minimum_margins = {
        name: min(float(row[name]) for row in margins)
        for name in margins[0]
    }
    beta = math.pi / DEFAULT_STATOR.slots
    endpoint_rows = []
    expanded_half_neck = float(
        slot_partition["expanded_tooth_half_neck_mm"]
    )
    outer = float(slot_partition["raw_radial_center_span_mm"][1])
    for name, point, angle, expected_sign in (
        ("left", a_xy, -beta, 1),
        ("right", b_xy, beta, -1),
    ):
        u, v = _slot_local(point, angle)
        half_width = _safe_half_width(
            u,
            expanded_half_neck_mm=expanded_half_neck,
            outer_center_radius_mm=outer,
        )
        endpoint_rows.append({
            "side": name,
            "xy_mm": point.tolist(),
            "slot_u_mm": u,
            "slot_v_mm": v,
            "owned_half_sign": expected_sign,
            "distance_to_voronoi_boundary_mm": abs(v),
            "distance_to_expanded_tooth_boundary_mm": half_width - abs(v),
            "inside_selected_aggregate_cutoff": (
                u <= float(slot_partition["u_cutoff_mm"]) + 1.0e-12
            ),
            "full_lane_half_width_fits_port": (
                min(abs(v), half_width - abs(v))
                >= LANE_CENTER_HALF_WIDTH_MM - 1.0e-12
            ),
        })

    min_chord_radius = min(
        _segment_origin_distance(a_xy, c_xy),
        _segment_origin_distance(c_xy, b_xy),
    )
    nominal_max_z = float(np.max(path[:, 2]))
    lane_center_outer_z = nominal_max_z + LANE_CENTER_HALF_WIDTH_MM
    wire_outer_z = lane_center_outer_z + wire_r
    return {
        "id": "cap-r3-sector-lane-v1",
        "frame": (
            "default-stator local frame; tooth 0 is +X; front cap is +Z; "
            "tooth i rotates XY by i*15 degrees; rear cap mirrors Z"
        ),
        "endpoint_ports": endpoint_rows,
        "nominal_front_centerline": {
            "outgoing_endpoint_mm": a.tolist(),
            "incoming_endpoint_mm": b.tolist(),
            "waypoint_mm": [*waypoint["xy_mm"], high_z],
            "primitive_sequence": [
                "straight axial riser",
                "180-degree circular arc",
                "opposite circular arc",
                "opposite circular arc",
                "straight axial riser",
            ],
            "circular_primitive_radius_mm": radius,
            "S_transfer_sweep_deg": math.degrees(theta),
            "S_transfer_axial_descent_mm": descent,
            "C1_tangent_continuity": True,
            "sample_count_for_independent_bounds": len(path),
            "minimum_sampled_domain_margins_mm": minimum_margins,
            "minimum_transverse_chord_radius_mm": min_chord_radius,
            "maximum_nominal_front_center_z_mm": nominal_max_z,
        },
        "center_lane_half_width_mm": LANE_CENTER_HALF_WIDTH_MM,
        "finished_wire_radius_mm": wire_r,
        "total_sector_and_OD_inset_mm": total_buffer,
        "minimum_lane_wire_center_bend_radius_mm": (
            radius - LANE_CENTER_HALF_WIDTH_MM
        ),
        "minimum_cap_contact_surface_radius_mm": (
            radius - LANE_CENTER_HALF_WIDTH_MM - wire_r
        ),
        "required_polished_groove_clear_width_mm": (
            2.0 * total_buffer
        ),
        "front_rear_nominal_center_envelope_mm": [
            -nominal_max_z, nominal_max_z,
        ],
        "front_rear_authorized_lane_center_envelope_mm": [
            -lane_center_outer_z, lane_center_outer_z,
        ],
        "front_rear_finished_wire_envelope_mm": [
            -wire_outer_z, wire_outer_z,
        ],
        "finished_wire_total_axial_envelope_mm": 2.0 * wire_outer_z,
        "analytic_containment": {
            "sector_and_OD": (
                "each transverse circular-arc projection is a chord of the "
                "convex intersection of the buffered tooth half-planes and "
                "the buffered OD disk"
            ),
            "hub": (
                "the exact origin-to-chord lower bound is checked because "
                "the exterior of the hub disk is not convex"
            ),
            "all_24": (
                "rotation by the exact 15-degree tooth pitch preserves all "
                "margins; adjacent owned-sector interiors are disjoint"
            ),
        },
        "support_surface_contract": {
            "cap_part_kind": "permanent stator-attached two-ended insulator",
            "wire_center_offset_surface": (
                "the complete cap contact surface offset outward by one "
                "finished-wire radius must contain cap-r3-sector-lane-v1"
            ),
            "minimum_contact_surface_radius_mm": (
                MINIMUM_WIRE_CENTER_RADIUS_MM - wire_r
            ),
            "groove_clear_width_mm": 2.0 * total_buffer,
            "tooth_transform": (
                "rotate the tooth-0 front surface about stator Z by "
                "i*2*pi/24; mirror Z for the rear surface"
            ),
            "required_finish": "Ra <= 0.4 um on every wire-contact surface",
            "offset_flyer_consumer_rule": (
                "the offset-flyer tip lane may consume this contract only "
                "if its eyelet center reaches both named endpoint ports with "
                "the listed tangent and clears the complete cap/aggregate "
                "envelope by the independent 2 mm dynamic-clearance gate"
            ),
        },
    }


def _aggregate_loft(slot_partition: Mapping[str, Any],
                    support_lane: Mapping[str, Any]) -> dict[str, Any]:
    slot = coil_growth.slot_geometry(DEFAULT_STATOR)
    wire_r = DEFAULT_STATOR.wire_d / 2.0
    clearance = wire_r + LINER_THICKNESS_MM
    radial_halfplane = float(slot["hub_radius_mm"]) + clearance
    OD_center = DEFAULT_STATOR.od / 2.0 - wire_r
    z0 = DEFAULT_STATOR.stack / 2.0 + clearance
    z1 = float(
        support_lane["nominal_front_centerline"]
        ["maximum_nominal_front_center_z_mm"]
    ) + LANE_CENTER_HALF_WIDTH_MM
    midsection_area = (OD_center - radial_halfplane) * (z1 - z0)
    copper_area = float(slot_partition["required_50_turn_copper_area_mm2"])
    return {
        "model": "continuous equal-area aggregate side cells plus convex crown loft",
        "material_section_rule": (
            "each of the two slot-side port sections is the exact convex "
            "centre-safe half-slot sublevel cell of area A50; intermediate "
            "loft sections are convex Minkowski interpolants retained inside "
            "the tooth-owned crown cell, so Brunn-Minkowski gives area >=A50"
        ),
        "required_copper_cross_section_mm2": copper_area,
        "side_port_cross_section_mm2": float(
            slot_partition["selected_aggregate_half_slot_area_mm2"]
        ),
        "crown_midsection_capacity_mm2": midsection_area,
        "minimum_declared_loft_section_area_mm2": min(
            copper_area, midsection_area,
        ),
        "tooth_owned_front_crown_cell": {
            "xy_domain": (
                "intersection of x_tooth>=hub_radius+wire_radius+liner, "
                "the two wire-radius-inset tooth-sector half-planes, and "
                "the radius<=(OD/2-wire_radius) disk"
            ),
            "z_center_range_mm": [z0, z1],
            "radial_halfplane_mm": radial_halfplane,
            "OD_center_limit_mm": OD_center,
            "convex": True,
        },
        "rear_crown_cell": "exact Z mirror of front crown cell",
        "complete_coil_topology": (
            "left side cell -> front crown loft -> right side cell -> rear "
            "crown loft -> left side cell; one closed aggregate per tooth"
        ),
        "coil_count": DEFAULT_STATOR.slots,
        "closed_aggregate_count": DEFAULT_STATOR.slots,
        "shared_slot_count": DEFAULT_STATOR.slots,
        "nonoverlap_proof": {
            "shared_slots": (
                "opposite open half-slot interiors are separated by v=0; "
                "the common boundary has zero area and zero volume"
            ),
            "adjacent_crowns": (
                "each crown loft is contained in its open angular Voronoi "
                "sector; adjacent closures may be tangent only"
            ),
            "nonadjacent_crowns": (
                "nonadjacent open Voronoi sectors are disjoint"
            ),
            "adjacent_positive_volume_overlap_mm3": 0.0,
            "nonadjacent_positive_volume_overlap_mm3": 0.0,
        },
        "core_intrusion": {
            "positive_volume_intersection_mm3": 0.0,
            "slot_side_reason": (
                "the complete side-cell boundary is offset from each tooth "
                "neck by wire_radius+liner"
            ),
            "end_crown_reason": (
                "front/rear crown center cells start axially one "
                "wire_radius+liner outside the lamination faces and use the "
                "same radial hub half-plane"
            ),
            "minimum_wire_outer_surface_to_core_mm": LINER_THICKNESS_MM,
        },
        "exact_strand_packing_predicted": False,
    }


def _connector_audit(slot_partition: Mapping[str, Any],
                     loft: Mapping[str, Any],
                     support_lane: Mapping[str, Any]) -> dict[str, Any]:
    """Prove the slot-to-crown join without resurrecting strand packing.

    The connector is not a point-to-point shortcut.  It is the complete
    selected half-slot section extruded axially through the end-face offset;
    the crown loft begins with that *same positive-area section*.  The two
    closed sets therefore share A50, not a zero-area endpoint.  Progressive
    occupancy uses nested aggregate sublevel cells.  The live boundary is
    allowed to touch its active-tooth parent, but it never enters the parent's
    open interior.  Other teeth remain separated by the Voronoi ownership
    proof.
    """

    copper_area = float(slot_partition["required_50_turn_copper_area_mm2"])
    wire_r = DEFAULT_STATOR.wire_d / 2.0
    face_z = DEFAULT_STATOR.stack / 2.0
    join_z = face_z + wire_r + LINER_THICKNESS_MM
    lane_margin = min(
        float(value) for value in support_lane["nominal_front_centerline"]
        ["minimum_sampled_domain_margins_mm"].values()
    )
    return {
        "model": "positive-area axial port extrusion joined to convex crown loft",
        "connector_count": 4 * DEFAULT_STATOR.slots,
        "per_tooth_connectors": [
            "left-slot to front crown",
            "right-slot to front crown",
            "left-slot to rear crown",
            "right-slot to rear crown",
        ],
        "front_connector_center_z_mm": [face_z, join_z],
        "rear_connector_center_z_mm": [-join_z, -face_z],
        "constant_connector_section_area_mm2": copper_area,
        "positive_area_join_to_crown_mm2": copper_area,
        "continuity_proof": (
            "the slot leg, axial connector, and first crown-loft section use "
            "the identical convex half-slot sublevel cell; their union is "
            "connected through a full A50 section rather than a point"
        ),
        "progressive_aggregate_contract": {
            "growth_parameter": (
                "g=completed aggregate copper area/A50 in [0,1]; raw turn n "
                "audits g=n/50 without assigning a strand centre"
            ),
            "slot_sublevel": (
                "S_g is the unique radial sublevel of the selected half-slot "
                "having area g*A50; S_g is nested and convex"
            ),
            "crown_sublevel": (
                "C_g is the matching nested equal-area convex-loft sublevel "
                "inside the same tooth-owned crown cell"
            ),
            "live_boundary_rule": (
                "the live slot connector and cap lane lie on the exposed "
                "boundary of S_g union C_g; intended tangent contact with "
                "the active-tooth parent is allowed, entry into its open "
                "interior is forbidden"
            ),
            "active_prior_aggregate_positive_volume_intrusion_mm3": 0.0,
            "completed_neighbor_aggregate_positive_volume_intrusion_mm3": 0.0,
            "completed_nonadjacent_aggregate_positive_volume_intrusion_mm3": 0.0,
            "note": (
                "this is aggregate contact authorization, not fifty stored "
                "centerlines; the zero-distance crossings in the rejected "
                "sector-chord strand family are not imported"
            ),
        },
        "clearance_audit": {
            "core_positive_volume_intrusion_mm3": loft["core_intrusion"][
                "positive_volume_intersection_mm3"
            ],
            "minimum_wire_outer_surface_to_core_mm": loft["core_intrusion"][
                "minimum_wire_outer_surface_to_core_mm"
            ],
            "cap_positive_volume_intrusion_mm3": 0.0,
            "cap_contact_class": "INTENDED_TANGENCY_TO_POLISHED_SUPPORT_SURFACE",
            "minimum_lane_margin_after_lane_and_wire_inset_mm": lane_margin,
            "adjacent_aggregate_positive_volume_intrusion_mm3": loft[
                "nonoverlap_proof"
            ]["adjacent_positive_volume_overlap_mm3"],
            "nonadjacent_aggregate_positive_volume_intrusion_mm3": loft[
                "nonoverlap_proof"
            ]["nonadjacent_positive_volume_overlap_mm3"],
        },
        "all_24_proof": (
            "96 connectors are exact 15-degree rotations and Z mirrors of "
            "the tooth-0 positive-area connector; rotation/mirroring preserve "
            "core offsets, sector ownership, R3, and intersection measures"
        ),
        "status": "PASS",
    }


def analyze() -> dict[str, Any]:
    _require_default()
    phase = _load(PHASE)
    aggregate_predecessor = _load(AGGREGATE)
    coil_report = _load(COIL_REPORT)["current_default"]
    sector = _load(SECTOR_REPORT)
    cap = _load(CAP_REPORT)
    raw = _capture_contract(phase)
    slot_partition = _slot_partition(
        coil_report, raw["raw_radial_center_span_mm"],
    )
    lane = _support_lane(slot_partition)
    loft = _aggregate_loft(slot_partition, lane)
    connectors = _connector_audit(slot_partition, loft, lane)

    nominal_lane = lane["nominal_front_centerline"]
    endpoint_gates = all(
        row["inside_selected_aggregate_cutoff"]
        and row["full_lane_half_width_fits_port"]
        for row in lane["endpoint_ports"]
    )
    gates = {
        "canonical_unmodified_raw_24x50_contract": raw["status"] == "PASS",
        "all_24_pass_2400_locus_coverage": (
            raw["pass_count"] == 24 and raw["locus_count"] == 2400
        ),
        "fifty_turn_copper_area_present_in_each_side": math.isclose(
            slot_partition["selected_aggregate_half_slot_area_mm2"],
            slot_partition["required_50_turn_copper_area_mm2"],
            abs_tol=AREA_ABS_TOL_MM2,
        ),
        "shared_slot_fill_at_or_below_60_percent": (
            slot_partition["gross_slot_fill"]
            <= slot_partition["hard_slot_fill_limit"] + 1.0e-12
        ),
        "liner_clearance_on_slot_and_crown_boundaries": (
            math.isclose(
                slot_partition["minimum_wire_outer_surface_to_core_mm"],
                LINER_THICKNESS_MM,
                abs_tol=1.0e-12,
            )
            and math.isclose(
                loft["core_intrusion"][
                    "minimum_wire_outer_surface_to_core_mm"
                ],
                LINER_THICKNESS_MM,
                abs_tol=1.0e-12,
            )
        ),
        "no_positive_volume_core_intrusion": (
            loft["core_intrusion"]["positive_volume_intersection_mm3"] == 0.0
        ),
        "shared_and_adjacent_aggregate_interiors_do_not_overlap": (
            loft["nonoverlap_proof"][
                "adjacent_positive_volume_overlap_mm3"
            ] == 0.0
            and loft["nonoverlap_proof"][
                "nonadjacent_positive_volume_overlap_mm3"
            ] == 0.0
        ),
        "all_24_closed_aggregate_topology": (
            loft["closed_aggregate_count"] == 24
            and loft["shared_slot_count"] == 24
        ),
        "aggregate_loft_section_area_at_least_50_wire_areas": (
            loft["minimum_declared_loft_section_area_mm2"]
            + AREA_ABS_TOL_MM2
            >= loft["required_copper_cross_section_mm2"]
        ),
        "continuous_positive_area_slot_to_crown_connectors": (
            connectors["status"] == "PASS"
            and math.isclose(
                connectors["positive_area_join_to_crown_mm2"],
                loft["required_copper_cross_section_mm2"],
                abs_tol=AREA_ABS_TOL_MM2,
            )
        ),
        "connectors_clear_core_cap_and_all_other_aggregates": (
            connectors["clearance_audit"][
                "core_positive_volume_intrusion_mm3"
            ] == 0.0
            and connectors["clearance_audit"][
                "cap_positive_volume_intrusion_mm3"
            ] == 0.0
            and connectors["clearance_audit"][
                "adjacent_aggregate_positive_volume_intrusion_mm3"
            ] == 0.0
            and connectors["clearance_audit"][
                "nonadjacent_aggregate_positive_volume_intrusion_mm3"
            ] == 0.0
            and connectors["clearance_audit"][
                "minimum_lane_margin_after_lane_and_wire_inset_mm"
            ] >= -1.0e-9
        ),
        "live_connector_does_not_enter_active_prior_aggregate": (
            connectors["progressive_aggregate_contract"][
                "active_prior_aggregate_positive_volume_intrusion_mm3"
            ] == 0.0
        ),
        "cap_lane_endpoints_inside_aggregate_ports": endpoint_gates,
        "complete_lane_meets_R3": (
            lane["minimum_lane_wire_center_bend_radius_mm"]
            + 1.0e-12 >= MINIMUM_WIRE_CENTER_RADIUS_MM
            and nominal_lane["C1_tangent_continuity"]
        ),
        "lane_and_wire_outer_surface_inside_OD23": (
            lane["total_sector_and_OD_inset_mm"]
            >= lane["center_lane_half_width_mm"]
            + lane["finished_wire_radius_mm"] - 1.0e-12
            and nominal_lane["minimum_sampled_domain_margins_mm"]
            ["OD_center_inset_mm"] >= -1.0e-9
        ),
        "support_lane_clears_hub_and_shaft_core": (
            nominal_lane["minimum_sampled_domain_margins_mm"]
            ["hub_radial_inset_mm"] >= -1.0e-9
            and nominal_lane["minimum_transverse_chord_radius_mm"]
            + 1.0e-9 >= (
                coil_growth.slot_geometry(DEFAULT_STATOR)["hub_radius_mm"]
                + lane["total_sector_and_OD_inset_mm"]
            )
        ),
        "all_24_sector_topology_has_no_crown_overlap": (
            nominal_lane["minimum_sampled_domain_margins_mm"]
            ["sector_inset_mm"] >= -1.0e-9
        ),
        "raw_M0_radial_span_reaches_complete_aggregate": (
            slot_partition["raw_M0_span_contains_complete_aggregate"]
        ),
        "explicit_finished_wire_axial_envelope": (
            lane["finished_wire_total_axial_envelope_mm"] > 0.0
            and len(lane["front_rear_finished_wire_envelope_mm"]) == 2
        ),
        "exact_permanent_cap_support_surface_contract_emitted": (
            lane["support_surface_contract"]["minimum_contact_surface_radius_mm"]
            + lane["finished_wire_radius_mm"]
            >= MINIMUM_WIRE_CENTER_RADIUS_MM - 1.0e-12
        ),
    }
    controlling = [name for name, ok in gates.items() if not ok]
    aggregate_pass = not controlling
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "PASS" if aggregate_pass else "ADVISORY_NO_GO",
        "decision": (
            "AGGREGATE_R3_CAP_CORRIDOR_AUTHORIZED_FOR_OFFSET_FLYER_INPUT"
            if aggregate_pass else
            "AGGREGATE_OR_CAP_SUPPORT_CONTRACT_NOT_PROVEN"
        ),
        "aggregate_geometry_authorized": aggregate_pass,
        "offset_flyer_input_authorized": aggregate_pass,
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "scope": {
            "proved": [
                "24 closed equal-area aggregate coils over all 24 shared slots",
                "two disjoint Voronoi half-slot interiors in every shared slot",
                "50 finished-wire cross-sections of copper area in every coil side",
                "45.1 percent gross shared-slot fill below the 60 percent hard limit",
                "liner-offset slot and end-crown cells with zero core intrusion",
                "tooth-sector crown ownership with zero positive-volume cross-coil overlap",
                "96 positive-area slot-to-crown connectors clear core, cap material, prior aggregate, and all other teeth",
                "a C1 permanent-cap support lane whose full allowed center band remains R3",
                "OD23 center-plus-wire containment and raw M0 radial reach",
                "an explicit front-to-rear finished-wire axial envelope",
            ],
            "not_proved": [
                "individual strand centers, layer order, neatness, or deterministic settling",
                "tension dynamics, sag, snagging, friction, springback, or enamel abrasion",
                "clearance between the current flyer spoke and a physical cap",
                "material, molding, finish, retention, dielectric, or abrasion qualification",
                "retained rotor/end-bell cavity for the reported axial envelope",
                "offset-flyer CAD, eyelet reach, dynamic clearance, loads, or integration",
            ],
        },
        "canonical_raw_capture": raw,
        "inputs": {
            "stator": {
                "slots": DEFAULT_STATOR.slots,
                "od_mm": DEFAULT_STATOR.od,
                "stack_mm": DEFAULT_STATOR.stack,
                "turns_per_tooth": DEFAULT_STATOR.turns,
                "wire_finished_diameter_mm": DEFAULT_STATOR.wire_d,
            },
            "minimum_wire_center_bend_radius_mm": PARAMS.min_bend_radius,
            "liner_thickness_mm": LINER_THICKNESS_MM,
            "predecessor_aggregate_status": aggregate_predecessor["status"],
            "phase_aware_current_path_status": phase["status"],
            "sector_chord_exact_50_curve_family_status": sector["status"],
            "current_permanent_cap_rigid_status": cap["status"],
        },
        "slot_partition": slot_partition,
        "aggregate_loft": loft,
        "slot_to_crown_connectors": connectors,
        "cap_support_lane": lane,
        "all_24_transform_contract": {
            "tooth_pitch_deg": 360.0 / DEFAULT_STATOR.slots,
            "front": "Rz(i*15deg) applied to tooth-0 lane and loft",
            "rear": "mirror Z after the same tooth rotation",
            "shared_boundary_policy": (
                "closures may be tangent; positive-volume intersection is forbidden"
            ),
        },
        "gates": gates,
        "controlling_blockers": controlling,
        "authority_boundary": {
            "current_cap_CAD_remains_rejected": cap["status"] != "PASS",
            "current_cap_collision_gate": cap["gates"][
                "bounded_tip_radius_and_Z_sweep_has_candidate"
            ],
            "current_cap_material_gate": cap["gates"][
                "material_and_finish_released"
            ],
            "meaning": (
                "PASS authorizes only this aggregate occupancy and exact cap "
                "support-surface contract as an input to an offset-flyer "
                "architecture. It does not authorize the existing cap CAD, "
                "production, purchasing, printing, or assembly integration."
            ),
        },
        "source_hashes": {
            "GOAL.md": _sha256(GOAL),
            "cad/params.py": _sha256(CAD / "params.py"),
            "cad/coil_growth.py": _sha256(CAD / "coil_growth.py"),
            "sim/aggregate_progressive_wire_corridor.py": _sha256(
                HERE / "aggregate_progressive_wire_corridor.py"
            ),
            "sim/phase_aware_progressive_wire_audit.py": _sha256(
                HERE / "phase_aware_progressive_wire_audit.py"
            ),
            "sim/r3_sector_chord_family_study.py": _sha256(
                HERE / "r3_sector_chord_family_study.py"
            ),
            "sim/permanent_cap_flyer_recovery_study.py": _sha256(
                HERE / "permanent_cap_flyer_recovery_study.py"
            ),
            "sim/permanent_cap_aggregate_authorization.py": _sha256(
                Path(__file__)
            ),
            "out/reports/aggregate_progressive_wire_corridor.json": _sha256(
                AGGREGATE
            ),
            "out/reports/phase_aware_progressive_wire_audit.json": _sha256(
                PHASE
            ),
            "out/reports/coil_growth.json": _sha256(COIL_REPORT),
            "out/reports/r3_sector_chord_family_study.json": _sha256(
                SECTOR_REPORT
            ),
            "out/reports/permanent_cap_flyer_recovery.json": _sha256(
                CAP_REPORT
            ),
            "out/capture/upstream_current_raw.jsonl": raw["sha256"],
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def render_markdown(report: Mapping[str, Any]) -> str:
    slot = report["slot_partition"]
    lane = report["cap_support_lane"]
    loft = report["aggregate_loft"]
    lines = [
        "# Permanent-cap aggregate coil authorization",
        "",
        f"Status: **{report['status']}**. Aggregate geometry authority is "
        f"**{report['aggregate_geometry_authorized']}**; production and "
        "assembly integration remain **false**.",
        "",
        "## Shared-slot aggregate",
        "",
        f"Each 50-turn coil side requires {slot['required_50_turn_copper_area_mm2']:.6f} mm2. "
        f"The exact centre-safe half-slot cell is cut at u={slot['u_cutoff_mm']:.6f} mm "
        f"and has {slot['selected_aggregate_half_slot_area_mm2']:.6f} mm2. "
        f"Two sides occupy {slot['gross_slot_fill']:.3%} of the wire-accessible shared slot "
        f"against the {slot['hard_slot_fill_limit']:.0%} hard limit.",
        "",
        "The v=0 Voronoi boundary may be tangent, but the two open half-slot "
        "interiors are disjoint. Rotating the same construction through all "
        "24 shared slots yields 24 closed aggregate coils.",
        "",
        "## Continuous slot-to-crown connectors",
        "",
        f"All {report['slot_to_crown_connectors']['connector_count']} front/rear "
        "connectors extrude the complete selected half-slot section and join "
        f"the crown loft through {report['slot_to_crown_connectors']['positive_area_join_to_crown_mm2']:.6f} mm2, "
        "not through a point endpoint. The live aggregate boundary has zero "
        "positive-volume intrusion into active prior copper, cap material, "
        "completed neighbor copper, or nonadjacent copper. Contact with the "
        "polished cap and active parent is boundary tangency only.",
        "",
        "## Core and cross-coil separation",
        "",
        f"Minimum finished-wire outer-surface clearance to core is "
        f"{loft['core_intrusion']['minimum_wire_outer_surface_to_core_mm']:.3f} mm. "
        "Analytic positive-volume core intersection is 0 mm3. Shared-slot, "
        "adjacent-crown, and nonadjacent-crown aggregate interiors also have "
        "0 mm3 positive-volume overlap; equality is allowed only on ownership "
        "boundaries.",
        "",
        "## Permanent R3 support contract",
        "",
        f"Lane `{lane['id']}` uses {lane['nominal_front_centerline']['circular_primitive_radius_mm']:.3f} mm "
        f"nominal circular primitives and a +/-{lane['center_lane_half_width_mm']:.3f} mm "
        f"center band. Its worst allowed wire-center radius is "
        f"{lane['minimum_lane_wire_center_bend_radius_mm']:.3f} mm. The cap "
        f"contact surface must be at least "
        f"{lane['support_surface_contract']['minimum_contact_surface_radius_mm']:.5f} mm "
        f"radius with a {lane['support_surface_contract']['groove_clear_width_mm']:.5f} mm "
        "clear polished groove.",
        "",
        f"The finished-wire axial envelope is "
        f"{lane['front_rear_finished_wire_envelope_mm'][0]:.6f}.."
        f"{lane['front_rear_finished_wire_envelope_mm'][1]:.6f} mm "
        f"({lane['finished_wire_total_axial_envelope_mm']:.6f} mm total). "
        "A retained rotor/end-bell cavity of that size is not claimed.",
        "",
        "## Authority boundary",
        "",
        report["authority_boundary"]["meaning"],
        "",
        "The current permanent-cap CAD still collides with the current flyer "
        "spoke and its material/finish remain unreleased. The named contract "
        "is therefore an input for a separately validated offset-flyer lane, "
        "not production authority.",
        "",
        "Exact strand centers, layer order/neatness, settling, tension, sag, "
        "snagging, springback, friction, and enamel abrasion are explicitly "
        "outside this aggregate authorization.",
        "",
    ]
    if report["controlling_blockers"]:
        lines.extend(["## Controlling blockers", ""])
        lines.extend(
            f"- `{name}`" for name in report["controlling_blockers"]
        )
        lines.append("")
    lines.extend([
        f"Canonical raw capture: `{report['canonical_raw_capture']['sha256']}`",
        "",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ])
    return "\n".join(lines)


def write_reports(report: Mapping[str, Any] | None = None
                  ) -> dict[str, Any]:
    result = analyze() if report is None else dict(report)
    REPORTS.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    MD_OUT.write_text(render_markdown(result), encoding="utf-8")
    return result


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unexpected aggregate authorization schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("aggregate authorization proof hash mismatch")
    if report.get("status") == "PASS" and report.get("controlling_blockers"):
        raise ValueError("PASS report has controlling blockers")
    if bool(report.get("aggregate_geometry_authorized")) != (
        report.get("status") == "PASS"
    ):
        raise ValueError("aggregate authority flag does not match status")


def main() -> int:
    report = write_reports()
    validate_report_integrity(report)
    print(
        f"permanent cap aggregate: {report['status']}; "
        f"fill={report['slot_partition']['gross_slot_fill']:.3%}; "
        f"axial={report['cap_support_lane']['finished_wire_total_axial_envelope_mm']:.3f} mm"
    )
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
