"""Authoritative slot-fill and wound-coil envelope model for Goal 1.

This module is authoritative for the *generated* stator geometry in
``stator_model.py``.  It deliberately does not claim that every combination in
the machine's mechanical handling envelope can accept the same turn count.  A 24-tooth
fully wound stator puts one side of each adjacent tooth coil in every slot, so
each slot must hold ``2 * turns`` wire cross-sections.

The fill calculation uses the exact radial dimensions and angular tooth/shoe
coverage used by ``stator_model.stator``.  ``maximum_slot_fill`` is a practical
upper limit for round enamelled wire rather than the unattainable 100 percent
geometric limit.  Jobs between the design target and hard limit are reported as
MARGINAL; jobs above the hard limit are rejected.

Coordinate convention for the optional collision geometry matches
``stator_model``: stator axis +Z, stack centred at Z=0, tooth 0 on +X.
``coil_collision_envelopes`` returns conservative closed solids surrounding
each tooth's final winding region.  They can be added to the spindle collision
link, or ``wound_stator_collision_model`` can replace the bare-stator body.

Run directly to write ``out/reports/coil_growth.json`` and
``out/reports/coil_growth.md``.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Iterable

from build123d import Align, Box, Compound, Cylinder, Ellipse, Plane, Pos, Rot, extrude

from params import DEFAULT_STATOR, PARAMS, StatorSpec
import stator_model


MACHINE_ROOT = Path(__file__).resolve().parents[1]
REPORT_ROOT = MACHINE_ROOT / "out" / "reports"


@dataclass(frozen=True)
class PackingPolicy:
    """Explicit assumptions behind the pass/fail decision.

    ``wire_d`` is the finished enamelled outside diameter, so its circular
    cross-section includes insulation. ``opening_edge_clearance_mm`` is the
    installed slot-liner thickness on each steel edge, not an unexplained air
    allowance.  The release liner is DuPont Nomex Type 410 5 mil, so the
    nominal schedule uses 0.127 mm. ``liner_receiving_min_mm`` and
    ``liner_receiving_max_mm`` bound the measured-sheet interval over which
    the fixed packing topology is separately revalidated. A receipt value is
    an input to schedule generation, not a hidden collision allowance.
    55% is the normal design target and 60% is the hard launch limit.  The
    latter is intentionally below ideal hexagonal packing to reserve room for
    boundary loss and winding disorder.
    """

    design_slot_fill: float = 0.55
    maximum_slot_fill: float = 0.60
    opening_edge_clearance_mm: float = 0.127
    liner_receiving_min_mm: float = 0.120
    liner_receiving_max_mm: float = 0.140
    radial_end_clearance_mm: float = 0.25
    collision_extra_mm: float = 0.25
    wire_d_min_mm: float = 0.20
    wire_d_max_mm: float = 0.50


DEFAULT_POLICY = PackingPolicy()
VALIDATION_WIRE_RADIUS_MM = 0.10


def _rounded_prism_x(length: float, width: float, height: float,
                     radius: float):
    """Convex X-axis prism with a rounded YZ rectangle cross-section."""
    if min(length, width, height) <= 0:
        raise ValueError("rounded prism dimensions must be positive")
    if not 0 < radius <= min(width, height) / 2.0:
        raise ValueError("rounded prism radius does not fit cross-section")
    align = (Align.CENTER, Align.CENTER, Align.CENTER)
    body = (Box(length, width, height - 2.0 * radius, align=align)
            + Box(length, width - 2.0 * radius, height, align=align))
    corner = Rot(0, 90, 0) * Cylinder(radius, length, align=align)
    for sy in (-1.0, 1.0):
        for sz in (-1.0, 1.0):
            body += Pos(
                0.0,
                sy * (width / 2.0 - radius),
                sz * (height / 2.0 - radius),
            ) * corner
    return body


def _ellipse_prism_x(length: float, tangential_radius: float,
                     axial_radius: float):
    """X-axis prism with a smooth elliptical YZ cross-section."""
    section = Ellipse(tangential_radius, axial_radius)
    return extrude(Plane.YZ * section, amount=length / 2.0, both=True)


def _validate_inputs(spec: StatorSpec, policy: PackingPolicy) -> None:
    if spec.slots < 3:
        raise ValueError(f"slots must be >= 3, got {spec.slots}")
    if spec.od <= 0 or spec.stack <= 0 or spec.shaft_d <= 0:
        raise ValueError("OD, stack, and shaft diameter must be positive")
    if not 0 < spec.hub_od_ratio < 1:
        raise ValueError("hub_od_ratio must be between 0 and 1")
    if spec.wire_d <= 0 or spec.turns <= 0:
        raise ValueError("wire diameter and turns must be positive")
    if not 0 < policy.design_slot_fill <= policy.maximum_slot_fill < 1:
        raise ValueError("packing fills must satisfy 0 < design <= maximum < 1")


def slot_geometry(spec: StatorSpec) -> dict[str, float]:
    """Return the exact one-slot geometry used by ``stator_model``.

    The area is integrated analytically over radius.  At a given radius the
    two neighbouring rectangular tooth necks consume ``asin((neck_w/2)/r)``
    apiece.  In the shoe band, the larger of that angle and the shoe's 0.36
    pitch half-angle controls.  This reproduces the Boolean CAD section while
    avoiding a slow BREP operation in every job check.
    """

    _validate_inputs(spec, DEFAULT_POLICY)
    radius = spec.od / 2.0
    hub_radius = spec.od * spec.hub_od_ratio / 2.0
    neck_width = max(2.5, spec.od * 0.07)
    half_neck = neck_width / 2.0
    shoe_thickness = max(1.6, spec.od * 0.045)
    shoe_inner_radius = radius - shoe_thickness
    pitch = 2.0 * math.pi / spec.slots
    shoe_half_angle = 0.36 * pitch
    opening_angle = pitch - 2.0 * shoe_half_angle

    # The neighbouring neck rectangles meet at the inner slot root.  Below
    # this radius the nominal sector opening is negative and therefore solid.
    slot_root_radius = max(
        hub_radius,
        half_neck / math.sin(pitch / 2.0),
    )
    if slot_root_radius >= shoe_inner_radius:
        raise ValueError("tooth geometry leaves no positive-area slot")

    def neck_integral(r: float) -> float:
        # Integral of r * asin(half_neck / r) dr.
        ratio = min(1.0, half_neck / r)
        return 0.5 * (
            r * r * math.asin(ratio)
            + half_neck * math.sqrt(max(0.0, r * r - half_neck * half_neck))
        )

    def neck_open_area(r0: float, r1: float) -> float:
        if r1 <= r0:
            return 0.0
        sector = 0.5 * pitch * (r1 * r1 - r0 * r0)
        necks = 2.0 * (neck_integral(r1) - neck_integral(r0))
        return sector - necks

    # Below the shoe, tooth necks bound the slot.
    area = neck_open_area(slot_root_radius, shoe_inner_radius)

    # The neck extends 1 mm into the shoe band in stator_model.  It only
    # controls where its angular half-width is greater than the shoe angle.
    neck_end_radius = shoe_inner_radius + 1.0
    neck_angle_equal_radius = half_neck / math.sin(shoe_half_angle)
    neck_dominates_to = min(radius, neck_end_radius, neck_angle_equal_radius)
    if neck_dominates_to > shoe_inner_radius:
        area += neck_open_area(shoe_inner_radius, neck_dominates_to)
    shoe_controls_from = max(shoe_inner_radius, neck_dominates_to)
    if radius > shoe_controls_from:
        area += 0.5 * opening_angle * (
            radius * radius - shoe_controls_from * shoe_controls_from
        )

    opening_width = 2.0 * shoe_inner_radius * math.sin(opening_angle / 2.0)
    return {
        "outer_radius_mm": radius,
        "hub_radius_mm": hub_radius,
        "slot_root_radius_mm": slot_root_radius,
        "shoe_inner_radius_mm": shoe_inner_radius,
        "shoe_thickness_mm": shoe_thickness,
        "tooth_neck_width_mm": neck_width,
        "tooth_pitch_deg": math.degrees(pitch),
        "shoe_coverage_fraction_of_pitch": 0.72,
        "opening_angle_deg": math.degrees(opening_angle),
        "opening_width_mm": opening_width,
        "radial_slot_depth_mm": shoe_inner_radius - slot_root_radius,
        "geometric_slot_area_mm2": area,
    }


def _max_turns(slot_area: float, wire_area: float, allowed_fill: float) -> int:
    # Two tooth-coil sides occupy every slot on a fully wound stator.
    return max(0, math.floor(slot_area * allowed_fill / (2.0 * wire_area)))


def _max_wire_d(slot_area: float, turns: int, allowed_fill: float) -> float:
    return math.sqrt(2.0 * slot_area * allowed_fill / (turns * math.pi))


def analyze_job(
    spec: StatorSpec,
    policy: PackingPolicy = DEFAULT_POLICY,
) -> dict[str, object]:
    """Evaluate one fully wound stator job and return a JSON-safe record."""

    _validate_inputs(spec, policy)
    geom = slot_geometry(spec)
    wire_area = math.pi * (spec.wire_d / 2.0) ** 2
    one_coil_side_area = spec.turns * wire_area
    required_per_slot = 2.0 * one_coil_side_area
    opening_required = spec.wire_d + 2.0 * policy.opening_edge_clearance_mm
    opening_margin = geom["opening_width_mm"] - opening_required

    # A positive-area slot is not automatically reachable by the wire.  The
    # neighbouring constant-width necks meet at ``slot_root_radius_mm`` and
    # only open gradually.  Start winding where their chord gap admits the
    # finished wire plus the declared edge allowance, and stop below the shoe
    # throat.  Copper cannot be packed into either excluded region, so the
    # fill gate must use this accessible area rather than the full geometric
    # void (the former calculation overstated default capacity by ~49%).
    pitch = 2.0 * math.pi / spec.slots
    neck_width = float(geom["tooth_neck_width_mm"])
    half_neck = neck_width / 2.0
    access_radius = (
        (neck_width + opening_required)
        / (2.0 * math.sin(pitch / 2.0))
    )
    radial_winding_start = max(
        float(geom["slot_root_radius_mm"]) + policy.radial_end_clearance_mm,
        access_radius,
    )
    radial_winding_end = (
        float(geom["shoe_inner_radius_mm"]) - policy.radial_end_clearance_mm
    )
    accessible_span = radial_winding_end - radial_winding_start

    def neck_integral(radius: float) -> float:
        ratio = min(1.0, half_neck / radius)
        return 0.5 * (
            radius * radius * math.asin(ratio)
            + half_neck * math.sqrt(
                max(0.0, radius * radius - half_neck * half_neck)
            )
        )

    if accessible_span > 0.0:
        accessible_slot_area = (
            0.5 * pitch
            * (radial_winding_end ** 2 - radial_winding_start ** 2)
            - 2.0
            * (neck_integral(radial_winding_end)
               - neck_integral(radial_winding_start))
        )
    else:
        accessible_slot_area = 0.0
    slot_area = accessible_slot_area
    gross_fill = (
        required_per_slot / slot_area if slot_area > 0.0 else math.inf
    )
    access_ok = accessible_span >= spec.wire_d and slot_area > 0.0

    within_launch_envelope = (
        PARAMS.stator_od_min <= spec.od <= PARAMS.stator_od_max
        and PARAMS.stack_min <= spec.stack <= PARAMS.stack_max
        and PARAMS.shaft_d_min <= spec.shaft_d <= PARAMS.shaft_d_max
        and policy.wire_d_min_mm <= spec.wire_d <= policy.wire_d_max_mm
    )
    capacity_ok = access_ok and gross_fill <= policy.maximum_slot_fill + 1e-12
    design_ok = access_ok and gross_fill <= policy.design_slot_fill + 1e-12
    opening_ok = opening_margin >= -1e-12

    reasons: list[str] = []
    if not within_launch_envelope:
        reasons.append("job is outside the declared launch handling envelope")
    if not access_ok:
        reasons.append(
            "wire-plus-edge access leaves no usable radial slot span"
        )
    if not capacity_ok:
        reasons.append(
            f"{gross_fill:.1%} gross slot fill exceeds the "
            f"{policy.maximum_slot_fill:.0%} hard packing limit"
        )
    elif not design_ok:
        reasons.append(
            f"{gross_fill:.1%} gross slot fill is above the "
            f"{policy.design_slot_fill:.0%} design target"
        )
    if not opening_ok:
        reasons.append(
            f"{geom['opening_width_mm']:.3f} mm slot opening is smaller than "
            f"the {opening_required:.3f} mm wire-plus-edge allowance"
        )

    if within_launch_envelope and access_ok and capacity_ok and opening_ok:
        status = "PASS" if design_ok else "MARGINAL"
    else:
        status = "FAIL"

    radial_span = max(spec.wire_d, accessible_span)
    # Minimum physical pack depth at the design packing density.  Quantized
    # rows ensure the envelope never becomes thinner than a realizable number
    # of wire layers.
    continuous_growth = one_coil_side_area / (
        radial_span * policy.design_slot_fill
    )
    wires_per_radial_row = max(1, math.floor(radial_span / spec.wire_d))
    layers = math.ceil(spec.turns / wires_per_radial_row)
    discrete_growth = layers * spec.wire_d
    predicted_growth = max(continuous_growth, discrete_growth)
    collision_growth = max(
        PARAMS.wire_bundle_allow,
        predicted_growth + spec.wire_d / 2.0 + policy.collision_extra_mm,
    )
    # A smooth ellipse is used only as the analytical final-bundle/contact
    # envelope.  It removes the flat-side support discontinuity of a box while
    # retaining the same conservative tangential/axial extents.  It is NOT a
    # printable former: adjacent tooth pitch makes a universal 3 mm former
    # physically impossible on the smaller launch stators.
    end_turn_axial_radius = spec.stack / 2.0 + collision_growth
    end_turn_tangential_base = geom["tooth_neck_width_mm"] / 2.0 + collision_growth
    end_turn_tangential_radius = end_turn_tangential_base
    if end_turn_axial_radius >= end_turn_tangential_radius:
        former_min_curvature = (
            end_turn_tangential_radius ** 2 / end_turn_axial_radius
        )
    else:
        former_min_curvature = (
            end_turn_axial_radius ** 2 / end_turn_tangential_radius
        )
    centerline_min_curvature = (
        former_min_curvature + VALIDATION_WIRE_RADIUS_MM
    )

    return {
        "status": status,
        "reasons": reasons,
        "spec": {
            "slots": spec.slots,
            "od_mm": spec.od,
            "stack_mm": spec.stack,
            "shaft_d_mm": spec.shaft_d,
            "hub_od_ratio": spec.hub_od_ratio,
            "wire_finished_d_mm": spec.wire_d,
            "turns_per_tooth": spec.turns,
        },
        "within_launch_handling_envelope": within_launch_envelope,
        "slot": geom,
        "packing": {
            "adjacent_coil_sides_per_slot": 2,
            "wire_cross_section_mm2": wire_area,
            "one_coil_side_wire_area_mm2": one_coil_side_area,
            "required_wire_area_per_slot_mm2": required_per_slot,
            "full_geometric_slot_area_mm2": geom["geometric_slot_area_mm2"],
            "wire_accessible_slot_area_mm2": slot_area,
            "excluded_slot_area_mm2": (
                geom["geometric_slot_area_mm2"] - slot_area
            ),
            "gross_slot_fill": gross_fill,
            "design_slot_fill_limit": policy.design_slot_fill,
            "maximum_slot_fill_limit": policy.maximum_slot_fill,
            "area_capacity_at_design_fill_mm2": slot_area
            * policy.design_slot_fill,
            "area_capacity_at_maximum_fill_mm2": slot_area
            * policy.maximum_slot_fill,
            "max_turns_at_design_fill": _max_turns(
                slot_area, wire_area, policy.design_slot_fill
            ),
            "max_turns_at_maximum_fill": _max_turns(
                slot_area, wire_area, policy.maximum_slot_fill
            ),
            "max_wire_d_for_turns_at_design_fill_mm": _max_wire_d(
                slot_area, spec.turns, policy.design_slot_fill
            ),
            "max_wire_d_for_turns_at_maximum_fill_mm": _max_wire_d(
                slot_area, spec.turns, policy.maximum_slot_fill
            ),
        },
        "slot_opening": {
            "required_width_mm": opening_required,
            "margin_mm": opening_margin,
            "ok": opening_ok,
        },
        "slot_access": {
            "required_neck_gap_mm": opening_required,
            "wire_accessible_start_radius_mm": radial_winding_start,
            "wire_accessible_end_radius_mm": radial_winding_end,
            "accessible_radial_span_mm": accessible_span,
            "accessible_slot_area_mm2": slot_area,
            "ok": access_ok,
        },
        "bundle": {
            "radial_winding_start_mm": radial_winding_start,
            "radial_winding_end_mm": radial_winding_end,
            "usable_radial_span_mm": radial_span,
            "wires_per_radial_row": wires_per_radial_row,
            "discrete_layers": layers,
            "continuous_growth_at_design_fill_mm": continuous_growth,
            "predicted_growth_mm": predicted_growth,
            "collision_growth_mm": collision_growth,
            "existing_machine_allowance_mm": PARAMS.wire_bundle_allow,
            "end_turn_envelope": {
                "model": "analytical smooth elliptical final-coil envelope",
                "tangential_radius_mm": end_turn_tangential_radius,
                "axial_radius_mm": end_turn_axial_radius,
                "base_tangential_radius_mm": end_turn_tangential_base,
                "validation_wire_radius_mm": VALIDATION_WIRE_RADIUS_MM,
                "minimum_deposited_wire_center_curvature_radius_mm": (
                    centerline_min_curvature
                ),
                "manufacturable_former": False,
                "bend_rule_scope": (
                    "reported workpiece conformity; not the free-running "
                    "machine-guide >=3 mm gate"
                ),
            },
        },
    }


def require_feasible(
    spec: StatorSpec,
    policy: PackingPolicy = DEFAULT_POLICY,
) -> dict[str, object]:
    """Return the assessment or raise before impossible CAD is generated."""

    result = analyze_job(spec, policy)
    if result["status"] == "FAIL":
        detail = "; ".join(result["reasons"]) or "unknown violation"
        raise ValueError(f"infeasible winding job: {detail}")
    return result


def coil_collision_envelopes(
    spec: StatorSpec,
    policy: PackingPolicy = DEFAULT_POLICY,
    *,
    growth_mm: float | None = None,
    allow_infeasible: bool = False,
) -> list:
    """Return one conservative closed collision solid per wound tooth.

    The solids intentionally include the tooth core volume.  That makes them
    robust closed replacement envelopes for external collision checks.  Use
    ``coil_bundle_shells`` when coil-only display geometry is desired.
    """

    result = analyze_job(spec, policy)
    if result["status"] == "FAIL" and not allow_infeasible:
        detail = "; ".join(result["reasons"]) or "unknown violation"
        raise ValueError(f"refusing envelope for infeasible job: {detail}")
    geom = result["slot"]
    growth = (
        float(result["bundle"]["collision_growth_mm"])
        if growth_mm is None
        else float(growth_mm)
    )
    if growth <= 0:
        raise ValueError("growth_mm must be positive")

    x0 = geom["slot_root_radius_mm"] + policy.radial_end_clearance_mm
    x1 = geom["shoe_inner_radius_mm"] - policy.radial_end_clearance_mm
    radial_length = x1 - x0
    if radial_length <= 0:
        raise ValueError("radial clearances leave no winding span")
    centre_x = (x0 + x1) / 2.0
    envelope = result["bundle"]["end_turn_envelope"]
    tangential_radius = float(envelope["tangential_radius_mm"])
    axial_radius = float(envelope["axial_radius_mm"])
    if growth_mm is not None:
        # Explicit diagnostic growth overrides retain the same smooth contact
        # model instead of silently reverting to a square box.
        axial_radius = spec.stack / 2.0 + growth
        tangential_radius = geom["tooth_neck_width_mm"] / 2.0 + growth
    envelopes = []
    for tooth in range(spec.slots):
        body = _ellipse_prism_x(
            radial_length, tangential_radius, axial_radius,
        )
        body = Pos(centre_x, 0, 0) * body
        body = Rot(0, 0, tooth * 360.0 / spec.slots) * body
        body.label = f"coil_collision_envelope_{tooth:02d}"
        envelopes.append(body)
    return envelopes


def coil_bundle_shells(
    spec: StatorSpec,
    policy: PackingPolicy = DEFAULT_POLICY,
    *,
    growth_mm: float | None = None,
    allow_infeasible: bool = False,
) -> list:
    """Return coil-only rectangular-tube envelopes for visualisation."""

    result = analyze_job(spec, policy)
    if result["status"] == "FAIL" and not allow_infeasible:
        detail = "; ".join(result["reasons"]) or "unknown violation"
        raise ValueError(f"refusing envelope for infeasible job: {detail}")
    geom = result["slot"]
    growth = (
        float(result["bundle"]["predicted_growth_mm"])
        if growth_mm is None
        else float(growth_mm)
    )
    if growth <= 0:
        raise ValueError("growth_mm must be positive")

    x0 = geom["slot_root_radius_mm"] + policy.radial_end_clearance_mm
    x1 = geom["shoe_inner_radius_mm"] - policy.radial_end_clearance_mm
    radial_length = x1 - x0
    centre_x = (x0 + x1) / 2.0
    neck_width = geom["tooth_neck_width_mm"]
    shells = []
    for tooth in range(spec.slots):
        axial_radius = spec.stack / 2.0 + growth
        tangential_radius = neck_width / 2.0 + growth
        outer = _ellipse_prism_x(
            radial_length, tangential_radius, axial_radius,
        )
        # Overshoot in X removes end caps and leaves the four physical bundle
        # bands around the tooth's tangential/axial cross-section.
        core = Box(radial_length + 2.0, neck_width, spec.stack,
                   align=(Align.CENTER, Align.CENTER, Align.CENTER))
        body = outer - core
        body = Pos(centre_x, 0, 0) * body
        body = Rot(0, 0, tooth * 360.0 / spec.slots) * body
        body.label = f"coil_bundle_{tooth:02d}"
        shells.append(body)
    return shells


def wound_stator_collision_model(
    spec: StatorSpec,
    policy: PackingPolicy = DEFAULT_POLICY,
    *,
    growth_mm: float | None = None,
) -> Compound:
    """Return bare stator plus conservative final-coil collision envelopes."""

    children = [stator_model.stator(spec, label="stator_core")]
    children.extend(
        coil_collision_envelopes(spec, policy, growth_mm=growth_mm)
    )
    model = Compound(children=children)
    model.label = "wound_stator_collision_model"
    return model


def _minimum_od_for_turns(
    wire_d: float,
    turns: int,
    allowed_fill: float,
    policy: PackingPolicy,
) -> float | None:
    def fill_at(od: float) -> float:
        spec = StatorSpec(od=od, wire_d=wire_d, turns=turns)
        return float(analyze_job(spec, policy)["packing"]["gross_slot_fill"])

    low = PARAMS.stator_od_min
    high = PARAMS.stator_od_max
    if fill_at(high) > allowed_fill:
        return None
    if fill_at(low) <= allowed_fill:
        return low
    for _ in range(60):
        mid = (low + high) / 2.0
        if fill_at(mid) <= allowed_fill:
            high = mid
        else:
            low = mid
    return high


def _source_hashes() -> dict[str, str]:
    names = ("params.py", "stator_model.py", "coil_growth.py")
    return {
        name: hashlib.sha256((Path(__file__).parent / name).read_bytes()).hexdigest()
        for name in names
    }


def generate_report(policy: PackingPolicy = DEFAULT_POLICY) -> dict[str, object]:
    """Build the complete Goal-1 slot/fill evidence record."""

    ods = (28.0, 36.0, 46.0, 55.0, 65.0)
    wires = (0.20, 0.24, 0.25, 0.30, 0.40, 0.50)
    matrix = []
    for od in ods:
        for wire_d in wires:
            result = analyze_job(
                StatorSpec(od=od, wire_d=wire_d, turns=75), policy
            )
            matrix.append(
                {
                    "od_mm": od,
                    "wire_finished_d_mm": wire_d,
                    "status": result["status"],
                    "wire_accessible_slot_area_mm2": result["packing"][
                        "wire_accessible_slot_area_mm2"
                    ],
                    "full_geometric_slot_area_mm2": result["packing"][
                        "full_geometric_slot_area_mm2"
                    ],
                    "gross_slot_fill": result["packing"]["gross_slot_fill"],
                    "max_turns_at_maximum_fill": result["packing"][
                        "max_turns_at_maximum_fill"
                    ],
                    "slot_opening_margin_mm": result["slot_opening"]["margin_mm"],
                }
            )

    minimum_od = []
    for wire_d in (0.20, 0.25, 0.30, 0.40, 0.50):
        minimum_od.append(
            {
                "wire_finished_d_mm": wire_d,
                "minimum_od_for_75_turns_design_fill_mm": _minimum_od_for_turns(
                    wire_d, 75, policy.design_slot_fill, policy
                ),
                "minimum_od_for_75_turns_maximum_fill_mm": _minimum_od_for_turns(
                    wire_d, 75, policy.maximum_slot_fill, policy
                ),
            }
        )

    # Stack and shaft do not enter the 2D lamination slot section, but their
    # endpoint combinations are evaluated explicitly so that invariance is
    # evidence rather than an undocumented assumption.
    endpoint_invariance = []
    for stack in (PARAMS.stack_min, PARAMS.stack_max):
        for shaft in (PARAMS.shaft_d_min, PARAMS.shaft_d_max):
            result = analyze_job(
                StatorSpec(
                    od=46.0,
                    stack=stack,
                    shaft_d=shaft,
                    wire_d=0.25,
                    turns=75,
                ),
                policy,
            )
            endpoint_invariance.append(
                {
                    "stack_mm": stack,
                    "shaft_d_mm": shaft,
                    "slot_area_mm2": result["slot"]["geometric_slot_area_mm2"],
                    "gross_slot_fill": result["packing"]["gross_slot_fill"],
                    "collision_envelope_total_height_mm": stack
                    + 2.0 * result["bundle"]["collision_growth_mm"],
                }
            )

    current_default = analyze_job(DEFAULT_STATOR, policy)
    legacy_default_030 = analyze_job(
        StatorSpec(
            slots=DEFAULT_STATOR.slots,
            od=DEFAULT_STATOR.od,
            stack=DEFAULT_STATOR.stack,
            shaft_d=DEFAULT_STATOR.shaft_d,
            shaft_below=DEFAULT_STATOR.shaft_below,
            shaft_above=DEFAULT_STATOR.shaft_above,
            hub_od_ratio=DEFAULT_STATOR.hub_od_ratio,
            wire_d=0.30,
            turns=75,
            winding_config=DEFAULT_STATOR.winding_config,
        ),
        policy,
    )
    legacy_default_024_75 = analyze_job(
        StatorSpec(
            slots=DEFAULT_STATOR.slots,
            od=DEFAULT_STATOR.od,
            stack=DEFAULT_STATOR.stack,
            shaft_d=DEFAULT_STATOR.shaft_d,
            shaft_below=DEFAULT_STATOR.shaft_below,
            shaft_above=DEFAULT_STATOR.shaft_above,
            hub_od_ratio=DEFAULT_STATOR.hub_od_ratio,
            wire_d=0.24,
            turns=75,
            winding_config=DEFAULT_STATOR.winding_config,
        ),
        policy,
    )
    default_024 = analyze_job(
        StatorSpec(
            slots=DEFAULT_STATOR.slots,
            od=DEFAULT_STATOR.od,
            stack=DEFAULT_STATOR.stack,
            shaft_d=DEFAULT_STATOR.shaft_d,
            shaft_below=DEFAULT_STATOR.shaft_below,
            shaft_above=DEFAULT_STATOR.shaft_above,
            hub_od_ratio=DEFAULT_STATOR.hub_od_ratio,
            wire_d=0.24,
            turns=DEFAULT_STATOR.turns,
            winding_config=DEFAULT_STATOR.winding_config,
        ),
        policy,
    )
    default_025 = analyze_job(
        StatorSpec(
            slots=DEFAULT_STATOR.slots,
            od=DEFAULT_STATOR.od,
            stack=DEFAULT_STATOR.stack,
            shaft_d=DEFAULT_STATOR.shaft_d,
            shaft_below=DEFAULT_STATOR.shaft_below,
            shaft_above=DEFAULT_STATOR.shaft_above,
            hub_od_ratio=DEFAULT_STATOR.hub_od_ratio,
            wire_d=0.25,
            turns=DEFAULT_STATOR.turns,
            winding_config=DEFAULT_STATOR.winding_config,
        ),
        policy,
    )

    return {
        "schema": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "authority": (
            "Exact for the simplified generated stator_model.py slot geometry; "
            "replace with a measured/vendor lamination section for a purchased stator."
        ),
        "source_sha256": _source_hashes(),
        "policy": asdict(policy),
        "model_assumptions": {
            "fully_wound_teeth": DEFAULT_STATOR.slots,
            "adjacent_coil_sides_per_slot": 2,
            "wire_diameter_means": "finished enamelled outside diameter",
            "packing_interpretation": (
                "gross circular wire area divided by the wire-accessible slot area"
            ),
            "stack_and_shaft_effect": (
                "stack changes axial collision envelope height; shaft does not "
                "change this parametric lamination's 2D slot area"
            ),
        },
        "launch_envelope": {
            "stator_od_mm": [PARAMS.stator_od_min, PARAMS.stator_od_max],
            "stack_mm": [PARAMS.stack_min, PARAMS.stack_max],
            "shaft_d_mm": [PARAMS.shaft_d_min, PARAMS.shaft_d_max],
            "wire_finished_d_mm": [policy.wire_d_min_mm, policy.wire_d_max_mm],
            "all_75_turn_combinations_supported": False,
        },
        "current_default": current_default,
        "legacy_default_0_30_mm": legacy_default_030,
        "legacy_default_0_24_mm_75_turns": legacy_default_024_75,
        "candidate_default_0_24_mm": default_024,
        "candidate_default_0_25_mm": default_025,
        "capacity_matrix_75_turns": matrix,
        "minimum_stator_od_for_75_turns": minimum_od,
        "stack_shaft_endpoint_invariance": endpoint_invariance,
        "recommendations": [
            (
                "The former OD46 / 0.24 mm / 75-turn job is rejected after "
                "excluding its inaccessible root wedge: it needs "
                f"{legacy_default_024_75['packing']['gross_slot_fill']:.1%} fill and "
                f"the hard-limit capacity is only "
                f"{legacy_default_024_75['packing']['max_turns_at_maximum_fill']} turns."
            ),
            (
                "The corrected OD46 / 50-turn default uses 0.24 mm finished wire "
                f"and passes at {current_default['packing']['gross_slot_fill']:.1%}. "
                f"Its 55% design capacity is "
                f"{current_default['packing']['max_turns_at_design_fill']} turns and "
                f"its 60% hard capacity is "
                f"{current_default['packing']['max_turns_at_maximum_fill']} turns."
            ),
            (
                "analyze_job/require_feasible is the settings-generation gate; "
                "the 0.2-0.5 mm handling range is not universal "
                "75-turn capacity."
            ),
            (
                "coil_collision_envelopes is included in the spindle link for "
                "the final-wound collision sweep.  Its default 3 mm growth preserves the "
                "machine's existing conservative bundle allowance."
            ),
        ],
    }


def render_markdown(report: dict[str, object]) -> str:
    policy = report["policy"]
    default = report["current_default"]
    lines = [
        "# Coil growth and slot-fill validation",
        "",
        f"Authority: {report['authority']}",
        "",
        "A fully wound 24-tooth stator puts two coil sides in each slot. "
        "The check uses finished enamelled wire OD, a "
        f"{policy['design_slot_fill']:.0%} design fill target, and a "
        f"{policy['maximum_slot_fill']:.0%} hard limit.",
        "",
        "## Default job",
        "",
        f"OD {default['spec']['od_mm']:.0f} mm, "
        f"{default['spec']['wire_finished_d_mm']:.2f} mm wire, "
        f"{default['spec']['turns_per_tooth']} turns: "
        f"**{default['status']}** at "
        f"{default['packing']['gross_slot_fill']:.1%} fill.  Hard-limit "
        f"capacity is {default['packing']['max_turns_at_maximum_fill']} turns.",
        "",
        "## 75-turn capacity matrix",
        "",
        "| stator OD | finished wire OD | wire-accessible slot area | gross fill | max turns at 60% | result |",
        "|---:|---:|---:|---:|---:|:---|",
    ]
    for row in report["capacity_matrix_75_turns"]:
        lines.append(
            f"| {row['od_mm']:.0f} mm | {row['wire_finished_d_mm']:.2f} mm | "
            f"{row['wire_accessible_slot_area_mm2']:.3f} mm2 | "
            f"{row['gross_slot_fill']:.1%} | "
            f"{row['max_turns_at_maximum_fill']} | {row['status']} |"
        )
    lines.extend(
        [
            "",
            "PASS is at or below the 55% design target; MARGINAL is above 55% "
            "but at or below the 60% hard limit; FAIL is not a buildable 75-turn job.",
            "",
            "## Minimum OD for 75 turns",
            "",
            "| finished wire OD | OD at 55% target | OD at 60% hard limit |",
            "|---:|---:|---:|",
        ]
    )
    for row in report["minimum_stator_od_for_75_turns"]:
        design = row["minimum_od_for_75_turns_design_fill_mm"]
        hard = row["minimum_od_for_75_turns_maximum_fill_mm"]
        design_text = f"{design:.2f} mm" if design is not None else ">65 mm"
        hard_text = f"{hard:.2f} mm" if hard is not None else ">65 mm"
        lines.append(
            f"| {row['wire_finished_d_mm']:.2f} mm | {design_text} | {hard_text} |"
        )
    lines.extend(["", "## Design conclusions", ""])
    for recommendation in report["recommendations"]:
        lines.append(f"- {recommendation}")
    lines.extend(
        [
            "",
            "## Limits",
            "",
            "This is a geometry/capacity gate, not a winding-quality proof. "
            "It does not predict enamel damage, actual layer ordering, wire "
            "springback, tension dynamics, or the slot dimensions of a vendor "
            "stator that differs from the generated model.",
            "",
        ]
    )
    return "\n".join(lines)


def write_report(
    json_path: Path = REPORT_ROOT / "coil_growth.json",
    markdown_path: Path = REPORT_ROOT / "coil_growth.md",
) -> dict[str, object]:
    report = generate_report()
    json_path.parent.mkdir(parents=True, exist_ok=True)
    markdown_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    markdown_path.write_text(render_markdown(report), encoding="utf-8")
    return report


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--json",
        type=Path,
        default=REPORT_ROOT / "coil_growth.json",
        help="JSON evidence output",
    )
    parser.add_argument(
        "--markdown",
        type=Path,
        default=REPORT_ROOT / "coil_growth.md",
        help="human-readable report output",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)
    report = write_report(args.json, args.markdown)
    default = report["current_default"]
    print(
        "coil-growth validation: "
        f"default={default['status']} "
        f"fill={default['packing']['gross_slot_fill']:.3f}; "
        f"wrote {args.json} and {args.markdown}"
    )
    # The report generator itself succeeds even though it truthfully records
    # impossible combinations.  Integrators gate individual jobs with
    # require_feasible().
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
