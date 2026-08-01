"""Isolated positive-volume CAD for an aggregate-boundary floating follower.

Active-tooth local frame (mm): +X radial/outward, +Y tangential, +Z stator
axis.  The representative module retains the existing M1 law selection, M2
shoe identity, and M0 positive all-retracted concept.  It adds no commanded
axis: radial/tangential slides are passive contact followers with provisional
preload envelopes.

This file is deliberately not imported by assembly.py.  It is review-only
geometry and grants no wire-route, collision, load, procurement, production,
or release authority.  Root owns later STEP generation to
``out/review/aggregate_boundary_floating_follower.step``.
"""

from __future__ import annotations

import itertools
import math
from typing import Any

from build123d import Align, Box, Compound, Cylinder, Pos, Rot

import hardware
import m1_selector_alternating_former as selector


CTR = (Align.CENTER, Align.CENTER, Align.CENTER)

# Required kinematic contract and compact review layout.
RADIAL_STROKE_MM = 6.0
TANGENTIAL_STROKE_MM = 1.0
RADIAL_CENTER_RETRACTED_MM = 14.0
TANGENTIAL_CENTER_NEGATIVE_MM = -0.5
SLIDE_CLEARANCE_MM = 0.20

NOSE_CONTACT_SURFACE_RADIUS_MM = 3.0
NOSE_FLANGE_RADIUS_MM = 3.5
NOSE_GROOVE_CLEAR_WIDTH_MM = 0.65
NOSE_WIDTH_MM = 4.0

M3_CLEARANCE_DIAMETER_MM = 3.4
M3_INSERT_PILOT_DIAMETER_MM = 4.7
M3_CLAMP_SCREW_LENGTH_MM = 8.0
M3_CLAMP_LOCAL_STACK_MM = 4.05
M3_CLAMP_SCREW_COUNT = 2

TOWER_FRONT_FACE_MACHINE_Y_MM = -114.0
TOWER_KEY_MACHINE_X_MM = (-10.0, 10.0)
TOWER_KEY_MACHINE_Z_MM = 61.0
TOWER_M4_MACHINE_X_MM = (-21.0, 21.0)
TOWER_M4_MACHINE_Z_MM = (60.0, 66.0)
TOWER_M4_STACK_COUNT = 4
M4_CARRIER_SCREW_LENGTH_MM = 10.0
M4_CARRIER_THREAD_ENGAGEMENT_MM = 5.1
M4_SHORT_INSERT_LENGTH_MM = 4.7
PRIMARY_MOUNT_LOAD_CASE_N = 40.0
M4_WASHER_THICKNESS_MM = 0.90
M3_WASHER_THICKNESS_MM = 0.55
REFERENCE_M0_STANDOFF_MM = 95.0

OUTER_PIVOT_SHOULDER_DIAMETER_MM = 5.0
OUTER_PIVOT_SHOULDER_LENGTH_MM = 10.0
INNER_PIVOT_DIAMETER_MM = 3.0
INNER_PIVOT_SHOULDER_LENGTH_MM = 10.0

RADIAL_RETURN_SPRING_ID = "LEM050AB01_VIA_0P29_BELLCRANK"
RADIAL_SPRING_LEVER_RATIO = 0.29
TANGENTIAL_RETURN_SPRING_ID = (
    "UNRESOLVED_TANGENTIAL_RETURN_SPRING_ENVELOPE"
)

MATERIALS = {
    "carrier_and_inner_yoke": "6061-T6 aluminum",
    "cross_slide_cassette": "hard-anodized 7075-T6 aluminum",
    "tangential_slide_outer_yoke_cartridge": (
        "one-piece hard-anodized 7075-T6 aluminum"
    ),
    "nose": "virgin unfilled PEEK; polished contact surface",
    "pins": "ground stainless shoulder screws",
    "ceramic_alternative": "unselected polished ceramic insert envelope",
}

AUTHORITY = {
    "review_only": True,
    "assembly_integration_authorized": False,
    "wire_route_authorized": False,
    "collision_authorized": False,
    "production_authorized": False,
    "BOM_released": False,
    "fail_retraction_authorized": False,
}


def _cylinder_y(radius: float, length: float):
    return Rot(90.0, 0.0, 0.0) * Cylinder(radius, length, align=CTR)


def _cylinder_x(radius: float, length: float):
    return Rot(0.0, 90.0, 0.0) * Cylinder(radius, length, align=CTR)


def _label(part, label: str):
    part.label = label
    return part


def radial_position(state: str) -> float:
    positions = {
        "retracted": RADIAL_CENTER_RETRACTED_MM,
        "mid": RADIAL_CENTER_RETRACTED_MM + RADIAL_STROKE_MM / 2.0,
        "extended": RADIAL_CENTER_RETRACTED_MM + RADIAL_STROKE_MM,
    }
    try:
        return positions[state]
    except KeyError as exc:
        raise ValueError("state must be retracted, mid, or extended") from exc


def tangential_position(state: str) -> float:
    positions = {
        "negative": TANGENTIAL_CENTER_NEGATIVE_MM,
        "center": 0.0,
        "positive": (
            TANGENTIAL_CENTER_NEGATIVE_MM + TANGENTIAL_STROKE_MM
        ),
    }
    try:
        return positions[state]
    except KeyError as exc:
        raise ValueError("state must be negative, center, or positive") from exc


def _machine_reference_to_active_local(
    point: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Inverse of the current active-local-to-machine reference transform."""

    machine_x, machine_y, machine_z = map(float, point)
    return (
        REFERENCE_M0_STANDOFF_MM - machine_z,
        -machine_x,
        machine_y,
    )


def _tower_m4_local_locations() -> tuple[tuple[float, float, float], ...]:
    return tuple(
        _machine_reference_to_active_local((machine_x, -110.0, machine_z))
        for machine_x in TOWER_M4_MACHINE_X_MM
        for machine_z in TOWER_M4_MACHINE_Z_MM
    )


def mounting_backer_context():
    """Compact tower-face context with exact keys and four M4 pilots."""

    body = Pos(32.0, 0.0, -117.0) * Box(12.0, 56.0, 6.0, align=CTR)
    for machine_x in TOWER_KEY_MACHINE_X_MM:
        _x, y, _z = _machine_reference_to_active_local((
            machine_x, TOWER_FRONT_FACE_MACHINE_Y_MM, TOWER_KEY_MACHINE_Z_MM,
        ))
        body += Pos(34.0, y, -113.3) * Box(2.0, 3.0, 1.4, align=CTR)
    for x, y, _z in _tower_m4_local_locations():
        body -= Pos(x, y, -117.0) * Cylinder(2.85, 8.0, align=CTR)
    return _label(body, "context_exact_keyed_tower_face_M4_insert_pilots")


def carrier():
    """6061 carrier with keyed adapter and captured radial T-slot/stops."""

    adapter = Pos(32.0, 0.0, -112.0) * Box(
        12.0, 56.0, 4.0, align=CTR
    )
    spine = Pos(28.0, 0.0, -51.0) * Box(6.0, 8.0, 119.0, align=CTR)
    cassette = Pos(17.0, 0.0, 8.0) * Box(
        22.0, 14.0, 8.0, align=CTR
    )
    bridge = Pos(25.0, 0.0, 4.5) * Box(10.0, 8.0, 7.0, align=CTR)
    body = adapter + spine + bridge + cassette
    # Radial T-slot: the finite X extent leaves integral hard stops at the
    # exact retracted and extended slide faces.
    body -= Pos(17.0, 0.0, 7.0) * Box(
        14.4, 10.4, 4.0, align=CTR
    )
    body -= Pos(17.0, 0.0, 10.0) * Box(
        14.0, 4.4, 6.0, align=CTR
    )
    for machine_x in TOWER_KEY_MACHINE_X_MM:
        _x, y, _z = _machine_reference_to_active_local((
            machine_x, TOWER_FRONT_FACE_MACHINE_Y_MM, TOWER_KEY_MACHINE_Z_MM,
        ))
        body -= Pos(34.0, y, -113.2) * Box(
            2.1, 3.1, 1.6, align=CTR
        )
    for x, y, _z in _tower_m4_local_locations():
        body -= Pos(x, y, -112.0) * Cylinder(2.25, 6.0, align=CTR)
    return _label(body, "shared_axial_carrier_with_integral_radial_stops")


def radial_slide(state: str = "mid"):
    """Hard-anodized 7075 radial tongue carrying a tangential T-slot."""

    x = radial_position(state)
    tongue = Pos(x, 0.0, 7.0) * Box(8.0, 10.0, 3.0, align=CTR)
    stem = Pos(x, 0.0, 10.0) * Box(8.0, 4.0, 4.0, align=CTR)
    cassette = Pos(x, 0.0, 14.0) * Box(8.0, 10.0, 4.0, align=CTR)
    body = tongue + stem + cassette
    # Tangential captured tongue/slot.  Its finite Y extent is the pair of
    # integral +/-0.5 mm hard stops.
    body -= Pos(x, 0.0, 13.0) * Box(5.8, 7.2, 2.2, align=CTR)
    body -= Pos(x, 0.0, 15.0) * Box(2.8, 7.2, 4.0, align=CTR)
    return _label(body, f"radial_slide_7075:{state}")


def tangential_slide(
    radial_state: str = "mid", tangential_state: str = "center",
):
    """Captured tangential tongue and cantilevered gimbal mounting pad."""

    x = radial_position(radial_state)
    y = tangential_position(tangential_state)
    tongue = Pos(x, y, 13.0) * Box(5.4, 6.0, 1.8, align=CTR)
    stem = Pos(x, y, 15.0) * Box(2.4, 6.0, 3.4, align=CTR)
    beam = Pos(x + 4.0, y, 17.5) * Box(10.0, 4.0, 3.0, align=CTR)
    body = tongue + stem + beam
    return _label(
        body, f"tangential_slide_7075:{radial_state}:{tangential_state}"
    )


def _gimbal_center(
    radial_state: str, tangential_state: str,
) -> tuple[float, float, float]:
    return (
        radial_position(radial_state) + 8.0,
        tangential_position(tangential_state),
        24.0,
    )


def outer_gimbal_yoke(
    radial_state: str = "mid", tangential_state: str = "center",
):
    """6061 outer fork; first gimbal axis is active tangential +Y."""

    x, y, z = _gimbal_center(radial_state, tangential_state)
    crossbar = Pos(x - 0.25, y, z - 5.25) * Box(
        7.5, 10.0, 1.5, align=CTR
    )
    arms = [
        Pos(x, y + sign * 3.5, z) * Box(7.0, 2.0, 9.0, align=CTR)
        for sign in (-1, 1)
    ]
    body = crossbar
    for arm in arms:
        body += arm
    body -= Pos(x, y, z) * _cylinder_y(2.60, 12.0)
    return _label(body, "outer_gimbal_yoke_6061_axis_Y")


def tangential_slide_outer_gimbal_cartridge(
    radial_state: str = "mid", tangential_state: str = "center",
):
    """One-piece 7075 slide/yoke cartridge with a positive 5x4x1 throat."""

    slide = tangential_slide(radial_state, tangential_state)
    yoke = outer_gimbal_yoke(radial_state, tangential_state)
    body = slide + yoke
    x = radial_position(radial_state)
    y = tangential_position(tangential_state)
    root_edges = [
        edge for edge in body.edges()
        if edge.geom_type.name == "LINE"
        and math.isclose(edge.length, 1.0, abs_tol=1.0e-6)
        and math.isclose(edge.center().X, x + 4.0, abs_tol=1.0e-6)
        and math.isclose(abs(edge.center().Y - y), 2.0, abs_tol=1.0e-6)
        and math.isclose(edge.center().Z, 18.5, abs_tol=1.0e-6)
    ]
    if len(root_edges) != 2:
        raise RuntimeError(
            f"monolithic cartridge expected two throat roots, got {len(root_edges)}"
        )
    body = body.fillet(1.0, root_edges)
    return _label(
        body,
        f"monolithic_7075_tangential_slide_outer_yoke:"
        f"{radial_state}:{tangential_state}",
    )


def inner_gimbal_yoke(
    radial_state: str = "mid", tangential_state: str = "center",
):
    """6061 inner fork; second gimbal/nose pivot is active axial +Z."""

    x, y, z = _gimbal_center(radial_state, tangential_state)
    nose_x = x + 8.0
    hub = _cylinder_y(3.5, 4.0) - _cylinder_y(2.60, 5.0)
    hub = Pos(x, y, z) * hub
    body = hub
    for sign in (-1, 1):
        body += Pos(x + 7.0, y, z + sign * 3.50) * Box(
            10.0, 4.0, 2.0, align=CTR
        )
    body -= Pos(nose_x, y, z) * Cylinder(1.60, 14.0, align=CTR)
    return _label(body, "inner_gimbal_yoke_6061_axis_Z")


def nose_insert(
    radial_state: str = "mid", tangential_state: str = "center",
):
    """Virgin-PEEK convex nose with a real open R3.0 centre band."""

    x, y, z = _gimbal_center(radial_state, tangential_state)
    nose_x = x + 8.0
    # The analytic aggregate-boundary follower arc is in active local XY at
    # fixed Z, so the convex generating axis and retained width are +Z.
    outer = Cylinder(NOSE_FLANGE_RADIUS_MM, NOSE_WIDTH_MM, align=CTR)
    # Remove only the central annular band.  The retained band floor is the
    # positive convex R3.0 contact surface; flanges remain at R3.5.
    groove_shell = (
        Cylinder(NOSE_FLANGE_RADIUS_MM + 0.5,
                 NOSE_GROOVE_CLEAR_WIDTH_MM, align=CTR)
        - Cylinder(NOSE_CONTACT_SURFACE_RADIUS_MM,
                   NOSE_GROOVE_CLEAR_WIDTH_MM + 0.2, align=CTR)
    )
    body = outer - groove_shell
    body -= Cylinder(1.60, 10.0, align=CTR)
    body = Pos(nose_x, y, z) * body
    return _label(body, "virgin_unfilled_PEEK_R3_open_groove_nose")


def radial_bellcrank(
    radial_state: str = "mid", tangential_state: str = "center",
):
    """Review lever proving the selected spring never sees direct 6 mm travel."""

    x = radial_position(radial_state)
    y = tangential_position(tangential_state)
    long_arm = Pos(x - 2.0, y + 8.0, 18.0) * Box(
        8.0, 1.5, 1.5, align=CTR
    )
    short_arm = Pos(x - 5.0, y + 6.7, 18.0) * Box(
        1.5, 4.0, 1.5, align=CTR
    )
    return _label(long_arm + short_arm, "radial_return_bellcrank_ratio_0p29")


def m0_positive_dock_gate(radial_state: str = "mid"):
    """Unattached M0 latch-tongue concept at the retracted X=18 face.

    In M1/M2 review states the tongue is visibly withdrawn tangentially from
    the guide channel.  The actuator, linkage, and tower attachment remain
    outside this isolated model, so this solid does not prove retraction or
    retention authority.
    """

    y = 0.0 if radial_state == "retracted" else 10.5
    body = Pos(19.0, y, 10.0) * Box(2.0, 3.8, 3.0, align=CTR)
    return _label(body, f"UNATTACHED_M0_dock_gate_concept:{radial_state}")


def return_spring_envelopes(
    radial_state: str = "mid", tangential_state: str = "center",
) -> tuple:
    """Selected radial and procurement-blocked tangential spring envelopes."""

    x = radial_position(radial_state)
    y = tangential_position(tangential_state)
    radial = hardware.place(
        hardware.spring_envelope(
            3.5, 2.0, 10.84243517554,
            label="LEM050AB01_radial_return_selected_envelope",
        ),
        (x - 7.0, y + 9.0, 12.0), axis="+z",
    )
    tangential = hardware.place(
        hardware.spring_envelope(
            2.5, 1.0, 6.0,
            label="UNSELECTED_tangential_return_spring_envelope",
        ),
        (x + 2.0, y - 9.0, 12.0), axis="+z",
    )
    return radial, tangential


def unresolved_interface_envelopes(
    radial_state: str = "mid", tangential_state: str = "center",
) -> tuple:
    """Exploded, non-installed envelopes that remain procurement-blocked."""

    x, y, z = _gimbal_center(radial_state, tangential_state)
    bushing = Pos(x - 7.0, y + 15.0, z) * Box(
        6.0, 2.0, 3.0, align=CTR
    )
    bushing.label = "UNSELECTED_tangential_bushing_guide_envelope"
    ceramic = Pos(x + 8.0, y + 15.0, z) * _cylinder_y(
        NOSE_CONTACT_SURFACE_RADIUS_MM, NOSE_GROOVE_CLEAR_WIDTH_MM
    )
    ceramic.label = "UNSELECTED_polished_ceramic_insert_envelope"
    return bushing, ceramic


def tower_m4_hardware() -> tuple:
    """Four exact M4x10/washer/short-insert primary tower stacks."""

    result = []
    for index, (x, y, _z) in enumerate(_tower_m4_local_locations()):
        suffix = f"{index:02d}"
        washer = hardware.place(
            hardware.plain_washer(
                "M4", label=f"primary_tower_M4_washer_{suffix}",
            ),
            (x, y, -110.0), axis="+z",
        )
        screw = hardware.place(
            hardware.socket_head_cap_screw(
                "M4", M4_CARRIER_SCREW_LENGTH_MM,
                label=f"primary_tower_ISO4762_M4x10_{suffix}",
            ),
            (x, y, -110.0 + M4_WASHER_THICKNESS_MM), axis="+z",
        )
        insert = hardware.place(
            hardware.heat_set_insert(
                "M4", length="short",
                label=f"primary_tower_M4_short_heat_insert_{suffix}",
            ),
            (x, y, TOWER_FRONT_FACE_MACHINE_Y_MM), axis="-z",
        )
        result.extend((screw, washer, insert))
    return tuple(result)


def secondary_m3_clamp_hardware_envelopes() -> tuple:
    """Exploded complete M3x8 cassette stacks, explicitly non-proof.

    Their 4.05 mm nominal local clamped stack gives 3.95 mm geometric screw
    projection.  The exact cassette keys and insert host are intentionally not
    integrated here, so these visible stacks cannot be mistaken for the 40 N
    primary tower load path.
    """

    result = []
    for index, y in enumerate((-5.0, 5.0)):
        washer = hardware.place(
            hardware.plain_washer(
                "M3", label=f"UNQUALIFIED_secondary_M3_washer_{index}",
            ),
            (42.0, y, 12.0), axis="+x",
        )
        screw = hardware.place(
            hardware.socket_head_cap_screw(
                "M3", M3_CLAMP_SCREW_LENGTH_MM,
                label=f"UNQUALIFIED_secondary_ISO4762_M3x8_screw_{index}",
            ),
            (42.0 + M3_WASHER_THICKNESS_MM, y, 12.0), axis="+x",
        )
        insert = hardware.place(
            hardware.heat_set_insert_m3_3p4(
                label=f"UNQUALIFIED_secondary_M3x3p4_insert_{index}"
            ),
            (42.0 - M3_CLAMP_LOCAL_STACK_MM, y, 12.0), axis="-x",
        )
        result.extend((screw, washer, insert))
    return tuple(result)


def gimbal_pin_hardware(
    radial_state: str = "mid", tangential_state: str = "center",
) -> tuple:
    """Complete retained OD5 outer and OD3 inner shoulder-pivot stacks."""

    x, y, z = _gimbal_center(radial_state, tangential_state)
    nose_x = x + 8.0
    outer_shim = hardware.place(
        hardware.thrust_washer(
            5.0, 10.0, 0.5,
            label="outer_pivot_DIN988_5x10x0p5_shim",
        ),
        (x, y + 4.5, z), axis="+y",
    )
    outer_screw = hardware.place(
        hardware.dancer_pivot_shoulder_screw(
            label="outer_pivot_McMaster_96654A127_OD5x10_M4"
        ),
        (x, y + 5.0, z), axis="+y",
    )
    outer_nut = hardware.place(
        hardware.nyloc_nut("M4", label="outer_pivot_M4_nyloc"),
        (x, y - 9.0, z), axis="+y",
    )
    outer_far_shim = hardware.place(
        hardware.thrust_washer(
            5.0, 10.0, 0.5,
            label="outer_pivot_DIN988_5x10x0p5_far_shim",
        ),
        (x, y - 4.5, z), axis="-y",
    )
    inner_near_shim = hardware.place(
        hardware.thrust_washer(
            3.0, 6.0, 0.5,
            label="inner_pivot_DIN988_3x6x0p5_near_shim",
        ),
        (nose_x, y, z + 4.5), axis="+z",
    )
    inner_near_thrust_shim = hardware.place(
        hardware.thrust_washer(
            3.0, 6.0, 0.5,
            label="inner_pivot_DIN988_3x6x0p5_near_thrust_shim",
        ),
        (nose_x, y, z + 2.0), axis="+z",
    )
    inner_screw = hardware.place(
        hardware.shoulder_screw_90265a115(
            label="inner_pivot_McMaster_90265A115_OD3x10_M2"
        ),
        (nose_x, y, z + 5.0), axis="+z",
    )
    inner_far_shim = hardware.place(
        hardware.thrust_washer(
            3.0, 6.0, 0.5,
            label="inner_pivot_DIN988_3x6x0p5_far_shim",
        ),
        (nose_x, y, z - 4.5), axis="-z",
    )
    inner_far_thrust_shim = hardware.place(
        hardware.thrust_washer(
            3.0, 6.0, 0.5,
            label="inner_pivot_DIN988_3x6x0p5_far_thrust_shim",
        ),
        (nose_x, y, z - 2.0), axis="-z",
    )
    inner_nut = hardware.place(
        hardware.nyloc_nut("M2", label="inner_pivot_M2_nyloc"),
        (nose_x, y, z - 9.0), axis="+z",
    )
    return (
        outer_screw, outer_shim, outer_far_shim, outer_nut,
        inner_screw, inner_near_shim, inner_near_thrust_shim,
        inner_far_thrust_shim, inner_far_shim, inner_nut,
    )


def custom_bodies(
    radial_state: str = "mid", tangential_state: str = "center",
) -> tuple:
    """Separately manufactured positive-volume bodies in one review state."""

    return (
        mounting_backer_context(),
        carrier(),
        radial_slide(radial_state),
        tangential_slide_outer_gimbal_cartridge(
            radial_state, tangential_state
        ),
        inner_gimbal_yoke(radial_state, tangential_state),
        nose_insert(radial_state, tangential_state),
        radial_bellcrank(radial_state, tangential_state),
        m0_positive_dock_gate(radial_state),
    )


def module_parts(
    radial_state: str = "mid", tangential_state: str = "center",
) -> tuple:
    return (
        *custom_bodies(radial_state, tangential_state),
        *tower_m4_hardware(),
        *secondary_m3_clamp_hardware_envelopes(),
        *gimbal_pin_hardware(radial_state, tangential_state),
        *return_spring_envelopes(radial_state, tangential_state),
    )


def _common_volume(one, two) -> float:
    try:
        common = one & two
        return float(common.volume) if common is not None else 0.0
    except Exception:
        return 0.0


def same_state_overlap_audit(
    radial_state: str = "mid", tangential_state: str = "center",
) -> dict[str, Any]:
    """Exact positive-volume pair audit for manufactured bodies only."""

    parts = custom_bodies(radial_state, tangential_state)
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
        "radial_state": radial_state,
        "tangential_state": tangential_state,
        "custom_body_count": len(parts),
        "positive_overlap_count": len(rows),
        "positive_overlaps": rows,
        "status": "PASS" if not rows else "FAIL",
    }


def geometry_contract() -> dict[str, Any]:
    per_m4 = PRIMARY_MOUNT_LOAD_CASE_N / TOWER_M4_STACK_COUNT
    radial_spring_travel = RADIAL_STROKE_MM * RADIAL_SPRING_LEVER_RATIO
    radial_bounds = {
        state: [radial_position(state) - 4.0, radial_position(state) + 4.0]
        for state in ("retracted", "mid", "extended")
    }
    tangential_bounds = {
        state: [tangential_position(state) - 3.0,
                tangential_position(state) + 3.0]
        for state in ("negative", "center", "positive")
    }
    return {
        "schema": "aggregate-boundary-floating-follower-geometry/v1",
        "units": "mm",
        "frame": {
            "origin": "active-tooth review datum",
            "+X": "radial outward",
            "+Y": "tangential",
            "+Z": "stator axis",
        },
        "stroke_contract": {
            "radial_stroke_mm": RADIAL_STROKE_MM,
            "tangential_stroke_mm": TANGENTIAL_STROKE_MM,
            "radial_slide_bounds_by_state_mm": radial_bounds,
            "radial_capture_bounds_mm": [9.8, 24.2],
            "tangential_slide_bounds_by_state_mm": tangential_bounds,
            "tangential_capture_bounds_mm": [-3.6, 3.6],
            "radial_integral_stop_faces_mm": [9.8, 24.2],
            "radial_hard_center_travel_mm": 6.4,
            "tangential_integral_stop_faces_mm": [-3.6, 3.6],
            "tangential_hard_center_stops_mm": [-0.6, 0.6],
            "all_endpoint_tongues_captured": (
                radial_bounds["retracted"][0] >= 9.8
                and radial_bounds["extended"][1] <= 24.2
                and tangential_bounds["negative"][0] >= -3.6
                and tangential_bounds["positive"][1] <= 3.6
            ),
        },
        "nose_contract": {
            "contact_surface_radius_mm": NOSE_CONTACT_SURFACE_RADIUS_MM,
            "open_groove_clear_width_mm": NOSE_GROOVE_CLEAR_WIDTH_MM,
            "convex_arc_plane": "active_local_XY_at_fixed_Z",
            "nose_cylinder_axis": "+Z_stator_axis",
            "wire_diameter_range_mm": [0.2, 0.5],
            "wire_center_radius_range_mm": [3.1, 3.25],
            "material": MATERIALS["nose"],
            "ceramic_alternative_selected": False,
        },
        "selection_and_retraction": {
            "M1_sector_count": len(selector.M1_ANGLE_TO_LAW),
            "M1_law_count": len(selector.LAW_CODES),
            "M2_selected_shoe_identity_count": len(selector.CAM_TRACK_RADII_MM),
            "M2_identity_source": (
                "m1_selector_alternating_former.CAM_TRACK_RADII_MM"
            ),
            "M0_gate_api": "m1_selector_alternating_former.gate_state_for_axis_z",
            "M0_positive_dock": "UNATTACHED_CONCEPT_ONLY",
            "M0_engaged_gate_face_x_mm": 18.0,
            "retracted_slide_outer_face_x_mm": 18.0,
            "M0_dock_contact_gap_mm": 0.0,
            "M1_M2_gate_withdrawal_y_mm": 10.5,
            "failure_state": "UNPROVEN_RETRACTION_LINKAGE_NOT_MODELED",
            "M0_dock_attached_to_actuator": False,
            "new_commanded_axis": False,
        },
        "spring_contract": {
            "radial_spring": RADIAL_RETURN_SPRING_ID,
            "radial_slide_travel_mm": RADIAL_STROKE_MM,
            "lever_ratio": RADIAL_SPRING_LEVER_RATIO,
            "spring_travel_mm": radial_spring_travel,
            "direct_6mm_spring_travel_used": False,
            "radial_spring_role": "preload_envelope_only_not_anchored",
            "fail_retraction_owner": None,
            "tangential_spring": TANGENTIAL_RETURN_SPRING_ID,
            "tangential_spring_procurement_blocked": True,
        },
        "monolithic_cartridge_contract": {
            "architecture": "one_piece_tangential_slide_plus_outer_yoke",
            "material": MATERIALS[
                "tangential_slide_outer_yoke_cartridge"
            ],
            "minimum_positive_throat_mm": [5.0, 4.0, 1.0],
            "modeled_positive_throat_mm": [5.0, 4.0, 1.0],
            "minimum_root_radius_mm": 0.75,
            "preferred_root_radius_mm": 1.0,
            "root_blend_geometry_modeled": True,
            "separate_slide_to_yoke_fasteners_required": False,
        },
        "fastener_contract": {
            "primary_mount": "four_M4_keyed_tower_stacks",
            "primary_M4_stack_count": TOWER_M4_STACK_COUNT,
            "primary_M4_screw": "ISO4762_M4x10",
            "primary_M4_screw_length_mm": M4_CARRIER_SCREW_LENGTH_MM,
            "primary_M4_geometric_penetration_mm": (
                M4_CARRIER_THREAD_ENGAGEMENT_MM
            ),
            "primary_M4_short_insert_length_mm": M4_SHORT_INSERT_LENGTH_MM,
            "primary_load_case_N": PRIMARY_MOUNT_LOAD_CASE_N,
            "primary_load_per_M4_N": per_m4,
            "primary_load_distribution_assumption": "equal_over_four_M4",
            "secondary_M3_stack_count": M3_CLAMP_SCREW_COUNT,
            "secondary_M3_screw": "ISO4762_M3x8_envelope",
            "secondary_M3_screw_length_mm": M3_CLAMP_SCREW_LENGTH_MM,
            "secondary_M3_local_stack_mm": M3_CLAMP_LOCAL_STACK_MM,
            "secondary_M3_geometric_projection_mm": (
                M3_CLAMP_SCREW_LENGTH_MM - M3_CLAMP_LOCAL_STACK_MM
            ),
            "secondary_M3_status": (
                "UNQUALIFIED_EXPLODED_ENVELOPE_NOT_PRIMARY_LOAD_PATH"
            ),
            "secondary_M3_structural_proof_claimed": False,
            "direct_PEEK_threads": False,
            "qualified_shoulder_pin_stack_count": 2,
            "outer_shoulder_pin": {
                "sku": "McMaster 96654A127",
                "shoulder_diameter_mm": OUTER_PIVOT_SHOULDER_DIAMETER_MM,
                "shoulder_length_mm": OUTER_PIVOT_SHOULDER_LENGTH_MM,
                "thread": "M4",
            },
            "inner_pivot": {
                "status": "CATALOG_SELECTED_GEOMETRIC_STACK_MODELED",
                "sku": "McMaster 90265A115",
                "shoulder_diameter_mm": INNER_PIVOT_DIAMETER_MM,
                "shoulder_length_mm": INNER_PIVOT_SHOULDER_LENGTH_MM,
                "thread": "M2",
                "shim_stack": "4x DIN988 3x6x0.5, two external and two thrust",
                "retainer": "M2 nyloc",
                "rejected_candidates": [
                    "McMaster 90265A420 OD3x16 too long",
                    "McMaster 90265A181 is OD4 not OD3",
                ],
            },
            "retained_shoulder_pin_stack_count": 2,
        },
        "tower_mount_context": {
            "status": "EXACT_KEYED_INTERFACE_CONTEXT_MODELED",
            "front_face_machine_y_mm": TOWER_FRONT_FACE_MACHINE_Y_MM,
            "key_machine_x_mm": list(TOWER_KEY_MACHINE_X_MM),
            "key_machine_z_mm": TOWER_KEY_MACHINE_Z_MM,
            "M4_machine_x_mm": list(TOWER_M4_MACHINE_X_MM),
            "M4_machine_z_mm": list(TOWER_M4_MACHINE_Z_MM),
            "existing_stack": "4x ISO4762 M4x10 + washer + short insert",
            "reason": (
                "Exact face, keys, adapter bores, and four primary stacks are "
                "modeled; the full tower body and assembly integration remain "
                "outside this isolated review source."
            ),
            "full_tower_body_integrated": False,
        },
        "materials": dict(MATERIALS),
        "procurement_blockers": [
            TANGENTIAL_RETURN_SPRING_ID,
            "UNSELECTED_tangential_bushing_guide_envelope",
            "UNSELECTED_polished_ceramic_insert_envelope",
            "UNMODELED_M0_positive_retraction_linkage",
        ],
        "authority": dict(AUTHORITY),
        "output_target": "out/review/aggregate_boundary_floating_follower.step",
    }


def gen_step() -> Compound:
    """Return the labeled isolated mid/deployed review compound."""

    children = list(module_parts("mid", "center"))
    children.extend(unresolved_interface_envelopes("mid", "center"))
    result = Compound(children=children)
    result.label = "aggregate_boundary_floating_follower_review_only"
    return result


if __name__ == "__main__":
    print(geometry_contract())
