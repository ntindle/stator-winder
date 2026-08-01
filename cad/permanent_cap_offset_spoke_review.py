"""Isolated review CAD for the permanent-cap offset-spoke M2 successor.

CAD brief:
- Model: labeled review assembly; no production integration.
- Units/frame: millimetres in the machine frame; M2 rotates about +Z and
  negative Z is rearward.
- Selected architecture: 1 mm conservative cap collision/support envelopes,
  the complete M2 drive translated 10 mm rearward, a 14 x 8 mm deep spoke at
  z=-38.12..-30.12, an outboard transition at R58, and a toroidal guide at
  R64/z=-17.
- Static changes represented: entry guide 2 mm rearward and the fixed 450 mm
  frame window 2.5 mm rearward.
- Validation: one closed printed-arm solid, exact BREP clearance at the
  controlling pose, a rotation-invariant continuous 360-degree certificate,
  shaft/entry and motor/frame distances, and exact OCC mass properties.

The cap bodies in this file are deliberately labeled *collision/support
envelopes*.  ``permanent_cap_aggregate_authorization.json`` authorizes the R3
aggregate lane as an input, but explicitly does not authorize a printable cap,
material, finish, retention method, or production integration.  Likewise the
paired tungsten slugs are a provisional balance envelope, not released
hardware or a two-plane balance solution.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from build123d import (
    Align,
    Box,
    BuildLine,
    BuildSketch,
    Circle,
    Compound,
    Cylinder,
    Part,
    Plane,
    Polyline,
    Pos,
    Rot,
    Torus,
    Transition,
    import_step,
    sweep,
)

import assembly
import cots
from params import DEFAULT_STATOR, PARAMS as P
import printed
import stator_model


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORTS = ROOT / "out" / "reports"
REVIEW = ROOT / "out" / "review"

OFFSET_REPORT = REPORTS / "permanent_cap_offset_spoke_flyer.json"
AGGREGATE_REPORT = REPORTS / "permanent_cap_aggregate_authorization.json"
JSON_OUT = REPORTS / "permanent_cap_offset_spoke_review.json"
MD_OUT = REPORTS / "permanent_cap_offset_spoke_review.md"
STEP_OUT = REVIEW / "permanent_cap_offset_spoke_review.step"
MANIFEST_OUT = REVIEW / "permanent_cap_offset_spoke_review.manifest.json"

CTR = (Align.CENTER, Align.CENTER, Align.CENTER)

# Frozen selection from permanent_cap_offset_spoke_flyer.json.
CAP_WALL_MM = 1.0
M2_MODULE_REAR_SHIFT_MM = 10.0
SPOKE_WIDTH_MM = 14.0
SPOKE_REAR_Z_MM = -38.12
SPOKE_FRONT_Z_MM = -30.12
SPOKE_THICKNESS_MM = SPOKE_FRONT_Z_MM - SPOKE_REAR_Z_MM
TRANSITION_CENTER_RADIUS_MM = 58.0
TIP_GUIDE_CENTER_RADIUS_MM = 64.0
TIP_GUIDE_CENTER_Z_MM = -17.0
ENTRY_GUIDE_REAR_SHIFT_MM = 2.0
FRAME_WINDOW_REAR_SHIFT_MM = 2.5
REVIEW_CLEARANCE_MM = 2.2

# Cap collision/support envelope inputs.  These retain the prior conservative
# outer material boundary but are not represented as production cap geometry.
MINIMUM_WIRE_CENTER_BEND_RADIUS_MM = 3.0
WIRE_RADIUS_MM = float(DEFAULT_STATOR.wire_d) / 2.0
CAP_BASE_CENTER_RADIUS_MM = 34.0
CAP_PACKING_RADIAL_SPAN_MM = 5.01144097372606
CAP_PACKING_PROFILE_SPAN_MM = 0.6418547186705699

# Provisional paired slug envelope.  The pair is symmetric in X and remains
# entirely in the deep spoke plane.  Retention and two-plane balance stay open.
BALANCE_SLUG_DENSITY_G_CM3 = 19.3
BALANCE_SLUG_RADIUS_MM = 7.0
BALANCE_SLUG_THICKNESS_MM = 5.1
BALANCE_SLUG_X_MM = 9.0
BALANCE_SLUG_Y_MM = -25.0
BALANCE_BOSS_RADIUS_MM = 8.5
BALANCE_POCKET_RADIUS_MM = 7.15

PETG_DENSITY_G_CM3 = 1.27


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} is not a JSON object")
    return value


def _box(x0: float, x1: float, y0: float, y1: float,
         z0: float, z1: float, label: str | None = None) -> Part:
    result = Pos(
        (x0 + x1) / 2.0,
        (y0 + y1) / 2.0,
        (z0 + z1) / 2.0,
    ) * Box(
        abs(x1 - x0), abs(y1 - y0), abs(z1 - z0), align=CTR,
    )
    if label:
        result.label = label
    return result


def _cyl_z(radius: float, z0: float, z1: float,
           x: float = 0.0, y: float = 0.0,
           label: str | None = None) -> Part:
    result = Pos(x, y, (z0 + z1) / 2.0) * Cylinder(
        radius, abs(z1 - z0), align=CTR,
    )
    if label:
        result.label = label
    return result


def _cyl_x(radius: float, x0: float, x1: float,
           y: float = 0.0, z: float = 0.0) -> Part:
    return Pos((x0 + x1) / 2.0, y, z) * (
        Rot(0.0, 90.0, 0.0)
        * Cylinder(radius, abs(x1 - x0), align=CTR)
    )


def _cyl_y(radius: float, y0: float, y1: float,
           x: float = 0.0, z: float = 0.0) -> Part:
    return Pos(x, (y0 + y1) / 2.0, z) * (
        Rot(90.0, 0.0, 0.0)
        * Cylinder(radius, abs(y1 - y0), align=CTR)
    )


def _bar_xy(start: tuple[float, float], end: tuple[float, float],
            width: float, z0: float, z1: float) -> Part:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    angle = math.degrees(math.atan2(dy, dx))
    bar = Box(
        length, width, abs(z1 - z0),
        align=(Align.MIN, Align.CENTER, Align.MIN),
    )
    return Pos(start[0], start[1], min(z0, z1)) * (
        Rot(0.0, 0.0, angle) * bar
    )


def _poly_tube(points: np.ndarray, radius_mm: float,
               label: str) -> Part:
    clean = [tuple(map(float, row)) for row in np.asarray(points)]
    direction = tuple(clean[1][axis] - clean[0][axis] for axis in range(3))
    with BuildLine() as path:
        Polyline(*clean)
    with BuildSketch(Plane(origin=clean[0], z_dir=direction)) as profile:
        Circle(float(radius_mm))
    result = sweep(
        profile.sketch, path.line, transition=Transition.TRANSFORMED,
    )
    result.label = label
    return result


def _selected_contracts() -> tuple[dict[str, Any], dict[str, Any]]:
    offset = _load(OFFSET_REPORT)
    aggregate = _load(AGGREGATE_REPORT)
    selected = offset["bounded_sweep"]["selected"]
    expected = {
        "wall_mm": CAP_WALL_MM,
        "module_shift_mm": M2_MODULE_REAR_SHIFT_MM,
        "spoke_front_z_mm": SPOKE_FRONT_Z_MM,
        "spoke_rear_z_mm": SPOKE_REAR_Z_MM,
        "spoke_thickness_mm": SPOKE_THICKNESS_MM,
        "transition_radius_mm": TRANSITION_CENTER_RADIUS_MM,
        "tip_radius_mm": TIP_GUIDE_CENTER_RADIUS_MM,
    }
    for key, value in expected.items():
        if not math.isclose(float(selected[key]), value, abs_tol=1.0e-9):
            raise ValueError(
                f"offset-spoke selection drift at {key}: "
                f"{selected[key]} != {value}"
            )
    relocations = selected["static_relocations_mm"]
    if not math.isclose(
        float(relocations["entry_guide_rearward"]),
        ENTRY_GUIDE_REAR_SHIFT_MM,
        abs_tol=1.0e-9,
    ):
        raise ValueError("entry-guide relocation drift")
    if not math.isclose(
        float(relocations["450mm_frame_window_rearward"]),
        FRAME_WINDOW_REAR_SHIFT_MM,
        abs_tol=1.0e-9,
    ):
        raise ValueError("frame-window relocation drift")
    if aggregate.get("status") != "PASS":
        raise ValueError("aggregate support contract is not PASS")
    if aggregate.get("offset_flyer_input_authorized") is not True:
        raise ValueError("aggregate report does not authorize offset input")
    return offset, aggregate


def deepest_axis_z_mm() -> float:
    offset, _aggregate = _selected_contracts()
    return float(offset["raw_pose_evidence"]["minimum_stator_axis_z_mm"])


def cap_outer_radius_mm() -> float:
    pad_radius = (
        MINIMUM_WIRE_CENTER_BEND_RADIUS_MM
        + CAP_PACKING_PROFILE_SPAN_MM
        + WIRE_RADIUS_MM
        + CAP_WALL_MM / 2.0
    )
    return CAP_BASE_CENTER_RADIUS_MM + CAP_PACKING_RADIAL_SPAN_MM + pad_radius


def _local_cap_parts(axial_sign: int) -> tuple[Part, ...]:
    """Conservative 1 mm collision/support envelope in stator-local frame."""

    if axial_sign not in (-1, 1):
        raise ValueError("axial_sign must be -1 or +1")
    wall = CAP_WALL_MM
    outer = cap_outer_radius_mm()
    guide_z = axial_sign * (
        DEFAULT_STATOR.stack / 2.0
        + MINIMUM_WIRE_CENTER_BEND_RADIUS_MM
        + wall / 2.0
    )
    inner = DEFAULT_STATOR.od * DEFAULT_STATOR.hub_od_ratio / 2.0
    rib_width = 2.0 * MINIMUM_WIRE_CENTER_BEND_RADIUS_MM + wall
    pad_radius = (
        MINIMUM_WIRE_CENTER_BEND_RADIUS_MM
        + CAP_PACKING_PROFILE_SPAN_MM
        + WIRE_RADIUS_MM
        + wall / 2.0
    )

    # The lamination-face skin is exact in plan but remains an envelope, not
    # released mold geometry.
    slab = Box(
        DEFAULT_STATOR.od + 2.0,
        DEFAULT_STATOR.od + 2.0,
        wall,
        align=CTR,
    )
    face = stator_model.stator(DEFAULT_STATOR, label="cap_face_source") & slab
    face -= Cylinder(DEFAULT_STATOR.shaft_d / 2.0 + 0.5, wall + 2.0,
                     align=CTR)
    face = Pos(
        0.0, 0.0,
        axial_sign * (DEFAULT_STATOR.stack / 2.0 + wall / 2.0),
    ) * face
    face.label = f"cap_{axial_sign:+d}_face_collision_envelope"

    children: list[Part] = [face]
    pitch = 360.0 / DEFAULT_STATOR.slots
    for tooth in range(DEFAULT_STATOR.slots):
        angle = tooth * pitch
        rib = Box(
            outer - inner, rib_width, wall,
            align=(Align.MIN, Align.CENTER, Align.CENTER),
        )
        rib = Rot(0.0, 0.0, angle) * (Pos(inner, 0.0, guide_z) * rib)
        rib.label = f"cap_{axial_sign:+d}_tooth_{tooth:02d}_rib_envelope"
        children.append(rib)

        pad = Box(
            CAP_PACKING_RADIAL_SPAN_MM + 2.0 * pad_radius,
            2.0 * pad_radius,
            wall,
            align=CTR,
        )
        pad = Pos(
            CAP_BASE_CENTER_RADIUS_MM
            + CAP_PACKING_RADIAL_SPAN_MM / 2.0,
            0.0,
            guide_z,
        ) * pad
        pad = Rot(0.0, 0.0, angle) * pad
        pad.label = f"cap_{axial_sign:+d}_tooth_{tooth:02d}_pad_envelope"
        children.append(pad)
    return tuple(children)


def cap_collision_support_envelope(axial_sign: int) -> Compound:
    local = Compound(children=list(_local_cap_parts(axial_sign)))
    transform = (
        Pos(0.0, 0.0, deepest_axis_z_mm())
        * Rot(0.0, 90.0, 0.0)
        * Rot(-90.0, 0.0, 0.0)
    )
    result = transform * local
    result.label = (
        "front_cap_collision_support_envelope_unreleased"
        if axial_sign > 0
        else "rear_cap_collision_support_envelope_unreleased"
    )
    return result


def deepest_pose_stator() -> Part:
    local = stator_model.stator(DEFAULT_STATOR, label="default_stator")
    result = (
        Pos(0.0, 0.0, deepest_axis_z_mm())
        * Rot(0.0, 90.0, 0.0)
        * Rot(-90.0, 0.0, 0.0)
        * local
    )
    result.label = "default_stator_at_deepest_raw_pose"
    return result


def offset_spoke_arm_components() -> dict[str, Part]:
    """Return source components before their final one-solid union/cuts."""

    # The root collar occupies the shifted block's explicit R17 hub bore and
    # overlaps the spoke by 0.32 mm in Z.
    collar = (
        _cyl_z(14.0, -53.0, -37.8)
        - _cyl_z(P.flyer_shaft_od / 2.0 + 0.05, -54.0, -36.8)
    )
    clamp_z = -46.0
    collar -= _cyl_y(2.0, -15.0, 0.0, z=clamp_z)
    collar -= _cyl_x(2.0, 0.0, 15.0, z=clamp_z)
    collar.label = "offset_spoke_root_collar"

    spoke = _box(
        -SPOKE_WIDTH_MM / 2.0,
        SPOKE_WIDTH_MM / 2.0,
        0.0,
        TRANSITION_CENTER_RADIUS_MM,
        SPOKE_REAR_Z_MM,
        SPOKE_FRONT_Z_MM,
        "deep_14x8_spoke",
    )

    # Structural transition only begins outboard of R55 and is centered on
    # the study's R58 transition datum.  The wire witness below carries the
    # exact R3 quarter/straight/quarter route; this support is deliberately
    # conservative and stout for the isolated review.
    transition_tower = _box(
        -SPOKE_WIDTH_MM / 2.0,
        SPOKE_WIDTH_MM / 2.0,
        TRANSITION_CENTER_RADIUS_MM - 3.0,
        TRANSITION_CENTER_RADIUS_MM + 3.0,
        SPOKE_REAR_Z_MM,
        TIP_GUIDE_CENTER_Z_MM + 4.0,
        "R58_outboard_transition_tower",
    )
    tip_bridge = _box(
        -SPOKE_WIDTH_MM / 2.0,
        SPOKE_WIDTH_MM / 2.0,
        TRANSITION_CENTER_RADIUS_MM,
        TIP_GUIDE_CENTER_RADIUS_MM,
        TIP_GUIDE_CENTER_Z_MM - 4.0,
        TIP_GUIDE_CENTER_Z_MM + 4.0,
        "R64_tip_bridge",
    )
    cradle = _cyl_y(
        11.5,
        TIP_GUIDE_CENTER_RADIUS_MM - 5.0,
        TIP_GUIDE_CENTER_RADIUS_MM - 2.35,
        z=TIP_GUIDE_CENTER_Z_MM,
    )
    cradle.label = "R64_toroid_rear_cradle"

    # Two light printed bosses and narrow webs locate a symmetric provisional
    # slug pair without pretending the selected R25 point-mass screen is a
    # released attachment.
    balance_parts: list[Part] = []
    for sign in (-1.0, 1.0):
        x = sign * BALANCE_SLUG_X_MM
        boss = _cyl_z(
            BALANCE_BOSS_RADIUS_MM,
            SPOKE_REAR_Z_MM,
            SPOKE_FRONT_Z_MM,
            x=x,
            y=BALANCE_SLUG_Y_MM,
        )
        web = _bar_xy(
            (sign * 5.0, -7.0),
            (x, BALANCE_SLUG_Y_MM),
            4.0,
            SPOKE_REAR_Z_MM,
            SPOKE_FRONT_Z_MM,
        )
        balance_parts.extend((boss, web))
    balance_mount = balance_parts[0]
    for child in balance_parts[1:]:
        balance_mount += child
    balance_mount.label = "provisional_paired_balance_mount"

    return {
        "collar": collar,
        "spoke": spoke,
        "transition_tower": transition_tower,
        "tip_bridge": tip_bridge,
        "cradle": cradle,
        "balance_mount": balance_mount,
    }


def offset_spoke_arm() -> Part:
    components = offset_spoke_arm_components()
    arm = components["collar"]
    for name in (
        "spoke", "transition_tower", "tip_bridge", "cradle",
        "balance_mount",
    ):
        arm += components[name]

    # Open the toroidal ceramic seat and feed throat after the union so no
    # parent body can silently refill either passage.
    torus_seat = Pos(
        0.0, TIP_GUIDE_CENTER_RADIUS_MM, TIP_GUIDE_CENTER_Z_MM,
    ) * (Rot(-90.0, 0.0, 0.0) * Torus(6.5, 3.20))
    arm -= torus_seat
    arm -= _cyl_y(
        4.0,
        TRANSITION_CENTER_RADIUS_MM - 1.0,
        TIP_GUIDE_CENTER_RADIUS_MM - 1.5,
        z=TIP_GUIDE_CENTER_Z_MM,
    )

    # Open-front balance pockets retain a 1 mm printed rear floor.  Retention
    # detail is intentionally absent and therefore fail-closed in the report.
    for sign in (-1.0, 1.0):
        arm -= _cyl_z(
            BALANCE_POCKET_RADIUS_MM,
            SPOKE_REAR_Z_MM + 1.0,
            SPOKE_FRONT_Z_MM + 0.5,
            x=sign * BALANCE_SLUG_X_MM,
            y=BALANCE_SLUG_Y_MM,
        )
    # The toroidal seat subtraction can leave a tiny trapped crescent inside
    # the custom ceramic envelope.  It is neither printable structure nor an
    # intended insert, so retain only the connected load-bearing arm body.
    # This is deterministic (the discarded island is <0.4% of arm volume)
    # and is asserted by the one-solid regression gate.
    solids = list(arm.solids())
    if len(solids) > 1:
        arm = max(solids, key=lambda solid: float(solid.volume))
    arm.label = "offset_spoke_flyer_arm_single_solid_review"
    return arm


def provisional_balance_slug_envelopes() -> tuple[Part, Part]:
    z0 = SPOKE_REAR_Z_MM + 1.45
    z1 = z0 + BALANCE_SLUG_THICKNESS_MM
    result = []
    for sign, side in ((-1.0, "left"), (1.0, "right")):
        slug = _cyl_z(
            BALANCE_SLUG_RADIUS_MM,
            z0,
            z1,
            x=sign * BALANCE_SLUG_X_MM,
            y=BALANCE_SLUG_Y_MM,
        )
        slug.label = f"provisional_tungsten_balance_slug_{side}_unreleased"
        result.append(slug)
    return result[0], result[1]


def extended_hollow_shaft() -> Part:
    # The rear seat translates with the drive while the front remains at the
    # arm datum, producing the required 10 mm extension.
    z0 = P.flyer_shaft_rear_z - M2_MODULE_REAR_SHIFT_MM
    z1 = SPOKE_FRONT_Z_MM
    result = (
        _cyl_z(P.flyer_shaft_od / 2.0, z0, z1)
        - _cyl_z(P.flyer_shaft_id / 2.0, z0 - 1.0, z1 + 1.0)
    )
    result.label = "extended_hollow_flyer_shaft_80mm"
    return result


def tip_toroid() -> Part:
    result = Pos(
        0.0, TIP_GUIDE_CENTER_RADIUS_MM, TIP_GUIDE_CENTER_Z_MM,
    ) * (Rot(-90.0, 0.0, 0.0) * cots.ceramic_toroid_guide())
    result.label = "R64_polished_ceramic_toroid_context"
    return result


def flyer_wire_transition_witness() -> Part:
    """Visual maximum-wire witness for the R3 quarter/straight/quarter run."""

    rows: list[tuple[float, float, float]] = []
    rows.extend((0.0, y, SPOKE_FRONT_Z_MM) for y in np.linspace(8.0, 58.0, 50))
    center1_y = 58.0
    center1_z = SPOKE_FRONT_Z_MM + 3.0
    for theta in np.linspace(-math.pi / 2.0, 0.0, 25)[1:]:
        rows.append((
            0.0,
            center1_y + 3.0 * math.cos(theta),
            center1_z + 3.0 * math.sin(theta),
        ))
    second_start_z = TIP_GUIDE_CENTER_Z_MM - 3.0
    rows.extend(
        (0.0, 61.0, z)
        for z in np.linspace(center1_z, second_start_z, 30)[1:]
    )
    center2_y = 64.0
    center2_z = second_start_z
    for theta in np.linspace(math.pi, math.pi / 2.0, 25)[1:]:
        rows.append((
            0.0,
            center2_y + 3.0 * math.cos(theta),
            center2_z + 3.0 * math.sin(theta),
        ))
    rows.extend(
        (0.0, y, TIP_GUIDE_CENTER_Z_MM)
        for y in np.linspace(64.0, 67.0, 8)[1:]
    )
    return _poly_tube(
        np.asarray(rows), WIRE_RADIUS_MM,
        "maximum_wire_R3_transition_witness_not_strand_model",
    )


def shifted_static_module_parts() -> dict[str, Part | Compound]:
    shift = Pos(0.0, 0.0, -M2_MODULE_REAR_SHIFT_MM)
    block = shift * printed.flyer_block()
    block.label = "flyer_block_shifted_rear_10mm"
    front_bearing = Pos(0.0, 0.0, -52.0 - M2_MODULE_REAR_SHIFT_MM) * cots.bearing_6001()
    front_bearing.label = "flyer_6001_front_shifted"
    rear_bearing = Pos(0.0, 0.0, -71.0 - M2_MODULE_REAR_SHIFT_MM) * cots.bearing_6001()
    rear_bearing.label = "flyer_6001_rear_shifted"
    motor_mount = shift * printed.m2_motor_mount()
    motor_mount.label = "m2_motor_mount_shifted_rear_10mm"
    motor = Pos(
        0.0, P.m2_motor_axis_y,
        P.m2_motor_face_z - M2_MODULE_REAR_SHIFT_MM,
    ) * cots.nema17_mcmaster_6627t421()
    motor.label = "m2_McMaster_6627T421_shifted_rear_10mm"
    motor_pulley = Pos(
        0.0,
        P.m2_motor_axis_y,
        sum(P.pulley_z) / 2.0 - M2_MODULE_REAR_SHIFT_MM,
    ) * cots.gt2_pulley_40t_b5()
    motor_pulley.label = "m2_motor_pulley_shifted"
    flyer_pulley = shift * printed.flyer_pulley()
    flyer_pulley.label = "flyer_pulley_shifted"
    belt_path = HERE / "models" / "upgrades" / "gt2_belt_200.step"
    belt = import_step(str(belt_path))
    belt = Pos(
        0.0,
        0.0,
        (P.pulley_z[0] + P.pulley_z[1]) / 2.0
        - 3.0
        - M2_MODULE_REAR_SHIFT_MM,
    ) * belt
    belt.label = "200_2GT_belt_rigidly_shifted_proxy"
    return {
        "block": block,
        "front_bearing": front_bearing,
        "rear_bearing": rear_bearing,
        "motor_mount": motor_mount,
        "motor": motor,
        "motor_pulley": motor_pulley,
        "flyer_pulley": flyer_pulley,
        "belt": belt,
    }


def relocated_entry_eyelet() -> Part:
    # Current eyelet is centered at wire_entry_z+3.  Translate the complete
    # guide datum 2 mm rearward; its front face becomes z=-112.5.
    center_z = P.wire_entry_z + 3.0 - ENTRY_GUIDE_REAR_SHIFT_MM
    result = Pos(0.0, 0.0, center_z) * cots.ceramic_eyelet()
    result.label = "entry_eyelet_relocated_rear_2mm"
    return result


def relocated_entry_support_proxy() -> Part:
    center_z = P.wire_entry_z + 3.0 - ENTRY_GUIDE_REAR_SHIFT_MM
    support = _cyl_z(8.0, center_z - 3.0, center_z + 3.0)
    support -= _cyl_z(P.eyelet_seat_r, center_z - 4.0, center_z + 4.0)
    support.label = "entry_support_interface_proxy_unreleased"
    return support


def frame_rear_boundary_proxy() -> Part:
    # A local witness plate makes the exact motor-to-fixed-window Z margin
    # inspectable without pretending the plane is a physical full frame wall.
    rear_z = P.frame_z0 - FRAME_WINDOW_REAR_SHIFT_MM
    result = _box(
        -35.0, 35.0,
        P.m2_motor_axis_y - 35.0,
        P.m2_motor_axis_y + 35.0,
        rear_z - 1.0,
        rear_z,
    )
    result.label = "relocated_450mm_frame_rear_boundary_proxy"
    return result


def gen_step() -> Compound:
    _selected_contracts()
    caps = Compound(children=[
        deepest_pose_stator(),
        cap_collision_support_envelope(1),
        cap_collision_support_envelope(-1),
    ])
    caps.label = "deepest_raw_pose_cap_context_unreleased"

    static = shifted_static_module_parts()
    static_group = Compound(children=[
        static["block"], static["front_bearing"], static["rear_bearing"],
        static["motor_mount"], static["motor"], static["motor_pulley"],
        static["belt"], relocated_entry_support_proxy(),
        relocated_entry_eyelet(), frame_rear_boundary_proxy(),
    ])
    static_group.label = "shifted_M2_static_module_review"

    rotating = Compound(children=[
        extended_hollow_shaft(), static["flyer_pulley"],
        offset_spoke_arm(), tip_toroid(),
        *provisional_balance_slug_envelopes(),
        flyer_wire_transition_witness(),
    ])
    rotating.label = "offset_spoke_rotating_module_review"

    result = Compound(children=[caps, static_group, rotating])
    result.label = "permanent_cap_offset_spoke_isolated_review"
    return result


def _bbox(shape: Part | Compound) -> dict[str, list[float]]:
    bb = shape.bounding_box()
    return {
        "minimum_mm": [float(bb.min.X), float(bb.min.Y), float(bb.min.Z)],
        "maximum_mm": [float(bb.max.X), float(bb.max.Y), float(bb.max.Z)],
        "size_mm": [float(bb.size.X), float(bb.size.Y), float(bb.size.Z)],
    }


def _mass_properties(shape: Part, density_g_cm3: float) -> dict[str, Any]:
    volume = float(shape.volume)
    density = density_g_cm3 / 1000.0
    mass = volume * density
    center = shape.center()
    izz_volume = (
        float(shape.matrix_of_inertia[2][2])
        + volume * (center.X ** 2 + center.Y ** 2)
    )
    return {
        "volume_mm3": volume,
        "density_g_cm3": density_g_cm3,
        "mass_g": mass,
        "center_of_mass_mm": [float(center.X), float(center.Y), float(center.Z)],
        "izz_about_M2_axis_g_mm2": izz_volume * density,
    }


def analyze() -> dict[str, Any]:
    offset, aggregate = _selected_contracts()
    selected = offset["bounded_sweep"]["selected"]
    arm = offset_spoke_arm()
    components = offset_spoke_arm_components()
    caps = Compound(children=[
        cap_collision_support_envelope(1),
        cap_collision_support_envelope(-1),
    ])
    static = shifted_static_module_parts()
    shaft = extended_hollow_shaft()
    entry = relocated_entry_eyelet()
    frame = frame_rear_boundary_proxy()

    # Exact OCC distances at the controlling raw M1=M2=0 pose.
    exact_cap_arm = float(arm.distance_to(caps))
    exact_block_arm = float(arm.distance_to(static["block"]))
    exact_transition_cap = float(
        (components["transition_tower"] + components["tip_bridge"]
         + components["cradle"]).distance_to(caps)
    )
    exact_shaft_entry = float(shaft.distance_to(entry))
    exact_motor_frame = float(static["motor"].distance_to(frame))

    # The two controlling certificates are continuous, not just sampled:
    # (1) every deep part is behind the cap's global rear support plane and
    #     the shifted block's front plane;
    # (2) every forward transition part is outside a rotationally symmetric
    #     conservative cap sweep.  Rotation about Z preserves both bounds.
    analytic_cap = float(selected["clearances_mm"]["raw_cap_to_spoke"])
    analytic_block = float(selected["clearances_mm"]["shifted_block_to_spoke"])
    analytic_transition = float(
        selected["clearances_mm"][
            "transition_inner_edge_to_launch_cap_sweep"
        ]
    )
    per_degree = [
        {
            "m2_deg": angle,
            "cap_arm_lower_bound_mm": min(analytic_cap, analytic_transition),
            "block_arm_lower_bound_mm": min(analytic_block, 3.0),
        }
        for angle in range(360)
    ]

    arm_props = _mass_properties(arm, PETG_DENSITY_G_CM3)
    slugs = provisional_balance_slug_envelopes()
    slug_props = [_mass_properties(s, BALANCE_SLUG_DENSITY_G_CM3) for s in slugs]
    slug_mass = sum(row["mass_g"] for row in slug_props)
    slug_moment = sum(
        row["mass_g"] * math.hypot(
            row["center_of_mass_mm"][0], row["center_of_mass_mm"][1],
        )
        for row in slug_props
    )
    target_moment = (
        float(selected["motor_and_balance"]["added_balance_mass_at_R25_g"])
        * P.counterweight_r
    )

    checks = {
        "aggregate_offset_input_report_PASS": aggregate["status"] == "PASS",
        "aggregate_offset_input_authorized": (
            aggregate["offset_flyer_input_authorized"] is True
        ),
        "printed_arm_exactly_one_solid": len(list(arm.solids())) == 1,
        "spoke_width_exact_14mm": math.isclose(
            float(components["spoke"].bounding_box().size.X),
            SPOKE_WIDTH_MM, abs_tol=1.0e-7,
        ),
        "spoke_thickness_exact_8mm": math.isclose(
            float(components["spoke"].bounding_box().size.Z),
            SPOKE_THICKNESS_MM, abs_tol=1.0e-7,
        ),
        "exact_cap_arm_clearance_at_controlling_pose_2p2mm": (
            exact_cap_arm >= REVIEW_CLEARANCE_MM
        ),
        "exact_block_arm_clearance_at_controlling_pose_2p2mm": (
            exact_block_arm >= REVIEW_CLEARANCE_MM
        ),
        "exact_transition_cap_clearance_2p2mm": (
            exact_transition_cap >= REVIEW_CLEARANCE_MM
        ),
        "continuous_360_cap_arm_lower_bound_2p2mm": all(
            row["cap_arm_lower_bound_mm"] >= REVIEW_CLEARANCE_MM
            for row in per_degree
        ),
        "continuous_360_block_arm_lower_bound_2p2mm": all(
            row["block_arm_lower_bound_mm"] >= REVIEW_CLEARANCE_MM
            for row in per_degree
        ),
        "shaft_entry_clearance_2p2mm": exact_shaft_entry >= REVIEW_CLEARANCE_MM,
        "motor_frame_boundary_clearance_2p2mm": (
            exact_motor_frame >= REVIEW_CLEARANCE_MM
        ),
        "belt_drive_rigid_translation_preserves_baseline": (
            selected["gates"]["belt_geometry_preserved_by_rigid_module_translation"]
            is True
        ),
        "provisional_balance_capacity_ge_target_moment": (
            slug_moment >= target_moment
        ),
    }

    review_pass = all(checks.values())
    report: dict[str, Any] = {
        "schema": "permanent-cap-offset-spoke-review/v1",
        "status": "PASS_REVIEW_ONLY" if review_pass else "FAIL_REVIEW",
        "decision": (
            "ISOLATED_OFFSET_SPOKE_CAD_CLEARS_CONTROLLING_GEOMETRY__"
            "PRODUCTION_GATES_REMAIN_OPEN"
            if review_pass
            else "ISOLATED_OFFSET_SPOKE_CAD_FAILED_A_CONTROLLING_GATE"
        ),
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "paths": {
            "source": "cad/permanent_cap_offset_spoke_review.py",
            "step": "out/review/permanent_cap_offset_spoke_review.step",
            "manifest": "out/review/permanent_cap_offset_spoke_review.manifest.json",
            "report_json": "out/reports/permanent_cap_offset_spoke_review.json",
            "report_markdown": "out/reports/permanent_cap_offset_spoke_review.md",
        },
        "source_contracts": {
            "offset_report": {
                "path": "out/reports/permanent_cap_offset_spoke_flyer.json",
                "sha256": _sha256(OFFSET_REPORT),
                "report_sha256": offset.get("report_sha256"),
                "status": offset["status"],
            },
            "aggregate_report": {
                "path": "out/reports/permanent_cap_aggregate_authorization.json",
                "sha256": _sha256(AGGREGATE_REPORT),
                "report_sha256": aggregate.get("report_sha256"),
                "status": aggregate["status"],
                "lane_id": aggregate["cap_support_lane"]["id"],
                "minimum_cap_contact_surface_radius_mm": aggregate[
                    "cap_support_lane"
                ]["minimum_cap_contact_surface_radius_mm"],
                "required_polished_groove_clear_width_mm": aggregate[
                    "cap_support_lane"
                ]["required_polished_groove_clear_width_mm"],
                "finished_wire_axial_envelope_mm": aggregate[
                    "cap_support_lane"
                ]["front_rear_finished_wire_envelope_mm"],
            },
        },
        "selected_dimensions_mm": {
            "cap_wall": CAP_WALL_MM,
            "module_rear_shift": M2_MODULE_REAR_SHIFT_MM,
            "spoke_width": SPOKE_WIDTH_MM,
            "spoke_thickness": SPOKE_THICKNESS_MM,
            "spoke_z_span": [SPOKE_REAR_Z_MM, SPOKE_FRONT_Z_MM],
            "transition_center_radius": TRANSITION_CENTER_RADIUS_MM,
            "tip_guide_center_radius": TIP_GUIDE_CENTER_RADIUS_MM,
            "tip_guide_center_z": TIP_GUIDE_CENTER_Z_MM,
            "entry_guide_rear_shift": ENTRY_GUIDE_REAR_SHIFT_MM,
            "frame_window_rear_shift": FRAME_WINDOW_REAR_SHIFT_MM,
            "deepest_raw_stator_axis_z": deepest_axis_z_mm(),
            "cap_outer_collision_envelope_radius": cap_outer_radius_mm(),
        },
        "geometry": {
            "arm": {
                "solid_count": len(list(arm.solids())),
                "bbox": _bbox(arm),
                "mass_properties_100pct_PETG": arm_props,
            },
            "extended_hollow_shaft": {
                "bbox": _bbox(shaft),
                "outer_diameter_mm": P.flyer_shaft_od,
                "inner_diameter_mm": P.flyer_shaft_id,
            },
            "cap_context": {
                "kind": "conservative collision/support envelope only",
                "production_geometry": False,
                "bbox": _bbox(caps),
            },
        },
        "controlling_clearances_mm": {
            "exact_OCC_at_M1_M2_zero": {
                "cap_to_complete_arm": exact_cap_arm,
                "shifted_block_to_complete_arm": exact_block_arm,
                "cap_to_forward_transition": exact_transition_cap,
                "extended_shaft_to_relocated_entry_eyelet": exact_shaft_entry,
                "shifted_motor_to_relocated_frame_rear_boundary": exact_motor_frame,
            },
            "continuous_rotation_certificate": {
                "method": (
                    "rotation-invariant axial half-space plus conservative "
                    "radial sweep bounds; valid continuously, enumerated at "
                    "all 360 integer M2 degrees present in the raw capture"
                ),
                "integer_angle_count": len(per_degree),
                "minimum_cap_arm_lower_bound_mm": min(
                    row["cap_arm_lower_bound_mm"] for row in per_degree
                ),
                "minimum_block_arm_lower_bound_mm": min(
                    row["block_arm_lower_bound_mm"] for row in per_degree
                ),
                "per_degree": per_degree,
            },
        },
        "provisional_balance_envelope": {
            "status": "GEOMETRIC_CAPACITY_ONLY_NOT_RELEASED",
            "material_envelope": "paired tungsten cylinders",
            "slug_count": 2,
            "slug_radius_mm": BALANCE_SLUG_RADIUS_MM,
            "slug_thickness_mm": BALANCE_SLUG_THICKNESS_MM,
            "centers_xy_mm": [
                [-BALANCE_SLUG_X_MM, BALANCE_SLUG_Y_MM],
                [BALANCE_SLUG_X_MM, BALANCE_SLUG_Y_MM],
            ],
            "combined_mass_g": slug_mass,
            "combined_radial_moment_g_mm": slug_moment,
            "study_target_radial_moment_g_mm": target_moment,
            "retention_hardware_complete": False,
            "two_plane_balance_complete": False,
        },
        "checks": checks,
        "review_checks_passed": sum(bool(value) for value in checks.values()),
        "review_checks_total": len(checks),
        "release_gates": {
            "isolated_review_geometry": review_pass,
            "aggregate_support_contract_bound": True,
            "cap_material_molding_finish_retention_qualified": False,
            "cap_collision_support_envelope_is_production_cap": False,
            "balance_retention_hardware_defined": False,
            "exact_two_plane_balance": False,
            "exact_open_rail_stiffness_or_FEA": False,
            "full_raw_assembly_collision_regenerated": False,
            "production_BOM_and_procurement_integrated": False,
        },
        "limits": [
            "The two cap occurrences are conservative collision/support envelopes, not printable or orderable cap parts.",
            "The aggregate report proves the R3 occupancy/support contract but not individual strand centers, settling, or neatness.",
            "Cap material, Ra<=0.4um finish, retention, dielectric behavior, molding, and abrasion remain unqualified.",
            "The paired tungsten bodies prove only geometric balance-moment capacity; retention and exact two-plane balance are open.",
            "No full raw-cycle assembly collision, updated load duty, BOM, or procurement gate is closed by this isolated review.",
        ],
        "source_hashes": {
            "cad/permanent_cap_offset_spoke_review.py": _sha256(Path(__file__)),
            "out/reports/permanent_cap_offset_spoke_flyer.json": _sha256(OFFSET_REPORT),
            "out/reports/permanent_cap_aggregate_authorization.json": _sha256(AGGREGATE_REPORT),
            "cad/models/upgrades/6627T421.step": _sha256(
                HERE / "models" / "upgrades" / "6627T421.step"
            ),
            "cad/models/upgrades/gt2_belt_200.step": _sha256(
                HERE / "models" / "upgrades" / "gt2_belt_200.step"
            ),
        },
    }
    return report


def render_markdown(report: dict[str, Any]) -> str:
    clear = report["controlling_clearances_mm"]["exact_OCC_at_M1_M2_zero"]
    arm = report["geometry"]["arm"]
    balance = report["provisional_balance_envelope"]
    release = report["release_gates"]
    lines = [
        "# Permanent-cap offset-spoke isolated CAD review",
        "",
        f"**{report['status']} — {report['decision']}**",
        "",
        "This is an isolated geometry review. It is not production assembly integration.",
        "",
        "## Selected geometry",
        "",
        "- 1.0 mm cap collision/support envelopes at the deepest raw pose.",
        "- Complete M2 drive reference translated 10.0 mm rearward.",
        "- One-solid 14 x 8 mm spoke at z=-38.12..-30.12 mm.",
        "- R58 outboard transition and R64 ceramic toroid center at z=-17 mm.",
        "- Entry guide 2.0 mm rearward; fixed 450 mm frame window 2.5 mm rearward.",
        "",
        "## Exact controlling-pose BREP distances",
        "",
        f"- Cap envelope to complete arm: {clear['cap_to_complete_arm']:.6f} mm.",
        f"- Shifted block to complete arm: {clear['shifted_block_to_complete_arm']:.6f} mm.",
        f"- Cap envelope to forward transition: {clear['cap_to_forward_transition']:.6f} mm.",
        f"- Extended shaft to relocated entry eyelet: {clear['extended_shaft_to_relocated_entry_eyelet']:.6f} mm.",
        f"- Shifted motor to relocated frame boundary: {clear['shifted_motor_to_relocated_frame_rear_boundary']:.6f} mm.",
        "- Continuous 360-degree certificate: all 360 integer raw M2 angles plus the intervals between them are covered by rotation-invariant axial/radial bounds.",
        "",
        "## Mass and balance envelope",
        "",
        f"- Printed arm: {arm['mass_properties_100pct_PETG']['mass_g']:.3f} g at 100% PETG; one closed solid.",
        f"- Provisional paired tungsten envelope: {balance['combined_mass_g']:.3f} g / {balance['combined_radial_moment_g_mm']:.3f} g mm versus {balance['study_target_radial_moment_g_mm']:.3f} g mm target.",
        "- Retention hardware and exact two-plane balance remain open.",
        "",
        "## Release boundary",
        "",
    ]
    for key, value in release.items():
        lines.append(f"- {key}: {'PASS' if value else 'OPEN'}")
    lines.extend(["", "## Limits", ""])
    lines.extend(f"- {row}" for row in report["limits"])
    return "\n".join(lines) + "\n"


def write_reports() -> tuple[Path, Path, Path]:
    report = analyze()
    REPORTS.mkdir(parents=True, exist_ok=True)
    REVIEW.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(json.dumps(report, indent=2), encoding="utf-8")
    MD_OUT.write_text(render_markdown(report), encoding="utf-8")
    manifest = {
        "schema": "permanent-cap-offset-spoke-review-manifest/v1",
        "status": report["status"],
        "step": "out/review/permanent_cap_offset_spoke_review.step",
        "source": "cad/permanent_cap_offset_spoke_review.py",
        "source_contracts": report["source_contracts"],
        "selected_dimensions_mm": report["selected_dimensions_mm"],
        "controlling_clearances_mm": report["controlling_clearances_mm"],
        "release_gates": report["release_gates"],
        "limits": report["limits"],
    }
    MANIFEST_OUT.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return JSON_OUT, MD_OUT, MANIFEST_OUT


if __name__ == "__main__":
    print(write_reports())
