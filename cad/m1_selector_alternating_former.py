"""Isolated review CAD for the M1-selected alternating-former successor.

This file is intentionally not imported by :mod:`assembly`.  It models the
smallest mechanically explicit successor to the rejected one-law M2 cam:

* a 24-sector ternary code collar on the existing M1 coupling;
* a roller reader and three-position docking tongue carried by M0;
* a machine-fixed receiver which is physically disconnected at the raw
  indexing/shaft-wrap pose;
* a four-track signed face cam on M2 and a three-row permutation comb; and
* four spring-return R3 polished end-turn fingers.

The STEP shows pass 0 at the deepest winding pose.  The simulation study owns
the timing, route, load, balance and release decision.  This source never
changes production geometry or the upstream protocol.

Machine frame (millimetres): +Z is the M0/flyer axis, +Y is up/M1 axis, and
+X is horizontal.  M1 rotates about Y and M2 rotates about Z.
"""

from __future__ import annotations

import math

from build123d import (
    Align,
    Box,
    BuildLine,
    BuildSketch,
    Circle,
    Compound,
    Cylinder,
    Plane,
    Polyline,
    Pos,
    Rot,
    Torus,
    Transition,
    sweep,
)

import assembly
from params import DEFAULT_STATOR, PARAMS


CTR = (Align.CENTER, Align.CENTER, Align.CENTER)

# Review pose and controlled mechanism dimensions.
REVIEW_M0_RAD = -61.918
REVIEW_M1_DEG = 0
REVIEW_M2_DEG = 0
REVIEW_AXIS_Z_MM = PARAMS.stator_axis_z(REVIEW_M0_RAD)

LAW_DIRECT = "origin_000_direction_+1"
LAW_REVERSE_ZERO = "origin_000_direction_-1"
LAW_REVERSE_180 = "origin_180_direction_-1"
LAW_CODES = (LAW_DIRECT, LAW_REVERSE_ZERO, LAW_REVERSE_180)

M1_ANGLE_TO_LAW = {
    0: LAW_REVERSE_ZERO,
    15: LAW_REVERSE_180,
    30: LAW_DIRECT,
    45: LAW_REVERSE_180,
    60: LAW_DIRECT,
    75: LAW_DIRECT,
    90: LAW_REVERSE_180,
    105: LAW_DIRECT,
    120: LAW_REVERSE_180,
    135: LAW_REVERSE_180,
    150: LAW_DIRECT,
    165: LAW_REVERSE_180,
    180: LAW_DIRECT,
    195: LAW_DIRECT,
    210: LAW_REVERSE_180,
    225: LAW_DIRECT,
    240: LAW_REVERSE_ZERO,
    255: LAW_REVERSE_180,
    270: LAW_DIRECT,
    285: LAW_REVERSE_180,
    300: LAW_DIRECT,
    315: LAW_DIRECT,
    330: LAW_REVERSE_180,
    345: LAW_DIRECT,
}

CODE_BASE_RADIUS_MM = 13.0
CODE_RADII_MM = {
    LAW_DIRECT: 13.4,
    LAW_REVERSE_ZERO: 14.4,
    LAW_REVERSE_180: 15.4,
}
CODE_COLLAR_WIDTH_MM = 5.0
CODE_COLLAR_CENTER_Y_MM = -153.0
CODE_BORE_RADIUS_MM = 12.05

TONGUE_X_MM = 60.0
TONGUE_Y_BY_LAW_MM = {
    LAW_DIRECT: -38.0,
    LAW_REVERSE_ZERO: -35.0,
    LAW_REVERSE_180: -32.0,
}
TONGUE_RADIUS_MM = 2.5
TONGUE_REAR_OFFSET_MM = 5.0
TONGUE_FRONT_OFFSET_MM = 43.0
RECEIVER_CENTER_Z_MM = -15.0
RECEIVER_LENGTH_Z_MM = 8.0

CAM_INNER_RADIUS_MM = 16.0
CAM_OUTER_RADIUS_MM = 43.0
CAM_THICKNESS_MM = 3.0
CAM_CENTER_Z_MM = -35.5
CAM_TRACK_RADII_MM = (19.0, 26.0, 33.0, 40.0)
CAM_TRACK_TUBE_RADIUS_MM = 0.70
CAM_STROKE_MM = 1.20

GUIDE_SURFACE_RADIUS_MM = 3.0
GUIDE_BLADE_THICKNESS_MM = 0.50
GUIDE_RETRACTION_MM = 4.0


def law_for_m1_angle(angle_deg: float) -> str:
    """Return the ternary code at one exact 15-degree M1 index."""

    snapped = int(round(float(angle_deg))) % 360
    if abs(((float(angle_deg) - snapped + 180.0) % 360.0) - 180.0) > 1e-7:
        raise ValueError("M1 selector angle is not integral degrees")
    if snapped not in M1_ANGLE_TO_LAW:
        raise ValueError("M1 selector requires a 15-degree indexed pose")
    return M1_ANGLE_TO_LAW[snapped]


def gate_state_for_axis_z(axis_z_mm: float) -> str:
    """Positive M0 safety gate state used by CAD and the analytical study."""

    value = float(axis_z_mm)
    if value <= 24.5:
        return "ENGAGED_LOCKED"
    if value < 29.0:
        return "FORCED_RETRACTION_RAMP"
    return "ALL_RETRACTED_DISCONNECTED"


def _cylinder_y(radius: float, length: float):
    return Rot(90.0, 0.0, 0.0) * Cylinder(radius, length, align=CTR)


def _poly_tube(points, radius_mm: float, label: str):
    clean = [tuple(map(float, point)) for point in points]
    direction = tuple(clean[1][axis] - clean[0][axis] for axis in range(3))
    with BuildLine() as path:
        Polyline(*clean)
    with BuildSketch(Plane(origin=clean[0], z_dir=direction)) as profile:
        Circle(float(radius_mm))
    result = sweep(
        profile.sketch, path.line, transition=Transition.TRANSFORMED
    )
    result.label = label
    return result


def selector_code_collar():
    """24 readable sectors with three positive radii and a coupling bore."""

    base = _cylinder_y(CODE_BASE_RADIUS_MM, CODE_COLLAR_WIDTH_MM)
    # The roller reads the centre of each 15-degree lug after M1 settles.
    # Small inter-sector gaps ensure one sector cannot bridge two codes.
    for angle_deg, law in sorted(M1_ANGLE_TO_LAW.items()):
        radius = CODE_RADII_MM[law]
        depth = radius - CODE_BASE_RADIUS_MM + 0.08
        theta = math.radians(angle_deg)
        radial_center = CODE_BASE_RADIUS_MM + depth / 2.0 - 0.04
        tangential = 2.0 * (CODE_BASE_RADIUS_MM + depth) * math.tan(
            math.radians(6.2)
        )
        lug = Box(depth, CODE_COLLAR_WIDTH_MM, tangential, align=CTR)
        lug = Rot(0.0, -angle_deg, 0.0) * lug
        lug = Pos(
            radial_center * math.cos(theta),
            0.0,
            radial_center * math.sin(theta),
        ) * lug
        base += lug
    base -= _cylinder_y(CODE_BORE_RADIUS_MM, CODE_COLLAR_WIDTH_MM + 2.0)
    collar = Pos(0.0, CODE_COLLAR_CENTER_Y_MM, REVIEW_AXIS_Z_MM) * base
    collar.label = "m1_ternary_24_sector_code_collar"
    return collar


def code_reader_roller():
    """MR52ZZ-sized follower envelope at the representative pass-0 code."""

    law = law_for_m1_angle(REVIEW_M1_DEG)
    outer_radius = 2.5
    roller = _cylinder_y(outer_radius, 2.5) - _cylinder_y(1.0, 3.0)
    roller = Pos(
        CODE_RADII_MM[law] + outer_radius,
        CODE_COLLAR_CENTER_Y_MM,
        REVIEW_AXIS_Z_MM,
    ) * roller
    roller.label = "m1_code_reader_MR52ZZ_envelope"
    return roller


def carriage_selector_linkage():
    """Reader lever, vertical transfer link and the selected docking tongue."""

    law = law_for_m1_angle(REVIEW_M1_DEG)
    reader_x = CODE_RADII_MM[law] + 2.5
    lever = Pos((reader_x + 28.0) / 2.0, CODE_COLLAR_CENTER_Y_MM,
                REVIEW_AXIS_Z_MM) * Box(
                    28.0 - reader_x, 4.0, 4.0, align=CTR
                )
    transfer = Pos(28.0, -94.0, REVIEW_AXIS_Z_MM) * Box(
        5.0, 118.0, 5.0, align=CTR
    )
    output = _poly_tube(
        ((28.0, -35.0, REVIEW_AXIS_Z_MM),
         (TONGUE_X_MM, -35.0, REVIEW_AXIS_Z_MM)),
        2.0,
        "m1_selector_output_link",
    )
    linkage = Compound(children=[lever, transfer, output])
    linkage.label = "m1_code_reader_positive_linkage"
    return linkage


def docking_tongue(axis_z_mm: float = REVIEW_AXIS_Z_MM,
                   law: str | None = None):
    selected = law or law_for_m1_angle(REVIEW_M1_DEG)
    if selected not in LAW_CODES:
        raise ValueError("unknown selector law")
    length = TONGUE_FRONT_OFFSET_MM - TONGUE_REAR_OFFSET_MM
    center_z = float(axis_z_mm) - (
        TONGUE_FRONT_OFFSET_MM + TONGUE_REAR_OFFSET_MM
    ) / 2.0
    tongue = Pos(
        TONGUE_X_MM,
        TONGUE_Y_BY_LAW_MM[selected],
        center_z,
    ) * Cylinder(TONGUE_RADIUS_MM, length, align=CTR)
    tongue.label = f"m0_docking_tongue:{selected}"
    return tongue


def selector_receiver():
    """Static three-channel receiver; neutral is spring-return retracted."""

    body = Pos(TONGUE_X_MM, -35.0, RECEIVER_CENTER_Z_MM) * Box(
        14.0, 15.0, RECEIVER_LENGTH_Z_MM, align=CTR
    )
    for y in TONGUE_Y_BY_LAW_MM.values():
        bore = Pos(TONGUE_X_MM, y, RECEIVER_CENTER_Z_MM) * Cylinder(
            TONGUE_RADIUS_MM + 0.20,
            RECEIVER_LENGTH_Z_MM + 2.0,
            align=CTR,
        )
        body -= bore
    body.label = "static_three_channel_fail_closed_receiver"
    return body


def m0_gate_ramp():
    """Envelope of the fixed ramp that lifts the reader before axis_z=29."""

    ramp = Pos(47.0, -35.0, -3.0) * Box(18.0, 16.0, 22.0, align=CTR)
    # The central opening is the selected tongue corridor; the remaining
    # shoulders positively pull the receiver comb to neutral on withdrawal.
    ramp -= Pos(TONGUE_X_MM, -35.0, -3.0) * Box(
        8.0, 9.0, 24.0, align=CTR
    )
    ramp.label = "m0_forced_retraction_gate_ramp"
    return ramp


def signed_face_cam_rotor():
    """Four closed-groove track envelopes on an M2-balanced annular rotor."""

    disc = Cylinder(CAM_OUTER_RADIUS_MM, CAM_THICKNESS_MM, align=CTR)
    disc -= Cylinder(CAM_INNER_RADIUS_MM, CAM_THICKNESS_MM + 2.0, align=CTR)
    disc = Pos(0.0, 0.0, CAM_CENTER_Z_MM) * disc
    disc.label = "m2_four_track_signed_face_cam_rotor"
    children = [disc]
    track_z = CAM_CENTER_Z_MM - CAM_THICKNESS_MM / 2.0
    for index, radius in enumerate(CAM_TRACK_RADII_MM):
        track = Pos(0.0, 0.0, track_z) * Torus(
            radius, CAM_TRACK_TUBE_RADIUS_MM
        )
        track.label = f"closed_cam_groove_centerline_track_{index}"
        children.append(track)
    result = Compound(children=children)
    result.label = "m2_signed_cam_rotor_and_review_tracks"
    return result


def cam_followers_and_selector_comb():
    """Four MR52ZZ-sized rollers and the three-row permutation comb."""

    children = []
    for index, radius in enumerate(CAM_TRACK_RADII_MM):
        roller = _cylinder_y(2.5, 2.5) - _cylinder_y(1.0, 3.0)
        roller = Pos(radius, 0.0, CAM_CENTER_Z_MM - 3.2) * roller
        roller.label = f"MR52ZZ_cam_follower_envelope_{index}"
        children.append(roller)
        rod = Pos(radius, 0.0, -44.0) * Cylinder(1.0, 14.0, align=CTR)
        rod.label = f"signed_cam_follower_rod_{index}"
        children.append(rod)
    comb = Pos(55.0, 0.0, -42.0) * Box(18.0, 26.0, 8.0, align=CTR)
    comb.label = "three_row_positive_permutation_comb"
    children.append(comb)
    return Compound(children=children)


def guide_finger(finger_id: int, deployed: bool):
    """One R3 quarter-cylinder guide shoe and its spring-return blade."""

    if finger_id not in range(4):
        raise ValueError("finger_id must be 0..3")
    top = finger_id in (0, 1)
    x_sign = 1.0 if finger_id in (0, 3) else -1.0
    y_sign = 1.0 if top else -1.0
    center_x = x_sign * 0.85
    center_y = y_sign * (10.5 + (0.0 if deployed else GUIDE_RETRACTION_MM))
    center_z = 2.0

    nose = Rot(0.0, 90.0, 0.0) * Cylinder(
        GUIDE_SURFACE_RADIUS_MM, 1.4, align=CTR
    )
    keep_y = center_y + y_sign * 1.6
    keep = Pos(center_x, keep_y, center_z + 2.0) * Box(
        3.0, 3.2, 4.0, align=CTR
    )
    nose = Pos(center_x, center_y, center_z) * nose
    nose &= keep
    blade_y = center_y + y_sign * 5.0
    blade = Pos(center_x, blade_y, center_z + 1.0) * Box(
        1.4, 10.0, GUIDE_BLADE_THICKNESS_MM, align=CTR
    )
    finger = nose + blade
    names = (
        "axial_positive_tangential_negative",
        "axial_positive_tangential_positive",
        "axial_negative_tangential_positive",
        "axial_negative_tangential_negative",
    )
    finger.label = f"R3_former_{names[finger_id]}:{'deployed' if deployed else 'retracted'}"
    return finger


def former_linkages():
    """Static push-pull cable envelopes; all routing stays outside M2 sweep."""

    children = []
    endpoints = ((0.85, 14.0, 4.0), (-0.85, 14.0, 4.0),
                 (-0.85, -14.0, 4.0), (0.85, -14.0, 4.0))
    for index, endpoint in enumerate(endpoints):
        side = 1.0 if index < 2 else -1.0
        points = (
            (55.0, side * 30.0, -38.0),
            (55.0, side * 30.0, 6.0),
            (12.0, side * 30.0, 6.0),
            endpoint,
        )
        children.append(_poly_tube(
            points, 0.8, f"spring_return_pushpull_link_{index}"
        ))
    return Compound(children=children)


def interlock_switch_envelopes():
    """Two normally-closed plunger switch envelopes for hardwired inhibits."""

    retract = Pos(69.0, -35.0, -7.0) * Box(13.0, 6.0, 23.0, align=CTR)
    retract.label = "NC_all_retracted_M1_enable_interlock"
    seated = Pos(69.0, -25.0, -17.0) * Box(13.0, 6.0, 23.0, align=CTR)
    seated.label = "NC_selector_seated_M2_enable_interlock"
    return Compound(children=[retract, seated])


def _context_parts():
    spindle_location = assembly.link_location(
        "spindle", m0=REVIEW_M0_RAD, m1=math.radians(REVIEW_M1_DEG)
    )
    stator = next(
        part for part in assembly.spindle_link(
            DEFAULT_STATOR, final_wound_collision=False
        ) if part.label == "stator"
    )
    stator = spindle_location * stator
    stator.label = "context_exact_default_stator"
    flyer_arm = next(part for part in assembly.flyer_link()
                     if part.label == "flyer_arm")
    flyer_arm = assembly.link_location(
        "flyer", m2=math.radians(REVIEW_M2_DEG)
    ) * flyer_arm
    flyer_arm.label = "context_current_flyer_arm"
    return stator, flyer_arm


def geometry_contract() -> dict[str, object]:
    law_counts = {
        law: sum(value == law for value in M1_ANGLE_TO_LAW.values())
        for law in LAW_CODES
    }
    tongue = docking_tongue()
    receiver = selector_receiver()
    return {
        "review_m0_rad": REVIEW_M0_RAD,
        "review_axis_z_mm": REVIEW_AXIS_Z_MM,
        "review_m1_deg": REVIEW_M1_DEG,
        "review_law": law_for_m1_angle(REVIEW_M1_DEG),
        "m1_sector_count": len(M1_ANGLE_TO_LAW),
        "law_counts": law_counts,
        "code_radius_range_mm": [min(CODE_RADII_MM.values()),
                                 max(CODE_RADII_MM.values())],
        "maximum_code_collar_front_extent_z_mm": (
            REVIEW_AXIS_Z_MM - max(CODE_RADII_MM.values())
        ),
        "tongue_receiver_overlap_mm": max(
            0.0,
            min(tongue.bounding_box().max.Z, receiver.bounding_box().max.Z)
            - max(tongue.bounding_box().min.Z, receiver.bounding_box().min.Z),
        ),
        "guide_surface_radius_mm": GUIDE_SURFACE_RADIUS_MM,
        "cam_track_count": len(CAM_TRACK_RADII_MM),
        "cam_stroke_mm": CAM_STROKE_MM,
        "gate_at_review": gate_state_for_axis_z(REVIEW_AXIS_Z_MM),
    }


def gen_step() -> Compound:
    """Return a labeled isolated review assembly; never production CAD."""

    context_stator, context_flyer = _context_parts()
    children = [
        context_stator,
        context_flyer,
        selector_code_collar(),
        code_reader_roller(),
        carriage_selector_linkage(),
        docking_tongue(),
        selector_receiver(),
        m0_gate_ramp(),
        signed_face_cam_rotor(),
        cam_followers_and_selector_comb(),
        former_linkages(),
        interlock_switch_envelopes(),
        guide_finger(0, deployed=True),
        guide_finger(1, deployed=False),
        guide_finger(2, deployed=False),
        guide_finger(3, deployed=False),
    ]
    result = Compound(children=children)
    result.label = "m1_selector_alternating_former_fail_closed_review"
    return result


if __name__ == "__main__":
    print(geometry_contract())
