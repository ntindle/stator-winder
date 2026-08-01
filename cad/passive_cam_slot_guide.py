"""Isolated review CAD for an M0-cammed, mouth-only slot guide.

This prototype is deliberately not imported by ``assembly.py``.  It captures
the smallest passive architecture evaluated by
``sim/passive_cam_slot_guide_study.py``:

* a pair of mirror guide fingers in the two slot mouths flanking tooth 0;
* one polished quarter-horn at each axial face of each finger;
* a carriage cam that makes the mouth head follow M0 during winding;
* lost motion after the winding range, so the stator extracts radially before
  the fingers make their small tangential park stroke.

Local frame is the exact stator frame: +X is radial through tooth 0, +Y is
tangential, +Z is the stator axis, and the lamination stack is centred at
Z=0.  The review STEP shows the engaged pose.  The analytical study owns every
motion, clearance, force, and release decision; this file is not production
geometry.

The horn is a quarter of a round torus.  Its convex physical contact radius is
3.000 mm, so the exact default wire centre follows R=3.11176 mm.  The complete
horn remains at or outside X=22.800 mm, 0.200 mm inside the OD mouth and well
outside the packed inner neck.
"""

from __future__ import annotations

import math

from build123d import Align, Box, Compound, Cylinder, Pos, Rot, Torus

from params import DEFAULT_STATOR
import stator_model


CTR = (Align.CENTER, Align.CENTER, Align.CENTER)

# Controlled review dimensions (mm).  These values are duplicated in the
# simulation study and regression-tested there to prevent silent divergence.
MOUTH_EXIT_RADIUS_MM = 22.800
GUIDE_TANGENTIAL_CENTER_MM = 3.0057829486164147
GUIDE_TANGENTIAL_THICKNESS_MM = 0.400
HORN_TUBE_RADIUS_MM = GUIDE_TANGENTIAL_THICKNESS_MM / 2.0
HORN_SURFACE_RADIUS_MM = 3.000
HORN_MAJOR_RADIUS_MM = HORN_SURFACE_RADIUS_MM - HORN_TUBE_RADIUS_MM
DEFAULT_WIRE_RADIUS_MM = float(DEFAULT_STATOR.wire_d) / 2.0
HORN_WIRE_CENTER_RADIUS_MM = (
    HORN_SURFACE_RADIUS_MM + DEFAULT_WIRE_RADIUS_MM
)

FINGER_RADIAL_LENGTH_MM = 5.0
FINGER_AXIAL_THICKNESS_MM = 0.80
FINGER_OUTER_RADIUS_MM = MOUTH_EXIT_RADIUS_MM + FINGER_RADIAL_LENGTH_MM

CAM_BLOCK_RADIAL_MM = 7.0
CAM_BLOCK_TANGENTIAL_MM = 3.0
CAM_BLOCK_AXIAL_MM = 3.0
CAM_FOLLOWER_DIAMETER_MM = 4.0


def _quarter_keep_box(axial_sign: int):
    """Boolean keep volume for the working quadrant of a Y-axis torus."""

    if axial_sign not in (-1, 1):
        raise ValueError("axial_sign must be -1 or +1")
    span = 12.0
    center_z = axial_sign * (
        float(DEFAULT_STATOR.stack) / 2.0 + HORN_WIRE_CENTER_RADIUS_MM
    )
    z_mid = (
        (float(DEFAULT_STATOR.stack) / 2.0 + center_z) / 2.0
        if axial_sign > 0
        else (-float(DEFAULT_STATOR.stack) / 2.0 + center_z) / 2.0
    )
    z_height = abs(center_z - axial_sign * float(DEFAULT_STATOR.stack) / 2.0)
    return Pos(
        MOUTH_EXIT_RADIUS_MM + span / 2.0,
        0.0,
        z_mid,
    ) * Box(
        span,
        4.0,
        z_height + 2.0 * HORN_TUBE_RADIUS_MM + 0.02,
        align=CTR,
    )


def quarter_horn(slot_side: int, axial_sign: int, *, label: str | None = None):
    """One polished R3 convex horn, wholly outside the inner slot neck."""

    if slot_side not in (-1, 1):
        raise ValueError("slot_side must be -1 or +1")
    if axial_sign not in (-1, 1):
        raise ValueError("axial_sign must be -1 or +1")

    center_z = axial_sign * (
        float(DEFAULT_STATOR.stack) / 2.0 + HORN_WIRE_CENTER_RADIUS_MM
    )
    # build123d Torus is centred on Z.  Rotate it so its symmetry axis is Y;
    # its ring then lies in the radial/axial XZ plane.
    ring = Rot(90.0, 0.0, 0.0) * Torus(
        HORN_MAJOR_RADIUS_MM,
        HORN_TUBE_RADIUS_MM,
    )
    ring = Pos(
        MOUTH_EXIT_RADIUS_MM,
        slot_side * GUIDE_TANGENTIAL_CENTER_MM,
        center_z,
    ) * ring
    keep = Pos(0.0, slot_side * GUIDE_TANGENTIAL_CENTER_MM, 0.0) * (
        _quarter_keep_box(axial_sign)
    )
    horn = ring & keep
    horn.label = label or (
        f"mouth_horn_side_{slot_side:+d}_face_{axial_sign:+d}"
    )
    return horn


def guide_finger(slot_side: int, *, label: str | None = None):
    """One mouth-only finger with front/rear quarter horns and cam follower."""

    if slot_side not in (-1, 1):
        raise ValueError("slot_side must be -1 or +1")
    y = slot_side * GUIDE_TANGENTIAL_CENTER_MM
    half_stack = float(DEFAULT_STATOR.stack) / 2.0

    # The thin ribbon begins at the mouth exit and grows only outward.
    ribbon = Pos(
        (MOUTH_EXIT_RADIUS_MM + FINGER_OUTER_RADIUS_MM) / 2.0,
        y,
        0.0,
    ) * Box(
        FINGER_RADIAL_LENGTH_MM,
        GUIDE_TANGENTIAL_THICKNESS_MM,
        float(DEFAULT_STATOR.stack) + 2.0 * DEFAULT_WIRE_RADIUS_MM,
        align=CTR,
    )
    finger = ribbon
    for axial_sign in (-1, 1):
        finger += quarter_horn(slot_side, axial_sign)

    # A compact, deliberately generic carrier and follower visualize the cam
    # interface without pretending to select a bearing or production mount.
    carrier_x = FINGER_OUTER_RADIUS_MM + CAM_BLOCK_RADIAL_MM / 2.0
    carrier = Pos(carrier_x, y, 0.0) * Box(
        CAM_BLOCK_RADIAL_MM,
        CAM_BLOCK_TANGENTIAL_MM,
        CAM_BLOCK_AXIAL_MM,
        align=CTR,
    )
    follower = Pos(
        FINGER_OUTER_RADIUS_MM + CAM_BLOCK_RADIAL_MM,
        y,
        0.0,
    ) * (Rot(90.0, 0.0, 0.0) * Cylinder(
        CAM_FOLLOWER_DIAMETER_MM / 2.0,
        GUIDE_TANGENTIAL_THICKNESS_MM + 1.0,
        align=CTR,
    ))
    finger += carrier + follower
    finger.label = label or f"cammed_mouth_finger_{slot_side:+d}"
    return finger


def guide_parts() -> tuple:
    return (
        guide_finger(+1, label="cammed_mouth_finger_left_slot"),
        guide_finger(-1, label="cammed_mouth_finger_right_slot"),
    )


def horn_contract() -> dict[str, float | bool]:
    """Unit-explicit dimensions consumed by tests and the simulation study."""

    return {
        "mouth_exit_radius_mm": MOUTH_EXIT_RADIUS_MM,
        "guide_tangential_center_mm": GUIDE_TANGENTIAL_CENTER_MM,
        "guide_tangential_thickness_mm": GUIDE_TANGENTIAL_THICKNESS_MM,
        "horn_surface_radius_mm": HORN_SURFACE_RADIUS_MM,
        "default_wire_radius_mm": DEFAULT_WIRE_RADIUS_MM,
        "horn_wire_center_radius_mm": HORN_WIRE_CENTER_RADIUS_MM,
        "minimum_horn_material_radius_mm": MOUTH_EXIT_RADIUS_MM,
        "packed_inner_neck_max_radius_mm": 20.68,
        "mouth_only": MOUTH_EXIT_RADIUS_MM > 20.68,
        "wire_center_R3": HORN_WIRE_CENTER_RADIUS_MM >= 3.0,
    }


def gen_step() -> Compound:
    """Return a labeled review assembly; never the production assembly."""

    stator = stator_model.stator(DEFAULT_STATOR, label="exact_default_stator")
    children = [stator, *guide_parts()]
    result = Compound(children=children)
    result.label = "passive_cam_slot_guide_no_go_review"
    return result


if __name__ == "__main__":
    print(horn_contract())
