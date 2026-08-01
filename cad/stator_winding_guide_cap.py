"""Isolated review CAD for a stator-attached winding guide cap.

This is deliberately *not* a production assembly part.  It captures the
smallest permanent, stator-rotating two-cap architecture considered by
``sim/stator_winding_guide_cap_study.py``:

* an exact default-stator end-face insulation layer;
* one open radial guide tongue per tooth on each axial end;
* an outboard U-return pad large enough for a 3 mm wire-centre bend; and
* a maximum-launch-wire witness on tooth 0.

The study fails this architecture closed.  Keeping the source isolated makes
the no-go geometry reviewable without adding a known collision body to the
machine assembly.

Stator-local frame (same as ``stator_model.py``): +X is tooth 0 radial,
+Y is tangential, +Z is the stator axis, and the lamination mid-plane is Z=0.
All dimensions are millimetres.
"""

from __future__ import annotations

import math
from pathlib import Path

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
    Transition,
    sweep,
)

from params import DEFAULT_STATOR
import stator_model


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

CTR = (Align.CENTER, Align.CENTER, Align.CENTER)

# Geometry contract shared with the companion study.
MINIMUM_WIRE_CENTER_BEND_RADIUS_MM = 3.0
MAXIMUM_LAUNCH_WIRE_DIAMETER_MM = 0.5
MAXIMUM_LAUNCH_WIRE_RADIUS_MM = MAXIMUM_LAUNCH_WIRE_DIAMETER_MM / 2.0
HORN_CONTACT_SURFACE_RADIUS_MM = (
    MINIMUM_WIRE_CENTER_BEND_RADIUS_MM - MAXIMUM_LAUNCH_WIRE_RADIUS_MM
)

# Solvay's Ryton PPS design guide reports common thin-wall applications at
# 0.38--0.51 mm.  0.50 mm is used as a geometry candidate, not as a released
# molding tolerance or abrasion qualification.
CAP_NOMINAL_WALL_MM = 0.50
CAP_MINIMUM_LIGAMENT_MM = 0.50
CAP_RADIAL_STRAIGHT_ALLOWANCE_MM = 0.50

# The companion study maps the authoritative packing coordinates isometrically
# onto the two outboard crown coordinates.  This radius is intentionally kept
# as a single named design parameter so the collision/no-go boundary is easy
# to review.  The value is recomputed and asserted by the study.
CROWN_BASE_CENTER_RADIUS_MM = 34.00
CROWN_WORKING_RADIUS_MM = MINIMUM_WIRE_CENTER_BEND_RADIUS_MM

CAP_FACE_THICKNESS_MM = CAP_NOMINAL_WALL_MM
CAP_GUIDE_PLANE_OFFSET_MM = MINIMUM_WIRE_CENTER_BEND_RADIUS_MM
CAP_GUIDE_RIB_WIDTH_MM = (
    2.0 * MINIMUM_WIRE_CENTER_BEND_RADIUS_MM + CAP_NOMINAL_WALL_MM
)


def tooth_half_width_mm() -> float:
    return max(2.5, float(DEFAULT_STATOR.od) * 0.07) / 2.0


def s_bend_forward_mm(offset_mm: float,
                      radius_mm: float = MINIMUM_WIRE_CENTER_BEND_RADIUS_MM
                      ) -> float:
    """Forward run of two tangent opposite-radius arcs for a lateral shift."""

    offset = float(offset_mm)
    radius = float(radius_mm)
    if radius <= 0.0 or not 0.0 <= offset <= 4.0 * radius + 1e-12:
        raise ValueError("S-bend offset is outside the two-arc construction")
    if offset <= 1e-12:
        return 0.0
    alpha = math.acos(1.0 - offset / (2.0 * radius))
    return 2.0 * radius * math.sin(alpha)


def _poly_tube(points: np.ndarray, radius_mm: float, label: str) -> Part:
    clean = [tuple(map(float, point)) for point in np.asarray(points)]
    if len(clean) < 2:
        raise ValueError("tube path needs at least two points")
    direction = tuple(clean[1][axis] - clean[0][axis] for axis in range(3))
    with BuildLine() as path:
        Polyline(*clean)
    with BuildSketch(Plane(origin=clean[0], z_dir=direction)) as profile:
        Circle(float(radius_mm))
    result = sweep(
        profile.sketch,
        path.line,
        transition=Transition.TRANSFORMED,
    )
    result.label = label
    return result


def _exact_face_layer(axial_sign: int) -> Part:
    """Thin exact lamination-face coverage, with a shaft clearance bore."""

    if axial_sign not in (-1, 1):
        raise ValueError("axial_sign must be -1 or +1")
    spec = DEFAULT_STATOR
    slab = Box(
        float(spec.od) + 2.0,
        float(spec.od) + 2.0,
        CAP_FACE_THICKNESS_MM,
        align=CTR,
    )
    face = stator_model.stator(spec, label="stator_face_source") & slab
    bore = Cylinder(
        float(spec.shaft_d) / 2.0 + 0.50,
        CAP_FACE_THICKNESS_MM + 2.0,
        align=CTR,
    )
    face -= bore
    z = axial_sign * (
        float(spec.stack) / 2.0 + CAP_FACE_THICKNESS_MM / 2.0
    )
    result = Pos(0.0, 0.0, z) * face
    result.label = f"exact_end_face_insulator_{axial_sign:+d}"
    return result


def _radial_rib(angle_deg: float, axial_sign: int,
                outer_radius_mm: float) -> Part:
    """Minimum open guide tongue; slot mouths remain open between tongues."""

    inner = float(DEFAULT_STATOR.od) * float(DEFAULT_STATOR.hub_od_ratio) / 2.0
    length = float(outer_radius_mm) - inner
    if length <= 0.0:
        raise ValueError("guide tongue outer radius must exceed the hub")
    z = axial_sign * (
        float(DEFAULT_STATOR.stack) / 2.0
        + CAP_GUIDE_PLANE_OFFSET_MM
        + CAP_NOMINAL_WALL_MM / 2.0
    )
    rib = Box(
        length,
        CAP_GUIDE_RIB_WIDTH_MM,
        CAP_NOMINAL_WALL_MM,
        align=(Align.MIN, Align.CENTER, Align.CENTER),
    )
    rib = Pos(inner, 0.0, z) * rib
    return Rot(0.0, 0.0, float(angle_deg)) * rib


def _outer_return_pad(angle_deg: float, axial_sign: int,
                      center_radius_mm: float,
                      radial_span_mm: float,
                      profile_span_mm: float) -> Part:
    """Conservative material under the complete translated R3 U-return set."""

    pad_radius = (
        MINIMUM_WIRE_CENTER_BEND_RADIUS_MM
        + float(profile_span_mm)
        + MAXIMUM_LAUNCH_WIRE_RADIUS_MM
        + CAP_NOMINAL_WALL_MM / 2.0
    )
    length = float(radial_span_mm) + 2.0 * pad_radius
    z = axial_sign * (
        float(DEFAULT_STATOR.stack) / 2.0
        + CAP_GUIDE_PLANE_OFFSET_MM
        + CAP_NOMINAL_WALL_MM / 2.0
    )
    pad = Box(
        length,
        2.0 * pad_radius,
        CAP_NOMINAL_WALL_MM,
        align=CTR,
    )
    pad = Pos(float(center_radius_mm) + float(radial_span_mm) / 2.0,
              0.0, z) * pad
    return Rot(0.0, 0.0, float(angle_deg)) * pad


def guide_cap_parts(axial_sign: int, *, radial_span_mm: float = 5.0115,
                    profile_span_mm: float = 0.6419,
                    ) -> tuple[Part, ...]:
    """Return the closed solids in one minimum-material cap candidate.

    The exact radial span is supplied by the study from the default packing
    graph.  The open gaps between ribs are the slot-mouth windows; this is not
    a closed nozzle and therefore does not consume the lined mouth width.
    """

    if axial_sign not in (-1, 1):
        raise ValueError("axial_sign must be -1 or +1")
    outer = (
        CROWN_BASE_CENTER_RADIUS_MM
        + float(radial_span_mm)
        + MINIMUM_WIRE_CENTER_BEND_RADIUS_MM
        + float(profile_span_mm)
        + MAXIMUM_LAUNCH_WIRE_RADIUS_MM
        + CAP_NOMINAL_WALL_MM / 2.0
    )
    parts: list[Part] = [_exact_face_layer(axial_sign)]
    pitch = 360.0 / int(DEFAULT_STATOR.slots)
    for tooth in range(int(DEFAULT_STATOR.slots)):
        angle = tooth * pitch
        parts.append(_radial_rib(angle, axial_sign, outer))
        parts.append(_outer_return_pad(
            angle,
            axial_sign,
            CROWN_BASE_CENTER_RADIUS_MM,
            float(radial_span_mm),
            float(profile_span_mm),
        ))
    for tooth in range(int(DEFAULT_STATOR.slots)):
        parts[1 + 2 * tooth].label = f"tooth_{tooth:02d}_radial_guide_rib"
        parts[2 + 2 * tooth].label = f"tooth_{tooth:02d}_outboard_return_pad"
    return tuple(parts)


def guide_cap(axial_sign: int, *, radial_span_mm: float = 5.0115,
              profile_span_mm: float = 0.6419,
              label: str | None = None) -> Compound:
    """Return a labeled review compound; no production fusion is claimed."""

    result = Compound(children=list(guide_cap_parts(
        axial_sign,
        radial_span_mm=radial_span_mm,
        profile_span_mm=profile_span_mm,
    )))
    result.label = label or f"permanent_guide_cap_{axial_sign:+d}_no_go"
    return result


def planar_horn_overlap_witness(profile_radius_mm: float,
                                axial_sign: int = 1) -> Compound:
    """Two R3 centreline quarter arcs showing the negative planar bridge.

    The witness is intentionally wire, not cap material.  It makes the
    controlling side-to-side overlap visible in the STEP review packet.
    """

    if axial_sign not in (-1, 1):
        raise ValueError("axial_sign must be -1 or +1")
    radius = MINIMUM_WIRE_CENTER_BEND_RADIUS_MM
    half_span = tooth_half_width_mm() + float(profile_radius_mm)
    radial = 15.0
    z_edge = axial_sign * float(DEFAULT_STATOR.stack) / 2.0
    children = []
    for tangential_sign in (-1, 1):
        start_y = tangential_sign * half_span
        center_y = start_y - tangential_sign * radius
        values = np.linspace(0.0, math.pi / 2.0, 41)
        points = []
        for value in values:
            # At value=0 the path is on the slot-side edge; it moves axially
            # outboard and then inward over the end face.
            y = start_y + tangential_sign * radius * (math.cos(value) - 1.0)
            z = z_edge + axial_sign * radius * math.sin(value)
            points.append((radial, y, z))
        children.append(_poly_tube(
            np.asarray(points),
            MAXIMUM_LAUNCH_WIRE_RADIUS_MM,
            f"required_R3_horn_{tangential_sign:+d}",
        ))
    result = Compound(children=children)
    result.label = "negative_planar_bridge_witness"
    return result


def gen_step() -> Compound:
    """Return the isolated cap/no-go review assembly."""

    stator = stator_model.stator(DEFAULT_STATOR, label="default_stator")
    front = guide_cap(1, label="front_permanent_guide_cap_no_go")
    rear = guide_cap(-1, label="rear_permanent_guide_cap_no_go")
    witness = planar_horn_overlap_witness(0.23876, 1)
    result = Compound(children=[stator, front, rear, witness])
    result.label = "stator_winding_guide_cap_no_go_review"
    return result


if __name__ == "__main__":
    print({
        "minimum_centerline_radius_mm": MINIMUM_WIRE_CENTER_BEND_RADIUS_MM,
        "maximum_launch_wire_diameter_mm": MAXIMUM_LAUNCH_WIRE_DIAMETER_MM,
        "candidate_wall_mm": CAP_NOMINAL_WALL_MM,
        "crown_base_center_radius_mm": CROWN_BASE_CENTER_RADIUS_MM,
    })
