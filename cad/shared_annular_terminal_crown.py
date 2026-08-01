"""Shared annular terminal-crown successor for the permanent PEEK caps.

CAD brief
---------
* One natural-unfilled-PEEK replacement part per stator end.  Each part is
  the released production-review cap fused to a continuous annular backbone,
  24 broad capture shoes, and 48 externally accessible transfer channels.
* Stator-local millimetres: +Z is the front/M1 shaft axis and tooth 0 is +X.
* The launch family (OD28/46/65) uses an R40 capture envelope.  Larger jobs
  use ``max(40, OD/2 + 7)``; OD90 therefore requests R52 and remains inside
  the retained flyer R64 tip.
* Each tooth has one 5.60 mm tangential mouth.  Its polished cylindrical
  contact surface is R3.65; even a 0.50 mm wire has R3.40 centreline support.
* The mouth feeds the existing left/right cap ports through two polished open
  C-channels.  Their elbow is analytic R3.60; a 0.90 mm clear channel and
  0.75 mm continuous opening accept 0.20..0.50 mm wire.  Worst minimum-wire
  wander still leaves R3.25, with machining/polishing reserve.
* Positive retention is inherited, without dilution, from the cap pair:
  three M2 through stacks and 24 stator-slot anti-rotation keys per end.

The public add-on and replacement APIs are intentionally separate.  The
add-on is useful for collision-link integration; the replacement is the
manufactured one-solid part.  Exact 2400-locus routing, rigid motion and
coupled M1/M2 loads live in ``sim/shared_annular_terminal_crown_audit.py``.
"""

from __future__ import annotations

from functools import lru_cache
import math
from pathlib import Path
from typing import Iterable

from build123d import (
    Align,
    Box,
    BuildLine,
    BuildSketch,
    Circle,
    Compound,
    Cylinder,
    Line,
    Locations,
    Mode,
    Part,
    Plane,
    Pos,
    RadiusArc,
    Rectangle,
    Rot,
    Sphere,
    sweep,
)

from params import DEFAULT_STATOR
import permanent_cap_production_review as cap


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REVIEW = ROOT / "out" / "review"
STEP_OUT = REVIEW / "shared_annular_terminal_crown.step"

CTR = (Align.CENTER, Align.CENTER, Align.CENTER)
SLOTS = 24
TOOTH_PITCH_DEG = 360.0 / SLOTS

PEEK_DENSITY_G_MM3 = 1.30e-3
WIRE_DIAMETER_MIN_MM = 0.20
WIRE_DIAMETER_JOB_MM = float(DEFAULT_STATOR.wire_d)
WIRE_DIAMETER_MAX_MM = 0.50

# Parametric outer architecture.
MINIMUM_CROWN_RADIUS_MM = 40.0
CROWN_OD_RADIAL_ALLOWANCE_MM = 7.0
CAPTURE_SURFACE_RADIUS_MM = 3.65
CAPTURE_MAX_WIRE_CENTER_RADIUS_MM = (
    CAPTURE_SURFACE_RADIUS_MM - WIRE_DIAMETER_MAX_MM / 2.0
)
CAPTURE_MOUTH_CLEAR_WIDTH_MM = 6.00
CAPTURE_SHOE_TANGENTIAL_WIDTH_MM = 7.20
CAPTURE_SHOE_RADIAL_OUTBOARD_MM = 1.20
CAPTURE_SHOE_AXIAL_OUTBOARD_MM = 0.90

# Transfer geometry.  Endpoints bind exactly to the existing cap ports.
TRANSFER_CENTERLINE_RADIUS_MM = 3.60
TRANSFER_OUTER_RADIUS_MM = 1.35
TRANSFER_CLEAR_RADIUS_MM = 0.45
TRANSFER_OPENING_WIDTH_MM = 0.75
TRANSFER_INNER_X_MM = 18.20
TRANSFER_TANGENTIAL_MM = 2.05
TRANSFER_PORT_Z_MM = 7.73876
TRANSFER_BEND_RADIAL_X_MM = (
    TRANSFER_INNER_X_MM + TRANSFER_CENTERLINE_RADIUS_MM
)
TRANSFER_HIGH_Z_MM = 24.65
TRANSFER_BEND_CENTER_Z_MM = (
    TRANSFER_HIGH_Z_MM - TRANSFER_CENTERLINE_RADIUS_MM
)

# The broad barrel centre is chosen so its outer contact is exactly the
# requested crown radius.  A thin continuous ring joins every tooth shoe and
# both branch pairs; it is not a collection of floating fixed mouths.
BACKBONE_INNER_RADIUS_MM = 30.50
BACKBONE_OUTER_OVERLAP_MM = 0.45
BACKBONE_AXIAL_THICKNESS_MM = 1.60
SELECTION_BOWL_X_MM = 28.0
SELECTION_BOWL_SURFACE_RADIUS_MM = 3.50
SELECTION_BOWL_OUTER_RADIUS_MM = 4.25
SELECTION_RAIL_TANGENTIAL_MM = 3.35
SELECTION_RAIL_WIDTH_MM = 0.50


def crown_radius_for_stator_od(od_mm: float) -> float:
    """Return the supported capture radius for a requested stator OD."""

    od = float(od_mm)
    if not math.isfinite(od) or od <= 0.0:
        raise ValueError("stator OD must be positive and finite")
    return max(MINIMUM_CROWN_RADIUS_MM, od / 2.0 + CROWN_OD_RADIAL_ALLOWANCE_MM)


def _validate_sign(axial_sign: int) -> int:
    if int(axial_sign) not in (-1, 1):
        raise ValueError("axial_sign must be -1 or +1")
    return int(axial_sign)


def _barrel_center_radius(crown_radius_mm: float) -> float:
    return float(crown_radius_mm) - CAPTURE_SURFACE_RADIUS_MM


def branch_centerline(
    side: int,
    axial_sign: int,
    *,
    crown_radius_mm: float | None = None,
    overshoot_mm: float = 0.0,
):
    """Physical port-to-crown branch centre wire for tooth zero.

    ``side=-1`` is the existing left/outgoing endpoint at y=-2.05;
    ``side=+1`` is the right/incoming endpoint at y=+2.05.
    """

    if int(side) not in (-1, 1):
        raise ValueError("side must be -1 or +1")
    sign = _validate_sign(axial_sign)
    radius = crown_radius_for_stator_od(DEFAULT_STATOR.od) if (
        crown_radius_mm is None
    ) else float(crown_radius_mm)
    tangent = int(side) * TRANSFER_TANGENTIAL_MM
    barrel_x = _barrel_center_radius(radius)
    low_z = TRANSFER_PORT_Z_MM - float(overshoot_mm)
    outer_x = SELECTION_BOWL_X_MM + BACKBONE_OUTER_OVERLAP_MM + float(overshoot_mm)
    with BuildLine() as path:
        Line(
            (TRANSFER_INNER_X_MM, tangent, sign * low_z),
            (TRANSFER_INNER_X_MM, tangent, sign * TRANSFER_BEND_CENTER_Z_MM),
        )
        RadiusArc(
            (TRANSFER_INNER_X_MM, tangent, sign * TRANSFER_BEND_CENTER_Z_MM),
            (TRANSFER_BEND_RADIAL_X_MM, tangent, sign * TRANSFER_HIGH_Z_MM),
            TRANSFER_CENTERLINE_RADIUS_MM,
        )
        Line(
            (TRANSFER_BEND_RADIAL_X_MM, tangent, sign * TRANSFER_HIGH_Z_MM),
            (outer_x, tangent, sign * TRANSFER_HIGH_Z_MM),
        )
    return path.wire()


def _branch_sweep(
    radius_mm: float,
    side: int,
    axial_sign: int,
    crown_radius_mm: float,
    *,
    overshoot_mm: float = 0.0,
) -> Part:
    sign = _validate_sign(axial_sign)
    start = (
        TRANSFER_INNER_X_MM,
        int(side) * TRANSFER_TANGENTIAL_MM,
        sign * (TRANSFER_PORT_Z_MM - float(overshoot_mm)),
    )
    profile_plane = Plane(
        origin=start,
        x_dir=(0.0, 1.0, 0.0),
        z_dir=(0.0, 0.0, float(sign)),
    )
    with BuildSketch(profile_plane) as profile:
        Circle(float(radius_mm))
    return sweep(
        profile.sketch,
        branch_centerline(
            side,
            sign,
            crown_radius_mm=crown_radius_mm,
            overshoot_mm=overshoot_mm,
        ),
    )


def _branch_open_channel(
    side: int,
    axial_sign: int,
    crown_radius_mm: float,
) -> Part:
    """Return an externally accessible swept C-channel with positive lips."""

    sign = _validate_sign(axial_sign)
    side_value = int(side)
    if side_value not in (-1, 1):
        raise ValueError("side must be -1 or +1")
    start = (
        TRANSFER_INNER_X_MM,
        side_value * TRANSFER_TANGENTIAL_MM,
        sign * TRANSFER_PORT_Z_MM,
    )
    # +profile-X points away from the tooth centre.  Removing a slot in that
    # direction creates a continuous, directly polishable/gaugeable opening.
    profile_plane = Plane(
        origin=start,
        x_dir=(0.0, float(side_value), 0.0),
        z_dir=(0.0, 0.0, float(sign)),
    )
    with BuildSketch(profile_plane) as profile:
        Circle(TRANSFER_OUTER_RADIUS_MM)
        Circle(TRANSFER_CLEAR_RADIUS_MM, mode=Mode.SUBTRACT)
        with Locations((TRANSFER_OUTER_RADIUS_MM, 0.0)):
            Rectangle(
                2.0 * TRANSFER_OUTER_RADIUS_MM,
                TRANSFER_OPENING_WIDTH_MM,
                mode=Mode.SUBTRACT,
            )
    result = sweep(
        profile.sketch,
        branch_centerline(side_value, sign, crown_radius_mm=crown_radius_mm),
    )
    result.label = (
        f"{'front' if sign > 0 else 'rear'}_"
        f"{'right' if side_value > 0 else 'left'}_open_R3p60_C_channel"
    )
    return result


def _backbone(axial_sign: int, crown_radius_mm: float) -> Part:
    sign = _validate_sign(axial_sign)
    barrel_x = _barrel_center_radius(crown_radius_mm)
    outer = barrel_x + BACKBONE_OUTER_OVERLAP_MM
    inner = min(BACKBONE_INNER_RADIUS_MM, outer - 3.0)
    ring = (
        Cylinder(outer, BACKBONE_AXIAL_THICKNESS_MM, align=CTR)
        - Cylinder(inner, BACKBONE_AXIAL_THICKNESS_MM + 1.0, align=CTR)
    )
    result = Pos(
        0.0, 0.0, sign * TRANSFER_HIGH_Z_MM,
    ) * ring
    result.label = f"{'front' if sign > 0 else 'rear'}_continuous_PEEK_crown_backbone"
    return result


def _tooth_zero_shoe(axial_sign: int, crown_radius_mm: float) -> Part:
    sign = _validate_sign(axial_sign)
    barrel_x = _barrel_center_radius(crown_radius_mm)
    radial_min = barrel_x - CAPTURE_SURFACE_RADIUS_MM
    radial_max = crown_radius_mm + CAPTURE_SHOE_RADIAL_OUTBOARD_MM
    axial_min = (
        TRANSFER_HIGH_Z_MM - 2.0 * CAPTURE_SURFACE_RADIUS_MM
        - CAPTURE_SHOE_AXIAL_OUTBOARD_MM
    )
    axial_max = TRANSFER_HIGH_Z_MM + CAPTURE_SHOE_AXIAL_OUTBOARD_MM
    block = Pos(
        (radial_min + radial_max) / 2.0,
        0.0,
        sign * (axial_min + axial_max) / 2.0,
    ) * Box(
        radial_max - radial_min,
        CAPTURE_SHOE_TANGENTIAL_WIDTH_MM,
        axial_max - axial_min,
        align=CTR,
    )
    # Tangential-axis cylindrical subtraction creates a real polished barrel
    # surface rather than a visual centreline or fan envelope.
    mouth = Pos(
        barrel_x, 0.0, sign * (TRANSFER_HIGH_Z_MM - CAPTURE_SURFACE_RADIUS_MM),
    ) * (
        Rot(90.0, 0.0, 0.0)
        * Cylinder(
            CAPTURE_SURFACE_RADIUS_MM,
            CAPTURE_MOUTH_CLEAR_WIDTH_MM,
            align=CTR,
        )
    )
    # The outer flare is deliberately open.  It contains the complete raw
    # approach cone while retaining the inboard R3.65 polished barrel that
    # actually redirects the tensioned wire.
    flare_min_x = crown_radius_mm - 2.0
    flare = Pos(
        (flare_min_x + radial_max + 0.5) / 2.0,
        0.0,
        sign * (axial_min + axial_max) / 2.0,
    ) * Box(
        radial_max + 0.5 - flare_min_x,
        CAPTURE_MOUTH_CLEAR_WIDTH_MM,
        axial_max - axial_min + 0.5,
        align=CTR,
    )
    result = block.cut(mouth, flare)
    result.label = f"{'front' if sign > 0 else 'rear'}_tooth00_broad_R3p65_capture_shoe"
    return result


def _tooth_zero_selection_bowls(
    axial_sign: int, crown_radius_mm: float,
) -> tuple[list[Part], list[Part]]:
    """Return positive bowl shells/rails and their common open cavities."""

    sign = _validate_sign(axial_sign)
    positives: list[Part] = []
    cuts: list[Part] = []
    for side in (-1, 1):
        tangent = side * TRANSFER_TANGENTIAL_MM
        center = (
            SELECTION_BOWL_X_MM, tangent, sign * TRANSFER_HIGH_Z_MM,
        )
        positives.append(Pos(*center) * Sphere(SELECTION_BOWL_OUTER_RADIUS_MM))
        cuts.append(Pos(*center) * Sphere(SELECTION_BOWL_SURFACE_RADIUS_MM))

    barrel_x = _barrel_center_radius(crown_radius_mm)
    rail_length = barrel_x - SELECTION_BOWL_X_MM + 0.8
    for tangent in (-SELECTION_RAIL_TANGENTIAL_MM, SELECTION_RAIL_TANGENTIAL_MM):
        positives.append(Pos(
            SELECTION_BOWL_X_MM + rail_length / 2.0 - 0.4,
            tangent,
            sign * TRANSFER_HIGH_Z_MM,
        ) * Box(
            rail_length,
            SELECTION_RAIL_WIDTH_MM,
            BACKBONE_AXIAL_THICKNESS_MM,
            align=CTR,
        ))

    # Remove the outward hemispheres to make a passive common selection
    # chamber.  The retained inboard hemispheres provide the R3.50 polished
    # branch-entry surface and join the two physical transfer bores.
    cuts.append(Pos(
        (SELECTION_BOWL_X_MM + barrel_x + 1.0) / 2.0,
        0.0,
        sign * TRANSFER_HIGH_Z_MM,
    ) * Box(
        barrel_x + 1.0 - SELECTION_BOWL_X_MM,
        2.0 * (SELECTION_RAIL_TANGENTIAL_MM - SELECTION_RAIL_WIDTH_MM),
        2.0 * SELECTION_BOWL_OUTER_RADIUS_MM + 1.0,
        align=CTR,
    ))
    return positives, cuts


def capture_shoe(tooth: int, axial_sign: int, *, stator_od_mm: float = 46.0) -> Part:
    if int(tooth) not in range(SLOTS):
        raise ValueError("tooth index outside 0..23")
    radius = crown_radius_for_stator_od(stator_od_mm)
    return Rot(0.0, 0.0, int(tooth) * TOOTH_PITCH_DEG) * (
        _tooth_zero_shoe(axial_sign, radius)
    )


@lru_cache(maxsize=8)
def crown_add_on(axial_sign: int, stator_od_mm: float = 46.0) -> Part:
    """Return the one-solid shared crown extension before cap fusion."""

    sign = _validate_sign(axial_sign)
    radius = crown_radius_for_stator_od(stator_od_mm)
    positives: list[Part] = [_backbone(sign, radius)]
    positives.extend(
        capture_shoe(tooth, sign, stator_od_mm=stator_od_mm)
        for tooth in range(SLOTS)
    )
    for tooth in range(SLOTS):
        rotation = Rot(0.0, 0.0, tooth * TOOTH_PITCH_DEG)
        bowl_positive, _bowl_cuts = _tooth_zero_selection_bowls(sign, radius)
        positives.extend(rotation * part for part in bowl_positive)
        for side in (-1, 1):
            positives.append(
                rotation * _branch_open_channel(side, sign, radius)
            )
    result = positives[0].fuse(*positives[1:])

    chamber_tools: list[Part] = []
    for tooth in range(SLOTS):
        rotation = Rot(0.0, 0.0, tooth * TOOTH_PITCH_DEG)
        _bowl_positive, bowl_cuts = _tooth_zero_selection_bowls(sign, radius)
        chamber_tools.extend(rotation * part for part in bowl_cuts)
    result = result.cut(*chamber_tools)
    solids = list(result.solids())
    if len(solids) != 1:
        raise RuntimeError(
            f"shared crown add-on must be one solid; observed {len(solids)}"
        )
    result.label = (
        f"{'front' if sign > 0 else 'rear'}_one_piece_shared_annular_PEEK_crown_add_on"
    )
    return result


@lru_cache(maxsize=8)
def crowned_cap_replacement(axial_sign: int, stator_od_mm: float = 46.0) -> Part:
    """Return the positively retained, manufactured one-solid replacement."""

    sign = _validate_sign(axial_sign)
    if not math.isclose(float(stator_od_mm), float(DEFAULT_STATOR.od), abs_tol=1e-12):
        raise ValueError(
            "physical replacement currently binds the supplied OD46 cap; "
            "other ODs are parametric route/envelope cases until their cap footprint is supplied"
        )
    result = cap.cap_part(sign).fuse(crown_add_on(sign, stator_od_mm))
    solids = list(result.solids())
    if len(solids) != 1:
        raise RuntimeError(
            f"crowned cap replacement must be one solid; observed {len(solids)}"
        )
    result.label = (
        f"{'front' if sign > 0 else 'rear'}_positive_retained_one_solid_PEEK_crowned_cap"
    )
    return result


def crown_pair(stator_od_mm: float = 46.0) -> Compound:
    result = Compound(children=[
        crowned_cap_replacement(1, stator_od_mm),
        crowned_cap_replacement(-1, stator_od_mm),
    ])
    result.label = "front_rear_shared_annular_PEEK_crowned_cap_pair"
    return result


def retention_hardware() -> tuple[Part, ...]:
    """Public replacement API: unchanged positive cap retention stack."""

    return cap.retention_hardware()


def spindle_link_parts() -> list[Part]:
    """Collision/export API in stator local coordinates."""

    return [
        crowned_cap_replacement(1),
        crowned_cap_replacement(-1),
        *retention_hardware(),
    ]


def peek_mass_g(part: Part) -> float:
    return float(part.volume) * PEEK_DENSITY_G_MM3


def gen_step() -> Compound:
    stator = __import__("stator_model").stator(
        DEFAULT_STATOR, label="default_OD46_stator_context"
    )
    pair = crown_pair()
    hardware_group = Compound(children=list(retention_hardware()))
    hardware_group.label = "unchanged_three_stack_positive_cap_retention"
    result = Compound(children=[stator, pair, hardware_group])
    result.label = "shared_annular_terminal_crown_positive_retained_review"
    return result


if __name__ == "__main__":
    front = crowned_cap_replacement(1)
    rear = crowned_cap_replacement(-1)
    print(
        "shared crown",
        len(list(front.solids())), len(list(rear.solids())),
        f"mass={peek_mass_g(front) + peek_mass_g(rear):.3f}g",
    )
