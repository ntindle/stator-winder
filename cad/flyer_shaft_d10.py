"""Released Rev-D shaft geometry for the stock NBK D10 flyer pulley."""

from __future__ import annotations

import math

from build123d import (
    Align,
    Box,
    Cone,
    Cylinder,
    GeomType,
    Part,
    Pos,
    fillet,
)


LENGTH_MM = 79.0
LOCAL_REAR_Z_MM = -LENGTH_MM / 2.0
LOCAL_FRONT_Z_MM = LENGTH_MM / 2.0
WORLD_REAR_Z_MM = -110.75
WORLD_FRONT_Z_MM = WORLD_REAR_Z_MM + LENGTH_MM
WORLD_CENTER_Z_MM = (WORLD_REAR_Z_MM + WORLD_FRONT_Z_MM) / 2.0

NECK_OD_MM = 10.0
NECK_ID_MM = 6.0
NECK_OD_H6_LIMITS_MM = (9.991, 10.000)
NECK_ID_LIMITS_MM = (6.000, 6.030)
NECK_LENGTH_MM = 18.5
MAIN_OD_MM = 12.0
MAIN_ID_MM = 9.0
MAIN_OD_LIMITS_MM = (11.980, 12.000)
MAIN_ID_LIMITS_MM = (9.000, 9.050)
SHOULDER_LOCAL_Z_MM = LOCAL_REAR_Z_MM + NECK_LENGTH_MM
SHOULDER_WORLD_Z_MM = WORLD_REAR_Z_MM + NECK_LENGTH_MM
TRANSITION_LENGTH_MM = 3.0
TRANSITION_END_LOCAL_Z_MM = SHOULDER_LOCAL_Z_MM + TRANSITION_LENGTH_MM
TRANSITION_END_WORLD_Z_MM = SHOULDER_WORLD_Z_MM + TRANSITION_LENGTH_MM
ARM_FLAT_WORLD_Z_MM = -46.0
ARM_FLAT_STATION_FROM_REAR_MM = 64.75
ARM_FLAT_LOCAL_Z_MM = LOCAL_REAR_Z_MM + ARM_FLAT_STATION_FROM_REAR_MM
ARM_FLAT_AXIAL_LENGTH_MM = 5.0
ARM_FLAT_DEPTH_MM = 0.30
MIN_NECK_RADIAL_WALL_MM = (NECK_OD_MM - NECK_ID_MM) / 2.0
MIN_NECK_RADIAL_WALL_AT_LIMITS_MM = (
    NECK_OD_H6_LIMITS_MM[0] - NECK_ID_LIMITS_MM[1]
) / 2.0
MOUTH_FILLET_MM = 0.50
LABEL = "flyer_shaft_d10_id6_to_id9_l79"

CTR = (Align.CENTER, Align.CENTER, Align.CENTER)


def flyer_shaft() -> Part:
    """Build the one-solid L79 hollow shaft in its centered local frame."""

    neck = Pos(0.0, 0.0, LOCAL_REAR_Z_MM) * Cylinder(
        NECK_OD_MM / 2.0,
        NECK_LENGTH_MM,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    main = Pos(0.0, 0.0, SHOULDER_LOCAL_Z_MM) * Cylinder(
        MAIN_OD_MM / 2.0,
        LOCAL_FRONT_Z_MM - SHOULDER_LOCAL_Z_MM,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    outer = neck + main

    neck_bore = Pos(0.0, 0.0, LOCAL_REAR_Z_MM - 1.0) * Cylinder(
        NECK_ID_MM / 2.0,
        NECK_LENGTH_MM + 1.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    transition = Pos(0.0, 0.0, SHOULDER_LOCAL_Z_MM) * Cone(
        NECK_ID_MM / 2.0,
        MAIN_ID_MM / 2.0,
        TRANSITION_LENGTH_MM,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    main_bore = Pos(0.0, 0.0, TRANSITION_END_LOCAL_Z_MM) * Cylinder(
        MAIN_ID_MM / 2.0,
        LOCAL_FRONT_Z_MM - TRANSITION_END_LOCAL_Z_MM + 1.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    part = outer - neck_bore - transition - main_bore

    # The stock split clamp needs a round D10 seat.  Retain only the two
    # orthogonal arm-clamp flats at their unchanged machine station.
    minus_y = Pos(0.0, -8.2, ARM_FLAT_LOCAL_Z_MM) * Box(
        20.0, 5.0, ARM_FLAT_AXIAL_LENGTH_MM, align=CTR
    )
    plus_x = Pos(8.2, 0.0, ARM_FLAT_LOCAL_Z_MM) * Box(
        5.0, 20.0, ARM_FLAT_AXIAL_LENGTH_MM, align=CTR
    )
    part = part - minus_y - plus_x

    expected_mouths = (
        (NECK_ID_MM, LOCAL_REAR_Z_MM),
        (MAIN_ID_MM, LOCAL_FRONT_Z_MM),
    )
    mouth_edges = []
    for diameter, station in expected_mouths:
        matches = [
            edge for edge in part.edges().filter_by(GeomType.CIRCLE)
            if math.isclose(edge.length, math.pi * diameter, abs_tol=1.0e-5)
            and math.isclose(edge.center().Z, station, abs_tol=1.0e-6)
        ]
        if len(matches) != 1:
            raise RuntimeError(
                f"expected one ID{diameter:g} mouth at z={station}, "
                f"found {len(matches)}"
            )
        mouth_edges.extend(matches)
    part = fillet(mouth_edges, MOUTH_FILLET_MM)
    part.label = LABEL
    if len(part.solids()) != 1 or not part.is_valid:
        raise RuntimeError("Rev-D D10 shaft must be one valid solid")
    return part


__all__ = [
    "ARM_FLAT_LOCAL_Z_MM",
    "ARM_FLAT_STATION_FROM_REAR_MM",
    "ARM_FLAT_WORLD_Z_MM",
    "LABEL",
    "LENGTH_MM",
    "LOCAL_FRONT_Z_MM",
    "LOCAL_REAR_Z_MM",
    "MAIN_ID_MM",
    "MAIN_ID_LIMITS_MM",
    "MAIN_OD_MM",
    "MAIN_OD_LIMITS_MM",
    "MIN_NECK_RADIAL_WALL_MM",
    "MIN_NECK_RADIAL_WALL_AT_LIMITS_MM",
    "NECK_ID_MM",
    "NECK_ID_LIMITS_MM",
    "NECK_LENGTH_MM",
    "NECK_OD_MM",
    "NECK_OD_H6_LIMITS_MM",
    "SHOULDER_LOCAL_Z_MM",
    "SHOULDER_WORLD_Z_MM",
    "TRANSITION_END_WORLD_Z_MM",
    "TRANSITION_LENGTH_MM",
    "WORLD_CENTER_Z_MM",
    "WORLD_FRONT_Z_MM",
    "WORLD_REAR_Z_MM",
    "flyer_shaft",
]
