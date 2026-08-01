"""Separate printable home-endstop flag for ``fabricated_carriage``.

The flag reuses the two rear spindle-tower M4 holes.  No new holes are added
to the aluminum plate.  It restores the current integral trigger envelope at
the center rear edge while remaining a separately printable, replaceable part.
"""

from __future__ import annotations

from build123d import Align, Box, Cylinder, Part, Pos, Rot

from params import PARAMS as P
from fabricated_carriage import M4_CLEARANCE_D, PLATE_BOTTOM_Y


FLAG_THICKNESS = 6.0
FLAG_TOP_Y = PLATE_BOTTOM_Y
FLAG_BOTTOM_Y = FLAG_TOP_Y - FLAG_THICKNESS

PAD_X = (-36.0, 36.0)
PAD_Z = (P.m0_home_standoff + 26.0, P.m0_home_standoff + 39.0)
TRIGGER_X = (-8.0, 8.0)
TRIGGER_Z = (P.m0_home_standoff + 39.0,
             P.m0_home_standoff + 47.0)

# Shared with the rear (+Z) row of the existing 62 x 62 tower pattern.
ATTACHMENT_HOLES_XZ = (
    (-31.0, P.m0_home_standoff + 31.0),
    (31.0, P.m0_home_standoff + 31.0),
)

FASTENER_RECOMMENDATION = {
    "quantity": 2,
    "screw": "ISO 4762 M4x25 socket-head cap screw",
    "washer": "ISO 7089 M4 plain washer",
    "nut": "ISO 10511 M4 prevailing-torque nut",
    "assembly_delta": (
        "replace only the two rear tower M4x20 screws with M4x25; retain "
        "the existing washer and nyloc stack"
    ),
}


def _box(x0: float, x1: float, y0: float, y1: float,
         z0: float, z1: float) -> Part:
    align = (Align.CENTER, Align.CENTER, Align.CENTER)
    return Pos((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0) * Box(
        x1 - x0, y1 - y0, z1 - z0, align=align,
    )


def _hole_y(x: float, z: float) -> Part:
    align = (Align.CENTER, Align.CENTER, Align.CENTER)
    cutter = Cylinder(M4_CLEARANCE_D / 2.0, FLAG_THICKNESS + 2.0,
                      align=align)
    return Pos(x, (FLAG_BOTTOM_Y + FLAG_TOP_Y) / 2.0, z) * (
        Rot(90.0, 0.0, 0.0) * cutter
    )


def endstop_flag() -> Part:
    """Return the in-place, single-solid printable flag concept."""

    pad = _box(PAD_X[0], PAD_X[1], FLAG_BOTTOM_Y, FLAG_TOP_Y,
               PAD_Z[0], PAD_Z[1])
    trigger = _box(TRIGGER_X[0], TRIGGER_X[1], FLAG_BOTTOM_Y, FLAG_TOP_Y,
                   TRIGGER_Z[0], TRIGGER_Z[1])
    flag = pad + trigger
    for x, z in ATTACHMENT_HOLES_XZ:
        flag -= _hole_y(x, z)
    flag.label = "printable_carriage_endstop_flag"
    return flag


def gen_step() -> Part:
    """CAD-skill entry point."""

    return endstop_flag()


if __name__ == "__main__":
    part = endstop_flag()
    bbox = part.bounding_box()
    print(
        f"endstop flag: {len(part.solids())} solid, "
        f"bbox=({bbox.size.X:.3f}, {bbox.size.Y:.3f}, {bbox.size.Z:.3f}) mm"
    )
