"""Production 0.250 inch MIC6 aluminum carriage plate.

This module is intentionally isolated from the production assembly.  It mirrors
the current :func:`printed.carriage_plate` X/Z interfaces while replacing the
printed, non-planar endstop tab with a separate part from
``carriage_endstop_flag.py``.

Machine-frame convention is retained in the STEP:

* X/Z are the sheet profile axes.
* Y is sheet thickness.
* the bottom face stays on the MGN12H block mounting plane.

The DXF is 1:1 in millimetres with DXF X = machine X and DXF Y = machine Z.
The exact upload is preflighted against the current SendCutSend
``ALUMIC6-250`` catalog/spec feeds by ``sendcutsend_preflight.py``.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import ezdxf
from ezdxf import units
from build123d import Align, Box, Cylinder, Part, Pos, Rot, export_step

from params import PARAMS as P


INCH_MM = 25.4
STOCK_THICKNESS_IN = 0.250
STOCK_THICKNESS_MM = STOCK_THICKNESS_IN * INCH_MM

# Keep the block interface fixed.  Integration should update P.plate_t to
# 6.35 mm, which moves the top-mounted tower upward by only 0.35 mm.
PLATE_BOTTOM_Y = P.block_top_y
PLATE_TOP_Y = PLATE_BOTTOM_Y + STOCK_THICKNESS_MM

PLATE_X_MIN = -88.0
PLATE_X_MAX = 60.0
# Trim 2.5 mm from the unused leading edge. At the deepest captured M0 pose
# this leaves 2.59 mm to the post-L front bracket's extra-low M5 head instead
# of the previous 0.09 mm near-tangent pass; the nearest plate hole remains
# more than 9 mm from this cut edge.
PLATE_Z_MIN = P.m0_home_standoff - 42.5
PLATE_Z_MAX = P.m0_home_standoff + 45.0

# printed.carriage_plate currently clears x=-88..-32 above z=115, but that
# clips the rear-left M4 tower hole at x=-31 by 1.2 mm.  Pulling the notch edge
# to x=-36 keeps the same rail-tail clearance intent (3 mm from the MGN12 rail
# envelope at x=-39) and leaves a 2.75 mm ligament around a standard 4.5 mm
# M4 clearance hole.
NOTCH_Z_MIN = P.m0_home_standoff + 20.0
NOTCH_RIGHT_X = -36.0

MOTOR_WINDOW = (-22.0, 22.0, P.m0_home_standoff - 22.0,
                P.m0_home_standoff + 22.0)
# The complete 22.4 mm anti-backlash set reaches below the old flange-only
# opening and clips the plate edge at y=-187. Open the existing T8 relief to
# the plate's lower edge; keeping x=-82..-60 unchanged preserves the nearby
# MGN12H Ø3.4 cut's 3.3 mm web (above SendCutSend's 2.413 mm rule).
T8_RELIEF = (-82.0, -60.0, PLATE_Z_MIN,
             P.m0_home_standoff - 3.5)

M3_CLEARANCE_D = 3.4
M4_CLEARANCE_D = 4.5


@dataclass(frozen=True)
class RoundCut:
    """One through-hole in machine X/Z coordinates."""

    x: float
    z: float
    diameter: float
    interface: str


def mgn12h_holes() -> tuple[RoundCut, ...]:
    return tuple(
        RoundCut(sx * P.rail_x + dx, P.m0_home_standoff + dz,
                 M3_CLEARANCE_D, "MGN12H")
        for sx in (-1.0, 1.0)
        for dz in (-10.0, 10.0)
        for dx in (-10.0, 10.0)
    )


def tower_holes() -> tuple[RoundCut, ...]:
    return tuple(
        RoundCut(dx, P.m0_home_standoff + dz, M4_CLEARANCE_D,
                 "spindle_tower")
        for dx in (-31.0, 31.0)
        for dz in (-31.0, 31.0)
    )


def nut_bracket_holes() -> tuple[RoundCut, ...]:
    return tuple(
        RoundCut(-78.0, P.m0_home_standoff + dz, M4_CLEARANCE_D,
                 "T8_nut_bracket")
        for dz in (2.0, 12.0)
    )


def mounting_holes() -> tuple[RoundCut, ...]:
    """All inherited round cuts; no extra holes are needed for the flag."""

    return mgn12h_holes() + tower_holes() + nut_bracket_holes()


def outer_contour_xz() -> tuple[tuple[float, float], ...]:
    """Closed cut perimeter, including the edge-open T8 relief notch."""

    t8_x0, t8_x1, t8_z0, t8_z1 = T8_RELIEF
    if abs(t8_z0 - PLATE_Z_MIN) > 1e-9:
        raise ValueError("the T8 relief is not an edge-open perimeter notch")
    return (
        (PLATE_X_MIN, PLATE_Z_MIN),
        (t8_x0, PLATE_Z_MIN),
        (t8_x0, t8_z1),
        (t8_x1, t8_z1),
        (t8_x1, PLATE_Z_MIN),
        (PLATE_X_MAX, PLATE_Z_MIN),
        (PLATE_X_MAX, PLATE_Z_MAX),
        (NOTCH_RIGHT_X, PLATE_Z_MAX),
        (NOTCH_RIGHT_X, NOTCH_Z_MIN),
        (PLATE_X_MIN, NOTCH_Z_MIN),
    )


def _rectangle_contour(rect: tuple[float, float, float, float]):
    x0, x1, z0, z1 = rect
    return ((x0, z0), (x1, z0), (x1, z1), (x0, z1))


def _box(x0: float, x1: float, y0: float, y1: float,
         z0: float, z1: float) -> Part:
    align = (Align.CENTER, Align.CENTER, Align.CENTER)
    return Pos((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0) * Box(
        x1 - x0, y1 - y0, z1 - z0, align=align,
    )


def _cylinder_y(cut: RoundCut) -> Part:
    align = (Align.CENTER, Align.CENTER, Align.CENTER)
    cylinder = Cylinder(cut.diameter / 2.0, STOCK_THICKNESS_MM + 2.0,
                        align=align)
    return Pos(cut.x, (PLATE_BOTTOM_Y + PLATE_TOP_Y) / 2.0, cut.z) * (
        Rot(90.0, 0.0, 0.0) * cylinder
    )


def carriage_plate() -> Part:
    """Return the single-solid, in-place aluminum plate candidate."""

    # Construct the notched outline as two overlapping prisms so the STEP and
    # the DXF share the exact named outline parameters above.
    lower = _box(PLATE_X_MIN, PLATE_X_MAX, PLATE_BOTTOM_Y, PLATE_TOP_Y,
                 PLATE_Z_MIN, NOTCH_Z_MIN)
    rear = _box(NOTCH_RIGHT_X, PLATE_X_MAX, PLATE_BOTTOM_Y, PLATE_TOP_Y,
                NOTCH_Z_MIN, PLATE_Z_MAX)
    plate = lower + rear

    for x0, x1, z0, z1 in (MOTOR_WINDOW, T8_RELIEF):
        plate -= _box(x0, x1, PLATE_BOTTOM_Y - 1.0, PLATE_TOP_Y + 1.0,
                      z0, z1)
    for cut in mounting_holes():
        plate -= _cylinder_y(cut)

    plate.label = "fabricated_carriage_0p250in_mic6"
    return plate


def gen_step() -> Part:
    """CAD-skill entry point."""

    return carriage_plate()


def _add_closed_polyline(modelspace, points: Iterable[tuple[float, float]]):
    return modelspace.add_lwpolyline(
        tuple(points), close=True, dxfattribs={"layer": "CUT"},
    )


def gen_dxf():
    """Return a millimetre, 1:1 cut-profile DXF for the candidate plate."""

    document = ezdxf.new("R2013")
    document.units = units.MM
    document.layers.add("CUT", color=1)
    modelspace = document.modelspace()

    _add_closed_polyline(modelspace, outer_contour_xz())
    _add_closed_polyline(modelspace, _rectangle_contour(MOTOR_WINDOW))
    # T8_RELIEF reaches PLATE_Z_MIN and is already part of the outer cut
    # perimeter.  Emitting it again as a closed internal contour would create
    # coincident toolpaths and a zero-width bridge at SendCutSend.
    for cut in mounting_holes():
        modelspace.add_circle(
            (cut.x, cut.z), cut.diameter / 2.0,
            dxfattribs={"layer": "CUT"},
        )
    return document


if __name__ == "__main__":
    part = carriage_plate()
    output_dir = Path(__file__).resolve().parent
    step_target = output_dir / "fabricated_carriage.step"
    dxf_target = output_dir / "fabricated_carriage.dxf"
    export_step(part, step_target)
    gen_dxf().saveas(dxf_target)
    bbox = part.bounding_box()
    print(
        f"fabricated MIC6 carriage: {len(part.solids())} solid, "
        f"bbox=({bbox.size.X:.3f}, {bbox.size.Y:.3f}, {bbox.size.Z:.3f}) mm, "
        f"holes={len(mounting_holes())}"
    )
    print(step_target)
    print(dxf_target)
