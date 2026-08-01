"""Print qualification coupon for the stator-winder PETG release.

This is deliberately isolated from :mod:`assembly`, :mod:`printed`, and the
nineteen production collision links.  It qualifies the actual printer,
filament, and process before production parts are released.

Coordinate system: millimetres, base centered on XY, build plate at Z=0.
The canonical part remains centered; :func:`gen_a1_plate_part` supplies the
explicit non-geometric translation required by headless OrcaSlicer.
"""

from __future__ import annotations

from build123d import Align, Box, Cylinder, Part, Pos


BASE_X = 180.0
BASE_Y = 90.0
BASE_T = 4.0
GAUGE_H = 6.0
CYLINDER_ZMIN = (Align.CENTER, Align.CENTER, Align.MIN)

# Exact production diameters.  Each bearing gauge is a short through-ring so
# the real bearing can be pushed back out without damaging it.
BEARING_GAUGES = (
    {"id": "6001", "diameter_mm": 28.1, "center": (-66.0, -23.0),
     "outer_diameter_mm": 36.0},
    {"id": "608", "diameter_mm": 22.1, "center": (-27.0, -23.0),
     "outer_diameter_mm": 30.0},
    {"id": "688", "diameter_mm": 16.1, "center": (4.0, -23.0),
     "outer_diameter_mm": 24.0},
    {"id": "623", "diameter_mm": 10.1, "center": (28.0, -23.0),
     "outer_diameter_mm": 18.0},
)

PULLEY_BORE_DIAMETER = 12.05
PULLEY_CENTER = (55.0, -23.0)
PULLEY_BOSS_OD = 22.0
PULLEY_GAUGE_H = 12.0

ELBOW_SLEEVE_DIAMETER = 8.96
ELBOW_CENTER = (78.0, -23.0)
ELBOW_PEG_H = 12.0

INSERT_PILOT_DIAMETER = 4.0
INSERT_PILOTS = (
    {"sku": "94459A769", "depth_mm": 3.4, "center": (-70.0, 20.0),
     "marker_count": 1},
    {"sku": "94459A130", "depth_mm": 4.3, "center": (-52.0, 20.0),
     "marker_count": 2},
    {"sku": "94459A140", "depth_mm": 5.7, "center": (-34.0, 20.0),
     "marker_count": 3},
)
INSERT_BOSS_OD = 10.0
INSERT_BOSS_H = 8.0

BRIDGE_GAP = 44.0
BRIDGE_WIDTH = 12.0
BRIDGE_PILLAR_T = 8.0
BRIDGE_UNDERSIDE_Z = 20.0
BRIDGE_ROOF_T = 4.0
BRIDGE_CENTER_X = 30.0
BRIDGE_CENTER_Y = 28.0
A1_PLATE_CENTER_XY = (128.0, 128.0)


def _box(x0: float, x1: float, y0: float, y1: float,
         z0: float, z1: float) -> Part:
    return Pos((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0) * Box(
        x1 - x0, y1 - y0, z1 - z0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )


def coupon_spec() -> dict:
    """Machine-readable acceptance geometry carried into release evidence."""
    return {
        "part": "fit_bridge_coupon",
        "revision": "A",
        "units": "mm",
        "material": "same dry PETG and spool intended for production",
        "base_mm": [BASE_X, BASE_Y, BASE_T],
        "bearing_through_gauges": [dict(row) for row in BEARING_GAUGES],
        "pulley_bore_gauge": {
            "diameter_mm": PULLEY_BORE_DIAMETER,
            "center": list(PULLEY_CENTER),
            "height_mm": PULLEY_GAUGE_H,
        },
        "elbow_sleeve_male_gauge": {
            "diameter_mm": ELBOW_SLEEVE_DIAMETER,
            "center": list(ELBOW_CENTER),
            "height_mm": ELBOW_PEG_H,
        },
        "heat_set_pilots": [dict(row) for row in INSERT_PILOTS],
        "bridge": {
            "unsupported_gap_mm": BRIDGE_GAP,
            "width_mm": BRIDGE_WIDTH,
            "underside_z_mm": BRIDGE_UNDERSIDE_Z,
            "roof_thickness_mm": BRIDGE_ROOF_T,
        },
        "acceptance": {
            "bearing_gauges": (
                "each named real bearing starts square, seats without cracking, "
                "has no radial rock, and can be pushed back out"
            ),
            "pulley_bore": "real OD12 flyer tube enters without splitting",
            "elbow_sleeve": "male OD8.96 gauge enters the real tube ID9 mouth",
            "heat_set_pilots": (
                "one insert of each SKU installs flush at its keyed depth "
                "without boss cracking or pullout"
            ),
            "bridge": (
                "44 mm underside is continuous, has no dropped strands, and "
                "midspan sag is at most 0.50 mm relative to both ends"
            ),
        },
    }


def gen_step() -> Part:
    base = Box(BASE_X, BASE_Y, BASE_T,
               align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = base

    # Bearing through-ring gauges.
    for row in BEARING_GAUGES:
        x, y = row["center"]
        part += Pos(x, y, BASE_T) * Cylinder(
            row["outer_diameter_mm"] / 2.0, GAUGE_H,
            align=CYLINDER_ZMIN)
        part -= Pos(x, y, -1.0) * Cylinder(
            row["diameter_mm"] / 2.0, BASE_T + GAUGE_H + 2.0,
            align=CYLINDER_ZMIN)

    # Female tube-fit gauge for the printed pulley bore.
    part += Pos(*PULLEY_CENTER, BASE_T) * Cylinder(
        PULLEY_BOSS_OD / 2.0, PULLEY_GAUGE_H, align=CYLINDER_ZMIN)
    part -= Pos(*PULLEY_CENTER, -1.0) * Cylinder(
        PULLEY_BORE_DIAMETER / 2.0,
        BASE_T + PULLEY_GAUGE_H + 2.0, align=CYLINDER_ZMIN)

    # Male fit gauge for the wire-elbow sleeve into the real ID9 tube.
    part += Pos(*ELBOW_CENTER, BASE_T) * Cylinder(
        ELBOW_SLEEVE_DIAMETER / 2.0, ELBOW_PEG_H,
        align=CYLINDER_ZMIN)

    # Three exact Ø4.0 blind pilots.  Raised tally keys identify 3.4/4.3/5.7
    # mm depth from left to right without relying on tiny embossed text.
    for row in INSERT_PILOTS:
        x, y = row["center"]
        top = BASE_T + INSERT_BOSS_H
        part += Pos(x, y, BASE_T) * Cylinder(
            INSERT_BOSS_OD / 2.0, INSERT_BOSS_H,
            align=CYLINDER_ZMIN)
        part -= Pos(x, y, top - row["depth_mm"]) * Cylinder(
            INSERT_PILOT_DIAMETER / 2.0, row["depth_mm"] + 1.0,
            align=CYLINDER_ZMIN)
        for index in range(row["marker_count"]):
            marker_x = x - 2.0 + 2.0 * index
            part += _box(marker_x - 0.6, marker_x + 0.6,
                         y + 4.5, y + 7.5, BASE_T, BASE_T + 2.0)

    # Exact unsupported replica of the spindle tower's 44 mm roof span.
    inner_left = BRIDGE_CENTER_X - BRIDGE_GAP / 2.0
    inner_right = BRIDGE_CENTER_X + BRIDGE_GAP / 2.0
    outer_left = inner_left - BRIDGE_PILLAR_T
    outer_right = inner_right + BRIDGE_PILLAR_T
    y0 = BRIDGE_CENTER_Y - BRIDGE_WIDTH / 2.0
    y1 = BRIDGE_CENTER_Y + BRIDGE_WIDTH / 2.0
    part += _box(outer_left, inner_left, y0, y1,
                 BASE_T, BRIDGE_UNDERSIDE_Z)
    part += _box(inner_right, outer_right, y0, y1,
                 BASE_T, BRIDGE_UNDERSIDE_Z)
    part += _box(outer_left, outer_right, y0, y1,
                 BRIDGE_UNDERSIDE_Z,
                 BRIDGE_UNDERSIDE_Z + BRIDGE_ROOF_T)

    part.label = "fit_bridge_coupon"
    return part


def gen_a1_plate_part() -> Part:
    """Return the unchanged coupon translated inside the A1 build volume."""
    x, y = A1_PLATE_CENTER_XY
    part = Pos(x, y, 0.0) * gen_step()
    part.label = "fit_bridge_coupon_a1_plate"
    return part


if __name__ == "__main__":
    coupon = gen_step()
    bb = coupon.bounding_box()
    print(
        f"{coupon.label}: solids={len(coupon.solids())}, "
        f"bbox={bb.size.X:.3f}x{bb.size.Y:.3f}x{bb.size.Z:.3f} mm"
    )
