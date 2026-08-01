"""Parametric BLDC outrunner stator model (lamination stack + shaft).

Local frame: stator axis = +Z (VERTICAL in machine frame — the assembly
rotates it into place), stack mid-plane at z=0. Tooth 0 centered on +X.
In the machine the presented tooth points toward the flyer (-Z machine);
the assembly maps local +X -> machine -Z at M1 = m1_zero.

Simplified but envelope-faithful: hub ring, N tooth necks, N tip shoes
(arc segments), shaft. Slot openings and lamination detail omitted; tip
shoe arc width is the collision-relevant envelope (set to 0.72 of pitch,
typical for 12N/24N/36N outrunners).
"""

import math
from build123d import (
    Part, Cylinder, Box, Pos, Rot, Align, Polygon, extrude, Plane,
)


def extrude_wedge(sketch2d, height: float):
    """Extrude an XY sketch symmetrically about z=0."""
    return extrude(Plane.XY * sketch2d, amount=height / 2, both=True)

from params import StatorSpec

CTR = (Align.CENTER, Align.CENTER, Align.CENTER)


def stator(spec: StatorSpec, label="stator") -> Part:
    hub_od = spec.od * spec.hub_od_ratio
    hub_id = max(spec.shaft_d + 4.0, hub_od - 10.0)
    neck_w = max(2.5, spec.od * 0.07)          # tooth neck width
    shoe_t = max(1.6, spec.od * 0.045)          # tip shoe radial thickness
    pitch = 2 * math.pi / spec.slots
    shoe_halfang = 0.36 * pitch                 # 0.72 pitch coverage

    hub = Cylinder(hub_od / 2, spec.stack, align=CTR) - \
        Cylinder(hub_id / 2, spec.stack + 2, align=CTR)

    teeth = []
    r_shoe_in = spec.od / 2 - shoe_t
    neck_len = r_shoe_in - hub_od / 2 + 2.0
    for k in range(spec.slots):
        ang_deg = k * 360.0 / spec.slots
        neck = Box(neck_len, neck_w, spec.stack, align=(Align.MIN,) +
                   (Align.CENTER, Align.CENTER))
        neck = Pos(hub_od / 2 - 1.0, 0, 0) * neck
        # tip shoe: exact ring sector = ring ∩ wedge prism about +X
        ring = Cylinder(spec.od / 2, spec.stack, align=CTR) - \
            Cylinder(r_shoe_in, spec.stack + 2, align=CTR)
        big = spec.od * 1.5
        half_open = big * math.tan(shoe_halfang)
        wedge2d = Polygon((0, 0), (big, half_open), (big, -half_open),
                          align=None)
        wedge = extrude_wedge(wedge2d, spec.stack + 2)
        shoe = ring & wedge
        tooth = neck + shoe
        teeth.append(Rot(0, 0, ang_deg) * tooth)

    shaft = Cylinder(
        spec.shaft_d / 2,
        spec.stack + spec.shaft_below + spec.shaft_above,
        align=(Align.CENTER, Align.CENTER, Align.MIN))
    shaft = Pos(0, 0, -spec.stack / 2 - spec.shaft_below) * shaft

    p = hub + teeth + shaft
    p.label = label
    return p


def gen_step():
    return stator(StatorSpec())
