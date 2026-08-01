"""Modeled GT2 belt loop -- a closed-loop belt solid for the M2 flyer drive.

Produces a visual + collision-conservative belt solid: two arcs wrapping the
two (equal, 40T) pulley pitch circles joined by two tangent straight runs.
Tooth detail is intentionally omitted; the solid is a uniform-thickness band.

GT2 belt references / construction (cited):
  - Pitch p = 2.0 mm; belt body thickness = 1.38 mm (published Gates GT2 /
    2GT belt overall thickness). This is the radial band thickness here.
  - The belt PITCH line wraps each pulley at its pitch radius pd/2. For two
    EQUAL pitch circles the belt path is a stadium (racetrack): two half-
    circles of radius pd/2 joined by two straight runs of length = the centre
    distance. Belt PITCH length = 2*CD + pi*pd.
      For pd=25.46, CD=60 (40T:40T): 2*60 + pi*25.46 = 199.985 mm  -> a
      standard 2GT-200 belt (100 teeth). This matches the machine's chosen
      200-2GT belt at the 60 mm centre distance (cad/params.py:m2_motor_axis_y
      = -60, 'belt center distance 60 -> 200-2GT').
  - The 1.38 mm band is laid symmetrically about the pitch line (pitch
    radius +/- 0.69 mm), so the pitch stadium is the true mid-surface of the
    solid; this is a conservative envelope for clash checking.

LOCAL FRAME (as required): the loop is generated in the XY plane and extruded
+Z over 0 .. width. The machine's belt plane is z -93.5 .. -83.5 in MACHINE
coordinates (cad/params.py:pulley_z); to place this local solid there, map
local z 0..width onto the machine belt band, e.g. Pos(0, 0, -93.5) with
width=10 (or the belt/pulley width in use). center_a / center_b are the two
pulley axes expressed in this local XY plane.
"""

import math
from build123d import Part, Circle, Rectangle, Pos, Rot, extrude

# GT2 belt published constant
BELT_THICKNESS = 1.38     # mm, GT2/2GT belt overall body thickness


def gt2_belt_loop(center_a=(0.0, 0.0), center_b=(0.0, -60.0),
                  pd: float = 25.46, width: float = 6.0,
                  thickness: float = BELT_THICKNESS,
                  label: str = "gt2_belt_loop") -> Part:
    """Closed-loop GT2 belt solid between two equal pitch circles.

    center_a, center_b : pulley axis centres in the local XY plane.
    pd                  : pulley pitch diameter (both pulleys, equal).
    width               : axial extrusion depth (local +Z, 0..width).
    thickness           : radial belt body thickness (default 1.38 mm GT2).
    Returns a stadium-annulus solid (two arcs + two tangent straights) whose
    mid-surface is the belt pitch line.  See module docstring for the frame.
    """
    R = pd / 2.0
    ax, ay = center_a
    bx, by = center_b
    L = math.hypot(bx - ax, by - ay)
    ang = math.degrees(math.atan2(by - ay, bx - ax))
    mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
    ro = R + thickness / 2.0
    ri = R - thickness / 2.0

    def _stadium(rho):
        # filled racetrack: disk at each centre + connecting rectangle
        rect = Pos(mx, my, 0.0) * (Rot(0, 0, ang - 90.0) * Rectangle(2 * rho, L))
        return (Pos(ax, ay) * Circle(rho)) + (Pos(bx, by) * Circle(rho)) + rect

    belt2d = _stadium(ro) - _stadium(ri)
    belt = extrude(belt2d, amount=width)
    belt.label = label
    return belt


def gen_step():
    return gt2_belt_loop()


def _measure(center_a=(0.0, 0.0), center_b=(0.0, -60.0), pd=25.46, width=6.0,
             thickness=BELT_THICKNESS):
    R = pd / 2.0
    L = math.hypot(center_b[0] - center_a[0], center_b[1] - center_a[1])
    pitch_len = 2 * L + math.pi * pd
    ro, ri = R + thickness / 2.0, R - thickness / 2.0
    return dict(
        pd=pd, R=R, center_distance=L, thickness=thickness,
        pitch_length=pitch_len,
        teeth_on_belt=pitch_len / 2.0,
        outer_perimeter_calc=2 * L + 2 * math.pi * ro,
        inner_perimeter_calc=2 * L + 2 * math.pi * ri,
    )


if __name__ == "__main__":
    from pathlib import Path
    from build123d import export_step

    out = Path(__file__).parent / "gt2_belt_200.step"
    belt = gt2_belt_loop()
    export_step(belt, str(out))

    m = _measure()
    bb = belt.bounding_box()

    # measured outer/inner perimeters from the extruded solid's top face
    top = belt.faces().filter_by(lambda f: abs(f.center().Z - 6.0) < 1e-6)[0]
    outer_len = top.outer_wire().length
    inner_wires = [w for w in top.wires() if w.length < outer_len - 1e-6]
    inner_len = inner_wires[0].length if inner_wires else float("nan")

    print("=== gt2_belt_loop  40T:40T, CD=60 ===")
    print(f"exported: {out}")
    print(f"pulley pitch diameter pd   : {m['pd']:.4f} mm")
    print(f"centre distance CD         : {m['center_distance']:.4f} mm")
    print(f"belt body thickness        : {m['thickness']:.4f} mm")
    print(f"computed PITCH length       : {m['pitch_length']:.4f} mm "
          f"(2*CD + pi*pd)  (expect ~200 -> 2GT-200)")
    print(f"belt teeth (pitch_len/2)   : {m['teeth_on_belt']:.2f}")
    print(f"OUTER perimeter (measured) : {outer_len:.4f} mm  "
          f"[calc {m['outer_perimeter_calc']:.4f}]")
    print(f"INNER perimeter (measured) : {inner_len:.4f} mm  "
          f"[calc {m['inner_perimeter_calc']:.4f}]")
    print(f"bbox x : {bb.min.X:.3f} .. {bb.max.X:.3f} mm")
    print(f"bbox y : {bb.min.Y:.3f} .. {bb.max.Y:.3f} mm")
    print(f"bbox z : {bb.min.Z:.3f} .. {bb.max.Z:.3f} mm")
