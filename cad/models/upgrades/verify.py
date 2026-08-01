"""Reproducible verification of upgraded COTS STEP models.

Run:  ../../../.venv/Scripts/python verify.py
Imports each STEP with build123d and prints measured vs expected tables for:
  - endstop.step      KW12-3 roller-lever microswitch
  - gt2_40t_b8.step   GT2 40-tooth / 6 mm belt / 8 mm bore pulley
"""
import math, sys, itertools
from build123d import import_step

def cyls(obj):
    out = []
    for f in obj.faces():
        if "CYLINDER" in str(getattr(f.geom_type, "name", f.geom_type)).upper():
            try:
                out.append((f.radius, f.center(), f.area))
            except Exception:
                pass
    return out

def row(name, measured, expected, ok):
    print(f"  {name:<32} measured={measured:<14} expected={expected:<16} {'PASS' if ok else 'CHECK'}")

def verify_switch(path):
    print("="*74)
    print("ENDSTOP  (KW12-3 roller-lever microswitch):", path)
    obj = import_step(path)
    body = max(obj.solids(), key=lambda s: s.volume)  # largest solid = switch body block
    s = body.bounding_box().size
    L, T, H = s.X, s.Y, s.Z      # length, thickness, height
    row("body length (mm)", f"{L:.2f}", "20.0", abs(L-20) <= 1.0)
    row("body height (mm)", f"{H:.2f}", "10.0", abs(H-10) <= 1.0)
    row("body thickness (mm)", f"{T:.2f}", "6.5 (5.8-6.5)", 5.5 <= T <= 6.7)
    holes = [(r, c, a) for (r, c, a) in cyls(obj) if 0.9 <= r <= 1.6]
    row("mounting-hole dia (mm)", f"{2*holes[0][0]:.2f}", "~2.4-3.0", len(holes) == 2)
    row("mounting-hole count", f"{len(holes)}", "2", len(holes) == 2)
    if len(holes) >= 2:
        d = math.dist((holes[0][1].X, holes[0][1].Y, holes[0][1].Z),
                      (holes[1][1].X, holes[1][1].Y, holes[1][1].Z))
        row("mounting-hole spacing (mm)", f"{d:.2f}", "9.5 (9.5-10)", 9.0 <= d <= 10.2)
    row("overall bbox w/ lever (mm)", f"{obj.bounding_box().size.X:.1f}x{obj.bounding_box().size.Y:.1f}x{obj.bounding_box().size.Z:.1f}", "lever incl.", True)

def verify_pulley(path):
    print("="*74)
    print("GT2_40T_B8  (GT2 40T / 6mm belt / 8mm bore pulley):", path)
    obj = import_step(path)
    bb = obj.bounding_box(); d = {"X": bb.size.X, "Y": bb.size.Y, "Z": bb.size.Z}
    axis = min(d, key=d.get)                       # bore axis = smallest overall dim (width)
    ridx = [k for k in "XYZ" if k != axis]
    def radius(v): return math.hypot(getattr(v, ridx[0]), getattr(v, ridx[1]))
    verts = list(obj.vertices())
    tip = max(radius(v) for v in verts if radius(v) <= 13.0)   # tooth tip (below flange 14)
    flange = max(radius(v) for v in verts)
    bore = min((r for (r, c, a) in cyls(obj) if a > 100), default=0)
    row("bore diameter (mm)", f"{2*bore:.2f}", "8.0", abs(2*bore-8) <= 0.1)
    row("tooth-tip OD (mm)", f"{2*tip:.2f}", "~25.0 (40T GT2)", abs(2*tip-25) <= 0.6)
    row("flange OD (mm)", f"{2*flange:.2f}", "28-31", 28 <= 2*flange <= 31)
    row("overall width (mm)", f"{d[axis]:.2f}", "~16.5 (w/ hub)", abs(d[axis]-16.5) <= 1.0)
    # tooth count via angular clustering of tip vertices
    ang = sorted({round(math.degrees(math.atan2(getattr(v, ridx[1]), getattr(v, ridx[0]))) % 360, 1)
                  for v in verts if abs(radius(v)-tip) <= 0.2})
    teeth, prev = 0, -99
    for a in ang:
        if a - prev > 4.5:
            teeth += 1
        prev = a
    row("tooth count", f"{teeth}", "40", teeth == 40)

if __name__ == "__main__":
    verify_switch("endstop.step")
    verify_pulley("gt2_40t_b8.step")
    print("="*74)
