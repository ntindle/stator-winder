"""Real GT2 (2 mm pitch) tooth-profile pulley for the PRINTED flyer pulley.

Upgrade for cad/printed.py:flyer_pulley(), which models the 40T GT2 pulley as
a smooth cylinder at the pitch diameter (Ø25.46). This module generates the
REAL GT2 curvilinear tooth so the printed part carries an engaging profile,
while preserving the exact mounting contract of the printed part (flyer-tube
clamp): bore Ø12.05, radial M3 clamp hole, rear flange r14.5, front flange
r11.5, and a hub behind the front flange.

GT2 2 mm geometry references / construction (cited):
  - Pitch p = 2.000 mm; pitch diameter PD = teeth * p / pi.
    For 40T: PD = 40*2/pi = 25.4648 mm (matches the printed part's 25.46).
  - Pitch Line Differential PLD = 0.254 mm. This is the published Gates
    PowerGrip GT2 value: the belt pitch line sits 0.254 mm radially OUTSIDE
    the pulley tooth-tip (outside) diameter. Hence the pulley outside
    (tip) diameter OD = PD - 2*PLD = 24.957 mm for 40T (the standard
    ~24.9-25.0 mm figure for a 40T GT2 pulley).
  - Tooth (groove) depth = 0.75 mm  ->  root diameter = OD - 2*0.75.
  - Arc-based tooth approximation (per the task's accepted GT2 arc
    construction): each tooth flank is built from two tangent circular arcs
    per half-tooth:
        * a convex LAND-TIP arc of radius r_tip = 0.555 mm (the published
          GT2 tip radius), centred on the tooth axis at radius (OD/2 - r_tip);
        * a concave VALLEY arc of radius r_val, centred over the groove at
          radius (Rroot + r_val).
    r_val is solved from the external-tangency condition between the tip and
    valley circles (|centre_tip - centre_valley| = r_tip + r_val) so the two
    arcs meet G1-tangent at the flank inflection. For 40T this yields
    r_val = 0.4151 mm. The full tooth face is therefore an alternating,
    fully tangent chain of 0.555 mm tip arcs and r_val valley arcs -- the
    characteristic rounded GT2 profile. (The related 1.38 mm figure is the
    GT2 belt body thickness; it drives the belt model in gt2_belt.py, not the
    pulley cut.)

LOCAL FRAME (this part is authored part-local, unlike cad/printed.py which is
authored in-place in machine coordinates):
  - Axis = +Z. Toothed body occupies z 0 .. width.
  - Rear flange (r14.5) at the -Z end (z -1.5 .. 0) -- belt retainer.
  - Front flange (r11.5) at the +Z end (z width .. width+1.5).
  - Hub (r10) behind the front flange (z width+1.5 .. width+1.5+hub_len).
  - Bore Ø12.05 runs the full length on the Z axis; radial M3 clamp hole
    through the hub wall along -Y.
  To place at the printed part's machine location (body z -93.5..-83.5,
  cad/params.py:pulley_z) translate by Pos(0, 0, -93.5): local z=0 -> -93.5.
"""

import math
from build123d import (
    Part, Cylinder, Pos, Rot, Align, BuildSketch, BuildLine, ThreePointArc,
    make_face, extrude,
)

CTR = (Align.CENTER, Align.CENTER, Align.CENTER)

# ---- GT2 2 mm published constants -------------------------------------------
PITCH = 2.0            # tooth pitch (mm)
PLD = 0.254            # pitch line differential (mm), Gates GT2
TOOTH_DEPTH = 0.75     # pulley groove depth (mm)
R_TIP = 0.555          # GT2 land-tip arc radius (mm)
# ---- printed-part interface constants ---------------------------------------
FLANGE_T = 1.5         # flange thickness (mm), mirrors printed.py:flyer_pulley
BORE_CLEAR = 0.05      # diametral clamp clearance -> Ø12.05 bore on a Ø12 tube
M3_CLEAR_R = 1.7       # M3 clearance-hole radius (Ø3.4)


def _cyl_z(r, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(r, abs(z1 - z0), align=CTR)


def _cyl_y(r, y0, y1, x=0.0, z=0.0):
    c = Cylinder(r, abs(y1 - y0), align=CTR)
    return Pos(x, (y0 + y1) / 2, z) * (Rot(90, 0, 0) * c)


def _polar(r, deg):
    a = math.radians(deg)
    return (r * math.cos(a), r * math.sin(a))


def _rot(pt, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return (pt[0] * c - pt[1] * s, pt[0] * s + pt[1] * c)


def _solve_r_val(Rt, Rr, r_tip, hp_deg):
    """Valley-arc radius from tip/valley external-tangency (bisection)."""
    A = (Rt - r_tip, 0.0)

    def f(v):
        B = _polar(Rr + v, hp_deg)
        return math.hypot(A[0] - B[0], A[1] - B[1]) - (r_tip + v)

    lo, hi = 0.02, 2.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if f(lo) * f(mid) <= 0:
            hi = mid
        else:
            lo = mid
    return 0.5 * (lo + hi)


def gt2_geometry(teeth: int):
    """Return the derived GT2 tooth geometry for `teeth`:
    dict(PD, Rt (tip radius), Rr (root radius), r_val, t_rad, t_ang, hp)."""
    PD = teeth * PITCH / math.pi
    Rt = PD / 2.0 - PLD
    Rr = Rt - TOOTH_DEPTH
    hp = 180.0 / teeth                       # half-pitch angle (deg)
    r_val = _solve_r_val(Rt, Rr, R_TIP, hp)
    A = (Rt - R_TIP, 0.0)
    B = _polar(Rr + r_val, hp)
    d = math.hypot(A[0] - B[0], A[1] - B[1])
    ux, uy = (B[0] - A[0]) / d, (B[1] - A[1]) / d
    T = (A[0] + R_TIP * ux, A[1] + R_TIP * uy)   # tangent point (tooth at 0)
    return dict(PD=PD, Rt=Rt, Rr=Rr, r_val=r_val,
                t_rad=math.hypot(*T), t_ang=math.degrees(math.atan2(T[1], T[0])),
                hp=hp, T=T)


def _tooth_face(teeth: int):
    """Closed 2-D GT2 tooth-profile face in the XY plane (single Face).

    Alternating tangent chain: `teeth` land-tip arcs (r=R_TIP) and `teeth`
    valley arcs (r=r_val). Tooth 0 apex on +X."""
    g = gt2_geometry(teeth)
    Rt, Rr, T, hp = g["Rt"], g["Rr"], g["T"], g["hp"]
    step = 360.0 / teeth
    with BuildSketch() as sk:
        with BuildLine():
            for k in range(teeth):
                th = k * step
                Tl = _rot((T[0], -T[1]), th)
                apex = _polar(Rt, th)
                Tr = _rot((T[0], T[1]), th)
                ThreePointArc(Tl, apex, Tr)                 # land-tip arc
                bottom = _polar(Rr, th + hp)
                Tl_next = _rot((T[0], -T[1]), th + step)
                ThreePointArc(Tr, bottom, Tl_next)          # valley arc
        make_face()
    return sk.sketch


def gt2_pulley(teeth: int = 40, bore_d: float = 12.0, width: float = 10.0,
               flange_r_rear: float = 14.5, flange_r_front: float = 11.5,
               hub_len: float = 7.5, hub_r: float = 10.0,
               label: str = "flyer_pulley_gt2") -> Part:
    """Printed 40T GT2 flyer pulley with a REAL GT2 tooth profile.

    Drop-in for cad/printed.py:flyer_pulley() (which is a plain PD cylinder).
    Mounting contract preserved: bore Ø(bore_d+0.05)=Ø12.05, radial M3 clamp
    hole, rear flange r14.5, front flange r11.5, hub behind the front flange.
    See module docstring for the GT2 construction and local frame.
    """
    body = extrude(_tooth_face(teeth), amount=width)        # z 0..width

    z_rf1, z_rf0 = 0.0, -FLANGE_T                           # rear flange
    z_ff0, z_ff1 = width, width + FLANGE_T                  # front flange
    z_h0, z_h1 = z_ff1, z_ff1 + hub_len                     # hub

    rear = _cyl_z(flange_r_rear, z_rf0, z_rf1)
    front = _cyl_z(flange_r_front, z_ff0, z_ff1)
    hub = _cyl_z(hub_r, z_h0, z_h1)

    p = body + rear + front + hub

    bore_r = bore_d / 2.0 + BORE_CLEAR / 2.0                # Ø12.05
    p -= _cyl_z(bore_r, z_rf0 - 1.0, z_h1 + 1.0)            # through bore
    z_clamp = (z_h0 + z_h1) / 2.0                           # hub mid-plane
    p -= _cyl_y(M3_CLEAR_R, -(hub_r + 1.0), 0.0, z=z_clamp)  # radial M3 clamp
    p.label = label
    return p


def gen_step():
    return gt2_pulley()


def _measure(teeth=40, bore_d=12.0, width=10.0, flange_r_rear=14.5,
             flange_r_front=11.5, hub_len=7.5, hub_r=10.0):
    """Measure OD/root from the actual tooth wire; return a report dict."""
    face = _tooth_face(teeth)
    wire = face.faces()[0].outer_wire()
    rmin, rmax = 1e9, 0.0
    for e in wire.edges():
        for i in range(41):
            q = e.position_at(i / 40.0)
            r = math.hypot(q.X, q.Y)
            rmin, rmax = min(rmin, r), max(rmax, r)
    g = gt2_geometry(teeth)
    return dict(
        teeth=teeth,
        PD=g["PD"], r_val=g["r_val"],
        tip_OD=2 * rmax, root_D=2 * rmin,
        tip_OD_calc=2 * g["Rt"], root_D_calc=2 * g["Rr"],
        flange_rear_D=2 * flange_r_rear, flange_front_D=2 * flange_r_front,
        hub_D=2 * hub_r, bore_D=bore_d + BORE_CLEAR,
    )


if __name__ == "__main__":
    from pathlib import Path
    from build123d import export_step

    out = Path(__file__).parent / "gt2_pulley_40t_b12.step"
    part = gt2_pulley()
    export_step(part, str(out))

    m = _measure()
    bb = part.bounding_box()
    print("=== gt2_pulley  40T GT2 (Ø12 bore) ===")
    print(f"exported: {out}")
    print(f"pitch diameter PD          : {m['PD']:.4f} mm")
    print(f"valley arc radius r_val    : {m['r_val']:.4f} mm "
          f"(tip arc r = {R_TIP} mm, depth = {TOOTH_DEPTH} mm)")
    print(f"OUTER tooth diameter (OD)  : {m['tip_OD']:.4f} mm  "
          f"[calc {m['tip_OD_calc']:.4f}]  (expect ~24.9-25.0)")
    print(f"ROOT diameter              : {m['root_D']:.4f} mm  "
          f"[calc {m['root_D_calc']:.4f}]")
    print(f"rear flange diameter       : {m['flange_rear_D']:.4f} mm (r14.5)")
    print(f"front flange diameter      : {m['flange_front_D']:.4f} mm (r11.5)")
    print(f"hub diameter               : {m['hub_D']:.4f} mm (r10.0)")
    print(f"bore diameter              : {m['bore_D']:.4f} mm")
    print(f"part bbox z                : {bb.min.Z:.3f} .. {bb.max.Z:.3f} mm")
    print(f"part bbox x/y radius       : {bb.max.X:.3f} / {bb.max.Y:.3f} mm")
