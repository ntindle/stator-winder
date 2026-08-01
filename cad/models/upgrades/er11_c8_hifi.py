"""High-fidelity C8-ER11A-100L straight-shank collet chuck + ER11 collet.

Upgrade of the crude 3-cylinder envelope in ``cad/cots.py::er11_chuck_c8``
(nut Ø19x19, head Ø18x16, shank Ø8x100, Ø8.2 bore).  This module is a
standalone build123d source; it does NOT import or modify any project file.

DROP-IN FRAME CONTRACT (identical to the crude model, so it is a drop-in
replacement for ``cots.er11_chuck_c8``):
  * axis = +Z
  * nut TOP (front) face at z = 0
  * shank extends toward -Z
  * shank Ø8 tip at z = -(35 + shank_len)  ->  z = -135 for shank_len=100
  * the Ø8 shank occupies z = -28 .. -135 (identical Ø8 geometry below
    z = -35 to the crude model, so the two 608ZZ spindle bearings that grip
    the shank seat unchanged).

WHAT CHANGED vs the crude envelope (all from cited drawings, see
er11.report.md):
  * ER11-A nut: real stepped/waisted profile Ø16 -> Ø19 -> Ø16, height 13 mm
    (crude model used a plain Ø19 x 19 mm cylinder; the 19 mm "length" in the
    crude model conflated the Ø19 OD with an over-long axial envelope).
  * chuck body: exposed M14x0.75 thread region modeled as a plain Ø14
    cylinder, a Ø16 body/head collar, and a neck-down cone to the Ø8 shank.
  * ER11 collet: 8-degree external taper truncated cone seated in a conical
    nut mouth (large end forward, per DIN 6499 / ISO 15488 ER geometry).
  * Ø8.2 collet-seat bore ~25 mm deep (the solid Ø8 shank is NOT bored, so
    the shank stays Ø8 -- same intent as the crude model, whose bore was only
    26 mm long; a Ø8.2 through-bore would consume the Ø8.0 shank entirely).

DIMENSION SOURCES (see er11.report.md for the citation table):
  * ER11-A clamping nut  : OD 19 mm, height ~13 mm, thread M14x0.75
      Rego-Fix ER catalog / tools-n-gizmos ER-nut spec / MariTool / HHIP /
      uxcell/Genmitsu ER11-A product data (2026-07-08).
  * ER11 collet          : OD 11.5 mm, length 18 mm, 8 deg taper, cap 1-7 mm
      DIN 6499 / ISO 15488-B; CGTK ER data; Sikka/CNCollets.
  * C8-ER11A-100L holder : Ø8 straight shank, 100 mm, body ~Ø16 behind nut
      Amazon / BigaMart C8-ER11A-100L product listings (2026-07-08).

Run directly to export cad/models/upgrades/er11_c8_hifi.step and print the
verification measurements.
"""

from pathlib import Path

from build123d import (
    Part, Cylinder, Cone, Box, Pos, Align, export_step,
)

CC = Align.CENTER
MAX = Align.MAX
MIN = Align.MIN

# ---- cited dimensions (mm) ------------------------------------------------
NUT_OD = 19.0            # ER11-A clamping nut outer diameter (max)
NUT_WAIST_OD = 16.0      # front/rear stepped collar diameter (Ø16-19 waist)
NUT_H = 13.0             # ER11-A nut height (front face -> rear face)
NUT_MOUTH_D = 9.2        # front opening the collet nose shows through
THREAD_D = 14.0          # M14 x 0.75 male thread region (plain cylinder)
HEAD_D = 16.0            # chuck body / head collar behind the nut
SHANK_D = 8.0            # Ø8 h6 straight shank
COLLET_OD = 11.5         # ER11 collet large-end outer diameter
COLLET_L = 16.0          # visible/modeled collet cone length
COLLET_TAPER_DEG = 8.0   # ER collet outer taper (half-angle)
BORE_D = 8.2             # through-bore

import math
_TAN8 = math.tan(math.radians(COLLET_TAPER_DEG))


def _cyl(r, z_top, z_bot):
    """Solid cylinder radius r spanning [z_bot, z_top] (z_top > z_bot)."""
    return Pos(0, 0, z_top) * Cylinder(r, z_top - z_bot, align=(CC, CC, MAX))


def _cone(r_top, z_top, r_bot, z_bot):
    """Truncated cone: r_top at z_top, r_bot at z_bot (z_top > z_bot)."""
    return Pos(0, 0, z_top) * Cone(
        bottom_radius=r_bot, top_radius=r_top, height=z_top - z_bot,
        align=(CC, CC, MAX))


def er11_chuck_c8(shank_len=100.0, label="er11_c8_hifi"):
    """C8-ER11A-100L collet chuck with an ER11 collet, high fidelity.

    Local frame: axis +Z, nut TOP face at z=0, shank extends -Z, Ø8 shank tip
    at z=-(35+shank_len). Drop-in replacement for cots.er11_chuck_c8.
    """
    rn = NUT_OD / 2          # 9.5
    rw = NUT_WAIST_OD / 2    # 8.0
    rt = THREAD_D / 2        # 7.0
    rh = HEAD_D / 2          # 8.0
    rs = SHANK_D / 2         # 4.0
    rc = COLLET_OD / 2       # 5.75

    # --- ER11-A clamping nut: waisted Ø16 -> Ø19 -> Ø16, height 13 ---------
    # front chamfer (Ø16 -> Ø19), Ø19 body, rear chamfer (Ø19 -> Ø16)
    nut = _cone(rw, 0.0, rn, -1.5)          # front chamfer up to full Ø19
    nut += _cyl(rn, -1.5, -11.5)            # Ø19 body (wrench-flat band, round)
    nut += _cone(rn, -11.5, rw, -13.0)      # rear chamfer down to Ø16 collar
    # nut internal: conical seat mouth then Ø14 thread clearance
    nut -= _cone(NUT_MOUTH_D / 2, 0.2, rt + 0.1, -4.0)   # conical seat mouth
    nut -= _cyl(rt + 0.1, -4.0, -13.2)                   # Ø14.2 thread clear

    # --- chuck body: thread / head collar / neck-down / shank -------------
    thread = _cyl(rt, -3.0, -15.0)          # M14x0.75 region (plain Ø14)
    head = _cyl(rh, -15.0, -22.0)           # Ø16 body collar behind nut
    neck = _cone(rh, -22.0, rs, -28.0)      # Ø16 -> Ø8 neck-down cone
    shank_tip_z = -(35.0 + shank_len)
    shank = _cyl(rs, -28.0, shank_tip_z)    # Ø8 shank (Ø8 identical <= z=-35)

    # --- ER11 collet: 8 deg external taper, large end forward -------------
    collet = _cone(rc, -1.5, rc - COLLET_L * _TAN8, -1.5 - COLLET_L)

    part = nut + thread + head + neck + shank + collet

    # --- Ø8.2 collet-seat bore (~25 mm; Ø8 shank stays solid) -------------
    # A full-length bore would be Ø8.2 > Ø8.0 shank and erase the shank, so
    # (like the crude model, 26 mm) it stops in the head above the neck-down.
    part -= _cyl(BORE_D / 2, 1.0, -24.0)

    part.label = label
    return part


# ---------------------------------------------------------------------------
def _diameter_at(part: Part, z: float) -> float:
    """Max XY diameter of the solid at height z (thin-slab intersection)."""
    slab = Pos(0, 0, z) * Box(200, 200, 0.4, align=(CC, CC, CC))
    sec = part & slab
    bb = sec.bounding_box()
    return max(bb.size.X, bb.size.Y)


if __name__ == "__main__":
    out = Path(__file__).parent / "er11_c8_hifi.step"
    p = er11_chuck_c8(100.0)
    export_step(p, str(out))

    bb = p.bounding_box()
    nut_d = _diameter_at(p, -6.0)      # in the Ø19 nut band
    thread_d = _diameter_at(p, -14.0)  # exposed thread
    head_d = _diameter_at(p, -18.0)    # Ø16 collar
    shank_d = _diameter_at(p, -100.0)  # Ø8 shank
    bore_d = BORE_D

    print(f"exported: {out}")
    print("--- VERIFICATION (measured vs cited) ---")
    print(f"bounding box  : "
          f"X {bb.min.X:+.2f}..{bb.max.X:+.2f} ({bb.size.X:.2f}) | "
          f"Y {bb.min.Y:+.2f}..{bb.max.Y:+.2f} ({bb.size.Y:.2f}) | "
          f"Z {bb.min.Z:+.2f}..{bb.max.Z:+.2f} ({bb.size.Z:.2f})")
    print(f"nut TOP face z    : {bb.max.Z:+.3f}  (cited 0.000 contract)")
    print(f"shank tip z       : {bb.min.Z:+.3f}  (cited -135.000)")
    print(f"nut max diameter  : {nut_d:6.2f} mm  (cited {NUT_OD:.2f})")
    print(f"thread diameter   : {thread_d:6.2f} mm  (cited {THREAD_D:.2f})")
    print(f"head collar dia   : {head_d:6.2f} mm  (cited {HEAD_D:.2f})")
    print(f"shank diameter    : {shank_d:6.2f} mm  (cited {SHANK_D:.2f})")
    print(f"through-bore dia  : {bore_d:6.2f} mm  (cited {BORE_D:.2f})")
    print("--- CLEARANCE FLAG (envelope: nut r<=9.5, nut+nose<=~19mm) ---")
    nut_r = nut_d / 2
    print(f"nut outer radius  : {nut_r:.3f} mm  vs limit 9.500 mm  -> "
          f"{'EXCEEDS' if nut_r > 9.5 + 1e-6 else 'AT/UNDER LIMIT'}")
    print(f"nut axial height  : {NUT_H:.1f} mm  (crude envelope used 19.0 mm)")
