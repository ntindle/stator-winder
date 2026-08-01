"""Reproducible verification of the MGN12H upgrade geometry vs HIWIN data.

Run:  ../../../.venv/Scripts/python mgn12_verify.py
Measures the mirrored source STEP, the assembly's dimension-corrected rail, and
the carriage block. The source rail's final-hole defect is reported explicitly;
the production geometry must have six holes at 25 mm pitch with E1/E2=10/15.
"""
import os
import sys
from pathlib import Path
from build123d import import_step, GeomType
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder

UP = os.path.dirname(os.path.abspath(__file__))
MACHINE = Path(__file__).resolve().parents[3]
if str(MACHINE) not in sys.path:
    sys.path.insert(0, str(MACHINE))
CHECKS = []

def vcyls(solid):
    out = []
    for f in solid.faces():
        if f.geom_type != GeomType.CYLINDER:
            continue
        ad = BRepAdaptor_Surface(f.wrapped)
        if ad.GetType() != GeomAbs_Cylinder:
            continue
        c = ad.Cylinder(); ax = c.Axis(); d = ax.Direction(); loc = ax.Location()
        out.append((d.X(), d.Y(), d.Z(), loc.X(), loc.Y(), loc.Z(), c.Radius()))
    return out

def row(name, measured, expected, ok):
    CHECKS.append(bool(ok))
    print(f"  {name:<34} measured={measured:<16} datasheet={expected:<12} {'PASS' if ok else 'FAIL'}")

def rail_hole_centers(solid, radius):
    """Unique Z centers of Y-axis cylindrical faces at ``radius``."""
    return sorted({
        round(lz, 2)
        for dx,dy,dz,lx,ly,lz,r in vcyls(solid)
        if abs(abs(dy)-1.0) < 0.05 and abs(r-radius) < 0.02
    })

# ---------- RAIL ----------
print("="*80)
print("RAIL  mgn12r_rail.step  (MGN12R, 150 mm)")
rail = import_step(f"{UP}/mgn12r_rail.step")
rb = rail.bounding_box()
w = rb.size.X; h = rb.size.Y; L = rb.size.Z
row("rail width (mm)",  f"{w:.3f}", "12.0", abs(w-12.0) <= 0.1)
row("rail height (mm)", f"{h:.3f}", "8.0",  abs(h-8.0) <= 0.1)
row("rail length (mm)", f"{L:.3f}", "150",  abs(L-150) <= 0.5)
rail_bottom = rb.min.Y
print(f"    rail bottom Y = {rail_bottom:.3f}, rail top Y = {rb.max.Y:.3f}")
source_centers = rail_hole_centers(rail, 3.0)
print(f"    mirrored STEP hole centers = {source_centers}")
print("    KNOWN SOURCE DEFECT: final +65 mm center breaks the 25 mm pitch")

# The production wrapper preserves the vendor body but repairs that final bore
# from +65 to +60 according to HIWIN P/E1/E2 dimensions.
from cad.cots import mgn12_rail
corrected = mgn12_rail()
corrected_centers = rail_hole_centers(corrected, 3.0)
corrected_through_centers = rail_hole_centers(corrected, 1.75)
expected_centers = [-65.0, -40.0, -15.0, 10.0, 35.0, 60.0]
pitches = [b-a for a,b in zip(corrected_centers, corrected_centers[1:])]
row("corrected hole count", f"{len(corrected_centers)}", "6", len(corrected_centers)==6)
row("corrected hole centers", str(corrected_centers), str(expected_centers), corrected_centers==expected_centers)
row("corrected through centers", str(corrected_through_centers), str(expected_centers), corrected_through_centers==expected_centers)
row("corrected pitch (mm)", str(pitches), "5 x 25", pitches == [25.0]*5)
row("corrected E1/E2 (mm)",
    f"{corrected_centers[0]+75:.1f}/{75-corrected_centers[-1]:.1f}",
    "10/15",
    corrected_centers[0]+75 == 10.0 and 75-corrected_centers[-1] == 15.0)
if corrected_centers != expected_centers or corrected_through_centers != expected_centers:
    raise SystemExit("production MGN12 rail hole pattern is not dimensionally valid")

# ---------- BLOCK ----------
print("="*80)
print("BLOCK  mgn12h_block.step  (MGN12H carriage w/ end seals)")
blk = import_step(f"{UP}/mgn12h_block.step")
bb = blk.bounding_box()
bw = bb.size.X; bh = bb.size.Y; bL = bb.size.Z
row("block width (mm)",  f"{bw:.3f}", "27.0",  abs(bw-27.0) <= 0.1)
row("block length (mm)", f"{bL:.3f}", "45.4",  abs(bL-45.4) <= 0.5)
row("block body height (mm)", f"{bh:.3f}", "10.0", abs(bh-10.0) <= 0.2)
block_top = bb.max.Y
assembled_h = block_top - rail_bottom
row("assembled height (mm)", f"{assembled_h:.3f}", "13.0", abs(assembled_h-13.0) <= 0.1)
print(f"    (block top Y = {block_top:.3f}) - (rail bottom Y = {rail_bottom:.3f}) = {assembled_h:.3f}")

# mount grid: vertical cylinders on largest-volume solid (the steel body)
body = max(blk.solids(), key=lambda s: s.volume)
vc = [c for c in vcyls(body) if abs(abs(c[1])-1.0) < 0.05]
mounts = {}
for dx,dy,dz,lx,ly,lz,r in vc:
    mounts.setdefault((round(lx,2), round(lz,2)), []).append(round(2*r,2))
centers = sorted(mounts.keys())
xs = sorted(set(x for x,z in centers)); zs = sorted(set(z for x,z in centers))
gx = (max(xs)-min(xs)) if len(xs)>=2 else 0
gz = (max(zs)-min(zs)) if len(zs)>=2 else 0
row("M3 mount hole count", f"{len(centers)}", "4", len(centers)==4)
row("M3 grid X spacing (mm)", f"{gx:.3f}", "20.0", abs(gx-20)<=0.1)
row("M3 grid Z spacing (mm)", f"{gz:.3f}", "20.0", abs(gz-20)<=0.1)
diams = sorted(set(d for v in mounts.values() for d in v))
print(f"    mount hole centers (X,Z) = {centers}")
print(f"    mount hole diameters present = {diams}  (M3 tap Ø2.5 + Ø3.0 nominal)")
print("="*80)
if not all(CHECKS):
    raise SystemExit("one or more MGN12 dimensional checks failed")
