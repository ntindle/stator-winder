# GT2 code-CAD upgrades — geometry references & verified measurements

Two pure code-CAD upgrades for the winding-machine flyer (M2) drive, authored
with build123d 0.11.1 and exported/validated with the project venv
(`.venv/Scripts/python`, Python 3.11.9). Neither existing project file was
modified; these are additive drop-in upgrades.

| file | function | STEP export |
|------|----------|-------------|
| `cad/models/upgrades/gt2_profile.py` | `gt2_pulley(teeth=40, bore_d=12.0, width=10.0, flange_r_rear=14.5, flange_r_front=11.5, hub_len=7.5, hub_r=10.0)` | `gt2_pulley_40t_b12.step` |
| `cad/models/upgrades/gt2_belt.py` | `gt2_belt_loop(center_a=(0,0), center_b=(0,-60), pd=25.46, width=6.0)` | `gt2_belt_200.step` |

Both files also expose `gen_step()` (skill/CI compatible) and a `__main__`
that re-exports the STEP and prints the measurements below.

---

## TASK A — Real GT2 tooth-profile printed flyer pulley

Upgrades `cad/printed.py:flyer_pulley()`, which modeled the 40T GT2 pulley as a
smooth cylinder at the pitch diameter (Ø25.46). This module cuts the **real GT2
2 mm curvilinear tooth** while preserving the printed part's exact mounting
contract (flyer-tube clamp).

### GT2 2 mm geometry references (cited)

- **Pitch** `p = 2.000 mm`; **pitch diameter** `PD = teeth·p/π`.
  40T → `PD = 80/π = 25.4648 mm` (matches the printed part's 25.46).
- **Pitch Line Differential** `PLD = 0.254 mm` — published Gates PowerGrip GT2
  value; the belt pitch line sits 0.254 mm radially **outside** the pulley
  tip circle, so the pulley **outside (tip) diameter**
  `OD = PD − 2·PLD = 24.957 mm` (the standard ~24.9–25.0 mm figure for a 40T
  GT2 pulley).
- **Tooth (groove) depth** `= 0.75 mm` → `root Ø = OD − 2·0.75`.
- **Arc-based tooth** (the task's accepted GT2 arc construction). Each tooth is
  a fully **G1-tangent alternating chain** of two circular-arc types:
  - convex **land-tip arc**, radius `r_tip = 0.555 mm` (published GT2 tip
    radius), centred on the tooth axis at radius `OD/2 − r_tip`;
  - concave **valley arc**, radius `r_val`, centred over the groove at radius
    `root_r + r_val`.
  `r_val` is solved (bisection) from the external-tangency condition
  `|C_tip − C_valley| = r_tip + r_val`, so tip and valley arcs meet tangentially
  at the flank inflection. **For 40T this gives `r_val = 0.4151 mm`.** The full
  face is 40 tip arcs + 40 valley arcs → the characteristic rounded GT2 profile
  (confirmed in the top-view snapshot).
- The related **1.38 mm** GT2 figure is the *belt body thickness* (used in Task
  B), not a pulley-cut dimension.

> Note on the arc construction: this is a documented arc **approximation** of
> the Gates GT2 2 mm profile (not the licensed Gates involute-of-arc point
> table). It is dimensionally correct at the controlling values (pitch, PLD,
> OD, root, depth 0.75, tip radius 0.555) and prints/meshes as a valid GT2
> tooth; use a licensed profile if exact Gates conformance is required.

### Mounting contract preserved (mirrors `printed.py:flyer_pulley`)

- Bore **Ø12.05** (Ø12 nominal + 0.05 diametral clamp clearance) on the axis.
- Radial **M3 clamp hole** (Ø3.4) through the hub wall (−Y).
- **Rear flange r14.5** (Ø29), **front flange r11.5** (Ø23),
  **hub r10.0** (Ø20), hub length 7.5, flanges 1.5 thick each.

### Local frame

Authored **part-local** (unlike `printed.py`, which is authored in machine
coordinates). Axis +Z: toothed body `z 0..width`; rear flange `z −1.5..0`;
front flange `z width..width+1.5`; hub `z width+1.5..width+1.5+hub_len`.
To place at the printed part's machine location (body `z −93.5..−83.5`,
`params.py:pulley_z`) translate by `Pos(0,0,−93.5)`.

### Measured (from the exported solid / tooth wire sampling)

```
pitch diameter PD          : 25.4648 mm
valley arc radius r_val    : 0.4151 mm  (tip arc r = 0.555 mm, depth = 0.75 mm)
OUTER tooth diameter (OD)  : 24.9568 mm   [calc 24.9568]   (expect ~24.9–25.0) ✓
ROOT diameter              : 23.4568 mm   [calc 23.4568]
rear flange diameter       : 29.0000 mm   (r14.5)
front flange diameter      : 23.0000 mm   (r11.5)
hub diameter               : 20.0000 mm   (r10.0)
bore diameter              : 12.0500 mm
part bbox z                : −1.500 .. 19.000 mm
```

`inspect refs --facts`: single valid part, 90 faces / 255 edges, bounds
±14.5 (flanges) in X/Y, z −1.5..19.

---

## TASK B — Modeled GT2 belt loop

Closed-loop belt solid: two arcs wrapping the two equal (40T) pulley pitch
circles + two tangent straight runs. Tooth detail omitted; a uniform 1.38 mm
band — a visual + collision-conservative envelope.

### References / construction

- **Belt body thickness `= 1.38 mm`** — published Gates GT2 / 2GT overall belt
  thickness; here the radial band thickness, laid symmetrically about the pitch
  line (pitch radius ± 0.69 mm) so the pitch stadium is the solid's
  mid-surface.
- Two **equal** pitch circles → the belt path is a **stadium (racetrack)**: two
  half-circles of radius `pd/2` joined by straight runs of length = centre
  distance `CD`.
- **Belt PITCH length** `= 2·CD + π·pd`.
  `pd=25.46, CD=60` → **`199.985 mm` ≈ a standard 2GT-200 belt (100 teeth)**,
  matching the machine's 200-2GT choice at the 60 mm centre distance
  (`params.py:m2_motor_axis_y = −60`, "belt center distance 60 → 200-2GT").

### Local frame (as required)

Loop generated in the **XY plane**, extruded **+Z over 0..width**. The machine's
belt plane is `z −93.5..−83.5` in machine coordinates (`params.py:pulley_z`); to
place, map local `z 0..width` onto that band (e.g. `Pos(0,0,−93.5)` with the
belt/pulley width in use). `center_a`/`center_b` are the pulley axes in this
local XY plane.

### Measured (from the exported solid)

```
pulley pitch diameter pd   : 25.4600 mm
centre distance CD         : 60.0000 mm
belt body thickness        : 1.3800 mm
computed PITCH length      : 199.9849 mm  (2·CD + π·pd)  → 2GT-200 ✓
belt teeth (pitch_len/2)   : 99.99  (→ 100T)
OUTER perimeter (measured) : 204.3203 mm   [calc 204.3203]
INNER perimeter (measured) : 195.6496 mm   [calc 195.6496]
bbox                       : x ±13.42,  y −73.42..13.42,  z 0..6
```

The **pitch length 199.985 mm** is the belt spec length (mid-surface). The
solid's literal outer/inner perimeters (204.32 / 195.65) bracket it by the
±0.69 mm half-thickness. `inspect refs --facts`: single valid part, 10 faces.

---

## Verification performed

- Ran both modules with `.venv/Scripts/python` → STEP exports written to
  `cad/models/upgrades/`.
- Regenerated STEP + CAD Viewer GLB/topology via the CAD skill
  `scripts/step` (`SOURCE.py=OUTPUT.step` pairs).
- `scripts/inspect refs --facts` on both STEPs → `ok: true`, single valid
  solids, bounds as tabulated.
- `scripts/snapshot` iso + top views: pulley shows 40 rounded GT2 teeth, both
  flanges, hub with radial M3 clamp hole; belt shows the closed stadium band.

## Caveats

- The GT2 tooth is a dimensionally-correct **arc approximation**, not the
  licensed Gates point profile (see note above).
- The belt is a smooth band (no teeth) — a collision/visual envelope, not a
  meshing-accurate belt.
- Both parts are **part-local**; the mapping to machine coordinates is
  documented above and in each module docstring. Existing project files are
  unchanged.
```
