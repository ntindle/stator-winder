# Carriage hardware audit — Goal 1

Date: 2026-07-10  
Scope: MIC6 carriage plate, MGN12H block fasteners, spindle tower, M1 motor,
T8 nut/bracket, rear endstop flag, and the M0 fixed-end support.

## Result

The isolated corrected candidates pass all eight deterministic tests.  Four
real positive-volume faults were reproduced and removed.  The M1 motor stack
was also checked explicitly and is already correct: the current two-millimetre
underside pocket leaves a four-millimetre printed roof under all four heads.

| source-level Boolean check | measured pre-fix | corrected candidate |
|---|---:|---:|
| four inner MGN12H M3 heads vs spindle tower | 275.158 mm³ | 0.000 mm³ |
| rear flag M4 washers/nylocs vs printable flag | 294.614 mm³ | 0.000 mm³ |
| lower two T8 washer/nyloc stacks vs bracket foot | 79.108 mm³ | 0.000 mm³ |
| printable flag vs M0 fixed-end mount | 12.000 mm³ | 0.000 mm³ |
| M1 head-bearing material, four 0.10 mm annular probes | 6.504 mm³ | 6.504 mm³ |

Run:

```powershell
.\.venv\Scripts\python.exe cad\test_carriage_hardware_audit.py
```

Observed: `Ran 8 tests ... OK`.

## Exact shared patch table

1. `cad/printed.py:spindle_tower`

   Subtract four axis-Y cylinders, Ø6.4 and 3.25 mm deep from the MIC6 mating
   face, at machine X/Z `(-35,85)`, `(-35,105)`, `(35,85)`, `(35,105)`.
   Start the overshooting tools at `plate_top_y - 0.5`.  The ISO 4762 M3 head
   is Ø5.68 x 3.00, so this provides 0.36 mm radial print allowance and leaves
   a 2.75 mm roof, above the 2.4 mm minimum wall.  Keep all eight M3x10 block
   screws.  Do not countersink the MIC6 plate.

2. `cad/printed.py:nut_bracket`

   Replace the current full-width 8 mm foot with:

   - a 2.4 mm-high web at `x=-84..-60`, `y=plate_top..plate_top+2.4`,
     `z=85..109`;
   - an 8 mm-high local M4 bearing rail at `x=-84..-72`,
     `y=plate_top..plate_top+8`, `z=92..112`;
   - Ø7.2 access channels through only the web, `z=85..92`, centered on the
     T8 flange's 225° and 315° holes.

   Keep the main T8 wall at `z=77..85` and its four Ø3.2 through-holes.  This
   gives every washer a full bearing face while clearing the two lower nylocs.

3. `cad/printed.py:m0_fixed_end_mount`

   Trim the support foot's X maximum from `-35` to `-38`.  This gives the
   moving flag 2.0 mm lateral clearance at every M0 pose.  The M5 center at
   `x=-45` retains a 4.3 mm edge ligament outside its Ø5.4 clearance hole.

4. `cad/hardware_placements.py:carriage_occurrences`

   Split front/rear tower under-head stack datums:

   - front row (`z=64`): washer `y=plate_bottom=-192.00`, nyloc `y=-192.90`;
   - rear row (`z=126`): washer `y=FLAG_BOTTOM_Y=-198.00`, nyloc `y=-198.90`.

   Keep ISO 4762 M4x20 at the front and M4x25 at the rear.  After the ISO 7089
   0.9 mm washer and ISO 10511 5.0 mm nyloc, visible thread is 1.75 mm front
   and 0.75 mm rear.

5. `cad/hardware.py` and `cad/hardware_placements.py`

   Change the four T8 flange screws from ISO 4762 M3x16 to M3x18.  Keep head
   plane `z=73.2`, set washer plane `z=85.0`, and set nyloc start `z=85.55`.
   The stack is 3.8 mm flange + 8.0 mm printed wall + 0.55 mm washer + 4.0 mm
   nyloc = 16.35 mm, leaving 1.65 mm visible thread.  The tip ends at `z=91.2`,
   0.8 mm before the local M4 rail begins.

No shared patch is required for M1: keep ISO 4762 M3x10 x4, head plane
`y=-179.65`.  The existing four-millimetre roof leaves 6.0 mm thread
engagement in the motor.

## SendCutSend and datum preservation

The MIC6 source, 1:1 DXF, outer contour, 14 round holes, motor window, and T8
relief remain byte-for-byte eligible for the existing through-cut preflight.
No countersink, counterbore, tapping, or second-side machining is introduced.
Measured plate volume remains 61,747.022181 mm³ in 6.35 mm ALUMIC6-250.

The stator/spindle datum is unchanged: axis `X=0`, home `Z=95`; MIC6 bottom
`Y=-192.00`, top `Y=-185.65`.  No bearing seat, tower M4 coordinate, motor
face, or MGN12H coordinate moves.

## Intended-contact and dynamic-fit mapping

The following relationships must be whitelisted as documented fits/contacts,
not ignored generically:

| pair/stack | intent |
|---|---|
| plate / tower | printed tower seats on MIC6 top face |
| plate / rear flag | flag seats on MIC6 bottom face |
| MGN screw / plate / tapped block | head bearing, clearance pass, 3.65 mm engagement |
| tower screw / tower / plate [/ flag] / washer / nyloc | complete front/rear clamp stack |
| bracket M4x25 / local rail / plate / washer / nyloc | complete bracket clamp stack |
| T8 M3x18 / flange / wall / washer / nyloc | four equivalent, wrench-accessible stacks |
| M1 M3x10 / four-millimetre roof / motor | head bearing plus 6.0 mm tapped engagement |
| inner-race spacer / outer-race spacer | explicit dynamic fit exemption; 0.20 mm concentric radial clearance, zero positive-volume overlap |
| lower inner spacer / lower DIN 472 ring | explicit dynamic fit exemption; about 0.60 mm concentric radial clearance, zero positive-volume overlap |

The last two exemptions are appropriate because the general 2 mm dynamic
clearance policy cannot apply across a bearing race stack.  They should remain
conditional on a zero-positive-volume Boolean check; do not make a blanket
spindle/carriage exemption.

## Artifact note

This work adds an inspection module, tests, and this report only.  It does not
export or modify a primary STEP, so CAD snapshot/viewer handoff is intentionally
deferred to the production assembly regeneration after the shared patch is
applied.
