# M2/M3 source-level hardware audit

Scope: current build123d/OpenCascade solids from `printed.py`, `cots.py`,
`hardware.py`, and `hardware_placements.py`.  The audit is isolated and does
not modify shared machine geometry.  It samples the dancer from -3.0 to +5.5
degrees at 0.25 degree increments and checks candidate repairs with exact BREP
Boolean intersections.

Result: **62/62 exact checks pass**.  Audit regression tests: **7/7 pass**
(**26/26** in the focused flyer/hardware suite).
Machine-readable evidence is generated at
`out/reports/m2_m3_hardware_audit.json`.

## Required shared-source patches

1. **M2 mount low-head access** (`printed.m2_motor_mount`)
   - Current left/right low M5x12 heads each embed in the join block by
     236.1413 mm3; the high pair is clear.
   - Subtract two OD10 axial tool/head tunnels at x=+-80, y=-72,
     z=-103..-66.  The z=-66 end is the existing screw bearing plane.
   - Candidate overlap is 0 for all four screws, the mount remains one solid,
     and 5363.81 mm3 is removed.

2. **Flush all eight rear-post base screws** (`printed` + hardware schedule)
   - Replace the spool/felt/dancer/entry M5x12 socket heads with ISO 10642
     M5x12 countersunk screws at the same axes and top plane z=-164.
   - Add OD10/OD5.4, 90 degree countersinks, depth 2.3, to all eight base
     holes.  This uniform choice avoids four separate head-access rules.
   - Current dancer-head/arm overlap reaches 128.157 mm3.  One entry head
     reaches 35.739 mm3.  Felt heads overlap each OD20 backing/pad stack by
     55.734 mm3 per side and also intersect the printed boss.
   - Candidate overlap is zero.  Dancer arm axial clearance is at least
     1.000 mm throughout the complete stop range; stop axes do not move.

3. **Entry lower-right moving-stack relief** (`printed.entry_bracket`)
   - Subtract the box x=-38.5..-27, y=-22..-7, z=-171..-163.5.
   - Current moving pulley nyloc overlap is 5.5423 mm3 and the last three
     rear shims each overlap by 1.4874 mm3.  The candidate has zero overlap;
     minimum repaired gaps are 2.7447 mm for the nyloc and 2.2655 mm for the
     shims.
   - The bracket remains one solid; 255.001 mm3 is removed.  The wire passage,
     fixed spring bridge, base screw axes, and both dancer stop centers remain
     unchanged.

4. **Flush moving spring-anchor head** (`printed.dancer_arm` + schedule)
   - Current M2x16 socket head overlaps the entry base by 10.4331 mm3.
   - Replace it with ISO 14581 M2x16 and cut an OD4.8/OD2.4 90 degree rear
     countersink, 1.2 mm deep, top plane z=-163, on the existing moving-anchor
     axis.  The 2.5 mm arm retains a 1.3 mm floor.
   - Candidate screw/arm overlap is zero, the arm remains one solid, and its
     minimum entry-bracket gap is 1.000 mm across the full sweep.

5. **M2 internal running clearances** (`assembly`/`printed`)
   - Change the outer-race spacer ID20 to ID22 while retaining OD27.8 and
     length 11.  Exact inner/outer-spacer gap rises from 1.000 to 2.000 mm.
   - Change the flyer-block through-running bore from OD25 to OD26.  Exact
     block clearances become: pulley 2.500 mm, pulley heat-set insert
     2.240 mm, pulley M3 screw 2.428 mm.
   - These are rotating clearances, not intended-contact whitelist pairs.

6. **Felt stud usable thread** (`hardware` + placement)
   - Replace M4x50 with M4x55, same rear datum z=-170.
   - Current thread is only 0.0867 mm proud of the wingnut; candidate is
     5.0867 mm proud.
   - Candidate clearance is 3.400 mm to the conservative OD2 wire model and
     37.779 mm to the exact belt solid.

7. **Counterweight clamp stack** (`hardware_placements`)
   - The three 0.55 mm M3 washers contact each other exactly, but the screw
     bearing plane is 0.55 mm ahead of the stack (exact solid distance
     0.3706 mm because of the head edge).
   - Move only the M3x12 screw origin from z=-11.8 to z=-12.35.  The candidate
     washer/head contact is zero-gap and zero-volume.

8. **Counterweight structural attachment** (`printed.flyer_arm`)
   - The old boss/collar joint had 0.000 mm3 volumetric overlap and depended
     on one coincident rear face.  The two Ø3.3 holes at x=+-6, y=-32 were
     unloaded trim bores, not fasteners, and made the attachment ambiguous.
   - Both trim bores are removed.  An integral PETG web spans x=+-7,
     y=-25..-8.45, z=-28.8..-20.  Exact overlap is 55.357 mm3 into the hub
     collar and 1643.683 mm3 into the counterweight boss; the finished flyer
     arm remains one solid.
   - The web stays 2.400 mm outside the Ø12.1 shaft passage, has 2.200 mm
     clearance to the stationary flyer block, and the complete rotating arm
     retains its existing 2.000 mm minimum block clearance.
   - The one real counterweight axis remains x=0, y=-25.  The M3 screw spans
     the full 4.300 mm McMaster 94459A130 insert; its intended heat-set
     interference is 4.210 mm3.  The three washers are the only tuning stack.
   - The solid boss outer radius is 42.4 mm, still inside the tip-guide sweep.
     Integrated balance is -0.5 g-mm residual (0.00 N rounded at 300 RPM);
     M2 simulation-duty torque margin remains 2.36x.

## Verified stacks and intended contacts

- Spool: OD8 axle has positive clearance through bracket and drum; both
  washers seat against their ears and the right nyloc seats against its
  washer.  No geometry change is required.
- Felt: fixed backing/pad, moving pad/backing, spring/thrust washer, and
  thrust washer/wingnut are zero-gap, zero-volume contacts.  The two pads have
  a 0.500 mm rigid-model wire gap.
- Flyer heat-set inserts: positive overlap is intentional plastic
  displacement, not a collision.  Exact embeds are 5.5953 mm3 for each arm
  insert, 15.7158 mm3 for the pulley insert, and 4.2102 mm3 for the
  counterweight insert.
- Screw shanks in clearance holes must have zero positive volume.  Threaded
  screw/nut or screw/insert pairs may be tangent in the simplified BREP.
- Screw-head/washer bearing interfaces must be zero-gap and zero-volume.
- Dancer stop sleeve/arm contact is allowed only at the specified -3.0 and
  +5.5 degree endpoints; no positive-volume embedding is allowed.
- Inner/outer M2 spacers and flyer clamp hardware/flyer block are running
  clearance pairs and must not be added to an intended-contact whitelist.

All added material is localized to the counterweight web/boss and was checked
explicitly against the shaft passage, stationary flyer block, and rotating
envelope.  The longer felt stud was also checked explicitly as reported above.
