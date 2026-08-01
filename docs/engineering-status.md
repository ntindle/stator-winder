# Engineering status notebook

> This is the detailed development snapshot preserved from the original root
> README. Some paths describe locally generated or supplier-provided artifacts
> that are intentionally absent from the public source tree. The root README,
> current audit outputs, and `TODO.md` govern release status.

## 4-Axis Stator Flyer Winding Machine

An original, parametric, 3D-printable flyer winder for small BLDC outrunner
stators. OD 28–65 mm is the launch **verification target**, with 90 mm retained
as an analytical parameter; no launch corner is production-authorized until all
24 OD/stack/wire/workholding certificates pass. The machine is designed as a drop-in
mechanical target for the unmodified
[aotenjo-xyz/winder](https://github.com/aotenjo-xyz/winder) control software
(`M{id}A{rad}` serial protocol). The untouched upstream software has run a raw
24-tooth, 50-turn-per-tooth, 3-phase `continuous_winding()` capture against the
digital machine. `out/capture/upstream_current_raw.jsonl` is the motion-contract
authority. A separate `ContractWind` packing/shaft-wrap experiment remains a
diagnostic design lane and is **not** evidence that the unmodified stream
passes. The selected default-job rigid geometry passes its current integrated
sweep, but hardware winding remains fail-closed: untouched upstream executes
1.375000 and 2.791667 shaft-wrap turns instead of two and two.  The conductor
audit now proves 2,400 held poses and 48 positive-duration constant-route
intervals, but moving lay/index/park/wrap intervals remain unproved; the launch
matrix is therefore still 0/24 production-authorized.

**License:** project software is MIT; original mechanical design source is
MIT or CERN-OHL-P-2.0. Third-party reference assets are outside that grant and
are omitted from the public source snapshot. See `../LICENSES/README.md`,
`../PROVENANCE.md`, and `../NOTICE.md`.

## Architecture (machine frame: Z = flyer axis, Y up, Z0 = flyer plane)

| Axis | Function | Implementation |
|---|---|---|
| M0 | Stator carriage, linear along Z | 2× MGN12H rails @ x±45, T8×8 lead screw (**1.27324 mm/rad**), NEMA17 closed-loop |
| M1 | Stator indexing, vertical axis | Explicit workholder: ER11A for Ø3–7 shafts or the custom `shaft8` socket holder for the Ø8 endpoint; shared Ø8×100 shank in 2× 608ZZ, direct-drive NEMA17, unlimited rotation |
| M2 | Flyer, continuous about Z | Selected Rev-D L79.00 aluminum shaft manufacturing geometry with a full OD10/ID6 stock-pulley seat and OD12/ID9 main span in 2x 6001ZZ, torus-free printed arm, one-piece PEEK guide/bell, six-point balance trim, stock NBK P30-3GT-BLP-6C-10 flyer pulley and 210-3GT-6 belt at exact 1:1, Leadshine closed-loop NEMA17, >=300 RPM |
| M3 | Wire tension | Passive v1: spool + felt drag + spring dancer (1–10 N); `dont_move_m3: true`; NEMA17-pattern mount reserved for future torque axis |

Wire path: spool -> rendered wool-felt pinch -> dancer pulley -> one fixed
entry eyelet on the flyer axis -> through the hollow shaft -> the one-piece
PEEK flyer guide and exit bell -> the M0-following active-sector PEEK guide ->
open short lead-in on the rotating PEEK cap -> tooth. The retired PTFE elbow
and ceramic flyer toroid are not release parts.

`cad/assembly.step` and `out/links/` are the legacy controller-compatibility
baseline, not the selected successor geometry. Review
`out/review/integrated_release_candidate.step` and the integrated adapter player
for the current active-sector/counterweight design.

With CAD Viewer already running on port 4178, use
`http://127.0.0.1:4178/?file=integrated_release_candidate.step` for the selected
assembly.  Do not use `?file=cad%2Fassembly.step` for release review.  The
counterweight load path has its own full and half-section review files at
`out/review/counterweight_attachment.step` and
`out/review/counterweight_attachment_section.step`; the section exposes the
blind printed bosses and inserts that the six balance screws terminate in.

M1 selection is physical, not an OD shortcut: use `--spindle er11` for a
measured Ø3–7 mm shaft and `--spindle shaft8` only for an Ø8.00 mm shaft.
There is no ER8 option. No OD28–65 workholding/job combination is a launch claim
until its exact certificate row passes the 24-case launch authority matrix.
The endpoint matrix is deliberately topology-bound rather than pretending one
lamination exists at every OD: OD28 uses upstream's `dev-12n14p-settings.yml`
pattern with a representative 12-slot core (hub OD 19.5 mm for ER11 or
16.0 mm for the shaft8 holder, each retaining 0.25 mm workholder-reach
reserve); OD65 uses the upstream 24n22p pattern.  This proves representative
feasible laminations only.  Every real stator drawing/measurement must
regenerate packing, slot access, and workholder reach before motion authority.

## Procurement scope

`cad/release_catalog.json` is the purchasing authority; `bom.csv` carries
build quantities and budget estimates but never authorizes a generic
substitute. Required mechanical lines fail closed until an exact compatible
supplier line or complete manufacturing handoff exists. The 36 V M2 supply
condition is required by the bound Leadshine torque curve, but no PSU or exact
CS-D508 current setting is order-authorized. Controller implementation remains
outside the mechanical scope. The PEEK parts, aluminum yoke, stock D10 flyer
P30 retention qualification, conditional D5+BNW motor-side P30, ASTM-B777
trims, and eight unmapped fastener lines are deliberately blocked until exact
supplier/material/pack/cost and coupon evidence exists.

Cart-ready means that the exact supplier SKU is identified; it is not motion
authorization. The Remington 32SNSP.125 wire is 0.0088 inch / 0.22352 mm
supplier-nominal finished OD, and the Nomex Type 410 sheet is 0.005 inch /
0.127 mm supplier-nominal thickness. Before a physical 50-turn job:

1. Measure finished wire OD at no fewer than five separated locations with a
   calibrated wire micrometer. Measure the formed, installed slot-cell coupon
   at no fewer than five locations. For each input, use the conservative
   maximum including instrument uncertainty.
2. Accept only wire inputs from 0.220 through 0.235 mm and installed-liner
   inputs from 0.120 through 0.140 mm. Supplier nominal values are digital-twin
   defaults, not receiving certificates.
3. Regenerate the job:

   ```powershell
   .venv/Scripts/python.exe cad/job_artifacts.py --wire-measured <wire-mm> --liner-measured <liner-mm>
   ```

   Then regenerate the packed routes, capture, continuous audit, and player
   from those same inputs.
4. Do not move hardware unless both `out/reports/slot_wire_routes.json` and
   `out/reports/continuous_wire_audit.json` are hash-bound to the job and say
   `PASS`. The release catalog checks those verdicts instead of treating file
   presence as proof.

## Repository layout

```
cad/params.py        master parametric config (single source of truth)
cad/cots.py          COTS imports (step.parts, sha-verified) + envelopes
cad/printed.py       all 21 printed parts (machine-frame in-place modeling)
cad/assembly.py      legacy 4-link controller baseline; gen_step() -> assembly.step
cad/settings_gen.py  CAD params -> draft settings.yml for the software
cad/job_artifacts.py measured wire+liner -> hash-bound packing/plan/settings
cad/export_links.py  per-link/per-part collision meshes + manifest
cad/loads.py         DoD#5 motor sizing        cad/buildability.py  DoD#6
sim/capture.py       captures raw upstream or an explicit diagnostic adapter
sim/traj.py          velocity-model trajectory reconstruction
sim/collide.py       DoD#2 interference proof   sim/wirepath.py  DoD#3
sim/animate.py       animated GLB + watchable full-cycle player
out/                 generated artifacts (STEP, STLs, GLB, reports)
docs/requirements.md derived requirements (the software contract)
```

## Reproduce everything

```bash
python -m venv .venv && .venv/Scripts/pip install build123d bd_warehouse \
  trimesh manifold3d python-fcl rtree shapely==2.1.2 numpy scipy pyyaml pydantic pygltflib pytest
.venv/Scripts/python.exe cad/settings_gen.py --spindle er11 # -> out/settings.yml
.venv/Scripts/python.exe cad/assembly.py
.venv/Scripts/python.exe cad/export_links.py --spindle er11
.venv/Scripts/python.exe sim/capture.py --controller upstream --settings out/settings.yml -o out/capture/upstream_current_raw.jsonl
.venv/Scripts/python.exe sim/verify_cycle.py --capture out/capture/upstream_current_raw.jsonl --report out/reports/upstream_current_raw_cycle.json --expect-controller upstream
.venv/Scripts/python.exe sim/shaft_wrap_regression_evidence.py
.venv/Scripts/python.exe sim/collide.py --workers 1 --capture out/capture/upstream_current_raw.jsonl --output out/reports/clearance_upstream_raw.json
.venv/Scripts/python.exe sim/wirepath.py --capture out/capture/upstream_current_raw.jsonl --output out/reports/wirepath_upstream_raw.json
.venv/Scripts/python.exe sim/animate.py --capture out/capture/upstream_current_raw.jsonl --speed 10 --output out/winding_cycle_upstream_raw.glb --html out/play_animation_upstream_raw.html
.venv/Scripts/python.exe cad/custom_parts.py
.venv/Scripts/python.exe -c "import sys; sys.path.insert(0, 'cad'); from build123d import export_step; import carriage_active_sector_terminal_guide as m; export_step(m.gen_step(), str(m.STEP_OUT))"
.venv/Scripts/python.exe sim/carriage_active_sector_terminal_guide_audit.py
.venv/Scripts/python.exe -c "import sys; sys.path.insert(0, 'cad'); from build123d import export_step; import integrated_felt_contact_review as m; export_step(m.gen_step(), str(m.STEP_OUT))"
.venv/Scripts/python.exe cad/integrated_felt_contact_review.py
.venv/Scripts/python.exe -c "import sys; sys.path.insert(0, 'cad'); from build123d import export_step; import integrated_release_candidate as m; export_step(m.gen_step(), str(m.STEP_OUT))"
.venv/Scripts/python.exe cad/integrated_release_candidate.py
.venv/Scripts/python.exe cad/custom_drawings.py --include-successor
.venv/Scripts/python.exe cad/successor_manufacturing.py --include-active-sector
.venv/Scripts/python.exe cad/loads.py
.venv/Scripts/python.exe cad/buildability.py
.venv/Scripts/python.exe cad/felt_loads.py
.venv/Scripts/python.exe cad/carriage_hardware_audit.py
.venv/Scripts/python.exe cad/frame_hardware_audit.py
.venv/Scripts/python.exe cad/m2_m3_hardware_audit.py
.venv/Scripts/python.exe cad/integrated_export_player_adapter.py
.venv/Scripts/python.exe sim/continuous_conductor_route.py
.venv/Scripts/python.exe sim/full_cycle_continuous_conductor_authority_audit.py
.venv/Scripts/python.exe sim/cap_live_tail_manufactured_support_trade.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_locus_study.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_g0_normal_audit.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_g0_landing_trade.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_cad_audit.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_hardware_qualification.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_retraction_topology.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_retraction_procurement.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_custom_return_screen.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_integration_audit.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_replacement_architecture.py
.venv/Scripts/python.exe cad/aggregate_boundary_follower_replacement_carriage.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_replacement_cad_audit.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_replacement_transition_sweep.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_replacement_load_wear.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_route_sweep.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_c1_rebound_sweep.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_placement_trade.py
.venv/Scripts/python.exe cad/aggregate_boundary_follower_successor_prototype.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_successor_prototype_audit.py
.venv/Scripts/python.exe -m pytest -q cad/test_aggregate_boundary_follower_successor_prototype.py sim/test_aggregate_boundary_follower_successor_prototype_audit.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_successor_prototype_placement_collision_audit.py
.venv/Scripts/python.exe -m pytest -q sim/test_aggregate_boundary_follower_successor_prototype_placement_collision_audit.py
.venv/Scripts/python.exe sim/aggregate_boundary_follower_acceptance.py
.venv/Scripts/python.exe sim/integrated_candidate_player_adapter.py --adapter-root out/review/integrated_adapter --capture out/capture/upstream_current_raw.jsonl --conductor-route out/review/integrated_adapter/reports/continuous_conductor_route.json --speed 10
.venv/Scripts/python.exe sim/launch_envelope_authority.py --allow-fail
.venv/Scripts/python.exe sim/report.py
.venv/Scripts/python.exe cad/procurement.py
.venv/Scripts/python.exe cad/release_readiness.py
```

After each integrated STEP export, regenerate its native GLB and mandatory
snapshot packet in CAD Viewer and visually review it before writing the final
felt/candidate report. The full-cycle, launch, validation, procurement, and
readiness commands intentionally return nonzero while their honest gates remain
open; that is a release result, not a reason to substitute stale outputs.

The integrated adapter exporter writes only to
`out/review/integrated_adapter_releases/<current release id>` and emits
`release_identity.json`. It intentionally refuses to write through the
`out/review/integrated_adapter` selector junction, because doing so would
overwrite the previously selected immutable release. Repoint that selector
only after the new identity-named directory has passed its manifest, collision,
player, and visible-review checks.

Open `out/review/integrated_adapter/play_integrated_candidate_raw.html` to
pause, scrub, step through all
captured M0/M1/M2/M3 commands, select slow-motion speeds/camera views, and watch
each of the 24 passes start and display 50 captured M2 revolutions. Those are
not 50 physically authorized deposited turns. The reconstructed
axis motion and raw M0 radial progression are capture authority.  Tangential and
layer placement of the displayed deposited coil remains an explicitly labelled
elastic approximation, not a route proof. `clearance_upstream_raw.json` and
`wirepath_upstream_raw.json` remain lower-level legacy-baseline checks. The
matching `winding_cycle_upstream_raw.glb` / `play_animation_upstream_raw.html`
player is also a non-governing legacy baseline: it shows captured rigid-axis
motion, the two rigid guide-wire meshes, and disconnected approximate turns,
but deliberately renders no flexible live span and makes no conductor-
continuity claim because `out/links/manifest.json` has no continuous-route or
active-terminal-locus contract. The integrated-adapter player above remains the
selected player for conductor review. The
selected rigid authority is the active-sector audit plus integrated candidate;
the conductor gate is `full_cycle_continuous_conductor_authority_audit.json`.
`continuous_conductor_route.json` is presentation-only and remains explicitly
FAIL. Exact layer order is deliberately not a release claim; `GOAL.md` reserves
layering neatness for hardware qualification.

The selected wire-visibility repair release is the immutable bundle
`out/review/integrated_adapter_releases/iar1-b1e34f1942705a460d14`.  It keeps
the exact machine-side terminal prefix cyan and draws the previously missing
cap-to-live-tail continuation red/dashed with an explicit no-support-owner
label.  It was promoted to the `integrated_adapter` selector only after its
contract tests and visible browser review passed; the dashed span is not
conductor authority.

The bounded support trade rejects all seven previously modeled manufactured
families.  Its first isolated gimbal-shoe prototype remains useful geometry
evidence, but it is neither an additive carriage module nor the final route
owner: the integration audit finds 21 positive-volume pairs at its authored
keyed-tower placement.

The mechanically finalized replacement review STEP instead uses one 6061
U-windowed carrier and four handed, carriage-owned follower occurrences.  A
selected shoe sits at |Y|=2.05 mm while its sibling parks at |Y|=10.95 mm, so
the coarse selector stroke is 8.90 mm before the passive +/-0.50 mm float.  Its
69 manufactured leaves plus four blocker-only linkage envelopes are 73 STEP
leaves.  Across all 36 selector/gate states, the exact BREP audit finds zero
positive-volume pairs; nominal sibling/pivot clearance is 3.00 mm and the
worst passive extreme retains 2.50 mm.  The sampled transition audit covers
232 poses with zero collisions or clearance violations and the same 2.50 mm
minimum.  This is mechanical envelope evidence only: the blocker envelopes
are not a manufactured selector, and positive M0 retraction/interlocks and a
tolerance-qualified mechanism remain missing.

The load/wear report remains FAIL despite its nominal screens.  High-side
passive bias reaches 1.982709251 N, only 0.017290749 N below the 2 N cap, and
the conservative local screen is 41.982709251 N for the bound 40 N / 5.52 N m
case.  Fastener external stress and bushing static pressure screens do not
qualify joint preload, inserts, retention, carrier stress, wear, fit, duty,
fatigue, or endurance.

The route sweep classifies all 2,400 loci for both wire diameters (4,800 cases),
and the rebound sweep finds exact analytic C1 S-biarcs for all 4,704 nonzero
cases.  They place zero positive-volume arcs and authorize zero physical cases.
The placement trade confirms the present circular R3 nose is referenced to the
wrong absolute radial/axial datum: its envelope covers 0 of 4,704 required C1
centres, and its curvature direction is 90 degrees from the required aggregate
compression normal.  Do not integrate that replacement carriage as the final
guide.

The selected next topology is four M0-owned, re-datumed XYZ flexure/slide
stages, each with a yaw/elevation-compliant polished C1 guide cartridge and
mechanically separate aggregate-normal preload leaf and polished PEEK shoe.
That topology now has an isolated positive-volume prototype: four
identity-specific modules with 21 single-solid leaves each, 84 STEP leaves in
all.  It models 1.50 x 2.40 x 1.10 mm XYZ travel, +/-55 degree yaw stops, and
+/-10 degree elevation tabs, exceeding the exact placement-trade minima.  Each
module includes an R5 floor-relief coupon targeting 2.00 mm clearance around
the conservative R3 guide envelope.  The STEP is intentionally arranged as a
2 x 2 review rack, not at asserted assembly placements.

The hash-bound prototype audit passes, as do its five CAD and two audit tests
(7 total).  This grants isolated topology and positive-volume evidence only.
The subsequent placement/collision audit and its three focused tests complete
successfully, but the audit verdict is
`PASS_AUDIT__PROTOTYPE_NOT_PLACEMENT_OR_COLLISION_READY`.  Center bounds,
modeled travel, and numeric yaw/elevation range cover all 4,704 analytic cases;
the actual prototype rotation realizes the requested tangent in 0/4,704.  R3
stays inside the fixed R5 relief in every case, but the full 2.00 mm margin
survives in 0/4,704 and falls to 0.635960420 mm worst case.  Exact BREP finds
12 guide-floor collisions.  The 172 endpoint poses expose 34 unique module
self-collision pairs, eight own-floor leaf pairs, and eight sibling pairs when
rebased to exact active-local coordinates.  The spaced review rack itself has
zero sibling collisions, but that result is not assembly evidence.

Successor v2 must use a guide-frame transform that realizes every requested
tangent, add a true parametric elevation member and modeled stops, and resize
or reshape the relief for a full 2.00 mm margin throughout travel.  It must
remove the 12 guide-floor, 34 self, eight own-floor, and eight rebased-sibling
collision findings in a real shared carrier with positive-volume fasteners and
flexures.  Then it needs all-4,704 direct checks plus continuous five-DOF
motion sweeps, followed by flexure/preload load sizing, tolerance, wear, wire
route/dynamics, buildability, machine integration, procurement, BOM,
production, and release qualification.  Until those pass, the player retains
the red/dashed span.

Governing evidence is in
`out/reports/aggregate_boundary_follower_replacement_cad_audit.json`,
`out/reports/aggregate_boundary_follower_replacement_transition_sweep.json`,
`out/reports/aggregate_boundary_follower_replacement_load_wear.json`,
`out/reports/aggregate_boundary_follower_route_sweep.json`,
`out/reports/aggregate_boundary_follower_c1_rebound_sweep.json`, and
`out/reports/aggregate_boundary_follower_placement_trade.json`.  The review geometry is
`out/review/aggregate_boundary_follower_replacement_carriage.step` with its
`out/review/aggregate_boundary_follower_replacement_carriage_manifest.json`
manifest.

The isolated successor handoff consists of source
`cad/aggregate_boundary_follower_successor_prototype.py`, brief
`cad/aggregate_boundary_follower_successor_prototype_brief.md`, snapshot job
`cad/aggregate_boundary_follower_successor_prototype.snapshots.json`, STEP
`out/review/aggregate_boundary_follower_successor_prototype.step`, manifest
`out/review/aggregate_boundary_follower_successor_prototype_manifest.json`,
and audit
`out/reports/aggregate_boundary_follower_successor_prototype_audit.json`.
Reviewed views are
`out/review/snapshots/aggregate_boundary_follower_successor_iso_20260713T050344Z.png`,
`out/review/snapshots/aggregate_boundary_follower_successor_front_20260713T050344Z.png`,
`out/review/snapshots/aggregate_boundary_follower_successor_right_20260713T050344Z.png`,
and
`out/review/snapshots/aggregate_boundary_follower_successor_top_20260713T050344Z.png`.
The fail-closed placement/collision layer is source
`sim/aggregate_boundary_follower_successor_prototype_placement_collision_audit.py`,
tests
`sim/test_aggregate_boundary_follower_successor_prototype_placement_collision_audit.py`,
JSON authority
`out/reports/aggregate_boundary_follower_successor_prototype_placement_collision_audit.json`,
and readable report
`out/reports/aggregate_boundary_follower_successor_prototype_placement_collision_audit.md`.

The final source architecture uses two 24-sector rotating PEEK caps and two
M0-following, M1-static active-sector PEEK guides. Four M3 stacks clamp the
guides to a keyed aluminum yoke and four M4 stacks attach that yoke to the
revised spindle tower. The torus-free flyer carries a removable one-piece PEEK
guide/bell and six serialized ASTM-B777 trims: four rear slugs retained by four
printed three-post caps plus two front trims. Geometry, raw-cycle clearance,
continuous-conductor authority, material/finish DFM, supplier drawings, insert
pull coupons, hot endurance and installed G2.5 balance remain independent
release gates; a green analytical model is not an order authorization.

The four rear counterweight screws pass through each slug/spacer/retainer into
heat-set inserts in continuous printed bosses with blind caps. The two front
M2x8 screws enter standard inserts in blind spoke pilots. That closes the CAD
load paths; insert-fit and per-stack pull coupons remain open.

Serve the repository root (for example, `.venv/Scripts/python.exe -m
http.server 4180 --bind 127.0.0.1`) and open
`http://127.0.0.1:4180/out/review/integrated_adapter/play_integrated_candidate_raw.html`.
A local server
is preferred over `file://` so the browser can load the Three.js modules
consistently.

Each of the 24 tooth passes has an explicit **COIL START** marker before its
positioning move, followed by the first physical placement crossing; phase,
tooth, direction, side, and turn index are shown independently so a positioning
move cannot be mistaken for deposited copper.  Felt pads render dark brown and
rough, while their stud/washer hardware remains metallic grey.  Wire contact
between the two pads is the intentional friction pinch; contact with the felt
base or stud is not.

## Build notes (assembly order)

1. **Frame:** base layer = 3 cross 2020 (z −180/−50/+160) under 2 long
   450 mm rails (x ±80) + 2 stringers (x ±45); install the 15 audited
   MISUMI HBKTST5 brackets and the printed rear-post shoe where a 25 mm
   bracket leg cannot fit.
   Bolt the two 235 mm posts (x ±80, z −50) and the 305 mm rear post (x 0).
2. **M0:** rails on stringers (12 mm shims to rail datum edge), carriage
   plate onto both MGN12H blocks; screw end block + M0 motor mount on the
   −X base rail; couple screw; endstop pedestal on the front cross.
3. **M1:** choose the holder that matches the measured shaft. For Ø3–7,
   use the ER11 chuck and correct collet; for Ø8.00, use the custom OD16
   socket holder from `out/custom/step/shaft8_socket_holder.step` and its
   supplier drawing. Press the 608s into the tower, drop the selected
   holder's common Ø8 shank through, and join it to the motor with the beam
   coupling. The custom holder uses two M4x5 cup-point screws; matching shaft
   flats are strongly recommended.
4. **M2:** press the 6001s into the flyer block; install the selected Rev-D
   L79.00 shaft with its full OD10 h6/ID6 rear seat and OD12/ID9 main span;
   seat the stock NBK P30-3GT-BLP-6C-10 pulley hub-rear on the round D10 seat,
   torque its supplied M2 clamp bolt to 0.5 N m, and retain the torus-free arm
   on the only two shaft flats; install the 210-3GT-6 belt to the official
   motor-side NBK P30. Mount the one-piece PEEK guide/bell
   with three M2x6/standard-insert stacks. Install the four serialized rear
   B777 slug/printed-retainer stacks with M3x6/94459A130 hardware and the two
   front trims with M2x8/washer/standard-insert hardware. Do not substitute the
   retired printed 40T pulley, 200-2GT belt, ceramic toroid, central M3x12 screw
   or three-washer balance stack. Complete pull tests, 300 RPM hot endurance
   and installed G2.5 balance before winding.
5. **M3:** spool bracket/drum, felt stack, dancer, entry eyelet bracket on
   the rear post; thread the wire path.
6. Home M0 (tab → switch), set `zero` offsets, run
   `python scripts/main.py -s` first, then live.

Weight remains approximately 4 kg with a 180 x 450 mm footprint and roughly
340 mm height. The current numeric planning subtotal is `$1,573.78`; it
deliberately excludes every blocked/TBD successor custom part and fastener
delta, the required regulated 36 V supply integration, and any NBK BNW quote
delta. Job-specific winding wire and Nomex prices are also excluded. This is a
partial budget annotation, not proof that blocked supplier lines are ready.
Third-party CAD provenance: NOTICE.md.

## What the simulation does NOT prove

Wire tension dynamics, snagging, layering neatness, enamel abrasion, belt
resonance, and printed-part fatigue are empirical; see
out/reports/validation.md §limitations. A PASS rigid collision report does not
prove the passive wire's path between sampled poses. Only the current,
hash-bound packed-route and continuous-motion reports can authorize a measured
job, and either missing or non-PASS verdict keeps hardware winding blocked.
