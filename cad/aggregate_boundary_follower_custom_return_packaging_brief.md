# Aggregate follower custom-return packaging CAD brief

- Model: isolated one-occurrence follower custom-return packaging assembly.
- Task type: new review-only assembly; it does not modify or feed the main
  follower, assembly, player, BOM, procurement, or release sources.
- Units: millimetres.
- Coordinate convention: existing active-tooth local frame; +X is radial
  outward, +Y is tangential, and +Z is the stator axis.
- Review state: radial slide midpoint at X=17 and tangential center at Y=0.
- Tangential hardware axis: +Y through X=17/Z=20.
- Tangential package: nominal OD3 x 16 shaft; igus WPFFM-0304-05 envelope with
  ID3, body OD4.5 x 5 and flange OD7.5 x 0.75; provisional OD4.54 body pocket
  and OD7.54 flange counterface; two unselected Ø0.30 wire, 4.00 mean-diameter,
  2.64-active-turn 17-7PH torsion-spring geometry envelopes; fixed indexed
  anchor plates and moving anchor tabs.
- Radial package: fixed 9293K122 coil envelope, OD15.75 x 6.35, in the screened
  negative-Y service pocket; closed containment envelope with strip exit;
  below-pocket reduction lever with three adjustment holes spanning the
  analytical 0.235-0.315 work-ratio range; fixed pivot bracket and moving
  radial-slide output anchor envelope.
- Positioning: explicit named source coordinates are the authority. The shaft
  and bushing are coaxial; the yoke and current radial-slide context meet only
  at the provisional Z=16 interface; fixed anchor plates meet the yoke ears at
  Y=+-6.5; the radial cartridge is fixed and is not attached to the LEM
  bellcrank.
- Output paths:
  `cad/aggregate_boundary_follower_custom_return_packaging.py`,
  `out/review/aggregate_boundary_follower_custom_return_packaging.step`, and
  `out/review/aggregate_boundary_follower_custom_return_packaging.json`.
- Validation targets: every custom body is one positive solid; exact
  positive-volume same-state overlap count is zero at tangential -0.6/0/+0.6;
  all catalog envelopes also remain non-overlapping in those states; shaft and
  bushing dimensions close; cartridge containment stays in X16.25-33.75,
  Y-28--11.5, Z16-24; full package including current tongue context stays
  inside X8-34, Y-28-8, Z5.5-26;
  STEP labels, topology, positioning facts, and multi-view snapshots inspect
  cleanly.
- Assumptions: torus-ring spring geometry is a conservative visible wire
  envelope rather than manufacturing-authoritative helical wire; the igus
  pocket clearance is provisional and does not claim the vendor's installed
  tolerance; the constant-force coil is a solid collision envelope, not strip
  manufacturing geometry; attachment screws, pivots, strip routing details,
  and guards remain unresolved.
- Fail-closed boundary: load, spring rate, fatigue, procurement, physical fit,
  assembly integration, collision release, BOM, order, production, and release
  authority all remain false.
