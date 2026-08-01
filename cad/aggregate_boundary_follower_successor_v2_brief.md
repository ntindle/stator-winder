# Aggregate-boundary follower successor V2 CAD brief

- Model: an isolated, active-local replacement for the failed V1 review-rack
  follower.  It retains one M0-owned 6061 U-carrier, puts four handed XYZ
  compliance pods on the two outboard carrier rails, carries each guide on a
  keyed boom, and provides a two-axis bearing gimbal, a replaceable polished
  PEEK C1 guide, and a mechanically independent normal-preload leaf and shoe.
- Authority: review-only.  This source is not imported by `assembly.py`, the
  animation/player, the selected integrated-adapter release, the BOM, or any
  manufacturing release.  Integration remains fail-closed until the exact
  4,704-case placement, full five-DOF collision, load, fatigue, wear,
  tolerance, route, and buildability gates pass.
- Units/frame: millimetres in the active-stator local frame: +X radial
  outward, +Y tangential, +Z stator axis.  At M0 home,
  `machine=(-local_y, local_z, 95-local_x)`.  M1 and M2 select an identity but
  do not spatially transform any V2 part.
- Frozen evidence: fail closed against
  `out/reports/aggregate_boundary_follower_placement_trade.json`.  Use all
  exact case centres, guide tangents, and contact-to-centre curvature normals;
  do not synthesize a guide pose by independently averaging angles.  The STEP
  neutral pose uses one real, source-keyed 0.2 mm-wire case per identity.
- Exact guide frame: authored local +X is the wire tangent, local +Y is the
  contact-to-R3-centre curvature normal, and local +Z is their cross product.
  Build with `Plane(origin=center, x_dir=tangent, z_dir=tangent x normal)`.
  The legacy `Rot(0,-elevation,yaw)` order is forbidden.
- Carrier: start from the current one-solid U-carrier corridor, preserve the
  migrated four-screw tower mount and datum keys, omit the old selection-bay
  floors/walls, and remove one shared through-window covering
  X=3.10..23.80, Y=-9.75..7.75, Z=-13.35..13.35 mm.  No carrier material may
  occupy the moving guide/head keepout.  Four keyed pod pads are attached only
  to the outer Y=+/-39.5 mm rail faces.
- Remote pods: front pod bases are centred at active-local Z=-0.5 mm, rear at
  Z=-12.5 mm; left/right pod centres are Y=-45.5/+45.5 mm.  Each pod provides
  at least +/-0.75 X, +/-1.20 Y, and +/-0.55 Z usable translation with distinct
  hard stops outside the usable range.  Folded-flexure blades are visible,
  positive-volume members; no overlapping boxes may stand in for a slide.
- Booms: the stage output first reaches an X=27 mm tangential trunk, then
  approaches its head from Y=-10.5 mm (left identities) or +10.5 mm (right
  identities).  Front and rear booms are independently keyed and remain
  separate.  Initial terminal section is no larger than 2.0 radial x 1.5
  axial; stiffness and vibration authority remain open until calculation and
  physical coupons close them.
- Head: yaw is about active-local +Z and elevation is about the yawed
  binormal axis.  Each axis uses one handed, one-sided barrel containing two
  NMB L-630ZZ bearings (3 x 6 x 2.5 mm), a McMaster 90265A115 OD3 x 10 mm/M2
  shoulder screw, two DIN988 3 x 6 x 0.5 shims, a 4 mm inner spacer, matching
  outer spacer, M2 nyloc, and a two-screw keeper.  The local 623ZZ part is
  rejected because its 10 mm OD cannot fit the 7.558 mm same-face head pitch.
- Guide: one positive-volume, virgin-unfilled PEEK cartridge per identity.
  Its open channel is straight--R3 quarter arc--straight with C1 joins.  A
  negative-Z backing web and hub mechanically connect the channel to the
  elevation pivot without closing the positive-Z wire loading opening.
- Preload: one independent 17-7PH leaf, M2 adjuster/jam nut, overtravel stop,
  and replaceable polished PEEK shoe per module.  It shares no body or
  fastener with the wire guide.  Guide/wire and shoe/aggregate are the only
  intended process contacts.
- Pod attachment: four M3 x 14 socket screws, four M3 washers, and four short
  M3 heat-set inserts per printed pod, plus two integral carrier keys.  The
  keys carry shear; the screws clamp.  Existing four NBK M4 tower screws and
  short M4 inserts remain unchanged.
- Manufacturing intent: carrier and boom are machined 6061/7075 unless later
  load work selects 17-4PH terminal members; flexure blades are 17-7PH;
  prototype pod cages are PA12-CF with heat-set-insert coupons; guide/shoe are
  machined natural unfilled PEEK with polished contact surfaces.  Bearing
  seats, shoulder bores, flexure edges, insert pullout, and all fits require
  drawing tolerances and qualification before procurement authority.
- Collision policy: positive overlap is never exempt for pivots, bores,
  guide/floor, guide/preload, flexure frames, boom/carrier, or siblings.
  Explicitly named bearing fits, insert pilots, screw/thread engagement, and
  fused carrier features are assembly interfaces, not general collision
  waivers.  Hard stops may touch only outside usable motion.
- Required artifacts: source, STEP, manifest, orthographic/isometric
  snapshots, exact all-4,704 guide/window audit, adaptive five-DOF
  self/carrier/sibling sweep, load/fatigue/wear/tolerance report, and focused
  tests.  Acceptance/docs may bind V2 only after those isolated gates pass.
