# Aggregate-boundary follower successor prototype CAD brief

- Model: isolated positive-volume successor-follower review rack. Four
  identity-specific modules represent the selected placement-trade topology:
  an M0-owned re-datumed XYZ stage, yaw/elevation compliance, a polished C1
  guide cartridge, and a mechanically separate aggregate-normal preload leaf
  and shoe.
- Task type: new review-only prototype. It is deliberately not imported by
  `assembly.py`, the player, the selected replacement carriage, any BOM, or
  any release source.
- Units and authority frame: millimetres in the active-stator local frame,
  where +X is radial/outward, +Y is tangential, and +Z is the stator axis. At
  M0 home, `machine=(-local_y, local_z, 95-local_x)`.
- Source contract: fail closed against
  `out/reports/aggregate_boundary_follower_placement_trade.json`; the report
  must retain its internal canonical SHA, selected topology, four identities,
  exact centre bounds, and false physical/release authority.
- Exact identity data: copy every active-local target datum, min/max centre
  bound, and tangent datum from the placement trade without rounding. The
  STEP uses a documented 2 x 2 review-rack display transform; tiny positive-
  volume min/max witnesses are placed by `(active point - active datum)` so
  the exact identity spans remain inspectable and invertible without claiming
  assembly placement.
- Common stage capacity: model at least 1.3822561230042538 x
  2.233484956719163 x 0.9733701456993078 mm XYZ centre travel. The prototype
  uses 1.50 x 2.40 x 1.10 mm and explicit rail, moving bridge, and hard-range
  witness geometry at the neutral pose.
- Orientation capacity: model at least +/-53.23669873274605 degrees yaw and
  +/-9.086049191773341 degrees elevation around each identity datum. The
  prototype uses visible +/-55 degree yaw stops and +/-10 degree elevation
  stop tabs. These are geometry envelopes, not a tolerance/load proof.
- Guide cartridge: one positive-volume polished open-channel cartridge per
  identity. Its review centreline is straight--R3 quarter arc--straight with
  tangent-continuous joins; the shell has a complete loading opening. Each
  cartridge is posed from the identity's exact guide-tangent datum. It is a
  topology prototype only, not a positive-volume surface proved over all
  4,704 analytic cases.
- Preload: one separate spring-leaf body and one separate polished shoe per
  identity. The shoe direction uses the datum radial aggregate-normal witness
  `(datum_X, datum_Y, 0)` normalized. No leaf force, stress, fatigue, wear,
  thermal, or retention claim is made.
- Carrier-floor relief: one positive-volume floor coupon per module is cut by
  an R5.00 spherical exclusion about the target centre. This targets 2.00 mm
  material clearance around the conservative R3 envelope and is intentionally
  separate from the selected carrier. It does not prove the complete carrier,
  fastener, stage, or transition collision envelope.
- Export: source `cad/aggregate_boundary_follower_successor_prototype.py`;
  STEP and manifest under `out/review/`; audit reports under `out/reports/`;
  required orthographic/iso snapshots under `out/review/snapshots/`.
- Validation targets: four modules; positive volume; single-solid custom
  leaves; exact report-bound datums/bounds; modeled travel and angle capacity;
  C1 tangent continuity; separate guide/leaf/shoe bodies; R5 relief cutter;
  source/report/STEP hash bindings; baseline STEP refs/facts/planes/positioning;
  and reviewed multi-view snapshots.
- Explicit blockers: full 4,704-case positive-volume guide surface, mechanism
  collision sweeps at all XYZ/yaw/elevation combinations, flexure sizing,
  preload force/stress/fatigue, guide finish and wear qualification, tolerances,
  fasteners/retention, assembly placement, wire route, dynamic snag/sag, build
  process, procurement, BOM, production, and release.

