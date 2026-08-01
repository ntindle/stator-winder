# Aggregate-boundary follower replacement-carriage CAD brief

- Model: isolated replacement carriage prototype for the aggregate-boundary
  follower; one shared carrier plus four handed moving follower occurrences.
- Task type: new review-only assembly. It is deliberately not imported by
  `assembly.py` and does not modify the player, BOM, procurement, or release
  catalog.
- Units: millimetres.
- Coordinate convention: active-stator local frame. +X is radial/outward, +Y
  tangential, and +Z is the stator axis. At M0 home,
  `machine=(-local_y, local_z, 95-local_x)`.
- Shared carrier: derive from the collision-cleared
  `carriage_active_sector_terminal_guide.carriage_yoke()` corridor; preserve
  its U-window, outboard rails, and tower datum keys. Restore the obsolete M3
  holes to solid aluminum; the obsolete guide-key fill is deliberately
  subsumed by the larger parked-follower clearance relief. Add four integral
  selection-bay floors/outboard walls, replace the impossible straight M4
  pattern with four diagonal recessed-head pockets, cut four local
  parked-follower reliefs at X 25.00..36.20, |Y| 5.45..16.50 and |Z|
  9.85..27.85, trim each selection-wall axial tip to |Z|=12.85, and export one
  positive-volume 6061 carrier.
- Tower adapter: local plate X 26..38, Y -28..28, Z -114..-110 mm. Preserve
  the U-window X 25..30.5, Y -17.5..17.5, Z -114.5..-109.5 mm and the four
  primary M4 axes at the four local points `(29,-24.5)`, `(35,-17.5)`,
  `(29,+24.5)`, and `(35,+17.5)`. The same-side diagonal is 6 x 7 mm,
  preserving the 6 mm
  proof-basis X row span while separating the recessed heads.
- Primary mount hardware: exactly four NBK `SSHS-M4-10-SD-ALK` M4x10
  ultra-low small-head screws with factory nylon patches, zero washers, and
  four short M4 heat-set inserts. The four screws are recessed; no duplicate
  follower-local M4 set is present.
- Moving occurrences: exactly four. Each contains one radial slide, one
  monolithic 7075 tangential-slide/outer-yoke cartridge, one inner yoke, one
  PEEK nose, and a complete two-pivot hardware stack. Each occurrence has 15
  manufactured leaves: four custom bodies; one MISUMI `SCCG5-10` outer pin,
  two internal DIN 988 shims, two included NETWS4 retaining rings; and six
  inner-pivot leaves. There is no inward OD5 shoulder screw or nyloc. Front/
  rear and left/right are handed by reflection, not represented as four
  copies of the rejected additive carrier.
- Identity order: 0 front-left `(+axial,-tangential)`; 1 front-right
  `(+,+)`; 2 rear-right `(-,+)`; 3 rear-left `(-,-)`.
- State API: cover three M1 laws, four M2 tracks, and the M0 gate states
  `ENGAGED_LOCKED`, `FORCED_RETRACTION_RAMP`, and
  `ALL_RETRACTED_DISCONNECTED`. M1/M2 choose identities only; every physical
  occurrence remains M0-carriage-owned.
- Coarse selection: active centre 2.05 mm, parked centre 10.95 mm, hence an
  explicit 8.90 mm tangential travel. Export one positive-volume linkage
  envelope per occurrence with a `BLOCKER_ONLY` label. This is not a solved
  linkage and is not the passive +/-0.5 mm tangential compliance.
- Clearance target: centered selected/parked yoke bodies and complete
  SCCG5-10 outer-pivot envelopes retain 3.00 mm at the 2.05/10.95 mm centers;
  the inward passive q=-0.50 mm extreme and every parked downstream-to-carrier
  interface target 2.50 mm, a nominal 0.50 mm reserve above the 2.00 mm
  requirement. The local carrier relief leaves a 2.80 mm outboard radial
  dogleg web. These are nominal geometry facts only; formal tolerance-stack
  and load authority remain open.
- Reference STEP pose: all four occurrences parked and radially retracted,
  corresponding to `ALL_RETRACTED_DISCONNECTED`.
- Exact exported leaf-solid counts: one carrier + four occurrences x 15 leaves
  + eight primary-mount leaves = 69 manufactured leaves; four coarse-linkage
  blocker envelopes bring the review tree to 73 positive-volume leaf solids.
- Forbidden prototype content: old PEEK guide occurrences, old guide M3
  hardware or holes, follower `mounting_backer_context`, duplicate primary M4
  stacks, the old additive follower carrier/spine, and integrated machine
  context.
- Paths: source
  `cad/aggregate_boundary_follower_replacement_carriage.py`; tests
  `cad/test_aggregate_boundary_follower_replacement_carriage.py`; generated
  STEP and manifest under `out/review/`.
- Validation targets: source tests for the 36 state combinations and mapping;
  one-solid carrier; preserved U-window/keys and diagonal M4 axes; restored M3 locations; exact
  occurrence/hardware/blocker counts and unique labels; baseline STEP refs,
  facts, planes, and positioning; reviewed multi-view snapshots.
- Authority: review only. Assembly integration, production collision, wire
  route, load, buildability, procurement, BOM, and release authority all stay
  false. The 8.90 mm actuation linkage, positive-M0 retraction linkage,
  SCCG5-10 retention load/wear qualification, nominal 2 mm tolerance stack,
  active/parked transition sweeps, occurrence route clearances, and 5.52 N*m
  mount proof remain blockers.
