# Aggregate-boundary floating follower CAD brief

## Intent

Create an isolated, positive-volume prototype of the passive follower required
between the selected guide endpoint and the exposed winding aggregate.  The
prototype must own the otherwise-discontinuous bend with a polished R3.0 or
larger surface while retaining the existing M0/M1/M2 axis architecture.

## Source frame and inputs

- Units: millimetres.
- Active-tooth local frame: +X radial/outward, +Y tangential, +Z stator axis.
- Exact selected aggregate: equal-area triangular sublevel from
  `sim/aggregate_boundary_follower_locus_study.py`.
- Required usable travel: 6.0 radial and 1.0 tangential.
- Primary mount datum: existing keyed tower face at machine Y=-114, keys at
  machine X=+-10/Z=61, and four M4 holes at X=+-21/Z=60,66.
- Wire range: 0.2-0.5 diameter.

## Manufacture and hardware

- Carrier/yokes: 6061-T6.
- Sliding members: hard-anodized 7075-T6; the tangential slide and outer
  gimbal yoke are one machined cartridge with a minimum 5x4x1 mm positive
  throat and R0.75 minimum/R1 preferred roots. Aluminum-on-aluminum bearing contact
  is not released without a selected bushing or liner.
- Contact nose: virgin unfilled PEEK, polished R3.0 groove floor, Ra <=0.4 um.
- Primary tower hardware: 4x ISO 4762 M4x10, 4x ISO 7089 M4 washers, and 4x
  McMaster 94459A150 short M4 heat-set inserts.
- Outer pivot candidate: McMaster 96654A127, OD5x10 shoulder, M4x5 thread.
- Inner pivot: McMaster 90265A115, OD3x10 shoulder, M2x4 thread, with four
  DIN 988 3x6x0.5 shims (two external plus two nose thrust shims) and an M2
  nyloc. The earlier OD3x16 90265A420 is too
  long for this yoke stack and is rejected here. McMaster 90265A181 is also
  rejected because its shoulder is OD4, despite an ambiguous search snippet.
- Radial preload candidate: LEM050AB01 through a 0.29 bellcrank ratio.
- Tangential return spring, tangential bushing/guide, and any ceramic nose
  alternative remain unselected.

## Acceptance contract

The isolated source and STEP may advance only when all separately manufactured
bodies are single solids, moving pairs have no unintended positive-volume
overlap in every modeled endpoint state, the R3 axis is +Z, both slide endpoints
remain captured by hard stops, and every shown structural fastener has its
washer and retained thread owner.

The primary structural screen uses 40 N total until the actual maximum wire
wrap is proven <=90 degrees.  Four-fastener equal sharing is only a preliminary
10 N/fastener load case; eccentric moment, carrier mass/inertia, and final
fastener reactions still require calculation.

## Fail-closed boundaries

This prototype does not authorize assembly integration, a BOM release, an RFQ,
wire-route validity, collision clearance, dynamic dancer behavior, production,
or ordering.  Advancement additionally requires:

- a physical cap/liner normal for all 48 empty-slot starts;
- constructive R3 contact for all 2,400 loci and both wire diameters;
- a continuous intra-half-turn follower law and adaptive swept collision test;
- downstream wire-length/dancer equilibrium and transient dynamics;
- selected tangential spring and bearing interface;
- positive retraction hardware attached to the actual M0 mechanism;
- eccentric load, wear, wire coupon, and 300 rpm endurance qualification.

The explicit STEP target is
`out/review/aggregate_boundary_floating_follower.step`.
