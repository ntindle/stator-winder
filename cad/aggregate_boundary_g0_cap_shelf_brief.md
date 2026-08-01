# Isolated robust g=0 cap-shelf CAD brief

## Purpose

Create a non-selected front/rear PEEK cap prototype that replaces the
22.496-micrometre seam witness with a manufacturable integral shelf. This
prototype is not imported by the selected cap, assembly, player, BOM, or
release sources.

## Geometry contract

- Stator-local millimetres: +Z stator axis; tooth 0 radial +X.
- Front and rear caps remain separate single solids.
- Twenty-four right-seam shelves per cap; 48 total.
- Integral natural-unfilled-PEEK shelf: 1.50 mm cap-side radial length,
  0.75 mm axial width, and 0.30 mm stock behind the contact face.
- Contact face: active-local Y=0.768671036571 mm, normal +Y.
- Diameter rebound endpoints:
  - d=0.2 mm: `(12.687039228441, 0.868671036571, +/-13.961655295982)`.
  - d=0.5 mm: `(12.687039228441, 1.018671036571, +/-13.961655295982)`.
- Re-centred right mouth: 2.40 mm radial x 1.00 mm tangential x 0.90 mm
  axial, centered at active-local Y=0.943671036571 mm.
- Complete permanent-cap lane negative: at least 0.65 mm clear and 0.25 mm
  floor-side radius envelope.
- Explicit 1.50 mm cap-side rebound corridor negatives for both the 0.2 and
  0.5 mm tangent wire envelopes; the shelf is fused after these cuts.
- R0.36 insertion gauges at both diameter endpoints.

## Required checks

- One solid per finished cap.
- Endpoint-to-cap distance equals d/2 for d=0.2 and 0.5 mm, front and rear.
- Zero positive cap/wire and cap/R0.36-gauge overlap.
- Twenty-fourfold front/rear symmetry and explicit mass facts.
- Labeled STEP compound and SHA-256-bound manifest.

## Fail-closed boundary

This prototype proves review geometry only. It does not prove force-resultant
compression, complete 2,400-locus routing, swept rigid/copper collision,
extraction, tolerance, cap balance/inertia release, retention loads, polish,
enamel safety, wear, endurance, assembly integration, or production.

Output: `out/review/aggregate_boundary_g0_cap_shelf.step`.
