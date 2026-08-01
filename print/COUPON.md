# PETG fit and bridge qualification coupon

Production printing is blocked until this coupon is printed with the frozen
`winder-a1-0p4-petg-strength-v1` profile and the physical checks below are
recorded as passed. The coupon is a qualification tool, not a machine part.

## Print setup

- Printer: Bambu Lab A1, 0.4 mm nozzle, Textured PEI Plate.
- Material: dry INLAND PETG Plus from the same spool intended for production.
- Process: 0.20 mm layers, 6 walls, 5 top layers, 3 bottom layers, 25% cubic
  infill, no supports.
- Use `out/print/coupon/fit_bridge_coupon.gcode`. It was sliced locally with
  OrcaSlicer 2.4.2 and has not been sent to a printer.

Regenerate the local artifact from the project-owned sources with:

```powershell
.\.venv\Scripts\python.exe print\slice_coupon.py
```

That command only invokes the local OrcaSlicer CLI; it contains no printer
discovery, upload, or start path.

## Coupon map

With the four bearing rings nearest you, read left to right:

1. 6001 bearing seat, diameter 28.1 mm.
2. 608 bearing seat, diameter 22.1 mm.
3. 688 bearing seat, diameter 16.1 mm.
4. 623 bearing seat, diameter 10.1 mm.
5. Flyer-pulley tube bore, diameter 12.05 mm.
6. Wire-elbow sleeve male gauge, diameter 8.96 mm.

The rear row has the three diameter 4.0 mm heat-set pilots. One, two, and three
tally tabs identify McMaster 94459A769 at 3.4 mm depth, 94459A130 at 4.3 mm,
and 94459A140 at 5.7 mm. The other rear feature is the exact 44 mm unsupported
spindle-pocket bridge.

## Physical acceptance record

Record actual observations; do not infer a pass from the CAD or G-code.

- Each real bearing starts square, seats with light controlled pressure without
  cracking, has no perceptible radial rock, and can be pushed back out.
- The real 12 mm OD flyer tube enters the diameter 12.05 pulley gauge without
  splitting it.
- The diameter 8.96 elbow gauge enters the real 9 mm-ID tube mouth without
  shaving material or forcing it.
- One insert of each listed SKU installs flush at its keyed depth without boss
  cracking, spin, or pullout.
- The 44 mm bridge has a continuous underside with no dropped strands. Measured
  midspan sag relative to the two ends is at most 0.50 mm.
- Record operator, date, printer, filament spool identity/dry state, measured
  bridge sag, each fit result, and any dimensional compensation applied.

Only after every item passes should `print_plan.qualification.status` become
`passed`, `production_release_allowed` become `true`, and the signed physical
record be added to the catalog.

## Production wire elbow

The released printed elbow material is PETG. Deburr and polish the complete
wire bore before use, then perform an enamel-abrasion pull test: run a sacrificial
length of the production wire through 100 reciprocating strokes under about
10 N tension and inspect under magnification. Any visible scratching, enamel
dust, or exposed copper is a failure. This test qualifies the finished elbow;
the coupon alone does not prove PETG wear life.

PTFE remains an unqualified alternative. It requires a separate machining,
retention, fit, and abrasion-validation record before substitution.
