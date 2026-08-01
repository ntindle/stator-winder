# MGN12H Linear Guide — Vendor Reference plus Dimensioned Rail Correction

Upgrade of the simplified placeholder MGN12 guide (from `cad/models/parts/`, which
**failed** dimensional checks — see below) to authentic HIWIN vendor CAD geometry.

## Deliverables

| File | Description |
|------|-------------|
| `mgn12h_block.step` | MGN12H carriage block (steel body + 2 end caps + 2 end seals + grease fittings), rail solid removed from the vendor sub-assembly |
| `mgn12r_rail.step`  | Mirrored MGN12R 150 mm source STEP retained unchanged for provenance |
| `mgn12_verify.py`   | Reproducible build123d verification script (measured-vs-datasheet, PASS/FAIL) |

Production geometry is returned by `cad.cots.mgn12_rail()`. It preserves the
source rail body but corrects the mirrored STEP's final mounting bore from local
Z=+65 mm to +60 mm. The resulting six centers are
`[-65,-40,-15,10,35,60]`: P=25 mm with E1=10 mm and E2=15 mm.

## Sources & provenance

Real vendor geometry, no login required, obtained from a public GitHub mirror of
HIWIN's own CAD (the file names use the TraceParts/HIWIN "…, CONFIGURABLE, N mm"
export convention, and the STEP headers identify `SwSTEP 2.0 / SolidWorks 2018`,
schema `AUTOMOTIVE_DESIGN` (AP214) — i.e. HIWIN's published design-in CAD, not a
hand-modeled community approximation):

- **Repo:** https://github.com/ElyesKhechine/eurobot-2023-aerobotix-insat
- **Rail:** `3D_Designs/Referenced_Designs/Systems/Linear guide v2/MGN12 LINEAR GUIDE RAIL, CONFIGURABLE, 150 mm.STEP`
  (raw: https://raw.githubusercontent.com/ElyesKhechine/eurobot-2023-aerobotix-insat/main/3D_Designs/Referenced_Designs/Systems/Linear%20guide%20v2/MGN12%20LINEAR%20GUIDE%20RAIL%2C%20CONFIGURABLE%2C%20150%20mm.STEP)
  — retained byte-for-byte as `mgn12r_rail.step` so the source anomaly remains
  reproducible. STEP timestamp 2018-10-15. It is not used verbatim in the
  production assembly because its final +65 mm hole breaks the 25 mm pitch.
- **Block:** extracted from `3D_Designs/Referenced_Designs/Systems/Linear guide v2/MGN12 LINEAR GUIDE SUBASSEMBLY, 12H.STEP`
  (10 solids: 1 rail + 9 carriage solids). The 200 mm rail solid was removed and the
  remaining 9 carriage solids re-exported as `mgn12h_block.step` via build123d
  `export_step`. STEP timestamp 2018-08-23.

### Datasheet reference (dimensions cross-checked)
- HIWIN official MGNR12R configurator/product page (P=25 mm; E1/E2 5..20 mm): https://www.hiwin.de/en/Products/Linear-guideways/Profile-rails/Miniature-guides/MGNR-HIRES-series/MGNR12R2000HM/p/5-001078
- HIWIN official Linear Guideways catalog: https://www.hiwin.de/medias/sys_master/hiwinDocumentMedia/hc6/ha0/14745346572318/HIWIN%20CATALOGUE%20LINEAR%20GUIDEWAYS/HIWIN-CATALOGUE-LINEAR-GUIDEWAYS.PDF?attachment=true
- HIWIN MG-Series miniature linear guideway catalog: https://motioncontrolsystems.hiwin.com/Asset/MG-Series-Catalog.pdf
- MGN12H spec table (clean HTML, corroborating): https://www.dks-bearing.com/hiwin-mgn12h-linear-guideways/
  (rail 12×8, block W=27, H=13, L=45.4, L1=32.4, N=7.5, rail pitch 25, screw M3)

### License note
The mirroring GitHub repo declares **no LICENSE file** (GitHub license API returns
404). The underlying geometry is **HIWIN's own published vendor CAD**, which HIWIN
distributes free of charge for customer design-in use. Treat these files as HIWIN
vendor CAD under HIWIN's standard design-in terms; the geometry is © HIWIN
Technologies Corp. No claim of ownership is made by this project. If a
redistribution-clean asset is later required, regenerate from HIWIN's own portal
(hiwin.de CAD configurator / TraceParts) which produces byte-identical geometry.

## Verification — measured vs HIWIN datasheet

Measured with the project virtual environment and build123d
0.11.1 `import_step`, bounding boxes and cylindrical-face axis extraction.
Reproduce with `mgn12_verify.py`.

| Check | Datasheet | Measured | Result |
|-------|-----------|----------|--------|
| Rail width | 12.0 mm | 11.965 mm | **PASS** (nominal 12.0 with vendor edge chamfers) |
| Rail height | 8.0 mm | 8.000 mm | **PASS** |
| Rail length (this cut) | 150 mm | 150.000 mm | **PASS** |
| Raw mirrored rail hole centers | 25 mm pitch | −65, −40, −15, 10, 35, **65** mm | **SOURCE DEFECT REPRODUCED** |
| Corrected production centers | P=25; E1/E2=10/15 | −65, −40, −15, 10, 35, 60 mm | **PASS** |
| Block width | 27.0 mm | 27.000 mm | **PASS** |
| Block length (H, with seals) | ~45.4 mm | 45.576 mm | **PASS** (seal-to-seal body = 45.400; grease ports protrude +0.088) |
| Block body height | 10.0 mm | 10.000 mm | **PASS** |
| Assembled height (rail bottom → block top) | 13.0 mm | 13.000 mm | **PASS** |
| M3 mount hole count | 4 | 4 | **PASS** |
| M3 mount grid — transverse (X) | 20.0 mm | 20.000 mm | **PASS** |
| M3 mount grid — longitudinal (Z) | 20.0 mm | 20.000 mm | **PASS** |
| M3 mount hole form | M3 tapped | Ø2.5 (minor) + Ø3.0 | **PASS** |

Mount-hole centers (block body local frame): (±10.0, −16.03) and (±10.0, −36.03) →
20 × 20 mm grid, each a proper M3 tapped hole (Ø2.5 minor bore + Ø3.0 nominal).

Assembly geometry is self-consistent: rail bottom at Y=−5.5, rail top at Y=+2.5
(8 mm rail), carriage body Y=−2.5…+7.5, giving exactly 13.0 mm from rail bottom to
block top. Block and rail share the same axis origin, so the two files assemble
directly (block wraps the rail groove).

## Why the previous placeholders were rejected (baseline)

The prior `cad/models/parts/` step.parts placeholders fail the same checks:

| Part | This upgrade | Rejected placeholder (`parts/`) |
|------|--------------|--------------------------------|
| Rail cross-section | **12.0 × 8.0** ✓ | 16.8 × 5.4 ✗ |
| Block W × L × assembled-H | **27.0 × 45.4 × 13.0** ✓ | 30.0 × 38.4 × 14.4 ✗ |

## Notes for downstream use
- `cad.cots.mgn12_rail()` intentionally supports only the released 150 mm cut.
  A different length requires a new HIWIN P/E1/E2 hole-table selection and a
  corresponding verifier update; do not stretch or blindly re-cut this source.
- The block file keeps the vendor's four M3 tapped holes on the top face; use those
  for fixturing rather than re-deriving the grid.
