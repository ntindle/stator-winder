# Closed-Loop Stepper Motor STEP Upgrades — Verification Report

## Current M0/M1 selection — 2026-07-10

The orderable replacement is the StepperOnline `1-CL42T-S05-V41` kit,
quantity two.  Each kit contains one `17HS19-2004D-E1K` closed-loop NEMA17,
one `CL42T-V41` driver, and the programming cable.  The manufacturer lists
52 N·cm holding torque, 42 mm frame, 68 mm body, Ø5 × 24 mm shaft,
2.0 A/phase, and a 1000 PPR differential optical encoder.

- **Product:**
  https://www.omc-stepperonline.com/1-axis-closed-loop-stepper-cnc-kit-52ncm-73-64oz-in-nema-17-motor-driver-1-cl42t-s05-v41
- **Reference STEP:** `17HS19-2004D-E1K.step` (AP214, 1,507,253 bytes)
- **STEP SHA-256:**
  `2cf71823dc9b09f255397e6b8a3e771d043945e8a131cce5f62a2bf06f788bad`
- **Torque curve:** `17HS19-2004D-E1K_Torque_Curve.pdf` (24 V, 2.0 A,
  2000 microstep)
- **Curve SHA-256:**
  `b5c736bb8116f51052f16952e55bdac9695977b6c1df2e329947d4f599462cbb`

The raw STEP contains 24 solids and measures 42.3 × 92.516 × 95.344 mm
because it includes the loose 500 mm cable and connector.  The rigid installed
motor is five solids: four body/end-cap solids plus the central shaft.  In the
vendor frame it measures X −21.15..+21.15, Y −68..+24, and
Z −21.15..+21.15 mm, with the mounting face at Y=0.  The collision model
rotates this interface into the project motor frame and uses a conservative
single-solid 42.3 mm square, 68 mm rear body, Ø22×2 boss, and Ø5×24 shaft.
The flexible lead is a cable-routing obligation, not a frozen rigid solid.

The official pull-out curve is substantially below holding torque: about
35 N·cm through 100 rpm and 33 N·cm near 200–300 rpm.  `cad/loads.py` uses a
lower-edge digitization of that curve, never the 52 N·cm holding value, and
the M0/M1 load margins must be regenerated whenever their speed limits or
mechanics change.

## McMaster M2 correction — 2026-07-10

Normal `GOAL.md` requires M0, M1, **and M2** to use closed-loop NEMA17
motors.  The former NEMA23 selection was therefore removed even though it had
adequate torque.

- **Selected M2:** McMaster-Carr `6627T421`, NEMA17 motor with encoder.
- **Catalog facts:** 124.6 in-oz (0.880 N·m) holding torque, 1400 rpm maximum,
  2 A/phase, 1000 counts/rev incremental encoder, Ø5 × 22 mm shaft.
- **Files supplied from the product page:** `6627T421.step` and
  `6627T421_torque_curve.png`.
- **STEP SHA-256:**
  `2600b11154de8a113f20e392b5d31d3e99f23858f7cb2c69cedd20329b63a7ba`.
- **Measured raw STEP bounds:** 42.418 × 57.437 × 100.022 mm including the
  22 mm front shaft, rear connector housing, and a loose coiled shipping
  cable.  The mounting face is raw Z = +28.0126 mm; the motor/encoder rear is
  Z = -50.0096 mm, so the installed body depth is 78.0222 mm.
- **Installed collision representation:** all motor/encoder/connector solids
  above 500 mm³ are retained from the source STEP.  Only the loose shipping
  cable and tiny connector-pin detail are omitted; the asymmetric rear
  connector envelope remains.  This produces verified local bounds
  X -21.209..+21.209, Y -36.228..+21.209, Z -78.022..+22.000 mm about the
  mounting face.
- **Torque evidence:** the McMaster 24 V, 2 A, half-step curve is saved in the
  repository and conservatively digitized in `cad/loads.py`; it gives roughly
  0.69 N·m at the software's 191 rpm limit and 0.63 N·m at the 300 rpm design
  point.

The McMaster file remains manufacturer reference geometry for design/fit use;
do not redistribute it independently of this project.

## Legacy E1000 reference files

The following E1000 files are retained only as provenance for the earlier
design state.  They are not the M0/M1 purchasing authority.

Real vendor STEP geometry for the two closed-loop (encoder) stepper motors used on
the earlier winding-machine design, replacing the simplified placeholder envelopes.

- **Download date:** 2026-07-08
- **Vendor:** StepperOnline (OMC / omc-stepperonline.com)
- **Tooling:** downloaded with `curl` (browser UA + product-page referer; the site's
  Cloudflare edge returns 403 to plain `WebFetch`/no-referer requests). Verified with
  `build123d 0.11.1` `import_step` + bounding-box / cross-section slicing on the project
  venv (`.venv/Scripts/python`).

## Files obtained

| Deliverable | Vendor part | STEP schema | Size |
|---|---|---|---|
| `17HS19-2004D-E1000.step` | 17HS19-2004D-E1000 (NEMA17 closed-loop, 1000PPR encoder) | AP203 (SolidWorks 2016) | 1.64 MB |
| `23HS22-4004D-E1000.step` | 23HS22-4004D-E1000 (NEMA23 closed-loop, 1000PPR encoder) | AP214 (SolidWorks 2016) | 9.20 MB |

### Part-number substitutions (read this)

The two exact part numbers in the task brief resolve to the closest **published**
E1000-series parts that actually host a STEP file — the substitutions are dimensionally
identical to the requested parts (same frame body, same E1000 optical encoder cap, same
shaft); only the winding/current spec differs.

- **NEMA17 — requested `17HS19-2004-E1000` → obtained `17HS19-2004D-E1000`.**
  StepperOnline's closed-loop 17HS19 with the E1000 encoder *is* the `-2004D-E1000`
  (the "D" = dual/rear shaft carrying the encoder; 59 Ncm, 2.0 A). This is the correct
  motor, not a substitute. Note: the product page is flagged "temporarily discontinued"
  (current buyable equivalent is `-E1K`, 52 Ncm), but the page and STEP are live and the
  mechanical envelope is unchanged.
- **NEMA23 — requested `23HS22-2804S-E1000` → obtained `23HS22-4004D-E1000`.**
  No `23HS22-2804S-E1000` product/STEP exists on the vendor site. The closest published
  E1000 NEMA23 on the **same 23HS22 frame** (56 mm body, 1.2 Nm, Ø8 shaft, same E1000
  encoder) is the S-series `23HS22-4004D-E1000` (4.0 A instead of 2.8 A). The physical
  envelope — body length, encoder cap, flange, shaft — is identical to what a
  `2804S-E1000` would be; only the coil current/resistance differs.

## Source URLs

**NEMA17 17HS19-2004D-E1000**
- Product page: https://www.omc-stepperonline.com/nema-17-closed-loop-stepper-motor-59ncm-84oz-in-w-encoder-1000ppr-4000cpr-17hs19-2004d-e1000
- STEP: `https://www.omc-stepperonline.com/index.php?route=product/product/get_file&file=794/17HS19-2004D-E1000.STEP`
- Datasheet: `https://www.omc-stepperonline.com/index.php?route=product/product/get_file&file=794/17HS19-2004D-E1000_Full_Datasheet.pdf`

**NEMA23 23HS22-4004D-E1000**
- Product page: https://www.omc-stepperonline.com/s-series-nema-23-closed-loop-stepper-motor-1-2-nm-170oz-in-encoder-1000ppr-4000cpr-23hs22-4004d-e1000
- STEP: `https://www.omc-stepperonline.com/index.php?route=product/product/get_file&file=1446/23HS22-4004D-E1000.STEP`
- Datasheet: `https://www.omc-stepperonline.com/index.php?route=product/product/get_file&file=1446/23HS22-4004D-E1000_Full_Datasheet.pdf`

## License / terms

Manufacturer-provided reference CAD, offered as a free public download link on each
product page (no login, no click-through EULA). StepperOnline publishes no explicit open
license for these files; the datasheet title block carries the standard "shall not be
reproduced … unless expressly authorized in writing by STEPPERONLINE" confidentiality
notice. Treat these as **vendor reference geometry for internal design/fit use** — fine
for our CAD assembly and clearance checks; do not redistribute as a standalone asset.

## Measured (build123d) vs datasheet

Motor axis = model **Z**. Each STEP includes the flexible cable + 15-pin DB connector,
which droop off-axis (−X/−Y) and were cropped out with a central column before measuring
the motor block. Cross-sections were slabbed along Z at 0.25–1 mm to separate
faceplate / body+encoder / shaft.

### NEMA17 — 17HS19-2004D-E1000

| Feature | Measured | Datasheet | Match |
|---|---|---|---|
| Faceplate (square) | 42.30 × 42.30 mm | 42.3 mm ("42 MAX") | exact |
| Shaft diameter | Ø5.00 mm (4.5 mm across flat) | Ø5, flat 4.5±0.1 | exact |
| Shaft protrusion (flange→tip) | 24.0 mm | 24±1 | exact |
| Pilot register boss | Ø22 | Ø22 | exact |
| **Body + encoder (flange→encoder rear)** | **71.0 mm** | 48 body + 24 encoder = 72 | −1.0 mm |
| Overall Z (shaft tip → encoder rear) | 95.0 mm | 24 + 48 + 24 = 96 | −1.0 mm |
| Raw bbox incl. cable/connector | 45.1 × 108.3 × 104.0 mm | (cable is reference only) | n/a |

### NEMA23 — 23HS22-4004D-E1000

| Feature | Measured | Datasheet | Match |
|---|---|---|---|
| Faceplate / body (square) | 57.30 × 57.30 mm | flange □56.4 MAX / 57 body | +0.9 vs flange, on-spec vs body |
| Shaft diameter | Ø8.00 mm (7.5 mm across flat) | Ø8, flat 7.5±0.1 | exact |
| Shaft protrusion (flange→tip) | 21.0 mm | 21±1 | exact |
| Pilot register boss | Ø38.1 | Ø38.1 | exact |
| **Body + encoder (flange→encoder rear)** | **81.0 mm** | 56 body + 25 encoder = 81 | exact |
| Overall Z (shaft tip → encoder rear) | 102.0 mm | 21 + 56 + 25 = 102 | exact |
| Raw bbox incl. cable/connector | 58.3 × 160.8 × 193.8 mm | (cable is reference only) | n/a |

> Note on NEMA23 flange width: bbox across-flats reads 57.3 mm (the lamination body,
> matching the "57×57" body spec). The mounting flange plate itself is chamfered to
> □56.4 MAX per datasheet; a min-area bbox would report the flange nearer 56.4. Either
> way it is inside the 56.4–57 mm expected range.

## Total length including encoder (headline numbers)

- **NEMA17 17HS19-2004D-E1000:** body **48 mm** + encoder cap **≈23 mm** = **71.0 mm**
  behind the mounting flange (95.0 mm overall including the 24 mm front shaft).
- **NEMA23 23HS22-4004D-E1000:** body **56 mm** + encoder cap **25 mm** = **81.0 mm**
  behind the mounting flange (102.0 mm overall including the 21 mm front shaft).

## Impact note — M1 NEMA17 flange (y=−184) vs frame members (y=−225…−245)

The M1 NEMA17 hangs with its flange face at **y = −184**, body/encoder extending in −y
toward the frame. The current placeholder is a **40 mm generic body** (no encoder): its
rear lands at y = −184 − 40 = **−224**, giving a bare ~1 mm clearance to the near frame
face at y = −225. The **real 17HS19-2004D-E1000 measures 71.0 mm** from flange face to
encoder rear, so its rear reaches y = −184 − 71 = **−255 mm** — that is **30 mm past** the
near frame face (−225) and **10 mm past** the far face (−245), i.e. the steel motor body
and its encoder cap pass **completely through** the y=−225…−245 frame members. The
placeholder hid a hard collision. To seat the real closed-loop motor, the M1 mount must
move the flange forward (toward +y) by **≥ 31 mm** (encoder rear ≤ −224), or the frame
pocket must be opened up / relieved, or a shorter closed-loop NEMA17 selected. Note the
front shaft also now protrudes a full 24 mm ahead of the flange (vs the placeholder),
which the pulley/belt stack on the +y side must accommodate.
