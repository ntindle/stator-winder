# Small COTS Parts — STEP Model Upgrade Report

Upgrades two simplified placeholders to real STEP geometry for the winding-machine project.
Both parts were obtained as **genuine downloaded STEP files from no-login public sources** —
no parametric fabrication was required, and no manufacturer substitution was needed.

| Deliverable | Part | Result |
|---|---|---|
| `endstop.step` | KW12-3 roller-lever microswitch | Real KW12-3 STEP, all checks PASS |
| `gt2_40t_b8.step` | GT2 40T / 6 mm belt / 8 mm bore pulley | Real GT2-40T STEP, all checks PASS |

Verification is reproducible: `../../../.venv/Scripts/python verify.py` (build123d 0.11.1, Python 3.11.9).

---

## 1. Sources & Licenses

### endstop.step — KW12-3 microswitch
- **Repo:** `github.com/tandude0/GrabCAD` (branch `main`)
- **Path in repo:** `Robotic Arm Project/IndustrialStyledVersion/V1/References/Fim de Curso - KW12-3.stp`
  ("Fim de Curso" = Portuguese for *limit switch*.)
- **Direct raw URL (verified HTTP 200, no auth):**
  `https://raw.githubusercontent.com/tandude0/GrabCAD/main/Robotic%20Arm%20Project/IndustrialStyledVersion/V1/References/Fim%20de%20Curso%20-%20KW12-3.stp`
- **Blob SHA-1:** `0fd1ef08aaaedd83a10c9e9a8b36e8231a369317` · **Size:** 355,451 bytes · AP242 ISO-10303-21.
- **Provenance:** STEP header shows a Fusion 360 → STEP conversion job; the model originated as a
  free GrabCAD community upload and was re-hosted (unmodified) in this public repo.
- **License:** No `LICENSE` file / no SPDX declared in the host repo. Treat as a community-shared
  reference model (original distributed free via GrabCAD's community library). Fine for internal
  engineering/reference use; confirm terms before any redistribution.

### gt2_40t_b8.step — GT2 40-tooth pulley
- **Repo:** `github.com/ThogDuy/One-Wheel-Balancing-Robot` (branch `master`)
- **Path in repo:** `Mô hình/Solidwork/GT2 Timing Pulley T40.stp`
- **Direct raw URL (verified HTTP 200, no auth):**
  `https://raw.githubusercontent.com/ThogDuy/One-Wheel-Balancing-Robot/master/M%C3%B4%20h%C3%ACnh/Solidwork/GT2%20Timing%20Pulley%20T40.stp`
- **Blob SHA-1:** `e9ebe59ddcd3715815a0a85968628197fb514f68` · **Size:** 241,119 bytes · AP242 ISO-10303-21.
- **Provenance:** STEP header `FILE_NAME` = `...\GrabCAD\GT2 Timing Pulley T40.stp` (authored
  "Niels Hermans") — a free GrabCAD community pulley, re-hosted unmodified in this public repo.
- **License:** No `LICENSE` file / no SPDX declared. Same community-model caveat as above.

Both raw files were re-verified fetchable with **unauthenticated `curl`** (HTTP 200, exact byte
counts), so they qualify as legitimate no-login sources. GrabCAD, TraceParts, SnapEDA and the Omron
partner CAD portals were all rejected because they gate downloads behind a login.

---

## 2. Verification — measured vs expected

Measured with build123d `import_step` + geometry analysis (bounding boxes of the largest solid,
cylindrical-face radii/areas, and vertex radius/angle clustering).

### endstop.step (KW12-3)
| Feature | Measured | Expected (KW12-3) | Status |
|---|---|---|---|
| Body length | 19.90 mm | ~20 | PASS |
| Body height | 10.40 mm | ~10 | PASS |
| Body thickness | 6.02 mm | ~6.5 (vendor 5.8–6.5) | PASS |
| Mounting-hole dia | 2.40 mm | ~2.4–3.0 (M2–M2.5) | PASS |
| Mounting-hole count | 2 | 2 | PASS |
| Mounting-hole spacing | 9.90 mm | ~9.5 (9.5–10) | PASS |
| Overall bbox incl. lever+roller | 19.9 × 6.0 × 28.3 mm | lever/roller included | PASS |

Notes: The model is a full roller-lever KW12-3 (8 solids: body, cover, roller lever, roller, pin,
3 terminals). Body thickness 6.02 mm and hole pitch 9.90 mm sit within the real KW12-3 vendor
spread (KW12-3 bodies run 5.8–6.5 mm thick; hole pitch is nominally 9.5–10 mm). No datasheet
substitution was needed, so the Omron SS-5GL stand-in was **not** used.

### gt2_40t_b8.step (GT2 40T, 6 mm belt, 8 mm bore)
| Feature | Measured | Expected | Status |
|---|---|---|---|
| Bore diameter | 8.00 mm | 8.0 (NEMA23 shaft) | PASS |
| Tooth-tip OD | 25.00 mm | ~25 (40T GT2: PD 25.46, tip ≈24.95) | PASS |
| Flange OD | 28.00 mm | 28–31 | PASS |
| Overall width (w/ hub) | 17.00 mm | ~16.5 | PASS |
| Tooth count | 40 | 40 | PASS |
| Belt channel (between flanges) | 7.00 mm | ~7 (fits 6 mm belt) | PASS |
| Hub OD | 16.00 mm | — (typ. 16–18) | info |

Notes: Canonical GT2 profile — 2 mm pitch, tip OD 25.0 mm, 40 teeth confirmed by 40 angular
tip-vertex clusters (and 120 = 40×3 tooth fillet faces). Flange thickness 1.5 mm each; 7.0 mm belt
channel is the standard groove for a 6 mm GT2 belt. Overall width 17.0 mm vs the ~16.5 mm target
is +0.5 mm (hub length), well within a drop-in match.

**Variant rejected:** the same repo's `40Tooth GT2 Pulley.stp` was also downloaded and measured but
discarded — distorted tooth profile (tip OD ~26.5 mm, irregular vertex radii) and a wider 20.9 mm
body. `GT2 Timing Pulley T40.stp` is the clean, spec-accurate one and is what shipped.

---

## 3. Reproduce

```
cd cad/models/upgrades
../../../.venv/Scripts/python verify.py      # prints both tables above; all PASS
```

Re-download from source (no auth needed):
```
curl -L -o endstop.step     "https://raw.githubusercontent.com/tandude0/GrabCAD/main/Robotic%20Arm%20Project/IndustrialStyledVersion/V1/References/Fim%20de%20Curso%20-%20KW12-3.stp"
curl -L -o gt2_40t_b8.step  "https://raw.githubusercontent.com/ThogDuy/One-Wheel-Balancing-Robot/master/M%C3%B4%20h%C3%ACnh/Solidwork/GT2%20Timing%20Pulley%20T40.stp"
```
