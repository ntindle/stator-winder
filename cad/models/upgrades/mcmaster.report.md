# McMaster-Carr STEP upgrade for T8x8 lead screw + flange nut

Date: 2026-07-08
Scope: replace the simplified `t8_p2_*` CAD placeholders with real McMaster-Carr STEP
models for a **T8x8 lead screw (8 mm dia, 8 mm lead, 4-start)** and its matching flange nut.

---

## Bottom line (read this first)

1. **McMaster-Carr does not sell a T8x8 lead screw.** There is no screw in the McMaster
   catalog that is simultaneously **8 mm diameter AND 8 mm lead (travel/turn)**. This was
   verified directly against McMaster's live catalog filters (evidence below). The task's
   premise — "pick the correct 8 mm / 8 mm-lead variant from the fast-travel family around
   98935A" — cannot be satisfied because that variant does not exist at McMaster.
     - The part number in the task hint, **98935A**, is not a resolvable product page today
       (McMaster's item-presentation endpoint returns HTTP 500 for it).
2. **No STEP file could be downloaded.** Independent of the part-selection problem, McMaster's
   authenticated CAD download could not be persisted to disk in this browser-automation
   environment (details in "STEP download attempts" below). The direct CAD URL returns
   **403 Forbidden** to any un-cookied client (PowerShell and WebFetch both confirmed).
3. **No STEP files were fabricated.** Per instruction, nothing was invented. The requested
   `t8_screw_mcmaster.step` / `t8_nut_mcmaster.step` files are intentionally **absent** — a
   genuine McMaster T8x8 STEP does not exist to download.

**Recommendation:** keep the existing simplified `cad/models/parts/t8_p2_*` geometry (it is
dimensionally correct for an 8 mm screw), or source a true Tr8x8 STEP from a trapezoidal-screw
vendor / the `step.parts` catalog. See "Recommendation" at the end.

---

## What was requested vs. what McMaster stocks

| Attribute            | Requested (T8x8)         | Available at McMaster?                                   |
|----------------------|--------------------------|---------------------------------------------------------|
| Shaft/major diameter | 8 mm                     | Only with **1.5 / 2 mm lead** (single-start). No 8 mm lead. |
| Lead (travel/turn)   | 8 mm                     | Only on **4 mm / 5 mm shaft** screws (8-start ultra-precision). |
| Thread starts        | 4                        | McMaster's 8 mm-lead screws are **8-start**, not 4.     |
| Length               | ~200 mm                  | 8 mm-lead screws max out at **150 mm**.                 |
| Thread form          | metric rounded trapezoidal | yes (both families are "Metric Rounded Trapezoidal")   |

The two requirements (8 mm diameter, 8 mm lead) live in **two mutually exclusive** McMaster
product families. No single part bridges them.

---

## Catalog investigation (evidence)

McMaster metric lead screws: <https://www.mcmaster.com/products/lead-screws/>

Filter used to isolate every 8 mm-lead screw:
`/products/lead-screws/component~lead-screw/travel-distance-per-turn~8-mm/`
→ **Result: exactly one family, 4537N** (Metric Fast-Travel Ultra-Precision).
   Available shaft diameters in that result set: **4 mm and 5 mm only**; thread starts: **8**;
   thread size: **M4**.

Cross-checks:
- `thread-type~metric-rounded-trapezoidal/shaft-diameter~8-mm/` → **0 products** (filter resets).
- `thread-type~metric-trapezoidal/component~lead-screw/` → family **7549K**, all **single-start**,
  travel per turn 2/3/4/5/7 mm (no 8 mm lead, no 4-start).
- `thread-type~metric-trapezoidal/shaft-diameter~8-mm/` → **0 products**.

Conclusion: an 8 mm-diameter / 8 mm-lead / 4-start trapezoidal screw is **not in the catalog**.

---

## Closest McMaster parts (real part numbers + specs, from the McMaster datasheets)

### Option A — closest on LEAD (8 mm lead), wrong diameter
McMaster "Metric Fast-Travel Ultra-Precision Lead Screws and Nuts" (this is the "fast-travel"
family the task pointed at). **8 mm lead, but M4 / 4–5 mm diameter, 8-start.**

- **Screw 4537N151** — <https://www.mcmaster.com/4537N151/>
  17-4 PH stainless; thread **M4 × 1 mm**, **8 thread starts**, **8 mm travel/turn (lead)**,
  speed ratio 8:1; shaft diameter **5 mm**; one coupling end (Ø9.5 mm × 15.5 mm); length **150 mm**;
  fully threaded; ultra-precision.
- **Flange nut 4537N208** — <https://www.mcmaster.com/4537N208/>
  PEEK, flange nut with wear-compensating internal O-ring (anti-backlash); thread **M4 × 1 mm**,
  **8 starts**, 8 mm lead; body Ø **0.406″ = 10.31 mm**; length **31/64″ = 12.30 mm**;
  flange **3/4″ = 19.05 mm** wide × **13/32″ = 10.32 mm** high × **1/8″ = 3.18 mm** thick;
  ~$83.89 each. (Datasheet marketed for 3D-printing / robotics.)

  Note: the nut's *body* diameter (10.31 mm) happens to be close to the T8 hobby-nut body
  (~10.2 mm), but its **bore is M4 (~4 mm)**, so it will NOT fit an 8 mm screw.

### Option B — closest on DIAMETER (8 mm), wrong lead
McMaster "Ultra-Precision" metric trapezoidal, single-start. **8 mm diameter, but 1.5 mm lead.**

- **Screw 7549K55** — <https://www.mcmaster.com/7549K55/>
  4140 alloy steel; thread **M8 × 1.5 mm**, **single start**, **1.5 mm travel/turn**,
  Metric Rounded Trapezoidal; length **100 mm**; ~$23.68 each. (Longer lengths and an M8 × 2 mm
  variant exist in the same 7549K / related families.) A matching M8 nut is offered alongside.

Neither option is a functional drop-in for a T8x8.

---

## Verification — measured vs. expected (build123d 0.11.1)

The requested `*_mcmaster.step` files do not exist, so there was nothing new to import. Instead
the project's **existing simplified** models were measured to establish the real "expected"
geometry, and the McMaster closest-part figures are quoted from the McMaster datasheets.

`import_step` + `bounding_box()` on the existing placeholders:

| Feature                | Expected (T8x8 hobby target) | Existing simplified model (measured) | McMaster 4537N151/208 (datasheet) |
|------------------------|------------------------------|--------------------------------------|-----------------------------------|
| Screw major diameter   | ~8 mm                        | **8.000 mm** ✓ (plain Ø8 cylinder)   | ~4 mm (M4)  ✗                     |
| Screw lead / start     | 8 mm / 4-start               | n/a (smooth cyl, no thread)          | 8 mm / **8-start**  ~/✗           |
| Screw length           | ~200 mm                      | **200.000 mm** ✓                     | 150 mm  ✗                         |
| Nut flange diameter    | ~22 mm                       | **20.0 mm** (~)                      | 19.05 mm flange width  (~)        |
| Nut flange bolt circle | ~16 mm, 4× M3                | not modeled as discrete holes        | not an M3/4-hole hobby pattern    |
| Nut body diameter      | ~10.2 mm                     | (flange dominates bbox; ~body n/a)   | **10.31 mm**  ✓                   |
| Nut overall length     | ~15 mm                       | **24.0 mm** height                   | 12.30 mm                          |

Existing-model raw measurements:
- `t8_p2_lead_screw_l0200_simple.step`: bbox **8.000 × 8.000 × 200.000 mm**, volume 10 053 mm³
  (a smooth Ø8 rod — note it is labeled "p2" = 2 mm pitch but is modeled without threads).
- `t8_p2_flange_nut.step`: bbox **20.000 × 20.000 × 24.000 mm**, volume 6 334 mm³.

(The task's "expected" nut values — 22 mm flange, 16 mm bolt circle, 4× M3, 10.2 body, 15 length —
describe the generic hobby "KFL/anti-backlash T8 flange nut," which is not a McMaster product.)

---

## STEP download attempts (what was tried, what blocked it)

Browser automation *was* available (Chrome/Edge extension) and the McMaster site was reachable,
but downloads could not be captured:

1. **McMaster SPA never reaches `document_idle`** (persistent tracking/polling; the `trk.aspx`
   tracker was returning 503). Consequence: the extension's screenshot / `read_page` / native
   `click` tools all time out (45 s) on McMaster pages. All page reading had to be done through
   `javascript_tool` (synchronous DOM reads only).
2. The correct CAD flow was driven via JS: format selector expanded → **"3-D STEP"** selected →
   the download anchor's href resolved to
   `https://www.mcmaster.com/mvC/Library/CAD2/20260103/4F55AEFB/4537N151_Fast-Travel Ultra-Precision Lead Screw.STEP`.
3. **Programmatic anchor `.click()`** issued the request, but the network log shows it stuck at
   **`statusCode: pending`** indefinitely (the SPA context stalls the CAD stream); no file saved.
4. **Opening the CAD URL in a fresh tab (top-level navigation)** returned the file as an
   attachment (empty document body) but the CDP/extension-driven navigation **did not persist the
   download** to the local downloads directory (verified: no new files; browser prefs
   `prompt_for_download=false`, default dir = Downloads).
5. **In-page `fetch()` of the CAD URL** returns **403** (McMaster's WAF requires a real
   navigation — `Sec-Fetch-Mode: navigate`; a `fetch`/XHR sends `Sec-Fetch-Dest: empty` and is
   rejected). Forbidden request headers cannot be overridden from JS.
6. **Un-cookied direct fetch** (PowerShell `Invoke-WebRequest`) → **403 Forbidden**.
7. **WebFetch** on the CAD URL (task-suggested fallback) → **403 Forbidden** (no session cookies).

Net: the authenticated bytes are only delivered to a genuine, user-gesture browser navigation,
which this automation environment does not save. No STEP was obtained. (Even if it had been, it
would be the wrong-size Option-A part, not a T8x8.)

---

## License / terms note

McMaster-Carr provides 2-D and 3-D CAD models (STEP, IGES, Parasolid, SolidWorks, etc.) **free of
charge for customers' own design/reference use**; each 3-D model page states "The information in
this 3-D model is provided for reference only." Redistribution of McMaster CAD is not permitted;
use it within your own project only. No McMaster CAD was actually downloaded or redistributed here.

---

## Recommendation

- **Functionally, keep `cad/models/parts/t8_p2_*`** — the existing Ø8 × 200 mm screw and 20 mm
  flange nut are already correct for an 8 mm trapezoidal screw envelope. If you need real threads,
  model a Tr8x8(P2) 4-start rounded-trapezoidal profile, or:
- **Source a genuine Tr8x8 STEP from a trapezoidal-screw vendor** (Igus dryspin, Nook, Thomson,
  or the many 3D-printer "T8" leadscrew + brass/POM anti-backlash flange-nut vendors) or via the
  `step.parts` catalog. These are the products that actually match "T8x8, 8 mm dia, 8 mm lead,
  4-start" — McMaster is simply not a supplier of that hobby-standard screw.
- If a **McMaster** part is mandatory, decide which attribute matters more and pick Option A
  (8 mm lead, 4–5 mm dia, `4537N151` + `4537N208`) or Option B (8 mm dia, 1.5 mm lead, `7549K55`);
  then download its STEP **manually in a normal browser** (select "3-D STEP" → Download) since the
  automated download path is blocked.
