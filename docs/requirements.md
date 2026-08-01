# Derived Requirements — 4-Axis Stator Flyer Winder

Source of truth: `winder` repo @ `6039b33` (2026-06-13), read in full. This sheet is derived from
`src/winding.py`, `src/position.py`, `src/utils.py`, `src/constants.py`, `scripts/main.py`,
`scripts/ws.py`, `simulation/node_3d.gd`, `settings-example.yml`, `tests/dev-*.yml`.
GOAL.md is the parent spec. **The software is the fixed contract; geometry must serve it.**

## 1. Protocol contract (serial, host → controller)

| Command | Meaning |
|---|---|
| `M{id}A{value}\n` | Absolute position target, **radians**, float rounded to 3 decimals. Fire-and-forget (no ack; host paces with `sleep()` and position polling). |
| `M{id}P\n` | Position query → response `M{id}P{value}` (radians). Host busy-polls M2 during winding (~30 Hz). |
| `ESTOP\n` | Stop all motors. |

Sent value = model-space target × `direction` sign (per-axis bool in settings) × `m2_gear_ratio`
(M2 only; code constant in `src/constants.py`, currently 50/50 = 1.0 — **not** in settings.yml).
Motion model assumed by the software's own simulator: constant-velocity slew toward target at the
per-axis `velocity` (rad/s) from settings; no acceleration model. Firmware must track **multi-turn
absolute position** on M1 and M2 (M2 accumulates roughly ±7,600 rad over one full 24-slot 50-turn cycle;
M1 zero drifts cumulatively via shaft wraps).

## 2. Kinematic architecture (confirmed by Godot scene mapping)

Machine frame: **Z** = flyer rotation axis (horizontal), **Y** = up.

- **M0 (linear):** translates the stator carriage (with M1 unit) along Z. Godot: `stator.position.z = −M0 + const`.
- **M1 (rotary):** rotates the stator about the **vertical Y axis** — stator shaft is held vertical,
  one tooth points horizontally along Z toward the flyer. Godot: `stator.rotation.y = −M1 + π/2`.
  Unlimited rotation both directions (no cables to the rotating side).
- **M2 (rotary, continuous):** flyer arm spins about Z, coaxial with M0 travel. Godot: `arm.rotation.z = M2`.
  Wire enters through hollow shaft from rear (−Z side), exits at tip eyelet.
- **M3 (torque):** wire tension between spool and hollow shaft. Values are normalized torque
  (`wind_torque: 0.04`, `pull_wire_torque: 0.15`), voltage-mode. `dont_move_m3: true` supported →
  passive tensioner is legitimate for v1.

Winding geometry: the presented tooth's radial axis is parallel to Z; the flyer wraps wire around
the tooth neck; M0 traverse during winding spreads turns **along the tooth length** (radial
direction of the stator), not along stack height.

## 3. M0 coordinate semantics (settings.yml quirks included)

Upstream example values (radians): `wind_range_start=−25.5 < wind_range_end=−14.0` <
rotating `=end+6.0=−8.0` < init zero `=end+9.0=−5.0` < home `0`.

- **More negative = deeper insertion toward/through the flyer plane. Home (absolute 0) = fully retracted = load/unload pose.**
- `m0_zero = M0.end_to_zero + wind_range_end` (init pose after `init_position()`).
- Rotating position `= M1.end_to_rotating_position + wind_range_end` — **note: this key lives under
  the `M1:` section of settings.yml even though it is an M0 target.** The settings generator must respect this.
- During a winding pass M0 slews `start → end → start` once (ease-out-sine on M2 progress,
  re-targeted every π of M2 travel). `ease-out-sine` path has **no range clamp** in
  `position.py` — generated ranges must be self-consistent (`start < end`).
- Transmission ratio (rad→mm) is ours to define and document. Baseline: T8×8 lead screw ⇒
  **1.2732 mm/rad** (upstream's numbers are consistent with this: wind range 11.5 rad ≈ 14.6 mm,
  home-to-wind-end 14 rad ≈ 17.8 mm, indexing retract 6 rad ≈ 7.6 mm).

## 4. M1 / M2 motion semantics the mechanism must survive

- Tooth indexing: `M1 = m1_zero − idx·(2π/N)` (counter-clockwise tooth order), N = slot count (12/24/36 supported today).
- Shaft wrap (`wind_wire_around_shaft`, between each of the 3 phases): current upstream applies an
  **absolute** M1 target derived from `m1_zero`, then permanently mutates `m1_zero` and `m2_zero`.
  From the actual indexed starting poses in the canonical 24-slot raw capture, the completed
  physical deltas are **1.375000 turns** and **2.791667 turns**, not two and two.  The digital twin
  must reproduce those exact raw deltas as evidence, and the mechanism must clear their complete
  swept envelope, but that does **not** make them acceptable winding motion: parent `GOAL.md`
  explicitly requires two full turns at each inter-phase maneuver.  An independent settings and
  fixed-transmission proof shows neither can map both raw deltas to two physical chuck turns while
  preserving 24-slot indexing.  Normal-GOAL release therefore remains fail-closed until an
  unmodified upstream revision restores a relative `current_position ± 4π` target (or the user
  explicitly changes the software-contract authority).
- Flyer parking state machine (`Motor2State`): TOP (12 o'clock), BOTTOM (6 o'clock), each ±
  `angle_to_prevent_collision` (1.0 rad ≈ 57.3° in every shipped config). All six park states occur.
- One winding pass = `turns` (50 in the current default and raw capture) full revolutions + up to 1.5 extra revs for entry/exit
  repositioning (±π wire-side flips, ±1–2 rad park offsets).

## 5. Collision-critical poses (drives the interference test matrix)

1. **Indexing:** M0 at rotating position (stator retracted only ~7.6 mm behind wind-end), M1
   sweeping up to 2π, flyer parked at any of the 6 park states. Rotating stator teeth + shaft stub
   vs. parked arm, eyelet, and wire run.
2. **Wire-side flip, stator partially inserted:** `set_motor2_wire_position()` rotates M2 up to
   2.0 rad with M0 at `wind_range_end`.
3. **Post-pass park, stator fully inserted:** `prevent_collision()` rotates M2 ±1.0 rad with M0
   still at `wind_range_start` (deepest).
4. **`move_wire_to_right_position()` (fires at tooth 16 of 24n22p):** M2 sweeps **π + 1.0 rad with
   the stator fully inserted** at `wind_range_start`. Strongest flyer-vs-stator clearance case.
5. **Shaft wrap:** M0 at rotating position and flyer at TOP ±1 rad.  Collision coverage includes
   both the intended two-full-turn maneuver and the larger current raw 2.791667-turn sweep; the
   wire path from eyelet down to the shaft must clear the arm and chuck throughout.  Passing this
   mechanical envelope cannot waive the separate upstream command-stream failure.
6. **Full winding sweep:** flyer 360° swept volume vs. inserted stator (all M0 wind-range depths),
   chuck body below the stator, and carriage/frame. The chuck column below the stator sits inside
   the flyer's swept cylinder region — arm reach/shape and tip circle must be proven against the
   chuck at max stator OD and min chuck standoff.
7. **Load/unload:** all axes at absolute 0 (`back_to_zero()`), operator access to the chuck.
8. Winding start from 6 o'clock (`starts_at≠0` restart path): M2 moves π from zero with M0 at init zero.

## 6. Velocity / duty data (motor sizing inputs)

| Axis | Config velocity | Peak in cycle | Notes |
|---|---|---|---|
| M0 | **20 rad/s** generated | 20 rad/s ≈ 25.5 mm/s | Settings-only timing fix; reaches deep winding start before the first raw M2 command |
| M1 | **20 rad/s** generated | 20 rad/s | Reaches both current raw absolute targets inside 1.5 s; a corrected two-turn relative target would finish in 0.628319 s. Velocity changes timing only and cannot repair the wrong raw displacement. |
| M2 | 20 rad/s | 20 rad/s = **191 RPM** | `fast_winding` commands ~10.5 rad/s (≈100 RPM), then a single long move runs at the 20 rad/s cap. Design target **≥ 300 RPM (31.4 rad/s)** per GOAL. |
| M3 | torque mode | 0.04–0.15 norm. | Passive in v1 (`dont_move_m3`) |

## 7. Digital-twin fidelity requirements

- Drive the **unmodified** software (`scripts/main.py -s` piped, or `Wind(cfg, simulation=True).continuous_winding()`)
  and capture the command stream via `WINDER_LOG_LEVEL=DEBUG` (env override built into `utils.init_logger`) —
  every `move_motor` logs the exact serial command with a millisecond timestamp.
- Twin must replicate: constant-velocity slew model, `m1_zero`/`m2_zero` drift, Motor2State ±offsets,
  and the M0↔M2 coupled traverse. Official validation run uses **real sleeps** (unmodified timing);
  iteration runs may cap `sleep` at 0.35 s exactly as the repo's own test suite does.
- `scripts/ws.py` hardcodes `settings.yml` in CWD and streams 60 fps interpolated positions —
  our generated settings.yml must also live at repo root for the Godot/WebSocket path to work.

## 8. Flags / contradictions vs GOAL.md

1. **Shaft-wrap contradiction -- release blocker:** GOAL and the code comment require two physical
   M1 turns, but untouched upstream `6039b33` completes 1.375000 and 2.791667 turns.  The regression
   entered at upstream `8e7904a`.  Zero, direction, velocity, `starts_at`, M0/M2 settings, all 48
   canonical electrical pattern equivalents, and any one fixed affine M1 transmission have been
   exhausted; none can make both maneuvers two turns without also breaking tooth indexing.  The
   twin reproduces the stream and geometry is validated against its larger sweep, but normal-GOAL
   DoD remains **FAIL_CLOSED**.  No adapter or local upstream patch is applied.  The smallest
   documented upstream correction is to command the queried live M1 position ± 4π.
2. **Layering limitation, not a software-contract contradiction:** untouched upstream hard-codes
   `ease-out-sine`.  With the current linear T8 transmission it matches the deterministic
   diagnostic honeycomb schedule only 6/2400 placement states and contains zero/sub-wire-diameter
   radial pitch near the traverse reversals.  That means the simulator cannot claim exact layer
   order or neatness; `GOAL.md` explicitly leaves those tension/contact outcomes to hardware
   qualification.  It does **not** by itself require a synchronized guide or nonlinear M0
   transmission.  Production authorization still requires the raw-cycle wire path to distinguish
   intended workpiece/copper contact from steel or hardware contact and to prove a continuous
   centreline bend radius of at least 3 mm.
3. **R3 workpiece-route input gap:** tight conformity to the lined default tooth corner is only
   R0.23876 and cannot satisfy the literal 3 mm rule. Earlier retractable-shroud, R58/R64 review-arm,
   and aggregate-former studies are historical design lanes, not the selected authority. The current
   rev6 architecture uses two M0-following/M1-static active-sector PEEK guides on an outboard
   coil-bypass aluminum yoke plus rotating short-leadin PEEK caps. It passes all 2400 default-job
   terminal loci and the complete rigid raw-motion sweep, without claiming deterministic layer
   order. The flexible guide-to-workpiece conductor family between states, both exact two-turn shaft
   wraps, sag/snags/abrasion, and the 24-case launch envelope remain fail-closed. `StatorSpec` also
   does not define the finished motor's available rotor/end-bell axial cavity, so finished-motor fit
   remains a separate measured constraint.
4. GOAL says software drives ~100 RPM — true for the stepped `fast_winding` phase, but the long
   move to target runs at the **velocity cap = 191 RPM** with shipped configs. The ≥300 RPM
   mechanical target still gives ≥1.57× headroom over the cap; keep the cap in generated settings ≤ our rating/1.5.
5. GOAL describes `angle_to_prevent_collision` as clearing the flyer "during indexing" — the code
   also uses it **doubled** (wire-side flip) and combined with π **while the stator is fully
   inserted** (pose 4 above). Our clearance validation must cover those stronger cases, not just indexing.
6. `end_to_rotating_position` lives under `M1:` in settings.yml despite being an M0 target (§3) —
   generator must follow the quirk, not "fix" it.
7. `m2_gear_ratio` is a code constant, not a settings key: if our flyer is belt-driven with a
   non-1:1 ratio, the **firmware** (out of scope) or a 1:1 belt must absorb it. **Decision: design
   flyer drive as 1:1 belt** so the shipped constant stays valid.
8. Winding configs exist for 12, 24, 36 slots — mechanism must not assume 24 (index angle is 2π/N).

## 9. Fixed design decisions derived above

- M0 baseline: T8×8 lead screw (1.2732 mm/rad), MGN12 rail, home switch at retracted end,
  ~100 mm usable travel.  The rejected inverse-sine/two-track studies do not veto this direct-linear
  mapping; it remains the production candidate while the independent raw-cycle R3/contact wire-path
  gate is open.
- M1: vertical hollow-free chuck axis, motor under carriage, unlimited rotation; ER11 for measured Ø3–7 mm shafts and the separate `shaft8` socket holder only for Ø8.00 mm.
- M2: hollow-shaft flyer with selected Rev-D L79 OD10/ID6-to-OD12/ID9 shaft geometry, stock D10 NBK P30 flyer pulley, conditional D5+BNW motor P30, 210-3GT-6 belt at exact 30T:30T, and six balance trims. OD28–65 is a verification matrix (90 mm remains parametric); 0/24 launch certificates currently pass, so no envelope corner is production-authorized.
- M3: passive dancer + adjustable felt tensioner, 1–10 N, motorized-spool-ready mount, `dont_move_m3: true` in generated settings v1.
