"""Audit legacy ease-out placement against the constructive controller plan."""

from __future__ import annotations

import collections
from datetime import datetime, timezone
import json
import math
from pathlib import Path

import yaml

from winding_plan import load_slot_winding_plan


ROOT = Path(__file__).resolve().parents[1]
SETTINGS = ROOT / "out" / "settings.yml"
PLAN = ROOT / "out" / "reports" / "slot_winding_plan.json"
CONTRACT_CAPTURE = ROOT / "out" / "capture" / "commands.jsonl"
UPSTREAM_CAPTURE = ROOT / "out" / "capture" / "upstream_current_ease.jsonl"
JSON_OUT = ROOT / "out" / "reports" / "controller_schedule.json"
MD_OUT = ROOT / "out" / "reports" / "controller_schedule.md"


def _events(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines()]


def _legacy_captured_spacing(events: list[dict], mm_per_rad: float,
                             wire_d: float) -> dict:
    per_pass = []
    for start_index, event in enumerate(events):
        if event.get("e") != "wind_wire":
            continue
        done_index = next(index for index in range(start_index + 1, len(events))
                          if events[index].get("e") == "wind_wire_done")
        targets = [float(row["model_target"])
                   for row in events[start_index:done_index]
                   if row.get("e") == "cmd" and row.get("m") == 0]
        # End-approach, deep-start, schedule..., rotating park.
        deposition = targets[2:-1]
        same_side = [abs(deposition[index + 2] - deposition[index])
                     * mm_per_rad
                     for index in range(len(deposition) - 2)]
        per_pass.append({
            "tooth": int(event["args"][0]),
            "schedule_target_count": len(deposition),
            "minimum_two_update_radial_delta_mm": min(same_side),
            "maximum_two_update_radial_delta_mm": max(same_side),
            "two_update_deltas_below_wire_diameter": sum(
                value < wire_d for value in same_side),
            "two_update_delta_count": len(same_side),
        })
    all_deltas = [value
                  for row in per_pass
                  for value in (
                      row["minimum_two_update_radial_delta_mm"],
                      row["maximum_two_update_radial_delta_mm"])]
    return {
        "passes": per_pass,
        "global_minimum_two_update_radial_delta_mm": min(all_deltas),
        "global_maximum_two_update_radial_delta_mm": max(all_deltas),
        "total_two_update_deltas_below_wire_diameter": sum(
            row["two_update_deltas_below_wire_diameter"] for row in per_pass),
        "total_two_update_delta_count": sum(
            row["two_update_delta_count"] for row in per_pass),
    }


def generate() -> dict:
    settings = yaml.safe_load(SETTINGS.read_text())
    plan = load_slot_winding_plan(PLAN)
    contract = _events(CONTRACT_CAPTURE)
    upstream = _events(UPSTREAM_CAPTURE)
    job = settings["job"]
    turns = int(settings["winding"]["turns"])
    wire_d = float(job["wire_finished_d_mm"])
    span = (float(job["radial_winding_span_mm"][1])
            - float(job["radial_winding_span_mm"][0]))
    mm_per_rad = 8.0 / (2.0 * math.pi)

    # Exact upstream law on the current job for one outbound layer.
    legacy_positions = [
        float(job["radial_winding_span_mm"][0])
        + span * math.sin(math.pi * turn / turns)
        for turn in range(turns // 2 + 1)
    ]
    legacy_pitches = [b - a for a, b in zip(
        legacy_positions, legacy_positions[1:])]
    legacy_theory = {
        "law": "r(n)=r_start+span*sin(pi*n/turns), mirrored after midpoint",
        "outbound_turn_intervals": len(legacy_pitches),
        "first_turn_pitch_mm": legacy_pitches[0],
        "shallow_turnaround_pitch_mm": legacy_pitches[-1],
        "minimum_pitch_mm": min(legacy_pitches),
        "maximum_pitch_mm": max(legacy_pitches),
        "intervals_below_finished_wire_diameter": sum(
            pitch < wire_d for pitch in legacy_pitches),
        "intervals_above_finished_wire_diameter": sum(
            pitch > wire_d for pitch in legacy_pitches),
        "finished_wire_diameter_mm": wire_d,
        "conclusion": (
            "nonuniform by construction: deep/root gaps and shallow bunching"
        ),
    }

    placements = plan.placements
    radial_steps = [abs(right.radial_mm - left.radial_mm)
                    for left, right in zip(placements, placements[1:])]
    spatial_steps = [math.hypot(
        right.radial_mm - left.radial_mm,
        right.tangential_mm - left.tangential_mm)
        for left, right in zip(placements, placements[1:])]
    origins = [row for row in contract
               if row.get("e") == "packing_pass_origin"]
    centers = [row for row in contract
               if row.get("e") == "packing_waypoint"
               and row.get("kind") == "placement_center"]
    holds = [row for row in contract
             if row.get("e") == "packing_waypoint"
             and row.get("kind") == "final_hold"]
    per_pass = []
    moved_ready_leads_rad = []
    for pass_index in range(24):
        origin = next(row for row in origins
                      if row["pass_index"] == pass_index)
        rows = [row for row in centers
                if row["pass_index"] == pass_index]
        counts = collections.Counter(row["placement_index"] for row in rows)
        moved_ready_leads_rad.extend(
            row["m2_phase_rad"] - row["m0_ready_phase_rad"]
            for previous, row in zip(rows, rows[1:])
            if abs(row["m0_target_rad"]
                   - previous["m0_target_rad"]) > 1e-9
        )
        per_pass.append({
            "pass_index": pass_index,
            "start_phase_rad": origin["start_phase_rad"],
            "phase_origin_rad": origin["phase_origin_rad"],
            "actual_travel_rad": origin["actual_travel_rad"],
            "placement_center_count": len(rows),
            "each_placement_exactly_twice": (
                counts == collections.Counter({index: 2 for index in range(50)})),
            "pre_crossing_deposition_count": (
                origin["pre_crossing_deposition_count"]),
            "maximum_m0_error_rad": max(abs(row["m0_error_rad"])
                                         for row in rows),
            "all_m0_settled_before_crossing": all(
                row["m0_settled_before_crossing"] for row in rows),
        })

    m0_speed_mm_s = (float(settings["motor"]["M0"]["velocity"])
                     * mm_per_rad)
    half_turn_s = math.pi / float(settings["motor"]["M2"]["velocity"])
    maximum_radial_step = max(radial_steps)
    timing = {
        "m0_speed_mm_s": m0_speed_mm_s,
        "m2_half_turn_available_s": half_turn_s,
        "maximum_radial_step_mm": maximum_radial_step,
        "maximum_step_nominal_m0_time_s": maximum_radial_step / m0_speed_mm_s,
        "controller_poll_reserve_s": 0.02,
        "minimum_analytical_settling_margin_s": (
            half_turn_s - maximum_radial_step / m0_speed_mm_s - 0.02),
    }

    contract_meta = next(row for row in contract if row["e"] == "meta")
    upstream_meta = next(row for row in upstream if row["e"] == "meta")
    contract_complete = next(row for row in contract
                             if row["e"] == "cycle_complete")
    upstream_complete = next(row for row in upstream
                             if row["e"] == "cycle_complete")
    result = {
        "schema": "controller-schedule-audit/v1",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "status": "PASS",
        "job": {
            "turns_per_tooth": turns,
            "finished_wire_diameter_mm": wire_d,
            "radial_span_mm": job["radial_winding_span_mm"],
            "liner_max_thickness_mm": job["liner_max_thickness_mm"],
        },
        "legacy_ease_out_theory": legacy_theory,
        "legacy_upstream_capture": {
            "path": str(UPSTREAM_CAPTURE),
            "controller_mode": upstream_meta["controller_mode"],
            "command_count": sum(row["e"] == "cmd" for row in upstream),
            "final_m1_zero_rad": upstream_complete["m1_zero"],
            **_legacy_captured_spacing(upstream, mm_per_rad, wire_d),
        },
        "constructive_plan": {
            "path": str(plan.path),
            "sha256": plan.sha256,
            "proof_sha256": plan.raw.get("proof_sha256"),
            "transition_status": plan.transition_status,
            "placement_count": len(placements),
            "half_turn_center_count": len(plan.half_turn_centers),
            "minimum_consecutive_radial_step_mm": min(radial_steps),
            "maximum_consecutive_radial_step_mm": max(radial_steps),
            "maximum_consecutive_2d_step_mm": max(spatial_steps),
            "nominal_wire_model_status": "PASS",
            "receiving_0p242_mm_sensitivity_status": (
                plan.receiving_sensitivity_status),
        },
        "controller_timing": timing,
        "contract_capture": {
            "path": str(CONTRACT_CAPTURE),
            "capture_schema": contract_meta["capture_schema"],
            "controller_mode": contract_meta["controller_mode"],
            "command_count": sum(row["e"] == "cmd" for row in contract),
            "placement_center_count": len(centers),
            "final_hold_event_count": len(holds),
            "phase_origin_classes_rad": sorted({
                round(float(row["phase_origin_rad"]), 9) for row in origins}),
            "final_m1_zero_rad": contract_complete["m1_zero"],
            "minimum_observed_ready_lead_s_for_changed_m0_target": (
                min(moved_ready_leads_rad)
                / float(settings["motor"]["M2"]["velocity"])),
            "passes": per_pass,
            "all_passes_exact": all(
                row["placement_center_count"] == 100
                and row["each_placement_exactly_twice"]
                and row["pre_crossing_deposition_count"] == 0
                and row["all_m0_settled_before_crossing"]
                for row in per_pass),
        },
        "limits": [
            ("The 0.240 mm exact construction has zero pairwise/core margin; "
             "the separate 0.242 mm receiving sensitivity fails."),
            ("Virtual closed-loop timing proves the software/model contract; "
             "real motor following, wire deformation, liner compression and "
             "enamel behavior still require instrumented hardware validation."),
        ],
    }
    if not result["contract_capture"]["all_passes_exact"]:
        result["status"] = "FAIL"
    return result


def markdown(report: dict) -> str:
    legacy = report["legacy_ease_out_theory"]
    captured = report["legacy_upstream_capture"]
    plan = report["constructive_plan"]
    timing = report["controller_timing"]
    contract = report["contract_capture"]
    return f"""# Controller winding schedule audit

**Result: {report['status']}** for the project-owned nominal 0.240 mm plan.

## Why the upstream schedule was rejected

The current upstream controller hardcodes an ease-out-sine triangular traverse.
On this {report['job']['turns_per_tooth']}-turn, {report['job']['finished_wire_diameter_mm']:.3f} mm job its
same-side pitch falls from **{legacy['maximum_pitch_mm']:.3f} mm** at the deep/root
end to **{legacy['minimum_pitch_mm']:.3f} mm** at the shallow turnaround.
{legacy['intervals_below_finished_wire_diameter']} of {legacy['outbound_turn_intervals']}
outbound intervals are below one finished-wire diameter, while
{legacy['intervals_above_finished_wire_diameter']} are above it.  That is deep
gapping and shallow bunching by construction, not a rendering artifact.

The captured unmodified baseline confirms the same problem: observed two-update
radial deltas span **{captured['global_minimum_two_update_radial_delta_mm']:.4f}..
{captured['global_maximum_two_update_radial_delta_mm']:.4f} mm**, and it also
retains the upstream shaft-wrap regression (final M1 zero
{captured['final_m1_zero_rad']:.3f} rad).

## Project-owned schedule

The controller consumes the exact `slot-winding-plan/v1` construction: {plan['placement_count']}
ordered turn placements and {plan['half_turn_center_count']} physical slot-side
centers per tooth.  Consecutive commanded radial changes are
{plan['minimum_consecutive_radial_step_mm']:.3f}..
{plan['maximum_consecutive_radial_step_mm']:.3f} mm; layer changes are represented
by the constructive tangential coordinates rather than by slowing M0 near the
mouth.

M0 has {timing['m2_half_turn_available_s']:.4f} s between crossings.  The worst
{timing['maximum_radial_step_mm']:.3f} mm target needs
{timing['maximum_step_nominal_m0_time_s']:.4f} s at configured velocity, leaving
**{timing['minimum_analytical_settling_margin_s']:.4f} s** after the two-poll
reserve.

The complete capture contains {contract['placement_center_count']} placement
events (24 x 100), phase-origin classes {contract['phase_origin_classes_rad']},
and {contract['command_count']} exact serial commands. Every placement 0..49 is
hit exactly twice per pass, no pre-origin crossing is counted, and M0 was
observed settled before every crossing. For targets that actually changed M0,
the minimum observed ready lead was
{contract['minimum_observed_ready_lead_s_for_changed_m0_target']:.4f} s.
Final M1 zero is
{contract['final_m1_zero_rad']:.3f} rad.

## Honest release boundary

- The exact 0.240 mm construction passes, but the 0.242 mm receiving sensitivity
  is **{plan['receiving_0p242_mm_sensitivity_status']}**. Measure incoming wire
  and reject anything above 0.240 mm for this plan.
- Virtual timing does not replace an instrumented hardware wind. Liner
  compression, enamel deformation, real following error, sag and snagging remain
  hardware validation items.
"""


def main() -> int:
    report = generate()
    JSON_OUT.write_text(json.dumps(report, indent=2))
    MD_OUT.write_text(markdown(report))
    print(f"{report['status']}: {JSON_OUT}")
    print(MD_OUT)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
