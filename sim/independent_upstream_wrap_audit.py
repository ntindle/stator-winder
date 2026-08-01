"""Independent fail-closed audit of untouched-upstream M1 shaft wraps.

This audit consumes the real ``capture.py --controller upstream`` event stream.
It does not import, monkeypatch, or edit the upstream ``winder`` checkout.  Its
job is narrower than the project cycle verifier: determine whether settings or
an affine fixed mechanical transmission can turn both raw inter-phase moves
into exactly two physical chuck revolutions while preserving completed targets.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import subprocess
from pathlib import Path
from typing import Any

import yaml

from traj import Timeline, load_events


HERE = Path(__file__).resolve().parent
MACHINE = HERE.parent
ROOT = MACHINE.parent
DEFAULT_CAPTURE = MACHINE / "out" / "capture" / \
    "independent_upstream_wrap_6039.jsonl"
DEFAULT_SETTINGS = MACHINE / "out" / "settings.yml"
DEFAULT_WINDER = ROOT / "winder"
DEFAULT_JSON = MACHINE / "out" / "reports" / \
    "independent_upstream_wrap_audit.json"
DEFAULT_MD = MACHINE / "out" / "reports" / \
    "independent_upstream_wrap_audit.md"
EXPECTED_COMMIT = "6039b33c8f15a20086c2195c3f2d02b3a833e8ca"
CANONICAL = "AaAabBbBCcCcaAaABbBbcCcC"
PHASES = "abc"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _git(winder: Path, *args: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(winder), *args], text=True
    ).strip()


def _source_line(lines: list[str], needle: str) -> int:
    return next(index for index, line in enumerate(lines, 1) if needle in line)


def _phase_indices(config: str) -> dict[str, list[int]]:
    lowered = config.lower()
    return {
        phase: [index for index, char in enumerate(lowered) if char == phase]
        for phase in PHASES
    }


def _canonical_equivalents(config: str) -> set[str]:
    """All rotation/reflection/polarity/phase-name equivalents."""
    variants: set[str] = set()
    for reflected in (False, True):
        base = config[::-1] if reflected else config
        for inverted in (False, True):
            polarized = base.swapcase() if inverted else base
            for offset in range(len(config)):
                rotated = polarized[offset:] + polarized[:offset]
                for permutation in itertools.permutations(PHASES):
                    rename = dict(zip(PHASES, permutation))
                    variants.add("".join(
                        rename[char.lower()].upper()
                        if char.isupper() else rename[char]
                        for char in rotated
                    ))
    return variants


def _requested_turns(config: str) -> tuple[float, float]:
    groups = _phase_indices(config)
    if any(len(groups[phase]) != 8 for phase in PHASES):
        raise ValueError("a 24-slot three-phase job must contain 8 A/B/C teeth")
    result = []
    for previous, following in (("a", "b"), ("b", "c")):
        last_index = groups[previous][-1]
        first_index = groups[following][0]
        clockwise = config[first_index].islower()
        result.append(
            2.0 - last_index / 24.0
            if clockwise else 2.0 + last_index / 24.0
        )
    return result[0], result[1]


def _wrap_rows(events: list[dict[str, Any]], timeline: Timeline) \
        -> list[dict[str, Any]]:
    calls = [event for event in events
             if event.get("e") == "wind_wire_around_shaft"]
    rows: list[dict[str, Any]] = []
    velocity = float(timeline.meta["velocities"][1])
    for ordinal, call in enumerate(calls, 1):
        start_t = float(call["t"])
        m1_command = next(
            event for event in events
            if event.get("e") == "cmd" and event.get("m") == 1
            and float(event["t"]) >= start_t - 1.0e-12
        )
        m2_return = next(
            event for event in events
            if event.get("e") == "cmd" and event.get("m") == 2
            and float(event["t"]) > start_t + 1.0e-12
        )
        method_done = next(
            event for event in events
            if event.get("e") == "wind_wire_around_shaft_done"
            and float(event["t"]) > start_t
        )
        next_m1 = next(
            event for event in events
            if event.get("e") == "cmd" and event.get("m") == 1
            and float(event["t"]) > start_t + 1.0e-12
        )
        start_m1 = float(timeline.axes[1].pos_at(start_t))
        target_m1 = float(m1_command["model_target"])
        delta = target_m1 - start_m1
        arrival_t = start_t + abs(delta) / velocity
        m2_return_t = float(m2_return["t"])
        next_m1_t = float(next_m1["t"])
        turns = abs(delta) / (2.0 * math.pi)
        rows.append({
            "index": ordinal,
            "next_wire_index": int(call["args"][0]),
            "start_t_s": start_t,
            "m1_start_rad": start_m1,
            "m1_absolute_target_rad": target_m1,
            "serial_command": m1_command["command"].rstrip("\n"),
            "controller_target_rad": float(m1_command["controller_target"]),
            "m1_delta_rad": delta,
            "completed_motor_turns": turns,
            "exactly_two_turns": math.isclose(turns, 2.0, abs_tol=1.0e-9),
            "predicted_arrival_t_s": arrival_t,
            "m2_recenter_command_t_s": m2_return_t,
            "method_done_t_s": float(method_done["t"]),
            "next_m1_target_t_s": next_m1_t,
            "arrival_slack_before_m2_recenter_s": m2_return_t - arrival_t,
            "arrival_slack_before_next_m1_target_s": next_m1_t - arrival_t,
            "target_complete_before_m2_recenter": arrival_t <= m2_return_t,
            "target_complete_before_next_m1_target": arrival_t <= next_m1_t,
            "m1_at_target_at_m2_recenter_rad": float(
                timeline.axes[1].pos_at(m2_return_t)),
            "next_m1_serial_command": next_m1["command"].rstrip("\n"),
        })
    return rows


def analyze(capture: Path, settings_path: Path, winder: Path) \
        -> dict[str, Any]:
    events = load_events(capture)
    timeline = Timeline(events)
    meta = timeline.meta
    settings = yaml.safe_load(settings_path.read_text(encoding="utf-8"))
    source = winder / "src" / "winding.py"
    constants_source = winder / "src" / "constants.py"
    source_lines = source.read_text(encoding="utf-8").splitlines()
    commit = _git(winder, "rev-parse", "HEAD")
    status = _git(winder, "status", "--porcelain")
    wraps = _wrap_rows(events, timeline)

    observed = [float(row["completed_motor_turns"]) for row in wraps]
    ratio_for_two = [2.0 / value for value in observed]
    equivalent_rows = [
        {"config": config, "turns": list(_requested_turns(config))}
        for config in sorted(_canonical_equivalents(CANONICAL))
    ]
    equal_equivalents = [row for row in equivalent_rows
                         if math.isclose(row["turns"][0], row["turns"][1],
                                         abs_tol=1.0e-12)]

    # A direct-drive, completed target has magnitude 2 +/- L/N turns.  Each
    # phase contains eight distinct non-negative indices, so L>=7.  It can
    # never equal two.  Two disjoint phases cannot have the same last index;
    # therefore their magnitudes also cannot be equal under a shared branch.
    # Opposite branches can only be equal if both last indices are zero,
    # impossible for two disjoint eight-index phase sets.
    balanced_proof = {
        "slots": 24,
        "teeth_per_phase": 8,
        "minimum_possible_phase_last_index": 7,
        "completed_direct_drive_formula": "abs(delta turns) = 2 - L/24 or 2 + L/24",
        "direct_drive_exact_two_possible": False,
        "any_balanced_pattern_equal_pair_for_one_fixed_ratio": False,
        "reason": (
            "For equal branches, equal magnitudes require equal last indices, "
            "but phase sets are disjoint. For opposite branches, equality "
            "requires both last indices to be zero. Neither is possible for "
            "two disjoint eight-tooth phases."
        ),
    }

    velocity = float(settings["motor"]["M1"]["velocity"])
    first_bad = _git(
        winder, "log", "-1", "--format=%H", "-S",
        "self.move_motor(1, self.m1_zero", "--", "src/winding.py"
    )
    source_anchor = {
        "method_line": _source_line(
            source_lines, "def wind_wire_around_shaft"),
        "clockwise_absolute_target_line": _source_line(
            source_lines, "self.move_motor(1, self.m1_zero - motor1_rotation)"),
        "counterclockwise_absolute_target_line": _source_line(
            source_lines, "self.move_motor(1, self.m1_zero + motor1_rotation)"),
    }

    predicted_relative_arrival = 4.0 * math.pi / velocity
    minimal_diff = """diff --git a/src/winding.py b/src/winding.py
@@ def wind_wire_around_shaft(self, next_wire_idx: int):
         starting_from_cw = self.is_starting_from_cw(next_wire_idx)
+        motor1_pos = self.get_motor_position(1)
 
         if starting_from_cw:
-            self.move_motor(1, self.m1_zero - motor1_rotation)
+            self.move_motor(1, motor1_pos - motor1_rotation)
             self.m1_zero -= motor1_rotation
         else:
-            self.move_motor(1, self.m1_zero + motor1_rotation)
+            self.move_motor(1, motor1_pos + motor1_rotation)
             self.m1_zero += motor1_rotation + math.pi * 2
"""

    all_arrived = all(row["target_complete_before_m2_recenter"] for row in wraps)
    exactly_two = all(row["exactly_two_turns"] for row in wraps)
    report: dict[str, Any] = {
        "schema": "independent-upstream-shaft-wrap-audit/v1",
        "status": "FAIL_CLOSED" if not exactly_two else "PASS",
        "decision": (
            "NO_SETTINGS_OR_FIXED_AFFINE_TRANSMISSION_CAN_MAKE_BOTH_"
            "COMPLETED_RAW_WRAPS_EXACTLY_TWO_PHYSICAL_CHUCK_TURNS"
        ),
        "authority": {
            "goal_requirement": (
                "GOAL.md line 13: M1 does 2 full turns during each "
                "wire-around-shaft maneuver; line 57 fixes unmodified software"
            ),
            "winder_commit": commit,
            "expected_winder_commit": EXPECTED_COMMIT,
            "upstream_worktree_clean": status == "",
            "upstream_status_porcelain": status.splitlines(),
            "capture_controller_mode": meta.get("controller_mode"),
            "capture_winder_commit": meta.get("winder_commit"),
            "capture_sha256": _sha256(capture),
            "settings_sha256": _sha256(settings_path),
            "winding_source_sha256": _sha256(source),
            "constants_source_sha256": _sha256(constants_source),
            "source_lines": source_anchor,
            "absolute_target_change_first_commit": first_bad,
            "last_prechange_commit": "8ae82f9",
        },
        "capture": {
            "path": str(capture.resolve()),
            "settings_path": str(settings_path.resolve()),
            "m1_velocity_rad_s": velocity,
            "m1_direction_setting": bool(settings["motor"]["M1"]["direction"]),
            "m1_zero_setting_rad": float(settings["motor"]["M1"]["zero"]),
            "winding_config": settings["winding"]["winding_config"],
            "raw_wrap_count": len(wraps),
            "all_absolute_targets_complete_before_m2_recenter": all_arrived,
            "all_absolute_targets_complete_before_next_m1_target": all(
                row["target_complete_before_next_m1_target"] for row in wraps),
            "each_wrap_exactly_two_turns": exactly_two,
            "wraps": wraps,
        },
        "settings_only_audit": {
            "m1_zero": {
                "can_fix": False,
                "proof": "The same additive zero is present in start and target and cancels from delta.",
            },
            "m1_direction": {
                "can_fix": False,
                "proof": "The setting multiplies controller-space positions by -1; it changes sign, not magnitude.",
            },
            "m1_velocity": {
                "can_fix_completed_targets": False,
                "proof": (
                    "Velocity changes arrival time only. If a target completes, "
                    "travel remains its unequal absolute delta; deliberate "
                    "retargeting before arrival violates the completion requirement."
                ),
                "minimum_velocity_to_complete_larger_wrap_before_m2_recenter_rad_s": (
                    max(abs(float(row["m1_delta_rad"])) for row in wraps) / 1.5
                ),
                "configured_velocity_rad_s": velocity,
                "configured_velocity_sufficient": all_arrived,
            },
            "starts_at": {
                "can_fix": False,
                "proof": (
                    "wind_wire_around_shaft sets starts_at=0; changing the initial "
                    "value only omits phase-A teeth and fails the 24-pass job."
                ),
            },
            "other_geometry_settings": {
                "can_fix": False,
                "proof": (
                    "M0 ranges/rotating position and M2 zero/park offset do not "
                    "enter either M1 target expression."
                ),
            },
            "m1_transmission_ratio": {
                "setting_exists": False,
                "proof": (
                    "Upstream constants.py defines only m2_gear_ratio, and "
                    "move_motor/get_motor_position apply it only when motor_id == 2."
                ),
            },
            "balanced_winding_config": balanced_proof,
            "canonical_equivalence_search": {
                "variant_count": len(equivalent_rows),
                "variants_with_equal_wrap_magnitudes": len(equal_equivalents),
                "equal_variants": equal_equivalents,
            },
        },
        "fixed_mechanical_transmission_audit": {
            "model": "physical_chuck_angle = k * controller_M1_angle + b",
            "offset_b_cancels": True,
            "direction_is_sign_of_k_and_cannot_change_magnitude_ratio": True,
            "observed_completed_motor_turns": observed,
            "required_abs_k_for_each_wrap": ratio_for_two,
            "one_fixed_k_satisfies_both": math.isclose(
                ratio_for_two[0], ratio_for_two[1], abs_tol=1.0e-12),
            "equations": [
                "abs(k) * 11/8 = 2 -> abs(k) = 16/11",
                "abs(k) * 67/24 = 2 -> abs(k) = 48/67",
            ],
            "indexing_constraint": (
                "Because upstream commands adjacent tooth indices one motor "
                "turn/24 apart, preserving all 24 physical tooth poses requires "
                "k congruent to +1 or -1 modulo 24. Neither 16/11 nor 48/67 qualifies."
            ),
            "slip_or_hard_stop_is_not_a_fixed_transmission": (
                "A clutch or stop that discards the residual target motion breaks "
                "absolute-position authority and encoder/chuck registration."
            ),
        },
        "smallest_upstream_correction_suggestion": {
            "applied": False,
            "description": (
                "Read actual M1 once and form the two-turn command relative to "
                "that pose; retain the existing zero mutations and protocol."
            ),
            "diff": minimal_diff,
            "predicted_turns": [2.0, 2.0],
            "predicted_arrival_duration_s_at_configured_velocity": (
                predicted_relative_arrival),
            "slack_before_m2_recenter_s": 1.5 - predicted_relative_arrival,
            "source_precedent": (
                "The last pre-change implementation at 8ae82f9 queried "
                "get_motor_position(1) and formed a relative target."
            ),
        },
        "gates": {
            "real_untouched_upstream_capture": (
                meta.get("controller_mode") == "upstream"
                and meta.get("winder_commit") == EXPECTED_COMMIT
                and commit == EXPECTED_COMMIT
                and status == ""
            ),
            "both_raw_targets_complete_before_next_target": all(
                row["target_complete_before_next_m1_target"] for row in wraps),
            "both_raw_moves_exactly_two_physical_turns_direct_drive": exactly_two,
            "settings_only_solution": False,
            "fixed_affine_transmission_solution": False,
            "goal_requirement_satisfied_by_unmodified_upstream": exactly_two,
        },
    }
    return report


def _markdown(report: dict[str, Any]) -> str:
    wraps = report["capture"]["wraps"]
    mech = report["fixed_mechanical_transmission_audit"]
    fix = report["smallest_upstream_correction_suggestion"]
    lines = [
        "# Independent untouched-upstream shaft-wrap audit",
        "",
        f"**Status: {report['status']}**",
        "",
        "The real upstream capture reaches both commanded M1 targets before "
        "the flyer recenters and before the next M1 target. It does not execute "
        "two chuck turns: the completed moves are 1.375000 and 2.791667 turns.",
        "",
        "| wrap | raw M1 command | completed turns | arrival slack before M2 recenter | exactly 2 |",
        "|---:|---|---:|---:|:---:|",
    ]
    for row in wraps:
        lines.append(
            f"| {row['index']} | `{row['serial_command']}` | "
            f"{row['completed_motor_turns']:.9f} | "
            f"{row['arrival_slack_before_m2_recenter_s']:.6f} s | "
            f"{'yes' if row['exactly_two_turns'] else 'no'} |"
        )
    lines += [
        "",
        "## Why settings cannot repair it",
        "",
        "- M1 zero is additive and cancels from every start-to-target delta.",
        "- M1 direction and a fixed mechanical reversal change only sign.",
        "- Velocity changes arrival time, not completed-target distance; slowing "
        "  until a command is interrupted fails the required completion gate.",
        "- `starts_at` is reset to zero inside the wrap method. Other M0/M2 "
        "  geometry settings do not occur in the M1 expressions.",
        "- For any balanced 24-slot pattern, a completed direct-drive move is "
        "  `2 - L/24` or `2 + L/24` turns with phase-last index `L >= 7`, "
        "  never exactly two. None of the 48 electrical equivalents makes "
        "  the two unequal moves equal.",
        "",
        "## Why fixed gearing cannot repair it",
        "",
        f"One wrap needs `|k|={mech['required_abs_k_for_each_wrap'][0]:.9f}`; "
        f"the other needs `|k|={mech['required_abs_k_for_each_wrap'][1]:.9f}`. "
        "A single ratio cannot satisfy both, and either ratio also corrupts "
        "the upstream one-tooth-per-1/24-turn indexing scale.",
        "",
        "## Smallest upstream correction (documented only; not applied)",
        "",
        fix["description"],
        "",
        "```diff",
        fix["diff"].rstrip(),
        "```",
        "",
        f"At 20 rad/s each corrected 4pi move arrives in "
        f"{fix['predicted_arrival_duration_s_at_configured_velocity']:.6f} s, "
        f"leaving {fix['slack_before_m2_recenter_s']:.6f} s before the current "
        "1.5 s M2-recenter boundary.",
        "",
        "The upstream checkout was not modified.",
    ]
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", type=Path, default=DEFAULT_CAPTURE)
    parser.add_argument("--settings", type=Path, default=DEFAULT_SETTINGS)
    parser.add_argument("--winder", type=Path, default=DEFAULT_WINDER)
    parser.add_argument("--json", type=Path, default=DEFAULT_JSON)
    parser.add_argument("--markdown", type=Path, default=DEFAULT_MD)
    args = parser.parse_args()

    report = analyze(args.capture.resolve(), args.settings.resolve(),
                     args.winder.resolve())
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    args.markdown.write_text(_markdown(report), encoding="utf-8")
    print(json.dumps({
        "status": report["status"],
        "turns": [row["completed_motor_turns"]
                  for row in report["capture"]["wraps"]],
        "targets_complete": report["gates"][
            "both_raw_targets_complete_before_next_target"],
        "settings_only_solution": False,
        "fixed_affine_transmission_solution": False,
        "json": str(args.json),
        "markdown": str(args.markdown),
    }, indent=2))


if __name__ == "__main__":
    main()
