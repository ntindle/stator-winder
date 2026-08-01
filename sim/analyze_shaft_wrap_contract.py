"""Prove whether the upstream shaft-wrap contract can be met by settings alone.

The upstream implementation commands M1 from ``m1_zero`` even though M1 is
left at the last tooth of the preceding phase.  This program derives the
resulting motion for every possible balanced 24-slot configuration and checks
the narrower set of electrically equivalent transforms of the canonical
24n22p pattern.

It intentionally does not import or modify the upstream project.  The formulas
below are a direct transcription of ``Wind.move_to_teeth()`` and
``Wind.wind_wire_around_shaft()`` at upstream commit 6039b33.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path

import yaml


CANONICAL_24N22P = "AaAabBbBCcCcaAaABbBbcCcC"
PHASES = "abc"


def phase_indices(config: str) -> dict[str, list[int]]:
    """Match upstream ``get_winding_teeth_indices`` exactly."""
    lowered = config.lower()
    return {phase: [i for i, char in enumerate(lowered) if char == phase]
            for phase in PHASES}


def validate_balanced_24(config: str) -> dict[str, list[int]]:
    """Require the condition needed to wind all 24 teeth exactly once."""
    if len(config) != 24:
        raise ValueError(f"expected 24 slots, got {len(config)}")
    if any(char.lower() not in PHASES for char in config):
        raise ValueError("winding_config may contain only A/a, B/b, and C/c")
    groups = phase_indices(config)
    counts = {phase: len(indices) for phase, indices in groups.items()}
    if counts != {phase: 8 for phase in PHASES}:
        raise ValueError(f"all teeth exactly once requires 8 per phase, got {counts}")
    return groups


def requested_wrap_turns(config: str) -> list[dict[str, object]]:
    """Return target displacement at the two upstream wrap calls.

    Immediately before a wrap, the preceding ``wind()`` leaves M1 at

        m1_zero - 2*pi*last_index/24.

    ``wind_wire_around_shaft()`` then targets ``m1_zero +/- 4*pi``.  The
    initial zero therefore cancels from the displacement.
    """
    groups = validate_balanced_24(config)
    result: list[dict[str, object]] = []
    for previous, following in (("a", "b"), ("b", "c")):
        last_index = groups[previous][-1]
        first_index = groups[following][0]
        clockwise = config[first_index].islower()
        turns = 2.0 - last_index / 24.0 if clockwise else 2.0 + last_index / 24.0
        result.append({
            "previous_phase": previous.upper(),
            "next_phase": following.upper(),
            "previous_last_index": last_index,
            "next_first_index": first_index,
            "branch": "clockwise" if clockwise else "counterclockwise",
            "requested_turns": turns,
            "target_error_from_two_turns": turns - 2.0,
        })
    return result


def canonical_equivalents(config: str) -> set[str]:
    """Generate rotations/reflections/polarity/phase-name equivalences."""
    variants: set[str] = set()
    for reflected in (False, True):
        base = config[::-1] if reflected else config
        for inverted in (False, True):
            polarized = base.swapcase() if inverted else base
            for offset in range(len(config)):
                rotated = polarized[offset:] + polarized[:offset]
                for permutation in itertools.permutations(PHASES):
                    rename = dict(zip(PHASES, permutation))
                    transformed = "".join(
                        rename[char.lower()].upper()
                        if char.isupper()
                        else rename[char]
                        for char in rotated
                    )
                    variants.add(transformed)
    return variants


def execution(turns: float, velocity_rad_s: float, interval_s: float) -> float:
    """Constant-velocity displacement before the next upstream command."""
    capacity_turns = velocity_rad_s * interval_s / (2.0 * math.pi)
    return min(turns, capacity_turns)


def analyze(config: str, velocity_rad_s: float, interval_s: float) -> dict[str, object]:
    groups = validate_balanced_24(config)
    wraps = requested_wrap_turns(config)
    for wrap in wraps:
        requested = float(wrap["requested_turns"])
        executed = execution(requested, velocity_rad_s, interval_s)
        wrap["executed_turns_before_next_command"] = executed
        wrap["target_reached"] = math.isclose(executed, requested, abs_tol=1e-12)
        wrap["exactly_two_executed"] = math.isclose(executed, 2.0, abs_tol=1e-12)

    # For eight distinct indices, max(indices) >= 7.  Exact target displacement
    # is two turns only when that max is zero, so no balanced configuration can
    # meet both the target-completion and two-turn requirements.
    lower_bound_last_index = 8 - 1
    min_target_error_turns = lower_bound_last_index / 24.0

    equivalents = canonical_equivalents(CANONICAL_24N22P)
    capped_hacks: list[dict[str, object]] = []
    for candidate in sorted(equivalents):
        candidate_wraps = requested_wrap_turns(candidate)
        if all(float(wrap["requested_turns"]) > 2.0 for wrap in candidate_wraps):
            capped_hacks.append({
                "config": candidate,
                "requested_turns": [wrap["requested_turns"] for wrap in candidate_wraps],
            })

    return {
        "input": {
            "winding_config": config,
            "m1_velocity_rad_s": velocity_rad_s,
            "wrap_method_interval_s": interval_s,
            "phase_indices": groups,
        },
        "wraps": wraps,
        "proof": {
            "balanced_phase_size": 8,
            "last_index_lower_bound": lower_bound_last_index,
            "minimum_absolute_target_error_turns": min_target_error_turns,
            "zero_offset_cancels": True,
            "direction_sign_preserves_magnitude": True,
            "settings_only_solution_with_completed_targets": False,
            "reason": (
                "requested displacement is 2-L/24 turns in the clockwise branch "
                "or 2+L/24 in the counterclockwise branch; balanced phases imply "
                "L>=7, so neither branch can request exactly 2 turns"
            ),
        },
        "timing": {
            "maximum_turns_at_configured_velocity": (
                velocity_rad_s * interval_s / (2.0 * math.pi)
            ),
            "two_turn_minimum_velocity_rad_s": 4.0 * math.pi / interval_s,
            "configured_velocity_can_execute_two_turns": (
                velocity_rad_s * interval_s >= 4.0 * math.pi
            ),
        },
        "canonical_equivalence_search": {
            "variant_count": len(equivalents),
            "variants_with_both_targets_over_two_turns": len(capped_hacks),
            "capped_two_turn_velocity_rad_s": 4.0 * math.pi / interval_s,
            "examples": capped_hacks[:8],
            "why_not_a_solution": (
                "Capping both moves at two turns by lowering velocity leaves each "
                "absolute M1 target unfinished; the immediately following tooth "
                "command retargets the motor from the wrong absolute position"
            ),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--settings", type=Path, default=Path("out/settings.yml"))
    parser.add_argument("--interval", type=float, default=2.0,
                        help="seconds from wrap command through method return")
    parser.add_argument("--output", type=Path,
                        default=Path("out/reports/shaft_wrap_contract.json"))
    parser.add_argument("--require-solution", action="store_true",
                        help="exit 2 when the analysis proves settings are insufficient")
    args = parser.parse_args()

    settings = yaml.safe_load(args.settings.read_text(encoding="utf-8"))
    config = settings["winding"]["winding_config"]
    velocity = float(settings["motor"]["M1"]["velocity"])
    report = analyze(config, velocity, args.interval)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if (args.require_solution
            and not report["proof"]["settings_only_solution_with_completed_targets"]):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
