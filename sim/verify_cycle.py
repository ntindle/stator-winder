"""Verify that a captured winding cycle proves DoD #1 motion claims.

This is intentionally independent of ``report.py``.  A GLB existing on disk
is not proof that the captured planner cycle is complete or physically did
what the requirements say.  Results are written to
``out/reports/cycle.json`` and failures return a non-zero exit status.
"""

from __future__ import annotations

import collections
import argparse
import hashlib
import json
import math
import re
import subprocess
from pathlib import Path

from traj import Timeline, load_events, winding_windows

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "out"
COMMAND_RE = re.compile(r"^M([0-3])A(-?(?:\d+(?:\.\d*)?|\.\d+))\n$")
REPORT_SCHEMA = "captured-cycle-verification/v2"
SHAFT_WRAP_TURNS_TOL = 1.0e-3


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(value):
    payload = dict(value)
    payload.pop("report_sha256", None)
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _controller_effective_model_target(command, meta):
    """Invert direction/gear mapping after serial target quantization.

    Schema-4 capture records the ideal model target used by upstream's Python
    simulation and the actual three-decimal controller target.  Geometry and
    physical-motion claims must use the latter mapped back into model space.
    """

    axis = command.get("m")
    directions = meta.get("directions")
    if type(axis) is not int or axis not in range(4):
        raise ValueError(f"invalid command axis {axis!r}")
    if (
        not isinstance(directions, list)
        or len(directions) != 4
        or any(type(value) is not bool for value in directions)
    ):
        raise ValueError("capture directions must contain four booleans")
    try:
        target = float(command["controller_target"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("command has no finite controller_target") from exc
    if not math.isfinite(target):
        raise ValueError("controller_target must be finite")
    if axis == 2:
        try:
            gear = float(meta["m2_gear_ratio"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("capture has no finite nonzero M2 gear ratio") from exc
        if not math.isfinite(gear) or abs(gear) <= 1.0e-12:
            raise ValueError("M2 gear ratio must be finite and nonzero")
        target /= gear
    if not directions[axis]:
        target = -target
    return target


def _controller_effective_events(events):
    """Clone a capture with command targets replaced by physical targets."""

    meta = next((event for event in events if event.get("e") == "meta"), None)
    if not isinstance(meta, dict):
        raise ValueError("capture has no meta event")
    result = []
    for event in events:
        row = dict(event)
        if row.get("e") == "cmd":
            source_target = float(row.get(
                "requested_model_target",
                row.get("model_target", row.get("a")),
            ))
            effective = _controller_effective_model_target(row, meta)
            row["source_model_target"] = source_target
            row["controller_effective_model_target"] = effective
            row["model_target"] = effective
        result.append(row)
    return result


def _upstream_checkout_identity(meta):
    """Describe the current upstream checkout without mutating it."""

    raw_path = meta.get("winder_path")
    checkout = Path(raw_path).resolve() if isinstance(raw_path, str) else None
    source = checkout / "src" / "winding.py" if checkout is not None else None
    result = {
        "path": str(checkout) if checkout is not None else None,
        "capture_commit": meta.get("winder_commit"),
        "current_commit": None,
        "status_short": None,
        "source_path": str(source) if source is not None else None,
        "source_sha256": None,
        "same_clean_commit_as_capture": False,
    }
    if checkout is None or source is None or not source.is_file():
        return result
    try:
        result["current_commit"] = subprocess.check_output(
            ["git", "-C", str(checkout), "rev-parse", "HEAD"],
            text=True,
        ).strip()
        result["status_short"] = subprocess.check_output(
            ["git", "-C", str(checkout), "status", "--short"],
            text=True,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return result
    result["source_sha256"] = _sha256(source)
    result["same_clean_commit_as_capture"] = (
        result["current_commit"] == result["capture_commit"]
        and result["status_short"] == ""
    )
    return result


def _revolution_intervals(track, start, end, count):
    """Return physical full-revolution intervals from one axis trajectory."""
    points = [(float(start), float(track.pos_at(start)))]
    points.extend((float(t), float(position)) for t, position in track.knots
                  if start < t < end)
    points.append((float(end), float(track.pos_at(end))))
    points.sort()
    result = []
    travelled = 0.0
    threshold = 2.0 * math.pi
    turn_start = float(start)
    for (t0, p0), (t1, p1) in zip(points, points[1:]):
        distance = abs(p1 - p0)
        if distance <= 1.0e-12 or t1 <= t0:
            continue
        while (len(result) < count
               and travelled + distance + 1.0e-9 >= threshold):
            fraction = (threshold - travelled) / distance
            turn_end = t0 + fraction * (t1 - t0)
            result.append((turn_start, turn_end))
            turn_start = turn_end
            threshold += 2.0 * math.pi
        travelled += distance
        if len(result) == count:
            break
    return result


def _track_extrema(track, start, end):
    values = [float(track.pos_at(start)), float(track.pos_at(end))]
    values.extend(float(position) for t, position in track.knots
                  if start < t < end)
    return min(values), max(values)


def _raw_shaft_wraps(events, timeline, *, source_timeline=None):
    """Infer raw wrap travel and prove its absolute-target source formula.

    ``timeline`` is controller-effective model space.  ``source_timeline`` is
    the ideal, unquantized upstream simulation when available.  Keeping both
    prevents the former report from calling ideal Python targets exact
    physical-controller travel while retaining the source-level contradiction.
    """

    starts = [index for index, event in enumerate(events)
              if event.get("e") == "wind_wire_around_shaft"]
    teeth_count = int(timeline.meta.get("teeth_count", 0))
    rows = []
    for start_index in starts:
        call = events[start_index]
        number = int(call.get("args", [len(rows) + 1])[0])
        start = float(call["t"])
        done_index = next((
            index for index in range(start_index + 1, len(events))
            if events[index].get("e") == "wind_wire_around_shaft_done"
        ), None)
        if done_index is None:
            rows.append({"index": number, "ok": False,
                         "reason": "missing shaft-wrap done marker"})
            continue
        m1_commands = [
            event for event in events[start_index + 1:done_index]
            if event.get("e") == "cmd" and event.get("m") == 1
        ]
        m2_commands = [
            event for event in events[start_index + 1:done_index]
            if event.get("e") == "cmd" and event.get("m") == 2
        ]
        if len(m1_commands) != 1 or not m2_commands:
            rows.append({
                "index": number,
                "ok": False,
                "reason": (
                    f"expected one M1 target and a following M2 return; got "
                    f"M1={len(m1_commands)}, M2={len(m2_commands)}"
                ),
            })
            continue
        m1_command = m1_commands[0]
        m2_return = m2_commands[0]
        end = float(m2_return["t"])
        start_pose = timeline.pose_at(start)
        end_pose = timeline.pose_at(end)
        target = float(m1_command.get(
            "model_target", m1_command.get("a", math.nan)))
        delta = target - float(start_pose[1])
        arrived = abs(float(end_pose[1]) - target) <= 0.01
        fixed = (
            abs(float(end_pose[0]) - float(start_pose[0])) <= 1.0e-9
            and abs(float(end_pose[2]) - float(start_pose[2])) <= 1.0e-9
        )

        previous_pass = next((
            events[index] for index in range(start_index - 1, -1, -1)
            if events[index].get("e") == "wind_wire"
        ), None)
        next_pass = next((
            events[index] for index in range(done_index + 1, len(events))
            if events[index].get("e") == "wind_wire"
        ), None)
        previous_args = (
            previous_pass.get("args", []) if isinstance(previous_pass, dict)
            else []
        )
        next_args = (
            next_pass.get("args", []) if isinstance(next_pass, dict) else []
        )
        previous_tooth = (
            int(previous_args[0]) if previous_args else None
        )
        next_tooth = int(next_args[0]) if next_args else None
        next_clockwise = (
            bool(next_args[1]) if len(next_args) > 1 else None
        )
        direction = -1 if next_clockwise is True else 1

        source_target = float(m1_command.get(
            "source_model_target",
            m1_command.get("requested_model_target",
                           m1_command.get("a", math.nan)),
        ))
        source_start_pose = (
            source_timeline.pose_at(start)
            if source_timeline is not None else start_pose
        )
        source_end_pose = (
            source_timeline.pose_at(end)
            if source_timeline is not None else end_pose
        )
        source_start_m1 = float(source_start_pose[1])
        source_delta = source_target - source_start_m1
        source_turns = abs(source_delta) / (2.0 * math.pi)
        bookkeeping_zero = source_target - direction * 4.0 * math.pi
        expected_indexed_start = (
            bookkeeping_zero
            - 2.0 * math.pi * previous_tooth / teeth_count
            if previous_tooth is not None and teeth_count > 0 else math.nan
        )
        expected_source_delta = (
            direction * 4.0 * math.pi
            + 2.0 * math.pi * previous_tooth / teeth_count
            if previous_tooth is not None and teeth_count > 0 else math.nan
        )
        formula_ok = (
            previous_tooth is not None
            and next_tooth is not None
            and next_clockwise is not None
            and teeth_count > 0
            and abs(source_start_m1 - expected_indexed_start) <= 1.0e-6
            and abs(source_delta - expected_source_delta) <= 1.0e-6
        )

        required_relative_target = (
            float(start_pose[1]) + direction * 4.0 * math.pi
        )
        required_target_correction = required_relative_target - target
        controller_turns = abs(delta) / (2.0 * math.pi)
        command_t = float(m1_command["t"])
        arrival_t = command_t + abs(delta) / float(
            timeline.meta["velocities"][1]
        )
        available_interval = end - command_t
        rows.append({
            "index": number,
            "start_t": start,
            "done_t": end,
            "start_m0": float(start_pose[0]),
            "done_m0": float(end_pose[0]),
            "start_m1": float(start_pose[1]),
            "target_m1": target,
            "done_m1": float(end_pose[1]),
            "source_model_start_m1": source_start_m1,
            "source_model_target_m1": source_target,
            "source_model_done_m1": float(source_end_pose[1]),
            "start_m2": float(start_pose[2]),
            "done_m2": float(end_pose[2]),
            "delta_m1": delta,
            "turns": controller_turns,
            "controller_effective_turns": controller_turns,
            "source_model_delta_m1": source_delta,
            "source_model_turns": source_turns,
            "serial_command": str(m1_command.get("command", "")).strip(),
            "controller_target_m1": m1_command.get("controller_target"),
            "controller_quantization_target_delta_rad": target - source_target,
            "target_semantics": "unbounded absolute M1 position in radians",
            "previous_phase_last_tooth_index": previous_tooth,
            "next_phase_first_tooth_index": next_tooth,
            "next_phase_branch": (
                "clockwise" if next_clockwise is True else
                "counterclockwise" if next_clockwise is False else None
            ),
            "source_formula": {
                "teeth_count": teeth_count,
                "bookkeeping_zero_rad": bookkeeping_zero,
                "indexed_start_rad": expected_indexed_start,
                "absolute_target_rad": source_target,
                "delta_rad": expected_source_delta,
                "turns": abs(expected_source_delta) / (2.0 * math.pi),
                "expression": (
                    "target=(bookkeeping_zero +/- 4*pi); "
                    "start=bookkeeping_zero-2*pi*last_tooth/teeth_count"
                ),
                "matches_capture": formula_ok,
            },
            "required_two_turn_relative_target_m1": required_relative_target,
            "required_target_correction_rad": required_target_correction,
            "required_target_correction_turns": (
                required_target_correction / (2.0 * math.pi)
            ),
            "arrival_t": arrival_t,
            "available_target_interval_s": available_interval,
            "arrival_margin_s": end - arrival_t,
            "m1_arrived": arrived,
            "m0_m2_fixed_during_contact": fixed,
            "absolute_target_formula_matches_capture": formula_ok,
            "interpretation_verdict": (
                "GENUINE_UPSTREAM_ABSOLUTE_TARGET_CONTRADICTION"
                if formula_ok else "UNRESOLVED_CAPTURE_INTERPRETATION"
            ),
            "ok": arrived and fixed and formula_ok,
        })
    return rows


def _check(report, name, ok, value=None, requirement=None):
    report["checks"][name] = {
        "ok": bool(ok), "value": value, "requirement": requirement,
    }
    if not ok:
        report["fail"].append(name)


def _paired_intervals(events, start_name, done_name):
    starts, intervals = [], []
    for event in events:
        if event["e"] == start_name:
            starts.append(event)
        elif event["e"] == done_name:
            if not starts:
                raise ValueError(f"unpaired {done_name}")
            intervals.append((starts.pop(0), event))
    if starts:
        raise ValueError(f"unpaired {start_name}")
    return intervals


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", type=Path,
                        default=OUT / "capture" / "commands.jsonl")
    parser.add_argument("--report", type=Path,
                        default=OUT / "reports" / "cycle.json")
    parser.add_argument("--expect-controller", choices=("contract", "upstream"),
                        default="contract")
    args = parser.parse_args()

    capture_path = args.capture
    if not capture_path.is_absolute():
        capture_path = (HERE.parent / capture_path).resolve()
    report_path = args.report
    if not report_path.is_absolute():
        report_path = (HERE.parent / report_path).resolve()
    if not capture_path.is_file():
        parser.error(f"capture does not exist: {capture_path}")

    events = load_events(capture_path)
    timeline = Timeline(events)
    meta = timeline.meta
    counts = collections.Counter(event["e"] for event in events)
    commands = [event for event in events if event["e"] == "cmd"]
    report = {
        "schema": REPORT_SCHEMA,
        "capture": {
            "path": str(capture_path),
            "sha256": _sha256(capture_path),
            "controller_mode": meta.get("controller_mode"),
            "capture_schema": meta.get("capture_schema"),
            "winder_commit": meta.get("winder_commit"),
        },
        "checks": {}, "counts": dict(counts), "axis_commands": {},
        "winding_starts": [], "shaft_wraps": [], "fail": [],
        "requirements_discrepancies": [],
    }
    upstream_identity = None

    _check(report, "capture schema", meta.get("capture_schema") == 4,
           meta.get("capture_schema"), "4")
    _check(report, "upstream commit recorded", bool(meta.get("winder_commit")),
           meta.get("winder_commit"), "git commit hash")
    controller_mode = meta.get("controller_mode")
    _check(report, "controller mode", controller_mode == args.expect_controller,
           controller_mode, args.expect_controller)
    if args.expect_controller == "contract":
        adapter = HERE / "controller_adapter.py"
        current_adapter_hash = hashlib.sha256(adapter.read_bytes()).hexdigest()
        _check(report, "controller adapter hash current",
               meta.get("controller_adapter_sha256") == current_adapter_hash,
               meta.get("controller_adapter_sha256"), current_adapter_hash)
        plan = meta.get("winding_plan")
        _check(report, "validated winding plan captured",
               isinstance(plan, dict)
               and plan.get("schema") == "slot-winding-plan/v1"
               and plan.get("transition_status") == "PASS"
               and plan.get("turns_per_tooth") == meta.get("turns")
               and plan.get("placement_count") == meta.get("turns")
               and plan.get("half_turn_center_count")
               == 2 * meta.get("turns", 0),
               plan,
               "PASS slot-winding-plan/v1 with 50/100 centers")
        if isinstance(plan, dict) and plan.get("path"):
            plan_path = Path(plan["path"])
            current_plan_hash = (
                hashlib.sha256(plan_path.read_bytes()).hexdigest()
                if plan_path.is_file() else None)
            _check(report, "winding plan hash current",
                   current_plan_hash == plan.get("sha256"),
                   plan.get("sha256"), current_plan_hash)
        _check(report, "nominal and model wire envelopes captured",
               isinstance(plan, dict)
               and plan.get("nominal_wire_mm") == 0.22352
               and plan.get("model_wire_envelope_mm") == 0.22352,
               ({"nominal_wire_mm": plan.get("nominal_wire_mm"),
                 "model_wire_envelope_mm":
                     plan.get("model_wire_envelope_mm")}
                if isinstance(plan, dict) else plan),
               "nominal = model = 0.22352 mm")
        _check(report, "receiving maximum wire envelope captured",
               isinstance(plan, dict)
               and plan.get("receiving_sensitivity_wire_envelope_mm") == 0.235
               and plan.get("receiving_sensitivity_status") == "PASS",
               ({"wire_envelope_mm":
                     plan.get("receiving_sensitivity_wire_envelope_mm"),
                 "status": plan.get("receiving_sensitivity_status")}
                if isinstance(plan, dict) else plan),
               "0.235 mm receiving maximum sensitivity PASS")
    settings_path = OUT / "settings.yml"
    current_settings_hash = (_sha256(settings_path)
                             if settings_path.is_file() else None)
    _check(report, "settings hash current",
           meta.get("settings_sha256") == current_settings_hash,
           meta.get("settings_sha256"), current_settings_hash)
    turns = meta.get("turns")
    settings_turns = meta.get("settings_turns")
    teeth_count = meta.get("teeth_count")
    configured_teeth = (
        int(teeth_count) if isinstance(teeth_count, int) and teeth_count > 0
        else 0
    )
    _check(report, "positive integer turns configured",
           type(turns) is int and turns > 0, turns, "> 0")
    _check(report, "capture turns match settings",
           turns == settings_turns, turns, settings_turns)
    job = meta.get("job")
    _check(report, "physical job contract captured",
           isinstance(job, dict)
           and float(job.get("wire_finished_d_mm", 0.0)) > 0.0
           and len(job.get("radial_winding_span_mm", [])) == 2,
           job, "settings.yml job with wire diameter and radial span")
    _check(report, "capture tooth count matches job slots",
           configured_teeth > 0 and isinstance(job, dict)
           and int(job.get("slots", 0)) == configured_teeth,
           {"capture_teeth": teeth_count,
            "job_slots": job.get("slots") if isinstance(job, dict) else None},
           "same positive configured slot count")
    _check(report, "cycle completion marker", counts["cycle_complete"] == 1,
           counts["cycle_complete"], "1")
    _check(report, "three phases", counts["wind"] == 3, counts["wind"], "3")
    _check(report, "configured tooth passes",
           configured_teeth > 0
           and counts["wind_wire"] == configured_teeth,
           counts["wind_wire"], str(configured_teeth))
    if configured_teeth == 24:
        _check(report, "24 tooth passes", counts["wind_wire"] == 24,
               counts["wind_wire"], "24")
    _check(report, "two shaft-wrap calls", counts["wind_wire_around_shaft"] == 2,
           counts["wind_wire_around_shaft"], "2")

    shaft_contract = meta.get("shaft_wrap_contract")
    if args.expect_controller == "contract":
        refine_path = (Path(shaft_contract["refinement_report"])
                       if isinstance(shaft_contract, dict)
                       and shaft_contract.get("refinement_report") else None)
        refine_hash = (
            hashlib.sha256(refine_path.read_bytes()).hexdigest()
            if refine_path is not None and refine_path.is_file() else None
        )
        shaft_contract_ok = (
            isinstance(shaft_contract, dict)
            and abs(float(shaft_contract.get(
                "m0_park_rad", math.nan))) <= 1e-12
            and abs(float(shaft_contract.get(
                "m2_park_phase_rad", math.nan))
                    - math.pi / 4.0) <= 1e-12
            and shaft_contract.get("refinement_status") == "PASS"
            and float(shaft_contract.get(
                "residual_clearance_after_budget_mm", -math.inf)) > 0.0
            and shaft_contract.get("refinement_report_sha256") == refine_hash
        )
        _check(
            report, "shaft-wrap park bound to current PASS refinement",
            shaft_contract_ok, shaft_contract,
            "M0=0, M2=45 deg and current positive-residual refinement report",
        )
    else:
        upstream_identity = _upstream_checkout_identity(meta)
        raw_identity = {
            "controller_adapter_sha256": meta.get(
                "controller_adapter_sha256"),
            "winding_plan": meta.get("winding_plan"),
            "shaft_wrap_contract": shaft_contract,
        }
        _check(
            report, "capture is untouched upstream rather than adapter output",
            meta.get("controller_adapter_sha256") is None
            and meta.get("winding_plan") is None
            and shaft_contract is None,
            raw_identity,
            "no controller adapter, injected packing plan, or wrap contract",
        )
        _check(
            report, "current upstream checkout matches captured clean commit",
            upstream_identity["same_clean_commit_as_capture"] is True,
            upstream_identity,
            "clean checkout at the exact capture commit with src/winding.py hashed",
        )
        report["upstream_source"] = upstream_identity

    tooth_ids = [int(event["args"][0]) for event in events
                 if event["e"] == "wind_wire"]
    _check(report, "all teeth visited once",
           configured_teeth > 0
           and sorted(tooth_ids) == list(range(configured_teeth)),
           tooth_ids, f"each tooth 0..{configured_teeth - 1} exactly once")

    origins = [event for event in events
               if event["e"] == "packing_pass_origin"]
    packing_points = [event for event in events
                      if event["e"] == "packing_waypoint"]
    phase_proof_ok = configured_teeth > 0 and len(origins) == configured_teeth
    phase_rows = []
    for pass_index in range(configured_teeth):
        origin_rows = [row for row in origins
                       if row["pass_index"] == pass_index]
        centers = [row for row in packing_points
                   if row["pass_index"] == pass_index
                   and row.get("kind") == "placement_center"]
        all_points = [row for row in packing_points
                      if row["pass_index"] == pass_index]
        if len(origin_rows) != 1:
            phase_proof_ok = False
            continue
        origin = origin_rows[0]
        first = float(origin["first_crossing_phase_rad"])
        start = float(origin["start_phase_rad"])
        expected_first = (0.0 if start <= 1e-6
                          else math.ceil((start - 1e-9) / math.pi) * math.pi)
        placement_counts = collections.Counter(
            row.get("placement_index") for row in centers)
        exact_phases = (
            len(centers) == 2 * int(turns or 0)
            and all(abs(float(row["m2_phase_rad"])
                        - (first + index * math.pi)) <= 1e-6
                    for index, row in enumerate(centers))
        )
        last_at_target = (
            bool(all_points)
            and abs(float(all_points[-1]["m2_phase_rad"])
                    - float(origin["actual_travel_rad"])) <= 1e-6
            and all_points[-1].get("placement_index") == 49
        )
        pass_ok = (
            abs(first - expected_first) <= 1e-6
            and origin.get("pre_crossing_deposition_count") == 0
            and exact_phases
            and placement_counts == collections.Counter({
                i: 2 for i in range(int(turns or 0))
            })
            and all(abs(float(row["m0_error_rad"])) <= 0.02
                    for row in centers)
            and last_at_target
        )
        phase_proof_ok &= pass_ok
        phase_rows.append({
            "pass_index": pass_index,
            "start_phase_rad": start,
            "first_crossing_phase_rad": first,
            "center_count": len(centers),
            "last_at_target": last_at_target,
            "ok": pass_ok,
        })
    origin_classes = {round(float(row["start_phase_rad"]), 6)
                      for row in origins}
    phase_proof_ok &= 0.0 in origin_classes and 1.0 in origin_classes
    report["packing_phase_proof"] = phase_rows
    _check(report, "all configured slot-side placements phase-locked",
           phase_proof_ok,
           {"origin_classes_rad": sorted(origin_classes),
            "center_count": sum(row["center_count"] for row in phase_rows)},
           f"{configured_teeth} passes x {2 * int(turns or 0)} centers; "
           f"each placement 0..{int(turns or 0) - 1} exactly twice")
    if configured_teeth == 24 and int(turns or 0) == 50:
        _check(report, "all 2400 slot-side placements phase-locked",
               phase_proof_ok,
               {"origin_classes_rad": sorted(origin_classes),
                "center_count": sum(
                    row["center_count"] for row in phase_rows)},
               "24 passes x 100 centers; each placement 0..49 exactly twice")

    if args.expect_controller == "upstream":
        # Packing waypoints are diagnostic adapter events and must not be
        # required (or silently treated as raw evidence) in the untouched
        # upstream lane.  Replace that adapter-only check with physical turn
        # crossings and the captured M0 ease-law extrema for every pass.
        report["checks"].pop(
            "all 2400 slot-side placements phase-locked", None)
        report["checks"].pop(
            "all configured slot-side placements phase-locked", None)
        report["fail"] = [name for name in report["fail"]
                          if name not in {
                              "all 2400 slot-side placements phase-locked",
                              "all configured slot-side placements phase-locked",
                          }]
        raw_windows = winding_windows(events)
        raw_progression = []
        raw_progression_ok = (
            configured_teeth > 0 and len(raw_windows) == configured_teeth
        )
        expected_range = sorted(float(value)
                                for value in meta["m0_wind_range"])
        for window in raw_windows:
            intervals = _revolution_intervals(
                timeline.axes[2], window["motionStart"], window["end"],
                int(turns or 0))
            if len(intervals) == int(turns or 0) and intervals:
                lay_end = intervals[-1][1]
                m0_min, m0_max = _track_extrema(
                    timeline.axes[0], window["motionStart"], lay_end)
            else:
                lay_end = math.nan
                m0_min = m0_max = math.nan
            pass_ok = (
                len(intervals) == int(turns or 0)
                and abs(m0_min - expected_range[0]) <= 0.02
                and abs(m0_max - expected_range[1]) <= 0.02
            )
            raw_progression_ok &= pass_ok
            raw_progression.append({
                "phase": window["phase"],
                "pass_index": window["passIndex"],
                "tooth": window["tooth"],
                "motion_start_t": window["motionStart"],
                "fiftieth_turn_t": lay_end,
                "completed_turns": len(intervals),
                "m0_min_rad": m0_min,
                "m0_max_rad": m0_max,
                "ok": pass_ok,
            })
        report["raw_winding_progression"] = raw_progression
        report["packing_phase_proof"] = {
            "applicable": False,
            "reason": (
                "packing_pass_origin/packing_waypoint are adapter-only; raw "
                "authority is physical M2 turn crossings plus captured M0"
            ),
        }
        _check(
            report, "all raw passes complete configured turns across the full M0 span",
            raw_progression_ok,
            {
                "passes": len(raw_progression),
                "turn_counts": sorted(set(
                    row["completed_turns"] for row in raw_progression)),
                "m0_min_rad": min((row["m0_min_rad"]
                                   for row in raw_progression),
                                  default=math.nan),
                "m0_max_rad": max((row["m0_max_rad"]
                                   for row in raw_progression),
                                  default=math.nan),
            },
            (f"{configured_teeth} passes x {turns} physical turns; each reaches M0 range "
             f"[{expected_range[0]}, {expected_range[1]}] rad"),
        )
        if configured_teeth == 24 and int(turns or 0) == 50:
            _check(
                report,
                "all raw passes complete 50 turns across the full M0 span",
                raw_progression_ok,
                {
                    "passes": len(raw_progression),
                    "turn_counts": sorted(set(
                        row["completed_turns"] for row in raw_progression)),
                    "m0_min_rad": min((row["m0_min_rad"]
                                       for row in raw_progression),
                                      default=math.nan),
                    "m0_max_rad": max((row["m0_max_rad"]
                                       for row in raw_progression),
                                      default=math.nan),
                },
                (f"24 passes x {turns} physical turns; each reaches M0 range "
                 f"[{expected_range[0]}, {expected_range[1]}] rad"),
            )

    wind_range = sorted(float(value) for value in meta["m0_wind_range"])
    windows = winding_windows(events)
    winding_start_ok = configured_teeth > 0 and len(windows) == configured_teeth
    for window in windows:
        m0 = timeline.axes[0].pos_at(window["motionStart"])
        ok = wind_range[0] - 1e-6 <= m0 <= wind_range[1] + 1e-6
        winding_start_ok &= ok
        report["winding_starts"].append({
            "phase": window["phase"],
            "pass_index": window["passIndex"],
            "tooth": window["tooth"],
            "motion_start_t": window["motionStart"],
            "m0_rad": m0,
            "ok": ok,
        })
    _check(report, "all winding motion starts inside finite M0 span",
           winding_start_ok,
           [round(item["m0_rad"], 6)
            for item in report["winding_starts"]],
           f"{configured_teeth} starts within "
           f"[{wind_range[0]}, {wind_range[1]}] rad")

    serial_errors = []
    mapping_errors = []
    directions = meta.get("directions")
    gear = float(meta["m2_gear_ratio"])
    for index, command in enumerate(commands):
        match = COMMAND_RE.fullmatch(command.get("command", ""))
        if not match or int(match.group(1)) != command["m"]:
            serial_errors.append(index)
            continue
        parsed = float(match.group(2))
        if abs(parsed - float(command["controller_target"])) > 1e-12:
            serial_errors.append(index)
        expected = float(command["model_target"])
        if not directions or not directions[command["m"]]:
            expected = -expected
        if command["m"] == 2:
            expected *= gear
        expected = round(expected, 3)
        if abs(expected - float(command["controller_target"])) > 1e-12:
            mapping_errors.append(index)
    _check(report, "exact serial command records", not serial_errors,
           serial_errors[:20], "all commands parse and match stored target")
    _check(report, "direction and gear mapping", not mapping_errors,
           mapping_errors[:20], "all controller targets reproduce upstream mapping")

    by_axis = collections.Counter(command["m"] for command in commands)
    report["axis_commands"] = {str(axis): by_axis[axis] for axis in range(4)}
    for axis in range(3):
        _check(report, f"axis M{axis} commanded", by_axis[axis] > 0,
               by_axis[axis], "> 0")
    # Passive M3 is still observable: zero commands show the upstream path
    # explicitly held it inactive rather than silently omitting the axis.
    m3_targets = {float(command["model_target"]) for command in commands
                  if command["m"] == 3}
    _check(report, "passive M3 explicitly zero", m3_targets == {0.0},
           sorted(m3_targets), "{0.0}")

    phase_rows = [event for event in events
                  if event.get("e") == "shaft_wrap_phase"]
    phase_groups = {
        number: [event for event in phase_rows
                 if int(event.get("next_wire_idx", -1)) == number]
        for number in (1, 2)
    }
    required_phases = [
        "prepark_start", "m0_parked", "contact_start", "contact_done",
    ]
    wrap_ok = len(phase_rows) == 8
    reference = (float(shaft_contract.get("machine_m2_reference_rad", 0.0))
                 if isinstance(shaft_contract, dict) else 0.0)
    park_phase = (float(shaft_contract.get("m2_park_phase_rad", math.nan))
                  if isinstance(shaft_contract, dict) else math.nan)
    m0_park = (float(shaft_contract.get("m0_park_rad", math.nan))
               if isinstance(shaft_contract, dict) else math.nan)
    for number in (1, 2):
        rows = phase_groups[number]
        sequence_ok = [row.get("phase") for row in rows] == required_phases
        if sequence_ok:
            start, done = rows[2], rows[3]
            p0 = float(start["m1_rad"])
            p1 = float(done["m1_rad"])
            turns = abs(p1 - p0) / (2.0 * math.pi)
            parked_m0 = abs(float(start["m0_rad"]) - m0_park) <= 0.0035
            parked_m2 = abs(
                ((float(start["m2_rad"]) - reference - park_phase
                  + math.pi) % (2.0 * math.pi)) - math.pi
            ) <= 0.005
            fixed = (
                abs(float(done["m0_rad"]) - float(start["m0_rad"])) <= 1e-9
                and abs(float(done["m2_rad"]) - float(start["m2_rad"]))
                <= 1e-9
            )
        else:
            start = done = {"t": None}
            p0 = p1 = turns = math.nan
            parked_m0 = parked_m2 = fixed = False
        ok = (sequence_ok and abs(turns - 2.0) <= 0.05
              and parked_m0 and parked_m2 and fixed)
        wrap_ok &= ok
        report["shaft_wraps"].append({
            "index": number, "start_t": start["t"], "done_t": done["t"],
            "phase_sequence": [row.get("phase") for row in rows],
            "start_m1": p0, "done_m1": p1, "turns": turns,
            "m0_parked": parked_m0, "m2_parked": parked_m2,
            "m0_m2_fixed_during_contact": fixed, "ok": ok,
        })
    _check(
        report, "two parked physical M1 turns per shaft wrap", wrap_ok,
        report["shaft_wraps"],
        "two complete phase-marker sequences; M0=0 and M2=45 deg fixed "
        "through exactly two M1 turns",
    )

    if args.expect_controller == "upstream":
        report["checks"].pop(
            "two parked physical M1 turns per shaft wrap", None)
        report["fail"] = [name for name in report["fail"]
                          if name !=
                          "two parked physical M1 turns per shaft wrap"]
        controller_events = _controller_effective_events(events)
        controller_timeline = Timeline(controller_events)
        raw_wraps = _raw_shaft_wraps(
            controller_events,
            controller_timeline,
            source_timeline=timeline,
        )
        park_m0 = float(meta.get("m1_rotating_position", math.nan))
        park_m2 = abs(float(meta.get(
            "angle_to_prevent_collision", math.nan)))
        for row in raw_wraps:
            target_complete = (
                row.get("m1_arrived") is True
                and row.get("m0_m2_fixed_during_contact") is True
            )
            pose_ok = (
                target_complete
                and abs(float(row["start_m0"]) - park_m0) <= 0.0035
                and abs(abs(float(row["start_m2"])) - park_m2) <= 0.005
            )
            exactly_two = (
                isinstance(row.get("turns"), (int, float))
                and abs(float(row["turns"]) - 2.0)
                <= SHAFT_WRAP_TURNS_TOL
            )
            row["raw_park_pose_matches_settings"] = pose_ok
            row["raw_target_motion_complete"] = target_complete and pose_ok
            row["exactly_two_physical_turns"] = exactly_two
            row["ok"] = (
                pose_ok
                and row.get("absolute_target_formula_matches_capture") is True
                and exactly_two
            )
        raw_motion_complete = (
            len(raw_wraps) == 2
            and sorted(row.get("index") for row in raw_wraps) == [1, 2]
            and all(row.get("raw_target_motion_complete") is True
                    for row in raw_wraps)
        )
        raw_wrap_contract_ok = (
            raw_motion_complete
            and all(row.get("exactly_two_physical_turns") is True
                    for row in raw_wraps)
        )
        absolute_formula_ok = (
            len(raw_wraps) == 2
            and all(
                row.get("absolute_target_formula_matches_capture") is True
                and row.get("interpretation_verdict")
                == "GENUINE_UPSTREAM_ABSOLUTE_TARGET_CONTRADICTION"
                and str(row.get("serial_command", "")).startswith("M1A")
                and row.get("m1_arrived") is True
                for row in raw_wraps
            )
        )
        quantization_only = (
            absolute_formula_ok
            and all(
                abs(float(row["controller_effective_turns"])
                    - float(row["source_model_turns"])) <= 1.0e-4
                for row in raw_wraps
            )
        )
        report["shaft_wraps"] = raw_wraps
        _check(
            report, "both raw shaft-wrap target intervals physically complete",
            raw_motion_complete, raw_wraps,
            ("two upstream calls; M0/M2 fixed at settings park pose and M1 "
             "arrives before the following M2 command"),
        )
        _check(
            report,
            "raw shaft-wrap absolute-target formula matches command stream",
            absolute_formula_ok,
            raw_wraps,
            (
                "for each wrap, indexed start=m1_zero-2*pi*last/teeth and "
                "emitted M1A target=m1_zero +/- 4*pi"
            ),
        )
        _check(
            report,
            "controller-effective wrap travel differs only by serial quantization",
            quantization_only,
            [{
                "index": row.get("index"),
                "source_model_turns": row.get("source_model_turns"),
                "controller_effective_turns": row.get(
                    "controller_effective_turns"
                ),
            } for row in raw_wraps],
            "absolute source failure reproduced after 3-decimal controller mapping",
        )
        _check(
            report, "both raw shaft-wrap intervals execute exactly two M1 turns",
            raw_wrap_contract_ok, raw_wraps,
            ("two upstream calls; each completed physical M1 displacement is "
             f"2.000 turns within {SHAFT_WRAP_TURNS_TOL:.3f} turn"),
        )
        observed_turns = [row.get("turns") for row in raw_wraps]
        source_turns = [row.get("source_model_turns") for row in raw_wraps]
        report["shaft_wrap_diagnostic"] = {
            "classification": (
                "GENUINE_UPSTREAM_ABSOLUTE_TARGET_CONTRADICTION"
                if absolute_formula_ok
                else "UNRESOLVED_CAPTURE_INTERPRETATION"
            ),
            "source_model_turns": source_turns,
            "controller_effective_turns": observed_turns,
            "serial_quantization_changes_verdict": not quantization_only,
            "absolute_angle_unwrap_defect": False if absolute_formula_ok else None,
            "targets_reached_before_following_command": raw_motion_complete,
            "required_relative_targets_m1_rad": [
                row.get("required_two_turn_relative_target_m1")
                for row in raw_wraps
            ],
            "required_target_corrections_rad": [
                row.get("required_target_correction_rad")
                for row in raw_wraps
            ],
            "actionable_controller_contract": (
                "derive each wrap target from live/current M1 as current_M1 "
                "+/- 4*pi, serialize the absolute target, and confirm arrival; "
                "do not derive it from the bookkeeping m1_zero"
            ),
            "upstream_motion_logic_modified_by_verifier": False,
        }
        if (len(observed_turns) == 2 and any(
                not isinstance(value, (int, float))
                or abs(float(value) - 2.0) > 0.05
                for value in observed_turns)):
            report["requirements_discrepancies"].append({
                "id": "shaft-wrap-relative-two-turn-claim",
                "goal_claim": "each wire-around-shaft maneuver is 2 M1 turns",
                "raw_upstream_source_model_turns": source_turns,
                "raw_upstream_controller_effective_turns": observed_turns,
                "absolute_target_rows": [{
                    "index": row.get("index"),
                    "previous_phase_last_tooth_index": row.get(
                        "previous_phase_last_tooth_index"
                    ),
                    "next_phase_branch": row.get("next_phase_branch"),
                    "current_m1_rad": row.get("start_m1"),
                    "absolute_target_m1_rad": row.get("target_m1"),
                    "required_two_turn_relative_target_m1_rad": row.get(
                        "required_two_turn_relative_target_m1"
                    ),
                    "required_target_correction_rad": row.get(
                        "required_target_correction_rad"
                    ),
                    "source_formula": row.get("source_formula"),
                } for row in raw_wraps],
                "reason": (
                    "upstream emits an absolute M1A target from bookkeeping "
                    "m1_zero; the previous indexed tooth angle therefore changes "
                    "the completed travel. Controller quantization is below "
                    "0.0001 turn and cannot explain or repair the contradiction"
                ),
                "required_upstream_contract": report[
                    "shaft_wrap_diagnostic"
                ]["actionable_controller_contract"],
                "release_policy": (
                    "validate the exact raw swept envelope, including the larger "
                    "one, but fail normal GOAL DoD until upstream emits two "
                    "physical turns for each maneuver"
                ),
            })

    report["timeline_end"] = timeline.t_end
    report["status"] = "PASS" if not report["fail"] else "FAIL"
    report["passed"] = not report["fail"]
    report["source_hashes"] = {
        "sim/verify_cycle.py": _sha256(Path(__file__)),
        "sim/traj.py": _sha256(HERE / "traj.py"),
        "out/settings.yml": current_settings_hash,
    }
    report["report_sha256"] = _canonical_hash(report)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n")

    print("=== captured cycle verification ===")
    for name, result in report["checks"].items():
        print(f"  [{'OK' if result['ok'] else 'FAIL'}] {name}: {result['value']}")
    print("axis commands:", report["axis_commands"])
    print(f"RESULT: {'PASS' if not report['fail'] else 'FAIL'}")
    return 0 if not report["fail"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
