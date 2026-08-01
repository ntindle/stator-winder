"""Fail-closed study of a passive inverse-sine M0 transmission.

The unmodified upstream controller commands an ease-out-sine M0 coordinate
during each 50-turn flyer pass.  This study asks whether a passive cam/linkage
can apply the inverse mapping and make physical tooth insertion linear at every
half-turn.  Candidate settings and captures live below ``out/studies``; the
production settings, canonical capture, controller, and CAD are never edited.

The exact static inverse exists only on the winding interval.  The study also
checks the real polling/settling timeline, serial target rounding, wire-center
overlap, cam curvature and pressure angle, follower/load/backlash sensitivity,
and the larger M0 retract/index domain.  Any missing mechanical state or
unbounded endpoint quantity fails release closed.

Run::

    python sim/passive_m0_arcsine_transmission_study.py --generate-captures
"""

from __future__ import annotations

import argparse
from collections import Counter
import copy
import csv
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from typing import Any, Iterable

import numpy as np
from shapely.geometry import LineString, Point
import yaml


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
WINDER = ROOT.parent / "winder"
for import_path in (HERE, CAD):
    if str(import_path) not in sys.path:
        sys.path.insert(0, str(import_path))

from params import PARAMS  # noqa: E402
import slot_packing_audit  # noqa: E402
from traj import AxisTrack, Timeline, load_events, winding_windows  # noqa: E402


SCHEMA = "passive-m0-arcsine-transmission-study/v1"
POLL_INTERVAL_S = 0.03
UPSTREAM_TRIGGER_ALLOWANCE_RAD = 0.01
PRESSURE_ANGLE_LIMIT_DEG = 30.0
FOLLOWER_RADIUS_MM = 2.0
FOLLOWER_CURVATURE_MARGIN_MM = 0.50
SCREW_EFFICIENCY = 0.50
SERIAL_TARGET_RESOLUTION_RAD = 0.001
OVERLAP_TOLERANCE_MM = 1.0e-6
PITCH_TOLERANCE_MM = 1.0e-6

PRODUCTION_SETTINGS = ROOT / "out" / "settings.yml"
CANONICAL_CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
PACKING_REPORT = ROOT / "out" / "reports" / "slot_packing.json"
LOADS_REPORT = ROOT / "out" / "reports" / "loads.json"
PLACEMENT_TOLERANCE_REPORT = (
    ROOT / "out" / "reports" / "placement_tolerance.json"
)
SLOT_ROUTES_REPORT = ROOT / "out" / "reports" / "slot_wire_routes.json"
STUDY_ROOT = ROOT / "out" / "studies" / "passive_m0_arcsine"
SETTINGS_DIR = STUDY_ROOT / "settings"
CAPTURE_DIR = STUDY_ROOT / "captures"
OUTPUT_JSON = ROOT / "out" / "reports" / "passive_m0_arcsine_transmission.json"
OUTPUT_MD = ROOT / "out" / "reports" / "passive_m0_arcsine_transmission.md"
OUTPUT_CSV = ROOT / "out" / "reports" / "passive_m0_arcsine_half_turns.csv"


@dataclass(frozen=True)
class Candidate:
    label: str
    poll_divisor_n: int
    detune_fraction: float = 0.0

    @property
    def exact_velocity_rad_s(self) -> float:
        return math.pi / (self.poll_divisor_n * POLL_INTERVAL_S)

    @property
    def velocity_rad_s(self) -> float:
        return self.exact_velocity_rad_s * (1.0 + self.detune_fraction)

    @property
    def settings_path(self) -> Path:
        return SETTINGS_DIR / f"{self.label}.yml"

    @property
    def capture_path(self) -> Path:
        return CAPTURE_DIR / f"{self.label}.jsonl"

    @property
    def failure_path(self) -> Path:
        return CAPTURE_DIR / f"{self.label}.failure.txt"


def candidates() -> list[Candidate]:
    """Exact n=4..8 polling ratios plus a local n=6 detune sweep."""

    return [
        Candidate("n4_exact", 4),
        Candidate("n5_exact", 5),
        Candidate("n6_minus_1pct", 6, -0.01),
        Candidate("n6_minus_0p1pct", 6, -0.001),
        Candidate("n6_exact", 6),
        Candidate("n6_plus_0p1pct", 6, 0.001),
        Candidate("n6_plus_1pct", 6, 0.01),
        Candidate("n7_exact", 7),
        Candidate("n8_exact", 8),
    ]


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: dict[str, Any]) -> str:
    copy_payload = dict(payload)
    copy_payload.pop("report_sha256", None)
    encoded = json.dumps(
        copy_payload, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _rel(path: Path) -> str:
    return path.resolve().relative_to(ROOT.resolve()).as_posix()


def _upstream_identity() -> dict[str, Any]:
    commit = subprocess.check_output(
        ["git", "-C", str(WINDER), "rev-parse", "HEAD"], text=True,
    ).strip()
    dirty = subprocess.check_output(
        ["git", "-C", str(WINDER), "status", "--porcelain"], text=True,
    ).strip()
    return {"commit": commit, "clean": not bool(dirty), "status": dirty}


def _candidate_settings_text(base: dict[str, Any], candidate: Candidate) -> str:
    payload = copy.deepcopy(base)
    payload["motor"]["M2"]["velocity"] = candidate.velocity_rad_s
    body = yaml.safe_dump(payload, sort_keys=False, default_flow_style=False)
    return (
        "# STUDY ONLY: passive M0 inverse-sine transmission timing sweep.\n"
        "# Generated from out/settings.yml; production settings are untouched.\n"
        f"# Candidate {candidate.label}; M2={candidate.velocity_rad_s:.12f} rad/s.\n"
        + body
    )


def generate_candidates(*, force: bool = False) -> list[dict[str, Any]]:
    """Write isolated settings copies and raw upstream captures."""

    upstream = _upstream_identity()
    if not upstream["clean"]:
        raise RuntimeError(
            "upstream winder checkout is dirty; cannot claim unmodified capture"
        )
    base_hash_before = _sha256(PRODUCTION_SETTINGS)
    capture_hash_before = _sha256(CANONICAL_CAPTURE)
    base = yaml.safe_load(PRODUCTION_SETTINGS.read_text(encoding="utf-8"))
    if [float(base["motor"][name]["velocity"]) for name in ("M0", "M1")] != [20.0, 20.0]:
        raise RuntimeError("production M0/M1 timing fix is not present")

    SETTINGS_DIR.mkdir(parents=True, exist_ok=True)
    CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
    generated: list[dict[str, Any]] = []
    for candidate in candidates():
        settings_text = _candidate_settings_text(base, candidate)
        candidate.settings_path.write_text(settings_text, encoding="utf-8")
        settings_hash = _sha256(candidate.settings_path)
        reuse = False
        if candidate.capture_path.is_file() and not force:
            existing = load_events(candidate.capture_path)
            meta = next((row for row in existing if row.get("e") == "meta"), {})
            reuse = (
                meta.get("settings_sha256") == settings_hash
                and meta.get("controller_mode") == "upstream"
                and meta.get("controller_adapter_sha256") is None
                and meta.get("winder_commit") == upstream["commit"]
            )
        if not reuse:
            env = dict(os.environ)
            env["WINDER_LOG_LEVEL"] = "WARNING"
            command = [
                sys.executable,
                str(HERE / "capture.py"),
                "--settings", str(candidate.settings_path),
                "--winder", str(WINDER),
                "--controller", "upstream",
                "--output", str(candidate.capture_path),
            ]
            result = subprocess.run(
                command, cwd=ROOT, env=env, text=True,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                check=False,
            )
            print(result.stdout.strip())
            if result.returncode != 0:
                candidate.failure_path.write_text(
                    result.stdout, encoding="utf-8")
                generated.append({
                    "label": candidate.label,
                    "settings": _rel(candidate.settings_path),
                    "settings_sha256": settings_hash,
                    "capture": None,
                    "capture_sha256": None,
                    "status": "FAIL",
                    "failure_log": _rel(candidate.failure_path),
                    "failure_log_sha256": _sha256(candidate.failure_path),
                    "returncode": result.returncode,
                    "reused_capture": False,
                })
                continue
        generated.append({
            "label": candidate.label,
            "settings": _rel(candidate.settings_path),
            "settings_sha256": settings_hash,
            "capture": _rel(candidate.capture_path),
            "capture_sha256": _sha256(candidate.capture_path),
            "status": "PASS",
            "reused_capture": reuse,
        })

    if _sha256(PRODUCTION_SETTINGS) != base_hash_before:
        raise RuntimeError("production settings changed during candidate generation")
    if _sha256(CANONICAL_CAPTURE) != capture_hash_before:
        raise RuntimeError("canonical capture changed during candidate generation")
    return generated


def _crossing_times(track: Any, start_t: float, start_pos: float,
                    direction: int, count: int) -> list[float]:
    """Invert a piecewise-linear axis track at directed pi crossings."""

    result: list[float] = []
    for index in range(count):
        target = start_pos + direction * index * math.pi
        if index == 0:
            result.append(start_t)
            continue
        found = None
        for (left_t, _), (right_t, right_p) in zip(
            track.knots, track.knots[1:]
        ):
            if right_t < start_t - 1.0e-12:
                continue
            local_left_t = max(left_t, start_t)
            local_left_p = track.pos_at(local_left_t)
            if direction * (right_p - local_left_p) < -1.0e-10:
                continue
            if ((target - local_left_p) * (target - right_p) <= 1.0e-10
                    and abs(right_p - local_left_p) > 1.0e-12):
                found = local_left_t + (
                    (right_t - local_left_t)
                    * (target - local_left_p)
                    / (right_p - local_left_p)
                )
                break
        if found is None:
            break
        result.append(float(found))
    return result


def inverse_sine_radius(m0_rad: float, wind_range: Iterable[float],
                        radial_range: Iterable[float]) -> float:
    """Map upstream sine coordinate to linear physical insertion radius."""

    q0, q1 = map(float, wind_range)
    r0, r1 = map(float, radial_range)
    u = (float(m0_rad) - q0) / (q1 - q0)
    if u < -1.0e-8 or u > 1.0 + 1.0e-8:
        raise ValueError(f"M0={m0_rad:.9f} rad is outside inverse-sine domain")
    u = min(1.0, max(0.0, u))
    return r0 + (r1 - r0) * (2.0 / math.pi) * math.asin(u)


def _serial_m0_track(events: list[dict[str, Any]],
                     meta: dict[str, Any]) -> AxisTrack:
    """Reconstruct the rounded serial M0 coordinate a real driver receives."""

    track = AxisTrack(float(meta["velocities"][0]))
    direction_sign = 1.0 if bool(meta["directions"][0]) else -1.0
    for row in events:
        if row.get("e") == "cmd" and int(row.get("m", -1)) == 0:
            target = direction_sign * float(row["controller_target"])
            track.command(float(row["t"]), target)
    return track


def _pairwise_overlap(values: list[float], wire_d: float) -> dict[str, Any]:
    pairs: list[tuple[int, int, float]] = []
    duplicates = 0
    minimum = math.inf
    for left in range(len(values)):
        for right in range(left + 1, len(values)):
            distance = abs(values[right] - values[left])
            minimum = min(minimum, distance)
            if distance < wire_d - 1.0e-12:
                pairs.append((left, right, distance))
            if distance <= OVERLAP_TOLERANCE_MM:
                duplicates += 1
    return {
        "wire_count": len(values),
        "minimum_pair_center_distance_mm": minimum,
        "pairs_below_finished_wire_diameter": len(pairs),
        "exact_duplicate_pairs": duplicates,
        "first_overlap_witness": (
            {"wire_indices": [pairs[0][0], pairs[0][1]],
             "center_distance_mm": pairs[0][2]}
            if pairs else None
        ),
    }


def _summarize_pitches(intervals: list[dict[str, Any]], key: str,
                       ideal_pitch: float) -> dict[str, Any]:
    pitches = [float(row[key]) for row in intervals]
    errors = [abs(value - ideal_pitch) for value in pitches]
    return {
        "interval_count": len(pitches),
        "minimum_abs_pitch_mm": min(pitches),
        "maximum_abs_pitch_mm": max(pitches),
        "mean_abs_pitch_mm": sum(pitches) / len(pitches),
        "maximum_abs_pitch_error_mm": max(errors),
        "rms_pitch_error_mm": math.sqrt(
            sum(value * value for value in errors) / len(errors)
        ),
        "zero_pitch_interval_count": sum(
            value <= OVERLAP_TOLERANCE_MM for value in pitches
        ),
        "within_1um_of_ideal_count": sum(
            value <= PITCH_TOLERANCE_MM for value in errors
        ),
        "all_intervals_linear_within_1um": all(
            value <= PITCH_TOLERANCE_MM for value in errors
        ),
    }


def _capture_identity(events: list[dict[str, Any]], candidate: Candidate,
                      capture_path: Path) -> tuple[dict[str, Any], Timeline]:
    meta = next(row for row in events if row.get("e") == "meta")
    if meta.get("capture_schema") != 4:
        raise RuntimeError(f"{candidate.label}: capture schema is not v4")
    if meta.get("controller_mode") != "upstream":
        raise RuntimeError(f"{candidate.label}: controller is not upstream")
    if meta.get("controller_adapter_sha256") is not None:
        raise RuntimeError(f"{candidate.label}: adapter-backed capture rejected")
    if meta.get("winding_plan") is not None:
        raise RuntimeError(f"{candidate.label}: raw upstream capture has a plan")
    if int(meta.get("turns", -1)) != 50 or int(meta.get("teeth_count", -1)) != 24:
        raise RuntimeError(f"{candidate.label}: expected 24 slots x 50 turns")
    actual_v = float(meta["velocities"][2])
    if not math.isclose(actual_v, candidate.velocity_rad_s, abs_tol=1.0e-12):
        raise RuntimeError(f"{candidate.label}: M2 velocity drifted")
    if meta.get("settings_sha256") != _sha256(candidate.settings_path):
        raise RuntimeError(f"{candidate.label}: settings hash mismatch")
    if _sha256(capture_path) != _sha256(candidate.capture_path):
        raise RuntimeError(f"{candidate.label}: capture path mismatch")
    return meta, Timeline(events)


def analyze_capture(candidate: Candidate, packing: dict[str, Any],
                    *, include_intervals: bool = False) -> dict[str, Any]:
    events = load_events(candidate.capture_path)
    meta, timeline = _capture_identity(events, candidate, candidate.capture_path)
    windows = winding_windows(events)
    if len(windows) != 24:
        raise RuntimeError(f"{candidate.label}: expected 24 wind passes")

    job = meta["job"]
    wind_range = [float(value) for value in meta["m0_wind_range"]]
    radial_range = [float(value) for value in job["radial_winding_span_mm"]]
    wire_d = float(job["wire_finished_d_mm"])
    ideal_pitch = (radial_range[1] - radial_range[0]) / 50.0
    serial_track = _serial_m0_track(events, meta)
    all_intervals: list[dict[str, Any]] = []
    pass_rows: list[dict[str, Any]] = []
    command_phase_errors: list[float] = []
    stale_sample_phase_errors: list[float] = []
    settle_margins: list[float] = []
    updates_per_pass: list[int] = []
    all_overlaps: list[dict[str, Any]] = []
    retract_rows: list[dict[str, Any]] = []

    for pass_index, window in enumerate(windows):
        direction = 1 if bool(window["clockwise"]) else -1
        start_t = float(window["motionStart"])
        start_m2 = timeline.axes[2].pos_at(start_t)
        crossings = _crossing_times(
            timeline.axes[2], start_t, start_m2, direction, 101,
        )
        if len(crossings) != 101:
            raise RuntimeError(
                f"{candidate.label} pass {pass_index}: "
                f"only {len(crossings)} of 101 half-turn states"
            )

        states: list[dict[str, Any]] = []
        for half_index, time_s in enumerate(crossings):
            model_q = timeline.axes[0].pos_at(time_s)
            serial_q = serial_track.pos_at(time_s)
            ideal_norm = 2.0 * min(half_index / 100.0,
                                   1.0 - half_index / 100.0)
            states.append({
                "half_turn_index": half_index,
                "time_s": time_s,
                "m0_model_rad": model_q,
                "m0_serial_rad": serial_q,
                "physical_radius_model_mm": inverse_sine_radius(
                    model_q, wind_range, radial_range),
                "physical_radius_serial_mm": inverse_sine_radius(
                    serial_q, wind_range, radial_range),
                "ideal_linear_radius_mm": (
                    radial_range[0]
                    + (radial_range[1] - radial_range[0]) * ideal_norm
                ),
            })

        intervals: list[dict[str, Any]] = []
        for half_index in range(1, 101):
            previous = states[half_index - 1]
            current = states[half_index]
            row = {
                "pass_index": pass_index,
                "tooth": int(window["tooth"]),
                "clockwise": bool(window["clockwise"]),
                "half_turn_index": half_index,
                "time_s": current["time_s"],
                "m0_model_rad": current["m0_model_rad"],
                "m0_serial_rad": current["m0_serial_rad"],
                "physical_radius_model_mm": current[
                    "physical_radius_model_mm"],
                "physical_radius_serial_mm": current[
                    "physical_radius_serial_mm"],
                "model_abs_pitch_mm": abs(
                    current["physical_radius_model_mm"]
                    - previous["physical_radius_model_mm"]),
                "serial_abs_pitch_mm": abs(
                    current["physical_radius_serial_mm"]
                    - previous["physical_radius_serial_mm"]),
                "ideal_abs_pitch_mm": ideal_pitch,
            }
            intervals.append(row)
            all_intervals.append(row)

        winding_commands = [
            row for row in events
            if row.get("e") == "cmd" and int(row.get("m", -1)) == 0
            and start_t - 1.0e-9 <= float(row["t"]) <= crossings[-1] + 1.0e-9
            and wind_range[0] - 1.0e-9
            <= float(row["model_target"]) <= wind_range[1] + 1.0e-9
        ]
        command_rows: list[dict[str, Any]] = []
        for command in winding_commands:
            command_t = float(command["t"])
            travelled_half_turns = (
                direction
                * (timeline.axes[2].pos_at(command_t) - start_m2)
                / math.pi
            )
            nearest_half_turn = round(travelled_half_turns)
            phase_error = travelled_half_turns - nearest_half_turn
            command_phase_errors.append(abs(phase_error))
            sampled_m2 = timeline.axes[2].pos_at(
                command_t - POLL_INTERVAL_S)
            sampled_half_turn = (
                direction * (sampled_m2 - start_m2) / math.pi
            )
            stale_sample_error = sampled_half_turn - round(sampled_half_turn)
            stale_sample_phase_errors.append(abs(stale_sample_error))
            pre_m0 = timeline.axes[0].pos_at(command_t)
            target_m0 = float(command["model_target"])
            arrival_t = command_t + abs(target_m0 - pre_m0) / float(
                meta["velocities"][0]
            )
            next_crossing = next(
                (value for value in crossings if value > command_t + 1.0e-9),
                None,
            )
            margin = (next_crossing - arrival_t
                      if next_crossing is not None else math.nan)
            if math.isfinite(margin):
                settle_margins.append(margin)
            command_rows.append({
                "time_s": command_t,
                "half_turn_coordinate": travelled_half_turns,
                "nearest_half_turn": nearest_half_turn,
                "phase_error_half_turn": phase_error,
                "stale_sample_half_turn": sampled_half_turn,
                "stale_sample_phase_error_half_turn": stale_sample_error,
                "target_m0_rad": target_m0,
                "m0_arrival_s": arrival_t,
                "settle_margin_before_next_half_turn_s": margin,
            })
        updates_per_pass.append(len(command_rows))

        side_centers = {
            "odd_half_turn_side": [
                states[index]["physical_radius_serial_mm"]
                for index in range(1, 101, 2)
            ],
            "even_half_turn_side": [
                states[index]["physical_radius_serial_mm"]
                for index in range(2, 101, 2)
            ],
        }
        overlaps = {
            name: _pairwise_overlap(values, wire_d)
            for name, values in side_centers.items()
        }
        all_overlaps.extend(overlaps.values())

        retract_command = next(
            row for row in events
            if row.get("e") == "cmd" and int(row.get("m", -1)) == 0
            and crossings[-1] < float(row["t"]) <= float(window["end"])
            and math.isclose(
                float(row["model_target"]), float(meta["m1_rotating_position"]),
                abs_tol=1.0e-6,
            )
        )
        retract_t = float(retract_command["t"])
        retract_pre = timeline.axes[0].pos_at(retract_t)
        retract_target = float(retract_command["model_target"])
        retract_arrival = retract_t + abs(retract_target - retract_pre) / float(
            meta["velocities"][0]
        )
        retract_rows.append({
            "pass_index": pass_index,
            "command_time_s": retract_t,
            "start_m0_rad": retract_pre,
            "target_m0_rad": retract_target,
            "motor_motion_time_s": retract_arrival - retract_t,
            "available_until_pass_done_s": float(window["end"]) - retract_t,
            "motor_settle_margin_s": float(window["end"]) - retract_arrival,
        })

        pass_rows.append({
            "pass_index": pass_index,
            "tooth": int(window["tooth"]),
            "clockwise": bool(window["clockwise"]),
            "motion_start_s": start_t,
            "motion_end_s": crossings[-1],
            "winding_duration_s": crossings[-1] - start_t,
            "m0_update_count": len(command_rows),
            "first_update_half_turn": command_rows[0]["half_turn_coordinate"],
            "last_update_half_turn": command_rows[-1]["half_turn_coordinate"],
            "maximum_update_phase_error_half_turn": max(
                abs(row["phase_error_half_turn"]) for row in command_rows
            ),
            "minimum_settle_margin_before_next_half_turn_s": min(
                row["settle_margin_before_next_half_turn_s"]
                for row in command_rows
                if math.isfinite(row[
                    "settle_margin_before_next_half_turn_s"])
            ),
            "model_pitch": _summarize_pitches(
                intervals, "model_abs_pitch_mm", ideal_pitch),
            "serial_pitch": _summarize_pitches(
                intervals, "serial_abs_pitch_mm", ideal_pitch),
            "side_overlap": overlaps,
        })

    serial_summary = _summarize_pitches(
        all_intervals, "serial_abs_pitch_mm", ideal_pitch,
    )
    model_summary = _summarize_pitches(
        all_intervals, "model_abs_pitch_mm", ideal_pitch,
    )
    command_counts = Counter(
        int(row["m"]) for row in events if row.get("e") == "cmd"
    )
    all_serial_radii = [
        float(row["physical_radius_serial_mm"]) for row in all_intervals
    ]
    result = {
        "label": candidate.label,
        "poll_divisor_n": candidate.poll_divisor_n,
        "detune_fraction": candidate.detune_fraction,
        "m2_velocity_rad_s": candidate.velocity_rad_s,
        "m2_velocity_rpm": candidate.velocity_rad_s * 60.0 / (2.0 * math.pi),
        "m2_per_poll_rad": candidate.velocity_rad_s * POLL_INTERVAL_S,
        "settings": {
            "path": _rel(candidate.settings_path),
            "sha256": _sha256(candidate.settings_path),
        },
        "capture": {
            "path": _rel(candidate.capture_path),
            "sha256": _sha256(candidate.capture_path),
            "controller_mode": meta["controller_mode"],
            "controller_adapter_sha256": meta["controller_adapter_sha256"],
            "winder_commit": meta["winder_commit"],
            "virtual_duration_s": max(float(row["t"]) for row in events),
            "command_count": sum(command_counts.values()),
            "command_counts": {
                f"M{axis}": command_counts.get(axis, 0) for axis in range(4)
            },
        },
        "pass_count": len(pass_rows),
        "motion_sign_counts": dict(Counter(
            1 if row["clockwise"] else -1 for row in pass_rows
        )),
        "half_turn_interval_count": len(all_intervals),
        "ideal_half_turn_pitch_mm": ideal_pitch,
        "model_coordinate_pitch": model_summary,
        "serial_coordinate_pitch": serial_summary,
        "poll_alignment": {
            "maximum_abs_stale_sample_phase_error_half_turn": max(
                stale_sample_phase_errors),
            "stale_poll_samples_half_turn_aligned": max(
                stale_sample_phase_errors) <= 1.0e-6,
            "maximum_abs_command_issue_phase_error_half_turn": max(
                command_phase_errors),
            "commands_physically_issued_at_half_turns": max(
                command_phase_errors) <= 1.0e-6,
            "sample_to_command_sleep_s": POLL_INTERVAL_S,
            "n6_exact_sleep_motion_half_turn": (
                candidate.velocity_rad_s * POLL_INTERVAL_S / math.pi),
            "minimum_m0_settle_margin_before_next_half_turn_s": min(
                settle_margins),
            "all_updates_settle_before_next_half_turn": all(
                value >= -1.0e-12 for value in settle_margins
            ),
            "m0_updates_per_pass_range": [
                min(updates_per_pass), max(updates_per_pass)],
            "first_update_after_fast_winding_prelude": all(
                row["first_update_half_turn"] >= 2.0 - 1.0e-6
                for row in pass_rows
            ),
        },
        "wire_overlap": {
            "assumption": (
                "cam controls only radial insertion; absent another proven "
                "tangential mechanism, wires on each tooth side share one "
                "tangential locus"
            ),
            "minimum_same_side_pair_center_distance_mm": min(
                row["minimum_pair_center_distance_mm"]
                for row in all_overlaps
            ),
            "total_same_side_pairs_below_wire_diameter": sum(
                row["pairs_below_finished_wire_diameter"]
                for row in all_overlaps
            ),
            "total_exact_duplicate_pairs": sum(
                row["exact_duplicate_pairs"] for row in all_overlaps
            ),
            "all_centers_inside_radial_bounds": (
                min(all_serial_radii) >= radial_range[0] - 1.0e-9
                and max(all_serial_radii) <= radial_range[1] + 1.0e-9
            ),
            "observed_radial_range_mm": [
                min(all_serial_radii), max(all_serial_radii)],
            "allowed_radial_range_mm": radial_range,
            "wire_finished_diameter_mm": wire_d,
            "authoritative_slot_plan_requires_2d_centers": (
                int(packing["selected_schedule"]["turns_per_tooth"]) == 50
                and int(packing["selected_schedule"]["centers_per_slot"]) == 100
            ),
        },
        "retract_motor_timing": {
            "all_motor_targets_arrive_before_pass_done": all(
                row["motor_settle_margin_s"] >= -1.0e-12
                for row in retract_rows
            ),
            "minimum_motor_settle_margin_s": min(
                row["motor_settle_margin_s"] for row in retract_rows
            ),
            "rows": retract_rows if include_intervals else None,
        },
        "passes": pass_rows if include_intervals else None,
        "half_turn_intervals": all_intervals if include_intervals else None,
    }
    return result


def _cam_mechanics(meta: dict[str, Any], loads: dict[str, Any],
                   placement: dict[str, Any]) -> dict[str, Any]:
    q0, q1 = map(float, meta["m0_wind_range"])
    r0, r1 = map(float, meta["job"]["radial_winding_span_mm"])
    input_stroke = (q1 - q0) * float(PARAMS.mm_per_rad)
    output_stroke = r1 - r0
    amplitude = 2.0 * output_stroke / (math.pi * input_stroke)
    minimum_pressure = math.degrees(math.atan(amplitude))
    curvature_radius = input_stroke * amplitude * amplitude
    half_pitch = output_stroke / 50.0
    pre_endpoint_u = math.sin(0.49 * math.pi)
    final_input_step = input_stroke * (1.0 - pre_endpoint_u)
    final_average_ratio = half_pitch / final_input_step
    final_average_pressure = math.degrees(math.atan(final_average_ratio))
    output_force = float(loads["m0"]["axial_force_n"])
    input_force = output_force * final_average_ratio
    screw_torque = (
        input_force * (float(PARAMS.m0_lead) / 1000.0)
        / (2.0 * math.pi * SCREW_EFFICIENCY)
    )
    selected_dynamic_torque = (
        float(loads["m0"]["t_required_nm"])
        * float(loads["m0"]["margin"])
    )
    coupling_capacity = float(
        loads["m0_m1_couplings"]["dynamic_reversing_capacity_nm"]
    )
    input_speed = float(meta["velocities"][0]) * float(PARAMS.mm_per_rad)
    final_average_output_speed = input_speed * final_average_ratio

    def endpoint_shortfall(input_error_mm: float) -> float:
        u = max(0.0, 1.0 - input_error_mm / input_stroke)
        return output_stroke * (
            1.0 - (2.0 / math.pi) * math.asin(u)
        )

    encoder_half_count = float(
        placement["axis_contract"]["encoder_half_count_quantization_mm"]
    )
    serial_half_quantum = (
        SERIAL_TARGET_RESOLUTION_RAD * float(PARAMS.mm_per_rad) / 2.0
    )
    sensitivity_inputs = [
        ("serial_half_quantum", serial_half_quantum),
        ("encoder_half_count", encoder_half_count),
        ("illustrative_0p01mm_cam_clearance", 0.01),
        ("illustrative_0p02mm_cam_clearance", 0.02),
    ]
    return {
        "law": {
            "upstream_normalized_coordinate": "u = sin(pi*p), 0<=p<=1",
            "passive_inverse": "y = y0 + H*(2/pi)*asin(u)",
            "ideal_result": "y = y0 + 2*H*min(p,1-p)",
            "input_domain": [0.0, 1.0],
            "endpoint_derivative": "positive infinity at u=1",
        },
        "geometry": {
            "input_screw_stroke_mm": input_stroke,
            "output_insertion_stroke_mm": output_stroke,
            "initial_slope_output_per_input": amplitude,
            "minimum_pressure_angle_deg": minimum_pressure,
            "endpoint_pressure_angle_deg": 90.0,
            "pressure_angle_limit_deg": PRESSURE_ANGLE_LIMIT_DEG,
            "entire_profile_exceeds_pressure_limit": (
                minimum_pressure > PRESSURE_ANGLE_LIMIT_DEG
            ),
            "minimum_pitch_curve_radius_mm": curvature_radius,
            "candidate_follower_radius_mm": FOLLOWER_RADIUS_MM,
            "roller_diameter_mm": 2.0 * FOLLOWER_RADIUS_MM,
            "roller_curvature_margin_mm": (
                curvature_radius - FOLLOWER_RADIUS_MM),
            "required_curvature_margin_mm": FOLLOWER_CURVATURE_MARGIN_MM,
            "curvature_margin_pass": (
                curvature_radius - FOLLOWER_RADIUS_MM
                >= FOLLOWER_CURVATURE_MARGIN_MM
            ),
            "note": (
                "curvature alone is not release: the minimum-radius point is "
                "also the 90-degree pressure-angle singularity"
            ),
        },
        "last_half_turn_quasistatic_bound": {
            "ideal_half_turn_pitch_mm": half_pitch,
            "input_step_mm": final_input_step,
            "average_transmission_ratio": final_average_ratio,
            "average_pressure_angle_deg": final_average_pressure,
            "output_load_n": output_force,
            "average_cam_input_force_n": input_force,
            "required_screw_torque_nm": screw_torque,
            "selected_m0_dynamic_torque_nm": selected_dynamic_torque,
            "selected_motor_margin": selected_dynamic_torque / screw_torque,
            "coupling_capacity_nm": coupling_capacity,
            "coupling_margin": coupling_capacity / screw_torque,
            "input_speed_mm_s_at_configured_m0_velocity": input_speed,
            "average_output_speed_mm_s": final_average_output_speed,
            "continuous_endpoint_force_torque_and_speed": "unbounded",
            "inertia_note": (
                "quasi-static force already fails; 2.245 kg carriage inertia "
                "would only increase the required force"
            ),
        },
        "backlash_and_quantization": {
            "endpoint_sensitivity": (
                "dy/dx diverges, so no finite worst-case output error exists "
                "for a nonzero input clearance bound at the exact endpoint"
            ),
            "input_error_examples": [
                {"source": label, "input_error_mm": value,
                 "endpoint_output_shortfall_mm": endpoint_shortfall(value)}
                for label, value in sensitivity_inputs
            ],
            "placement_tolerance_status": placement.get("status"),
            "physical_m0_error_is_measured": (
                placement.get("physical_evidence", {})
                .get("m0_carriage_physical_error_mm", {})
                .get("status") == "PASS"
            ),
        },
        "manufacturability": {
            "status": "FAIL",
            "reasons": [
                "90-degree pressure angle/toggle at maximum insertion",
                "unbounded theoretical mechanical advantage and follower load",
                "selected M0 motor and coupling both fail even the finite last-step average",
                "reversing contact at the singular endpoint requires zero play",
                "a diameter-4 mm roller has only a narrow curvature margin and no selected load-rated bearing",
            ],
        },
    }


def _retract_domain(meta: dict[str, Any], selected: dict[str, Any]) -> dict[str, Any]:
    q0, q1 = map(float, meta["m0_wind_range"])
    q_index = float(meta["m1_rotating_position"])
    q_zero = float(meta["m0_zero"])
    q_home = 0.0
    dq = q1 - q0

    def domain_row(label: str, q: float) -> dict[str, Any]:
        u = (q - q0) / dq
        return {
            "pose": label,
            "m0_rad": q,
            "normalized_inverse_sine_input": u,
            "within_real_arcsine_domain": 0.0 <= u <= 1.0,
            "screw_distance_beyond_wind_endpoint_mm": (
                (q - q1) * float(PARAMS.mm_per_rad)
            ),
        }

    rows = [
        domain_row("wind_deep", q0),
        domain_row("wind_outer_endpoint", q1),
        domain_row("index_retract", q_index),
        domain_row("configured_m0_zero", q_zero),
        domain_row("simulation_initial_home", q_home),
    ]
    return {
        "poses": rows,
        "all_required_poses_inside_inverse_sine_domain": all(
            row["within_real_arcsine_domain"] for row in rows
        ),
        "monotone_extension_status": "FAIL",
        "reason": (
            "index, zero, and initial-home inputs exceed u=1; any continuous "
            "monotone extension must leave the winding branch through its "
            "90-degree/infinite-slope endpoint. A dwell would not retract, "
            "and a finite-slope splice would abandon exact inversion."
        ),
        "motor_space_timing_still_passes": (
            selected["retract_motor_timing"]
            ["all_motor_targets_arrive_before_pass_done"]
        ),
        "physical_timing_is_unproven": True,
    }


def _direction_memory_raster(meta: dict[str, Any]) -> dict[str, Any]:
    """Test the strongest two-track/two-lane inverse-sine variant.

    Outbound motion deposits 25 centers on the lane farther from the shared
    slot partition, deep to shallow.  Inbound motion selects a conjugate track
    and deposits 25 centers on the near lane, shallow to deep.  Both lanes use
    the full requested active-radial stroke.  Tangential offsets are chosen at
    their optimistic geometric minimum: the near lane is exactly one wire
    radius from the mirror partition, and the far lane is only far enough away
    that equal-active-radius centers are tangent after the frame rotation.
    Any larger separation worsens the tapered-slot/core fit.
    """

    job = slot_packing_audit.PackingInput(
        float(meta["job"]["wire_finished_d_mm"]),
        float(meta["job"]["liner_max_thickness_mm"]),
    )
    wire_d = job.wire_d_mm
    theta = float(slot_packing_audit.SLOT_BISECTOR_RAD)
    radial_deep, radial_shallow = map(
        float, meta["job"]["radial_winding_span_mm"])
    active_radii = np.linspace(radial_deep, radial_shallow, 25)
    near_v = wire_d / 2.0
    far_v = near_v + wire_d * math.cos(theta)

    def slot_point(active_radial: float, tangential_v: float) -> list[float]:
        # active_radial = u*cos(theta) + v*sin(theta)
        u = (
            float(active_radial) - tangential_v * math.sin(theta)
        ) / math.cos(theta)
        return [u, tangential_v]

    far_lane = [slot_point(value, far_v) for value in active_radii]
    near_lane_geometric = [slot_point(value, near_v) for value in active_radii]
    # Controller order: outbound far deep->shallow, inbound near shallow->deep.
    positive_slot = [*far_lane, *reversed(near_lane_geometric)]
    positive_records = [
        {"turn_index": index, "slot_frame_uv_mm": point,
         "lane": "far_outbound" if index < 25 else "near_inbound"}
        for index, point in enumerate(positive_slot)
    ]
    mirrored_records = [
        {**record, "slot_frame_uv_mm": [
            float(record["slot_frame_uv_mm"][0]),
            -float(record["slot_frame_uv_mm"][1]),
        ]}
        for record in positive_records
    ]

    slot_axis = slot_packing_audit._unit(theta)
    slot_tangent = np.array((-slot_axis[1], slot_axis[0]))
    positive_global = np.asarray([
        float(u) * slot_axis + float(v) * slot_tangent
        for u, v in positive_slot
    ])
    negative_global = np.asarray([
        slot_packing_audit._reflect_about_axis(point, slot_axis)
        for point in positive_global
    ])
    all_points = np.vstack((positive_global, negative_global))
    pair_min, pair_indices = slot_packing_audit._minimum_pair(all_points)
    core_distances = slot_packing_audit._exact_core_distances(all_points, job)
    minimum_core = min(core_distances)
    maximum_radius = float(np.linalg.norm(all_points, axis=1).max())
    required_core = job.center_core_access_mm
    radial_cap = float(slot_packing_audit.RADIAL_CENTER_CAP_MM)

    positive_mouth = slot_packing_audit._sequential_mouth_audit(
        job, positive_records, mirrored_records,
    )
    # Reflection is an exact symmetry of the selected stator section.  Repeat
    # the audit in the positive half-domain with roles exchanged so both M2
    # signs retain independent named results.
    reflected_targets = [
        {**record, "slot_frame_uv_mm": [
            float(record["slot_frame_uv_mm"][0]),
            -float(record["slot_frame_uv_mm"][1]),
        ]}
        for record in mirrored_records
    ]
    reflected_prefill = [
        {**record, "slot_frame_uv_mm": [
            float(record["slot_frame_uv_mm"][0]),
            -float(record["slot_frame_uv_mm"][1]),
        ]}
        for record in positive_records
    ]
    negative_mouth = slot_packing_audit._sequential_mouth_audit(
        job, reflected_targets, reflected_prefill,
    )

    domain = slot_packing_audit._positive_slot_center_domain(job)

    def horizontal_span(v: float) -> float:
        section = domain.intersection(LineString(((-1.0, v), (30.0, v))))
        if section.is_empty:
            return 0.0
        pieces = list(section.geoms) if hasattr(section, "geoms") else [section]
        return max(float(piece.length) for piece in pieces)

    near_span = horizontal_span(near_v)
    far_span = horizontal_span(far_v)
    full_active_span = radial_shallow - radial_deep
    required_full_slot_u_span = full_active_span / math.cos(theta)
    required_25_center_min_u_span = 24.0 * wire_d
    first_target = Point(*map(float, positive_slot[0]))
    first_target_in_domain = domain.buffer(1.0e-9).covers(first_target)
    sign_rows = {
        "M2_positive": {
            "status": positive_mouth["status"],
            "first_empty_neighbor_failure": positive_mouth[
                "first_empty_neighbor_failure"],
            "first_prefilled_neighbor_failure": positive_mouth[
                "first_prefilled_neighbor_failure"],
            "connected_empty_neighbor_count": sum(
                positive_mouth["empty_neighbor_side_mouth_connected"]),
            "connected_prefilled_neighbor_count": sum(
                positive_mouth["prefilled_neighbor_side_mouth_connected"]),
        },
        "M2_negative": {
            "status": negative_mouth["status"],
            "first_empty_neighbor_failure": negative_mouth[
                "first_empty_neighbor_failure"],
            "first_prefilled_neighbor_failure": negative_mouth[
                "first_prefilled_neighbor_failure"],
            "connected_empty_neighbor_count": sum(
                negative_mouth["empty_neighbor_side_mouth_connected"]),
            "connected_prefilled_neighbor_count": sum(
                negative_mouth["prefilled_neighbor_side_mouth_connected"]),
        },
    }
    exact_fit = (
        pair_min >= wire_d - 1.0e-9
        and minimum_core >= required_core - 1.0e-9
        and maximum_radius <= radial_cap + 1.0e-9
    )
    both_mouth = all(row["status"] == "PASS" for row in sign_rows.values())
    return {
        "variant": (
            "direction-memory conjugate tracks: far lane deep-to-shallow, "
            "near lane shallow-to-deep"
        ),
        "status": "PASS" if exact_fit and both_mouth else "FAIL",
        "motion_memory": {
            "outbound_track": "far tangential lane, 25 centers",
            "inbound_track": "near tangential lane, 25 centers",
            "selection_requirement": (
                "passive zero-backlash direction selector at the inverse-sine "
                "endpoint; not represented by a single-valued f(M0)"
            ),
        },
        "optimistic_lane_geometry": {
            "near_slot_tangential_mm": near_v,
            "far_slot_tangential_mm": far_v,
            "lane_separation_mm": far_v - near_v,
            "equal_active_radius_center_distance_mm": wire_d,
            "active_radial_range_mm": [radial_deep, radial_shallow],
            "active_radial_span_mm": full_active_span,
            "centers_per_lane": 25,
            "average_radial_pitch_mm": full_active_span / 24.0,
            "note": (
                "offsets are the minimum compatible with mirror and inter-lane "
                "wire tangency; increasing either offset only reduces tapered-slot room"
            ),
        },
        "exact_slot_fit": {
            "status": "PASS" if exact_fit else "FAIL",
            "minimum_pair_center_distance_mm": pair_min,
            "minimum_pair_indices": list(pair_indices),
            "wire_finished_diameter_mm": wire_d,
            "pair_clearance_ok": pair_min >= wire_d - 1.0e-9,
            "minimum_center_core_distance_mm": minimum_core,
            "required_center_core_distance_mm": required_core,
            "center_core_margin_mm": minimum_core - required_core,
            "core_liner_clearance_ok": minimum_core >= required_core - 1.0e-9,
            "maximum_center_radius_mm": maximum_radius,
            "radial_center_cap_mm": radial_cap,
            "radial_cap_margin_mm": radial_cap - maximum_radius,
            "radial_cap_ok": maximum_radius <= radial_cap + 1.0e-9,
            "core_distance_method": (
                "OpenCascade Part.distance_to(Vertex) against source stator"
            ),
        },
        "straight_lane_capacity_bound": {
            "near_lane_available_slot_u_span_mm": near_span,
            "far_lane_available_slot_u_span_mm": far_span,
            "required_full_stroke_slot_u_span_mm": required_full_slot_u_span,
            "minimum_u_span_for_25_nonoverlapping_centers_mm": (
                required_25_center_min_u_span),
            "far_lane_max_centers_at_wire_pitch": (
                math.floor(far_span / wire_d + 1.0e-12) + 1),
            "far_lane_can_hold_25": (
                far_span >= required_25_center_min_u_span - 1.0e-9),
        },
        "sequential_mouth_and_r3": {
            "minimum_wire_center_bend_radius_mm": 3.0,
            "first_far_deep_target_inside_center_domain": first_target_in_domain,
            "both_flyer_signs": sign_rows,
            "both_signs_mouth_connected": both_mouth,
            "r3_route_exists_both_signs": (
                both_mouth and first_target_in_domain),
            "no_crossing_proved_both_signs": both_mouth,
            "failure_order": (
                "target center violates exact steel/liner domain before an R3 "
                "mouth route or no-crossing route can exist"
                if not first_target_in_domain else
                "sequential center-space mouth component disconnects"
            ),
            "mouth_method": positive_mouth["method"],
            "polygon_circle_relaxation_mm": positive_mouth[
                "polygon_circle_relaxation_mm"],
        },
        "conclusion": (
            "Direction memory removes the exact out/back duplicate, but the "
            "optimistic two-lane 25+25 raster penetrates the lined core, "
            "exceeds the radial cap, and has no sequential mouth route under "
            "either flyer sign. The far straight lane has room for fewer than "
            "25 wire-pitch-separated centers even before endpoint mechanics."
        ),
    }


def _serpentine_successor(packing: dict[str, Any], meta: dict[str, Any],
                          routes: dict[str, Any]) -> dict[str, Any]:
    """Partition the proven honeycomb order into two 25-center cam branches.

    This is deliberately reported separately from the rejected straight-lane
    raster.  It allows tangential offset to vary with M0 and therefore needs a
    second guided output in addition to the radial carriage.  The partition is
    geometrically real, but it does not cure the inverse-sine endpoint or the
    currently failed sign-specific R3 route certificate.
    """

    selected = packing["selected_schedule"]
    centers = selected["side_positive"]
    if len(centers) != 50:
        raise RuntimeError("successor partition requires 50 packing centers")
    wire_d = float(packing["config"]["wire_finished_diameter_mm"])
    branch_records = [centers[:25], centers[25:]]
    branch_names = ("outbound_track", "inbound_track")
    input_stroke = (
        (float(meta["m0_wind_range"][1])
         - float(meta["m0_wind_range"][0]))
        * float(PARAMS.mm_per_rad)
    )
    # Twenty-five wire centers include both branch endpoints, hence 24 moves.
    branch_input = [
        input_stroke * math.sin(0.5 * math.pi * index / 24.0)
        for index in range(25)
    ]
    input_steps = [
        branch_input[index] - branch_input[index - 1]
        for index in range(1, 25)
    ]
    branch_rows = []
    all_path_ratios: list[float] = []
    all_radial_slopes: list[float] = []
    all_tangential_slopes: list[float] = []
    for name, branch in zip(branch_names, branch_records):
        path_steps = []
        radial_steps = []
        tangential_steps = []
        for index in range(1, len(branch)):
            previous = np.asarray(
                branch[index - 1]["active_tooth_frame_uv_mm"], dtype=float)
            current = np.asarray(
                branch[index]["active_tooth_frame_uv_mm"], dtype=float)
            delta = current - previous
            path_steps.append(float(np.linalg.norm(delta)))
            radial_steps.append(float(delta[0]))
            tangential_steps.append(float(delta[1]))
        path_ratios = [
            path / input_step
            for path, input_step in zip(path_steps, input_steps)
        ]
        radial_slopes = [
            abs(value) / input_step
            for value, input_step in zip(radial_steps, input_steps)
        ]
        tangential_slopes = [
            abs(value) / input_step
            for value, input_step in zip(tangential_steps, input_steps)
        ]
        all_path_ratios.extend(path_ratios)
        all_radial_slopes.extend(radial_slopes)
        all_tangential_slopes.extend(tangential_slopes)
        branch_rows.append({
            "name": name,
            "center_indices": [
                int(record["turn_index"]) for record in branch
            ],
            "center_count": len(branch),
            "transition_count": len(path_steps),
            "minimum_center_step_mm": min(path_steps),
            "maximum_center_step_mm": max(path_steps),
            "all_steps_one_wire_diameter": all(
                abs(value - wire_d) <= 1.0e-9 for value in path_steps
            ),
            "maximum_average_2d_track_slope": max(path_ratios),
            "maximum_average_radial_slope": max(radial_slopes),
            "maximum_average_tangential_slope": max(tangential_slopes),
            "maximum_average_pressure_angle_deg": math.degrees(
                math.atan(max(path_ratios))),
        })

    transition_left = np.asarray(
        centers[24]["active_tooth_frame_uv_mm"], dtype=float)
    transition_right = np.asarray(
        centers[25]["active_tooth_frame_uv_mm"], dtype=float)
    selector_transition = float(np.linalg.norm(
        transition_right - transition_left))
    mouth = selected["sequential_mouth_access"]
    route_validation = routes.get("validation", {})
    static_geometry_pass = all((
        packing.get("status") == "PASS",
        all(row["all_steps_one_wire_diameter"] for row in branch_rows),
        abs(selector_transition - wire_d) <= 1.0e-9,
        mouth.get("status") == "PASS",
        mouth.get("all_empty_neighbor_side_connected") is True,
        mouth.get("all_prefilled_neighbor_side_connected") is True,
    ))
    max_ratio = max(all_path_ratios)
    reasonable_slope = math.degrees(math.atan(max_ratio)) <= PRESSURE_ANGLE_LIMIT_DEG
    r3_release = (
        routes.get("status") == "PASS"
        and route_validation.get("both_motion_signs_covered") is True
        and route_validation.get("progressive_support_validated") is True
        and route_validation.get("release_proof_flags", {}).get(
            "c1_bend_continuity") is True
    )
    return {
        "architecture": (
            "two direction-selected 2D cam tracks driving radial carriage "
            "plus a small tangential mouth guide"
        ),
        "scope": (
            "surviving successor architecture; not disproved by the straight-"
            "lane raster and not authorized by this study"
        ),
        "packing_partition": {
            "status": "PASS" if static_geometry_pass else "FAIL",
            "branches": branch_rows,
            "selector_transition_center_distance_mm": selector_transition,
            "selector_transition_is_one_wire_diameter": (
                abs(selector_transition - wire_d) <= 1.0e-9),
            "exact_final_slot_pair_core_and_cap_status": packing["status"],
            "static_sequential_mouth_status": mouth["status"],
            "both_neighbor_cases_static_mouth_connected": (
                mouth["all_empty_neighbor_side_connected"]
                and mouth["all_prefilled_neighbor_side_connected"]
            ),
        },
        "cam_track_slope": {
            "input_stroke_mm": input_stroke,
            "minimum_input_step_mm": min(input_steps),
            "maximum_input_step_mm": max(input_steps),
            "maximum_average_2d_slope": max_ratio,
            "maximum_average_radial_slope": max(all_radial_slopes),
            "maximum_average_tangential_slope": max(all_tangential_slopes),
            "maximum_average_pressure_angle_deg": math.degrees(
                math.atan(max_ratio)),
            "pressure_angle_limit_deg": PRESSURE_ANGLE_LIMIT_DEG,
            "reasonable_slope_pass": reasonable_slope,
            "continuous_endpoint_slope": "unbounded",
        },
        "r3_sequential_route": {
            "source": _rel(SLOT_ROUTES_REPORT),
            "source_sha256": _sha256(SLOT_ROUTES_REPORT),
            "status": "PASS" if r3_release else "FAIL",
            "route_report_status": routes.get("status"),
            "generated_geometry_cases": route_validation.get(
                "generated_geometry_cases"),
            "passed_geometry_cases": route_validation.get(
                "passed_geometry_cases"),
            "expected_direction_cases": route_validation.get(
                "expected_direction_cases"),
            "covered_direction_cases": route_validation.get(
                "covered_direction_cases"),
            "both_motion_signs_covered": route_validation.get(
                "both_motion_signs_covered"),
            "progressive_support_validated": route_validation.get(
                "progressive_support_validated"),
            "c1_bend_continuity": route_validation.get(
                "release_proof_flags", {}).get("c1_bend_continuity"),
            "release_blockers": route_validation.get("release_blockers", []),
        },
        "successor_concept_survives_static_geometry": static_geometry_pass,
        "production_integration_authorized": False,
        "why_not_authorized": [
            "last discrete 2D cam step already exceeds the pressure-angle criterion",
            "continuous inverse-sine endpoint remains singular on both tracks",
            "current R3 route report does not cover either motion sign and lacks C1 bend continuity/progressive support",
            "direction selector, tangential guide, full retract branch, loads, backlash, collision, and coupon evidence do not exist",
        ],
    }
def _write_half_turn_csv(intervals: list[dict[str, Any]]) -> None:
    OUTPUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "pass_index", "tooth", "clockwise", "half_turn_index", "time_s",
        "m0_model_rad", "m0_serial_rad", "physical_radius_model_mm",
        "physical_radius_serial_mm", "model_abs_pitch_mm",
        "serial_abs_pitch_mm", "ideal_abs_pitch_mm",
    ]
    with OUTPUT_CSV.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in intervals:
            writer.writerow({field: row[field] for field in fields})


def build_report() -> dict[str, Any]:
    upstream = _upstream_identity()
    if not upstream["clean"]:
        raise RuntimeError("upstream checkout is not clean")
    packing = _load_json(PACKING_REPORT)
    loads = _load_json(LOADS_REPORT)
    placement = _load_json(PLACEMENT_TOLERANCE_REPORT)
    routes = _load_json(SLOT_ROUTES_REPORT)
    if packing.get("schema") != "slot-packing/v2" or packing.get("status") != "PASS":
        raise RuntimeError("slot packing input is not PASS v2")

    selected_candidate = next(
        candidate for candidate in candidates() if candidate.label == "n6_exact"
    )
    sweep: list[dict[str, Any]] = []
    selected: dict[str, Any] | None = None
    for candidate in candidates():
        if not candidate.settings_path.is_file():
            raise RuntimeError(
                f"missing {candidate.label} study settings; run with "
                "--generate-captures"
            )
        if not candidate.capture_path.is_file():
            if not candidate.failure_path.is_file():
                raise RuntimeError(
                    f"missing {candidate.label} capture/failure artifact; run "
                    "with --generate-captures"
                )
            sweep.append({
                "label": candidate.label,
                "poll_divisor_n": candidate.poll_divisor_n,
                "detune_fraction": candidate.detune_fraction,
                "m2_velocity_rad_s": candidate.velocity_rad_s,
                "m2_velocity_rpm": (
                    candidate.velocity_rad_s * 60.0 / (2.0 * math.pi)),
                "m2_per_poll_rad": (
                    candidate.velocity_rad_s * POLL_INTERVAL_S),
                "settings": {
                    "path": _rel(candidate.settings_path),
                    "sha256": _sha256(candidate.settings_path),
                },
                "capture": {
                    "path": None,
                    "sha256": None,
                    "status": "FAIL",
                    "failure_log": _rel(candidate.failure_path),
                    "failure_log_sha256": _sha256(candidate.failure_path),
                    "reason": candidate.failure_path.read_text(
                        encoding="utf-8").strip().splitlines()[-1],
                },
                "pass_count": None,
                "half_turn_interval_count": None,
                "ideal_half_turn_pitch_mm": None,
                "model_coordinate_pitch": None,
                "serial_coordinate_pitch": None,
                "poll_alignment": None,
                "wire_overlap": None,
                "retract_motor_timing": None,
            })
            continue
        analysis = analyze_capture(
            candidate, packing, include_intervals=(candidate == selected_candidate),
        )
        if candidate == selected_candidate:
            selected = analysis
        sweep.append({
            key: analysis[key] for key in (
                "label", "poll_divisor_n", "detune_fraction",
                "m2_velocity_rad_s", "m2_velocity_rpm", "m2_per_poll_rad",
                "settings", "capture", "pass_count", "half_turn_interval_count",
                "ideal_half_turn_pitch_mm", "model_coordinate_pitch",
                "serial_coordinate_pitch", "poll_alignment", "wire_overlap",
                "retract_motor_timing",
            )
        })
    assert selected is not None

    canonical_events = load_events(CANONICAL_CAPTURE)
    canonical_meta = next(row for row in canonical_events if row.get("e") == "meta")
    mechanics = _cam_mechanics(canonical_meta, loads, placement)
    retract = _retract_domain(canonical_meta, selected)
    direction_memory = _direction_memory_raster(canonical_meta)
    successor = _serpentine_successor(packing, canonical_meta, routes)
    intervals = selected.pop("half_turn_intervals")
    _write_half_turn_csv(intervals)
    selected["half_turn_state_report"] = {
        "path": _rel(OUTPUT_CSV),
        "sha256": _sha256(OUTPUT_CSV),
        "row_count": len(intervals),
        "contract": "24 passes x 100 physical half-turn intervals",
    }

    ideal_static_errors = []
    q0, q1 = map(float, canonical_meta["m0_wind_range"])
    r0, r1 = map(float, canonical_meta["job"]["radial_winding_span_mm"])
    for half_index in range(101):
        progress = half_index / 100.0
        q = q0 + (q1 - q0) * math.sin(math.pi * progress)
        ideal = r0 + (r1 - r0) * 2.0 * min(progress, 1.0 - progress)
        ideal_static_errors.append(abs(
            inverse_sine_radius(q, (q0, q1), (r0, r1)) - ideal
        ))

    gates = {
        "upstream_checkout_clean": upstream["clean"],
        "all_completed_captures_unmodified_upstream": all(
            row["capture"].get("status") != "FAIL"
            and
            row["capture"]["controller_mode"] == "upstream"
            and row["capture"]["controller_adapter_sha256"] is None
            and row["capture"]["winder_commit"] == upstream["commit"]
            for row in sweep if row["capture"].get("status") != "FAIL"
        ),
        "all_velocity_sweep_points_complete_upstream_cycle": all(
            row["capture"].get("status") != "FAIL" for row in sweep
        ),
        "static_arcsine_exactly_inverts_sine": max(ideal_static_errors) <= 1.0e-9,
        "n6_stale_poll_samples_half_turn_aligned": (
            selected["poll_alignment"]
            ["stale_poll_samples_half_turn_aligned"]
        ),
        "n6_m0_commands_physically_issued_at_half_turns": (
            selected["poll_alignment"]
            ["commands_physically_issued_at_half_turns"]
        ),
        "all_24x100_physical_pitches_linear": (
            selected["serial_coordinate_pitch"]
            ["all_intervals_linear_within_1um"]
        ),
        "all_24x100_m0_updates_settle": (
            selected["poll_alignment"]
            ["all_updates_settle_before_next_half_turn"]
        ),
        "no_same_side_wire_overlap": (
            selected["wire_overlap"]
            ["total_same_side_pairs_below_wire_diameter"] == 0
        ),
        "direction_memory_raster_exact_slot_fit": (
            direction_memory["exact_slot_fit"]["status"] == "PASS"
        ),
        "direction_memory_far_lane_holds_25_wires": (
            direction_memory["straight_lane_capacity_bound"]
            ["far_lane_can_hold_25"]
        ),
        "direction_memory_r3_mouth_access_both_flyer_signs": (
            direction_memory["sequential_mouth_and_r3"]
            ["r3_route_exists_both_signs"]
        ),
        "direction_memory_no_crossing_both_flyer_signs": (
            direction_memory["sequential_mouth_and_r3"]
            ["no_crossing_proved_both_signs"]
        ),
        "serpentine_successor_partition_static_geometry": (
            successor["successor_concept_survives_static_geometry"]
        ),
        "all_wire_centers_within_radial_bounds": (
            selected["wire_overlap"]["all_centers_inside_radial_bounds"]
        ),
        "cam_pressure_angle_within_limit": not (
            mechanics["geometry"]["entire_profile_exceeds_pressure_limit"]
        ),
        "finite_endpoint_transmission": False,
        "follower_curvature_margin": mechanics["geometry"][
            "curvature_margin_pass"],
        "m0_motor_margin_at_least_2x": (
            mechanics["last_half_turn_quasistatic_bound"]
            ["selected_motor_margin"] >= 2.0
        ),
        "m0_coupling_margin_at_least_2x": (
            mechanics["last_half_turn_quasistatic_bound"]
            ["coupling_margin"] >= 2.0
        ),
        "measured_backlash_error_budget_pass": (
            mechanics["backlash_and_quantization"]
            ["physical_m0_error_is_measured"]
            and placement.get("status") == "PASS"
        ),
        "monotone_cam_covers_winding_retract_and_home": (
            retract["all_required_poses_inside_inverse_sine_domain"]
        ),
    }
    release = all(gates.values())
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "PASS" if release else "FAIL",
        "decision": (
            "PRODUCTION_INTEGRATION_AUTHORIZED" if release
            else "REJECT_PASSIVE_MONOTONE_ARCSINE_TRANSMISSION"
        ),
        "role": "analytical_study_only_no_production_mutation",
        "cad_brief": {
            "task_type": "kinematic transmission feasibility inspection",
            "model": "passive monotone M0 inverse-sine cam/linkage",
            "units": "millimetres, radians, seconds, newtons, newton-metres",
            "input_axis": "T8x8 screw coordinate from upstream M0 radians",
            "output_axis": "physical stator/tooth insertion along machine Z",
            "required_domain": "full winding, post-pass index retract, configured zero, initial home",
            "outputs": [
                _rel(OUTPUT_JSON), _rel(OUTPUT_MD), _rel(OUTPUT_CSV),
                _rel(SETTINGS_DIR), _rel(CAPTURE_DIR),
            ],
            "validation_targets": [
                "24x100 physical half-turn pitches", "wire overlap and slot bounds",
                "poll alignment and M0 settling", "cam pressure angle and curvature",
                "follower size/load and M0 torque", "backlash sensitivity",
                "monotone retract/index/home extension",
            ],
            "geometry_artifact": None,
            "snapshot_skip_reason": (
                "fail-closed analytical inspection only; no STEP/CAD geometry was "
                "created or modified because the kinematic/load gates fail first"
            ),
        },
        "authority": {
            "upstream_winder": upstream,
            "production_settings": {
                "path": _rel(PRODUCTION_SETTINGS),
                "sha256": _sha256(PRODUCTION_SETTINGS),
            },
            "canonical_capture": {
                "path": _rel(CANONICAL_CAPTURE),
                "sha256": _sha256(CANONICAL_CAPTURE),
                "controller_mode": canonical_meta["controller_mode"],
                "adapter_sha256": canonical_meta["controller_adapter_sha256"],
            },
            "packing_report": {
                "path": _rel(PACKING_REPORT), "sha256": _sha256(PACKING_REPORT),
                "status": packing["status"],
            },
            "loads_report": {
                "path": _rel(LOADS_REPORT), "sha256": _sha256(LOADS_REPORT),
            },
            "placement_tolerance": {
                "path": _rel(PLACEMENT_TOLERANCE_REPORT),
                "sha256": _sha256(PLACEMENT_TOLERANCE_REPORT),
                "status": placement.get("status"),
            },
            "slot_wire_routes": {
                "path": _rel(SLOT_ROUTES_REPORT),
                "sha256": _sha256(SLOT_ROUTES_REPORT),
                "status": routes.get("status"),
            },
        },
        "sweep_definition": {
            "poll_interval_s": POLL_INTERVAL_S,
            "velocity_law": "M2_velocity = pi/(n*0.03) * (1+detune)",
            "upstream_trigger": "delta_M2 >= pi - 0.01 rad",
            "candidate_count": len(sweep),
            "selected_for_full_state_audit": "n6_exact",
        },
        "sweep": sweep,
        "selected_candidate": selected,
        "static_mapping_proof": {
            "sample_count": len(ideal_static_errors),
            "maximum_static_inversion_error_mm": max(ideal_static_errors),
            "result": "PASS",
            "scope": (
                "instantaneous commanded coordinate only; excludes motor lag, "
                "serial rounding, follower mechanics, and retract extension"
            ),
        },
        "cam_mechanics": mechanics,
        "direction_memory_two_track_variant": direction_memory,
        "serpentine_honeycomb_successor": successor,
        "retract_and_index_extension": retract,
        "gates": gates,
        "release_authorized": release,
        "production_integration": {
            "authorized": release,
            "specification": None if not release else {
                "note": "unreachable unless every fail-closed gate passes"
            },
            "blocking_findings": [
                key for key, value in gates.items() if not value
            ],
            "surviving_successor_architecture": (
                "two direction-selected 2D serpentine tracks following the "
                "existing 25+25 honeycomb partition, with a radial carriage "
                "output plus tangential mouth-guide output; static geometry "
                "only, not authorized"
            ),
            "remaining_redesign_gates": [
                "replace the singular inverse with a finite-slope motion law or a separately actuated/safely latched two-mode transmission",
                "phase M0 output before the first half-turn without changing the upstream serial contract",
                "provide a deterministic two-dimensional 50-wire-per-side packing mechanism rather than a radial-only locus",
                "prove full-domain retract/index/home kinematics and rerun collision/wire-path audits",
                "select a load-rated follower and prove Hertz/contact fatigue, wear, lubrication, and retention",
                "close measured backlash, encoder, carriage, TIR, wire, liner, and contact-position tolerance budgets",
                "rerun motor/coupling loads with at least 2x dynamic margin and qualify on an instrumented winding coupon",
            ],
        },
        "limitations": [
            "No cam CAD was generated because analytical kinematic and load gates fail.",
            "Wire overlap assumes no unmodeled tangential self-organization; such behavior cannot be a release authority.",
            "Quasi-static cam force excludes carriage inertia, impact, friction, Hertz stress, and wear, all of which worsen the endpoint result.",
            "Candidate captures use the upstream simulation motion model; hardware following error remains unmeasured.",
        ],
        "source_hashes": {
            "sim/passive_m0_arcsine_transmission_study.py": _sha256(Path(__file__)),
            "sim/test_passive_m0_arcsine_transmission_study.py": _sha256(
                HERE / "test_passive_m0_arcsine_transmission_study.py"),
            "sim/capture.py": _sha256(HERE / "capture.py"),
            "sim/traj.py": _sha256(HERE / "traj.py"),
            "cad/params.py": _sha256(CAD / "params.py"),
            "cad/slot_packing_audit.py": _sha256(
                CAD / "slot_packing_audit.py"),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def _markdown(report: dict[str, Any]) -> str:
    selected = report["selected_candidate"]
    pitch = selected["serial_coordinate_pitch"]
    overlap = selected["wire_overlap"]
    geometry = report["cam_mechanics"]["geometry"]
    load = report["cam_mechanics"]["last_half_turn_quasistatic_bound"]
    memory = report["direction_memory_two_track_variant"]
    memory_fit = memory["exact_slot_fit"]
    memory_mouth = memory["sequential_mouth_and_r3"]
    capacity = memory["straight_lane_capacity_bound"]
    successor = report["serpentine_honeycomb_successor"]
    successor_partition = successor["packing_partition"]
    successor_slope = successor["cam_track_slope"]
    successor_route = successor["r3_sequential_route"]
    retract = report["retract_and_index_extension"]
    failed = [key for key, value in report["gates"].items() if not value]
    sweep_lines = []
    for row in report["sweep"]:
        if row["capture"].get("status") == "FAIL":
            sweep_lines.append(
                f"| {row['label']} | {row['m2_velocity_rad_s']:.9f} | "
                "cycle FAIL | -- | -- | -- | -- |"
            )
            continue
        sweep_lines.append(
            f"| {row['label']} | {row['m2_velocity_rad_s']:.9f} | "
            f"{row['poll_alignment']['maximum_abs_stale_sample_phase_error_half_turn']:.6f} | "
            f"{row['poll_alignment']['maximum_abs_command_issue_phase_error_half_turn']:.6f} | "
            f"{row['serial_coordinate_pitch']['minimum_abs_pitch_mm']:.6f} | "
            f"{row['serial_coordinate_pitch']['maximum_abs_pitch_mm']:.6f} | "
            f"{row['serial_coordinate_pitch']['zero_pitch_interval_count']} |"
        )
    return f"""# Passive nonlinear M0 inverse-sine transmission study

**Status: {report['status']} — {report['decision']}.** Production CAD and
settings were not changed. Candidate settings/captures are isolated under
`out/studies/passive_m0_arcsine/` and all use the clean unmodified upstream
controller at `{report['authority']['upstream_winder']['commit']}`.

## Static law versus the real timeline

The static identity is exact on the winding interval:
`u=sin(pi*p)`, `y=y0+H*(2/pi)*asin(u)`, therefore
`y=y0+2H*min(p,1-p)`. The 101-point static error is
{report['static_mapping_proof']['maximum_static_inversion_error_mm']:.3e} mm.

That identity does **not** make the real 24 x 100 half-turn timeline linear.
For the requested n=6 setting ({selected['m2_velocity_rad_s']:.11f} rad/s,
{selected['m2_velocity_rpm']:.6f} rpm), all {selected['half_turn_interval_count']}
intervals are recorded in `{selected['half_turn_state_report']['path']}`.
The real rounded-serial-coordinate pitch is {pitch['minimum_abs_pitch_mm']:.6f}
to {pitch['maximum_abs_pitch_mm']:.6f} mm versus the ideal
{selected['ideal_half_turn_pitch_mm']:.6f} mm; {pitch['zero_pitch_interval_count']}
intervals have zero pitch. Upstream performs its one-turn `fast_winding()`
prelude before the first M0 update. At exact n=6 the stale polled M2 values are
half-turn aligned, but upstream then sleeps 0.03 s before issuing M0; the flyer
advances another 1/6 half-turn during that sleep. The M0 target settles for the
*next* crossing, not the sampled crossing.

| candidate | M2 rad/s | stale-sample phase error, half-turn | command-issue phase error, half-turn | min pitch mm | max pitch mm | zero intervals |
|---|---:|---:|---:|---:|---:|---:|
{chr(10).join(sweep_lines)}

## Wire placement

All modeled radial centers remain inside {overlap['allowed_radial_range_mm'][0]:.6f}
to {overlap['allowed_radial_range_mm'][1]:.6f} mm. That is insufficient: a
radial-only cam supplies no tangential/layer coordinate. Treating each tooth
side as the one locus the cam actually defines gives a minimum same-side
center distance of {overlap['minimum_same_side_pair_center_distance_mm']:.6f}
mm and {overlap['total_same_side_pairs_below_wire_diameter']} pairs closer than
the {overlap['wire_finished_diameter_mm']:.5f} mm finished wire diameter
({overlap['total_exact_duplicate_pairs']} exact duplicate pairs).

The stronger direction-memory variant was also tested. Outbound motion uses a
far lane deep-to-shallow and inbound motion a near lane shallow-to-deep, with
25 centers per lane and the minimum possible tangential offsets. It removes
the exact out/back duplicate, but exact CAD distance gives only
{memory_fit['minimum_center_core_distance_mm']:.6f} mm center/core distance
against {memory_fit['required_center_core_distance_mm']:.6f} mm required, and
the outer center exceeds the cap by {-memory_fit['radial_cap_margin_mm']:.6f}
mm. The tapered far lane provides {capacity['far_lane_available_slot_u_span_mm']:.6f}
mm versus {capacity['minimum_u_span_for_25_nonoverlapping_centers_mm']:.6f} mm
needed, enough for at most {capacity['far_lane_max_centers_at_wire_pitch']}
wire-pitch-separated centers. Sequential mouth access fails from placement 0
under both M2 signs; because the target center itself is outside the lined
center domain, no R3 route or no-crossing route exists
(`r3_route_exists_both_signs={memory_mouth['r3_route_exists_both_signs']}`).

### Surviving successor architecture

The straight-lane failure does **not** rule out a two-track 2D serpentine cam
that drives both radial carriage position and a small tangential mouth guide.
The existing exact 50-center honeycomb order partitions cleanly into two
continuous 25-center branches: all 24 transitions within each branch and the
branch-selector transition are exactly one {overlap['wire_finished_diameter_mm']:.5f}
mm wire diameter. Exact final-slot fit and static sequential mouth access both
remain `{successor_partition['status']}`.

It is not production-ready. The clustered inverse-sine input leaves only
{successor_slope['minimum_input_step_mm']:.6f} mm for the final discrete cam
step, producing an average 2D slope of {successor_slope['maximum_average_2d_slope']:.3f}
and {successor_slope['maximum_average_pressure_angle_deg']:.3f} degrees before
the continuous endpoint becomes singular. The current R3 route authority is
`{successor_route['status']}`: {successor_route['passed_geometry_cases']}/
{successor_route['generated_geometry_cases']} geometry cases pass, but
{successor_route['covered_direction_cases']}/{successor_route['expected_direction_cases']}
motion-sign cases are covered, progressive support is
{successor_route['progressive_support_validated']}, and C1 bend continuity is
{successor_route['c1_bend_continuity']}. This is the next architecture worth a
separate finite-slope/two-output study; it is not disproved by this report and
is not an integration specification.

## Cam, follower, and M0 load

- Input screw stroke: {geometry['input_screw_stroke_mm']:.6f} mm; output stroke:
  {geometry['output_insertion_stroke_mm']:.6f} mm.
- Pressure angle starts at {geometry['minimum_pressure_angle_deg']:.3f} degrees,
  already above the {geometry['pressure_angle_limit_deg']:.1f}-degree study
  criterion, and tends to 90 degrees at maximum insertion.
- Minimum pitch-curve radius is {geometry['minimum_pitch_curve_radius_mm']:.6f}
  mm. A diameter-{geometry['roller_diameter_mm']:.1f} mm follower leaves only
  {geometry['roller_curvature_margin_mm']:.6f} mm curvature margin; this does
  not cure the pressure-angle singularity.
- The final finite half-turn averages {load['average_transmission_ratio']:.3f}:1,
  {load['average_cam_input_force_n']:.1f} N, and
  {load['required_screw_torque_nm']:.3f} N m. Selected M0 dynamic torque margin
  is {load['selected_motor_margin']:.3f}x and coupling margin is
  {load['coupling_margin']:.3f}x; the required criterion is 2x. Continuous
  endpoint force, speed, and torque are unbounded.
- Any backlash is amplified without a finite endpoint bound. The existing
  placement-tolerance release report is
  `{report['authority']['placement_tolerance']['status']}` and physical M0
  error remains unmeasured.

## Retract/index domain

The inverse-sine branch ends at normalized input u=1. Required index, configured
zero, and initial-home poses are outside it. Motor-space retract timing still
passes ({retract['motor_space_timing_still_passes']}), but no physical output
state exists: a dwell does not retract, a finite-slope splice breaks exact
inversion, and any monotone continuous extension must cross the same 90-degree
toggle. Therefore a single monotone passive cam cannot cover winding plus
retract safely.

## Failed release gates

{chr(10).join(f'- `{value}`' for value in failed)}

No production integration specification is issued. A finite-slope/two-mode
redesign would require every gate listed in
`production_integration.remaining_redesign_gates` to be closed before CAD or
procurement release.

Report proof hash: `{report['report_sha256']}`.
"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--generate-captures", action="store_true",
        help="write isolated settings copies and recapture every sweep point",
    )
    parser.add_argument(
        "--force", action="store_true",
        help="regenerate captures even when hashes already match",
    )
    args = parser.parse_args()
    if args.force and not args.generate_captures:
        parser.error("--force requires --generate-captures")
    if args.generate_captures:
        generated = generate_candidates(force=args.force)
        print(f"prepared {len(generated)} isolated candidate captures")
    report = build_report()
    OUTPUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8",
    )
    OUTPUT_MD.write_text(_markdown(report), encoding="utf-8")
    print(
        f"{report['status']}: wrote {_rel(OUTPUT_JSON)}, {_rel(OUTPUT_MD)}, "
        f"and {_rel(OUTPUT_CSV)}"
    )
    print(f"report sha256 {report['report_sha256']}")


if __name__ == "__main__":
    main()
