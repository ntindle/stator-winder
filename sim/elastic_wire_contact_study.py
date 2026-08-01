"""Fail-closed elastic/contact audit of the fixed flyer slot route.

This is deliberately a *study*, not a production route generator.  It asks
two questions which the rigid packed-wire certificate cannot answer:

1. Do both turn-45 support approaches admit exact intended parent-wire
   contact?  The current route table may already repair the historical rigid
   failures, so the two cases are selected by turn/half identity rather than
   failure status.  For each we construct the exact taut tangent/contact-arc
   diagnostic and independently check steel, non-parent copper, topology,
   and numerical convergence.
2. Does that repaired geometry prove the actual unmodified upstream motion?
   The authoritative raw capture is replayed as the upstream simulator does:
   every axis is a velocity-limited point mass following asynchronous
   absolute targets.  One hundred equal-flyer-travel states are solved for
   every one of the 24 winding passes and compared with the hash-bound
   packed route schedule.

Frictionless contact may carry compression but not adhesion or tangential
traction.  Prior copper and Nomex may be touched/slid on.  Steel and
non-parent copper penetration remain forbidden.  A contact construction is
not called elastic-feasible when its local centreline bend radius violates
the 3 mm GOAL.md limit.  No undocumented enamel compression or plastic
conductor hinge is credited.

The study therefore distinguishes a useful geometric result (the two rigid
collisions can be replaced by exact contact) from release authorization (the
raw motion and elastic/curvature contract still must all pass).
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable

import numpy as np
from shapely.geometry import LineString


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
PACKING = REPORTS / "slot_packing.json"
ROUTES = REPORTS / "slot_wire_routes.json"
MANIFEST = ROOT / "out" / "links" / "manifest.json"
SETTINGS = ROOT / "out" / "settings.yml"
GOAL = ROOT.parent / "GOAL.md"
OUTPUT_JSON = REPORTS / "elastic_wire_contact_study.json"
OUTPUT_MD = REPORTS / "elastic_wire_contact_study.md"

if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

from params import DEFAULT_STATOR, PARAMS  # noqa: E402
from slot_route import (  # noqa: E402
    CopperField,
    PackingSupportGraph,
    SlotRoutePlanner,
    active_copper_before,
    dependency_versions,
    exact_polyline_part_clearance,
    neighbor_prefill_copper,
)
import slot_wire_routes  # noqa: E402
from traj import Timeline, winding_windows  # noqa: E402


SCHEMA = "elastic-wire-contact-study/v1"
EXPECTED_CAPTURE_SCHEMA = 4
EXPECTED_PASSES = 24
STATES_PER_PASS = 100
TIME_DECIMALS = 4
TARGET_DECIMALS = 9
BISECTION_STEPS = 72
CONTACT_ROUTE_STEPS_DEG = (2.0, 1.0, 0.5, 0.25, 0.125)
CONTACT_OBSTACLE_STEP_DEG = 1.0
CONTACT_TURN = 45
CONTACT_PARENT = 44
REMINGTON_BARE_COPPER_DIAMETER_MM = 0.2032
REMINGTON_PRODUCT_URL = (
    "https://www.remingtonindustries.com/magnet-wire/"
    "magnet-wire-32-awg-enameled-copper-9-spool-sizes/"
)
REMINGTON_DATA_URL = (
    "https://www.remingtonindustries.com/content/"
    "Remington%20Copper%20and%20Magnet%20Wire%20Data%20Chart.pdf"
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: dict[str, Any]) -> str:
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = [json.loads(line) for line in path.read_text(
        encoding="utf-8").splitlines() if line.strip()]
    if not rows:
        raise ValueError("raw capture is empty")
    return rows


def _validate_stored_route_payload(
    report: dict[str, Any], packing: dict[str, Any],
) -> dict[str, Any]:
    """Validate the immutable stored table even when generator sources moved.

    Concurrent architecture studies may legitimately make the production
    source-hash gate stale.  That must remain a release failure, but it does
    not erase the geometry encoded in the hash-bound diagnostic which this
    study is explicitly re-analysing.
    """

    if report.get("schema") != slot_wire_routes.SCHEMA:
        raise ValueError("unsupported stored slot-wire route schema")
    payload = dict(report)
    expected = payload.pop("report_sha256", None)
    if (not isinstance(expected, str)
            or slot_wire_routes._canonical_hash(payload) != expected):
        raise ValueError("stored slot-wire route report hash mismatch")
    if report.get("input_contract", {}).get(
            "packing_report_sha256") != packing.get("report_sha256"):
        raise ValueError("stored slot-wire routes do not bind current packing")
    routes = report.get("routes")
    expected_coverage = {
        (turn, half) for turn in range(50) for half in (0, 1)
    }
    coverage = {
        (int(row["turn_index"]), int(row["half_turn_index"]))
        for row in routes if isinstance(row, dict)
    } if isinstance(routes, list) else set()
    if len(routes or ()) != 100 or coverage != expected_coverage:
        raise ValueError("stored slot-wire route coverage is incomplete")
    current_sources = True
    stale_reason = None
    try:
        slot_wire_routes.validate_report_integrity(report, packing)
    except ValueError as exc:
        if "source hashes are stale" not in str(exc):
            raise
        current_sources = False
        stale_reason = str(exc)
    return {
        "stored_payload_hash_valid": True,
        "packing_hash_bound": True,
        "coverage_complete": True,
        "generator_sources_current": current_sources,
        "stale_reason": stale_reason,
    }


@dataclass(frozen=True)
class MotionSegment:
    t0_s: float
    t1_s: float
    p0_rad: float
    target_rad: float
    velocity_rad_s: float

    def position(self, time_s: float) -> float:
        dt = max(0.0, min(float(time_s), self.t1_s) - self.t0_s)
        delta = self.target_rad - self.p0_rad
        travel = max(
            -self.velocity_rad_s * dt,
            min(self.velocity_rad_s * dt, delta),
        )
        return float(self.p0_rad + travel)


class AxisTimeline:
    """Exact replay of ``calculate_motor_position_in_simulation``."""

    def __init__(self, velocity_rad_s: float):
        if not math.isfinite(velocity_rad_s) or velocity_rad_s <= 0.0:
            raise ValueError("axis velocity must be finite and positive")
        self.velocity_rad_s = float(velocity_rad_s)
        self._position = 0.0
        self._target = 0.0
        self._time = 0.0
        self.segments: list[MotionSegment] = []

    def command(self, time_s: float, target_rad: float) -> None:
        time_s = float(time_s)
        target_rad = float(target_rad)
        if time_s + 1e-12 < self._time:
            raise ValueError("axis commands are not time ordered")
        segment = MotionSegment(
            self._time, time_s, self._position, self._target,
            self.velocity_rad_s,
        )
        self.segments.append(segment)
        self._position = segment.position(time_s)
        self._time = time_s
        self._target = target_rad

    def finish(self, time_s: float) -> None:
        self.command(time_s, self._target)

    def position(self, time_s: float) -> float:
        time_s = float(time_s)
        for segment in self.segments:
            if segment.t0_s - 1e-12 <= time_s <= segment.t1_s + 1e-12:
                return segment.position(time_s)
        raise ValueError(f"time {time_s:.9f} is outside the axis timeline")


def build_timelines(events: list[dict[str, Any]]) -> tuple[AxisTimeline, ...]:
    meta = events[0]
    velocities = meta.get("velocities")
    if not isinstance(velocities, list) or len(velocities) != 4:
        raise ValueError("raw capture does not declare four axis velocities")
    axes = tuple(AxisTimeline(float(value)) for value in velocities)
    for row in events:
        if row.get("e") != "cmd":
            continue
        motor = int(row["m"])
        if not 0 <= motor < 4:
            raise ValueError("raw capture contains an invalid motor id")
        axes[motor].command(float(row["t"]), float(row["model_target"]))
    end_time = float(events[-1]["t"]) + 2.0
    for axis in axes:
        axis.finish(end_time)
    return axes


def validate_capture_contract(
    events: list[dict[str, Any]],
    capture_path: Path = CAPTURE,
) -> dict[str, Any]:
    meta = events[0]
    checks = {
        "meta_first": meta.get("e") == "meta",
        "schema_4": meta.get("capture_schema") == EXPECTED_CAPTURE_SCHEMA,
        "unmodified_upstream_controller": meta.get("controller_mode") == "upstream",
        "no_controller_adapter": meta.get("controller_adapter_sha256") is None,
        "24_teeth": int(meta.get("teeth_count", -1)) == 24,
        "50_turn_job": int(meta.get("turns", -1)) == 50,
        "cycle_complete": sum(
            row.get("e") == "cycle_complete" for row in events) == 1,
        "24_winding_passes": sum(
            row.get("e") == "wind_wire" for row in events) == EXPECTED_PASSES,
        "current_settings_hash": (
            SETTINGS.is_file()
            and meta.get("settings_sha256") == _sha256(SETTINGS)
        ),
    }
    if not all(checks.values()):
        failed = ", ".join(name for name, ok in checks.items() if not ok)
        raise ValueError(f"raw capture contract failed: {failed}")
    return {
        "status": "PASS",
        "checks": checks,
        "path": str(capture_path.resolve()),
        "sha256": _sha256(capture_path),
        "winder_commit": meta["winder_commit"],
        "settings_sha256": meta["settings_sha256"],
        "velocities_rad_s": list(map(float, meta["velocities"])),
        "event_count": len(events),
    }


def _event_windows(
    events: list[dict[str, Any]], start_name: str, done_name: str,
) -> list[tuple[int, int]]:
    starts = [index for index, row in enumerate(events)
              if row.get("e") == start_name]
    result = []
    for start in starts:
        done = next((
            index for index in range(start + 1, len(events))
            if events[index].get("e") == done_name
        ), None)
        if done is None:
            raise ValueError(f"unclosed {start_name} event")
        result.append((start, done))
    return result


def _first_reach_time(
    axis: AxisTimeline,
    target_rad: float,
    sign: int,
    lo_s: float,
    hi_s: float,
    *,
    steps: int = BISECTION_STEPS,
) -> float:
    if sign not in (-1, 1):
        raise ValueError("motion sign must be -1 or +1")
    if sign * (axis.position(lo_s) - target_rad) >= -1e-10:
        return float(lo_s)
    if sign * (axis.position(hi_s) - target_rad) < -1e-8:
        raise ValueError("axis never reaches requested phase in pass window")
    lo, hi = float(lo_s), float(hi_s)
    for _ in range(int(steps)):
        mid = (lo + hi) / 2.0
        if sign * (axis.position(mid) - target_rad) >= 0.0:
            hi = mid
        else:
            lo = mid
    return float((lo + hi) / 2.0)


def _directed_crossing_times(
    track: Any, start_t: float, start_pos: float,
    direction: int, count: int,
) -> list[float]:
    """Invert the exact piecewise-linear raw M2 track at pi crossings."""

    if direction not in (-1, 1):
        raise ValueError("crossing direction must be -1 or +1")
    result: list[float] = []
    for index in range(int(count)):
        target = float(start_pos + direction * index * math.pi)
        if index == 0:
            result.append(float(start_t))
            continue
        found = None
        for (left_t, _), (right_t, right_p) in zip(
                track.knots, track.knots[1:]):
            if right_t < start_t - 1e-12:
                continue
            local_left_t = max(float(left_t), float(start_t))
            local_left_p = float(track.pos_at(local_left_t))
            if direction * (float(right_p) - local_left_p) < -1e-10:
                continue
            if ((target - local_left_p) * (target - float(right_p))
                    <= 1e-10
                    and abs(float(right_p) - local_left_p) > 1e-12):
                found = local_left_t + (
                    (float(right_t) - local_left_t)
                    * (target - local_left_p)
                    / (float(right_p) - local_left_p)
                )
                break
        if found is None:
            break
        result.append(float(found))
    return result


def replay_raw_winding_states(
    events: list[dict[str, Any]],
    timeline: Any,
    graph: PackingSupportGraph,
) -> dict[str, Any]:
    """Bind the exact 100 deposition crossings for every raw pass.

    The first post-positioning M2 command is logical crossing zero.  The
    piecewise-linear raw track is then inverted at 101 directed pi-spaced
    phases.  Crossings 0..99 are the two sides of 50 turns; crossing 100
    closes the last full turn.  This includes any final captured side/parking
    correction needed to physically reach the logical phase instead of
    truncating the pass at its single long M2 target.
    """

    windows = winding_windows(events)
    if len(windows) != EXPECTED_PASSES:
        raise ValueError("raw capture does not contain 24 winding windows")
    meta = events[0]
    span = tuple(map(float, meta["job"]["radial_winding_span_mm"]))
    mm_per_rad = float(PARAMS.mm_per_rad)
    timestamp_error_s = 0.5 * 10.0 ** (-TIME_DECIMALS)
    target_error_rad = 0.5 * 10.0 ** (-TARGET_DECIMALS)
    m0_position_error_rad = (
        float(meta["velocities"][0]) * timestamp_error_s
        + target_error_rad
    )
    radial_error_mm = m0_position_error_rad * mm_per_rad
    schedule_tolerance_rad = m0_position_error_rad + 1e-9

    rows: list[dict[str, Any]] = []
    passes: list[dict[str, Any]] = []
    repacking_sides: list[dict[str, Any]] = []
    sign_counts = {-1: 0, 1: 0}
    for pass_index, window in enumerate(windows):
        t0 = float(window["motionStart"])
        p0 = float(timeline.axes[2].pos_at(t0))
        sign = 1 if bool(window["clockwise"]) else -1
        sign_counts[sign] += 1
        crossings = _directed_crossing_times(
            timeline.axes[2], t0, p0, sign, STATES_PER_PASS + 1)
        if len(crossings) != STATES_PER_PASS + 1:
            raise ValueError(
                f"pass {pass_index} has {len(crossings)} of 101 pi crossings")
        physical_turns = (
            abs(float(timeline.axes[2].pos_at(crossings[-1])) - p0)
            / (2.0 * math.pi))
        local_rows = []
        for state_index in range(STATES_PER_PASS):
            fraction = float(state_index) / STATES_PER_PASS
            time_s = float(crossings[state_index])
            phase = float(timeline.axes[2].pos_at(time_s))
            m0 = float(timeline.axes[0].pos_at(time_s))
            radial = (
                float(PARAMS.m0_home_standoff)
                + m0 * mm_per_rad
                - float(meta["job"]["wire_contact_z_mm"])
            )
            turn_index = state_index // 2
            scheduled = graph.turn(turn_index)
            # Packing graph radial -> exact M0 inverse of the same machine map.
            scheduled_m0 = (
                scheduled.radial_mm
                + float(meta["job"]["wire_contact_z_mm"])
                - float(PARAMS.m0_home_standoff)
            ) / mm_per_rad
            mismatch = abs(m0 - scheduled_m0)
            range_ok = bool(
                radial + radial_error_mm >= span[0]
                and radial - radial_error_mm <= span[1]
            )
            schedule_ok = mismatch <= schedule_tolerance_rad
            row = {
                "pass_index": pass_index,
                "tooth_index": int(window["tooth"]),
                "clockwise_argument": bool(window["clockwise"]),
                "motion_sign": sign,
                "state_index": state_index,
                "turn_index": turn_index,
                "half_turn_index": state_index & 1,
                "travel_fraction": fraction,
                "time_s": time_s,
                "m2_position_rad": phase,
                "m0_position_rad": m0,
                "radial_contact_mm": radial,
                "packed_schedule_m0_rad": scheduled_m0,
                "packed_schedule_radial_mm": scheduled.radial_mm,
                "m0_schedule_error_rad": mismatch,
                "radial_schedule_error_mm": mismatch * mm_per_rad,
                "inside_winding_span_with_error_bound": range_ok,
                "matches_packed_route_schedule_with_error_bound": schedule_ok,
            }
            rows.append(row)
            local_rows.append(row)
        passes.append({
            "pass_index": pass_index,
            "tooth_index": int(window["tooth"]),
            "clockwise_argument": bool(window["clockwise"]),
            "motion_sign": sign,
            "start_time_s": t0,
            "arrival_time_s": float(crossings[-1]),
            "start_m2_position_rad": p0,
            "target_m2_position_rad": float(
                timeline.axes[2].pos_at(crossings[-1])),
            "actual_logical_winding_travel_turns": physical_turns,
            "state_count": len(local_rows),
            "minimum_radial_contact_mm": min(
                row["radial_contact_mm"] for row in local_rows),
            "maximum_radial_contact_mm": max(
                row["radial_contact_mm"] for row in local_rows),
            "maximum_schedule_error_rad": max(
                row["m0_schedule_error_rad"] for row in local_rows),
            "states_matching_packed_schedule": sum(
                row["matches_packed_route_schedule_with_error_bound"]
                for row in local_rows),
        })
        for half in (0, 1):
            side_rows = [row for row in local_rows
                         if row["half_turn_index"] == half]
            radial = np.asarray(
                [row["radial_contact_mm"] for row in side_rows], dtype=float)
            pitches = np.abs(np.diff(radial))
            wire_d = float(graph.wire_diameter_mm)
            shortfall = np.maximum(0.0, wire_d - pitches)
            orthogonal = np.sqrt(np.maximum(
                0.0, wire_d * wire_d - np.minimum(pitches, wire_d) ** 2))
            below = pitches + 2.0 * radial_error_mm < wire_d
            nominal_below = pitches < wire_d - 1e-9
            zero_pitch = pitches <= 2.0 * radial_error_mm
            exact_zero_pitch = pitches < 1e-9
            repacking_sides.append({
                "pass_index": pass_index,
                "tooth_index": int(window["tooth"]),
                "motion_sign": sign,
                "half_turn_index": half,
                "turn_center_count": len(side_rows),
                "same_side_interval_count": len(pitches),
                "minimum_raw_radial_pitch_mm": float(np.min(pitches)),
                "maximum_raw_radial_pitch_mm": float(np.max(pitches)),
                "intervals_robustly_below_wire_diameter": int(np.sum(below)),
                "intervals_nominally_below_wire_diameter": int(
                    np.sum(nominal_below)),
                "zero_pitch_intervals_with_error_bound": int(
                    np.sum(zero_pitch)),
                "nominal_exact_zero_pitch_intervals": int(
                    np.sum(exact_zero_pitch)),
                "maximum_same_track_centerline_overlap_mm": float(
                    np.max(shortfall)),
                "maximum_minimum_orthogonal_repacking_mm": float(
                    np.max(orthogonal)),
                "pitch_values_mm": pitches.tolist(),
            })

    if len(rows) != EXPECTED_PASSES * STATES_PER_PASS:
        raise RuntimeError("raw state coverage is incomplete")
    range_passed = sum(
        row["inside_winding_span_with_error_bound"] for row in rows)
    schedule_passed = sum(
        row["matches_packed_route_schedule_with_error_bound"] for row in rows)
    profile_values = np.asarray(
        [turn.profile_radius_mm for turn in graph.turns], dtype=float)
    tangential_packing_envelope = float(
        np.max(profile_values) - np.min(profile_values))
    maximum_orthogonal_repacking = max(
        row["maximum_minimum_orthogonal_repacking_mm"]
        for row in repacking_sides)
    below_counts = [
        row["intervals_robustly_below_wire_diameter"]
        for row in repacking_sides
    ]
    nominal_below_counts = [
        row["intervals_nominally_below_wire_diameter"]
        for row in repacking_sides
    ]
    primary_nominal_counts = [
        row["intervals_nominally_below_wire_diameter"]
        for row in repacking_sides if row["half_turn_index"] == 0
    ]
    opposite_nominal_counts = [
        row["intervals_nominally_below_wire_diameter"]
        for row in repacking_sides if row["half_turn_index"] == 1
    ]
    direct_track_clear = not any(below_counts)
    local_lateral_room = bool(
        maximum_orthogonal_repacking <= tangential_packing_envelope + 1e-9)
    return {
        "status": "PASS" if range_passed == len(rows) else "FAIL",
        "sampling_contract": (
            "101 directed pi-spaced crossings from the first post-positioning "
            "M2 command in every raw wind_wire window; crossings 0..99 are "
            "the two sides of 50 deposited turns and crossing 100 closes the "
            "last turn; M0 is replayed concurrently with the upstream "
            "velocity-limited point-mass law"
        ),
        "pass_count": len(passes),
        "state_count": len(rows),
        "motion_sign_counts": {
            "negative": sign_counts[-1], "positive": sign_counts[1],
        },
        "both_motion_signs_covered": sign_counts == {-1: 12, 1: 12},
        "timestamp_quantization_error_s": timestamp_error_s,
        "target_quantization_error_rad": target_error_rad,
        "m0_position_error_bound_rad": m0_position_error_rad,
        "radial_position_error_bound_mm": radial_error_mm,
        "winding_span_mm": list(span),
        "states_inside_winding_span": range_passed,
        "states_matching_packed_route_schedule": schedule_passed,
        "packed_schedule_binding_status": (
            "PASS" if schedule_passed == len(rows) else "FAIL"
        ),
        "maximum_m0_schedule_error_rad": max(
            row["m0_schedule_error_rad"] for row in rows),
        "maximum_radial_schedule_error_mm": max(
            row["radial_schedule_error_mm"] for row in rows),
        "raw_repacking_demand": {
            "status": "NOT_PROVEN",
            "wire_finished_diameter_mm": graph.wire_diameter_mm,
            "same_side_interval_count": sum(
                row["same_side_interval_count"] for row in repacking_sides),
            "minimum_intervals_below_wire_diameter_per_side": min(below_counts),
            "maximum_intervals_below_wire_diameter_per_side": max(below_counts),
            "minimum_nominal_intervals_below_wire_diameter_any_lane": min(
                nominal_below_counts),
            "maximum_nominal_intervals_below_wire_diameter_any_lane": max(
                nominal_below_counts),
            "minimum_nominal_intervals_below_wire_diameter_primary_phase_lane": min(
                primary_nominal_counts),
            "maximum_nominal_intervals_below_wire_diameter_primary_phase_lane": max(
                primary_nominal_counts),
            "minimum_nominal_intervals_below_wire_diameter_opposite_phase_lane": min(
                opposite_nominal_counts),
            "maximum_nominal_intervals_below_wire_diameter_opposite_phase_lane": max(
                opposite_nominal_counts),
            "minimum_raw_radial_pitch_mm": min(
                row["minimum_raw_radial_pitch_mm"]
                for row in repacking_sides),
            "maximum_raw_radial_pitch_mm": max(
                row["maximum_raw_radial_pitch_mm"]
                for row in repacking_sides),
            "maximum_same_track_centerline_overlap_mm": max(
                row["maximum_same_track_centerline_overlap_mm"]
                for row in repacking_sides),
            "maximum_same_track_finished_diameter_compression_fraction": (
                max(row["maximum_same_track_centerline_overlap_mm"]
                    for row in repacking_sides) / graph.wire_diameter_mm),
            "maximum_same_track_overlap_as_total_enamel_build_fraction": (
                max(row["maximum_same_track_centerline_overlap_mm"]
                    for row in repacking_sides)
                / (graph.wire_diameter_mm
                   - REMINGTON_BARE_COPPER_DIAMETER_MM)),
            "minimum_zero_pitch_intervals_per_lane": min(
                row["zero_pitch_intervals_with_error_bound"]
                for row in repacking_sides),
            "maximum_zero_pitch_intervals_per_lane": max(
                row["zero_pitch_intervals_with_error_bound"]
                for row in repacking_sides),
            "every_primary_phase_pass_has_nominal_zero_pitch": all(
                row["nominal_exact_zero_pitch_intervals"] >= 1
                for row in repacking_sides
                if row["half_turn_index"] == 0),
            "maximum_minimum_orthogonal_repacking_mm": (
                maximum_orthogonal_repacking),
            "certified_packing_tangential_envelope_mm": (
                tangential_packing_envelope),
            "direct_same_track_nonoverlap": direct_track_clear,
            "local_orthogonal_relief_fits_certified_envelope": (
                local_lateral_room),
            "global_noncrossing_repacking_certificate": False,
            "interpretation": (
                "Low or zero raw radial pitch cannot be absorbed as elastic "
                "diameter compression: the zero-pitch case would coincide "
                "two copper centre-lines and demand one full finished-wire "
                "diameter of relief.  A new strand must instead slide "
                "laterally onto a different support branch by up to one wire "
                "diameter. "
                "The certified bundle has enough lateral envelope as a local "
                "necessary condition, but the raw motion supplies no "
                "tangential selector and this study found no constructive "
                "global branch/order proof without strand crossing."
            ),
            "sides": repacking_sides,
        },
        "minimum_actual_logical_winding_travel_turns": min(
            row["actual_logical_winding_travel_turns"] for row in passes),
        "maximum_actual_logical_winding_travel_turns": max(
            row["actual_logical_winding_travel_turns"] for row in passes),
        "passes": passes,
        "states": rows,
    }


def _shortest_contact_arc(
    source_xy: np.ndarray,
    target_xy: np.ndarray,
    center_xy: np.ndarray,
    radius_mm: float,
) -> tuple[float, int, float, float]:
    vector = source_xy - center_xy
    distance = float(np.linalg.norm(vector))
    if distance <= radius_mm:
        raise ValueError("contact-arc source is not outside parent copper")
    target_radius = float(np.linalg.norm(target_xy - center_xy))
    if abs(target_radius - radius_mm) > 1e-8:
        raise ValueError("contact-arc target is not tangent to parent copper")
    source_angle = math.atan2(vector[1], vector[0])
    offset = math.acos(radius_mm / distance)
    target_angle = math.atan2(
        target_xy[1] - center_xy[1], target_xy[0] - center_xy[0])
    tangent_length = math.sqrt(distance * distance - radius_mm * radius_mm)
    candidates = []
    for tangent_angle in (source_angle - offset, source_angle + offset):
        for direction in (-1, 1):
            arc = (direction * (target_angle - tangent_angle)) % (2.0 * math.pi)
            candidates.append((
                tangent_length + radius_mm * arc,
                direction, tangent_angle, arc,
            ))
    _, direction, tangent_angle, arc = min(candidates)
    return float(tangent_angle), int(direction), float(arc), tangent_length


def contact_detour(
    route_row: dict[str, Any],
    graph: PackingSupportGraph,
    *,
    step_deg: float = 0.125,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Construct exact parent contact over the terminal end-plane approach."""

    if int(route_row["turn_index"]) != CONTACT_TURN:
        raise ValueError("contact detour is only defined for turn 45")
    half = int(route_row["half_turn_index"])
    if half not in (0, 1):
        raise ValueError("half-turn index must be 0 or 1")
    points = np.asarray(route_row["route"]["points_local_mm"], dtype=float)
    if len(points) < 4:
        raise ValueError("route does not contain the guarded mouth approach")
    if points.ndim != 2 or points.shape[1] != 3 \
            or not np.all(np.isfinite(points)):
        raise ValueError("route points must be finite local xyz coordinates")
    target = points[-1]
    declared_target = np.asarray(route_row.get("target_local_mm"), dtype=float)
    if (declared_target.shape != (3,)
            or not np.allclose(target, declared_target, atol=1e-9,
                               rtol=0.0)):
        raise ValueError("contact route terminal does not match its target")
    expected_end_plane_z = (
        float(DEFAULT_STATOR.stack) / 2.0
        if half == 0 else -float(DEFAULT_STATOR.stack) / 2.0
    )
    if abs(float(target[2]) - expected_end_plane_z) > 1e-9:
        raise ValueError("contact target is not on the winding end plane")

    # Slot routes may insert any number of guide/torus samples before the
    # guarded mouth approach.  The former fixed ``points[-3]`` assumption
    # therefore stopped identifying the end-plane source as soon as that
    # upstream geometry became more detailed.  The route itself is the
    # authority: replace the complete trailing run which is coplanar with the
    # declared packed target.  This also preserves the older three-point
    # mouth representation without coupling this study to either point count.
    source_index = len(points) - 1
    while (source_index > 0
           and abs(float(points[source_index - 1, 2])
                   - expected_end_plane_z) <= 1e-9):
        source_index -= 1
    if source_index >= len(points) - 1:
        raise ValueError(
            "route does not contain a terminal end-plane approach")
    source = points[source_index]
    if float(np.linalg.norm(target[:2] - source[:2])) <= 1e-12:
        raise ValueError("terminal end-plane approach has zero xy length")

    segment_tags = route_row["route"].get("segment_tags")
    if segment_tags is not None:
        if len(segment_tags) != len(points) - 1:
            raise ValueError("route segment tags do not match route points")
        endpoint_support = route_row.get("endpoint_support")
        if (endpoint_support is not None
                and segment_tags[-1] != endpoint_support):
            raise ValueError(
                "terminal route segment does not match endpoint support")
    parent = graph.turn(CONTACT_PARENT)
    half_neck = max(2.5, float(DEFAULT_STATOR.od) * 0.07) / 2.0
    side = -1.0 if half == 0 else 1.0
    center = np.array((
        parent.radial_mm,
        side * (half_neck + parent.profile_radius_mm),
    ))
    radius = float(graph.wire_diameter_mm)
    tangent_angle, direction, arc_angle, tangent_length = (
        _shortest_contact_arc(source[:2], target[:2], center, radius))
    count = max(2, math.ceil(math.degrees(arc_angle) / float(step_deg)))
    angles = tangent_angle + direction * np.linspace(
        0.0, arc_angle, count + 1)
    arc = np.column_stack((
        center[0] + radius * np.cos(angles),
        center[1] + radius * np.sin(angles),
        np.full(len(angles), target[2]),
    ))
    repaired = np.vstack((points[:source_index + 1], arc))
    tangent_point = arc[0]
    line_direction = tangent_point[:2] - source[:2]
    radius_direction = tangent_point[:2] - center
    tangent_error = abs(float(np.dot(
        line_direction / np.linalg.norm(line_direction),
        radius_direction / np.linalg.norm(radius_direction),
    )))
    local = np.vstack((source, arc))
    metadata = {
        "turn_index": CONTACT_TURN,
        "half_turn_index": half,
        "parent_turn_index": CONTACT_PARENT,
        "source_local_mm": source.tolist(),
        "target_local_mm": target.tolist(),
        "end_plane_z_mm": expected_end_plane_z,
        "end_plane_source_point_index": source_index,
        "replaced_end_plane_point_count": len(points) - source_index - 1,
        "parent_contact_center_xy_mm": center.tolist(),
        "wire_center_exclusion_radius_mm": radius,
        "tangent_point_local_mm": tangent_point.tolist(),
        "contact_arc_angle_deg": math.degrees(arc_angle),
        "tangent_segment_length_mm": tangent_length,
        "contact_arc_length_mm": radius * arc_angle,
        "analytic_minimum_parent_centerline_distance_mm": radius,
        "analytic_local_bend_radius_mm": radius,
        "tangent_orthogonality_error": tangent_error,
        "local_contact_path_simple": bool(LineString(local[:, :2]).is_simple),
        "point_count": len(repaired),
    }
    return repaired, metadata


def contact_arc_convergence(
    route_row: dict[str, Any], graph: PackingSupportGraph,
) -> list[dict[str, Any]]:
    result = []
    radius = float(graph.wire_diameter_mm)
    previous = None
    for step in CONTACT_ROUTE_STEPS_DEG:
        _, meta = contact_detour(route_row, graph, step_deg=step)
        span = float(meta["contact_arc_angle_deg"])
        intervals = max(1, math.ceil(span / step))
        actual_step = math.radians(span / intervals)
        chord_min = radius * math.cos(actual_step / 2.0)
        sag = radius - chord_min
        row = {
            "requested_step_deg": step,
            "interval_count": intervals,
            "actual_max_step_deg": math.degrees(actual_step),
            "chordal_minimum_to_parent_center_mm": chord_min,
            "arc_sag_error_bound_mm": sag,
            "analytic_continuous_minimum_mm": radius,
            "change_from_previous_chordal_minimum_mm": (
                None if previous is None else chord_min - previous),
        }
        result.append(row)
        previous = chord_min
    return result


def _contact_geometry_audit(
    packing: dict[str, Any], route_report: dict[str, Any],
) -> dict[str, Any]:
    graph = PackingSupportGraph.from_report(packing, spec=DEFAULT_STATOR)
    contact_cases = [
        row for row in route_report["routes"]
        if int(row["turn_index"]) == CONTACT_TURN
    ]
    if (len(contact_cases) != 2
            or {(int(row["turn_index"]), int(row["half_turn_index"]))
                for row in contact_cases} != {(45, 0), (45, 1)}):
        raise ValueError("turn-45 route coverage is not both half-turns")
    failures = [
        row for row in contact_cases if row.get("status") != "PASS"
    ]
    manifest = _load_json(MANIFEST)
    planner = SlotRoutePlanner.from_project(
        manifest, spec=DEFAULT_STATOR,
        access_radius_mm=graph.center_core_access_mm,
        planner_offset_mm=graph.center_core_access_mm,
    )
    active = active_copper_before(
        graph, CONTACT_TURN, DEFAULT_STATOR,
        arc_step_deg=CONTACT_OBSTACLE_STEP_DEG)
    neighbours = (
        *neighbor_prefill_copper(
            graph, DEFAULT_STATOR, -1,
            arc_step_deg=CONTACT_OBSTACLE_STEP_DEG),
        *neighbor_prefill_copper(
            graph, DEFAULT_STATOR, 1,
            arc_step_deg=CONTACT_OBSTACLE_STEP_DEG),
    )
    declared_parent_ids = {
        f"active-turn-{index:02d}"
        for index in graph.turn(CONTACT_TURN).parent_turn_indices
    }
    nonparents = tuple(
        obstacle for obstacle in (*active, *neighbours)
        if obstacle.obstacle_id not in declared_parent_ids)
    nonparent_field = CopperField(nonparents)
    rows = []
    for route_row in sorted(
            contact_cases, key=lambda row: row["half_turn_index"]):
        repaired, meta = contact_detour(route_row, graph)
        core = exact_polyline_part_clearance(repaired, planner.stator_part)
        nonparent = nonparent_field.clearance(
            repaired, max(0.5, graph.wire_diameter_mm + 0.05))
        obstacle_profile_max = max(turn.profile_radius_mm for turn in graph.turns)
        obstacle_chord_error = (
            obstacle_profile_max
            * (1.0 - math.cos(math.radians(
                CONTACT_OBSTACLE_STEP_DEG) / 2.0))
        )
        nonparent_lower_bound = (
            nonparent.minimum_centerline_distance_mm - obstacle_chord_error)
        convergence = contact_arc_convergence(route_row, graph)
        finest_sag = convergence[-1]["arc_sag_error_bound_mm"]
        finished_radius = graph.wire_diameter_mm / 2.0
        bare_radius = REMINGTON_BARE_COPPER_DIAMETER_MM / 2.0
        checks = {
            "exact_parent_contact_no_penetration": (
                abs(meta["analytic_minimum_parent_centerline_distance_mm"]
                    - graph.wire_diameter_mm) <= 1e-12),
            "nonparent_copper_clearance": (
                nonparent_lower_bound + 1e-9 >= graph.wire_diameter_mm),
            "steel_core_clearance": (
                core + 1e-9 >= graph.center_core_access_mm),
            "simple_local_topology": bool(meta["local_contact_path_simple"]),
            "tangent_contact": meta["tangent_orthogonality_error"] <= 1e-12,
            "numerical_error_bounded": finest_sag <= 2e-7,
            "goal_minimum_bend_radius_3mm": (
                meta["analytic_local_bend_radius_mm"] + 1e-12
                >= float(PARAMS.min_bend_radius)),
        }
        geometry_checks = {
            name: value for name, value in checks.items()
            if name != "goal_minimum_bend_radius_3mm"
        }
        rows.append({
            **meta,
            "stored_route_status": route_row.get("status"),
            "was_rigid_failure": route_row.get("status") != "PASS",
            "status": "PASS" if all(checks.values()) else "FAIL",
            "geometric_contact_status": (
                "PASS" if all(geometry_checks.values()) else "FAIL"),
            "checks": checks,
            "minimum_core_center_distance_mm": core,
            "required_core_center_distance_mm": graph.center_core_access_mm,
            "minimum_nonparent_copper_centerline_distance_mm": (
                nonparent.minimum_centerline_distance_mm),
            "nonparent_obstacle_chord_error_bound_mm": obstacle_chord_error,
            "minimum_nonparent_copper_lower_bound_mm": nonparent_lower_bound,
            "minimum_nonparent_obstacle_id": nonparent.obstacle_id,
            "required_copper_centerline_distance_mm": graph.wire_diameter_mm,
            "bare_copper_outer_fibre_bending_strain_ratio": (
                bare_radius / meta["analytic_local_bend_radius_mm"]),
            "finished_wire_outer_fibre_bending_strain_ratio": (
                finished_radius / meta["analytic_local_bend_radius_mm"]),
            "convergence": convergence,
        })
    geometric_passes = sum(
        row["geometric_contact_status"] == "PASS" for row in rows)
    elastic_passes = sum(row["status"] == "PASS" for row in rows)
    original_overlap = min(
        float(row["planner_metadata"]["exact_release_postcheck"]
              ["parent_prefix_centerline_lower_bound_mm"])
        for row in contact_cases)
    required_overlap_relief = max(
        0.0, graph.wire_diameter_mm - original_overlap)
    enamel_total_diameter_build = (
        graph.wire_diameter_mm - REMINGTON_BARE_COPPER_DIAMETER_MM)
    return {
        "status": "PASS" if elastic_passes == 2 else "FAIL",
        "rigid_failure_case_count": len(failures),
        "contact_case_count": len(contact_cases),
        "current_rigid_geometry_pass_count": sum(
            row.get("status") == "PASS" for row in contact_cases),
        "failed_route_contact_geometric_pass_count": sum(
            row["was_rigid_failure"]
            and row["geometric_contact_status"] == "PASS"
            for row in rows),
        "contact_geometric_pass_count": geometric_passes,
        "elastic_curvature_pass_count": elastic_passes,
        "original_minimum_parent_prefix_distance_mm": original_overlap,
        "rigid_overlap_depth_mm": required_overlap_relief,
        "rigid_overlap_as_finished_diameter_fraction": (
            required_overlap_relief / graph.wire_diameter_mm),
        "compression_if_geometry_were_not_rerouted_as_total_enamel_build_fraction": (
            required_overlap_relief / enamel_total_diameter_build),
        "geometric_conclusion": (
            "The current turn-45 routes already pass the rigid geometry "
            "postcheck using a tagged support-normal end-plane approach.  "
            "The exact frictionless tangent/contact diagnostic remains "
            "nonpenetrating without a strand crossing."
            if not failures else
            "The turn-45 rigid failures are contact-model artifacts: an "
            "exact frictionless tangent/contact route exists without steel "
            "or non-parent copper penetration and without a strand crossing."
        ),
        "elastic_conclusion": (
            "The shortest exact contact route bends around one wire-centre "
            "diameter (0.22352 mm), below the 3 mm project minimum.  The "
            "study has not found or proved a larger-radius elastic "
            "equilibrium, and credits neither undocumented enamel "
            "compression nor a plastic copper hinge."
        ),
        "cases": rows,
    }


def analyze() -> dict[str, Any]:
    events = _load_jsonl(CAPTURE)
    capture = validate_capture_contract(events)
    packing = _load_json(PACKING)
    route_report = _load_json(ROUTES)
    graph = PackingSupportGraph.from_report(packing, spec=DEFAULT_STATOR)
    route_artifact = _validate_stored_route_payload(route_report, packing)
    timeline = Timeline(events)
    raw = replay_raw_winding_states(events, timeline, graph)
    contact = _contact_geometry_audit(packing, route_report)
    release_flags = {
        "authoritative_raw_capture_hash_bound": capture["status"] == "PASS",
        "stored_route_generator_sources_current": route_artifact[
            "generator_sources_current"],
        "all_2400_raw_winding_states_covered": raw["state_count"] == 2400,
        "both_raw_m2_signs_covered": raw["both_motion_signs_covered"],
        "all_raw_m0_states_inside_winding_span": (
            raw["states_inside_winding_span"] == raw["state_count"]),
        "raw_motion_matches_hash_bound_packing_schedule": (
            raw["packed_schedule_binding_status"] == "PASS"),
        "raw_low_pitch_repacking_has_global_noncrossing_proof": raw[
            "raw_repacking_demand"]["global_noncrossing_repacking_certificate"],
        "all_100_rigid_or_contact_routes_geometrically_nonpenetrating": (
            route_report["validation"]["passed_geometry_cases"]
            + contact["failed_route_contact_geometric_pass_count"]
            == route_report["validation"]["expected_geometry_cases"]),
        "contact_routes_meet_3mm_bend_contract": contact["status"] == "PASS",
        "complete_material_error_budget": False,
    }
    status = "PASS" if all(release_flags.values()) else "FAIL"
    payload: dict[str, Any] = {
        "schema": SCHEMA,
        "status": status,
        "decision": "FIXED_FLYER_NOT_PROVEN_WITHOUT_ACTIVE_TOOLING",
        "scope": {
            "question": (
                "Can the current fixed flyer lay all 50 turns under the "
                "authoritative raw upstream motion when prior-wire and "
                "liner frictionless contact/sliding and bounded elastic "
                "deformation are allowed?"
            ),
            "answer": (
                "Not proven.  The current turn-45 routes pass their rigid "
                "geometry postchecks, but the exact parent-contact "
                "diagnostic still has a sub-3 mm local radius and the raw "
                "asynchronous M0 law is not the packed schedule certified "
                "by slot_wire_routes.json."
            ),
            "production_sources_modified": False,
            "friction_model": "frictionless unilateral contact",
            "forbidden": [
                "steel penetration", "non-parent copper penetration",
                "strand crossing", "adhesive contact",
                "undocumented enamel compression", "plastic hinge credit",
            ],
        },
        "release_flags": release_flags,
        "release_blockers": [
            name for name, ok in release_flags.items() if not ok
        ],
        "raw_capture": capture,
        "stored_route_artifact": route_artifact,
        "raw_motion_replay": raw,
        "elastic_contact_reanalysis": contact,
        "material_contract": {
            "selected_wire": "Remington Industries 32SNSP.125",
            "finished_diameter_mm": graph.wire_diameter_mm,
            "bare_copper_diameter_mm": REMINGTON_BARE_COPPER_DIAMETER_MM,
            "insulation": "solderable polyurethane with polyamide overcoat",
            "nema_specification": "MW 80-C",
            "product_url": REMINGTON_PRODUCT_URL,
            "dimension_source_url": REMINGTON_DATA_URL,
            "published_numeric_elastic_contact_or_enamel_compression_limit": None,
            "fail_closed_effect": (
                "No material deformation is credited beyond exact "
                "nonpenetrating contact; missing lot-specific elastic and "
                "enamel compression data cannot turn this report PASS."
            ),
        },
        "numerics": {
            "dependency_versions": dependency_versions(),
            "axis_model": "piecewise constant velocity, exact target clamp",
            "root_solver": "monotone bisection",
            "root_bisection_steps": BISECTION_STEPS,
            "capture_time_decimals": TIME_DECIMALS,
            "capture_target_decimals": TARGET_DECIMALS,
            "contact_route_step_degrees": list(CONTACT_ROUTE_STEPS_DEG),
            "contact_obstacle_step_degrees": CONTACT_OBSTACLE_STEP_DEG,
            "continuous_contact_geometry": (
                "analytic line tangent plus exact circular parent-contact arc"
            ),
        },
        "source_hashes": {
            "raw_capture_sha256": _sha256(CAPTURE),
            "packing_file_sha256": _sha256(PACKING),
            "packing_report_sha256": packing["report_sha256"],
            "slot_wire_routes_file_sha256": _sha256(ROUTES),
            "slot_wire_routes_report_sha256": route_report["report_sha256"],
            "settings_sha256": _sha256(SETTINGS),
            "goal_sha256": _sha256(GOAL),
            "traj_source_sha256": _sha256(HERE / "traj.py"),
            "study_source_sha256": _sha256(Path(__file__)),
        },
        "limitations": [
            "This is quasi-static geometry/contact, not dynamic wire FEA.",
            "Friction, sag, residual plastic set, enamel abrasion, and actual "
            "lot material curves remain hardware/coupon measurements.",
            "A geometric contact route is not evidence that an uncontrolled "
            "wire will select that branch during reversal.",
            "The raw upstream ease-out-sine motion may produce a different "
            "valid packing, but no constructive noncrossing 50-turn contact "
            "solution for that different schedule exists in the current "
            "release artifacts.",
            "Raw same-side pitches below one wire diameter are allowed to "
            "repack laterally in the best-case contact model; they are not "
            "credited as resolved without a constructive global no-crossing "
            "branch certificate.",
        ],
    }
    payload["report_sha256"] = _canonical_hash(payload)
    return payload


def validate_report_integrity(report: dict[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported elastic-contact report schema")
    payload = dict(report)
    expected = payload.pop("report_sha256", None)
    if not isinstance(expected, str) or _canonical_hash(payload) != expected:
        raise ValueError("elastic-contact report hash mismatch")
    expected_sources = {
        "raw_capture_sha256": _sha256(CAPTURE),
        "packing_file_sha256": _sha256(PACKING),
        "slot_wire_routes_file_sha256": _sha256(ROUTES),
        "settings_sha256": _sha256(SETTINGS),
        "goal_sha256": _sha256(GOAL),
        "traj_source_sha256": _sha256(HERE / "traj.py"),
        "study_source_sha256": _sha256(Path(__file__)),
    }
    actual = report.get("source_hashes", {})
    stale = [name for name, value in expected_sources.items()
             if actual.get(name) != value]
    if stale:
        raise ValueError("elastic-contact report has stale sources: "
                         + ", ".join(stale))


def validate_release(report: dict[str, Any]) -> None:
    validate_report_integrity(report)
    if report.get("status") != "PASS":
        raise ValueError("elastic-contact report is not PASS")
    if not all(report.get("release_flags", {}).values()):
        raise ValueError("elastic-contact release flags are incomplete")


def render_markdown(report: dict[str, Any]) -> str:
    raw = report["raw_motion_replay"]
    repacking = raw["raw_repacking_demand"]
    contact = report["elastic_contact_reanalysis"]
    flags = report["release_flags"]
    lines = [
        "# Elastic wire/contact feasibility study",
        "",
        f"**Overall status: {report['status']}**  ",
        f"**Decision: {report['decision']}**",
        "",
        report["scope"]["answer"],
        "",
        "## What changed relative to the rigid route audit",
        "",
        contact["geometric_conclusion"],
        "",
        contact["elastic_conclusion"],
        "",
        f"- Current rigid turn-45 routes passing: {contact['current_rigid_geometry_pass_count']}/2",
        f"- Rigid overlap depth requiring contact repair: {contact['rigid_overlap_depth_mm'] * 1000:.3f} um",
        f"- Contact diagnostics geometrically clear: {contact['contact_geometric_pass_count']}/2",
        f"- Contact geometry meeting 3 mm bend limit: {contact['elastic_curvature_pass_count']}/2",
        "",
        "## Authoritative raw motion",
        "",
        f"- Capture SHA-256: `{report['raw_capture']['sha256']}`",
        f"- Passes: {raw['pass_count']}; states: {raw['state_count']}",
        f"- M2 signs: {raw['motion_sign_counts']}",
        f"- States inside winding span: {raw['states_inside_winding_span']}/{raw['state_count']}",
        f"- States matching the current packed route schedule: {raw['states_matching_packed_route_schedule']}/{raw['state_count']}",
        f"- Worst M0 schedule mismatch: {raw['maximum_m0_schedule_error_rad']:.6f} rad ({raw['maximum_radial_schedule_error_mm']:.6f} mm)",
        f"- Primary phase lane intervals nominally below one wire diameter: {repacking['minimum_nominal_intervals_below_wire_diameter_primary_phase_lane']}..{repacking['maximum_nominal_intervals_below_wire_diameter_primary_phase_lane']} of 49 per pass",
        f"- Opposite phase lane intervals nominally below one wire diameter: {repacking['minimum_nominal_intervals_below_wire_diameter_opposite_phase_lane']}..{repacking['maximum_nominal_intervals_below_wire_diameter_opposite_phase_lane']} of 49 per pass",
        f"- Intervals robustly below diameter after timestamp error: {repacking['minimum_intervals_below_wire_diameter_per_side']}..{repacking['maximum_intervals_below_wire_diameter_per_side']} of 49 across both lanes",
        f"- Raw radial pitch: {repacking['minimum_raw_radial_pitch_mm']:.6f}..{repacking['maximum_raw_radial_pitch_mm']:.6f} mm",
        f"- Required best-case lateral repacking: up to {repacking['maximum_minimum_orthogonal_repacking_mm']:.6f} mm",
        f"- Global noncrossing repacking proof: {repacking['global_noncrossing_repacking_certificate']}",
        "",
        "The raw ease-out-sine M0 trajectory can still conceivably settle into "
        "a different packing.  This report does not treat that possibility as "
        "proof: the current 50-turn route table certifies a different, "
        "hash-bound placement order.",
        "",
        "## Release flags",
        "",
    ]
    for name, ok in flags.items():
        lines.append(f"- {'PASS' if ok else 'FAIL'} — `{name}`")
    lines.extend((
        "",
        "## Honest limit",
        "",
        "No purchase or hardware motion is authorized by this study.  A "
        "larger-radius elastic equilibrium or an active/passive former must be "
        "proved against the raw asynchronous motion before the fixed flyer can "
        "be called winding-ready.",
        "",
    ))
    return "\n".join(lines)


def write_outputs(report: dict[str, Any]) -> None:
    OUTPUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    OUTPUT_MD.write_text(render_markdown(report), encoding="utf-8")


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", type=Path, default=OUTPUT_JSON)
    parser.add_argument("--markdown", type=Path, default=OUTPUT_MD)
    args = parser.parse_args(list(argv) if argv is not None else None)
    report = analyze()
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    args.markdown.parent.mkdir(parents=True, exist_ok=True)
    args.markdown.write_text(render_markdown(report), encoding="utf-8")
    print(
        f"elastic wire/contact {report['status']}: "
        f"geometry contact "
        f"{report['elastic_contact_reanalysis']['contact_geometric_pass_count']}/2; "
        f"raw schedule matches "
        f"{report['raw_motion_replay']['states_matching_packed_route_schedule']}/"
        f"{report['raw_motion_replay']['state_count']}"
    )
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
