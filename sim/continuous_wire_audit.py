"""Fail-closed audit of the captured, continuously moving winding wire.

``slot_wire_routes.py`` proves isolated half-turn crossing poses.  That is not
enough to prove the *motion between* those poses: the wire is passive, its
contact point is not an axis, and an unobserved contact profile must never be
replaced by an invented "safe-mouth" trajectory.

This module binds the capture, winding plan, packing proof, and crossing-route
table by SHA-256, reconstructs actual M0/M1/M2 command and arrival states with
``traj.Timeline``, and partitions the complete captured wire-live window at:

* every command and reconstructed arrival;
* every high-level/packing event;
* exact M2 half-turn roots; and
* conservative adaptive motion subdivisions.

The resulting intervals pass only when a hash-bound contact-envelope report
provides either OBSERVED contact evidence or a UNIVERSALLY_BOUNDED proof for
every interval.  A swept interval consumes clearance according to

    |dM0| * mm_per_rad
      + 2 * flyer_radius * sin(|dM2| / 2)
      + 2 * stator_radius * sin(|dM1| / 2)
      + contact_uncertainty.

Progressive copper is explicit.  During the closing half of a turn, the first
half of that same conductor must be present; only one short adjacent self
segment may be excluded.  Missing/stale evidence is NOT_PROVEN or FAIL, never
silently accepted.
"""

from __future__ import annotations

import argparse
from bisect import bisect_right
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from traj import Timeline, load_events


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORTS = ROOT / "out" / "reports"
CAPTURE_PATH = ROOT / "out" / "capture" / "commands.jsonl"
PLAN_PATH = REPORTS / "slot_winding_plan.json"
PACKING_PATH = REPORTS / "slot_packing.json"
ROUTES_PATH = REPORTS / "slot_wire_routes.json"
CONTACT_PATH = REPORTS / "wire_contact_envelope.json"
OUTPUT_PATH = REPORTS / "continuous_wire_audit.json"

SCHEMA = "continuous-wire-audit/v1"
CONTACT_SCHEMA = "continuous-wire-contact/v1"
UNIVERSAL_PROOF_SCHEMA = "continuous-wire-universal-proof/v1"
TIME_EPS_S = 1e-7
PHASE_EPS_RAD = 1e-6
DEFAULT_MM_PER_RAD_M0 = 8.0 / (2.0 * math.pi)
DEFAULT_FLYER_RADIUS_MM = 60.0
DEFAULT_MAX_ELEMENT_MOTION_MM = 0.25


@dataclass(frozen=True)
class AuditConfig:
    """Numerical policy for partitioning and fail-closed comparisons."""

    max_element_motion_mm: float = DEFAULT_MAX_ELEMENT_MOTION_MM
    default_flyer_radius_mm: float = DEFAULT_FLYER_RADIUS_MM
    mm_per_rad_m0: float = DEFAULT_MM_PER_RAD_M0
    time_tolerance_s: float = TIME_EPS_S
    phase_tolerance_rad: float = PHASE_EPS_RAD


@dataclass
class Issue:
    severity: str
    code: str
    message: str
    context: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "severity": self.severity,
            "code": self.code,
            "message": self.message,
            "context": self.context,
        }


@dataclass(frozen=True)
class PassWindow:
    pass_index: int
    start_t: float
    end_t: float
    origin_event_t: float
    captured_pass_end_t: float
    phase_zero_m2_rad: float
    direction: int
    start_phase_rad: float
    phase_origin_rad: float
    actual_travel_rad: float
    turns: int

    def phase_at(self, timeline: Timeline, t: float) -> float:
        return self.direction * (
            timeline.axes[2].pos_at(t) - self.phase_zero_m2_rad)


@dataclass(frozen=True)
class ElementaryInterval:
    index: int
    t0: float
    t1: float
    pose0: tuple[float, float, float]
    pose1: tuple[float, float, float]
    start_reasons: tuple[str, ...]
    end_reasons: tuple[str, ...]
    kind: str
    pass_index: int | None
    half_turn_index: int | None
    transition_index: int | None

    def case_key(self) -> tuple[Any, ...]:
        if self.kind == "packing":
            return ("packing", self.pass_index, self.half_turn_index)
        return ("transition", self.transition_index)


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    text = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(text.encode()).hexdigest()


def _without(mapping: Mapping[str, Any], key: str) -> dict[str, Any]:
    result = dict(mapping)
    result.pop(key, None)
    return result


def _is_sha256(value: Any) -> bool:
    if not isinstance(value, str) or len(value) != 64:
        return False
    try:
        int(value, 16)
    except ValueError:
        return False
    return True


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _close(a: Any, b: Any, tolerance: float = 1e-9) -> bool:
    return _finite(a) and _finite(b) and math.isclose(
        float(a), float(b), rel_tol=0.0, abs_tol=tolerance)


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text())
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def _check_shaft_wrap_phase_contract(events: Sequence[Mapping[str, Any]],
                                     meta: Mapping[str, Any],
                                     issues: list[Issue]) -> None:
    """Require explicit free-tail parking and sleeve-contact boundaries."""
    if int(meta.get("capture_schema", -1)) < 4:
        return
    rows = [event for event in events
            if event.get("e") == "shaft_wrap_phase"]
    required = [
        "prepark_start", "m0_parked", "contact_start", "contact_done",
    ]
    contract = meta.get("shaft_wrap_contract")
    if not isinstance(contract, Mapping):
        issues.append(Issue(
            "FAIL", "shaft_wrap_contract_missing",
            "schema-4 capture has no bound shaft-wrap park contract"))
        return
    report_path_value = contract.get("refinement_report")
    report_path = (Path(report_path_value)
                   if isinstance(report_path_value, str) else None)
    current_hash = (_sha256(report_path)
                    if report_path is not None and report_path.is_file()
                    else None)
    if (contract.get("refinement_status") != "PASS"
            or not _finite(contract.get(
                "residual_clearance_after_budget_mm"))
            or float(contract["residual_clearance_after_budget_mm"]) <= 0.0
            or contract.get("refinement_report_sha256") != current_hash):
        issues.append(Issue(
            "FAIL", "shaft_wrap_refinement_stale",
            "shaft-wrap park is not bound to a current positive-residual "
            "refinement report"))

    try:
        m0_park = float(contract["m0_park_rad"])
        m2_phase = float(contract["m2_park_phase_rad"])
        reference = float(contract["machine_m2_reference_rad"])
    except (KeyError, TypeError, ValueError):
        issues.append(Issue(
            "FAIL", "shaft_wrap_contract_invalid",
            "shaft-wrap contract lacks finite M0/M2 park coordinates"))
        return

    if len(rows) != 8:
        issues.append(Issue(
            "FAIL", "shaft_wrap_phase_count",
            f"expected eight shaft-wrap phase markers, found {len(rows)}"))
    for number in (1, 2):
        group = [row for row in rows
                 if row.get("next_wire_idx") == number]
        if [row.get("phase") for row in group] != required:
            issues.append(Issue(
                "FAIL", "shaft_wrap_phase_sequence",
                f"shaft wrap {number} marker sequence is incomplete",
                {"observed": [row.get("phase") for row in group]}))
            continue
        start, done = group[2], group[3]
        parked_m0 = abs(float(start["m0_rad"]) - m0_park) <= 0.0035
        parked_m2 = abs(
            ((float(start["m2_rad"]) - reference - m2_phase + math.pi)
             % (2.0 * math.pi)) - math.pi
        ) <= 0.005
        fixed = (
            _close(done.get("m0_rad"), start.get("m0_rad"), 1e-9)
            and _close(done.get("m2_rad"), start.get("m2_rad"), 1e-9)
        )
        turns = abs(float(done["m1_rad"]) - float(start["m1_rad"])) \
            / (2.0 * math.pi)
        if not (parked_m0 and parked_m2 and fixed
                and abs(turns - 2.0) <= 1e-9):
            issues.append(Issue(
                "FAIL", "shaft_wrap_contact_pose",
                f"shaft wrap {number} contact interval violates its fixed "
                "park/two-turn contract",
                {"m0_parked": parked_m0, "m2_parked": parked_m2,
                 "m0_m2_fixed": fixed, "turns": turns}))


def _source_path(source_root: Path, name: str) -> Path | None:
    """Resolve source-hash keys used by the packing/plan/route reports."""

    normalized = Path(str(name).replace("\\", "/"))
    candidates = [source_root / normalized]
    if len(normalized.parts) == 1:
        candidates.extend((source_root / "cad" / normalized,
                           source_root / "sim" / normalized))
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return None


def _check_source_hashes(report_name: str,
                         source_hashes: Any,
                         source_root: Path,
                         issues: list[Issue]) -> None:
    if not isinstance(source_hashes, dict) or not source_hashes:
        issues.append(Issue(
            "FAIL", f"{report_name}_source_hashes_missing",
            f"{report_name} has no non-empty source_hashes map"))
        return
    for name, expected in sorted(source_hashes.items()):
        path = _source_path(source_root, str(name))
        if path is None:
            issues.append(Issue(
                "FAIL", f"{report_name}_source_missing",
                f"{report_name} source does not exist: {name}"))
            continue
        actual = _sha256(path)
        if actual != expected:
            issues.append(Issue(
                "FAIL", f"{report_name}_source_stale",
                f"{report_name} source hash is stale: {name}",
                {"expected": expected, "actual": actual}))


def _check_self_hash(report_name: str,
                     report: Mapping[str, Any],
                     field_name: str,
                     issues: list[Issue]) -> None:
    expected = report.get(field_name)
    if not _is_sha256(expected):
        issues.append(Issue(
            "FAIL", f"{report_name}_hash_missing",
            f"{report_name}.{field_name} is missing or malformed"))
        return
    actual = _canonical_hash(_without(report, field_name))
    if actual != expected:
        issues.append(Issue(
            "FAIL", f"{report_name}_hash_mismatch",
            f"{report_name}.{field_name} does not match its payload",
            {"expected": expected, "actual": actual}))


def _event_time(event: Mapping[str, Any]) -> float:
    value = event.get("t")
    if not _finite(value):
        raise ValueError(f"event has invalid time: {event!r}")
    return float(value)


def _motion_bound(pose0: Sequence[float], pose1: Sequence[float],
                  *, mm_per_rad_m0: float,
                  flyer_radius_mm: float,
                  stator_radius_mm: float,
                  contact_uncertainty_mm: float = 0.0) -> dict[str, float]:
    dm0 = abs(float(pose1[0]) - float(pose0[0]))
    dm1 = abs(float(pose1[1]) - float(pose0[1]))
    dm2 = abs(float(pose1[2]) - float(pose0[2]))
    m0 = dm0 * mm_per_rad_m0
    # Chord displacement cannot exceed a diameter.  Exact half-turn roots in
    # the partition ensure ordinary winding elements stay below pi anyway.
    flyer = 2.0 * flyer_radius_mm * math.sin(min(math.pi, dm2) / 2.0)
    stator = 2.0 * stator_radius_mm * math.sin(min(math.pi, dm1) / 2.0)
    total = m0 + flyer + stator + contact_uncertainty_mm
    return {
        "m0_translation_mm": m0,
        "flyer_rotation_chord_mm": flyer,
        "stator_rotation_chord_mm": stator,
        "contact_uncertainty_mm": contact_uncertainty_mm,
        "total_mm": total,
    }


def _add_boundary(boundaries: dict[float, set[str]],
                  t: float, reason: str,
                  start_t: float, end_t: float) -> None:
    if t < start_t - TIME_EPS_S or t > end_t + TIME_EPS_S:
        return
    t = min(end_t, max(start_t, float(t)))
    key = round(t, 10)
    boundaries.setdefault(key, set()).add(reason)


def _roots_on_track(track: Any, start_t: float, end_t: float,
                    targets: Iterable[float]) -> list[float]:
    """Solve exact target crossings on a piecewise-linear AxisTrack."""

    knots = sorted({start_t, end_t, *(
        float(t) for t, _ in track.knots
        if start_t < float(t) < end_t)})
    target_values = tuple(sorted(set(float(value) for value in targets)))
    roots: list[float] = []
    for ta, tb in zip(knots, knots[1:]):
        pa, pb = float(track.pos_at(ta)), float(track.pos_at(tb))
        if math.isclose(pa, pb, rel_tol=0.0, abs_tol=1e-15):
            continue
        low, high = sorted((pa, pb))
        for target in target_values:
            if target < low - 1e-12 or target > high + 1e-12:
                continue
            fraction = (target - pa) / (pb - pa)
            if -1e-12 <= fraction <= 1.0 + 1e-12:
                roots.append(ta + (tb - ta) * fraction)
    return roots


def _derive_pass_windows(events: Sequence[Mapping[str, Any]],
                         timeline: Timeline,
                         turns: int,
                         issues: list[Issue],
                         config: AuditConfig) -> list[PassWindow]:
    origins: dict[int, Mapping[str, Any]] = {}
    waypoints: dict[int, list[Mapping[str, Any]]] = {}
    for event in events:
        if event.get("e") == "packing_pass_origin":
            index = event.get("pass_index")
            if not isinstance(index, int) or index in origins:
                issues.append(Issue(
                    "FAIL", "packing_origin_duplicate_or_invalid",
                    "packing pass origins must have unique integer indices"))
                continue
            origins[index] = event
        elif event.get("e") == "packing_waypoint":
            index = event.get("pass_index")
            if not isinstance(index, int):
                issues.append(Issue(
                    "FAIL", "packing_waypoint_pass_invalid",
                    "packing waypoint pass_index must be an integer"))
                continue
            waypoints.setdefault(index, []).append(event)

    windows: list[PassWindow] = []
    expected_centers = 2 * turns
    for pass_index in sorted(origins):
        origin = origins[pass_index]
        rows = sorted(waypoints.get(pass_index, []),
                      key=lambda row: int(row.get("waypoint_index", -1)))
        if not rows:
            issues.append(Issue(
                "FAIL", "packing_waypoint_coverage",
                f"pass {pass_index} has no packing waypoints"))
            continue
        indices = [row.get("waypoint_index") for row in rows]
        if indices != list(range(len(rows))):
            issues.append(Issue(
                "FAIL", "packing_waypoint_sequence",
                f"pass {pass_index} waypoint indices are not exact"))
            continue
        if any(_event_time(a) > _event_time(b) + config.time_tolerance_s
               for a, b in zip(rows, rows[1:])):
            issues.append(Issue(
                "FAIL", "packing_waypoint_time_regression",
                f"pass {pass_index} waypoint times regress"))
            continue
        centers = [row for row in rows
                   if row.get("kind") == "placement_center"]
        holds = [row for row in rows if row.get("kind") == "final_hold"]
        if len(centers) != expected_centers:
            issues.append(Issue(
                "FAIL", "packing_center_coverage",
                f"pass {pass_index} has {len(centers)} placement centers; "
                f"expected {expected_centers}"))
            continue
        if rows[:expected_centers] != centers or rows[expected_centers:] != holds:
            issues.append(Issue(
                "FAIL", "packing_waypoint_kind_order",
                f"pass {pass_index} must record all centers before lead-out holds"))
            continue

        origin_t = _event_time(origin)
        captured_end_t = _event_time(rows[-1])
        origin_m2 = float(timeline.axes[2].pos_at(origin_t))
        end_m2 = float(timeline.axes[2].pos_at(captured_end_t))
        delta = end_m2 - origin_m2
        if abs(delta) <= config.phase_tolerance_rad:
            issues.append(Issue(
                "FAIL", "packing_m2_no_travel",
                f"pass {pass_index} has no reconstructed M2 travel"))
            continue
        direction = 1 if delta > 0.0 else -1
        start_phase = float(origin.get("start_phase_rad", math.nan))
        phase_origin = float(origin.get("phase_origin_rad", math.nan))
        first_crossing = float(origin.get(
            "first_crossing_phase_rad", math.nan))
        travel = float(origin.get("actual_travel_rad", math.nan))
        if not all(_finite(value) for value in (
                start_phase, phase_origin, first_crossing, travel)):
            issues.append(Issue(
                "FAIL", "packing_origin_phase_invalid",
                f"pass {pass_index} has invalid origin phase metadata"))
            continue
        if not math.isclose(phase_origin, first_crossing, rel_tol=0.0,
                            abs_tol=config.phase_tolerance_rad):
            issues.append(Issue(
                "FAIL", "packing_phase_origin_mismatch",
                f"pass {pass_index} phase_origin is not its first crossing"))
        phase_zero_m2 = origin_m2 - direction * start_phase
        reconstructed_end_phase = direction * (end_m2 - phase_zero_m2)
        if not math.isclose(
                reconstructed_end_phase, travel, rel_tol=0.0,
                abs_tol=max(config.phase_tolerance_rad, 1e-4)):
            issues.append(Issue(
                "FAIL", "packing_travel_mismatch",
                f"pass {pass_index} captured M2 travel disagrees with origin",
                {"timeline_final_phase_rad": reconstructed_end_phase,
                 "declared_travel_rad": travel}))

        for center_index, row in enumerate(centers):
            target_phase = float(row.get("m2_phase_rad", math.nan))
            expected_phase = first_crossing + center_index * math.pi
            if not math.isclose(
                    target_phase, expected_phase, rel_tol=0.0,
                    abs_tol=config.phase_tolerance_rad):
                issues.append(Issue(
                    "FAIL", "packing_target_phase_sequence",
                    f"pass {pass_index} center {center_index} phase drifted",
                    {"target_rad": target_phase,
                     "expected_rad": expected_phase}))
        previous_phase = first_crossing + (expected_centers - 1) * math.pi
        for hold_index, row in enumerate(holds):
            target_phase = float(row.get("m2_phase_rad", math.nan))
            if not _finite(target_phase) or target_phase <= previous_phase + 1e-12:
                issues.append(Issue(
                    "FAIL", "packing_final_hold_phase_sequence",
                    f"pass {pass_index} final hold {hold_index} is not later "
                    "than the preceding crossing"))
            previous_phase = target_phase
        if not _close(rows[-1].get("m2_phase_rad"), travel,
                      max(config.phase_tolerance_rad, 1e-6)):
            issues.append(Issue(
                "FAIL", "packing_final_phase_missing",
                f"pass {pass_index} has no waypoint at actual M2 travel"))

        # A complete 50-turn loop needs one closing half-turn after the last
        # of its 100 placement-center crossings.  If upstream travel ends at
        # the last center, the final conductor segment is genuinely missing.
        closure_phase = first_crossing + expected_centers * math.pi
        closure_rows = [row for row in rows if _close(
            row.get("m2_phase_rad"), closure_phase,
            config.phase_tolerance_rad)]
        if len(closure_rows) != 1:
            issues.append(Issue(
                "FAIL", "packing_closure_crossing_missing",
                f"pass {pass_index} has no unique closing crossing at "
                f"{closure_phase:.9f} rad"))
            continue

        window = PassWindow(
            pass_index=pass_index,
            start_t=_event_time(centers[0]),
            end_t=_event_time(closure_rows[0]),
            origin_event_t=origin_t,
            captured_pass_end_t=captured_end_t,
            phase_zero_m2_rad=phase_zero_m2,
            direction=direction,
            start_phase_rad=start_phase,
            phase_origin_rad=first_crossing,
            actual_travel_rad=travel,
            turns=turns,
        )

        for expected_index, row in enumerate(rows):
            target_phase = float(row.get("m2_phase_rad", math.nan))
            observed_phase = float(row.get(
                "observed_m2_phase_rad", math.nan))
            computed_phase = window.phase_at(timeline, _event_time(row))
            # The controller may observe just after a target, but it may not
            # claim a crossing before the actual reconstructed axis reaches it.
            if (computed_phase + config.phase_tolerance_rad < target_phase
                    or observed_phase + config.phase_tolerance_rad
                    < target_phase):
                issues.append(Issue(
                    "FAIL", "packing_early_phase_acceptance",
                    f"pass {pass_index} waypoint {expected_index} was "
                    "accepted before its physical phase",
                    {"target_rad": target_phase,
                     "timeline_phase_rad": computed_phase,
                     "observed_phase_rad": observed_phase}))
            if row.get("m0_settled_before_crossing") is not True:
                issues.append(Issue(
                    "FAIL", "packing_m0_not_settled",
                    f"pass {pass_index} waypoint {expected_index} lacks "
                    "settled-M0 evidence"))
            if not _finite(row.get("m0_error_rad")):
                issues.append(Issue(
                    "FAIL", "packing_m0_error_invalid",
                    f"pass {pass_index} waypoint {expected_index} has no "
                    "finite M0 error"))
        windows.append(window)

    return windows


def _capture_scope(events: Sequence[Mapping[str, Any]],
                   issues: list[Issue]) -> tuple[float, float] | None:
    starts = [_event_time(event) for event in events
              if event.get("e") == "wind_wire"]
    ends = [_event_time(event) for event in events
            if event.get("e") == "wind_wire_done"]
    if not starts or not ends:
        issues.append(Issue(
            "FAIL", "wire_live_markers_missing",
            "capture must contain wind_wire and wind_wire_done markers"))
        return None
    start_t, end_t = min(starts), max(ends)
    if end_t <= start_t:
        issues.append(Issue(
            "FAIL", "wire_live_window_invalid",
            "captured wire-live window is empty or reversed"))
        return None
    return start_t, end_t


def _partition(events: Sequence[Mapping[str, Any]],
               timeline: Timeline,
               windows: Sequence[PassWindow],
               scope: tuple[float, float],
               *, flyer_radius_mm: float,
               stator_radius_mm: float,
               config: AuditConfig) -> tuple[list[ElementaryInterval],
                                              dict[str, Any]]:
    start_t, end_t = scope
    boundaries: dict[float, set[str]] = {}
    _add_boundary(boundaries, start_t, "wire_live_start", start_t, end_t)
    _add_boundary(boundaries, end_t, "wire_live_end", start_t, end_t)

    command_times: dict[int, set[float]] = {0: set(), 1: set(), 2: set()}
    for event in events:
        t = _event_time(event)
        name = str(event.get("e"))
        if name == "cmd" and event.get("m") in (0, 1, 2):
            command_times[int(event["m"])].add(round(t, 10))
            _add_boundary(boundaries, t, "command", start_t, end_t)
            _add_boundary(
                boundaries, t, f"command_m{event['m']}", start_t, end_t)
        elif name != "meta":
            reason = "packing_event" if name.startswith("packing_") \
                else "capture_event"
            _add_boundary(boundaries, t, reason, start_t, end_t)

    for axis_id in (0, 1, 2):
        for t, _position in timeline.axes[axis_id].knots:
            if round(float(t), 10) not in command_times[axis_id]:
                _add_boundary(
                    boundaries, float(t), "arrival", start_t, end_t)
                _add_boundary(
                    boundaries, float(t), f"arrival_m{axis_id}",
                    start_t, end_t)

    # Absolute M2 half-turn roots cover setup, indexing, shaft-wrap, and
    # teardown motion.  Packing-relative roots are added below as well.
    m2_track = timeline.axes[2]
    probe_times = sorted({start_t, end_t, *(
        float(t) for t, _ in m2_track.knots
        if start_t <= float(t) <= end_t)})
    positions = [float(m2_track.pos_at(t)) for t in probe_times]
    if positions:
        n_min = math.floor(min(positions) / math.pi) - 1
        n_max = math.ceil(max(positions) / math.pi) + 1
        absolute_targets = (n * math.pi for n in range(n_min, n_max + 1))
        for root in _roots_on_track(
                m2_track, start_t, end_t, absolute_targets):
            _add_boundary(
                boundaries, root, "m2_halfturn_root", start_t, end_t)

    for window in windows:
        targets = (
            window.phase_zero_m2_rad
            + window.direction * (window.phase_origin_rad + k * math.pi)
            for k in range(2 * window.turns + 1)
        )
        for root in _roots_on_track(
                m2_track, window.start_t, window.end_t, targets):
            _add_boundary(
                boundaries, root, "packing_halfturn_root", start_t, end_t)

    base_times = sorted(boundaries)
    for ta, tb in zip(base_times, base_times[1:]):
        if tb - ta <= config.time_tolerance_s:
            continue
        bound = _motion_bound(
            timeline.pose_at(ta), timeline.pose_at(tb),
            mm_per_rad_m0=config.mm_per_rad_m0,
            flyer_radius_mm=flyer_radius_mm,
            stator_radius_mm=stator_radius_mm)
        pieces = max(1, int(math.ceil(
            bound["total_mm"] / config.max_element_motion_mm)))
        for index in range(1, pieces):
            t = ta + (tb - ta) * index / pieces
            _add_boundary(
                boundaries, t, "adaptive_subdivision", start_t, end_t)

    times = sorted(boundaries)
    raw: list[dict[str, Any]] = []
    for index, (t0, t1) in enumerate(zip(times, times[1:])):
        if t1 - t0 <= config.time_tolerance_s:
            continue
        midpoint = (t0 + t1) / 2.0
        containing = [window for window in windows
                      if window.start_t - config.time_tolerance_s <= midpoint
                      <= window.end_t + config.time_tolerance_s]
        if len(containing) > 1:
            raise ValueError("packing pass windows overlap")
        if containing:
            window = containing[0]
            logical = window.phase_at(timeline, midpoint)
            half = int(math.floor(
                (logical - window.phase_origin_rad) / math.pi
                + config.phase_tolerance_rad))
            half = min(2 * window.turns - 1, max(0, half))
            kind = "packing"
            pass_index = window.pass_index
        else:
            half = None
            kind = "transition"
            pass_index = None
        raw.append({
            "index": index,
            "t0": t0,
            "t1": t1,
            "pose0": tuple(float(value) for value in timeline.pose_at(t0)),
            "pose1": tuple(float(value) for value in timeline.pose_at(t1)),
            "start_reasons": tuple(sorted(boundaries[t0])),
            "end_reasons": tuple(sorted(boundaries[t1])),
            "kind": kind,
            "pass_index": pass_index,
            "half_turn_index": half,
        })

    transition_index = -1
    in_transition = False
    intervals: list[ElementaryInterval] = []
    for record in raw:
        if record["kind"] == "transition":
            if not in_transition:
                transition_index += 1
                in_transition = True
            record["transition_index"] = transition_index
        else:
            in_transition = False
            record["transition_index"] = None
        intervals.append(ElementaryInterval(**record))

    reason_counts: dict[str, int] = {}
    for reasons in boundaries.values():
        for reason in reasons:
            reason_counts[reason] = reason_counts.get(reason, 0) + 1
    return intervals, {
        "wire_live_start_s": start_t,
        "wire_live_end_s": end_t,
        "wire_live_duration_s": end_t - start_t,
        "boundary_count": len(boundaries),
        "elementary_interval_count": len(intervals),
        "packing_elementary_interval_count": sum(
            interval.kind == "packing" for interval in intervals),
        "transition_elementary_interval_count": sum(
            interval.kind == "transition" for interval in intervals),
        "transition_segment_count": transition_index + 1,
        "boundary_reason_counts": reason_counts,
        "maximum_adaptive_motion_mm": config.max_element_motion_mm,
    }


def _group_intervals(intervals: Sequence[ElementaryInterval]) \
        -> dict[tuple[Any, ...], list[ElementaryInterval]]:
    groups: dict[tuple[Any, ...], list[ElementaryInterval]] = {}
    for interval in intervals:
        groups.setdefault(interval.case_key(), []).append(interval)
    return groups


def _route_rows(routes: Mapping[str, Any], turns: int,
                issues: list[Issue]) -> dict[tuple[int, int], Mapping[str, Any]]:
    rows = routes.get("routes")
    if not isinstance(rows, list):
        issues.append(Issue(
            "FAIL", "route_rows_missing",
            "crossing route report has no routes list"))
        return {}
    result: dict[tuple[int, int], Mapping[str, Any]] = {}
    for row in rows:
        try:
            key = (int(row["turn_index"]), int(row["half_turn_index"]))
        except (KeyError, TypeError, ValueError):
            issues.append(Issue(
                "FAIL", "route_key_invalid",
                "crossing route row has an invalid turn/half-turn key"))
            continue
        if key in result:
            issues.append(Issue(
                "FAIL", "route_key_duplicate",
                f"crossing route key is duplicated: {key}"))
        result[key] = row
        expected_phase = (2 * key[0] + key[1]) * math.pi
        if not _close(row.get("logical_phase_rad"), expected_phase, 1e-9):
            issues.append(Issue(
                "FAIL", "route_phase_mismatch",
                f"crossing route {key} has the wrong logical phase"))
        if (row.get("status") != "PASS"
                or row.get("validated_motion_signs") != [-1, 1]
                or row.get("progressive_support_validated") is not True):
            issues.append(Issue(
                "FAIL", "route_row_not_release_pass",
                f"crossing route {key} is not a two-direction progressive PASS"))
        postcheck = row.get("postcheck", {})
        for actual_name, required_name in (
                ("minimum_core_center_distance_mm",
                 "required_core_center_distance_mm"),
                ("minimum_copper_center_distance_mm",
                 "required_copper_center_distance_mm")):
            actual, required = postcheck.get(actual_name), postcheck.get(
                required_name)
            if not (_finite(actual) and _finite(required)
                    and float(actual) + 1e-9 >= float(required)):
                issues.append(Issue(
                    "FAIL", "route_postcheck_clearance",
                    f"crossing route {key} failed {actual_name}"))
        geometry = row.get("route", {})
        points, tags = geometry.get("points_local_mm"), geometry.get(
            "segment_tags")
        if (not isinstance(points, list) or len(points) < 2
                or not isinstance(tags, list) or len(tags) != len(points) - 1):
            issues.append(Issue(
                "FAIL", "route_polyline_invalid",
                f"crossing route {key} has invalid points/tags"))
    expected = {(turn, half) for turn in range(turns) for half in (0, 1)}
    if set(result) != expected:
        issues.append(Issue(
            "FAIL", "route_coverage_incomplete",
            "crossing route table has missing or extra turn/half-turn rows",
            {"expected": len(expected), "actual": len(result)}))
    return result


def _case_key(case: Mapping[str, Any]) -> tuple[Any, ...] | None:
    if case.get("kind") == "packing":
        if (isinstance(case.get("pass_index"), int)
                and isinstance(case.get("half_turn_index"), int)):
            return ("packing", case["pass_index"], case["half_turn_index"])
    elif case.get("kind") == "transition" and isinstance(
            case.get("transition_index"), int):
        return ("transition", case["transition_index"])
    return None


def _progressive_expected(half_turn_index: int) -> dict[str, Any]:
    turn = half_turn_index // 2
    completed = list(range(turn))
    completed_hash = hashlib.sha256(json.dumps(
        completed, separators=(",", ":")).encode()).hexdigest()
    return {
        "completed_turn_count": turn,
        "completed_turn_indices_sha256": completed_hash,
        "active_turn_index": turn,
        "already_laid_current_half_turns": half_turn_index % 2,
        "all_prior_completed_turns_included": True,
        "current_conductor_exclusion": "adjacent_segment_only",
        "neighbor_prefill_sides": [-1, 1],
    }


def _check_progressive(case: Mapping[str, Any],
                       half_turn_index: int,
                       wire_diameter_mm: float) -> str | None:
    state = case.get("progressive_copper")
    if not isinstance(state, dict):
        return "progressive_copper is missing"
    expected = _progressive_expected(half_turn_index)
    for key, value in expected.items():
        if state.get(key) != value:
            return f"progressive_copper.{key} is not {value!r}"
    exclusion = state.get("adjacent_self_exclusion_length_mm")
    if not (_finite(exclusion) and 0.0 <= float(exclusion)
            <= 2.0 * wire_diameter_mm + 1e-12):
        return (
            "adjacent self exclusion must be finite and no longer than two "
            "wire diameters")
    return None


def _universal_case_hash(case: Mapping[str, Any]) -> str:
    """Hash only the physical contract that a universal proof must cover."""

    fields = (
        "case_id", "kind", "pass_index", "half_turn_index",
        "transition_index", "start_t_s", "end_t_s", "homotopy_tag",
        "minimum_noncontact_clearance_mm", "contact_uncertainty_mm",
        "maximum_flyer_radius_mm", "maximum_stator_radius_mm",
        "progressive_copper", "progressive_copper_state",
    )
    payload = {name: case[name] for name in fields if name in case}
    return _canonical_hash(payload)


def _validate_proof_artifacts(
        contact: Mapping[str, Any],
        contact_path: Path,
        source_root: Path,
        issues: list[Issue]) -> dict[str, Mapping[str, str]]:
    artifacts = contact.get("proof_artifacts")
    if not isinstance(artifacts, dict) or not artifacts:
        issues.append(Issue(
            "FAIL", "contact_proof_artifacts_missing",
            "UNIVERSALLY_BOUNDED contact evidence needs proof_artifacts"))
        return {}
    valid: dict[str, Mapping[str, str]] = {}
    for proof_id, record in artifacts.items():
        if not isinstance(record, dict):
            issues.append(Issue(
                "FAIL", "contact_proof_artifact_invalid",
                f"proof artifact {proof_id!r} is not an object"))
            continue
        path_value, expected = record.get("path"), record.get("sha256")
        if not isinstance(path_value, str) or not _is_sha256(expected):
            issues.append(Issue(
                "FAIL", "contact_proof_artifact_invalid",
                f"proof artifact {proof_id!r} has no path/hash"))
            continue
        path = Path(path_value)
        if not path.is_absolute():
            path = contact_path.parent / path
        if not path.is_file():
            issues.append(Issue(
                "FAIL", "contact_proof_artifact_missing",
                f"proof artifact does not exist: {path}"))
            continue
        actual = _sha256(path)
        if actual != expected:
            issues.append(Issue(
                "FAIL", "contact_proof_artifact_stale",
                f"proof artifact hash mismatch: {path}",
                {"expected": expected, "actual": actual}))
            continue
        try:
            proof = _load_json(path)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            issues.append(Issue(
                "FAIL", "contact_proof_artifact_parse_error",
                f"proof artifact is not valid JSON: {path}: {exc}"))
            continue
        if (proof.get("schema") != UNIVERSAL_PROOF_SCHEMA
                or proof.get("status") != "PASS"):
            issues.append(Issue(
                "FAIL", "contact_proof_artifact_not_pass",
                f"proof artifact is not {UNIVERSAL_PROOF_SCHEMA} PASS: {path}"))
            continue
        before = len(issues)
        _check_self_hash(
            f"contact_proof_{proof_id}", proof, "report_sha256", issues)
        _check_source_hashes(
            f"contact_proof_{proof_id}", proof.get("source_hashes"),
            source_root, issues)
        case_hashes = proof.get("case_sha256")
        if not isinstance(case_hashes, dict) or not case_hashes or any(
                not isinstance(case_id, str) or not _is_sha256(case_hash)
                for case_id, case_hash in case_hashes.items()):
            issues.append(Issue(
                "FAIL", "contact_proof_case_hashes_invalid",
                f"proof artifact has no valid case_sha256 map: {path}"))
        if len(issues) == before:
            valid[str(proof_id)] = case_hashes
    return valid


def _case_motion_result(case: Mapping[str, Any],
                        group: Sequence[ElementaryInterval],
                        *, timeline: Timeline,
                        provenance: str,
                        capture_observations: Mapping[str, Mapping[str, Any]],
                        mm_per_rad_m0: float) -> tuple[str, dict[str, Any]]:
    clearance = case.get("minimum_noncontact_clearance_mm")
    uncertainty = case.get("contact_uncertainty_mm")
    flyer_radius = case.get("maximum_flyer_radius_mm")
    stator_radius = case.get("maximum_stator_radius_mm")
    numeric = (clearance, uncertainty, flyer_radius, stator_radius)
    if not all(_finite(value) for value in numeric):
        return "FAIL", {"reason": "case clearance/bounds are not finite"}
    clearance = float(clearance)
    uncertainty = float(uncertainty)
    flyer_radius = float(flyer_radius)
    stator_radius = float(stator_radius)
    if min(clearance, uncertainty, flyer_radius, stator_radius) < 0.0:
        return "FAIL", {"reason": "case clearance/bounds are negative"}

    maximum_bound = 0.0
    minimum_margin = math.inf
    limiting_interval = None
    if provenance == "UNIVERSALLY_BOUNDED":
        for interval in group:
            bound = _motion_bound(
                interval.pose0, interval.pose1,
                mm_per_rad_m0=mm_per_rad_m0,
                flyer_radius_mm=flyer_radius,
                stator_radius_mm=stator_radius,
                contact_uncertainty_mm=uncertainty)
            margin = clearance - bound["total_mm"]
            if margin < minimum_margin:
                minimum_margin = margin
                maximum_bound = bound["total_mm"]
                limiting_interval = interval.index
    else:
        observation_ids = case.get("observation_ids")
        if not isinstance(observation_ids, list) or len(observation_ids) < 2:
            return "FAIL", {"reason": "OBSERVED case needs >=2 observations"}
        try:
            observations = [capture_observations[str(identifier)]
                            for identifier in observation_ids]
        except KeyError as exc:
            return "FAIL", {"reason": f"capture observation missing: {exc}"}
        observations.sort(key=_event_time)
        observation_times = [_event_time(item) for item in observations]
        homotopy = case.get("homotopy_tag")
        if any(item.get("homotopy_tag") != homotopy
               for item in observations):
            return "FAIL", {"reason": "observed homotopy tag changed"}
        for item in observations:
            if not _finite(item.get("minimum_noncontact_clearance_mm")):
                return "FAIL", {"reason": "observation clearance is invalid"}
        if (observation_times[0] > group[0].t0 + TIME_EPS_S
                or observation_times[-1] < group[-1].t1 - TIME_EPS_S):
            return "FAIL", {"reason": "observations do not bracket case"}
        for interval in group:
            right = bisect_right(observation_times, interval.t0)
            left_index = max(0, right - 1)
            right_index = min(len(observations) - 1, left_index + 1)
            left, right_obs = observations[left_index], observations[right_index]
            if _event_time(right_obs) < interval.t1 - TIME_EPS_S:
                return "FAIL", {
                    "reason": "an interval is not bracketed by observations",
                    "interval_index": interval.index,
                }
            observed_clearance = min(
                float(left["minimum_noncontact_clearance_mm"]),
                float(right_obs["minimum_noncontact_clearance_mm"]),
                clearance)
            bound = _motion_bound(
                timeline.pose_at(_event_time(left)),
                timeline.pose_at(_event_time(right_obs)),
                mm_per_rad_m0=mm_per_rad_m0,
                flyer_radius_mm=flyer_radius,
                stator_radius_mm=stator_radius,
                contact_uncertainty_mm=uncertainty)
            margin = observed_clearance - bound["total_mm"]
            if margin < minimum_margin:
                minimum_margin = margin
                maximum_bound = bound["total_mm"]
                limiting_interval = interval.index

    status = "PASS" if minimum_margin >= -1e-9 else "FAIL"
    return status, {
        "reason": "ok" if status == "PASS"
        else "swept motion bound consumes noncontact clearance",
        "minimum_noncontact_clearance_mm": clearance,
        "maximum_swept_bound_mm": maximum_bound,
        "minimum_swept_margin_mm": minimum_margin,
        "limiting_elementary_interval": limiting_interval,
        "elementary_interval_count": len(group),
        "maximum_flyer_radius_mm": flyer_radius,
        "maximum_stator_radius_mm": stator_radius,
        "contact_uncertainty_mm": uncertainty,
    }


def _audit_contact(contact: Mapping[str, Any] | None,
                   contact_path: Path,
                   groups: Mapping[tuple[Any, ...], Sequence[ElementaryInterval]],
                   *, timeline: Timeline,
                   route_rows: Mapping[tuple[int, int], Mapping[str, Any]],
                   capture_events: Sequence[Mapping[str, Any]],
                   wire_diameter_mm: float,
                   required_stator_radius_mm: float,
                   scope: tuple[float, float],
                   valid_proofs: Mapping[str, Mapping[str, str]],
                   issues: list[Issue],
                   config: AuditConfig) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    if contact is None:
        issues.append(Issue(
            "NOT_PROVEN", "contact_provenance_missing",
            "capture has no hash-bound OBSERVED or UNIVERSALLY_BOUNDED "
            "wire-contact envelope; passive contact cannot be inferred from "
            "motor commands"))
        records = []
        for key, group in sorted(groups.items(), key=lambda item: str(item[0])):
            records.append({
                "case_key": list(key),
                "status": "NOT_PROVEN",
                "reason": "no contact evidence",
                "start_t_s": group[0].t0,
                "end_t_s": group[-1].t1,
                "elementary_interval_count": len(group),
            })
        return records, {
            "provenance": "NOT_PROVEN",
            "expected_cases": len(groups),
            "provided_cases": 0,
            "passed_cases": 0,
            "failed_cases": 0,
            "not_proven_cases": len(groups),
        }

    provenance = contact.get("provenance")
    if provenance not in ("OBSERVED", "UNIVERSALLY_BOUNDED"):
        issues.append(Issue(
            "FAIL", "contact_provenance_invalid",
            "contact provenance must be OBSERVED or UNIVERSALLY_BOUNDED"))
        provenance = "NOT_PROVEN"
    declared_window = contact.get("wire_live_window_s")
    if (not isinstance(declared_window, list) or len(declared_window) != 2
            or not _close(declared_window[0], scope[0], TIME_EPS_S)
            or not _close(declared_window[1], scope[1], TIME_EPS_S)):
        issues.append(Issue(
            "FAIL", "contact_wire_live_scope_mismatch",
            "contact evidence does not cover the exact captured wire-live window",
            {"capture_window_s": list(scope),
             "contact_window_s": declared_window}))

    raw_cases = contact.get("cases")
    if not isinstance(raw_cases, list):
        raw_cases = []
        issues.append(Issue(
            "FAIL", "contact_cases_missing",
            "contact evidence has no cases list"))
    cases: dict[tuple[Any, ...], Mapping[str, Any]] = {}
    for case in raw_cases:
        if not isinstance(case, dict):
            issues.append(Issue(
                "FAIL", "contact_case_invalid",
                "contact case is not an object"))
            continue
        key = _case_key(case)
        if key is None or key in cases:
            issues.append(Issue(
                "FAIL", "contact_case_key_invalid",
                f"contact case key is invalid or duplicated: {key!r}"))
            continue
        cases[key] = case
    if set(cases) != set(groups):
        issues.append(Issue(
            "FAIL", "contact_case_coverage_mismatch",
            "contact cases do not exactly cover every partition case",
            {"expected": len(groups), "provided": len(cases),
             "missing": [list(key) for key in sorted(
                 set(groups) - set(cases), key=str)[:25]],
             "extra": [list(key) for key in sorted(
                 set(cases) - set(groups), key=str)[:25]]}))

    observations: dict[str, Mapping[str, Any]] = {}
    for event in capture_events:
        if event.get("e") != "wire_contact_observation":
            continue
        identifier = event.get("observation_id")
        if not isinstance(identifier, str) or identifier in observations:
            issues.append(Issue(
                "FAIL", "capture_contact_observation_id_invalid",
                "capture contact observation IDs must be unique strings"))
            continue
        observations[identifier] = event

    records: list[dict[str, Any]] = []
    previous: tuple[tuple[Any, ...], Mapping[str, Any], float] | None = None
    for key, group in sorted(groups.items(), key=lambda item: item[1][0].t0):
        case = cases.get(key)
        base = {
            "case_key": list(key),
            "start_t_s": group[0].t0,
            "end_t_s": group[-1].t1,
            "elementary_interval_count": len(group),
        }
        if case is None:
            records.append({**base, "status": "NOT_PROVEN",
                            "reason": "contact case missing"})
            continue
        if case.get("provenance", provenance) != provenance:
            records.append({**base, "status": "FAIL",
                            "reason": "case provenance mismatch"})
            continue
        if not isinstance(case.get("homotopy_tag"), str) \
                or not case["homotopy_tag"]:
            records.append({**base, "status": "FAIL",
                            "reason": "homotopy_tag is missing"})
            continue
        if (not _close(case.get("start_t_s"), group[0].t0, TIME_EPS_S)
                or not _close(case.get("end_t_s"), group[-1].t1,
                              TIME_EPS_S)):
            records.append({**base, "status": "FAIL",
                            "reason": "case time coverage mismatch"})
            continue
        if (not _finite(case.get("maximum_flyer_radius_mm"))
                or float(case["maximum_flyer_radius_mm"])
                + 1e-12 < config.default_flyer_radius_mm
                or not _finite(case.get("maximum_stator_radius_mm"))
                or float(case["maximum_stator_radius_mm"])
                + 1e-12 < required_stator_radius_mm):
            records.append({**base, "status": "FAIL",
                            "reason": (
                                "declared rotation radii are smaller than "
                                "the independently required machine bounds")})
            continue

        if key[0] == "packing":
            half = int(key[2])
            progressive_error = _check_progressive(
                case, half, wire_diameter_mm)
            if progressive_error:
                records.append({**base, "status": "FAIL",
                                "reason": progressive_error})
                continue
            start_route = (half // 2, half % 2)
            end_phase = half + 1
            end_route = ((end_phase // 2, end_phase % 2)
                         if end_phase < 2 * (max(
                             (route[0] for route in route_rows), default=-1) + 1)
                         else (half // 2, 0))
            if start_route not in route_rows or end_route not in route_rows:
                records.append({**base, "status": "FAIL",
                                "reason": "crossing endpoint route missing"})
                continue
            base["crossing_route_keys"] = [list(start_route), list(end_route)]
        elif case.get("progressive_copper_state") != "full_capture_state":
            records.append({**base, "status": "FAIL",
                            "reason": "transition lacks full copper state"})
            continue
        elif key[0] == "transition":
            transition_state = case.get("progressive_copper")
            if (not isinstance(transition_state, dict)
                    or transition_state.get("all_prior_copper_included")
                    is not True
                    or transition_state.get("current_conductor_exclusion")
                    != "adjacent_segment_only"
                    or not _is_sha256(transition_state.get(
                        "deposited_geometry_sha256"))
                    or not _finite(transition_state.get(
                        "adjacent_self_exclusion_length_mm"))
                    or not 0.0 <= float(transition_state[
                        "adjacent_self_exclusion_length_mm"]) <= (
                            2.0 * wire_diameter_mm + 1e-12)):
                records.append({**base, "status": "FAIL",
                                "reason": (
                                    "transition progressive copper geometry "
                                    "is incomplete or excludes more than one "
                                    "short adjacent self segment")})
                continue

        if provenance == "UNIVERSALLY_BOUNDED":
            proof_id = case.get("proof_id")
            case_id = case.get("case_id")
            proof_cases = valid_proofs.get(str(proof_id))
            if (not isinstance(case_id, str) or proof_cases is None
                    or proof_cases.get(case_id) != _universal_case_hash(case)
                    or not isinstance(case.get("proof_method"), str)
                    or not case["proof_method"]):
                records.append({**base, "status": "FAIL",
                                "reason": (
                                    "universal proof/case hash binding missing")})
                continue

        # A homotopy change is accepted only at an already partitioned
        # boundary with one explicit, hash-bound transition proof shared by
        # both adjacent cases.  Otherwise topology is not interpolated.
        if previous is not None:
            prev_key, prev_case, boundary_t = previous
            if prev_case.get("homotopy_tag") != case.get("homotopy_tag"):
                left_proof = prev_case.get("homotopy_transition_proof_sha256")
                right_proof = case.get("homotopy_transition_proof_sha256")
                declared_t = case.get("homotopy_change_at_s")
                if (not _is_sha256(left_proof) or left_proof != right_proof
                        or not _close(declared_t, boundary_t, TIME_EPS_S)):
                    records.append({**base, "status": "FAIL",
                                    "reason": "unproved homotopy tag change"})
                    previous = (key, case, group[-1].t1)
                    continue

        status, detail = _case_motion_result(
            case, group, timeline=timeline, provenance=provenance,
            capture_observations=observations,
            mm_per_rad_m0=config.mm_per_rad_m0)
        records.append({**base, "status": status, **detail})
        previous = (key, case, group[-1].t1)

    passed = sum(record["status"] == "PASS" for record in records)
    failed = sum(record["status"] == "FAIL" for record in records)
    not_proven = sum(record["status"] == "NOT_PROVEN" for record in records)
    if failed:
        issues.append(Issue(
            "FAIL", "continuous_contact_case_failure",
            f"{failed} continuous contact case(s) failed"))
    if not_proven:
        issues.append(Issue(
            "NOT_PROVEN", "continuous_contact_case_missing",
            f"{not_proven} continuous contact case(s) are not proven"))
    return records, {
        "provenance": provenance,
        "expected_cases": len(groups),
        "provided_cases": len(cases),
        "passed_cases": passed,
        "failed_cases": failed,
        "not_proven_cases": not_proven,
        "observed_capture_sample_count": len(observations),
    }


def analyze(capture_path: Path = CAPTURE_PATH,
            plan_path: Path = PLAN_PATH,
            packing_path: Path = PACKING_PATH,
            routes_path: Path = ROUTES_PATH,
            contact_path: Path | None = CONTACT_PATH,
            *, source_root: Path = ROOT,
            config: AuditConfig = AuditConfig()) -> dict[str, Any]:
    """Build the complete capture-bound continuous-wire audit report."""

    capture_path = Path(capture_path)
    plan_path = Path(plan_path)
    packing_path = Path(packing_path)
    routes_path = Path(routes_path)
    contact_path = Path(contact_path) if contact_path is not None else None
    source_root = Path(source_root)
    issues: list[Issue] = []

    required_paths = {
        "capture": capture_path,
        "plan": plan_path,
        "packing": packing_path,
        "routes": routes_path,
    }
    missing = [name for name, path in required_paths.items()
               if not path.is_file()]
    if missing:
        for name in missing:
            issues.append(Issue(
                "FAIL", f"{name}_artifact_missing",
                f"required {name} artifact is missing: {required_paths[name]}"))
        return _finish_report(
            issues, required_paths, None, None, None, [], {}, [], {})

    try:
        events = load_events(capture_path)
        plan = _load_json(plan_path)
        packing = _load_json(packing_path)
        routes = _load_json(routes_path)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        issues.append(Issue(
            "FAIL", "input_parse_error", str(exc)))
        return _finish_report(
            issues, required_paths, None, None, None, [], {}, [], {})

    contact: dict[str, Any] | None = None
    if contact_path is not None and contact_path.is_file():
        try:
            contact = _load_json(contact_path)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            issues.append(Issue(
                "FAIL", "contact_parse_error", str(exc)))

    if not events:
        issues.append(Issue("FAIL", "capture_empty", "capture is empty"))
        return _finish_report(
            issues, required_paths, plan, packing, routes, [], {}, [], {})
    metas = [event for event in events if event.get("e") == "meta"]
    if len(metas) != 1:
        issues.append(Issue(
            "FAIL", "capture_meta_count",
            f"capture must contain exactly one meta event, found {len(metas)}"))
        meta: Mapping[str, Any] = {}
    else:
        meta = metas[0]

    # Commands must remain in capture order; metadata at t=0 may be inserted
    # separately and is intentionally ignored by this monotonicity check.
    command_times = [_event_time(event) for event in events
                     if event.get("e") == "cmd"]
    if any(a > b + config.time_tolerance_s
           for a, b in zip(command_times, command_times[1:])):
        issues.append(Issue(
            "FAIL", "capture_command_time_regression",
            "motor command timestamps regress"))
    if sum(event.get("e") == "cycle_complete" for event in events) != 1:
        issues.append(Issue(
            "FAIL", "capture_cycle_incomplete",
            "capture must have exactly one cycle_complete marker"))
    if meta.get("controller_mode") != "contract":
        issues.append(Issue(
            "FAIL", "capture_controller_not_contract",
            "continuous audit requires the project contract controller"))
    if int(meta.get("capture_schema", -1)) < 3:
        issues.append(Issue(
            "FAIL", "capture_schema_unsupported",
            "capture schema 3 or newer is required"))
    _check_shaft_wrap_phase_contract(events, meta, issues)

    _check_self_hash("packing", packing, "report_sha256", issues)
    _check_self_hash("plan", plan, "proof_sha256", issues)
    _check_self_hash("routes", routes, "report_sha256", issues)
    _check_source_hashes(
        "packing", packing.get("source_hashes"), source_root, issues)
    _check_source_hashes(
        "plan", plan.get("source_hashes"), source_root, issues)
    _check_source_hashes(
        "routes", routes.get("source_hashes"), source_root, issues)

    if packing.get("schema") != "slot-packing/v2" \
            or packing.get("status") != "PASS":
        issues.append(Issue(
            "FAIL", "packing_not_release_pass",
            "packing report is not slot-packing/v2 PASS"))
    if plan.get("schema") != "slot-winding-plan/v1" \
            or plan.get("selected_case", {}).get("status") != "PASS":
        issues.append(Issue(
            "FAIL", "plan_not_release_pass",
            "winding plan is not slot-winding-plan/v1 PASS"))
    if routes.get("schema") != "slot-wire-routes/v1" \
            or routes.get("status") != "PASS":
        issues.append(Issue(
            "FAIL", "routes_not_release_pass",
            "crossing route table is not slot-wire-routes/v1 PASS"))

    packing_report_hash = packing.get("report_sha256")
    if plan.get("packing_report", {}).get(
            "report_sha256") != packing_report_hash:
        issues.append(Issue(
            "FAIL", "plan_packing_binding_mismatch",
            "winding plan is not bound to the loaded packing report"))
    route_input = routes.get("input_contract", {})
    if route_input.get("packing_report_sha256") != packing_report_hash:
        issues.append(Issue(
            "FAIL", "routes_packing_binding_mismatch",
            "crossing route table is not bound to the loaded packing report"))
    if route_input.get("packing_file_sha256") != _sha256(packing_path):
        issues.append(Issue(
            "FAIL", "routes_packing_file_binding_mismatch",
            "crossing route table packing file SHA-256 is stale"))

    capture_plan = meta.get("winding_plan", {})
    if capture_plan.get("sha256") != _sha256(plan_path):
        issues.append(Issue(
            "FAIL", "capture_plan_file_binding_mismatch",
            "capture is not bound to the loaded winding-plan file"))
    if capture_plan.get("proof_sha256") != plan.get("proof_sha256"):
        issues.append(Issue(
            "FAIL", "capture_plan_proof_binding_mismatch",
            "capture winding-plan proof SHA-256 is stale"))
    adapter_hash = meta.get("controller_adapter_sha256")
    adapter_path = source_root / "sim" / "controller_adapter.py"
    if not adapter_path.is_file() or adapter_hash != _sha256(adapter_path):
        issues.append(Issue(
            "FAIL", "capture_controller_source_binding_mismatch",
            "capture controller-adapter source SHA-256 is stale"))

    job = plan.get("job", {})
    try:
        turns = int(job["turns_per_tooth"])
        teeth = int(job["slots"])
        wire_diameter = float(job["wire_finished_d_mm"])
        stator_radius = float(job["od_mm"]) / 2.0 + wire_diameter / 2.0
    except (KeyError, TypeError, ValueError):
        issues.append(Issue(
            "FAIL", "plan_job_invalid",
            "plan job lacks slots/turns/wire/OD"))
        turns, teeth, wire_diameter, stator_radius = 0, 0, math.nan, 25.0
    if meta.get("turns") != turns or meta.get("teeth_count") != teeth:
        issues.append(Issue(
            "FAIL", "capture_job_count_mismatch",
            "capture teeth/turn count does not match winding plan"))
    capture_job = meta.get("job", {})
    for name in ("slots", "od_mm", "stack_mm", "wire_finished_d_mm"):
        plan_name = "wire_finished_d_mm" if name == "wire_finished_d_mm" else name
        expected = job.get(plan_name)
        actual = capture_job.get(name) if isinstance(capture_job, dict) else None
        if isinstance(expected, (int, float)):
            matches = _close(actual, expected, 1e-9)
        else:
            matches = actual == expected
        if not matches:
            issues.append(Issue(
                "FAIL", "capture_job_geometry_mismatch",
                f"capture job {name} does not match winding plan",
                {"capture": actual, "plan": expected}))

    route_rows = _route_rows(routes, turns, issues) if turns > 0 else {}
    try:
        timeline = Timeline(events)
    except Exception as exc:  # a malformed capture must become a report
        issues.append(Issue(
            "FAIL", "capture_timeline_invalid", str(exc)))
        return _finish_report(
            issues, required_paths, plan, packing, routes, [], {}, [], {})

    windows = _derive_pass_windows(
        events, timeline, turns, issues, config) if turns > 0 else []
    if len(windows) != teeth:
        issues.append(Issue(
            "FAIL", "packing_pass_coverage",
            f"capture has {len(windows)} valid packing passes; expected {teeth}"))
    scope = _capture_scope(events, issues)
    if scope is None:
        return _finish_report(
            issues, required_paths, plan, packing, routes, [], {}, [], {})

    contact_radius = config.default_flyer_radius_mm
    if contact is not None:
        values = [case.get("maximum_flyer_radius_mm")
                  for case in contact.get("cases", [])
                  if isinstance(case, dict)
                  and _finite(case.get("maximum_flyer_radius_mm"))]
        if values:
            contact_radius = max(
                contact_radius, *(float(value) for value in values))
    try:
        intervals, partition = _partition(
            events, timeline, windows, scope,
            flyer_radius_mm=contact_radius,
            stator_radius_mm=stator_radius,
            config=config)
    except Exception as exc:
        issues.append(Issue(
            "FAIL", "capture_partition_failed", str(exc)))
        return _finish_report(
            issues, required_paths, plan, packing, routes, [], {}, [], {})

    groups = _group_intervals(intervals)
    expected_packing_keys = {
        ("packing", pass_index, half)
        for pass_index in range(teeth)
        for half in range(2 * turns)
    }
    actual_packing_keys = {key for key in groups if key[0] == "packing"}
    if actual_packing_keys != expected_packing_keys:
        issues.append(Issue(
            "FAIL", "continuous_packing_coverage_mismatch",
            "partition does not contain every pass/half-turn interval exactly",
            {"expected": len(expected_packing_keys),
             "actual": len(actual_packing_keys)}))

    valid_proofs: dict[str, Mapping[str, str]] = {}
    if contact is not None:
        if contact.get("schema") != CONTACT_SCHEMA \
                or contact.get("status") != "PASS":
            issues.append(Issue(
                "FAIL", "contact_report_not_pass",
                f"contact evidence is not {CONTACT_SCHEMA} PASS"))
        _check_self_hash("contact", contact, "report_sha256", issues)
        _check_source_hashes(
            "contact", contact.get("source_hashes"), source_root, issues)
        bindings = {
            "capture_file_sha256": _sha256(capture_path),
            "plan_file_sha256": _sha256(plan_path),
            "plan_proof_sha256": plan.get("proof_sha256"),
            "packing_report_sha256": packing.get("report_sha256"),
            "route_file_sha256": _sha256(routes_path),
            "route_report_sha256": routes.get("report_sha256"),
        }
        for name, expected in bindings.items():
            if contact.get(name) != expected:
                issues.append(Issue(
                    "FAIL", "contact_input_binding_mismatch",
                    f"contact evidence {name} is stale",
                    {"expected": expected, "actual": contact.get(name)}))
        if contact.get("provenance") == "UNIVERSALLY_BOUNDED":
            valid_proofs = _validate_proof_artifacts(
                contact, contact_path, source_root, issues)

    interval_records, contact_summary = _audit_contact(
        contact, contact_path or CONTACT_PATH, groups,
        timeline=timeline, route_rows=route_rows,
        capture_events=events, wire_diameter_mm=wire_diameter,
        required_stator_radius_mm=stator_radius,
        scope=scope, valid_proofs=valid_proofs,
        issues=issues, config=config)

    return _finish_report(
        issues, required_paths, plan, packing, routes,
        windows, partition, interval_records, contact_summary,
        contact_path=contact_path,
        contact=contact,
        extra_coverage={
            "expected_packing_passes": teeth,
            "captured_packing_passes": len(windows),
            "turns_per_tooth": turns,
            "expected_packing_halfturn_cases": teeth * 2 * turns,
            "partitioned_packing_halfturn_cases": len(actual_packing_keys),
            "transition_cases": sum(key[0] == "transition" for key in groups),
            "command_event_count": sum(
                event.get("e") == "cmd" and event.get("m") in (0, 1, 2)
                for event in events),
            "command_event_count_by_axis": {
                f"M{axis}": sum(
                    event.get("e") == "cmd" and event.get("m") == axis
                    for event in events)
                for axis in (0, 1, 2)
            },
            "packing_waypoint_event_count": sum(
                event.get("e") == "packing_waypoint" for event in events),
            "shaft_wrap_phase_event_count": sum(
                event.get("e") == "shaft_wrap_phase" for event in events),
        })


def _finish_report(issues: Sequence[Issue],
                   required_paths: Mapping[str, Path],
                   plan: Mapping[str, Any] | None,
                   packing: Mapping[str, Any] | None,
                   routes: Mapping[str, Any] | None,
                   windows: Sequence[PassWindow],
                   partition: Mapping[str, Any],
                   interval_records: Sequence[Mapping[str, Any]],
                   contact_summary: Mapping[str, Any],
                   *, contact_path: Path | None = None,
                   contact: Mapping[str, Any] | None = None,
                   extra_coverage: Mapping[str, Any] | None = None) -> dict[str, Any]:
    severities = {issue.severity for issue in issues}
    if "FAIL" in severities:
        status = "FAIL"
    elif "NOT_PROVEN" in severities:
        status = "NOT_PROVEN"
    elif interval_records and all(
            record.get("status") == "PASS" for record in interval_records):
        status = "PASS"
    else:
        status = "NOT_PROVEN"
    input_hashes = {
        name: (_sha256(path) if path.is_file() else None)
        for name, path in required_paths.items()
    }
    if contact_path is not None:
        input_hashes["contact"] = (
            _sha256(contact_path) if contact_path.is_file() else None)
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": status,
        "scope": {
            "wire_live_definition": (
                "first captured wind_wire call through final "
                "wind_wire_done, including setup, all packed half-turns, "
                "tooth indexing, shaft wraps, and inter-pass transitions"),
            "crossing_route_role": (
                "static endpoint evidence only; never substituted for "
                "continuous passive contact"),
            "no_invented_profile": True,
            "progressive_current_conductor_rule": (
                "all already laid copper included; only one adjacent self "
                "segment no longer than two wire diameters may be excluded"),
        },
        "input_files": {
            name: str(path) for name, path in required_paths.items()
        },
        "input_file_sha256": input_hashes,
        "bound_artifact_ids": {
            "plan_proof_sha256": plan.get("proof_sha256") if plan else None,
            "packing_report_sha256": (
                packing.get("report_sha256") if packing else None),
            "route_report_sha256": (
                routes.get("report_sha256") if routes else None),
            "contact_report_sha256": (
                contact.get("report_sha256") if contact else None),
        },
        "motion_bound": {
            "formula": (
                "abs(dM0)*mm_per_rad + "
                "2*R_flyer*sin(min(pi,abs(dM2))/2) + "
                "2*R_stator*sin(min(pi,abs(dM1))/2) + "
                "contact_uncertainty"),
            "m0_mm_per_rad": DEFAULT_MM_PER_RAD_M0,
        },
        "partition": dict(partition),
        "coverage": dict(extra_coverage or {}),
        "contact_evidence": dict(contact_summary),
        "packing_passes": [
            {
                "pass_index": window.pass_index,
                "start_t_s": window.start_t,
                "end_t_s": window.end_t,
                "origin_event_t_s": window.origin_event_t,
                "captured_pass_end_t_s": window.captured_pass_end_t,
                "m2_direction": window.direction,
                "start_phase_rad": window.start_phase_rad,
                "phase_origin_rad": window.phase_origin_rad,
                "actual_travel_rad": window.actual_travel_rad,
            }
            for window in windows
        ],
        "interval_cases": list(interval_records),
        "issues": [issue.as_dict() for issue in issues],
        "limitations": [
            "Axis motion comes from the captured constant-velocity simulation semantics; hardware encoder/runout uncertainty belongs in contact_uncertainty_mm.",
            "A PASS requires an external observed contact stream or universal contact proof. Commands alone intentionally produce NOT_PROVEN.",
            "The audit does not invent sag, compliance, friction, or a safe-mouth crossover profile.",
        ],
        "source_hashes": {
            "sim/continuous_wire_audit.py": _sha256(Path(__file__)),
            "sim/traj.py": _sha256(HERE / "traj.py"),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def write_report(report: Mapping[str, Any],
                 path: Path = OUTPUT_PATH) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", type=Path, default=CAPTURE_PATH)
    parser.add_argument("--plan", type=Path, default=PLAN_PATH)
    parser.add_argument("--packing", type=Path, default=PACKING_PATH)
    parser.add_argument("--routes", type=Path, default=ROUTES_PATH)
    parser.add_argument("--contact", type=Path, default=CONTACT_PATH)
    parser.add_argument("--output", type=Path, default=OUTPUT_PATH)
    parser.add_argument("--max-element-motion-mm", type=float,
                        default=DEFAULT_MAX_ELEMENT_MOTION_MM)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    report = analyze(
        args.capture, args.plan, args.packing, args.routes, args.contact,
        config=AuditConfig(
            max_element_motion_mm=args.max_element_motion_mm))
    if not args.check:
        write_report(report, args.output)
        print(f"wrote {args.output}")
    coverage = report.get("coverage", {})
    print(
        f"continuous wire {report['status']}: "
        f"{coverage.get('partitioned_packing_halfturn_cases', 0)}/"
        f"{coverage.get('expected_packing_halfturn_cases', 0)} packed "
        f"half-turn cases; provenance="
        f"{report.get('contact_evidence', {}).get('provenance', 'NOT_PROVEN')}")
    if report["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
