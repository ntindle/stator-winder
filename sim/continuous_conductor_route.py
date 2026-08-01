"""Generate one ordered, phase-aware conductor route from the raw capture.

The watchable animation used to draw completed turns and the live flyer span
from two independent pieces of JavaScript.  Between ``wind_wire`` calls the
live span was hidden, and the deposited turns did not contain the conductor
which must advance between turns, cross the end faces, move to the next tooth,
or remain around the shaft between phases.

This module is the single source for that presentation geometry.  It consumes
the unmodified upstream capture, its exact reconstructed Timeline, the checked
slot plan, and the CAD wire manifest.  The output is one ordered polyline graph
in the spindle-local machine frame.  Every item starts at the preceding item's
tail, and every item covers a contiguous interval of the complete virtual
timeline.  The player and the structural validator both consume the checked-in
JSON artifact produced here.

Only the raw M0 radial motion and raw M1/M2 clocks are controller authority.
The slot tangential offsets are diagnostic, and the current end-cap,
inter-turn, tooth-transition, and shaft-wrap paths have not yet been cleared
against the production caps and retained flyer.  Those edges are deliberately
``UNPROVEN_FAIL_CLOSED`` / ``dashed_red`` and force the report to FAIL.  A
structurally connected visualization is not a geometric release claim.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from traj import Timeline, load_events, winding_windows


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
PLAN = ROOT / "out" / "reports" / "slot_winding_plan.json"
# The continuous route consumes the integrated candidate's explicit static
# supply, shaft-bore, flyer-guide, and terminal-locus handoff contract.  The
# legacy assembly manifest has no continuous wire sections and must not be a
# silent default for this authority artifact.
MANIFEST = (
    ROOT / "out" / "review" / "integrated_adapter" / "links" / "manifest.json"
)
OUTPUT_JSON = ROOT / "out" / "reports" / "continuous_conductor_route.json"
OUTPUT_MD = ROOT / "out" / "reports" / "continuous_conductor_route.md"

SCHEMA = "continuous-conductor-route/v1"
EXPECTED_CAPTURE_SCHEMA = 4
EXPECTED_PASSES = 24
EXPECTED_TURNS_PER_PASS = 50
EXPECTED_HALF_TURNS_PER_PASS = 100
MAX_POINT_JUMP_MM = 1.0
POINT_TOL_MM = 2.0e-6
TIME_TOL_S = 2.0e-6
ROUND_DIGITS = 9

DIAGNOSTIC = "DIAGNOSTIC_ONLY"
UNPROVEN = "UNPROVEN_FAIL_CLOSED"
SOLID_COPPER = "solid_copper"
DASHED_RED = "dashed_red"

TRANSITION_KINDS = {
    "cap_transition",
    "inter_turn_advance",
    "tooth_transition",
    "to_shaft_wrap",
    "shaft_wrap",
    "from_shaft_wrap",
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    body = deepcopy(dict(payload))
    body.pop("report_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load_object(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be finite")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be finite") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _round_point(point: Sequence[float]) -> list[float]:
    if len(point) != 3:
        raise ValueError("route point must have exactly three coordinates")
    return [round(_finite(value, "route coordinate"), ROUND_DIGITS)
            for value in point]


def _distance(left: Sequence[float], right: Sequence[float]) -> float:
    return math.sqrt(sum(
        (float(a) - float(b)) ** 2 for a, b in zip(left, right)
    ))


def _lerp(left: Sequence[float], right: Sequence[float], fraction: float) \
        -> list[float]:
    return _round_point([
        float(a) + (float(b) - float(a)) * float(fraction)
        for a, b in zip(left, right)
    ])


def _subdivide_polyline(points: Sequence[Sequence[float]],
                        maximum_jump_mm: float = MAX_POINT_JUMP_MM) \
        -> list[list[float]]:
    """Return the same polyline with every chord bounded by ``maximum``."""

    if not points:
        raise ValueError("route piece has no points")
    result = [_round_point(points[0])]
    for raw_end in points[1:]:
        start = result[-1]
        end = _round_point(raw_end)
        length = _distance(start, end)
        count = max(1, int(math.ceil(length / maximum_jump_mm)))
        for index in range(1, count + 1):
            candidate = _lerp(start, end, index / count)
            if candidate != result[-1]:
                result.append(candidate)
    return result


def _line(left: Sequence[float], right: Sequence[float]) \
        -> list[list[float]]:
    return _subdivide_polyline([left, right])


def _wire_handoff_contract(manifest: Mapping[str, Any]) -> dict[str, Any]:
    """Require the fixed supply and flyer bore to meet on the M2 axis.

    The ordered items describe the workpiece-side conductor tail.  Their
    player presentation is continuous only when the separately rendered
    supply wire reaches the exact first point of the flyer-owned guide bore.
    Bind that seam here so generation rejects the obsolete rear-mouth endpoint.
    """

    tolerance = 2.0e-6
    wire = manifest.get("wire")
    if not isinstance(wire, Mapping):
        raise RuntimeError("CAD manifest has no continuous wire contract")
    static = wire.get("static")
    flyer = wire.get("flyer")
    guide = wire.get("active_terminal_guide")
    if not all(isinstance(value, Mapping) for value in (static, flyer, guide)):
        raise RuntimeError("CAD manifest continuous wire sections are missing")
    static_points = static.get("points")
    flyer_points = flyer.get("points")
    guide_points = guide.get("bore_centerline_local_mm")
    if not all(
        isinstance(points, list) and len(points) >= 2
        for points in (static_points, flyer_points, guide_points)
    ):
        raise RuntimeError("CAD manifest continuous wire polylines are missing")
    landmarks = static.get("landmarks")
    if not isinstance(landmarks, Mapping) or "guide_root" not in landmarks:
        raise RuntimeError("CAD manifest static wire has no guide-root landmark")

    seam = _round_point(static_points[-1])
    gaps = {
        "static_endpoint_to_guide_root_mm": _distance(
            seam, landmarks["guide_root"]
        ),
        "static_to_flyer_bore_start_mm": _distance(
            seam, flyer_points[0]
        ),
        "flyer_to_guide_bore_start_mm": _distance(
            flyer_points[0], guide_points[0]
        ),
        "flyer_to_guide_bore_end_mm": _distance(
            flyer_points[-1], guide_points[-1]
        ),
        "flyer_to_dynamic_origin_mm": _distance(
            flyer_points[-1], guide.get("unproved_transition_origin_local_mm")
        ),
    }
    maximum_gap = max(gaps.values())
    if maximum_gap > tolerance:
        raise RuntimeError(f"CAD manifest continuous wire has a gap: {gaps}")
    if abs(seam[0]) > tolerance or abs(seam[1]) > tolerance:
        raise RuntimeError("CAD wire handoff is not invariant on the M2 axis")
    if len(flyer_points) != len(guide_points) or any(
        _distance(left, right) > tolerance
        for left, right in zip(flyer_points, guide_points)
    ):
        raise RuntimeError("CAD flyer wire differs from the PEEK bore")
    return {
        "status": "PASS",
        "static_to_flyer_seam_local_mm": seam,
        "flyer_to_dynamic_seam_local_mm": _round_point(flyer_points[-1]),
        "maximum_gap_mm": round(maximum_gap, ROUND_DIGITS),
        "tolerance_mm": tolerance,
        "gap_measurements_mm": {
            name: round(value, ROUND_DIGITS) for name, value in gaps.items()
        },
        "static_owner_continues_through_shaft_to_guide_root": True,
        "static_to_flyer_handoff_is_M2_axis_invariant": True,
        "unsupported_flexible_intervals_authorized": False,
    }


def _rotation_intervals(track: Any, start: float, end: float, count: int,
                        direction: int) -> list[tuple[float, float]]:
    """Invert the monotone raw M2 track at directed pi-spaced crossings."""

    if direction not in (-1, 1):
        raise ValueError("raw winding direction must be -1 or +1")
    start_position = float(track.pos_at(start))
    boundaries = [float(start)]
    for crossing_index in range(1, count + 1):
        target = start_position + direction * crossing_index * math.pi
        found = None
        knots = [(float(start), float(track.pos_at(start)))]
        knots.extend((float(t), float(p)) for t, p in track.knots
                     if start < float(t) < end)
        knots.append((float(end), float(track.pos_at(end))))
        knots.sort()
        for (t0, p0), (t1, p1) in zip(knots, knots[1:]):
            if direction * (p1 - p0) < -1.0e-9:
                raise RuntimeError("raw M2 reversed inside a winding pass")
            if (target - p0) * (target - p1) > 1.0e-10:
                continue
            if abs(p1 - p0) <= 1.0e-12:
                continue
            fraction = (target - p0) / (p1 - p0)
            if -1.0e-10 <= fraction <= 1.0 + 1.0e-10:
                found = t0 + (t1 - t0) * fraction
                break
        if found is None:
            break
        boundaries.append(float(found))
    if len(boundaries) != count + 1:
        raise RuntimeError(
            f"raw pass has {len(boundaries) - 1} of {count} half turns"
        )
    return list(zip(boundaries, boundaries[1:]))


def _raw_shaft_wraps(events: Sequence[Mapping[str, Any]],
                     timeline: Timeline) -> list[dict[str, Any]]:
    """Infer the two physical shaft intervals from raw M1 commands."""

    starts = [index for index, event in enumerate(events)
              if event.get("e") == "wind_wire_around_shaft"]
    if len(starts) != 2:
        raise RuntimeError("canonical raw capture must contain two shaft wraps")
    velocity = _finite(timeline.meta["velocities"][1], "raw M1 velocity")
    result = []
    for number, start_index in enumerate(starts, start=1):
        done_index = next((
            index for index in range(start_index + 1, len(events))
            if events[index].get("e") == "wind_wire_around_shaft_done"
        ), None)
        if done_index is None:
            raise RuntimeError(f"raw shaft wrap {number} has no done marker")
        commands = [
            (index, events[index])
            for index in range(start_index + 1, done_index)
            if events[index].get("e") == "cmd"
            and events[index].get("m") == 1
        ]
        if len(commands) != 1:
            raise RuntimeError(
                f"raw shaft wrap {number} has {len(commands)} M1 commands"
            )
        command_index, command = commands[0]
        start_t = _finite(command["t"], "raw shaft command time")
        start_m1 = float(timeline.axes[1].pos_at(start_t))
        end_m1 = _finite(
            command.get("model_target", command.get("a")),
            "raw shaft M1 target",
        )
        delta = end_m1 - start_m1
        end_t = start_t + abs(delta) / velocity
        marker_done = _finite(events[done_index]["t"], "raw shaft done marker")
        if end_t > marker_done + 1.0e-9:
            raise RuntimeError("raw M1 shaft target arrives after its marker")
        result.append({
            "number": number,
            "start": start_t,
            "end": end_t,
            "start_m1_rad": start_m1,
            "end_m1_rad": end_m1,
            "delta_m1_rad": delta,
            "direction": 1 if delta >= 0.0 else -1,
            "turns": abs(delta) / (2.0 * math.pi),
            "source_command_index": command_index,
            "source_command": str(command.get("command", "")).strip(),
            "marker_done": marker_done,
        })
    return result


def _active_placement(raw_plan: Mapping[str, Any], turn_index: int) \
        -> dict[str, float]:
    placement = raw_plan["placements"][turn_index]
    frame = raw_plan["coordinate_frame"]
    angle = math.radians(_finite(
        frame["active_tooth_center_angle_deg"], "active tooth frame angle",
    ))
    left = placement["left_slot_half_turn_center_mm"]
    left_r, left_t = map(float, left)
    active_t = -left_r * math.sin(angle) + left_t * math.cos(angle)
    if active_t >= 0.0:
        raise RuntimeError("slot-plan left active side is not negative")
    return {
        "left_t": active_t,
        "right_t": -active_t,
    }


def _coil_point(*, tooth: int, slots: int, standoff: float,
                radial: float, tangential: float, axial: float) \
        -> list[float]:
    angle = tooth * 2.0 * math.pi / slots
    radial_axis = (-math.sin(angle), 0.0, -math.cos(angle))
    tangent_axis = (-math.cos(angle), 0.0, math.sin(angle))
    return _round_point((
        radial_axis[0] * radial + tangent_axis[0] * tangential,
        axial,
        standoff + radial_axis[2] * radial + tangent_axis[2] * tangential,
    ))


def _active_radial(meta: Mapping[str, Any], manifest: Mapping[str, Any],
                   timeline: Timeline, time_s: float) -> float:
    contact_z = _finite(meta["job"]["wire_contact_z_mm"], "contact z")
    standoff = _finite(manifest["m0_home_standoff"], "M0 standoff")
    mm_per_rad = _finite(manifest["mm_per_rad_m0"], "M0 mm/rad")
    axis_z = standoff + float(timeline.axes[0].pos_at(time_s)) * mm_per_rad
    return axis_z - contact_z


def _piece(kind: str, points: Sequence[Sequence[float]],
           authorization: str, visual_style: str) -> dict[str, Any]:
    return {
        "kind": kind,
        "points": _subdivide_polyline(points),
        "authorization": authorization,
        "visual_style": visual_style,
    }


def _coil_half_pieces(*, half_index: int, tooth: int, slots: int,
                      standoff: float, stack_half: float,
                      radial_start: float, radial_end: float,
                      start_tangential: float, opposite_tangential: float,
                      next_start: Sequence[float] | None) \
        -> list[dict[str, Any]]:
    """Return one raw half-turn plus explicit cap/advancement geometry."""

    first_half = half_index % 2 == 0
    if first_half:
        axial_start, axial_end = stack_half, -stack_half
        leg_start_t, leg_end_t = start_tangential, start_tangential
        cap_start_t, cap_end_t = start_tangential, opposite_tangential
        cap_sign = -1.0
    else:
        axial_start, axial_end = -stack_half, stack_half
        leg_start_t, leg_end_t = opposite_tangential, opposite_tangential
        cap_start_t, cap_end_t = opposite_tangential, start_tangential
        cap_sign = 1.0

    leg = []
    leg_count = max(2, int(math.ceil(2.0 * stack_half / MAX_POINT_JUMP_MM)))
    for index in range(leg_count + 1):
        fraction = index / leg_count
        leg.append(_coil_point(
            tooth=tooth,
            slots=slots,
            standoff=standoff,
            radial=radial_start + (radial_end - radial_start) * fraction,
            tangential=leg_start_t + (leg_end_t - leg_start_t) * fraction,
            axial=axial_start + (axial_end - axial_start) * fraction,
        ))

    radius = abs(start_tangential - opposite_tangential) / 2.0
    center_t = (start_tangential + opposite_tangential) / 2.0
    # The diagnostic cap connector bows outward from the stack face.  It is
    # intentionally red/dashed because the production cap channels are not
    # yet integrated into this raw-cycle artifact.
    cap_count = max(4, int(math.ceil(math.pi * radius / MAX_POINT_JUMP_MM)))
    cap = []
    for index in range(cap_count + 1):
        fraction = index / cap_count
        theta = math.pi * fraction
        tangential = center_t + (cap_start_t - center_t) * math.cos(theta)
        axial = axial_end + cap_sign * radius * math.sin(theta)
        cap.append(_coil_point(
            tooth=tooth,
            slots=slots,
            standoff=standoff,
            radial=radial_end,
            tangential=tangential,
            axial=axial,
        ))
    cap[-1] = _coil_point(
        tooth=tooth, slots=slots, standoff=standoff,
        radial=radial_end, tangential=cap_end_t, axial=axial_end,
    )

    result = [
        _piece("slot_leg", leg, DIAGNOSTIC, SOLID_COPPER),
        _piece("cap_transition", cap, UNPROVEN, DASHED_RED),
    ]
    if not first_half and next_start is not None:
        result.append(_piece(
            "inter_turn_advance", [cap[-1], next_start],
            UNPROVEN, DASHED_RED,
        ))
    return result


def _flatten_pieces(pieces: Sequence[Mapping[str, Any]]) \
        -> tuple[list[list[float]], list[dict[str, Any]]]:
    points: list[list[float]] = []
    runs: list[dict[str, Any]] = []
    for piece in pieces:
        local = [_round_point(point) for point in piece["points"]]
        if not local:
            continue
        if not points:
            points.extend(local)
            start_edge = 0
        else:
            if _distance(points[-1], local[0]) > POINT_TOL_MM:
                raise RuntimeError(
                    f"piece {piece['kind']} is disconnected from prior piece"
                )
            local[0] = points[-1]
            start_edge = len(points) - 1
            points.extend(local[1:])
        end_edge = len(points) - 1
        if end_edge > start_edge:
            runs.append({
                "start_edge": start_edge,
                "end_edge": end_edge,
                "kind": piece["kind"],
                "authorization": piece["authorization"],
                "visual_style": piece["visual_style"],
            })
    if not points:
        raise RuntimeError("route item has no points")
    return points, runs


def _make_item(index: int, kind: str, start_time: float, end_time: float,
               pieces: Sequence[Mapping[str, Any]], **metadata: Any) \
        -> dict[str, Any]:
    if end_time + TIME_TOL_S < start_time:
        raise RuntimeError(f"route item {kind} has negative duration")
    points, runs = _flatten_pieces(pieces)
    lengths = [_distance(left, right)
               for left, right in zip(points, points[1:])]
    total = sum(lengths)
    point_times = [float(start_time)]
    if total > 1.0e-12:
        cumulative = 0.0
        duration = max(0.0, float(end_time) - float(start_time))
        for length in lengths:
            cumulative += length
            point_times.append(float(start_time) + duration * cumulative / total)
    item = {
        "index": index,
        "kind": kind,
        "start_time_s": round(float(start_time), ROUND_DIGITS),
        "end_time_s": round(float(end_time), ROUND_DIGITS),
        "points_mm": points,
        "point_times_s": [round(value, ROUND_DIGITS) for value in point_times],
        "runs": runs,
        "start_point_mm": points[0],
        "end_point_mm": points[-1],
        "live_endpoint_start_mm": points[0],
        "live_endpoint_end_mm": points[-1],
        "persistent_after_end": kind not in ("initial_hold", "final_hold"),
        **metadata,
    }
    return item


def _hold_item(index: int, kind: str, start_time: float, end_time: float,
               point: Sequence[float], **metadata: Any) -> dict[str, Any]:
    p = _round_point(point)
    return {
        "index": index,
        "kind": kind,
        "start_time_s": round(float(start_time), ROUND_DIGITS),
        "end_time_s": round(float(end_time), ROUND_DIGITS),
        "points_mm": [p],
        "point_times_s": [round(float(start_time), ROUND_DIGITS)],
        "runs": [],
        "start_point_mm": p,
        "end_point_mm": p,
        "live_endpoint_start_mm": p,
        "live_endpoint_end_mm": p,
        "persistent_after_end": False,
        **metadata,
    }


def _shaft_start(tail: Sequence[float], standoff: float, radius: float,
                 axial: float) -> list[float]:
    dx = float(tail[0])
    dz = float(tail[2]) - standoff
    norm = math.hypot(dx, dz)
    if norm <= 1.0e-9:
        dx, dz, norm = 0.0, -1.0, 1.0
    return _round_point((
        radius * dx / norm,
        axial,
        standoff + radius * dz / norm,
    ))


def _shaft_polyline(start: Sequence[float], standoff: float, radius: float,
                    delta_m1: float, axial_end: float) -> list[list[float]]:
    x0 = float(start[0])
    z0 = float(start[2]) - standoff
    y0 = float(start[1])
    arc_length = abs(delta_m1) * radius
    count = max(4, int(math.ceil(arc_length / MAX_POINT_JUMP_MM)))
    points = []
    # A world-stationary lay contact becomes the opposite angular trace in the
    # rotating spindle frame.  This is a diagnostic helix and remains unproven.
    for index in range(count + 1):
        fraction = index / count
        angle = -delta_m1 * fraction
        c, s = math.cos(angle), math.sin(angle)
        points.append(_round_point((
            x0 * c + z0 * s,
            y0 + (axial_end - y0) * fraction,
            standoff + (-x0 * s + z0 * c),
        )))
    return _subdivide_polyline(points)


def _first_half_start(*, winding: Mapping[str, Any], half_interval: Sequence[float],
                      raw_plan: Mapping[str, Any], meta: Mapping[str, Any],
                      manifest: Mapping[str, Any], timeline: Timeline) \
        -> list[float]:
    placement = _active_placement(raw_plan, 0)
    clockwise = bool(winding["clockwise"])
    tangential = placement["right_t"] if clockwise else placement["left_t"]
    return _coil_point(
        tooth=int(winding["tooth"]),
        slots=int(meta["teeth_count"]),
        standoff=float(manifest["m0_home_standoff"]),
        radial=_active_radial(meta, manifest, timeline, float(half_interval[0])),
        tangential=tangential,
        axial=float(raw_plan["job"]["stack_mm"]) / 2.0,
    )


def build_route(capture_path: Path = CAPTURE, plan_path: Path = PLAN,
                manifest_path: Path = MANIFEST) -> dict[str, Any]:
    """Build and internally validate the canonical raw conductor route."""

    capture_path = Path(capture_path).resolve()
    plan_path = Path(plan_path).resolve()
    manifest_path = Path(manifest_path).resolve()
    events = load_events(capture_path)
    timeline = Timeline(events)
    meta = timeline.meta
    raw_plan = _load_object(plan_path)
    manifest = _load_object(manifest_path)
    wire_handoff = _wire_handoff_contract(manifest)
    if (meta.get("controller_mode") != "upstream"
            or meta.get("controller_adapter_sha256") is not None
            or meta.get("winding_plan") is not None):
        raise RuntimeError("conductor route requires unmodified upstream capture")
    if int(meta.get("capture_schema", -1)) != EXPECTED_CAPTURE_SCHEMA:
        raise RuntimeError("canonical raw capture schema drifted")
    windings = winding_windows(events)
    if len(windings) != EXPECTED_PASSES:
        raise RuntimeError("canonical raw capture must contain 24 passes")
    turns = int(meta["turns"])
    if turns != EXPECTED_TURNS_PER_PASS:
        raise RuntimeError("canonical raw capture must contain 50 turns/pass")
    if (int(raw_plan["job"]["slots"]) != int(meta["teeth_count"])
            or int(raw_plan["job"]["turns_per_tooth"]) != turns):
        raise RuntimeError("raw capture and slot plan winding job differ")

    half_intervals: list[list[tuple[float, float]]] = []
    for winding in windings:
        direction = 1 if bool(winding["clockwise"]) else -1
        rows = _rotation_intervals(
            timeline.axes[2],
            float(winding["motionStart"]),
            float(winding["end"]),
            2 * turns,
            direction,
        )
        half_intervals.append(rows)

    first_starts = [
        _first_half_start(
            winding=winding,
            half_interval=half_intervals[index][0],
            raw_plan=raw_plan,
            meta=meta,
            manifest=manifest,
            timeline=timeline,
        )
        for index, winding in enumerate(windings)
    ]
    wraps = _raw_shaft_wraps(events, timeline)
    duration = max(
        float(timeline.t_end),
        max(_finite(event["t"], "event time") for event in events),
    )
    standoff = float(manifest["m0_home_standoff"])
    slots = int(meta["teeth_count"])
    stack_half = float(raw_plan["job"]["stack_mm"]) / 2.0
    shaft = manifest["wire"]["shaft_contact"]
    shaft_radius = float(shaft["radius_to_wire_center_mm"])

    items: list[dict[str, Any]] = []

    def add(item: dict[str, Any]) -> None:
        if item["index"] != len(items):
            raise RuntimeError("route item index drifted")
        if items:
            previous = items[-1]
            if abs(item["start_time_s"] - previous["end_time_s"]) > TIME_TOL_S:
                raise RuntimeError("route timeline contains a gap or overlap")
            if _distance(previous["end_point_mm"], item["start_point_mm"]) \
                    > POINT_TOL_MM:
                raise RuntimeError("route conductor graph is disconnected")
            item["start_point_mm"] = previous["end_point_mm"]
            item["live_endpoint_start_mm"] = previous["end_point_mm"]
            item["points_mm"][0] = previous["end_point_mm"]
        items.append(item)

    first_lay = half_intervals[0][0][0]
    add(_hold_item(
        0, "initial_hold", 0.0, first_lay, first_starts[0],
        phase=0, pass_index=0, tooth=int(windings[0]["tooth"]),
        authorization=UNPROVEN, visual_style=DASHED_RED,
        note=("incoming live conductor held at the first diagnostic lay point; "
              "no initial workholding/lead anchor has been authorized"),
    ))

    wrap_cursor = 0
    for pass_index, winding in enumerate(windings):
        phase = int(winding["phase"])
        tooth = int(winding["tooth"])
        clockwise = bool(winding["clockwise"])
        intervals = half_intervals[pass_index]
        for half_index, (start_t, end_t) in enumerate(intervals):
            turn_index = half_index // 2
            placement = _active_placement(raw_plan, turn_index)
            start_side = (placement["right_t"] if clockwise
                          else placement["left_t"])
            opposite = (placement["left_t"] if clockwise
                        else placement["right_t"])
            next_start = None
            if half_index % 2 == 1 and turn_index + 1 < turns:
                next_placement = _active_placement(raw_plan, turn_index + 1)
                next_t = (next_placement["right_t"] if clockwise
                          else next_placement["left_t"])
                next_interval = intervals[half_index + 1]
                next_start = _coil_point(
                    tooth=tooth,
                    slots=slots,
                    standoff=standoff,
                    radial=_active_radial(
                        meta, manifest, timeline, next_interval[0],
                    ),
                    tangential=next_t,
                    axial=stack_half,
                )
            pieces = _coil_half_pieces(
                half_index=half_index,
                tooth=tooth,
                slots=slots,
                standoff=standoff,
                stack_half=stack_half,
                radial_start=_active_radial(
                    meta, manifest, timeline, start_t,
                ),
                radial_end=_active_radial(meta, manifest, timeline, end_t),
                start_tangential=start_side,
                opposite_tangential=opposite,
                next_start=next_start,
            )
            add(_make_item(
                len(items), "winding_half_turn", start_t, end_t, pieces,
                phase=phase,
                pass_index=pass_index,
                tooth=tooth,
                turn=turn_index,
                half=half_index % 2,
                half_turn_index=half_index,
                clockwise=clockwise,
                raw_m0_start_rad=round(
                    float(timeline.axes[0].pos_at(start_t)), ROUND_DIGITS,
                ),
                raw_m0_end_rad=round(
                    float(timeline.axes[0].pos_at(end_t)), ROUND_DIGITS,
                ),
                clock_authority="raw Timeline directed M2 pi crossing",
                radial_authority="raw M0 mapped through CAD mm_per_rad",
                tangential_authority=(
                    "diagnostic slot plan only; not raw controller authority"
                ),
            ))

        if pass_index + 1 >= len(windings):
            continue
        next_start_time = half_intervals[pass_index + 1][0][0]
        next_start_point = first_starts[pass_index + 1]
        active_wrap = None
        if wrap_cursor < len(wraps):
            candidate = wraps[wrap_cursor]
            if (items[-1]["end_time_s"] - TIME_TOL_S <= candidate["start"]
                    and candidate["end"] <= next_start_time + TIME_TOL_S):
                active_wrap = candidate
        if active_wrap is None:
            add(_make_item(
                len(items), "tooth_transition",
                items[-1]["end_time_s"], next_start_time,
                [_piece(
                    "tooth_transition",
                    _line(items[-1]["end_point_mm"], next_start_point),
                    UNPROVEN, DASHED_RED,
                )],
                from_phase=phase,
                to_phase=int(windings[pass_index + 1]["phase"]),
                from_pass_index=pass_index,
                to_pass_index=pass_index + 1,
                from_tooth=tooth,
                to_tooth=int(windings[pass_index + 1]["tooth"]),
                authorization=UNPROVEN,
                visual_style=DASHED_RED,
                note=("spindle-local straight diagnostic connector across raw "
                      "indexing; production cap/flyer route not integrated"),
            ))
            continue

        wrap_cursor += 1
        axial_start = 10.25 if active_wrap["number"] == 1 else 12.5
        axial_end = axial_start + min(1.4, 0.5 * active_wrap["turns"])
        shaft_start = _shaft_start(
            items[-1]["end_point_mm"], standoff, shaft_radius, axial_start,
        )
        add(_make_item(
            len(items), "to_shaft_wrap",
            items[-1]["end_time_s"], active_wrap["start"],
            [_piece(
                "to_shaft_wrap",
                _line(items[-1]["end_point_mm"], shaft_start),
                UNPROVEN, DASHED_RED,
            )],
            phase=phase,
            after_pass_index=pass_index,
            wrap_number=active_wrap["number"],
            authorization=UNPROVEN,
            visual_style=DASHED_RED,
        ))
        shaft_points = _shaft_polyline(
            shaft_start, standoff, shaft_radius,
            active_wrap["delta_m1_rad"], axial_end,
        )
        add(_make_item(
            len(items), "shaft_wrap",
            active_wrap["start"], active_wrap["end"],
            [_piece("shaft_wrap", shaft_points, UNPROVEN, DASHED_RED)],
            phase_before=phase,
            phase_after=int(windings[pass_index + 1]["phase"]),
            after_pass_index=pass_index,
            before_pass_index=pass_index + 1,
            wrap_number=active_wrap["number"],
            raw_start_m1_rad=active_wrap["start_m1_rad"],
            raw_end_m1_rad=active_wrap["end_m1_rad"],
            raw_delta_m1_rad=active_wrap["delta_m1_rad"],
            raw_turns=active_wrap["turns"],
            source_command=active_wrap["source_command"],
            authorization=UNPROVEN,
            visual_style=DASHED_RED,
            persistent_after_end=True,
            note=("raw M1 clock and turn count; diagnostic shaft helix is not "
                  "authorized against the integrated sleeve/caps/flyer"),
        ))
        add(_make_item(
            len(items), "from_shaft_wrap",
            active_wrap["end"], next_start_time,
            [_piece(
                "from_shaft_wrap",
                _line(items[-1]["end_point_mm"], next_start_point),
                UNPROVEN, DASHED_RED,
            )],
            from_phase=phase,
            to_phase=int(windings[pass_index + 1]["phase"]),
            wrap_number=active_wrap["number"],
            to_pass_index=pass_index + 1,
            authorization=UNPROVEN,
            visual_style=DASHED_RED,
        ))

    if wrap_cursor != len(wraps):
        raise RuntimeError("not every raw shaft wrap was inserted into route")
    add(_hold_item(
        len(items), "final_hold", items[-1]["end_time_s"], duration,
        items[-1]["end_point_mm"],
        phase=int(windings[-1]["phase"]),
        pass_index=len(windings) - 1,
        tooth=int(windings[-1]["tooth"]),
        authorization=UNPROVEN,
        visual_style=DASHED_RED,
        note="terminal live span remains visible through cycle completion",
    ))

    all_runs = [run for item in items for run in item["runs"]]
    maximum_jump = max(
        (_distance(left, right)
         for item in items
         for left, right in zip(item["points_mm"], item["points_mm"][1:])),
        default=0.0,
    )
    edge_kind_counts: dict[str, int] = {}
    for run in all_runs:
        count = int(run["end_edge"]) - int(run["start_edge"])
        edge_kind_counts[run["kind"]] = (
            edge_kind_counts.get(run["kind"], 0) + count
        )
    unproven_kinds = sorted({
        run["kind"] for run in all_runs
        if run["authorization"] == UNPROVEN
    })
    gates = {
        "canonical_unmodified_raw_capture": True,
        "full_virtual_timeline_has_live_endpoint": True,
        "ordered_conductor_graph_connected": True,
        "live_endpoint_fields_equal_ordered_tail": True,
        "point_jump_bound_satisfied": maximum_jump <= MAX_POINT_JUMP_MM + 1e-8,
        "all_24_passes_and_2400_half_turns_present": (
            sum(item["kind"] == "winding_half_turn" for item in items)
            == EXPECTED_PASSES * EXPECTED_HALF_TURNS_PER_PASS
        ),
        "inter_turn_advancement_explicit": (
            edge_kind_counts.get("inter_turn_advance", 0)
            >= EXPECTED_PASSES * (EXPECTED_TURNS_PER_PASS - 1)
        ),
        "end_cap_transitions_explicit": (
            edge_kind_counts.get("cap_transition", 0) > 0
        ),
        "tooth_or_phase_transitions_explicit": (
            sum(item["kind"] in {
                "tooth_transition", "to_shaft_wrap", "from_shaft_wrap",
            } for item in items) == EXPECTED_PASSES - 1 + len(wraps)
        ),
        "two_raw_shaft_wraps_persist": (
            sum(item["kind"] == "shaft_wrap"
                and item["persistent_after_end"] for item in items) == 2
        ),
        "every_unproven_transition_is_dashed_red": all(
            run["authorization"] != UNPROVEN
            or run["visual_style"] == DASHED_RED
            for run in all_runs
        ),
        "static_supply_to_flyer_bore_seam_exact": (
            wire_handoff["status"] == "PASS"
            and wire_handoff["maximum_gap_mm"]
            <= wire_handoff["tolerance_mm"]
        ),
    }
    structural_status = "PASS" if all(gates.values()) else "FAIL"
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "FAIL",
        "structural_status": structural_status,
        "decision": "CONNECTED_PRESENTATION_ROUTE_GEOMETRY_UNPROVEN",
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "coordinate_frame": {
            "name": "spindle_local_machine_mm",
            "axis": "+Y is stator shaft; M0 carriage translation and M1 rotation are player transforms",
            "origin_contract": (
                "same coordinates used by the spindle child in the GLB; "
                f"stator axis is z={standoff:.9f} mm before M0 transform"
            ),
        },
        "timeline": {
            "start_time_s": 0.0,
            "end_time_s": round(duration, ROUND_DIGITS),
            "coverage_item_count": len(items),
            "no_hidden_live_interval": bool(gates[
                "full_virtual_timeline_has_live_endpoint"
            ]),
        },
        "job": {
            "pass_count": len(windings),
            "turns_per_pass": turns,
            "half_turn_count": EXPECTED_PASSES * 2 * turns,
            "slot_count": slots,
            "phase_count": len({int(row["phase"]) for row in windings}),
            "wire_finished_diameter_mm": float(
                raw_plan["job"]["wire_finished_d_mm"]
            ),
        },
        "pass_order": [{
            "pass_index": index,
            "phase": int(winding["phase"]),
            "tooth": int(winding["tooth"]),
            "clockwise": bool(winding["clockwise"]),
        } for index, winding in enumerate(windings)],
        "items": items,
        "shaft_wrap_item_indices": [
            item["index"] for item in items if item["kind"] == "shaft_wrap"
        ],
        "wire_handoff_contract": wire_handoff,
        "edge_kind_counts": edge_kind_counts,
        "unproven_edge_kinds": unproven_kinds,
        "structural_gates": gates,
        "metrics": {
            "maximum_point_jump_mm": maximum_jump,
            "maximum_allowed_point_jump_mm": MAX_POINT_JUMP_MM,
            "ordered_item_count": len(items),
            "ordered_point_count": sum(len(item["points_mm"])
                                       for item in items),
            "ordered_edge_count": sum(max(0, len(item["points_mm"]) - 1)
                                      for item in items),
            "unproven_run_count": sum(
                run["authorization"] == UNPROVEN for run in all_runs
            ),
        },
        "release_blockers": [
            "Production end-cap channels are not integrated into this route.",
            "Inter-turn advancement has no swept cap/liner clearance authority.",
            "Tooth-to-tooth transitions are diagnostic straight connectors.",
            "Raw shaft clocks are exact, but the displayed shaft helices are not cleared against the integrated sleeve/caps/flyer.",
            "Live flyer-tip-to-tail geometry remains a presentation span, not a tensioned-wire or snag proof.",
        ],
        "source_hashes": {
            "raw_capture_sha256": _sha256(capture_path),
            "slot_winding_plan_sha256": _sha256(plan_path),
            "slot_winding_plan_proof_sha256": raw_plan.get("proof_sha256"),
            "cad_manifest_sha256": _sha256(manifest_path),
            "generator_source_sha256": _sha256(Path(__file__)),
            "traj_source_sha256": _sha256(HERE / "traj.py"),
        },
        "source_paths": {
            "raw_capture": str(capture_path),
            "slot_winding_plan": str(plan_path),
            "cad_manifest": str(manifest_path),
        },
        "limitations": [
            "The route is quasi-static presentation geometry, not an elastic rod, friction, sag, tension, or snag simulation.",
            "Only raw axis clocks and M0 radial motion are controller authority; diagnostic tangential/transition geometry is visibly distinguished.",
            "A structural PASS means the player cannot hide or disconnect the conductor. It does not override the top-level FAIL.",
        ],
    }
    report["report_sha256"] = _canonical_hash(report)
    validate_route_artifact(
        report,
        capture_path=capture_path,
        plan_path=plan_path,
        manifest_path=manifest_path,
    )
    return report


def _item_tail_at(item: Mapping[str, Any], time_s: float) -> list[float]:
    points = item["points_mm"]
    if len(points) == 1:
        return list(points[0])
    times = item["point_times_s"]
    if time_s <= times[0]:
        return list(points[0])
    if time_s >= times[-1]:
        return list(points[-1])
    for index, (left_t, right_t) in enumerate(zip(times, times[1:])):
        if left_t - TIME_TOL_S <= time_s <= right_t + TIME_TOL_S:
            if right_t <= left_t + 1.0e-12:
                return list(points[index + 1])
            return _lerp(
                points[index], points[index + 1],
                (time_s - left_t) / (right_t - left_t),
            )
    raise RuntimeError("route time is not inside item point clock")


def live_endpoint_at(report: Mapping[str, Any], time_s: float) -> list[float]:
    """Return the unique live/deposited tail at any virtual time."""

    start = float(report["timeline"]["start_time_s"])
    end = float(report["timeline"]["end_time_s"])
    if time_s < start - TIME_TOL_S or time_s > end + TIME_TOL_S:
        raise ValueError("time is outside conductor route timeline")
    for item in report["items"]:
        if (float(item["start_time_s"]) - TIME_TOL_S <= time_s
                <= float(item["end_time_s"]) + TIME_TOL_S):
            return _item_tail_at(item, time_s)
    raise RuntimeError("no conductor item covers requested time")


def validate_route_artifact(report: Mapping[str, Any], *,
                            capture_path: Path = CAPTURE,
                            plan_path: Path = PLAN,
                            manifest_path: Path = MANIFEST) -> None:
    """Fail closed on stale sources, disconnected geometry, or hidden time."""

    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported continuous conductor route schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("continuous conductor route report hash mismatch")
    expected_hashes = {
        "raw_capture_sha256": _sha256(Path(capture_path)),
        "slot_winding_plan_sha256": _sha256(Path(plan_path)),
        "cad_manifest_sha256": _sha256(Path(manifest_path)),
        "generator_source_sha256": _sha256(Path(__file__)),
        "traj_source_sha256": _sha256(HERE / "traj.py"),
    }
    actual_hashes = report.get("source_hashes", {})
    stale = [name for name, value in expected_hashes.items()
             if actual_hashes.get(name) != value]
    if stale:
        raise ValueError("continuous conductor route has stale sources: "
                         + ", ".join(stale))
    expected_handoff = _wire_handoff_contract(
        _load_object(Path(manifest_path))
    )
    if report.get("wire_handoff_contract") != expected_handoff:
        raise ValueError("continuous conductor route wire handoff drifted")
    items = report.get("items")
    if not isinstance(items, list) or not items:
        raise ValueError("continuous conductor route has no ordered items")
    if [item.get("index") for item in items] != list(range(len(items))):
        raise ValueError("continuous conductor route item order drifted")
    if abs(float(items[0]["start_time_s"])) > TIME_TOL_S:
        raise ValueError("continuous conductor route does not start at t=0")
    if abs(float(items[-1]["end_time_s"])
           - float(report["timeline"]["end_time_s"])) > TIME_TOL_S:
        raise ValueError("continuous conductor route does not cover final time")

    required_edge_kinds: set[str] = set()
    maximum_jump = 0.0
    for index, item in enumerate(items):
        points = item.get("points_mm")
        if not isinstance(points, list) or not points:
            raise ValueError(f"route item {index} has no points")
        if item.get("start_point_mm") != points[0] \
                or item.get("end_point_mm") != points[-1]:
            raise ValueError(f"route item {index} endpoint fields drifted")
        if (item.get("live_endpoint_start_mm") != points[0]
                or item.get("live_endpoint_end_mm") != points[-1]):
            raise ValueError(f"route item {index} live endpoint is not tail")
        if index:
            previous = items[index - 1]
            if abs(float(item["start_time_s"])
                   - float(previous["end_time_s"])) > TIME_TOL_S:
                raise ValueError(f"route item {index} leaves timeline gap")
            if _distance(previous["end_point_mm"], points[0]) > POINT_TOL_MM:
                raise ValueError(f"route item {index} disconnects conductor")
        edge_count = max(0, len(points) - 1)
        cursor = 0
        for run in item.get("runs", []):
            if int(run["start_edge"]) != cursor:
                raise ValueError(f"route item {index} run coverage has gap")
            cursor = int(run["end_edge"])
            if cursor <= int(run["start_edge"]) or cursor > edge_count:
                raise ValueError(f"route item {index} run range is invalid")
            required_edge_kinds.add(str(run["kind"]))
            if (run["kind"] in TRANSITION_KINDS
                    and (run.get("authorization") != UNPROVEN
                         or run.get("visual_style") != DASHED_RED)):
                raise ValueError(
                    f"route item {index} invents transition authorization"
                )
        if cursor != edge_count:
            raise ValueError(f"route item {index} runs do not cover all edges")
        for left, right in zip(points, points[1:]):
            maximum_jump = max(maximum_jump, _distance(left, right))
        for probe in (float(item["start_time_s"]),
                      (float(item["start_time_s"])
                       + float(item["end_time_s"])) / 2.0,
                      float(item["end_time_s"])):
            tail = _item_tail_at(item, probe)
            if len(tail) != 3 or not all(math.isfinite(float(v)) for v in tail):
                raise ValueError(f"route item {index} has invalid live tail")
    if maximum_jump > MAX_POINT_JUMP_MM + 1.0e-8:
        raise ValueError("continuous conductor route exceeds point jump bound")
    missing = TRANSITION_KINDS - required_edge_kinds
    if missing:
        raise ValueError("continuous conductor route omits transitions: "
                         + ", ".join(sorted(missing)))
    shaft_items = [item for item in items if item["kind"] == "shaft_wrap"]
    if len(shaft_items) != 2 or not all(
            item.get("persistent_after_end") is True for item in shaft_items):
        raise ValueError("two persistent raw shaft wraps are required")
    if report.get("structural_status") != "PASS":
        raise ValueError("continuous conductor structural gates did not pass")
    if (report.get("status") != "FAIL"
            or report.get("production_authorized") is not False
            or not report.get("release_blockers")):
        raise ValueError("continuous conductor route did not fail closed")


def render_markdown(report: Mapping[str, Any]) -> str:
    gates = report["structural_gates"]
    lines = [
        "# Continuous phase-aware conductor route",
        "",
        f"**Overall release status: {report['status']}**  ",
        f"**Structural presentation status: {report['structural_status']}**  ",
        f"**Decision: {report['decision']}**",
        "",
        "The canonical raw player and this validator consume the same ordered "
        "route artifact. The live endpoint exists through the full timeline; "
        "completed turns, advancement, tooth transitions, and both shaft "
        "wraps are one connected conductor.",
        "",
        "## Structural gates",
        "",
    ]
    lines.extend(
        f"- {'PASS' if ok else 'FAIL'} - `{name}`"
        for name, ok in gates.items()
    )
    lines.extend((
        "",
        "## Route metrics",
        "",
        f"- Ordered items: {report['metrics']['ordered_item_count']}",
        f"- Ordered points: {report['metrics']['ordered_point_count']}",
        f"- Maximum point jump: {report['metrics']['maximum_point_jump_mm']:.9f} mm",
        f"- Persistent shaft wraps: {len(report['shaft_wrap_item_indices'])}",
        "",
        "## Release blockers",
        "",
    ))
    lines.extend(f"- {blocker}" for blocker in report["release_blockers"])
    lines.extend((
        "",
        "All unsupported transitions remain red/dashed and the top-level "
        "status remains FAIL. Structural continuity does not authorize parts, "
        "production CAD integration, or winding.",
        "",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ))
    return "\n".join(lines)


def write_outputs(report: Mapping[str, Any], json_path: Path = OUTPUT_JSON,
                  markdown_path: Path = OUTPUT_MD) -> None:
    json_path = Path(json_path)
    markdown_path = Path(markdown_path)
    json_path.parent.mkdir(parents=True, exist_ok=True)
    markdown_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    markdown_path.write_text(render_markdown(report), encoding="utf-8")


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", type=Path, default=CAPTURE)
    parser.add_argument("--plan", type=Path, default=PLAN)
    parser.add_argument("--manifest", type=Path, default=MANIFEST)
    parser.add_argument("--json", type=Path, default=OUTPUT_JSON)
    parser.add_argument("--markdown", type=Path, default=OUTPUT_MD)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(list(argv) if argv is not None else None)
    if args.validate_only:
        report = _load_object(args.json)
        validate_route_artifact(
            report, capture_path=args.capture, plan_path=args.plan,
            manifest_path=args.manifest,
        )
    else:
        report = build_route(args.capture, args.plan, args.manifest)
        write_outputs(report, args.json, args.markdown)
    print(
        f"continuous conductor {report['status']} / "
        f"structural {report['structural_status']}: "
        f"{report['metrics']['ordered_item_count']} items, "
        f"max jump {report['metrics']['maximum_point_jump_mm']:.6f} mm, "
        f"sha256 {report['report_sha256']}"
    )
    # The artifact is intentionally release-FAIL.  Generation succeeds when
    # the structural route is valid; callers inspect status for authorization.
    return 0 if report["structural_status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
