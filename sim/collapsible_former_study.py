"""Fail-closed study of a passive split/collapsible coil former.

The candidate intentionally leaves the production controller and assembly
untouched.  It asks whether the exact 24-slot, OD46 x stack15, 50-turn job can
be wound on a generous R3 former and then transferred into the two lined slots
flanking the active tooth using only the existing post-pass M0 retract.

Authority for motion is the canonical *unmodified upstream* capture
``out/capture/upstream_current_raw.jsonl``.  Project-adapter captures are not
accepted.  The transfer calculation is deliberately optimistic: the complete
active 50-wire side may translate as a perfectly controlled rigid pack and a
two-half former may add one common tangential collapse.  If even that ideal
motion has no state, a real passive two-piece former is not released.

Run directly to write ``out/reports/collapsible_former.json`` and ``.md``.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable

import numpy as np
import shapely
from build123d import Vertex
from shapely.affinity import rotate
from shapely.geometry import Point, box
from shapely.ops import unary_union


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import slot_packing_audit  # noqa: E402
import stator_model  # noqa: E402
from params import DEFAULT_STATOR, PARAMS  # noqa: E402
from traj import Timeline, load_events, winding_windows  # noqa: E402


SCHEMA = "collapsible-former-study/v1"
RAW_CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
PACKING_REPORT = ROOT / "out" / "reports" / "slot_packing.json"
OUTPUT_JSON = ROOT / "out" / "reports" / "collapsible_former.json"
OUTPUT_MD = ROOT / "out" / "reports" / "collapsible_former.md"

TRANSFER_WITNESS_SHIFT_MM = 2.0
TRANSFER_DELTA_STEP_MM = 0.0005
MINIMUM_WIRE_CENTER_BEND_RADIUS_MM = 3.0
FORMER_SHELL_THICKNESS_MM = 0.40
CAPTURE_POSITION_TOLERANCE_RAD = 0.0035


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(value: dict[str, Any]) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _crossing_times(track, start_t: float, start_pos: float,
                    direction: int, count: int) -> list[float]:
    """Invert a piecewise-linear axis track at directed pi crossings."""

    result: list[float] = []
    for index in range(count):
        target = start_pos + direction * index * math.pi
        if index == 0:
            result.append(start_t)
            continue
        found = None
        for (left_t, _left_p), (right_t, right_p) in zip(
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


def _capture_contract(events: list[dict[str, Any]],
                      packing: dict[str, Any]) -> dict[str, Any]:
    meta = next(row for row in events if row["e"] == "meta")
    if meta.get("controller_mode") != "upstream":
        raise RuntimeError("collapsible-former study requires upstream capture")
    if meta.get("controller_adapter_sha256") is not None:
        raise RuntimeError("adapter-backed capture is not authoritative")
    if meta.get("capture_schema") != 4:
        raise RuntimeError("expected capture schema 4")
    job = meta.get("job", {})
    expected = {
        "slots": 24,
        "od_mm": 46.0,
        "stack_mm": 15.0,
        "wire_finished_d_mm": 0.22352,
    }
    for key, value in expected.items():
        if abs(float(job.get(key, math.nan)) - float(value)) > 1.0e-12:
            raise RuntimeError(f"raw capture job drifted at {key}")
    if int(meta.get("turns", -1)) != 50:
        raise RuntimeError("raw capture is not the 50-turn job")
    if packing.get("schema") != "slot-packing/v2" or packing.get("status") != "PASS":
        raise RuntimeError("authoritative packing report is not PASS v2")
    if int(packing["selected_schedule"]["turns_per_tooth"]) != 50:
        raise RuntimeError("packing report is not the 50-turn release job")
    velocities = [float(value) for value in meta["velocities"]]
    if velocities[:3] != [20.0, 20.0, 20.0]:
        raise RuntimeError("canonical settings-only v20 timing fix is absent")

    timeline = Timeline(events)
    windows = winding_windows(events)
    if len(windows) != 24:
        raise RuntimeError("raw capture does not contain 24 tooth passes")

    radial_span = [float(value) for value in job["radial_winding_span_mm"]]
    contact_z = float(job["wire_contact_z_mm"])
    wire_d = float(job["wire_finished_d_mm"])
    pass_rows: list[dict[str, Any]] = []
    for pass_index, window in enumerate(windows):
        direction = 1 if window["clockwise"] else -1
        motion_start = float(window["motionStart"])
        m2_start = timeline.axes[2].pos_at(motion_start)
        crossings = _crossing_times(
            timeline.axes[2], motion_start, m2_start, direction, 101,
        )
        if len(crossings) != 101:
            raise RuntimeError(
                f"pass {pass_index} has {len(crossings)} of 101 crossings"
            )
        radial = [
            float(PARAMS.stator_axis_z(timeline.axes[0].pos_at(time))
                  - contact_z)
            for time in crossings
        ]
        turns = radial[::2][:50]
        pitch = [abs(right - left) for left, right in zip(turns, turns[1:])]
        pass_rows.append({
            "pass_index": pass_index,
            "tooth": int(window["tooth"]),
            "motion_sign": direction,
            "motion_start_s": motion_start,
            "m0_at_first_flyer_motion_rad": float(
                timeline.axes[0].pos_at(motion_start)
            ),
            "half_turn_crossings": len(crossings),
            "radial_center_range_mm": [min(radial), max(radial)],
            "minimum_full_turn_pitch_mm": min(pitch),
            "maximum_full_turn_pitch_mm": max(pitch),
            "intervals_below_finished_wire_diameter": sum(
                value < wire_d - 1.0e-9 for value in pitch
            ),
            "zero_pitch_intervals": sum(value < 1.0e-9 for value in pitch),
        })

    tooth_order = [int(row["tooth"]) for row in pass_rows]
    seen: set[int] = set()
    neighbor_counts: list[dict[str, Any]] = []
    for pass_index, tooth in enumerate(tooth_order):
        neighbors = ((tooth - 1) % 24, (tooth + 1) % 24)
        already = [value for value in neighbors if value in seen]
        neighbor_counts.append({
            "pass_index": pass_index,
            "tooth": tooth,
            "already_wound_neighbor_count": len(already),
            "already_wound_neighbors": already,
        })
        seen.add(tooth)

    index_target = float(meta["m1_rotating_position"])
    retract_rows: list[dict[str, Any]] = []
    for pass_index, window in enumerate(windows):
        start_index = events.index(next(
            row for row in events
            if row["e"] == "wind_wire"
            and float(row["t"]) == float(window["start"])
            and int(row["args"][0]) == int(window["tooth"])
        ))
        end_index = next(
            index for index in range(start_index, len(events))
            if events[index]["e"] == "wind_wire_done"
        )
        sub = events[start_index:end_index + 1]
        retract = next(
            row for row in reversed(sub)
            if row["e"] == "cmd" and int(row.get("m", -1)) == 0
            and abs(float(row["a"]) - index_target) <= 1.0e-6
        )
        retract_t = float(retract["t"])
        start_m0 = float(timeline.axes[0].pos_at(retract_t))
        stroke_rad = abs(start_m0 - index_target)
        available_s = float(window["end"]) - retract_t
        required_s = stroke_rad / velocities[0]
        retract_rows.append({
            "pass_index": pass_index,
            "start_m0_rad": start_m0,
            "index_target_m0_rad": index_target,
            "stroke_mm": stroke_rad * PARAMS.mm_per_rad,
            "motion_time_s": required_s,
            "available_before_index_s": available_s,
            "settling_margin_s": available_s - required_s,
            "arrives_before_index": required_s <= available_s + 1.0e-12,
        })

    shaft_rows: list[dict[str, Any]] = []
    for row in events:
        if row["e"] != "wind_wire_around_shaft_done":
            continue
        pose = timeline.pose_at(float(row["t"]))
        shaft_rows.append({
            "done_t_s": float(row["t"]),
            "m0_rad": float(pose[0]),
            "m1_rad": float(pose[1]),
            "m2_rad": float(pose[2]),
            "m0_at_retracted_index_pose": (
                abs(float(pose[0]) - index_target)
                <= CAPTURE_POSITION_TOLERANCE_RAD
            ),
        })
    if len(shaft_rows) != 2:
        raise RuntimeError("raw capture does not contain two shaft wraps")

    ease_rows = pass_rows
    return {
        "capture_path": RAW_CAPTURE.relative_to(ROOT).as_posix(),
        "capture_sha256": _sha256(RAW_CAPTURE),
        "settings_sha256": str(meta["settings_sha256"]),
        "winder_commit": str(meta["winder_commit"]),
        "controller_mode": str(meta["controller_mode"]),
        "adapter_sha256": meta["controller_adapter_sha256"],
        "velocities_rad_s": velocities,
        "job": job,
        "m0_wind_range_rad": [float(value) for value in meta["m0_wind_range"]],
        "tooth_order": tooth_order,
        "motion_sign_counts": dict(sorted(Counter(
            int(row["motion_sign"]) for row in pass_rows
        ).items())),
        "neighbor_history": neighbor_counts,
        "neighbor_case_counts": dict(sorted(Counter(
            int(row["already_wound_neighbor_count"])
            for row in neighbor_counts
        ).items())),
        "ease_law": {
            "pass_count": len(ease_rows),
            "all_first_flyer_motion_inside_raw_wind_range": all(
                min(meta["m0_wind_range"]) - 1.0e-9
                <= row["m0_at_first_flyer_motion_rad"]
                <= max(meta["m0_wind_range"]) + 1.0e-9
                for row in ease_rows
            ),
            "minimum_full_turn_pitch_mm": min(
                row["minimum_full_turn_pitch_mm"] for row in ease_rows
            ),
            "maximum_full_turn_pitch_mm": max(
                row["maximum_full_turn_pitch_mm"] for row in ease_rows
            ),
            "minimum_intervals_below_wire_per_pass": min(
                row["intervals_below_finished_wire_diameter"]
                for row in ease_rows
            ),
            "maximum_intervals_below_wire_per_pass": max(
                row["intervals_below_finished_wire_diameter"]
                for row in ease_rows
            ),
            "every_pass_has_zero_pitch_interval": all(
                row["zero_pitch_intervals"] >= 1 for row in ease_rows
            ),
            "commanded_radial_span_mm": radial_span,
            "rows": ease_rows,
        },
        "post_pass_retract": {
            "index_target_m0_rad": index_target,
            "minimum_stroke_mm": min(row["stroke_mm"] for row in retract_rows),
            "maximum_stroke_mm": max(row["stroke_mm"] for row in retract_rows),
            "minimum_settling_margin_s": min(
                row["settling_margin_s"] for row in retract_rows
            ),
            "all_arrive_before_index": all(
                row["arrives_before_index"] for row in retract_rows
            ),
            "rows": retract_rows,
        },
        "shaft_wrap_done_poses": shaft_rows,
    }


def _slot_core_polygon():
    """Source stator section in the slot-bisector local frame."""

    spec = DEFAULT_STATOR
    hub_radius = spec.od * spec.hub_od_ratio / 2.0
    shoe_outer = spec.od / 2.0
    neck_start = hub_radius - 1.0
    neck_end = neck_start + (
        slot_packing_audit.SHOE_INNER_RADIUS_MM - hub_radius + 2.0
    )
    core = [Point(0.0, 0.0).buffer(hub_radius, quad_segs=256)]
    for angle in (
        -slot_packing_audit.SLOT_BISECTOR_RAD,
        slot_packing_audit.SLOT_BISECTOR_RAD,
    ):
        neck = box(
            neck_start, -slot_packing_audit.TOOTH_HALF_NECK_MM,
            neck_end, slot_packing_audit.TOOTH_HALF_NECK_MM,
        )
        core.append(rotate(
            neck, math.degrees(angle), origin=(0.0, 0.0),
        ))
        core.append(slot_packing_audit._annular_sector(
            slot_packing_audit.SHOE_INNER_RADIUS_MM,
            shoe_outer,
            angle,
            0.36 * slot_packing_audit.SLOT_PITCH_RAD,
            samples=1024,
        ))
    return unary_union(core)


def _packing_arrays(packing: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    schedule = packing["selected_schedule"]
    positive = np.asarray([
        row["slot_frame_uv_mm"] for row in schedule["side_positive"]
    ], dtype=float)
    negative = np.asarray([
        row["slot_frame_uv_mm"] for row in schedule["side_negative"]
    ], dtype=float)
    if positive.shape != (50, 2) or negative.shape != (50, 2):
        raise RuntimeError("packing arrays are not 50 x 2")
    return positive, negative


def _tangent_contact_graph(points: np.ndarray, wire_d: float) -> dict[str, Any]:
    bonds: list[tuple[int, int, np.ndarray]] = []
    for left in range(len(points)):
        for right in range(left + 1, len(points)):
            delta = points[right] - points[left]
            if abs(float(np.linalg.norm(delta)) - wire_d) <= 1.0e-8:
                bonds.append((left, right, delta))
    orientations = sorted({
        (round(float(delta[0] / wire_d), 6),
         round(float(delta[1] / wire_d), 6))
        for _left, _right, delta in bonds
    })
    required = {(0.5, 0.866025), (0.5, -0.866025)}
    normalized = set(orientations)
    if not required.issubset(normalized):
        raise RuntimeError("triangular contact graph lost both shear witnesses")
    return {
        "tangent_bond_count": len(bonds),
        "normalized_bond_orientations": [list(value) for value in orientations],
        "opposed_diagonal_bonds_present": True,
        "affine_shear_noncompression_branches": {
            "derivation": (
                "For bonds (du,dv)=d(1/2,+/-sqrt(3)/2), the shear "
                "v'=v+k*u gives L^2/d^2=1 +/- sqrt(3)k/2 + k^2/4. "
                "Requiring both lengths >=d leaves k=0 or |k|>=2sqrt(3)."
            ),
            "zero_branch": 0.0,
            "large_branch_abs_k_min": 2.0 * math.sqrt(3.0),
        },
    }


def _shear_wedge_bounds(points: np.ndarray, radial_shift: float) -> dict[str, float]:
    shifted_u = points[:, 0] + radial_shift
    v = points[:, 1]
    centered_u = points[:, 0] - float(points[:, 0].mean())
    tangent = math.tan(slot_packing_audit.SLOT_BISECTOR_RAD)

    def feasible(k: float) -> bool:
        lower = float(np.max(-shifted_u * tangent - v - k * centered_u))
        upper = float(np.min(shifted_u * tangent - v - k * centered_u))
        return lower <= upper + 1.0e-12

    def boundary(sign: int) -> float:
        low, high = 0.0, 4.0
        if not feasible(low):
            raise RuntimeError("zero shear unexpectedly leaves slot wedge")
        while feasible(sign * high):
            high *= 2.0
        for _ in range(80):
            middle = (low + high) / 2.0
            if feasible(sign * middle):
                low = middle
            else:
                high = middle
        return sign * low

    negative = boundary(-1)
    positive = boundary(+1)
    return {
        "minimum_feasible_k": negative,
        "maximum_feasible_k": positive,
        "large_noncompressing_negative_branch_fits": (
            negative <= -2.0 * math.sqrt(3.0)
        ),
        "large_noncompressing_positive_branch_fits": (
            positive >= 2.0 * math.sqrt(3.0)
        ),
    }


@dataclass(frozen=True)
class TransferWitness:
    best_delta_mm: float
    sampled_best_margin_mm: float
    global_upper_bound_margin_mm: float
    polygon_core_margin_mm: float
    exact_occ_core_margin_mm: float
    neighbor_margin_mm: float
    core_witness_wire: int
    neighbor_witness_pair: tuple[int, int]
    delta_search_bounds_mm: tuple[float, float]
    delta_step_mm: float


def _uniform_transfer_witness(points: np.ndarray, neighbor: np.ndarray,
                              wire_d: float, liner: float,
                              radial_shift: float) -> TransferWitness:
    """Exhaust one common tangential collapse at a fatal transfer slice.

    The score is 1-Lipschitz in the common tangential offset.  Sampling the
    complete slot-wedge interval at ``TRANSFER_DELTA_STEP_MM`` therefore gives
    a global upper bound equal to sampled_best + half a step (plus the tiny
    polygon chord bound).  A negative upper bound proves that no unsampled
    common-offset state can pass.
    """

    core = _slot_core_polygon()
    required_core = wire_d / 2.0 + liner
    shifted_u = points[:, 0] + radial_shift
    tangent = math.tan(slot_packing_audit.SLOT_BISECTOR_RAD)
    lower = float(np.max(-shifted_u * tangent - points[:, 1]))
    upper = float(np.min(shifted_u * tangent - points[:, 1]))
    sample_count = int(math.ceil((upper - lower) / TRANSFER_DELTA_STEP_MM)) + 1
    deltas = np.linspace(lower, upper, sample_count)
    actual_step = float(deltas[1] - deltas[0])

    core_margins = np.empty(len(deltas), dtype=float)
    neighbor_margins = np.empty(len(deltas), dtype=float)
    core_indices = np.empty(len(deltas), dtype=int)
    neighbor_pairs: list[tuple[int, int]] = []
    chunk = 400
    for start in range(0, len(deltas), chunk):
        local = deltas[start:start + chunk]
        poses = np.repeat(points[None, :, :], len(local), axis=0)
        poses[:, :, 0] += radial_shift
        poses[:, :, 1] += local[:, None]
        distances = np.asarray(shapely.distance(
            shapely.points(poses[:, :, 0].ravel(), poses[:, :, 1].ravel()),
            core,
        ), dtype=float).reshape(len(local), len(points))
        core_margins[start:start + len(local)] = (
            distances.min(axis=1) - required_core
        )
        core_indices[start:start + len(local)] = distances.argmin(axis=1)

        for local_index, pose in enumerate(poses):
            pair_distances = np.linalg.norm(
                pose[:, None, :] - neighbor[None, :, :], axis=2,
            )
            pair = np.unravel_index(
                int(np.argmin(pair_distances)), pair_distances.shape,
            )
            neighbor_margins[start + local_index] = (
                float(pair_distances[pair]) - wire_d
            )
            neighbor_pairs.append((int(pair[0]), int(pair[1])))

    scores = np.minimum(core_margins, neighbor_margins)
    best_index = int(np.argmax(scores))
    best_delta = float(deltas[best_index])
    polygon_core_margin = float(core_margins[best_index])
    neighbor_margin = float(neighbor_margins[best_index])
    core_index = int(core_indices[best_index])
    neighbor_pair = neighbor_pairs[best_index]

    # Exact OpenCascade postcheck in the same slot-bisector frame.  The
    # source stator has a slot centered on +X between teeth +/-7.5 degrees.
    exact_core = stator_model.stator(DEFAULT_STATOR)
    witness_pose = points.copy()
    witness_pose[:, 0] += radial_shift
    witness_pose[:, 1] += best_delta
    slot_axis = np.array((
        math.cos(slot_packing_audit.SLOT_BISECTOR_RAD),
        math.sin(slot_packing_audit.SLOT_BISECTOR_RAD),
    ))
    slot_tangent = np.array((-slot_axis[1], slot_axis[0]))
    witness_world = (
        witness_pose[:, 0, None] * slot_axis
        + witness_pose[:, 1, None] * slot_tangent
    )
    exact_distances = [
        float(exact_core.distance_to(Vertex(float(point[0]), float(point[1]), 0.0)))
        for point in witness_world
    ]
    exact_margin = min(exact_distances) - required_core

    shoe_angle_step = (
        2.0 * 0.36 * slot_packing_audit.SLOT_PITCH_RAD / 1024.0
    )
    shoe_chord_error = (
        DEFAULT_STATOR.od / 2.0
        * (1.0 - math.cos(shoe_angle_step / 2.0))
    )
    global_upper = (
        float(scores[best_index]) + actual_step / 2.0 + shoe_chord_error
    )
    return TransferWitness(
        best_delta_mm=best_delta,
        sampled_best_margin_mm=float(scores[best_index]),
        global_upper_bound_margin_mm=global_upper,
        polygon_core_margin_mm=polygon_core_margin,
        exact_occ_core_margin_mm=float(exact_margin),
        neighbor_margin_mm=neighbor_margin,
        core_witness_wire=core_index,
        neighbor_witness_pair=neighbor_pair,
        delta_search_bounds_mm=(lower, upper),
        delta_step_mm=actual_step,
    )


def _transfer_contract(packing: dict[str, Any],
                       capture: dict[str, Any]) -> dict[str, Any]:
    positive, negative = _packing_arrays(packing)
    config = packing["config"]
    wire_d = float(config["wire_finished_diameter_mm"])
    liner = float(config["liner_thickness_mm"])
    wire_radius = wire_d / 2.0
    required_core = wire_radius + liner

    target_radius = DEFAULT_STATOR.od / 2.0 + required_core
    low, high = 0.0, 20.0
    for _ in range(80):
        middle = (low + high) / 2.0
        minimum_radius = float(np.linalg.norm(
            positive + np.array((middle, 0.0)), axis=1,
        ).min())
        if minimum_radius >= target_radius:
            high = middle
        else:
            low = middle
    outside_shift = high

    graph = _tangent_contact_graph(positive, wire_d)
    wedge = _shear_wedge_bounds(positive, TRANSFER_WITNESS_SHIFT_MM)
    witness = _uniform_transfer_witness(
        positive, negative, wire_d, liner, TRANSFER_WITNESS_SHIFT_MM,
    )

    retract = capture["post_pass_retract"]
    minimum_stroke = float(retract["minimum_stroke_mm"])
    contact_z = float(capture["job"]["wire_contact_z_mm"])
    index_axis_z = PARAMS.stator_axis_z(float(retract["index_target_m0_rad"]))
    load_axis_z = PARAMS.stator_axis_z(0.0)
    index_tip_clearance = index_axis_z - DEFAULT_STATOR.od / 2.0
    load_tip_clearance = load_axis_z - DEFAULT_STATOR.od / 2.0

    former_surface_radius = MINIMUM_WIRE_CENTER_BEND_RADIUS_MM - wire_radius
    endpoint_center_clearance = liner + wire_radius
    required_physical_growth = (
        endpoint_center_clearance
        + MINIMUM_WIRE_CENTER_BEND_RADIUS_MM
        + wire_radius
    )
    current_growth = float(PARAMS.wire_bundle_allow)

    return {
        "idealized_architecture": {
            "description": (
                "Two stator-following former halves support a 50-turn coil "
                "outside the shoe. Post-pass M0 retract strips the coil inward; "
                "one common tangential collapse per half is allowed."
            ),
            "commanded_axes_added": 0,
            "controller_change": False,
            "settings_change": "already-authoritative M0/M1 velocity=20 rad/s only",
            "new_files_only": True,
        },
        "R3_form": {
            "minimum_wire_center_radius_mm": MINIMUM_WIRE_CENTER_BEND_RADIUS_MM,
            "wire_radius_mm": wire_radius,
            "minimum_convex_form_surface_radius_mm": former_surface_radius,
            "minimum_convex_form_surface_diameter_mm": 2.0 * former_surface_radius,
            "minimum_centerline_side_spacing_for_simple_stadium_mm": (
                2.0 * MINIMUM_WIRE_CENTER_BEND_RADIUS_MM
            ),
            "current_final_coil_growth_allowance_each_face_mm": current_growth,
            "required_physical_growth_each_face_mm": required_physical_growth,
            "growth_allowance_shortfall_each_face_mm": (
                required_physical_growth - current_growth
            ),
            "fits_current_final_coil_envelope": (
                required_physical_growth <= current_growth + 1.0e-12
            ),
        },
        "radial_transfer": {
            "minimum_shift_to_clear_shoe_and_liner_mm": outside_shift,
            "minimum_preform_center_radius_mm": target_radius,
            "maximum_preform_center_radius_mm": float(np.linalg.norm(
                positive + np.array((outside_shift, 0.0)), axis=1,
            ).max()),
            "implied_machine_wire_contact_z_mm": contact_z - outside_shift,
            "current_machine_wire_contact_z_mm": contact_z,
            "contact_plane_change_mm": -outside_shift,
            "minimum_available_post_pass_M0_stroke_mm": minimum_stroke,
            "stroke_margin_mm": minimum_stroke - outside_shift,
            "stroke_gate": "PASS" if minimum_stroke >= outside_shift else "FAIL",
            "contact_plane_rigid_flyer_clearance": "NOT_PROVEN",
        },
        "two_half_transfer_witness": {
            "radial_shift_mm": TRANSFER_WITNESS_SHIFT_MM,
            "modeled_active_wire_count": 50,
            "modeled_prefilled_neighbor_wire_count": 50,
            "common_tangential_delta_search_bounds_mm": list(
                witness.delta_search_bounds_mm
            ),
            "delta_step_mm": witness.delta_step_mm,
            "best_common_tangential_delta_mm": witness.best_delta_mm,
            "sampled_best_joint_margin_mm": witness.sampled_best_margin_mm,
            "global_upper_bound_joint_margin_mm": (
                witness.global_upper_bound_margin_mm
            ),
            "polygon_core_margin_at_best_mm": witness.polygon_core_margin_mm,
            "exact_OCC_core_margin_at_best_mm": witness.exact_occ_core_margin_mm,
            "neighbor_margin_at_best_mm": witness.neighbor_margin_mm,
            "core_witness_wire": witness.core_witness_wire,
            "neighbor_witness_pair": list(witness.neighbor_witness_pair),
            "status": (
                "PASS" if witness.global_upper_bound_margin_mm >= 0.0
                else "FAIL"
            ),
            "interpretation": (
                "The complete slot-wedge range for one common two-half "
                "collapse was exhausted. Even an unsampled offset cannot "
                "remove the simultaneous steel/liner and full-neighbor overlap."
            ),
        },
        "affine_taper_bound": {
            **graph,
            "slot_wedge_feasible_shear_range_at_witness": wedge,
            "status": (
                "PASS" if (
                    wedge["large_noncompressing_negative_branch_fits"]
                    or wedge["large_noncompressing_positive_branch_fits"]
                ) else "FAIL"
            ),
            "interpretation": (
                "The exact triangular contact graph jams a radial-linear "
                "taper: nonzero small shear compresses an already tangent wire "
                "pair, while the only noncompressing large-shear branches do "
                "not fit between the two tooth axes. Therefore the admissible "
                "two-half affine motion reduces to the failed common offset."
            ),
        },
        "machine_poses": {
            "winding_axis_z_range_mm": [
                min(PARAMS.stator_axis_z(float(value))
                    for value in capture["m0_wind_range_rad"]),
                max(PARAMS.stator_axis_z(float(value))
                    for value in capture["m0_wind_range_rad"]),
            ],
            "index_and_shaft_wrap_axis_z_mm": index_axis_z,
            "load_axis_z_mm": load_axis_z,
            "bare_stator_tip_clearance_at_index_mm": index_tip_clearance,
            "bare_stator_tip_clearance_at_load_mm": load_tip_clearance,
            "mechanical_M0_axis_z_range_mm": [
                PARAMS.m0_axis_z_min, PARAMS.m0_home_standoff,
            ],
            "post_pass_retract_timing_gate": (
                "PASS" if capture["post_pass_retract"][
                    "all_arrive_before_index"
                ] else "FAIL"
            ),
            "shaft_wrap_done_pose_gate": (
                "PASS" if all(
                    row["m0_at_retracted_index_pose"]
                    for row in capture["shaft_wrap_done_poses"]
                ) else "FAIL"
            ),
            "parked_former_rigid_envelope": "NOT_PROVEN",
        },
    }


def analyze() -> dict[str, Any]:
    packing = _load_json(PACKING_REPORT)
    events = load_events(RAW_CAPTURE)
    capture = _capture_contract(events, packing)
    transfer = _transfer_contract(packing, capture)
    ease = capture["ease_law"]

    gates = {
        "canonical_unmodified_upstream_capture": (
            capture["controller_mode"] == "upstream"
            and capture["adapter_sha256"] is None
        ),
        "exact_24_slot_50_turn_measured_job": True,
        "both_motion_signs_in_raw_capture": (
            capture["motion_sign_counts"] == {-1: 12, 1: 12}
        ),
        "all_neighbor_histories_covered": (
            capture["neighbor_case_counts"] == {0: 2, 1: 20, 2: 2}
        ),
        "settings_only_v20_motion_arrives": (
            capture["post_pass_retract"]["all_arrive_before_index"]
            and all(row["m0_at_retracted_index_pose"]
                    for row in capture["shaft_wrap_done_poses"])
        ),
        "raw_ease_law_constructs_deterministic_50_turn_pack": (
            ease["minimum_full_turn_pitch_mm"]
            >= float(capture["job"]["wire_finished_d_mm"]) - 1.0e-9
            and ease["maximum_full_turn_pitch_mm"]
            <= float(capture["job"]["wire_finished_d_mm"]) + 1.0e-9
        ),
        "M0_stroke_sufficient_for_ideal_transfer": (
            transfer["radial_transfer"]["stroke_gate"] == "PASS"
        ),
        "R3_preform_fits_current_final_coil_envelope": (
            transfer["R3_form"]["fits_current_final_coil_envelope"]
        ),
        "two_half_transfer_clears_steel_liner_and_neighbor": (
            transfer["two_half_transfer_witness"]["status"] == "PASS"
        ),
        "affine_taper_preserves_exact_wire_contact_graph": (
            transfer["affine_taper_bound"]["status"] == "PASS"
        ),
        "flyer_contact_plane_and_parked_former_rigid_envelope": False,
        "continuous_end_turn_handedness_both_signs": False,
        "controlled_50_turn_topology_after_form_release": False,
    }
    release = all(gates.values())
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "PASS" if release else "DESIGN_NO_GO",
        "release_authorized": release,
        "assembly_integration_authorized": release,
        "scope": {
            "architecture": (
                "split/collapsible R3 winding former driven only by existing "
                "post-pass M0 retract"
            ),
            "job": "OD46 x stack15 x 24-slot x 50-turn, 0.22352 mm wire",
            "controller_contract": (
                "canonical raw unmodified upstream stream; settings-only "
                "M0/M1 velocity change allowed"
            ),
            "production_files_modified": False,
            "new_axis_or_serial_command": False,
        },
        "input_hashes": {
            RAW_CAPTURE.relative_to(ROOT).as_posix(): _sha256(RAW_CAPTURE),
            PACKING_REPORT.relative_to(ROOT).as_posix(): _sha256(PACKING_REPORT),
            "cad/params.py": _sha256(CAD / "params.py"),
            "cad/stator_model.py": _sha256(CAD / "stator_model.py"),
            "cad/slot_packing_audit.py": _sha256(CAD / "slot_packing_audit.py"),
            "sim/traj.py": _sha256(HERE / "traj.py"),
            "sim/collapsible_former_study.py": _sha256(Path(__file__)),
        },
        "capture_contract": capture,
        "transfer_study": transfer,
        "gates": gates,
        "decision": {
            "classification": "NO_GO_FOR_TWO_HALF_PASSIVE_FORMER",
            "controlling_failures": [
                (
                    "The raw ease law has 0.000..0.7078 mm turn pitch and "
                    "23..25 of 49 intervals per pass below the 0.22352 mm "
                    "finished wire diameter; a smooth passive former does not "
                    "create the exact four-layer Hamiltonian pack."
                ),
                (
                    "At the 2.0 mm transfer slice, the complete common-collapse "
                    "search has a negative global upper-bound clearance against "
                    "steel/liner plus one fully wound neighboring side."
                ),
                (
                    "The tangent triangular pack prohibits the small affine "
                    "shear a tapered two-half former needs; large noncompressing "
                    "shear branches do not fit the slot wedge."
                ),
                (
                    "A simple R3 stadium exceeds the current final-coil axial "
                    "growth envelope, and the shifted contact plane/parked "
                    "mechanism have no rigid-body certificate."
                ),
            ],
            "what_would_change_the_result": [
                (
                    "A turn-indexed or many-finger insertion tool that controls "
                    "individual wires/rows, with a new raw upstream-compatible "
                    "capture proving its passive mechanical phase law."
                ),
                (
                    "A different electrical/packing job whose complete preformed "
                    "side translates through the occupied slot with positive "
                    "manufacturing-error budget."
                ),
            ],
            "production_CAD_controller_changes": "NONE",
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def render_markdown(report: dict[str, Any]) -> str:
    capture = report["capture_contract"]
    ease = capture["ease_law"]
    transfer = report["transfer_study"]
    witness = transfer["two_half_transfer_witness"]
    retract = transfer["radial_transfer"]
    r3 = transfer["R3_form"]
    return f"""# Split/collapsible winding-former study

**{report['status']} — production integration is not authorized.**

## Motion authority

The study is bound to the canonical unmodified-upstream capture
`{capture['capture_path']}` at `{capture['capture_sha256']}`.  It contains 24
passes, 12 in each flyer sign, and the exact OD46 x stack15 x 24-slot x
50-turn job.  The settings-only M0/M1 20 rad/s change is present; every
post-pass M0 retract and both shaft-wrap arrival poses complete in time.

## What passes

- Available post-pass M0 stroke is {capture['post_pass_retract']['minimum_stroke_mm']:.6f}
  to {capture['post_pass_retract']['maximum_stroke_mm']:.6f} mm.  Moving the
  complete preform outside the lined shoe needs {retract['minimum_shift_to_clear_shoe_and_liner_mm']:.6f}
  mm, leaving {retract['stroke_margin_mm']:.6f} mm geometric stroke margin.
- The raw tooth order covers 2 passes with no wound neighbor, 20 with one,
  and 2 with both neighbors already wound.  The transfer witness includes a
  complete 50-wire neighboring side.
- At index and shaft-wrap M0 is at the retracted -47.124 rad pose; the bare
  stator tip is {transfer['machine_poses']['bare_stator_tip_clearance_at_index_mm']:.3f}
  mm behind the flyer plane.  A parked former body itself is not yet proven.

## Controlling no-go results

1. The raw ease law has {ease['minimum_full_turn_pitch_mm']:.6f} to
   {ease['maximum_full_turn_pitch_mm']:.6f} mm full-turn pitch.  Every pass has
   an exact zero-pitch interval and {ease['minimum_intervals_below_wire_per_pass']} to
   {ease['maximum_intervals_below_wire_per_pass']} of 49 intervals are below
   the {capture['job']['wire_finished_d_mm']:.5f} mm finished wire diameter.
   A smooth passive former therefore does not deterministically make the
   exact four-layer packing topology.
2. At a {witness['radial_shift_mm']:.1f} mm insertion slice, the exhaustive
   common-collapse search gives sampled best margin
   {witness['sampled_best_joint_margin_mm']:.6f} mm and a fail-closed global
   upper bound of {witness['global_upper_bound_joint_margin_mm']:.6f} mm.
   The conflict is simultaneous lined-steel and fully wound-neighbor copper;
   no unsampled two-half offset can pass.
3. The exact tangent contact graph contains opposed +/-60-degree bonds.  A
   radial-linear taper can avoid compressing both only at shear k=0 or
   |k|>=3.464102.  The slot wedge permits only
   {transfer['affine_taper_bound']['slot_wedge_feasible_shear_range_at_witness']['minimum_feasible_k']:.6f} to
   {transfer['affine_taper_bound']['slot_wedge_feasible_shear_range_at_witness']['maximum_feasible_k']:.6f},
   so the large branches do not fit and k=0 reduces to the failed common
   offset.
4. A simple R3 form needs {r3['required_physical_growth_each_face_mm']:.6f}
   mm physical growth beyond each stack face, exceeding the current
   {r3['current_final_coil_growth_allowance_each_face_mm']:.3f} mm envelope by
   {r3['growth_allowance_shortfall_each_face_mm']:.6f} mm.  Moving the preform
   outside the shoe also shifts the contact plane from
   {retract['current_machine_wire_contact_z_mm']:.3f} to
   {retract['implied_machine_wire_contact_z_mm']:.6f} mm; flyer/parked-former
   clearance at that plane is not proven.

## Decision

This is a **no-go for a two-half passive former**, not a claim that all coil
insertion machines are impossible.  Escaping it requires individually
controlled rows/fingers or a different electrical/packing job.  Either is a
new architecture and needs its own unmodified-upstream raw capture, topology
proof, rigid sweep, error budget, and hardware validation.  No production CAD
or controller file was changed.

Report SHA-256: `{report['report_sha256']}`
"""


def write_reports(json_path: Path = OUTPUT_JSON,
                  markdown_path: Path = OUTPUT_MD) -> dict[str, Any]:
    report = analyze()
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    markdown_path.write_text(render_markdown(report), encoding="utf-8")
    return report


def main() -> None:
    report = write_reports()
    witness = report["transfer_study"]["two_half_transfer_witness"]
    print(
        f"collapsible former {report['status']}; "
        f"transfer upper bound {witness['global_upper_bound_joint_margin_mm']:.6f} mm"
    )


if __name__ == "__main__":
    main()
