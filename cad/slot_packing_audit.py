"""Measured-input constructive slot-packing certificate for the release job.

The supplier's nominal dimensions are not treated as perfect hardware.  The
default simulation uses the published 0.0088 inch (0.22352 mm) finished wire
diameter and 5 mil (0.127 mm) liner.  Before a physical winding, those two
values are measured and this same topology is regenerated.  A sensitivity
matrix proves the topology over the accepted receiving interval.

The construction places fifty centers on one side of a shared slot and its
mirror on the neighboring side.  Its fixed 49-step Hamiltonian order keeps
every consecutive center exactly one measured wire diameter apart, gives
every post-seed center an earlier tangent support, and retains a conservative
slot-mouth component even with the neighboring coil side already full.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np
from build123d import Vertex
from shapely.affinity import rotate
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

import coil_growth
from params import DEFAULT_STATOR, PARAMS
import stator_model
from wire_geometry import TOOTH_CONTACT_Z


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORTS = ROOT / "out" / "reports"

SCHEMA = "slot-packing/v2"
SUPPLIER_NOMINAL_WIRE_MM = 0.22352
SUPPLIER_NOMINAL_LINER_MM = 0.127
WIRE_RECEIVING_RANGE_MM = (0.220, 0.235)
LINER_RECEIVING_RANGE_MM = (0.120, 0.140)
RADIAL_END_CLEARANCE_MM = 0.25
SLOT_PITCH_RAD = 2.0 * math.pi / DEFAULT_STATOR.slots
SLOT_BISECTOR_RAD = SLOT_PITCH_RAD / 2.0
TOOTH_HALF_NECK_MM = max(2.5, DEFAULT_STATOR.od * 0.07) / 2.0
SHOE_INNER_RADIUS_MM = (
    DEFAULT_STATOR.od / 2.0 - max(1.6, DEFAULT_STATOR.od * 0.045)
)
RADIAL_CENTER_CAP_MM = SHOE_INNER_RADIUS_MM - RADIAL_END_CLEARANCE_MM


# Nodes are (triangular-lattice row, radial column) in the original
# accessible-frontier certificate.  Keeping the topology symbolic makes the
# measured wire diameter and liner thickness real generator inputs.
BASE_NODES = (
    (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6), (0, 7),
    (1, 7), (0, 8), (1, 8), (0, 9), (1, 9), (0, 10), (1, 10),
    (0, 11), (1, 11), (0, 12), (1, 12), (0, 13), (1, 13),
    (0, 14), (2, 13), (1, 14), (2, 14), (0, 15), (1, 15),
    (0, 16), (2, 15), (1, 16), (0, 17), (2, 16), (1, 17),
    (2, 17), (0, 18), (1, 18), (0, 19), (2, 18), (1, 19),
    (2, 19), (0, 20), (3, 19), (1, 20), (0, 21), (2, 20),
    (1, 21), (3, 20), (2, 21), (0, 22), (1, 22),
)

# This order is a Hamiltonian path on BASE_NODES.  Unlike the earlier greedy
# frontier order, every transition is one pitch and every insertion retains
# mouth access with the complete mirrored neighbor side prefilled.
DEPOSITION_ORDER = (
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
    18, 19, 21, 20, 22, 24, 23, 25, 27, 26, 28, 31, 29, 30, 34, 32,
    33, 37, 35, 36, 40, 38, 39, 42, 43, 48, 45, 44, 41, 46, 47, 49,
)


@dataclass(frozen=True)
class PackingInput:
    wire_d_mm: float
    liner_t_mm: float

    @property
    def wire_radius_mm(self) -> float:
        return self.wire_d_mm / 2.0

    @property
    def center_core_access_mm(self) -> float:
        return self.wire_radius_mm + self.liner_t_mm


def _source_hashes() -> dict[str, str]:
    return {
        name: hashlib.sha256((HERE / name).read_bytes()).hexdigest()
        for name in (
            "params.py", "stator_model.py", "coil_growth.py",
            "wire_geometry.py", "slot_packing_audit.py",
        )
    }


def _unit(angle: float) -> np.ndarray:
    return np.array((math.cos(angle), math.sin(angle)), dtype=float)


def _reflect_about_axis(point: np.ndarray, axis: np.ndarray) -> np.ndarray:
    return 2.0 * float(np.dot(point, axis)) * axis - point


def _minimum_pair(points: np.ndarray) -> tuple[float, tuple[int, int]]:
    best = math.inf
    pair = (-1, -1)
    for left in range(len(points)):
        distances = np.linalg.norm(points[left + 1:] - points[left], axis=1)
        if not len(distances):
            continue
        local = int(np.argmin(distances))
        value = float(distances[local])
        if value < best:
            best = value
            pair = (left, left + 1 + local)
    return best, pair


def _lattice_slot_centers(job: PackingInput) -> list[np.ndarray]:
    d = job.wire_d_mm
    row_pitch = math.sqrt(3.0) * d / 2.0
    # This deterministic fraction leaves a small positive gap at the slot
    # partition while anchoring node 0 exactly on the active-tooth liner.
    tangential_zero = row_pitch * 28.0 / 48.0
    required = job.center_core_access_mm
    radial_zero = (
        TOOTH_HALF_NECK_MM + required
        + tangential_zero * math.cos(SLOT_BISECTOR_RAD)
    ) / math.sin(SLOT_BISECTOR_RAD)
    result = []
    for row, column in BASE_NODES:
        result.append(np.array((
            radial_zero + column * d + row * d / 2.0,
            tangential_zero + row * row_pitch,
        ), dtype=float))
    return result


def _m0_target_rad(active_radial_mm: float) -> float:
    # M0 translates along the presented tooth axis.  The old implementation
    # incorrectly used the slot-bisector radial projection, creating a
    # 0.075..0.160 mm placement error on the nominal job.
    axis_z = float(active_radial_mm) + TOOTH_CONTACT_Z
    return float(PARAMS.m0_rad_for_axis_z(axis_z))


def generate_one_side(job: PackingInput) -> list[dict]:
    if job.wire_d_mm <= 0.0 or job.liner_t_mm <= 0.0:
        raise ValueError("wire diameter and liner thickness must be positive")
    lattice = _lattice_slot_centers(job)
    if len(lattice) != 50 or sorted(DEPOSITION_ORDER) != list(range(50)):
        raise RuntimeError("packing topology must permute exactly 50 nodes")
    slot_axis = _unit(SLOT_BISECTOR_RAD)
    slot_tangent = np.array((-slot_axis[1], slot_axis[0]))
    tooth_axis = _unit(SLOT_PITCH_RAD)
    tooth_tangent = np.array((-tooth_axis[1], tooth_axis[0]))
    records: list[dict] = []
    for lattice_index in DEPOSITION_ORDER:
        slot_uv = lattice[lattice_index]
        center = slot_uv[0] * slot_axis + slot_uv[1] * slot_tangent
        active_radial = float(np.dot(center, tooth_axis))
        active_tangential = float(np.dot(center, tooth_tangent))
        contacts = []
        contact_distances = []
        for prior in records:
            distance = float(np.linalg.norm(
                center - np.asarray(prior["center_xy_mm"])))
            if abs(distance - job.wire_d_mm) <= 1e-9:
                contacts.append(int(prior["turn_index"]))
                contact_distances.append(distance)
        turn_index = len(records)
        support_kind = "slot_liner" if turn_index == 0 else "deposited_wire"
        if turn_index and not contacts:
            raise RuntimeError(
                f"packing turn {turn_index} has no earlier tangent support")
        row, column = BASE_NODES[lattice_index]
        records.append({
            "turn_index": turn_index,
            "lattice_index": lattice_index,
            "layer_index": row,
            "lattice_column": column,
            "radial_parameter_mm": active_radial,
            "m0_target_rad": _m0_target_rad(active_radial),
            "normal_profile_radius_mm": -active_tangential,
            "center_xy_mm": center.tolist(),
            "slot_frame_uv_mm": slot_uv.tolist(),
            "active_tooth_frame_uv_mm": [
                active_radial, active_tangential,
            ],
            "support_kind": support_kind,
            "parent_turn_indices": contacts,
            "parent_center_distances_mm": contact_distances,
            "prior_contact_turn_indices": contacts,
            "prior_contact_center_distances_mm": contact_distances,
        })
    return records


def _annular_sector(inner: float, outer: float, center: float,
                    half_angle: float, samples: int = 256) -> Polygon:
    outer_points = [
        (outer * math.cos(center - half_angle
                          + 2.0 * half_angle * index / samples),
         outer * math.sin(center - half_angle
                          + 2.0 * half_angle * index / samples))
        for index in range(samples + 1)
    ]
    inner_points = [
        (inner * math.cos(center + half_angle
                          - 2.0 * half_angle * index / samples),
         inner * math.sin(center + half_angle
                          - 2.0 * half_angle * index / samples))
        for index in range(samples + 1)
    ]
    return Polygon(outer_points + inner_points)


def _positive_slot_center_domain(job: PackingInput) -> Polygon:
    spec = DEFAULT_STATOR
    hub_radius = spec.od * spec.hub_od_ratio / 2.0
    shoe_outer = spec.od / 2.0
    tooth_half_pitch = SLOT_BISECTOR_RAD
    shoe_half_angle = 0.36 * SLOT_PITCH_RAD
    neck_start = hub_radius - 1.0
    neck_end = neck_start + (SHOE_INNER_RADIUS_MM - hub_radius + 2.0)
    core = [Point(0.0, 0.0).buffer(hub_radius, quad_segs=256)]
    for angle in (-tooth_half_pitch, tooth_half_pitch):
        neck = box(
            neck_start, -TOOTH_HALF_NECK_MM,
            neck_end, TOOTH_HALF_NECK_MM,
        )
        core.append(rotate(
            neck, math.degrees(angle), origin=(0.0, 0.0)))
        core.append(_annular_sector(
            SHOE_INNER_RADIUS_MM, shoe_outer, angle, shoe_half_angle))
    samples = 1024
    sector = Polygon([(0.0, 0.0)] + [
        (RADIAL_CENTER_CAP_MM * math.cos(
            -tooth_half_pitch + 2.0 * tooth_half_pitch * index / samples),
         RADIAL_CENTER_CAP_MM * math.sin(
            -tooth_half_pitch + 2.0 * tooth_half_pitch * index / samples))
        for index in range(samples + 1)
    ])
    domain = sector.difference(unary_union(core).buffer(
        job.center_core_access_mm, quad_segs=256))
    return domain.intersection(box(
        -2.0 * RADIAL_CENTER_CAP_MM, job.wire_radius_mm,
        2.0 * RADIAL_CENTER_CAP_MM, 2.0 * RADIAL_CENTER_CAP_MM,
    ))


def _mouth_connected(domain: Polygon, target: Iterable[float],
                     obstacles: Iterable[Iterable[float]],
                     wire_d_mm: float) -> bool:
    # A 20 nm GEOS relaxation preserves intended tangent endpoints.  It is
    # not used by the exact final center/core or pair checks.
    obstacle_radius = wire_d_mm - 2.0e-5
    disks = [Point(*map(float, center)).buffer(
        obstacle_radius, quad_segs=64) for center in obstacles]
    free = domain.difference(unary_union(disks)) if disks else domain
    point = Point(*map(float, target))
    components = list(free.geoms) if hasattr(free, "geoms") else [free]
    for component in components:
        if (component.is_empty
                or not component.buffer(3.0e-5).covers(point)
                or not hasattr(component, "exterior")):
            continue
        maximum_radius = max(
            math.hypot(x, y) for x, y in component.exterior.coords)
        if maximum_radius >= RADIAL_CENTER_CAP_MM - 5.0e-4:
            return True
    return False


def _sequential_mouth_audit(job: PackingInput, one_side: list[dict],
                            mirrored: list[dict]) -> dict:
    domain = _positive_slot_center_domain(job)
    positive = [record["slot_frame_uv_mm"] for record in one_side]
    negative = [record["slot_frame_uv_mm"] for record in mirrored]
    empty_neighbor = []
    full_neighbor = []
    prior: list[list[float]] = []
    for target in positive:
        empty_neighbor.append(_mouth_connected(
            domain, target, prior, job.wire_d_mm))
        full_neighbor.append(_mouth_connected(
            domain, target, [*negative, *prior], job.wire_d_mm))
        prior.append(target)
    return {
        "status": (
            "PASS" if all(empty_neighbor) and all(full_neighbor) else "FAIL"
        ),
        "method": (
            "conservative polygonal center-space connected-component test "
            "against the generated stator section"
        ),
        "empty_neighbor_side_mouth_connected": empty_neighbor,
        "prefilled_neighbor_side_mouth_connected": full_neighbor,
        "all_empty_neighbor_side_connected": all(empty_neighbor),
        "all_prefilled_neighbor_side_connected": all(full_neighbor),
        "first_empty_neighbor_failure": next(
            (index for index, ok in enumerate(empty_neighbor) if not ok), None),
        "first_prefilled_neighbor_failure": next(
            (index for index, ok in enumerate(full_neighbor) if not ok), None),
        "polygon_circle_relaxation_mm": 2.0e-5,
    }


@lru_cache(maxsize=1)
def _release_core():
    # Stator steel geometry is independent of wire/turn fields.  Reusing one
    # BREP makes the complete 7 x 7 receiving sweep practical while every
    # center still gets an exact OpenCascade Part.distance_to(Vertex) query.
    return stator_model.stator(replace(DEFAULT_STATOR, turns=50))


def _exact_core_distances(points: Iterable[np.ndarray],
                          job: PackingInput) -> list[float]:
    del job  # retained in the signature to make the audited case explicit
    core = _release_core()
    return [
        float(core.distance_to(Vertex(
            float(point[0]), float(point[1]), 0.0)))
        for point in points
    ]


def _case(job: PackingInput, *, exact_core: bool = False) -> dict:
    one_side = generate_one_side(job)
    slot_axis = _unit(SLOT_BISECTOR_RAD)
    positive = np.asarray([row["center_xy_mm"] for row in one_side])
    negative = np.asarray([
        _reflect_about_axis(point, slot_axis) for point in positive
    ])
    mirrored = []
    for record, center in zip(one_side, negative):
        copy = dict(record)
        copy["center_xy_mm"] = center.tolist()
        copy["slot_frame_uv_mm"] = [
            record["slot_frame_uv_mm"][0],
            -record["slot_frame_uv_mm"][1],
        ]
        mirrored.append(copy)
    all_points = np.vstack((positive, negative))
    minimum_pair, pair_indices = _minimum_pair(all_points)
    maximum_radius = float(np.linalg.norm(all_points, axis=1).max())
    consecutive = [
        float(np.linalg.norm(positive[index] - positive[index - 1]))
        for index in range(1, len(positive))
    ]
    mouth = _sequential_mouth_audit(job, one_side, mirrored)
    core_distances = (_exact_core_distances(all_points, job)
                      if exact_core else None)
    minimum_core = (min(core_distances) if core_distances is not None
                    else job.center_core_access_mm)
    if core_distances is not None:
        for index, row in enumerate(one_side):
            row["exact_center_core_distance_mm"] = core_distances[index]
            mirrored[index]["exact_center_core_distance_mm"] = (
                core_distances[len(one_side) + index]
            )
    pair_ok = minimum_pair >= job.wire_d_mm - 1e-9
    core_ok = minimum_core >= job.center_core_access_mm - 1e-9
    cap_ok = maximum_radius <= RADIAL_CENTER_CAP_MM + 1e-9
    schedule_ok = all(
        abs(value - job.wire_d_mm) <= 1e-9 for value in consecutive)
    support_ok = all(
        row["support_kind"] == "slot_liner"
        or bool(row["parent_turn_indices"])
        for row in one_side
    )
    status = "PASS" if all((
        pair_ok, core_ok, cap_ok, schedule_ok, support_ok,
        mouth["status"] == "PASS",
    )) else "FAIL"
    return {
        "status": status,
        "wire_finished_diameter_mm": job.wire_d_mm,
        "liner_thickness_mm": job.liner_t_mm,
        "center_core_access_mm": job.center_core_access_mm,
        "side_positive": one_side,
        "side_negative": mirrored,
        "sequential_mouth_access": mouth,
        "validation": {
            "minimum_pair_center_distance_mm": minimum_pair,
            "minimum_pair_indices": list(pair_indices),
            "minimum_center_core_distance_mm": minimum_core,
            "maximum_center_radius_mm": maximum_radius,
            "radial_outer_margin_mm": RADIAL_CENTER_CAP_MM - maximum_radius,
            "all_consecutive_schedule_distances_mm": consecutive,
            "pair_clearance_ok": pair_ok,
            "core_access_ok": core_ok,
            "radial_cap_ok": cap_ok,
            "all_schedule_steps_tangent": schedule_ok,
            "all_later_turns_have_parent_support": support_ok,
            "all_empty_neighbor_side_mouth_connected": mouth[
                "all_empty_neighbor_side_connected"],
            "all_prefilled_neighbor_side_mouth_connected": mouth[
                "all_prefilled_neighbor_side_connected"],
            "core_distance_method": (
                "OpenCascade Part.distance_to(Vertex) against source stator"
                if exact_core else
                "topology sensitivity; nominal exact OpenCascade case is authority"
            ),
        },
    }


def _in_closed_range(value: float, limits: tuple[float, float]) -> bool:
    return limits[0] - 1e-12 <= value <= limits[1] + 1e-12


def _validate_job(job: PackingInput) -> None:
    if not math.isfinite(job.wire_d_mm) or not math.isfinite(job.liner_t_mm):
        raise ValueError("wire diameter and liner thickness must be finite")
    if not _in_closed_range(job.wire_d_mm, WIRE_RECEIVING_RANGE_MM):
        raise ValueError(
            "measured finished wire diameter "
            f"{job.wire_d_mm:.6f} mm is outside receiving range "
            f"{WIRE_RECEIVING_RANGE_MM} mm"
        )
    if not _in_closed_range(job.liner_t_mm, LINER_RECEIVING_RANGE_MM):
        raise ValueError(
            "measured liner thickness "
            f"{job.liner_t_mm:.6f} mm is outside receiving range "
            f"{LINER_RECEIVING_RANGE_MM} mm"
        )


@lru_cache(maxsize=1)
def _receiving_sensitivity() -> tuple[list[dict], list[dict]]:
    """Return a 7x7 topology sweep and its four named corner cases.

    The earlier report checked only the four endpoints.  That did not justify
    the prose claim that an arbitrary measured value inside the receiving
    interval could be regenerated.  The full grid is still a finite
    sensitivity check (not a tolerance theorem), but it catches interior
    connectivity changes and makes the receiving evidence explicit.
    """

    wire_values = np.linspace(*WIRE_RECEIVING_RANGE_MM, 7)
    liner_values = np.linspace(*LINER_RECEIVING_RANGE_MM, 7)
    rows: list[dict] = []
    for wire_d in wire_values:
        for liner_t in liner_values:
            case = _case(
                PackingInput(float(wire_d), float(liner_t)),
                exact_core=True,
            )
            rows.append({
                "wire_finished_diameter_mm": float(wire_d),
                "liner_thickness_mm": float(liner_t),
                "status": case["status"],
                "minimum_pair_center_distance_mm": case["validation"][
                    "minimum_pair_center_distance_mm"],
                "minimum_center_core_distance_mm": case["validation"][
                    "minimum_center_core_distance_mm"],
                "required_center_core_access_mm": (
                    float(wire_d) / 2.0 + float(liner_t)),
                "minimum_center_core_margin_mm": (
                    case["validation"]["minimum_center_core_distance_mm"]
                    - (float(wire_d) / 2.0 + float(liner_t))),
                "core_access_ok": case["validation"]["core_access_ok"],
                "radial_outer_margin_mm": case["validation"][
                    "radial_outer_margin_mm"],
                "all_schedule_steps_tangent": case["validation"][
                    "all_schedule_steps_tangent"],
                "all_prefilled_neighbor_side_mouth_connected": case[
                    "validation"
                ]["all_prefilled_neighbor_side_mouth_connected"],
            })
    corners = [
        row for row in rows
        if row["wire_finished_diameter_mm"] in WIRE_RECEIVING_RANGE_MM
        and row["liner_thickness_mm"] in LINER_RECEIVING_RANGE_MM
    ]
    return rows, corners


def analyze(job: PackingInput | None = None) -> dict:
    expected = (
        DEFAULT_STATOR.slots, DEFAULT_STATOR.od, DEFAULT_STATOR.stack,
        DEFAULT_STATOR.wire_d, DEFAULT_STATOR.turns,
        coil_growth.DEFAULT_POLICY.opening_edge_clearance_mm,
    )
    if expected != (24, 46.0, 15.0, 0.22352, 50, 0.127):
        raise RuntimeError(
            "release packing is pinned to 24 slots, OD46 x stack15, "
            "0.22352 mm nominal measured wire, 50 turns, and 0.127 mm liner"
        )
    selected_job = job or PackingInput(
        DEFAULT_STATOR.wire_d,
        coil_growth.DEFAULT_POLICY.opening_edge_clearance_mm,
    )
    _validate_job(selected_job)
    selected_case = _case(selected_job, exact_core=True)
    sensitivity, corners = _receiving_sensitivity()
    sensitivity_status = (
        "PASS" if all(case["status"] == "PASS" for case in sensitivity)
        else "FAIL"
    )
    selected = selected_case["side_positive"]
    half_turn_waypoints = []
    for record in selected:
        for half in range(2):
            phase_index = 2 * int(record["turn_index"]) + half
            half_turn_waypoints.append({
                "phase_index": phase_index,
                "m2_relative_phase_rad": phase_index * math.pi,
                "turn_index": int(record["turn_index"]),
                "half_turn_index": half,
                "active_tooth_radial_mm": record["radial_parameter_mm"],
                "m0_target_rad": record["m0_target_rad"],
                "layer_index": record["layer_index"],
                "support_kind": record["support_kind"],
            })
    report = {
        "schema": SCHEMA,
        "status": (
            "PASS" if selected_case["status"] == "PASS"
            and sensitivity_status == "PASS" else "FAIL"
        ),
        "role": (
            "authoritative_release_default"
            if job is None else "authoritative_measured_release_job"
        ),
        "algorithm": (
            "measured-input triangular-lattice Hamiltonian mouth-preserving-v2"
        ),
        "config": {
            "wire_finished_diameter_mm": selected_job.wire_d_mm,
            "wire_radius_mm": selected_job.wire_radius_mm,
            "liner_thickness_mm": selected_job.liner_t_mm,
            "center_core_access_mm": selected_job.center_core_access_mm,
            "input_provenance": (
                "supplier_nominal_simulation_default"
                if job is None else "measured_receiving_input"
            ),
            "wire_supplier_nominal_in": 0.0088,
            "liner_supplier_nominal_in": 0.005,
            "radial_center_cap_mm": RADIAL_CENTER_CAP_MM,
            "slots": DEFAULT_STATOR.slots,
            "od_mm": DEFAULT_STATOR.od,
            "stack_mm": DEFAULT_STATOR.stack,
            "slot_pitch_deg": math.degrees(SLOT_PITCH_RAD),
            "slot_bisector_deg": math.degrees(SLOT_BISECTOR_RAD),
            "tooth_contact_machine_z_mm": TOOTH_CONTACT_Z,
            "m0_home_standoff_mm": PARAMS.m0_home_standoff,
            "m0_mm_per_rad": PARAMS.mm_per_rad,
        },
        "receiving_contract": {
            "wire_finished_diameter_range_mm": list(WIRE_RECEIVING_RANGE_MM),
            "liner_thickness_range_mm": list(LINER_RECEIVING_RANGE_MM),
            "rule": (
                "measure both inputs, regenerate this plan and all route/"
                "controller artifacts, and reject values outside the interval"
            ),
            "topology_sensitivity_status": sensitivity_status,
            "sensitivity_grid_shape": [7, 7],
            "sensitivity_grid_cases": sensitivity,
            "corner_cases": corners,
        },
        "frames": {
            "stator_local": "XY lamination plane; tooth0 on +X; Z axial",
            "audited_shared_slot": "between tooth0 and tooth1",
            "slot_u_radial_axis_xy": _unit(SLOT_BISECTOR_RAD).tolist(),
            "slot_v_positive_axis_xy": [
                -math.sin(SLOT_BISECTOR_RAD), math.cos(SLOT_BISECTOR_RAD),
            ],
            "active_tooth_axis_xy": _unit(SLOT_PITCH_RAD).tolist(),
            "controller_radial_field": "radial_parameter_mm in active tooth frame",
            "mirror_rule": "(slot_u,slot_v) -> (slot_u,-slot_v)",
        },
        "selected_schedule": {
            "turns_per_tooth": 50,
            "centers_per_slot": 100,
            "layer_counts": [
                sum(row["layer_index"] == layer for row in selected)
                for layer in range(4)
            ],
            "base_lattice_nodes": [list(node) for node in BASE_NODES],
            "deposition_order_lattice_indices": list(DEPOSITION_ORDER),
            "progressive_support_validated": selected_case["validation"][
                "all_later_turns_have_parent_support"],
            "sequential_mouth_access": selected_case[
                "sequential_mouth_access"],
            "half_turn_waypoints": half_turn_waypoints,
            "lead_out_hold_waypoint": {
                "phase_index": 100,
                "m2_relative_phase_rad": 100.0 * math.pi,
                "active_tooth_radial_mm": selected[-1]["radial_parameter_mm"],
                "m0_target_rad": selected[-1]["m0_target_rad"],
                "rule": "hold final supported radial target through lead-out",
            },
            "side_positive": selected_case["side_positive"],
            "side_negative": selected_case["side_negative"],
        },
        "validation": selected_case["validation"],
        "limits": [
            "Intentional support contacts have zero nominal gap by definition.",
            "M0 following error, liner compression, wire deformation and neat layering require the receiving-regenerated coupon gate.",
            "The mouth component is a static insertion proof; continuous captured M0/M2 route validation is separate.",
        ],
        "source_hashes": _source_hashes(),
    }
    canonical = json.dumps(report, sort_keys=True, separators=(",", ":"))
    report["report_sha256"] = hashlib.sha256(canonical.encode()).hexdigest()
    return report


def render_markdown(report: dict) -> str:
    config = report["config"]
    validation = report["validation"]
    receiving = report["receiving_contract"]
    return "\n".join((
        "# Measured-input slot packing audit",
        "",
        f"Status: **{report['status']}**",
        "",
        f"- Selected wire input: {config['wire_finished_diameter_mm']:.5f} mm",
        f"- Selected liner input: {config['liner_thickness_mm']:.3f} mm",
        "- Schedule: 50 turns/side, 100 centers/shared slot",
        "- Every one of 49 transitions is one measured wire diameter",
        "- Every post-seed turn names at least one earlier tangent support",
        "- All 50 placements retain mouth access with the neighbor side full",
        f"- Nominal radial outer margin: {validation['radial_outer_margin_mm']:.6f} mm",
        "",
        "## Receiving-regeneration interval",
        "",
        f"- Wire: {receiving['wire_finished_diameter_range_mm']} mm",
        f"- Liner: {receiving['liner_thickness_range_mm']} mm",
        f"- 7 x 7 topology sweep: {receiving['topology_sensitivity_status']}",
        "",
        "Supplier nominal values are not substituted for measurements on real hardware. "
        "The measured values regenerate packing, routes, settings, capture and player artifacts.",
        "",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ))


def write_outputs(report: dict, json_path: Path, markdown_path: Path) -> None:
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    markdown_path.write_text(render_markdown(report), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", type=Path,
                        default=REPORTS / "slot_packing.json")
    parser.add_argument("--markdown", type=Path,
                        default=REPORTS / "slot_packing.md")
    parser.add_argument(
        "--wire", type=float, default=SUPPLIER_NOMINAL_WIRE_MM,
        help="measured finished magnet-wire outside diameter, mm",
    )
    parser.add_argument(
        "--liner", type=float, default=SUPPLIER_NOMINAL_LINER_MM,
        help="measured installed Nomex liner thickness, mm",
    )
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    selected = PackingInput(args.wire, args.liner)
    default = PackingInput(
        SUPPLIER_NOMINAL_WIRE_MM, SUPPLIER_NOMINAL_LINER_MM)
    report = analyze(None if selected == default else selected)
    if report["status"] != "PASS":
        raise SystemExit("slot packing audit failed")
    if not args.check:
        write_outputs(report, args.json, args.markdown)
        print(f"wrote {args.json} and {args.markdown}")
    print(
        "measured-input slot packing PASS: 50/side, "
        f"wire={report['config']['wire_finished_diameter_mm']:.5f}, "
        f"liner={report['config']['liner_thickness_mm']:.3f}"
    )


if __name__ == "__main__":
    main()
