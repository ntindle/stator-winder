"""Exact source-level audit of the winder's M2/M3 retention hardware.

This module is intentionally read-only with respect to the machine CAD.  It
builds the current :mod:`printed`, :mod:`hardware_placements`, and :mod:`cots`
BREPs, performs OpenCascade Boolean intersections, and also builds candidate
repair geometry so a shared-source patch can be specified numerically before
it is applied.

Coordinate convention: machine millimetres; X horizontal, Y up, Z along the
flyer axis.  The nominal dancer arm is at 0 degrees and its audited hard-stop
range is ``PARAMS.dancer_stop_offsets_deg``.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Iterable, Sequence

from build123d import Align, Cone, Cylinder, Part, Pos, Rot
from bd_warehouse.fastener import CounterSunkScrew

import cots
import hardware
import hardware_placements
import integrated_release_candidate as release_candidate
from params import PARAMS as P
import printed
import wire_vis


CTR = (Align.CENTER, Align.CENTER, Align.CENTER)
MIN = (Align.CENTER, Align.CENTER, Align.MIN)
BOOLEAN_TOL_MM3 = 1.0e-5

SERIALIZED_REAR_STACK_IDS = (
    "rear_left",
    "rear_right",
    "front_left",
    "front_right",
)
SERIALIZED_REAR_STACK_SUFFIXES = (
    "tungsten_slug",
    "printed_retainer_with_three_spacers",
    "McMaster_94459A130_insert",
    "McMaster_92125A126_M3x6_screw",
)
SERIALIZED_REAR_OCCURRENCE_LABELS = tuple(
    f"{stack_id}_{suffix}"
    for stack_id in SERIALIZED_REAR_STACK_IDS
    for suffix in SERIALIZED_REAR_STACK_SUFFIXES
)
SERIALIZED_FRONT_OCCURRENCE_LABELS = (
    "front_trim_B777_-3.6",
    "front_trim_B777_+3.6",
    *(f"front_trim_hardware_{index}" for index in range(1, 7)),
)
REPRESENTATIVE_REAR_STACK_ID = "rear_right"


@dataclass(frozen=True)
class Check:
    """One deterministic geometry result."""

    name: str
    passed: bool
    measured: float
    limit: float
    units: str
    relation: str
    note: str = ""


def intersection_volume(a: Part, b: Part) -> float:
    """Return exact OpenCascade common volume in mm^3."""
    common = a & b
    return float(common.volume) if common is not None else 0.0


def distance(a: Part, b: Part) -> float:
    """Return exact minimum BREP distance in mm."""
    return float(a.distance_to(b))


def _bbox(shape: Part) -> dict[str, list[float]]:
    box = shape.bounding_box()
    return {
        "min": [float(box.min.X), float(box.min.Y), float(box.min.Z)],
        "max": [float(box.max.X), float(box.max.Y), float(box.max.Z)],
        "size": [float(box.size.X), float(box.size.Y), float(box.size.Z)],
    }


def _rotate_about_dancer_pivot(shape: Part, angle_deg: float) -> Part:
    """Rotate a machine-coordinate shape about the dancer's Z pivot."""
    px, py = P.rear_post_x, P.dancer_y
    return (Pos(px, py, 0.0) * Rot(0.0, 0.0, angle_deg) *
            Pos(-px, -py, 0.0)) * shape


def _static_occurrences() -> dict[str, hardware_placements.HardwareOccurrence]:
    return {o.label: o for o in hardware_placements.static_occurrences(P)}


def _flyer_occurrences() -> dict[str, hardware_placements.HardwareOccurrence]:
    return {o.label: o for o in hardware_placements.flyer_occurrences(P)}


def _build(occurrences, labels: Iterable[str]) -> dict[str, Part]:
    return {label: occurrences[label].build() for label in labels}


def m2_mount_access_relief(mount: Part | None = None) -> Part:
    """Candidate low-fastener tool/head tunnels for the M2 join blocks.

    The M5 socket-head bearing plane is z=-66.  The existing low join blocks
    continue rearward to z=-102, burying the heads.  Two OD10 tunnels open
    from the rear and terminate at that bearing plane; all mounting axes and
    the 6 mm post web remain unchanged.
    """
    result = printed.m2_motor_mount() if mount is None else mount
    for x in (-P.post_x, P.post_x):
        cutter = Pos(x, P.m2_motor_axis_y - 12.0, -103.0) * Cylinder(
            5.0, 37.0, align=MIN)
        result = result - cutter
    result.label = "m2_motor_mount_access_relief_candidate"
    return result


def _countersink_cut(x: float, y: float, surface_z: float = -164.0) -> Part:
    """OD10/OD5.4, 90 degree M5 countersink opening at ``surface_z``."""
    depth = (10.0 - 5.4) / 2.0
    return Pos(x, y, surface_z - depth) * Cone(
        2.7, 5.0, depth, align=MIN)


def rear_post_base_with_countersinks(part: Part,
                                     centers: Sequence[tuple[float, float]],
                                     surface_z: float = -164.0) -> Part:
    """Candidate flush M5 seats without moving any rear-post base axis."""
    result = part
    for x, y in centers:
        result = result - _countersink_cut(x, y, surface_z)
    result.label = f"{getattr(part, 'label', 'base')}_flush_m5_candidate"
    return result


def entry_bracket_with_pulley_notch(part: Part | None = None) -> Part:
    """Candidate lower-right relief for the pulley rear retention stack.

    The notch is wholly outside the x=-45 rear-post slot axis and the fixed
    spring-anchor bridge.  It removes the corner swept by the pulley nyloc
    and the otherwise harmless excess-shoulder shims without changing either
    hard stop or the wire passage.
    """
    from build123d import Box
    result = printed.entry_bracket() if part is None else part
    cutter = Pos(-32.75, -14.5, -167.25) * Box(
        11.5, 15.0, 7.5, align=CTR)
    result = result - cutter
    result.label = "entry_bracket_pulley_notch_candidate"
    return result


def dancer_arm_with_flush_moving_anchor(part: Part | None = None) -> Part:
    """Candidate 90 degree M2 countersink on the moving anchor's rear."""
    result = printed.dancer_arm() if part is None else part
    dx = P.dancer_pulley_x - P.rear_post_x
    dy = P.dancer_pulley_y - P.dancer_y
    arm_length = math.hypot(dx, dy)
    x = P.rear_post_x + P.dancer_spring_moving_r * dx / arm_length
    y = P.dancer_y + P.dancer_spring_moving_r * dy / arm_length
    # Rear face z=-163; OD4.8/OD2.4 90 degree printable-clearance seat,
    # 1.2 mm deep.  The extra 0.5 mm over the ISO 14581 nominal head OD
    # clears the catalog head-edge fillet while retaining 1.3 mm arm floor.
    result = result - Pos(x, y, -163.0) * Cone(2.4, 1.2, 1.2, align=MIN)
    result.label = "dancer_arm_flush_m2_anchor_candidate"
    return result


def moving_anchor_m2x16_countersunk() -> Part:
    """ISO 14581 M2x16, top face flush with dancer-arm rear z=-163."""
    dx = P.dancer_pulley_x - P.rear_post_x
    dy = P.dancer_pulley_y - P.dancer_y
    arm_length = math.hypot(dx, dy)
    x = P.rear_post_x + P.dancer_spring_moving_r * dx / arm_length
    y = P.dancer_y + P.dancer_spring_moving_r * dy / arm_length
    screw = CounterSunkScrew(
        "M2-0.4", 16.0, fastener_type="iso14581", simple=True)
    screw.label = "dancer_spring_moving_iso14581_m2x16"
    return hardware.place(screw, (x, y, -163.0), axis="-z",
                          label=screw.label)


def _m5x12_countersunk(label: str, x: float, y: float,
                       surface_z: float = -164.0) -> Part:
    screw = CounterSunkScrew(
        "M5-0.8", 12.0, fastener_type="iso10642", simple=True)
    screw.label = label
    return hardware.place(screw, (x, y, surface_z), axis="+z", label=label)


def flyer_block_with_od26_running_bore(block: Part | None = None) -> Part:
    """Candidate Ø26 through-running bore around pulley clamp hardware."""
    result = printed.flyer_block() if block is None else block
    result = result - Pos(0.0, 0.0, -81.0) * Cylinder(13.0, 52.0, align=MIN)
    result.label = "flyer_block_od26_running_bore_candidate"
    return result


def counterweight_attachment_components(
    parts: dict[str, Part] | None = None,
) -> dict[str, Part]:
    """Return one complete current rear M3 stack and its printed arm.

    The obsolete single central screw/three-washer stack no longer exists in
    ``hardware_placements``.  This representative stack is selected by the
    same serialized occurrence labels used by the integrated candidate; its
    exact arm floor, slug, retainer/spacers, insert, and screw preserve the
    original audit's closed-load-path intent without generic aliases.
    """

    current = _current_target_shapes() if parts is None else parts
    stack_id = REPRESENTATIVE_REAR_STACK_ID
    return {
        "arm": current["retained_arm"],
        "slug": current[f"{stack_id}_tungsten_slug"],
        "retainer": current[
            f"{stack_id}_printed_retainer_with_three_spacers"
        ],
        "insert": current[f"{stack_id}_McMaster_94459A130_insert"],
        "screw": current[
            f"{stack_id}_McMaster_92125A126_M3x6_screw"
        ],
    }


def m2_outer_spacer_id22() -> Part:
    """Candidate outer-race spacer retaining OD27.8/11 length, with ID22."""
    part = Pos(0.0, 0.0, -61.5) * cots.tube_spacer(27.8, 22.0, 11.0)
    part.label = "m2_outer_race_spacer_id22_candidate"
    return part


def _current_target_shapes() -> dict[str, Part]:
    static = _static_occurrences()
    flyer = _flyer_occurrences()
    labels_static = [
        label for label in static
        if (label.startswith("m2_mount_") or
            label.startswith("dancer_") or
            label.startswith("entry_base_") or
            label.startswith("felt_") or
            label.startswith("spool_"))
    ]
    labels_flyer = list(flyer)
    parts = _build(static, labels_static)
    parts.update(_build(flyer, labels_flyer))
    parts.update({
        "m2_motor_mount": printed.m2_motor_mount(),
        "flyer_block": printed.flyer_block(),
        "flyer_arm": printed.flyer_arm(),
        "flyer_pulley": printed.flyer_pulley(),
        "dancer_base": printed.dancer_base(),
        "dancer_arm": printed.dancer_arm(),
        "dancer_pulley": printed.dancer_pulley(),
        "entry_bracket": printed.entry_bracket(),
        "felt_tensioner": printed.felt_tensioner(),
        "spool_bracket": printed.spool_bracket(),
        "spool_drum": printed.spool_drum(),
        "m2_inner_center_spacer": (
            Pos(0.0, 0.0, -61.5) * cots.tube_spacer(18.0, 12.05, 11.0)),
        "m2_outer_race_spacer": (
            Pos(0.0, 0.0, -61.5) * cots.tube_spacer(27.8, 20.0, 11.0)),
    })
    rotating = release_candidate.retained_rotating_parts()
    serialized_labels = (
        *SERIALIZED_REAR_OCCURRENCE_LABELS,
        *SERIALIZED_FRONT_OCCURRENCE_LABELS,
    )
    missing = [label for label in serialized_labels if label not in rotating]
    if missing:
        raise KeyError(
            "M2/M3 audit serialized counterweight contract drift: "
            + ", ".join(missing)
        )
    parts["retained_arm"] = rotating["retained_arm"]
    parts["flyer_pulley"] = rotating["flyer_pulley"]
    parts.update({label: rotating[label] for label in serialized_labels})
    return parts


def run_audit() -> dict:
    """Run the exact targeted audit and return JSON-serializable evidence."""
    p = _current_target_shapes()
    checks: list[Check] = []
    evidence: dict[str, object] = {}

    # ------------------------------------------ flyer counterweight structure
    attachment = release_candidate.integrated_six_stack_attachment_audit()
    rear = attachment["rear_M3_retained_stacks"]
    rear_rows = rear["stacks"]
    front_rows = attachment["front_M2_blind_spoke_stacks"]
    rear_count = sum(
        all(
            f"{stack_id}_{suffix}" in p
            for suffix in SERIALIZED_REAR_STACK_SUFFIXES
        )
        for stack_id in SERIALIZED_REAR_STACK_IDS
    )
    front_count = (
        2
        if all(label in p for label in SERIALIZED_FRONT_OCCURRENCE_LABELS)
        else 0
    )
    minimum_rear_insert_engagement = min(
        float(row["full_insert_engagement_mm"]) for row in rear_rows
    )
    minimum_rear_blind_material = min(
        float(row["blind_positive_material_ahead_of_tip_mm"])
        for row in rear_rows
    )
    minimum_front_insert_engagement = min(
        float(row["full_insert_engagement_mm"]) for row in front_rows
    )
    minimum_front_tip_clearance = min(
        float(row["screw_tip_clearance_behind_insert_mm"])
        for row in front_rows
    )
    minimum_front_blind_material = min(
        float(row["blind_printed_material_behind_pilot_mm"])
        for row in front_rows
    )
    evidence["counterweight_attachment"] = {
        "flyer_arm_solids": len(p["retained_arm"].solids()),
        "serialized_occurrence_count": (
            len(SERIALIZED_REAR_OCCURRENCE_LABELS)
            + len(SERIALIZED_FRONT_OCCURRENCE_LABELS)
        ),
        "rear_M3_stack_count": rear_count,
        "front_M2_stack_count": front_count,
        "rear_M3_occurrences": list(SERIALIZED_REAR_OCCURRENCE_LABELS),
        "front_M2_occurrences": list(SERIALIZED_FRONT_OCCURRENCE_LABELS),
        "rear_M3_retained_stacks": rear,
        "front_M2_blind_spoke_stacks": front_rows,
        "minimum_rear_insert_engagement_mm": (
            minimum_rear_insert_engagement
        ),
        "minimum_rear_blind_positive_material_mm": (
            minimum_rear_blind_material
        ),
        "minimum_front_insert_engagement_mm": (
            minimum_front_insert_engagement
        ),
        "minimum_front_screw_tip_clearance_mm": (
            minimum_front_tip_clearance
        ),
        "minimum_front_blind_positive_material_mm": (
            minimum_front_blind_material
        ),
        "all_six_screws_terminate_in_positive_printed_material": (
            attachment[
                "all_six_screws_terminate_in_positive_printed_material"
            ]
        ),
        "any_balance_fastener_over_open_air": attachment[
            "any_balance_fastener_over_open_air"
        ],
        "physical_pull_proof_complete": attachment[
            "physical_pull_proof_complete"
        ],
    }
    checks.extend((
        Check(
            "retained_flyer_arm_single_solid",
            len(p["retained_arm"].solids()) == 1,
            float(len(p["retained_arm"].solids())), 1.0, "solids", "==",
        ),
        Check(
            "four_serialized_rear_M3_stacks_present",
            rear_count == 4, float(rear_count), 4.0, "stacks", "==",
        ),
        Check(
            "two_serialized_front_M2_stacks_present",
            front_count == 2, float(front_count), 2.0, "stacks", "==",
        ),
        Check(
            "all_six_counterweight_screws_end_in_positive_material",
            attachment[
                "all_six_screws_terminate_in_positive_printed_material"
            ],
            float(attachment[
                "all_six_screws_terminate_in_positive_printed_material"
            ]), 1.0, "boolean", "==",
        ),
        Check(
            "no_counterweight_fastener_over_open_air",
            not attachment["any_balance_fastener_over_open_air"],
            float(attachment["any_balance_fastener_over_open_air"]),
            0.0, "boolean", "==",
        ),
        Check(
            "four_rear_retainer_caps_and_spacers_are_single_solids",
            rear["all_caps_and_posts_single_solid"],
            float(rear["all_caps_and_posts_single_solid"]),
            1.0, "boolean", "==",
        ),
        Check(
            "rear_M3_insert_full_axial_engagement",
            minimum_rear_insert_engagement >= 4.299,
            minimum_rear_insert_engagement, 4.299, "mm", ">=",
        ),
        Check(
            "rear_M3_blind_positive_material",
            minimum_rear_blind_material >= 1.8 - 1.0e-9,
            minimum_rear_blind_material, 1.8, "mm", ">=",
        ),
        Check(
            "front_M2_insert_full_axial_engagement",
            minimum_front_insert_engagement >= 4.0 - 1.0e-9,
            minimum_front_insert_engagement, 4.0, "mm", ">=",
        ),
        Check(
            "front_M2_screw_tip_clearance",
            minimum_front_tip_clearance >= 0.5,
            minimum_front_tip_clearance, 0.5, "mm", ">=",
        ),
        Check(
            "front_M2_blind_positive_material",
            minimum_front_blind_material >= 2.4,
            minimum_front_blind_material, 2.4, "mm", ">=",
        ),
    ))

    # ------------------------------------------------------------------ M2
    mount = p["m2_motor_mount"]
    mount_fixed = m2_mount_access_relief(mount)
    m2_screw_labels = [
        f"m2_mount_{side}_{height}_m5x12"
        for side in ("L", "R") for height in ("low", "high")
    ]
    m2_rows = []
    for label in m2_screw_labels:
        before = intersection_volume(mount, p[label])
        after = intersection_volume(mount_fixed, p[label])
        m2_rows.append({"label": label, "before_mm3": before,
                        "after_mm3": after})
        expect_clear = label.endswith("m5x12")
        checks.append(Check(
            f"{label}:candidate_mount_overlap", after <= BOOLEAN_TOL_MM3,
            after, BOOLEAN_TOL_MM3, "mm^3", "<=" if expect_clear else "",
            f"current exact overlap {before:.6f} mm^3"))
    evidence["m2_mount_screws"] = m2_rows
    evidence["m2_mount_candidate"] = {
        "bbox": _bbox(mount_fixed),
        "solids": len(mount_fixed.solids()),
        "removed_volume_mm3": float(mount.volume - mount_fixed.volume),
        "repair": "two OD10 tunnels x=+-80,y=-72,z=-103..-66",
    }

    current_spacer_gap = distance(p["m2_inner_center_spacer"],
                                  p["m2_outer_race_spacer"])
    candidate_spacer_gap = distance(p["m2_inner_center_spacer"],
                                    m2_outer_spacer_id22())
    checks.append(Check(
        "m2_concentric_spacer_running_clearance", candidate_spacer_gap >= 2.0 - 1e-6,
        candidate_spacer_gap, 2.0, "mm", ">=",
        f"current exact radial clearance {current_spacer_gap:.6f} mm"))
    evidence["m2_spacer_running_clearance"] = {
        "current_mm": current_spacer_gap,
        "candidate_mm": candidate_spacer_gap,
        "repair": "outer-race spacer ID20 -> ID22; OD27.8 and length11 unchanged",
    }

    block_fixed = flyer_block_with_od26_running_bore(p["flyer_block"])
    # The released D10 NBK split-clamp pulley is one vendor occurrence whose
    # supplied M2 clamp bolt is already included.  The retired printed pulley
    # M3 insert/set-screw occurrences must not be recreated as audit aliases.
    rotating_labels = ["flyer_pulley"]
    block_rows = []
    for label in rotating_labels:
        before = distance(p["flyer_block"], p[label])
        after = distance(block_fixed, p[label])
        block_rows.append({"label": label, "before_mm": before,
                           "after_mm": after})
        checks.append(Check(
            f"{label}:flyer_block_candidate_clearance", after >= 2.0 - 1e-6,
            after, 2.0, "mm", ">=",
            f"current exact clearance {before:.6f} mm"))
    evidence["flyer_block_running_clearance"] = block_rows

    # ---------------------------------------------------------- rear bases
    base_specs = {
        "spool": (p["spool_bracket"],
                  [(P.rear_post_x, P.spool_y + dy) for dy in (-18.0, 18.0)]),
        "felt": (p["felt_tensioner"],
                 [(P.rear_post_x, P.felt_y + dy) for dy in (-9.0, 9.0)]),
        "dancer": (p["dancer_base"],
                   [(P.rear_post_x, P.dancer_y + dy)
                    for dy in P.dancer_base_mount_offsets]),
        "entry": (p["entry_bracket"],
                  [(P.rear_post_x, dy) for dy in (-8.0, 8.0)]),
    }
    base_rows = []
    candidate_screws: dict[str, Part] = {}
    for name, (body, centers) in base_specs.items():
        candidate_body = rear_post_base_with_countersinks(body, centers)
        for index, (x, y) in enumerate(centers, 1):
            current_label = f"{name}_base_m5x12_{index}"
            candidate = _m5x12_countersunk(
                f"{name}_base_iso10642_m5x12_{index}", x, y)
            candidate_screws[current_label] = candidate
            current_overlap = intersection_volume(body, p[current_label])
            candidate_overlap = intersection_volume(candidate_body, candidate)
            base_rows.append({
                "base": name, "index": index,
                "current_body_screw_overlap_mm3": current_overlap,
                "candidate_body_screw_overlap_mm3": candidate_overlap,
            })
            checks.append(Check(
                f"{name}_base_flush_screw_{index}",
                candidate_overlap <= BOOLEAN_TOL_MM3,
                candidate_overlap, BOOLEAN_TOL_MM3, "mm^3", "<=",
                f"current overlap {current_overlap:.6f} mm^3"))
    evidence["rear_post_base_screws"] = base_rows

    # Exact arm sweep against current and candidate fixed screw heads.
    sweep_angles = [P.dancer_stop_offsets_deg[0] + i * 0.25
                    for i in range(int(round(
                        (P.dancer_stop_offsets_deg[1] -
                         P.dancer_stop_offsets_deg[0]) / 0.25)) + 1)]
    fixed_screw_labels = [
        "dancer_base_m5x12_1", "dancer_base_m5x12_2",
        "entry_base_m5x12_1", "entry_base_m5x12_2",
    ]
    sweep_rows = []
    for label in fixed_screw_labels:
        current_max = 0.0
        current_min_distance = math.inf
        candidate_max = 0.0
        candidate_min_distance = math.inf
        worst_angle = None
        for angle in sweep_angles:
            arm = _rotate_about_dancer_pivot(p["dancer_arm"], angle)
            cur_vol = intersection_volume(arm, p[label])
            cand_vol = intersection_volume(arm, candidate_screws[label])
            if cur_vol > current_max:
                current_max, worst_angle = cur_vol, angle
            current_min_distance = min(current_min_distance,
                                       distance(arm, p[label]))
            candidate_max = max(candidate_max, cand_vol)
            candidate_min_distance = min(candidate_min_distance,
                                         distance(arm, candidate_screws[label]))
        sweep_rows.append({
            "label": label,
            "current_max_overlap_mm3": current_max,
            "current_min_distance_mm": current_min_distance,
            "current_worst_angle_deg": worst_angle,
            "candidate_max_overlap_mm3": candidate_max,
            "candidate_min_distance_mm": candidate_min_distance,
        })
        checks.append(Check(
            f"dancer_sweep:{label}:flush_candidate",
            candidate_max <= BOOLEAN_TOL_MM3 and candidate_min_distance >= 0.9,
            candidate_min_distance, 0.9, "mm", ">=",
            f"current max overlap {current_max:.6f} mm^3 at {worst_angle} deg"))
    evidence["dancer_arm_vs_base_screws"] = sweep_rows

    # Entry bracket against the rotating pulley and its axle/anchor stack.
    moving_labels = [
        "dancer_pulley_shim_front", "dancer_pulley_shoulder_m2p5",
        "dancer_pulley_shim_rear", "dancer_pulley_nyloc_m2p5",
        "dancer_spring_moving_m2x16_flush", "dancer_spring_moving_sleeve",
        "dancer_spring_moving_washer_rear",
        "dancer_spring_moving_washer_front",
        "dancer_spring_moving_m2_nyloc",
    ] + [f"dancer_pulley_shim_arm_rear_{i}" for i in range(1, 6)]
    moving_shapes = {"dancer_pulley": p["dancer_pulley"]}
    moving_shapes.update({label: p[label] for label in moving_labels})
    entry_fixed = entry_bracket_with_pulley_notch(p["entry_bracket"])
    moving_shapes["dancer_spring_moving_m2x16_flush"] = (
        moving_anchor_m2x16_countersunk())
    entry_sweep = []
    for label, shape in moving_shapes.items():
        current_shape = p[label] if label in p else shape
        max_overlap = 0.0
        current_max_overlap = 0.0
        min_gap = math.inf
        worst_angle = None
        for angle in sweep_angles:
            moved = _rotate_about_dancer_pivot(shape, angle)
            current_moved = _rotate_about_dancer_pivot(current_shape, angle)
            current_max_overlap = max(
                current_max_overlap,
                intersection_volume(p["entry_bracket"], current_moved))
            overlap = intersection_volume(entry_fixed, moved)
            gap = distance(entry_fixed, moved)
            if overlap > max_overlap:
                max_overlap, worst_angle = overlap, angle
            min_gap = min(min_gap, gap)
        entry_sweep.append({"label": label,
                            "current_max_overlap_mm3": current_max_overlap,
                            "candidate_max_overlap_mm3": max_overlap,
                            "candidate_min_distance_mm": min_gap,
                            "worst_angle_deg": worst_angle})
        checks.append(Check(
            f"entry_bracket_sweep:{label}", max_overlap <= BOOLEAN_TOL_MM3,
            max_overlap, BOOLEAN_TOL_MM3, "mm^3", "<=",
            f"current max overlap {current_max_overlap:.6f} mm^3; "
            f"candidate minimum distance {min_gap:.6f} mm"))
    evidence["entry_bracket_vs_moving_hardware"] = entry_sweep
    evidence["entry_bracket_candidate"] = {
        "solids": len(entry_fixed.solids()),
        "removed_volume_mm3": float(
            p["entry_bracket"].volume - entry_fixed.volume),
        "notch": "x=-38.5..-27,y=-22..-7,z=-171..-163.5",
        "stop_centers_unchanged": [list(v) for v in P.dancer_stop_centers],
        "stop_angles_unchanged_deg": list(P.dancer_stop_offsets_deg),
    }
    arm_fixed = dancer_arm_with_flush_moving_anchor(p["dancer_arm"])
    anchor_screw_fixed = moving_anchor_m2x16_countersunk()
    anchor_embed = intersection_volume(arm_fixed, anchor_screw_fixed)
    checks.append(Check(
        "dancer_moving_anchor_flush_seat", anchor_embed <= BOOLEAN_TOL_MM3,
        anchor_embed, BOOLEAN_TOL_MM3, "mm^3", "<=",
        "ISO10642 top face is flush at arm rear z=-163"))
    evidence["dancer_moving_anchor_candidate"] = {
        "arm_solids": len(arm_fixed.solids()),
        "arm_removed_volume_mm3": float(p["dancer_arm"].volume -
                                         arm_fixed.volume),
        "screw_arm_overlap_mm3": anchor_embed,
        "repair": "OD4.8/OD2.4 90-degree rear countersink + ISO14581 M2x16",
    }

    # ---------------------------------------------------------- felt stack
    felt_order = [
        "felt_backing_fixed", "felt_pad_fixed", "felt_pad_moving",
        "felt_backing_moving", "felt_compression_spring",
        "felt_spring_thrust_washer", "felt_m4_wingnut",
    ]
    felt_rows = []
    for left, right in zip(felt_order, felt_order[1:]):
        gap = distance(p[left], p[right])
        overlap = intersection_volume(p[left], p[right])
        felt_rows.append({"pair": [left, right], "distance_mm": gap,
                          "overlap_mm3": overlap})
    stud_bbox = _bbox(p["felt_m4x55_stud"])
    wing_bbox = _bbox(p["felt_m4_wingnut"])
    thread_proud = stud_bbox["max"][2] - wing_bbox["max"][2]
    stud55 = hardware.place(
        hardware.threaded_stud("M4", 55.0),
        (P.rear_post_x, P.felt_y, -170.0), axis="+z",
        label="felt_m4x55_stud_candidate")
    stud55_bbox = _bbox(stud55)
    candidate_thread_proud = stud55_bbox["max"][2] - wing_bbox["max"][2]
    felt_rows.append({"thread_proud_mm": thread_proud,
                      "stud_front_z": stud_bbox["max"][2],
                      "wingnut_front_z": wing_bbox["max"][2]})
    felt_rows[-1]["candidate_m4x55_thread_proud_mm"] = candidate_thread_proud
    checks.append(Check(
        "felt_stud_thread_proud_candidate", candidate_thread_proud >= 1.4,
        candidate_thread_proud, 1.4, "mm", ">=",
        f"production M4x55 proud length is {thread_proud:.6f} mm"))
    # This is the only candidate repair that adds material.  All other M2/M3
    # repairs are subtractive or move existing hardware inward, so they cannot
    # worsen belt/wire clearance.  Check the longer stud explicitly against
    # both visible wire geometry and the exact imported 200-2GT belt.
    import assembly
    stud55_wire_gap = distance(stud55, wire_vis.wire_static())
    stud55_belt_gap = distance(stud55, assembly._belt())
    checks.append(Check(
        "felt_m4x55_to_static_wire", stud55_wire_gap >= 2.0,
        stud55_wire_gap, 2.0, "mm", ">=",
        "wire visualization is conservative OD2"))
    checks.append(Check(
        "felt_m4x55_to_gt2_belt", stud55_belt_gap >= 2.0,
        stud55_belt_gap, 2.0, "mm", ">="))
    felt_rows[-1]["candidate_m4x55_wire_gap_mm"] = stud55_wire_gap
    felt_rows[-1]["candidate_m4x55_belt_gap_mm"] = stud55_belt_gap
    evidence["felt_stack"] = felt_rows

    # Current felt base heads against the OD20 drag stack, then flush repair.
    felt_stack_labels = ["felt_backing_fixed", "felt_pad_fixed",
                         "felt_pad_moving", "felt_backing_moving"]
    felt_head_rows = []
    for index in (1, 2):
        label = f"felt_base_m5x12_{index}"
        for stack_label in felt_stack_labels:
            current_overlap = intersection_volume(p[label], p[stack_label])
            candidate_overlap = intersection_volume(
                candidate_screws[label], p[stack_label])
            felt_head_rows.append({
                "screw": label, "stack": stack_label,
                "current_overlap_mm3": current_overlap,
                "candidate_overlap_mm3": candidate_overlap,
            })
            checks.append(Check(
                f"{label}:{stack_label}:flush_candidate",
                candidate_overlap <= BOOLEAN_TOL_MM3,
                candidate_overlap, BOOLEAN_TOL_MM3, "mm^3", "<=",
                f"current overlap {current_overlap:.6f} mm^3"))
    evidence["felt_base_heads_vs_stack"] = felt_head_rows

    # ---------------------------------------------------------- spool stack
    spool_order = ["spool_axle_washer_left", "spool_bracket",
                   "spool_drum", "spool_axle_washer_right",
                   "spool_axle_nyloc"]
    spool_rows = []
    axle = p["spool_axle_m8x75"]
    for label in spool_order:
        spool_rows.append({
            "label": label,
            "axle_overlap_mm3": intersection_volume(axle, p[label]),
            "axle_distance_mm": distance(axle, p[label]),
        })
    left_ear_gap = distance(p["spool_axle_washer_left"],
                            p["spool_bracket"])
    right_ear_gap = distance(p["spool_axle_washer_right"],
                             p["spool_bracket"])
    right_nut_gap = distance(p["spool_axle_washer_right"],
                             p["spool_axle_nyloc"])
    spool_rows.append({"left_washer_to_ear_gap_mm": left_ear_gap,
                       "right_washer_to_ear_gap_mm": right_ear_gap,
                       "right_washer_to_nut_gap_mm": right_nut_gap})
    evidence["spool_stack"] = spool_rows

    # --------------------------------------------------------- flyer clamps
    flyer_insert_specs = {
        f"{stack_id}_McMaster_94459A130_insert": (
            f"{stack_id}_printed_retainer_with_three_spacers"
        )
        for stack_id in SERIALIZED_REAR_STACK_IDS
    }
    insert_rows = []
    for insert_label, body_label in flyer_insert_specs.items():
        embed = intersection_volume(p[insert_label], p[body_label])
        insert_rows.append({"insert": insert_label, "body": body_label,
                            "interference_embed_mm3": embed,
                            "classification": "intended_heat_set_interference"})
        checks.append(Check(
            f"{insert_label}:heat_set_interference", embed > 0.1,
            embed, 0.1, "mm^3", ">",
            "positive plastic displacement is intended for a heat-set insert"))
    evidence["flyer_heat_set_inserts"] = insert_rows

    cw_parts = counterweight_attachment_components(p)
    cw_pairs = (
        ("M3x6_screw", "positive_1mm_arm_floor", "screw", "arm"),
        ("positive_1mm_arm_floor", "tungsten_slug", "arm", "slug"),
        ("tungsten_slug", "printed_retainer_spacers", "slug", "retainer"),
        ("printed_retainer_boss", "M3_heat_set_insert", "retainer", "insert"),
    )
    cw_rows = []
    for left_name, right_name, left_key, right_key in cw_pairs:
        left = cw_parts[left_key]
        right = cw_parts[right_key]
        cw_rows.append({
            "stack_id": REPRESENTATIVE_REAR_STACK_ID,
            "pair": [left_name, right_name],
            "overlap_mm3": intersection_volume(left, right),
            "distance_mm": distance(left, right),
        })
    checks.append(Check(
        "representative_rear_M3_stack_has_closed_contacts",
        max(row["distance_mm"] for row in cw_rows) <= 1.0e-6,
        max(row["distance_mm"] for row in cw_rows),
        1.0e-6, "mm", "<=",
        "screw, positive arm floor, slug, retainer and insert are contiguous",
    ))
    evidence["counterweight_stack"] = cw_rows

    passed = all(check.passed for check in checks)
    return {
        "schema": 1,
        "passed": passed,
        "boolean_tolerance_mm3": BOOLEAN_TOL_MM3,
        "dancer_sweep_deg": [P.dancer_stop_offsets_deg[0],
                              P.dancer_stop_offsets_deg[1]],
        "dancer_sweep_step_deg": 0.25,
        "checks": [asdict(check) for check in checks],
        "evidence": evidence,
        "intended_contacts": intended_contact_map(),
    }


def intended_contact_map() -> list[dict[str, str]]:
    """Semantic map used to distinguish fits from forbidden embedding."""
    return [
        {"pair": "heat-set insert / printed pilot",
         "class": "intentional interference",
         "rule": "positive volume required; insert must not break into shaft bore"},
        {"pair": "screw head or washer / bearing face",
         "class": "intended face contact",
         "rule": "zero volume; zero axial gap"},
        {"pair": "screw shank / clearance hole",
         "class": "clearance",
         "rule": "zero positive volume"},
        {"pair": "threaded screw or stud / nut or insert bore",
         "class": "thread engagement",
         "rule": "nominal coaxial engagement; simplified BREP may be tangent only"},
        {"pair": "felt pad / adjacent backing or wire",
         "class": "intended compliant contact",
         "rule": "rigid backing interfaces zero volume; pad compression is hardware-only"},
        {"pair": "dancer hard-stop sleeve / arm edge at stop angle",
         "class": "intended endpoint contact",
         "rule": "contact allowed only at -3/+5.5 deg, never positive-volume embedding"},
        {"pair": "M2 inner-race spacer / outer-race spacer",
         "class": "running clearance",
         "rule": "not a contact exemption; target >=2.0 mm radial clearance"},
        {"pair": "flyer pulley clamp hardware / static flyer block",
         "class": "running clearance",
         "rule": "not a contact exemption; target >=2.0 mm exact BREP clearance"},
    ]


def main() -> int:
    result = run_audit()
    out = Path(__file__).resolve().parent.parent / "out" / "reports"
    out.mkdir(parents=True, exist_ok=True)
    target = out / "m2_m3_hardware_audit.json"
    target.write_text(json.dumps(result, indent=2), encoding="utf-8")
    failed = [check for check in result["checks"] if not check["passed"]]
    print(f"{len(result['checks']) - len(failed)}/{len(result['checks'])} checks pass")
    for check in failed:
        print(f"FAIL {check['name']}: {check['measured']:.6f} "
              f"{check['units']} {check['relation']} {check['limit']:.6f}")
    print(target)
    return len(failed)


if __name__ == "__main__":
    raise SystemExit(main())
