"""Fail-closed retraction topology for the aggregate-boundary follower.

This deterministic design-analysis module turns the isolated mechanical audit
into an explicit geometry, force, and interlock contract.  It does not create
CAD, select the unresolved production hardware, modify the controller, or
authorize assembly integration, procurement, winding, or release.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORTS = ROOT / "out" / "reports"

OUTPUT_JSON = REPORTS / "aggregate_boundary_follower_retraction_topology.json"
OUTPUT_MD = REPORTS / "aggregate_boundary_follower_retraction_topology.md"

FOLLOWER_CAD_SOURCE = ROOT / "cad" / "aggregate_boundary_floating_follower.py"
M0_GATE_SOURCE = ROOT / "cad" / "m1_selector_alternating_former.py"
CONTROLLER_SOURCE = ROOT / "sim" / "controller_adapter.py"

SCHEMA = "aggregate-boundary-follower-retraction-topology/v1"

# Follower-local frame: +X radial/outward, +Y tangential, +Z stator axis.
REFERENCE_AXIS_Z_MM = 95.0

RADIAL_HARD_RETRACTED_MM = 13.8
RADIAL_USABLE_RETRACTED_MM = 14.0
RADIAL_MID_MM = 17.0
RADIAL_USABLE_EXTENDED_MM = 20.0
RADIAL_HARD_EXTENDED_MM = 20.2

BELLCRANK_PIVOT_LOCAL_MM = (17.0, 26.0, 20.0)
SLIDE_PIN_Y_MM = 6.0
SLIDE_PIN_Z_MM = 20.0
BELLCRANK_LONG_ARM_NOMINAL_MM = 20.0
BELLCRANK_SHORT_ARM_MM = 5.8
FIXED_SPRING_ANCHOR_LOCAL_MM = (
    22.727155665,
    15.583655094,
    20.0,
)

LEM_FREE_LENGTH_MM = 9.5
LEM_MAX_LENGTH_MM = 13.82
LEM_INITIAL_LOAD_N = 1.77
LEM_MAX_LOAD_N = 12.0
LEM_RATE_N_PER_MM = 2.35
LEM_OD_MM = 3.5
CONTACT_FORCE_HARD_CAP_N = 2.0

INDEPENDENT_RETURN_FORCE_N = 0.25
MAXIMUM_BREAKAWAY_FORCE_N = 0.125

TANGENTIAL_USABLE_HALF_TRAVEL_MM = 0.5
TANGENTIAL_HARD_HALF_TRAVEL_MM = 0.6
TANGENTIAL_SHAFT_DIAMETER_MM = 3.0
TANGENTIAL_SHAFT_LENGTH_MM = 16.0
TANGENTIAL_SHAFT_CENTER_Z_MM = 19.0
TANGENTIAL_BUSHING_ID_MM = 3.0
TANGENTIAL_BUSHING_OD_MM = 5.0
TANGENTIAL_BUSHING_LENGTH_MM = 6.0
TANGENTIAL_SPRING_FREE_LENGTH_MM = 5.5
TANGENTIAL_SPRING_INSTALLED_LENGTH_MM = 4.5
TANGENTIAL_SPRING_RATE_N_PER_MM = 0.15

M0_ENGAGED_AXIS_Z_MAX_MM = 24.5
M0_ALL_RETRACTED_AXIS_Z_MIN_MM = 29.0
M0_CAM_RADIAL_RATIO = 1.5
M0_HOME_AXIS_Z_MM = 95.0
M0_LEAD_MM_PER_REV = 8.0
M0_ROLLER_FACE_OFFSET_X_MM = 4.0

RADIAL_PROOF_LIMIT_MM = 14.05
TANGENTIAL_PROOF_LIMIT_MM = 0.05


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(value: dict[str, Any]) -> str:
    body = dict(value)
    body.pop("report_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _rounded(value: float) -> float:
    """Keep generated evidence stable and readable without hiding margins."""

    return round(float(value), 9)


def local_to_machine(
    local_xyz_mm: tuple[float, float, float], axis_z_mm: float,
) -> tuple[float, float, float]:
    """Transform one follower-local point into the current machine frame."""

    x_local, y_local, z_local = local_xyz_mm
    return (-y_local, z_local, float(axis_z_mm) - x_local)


def slide_pin_local(x_radial_mm: float) -> tuple[float, float, float]:
    return (float(x_radial_mm), SLIDE_PIN_Y_MM, SLIDE_PIN_Z_MM)


def bellcrank_long_arm_length_mm(x_radial_mm: float) -> float:
    dx = float(x_radial_mm) - BELLCRANK_PIVOT_LOCAL_MM[0]
    return math.hypot(dx, SLIDE_PIN_Y_MM - BELLCRANK_PIVOT_LOCAL_MM[1])


def moving_spring_anchor_local(
    x_radial_mm: float,
) -> tuple[float, float, float]:
    """Return the 5.8 mm arm endpoint perpendicular to the long arm."""

    dx = float(x_radial_mm) - BELLCRANK_PIVOT_LOCAL_MM[0]
    long_length = bellcrank_long_arm_length_mm(x_radial_mm)
    pivot_x, pivot_y, pivot_z = BELLCRANK_PIVOT_LOCAL_MM
    return (
        pivot_x + BELLCRANK_SHORT_ARM_MM * 20.0 / long_length,
        pivot_y + BELLCRANK_SHORT_ARM_MM * dx / long_length,
        pivot_z,
    )


def spring_length_mm(x_radial_mm: float) -> float:
    moving = moving_spring_anchor_local(x_radial_mm)
    fixed = FIXED_SPRING_ANCHOR_LOCAL_MM
    return math.sqrt(sum((moving[i] - fixed[i]) ** 2 for i in range(3)))


def spring_motion_ratio(x_radial_mm: float) -> float:
    """Analytical d(spring length)/d(radial travel)."""

    x_value = float(x_radial_mm)
    dx = x_value - BELLCRANK_PIVOT_LOCAL_MM[0]
    long_length = bellcrank_long_arm_length_mm(x_value)
    moving_x, moving_y, _ = moving_spring_anchor_local(x_value)
    fixed_x, fixed_y, _ = FIXED_SPRING_ANCHOR_LOCAL_MM
    d_moving_x_dx = -116.0 * dx / long_length ** 3
    d_moving_y_dx = 2320.0 / long_length ** 3
    length = spring_length_mm(x_value)
    return (
        (moving_x - fixed_x) * d_moving_x_dx
        + (moving_y - fixed_y) * d_moving_y_dx
    ) / length


def radial_force_row(x_radial_mm: float) -> dict[str, float]:
    length = spring_length_mm(x_radial_mm)
    ratio = spring_motion_ratio(x_radial_mm)
    spring_load = LEM_INITIAL_LOAD_N + LEM_RATE_N_PER_MM * (
        length - LEM_FREE_LENGTH_MM
    )
    follower_force = spring_load * ratio
    report = {
        "radial_center_mm": _rounded(x_radial_mm),
        "long_arm_radius_mm": _rounded(
            bellcrank_long_arm_length_mm(x_radial_mm)
        ),
        "spring_length_mm": _rounded(length),
        "instantaneous_motion_ratio": _rounded(ratio),
        "spring_load_N": _rounded(spring_load),
        "LEM_follower_force_N": _rounded(follower_force),
        "independent_return_force_N": INDEPENDENT_RETURN_FORCE_N,
        "combined_inward_contact_force_N": _rounded(
            follower_force + INDEPENDENT_RETURN_FORCE_N
        ),
    }
    return report


def radial_topology() -> dict[str, Any]:
    sample_positions = (
        RADIAL_HARD_RETRACTED_MM,
        RADIAL_USABLE_RETRACTED_MM,
        RADIAL_MID_MM,
        RADIAL_USABLE_EXTENDED_MM,
        RADIAL_HARD_EXTENDED_MM,
    )
    rows = [radial_force_row(value) for value in sample_positions]
    hard_rows = (rows[0], rows[-1])
    maximum_combined = max(
        row["combined_inward_contact_force_N"] for row in rows
    )
    maximum_spring_length = max(row["spring_length_mm"] for row in rows)
    maximum_spring_load = max(row["spring_load_N"] for row in rows)
    return {
        "status": "ANALYTICAL_TOPOLOGY_ONLY",
        "frame": {
            "axes": "+X radial/outward, +Y tangential, +Z stator axis",
            "local_to_machine_equation": "(-y_local, z_local, axis_z-x_local)",
            "reference_axis_z_mm": REFERENCE_AXIS_Z_MM,
        },
        "primary_tower_datums": {
            "machine_plane_y_mm": -114.0,
            "key_centers_machine_x_mm": [-10.0, 10.0],
            "key_center_machine_z_mm": 61.0,
            "key_size_mm": [3.0, 2.0, 1.5],
            "key_clearance_per_side_mm": 0.05,
            "M4_axes_machine_x_mm": [-21.0, 21.0],
            "M4_axes_machine_z_mm": [60.0, 66.0],
            "hardware": "4x M4x10 + washer + short insert",
            "wire_reaction_routes_to_primary_tower_not_bellcrank": True,
        },
        "bellcrank": {
            "carrier_fixed_pivot_local_mm": list(BELLCRANK_PIVOT_LOCAL_MM),
            "pivot_axis": "+Z",
            "pivot_hardware_envelope": (
                "OD5x10 shoulder screw, double-shear clevis, through-retained"
            ),
            "moving_slide_pin_equation_local_mm": "(x_radial, 6.0, 20.0)",
            "moving_pin_axis": "+Z",
            "moving_pin_hardware_envelope": (
                "OD3 shoulder pin/roller on integral radial-slide ear"
            ),
            "long_arm_radius_equation_mm": (
                "sqrt((x_radial-17)^2 + 20^2)"
            ),
            "nominal_long_arm_mm": BELLCRANK_LONG_ARM_NOMINAL_MM,
            "short_arm_mm": BELLCRANK_SHORT_ARM_MM,
            "nominal_motion_ratio": _rounded(
                BELLCRANK_SHORT_ARM_MM / BELLCRANK_LONG_ARM_NOMINAL_MM
            ),
            "moving_anchor_equation_local_mm": (
                "P + 5.8*(20/L, (x_radial-17)/L, 0), "
                "L=sqrt((x_radial-17)^2+400)"
            ),
            "fixed_spring_anchor_local_mm": list(
                FIXED_SPRING_ANCHOR_LOCAL_MM
            ),
            "moving_pin_radial_slot_width_mm": 3.2,
            "moving_pin_radial_slot_range_mm": [19.8, 20.5],
            "positive_volume_attachment_requirements": [
                "carrier-owned gusseted double-shear pivot clevis",
                "integral or positively retained radial-slide pin ear",
                "carrier-owned fixed spring-anchor boss",
                "through retention; no short direct thread into aluminum",
            ],
        },
        "LEM050AB01": {
            "role": "radial preload and one return path",
            "free_length_mm": LEM_FREE_LENGTH_MM,
            "maximum_length_mm": LEM_MAX_LENGTH_MM,
            "initial_load_N": LEM_INITIAL_LOAD_N,
            "maximum_load_N": LEM_MAX_LOAD_N,
            "rate_N_per_mm": LEM_RATE_N_PER_MM,
            "outside_diameter_mm": LEM_OD_MM,
            "force_equation": (
                "F_follower=(1.77+2.35*(spring_length-9.5))*"
                "d(spring_length)/d(x_radial)"
            ),
            "force_rows": rows,
            "motion_ratio_range": [
                min(row["instantaneous_motion_ratio"] for row in rows),
                max(row["instantaneous_motion_ratio"] for row in rows),
            ],
            "hard_endpoint_rows": list(hard_rows),
            "maximum_sampled_spring_length_mm": maximum_spring_length,
            "maximum_sampled_spring_load_N": maximum_spring_load,
            "maximum_combined_inward_contact_force_N": maximum_combined,
            "contact_force_hard_cap_N": CONTACT_FORCE_HARD_CAP_N,
            "spring_service_envelope_local_mm": {
                "x": [20.5, 25.0],
                "y": [13.0, 30.0],
                "z": [17.5, 22.5],
            },
            "existing_constant_0p29_report_must_be_rerun": True,
        },
        "independent_radial_return": {
            "required": True,
            "topology": (
                "direct radial-slide-to-carrier constant-force spring or "
                "qualified flexure; not routed through the bellcrank"
            ),
            "inward_force_target_N": INDEPENDENT_RETURN_FORCE_N,
            "full_hard_travel_mm": _rounded(
                RADIAL_HARD_EXTENDED_MM - RADIAL_HARD_RETRACTED_MM
            ),
            "maximum_measured_breakaway_force_N": MAXIMUM_BREAKAWAY_FORCE_N,
            "minimum_return_to_breakaway_ratio": _rounded(
                INDEPENDENT_RETURN_FORCE_N / MAXIMUM_BREAKAWAY_FORCE_N
            ),
            "single_broken_LEM_or_bellcrank_link_still_returns": True,
            "exact_spring_or_flexure_selected": False,
            "breakaway_force_measured": False,
        },
        "analytical_gates": {
            "hard_spring_length_within_LEM_limit": (
                maximum_spring_length <= LEM_MAX_LENGTH_MM
            ),
            "hard_spring_load_within_LEM_limit": (
                maximum_spring_load <= LEM_MAX_LOAD_N
            ),
            "combined_contact_force_below_2N": (
                maximum_combined < CONTACT_FORCE_HARD_CAP_N
            ),
            "independent_return_has_2x_breakaway_requirement": (
                INDEPENDENT_RETURN_FORCE_N
                >= 2.0 * MAXIMUM_BREAKAWAY_FORCE_N
            ),
        },
    }


def tangential_force_row(y_tangential_mm: float) -> dict[str, float]:
    center_compression = (
        TANGENTIAL_SPRING_FREE_LENGTH_MM
        - TANGENTIAL_SPRING_INSTALLED_LENGTH_MM
    )
    positive_compression = center_compression + float(y_tangential_mm)
    negative_compression = center_compression - float(y_tangential_mm)
    positive_force = TANGENTIAL_SPRING_RATE_N_PER_MM * positive_compression
    negative_force = TANGENTIAL_SPRING_RATE_N_PER_MM * negative_compression
    return {
        "tangential_offset_mm": _rounded(y_tangential_mm),
        "positive_side_spring_force_N": _rounded(positive_force),
        "negative_side_spring_force_N": _rounded(negative_force),
        "net_force_positive_y_N": _rounded(negative_force - positive_force),
        "restoring_force_magnitude_N": _rounded(
            abs(negative_force - positive_force)
        ),
    }


def tangential_topology() -> dict[str, Any]:
    rows = [
        tangential_force_row(value)
        for value in (-0.6, -0.5, 0.0, 0.5, 0.6)
    ]
    center_preload = TANGENTIAL_SPRING_RATE_N_PER_MM * (
        TANGENTIAL_SPRING_FREE_LENGTH_MM
        - TANGENTIAL_SPRING_INSTALLED_LENGTH_MM
    )
    return {
        "status": "TARGET_ENVELOPE_ONLY_UNSELECTED",
        "bearing": {
            "fixed_shaft_axis": "+Y",
            "fixed_shaft_center_equation_local_mm": (
                "(x_radial, 0.0, 19.0)"
            ),
            "ground_shaft_diameter_mm": TANGENTIAL_SHAFT_DIAMETER_MM,
            "shaft_length_envelope_mm": TANGENTIAL_SHAFT_LENGTH_MM,
            "support_ear_centers_y_mm": [-8.0, 8.0],
            "moving_flanged_polymer_bushing_envelope_mm": {
                "ID": TANGENTIAL_BUSHING_ID_MM,
                "OD": TANGENTIAL_BUSHING_OD_MM,
                "length": TANGENTIAL_BUSHING_LENGTH_MM,
            },
            "captured_T_slot_role": "anti-rotation and fragment capture only",
            "dry_running_liner_target_per_face_mm": 0.10,
            "aluminum_on_aluminum_bearing_authorized": False,
            "exact_shaft_and_bushing_SKUs_selected": False,
        },
        "opposed_centering_springs": {
            "fixed_ear_inner_faces_y_mm": [-7.5, 7.5],
            "free_length_target_mm": TANGENTIAL_SPRING_FREE_LENGTH_MM,
            "installed_center_length_target_mm": (
                TANGENTIAL_SPRING_INSTALLED_LENGTH_MM
            ),
            "rate_target_each_N_per_mm": TANGENTIAL_SPRING_RATE_N_PER_MM,
            "inside_diameter_minimum_mm": 3.2,
            "outside_diameter_maximum_mm": 5.0,
            "center_preload_each_N": _rounded(center_preload),
            "net_centering_stiffness_N_per_mm": _rounded(
                2.0 * TANGENTIAL_SPRING_RATE_N_PER_MM
            ),
            "restoring_force_at_usable_limit_N": _rounded(
                2.0
                * TANGENTIAL_SPRING_RATE_N_PER_MM
                * TANGENTIAL_USABLE_HALF_TRAVEL_MM
            ),
            "usable_half_travel_mm": TANGENTIAL_USABLE_HALF_TRAVEL_MM,
            "hard_stop_half_travel_mm": TANGENTIAL_HARD_HALF_TRAVEL_MM,
            "force_equation": "F_net_y=-2*k*y, k=0.15 N/mm",
            "force_rows": rows,
            "exact_spring_SKU_selected": False,
        },
        "attachment_requirements": [
            "shaft-support ears integral with or positively fixed to radial slide",
            "moving bushing cartridge integral with or retained to tangential yoke",
            "T-slot remains captured at both hard stops",
            "M0 V-dock must center a single-broken-spring carriage",
        ],
        "analytical_gates": {
            "both_springs_remain_compressed_at_hard_stops": all(
                row["positive_side_spring_force_N"] > 0.0
                and row["negative_side_spring_force_N"] > 0.0
                for row in rows
            ),
            "usable_limit_restoring_force_is_0p15N": math.isclose(
                rows[1]["restoring_force_magnitude_N"],
                0.15,
                rel_tol=0.0,
                abs_tol=1.0e-9,
            ),
        },
    }


def axis_z_to_m0_rad(axis_z_mm: float) -> float:
    return (
        (float(axis_z_mm) - REFERENCE_AXIS_Z_MM)
        * 2.0
        * math.pi
        / M0_LEAD_MM_PER_REV
    )


def cam_radial_center_mm(axis_z_mm: float) -> float:
    """Worst-case radial center forced by the positive M0 compression cam."""

    z_value = float(axis_z_mm)
    if z_value <= M0_ENGAGED_AXIS_Z_MAX_MM:
        return RADIAL_HARD_EXTENDED_MM
    return max(
        RADIAL_USABLE_RETRACTED_MM,
        RADIAL_HARD_EXTENDED_MM
        - M0_CAM_RADIAL_RATIO * (z_value - M0_ENGAGED_AXIS_Z_MAX_MM),
    )


def m0_retraction_topology() -> dict[str, Any]:
    complete_axis_z = M0_ENGAGED_AXIS_Z_MAX_MM + (
        RADIAL_HARD_EXTENDED_MM - RADIAL_USABLE_RETRACTED_MM
    ) / M0_CAM_RADIAL_RATIO
    start_face_x = RADIAL_HARD_EXTENDED_MM + M0_ROLLER_FACE_OFFSET_X_MM
    end_face_x = RADIAL_USABLE_RETRACTED_MM + M0_ROLLER_FACE_OFFSET_X_MM
    start_machine_z = M0_ENGAGED_AXIS_Z_MAX_MM - start_face_x
    end_machine_z = complete_axis_z - end_face_x
    dwell_end_machine_z = M0_HOME_AXIS_Z_MM - end_face_x
    surface_slope = (
        (end_face_x - start_face_x) / (end_machine_z - start_machine_z)
    )
    return {
        "status": "POSITIVE_CAM_TOPOLOGY_ONLY_UNMODELED",
        "existing_gate_API": {
            "ENGAGED_LOCKED_axis_z_max_mm": M0_ENGAGED_AXIS_Z_MAX_MM,
            "FORCED_RETRACTION_RAMP_axis_z_open_interval_mm": [24.5, 29.0],
            "ALL_RETRACTED_DISCONNECTED_axis_z_min_mm": (
                M0_ALL_RETRACTED_AXIS_Z_MIN_MM
            ),
            "engaged_threshold_M0_rad": _rounded(
                axis_z_to_m0_rad(M0_ENGAGED_AXIS_Z_MAX_MM)
            ),
            "all_retracted_threshold_M0_rad": _rounded(
                axis_z_to_m0_rad(M0_ALL_RETRACTED_AXIS_Z_MIN_MM)
            ),
        },
        "radial_positive_retraction": {
            "topology": (
                "two direct compression rollers on the radial slide and two "
                "machine-fixed closed rails; no safety pull through bellcrank"
            ),
            "roller_axes_local_mm": [
                ["x_radial+4.0", 0.0, 8.0],
                ["x_radial+4.0", 0.0, 12.0],
            ],
            "roller_axis_direction": "+Y",
            "cam_equation": "x=20.2-1.5*(axis_z-24.5), clamped at x=14.0",
            "cam_radial_motion_per_axis_motion": M0_CAM_RADIAL_RATIO,
            "ramp_start_axis_z_mm": M0_ENGAGED_AXIS_Z_MAX_MM,
            "ramp_start_radial_center_mm": RADIAL_HARD_EXTENDED_MM,
            "retraction_complete_axis_z_mm": _rounded(complete_axis_z),
            "retraction_complete_radial_center_mm": (
                RADIAL_USABLE_RETRACTED_MM
            ),
            "positive_dwell_before_API_boundary_mm": _rounded(
                M0_ALL_RETRACTED_AXIS_Z_MIN_MM - complete_axis_z
            ),
            "ramp_start_machine_z_mm": _rounded(start_machine_z),
            "ramp_end_machine_z_mm": _rounded(end_machine_z),
            "static_rail_surface_slope_dx_dz": _rounded(surface_slope),
            "static_rail_surface_angle_deg": _rounded(
                math.degrees(math.atan(abs(surface_slope)))
            ),
            "closed_dwell_machine_z_range_mm": [
                _rounded(end_machine_z),
                _rounded(dwell_end_machine_z),
            ],
            "closed_dwell_reaches_M0_home_axis_z_mm": M0_HOME_AXIS_Z_MM,
            "broken_LEM_or_bellcrank_cannot_redeploy_inside_closed_dwell": True,
        },
        "tangential_positive_centering_dock": {
            "topology": "symmetric V-funnel followed by closed parallel dwell",
            "acceptance_at_axis_z_24p5_mm": "abs(y)<=0.6 mm",
            "centered_by_axis_z_mm": 25.5,
            "centered_proof_limit_mm": TANGENTIAL_PROOF_LIMIT_MM,
            "approximate_output_per_axis_motion": 0.6,
            "closed_dwell_through_axis_z_mm": M0_HOME_AXIS_Z_MM,
            "CAD_modeled": False,
        },
        "gimbal_positive_neutral_dock": {
            "required": True,
            "topology_selected": False,
            "angle_limit_selected": False,
            "physical_all_DOF_retraction_proven": False,
        },
        "analytical_gates": {
            "cam_completes_before_axis_z_29": (
                complete_axis_z < M0_ALL_RETRACTED_AXIS_Z_MIN_MM
            ),
            "cam_forces_full_6p2mm_from_worst_case": math.isclose(
                cam_radial_center_mm(complete_axis_z),
                RADIAL_USABLE_RETRACTED_MM,
                rel_tol=0.0,
                abs_tol=1.0e-9,
            ),
            "closed_dwell_reaches_machine_z_77": math.isclose(
                dwell_end_machine_z, 77.0, rel_tol=0.0, abs_tol=1.0e-9
            ),
        },
    }


def dual_nc_pair(channel_a_closed: bool, channel_b_closed: bool) -> dict[str, Any]:
    """Evaluate one dual, positive-opening, de-energize-to-trip NC pair."""

    a_closed = bool(channel_a_closed)
    b_closed = bool(channel_b_closed)
    disagreement = a_closed != b_closed
    proved = a_closed and b_closed and not disagreement
    if proved:
        meaning = "safe_position_proved"
    elif disagreement:
        meaning = "channel_disagreement_fault_latched"
    else:
        meaning = "safe_position_not_proved"
    return {
        "channel_A_closed": a_closed,
        "channel_B_closed": b_closed,
        "safe_position_proved": proved,
        "disagreement_fault_latched": disagreement,
        "meaning": meaning,
    }


def evaluate_interlock(
    *,
    m0_all_retracted_zone: bool,
    winding_zone: bool,
    selector_gate_engaged_locked: bool,
    radial_nc: tuple[bool, bool],
    tangential_nc: tuple[bool, bool],
    gimbal_nc: tuple[bool, bool],
    selector_nc: tuple[bool, bool],
) -> dict[str, Any]:
    pairs = {
        "radial": dual_nc_pair(*radial_nc),
        "tangential": dual_nc_pair(*tangential_nc),
        "gimbal": dual_nc_pair(*gimbal_nc),
        "selector": dual_nc_pair(*selector_nc),
    }
    system_healthy = not any(
        pair["disagreement_fault_latched"] for pair in pairs.values()
    )
    actual_all_retracted = all(
        pairs[name]["safe_position_proved"]
        for name in ("radial", "tangential", "gimbal")
    )
    all_retracted_permissive = (
        system_healthy
        and bool(m0_all_retracted_zone)
        and actual_all_retracted
    )
    selector_seated_permissive = (
        system_healthy
        and bool(winding_zone)
        and bool(selector_gate_engaged_locked)
        and pairs["selector"]["safe_position_proved"]
    )
    return {
        "pairs": pairs,
        "system_channels_healthy": system_healthy,
        "actual_all_retracted": actual_all_retracted,
        "all_retracted_permissive": all_retracted_permissive,
        "selector_seated_permissive": selector_seated_permissive,
        "M1_enable": all_retracted_permissive,
        "M2_enable": (
            all_retracted_permissive or selector_seated_permissive
        ),
    }


def interlock_topology() -> dict[str, Any]:
    pair_truth_table = [
        dual_nc_pair(a_closed, b_closed)
        for a_closed in (False, True)
        for b_closed in (False, True)
    ]
    scenarios = [
        {
            "name": "home_all_actual_positions_proved",
            "inputs": {
                "m0_all_retracted_zone": True,
                "winding_zone": False,
                "selector_gate_engaged_locked": False,
                "radial_nc": (True, True),
                "tangential_nc": (True, True),
                "gimbal_nc": (True, True),
                "selector_nc": (False, False),
            },
        },
        {
            "name": "winding_selector_seated",
            "inputs": {
                "m0_all_retracted_zone": False,
                "winding_zone": True,
                "selector_gate_engaged_locked": True,
                "radial_nc": (False, False),
                "tangential_nc": (False, False),
                "gimbal_nc": (False, False),
                "selector_nc": (True, True),
            },
        },
        {
            "name": "intermediate_M0_no_dock_proved",
            "inputs": {
                "m0_all_retracted_zone": False,
                "winding_zone": False,
                "selector_gate_engaged_locked": False,
                "radial_nc": (False, False),
                "tangential_nc": (False, False),
                "gimbal_nc": (False, False),
                "selector_nc": (False, False),
            },
        },
        {
            "name": "radial_retraction_not_proved",
            "inputs": {
                "m0_all_retracted_zone": True,
                "winding_zone": False,
                "selector_gate_engaged_locked": False,
                "radial_nc": (False, False),
                "tangential_nc": (True, True),
                "gimbal_nc": (True, True),
                "selector_nc": (False, False),
            },
        },
        {
            "name": "radial_NC_channel_disagreement",
            "inputs": {
                "m0_all_retracted_zone": True,
                "winding_zone": False,
                "selector_gate_engaged_locked": False,
                "radial_nc": (True, False),
                "tangential_nc": (True, True),
                "gimbal_nc": (True, True),
                "selector_nc": (False, False),
            },
        },
        {
            "name": "selector_NC_channel_disagreement_during_winding",
            "inputs": {
                "m0_all_retracted_zone": False,
                "winding_zone": True,
                "selector_gate_engaged_locked": True,
                "radial_nc": (False, False),
                "tangential_nc": (False, False),
                "gimbal_nc": (False, False),
                "selector_nc": (True, False),
            },
        },
        {
            "name": "all_positions_proved_but_M0_zone_not_safe",
            "inputs": {
                "m0_all_retracted_zone": False,
                "winding_zone": False,
                "selector_gate_engaged_locked": False,
                "radial_nc": (True, True),
                "tangential_nc": (True, True),
                "gimbal_nc": (True, True),
                "selector_nc": (False, False),
            },
        },
        {
            "name": "selector_proved_outside_winding_zone",
            "inputs": {
                "m0_all_retracted_zone": False,
                "winding_zone": False,
                "selector_gate_engaged_locked": True,
                "radial_nc": (False, False),
                "tangential_nc": (False, False),
                "gimbal_nc": (False, False),
                "selector_nc": (True, True),
            },
        },
    ]
    evaluated = []
    for scenario in scenarios:
        result = evaluate_interlock(**scenario["inputs"])
        evaluated.append({
            "name": scenario["name"],
            "M1_enable": result["M1_enable"],
            "M2_enable": result["M2_enable"],
            "system_channels_healthy": result["system_channels_healthy"],
            "all_retracted_permissive": result["all_retracted_permissive"],
            "selector_seated_permissive": result[
                "selector_seated_permissive"
            ],
        })
    return {
        "status": "LOGIC_TOPOLOGY_ONLY_CIRCUIT_UNVALIDATED",
        "sensor_requirements": {
            "radial_actual_position": f"x_radial<={RADIAL_PROOF_LIMIT_MM} mm",
            "tangential_actual_position": (
                f"abs(y_tangential)<={TANGENTIAL_PROOF_LIMIT_MM} mm"
            ),
            "gimbal_actual_position": "positive dock seated; angle TBD",
            "sensed_features": "broad actual slide/yoke/dock faces, not links",
            "contacts": (
                "dual positive-opening NC, de-energize-to-trip, with EDM and "
                "channel-discrepancy latching"
            ),
            "M0_encoder_is_permissive_not_position_authority": True,
        },
        "gate_equations": {
            "all_retracted": (
                "channels_healthy AND M0_all_retracted_zone AND "
                "radial_A&B AND tangential_A&B AND gimbal_A&B"
            ),
            "selector_seated": (
                "channels_healthy AND winding_zone AND engaged_locked AND "
                "selector_A&B"
            ),
            "M1_enable": "all_retracted",
            "M2_enable": "all_retracted OR selector_seated",
            "OR_branch_requirement": (
                "both branches independently dual-monitored; a raw stuck-closed "
                "OR contact is not acceptable"
            ),
        },
        "dual_NC_pair_truth_table": pair_truth_table,
        "system_truth_table": evaluated,
        "expected_enabled_scenarios": {
            "M1": ["home_all_actual_positions_proved"],
            "M2": [
                "home_all_actual_positions_proved",
                "winding_selector_seated",
            ],
        },
        "exact_switch_SKUs_selected": False,
        "safety_relay_schematic_validated": False,
        "fault_injection_complete": False,
    }


def _failure_modes() -> list[dict[str, str]]:
    return [
        {
            "fault": "LEM spring or hook breaks",
            "mechanical_response": (
                "independent direct return retracts; M0 closed cam positively "
                "retracts and holds"
            ),
            "interlock_response": "actual-position NC must prove retracted",
        },
        {
            "fault": "bellcrank pin, slot, or lever breaks",
            "mechanical_response": (
                "independent direct return bypasses link; M0 cam remains direct"
            ),
            "interlock_response": "actual-position NC must prove retracted",
        },
        {
            "fault": "independent return breaks",
            "mechanical_response": "LEM return remains; M0 cam remains direct",
            "interlock_response": "actual-position NC must prove retracted",
        },
        {
            "fault": "one M0 roller or rail fails",
            "mechanical_response": "second direct roller/rail retains path",
            "interlock_response": "failure to reach proof opens enable chain",
        },
        {
            "fault": "both M0 rails fail or radial slide jams",
            "mechanical_response": "automatic physical retraction not guaranteed",
            "interlock_response": "M1 and M2 inhibited; safe stop only",
        },
        {
            "fault": "one tangential centering spring breaks",
            "mechanical_response": "captured T-slot enters positive M0 V-dock",
            "interlock_response": "tangential dual NC must prove centered",
        },
        {
            "fault": "tangential bushing seizes",
            "mechanical_response": "positive dock may not center",
            "interlock_response": "M1 and M2 inhibited; safe stop only",
        },
        {
            "fault": "one NC wire opens",
            "mechanical_response": "none",
            "interlock_response": "de-energize-to-trip channel disables motion",
        },
        {
            "fault": "one NC contact welds closed",
            "mechanical_response": "none",
            "interlock_response": (
                "second channel plus EDM/discrepancy must latch the fault"
            ),
        },
        {
            "fault": "power loss during winding",
            "mechanical_response": "axes stop; passive return remains available",
            "interlock_response": (
                "restart retracts M0 first and re-proves every actual-position "
                "channel before M1 or M2"
            ),
        },
    ]


def build_report() -> dict[str, Any]:
    radial = radial_topology()
    tangential = tangential_topology()
    m0 = m0_retraction_topology()
    interlock = interlock_topology()
    analysis_gates = {
        "radial_force_and_spring_envelope_analytically_feasible": all(
            radial["analytical_gates"].values()
        ),
        "tangential_target_envelope_analytically_feasible": all(
            tangential["analytical_gates"].values()
        ),
        "M0_cam_and_dwell_equations_close": all(
            m0["analytical_gates"].values()
        ),
        "dual_NC_truth_table_fail_closed": all(
            row["safe_position_proved"]
            == (row["channel_A_closed"] and row["channel_B_closed"])
            for row in interlock["dual_NC_pair_truth_table"]
        ),
    }
    physical_authority_gates = {
        "bellcrank_and_anchors_integrated_in_CAD": False,
        "independent_radial_return_selected_and_measured": False,
        "tangential_shaft_bushing_and_springs_selected": False,
        "tangential_bushing_retention_in_monolithic_cartridge_proven": False,
        "dual_M0_rails_and_full_machine_Z77_dwell_integrated_in_CAD": False,
        "positive_gimbal_neutral_dock_integrated": False,
        "dual_NC_switches_and_safety_relay_circuit_validated": False,
        "full_state_collision_sweep_complete": False,
        "40N_primary_load_path_validation_complete": False,
        "spring_fatigue_and_breakaway_tests_complete": False,
        "single_fault_hardware_injection_complete": False,
    }
    report = {
        "schema": SCHEMA,
        "status": "DESIGN_ANALYSIS_ONLY_FAIL_CLOSED",
        "scope": (
            "isolated deterministic radial/tangential/M0 retraction and "
            "actual-position interlock topology"
        ),
        "radial": radial,
        "tangential": tangential,
        "M0_positive_retraction": m0,
        "actual_position_interlock": interlock,
        "single_fault_responses": _failure_modes(),
        "analysis_gates": analysis_gates,
        "physical_authority_gates": physical_authority_gates,
        "physical_authority": False,
        "CAD_integration_authorized": False,
        "assembly_integration_authorized": False,
        "player_integration_authorized": False,
        "BOM_change_authorized": False,
        "procurement_authorized": False,
        "release_authorized": False,
        "required_next_evidence": [
            "positive-volume CAD for pivot clevis, slide ear and both spring anchors",
            "selected independent return, bushing, shaft and tangential springs",
            "integrated twin M0 ramp/closed-dwell rails through machine Z=77 mm",
            "positive tangential and gimbal docks with actual-position switch faces",
            "all-state mesh clearance and full-load attachment proof",
            "breakaway, fatigue, switch-circuit and injected single-fault tests",
        ],
        "source_bindings": {
            str(FOLLOWER_CAD_SOURCE.relative_to(ROOT)).replace("\\", "/"): (
                _sha256(FOLLOWER_CAD_SOURCE)
            ),
            str(M0_GATE_SOURCE.relative_to(ROOT)).replace("\\", "/"): (
                _sha256(M0_GATE_SOURCE)
            ),
            str(CONTROLLER_SOURCE.relative_to(ROOT)).replace("\\", "/"): (
                _sha256(CONTROLLER_SOURCE)
            ),
            str(Path(__file__).resolve().relative_to(ROOT)).replace("\\", "/"): (
                _sha256(Path(__file__).resolve())
            ),
        },
        "decision": (
            "The equations define a feasible fail-closed topology, but no "
            "physical mechanism or safety authority exists until every physical "
            "gate is closed. Do not integrate, order, wind, or release from this "
            "analysis."
        ),
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def validate_report_integrity(report: dict[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported follower retraction topology schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("follower retraction topology report hash mismatch")
    for relative, expected in report.get("source_bindings", {}).items():
        path = ROOT / Path(str(relative).replace("\\", "/"))
        if not path.is_file() or _sha256(path) != expected:
            raise ValueError(f"stale retraction topology source {relative}")


def _markdown(report: dict[str, Any]) -> str:
    radial = report["radial"]
    tangential = report["tangential"]
    m0 = report["M0_positive_retraction"]
    interlock = report["actual_position_interlock"]
    force_rows = radial["LEM050AB01"]["force_rows"]
    truth_rows = interlock["system_truth_table"]
    lines = [
        "# Aggregate-boundary follower retraction topology",
        "",
        f"- Status: **{report['status']}**",
        "- Physical authority: **false**",
        "- Assembly/player/BOM/release integration: **not authorized**",
        "",
        "## Radial bellcrank and independent return",
        "",
        "- Pivot P: `(17.000, 26.000, 20.000)` local, axis +Z.",
        "- Moving pin S(x): `(x_radial, 6.000, 20.000)` local, axis +Z.",
        "- Arms: 20.0 mm nominal long arm and 5.8 mm short arm.",
        "- Fixed LEM anchor F: `(22.727156, 15.583655, 20.000)` local.",
        "- Moving anchor: `P + 5.8*(20/L, (x-17)/L, 0)`.",
        "- Independent direct slide-to-carrier return: 0.25 N over 6.4 mm; "
        "measured breakaway must be <=0.125 N.",
        "",
        "| radial x (mm) | spring length (mm) | ratio | LEM output (N) | "
        "combined inward (N) |",
        "|---:|---:|---:|---:|---:|",
    ]
    for row in force_rows:
        lines.append(
            f"| {row['radial_center_mm']:.1f} | "
            f"{row['spring_length_mm']:.6f} | "
            f"{row['instantaneous_motion_ratio']:.6f} | "
            f"{row['LEM_follower_force_N']:.6f} | "
            f"{row['combined_inward_contact_force_N']:.6f} |"
        )
    lines.extend([
        "",
        "## Tangential bearing and opposed centering",
        "",
        "- Fixed ground shaft: Ø3 x about 16 mm, +Y axis at "
        "`(x_radial, 0, 19)`.",
        "- Moving dry-running flanged bushing target: ID3 / OD5 / L6 mm.",
        "- Opposed springs: free 5.5 mm, installed 4.5 mm, 0.15 N/mm each.",
        f"- Net centering stiffness: "
        f"{tangential['opposed_centering_springs']['net_centering_stiffness_N_per_mm']:.2f} N/mm; "
        "restoring force 0.15 N at ±0.5 mm.",
        "- Exact bushing, shaft, and spring SKUs remain unselected.",
        "",
        "## Positive M0 retraction",
        "",
        "- Two direct slide rollers and two closed fixed rails; the safety path "
        "does not pass through the bellcrank.",
        "- Ramp law: `x=20.2-1.5*(axis_z-24.5)`, clamped at x=14.0 mm.",
        f"- Retraction completes at axis Z="
        f"{m0['radial_positive_retraction']['retraction_complete_axis_z_mm']:.6f} mm, "
        f"leaving {m0['radial_positive_retraction']['positive_dwell_before_API_boundary_mm']:.6f} mm "
        "before the Z=29 API boundary.",
        f"- Fixed ramp spans machine Z="
        f"{m0['radial_positive_retraction']['ramp_start_machine_z_mm']:.3f} to "
        f"{m0['radial_positive_retraction']['ramp_end_machine_z_mm']:.3f} mm.",
        "- Closed dwell continues through machine Z=77.000 mm / M0 home "
        "axis Z=95.000 mm.",
        "- Tangential V-dock accepts |y|<=0.6 mm and proves |y|<=0.05 mm.",
        "- Positive gimbal neutral docking remains unresolved.",
        "",
        "## Dual-NC actual-position interlock",
        "",
        "`M1 = all_retracted`",
        "",
        "`M2 = all_retracted OR selector_seated`",
        "",
        "Every position uses two positive-opening NC channels. Any channel "
        "disagreement latches a fault; encoder position alone is not authority.",
        "",
        "| scenario | M1 | M2 | channels healthy |",
        "|---|:---:|:---:|:---:|",
    ])
    for row in truth_rows:
        lines.append(
            f"| {row['name']} | {str(row['M1_enable']).lower()} | "
            f"{str(row['M2_enable']).lower()} | "
            f"{str(row['system_channels_healthy']).lower()} |"
        )
    lines.extend([
        "",
        "## Authority boundary",
        "",
    ])
    for gate, value in report["physical_authority_gates"].items():
        lines.append(f"- `{gate}`: **{str(value).lower()}**")
    lines.extend([
        "",
        report["decision"],
        "",
    ])
    return "\n".join(lines)


def write_reports() -> dict[str, Any]:
    report = build_report()
    validate_report_integrity(report)
    REPORTS.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    OUTPUT_MD.write_text(_markdown(report), encoding="utf-8")
    return report


if __name__ == "__main__":
    result = write_reports()
    print(f"{result['status']}: {OUTPUT_JSON}")
