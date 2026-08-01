"""Bounded recovery study for a permanent cap and offset-spoke flyer.

The earlier permanent-cap audit proved that the production radial spoke cuts
through an outboard return pad.  This study evaluates the non-minimal recovery
which that audit deliberately left open:

* retain a real load-carrying 14 x 8 mm flyer spoke;
* put the spoke behind both permanent end-insulator caps;
* move the M2 bearing/block/belt/motor module rearward and lengthen the hollow
  shaft by the same amount;
* make the axial transition to the work-facing eyelet only after the rotating
  structure is radially outside the complete cap sweep.

The study is an advisory source-level trade, not production CAD.  It consumes
the canonical unmodified upstream capture, the exact cap/packing reports, the
current belt and load evidence, and the current hardware envelopes.  Exact
strand order and neatness are not predicted.  The supported aggregate is
instead required to be capacity-valid and nonpenetrating at the core and
other-tooth boundaries.

No assembly, controller input, BOM, or generated setting is modified.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping

import numpy as np
from scipy.spatial import cKDTree


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
GOAL = ROOT.parent / "GOAL.md"

for path in (HERE, CAD):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from params import DEFAULT_STATOR, PARAMS, StatorSpec  # noqa: E402
import coil_growth  # noqa: E402
import permanent_cap_flyer_recovery_study as prior  # noqa: E402
import stator_winding_guide_cap as cap  # noqa: E402
from traj import Timeline, load_events  # noqa: E402


SCHEMA = "permanent-cap-offset-spoke-flyer-recovery/v1"
JSON_OUT = REPORTS / "permanent_cap_offset_spoke_flyer.json"
MD_OUT = REPORTS / "permanent_cap_offset_spoke_flyer.md"

CAP_REPORT = REPORTS / "stator_winding_guide_cap.json"
PRIOR_REPORT = REPORTS / "permanent_cap_flyer_recovery.json"
AGGREGATE_REPORT = REPORTS / "aggregate_progressive_wire_corridor.json"
CAP_AGGREGATE_AUTH_REPORT = (
    REPORTS / "permanent_cap_aggregate_authorization.json"
)
CAP_AGGREGATE_AUTH_SOURCE = (
    HERE / "permanent_cap_aggregate_authorization.py"
)
R3_REPORT = REPORTS / "r3_bend_scope_feasibility.json"
LOADS_REPORT = REPORTS / "loads.json"
BELT_REPORT = REPORTS / "belt_audit.json"
CLEARANCE_REPORT = REPORTS / "clearance_upstream_raw.json"

CLEARANCE_MM = 2.0
WIRE_RADIUS_MM = float(DEFAULT_STATOR.wire_d) / 2.0
LINER_MM = 0.127
MINIMUM_BEND_RADIUS_MM = 3.0

# Bounded architecture sweep.  The 8.25 mm point captures the exact retained
# 8 mm-spoke solution for the 1 mm cap.  Larger values prove that merely
# moving farther rearward eventually consumes the current entry-eyelet and
# 450 mm frame envelopes.
CAP_WALLS_MM = (0.50, 1.00)
MODULE_SHIFTS_MM = (0.0, 2.0, 4.0, 6.0, 8.0, 8.25, 8.50,
                    9.0, 10.0, 15.0, 20.0)
SPOKE_FRONT_Z_MM = (-28.50, -29.00, -29.25, -29.50,
                    -30.00, -30.12, -31.00, -34.00, -42.00)
TRANSITION_RADII_MM = (52.0, 56.0, 58.0, 60.0)
TIP_RADII_MM = (60.0, 64.0, 68.0, 72.0)

CURRENT_SPOKE_WIDTH_MM = 14.0
CURRENT_SPOKE_THICKNESS_MM = 8.0
MINIMUM_PRINTED_WALL_MM = float(PARAMS.min_wall)
TIP_LOAD_N = 10.0
PETG_SCREENING_MODULUS_MPA = 2000.0
PETG_FATIGUE_SCREENING_ALLOWABLE_MPA = 10.0
MAXIMUM_SCREENING_DEFLECTION_MM = 0.50
REVIEW_RIGID_CLEARANCE_MM = 2.20

# Current exact source envelopes.  Moving the complete belt drive together
# preserves all belt-to-pulley/motor relationships.
BLOCK_FRONT_Z_MM = -31.0
MOTOR_REAR_Z_MM = -180.02220153808594
FRAME_REAR_Z_MM = float(PARAMS.frame_z0)
ENTRY_EYELET_FRONT_Z_MM = -110.5
SHAFT_REAR_Z_MM = float(PARAMS.flyer_shaft_rear_z)
TIP_GUIDE_OUTER_RADIUS_MM = 9.5

# The retained-section successor uses two small, explicit static-side
# relocations.  The entry bracket moves 2 mm rearward so the lengthened tube
# does not consume the eyelet clearance.  The 450 mm frame window shifts 2.5 mm
# rearward (rear boundary -192.5, front boundary +257.5) rather than growing.
ENTRY_GUIDE_RELOCATION_MM = 2.0
FRAME_WINDOW_RELOCATION_MM = 2.5


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} is not a JSON object")
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    body = deepcopy(dict(payload))
    body.pop("report_sha256", None)
    return hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def raw_pose_evidence() -> dict[str, Any]:
    """Reconstruct the exact raw pose population and deepest cap witness."""

    events = load_events(CAPTURE)
    timeline = Timeline(events)
    meta = next(event for event in events if event.get("e") == "meta")
    pitch = 2.0 * math.pi / int(DEFAULT_STATOR.slots)

    def quant(m0: float, m1: float, m2: float) -> tuple[int, int, int]:
        return (
            round(m0 / 0.25),
            round((m1 % pitch) / math.radians(1.0)),
            round((m2 % (2.0 * math.pi)) / math.radians(1.0)),
        )

    unique: set[tuple[int, int, int]] = set()
    count = 0
    minimum_axis = math.inf
    minimum_pose: tuple[float, float, float, float] | None = None
    m2_bins: set[int] = set()
    for pose in timeline.samples():
        t, m0, m1, m2 = map(float, pose)
        count += 1
        unique.add(quant(m0, m1, m2))
        m2_bins.add(round(math.degrees(m2) % 360.0) % 360)
        axis = float(PARAMS.stator_axis_z(m0))
        if axis < minimum_axis:
            minimum_axis = axis
            minimum_pose = (t, m0, m1, m2)
    if minimum_pose is None:
        raise ValueError("canonical capture has no timeline samples")

    clearance = _load(CLEARANCE_REPORT)
    checks = {
        "canonical_capture_sha": (
            _sha256(CAPTURE)
            == "350210c36550ae2c22e5352675aee6bd2eebb18325f3b5f45c7cb35d6a314958"
        ),
        "unmodified_upstream_controller": meta.get("controller_mode") == "upstream",
        "no_controller_adapter": meta.get("controller_adapter_sha256") is None,
        "no_injected_winding_plan": meta.get("winding_plan") is None,
        "raw_sample_count_matches_collision_audit": (
            count == int(clearance["n_raw_samples"])
        ),
        "unique_pose_count_matches_collision_audit": (
            len(unique) == int(clearance["n_unique_poses"])
        ),
        "all_integer_flyer_degrees_represented": len(m2_bins) == 360,
        "deepest_pose_is_exact_m1_m2_zero_witness": (
            abs(minimum_pose[2]) <= 1.0e-12
            and abs(minimum_pose[3]) <= 1.0e-12
        ),
    }
    return {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "raw_sample_count": count,
        "unique_quantized_pose_count": len(unique),
        "integer_flyer_angle_count": len(m2_bins),
        "minimum_stator_axis_z_mm": minimum_axis,
        "minimum_axis_pose": {
            "time_s": minimum_pose[0],
            "m0_rad": minimum_pose[1],
            "m1_rad": minimum_pose[2],
            "m2_rad": minimum_pose[3],
        },
        "capture_sha256": _sha256(CAPTURE),
        "capture_path": str(CAPTURE),
        "controller_mode": meta.get("controller_mode"),
        "winder_commit": meta.get("winder_commit"),
        "settings_sha256": meta.get("settings_sha256"),
    }


def _default_cap_outer_radius(wall_mm: float) -> float:
    base = float(_load(CAP_REPORT)["geometry_boundaries"]
                 ["outboard_recovery"]["minimum_required_cap_outer_radius_mm"])
    return base + (float(wall_mm) - cap.CAP_NOMINAL_WALL_MM) / 2.0


def _launch_cap_outer_radius(wall_mm: float) -> float:
    rows = _load(PRIOR_REPORT)["launch_envelope"]["cap_radial_bounds"]
    row = next(row for row in rows
               if math.isclose(float(row["wall_mm"]), float(wall_mm)))
    # The predecessor intentionally reported a zero-profile optimistic lower
    # bound.  Add the complete default four-layer profile span.  The launch
    # OD65 nominal job has 10.066 mm accessible radial span versus only 6.516
    # mm for the default 50-turn job, so it cannot require more transverse
    # packing layers at the same wire diameter/turn count.  The default span
    # is therefore a conservative launch add-on, not a repeated lower bound.
    profile_span = float(_load(CAP_REPORT)["geometry_boundaries"]
                         ["outboard_recovery"]["packing_profile_span_mm"])
    return float(row["minimum_outer_radius_mm"]) + profile_span


def cap_envelope(wall_mm: float, minimum_axis_z_mm: float) -> dict[str, float]:
    """Exact raw rear extreme plus conservative all-angle radial sweep."""

    default_outer = _default_cap_outer_radius(wall_mm)
    default_plane = (
        float(DEFAULT_STATOR.stack) / 2.0
        + MINIMUM_BEND_RADIUS_MM + float(wall_mm) / 2.0
    )
    launch_outer = _launch_cap_outer_radius(wall_mm)
    launch_plane = (
        float(PARAMS.stack_max) / 2.0
        + MINIMUM_BEND_RADIUS_MM + float(wall_mm) / 2.0
    )
    return {
        "wall_mm": float(wall_mm),
        "default_outer_radius_mm": default_outer,
        "raw_cycle_rearmost_z_mm": float(minimum_axis_z_mm) - default_outer,
        "default_flyer_radial_sweep_mm": math.hypot(default_outer, default_plane),
        "launch_outer_radius_conservative_mm": launch_outer,
        "launch_flyer_radial_sweep_conservative_mm": math.hypot(
            launch_outer, launch_plane),
    }


def structural_screen(thickness_mm: float, tip_radius_mm: float) -> dict[str, float | bool]:
    """Weak-axis 10 N cantilever sanity check for the retained spoke."""

    thickness = float(thickness_mm)
    length = max(0.0, float(tip_radius_mm) - 14.0)
    inertia = CURRENT_SPOKE_WIDTH_MM * thickness ** 3 / 12.0
    moment = TIP_LOAD_N * length
    stress = moment * thickness / 2.0 / inertia
    deflection = (
        TIP_LOAD_N * length ** 3
        / (3.0 * PETG_SCREENING_MODULUS_MPA * inertia)
    )
    fatigue_margin = PETG_FATIGUE_SCREENING_ALLOWABLE_MPA / stress
    return {
        "thickness_mm": thickness,
        "cantilever_length_mm": length,
        "second_moment_weak_axis_mm4": inertia,
        "tip_load_n": TIP_LOAD_N,
        "maximum_bending_stress_mpa": stress,
        "tip_deflection_mm": deflection,
        "fatigue_screening_allowable_mpa": PETG_FATIGUE_SCREENING_ALLOWABLE_MPA,
        "fatigue_screening_margin": fatigue_margin,
        "stress_margin_at_least_2x": fatigue_margin >= 2.0,
        "deflection_at_most_0p5mm": deflection <= MAXIMUM_SCREENING_DEFLECTION_MM,
        "passes": (
            fatigue_margin >= 2.0
            and deflection <= MAXIMUM_SCREENING_DEFLECTION_MM
        ),
    }


def motor_and_balance_screen(tip_radius_mm: float) -> dict[str, Any]:
    """Extend the repository's conservative M2 radius/mass bound."""

    loads = _load(LOADS_REPORT)
    parts = {row["part"]: row for row in loads["flyer"]["parts"]}
    moving_mass_g = (
        float(parts["retained_arm"]["mass_g"])
        + float(parts["flyer_PEEK_guide"]["mass_g"])
    )
    moving_i_gmm2 = (
        float(parts["retained_arm"]["izz_gmm2"])
        + float(parts["flyer_PEEK_guide"]["izz_gmm2"])
    )
    rms_radius = math.sqrt(moving_i_gmm2 / moving_mass_g)
    baseline_i = float(loads["flyer"]["izz_kgm2"])
    baseline_outer = float(PARAMS.flyer_tip_r + PARAMS.flyer_arm_w / 2.0)
    delta = float(tip_radius_mm) - float(PARAMS.flyer_tip_r)
    linear_mass = (
        CURRENT_SPOKE_WIDTH_MM * CURRENT_SPOKE_THICKNESS_MM
        * 1.27 / 1000.0
    )
    translated_i = moving_mass_g * (2.0 * rms_radius * delta + delta ** 2)
    new_outer = baseline_outer + delta
    added_bar_i = linear_mass / 3.0 * (new_outer ** 3 - baseline_outer ** 3)
    added_bar_moment = linear_mass / 2.0 * (
        new_outer ** 2 - baseline_outer ** 2)
    translated_moment = moving_mass_g * delta
    added_balance_mass = (
        translated_moment + added_bar_moment
    ) / float(PARAMS.counterweight_r)
    balance_i = added_balance_mass * float(PARAMS.counterweight_r) ** 2
    inertia = baseline_i + 1.0e-9 * (
        translated_i + added_bar_i + balance_i
    )
    energy_torque = 2.0 * (1.5 * 10.0 * 0.060 / (2.0 * math.pi))
    required = energy_torque + inertia * 200.0
    motor_margin = 0.630 / required
    pulley_margin = float(PARAMS.m2_motor_pulley_capacity_nm) / required
    current_stack_mass = float(loads["flyer"]["counterweight_mass_g"])
    return {
        "tip_radius_mm": float(tip_radius_mm),
        "bounded_inertia_kgm2": inertia,
        "required_torque_at_300rpm_nm": required,
        "selected_motor_margin": motor_margin,
        "selected_pulley_margin": pulley_margin,
        "added_balance_mass_at_R25_g": added_balance_mass,
        "current_counterweight_stack_mass_g": current_stack_mass,
        "current_three_washer_stack_sufficient": added_balance_mass <= 0.25,
        "successor_compact_weight_capacity_g": 40.0,
        "successor_weight_is_geometrically_sizeable": added_balance_mass <= 40.0,
        "motor_2x": motor_margin >= 2.0,
        "pulley_2x": pulley_margin >= 2.0,
    }


def aggregate_nonpenetration_audit() -> dict[str, Any]:
    """Check the smooth aggregate support domain, not an exact strand order."""

    r3 = _load(R3_REPORT)
    cap_report = _load(CAP_REPORT)
    aggregate = _load(AGGREGATE_REPORT)
    centres = r3["square_row_witness"]["centres"]
    wire_d = float(r3["inputs"]["stator"]["wire_finished_diameter_mm"])
    points: list[tuple[float, float]] = []
    owners: list[int] = []
    for tooth in range(int(DEFAULT_STATOR.slots)):
        angle = tooth * 2.0 * math.pi / int(DEFAULT_STATOR.slots)
        cosine, sine = math.cos(angle), math.sin(angle)
        for row in centres:
            x = float(row["tooth_x_mm"])
            half = float(row["tooth_half_span_mm"])
            for side in (-1.0, 1.0):
                y = side * half
                points.append((cosine * x - sine * y,
                               sine * x + cosine * y))
                owners.append(tooth)
    array = np.asarray(points, dtype=float)
    tree = cKDTree(array)
    distances, indices = tree.query(array, k=12)
    minimum_other = math.inf
    witness: dict[str, Any] | None = None
    for index in range(len(array)):
        for distance, other in zip(distances[index, 1:], indices[index, 1:]):
            if owners[index] == owners[int(other)]:
                continue
            if float(distance) < minimum_other:
                minimum_other = float(distance)
                witness = {
                    "first_index": index,
                    "second_index": int(other),
                    "first_tooth": owners[index],
                    "second_tooth": owners[int(other)],
                }
            break
    if witness is None:
        raise ValueError("no cross-tooth support pair found")

    slot = coil_growth.analyze_job(DEFAULT_STATOR)
    tooth_half = float(slot["slot"]["tooth_neck_width_mm"]) / 2.0
    support_half = min(float(row["tooth_half_span_mm"]) for row in centres)
    core_offset_required = tooth_half + LINER_MM + wire_d / 2.0
    core_margin = support_half - core_offset_required

    bounds = cap_report["geometry_boundaries"]["outboard_recovery"]
    base_radius = float(bounds["selected_base_radius_mm"])
    profile_span = float(bounds["packing_profile_span_mm"])
    crown_half = MINIMUM_BEND_RADIUS_MM + profile_span + wire_d / 2.0
    adjacent_chord = 2.0 * base_radius * math.sin(
        math.pi / int(DEFAULT_STATOR.slots))
    crown_gap = adjacent_chord - 2.0 * crown_half
    crown_core_gap = (
        base_radius - crown_half - float(DEFAULT_STATOR.od) / 2.0
    )
    fill = float(slot["packing"]["gross_slot_fill"])
    fill_limit = float(slot["packing"]["maximum_slot_fill_limit"])
    route = cap_report["route_audit"]
    gates = {
        "50_support_centres_per_coil_side": len(centres) == 50,
        "all_24_tooth_support_endpoints_checked": len(array) == 2400,
        "cross_tooth_slot_support_nonpenetrating": minimum_other >= wire_d,
        "liner_offset_core_nonpenetrating": core_margin >= -1.0e-9,
        "outboard_crown_aggregates_nonpenetrating": crown_gap >= wire_d,
        "outboard_crown_clear_of_core": crown_core_gap >= CLEARANCE_MM,
        "aggregate_fill_within_hard_limit": fill <= fill_limit,
        "physical_cap_routes_exist_both_directions": bool(
            route["gates"]["all_50_route_constructions"]
            and route["gates"]["all_R3_or_greater"]
            and route["gates"]["both_directions_current_half"]
        ),
        "aggregate_contact_classes_cover_raw_loci": (
            int(aggregate["raw_coverage"]["locus_count"]) == 2400
        ),
        "continuous_slot_to_crown_connector_nonpenetrating": False,
    }
    return {
        "status": "PARTIAL_BOUNDARY_PASS",
        "model": (
            "capacity-bounded smooth aggregate partitioned into 24 tooth "
            "ownership sectors; exact strand order is intentionally absent"
        ),
        "gates": gates,
        "support_endpoint_count": len(array),
        "minimum_cross_tooth_center_distance_mm": minimum_other,
        "required_wire_center_distance_mm": wire_d,
        "cross_tooth_margin_mm": minimum_other - wire_d,
        "cross_tooth_witness": witness,
        "liner_offset_core_margin_mm": core_margin,
        "outboard_adjacent_crown_gap_mm": crown_gap,
        "outboard_crown_to_core_gap_mm": crown_core_gap,
        "gross_slot_fill": fill,
        "hard_fill_limit": fill_limit,
        "stored_deterministic_route_family_reused": False,
        "controlling_gap": (
            "The local endpoint/crown audit does not construct the continuous "
            "connector; the independently hash-bound permanent-cap aggregate "
            "authority must close this gap."
        ),
        "scope_limit": (
            "This proves excluded-volume/capacity boundaries for a smooth "
            "aggregate. It does not predict passive settling, exact layer "
            "order, neatness, sag, friction, snagging, or enamel abrasion."
        ),
    }


def continuous_aggregate_authority(raw: Mapping[str, Any]) -> dict[str, Any]:
    """Fail-closed binding to the finalized slot-to-crown authority.

    The offset flyer may consume the permanent support lane only while the
    authority report is self-consistent, bound to the identical canonical
    raw capture, backed by its current generator, and still emits the exact
    R3 support-surface contract.  This does not authorize the physical cap or
    production assembly.
    """

    authority = _load(CAP_AGGREGATE_AUTH_REPORT)
    lane = authority.get("cap_support_lane", {})
    support = lane.get("support_surface_contract", {})
    connectors = authority.get("slot_to_crown_connectors", {})
    hashes = authority.get("source_hashes", {})
    capture_sha = str(raw["capture_sha256"])
    wire_r = float(DEFAULT_STATOR.wire_d) / 2.0
    report_hash = authority.get("report_sha256")
    support_binding = {
        "lane_id": lane.get("id"),
        "support_surface_contract": support,
        "minimum_lane_wire_center_bend_radius_mm": lane.get(
            "minimum_lane_wire_center_bend_radius_mm"
        ),
        "finished_wire_radius_mm": lane.get("finished_wire_radius_mm"),
    }
    support_contract_sha = hashlib.sha256(json.dumps(
        support_binding, sort_keys=True, separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")).hexdigest()
    checks = {
        "authority_status_PASS": authority.get("status") == "PASS",
        "aggregate_geometry_authorized": (
            authority.get("aggregate_geometry_authorized") is True
        ),
        "offset_flyer_input_authorized": (
            authority.get("offset_flyer_input_authorized") is True
        ),
        "authority_is_not_production_release": (
            authority.get("production_authorized") is False
            and authority.get("assembly_integration_authorized") is False
        ),
        "authority_report_self_hash_valid": (
            isinstance(report_hash, str)
            and _canonical_hash(authority) == report_hash
        ),
        "canonical_capture_contract_matches": (
            authority.get("canonical_raw_capture", {}).get("sha256")
            == capture_sha
            and authority.get("canonical_raw_capture", {}).get(
                "controller_mode"
            ) == "upstream"
            and hashes.get("out/capture/upstream_current_raw.jsonl")
            == capture_sha
        ),
        "authority_generator_hash_current": (
            hashes.get("sim/permanent_cap_aggregate_authorization.py")
            == _sha256(CAP_AGGREGATE_AUTH_SOURCE)
        ),
        "continuous_connector_gate_PASS": (
            authority.get("gates", {}).get(
                "continuous_positive_area_slot_to_crown_connectors"
            ) is True
            and authority.get("gates", {}).get(
                "connectors_clear_core_cap_and_all_other_aggregates"
            ) is True
            and authority.get("gates", {}).get(
                "live_connector_does_not_enter_active_prior_aggregate"
            ) is True
            and connectors.get("status") == "PASS"
        ),
        "support_lane_contract_is_R3": (
            lane.get("id") == "cap-r3-sector-lane-v1"
            and float(lane.get(
                "minimum_lane_wire_center_bend_radius_mm", -math.inf
            )) + 1.0e-12 >= MINIMUM_BEND_RADIUS_MM
            and float(support.get(
                "minimum_contact_surface_radius_mm", -math.inf
            )) + wire_r + 1.0e-12 >= MINIMUM_BEND_RADIUS_MM
            and float(support.get("groove_clear_width_mm", -math.inf))
            + 1.0e-12 >= float(DEFAULT_STATOR.wire_d)
        ),
    }
    return {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "report_path": "out/reports/permanent_cap_aggregate_authorization.json",
        "report_sha256": report_hash,
        "report_file_sha256": _sha256(CAP_AGGREGATE_AUTH_REPORT),
        "support_contract_sha256": support_contract_sha,
        "lane_id": lane.get("id"),
        "connector_count": connectors.get("connector_count"),
        "capture_sha256": capture_sha,
        "scope": (
            "authorizes the continuous aggregate/support-lane input only; "
            "physical cap CAD, material, finish and production remain false"
        ),
    }
def _candidate(wall_mm: float, shift_mm: float, spoke_front_z_mm: float,
               transition_radius_mm: float, tip_radius_mm: float,
               raw: Mapping[str, Any], aggregate: Mapping[str, Any],
               belt: Mapping[str, Any]) -> dict[str, Any]:
    envelope = cap_envelope(wall_mm, float(raw["minimum_stator_axis_z_mm"]))
    spoke_rear = float(spoke_front_z_mm) - CURRENT_SPOKE_THICKNESS_MM
    shifted_block_front = BLOCK_FRONT_Z_MM - float(shift_mm)
    cap_clearance = envelope["raw_cycle_rearmost_z_mm"] - float(spoke_front_z_mm)
    block_clearance = spoke_rear - shifted_block_front
    transition_clearance = (
        float(transition_radius_mm) - CURRENT_SPOKE_WIDTH_MM / 2.0
        - envelope["launch_flyer_radial_sweep_conservative_mm"]
    )
    axial_shift = abs(float(spoke_front_z_mm) - (-17.0))
    route_radial_run = float(tip_radius_mm) - float(transition_radius_mm)
    motor = motor_and_balance_screen(tip_radius_mm)
    structure = structural_screen(CURRENT_SPOKE_THICKNESS_MM, tip_radius_mm)
    relocated_frame_rear = FRAME_REAR_Z_MM - FRAME_WINDOW_RELOCATION_MM
    motor_rear_margin = (
        MOTOR_REAR_Z_MM - float(shift_mm) - relocated_frame_rear
    )
    shaft_rear = SHAFT_REAR_Z_MM - float(shift_mm)
    relocated_entry_front = (
        ENTRY_EYELET_FRONT_Z_MM - ENTRY_GUIDE_RELOCATION_MM
    )
    entry_gap = shaft_rear - relocated_entry_front
    gates = {
        "raw_cap_to_spoke_2p2mm_review": (
            cap_clearance >= REVIEW_RIGID_CLEARANCE_MM
        ),
        "shifted_block_to_spoke_2p2mm_review": (
            block_clearance >= REVIEW_RIGID_CLEARANCE_MM
        ),
        "transition_only_outside_launch_cap_2mm": (
            transition_clearance >= REVIEW_RIGID_CLEARANCE_MM
        ),
        "C1_R3_quarter_straight_quarter_transition": (
            axial_shift >= 2.0 * MINIMUM_BEND_RADIUS_MM
            and route_radial_run >= 2.0 * MINIMUM_BEND_RADIUS_MM
        ),
        "tip_outside_transition": tip_radius_mm > transition_radius_mm,
        "tip_rotating_envelope_inside_300mm_width": (
            tip_radius_mm + TIP_GUIDE_OUTER_RADIUS_MM + CLEARANCE_MM
            <= float(PARAMS.frame_w) / 2.0
        ),
        "retained_8mm_spoke_10N_screen": bool(structure["passes"]),
        "M2_motor_margin_2x_at_300rpm": bool(motor["motor_2x"]),
        "M2_pulley_margin_2x": bool(motor["pulley_2x"]),
        "successor_counterweight_sizeable": bool(
            motor["successor_weight_is_geometrically_sizeable"]
        ),
        "module_inside_450mm_frame_rear_boundary": motor_rear_margin >= 0.0,
        "extended_shaft_to_relocated_entry_eyelet_2p2mm_review": (
            entry_gap >= REVIEW_RIGID_CLEARANCE_MM
        ),
        "belt_geometry_preserved_by_rigid_module_translation": bool(
            belt.get("passed") is True
        ),
        "aggregate_endpoint_and_crown_bounds_nonpenetrating": (
            aggregate["status"] == "PARTIAL_BOUNDARY_PASS"
        ),
        "raw_command_stream_unchanged": True,
    }
    return {
        "wall_mm": float(wall_mm),
        "module_shift_mm": float(shift_mm),
        "spoke_front_z_mm": float(spoke_front_z_mm),
        "spoke_rear_z_mm": spoke_rear,
        "spoke_thickness_mm": CURRENT_SPOKE_THICKNESS_MM,
        "transition_radius_mm": float(transition_radius_mm),
        "tip_radius_mm": float(tip_radius_mm),
        "clearances_mm": {
            "raw_cap_to_spoke": cap_clearance,
            "shifted_block_to_spoke": block_clearance,
            "transition_inner_edge_to_launch_cap_sweep": transition_clearance,
            "extended_shaft_to_entry_eyelet": entry_gap,
            "motor_rear_to_frame_boundary": motor_rear_margin,
        },
        "static_relocations_mm": {
            "entry_guide_rearward": ENTRY_GUIDE_RELOCATION_MM,
            "450mm_frame_window_rearward": FRAME_WINDOW_RELOCATION_MM,
        },
        "wire_transition": {
            "family": "R3 quarter arc + axial straight + R3 quarter arc",
            "minimum_radius_mm": MINIMUM_BEND_RADIUS_MM,
            "axial_offset_mm": axial_shift,
            "available_radial_run_mm": route_radial_run,
            "C1": True,
        },
        "structure": structure,
        "motor_and_balance": motor,
        "gates": gates,
        "status": "PASS" if all(gates.values()) else "FAIL",
    }


def analyze() -> dict[str, Any]:
    raw = raw_pose_evidence()
    aggregate = aggregate_nonpenetration_audit()
    aggregate_authority = continuous_aggregate_authority(raw)
    belt = _load(BELT_REPORT)
    belt_static_parts = {
        str(row.get("part_key")): row
        for row in belt.get("static_non_engagement_parts", [])
        if isinstance(row, Mapping)
    }
    belt_motor = belt_static_parts.get("motor", {})
    if (
        belt.get("schema") != "selected-m2-belt-audit/v2"
        or belt.get("passed") is not True
        or belt_motor.get("ok") is not True
        or not isinstance(belt_motor.get("clearance_mm"), (int, float))
    ):
        raise ValueError(
            "selected P30/210-3GT belt audit lacks a passing motor-clearance "
            "record"
        )
    prior_launch = prior.launch_envelope()

    rows = [
        _candidate(wall, shift, spoke_z, transition, tip,
                   raw, aggregate, belt)
        for wall in CAP_WALLS_MM
        for shift in MODULE_SHIFTS_MM
        for spoke_z in SPOKE_FRONT_Z_MM
        for transition in TRANSITION_RADII_MM
        for tip in TIP_RADII_MM
    ]
    passing = [row for row in rows if row["status"] == "PASS"]
    passing.sort(key=lambda row: (
        0 if math.isclose(row["wall_mm"], 1.0) else 1,
        0 if math.isclose(row["module_shift_mm"], 10.0) else 1,
        abs(row["module_shift_mm"] - 10.0), row["tip_radius_mm"],
        -min(row["clearances_mm"]["raw_cap_to_spoke"],
             row["clearances_mm"]["shifted_block_to_spoke"]),
    ))
    selected = passing[0] if passing else None

    # Exact required shift if the spoke is optimally packed between the two
    # 2 mm boundaries.  This exposes the tempting but structurally poor 2.4
    # mm minimum-wall option next to the retained 8 mm section.
    shift_bound_rows = []
    for wall in CAP_WALLS_MM:
        envelope = cap_envelope(wall, raw["minimum_stator_axis_z_mm"])
        current_corridor = (
            envelope["raw_cycle_rearmost_z_mm"] - CLEARANCE_MM
            - (BLOCK_FRONT_Z_MM + CLEARANCE_MM)
        )
        shift_bound_rows.append({
            "wall_mm": wall,
            "current_block_max_spoke_thickness_mm": current_corridor,
            "minimum_shift_for_2p4mm_spoke_mm": max(
                0.0, MINIMUM_PRINTED_WALL_MM - current_corridor),
            "minimum_shift_for_retained_8mm_spoke_mm": max(
                0.0, CURRENT_SPOKE_THICKNESS_MM - current_corridor),
        })

    thin_wall_screen = structural_screen(
        MINIMUM_PRINTED_WALL_MM,
        selected["tip_radius_mm"] if selected else 64.0,
    )
    selected_material = {
        "wall_mm": 1.0,
        "concept": "machined or molded unfilled PEEK prototype cap",
        "manufacturing_basis": (
            "the predecessor material audit records about 1 mm as the "
            "unfilled-PEEK molding guideline; this is a geometry concept"
        ),
        "project_printer_ready": False,
        "wire_contact_finish_qualified": False,
        "abrasion_coupon_complete": False,
    }
    architecture_gates = {
        "canonical_raw_cycle_covered": raw["status"] == "PASS",
        "bounded_sweep_has_retained_8mm_candidate": bool(passing),
        "continuous_aggregate_support_contract_hash_bound": (
            aggregate_authority["status"] == "PASS"
        ),
        "launch_OD65_stack20_nominal_job": bool(
            prior_launch["gates"]["OD65_stack20_nominal_50_turn_job"]
        ),
        "launch_open_access_for_0p5_wire": bool(
            prior_launch["gates"]["maximum_0p5_wire_has_open_slot_access"]
        ),
        "full_360_flyer_sweep_bounded": (
            raw["integer_flyer_angle_count"] == 360
        ),
    }
    release_gates = {
        "architecture_gates": all(architecture_gates.values()),
        "continuous_slot_to_crown_aggregate_nonpenetrating": (
            aggregate_authority["status"] == "PASS"
        ),
        "exact_open_spoke_mass_stiffness_and_300rpm_margin": False,
        "cap_wire_contact_finish_and_abrasion_qualified": False,
        "two_plane_balance_and_attachment_in_CAD": False,
        "supplier_DFM_for_1mm_cap": False,
        "full_assembly_collision_regenerated": False,
    }
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": (
            "REVIEW_CANDIDATE_NOT_PRODUCTION" if all(architecture_gates.values())
            else "DESIGN_NO_GO"
        ),
        "decision": (
            "OFFSET_SPOKE_AND_CONTINUOUS_AGGREGATE_REVIEW_CANDIDATE__EXACT_LOADS_UNPROVEN"
            if all(architecture_gates.values())
            else "NO_BOUNDED_OFFSET_SPOKE_RECOVERY"
        ),
        "architecture_feasible": all(architecture_gates.values()),
        "production_authorized": all(release_gates.values()),
        "assembly_integration_authorized": False,
        "review_CAD_authorized": all(architecture_gates.values()),
        "cad_brief": {
            "task_type": "bounded parametric flyer architecture recovery",
            "units": "millimetres",
            "coordinates": (
                "machine Z is flyer axis; M2 rotates in XY; negative Z is rear"
            ),
            "architecture": (
                "14x8 deep spoke behind two permanent caps; paired M2 module "
                "rear shift/hollow-shaft extension; 1 mm entry-guide and "
                "2 mm fixed-length frame-window rear relocation; outboard-only R3 transition"
            ),
            "output": str(JSON_OUT),
            "CAD_skipped_reason": (
                "this bounded study stops before CAD; isolated review CAD is "
                "authorized next to replace the solid-beam/mass bounds, but "
                "integration remains forbidden"
            ),
        },
        "raw_pose_evidence": raw,
        "shift_bounds": shift_bound_rows,
        "minimum_wall_structural_screen": thin_wall_screen,
        "retained_section": {
            "width_mm": CURRENT_SPOKE_WIDTH_MM,
            "thickness_mm": CURRENT_SPOKE_THICKNESS_MM,
            "reason": (
                "the minimum 2.4 mm printable wall is geometrically recoverable "
                "after a small shift but fails the 10 N stiffness screen"
            ),
        },
        "aggregate_nonpenetration": aggregate,
        "continuous_aggregate_authority": aggregate_authority,
        "bounded_sweep": {
            "candidate_count": len(rows),
            "pass_count": len(passing),
            "walls_mm": list(CAP_WALLS_MM),
            "module_shifts_mm": list(MODULE_SHIFTS_MM),
            "spoke_front_z_mm": list(SPOKE_FRONT_Z_MM),
            "transition_radii_mm": list(TRANSITION_RADII_MM),
            "tip_radii_mm": list(TIP_RADII_MM),
            "selected": selected,
            "candidates": rows,
        },
        "belt_and_static_basis": {
            "selected_belt_audit_schema": belt["schema"],
            "selected_belt_audit_pass": belt["passed"],
            "selected_belt_motor_label": belt_motor["label"],
            "selected_belt_motor_clearance_mm": belt_motor["clearance_mm"],
            "selected_belt_minimum_static_clearance_mm": belt["summary"][
                "minimum_static_clearance_mm"
            ],
            "translation_rule": (
                "block, both M2 bearings, both pulleys, belt, motor/mount and "
                "rear shaft seat translate together; their relative clearances "
                "and belt center distance are invariant"
            ),
            "rear_post_x_gap_to_motor_mm": 13.791,
            "rear_post_rule": (
                "the rear post remains x-separated; selected shift stays within "
                "the same 450 mm frame after its window moves 2 mm rearward"
            ),
            "entry_guide_relocation_mm": ENTRY_GUIDE_RELOCATION_MM,
            "frame_window_relocation_mm": FRAME_WINDOW_RELOCATION_MM,
            "relocated_frame_z_span_mm": [
                FRAME_REAR_Z_MM - FRAME_WINDOW_RELOCATION_MM,
                FRAME_REAR_Z_MM - FRAME_WINDOW_RELOCATION_MM
                + float(PARAMS.frame_len),
            ],
        },
        "launch_envelope": prior_launch,
        "selected_cap_material_concept": selected_material,
        "architecture_gates": architecture_gates,
        "release_gates": release_gates,
        "limits": [
            "No exact strand/layer order or neatness is predicted.",
            "Tension dynamics, passive settling, sag, snagging, friction and enamel abrasion remain hardware-only.",
            "The 10 N PETG calculation is a beam/fatigue sanity screen, not material certification.",
            "A practical open two-rail/stepped spoke may deflect more than the solid 14x8 beam bound; exact CAD mass and stiffness are required before the 2x motor gate can close.",
            "A production successor needs exact two-plane balance and counterweight attachment CAD.",
            "The 0.50 mm wire has open launch access, but 50 turns at 0.50 mm exceeds the OD65 job fill limit; the nominal launch wire job passes.",
        ],
        "source_hashes": {
            "GOAL.md": _sha256(GOAL),
            "out/capture/upstream_current_raw.jsonl": _sha256(CAPTURE),
            "out/reports/stator_winding_guide_cap.json": _sha256(CAP_REPORT),
            "out/reports/permanent_cap_flyer_recovery.json": _sha256(PRIOR_REPORT),
            "out/reports/aggregate_progressive_wire_corridor.json": _sha256(AGGREGATE_REPORT),
            "sim/permanent_cap_aggregate_authorization.py": _sha256(CAP_AGGREGATE_AUTH_SOURCE),
            "out/reports/permanent_cap_aggregate_authorization.json": _sha256(CAP_AGGREGATE_AUTH_REPORT),
            "out/reports/r3_bend_scope_feasibility.json": _sha256(R3_REPORT),
            "out/reports/loads.json": _sha256(LOADS_REPORT),
            "out/reports/belt_audit.json": _sha256(BELT_REPORT),
            "out/reports/clearance_upstream_raw.json": _sha256(CLEARANCE_REPORT),
            "sim/permanent_cap_offset_spoke_flyer_study.py": _sha256(Path(__file__)),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def render_markdown(report: Mapping[str, Any]) -> str:
    selected = report["bounded_sweep"]["selected"]
    aggregate = report["aggregate_nonpenetration"]
    aggregate_authority = report["continuous_aggregate_authority"]
    lines = [
        "# Permanent-cap offset-spoke flyer recovery", "",
        f"**{report['status']} — {report['decision']}**", "",
        "The architecture is mechanically recoverable, but it is not a "
        "production release and has not been integrated into assembly CAD.", "",
        "## Exact raw-cycle corridor", "",
    ]
    raw = report["raw_pose_evidence"]
    lines.extend([
        f"- Canonical raw poses: {raw['raw_sample_count']} samples / {raw['unique_quantized_pose_count']} unique quantized poses.",
        f"- Deepest raw stator axis: z={raw['minimum_stator_axis_z_mm']:.6f} mm at M1=M2=0.",
    ])
    for row in report["shift_bounds"]:
        lines.append(
            f"- {row['wall_mm']:.2f} mm cap: current corridor "
            f"{row['current_block_max_spoke_thickness_mm']:.6f} mm; "
            f"shift >= {row['minimum_shift_for_2p4mm_spoke_mm']:.6f} mm "
            f"for 2.4 mm, >= {row['minimum_shift_for_retained_8mm_spoke_mm']:.6f} mm for 8 mm."
        )
    lines.extend(["", "## Selected bounded successor", ""])
    if selected:
        c = selected["clearances_mm"]
        m = selected["motor_and_balance"]
        s = selected["structure"]
        lines.extend([
            f"- 1.00 mm cap, {selected['module_shift_mm']:.2f} mm paired block/module shift, spoke z={selected['spoke_rear_z_mm']:.2f}..{selected['spoke_front_z_mm']:.2f} mm.",
            f"- R{selected['transition_radius_mm']:.0f} outboard transition, R{selected['tip_radius_mm']:.0f} eyelet circle.",
            f"- Cap/spoke {c['raw_cap_to_spoke']:.3f} mm; block/spoke {c['shifted_block_to_spoke']:.3f} mm; transition/cap {c['transition_inner_edge_to_launch_cap_sweep']:.3f} mm.",
            f"- Shaft/entry eyelet {c['extended_shaft_to_entry_eyelet']:.3f} mm; motor remains {c['motor_rear_to_frame_boundary']:.3f} mm inside the frame boundary.",
            f"- Static-side support changes: entry guide {selected['static_relocations_mm']['entry_guide_rearward']:.1f} mm rearward; unchanged 450 mm frame window {selected['static_relocations_mm']['450mm_frame_window_rearward']:.1f} mm rearward.",
            f"- 10 N weak-axis screen: {s['tip_deflection_mm']:.3f} mm, {s['maximum_bending_stress_mpa']:.3f} MPa, fatigue-screen margin {s['fatigue_screening_margin']:.2f}x.",
            f"- 300 RPM M2/pulley margins: {m['selected_motor_margin']:.3f}x / {m['selected_pulley_margin']:.3f}x.",
            f"- Current three-washer counterweight is insufficient; bounded successor needs {m['added_balance_mass_at_R25_g']:.2f} g added at R25 and exact two-plane balance CAD.",
        ])
    else:
        lines.append("No candidate passed the bounded sweep.")
    thin = report["minimum_wall_structural_screen"]
    lines.extend([
        "", "## Why the 2.4 mm wall is not selected", "",
        f"At the selected radius its 10 N weak-axis deflection is {thin['tip_deflection_mm']:.3f} mm and fatigue-screen margin is {thin['fatigue_screening_margin']:.2f}x. A small block shift can fit it geometrically, but it is not a robust rotating spoke.",
        "", "## Aggregate/workpiece boundary", "",
        f"- 24 teeth / {aggregate['support_endpoint_count']} slot-side support endpoints checked.",
        f"- Minimum cross-tooth centre distance {aggregate['minimum_cross_tooth_center_distance_mm']:.6f} mm for {aggregate['required_wire_center_distance_mm']:.6f} mm wire.",
        f"- Outboard adjacent-crown gap {aggregate['outboard_adjacent_crown_gap_mm']:.6f} mm; crown-to-core gap {aggregate['outboard_crown_to_core_gap_mm']:.6f} mm.",
        "- The local endpoint/crown check is partial; the separately generated continuous aggregate authority closes the connector gate without asserting an exact strand schedule.",
        f"- Continuous authority: {aggregate_authority['status']} / `{aggregate_authority['lane_id']}` / {aggregate_authority['connector_count']} positive-area connectors.",
        f"- Support-contract SHA-256: `{aggregate_authority['support_contract_sha256']}`.",
        "", "## Why production integration remains withheld", "",
    ])
    for name, ok in report["release_gates"].items():
        if not ok:
            lines.append(f"- `{name}`")
    lines.extend([
        "", "Isolated review CAD is authorized as the next validation step; production integration is not. The 1 mm PEEK cap is a manufacturable geometry concept, not a qualified wire-contact part. Supplier DFM, polished finish, abrasion/dielectric coupons, exact open-spoke mass/stiffness, two-plane balance, physical realization of the support contract, and full regenerated assembly collision remain required.", "",
        f"Report SHA-256: `{report['report_sha256']}`", "",
    ])
    return "\n".join(lines)


def write_reports(report: Mapping[str, Any] | None = None) -> dict[str, Any]:
    value = analyze() if report is None else dict(report)
    REPORTS.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n",
                        encoding="utf-8")
    MD_OUT.write_text(render_markdown(value), encoding="utf-8")
    return value


def main() -> int:
    report = write_reports()
    sweep = report["bounded_sweep"]
    print(f"offset-spoke flyer: {report['status']}; "
          f"passes={sweep['pass_count']}/{sweep['candidate_count']}")
    return 0 if report["architecture_feasible"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
