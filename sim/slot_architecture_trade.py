"""Fail-closed architecture trade for the active slot-mouth wire collision.

This study deliberately does not alter the release stator, packing, motion,
or route generators.  It combines their measured-input artifacts with the
same analytic tooth boundaries used by ``cad/stator_model.py`` to answer a
narrow question: what is the smallest *architecture* change that could remove
the bare-steel/deposited-current-half conflict without hiding it as a collision
exclusion?

The output is comparative evidence, not a release certificate.  In
particular, a geometrically credible guide stays ``CONCEPT_ONLY`` until one
exact route passes both flyer directions against the complete progressive
copper state, including the already-laid half of the current turn.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np
from scipy.optimize import linprog


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

import coil_growth  # noqa: E402
from params import DEFAULT_STATOR  # noqa: E402
import stator_insulation_nomex410 as insulation  # noqa: E402
import slot_packing_audit  # noqa: E402


SCHEMA = "slot-architecture-trade/v1"
OUTPUT_JSON = REPORTS / "slot_architecture_trade.json"
OUTPUT_MD = REPORTS / "slot_architecture_trade.md"

PACKING_PATH = REPORTS / "slot_packing.json"
FEASIBILITY_PATH = REPORTS / "flyer_slot_guide_feasibility.json"
ROUTE_PATHS = (
    REPORTS / "slot_crown_routes.json",
    REPORTS / "slot_half_twist_routes.json",
    REPORTS / "slot_radial_axial_routes.json",
)

CURRENT_SHOE_COVERAGE = 0.72
MINIMUM_CENTERLINE_BEND_RADIUS_MM = 3.0
MAXIMUM_ROUTE_WIRE_DIAMETER_MM = 0.50
ROUTE_WIRE_RADIUS_MM = MAXIMUM_ROUTE_WIRE_DIAMETER_MM / 2.0
RUNNING_CLEARANCE_MM = 0.10
REFERENCE_BLADE_THICKNESS_MM = 0.10
REFERENCE_CLOSED_WALL_MM = 0.15
ROBUST_CLOSED_WALL_MM = 0.25
RETRACTION_CLEARANCE_MM = 0.05
CORRIDOR_SAMPLE_COUNT = 257
DEPTH_SAMPLE_COUNT = 9


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(value: dict[str, Any]) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _pitch_rad() -> float:
    return 2.0 * math.pi / int(DEFAULT_STATOR.slots)


def _shoe_inner_radius_mm() -> float:
    return (
        float(DEFAULT_STATOR.od) / 2.0
        - max(1.6, float(DEFAULT_STATOR.od) * 0.045)
    )


def _tooth_boundaries(radius_mm: float) -> tuple[float, float]:
    """Return the two exact analytic steel walls of tooth-0/tooth-1 slot.

    This is the 2D section of the neck-box plus annular-sector construction in
    ``cad/stator_model.py``.  The more intrusive wall controls in the shoe
    band, exactly as it does in the source Boolean.
    """

    outer = float(DEFAULT_STATOR.od) / 2.0
    if not 0.0 < radius_mm <= outer + 1.0e-9:
        raise ValueError("radius is outside the stator radial envelope")
    pitch = _pitch_rad()
    half_neck = max(2.5, float(DEFAULT_STATOR.od) * 0.07) / 2.0
    shoe_inner = _shoe_inner_radius_mm()
    shoe_half_angle = CURRENT_SHOE_COVERAGE * pitch / 2.0

    active = half_neck
    neighbor = (
        -half_neck + math.sin(pitch) * radius_mm
    ) / math.cos(pitch)
    if radius_mm >= shoe_inner:
        active = max(active, radius_mm * math.tan(shoe_half_angle))
        neighbor = min(
            neighbor,
            radius_mm * math.tan(pitch - shoe_half_angle),
        )
    return float(active), float(neighbor)


def _opening_for_coverage(coverage: float) -> float:
    if not 0.0 <= coverage <= 1.0:
        raise ValueError("shoe coverage must be within [0, 1]")
    return float(
        2.0 * _shoe_inner_radius_mm()
        * math.sin((1.0 - coverage) * _pitch_rad() / 2.0)
    )


def _maximum_coverage_for_opening(opening_mm: float) -> float | None:
    radius = _shoe_inner_radius_mm()
    maximum = 2.0 * radius * math.sin(_pitch_rad() / 2.0)
    if opening_mm > maximum + 1.0e-12:
        return None
    return float(
        1.0
        - 2.0 * math.asin(opening_mm / (2.0 * radius)) / _pitch_rad()
    )


def _corridor_rows(blade_thickness_mm: float) -> tuple[
        np.ndarray, np.ndarray, list[tuple[int, int, float, float]]]:
    """Return fixed-world stations, M0 depths, and lined guide intervals."""

    job = coil_growth.analyze_job(DEFAULT_STATOR)["bundle"]
    radial_start = float(job["radial_winding_start_mm"])
    radial_end = float(job["radial_winding_end_mm"])
    span = radial_end - radial_start
    lay_world_z = 2.0
    stations = np.linspace(
        lay_world_z - span, lay_world_z, CORRIDOR_SAMPLE_COUNT,
    )
    depths = np.linspace(radial_start, radial_end, DEPTH_SAMPLE_COUNT)
    allowance = (
        float(insulation.MATERIAL_RECEIVING_MAX_MM)
        + float(blade_thickness_mm) / 2.0
    )
    rows: list[tuple[int, int, float, float]] = []
    shallow_axis_z = lay_world_z + radial_start
    outer = float(DEFAULT_STATOR.od) / 2.0
    for station_index, world_z in enumerate(stations):
        shallow_radius = shallow_axis_z - float(world_z)
        for depth_index, depth in enumerate(depths):
            radius = shallow_radius + float(depth) - radial_start
            if radius > outer + 1.0e-9:
                continue
            lower, upper = _tooth_boundaries(radius)
            rows.append((
                station_index,
                depth_index,
                lower + allowance,
                upper - allowance,
            ))
    return stations, depths, rows


def fixed_and_tracking_corridor(blade_thickness_mm: float) -> dict[str, Any]:
    """Audit a rigid fixed blade and an optimistic translated-blade lower bound.

    The linear program is intentionally generous: it gives every axial/radial
    station an independently shaped blade center and permits one rigid
    tangential offset per M0 depth.  Its stroke is therefore a lower bound,
    not an actuator specification.
    """

    stations, depths, rows = _corridor_rows(blade_thickness_mm)
    by_station: dict[int, list[tuple[float, float]]] = {}
    for station, _depth, lower, upper in rows:
        by_station.setdefault(station, []).append((lower, upper))
    fixed_margins = [
        min(upper for _lower, upper in values)
        - max(lower for lower, _upper in values)
        for values in by_station.values()
    ]
    minimum_fixed = float(min(fixed_margins))
    failed_stations = int(sum(value < -1.0e-12 for value in fixed_margins))

    nz = len(stations)
    nd = len(depths)
    lower_index = nz + nd
    upper_index = lower_index + 1
    variable_count = upper_index + 1
    inequalities: list[np.ndarray] = []
    limits: list[float] = []
    for station, depth, lower, upper in rows:
        row = np.zeros(variable_count)
        row[station] = 1.0
        row[nz + depth] = 1.0
        inequalities.append(row)
        limits.append(upper)
        row = np.zeros(variable_count)
        row[station] = -1.0
        row[nz + depth] = -1.0
        inequalities.append(row)
        limits.append(-lower)
    for depth in range(nd):
        row = np.zeros(variable_count)
        row[nz + depth] = 1.0
        row[upper_index] = -1.0
        inequalities.append(row)
        limits.append(0.0)
        row = np.zeros(variable_count)
        row[lower_index] = 1.0
        row[nz + depth] = -1.0
        inequalities.append(row)
        limits.append(0.0)

    equality = np.zeros((1, variable_count))
    equality[0, nz] = 1.0
    objective = np.zeros(variable_count)
    objective[upper_index] = 1.0
    objective[lower_index] = -1.0
    result = linprog(
        objective,
        A_ub=np.asarray(inequalities),
        b_ub=np.asarray(limits),
        A_eq=equality,
        b_eq=np.zeros(1),
        bounds=[(None, None)] * variable_count,
        method="highs",
    )
    offsets = (
        [float(value) for value in result.x[nz:nz + nd]]
        if result.success else None
    )
    return {
        "blade_thickness_mm": float(blade_thickness_mm),
        "liner_allowance_each_wall_mm": float(
            insulation.MATERIAL_RECEIVING_MAX_MM
            + blade_thickness_mm / 2.0
        ),
        "station_count": nz,
        "depth_count": nd,
        "fixed_blade": {
            "status": "PASS" if failed_stations == 0 else "FAIL",
            "failed_station_count": failed_stations,
            "minimum_common_corridor_margin_mm": minimum_fixed,
        },
        "optimistic_tangential_tracking": {
            "status": "GEOMETRIC_LOWER_BOUND" if result.success else "FAIL",
            "linear_program_solved": bool(result.success),
            "minimum_offset_range_mm": (
                float(result.fun) if result.success else None
            ),
            "depth_offsets_mm": offsets,
            "warning": (
                "This ignores guide continuity, extraction, copper, flyer, "
                "mount stiffness, and actuator error; real stroke must be larger."
            ),
        },
    }


def coverage_trade(feasibility: dict[str, Any]) -> dict[str, Any]:
    budget = feasibility["mouth_and_guide_budget"]
    current_bare = float(budget["slot"]["bare_mouth_mm"])
    analytic_bare = _opening_for_coverage(CURRENT_SHOE_COVERAGE)
    if abs(current_bare - analytic_bare) > 1.0e-9:
        raise RuntimeError("analytic shoe-opening model drifted from source report")

    cap_overlap = float(budget["slot"]["existing_cap_overlap_each_mm"])
    liner = float(budget["slot"]["liner_receiving_max_each_mm"])
    cases = []
    for label, wall, termination in (
        ("current_cap_0p15_wall", REFERENCE_CLOSED_WALL_MM, "current_cap"),
        ("current_cap_0p25_wall", ROBUST_CLOSED_WALL_MM, "current_cap"),
        ("local_cap_relief_0p25_wall", ROBUST_CLOSED_WALL_MM, "liner_only"),
    ):
        edge_loss = 2.0 * (cap_overlap if termination == "current_cap" else liner)
        required_bare = (
            edge_loss
            + MAXIMUM_ROUTE_WIRE_DIAMETER_MM
            + RUNNING_CLEARANCE_MM
            + 2.0 * wall
        )
        coverage = _maximum_coverage_for_opening(required_bare)
        cases.append({
            "case": label,
            "termination": termination,
            "closed_wall_each_mm": wall,
            "required_bare_mouth_mm": required_bare,
            "maximum_tooth_shoe_coverage_fraction": coverage,
            "coverage_reduction_from_current_fraction": (
                CURRENT_SHOE_COVERAGE - coverage
                if coverage is not None else None
            ),
            "current_geometry_has_width": current_bare + 1.0e-12 >= required_bare,
        })

    external_surface_radius = (
        MINIMUM_CENTERLINE_BEND_RADIUS_MM - ROUTE_WIRE_RADIUS_MM
    )
    pin_diameter = 2.0 * external_surface_radius
    inner_pitch = (
        2.0 * _shoe_inner_radius_mm() * math.sin(_pitch_rad() / 2.0)
    )
    outer_radius = float(DEFAULT_STATOR.od) / 2.0
    outer_pitch = 2.0 * outer_radius * math.sin(_pitch_rad() / 2.0)
    center_radius = pin_diameter / (2.0 * math.sin(_pitch_rad() / 2.0))
    return {
        "architecture": "change lamination tooth-shoe coverage / widen mouth",
        "status": "NO_GO_AS_SOLE_CHANGE",
        "current_coverage_fraction": CURRENT_SHOE_COVERAGE,
        "current_bare_mouth_mm": current_bare,
        "closed_nozzle_width_cases": cases,
        "three_mm_bend_boundary": {
            "required_wire_center_radius_mm": MINIMUM_CENTERLINE_BEND_RADIUS_MM,
            "external_guide_surface_radius_mm": external_surface_radius,
            "external_pin_diameter_mm": pin_diameter,
            "maximum_possible_inner_mouth_if_shoe_deleted_mm": inner_pitch,
            "inner_pitch_shortfall_mm": inner_pitch - pin_diameter,
            "coverage_needed_to_pass_pin": _maximum_coverage_for_opening(
                pin_diameter),
            "minimum_24_fold_pin_center_radius_mm": center_radius,
            "stator_od_pitch_chord_mm": outer_pitch,
            "stator_od_pin_gap_mm": outer_pitch - pin_diameter,
        },
        "progressive_current_half_proof": "FAIL_NOT_GENERATED",
        "both_flyer_directions": "FAIL_NOT_GENERATED",
        "new_actuator": False,
        "software_change": (
            "new stator identity, packing regeneration, and route/capture "
            "qualification; no new motion degree of freedom"
        ),
        "finding": (
            "A modest coverage cut can create closed-nozzle wall width, but "
            "even deleting the complete shoe leaves the inner 24-fold pitch "
            "too narrow for the R3 external pin.  The R3 turn must be axial/"
            "outboard, and mouth width alone does not separate the moving "
            "span from progressive/current-half copper."
        ),
    }


def reduction_trade(packing: dict[str, Any],
                    route_reports: list[tuple[Path, dict[str, Any]]]) -> dict[str, Any]:
    rows = packing["selected_schedule"]["side_positive"]
    first_deep = next(
        int(row["turn_index"]) for row in rows
        if int(row["layer_index"]) >= 3
    )
    original_turns = int(
        packing["selected_schedule"]["turns_per_tooth"]
    )
    maximum_without_layer_three = first_deep

    families = []
    for path, report in route_reports:
        current_distances = [
            float(case["continuous_lower_bound_mm"])
            for route in report.get("routes", [])
            for case in route.get("motion_sign_cases", [])
        ]
        if not current_distances:
            raise RuntimeError(f"{path} has no sign-specific current-half data")
        deep_distances = [
            float(case["continuous_lower_bound_mm"])
            for route in report["routes"]
            if int(route["turn_index"]) >= first_deep
            for case in route["motion_sign_cases"]
        ]
        first_failed = next((
            int(route["turn_index"])
            for route in report["routes"]
            if any(
                not bool(case["checks"].get(
                    "nonadjacent_current_half_known_budget", False))
                for case in route["motion_sign_cases"]
            )
        ), None)
        families.append({
            "path": path.relative_to(ROOT).as_posix(),
            "geometry_family": report["policy"]["geometry_family"],
            "report_status": report["status"],
            "minimum_current_half_lower_bound_all_turns_mm": min(
                current_distances),
            "minimum_current_half_lower_bound_layer3_mm": min(deep_distances),
            "first_turn_failing_current_half_budget": first_failed,
            "motion_sign_passed": int(
                report["validation"]["motion_sign_passed"]),
            "motion_sign_expected": int(
                report["validation"]["motion_sign_expected"]),
        })
    best_frozen = max(
        row["minimum_current_half_lower_bound_all_turns_mm"]
        for row in families
    )
    best_family = next(
        row["geometry_family"] for row in families
        if row["minimum_current_half_lower_bound_all_turns_mm"] == best_frozen
    )
    known_budget = min(
        float(report["known_physical_error_lower_bound_mm"])
        for _path, report in route_reports
    )
    diameter_after_budget = max(0.0, best_frozen - known_budget)
    nominal_wire = float(packing["config"]["wire_finished_diameter_mm"])
    receiving = packing["receiving_contract"][
        "wire_finished_diameter_range_mm"]
    return {
        "architecture": "reduce turns/depth or wire envelope",
        "status": "NO_GO_FOR_RELEASE_JOB",
        "turn_reduction": {
            "first_layer3_turn_index": first_deep,
            "maximum_turns_per_tooth_without_layer3": maximum_without_layer_three,
            "original_turns_per_tooth": original_turns,
            "turn_loss_count": original_turns - maximum_without_layer_three,
            "turn_loss_fraction": (
                (original_turns - maximum_without_layer_three) / original_turns
            ),
            "finding": (
                "Removing row 3 needs a <=46-turn job, but the sign-specific "
                "current-half conflict already appears at turn 0 in every "
                "tested route family; truncation does not cure the mechanism."
            ),
        },
        "wire_reduction_frozen_route_upper_bound": {
            "nominal_finished_diameter_mm": nominal_wire,
            "receiving_range_mm": receiving,
            "best_family": best_family,
            "maximum_diameter_before_known_budget_mm": best_frozen,
            "known_physical_error_lower_bound_mm": known_budget,
            "maximum_diameter_after_known_budget_mm": diameter_after_budget,
            "diameter_reduction_before_budget_fraction": 1.0 - best_frozen / nominal_wire,
            "diameter_reduction_after_budget_fraction": (
                1.0 - diameter_after_budget / nominal_wire
            ),
            "relative_finished_cross_section_after_budget": (
                (diameter_after_budget / nominal_wire) ** 2
            ),
            "warning": (
                "This is only a frozen-route no-go upper bound.  A changed "
                "wire diameter requires a regenerated packing and route; it "
                "cannot improve this conclusion into a release proof."
            ),
        },
        "route_families": families,
        "progressive_current_half_proof": "FAIL_IN_ALL_TESTED_FAMILIES",
        "both_flyer_directions": "FAIL_0_OF_200_IN_BEST_RELEVANT_FAMILIES",
        "new_actuator": False,
        "software_change": "new electrical job and complete packing/controller regeneration",
    }


def guide_trade(feasibility: dict[str, Any], packing: dict[str, Any]) -> dict[str, Any]:
    zero_blade = fixed_and_tracking_corridor(0.0)
    reference_blade = fixed_and_tracking_corridor(
        REFERENCE_BLADE_THICKNESS_MM)
    job = coil_growth.analyze_job(DEFAULT_STATOR)["bundle"]
    radial_start = float(job["radial_winding_start_mm"])
    lower, upper = _tooth_boundaries(radial_start)
    raw_inner_width = upper - lower
    liner = float(insulation.MATERIAL_RECEIVING_MAX_MM)
    lined_inner = raw_inner_width - 2.0 * liner
    packed_wire = float(packing["config"]["wire_finished_diameter_mm"])
    blade_alongside_wire = lined_inner - packed_wire

    budget = feasibility["mouth_and_guide_budget"]
    lined_mouth = float(budget["slot"]["lined_mouth_mm"])
    centered_clear_stroke = (
        lined_mouth / 2.0
        + REFERENCE_BLADE_THICKNESS_MM / 2.0
    )
    design_clear_stroke = centered_clear_stroke + RETRACTION_CLEARANCE_MM
    bend = budget["three_mm_bend_geometry"]
    active = budget["active_shoe_candidate"]

    fixed_carriage = {
        "architecture": "machine/carriage-fixed full-depth split blade",
        "status": "NO_GO",
        "zero_blade_corridor": zero_blade,
        "reference_0p10mm_blade_corridor": reference_blade,
        "narrowest_inner_slot": {
            "radial_station_mm": radial_start,
            "bare_wall_to_wall_mm": raw_inner_width,
            "lined_wall_to_wall_mm": lined_inner,
            "packed_finished_wire_mm": packed_wire,
            "maximum_remaining_thickness_alongside_wire_mm": blade_alongside_wire,
            "reference_blade_deficit_mm": (
                blade_alongside_wire - REFERENCE_BLADE_THICKNESS_MM
            ),
        },
        "new_actuator": False,
        "software_change": "M0 withdrawal/index interlock only",
        "finding": (
            "The rigid depth-spanning shoe is impossible even before copper: "
            "the common lined corridor is negative with a zero-thickness "
            "blade.  At the inner neck, the actual packed wire leaves only "
            "the reported residual thickness, so a real blade cannot remain "
            "beside the wire."
        ),
    }

    stator_attached = {
        "architecture": "stator-attached 24-fold mouth-only dual-end polished horn/end-cap",
        "status": "SMALLEST_CREDIBLE_CONCEPT_NOT_RELEASE_PROVEN",
        "radial_scope": (
            "mouth/shoe band only; no blade may remain alongside wire at the "
            "inner neck"
        ),
        "minimum_horn_center_radius_mm": float(
            bend["minimum_24_fold_pin_center_radius_mm"]),
        "available_radial_band_to_od_mm": (
            float(DEFAULT_STATOR.od) / 2.0
            - float(bend["minimum_24_fold_pin_center_radius_mm"])
        ),
        "24_fold_gap_at_od_mm": float(bend["pin_gap_at_stator_od_mm"]),
        "minimum_axial_horn_projection_mm": float(
            bend["minimum_quarter_turn_axial_projection_mm"]),
        "existing_flare_shortfall_mm": float(
            bend["existing_flare_projection_shortfall_mm"]),
        "liner_only_residual_lateral_budget_for_0p25_blade_mm": float(
            active["liner_only_residual_lateral_budget_mm"]),
        "required_change": (
            "replace/local-relieve both Nomex star-cap mouths and add polished "
            "front/rear R3-centerline horns that translate and index with the stator"
        ),
        "new_actuator": False,
        "software_change": (
            "no new motion DOF; route/capture must select the correct front/"
            "rear horn for each flyer sign and retain M1 indexing interlocks"
        ),
        "progressive_prior_copper": "NOT_PROVEN",
        "current_half_both_signs": "NOT_PROVEN",
        "rigid_flyer_indexing_sweep": "NOT_PROVEN",
        "reason_ranked_first": (
            "It follows M0/M1 by construction, avoids the impossible shared "
            "fixed-depth corridor, stays out of the narrow packed neck, and "
            "adds no actuator.  It is the smallest architecture that can "
            "physically move the R3 turn outboard of the steel collision."
        ),
    }

    retracting = {
        "architecture": "carriage-mounted mouth-only tangentially retracting polished guide",
        "status": "CREDIBLE_FALLBACK_NOT_RELEASE_PROVEN",
        "minimum_depth_tracking_offset_range_lower_bound_mm": float(
            reference_blade["optimistic_tangential_tracking"][
                "minimum_offset_range_mm"]
        ),
        "minimum_centered_clear_of_lined_mouth_stroke_lower_bound_mm": (
            centered_clear_stroke
        ),
        "illustrative_stroke_with_0p05mm_clearance_mm": design_clear_stroke,
        "warning": (
            "The clear-mouth number is only a kinematic lower bound; a real "
            "pivot/slide must add tracking error, wear, mount compliance, and "
            "a proven extraction trajectory that never crosses copper."
        ),
        "new_actuator": True,
        "software_change": (
            "new synchronized per-half-turn guide motion, homing, position "
            "feedback, fail-safe retracted state, and M0/M1/M2 interlocks"
        ),
        "progressive_prior_copper": "NOT_PROVEN",
        "current_half_both_signs": "NOT_PROVEN",
        "reason_ranked_second": (
            "It can withdraw before the wire enters the 0.238 mm lined inner "
            "neck, but it costs a new safety-critical motion axis and its "
            "extraction path is not yet certified."
        ),
    }

    return {
        "architecture": "polished guide variants",
        "status": "DESIGN_CHANGE_REQUIRED",
        "fixed_carriage_full_depth": fixed_carriage,
        "stator_attached_mouth_only": stator_attached,
        "tangentially_retracting_mouth_only": retracting,
    }


def analyze() -> dict[str, Any]:
    packing = _load(PACKING_PATH)
    feasibility = _load(FEASIBILITY_PATH)
    route_reports = [(path, _load(path)) for path in ROUTE_PATHS]
    if packing.get("schema") != "slot-packing/v2" or packing.get("status") != "PASS":
        raise RuntimeError("authoritative packing input is not PASS slot-packing/v2")
    if (
        int(packing["config"]["slots"]) != 24
        or int(packing["selected_schedule"]["turns_per_tooth"]) != 50
    ):
        raise RuntimeError("this trade is bound to the 24-slot, 50-turn release job")
    if (
        abs(
            float(packing["config"]["wire_finished_diameter_mm"])
            - float(slot_packing_audit.SUPPLIER_NOMINAL_WIRE_MM)
        ) > 1.0e-12
        or abs(
            float(packing["config"]["liner_thickness_mm"])
            - float(slot_packing_audit.SUPPLIER_NOMINAL_LINER_MM)
        ) > 1.0e-12
    ):
        raise RuntimeError(
            "trade input drifted from the measured nominal packing kernel"
        )
    if feasibility.get("status") != "DESIGN_CHANGE_REQUIRED":
        raise RuntimeError("slot guide feasibility input unexpectedly changed status")

    trade_coverage = coverage_trade(feasibility)
    trade_reduction = reduction_trade(packing, route_reports)
    trade_guide = guide_trade(feasibility, packing)
    required_flags = {
        "minimum_centerline_bend_radius_at_least_3mm": False,
        "both_flyer_directions": False,
        "all_progressive_prior_copper": False,
        "already_laid_current_half_sign_specific": False,
        "exact_core_and_liner": False,
        "rigid_guide_and_indexing_sweep": False,
        "complete_physical_error_budget": False,
    }
    result: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "DESIGN_CHANGE_REQUIRED",
        "release_authorized": False,
        "scope": {
            "job": "24-slot OD46 x stack15, 50 turns/tooth",
            "minimum_wire_center_bend_radius_mm": MINIMUM_CENTERLINE_BEND_RADIUS_MM,
            "required_motion_signs": [-1, 1],
            "progressive_prior_and_current_half_required": True,
            "purpose": "architecture ranking and quantitative no-go boundaries",
        },
        "input_hashes": {
            path.relative_to(ROOT).as_posix(): _sha256(path)
            for path in (
                PACKING_PATH, FEASIBILITY_PATH, *ROUTE_PATHS,
                CAD / "params.py", CAD / "stator_model.py",
                CAD / "coil_growth.py", CAD / "slot_packing_audit.py",
                CAD / "stator_insulation_nomex410.py",
            )
        },
        "trade_1_mouth_and_tooth_shoe_coverage": trade_coverage,
        "trade_2_turn_depth_or_wire_reduction": trade_reduction,
        "trade_3_polished_guide": trade_guide,
        "ranking": [
            {
                "rank": 1,
                "candidate": "stator-attached 24-fold mouth-only dual-end polished horn/end-cap",
                "classification": "smallest credible physical architecture",
                "release": "NOT_AUTHORIZED",
            },
            {
                "rank": 2,
                "candidate": "carriage-mounted tangentially retracting mouth-only guide",
                "classification": "fallback requiring a new actuator/software axis",
                "release": "NOT_AUTHORIZED",
            },
            {
                "rank": 3,
                "candidate": "lamination tooth-shoe coverage change",
                "classification": "cannot provide the R3 turn or current-half separation alone",
                "release": "NO_GO_AS_SOLE_CHANGE",
            },
            {
                "rank": 4,
                "candidate": "reduce turns or wire envelope",
                "classification": "does not cure sign-specific collision and changes electrical job",
                "release": "NO_GO",
            },
            {
                "rank": 5,
                "candidate": "machine-fixed full-depth split shoe",
                "classification": "exact common corridor is geometrically empty",
                "release": "NO_GO",
            },
        ],
        "selected_minimum_change": {
            "architecture": (
                "replace/local-relieve the two stator end caps with 24-fold, "
                "stator-attached mouth-only front/rear polished horns; keep "
                "all guide material out of the inner packed neck"
            ),
            "why": (
                "It is the only compared concept that follows the existing "
                "M0/M1 motion without a new actuator while placing the R3 "
                "bend at radius >=21.0686 mm, outside the fatal steel endpoint."
            ),
            "authorization": "CONCEPT_ONLY",
        },
        "release_gate": {
            "status": "FAIL",
            "required_flags": required_flags,
            "reason": (
                "No compared architecture yet has one exact 360-degree x 9-"
                "depth route certificate for both flyer signs against every "
                "prior turn and the sign-specific current half."
            ),
        },
        "next_proof": [
            "Model only the stator-attached mouth horns/end caps; do not extend a blade into the inner neck.",
            "Generate 360 degree x 9 depth x 2 sign routes with exact core/liner, all prior copper, and sign-specific current-half capsules.",
            "Require >=3.0 mm centerline curvature and positive error-budgeted clearance at every sample/segment.",
            "Sweep the repeated horn bodies against flyer, chuck, carriage, and M1 indexing; prove deposited copper is never trapped.",
            "If that fails, prototype the retracting guide with >=0.677 mm geometric clear-mouth stroke plus measured allowance and a fail-safe retracted sensor.",
        ],
    }
    result["report_sha256"] = _canonical_hash(result)
    return result


def render_markdown(report: dict[str, Any]) -> str:
    one = report["trade_1_mouth_and_tooth_shoe_coverage"]
    two = report["trade_2_turn_depth_or_wire_reduction"]
    guide = report["trade_3_polished_guide"]
    fixed = guide["fixed_carriage_full_depth"]
    stator = guide["stator_attached_mouth_only"]
    retract = guide["tangentially_retracting_mouth_only"]
    wire = two["wire_reduction_frozen_route_upper_bound"]
    bend = one["three_mm_bend_boundary"]
    return f"""# Slot-mouth architecture trade

**Overall: {report['status']} — release is not authorized.**

## Decision

The smallest physically credible change is a **stator-attached, mouth-only,
dual-end polished horn/end-cap**.  It must move with M0/M1, keep all guide
material out of the packed inner neck, and put the R3 wire-center turn at or
beyond radius {stator['minimum_horn_center_radius_mm']:.6f} mm.  This is a
concept ranking, not a passed route.

## Quantitative comparison

| Change | Controlling result | Actuator / software | Decision |
|---|---:|---|---|
| Widen tooth-shoe mouth | even zero shoe gives {bend['maximum_possible_inner_mouth_if_shoe_deleted_mm']:.6f} mm vs {bend['external_pin_diameter_mm']:.3f} mm R3 pin | no new axis; new stator + full regeneration | no-go alone |
| Remove deepest layer | <= {two['turn_reduction']['maximum_turns_per_tooth_without_layer3']} turns, {100.0 * two['turn_reduction']['turn_loss_fraction']:.1f}% turn loss | new electrical/controller job | collision still begins at turn 0 |
| Shrink wire | <= {wire['maximum_diameter_after_known_budget_mm']:.6f} mm frozen-route upper bound vs {wire['nominal_finished_diameter_mm']:.5f} mm nominal | new packing/electrical job | no-go |
| Fixed carriage full-depth blade | {fixed['reference_0p10mm_blade_corridor']['fixed_blade']['minimum_common_corridor_margin_mm']:.6f} mm common-corridor margin | no new axis | geometric no-go |
| Stator-attached mouth-only horns | {stator['24_fold_gap_at_od_mm']:.6f} mm OD pitch gap; {stator['available_radial_band_to_od_mm']:.6f} mm radial band | no new axis; sign-aware route | smallest credible concept |
| Tangential retract guide | >= {retract['minimum_centered_clear_of_lined_mouth_stroke_lower_bound_mm']:.6f} mm bare lower-bound stroke ({retract['illustrative_stroke_with_0p05mm_clearance_mm']:.6f} mm with 0.05 mm illustrative allowance) | new actuator, sensor, synchronized half-turn motion | fallback |

## Important no-go boundaries

- A zero-thickness fixed blade still has
  {fixed['zero_blade_corridor']['fixed_blade']['minimum_common_corridor_margin_mm']:.6f} mm
  common-corridor margin.  The depth mismatch is in the stator neck-to-shoe
  geometry, not merely the proposed blade thickness.
- At the inner winding station the lined slot is only
  {fixed['narrowest_inner_slot']['lined_wall_to_wall_mm']:.6f} mm wide.  The
  {fixed['narrowest_inner_slot']['packed_finished_wire_mm']:.5f} mm packed wire
  leaves {fixed['narrowest_inner_slot']['maximum_remaining_thickness_alongside_wire_mm']:.6f} mm;
  a 0.10 mm blade cannot remain beside it.
- The R3 external surface needs a {bend['external_pin_diameter_mm']:.3f} mm
  pin.  The complete inner tooth pitch is still
  {abs(bend['inner_pitch_shortfall_mm']):.6f} mm too small, so that turn must be
  axial/outboard rather than forced through a widened mouth.
- Every tested route family still fails sign-specific current-half clearance;
  neither turn truncation nor a cosmetic wire-size change is a route proof.

## Required next proof

1. Model mouth-only stator-attached front/rear horns; no deep blade.
2. Run every flyer angle, all nine M0 depths, and both motion signs against
   exact steel/liner, every prior turn, and the already-laid current half.
3. Require R >= 3.0 mm, a positive physical error budget, and rigid flyer/
   chuck/indexing clearance.
4. If the passive stator-attached version fails, use the retracting-guide
   fallback with a larger measured stroke, position feedback, and a fail-safe
   retracted interlock.

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
    selected = report["selected_minimum_change"]
    print(
        f"slot architecture trade {report['status']}; "
        f"selected={selected['authorization']}: {selected['architecture']}"
    )


if __name__ == "__main__":
    main()
