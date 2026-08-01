"""Bounded fail-closed trade for a manufactured cap-to-live-tail support.

This audit asks a narrower question than the older wire-path studies: can a
real positive-volume PEEK/polished feature support the conductor from the
permanent cap lane to the growing live tail, without changing the upstream
M0/M1/M2 program?  It partitions the currently credible attachment/motion
families and consumes their exact CAD and route audits.  A free mathematical
S-curve is deliberately not accepted as physical support authority.

No assembly, controller, capture, BOM, or immutable release is modified.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping


ROOT = Path(__file__).resolve().parent.parent
REPORTS = ROOT / "out" / "reports"
JSON_OUT = REPORTS / "cap_live_tail_manufactured_support_trade.json"
MD_OUT = REPORTS / "cap_live_tail_manufactured_support_trade.md"
SCHEMA = "cap-live-tail-manufactured-support-trade/v1"

REPORT_PATHS = {
    "fixed_cap": REPORTS / "passive_terminal_guide_successor.json",
    "full_depth_shoe": REPORTS / "active_tooth_shoe.json",
    "mouth_cam": REPORTS / "passive_cam_slot_guide.json",
    "collapsible": REPORTS / "collapsible_former.json",
    "retained_former": REPORTS / "r3_tooth_end_former.json",
    "full_shroud": REPORTS / "m0_following_full_shroud.json",
    "m2_cam": REPORTS / "m2_cammed_alternating_former.json",
    "selected_gimbal": REPORTS / "m1_selector_alternating_former.json",
    "aggregate_authority": (
        REPORTS / "permanent_cap_aggregate_authorization.json"
    ),
    "elastic_turn45": REPORTS / "elastic_3d_turn45_route_study.json",
}

SOURCE_PATHS = (
    Path("sim/cap_live_tail_manufactured_support_trade.py"),
    Path("cad/permanent_cap_production_review.py"),
    Path("cad/carriage_active_sector_terminal_guide.py"),
    Path("cad/active_tooth_shoe.py"),
    Path("cad/m1_selector_alternating_former.py"),
    Path("cad/r3_tooth_end_former.py"),
)

WIRE_DIAMETER_RANGE_MM = (0.2, 0.5)
MINIMUM_WIRE_CENTER_RADIUS_MM = 3.0
POLISHED_GROOVE_SIDE_ALLOWANCE_MM = 0.075


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    body = deepcopy(dict(payload))
    body.pop("report_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load_bound_report(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} is not one JSON object")
    if value.get("report_sha256") != _canonical_hash(value):
        raise ValueError(f"predecessor report hash drift: {path.name}")
    return value


def _candidate(
    candidate_id: str,
    *,
    attachment: str,
    physical_feature: str,
    source_cad: str | None,
    manufactured: bool,
    exact_checks: Mapping[str, bool],
    evidence: Mapping[str, Any],
    failure: str,
) -> dict[str, Any]:
    gates = {str(name): bool(value) for name, value in exact_checks.items()}
    survives = manufactured and all(gates.values())
    return {
        "id": candidate_id,
        "attachment_and_state": attachment,
        "physical_feature": physical_feature,
        "source_CAD": source_cad,
        "positive_volume_manufactured_contact_defined": manufactured,
        "exact_gates": gates,
        "survives_bounded_trade": survives,
        "evidence": dict(evidence),
        "controlling_failure": None if survives else failure,
    }


def analyze() -> dict[str, Any]:
    reports = {
        name: _load_bound_report(path)
        for name, path in REPORT_PATHS.items()
    }
    fixed = reports["fixed_cap"]
    shoe = reports["full_depth_shoe"]
    mouth = reports["mouth_cam"]
    collapsible = reports["collapsible"]
    retained = reports["retained_former"]
    shroud = reports["full_shroud"]
    m2_cam = reports["m2_cam"]
    gimbal = reports["selected_gimbal"]
    aggregate = reports["aggregate_authority"]
    elastic = reports["elastic_turn45"]

    maximum_wire_radius = WIRE_DIAMETER_RANGE_MM[1] / 2.0
    minimum_convex_surface_radius = (
        MINIMUM_WIRE_CENTER_RADIUS_MM - maximum_wire_radius
    )
    minimum_groove_clear_width = (
        WIRE_DIAMETER_RANGE_MM[1]
        + 2.0 * POLISHED_GROOVE_SIDE_ALLOWANCE_MM
    )

    fixed_bound = fixed["fixed_cap_lead_in_impossibility"]
    fixed_spacing = fixed_bound["port_spacing"]
    shoe_route = shoe["route_sweep"]
    mouth_route = mouth["progressive_current_half_route"]
    retained_neighbors = retained["wire_routes_and_neighbors"]
    retained_overlap = retained["physical_former_overlap"]
    shroud_motion = shroud["motion"]
    shroud_rigid = shroud["capacity_rigid_load_build"]["rigid_clearance"]
    selected_route = gimbal["exact_progressive_wire_route"]
    partition = aggregate["slot_partition"]
    aggregate_contract = aggregate["slot_to_crown_connectors"]
    progressive_contract = aggregate_contract[
        "progressive_aggregate_contract"
    ]
    elastic_cases = elastic["cases"]

    candidates = [
        _candidate(
            "integral_fixed_PEEK_cap_horn",
            attachment="stator cap; memoryless fixed geometry",
            physical_feature=(
                "integral polished PEEK R3 horn continuing each permanent "
                "cap lane into the active slot"
            ),
            source_cad="cad/permanent_cap_production_review.py",
            manufactured=True,
            exact_checks={
                "current_cap_lane_authority": fixed["authority_gates"]
                ["aggregate_cap_lane_authority_PASS"],
                "R3_sweep_fits_owned_sector": fixed["fixed_cap_gates"]
                ["R3_turn_sweep_fits_authorized_sector_margin"],
                "capture_width_fits_open_mouth": fixed["fixed_cap_gates"]
                ["approach_cone_fits_current_open_mouth"],
                "48_R3_envelopes_do_not_overlap": fixed["fixed_cap_gates"]
                ["independent_R3_mouth_guides_do_not_overlap"],
                "all_2400_no_core_crossing": fixed["fixed_cap_gates"]
                ["no_terminal_span_core_crossing"],
            },
            evidence={
                "raw_locus_count": fixed["raw_terminal_sweep"]["locus_count"],
                "core_crossing_locus_count": fixed["raw_terminal_sweep"]
                ["core_crossing_loci"],
                "required_R3_lateral_sweep_mm": fixed_bound
                ["R3_minimum_lateral_turn_sweep_mm"],
                "authorized_sector_margin_mm": fixed_bound
                ["authorized_lane_sector_margin_mm"],
                "required_capture_width_mm": fixed_bound
                ["required_full_capture_width_mm"],
                "available_mouth_width_mm": fixed_bound
                ["current_open_mouth_width_mm"],
                "minimum_adjacent_port_spacing_mm": fixed_spacing
                ["minimum_center_spacing_mm"],
                "independent_R3_overlap_mm": fixed_spacing
                ["independent_R3_envelope_overlap_mm"],
            },
            failure=(
                "A real fixed R3 turn exceeds the cap-owned sector and mouth; "
                "the 48 required envelopes overlap and 1,000 raw spans cross core."
            ),
        ),
        _candidate(
            "machine_fixed_full_depth_split_PEEK_shoe",
            attachment="carriage/machine fixed; stator traverses it with M0",
            physical_feature=(
                "two positive-volume polished PEEK blades with R3 end horns"
            ),
            source_cad="cad/active_tooth_shoe.py",
            manufactured=True,
            exact_checks={
                "common_liner_preserving_M0_corridor": (
                    shoe["corridor"]["status"] == "PASS"
                ),
                "all_6480_progressive_routes": (
                    shoe_route["passing_case_count"]
                    == shoe_route["required_case_count"]
                ),
                "flyer_rigid_clearance": shoe["gates"]
                ["flyer_rigid_clearance"],
                "chuck_rigid_clearance": shoe["gates"]
                ["chuck_rigid_clearance"],
                "extraction": shoe["extraction"]["status"] == "PASS",
            },
            evidence={
                "minimum_common_corridor_margin_mm": shoe["corridor"]
                ["minimum_common_corridor_margin_mm"],
                "passing_route_cases": shoe_route["passing_case_count"],
                "required_route_cases": shoe_route["required_case_count"],
                "minimum_flyer_clearance_mm": shoe["rigid_motion"]
                ["flyer_360deg"]["minimum_clearance_mm"],
                "minimum_chuck_clearance_mm": shoe["rigid_motion"]
                ["chuck_9_depths"]["minimum_clearance_mm"],
            },
            failure=(
                "The physical blade has no common M0 corridor, clears zero of "
                "6,480 progressive route cases, and intersects the chuck."
            ),
        ),
        _candidate(
            "M0_cammed_mouth_only_PEEK_fingers",
            attachment="carriage; M0 cam tracks depth and retracts for index",
            physical_feature=(
                "spring-return tangential PEEK mouth fingers with polished R3 noses"
            ),
            source_cad="cad/flyer_slot_guide_feasibility.py",
            manufactured=True,
            exact_checks={
                "M0_engage_extract_retract_law": (
                    mouth["motion_synchronization"]["status"] == "PASS"
                ),
                "manufacturing_error_budget": (
                    mouth["manufacturing_error_budget"]["status"] == "PASS"
                ),
                "all_7200_downstream_routes": (
                    mouth_route["complete_passing_case_count"]
                    == mouth_route["expected_case_count"]
                ),
                "core_liner_clearance": (
                    mouth_route["core_liner_failure_count"] == 0
                ),
                "prior_and_current_copper_clearance": (
                    mouth_route["prior_and_neighbor_copper_failure_count"] == 0
                    and mouth_route["already_laid_current_half_failure_count"] == 0
                ),
            },
            evidence={
                "radial_tracking_stroke_mm": mouth["motion_synchronization"]
                ["radial_tracking_stroke_mm"],
                "passing_route_cases": mouth_route
                ["complete_passing_case_count"],
                "required_route_cases": mouth_route["expected_case_count"],
                "minimum_core_center_clearance_mm": mouth_route
                ["minimum_core_center_clearance_mm"],
                "required_core_center_clearance_mm": mouth_route
                ["required_core_center_clearance_mm"],
                "minimum_prior_copper_clearance_mm": mouth_route
                ["minimum_prior_copper_centerline_clearance_mm"],
                "required_copper_clearance_mm": mouth_route
                ["required_copper_centerline_clearance_mm"],
            },
            failure=(
                "The mechanism closes, but a mouth-only contact leaves the "
                "unsupported inner span: only 1,332/7,200 exact routes pass."
            ),
        ),
        _candidate(
            "passive_two_half_collapsible_R3_former",
            attachment="stator following; post-pass M0 retract attempts transfer",
            physical_feature=(
                "two smooth PEEK former halves carrying the complete 50-turn preform"
            ),
            source_cad=None,
            manufactured=False,
            exact_checks={
                "available_M0_transfer_stroke": collapsible["gates"]
                ["M0_stroke_sufficient_for_ideal_transfer"],
                "steel_liner_neighbor_transfer_clearance": collapsible["gates"]
                ["two_half_transfer_clears_steel_liner_and_neighbor"],
                "contact_plane_rigid_clearance": (
                    collapsible["transfer_study"]["radial_transfer"]
                    ["contact_plane_rigid_flyer_clearance"] == "PASS"
                ),
            },
            evidence={
                "available_stroke_margin_mm": collapsible["transfer_study"]
                ["radial_transfer"]["stroke_margin_mm"],
                "best_joint_margin_mm": collapsible["transfer_study"]
                ["two_half_transfer_witness"]["sampled_best_joint_margin_mm"],
                "exact_OCC_core_margin_mm": collapsible["transfer_study"]
                ["two_half_transfer_witness"]["exact_OCC_core_margin_at_best_mm"],
            },
            failure=(
                "Ideal M0 stroke exists, but no positive-volume former is "
                "defined and the exact transfer witness penetrates core/neighbor."
            ),
        ),
        _candidate(
            "retained_stator_R3_end_former",
            attachment="retained on both stator end faces",
            physical_feature=(
                "two positive-volume tooth paddles defining R3 crown contact"
            ),
            source_cad="cad/r3_tooth_end_former.py",
            manufactured=True,
            exact_checks={
                "one_tooth_R3_route": (
                    retained_neighbors["same_tooth"]["analytic_status"] == "PASS"
                ),
                "all_24_neighbor_clearance": retained["gates"]
                ["all_24_neighbor_wire_clearance"],
                "physical_former_nonoverlap": (
                    retained_overlap["status"] == "PASS"
                ),
                "all_raw_pose_rigid_clearance": retained["gates"]
                ["every_raw_pose_rigid_clearance"],
                "rotor_end_bell_cavity": (
                    retained["slot_fill_and_motor_envelope"]
                    ["retained_motor_envelope"]["status"] == "PASS"
                ),
            },
            evidence={
                "turn_count": retained_neighbors["turn_count"],
                "selected_lane_status": retained_neighbors["selected_lane"]
                ["status"],
                "minimum_solid_clearance_mm": retained_overlap
                ["minimum_solid_clearance_mm"],
                "motor_axial_cavity_status": retained
                ["slot_fill_and_motor_envelope"]["retained_motor_envelope"]
                ["rotor_end_bell_axial_cavity_status"],
            },
            failure=(
                "The one-tooth R3 construction passes, but all-tooth wires "
                "and the physical paddles overlap; motor axial cavity is undefined."
            ),
        ),
        _candidate(
            "M0_following_retractable_full_PEEK_shroud",
            attachment="carriage; tracks M0 and fully retracts before M1",
            physical_feature=(
                "convex two-face R3 PEEK shroud with spring-return retraction"
            ),
            source_cad=None,
            manufactured=False,
            exact_checks={
                "tracking_extraction_timing_clearance": shroud["gates"]
                ["M0_tracking_extraction_timing_and_clearance"],
                "two_mm_rigid_clearance": shroud["gates"]
                ["two_millimetre_rigid_clearance"],
                "former_supported_R3_route": (
                    shroud["wire_route_and_contact"]["status"] == "PASS"
                ),
            },
            evidence={
                "extraction_stroke_shortfall_mm": shroud_motion
                ["extraction_stroke_shortfall_mm"],
                "minimum_rigid_clearance_mm": shroud_rigid
                ["minimum_rigid_clearance_mm"],
                "required_rigid_clearance_mm": 2.0,
            },
            failure=(
                "The raw retract stroke is 1.766 mm short, rigid clearance is "
                "0.234 mm rather than 2 mm, and no supported R3 route exists."
            ),
        ),
        _candidate(
            "M1_selected_M2_phased_single_gimbal_polished_shoe",
            attachment=(
                "carriage selector; M1 selects law, M2 phases one shoe, M0 "
                "forces all-retracted"
            ),
            physical_feature=(
                "one mutually exclusive two-axis polished ceramic/PEEK R3.25 shoe"
            ),
            source_cad="cad/m1_selector_alternating_former.py",
            manufactured=True,
            exact_checks={
                "three_laws_selected": (
                    gimbal["selector_and_signed_cam"]["status"] == "PASS"
                ),
                "M0_fail_safe_retraction": (
                    gimbal["M0_fail_safe_gate"]["status"] == "PASS"
                ),
                "rigid_clearance": gimbal["rigid_clearances"]["status"] == "PASS",
                "300rpm_load_balance_interlock": (
                    gimbal["loads_balance_sensors"]["status"]
                    == "PASS_ANALYTICAL"
                ),
                "all_turn_growth_R3_tail_routes": (
                    selected_route["copper_clear_R3_tail_candidates"] > 0
                    and selected_route["status"] == "PASS"
                ),
                "elastic_R3_contact": (
                    selected_route["elastic_curvature_pass_count"] > 0
                ),
            },
            evidence={
                "raw_pass_count": selected_route["raw_pass_count"],
                "turn_growth_state_count": 50,
                "bounded_tail_candidate_count": 6240,
                "copper_clear_R3_tail_candidates": selected_route
                ["copper_clear_R3_tail_candidates"],
                "elastic_R3_pass_count": selected_route
                ["elastic_curvature_pass_count"],
                "minimum_deployed_flyer_clearance_mm": gimbal
                ["rigid_clearances"]
                ["minimum_deployed_finger_to_flyer_clearance_mm"],
                "forced_retracted_M1_margin_mm": gimbal["M0_fail_safe_gate"]
                ["minimum_forced_retracted_margin_at_M1_move_mm"],
                "M2_torque_margin": gimbal["loads_balance_sensors"]
                ["revised_M2_margin"],
                "stationary_M2_cam_alias_status": m2_cam
                ["stationary_cam_phase_alias"]["status"],
            },
            failure=(
                "This is the smallest mechanically closed moving architecture, "
                "but 0/6,240 bounded one-stage R3 tails clear progressive copper "
                "and 0 elastic/contact cases meet R3."
            ),
        ),
    ]

    survivors = [row for row in candidates if row["survives_bounded_trade"]]
    aggregate_radial_span = list(map(
        float, partition["aggregate_radial_center_span_mm"]
    ))
    aggregate_radial_travel = (
        aggregate_radial_span[1] - aggregate_radial_span[0]
    )
    aggregate_tangential_travel = float(partition["cutoff_half_width_mm"])
    diagnostic_detour = max(
        float(case["analytic_R3_multiarc"]["length_mm"])
        for case in elastic_cases
    )
    diagnostic_chord = max(
        float(case["source_target_chord_mm"])
        for case in elastic_cases
    )
    shallow_without_clamped_terminal = all(
        case["shallow_normal_bow"]["meets_terminal_clearance_and_R3"] is True
        and case["shallow_normal_bow"]
        ["endpoint_tangent_contract_evaluated"] is False
        for case in elastic_cases
    )
    aggregate_tangent_successor = {
        "id": "M1_M2_selected_two_translation_aggregate_boundary_follower/v1",
        "status": "PROMISING_AGGREGATE_AUTHORITY_ONLY_NOT_CAD_PROVED",
        "recommended": True,
        "new_commanded_axis": False,
        "upstream_motion_change": False,
        "smallest_additional_physical_DOF": [
            {
                "axis": "active-tooth radial translation",
                "minimum_current_aggregate_tracking_span_mm": (
                    aggregate_radial_travel
                ),
                "prototype_design_stroke_mm": 6.0,
                "state_source": (
                    "spring preload/contact reaction against the exposed "
                    "aggregate boundary; not deterministic strand centres"
                ),
            },
            {
                "axis": "active-slot tangential translation",
                "minimum_current_half_slot_tracking_span_mm": (
                    aggregate_tangential_travel
                ),
                "prototype_design_stroke_mm": 1.0,
                "state_source": (
                    "spring-centred cross slide/gimbal contact reaction; "
                    "separate left/right shoe identities remain M2 selected"
                ),
            },
        ],
        "retained_existing_state_selection": {
            "M1": "select one of three existing raw phase laws",
            "M2": "select one of four end/tangential shoe identities",
            "M0": (
                "positive cam withdrawal captures both translation slides "
                "and the gimbal in all-retracted state before M1"
            ),
            "failure_state": "spring return plus positive dock is all-retracted",
            "hardwired_M1_permission": "NC all-retracted switch only",
            "existing_forced_retracted_margin_at_M1_mm": gimbal
            ["M0_fail_safe_gate"]
            ["minimum_forced_retracted_margin_at_M1_move_mm"],
        },
        "manufactured_support_surface": {
            "kind": (
                "two-axis floating polished shoe with a distributed convex "
                "R3 contact nose, mounted in the existing selected gimbal"
            ),
            "surface_radius_mm": 3.0,
            "wire_center_radius_range_mm": [3.1, 3.25],
            "polished_groove_clear_width_mm": minimum_groove_clear_width,
            "finish": "Ra <= 0.4 um; PEEK or polished ceramic contact insert",
            "contact_owner_at_g0": "Nomex-lined active-tooth boundary",
            "contact_owner_after_g0": "exposed active prior-copper aggregate",
        },
        "aggregate_tangent_evaluation": {
            "authority_status": aggregate["status"],
            "aggregate_geometry_authorized": aggregate
            ["aggregate_geometry_authorized"],
            "exact_strand_packing_predicted": aggregate["aggregate_loft"]
            ["exact_strand_packing_predicted"],
            "growth_parameter": progressive_contract["growth_parameter"],
            "live_boundary_rule": progressive_contract["live_boundary_rule"],
            "active_prior_positive_volume_intrusion_mm3": (
                progressive_contract
                ["active_prior_aggregate_positive_volume_intrusion_mm3"]
            ),
            "completed_neighbor_positive_volume_intrusion_mm3": (
                progressive_contract
                ["completed_neighbor_aggregate_positive_volume_intrusion_mm3"]
            ),
            "convex_supporting_tangent_exists": True,
            "taut_free_span_curvature": "straight; infinite bend radius",
            "terminal_tangent": (
                "supporting tangent selected from the exposed convex S_g/C_g "
                "boundary; no stored-loop axial terminal tangent is imposed"
            ),
            "all_50_growth_states_classified_without_strand_order": True,
        },
        "obsolete_packed_tangent_comparison": {
            "diagnostic_turn": 45,
            "source_target_chord_mm": diagnostic_chord,
            "five_arc_detour_length_mm": diagnostic_detour,
            "detour_is_required_by_aggregate_authority": False,
            "shallow_R3_bow_passes_when_terminal_tangent_not_enforced": (
                shallow_without_clamped_terminal
            ),
            "interpretation": (
                "The 36.904 mm route closes two clamped endpoint tangents on "
                "a 0.5 mm diagnostic packed-loop chord. The aggregate contact "
                "contract does not authorize that terminal tangent, so this "
                "detour is not a lower bound on an aggregate-tangent follower."
            ),
        },
        "gates": {
            "aggregate_contact_authority_current_job": (
                aggregate["status"] == "PASS"
                and aggregate["aggregate_geometry_authorized"] is True
                and aggregate_contract["status"] == "PASS"
            ),
            "all_50_growth_states_have_named_contact_owner": True,
            "taut_span_meets_R3_away_from_contact": True,
            "obsolete_diagnostic_terminal_tangent_removed": (
                shallow_without_clamped_terminal
            ),
            "positive_volume_slide_gimbal_shoe_CAD_complete": False,
            "all_2400_raw_pose_route_and_rigid_clearance": False,
            "fail_safe_retraction_collision_sweep_before_M1": False,
            "wire_range_contact_load_wear_endurance": False,
        },
        "authority_limit": (
            "Convexity proves a supporting tangent and the aggregate report "
            "owns intended contact; it does not supply the follower's exact "
            "positive-volume placement, preload, continuous trajectory, or "
            "release/extraction proof."
        ),
        "next_prototype_condition": (
            "Generate isolated positive-volume slide/gimbal CAD only after "
            "one exact tooth-0 route family binds the cap-lane exit, floating "
            "R3 nose, and S_g/C_g supporting tangent at all g=n/50 without "
            "entering the aggregate interior."
        ),
    }
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "DESIGN_NO_GO",
        "decision": "NO_MANUFACTURED_SUPPORT_SURVIVES_CURRENT_BOUNDED_TRADE",
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "upstream_motion_modified": False,
        "free_mathematical_curve_is_support_authority": False,
        "scope": {
            "wire_diameter_range_mm": list(WIRE_DIAMETER_RANGE_MM),
            "minimum_wire_center_bend_radius_mm": MINIMUM_WIRE_CENTER_RADIUS_MM,
            "minimum_convex_support_surface_radius_for_range_mm": (
                minimum_convex_surface_radius
            ),
            "minimum_polished_groove_clear_width_for_range_mm": (
                minimum_groove_clear_width
            ),
            "required_growth_states_per_tooth": 50,
            "required_raw_loci": 2400,
            "attachment_partition": [
                "fixed to permanent cap/stator",
                "fixed to carriage across M0 depth",
                "M0-cammed mouth-only",
                "passive collapsible/retained former",
                "M0-following full shroud",
                "M1/M2-selected M0-retracted single moving shoe",
            ],
            "not_a_universal_impossibility_proof": True,
        },
        "reviewed_CAD_contract": {
            "permanent_PEEK_cap_lanes": "cad/permanent_cap_production_review.py",
            "active_sector_terminal_guides": "cad/carriage_active_sector_terminal_guide.py",
            "full_depth_split_shoe": "cad/active_tooth_shoe.py",
            "selected_retractable_former": "cad/m1_selector_alternating_former.py",
            "retained_R3_former": "cad/r3_tooth_end_former.py",
        },
        "candidates": candidates,
        "surviving_candidate_count": len(survivors),
        "surviving_candidate_ids": [row["id"] for row in survivors],
        "smallest_mechanically_closed_candidate": {
            "id": "M1_selected_M2_phased_single_gimbal_polished_shoe",
            "mechanism_and_rigid_envelope": "PASS",
            "wire_route": "FAIL",
            "why_no_prototype": (
                "A positive-volume prototype would imply a contact route the "
                "exact 6,240-candidate progressive-copper sweep disproves."
            ),
        },
        "recommended_successor": aggregate_tangent_successor,
        "prototype": {
            "created": False,
            "path": None,
            "reason": "No candidate survives every mandatory route/clearance gate.",
        },
        "next_geometry_that_would_change_the_result": {
            "requirement": (
                "Add radial and tangential floating translations to the "
                "selected gimbal shoe so its polished R3 nose rides the "
                "exposed convex S_g/C_g boundary rather than a deterministic "
                "strand endpoint. It must then produce one exact C1, R>=3 mm "
                "route at every growth state in both M2 directions."
            ),
            "must_recheck": [
                "all 2,400 raw poses",
                "lined core and named prior/current copper classes",
                "flyer, chuck, cap, selector and adjacent-tooth rigid clearance",
                "release/extraction without upstream motion changes",
                "0.2-0.5 mm wire groove and surface-radius bounds",
            ],
        },
        "source_hashes": {
            **{
                str(path).replace("\\", "/"): _sha256(ROOT / path)
                for path in SOURCE_PATHS
            },
            **{
                str(path.relative_to(ROOT)).replace("\\", "/"): _sha256(path)
                for path in REPORT_PATHS.values()
            },
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    validate_report(report)
    return report


def validate_report(
    report: Mapping[str, Any], *, source_root: Path = ROOT,
) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("manufactured support trade schema drift")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("manufactured support trade payload hash mismatch")
    if report.get("status") != "DESIGN_NO_GO":
        raise ValueError("manufactured support trade invented a survivor")
    if report.get("production_authorized") is not False:
        raise ValueError("manufactured support trade invented production authority")
    if report.get("upstream_motion_modified") is not False:
        raise ValueError("manufactured support trade modified upstream motion")
    if report.get("free_mathematical_curve_is_support_authority") is not False:
        raise ValueError("free curve was promoted to physical support")
    candidates = report.get("candidates")
    if not isinstance(candidates, list) or len(candidates) != 7:
        raise ValueError("manufactured support attachment partition is incomplete")
    observed_survivors = [
        row["id"] for row in candidates if row.get("survives_bounded_trade")
    ]
    if observed_survivors or report.get("surviving_candidate_count") != 0:
        raise ValueError("manufactured support survivor count drift")
    if report.get("prototype", {}).get("created") is not False:
        raise ValueError("failed support trade generated a prototype")
    successor = report.get("recommended_successor")
    if not isinstance(successor, Mapping):
        raise ValueError("manufactured support successor is missing")
    if successor.get("status") != (
            "PROMISING_AGGREGATE_AUTHORITY_ONLY_NOT_CAD_PROVED"):
        raise ValueError("manufactured support successor status drift")
    if successor.get("new_commanded_axis") is not False:
        raise ValueError("manufactured support successor added a commanded axis")
    successor_gates = successor.get("gates", {})
    if (successor_gates.get("aggregate_contact_authority_current_job") is not True
            or successor_gates.get(
                "positive_volume_slide_gimbal_shoe_CAD_complete") is not False
            or successor_gates.get(
                "all_2400_raw_pose_route_and_rigid_clearance") is not False):
        raise ValueError("manufactured support successor authority drift")
    hashes = report.get("source_hashes")
    if not isinstance(hashes, Mapping):
        raise ValueError("manufactured support source hashes are missing")
    for relative, expected in hashes.items():
        path = Path(source_root) / str(relative)
        if not path.is_file() or _sha256(path) != expected:
            raise ValueError(f"manufactured support source hash mismatch: {relative}")


def render_markdown(report: Mapping[str, Any]) -> str:
    scope = report["scope"]
    lines = [
        "# Manufactured cap-to-live-tail support trade", "",
        f"**{report['status']} — {report['decision']}**", "",
        "No free S-curve is counted as support. Every candidate below names a "
        "real manufactured contact feature or fails before positive-volume CAD.",
        "",
        "## Bounds", "",
        f"- Wire: {scope['wire_diameter_range_mm'][0]:.1f}–"
        f"{scope['wire_diameter_range_mm'][1]:.1f} mm.",
        f"- Wire-centre bend radius: >= "
        f"{scope['minimum_wire_center_bend_radius_mm']:.1f} mm.",
        f"- Smallest convex support surface at 0.5 mm wire: R"
        f"{scope['minimum_convex_support_surface_radius_for_range_mm']:.3f} mm.",
        f"- Minimum polished groove clear width: "
        f"{scope['minimum_polished_groove_clear_width_for_range_mm']:.3f} mm.",
        "- Required coverage: 50 growth states/tooth and 2,400 raw loci; "
        "upstream motion unchanged.", "",
        "## Candidate disposition", "",
    ]
    for row in report["candidates"]:
        lines.extend([
            f"### `{row['id']}`", "",
            row["physical_feature"], "",
            f"**FAIL:** {row['controlling_failure']}", "",
        ])
    lines.extend([
        "## Concrete successor", "",
        "Add radial and tangential floating slides to the already selected "
        "M1/M2 gimbal shoe. Its polished R3 nose rides the exposed convex "
        "aggregate boundary: liner-owned at g=0 and prior-copper-owned after "
        "g=0. The current aggregate needs "
        f"{report['recommended_successor']['smallest_additional_physical_DOF'][0]['minimum_current_aggregate_tracking_span_mm']:.3f} mm "
        "radial travel and "
        f"{report['recommended_successor']['smallest_additional_physical_DOF'][1]['minimum_current_half_slot_tracking_span_mm']:.3f} mm "
        "tangential travel; 6.0/1.0 mm are the first prototype targets.", "",
        "The span is tangent to S_g/C_g rather than clamped to a diagnostic "
        "packed-loop axial tangent. That removes the 36.904 mm five-arc detour "
        "as a requirement. It does not yet prove the positive-volume follower: "
        "all 2,400 poses, retraction clearance, preload, and wear remain open.", "",
        "## Prototype decision", "",
        "No prototype was generated. The selected M1/M2/M0-retracted gimbal "
        "architecture closes its mechanism, rigid envelope, load and interlock "
        "checks, but the exact search found 0/6,240 copper-clear one-stage R3 "
        "tails and zero compliant R3 contact cases. Building that solid would "
        "misrepresent an unsupported conductor path.", "",
        "This is a bounded no-go across the reviewed attachment/state families, "
        "not a theorem against every possible distributed-contact mechanism.", "",
        f"Report SHA-256: `{report['report_sha256']}`", "",
    ])
    return "\n".join(lines)


def write_outputs(report: Mapping[str, Any] | None = None) -> dict[str, Any]:
    value = analyze() if report is None else dict(report)
    JSON_OUT.parent.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(
        json.dumps(value, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    MD_OUT.write_text(render_markdown(value), encoding="utf-8")
    return value


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(list(argv) if argv is not None else None)
    if args.validate_only:
        report = json.loads(JSON_OUT.read_text(encoding="utf-8"))
        validate_report(report)
    else:
        report = write_outputs()
    print(
        f"manufactured support trade: {report['status']}; "
        f"survivors={report['surviving_candidate_count']}; "
        f"prototype={report['prototype']['created']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
