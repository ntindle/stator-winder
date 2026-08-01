"""Optimize and fail-closed audit the layer-staggered crown route family."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

import slot_wire_routes  # noqa: E402
from crown_routes import (  # noqa: E402
    CrownPolicy,
    adjacent_self_clearance,
    build_c1_crown_route,
    build_current_half_obstacle,
    crown_policy_geometry_sha256,
    crowned_active_copper_before,
    crowned_loop_centerline,
    crowned_neighbor_prefill_copper,
    half_twist_bridge_midpoint_local_z_mm,
    half_twist_curvature_proof,
    packing_frame_half_twist_policy,
    radial_axial_curvature_study,
    radial_axial_dubins_policy,
)
from placement_tolerance import (  # noqa: E402
    DEFAULT_ENCODER_PPR,
    DEFAULT_QUADRATURE_MULTIPLIER,
    PACKING_M0_SETTLE_TOLERANCE_RAD,
    PARAMS,
)
from slot_route import (  # noqa: E402
    CopperField,
    CopperPolyline,
    PackingSupportGraph,
    exact_polyline_part_clearance,
)


SCHEMA = "slot-crown-route-study/v1"
PACKING_PATH = REPORTS / "slot_packing.json"
OUTPUT_PATH = REPORTS / "slot_crown_routes.json"
HALF_TWIST_OUTPUT_PATH = REPORTS / "slot_half_twist_routes.json"
RADIAL_AXIAL_OUTPUT_PATH = REPORTS / "slot_radial_axial_routes.json"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: dict[str, Any]) -> str:
    canonical = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(canonical.encode()).hexdigest()


def known_physical_lower_bound_mm() -> float:
    counts = DEFAULT_ENCODER_PPR * DEFAULT_QUADRATURE_MULTIPLIER
    encoder = 0.5 * (2.0 * math.pi / counts) * PARAMS.mm_per_rad
    controller = PACKING_M0_SETTLE_TOLERANCE_RAD * PARAMS.mm_per_rad
    return float(controller + encoder)


def _maximum_profile_radius(graph: PackingSupportGraph) -> float:
    return max(float(turn.profile_radius_mm) for turn in graph.turns)


def _obstacle_chord_error_mm(
        graph: PackingSupportGraph, policy: CrownPolicy) -> float:
    if policy.geometry_family == "packing_frame_half_twist":
        maximum_curve_radius = (
            policy.half_twist_base_radius_mm
            + _maximum_profile_radius(graph))
        arc_error = maximum_curve_radius * (
            1.0 - math.cos(
                math.radians(policy.obstacle_arc_step_deg) / 2.0))
        # For a proven radius >=3, the bridge sagitta of a step-length chord
        # is bounded by l^2/(8R).
        bridge_error = policy.half_twist_bridge_step_mm ** 2 / (8.0 * 3.0)
        return float(max(arc_error, bridge_error) + 1.0e-9)
    if policy.geometry_family == "radial_axial_dubins":
        maximum_step = (
            policy.radial_axial_radius_mm
            * math.radians(policy.obstacle_arc_step_deg)
            + policy.radial_axial_outward_bias_mm
            * math.pi * math.radians(policy.obstacle_arc_step_deg)
            / (7.0 * math.pi / 3.0))
        return float(maximum_step ** 2 / (8.0 * 3.0) + 1.0e-9)
    return float(
        _maximum_profile_radius(graph)
        * (1.0 - math.cos(
            math.radians(policy.obstacle_arc_step_deg) / 2.0))
        + 1.0e-9)


def _deposited_crown_audit(
    planner: Any,
    graph: PackingSupportGraph,
    spec: Any,
    policy: CrownPolicy,
    obstacle_chord_error: float,
) -> dict[str, Any]:
    deposited = []
    minimum_pairwise = math.inf
    minimum_pairwise_pair = None
    minimum_noncontact = math.inf
    minimum_noncontact_pair = None
    minimum_core = math.inf
    minimum_core_turn = None
    minimum_neighbor = math.inf
    minimum_neighbor_case = None
    failed_pairs = []
    for turn in graph.turns:
        points = crowned_loop_centerline(turn, spec, policy)
        core = exact_polyline_part_clearance(points, planner.stator_part)
        core_lower = core - obstacle_chord_error
        if core_lower < minimum_core:
            minimum_core = core_lower
            minimum_core_turn = turn.turn_index
        deposited.append((turn, points, CopperPolyline(
            obstacle_id=f"active-turn-{turn.turn_index:02d}",
            owner="crowned_deposited_loop",
            turn_index=turn.turn_index,
            centerline_local_mm=tuple(tuple(map(float, point))
                                      for point in points),
        )))

    # Audit all 1,225 pairs.  Looking only at the closest earlier loop for
    # each turn would miss a collision hidden behind an intended tangent.
    for index, (turn, points, _) in enumerate(deposited):
        for other, _, obstacle in deposited[:index]:
            clearance = CopperField((obstacle,)).clearance(
                points, max(0.5, graph.wire_diameter_mm + 0.1))
            raw = clearance.minimum_centerline_distance_mm
            lower = raw - 2.0 * obstacle_chord_error
            pair = [other.turn_index, turn.turn_index]
            if lower < minimum_pairwise:
                minimum_pairwise = lower
                minimum_pairwise_pair = pair
            graph_distance = math.hypot(
                turn.radial_mm - other.radial_mm,
                turn.profile_radius_mm - other.profile_radius_mm)
            tangent = abs(graph_distance - graph.wire_diameter_mm) <= 1e-8
            if not tangent and lower < minimum_noncontact:
                minimum_noncontact = lower
                minimum_noncontact_pair = pair
            # An intended packed tangent is exact on the unchanged straight
            # side, so use its raw segment distance.  All non-contact crown
            # pairs retain the two-sided chord-error lower bound.
            pair_passes = (
                raw + 1e-9 >= graph.wire_diameter_mm
                if tangent else
                lower + 1e-9 >= graph.wire_diameter_mm)
            if not pair_passes:
                failed_pairs.append({
                    "pair_kind": "same_tooth",
                    "turn_pair": pair,
                    "raw_chordal_distance_mm": raw,
                    "lower_bound_mm": lower,
                    "graph_center_distance_mm": graph_distance,
                    "intended_tangent": tangent,
                })

    # Same-tooth progressive checks do not cover the two fully wound
    # adjacent teeth.  Query one combined spatial index so every active crown
    # is compared with all 100 transformed neighbor crowns.
    neighbor_obstacles = tuple(
        obstacle
        for side in (-1, 1)
        for obstacle in crowned_neighbor_prefill_copper(
            graph, spec, side, policy)
    )
    neighbor_field = CopperField(neighbor_obstacles)
    for turn, points, _ in deposited:
        clearance = neighbor_field.clearance(
            points, max(0.5, graph.wire_diameter_mm + 0.1))
        raw = clearance.minimum_centerline_distance_mm
        lower = raw - 2.0 * obstacle_chord_error
        case = [turn.turn_index, clearance.obstacle_id]
        if lower < minimum_neighbor:
            minimum_neighbor = lower
            minimum_neighbor_case = case
        if lower + 1e-9 < graph.wire_diameter_mm:
            failed_pairs.append({
                "pair_kind": "active_to_neighbor_tooth",
                "turn_index": turn.turn_index,
                "neighbor_obstacle_id": clearance.obstacle_id,
                "raw_chordal_distance_mm": raw,
                "lower_bound_mm": lower,
                "intended_tangent": False,
            })
    return {
        "status": "PASS" if not failed_pairs else "FAIL",
        "minimum_pairwise_centerline_lower_bound_mm": minimum_pairwise,
        "minimum_pairwise_turn_pair": minimum_pairwise_pair,
        "minimum_noncontact_centerline_lower_bound_mm": minimum_noncontact,
        "minimum_noncontact_turn_pair": minimum_noncontact_pair,
        "minimum_core_centerline_lower_bound_mm": minimum_core,
        "minimum_core_turn_index": minimum_core_turn,
        "minimum_neighbor_centerline_lower_bound_mm": minimum_neighbor,
        "minimum_neighbor_case": minimum_neighbor_case,
        "required_copper_centerline_mm": graph.wire_diameter_mm,
        "required_core_centerline_mm": graph.center_core_access_mm,
        "failed_pairs": failed_pairs,
    }


def analyze(
    packing_path: Path = PACKING_PATH,
    *,
    policy: CrownPolicy | None = None,
    progress: bool = False,
) -> dict[str, Any]:
    policy = CrownPolicy() if policy is None else policy
    policy.validate()
    packing_path = Path(packing_path)
    packing = json.loads(packing_path.read_text())
    spec = slot_wire_routes._validate_packing_contract(packing)
    graph = PackingSupportGraph.from_report(packing, spec=spec)
    planner = slot_wire_routes.build_planner(graph, spec)
    known_budget = known_physical_lower_bound_mm()
    obstacle_chord = _obstacle_chord_error_mm(graph, policy)
    required_nonparent_raw = (
        graph.wire_diameter_mm + known_budget + obstacle_chord)

    crown_hash = crown_policy_geometry_sha256(graph, spec, policy)
    turn42 = graph.turn(42)
    turn45 = graph.turn(45)
    turn42_crown = crowned_loop_centerline(turn42, spec, policy)
    turn45_crown = crowned_loop_centerline(turn45, spec, policy)
    crown_bounds = np.asarray([
        (np.min(points, axis=0), np.max(points, axis=0))
        for points in (
            crowned_loop_centerline(turn, spec, policy)
            for turn in graph.turns)
    ])
    crown_envelope_min = np.min(crown_bounds[:, 0, :], axis=0)
    crown_envelope_max = np.max(crown_bounds[:, 1, :], axis=0)
    turn42_positive_apex = float(np.max(turn42_crown[:, 2]))
    turn45_positive_apex = float(np.max(turn45_crown[:, 2]))
    if policy.geometry_family == "packing_frame_half_twist":
        turn42_seed_height = half_twist_bridge_midpoint_local_z_mm(
            turn42, spec, policy)
        turn45_seed_height = half_twist_bridge_midpoint_local_z_mm(
            turn45, spec, policy)
        seed_separation = turn45_seed_height - turn42_seed_height
        curvature_proof = half_twist_curvature_proof(
            graph, spec, policy)
    elif policy.geometry_family == "radial_axial_dubins":
        profile_reference = 0.0
        turn42_seed_height = (
            profile_reference
            - policy.radial_axial_profile_scale
            * float(turn42.profile_radius_mm))
        turn45_seed_height = (
            profile_reference
            - policy.radial_axial_profile_scale
            * float(turn45.profile_radius_mm))
        seed_separation = turn45_seed_height - turn42_seed_height
        curvature_proof = radial_axial_curvature_study(
            graph, spec, policy)
    else:
        turn42_seed_height = turn42_positive_apex
        turn45_seed_height = turn45_positive_apex
        seed_separation = abs(
            turn42_positive_apex - turn45_positive_apex)
        curvature_proof = None

    neighbor_obstacles = tuple(
        obstacle
        for side in (-1, 1)
        for obstacle in crowned_neighbor_prefill_copper(
            graph, spec, side, policy)
    )
    records = []
    motion_sign_cases = []
    minimum_core_margin = math.inf
    minimum_core_case = None
    minimum_nonparent_margin = math.inf
    minimum_nonparent_case = None
    minimum_parent_prefix = math.inf
    minimum_parent_case = None
    minimum_current_margin = math.inf
    minimum_current_case = None

    for turn in graph.turns:
        prior = crowned_active_copper_before(
            graph, turn.turn_index, spec, policy)
        field_obstacles = prior + neighbor_obstacles
        parent_ids = {
            f"active-turn-{index:02d}"
            for index in turn.parent_turn_indices
        }
        parent_field = CopperField(tuple(
            obstacle for obstacle in field_obstacles
            if obstacle.obstacle_id in parent_ids))
        nonparent_field = CopperField(tuple(
            obstacle for obstacle in field_obstacles
            if obstacle.obstacle_id not in parent_ids))
        for half_turn_index in (0, 1):
            route = build_c1_crown_route(
                planner, graph, spec, turn.turn_index,
                half_turn_index, policy)
            points = np.asarray(route.points_local_mm, dtype=float)
            core_raw = exact_polyline_part_clearance(
                points, planner.stator_part)
            core_lower = core_raw - route.sampled_arc_chord_error_bound_mm
            core_margin = core_lower - graph.center_core_access_mm
            nonparent = nonparent_field.clearance(
                points, max(0.5, required_nonparent_raw + 0.1))
            nonparent_lower = (
                nonparent.minimum_centerline_distance_mm
                - route.sampled_arc_chord_error_bound_mm
                - obstacle_chord)
            nonparent_margin = (
                nonparent_lower - graph.wire_diameter_mm)
            if parent_ids:
                parent_prefix = parent_field.clearance(
                    points[:-1],
                    max(0.5, graph.wire_diameter_mm + 0.1))
                parent_prefix_lower = (
                    parent_prefix.minimum_centerline_distance_mm
                    - route.sampled_arc_chord_error_bound_mm
                    - obstacle_chord)
                parent_prefix_obstacle = parent_prefix.obstacle_id
            else:
                parent_prefix_lower = math.inf
                parent_prefix_obstacle = None
            endpoint_parent_distances = {}
            for parent_index in turn.parent_turn_indices:
                parent = graph.turn(parent_index)
                endpoint_parent_distances[
                    f"active-turn-{parent_index:02d}"] = math.hypot(
                        turn.radial_mm - parent.radial_mm,
                        turn.profile_radius_mm - parent.profile_radius_mm)
            parent_contact_ok = all(
                abs(distance - graph.wire_diameter_mm) <= 1e-9
                for distance in endpoint_parent_distances.values())
            c1_ok = bool(
                route.minimum_bend_radius_mm >= 3.0
                and route.tip_exit_tangent_error_deg <= 1e-6
                and route.join_tangent_error_deg
                <= policy.route_arc_step_deg / 2.0 + 1e-6
                and route.terminal_tangent_error_deg
                <= policy.route_arc_step_deg / 2.0 + 1e-6)
            base_checks = {
                "endpoint_identity": bool(np.linalg.norm(
                    points[-1] - np.asarray(route.target_local_mm)) <= 1e-8),
                "exact_occ_core_with_chord_bound": (
                    core_margin >= -1e-9),
                "nonparent_copper_known_budget": (
                    nonparent_margin > known_budget + 1e-12),
                "intended_parent_endpoint_tangency": parent_contact_ok,
                "intended_parent_prefix_clear": (
                    parent_prefix_lower + 1e-9
                    >= graph.wire_diameter_mm),
                "analytic_c1_radius": c1_ok,
            }
            base_status = "PASS" if all(base_checks.values()) else "FAIL"
            case_key = [turn.turn_index, half_turn_index]
            if core_margin < minimum_core_margin:
                minimum_core_margin = core_margin
                minimum_core_case = case_key
            if nonparent_margin < minimum_nonparent_margin:
                minimum_nonparent_margin = nonparent_margin
                minimum_nonparent_case = case_key
            if parent_prefix_lower < minimum_parent_prefix:
                minimum_parent_prefix = parent_prefix_lower
                minimum_parent_case = case_key

            sign_rows = []
            for motion_sign in (-1, 1):
                current = build_current_half_obstacle(
                    graph, spec, turn.turn_index,
                    half_turn_index, motion_sign, policy)
                adjacent = adjacent_self_clearance(
                    points, current, graph.wire_diameter_mm, policy,
                    search_band_mm=max(
                        0.5, graph.wire_diameter_mm + known_budget + 0.1))
                current_lower = (
                    adjacent.minimum_centerline_distance_mm
                    - route.sampled_arc_chord_error_bound_mm
                    - obstacle_chord)
                current_margin = current_lower - graph.wire_diameter_mm
                sign_checks = {
                    "base_route": base_status == "PASS",
                    "full_current_half_present": (
                        len(current.points_local_mm) >= 2
                        and current.length_mm > 0.0),
                    "adjacent_self_limit_at_most_2d": (
                        adjacent.adjacency_limit_mm
                        <= 2.0 * graph.wire_diameter_mm + 1e-12),
                    "nonadjacent_current_half_known_budget": (
                        current_margin > known_budget + 1e-12),
                }
                sign_status = (
                    "PASS" if all(sign_checks.values()) else "FAIL")
                sign_row = {
                    "turn_index": turn.turn_index,
                    "physical_half_index": half_turn_index,
                    "motion_sign": motion_sign,
                    "status": sign_status,
                    "checks": sign_checks,
                    "current_half": {
                        "sha256": current.sha256,
                        "start_phase_rad": current.start_phase_rad,
                        "end_phase_rad": current.end_phase_rad,
                        "length_mm": current.length_mm,
                        "point_count": len(current.points_local_mm),
                    },
                    "adjacent_self": asdict(adjacent),
                    "continuous_lower_bound_mm": current_lower,
                    "margin_over_wire_mm": current_margin,
                    "margin_after_known_budget_mm": (
                        current_margin - known_budget),
                }
                sign_rows.append(sign_row)
                motion_sign_cases.append(sign_row)
                if current_margin < minimum_current_margin:
                    minimum_current_margin = current_margin
                    minimum_current_case = [
                        turn.turn_index, half_turn_index, motion_sign]

            record = {
                "turn_index": turn.turn_index,
                "half_turn_index": half_turn_index,
                "status": base_status,
                "checks": base_checks,
                "route": {
                    "sha256": route.sha256,
                    "points_local_mm": [list(point)
                                        for point in route.points_local_mm],
                    "point_count": len(route.points_local_mm),
                    "target_local_mm": list(route.target_local_mm),
                    "support_direction_local": list(
                        route.support_direction_local),
                    "support_cone_minimum_dot": (
                        route.support_cone_minimum_dot),
                    "minimum_bend_radius_mm": route.minimum_bend_radius_mm,
                    "bridge_length_mm": route.bridge_length_mm,
                    "tip_exit_tangent_error_deg": (
                        route.tip_exit_tangent_error_deg),
                    "sampled_join_tangent_error_deg": (
                        route.join_tangent_error_deg),
                    "sampled_terminal_tangent_error_deg": (
                        route.terminal_tangent_error_deg),
                    "sampled_arc_chord_error_bound_mm": (
                        route.sampled_arc_chord_error_bound_mm),
                },
                "exact_occ_core_raw_mm": core_raw,
                "continuous_core_lower_bound_mm": core_lower,
                "core_margin_mm": core_margin,
                "nonparent_raw_chordal_distance_mm": (
                    nonparent.minimum_centerline_distance_mm),
                "nonparent_obstacle_id": nonparent.obstacle_id,
                "continuous_nonparent_lower_bound_mm": nonparent_lower,
                "nonparent_margin_over_wire_mm": nonparent_margin,
                "nonparent_margin_after_known_budget_mm": (
                    nonparent_margin - known_budget),
                "parent_prefix_continuous_lower_bound_mm": (
                    None if not math.isfinite(parent_prefix_lower)
                    else parent_prefix_lower),
                "parent_prefix_obstacle_id": parent_prefix_obstacle,
                "endpoint_parent_centerline_distances_mm": (
                    endpoint_parent_distances),
                "motion_sign_cases": sign_rows,
            }
            records.append(record)
            if progress:
                print(
                    f"turn {turn.turn_index:02d} half {half_turn_index}: "
                    f"base={base_status} signs="
                    f"{sign_rows[0]['status']}/{sign_rows[1]['status']}",
                    flush=True)

    base_passed = sum(row["status"] == "PASS" for row in records)
    sign_passed = sum(row["status"] == "PASS"
                      for row in motion_sign_cases)
    sign_passed_by_sign = {
        str(sign): sum(
            row["status"] == "PASS"
            for row in motion_sign_cases
            if row["motion_sign"] == sign)
        for sign in (-1, 1)
    }
    base_failure_counts = {
        check: sum(not row["checks"][check] for row in records)
        for check in records[0]["checks"]
    }
    crown_audit = _deposited_crown_audit(
        planner, graph, spec, policy, obstacle_chord)
    release_flags = {
        "preserves_2d_packing": True,
        "measured_contract_bound": True,
        "turn42_turn45_seed_separation_at_least_1_25mm": (
            seed_separation + 1e-12 >= 1.25),
        "deposited_crowns_clear": crown_audit["status"] == "PASS",
        "all_100_base_routes": base_passed == 100,
        "analytic_c1_minimum_radius_3mm": all(
            row["checks"]["analytic_c1_radius"] for row in records),
        "deposited_crown_curvature_proven": (
            False if curvature_proof is None
            else curvature_proof["status"] == "PASS"),
        "machine_crown_envelope_validated": False,
        "exact_occ_core_all_routes": all(
            row["checks"]["exact_occ_core_with_chord_bound"]
            for row in records),
        "physical_error_budget_known_terms": all(
            row["checks"]["nonparent_copper_known_budget"]
            for row in records),
        "current_half_sign_specific": sign_passed == 200,
        "both_motion_directions": sign_passed == 200,
        # Hardware TIR/contact/instrument values remain intentionally unknown.
        "physical_error_budget_complete_measured_evidence": False,
    }
    status = "PASS" if all(release_flags.values()) else "FAIL"
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": status,
        "scope": {
            "preserved_geometry": "authoritative 2D packed side centers",
            "changed_geometry": (
                "outside-stack deposited crowns and moving span only"),
            "route_cases": 100,
            "motion_sign_cases": 200,
            "adjacent_self_rule": (
                "combined route plus current-half geodesic from shared "
                "endpoint; exempt only <=2 wire diameters"),
        },
        "inputs": {
            "packing_path": str(packing_path.relative_to(ROOT)).replace(
                "\\", "/"),
            "packing_file_sha256": _sha256(packing_path),
            "packing_report_sha256": graph.report_sha256,
            "packing_role": packing.get("role"),
            "input_provenance": packing.get("config", {}).get(
                "input_provenance"),
            "wire_finished_diameter_mm": graph.wire_diameter_mm,
            "liner_thickness_mm": float(
                packing["config"]["liner_thickness_mm"]),
            "required_core_centerline_mm": graph.center_core_access_mm,
        },
        "policy": policy.canonical_dict(),
        "crown_geometry_sha256": crown_hash,
        "turn42_extension_mm": (
            None if policy.geometry_family != "layer_tier_box"
            else policy.crown_extension_mm(turn42)),
        "turn45_extension_mm": (
            None if policy.geometry_family != "layer_tier_box"
            else policy.crown_extension_mm(turn45)),
        "turn42_positive_apex_local_z_mm": turn42_positive_apex,
        "turn45_positive_apex_local_z_mm": turn45_positive_apex,
        "turn42_seed_height_local_z_mm": turn42_seed_height,
        "turn45_seed_height_local_z_mm": turn45_seed_height,
        "turn42_turn45_seed_separation_mm": seed_separation,
        "deposited_crown_curvature_proof": curvature_proof,
        "crown_envelope_local_mm": {
            "minimum_xyz": crown_envelope_min.tolist(),
            "maximum_xyz": crown_envelope_max.tolist(),
            "status": (
                "NOT_PROVEN"
                if policy.geometry_family != "layer_tier_box"
                else "DIAGNOSTIC"),
        },
        "known_physical_error_lower_bound_mm": known_budget,
        "known_budget_scope": (
            "M0 controller acceptance plus selected encoder quantization; "
            "hardware TIR/contact/instrument evidence remains unknown"),
        "obstacle_chord_error_bound_mm": obstacle_chord,
        "required_nonparent_continuous_distance_mm": (
            graph.wire_diameter_mm + known_budget),
        "required_nonparent_raw_chordal_distance_mm": (
            graph.wire_diameter_mm + known_budget
            + obstacle_chord
            + max(row["route"]["sampled_arc_chord_error_bound_mm"]
                  for row in records)),
        "deposited_crown_audit": crown_audit,
        "routes": records,
        "validation": {
            "release_flags": release_flags,
            "base_route_passed": base_passed,
            "base_route_expected": 100,
            "motion_sign_passed": sign_passed,
            "motion_sign_expected": 200,
            "motion_sign_passed_by_sign": sign_passed_by_sign,
            "base_failure_counts_by_check": base_failure_counts,
            "minimum_core_margin_mm": minimum_core_margin,
            "minimum_core_case": minimum_core_case,
            "minimum_nonparent_margin_over_wire_mm": (
                minimum_nonparent_margin),
            "minimum_nonparent_case": minimum_nonparent_case,
            "minimum_parent_prefix_lower_bound_mm": (
                minimum_parent_prefix),
            "minimum_parent_prefix_case": minimum_parent_case,
            "minimum_current_half_margin_over_wire_mm": (
                minimum_current_margin),
            "minimum_current_half_case": minimum_current_case,
        },
        "limitations": [
            "Unknown hardware TIR, contact-position, and receipt-instrument "
            "uncertainties prevent complete physical budget authorization.",
            "A PASS is forbidden unless all 100 base routes and all 200 "
            "sign-specific arriving-current-half cases pass.",
            "The tested fixed terminal branch is C1 for one arriving-half "
            "orientation but collides with the complementary orientation; "
            "a both-direction release needs a sign-selectable guide branch "
            "or a different joint route/crown topology.",
            "Axial tiering alone does not separate all cross-layer crowns. "
            "The next crown family must add a smooth profile-indexed radial "
            "lane and then repeat the full pairwise/OCC/sign sweep.",
            "The packing-frame half-twist is a tangential hairpin.  Its "
            "minimum-radius reversal exceeds the adjacent-slot corridor and "
            "therefore remains blocked unless the active-versus-neighbor "
            "deposited-crown audit and a machine-envelope audit both pass.",
        ],
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--packing", type=Path, default=PACKING_PATH)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--progress", action="store_true")
    parser.add_argument(
        "--family",
        choices=(
            "layer_tier_box", "packing_frame_half_twist",
            "radial_axial_dubins"),
        default="layer_tier_box")
    parser.add_argument("--layer-step-mm", type=float, default=0.85)
    parser.add_argument("--bridge-mm", type=float, default=5.0)
    parser.add_argument("--half-twist-radius-mm", type=float, default=21.0)
    parser.add_argument("--half-twist-scale", type=float, default=3.5)
    parser.add_argument(
        "--half-twist-bridge-step-mm", type=float, default=0.025)
    parser.add_argument("--radial-axial-radius-mm", type=float, default=10.0)
    parser.add_argument("--radial-axial-bias-mm", type=float, default=10.0)
    parser.add_argument("--radial-axial-scale", type=float, default=3.5)
    parser.add_argument(
        "--radial-axial-arc-step-deg", type=float, default=0.5)
    args = parser.parse_args()
    if args.family == "packing_frame_half_twist":
        packing = json.loads(args.packing.read_text())
        spec = slot_wire_routes._validate_packing_contract(packing)
        graph = PackingSupportGraph.from_report(packing, spec=spec)
        policy = packing_frame_half_twist_policy(
            graph,
            base_radius_mm=args.half_twist_radius_mm,
            profile_scale=args.half_twist_scale,
            bridge_step_mm=args.half_twist_bridge_step_mm)
    elif args.family == "radial_axial_dubins":
        policy = radial_axial_dubins_policy(
            radius_mm=args.radial_axial_radius_mm,
            outward_bias_mm=args.radial_axial_bias_mm,
            profile_scale=args.radial_axial_scale,
            arc_step_deg=args.radial_axial_arc_step_deg)
    else:
        policy = CrownPolicy(
            layer_step_mm=args.layer_step_mm,
            bridge_length_mm=args.bridge_mm)
    output = (args.output if args.output is not None else
              (HALF_TWIST_OUTPUT_PATH
               if args.family == "packing_frame_half_twist"
               else (RADIAL_AXIAL_OUTPUT_PATH
                     if args.family == "radial_axial_dubins"
                     else OUTPUT_PATH)))
    report = analyze(args.packing, policy=policy, progress=args.progress)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, allow_nan=False))
    validation = report["validation"]
    print(f"wrote {output.resolve()}")
    print(
        f"crown routes {report['status']}: "
        f"base {validation['base_route_passed']}/100; "
        f"sign {validation['motion_sign_passed']}/200")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
