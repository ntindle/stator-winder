"""Fail-closed M2 wire-force torque audit for the offset-spoke flyer.

The incoming conductor enters the rotating flyer on the M2 axis.  Summing the
wire reactions over the *complete* rotating flyer therefore cancels every
internal guide reaction and leaves one external moment: the 10 N outgoing
strand force at the terminal toroid.  Its M2 torque is exactly

    tau_z = T * abs((r_exit x u_out)_z)

where ``u_out`` is the unit tangent from the toroid exit toward the work.

This audit keeps three scopes separate:

* the rejected current ellipse path, replayed only as a force-vector
  diagnostic over every canonical raw half-turn and every intervening degree;
* the authorized default-stator aggregate target-radius bound; and
* conservative planar tangent bounds for the GOAL OD65 launch envelope and
  parametric OD90 advisory envelope.

No CAD, controller source, settings, BOM, or procurement data is changed.
"""

from __future__ import annotations

from copy import deepcopy
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping

import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
PHASE_REPORT = REPORTS / "phase_aware_progressive_wire_audit.json"
AGGREGATE_REPORT = REPORTS / "permanent_cap_aggregate_authorization.json"
MANIFEST = ROOT / "out" / "links" / "manifest.json"
LOADS_SOURCE = CAD / "loads.py"
PARAMS_SOURCE = CAD / "params.py"
UPSTREAM_CONSTANTS = ROOT.parent / "winder" / "src" / "constants.py"
NEMA23_STEP = CAD / "models" / "upgrades" / "23HS22-4004D-E1000.step"
MOTOR_PROVENANCE_REPORT = CAD / "models" / "upgrades" / "motors.report.md"
JSON_OUT = REPORTS / "permanent_cap_offset_spoke_wire_force_torque.json"
MD_OUT = REPORTS / "permanent_cap_offset_spoke_wire_force_torque.md"

for path in (CAD, HERE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import loads  # noqa: E402
from params import DEFAULT_STATOR, PARAMS  # noqa: E402
import permanent_cap_offset_spoke_review as review  # noqa: E402
import wirepath  # noqa: E402


SCHEMA = "permanent-cap-offset-spoke-wire-force-torque/v1"
EXPECTED_CAPTURE_SHA256 = (
    "350210c36550ae2c22e5352675aee6bd2eebb18325f3b5f45c7cb35d6a314958"
)
MOTOR_NAME = "NEMA17 McMaster 6627T421 encoder motor @24V (M2)"
WIRE_TENSION_N = 10.0
RPM = 300.0
ACCEL_RAD_S2 = 200.0
ANGLE_STEP_DEG = 1.0
EXPECTED_PASSES = 24
EXPECTED_HALF_TURN_LOCI = 2400
HALF_TURNS_PER_PASS = 100

# Independent dry-rotor review bound.  This deliberately includes the complete
# two 6001 bearing envelopes, belt, and motor pulley as if all their mass
# rotated at flyer speed.  It does not include the selected motor's unknown
# rotor inertia, the not-yet-defined transition guide/adhesive, or omitted
# motor-pulley set screws; those remain explicit blockers below.
KNOWN_ROTATING_INERTIA_UPPER_BOUND_KG_M2 = 8.76566e-5

# A design allowance, not a measured release value, for two bearings, belt
# flexure, and guide drag.  Installed breakaway/running friction must be shown
# not to exceed this number before the numeric margins can be released.
FRICTION_ALLOWANCE_NM = 0.020


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} is not a JSON object")
    return value


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    body = deepcopy(dict(payload))
    body.pop("report_sha256", None)
    return hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def _unit(vector: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm <= 1.0e-12:
        raise ValueError("zero-length force vector")
    return vector / norm


def offset_guide_spec() -> dict[str, Any]:
    """Exact isolated-review toroid with an on-axis incoming boundary."""

    return {
        "model": "R64 offset toroid; whole-flyer incoming boundary at M2 axis",
        "center_local_mm": [
            0.0, review.TIP_GUIDE_CENTER_RADIUS_MM,
            review.TIP_GUIDE_CENTER_Z_MM,
        ],
        "axis_local": [0.0, 1.0, 0.0],
        "feed_local_mm": [0.0, 0.0, review.TIP_GUIDE_CENTER_Z_MM],
        "major_radius_mm": 6.5,
        "tube_radius_mm": 3.0,
    }


def exact_path_force_witness(angle_rad: float, motion_sign: int,
                             contact: Mapping[str, Any]) -> dict[str, Any]:
    """Exact current-diagnostic toroid exit and outgoing force vector."""

    guide = offset_guide_spec()
    rotation = wirepath.rot_z(float(angle_rad))
    feed = rotation @ np.asarray(guide["feed_local_mm"], dtype=float)
    center = rotation @ np.asarray(guide["center_local_mm"], dtype=float)
    target = wirepath.tooth_contact_point(center, contact, int(motion_sign))
    path, metadata = wirepath.tip_guide_path(
        feed, target, guide, float(DEFAULT_STATOR.wire_d) / 2.0, rotation,
        arc_step_deg=1.0,
    )
    exit_point = np.asarray(path[-2], dtype=float)
    tangent = _unit(np.asarray(path[-1], dtype=float) - exit_point)
    cross_z_mm = float(
        exit_point[0] * tangent[1] - exit_point[1] * tangent[0]
    )
    effective_lever_mm = abs(cross_z_mm)
    return {
        "flyer_angle_deg": math.degrees(float(angle_rad)) % 360.0,
        "motion_sign": int(motion_sign),
        "incoming_boundary_mm": feed.tolist(),
        "incoming_boundary_radius_mm": float(np.linalg.norm(feed[:2])),
        "toroid_center_mm": center.tolist(),
        "toroid_exit_mm": exit_point.tolist(),
        "target_mm": np.asarray(target, dtype=float).tolist(),
        "outgoing_unit_tangent": tangent.tolist(),
        "signed_cross_z_mm": cross_z_mm,
        "effective_line_of_action_distance_mm": effective_lever_mm,
        "wire_torque_at_10N_nm": WIRE_TENSION_N * effective_lever_mm / 1000.0,
        "guide_arc_turn_deg": float(metadata["arc_turn_deg"]),
        "polyline_exit_tangent_discretization_error_deg": float(
            metadata["exit_tangent_error"]
        ),
    }


def _diagnostic_raw_sweep(phase: Mapping[str, Any],
                          manifest: Mapping[str, Any]) -> dict[str, Any]:
    records = phase["locus_records"]
    if len(records) != EXPECTED_HALF_TURN_LOCI:
        raise ValueError("phase report is not the canonical 2400-locus replay")
    contact = manifest["wire"]["tooth_contact"]

    # Exact 1-degree templates.  Each canonical deposition interval begins at
    # an integer multiple of 180 degrees and moves monotonically through 180
    # one-degree cells, so these 720 exact path solutions cover every angle in
    # all 2400 half-turn intervals without pretending 432,000 duplicated path
    # solves are independent geometry.
    templates: dict[tuple[int, int], dict[str, Any]] = {}
    for motion_sign in (-1, 1):
        for angle_deg in range(360):
            templates[(motion_sign, angle_deg)] = exact_path_force_witness(
                math.radians(angle_deg), motion_sign, contact,
            )

    raw_worst: tuple[float, dict[str, Any], dict[str, Any]] | None = None
    pass_indices: set[int] = set()
    phase_indices: set[int] = set()
    signs = {-1: 0, 1: 0}
    for record in records:
        locus = record["locus"]
        sign = int(locus["motion_sign"])
        angle = int(round(math.degrees(float(locus["m2_rad"])))) % 360
        witness = templates[(sign, angle)]
        value = float(witness["effective_line_of_action_distance_mm"])
        if raw_worst is None or value > raw_worst[0]:
            raw_worst = (value, locus, witness)
        pass_indices.add(int(locus["pass_index"]))
        phase_indices.add(int(locus["phase_index"]))
        signs[sign] += 1
    if raw_worst is None:
        raise RuntimeError("no raw force-vector witness")

    continuous_worst: tuple[float, int, int, int, dict[str, Any]] | None = None
    evaluated = 0
    for record in records:
        locus = record["locus"]
        sign = int(locus["motion_sign"])
        start = int(round(math.degrees(float(locus["m2_rad"]))))
        for offset_deg in range(180):
            angle = (start + sign * offset_deg) % 360
            witness = templates[(sign, angle)]
            value = float(witness["effective_line_of_action_distance_mm"])
            candidate = (
                value, int(locus["pass_index"]), int(locus["state_index"]),
                angle, witness,
            )
            if continuous_worst is None or value > continuous_worst[0]:
                continuous_worst = candidate
            evaluated += 1
    if continuous_worst is None:
        raise RuntimeError("no continuous raw force-vector witness")

    return {
        "authority": "DIAGNOSTIC_ONLY_CURRENT_ELLIPSE_PATH_IS_REJECTED",
        "reason": (
            "The phase-aware report proves this stored ellipse terminal path "
            "crosses core; this replay validates force-vector coverage and "
            "does not authorize that wire path."
        ),
        "pass_count": len(pass_indices),
        "phase_indices": sorted(phase_indices),
        "half_turn_locus_count": len(records),
        "motion_sign_locus_counts": {str(k): v for k, v in signs.items()},
        "angle_step_deg": ANGLE_STEP_DEG,
        "unique_exact_path_templates": len(templates),
        "all_interval_angle_evaluations": evaluated,
        "expected_all_interval_angle_evaluations": (
            EXPECTED_HALF_TURN_LOCI * 180
        ),
        "raw_locus_worst": {
            "effective_line_of_action_distance_mm": raw_worst[0],
            "locus": raw_worst[1],
            "path_force_witness": raw_worst[2],
        },
        "continuous_one_degree_worst": {
            "effective_line_of_action_distance_mm": continuous_worst[0],
            "pass_index": continuous_worst[1],
            "half_turn_state_index": continuous_worst[2],
            "flyer_angle_deg": continuous_worst[3],
            "path_force_witness": continuous_worst[4],
        },
    }


@lru_cache(maxsize=8)
def _planar_tangent_witness(radius_mm: float) -> dict[str, Any]:
    """Construct an exact worst-planar outgoing tangent to a centered circle.

    The toroid exit itself depends on the target direction, so solve the
    target angle for perpendicularity between target radius and outgoing unit
    tangent.  The resulting cross product independently recovers ``radius``.
    """

    radius = float(radius_mm)
    if not 0.0 < radius < review.TIP_GUIDE_CENTER_RADIUS_MM:
        raise ValueError("tangent bound radius must be inside the R64 guide")
    guide = offset_guide_spec()
    feed = np.asarray(guide["feed_local_mm"], dtype=float)

    def solve_at(theta: float) -> tuple[float, np.ndarray, np.ndarray,
                                        dict[str, Any]]:
        target = np.asarray([
            radius * math.cos(theta), radius * math.sin(theta),
            review.TIP_GUIDE_CENTER_Z_MM,
        ])
        path, metadata = wirepath.tip_guide_path(
            feed, target, guide, float(DEFAULT_STATOR.wire_d) / 2.0,
            np.eye(3), arc_step_deg=1.0,
        )
        exit_point = np.asarray(path[-2], dtype=float)
        tangent = _unit(target - exit_point)
        return float(np.dot(target[:2], tangent[:2])), exit_point, tangent, metadata

    # One-degree bracketing followed by bisection is both deterministic and
    # far tighter than the dimensional uncertainty in the not-yet-made guide.
    grid = np.linspace(-math.pi, math.pi, 361)
    values = [solve_at(float(theta))[0] for theta in grid]
    roots: list[float] = []
    for index in range(len(grid) - 1):
        lo, hi = float(grid[index]), float(grid[index + 1])
        flo, fhi = values[index], values[index + 1]
        if flo == 0.0:
            roots.append(lo)
            continue
        if flo * fhi > 0.0:
            continue
        for _ in range(60):
            mid = (lo + hi) / 2.0
            fmid = solve_at(mid)[0]
            if flo * fmid <= 0.0:
                hi, fhi = mid, fmid
            else:
                lo, flo = mid, fmid
        roots.append((lo + hi) / 2.0)
    candidates = []
    for theta in roots:
        target = np.asarray([
            radius * math.cos(theta), radius * math.sin(theta),
            review.TIP_GUIDE_CENTER_Z_MM,
        ])
        if target[0] <= 0.0:
            continue
        residual, exit_point, tangent, metadata = solve_at(theta)
        lever = abs(float(
            exit_point[0] * tangent[1] - exit_point[1] * tangent[0]
        ))
        candidates.append((abs(lever - radius), theta, target, residual,
                           exit_point, tangent, metadata, lever))
    if not candidates:
        raise RuntimeError(f"no positive-X tangent solution for R{radius:g}")
    _, theta, target, residual, exit_point, tangent, metadata, lever = min(
        candidates, key=lambda row: row[0]
    )
    return {
        "bound_kind": (
            "exact worst-planar tangent; any outgoing line meeting a target "
            "inside this centered radius has no larger M2 moment arm"
        ),
        "target_radius_bound_mm": radius,
        "target_angle_deg": math.degrees(theta),
        "incoming_boundary_mm": feed.tolist(),
        "incoming_boundary_radius_mm": float(np.linalg.norm(feed[:2])),
        "toroid_exit_mm": exit_point.tolist(),
        "target_mm": target.tolist(),
        "outgoing_unit_tangent": tangent.tolist(),
        "tangent_perpendicularity_residual_mm": residual,
        "effective_line_of_action_distance_mm": lever,
        "lever_minus_radius_error_mm": lever - radius,
        "wire_torque_at_10N_nm": WIRE_TENSION_N * lever / 1000.0,
        "guide_arc_turn_deg": float(metadata["arc_turn_deg"]),
    }


def _duty_case(name: str, radius_mm: float, scope: str,
               raw_capture_supports: bool) -> dict[str, Any]:
    witness = _planar_tangent_witness(radius_mm)
    wire_torque = float(witness["wire_torque_at_10N_nm"])
    accel_torque = KNOWN_ROTATING_INERTIA_UPPER_BOUND_KG_M2 * ACCEL_RAD_S2
    known_required = wire_torque + accel_torque + FRICTION_ALLOWANCE_NM
    motor_available = loads.torque_at_rpm(MOTOR_NAME, RPM)
    pulley_capacity = float(PARAMS.m2_motor_pulley_capacity_nm)
    return {
        "name": name,
        "scope": scope,
        "raw_capture_supports_this_stator": bool(raw_capture_supports),
        "force_vector": witness,
        "wire_tension_N": WIRE_TENSION_N,
        "known_rotating_inertia_upper_bound_kg_m2": (
            KNOWN_ROTATING_INERTIA_UPPER_BOUND_KG_M2
        ),
        "acceleration_rad_s2": ACCEL_RAD_S2,
        "known_acceleration_torque_nm": accel_torque,
        "friction_allowance_nm": FRICTION_ALLOWANCE_NM,
        "known_load_required_torque_nm": known_required,
        "selected_motor_available_torque_at_300rpm_nm": motor_available,
        "selected_pulley_capacity_nm": pulley_capacity,
        "known_load_motor_margin": motor_available / known_required,
        "known_load_pulley_margin": pulley_capacity / known_required,
        "known_load_motor_margin_ge_2": motor_available / known_required >= 2.0,
        "known_load_pulley_margin_ge_2": pulley_capacity / known_required >= 2.0,
        "minimum_motor_running_torque_for_2x_nm": 2.0 * known_required,
        "minimum_pulley_and_belt_output_capacity_for_2x_nm": (
            2.0 * known_required
        ),
        "margin_is_optimistic_until_motor_rotor_inertia_is_bounded": True,
    }


def analyze() -> dict[str, Any]:
    phase = _load(PHASE_REPORT)
    aggregate = _load(AGGREGATE_REPORT)
    manifest = _load(MANIFEST)
    diagnostic = _diagnostic_raw_sweep(phase, manifest)

    raw_contract = aggregate["canonical_raw_capture"]
    default_radius = float(
        aggregate["aggregate_loft"]["tooth_owned_front_crown_cell"]
        ["OD_center_limit_mm"]
    )
    default = _duty_case(
        "canonical_default_OD46_aggregate_target_bound", default_radius,
        "canonical unmodified OD46 24x50 raw cycle only", True,
    )
    launch65 = _duty_case(
        "GOAL_launch_OD65_conservative_target_bound", 65.0 / 2.0,
        "GOAL launch envelope; not proved by the OD46 raw capture", False,
    )
    advisory90 = _duty_case(
        "parametric_OD90_advisory_target_bound", 90.0 / 2.0,
        "parametric advisory only; not a launch gate", False,
    )

    accel_torque = KNOWN_ROTATING_INERTIA_UPPER_BOUND_KG_M2 * ACCEL_RAD_S2
    motor_available = loads.torque_at_rpm(MOTOR_NAME, RPM)
    pulley_capacity = float(PARAMS.m2_motor_pulley_capacity_nm)
    motor_line_limit = (
        motor_available / 2.0 - accel_torque - FRICTION_ALLOWANCE_NM
    ) * 1000.0 / WIRE_TENSION_N
    pulley_line_limit = (
        pulley_capacity / 2.0 - accel_torque - FRICTION_ALLOWANCE_NM
    ) * 1000.0 / WIRE_TENSION_N
    direct_wire_torque = (
        WIRE_TENSION_N * review.TIP_GUIDE_CENTER_RADIUS_MM / 1000.0
    )
    direct_required = direct_wire_torque + accel_torque + FRICTION_ALLOWANCE_NM
    direct = {
        "scope": "unconstrained pure-tangential R64 outgoing segment",
        "effective_line_of_action_distance_mm": (
            review.TIP_GUIDE_CENTER_RADIUS_MM
        ),
        "wire_torque_at_10N_nm": direct_wire_torque,
        "known_load_required_torque_nm": direct_required,
        "known_load_motor_margin": motor_available / direct_required,
        "known_load_pulley_margin": pulley_capacity / direct_required,
        "minimum_drive_capacity_for_2x_nm": 2.0 * direct_required,
    }

    gates = {
        "canonical_raw_capture_sha256_exact": (
            _sha256(CAPTURE) == EXPECTED_CAPTURE_SHA256
            and raw_contract["sha256"] == EXPECTED_CAPTURE_SHA256
        ),
        "all_24_passes_and_2400_half_turn_loci_bound": (
            diagnostic["pass_count"] == EXPECTED_PASSES
            and diagnostic["half_turn_locus_count"] == EXPECTED_HALF_TURN_LOCI
        ),
        "every_captured_half_turn_interval_sampled_at_1deg": (
            diagnostic["all_interval_angle_evaluations"]
            == EXPECTED_HALF_TURN_LOCI * 180
        ),
        "whole_flyer_incoming_boundary_crosses_M2_axis": (
            diagnostic["continuous_one_degree_worst"]
            ["path_force_witness"]["incoming_boundary_radius_mm"] <= 1.0e-12
        ),
        "canonical_default_known_load_motor_margin_ge_2": (
            default["known_load_motor_margin_ge_2"]
        ),
        "canonical_default_known_load_pulley_margin_ge_2": (
            default["known_load_pulley_margin_ge_2"]
        ),
        "GOAL_OD65_known_load_motor_margin_ge_2": (
            launch65["known_load_motor_margin_ge_2"]
        ),
        "GOAL_OD65_known_load_pulley_margin_ge_2": (
            launch65["known_load_pulley_margin_ge_2"]
        ),
        "actual_successor_outgoing_path_and_transition_guide_defined": False,
        "transition_guide_adhesive_and_motor_pulley_set_screw_mass_bound": False,
        "selected_motor_rotor_inertia_bound": False,
        "installed_M2_friction_measured_le_0p020Nm": False,
        "NEMA23_candidate_dynamic_curve_at_300rpm_verified": False,
        "frozen_upstream_exact_1_to_1_ratio_preserved": (
            math.isclose(float(PARAMS.m2_gear_ratio), 1.0, abs_tol=1.0e-12)
        ),
    }
    blockers = [name for name, value in gates.items() if not value]
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "FAIL_CLOSED",
        "decision": "CURRENT_M2_DRIVE_NOT_AUTHORIZED_FOR_FULL_OD65_LAUNCH_ENVELOPE",
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "physics": {
            "whole_flyer_free_body": (
                "internal wire/guide forces telescope; incoming wire crosses "
                "the M2 axis, so tau_z = T*abs((r_exit x u_out)_z)"
            ),
            "incoming_external_torque_nm": 0.0,
            "wire_tension_N": WIRE_TENSION_N,
            "rpm": RPM,
        },
        "canonical_raw_diagnostic": diagnostic,
        "duty_cases": {
            "canonical_default_OD46": default,
            "GOAL_launch_OD65": launch65,
            "parametric_OD90_advisory": advisory90,
            "unconstrained_direct_R64": direct,
        },
        "line_of_action_limits_for_current_1_to_1_drive": {
            "motor_controlling_maximum_mm_for_2x_known_load": motor_line_limit,
            "pulley_maximum_mm_for_2x_known_load": pulley_line_limit,
            "required_OD65_successor_proof": (
                "prove the exact outgoing line of action remains at or below "
                f"{motor_line_limit:.6f} mm over the complete OD65 job"
            ),
        },
        "drive_recommendation": {
            "geometry_first_option": (
                "retain the current exact 1:1 drive only if the production "
                "OD65 cap/guide route proves the motor-controlling line-of-"
                f"action bound <= {motor_line_limit:.6f} mm and all missing "
                "inertia/friction terms fit the remaining torque budget"
            ),
            "otherwise_required_motor": (
                "stronger closed-loop motor with documented dynamic running "
                f"torque > {launch65['minimum_motor_running_torque_for_2x_nm']:.6f} "
                "Nm at 300 RPM, plus its rotor-acceleration term"
            ),
            "otherwise_required_transmission": (
                "matched exact 1:1 pulleys and belt with output capacity > "
                f"{launch65['minimum_pulley_and_belt_output_capacity_for_2x_nm']:.6f} Nm"
            ),
            "48T_flyer_40T_motor_selected": False,
            "48T_40T_reason_rejected": (
                "winder/src/constants.py fixes m2_gear_ratio = 50/50 and the "
                "unmodified upstream command radians are flyer radians; no "
                "settings-only ratio mapping exists in the captured contract"
            ),
            "unapproved_exact_1_to_1_NEMA23_candidate": {
                "status": "CANDIDATE_ONLY__300RPM_MOTOR_CURVE_NOT_VERIFIED",
                "selected": False,
                "motor": "StepperOnline 23HS22-4004D-E1000 closed-loop NEMA23",
                "holding_torque_nm_not_accepted_as_running_torque": 1.2,
                "rated_current_A": 4.0,
                "encoder_PPR": 1000,
                "body_xy_mm": [57.0, 57.0],
                "body_plus_encoder_length_mm": 81.0,
                "shaft_diameter_mm": 8.0,
                "shaft_extension_mm": 21.0,
                "minimum_required_dynamic_motor_torque_at_300rpm_nm": (
                    launch65["minimum_motor_running_torque_for_2x_nm"]
                ),
                "motor_curve_gate": False,
                "motor_curve_required_proof": (
                    "fetch and digitize the official driver-condition torque "
                    "curve; conservative running torque at 300 RPM must exceed "
                    f"{launch65['minimum_motor_running_torque_for_2x_nm']:.6f} Nm "
                    "plus the candidate rotor-acceleration term"
                ),
                "exact_STEP_path": (
                    "cad/models/upgrades/23HS22-4004D-E1000.step"
                ),
                "exact_STEP_sha256": _sha256(NEMA23_STEP),
                "product_url": (
                    "https://www.omc-stepperonline.com/s-series-nema-23-closed-"
                    "loop-stepper-motor-1-2-nm-170oz-in-encoder-1000ppr-4000cpr-"
                    "23hs22-4004d-e1000"
                ),
                "STEP_url": (
                    "https://www.omc-stepperonline.com/index.php?route=product/"
                    "product/get_file&file=1446/23HS22-4004D-E1000.STEP"
                ),
                "transmission": {
                    "ratio": 1.0,
                    "motor_pulley": "NBK P30-3GT-BLP-6C-8",
                    "flyer_pulley": "matching 30T 3GT 6 mm exact-1:1 pulley",
                    "belt": "210-3GT-6",
                    "center_distance_mm": 60.0,
                    "pitch_circumference_each_pulley_mm": 90.0,
                    "closed_belt_pitch_length_derivation_mm": "2*60 + 90 = 210",
                    "pulley_outside_diameter_mm": 32.0,
                    "pulley_overall_width_mm": 11.0,
                    "pulley_channel_width_mm": 7.3,
                    "official_P30_base_T_Tr_at_300rpm_nm": 2.06,
                    "210mm_belt_length_factor_K_L": 0.9,
                    "engagement_factor_K_m": 1.0,
                    "allowable_torque_at_300rpm_nm": 1.854,
                    "pulley_capacity_ge_required_2x": (
                        1.854 >= launch65[
                            "minimum_pulley_and_belt_output_capacity_for_2x_nm"
                        ]
                    ),
                    "pulley_family_url": (
                        "https://www.nbk1560.com/products/pulley/timingpulley/"
                        "3GT-BLP-6C/P30-3GT-BLP-6C/"
                    ),
                    "official_capacity_table_pdf": (
                        "https://www.nbk1560.com/images/ja-JP/product/"
                        "timingpulley/3GT-BLP-6C/3GT-BLP-6C_1.pdf"
                    ),
                },
                "remaining_geometry_gates": [
                    "regenerate M2 mount for 57 mm frame and 8 mm shaft",
                    "sweep NEMA23 body, encoder, and cable against shifted frame",
                    "regenerate matched 30T flyer pulley and 210-3GT-6 belt",
                    "rerun belt, set-screw, collision, inertia, and assembly audits",
                ],
            },
        },
        "gates": gates,
        "controlling_blockers": blockers,
        "source_contracts": {
            "capture_path": "out/capture/upstream_current_raw.jsonl",
            "capture_sha256": _sha256(CAPTURE),
            "phase_report_sha256": _sha256(PHASE_REPORT),
            "aggregate_report_sha256": _sha256(AGGREGATE_REPORT),
            "manifest_sha256": _sha256(MANIFEST),
            "loads_source_sha256": _sha256(LOADS_SOURCE),
            "params_source_sha256": _sha256(PARAMS_SOURCE),
            "upstream_constants_sha256": _sha256(UPSTREAM_CONSTANTS),
            "NEMA23_exact_STEP_sha256": _sha256(NEMA23_STEP),
            "motor_provenance_report_sha256": _sha256(MOTOR_PROVENANCE_REPORT),
            "selected_motor": MOTOR_NAME,
            "selected_motor_curve_source": (
                "cad/models/upgrades/6627T421_torque_curve.png"
            ),
            "selected_motor_available_at_300rpm_nm": motor_available,
            "selected_pulley": "NBK P40-2GT-BLP-6C-5",
            "selected_pulley_capacity_nm": pulley_capacity,
        },
        "limits": [
            "The OD46 ellipse replay is diagnostic only because that path crosses core.",
            "The OD65 and OD90 cases are conservative centered target-radius bounds, not raw captures.",
            "The reported known-load margins omit unknown motor rotor inertia and are therefore optimistic.",
            "The 0.020 Nm friction term is an allowance that requires installed measurement.",
            "No exact successor transition guide, adhesive, or individual aggregate strand route exists yet.",
            "The NEMA23/30T-3GT candidate is not selected until its 300 RPM running curve and complete installed geometry are verified.",
        ],
        "source_hashes": {
            "sim/permanent_cap_offset_spoke_wire_force_torque.py": _sha256(Path(__file__)),
            "cad/permanent_cap_offset_spoke_review.py": _sha256(Path(review.__file__)),
            "cad/loads.py": _sha256(LOADS_SOURCE),
            "cad/params.py": _sha256(PARAMS_SOURCE),
            "winder/src/constants.py": _sha256(UPSTREAM_CONSTANTS),
            "cad/models/upgrades/23HS22-4004D-E1000.step": _sha256(NEMA23_STEP),
            "cad/models/upgrades/motors.report.md": _sha256(
                MOTOR_PROVENANCE_REPORT
            ),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def render_markdown(report: Mapping[str, Any]) -> str:
    default = report["duty_cases"]["canonical_default_OD46"]
    launch = report["duty_cases"]["GOAL_launch_OD65"]
    advisory = report["duty_cases"]["parametric_OD90_advisory"]
    direct = report["duty_cases"]["unconstrained_direct_R64"]
    limits = report["line_of_action_limits_for_current_1_to_1_drive"]
    lines = [
        "# Offset-spoke whole-flyer wire-force torque audit",
        "",
        f"Status: **{report['status']}** — `{report['decision']}`.",
        "",
        "The incoming strand crosses the M2 axis, so its external torque is zero. "
        "For the complete flyer, the wire torque is exactly 10 N times the outgoing "
        "segment's line-of-action distance from M2.",
        "",
        "| scope | line distance | known required torque | motor margin | pulley margin |",
        "|---|---:|---:|---:|---:|",
    ]
    for label, case in (
        ("canonical OD46 aggregate bound", default),
        ("GOAL OD65 launch bound", launch),
        ("OD90 advisory bound", advisory),
    ):
        lines.append(
            f"| {label} | {case['force_vector']['effective_line_of_action_distance_mm']:.6f} mm "
            f"| {case['known_load_required_torque_nm']:.6f} Nm "
            f"| {case['known_load_motor_margin']:.3f}x "
            f"| {case['known_load_pulley_margin']:.3f}x |"
        )
    lines.extend([
        f"| unconstrained R64 direct lever | {direct['effective_line_of_action_distance_mm']:.6f} mm "
        f"| {direct['known_load_required_torque_nm']:.6f} Nm "
        f"| {direct['known_load_motor_margin']:.3f}x "
        f"| {direct['known_load_pulley_margin']:.3f}x |",
        "",
        "The canonical replay binds all 24 passes, 2,400 half-turn loci, both "
        "directions, and 432,000 one-degree interval evaluations. Its ellipse "
        "geometry remains rejected and is not launch authority.",
        "",
        f"The present motor is the controlling component: to retain it, the exact "
        f"OD65 outgoing line of action must remain <= "
        f"{limits['motor_controlling_maximum_mm_for_2x_known_load']:.6f} mm. "
        "Otherwise use a stronger closed-loop motor and higher-capacity matched "
        "1:1 pulleys/belt. A 48T/40T reduction is not selected because it violates "
        "the frozen upstream 1:1 radians contract.",
        "",
        "The existing StepperOnline 23HS22-4004D-E1000 STEP plus an exact-1:1 "
        "30T/30T 3GT drive is recorded as a candidate, not a selection. The NBK "
        "P30 base T_Tr is 2.06 Nm at 300 RPM; the 210 mm belt's K_L=0.9 "
        "reduces allowable torque to 1.854 Nm, still clearing the threshold. The "
        "the motor's dynamic 300 RPM curve and the larger NEMA23 installation "
        "remain unverified.",
        "",
        "Reported margins include a 0.020 Nm friction allowance and the reviewed "
        "known rotating inertia, but still omit unknown motor rotor inertia; they "
        "are optimistic until that term and installed friction are bounded.",
        "",
        "## Controlling blockers",
        "",
    ])
    lines.extend(f"- `{name}`" for name in report["controlling_blockers"])
    lines.extend(["", f"Report SHA-256: `{report['report_sha256']}`", ""])
    return "\n".join(lines)


def write_reports(report: Mapping[str, Any] | None = None) -> dict[str, Any]:
    result = analyze() if report is None else dict(report)
    REPORTS.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8",
    )
    MD_OUT.write_text(render_markdown(result), encoding="utf-8")
    return result


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unexpected wire-force torque schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("wire-force torque report hash mismatch")
    if report.get("status") != "FAIL_CLOSED":
        raise ValueError("unresolved OD65 force-vector audit must fail closed")
    if not report.get("controlling_blockers"):
        raise ValueError("fail-closed torque audit has no blockers")


def main() -> int:
    report = write_reports()
    validate_report_integrity(report)
    launch = report["duty_cases"]["GOAL_launch_OD65"]
    print(
        f"offset wire-force torque: {report['status']}; "
        f"OD65={launch['known_load_required_torque_nm']:.6f} Nm; "
        f"motor={launch['known_load_motor_margin']:.3f}x; "
        f"pulley={launch['known_load_pulley_margin']:.3f}x"
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
