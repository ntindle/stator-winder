"""Deterministic, review-only M2 production-configuration trade study.

This module answers a narrow follow-up question without changing the selected
normal-GOAL drive, CAD, BOM, load authority, controller contract, or release
state.  It evaluates:

* exact CS-D508 PR2 settings around the CS-M21708 RMS 2.5 A curve condition;
* stock NBK P28 and P26 motor pulleys with the retained P30 flyer pulley; and
* the stronger Leadshine CS-M22313 NEMA23 at an exact 1:1 ratio.

Every alternative fails at least one frozen requirement.  The report is a
review record, not a selection or an authorization to integrate or procure.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

import m2_normal_goal_drive_selection as selected


ROOT = Path(__file__).resolve().parent.parent
WORKSPACE = ROOT.parent
REPORTS = ROOT / "out" / "reports"
OUTPUT_JSON = REPORTS / "m2_exact_configuration_trade_study.json"
OUTPUT_MD = REPORTS / "m2_exact_configuration_trade_study.md"

SELECTED_REPORT = REPORTS / "m2_normal_goal_drive_selection.json"
GOAL = WORKSPACE / "GOAL.md"
REQUIREMENTS = ROOT / "docs" / "requirements.md"
UPSTREAM_CONFIG = WORKSPACE / "winder-goal1-contract" / "src" / "config.py"

M22313_ZIP = ROOT / "tmp" / "CS-M22313_3D.zip"
M22313_STEP = ROOT / "tmp" / "CS-M22313_3D" / "CS-M22313_3D.STEP"
M22313_DRAWING = ROOT / "tmp" / "CS-M22313_MS31.pdf"
M22313_CURVE = ROOT / "tmp" / "Leadshine_CS-M22313_torque_curve.png"

EXPECTED_PRODUCT_HASHES = {
    "tmp/CS-M22313_3D.zip": (
        "d3c4c17dabcd9fbb94eb5ab4fa0a1265841c74e0b22296331037269443bbb516"
    ),
    "tmp/CS-M22313_3D/CS-M22313_3D.STEP": (
        "01813231d4bd0c1de12f966f8c7352467a757157c81ae43ee2031cb774a4b5e5"
    ),
    "tmp/CS-M22313_MS31.pdf": (
        "5670b96517feefcd81a284ef419da998d4c84325055247342661216b6c7f15e7"
    ),
    "tmp/Leadshine_CS-M22313_torque_curve.png": (
        "d6dc3b643d0f17546922f7d96ad8a2824fc3cb784902d2deb112e6ecbe711358"
    ),
}

PRODUCT_URLS = {
    "Leadshine_CS_M21708": (
        "https://m.leadshine.com/product-detail/CS-M21708.html"
    ),
    "Leadshine_CS_D508": "https://www.leadshine.com/product-detail/CS-D508.html",
    "Leadshine_CS_D508_software_manual": (
        "https://www.leadshine.com/upfiles/downloads/"
        "14419125f0bab74945cb1577be162c56_1665570154673.pdf"
    ),
    "NBK_P28": (
        "https://www.nbk1560.com/products/pulley/timingpulley/"
        "3GT-BLP-6C/P28-3GT-BLP-6C/"
    ),
    "NBK_P26": (
        "https://www.nbk1560.com/products/pulley/timingpulley/"
        "3GT-BLP-6C/P26-3GT-BLP-6C/"
    ),
    "NBK_P30": (
        "https://www.nbk1560.com/products/pulley/timingpulley/"
        "3GT-BLP-6C/P30-3GT-BLP-6C/"
    ),
    "NBK_3GT_6_belt": (
        "https://www.nbk1560.com/products/pulley/timingpulley_option/3GT-6/"
    ),
    "Leadshine_CS_M22313": (
        "https://en.leadshine.com/product-detail/m-closed-loop-stepper-drives/"
        "Modbus-RTU/CS-M22313.html"
    ),
    "Leadshine_CS_M22313_STEP_zip": (
        "https://en.leadshine.com/upfiles/downloads/"
        "5ad29ca8576efdb4be7deee7435bd869_1741331402019.zip"
    ),
    "Leadshine_CS_M22313_drawing": (
        "https://en.leadshine.com/upfiles/downloads/"
        "c46316f861be0d38fb146c8af98101fe_1651889790526.pdf"
    ),
    "Leadshine_CS_M22313_curve": (
        "https://en.leadshine.com/downUeditor/image/20220615/"
        "e7a23dfc1f79f6e4574fb9c69de17ffc.png"
    ),
}

MIN_CLEARANCE_MM = 2.0
PITCH_MM = 3.0
BELT_PITCH_LENGTH_MM = 210.0
FLYER_TEETH = 30
BASE_CENTER_DISTANCE_MM = 60.0
CURVE_TOTAL_HAIRCUT = 0.10


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT)).replace("\\", "/")
    except ValueError:
        return str(path.relative_to(WORKSPACE)).replace("\\", "/")


def _open_belt_geometry(motor_teeth: int) -> dict[str, float]:
    """Exact pitch-line geometry for a 210 mm open 3GT belt."""
    motor_d = motor_teeth * PITCH_MM / math.pi
    flyer_d = FLYER_TEETH * PITCH_MM / math.pi
    delta = flyer_d - motor_d
    reduced_length = BELT_PITCH_LENGTH_MM - math.pi * (
        flyer_d + motor_d
    ) / 2.0
    discriminant = reduced_length**2 - 2.0 * delta**2
    if discriminant <= 0.0:
        raise ValueError("belt length cannot span the selected pitch diameters")
    center = (reduced_length + math.sqrt(discriminant)) / 4.0
    small_wrap_rad = math.pi - 2.0 * math.asin(delta / (2.0 * center))
    return {
        "motor_pitch_diameter_mm": motor_d,
        "flyer_pitch_diameter_mm": flyer_d,
        "center_distance_mm": center,
        "center_shift_from_selected_60mm_mm": center - BASE_CENTER_DISTANCE_MM,
        "small_pulley_wrap_deg": math.degrees(small_wrap_rad),
        "small_pulley_engaged_teeth": (
            motor_teeth * small_wrap_rad / (2.0 * math.pi)
        ),
    }


def _reduced_pulley_lane(
    *,
    teeth: int,
    stock_inertia_kgm2: float,
    mass_g: float,
    conservative_curve_torque_nm: float,
    curve_reading_speed_rpm: float,
    dimensions_mm: dict[str, float],
    baseline_components: dict[str, float],
) -> dict[str, Any]:
    ratio = FLYER_TEETH / teeth
    bnw_and_clamp_witness_extra = (
        baseline_components[
            "motor_P30_stock_split_clamp_BNW_and_three_screw_upper_bound"
        ]
        - selected.MOTOR_P30_PUBLISHED_INERTIA_KGM2
    )
    reflected_motor_side = (
        selected.LEADSHINE_ROTOR_INERTIA_KGM2
        + stock_inertia_kgm2
        + bnw_and_clamp_witness_extra
    ) * ratio**2
    full_inertia = sum(
        (
            baseline_components[
                "retained_flyer_without_legacy_pulley_hardware"
            ],
            baseline_components["new_flyer_P30_and_two_screws"],
            baseline_components["210_3GT_6_belt"],
            baseline_components["two_complete_6001_bearings_upper_bound"],
            reflected_motor_side,
        )
    )
    required = (
        selected.WIRE_TORQUE_NM
        + selected.FRICTION_ALLOWANCE_NM
        + selected.ANGULAR_ACCELERATION_RAD_S2 * full_inertia
    )
    available = (
        conservative_curve_torque_nm
        * (1.0 - CURVE_TOTAL_HAIRCUT)
        * ratio
    )
    product_record = {
        "model": f"P{teeth}-3GT-BLP-6C-5",
        "stock_D5_status": "official_page_listed_in_stock",
        "mass_g": mass_g,
        "published_inertia_kgm2": stock_inertia_kgm2,
        "dimensions_mm": dimensions_mm,
        "source_url": PRODUCT_URLS[f"NBK_P{teeth}"],
    }
    return {
        "candidate": f"NBK P{teeth} motor / P30 flyer / 210-3GT-6",
        "review_record_sha256": _canonical_sha256(product_record),
        "product_record": product_record,
        "ratio_motor_speed_over_flyer_speed": ratio,
        "motor_rpm_at_300_flyer_rpm": 300.0 * ratio,
        "curve_reading": {
            "lower_edge_nm": conservative_curve_torque_nm,
            "reading_speed_rpm": curve_reading_speed_rpm,
            "reading_is_at_or_faster_than_candidate_speed": (
                curve_reading_speed_rpm >= 300.0 * ratio
            ),
            "additional_total_haircut_fraction": CURVE_TOTAL_HAIRCUT,
        },
        "full_output_inertia_kgm2": full_inertia,
        "required_output_torque_nm": required,
        "available_output_torque_after_haircut_nm": available,
        "available_to_required_multiple": available / required,
        "reserve_above_2x_nm": available - 2.0 * required,
        "torque_screen_ge_2x": available >= 2.0 * required,
        "belt_geometry": _open_belt_geometry(teeth),
        "fixed_upstream_ratio": 1.0,
        "candidate_physical_ratio": ratio,
        "upstream_absolute_radians_and_readback_preserved": False,
        "electronic_gearing_or_firmware_change_verified": False,
        "controller_contract_gate": False,
        "selected_CAD_packaging_gate": False,
        "eligible_for_normal_GOAL_selection": False,
        "decision": (
            "REJECT: non-1:1 physical gearing violates the frozen absolute-"
            "radians command/readback contract. A firmware or verified closed-"
            "loop electronic-gearing layer would be a new dependency; neither "
            "is authorized or proven. No selected CAD change is authorized."
        ),
    }


def analyze() -> dict[str, Any]:
    baseline = selected.analyze()
    duty = baseline["OD65_10N_full_inertia_torque"]
    components = duty["components_kgm2"]

    product_paths = (M22313_ZIP, M22313_STEP, M22313_DRAWING, M22313_CURVE)
    observed_product_hashes = {_relative(path): _sha256(path) for path in product_paths}
    if observed_product_hashes != EXPECTED_PRODUCT_HASHES:
        raise RuntimeError("CS-M22313 review artifacts do not match pinned hashes")

    source_paths = (
        GOAL,
        REQUIREMENTS,
        UPSTREAM_CONFIG,
        Path(selected.__file__).resolve(),
        SELECTED_REPORT,
        selected.LEADSHINE_DRIVER_MANUAL,
        selected.LEADSHINE_CURVE,
    )
    observed_source_hashes = {_relative(path): _sha256(path) for path in source_paths}

    curve_equivalent_peak_a = selected.LEADSHINE_CURVE_RMS_A * math.sqrt(2.0)
    pr2_35_peak_a = 3.5
    pr2_36_peak_a = 3.6
    pr2_35_available = (
        selected.LEADSHINE_36V_300RPM_LOWER_EDGE_NM
        * pr2_35_peak_a
        / curve_equivalent_peak_a
    )
    pr2_35_margin = pr2_35_available / duty["required_output_torque_nm"]
    max_alpha_pr2_35 = (
        pr2_35_available / 2.0
        - selected.WIRE_TORQUE_NM
        - selected.FRICTION_ALLOWANCE_NM
    ) / duty["full_output_inertia_kgm2"]
    pr2_35_margin_at_190 = pr2_35_available / (
        selected.WIRE_TORQUE_NM
        + selected.FRICTION_ALLOWANCE_NM
        + 190.0 * duty["full_output_inertia_kgm2"]
    )

    p28 = _reduced_pulley_lane(
        teeth=28,
        stock_inertia_kgm2=2.1e-6,
        mass_g=24.0,
        conservative_curve_torque_nm=0.722,
        curve_reading_speed_rpm=330.0,
        dimensions_mm={
            "pitch_diameter_Dp": 26.7,
            "tooth_OD_De": 26.0,
            "hub_OD_Db": 14.0,
            "clamp_envelope_E": 21.0,
            "flange_OD_Df": 30.0,
            "overall_width_W": 11.0,
            "belt_channel_width_A": 7.3,
            "flange_thickness_I": 1.85,
            "hub_length_L": 7.0,
            "clamp_bolt_axial_F": 2.75,
            "clamp_bolt_radial_G": 5.0,
            "stock_bore": 5.0,
        },
        baseline_components=components,
    )
    p26 = _reduced_pulley_lane(
        teeth=26,
        stock_inertia_kgm2=1.6e-6,
        mass_g=21.0,
        conservative_curve_torque_nm=0.710,
        curve_reading_speed_rpm=375.0,
        dimensions_mm={
            "pitch_diameter_Dp": 24.8,
            "tooth_OD_De": 24.1,
            "hub_OD_Db": 14.0,
            "clamp_envelope_E": 19.0,
            "flange_OD_Df": 28.0,
            "overall_width_W": 11.0,
            "belt_channel_width_A": 7.3,
            "flange_thickness_I": 1.85,
            "hub_length_L": 7.0,
            "clamp_bolt_axial_F": 2.75,
            "clamp_bolt_radial_G": 5.0,
            "stock_bore": 5.0,
        },
        baseline_components=components,
    )

    m22313_product_record = {
        "model": "CS-M22313",
        "frame": "NEMA23",
        "frame_width_mm": 57.15,
        "body_length_mm": 75.0,
        "phase_current_A_RMS": 4.0,
        "holding_torque_nm": 1.3,
        "shaft_diameter_mm": 8.0,
        "shaft_tolerance_mm": [0.0, -0.013],
        "rotor_inertia_kgm2": 3.0e-5,
        "mass_kg": 0.9,
        "encoder_lines": 1000,
        "source_urls": {
            key: value
            for key, value in PRODUCT_URLS.items()
            if key.startswith("Leadshine_CS_M22313")
        },
        "local_artifact_sha256": observed_product_hashes,
    }
    m22313_inertia = (
        duty["full_output_inertia_kgm2"]
        - selected.LEADSHINE_ROTOR_INERTIA_KGM2
        + m22313_product_record["rotor_inertia_kgm2"]
    )
    m22313_required = (
        selected.WIRE_TORQUE_NM
        + selected.FRICTION_ALLOWANCE_NM
        + selected.ANGULAR_ACCELERATION_RAD_S2 * m22313_inertia
    )
    m22313_conservative_available = 1.20 * (1.0 - CURVE_TOTAL_HAIRCUT)
    m22313 = {
        "candidate": "Leadshine CS-M22313 exact 30T:30T 1:1",
        "review_record_sha256": _canonical_sha256(m22313_product_record),
        "product_record": m22313_product_record,
        "driver_current": {
            "curve_condition_A_RMS": 4.0,
            "curve_equivalent_A_peak": 4.0 * math.sqrt(2.0),
            "screened_CS_D508_PR2": 56,
            "screened_peak_current_A": 5.6,
            "setting_exactly_reproduces_curve_condition": False,
        },
        "torque_screen": {
            "official_curve_300rpm_center_approx_nm": 1.27,
            "review_floor_before_haircut_nm": 1.20,
            "additional_total_haircut_fraction": CURVE_TOTAL_HAIRCUT,
            "available_after_haircut_nm": m22313_conservative_available,
            "full_output_inertia_kgm2": m22313_inertia,
            "required_output_torque_nm": m22313_required,
            "available_to_required_multiple": (
                m22313_conservative_available / m22313_required
            ),
            "reserve_above_2x_nm": (
                m22313_conservative_available - 2.0 * m22313_required
            ),
            "math_screen_ge_2x": (
                m22313_conservative_available >= 2.0 * m22313_required
            ),
            "production_torque_released": False,
        },
        "official_STEP_inspection": {
            "solids": 32,
            "faces": 3189,
            "edges": 8383,
            "bounds_min_mm": [-32.0, -56.4, -12.1],
            "bounds_max_mm": [63.3, 0.0, 56.4],
            "size_mm": [95.3, 56.4, 68.5],
            "contains_cable_and_connector_geometry": True,
        },
        "current_machine_placement_screen": {
            "motor_axis_y_mm": -60.0,
            "motor_mounting_face_z_mm": -112.0,
            "simplified_body_bounds_mm": {
                "x": [-28.575, 28.575],
                "y": [-88.575, -31.425],
                "z": [-187.0, -112.0],
            },
            "simplified_overall_z_with_pilot_and_shaft_mm": [-187.0, -91.0],
            "wire_clearance_mm": 10.175,
            "felt_tensioner_clearance_mm": 1.425,
            "belt_clearance_mm": 8.8039,
            "existing_NEMA17_mount_overlap_mm3": 1459.4235,
            "minimum_required_clearance_mm": MIN_CLEARANCE_MM,
            "felt_clearance_gate": False,
            "existing_mount_collision_gate": False,
        },
        "mount_interface": {
            "required_bolt_square_mm": 47.14,
            "required_pilot_diameter_mm": 38.1,
            "drawing_mount_holes": "4 x diameter 5 mm",
            "selected_GOAL_interface": "NEMA17 42.3 mm frame / 31 mm bolt square",
            "new_mount_required": True,
        },
        "exact_1_to_1_contract_preserved": True,
        "normal_GOAL_requires_NEMA17": True,
        "candidate_is_NEMA17": False,
        "normal_GOAL_frame_gate": False,
        "selected_CAD_packaging_gate": False,
        "eligible_for_normal_GOAL_selection": False,
        "decision": (
            "REJECT: the torque screen is strong, but CS-M22313 is NEMA23 "
            "where normal GOAL freezes standard NEMA17. At the current machine "
            "placement it also leaves only 1.425 mm to the felt tensioner "
            "against the 2.0 mm gate and overlaps the NEMA17 mount, requiring a "
            "new mount and packaging redesign."
        ),
    }

    current_lane = {
        "candidate": "CS-M21708 / CS-D508 / exact 30T:30T 1:1",
        "curve_condition": {
            "RMS_A": selected.LEADSHINE_CURVE_RMS_A,
            "sinusoidal_equivalent_peak_A": curve_equivalent_peak_a,
            "lower_edge_torque_at_300rpm_nm": (
                selected.LEADSHINE_36V_300RPM_LOWER_EDGE_NM
            ),
        },
        "CS_D508_PR2": {
            "unit": "0.1 A peak",
            "exact_curve_equivalent_is_programmable": False,
            "PR2_35": {
                "peak_A": pr2_35_peak_a,
                "equivalent_RMS_A": pr2_35_peak_a / math.sqrt(2.0),
                "linear_current_scaled_torque_nm": pr2_35_available,
                "linear_scaling_is_manufacturer_verified": False,
                "margin_at_200rad_s2_multiple": pr2_35_margin,
                "gate_at_200rad_s2_ge_2x": pr2_35_margin >= 2.0,
                "maximum_alpha_for_exact_2x_rad_s2": max_alpha_pr2_35,
                "margin_at_190rad_s2_multiple": pr2_35_margin_at_190,
                "acceleration_limit_bound_in_upstream_configuration": False,
            },
            "PR2_36": {
                "peak_A": pr2_36_peak_a,
                "equivalent_RMS_A": pr2_36_peak_a / math.sqrt(2.0),
                "percent_above_curve_RMS_current": (
                    (
                        pr2_36_peak_a
                        / math.sqrt(2.0)
                        / selected.LEADSHINE_CURVE_RMS_A
                        - 1.0
                    )
                    * 100.0
                ),
                "manufacturer_authorized_for_CS_M21708": False,
            },
        },
        "eligible_as_exact_production_configuration": False,
        "decision": (
            "NO EXACT RELEASED SETTING: PR2=35 misses 2x at the frozen 200 "
            "rad/s^2 duty even under the unverified favorable assumption that "
            "torque scales linearly with current. PR2=36 exceeds the RMS 2.5 A "
            "curve condition. Manufacturer confirmation or the hot 36 V / 300 "
            "RPM dyno remains mandatory."
        ),
    }

    report = {
        "schema": "m2-exact-configuration-trade-study/v1",
        "analysis_source": "sim/m2_exact_configuration_trade_study.py",
        "analysis_source_sha256": _sha256(Path(__file__).resolve()),
        "status": "REVIEW_ONLY__NO_ELIGIBLE_EXACT_PRODUCTION_CONFIGURATION",
        "review_only": True,
        "normal_GOAL_modified": False,
        "selected_CAD_modified": False,
        "selected_BOM_modified": False,
        "selected_load_authority_modified": False,
        "controller_or_firmware_modified": False,
        "integration_authorized": False,
        "procurement_authorized": False,
        "production_authorized": False,
        "frozen_requirements": {
            "motor_frame": "standard NEMA17 closed-loop stepper",
            "motor_frame_source": "GOAL.md:24",
            "flyer_speed_rpm": 300.0,
            "wire_tension_N": 10.0,
            "torque_margin_multiple": 2.0,
            "required_running_torque_nm": duty["required_output_torque_nm"],
            "required_2x_threshold_nm": duty["required_2x_running_torque_nm"],
            "angular_acceleration_rad_s2": selected.ANGULAR_ACCELERATION_RAD_S2,
            "minimum_running_clearance_mm": MIN_CLEARANCE_MM,
            "upstream_m2_gear_ratio": 1.0,
            "upstream_contract_source": "winder-goal1-contract/src/config.py:4",
            "derived_requirement": (
                "machine/docs/requirements.md:155-157 fixes a 1:1 belt so "
                "absolute-radians commands and readback remain unchanged"
            ),
        },
        "source_evidence": {
            "official_product_urls": PRODUCT_URLS,
            "observed_source_sha256": observed_source_hashes,
            "pinned_product_artifact_sha256": observed_product_hashes,
            "canonical_review_input_sha256": "",
        },
        "selected_authority_snapshot": {
            "schema": baseline["schema"],
            "status": baseline["status"],
            "motor": baseline["selection"]["motor"],
            "ratio": baseline["selection"]["ratio"],
            "production_authorized": baseline["production_authorized"],
            "note": (
                "Referenced for load inputs only; this trade study does not "
                "supersede or mutate the selection."
            ),
        },
        "alternatives": {
            "exact_CS_D508_current": current_lane,
            "NBK_P28_reduction": p28,
            "NBK_P26_reduction": p26,
            "Leadshine_CS_M22313_NEMA23": m22313,
        },
        "decision": {
            "eligible_alternative": None,
            "selected_configuration_changed": False,
            "conclusion": (
                "No studied lane is an exact production-configurable solution "
                "inside the unchanged normal GOAL and upstream contract. Keep "
                "the conditional exact-1:1 NEMA17 selection and close its "
                "existing manufacturer-current or hot-dyno gate; changing "
                "ratio, frame, packaging, CAD, BOM, or load authority is not "
                "authorized by this review."
            ),
        },
    }
    canonical_inputs = {
        "frozen_requirements": report["frozen_requirements"],
        "official_product_urls": PRODUCT_URLS,
        "source_sha256": observed_source_hashes,
        "product_artifact_sha256": observed_product_hashes,
        "candidate_review_record_sha256": {
            "P28": p28["review_record_sha256"],
            "P26": p26["review_record_sha256"],
            "CS_M22313": m22313["review_record_sha256"],
        },
    }
    report["source_evidence"]["canonical_review_input_sha256"] = (
        _canonical_sha256(canonical_inputs)
    )
    return report


def _markdown(report: dict[str, Any]) -> str:
    current = report["alternatives"]["exact_CS_D508_current"]
    p28 = report["alternatives"]["NBK_P28_reduction"]
    p26 = report["alternatives"]["NBK_P26_reduction"]
    m23 = report["alternatives"]["Leadshine_CS_M22313_NEMA23"]
    pr35 = current["CS_D508_PR2"]["PR2_35"]
    m23_torque = m23["torque_screen"]
    m23_fit = m23["current_machine_placement_screen"]
    return f"""# M2 exact-configuration trade study

Status: **review only; no eligible exact production configuration.** This report does not change the selected CAD, BOM, load authority, controller contract, procurement state, or normal-GOAL selection.

| Lane | Quantified screen | Frozen-gate result | Decision |
|---|---:|---|---|
| CS-M21708, CS-D508 PR2=35, exact 1:1 | `{pr35['margin_at_200rad_s2_multiple']:.6f}x`; exact-2x alpha limit `{pr35['maximum_alpha_for_exact_2x_rad_s2']:.3f} rad/s^2` | 200 rad/s^2 fails; current-to-torque scaling is not manufacturer-verified | Reject as an exact production setting |
| NBK P28 motor / P30 flyer | `{p28['available_to_required_multiple']:.6f}x`; center `{p28['belt_geometry']['center_distance_mm']:.3f} mm` | Fails torque and changes physical ratio to `{p28['candidate_physical_ratio']:.9f}:1` | Reject; absolute-radians contract is fixed 1:1 |
| NBK P26 motor / P30 flyer | `{p26['available_to_required_multiple']:.6f}x`; center `{p26['belt_geometry']['center_distance_mm']:.3f} mm` | Torque screen passes narrowly, but physical ratio becomes `{p26['candidate_physical_ratio']:.9f}:1` | Reject; firmware/electronic gearing is unverified and unauthorized |
| CS-M22313 NEMA23, exact 1:1 | `{m23_torque['available_to_required_multiple']:.6f}x` after 10% haircut | GOAL requires NEMA17; felt clearance `{m23_fit['felt_tensioner_clearance_mm']:.3f} mm < {m23_fit['minimum_required_clearance_mm']:.1f} mm`; existing mount overlap `{m23_fit['existing_NEMA17_mount_overlap_mm3']:.4f} mm^3` | Reject; new mount and packaging redesign required |

The CS-D508 exposes peak current as PR2 in 0.1 A increments. The CS-M21708 curve condition, RMS 2.5 A, corresponds to `{current['curve_condition']['sinusoidal_equivalent_peak_A']:.6f} A` peak. PR2=35 is below it and only reaches `{pr35['margin_at_190rad_s2_multiple']:.6f}x` even if acceleration is reduced to 190 rad/s^2; no upstream acceleration cap is presently bound. PR2=36 is `{current['CS_D508_PR2']['PR2_36']['percent_above_curve_RMS_current']:.3f}%` above the plotted RMS condition and is not manufacturer-authorized here.

P28 and P26 use exact open-belt pitch geometry for the retained 210-3GT-6 belt. Their geometry can be studied mechanically, but both violate the frozen 1:1 command/readback semantics. The P26 torque result is therefore not a selection.

The official CS-M22313 STEP, drawing, and curve are hash-pinned in the JSON report. Its torque headroom is attractive, but it is a 57.15 mm NEMA23 with a 47.14 mm bolt square and 38.1 mm pilot. The normal GOAL explicitly specifies standard NEMA17 and the current placement also fails the 2 mm felt-clearance gate.

Conclusion: **keep the conditional exact-1:1 NEMA17 selection unchanged.** Production release still needs Leadshine confirmation of an exact current configuration or a hot 36 V / 300 RPM dyno pass at the existing 2x threshold. This review authorizes no CAD/BOM/load change.
"""


def write_outputs() -> dict[str, Any]:
    report = analyze()
    OUTPUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    OUTPUT_MD.write_text(_markdown(report), encoding="utf-8")
    return report


if __name__ == "__main__":
    result = write_outputs()
    print(result["status"])
    print(OUTPUT_JSON)
    print(OUTPUT_MD)
