"""Fail-closed normal-GOAL M2 motor and exact-1:1 drive selection.

This analysis supersedes the 17HS24 candidate as the recommended M2 motor,
but it does not authorize production.  It binds the manufacturer torque curve,
rotor inertia, exact retained flyer inertia, conservative whole-bearing and
drive-component inertia bounds, the exact 10 N / OD65 load, and the upstream
1:1 motion contract.  Pulley-to-shaft retention and installed friction remain
physical/supplier release gates.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
REPORTS = ROOT / "out" / "reports"
RETAINED_REPORT = REPORTS / "permanent_cap_offset_spoke_retained_review.json"
OUTPUT_JSON = REPORTS / "m2_normal_goal_drive_selection.json"
OUTPUT_MD = REPORTS / "m2_normal_goal_drive_selection.md"

CAD_UPGRADES = ROOT / "cad" / "models" / "upgrades"
LEADSHINE_STEP = CAD_UPGRADES / "CS-M21708.STEP"
LEADSHINE_2D = CAD_UPGRADES / "Leadshine_CS-M21708_2D.pdf"
LEADSHINE_CURVE = CAD_UPGRADES / "Leadshine_CS-M21708_torque_curve.png"
LEADSHINE_CABLELESS = CAD_UPGRADES / "CS-M21708_cableless.step"
LEADSHINE_DRIVER_MANUAL = (
    CAD_UPGRADES / "Leadshine_CS-D508_manual_v1.0.pdf"
)
NBK_SLIP_CHART = CAD_UPGRADES / "NBK_aluminum_set_screw_slip_torque_reference.jpg"

LEADSHINE_PRODUCT_URL = (
    "https://www.leadshine.com/product-detail/closed-loop-stepper-drive/"
    "closed-loop-stepper/CS-M21708.html"
)
LEADSHINE_3D_URL = (
    "https://www.leadshine.com/upfiles/downloads/"
    "3c4df9dc7e3237fbdafee94f4142513a_1651890419697.zip"
)
LEADSHINE_2D_URL = (
    "https://www.leadshine.com/upfiles/downloads/"
    "fd433814f61a6818f9141c1952fc33eb_1651890544432.pdf"
)
LEADSHINE_DRIVER_URL = (
    "https://www.leadshine.com/product-detail/CS-D508.html"
)
LEADSHINE_DRIVER_MANUAL_URL = (
    "https://www.leadshine.com/upfiles/downloads/"
    "6a05fc89c7f263374eafe258220c3463_1665569910600.pdf"
)
LEADSHINE_DRIVER_SOFTWARE_MANUAL_URL = (
    "https://www.leadshine.com/upfiles/downloads/"
    "14419125f0bab74945cb1577be162c56_1665570154673.pdf"
)
LEADSHINE_CURRENT_STEPPER_CATALOG_URL = (
    "https://www.leadshine.com/upfiles/downloads/"
    "9d1b16bdd74302c1d533074657f1b972_1743650341386.pdf"
)
LEADSHINE_36V_PSU_URL = (
    "https://www.leadshine.com/product-detail/LSP-360-36.html"
)
NBK_P30_URL = (
    "https://www.nbk1560.com/products/pulley/timingpulley/"
    "3GT-BLP-6C/P30-3GT-BLP-6C/"
)
NBK_D_SHAFT_GUIDANCE_URL = (
    "https://www.nbk1560.com/images/en-US/product/contents/"
    "toritsuke_coupling_NBK/toritsuke_coupling_NBK_1.pdf"
)
NBK_SLIP_TORQUE_URL = (
    "https://www.nbk1560.com/en-US/resources/coupling/article/"
    "couplicon-set-screw-type-slip-torque/?SelectedLanguage=en-US"
)
NBK_CAD_GUIDE_URL = "https://www.nbk1560.com/en-US/guide/cad/"
NBK_CAD_REQUEST_URL = (
    "https://www.nbk1560.com/en/contact/cadrequest/?SelectedLanguage=en"
)
NBK_BELT_URL = "https://www.nbk1560.com/products/pulley/timingpulley_option/3GT-6/"
SKF_6001_URL = "https://www.emarketplace.in.skf.com/deep-groove-ball-bearing/6001-2z"

EXPECTED_SOURCE_HASHES = {
    "cad/models/upgrades/CS-M21708.STEP": (
        "7e995e724fc7e019278e0a919ba1db8c8abb3333f156c64eb6e62485e0f6662b"
    ),
    "cad/models/upgrades/Leadshine_CS-M21708_2D.pdf": (
        "b0edb4d9486562f2ead76c1363ac78ee4c20d3abaa3ad5a93eab60eb83a81141"
    ),
    "cad/models/upgrades/Leadshine_CS-M21708_torque_curve.png": (
        "00b7382799799a7a233abac07e3fd2a3dc77dca3a19c0b29dda72f9fa3c938cf"
    ),
    "cad/models/upgrades/Leadshine_CS-D508_manual_v1.0.pdf": (
        "0faaf40eebe24203511b50b3e3658bec9fc298b13221031759b84d3eb9bdba60"
    ),
    "cad/models/upgrades/NBK_aluminum_set_screw_slip_torque_reference.jpg": (
        "70cae21dcd074ed35dc534fffafc9787f3a02f2c2236f408375447568250222e"
    ),
}

# Frozen normal-GOAL duty.
FLYER_RPM = 300.0
WIRE_TENSION_N = 10.0
LINE_OF_ACTION_MM = 32.5
WIRE_TORQUE_NM = WIRE_TENSION_N * LINE_OF_ACTION_MM / 1000.0
FRICTION_ALLOWANCE_NM = 0.020
ANGULAR_ACCELERATION_RAD_S2 = 200.0
REQUIRED_TORQUE_MULTIPLE = 2.0

# Leadshine product and plot data.  The 0.735 value is one pixel below the
# lower red-curve edge at 300 RPM, not the curve center (~0.741 N m).
LEADSHINE_MODEL = "CS-M21708"
LEADSHINE_ROTOR_INERTIA_KGM2 = 0.11e-4  # 0.11 kg cm^2
LEADSHINE_36V_300RPM_LOWER_EDGE_NM = 0.735
LEADSHINE_24V_300RPM_LOWER_EDGE_NM = 0.722
LEADSHINE_CURVE_RMS_A = 2.5
LEADSHINE_CURVE_EQUIVALENT_PEAK_A = LEADSHINE_CURVE_RMS_A * math.sqrt(2.0)
LEADSHINE_DRIVER_CURRENT_INCREMENT_A_PEAK = 0.1
LEADSHINE_DRIVER_NEAREST_PEAK_SETTINGS_A = (3.5, 3.6)
LEADSHINE_SUPPLY_MODEL = "LSP-360-36"
LEADSHINE_SUPPLY_OUTPUT_VDC = 36.0
LEADSHINE_SUPPLY_CONTINUOUS_A = 10.0
LEADSHINE_SUPPLY_PEAK_A = 18.0
LEADSHINE_SUPPLY_POWER_W = 360.0
LEADSHINE_SUPPLY_INPUT_WINDOWS_VAC = ((92.0, 138.0), (184.0, 276.0))
LEADSHINE_SUPPLY_SIZE_MM = (215.0, 115.0, 30.0)
LEADSHINE_SUPPLY_OFFICIAL_PRICE_USD = 199.0
LEADSHINE_DRIVER_CONDITION = (
    "36 VDC, RMS 2.5 A motor-curve condition; Leadshine CS-D508 is the "
    "selected officially tested-compatible 20-50 VDC / 8.0 A-peak driver, "
    "but its 0.1 A peak increments cannot encode the exact 3.5355 A curve "
    "equivalent, so 3.5 A versus 3.6 A remains a production proof gate"
)

NBK_P30_D5_STOCK_STEP = (
    CAD_UPGRADES / "NBK_P30-3GT-BLP-6C-5_AP214.step"
)
NBK_P30_D5_STOCK_STEP_SHA256 = (
    "996449b7d9ec7703e7b38c6f75eff00a1174e3e1f088c05f0f1460b205169df9"
)

# Exact/reference drive inertia at 1:1.
FLYER_P30_AND_TWO_SCREWS_INERTIA_KGM2 = 2.61871821045676e-6
MOTOR_P30_PUBLISHED_INERTIA_KGM2 = 3.0e-6
MOTOR_PULLEY_M2_WITNESS_INERTIA_KGM2 = 6.8252109555268e-9
# The BNW product-specific set-screw size/length is not exposed before the
# generated drawing/quote.  Bound its acceleration load with two much larger
# M3x12 steel radial screws, spanning radius 2.5..14.5 mm (including 3 mm proud
# of the published E23 clamp envelope).  This is an inertia bound only; it does
# not assert that M3x12 is the delivered BNW hardware or prove slip torque.
BNW_SET_SCREW_COUNT = 2
BNW_INERTIA_BOUND_DIAMETER_M = 0.003
BNW_INERTIA_BOUND_LENGTH_M = 0.012
BNW_INERTIA_BOUND_STEEL_DENSITY_KG_M3 = 7850.0
BNW_INERTIA_BOUND_R0_M = 0.0025
BNW_INERTIA_BOUND_R1_M = 0.0145
ONE_BNW_INERTIA_BOUND_SCREW_MASS_KG = (
    BNW_INERTIA_BOUND_STEEL_DENSITY_KG_M3
    * math.pi
    * (BNW_INERTIA_BOUND_DIAMETER_M / 2.0) ** 2
    * BNW_INERTIA_BOUND_LENGTH_M
)
ONE_BNW_INERTIA_BOUND_SCREW_KGM2 = ONE_BNW_INERTIA_BOUND_SCREW_MASS_KG * (
    (
        BNW_INERTIA_BOUND_R0_M**2
        + BNW_INERTIA_BOUND_R0_M * BNW_INERTIA_BOUND_R1_M
        + BNW_INERTIA_BOUND_R1_M**2
    )
    / 3.0
    + (BNW_INERTIA_BOUND_DIAMETER_M / 2.0) ** 2 / 4.0
)
MOTOR_P30_SPLIT_CLAMP_BNW_INERTIA_UPPER_KGM2 = (
    MOTOR_P30_PUBLISHED_INERTIA_KGM2
    + MOTOR_PULLEY_M2_WITNESS_INERTIA_KGM2
    + BNW_SET_SCREW_COUNT * ONE_BNW_INERTIA_BOUND_SCREW_KGM2
)
BELT_MASS_KG = 0.0032
PITCH_MM = 3.0
PULLEY_TEETH = 30
PITCH_RADIUS_M = (PULLEY_TEETH * PITCH_MM / math.pi) / 2.0 / 1000.0
BELT_INERTIA_KGM2 = BELT_MASS_KG * PITCH_RADIUS_M**2

# Two 6001 bearings, conservatively treating each complete 22 g bearing as a
# rotating annulus between its 12 mm bore and 28 mm OD.
BEARING_COUNT = 2
BEARING_MASS_KG_UPPER = 0.022
BEARING_INNER_RADIUS_M = 0.006
BEARING_OUTER_RADIUS_M = 0.014
ONE_WHOLE_BEARING_INERTIA_KGM2 = 0.5 * BEARING_MASS_KG_UPPER * (
    BEARING_INNER_RADIUS_M**2 + BEARING_OUTER_RADIUS_M**2
)
BEARING_INERTIA_UPPER_KGM2 = BEARING_COUNT * ONE_WHOLE_BEARING_INERTIA_KGM2

# Official 36 V red-curve conservative readings for the requested exploratory
# ratio sweep.  These are lower edges (plus one pixel), not centerline values.
RATIO_CURVE_LOWER_EDGE_NM = {
    1.00: 0.735,
    1.10: 0.722,
    1.25: 0.710,
    1.50: 0.673,
    2.00: 0.592,
    2.50: 0.539,
    3.00: 0.486,
}

LEGACY_ROTATING_ROWS = (
    "shifted_flyer_pulley_exact_1_to_1",
    "flyer_pulley_radial_M3x8_set_screw",
    "flyer_pulley_radial_M3_short_insert",
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _retained_inertia() -> tuple[float, float, dict[str, float]]:
    report = json.loads(RETAINED_REPORT.read_text(encoding="utf-8"))
    total = report["exact_rotating_mass_properties"]["izz_about_M2_axis_kg_m2"]
    rows = {
        row["name"]: row["izz_about_M2_axis_g_mm2"] * 1.0e-9
        for row in report["exact_rotating_mass_rows"]
    }
    missing = set(LEGACY_ROTATING_ROWS) - set(rows)
    if missing:
        raise RuntimeError(f"retained report missing legacy drive rows: {sorted(missing)}")
    removed = {name: rows[name] for name in LEGACY_ROTATING_ROWS}
    base_without_legacy_drive = total - sum(removed.values())
    return total, base_without_legacy_drive, removed


def _ratio_trade(base_without_legacy_drive: float) -> list[dict]:
    rows = []
    motor_pitch_diameter_mm = PULLEY_TEETH * PITCH_MM / math.pi
    for ratio, motor_torque_nm in RATIO_CURVE_LOWER_EDGE_NM.items():
        flyer_pitch_diameter_mm = motor_pitch_diameter_mm * ratio
        belt_pitch_length_mm = (
            2.0 * 60.0
            + math.pi / 2.0 * (motor_pitch_diameter_mm + flyer_pitch_diameter_mm)
            + (flyer_pitch_diameter_mm - motor_pitch_diameter_mm) ** 2
            / (4.0 * 60.0)
        )
        belt_mass_kg = BELT_MASS_KG * belt_pitch_length_mm / 210.0
        belt_output_inertia = belt_mass_kg * (
            flyer_pitch_diameter_mm / 2.0 / 1000.0
        ) ** 2
        # Conservative similar-radial-geometry scaling: mass ~r^2 and J~r^4.
        flyer_pulley_output_inertia = (
            FLYER_P30_AND_TWO_SCREWS_INERTIA_KGM2 * ratio**4
        )
        motor_side_reflected = (
            LEADSHINE_ROTOR_INERTIA_KGM2
            + MOTOR_P30_SPLIT_CLAMP_BNW_INERTIA_UPPER_KGM2
        ) * ratio**2
        full_output_inertia = (
            base_without_legacy_drive
            + BEARING_INERTIA_UPPER_KGM2
            + flyer_pulley_output_inertia
            + belt_output_inertia
            + motor_side_reflected
        )
        required_output_torque = (
            WIRE_TORQUE_NM
            + FRICTION_ALLOWANCE_NM
            + ANGULAR_ACCELERATION_RAD_S2 * full_output_inertia
        )
        available_output_torque = ratio * motor_torque_nm
        rows.append(
            {
                "ratio_motor_speed_over_flyer_speed": ratio,
                "motor_rpm_at_300_flyer_rpm": ratio * FLYER_RPM,
                "digitized_36V_motor_torque_lower_edge_nm": motor_torque_nm,
                "ideal_available_output_torque_nm": available_output_torque,
                "approximate_belt_pitch_length_mm": belt_pitch_length_mm,
                "full_output_inertia_kgm2": full_output_inertia,
                "required_output_torque_nm": required_output_torque,
                "available_to_required_multiple": (
                    available_output_torque / required_output_torque
                ),
                "motor_math_ge_2x": (
                    available_output_torque
                    >= REQUIRED_TORQUE_MULTIPLE * required_output_torque
                ),
            }
        )
    return rows


def analyze() -> dict:
    source_hashes = {
        str(path.relative_to(ROOT)).replace("\\", "/"): _sha256(path)
        for path in (
            LEADSHINE_STEP,
            LEADSHINE_2D,
            LEADSHINE_CURVE,
            LEADSHINE_DRIVER_MANUAL,
            NBK_SLIP_CHART,
        )
    }
    if source_hashes != EXPECTED_SOURCE_HASHES:
        raise RuntimeError("Leadshine source artifacts no longer match the reviewed files")

    retained_total, retained_base, removed = _retained_inertia()
    full_inertia = sum(
        (
            retained_base,
            FLYER_P30_AND_TWO_SCREWS_INERTIA_KGM2,
            MOTOR_P30_SPLIT_CLAMP_BNW_INERTIA_UPPER_KGM2,
            BELT_INERTIA_KGM2,
            LEADSHINE_ROTOR_INERTIA_KGM2,
            BEARING_INERTIA_UPPER_KGM2,
        )
    )
    acceleration_torque = ANGULAR_ACCELERATION_RAD_S2 * full_inertia
    required_torque = WIRE_TORQUE_NM + FRICTION_ALLOWANCE_NM + acceleration_torque
    two_x_threshold = REQUIRED_TORQUE_MULTIPLE * required_torque
    motor_multiple = LEADSHINE_36V_300RPM_LOWER_EDGE_NM / required_torque
    motor_gate = LEADSHINE_36V_300RPM_LOWER_EDGE_NM >= two_x_threshold
    at_24v_multiple = LEADSHINE_24V_300RPM_LOWER_EDGE_NM / required_torque

    pulley_allowable_nm = 2.06 * 0.9 * 1.0
    pulley_capacity_gate = pulley_allowable_nm >= two_x_threshold

    dual_inertia = (
        retained_base
        + BEARING_INERTIA_UPPER_KGM2
        + 2.0
        * (
            FLYER_P30_AND_TWO_SCREWS_INERTIA_KGM2
            + BELT_INERTIA_KGM2
            + MOTOR_P30_SPLIT_CLAMP_BNW_INERTIA_UPPER_KGM2
            + 9.0e-6
        )
    )
    dual_required = (
        WIRE_TORQUE_NM
        + FRICTION_ALLOWANCE_NM
        + ANGULAR_ACCELERATION_RAD_S2 * dual_inertia
    )
    dual_available = 2.0 * 0.430

    report = {
        "schema": "m2-normal-goal-drive-selection/v1",
        "analysis_source": "sim/m2_normal_goal_drive_selection.py",
        "analysis_source_sha256": _sha256(Path(__file__).resolve()),
        "status": (
            "CONDITIONAL_SELECTION__LEADSHINE_36V_EXACT_1_TO_1_MOTOR_AND_BELT_"
            "TORQUE_PASS__DRIVER_CONFIG_RETENTION_FRICTION_AND_INTEGRATED_"
            "PACKAGING_OPEN"
        ),
        "production_authorized": False,
        "procurement_authorized": False,
        "reference_CAD_integration_authorized": True,
        "normal_GOAL_modified": False,
        "selection": {
            "motor": "Leadshine CS-M21708 closed-loop NEMA17",
            "driver_condition": LEADSHINE_DRIVER_CONDITION,
            "driver": (
                "SELECTED HARDWARE / CONDITIONAL CONFIGURATION: Leadshine "
                "CS-D508; the official manual explicitly lists CS-M21708 as "
                "tested compatible, but 3.5 A versus 3.6 A peak and the hot "
                "300 RPM torque proof are not released"
            ),
            "supply_condition": (
                "SELECTED BENCH CANDIDATE: Leadshine LSP-360-36 regulated "
                "supply, 36 VDC / 10 A continuous / 18 A peak / 360 W; the "
                "mains enclosure, branch protection, disconnect, PE bonding, "
                "mounting and wiring remain blocked electronics integration"
            ),
            "ratio": 1.0,
            "motor_pulley": (
                "NBK P30-3GT-BLP-6C-5 stock split clamp plus BNW additional "
                "machining (two included set screws at 90 degrees); orient one "
                "set screw on the motor D-flat and keep the flat clear of the "
                "split and clamp-bolt spot face"
            ),
            "flyer_pulley": (
                "P30 3GT 6 mm exact-profile supplier-machined equivalent, "
                "12.05 mm flyer bore and released reversing retention feature"
            ),
            "belt": "NBK 210-3GT-6",
            "decision": (
                "keep exact 1:1; do not add reduction or a second M2 motor because "
                "the 36 V single-motor curve clears the full-inertia 2x gate"
            ),
        },
        "manufacturer_evidence": {
            "leadshine_product_url": LEADSHINE_PRODUCT_URL,
            "leadshine_3D_archive_url": LEADSHINE_3D_URL,
            "leadshine_2D_url": LEADSHINE_2D_URL,
            "leadshine_CS_D508_product_url": LEADSHINE_DRIVER_URL,
            "leadshine_CS_D508_manual_url": LEADSHINE_DRIVER_MANUAL_URL,
            "leadshine_CS_D508_software_manual_url": (
                LEADSHINE_DRIVER_SOFTWARE_MANUAL_URL
            ),
            "leadshine_current_stepper_catalog_url": (
                LEADSHINE_CURRENT_STEPPER_CATALOG_URL
            ),
            "leadshine_36V_power_supply_url": LEADSHINE_36V_PSU_URL,
            "NBK_P30_url": NBK_P30_URL,
            "NBK_D_shaft_guidance_url": NBK_D_SHAFT_GUIDANCE_URL,
            "NBK_set_screw_slip_torque_url": NBK_SLIP_TORQUE_URL,
            "NBK_CAD_guide_url": NBK_CAD_GUIDE_URL,
            "NBK_CAD_request_url": NBK_CAD_REQUEST_URL,
            "NBK_belt_url": NBK_BELT_URL,
            "SKF_6001_url": SKF_6001_URL,
            "source_hashes": source_hashes,
            "driver_binding": {
                "model": "CS-D508",
                "manual_revision": "1.0 (September 2017)",
                "manual_local_path": (
                    "cad/models/upgrades/Leadshine_CS-D508_manual_v1.0.pdf"
                ),
                "manual_appendix_A_explicitly_tested_motor": "CS-M21708",
                "manual_motor_encoder_requirement_lines": 1000,
                "input_voltage_range_vdc": [20.0, 50.0],
                "maximum_output_current_A_peak": 8.0,
                "selected_supply_vdc": 36.0,
                "curve_condition_A_RMS": LEADSHINE_CURVE_RMS_A,
                "curve_condition_equivalent_A_peak_if_sinusoidal": (
                    LEADSHINE_CURVE_EQUIVALENT_PEAK_A
                ),
                "software_peak_current_parameter": "PR 2",
                "software_peak_current_increment_A": (
                    LEADSHINE_DRIVER_CURRENT_INCREMENT_A_PEAK
                ),
                "nearest_0p1A_peak_settings": list(
                    LEADSHINE_DRIVER_NEAREST_PEAK_SETTINGS_A
                ),
                "lower_setting_equivalent_RMS_A": (
                    LEADSHINE_DRIVER_NEAREST_PEAK_SETTINGS_A[0]
                    / math.sqrt(2.0)
                ),
                "upper_setting_equivalent_RMS_A": (
                    LEADSHINE_DRIVER_NEAREST_PEAK_SETTINGS_A[1]
                    / math.sqrt(2.0)
                ),
                "prior_CS1_D503S_3A_peak_rejected": True,
                "exact_peak_setting_reproducing_curve_confirmed": False,
                "motor_product_page_current_rating_is_self_consistent": False,
                "current_catalog_and_curve_RMS_current_match": True,
                "commissioning_note": (
                    "The current Leadshine catalog and plotted curve both use "
                    "RMS 2.5 A, while the product-page summary/table remain "
                    "internally inconsistent. Do not infer curve equivalence "
                    "from rounding alone: obtain Leadshine confirmation of "
                    "PR 2 or reproduce the required lower edge on a hot 36 V, "
                    "300 RPM dyno before production release."
                ),
            },
            "supply_binding": {
                "model": LEADSHINE_SUPPLY_MODEL,
                "product_page_status": "active_add_to_cart",
                "official_online_price_usd": (
                    LEADSHINE_SUPPLY_OFFICIAL_PRICE_USD
                ),
                "output_voltage_vdc": LEADSHINE_SUPPLY_OUTPUT_VDC,
                "continuous_output_current_A": (
                    LEADSHINE_SUPPLY_CONTINUOUS_A
                ),
                "peak_output_current_A": LEADSHINE_SUPPLY_PEAK_A,
                "rated_power_W": LEADSHINE_SUPPLY_POWER_W,
                "input_voltage_windows_vac": [
                    list(window) for window in LEADSHINE_SUPPLY_INPUT_WINDOWS_VAC
                ],
                "input_selector_required": True,
                "size_mm": list(LEADSHINE_SUPPLY_SIZE_MM),
                "continuous_current_to_curve_equivalent_peak_multiple": (
                    LEADSHINE_SUPPLY_CONTINUOUS_A
                    / LEADSHINE_CURVE_EQUIVALENT_PEAK_A
                ),
                "headroom_below_driver_max_input_v": (
                    50.0 - LEADSHINE_SUPPLY_OUTPUT_VDC
                ),
                "exact_SKU_and_electrical_capacity_pinned": True,
                "mains_safety_integration_pinned": False,
                "candidate_cart_ready": True,
                "production_order_authorized": False,
            },
            "cableless_exact_body": {
                "generator": "cad/leadshine_cs_m21708_cableless.py",
                "step": "cad/models/upgrades/CS-M21708_cableless.step",
                "mount_frame": "mount face z=0, shaft +Z, nominal rear -83.0 mm, exact feature rear -83.2 mm, tip +24.0 mm",
            },
        },
        "motor": {
            "model": LEADSHINE_MODEL,
            "NEMA_frame": 17,
            "closed_loop": True,
            "encoder_lines": 1000,
            "body_length_mm": 83.0,
            "shaft": {
                "diameter_mm": 5.0,
                "exposed_D_flat_across_mm": 4.5,
                "protrusion_mm": 24.0,
                "D_length_mm": 15.0,
                "exact_STEP_D_section_area_mm2": 18.6131,
            },
            "rotor_inertia_kgm2": LEADSHINE_ROTOR_INERTIA_KGM2,
            "curve_condition": "36 VDC, RMS 2.5 A",
            "curve_centerline_300rpm_nm_approx": 0.741,
            "curve_conservative_lower_edge_300rpm_nm": (
                LEADSHINE_36V_300RPM_LOWER_EDGE_NM
            ),
            "24V_lower_edge_300rpm_nm": LEADSHINE_24V_300RPM_LOWER_EDGE_NM,
        },
        "OD65_10N_full_inertia_torque": {
            "wire_torque_nm": WIRE_TORQUE_NM,
            "friction_allowance_nm_unmeasured": FRICTION_ALLOWANCE_NM,
            "angular_acceleration_rad_s2": ANGULAR_ACCELERATION_RAD_S2,
            "retained_report_path": str(RETAINED_REPORT.relative_to(ROOT)).replace("\\", "/"),
            "retained_report_sha256": _sha256(RETAINED_REPORT),
            "retained_exact_inertia_before_legacy_drive_removal_kgm2": retained_total,
            "removed_legacy_drive_inertia_rows_kgm2": removed,
            "components_kgm2": {
                "retained_flyer_without_legacy_pulley_hardware": retained_base,
                "new_flyer_P30_and_two_screws": FLYER_P30_AND_TWO_SCREWS_INERTIA_KGM2,
                "motor_P30_stock_split_clamp_BNW_and_three_screw_upper_bound": (
                    MOTOR_P30_SPLIT_CLAMP_BNW_INERTIA_UPPER_KGM2
                ),
                "210_3GT_6_belt": BELT_INERTIA_KGM2,
                "Leadshine_rotor": LEADSHINE_ROTOR_INERTIA_KGM2,
                "two_complete_6001_bearings_upper_bound": BEARING_INERTIA_UPPER_KGM2,
            },
            "full_output_inertia_kgm2": full_inertia,
            "acceleration_torque_nm": acceleration_torque,
            "required_output_torque_nm": required_torque,
            "required_2x_running_torque_nm": two_x_threshold,
            "available_36V_lower_edge_nm": LEADSHINE_36V_300RPM_LOWER_EDGE_NM,
            "available_to_required_multiple": motor_multiple,
            "reserve_above_2x_threshold_nm": (
                LEADSHINE_36V_300RPM_LOWER_EDGE_NM - two_x_threshold
            ),
            "percent_above_2x_threshold": (
                (LEADSHINE_36V_300RPM_LOWER_EDGE_NM / two_x_threshold - 1.0)
                * 100.0
            ),
            "manufacturer_curve_motor_gate_ge_2x": motor_gate,
            "24V_available_to_required_multiple": at_24v_multiple,
            "24V_gate_ge_2x": LEADSHINE_24V_300RPM_LOWER_EDGE_NM >= two_x_threshold,
        },
        "transmission": {
            "motor_teeth": 30,
            "flyer_teeth": 30,
            "pitch_mm": 3.0,
            "belt_pitch_length_mm": 210.0,
            "exact_ratio": 1.0,
            "upstream_radians_contract_preserved": True,
            "NBK_base_allowable_at_300rpm_nm": 2.06,
            "belt_length_factor_210mm": 0.9,
            "mesh_factor": 1.0,
            "allowable_transmission_torque_nm": pulley_allowable_nm,
            "allowable_to_required_multiple": pulley_allowable_nm / required_torque,
            "transmission_capacity_gate_ge_2x": pulley_capacity_gate,
        },
        "motor_pulley_geometry_and_inertia": {
            "stock_part": "P30-3GT-BLP-6C-5",
            "selected_additional_machining": "BNW",
            "selected_order_description": (
                "stock split-clamp pulley, 5 mm bore, plus two radial set-screw "
                "holes at 90 degrees; final bore tolerance, screw size, screw "
                "length and insertion directions pending generated NBK drawing"
            ),
            "published_geometry_mm": {
                "teeth": 30,
                "pitch_diameter_Dp": 28.7,
                "tooth_OD_De": 27.9,
                "stock_hub_OD_Db": 20.0,
                "clamp_envelope_E": 23.0,
                "flange_OD_Df": 32.0,
                "overall_width_W": 11.0,
                "belt_channel_width_A": 7.3,
                "flange_thickness_I": 1.85,
                "hub_length_L": 7.5,
                "clamp_bolt_axial_F": 2.75,
                "clamp_bolt_radial_G": 7.5,
                "stock_bore": 5.0,
            },
            "published_materials": {
                "pulley": "A2017 anodized aluminum",
                "stock_clamp_bolt": "SCM435 black oxide",
            },
            "stock_clamp_bolt": {
                "size": "M2",
                "tightening_torque_nm": 0.5,
                "recommended_shaft_tolerance": "round shaft h6 or h7",
            },
            "BNW": {
                "set_screw_count": 2,
                "angular_spacing_deg": 90.0,
                "set_screws_included": True,
                "exact_set_screw_size_published_for_this_configuration": False,
                "exact_set_screw_length_published_for_this_configuration": False,
            },
            "published_stock_mass_g": 28.0,
            "published_stock_inertia_kgm2": MOTOR_P30_PUBLISHED_INERTIA_KGM2,
            "stock_M2_clamp_bolt_inertia_witness_kgm2": (
                MOTOR_PULLEY_M2_WITNESS_INERTIA_KGM2
            ),
            "BNW_inertia_bound": {
                "method": (
                    "two M3x12 steel radial cylinders from r=2.5 to14.5 mm; "
                    "conservative acceleration-load envelope, not delivered hardware"
                ),
                "one_bound_screw_mass_kg": ONE_BNW_INERTIA_BOUND_SCREW_MASS_KG,
                "one_bound_screw_inertia_kgm2": ONE_BNW_INERTIA_BOUND_SCREW_KGM2,
                "two_bound_screws_inertia_kgm2": (
                    BNW_SET_SCREW_COUNT * ONE_BNW_INERTIA_BOUND_SCREW_KGM2
                ),
            },
            "full_motor_pulley_inertia_upper_kgm2": (
                MOTOR_P30_SPLIT_CLAMP_BNW_INERTIA_UPPER_KGM2
            ),
            "official_stock_CAD": {
                "NBK_says_provider": "PARTcommunity by CADENAS WEB2CAD",
                "local_exact_STEP_acquired": True,
                "path": str(NBK_P30_D5_STOCK_STEP.relative_to(ROOT)).replace(
                    "\\", "/"
                ),
                "expected_sha256": NBK_P30_D5_STOCK_STEP_SHA256,
                "observed_sha256": _sha256(NBK_P30_D5_STOCK_STEP),
                "immutable_stock_only_BNW_absent": True,
                "configured_BNW_drawing_received": False,
                "request_form_available": True,
                "exact_stock_CAD_gate": (
                    _sha256(NBK_P30_D5_STOCK_STEP)
                    == NBK_P30_D5_STOCK_STEP_SHA256
                ),
            },
        },
        "motor_shaft_retention": {
            "shaft_is_round": False,
            "exact_profile": "5 mm D-profile, 4.5 mm across flat",
            "stock_split_clamp_round_h6_h7_interface_authorized": False,
            "selected_method": (
                "retain the stock M2 split clamp and add NBK BNW machining: two "
                "included radial set screws at 90 degrees; orient one screw on "
                "the D-flat and keep the flat clear of split/bolt spot faces"
            ),
            "NBK_guidance": (
                "clamping types are generally for round shafts; set the D-cut flat "
                "as the set-screw fastening position for set-screw types, and keep "
                "the flat clear of clamp slits and bolt spot faces"
            ),
            "stock_split_clamp_plus_BNW_configuration_is_orderable": True,
            "BNW_set_screw_count": 2,
            "BNW_set_screw_spacing_deg": 90.0,
            "exact_BNW_set_screw_size_known": False,
            "exact_motor_shaft_hardness_known": False,
            "reference_slip_chart": {
                "local_path": (
                    "cad/models/upgrades/"
                    "NBK_aluminum_set_screw_slip_torque_reference.jpg"
                ),
                "sha256": EXPECTED_SOURCE_HASHES[
                    "cad/models/upgrades/"
                    "NBK_aluminum_set_screw_slip_torque_reference.jpg"
                ],
                "material_conditions": (
                    "S45C shaft 16-27 HRC; SCM435 black-oxide set screws; "
                    "anodized aluminum-alloy hub"
                ),
                "two_screw_chart_spacing_deg": 90.0,
                "reference_only_not_guaranteed": True,
                "actual_use_testing_required_by_NBK": True,
                "direct_curve_at_5mm_for_unknown_BNW_screw_size_available": False,
                "proves_required_slip_torque": False,
            },
            "modified_pulley_retention_torque_published": False,
            "supplier_drawing_and_quote_received": False,
            "reversing_slip_coupon_passed": False,
            "retention_release_gate": False,
            "required_proof_nm": two_x_threshold,
        },
        "ratio_trade_1p0_to_3p0": {
            "scope": (
                "conservative motor/full-inertia math only; ratios above 1.0 need "
                "new exact tooth/belt packaging and change the frozen radians mapping"
            ),
            "selected_ratio": 1.0,
            "rows": _ratio_trade(retained_base),
        },
        "dual_NEMA17_fallback": {
            "motors": "2x 17HS24-2004D-E1K, two independent 1:1 belt lanes",
            "conservative_torque_each_at_300rpm_nm": 0.430,
            "full_output_inertia_kgm2": dual_inertia,
            "required_output_torque_nm": dual_required,
            "available_output_torque_nm": dual_available,
            "available_to_required_multiple": dual_available / dual_required,
            "static_motor_math_ge_2x": dual_available >= 2.0 * dual_required,
            "architecture_authorized": False,
            "reason": (
                "no manufacturer current/load-sharing method, encoder phase alignment, "
                "or common-fault interlock is published for two closed-loop drives on "
                "one mechanically coupled M2 axis; unnecessary because CS-M21708 passes"
            ),
        },
        "release_gates": {
            "standard_closed_loop_NEMA17": True,
            "exact_CS_D508_driver_officially_tested_with_CS_M21708": True,
            "driver_peak_capacity_exceeds_RMS_2p5A_equivalent": True,
            "driver_configuration_reproduces_RMS_2p5A_curve": False,
            "regulated_36V_supply_condition_defined": True,
            "exact_36V_supply_SKU_pinned": True,
            "exact_36V_supply_capacity_and_input_windows_pinned": True,
            "36V_supply_mains_safety_integration_pinned": False,
            "exact_1_to_1_ratio": True,
            "official_orderable_motor_and_CAD": True,
            "manufacturer_curve_36V_full_inertia_margin_ge_2": motor_gate,
            "P30_210_3GT_transmission_capacity_ge_2": pulley_capacity_gate,
            "motor_D_flat_interface_method_published": True,
            "motor_pulley_exact_stock_CAD_acquired": True,
            "motor_pulley_BNW_set_screw_size_and_drawing_known": False,
            "motor_shaft_hardness_matches_NBK_reference_chart": False,
            "NBK_reference_chart_guarantees_required_slip_torque": False,
            "motor_pulley_supplier_drawing_and_quote": False,
            "motor_pulley_reversing_retention_coupon": False,
            "flyer_pulley_supplier_drawing_and_quote": False,
            "flyer_pulley_reversing_retention_coupon": False,
            "installed_M2_friction_le_0p020Nm_measured": False,
            "integrated_exact_CAD_packaging_and_raw_collision_green": False,
            "hot_36V_300rpm_dyno_ge_required_2x": False,
            "production_authorized": False,
        },
        "controlling_open_gates": [
            "driver_configuration_reproduces_RMS_2p5A_curve",
            "36V_supply_mains_safety_integration_pinned",
            "motor_pulley_BNW_set_screw_size_and_drawing_known",
            "motor_shaft_hardness_matches_NBK_reference_chart",
            "NBK_reference_chart_guarantees_required_slip_torque",
            "motor_pulley_supplier_drawing_and_quote",
            "motor_pulley_reversing_retention_coupon",
            "flyer_pulley_supplier_drawing_and_quote",
            "flyer_pulley_reversing_retention_coupon",
            "installed_M2_friction_le_0p020Nm_measured",
            "integrated_exact_CAD_packaging_and_raw_collision_green",
            "hot_36V_300rpm_dyno_ge_required_2x",
        ],
    }
    return report


def _markdown(report: dict) -> str:
    duty = report["OD65_10N_full_inertia_torque"]
    transmission = report["transmission"]
    return f"""# Normal-GOAL M2 drive selection

Status: **conditional Leadshine CS-M21708 exact-1:1 selection; not production-authorized.**

The single CS-M21708 on its official 36 V / RMS 2.5 A curve supplies a conservative lower-edge **{duty['available_36V_lower_edge_nm']:.3f} N m at 300 RPM**.  The full bounded inertia is `{duty['full_output_inertia_kgm2']:.12g} kg m^2`; OD65/10 N wire torque, 0.020 N m friction allowance, and 200 rad/s^2 acceleration require `{duty['required_output_torque_nm']:.6f} N m`, so the 2x threshold is `{duty['required_2x_running_torque_nm']:.6f} N m`.  Curve margin is **{duty['available_to_required_multiple']:.3f}x** ({duty['percent_above_2x_threshold']:.3f}% above the threshold).  The 24 V curve does not pass.

The exact CS-D508 manual lists CS-M21708 as tested-compatible and its 8.0 A-peak capacity exceeds the sinusoidal equivalent of the curve current.  The **hardware model is selected but its production configuration is conditional**: 2.5 A RMS is 3.5355 A peak, while ProTuner PR 2 programs 0.1 A increments, and Leadshine's motor page also contradicts itself between 2.5 A marketing text and a 1.5 A table entry.  Neither 3.5 nor 3.6 A peak may be assumed to reproduce the curve.  Obtain Leadshine's exact setup or pass the hot 36 V / 300 RPM dyno gate.

The exact regulated-supply candidate is Leadshine **LSP-360-36**: 36 VDC, 10 A continuous, 18 A peak, 360 W, with selectable 92-138 or 184-276 VAC input.  Its active manufacturer page is cart-enabled, but that does not release the mains enclosure, branch protection, disconnect, protective-earth bonding, mounting, or wiring.

Keep the frozen 30T:30T exact 1:1 drive and 210-3GT-6 belt.  NBK's corrected allowable transmission torque is `{transmission['allowable_transmission_torque_nm']:.3f} N m`, comfortably above the 2x load threshold.

The exact STEP proves the exposed shaft is a 5 mm D-profile, 4.5 mm across the flat.  The provisional motor pulley is therefore the stock NBK P30-3GT-BLP-6C-5 split clamp plus **BNW** additional machining: two included set screws at 90 degrees, with one aimed at the D-flat and the flat kept clear of the clamp slit/bolt spot face.  The full torque calculation includes a conservative two-M3x12 inertia envelope for those screws.

NBK's aluminum-hub slip chart is reference-only, assumes an S45C shaft at 16-27 HRC, and requires actual-use testing.  It provides no direct 5 mm point for the unknown delivered BNW screw size, so it does not prove `{duty['required_2x_running_torque_nm']:.6f} N m`.  The immutable exact stock D5 CAD is acquired and hash-pinned, but it intentionally contains no BNW holes.  Obtain the generated BNW drawing/quote, confirm screw size and Leadshine shaft hardness, and pass a reversing coupon before procurement release.  The flyer-side custom P30 interface needs the same proof.

Open production gates: installed M2 friction <=0.020 N m, hot 36 V dyno >={duty['required_2x_running_torque_nm']:.6f} N m at 300 RPM, exact integrated packaging/raw collision, both supplier retention drawings/quotes, both reversing retention coupons, and physical balance gates inherited from the retained flyer review.
"""


def write_outputs() -> dict:
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
