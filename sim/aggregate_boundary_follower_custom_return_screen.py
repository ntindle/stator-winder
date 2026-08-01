"""Deterministic custom spring/flexure screen for follower retraction.

This module records analytical design candidates following the stock-component
procurement no-go.  It is a design screen only: no candidate is physically
qualified, integrated in CAD, selected for procurement, added to the BOM, or
authorized for release.
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
OUTPUT_JSON = REPORTS / "aggregate_boundary_follower_custom_return_screen.json"
OUTPUT_MD = REPORTS / "aggregate_boundary_follower_custom_return_screen.md"

SCHEMA = "aggregate-boundary-follower-custom-return-screen/v1"
STUDY_DATE = "2026-07-12"
LBF_TO_N = 4.4482216152605

TANGENTIAL_RATE_TARGET_N_PER_MM = 0.30
TANGENTIAL_HARD_HALF_TRAVEL_MM = 0.60
RADIAL_RETURN_MINIMUM_N = 0.25
RADIAL_RETURN_HARD_MAXIMUM_N = 0.303291
RADIAL_HARD_TRAVEL_MM = 6.40
RADIAL_RATE_CEILING_N_PER_MM = (
    RADIAL_RETURN_HARD_MAXIMUM_N - RADIAL_RETURN_MINIMUM_N
) / RADIAL_HARD_TRAVEL_MM


def _rounded(value: float) -> float:
    return round(float(value), 9)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(value: dict[str, Any]) -> str:
    body = dict(value)
    body.pop("report_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def source_records() -> dict[str, dict[str, Any]]:
    """Return the material, process and spring sources used by this screen."""

    return {
        "ulbrich_301_wire": {
            "source": "Ulbrich official alloy data sheet",
            "accessed_date": STUDY_DATE,
            "url": (
                "https://www.ulbrich.com/uploads/data-sheets/"
                "301-Stainless-Steel-Wire-UNS-S30100-Wire.pdf"
            ),
            "use": "301 full-hard strength screen",
        },
        "ulbrich_301si": {
            "source": "Ulbrich official alloy page",
            "accessed_date": STUDY_DATE,
            "url": "https://www.ulbrich.com/alloys/301si-stainless-steel-uns-s30116/",
            "use": "301Si elastic modulus and spring-material context",
        },
        "elgiloy_17_7ph": {
            "source": "Elgiloy official 17-7PH strip data sheet",
            "accessed_date": STUDY_DATE,
            "url": (
                "https://www.elgiloy.com/wp-content/uploads/2021/03/"
                "Strip-17-7-PH-Stainless-Steel.pdf"
            ),
            "use": "17-7PH spring application and fatigue context",
        },
        "smith_17_7ph": {
            "source": "Smiths Metal Centres 17-7PH technical sheet",
            "accessed_date": STUDY_DATE,
            "url": "https://www.smithmetal.com/pdf/stainless/ph/17-7ph.pdf",
            "use": "CH900 ultimate and yield strength screen",
        },
        "smalley_material_guide": {
            "source": "Smalley official material selection guide hosted by ASME",
            "accessed_date": STUDY_DATE,
            "url": (
                "https://resources.asme.org/hubfs/ME%20Mag/Assets/"
                "Smalley_Material%20Selection%20Guide.pdf"
            ),
            "use": "spring material selection context",
        },
        "precision_micro_flat_springs": {
            "source": "Precision Micro official process guidance",
            "accessed_date": STUDY_DATE,
            "url": (
                "https://www.precisionmicro.com/chemical-etchings-role-in-"
                "custom-flat-spring-production/"
            ),
            "use": "non-thermal etched spring-steel manufacturing screen",
        },
        "precision_micro_design": {
            "source": "Precision Micro official etching design guide",
            "accessed_date": STUDY_DATE,
            "url": (
                "https://www.precisionmicro.com/"
                "design-engineers-guide-photo-chemical-etching/"
            ),
            "use": "feature and tolerance screen",
        },
        "stratasys_nylon12": {
            "source": "Stratasys official FDM Nylon 12 material data sheet",
            "accessed_date": STUDY_DATE,
            "url": (
                "https://www.stratasys.com/siteassets/materials/materials-catalog/"
                "fdm-materials/nylon-12/mds_fdm_nylon-12_0921a.pdf"
            ),
            "use": "conditioned Nylon 12 modulus and yield screen",
        },
        "stratasys_nylon12cf": {
            "source": "Stratasys official FDM Nylon 12CF material data sheet",
            "accessed_date": STUDY_DATE,
            "url": (
                "https://aerospace.stratasys.com/assets/downloads/ergonomics/"
                "Data_Sheet-EN_FDM_Nylon_12CF.pdf"
            ),
            "use": "anisotropy and low-elongation rejection evidence",
        },
        "vulcan_constant_force": {
            "source": "Vulcan Spring official design guide",
            "accessed_date": STUDY_DATE,
            "url": (
                "https://info.vulcanspring.com/hubfs/"
                "Vulcan_Spring_Design_Guide-1.pdf"
            ),
            "use": "constant-force tolerance and life context",
        },
        "lee_constant_force": {
            "source": "Lee Spring official engineering guidance",
            "accessed_date": STUDY_DATE,
            "url": "https://www.leespring.com/learn-about-constant-force-springs",
            "use": "initial extension and mounting screen",
        },
        "lee_flat_springs": {
            "source": "Lee Spring official flat-spring guidance",
            "accessed_date": STUDY_DATE,
            "url": "https://www.leespring.com/flat-springs",
            "use": "flat-spring manufacturing context",
        },
        "mcmaster_9293K122": {
            "source": "McMaster-Carr official constant-force spring catalog",
            "accessed_date": STUDY_DATE,
            "url": "https://www.mcmaster.com/products/constant-force-springs/",
            "catalog_number": "9293K122",
            "use": "reduced-force cartridge input dimensions and force",
        },
    }


def torsion_wire_pair() -> dict[str, Any]:
    """Screen a pair of opposed torsion springs on the proposed 3 mm shaft."""

    elastic_modulus_mpa = 200_000.0
    wire_diameter_mm = 0.30
    mean_coil_diameter_mm = 4.00
    active_coils = 2.63671875
    arm_radius_mm = 4.00
    shaft_diameter_mm = 3.00
    spring_uts_mpa = 1_827.0
    spring_yield_mpa = 1_793.0
    endurance_fraction_of_uts = 0.35

    rotational_rate = (
        elastic_modulus_mpa * wire_diameter_mm ** 4
        / (64.0 * mean_coil_diameter_mm * active_coils)
    )
    linear_rate_each = rotational_rate / arm_radius_mm ** 2
    center_force_each = 0.15
    center_torque = center_force_each * arm_radius_mm
    center_prewind_rad = center_torque / rotational_rate
    hard_rotation_rad = TANGENTIAL_HARD_HALF_TRAVEL_MM / arm_radius_mm
    low_torque = rotational_rate * (center_prewind_rad - hard_rotation_rad)
    high_torque = rotational_rate * (center_prewind_rad + hard_rotation_rad)
    low_force = low_torque / arm_radius_mm
    high_force = high_torque / arm_radius_mm

    spring_index = mean_coil_diameter_mm / wire_diameter_mm
    inside_fiber_factor = (
        (4.0 * spring_index ** 2 - spring_index - 1.0)
        / (4.0 * spring_index * (spring_index - 1.0))
    )

    def stress_mpa(torque_n_mm: float) -> float:
        return (
            inside_fiber_factor * 32.0 * torque_n_mm
            / (math.pi * wire_diameter_mm ** 3)
        )

    low_stress = stress_mpa(low_torque)
    high_stress = stress_mpa(high_torque)
    mean_stress = (low_stress + high_stress) / 2.0
    alternating_stress = (high_stress - low_stress) / 2.0
    screening_endurance_mpa = endurance_fraction_of_uts * spring_uts_mpa
    goodman_utilization = (
        alternating_stress / screening_endurance_mpa
        + mean_stress / spring_uts_mpa
    )

    tolerance_terms = {
        "wire_diameter_d4_fraction": 4.0 * 0.005 / wire_diameter_mm,
        "mean_diameter_inverse_fraction": 0.05 / mean_coil_diameter_mm,
        "active_coils_inverse_fraction": 0.03 / active_coils,
        "arm_radius_inverse_square_fraction": 2.0 * 0.05 / arm_radius_mm,
    }
    tolerance_rss = math.sqrt(sum(v ** 2 for v in tolerance_terms.values()))
    tolerance_worst = sum(tolerance_terms.values())

    coil_id = mean_coil_diameter_mm - wire_diameter_mm
    coil_od = mean_coil_diameter_mm + wire_diameter_mm
    estimated_body_length_max = 1.20
    bushing_body_length = 5.00
    shaft_span = 16.00

    return {
        "status": "PREFERRED_CUSTOM_TANGENTIAL_CONCEPT_NOT_QUALIFIED",
        "concept": "opposed 17-7PH CH900 torsion-wire pair",
        "geometry": {
            "shaft_diameter_mm": shaft_diameter_mm,
            "wire_diameter_mm": wire_diameter_mm,
            "mean_coil_diameter_mm": mean_coil_diameter_mm,
            "coil_inside_diameter_mm": _rounded(coil_id),
            "coil_outside_diameter_mm": _rounded(coil_od),
            "active_coils_each": active_coils,
            "arm_radius_mm": arm_radius_mm,
            "estimated_coil_body_length_max_each_mm": estimated_body_length_max,
            "proposed_shaft_span_mm": shaft_span,
            "bushing_body_length_mm": bushing_body_length,
            "remaining_axial_span_after_bushing_and_two_coils_mm": _rounded(
                shaft_span - bushing_body_length - 2.0 * estimated_body_length_max
            ),
            "radial_clearance_to_shaft_each_side_mm": _rounded(
                (coil_id - shaft_diameter_mm) / 2.0
            ),
        },
        "equations": {
            "rotational_rate": "k_theta = E*d^4/(64*D*N)",
            "linear_rate": "k_y = k_theta/r^2",
            "inside_fiber_factor": "K_i=(4*C^2-C-1)/(4*C*(C-1))",
            "wire_stress": "sigma=K_i*32*T/(pi*d^3)",
            "goodman": "U=sigma_a/S_e+sigma_m/S_ut",
        },
        "force_rate_screen": {
            "rotational_rate_N_mm_per_rad": _rounded(rotational_rate),
            "linear_rate_each_N_per_mm": _rounded(linear_rate_each),
            "opposed_net_rate_N_per_mm": _rounded(2.0 * linear_rate_each),
            "center_preload_each_N": center_force_each,
            "center_torque_each_N_mm": _rounded(center_torque),
            "center_prewind_rad": _rounded(center_prewind_rad),
            "center_prewind_deg": _rounded(math.degrees(center_prewind_rad)),
            "hard_rotation_each_direction_rad": _rounded(hard_rotation_rad),
            "hard_rotation_each_direction_deg": _rounded(
                math.degrees(hard_rotation_rad)
            ),
            "individual_force_range_N": [
                _rounded(low_force), _rounded(high_force),
            ],
            "net_restoring_force_at_hard_travel_N": _rounded(
                2.0 * linear_rate_each * TANGENTIAL_HARD_HALF_TRAVEL_MM
            ),
        },
        "stress_fatigue_screen": {
            "material": "17-7PH CH900 spring wire",
            "elastic_modulus_MPa": elastic_modulus_mpa,
            "ultimate_strength_MPa": spring_uts_mpa,
            "yield_strength_MPa": spring_yield_mpa,
            "spring_index": _rounded(spring_index),
            "inside_fiber_factor": _rounded(inside_fiber_factor),
            "stress_range_MPa": [_rounded(low_stress), _rounded(high_stress)],
            "mean_stress_MPa": _rounded(mean_stress),
            "alternating_stress_MPa": _rounded(alternating_stress),
            "assumed_screening_endurance_fraction_of_uts": (
                endurance_fraction_of_uts
            ),
            "assumed_screening_endurance_MPa": _rounded(screening_endurance_mpa),
            "modified_goodman_utilization": _rounded(goodman_utilization),
            "modified_goodman_screening_factor": _rounded(
                1.0 / goodman_utilization
            ),
            "static_yield_screening_factor": _rounded(
                spring_yield_mpa / high_stress
            ),
            "fatigue_life_qualified": False,
            "qualification_warning": (
                "The 0.35*Sut endurance value is an analytical screening "
                "assumption; winding damage, residual stress, heat treatment, "
                "surface finish and the real spectrum remain untested."
            ),
        },
        "tolerance_screen": {
            "assumed_tolerances": {
                "wire_diameter_mm": 0.005,
                "mean_coil_diameter_mm": 0.05,
                "active_coils": 0.03,
                "arm_radius_mm": 0.05,
                "prewind_index_deg": 0.5,
            },
            "first_order_relative_terms": {
                key: _rounded(value) for key, value in tolerance_terms.items()
            },
            "estimated_rate_RSS_fraction": _rounded(tolerance_rss),
            "estimated_rate_worst_case_fraction": _rounded(tolerance_worst),
            "force_shift_for_2deg_prewind_error_N": _rounded(
                rotational_rate * math.radians(2.0) / arm_radius_mm
            ),
            "production_controls": [
                "indexed or slotted fixed-leg anchor with <=0.5 degree adjustment",
                "load-test and match opposed pairs",
                "proof-load after final age treatment",
            ],
        },
        "manufacturing": {
            "credible_route": (
                "specialist winding of 17-7PH wire followed by the supplier's "
                "controlled age treatment and proof test"
            ),
            "fits_analytical_shaft_envelope": True,
            "shaft_and_anchor_CAD_integrated": False,
            "endurance_tested": False,
        },
        "source_keys": [
            "elgiloy_17_7ph", "smith_17_7ph", "smalley_material_guide",
        ],
    }


def etched_flat_flexure() -> dict[str, Any]:
    """Screen two fixed-guided etched 17-7PH leaves."""

    elastic_modulus_mpa = 200_000.0
    length_mm = 11.0
    thickness_mm = 0.10
    rate_each = TANGENTIAL_RATE_TARGET_N_PER_MM / 2.0
    width_mm = (
        rate_each * length_mm ** 3
        / (elastic_modulus_mpa * thickness_mm ** 3)
    )
    computed_rate_each = (
        elastic_modulus_mpa * width_mm * thickness_mm ** 3 / length_mm ** 3
    )
    stress = (
        3.0 * elastic_modulus_mpa * thickness_mm
        * TANGENTIAL_HARD_HALF_TRAVEL_MM / length_mm ** 2
    )
    uts_mpa = 1_827.0
    yield_mpa = 1_793.0
    endurance_mpa = 0.35 * uts_mpa

    tolerance_terms = {
        "width_fraction": 0.02 / width_mm,
        "thickness_cubed_fraction": 3.0 * 0.002 / thickness_mm,
        "length_inverse_cubed_fraction": 3.0 * 0.05 / length_mm,
    }
    tolerance_rss = math.sqrt(sum(v ** 2 for v in tolerance_terms.values()))

    return {
        "status": "CREDIBLE_TANGENTIAL_ALTERNATIVE_NOT_QUALIFIED",
        "concept": "monolithic etched 17-7PH CH900 fixed-guided leaf pair",
        "geometry_each_leaf": {
            "length_mm": length_mm,
            "width_mm": _rounded(width_mm),
            "thickness_mm": thickness_mm,
            "hard_translation_mm": TANGENTIAL_HARD_HALF_TRAVEL_MM,
            "recommended_root_radius_min_mm": 0.50,
            "leaf_count": 2,
        },
        "equations": {
            "fixed_guided_rate": "k=E*b*t^3/L^3",
            "maximum_bending_stress": "sigma_max=3*E*t*delta/L^2",
        },
        "force_rate_screen": {
            "rate_each_N_per_mm": _rounded(computed_rate_each),
            "combined_rate_N_per_mm": _rounded(2.0 * computed_rate_each),
            "net_restoring_force_at_hard_travel_N": _rounded(
                2.0 * computed_rate_each * TANGENTIAL_HARD_HALF_TRAVEL_MM
            ),
            "static_preload_required": False,
        },
        "stress_fatigue_screen": {
            "material": "17-7PH CH900 sheet or foil",
            "elastic_modulus_MPa": elastic_modulus_mpa,
            "ultimate_strength_MPa": uts_mpa,
            "yield_strength_MPa": yield_mpa,
            "maximum_stress_MPa": _rounded(stress),
            "static_yield_screening_factor": _rounded(yield_mpa / stress),
            "assumed_screening_endurance_MPa": _rounded(endurance_mpa),
            "zero_mean_fatigue_screening_factor": _rounded(endurance_mpa / stress),
            "fatigue_life_qualified": False,
        },
        "tolerance_screen": {
            "assumed_tolerances": {
                "width_mm": 0.02,
                "thickness_mm": 0.002,
                "length_mm": 0.05,
            },
            "first_order_relative_terms": {
                key: _rounded(value) for key, value in tolerance_terms.items()
            },
            "estimated_rate_RSS_fraction": _rounded(
                math.sqrt(sum(v ** 2 for v in tolerance_terms.values()))
            ),
            "estimated_rate_worst_case_fraction": _rounded(
                sum(tolerance_terms.values())
            ),
            "production_controls": [
                "fixture final leaf spacing and parallelism",
                "proof-load and sort finished plates by rate",
                "keep etched root transitions free of witness notches",
            ],
        },
        "manufacturing": {
            "preferred_route": "photochemical etching of full-hard or age-hardened foil",
            "preferred_route_credible": True,
            "preferred_route_reason": (
                "non-thermal, burr-free cutting is compatible with the 0.10 mm "
                "foil and approximately 1 mm leaves"
            ),
            "laser_cut_route": "prototype-only",
            "laser_cut_warning": (
                "heat-affected edges, recast, burrs and distortion occur at the "
                "fatigue-critical leaf roots; edge finishing and coupon fatigue "
                "tests would be mandatory"
            ),
            "CAD_integrated": False,
        },
        "source_keys": [
            "elgiloy_17_7ph", "smith_17_7ph",
            "precision_micro_flat_springs", "precision_micro_design",
            "lee_flat_springs",
        ],
    }


def nylon12_prototype_flexure() -> dict[str, Any]:
    """Screen a zero-preload printed tangential prototype flexure."""

    nominal_modulus_mpa = 1_200.0
    modulus_range_mpa = [1_140.0, 1_280.0]
    length_mm = 11.0
    width_mm = 1.0
    target_each = TANGENTIAL_RATE_TARGET_N_PER_MM / 2.0
    thickness_mm = (
        target_each * length_mm ** 3 / (nominal_modulus_mpa * width_mm)
    ) ** (1.0 / 3.0)
    rate_each = (
        nominal_modulus_mpa * width_mm * thickness_mm ** 3 / length_mm ** 3
    )
    stress = (
        3.0 * nominal_modulus_mpa * thickness_mm
        * TANGENTIAL_HARD_HALF_TRAVEL_MM / length_mm ** 2
    )
    strain = stress / nominal_modulus_mpa
    conditioned_yield_range_mpa = [28.0, 32.0]

    def pair_rate(modulus_mpa: float, thickness: float) -> float:
        return (
            2.0 * modulus_mpa * width_mm * thickness ** 3 / length_mm ** 3
        )

    lower_rate = pair_rate(modulus_range_mpa[0], thickness_mm - 0.05)
    upper_rate = pair_rate(modulus_range_mpa[1], thickness_mm + 0.05)

    return {
        "status": "TANGENTIAL_REPLACEABLE_PROTOTYPE_ONLY",
        "concept": "printed Nylon 12 zero-preload fixed-guided leaf pair",
        "geometry_each_leaf": {
            "length_mm": length_mm,
            "width_mm": width_mm,
            "thickness_mm": _rounded(thickness_mm),
            "hard_translation_mm": TANGENTIAL_HARD_HALF_TRAVEL_MM,
            "leaf_count": 2,
        },
        "equations": {
            "fixed_guided_rate": "k=E*b*t^3/L^3",
            "maximum_bending_stress": "sigma_max=3*E*t*delta/L^2",
            "linear_strain_screen": "epsilon=sigma/E",
        },
        "force_rate_screen": {
            "nominal_conditioned_modulus_MPa": nominal_modulus_mpa,
            "published_conditioned_modulus_range_MPa": modulus_range_mpa,
            "nominal_rate_each_N_per_mm": _rounded(rate_each),
            "nominal_combined_rate_N_per_mm": _rounded(2.0 * rate_each),
            "combined_rate_range_for_E_and_thickness_screen_N_per_mm": [
                _rounded(lower_rate), _rounded(upper_rate),
            ],
            "thickness_tolerance_screen_mm": 0.05,
            "static_preload_required": False,
        },
        "stress_screen": {
            "maximum_nominal_stress_MPa": _rounded(stress),
            "maximum_nominal_strain_fraction": _rounded(strain),
            "conditioned_yield_strength_range_MPa": conditioned_yield_range_mpa,
            "static_screening_factor_using_low_yield": _rounded(
                conditioned_yield_range_mpa[0] / stress
            ),
            "fatigue_life_qualified": False,
            "creep_qualified": False,
            "humidity_temperature_conditioned_rate_qualified": False,
        },
        "manufacturing": {
            "tangential_prototype_credible": True,
            "radial_continuous_bias_credible": False,
            "reason": (
                "orientation, bead dimensions, conditioning, moisture, "
                "temperature and creep can move the nominal rate; the concept is "
                "bounded only because it is replaceable, has zero preload, and "
                "the captured slot and M0 interlock remain independent"
            ),
            "nylon12cf_preferred": False,
            "nylon12cf_reject_reason": (
                "high directional stiffness and low elongation make the small "
                "repeated flexure more brittle and less predictable"
            ),
            "CAD_integrated": False,
        },
        "source_keys": ["stratasys_nylon12", "stratasys_nylon12cf"],
    }


def reduced_constant_force_cartridge() -> dict[str, Any]:
    """Screen 9293K122 behind a calibrated force-reduction lever."""

    nominal_force_n = 0.23 * LBF_TO_N
    minimum_force_n = 0.20 * LBF_TO_N
    maximum_force_n = 0.26 * LBF_TO_N
    calibrated_target_n = 0.276
    calibration_tolerance_n = 0.010
    calibrated_range = [
        _rounded(calibrated_target_n - calibration_tolerance_n),
        _rounded(calibrated_target_n + calibration_tolerance_n),
    ]
    ratios = {
        "at_catalog_max_force": calibrated_target_n / maximum_force_n,
        "at_catalog_nominal_force": calibrated_target_n / nominal_force_n,
        "at_catalog_min_force": calibrated_target_n / minimum_force_n,
    }
    adjustment_range = [0.235, 0.315]
    coil_id_mm = 0.53 * 25.4
    coil_od_mm = 0.62 * 25.4
    strip_width_mm = 0.25 * 25.4
    envelope = {
        "x_mm": [17.125, 32.875],
        "y_mm": [-27.875, -12.125],
        "z_mm": [16.825, 23.175],
    }
    symmetric_allowed_tolerance = (
        (RADIAL_RETURN_HARD_MAXIMUM_N - RADIAL_RETURN_MINIMUM_N)
        / (RADIAL_RETURN_HARD_MAXIMUM_N + RADIAL_RETURN_MINIMUM_N)
    )

    return {
        "status": "BOUNDED_RADIAL_PROTOTYPE_CONCEPT_NOT_QUALIFIED",
        "concept": (
            "fixed McMaster 9293K122 cartridge with an adjustable independent "
            "force-reduction lever or strap"
        ),
        "input_cartridge": {
            "catalog_number": "9293K122",
            "nominal_force_N": _rounded(nominal_force_n),
            "minimum_catalog_tolerance_force_N": _rounded(minimum_force_n),
            "maximum_catalog_tolerance_force_N": _rounded(maximum_force_n),
            "coil_inside_diameter_mm": _rounded(coil_id_mm),
            "coil_outside_diameter_mm": _rounded(coil_od_mm),
            "strip_width_mm": _rounded(strip_width_mm),
            "extended_length_mm": 457.2,
            "catalog_life_cycles": 25_000,
        },
        "equations": {
            "work_ratio": "m=ds_spring/dx_slide=F_output/F_spring",
            "effective_rate_ceiling": (
                "k_max=(F_hard_max-F_return_min)/radial_hard_travel"
            ),
            "symmetric_force_tolerance": (
                "tol=(F_high-F_low)/(F_high+F_low)"
            ),
        },
        "force_budget": {
            "required_minimum_return_force_N": RADIAL_RETURN_MINIMUM_N,
            "hard_maximum_return_force_N": RADIAL_RETURN_HARD_MAXIMUM_N,
            "hard_travel_mm": RADIAL_HARD_TRAVEL_MM,
            "maximum_effective_rate_N_per_mm": _rounded(
                RADIAL_RATE_CEILING_N_PER_MM
            ),
            "maximum_symmetric_tolerance_fraction": _rounded(
                symmetric_allowed_tolerance
            ),
            "calibrated_target_N": calibrated_target_n,
            "calibrated_tolerance_N": calibration_tolerance_n,
            "calibrated_acceptance_range_N": calibrated_range,
            "lower_requirement_margin_N": _rounded(
                calibrated_range[0] - RADIAL_RETURN_MINIMUM_N
            ),
            "upper_requirement_margin_N": _rounded(
                RADIAL_RETURN_HARD_MAXIMUM_N - calibrated_range[1]
            ),
            "full_stroke_force_and_rate_qualified": False,
        },
        "reduction_mechanism": {
            "required_ratios": {
                key: _rounded(value) for key, value in ratios.items()
            },
            "proposed_adjustment_ratio_range": adjustment_range,
            "spring_motion_over_6p4mm_using_required_ratios_mm": [
                _rounded(min(ratios.values()) * RADIAL_HARD_TRAVEL_MM),
                _rounded(max(ratios.values()) * RADIAL_HARD_TRAVEL_MM),
            ],
            "spring_motion_over_6p4mm_adjustment_extremes_mm": [
                _rounded(adjustment_range[0] * RADIAL_HARD_TRAVEL_MM),
                _rounded(adjustment_range[1] * RADIAL_HARD_TRAVEL_MM),
            ],
            "calibration_required_for_each_cartridge": True,
            "acceptance": [
                "measure output at multiple points across the complete 6.4 mm stroke",
                "all measured output forces remain within 0.266 to 0.286 N",
                "measured effective rate remains <=0.008326719 N/mm",
            ],
        },
        "package_screen": {
            "proposed_fixed_cartridge_local_envelope": envelope,
            "coil_axis": "+Z",
            "negative_Y_service_pocket": True,
            "analytically_envelope_plausible": True,
            "full_carrier_gimbal_collision_sweep_complete": False,
            "fragment_containment_required": True,
            "initial_takeup_for_full_load_mm": [
                _rounded(1.25 * coil_id_mm), _rounded(1.25 * coil_od_mm),
            ],
        },
        "manufacturing_and_life": {
            "prototype_credible_after_CAD_sweep_and_calibration": True,
            "production_credible": False,
            "reason": (
                "catalog life is only 25,000 cycles and neither the real "
                "microcycle spectrum nor hot endurance has been qualified"
            ),
            "direct_connection_to_radial_slide": True,
            "bypasses_LEM_bellcrank": True,
            "CAD_integrated": False,
            "procurement_selected": False,
        },
        "source_keys": [
            "mcmaster_9293K122", "lee_constant_force", "vulcan_constant_force",
        ],
    }


def rejected_concepts() -> list[dict[str, Any]]:
    """Return deterministic rejection calculations for discarded concepts."""

    minimum_linear_preload_deflection = (
        RADIAL_RETURN_MINIMUM_N / RADIAL_RATE_CEILING_N_PER_MM
    )

    strip_elastic_modulus_mpa = 193_000.0

    def strip_width(force_n: float, thickness_mm: float, radius_mm: float) -> float:
        return (
            force_n * 24.0 * radius_mm ** 2
            / (strip_elastic_modulus_mpa * thickness_mm ** 3)
        )

    def strip_stress(thickness_mm: float, radius_mm: float) -> float:
        return strip_elastic_modulus_mpa * thickness_mm / (2.0 * radius_mm)

    compact_strip_examples = []
    for thickness, radius in ((0.05, 4.0), (0.07, 7.0), (0.10, 10.0), (0.10, 14.0)):
        compact_strip_examples.append({
            "thickness_mm": thickness,
            "coil_radius_mm": radius,
            "coil_diameter_mm": 2.0 * radius,
            "required_width_mm": _rounded(
                strip_width(0.276, thickness, radius)
            ),
            "idealized_straightening_stress_MPa": _rounded(
                strip_stress(thickness, radius)
            ),
        })

    buckling_length_mm = 20.0
    buckling_width_mm = 3.0
    buckling_load_n = 0.276
    buckling_thickness = (
        buckling_load_n * 12.0 * buckling_length_mm ** 2
        / (math.pi ** 2 * strip_elastic_modulus_mpa * buckling_width_mm)
    ) ** (1.0 / 3.0)

    return [
        {
            "concept": "direct 9293K122 cartridge",
            "status": "REJECT_FORCE",
            "calculation": {
                "minimum_catalog_force_N": _rounded(0.20 * LBF_TO_N),
                "hard_force_ceiling_N": RADIAL_RETURN_HARD_MAXIMUM_N,
                "minimum_force_multiple_of_ceiling": _rounded(
                    (0.20 * LBF_TO_N) / RADIAL_RETURN_HARD_MAXIMUM_N
                ),
            },
            "reason": "even minimum catalog force exceeds the complete reserve",
        },
        {
            "concept": "preloaded linear leaf radial return",
            "status": "REJECT_REQUIRED_PRELOAD_TRAVEL",
            "calculation": {
                "equation": "x_preload_min=F_min/k_max",
                "minimum_preload_deflection_mm": _rounded(
                    minimum_linear_preload_deflection
                ),
                "available_radial_hard_travel_mm": RADIAL_HARD_TRAVEL_MM,
            },
            "reason": "minimum preload displacement is outside the current envelope",
        },
        {
            "concept": "hand-made compact raw constant-force strip",
            "status": "REJECT_UNCONTROLLED_STRESS_AND_PROCESS",
            "calculation": {
                "force_equation": "F approximately E*b*t^3/(24*R^2)",
                "stress_equation": "sigma approximately E*t/(2*R)",
                "material_screen": "301/301Si full hard, E=193 GPa",
                "examples": compact_strip_examples,
                "full_hard_301_yield_MPa": 965.0,
            },
            "reason": (
                "compact examples approach or exceed full-hard 301 yield before "
                "edge, residual-stress and preforming effects; controlled rolling "
                "and forming require a specialist spring manufacturer"
            ),
        },
        {
            "concept": "post-buckled steel plateau flexure",
            "status": "REJECT_NONLINEAR_STABILITY_UNPROVEN",
            "calculation": {
                "equation": "P_cr=pi^2*E*b*t^3/(12*L^2)",
                "target_load_N": buckling_load_n,
                "length_mm": buckling_length_mm,
                "width_mm": buckling_width_mm,
                "ideal_pinned_beam_thickness_mm": _rounded(buckling_thickness),
            },
            "reason": (
                "plateau load and snap behavior are highly sensitive to end "
                "rotation, thickness and initial curvature"
            ),
        },
        {
            "concept": "printed polymer continuously biased radial return",
            "status": "REJECT_CREEP_AND_FORCE_DRIFT",
            "calculation": {
                "allowed_symmetric_force_tolerance_fraction": _rounded(
                    (RADIAL_RETURN_HARD_MAXIMUM_N - RADIAL_RETURN_MINIMUM_N)
                    / (RADIAL_RETURN_HARD_MAXIMUM_N + RADIAL_RETURN_MINIMUM_N)
                ),
                "creep_qualified": False,
                "moisture_temperature_qualified": False,
            },
            "reason": (
                "continuous preload plus moisture, temperature and creep drift is "
                "not bounded inside the narrow radial force window"
            ),
        },
        {
            "concept": "laser-cut and hand-formed constant-force strip",
            "status": "REJECT_MANUFACTURING_ROUTE",
            "calculation": {
                "controlled_cold_rolling_proven": False,
                "preforming_and_residual_stress_proven": False,
                "edge_fatigue_qualified": False,
            },
            "reason": "the route cannot establish repeatable constant-force behavior",
        },
    ]


def build_report() -> dict[str, Any]:
    torsion = torsion_wire_pair()
    etched = etched_flat_flexure()
    nylon = nylon12_prototype_flexure()
    cartridge = reduced_constant_force_cartridge()
    rejected = rejected_concepts()

    evidence_gates = {
        "torsion_pair_hits_0p30N_per_mm": math.isclose(
            torsion["force_rate_screen"]["opposed_net_rate_N_per_mm"],
            TANGENTIAL_RATE_TARGET_N_PER_MM,
            abs_tol=1e-9,
        ),
        "torsion_pair_fits_analytical_3mm_shaft_span": (
            torsion["manufacturing"]["fits_analytical_shaft_envelope"]
            and torsion["geometry"]["radial_clearance_to_shaft_each_side_mm"] > 0
            and torsion["geometry"]
            ["remaining_axial_span_after_bushing_and_two_coils_mm"] > 0
        ),
        "etched_leaf_pair_hits_0p30N_per_mm": math.isclose(
            etched["force_rate_screen"]["combined_rate_N_per_mm"],
            TANGENTIAL_RATE_TARGET_N_PER_MM,
            abs_tol=1e-9,
        ),
        "nylon_case_is_prototype_only_and_not_radial": (
            nylon["status"] == "TANGENTIAL_REPLACEABLE_PROTOTYPE_ONLY"
            and nylon["manufacturing"]["radial_continuous_bias_credible"] is False
        ),
        "cartridge_adjustment_covers_catalog_force_tolerance": (
            min(cartridge["reduction_mechanism"]["required_ratios"].values())
            >= cartridge["reduction_mechanism"]["proposed_adjustment_ratio_range"][0]
            and max(cartridge["reduction_mechanism"]["required_ratios"].values())
            <= cartridge["reduction_mechanism"]["proposed_adjustment_ratio_range"][1]
        ),
        "cartridge_calibration_band_inside_force_window": (
            cartridge["force_budget"]["calibrated_acceptance_range_N"][0]
            >= RADIAL_RETURN_MINIMUM_N
            and cartridge["force_budget"]["calibrated_acceptance_range_N"][1]
            <= RADIAL_RETURN_HARD_MAXIMUM_N
        ),
        "all_discarded_concepts_remain_rejected": all(
            row["status"].startswith("REJECT") for row in rejected
        ),
        "source_urls_bound": all(
            value.get("url", "").startswith("https://")
            for value in source_records().values()
        ),
    }

    fail_closed_gates = {
        "torsion_pair_manufactured_and_metrologized": False,
        "torsion_pair_endurance_qualified": False,
        "etched_flexure_manufactured_and_rate_sorted": False,
        "etched_flexure_endurance_qualified": False,
        "nylon12_creep_and_environment_qualified": False,
        "radial_cartridge_full_sweep_collision_clear": False,
        "radial_cartridge_full_stroke_force_qualified": False,
        "radial_cartridge_hot_microcycle_endurance_qualified": False,
        "fragment_containment_validated": False,
        "hardware_single_fault_validation_complete": False,
    }

    report = {
        "schema": SCHEMA,
        "status": "CUSTOM_RETURN_DESIGN_SCREEN_COMPLETE_NO_PHYSICAL_AUTHORITY",
        "study_date": STUDY_DATE,
        "requirements": {
            "tangential_opposed_net_rate_N_per_mm": (
                TANGENTIAL_RATE_TARGET_N_PER_MM
            ),
            "tangential_hard_half_travel_mm": TANGENTIAL_HARD_HALF_TRAVEL_MM,
            "radial_independent_minimum_force_N": RADIAL_RETURN_MINIMUM_N,
            "radial_hard_maximum_force_N": RADIAL_RETURN_HARD_MAXIMUM_N,
            "radial_hard_travel_mm": RADIAL_HARD_TRAVEL_MM,
            "radial_maximum_effective_rate_N_per_mm": _rounded(
                RADIAL_RATE_CEILING_N_PER_MM
            ),
        },
        "recommendation": {
            "tangential_primary": "opposed 17-7PH CH900 torsion-wire pair",
            "tangential_secondary": "etched 17-7PH CH900 fixed-guided leaf pair",
            "tangential_replaceable_prototype": (
                "printed Nylon 12 zero-preload flexure"
            ),
            "radial_bounded_prototype": (
                "9293K122 cartridge with adjustable reduction lever"
            ),
            "production_selection": None,
        },
        "torsion_wire_pair": torsion,
        "etched_17_7ph_flexure": etched,
        "nylon12_prototype_flexure": nylon,
        "reduced_9293K122_cartridge": cartridge,
        "rejected_concepts": rejected,
        "sources": source_records(),
        "evidence_gates": evidence_gates,
        "fail_closed_gates": fail_closed_gates,
        "physical_authority": False,
        "CAD_authority": False,
        "procurement_authority": False,
        "BOM_change_authorized": False,
        "order_authorized": False,
        "release_authority": False,
        "source_bindings": {
            "sim/aggregate_boundary_follower_custom_return_screen.py": _sha256(
                Path(__file__).resolve()
            ),
            "sim/aggregate_boundary_follower_retraction_procurement.py": _sha256(
                HERE / "aggregate_boundary_follower_retraction_procurement.py"
            ),
            "sim/aggregate_boundary_follower_retraction_topology.py": _sha256(
                HERE / "aggregate_boundary_follower_retraction_topology.py"
            ),
        },
        "decision": (
            "Advance only coupon and CAD-sweep work: the torsion pair is the "
            "preferred tangential concept, the etched plate is a credible "
            "alternative, Nylon 12 is replaceable-prototype-only, and the reduced "
            "9293K122 cartridge is only a bounded radial prototype concept. No "
            "candidate has physical, CAD, procurement, BOM, order, or release "
            "authority."
        ),
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def validate_report_integrity(report: dict[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported custom-return design-screen schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("custom-return design-screen report hash mismatch")
    for relative, expected in report.get("source_bindings", {}).items():
        path = ROOT / Path(str(relative).replace("\\", "/"))
        if not path.is_file() or _sha256(path) != expected:
            raise ValueError(f"stale custom-return source {relative}")


def _markdown(report: dict[str, Any]) -> str:
    torsion = report["torsion_wire_pair"]
    etched = report["etched_17_7ph_flexure"]
    nylon = report["nylon12_prototype_flexure"]
    cartridge = report["reduced_9293K122_cartridge"]
    lines = [
        "# Aggregate follower custom return design screen",
        "",
        f"- Study date: {report['study_date']}",
        f"- Status: **{report['status']}**",
        "- Physical/CAD/procurement/BOM/order/release authority: **false**",
        "",
        "## Bounded recommendation",
        "",
        "| Function | Candidate | Screen result |",
        "|---|---|---|",
        "| Tangential primary | Opposed 17-7PH torsion-wire pair | Preferred; unqualified |",
        "| Tangential alternative | Etched 17-7PH leaf pair | Credible; unqualified |",
        "| Tangential prototype | Printed Nylon 12 leaf pair | Replaceable prototype only |",
        "| Independent radial return | Reduced 9293K122 cartridge | Bounded prototype only |",
        "",
        "## Opposed torsion-wire pair",
        "",
        f"Each spring uses Ø{torsion['geometry']['wire_diameter_mm']:.2f} mm wire, "
        f"a {torsion['geometry']['mean_coil_diameter_mm']:.2f} mm mean coil, "
        f"{torsion['geometry']['active_coils_each']:.3f} active turns and a "
        f"{torsion['geometry']['arm_radius_mm']:.1f} mm arm. Coil ID/OD is "
        f"{torsion['geometry']['coil_inside_diameter_mm']:.2f}/"
        f"{torsion['geometry']['coil_outside_diameter_mm']:.2f} mm.",
        "",
        "`k_theta=E*d^4/(64*D*N)` and `k_y=k_theta/r^2` give "
        f"{torsion['force_rate_screen']['linear_rate_each_N_per_mm']:.3f} N/mm "
        f"per spring and {torsion['force_rate_screen']['opposed_net_rate_N_per_mm']:.3f} "
        "N/mm opposed. Center preload is "
        f"{torsion['force_rate_screen']['center_preload_each_N']:.2f} N at "
        f"{torsion['force_rate_screen']['center_prewind_deg']:.2f} degrees prewind. "
        f"Hard-travel restoring force is "
        f"{torsion['force_rate_screen']['net_restoring_force_at_hard_travel_N']:.2f} N.",
        "",
        f"Corrected stress is {torsion['stress_fatigue_screen']['stress_range_MPa'][0]:.1f}–"
        f"{torsion['stress_fatigue_screen']['stress_range_MPa'][1]:.1f} MPa. "
        f"The assumed 0.35*Sut modified-Goodman screening factor is "
        f"{torsion['stress_fatigue_screen']['modified_goodman_screening_factor']:.2f}; "
        "this is not an endurance qualification.",
        "",
        f"First-order rate tolerance is approximately "
        f"{100*torsion['tolerance_screen']['estimated_rate_RSS_fraction']:.1f}% RSS "
        f"or {100*torsion['tolerance_screen']['estimated_rate_worst_case_fraction']:.1f}% "
        "worst-case. Use an indexed anchor, proof-load, test and matched pairs.",
        "",
        "## Etched 17-7PH plate",
        "",
        f"Two fixed-guided leaves, each {etched['geometry_each_leaf']['length_mm']:.1f} x "
        f"{etched['geometry_each_leaf']['width_mm']:.3f} x "
        f"{etched['geometry_each_leaf']['thickness_mm']:.2f} mm, provide "
        f"{etched['force_rate_screen']['combined_rate_N_per_mm']:.3f} N/mm combined. "
        f"Estimated hard-travel stress is "
        f"{etched['stress_fatigue_screen']['maximum_stress_MPa']:.1f} MPa, with "
        f"static/fatigue screening factors of "
        f"{etched['stress_fatigue_screen']['static_yield_screening_factor']:.2f}/"
        f"{etched['stress_fatigue_screen']['zero_mean_fatigue_screening_factor']:.2f}.",
        "",
        "Photochemical etching is the credible route. Laser-cut 0.10 mm full-hard "
        "spring steel remains prototype-only because the fatigue-critical edges "
        "would require finishing and coupon qualification.",
        "",
        "## Printed Nylon 12 prototype",
        "",
        f"Two 11 x 1 x {nylon['geometry_each_leaf']['thickness_mm']:.2f} mm leaves "
        f"give {nylon['force_rate_screen']['nominal_combined_rate_N_per_mm']:.3f} "
        "N/mm nominal. With the published conditioned modulus range and a +/-0.05 "
        "mm thickness screen, combined rate spans "
        f"{nylon['force_rate_screen']['combined_rate_range_for_E_and_thickness_screen_N_per_mm'][0]:.3f}–"
        f"{nylon['force_rate_screen']['combined_rate_range_for_E_and_thickness_screen_N_per_mm'][1]:.3f} "
        "N/mm. It is tangential replaceable-prototype-only and is rejected for the "
        "continuously biased radial return.",
        "",
        "## Reduced 9293K122 radial cartridge",
        "",
        f"Calibrate output to {cartridge['force_budget']['calibrated_target_N']:.3f} +/- "
        f"{cartridge['force_budget']['calibrated_tolerance_N']:.3f} N. Required "
        "work ratios across catalog force tolerance are "
        f"{min(cartridge['reduction_mechanism']['required_ratios'].values()):.3f}–"
        f"{max(cartridge['reduction_mechanism']['required_ratios'].values()):.3f}; "
        "provide 0.235–0.315 adjustment.",
        "",
        f"The proposed negative-Y envelope is X "
        f"{cartridge['package_screen']['proposed_fixed_cartridge_local_envelope']['x_mm']}, "
        f"Y {cartridge['package_screen']['proposed_fixed_cartridge_local_envelope']['y_mm']}, "
        f"Z {cartridge['package_screen']['proposed_fixed_cartridge_local_envelope']['z_mm']} mm. "
        "It is analytically plausible, not CAD swept. Initial take-up is approximately "
        f"{cartridge['package_screen']['initial_takeup_for_full_load_mm'][0]:.1f}–"
        f"{cartridge['package_screen']['initial_takeup_for_full_load_mm'][1]:.1f} mm.",
        "",
        "## Rejected concepts",
        "",
    ]
    for row in report["rejected_concepts"]:
        lines.append(f"- **{row['concept']}** — `{row['status']}`: {row['reason']}")

    lines.extend(["", "## Source URLs", ""])
    for key, source in report["sources"].items():
        lines.append(f"- `{key}`: {source['url']}")

    lines.extend(["", "## Fail-closed authority", ""])
    for gate, value in report["fail_closed_gates"].items():
        lines.append(f"- `{gate}`: **{str(value).lower()}**")
    lines.extend(["", report["decision"], ""])
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
