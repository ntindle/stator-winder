"""Fail-closed load and wear audit for the replacement follower carriage.

This report converts the current hash-bound CAD and analytical contracts into
nominal section-stress screens and an explicit physical-qualification matrix.
It is deliberately not FEA, a supplier-rating substitute, or release evidence.
Where a strength, preload, fit, side load, duty spectrum, or process tolerance
is not bound by upstream evidence, the corresponding margin remains ``None``
and the qualification row fails closed.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
REVIEW = ROOT / "out" / "review"

OUTPUT_JSON = REPORTS / "aggregate_boundary_follower_replacement_load_wear.json"
OUTPUT_MD = REPORTS / "aggregate_boundary_follower_replacement_load_wear.md"

SCHEMA = "aggregate-boundary-follower-replacement-load-wear/v1"

SOURCE_PATHS = {
    "replacement_carriage_source": CAD / "aggregate_boundary_follower_replacement_carriage.py",
    "floating_follower_source": CAD / "aggregate_boundary_floating_follower.py",
    "replacement_manifest": REVIEW / "aggregate_boundary_follower_replacement_carriage_manifest.json",
    "return_packaging_manifest": REVIEW / "aggregate_boundary_follower_custom_return_packaging.json",
    "replacement_CAD_audit": REPORTS / "aggregate_boundary_follower_replacement_cad_audit.json",
    "replacement_architecture": REPORTS / "aggregate_boundary_follower_replacement_architecture.json",
    "mount_screen": REPORTS / "aggregate_boundary_follower_mount_screen.json",
    "hardware_qualification": REPORTS / "aggregate_boundary_follower_hardware_qualification.json",
    "retraction_topology": REPORTS / "aggregate_boundary_follower_retraction_topology.json",
    "retraction_procurement": REPORTS / "aggregate_boundary_follower_retraction_procurement.json",
    "custom_return_screen": REPORTS / "aggregate_boundary_follower_custom_return_screen.json",
}

AUTHORITY_KEYS = (
    "load_authorized",
    "fatigue_authorized",
    "wear_authorized",
    "retention_authorized",
    "tolerance_authorized",
    "assembly_integration_authorized",
    "production_authorized",
    "procurement_authorized",
    "BOM_change_authorized",
    "order_authorized",
    "release_authorized",
)

# Exact source-feature dimensions used by the current bound Python BREP.
# These are geometry facts only, not tolerance or material-property authority.
OUTER_PIN_DIAMETER_MM = 5.0
INNER_PIN_DIAMETER_MM = 3.0
PIVOT_SUPPORT_CENTER_SPAN_MM = 7.0
OUTER_YOKE_ARM_THICKNESS_MM = 2.0
INNER_HUB_LENGTH_MM = 4.0
INNER_YOKE_ARM_THICKNESS_MM = 2.0
PEEK_NOSE_WIDTH_MM = 4.0
PEEK_CONTACT_RADIUS_MM = 3.0
PEEK_BORE_DIAMETER_MM = 3.2
PEEK_OPEN_GROOVE_WIDTH_MM = 0.65

M4_DIAMETER_MM = 4.0
M4_PITCH_MM = 0.7
M4_HEAD_DIAMETER_MM = 6.0
M4_CLEARANCE_DIAMETER_MM = 4.5
M4_INSERT_OD_MM = 6.3
M4_INSERT_LENGTH_MM = 4.7
M4_AXES_XY_MM = (
    (29.0, -24.5),
    (35.0, -17.5),
    (29.0, 24.5),
    (35.0, 17.5),
)

PSI_TO_MPA = 0.006894757293168
FPM_TO_M_PER_S = 0.00508

NBK_SKU_URL = (
    "https://www.nbk1560.com/en-US/products/specialscrew/nedzicom/"
    "miniaturescrew/SSHS-SD-ALK/SSHS-M4-SD-ALK/SSHS-M4-10-SD-ALK/"
)
NBK_SMALL_HEAD_ENGINEERING_URL = (
    "https://www.nbk1560.com/en/products/specialscrew/nedzicom/"
    "lowsmallheadscrew/SSH-SD/SSH-M4-SD/SSH-M4-10-SD/"
)
NBK_A2_50_REFERENCE_URL = (
    "https://www.nbk1560.com/en-US/products/specialscrew/nedzicom/"
    "stainlessscrew/SNSX-109/SNSX-M6-109/"
)
IGUS_W300_MATERIAL_URL = (
    "https://www.igus.com/plastic-bearings/resources/iglide-material-l280"
)
IGUS_WPFFM_PART_URL = (
    "https://www.igus.com/iglide-ibh/flange-bearings/product-details/"
    "iglide-w300pf-m"
)
MISUMI_NETWS_CATALOG_URL = (
    "https://in.misumi-ec.com/vona2/detail/110300258420/?HissuCode=NETWS4"
)


def _round(value: float) -> float:
    return round(float(value), 9)


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(value: Mapping[str, Any]) -> str:
    body = deepcopy(dict(value))
    body.pop("report_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _source_record_hash(value: Mapping[str, Any]) -> str:
    body = deepcopy(dict(value))
    body.pop("source_record_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object at {path}")
    internal = value.get("report_sha256")
    if internal is not None and internal != _canonical_hash(value):
        raise ValueError(f"upstream report hash mismatch at {path}")
    return value


def _source_bindings() -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for name, path in SOURCE_PATHS.items():
        value = _load(path) if path.suffix.lower() == ".json" else None
        result[name] = {
            "path": str(path.relative_to(ROOT)).replace("\\", "/"),
            "sha256": _sha256(path),
            "byte_count": path.stat().st_size,
            "internal_report_sha256": (
                value.get("report_sha256") if value is not None else None
            ),
        }
    return result


def _supplier_source_records() -> dict[str, dict[str, Any]]:
    """Official supplier facts used only for bounded analytical screens."""

    records: dict[str, dict[str, Any]] = {
        "NBK_SSHS_M4_10_SD_ALK": {
            "supplier": "NBK America LLC",
            "url": NBK_SKU_URL,
            "retrieved_local_date": "2026-07-12",
            "facts": {
                "part_number": "SSHS-M4-10-SD-ALK",
                "thread": "M4x0.7",
                "length_mm": 10.0,
                "head_diameter_mm": 6.0,
                "head_height_mm": 1.5,
                "maximum_tightening_torque_Nm": 1.0,
                "body_material": "SUSXM7 equivalent to SUS304",
                "strength_class": "A2-50",
                "patch_material": "Nylon 11",
                "patch_heat_resistant_temperature_C": 120.0,
            },
            "authority_boundary": (
                "SKU dimensions/class/max torque only; selected torque, "
                "preload, bearing pressure and joint behavior are not supplied"
            ),
        },
        "NBK_small_head_engineering_table": {
            "supplier": "NBK",
            "url": NBK_SMALL_HEAD_ENGINEERING_URL,
            "retrieved_local_date": "2026-07-12",
            "facts": {
                "M4_head_diameter_mm": 6.0,
                "M4_effective_cross_sectional_area_mm2": 8.78,
                "small_head_bearing_pressure_caution": True,
            },
            "authority_boundary": (
                "same NBK small-head geometry family; tightening bearing "
                "pressure still requires the actual preload and carrier allowable"
            ),
        },
        "NBK_A2_50_reference_properties": {
            "supplier": "NBK",
            "url": NBK_A2_50_REFERENCE_URL,
            "retrieved_local_date": "2026-07-12",
            "facts": {
                "strength_class": "A2-50",
                "reference_tensile_strength_MPa": 500.0,
                "reference_0p2_proof_stress_MPa": 210.0,
                "values_guaranteed_for_this_received_lot": False,
            },
            "authority_boundary": (
                "NBK labels the class values reference-only and not guaranteed; "
                "the screen is not a received-lot certificate"
            ),
        },
        "igus_W300_material": {
            "supplier": "igus",
            "url": IGUS_W300_MATERIAL_URL,
            "retrieved_local_date": "2026-07-12",
            "facts": {
                "material": "iglide W300 (previously L280)",
                "permissible_static_surface_pressure_psi_at_68F": 8702.0,
                "maximum_dry_PV_psi_fpm": 6600.0,
                "dynamic_friction_against_steel_range": [0.08, 0.23],
                "maximum_linear_surface_speed_fpm": 984.0,
                "maximum_rotating_surface_speed_fpm": 295.0,
                "long_term_maximum_temperature_F": 194.0,
                "short_term_maximum_temperature_F": 356.0,
                "minimum_temperature_F": -40.0,
                "PV_table_conditions": (
                    "dry, steel shaft, 68 F, 1 mm wall, steel housing"
                ),
                "preferred_shaft_finish_rms_microinch": [16.0, 20.0],
            },
            "authority_boundary": (
                "material-level screen only; current bushing wall is 0.75 mm "
                "and the final housing/temperature/speed/duty are not bound"
            ),
        },
        "igus_WPFFM_0304_05": {
            "supplier": "igus",
            "url": IGUS_WPFFM_PART_URL,
            "retrieved_local_date": "2026-07-12",
            "facts": {
                "part_number": "WPFFM-0304-05",
                "material": "iglide W300PF",
                "d1_ID_mm": 3.0,
                "d2_body_OD_mm": 4.5,
                "d3_flange_OD_mm": 7.5,
                "b1_body_length_mm": 5.0,
                "b2_flange_thickness_mm": 0.75,
            },
            "authority_boundary": (
                "catalog dimensions only; installed ID and housing tolerance "
                "remain unbound in the current packaging model"
            ),
        },
        "MISUMI_NETWS": {
            "supplier": "MISUMI",
            "url": MISUMI_NETWS_CATALOG_URL,
            "retrieved_local_date": "2026-07-12",
            "facts": {
                "part_number_used": "NETWS4",
                "material": "304 stainless steel equivalent (SUS304-CSP)",
                "hardness_HRC_range": [37.0, 46.0],
                "per_part_axial_thrust_rating_available": False,
            },
            "authority_boundary": (
                "material/hardness only; no NETWS4 groove/axial retention "
                "rating was found, so retention authority remains false"
            ),
        },
    }
    for record in records.values():
        record["source_record_sha256"] = _source_record_hash(record)
    return records


def _require_current_geometry_chain(replacement: Mapping[str, Any]) -> None:
    """Refuse to bind a mixed old-STEP/new-source carrier redesign state."""

    current_paths = {
        "cad_source": SOURCE_PATHS["replacement_carriage_source"],
        "manifest": SOURCE_PATHS["replacement_manifest"],
        "step": REVIEW / "aggregate_boundary_follower_replacement_carriage.step",
        "architecture_report": SOURCE_PATHS["replacement_architecture"],
    }
    stale: list[str] = []
    for name, path in current_paths.items():
        evidence = replacement.get("artifact_binding", {}).get(name, {})
        current_hash = _sha256(path) if path.is_file() else None
        if (
            not path.is_file()
            or evidence.get("matches_inspected_sha256") is not True
            or evidence.get("sha256") != current_hash
            or evidence.get("expected_sha256") != current_hash
            or evidence.get("byte_count") != path.stat().st_size
        ):
            stale.append(name)
    if stale:
        raise ValueError(
            "replacement geometry chain is not current; regenerate final "
            "STEP/manifest/architecture/CAD audit before load binding: "
            + ", ".join(stale)
        )


def _double_shear_pin_screen(force_n: float, diameter_mm: float) -> dict[str, Any]:
    area = math.pi * diameter_mm ** 2 / 4.0
    section_modulus = math.pi * diameter_mm ** 3 / 32.0
    bending_moment = force_n * PIVOT_SUPPORT_CENTER_SPAN_MM / 4.0
    shear = force_n / (2.0 * area)
    bending = bending_moment / section_modulus
    von_mises = math.sqrt(bending ** 2 + 3.0 * shear ** 2)
    return {
        "model": "central point load; simply supported span; double shear",
        "force_N": _round(force_n),
        "diameter_mm": diameter_mm,
        "support_center_span_mm": PIVOT_SUPPORT_CENTER_SPAN_MM,
        "gross_double_shear_stress_MPa": _round(shear),
        "gross_bending_moment_Nmm": _round(bending_moment),
        "gross_bending_stress_MPa": _round(bending),
        "gross_von_Mises_proxy_MPa": _round(von_mises),
        "material_allowable_MPa": None,
        "margin_to_allowable": None,
        "warning": "gross shoulder only; fit, clearance, groove, thread, side load, impact and fatigue omitted",
    }


def _mount_screen(moment_nmm: float, direct_force_n: float) -> dict[str, Any]:
    cx = sum(x for x, _y in M4_AXES_XY_MM) / len(M4_AXES_XY_MM)
    cy = sum(y for _x, y in M4_AXES_XY_MM) / len(M4_AXES_XY_MM)
    centered = [(x - cx, y - cy) for x, y in M4_AXES_XY_MM]
    sum_x2 = sum(x * x for x, _y in centered)
    sum_y2 = sum(y * y for _x, y in centered)
    polar = sum(x * x + y * y for x, y in centered)

    radial_axial = [abs(moment_nmm * x / sum_x2) for x, _y in centered]
    tangential_axial = [abs(moment_nmm * y / sum_y2) for _x, y in centered]
    arbitrary_axial = [
        moment_nmm * math.hypot(x / sum_x2, y / sum_y2)
        for x, y in centered
    ]

    # The old exact mount screen also binds a 1 mm in-plane offset, hence a
    # 40 Nmm torsional moment for the pure tangential case.
    torsion_nmm = 40.0
    direct_each = direct_force_n / 4.0
    shear_resultants = []
    for x, y in centered:
        sx = -torsion_nmm * y / polar
        sy = direct_each + torsion_nmm * x / polar
        shear_resultants.append(math.hypot(sx, sy))

    worst_tension = max(arbitrary_axial)
    worst_shear = max(shear_resultants)
    formula_tensile_area = math.pi / 4.0 * (
        M4_DIAMETER_MM - 0.9382 * M4_PITCH_MM
    ) ** 2
    tensile_area = 8.78  # NBK official M4 effective cross-sectional area.
    shank_area = math.pi * M4_DIAMETER_MM ** 2 / 4.0
    head_annulus = math.pi / 4.0 * (
        M4_HEAD_DIAMETER_MM ** 2 - M4_CLEARANCE_DIAMETER_MM ** 2
    )
    insert_interface = math.pi * M4_INSERT_OD_MM * M4_INSERT_LENGTH_MM
    screw_tension = worst_tension / tensile_area
    screw_shear = worst_shear / shank_area

    return {
        "model": "ideal rigid bolt group; no preload, prying, joint separation, key sharing or face contact",
        "axes_local_XY_mm": [list(row) for row in M4_AXES_XY_MM],
        "centroid_local_XY_mm": [_round(cx), _round(cy)],
        "sum_x_squared_mm2": _round(sum_x2),
        "sum_y_squared_mm2": _round(sum_y2),
        "polar_sum_r_squared_mm2": _round(polar),
        "proof_force_N": _round(direct_force_n),
        "proof_moment_Nmm": _round(moment_nmm),
        "pure_radial_max_ideal_axial_reaction_per_screw_N": _round(max(radial_axial)),
        "pure_tangential_max_ideal_axial_reaction_per_screw_N": _round(max(tangential_axial)),
        "arbitrary_in_plane_direction_max_ideal_axial_reaction_per_screw_N": _round(worst_tension),
        "pure_tangential_max_ideal_shear_resultant_per_screw_N": _round(worst_shear),
        "M4_formula_tensile_stress_area_mm2": _round(formula_tensile_area),
        "M4_nominal_tensile_stress_area_mm2": _round(tensile_area),
        "M4_area_basis": "NBK official small-head M4 effective cross-sectional area",
        "M4_nominal_external_tensile_stress_MPa": _round(screw_tension),
        "M4_nominal_external_shear_stress_MPa": _round(screw_shear),
        "M4_nominal_external_von_Mises_proxy_MPa": _round(
            math.sqrt(screw_tension ** 2 + 3.0 * screw_shear ** 2)
        ),
        "small_head_nominal_annular_bearing_area_mm2": _round(head_annulus),
        "small_head_nominal_annular_bearing_pressure_MPa": _round(
            worst_tension / head_annulus
        ),
        "insert_nominal_cylindrical_interface_area_mm2": _round(insert_interface),
        "insert_mean_interface_pull_stress_proxy_MPa": _round(
            worst_tension / insert_interface
        ),
        "NBK_strength_class": "A2-50",
        "NBK_reference_0p2_proof_stress_MPa": 210.0,
        "NBK_reference_tensile_strength_MPa": 500.0,
        "NBK_reference_values_guaranteed_for_received_lot": False,
        "NBK_reference_0p2_proof_load_N": _round(210.0 * tensile_area),
        "external_axial_reaction_to_reference_proof_load_margin_N": _round(
            210.0 * tensile_area - worst_tension
        ),
        "nominal_external_von_Mises_to_reference_proof_factor": _round(
            210.0 / math.sqrt(screw_tension ** 2 + 3.0 * screw_shear ** 2)
        ),
        "nominal_external_tensile_to_reference_ultimate_factor": _round(
            500.0 / screw_tension
        ),
        "NBK_maximum_tightening_torque_Nm": 1.0,
        "selected_assembly_torque_Nm": None,
        "preload_from_torque_N": None,
        "bearing_pressure_from_tightening_MPa": None,
        "supplier_small_head_bearing_pressure_caution": True,
        "screw_received_lot_allowable_MPa": None,
        "head_carrier_bearing_allowable_MPa": None,
        "printed_tower_insert_pull_allowable_MPa": None,
        "joint_preload_and_separation_margin": None,
        "insert_pull_margin": None,
        "installation_torque_margin": None,
    }


def analyze() -> dict[str, Any]:
    replacement = _load(SOURCE_PATHS["replacement_CAD_audit"])
    _require_current_geometry_chain(replacement)
    architecture = _load(SOURCE_PATHS["replacement_architecture"])
    mount = _load(SOURCE_PATHS["mount_screen"])
    hardware = _load(SOURCE_PATHS["hardware_qualification"])
    topology = _load(SOURCE_PATHS["retraction_topology"])
    procurement = _load(SOURCE_PATHS["retraction_procurement"])
    returns = _load(SOURCE_PATHS["custom_return_screen"])
    replacement_manifest = _load(SOURCE_PATHS["replacement_manifest"])
    packaging = _load(SOURCE_PATHS["return_packaging_manifest"])

    proof_force = float(hardware["load_contract"]["required_proof_load_N"])
    proof_moment = float(
        mount["load_cases"]["radial_X_40N"]["moment_about_Y_Nmm"]
    )
    radial = topology["radial"]["LEM050AB01"]
    lem_max = float(max(
        row["LEM_follower_force_N"] for row in radial["force_rows"]
    ))
    topology_combined = float(radial["maximum_combined_inward_contact_force_N"])
    cartridge = returns["reduced_9293K122_cartridge"]
    independent_low, independent_high = map(
        float, cartridge["force_budget"]["calibrated_acceptance_range_N"]
    )
    candidate_combined_low = lem_max + independent_low
    candidate_combined_high = lem_max + independent_high
    force_cap = float(radial["contact_force_hard_cap_N"])
    local_superposition = proof_force + candidate_combined_high

    torsion = returns["torsion_wire_pair"]
    tangential_hard = float(
        torsion["force_rate_screen"]["net_restoring_force_at_hard_travel_N"]
    )
    tangential_single_spring = float(max(
        torsion["force_rate_screen"]["individual_force_range_N"]
    ))

    outer_pin = _double_shear_pin_screen(
        local_superposition, OUTER_PIN_DIAMETER_MM,
    )
    inner_pin = _double_shear_pin_screen(
        local_superposition, INNER_PIN_DIAMETER_MM,
    )
    mount_result = _mount_screen(proof_moment, proof_force)

    outer_yoke_bearing = local_superposition / (
        2.0 * OUTER_PIN_DIAMETER_MM * OUTER_YOKE_ARM_THICKNESS_MM
    )
    inner_hub_bearing = local_superposition / (
        OUTER_PIN_DIAMETER_MM * INNER_HUB_LENGTH_MM
    )
    inner_yoke_bearing = local_superposition / (
        2.0 * INNER_PIN_DIAMETER_MM * INNER_YOKE_ARM_THICKNESS_MM
    )
    peek_bore_bearing = local_superposition / (
        INNER_PIN_DIAMETER_MM * PEEK_NOSE_WIDTH_MM
    )
    peek_ligament = PEEK_CONTACT_RADIUS_MM - PEEK_BORE_DIAMETER_MM / 2.0
    peek_ligament_proxy = local_superposition / (
        peek_ligament * PEEK_NOSE_WIDTH_MM
    )

    shaft = procurement["shaft"]["selected_purchase_candidate"]
    bushing = procurement["bushing"]["selected_candidate"]
    shaft_d = float(shaft["diameter_mm"])
    shaft_span = float(packaging["shaft"]["length_mm"])
    bushing_length = float(bushing["bearing_length_b1_mm"])
    shaft_moment = local_superposition * shaft_span / 4.0
    shaft_bending = shaft_moment / (math.pi * shaft_d ** 3 / 32.0)
    bushing_pressure = local_superposition / (shaft_d * bushing_length)
    igus_static_limit = 8702.0 * PSI_TO_MPA
    igus_dry_pv_limit = 6600.0 * PSI_TO_MPA * FPM_TO_M_PER_S
    igus_static_factor = igus_static_limit / bushing_pressure
    igus_pv_limited_speed = igus_dry_pv_limit / bushing_pressure
    supplier_sources = _supplier_source_records()

    analytical_gates = {
        "replacement_static_CAD_hash_bound": replacement["static_CAD_geometry_proven"] is True,
        "replacement_architecture_remains_fail_closed": architecture["status"] == "DESIGN_CONTRACT_ONLY_FAIL_CLOSED",
        "governing_40N_and_5520Nmm_case_bound": math.isclose(proof_force, 40.0) and math.isclose(proof_moment, 5520.0),
        "candidate_high_side_combined_bias_below_2N": candidate_combined_high < force_cap,
        "diagonal_M4_pattern_matches_final_CAD": tuple(
            (float(row[0]), float(row[1]))
            for row in replacement_manifest["carrier"]["diagonal_primary_M4_local_locations_mm"]
        ) == M4_AXES_XY_MM,
        "outer_SCCG5_10_geometry_matches_CAD": (
            float(replacement_manifest["occurrences"]["outer_pivot_catalog_stack"]["pin_diameter_mm"])
            == OUTER_PIN_DIAMETER_MM
        ),
        "OD3_shaft_and_WPFFM_geometry_match_packaging": (
            float(packaging["shaft"]["diameter_mm"]) == shaft_d == 3.0
            and float(packaging["igus_bushing_and_pocket"]["body_length_mm"])
            == bushing_length == 5.0
            and packaging["igus_bushing_and_pocket"]["catalog_number"]
            == bushing["catalog_number"] == "WPFFM-0304-05"
        ),
        "NBK_official_SKU_class_geometry_area_and_max_torque_bound": (
            supplier_sources["NBK_SSHS_M4_10_SD_ALK"]["facts"]
            ["strength_class"] == "A2-50"
            and supplier_sources["NBK_small_head_engineering_table"]
            ["facts"]["M4_effective_cross_sectional_area_mm2"] == 8.78
            and supplier_sources["NBK_SSHS_M4_10_SD_ALK"]["facts"]
            ["maximum_tightening_torque_Nm"] == 1.0
        ),
        "igus_official_W300_static_pressure_screen_passes": (
            bushing_pressure < igus_static_limit
            and supplier_sources["igus_WPFFM_0304_05"]["facts"]
            ["part_number"] == "WPFFM-0304-05"
        ),
        "MISUMI_NETWS_material_and_hardness_bound_without_retention_rating": (
            supplier_sources["MISUMI_NETWS"]["facts"]["hardness_HRC_range"]
            == [37.0, 46.0]
            and supplier_sources["MISUMI_NETWS"]["facts"]
            ["per_part_axial_thrust_rating_available"] is False
        ),
    }

    qualification_gates = {
        "NBK_selected_torque_preload_and_small_head_bearing_pressure_validated": False,
        "M4_joint_preload_prying_separation_and_key_share_validated": False,
        "printed_tower_insert_pull_torque_hot_creep_qualified": False,
        "carrier_U_window_static_and_fatigue_stress_validated": False,
        "SCCG5_10_material_groove_and_NETWS4_retention_ratings_bound": False,
        "DIN988_thrust_side_load_lubrication_and_wear_qualified": False,
        "inner_M2_shoulder_pin_grade_thread_nut_retention_qualified": False,
        "hard_anodize_spec_thickness_friction_debris_and_endurance_bound": False,
        "virgin_PEEK_grade_compression_creep_temperature_and_wire_wear_qualified": False,
        "shaft_cut_chamfer_hardness_and_bending_allowable_qualified": False,
        "igus_press_fit_installed_ID_pressure_PV_and_cycle_life_qualified": False,
        "torsion_pair_manufactured_metrologized_and_endurance_qualified": False,
        "radial_cartridge_output_full_stroke_hot_endurance_qualified": False,
        "positive_retraction_cam_and_dock_force_with_friction_inertia_bound": False,
        "full_40N_multidirectional_physical_proof_completed": False,
    }

    matrix = [
        {
            "id": "LOAD-01",
            "component": "radial passive-contact system",
            "analytical_result": {
                "topology_nominal_combined_N": _round(topology_combined),
                "custom_return_high_side_combined_N": _round(candidate_combined_high),
                "2N_cap_margin_N": _round(force_cap - candidate_combined_high),
            },
            "analytical_status": "PASS_SCREEN_ONLY",
            "qualification_status": "FAIL_UNBUILT_UNMEASURED",
            "missing": ["measured full-stroke force", "breakaway", "temperature drift", "life spectrum"],
        },
        {
            "id": "LOAD-02",
            "component": "M0 radial cam and tangential/gimbal docks",
            "analytical_result": {
                "tangential_pair_hard_restoring_force_N": _round(tangential_hard),
                "single_remaining_spring_max_static_force_N": _round(tangential_single_spring),
                "required_cam_or_dock_force_N": None,
            },
            "analytical_status": "FAIL_NO_FRICTION_INERTIA_OR_DRIVE_BOUND",
            "qualification_status": "FAIL",
            "missing": ["breakaway map", "cam friction", "acceleration/shock", "single-fault dock load"],
        },
        {
            "id": "STRUCT-01",
            "component": "SCCG5-10 outer pin, NETWS4 rings and DIN988 shims",
            "analytical_result": {
                **outer_pin,
                "NETWS4_material": "SUS304-CSP equivalent",
                "NETWS4_hardness_HRC_range": [37.0, 46.0],
                "NETWS4_supplier_axial_thrust_rating_N": None,
            },
            "analytical_status": "FAIL_NO_ALLOWABLE_OR_RETENTION_RATING",
            "qualification_status": "FAIL",
            "missing": ["pin material/heat treatment", "groove allowable", "NETWS4 ring axial rating", "side thrust", "shim wear"],
        },
        {
            "id": "STRUCT-02",
            "component": "OD3x10 M2 inner shoulder pivot and inner yoke",
            "analytical_result": {
                **inner_pin,
                "inner_6061_yoke_nominal_bearing_stress_MPa": _round(inner_yoke_bearing),
            },
            "analytical_status": "FAIL_NO_PIN_GRADE_FIT_OR_FATIGUE_ALLOWABLE",
            "qualification_status": "FAIL",
            "missing": ["shoulder pin grade", "M2 thread/nut retention", "hole tolerance", "impact/fatigue spectrum"],
        },
        {
            "id": "STRUCT-03",
            "component": "7075 outer yoke and 6061 inner hub",
            "analytical_result": {
                "7075_outer_yoke_nominal_bearing_stress_MPa": _round(outer_yoke_bearing),
                "6061_inner_hub_nominal_bearing_stress_MPa": _round(inner_hub_bearing),
                "section_and_fillet_stress_concentration": None,
                "margin_to_allowable": None,
            },
            "analytical_status": "FAIL_NO_BOUND_ALLOWABLE_OR_SECTION_SOLUTION",
            "qualification_status": "FAIL",
            "missing": ["material certificates", "hole/fit tolerances", "fillet stress", "fatigue and anodize knockdown"],
        },
        {
            "id": "WEAR-01",
            "component": "virgin unfilled PEEK R3 nose",
            "analytical_result": {
                "nominal_pin_bore_bearing_stress_MPa": _round(peek_bore_bearing),
                "minimum_radial_ligament_mm": _round(peek_ligament),
                "ligament_direct_stress_proxy_MPa": _round(peek_ligament_proxy),
                "wire_line_load_on_0p65mm_band_N_per_mm": _round(local_superposition / PEEK_OPEN_GROOVE_WIDTH_MM),
                "contact_pressure_or_Hertz_solution": None,
                "margin_to_allowable": None,
            },
            "analytical_status": "FAIL_NO_GRADE_CREEP_CONTACT_OR_WEAR_ALLOWABLE",
            "qualification_status": "FAIL",
            "missing": ["PEEK grade/certificate", "surface finish", "temperature", "creep", "0.2/0.5 mm enamel wear"],
        },
        {
            "id": "STRUCT-04",
            "component": "OD3 cut shaft and igus WPFFM-0304-05 bushing",
            "analytical_result": {
                "shaft_model": "simply supported 16 mm span; central perpendicular load",
                "shaft_max_bending_moment_Nmm": _round(shaft_moment),
                "shaft_gross_bending_stress_MPa": _round(shaft_bending),
                "shaft_hardness": shaft["hardness"],
                "shaft_surface_smoothness_microinch": shaft["surface_smoothness_microinch"],
                "shaft_finish_inside_igus_preferred_16_to_20_rms_screen": (
                    16.0 <= float(shaft["surface_smoothness_microinch"]) <= 20.0
                ),
                "shaft_yield_or_fatigue_allowable_MPa": None,
                "igus_nominal_projected_pressure_MPa": _round(bushing_pressure),
                "igus_W300_supplier_static_limit_MPa_at_20C": _round(igus_static_limit),
                "igus_static_pressure_screen_margin_MPa": _round(
                    igus_static_limit - bushing_pressure
                ),
                "igus_supplier_static_projected_load_screen_N": _round(
                    igus_static_limit * shaft_d * bushing_length
                ),
                "igus_static_pressure_screen_factor": _round(igus_static_factor),
                "igus_static_pressure_screen_pass": bushing_pressure < igus_static_limit,
                "igus_W300_supplier_dry_PV_limit_MPa_m_per_s": _round(igus_dry_pv_limit),
                "PV_limited_speed_ceiling_at_screen_pressure_m_per_s": _round(igus_pv_limited_speed),
                "supplier_max_linear_surface_speed_m_per_s": _round(984.0 * FPM_TO_M_PER_S),
                "supplier_max_rotating_surface_speed_m_per_s": _round(295.0 * FPM_TO_M_PER_S),
                "supplier_dynamic_friction_against_steel_range": [0.08, 0.23],
                "supplier_temperature_range_C": {
                    "minimum": -40.0,
                    "long_term_maximum": 90.0,
                    "short_term_maximum": 180.0,
                },
                "current_bushing_wall_thickness_mm": _round(
                    (float(bushing["body_OD_d2_mm"]) - shaft_d) / 2.0
                ),
                "supplier_PV_table_wall_thickness_mm": 1.0,
                "supplier_PV_table_housing": "steel",
                "actual_velocity_m_per_s": None,
                "actual_PV_MPa_m_per_s": None,
                "igus_dynamic_PV_margin": None,
                "igus_pressure_PV_and_wear_authority": False,
            },
            "analytical_status": "PASS_STATIC_PRESSURE_SCREEN_ONLY__FAIL_DYNAMIC_FIT_AND_LIFE",
            "qualification_status": "FAIL",
            "missing": ["shaft material strength", "post-cut hardness/chamfer", "installed ID", "housing tolerance", "actual velocity/duty/PV", "0.75mm-wall/nonsteel-housing supplier approval"],
        },
        {
            "id": "MOUNT-01",
            "component": "diagonal NBK M4 carrier-to-printed-tower joint",
            "analytical_result": mount_result,
            "analytical_status": "PASS_A2_50_EXTERNAL_STRESS_SCREEN_ONLY__FAIL_JOINT",
            "qualification_status": "FAIL",
            "missing": ["received-lot certificate", "selected assembly torque/preload", "small-head tightening bearing pressure", "joint separation/prying", "insert pull/torque", "PETG hot creep", "key/face load share"],
        },
        {
            "id": "WEAR-02",
            "component": "7075 hard-anodized slides and captured T-slot",
            "analytical_result": {
                "force_envelope_N": _round(local_superposition),
                "contact_area_pressure_friction_and_PV": None,
                "anodize_thickness_and_process": None,
            },
            "analytical_status": "FAIL_NO_FINISH_OR_CONTACT_DEFINITION",
            "qualification_status": "FAIL",
            "missing": ["anodize class/thickness", "counterface/liner", "lubrication", "particle/debris limit", "cycle spectrum"],
        },
        {
            "id": "SPRING-01",
            "component": "opposed 17-7PH tangential torsion pair",
            "analytical_result": {
                "stress_range_MPa": torsion["stress_fatigue_screen"]["stress_range_MPa"],
                "static_yield_screening_factor": torsion["stress_fatigue_screen"]["static_yield_screening_factor"],
                "modified_Goodman_screening_factor": torsion["stress_fatigue_screen"]["modified_goodman_screening_factor"],
                "screening_endurance_is_assumed": True,
            },
            "analytical_status": "PASS_ASSUMPTION_BASED_SCREEN_ONLY",
            "qualification_status": "FAIL_NOT_MANUFACTURED_OR_ENDURANCE_TESTED",
            "missing": ["lot material", "age treatment", "residual stress", "surface damage", "real duty spectrum"],
        },
        {
            "id": "CARRIER-01",
            "component": "one-piece 6061 U-window replacement carrier",
            "analytical_result": {
                "inherited_tower_proof_force_N": _round(proof_force),
                "inherited_tower_proof_moment_Nmm": _round(proof_moment),
                "full_carrier_stress_or_buckling_solution": None,
            },
            "analytical_status": "FAIL_NO_SECTION_OR_FEA_SOLUTION",
            "qualification_status": "FAIL",
            "missing": ["all load-path sections", "U-window stress concentrations", "machining tolerance", "fatigue", "physical proof"],
        },
    ]

    test_requirements = {
        "primary_mount_joint": {
            "configuration": "production-intent carrier, printed tower, inserts, keys, NBK screws, final torque and thermal conditioning",
            "NBK_supplier_maximum_tightening_torque_Nm": 1.0,
            "loads": [
                "40 N at the nose fixture with 138 mm inherited lever arm in radial +/- directions",
                "40 N tangential +/- and combined in-plane directions that exercise the 466.008962942 N ideal worst screw reaction",
            ],
            "measure": ["bolt preload", "joint opening/prying", "key/face strain", "insert motion", "permanent set"],
            "acceptance": "survive the bound 40 N / 5.52 N m proof case without joint separation, insert motion, thread damage or permanent deformation; numerical metrology limits and hold time remain to be specified",
            "unresolved_before_test": ["final torque at or below 1 N m", "preload method", "small-head bearing pressure", "temperature range", "measurement resolution", "hold duration"],
        },
        "single_insert_and_torque_coupons": {
            "configuration": "same PETG lot/profile/orientation, pilot, insertion tool and thermal history as the tower",
            "derived_external_axial_load_per_insert_N": _round(mount_result["arbitrary_in_plane_direction_max_ideal_axial_reaction_per_screw_N"]),
            "measure": ["pull load-displacement", "insert rotation", "stripping torque", "hot-creep retention after final assembly torque"],
            "acceptance": "must exceed the derived external reaction after final preload/torque is included; acceptance factor cannot be set until torque, temperature and life requirements are frozen",
        },
        "pivot_and_retention_fixture": {
            "configuration": "production SCCG5-10/NETWS4/DIN988 and OD3 inner-pin stacks in representative 7075/6061/PEEK bores",
            "transverse_structural_load_N": _round(local_superposition),
            "measure": ["pin deflection", "hole ovalization", "ring/groove motion", "shim wear", "retention after cycling"],
            "acceptance": "supplier-rated or physically proved transverse, axial-retention, impact and fatigue margins; axial side-thrust test load remains undefined until gimbal dynamics are measured",
        },
        "shaft_bushing_breakaway_and_PV": {
            "configuration": "5033N11 cut/chamfered to 16 mm, WPFFM-0304-05 installed in final pocket tolerance",
            "perpendicular_structural_load_N": _round(local_superposition),
            "breakaway_requirement_N": 0.125,
            "supplier_static_pressure_screen_factor": _round(igus_static_factor),
            "PV_limited_speed_ceiling_at_screen_pressure_m_per_s": _round(igus_pv_limited_speed),
            "measure": ["installed ID", "breakaway over temperature", "full-stroke force", "temperature", "wear/debris", "post-cut shaft hardness"],
            "acceptance": "breakaway <=0.125 N and supplier pressure/PV/life limits satisfied over the actual velocity and duty spectrum; the nominal static screen passes, but velocity and duty are not yet bound",
        },
        "PEEK_wire_and_anodize_wear": {
            "configuration": "production-intent virgin PEEK and anodize/liner with both 0.2 and 0.5 mm enamel wire at 10 N line tension",
            "follower_bias_high_N": _round(candidate_combined_high),
            "measure": ["enamel damage", "PEEK groove growth", "anodize loss", "friction/breakaway drift", "wear debris", "temperature"],
            "acceptance": "no functional or enamel damage over the project service spectrum; spectrum, wear limit and inspection method remain unbound",
        },
        "spring_and_retraction_endurance": {
            "configuration": "metrologized 17-7PH pair, calibrated 9293K122 cartridge, final cam/docks and actual-position interlock",
            "force_acceptance": [
                "independent radial output 0.266 to 0.286 N across 6.4 mm",
                "tangential net rate 0.30 N/mm and 0.18 N at +/-0.6 mm",
                "combined inward follower bias <=2.0 N",
            ],
            "measure": ["force/rate drift", "breakaway", "temperature", "spring fragments", "cam/dock force", "single-fault response"],
            "acceptance": "complete hot microcycle and service-life spectrum without fracture, force escape or unsafe deployment; required cycle count is not yet bound",
        },
    }

    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "FAIL",
        "decision": "NOMINAL_LOAD_SCREENS_COMPLETE__PHYSICAL_LOAD_RETENTION_FATIGUE_WEAR_AND_TOLERANCE_AUTHORITY_OPEN",
        "scope": {
            "geometry_and_inputs_hash_bound": True,
            "nominal_linear_section_screens_only": True,
            "FEA_or_certification": False,
            "supplier_rating_substitute": False,
        },
        "load_envelope": {
            "design_wire_tension_N": hardware["load_contract"]["design_wire_tension_N"],
            "unbound_static_wire_reaction_N": hardware["load_contract"]["unbound_static_reaction_N"],
            "proof_factor": hardware["load_contract"]["proof_factor"],
            "governing_structural_proof_force_N": _round(proof_force),
            "governing_primary_mount_moment_Nmm": _round(proof_moment),
            "LEM_max_follower_force_N": _round(lem_max),
            "topology_nominal_combined_bias_N": _round(topology_combined),
            "candidate_independent_return_range_N": [_round(independent_low), _round(independent_high)],
            "candidate_combined_bias_range_N": [_round(candidate_combined_low), _round(candidate_combined_high)],
            "candidate_high_side_margin_to_2N_N": _round(force_cap - candidate_combined_high),
            "conservative_local_structural_superposition_N": _round(local_superposition),
            "superposition_note": "40 N proof reaction plus high-side passive bias; used for local follower screens, not to alter the inherited 5.52 N m mount requirement",
            "positive_retraction_drive_load_bound": False,
        },
        "analytical_gates": analytical_gates,
        "qualification_gates": qualification_gates,
        "component_pass_fail_matrix": matrix,
        "physical_test_requirements": test_requirements,
        "authority": {key: False for key in AUTHORITY_KEYS},
        "source_bindings": _source_bindings(),
        "official_supplier_sources": supplier_sources,
        "official_supplier_source_policy": {
            "binding_mode": "OFFICIAL_URL_PLUS_CANONICAL_FACT_RECORD_SHA256",
            "live_page_bytes_archived": False,
            "warning": (
                "record hashes bind the transcribed facts in this report, not "
                "a frozen copy of mutable supplier web pages"
            ),
        },
        "open_blockers": [
            key for key, value in qualification_gates.items() if value is False
        ],
    }
    report["report_sha256"] = _canonical_hash(report)
    validate_report_integrity(report)
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported replacement load/wear schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("replacement load/wear report hash mismatch")
    if report.get("status") != "FAIL":
        raise ValueError("replacement load/wear audit must fail closed")
    if not all(report.get("analytical_gates", {}).values()):
        raise ValueError("replacement load/wear analytical input gate failed")
    if any(report.get("qualification_gates", {}).values()):
        raise ValueError("replacement load/wear audit invented qualification")
    authority = report.get("authority", {})
    if set(authority) != set(AUTHORITY_KEYS) or any(authority.values()):
        raise ValueError("replacement load/wear audit invented authority")
    matrix = report.get("component_pass_fail_matrix", [])
    if not matrix or any(row["qualification_status"] == "PASS" for row in matrix):
        raise ValueError("replacement load/wear matrix promoted an unqualified row")
    for binding in report.get("source_bindings", {}).values():
        path = ROOT / binding["path"]
        if (
            not path.is_file()
            or path.stat().st_size != binding["byte_count"]
            or _sha256(path) != binding["sha256"]
        ):
            raise ValueError(f"stale replacement load/wear source {path}")
        if binding["internal_report_sha256"] is not None:
            value = _load(path)
            if value.get("report_sha256") != binding["internal_report_sha256"]:
                raise ValueError(f"stale upstream internal hash {path}")
    sources = report.get("official_supplier_sources", {})
    if not sources or any(
        not value.get("url", "").startswith("https://")
        or value.get("source_record_sha256") != _source_record_hash(value)
        for value in sources.values()
    ):
        raise ValueError("invalid official supplier source record")


def render_markdown(report: Mapping[str, Any]) -> str:
    loads = report["load_envelope"]
    mount = next(
        row for row in report["component_pass_fail_matrix"]
        if row["id"] == "MOUNT-01"
    )["analytical_result"]
    bushing = next(
        row for row in report["component_pass_fail_matrix"]
        if row["id"] == "STRUCT-04"
    )["analytical_result"]
    lines = [
        "# Replacement follower load and wear qualification", "",
        f"**{report['status']} — {report['decision']}**", "",
        "Nominal geometry-based stress screens are complete. No supplier, physical, fatigue, wear, tolerance, procurement, integration, or release authority is granted.", "",
        "## Governing loads", "",
        f"- Bound wire proof: {loads['governing_structural_proof_force_N']:.3f} N and {loads['governing_primary_mount_moment_Nmm']:.3f} N mm.",
        f"- Candidate high-side passive bias: {loads['candidate_combined_bias_range_N'][1]:.9f} N; margin to the 2 N cap is only {loads['candidate_high_side_margin_to_2N_N']:.9f} N.",
        f"- Conservative local follower screen: {loads['conservative_local_structural_superposition_N']:.9f} N.",
        f"- Ideal diagonal M4 group: {mount['pure_radial_max_ideal_axial_reaction_per_screw_N']:.3f} N per screw for pure radial load and {mount['arbitrary_in_plane_direction_max_ideal_axial_reaction_per_screw_N']:.3f} N for the worst in-plane direction.", "",
        "## Official supplier screens", "",
        f"- NBK SSHS-M4-10-SD-ALK is A2-50 with an 8.78 mm2 effective area and 1 N m maximum tightening torque. External-load-only reference proof factor: {mount['nominal_external_von_Mises_to_reference_proof_factor']:.3f}. Preload and tightening bearing pressure remain uncalculated.",
        f"- igus W300 static-pressure screen: {bushing['igus_nominal_projected_pressure_MPa']:.3f} MPa versus {bushing['igus_W300_supplier_static_limit_MPa_at_20C']:.3f} MPa, factor {bushing['igus_static_pressure_screen_factor']:.3f}. At that pressure the dry PV ceiling implies {bushing['PV_limited_speed_ceiling_at_screen_pressure_m_per_s']:.6f} m/s, but actual speed/duty and the supplier test housing/wall assumptions remain open.",
        "- MISUMI NETWS4 material/hardness is bound as SUS304-CSP equivalent, 37-46 HRC. No part-specific axial thrust rating is bound.", "",
        "## Component matrix", "",
        "| ID | Component | Analytical status | Qualification |", "|---|---|---|---|",
    ]
    for row in report["component_pass_fail_matrix"]:
        lines.append(
            f"| {row['id']} | {row['component']} | {row['analytical_status']} | {row['qualification_status']} |"
        )
    lines.extend(["", "## Required physical evidence", ""])
    for name, value in report["physical_test_requirements"].items():
        lines.append(f"- **{name}** — {value['acceptance']}")
    lines.extend(["", "## Open qualification blockers", ""])
    lines.extend(f"- `{value}`" for value in report["open_blockers"])
    lines.extend(["", "## Official supplier sources", ""])
    lines.extend(
        f"- [{name}]({value['url']})"
        for name, value in report["official_supplier_sources"].items()
    )
    lines.extend(["", f"Report SHA-256: `{report['report_sha256']}`", ""])
    return "\n".join(lines)


def write_outputs(report: Mapping[str, Any] | None = None) -> dict[str, Any]:
    value = dict(report or analyze())
    validate_report_integrity(value)
    REPORTS.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(
        json.dumps(value, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    OUTPUT_MD.write_text(render_markdown(value), encoding="utf-8")
    return value


def main(argv: Iterable[str] | None = None) -> int:
    del argv
    report = write_outputs()
    loads = report["load_envelope"]
    print(
        "replacement load/wear: "
        f"status={report['status']}; "
        f"bias={loads['candidate_combined_bias_range_N'][1]:.9f}N; "
        f"local={loads['conservative_local_structural_superposition_N']:.9f}N; "
        f"blockers={len(report['open_blockers'])}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
