"""Procurement evidence for the isolated follower retraction topology.

The records here are a deterministic snapshot of the 2026-07-12 catalog
study.  They do not modify the BOM, order hardware, change CAD, or grant
assembly, controller, winding, or release authority.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORTS = ROOT / "out" / "reports"
OUTPUT_JSON = REPORTS / "aggregate_boundary_follower_retraction_procurement.json"
OUTPUT_MD = REPORTS / "aggregate_boundary_follower_retraction_procurement.md"

SCHEMA = "aggregate-boundary-follower-retraction-procurement/v1"
ACCESSED_DATE = "2026-07-12"
LBF_TO_N = 4.4482216152605


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(value: dict[str, Any]) -> str:
    body = dict(value)
    body.pop("report_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()

SHAFT_DIAMETER_REQUIRED_MM = 3.0
SHAFT_LENGTH_REQUIRED_MM = 16.0

BUSHING_ID_REQUIRED_MM = 3.0
BUSHING_BODY_OD_TARGET_MM = 5.0
BUSHING_LENGTH_TARGET_MM = 6.0

TANGENTIAL_SPRING_ID_MIN_MM = 3.2
TANGENTIAL_SPRING_OD_MAX_MM = 5.0
TANGENTIAL_SPRING_FREE_LENGTH_TARGET_MM = 5.5
TANGENTIAL_SPRING_INSTALLED_LENGTH_MM = 4.5
TANGENTIAL_SPRING_RATE_TARGET_N_PER_MM = 0.15
TANGENTIAL_USABLE_HALF_TRAVEL_MM = 0.5
TANGENTIAL_HARD_HALF_TRAVEL_MM = 0.6

LEM_HARD_EXTENDED_FORCE_N = 1.696709
CONTACT_FORCE_CAP_N = 2.0
INDEPENDENT_RETURN_TARGET_N = 0.25
RADIAL_HARD_TRAVEL_MM = 6.4


def _rounded(value: float) -> float:
    return round(float(value), 9)


def source_records() -> dict[str, dict[str, Any]]:
    """Return official/current source identities and the step.parts pre-pass."""

    return {
        "step_parts_D3x25_shaft": {
            "source": "step.parts canonical API/site",
            "accessed_date": ACCESSED_DATE,
            "catalog_last_modified": "2018-10-20",
            "url": (
                "https://www.step.parts/parts/"
                "precision_shaft_d03_l0025_chamfered"
            ),
            "record_id": "precision_shaft_d03_l0025_chamfered",
            "catalog_search_result": "exact diameter, 25 mm minimum result",
        },
        "mcmaster_5033N11": {
            "source": "McMaster-Carr official catalog",
            "accessed_date": ACCESSED_DATE,
            "url": "https://www.mcmaster.com/products/4316N171/",
            "catalog_number": "5033N11",
        },
        "igus_WPFFM_0304_05": {
            "source": "igus official product catalog",
            "accessed_date": ACCESSED_DATE,
            "url": (
                "https://www.igus.com/iglide-ibh/flange-bearings/"
                "product-details/iglide-w300pf-m"
            ),
            "catalog_number": "WPFFM-0304-05",
        },
        "century_S_1576CS": {
            "source": "Century Spring official product catalog",
            "accessed_date": ACCESSED_DATE,
            "url": "https://www.centuryspring.com/shop/s-1576cs",
            "catalog_number": "S-1576CS",
        },
        "century_B_50CS": {
            "source": "Century Spring official product catalog",
            "accessed_date": ACCESSED_DATE,
            "url": "https://www.centuryspring.com/shop/b-50cs",
            "catalog_number": "B-50CS",
        },
        "mcmaster_9293K122": {
            "source": "McMaster-Carr official constant-force spring catalog",
            "accessed_date": ACCESSED_DATE,
            "url": "https://www.mcmaster.com/products/constant-force-springs/",
            "catalog_number": "9293K122",
        },
        "omron_D4F": {
            "source": "Omron official lineup and datasheet",
            "accessed_date": ACCESSED_DATE,
            "url": "https://www.ia.omron.com/products/family/1327/lineup.html",
            "datasheet_url": (
                "https://www.ia.omron.com/data_pdf/cat/"
                "d4f_ds_e_5_7_csm1249.pdf"
            ),
            "catalog_number": "D4F-120-1R",
        },
    }


def shaft_selection() -> dict[str, Any]:
    purchased_length = 25.0
    cut_removed = purchased_length - SHAFT_LENGTH_REQUIRED_MM
    return {
        "status": "SELECTABLE_AFTER_SECONDARY_CUT",
        "requirement": {
            "diameter_mm": SHAFT_DIAMETER_REQUIRED_MM,
            "finished_length_mm": SHAFT_LENGTH_REQUIRED_MM,
            "ground_hardened_wear_surface": True,
        },
        "step_parts_precheck": {
            "record_id": "precision_shaft_d03_l0025_chamfered",
            "diameter_mm": 3.0,
            "length_mm": 25.0,
            "exact_diameter": True,
            "exact_length": False,
            "no_16mm_catalog_member_found": True,
        },
        "selected_purchase_candidate": {
            "supplier": "McMaster-Carr",
            "catalog_number": "5033N11",
            "diameter_mm": 3.0,
            "purchased_length_mm": purchased_length,
            "diameter_tolerance_mm": [-0.012, -0.004],
            "actual_diameter_range_mm": [2.988, 2.996],
            "hardness": "Rockwell C60",
            "surface_smoothness_microinch": 16,
            "source_key": "mcmaster_5033N11",
        },
        "fit_calculation": {
            "diameter_error_mm": 0.0,
            "length_excess_to_remove_mm": _rounded(cut_removed),
            "cut_to_finished_length_mm": SHAFT_LENGTH_REQUIRED_MM,
            "restore_end_chamfer_after_cut": True,
            "avoid_thermal_damage_to_hardened_surface": True,
        },
        "exact_drop_in_fit": False,
        "reason_not_drop_in": "stock length is 25 mm; 9 mm cut and chamfer required",
    }


def bushing_selection() -> dict[str, Any]:
    candidate = {
        "manufacturer": "igus",
        "catalog_number": "WPFFM-0304-05",
        "material": "iglide W300PF dry-running polymer",
        "shaft_ID_d1_mm": 3.0,
        "body_OD_d2_mm": 4.5,
        "flange_OD_d3_mm": 7.5,
        "bearing_length_b1_mm": 5.0,
        "flange_thickness_b2_mm": 0.75,
        "source_key": "igus_WPFFM_0304_05",
    }
    return {
        "status": "FUNCTIONAL_NEAR_MATCH_REQUIRES_POCKET_REDESIGN",
        "requirement": {
            "shaft_ID_mm": BUSHING_ID_REQUIRED_MM,
            "body_OD_target_mm": BUSHING_BODY_OD_TARGET_MM,
            "bearing_length_target_mm": BUSHING_LENGTH_TARGET_MM,
            "dry_running": True,
            "flanged": True,
        },
        "selected_candidate": candidate,
        "fit_calculation": {
            "nominal_shaft_ID_match": True,
            "body_OD_delta_from_target_mm": _rounded(
                candidate["body_OD_d2_mm"] - BUSHING_BODY_OD_TARGET_MM
            ),
            "bearing_length_delta_from_target_mm": _rounded(
                candidate["bearing_length_b1_mm"] - BUSHING_LENGTH_TARGET_MM
            ),
            "flange_overhang_each_side_mm": _rounded(
                (candidate["flange_OD_d3_mm"] - candidate["body_OD_d2_mm"])
                / 2.0
            ),
        },
        "required_CAD_changes": [
            "replace nominal OD5 pocket with igus-specified OD4.5 press-fit bore",
            "change supported bearing length from 6 mm to 5 mm",
            "provide 7.5 mm flange counterface and 0.75 mm axial flange space",
            "bind final housing tolerance to igus installed-ID guidance",
        ],
        "exact_drop_in_fit": False,
    }


def _opposed_spring_forces(
    *, free_length_mm: float, installed_length_mm: float,
    rate_n_per_mm: float,
) -> dict[str, float]:
    center_compression = free_length_mm - installed_length_mm
    center_preload = rate_n_per_mm * center_compression
    positive_hard_compression = center_compression + TANGENTIAL_HARD_HALF_TRAVEL_MM
    negative_hard_compression = center_compression - TANGENTIAL_HARD_HALF_TRAVEL_MM
    return {
        "center_compression_mm": _rounded(center_compression),
        "center_preload_each_N": _rounded(center_preload),
        "net_centering_stiffness_N_per_mm": _rounded(2.0 * rate_n_per_mm),
        "restoring_force_at_usable_limit_N": _rounded(
            2.0 * rate_n_per_mm * TANGENTIAL_USABLE_HALF_TRAVEL_MM
        ),
        "hard_stop_high_spring_force_N": _rounded(
            rate_n_per_mm * positive_hard_compression
        ),
        "hard_stop_low_spring_force_N": _rounded(
            rate_n_per_mm * negative_hard_compression
        ),
    }


def tangential_spring_selection() -> dict[str, Any]:
    s1576 = {
        "manufacturer": "Century Spring",
        "catalog_number": "S-1576CS",
        "free_length_mm": 5.59,
        "outside_diameter_mm": 3.18,
        "inside_diameter_mm": 2.77,
        "rate_N_per_mm": 0.11,
        "solid_length_mm": 1.52,
        "suggested_maximum_deflection_mm": 4.06,
        "source_key": "century_S_1576CS",
    }
    s1576["force_calculation"] = _opposed_spring_forces(
        free_length_mm=s1576["free_length_mm"],
        installed_length_mm=TANGENTIAL_SPRING_INSTALLED_LENGTH_MM,
        rate_n_per_mm=s1576["rate_N_per_mm"],
    )
    s1576["shaft_interference_nominal_mm"] = _rounded(
        SHAFT_DIAMETER_REQUIRED_MM - s1576["inside_diameter_mm"]
    )
    s1576["status"] = "REJECT_CURRENT_SHAFT_CLOSEST_FORCE_LENGTH"
    s1576["reject_reasons"] = [
        "2.77 mm spring ID cannot pass over the nominal 3 mm shaft",
        "0.22 N/mm opposed stiffness is below the 0.30 N/mm target",
        "requires separate <=2.7 mm pilot pins or reduced shaft diameter",
    ]

    b50 = {
        "manufacturer": "Century Spring",
        "catalog_number": "B-50CS",
        "free_length_mm": 33.27,
        "outside_diameter_mm": 4.78,
        "inside_diameter_mm": 4.11,
        "rate_N_per_mm": 0.14,
        "solid_length_mm": 3.81,
        "suggested_maximum_deflection_mm": 15.75,
        "source_key": "century_B_50CS",
    }
    b50_deflection = b50["free_length_mm"] - TANGENTIAL_SPRING_INSTALLED_LENGTH_MM
    b50["fit_calculation"] = {
        "deflection_at_4p5mm_install_mm": _rounded(b50_deflection),
        "deflection_beyond_suggested_maximum_mm": _rounded(
            b50_deflection - b50["suggested_maximum_deflection_mm"]
        ),
        "center_preload_each_N": _rounded(b50["rate_N_per_mm"] * b50_deflection),
        "clearance_above_solid_at_install_mm": _rounded(
            TANGENTIAL_SPRING_INSTALLED_LENGTH_MM - b50["solid_length_mm"]
        ),
    }
    b50["status"] = "REJECT_LENGTH_DEFLECTION_PRELOAD"
    b50["reject_reasons"] = [
        "33.27 mm free length cannot fit the 4.5 mm installed envelope",
        "required compression exceeds suggested maximum by 13.02 mm",
        "4.03 N preload per side is incompatible with the lightweight stage",
    ]

    return {
        "status": "NO_EXACT_STOCK_MATCH_CUSTOM_OR_REDESIGN_REQUIRED",
        "requirement": {
            "inside_diameter_minimum_mm": TANGENTIAL_SPRING_ID_MIN_MM,
            "outside_diameter_maximum_mm": TANGENTIAL_SPRING_OD_MAX_MM,
            "free_length_target_mm": TANGENTIAL_SPRING_FREE_LENGTH_TARGET_MM,
            "installed_center_length_mm": TANGENTIAL_SPRING_INSTALLED_LENGTH_MM,
            "rate_target_each_N_per_mm": TANGENTIAL_SPRING_RATE_TARGET_N_PER_MM,
            "opposed_net_stiffness_target_N_per_mm": 0.30,
        },
        "rejected_candidates": [s1576, b50],
        "step_parts_exact_match_found": False,
        "selected_stock_candidate": None,
        "custom_spring_required_for_current_geometry": True,
    }


def independent_return_selection() -> dict[str, Any]:
    force_reserve = CONTACT_FORCE_CAP_N - LEM_HARD_EXTENDED_FORCE_N
    growth_allowance = force_reserve - INDEPENDENT_RETURN_TARGET_N
    effective_rate_ceiling = growth_allowance / RADIAL_HARD_TRAVEL_MM
    stock_load_nominal = 0.23 * LBF_TO_N
    stock_load_minimum = (0.23 - 0.03) * LBF_TO_N
    return {
        "status": "NO_EXACT_STOCK_MATCH_CUSTOM_CONSTANT_FORCE_REQUIRED",
        "requirement": {
            "minimum_inward_return_force_N": INDEPENDENT_RETURN_TARGET_N,
            "hard_travel_mm": RADIAL_HARD_TRAVEL_MM,
            "combined_contact_force_cap_N": CONTACT_FORCE_CAP_N,
            "LEM_force_at_hard_extension_N": LEM_HARD_EXTENDED_FORCE_N,
        },
        "force_budget": {
            "remaining_force_at_hard_extension_N": _rounded(force_reserve),
            "allowed_force_growth_across_travel_N": _rounded(growth_allowance),
            "maximum_effective_rate_N_per_mm": _rounded(effective_rate_ceiling),
            "allowable_return_force_window_N": [
                INDEPENDENT_RETURN_TARGET_N,
                _rounded(force_reserve),
            ],
        },
        "rejected_stock_candidate": {
            "supplier": "McMaster-Carr",
            "catalog_number": "9293K122",
            "type": "25,000-cycle constant-force strip spring",
            "nominal_load_lbf": 0.23,
            "load_tolerance_lbf": [-0.03, 0.03],
            "nominal_load_N": _rounded(stock_load_nominal),
            "minimum_tolerance_load_N": _rounded(stock_load_minimum),
            "width_mm": _rounded(0.25 * 25.4),
            "coil_ID_mm": _rounded(0.53 * 25.4),
            "coil_OD_mm": _rounded(0.62 * 25.4),
            "extended_length_mm": _rounded(18.0 * 25.4),
            "source_key": "mcmaster_9293K122",
            "reject_reasons": [
                "minimum tolerance load exceeds the complete 0.303291 N reserve",
                "coil and strip envelope is much larger than the follower carrier",
            ],
        },
        "selected_stock_candidate": None,
        "custom_options_requiring_qualification": [
            "approximately 0.25 N constant-force strip spring",
            "qualified quasi-constant-force flexure",
            "revised LEM geometry with a new total force proof",
        ],
    }


def safety_switch_selection() -> dict[str, Any]:
    per_switch_operating_force = 5.0
    per_switch_direct_opening_force = 20.0
    count = 2
    return {
        "status": "EXACT_COMPONENT_REMOTE_ONLY_NOT_DIRECT_FOLLOWER_FIT",
        "requirement": {
            "independent_switch_bodies": 2,
            "contact": "positive-opening NC",
            "monitoring": "dual channel with discrepancy and EDM",
            "must_sense": "broad actual follower faces, not bellcrank flags",
        },
        "candidate": {
            "manufacturer": "Omron",
            "catalog_number": "D4F-120-1R",
            "quantity": count,
            "actuator": "metal roller lever with resin roller",
            "contact_form": "1NC/1NO slow action",
            "direct_opening": True,
            "cable_length_m": 1.0,
            "cable_exit": "horizontal",
            "body_envelope_mm": [18.0, 30.0],
            "approximate_overall_length_mm": 42.7,
            "mounting_centers_mm": 20.0,
            "cable_OD_mm": 8.3,
            "approximate_fixed_mass_each_g": 220.0,
            "operating_force_max_each_N": per_switch_operating_force,
            "direct_opening_force_min_each_N": per_switch_direct_opening_force,
            "source_key": "omron_D4F",
        },
        "force_calculation": {
            "two_switch_normal_operating_reaction_max_N": (
                count * per_switch_operating_force
            ),
            "two_switch_direct_opening_reaction_min_N": (
                count * per_switch_direct_opening_force
            ),
            "normal_reaction_multiple_of_2N_cap": _rounded(
                count * per_switch_operating_force / CONTACT_FORCE_CAP_N
            ),
            "direct_opening_multiple_of_2N_cap": _rounded(
                count * per_switch_direct_opening_force / CONTACT_FORCE_CAP_N
            ),
        },
        "direct_slide_actuation_feasible": False,
        "remote_only_conditions": [
            "switch bodies fixed outside the follower force path",
            "two independently captured M0-powered positive transfer mechanisms",
            "each transfer must still read an independent broad actual follower face",
            "broken transfer, welded contact and channel disagreement fault injection",
        ],
        "single_D4F_220_1R_2NC_rejected": (
            "two NC contacts share one mechanical switch body and do not provide "
            "independent mechanical channels"
        ),
        "selected_direct_fit_candidate": None,
    }


def build_report() -> dict[str, Any]:
    shaft = shaft_selection()
    bushing = bushing_selection()
    tangential = tangential_spring_selection()
    independent = independent_return_selection()
    switches = safety_switch_selection()
    evidence_gates = {
        "shaft_exact_diameter_and_cut_requirement_bound": (
            shaft["fit_calculation"]["length_excess_to_remove_mm"] == 9.0
        ),
        "igus_pocket_redesign_dimensions_bound": (
            bushing["selected_candidate"]["body_OD_d2_mm"] == 4.5
            and bushing["selected_candidate"]["flange_OD_d3_mm"] == 7.5
            and bushing["selected_candidate"]["bearing_length_b1_mm"] == 5.0
        ),
        "both_tangential_stock_candidates_rejected": all(
            row["status"].startswith("REJECT")
            for row in tangential["rejected_candidates"]
        ),
        "independent_return_rate_ceiling_below_0p009N_per_mm": (
            independent["force_budget"]["maximum_effective_rate_N_per_mm"]
            < 0.009
        ),
        "dual_D4F_direct_follower_actuation_rejected": (
            switches["direct_slide_actuation_feasible"] is False
        ),
    }
    fail_closed_gates = {
        "all_required_hardware_has_exact_drop_in_selection": False,
        "tangential_spring_selected_for_current_shaft": False,
        "independent_return_selected_within_force_ceiling": False,
        "dual_positive_opening_actual_position_switch_topology_integrated": False,
        "igus_press_fit_and_flange_pocket_integrated_in_CAD": False,
        "shaft_cut_chamfer_and_final_metrology_proven": False,
        "breakaway_force_and_spring_fatigue_qualified": False,
        "switch_transfer_single_fault_injection_complete": False,
    }
    report = {
        "schema": SCHEMA,
        "status": "PROCUREMENT_NO_GO_CUSTOM_HARDWARE_REQUIRED",
        "catalog_snapshot_date": ACCESSED_DATE,
        "requirements": {
            "shaft": "ground hardened diameter 3 x finished length 16 mm",
            "bushing": "dry-running flange ID3 / body OD about 5 / length about 6 mm",
            "tangential_springs": (
                "two opposed, ID>=3.2, OD<=5, free about 5.5 mm, "
                "rate about 0.15 N/mm each"
            ),
            "independent_return": "at least 0.25 N over 6.4 mm within 2 N total cap",
            "interlock": "dual independent positive-opening NC actual-position channels",
        },
        "shaft": shaft,
        "bushing": bushing,
        "tangential_springs": tangential,
        "independent_radial_return": independent,
        "dual_positive_opening_switches": switches,
        "sources": source_records(),
        "evidence_gates": evidence_gates,
        "fail_closed_gates": fail_closed_gates,
        "physical_procurement_authority": False,
        "BOM_change_authorized": False,
        "order_authorized": False,
        "CAD_change_authorized": False,
        "assembly_integration_authorized": False,
        "release_authorized": False,
        "source_bindings": {
            "sim/aggregate_boundary_follower_retraction_procurement.py": (
                _sha256(Path(__file__).resolve())
            ),
            "sim/aggregate_boundary_follower_retraction_topology.py": _sha256(
                HERE / "aggregate_boundary_follower_retraction_topology.py"
            ),
        },
        "decision": (
            "Purchase selection cannot close: the shaft needs a cut, the igus "
            "bushing needs a new pocket, both stock tangential springs are "
            "rejected, the independent return must be custom, and the D4F "
            "switches are remote-only. Do not add these lines to the BOM or order "
            "them as a production stack."
        ),
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def validate_report_integrity(report: dict[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported follower procurement schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("follower procurement report hash mismatch")
    for relative, expected in report.get("source_bindings", {}).items():
        path = ROOT / Path(str(relative).replace("\\", "/"))
        if not path.is_file() or _sha256(path) != expected:
            raise ValueError(f"stale follower procurement source {relative}")


def _markdown(report: dict[str, Any]) -> str:
    shaft = report["shaft"]
    bushing = report["bushing"]
    tangential = report["tangential_springs"]
    independent = report["independent_radial_return"]
    switches = report["dual_positive_opening_switches"]
    source = report["sources"]
    s1576, b50 = tangential["rejected_candidates"]
    lines = [
        "# Aggregate follower retraction procurement evidence",
        "",
        f"- Catalog snapshot: {report['catalog_snapshot_date']}",
        f"- Status: **{report['status']}**",
        "- BOM/order/CAD/assembly/release authority: **false**",
        "",
        "## Selection summary",
        "",
        "| Function | Candidate | Result |",
        "|---|---|---|",
        "| Ground Ø3 shaft | McMaster 5033N11 | Cut 25 mm stock to 16 mm |",
        "| Dry-running flange bushing | igus WPFFM-0304-05 | Pocket redesign |",
        "| Opposed centering springs | S-1576CS / B-50CS | Both rejected |",
        "| Independent 0.25 N return | McMaster 9293K122 screened | Rejected; custom required |",
        "| Dual positive-opening NC | 2x Omron D4F-120-1R | Remote-only |",
        "",
        "## Shaft",
        "",
        f"McMaster `5033N11` is Ø3 x 25 mm, Rockwell C60, with actual "
        f"diameter range {shaft['selected_purchase_candidate']['actual_diameter_range_mm'][0]:.3f}–"
        f"{shaft['selected_purchase_candidate']['actual_diameter_range_mm'][1]:.3f} mm. "
        f"Remove {shaft['fit_calculation']['length_excess_to_remove_mm']:.1f} mm and restore the chamfer.",
        "",
        f"Source: {source['mcmaster_5033N11']['url']}",
        "",
        "## Bushing",
        "",
        f"igus `WPFFM-0304-05`: ID {bushing['selected_candidate']['shaft_ID_d1_mm']:.1f}, "
        f"body OD {bushing['selected_candidate']['body_OD_d2_mm']:.1f}, flange OD "
        f"{bushing['selected_candidate']['flange_OD_d3_mm']:.1f}, bearing length "
        f"{bushing['selected_candidate']['bearing_length_b1_mm']:.1f}, flange thickness "
        f"{bushing['selected_candidate']['flange_thickness_b2_mm']:.2f} mm.",
        "",
        "The current OD5/L6 target must become an igus-toleranced OD4.5 press-fit "
        "pocket with 5 mm support and a 7.5 mm flange counterface.",
        "",
        f"Source: {source['igus_WPFFM_0304_05']['url']}",
        "",
        "## Rejected tangential springs",
        "",
        f"- `S-1576CS`: free {s1576['free_length_mm']:.2f}, ID "
        f"{s1576['inside_diameter_mm']:.2f}, OD {s1576['outside_diameter_mm']:.2f} mm, "
        f"rate {s1576['rate_N_per_mm']:.2f} N/mm. It interferes with Ø3 by "
        f"{s1576['shaft_interference_nominal_mm']:.2f} mm and yields only "
        f"{s1576['force_calculation']['net_centering_stiffness_N_per_mm']:.2f} N/mm opposed stiffness.",
        f"- `B-50CS`: free {b50['free_length_mm']:.2f}, ID "
        f"{b50['inside_diameter_mm']:.2f}, OD {b50['outside_diameter_mm']:.2f} mm, "
        f"rate {b50['rate_N_per_mm']:.2f} N/mm. A 4.5 mm installation needs "
        f"{b50['fit_calculation']['deflection_at_4p5mm_install_mm']:.2f} mm deflection "
        f"and {b50['fit_calculation']['center_preload_each_N']:.2f} N preload per side.",
        "",
        f"Sources: {source['century_S_1576CS']['url']} and "
        f"{source['century_B_50CS']['url']}",
        "",
        "## Independent return blocker",
        "",
        f"The LEM leaves {independent['force_budget']['remaining_force_at_hard_extension_N']:.6f} N "
        f"below the 2 N cap. Starting from 0.25 N, effective rate must be <= "
        f"{independent['force_budget']['maximum_effective_rate_N_per_mm']:.9f} N/mm.",
        "",
        f"McMaster `9293K122` is {independent['rejected_stock_candidate']['nominal_load_N']:.3f} N "
        f"nominal and {independent['rejected_stock_candidate']['minimum_tolerance_load_N']:.3f} N "
        "at its lowest tolerance, so it is rejected. A custom constant-force "
        "element or qualified flexure is required.",
        "",
        f"Source: {source['mcmaster_9293K122']['url']}",
        "",
        "## Remote-only positive-opening switches",
        "",
        "Two independent Omron `D4F-120-1R` bodies provide the required NC "
        "direct-opening contact type, but direct follower actuation is not viable.",
        "",
        f"Two switches require up to "
        f"{switches['force_calculation']['two_switch_normal_operating_reaction_max_N']:.0f} N "
        f"normal reaction and at least "
        f"{switches['force_calculation']['two_switch_direct_opening_reaction_min_N']:.0f} N "
        "for certified direct opening. They are acceptable only through two "
        "independent, positively captured, M0-powered remote transfers.",
        "",
        f"Source: {source['omron_D4F']['url']}",
        "",
        "## Fail-closed authority",
        "",
    ]
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
