"""Fail-closed motor-pulley shaft-retention decision for normal GOAL M2.

This analysis deliberately keeps three torque interfaces separate:

* the 3GT belt/tooth allowable transmission torque;
* the stock M2 clamp-bolt installation torque; and
* the pulley-to-motor-shaft slip/retention torque.

Only the third quantity can release the P30 on the Leadshine shaft.  NBK does
not publish that quantity for P30-3GT-BLP-6C-5, BNS, or BNW, so the lower
route-derived wire moment does not by itself authorize the stock clamp.

The shared-crown route report is intentionally optional while that audit is
being frozen.  Until its canonical locus hash and final M2 inertia exist, the
calculation uses the explicitly marked provisional pass-22/state-51 result and
keeps the final-load binding gate open.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parent.parent
REPORTS = ROOT / "out" / "reports"
OUTPUT_JSON = REPORTS / "m2_p30_stock_retention_decision.json"
OUTPUT_MD = REPORTS / "m2_p30_stock_retention_decision.md"

UPGRADES = ROOT / "cad" / "models" / "upgrades"
NBK_STEP = UPGRADES / "NBK_P30-3GT-BLP-6C-5_AP214.step"
NBK_SOURCE = UPGRADES / "NBK_P30-3GT-BLP-6C-5.source.json"
NBK_DRAWING = UPGRADES / "NBK_3GT-BLP-6C_official_drawing.pdf"
NBK_SLIP_CHART = UPGRADES / "NBK_aluminum_set_screw_slip_torque_reference.jpg"
LEADSHINE_STEP = UPGRADES / "CS-M21708.STEP"
LEADSHINE_DRAWING = UPGRADES / "Leadshine_CS-M21708_2D.pdf"

DRIVE_REPORT = REPORTS / "m2_normal_goal_drive_selection.json"
ROUTE_REPORT = REPORTS / "shared_annular_terminal_crown_audit.json"

EXPECTED_VENDOR_HASHES = {
    "cad/models/upgrades/NBK_P30-3GT-BLP-6C-5_AP214.step": (
        "996449b7d9ec7703e7b38c6f75eff00a1174e3e1f088c05f0f1460b205169df9"
    ),
    "cad/models/upgrades/NBK_3GT-BLP-6C_official_drawing.pdf": (
        "a6559b594f927fd3e7e4878ec341f6877f628e14cd9fc46a79cac7dc8e1dde87"
    ),
    "cad/models/upgrades/NBK_aluminum_set_screw_slip_torque_reference.jpg": (
        "70cae21dcd074ed35dc534fffafc9787f3a02f2c2236f408375447568250222e"
    ),
    "cad/models/upgrades/CS-M21708.STEP": (
        "7e995e724fc7e019278e0a919ba1db8c8abb3333f156c64eb6e62485e0f6662b"
    ),
    "cad/models/upgrades/Leadshine_CS-M21708_2D.pdf": (
        "b0edb4d9486562f2ead76c1363ac78ee4c20d3abaa3ad5a93eab60eb83a81141"
    ),
}

NBK_PRODUCT_URL = (
    "https://www.nbk1560.com/products/pulley/timingpulley/"
    "3GT-BLP-6C/P30-3GT-BLP-6C/"
)
NBK_DRAWING_URL = (
    "https://www.nbk1560.com/images/ja-JP/product/timingpulley/"
    "3GT-BLP-6C/3GT-BLP-6C_1.pdf"
)
NBK_SET_SCREW_SLIP_URL = (
    "https://www.nbk1560.com/en-US/resources/coupling/article/"
    "couplicon-set-screw-type-slip-torque/?SelectedLanguage=en-US"
)
NBK_D_SHAFT_GUIDANCE_URL = (
    "https://www.nbk1560.com/images/en-US/product/contents/"
    "toritsuke_coupling_NBK/toritsuke_coupling_NBK_1.pdf"
)
LEADSHINE_PRODUCT_URL = (
    "https://www.leadshine.com/product-detail/closed-loop-stepper-drive/"
    "closed-loop-stepper/CS-M21708.html"
)

WIRE_TENSION_N = 10.0
REQUIRED_RETENTION_MULTIPLE = 2.0
FRICTION_ALLOWANCE_NM = 0.020
ANGULAR_ACCELERATION_RAD_S2 = 200.0

# Provisional shared-crown result supplied by the route audit owner.  These
# values remain non-authoritative until ROUTE_REPORT binds the final canonical
# locus array and exact rotating guide inertia.
PROVISIONAL_ROUTE_MAX_ARM_MM = 19.670716623717
PROVISIONAL_ROUTE_WIRE_TORQUE_NM = 0.19670716623717
PROVISIONAL_ROUTE_ROW_HASH = (
    "ffdb0155e81f6b1db6d5fc6983eed14b0b157cb8dfa252bd76931a773e5fa0ab"
)
PROVISIONAL_GUIDE_INERTIA_KGM2 = 1.8485861791662887e-7

# NBK P30 belt-capacity values.  This is intentionally not used as a hub-slip
# rating.  P30 at 300 rpm: 2.06 N m base, KL=0.9 for a 210 mm belt, Km=1.0.
NBK_BELT_BASE_ALLOWABLE_300RPM_NM = 2.06
NBK_BELT_LENGTH_FACTOR_210MM = 0.9
NBK_BELT_MESH_FACTOR = 1.0


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _relative(path: Path) -> str:
    return str(path.relative_to(ROOT)).replace("\\", "/")


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _vendor_evidence() -> tuple[dict[str, str], dict[str, Any]]:
    paths = (NBK_STEP, NBK_DRAWING, NBK_SLIP_CHART, LEADSHINE_STEP, LEADSHINE_DRAWING)
    hashes = {_relative(path): _sha256(path) for path in paths}
    if hashes != EXPECTED_VENDOR_HASHES:
        raise RuntimeError("P30/Leadshine vendor evidence no longer matches reviewed hashes")

    source = _load_json(NBK_SOURCE)
    if source["manufacturer"] != "NBK (Nabeya Bi-tech Kaisha)":
        raise RuntimeError("unexpected NBK source authority")
    if source["part_number"] != "P30-3GT-BLP-6C-5":
        raise RuntimeError("unexpected NBK part number")
    if source["configuration"]["additional_machining"] is not None:
        raise RuntimeError("official STEP is no longer the unmachined stock configuration")
    if source["configuration"]["bnw_two_set_screw_machining_present"]:
        raise RuntimeError("official stock STEP unexpectedly claims BNW machining")
    return hashes, source


def _route_and_inertia_evidence(drive: dict[str, Any]) -> dict[str, Any]:
    current_base_j = float(
        drive["OD65_10N_full_inertia_torque"]["full_output_inertia_kgm2"]
    )
    provisional_full_j = current_base_j + PROVISIONAL_GUIDE_INERTIA_KGM2
    provisional = {
        "authority": "provisional_inter_agent_result__not_release_bound",
        "report_present": False,
        "report_path": _relative(ROUTE_REPORT),
        "report_sha256": None,
        "canonical_locus_sha256": None,
        "provisional_row_sha256": PROVISIONAL_ROUTE_ROW_HASH,
        "max_projected_moment_arm_mm": PROVISIONAL_ROUTE_MAX_ARM_MM,
        "wire_torque_at_10N_nm": PROVISIONAL_ROUTE_WIRE_TORQUE_NM,
        "rotating_flyer_PEEK_guide_izz_about_axis_kg_m2": (
            PROVISIONAL_GUIDE_INERTIA_KGM2
        ),
        "base_drive_inertia_from_current_drive_report_kg_m2": current_base_j,
        "final_full_output_inertia_kg_m2": provisional_full_j,
        "canonical_route_and_final_inertia_bound": False,
    }
    if not ROUTE_REPORT.exists():
        return provisional

    route = _load_json(ROUTE_REPORT)
    if route.get("schema") != "shared-annular-terminal-crown-audit/v1":
        return {
            **provisional,
            "authority": "route_report_present_but_schema_not_release_compatible",
            "report_present": True,
            "report_sha256": _sha256(ROUTE_REPORT),
        }

    try:
        m2 = route["coupled_loads"]["M2"]
        arm_mm = float(m2["max_projected_moment_arm_mm"])
        wire_torque_nm = float(m2["wire_torque_at_10N_nm"])
        guide_j = float(m2["rotating_flyer_PEEK_guide_izz_about_axis_kg_m2"])
        full_j = float(m2["final_full_output_inertia_kg_m2"])
        locus_sha = str(route["canonical_locus_array"]["sha256"])
    except (KeyError, TypeError, ValueError):
        return {
            **provisional,
            "authority": "route_report_present_but_required_release_keys_missing",
            "report_present": True,
            "report_sha256": _sha256(ROUTE_REPORT),
        }

    self_consistent = (
        len(locus_sha) == 64
        and all(char in "0123456789abcdef" for char in locus_sha.lower())
        and math.isclose(
            wire_torque_nm,
            WIRE_TENSION_N * arm_mm / 1000.0,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        )
        and guide_j >= 0.0
        and full_j >= guide_j
    )
    return {
        "authority": (
            "canonical_shared_crown_report"
            if self_consistent
            else "route_report_present_but_not_self_consistent"
        ),
        "report_present": True,
        "report_path": _relative(ROUTE_REPORT),
        "report_sha256": _sha256(ROUTE_REPORT),
        "canonical_locus_sha256": locus_sha,
        "provisional_row_sha256": PROVISIONAL_ROUTE_ROW_HASH,
        "max_projected_moment_arm_mm": arm_mm,
        "wire_torque_at_10N_nm": wire_torque_nm,
        "rotating_flyer_PEEK_guide_izz_about_axis_kg_m2": guide_j,
        "base_drive_inertia_from_current_drive_report_kg_m2": current_base_j,
        "final_full_output_inertia_kg_m2": full_j,
        "canonical_route_and_final_inertia_bound": self_consistent,
    }


def _option_rows(required_2x_nm: float) -> list[dict[str, Any]]:
    common_unknowns = {
        "published_P30_shaft_slip_rating_nm": None,
        "supplier_guaranteed_reversing_retention_nm": None,
        "proves_required_2x_retention": False,
        "production_release": False,
    }
    return [
        {
            "id": "stock_split_clamp",
            "purchase_configuration": "P30-3GT-BLP-6C-5 stock line item",
            "orderability": "stocked exact NBK part",
            "exact_official_STEP_available": True,
            "additional_machining": None,
            "retention_features": (
                "5 mm stock split-clamp bore; one SCM435 M2 clamp bolt at "
                "0.5 N m installation torque"
            ),
            "leadshine_D_shaft_interface": (
                "geometry is possible only with the flat clear of the split and "
                "bolt spot face; NBK publishes no P30 D-shaft slip rating"
            ),
            "why_not_released": (
                "0.5 N m is clamp-bolt tightening torque, not output-shaft slip "
                "torque; NBK publishes no stock P30 retention rating"
            ),
            "smallest_decisive_gate": (
                f"NBK written guaranteed bidirectional shaft retention >= "
                f"{required_2x_nm:.6f} N m for the exact CS-M21708 shaft, or an "
                "actual-motor reversing slip coupon at the final threshold"
            ),
            **common_unknowns,
        },
        {
            "id": "stock_plus_BNS",
            "purchase_configuration": (
                "P30-3GT-BLP-6C-5 with NBK BNS additional machining"
            ),
            "orderability": (
                "listed NBK configurator option; generated drawing/order code required"
            ),
            "exact_official_STEP_available": False,
            "additional_machining": "one radial set-screw hole; screw included",
            "retention_features": "stock split clamp plus one set screw on the D-flat",
            "leadshine_D_shaft_interface": (
                "NBK says the D-cut flat is the fastening position for a set screw"
            ),
            "why_not_released": (
                "delivered screw size, position, tightening torque, shaft material/"
                "hardness match, and product-specific retention rating are unknown; "
                "the coupling slip chart is reference-only"
            ),
            "smallest_decisive_gate": (
                "configured BNS drawing plus NBK guaranteed rating, or one exact "
                "BNS/CS-M21708 reversing slip coupon at the final threshold"
            ),
            **common_unknowns,
        },
        {
            "id": "stock_plus_BNW",
            "purchase_configuration": (
                "P30-3GT-BLP-6C-5 with NBK BNW additional machining"
            ),
            "orderability": (
                "listed NBK configurator option; generated drawing/order code required"
            ),
            "exact_official_STEP_available": False,
            "additional_machining": (
                "two radial set-screw holes at 90 degrees; screws included"
            ),
            "retention_features": (
                "stock split clamp plus two set screws at 90 degrees, with one on "
                "the D-flat and the flat clear of the split/bolt spot face"
            ),
            "leadshine_D_shaft_interface": (
                "matches NBK's D-flat set-screw placement guidance while retaining "
                "the stock split clamp"
            ),
            "why_not_released": (
                "delivered screw size/position/torque and Leadshine shaft hardness "
                "are unpublished; NBK explicitly says its two-screw coupling chart "
                "is reference-only and actual-use testing is required"
            ),
            "smallest_decisive_gate": (
                "obtain the generated BNW drawing/order code, then either an NBK "
                f"guaranteed bidirectional rating >= {required_2x_nm:.6f} N m or one "
                "exact BNW-on-CS-M21708 reversing slip coupon at the final threshold"
            ),
            **common_unknowns,
        },
        {
            "id": "keyed_or_taper_lock_P30",
            "purchase_configuration": None,
            "orderability": "no exact stock drop-in configuration found",
            "exact_official_STEP_available": False,
            "additional_machining": (
                "NBK lists BKN/BKS/BKW/BKT keyways and BCN 1:100 tapered keyway "
                "as additional machining, not as stock P30-3GT-BLP-6C-5 parts"
            ),
            "retention_features": None,
            "leadshine_D_shaft_interface": (
                "CS-M21708 has a D-flat but no keyway; a keyed interface would "
                "require unauthorized motor-shaft modification"
            ),
            "why_not_released": (
                "BCN is a tapered keyway, not a taper-lock bushing; NBK's stock P30 "
                "list exposes only 5, 6, 8, and 10 mm split-clamp bores"
            ),
            "smallest_decisive_gate": (
                "none within the exact motor/P30 contract; change the motor shaft or "
                "drive architecture and re-review packaging if a keyed/taper-lock "
                "interface is required"
            ),
            **common_unknowns,
        },
    ]


def analyze() -> dict[str, Any]:
    vendor_hashes, nbk_source = _vendor_evidence()
    drive = _load_json(DRIVE_REPORT)
    route = _route_and_inertia_evidence(drive)

    wire_torque_nm = float(route["wire_torque_at_10N_nm"])
    full_j = float(route["final_full_output_inertia_kg_m2"])
    acceleration_torque_nm = ANGULAR_ACCELERATION_RAD_S2 * full_j
    required_running_nm = wire_torque_nm + FRICTION_ALLOWANCE_NM + acceleration_torque_nm
    required_2x_nm = REQUIRED_RETENTION_MULTIPLE * required_running_nm
    no_inertia_floor_2x_nm = REQUIRED_RETENTION_MULTIPLE * (
        wire_torque_nm + FRICTION_ALLOWANCE_NM
    )

    belt_allowable_nm = (
        NBK_BELT_BASE_ALLOWABLE_300RPM_NM
        * NBK_BELT_LENGTH_FACTOR_210MM
        * NBK_BELT_MESH_FACTOR
    )
    options = _option_rows(required_2x_nm)

    report = {
        "schema": "m2-p30-stock-retention-decision/v1",
        "status": (
            "FAIL_CLOSED__NO_PUBLISHED_P30_SHAFT_RETENTION_RATING__"
            "BNW_DRAWING_AND_RATING_OR_COUPON_REQUIRED"
        ),
        "normal_GOAL_modified": False,
        "production_authorized": False,
        "production_procurement_authorized": False,
        "prototype_coupon_purchase_recommended": True,
        "decision": {
            "stock_split_clamp_can_be_released_now": False,
            "lower_live_line_torque_changes_release_answer": False,
            "recommended_configuration": "stock_plus_BNW",
            "recommendation": (
                "Use exact P30-3GT-BLP-6C-5 plus NBK BNW as the RFQ/coupon "
                "configuration.  It preserves the stock split clamp and adds the "
                "official two-set-screw 90-degree option, with one screw on the "
                "Leadshine D-flat.  Do not production-order it until the generated "
                "drawing and a guaranteed rating or exact reversing coupon close "
                "the retention gate."
            ),
            "why_not_stock_only": (
                "NBK gives the stock M2 clamp-bolt installation torque but no "
                "pulley-to-shaft slip torque on a 5 mm D-shaft."
            ),
            "why_not_BNS": (
                "BNS adds only one unconfigured set screw; its size and rating are "
                "unknown, and the reference chart cannot be applied."
            ),
            "keyed_or_taper_lock_drop_in_exists": False,
        },
        "load_contract": {
            "route": route,
            "wire_tension_N": WIRE_TENSION_N,
            "friction_allowance_nm_unmeasured": FRICTION_ALLOWANCE_NM,
            "angular_acceleration_rad_s2": ANGULAR_ACCELERATION_RAD_S2,
            "acceleration_torque_nm": acceleration_torque_nm,
            "required_running_torque_nm": required_running_nm,
            "required_retention_multiple": REQUIRED_RETENTION_MULTIPLE,
            "required_2x_reversing_retention_nm": required_2x_nm,
            "wire_plus_friction_2x_floor_excluding_inertia_nm": no_inertia_floor_2x_nm,
            "final_required_2x_value_release_bound": route[
                "canonical_route_and_final_inertia_bound"
            ],
            "note": (
                "Current numeric threshold is provisional until the canonical crown "
                "report binds its locus hash and final M2 inertia.  The no-rating "
                "retention decision remains fail-closed at any value."
            ),
        },
        "interface_separation": {
            "belt_and_tooth_transmission": {
                "NBK_base_allowable_at_300rpm_nm": NBK_BELT_BASE_ALLOWABLE_300RPM_NM,
                "210mm_length_factor": NBK_BELT_LENGTH_FACTOR_210MM,
                "mesh_factor": NBK_BELT_MESH_FACTOR,
                "corrected_allowable_transmission_torque_nm": belt_allowable_nm,
                "allowable_to_current_required_2x_multiple": belt_allowable_nm
                / required_2x_nm,
                "passes_belt_capacity_math": belt_allowable_nm >= required_2x_nm,
                "proves_pulley_to_shaft_retention": False,
                "reason": (
                    "NBK's allowable-transmission table governs the belt/tooth path, "
                    "not hub slip on the motor shaft."
                ),
            },
            "stock_clamp_fastener": {
                "bolt": "M2 SCM435 black oxide",
                "installation_tightening_torque_nm": 0.5,
                "is_output_retention_rating": False,
                "must_not_be_compared_numerically_to_output_torque": True,
            },
            "pulley_to_shaft_retention": {
                "published_exact_stock_rating_nm": None,
                "published_BNS_rating_nm": None,
                "published_BNW_rating_nm": None,
                "governing_release_interface": True,
            },
        },
        "exact_stock_CAD_and_drawing": {
            "part_number": nbk_source["part_number"],
            "STEP_sha256": vendor_hashes[_relative(NBK_STEP)],
            "STEP_kind": "single stock part",
            "STEP_solid_count": 1,
            "STEP_face_count": 49,
            "shaft_axis_local": "+X",
            "bore_mm": 5.0,
            "overall_axial_length_mm": 18.5,
            "hub_diameter_mm": 20.0,
            "flange_diameter_mm": 32.0,
            "stock_clamp_bolt": "M2 at 0.5 N m",
            "contains_BNS_or_BNW": False,
            "contains_keyway_or_taper_lock": False,
            "license_boundary": (
                "official AP214 declares CC BY-ND 4.0 and remains byte-for-byte "
                "unmodified"
            ),
        },
        "Leadshine_shaft": {
            "model": "CS-M21708",
            "official_product_shaft_diameter_mm": 5.0,
            "official_2D_nominal_diameter_tolerance_mm": [0.0, -0.013],
            "D_flat_across_mm": 4.5,
            "D_flat_across_tolerance_mm": 0.1,
            "D_flat_length_mm": 15.0,
            "shaft_protrusion_mm": 24.0,
            "shaft_material_published": False,
            "shaft_hardness_published": False,
            "keyway_present": False,
        },
        "NBK_reference_slip_chart": {
            "scope": "coupling set-screw reference test data; not a P30 rating",
            "aluminum_hub_material_condition": "anodized aluminum alloy",
            "shaft_condition": "S45C, 16-27 HRC",
            "set_screw_condition": "SCM435 black oxide",
            "two_screw_curve_means_90_degree_spacing": True,
            "reference_only_not_guaranteed": True,
            "NBK_requires_actual_use_testing": True,
            "P30_BNS_delivered_screw_size_known": False,
            "P30_BNW_delivered_screw_size_known": False,
            "Leadshine_material_and_hardness_match_known": False,
            "can_prove_any_compared_configuration": False,
        },
        "options": options,
        "smallest_release_package": {
            "supplier_input": (
                "NBK generated drawing/order code for P30-3GT-BLP-6C-5 + BNW, "
                "showing exact set-screw size, axial/radial positions, tightening "
                "torque, and acceptance of the 5 mm D-shaft orientation"
            ),
            "decisive_proof": (
                f"Either NBK's written guaranteed bidirectional retention >= "
                f"{required_2x_nm:.6f} N m for the exact CS-M21708 shaft, or one "
                "sample installed on an actual CS-M21708 shaft and tested under "
                "the intended reversing/thermal conditions with no angular slip at "
                "the final hash-bound 2x threshold"
            ),
            "why_a_coupon_is_still_needed_without_a_guarantee": (
                "NBK calls its slip curves reference-only and explicitly requires "
                "testing under actual-use conditions."
            ),
        },
        "official_sources": {
            "NBK_product_and_configuration_page": NBK_PRODUCT_URL,
            "NBK_family_drawing": NBK_DRAWING_URL,
            "NBK_set_screw_slip_reference": NBK_SET_SCREW_SLIP_URL,
            "NBK_D_shaft_installation_guidance": NBK_D_SHAFT_GUIDANCE_URL,
            "Leadshine_product_page": LEADSHINE_PRODUCT_URL,
            "local_vendor_artifact_sha256": vendor_hashes,
            "drive_report_path": _relative(DRIVE_REPORT),
            "drive_report_sha256": _sha256(DRIVE_REPORT),
        },
        "release_gates": {
            "exact_stock_NBK_STEP_and_drawing_hash_bound": True,
            "exact_Leadshine_STEP_and_drawing_hash_bound": True,
            "belt_capacity_kept_separate_from_shaft_retention": True,
            "canonical_route_and_final_M2_inertia_bound": route[
                "canonical_route_and_final_inertia_bound"
            ],
            "configured_BNW_supplier_drawing_received": False,
            "exact_BNW_set_screw_size_position_and_torque_known": False,
            "supplier_guaranteed_bidirectional_retention_ge_final_2x": False,
            "exact_BNW_on_CS_M21708_reversing_coupon_passed": False,
            "stock_only_retention_release": False,
            "production_authorized": False,
        },
        "controlling_open_gates": [
            "canonical_route_and_final_M2_inertia_bound",
            "configured_BNW_supplier_drawing_received",
            "exact_BNW_set_screw_size_position_and_torque_known",
            (
                "supplier_guaranteed_bidirectional_retention_ge_final_2x OR "
                "exact_BNW_on_CS_M21708_reversing_coupon_passed"
            ),
        ],
    }
    return report


def _markdown(report: dict[str, Any]) -> str:
    load = report["load_contract"]
    route = load["route"]
    belt = report["interface_separation"]["belt_and_tooth_transmission"]
    rows = []
    for option in report["options"]:
        rows.append(
            "| {id} | {orderability} | {proof} | {release} |".format(
                id=option["id"],
                orderability=option["orderability"],
                proof="none published" if option["published_P30_shaft_slip_rating_nm"] is None else "published",
                release="NO" if not option["production_release"] else "YES",
            )
        )
    return f"""# M2 P30 stock-retention decision

Status: **FAIL CLOSED.  Keep P30-3GT-BLP-6C-5 + BNW as the RFQ/coupon configuration; do not production-order it yet.**

The route audit's current maximum M2 force-line arm is `{route['max_projected_moment_arm_mm']:.12f} mm`, or `{route['wire_torque_at_10N_nm']:.12f} N m` at 10 N.  With the current 0.020 N m friction allowance, 200 rad/s^2 acceleration, and `{route['final_full_output_inertia_kg_m2']:.12g} kg m^2` provisional full inertia, the current 2x reversing-retention target is **`{load['required_2x_reversing_retention_nm']:.9f} N m`**.  That value is not final until the shared-crown report binds the canonical locus hash and final guide inertia.

The lower wire torque does **not** release the stock split clamp.  NBK publishes a 0.5 N m *installation torque for the M2 clamp bolt*, not a pulley-to-shaft slip rating.  Likewise, the corrected 210-3GT-6 belt/tooth allowable torque is `{belt['corrected_allowable_transmission_torque_nm']:.3f} N m`; it passes belt capacity but is not shaft-retention evidence.

| Configuration | Orderability | Exact P30 shaft-slip proof | Release now |
| --- | --- | --- | --- |
{chr(10).join(rows)}

NBK lists BNS (one set screw), BNW (two set screws at 90 degrees), and keyed machining codes on the P30 page.  They are additional-machining configurations, not alternate stock CAD.  No exact stock keyed or taper-lock P30 drop-in was found: BCN is a 1:100 tapered **keyway**, not a taper-lock bushing, and the CS-M21708 shaft has no keyway.

NBK's set-screw slip chart cannot close this gate.  It is coupling reference data, assumes an S45C shaft at 16-27 HRC with known SCM435 screw size/torque, is explicitly not guaranteed, and requires actual-use testing.  Leadshine publishes the 5 mm D-profile geometry but not shaft material or hardness; NBK does not expose the delivered BNS/BNW screw size for this P30 configuration.

Smallest release package: obtain the generated **P30-3GT-BLP-6C-5 + BNW** drawing/order code with exact screw geometry and torque.  Then obtain either an NBK written bidirectional retention guarantee at or above the final hash-bound 2x target, or run one exact BNW pulley on an actual CS-M21708 shaft under the intended reversing/thermal conditions and demonstrate no angular slip at that target.
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
