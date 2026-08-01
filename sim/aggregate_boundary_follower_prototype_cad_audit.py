"""Fail-closed consolidation of the two isolated follower CAD prototypes.

The audit binds immutable bytes for the robust g=0 cap-shelf prototype and
the custom-return packaging prototype.  It loads their review manifests and
the predecessor analytical reports, but deliberately does not import the
aggregate-boundary acceptance module or any release workflow.

Only already-established local geometry facts are restated.  Selection,
loads, spring rate, fatigue, general clearance, complete route coverage,
integration, procurement, BOM, order, production and release authority all
remain false.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORTS = ROOT / "out" / "reports"
OUTPUT_JSON = REPORTS / "aggregate_boundary_follower_prototype_cad_audit.json"
OUTPUT_MD = REPORTS / "aggregate_boundary_follower_prototype_cad_audit.md"

SCHEMA = "aggregate-boundary-follower-prototype-cad-audit/v1"

G0_SOURCE = ROOT / "cad" / "aggregate_boundary_g0_cap_shelf.py"
G0_MANIFEST = ROOT / "out" / "review" / (
    "aggregate_boundary_g0_cap_shelf.manifest.json"
)
G0_STEP = ROOT / "out" / "review" / "aggregate_boundary_g0_cap_shelf.step"
G0_TRADE = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_g0_landing_trade.json"
)

RETURN_SOURCE = ROOT / "cad" / (
    "aggregate_boundary_follower_custom_return_packaging.py"
)
RETURN_MANIFEST = ROOT / "out" / "review" / (
    "aggregate_boundary_follower_custom_return_packaging.json"
)
RETURN_STEP = ROOT / "out" / "review" / (
    "aggregate_boundary_follower_custom_return_packaging.step"
)
RETURN_SCREEN = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_custom_return_screen.json"
)

EXPECTED_G0_SCHEMA = "aggregate-boundary-g0-cap-shelf-manifest/v1"
EXPECTED_G0_TRADE_SCHEMA = "aggregate-boundary-follower-g0-landing-trade/v1"
EXPECTED_RETURN_SCHEMA = (
    "aggregate-boundary-follower-custom-return-packaging/v1"
)
EXPECTED_RETURN_SCREEN_SCHEMA = (
    "aggregate-boundary-follower-custom-return-screen/v1"
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(
    value: Mapping[str, Any], *, omitted_key: str = "report_sha256",
) -> str:
    body = deepcopy(dict(value))
    body.pop(omitted_key, None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def _relative(path: Path) -> str:
    return str(Path(path).resolve().relative_to(ROOT.resolve())).replace("\\", "/")


def _validate_hashed_report(
    report: Mapping[str, Any], *, schema: str, label: str,
) -> None:
    if report.get("schema") != schema:
        raise ValueError(f"{label} schema drift")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError(f"{label} canonical report hash mismatch")


def _validate_g0_manifest(manifest: Mapping[str, Any]) -> None:
    if manifest.get("schema") != EXPECTED_G0_SCHEMA:
        raise ValueError("g0 cap-shelf manifest schema drift")
    if manifest.get("manifest_sha256") != _canonical_hash(
        manifest, omitted_key="manifest_sha256"
    ):
        raise ValueError("g0 cap-shelf canonical manifest hash mismatch")
    if manifest.get("artifact") != _relative(G0_STEP):
        raise ValueError("g0 cap-shelf artifact path drift")
    if manifest.get("artifact_sha256") != _sha256(G0_STEP):
        raise ValueError("g0 cap-shelf STEP hash mismatch")
    if manifest.get("artifact_byte_count") != G0_STEP.stat().st_size:
        raise ValueError("g0 cap-shelf STEP byte-count mismatch")
    for relative, expected in manifest.get("source_hashes", {}).items():
        source = ROOT / Path(str(relative).replace("\\", "/"))
        if not source.is_file() or _sha256(source) != expected:
            raise ValueError(f"stale g0 cap-shelf source {relative}")


def g0_cap_shelf_evidence() -> dict[str, Any]:
    """Return only the established cap-shelf geometry evidence."""

    manifest = _load(G0_MANIFEST)
    trade = _load(G0_TRADE)
    _validate_g0_manifest(manifest)
    _validate_hashed_report(
        trade, schema=EXPECTED_G0_TRADE_SCHEMA, label="g0 landing trade"
    )

    validation = manifest["validation"]
    contract = manifest["geometry_contract"]
    cases = validation["diameter_cases"]
    normalized_cases = []
    for row in cases:
        normalized_cases.append({
            "axial_end": row["axial_end"],
            "wire_diameter_mm": row["wire_diameter_mm"],
            "endpoint_to_cap_distance_mm": row[
                "endpoint_to_cap_distance_mm"
            ],
            "expected_wire_radius_mm": row["expected_wire_radius_mm"],
            "cap_to_wire_positive_overlap_mm3": row[
                "cap_to_wire_positive_overlap_mm3"
            ],
            "cap_to_R0p36_gauge_positive_overlap_mm3": row[
                "cap_to_R0p36_gauge_positive_overlap_mm3"
            ],
            "distance_equals_wire_radius": row["distance_equals_wire_radius"],
            "wire_zero_positive_overlap": row["wire_zero_positive_overlap"],
            "gauge_zero_positive_overlap": row["gauge_zero_positive_overlap"],
        })

    gates = {
        "front_and_rear_caps_are_single_solids": (
            validation["cap_solid_counts"] == {"front": 1, "rear": 1}
        ),
        "four_front_rear_diameter_cases_bound": (
            len(normalized_cases) == 4
            and {
                (row["axial_end"], row["wire_diameter_mm"])
                for row in normalized_cases
            } == {
                ("front", 0.2), ("front", 0.5),
                ("rear", 0.2), ("rear", 0.5),
            }
        ),
        "0p2_and_0p5_tangency_exact_to_1e_8mm": all(
            row["distance_equals_wire_radius"]
            and abs(
                row["endpoint_to_cap_distance_mm"]
                - row["expected_wire_radius_mm"]
            ) <= 1.0e-8
            for row in normalized_cases
        ),
        "wire_and_R0p36_gauge_zero_positive_overlap": all(
            row["wire_zero_positive_overlap"]
            and row["gauge_zero_positive_overlap"]
            and row["cap_to_wire_positive_overlap_mm3"] == 0.0
            and row["cap_to_R0p36_gauge_positive_overlap_mm3"] == 0.0
            for row in normalized_cases
        ),
        "complete_cap_lane_clear_width_is_0p65mm": (
            contract["lane_clear_width_mm"] == 0.65
        ),
        "landing_trade_selected_only_topology_for_redesign": (
            trade["status"] == "FAIL"
            and trade["selected_candidate_id"]
            == "diameter_rebound_integral_PEEK_cap_side_shelf"
            and trade["selected_release_modified"] is False
        ),
    }

    return {
        "prototype": "aggregate_boundary_g0_cap_shelf",
        "status": "PASS_ESTABLISHED_LOCAL_GEOMETRY_ONLY",
        "artifact": {
            "path": _relative(G0_STEP),
            "byte_count": G0_STEP.stat().st_size,
            "sha256": _sha256(G0_STEP),
        },
        "bindings": {
            "source": {
                "path": _relative(G0_SOURCE),
                "sha256": _sha256(G0_SOURCE),
            },
            "manifest": {
                "path": _relative(G0_MANIFEST),
                "sha256": _sha256(G0_MANIFEST),
                "canonical_manifest_sha256": manifest["manifest_sha256"],
            },
            "g0_landing_trade": {
                "path": _relative(G0_TRADE),
                "sha256": _sha256(G0_TRADE),
                "canonical_report_sha256": trade["report_sha256"],
            },
        },
        "established_geometry": {
            "cap_solid_counts": validation["cap_solid_counts"],
            "lane_clear_width_mm": contract["lane_clear_width_mm"],
            "wire_diameter_cases_mm": contract["wire_diameter_review_mm"],
            "diameter_cases": normalized_cases,
        },
        "predecessor_trade_context": {
            "status": trade["status"],
            "decision": trade["decision"],
            "selected_candidate_id_for_redesign_only": trade[
                "selected_candidate_id"
            ],
            "selected_release_modified": trade["selected_release_modified"],
        },
        "evidence_gates": gates,
        "selection_authority": False,
        "clearance_authority": False,
        "complete_2400_route_authority": False,
    }


def custom_return_package_evidence() -> dict[str, Any]:
    """Return only the established custom-return package geometry evidence."""

    manifest = _load(RETURN_MANIFEST)
    screen = _load(RETURN_SCREEN)
    if manifest.get("schema") != EXPECTED_RETURN_SCHEMA:
        raise ValueError("custom-return package manifest schema drift")
    _validate_hashed_report(
        screen,
        schema=EXPECTED_RETURN_SCREEN_SCHEMA,
        label="custom-return analytical screen",
    )

    artifacts = manifest["artifacts"]
    if artifacts["source"] != _relative(RETURN_SOURCE):
        raise ValueError("custom-return source path drift")
    if artifacts["step"] != _relative(RETURN_STEP):
        raise ValueError("custom-return STEP path drift")
    if artifacts["manifest"] != _relative(RETURN_MANIFEST):
        raise ValueError("custom-return manifest path drift")

    shaft = manifest["shaft"]
    igus = manifest["igus_bushing_and_pocket"]
    cartridge = manifest["radial_cartridge"]
    body = manifest["body_contract"]
    state_q = {
        "negative_hard": -0.6,
        "center": 0.0,
        "positive_hard": 0.6,
    }
    state_rows = []
    for state, q_mm in state_q.items():
        row = body["overlap_and_bounds_by_state"][state]
        state_rows.append({
            "state": state,
            "tangential_q_mm": q_mm,
            "all_part_leaf_count": row["all_parts"]["body_count"],
            "all_part_positive_overlap_count": row[
                "all_parts"
            ]["positive_overlap_count"],
            "custom_body_positive_overlap_count": row[
                "custom"
            ]["positive_overlap_count"],
            "all_part_overlap_status": row["all_parts"]["status"],
            "custom_body_overlap_status": row["custom"]["status"],
        })

    gates = {
        "center_STEP_package_has_15_leaf_bodies": (
            body["all_center_body_count"] == 15
            and all(row["all_part_leaf_count"] == 15 for row in state_rows)
        ),
        "custom_center_bodies_are_single_solids": body[
            "all_custom_center_bodies_single_solid"
        ],
        "shaft_is_exact_nominal_OD3x16_envelope": (
            shaft["diameter_mm"] == 3.0
            and shaft["length_mm"] == 16.0
            and shaft["axis"] == "+Y"
        ),
        "igus_dimensions_exactly_bound": (
            igus["catalog_number"] == "WPFFM-0304-05"
            and igus["ID_mm"] == 3.0
            and igus["body_OD_mm"] == 4.5
            and igus["body_length_mm"] == 5.0
            and igus["flange_OD_mm"] == 7.5
            and igus["flange_thickness_mm"] == 0.75
        ),
        "9293K122_envelope_dimensions_exactly_bound": (
            cartridge["catalog_number"] == "9293K122"
            and cartridge["coil_OD_mm"] == 15.75
            and cartridge["coil_width_mm"] == 6.35
        ),
        "zero_same_state_overlap_at_q_minus0p6_0_plus0p6": all(
            row["all_part_leaf_count"] == 15
            and row["all_part_positive_overlap_count"] == 0
            and row["custom_body_positive_overlap_count"] == 0
            and row["all_part_overlap_status"] == "PASS"
            and row["custom_body_overlap_status"] == "PASS"
            for row in state_rows
        ),
        "analytical_screen_has_no_production_selection": (
            screen["recommendation"]["production_selection"] is None
            and screen["physical_authority"] is False
            and screen["CAD_authority"] is False
            and screen["procurement_authority"] is False
            and screen["release_authority"] is False
        ),
    }

    return {
        "prototype": "aggregate_boundary_follower_custom_return_packaging",
        "status": "PASS_ESTABLISHED_LOCAL_GEOMETRY_ONLY",
        "artifact": {
            "path": _relative(RETURN_STEP),
            "byte_count": RETURN_STEP.stat().st_size,
            "sha256": _sha256(RETURN_STEP),
        },
        "bindings": {
            "source": {
                "path": _relative(RETURN_SOURCE),
                "sha256": _sha256(RETURN_SOURCE),
            },
            "manifest": {
                "path": _relative(RETURN_MANIFEST),
                "sha256": _sha256(RETURN_MANIFEST),
            },
            "custom_return_screen": {
                "path": _relative(RETURN_SCREEN),
                "sha256": _sha256(RETURN_SCREEN),
                "canonical_report_sha256": screen["report_sha256"],
            },
        },
        "established_geometry": {
            "STEP_leaf_body_count": body["all_center_body_count"],
            "custom_center_body_count": body["custom_center_body_count"],
            "shaft": {
                "diameter_mm": shaft["diameter_mm"],
                "length_mm": shaft["length_mm"],
                "axis": shaft["axis"],
            },
            "igus_WPFFM_0304_05": {
                "ID_mm": igus["ID_mm"],
                "body_OD_mm": igus["body_OD_mm"],
                "body_length_mm": igus["body_length_mm"],
                "flange_OD_mm": igus["flange_OD_mm"],
                "flange_thickness_mm": igus["flange_thickness_mm"],
            },
            "McMaster_9293K122_envelope": {
                "coil_OD_mm": cartridge["coil_OD_mm"],
                "coil_width_mm": cartridge["coil_width_mm"],
            },
            "same_state_overlap_rows": state_rows,
        },
        "analytical_screen_context": {
            "status": screen["status"],
            "tangential_primary": screen["recommendation"][
                "tangential_primary"
            ],
            "radial_bounded_prototype": screen["recommendation"][
                "radial_bounded_prototype"
            ],
            "production_selection": screen["recommendation"][
                "production_selection"
            ],
        },
        "evidence_gates": gates,
        "selection_authority": False,
        "load_authority": False,
        "spring_rate_authority": False,
        "fatigue_authority": False,
        "clearance_authority": False,
    }


def build_report() -> dict[str, Any]:
    g0 = g0_cap_shelf_evidence()
    custom_return = custom_return_package_evidence()
    evidence_gates = {
        "g0_cap_shelf_established_geometry_bound": all(
            g0["evidence_gates"].values()
        ),
        "custom_return_established_geometry_bound": all(
            custom_return["evidence_gates"].values()
        ),
        "both_STEP_artifact_hashes_present": all(
            len(row["artifact"]["sha256"]) == 64
            for row in (g0, custom_return)
        ),
        "no_acceptance_or_release_input_imported": True,
    }
    fail_closed_gates = {
        "prototype_selected_for_machine": False,
        "load_authority": False,
        "spring_rate_authority": False,
        "fatigue_authority": False,
        "general_clearance_authority": False,
        "complete_2400_route_authority": False,
        "assembly_integration_authority": False,
        "procurement_authority": False,
        "BOM_change_authorized": False,
        "order_authorized": False,
        "production_authority": False,
        "release_authority": False,
    }
    report = {
        "schema": SCHEMA,
        "status": "PASS_ISOLATED_PROTOTYPE_GEOMETRY_ONLY_NO_AUTHORITY",
        "prototype_count": 2,
        "artifacts": {
            "aggregate_boundary_g0_cap_shelf_STEP": g0["artifact"],
            "aggregate_boundary_follower_custom_return_packaging_STEP": (
                custom_return["artifact"]
            ),
        },
        "aggregate_boundary_g0_cap_shelf": g0,
        "aggregate_boundary_follower_custom_return_packaging": custom_return,
        "evidence_gates": evidence_gates,
        "fail_closed_gates": fail_closed_gates,
        "selection_authority": False,
        "load_authority": False,
        "spring_rate_authority": False,
        "fatigue_authority": False,
        "clearance_authority": False,
        "complete_2400_route_authority": False,
        "integration_authority": False,
        "procurement_authority": False,
        "BOM_change_authorized": False,
        "order_authorized": False,
        "production_authority": False,
        "release_authority": False,
        "source_bindings": {
            _relative(Path(__file__).resolve()): _sha256(Path(__file__).resolve()),
            _relative(G0_SOURCE): _sha256(G0_SOURCE),
            _relative(G0_MANIFEST): _sha256(G0_MANIFEST),
            _relative(G0_STEP): _sha256(G0_STEP),
            _relative(G0_TRADE): _sha256(G0_TRADE),
            _relative(RETURN_SOURCE): _sha256(RETURN_SOURCE),
            _relative(RETURN_MANIFEST): _sha256(RETURN_MANIFEST),
            _relative(RETURN_STEP): _sha256(RETURN_STEP),
            _relative(RETURN_SCREEN): _sha256(RETURN_SCREEN),
        },
        "decision": (
            "Both isolated STEP artifacts are current and their bounded local "
            "geometry evidence passes. This audit does not select either "
            "prototype and grants no load, rate, fatigue, clearance, complete "
            "route, integration, procurement, BOM, order, production or "
            "release authority."
        ),
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("prototype CAD audit schema drift")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("prototype CAD audit canonical hash mismatch")
    for relative, expected in report.get("source_bindings", {}).items():
        path = ROOT / Path(str(relative).replace("\\", "/"))
        if not path.is_file() or _sha256(path) != expected:
            raise ValueError(f"stale prototype CAD audit source {relative}")
    if not all(report.get("evidence_gates", {}).values()):
        raise ValueError("prototype CAD evidence gate failed")
    if not all(
        value is False for value in report.get("fail_closed_gates", {}).values()
    ):
        raise ValueError("prototype CAD authority gate opened")


def _markdown(report: Mapping[str, Any]) -> str:
    g0 = report["aggregate_boundary_g0_cap_shelf"]
    ret = report["aggregate_boundary_follower_custom_return_packaging"]
    lines = [
        "# Aggregate-boundary follower isolated prototype CAD audit",
        "",
        f"- Status: **{report['status']}**",
        "- Prototype selection and every broader authority: **false**",
        "",
        "## Artifact bindings",
        "",
        "| Prototype | STEP bytes | STEP SHA-256 |",
        "|---|---:|---|",
        f"| g=0 cap shelf | {g0['artifact']['byte_count']} | `{g0['artifact']['sha256']}` |",
        f"| Custom return package | {ret['artifact']['byte_count']} | `{ret['artifact']['sha256']}` |",
        "",
        "## g=0 cap-shelf geometry",
        "",
        f"- Front/rear cap solid counts: {g0['established_geometry']['cap_solid_counts']}.",
        f"- Complete modeled cap lane clear width: {g0['established_geometry']['lane_clear_width_mm']:.2f} mm.",
        "- Exact local tangency and overlap cases:",
        "",
        "| End | Wire dia. | Cap distance | Wire overlap | R0.36 gauge overlap |",
        "|---|---:|---:|---:|---:|",
    ]
    for row in g0["established_geometry"]["diameter_cases"]:
        lines.append(
            f"| {row['axial_end']} | {row['wire_diameter_mm']:.1f} mm | "
            f"{row['endpoint_to_cap_distance_mm']:.8f} mm | "
            f"{row['cap_to_wire_positive_overlap_mm3']:.1f} mm3 | "
            f"{row['cap_to_R0p36_gauge_positive_overlap_mm3']:.1f} mm3 |"
        )
    lines.extend([
        "",
        "The predecessor landing trade selected this topology only for detailed "
        "redesign and remained `FAIL`; it did not modify the selected release.",
        "",
        "## Custom-return package geometry",
        "",
        f"- STEP leaf/body count: {ret['established_geometry']['STEP_leaf_body_count']}.",
        "- Nominal shaft envelope: OD3 x 16 mm, axis +Y.",
        "- igus WPFFM-0304-05: ID3, body OD4.5 x 5, flange OD7.5 x 0.75 mm.",
        "- McMaster 9293K122 envelope: coil OD15.75 x 6.35 mm wide.",
        "",
        "| Tangential q | All-part leaves | All-part overlaps | Custom overlaps |",
        "|---:|---:|---:|---:|",
    ])
    for row in ret["established_geometry"]["same_state_overlap_rows"]:
        lines.append(
            f"| {row['tangential_q_mm']:+.1f} mm | "
            f"{row['all_part_leaf_count']} | "
            f"{row['all_part_positive_overlap_count']} | "
            f"{row['custom_body_positive_overlap_count']} |"
        )
    lines.extend(["", "## Fail-closed authority", ""])
    for gate, value in report["fail_closed_gates"].items():
        lines.append(f"- `{gate}`: **{str(value).lower()}**")
    lines.extend([
        "",
        "Local zero positive-volume overlap is not general clearance authority; "
        "the complete 2,400-route sweep remains open.",
        "",
        report["decision"],
        "",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ])
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
