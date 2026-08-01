"""Fail-closed audit of the isolated aggregate-boundary follower CAD.

This proves the authored positive-volume R3 prototype geometry only.  It does
not turn that prototype into a complete mechanism and grants no assembly,
collision, wire-route, procurement, BOM, or production authority.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import itertools
import json
from pathlib import Path
import sys
from typing import Any, Iterable, Mapping

from build123d import Vector


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

import aggregate_boundary_floating_follower as follower


CAD_SOURCE = CAD / "aggregate_boundary_floating_follower.py"
CAD_BRIEF = CAD / "aggregate_boundary_floating_follower_brief.md"
STEP_PATH = ROOT / "out" / "review" / (
    "aggregate_boundary_floating_follower.step"
)
OUTPUT_JSON = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_cad_audit.json"
)
OUTPUT_MD = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_cad_audit.md"
)
SCHEMA = "aggregate-boundary-follower-cad-audit/v1"
INSPECTED_STEP_SHA256 = (
    "092db9a20b404af4a54f1df700f9c7831c1784edd28f9385bbf232b9b7eec6d0"
)
INSPECTED_STEP_LEAF_COUNT = 40
INSPECTED_STEP_BOUNDS_MM = [39.55, 56.0, 151.0]

RADIAL_STATES = ("retracted", "mid", "extended")
TANGENTIAL_STATES = ("negative", "center", "positive")


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(value: Mapping[str, Any]) -> str:
    body = deepcopy(dict(value))
    body.pop("report_sha256", None)
    return hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def _state_row(radial_state: str, tangential_state: str) -> dict[str, Any]:
    parts = follower.custom_bodies(radial_state, tangential_state)
    bodies = [{
        "label": part.label,
        "volume_mm3": float(part.volume),
        "solid_count": len(part.solids()),
    } for part in parts]
    overlaps = []
    for one, two in itertools.combinations(parts, 2):
        volume = follower._common_volume(one, two)
        if volume > 1.0e-7:
            overlaps.append({
                "one": one.label,
                "two": two.label,
                "common_volume_mm3": volume,
            })
    positive_single = all(
        row["volume_mm3"] > 0.0 and row["solid_count"] == 1
        for row in bodies
    )
    return {
        "radial_state": radial_state,
        "tangential_state": tangential_state,
        "custom_body_count": len(bodies),
        "all_custom_bodies_positive_single_solid": positive_single,
        "positive_overlap_count": len(overlaps),
        "positive_overlaps": overlaps,
        "status": "PASS" if positive_single and not overlaps else "FAIL",
        "bodies": bodies,
    }


def _step_evidence() -> dict[str, Any]:
    if not STEP_PATH.is_file():
        return {
            "path": "out/review/aggregate_boundary_floating_follower.step",
            "exists": False,
            "byte_count": None,
            "sha256": None,
            "matches_inspected_authoritative_sha256": False,
            "leaf_count": None,
            "leaf_count_method": "NOT_AVAILABLE",
            "bounds_mm": None,
            "inspection_warning_count": None,
        }
    observed_hash = _sha256(STEP_PATH)
    inspection_bound = observed_hash == INSPECTED_STEP_SHA256
    return {
        "path": "out/review/aggregate_boundary_floating_follower.step",
        "exists": True,
        "byte_count": STEP_PATH.stat().st_size,
        "sha256": observed_hash,
        "matches_inspected_authoritative_sha256": inspection_bound,
        "leaf_count": INSPECTED_STEP_LEAF_COUNT if inspection_bound else None,
        "leaf_count_method": (
            "ROOT_OCC_INSPECTION_BOUND_BY_SHA256"
            if inspection_bound else "UNBOUND_HASH_DRIFT"
        ),
        "bounds_mm": INSPECTED_STEP_BOUNDS_MM if inspection_bound else None,
        "inspection_warning_count": 0 if inspection_bound else None,
    }


def analyze() -> dict[str, Any]:
    contract = follower.geometry_contract()
    step_evidence = _step_evidence()
    state_rows = [
        _state_row(radial, tangential)
        for radial, tangential in itertools.product(
            RADIAL_STATES, TANGENTIAL_STATES,
        )
    ]

    nose = follower.nose_insert("mid", "center")
    gimbal_x, gimbal_y, gimbal_z = follower._gimbal_center("mid", "center")
    nose_x = gimbal_x + 8.0
    nose_contract = contract["nose_contract"]
    nose_witness = {
        "positive_volume_mm3": float(nose.volume),
        "solid_count": len(nose.solids()),
        "axis_center_is_open_for_pivot": not nose.is_inside(
            Vector(nose_x, gimbal_y, gimbal_z)
        ),
        "R2p99_floor_point_is_solid": nose.is_inside(
            Vector(nose_x + 2.99, gimbal_y, gimbal_z)
        ),
        "R3p10_open_groove_point_is_clear": not nose.is_inside(
            Vector(nose_x + 3.10, gimbal_y, gimbal_z)
        ),
        "R3p40_flange_point_is_solid": nose.is_inside(
            Vector(nose_x + 3.40, gimbal_y, gimbal_z + 1.0)
        ),
        "source_contact_radius_mm": nose_contract[
            "contact_surface_radius_mm"
        ],
        "source_cylinder_axis": nose_contract["nose_cylinder_axis"],
        "source_convex_arc_plane": nose_contract["convex_arc_plane"],
        "source_open_groove_clear_width_mm": nose_contract[
            "open_groove_clear_width_mm"
        ],
    }
    positive_r3 = (
        nose_witness["positive_volume_mm3"] > 0.0
        and nose_witness["solid_count"] == 1
        and nose_witness["axis_center_is_open_for_pivot"]
        and nose_witness["R2p99_floor_point_is_solid"]
        and nose_witness["R3p10_open_groove_point_is_clear"]
        and nose_witness["R3p40_flange_point_is_solid"]
        and nose_witness["source_contact_radius_mm"] == 3.0
        and nose_witness["source_cylinder_axis"] == "+Z_stator_axis"
        and nose_witness["source_convex_arc_plane"]
        == "active_local_XY_at_fixed_Z"
        and nose_witness["source_open_groove_clear_width_mm"] >= 0.65
    )

    primary_parts = follower.tower_m4_hardware()
    primary_labels = [part.label for part in primary_parts]
    secondary_parts = follower.secondary_m3_clamp_hardware_envelopes()
    secondary_labels = [part.label for part in secondary_parts]
    fasteners = contract["fastener_contract"]
    primary_complete = (
        len(primary_parts) == 12
        and sum("M4x10" in label for label in primary_labels) == 4
        and sum("M4_washer" in label for label in primary_labels) == 4
        and sum("M4_short_heat_insert" in label
                for label in primary_labels) == 4
        and all(part.volume > 0.0 for part in primary_parts)
        and fasteners["primary_load_case_N"] == 40.0
        and fasteners["primary_load_per_M4_N"] == 10.0
    )
    secondary_nonproof = (
        len(secondary_parts) == 6
        and sum("M3x8_screw" in label for label in secondary_labels) == 2
        and sum("M3_washer" in label for label in secondary_labels) == 2
        and sum("M3x3p4_insert" in label for label in secondary_labels) == 2
        and fasteners["secondary_M3_structural_proof_claimed"] is False
        and fasteners["secondary_M3_status"]
        == "UNQUALIFIED_EXPLODED_ENVELOPE_NOT_PRIMARY_LOAD_PATH"
    )
    pivot_parts = follower.gimbal_pin_hardware("mid", "center")
    pivot_labels = [part.label for part in pivot_parts]
    inner_pivot = fasteners["inner_pivot"]
    inner_pivot_selected_retained = (
        inner_pivot["status"] == "CATALOG_SELECTED_GEOMETRIC_STACK_MODELED"
        and bool(inner_pivot.get("sku"))
        and sum("inner_pivot_McMaster" in label
                for label in pivot_labels) == 1
        and sum("inner_pivot_DIN988_3x6x0p5" in label
                for label in pivot_labels) == 4
        and sum("inner_pivot_M2_nyloc" in label
                for label in pivot_labels) == 1
    )

    stroke = contract["stroke_contract"]
    geometry_gates = {
        "all_9_endpoint_states_audited": len(state_rows) == 9,
        "every_custom_body_positive_single_solid": all(
            row["all_custom_bodies_positive_single_solid"]
            for row in state_rows
        ),
        "zero_same_state_positive_overlap": all(
            row["positive_overlap_count"] == 0 for row in state_rows
        ),
        "usable_radial_travel_is_6mm": stroke["radial_stroke_mm"] == 6.0,
        "usable_tangential_travel_is_1mm": (
            stroke["tangential_stroke_mm"] == 1.0
        ),
        "hard_radial_center_travel_is_6p4mm": (
            stroke["radial_hard_center_travel_mm"] == 6.4
        ),
        "hard_tangential_center_stops_are_plus_minus_0p6mm": (
            stroke["tangential_hard_center_stops_mm"] == [-0.6, 0.6]
        ),
        "all_endpoint_tongues_captured": stroke[
            "all_endpoint_tongues_captured"
        ] is True,
        "positive_volume_R3_nose_with_plus_Z_axis": positive_r3,
        "four_complete_primary_M4_stacks_at_40N_10N_each": primary_complete,
        "secondary_M3_stacks_are_nonproof": secondary_nonproof,
        "STEP_matches_inspected_40_leaf_authoritative_artifact": (
            step_evidence["exists"]
            and step_evidence["matches_inspected_authoritative_sha256"]
            and step_evidence["leaf_count"] == 40
            and step_evidence["inspection_warning_count"] == 0
        ),
    }

    selection = contract["selection_and_retraction"]
    springs = contract["spring_contract"]
    authority = contract["authority"]
    mechanism_gates = {
        "inner_pivot_selected_and_retained": inner_pivot_selected_retained,
        "radial_spring_anchors_and_bellcrank_linkage_modeled": False,
        "tangential_bearing_selected_and_modeled": False,
        "tangential_return_spring_selected_and_anchored": False,
        "monolithic_tangential_slide_outer_yoke_complete": (
            contract["monolithic_cartridge_contract"][
                "root_blend_geometry_modeled"
            ] is True
            and contract["monolithic_cartridge_contract"][
                "separate_slide_to_yoke_fasteners_required"
            ] is False
        ),
        "M0_positive_retraction_linkage_attached": (
            selection["M0_dock_attached_to_actuator"] is True
        ),
        "assembly_integration_authorized": authority[
            "assembly_integration_authorized"
        ],
        "collision_authorized": authority["collision_authorized"],
        "wire_route_authorized": authority["wire_route_authorized"],
    }
    mechanism_complete = all(mechanism_gates.values())
    geometry_proven = all(geometry_gates.values())

    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "FAIL" if not mechanism_complete else "PASS",
        "decision": (
            "POSITIVE_VOLUME_R3_PROTOTYPE_GEOMETRY_PROVEN__"
            "MECHANISM_INCOMPLETE"
        ),
        "positive_volume_R3_prototype_geometry_proven": geometry_proven,
        "mechanism_complete": mechanism_complete,
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "collision_authorized": False,
        "wire_route_authorized": False,
        "route_authority_source": None,
        "state_coverage": {
            "radial_states": list(RADIAL_STATES),
            "tangential_states": list(TANGENTIAL_STATES),
            "required_state_count": 9,
            "audited_state_count": len(state_rows),
            "passing_state_count": sum(
                row["status"] == "PASS" for row in state_rows
            ),
            "states": state_rows,
        },
        "stroke_and_capture": {
            "usable_radial_mm": stroke["radial_stroke_mm"],
            "usable_tangential_mm": stroke["tangential_stroke_mm"],
            "hard_radial_center_travel_mm": stroke[
                "radial_hard_center_travel_mm"
            ],
            "hard_tangential_center_stops_mm": stroke[
                "tangential_hard_center_stops_mm"
            ],
            "all_endpoint_tongues_captured": stroke[
                "all_endpoint_tongues_captured"
            ],
        },
        "R3_nose_witness": nose_witness,
        "hardware_witness": {
            "primary_M4": {
                "part_count": len(primary_parts),
                "labels": primary_labels,
                "complete_stack_count": 4,
                "load_case_N": fasteners["primary_load_case_N"],
                "load_per_fastener_N": fasteners["primary_load_per_M4_N"],
                "complete": primary_complete,
            },
            "secondary_M3": {
                "part_count": len(secondary_parts),
                "labels": secondary_labels,
                "complete_stack_count": 2,
                "structural_proof_claimed": False,
                "status": fasteners["secondary_M3_status"],
                "nonproof_contract_pass": secondary_nonproof,
            },
            "gimbal_pivots": {
                "part_count": len(pivot_parts),
                "labels": pivot_labels,
                "inner_pivot_sku": inner_pivot.get("sku"),
                "inner_pivot_selected_and_retained": (
                    inner_pivot_selected_retained
                ),
            },
        },
        "geometry_gates": geometry_gates,
        "mechanism_gates": mechanism_gates,
        "mechanism_blockers": [
            "UNMODELED_radial_spring_anchors_and_bellcrank_linkage",
            springs["tangential_spring"],
            "UNSELECTED_tangential_bushing_guide_envelope",
            "UNMODELED_M0_positive_retraction_linkage",
            "UNRUN_assembly_collision_and_wire_route_validation",
        ],
        "source_evidence": {
            "cad_source": "cad/aggregate_boundary_floating_follower.py",
            "cad_source_sha256": _sha256(CAD_SOURCE),
            "cad_brief": "cad/aggregate_boundary_floating_follower_brief.md",
            "cad_brief_sha256": _sha256(CAD_BRIEF),
            "step": step_evidence,
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    validate_report_integrity(report)
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported follower CAD audit schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("follower CAD audit report hash mismatch")
    if report.get("positive_volume_R3_prototype_geometry_proven") is not True:
        raise ValueError("positive-volume R3 prototype geometry not proven")
    if report.get("mechanism_complete") is not False:
        raise ValueError("incomplete follower was promoted to complete mechanism")
    if report.get("status") != "FAIL":
        raise ValueError("incomplete follower audit must fail closed")
    for name in (
        "production_authorized", "assembly_integration_authorized",
        "collision_authorized", "wire_route_authorized",
    ):
        if report.get(name) is not False:
            raise ValueError(f"follower CAD audit invented {name}")
    evidence = report.get("source_evidence", {})
    if evidence.get("cad_source_sha256") != _sha256(CAD_SOURCE):
        raise ValueError("follower CAD source hash mismatch")
    if evidence.get("cad_brief_sha256") != _sha256(CAD_BRIEF):
        raise ValueError("follower CAD brief hash mismatch")
    step = evidence.get("step", {})
    if step.get("exists"):
        if (not STEP_PATH.is_file()
                or step.get("sha256") != _sha256(STEP_PATH)
                or step.get("byte_count") != STEP_PATH.stat().st_size):
            raise ValueError("follower STEP hash or size mismatch")


def render_markdown(report: Mapping[str, Any]) -> str:
    coverage = report["state_coverage"]
    stroke = report["stroke_and_capture"]
    step = report["source_evidence"]["step"]
    lines = [
        "# Aggregate-boundary floating follower CAD audit", "",
        f"**{report['status']} — {report['decision']}**", "",
        "The isolated positive-volume R3 prototype geometry is proven. The "
        "mechanism is not complete and remains fail-closed.", "",
        "## Proven geometry", "",
        f"- Endpoint states: {coverage['passing_state_count']} / "
        f"{coverage['required_state_count']} pass single-solid and zero-overlap checks.",
        f"- Usable travel: {stroke['usable_radial_mm']:.1f} mm radial and "
        f"{stroke['usable_tangential_mm']:.1f} mm tangential.",
        f"- Hard travel/stops: {stroke['hard_radial_center_travel_mm']:.1f} mm radial; "
        f"tangential {stroke['hard_tangential_center_stops_mm']} mm.",
        "- PEEK nose: positive-volume R3.0 groove floor, axis +Z, open width 0.65 mm.",
        "- Primary mount: four complete M4x10 stacks at the preliminary 40 N / 10 N-each load case.",
        "- Secondary M3 stacks: visible but explicitly non-proof.", "",
        "## Mechanism blockers", "",
    ]
    lines.extend(f"- `{blocker}`" for blocker in report["mechanism_blockers"])
    lines.extend([
        "", "## Artifact binding", "",
        f"- CAD source SHA-256: `{report['source_evidence']['cad_source_sha256']}`",
        f"- CAD brief SHA-256: `{report['source_evidence']['cad_brief_sha256']}`",
        f"- STEP exists: {step['exists']}; bytes: {step['byte_count']}; SHA-256: `{step['sha256']}`",
        f"- STEP leaf count: {step['leaf_count']} ({step['leaf_count_method']}).", "",
        "Assembly integration, collision clearance, wire-route validity, BOM "
        "release, procurement, and production remain unauthorized.", "",
        f"Report SHA-256: `{report['report_sha256']}`", "",
    ])
    return "\n".join(lines)


def write_outputs(report: Mapping[str, Any] | None = None) -> dict[str, Any]:
    value = dict(report or analyze())
    validate_report_integrity(value)
    OUTPUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(
        json.dumps(value, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    OUTPUT_MD.write_text(render_markdown(value), encoding="utf-8")
    return value


def main(argv: Iterable[str] | None = None) -> int:
    del argv
    report = write_outputs()
    print(
        "aggregate follower CAD audit: "
        f"geometry={report['positive_volume_R3_prototype_geometry_proven']}; "
        f"mechanism={report['mechanism_complete']}; "
        f"states={report['state_coverage']['passing_state_count']}/9"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
