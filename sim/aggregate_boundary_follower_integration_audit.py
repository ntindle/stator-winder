"""Deterministic fail-closed integration audit for the isolated follower.

The audit binds the current follower and integrated-assembly artifacts,
reproduces the decisive exact carrier/tower and carrier/yoke intersections,
and preserves the bounded 2026-07-12 OCC diagnostic witness set.  It does not
modify or integrate CAD and grants no clearance, route, release, or production
authority.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable, Mapping

from build123d import Align, Box, Pos


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

import aggregate_boundary_floating_follower as follower
import carriage_active_sector_terminal_guide as guide
from params import PARAMS


SCHEMA = "aggregate-boundary-follower-integration-audit/v1"
OUTPUT_JSON = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_integration_audit.json"
)
OUTPUT_MD = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_integration_audit.md"
)

FOLLOWER_STEP = ROOT / "out" / "review" / (
    "aggregate_boundary_floating_follower.step"
)
ASSEMBLY_STEP = ROOT / "out" / "review" / (
    "integrated_release_candidate.step"
)
EXISTING_COLLISION_REPORT = ROOT / "out" / "reports" / (
    "carriage_active_sector_terminal_guide_audit.json"
)

SOURCE_PATHS = (
    Path("cad/aggregate_boundary_floating_follower.py"),
    Path("cad/aggregate_boundary_floating_follower_brief.md"),
    Path("cad/carriage_active_sector_terminal_guide.py"),
    Path("cad/assembly.py"),
    Path("cad/integrated_release_candidate.py"),
    Path("cad/params.py"),
    Path("sim/integrated_phase_aware_wire_path.py"),
)

REFERENCE_RADIAL_STATE = "mid"
REFERENCE_TANGENTIAL_STATE = "center"
REFERENCE_M0_RAD = 0.0
REFERENCE_M1_DEG = 0.0
REFERENCE_M2_DEG = 0.0
DEEP_DIAGNOSTIC_M0_RAD = -61.918


# Exact positive-volume rows from the bounded OCC source-to-source scan.  The
# rows are valid only while every bound source/artifact hash remains current.
REFERENCE_POSITIVE_PAIRS: tuple[tuple[str, str, str, float], ...] = (
    ("context_exact_keyed_tower_face_M4_insert_pilots", "carriage",
     "spindle_tower_with_active_sector_M4_insert_pilots", 3398.745928),
    ("shared_axial_carrier_with_integral_radial_stops", "carriage",
     "M0_carriage_owned_aluminum_active_sector_split_yoke", 1782.698995),
    ("shared_axial_carrier_with_integral_radial_stops", "carriage",
     "spindle_tower_with_active_sector_M4_insert_pilots", 1268.0),
    ("shared_axial_carrier_with_integral_radial_stops", "spindle",
     "stator", 439.504679),
    ("radial_slide_7075:mid", "spindle", "stator", 119.922751),
    ("shared_axial_carrier_with_integral_radial_stops", "spindle",
     "front_one_solid_PEEK_cap_with_short_open_leadins", 113.910490),
    ("radial_slide_7075:mid", "spindle",
     "front_one_solid_PEEK_cap_with_short_open_leadins", 82.932047),
    ("shared_axial_carrier_with_integral_radial_stops", "carriage",
     "rear_M0_following_M1_static_PEEK_active_sector", 39.598794),
    ("virgin_unfilled_PEEK_R3_open_groove_nose", "carriage",
     "front_M0_following_M1_static_PEEK_active_sector", 39.568910),
    ("context_exact_keyed_tower_face_M4_insert_pilots", "carriage",
     "M0_carriage_owned_aluminum_active_sector_split_yoke", 34.8),
    ("outer_gimbal_yoke_6061_axis_Y", "carriage",
     "front_M0_following_M1_static_PEEK_active_sector", 17.197226),
    ("tangential_slide_7075:mid:center", "spindle",
     "front_one_solid_PEEK_cap_with_short_open_leadins", 16.423344),
    ("shared_axial_carrier_with_integral_radial_stops", "spindle",
     "cap_retention_iso4762_M2x20_0", 4.810486),
    ("radial_return_bellcrank_ratio_0p29", "spindle",
     "front_one_solid_PEEK_cap_with_short_open_leadins", 4.299776),
    ("context_exact_keyed_tower_face_M4_insert_pilots", "carriage",
     "active_sector_tower_M4_heat_insert_NX_60Z", 3.277842),
    ("context_exact_keyed_tower_face_M4_insert_pilots", "carriage",
     "active_sector_tower_M4_heat_insert_PX_60Z", 3.277842),
    ("context_exact_keyed_tower_face_M4_insert_pilots", "carriage",
     "active_sector_tower_M4_heat_insert_NX_66Z", 3.277837),
    ("context_exact_keyed_tower_face_M4_insert_pilots", "carriage",
     "active_sector_tower_M4_heat_insert_PX_66Z", 3.277837),
    ("shared_axial_carrier_with_integral_radial_stops", "spindle",
     "cap_retention_front_washer_0", 1.712385),
    ("inner_gimbal_yoke_6061_axis_Z", "carriage",
     "front_M0_following_M1_static_PEEK_active_sector", 1.477158),
    ("outer_gimbal_yoke_6061_axis_Y", "spindle",
     "front_one_solid_PEEK_cap_with_short_open_leadins", 0.825713),
)


COARSE_M2_DIAGNOSTIC = {
    "M0_rad": DEEP_DIAGNOSTIC_M0_RAD,
    "radial_state": REFERENCE_RADIAL_STATE,
    "tangential_state": REFERENCE_TANGENTIAL_STATE,
    "samples": [
        {
            "M2_deg": 0.0,
            "positive_pair_count": 3,
            "positive_pairs": [
                ["shared_axial_carrier_with_integral_radial_stops",
                 "torus_free_retained_arm_with_open_PEEK_cradle_seat_one_solid",
                 152.385609],
                ["shared_axial_carrier_with_integral_radial_stops",
                 "front_left_printed_retainer_face_boss_three_point_spacer_single_solid",
                 25.204927],
                ["shared_axial_carrier_with_integral_radial_stops",
                 "front_right_printed_retainer_face_boss_three_point_spacer_single_solid",
                 23.312072],
            ],
        },
        {"M2_deg": 90.0, "positive_pair_count": 0,
         "positive_pairs": []},
        {
            "M2_deg": 180.0,
            "positive_pair_count": 10,
            "positive_pairs": [
                ["shared_axial_carrier_with_integral_radial_stops",
                 "torus_free_retained_arm_with_open_PEEK_cradle_seat_one_solid",
                 114.913428],
                ["shared_axial_carrier_with_integral_radial_stops",
                 "one_piece_polished_unfilled_PEEK_shaft_to_tip_guide",
                 105.094042],
                ["shared_axial_carrier_with_integral_radial_stops",
                 "front_balance_B777_annular_slug_x-3.6_t1.502168", 20.683615],
                ["shared_axial_carrier_with_integral_radial_stops",
                 "front_balance_B777_annular_slug_x+3.6_t1.502168", 20.683615],
                ["shared_axial_carrier_with_integral_radial_stops",
                 "front_balance_ISO4762_M2x8_x-3.6", 19.759580],
                ["shared_axial_carrier_with_integral_radial_stops",
                 "front_balance_ISO4762_M2x8_x+3.6", 19.759580],
                ["shared_axial_carrier_with_integral_radial_stops",
                 "front_balance_M2_washer_x-3.6", 3.166816],
                ["shared_axial_carrier_with_integral_radial_stops",
                 "front_balance_M2_washer_x+3.6", 3.166816],
                ["shared_axial_carrier_with_integral_radial_stops",
                 "front_balance_M2_heat_insert_x+3.6", 2.490078],
                ["shared_axial_carrier_with_integral_radial_stops",
                 "front_balance_M2_heat_insert_x-3.6", 2.490067],
            ],
        },
        {"M2_deg": 270.0, "positive_pair_count": 0,
         "positive_pairs": []},
    ],
    "deep_M0_static_AABB_candidate_count": 0,
    "limitations": [
        "four diagnostic M2 phases are not a continuous sweep",
        "zero positive overlap at 90/270 degrees is not a distance result",
        "the four samples are not the missing four-shoe identity map",
        "only the mid/center follower state was tested",
        "no M1 indexed or transition sweep completed",
    ],
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(value: Mapping[str, Any]) -> str:
    body = deepcopy(dict(value))
    body.pop("report_sha256", None)
    return hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def _common_volume(one, two) -> float:
    common = one & two
    return 0.0 if common is None else float(common.volume)


def _bbox(shape) -> list[float]:
    bounds = shape.bounding_box()
    return [float(value) for value in (
        bounds.min.X, bounds.max.X,
        bounds.min.Y, bounds.max.Y,
        bounds.min.Z, bounds.max.Z,
    )]


def _live_decisive_witnesses() -> dict[str, Any]:
    carrier = guide.to_machine_reference(follower.carrier())
    tower = guide.revised_spindle_tower()
    existing_yoke = guide.to_machine_reference(guide.carriage_yoke())
    context = guide.to_machine_reference(follower.mounting_backer_context())

    ctr = (Align.CENTER, Align.CENTER, Align.CENTER)
    spine = guide.to_machine_reference(
        Pos(28.0, 0.0, -51.0) * Box(6.0, 8.0, 119.0, align=ctr)
    )
    adapter = guide.to_machine_reference(
        Pos(32.0, 0.0, -112.0) * Box(12.0, 56.0, 4.0, align=ctr)
    )
    return {
        "carrier_machine_bbox_xyz_minmax_mm": _bbox(carrier),
        "tower_machine_bbox_xyz_minmax_mm": _bbox(tower),
        "existing_yoke_machine_bbox_xyz_minmax_mm": _bbox(existing_yoke),
        "authored_spine_machine_bbox_xyz_minmax_mm": _bbox(spine),
        "authored_spine_length_mm": 119.0,
        "full_STEP_machine_Y_extent_mm": 151.0,
        "full_STEP_extent_is_not_spine_length": True,
        "carrier_vs_revised_tower_common_mm3": _common_volume(carrier, tower),
        "carrier_vs_existing_yoke_common_mm3": _common_volume(
            carrier, existing_yoke
        ),
        "context_vs_revised_tower_common_mm3": _common_volume(context, tower),
        "spine_vs_revised_tower_common_mm3": _common_volume(spine, tower),
        "adapter_vs_revised_tower_common_mm3": _common_volume(adapter, tower),
    }


def _load_existing_collision_contract() -> dict[str, Any]:
    report = json.loads(EXISTING_COLLISION_REPORT.read_text(encoding="utf-8"))
    full = report["full_raw_rigid_motion"]
    outboard = report["outboard_yoke_packaging"]
    m2 = report["front_plane_yoke_full_M2_clearance"]
    return {
        "path": "out/reports/carriage_active_sector_terminal_guide_audit.json",
        "sha256": _sha256(EXISTING_COLLISION_REPORT),
        "schema": report["schema"],
        "status": report["status"],
        "existing_yoke_full_raw_sample_count": full["sample_count"],
        "existing_yoke_full_raw_status": full["status"],
        "existing_yoke_static_frame_minimum_clearance_mm": outboard[
            "static_frame_full_M0"
        ]["minimum_clearance_mm"],
        "existing_yoke_full_M2_minimum_clearance_mm": m2[
            "minimum_clearance_mm"
        ],
        "transferable_to_follower": False,
        "reason": (
            "the follower carrier lacks the current yoke U-window and has a "
            "different positive-volume envelope"
        ),
    }


def analyze() -> dict[str, Any]:
    contract = follower.geometry_contract()
    witnesses = _live_decisive_witnesses()
    existing = _load_existing_collision_contract()
    axis_z_deep = float(PARAMS.stator_axis_z(DEEP_DIAGNOSTIC_M0_RAD))

    physical_owner_map_absent = (
        not hasattr(follower, "selected_occurrences")
        and not hasattr(follower, "selected_follower_occurrences")
        and not hasattr(follower, "m2_identity_transform")
    )
    exact_transform = {
        "active_local_frame": {
            "+X": "radial outward",
            "+Y": "tangential",
            "+Z": "stator axis",
        },
        "reference_formula": (
            "machine(x,y,z)=(-local_y, local_z, 95-local_x)"
        ),
        "carriage_owned_pose_formula": (
            "world=(-local_y, local_z, stator_axis_z(M0)-local_x)"
        ),
        "stator_axis_z_formula": "95 + M0_rad*(8/(2*pi))",
        "m0_home_standoff_mm": float(PARAMS.m0_home_standoff),
        "m0_mm_per_rad": float(PARAMS.mm_per_rad),
        "owner": "carriage",
        "M1_effect_on_carriage_owned_follower": "none",
        "M2_effect_on_carriage_owned_follower": "none",
        "source_functions": [
            "carriage_active_sector_terminal_guide.carriage_local_to_machine_reference",
            "carriage_active_sector_terminal_guide.to_machine_reference",
            "assembly.link_location",
            "params.WinderParams.stator_axis_z",
        ],
    }

    rows = [{
        "follower_label": follower_label,
        "assembly_owner": owner,
        "assembly_label": assembly_label,
        "positive_common_volume_mm3": volume,
    } for follower_label, owner, assembly_label, volume
        in REFERENCE_POSITIVE_PAIRS]

    gates = {
        "exact_carriage_owned_transform_source_bound": True,
        "physical_M1_M2_selected_occurrence_owner_map_defined": (
            not physical_owner_map_absent
        ),
        "reference_pose_has_zero_positive_pairs": len(rows) == 0,
        "carrier_clears_revised_tower": math.isclose(
            witnesses["carrier_vs_revised_tower_common_mm3"], 0.0,
            abs_tol=1.0e-7,
        ),
        "carrier_clears_existing_active_sector_yoke": math.isclose(
            witnesses["carrier_vs_existing_yoke_common_mm3"], 0.0,
            abs_tol=1.0e-7,
        ),
        "coarse_M2_diagnostic_is_continuous_clearance_sweep": False,
        "existing_yoke_collision_report_transfers_to_follower": False,
        "additive_integration_feasible": False,
    }

    source_hashes = {
        str(path).replace("\\", "/"): _sha256(ROOT / path)
        for path in SOURCE_PATHS
    }
    artifacts = {
        "follower_STEP": {
            "path": "out/review/aggregate_boundary_floating_follower.step",
            "byte_count": FOLLOWER_STEP.stat().st_size,
            "sha256": _sha256(FOLLOWER_STEP),
        },
        "integrated_release_candidate_STEP": {
            "path": "out/review/integrated_release_candidate.step",
            "byte_count": ASSEMBLY_STEP.stat().st_size,
            "sha256": _sha256(ASSEMBLY_STEP),
        },
    }

    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "FAIL",
        "decision": (
            "ADDITIVE_INTEGRATION_REJECTED__REPLACEMENT_REDESIGN_REQUIRED"
        ),
        "additive_integration_feasible": False,
        "clearance_claimed": False,
        "assembly_integration_authorized": False,
        "collision_authorized": False,
        "wire_route_authorized": False,
        "production_authorized": False,
        "selected_release_modified": False,
        "scope": {
            "read_only": True,
            "reference_radial_state": REFERENCE_RADIAL_STATE,
            "reference_tangential_state": REFERENCE_TANGENTIAL_STATE,
            "reference_M0_rad": REFERENCE_M0_RAD,
            "reference_M1_deg": REFERENCE_M1_DEG,
            "reference_M2_deg": REFERENCE_M2_DEG,
            "proved": [
                "exact carriage-owned active-local transform",
                "21 exact reference-pose positive-volume pairs",
                "decisive carrier/tower and carrier/current-yoke intersections",
                "coarse deepest-M0 M2 intersections at 0 and 180 degrees",
            ],
            "not_proved": [
                "M1/M2 selected physical occurrence mapping",
                "continuous M0/M1/M2 or slide/gimbal collision clearance",
                "minimum distance at zero-overlap diagnostics",
                "wire route, release, extraction, or production feasibility",
            ],
        },
        "exact_transform": exact_transform,
        "selection_owner_map": {
            "M1_law_count": contract["selection_and_retraction"][
                "M1_law_count"
            ],
            "M2_identity_count": contract["selection_and_retraction"][
                "M2_selected_shoe_identity_count"
            ],
            "physical_occurrence_owner_map_absent": physical_owner_map_absent,
            "M1_M2_values_are_counts_not_placements": True,
            "required_api": (
                "selected_occurrences(M1_law, M2_identity, M0_gate_state)"
            ),
        },
        "reference_pose_OCC_scan": {
            "method": (
                "exact build123d/OpenCascade common volume after AABB prefilter; "
                "follower custom bodies versus integrated_release_candidate.build_links"
            ),
            "bound_to_current_source_hashes": True,
            "positive_pair_count": len(rows),
            "positive_pairs": rows,
            "principal": {
                "carrier_vs_existing_yoke_mm3": 1782.698995,
                "carrier_vs_revised_tower_mm3": 1268.0,
                "carrier_vs_stator_mm3": 439.504679,
                "carrier_vs_front_cap_mm3": 113.910490,
            },
        },
        "live_decisive_OCC_witnesses": witnesses,
        "coarse_M2_diagnostic": {
            **deepcopy(COARSE_M2_DIAGNOSTIC),
            "axis_z_mm": axis_z_deep,
            "clearance_claimed": False,
        },
        "existing_collision_contract": existing,
        "gates": gates,
        "minimum_implementation_plan": [
            "Treat the follower as a replacement for the current active-sector yoke/guides, not an additive module.",
            "Exclude mounting_backer_context, exploded blocker envelopes, and duplicate M4 occurrences from installable parts.",
            "Add the current U-window around the tower bearing bridge and shorten or reroute the central spine until exact reference intersections are zero.",
            "Define selected_occurrences(M1_law, M2_identity, M0_gate_state) with one explicit owner and transform per physical shoe.",
            "Replace the old active-sector occurrences exactly once in integrated_release_candidate.carriage_module_parts().",
            "Reuse per-occurrence meshes, _ReusableFCLPair, and the 225775-pose stream for all nine slide endpoints plus continuous slide/gimbal/M0/M1/M2 transitions.",
        ],
        "artifacts": artifacts,
        "source_hashes": source_hashes,
    }
    report["report_sha256"] = _canonical_hash(report)
    validate_report_integrity(report)
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported follower integration audit schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("follower integration audit report hash mismatch")
    if report.get("status") != "FAIL":
        raise ValueError("follower integration audit must fail closed")
    if report.get("additive_integration_feasible") is not False:
        raise ValueError("colliding follower promoted to additive integration")
    if report.get("clearance_claimed") is not False:
        raise ValueError("bounded diagnostics were promoted to clearance")
    for key in (
        "assembly_integration_authorized", "collision_authorized",
        "wire_route_authorized", "production_authorized",
        "selected_release_modified",
    ):
        if report.get(key) is not False:
            raise ValueError(f"integration audit invented {key}")
    scan = report.get("reference_pose_OCC_scan", {})
    if scan.get("positive_pair_count") != 21:
        raise ValueError("reference positive-pair witness count drift")
    live = report.get("live_decisive_OCC_witnesses", {})
    expected = {
        "carrier_vs_revised_tower_common_mm3": 1268.0,
        "carrier_vs_existing_yoke_common_mm3": 1782.6989950592267,
        "context_vs_revised_tower_common_mm3": 3398.7459281092056,
        "spine_vs_revised_tower_common_mm3": 740.0,
        "adapter_vs_revised_tower_common_mm3": 544.0,
    }
    for key, value in expected.items():
        if not math.isclose(float(live.get(key, math.nan)), value,
                            rel_tol=0.0, abs_tol=1.0e-6):
            raise ValueError(f"decisive OCC witness drift: {key}")
    selection = report.get("selection_owner_map", {})
    if selection.get("physical_occurrence_owner_map_absent") is not True:
        raise ValueError("missing selection owner map was invented")
    gates = report.get("gates", {})
    if gates.get("additive_integration_feasible") is not False:
        raise ValueError("additive integration gate drift")
    for relative, expected_hash in report.get("source_hashes", {}).items():
        path = ROOT / Path(str(relative).replace("\\", "/"))
        if not path.is_file() or _sha256(path) != expected_hash:
            raise ValueError(f"source hash drift: {relative}")
    for artifact in report.get("artifacts", {}).values():
        path = ROOT / Path(str(artifact["path"]).replace("\\", "/"))
        if (not path.is_file()
                or path.stat().st_size != artifact["byte_count"]
                or _sha256(path) != artifact["sha256"]):
            raise ValueError(f"artifact hash drift: {artifact['path']}")
    existing = report.get("existing_collision_contract", {})
    if existing.get("sha256") != _sha256(EXISTING_COLLISION_REPORT):
        raise ValueError("existing collision contract hash drift")
    if existing.get("transferable_to_follower") is not False:
        raise ValueError("existing-yoke clearance was transferred to follower")


def render_markdown(report: Mapping[str, Any]) -> str:
    transform = report["exact_transform"]
    scan = report["reference_pose_OCC_scan"]
    live = report["live_decisive_OCC_witnesses"]
    m2 = report["coarse_M2_diagnostic"]
    lines = [
        "# Aggregate-boundary follower integration audit", "",
        f"**{report['status']} — {report['decision']}**", "",
        "The isolated follower cannot be added to the current keyed-tower "
        "assembly. This is a fail-closed integration result, not a clearance "
        "or production decision.", "",
        "## Exact owner transform", "",
        f"- Reference: `{transform['reference_formula']}`.",
        f"- M0 carriage pose: `{transform['carriage_owned_pose_formula']}`.",
        "- M1 and M2 do not transform a carriage-owned occurrence.",
        "- The three M1 laws and four M2 identities have no physical occurrence/owner map.", "",
        "## Reference-pose exact intersections", "",
        f"The bounded OCC scan found **{scan['positive_pair_count']}** positive-volume pairs.", "",
        f"- Carrier vs current yoke: {scan['principal']['carrier_vs_existing_yoke_mm3']:.6f} mm3.",
        f"- Carrier vs revised tower: {scan['principal']['carrier_vs_revised_tower_mm3']:.6f} mm3.",
        f"- Carrier vs stator: {scan['principal']['carrier_vs_stator_mm3']:.6f} mm3.",
        f"- Carrier vs front cap: {scan['principal']['carrier_vs_front_cap_mm3']:.6f} mm3.", "",
        "## 151 mm envelope and tower", "",
        f"The 151 mm value is the full STEP machine-Y extent, not the "
        f"{live['authored_spine_length_mm']:.0f} mm authored spine length. "
        f"The spine intersects the revised tower by "
        f"{live['spine_vs_revised_tower_common_mm3']:.3f} mm3; the adapter "
        f"intersects it by {live['adapter_vs_revised_tower_common_mm3']:.3f} mm3.", "",
        "## Coarse M2 diagnostic", "",
        f"At M0={m2['M0_rad']:.3f} rad (axis Z={m2['axis_z_mm']:.6f} mm), "
        "the carrier intersects the final flyer at 0 and 180 degrees. "
        "The zero-overlap 90/270-degree probes are not clearance evidence.", "",
        "## Minimum implementation plan", "",
    ]
    lines.extend(
        f"{index}. {step}"
        for index, step in enumerate(report["minimum_implementation_plan"], 1)
    )
    lines.extend([
        "", "## Authority boundary", "",
        "Assembly integration, collision clearance, wire-route validity, "
        "release, and production remain unauthorized.", "",
        f"Follower STEP SHA-256: `{report['artifacts']['follower_STEP']['sha256']}`", "",
        f"Assembly STEP SHA-256: `{report['artifacts']['integrated_release_candidate_STEP']['sha256']}`", "",
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
        f"follower integration audit {report['status']}: "
        f"pairs={report['reference_pose_OCC_scan']['positive_pair_count']}; "
        f"additive={report['additive_integration_feasible']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
