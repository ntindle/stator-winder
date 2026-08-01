"""Fail-closed source-level integration candidate for the normal GOAL.

CAD brief
---------
Model
    Full reference-pose machine assembly composed from the current frame,
    carriage and spindle sources, the physical PEEK cap pair, the retained
    six-stack balanced offset flyer with one-piece PEEK guide/bell, and the
    exact-1:1 NEMA17/P30/3GT drive review.
Frame
    Existing machine millimetre frame.  M0=M1=M2=0 is the exported pose;
    +Z is toward the work and the M2 axis is world Z.
Authoritative transforms
    The retained architecture rigidly translates the complete M2 bearing and
    drive module -10 mm in Z while the arm front datum remains fixed.  The
    entry bracket/eyelet and its mounting hardware translate -4.25 mm in Z;
    an integral keeper preserves the existing dancer-spring screw datum.  The
    fixed-length base-rail window translates -7.5 mm in Z to retain 2.3 mm
    behind the exact 83.2 mm Leadshine motor body.  The spring-loaded felt stack advances 0.27648 mm
    for the configured 0.22352 mm wire operating-contact state.
Outputs
    ``out/review/integrated_release_candidate.step`` plus JSON, Markdown and
    manifest artifacts.  ``cad/assembly.py`` is deliberately untouched.
Validation boundary
    Exact BREP checks cover every new/replaced cross-module interface at the
    reference pose and the actual PEEK caps at the deepest M0 pose.  Existing
    unchanged hardware remains bound to its source occurrence set.  The final
    per-occurrence rigid sweep covers the complete raw command cycle, and the
    exact active terminal route covers all 2,400 deposition loci.  Flexible
    park/index/load/unload conductor geometry, wire dynamics and physical
    procurement/qualification gates remain open.

This is intentionally not production-authorized.  It makes the current
interfaces inspectable in one assembly without laundering review geometry or
open supplier/physical gates into a release.
"""

from __future__ import annotations

from copy import deepcopy
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np
from build123d import (
    Align,
    AngularDirection,
    BuildSketch,
    Circle,
    Compound,
    Cone,
    Cylinder,
    Edge,
    GeomType,
    Part,
    Plane,
    Pos,
    Rot,
    Transition,
    Wire,
    sweep,
)

import assembly
import cots
import custom_parts
import flyer_shaft_d10
from params import DEFAULT_STATOR, PARAMS as P
import permanent_cap_offset_spoke_retained_review as retained
import permanent_cap_production_review as caps
import retained_flyer_peek_guide_successor as flyer_successor
import carriage_active_sector_terminal_guide as terminal_guide
import m2_drive_successor_review as drive
import leadshine_cs_m21708_cableless as leadshine
import nbk_p30_official_occurrence as nbk_p30
import nbk_p30_d10_official_occurrence as nbk_p30_d10
import wire_geometry
import wire_vis


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORTS = ROOT / "out" / "reports"
REVIEW = ROOT / "out" / "review"

SOURCE = HERE / "integrated_release_candidate.py"
STEP_OUT = REVIEW / "integrated_release_candidate.step"
JSON_OUT = REPORTS / "integrated_release_candidate.json"
MD_OUT = REPORTS / "integrated_release_candidate.md"
MANIFEST_OUT = REVIEW / "integrated_release_candidate.manifest.json"

RETAINED_REPORT = REPORTS / "permanent_cap_offset_spoke_retained_review.json"
CAP_REPORT = REPORTS / "permanent_cap_production_review.json"
DRIVE_REPORT = REPORTS / "m2_drive_successor_review.json"
MAIN_VALIDATION_REPORT = REPORTS / "validation.json"
FRAME_HARDWARE_REPORT = HERE / "frame_hardware_audit.report.json"
CARRIAGE_HARDWARE_REPORT = HERE / "carriage_hardware_audit.report.json"
HARDWARE_RELEASE_AUDIT = REPORTS / "hardware_release_audit.json"
LEADSHINE_REPORT = REPORTS / "leadshine_cs_m21708_cableless.json"
M2_SELECTION_REPORT = REPORTS / "m2_normal_goal_drive_selection.json"
M2_SELECTION_SOURCE = ROOT / "sim" / "m2_normal_goal_drive_selection.py"
FELT_LOADS_REPORT = REPORTS / "felt_loads.json"
FELT_LOADS_SOURCE = HERE / "felt_loads.py"
FELT_CONTACT_REPORT = REPORTS / "integrated_felt_contact_review.json"
FELT_CONTACT_SOURCE = HERE / "integrated_felt_contact_review.py"
FLYER_GUIDE_REPORT = REPORTS / "retained_flyer_peek_guide_successor.json"
FLYER_GUIDE_MANIFEST = (
    REVIEW / "retained_flyer_peek_guide_successor.manifest.json"
)
ACTIVE_SECTOR_REPORT = (
    REPORTS / "carriage_active_sector_terminal_guide_audit.json"
)
ACTIVE_SECTOR_LOCI = (
    REPORTS / "carriage_active_sector_terminal_guide_loci.json"
)
ACTIVE_SECTOR_STEP = (
    REVIEW / "carriage_active_sector_terminal_guide.step"
)
ACTIVE_SECTOR_MANIFEST = (
    REVIEW / "carriage_active_sector_terminal_guide.manifest.json"
)
ACTIVE_SECTOR_AUDIT_SOURCE = (
    ROOT / "sim" / "carriage_active_sector_terminal_guide_audit.py"
)
ACTIVE_SECTOR_GUIDE_SOURCE = (
    HERE / "carriage_active_sector_terminal_guide.py"
)
BASE_RAW_CLEARANCE_REPORT = (
    REPORTS / "integrated_candidate_base_clearance_raw.json"
)
BASE_RAW_PARTS_REPORT = (
    REPORTS / "integrated_candidate_base_clearance_raw_parts.json"
)
RELEASE_CATALOG = HERE / "release_catalog.json"
CUSTOM_PARTS_SOURCE = HERE / "custom_parts.py"
CUSTOM_PARTS_MANIFEST = ROOT / "out" / "custom" / "manifest.json"
RELEASED_SHAFT_STEP = (
    ROOT / "out" / "custom" / "step" /
    "flyer_shaft_d10_id6_to_id9_l79.step"
)
RELEASED_SHAFT_PDF = (
    ROOT / "output" / "pdf" / "flyer_shaft_d10_id6_to_id9_l79.pdf"
)
RETIRED_REV_C_SHAFT_STEP = (
    ROOT / "out" / "custom" / "step" /
    "flyer_shaft_d10_id6_to_id9_l80p75.step"
)
RETIRED_REV_C_SHAFT_PDF = (
    ROOT / "output" / "pdf" /
    "flyer_shaft_d10_id6_to_id9_l80p75.pdf"
)

SCHEMA = "integrated-release-candidate/v1"
MANIFEST_SCHEMA = "integrated-release-candidate-manifest/v1"
COLLISION_GEOMETRY_REVISION = (
    "active-sector-r39p2__physical-bell-root-sleeve-six-slug__"
    "L79-stock-D10-P30__short-cap-leadins__"
    "rev6-front-plane-outboard-coil-bypass-yoke"
)
BOOLEAN_TOL_MM3 = 1.0e-5
CONTACT_TOL_MM = 1.0e-5
MIN_REVIEW_CLEARANCE_MM = 2.2
M2_REAR_SHIFT_MM = retained.base.M2_MODULE_REAR_SHIFT_MM
ENTRY_REAR_SHIFT_MM = 4.25
# Frozen immediately preceding integrated-candidate placement.  The upstream
# retained review used an earlier 2.0 mm shift and is not this comparison.
ENTRY_PRIOR_REAR_SHIFT_MM = ENTRY_REAR_SHIFT_MM - 0.75
ENTRY_ADDITIONAL_REAR_SHIFT_MM = (
    ENTRY_REAR_SHIFT_MM - ENTRY_PRIOR_REAR_SHIFT_MM
)
ENTRY_PASSAGE_RADIUS_MM = 1.6
FRAME_WINDOW_REAR_SHIFT_MM = retained.base.FRAME_WINDOW_REAR_SHIFT_MM
INTEGRATED_FRAME_WINDOW_REAR_SHIFT_MM = 7.5
FELT_MAX_GAP_MM = 2.0 * wire_geometry.WIRE_RADIUS_MAX
FELT_MOVING_STACK_TRAVEL_MM = FELT_MAX_GAP_MM - DEFAULT_STATOR.wire_d
FELT_FIXED_CONTACT_Z_MM = P.rear_post_z + 22.75
CONFIGURED_WIRE_PLANE_Z_MM = FELT_FIXED_CONTACT_Z_MM + wire_vis.R_VIS
NBK_P30_STOCK_ROLL_DEG = 45.0
NBK_P30_BNW_FIRST_AZIMUTH_DEG = 0.0
NBK_P30_BNW_LOCAL_X_MM = nbk_p30.BNW_WITNESS_DEFAULT_LOCAL_X_MM
NBK_P30_BNW_SCREW_INWARD_ADJUSTMENTS_MM = (0.0, 0.5)
RELEASED_SHAFT_CENTER_Z_MM = flyer_shaft_d10.WORLD_CENTER_Z_MM
ARM_M3X8_SCREW_INWARD_ADJUSTMENT_MM = 0.3
ARM_SHAFT_BORE_DIAMETER_MM = 12.10
ARM_SHAFT_BORE_RADIAL_CLEARANCE_MM = (
    ARM_SHAFT_BORE_DIAMETER_MM - 12.0
) / 2.0
ARM_SHAFT_BORE_CUT_Z_MM = (-55.0, -29.0)
ARM_SHAFT_ROOT_WEB_OUTER_DIAMETER_MM = 18.0
ARM_SHAFT_ROOT_WEB_Z_MM = (-43.0, -31.5)
ARM_ROOT_LOAD_TENSION_N = 10.0
ARM_ROOT_LOAD_RADIUS_MM = 64.0
ARM_ROOT_PETG_REVIEW_ALLOWABLE_MPA = 10.0
ARM_ROOT_REVIEW_SAFETY_FACTOR = 3.0
ARM_SHAFT_BORE_REAM_TOLERANCE_MM = (12.10, 12.13)
RELEASED_SHAFT_OD_TOLERANCE_MM = (11.98, 12.00)
RELEASED_SHAFT_D10_SEAT_TOLERANCE = "ISO 286 h6"
MIN_SHAFT_FRONT_SETBACK_FROM_ROOT_SLEEVE_MM = 0.25
CTR = (Align.CENTER, Align.CENTER, Align.CENTER)


STATIC_M2_BODY_LABELS = {
    "flyer_block",
    "flyer_6001_front",
    "flyer_6001_rear",
    "m2_outer_race_spacer",
    "m2_din472_28",
}
OLD_STATIC_M2_REPLACED_LABELS = {
    "m2_motor_mount",
    "m2_motor",
    "m2_motor_pulley",
    "gt2_belt",
}
OLD_FLYER_REPLACED_LABELS = {
    "alu_tube",
    "flyer_arm",
    "flyer_pulley",
    "wire_elbow",
    "tip_toroid_guide",
    "m2_inner_rear_shim",
    "m2_inner_center_spacer",
    "m2_inner_front_spacer",
    "flyer_arm_neg_y_m3x8",
    "flyer_arm_neg_y_m3_insert",
    "flyer_arm_pos_x_m3x8",
    "flyer_arm_pos_x_m3_insert",
    "flyer_pulley_m3x8",
    "flyer_pulley_m3_insert",
    "counterweight_m3_insert",
    "counterweight_m3x12",
    "counterweight_washer_m3_1",
    "counterweight_washer_m3_2",
    "counterweight_washer_m3_3",
}

FELT_MOVING_STACK_LABELS = {
    "felt_pad_moving",
    "felt_backing_moving",
    "felt_compression_spring",
    "felt_spring_thrust_washer",
    "felt_m4_wingnut",
}


def _sha256(path: Path) -> str | None:
    if not path.exists():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} does not contain a JSON object")
    return value


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    body = deepcopy(dict(payload))
    body.pop("report_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _locus_payload_hash(payload: Mapping[str, Any]) -> str:
    """Hash a locus payload before its own digest field is attached."""

    body = deepcopy(dict(payload))
    body.pop("locus_payload_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def integrated_snapshot_packet() -> list[str]:
    """Return only a render packet made after the current primary STEP."""

    snapshot_dir = REVIEW / "snapshots"
    native_glb = REVIEW / ".integrated_release_candidate.step.glb"
    if not STEP_OUT.is_file() or not native_glb.is_file():
        return []
    step_mtime_ns = STEP_OUT.stat().st_mtime_ns
    if native_glb.stat().st_mtime_ns < step_mtime_ns:
        return []
    stems = (
        "integrated_release_candidate_iso",
        "integrated_release_candidate_iso_opposite",
        "integrated_release_candidate_top",
        "integrated_release_candidate_front",
    )
    result: list[str] = []
    for stem in stems:
        matches = sorted(snapshot_dir.glob(f"{stem}_[0-9]*.png"))
        matches = [
            path for path in matches
            if path.stat().st_mtime_ns >= step_mtime_ns
        ]
        if matches:
            result.append(
                str(matches[-1].relative_to(ROOT)).replace("\\", "/")
            )
    return result


def _moved(shape: Part | Compound, dz: float, label: str | None = None):
    result = Pos(0.0, 0.0, dz) * shape
    result.label = label or getattr(shape, "label", "part")
    return result


def _compound(parts: Iterable[Part | Compound], label: str) -> Compound:
    result = Compound(children=list(parts))
    result.label = label
    return result


def _bbox(shape: Part | Compound) -> dict[str, list[float]]:
    box = shape.bounding_box()
    return {
        "minimum_mm": [float(box.min.X), float(box.min.Y), float(box.min.Z)],
        "maximum_mm": [float(box.max.X), float(box.max.Y), float(box.max.Z)],
        "size_mm": [float(box.size.X), float(box.size.Y), float(box.size.Z)],
    }


def _overlap(a: Part | Compound, b: Part | Compound) -> float:
    common = a & b
    return 0.0 if common is None else float(common.volume)


def _distance(a: Part | Compound, b: Part | Compound) -> float:
    return float(a.distance_to(b))


def _validate_active_sector_contract(
    report: Mapping[str, Any], locus_payload: Mapping[str, Any],
) -> None:
    """Fail closed on the final rigid/terminal/player audit authority."""

    if not (
        report.get("schema")
        == "carriage-active-sector-terminal-guide-audit/v1"
        and report.get("status")
        == (
            "GEOMETRY_TERMINAL_TOLERANCE_PASS_REVIEW_ONLY__"
            "CONTINUOUS_CONDUCTOR_AND_PHYSICAL_GATES_FAIL"
        )
        and report.get("production_authorized") is False
        and report.get("assembly_geometry_integration_authorized") is True
        and report.get("report_sha256") == _canonical_hash(report)
    ):
        raise ValueError("active-sector audit schema/status/hash is not final")

    gates = report.get("release_gates", {})
    required_true = (
        "M0_only_periodic_equivalence_all_2400",
        "all_2400_physical_bell_terminal_routes_pass",
        "minimum_and_maximum_wire_bell_extremes_pass",
        "per_occurrence_collision_meshes_closed",
        "deposition_exact_rigid_pairs_clear_ge_2mm",
        "arbitrary_M1_caps_and_copper_clear_ge_2mm",
        "summed_tolerance_and_10N_deflection_clearance_ge_2mm",
        "guide_yoke_tower_attachment_chain",
        "yoke_full_section_10N_stress_screen",
        "short_leadin_C_section_10N_screen",
        "M2_exact_live_line_Leadshine_36V_margin_ge_2x",
        "M2_P30_210_3GT_capacity_ge_2x",
        "M1_wrap_governed_margin_ge_2x",
        "M0_terminal_force_and_added_mass_margin_ge_2x",
        "both_raw_wrap_wire_paths_bypass_fixed_guide_yoke",
        "full_raw_rigid_sweep_clear",
        "outboard_yoke_full_M0_carriage_and_static_packaging_clear",
        "front_plane_yoke_full_M2_final_flyer_clear_ge_2mm",
    )
    required_false = (
        "M2_36V_driver_configuration_verified",
        "M2_installed_hot_dyno_verified",
        "M1_closed_loop_drive_fault_safe_behavior_verified",
        "both_raw_shaft_wraps_exactly_two_turns",
        "park_index_load_unload_continuous_conductor_proven",
        "PEEK_forming_gauge_polish_abrasion_coupon",
        "M3_M4_insert_pull_and_endurance_coupon",
        "production_authorized",
    )
    missing_true = [name for name in required_true if gates.get(name) is not True]
    missing_false = [
        name for name in required_false if gates.get(name) is not False
    ]
    if missing_true or missing_false:
        raise ValueError(
            "active-sector release-gate drift: "
            f"true={missing_true}, false={missing_false}"
        )

    source_hashes = report.get("source_hashes", {})
    expected_source_hashes = {
        "cad/carriage_active_sector_terminal_guide.py": _sha256(
            ACTIVE_SECTOR_GUIDE_SOURCE
        ),
        "cad/retained_flyer_peek_guide_successor.py": _sha256(
            Path(flyer_successor.__file__)
        ),
        "cad/integrated_release_candidate.py": _sha256(SOURCE),
        "sim/carriage_active_sector_terminal_guide_audit.py": _sha256(
            ACTIVE_SECTOR_AUDIT_SOURCE
        ),
    }
    if any(
        source_hashes.get(path) != digest
        for path, digest in expected_source_hashes.items()
    ):
        raise ValueError("active-sector audit source hash drift")

    collision = report.get("collision_authority", {})
    if collision.get("geometry_revision") != COLLISION_GEOMETRY_REVISION:
        raise ValueError("active-sector collision geometry revision drift")
    for name in ("deposition_rigid_collision", "arbitrary_m1_clearance"):
        row = report.get("collision_cache_provenance", {}).get(name, {})
        cache_path = ROOT / str(row.get("path", ""))
        if not (
            row.get("cache_key")
            and cache_path.is_file()
            and row.get("file_sha256") == _sha256(cache_path)
        ):
            raise ValueError(f"active-sector collision cache drift: {name}")

    step = report.get("artifacts", {}).get("step", {})
    if not (
        step.get("path")
        == "out/review/carriage_active_sector_terminal_guide.step"
        and step.get("exists") is True
        and ACTIVE_SECTOR_STEP.is_file()
        and step.get("file_sha256") == _sha256(ACTIVE_SECTOR_STEP)
        and int(step.get("size_bytes", -1)) == ACTIVE_SECTOR_STEP.stat().st_size
    ):
        raise ValueError("active-sector STEP artifact is absent or stale")

    if not (
        locus_payload.get("schema")
        == "carriage-active-sector-terminal-guide-loci/v1"
        and locus_payload.get("run", {}).get("locus_count") == 2400
        and len(locus_payload.get("loci", [])) == 2400
        and locus_payload.get("locus_payload_sha256")
        == _locus_payload_hash(locus_payload)
    ):
        raise ValueError("active-sector player locus payload is stale or failed")
    flyer_reference = locus_payload.get("flyer_reference", {})
    if not (
        flyer_reference.get("conductor_prefix_point_count") == 175
        and len(flyer_reference.get(
            "geometric_bore_to_tensioned_handoff_local_samples_mm", []
        )) == 175
        and len(flyer_reference.get(
            "full_geometric_bore_local_samples_mm", []
        )) == 181
        and all(
            all(
                segment.get("name") != "flyer_geometric_bore"
                and bool(segment.get("machine_world_samples_mm"))
                for segment in locus.get("segments", [])
            )
            for locus in locus_payload.get("loci", [])
        )
    ):
        raise ValueError("active-sector shared flyer-prefix compaction drift")
    route = report.get("player_route_api", {})
    if not (
        route.get("schema") == locus_payload.get("schema")
        and route.get("locus_count") == 2400
        and route.get("canonical_payload_sha256")
        == locus_payload.get("locus_payload_sha256")
        and route.get("compact_file_sha256") == _sha256(ACTIVE_SECTOR_LOCI)
        and int(route.get("compact_size_bytes", -1))
        == ACTIVE_SECTOR_LOCI.stat().st_size
        and "torus" not in json.dumps(locus_payload).lower()
    ):
        raise ValueError("active-sector player route API/file drift")


@lru_cache(maxsize=1)
def _contract_reports() -> dict[str, dict[str, Any]]:
    retained_report = _load(RETAINED_REPORT)
    caps_report = _load(CAP_REPORT)
    drive_report = _load(DRIVE_REPORT)
    leadshine_report = _load(LEADSHINE_REPORT)
    m2_selection_report = _load(M2_SELECTION_REPORT)
    felt_loads_report = _load(FELT_LOADS_REPORT)
    felt_contact_report = _load(FELT_CONTACT_REPORT)
    flyer_guide_report = _load(FLYER_GUIDE_REPORT)
    active_sector_report = _load(ACTIVE_SECTOR_REPORT)
    active_sector_loci = _load(ACTIVE_SECTOR_LOCI)
    base_raw_report = _load(BASE_RAW_CLEARANCE_REPORT)
    base_raw_parts_report = _load(BASE_RAW_PARTS_REPORT)
    custom_manifest = _load(CUSTOM_PARTS_MANIFEST)
    release_catalog = _load(RELEASE_CATALOG)
    retained.validate_report_integrity(retained_report)
    caps.validate_report_integrity(caps_report)
    _validate_active_sector_contract(
        active_sector_report, active_sector_loci
    )
    if not (
        flyer_guide_report.get("schema")
        == "retained-flyer-peek-guide-successor/v1"
        and flyer_guide_report.get("status")
        == "GEOMETRY_PASS_REVIEW_ONLY__TERMINAL_ROUTE_FAIL"
        and flyer_guide_report.get("production_authorized") is False
        and flyer_guide_report.get("geometry_gates")
        and all(flyer_guide_report["geometry_gates"].values())
        and flyer_guide_report.get("source_hashes", {}).get(
            "cad/retained_flyer_peek_guide_successor.py"
        ) == _sha256(Path(flyer_successor.__file__))
        and flyer_guide_report.get("report_sha256")
        == _canonical_hash(flyer_guide_report)
    ):
        raise ValueError("retained PEEK flyer guide successor is stale or failed")
    expected = {
        "retained": (
            retained_report,
            "permanent-cap-offset-spoke-retained-review/v1",
        ),
        "caps": (caps_report, "permanent-cap-production-review/v1"),
        "drive": (drive_report, "m2-drive-successor-review/v1"),
    }
    for name, (report, schema) in expected.items():
        if report.get("schema") != schema:
            raise ValueError(f"{name} report schema drift")
    leadshine_gates = leadshine_report.get("gates", {})
    for gate in ("source_hash", "solid_partition", "mount_face_rebased", "frame_xy"):
        if leadshine_gates.get(gate) is not True:
            raise ValueError(f"Leadshine cableless source gate is not closed: {gate}")
    if m2_selection_report.get("schema") != "m2-normal-goal-drive-selection/v1":
        raise ValueError("normal-GOAL M2 selection schema drift")
    if m2_selection_report.get("reference_CAD_integration_authorized") is not True:
        raise ValueError("normal-GOAL M2 selection does not authorize review CAD")
    if m2_selection_report.get("production_authorized") is not False:
        raise ValueError("normal-GOAL M2 selection production boundary drift")
    if m2_selection_report["selection"].get("motor") != (
        "Leadshine CS-M21708 closed-loop NEMA17"
    ):
        raise ValueError("normal-GOAL M2 selected motor drift")
    pulley_authority = m2_selection_report.get(
        "motor_pulley_geometry_and_inertia", {}
    )
    if not (
        nbk_p30.source_sha256() == nbk_p30.SOURCE_STEP_SHA256
        and pulley_authority.get("published_stock_mass_g")
        == nbk_p30.OFFICIAL_MASS_G
        and pulley_authority.get("published_stock_inertia_kgm2")
        == nbk_p30.OFFICIAL_AXIAL_MOMENT_OF_INERTIA_KG_M2
        and pulley_authority.get("BNW", {}).get("set_screw_count") == 2
    ):
        raise ValueError("official NBK P30 stock source/authority drift")
    flyer_d10_rows = [
        row for row in release_catalog.get("items", [])
        if row.get("id") == "flyer-pulley-nbk-p30-3gt-blp-6c-10"
    ]
    if not (
        len(flyer_d10_rows) == 1
        and nbk_p30_d10.source_sha256() == nbk_p30_d10.SOURCE_STEP_SHA256
        and flyer_d10_rows[0].get("selection", {}).get("mpn")
        == nbk_p30_d10.OFFICIAL_PART_NUMBER
        and flyer_d10_rows[0].get("model", {}).get("sha256")
        == nbk_p30_d10.SOURCE_STEP_SHA256
        and flyer_d10_rows[0].get("model", {}).get("path")
        == "cad/models/upgrades/NBK_P30_D10_download/P30-3GT-BLP-6C-10.stp"
        and flyer_d10_rows[0].get("receiving_contract", {}).get("mass_g")
        == nbk_p30_d10.OFFICIAL_MASS_G
        and flyer_d10_rows[0].get("receiving_contract", {}).get(
            "axial_j_kgm2"
        ) == nbk_p30_d10.OFFICIAL_AXIAL_MOMENT_OF_INERTIA_KG_M2
        and flyer_d10_rows[0].get("receiving_contract", {}).get(
            "clamp_torque_nm"
        ) == nbk_p30_d10.OFFICIAL_CLAMP_TORQUE_NM
        and flyer_d10_rows[0].get("authorization_status") == "blocked"
    ):
        raise ValueError("official flyer-side NBK P30 D10 authority drift")
    if not (
        felt_loads_report.get("schema") == 1
        and felt_loads_report.get("status") == "PASS"
        and felt_loads_report.get("current_integration_ready") is True
        and felt_loads_report.get("selected_spring_sizing_ready") is True
        and felt_loads_report.get("selected_spring", {}).get("sku")
        == "94125K614"
        and all(
            row.get("pass") is True
            for row in felt_loads_report.get("selected_spring_checks", [])
        )
        and all(
            row.get("pass") is True
            for row in felt_loads_report.get("current_integration_checks", [])
        )
    ):
        raise ValueError("felt preload/load sizing contract is not PASS")
    if not (
        felt_contact_report.get("schema")
        == "integrated-felt-contact-review/v1"
        and felt_contact_report.get("status") == "PASS_REVIEW_ONLY"
        and felt_contact_report.get("checks")
        and all(felt_contact_report["checks"].values())
        and felt_contact_report.get("source_hashes", {}).get(
            "cad/integrated_release_candidate.py"
        )
        == _sha256(SOURCE)
        and felt_contact_report.get("source_hashes", {}).get(
            "cad/integrated_felt_contact_review.py"
        )
        == _sha256(FELT_CONTACT_SOURCE)
        and felt_contact_report.get("report_sha256")
        == _canonical_hash(felt_contact_report)
    ):
        raise ValueError("integrated felt-contact companion review is stale or failed")
    flyer_static_rows = next(
        (
            row
            for row in base_raw_parts_report.get("pairs", [])
            if row.get("pair") == ["flyer", "static"]
        ),
        None,
    )
    nearest = (
        flyer_static_rows.get("nearest_noncolliding", [])
        if isinstance(flyer_static_rows, dict)
        else []
    )
    if not (
        base_raw_report.get("schema") == "collision-clearance/v2"
        and base_raw_report.get("status") == "FAIL"
        and base_raw_report.get("collisions") == []
        and math.isclose(
            float(base_raw_report.get("minimum_dynamic_clearance_mm", -1.0)),
            1.0,
            abs_tol=1.0e-9,
        )
        and base_raw_parts_report.get("schema")
        == "collision-part-diagnostics/v2"
        and len(nearest) >= 2
        # Frozen pre-stock-D10 diagnostic label; never used as current geometry.
        and nearest[0].get("a")
        == "extended_hollow_flyer_shaft_80mm_with_P30_indexed_flats"
        and nearest[0].get("b") == "entry_bracket"
        and math.isclose(float(nearest[0].get("distance_mm")), 1.0)
        and nearest[1].get("a") == "m2_successor_flyer_P30_3GT_6C"
        and nearest[1].get("b") == "entry_bracket"
        and math.isclose(float(nearest[1].get("distance_mm")), 1.75)
    ):
        raise ValueError("frozen base raw-clearance diagnostic contract drift")
    shaft_manifest_rows = [
        row
        for row in custom_manifest.get("parts", [])
        if row.get("id") == "flyer_shaft_d10_id6_to_id9_l79"
    ]
    shaft_catalog_rows = [
        row
        for row in release_catalog.get("items", [])
        if row.get("id") == "flyer-shaft-stock-d10-rev-d"
    ]
    if len(shaft_manifest_rows) != 1 or len(shaft_catalog_rows) != 1:
        raise ValueError("released flyer-shaft manifest/catalog row missing")
    shaft_manifest = shaft_manifest_rows[0]
    shaft_catalog = shaft_catalog_rows[0]
    artifacts = {
        row.get("role"): row.get("path")
        for row in shaft_catalog.get("manufacturing", {}).get("artifacts", [])
    }
    if not (
        custom_manifest.get("schema") == 1
        and shaft_manifest.get("file")
        == "out/custom/step/flyer_shaft_d10_id6_to_id9_l79.step"
        and shaft_manifest.get("sha256") == _sha256(RELEASED_SHAFT_STEP)
        and shaft_manifest.get("single_solid") is True
        and shaft_manifest.get("bbox_mm") == [12.0, 12.0, 79.0]
        and shaft_manifest.get("artifact_id")
        == "m2-flyer-shaft-stock-d10-rev-d"
        and shaft_manifest.get("drawing")
        == "output/pdf/flyer_shaft_d10_id6_to_id9_l79.pdf"
        and shaft_catalog.get("design_status") == "selected"
        and shaft_catalog.get("purchase_status") == "rfq_ready"
        and shaft_catalog.get("manufacturing", {}).get("revision") == "D"
        and shaft_catalog.get("manufacturing", {}).get("process")
        == "machine hollow bar with full 18.5 mm OD9.991-10.000 h6 / ID6.000-6.030 seat, 3 mm transition to ID9.000-9.050, OD11.980-12.000 main span, two indexed arm flats, shoulder datum and R0.5 internal-mouth polish"
        and shaft_catalog.get("manufacturing", {}).get("material")
        == "6061-T6 aluminum or drawing-approved equivalent"
        and artifacts.get("step")
        == "out/custom/step/flyer_shaft_d10_id6_to_id9_l79.step"
        and artifacts.get("drawing_pdf")
        == "output/pdf/flyer_shaft_d10_id6_to_id9_l79.pdf"
        and RELEASED_SHAFT_PDF.exists()
    ):
        raise ValueError("released M2-001 Rev D shaft authority drift")
    return {
        "retained": retained_report,
        "caps": caps_report,
        "drive": drive_report,
        "leadshine": leadshine_report,
        "m2_selection": m2_selection_report,
        "felt_loads": felt_loads_report,
        "felt_contact": felt_contact_report,
        "flyer_guide": flyer_guide_report,
        "active_sector": active_sector_report,
        "active_sector_loci": active_sector_loci,
        "base_raw": base_raw_report,
        "base_raw_parts": base_raw_parts_report,
        "released_shaft_manifest": shaft_manifest,
        "released_shaft_catalog": shaft_catalog,
        "flyer_D10_catalog": flyer_d10_rows[0],
    }


def retained_review_slug_lengths() -> tuple[float, float, float, float]:
    report = _contract_reports()["retained"]
    values = report["slug_length_solution_mm"]
    return tuple(float(values[pocket.id]) for pocket in retained.POCKETS)


@lru_cache(maxsize=1)
def _main_links() -> dict[str, list[Part]]:
    return assembly.build_links(DEFAULT_STATOR)


def _is_shifted_m2_hardware(label: str) -> bool:
    return label.startswith("flyer_block_") or label.startswith("m2_mount_")


def _is_old_m2_motor_hardware(label: str) -> bool:
    return label.startswith("m2_motor_m3x10_")


def _is_shifted_entry(label: str) -> bool:
    return label in {"entry_bracket", "entry_eyelet"} or label.startswith(
        "entry_base_"
    )


def integrated_entry_bracket(source: Part) -> Part:
    """Rigidly rear-shift the entry module and retain the dancer-anchor datum.

    The complete entry print moves to the selected -4.25 mm datum.  Its fixed
    dancer-spring boss is then extended integrally back to the existing screw
    seat so this clearance correction does not skew or relocate the validated
    spring plane.  Finally, the exact configured supply-wire centerline is
    recut at the original generous R1.6 passage radius.
    """

    shifted = _moved(source, -ENTRY_REAR_SHIFT_MM)
    fx, fy = P.dancer_spring_fixed_x, P.dancer_spring_fixed_y
    keeper_rear_z = -160.1 - ENTRY_ADDITIONAL_REAR_SHIFT_MM
    keeper = Pos(fx, fy, keeper_rear_z) * Cylinder(
        4.5,
        1.6 + ENTRY_ADDITIONAL_REAR_SHIFT_MM,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    result = shifted + keeper
    result -= Pos(fx, fy, keeper_rear_z - 0.1) * Cylinder(
        1.2,
        4.0 + ENTRY_ADDITIONAL_REAR_SHIFT_MM,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    result -= Pos(fx, fy, -159.5) * Cone(
        2.4,
        1.2,
        1.2,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    result -= configured_static_supply_passage_tool()
    result.label = "entry_bracket"
    if len(result.solids()) != 1 or not result.is_valid:
        raise RuntimeError("integrated entry bracket must remain one valid solid")
    return result


def main_static_groups() -> dict[str, list[Part]]:
    """Reuse every current static occurrence, replacing only named M2 parts."""

    unchanged: list[Part] = []
    shifted_support: list[Part] = []
    shifted_entry: list[Part] = []
    for shape in _main_links()["static"]:
        label = str(getattr(shape, "label", ""))
        if label in OLD_STATIC_M2_REPLACED_LABELS:
            continue
        if _is_old_m2_motor_hardware(label):
            continue
        if label in STATIC_M2_BODY_LABELS or _is_shifted_m2_hardware(label):
            shifted_support.append(_moved(shape, -M2_REAR_SHIFT_MM))
        elif label == "entry_bracket":
            shifted_entry.append(integrated_entry_bracket(shape))
        elif _is_shifted_entry(label):
            shifted_entry.append(_moved(shape, -ENTRY_REAR_SHIFT_MM))
        elif label in {"base_rail_L", "base_rail_R"}:
            unchanged.append(
                _moved(shape, -INTEGRATED_FRAME_WINDOW_REAR_SHIFT_MM)
            )
        elif label in FELT_MOVING_STACK_LABELS:
            unchanged.append(_moved(shape, -FELT_MOVING_STACK_TRAVEL_MM))
        else:
            unchanged.append(shape)
    return {
        "unchanged": unchanged,
        "shifted_support": shifted_support,
        "shifted_entry": shifted_entry,
    }


def frame_relocation_attachment_audit(
    static_groups: Mapping[str, list[Part]] | None = None,
) -> dict[str, Any]:
    """Prove fixed brackets/feet/T-nuts remain over real shifted base rails."""

    groups = static_groups or main_static_groups()
    static_parts = [
        *groups["unchanged"],
        *groups["shifted_support"],
        *groups["shifted_entry"],
    ]
    rails = {
        str(part.label): part
        for part in static_parts
        if getattr(part, "label", "") in {"base_rail_L", "base_rail_R"}
    }
    if set(rails) != {"base_rail_L", "base_rail_R"}:
        raise RuntimeError("shifted base-rail occurrences are incomplete")

    def is_attachment(label: str) -> bool:
        frame_base = (
            label.startswith("frame_bracket_")
            and "_base_" in label
            and (
                label.endswith("_L")
                or label.endswith("_R")
                or "_floor_" in label
            )
        )
        return frame_base or label.startswith("foot_")

    rows: list[dict[str, Any]] = []
    selected_parts: dict[str, Part] = {}
    for part in static_parts:
        label = str(getattr(part, "label", ""))
        if not is_attachment(label):
            continue
        selected_parts[label] = part
        distances = {
            rail_name: _distance(part, rail)
            for rail_name, rail in rails.items()
        }
        rail_name = min(distances, key=distances.get)
        rail = rails[rail_name]
        part_box = part.bounding_box()
        rail_box = rail.bounding_box()
        longitudinal_overlap = max(
            0.0,
            min(float(part_box.max.Z), float(rail_box.max.Z))
            - max(float(part_box.min.Z), float(rail_box.min.Z)),
        )
        rear_reserve = float(part_box.min.Z) - float(rail_box.min.Z)
        front_reserve = float(rail_box.max.Z) - float(part_box.max.Z)
        overlap_volume = _overlap(part, rail)
        rows.append({
            "label": label,
            "nearest_rail": rail_name,
            "distance_to_real_rail_BREP_mm": distances[rail_name],
            "positive_intersection_with_rail_mm3": overlap_volume,
            "longitudinal_projection_overlap_mm": longitudinal_overlap,
            "rear_end_material_beyond_occurrence_mm": rear_reserve,
            "front_end_material_beyond_occurrence_mm": front_reserve,
            "fully_inside_shifted_rail_longitudinal_span": (
                rear_reserve >= -1.0e-6 and front_reserve >= -1.0e-6
            ),
            "directly_touches_or_engages_real_extrusion_slot": (
                distances[rail_name] <= 1.0e-5
                or overlap_volume > BOOLEAN_TOL_MM3
            ),
            "projects_over_real_rail_longitudinal_material": (
                longitudinal_overlap > 0.0
                and rear_reserve >= -1.0e-6
                and front_reserve >= -1.0e-6
            ),
        })
    if not rows:
        raise RuntimeError("no base-rail attachment occurrences selected")
    def group_key(label: str) -> str:
        if label.startswith("foot_"):
            return "_".join(label.split("_")[:2])
        return label.split("_floor_", 1)[0]

    rows_by_label = {row["label"]: row for row in rows}
    grouped: dict[str, list[str]] = {}
    for label in selected_parts:
        grouped.setdefault(group_key(label), []).append(label)
    groups: list[dict[str, Any]] = []
    for name, labels in sorted(grouped.items()):
        rail_name = rows_by_label[labels[0]]["nearest_rail"]
        direct = {
            label for label in labels
            if rows_by_label[label][
                "directly_touches_or_engages_real_extrusion_slot"
            ]
        }
        reached = set(direct)
        changed = True
        while changed:
            changed = False
            for label in labels:
                if label in reached:
                    continue
                if any(
                    _distance(
                        selected_parts[label], selected_parts[anchor]
                    ) <= 0.51
                    for anchor in reached
                ):
                    reached.add(label)
                    changed = True
        groups.append({
            "group": name,
            "nearest_rail": rail_name,
            "member_labels": sorted(labels),
            "direct_rail_anchor_labels": sorted(direct),
            "supported_chain_labels": sorted(reached),
            "all_members_supported_by_direct_or_chained_contact": (
                len(reached) == len(labels) and bool(direct)
            ),
            "all_members_project_over_real_rail_longitudinal_material": all(
                rows_by_label[label][
                    "projects_over_real_rail_longitudinal_material"
                ]
                for label in labels
            ),
        })

    return {
        "base_rail_bounds_mm": {
            name: _bbox(rail) for name, rail in rails.items()
        },
        "attachment_occurrence_count": len(rows),
        "attachments": rows,
        "attachment_groups": groups,
        "minimum_rear_end_material_beyond_occurrence_mm": min(
            row["rear_end_material_beyond_occurrence_mm"] for row in rows
        ),
        "minimum_front_end_material_beyond_occurrence_mm": min(
            row["front_end_material_beyond_occurrence_mm"] for row in rows
        ),
        "all_fully_inside_shifted_rail_longitudinal_span": all(
            row["fully_inside_shifted_rail_longitudinal_span"] for row in rows
        ),
        "all_groups_have_supported_attachment_chain": all(
            group["all_members_supported_by_direct_or_chained_contact"]
            for group in groups
        ),
        "all_occurrences_project_over_real_rail_longitudinal_material": all(
            row["projects_over_real_rail_longitudinal_material"]
            for row in rows
        ),
        "any_attachment_over_open_air": any(
            not row["fully_inside_shifted_rail_longitudinal_span"]
            for row in rows
        ),
    }


def entry_module_attachment_audit(
    static_groups: Mapping[str, list[Part]] | None = None,
) -> dict[str, Any]:
    """Prove the rear-shifted entry module remains supported and seated."""

    groups = static_groups or main_static_groups()
    parts = {
        str(part.label): part
        for bucket in groups.values()
        for part in bucket
    }
    required = {
        "entry_bracket",
        "entry_eyelet",
        "rear_post",
        "dancer_spring_fixed_m2x12",
        "entry_base_m5x12_1",
        "entry_base_tnut_1",
        "entry_base_m5x12_2",
        "entry_base_tnut_2",
        "base_rail_L",
        "base_rail_R",
    }
    missing = required - set(parts)
    if missing:
        raise RuntimeError(f"entry attachment audit missing {sorted(missing)}")
    bracket = parts["entry_bracket"]
    rear_post = parts["rear_post"]
    mounting_labels = sorted(
        label for label in required if label.startswith("entry_base_")
    )
    mounting = [
        {
            "label": label,
            "distance_to_entry_bracket_mm": _distance(parts[label], bracket),
            "distance_to_rear_post_mm": _distance(parts[label], rear_post),
            "positive_engagement_with_rear_post_mm3": _overlap(
                parts[label], rear_post
            ),
            "bbox_mm": _bbox(parts[label]),
        }
        for label in mounting_labels
    ]
    return {
        "selected_entry_rear_shift_mm": ENTRY_REAR_SHIFT_MM,
        "additional_shift_beyond_prior_candidate_mm": (
            ENTRY_ADDITIONAL_REAR_SHIFT_MM
        ),
        "entry_bracket_one_valid_solid": (
            len(bracket.solids()) == 1 and bracket.is_valid
        ),
        "entry_bracket_to_rear_post_distance_mm": _distance(
            bracket, rear_post
        ),
        "entry_bracket_to_rear_post_engagement_mm3": _overlap(
            bracket, rear_post
        ),
        "entry_eyelet_to_bracket_distance_mm": _distance(
            parts["entry_eyelet"], bracket
        ),
        "entry_eyelet_seat_overlap_mm3": _overlap(
            parts["entry_eyelet"], bracket
        ),
        "preserved_dancer_fixed_screw_to_bracket_distance_mm": _distance(
            parts["dancer_spring_fixed_m2x12"], bracket
        ),
        "mounting_hardware": mounting,
        "all_mounting_hardware_contacts_bracket_and_post": all(
            row["distance_to_entry_bracket_mm"] <= CONTACT_TOL_MM
            and row["distance_to_rear_post_mm"] <= CONTACT_TOL_MM
            and row["positive_engagement_with_rear_post_mm3"]
            > BOOLEAN_TOL_MM3
            for row in mounting
        ),
        "base_rail_clearances_mm": {
            label: _distance(bracket, parts[label])
            for label in ("base_rail_L", "base_rail_R")
        },
    }


@lru_cache(maxsize=1)
def selected_m2_motor_local() -> Compound:
    """Exact cable-less Leadshine body at its source mounting datum."""

    return leadshine.gen_step()


@lru_cache(maxsize=1)
def official_motor_pulley_review() -> nbk_p30.PlacedP30Review:
    """Place immutable official stock P30 plus non-destructive BNW witnesses.

    The helper imports the byte-identical vendor STEP and applies only the
    world placement.  Its local tooth midplane is the origin, so the supplied
    center is exactly the already-reviewed M2 motor axis and P30 tooth
    midplane.  The two hole paths and M3x12 screw envelopes remain separate
    positive review occurrences; they are never cut from, fused into, or used
    to re-export the vendor model by itself.
    """

    return nbk_p30.place_for_m2(
        (0.0, drive.MOTOR_AXIS_Y, drive.PULLEY_CENTER_Z),
        stock_roll_deg=NBK_P30_STOCK_ROLL_DEG,
        bnw_first_azimuth_deg=NBK_P30_BNW_FIRST_AZIMUTH_DEG,
        bnw_local_x_mm=NBK_P30_BNW_LOCAL_X_MM,
        bnw_screw_inward_adjustments_mm=(
            NBK_P30_BNW_SCREW_INWARD_ADJUSTMENTS_MM
        ),
    )


@lru_cache(maxsize=1)
def official_flyer_pulley_review() -> nbk_p30_d10.PlacedD10:
    """Place the immutable stock D10 flyer pulley hub-rear.

    The source tooth midplane is placed at the same pre-module datum as the
    motor pulley, then the complete M2 module receives its existing -10 mm
    shift.  The final flyer occurrence therefore spans z=-110.75..-92.25.
    """

    return nbk_p30_d10.place_hub_rear(
        (0.0, 0.0, drive.PULLEY_CENTER_Z),
        stock_roll_deg=NBK_P30_STOCK_ROLL_DEG,
    )


@lru_cache(maxsize=1)
def successor_drive_parts() -> dict[str, Part | Compound]:
    source = drive.review_parts()
    official = official_motor_pulley_review()
    flyer_official = official_flyer_pulley_review()
    static_keys = [
        "mount", "motor", "motor_pulley",
        "motor_pulley_BNW_hole_path_0", "motor_pulley_BNW_hole_path_1",
        "motor_pulley_BNW_set_screw_0", "motor_pulley_BNW_set_screw_1",
        "belt",
        *sorted(key for key in source if key.startswith("motor_screw_")),
    ]
    rotating_keys = ["flyer_pulley"]
    result: dict[str, Part | Compound] = {}
    for key in (*static_keys, *rotating_keys):
        if key == "motor_pulley":
            selected = official.stock_occurrence
        elif key == "flyer_pulley":
            selected = flyer_official.stock_occurrence
        elif key.startswith("motor_pulley_BNW_hole_path_"):
            index = int(key.rsplit("_", 1)[1])
            selected = official.bnw_hole_witnesses[index]
        elif key.startswith("motor_pulley_BNW_set_screw_"):
            index = int(key.rsplit("_", 1)[1])
            selected = official.bnw_set_screw_witnesses[index]
        else:
            selected = source[key]
        if key == "motor":
            selected = Pos(
                0.0, drive.MOTOR_AXIS_Y, drive.MOTOR_FACE_Z
            ) * selected_m2_motor_local()
            selected.label = "m2_Leadshine_CS-M21708_exact_cableless"
        result[key] = _moved(selected, -M2_REAR_SHIFT_MM)
    return result


def shaft_with_integrated_p30_flats() -> Part:
    """Place the released M2-001 Rev D stock-D10 shaft authority."""

    result = Pos(0.0, 0.0, RELEASED_SHAFT_CENTER_Z_MM) * (
        flyer_shaft_d10.flyer_shaft()
    )
    result.label = "released_M2_001_Rev_D_flyer_shaft_D10_ID6_ID9_L79"
    return result


@lru_cache(maxsize=1)
def retained_arm_shaft_root_web() -> Part:
    """Return the annular root sleeve/web bridging collar, spoke and rail."""

    z0, z1 = ARM_SHAFT_ROOT_WEB_Z_MM
    outer = Pos(0.0, 0.0, z0) * Cylinder(
        ARM_SHAFT_ROOT_WEB_OUTER_DIAMETER_MM / 2.0,
        z1 - z0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    bore = Pos(0.0, 0.0, z0 - 1.0) * Cylinder(
        ARM_SHAFT_BORE_DIAMETER_MM / 2.0,
        z1 - z0 + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    result = outer - bore
    result.label = "OD18_x11p5_annular_root_load_path_web_ID12p10"
    return result


@lru_cache(maxsize=1)
def retained_arm_with_released_shaft_bore() -> Part:
    """Open the final one-piece arm to its manufacturable OD12 shaft bore.

    The retained review cut the Ø12.10 bore in the collar component before
    unioning the spoke.  The spoke subsequently refilled the front half of
    that passage, creating a real shaft/print penetration.  An 11.5 mm axial,
    OD18 annular root sleeve/web first joins the existing collar, main spoke
    and rear counterrail outside the shaft envelope; recutting the Ø12.10
    interface last then preserves 0.05 mm radial running clearance while the
    two orthogonal M3x8 screws positively retain the shaft on indexed flats.
    """

    z0, z1 = ARM_SHAFT_BORE_CUT_Z_MM
    cutter = Pos(0.0, 0.0, z0) * Cylinder(
        ARM_SHAFT_BORE_DIAMETER_MM / 2.0,
        z1 - z0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    result = (
        retained.retained_arm() + retained_arm_shaft_root_web()
    ) - cutter
    solids = list(result.solids())
    if len(solids) != 1 or not result.is_valid:
        raise RuntimeError(
            "released shaft-bore correction did not preserve one valid "
            f"printed arm solid: {len(solids)} solids"
        )
    result.label = (
        "retained_offset_spoke_flyer_arm_one_printed_solid_"
        "OD12p10_final_union_bore"
    )
    return result


def released_shaft_brep_audit(shaft: Part) -> dict[str, Any]:
    """Measure the placed Rev-D stock-D10 shaft from its final BREP."""

    cylindrical_radii = sorted({
        round(float(face.radius), 9)
        for face in shaft.faces()
        if face.geom_type == GeomType.CYLINDER
    })
    flat_rows: list[dict[str, Any]] = []
    for face in shaft.faces():
        if face.geom_type != GeomType.PLANE:
            continue
        center = face.center()
        normal = face.normal_at()
        box = face.bounding_box()
        axis: str | None = None
        if math.isclose(center.X, 5.7, abs_tol=1.0e-6) and math.isclose(
            normal.X, 1.0, abs_tol=1.0e-6
        ):
            axis = "plus_x"
        elif math.isclose(center.Y, -5.7, abs_tol=1.0e-6) and math.isclose(
            normal.Y, -1.0, abs_tol=1.0e-6
        ):
            axis = "minus_y"
        if axis is not None:
            flat_rows.append({
                "normal": axis,
                "station_from_rear_datum_mm": (
                    float(center.Z) - flyer_shaft_d10.WORLD_REAR_Z_MM
                ),
                "axial_length_mm": float(box.max.Z - box.min.Z),
                "flat_radius_mm": 5.7,
                "depth_from_OD_mm": 0.3,
            })
    flat_rows.sort(key=lambda row: (
        row["station_from_rear_datum_mm"], row["normal"]
    ))
    mouth_faces = [
        face for face in shaft.faces()
        if face.geom_type == GeomType.TORUS
    ]
    return {
        "label": str(shaft.label),
        "bbox_mm": _bbox(shaft),
        "solid_count": len(list(shaft.solids())),
        "valid": bool(shaft.is_valid),
        "volume_mm3": float(shaft.volume),
        "cylindrical_surface_radii_mm": cylindrical_radii,
        "neck_outer_diameter_mm": flyer_shaft_d10.NECK_OD_MM,
        "neck_inner_diameter_mm": flyer_shaft_d10.NECK_ID_MM,
        "neck_outer_diameter_h6_limits_mm": list(
            flyer_shaft_d10.NECK_OD_H6_LIMITS_MM
        ),
        "neck_inner_diameter_limits_mm": list(
            flyer_shaft_d10.NECK_ID_LIMITS_MM
        ),
        "minimum_neck_radial_wall_at_limits_mm": (
            flyer_shaft_d10.MIN_NECK_RADIAL_WALL_AT_LIMITS_MM
        ),
        "main_outer_diameter_mm": flyer_shaft_d10.MAIN_OD_MM,
        "main_inner_diameter_mm": flyer_shaft_d10.MAIN_ID_MM,
        "main_outer_diameter_limits_mm": list(
            flyer_shaft_d10.MAIN_OD_LIMITS_MM
        ),
        "main_inner_diameter_limits_mm": list(
            flyer_shaft_d10.MAIN_ID_LIMITS_MM
        ),
        "length_mm": flyer_shaft_d10.LENGTH_MM,
        "rear_datum_z_mm": flyer_shaft_d10.WORLD_REAR_Z_MM,
        "front_datum_z_mm": flyer_shaft_d10.WORLD_FRONT_Z_MM,
        "D10_seat_length_mm": flyer_shaft_d10.NECK_LENGTH_MM,
        "shoulder_z_mm": flyer_shaft_d10.SHOULDER_WORLD_Z_MM,
        "ID6_to_ID9_transition_length_mm": (
            flyer_shaft_d10.TRANSITION_LENGTH_MM
        ),
        "ID6_to_ID9_transition_end_z_mm": (
            flyer_shaft_d10.TRANSITION_END_WORLD_Z_MM
        ),
        "indexed_flats": flat_rows,
        "wire_mouth_fillet_radius_mm": flyer_shaft_d10.MOUTH_FILLET_MM,
        "wire_mouth_toroidal_face_count": len(mouth_faces),
        "wire_mouth_toroidal_face_centers_z_mm": sorted(
            float(face.center().Z) for face in mouth_faces
        ),
    }


def released_shaft_front_interface_audit(
    shaft: Part, guide: Part, flyer_wire: Part,
) -> dict[str, Any]:
    """Fail closed on the Rev-D shaft front, guide and conductor interface."""

    root_sleeve_front_z = float(flyer_successor.ROOT_SLEEVE_Z_MM[1])
    shaft_front_z = float(flyer_shaft_d10.WORLD_FRONT_Z_MM)
    setback = root_sleeve_front_z - shaft_front_z
    guide_overlap = _overlap(shaft, guide)
    wire_overlap = _overlap(shaft, flyer_wire)
    result = {
        "shaft_front_z_mm": shaft_front_z,
        "root_sleeve_front_z_mm": root_sleeve_front_z,
        "shaft_front_setback_from_root_sleeve_front_mm": setback,
        "minimum_required_setback_mm": (
            MIN_SHAFT_FRONT_SETBACK_FROM_ROOT_SLEEVE_MM
        ),
        "shaft_to_PEEK_guide_outer_distance_mm": _distance(shaft, guide),
        "shaft_vs_PEEK_guide_outer_overlap_mm3": guide_overlap,
        "shaft_to_flyer_wire_distance_mm": _distance(shaft, flyer_wire),
        "shaft_vs_flyer_wire_overlap_mm3": wire_overlap,
    }
    result["source_gate"] = (
        setback + 1.0e-9
        >= MIN_SHAFT_FRONT_SETBACK_FROM_ROOT_SLEEVE_MM
        and guide_overlap <= BOOLEAN_TOL_MM3
        and wire_overlap <= BOOLEAN_TOL_MM3
    )
    return result


def arm_root_sleeve_load_path_audit(
    arm: Part, shaft: Part, flyer_wire: Part,
) -> dict[str, Any]:
    """Prove the corrected print load path and conservative root stresses."""

    web = retained_arm_shaft_root_web()
    components = retained.base.offset_spoke_arm_components()
    rail = retained._box(
        -retained.RAIL_WIDTH_MM / 2.0,
        retained.RAIL_WIDTH_MM / 2.0,
        retained.RAIL_Y_MM[0], retained.RAIL_Y_MM[1],
        retained.RAIL_Z_MM[0], retained.RAIL_Z_MM[1],
    )
    outer_r = ARM_SHAFT_ROOT_WEB_OUTER_DIAMETER_MM / 2.0
    inner_r = ARM_SHAFT_BORE_DIAMETER_MM / 2.0
    area = math.pi * (outer_r**2 - inner_r**2)
    polar_j = math.pi / 2.0 * (outer_r**4 - inner_r**4)
    second_i = polar_j / 2.0
    moment = ARM_ROOT_LOAD_TENSION_N * ARM_ROOT_LOAD_RADIUS_MM
    torsion_shear = moment * outer_r / polar_j
    bending_stress = moment * outer_r / second_i
    von_mises = math.sqrt(bending_stress**2 + 3.0 * torsion_shear**2)
    factored = ARM_ROOT_REVIEW_SAFETY_FACTOR * von_mises
    job_wire_clearance = (
        retained.base.SPOKE_FRONT_Z_MM
        - wire_vis.R_VIS
        - ARM_SHAFT_ROOT_WEB_Z_MM[1]
    )
    max_wire_clearance = (
        retained.base.SPOKE_FRONT_Z_MM
        - wire_geometry.WIRE_RADIUS_MAX
        - ARM_SHAFT_ROOT_WEB_Z_MM[1]
    )
    return {
        "failure_found_before_correction": {
            "retained_arm_vs_released_shaft_overlap_mm3": _overlap(
                retained.retained_arm(), shaft
            ),
            "classification": "unintended positive penetration; not a clamp fit",
        },
        "corrected_interface": {
            "arm_label": str(arm.label),
            "arm_solid_count": len(list(arm.solids())),
            "arm_valid": bool(arm.is_valid),
            "arm_vs_shaft_overlap_mm3": _overlap(arm, shaft),
            "arm_to_shaft_radial_clearance_mm": _distance(arm, shaft),
            "shaft_bore_diameter_mm": ARM_SHAFT_BORE_DIAMETER_MM,
            "shaft_nominal_OD_mm": 12.0,
            "diametral_clearance_mm": ARM_SHAFT_BORE_DIAMETER_MM - 12.0,
            "radial_clearance_mm": ARM_SHAFT_BORE_RADIAL_CLEARANCE_MM,
            "final_bore_recut_after_all_unions": True,
        },
        "post_print_manufacturing_contract": {
            "CAD_bore_diameter_mm": ARM_SHAFT_BORE_DIAMETER_MM,
            "operation": (
                "ream final unioned collar/root sleeve through in one setup "
                "from the rear datum; remove all burrs; no diameter step, "
                "ovality or local FDM constriction is permitted"
            ),
            "finished_bore_tolerance_mm": list(
                ARM_SHAFT_BORE_REAM_TOLERANCE_MM
            ),
            "measured_shaft_OD_acceptance_mm": list(
                RELEASED_SHAFT_OD_TOLERANCE_MM
            ),
            "resulting_diametral_clearance_range_mm": [
                ARM_SHAFT_BORE_REAM_TOLERANCE_MM[0]
                - RELEASED_SHAFT_OD_TOLERANCE_MM[1],
                ARM_SHAFT_BORE_REAM_TOLERANCE_MM[1]
                - RELEASED_SHAFT_OD_TOLERANCE_MM[0],
            ],
            "resulting_radial_clearance_range_mm": [
                (
                    ARM_SHAFT_BORE_REAM_TOLERANCE_MM[0]
                    - RELEASED_SHAFT_OD_TOLERANCE_MM[1]
                ) / 2.0,
                (
                    ARM_SHAFT_BORE_REAM_TOLERANCE_MM[1]
                    - RELEASED_SHAFT_OD_TOLERANCE_MM[0]
                ) / 2.0,
            ],
            "gauge_plan": (
                "record three-axis shaft micrometer readings at both clamp "
                "stations; Ø12.10 GO passes the full bore and Ø12.13 NO-GO "
                "does not; dry shaft must traverse the full bore by hand "
                "without binding before the two M3x8 screws are seated"
            ),
            "as_printed_FDM_bore_is_accepted_without_reaming": False,
            "measured_fit_and_assembly_check_required_before_balance": True,
            "measured_fit_and_assembly_check_complete": False,
        },
        "root_sleeve_web": {
            "outer_diameter_mm": ARM_SHAFT_ROOT_WEB_OUTER_DIAMETER_MM,
            "inner_diameter_mm": ARM_SHAFT_BORE_DIAMETER_MM,
            "axial_span_z_mm": list(ARM_SHAFT_ROOT_WEB_Z_MM),
            "axial_length_mm": (
                ARM_SHAFT_ROOT_WEB_Z_MM[1] - ARM_SHAFT_ROOT_WEB_Z_MM[0]
            ),
            "radial_ligament_mm": outer_r - inner_r,
            "solid_count": len(list(web.solids())),
            "web_to_existing_collar_overlap_mm3": _overlap(
                web, components["collar"]
            ),
            "web_to_main_spoke_overlap_mm3": _overlap(
                web, components["spoke"]
            ),
            "web_to_rear_counterrail_overlap_mm3": _overlap(web, rail),
            "existing_spoke_to_collar_overlap_mm3": _overlap(
                components["spoke"], components["collar"]
            ),
            "web_to_actual_job_wire_clearance_mm": _distance(
                web, flyer_wire
            ),
            "analytic_job_wire_clearance_mm": job_wire_clearance,
            "analytic_0p5mm_wire_clearance_mm": max_wire_clearance,
        },
        "conservative_combined_root_load_case": {
            "wire_tension_N": ARM_ROOT_LOAD_TENSION_N,
            "load_radius_mm": ARM_ROOT_LOAD_RADIUS_MM,
            "simultaneous_bending_and_torsion_moment_each_Nmm": moment,
            "basis": (
                "conservative simultaneous full 10 N at R64 bending and "
                "torsion; a single physical wire-force vector cannot apply "
                "both full components at once"
            ),
            "annular_area_mm2": area,
            "polar_second_moment_J_mm4": polar_j,
            "planar_second_moment_I_mm4": second_i,
            "outer_fiber_torsional_shear_MPa": torsion_shear,
            "outer_fiber_bending_stress_MPa": bending_stress,
            "von_Mises_equivalent_MPa": von_mises,
            "review_safety_factor": ARM_ROOT_REVIEW_SAFETY_FACTOR,
            "safety_factored_equivalent_MPa": factored,
            "PETG_review_allowable_MPa": ARM_ROOT_PETG_REVIEW_ALLOWABLE_MPA,
            "allowable_to_factored_load_margin": (
                ARM_ROOT_PETG_REVIEW_ALLOWABLE_MPA / factored
            ),
            "passes_review_allowable": (
                factored <= ARM_ROOT_PETG_REVIEW_ALLOWABLE_MPA
            ),
            "print_orientation": (
                "broad spoke face on the XY bed; shaft/root-sleeve axis "
                "normal to the bed; continuous concentric perimeters through "
                "the full OD18/ID12.10 sleeve, with no seam or sparse-infill "
                "load path accepted"
            ),
            "allowable_scope": (
                "10 MPa is a conservative across-layer screening input, not "
                "a filament/temperature certificate; production remains "
                "blocked on an orientation-matched printed coupon"
            ),
            "orientation_matched_physical_coupon_complete": False,
        },
    }


def _swept_wire(
    edges: list[Edge],
    start: tuple[float, float, float],
    initial_direction: tuple[float, float, float],
    radius_mm: float,
    label: str,
) -> Part:
    paths = Wire.combine(edges)
    if len(paths) != 1:
        raise ValueError(f"{label} has {len(paths)} disconnected centerlines")
    with BuildSketch(
        Plane(origin=start, z_dir=initial_direction)
    ) as profile:
        Circle(radius_mm)
    result = sweep(
        profile.sketch, paths[0], transition=Transition.TRANSFORMED
    )
    result.label = label
    return result


def _configured_static_supply_route(
    sweep_radius_mm: float,
    label: str,
) -> Part:
    """Sweep the actual-job centerline at a requested review/tool radius.

    The shared route is a maximum-wire clearance path (R8.25 around the
    dancer and a 0.5 mm felt gap).  The default job is only 0.22352 mm wire,
    so using that centerline makes the visible strand float 0.13824 mm from
    both the pulley and each pad.  This candidate constructs the operating
    contact state at R8+actual-radius and advances the spring-loaded felt
    stack by ``0.5-wire_d``.  The original maximum-wire path remains a
    separately reported configuration envelope, not a simultaneous solid.
    """

    centerline_wire_radius = wire_vis.R_VIS
    path_radius = (
        wire_geometry.DANCER_BODY_RADIUS + centerline_wire_radius
    )
    z_plane = CONFIGURED_WIRE_PLANE_Z_MM
    dancer_center = (P.dancer_pulley_x, P.dancer_pulley_y, z_plane)
    theta_in = math.pi
    theta_out = theta_in - math.radians(wire_geometry.DANCER_WRAP_DEG)

    def on_pulley(theta: float) -> tuple[float, float, float]:
        return (
            dancer_center[0] + path_radius * math.cos(theta),
            dancer_center[1] + path_radius * math.sin(theta),
            z_plane,
        )

    tangent_in = on_pulley(theta_in)
    tangent_out = on_pulley(theta_out)
    spool = (tangent_in[0], P.spool_y, z_plane)
    entry_corner = (0.0, 0.0, z_plane)
    incoming = wire_geometry._unit(
        wire_geometry._sub(entry_corner, tangent_out)
    )
    _entry_points, entry = wire_geometry._circular_fillet(
        entry_corner,
        incoming,
        (0.0, 0.0, 1.0),
        wire_geometry.ENTRY_BEND_RADIUS,
    )
    dancer_plane = Plane(
        origin=dancer_center, x_dir=(1.0, 0.0, 0.0), z_dir=(0.0, 0.0, 1.0)
    )
    guide_root = (
        0.0,
        0.0,
        flyer_successor.GUIDE_ROOT_AXIAL_START_Z_MM,
    )
    edges = [
        Edge.make_line(spool, tangent_in),
        Edge.make_circle(
            path_radius,
            dancer_plane,
            math.degrees(theta_in),
            math.degrees(theta_out),
            AngularDirection.CLOCKWISE,
        ),
        Edge.make_line(tangent_out, tuple(entry["start"])),
        wire_vis._fillet_edge(entry),
        Edge.make_line(tuple(entry["end"]), guide_root),
    ]
    return _swept_wire(
        edges,
        spool,
        (0.0, 1.0, 0.0),
        sweep_radius_mm,
        label,
    )


@lru_cache(maxsize=1)
def configured_static_supply_wire() -> Part:
    """Actual supply wire through entry, shaft bore and guide-root seam.

    The shaft-axis run is owned by the static link because M2 rotation leaves
    every point on that axis invariant.  It terminates exactly where the
    flyer-owned one-piece-guide conductor begins, eliminating the prior
    invisible 68.75 mm gap without inventing flexible off-axis dynamics.
    """

    return _configured_static_supply_route(
        wire_vis.R_VIS,
        "wire_static_actual_job_contact_state_through_shaft_to_guide_root",
    )


@lru_cache(maxsize=1)
def configured_static_supply_passage_tool() -> Part:
    """R1.6 passage swept on the exact actual-job centerline."""

    return _configured_static_supply_route(
        ENTRY_PASSAGE_RADIUS_MM,
        "entry_passage_tool_on_actual_job_centerline",
    )


def cap_module_parts(axis_z_mm: float = P.m0_home_standoff) -> list[Part]:
    transform = (
        Pos(0.0, 0.0, axis_z_mm)
        * Rot(0.0, 90.0, 0.0)
        * Rot(-90.0, 0.0, 0.0)
    )
    local_parts: list[Part] = [
        terminal_guide.cap_with_short_leadins(1),
        terminal_guide.cap_with_short_leadins(-1),
        *caps.retention_hardware(),
    ]
    result: list[Part] = []
    for shape in local_parts:
        moved = transform * shape
        moved.label = str(getattr(shape, "label", "cap_part"))
        result.append(moved)
    return result


def carriage_module_parts() -> list[Part]:
    """Current M0 link with the keyed active-sector successor installed.

    The original tower attachment hardware stays valid.  Only its printed
    tower body is replaced by the revised four-pilot/keyed body returned by
    the active-sector module; the guide pair, aluminum yoke and their complete
    M3/M4 retention stacks are added in the same M0-following/M1-static frame.
    """

    result = [
        shape for shape in _main_links()["carriage"]
        if str(getattr(shape, "label", "")) != "spindle_tower"
    ]
    result.extend(terminal_guide.carriage_link_reference_parts())
    labels = [str(getattr(shape, "label", "")) for shape in result]
    if labels.count("spindle_tower_with_active_sector_M4_insert_pilots") != 1:
        raise RuntimeError("active-sector carriage must contain one revised tower")
    if "spindle_tower" in labels:
        raise RuntimeError("obsolete spindle tower survived active-sector merge")
    return result


def cap_wire_witnesses(axis_z_mm: float = P.m0_home_standoff) -> list[Part]:
    transform = (
        Pos(0.0, 0.0, axis_z_mm)
        * Rot(0.0, 90.0, 0.0)
        * Rot(-90.0, 0.0, 0.0)
    )
    result: list[Part] = []
    for axial_sign in (1, -1):
        shape = caps.nominal_wire_witness(0, axial_sign)
        moved = transform * shape
        moved.label = str(shape.label)
        result.append(moved)
    return result


def spindle_without_stator() -> tuple[list[Part], Part]:
    context: list[Part] = []
    stator: Part | None = None
    for shape in _main_links()["spindle"]:
        if getattr(shape, "label", "") == "stator":
            stator = shape
        else:
            context.append(shape)
    if stator is None:
        raise RuntimeError("current spindle link has no labeled stator")
    return context, stator


@lru_cache(maxsize=1)
def integrated_base_rotating_parts() -> dict[str, Part]:
    drive_parts = successor_drive_parts()
    result: dict[str, Part] = {
        "retained_arm": flyer_successor.revised_retained_arm(),
        "shaft": shaft_with_integrated_p30_flats(),
        "flyer_pulley": drive_parts["flyer_pulley"],
        "flyer_PEEK_guide": flyer_successor.peek_guide_insert(),
        "DIN988_shim": retained.din988_axial_shim(),
        "m2_inner_rear_shim": Pos(0.0, 0.0, -85.25) * cots.tube_spacer(
            18.0, 12.05, 0.5,
        ),
        "m2_inner_center_spacer": Pos(0.0, 0.0, -71.5) * cots.tube_spacer(
            17.8, 12.05, 11.0,
        ),
        "m2_inner_front_spacer": Pos(0.0, 0.0, -56.0) * cots.tube_spacer(
            18.0, 12.05, 4.0,
        ),
    }
    for key in ("m2_inner_rear_shim", "m2_inner_center_spacer", "m2_inner_front_spacer"):
        result[key].label = key
    for index, shape in enumerate(
        flyer_successor.guide_retention_hardware(), start=1
    ):
        kind = "screw" if index % 2 == 1 else "insert"
        result[f"flyer_PEEK_guide_retention_{kind}_{(index + 1) // 2}"] = shape
    for name, shape, _material in retained._existing_rotating_clamp_hardware():
        if not name.startswith("flyer_pulley_"):
            if name == "shaft_clamp_neg_y_radial_M3x8_set_screw_not_counterweight":
                shape = Pos(
                    0.0, ARM_M3X8_SCREW_INWARD_ADJUSTMENT_MM, 0.0
                ) * shape
                shape.label = name
            elif name == "shaft_clamp_pos_x_radial_M3x8_set_screw_not_counterweight":
                shape = Pos(
                    -ARM_M3X8_SCREW_INWARD_ADJUSTMENT_MM, 0.0, 0.0
                ) * shape
                shape.label = name
            result[name] = shape
    return result


def _material_for_rotating_name(name: str) -> str:
    fixed = {
        "retained_arm": "PETG",
        "shaft": "aluminum",
        "flyer_pulley": "aluminum",
        "flyer_PEEK_guide": "natural unfilled PEEK",
        "DIN988_shim": "steel",
        "m2_inner_rear_shim": "steel",
        "m2_inner_center_spacer": "steel",
        "m2_inner_front_spacer": "steel",
    }
    if name in fixed:
        return fixed[name]
    if "set_screw" in name:
        return "steel"
    if "PEEK_guide_retention_screw" in name:
        return "steel"
    if "PEEK_guide_retention_insert" in name:
        return "brass"
    if "insert" in name:
        return "brass"
    if "tungsten_slug" in name:
        return "ASTM-B777 tungsten alloy"
    if "printed_retainer" in name:
        return "PETG"
    if "M3x6_screw" in name:
        return "18-8 stainless steel"
    raise KeyError(f"no rotating material for {name}")


def _official_flyer_pulley_mass_row(shape: Part) -> dict[str, Any]:
    """Bind exact stock mass/J while using the official BREP COM direction.

    NBK's table values include the supplied SCM435 M2 clamp bolt.  The vendor
    STEP controls the placed radial center direction but its homogeneous BREP
    volume is not used to recalculate the published 28 g or 3e-6 kg m2.
    Delivered balance still remains an empirical receiving gate.
    """

    center = shape.center()
    mass = nbk_p30_d10.OFFICIAL_MASS_G
    return {
        "name": "flyer_pulley",
        "material": "NBK stock A2017 pulley plus supplied SCM435 M2 clamp bolt",
        "density_g_cm3": None,
        "volume_mm3": float(shape.volume),
        "mass_g": mass,
        "center_of_mass_mm": [
            float(center.X), float(center.Y), float(center.Z),
        ],
        "static_first_moment_g_mm": [
            mass * float(center.X), mass * float(center.Y),
        ],
        "couple_first_moment_g_mm2": [
            mass * float(center.X) * float(center.Z),
            mass * float(center.Y) * float(center.Z),
        ],
        "izz_about_M2_axis_g_mm2": (
            nbk_p30_d10.OFFICIAL_AXIAL_MOMENT_OF_INERTIA_KG_M2 * 1.0e9
        ),
        "mass_and_J_authority": "NBK P30-3GT-BLP-6C product table",
        "source_step_sha256": nbk_p30_d10.SOURCE_STEP_SHA256,
        "supplied_clamp_bolt_included": True,
    }


@lru_cache(maxsize=1)
def _integrated_base_mass_rows() -> tuple[dict[str, Any], ...]:
    rows: list[dict[str, Any]] = []
    for name, shape in integrated_base_rotating_parts().items():
        material = _material_for_rotating_name(name)
        if name == "flyer_pulley":
            rows.append(_official_flyer_pulley_mass_row(shape))
        elif material == "natural unfilled PEEK":
            rows.append(flyer_successor._density_properties(
                name, shape, flyer_successor.PEEK_DENSITY_G_MM3, material,
            ))
        else:
            rows.append(retained._properties(name, shape, material))
    return tuple(rows)


def _integrated_exact_mass_rows(
    lengths_mm: Iterable[float],
    front_trim_thickness_mm: float,
) -> list[dict[str, Any]]:
    rows = [deepcopy(row) for row in _integrated_base_mass_rows()]
    thickness = float(front_trim_thickness_mm)
    rows.extend(
        flyer_successor._density_properties(
            f"integrated_front_trim_B777_{x_mm:+.1f}",
            flyer_successor.front_trim_slug(x_mm, thickness),
            retained.TUNGSTEN_DENSITY_G_CM3 / 1000.0,
            "ASTM-B777 tungsten alloy",
        )
        for x_mm in flyer_successor.FRONT_TRIM_X_MM
    )
    rows.extend(
        retained._properties(
            f"integrated_front_trim_hardware_{index + 1}", shape,
            "brass" if index % 3 == 2 else "steel",
        )
        for index, shape in enumerate(
            flyer_successor.front_trim_hardware(thickness)
        )
    )
    for pocket, length in zip(retained.POCKETS, lengths_mm):
        rows.extend(
            retained._properties(name, shape, material)
            for name, shape, material in retained.stack_parts(
                pocket, float(length)
            )
        )
    return rows


@lru_cache(maxsize=1)
def integrated_balance_solution() -> dict[str, Any]:
    result = flyer_successor.solve_successor_balance_with_base_rows(
        _integrated_base_mass_rows()
    )
    result["authority"] = (
        "INTEGRATED_REV_D_L79_D10_SHAFT_OFFICIAL_NBK_P30_D10_28G_"
        "J3E6_SUPPLIED_M2_CLAMP_PEEK_GUIDE_ROOT_SLEEVE_BASE_ROWS"
    )
    # Independent exact-row reconstruction catches any drift between the
    # public successor solve seam and the occurrence builder used below.
    reconstructed = retained._sum_properties(_integrated_exact_mass_rows(
        result["rear_slug_lengths_mm"],
        result["front_trim_common_thickness_mm"],
    ))
    for key in (
        "static_imbalance_g_mm", "couple_imbalance_g_mm2",
        "izz_about_M2_axis_kg_m2",
    ):
        if not math.isclose(
            float(reconstructed[key]),
            float(result["mass_properties"][key]),
            rel_tol=0.0, abs_tol=1.0e-12,
        ):
            raise RuntimeError(f"integrated balance row reconstruction drift: {key}")
    return result


def integrated_slug_lengths() -> tuple[float, float, float, float]:
    values = integrated_balance_solution()["rear_slug_lengths_mm"]
    return tuple(map(float, values))  # type: ignore[return-value]


def retained_rotating_parts(
    *, tip_guide_override: Part | None = None,
) -> dict[str, Part]:
    result = dict(integrated_base_rotating_parts())
    if tip_guide_override is not None:
        raise ValueError(
            "tip-guide overrides are superseded by the retained one-piece "
            "PEEK guide/bell and invalidate balance"
        )
    solution = integrated_balance_solution()
    lengths = tuple(map(float, solution["rear_slug_lengths_mm"]))
    for pocket, length in zip(retained.POCKETS, lengths):
        for name, shape, _material in retained.stack_parts(pocket, length):
            result[name] = shape
    thickness = float(solution["front_trim_common_thickness_mm"])
    for x_mm in flyer_successor.FRONT_TRIM_X_MM:
        shape = flyer_successor.front_trim_slug(x_mm, thickness)
        result[f"front_trim_B777_{x_mm:+.1f}"] = shape
    for index, shape in enumerate(
        flyer_successor.front_trim_hardware(thickness), start=1
    ):
        result[f"front_trim_hardware_{index}"] = shape
    return result


def rotating_mass_rows() -> tuple[list[dict[str, Any]], dict[str, Any]]:
    solution = integrated_balance_solution()
    rows = [deepcopy(row) for row in solution["mass_rows"]]
    return rows, retained._sum_properties(rows)


def integrated_six_stack_attachment_audit() -> dict[str, Any]:
    """Prove what each of the six balance-correction screws threads into.

    Four rear M3 screws enter the heat-set inserts carried by continuous
    printed retainer bosses; two front M2 screws enter standard heat-set
    inserts in blind pilots inside the main printed spoke.  The report keeps
    the exact face contacts, walls and material remaining beyond each pilot so
    a screw occurrence over empty space cannot satisfy this contract.
    """

    solution = integrated_balance_solution()
    rear_lengths = tuple(map(float, solution["rear_slug_lengths_mm"]))
    mass_rows = [deepcopy(row) for row in solution["mass_rows"]]
    rear = retained._retention_audit(rear_lengths, mass_rows)

    arm = flyer_successor.revised_retained_arm()
    thickness = float(solution["front_trim_common_thickness_mm"])
    slugs = [
        flyer_successor.front_trim_slug(x_mm, thickness)
        for x_mm in flyer_successor.FRONT_TRIM_X_MM
    ]
    hardware_rows = flyer_successor.front_trim_hardware(thickness)
    washers = hardware_rows[0::3]
    screws = hardware_rows[1::3]
    inserts = hardware_rows[2::3]
    screw_tip_z = (
        flyer_successor.FRONT_TRIM_SEAT_Z_MM + thickness
        + flyer_successor.FRONT_TRIM_WASHER_THICKNESS_MM
        - flyer_successor.FRONT_TRIM_M2_LENGTH_MM
    )
    screw_tip_clearance = (
        screw_tip_z - flyer_successor.FRONT_TRIM_PILOT_BOTTOM_Z_MM
    )
    blind_material = (
        flyer_successor.FRONT_TRIM_PILOT_BOTTOM_Z_MM
        - float(retained.base.SPOKE_REAR_Z_MM)
    )
    outer_wall = (
        retained.base.SPOKE_WIDTH_MM / 2.0
        - (
            abs(flyer_successor.FRONT_TRIM_X_MM[1])
            + flyer_successor.M2_INSERT_PILOT_RADIUS_MM
        )
    )
    front_stacks: list[dict[str, Any]] = []
    for index, (x_mm, slug, washer, screw, insert) in enumerate(zip(
        flyer_successor.FRONT_TRIM_X_MM,
        slugs,
        washers,
        screws,
        inserts,
    ), start=1):
        screw_box = screw.bounding_box()
        insert_box = insert.bounding_box()
        screw_axis_xy = np.asarray([
            (float(screw_box.min.X) + float(screw_box.max.X)) / 2.0,
            (float(screw_box.min.Y) + float(screw_box.max.Y)) / 2.0,
        ])
        insert_axis_xy = np.asarray([
            (float(insert_box.min.X) + float(insert_box.max.X)) / 2.0,
            (float(insert_box.min.Y) + float(insert_box.max.Y)) / 2.0,
        ])
        axial_thread_engagement = max(0.0, min(
            float(screw_box.max.Z), float(insert_box.max.Z)
        ) - max(float(screw_box.min.Z), float(insert_box.min.Z)))
        front_stacks.append({
            "id": f"front_trim_{index}",
            "center_xy_mm": [float(x_mm), flyer_successor.FRONT_TRIM_Y_MM],
            "slug_thickness_mm": thickness,
            "slug_to_printed_spoke_seat_distance_mm": _distance(slug, arm),
            "washer_to_slug_distance_mm": _distance(washer, slug),
            "screw_head_to_washer_distance_mm": _distance(screw, washer),
            "simplified_thread_envelope_radial_clearance_mm": _distance(
                screw, insert
            ),
            "screw_insert_axis_concentricity_error_mm": float(
                np.linalg.norm(screw_axis_xy - insert_axis_xy)
            ),
            "screw_insert_axial_thread_engagement_mm": (
                axial_thread_engagement
            ),
            "screw_to_printed_arm_intersection_mm3": _overlap(screw, arm),
            "insert_to_printed_arm_unintended_intersection_mm3": _overlap(
                insert, arm
            ),
            "full_insert_engagement_mm": (
                flyer_successor.FRONT_TRIM_INSERT_TOP_Z_MM
                - flyer_successor.FRONT_TRIM_INSERT_BOTTOM_Z_MM
            ),
            "screw_tip_clearance_behind_insert_mm": screw_tip_clearance,
            "blind_printed_material_behind_pilot_mm": blind_material,
            "minimum_outer_radial_printed_wall_mm": outer_wall,
            "fastener_terminates_in_positive_blind_printed_material": (
                screw_tip_clearance >= 0.5
                and blind_material >= 2.4
                and outer_wall >= 1.5
            ),
            "closed_structural_load_path": (
                "M2x8 socket head -> M2 plain washer -> OD6/ID2.2 B777 "
                "annular trim seated on the main PETG spoke -> full 4 mm "
                "standard M2 heat-set insert in a blind printed pilot -> "
                f"{blind_material:.2f} mm positive printed material behind "
                "the pilot"
            ),
            "physical_pull_proof_complete": False,
        })

    all_front_attached = all(
        row["slug_to_printed_spoke_seat_distance_mm"] <= CONTACT_TOL_MM
        and row["washer_to_slug_distance_mm"] <= CONTACT_TOL_MM
        and row["screw_head_to_washer_distance_mm"] <= CONTACT_TOL_MM
        and row["screw_insert_axis_concentricity_error_mm"] <= 1.0e-5
        and row["screw_insert_axial_thread_engagement_mm"] >= 4.0 - 1.0e-6
        and row["screw_to_printed_arm_intersection_mm3"] <= BOOLEAN_TOL_MM3
        and row["insert_to_printed_arm_unintended_intersection_mm3"]
        <= BOOLEAN_TOL_MM3
        and row["fastener_terminates_in_positive_blind_printed_material"]
        for row in front_stacks
    )
    all_rear_attached = (
        rear["stack_count"] == 4
        and rear["all_screws_end_in_positive_blind_material"]
        and rear["all_stacks_within_pocket_axial_envelope"]
        and rear["all_caps_and_posts_single_solid"]
        and all(
            row["slug_to_retainer_contact_distance_mm"] <= CONTACT_TOL_MM
            and row["fastener_terminates_in_positive_blind_material"]
            for row in rear["stacks"]
        )
    )
    return {
        "status": (
            "GEOMETRY_PASS_PHYSICAL_PULL_OPEN"
            if all_front_attached and all_rear_attached
            else "FAIL_OPEN_AIR_OR_DISCONNECTED_BALANCE_STACK"
        ),
        "stack_count": 6,
        "rear_M3_retained_stacks": rear,
        "front_M2_blind_spoke_stacks": front_stacks,
        "all_four_rear_stacks_have_closed_printed_boss_load_paths": (
            all_rear_attached
        ),
        "both_front_stacks_thread_into_blind_spoke_inserts": (
            all_front_attached
        ),
        "all_six_screws_terminate_in_positive_printed_material": (
            all_front_attached and all_rear_attached
        ),
        "any_balance_fastener_over_open_air": not (
            all_front_attached and all_rear_attached
        ),
        "physical_pull_proof_complete": False,
    }


def _find(parts: Iterable[Part], label: str) -> Part:
    for part in parts:
        if getattr(part, "label", "") == label:
            return part
    raise KeyError(label)


def pre_terminal_integrated_m2_torque_audit(
    integrated_rotating_izz_kgm2: float,
) -> dict[str, Any]:
    """Rebind the conservative pre-terminal 36 V gate to the exact flyer J."""

    selection = _contract_reports()["m2_selection"]
    prior = selection["OD65_10N_full_inertia_torque"]
    components = prior["components_kgm2"]
    pulley_authority = selection["motor_pulley_geometry_and_inertia"]
    official_stock_j = float(
        nbk_p30.OFFICIAL_AXIAL_MOMENT_OF_INERTIA_KG_M2
    )
    if not math.isclose(
        float(pulley_authority["published_stock_inertia_kgm2"]),
        official_stock_j,
        rel_tol=0.0,
        abs_tol=1.0e-15,
    ):
        raise ValueError("governing selection and official P30 J disagree")
    bnw_two_screw_upper_j = float(
        pulley_authority["BNW_inertia_bound"][
            "two_bound_screws_inertia_kgm2"
        ]
    )
    added = {
        "official_NBK_P30_stock_complete_assembly": official_stock_j,
        "unreleased_BNW_two_M3x12_screw_upper_bound": (
            bnw_two_screw_upper_j
        ),
        "210_3GT_6_belt": float(components["210_3GT_6_belt"]),
        "Leadshine_rotor": float(components["Leadshine_rotor"]),
        "two_complete_6001_bearings_upper_bound": float(
            components["two_complete_6001_bearings_upper_bound"]
        ),
    }
    full_j = float(integrated_rotating_izz_kgm2) + sum(added.values())
    alpha = float(prior["angular_acceleration_rad_s2"])
    acceleration_torque = full_j * alpha
    required = (
        float(prior["wire_torque_nm"])
        + float(prior["friction_allowance_nm_unmeasured"])
        + acceleration_torque
    )
    available_36 = float(prior["available_36V_lower_edge_nm"])
    available_24 = float(selection["motor"]["24V_lower_edge_300rpm_nm"])
    margin_36 = available_36 / required
    margin_24 = available_24 / required
    transmission_capacity = float(
        selection["transmission"]["allowable_transmission_torque_nm"]
    )
    return {
        "status": "PROVISIONAL_PASS_AWAITING_TERMINAL_GUIDE_LIVE_LINE_MOMENT_ARM",
        "governing_selection_report": "out/reports/m2_normal_goal_drive_selection.json",
        "governing_selection_report_sha256": _sha256(M2_SELECTION_REPORT),
        "governing_selection_source": "sim/m2_normal_goal_drive_selection.py",
        "governing_selection_source_sha256": _sha256(M2_SELECTION_SOURCE),
        "old_m2_drive_successor_review_is_governing": False,
        "motor_pulley_mass_and_J_authority": {
            "source": "NBK P30-3GT-BLP-6C-5 product table",
            "official_stock_mass_g": nbk_p30.OFFICIAL_MASS_G,
            "official_stock_axial_J_kgm2": official_stock_j,
            "official_stock_J_includes_stock_split_clamp_assembly": True,
            "separate_stock_M2_bolt_witness_J_added": False,
            "BNW_two_M3x12_review_upper_bound_J_kgm2": (
                bnw_two_screw_upper_j
            ),
            "governing_report_old_combined_component_kgm2": float(
                components[
                    "motor_P30_stock_split_clamp_BNW_and_three_screw_upper_bound"
                ]
            ),
            "governing_report_stock_M2_witness_J_not_reused_kgm2": float(
                pulley_authority[
                    "stock_M2_clamp_bolt_inertia_witness_kgm2"
                ]
            ),
        },
        "wire_torque_input_scope": (
            "pre-terminal-guide 0.325 Nm bound from governing M2 selection"
        ),
        "terminal_guide_2400_locus_max_perpendicular_moment_arm_consumed": False,
        "coupled_final_motor_gate_ge_2x": False,
        "integrated_rotating_izz_including_flyer_P30_and_screws_kgm2": float(
            integrated_rotating_izz_kgm2
        ),
        "added_output_referred_components_kgm2": added,
        "full_output_inertia_kgm2": full_j,
        "angular_acceleration_rad_s2": alpha,
        "acceleration_torque_nm": acceleration_torque,
        "wire_torque_nm": float(prior["wire_torque_nm"]),
        "friction_allowance_nm_unmeasured": float(
            prior["friction_allowance_nm_unmeasured"]
        ),
        "required_output_torque_nm": required,
        "required_2x_running_torque_nm": 2.0 * required,
        "Leadshine_36V_lower_edge_nm": available_36,
        "Leadshine_36V_available_to_required_multiple": margin_36,
        "Leadshine_36V_gate_ge_2x": margin_36 >= 2.0,
        "Leadshine_24V_lower_edge_nm": available_24,
        "Leadshine_24V_available_to_required_multiple": margin_24,
        "Leadshine_24V_gate_ge_2x": margin_24 >= 2.0,
        "P30_210_3GT_allowable_transmission_torque_nm": transmission_capacity,
        "P30_210_3GT_available_to_required_multiple": (
            transmission_capacity / required
        ),
        "P30_210_3GT_gate_ge_2x": transmission_capacity / required >= 2.0,
        "motor_pulley_BNW_retention_release_gate": False,
        "flyer_pulley_retention_release_gate": False,
        "installed_friction_measurement_gate": False,
        "hot_dyno_gate": False,
    }


def final_integrated_m2_torque_audit(
    integrated_rotating_izz_kgm2: float,
) -> dict[str, Any]:
    """Consume the exact 2,400-locus bell-to-capture live-line load.

    The active-sector audit deliberately calls neither this wrapper nor the
    integrated report loader: it derives the same drivetrain constants from
    the pinned M2 selection and substitutes the exact current flyer inertia.
    That one-way seam avoids a report cycle while this final candidate binds
    the completed active-sector report fail closed.
    """

    pre = pre_terminal_integrated_m2_torque_audit(
        integrated_rotating_izz_kgm2
    )
    active = _contract_reports()["active_sector"]
    loads = active["coupled_live_line_loads"]
    m2 = loads["M2"]
    if not math.isclose(
        float(m2["integrated_rotating_izz_kg_m2"]),
        float(integrated_rotating_izz_kgm2),
        rel_tol=0.0,
        abs_tol=1.0e-14,
    ):
        raise ValueError("active-sector report flyer inertia drift")
    if not math.isclose(
        float(m2["full_output_inertia_kg_m2"]),
        float(pre["full_output_inertia_kgm2"]),
        rel_tol=0.0,
        abs_tol=1.0e-14,
    ):
        raise ValueError("active-sector report full output inertia drift")
    if not math.isclose(
        float(m2["angular_acceleration_rad_s2"]),
        float(pre["angular_acceleration_rad_s2"]),
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise ValueError("active-sector report acceleration contract drift")
    if not math.isclose(
        float(m2["friction_allowance_nm"]),
        float(pre["friction_allowance_nm_unmeasured"]),
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise ValueError("active-sector report friction contract drift")

    result = deepcopy(pre)
    result.update({
        "status": (
            "COUPLED_MECHANICAL_MARGIN_PASS__DRIVER_CONFIG_HOT_DYNO_"
            "AND_PHYSICAL_GATES_OPEN"
        ),
        "active_sector_report": (
            "out/reports/carriage_active_sector_terminal_guide_audit.json"
        ),
        "active_sector_report_file_sha256": _sha256(ACTIVE_SECTOR_REPORT),
        "active_sector_report_sha256": active["report_sha256"],
        "wire_torque_input_scope": (
            "exact maximum perpendicular 10 N live-line lever from the "
            "physical polished PEEK bell exit to the M0-following capture "
            "across all 2400 deposition loci"
        ),
        "terminal_guide_2400_locus_max_perpendicular_moment_arm_consumed": True,
        "maximum_perpendicular_live_line_lever_mm": float(
            m2["maximum_perpendicular_live_line_lever_mm"]
        ),
        "wire_torque_nm": float(m2["wire_torque_at_10N_nm"]),
        "full_output_inertia_kgm2": float(
            m2["full_output_inertia_kg_m2"]
        ),
        "acceleration_torque_nm": float(m2["acceleration_torque_nm"]),
        "required_output_torque_nm": float(
            m2["required_output_torque_nm"]
        ),
        "required_2x_running_torque_nm": float(
            m2["required_2x_running_torque_nm"]
        ),
        "Leadshine_36V_available_to_required_multiple": float(
            m2["Leadshine_36V_available_to_required_multiple"]
        ),
        "Leadshine_36V_gate_ge_2x": bool(
            m2["Leadshine_36V_gate_ge_2x"]
        ),
        "Leadshine_24V_lower_edge_nm": float(
            m2["Leadshine_24V_lower_edge_nm"]
        ),
        "Leadshine_24V_available_to_required_multiple": float(
            m2["Leadshine_24V_available_to_required_multiple"]
        ),
        "Leadshine_24V_gate_ge_2x": bool(
            m2["Leadshine_24V_gate_ge_2x"]
        ),
        "Leadshine_24V_release_authorized": bool(
            m2["Leadshine_24V_release_authorized"]
        ),
        "P30_210_3GT_available_to_required_multiple": float(
            m2["P30_210_3GT_available_to_required_multiple"]
        ),
        "P30_210_3GT_gate_ge_2x": bool(
            m2["P30_210_3GT_gate_ge_2x"]
        ),
        "coupled_final_motor_gate_ge_2x": bool(
            m2["Leadshine_36V_gate_ge_2x"]
        ),
        "theoretical_curve_gate_scope": m2["theoretical_curve_gate_scope"],
        "driver_36V_current_microstep_limits_configured_and_verified": bool(
            m2["driver_36V_current_microstep_limits_configured_and_verified"]
        ),
        "installed_hot_dyno_verified": bool(
            m2["installed_hot_dyno_verified"]
        ),
        "coupled_axis_loads": {
            "M1": deepcopy(loads["M1"]),
            "M0": deepcopy(loads["M0"]),
        },
    })
    result["hot_dyno_gate"] = result["installed_hot_dyno_verified"]
    return result


def geometry_audit() -> dict[str, Any]:
    reports = _contract_reports()
    static_groups = main_static_groups()
    frame_attachments = frame_relocation_attachment_audit(static_groups)
    entry_attachments = entry_module_attachment_audit(static_groups)
    drive_parts = successor_drive_parts()
    official_p30 = official_motor_pulley_review()
    official_flyer_p30 = official_flyer_pulley_review()
    rotating = retained_rotating_parts()
    spindle_context, stator_home = spindle_without_stator()
    cap_home_parts = cap_module_parts()
    cap_deep_parts = cap_module_parts(retained.base.deepest_axis_z_mm())

    cap_home = _compound(cap_home_parts[:2], "home_cap_pair")
    cap_deep = _compound(cap_deep_parts[:2], "deep_cap_pair")
    cap_hw_home = _compound(cap_home_parts[2:], "home_cap_hardware")
    cap_hw_deep = _compound(cap_deep_parts[2:], "deep_cap_hardware")
    arm = rotating["retained_arm"]
    shaft = rotating["shaft"]
    pulley = rotating["flyer_pulley"]
    guide = rotating["flyer_PEEK_guide"]
    belt = drive_parts["belt"]
    bnw_screws = [
        drive_parts[key]
        for key in sorted(drive_parts)
        if key.startswith("motor_pulley_BNW_set_screw_")
    ]
    bnw_holes = [
        drive_parts[key]
        for key in sorted(drive_parts)
        if key.startswith("motor_pulley_BNW_hole_path_")
    ]
    arm_screws = [
        rotating[key]
        for key in sorted(rotating)
        if key.startswith("shaft_clamp_") and "set_screw" in key
    ]
    wire = configured_static_supply_wire()
    flyer_wire = flyer_successor.guide_wire_envelope(
        float(DEFAULT_STATOR.wire_d),
        "configured_wire_inside_one_piece_PEEK_guide_and_exit_bell",
    )
    shaft_brep = released_shaft_brep_audit(shaft)
    shaft_front_interface = released_shaft_front_interface_audit(
        shaft, guide, flyer_wire,
    )
    arm_root = arm_root_sleeve_load_path_audit(arm, shaft, flyer_wire)
    shifted_support = static_groups["shifted_support"]
    shifted_block = _find(shifted_support, "flyer_block")
    shaft_bearings = [
        _find(shifted_support, "flyer_6001_front"),
        _find(shifted_support, "flyer_6001_rear"),
    ]
    entry_bracket = _find(static_groups["shifted_entry"], "entry_bracket")
    entry_eyelet = _find(static_groups["shifted_entry"], "entry_eyelet")
    felt_fixed = _find(static_groups["unchanged"], "felt_pad_fixed")
    felt_moving = _find(static_groups["unchanged"], "felt_pad_moving")
    dancer = _find(static_groups["unchanged"], "dancer_pulley")

    unintended = {
        "actual_deep_cap_pair_vs_retained_arm_mm3": _overlap(cap_deep, arm),
        "actual_deep_cap_hardware_vs_retained_arm_mm3": _overlap(cap_hw_deep, arm),
        "home_cap_pair_vs_retained_arm_mm3": _overlap(cap_home, arm),
        "home_cap_hardware_vs_retained_arm_mm3": _overlap(cap_hw_home, arm),
        "home_cap_pair_vs_spindle_holder_context_mm3": _overlap(
            cap_home, _compound(spindle_context, "spindle_context")
        ),
        "retained_arm_vs_shifted_flyer_block_mm3": _overlap(
            arm, shifted_block
        ),
        "flyer_pulley_vs_shifted_flyer_block_mm3": _overlap(
            pulley, shifted_block
        ),
        "static_wire_vs_drive_motor_mm3": _overlap(wire, drive_parts["motor"]),
        "static_wire_vs_drive_belt_mm3": _overlap(wire, belt),
        "static_wire_vs_motor_pulley_mm3": _overlap(
            wire, drive_parts["motor_pulley"]
        ),
        "static_wire_vs_flyer_pulley_mm3": _overlap(wire, pulley),
        "static_wire_vs_extended_hollow_shaft_mm3": _overlap(wire, shaft),
        "static_axis_wire_vs_corrected_retained_arm_mm3": _overlap(
            wire, arm
        ),
        "front_root_wire_vs_released_shaft_mm3": _overlap(
            flyer_wire, shaft
        ),
        "released_shaft_vs_PEEK_guide_outer_mm3": (
            shaft_front_interface[
                "shaft_vs_PEEK_guide_outer_overlap_mm3"
            ]
        ),
        "front_root_wire_vs_corrected_retained_arm_mm3": _overlap(
            flyer_wire, arm
        ),
        "corrected_retained_arm_vs_released_shaft_mm3": _overlap(
            arm, shaft
        ),
        "arm_M3x8_set_screws_vs_released_shaft_max_mm3": max(
            _overlap(screw, shaft) for screw in arm_screws
        ),
        "shaft_vs_6001_bearings_max_mm3": max(
            _overlap(shaft, bearing) for bearing in shaft_bearings
        ),
        "belt_vs_successor_mount_mm3": _overlap(belt, drive_parts["mount"]),
        "belt_vs_shifted_flyer_block_mm3": _overlap(belt, shifted_block),
        "BNW_set_screws_vs_belt_max_mm3": max(
            _overlap(screw, belt) for screw in bnw_screws
        ),
        "BNW_hole_path_witnesses_vs_belt_max_mm3": max(
            _overlap(hole, belt) for hole in bnw_holes
        ),
        "BNW_set_screw_witnesses_vs_exact_Leadshine_shaft_max_mm3": max(
            _overlap(screw, drive_parts["motor"])
            for screw in bnw_screws
        ),
        "extended_hollow_shaft_vs_entry_bracket_mm3": _overlap(
            shaft, entry_bracket
        ),
        "flyer_P30_vs_entry_bracket_mm3": _overlap(
            pulley, entry_bracket
        ),
        "configured_wire_vs_entry_bracket_wall_mm3": _overlap(
            wire, entry_bracket
        ),
        "entry_bracket_vs_drive_motor_mm3": _overlap(
            entry_bracket, drive_parts["motor"]
        ),
        "entry_bracket_vs_drive_belt_mm3": _overlap(
            entry_bracket, belt
        ),
    }
    intended = {
        "belt_to_motor_pulley_overlap_mm3": _overlap(
            belt, drive_parts["motor_pulley"]
        ),
        "belt_to_flyer_pulley_overlap_mm3": _overlap(belt, pulley),
        "motor_pulley_to_exact_Leadshine_D_shaft_distance_mm": _distance(
            drive_parts["motor_pulley"], drive_parts["motor"]
        ),
        "BNW_set_screw_to_exact_Leadshine_shaft_distances_mm": [
            _distance(screw, drive_parts["motor"])
            for screw in bnw_screws
        ],
        "BNW_hole_path_to_exact_Leadshine_shaft_distances_mm": [
            _distance(hole, drive_parts["motor"])
            for hole in bnw_holes
        ],
        "BNW_hole_path_to_official_stock_overlap_mm3": [
            _overlap(hole, drive_parts["motor_pulley"])
            for hole in bnw_holes
        ],
        "BNW_set_screw_to_official_stock_overlap_mm3": [
            _overlap(screw, drive_parts["motor_pulley"])
            for screw in bnw_screws
        ],
        "BNW_hole_path_to_matching_screw_distances_mm": [
            _distance(hole, screw)
            for hole, screw in zip(bnw_holes, bnw_screws)
        ],
        "BNW_hole_path_to_matching_screw_overlap_mm3": [
            _overlap(hole, screw)
            for hole, screw in zip(bnw_holes, bnw_screws)
        ],
        "front_cap_to_stator_distance_mm": _distance(cap_home_parts[0], stator_home),
        "rear_cap_to_stator_distance_mm": _distance(cap_home_parts[1], stator_home),
        "wire_to_fixed_felt_distance_mm": _distance(wire, felt_fixed),
        "wire_to_moving_felt_distance_mm": _distance(wire, felt_moving),
        "wire_to_dancer_pulley_distance_mm": _distance(wire, dancer),
        "static_axis_wire_to_flyer_guide_wire_distance_mm": _distance(
            wire, flyer_wire
        ),
        "static_axis_wire_to_flyer_guide_wire_overlap_mm3": _overlap(
            wire, flyer_wire
        ),
        "arm_M3x8_to_shaft_flat_distances_mm": [
            _distance(screw, shaft) for screw in arm_screws
        ],
        "entry_bracket_to_rear_post_distance_mm": entry_attachments[
            "entry_bracket_to_rear_post_distance_mm"
        ],
        "entry_eyelet_to_bracket_distance_mm": entry_attachments[
            "entry_eyelet_to_bracket_distance_mm"
        ],
        "dancer_fixed_screw_to_integral_entry_keeper_distance_mm": (
            entry_attachments[
                "preserved_dancer_fixed_screw_to_bracket_distance_mm"
            ]
        ),
    }
    clearances = {
        "actual_deep_cap_pair_to_retained_arm_mm": _distance(cap_deep, arm),
        "retained_arm_to_shifted_flyer_block_mm": _distance(arm, shifted_block),
        "flyer_pulley_to_shifted_flyer_block_mm": _distance(pulley, shifted_block),
        "static_wire_to_drive_motor_mm": _distance(wire, drive_parts["motor"]),
        "static_wire_to_drive_belt_mm": _distance(wire, belt),
        "static_wire_to_flyer_pulley_mm": _distance(wire, pulley),
        "static_wire_to_hollow_shaft_wall_mm": _distance(wire, shaft),
        "static_axis_wire_to_corrected_retained_arm_mm": _distance(
            wire, arm
        ),
        "front_root_wire_to_released_shaft_mm": _distance(
            flyer_wire, shaft
        ),
        "released_shaft_to_PEEK_guide_outer_mm": (
            shaft_front_interface[
                "shaft_to_PEEK_guide_outer_distance_mm"
            ]
        ),
        "corrected_retained_arm_to_released_shaft_mm": _distance(
            arm, shaft
        ),
        "root_sleeve_web_to_actual_job_wire_mm": arm_root[
            "root_sleeve_web"
        ]["web_to_actual_job_wire_clearance_mm"],
        "belt_to_BNW_set_screws_min_mm": min(
            _distance(belt, screw) for screw in bnw_screws
        ),
        "belt_to_BNW_hole_path_witnesses_min_mm": min(
            _distance(belt, hole) for hole in bnw_holes
        ),
        "extended_hollow_shaft_to_entry_bracket_mm": _distance(
            shaft, entry_bracket
        ),
        "flyer_P30_to_entry_bracket_mm": _distance(
            pulley, entry_bracket
        ),
        "extended_hollow_shaft_to_entry_eyelet_mm": _distance(
            shaft, entry_eyelet
        ),
        "flyer_P30_to_entry_eyelet_mm": _distance(
            pulley, entry_eyelet
        ),
        "configured_wire_to_entry_passage_wall_mm": _distance(
            wire, entry_bracket
        ),
        "entry_bracket_to_drive_motor_mm": _distance(
            entry_bracket, drive_parts["motor"]
        ),
        "entry_bracket_to_drive_belt_mm": _distance(
            entry_bracket, belt
        ),
    }
    entry_eyelet_center_z = P.wire_entry_z + 3.0 - ENTRY_REAR_SHIFT_MM
    prior_shaft_rear_z = P.flyer_shaft_rear_z - M2_REAR_SHIFT_MM
    prior_bridge_length = (
        prior_shaft_rear_z
        - (P.wire_entry_z + 3.0 - ENTRY_PRIOR_REAR_SHIFT_MM)
    )
    bridge_length = (
        flyer_shaft_d10.WORLD_REAR_Z_MM - entry_eyelet_center_z
    )
    entry_wire_bridge = {
        "entry_eyelet_center_z_mm": entry_eyelet_center_z,
        "shifted_hollow_shaft_bore_rear_z_mm": (
            flyer_shaft_d10.WORLD_REAR_Z_MM
        ),
        "straight_axial_bridge_length_mm": bridge_length,
        "prior_candidate_shaft_rear_z_mm": prior_shaft_rear_z,
        "prior_candidate_bridge_length_mm": prior_bridge_length,
        "entry_additional_rear_shift_mm": ENTRY_ADDITIONAL_REAR_SHIFT_MM,
        "shaft_additional_rear_extension_mm": (
            prior_shaft_rear_z - flyer_shaft_d10.WORLD_REAR_Z_MM
        ),
        "net_bridge_length_change_mm": bridge_length - prior_bridge_length,
        "passage_tool_radius_mm": ENTRY_PASSAGE_RADIUS_MM,
        "configured_wire_radius_mm": wire_vis.R_VIS,
        "radial_wall_clearance_exact_BREP_mm": clearances[
            "configured_wire_to_entry_passage_wall_mm"
        ],
        "wire_to_entry_wall_overlap_mm3": unintended[
            "configured_wire_vs_entry_bracket_wall_mm3"
        ],
        "centerline_bend_radii_unchanged": True,
        "handoff_is_collinear_with_hollow_shaft_axis": True,
    }
    bnw_screw_radial_starts = [
        nbk_p30.BNW_WITNESS_SCREW_RADIAL_START_MM - inward
        for inward in NBK_P30_BNW_SCREW_INWARD_ADJUSTMENTS_MM
    ]
    bnw_screw_radial_ends = [
        nbk_p30.BNW_WITNESS_SCREW_RADIAL_END_MM - inward
        for inward in NBK_P30_BNW_SCREW_INWARD_ADJUSTMENTS_MM
    ]
    bnw_socket_packaging = {
        "full_cylinder_upper_bound_intentionally_retains_socket_recess_material": True,
        "socket_end_is_outer_radial_end_datum": True,
        "screw_inward_adjustments_mm": list(
            NBK_P30_BNW_SCREW_INWARD_ADJUSTMENTS_MM
        ),
        "effective_radial_start_mm": bnw_screw_radial_starts,
        "effective_socket_end_radius_mm": bnw_screw_radial_ends,
        "M3x12_length_preserved_mm": [
            end - start
            for start, end in zip(
                bnw_screw_radial_starts, bnw_screw_radial_ends
            )
        ],
        "socket_end_protrusion_beyond_published_E23_clamp_radius_mm": [
            end - drive.NBK_CLAMP_ENVELOPE_E_MM / 2.0
            for end in bnw_screw_radial_ends
        ],
        "socket_end_reserve_inside_OD32_max_radius_mm": [
            nbk_p30.SOURCE_FLANGE_DIAMETER_MM / 2.0 - end
            for end in bnw_screw_radial_ends
        ],
        "belt_clearance_min_mm": clearances["belt_to_BNW_set_screws_min_mm"],
        "configured_socket_recess_depth_or_hex_size_claimed": False,
    }

    arm_inserts = {
        key: shape for key, shape in rotating.items()
        if key.startswith("shaft_clamp_") and "insert" in key
    }
    arm_screw_packaging_rows: list[dict[str, Any]] = []
    for screw in arm_screws:
        name = str(screw.label)
        if "neg_y" in name:
            insert = next(
                shape for key, shape in arm_inserts.items()
                if "neg_y" in key
            )
            screw_interval = (
                float(screw.bounding_box().min.Y),
                float(screw.bounding_box().max.Y),
            )
            insert_interval = (
                float(insert.bounding_box().min.Y),
                float(insert.bounding_box().max.Y),
            )
            outer_radius = abs(screw_interval[0])
        else:
            insert = next(
                shape for key, shape in arm_inserts.items()
                if "pos_x" in key
            )
            screw_interval = (
                float(screw.bounding_box().min.X),
                float(screw.bounding_box().max.X),
            )
            insert_interval = (
                float(insert.bounding_box().min.X),
                float(insert.bounding_box().max.X),
            )
            outer_radius = screw_interval[1]
        arm_screw_packaging_rows.append({
            "screw_label": name,
            "insert_label": str(insert.label),
            "screw_axis_interval_mm": list(screw_interval),
            "insert_axis_interval_mm": list(insert_interval),
            "M3x8_length_preserved_mm": (
                screw_interval[1] - screw_interval[0]
            ),
            "outer_socket_end_radius_mm": outer_radius,
            "projected_screw_insert_engagement_mm": max(
                0.0,
                min(screw_interval[1], insert_interval[1])
                - max(screw_interval[0], insert_interval[0]),
            ),
        })
    arm_screw_packaging = {
        "screw_inward_adjustment_mm": ARM_M3X8_SCREW_INWARD_ADJUSTMENT_MM,
        "flat_contact_radius_mm": 5.7,
        "rows": arm_screw_packaging_rows,
    }
    shaft_fit = {
        "6001_bearings": {
            "labels": [str(part.label) for part in shaft_bearings],
            "distances_mm": [
                _distance(shaft, part) for part in shaft_bearings
            ],
            "overlaps_mm3": [
                _overlap(shaft, part) for part in shaft_bearings
            ],
            "nominal_bore_diameter_mm": 12.0,
        },
        "inner_race_spacers": {
            name: {
                "nominal_bore_diameter_mm": 12.05,
                "radial_clearance_mm": _distance(shaft, rotating[name]),
                "overlap_mm3": _overlap(shaft, rotating[name]),
            }
            for name in (
                "m2_inner_rear_shim",
                "m2_inner_center_spacer",
                "m2_inner_front_spacer",
            )
        },
        "flyer_P30_stock_D10_clamp": {
            "official_part_number": nbk_p30_d10.OFFICIAL_PART_NUMBER,
            "nominal_bore_diameter_mm": (
                nbk_p30_d10.SOURCE_BORE_DIAMETER_MM
            ),
            "shaft_seat_outer_diameter_mm": flyer_shaft_d10.NECK_OD_MM,
            "shaft_seat_tolerance": RELEASED_SHAFT_D10_SEAT_TOLERANCE,
            "contact_distance_mm": _distance(shaft, pulley),
            "overlap_mm3": _overlap(shaft, pulley),
            "pulley_axial_span_z_mm": [
                float(pulley.bounding_box().min.Z),
                float(pulley.bounding_box().max.Z),
            ],
            "shaft_D10_seat_axial_span_z_mm": [
                flyer_shaft_d10.WORLD_REAR_Z_MM,
                flyer_shaft_d10.SHOULDER_WORLD_Z_MM,
            ],
            "D10_seat_length_mm": flyer_shaft_d10.NECK_LENGTH_MM,
            "stock_clamp_length_mm": nbk_p30_d10.STOCK_CLAMP_LENGTH_MM,
            "supplied_clamp_bolt": nbk_p30_d10.OFFICIAL_CLAMP_BOLT,
            "supplied_clamp_torque_Nm": (
                nbk_p30_d10.OFFICIAL_CLAMP_TORQUE_NM
            ),
        },
        "DIN988_shim": {
            "nominal_bore_diameter_mm": 12.0,
            "distance_mm": _distance(shaft, rotating["DIN988_shim"]),
            "overlap_mm3": _overlap(shaft, rotating["DIN988_shim"]),
        },
        "printed_arm_post_ream_bore": arm_root["corrected_interface"],
    }
    shaft_wire_handoffs = {
        "rear_entry": {
            "shaft_mouth_z_mm": flyer_shaft_d10.WORLD_REAR_Z_MM,
            "shaft_mouth_inner_diameter_mm": flyer_shaft_d10.NECK_ID_MM,
            "wire_bbox_mm": _bbox(wire),
            "wire_to_shaft_wall_clearance_mm": _distance(wire, shaft),
            "analytic_configured_wire_radial_clearance_mm": (
                flyer_shaft_d10.NECK_ID_MM / 2.0 - wire_vis.R_VIS
            ),
            "analytic_0p5mm_wire_radial_clearance_mm": (
                flyer_shaft_d10.NECK_ID_MM / 2.0
                - wire_geometry.WIRE_RADIUS_MAX
            ),
            "wire_vs_shaft_overlap_mm3": _overlap(wire, shaft),
            "collinear_with_shaft_axis": True,
            "R0p50_internal_mouth_present": (
                shaft_brep["wire_mouth_toroidal_face_count"] == 2
            ),
        },
        "front_root": {
            "shaft_front_z_mm": flyer_shaft_d10.WORLD_FRONT_Z_MM,
            "root_sleeve_front_z_mm": shaft_front_interface[
                "root_sleeve_front_z_mm"
            ],
            "shaft_front_setback_from_root_sleeve_front_mm": (
                shaft_front_interface[
                    "shaft_front_setback_from_root_sleeve_front_mm"
                ]
            ),
            "minimum_required_setback_mm": shaft_front_interface[
                "minimum_required_setback_mm"
            ],
            "shaft_mouth_inner_diameter_mm": flyer_shaft_d10.MAIN_ID_MM,
            "retained_wire_root_plane_z_mm": (
                retained.base.SPOKE_FRONT_Z_MM
            ),
            "shaft_front_extension_past_root_plane_mm": (
                flyer_shaft_d10.WORLD_FRONT_Z_MM
                - retained.base.SPOKE_FRONT_Z_MM
            ),
            "wire_to_shaft_distance_mm": _distance(flyer_wire, shaft),
            "wire_vs_shaft_overlap_mm3": _overlap(flyer_wire, shaft),
            "PEEK_guide_outer_to_shaft_distance_mm": (
                shaft_front_interface[
                    "shaft_to_PEEK_guide_outer_distance_mm"
                ]
            ),
            "PEEK_guide_outer_vs_shaft_overlap_mm3": (
                shaft_front_interface[
                    "shaft_vs_PEEK_guide_outer_overlap_mm3"
                ]
            ),
            "wire_vs_corrected_arm_overlap_mm3": _overlap(flyer_wire, arm),
            "root_web_to_job_wire_clearance_mm": arm_root[
                "root_sleeve_web"
            ]["web_to_actual_job_wire_clearance_mm"],
            "root_web_to_0p5mm_wire_clearance_mm": arm_root[
                "root_sleeve_web"
            ]["analytic_0p5mm_wire_clearance_mm"],
            "continuous_moving_conductor_claimed": False,
        },
        "axis_ownership_seam": {
            "static_axis_run_rear_mouth_z_mm": (
                flyer_shaft_d10.WORLD_REAR_Z_MM
            ),
            "static_axis_run_centerline_end_z_mm": (
                flyer_successor.GUIDE_ROOT_AXIAL_START_Z_MM
            ),
            "flyer_guide_centerline_start_z_mm": (
                flyer_successor.GUIDE_ROOT_AXIAL_START_Z_MM
            ),
            "centerline_gap_mm": 0.0,
            "static_to_flyer_wire_distance_mm": intended[
                "static_axis_wire_to_flyer_guide_wire_distance_mm"
            ],
            "static_to_flyer_wire_overlap_mm3": intended[
                "static_axis_wire_to_flyer_guide_wire_overlap_mm3"
            ],
            "static_axis_wire_vs_shaft_wall_overlap_mm3": unintended[
                "static_wire_vs_extended_hollow_shaft_mm3"
            ],
            "static_axis_wire_vs_corrected_arm_overlap_mm3": unintended[
                "static_axis_wire_vs_corrected_retained_arm_mm3"
            ],
            "axis_segment_is_invariant_under_M2_rotation": True,
        },
    }

    motor_rear_z = float(drive_parts["motor"].bounding_box().min.Z)
    current_boundary_z = float(P.frame_z0)
    selected_boundary_z = (
        current_boundary_z - INTEGRATED_FRAME_WINDOW_REAR_SHIFT_MM
    )
    selected_boundary_clearance = motor_rear_z - selected_boundary_z
    required_boundary_z = motor_rear_z - MIN_REVIEW_CLEARANCE_MM
    additional_rear_shift = max(0.0, selected_boundary_z - required_boundary_z)

    rows, mass = rotating_mass_rows()
    flyer_mass_row = next(row for row in rows if row["name"] == "flyer_pulley")
    correction_attachment = integrated_six_stack_attachment_audit()
    m2_torque = final_integrated_m2_torque_audit(
        mass["izz_about_M2_axis_kg_m2"]
    )
    complete_unintended = max(unintended.values()) <= BOOLEAN_TOL_MM3
    intended_contact_gate = (
        intended["belt_to_motor_pulley_overlap_mm3"] > BOOLEAN_TOL_MM3
        and intended["belt_to_flyer_pulley_overlap_mm3"] > BOOLEAN_TOL_MM3
        and intended[
            "motor_pulley_to_exact_Leadshine_D_shaft_distance_mm"
        ] <= CONTACT_TOL_MM
        and len(bnw_screws) == 2
        and len(bnw_holes) == 2
        and min(intended[
            "BNW_set_screw_to_exact_Leadshine_shaft_distances_mm"
        ]) <= CONTACT_TOL_MM
        and max(intended[
            "BNW_set_screw_to_exact_Leadshine_shaft_distances_mm"
        ]) <= CONTACT_TOL_MM
        and min(intended[
            "BNW_hole_path_to_official_stock_overlap_mm3"
        ]) > BOOLEAN_TOL_MM3
        and min(intended[
            "BNW_set_screw_to_official_stock_overlap_mm3"
        ]) > BOOLEAN_TOL_MM3
        and min(intended[
            "BNW_hole_path_to_matching_screw_overlap_mm3"
        ]) > BOOLEAN_TOL_MM3
        and intended["front_cap_to_stator_distance_mm"] <= CONTACT_TOL_MM
        and intended["rear_cap_to_stator_distance_mm"] <= CONTACT_TOL_MM
        and intended["wire_to_fixed_felt_distance_mm"] <= CONTACT_TOL_MM
        and intended["wire_to_moving_felt_distance_mm"] <= CONTACT_TOL_MM
        and intended["wire_to_dancer_pulley_distance_mm"] <= CONTACT_TOL_MM
        and len(arm_screws) == 2
        and max(intended["arm_M3x8_to_shaft_flat_distances_mm"])
        <= CONTACT_TOL_MM
        and min(
            row["projected_screw_insert_engagement_mm"]
            for row in arm_screw_packaging_rows
        ) >= 5.4 - 1.0e-6
    )
    flat_signature = [
        (
            row["normal"],
            round(float(row["station_from_rear_datum_mm"]), 6),
            round(float(row["axial_length_mm"]), 6),
            round(float(row["depth_from_OD_mm"]), 6),
        )
        for row in shaft_brep["indexed_flats"]
    ]
    shaft_geometry_gate = (
        shaft_brep["solid_count"] == 1
        and shaft_brep["valid"]
        and all(math.isclose(value, target, abs_tol=1.0e-5) for value, target in zip(
            shaft_brep["bbox_mm"]["minimum_mm"],
            [-6.0, -6.0, flyer_shaft_d10.WORLD_REAR_Z_MM]
        ))
        and all(math.isclose(value, target, abs_tol=1.0e-5) for value, target in zip(
            shaft_brep["bbox_mm"]["maximum_mm"],
            [6.0, 6.0, flyer_shaft_d10.WORLD_FRONT_Z_MM]
        ))
        and shaft_brep["cylindrical_surface_radii_mm"] == [3.0, 4.5, 5.0, 6.0]
        and math.isclose(shaft_brep["length_mm"], 79.0, abs_tol=1.0e-9)
        and math.isclose(
            shaft_brep["D10_seat_length_mm"], 18.5, abs_tol=1.0e-9
        )
        and shaft_brep["minimum_neck_radial_wall_at_limits_mm"] >= 1.98
        and math.isclose(
            shaft_brep["ID6_to_ID9_transition_length_mm"],
            3.0,
            abs_tol=1.0e-9,
        )
        and flat_signature == [
            ("minus_y", 64.75, 5.0, 0.3),
            ("plus_x", 64.75, 5.0, 0.3),
        ]
        and shaft_brep["wire_mouth_toroidal_face_count"] == 2
        and math.isclose(
            shaft_brep["wire_mouth_fillet_radius_mm"],
            0.5,
            abs_tol=1.0e-12,
        )
    )
    shaft_arm_retention_gate = (
        len(arm_screws) == 2
        and max(intended["arm_M3x8_to_shaft_flat_distances_mm"])
        <= CONTACT_TOL_MM
        and unintended["arm_M3x8_set_screws_vs_released_shaft_max_mm3"]
        <= BOOLEAN_TOL_MM3
        and all(
            math.isclose(
                row["M3x8_length_preserved_mm"], 8.0, abs_tol=1.0e-6
            )
            and math.isclose(
                row["outer_socket_end_radius_mm"], 13.7, abs_tol=1.0e-6
            )
            and row["projected_screw_insert_engagement_mm"]
            >= 5.4 - 1.0e-6
            for row in arm_screw_packaging_rows
        )
    )
    shaft_fit_gate = (
        max(shaft_fit["6001_bearings"]["distances_mm"])
        <= CONTACT_TOL_MM
        and max(shaft_fit["6001_bearings"]["overlaps_mm3"])
        <= BOOLEAN_TOL_MM3
        and all(
            math.isclose(
                row["radial_clearance_mm"], 0.025, abs_tol=1.0e-5
            )
            and row["overlap_mm3"] <= BOOLEAN_TOL_MM3
            for row in shaft_fit["inner_race_spacers"].values()
        )
        and shaft_fit["flyer_P30_stock_D10_clamp"]["contact_distance_mm"]
        <= CONTACT_TOL_MM
        and shaft_fit["flyer_P30_stock_D10_clamp"]["overlap_mm3"]
        <= BOOLEAN_TOL_MM3
        and all(math.isclose(a, b, abs_tol=1.0e-5) for a, b in zip(
            shaft_fit["flyer_P30_stock_D10_clamp"]["pulley_axial_span_z_mm"],
            shaft_fit["flyer_P30_stock_D10_clamp"][
                "shaft_D10_seat_axial_span_z_mm"
            ],
        ))
        and official_flyer_p30.source_sha256_before
        == nbk_p30_d10.SOURCE_STEP_SHA256
        == official_flyer_p30.source_sha256_after
        == nbk_p30_d10.source_sha256()
        and shaft_fit["DIN988_shim"]["distance_mm"] <= CONTACT_TOL_MM
        and shaft_fit["DIN988_shim"]["overlap_mm3"]
        <= BOOLEAN_TOL_MM3
        and math.isclose(
            arm_root["corrected_interface"][
                "arm_to_shaft_radial_clearance_mm"
            ],
            0.05,
            abs_tol=1.0e-5,
        )
        and arm_root["corrected_interface"]["arm_vs_shaft_overlap_mm3"]
        <= BOOLEAN_TOL_MM3
    )
    arm_root_gate = (
        arm_root["corrected_interface"]["arm_solid_count"] == 1
        and arm_root["corrected_interface"]["arm_valid"]
        and arm_root["failure_found_before_correction"][
            "retained_arm_vs_released_shaft_overlap_mm3"
        ] > BOOLEAN_TOL_MM3
        and arm_root["root_sleeve_web"]["radial_ligament_mm"] >= 2.4
        and arm_root["root_sleeve_web"][
            "web_to_existing_collar_overlap_mm3"
        ] > BOOLEAN_TOL_MM3
        and arm_root["root_sleeve_web"][
            "web_to_main_spoke_overlap_mm3"
        ] > BOOLEAN_TOL_MM3
        and arm_root["root_sleeve_web"][
            "web_to_rear_counterrail_overlap_mm3"
        ] > BOOLEAN_TOL_MM3
        and arm_root["conservative_combined_root_load_case"][
            "passes_review_allowable"
        ]
    )
    shaft_wire_handoff_gate = (
        shaft_wire_handoffs["rear_entry"]["wire_vs_shaft_overlap_mm3"]
        <= BOOLEAN_TOL_MM3
        and shaft_wire_handoffs["rear_entry"][
            "wire_to_shaft_wall_clearance_mm"
        ] >= 2.75
        and shaft_wire_handoffs["rear_entry"][
            "analytic_0p5mm_wire_radial_clearance_mm"
        ] >= 2.75
        and shaft_wire_handoffs["rear_entry"][
            "R0p50_internal_mouth_present"
        ]
        and shaft_wire_handoffs["front_root"][
            "wire_vs_shaft_overlap_mm3"
        ] <= BOOLEAN_TOL_MM3
        and shaft_front_interface["source_gate"]
        and shaft_wire_handoffs["front_root"][
            "wire_vs_corrected_arm_overlap_mm3"
        ] <= BOOLEAN_TOL_MM3
        and shaft_wire_handoffs["front_root"][
            "root_web_to_job_wire_clearance_mm"
        ] >= 1.25
        and shaft_wire_handoffs["front_root"][
            "root_web_to_0p5mm_wire_clearance_mm"
        ] >= 1.1
        and shaft_wire_handoffs["axis_ownership_seam"][
            "centerline_gap_mm"
        ] <= CONTACT_TOL_MM
        and shaft_wire_handoffs["axis_ownership_seam"][
            "static_to_flyer_wire_distance_mm"
        ] <= CONTACT_TOL_MM
        and shaft_wire_handoffs["axis_ownership_seam"][
            "static_to_flyer_wire_overlap_mm3"
        ] <= BOOLEAN_TOL_MM3
        and shaft_wire_handoffs["axis_ownership_seam"][
            "static_axis_wire_vs_shaft_wall_overlap_mm3"
        ] <= BOOLEAN_TOL_MM3
        and shaft_wire_handoffs["axis_ownership_seam"][
            "static_axis_wire_vs_corrected_arm_overlap_mm3"
        ] <= BOOLEAN_TOL_MM3
    )
    checks = {
        "source_reports_schema_and_self_hash_valid": True,
        "released_M2_001_Rev_D_L79_D10_ID6_ID9_shaft_exact_BREP_and_R0p50_mouths": (
            shaft_geometry_gate
        ),
        "released_shaft_stock_D10_clamp_and_arm_M3x8_retention_valid": (
            shaft_arm_retention_gate
        ),
        "released_shaft_bearing_spacer_pulley_shim_and_arm_fits_valid": (
            shaft_fit_gate
        ),
        "released_shaft_rear_entry_and_front_root_wire_handoffs_open": (
            shaft_wire_handoff_gate
        ),
        "released_Rev_D_shaft_front_setback_and_PEEK_wire_clearance_valid": (
            shaft_front_interface["source_gate"]
        ),
        "corrected_arm_root_sleeve_is_one_solid_positive_load_path_and_strength_PASS": (
            arm_root_gate
        ),
        "printed_arm_ID12p10_single_setup_ream_and_measured_fit_contract_specified": (
            not arm_root["post_print_manufacturing_contract"][
                "as_printed_FDM_bore_is_accepted_without_reaming"
            ]
            and arm_root["post_print_manufacturing_contract"][
                "measured_fit_and_assembly_check_required_before_balance"
            ]
            and arm_root["post_print_manufacturing_contract"][
                "resulting_diametral_clearance_range_mm"
            ] == [0.09999999999999964, 0.15000000000000036]
        ),
        "official_NBK_P30_D5_motor_vendor_STEP_hash_pinned_and_unmodified": (
            official_p30.source_sha256_before
            == nbk_p30.SOURCE_STEP_SHA256
            == official_p30.source_sha256_after
            == nbk_p30.source_sha256()
        ),
        "official_NBK_P30_D5_motor_occurrence_one_solid_at_existing_tooth_midplane": (
            len(drive_parts["motor_pulley"].solids()) == 1
            and math.isclose(
                drive_parts["motor_pulley"].bounding_box().min.Z,
                drive.PULLEY_CENTER_Z - M2_REAR_SHIFT_MM
                + nbk_p30.SOURCE_AXIAL_MIN_MM,
                abs_tol=1.0e-6,
            )
            and math.isclose(
                drive_parts["motor_pulley"].bounding_box().max.Z,
                drive.PULLEY_CENTER_Z - M2_REAR_SHIFT_MM
                + nbk_p30.SOURCE_AXIAL_MAX_MM,
                abs_tol=1.0e-6,
            )
            and math.isclose(
                drive_parts["motor_pulley"].volume,
                official_p30.stock_occurrence.volume,
                abs_tol=1.0e-5,
            )
        ),
        "official_NBK_P30_D5_motor_mass_28g_and_axial_J_3e_6_authority_recorded": (
            official_p30.official_mass_properties.mass_g == 28.0
            and official_p30.official_mass_properties.axial_moment_of_inertia_kg_m2
            == 3.0e-6
        ),
        "official_NBK_P30_D10_flyer_vendor_STEP_hash_pinned_and_unmodified": (
            official_flyer_p30.source_sha256_before
            == nbk_p30_d10.SOURCE_STEP_SHA256
            == official_flyer_p30.source_sha256_after
            == nbk_p30_d10.source_sha256()
        ),
        "official_NBK_P30_D10_flyer_occurrence_one_solid_hub_rear_on_full_seat": (
            len(drive_parts["flyer_pulley"].solids()) == 1
            and all(math.isclose(a, b, abs_tol=1.0e-5) for a, b in zip(
                shaft_fit["flyer_P30_stock_D10_clamp"][
                    "pulley_axial_span_z_mm"
                ],
                [
                    flyer_shaft_d10.WORLD_REAR_Z_MM,
                    flyer_shaft_d10.SHOULDER_WORLD_Z_MM,
                ],
            ))
            and shaft_fit["flyer_P30_stock_D10_clamp"]["contact_distance_mm"]
            <= CONTACT_TOL_MM
            and shaft_fit["flyer_P30_stock_D10_clamp"]["overlap_mm3"]
            <= BOOLEAN_TOL_MM3
        ),
        "official_NBK_P30_D10_flyer_mass_28g_and_axial_J_3e_6_consumed_in_balance": (
            math.isclose(
                float(flyer_mass_row["mass_g"]),
                nbk_p30_d10.OFFICIAL_MASS_G,
                abs_tol=1.0e-12,
            )
            and math.isclose(
                float(flyer_mass_row["izz_about_M2_axis_g_mm2"]),
                nbk_p30_d10.OFFICIAL_AXIAL_MOMENT_OF_INERTIA_KG_M2
                * 1.0e9,
                abs_tol=1.0e-12,
            )
            and flyer_mass_row["supplied_clamp_bolt_included"] is True
        ),
        "BNW_two_M3_hole_paths_and_two_M3x12_screws_are_separate_witnesses": (
            len(bnw_holes) == 2
            and len(bnw_screws) == 2
            and all("hole_path_witness" in str(hole.label) for hole in bnw_holes)
            and all(
                "M3x12_set_screw_envelope_witness" in str(screw.label)
                for screw in bnw_screws
            )
        ),
        "BNW_round_and_D_flat_screw_witnesses_touch_without_shaft_penetration": (
            max(intended[
                "BNW_set_screw_to_exact_Leadshine_shaft_distances_mm"
            ]) <= CONTACT_TOL_MM
            and unintended[
                "BNW_set_screw_witnesses_vs_exact_Leadshine_shaft_max_mm3"
            ] <= BOOLEAN_TOL_MM3
        ),
        "BNW_socket_ends_remain_accessible_and_clear_belt": (
            min(bnw_socket_packaging[
                "socket_end_protrusion_beyond_published_E23_clamp_radius_mm"
            ]) >= 2.5 - 1.0e-9
            and min(bnw_socket_packaging[
                "socket_end_reserve_inside_OD32_max_radius_mm"
            ]) >= 1.5 - 1.0e-9
            and bnw_socket_packaging["belt_clearance_min_mm"]
            >= MIN_REVIEW_CLEARANCE_MM
            and all(
                math.isclose(length, 12.0, abs_tol=1.0e-9)
                for length in bnw_socket_packaging[
                    "M3x12_length_preserved_mm"
                ]
            )
        ),
        "normal_GOAL_nema17_frame_retained": reports["leadshine"][
            "gates"
        ]["frame_xy"],
        "exact_1_to_1_ratio_retained": reports["drive"]["geometry"][
            "checks"
        ]["exact_1_to_1_ratio"],
        "complete_M2_module_rigidly_shifted_rear_10mm": math.isclose(
            M2_REAR_SHIFT_MM, 10.0, abs_tol=1.0e-12
        ),
        "entry_module_rigidly_shifted_rear_4p25mm": math.isclose(
            ENTRY_REAR_SHIFT_MM, 4.25, abs_tol=1.0e-12
        ),
        "frozen_raw_entry_clearance_failure_consumed_and_superseded": (
            reports["base_raw"]["status"] == "FAIL"
            and math.isclose(
                reports["base_raw"]["minimum_dynamic_clearance_mm"],
                1.0,
                abs_tol=1.0e-9,
            )
        ),
        "entry_shaft_and_flyer_P30_clear_ge_2p2mm_with_reserve": (
            clearances["extended_hollow_shaft_to_entry_bracket_mm"]
            >= MIN_REVIEW_CLEARANCE_MM
            and clearances["flyer_P30_to_entry_bracket_mm"]
            >= MIN_REVIEW_CLEARANCE_MM
            and clearances["extended_hollow_shaft_to_entry_eyelet_mm"]
            >= MIN_REVIEW_CLEARANCE_MM
            and clearances["flyer_P30_to_entry_eyelet_mm"]
            >= MIN_REVIEW_CLEARANCE_MM
        ),
        "entry_wire_passage_and_axial_handoff_are_open_and_collinear": (
            entry_wire_bridge["wire_to_entry_wall_overlap_mm3"]
            <= BOOLEAN_TOL_MM3
            and entry_wire_bridge["radial_wall_clearance_exact_BREP_mm"]
            >= 1.4
            and entry_wire_bridge[
                "handoff_is_collinear_with_hollow_shaft_axis"
            ]
            and entry_wire_bridge["straight_axial_bridge_length_mm"] > 0.0
        ),
        "entry_module_mounting_and_dancer_anchor_support_preserved": (
            entry_attachments["entry_bracket_one_valid_solid"]
            and entry_attachments[
                "entry_bracket_to_rear_post_distance_mm"
            ] <= CONTACT_TOL_MM
            and entry_attachments[
                "entry_bracket_to_rear_post_engagement_mm3"
            ] > BOOLEAN_TOL_MM3
            and entry_attachments[
                "entry_eyelet_to_bracket_distance_mm"
            ] <= CONTACT_TOL_MM
            and entry_attachments[
                "entry_eyelet_seat_overlap_mm3"
            ] > BOOLEAN_TOL_MM3
            and entry_attachments[
                "preserved_dancer_fixed_screw_to_bracket_distance_mm"
            ] <= CONTACT_TOL_MM
            and entry_attachments[
                "all_mounting_hardware_contacts_bracket_and_post"
            ]
        ),
        "retained_arm_one_solid": len(list(arm.solids())) == 1,
        "all_six_positive_counterweight_stacks_present": sum(
            "M3x6_screw" in name for name in rotating
        ) == 4 and sum("94459A130_insert" in name for name in rotating) == 4
        and sum(name.startswith("front_trim_B777_") for name in rotating) == 2
        and sum(name.startswith("front_trim_hardware_") for name in rotating) == 6,
        "all_six_counterweight_fasteners_terminate_in_positive_material": (
            correction_attachment[
                "all_six_screws_terminate_in_positive_printed_material"
            ]
            and not correction_attachment[
                "any_balance_fastener_over_open_air"
            ]
        ),
        "one_piece_PEEK_guide_bell_and_three_retention_stacks_present": (
            "flyer_PEEK_guide" in rotating
            and sum("flyer_PEEK_guide_retention_screw" in name
                    for name in rotating) == 3
            and sum("flyer_PEEK_guide_retention_insert" in name
                    for name in rotating) == 3
        ),
        "production_cap_pair_and_all_12_hardware_occurrences_present": (
            len(cap_home_parts) == 14
        ),
        "targeted_new_cross_module_unintended_overlaps_zero": complete_unintended,
        "targeted_intended_contacts_established": intended_contact_gate,
        "actual_deep_PEEK_caps_clear_retained_arm_ge_2p2mm": (
            clearances["actual_deep_cap_pair_to_retained_arm_mm"]
            >= MIN_REVIEW_CLEARANCE_MM
        ),
        "retained_arm_clears_shifted_block_ge_2p2mm": (
            clearances["retained_arm_to_shifted_flyer_block_mm"]
            >= MIN_REVIEW_CLEARANCE_MM
        ),
        "extended_wire_reaches_shifted_bore_without_shaft_intersection": (
            unintended["static_wire_vs_extended_hollow_shaft_mm3"]
            <= BOOLEAN_TOL_MM3
            and len(list(wire.solids())) == 1
            and wire.is_valid
            and wire.bounding_box().min.Z
            <= flyer_shaft_d10.WORLD_REAR_Z_MM + 0.05
            and wire.bounding_box().max.Z
            >= flyer_shaft_d10.WORLD_REAR_Z_MM - 0.05
        ),
        "selected_frame_window_relocation_keeps_2p2mm_motor_rear_clearance": (
            selected_boundary_clearance >= MIN_REVIEW_CLEARANCE_MM
        ),
        "all_shifted_base_rail_brackets_feet_and_tnuts_remain_over_material": (
            frame_attachments[
                "all_fully_inside_shifted_rail_longitudinal_span"
            ]
            and frame_attachments[
                "all_occurrences_project_over_real_rail_longitudinal_material"
            ]
            and frame_attachments[
                "all_groups_have_supported_attachment_chain"
            ]
            and not frame_attachments["any_attachment_over_open_air"]
        ),
        "integrated_OCC_two_plane_balance_exact": (
            mass["static_imbalance_g_mm"] < 1.0e-6
            and mass["couple_imbalance_g_mm2"] < 1.0e-6
        ),
        "coupled_exact_live_line_Leadshine_36V_margin_ge_2x": (
            m2_torque["Leadshine_36V_gate_ge_2x"]
        ),
        "coupled_exact_live_line_P30_210_3GT_capacity_ge_2x": (
            m2_torque["P30_210_3GT_gate_ge_2x"]
        ),
        "coupled_exact_live_line_M1_margin_ge_2x": (
            m2_torque["coupled_axis_loads"]["M1"]["gate_ge_2x"]
        ),
        "coupled_exact_live_line_M0_margin_ge_2x": (
            m2_torque["coupled_axis_loads"]["M0"]["gate_ge_2x"]
        ),
        "selected_36V_curve_condition_consumed_without_24V_failure_claim": (
            m2_torque["theoretical_curve_gate_scope"]
            == "published 36 V lower-edge curve only"
        ),
    }
    return {
        "checks": checks,
        "unintended_overlaps_mm3": unintended,
        "intended_contacts": intended,
        "clearances_mm": clearances,
        "frame_window_interface": {
            "candidate_motor_rear_z_mm": motor_rear_z,
            "current_frame_boundary_z_mm": current_boundary_z,
            "selected_integrated_boundary_z_mm": selected_boundary_z,
            "integrated_frame_window_rear_shift_mm": (
                INTEGRATED_FRAME_WINDOW_REAR_SHIFT_MM
            ),
            "selected_clearance_mm": selected_boundary_clearance,
            "required_boundary_for_2p2mm_clearance_z_mm": required_boundary_z,
            "additional_rearward_boundary_shift_required_mm": additional_rear_shift,
            "total_rearward_boundary_shift_required_mm": (
                INTEGRATED_FRAME_WINDOW_REAR_SHIFT_MM + additional_rear_shift
            ),
        },
        "frame_relocation_attachment_audit": frame_attachments,
        "entry_module_attachment_audit": entry_attachments,
        "entry_wire_bridge": entry_wire_bridge,
        "released_M2_001_Rev_D_shaft_BREP": shaft_brep,
        "released_Rev_D_shaft_front_interface": shaft_front_interface,
        "released_shaft_screw_packaging": {
            "stock_D10_supplied_clamp_bolt_count": 1,
            "stock_D10_supplied_clamp_bolt": (
                nbk_p30_d10.OFFICIAL_CLAMP_BOLT
            ),
            "arm_M3x8_count": len(arm_screws),
            "arm_M3x8": arm_screw_packaging,
        },
        "released_shaft_bearing_spacer_collar_fits": shaft_fit,
        "released_shaft_wire_handoffs": shaft_wire_handoffs,
        "corrected_printed_arm_root_sleeve_load_path": arm_root,
        "frozen_raw_clearance_diagnostic_reconciliation": {
            "clearance_report_path": (
                "out/reports/integrated_candidate_base_clearance_raw.json"
            ),
            "clearance_report_sha256": _sha256(BASE_RAW_CLEARANCE_REPORT),
            "part_report_path": (
                "out/reports/integrated_candidate_base_clearance_raw_parts.json"
            ),
            "part_report_sha256": _sha256(BASE_RAW_PARTS_REPORT),
            "diagnostic_manifest_sha256": reports["base_raw"]["links"][
                "manifest_sha256"
            ],
            "old_shaft_to_entry_bracket_mm": 1.0,
            "old_flyer_P30_to_entry_bracket_mm": 1.75,
            "old_target_mm": reports["base_raw"]["target_mm"],
            "candidate_required_target_mm": MIN_REVIEW_CLEARANCE_MM,
            "candidate_shaft_to_entry_bracket_mm": clearances[
                "extended_hollow_shaft_to_entry_bracket_mm"
            ],
            "candidate_flyer_P30_to_entry_bracket_mm": clearances[
                "flyer_P30_to_entry_bracket_mm"
            ],
            "frozen_adapter_predates_entry_redesign": True,
            "full_raw_rerun_was_required_by_frozen_diagnostic": True,
            "current_exact_full_raw_rerun_consumed": reports[
                "active_sector"
            ]["release_gates"]["full_raw_rigid_sweep_clear"],
        },
        "official_NBK_P30_stock_and_BNW_review": {
            "vendor_source_path": str(
                nbk_p30.SOURCE_STEP.relative_to(ROOT)
            ).replace("\\", "/"),
            "vendor_source_sha256": nbk_p30.SOURCE_STEP_SHA256,
            "helper_source_path": "cad/nbk_p30_official_occurrence.py",
            "helper_source_sha256": _sha256(Path(nbk_p30.__file__)),
            "stock_occurrence_label": str(
                drive_parts["motor_pulley"].label
            ),
            "stock_occurrence_bbox_mm": _bbox(
                drive_parts["motor_pulley"]
            ),
            "tooth_midplane_center_xyz_mm": [
                0.0,
                drive.MOTOR_AXIS_Y,
                drive.PULLEY_CENTER_Z - M2_REAR_SHIFT_MM,
            ],
            "stock_roll_deg": NBK_P30_STOCK_ROLL_DEG,
            "official_mass_g": nbk_p30.OFFICIAL_MASS_G,
            "official_axial_moment_of_inertia_kgm2": (
                nbk_p30.OFFICIAL_AXIAL_MOMENT_OF_INERTIA_KG_M2
            ),
            "BNW_review_configuration": {
                "first_azimuth_deg": NBK_P30_BNW_FIRST_AZIMUTH_DEG,
                "second_azimuth_deg": (
                    NBK_P30_BNW_FIRST_AZIMUTH_DEG
                    + nbk_p30.BNW_WITNESS_ANGLE_DEG
                ),
                "source_axis_station_mm": NBK_P30_BNW_LOCAL_X_MM,
                "hole_path_count": len(bnw_holes),
                "hole_path_diameter_mm": (
                    nbk_p30.BNW_WITNESS_HOLE_DIAMETER_MM
                ),
                "screw_witness_count": len(bnw_screws),
                "screw_witness_diameter_mm": (
                    nbk_p30.BNW_WITNESS_SCREW_DIAMETER_MM
                ),
                "screw_witness_length_mm": (
                    nbk_p30.BNW_WITNESS_SCREW_LENGTH_MM
                ),
                "screw_inward_adjustments_mm": list(
                    NBK_P30_BNW_SCREW_INWARD_ADJUSTMENTS_MM
                ),
                "round_side_witness_index": 0,
                "D_flat_witness_index": 1,
                "positive_separate_occurrences_not_boolean_holes": True,
                "delivered_BNW_size_or_station_claimed": False,
            },
        },
        "BNW_set_screw_socket_end_and_packaging": bnw_socket_packaging,
        "integrated_rotating_mass_rows": rows,
        "integrated_slug_length_solution_mm": {
            pocket.id: length
            for pocket, length in zip(
                retained.POCKETS, integrated_slug_lengths()
            )
        },
        "integrated_front_trim_common_thickness_mm": (
            integrated_balance_solution()[
                "front_trim_common_thickness_mm"
            ]
        ),
        "integrated_six_slug_balance_authority": (
            integrated_balance_solution()["authority"]
        ),
        "superseded_retained_review_slug_lengths_mm": {
            pocket.id: length
            for pocket, length in zip(
                retained.POCKETS, retained_review_slug_lengths()
            )
        },
        "integrated_rotating_mass_properties": mass,
        "integrated_six_stack_attachment_audit": correction_attachment,
        "final_integrated_M2_torque": m2_torque,
        "reference_pose": {"m0_rad": 0.0, "m1_rad": 0.0, "m2_rad": 0.0},
    }


def build_links(
    *, tip_guide_override: Part | None = None,
) -> dict[str, list[Part | Compound]]:
    """Return physical reference-pose links for the canonical twin.

    The public contract deliberately matches ``assembly.build_links``.  Wire
    and positive BNW hole-path review solids are excluded from collision links
    so a raw-cycle runner cannot mistake either witness class for rigid
    machine material.  The conservative M3x12 screw envelopes remain in the
    static link for clearance checking.
    """

    _contract_reports()
    static = main_static_groups()
    drive_parts = successor_drive_parts()
    rotating = retained_rotating_parts(tip_guide_override=tip_guide_override)
    spindle_context, stator = spindle_without_stator()

    return {
        "static": [
            *static["unchanged"],
            *static["shifted_support"],
            *static["shifted_entry"],
            *(shape for key, shape in drive_parts.items()
              if key != "flyer_pulley"
              and not key.startswith("motor_pulley_BNW_hole_path_")),
        ],
        "carriage": carriage_module_parts(),
        "spindle": [stator, *spindle_context, *cap_module_parts()],
        "flyer": list(rotating.values()),
    }


def wire_visuals() -> dict[str, list[Part]]:
    """Return visual conductor witnesses keyed by their kinematic owner."""

    return {
        "static": [configured_static_supply_wire()],
        "flyer": [flyer_successor.guide_wire_envelope(
            float(DEFAULT_STATOR.wire_d),
            "configured_wire_inside_one_piece_PEEK_guide_and_exit_bell",
        )],
        "spindle": cap_wire_witnesses(),
    }


def link_location(
    link: str, m0: float = 0.0, m1: float = 0.0, m2: float = 0.0
):
    """Use the unchanged upstream radians/mm kinematics contract."""

    return assembly.link_location(link, m0=m0, m1=m1, m2=m2)


def machine(
    m0: float = 0.0,
    m1: float = 0.0,
    m2: float = 0.0,
    *,
    tip_guide_override: Part | None = None,
) -> Compound:
    children: list[Compound] = []
    labels = {
        "static": "static_current_frame_with_shifted_successor_M2",
        "carriage": "current_carriage_with_keyed_M1_static_active_sector_guides",
        "spindle": "current_spindle_chuck_and_short_leadin_PEEK_cap_stator",
        "flyer": "retained_offset_spoke_flyer_six_positive_stacks_PEEK_guide_and_P30_drive",
    }
    for name, parts in build_links(
        tip_guide_override=tip_guide_override
    ).items():
        location = link_location(name, m0=m0, m1=m1, m2=m2)
        moved: list[Part | Compound] = []
        for original in parts:
            shape = location * original
            shape.label = str(getattr(original, "label", "part"))
            moved.append(shape)
        children.append(_compound(moved, labels[name]))

    wire_groups: list[Compound] = []
    for owner, parts in wire_visuals().items():
        location = link_location(owner, m0=m0, m1=m1, m2=m2)
        moved: list[Part] = []
        for original in parts:
            shape = location * original
            shape.label = str(getattr(original, "label", "wire"))
            moved.append(shape)
        wire_groups.append(_compound(moved, f"wire_{owner}"))
    children.append(_compound(
        wire_groups,
        "wire_review_static_supply_plus_exact_PEEK_bore_and_cap_witnesses",
    ))
    drive_parts = successor_drive_parts()
    children.append(_compound(
        [
            shape
            for key, shape in drive_parts.items()
            if key.startswith("motor_pulley_BNW_hole_path_")
        ],
        "NBK_BNW_positive_hole_path_review_witnesses_not_machine_material",
    ))
    return _compound(
        children,
        "normal_GOAL_integrated_release_candidate_FAIL_CLOSED",
    )


def gen_step() -> Compound:
    return machine()


def analyze() -> dict[str, Any]:
    reports = _contract_reports()
    audit = geometry_audit()
    geometry_pass = all(audit["checks"].values())
    source_contracts = {
        "retained_flyer": {
            "path": "out/reports/permanent_cap_offset_spoke_retained_review.json",
            "schema": reports["retained"]["schema"],
            "status": reports["retained"]["status"],
            "production_authorized": reports["retained"]["production_authorized"],
            "file_sha256": _sha256(RETAINED_REPORT),
            "report_sha256": reports["retained"].get("report_sha256"),
            "source_sha256": _sha256(retained.SOURCE),
        },
        "retained_flyer_PEEK_guide_successor": {
            "path": "out/reports/retained_flyer_peek_guide_successor.json",
            "schema": reports["flyer_guide"]["schema"],
            "status": reports["flyer_guide"]["status"],
            "production_authorized": reports["flyer_guide"][
                "production_authorized"
            ],
            "file_sha256": _sha256(FLYER_GUIDE_REPORT),
            "report_sha256": reports["flyer_guide"]["report_sha256"],
            "source_path": "cad/retained_flyer_peek_guide_successor.py",
            "source_sha256": _sha256(Path(flyer_successor.__file__)),
            "manifest_path": (
                "out/review/retained_flyer_peek_guide_successor.manifest.json"
            ),
            "manifest_sha256": _sha256(FLYER_GUIDE_MANIFEST),
            "isolated_balance_is_final_authority": False,
            "integrated_balance_authority": integrated_balance_solution()[
                "authority"
            ],
        },
        "active_sector_terminal_route_and_rigid_sweep": {
            "path": (
                "out/reports/"
                "carriage_active_sector_terminal_guide_audit.json"
            ),
            "schema": reports["active_sector"]["schema"],
            "status": reports["active_sector"]["status"],
            "production_authorized": reports["active_sector"][
                "production_authorized"
            ],
            "assembly_geometry_integration_authorized": reports[
                "active_sector"
            ]["assembly_geometry_integration_authorized"],
            "file_sha256": _sha256(ACTIVE_SECTOR_REPORT),
            "report_sha256": reports["active_sector"]["report_sha256"],
            "source_path": (
                "sim/carriage_active_sector_terminal_guide_audit.py"
            ),
            "source_sha256": _sha256(ACTIVE_SECTOR_AUDIT_SOURCE),
            "guide_source_path": (
                "cad/carriage_active_sector_terminal_guide.py"
            ),
            "guide_source_sha256": _sha256(ACTIVE_SECTOR_GUIDE_SOURCE),
            "collision_geometry_revision": COLLISION_GEOMETRY_REVISION,
            "locus_path": (
                "out/reports/"
                "carriage_active_sector_terminal_guide_loci.json"
            ),
            "locus_file_sha256": _sha256(ACTIVE_SECTOR_LOCI),
            "locus_payload_sha256": reports["active_sector"][
                "player_route_api"
            ]["canonical_payload_sha256"],
            "locus_count": reports["active_sector"][
                "player_route_api"
            ]["locus_count"],
            "step_path": (
                "out/review/carriage_active_sector_terminal_guide.step"
            ),
            "step_sha256": _sha256(ACTIVE_SECTOR_STEP),
            "manifest_path": (
                "out/review/"
                "carriage_active_sector_terminal_guide.manifest.json"
            ),
            "manifest_sha256": _sha256(ACTIVE_SECTOR_MANIFEST),
            "continuous_park_index_load_unload_proven": False,
            "raw_wraps_exactly_two_turns": False,
        },
        "physical_caps": {
            "path": "out/reports/permanent_cap_production_review.json",
            "schema": reports["caps"]["schema"],
            "status": reports["caps"]["status"],
            "production_authorized": reports["caps"]["production_authorized"],
            "file_sha256": _sha256(CAP_REPORT),
            "report_sha256": reports["caps"].get("report_sha256"),
            "source_sha256": _sha256(Path(caps.__file__)),
        },
        "legacy_P30_geometry_source_non_governing": {
            "path": "out/reports/m2_drive_successor_review.json",
            "schema": reports["drive"]["schema"],
            "status": reports["drive"]["status"],
            "production_authorized": reports["drive"]["production_authorized"],
            "file_sha256": _sha256(DRIVE_REPORT),
            "source_sha256": _sha256(Path(drive.__file__)),
            "governing_drive_result": False,
            "motor_pulley_geometry_used": False,
            "retained_scope": "motor mount and 210-3GT-6 belt envelope only",
        },
        "governing_normal_GOAL_M2_selection": {
            "path": "out/reports/m2_normal_goal_drive_selection.json",
            "schema": reports["m2_selection"]["schema"],
            "status": reports["m2_selection"]["status"],
            "production_authorized": reports["m2_selection"][
                "production_authorized"
            ],
            "reference_CAD_integration_authorized": reports["m2_selection"][
                "reference_CAD_integration_authorized"
            ],
            "file_sha256": _sha256(M2_SELECTION_REPORT),
            "source_sha256": _sha256(M2_SELECTION_SOURCE),
        },
        "official_NBK_P30_D5_motor_stock_occurrence": {
            "helper_path": "cad/nbk_p30_official_occurrence.py",
            "helper_sha256": _sha256(Path(nbk_p30.__file__)),
            "vendor_STEP_path": str(
                nbk_p30.SOURCE_STEP.relative_to(ROOT)
            ).replace("\\", "/"),
            "vendor_STEP_sha256": nbk_p30.SOURCE_STEP_SHA256,
            "current_vendor_STEP_sha256": nbk_p30.source_sha256(),
            "stock_occurrence_is_byte_identical_transform_only": True,
            "vendor_STEP_reexported_alone": False,
            "stock_mass_g": nbk_p30.OFFICIAL_MASS_G,
            "stock_axial_moment_of_inertia_kgm2": (
                nbk_p30.OFFICIAL_AXIAL_MOMENT_OF_INERTIA_KG_M2
            ),
            "placement": {
                "tooth_midplane_center_xyz_mm": [
                    0.0,
                    drive.MOTOR_AXIS_Y,
                    drive.PULLEY_CENTER_Z - M2_REAR_SHIFT_MM,
                ],
                "stock_roll_deg": NBK_P30_STOCK_ROLL_DEG,
            },
            "configured_BNW_boundary": {
                "two_M3_upper_bound_hole_paths": 2,
                "two_M3x12_upper_bound_screw_witnesses": 2,
                "first_azimuth_deg": NBK_P30_BNW_FIRST_AZIMUTH_DEG,
                "source_axis_station_mm": NBK_P30_BNW_LOCAL_X_MM,
                "screw_inward_adjustments_mm": list(
                    NBK_P30_BNW_SCREW_INWARD_ADJUSTMENTS_MM
                ),
                "D_flat_witness_adjustment_mm": 0.5,
                "positive_separate_review_occurrences": True,
                "exact_delivered_BNW_configuration_known": False,
                "retention_release_authorized": False,
            },
            "governing_selection_report_predates_official_STEP_acquisition": True,
        },
        "official_NBK_P30_D10_flyer_stock_occurrence": {
            "helper_path": "cad/nbk_p30_d10_official_occurrence.py",
            "helper_sha256": _sha256(Path(nbk_p30_d10.__file__)),
            "vendor_STEP_path": str(
                nbk_p30_d10.SOURCE_STEP.relative_to(ROOT)
            ).replace("\\", "/"),
            "vendor_STEP_sha256": nbk_p30_d10.SOURCE_STEP_SHA256,
            "vendor_STEP_bytes": nbk_p30_d10.SOURCE_STEP_BYTES,
            "current_vendor_STEP_sha256": nbk_p30_d10.source_sha256(),
            "CADENAS_order_id": nbk_p30_d10.CADENAS_ORDER_ID,
            "CADENAS_expression": nbk_p30_d10.CADENAS_EXPRESSION,
            "official_part_number": nbk_p30_d10.OFFICIAL_PART_NUMBER,
            "release_catalog_row": reports["flyer_D10_catalog"],
            "stock_occurrence_is_byte_identical_transform_only": True,
            "vendor_STEP_reexported_alone": False,
            "hub_orientation": "rear; source +X maps to machine -Z",
            "placement": {
                "tooth_midplane_center_xyz_mm": [
                    0.0,
                    0.0,
                    drive.PULLEY_CENTER_Z - M2_REAR_SHIFT_MM,
                ],
                "stock_roll_deg": NBK_P30_STOCK_ROLL_DEG,
                "axial_bounds_z_mm": [
                    flyer_shaft_d10.WORLD_REAR_Z_MM,
                    flyer_shaft_d10.SHOULDER_WORLD_Z_MM,
                ],
            },
            "stock_D10_bore_mm": nbk_p30_d10.SOURCE_BORE_DIAMETER_MM,
            "stock_mass_g_consumed_in_balance": nbk_p30_d10.OFFICIAL_MASS_G,
            "stock_axial_moment_of_inertia_kgm2_consumed": (
                nbk_p30_d10.OFFICIAL_AXIAL_MOMENT_OF_INERTIA_KG_M2
            ),
            "supplied_clamp_bolt": nbk_p30_d10.OFFICIAL_CLAMP_BOLT,
            "stock_clamp_torque_Nm": nbk_p30_d10.OFFICIAL_CLAMP_TORQUE_NM,
            "physical_receiving_and_retention_release_authorized": False,
        },
        "released_M2_001_Rev_D_shaft": {
            "authority": "flyer_shaft_d10.flyer_shaft()",
            "source_path": "cad/flyer_shaft_d10.py",
            "source_sha256": _sha256(Path(flyer_shaft_d10.__file__)),
            "STEP_path": "out/custom/step/flyer_shaft_d10_id6_to_id9_l79.step",
            "STEP_sha256": _sha256(RELEASED_SHAFT_STEP),
            "drawing_PDF_path": (
                "output/pdf/flyer_shaft_d10_id6_to_id9_l79.pdf"
            ),
            "drawing_PDF_sha256": _sha256(RELEASED_SHAFT_PDF),
            "custom_manifest_path": "out/custom/manifest.json",
            "custom_manifest_sha256": _sha256(CUSTOM_PARTS_MANIFEST),
            "custom_manifest_row": reports["released_shaft_manifest"],
            "release_catalog_path": "cad/release_catalog.json",
            "release_catalog_sha256": _sha256(RELEASE_CATALOG),
            "release_catalog_row": reports["released_shaft_catalog"],
            "revision": "D",
            "artifact_id": "m2-flyer-shaft-stock-d10-rev-d",
            "design_status": "selected",
            "purchase_status": "rfq_ready",
            "material": "6061-T6 aluminum or drawing-approved equivalent",
            "placement": {
                "rear_datum_z_mm": flyer_shaft_d10.WORLD_REAR_Z_MM,
                "front_datum_z_mm": flyer_shaft_d10.WORLD_FRONT_Z_MM,
                "center_z_mm": RELEASED_SHAFT_CENTER_Z_MM,
            },
            "geometry_authority": {
                "D10_seat_outer_diameter_mm": flyer_shaft_d10.NECK_OD_MM,
                "D10_seat_inner_diameter_mm": flyer_shaft_d10.NECK_ID_MM,
                "D10_seat_length_mm": flyer_shaft_d10.NECK_LENGTH_MM,
                "D10_seat_tolerance": RELEASED_SHAFT_D10_SEAT_TOLERANCE,
                "D10_seat_OD_limits_mm": list(
                    flyer_shaft_d10.NECK_OD_H6_LIMITS_MM
                ),
                "D10_seat_ID_limits_mm": list(
                    flyer_shaft_d10.NECK_ID_LIMITS_MM
                ),
                "minimum_neck_radial_wall_at_limits_mm": (
                    flyer_shaft_d10.MIN_NECK_RADIAL_WALL_AT_LIMITS_MM
                ),
                "main_outer_diameter_mm": flyer_shaft_d10.MAIN_OD_MM,
                "main_inner_diameter_mm": flyer_shaft_d10.MAIN_ID_MM,
                "main_OD_limits_mm": list(
                    flyer_shaft_d10.MAIN_OD_LIMITS_MM
                ),
                "main_ID_limits_mm": list(
                    flyer_shaft_d10.MAIN_ID_LIMITS_MM
                ),
                "length_mm": flyer_shaft_d10.LENGTH_MM,
                "ID6_to_ID9_transition_length_mm": (
                    flyer_shaft_d10.TRANSITION_LENGTH_MM
                ),
                "arm_flat_stations_from_rear_mm": [
                    flyer_shaft_d10.ARM_FLAT_STATION_FROM_REAR_MM,
                    flyer_shaft_d10.ARM_FLAT_STATION_FROM_REAR_MM,
                ],
                "flat_depth_mm": 0.3,
                "both_wire_mouth_fillet_radius_mm": 0.5,
                "pulley_side_flats_prohibited": True,
            },
            "retired_Rev_C_L80p75_artifacts": {
                "governing": False,
                "reason": (
                    "front z=-30 penetrates the PEEK guide and wire; retained "
                    "on disk only until the controlled Rev-D packet regenerates"
                ),
                "STEP_path": (
                    "out/custom/step/"
                    "flyer_shaft_d10_id6_to_id9_l80p75.step"
                ),
                "STEP_sha256": _sha256(RETIRED_REV_C_SHAFT_STEP),
                "drawing_PDF_path": (
                    "output/pdf/"
                    "flyer_shaft_d10_id6_to_id9_l80p75.pdf"
                ),
                "drawing_PDF_sha256": _sha256(RETIRED_REV_C_SHAFT_PDF),
            },
            "old_70mm_three_flat_artifact_governing": False,
        },
        "corrected_printed_arm_shaft_interface": {
            "arm_source_path": "cad/retained_flyer_peek_guide_successor.py",
            "arm_source_sha256": _sha256(Path(flyer_successor.__file__)),
            "integration_source_path": "cad/integrated_release_candidate.py",
            "integration_source_sha256": _sha256(SOURCE),
            "final_union_bore_diameter_mm": ARM_SHAFT_BORE_DIAMETER_MM,
            "root_sleeve_outer_diameter_mm": (
                ARM_SHAFT_ROOT_WEB_OUTER_DIAMETER_MM
            ),
            "root_sleeve_axial_span_z_mm": list(ARM_SHAFT_ROOT_WEB_Z_MM),
            "post_print_operation": audit[
                "corrected_printed_arm_root_sleeve_load_path"
            ]["post_print_manufacturing_contract"],
            "physical_measured_fit_complete": False,
        },
        "felt_preload_and_drag_sizing": {
            "path": "out/reports/felt_loads.json",
            "schema": reports["felt_loads"]["schema"],
            "status": reports["felt_loads"]["status"],
            "file_sha256": _sha256(FELT_LOADS_REPORT),
            "source_path": "cad/felt_loads.py",
            "source_sha256": _sha256(FELT_LOADS_SOURCE),
            "selected_spring": "McMaster 94125K614",
            "normal_preload_band_N": [
                reports["felt_loads"]["design_preload_band"][
                    "minimum_normal_force_n"
                ],
                reports["felt_loads"]["design_preload_band"][
                    "maximum_normal_force_n"
                ],
            ],
            "modeled_drag_band_N": [1.0, 10.0],
            "wingnut_travel_turns": reports["felt_loads"][
                "design_preload_band"
            ]["wingnut_turns"],
            "current_integration_ready": True,
        },
        "felt_contact_companion_review": {
            "path": "out/reports/integrated_felt_contact_review.json",
            "schema": reports["felt_contact"]["schema"],
            "status": reports["felt_contact"]["status"],
            "file_sha256": _sha256(FELT_CONTACT_REPORT),
            "source_path": "cad/integrated_felt_contact_review.py",
            "source_sha256": _sha256(FELT_CONTACT_SOURCE),
            "actual_and_0p5mm_changeover_geometry_PASS": True,
        },
        "frozen_base_raw_clearance_diagnostic": {
            "clearance_path": (
                "out/reports/integrated_candidate_base_clearance_raw.json"
            ),
            "clearance_sha256": _sha256(BASE_RAW_CLEARANCE_REPORT),
            "part_diagnostic_path": (
                "out/reports/integrated_candidate_base_clearance_raw_parts.json"
            ),
            "part_diagnostic_sha256": _sha256(BASE_RAW_PARTS_REPORT),
            "status": reports["base_raw"]["status"],
            "penetration_count": len(reports["base_raw"]["collisions"]),
            "old_target_mm": reports["base_raw"]["target_mm"],
            "old_minimum_mm": reports["base_raw"][
                "minimum_dynamic_clearance_mm"
            ],
            "identified_part_minima_mm": {
                "extended_hollow_shaft_to_entry_bracket": 1.0,
                "flyer_P30_to_entry_bracket": 1.75,
            },
            "consumed_by_entry_rear_shift_redesign": True,
            "frozen_adapter_predates_this_source": True,
            "raw_rerun_was_required_by_frozen_diagnostic": True,
            "current_exact_full_raw_rerun_consumed": reports[
                "active_sector"
            ]["release_gates"]["full_raw_rigid_sweep_clear"],
        },
        "selected_M2_motor": {
            "model": "Leadshine CS-M21708",
            "geometry": "exact vendor solids with cable/connector solids filtered",
            "report_path": "out/reports/leadshine_cs_m21708_cableless.json",
            "report_sha256": _sha256(LEADSHINE_REPORT),
            "source_path": "cad/leadshine_cs_m21708_cableless.py",
            "source_sha256": _sha256(Path(leadshine.__file__)),
            "vendor_step_path": "cad/models/upgrades/CS-M21708.STEP",
            "vendor_step_sha256": leadshine.SOURCE_SHA256,
            "cableless_step_path": "cad/models/upgrades/CS-M21708_cableless.step",
            "cableless_step_sha256": _sha256(leadshine.OUTPUT_STEP),
            "shaft_profile": "D",
            "shaft_across_flat_mm": 4.5,
            "stock_round_bore_split_clamp_authorized": False,
            "old_17HS24_motor_gate_carried_forward": False,
        },
        "current_main": {
            "source_sha256": _sha256(Path(assembly.__file__)),
            "validation_path": "out/reports/validation.json",
            "validation_sha256": _sha256(MAIN_VALIDATION_REPORT),
            "frame_hardware_audit_sha256": _sha256(FRAME_HARDWARE_REPORT),
            "carriage_hardware_audit_sha256": _sha256(CARRIAGE_HARDWARE_REPORT),
        },
        "hardware_release_audit_snapshot": {
            "path": "out/reports/hardware_release_audit.json",
            "sha256": _sha256(HARDWARE_RELEASE_AUDIT),
            "schema": _load(HARDWARE_RELEASE_AUDIT).get("schema"),
            "status": _load(HARDWARE_RELEASE_AUDIT).get("status"),
            "audit_predates_this_candidate": True,
            "reconciled_blocker_ids": {
                "release.integrated_candidate_missing": "resolved geometrically by this candidate; release artifacts still open",
                "release.balance_solution_invalid_after_p30_swap": "resolved nominally by integrated OCC re-solve; physical G2.5 remains open",
                "release.retained_review_uses_cap_proxies": "resolved geometrically with actual cap_part(+/-1)",
                "release.retained_review_uses_legacy_m2_drive": "resolved geometrically with exact Leadshine, official stock D5 motor-side NBK P30, official stock D10 flyer-side NBK P30, and the retained 210-3GT belt envelope",
                "release.extended_shaft_artifact_wrong_length": (
                    "RESOLVED by released M2-001 Rev D L79.00 stock-D10 "
                    "OD10/ID6-to-OD12/ID9 STEP, drawing PDF, RFQ-ready row "
                    "and exact BREP "
                    "integration; old 70 mm artifact is non-governing"
                ),
                "release.candidate_fasteners_missing_or_stale": "OPEN",
            },
        },
    }

    hardware = {
        "counterweight_stacks": 6,
        "rear_counterweight_stacks": 4,
        "rear_counterweight_stack_each": [
            "ASTM-B777 annular tungsten slug",
            "integral printed retainer with three spacer posts",
            "McMaster 94459A130 M3x4.3 heat-set insert",
            "McMaster 92125A126 / ISO10642 M3x6 screw",
        ],
        "front_balance_trim_stacks": {
            "count": 2,
            "ASTM_B777_OD_ID_mm": [
                flyer_successor.FRONT_TRIM_OD_MM,
                flyer_successor.FRONT_TRIM_ID_MM,
            ],
            "common_thickness_mm": integrated_balance_solution()[
                "front_trim_common_thickness_mm"
            ],
            "M2x8_screws": 2,
            "M2_plain_washers": 2,
            "M2_standard_heat_set_inserts": 2,
        },
        "one_piece_PEEK_flyer_guide": {
            "guide_and_exit_bell": 1,
            "M2x6_screws": 3,
            "M2_standard_heat_set_inserts": 3,
            "obsolete_ceramic_torus": 0,
        },
        "cap_retention": {
            "M2x20_screws": 3,
            "M2_front_washers": 3,
            "M2_rear_washers": 3,
            "M2_nyloc_nuts": 3,
        },
        "entry_module": {
            "rear_shift_mm": ENTRY_REAR_SHIFT_MM,
            "entry_bracket_prints": 1,
            "ceramic_eyelets": 1,
            "M5x12_base_screws": 2,
            "base_tnuts": 2,
            "integral_dancer_anchor_keeper": 1,
            "configured_wire_passage_radius_mm": ENTRY_PASSAGE_RADIUS_MM,
        },
        "released_flyer_shaft": {
            "quantity": 1,
            "part": "M2-001 Rev D L79.00 stock-D10 OD10/ID6 to OD12/ID9 shaft",
            "material": "6061-T6 aluminum or drawing-approved equivalent",
            "pulley_side_flats": 0,
            "printed_arm_M3x8_set_screws": 2,
            "ID6_rear_R0p50_wire_mouths": 1,
            "ID9_front_R0p50_wire_mouths": 1,
            "old_70mm_artifact_used": False,
        },
        "printed_arm_shaft_retention": {
            "OD18_ID12p10_root_sleeve_length_mm": 11.5,
            "post_print_ID12p10_ream_required": True,
            "as_printed_FDM_bore_accepted": False,
            "measured_fit_required_before_physical_balance": True,
            "measured_fit_complete": False,
        },
        "successor_drive": {
            "P30_pulleys": 2,
            "210_3GT_6_belts": 1,
            "Leadshine_CS_M21708_NEMA17": 1,
            "motor_M3x10_screws": 4,
            "official_NBK_P30_stock_vendor_occurrences": 2,
            "official_motor_D5_stock_occurrences": 1,
            "official_flyer_D10_stock_occurrences": 1,
            "official_stock_split_clamp_and_bolt_in_each_vendor_occurrence": True,
            "NBK_BNW_M3_upper_bound_hole_path_witnesses": 2,
            "NBK_BNW_M3x12_set_screw_upper_bound_witnesses": 2,
            "NBK_BNW_delivered_set_screw_size_known": False,
            "flyer_stock_supplied_M2_clamp_bolts": 1,
            "retired_smooth_envelope_pulley_occurrences": 0,
        },
        "active_sector_terminal_guide": {
            "M0_following_M1_static_PEEK_guides": 2,
            "one_solid_aluminum_yoke": 1,
            "revised_keyed_spindle_tower": 1,
            "M3x14_screws": 4,
            "M3_plain_washers": 4,
            "M3_short_heat_set_inserts": 4,
            "M4x10_screws": 4,
            "M4_plain_washers": 4,
            "M4_short_heat_set_inserts": 4,
        },
        "shaft_clamp_radial_holes_are_not_counterweights": 2,
    }
    active_gates = reports["active_sector"]["release_gates"]
    release_gates = {
        "targeted_reference_pose_geometry": geometry_pass,
        "full_raw_cycle_collision_regenerated": active_gates[
            "full_raw_rigid_sweep_clear"
        ],
        "all_2400_deposition_terminal_routes_exact_and_clear": (
            active_gates["all_2400_physical_bell_terminal_routes_pass"]
            and active_gates["deposition_exact_rigid_pairs_clear_ge_2mm"]
            and active_gates["arbitrary_M1_caps_and_copper_clear_ge_2mm"]
            and active_gates[
                "summed_tolerance_and_10N_deflection_clearance_ge_2mm"
            ]
        ),
        "continuous_conductor_from_spool_through_every_deposited_turn": False,
        "park_index_load_unload_continuous_conductor_proven": False,
        "both_raw_wrap_wire_paths_bypass_fixed_guide_yoke": active_gates[
            "both_raw_wrap_wire_paths_bypass_fixed_guide_yoke"
        ],
        "both_raw_shaft_wraps_exactly_two_turns": False,
        "dynamic_wire_tension_sag_snag_and_enamel_wear_proven": False,
        "configured_0p22352mm_wire_contacts_both_felt_pads_and_dancer": (
            audit["checks"]["targeted_intended_contacts_established"]
        ),
        "entry_reference_BREP_shaft_and_flyer_P30_clear_ge_2p2mm": (
            audit["checks"][
                "entry_shaft_and_flyer_P30_clear_ge_2p2mm_with_reserve"
            ]
        ),
        "entry_wire_handoff_and_mounting_support_revalidated": (
            audit["checks"][
                "entry_wire_passage_and_axial_handoff_are_open_and_collinear"
            ]
            and audit["checks"][
                "entry_module_mounting_and_dancer_anchor_support_preserved"
            ]
        ),
        "felt_preload_spring_and_drag_sizing_PASS": True,
        "felt_actual_and_0p5mm_changeover_contact_geometry_PASS": True,
        "felt_operating_drag_pull_gauge_calibrated": False,
        "integrated_nominal_two_plane_balance_re_solved": audit["checks"][
            "integrated_OCC_two_plane_balance_exact"
        ],
        "physical_G2p5_two_plane_balance": False,
        "new_L79_stock_D10_shaft_STEP_drawing_RFQ_and_arm_flats_released": (
            audit["checks"][
                "released_M2_001_Rev_D_L79_D10_ID6_ID9_shaft_exact_BREP_and_R0p50_mouths"
            ]
            and audit["checks"][
                "released_shaft_stock_D10_clamp_and_arm_M3x8_retention_valid"
            ]
            and audit["checks"][
                "released_shaft_bearing_spacer_pulley_shim_and_arm_fits_valid"
            ]
            and audit["checks"][
                "released_shaft_rear_entry_and_front_root_wire_handoffs_open"
            ]
            and audit["checks"][
                "released_Rev_D_shaft_front_setback_and_PEEK_wire_clearance_valid"
            ]
            and audit["checks"][
                "corrected_arm_root_sleeve_is_one_solid_positive_load_path_and_strength_PASS"
            ]
            and audit["checks"][
                "printed_arm_ID12p10_single_setup_ream_and_measured_fit_contract_specified"
            ]
        ),
        "printed_arm_ID12p10_post_ream_measured_fit_before_physical_balance": False,
        "printed_arm_root_sleeve_orientation_matched_strength_coupon": False,
        "coupled_exact_live_line_Leadshine_36V_margin_ge_2x": audit[
            "final_integrated_M2_torque"
        ]["Leadshine_36V_gate_ge_2x"],
        "coupled_exact_live_line_P30_210_3GT_capacity_ge_2x": audit[
            "final_integrated_M2_torque"
        ]["P30_210_3GT_gate_ge_2x"],
        "coupled_exact_live_line_M1_margin_ge_2x": active_gates[
            "M1_wrap_governed_margin_ge_2x"
        ],
        "coupled_exact_live_line_M0_margin_ge_2x": active_gates[
            "M0_terminal_force_and_added_mass_margin_ge_2x"
        ],
        "terminal_guide_2400_locus_live_line_moment_arm_consumed": True,
        "M2_36V_driver_configuration_verified": False,
        "M2_installed_hot_dyno_verified": False,
        "M1_closed_loop_drive_fault_safe_behavior_verified": False,
        "old_17HS24_motor_gate_not_reused": True,
        "official_stock_NBK_P30_D5_and_D10_STEP_placement_and_mass_authority": (
            audit["checks"][
                "official_NBK_P30_D5_motor_vendor_STEP_hash_pinned_and_unmodified"
            ]
            and audit["checks"][
                "official_NBK_P30_D5_motor_occurrence_one_solid_at_existing_tooth_midplane"
            ]
            and audit["checks"][
                "official_NBK_P30_D5_motor_mass_28g_and_axial_J_3e_6_authority_recorded"
            ]
            and audit["checks"][
                "official_NBK_P30_D10_flyer_vendor_STEP_hash_pinned_and_unmodified"
            ]
            and audit["checks"][
                "official_NBK_P30_D10_flyer_occurrence_one_solid_hub_rear_on_full_seat"
            ]
            and audit["checks"][
                "official_NBK_P30_D10_flyer_mass_28g_and_axial_J_3e_6_consumed_in_balance"
            ]
        ),
        "exact_configured_NBK_P30_BNW_CAD_and_retention_rating": False,
        "stock_flyer_D10_bore_clamp_reversing_slip_and_crush_cycle": False,
        "selected_frame_window_relocation_ge_2p2mm": audit["checks"][
            "selected_frame_window_relocation_keeps_2p2mm_motor_rear_clearance"
        ],
        "all_six_counterweight_fasteners_have_closed_material_load_paths": (
            audit["checks"][
                "all_six_counterweight_fasteners_terminate_in_positive_material"
            ]
        ),
        "counterweight_heat_set_fit_and_each_20N_pull_coupon": False,
        "PEEK_supplier_DFM_certified_lot_60N_hot_varnish_coupon": False,
        "PEEK_abrasion_and_dielectric_coupons": False,
        "BOM_procurement_settings_and_print_files_integrated": False,
        "mandatory_integrated_snapshot_packet_reviewed": (
            len(integrated_snapshot_packet()) == 4
        ),
        "production_authorized": False,
    }
    status = (
        "REFERENCE_GEOMETRY_PASS_RELEASE_GATES_OPEN"
        if geometry_pass
        else "FAIL_CLOSED_INTEGRATION_GEOMETRY_AND_RELEASE_GATES_OPEN"
    )
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": status,
        "production_authorized": False,
        "main_assembly_replacement_authorized": False,
        "coordinate_frame": (
            "machine reference pose millimetres; M0=M1=M2=0; complete M2 "
            "module translated -10Z; exact Leadshine CS-M21708 motor; entry "
            "module translated -4.25Z with preserved dancer-anchor datum; "
            "base-rail window translated -7.5Z; felt moving "
            "stack set for 0.22352 mm contact; official stock NBK D5 motor "
            "and D10 flyer P30 tooth midplanes retained, with separate "
            "positive motor-side BNW review witnesses"
        ),
        "paths": {
            "source": "cad/integrated_release_candidate.py",
            "step": "out/review/integrated_release_candidate.step",
            "manifest": "out/review/integrated_release_candidate.manifest.json",
            "report_json": "out/reports/integrated_release_candidate.json",
            "report_markdown": "out/reports/integrated_release_candidate.md",
            "active_sector_report": (
                "out/reports/"
                "carriage_active_sector_terminal_guide_audit.json"
            ),
            "active_sector_locus_api": (
                "out/reports/"
                "carriage_active_sector_terminal_guide_loci.json"
            ),
        },
        "public_api": {
            "build_links": "physical collision links; static/carriage/spindle/flyer; positive BNW hole-path review solids excluded",
            "link_location": "unchanged upstream radians/mm kinematics",
            "wire_visuals": "non-rigid review solids keyed by kinematic owner",
            "successor_drive_parts": "named Leadshine, immutable official stock D5 motor and D10 flyer P30 occurrences, motor-side BNW hole/screw witnesses, belt and mount",
            "official_motor_pulley_review": "immutable D5 vendor occurrence placement plus four non-destructive BNW review witnesses",
            "official_flyer_pulley_review": "immutable D10 vendor occurrence placed hub-rear on the complete 18.5 mm round seat",
            "shaft_with_integrated_p30_flats": "released M2-001 Rev D L79.00 shaft with OD10/ID6 pulley seat, OD12/ID9 main span, two arm flats and R0.50 mouths at z=-110.75..-31.75",
            "released_shaft_front_interface_audit": "exact Rev-D shaft setback plus zero-overlap PEEK-guide and conductor gate",
            "retained_arm_with_released_shaft_bore": "superseded helper retained only as the pre-PEEK failure comparison",
            "retained_rotating_parts": "exact L79.00 shaft/official stock D10 P30/root-sleeve/PEEK-guide/six-stack occurrences; overrides fail closed",
            "integrated_balance_solution": "public six-trim solve consuming the official flyer-pulley 28 g and 3e-6 kg m2 authority plus exact remaining OCC rows",
            "cap_module_parts": "one-solid short-leadin PEEK cap pair plus 12 retention occurrences at any stator-axis Z",
            "carriage_module_parts": "M0-following M1-static keyed PEEK guide pair, one-solid aluminum yoke, revised tower and complete M3/M4 retention",
            "configured_static_supply_wire": "actual 0.22352 mm felt/dancer/bore contact route",
            "configured_static_supply_passage_tool": "R1.6 cut swept on the exact actual-wire centerline",
            "integrated_entry_bracket": "rear-shifted one-solid entry print with integral dancer-anchor keeper and recut passage",
            "integrated_slug_lengths": "four rear B777 cuts paired with the solved common thickness of two front B777 trims",
            "integrated_six_stack_attachment_audit": "four rear M3 retainer-boss and two front M2 blind-spoke insert load paths; open-air fasteners fail closed",
            "pre_terminal_integrated_m2_torque_audit": "conservative selection-bound drivetrain seam used only before the active route",
            "final_integrated_m2_torque_audit": "exact 2400-locus physical-bell live-line lever with final flyer J and coupled M0/M1/M2 margins",
            "machine": "posed labeled review assembly",
            "gen_step": "M0=M1=M2=0 primary STEP",
        },
        "source_contracts": source_contracts,
        "transforms": {
            "M2_static_bearing_drive_and_pulley_module_mm": [0.0, 0.0, -M2_REAR_SHIFT_MM],
            "entry_bracket_eyelet_and_mounting_hardware_mm": [0.0, 0.0, -ENTRY_REAR_SHIFT_MM],
            "base_rail_frame_window_mm": [
                0.0, 0.0, -INTEGRATED_FRAME_WINDOW_REAR_SHIFT_MM
            ],
            "felt_moving_pad_backing_spring_thrust_and_wingnut_mm": [
                0.0, 0.0, -FELT_MOVING_STACK_TRAVEL_MM
            ],
            "stator_caps_reference_axis_z_mm": P.m0_home_standoff,
            "actual_cap_deep_clearance_axis_z_mm": retained.base.deepest_axis_z_mm(),
        },
        "hardware_occurrence_contract": hardware,
        "P30_NBK_interface_assumptions": {
            "ratio": "30T motor / 30T flyer = exact 1:1",
            "belt": "210-3GT-6 at frozen 60 mm centre distance",
            "selection_interface_refinement": (
                "immutable byte-identical official stock NBK D5 motor and "
                "D10 flyer STEPs now replace both old pulley envelopes at "
                "their unchanged tooth midplanes. Motor-side BNW remains "
                "two unreleased holes at 90 degrees; "
                "two separate M3 hole paths and M3x12 screw solids are "
                "upper-bound review witnesses, not modified vendor CAD or "
                "delivered-hardware claims"
            ),
            "motor_pulley_geometry": (
                "official NBK P30-3GT-BLP-6C-5 AP214 stock STEP, pinned "
                "byte-for-byte and transformed only, including its stock "
                "split-clamp details. BNW hole paths and M3x12 screws remain "
                "four separate positive review occurrences and are never "
                "booleanned into or out of the vendor solid"
            ),
            "motor_shaft_geometry": (
                "exact Leadshine vendor D-profile, OD5 nominal and 4.5 mm "
                "across flat"
            ),
            "motor_interface_authorized": False,
            "motor_interface_reason": (
                "the two orthogonal M3x12 upper-bound witnesses now touch the "
                "round side and D-flat exactly; the D-flat witness is threaded "
                "0.5 mm inward while preserving full length and socket access. "
                "The configured supplier drawing, exact "
                "delivered screw size/station, shaft hardness match and "
                "reversing torque rating remain absent"
            ),
            "flyer_pulley_geometry": (
                "official NBK P30-3GT-BLP-6C-10 stock A2017 STEP, hub-rear, "
                "D10 through bore on a full-length OD10 h6 round seat, with "
                "the supplied SCM435 M2 split-clamp bolt at 0.5 N m"
            ),
            "flyer_hub_torque_capacity_authorized": False,
            "belt_and_pulley_teeth": (
                "both pulleys are exact official vendor topology; the belt "
                "retains its validated 210-3GT-6 supplier envelope"
            ),
        },
        "geometry": audit,
        "release_gates": release_gates,
        "open_blockers": [
            "M2-001 Rev D selects the exact L79.00 stock-D10 OD10/ID6-to-OD12/ID9 shaft source and RFQ row; the Rev-C L80.75 STEP/PDF remain stale, explicitly non-governing outputs until controlled regeneration. Before physical balance, regenerate and inspect the Rev-D D10 h6 seat and shoulder, ream the final unioned printed collar/root sleeve in one setup to 12.10..12.13 mm, gauge the OD12 span at 11.98..12.00 mm, record the resulting 0.10..0.15 mm diametral arm fit and prove a burr-free full-length hand slide before seating both M3x8 screws.",
            "The static spool/felt/dancer/entry route and all 2400 winding terminal loci through the shaft, one-piece PEEK bell, M0-following active guide, short cap lead-in and deposited-turn terminal binding are now integrated. The flexible conductor during park, index, load and unload remains unproved; both raw shaft-wrap free spans bypass the fixed guide, but the immutable upstream capture commands 1.375 and 2.791667 turns rather than the GOAL text's two turns each.",
            "Felt preload sizing and both the operating 0.22352 mm and separate 0.5 mm contact geometries pass. The remaining felt-specific empirical limitation is pull-gauge calibration on production wire at operating speed; wingnut turns are not a force certificate.",
            f"The exact all-2400-locus maximum perpendicular live-line lever is {audit['final_integrated_M2_torque']['maximum_perpendicular_live_line_lever_mm']:.6f} mm. With the official flyer-pulley 3e-6 kg m2 contribution in the exact final flyer J, the selected 36 V Leadshine curve gives {audit['final_integrated_M2_torque']['Leadshine_36V_available_to_required_multiple']:.6f}x and the P30/210-3GT transmission gives {audit['final_integrated_M2_torque']['P30_210_3GT_available_to_required_multiple']:.6f}x. Driver current/microstep configuration, installed friction/hot dyno, configured motor-side BNW drawing/coupon, delivered D10 receiving inspection, stock-clamp reversing-slip cycle, belt pretension and shoulder fatigue remain open; 24 V arithmetic is reported separately and is not release-authorized.",
            "All six balance-correction fasteners now terminate in real printed material: four M3 screws close through slug/spacer/retainer bosses with blind material, and two M2x8 screws enter standard inserts in blind main-spoke pilots. Their exact geometry passes; insert-fit and per-stack pull coupons remain open.",
            "The OD18/ID12.10 root sleeve passes a conservative simultaneous 10 N-at-R64 bending/torsion screen at 3x safety factor, but the 10 MPa PETG screening allowable is not a material certificate. An orientation-matched printed root-sleeve coupon, physical counterweight, PEEK and G2.5 balance qualifications remain open.",
            "The current full raw rigid sweep is regenerated against the exact candidate links and active-sector hardware. It proves rigid clearance through winding, parking, indexing, load-position and both shaft-wrap motion classes, but does not prove flexible-conductor sag, snagging, friction, enamel wear or the unmodeled park/index/load/unload wire shape.",
        ],
        "source_hashes": {
            "cad/integrated_release_candidate.py": _sha256(SOURCE),
            "cad/assembly.py": _sha256(Path(assembly.__file__)),
            "cad/permanent_cap_offset_spoke_retained_review.py": _sha256(Path(retained.__file__)),
            "cad/permanent_cap_production_review.py": _sha256(Path(caps.__file__)),
            "cad/retained_flyer_peek_guide_successor.py": _sha256(
                Path(flyer_successor.__file__)
            ),
            "out/reports/retained_flyer_peek_guide_successor.json": _sha256(
                FLYER_GUIDE_REPORT
            ),
            "cad/carriage_active_sector_terminal_guide.py": _sha256(
                Path(terminal_guide.__file__)
            ),
            "sim/carriage_active_sector_terminal_guide_audit.py": _sha256(
                ACTIVE_SECTOR_AUDIT_SOURCE
            ),
            "out/reports/carriage_active_sector_terminal_guide_audit.json": _sha256(
                ACTIVE_SECTOR_REPORT
            ),
            "out/reports/carriage_active_sector_terminal_guide_loci.json": _sha256(
                ACTIVE_SECTOR_LOCI
            ),
            "out/review/carriage_active_sector_terminal_guide.step": _sha256(
                ACTIVE_SECTOR_STEP
            ),
            "out/review/carriage_active_sector_terminal_guide.manifest.json": _sha256(
                ACTIVE_SECTOR_MANIFEST
            ),
            "cad/m2_drive_successor_review.py": _sha256(Path(drive.__file__)),
            "cad/custom_parts.py": _sha256(CUSTOM_PARTS_SOURCE),
            "cad/flyer_shaft_d10.py": _sha256(Path(flyer_shaft_d10.__file__)),
            "out/custom/step/flyer_shaft_d10_id6_to_id9_l79.step": _sha256(
                RELEASED_SHAFT_STEP
            ),
            "output/pdf/flyer_shaft_d10_id6_to_id9_l79.pdf": _sha256(
                RELEASED_SHAFT_PDF
            ),
            "out/custom/manifest.json": _sha256(CUSTOM_PARTS_MANIFEST),
            "cad/release_catalog.json": _sha256(RELEASE_CATALOG),
            "cad/nbk_p30_official_occurrence.py": _sha256(Path(nbk_p30.__file__)),
            str(nbk_p30.SOURCE_STEP.relative_to(ROOT)).replace("\\", "/"): (
                _sha256(nbk_p30.SOURCE_STEP)
            ),
            "cad/nbk_p30_d10_official_occurrence.py": _sha256(
                Path(nbk_p30_d10.__file__)
            ),
            str(nbk_p30_d10.SOURCE_STEP.relative_to(ROOT)).replace("\\", "/"): (
                _sha256(nbk_p30_d10.SOURCE_STEP)
            ),
            "cad/leadshine_cs_m21708_cableless.py": _sha256(Path(leadshine.__file__)),
            "cad/models/upgrades/CS-M21708.STEP": _sha256(leadshine.SOURCE_STEP),
            "cad/models/upgrades/CS-M21708_cableless.step": _sha256(leadshine.OUTPUT_STEP),
            "cad/felt_loads.py": _sha256(FELT_LOADS_SOURCE),
            "out/reports/felt_loads.json": _sha256(FELT_LOADS_REPORT),
            "cad/integrated_felt_contact_review.py": _sha256(FELT_CONTACT_SOURCE),
            "out/reports/integrated_felt_contact_review.json": _sha256(FELT_CONTACT_REPORT),
            "out/reports/integrated_candidate_base_clearance_raw.json": _sha256(BASE_RAW_CLEARANCE_REPORT),
            "out/reports/integrated_candidate_base_clearance_raw_parts.json": _sha256(BASE_RAW_PARTS_REPORT),
            "out/review/integrated_release_candidate.step": _sha256(STEP_OUT),
            "out/review/.integrated_release_candidate.step.glb": _sha256(
                REVIEW / ".integrated_release_candidate.step.glb"
            ),
        },
        "visual_review": {
            "reviewed": len(integrated_snapshot_packet()) == 4,
            "snapshot_packet": integrated_snapshot_packet(),
            "findings": [
                "full frame, carriage, spindle/cap stator, retained flyer and supply modules are present in both opposed isometric views",
                "the exact Leadshine body appears once at the shifted M2 mount; no duplicate old M2 motor, P40 or 200-2GT drive is visible",
                "official NBK stock P30 occurrences replace both prior pulley envelopes: D5 at the motor and D10 hub-rear at the flyer; separate BNW hole/screw witnesses remain local to the motor hub",
                "the entry print, tube, eyelet and base hardware occupy the selected rearward datum while the integral keeper retains the fixed dancer-spring screw seat",
                "the released M2-001 Rev D shaft spans z=-110.75..-31.75 with a full OD10/ID6 stock-pulley seat, OD12/ID9 main span, polished R0.50 rear/front mouths and only two retained-arm flats; its front is 0.25 mm behind the corrected OD18 root-sleeve front and clears the PEEK guide and conductor",
                "the physical PEEK cap pair is visibly the repeated 24-sector geometry rather than fan collision proxies",
                "no counterweight, cap-retention or drive occurrence appears visibly detached at full-assembly scale",
                "top and front orthographic views preserve the intended machine symmetry and shifted rear frame boundary",
            ],
            "visual_concerns_converted_to_deterministic_checks": [
                "all six positive balance-correction stack occurrence, blind-material termination and one-solid arm gates",
                "actual deep PEEK-cap versus arm intersection and distance",
                "targeted cross-module positive-volume intersections",
                "official belt/pulley contact plus separate BNW hole/screw witness packaging",
                "configured wire/felt/dancer contact distances",
                "exact motor rear to frame-window clearance",
                "shaft/flyer-P30 to entry clearances, entry attachment chain and exact wire-passage wall clearance",
                "released shaft one-solid/D10-seat/transition/datum/arm-flat/mouth topology, two arm screw-flat contacts, stock clamp/bearing/spacer/shim fits, corrected arm-bore clearance and positive root-sleeve overlap chain",
            ],
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("integrated release-candidate report hash mismatch")


def render_markdown(report: Mapping[str, Any]) -> str:
    geom = report["geometry"]
    frame = geom["frame_window_interface"]
    mass = geom["integrated_rotating_mass_properties"]
    shaft = geom["released_M2_001_Rev_D_shaft_BREP"]
    root = geom["corrected_printed_arm_root_sleeve_load_path"]
    attachment = geom["integrated_six_stack_attachment_audit"]
    torque = geom["final_integrated_M2_torque"]
    slug_text = ", ".join(
        f"{name}={value:.6f}"
        for name, value in geom["integrated_slug_length_solution_mm"].items()
    )
    lines = [
        "# Normal-GOAL integrated release candidate",
        "",
        f"**{report['status']}** — production and main-assembly replacement authority are false.",
        "",
        "This source-level candidate combines the current frame/carriage/chuck, active-sector guide and short-leadin PEEK caps with complete hardware, the retained six-stack flyer and one-piece PEEK guide/bell, the exact winding terminal route, and the exact-1:1 Leadshine/P30/3GT drive geometry. `cad/assembly.py` is unchanged.",
        "",
        "## Exact integration findings",
        "",
        f"- Actual deep PEEK-cap to retained-arm distance: {geom['clearances_mm']['actual_deep_cap_pair_to_retained_arm_mm']:.6f} mm.",
        f"- Retained arm to shifted flyer block: {geom['clearances_mm']['retained_arm_to_shifted_flyer_block_mm']:.6f} mm.",
        f"- Released M2-001 Rev D shaft: L{shaft['length_mm']:.2f} mm at z={shaft['rear_datum_z_mm']:.2f}..{shaft['front_datum_z_mm']:.2f}; rear seat OD{shaft['neck_outer_diameter_mm']:.2f}/ID{shaft['neck_inner_diameter_mm']:.2f} x {shaft['D10_seat_length_mm']:.2f} mm, main OD{shaft['main_outer_diameter_mm']:.2f}/ID{shaft['main_inner_diameter_mm']:.2f}, both mouths R{shaft['wire_mouth_fillet_radius_mm']:.2f}.",
        f"- Corrected arm/shaft radial clearance: {root['corrected_interface']['arm_to_shaft_radial_clearance_mm']:.6f} mm; root sleeve web overlaps collar/spoke/rail by {root['root_sleeve_web']['web_to_existing_collar_overlap_mm3']:.6f}/{root['root_sleeve_web']['web_to_main_spoke_overlap_mm3']:.6f}/{root['root_sleeve_web']['web_to_rear_counterrail_overlap_mm3']:.6f} mm3.",
        f"- Conservative simultaneous 10 N at R64 root load: {root['conservative_combined_root_load_case']['von_Mises_equivalent_MPa']:.6f} MPa equivalent, {root['conservative_combined_root_load_case']['safety_factored_equivalent_MPa']:.6f} MPa at {root['conservative_combined_root_load_case']['review_safety_factor']:.1f}x versus {root['conservative_combined_root_load_case']['PETG_review_allowable_MPa']:.1f} MPa screening allowable.",
        f"- Candidate motor rear / selected frame-window clearance: {frame['selected_clearance_mm']:.6f} mm; additional rearward boundary shift required for 2.2 mm: {frame['additional_rearward_boundary_shift_required_mm']:.6f} mm.",
        f"- Integrated nominal static residual: {mass['static_imbalance_g_mm']:.9g} g mm; couple residual: {mass['couple_imbalance_g_mm2']:.9g} g mm2.",
        f"- Integrated tungsten cut lengths (mm): {slug_text}.",
        f"- Six-stack attachment: {attachment['status']}; open-air fastener present={attachment['any_balance_fastener_over_open_air']}.",
        f"- Exact 2400-locus M2 live-line lever: {torque['maximum_perpendicular_live_line_lever_mm']:.6f} mm; 36 V margin {torque['Leadshine_36V_available_to_required_multiple']:.6f}x; P30/210-3GT margin {torque['P30_210_3GT_available_to_required_multiple']:.6f}x.",
        f"- Targeted unintended-overlap maximum: {max(geom['unintended_overlaps_mm3'].values()):.9g} mm3.",
        "",
        "## Checks",
        "",
    ]
    lines.extend(
        f"- {name}: {'PASS' if value else 'FAIL'}"
        for name, value in geom["checks"].items()
    )
    lines.extend(["", "## Release gates", ""])
    lines.extend(
        f"- {name}: {'PASS' if value else 'OPEN'}"
        for name, value in report["release_gates"].items()
    )
    lines.extend(["", "## Open blockers", ""])
    lines.extend(f"- {item}" for item in report["open_blockers"])
    return "\n".join(lines) + "\n"


def write_reports(report: Mapping[str, Any] | None = None) -> dict[str, Any]:
    value = dict(report or analyze())
    validate_report_integrity(value)
    REPORTS.mkdir(parents=True, exist_ok=True)
    REVIEW.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(json.dumps(value, indent=2), encoding="utf-8")
    MD_OUT.write_text(render_markdown(value), encoding="utf-8")
    manifest = {
        "schema": MANIFEST_SCHEMA,
        "status": value["status"],
        "production_authorized": False,
        "main_assembly_replacement_authorized": False,
        "source": value["paths"]["source"],
        "step": value["paths"]["step"],
        "public_api": value["public_api"],
        "source_contracts": value["source_contracts"],
        "transforms": value["transforms"],
        "hardware_occurrence_contract": value["hardware_occurrence_contract"],
        "P30_NBK_interface_assumptions": value["P30_NBK_interface_assumptions"],
        "geometry_checks": value["geometry"]["checks"],
        "release_gates": value["release_gates"],
        "report_sha256": value["report_sha256"],
    }
    MANIFEST_OUT.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return value


if __name__ == "__main__":
    result = write_reports()
    print(JSON_OUT)
    print(MD_OUT)
    print(MANIFEST_OUT)
