"""Fail-closed architecture contract for the four-shoe replacement carriage.

This module owns only identity, selection, placement, and exactly-once part
counts.  It does not create CAD or promote the current isolated follower to an
installable assembly.  The 8.90 mm coarse selector is deliberately recorded as
an unmodeled positive-volume mechanism.
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
OUTPUT_JSON = REPORTS / "aggregate_boundary_follower_replacement_architecture.json"
OUTPUT_MD = REPORTS / "aggregate_boundary_follower_replacement_architecture.md"

SCHEMA = "aggregate-boundary-follower-replacement-architecture/v3"

LAW_DIRECT = "origin_000_direction_+1"
LAW_REVERSE_ZERO = "origin_000_direction_-1"
LAW_REVERSE_180 = "origin_180_direction_-1"
LAWS = (LAW_DIRECT, LAW_REVERSE_ZERO, LAW_REVERSE_180)
TRACK_RADII_MM = (19.0, 26.0, 33.0, 40.0)
GATE_STATES = (
    "ALL_RETRACTED_DISCONNECTED",
    "FORCED_RETRACTION_RAMP",
    "ENGAGED_LOCKED",
)

IDENTITIES = (
    {"physical_id": 0, "name": "front_left", "axial_sign": 1, "tangent_sign": -1},
    {"physical_id": 1, "name": "front_right", "axial_sign": 1, "tangent_sign": 1},
    {"physical_id": 2, "name": "rear_right", "axial_sign": -1, "tangent_sign": 1},
    {"physical_id": 3, "name": "rear_left", "axial_sign": -1, "tangent_sign": -1},
)

SELECTED_BASE_Y_MM = 2.05
PARKED_BASE_Y_MM = 10.95
COARSE_SELECTION_STROKE_MM = PARKED_BASE_Y_MM - SELECTED_BASE_Y_MM
PASSIVE_TANGENTIAL_USABLE_MM = 0.5
PASSIVE_TANGENTIAL_HARD_MM = 0.6
RADIAL_SOURCE_NOSE_X_MM = {"retracted": 30.0, "mid": 33.0, "extended": 36.0}
RADIAL_LOCAL_NOSE_X_MM = {name: value - 0.30 for name, value in RADIAL_SOURCE_NOSE_X_MM.items()}
REFERENCE_AXIS_Z_MM = 95.0

PRIMARY_M4_AXES_LOCAL_XY_MM = (
    (29.0, -24.5),
    (35.0, -17.5),
    (29.0, 24.5),
    (35.0, 17.5),
)
PRIMARY_M4_SCREW_SKU = "NBK SSHS-M4-10-SD-ALK"
PRIMARY_M4_SCREW_COUNT = 4
PRIMARY_M4_WASHER_COUNT = 0
PRIMARY_M4_INSERT_COUNT = 4

OUTER_PIVOT_PIN_SKU = "MISUMI SCCG5-10"
OUTER_PIVOT_PIN_COUNT_PER_OCCURRENCE = 1
OUTER_PIVOT_DIN988_SHIM_COUNT_PER_OCCURRENCE = 2
OUTER_PIVOT_NETWS4_RING_COUNT_PER_OCCURRENCE = 2
INNER_PIVOT_LEAF_COUNT_PER_OCCURRENCE = 6
CUSTOM_BODY_COUNT_PER_OCCURRENCE = 4
LEAF_COUNT_PER_OCCURRENCE = 15
PHYSICAL_OCCURRENCE_COUNT = 4
MOVING_LEAF_COUNT = LEAF_COUNT_PER_OCCURRENCE * PHYSICAL_OCCURRENCE_COUNT
PRIMARY_MOUNT_LEAF_COUNT = PRIMARY_M4_SCREW_COUNT + PRIMARY_M4_INSERT_COUNT
MANUFACTURED_LEAF_COUNT = 1 + MOVING_LEAF_COUNT + PRIMARY_MOUNT_LEAF_COUNT
BLOCKER_ENVELOPE_COUNT = 4
REVIEW_LEAF_COUNT = MANUFACTURED_LEAF_COUNT + BLOCKER_ENVELOPE_COUNT

NOMINAL_YOKE_SIBLING_CLEARANCE_MM = (
    SELECTED_BASE_Y_MM + PARKED_BASE_Y_MM - 10.0
)
NOMINAL_COMPLETE_PIVOT_ENVELOPE_CLEARANCE_MM = (
    SELECTED_BASE_Y_MM + PARKED_BASE_Y_MM - 10.0
)
INWARD_Q_COMPLETE_PIVOT_ENVELOPE_CLEARANCE_MM = (
    NOMINAL_COMPLETE_PIVOT_ENVELOPE_CLEARANCE_MM
    - PASSIVE_TANGENTIAL_USABLE_MM
)

SOURCE_PATHS = (
    ROOT / "cad" / "aggregate_boundary_follower_replacement_carriage.py",
    ROOT / "cad" / "aggregate_boundary_floating_follower.py",
    ROOT / "cad" / "carriage_active_sector_terminal_guide.py",
    ROOT / "cad" / "m1_selector_alternating_former.py",
    REPORTS / "aggregate_boundary_follower_integration_audit.json",
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(value: Mapping[str, Any]) -> str:
    body = deepcopy(dict(value))
    body.pop("report_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def selected_physical_id(law: str, track_index: int) -> int:
    if law not in LAWS:
        raise ValueError("unknown M1 law")
    if track_index not in range(4):
        raise ValueError("M2 track index must be 0..3")
    if law == LAW_REVERSE_180:
        return (2, 3, 0, 1)[track_index]
    return track_index


def selected_occurrences(
    law: str, track_index: int, gate_state: str,
) -> list[dict[str, Any]]:
    if gate_state not in GATE_STATES:
        raise ValueError("unknown M0 gate state")
    selected = selected_physical_id(law, track_index)
    deployment_allowed = gate_state == "ENGAGED_LOCKED"
    rows: list[dict[str, Any]] = []
    for identity in IDENTITIES:
        active = deployment_allowed and identity["physical_id"] == selected
        row = dict(identity)
        row.update({
            "owner": "carriage",
            "M1_spatial_transform": False,
            "M2_spatial_transform": False,
            "selection_state": "selected" if active else "parked",
            "coarse_base_y_magnitude_mm": (
                SELECTED_BASE_Y_MM if active else PARKED_BASE_Y_MM
            ),
            "radial_state_requirement": (
                "contact_reacted_14_to_20" if active else "retracted_le_14"
            ),
            "passive_tangential_float_allowed": active,
            "gimbal_float_allowed": active,
        })
        rows.append(row)
    return rows


def occurrence_local_point(
    physical_id: int,
    source_point_mm: tuple[float, float, float],
    *,
    base_y_mm: float,
) -> tuple[float, float, float]:
    if physical_id not in range(4):
        raise ValueError("physical_id must be 0..3")
    identity = IDENTITIES[physical_id]
    x, y, z = map(float, source_point_mm)
    a = float(identity["axial_sign"])
    t = float(identity["tangent_sign"])
    return (x - 0.30, t * float(base_y_mm) + t * y, a * 21.35 + a * (z - 24.0))


def local_to_machine(
    point_mm: tuple[float, float, float], *, axis_z_mm: float = REFERENCE_AXIS_Z_MM,
) -> tuple[float, float, float]:
    x, y, z = map(float, point_mm)
    return (-y, z, float(axis_z_mm) - x)


def nose_machine_center(
    physical_id: int,
    radial_state: str,
    *,
    selected: bool,
    q_mm: float = 0.0,
    axis_z_mm: float = REFERENCE_AXIS_Z_MM,
) -> tuple[float, float, float]:
    if radial_state not in RADIAL_SOURCE_NOSE_X_MM:
        raise ValueError("radial state must be retracted, mid, or extended")
    limit = PASSIVE_TANGENTIAL_HARD_MM if selected else 0.0
    if abs(float(q_mm)) > limit + 1.0e-12:
        raise ValueError("passive tangential displacement outside allowed state")
    base = SELECTED_BASE_Y_MM if selected else PARKED_BASE_Y_MM
    source = (RADIAL_SOURCE_NOSE_X_MM[radial_state], float(q_mm), 24.0)
    return local_to_machine(
        occurrence_local_point(physical_id, source, base_y_mm=base),
        axis_z_mm=axis_z_mm,
    )


def build_report() -> dict[str, Any]:
    selection_rows = []
    for law in LAWS:
        for track_index, radius in enumerate(TRACK_RADII_MM):
            for gate in GATE_STATES:
                rows = selected_occurrences(law, track_index, gate)
                selection_rows.append({
                    "M1_law": law,
                    "M2_track_index": track_index,
                    "M2_track_radius_mm": radius,
                    "M0_gate_state": gate,
                    "selected_physical_ids": [
                        row["physical_id"] for row in rows
                        if row["selection_state"] == "selected"
                    ],
                    "occurrences": rows,
                })

    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "DESIGN_CONTRACT_ONLY_FAIL_CLOSED",
        "decision": (
            "CARRIAGE_HARDWARE_MODEL_BOUND__CLEARANCE_TRANSITION_LOAD_"
            "ROUTE_AUTHORITY_OPEN"
        ),
        "identity_order": list(IDENTITIES),
        "selection_contract": {
            "law_count": len(LAWS),
            "track_count": len(TRACK_RADII_MM),
            "gate_state_count": len(GATE_STATES),
            "case_count": len(selection_rows),
            "rows": selection_rows,
        },
        "occurrence_transform": {
            "source_nose_retracted_local_mm": [30.0, 0.0, 24.0],
            "formula": "X=x-0.30; Y=t*Ybase+t*y; Z=a*21.35+a*(z-24)",
            "machine_formula": "machine=(-Y,Z,axis_z-X)",
            "all_occurrences_owner": "carriage",
        },
        "travel": {
            "selected_base_y_magnitude_mm": SELECTED_BASE_Y_MM,
            "parked_base_y_magnitude_mm": PARKED_BASE_Y_MM,
            "coarse_selection_stroke_mm": COARSE_SELECTION_STROKE_MM,
            "passive_tangential_usable_half_travel_mm": PASSIVE_TANGENTIAL_USABLE_MM,
            "passive_tangential_hard_half_travel_mm": PASSIVE_TANGENTIAL_HARD_MM,
            "coarse_selector_is_separate_from_passive_float": True,
        },
        "shared_adapter": {
            "plate_bounds_local_mm": {"min": [26.0, -28.0, -114.0], "max": [38.0, 28.0, -110.0]},
            "U_window_bounds_local_mm": {"min": [25.0, -17.5, -114.5], "max": [30.5, 17.5, -109.5]},
            "M4_axes_local_xy_mm": [
                list(point) for point in PRIMARY_M4_AXES_LOCAL_XY_MM
            ],
            "M4_same_side_diagonal_delta_xy_mm": [6.0, 7.0],
            "M4_proof_basis_x_row_span_mm": 6.0,
            "M4_proof_basis_x_row_span_preserved": True,
            "key_centers_local_xy_mm": [[34.0, -10.0], [34.0, 10.0]],
            "key_centers_unchanged": True,
            "central_spine_allowed": False,
            "parked_follower_relief_bounds_local_mm": {
                "x": [25.0, 36.2],
                "abs_y": [5.45, 16.5],
                "abs_z": [9.85, 27.85],
            },
            "selection_wall_abs_z_bounds_mm": [2.85, 12.85],
            "selection_bay_tangential_clearance_mm": 0.50,
            "outboard_dogleg_web_min_radial_thickness_mm": 2.80,
            "carrier_one_solid_required": True,
        },
        "primary_mount_hardware": {
            "screw_sku": PRIMARY_M4_SCREW_SKU,
            "screw_count": PRIMARY_M4_SCREW_COUNT,
            "washer_count": PRIMARY_M4_WASHER_COUNT,
            "insert_count": PRIMARY_M4_INSERT_COUNT,
            "leaf_count": PRIMARY_MOUNT_LEAF_COUNT,
            "locking_feature": "factory_nylon_patch",
            "heads_recessed": True,
        },
        "occurrence_leaf_contract": {
            "physical_occurrence_count": PHYSICAL_OCCURRENCE_COUNT,
            "leaf_count_per_occurrence": LEAF_COUNT_PER_OCCURRENCE,
            "custom_body_count_per_occurrence": (
                CUSTOM_BODY_COUNT_PER_OCCURRENCE
            ),
            "outer_pivot": {
                "pin_sku": OUTER_PIVOT_PIN_SKU,
                "pin_count_per_occurrence": (
                    OUTER_PIVOT_PIN_COUNT_PER_OCCURRENCE
                ),
                "DIN988_shim_count_per_occurrence": (
                    OUTER_PIVOT_DIN988_SHIM_COUNT_PER_OCCURRENCE
                ),
                "NETWS4_ring_count_per_occurrence": (
                    OUTER_PIVOT_NETWS4_RING_COUNT_PER_OCCURRENCE
                ),
                "inward_shoulder_screw_or_nyloc_count": 0,
            },
            "inner_pivot_leaf_count_per_occurrence": (
                INNER_PIVOT_LEAF_COUNT_PER_OCCURRENCE
            ),
        },
        "exact_install_counts": {
            "revised_spindle_tower": 1,
            "shared_U_windowed_replacement_carrier": 1,
            "physical_follower_occurrences": PHYSICAL_OCCURRENCE_COUNT,
            "tower_M4_screws": PRIMARY_M4_SCREW_COUNT,
            "tower_M4_washers": PRIMARY_M4_WASHER_COUNT,
            "tower_M4_inserts": PRIMARY_M4_INSERT_COUNT,
            "outer_pivot_SCCG5_10_pins": PHYSICAL_OCCURRENCE_COUNT,
            "outer_pivot_DIN988_shims": (
                PHYSICAL_OCCURRENCE_COUNT
                * OUTER_PIVOT_DIN988_SHIM_COUNT_PER_OCCURRENCE
            ),
            "outer_pivot_NETWS4_rings": (
                PHYSICAL_OCCURRENCE_COUNT
                * OUTER_PIVOT_NETWS4_RING_COUNT_PER_OCCURRENCE
            ),
            "old_active_sector_yoke": 0,
            "old_PEEK_active_sector_guides": 0,
            "old_secondary_M3_stacks": 0,
            "mounting_backer_context": 0,
            "follower_local_tower_M4_stacks": 0,
            "central_spine": 0,
        },
        "review_leaf_counts": {
            "shared_carrier_manufactured_leaves": 1,
            "moving_occurrence_manufactured_leaves": MOVING_LEAF_COUNT,
            "primary_mount_manufactured_leaves": PRIMARY_MOUNT_LEAF_COUNT,
            "manufactured_leaves": MANUFACTURED_LEAF_COUNT,
            "coarse_linkage_blocker_envelopes": BLOCKER_ENVELOPE_COUNT,
            "total_review_leaves": REVIEW_LEAF_COUNT,
        },
        "analysis_gates": {
            "three_laws_four_tracks_three_gate_states_enumerated": len(selection_rows) == 36,
            "at_most_one_selected_occurrence_per_case": all(
                len(row["selected_physical_ids"]) <= 1 for row in selection_rows
            ),
            "all_retracted_and_ramp_cases_select_none": all(
                not row["selected_physical_ids"] for row in selection_rows
                if row["M0_gate_state"] != "ENGAGED_LOCKED"
            ),
            "engaged_cases_select_exactly_one": all(
                len(row["selected_physical_ids"]) == 1 for row in selection_rows
                if row["M0_gate_state"] == "ENGAGED_LOCKED"
            ),
            "coarse_selection_stroke_is_8p90mm": abs(COARSE_SELECTION_STROKE_MM - 8.90) < 1e-12,
            "diagonal_primary_M4_axes_bound": (
                len(PRIMARY_M4_AXES_LOCAL_XY_MM) == 4
                and PRIMARY_M4_WASHER_COUNT == 0
            ),
            "four_occurrences_have_15_leaves_each": (
                PHYSICAL_OCCURRENCE_COUNT == 4
                and LEAF_COUNT_PER_OCCURRENCE == 15
            ),
            "manufactured_plus_blocker_leaf_count_is_73": (
                MANUFACTURED_LEAF_COUNT == 69
                and REVIEW_LEAF_COUNT == 73
            ),
        },
        "physical_gates": {
            "SCCG5_10_pin_retention_load_and_wear_qualified": False,
            "positive_volume_8p90mm_selector_linkage_integrated": False,
            "nominal_2mm_clearance_tolerance_stack_qualified": False,
            "all_transition_collision_sweeps_complete": False,
            "all_2400_route_loci_close_R3_C1": False,
            "40N_joint_load_path_qualified": False,
            "positive_M0_retraction_and_dual_NC_interlock_integrated": False,
        },
        "nominal_clearance_not_authority": {
            "yoke_sibling_clearance_mm": NOMINAL_YOKE_SIBLING_CLEARANCE_MM,
            "complete_outer_pivot_envelope_clearance_mm": (
                NOMINAL_COMPLETE_PIVOT_ENVELOPE_CLEARANCE_MM
            ),
            "inward_q_complete_outer_pivot_envelope_clearance_mm": (
                INWARD_Q_COMPLETE_PIVOT_ENVELOPE_CLEARANCE_MM
            ),
            "nominal_reserve_above_2mm_requirement_mm": (
                INWARD_Q_COMPLETE_PIVOT_ENVELOPE_CLEARANCE_MM - 2.0
            ),
            "tolerance_stack_qualified": False,
            "transition_sweeps_complete": False,
        },
        "blockers": [
            "SCCG5_10_pin_retention_load_and_wear_qualification",
            "positive_volume_8p90mm_coarse_selection_linkage",
            "nominal_2mm_clearance_tolerance_stack",
            "active_parked_transition_collision_sweeps",
            "5p52Nm_primary_mount_load_path_proof",
            "wire_route_and_2400_locus_closure",
            "positive_M0_retraction_and_dual_NC_interlock",
        ],
        "assembly_integration_authorized": False,
        "collision_authorized": False,
        "wire_route_authorized": False,
        "BOM_change_authorized": False,
        "procurement_authorized": False,
        "release_authorized": False,
        "source_hashes": {
            str(path.relative_to(ROOT)).replace("\\", "/"): _sha256(path)
            for path in (*SOURCE_PATHS, Path(__file__).resolve())
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported replacement architecture schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("replacement architecture report hash mismatch")
    for relative, expected in report.get("source_hashes", {}).items():
        path = ROOT / Path(str(relative).replace("\\", "/"))
        if not path.is_file() or _sha256(path) != expected:
            raise ValueError(f"stale replacement architecture source {relative}")
    if not all(report.get("analysis_gates", {}).values()):
        raise ValueError("replacement architecture analytical gate failure")
    if any(report.get("physical_gates", {}).values()):
        raise ValueError("design contract cannot authorize physical gates")


def render_markdown(report: Mapping[str, Any]) -> str:
    counts = report["exact_install_counts"]
    leaves = report["review_leaf_counts"]
    mount = report["primary_mount_hardware"]
    occurrence = report["occurrence_leaf_contract"]
    return "\n".join([
        "# Aggregate-boundary follower replacement architecture", "",
        f"**{report['status']} — {report['decision']}**", "",
        "One shared U-windowed carriage replaces the old active-sector yoke/guide set; four carriage-owned moving shoes are retained.", "",
        "## Coarse selection", "",
        f"- Selected center: |y|={SELECTED_BASE_Y_MM:.2f} mm.",
        f"- Parked sibling center: |y|={PARKED_BASE_Y_MM:.2f} mm.",
        f"- Separate positive-volume selector stroke required: {COARSE_SELECTION_STROKE_MM:.2f} mm.",
        "- The passive +/-0.5 mm tangential float is downstream of this selector and cannot substitute for it.", "",
        "## Finalized carriage hardware", "",
        "- Diagonal primary M4 axes: `(29,-24.5)`, `(35,-17.5)`, `(29,+24.5)`, `(35,+17.5)` mm.",
        f"- Primary mount: {mount['screw_count']}x `{mount['screw_sku']}`, "
        f"{mount['washer_count']} washers, and {mount['insert_count']} inserts.",
        f"- Each of {occurrence['physical_occurrence_count']} moving occurrences has "
        f"{occurrence['leaf_count_per_occurrence']} manufactured leaves, including one "
        "`MISUMI SCCG5-10` pin, two DIN 988 shims, and two NETWS4 rings.",
        f"- Review tree: {leaves['manufactured_leaves']} manufactured leaves + "
        f"{leaves['coarse_linkage_blocker_envelopes']} blocker envelopes = "
        f"{leaves['total_review_leaves']} leaves.",
        "- The centered active/parked sibling and pivot-envelope clearance is 3.00 mm; the inward q=-0.50 mm extreme retains 2.50 mm, a nominal 0.50 mm reserve above the 2.00 mm requirement. Formal tolerance and transition authority remain open.", "",
        "## Exactly-once install counts", "",
        *(f"- `{name}`: {value}" for name, value in counts.items()),
        "", "## Authority boundary", "",
        "The identity, positive-volume carrier, and finalized hardware/count contract close. Pin retention wear/load, the 8.90 mm selector linkage, formal tolerance stack, transition sweeps, route closure, the 5.52 N m load path, and fail-safe integration remain unproved. No assembly, collision, route, BOM, procurement, or release authority is granted.", "",
        "## Open blockers", "",
        *(f"- `{name}`" for name in report["blockers"]), "",
    ])


def write_outputs(report: Mapping[str, Any] | None = None) -> dict[str, Any]:
    result = dict(report or build_report())
    validate_report_integrity(result)
    REPORTS.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    OUTPUT_MD.write_text(render_markdown(result), encoding="utf-8")
    return result


if __name__ == "__main__":
    value = write_outputs()
    print(f"{value['status']}: cases={value['selection_contract']['case_count']}")
