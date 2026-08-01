"""Fail-closed binder for the finalized replacement-carriage CAD evidence.

The audited STEP proves the labeled review tree and zero positive common
volume at the five static selector-state signatures represented by all 36
selector/gate cases.  It does not prove clearances, state transitions, wire
route, loads, selector/retraction linkages, assembly integration, production,
procurement, or release readiness.

This module deliberately does not import the aggregate follower acceptance
module.  Evidence flows from CAD/architecture into acceptance, never back
from acceptance into its own dependencies.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Iterable, Mapping


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
REVIEW = ROOT / "out" / "review"
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_replacement_architecture as architecture
import aggregate_boundary_follower_replacement_carriage as carriage


CAD_SOURCE = CAD / "aggregate_boundary_follower_replacement_carriage.py"
MANIFEST_PATH = REVIEW / (
    "aggregate_boundary_follower_replacement_carriage_manifest.json"
)
STEP_PATH = REVIEW / "aggregate_boundary_follower_replacement_carriage.step"
ARCHITECTURE_REPORT_PATH = REPORTS / (
    "aggregate_boundary_follower_replacement_architecture.json"
)
OUTPUT_JSON = REPORTS / (
    "aggregate_boundary_follower_replacement_cad_audit.json"
)
OUTPUT_MD = REPORTS / (
    "aggregate_boundary_follower_replacement_cad_audit.md"
)

SCHEMA = "aggregate-boundary-follower-replacement-cad-audit/v1"
INSPECTED_CAD_SOURCE_SHA256 = (
    "c3b9fa201149a44c771aeb218f9120411ee93b93fc459861c876f3f28bb85136"
)
INSPECTED_MANIFEST_SHA256 = (
    "73b079b64ca3d7db2b6a23e63dfbbe22d1c176c300aa9ba6781537b162c76ae3"
)
INSPECTED_STEP_SHA256 = (
    "3c1a8299ade7bb2487a528b0b39f03e00cfd0eeb702734c6f7a2d898bcb55468"
)
INSPECTED_ARCHITECTURE_REPORT_SHA256 = (
    "65e20394320ebcca6f500667e0eeca49849c2a8a92ccaf735fdcb149c973d938"
).lower()
INSPECTED_STEP_LEAF_COUNT = 73
MANUFACTURED_LEAF_COUNT = 69
BLOCKER_ONLY_ENVELOPE_COUNT = 4

EXPECTED_M4_AXES_LOCAL_XY_MM = (
    (29.0, -24.5),
    (35.0, -17.5),
    (29.0, 24.5),
    (35.0, 17.5),
)
EXPECTED_GEOMETRY_SIGNATURES = {
    "all_parked_retracted",
    "engaged_selected_0",
    "engaged_selected_1",
    "engaged_selected_2",
    "engaged_selected_3",
}

AUTHORITY_KEYS = (
    "clearance_authorized",
    "wire_route_authorized",
    "transition_collision_authorized",
    "load_authorized",
    "positive_linkage_authorized",
    "assembly_integration_authorized",
    "production_authorized",
    "procurement_authorized",
    "BOM_change_authorized",
    "release_authorized",
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(value: Mapping[str, Any]) -> str:
    body = deepcopy(dict(value))
    body.pop("report_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object at {path}")
    return value


def _artifact_evidence(
    path: Path, expected_sha256: str,
) -> dict[str, Any]:
    exists = path.is_file()
    observed = _sha256(path) if exists else None
    return {
        "path": str(path.relative_to(ROOT)).replace("\\", "/"),
        "exists": exists,
        "byte_count": path.stat().st_size if exists else None,
        "sha256": observed,
        "expected_sha256": expected_sha256,
        "matches_inspected_sha256": exists and observed == expected_sha256,
    }


def _source_hardware_witness() -> dict[str, Any]:
    mount = carriage.primary_tower_m4_hardware()
    mount_labels = [str(part.label) for part in mount]

    occurrence_labels: list[str] = []
    occurrence_leaf_counts: list[int] = []
    for identity in carriage.OCCURRENCE_IDENTITIES:
        occurrence = carriage.moving_occurrence(identity)
        labels = [str(child.label) for child in occurrence.children]
        occurrence_labels.extend(labels)
        occurrence_leaf_counts.append(len(labels))

    return {
        "carrier_count": int(carriage.geometry_contract()["carrier"]["count"]),
        "occurrence_count": len(carriage.OCCURRENCE_IDENTITIES),
        "occurrence_leaf_counts": occurrence_leaf_counts,
        "primary_mount_leaf_count": len(mount_labels),
        "primary_mount_labels": mount_labels,
        "NBK_M4_screw_count": sum(
            "primary_tower_NBK_SSHS_M4x10_SD_ALK" in label
            for label in mount_labels
        ),
        "M4_washer_count": sum(
            "washer" in label.lower() for label in mount_labels
        ),
        "M4_insert_count": sum(
            "primary_tower_M4_short_heat_insert" in label
            for label in mount_labels
        ),
        "outer_SCCG5_10_pin_count": sum(
            "outer_pivot_MISUMI_SCCG5-10" in label
            for label in occurrence_labels
        ),
        "outer_NETWS4_ring_count": sum(
            "outer_pivot_MISUMI_NETWS4" in label
            for label in occurrence_labels
        ),
        "outer_DIN988_shim_count": sum(
            "outer_pivot_DIN988" in label for label in occurrence_labels
        ),
        "all_leaf_labels_unique": (
            len(mount_labels) + len(occurrence_labels)
            == len(set(mount_labels + occurrence_labels))
        ),
    }


def analyze() -> dict[str, Any]:
    manifest = _load_json(MANIFEST_PATH)
    architecture_report = _load_json(ARCHITECTURE_REPORT_PATH)
    architecture.validate_report_integrity(architecture_report)

    artifacts = {
        "cad_source": _artifact_evidence(
            CAD_SOURCE, INSPECTED_CAD_SOURCE_SHA256,
        ),
        "manifest": _artifact_evidence(
            MANIFEST_PATH, INSPECTED_MANIFEST_SHA256,
        ),
        "step": _artifact_evidence(STEP_PATH, INSPECTED_STEP_SHA256),
        "architecture_report": _artifact_evidence(
            ARCHITECTURE_REPORT_PATH,
            INSPECTED_ARCHITECTURE_REPORT_SHA256,
        ),
    }

    exact = manifest["exact_counts"]
    manifest_pair = manifest["manufactured_leaf_pair_audit"]
    architecture_counts = architecture_report["review_leaf_counts"]
    architecture_mount = architecture_report["primary_mount_hardware"]
    architecture_occurrence = architecture_report["occurrence_leaf_contract"]
    source_hardware = _source_hardware_witness()

    manifest_axes = tuple(
        (float(row[0]), float(row[1]))
        for row in manifest["carrier"][
            "diagonal_primary_M4_local_locations_mm"
        ]
    )
    architecture_axes = tuple(
        tuple(map(float, row))
        for row in architecture_report["shared_adapter"][
            "M4_axes_local_xy_mm"
        ]
    )
    signatures = {
        state["geometry_signature"] for state in manifest_pair["states"]
    }
    state_scopes_zero = all(
        state[scope]["positive_overlap_count"] == 0
        and float(state[scope]["positive_common_volume_mm3"]) == 0.0
        and state[scope]["status"] == "PASS_ZERO_POSITIVE"
        for state in manifest_pair["states"]
        for scope in (
            "follower_carrier_scope", "complete_installed_scope",
        )
    )

    step_binding = {
        "leaf_count": (
            INSPECTED_STEP_LEAF_COUNT
            if artifacts["step"]["matches_inspected_sha256"] else None
        ),
        "leaf_count_method": (
            "ROOT_OCC_INSPECTION_BOUND_BY_SHA256"
            if artifacts["step"]["matches_inspected_sha256"]
            else "UNBOUND_HASH_DRIFT"
        ),
        "inspection_warning_count": (
            0 if artifacts["step"]["matches_inspected_sha256"] else None
        ),
        "manifest_embedded_step_sha256": manifest["artifacts"][
            "step_sha256"
        ],
        "manifest_embedded_step_size_bytes": manifest["artifacts"][
            "step_size_bytes"
        ],
    }

    proof_gates = {
        "all_four_artifacts_match_inspected_hashes": all(
            row["matches_inspected_sha256"] for row in artifacts.values()
        ),
        "manifest_embedded_STEP_binding_matches_file": (
            manifest["artifacts"]["step_exists"] is True
            and manifest["artifacts"]["step_sha256"]
            == artifacts["step"]["sha256"]
            and manifest["artifacts"]["step_size_bytes"]
            == artifacts["step"]["byte_count"]
        ),
        "STEP_has_73_SHA_bound_inspected_leaves": (
            step_binding["leaf_count"] == 73
            and step_binding["inspection_warning_count"] == 0
        ),
        "review_tree_is_69_manufactured_plus_4_blocker_only": (
            exact["carrier_leaf_solids"] == 1
            and exact["moving_leaf_solids"] == 60
            and exact["primary_M4_leaf_solids"] == 8
            and exact["coarse_blocker_leaf_solids"] == 4
            and exact["total_leaf_solids"] == 73
            and architecture_counts["manufactured_leaves"] == 69
            and architecture_counts["coarse_linkage_blocker_envelopes"] == 4
            and manifest_pair[
                "blocker_envelopes_excluded_as_non_manufactured"
            ] == 4
        ),
        "one_carrier_and_four_15_leaf_occurrences": (
            source_hardware["carrier_count"] == 1
            and source_hardware["occurrence_count"] == 4
            and source_hardware["occurrence_leaf_counts"] == [15, 15, 15, 15]
            and exact["moving_occurrence_count"] == 4
        ),
        "four_diagonal_NBK_M4_zero_washers_four_inserts": (
            manifest_axes == EXPECTED_M4_AXES_LOCAL_XY_MM
            and architecture_axes == EXPECTED_M4_AXES_LOCAL_XY_MM
            and source_hardware["NBK_M4_screw_count"] == 4
            and source_hardware["M4_washer_count"] == 0
            and source_hardware["M4_insert_count"] == 4
            and architecture_mount["screw_count"] == 4
            and architecture_mount["washer_count"] == 0
            and architecture_mount["insert_count"] == 4
        ),
        "four_SCCG5_10_and_eight_NETWS4_outer_stack_parts": (
            source_hardware["outer_SCCG5_10_pin_count"] == 4
            and source_hardware["outer_NETWS4_ring_count"] == 8
            and source_hardware["outer_DIN988_shim_count"] == 8
            and architecture_occurrence["outer_pivot"][
                "pin_sku"
            ] == "MISUMI SCCG5-10"
            and architecture_occurrence["outer_pivot"][
                "NETWS4_ring_count_per_occurrence"
            ] == 2
        ),
        "all_36_states_reduce_to_5_exact_signatures": (
            manifest_pair["state_count"] == 36
            and len(manifest_pair["states"]) == 36
            and manifest_pair["unique_geometry_signature_count"] == 5
            and signatures == EXPECTED_GEOMETRY_SIGNATURES
            and architecture_report["selection_contract"]["case_count"] == 36
        ),
        "all_follower_carrier_and_complete_scopes_zero_positive": (
            manifest_pair["all_follower_carrier_states_zero_positive"] is True
            and manifest_pair["all_complete_installed_states_zero_positive"] is True
            and manifest_pair["follower_carrier_failure_state_count"] == 0
            and manifest_pair["complete_installed_failure_state_count"] == 0
            and state_scopes_zero
        ),
        "source_hardware_leaf_labels_are_unique": source_hardware[
            "all_leaf_labels_unique"
        ],
        "architecture_report_is_fail_closed": (
            architecture_report["status"]
            == "DESIGN_CONTRACT_ONLY_FAIL_CLOSED"
            and not any(architecture_report["physical_gates"].values())
        ),
    }

    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "FAIL",
        "decision": (
            "FINAL_STATIC_CAD_AND_ZERO_POSITIVE_VOLUME_EVIDENCE_BOUND__"
            "PHYSICAL_MECHANISM_AUTHORITY_OPEN"
        ),
        "static_CAD_geometry_proven": all(proof_gates.values()),
        "mechanism_complete": False,
        "proof_gates": proof_gates,
        "artifact_binding": artifacts,
        "step_binding": step_binding,
        "leaf_accounting": {
            "STEP_review_leaf_count": INSPECTED_STEP_LEAF_COUNT,
            "manufactured_leaf_count": MANUFACTURED_LEAF_COUNT,
            "blocker_only_envelope_count": BLOCKER_ONLY_ENVELOPE_COUNT,
            "carrier_leaf_count": exact["carrier_leaf_solids"],
            "moving_occurrence_count": exact["moving_occurrence_count"],
            "moving_leaf_count": exact["moving_leaf_solids"],
            "primary_mount_leaf_count": exact["primary_M4_leaf_solids"],
        },
        "hardware_witness": {
            "source": source_hardware,
            "diagonal_M4_axes_local_XY_mm": [
                list(row) for row in manifest_axes
            ],
            "M4_screw_sku": manifest["fasteners"][
                "primary_M4_screw_sku"
            ],
            "outer_pin_sku": manifest["occurrences"][
                "outer_pivot_catalog_stack"
            ]["pin_sku"],
            "outer_ring_sku": manifest["occurrences"][
                "outer_pivot_catalog_stack"
            ]["included_ring_sku"],
        },
        "state_pair_audit": {
            "schema": manifest_pair["schema"],
            "state_count": manifest_pair["state_count"],
            "engaged_state_count": manifest_pair["engaged_state_count"],
            "all_parked_state_count": manifest_pair["all_parked_state_count"],
            "unique_geometry_signature_count": manifest_pair[
                "unique_geometry_signature_count"
            ],
            "geometry_signatures": sorted(signatures),
            "follower_carrier_failure_state_count": manifest_pair[
                "follower_carrier_failure_state_count"
            ],
            "complete_installed_failure_state_count": manifest_pair[
                "complete_installed_failure_state_count"
            ],
            "all_scopes_zero_positive": state_scopes_zero,
            "clearance_authority": False,
        },
        "authority": {name: False for name in AUTHORITY_KEYS},
        "open_blockers": [
            "UNQUALIFIED_nominal_clearance_and_tolerance_stack",
            "SEPARATE_sampled_transition_sweep_is_not_physical_authority",
            "UNPROVEN_continuous_wire_route_and_2400_locus_closure",
            "UNQUALIFIED_static_dynamic_and_fatigue_loads",
            "UNMODELED_positive_volume_8p90mm_coarse_selection_linkage",
            "UNMODELED_positive_M0_retraction_linkage_and_interlock",
            "UNINTEGRATED_replacement_carriage_in_machine_assembly",
            "UNRELEASED_procurement_BOM_production_and_release",
        ],
        "architecture_report_internal_sha256": architecture_report[
            "report_sha256"
        ],
    }
    report["report_sha256"] = _canonical_hash(report)
    validate_report_integrity(report)
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported replacement CAD audit schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("replacement CAD audit report hash mismatch")
    if report.get("status") != "FAIL":
        raise ValueError("replacement CAD audit must remain fail closed")
    if report.get("static_CAD_geometry_proven") is not True:
        raise ValueError("replacement static CAD evidence is not proven")
    if report.get("mechanism_complete") is not False:
        raise ValueError("replacement CAD audit promoted incomplete mechanism")
    if not all(report.get("proof_gates", {}).values()):
        raise ValueError("replacement CAD proof gate failure")
    authority = report.get("authority", {})
    if set(authority) != set(AUTHORITY_KEYS) or any(authority.values()):
        raise ValueError("replacement CAD audit invented physical authority")
    for evidence in report.get("artifact_binding", {}).values():
        path = ROOT / evidence["path"]
        if (
            not path.is_file()
            or _sha256(path) != evidence["sha256"]
            or evidence["sha256"] != evidence["expected_sha256"]
            or path.stat().st_size != evidence["byte_count"]
        ):
            raise ValueError(f"stale replacement CAD artifact {path}")


def render_markdown(report: Mapping[str, Any]) -> str:
    leaves = report["leaf_accounting"]
    hardware = report["hardware_witness"]["source"]
    pairs = report["state_pair_audit"]
    artifacts = report["artifact_binding"]
    lines = [
        "# Aggregate-boundary follower replacement CAD audit", "",
        f"**{report['status']} — {report['decision']}**", "",
        "The finalized static CAD evidence is bound and passes every encoded "
        "geometry/count gate. The physical mechanism remains incomplete and "
        "fail-closed.", "",
        "## Bound static geometry", "",
        f"- STEP review tree: {leaves['STEP_review_leaf_count']} leaves = "
        f"{leaves['manufactured_leaf_count']} manufactured + "
        f"{leaves['blocker_only_envelope_count']} blocker-only envelopes.",
        f"- Ownership: {leaves['carrier_leaf_count']} shared carrier and "
        f"{leaves['moving_occurrence_count']} moving occurrences "
        f"({leaves['moving_leaf_count']} leaves).",
        f"- Primary mount: {hardware['NBK_M4_screw_count']} diagonal NBK M4 "
        f"screws, {hardware['M4_washer_count']} washers, and "
        f"{hardware['M4_insert_count']} inserts.",
        f"- Outer pivots: {hardware['outer_SCCG5_10_pin_count']} SCCG5-10 "
        f"pins, {hardware['outer_NETWS4_ring_count']} NETWS4 rings, and "
        f"{hardware['outer_DIN988_shim_count']} DIN 988 shims.",
        f"- Exact pair audit: {pairs['state_count']} selector/gate states, "
        f"{pairs['unique_geometry_signature_count']} geometry signatures, "
        "zero positive common volume in both follower/carrier and complete "
        "installed scopes.", "",
        "## SHA-256 binding", "",
    ]
    lines.extend(
        f"- `{name}`: `{value['sha256']}`"
        for name, value in artifacts.items()
    )
    lines.extend([
        "", "## Authority boundary", "",
        "Clearance, continuous route, state transitions, loads, positive "
        "selector/retraction linkages, integration, production, procurement, "
        "BOM change, and release authority remain false.", "",
        "## Open blockers", "",
    ])
    lines.extend(f"- `{blocker}`" for blocker in report["open_blockers"])
    lines.extend([
        "", f"Report SHA-256: `{report['report_sha256']}`", "",
    ])
    return "\n".join(lines)


def write_outputs(
    report: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
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
    pairs = report["state_pair_audit"]
    print(
        "replacement CAD audit: "
        f"static={report['static_CAD_geometry_proven']}; "
        f"states={pairs['state_count']}; "
        f"signatures={pairs['unique_geometry_signature_count']}; "
        f"authority={any(report['authority'].values())}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
