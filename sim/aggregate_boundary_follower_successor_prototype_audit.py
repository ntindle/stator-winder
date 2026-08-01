"""Fail-closed audit for the isolated successor-follower prototype."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SOURCE = ROOT / "cad" / "aggregate_boundary_follower_successor_prototype.py"
STEP = ROOT / "out" / "review" / (
    "aggregate_boundary_follower_successor_prototype.step"
)
MANIFEST = ROOT / "out" / "review" / (
    "aggregate_boundary_follower_successor_prototype_manifest.json"
)
PLACEMENT = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_placement_trade.json"
)
REPORT_JSON = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_successor_prototype_audit.json"
)
REPORT_MD = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_successor_prototype_audit.md"
)

EXPECTED_HASHES = {
    "source": "782456ef56019427d2bdf4fa3be8fa2c4e1684f1dd3be9e6cee7b04422c9677b",
    "step": "6bf20bbca4f166a7c39cee4aec309e8f7765655597a8c8f8e4a335e12a2db183",
    "manifest": "0e8ef6bbd0e59a8025d39abd48bf20acc381688ab7b2f63c29540f6c6fc26edb",
    "placement": "be599cbfed61afdfdaa7fc9c053ee1e20a3ab20cfe723699be1eb5a81e4dbb4c",
}
EXPECTED_PLACEMENT_INTERNAL_SHA = (
    "1800b5f9500f5b0041758991cc8f42f8dc0b62654bec3ce84e402b59dd79dbc3"
)
AUTHORITY_FALSE_KEYS = {
    "assembly_integration_authorized", "wire_route_authorized",
    "collision_authorized", "load_authorized", "dynamics_authorized",
    "buildability_authorized", "procurement_authorized",
    "BOM_change_authorized", "production_authorized", "release_authorized",
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _canonical_hash(value: Mapping[str, Any]) -> str:
    body = deepcopy(dict(value))
    body.pop("report_sha256", None)
    return hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def run_audit() -> dict[str, Any]:
    actual_hashes = {
        "source": _sha256(SOURCE), "step": _sha256(STEP),
        "manifest": _sha256(MANIFEST), "placement": _sha256(PLACEMENT),
    }
    manifest = _load(MANIFEST)
    placement = _load(PLACEMENT)
    stage = manifest["stage"]
    relief = manifest["carrier_floor_relief"]
    authority = manifest["authority"]
    checks = {
        "all_input_file_hashes_match_frozen_values": actual_hashes == EXPECTED_HASHES,
        "placement_internal_hash_matches": (
            placement.get("report_sha256") == EXPECTED_PLACEMENT_INTERNAL_SHA
            and _canonical_hash(placement) == EXPECTED_PLACEMENT_INTERNAL_SHA
        ),
        "manifest_embeds_current_source_and_STEP_hashes": (
            manifest["artifacts"]["source_sha256"] == actual_hashes["source"]
            and manifest["artifacts"]["step_sha256"] == actual_hashes["step"]
            and manifest["artifacts"]["step_exists"] is True
        ),
        "four_re_datumed_stages_present": stage["count"] == 4,
        "XYZ_travel_covers_exact_common_minimum": (
            stage["all_modeled_travel_meets_required"] is True
            and all(m >= r for m, r in zip(
                stage["modeled_XYZ_travel_mm"],
                stage["required_common_XYZ_travel_mm"],
            ))
        ),
        "yaw_and_elevation_ranges_cover_requirement": (
            stage["modeled_yaw_half_range_deg"]
            >= stage["required_yaw_half_range_deg"]
            and stage["modeled_elevation_half_range_deg"]
            >= stage["required_elevation_half_range_deg"]
        ),
        "exact_four_identity_bounds_copied": (
            len(manifest["identities"]) == 4 and all(
                manifest["identities"][key][
                    "exact_target_center_bounds_local_mm"
                ] == placement["successor_trade"]["per_identity"][key][
                    "exact_target_center_bounds_local_mm"
                ]
                for key in ("0", "1", "2", "3")
            )
        ),
        "polished_C1_guide_and_separate_preload_present": (
            manifest["guide"]["count"] == 4
            and manifest["guide"]["join_continuity"]
            == "C1_tangent_continuous_by_construction"
            and manifest["preload"]["leaf_count"] == 4
            and manifest["preload"]["shoe_count"] == 4
            and manifest["preload"]["mechanically_separate_from_guide"] is True
        ),
        "carrier_floor_relief_targets_2mm_around_R3": (
            relief["conservative_envelope_radius_mm"] == 3.0
            and relief["relief_radius_mm"] == 5.0
            and relief["radial_clearance_mm"] >= 2.0
            and relief["selected_carrier_modified"] is False
        ),
        "all_physical_and_release_authority_false": (
            set(authority) == AUTHORITY_FALSE_KEYS | {"isolated_review_only"}
            and authority["isolated_review_only"] is True
            and all(authority[key] is False for key in AUTHORITY_FALSE_KEYS)
        ),
        "known_scope_blockers_retained": len(manifest["blockers"]) >= 6,
    }
    report = {
        "schema": "aggregate_boundary_follower_successor_prototype_audit_v1",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "decision": "ISOLATED_TOPOLOGY_PROTOTYPE_ONLY__NO_ASSEMBLY_OR_PHYSICAL_AUTHORITY",
        "checks": checks,
        "input_hashes": actual_hashes,
        "evidence": {
            "STEP_size_bytes": STEP.stat().st_size,
            "stage": stage,
            "guide": manifest["guide"],
            "preload": manifest["preload"],
            "carrier_floor_relief": relief,
            "identity_count": len(manifest["identities"]),
            "blockers": manifest["blockers"],
        },
        "authority": {key: False for key in sorted(AUTHORITY_FALSE_KEYS)},
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def validate_report(report: Mapping[str, Any]) -> None:
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("successor-prototype audit hash invalid")
    if report.get("status") != "PASS":
        failed = [key for key, value in report.get("checks", {}).items()
                  if value is not True]
        raise ValueError(f"successor-prototype audit failed: {failed}")
    if any(report.get("authority", {}).values()):
        raise ValueError("successor-prototype audit invented authority")


def write_reports() -> dict[str, Any]:
    report = run_audit()
    validate_report(report)
    REPORT_JSON.parent.mkdir(parents=True, exist_ok=True)
    REPORT_JSON.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    checks = [f"- {'PASS' if value else 'FAIL'}: `{key}`"
              for key, value in report["checks"].items()]
    REPORT_MD.write_text("\n".join([
        "# Aggregate-boundary successor prototype audit", "",
        f"Status: **{report['status']}**", "",
        "This is an isolated topology prototype only. Assembly integration, "
        "wire route, collision, load, dynamics, buildability, procurement, "
        "BOM, production, and release authority remain false.", "",
        "## Checks", "", *checks, "", "## Remaining blockers", "",
        *[f"- `{item}`" for item in report["evidence"]["blockers"]], "",
        f"Report SHA-256: `{report['report_sha256']}`", "",
    ]), encoding="utf-8")
    return report


if __name__ == "__main__":
    report = write_reports()
    print(f"{report['status']} {report['report_sha256']}")

