"""Exact transition trade for moving the parked follower bays to |Y|=10.45.

This is an isolated redesign evaluation.  It calls the authoritative
replacement transition sweep unchanged except for the proposed parked datum
and the subdivision count required to keep every independent translation step
at or below 0.50 mm.  It does not modify carriage CAD, assembly, BOM, or a
release artifact.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping

import aggregate_boundary_follower_replacement_transition_sweep as sweep


ROOT = Path(__file__).resolve().parents[1]
REPORTS = ROOT / "out" / "reports"
OUTPUT_JSON = REPORTS / (
    "aggregate_boundary_follower_replacement_parked_10p45_trade.json"
)
OUTPUT_MD = REPORTS / (
    "aggregate_boundary_follower_replacement_parked_10p45_trade.md"
)
SCHEMA = "aggregate-boundary-follower-replacement-parked-10p45-trade/v1"

CANDIDATE_PARKED_BASE_ABS_Y_MM = 10.45
CANDIDATE_COARSE_STROKE_MM = (
    CANDIDATE_PARKED_BASE_ABS_Y_MM - sweep.ACTIVE_BASE_ABS_Y_MM
)
CANDIDATE_COARSE_SUBDIVISIONS = 17
REQUIRED_NOMINAL_RESERVE_MM = 0.50


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(value: Mapping[str, Any]) -> str:
    body = deepcopy(dict(value))
    body.pop("report_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def analyze() -> dict[str, Any]:
    original_parked = sweep.PARKED_BASE_ABS_Y_MM
    original_subdivisions = sweep.COARSE_SUBDIVISIONS
    try:
        sweep.PARKED_BASE_ABS_Y_MM = CANDIDATE_PARKED_BASE_ABS_Y_MM
        sweep.COARSE_SUBDIVISIONS = CANDIDATE_COARSE_SUBDIVISIONS
        report = sweep.analyze()
    finally:
        sweep.PARKED_BASE_ABS_Y_MM = original_parked
        sweep.COARSE_SUBDIVISIONS = original_subdivisions

    report.pop("report_sha256", None)
    report["schema"] = SCHEMA
    report["trade_candidate"] = {
        "mode": "ISOLATED_DATUM_OVERRIDE_NOT_CAD_INTEGRATED",
        "baseline_parked_base_abs_y_mm": original_parked,
        "candidate_parked_base_abs_y_mm": CANDIDATE_PARKED_BASE_ABS_Y_MM,
        "active_base_abs_y_mm": sweep.ACTIVE_BASE_ABS_Y_MM,
        "candidate_coarse_stroke_mm": CANDIDATE_COARSE_STROKE_MM,
        "coarse_subdivisions": CANDIDATE_COARSE_SUBDIVISIONS,
        "maximum_coarse_sample_step_mm": (
            CANDIDATE_COARSE_STROKE_MM
            / CANDIDATE_COARSE_SUBDIVISIONS
        ),
        "carrier_CAD_modified": False,
        "assembly_modified": False,
        "release_modified": False,
    }
    clearance = report["clearance_audit"]
    collision = report["collision_audit"]
    restores_full_2mm = (
        collision["positive_failure_count"] == 0
        and clearance["passes_2p00mm_gate"] is True
    )
    nominal_reserve_mm = (
        float(clearance["minimum_sampled_exact_clearance_mm"])
        - float(clearance["required_minimum_mm"])
    )
    meets_selected_reserve = (
        restores_full_2mm
        and nominal_reserve_mm >= REQUIRED_NOMINAL_RESERVE_MM - 1.0e-7
    )
    report["trade_result"] = {
        "all_sampled_positive_common_volumes_zero": collision[
            "all_sampled_positive_common_volumes_zero"
        ],
        "minimum_sampled_exact_clearance_mm": clearance[
            "minimum_sampled_exact_clearance_mm"
        ],
        "clearance_violation_count": clearance["violation_count"],
        "restores_full_2p00mm_gate": restores_full_2mm,
        "nominal_reserve_above_2p00mm_mm": nominal_reserve_mm,
        "required_selected_nominal_reserve_mm": REQUIRED_NOMINAL_RESERVE_MM,
        "meets_selected_nominal_reserve": meets_selected_reserve,
        "selected_for_redesign": meets_selected_reserve,
        "recommendation": (
            "CANDIDATE_MEETS_2P00MM_AND_0P50MM_NOMINAL_RESERVE"
            if meets_selected_reserve else (
            "REJECT_10P45_ZERO_TOLERANCE_RESERVE__SELECT_10P95"
            if restores_full_2mm else
            "REJECT_10P45_DATUM_ALONE_DOES_NOT_RESTORE_2P00MM_GATE"
            )
        ),
    }
    report["status"] = (
        "PASS_SAMPLED_TRADE_GEOMETRY_ONLY"
        if restores_full_2mm else "FAIL_CLOSED"
    )
    report["decision"] = report["trade_result"]["recommendation"]
    report["source_hashes"][
        "sim/aggregate_boundary_follower_replacement_parked_10p45_trade.py"
    ] = _sha256(Path(__file__).resolve())
    report["report_sha256"] = _canonical_hash(report)
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported 10.45 parked trade schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("10.45 parked trade hash mismatch")
    for relative, expected in report.get("source_hashes", {}).items():
        path = ROOT / Path(str(relative).replace("\\", "/"))
        if not path.is_file() or _sha256(path) != expected:
            raise ValueError(f"stale 10.45 parked trade source {relative}")
    if not all(report["sampling"]["gates"].values()):
        raise ValueError("10.45 parked trade sampling contract failed")
    if any(report["authority"].values()):
        raise ValueError("10.45 parked trade cannot grant physical authority")
    result = report["trade_result"]
    expected = (
        "PASS_SAMPLED_TRADE_GEOMETRY_ONLY"
        if result["restores_full_2p00mm_gate"] else "FAIL_CLOSED"
    )
    if report.get("status") != expected:
        raise ValueError("10.45 parked trade status is not fail-closed")


def render_markdown(report: Mapping[str, Any]) -> str:
    candidate = report["trade_candidate"]
    result = report["trade_result"]
    collision = report["collision_audit"]
    clearance = report["clearance_audit"]
    leaf = clearance["minimum_leaf_pair_witness"] or {}
    return "\n".join([
        "# Replacement follower parked-|Y|=10.45 trade",
        "",
        f"**{report['status']} — {report['decision']}**",
        "",
        f"- Candidate parked datum: |Y|={candidate['candidate_parked_base_abs_y_mm']:.2f} mm.",
        f"- Active datum: |Y|={candidate['active_base_abs_y_mm']:.2f} mm.",
        f"- Coarse stroke: {candidate['candidate_coarse_stroke_mm']:.2f} mm.",
        f"- Exact sweep poses: {report['sampling']['total_pose_count']} ({report['sampling']['sample_count_per_identity']} per identity).",
        f"- Maximum independent translation step: {report['sampling']['maximum_independent_translation_step_mm']:.6f} mm.",
        f"- Positive-volume failures: {collision['positive_failure_count']}.",
        f"- Minimum exact non-contact clearance: {clearance['minimum_sampled_exact_clearance_mm']:.9f} mm (required 2.00 mm).",
        f"- Clearance violations: {clearance['violation_count']}.",
        f"- Nominal reserve above 2.00 mm: {result['nominal_reserve_above_2p00mm_mm']:.6f} mm (selected design requires {result['required_selected_nominal_reserve_mm']:.2f} mm).",
        f"- Closest selected leaf: `{leaf.get('selected_label', 'unresolved')}`.",
        f"- Closest static leaf: `{leaf.get('target_label', 'unresolved')}`.",
        "",
        f"The 10.45 mm datum {'does' if result['restores_full_2p00mm_gate'] else 'does not'} restore the bare sampled 2.00 mm gate, but it {'meets' if result['meets_selected_nominal_reserve'] else 'does not meet'} the selected 0.50 mm nominal tolerance reserve.",
        "",
        "This is a datum-only trade. The carrier CAD, positive selector, retraction linkage/interlock, assembly, BOM, and release remain unchanged and unauthorized.",
        "",
    ])


def write_reports(report: Mapping[str, Any]) -> None:
    validate_report_integrity(report)
    REPORTS.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    OUTPUT_MD.write_text(render_markdown(report), encoding="utf-8")


def main() -> int:
    report = analyze()
    write_reports(report)
    print(json.dumps({
        "status": report["status"],
        "pose_count": report["sampling"]["total_pose_count"],
        "collision_failures": report["collision_audit"][
            "positive_failure_count"
        ],
        "minimum_exact_clearance_mm": report["clearance_audit"][
            "minimum_sampled_exact_clearance_mm"
        ],
        "clearance_violations": report["clearance_audit"][
            "violation_count"
        ],
        "restores_full_2p00mm_gate": report["trade_result"][
            "restores_full_2p00mm_gate"
        ],
        "report_sha256": report["report_sha256"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
