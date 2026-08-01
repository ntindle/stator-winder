"""Bounded aggregate-envelope alternative to exact strand-schedule matching.

The raw controller does not select an exact lateral strand centre.  This study
therefore classifies every physical half-turn against a smooth *growing coil
aggregate* and uses the independent 50-centre slot-capacity scaffold only as a
nonpenetrating support domain.  It deliberately fails closed where the model
would need a real support: before turn zero there is no copper surface, and an
analytic free-space R3 transfer is not a shroud/former.

No CAD, capture, settings, BOM, or production gate is modified.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np
from scipy.spatial import cKDTree


ROOT = Path(__file__).resolve().parent.parent
REPORTS = ROOT / "out" / "reports"
PHASE = REPORTS / "phase_aware_progressive_wire_audit.json"
R3 = REPORTS / "r3_bend_scope_feasibility.json"
SECTOR = REPORTS / "r3_sector_chord_family_study.json"
WIRE = REPORTS / "wirepath_upstream_raw.json"
COIL = REPORTS / "coil_growth.json"
GOAL = ROOT.parent / "GOAL.md"
JSON_OUT = REPORTS / "aggregate_progressive_wire_corridor.json"
MD_OUT = REPORTS / "aggregate_progressive_wire_corridor.md"
SCHEMA = "aggregate-progressive-wire-corridor/v1"
MINIMUM_RADIUS_MM = 3.0


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} is not a JSON object")
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    body = deepcopy(dict(payload))
    body.pop("report_sha256", None)
    return hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def _s_transfer(offset_mm: float) -> dict[str, float | bool]:
    """Two opposite R3 arcs between parallel axial tangents."""

    offset = abs(float(offset_mm))
    if offset > 4.0 * MINIMUM_RADIUS_MM + 1.0e-12:
        return {"constructible": False, "lateral_offset_mm": offset}
    theta = math.acos(max(-1.0, min(
        1.0, 1.0 - offset / (2.0 * MINIMUM_RADIUS_MM))))
    return {
        "constructible": True,
        "lateral_offset_mm": offset,
        "arc_radius_mm": MINIMUM_RADIUS_MM,
        "arc_sweep_deg": math.degrees(theta),
        "required_axial_run_mm": 2.0 * MINIMUM_RADIUS_MM * math.sin(theta),
    }


def analyze() -> dict[str, Any]:
    phase = _load(PHASE)
    r3 = _load(R3)
    sector = _load(SECTOR)
    wire = _load(WIRE)
    coil = _load(COIL)["current_default"]
    records = phase["locus_records"]
    centres = r3["square_row_witness"]["centres"]
    if len(records) != 2400 or len(centres) != 50:
        raise ValueError("aggregate study requires 2400 raw loci and 50 supports")

    wire_d = float(r3["inputs"]["stator"]["wire_finished_diameter_mm"])
    stack = float(r3["inputs"]["stator"]["stack_mm"])
    support_xy = np.asarray([
        (float(row["tooth_x_mm"]), float(row["tooth_half_span_mm"]))
        for row in centres
    ])
    support_min = float(np.min(cKDTree(support_xy).query(
        support_xy, k=2)[0][:, 1]))

    corrections: list[dict[str, Any]] = []
    contact_counts = {
        "LINED_ACTIVE_TOOTH_FIRST_TURN_SUPPORT_REQUIRED": 0,
        "ACTIVE_PRIOR_COPPER_SURFACE_CONTACT_INTENDED": 0,
    }
    sign_counts = {-1: 0, 1: 0}
    for record in records:
        locus = record["locus"]
        turn = int(locus["turn_index"])
        raw_x = float(locus["radial_x_mm"])
        support_x = float(centres[turn]["tooth_x_mm"])
        transfer = _s_transfer(support_x - raw_x)
        corrections.append({
            "pass_index": int(locus["pass_index"]),
            "state_index": int(locus["state_index"]),
            "turn_index": turn,
            "half_turn_index": int(locus["half_turn_index"]),
            "raw_contact_x_mm": raw_x,
            "aggregate_support_x_mm": support_x,
            "contact_class": (
                "LINED_ACTIVE_TOOTH_FIRST_TURN_SUPPORT_REQUIRED"
                if turn == 0 else
                "ACTIVE_PRIOR_COPPER_SURFACE_CONTACT_INTENDED"
            ),
            "r3_capture_transfer": transfer,
        })
        contact_counts[corrections[-1]["contact_class"]] += 1
        sign_counts[int(locus["motion_sign"])] += 1

    worst = max(corrections, key=lambda row: float(
        row["r3_capture_transfer"]["lateral_offset_mm"]))
    max_run = max(float(row["r3_capture_transfer"].get(
        "required_axial_run_mm", math.inf)) for row in corrections)
    all_transfer_math = all(bool(row["r3_capture_transfer"]["constructible"])
                            for row in corrections)
    all_transfer_math &= max_run <= stack + 1.0e-9

    aggregate_model = {
        "kind": "smooth excluded-volume/contact-surface abstraction",
        "growth_parameter": "completed turns / 50 on the active tooth",
        "material_area_per_completed_side_mm2": (
            math.pi * (wire_d / 2.0) ** 2),
        "final_one_side_wire_area_mm2": coil["packing"][
            "one_coil_side_wire_area_mm2"],
        "accessible_slot_area_mm2": coil["packing"][
            "wire_accessible_slot_area_mm2"],
        "final_shared_slot_gross_fill": coil["packing"]["gross_slot_fill"],
        "hard_fill_limit": coil["packing"]["maximum_slot_fill_limit"],
        "surface_rule": (
            "the live wire tube may be tangent to the named lined active "
            "tooth or active prior-copper boundary; its centreline may not "
            "enter that boundary's wire-radius offset interior"
        ),
        "exact_layer_centres_predicted": False,
    }

    gates = {
        "canonical_24_pass_2400_locus_coverage": (
            len({int(r["locus"]["pass_index"]) for r in records}) == 24
            and len(records) == 2400),
        "both_raw_motion_signs_1200_each": sign_counts == {-1: 1200, 1: 1200},
        "all_live_contacts_phase_classified": sum(contact_counts.values()) == 2400,
        "aggregate_slot_capacity": (
            aggregate_model["final_shared_slot_gross_fill"]
            <= aggregate_model["hard_fill_limit"]),
        "support_scaffold_nonpenetrating": support_min + 1.0e-9 >= wire_d,
        "analytic_raw_to_support_transfer_is_C1_R3_and_stack_bounded": (
            all_transfer_math),
        "static_spool_to_tip_machine_path": wire["static"]["ok"] is True,
        "captured_shaft_wrap_path": wire["shaft_wrap"]["ok"] is True,
        "physical_R3_first_turn_support_surface_identified": False,
        "raw_to_support_transfer_clear_of_core_and_prior_aggregate": False,
        "continuous_all_flyer_angle_supported_route": False,
        "all_24_completed_crown_aggregates_noninterpenetrating": False,
    }
    controlling = [name for name, ok in gates.items() if not ok]
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "ADVISORY_NO_GO",
        "decision": "AGGREGATE_CONTACT_SEMANTICS_VALID__PHYSICAL_SUPPORT_UNPROVEN",
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "scope": {
            "proved": [
                "all 24 passes and 2400 physical half-turns receive an explicit intended-contact class",
                "the area/capacity aggregate does not require exact raw-to-packed centre equality",
                "the independent 50-support slot scaffold is nonpenetrating",
                "every raw radial locus can reach its scaffold radius with an analytic C1 R3 two-arc transfer inside 15 mm axial run",
            ],
            "not_proved": [
                "a physical R3 shroud/former for the first turn",
                "core/copper clearance of the analytic transfer itself",
                "continuous support through every flyer angle between half-turn loci",
                "noninterpenetrating final crown aggregates on all 24 teeth",
                "exact passive settling, layer order/neatness, tension, sag, snagging, friction, or enamel abrasion",
            ],
        },
        "aggregate_model": aggregate_model,
        "raw_coverage": {
            "pass_count": 24,
            "locus_count": len(records),
            "motion_sign_counts": {"negative": sign_counts[-1],
                                   "positive": sign_counts[1]},
            "contact_class_counts": contact_counts,
        },
        "support_scaffold": {
            "support_count": len(centres),
            "minimum_same_side_center_distance_mm": support_min,
            "required_center_distance_mm": wire_d,
            "source_status": r3["status"],
            "raw_schedule_equality_required": False,
        },
        "capture_transfer": {
            "family": "two opposite circular arcs between parallel axial tangents",
            "piece_radius_mm": MINIMUM_RADIUS_MM,
            "C1": True,
            "maximum_required_axial_run_mm": max_run,
            "available_stack_run_mm": stack,
            "worst_locus": worst,
            "all_loci_mathematically_constructible": all_transfer_math,
            "physical_support_authority": False,
            "reason": (
                "a free-space analytic centreline is not a guide; before the "
                "first copper exists, a real lined R3 shroud/former must "
                "supply this shape and must be collision-checked"
            ),
        },
        "cross_tooth_bound": {
            "sector_family_status": sector["status"],
            "adjacent_sector_centerline_lower_bound_mm": sector["checks"][
                "adjacent_tooth_analytic_centerline_lower_bound_mm"],
            "same_tooth_clearance_proved": sector["checks"][
                "same_tooth_clearance_proved"],
            "interpretation": (
                "sector ownership prevents one isolated centreline from "
                "entering its neighbor's inset sector, but the studied 50-"
                "curve crown family self-intersects; an aggregate surface "
                "cannot erase that missing physical geometry"
            ),
        },
        "gates": gates,
        "controlling_blockers": controlling,
        "per_locus": corrections,
        "source_hashes": {
            "GOAL.md": _sha256(GOAL),
            "out/reports/phase_aware_progressive_wire_audit.json": _sha256(PHASE),
            "out/reports/r3_bend_scope_feasibility.json": _sha256(R3),
            "out/reports/r3_sector_chord_family_study.json": _sha256(SECTOR),
            "out/reports/wirepath_upstream_raw.json": _sha256(WIRE),
            "out/reports/coil_growth.json": _sha256(COIL),
            "sim/aggregate_progressive_wire_corridor.py": _sha256(Path(__file__)),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def render_markdown(report: Mapping[str, Any]) -> str:
    transfer = report["capture_transfer"]
    scaffold = report["support_scaffold"]
    counts = report["raw_coverage"]["contact_class_counts"]
    lines = [
        "# Aggregate progressive wire-corridor audit", "",
        f"**{report['status']} — {report['decision']}**", "",
        "The aggregate/contact-surface abstraction is valid for capacity and "
        "contact semantics, but it does not create the physical surface that "
        "the first turn needs. No CAD or release integration is authorized.", "",
        "## Proven", "",
        f"- 24 passes / {report['raw_coverage']['locus_count']} raw half-turn loci classified.",
        f"- First-turn lined support loci: {counts['LINED_ACTIVE_TOOTH_FIRST_TURN_SUPPORT_REQUIRED']}.",
        f"- Intended active-prior-copper surface loci: {counts['ACTIVE_PRIOR_COPPER_SURFACE_CONTACT_INTENDED']}.",
        f"- Support scaffold minimum: {scaffold['minimum_same_side_center_distance_mm']:.6f} mm for {scaffold['required_center_distance_mm']:.6f} mm wire.",
        f"- Worst raw-to-support R3 transfer needs {transfer['worst_locus']['r3_capture_transfer']['lateral_offset_mm']:.6f} mm lateral correction and {transfer['maximum_required_axial_run_mm']:.6f} mm axial run inside the {transfer['available_stack_run_mm']:.3f} mm stack.",
        "", "## Why this remains a no-go", "",
        "The two-arc transfer is a mathematical C1/R3 curve, not a physical "
        "guide. At turn zero there is no prior copper; the current nominal "
        "wire path already has a core/shoe witness. A real R3 lined shroud or "
        "former must be identified and then checked continuously against the "
        "core, prior aggregates, and all 24 completed crowns.", "",
        "## Controlling blockers", "",
    ]
    lines.extend(f"- `{name}`" for name in report["controlling_blockers"])
    lines.extend(["", "Exact settling, neatness, tension, sag, snagging, friction, and enamel abrasion remain hardware-only.", ""])
    return "\n".join(lines)


def write_reports(report: Mapping[str, Any] | None = None) -> dict[str, Any]:
    value = analyze() if report is None else dict(report)
    JSON_OUT.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n",
                        encoding="utf-8")
    MD_OUT.write_text(render_markdown(value), encoding="utf-8")
    return value


def main() -> int:
    report = write_reports()
    print(f"aggregate wire corridor: {report['status']}; "
          f"blockers={len(report['controlling_blockers'])}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
