"""Advisory scope audit for GOAL.md wire-path validation.

This report deliberately does not change or replace any production gate.  It
separates the literal GOAL.md DoD #3 requirements from stricter packed-layer
and tooling-selection policies that accumulated in the diagnostic studies.
The distinction matters because GOAL.md explicitly reserves layering neatness
and tension dynamics for hardware, while still requiring an end-to-end R3
wire path and freedom from unintended contact.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parent.parent
GOAL = ROOT.parent / "GOAL.md"
REPORTS = ROOT / "out" / "reports"
WIRE = REPORTS / "wirepath_upstream_raw.json"
ELASTIC = REPORTS / "elastic_wire_contact_study.json"
TOOLING = REPORTS / "winding_tooling_authority.json"
COIL = REPORTS / "coil_growth.json"
OUTPUT_JSON = REPORTS / "wire_gate_scope_audit.json"
OUTPUT_MD = REPORTS / "wire_gate_scope_audit.md"
SCHEMA = "wire-gate-scope-audit/v1"


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


def analyze() -> dict[str, Any]:
    goal = GOAL.read_text(encoding="utf-8")
    wire = _load(WIRE)
    elastic = _load(ELASTIC)
    tooling = _load(TOOLING)
    coil = _load(COIL)

    raw = elastic["raw_motion_replay"]
    repacking = raw["raw_repacking_demand"]
    contact = elastic["elastic_contact_reanalysis"]
    default = coil["current_default"]
    guide_radii = {
        name: float(row["radius"])
        for name, row in wire["guides"].items()
    }
    minimum_guide = min(guide_radii.values())
    moving_fail = list(wire.get("fail", []))

    required_proven = [
        {
            "id": "canonical_raw_motion_binding",
            "passed": (
                wire.get("evidence", {}).get("capture_schema") == 4
                and wire.get("evidence", {}).get("controller_mode") == "upstream"
                and elastic.get("raw_capture", {}).get("status") == "PASS"
            ),
            "evidence": "wire path and deposition crossings are bound to the unmodified upstream capture",
        },
        {
            "id": "static_machine_path_clear",
            "passed": wire["static"]["ok"] is True,
            "evidence": {
                "minimum_clearance_mm": wire["static"]["worst_clearance"],
                "nearest_part": wire["static"]["nearest_part"],
            },
        },
        {
            "id": "shaft_wrap_path_clear",
            "passed": wire["shaft_wrap"]["ok"] is True,
            "evidence": {
                "minimum_clearance_mm": wire["shaft_wrap"]["worst_clearance"],
                "nearest_part": wire["shaft_wrap"]["nearest_part_at_worst"],
            },
        },
        {
            "id": "machine_guide_radii_R3",
            "passed": minimum_guide >= 3.0,
            "evidence": {
                "minimum_radius_mm": minimum_guide,
                "radii_mm": guide_radii,
            },
        },
        {
            "id": "raw_M0_contact_loci_inside_accessible_span",
            "passed": raw["states_inside_winding_span"] == raw["state_count"],
            "evidence": {
                "states_inside": raw["states_inside_winding_span"],
                "state_count": raw["state_count"],
                "span_mm": raw["winding_span_mm"],
                "minimum_turns_per_pass": raw[
                    "minimum_actual_logical_winding_travel_turns"
                ],
                "maximum_turns_per_pass": raw[
                    "maximum_actual_logical_winding_travel_turns"
                ],
            },
        },
        {
            "id": "slot_capacity_and_final_bundle_envelope",
            "passed": default["status"] == "PASS",
            "evidence": {
                "gross_slot_fill": default["packing"]["gross_slot_fill"],
                "hard_fill_limit": default["packing"][
                    "maximum_slot_fill_limit"
                ],
                "turns": default["spec"]["turns_per_tooth"],
                "hard_limit_turn_capacity": default["packing"][
                    "max_turns_at_maximum_fill"
                ],
                "slot_opening_margin_mm": default["slot_opening"]["margin_mm"],
            },
        },
    ]

    required_unproven = [
        {
            "id": "moving_path_contact_semantics",
            "status": "UNPROVEN",
            "why_required": "GOAL.md forbids contact with unintended edges",
            "current_evidence": {
                "wirepath_status": wire["status"],
                "reported_failures": moving_fail,
                "worst_clearance_mm": wire["moving"]["worst_clearance"],
                "nearest_part": wire["moving"]["nearest_part_at_worst"],
                "aggregate_contact_part": "stator_final_wound_envelope",
            },
            "why_not_waivable": (
                "The aggregate spindle solid combines the stator and every final coil envelope. "
                "The unsigned-distance ranking reports only the globally nearest part/link, so it "
                "cannot distinguish allowed active-tooth/parent-wire contact from contact with "
                "steel, another tooth's coil, or a second hidden colliding part."
            ),
        },
        {
            "id": "R3_workpiece_turning_path",
            "status": "UNPROVEN",
            "why_required": "the literal end-to-end DoD #3 says no bend radius under 3 mm",
            "current_evidence": {
                "final_envelope_minimum_centerline_curvature_mm": default[
                    "bundle"
                ]["end_turn_envelope"][
                    "minimum_deposited_wire_center_curvature_radius_mm"
                ],
                "envelope_manufacturable_former": default["bundle"][
                    "end_turn_envelope"
                ]["manufacturable_former"],
                "exact_packed_candidate_contact_detour_radius_mm": contact[
                    "cases"
                ][0]["analytic_local_bend_radius_mm"],
                "exact_packed_candidate_R3_pass_count": contact[
                    "elastic_curvature_pass_count"
                ],
            },
            "why_not_waivable": (
                "Layer ordering may remain empirical, but static centreline curvature is an "
                "explicit geometric requirement. The guardrail does not exempt deposited or "
                "workpiece-contact curvature."
            ),
        },
    ]

    candidate_specific = [
        {
            "id": "stored_exact_packing_route",
            "result": "REJECT_THIS_CANDIDATE_ONLY",
            "evidence": {
                "geometric_contact_repairs": contact[
                    "contact_geometric_pass_count"
                ],
                "R3_repairs": contact["elastic_curvature_pass_count"],
                "contact_detour_radius_mm": contact["cases"][0][
                    "analytic_local_bend_radius_mm"
                ],
            },
            "interpretation": (
                "The R0.22352 parent-wire detour invalidates the stored exact packing route. "
                "It does not prove every packing compatible with the raw ease-out-sine M0 law "
                "must use that detour."
            ),
        },
    ]

    extra_policy = [
        {
            "id": "match_one_preselected_hash_bound_packing_schedule",
            "goal_required": False,
            "current": {
                "matching_states": raw[
                    "states_matching_packed_route_schedule"
                ],
                "total_states": raw["state_count"],
            },
            "reason": (
                "GOAL requires the raw M0 traverse to remain geometrically valid; it does not "
                "require exact strand centres or a predetermined layer order, and explicitly "
                "reserves layering neatness for hardware."
            ),
        },
        {
            "id": "global_noncrossing_repacking_branch_certificate",
            "goal_required": False,
            "current": repacking["global_noncrossing_repacking_certificate"],
            "reason": (
                "This would predict passive layer selection and neatness. Capacity, access, "
                "an R3 turning corridor, and absence of unintended external contact remain "
                "required; deterministic selection of one strand ordering does not."
            ),
        },
        {
            "id": "complete_lot_specific_material_error_budget",
            "goal_required_for_DOD3": False,
            "current": elastic["release_flags"]["complete_material_error_budget"],
            "reason": (
                "Useful procurement/coupon policy, but not the stated wire-path geometry gate; "
                "the GOAL explicitly leaves tension dynamics and enamel abrasion to hardware."
            ),
        },
        {
            "id": "exactly_one_architecture_study_self_declares_production_authority",
            "goal_required": False,
            "current": {
                "selected": tooling.get("selected_production_candidate"),
                "blockers": tooling.get("release_blockers"),
            },
            "reason": (
                "GOAL.md does not require a shoe/former/selector or a meta-selection report. "
                "Tooling is required only if direct geometry cannot satisfy the actual wire-path "
                "criteria."
            ),
        },
    ]

    acceptance_model = {
        "name": "phase-aware active-contact corridor",
        "required_checks": [
            {
                "id": "A_free_path",
                "rule": (
                    "Replay the canonical raw M0/M1/M2 poses and also sweep each supported lay "
                    "depth through 360 degrees in both directions. Model the true wire-radius "
                    "tube from spool through the tip to the first workpiece contact. Require "
                    "positive clearance from every non-contact solid."
                ),
            },
            {
                "id": "B_contact_classes",
                "rule": (
                    "Export per-tooth steel, liner, and coil-envelope meshes. At each pass identify "
                    "the active tooth and the coils already present. Permit contact only with "
                    "named guides, the active tooth's lined capture region, declared parent copper, "
                    "and the shaft sleeve during shaft wrap. Check every other part separately so "
                    "one allowed contact cannot mask another collision."
                ),
            },
            {
                "id": "C_R3_corridor",
                "rule": (
                    "Construct at least one continuous, tangent-continuous centreline corridor from "
                    "the tip through the active-tooth turn and back to the opposite side with local "
                    "radius >=3.0 mm, for every raw M0 locus and both M2 directions. Intended "
                    "contact does not waive this curvature rule."
                ),
            },
            {
                "id": "D_capacity_not_neatness",
                "rule": (
                    "For every pass require all raw half-turn loci to remain inside the lined "
                    "accessible span, require slot opening and total finished-wire area to pass the "
                    "declared hard fill limit, and collision-check the conservative final bundle "
                    "envelopes. Do not require the raw loci to equal one preselected set of strand "
                    "centres."
                ),
            },
            {
                "id": "E_honest_limit",
                "rule": (
                    "Report passive lateral settling, exact layer order/neatness, tension/sag, "
                    "snagging, friction, and enamel abrasion as hardware-only. Do not claim that "
                    "the envelope/corridor model predicts them."
                ),
            },
        ],
        "passing_claim": (
            "Geometric and kinematic feasibility only: the machine provides an unobstructed R3 "
            "route and enough lined volume for the raw winding law. It does not predict the exact "
            "strand packing selected by real tension and contact."
        ),
    }

    result = {
        "schema": SCHEMA,
        "advisory_only": True,
        "production_gate_modified": False,
        "conclusion": "DOD3_NOT_YET_PROVEN_FOR_TWO_REAL_REASONS",
        "conclusion_detail": (
            "The current DoD #3 FAIL must remain, but its decisive reasons should be the unresolved "
            "phase-aware intended-contact classification and the missing >=3 mm workpiece-turning "
            "corridor. Exact packed-schedule agreement, deterministic passive layer ordering, and "
            "the tooling-authority meta-selection are stricter than GOAL.md."
        ),
        "goal_contract": {
            "wire_path_clause_present": (
                "No bend radius under 3 mm, no contact with unintended edges, valid at all flyer angles."
                in goal
            ),
            "hardware_limit_clause_present": (
                "real wire tension dynamics, snagging, layering neatness, and enamel abrasion are empirical"
                in goal
            ),
        },
        "required_and_currently_proven": required_proven,
        "required_and_currently_unproven": required_unproven,
        "candidate_specific_failures": candidate_specific,
        "stricter_internal_policy_not_DOD3": extra_policy,
        "proposed_acceptance_model": acceptance_model,
        "source_hashes": {
            path.relative_to(ROOT).as_posix() if path.is_relative_to(ROOT)
            else "../GOAL.md": _sha256(path)
            for path in (GOAL, WIRE, ELASTIC, TOOLING, COIL, Path(__file__))
        },
    }
    result["report_sha256"] = _canonical_hash(result)
    return result


def render_markdown(report: Mapping[str, Any]) -> str:
    lines = [
        "# GOAL.md DoD #3 scope audit",
        "",
        f"**Conclusion: {report['conclusion']}**",
        "",
        str(report["conclusion_detail"]),
        "",
        "This is advisory evidence only; it does not alter any production gate.",
        "",
        "## Required and already proven",
        "",
    ]
    for row in report["required_and_currently_proven"]:
        lines.append(
            f"- {'PASS' if row['passed'] else 'FAIL'} — `{row['id']}`: "
            f"{row['evidence']}"
        )
    lines.extend(["", "## Required and still unproven", ""])
    for row in report["required_and_currently_unproven"]:
        lines.extend([
            f"- **{row['status']} — `{row['id']}`**",
            f"  - Required because: {row['why_required']}",
            f"  - Why it cannot be waived: {row['why_not_waivable']}",
            f"  - Current evidence: `{json.dumps(row['current_evidence'], sort_keys=True)}`",
        ])
    lines.extend(["", "## Candidate-specific failure", ""])
    for row in report["candidate_specific_failures"]:
        lines.append(f"- `{row['id']}` — {row['interpretation']}")
    lines.extend(["", "## Stricter internal policy, not DoD #3", ""])
    for row in report["stricter_internal_policy_not_DOD3"]:
        lines.append(f"- `{row['id']}` — {row['reason']}")
    lines.extend([
        "",
        "## Proposed acceptance model",
        "",
        f"**{report['proposed_acceptance_model']['name']}**",
        "",
    ])
    for row in report["proposed_acceptance_model"]["required_checks"]:
        lines.append(f"- `{row['id']}` — {row['rule']}")
    lines.extend([
        "",
        "Passing claim:",
        "",
        f"> {report['proposed_acceptance_model']['passing_claim']}",
        "",
    ])
    return "\n".join(lines)


def write_outputs(report: Mapping[str, Any]) -> None:
    OUTPUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    OUTPUT_MD.write_text(render_markdown(report), encoding="utf-8")


def main() -> int:
    report = analyze()
    write_outputs(report)
    print(f"wrote {OUTPUT_JSON}")
    print(f"wrote {OUTPUT_MD}")
    print(report["conclusion"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
