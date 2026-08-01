"""Build the controller-facing plan from the exact release packing audit.

``cad/slot_packing_audit.py`` owns the measured-input geometry, fixed
Hamiltonian topology, exact OpenCascade core distances, and receiving-window
sensitivity proof. This module does not invent another packing. It adapts
that report into the strict
``slot-winding-plan/v1`` artifact consumed by the controller capture and the
browser player.

Run directly to regenerate ``out/reports/slot_winding_plan.json`` and ``.md``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

from params import DEFAULT_STATOR  # noqa: E402
import coil_growth  # noqa: E402
import slot_packing_audit  # noqa: E402


SCHEMA = "slot-winding-plan/v1"
OUT_JSON = ROOT / "out" / "reports" / "slot_winding_plan.json"
OUT_MD = ROOT / "out" / "reports" / "slot_winding_plan.md"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: dict) -> str:
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _require_release_identity(report: dict) -> None:
    spec = DEFAULT_STATOR
    policy = coil_growth.DEFAULT_POLICY
    expected = (24, 46.0, 15.0, 50, 0.22352, 0.127)
    actual = (
        int(spec.slots), float(spec.od), float(spec.stack), int(spec.turns),
        float(spec.wire_d),
        float(policy.opening_edge_clearance_mm),
    )
    if actual != expected:
        raise RuntimeError(
            "release winding plan identity drifted: "
            f"expected {expected}, got {actual}"
        )
    if report.get("schema") != "slot-packing/v2":
        raise RuntimeError("packing audit schema is not slot-packing/v2")
    if report.get("status") != "PASS":
        raise RuntimeError("packing audit is not PASS")
    if report.get("role") not in {
            "authoritative_release_default",
            "authoritative_measured_release_job"}:
        raise RuntimeError("packing audit is not marked authoritative")
    config = report.get("config", {})
    selected = report.get("selected_schedule", {})
    base_identity = (
        int(config.get("slots", -1)),
        float(config.get("od_mm", math.nan)),
        float(config.get("stack_mm", math.nan)),
        int(selected.get("turns_per_tooth", -1)),
    )
    if base_identity != (24, 46.0, 15.0, 50):
        raise RuntimeError(
            "packing audit base job mismatch: " f"{base_identity}"
        )
    wire_d = float(config.get("wire_finished_diameter_mm", math.nan))
    liner_t = float(config.get("liner_thickness_mm", math.nan))
    wire_range = report.get("receiving_contract", {}).get(
        "wire_finished_diameter_range_mm", [])
    liner_range = report.get("receiving_contract", {}).get(
        "liner_thickness_range_mm", [])
    if (len(wire_range) != 2 or len(liner_range) != 2
            or not wire_range[0] <= wire_d <= wire_range[1]
            or not liner_range[0] <= liner_t <= liner_range[1]):
        raise RuntimeError(
            "packing audit measured inputs are outside its receiving contract"
        )


def build_plan(
    job: slot_packing_audit.PackingInput | None = None,
    *,
    report: dict | None = None,
) -> dict:
    """Return one hash-bound controller/presentation plan."""

    if report is not None and job is not None:
        raise ValueError("pass either a packing report or a job, not both")
    report = report or slot_packing_audit.analyze(job)
    _require_release_identity(report)
    selected = report["selected_schedule"]
    validation = report["validation"]
    positive = selected["side_positive"]
    negative = selected["side_negative"]
    if len(positive) != 50 or len(negative) != 50:
        raise RuntimeError("packing audit must contain 50 turns on both sides")

    placements = []
    support_rows = []
    for index, (left, right) in enumerate(zip(positive, negative)):
        left_uv = [float(value) for value in left["slot_frame_uv_mm"]]
        right_uv = [float(value) for value in right["slot_frame_uv_mm"]]
        if (abs(left_uv[0] - right_uv[0]) > 1e-10
                or abs(left_uv[1] + right_uv[1]) > 1e-10):
            raise RuntimeError(f"packing turn {index} is not exactly mirrored")
        support = str(left["support_kind"])
        parents = [int(value) for value in left["parent_turn_indices"]]
        contacts = [int(value)
                    for value in left["prior_contact_turn_indices"]]
        active_radial = float(left["radial_parameter_mm"])
        active_tangential = -float(left["normal_profile_radius_mm"])
        placements.append({
            "turn_index": index,
            # These two values are in the slot-bisector frame and exist for
            # exact two-sided presentation geometry.
            "radial_mm": left_uv[0],
            "tangential_mm": left_uv[1],
            # M0 moves along the active tooth radial axis.  Controller code
            # must use this value, never the slot-bisector projection above.
            "active_tooth_radial_mm": active_radial,
            "active_tooth_tangential_mm": active_tangential,
            "m0_target_rad": float(left["m0_target_rad"]),
            "layer": int(left["layer_index"]),
            "row": int(left["layer_index"]),
            "row_order": int(left["lattice_column"]),
            "contact_id": (
                "slot_liner" if support == "slot_liner"
                else "deposited_wire"
            ),
            "support_kind": support,
            "support_predecessor_indices": parents,
            "prior_contact_turn_indices": contacts,
            "left_slot_half_turn_center_mm": left_uv,
            "right_slot_half_turn_center_mm": right_uv,
        })
        support_rows.append({
            "placement_index": index,
            "support": support,
            "support_predecessor_indices": parents,
            "prior_contact_turn_indices": contacts,
            "progressively_supported": True,
        })

    half_turn_centers = []
    for raw in selected["half_turn_waypoints"]:
        index = int(raw["phase_index"])
        placement_index = int(raw["turn_index"])
        placement = placements[placement_index]
        half_turn_centers.append({
            "half_turn_index": index,
            "phase_turns": index / 2.0,
            "placement_index": placement_index,
            "radial_mm": placement["active_tooth_radial_mm"],
            "m0_target_rad": float(raw["m0_target_rad"]),
        })
    if len(half_turn_centers) != 100:
        raise RuntimeError("packing audit must contain 100 half-turn centers")

    pair_min = float(validation["minimum_pair_center_distance_mm"])
    core_min = float(validation["minimum_center_core_distance_mm"])
    wire_d = float(report["config"]["wire_finished_diameter_mm"])
    liner = float(report["config"]["liner_thickness_mm"])
    transition_ok = bool(
        selected["progressive_support_validated"]
        and validation["pair_clearance_ok"]
        and validation["core_access_ok"]
        and validation["radial_cap_ok"]
        and validation["all_schedule_steps_tangent"]
        and validation["all_empty_neighbor_side_mouth_connected"]
        and validation["all_prefilled_neighbor_side_mouth_connected"]
    )
    result = {
        "schema": SCHEMA,
        "algorithm": "measured-input-hamiltonian-mouth-preserving-v2",
        "job": {
            "slots": DEFAULT_STATOR.slots,
            "od_mm": DEFAULT_STATOR.od,
            "stack_mm": DEFAULT_STATOR.stack,
            "wire_finished_d_mm": wire_d,
            "supplier_nominal_wire_finished_d_mm": 0.22352,
            "model_wire_envelope_mm": wire_d,
            "receiving_max_wire_envelope_mm": report[
                "receiving_contract"
            ]["wire_finished_diameter_range_mm"][1],
            "turns_per_tooth": DEFAULT_STATOR.turns,
            "wires_per_final_slot": 2 * DEFAULT_STATOR.turns,
            "liner_model": "DuPont Nomex Type 410 5 mil / BAE INNMX410005S",
            "liner_nominal_thickness_mm": liner,
            "liner_measured_thickness_mm": liner,
            "liner_max_thickness_mm": liner,
            "liner_receiving_max_thickness_mm": report[
                "receiving_contract"
            ]["liner_thickness_range_mm"][1],
            "liner_receiving_range_mm": report[
                "receiving_contract"
            ]["liner_thickness_range_mm"],
        },
        "coordinate_frame": {
            "name": "slot_bisector_local",
            "radial_axis": "+x outward along slot bisector",
            "tangential_axis": "+y toward active tooth",
            "active_tooth_center_angle_deg": (
                float(report["config"]["slot_pitch_deg"]) / 2.0
            ),
            "controller_radial_semantics": (
                "active_tooth_radial_mm; never slot-bisector radial_mm"
            ),
        },
        "source_hashes": {
            **report["source_hashes"],
            "sim/slot_packing.py": _sha256(Path(__file__)),
        },
        "packing_report": {
            "path": "out/reports/slot_packing.json",
            "schema": report["schema"],
            "report_sha256": report["report_sha256"],
        },
        "selected_case": {
            "wire_finished_d_mm": wire_d,
            "wire_radius_mm": wire_d / 2.0,
            "liner_max_thickness_mm": liner,
            "required_center_core_clearance_mm": wire_d / 2.0 + liner,
            "status": "PASS" if transition_ok else "FAIL",
            "transition_proof": {
                "status": "PASS" if transition_ok else "FAIL",
                "method": (
                    "every consecutive placement is one modeled wire "
                    "diameter apart; each deposited layer names tangent "
                    "earlier parents; the Hamiltonian order retains the "
                    "conservative slot-mouth component with the opposite "
                    "neighbor side already full"
                ),
                "sequential_mouth_access": selected[
                    "sequential_mouth_access"
                ],
                "first_side_insertion": support_rows,
                "all_consecutive_center_distances_mm": validation[
                    "all_consecutive_schedule_distances_mm"
                ],
            },
            "final_slot_proof": {
                "status": "PASS" if (
                    validation["pair_clearance_ok"]
                    and validation["core_access_ok"]
                    and validation["radial_cap_ok"]
                ) else "FAIL",
                "center_count": 100,
                "centers_per_coil_side": 50,
                "pair_count_checked": 4950,
                "minimum_pairwise_center_distance_mm": pair_min,
                "minimum_pairwise_margin_mm": pair_min - wire_d,
                "minimum_center_core_clearance_mm": core_min,
                "required_center_core_clearance_mm": wire_d / 2.0 + liner,
                "minimum_center_core_margin_mm": (
                    core_min - (wire_d / 2.0 + liner)
                ),
                "maximum_center_radius_mm": validation[
                    "maximum_center_radius_mm"
                ],
            },
        },
        "placements": placements,
        "half_turn_centers": half_turn_centers,
        "final_hold_policy": {
            "placement_index": 49,
            "hold_through_lead_out": True,
            "m0_target_rad": placements[-1]["m0_target_rad"],
            "reason": "do not retract M0 during a fractional lead-out turn",
        },
        "receiving_wire_envelope_case": {
            "wire_finished_d_mm": report[
                "receiving_contract"
            ]["wire_finished_diameter_range_mm"][1],
            "supplier_nominal_finished_d_mm": 0.22352,
            "wire_finished_diameter_range_mm": report[
                "receiving_contract"
            ]["wire_finished_diameter_range_mm"],
            "status": report["receiving_contract"][
                "topology_sensitivity_status"],
            "rule": report["receiving_contract"]["rule"],
        },
        "scope": {
            "proved": [
                "100 exact final wire centers in one shared slot",
                "exact OpenCascade core and full-liner clearance",
                "all 4,950 pairwise non-overlap checks",
                "49 consecutive tangent motion transitions",
                "liner support for the seed and named earlier-wire parents",
                "fixed topology at every receiving-window corner",
            ],
            "separate_release_gates": [
                "3D moving-wire routes against core and deposited copper",
                "controller timing and full-cycle captured motion",
                "hardware receiving inspection and instrumented winding coupon",
            ],
        },
    }
    result["proof_sha256"] = _canonical_hash(result)
    return result


def render_markdown(plan: dict) -> str:
    selected = plan["selected_case"]
    proof = selected["final_slot_proof"]
    job = plan["job"]
    steps = selected["transition_proof"][
        "all_consecutive_center_distances_mm"
    ]
    return f"""# Constructive release winding plan

**Overall: {selected['status']}**

This controller plan is an adapter over the exact OpenCascade packing audit;
it is not a second independently invented layout.

- Nominal measured wire input: {job['wire_finished_d_mm']:.5f} mm
- Selected wire nominal finished OD: {job['supplier_nominal_wire_finished_d_mm']:.4f} mm
- Nominal Nomex liner input: {job['liner_max_thickness_mm']:.3f} mm
- Turns per tooth: {job['turns_per_tooth']}
- Wire centers per final shared slot: {proof['center_count']}
- Exact pair checks: {proof['pair_count_checked']}
- Minimum center/core distance: {proof['minimum_center_core_clearance_mm']:.12f} mm
- Consecutive placement distance: {min(steps):.12f}..{max(steps):.12f} mm
- Hamiltonian lattice layers: 0 through 3

The seed bears on the liner. Every later center names at least one already
deposited tangent parent. The separate packed-route report
must pass before this controller schedule is a release artifact.

Proof SHA-256: `{plan['proof_sha256']}`
"""


def write_reports(json_path: Path = OUT_JSON,
                  md_path: Path = OUT_MD,
                  job: slot_packing_audit.PackingInput | None = None,
                  *, report: dict | None = None) -> dict:
    plan = build_plan(job, report=report)
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(
        json.dumps(plan, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(render_markdown(plan), encoding="utf-8")
    return plan


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--wire", type=float,
        default=slot_packing_audit.SUPPLIER_NOMINAL_WIRE_MM,
        help="measured finished magnet-wire outside diameter, mm",
    )
    parser.add_argument(
        "--liner", type=float,
        default=slot_packing_audit.SUPPLIER_NOMINAL_LINER_MM,
        help="measured installed liner thickness, mm",
    )
    parser.add_argument("--json", type=Path, default=OUT_JSON)
    parser.add_argument("--markdown", type=Path, default=OUT_MD)
    args = parser.parse_args()
    selected = slot_packing_audit.PackingInput(args.wire, args.liner)
    default = slot_packing_audit.PackingInput(
        slot_packing_audit.SUPPLIER_NOMINAL_WIRE_MM,
        slot_packing_audit.SUPPLIER_NOMINAL_LINER_MM,
    )
    # Preserve the canonical default artifact identity when CLI defaults are
    # used. Passing an explicit-but-equal measured job changes provenance and
    # the packing hash even though every geometry coordinate is identical,
    # which unnecessarily invalidates capture/route bindings.
    generated = write_reports(
        args.json, args.markdown,
        None if selected == default else selected,
    )
    selected = generated["selected_case"]
    print(
        f"slot winding packing {selected['status']}; controller transition "
        f"{selected['transition_proof']['status']}: "
        f"{selected['final_slot_proof']['center_count']} centers; "
        "measured-input Hamiltonian schedule"
    )


if __name__ == "__main__":
    main()
