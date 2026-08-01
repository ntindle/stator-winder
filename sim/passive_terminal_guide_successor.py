"""Fixed-cap impossibility proof and passive terminal-guide successor.

This study consumes the canonical raw capture, the actual production-review
PEEK cap lane, and the physical shaft-to-tip PEEK guide successor.  It reruns
all 2,400 half-turn terminal paths.  Exact strand packing remains outside the
authority boundary: only raw axis loci, the aggregate cap/connector contract,
and source-level guide geometry are used.

The study first asks whether a fixed R3 lead-in can fit inside every existing
cap mouth.  If not, it returns the smallest passive architecture which can
change state without a new commanded axis: one mutually-exclusive R3 ceramic
shoe, self-aligning over the measured approach cone, selected by the existing
M1 index and phased by M2, with forced M0 retraction.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable, Mapping

import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
AGGREGATE_REPORT = REPORTS / "permanent_cap_aggregate_authorization.json"
CAP_REPORT = REPORTS / "permanent_cap_production_review.json"
PEEK_GUIDE_REPORT = REPORTS / "retained_flyer_peek_guide_successor.json"
PREDECESSOR_WIRE_REPORT = REPORTS / "integrated_phase_aware_wire_path.json"
M2_FORMER_REPORT = REPORTS / "m2_cammed_alternating_former.json"
M1_FORMER_REPORT = REPORTS / "m1_selector_alternating_former.json"
OUTPUT_JSON = REPORTS / "passive_terminal_guide_successor.json"
OUTPUT_MD = REPORTS / "passive_terminal_guide_successor.md"
SOURCE = HERE / "passive_terminal_guide_successor.py"

for search_path in (CAD, HERE):
    if str(search_path) not in sys.path:
        sys.path.insert(0, str(search_path))

from phase_aware_progressive_wire_audit import (  # noqa: E402
    EXPECTED_STATE_COUNT,
    extract_raw_loci,
)
from traj import Timeline, load_events  # noqa: E402
import integrated_phase_aware_wire_path as integrated  # noqa: E402
import retained_flyer_peek_guide_successor as peek_guide  # noqa: E402


SCHEMA = "passive-terminal-guide-successor/v1"
R3_MM = 3.0
SELECTED_SHOE_CENTERLINE_RADIUS_MM = 3.25
GIMBAL_RANGE_DEG = 65.0
GIMBAL_ALLOWANCE_DEG = 4.0


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    body = deepcopy(dict(payload))
    body.pop("report_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} is not a JSON object")
    return value


def _minimum_port_spacing(lane: Mapping[str, Any]) -> dict[str, Any]:
    front = lane["nominal_front_centerline"]
    base = [
        np.asarray(front["outgoing_endpoint_mm"], dtype=float)[:2],
        np.asarray(front["incoming_endpoint_mm"], dtype=float)[:2],
    ]
    points = []
    for tooth in range(24):
        angle = tooth * 2.0 * math.pi / 24.0
        c, s = math.cos(angle), math.sin(angle)
        rotation = np.array([[c, -s], [s, c]])
        for side, point in zip(("left", "right"), base):
            points.append({
                "tooth": tooth,
                "side": side,
                "xy_mm": (rotation @ point).tolist(),
            })
    best = None
    for index, left in enumerate(points):
        for right in points[index + 1:]:
            distance = float(np.linalg.norm(
                np.asarray(left["xy_mm"]) - np.asarray(right["xy_mm"])
            ))
            if best is None or distance < best[0]:
                best = (distance, left, right)
    assert best is not None
    return {
        "port_count_per_cap": len(points),
        "minimum_center_spacing_mm": best[0],
        "first_port": best[1],
        "second_port": best[2],
        "two_independent_R3_diameter_mm": 2.0 * R3_MM,
        "independent_R3_envelope_overlap_mm": 2.0 * R3_MM - best[0],
    }


def analyze() -> dict[str, Any]:
    aggregate = _load(AGGREGATE_REPORT)
    cap = _load(CAP_REPORT)
    guide_report = _load(PEEK_GUIDE_REPORT)
    predecessor = _load(PREDECESSOR_WIRE_REPORT)
    m2_former = _load(M2_FORMER_REPORT)
    m1_former = _load(M1_FORMER_REPORT)

    events = load_events(CAPTURE)
    timeline = Timeline(events)
    loci, passes = extract_raw_loci(events, timeline)
    guide = {
        "center_local_mm": [
            0.0,
            float(peek_guide.GUIDE_SECOND_BEND_CENTER_Y_MM),
            float(peek_guide.base.TIP_GUIDE_CENTER_Z_MM),
        ],
        "axis_local": [0.0, 1.0, 0.0],
        "feed_local_mm": [
            0.0,
            float(peek_guide.GUIDE_FEED_END_Y_MM),
            float(peek_guide.base.TIP_GUIDE_CENTER_Z_MM),
        ],
        "major_radius_mm": 6.5,
        "tube_radius_mm": 3.0,
        "material": "99.8% alumina ceramic",
    }
    transfer = integrated._transfer_audit(
        loci, aggregate["cap_support_lane"], guide,
    )
    if transfer["raw_locus_count"] != EXPECTED_STATE_COUNT:
        raise RuntimeError("terminal successor did not cover 2,400 loci")

    minimum_error = float(transfer["minimum_cap_lane_tangent_error_deg"])
    maximum_error = float(transfer["maximum_cap_lane_tangent_error_deg"])
    theta = math.radians(maximum_error)
    required_lateral_sweep = R3_MM * (1.0 - math.cos(theta))
    required_capture_half_width = R3_MM * math.sin(theta)
    lane_margin = float(
        aggregate["cap_support_lane"]["nominal_front_centerline"]
        ["minimum_sampled_domain_margins_mm"]["sector_inset_mm"]
    )
    mouth_width = float(
        cap["geometry"]["wire_contact"]["open_access_mouth_mm"]
    )
    spacing = _minimum_port_spacing(aggregate["cap_support_lane"])
    fixed_impossible = bool(
        transfer["implicit_kink_locus_count"] > 0
        and transfer["core_crossing_locus_count"] > 0
        and required_lateral_sweep > lane_margin
        and 2.0 * required_capture_half_width > mouth_width
    )

    capture_law_count = len(m2_former["capture_contract"]["cam_laws"])
    required_fingers = [
        row["required_finger"]
        for row in m2_former["necessary_support_geometry"]
        ["necessary_former_demands"]
    ]
    architecture = {
        "id": "m1-index-selected_m2-phased_single-deployed_R3-gimbal-shoe/v1",
        "classification": "SMALLEST_PASSIVE_MOVING_SUCCESSOR",
        "new_commanded_axis": False,
        "upstream_protocol_change": False,
        "state_sources": {
            "M1": "24-sector ternary selector chooses one of three raw cam laws",
            "M2": "signed face cam selects one of four end/tangential shoe identities",
            "M0": "positive withdrawal ramp forces every shoe retracted before index, wrap, load or unload",
            "spring": "all shoes fail to retracted; only positively selected shoe can deploy",
            "wire_tension": "two-axis ceramic shoe gimbal self-aligns within the selected approach cone",
        },
        "law_count": capture_law_count,
        "shoe_count": 4,
        "required_shoe_identities": required_fingers,
        "simultaneously_deployed_shoes": 1,
        "ceramic_wire_center_radius_mm": SELECTED_SHOE_CENTERLINE_RADIUS_MM,
        "gimbal_range_deg": [-GIMBAL_RANGE_DEG, GIMBAL_RANGE_DEG],
        "measured_required_cone_deg": [-maximum_error, maximum_error],
        "angular_allowance_deg": GIMBAL_RANGE_DEG - maximum_error,
        "positive_selection_required": True,
        "integration_source": "cad/m1_selector_alternating_former.py",
        "integration_api": {
            "M1_law": "law_for_m1_angle(angle_deg)",
            "M0_gate": "gate_state_for_axis_z(axis_z_mm)",
            "selector": "selector_code_collar() + docking_tongue() + selector_receiver()",
            "M2_cam": "signed_face_cam_rotor() + cam_followers_and_selector_comb()",
            "shoe_predecessor": "guide_finger(finger_id, deployed)",
            "required_change": "replace rigid nose with +/-65deg two-axis ceramic gimbal shoe and bind its distal R3 centerline to cap-r3-sector-lane-v1",
        },
        "why_one_memoryless_fixed_cam_is_not_enough": m2_former[
            "stationary_cam_phase_alias"
        ]["decisive_same_direction_witness"],
        "predecessor_selector_status": m1_former["status"],
        "predecessor_selector_limit": (
            "prior study used exact packed-strand route topology; it is not "
            "promoted to current aggregate authority, but its positive "
            "selector/interlock hardware remains reusable"
        ),
    }

    authority_gates = {
        "canonical_raw_24_pass_2400_locus_capture": (
            len(passes) == 24 and len(loci) == EXPECTED_STATE_COUNT
        ),
        "physical_shaft_to_tip_PEEK_guide_geometry_pass": all(
            guide_report["geometry_gates"].values()
        ),
        "aggregate_cap_lane_authority_PASS": (
            aggregate["status"] == "PASS"
            and aggregate["aggregate_geometry_authorized"] is True
        ),
        "exact_strand_packing_not_authority": (
            aggregate["aggregate_loft"]["exact_strand_packing_predicted"]
            is False
        ),
    }
    fixed_gates = {
        "all_2400_paths_construct": (
            transfer["constructed_locus_count"] == EXPECTED_STATE_COUNT
        ),
        "all_2400_arrive_tangent": (
            transfer["implicit_kink_locus_count"] == 0
        ),
        "no_terminal_span_core_crossing": (
            transfer["core_crossing_locus_count"] == 0
        ),
        "R3_turn_sweep_fits_authorized_sector_margin": (
            required_lateral_sweep <= lane_margin
        ),
        "approach_cone_fits_current_open_mouth": (
            2.0 * required_capture_half_width <= mouth_width
        ),
        "independent_R3_mouth_guides_do_not_overlap": (
            spacing["independent_R3_envelope_overlap_mm"] <= 0.0
        ),
    }
    passive_gates = {
        "three_raw_cam_laws_positively_selected_by_M1": capture_law_count == 3,
        "four_mutually_exclusive_R3_shoes_defined": (
            len(set(required_fingers)) == 4
        ),
        "gimbal_contains_measured_approach_cone": (
            GIMBAL_RANGE_DEG >= maximum_error + GIMBAL_ALLOWANCE_DEG
        ),
        "M0_forced_retraction_and_hardwired_interlock_source_exists": (
            m1_former["gates"]["M0_forced_retracted_index_wrap_load"]
            is True
        ),
        "all_2400_gimballed_shoe_routes_clear_core_and_aggregate": False,
        "complete_passive_shoe_collision_sweep": False,
        "loads_balance_wear_and_endurance_complete": False,
    }

    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "FAIL",
        "decision": (
            "FIXED_CAP_LEAD_IN_IMPOSSIBLE_WITHIN_AUTHORIZED_SECTOR__M1_M2_SELECTED_SINGLE_GIMBAL_SHOE_REQUIRED"
            if fixed_impossible else
            "FIXED_CAP_LEAD_IN_NOT_YET_DISPROVED__REMAIN_FAIL_CLOSED"
        ),
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "authority_boundary": {
            "raw_M0_M1_M2": "authoritative",
            "aggregate_copper_and_cap_lane": "authoritative",
            "exact_strand_centers_order_settling_neatness": "non-authoritative",
        },
        "source_guide": {
            "path": "out/reports/retained_flyer_peek_guide_successor.json",
            "status": guide_report["status"],
            "geometry_gates_pass": all(guide_report["geometry_gates"].values()),
            "report_sha256": guide_report["report_sha256"],
        },
        "raw_terminal_sweep": {
            "pass_count": len(passes),
            "locus_count": transfer["raw_locus_count"],
            "unique_geometry_cases": transfer["unique_geometry_case_count"],
            "constructed_loci": transfer["constructed_locus_count"],
            "implicit_kink_loci": transfer["implicit_kink_locus_count"],
            "core_crossing_loci": transfer["core_crossing_locus_count"],
            "minimum_tangent_error_deg": minimum_error,
            "maximum_tangent_error_deg": maximum_error,
            "worst_witness": transfer["worst_cap_entry_witness"],
            "first_core_witness": transfer["first_core_crossing_witness"],
            "predecessor_counts_match": (
                transfer["implicit_kink_locus_count"]
                == predecessor["tip_to_active_PEEK_cap"]
                ["implicit_kink_locus_count"]
                and transfer["core_crossing_locus_count"]
                == predecessor["tip_to_active_PEEK_cap"]
                ["core_crossing_locus_count"]
            ),
        },
        "fixed_cap_lead_in_impossibility": {
            "proved": fixed_impossible,
            "worst_approach_turn_deg": maximum_error,
            "R3_minimum_lateral_turn_sweep_mm": required_lateral_sweep,
            "authorized_lane_sector_margin_mm": lane_margin,
            "lateral_sweep_deficit_mm": required_lateral_sweep - lane_margin,
            "R3_capture_half_width_at_worst_angle_mm": required_capture_half_width,
            "required_full_capture_width_mm": 2.0 * required_capture_half_width,
            "current_open_mouth_width_mm": mouth_width,
            "mouth_width_deficit_mm": 2.0 * required_capture_half_width - mouth_width,
            "port_spacing": spacing,
            "core_crossing_loci_before_any_fixed_mouth_turn": transfer[
                "core_crossing_locus_count"
            ],
            "proof": (
                "A radius-R circular redirection through angle theta needs "
                "lateral sweep R*(1-cos(theta)) and capture half-width "
                "R*sin(theta).  At the measured worst raw approach both "
                "exceed the cap's authorized owned-sector margin/open mouth; "
                "1,000 direct spans already enter bare core. Enlarging a "
                "fixed mouth would leave the authorized tooth sector and "
                "enter neighboring cap/copper ownership."
            ),
        },
        "smallest_passive_architecture": architecture,
        "authority_gates": authority_gates,
        "fixed_cap_gates": fixed_gates,
        "passive_successor_gates": passive_gates,
        "release_blockers": [
            "The M1/M2-selected gimbal shoe does not yet have an all-2,400 aggregate-aware route proof.",
            "No complete gimbal/finger/cap/flyer collision sweep exists.",
            "The current canonical upstream shaft wraps are not two physical turns each.",
            "Loads, balance, ceramic/PEEK wear, spring life and enamel abrasion remain physical gates.",
        ],
        "source_hashes": {
            "sim/passive_terminal_guide_successor.py": _sha256(SOURCE),
            "out/capture/upstream_current_raw.jsonl": _sha256(CAPTURE),
            "out/reports/permanent_cap_aggregate_authorization.json": _sha256(AGGREGATE_REPORT),
            "out/reports/permanent_cap_production_review.json": _sha256(CAP_REPORT),
            "out/reports/retained_flyer_peek_guide_successor.json": _sha256(PEEK_GUIDE_REPORT),
            "out/reports/integrated_phase_aware_wire_path.json": _sha256(PREDECESSOR_WIRE_REPORT),
            "out/reports/m2_cammed_alternating_former.json": _sha256(M2_FORMER_REPORT),
            "out/reports/m1_selector_alternating_former.json": _sha256(M1_FORMER_REPORT),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    validate_report(report)
    return report


def validate_report(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported passive terminal guide schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("passive terminal guide report hash mismatch")
    if report.get("status") != "FAIL":
        raise ValueError("passive terminal successor must remain fail closed")
    if report.get("production_authorized") is not False:
        raise ValueError("passive terminal guide invented production authority")
    sweep = report.get("raw_terminal_sweep", {})
    if int(sweep.get("locus_count", -1)) != EXPECTED_STATE_COUNT:
        raise ValueError("passive terminal guide lacks 2,400 raw loci")
    if not report.get("fixed_cap_lead_in_impossibility", {}).get("proved"):
        raise ValueError("fixed cap impossibility proof did not close")
    if report.get("authority_boundary", {}).get(
            "exact_strand_centers_order_settling_neatness") \
            != "non-authoritative":
        raise ValueError("exact strand packing was promoted to authority")


def render_markdown(report: Mapping[str, Any]) -> str:
    sweep = report["raw_terminal_sweep"]
    fixed = report["fixed_cap_lead_in_impossibility"]
    arch = report["smallest_passive_architecture"]
    lines = [
        "# Passive terminal-guide successor",
        "",
        f"**{report['status']} - {report['decision']}**",
        "",
        "The physical PEEK shaft-to-tip guide passes its isolated geometry gates. The terminal route still fails the complete canonical raw sweep.",
        "",
        "## All-2,400 raw sweep",
        "",
        f"- Constructed: {sweep['constructed_loci']} / {sweep['locus_count']}",
        f"- Non-tangent cap entries: {sweep['implicit_kink_loci']}",
        f"- Bare-core crossings: {sweep['core_crossing_loci']}",
        f"- Approach cone: {sweep['minimum_tangent_error_deg']:.6f}..{sweep['maximum_tangent_error_deg']:.6f} degrees",
        "",
        "## Why a fixed cap mouth cannot close it",
        "",
        f"- Worst R3 lateral turn sweep: {fixed['R3_minimum_lateral_turn_sweep_mm']:.6f} mm.",
        f"- Authorized sector margin: {fixed['authorized_lane_sector_margin_mm']:.6f} mm.",
        f"- Required/current mouth width: {fixed['required_full_capture_width_mm']:.6f}/{fixed['current_open_mouth_width_mm']:.6f} mm.",
        f"- Minimum spacing among 48 ports on one cap: {fixed['port_spacing']['minimum_center_spacing_mm']:.6f} mm; independent R3 overlap {fixed['port_spacing']['independent_R3_envelope_overlap_mm']:.6f} mm.",
        "",
        "## Smallest passive moving architecture",
        "",
        f"`{arch['id']}`",
        "",
        "One mutually-exclusive R3.25 ceramic shoe is selected by the existing M1 index and phased by M2. A two-axis +/-65 degree gimbal follows the measured approach cone; M0 withdrawal positively retracts every shoe. No new commanded axis or protocol change is introduced.",
        "",
        "This architecture is not released: its aggregate-aware 2,400-route, collision, load, wear and endurance gates remain open. Exact strand packing is not claimed.",
        "",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ]
    return "\n".join(lines)


def write_outputs(report: Mapping[str, Any]) -> None:
    OUTPUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    OUTPUT_MD.write_text(render_markdown(report), encoding="utf-8")


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(list(argv) if argv is not None else None)
    if args.validate_only:
        report = _load(OUTPUT_JSON)
        validate_report(report)
    else:
        report = analyze()
        write_outputs(report)
    sweep = report["raw_terminal_sweep"]
    print(
        f"passive terminal {report['status']}: "
        f"{sweep['implicit_kink_loci']} kinks, "
        f"{sweep['core_crossing_loci']} core crossings, "
        f"sha256 {report['report_sha256']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
