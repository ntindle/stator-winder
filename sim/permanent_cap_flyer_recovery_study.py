"""Bounded recovery audit for the permanent stator winding-guide cap.

The earlier cap study rejected the cap against the production R45 flyer.  This
audit asks the narrower follow-up question: can that same cap architecture be
recovered by changing only the flyer tip radius and the axial position of its
ceramic tip guide while leaving the upstream Aotenjo command stream intact?

The answer is fail-closed.  The controlling collision is not at the movable
tip guide.  A tooth-0 outboard return pad intersects the two load-carrying
rails of the flyer's radial spoke.  Those rails are invariant under both
allowed changes, so one exact positive-volume witness rejects every point in
the bounded sweep.  A wider/windowed or non-radial arm would be a new flyer
architecture and is deliberately outside this recovery audit.

No production CAD, controller input, collision exclusion, or stored packing
schedule is modified by this source.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

from build123d import Align, Box, Part, Pos, Rot


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
CAP_REPORT = REPORTS / "stator_winding_guide_cap.json"
RAW_CYCLE_REPORT = REPORTS / "upstream_current_raw_cycle.json"
LOADS_REPORT = REPORTS / "loads.json"
CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
JSON_OUT = REPORTS / "permanent_cap_flyer_recovery.json"
MD_OUT = REPORTS / "permanent_cap_flyer_recovery.md"

for path in (CAD, HERE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import coil_growth  # noqa: E402
from params import PARAMS, StatorSpec  # noqa: E402
import settings_gen  # noqa: E402
import stator_winding_guide_cap as cap  # noqa: E402
import stator_winding_guide_cap_study as cap_study  # noqa: E402
import wire_geometry  # noqa: E402


SCHEMA = "permanent-cap-flyer-recovery/v1"
DYNAMIC_CLEARANCE_MM = 2.0
R3_MM = 3.0
LAUNCH_SPEC = StatorSpec(od=65.0, stack=20.0, shaft_d=7.0)
MAX_LAUNCH_WIRE_MM = 0.50

# The production radius is included, the reasonable geometry-only extension
# reaches 60 mm, and the axial range straddles the existing -17 mm datum.
# A positive-volume invariant makes a denser grid unnecessary.
TIP_RADII_MM = (45.0, 48.0, 52.0, 56.0, 60.0)
TIP_GUIDE_Z_MM = (-24.0, -21.0, -18.0, -17.0, -15.0, -12.0, -9.0)
CAP_WALL_OPTIONS_MM = (0.50, 1.00)

# Exact production flyer-spoke contract from cad/printed.py.  The open center
# window leaves two 3.5 mm rails; the guide-pad witness cuts both rails.
SPOKE_HALF_WIDTH_MM = PARAMS.flyer_arm_w / 2.0
SPOKE_WINDOW_HALF_WIDTH_MM = 3.5
SPOKE_Y_MAX_COMMON_MM = PARAMS.flyer_tip_r + PARAMS.flyer_arm_w / 2.0
SPOKE_Z0_MM, SPOKE_Z1_MM = map(float, PARAMS.spoke_z)
SPOKE_WINDOW_Y0_MM = -1.0
SPOKE_WINDOW_Y1_MM = 27.0

# cad/wire_geometry.py documents this exact maximum-stack captured-shaft
# envelope.  Keeping the expression here makes the axial sweep fail closed.
MAX_STACK_SHAFT_TIP_Z_MM = -10.915

SOURCE_PPS_URL = (
    "https://www.solvay.com/sites/g/files/srpend616/files/2018-10/"
    "Ryton-PPS-Design-Guide_EN-v2.3_0.pdf"
)
SOURCE_PPS_STATOR_URL = (
    "https://www.solvay.com/en/press-release/"
    "new-ryton-pps-supreme-high-voltage-and-high-flow-polymers"
)
SOURCE_PEEK_URL = (
    "https://www.victrex.com/en/blog/2019/"
    "five-factors-to-consider-when-moulding-peek"
)


@dataclass(frozen=True)
class SweepCandidate:
    tip_radius_mm: float
    tip_guide_z_mm: float


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: dict[str, Any]) -> str:
    value = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False
    )
    return hashlib.sha256(value.encode()).hexdigest()


def _box(x0: float, x1: float, y0: float, y1: float,
         z0: float, z1: float) -> Part:
    """Closed axis-aligned part from explicit bounds."""

    result = Pos(
        (x0 + x1) / 2.0,
        (y0 + y1) / 2.0,
        (z0 + z1) / 2.0,
    ) * Box(
        x1 - x0,
        y1 - y0,
        z1 - z0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    return result


def _bbox_record(part: Part) -> dict[str, list[float]]:
    box = part.bounding_box()
    return {
        "minimum_mm": [float(box.min.X), float(box.min.Y), float(box.min.Z)],
        "maximum_mm": [float(box.max.X), float(box.max.Y), float(box.max.Z)],
        "size_mm": [float(box.size.X), float(box.size.Y), float(box.size.Z)],
    }


def raw_contract() -> dict[str, Any]:
    cycle = json.loads(RAW_CYCLE_REPORT.read_text())
    meta = json.loads(CAPTURE.read_text().splitlines()[0])
    capture_sha = _sha256(CAPTURE)
    exact_two_turn_check = cycle["checks"].get(
        "both raw shaft-wrap intervals execute exactly two M1 turns", {}
    )
    other_cycle_checks_pass = all(
        bool(row.get("ok"))
        for name, row in cycle["checks"].items()
        if name != "both raw shaft-wrap intervals execute exactly two M1 turns"
    )
    checks = {
        # The untouched upstream cycle is deliberately fail-closed because its
        # two completed shaft-wrap displacements are 1.375 and 2.791667 turns.
        # That physical blocker must not make an otherwise intact raw-capture
        # provenance check look corrupt.
        "cycle_report_fail_is_only_exact_two_turn_blocker": (
            cycle.get("status") == "FAIL"
            and cycle.get("fail") == [
                "both raw shaft-wrap intervals execute exactly two M1 turns"
            ]
            and other_cycle_checks_pass
            and exact_two_turn_check.get("ok") is False
        ),
        "capture_sha_matches_cycle_report": (
            capture_sha == cycle["capture"]["sha256"]
        ),
        "controller_mode_is_unmodified_upstream": (
            meta.get("controller_mode") == "upstream"
        ),
        "no_controller_adapter": meta.get("controller_adapter_sha256") is None,
        "no_injected_winding_plan": meta.get("winding_plan") is None,
        "24_passes_50_turns": (
            cycle["counts"].get("wind_wire") == 24
            and meta.get("turns") == 50
        ),
        "all_four_axis_records_present": (
            set(map(int, cycle["axis_commands"])) == {0, 1, 2, 3}
        ),
        "two_raw_shaft_wrap_calls": (
            cycle["counts"].get("wind_wire_around_shaft") == 2
        ),
    }
    return {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "capture": str(CAPTURE),
        "capture_sha256": capture_sha,
        "capture_schema": meta.get("capture_schema"),
        "winder_commit": meta.get("winder_commit"),
        "settings_sha256": meta.get("settings_sha256"),
        "candidate_changes_command_stream": False,
    }


def exact_common_spoke_witness() -> dict[str, Any]:
    """Return an exact positive-volume cap/spoke intersection witness.

    The witness uses turn 0 at the same M0 depth and M2=0 pose reported by
    the full 18,000-sample cap audit.  Only a subset of the real flyer spoke
    is used.  Therefore positive intersection here is sufficient to reject
    the complete flyer; no mesh tolerance or nearest-part ranking is involved.
    """

    _packing, graph = cap_study._load_graph()
    radial_min = min(float(turn.radial_mm) for turn in graph.turns)
    radial_max = max(float(turn.radial_mm) for turn in graph.turns)
    profile_min = min(float(turn.profile_radius_mm) for turn in graph.turns)
    profile_max = max(float(turn.profile_radius_mm) for turn in graph.turns)
    radial_span = radial_max - radial_min
    profile_span = profile_max - profile_min

    # front cap tooth-0 pad: parts are face, rib, pad, then repeated.
    pad_local = cap.guide_cap_parts(
        1, radial_span_mm=radial_span, profile_span_mm=profile_span
    )[2]
    axis_z = wire_geometry.TOOTH_CONTACT_Z + radial_min
    local_to_world = Pos(0.0, 0.0, axis_z) * Rot(0.0, 90.0, 0.0) * Rot(
        -90.0, 0.0, 0.0
    )
    pad_world = local_to_world * pad_local

    spoke = _box(
        -SPOKE_HALF_WIDTH_MM, SPOKE_HALF_WIDTH_MM,
        0.0, SPOKE_Y_MAX_COMMON_MM,
        SPOKE_Z0_MM, SPOKE_Z1_MM,
    )
    window = _box(
        -SPOKE_WINDOW_HALF_WIDTH_MM, SPOKE_WINDOW_HALF_WIDTH_MM,
        SPOKE_WINDOW_Y0_MM, SPOKE_WINDOW_Y1_MM,
        SPOKE_Z0_MM - 2.0, SPOKE_Z1_MM + 2.0,
    )
    common_spoke_rails = spoke - window
    intersection = common_spoke_rails & pad_world
    volume = float(intersection.volume)
    solids = list(intersection.solids())

    return {
        "status": "FAIL" if volume > 1e-9 else "PASS",
        "pose": {
            "packing_turn_index": 0,
            "stator_axis_machine_z_mm": axis_z,
            "flyer_angle_deg": 0.0,
        },
        "geometry": {
            "spoke_half_width_mm": SPOKE_HALF_WIDTH_MM,
            "spoke_window_half_width_mm": SPOKE_WINDOW_HALF_WIDTH_MM,
            "spoke_z_span_mm": [SPOKE_Z0_MM, SPOKE_Z1_MM],
            "common_spoke_y_span_mm": [0.0, SPOKE_Y_MAX_COMMON_MM],
            "cap_pad_world_bbox": _bbox_record(pad_world),
        },
        "intersection_volume_mm3": volume,
        "intersection_solid_count": len(solids),
        "intersection_bbox": (
            _bbox_record(intersection) if volume > 1e-9 else None
        ),
        "minimum_clearance_upper_bound_mm": 0.0,
        "required_dynamic_clearance_mm": DYNAMIC_CLEARANCE_MM,
        "invariant_under_tip_radius": (
            "all swept radii are >=45 mm, while the witness lies inside the "
            "unchanged 0..52 mm radial spoke subset"
        ),
        "invariant_under_tip_guide_z": (
            "changing the ceramic torus/seat Z does not alter the spoke rails"
        ),
    }


def stored_route_contact_semantics() -> dict[str, Any]:
    """Classify the old route witnesses without imposing neat layering."""

    old = json.loads(CAP_REPORT.read_text())
    route = old["route_audit"]
    chord = float(route["sampled_arc_chord_error_bound_each_mm"])
    wire = float(route["wire_diameter_mm"])
    rows = []
    for name, minimum_key, case_key in (
        ("prior_nonparent", "minimum_nonparent_centerline_lower_bound_mm",
         "minimum_nonparent_case"),
        ("declared_parent", "minimum_parent_centerline_lower_bound_mm",
         "minimum_parent_case"),
        ("neighbor_tooth", "minimum_neighbor_centerline_lower_bound_mm",
         "minimum_neighbor_case"),
    ):
        lower = float(route[minimum_key])
        raw = float(route[case_key]["raw_centerline_distance_mm"])
        # Intended adjacent copper requires at least one wire diameter between
        # centrelines.  Near-zero is a true centreline crossing, not contact.
        classification = (
            "ACTUAL_CENTERLINE_CROSSING"
            if raw + 2.0 * chord < wire else "INTENDED_ADJACENT_CONTACT"
        )
        rows.append({
            "class": name,
            "raw_centerline_distance_mm": raw,
            "certified_lower_bound_mm": lower,
            "required_for_adjacent_contact_mm": wire,
            "classification": classification,
            "witness": route[case_key],
        })
    return {
        "status": "FAIL" if any(
            row["classification"] == "ACTUAL_CENTERLINE_CROSSING"
            for row in rows
        ) else "PASS",
        "cases": rows,
        "interpretation": (
            "The stored deterministic 50-route family cannot be accepted by "
            "renaming zero-distance crossings as intended contact. GOAL.md "
            "does not require this exact strand order, so this rejects that "
            "route family rather than every possible naturally settled pack. "
            "No alternative cap-guided noncrossing R3 family is proven here."
        ),
        "controlling_architecture_rejection": False,
    }


def launch_envelope() -> dict[str, Any]:
    nominal = coil_growth.analyze_job(LAUNCH_SPEC)
    maximum_wire_spec = StatorSpec(
        od=LAUNCH_SPEC.od,
        stack=LAUNCH_SPEC.stack,
        shaft_d=LAUNCH_SPEC.shaft_d,
        wire_d=MAX_LAUNCH_WIRE_MM,
        turns=LAUNCH_SPEC.turns,
    )
    maximum_wire = coil_growth.analyze_job(maximum_wire_spec)
    cfg = settings_gen.derive(LAUNCH_SPEC)
    contact = wire_geometry.tooth_contact_spec(LAUNCH_SPEC, nominal)
    deep = float(contact["insertion_depth_range_mm"][1])
    half_chord = math.sqrt(max(
        LAUNCH_SPEC.od ** 2 / 4.0
        - (LAUNCH_SPEC.od / 2.0 - deep) ** 2,
        0.0,
    ))
    required_flyer_radius = (
        math.hypot(half_chord, LAUNCH_SPEC.stack / 2.0)
        + PARAMS.wire_bundle_allow + DYNAMIC_CLEARANCE_MM
    )
    chuck_radius = max(
        float(segment[0])
        for segment in PARAMS.chuck_neck_profile(LAUNCH_SPEC, "er11")
    )
    cap_inner_radius = (
        LAUNCH_SPEC.od * LAUNCH_SPEC.hub_od_ratio / 2.0
    )
    chuck_clearance = cap_inner_radius - chuck_radius

    # Optimistic cap OD lower bounds: no exact packing/profile translation is
    # added.  Any realizable 50-turn cap is at least this large.
    radial_start = min(
        float(nominal["slot_access"]["wire_accessible_start_radius_mm"]),
        float(maximum_wire["slot_access"]["wire_accessible_start_radius_mm"]),
    )
    radial_end = max(
        float(nominal["slot_access"]["wire_accessible_end_radius_mm"]),
        float(maximum_wire["slot_access"]["wire_accessible_end_radius_mm"]),
    )
    radial_span = radial_end - radial_start
    pitch_half = math.pi / LAUNCH_SPEC.slots
    wall_rows = []
    for wall in CAP_WALL_OPTIONS_MM:
        pad_half_width = R3_MM + MAX_LAUNCH_WIRE_MM / 2.0 + wall / 2.0
        ligament_base = (
            2.0 * pad_half_width + cap.CAP_MINIMUM_LIGAMENT_MM
        ) / (2.0 * math.sin(pitch_half))
        neighbor_base = (
            2.0 * (R3_MM + MAX_LAUNCH_WIRE_MM / 2.0)
        ) / (2.0 * math.sin(pitch_half))
        tooth_half = max(2.5, LAUNCH_SPEC.od * 0.07) / 2.0
        contact_offset = (
            MAX_LAUNCH_WIRE_MM / 2.0
            + coil_growth.DEFAULT_POLICY.opening_edge_clearance_mm
        )
        lateral = max(0.0, R3_MM - (tooth_half + contact_offset))
        connector_base = (
            radial_start + R3_MM
            + cap.s_bend_forward_mm(lateral)
            + cap.CAP_RADIAL_STRAIGHT_ALLOWANCE_MM
        )
        base = max(ligament_base, neighbor_base, connector_base)
        outer = (
            base + radial_span + R3_MM
            + MAX_LAUNCH_WIRE_MM / 2.0 + wall / 2.0
        )
        wall_rows.append({
            "wall_mm": wall,
            "pad_half_width_mm": pad_half_width,
            "minimum_base_center_radius_mm": base,
            "minimum_outer_radius_mm": outer,
            "minimum_outer_diameter_mm": 2.0 * outer,
            "bound_kind": "optimistic lower bound; zero packing profile span",
        })

    gates = {
        "OD65_stack20_nominal_50_turn_job": nominal["status"] == "PASS",
        "maximum_0p5_wire_has_open_slot_access": (
            bool(maximum_wire["slot_access"]["ok"])
            and bool(maximum_wire["slot_opening"]["ok"])
        ),
        "maximum_0p5_wire_50_turn_fill": maximum_wire["status"] != "FAIL",
        "current_R45_clears_required_launch_radius": (
            PARAMS.flyer_tip_r >= required_flyer_radius
        ),
        "cap_to_ER11_chuck_clearance_2mm": (
            chuck_clearance >= DYNAMIC_CLEARANCE_MM
        ),
        "settings_derivation_accepts_job": bool(cfg),
    }
    return {
        "status": "PASS" if all(gates.values()) else "MIXED",
        "gates": gates,
        "spec": {
            "od_mm": LAUNCH_SPEC.od,
            "stack_mm": LAUNCH_SPEC.stack,
            "shaft_d_mm": LAUNCH_SPEC.shaft_d,
            "slots": LAUNCH_SPEC.slots,
            "turns": LAUNCH_SPEC.turns,
        },
        "nominal_wire_job": {
            "wire_mm": LAUNCH_SPEC.wire_d,
            "status": nominal["status"],
            "gross_slot_fill": nominal["packing"]["gross_slot_fill"],
            "accessible_radial_span_mm": nominal["slot_access"][
                "accessible_radial_span_mm"
            ],
        },
        "maximum_wire_job": {
            "wire_mm": MAX_LAUNCH_WIRE_MM,
            "status_at_50_turns": maximum_wire["status"],
            "gross_slot_fill": maximum_wire["packing"]["gross_slot_fill"],
            "maximum_turns_at_hard_fill": maximum_wire["packing"][
                "max_turns_at_maximum_fill"
            ],
            "slot_access_ok": maximum_wire["slot_access"]["ok"],
            "slot_opening_ok": maximum_wire["slot_opening"]["ok"],
            "note": (
                "0.50 mm wire is physically accessible, but 50 turns cannot "
                "fit this stator at the hard fill limit; this is a job/stator "
                "capacity limit, not a guide-cap mouth failure."
            ),
        },
        "required_flyer_tip_radius_mm": required_flyer_radius,
        "cap_to_ER11_chuck_radial_clearance_mm": chuck_clearance,
        "cap_radial_bounds": wall_rows,
        "generated_M0_wind_range_rad": [
            cfg["motor"]["M0"]["wind_range_start"],
            cfg["motor"]["M0"]["wind_range_end"],
        ],
    }


def material_audit() -> dict[str, Any]:
    return {
        "status": "UNQUALIFIED",
        "release_gate": False,
        "wall_options": [
            {
                "wall_mm": 0.50,
                "material": "injection-molded Ryton PPS candidate",
                "geometry_plausible": True,
                "ready_for_project_printer": False,
                "ready_to_order": False,
                "reason": (
                    "Solvay documents 0.38-0.51 mm PPS applications and a "
                    "high-flow 0.3 mm stator-insulator grade, but this 24-rib "
                    "part still needs resin/flow, gate, weld-line, shrinkage, "
                    "flash, dielectric, finish and abrasion qualification."
                ),
            },
            {
                "wall_mm": 1.00,
                "material": "machined or molded unfilled PEEK prototype",
                "geometry_plausible": True,
                "ready_for_project_printer": False,
                "ready_to_order": False,
                "reason": (
                    "Victrex gives about 1 mm as the minimum for unfilled "
                    "molded PEEK. The thicker wall enlarges the collision "
                    "envelope and does not repair the spoke intersection."
                ),
            },
        ],
        "required_before_release": [
            "supplier DFM and selected resin/grade",
            "polished wire-contact surface Ra <=0.4 um",
            "full-tension reversal abrasion coupon with production wire",
            "dielectric test before and after winding/thermal cycling",
            "retention and varnish/impregnation compatibility",
        ],
        "sources": [SOURCE_PPS_URL, SOURCE_PPS_STATOR_URL, SOURCE_PEEK_URL],
    }


def motor_envelope() -> dict[str, Any]:
    """Conservative radius-extension bound for the selected M2 drive.

    The bound pessimistically translates every gram of the existing printed
    arm plus ceramic guide by the full radius increase, adds a solid 14 x 8 mm
    PETG spoke extension, and adds enough point counterweight at R25 to balance
    that entire first-moment upper bound.  Real source-level redesign would be
    lighter; this is intentionally a no-CAD screening bound.
    """

    loads = json.loads(LOADS_REPORT.read_text())
    parts = {row["part"]: row for row in loads["flyer"]["parts"]}
    moving_mass_g = (
        float(parts["retained_arm"]["mass_g"])
        + float(parts["flyer_PEEK_guide"]["mass_g"])
    )
    moving_i_gmm2 = (
        float(parts["retained_arm"]["izz_gmm2"])
        + float(parts["flyer_PEEK_guide"]["izz_gmm2"])
    )
    rms_radius = math.sqrt(moving_i_gmm2 / moving_mass_g)
    baseline_i = float(loads["flyer"]["izz_kgm2"])
    baseline_outer_spoke = SPOKE_Y_MAX_COMMON_MM
    linear_mass_g_per_mm = (
        PARAMS.flyer_arm_w
        * (SPOKE_Z1_MM - SPOKE_Z0_MM)
        * 1.27 / 1000.0
    )
    alpha = 200.0
    energy_torque = 2.0 * (1.5 * 10.0 * 0.060 / (2.0 * math.pi))
    motor_torque_300 = 0.630
    pulley_capacity = float(PARAMS.m2_motor_pulley_capacity_nm)
    rows = []
    for radius in TIP_RADII_MM:
        delta = radius - PARAMS.flyer_tip_r
        translated_i_gmm2 = moving_mass_g * (
            2.0 * rms_radius * delta + delta * delta
        )
        new_outer = baseline_outer_spoke + delta
        added_bar_i_gmm2 = linear_mass_g_per_mm / 3.0 * (
            new_outer ** 3 - baseline_outer_spoke ** 3
        )
        added_bar_moment_gmm = linear_mass_g_per_mm / 2.0 * (
            new_outer ** 2 - baseline_outer_spoke ** 2
        )
        translated_moment_upper = moving_mass_g * delta
        balance_mass_g = (
            translated_moment_upper + added_bar_moment_gmm
        ) / PARAMS.counterweight_r
        balance_i_gmm2 = balance_mass_g * PARAMS.counterweight_r ** 2
        inertia = baseline_i + 1e-9 * (
            translated_i_gmm2 + added_bar_i_gmm2 + balance_i_gmm2
        )
        torque = energy_torque + inertia * alpha
        motor_margin = motor_torque_300 / torque
        pulley_margin = pulley_capacity / torque
        rotating_outer_radius = radius + (
            wire_geometry.TIP_GUIDE_MAJOR_RADIUS
            + wire_geometry.TIP_GUIDE_TUBE_RADIUS
        )
        gates = {
            "selected_M2_margin_2x_at_300rpm": motor_margin >= 2.0,
            "selected_pulley_margin_2x": pulley_margin >= 2.0,
            "inside_300mm_machine_width": (
                rotating_outer_radius + DYNAMIC_CLEARANCE_MM
                <= PARAMS.frame_w / 2.0
            ),
        }
        rows.append({
            "tip_radius_mm": radius,
            "bounded_inertia_kgm2": inertia,
            "bounded_added_balance_mass_g": balance_mass_g,
            "required_torque_nm": torque,
            "selected_M2_margin": motor_margin,
            "selected_pulley_margin": pulley_margin,
            "rotating_outer_radius_mm": rotating_outer_radius,
            "gates": gates,
            "status": "PASS" if all(gates.values()) else "FAIL",
        })
    return {
        "status": "PASS" if all(row["status"] == "PASS" for row in rows)
        else "MIXED",
        "bound": (
            "pessimistic translated arm+guide, solid PETG spoke extension, "
            "and point counterweight; geometry screening only"
        ),
        "selected_motor": loads["motors"]["m2"],
        "selected_pulley": loads["m2"]["pulley"]["selection"],
        "candidates": rows,
    }


def candidate_sweep(witness: dict[str, Any], launch: dict[str, Any],
                    motor: dict[str, Any]
                    ) -> dict[str, Any]:
    rows = []
    required_radius = float(launch["required_flyer_tip_radius_mm"])
    motor_by_radius = {
        row["tip_radius_mm"]: row for row in motor["candidates"]
    }
    for radius in TIP_RADII_MM:
        for guide_z in TIP_GUIDE_Z_MM:
            shaft_clearance = (
                MAX_STACK_SHAFT_TIP_Z_MM
                - (guide_z + wire_geometry.FLYER_ELBOW_BODY_RADIUS)
            )
            gates = {
                "launch_radius": radius >= required_radius,
                "tip_elbow_to_max_stack_shaft_2mm": (
                    shaft_clearance >= DYNAMIC_CLEARANCE_MM
                ),
                "R3_tip_guide": (
                    wire_geometry.tip_guide_spec()[
                        "minimum_wire_center_bend_radius_mm"
                    ] >= R3_MM
                ),
                "reasonable_M2_motor_and_pulley_envelope": (
                    motor_by_radius[radius]["status"] == "PASS"
                ),
                "cap_to_common_spoke_2mm": witness["status"] == "PASS",
                "raw_command_stream_unchanged": True,
            }
            rows.append({
                "tip_radius_mm": radius,
                "tip_guide_z_mm": guide_z,
                "tip_elbow_to_max_stack_shaft_clearance_mm": shaft_clearance,
                "gates": gates,
                "status": "PASS" if all(gates.values()) else "FAIL",
            })
    pass_count = sum(row["status"] == "PASS" for row in rows)
    return {
        "status": "PASS" if pass_count else "NO_PASS",
        "candidate_count": len(rows),
        "pass_count": pass_count,
        "radii_mm": list(TIP_RADII_MM),
        "tip_guide_z_mm": list(TIP_GUIDE_Z_MM),
        "larger_flyer_clears_cap": False,
        "reason": (
            "Every candidate retains the exact positive-volume common-spoke "
            "witness. Radius extension and guide-Z relocation do not remove it."
        ),
        "candidates": rows,
    }


def analyze() -> dict[str, Any]:
    raw = raw_contract()
    witness = exact_common_spoke_witness()
    semantics = stored_route_contact_semantics()
    launch = launch_envelope()
    materials = material_audit()
    motor = motor_envelope()
    sweep = candidate_sweep(witness, launch, motor)
    gates = {
        "unmodified_raw_cycle_bound": raw["status"] == "PASS",
        "bounded_tip_radius_and_Z_sweep_has_candidate": (
            sweep["status"] == "PASS"
        ),
        "launch_OD65_stack20_geometry": (
            launch["gates"]["OD65_stack20_nominal_50_turn_job"]
            and launch["gates"]["current_R45_clears_required_launch_radius"]
            and launch["gates"]["cap_to_ER11_chuck_clearance_2mm"]
        ),
        "open_slot_access_for_0p5_wire": launch["gates"][
            "maximum_0p5_wire_has_open_slot_access"
        ],
        "reasonable_motor_envelope": motor["status"] == "PASS",
        "R3_contact_surface": math.isclose(
            cap.HORN_CONTACT_SURFACE_RADIUS_MM
            + cap.MAXIMUM_LAUNCH_WIRE_RADIUS_MM,
            R3_MM,
            abs_tol=1e-12,
        ),
        "stored_route_family_has_no_actual_crossing": (
            semantics["status"] == "PASS"
        ),
        "material_and_finish_released": materials["release_gate"],
    }
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "PASS" if all(gates.values()) else "DESIGN_NO_GO",
        "release_authorized": False,
        "assembly_integration_authorized": False,
        "cad_brief": {
            "task_type": "bounded source-level geometry inspection",
            "model": "existing permanent cap plus radius/Z-only flyer change",
            "units": "millimetres",
            "coordinates": (
                "machine Z flyer axis; flyer rotates in XY; stator-local +Z "
                "maps to machine +Y"
            ),
            "allowed_changes": ["flyer tip radius", "tip-guide center Z"],
            "preserved": [
                "radial flyer spoke/window",
                "unmodified Aotenjo raw command stream",
                "24-slot, 50-turn default winding cycle",
            ],
            "validation_targets": [
                "2 mm dynamic rigid clearance",
                "R3 wire contact",
                "OD65 x stack20 launch envelope",
                "0.5 mm wire slot access",
                "ER11 chuck clearance",
                "material/wall release basis",
            ],
            "output": str(JSON_OUT),
        },
        "gates": gates,
        "raw_contract": raw,
        "exact_common_spoke_witness": witness,
        "stored_route_contact_semantics": semantics,
        "launch_envelope": launch,
        "material_and_finish": materials,
        "motor_envelope": motor,
        "bounded_sweep": sweep,
        "decision": (
            "Do not integrate the cap or a larger/repositioned flyer tip. "
            "The cap intersects the unchanged load-carrying spoke with "
            "positive volume at a required flyer pose, so no radius/Z-only "
            "candidate can meet the 2 mm gate. The old route family also "
            "contains true zero-distance centerline crossings, and neither "
            "the 0.50 mm PPS molding nor its wire-contact finish is released."
        ),
        "successor_boundary": (
            "Recovery would require a wider/open or non-radial flyer arm and "
            "a separately proven noncrossing cap corridor. That is a new "
            "mechanism, not the requested minimal radius/Z redesign."
        ),
        "limitations": [
            "No exact strand neatness is required or claimed.",
            "The OD65 cap diameter values are optimistic analytical lower bounds.",
            "No production STEP is generated because a controlling rigid gate fails.",
        ],
        "source_hashes": {
            "GOAL.md": _sha256(ROOT.parent / "GOAL.md"),
            "cad/stator_winding_guide_cap.py": _sha256(
                CAD / "stator_winding_guide_cap.py"
            ),
            "sim/stator_winding_guide_cap_study.py": _sha256(
                HERE / "stator_winding_guide_cap_study.py"
            ),
            "out/reports/stator_winding_guide_cap.json": _sha256(CAP_REPORT),
            "out/reports/upstream_current_raw_cycle.json": _sha256(
                RAW_CYCLE_REPORT
            ),
            "out/reports/loads.json": _sha256(LOADS_REPORT),
            "sim/permanent_cap_flyer_recovery_study.py": _sha256(
                Path(__file__)
            ),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def write(report: dict[str, Any]) -> None:
    REPORTS.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    witness = report["exact_common_spoke_witness"]
    sweep = report["bounded_sweep"]
    launch = report["launch_envelope"]
    material = report["material_and_finish"]
    motor = report["motor_envelope"]
    semantics = report["stored_route_contact_semantics"]
    wall_lines = [
        (
            f"- {row['wall_mm']:.2f} mm wall: optimistic cap OD >= "
            f"{row['minimum_outer_diameter_mm']:.3f} mm"
        )
        for row in launch["cap_radial_bounds"]
    ]
    crossing_lines = [
        (
            f"- {row['class']}: {row['raw_centerline_distance_mm']:.6g} mm "
            f"vs {row['required_for_adjacent_contact_mm']:.5f} mm required — "
            f"{row['classification']}"
        )
        for row in semantics["cases"]
    ]
    lines = [
        "# Permanent cap + minimal flyer recovery audit",
        "",
        f"Status: **{report['status']}**. Release and assembly integration are false.",
        "",
        "## Controlling rigid witness",
        "",
        (
            f"The exact tooth-0 cap pad intersects the unchanged flyer-spoke "
            f"rails by **{witness['intersection_volume_mm3']:.6f} mm^3** in "
            f"{witness['intersection_solid_count']} solids at M2=0. Required "
            f"clearance is {DYNAMIC_CLEARANCE_MM:.1f} mm."
        ),
        (
            f"The bounded sweep checked {sweep['candidate_count']} radius/Z "
            f"combinations ({min(sweep['radii_mm']):.0f}.."
            f"{max(sweep['radii_mm']):.0f} mm radius); {sweep['pass_count']} pass."
        ),
        "A larger flyer does not clear the cap because the collision is inside the unchanged radial spoke, not at the eyelet.",
        "",
        "## Wire-contact semantics",
        "",
        *crossing_lines,
        "",
        semantics["interpretation"],
        "",
        "## Launch envelope",
        "",
        (
            f"OD65 x stack20 with {launch['nominal_wire_job']['wire_mm']:.5f} "
            f"mm wire and 50 turns is {launch['nominal_wire_job']['status']}; "
            f"the machine needs only R{launch['required_flyer_tip_radius_mm']:.3f} mm."
        ),
        (
            f"ER11 radial clearance is "
            f"{launch['cap_to_ER11_chuck_radial_clearance_mm']:.3f} mm."
        ),
        (
            f"The 0.50 mm wire has open slot access, but 50 turns reaches "
            f"{launch['maximum_wire_job']['gross_slot_fill']:.1%} fill and is "
            f"therefore a job-capacity FAIL; at the hard limit the stator "
            f"accepts {launch['maximum_wire_job']['maximum_turns_at_hard_fill']} turns."
        ),
        *wall_lines,
        "",
        "## Motor envelope",
        "",
        (
            f"The conservative R45..R60 screening bound is {motor['status']}; "
            f"the worst selected-M2 and pulley margins are "
            f"{min(row['selected_M2_margin'] for row in motor['candidates']):.3f}x "
            f"and {min(row['selected_pulley_margin'] for row in motor['candidates']):.3f}x."
        ),
        "",
        "## Material boundary",
        "",
        (
            "A 0.50 mm injection-molded PPS wall is geometrically plausible, "
            "but not printer-ready or released. The 1.00 mm PEEK alternative "
            "only enlarges the collision envelope."
        ),
        f"Material status: **{material['status']}**.",
        "",
        "## Decision",
        "",
        report["decision"],
        "",
        report["successor_boundary"],
        "",
        f"Raw capture: `{report['raw_contract']['capture_sha256']}`",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ]
    MD_OUT.write_text("\n".join(lines))


def main() -> int:
    report = analyze()
    write(report)
    print(json.dumps({
        "status": report["status"],
        "intersection_volume_mm3": report[
            "exact_common_spoke_witness"
        ]["intersection_volume_mm3"],
        "sweep_pass_count": report["bounded_sweep"]["pass_count"],
        "report_sha256": report["report_sha256"],
    }, indent=2))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
