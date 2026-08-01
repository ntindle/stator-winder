"""Bounded study of an M0-following, radially retracting full shroud.

The candidate is the conventional two-face architecture, adapted clean-room
to the current machine: an upper and a lower polished forming paddle travel
with the M0 stator carriage throughout the complete winding traverse.  After
the shallow winding endpoint, a frame-fixed lost-motion stop holds the two
paddles while M0 continues to the indexing pose.  The resulting one-for-one
relative radial motion extracts both paddles from the active tooth before any
M1 index or shaft-wrap motion.  Return springs and a positive deployed shoulder
close and lock the paddles when M0 re-enters the winding range.

This is deliberately a fail-closed analytical study.  It distinguishes the
M0 tracking motion from the controlling extraction and wire-path questions.
The available post-range M0 travel cannot extract the paddles beyond the
conservative completed-coil envelope with the required rigid clearance.  In
addition, a fully retractable former cannot leave the first deposited turn
with a physical R3 support, and a standard convex R3 crown cannot fit between
the two lined coil sides of the narrow default tooth.  The already-proven
optimistic non-convex L-R-L escape crosses the active tooth sector and
therefore cannot be certified against the unknown-but-capacity-bounded
completed-neighbour aggregate.  No exact layer order is required or inferred.

Run directly to write ``out/reports/m0_following_full_shroud.{json,md}``.
No CAD, controller, capture, BOM, or production allow-list is modified.
"""

from __future__ import annotations

from collections import Counter
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
GOAL = ROOT.parent / "GOAL.md"
SCOPE_REPORT = REPORTS / "r3_bend_scope_feasibility.json"
PHASE_REPORT = REPORTS / "phase_aware_progressive_wire_audit.json"
JSON_OUT = REPORTS / "m0_following_full_shroud.json"
MD_OUT = REPORTS / "m0_following_full_shroud.md"

for path in (CAD, HERE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import coil_growth  # noqa: E402
from params import DEFAULT_STATOR, PARAMS  # noqa: E402
from traj import Timeline, load_events, winding_windows  # noqa: E402


SCHEMA = "m0-following-full-shroud-study/v1"
EXPECTED_CAPTURE_SHA256 = (
    "350210c36550ae2c22e5352675aee6bd2eebb18325f3b5f45c7cb35d6a314958"
)
EXPECTED_SETTINGS_SHA256 = (
    "6c4dbd8287c14dfaf98203a3733b743dd1ea04a39abe8f36f7be502d641cf4d1"
)
LINER_THICKNESS_MM = 0.127
PHYSICAL_SUPPORT_RADIUS_MM = 3.0
RIGID_CLEARANCE_MM = 2.0
RETRACTION_TOLERANCE_RESERVE_MM = 0.25
CONSERVATIVE_COMPLETED_COIL_GROWTH_MM = float(PARAMS.wire_bundle_allow)
RETURN_SPRING_FORCE_PER_HALF_N = 1.5
CONSERVATIVE_EXTRACTION_DRAG_N = 10.0
DESIGN_WIRE_TENSION_N = 10.0
M0_EXISTING_AXIAL_FORCE_N = 10.4
M0_EXISTING_FORCE_MARGIN = 12.5
M0_ACCELERATION_RAD_S2 = 50.0
POSITION_TOLERANCE_RAD = 0.0035


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: dict[str, Any]) -> str:
    body = deepcopy(payload)
    body.pop("report_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} does not contain a JSON object")
    return value


def _require_identity(meta: dict[str, Any]) -> None:
    expected = {
        "capture_schema": 4,
        "controller_mode": "upstream",
        "controller_adapter_sha256": None,
        "settings_sha256": EXPECTED_SETTINGS_SHA256,
        "teeth_count": 24,
        "turns": 50,
    }
    for key, wanted in expected.items():
        if meta.get(key) != wanted:
            raise RuntimeError(
                f"canonical raw identity drifted at {key}: "
                f"{meta.get(key)!r} != {wanted!r}"
            )
    job = meta.get("job", {})
    job_expected = {
        "od_mm": 46.0,
        "stack_mm": 15.0,
        "slots": 24,
        "wire_finished_d_mm": 0.22352,
    }
    for key, wanted in job_expected.items():
        if not math.isclose(float(job.get(key, math.nan)), wanted,
                            rel_tol=0.0, abs_tol=1.0e-12):
            raise RuntimeError(f"canonical raw job drifted at {key}")
    if _sha256(CAPTURE) != EXPECTED_CAPTURE_SHA256:
        raise RuntimeError("canonical raw capture SHA-256 drifted")


def _shroud_state(axis_z_mm: float, shallow_wind_axis_z_mm: float,
                  required_stroke_mm: float) -> tuple[str, float]:
    relative = max(0.0, float(axis_z_mm) - shallow_wind_axis_z_mm)
    stroke = min(relative, required_stroke_mm)
    if relative <= 1.0e-9:
        return "DEPLOYED_TRACKING", stroke
    if relative < required_stroke_mm - 1.0e-9:
        return "RADIAL_EXTRACTION", stroke
    return "RETRACTED_DWELL", stroke


def capture_and_motion_study() -> dict[str, Any]:
    events = load_events(CAPTURE)
    meta = next(row for row in events if row.get("e") == "meta")
    _require_identity(meta)
    timeline = Timeline(events)
    windows = winding_windows(events)
    if len(windows) != 24:
        raise RuntimeError("raw capture no longer contains 24 winding passes")

    wind_deep, wind_shallow = map(float, meta["m0_wind_range"])
    wind_axis_deep = float(PARAMS.stator_axis_z(wind_deep))
    wind_axis_shallow = float(PARAMS.stator_axis_z(wind_shallow))
    safe_m0 = float(meta["m1_rotating_position"])
    safe_axis = float(PARAMS.stator_axis_z(safe_m0))
    radial_start, radial_end = map(
        float, meta["job"]["radial_winding_span_mm"])

    # A face shroud begins at the innermost supported radial locus.  At index
    # its leading edge must be outside the conservative completed-coil radius
    # (bare OD/2 plus the project's 3 mm bundle allowance) by 2 mm plus a
    # separate 0.25 mm machining/registration reserve.  Clearing only bare
    # OD/2 is invalid on every later index.  The fixed stop supplies direct
    # one-for-one relative motion, so this is not a high-pressure-angle wedge.
    completed_coil_radius = (
        float(DEFAULT_STATOR.od) / 2.0
        + CONSERVATIVE_COMPLETED_COIL_GROWTH_MM
    )
    required_stroke = (
        completed_coil_radius
        + RIGID_CLEARANCE_MM
        + RETRACTION_TOLERANCE_RESERVE_MM
        - radial_start
    )
    available_stroke = safe_axis - wind_axis_shallow
    dwell = available_stroke - required_stroke
    speed = float(meta["velocities"][0]) * float(PARAMS.mm_per_rad)
    required_time = required_stroke / speed
    available_time = available_stroke / speed

    start_rows = []
    for window in windows:
        time_s = float(window["motionStart"])
        m0 = float(timeline.axes[0].pos_at(time_s))
        axis = float(PARAMS.stator_axis_z(m0))
        state, stroke = _shroud_state(
            axis, wind_axis_shallow, required_stroke)
        start_rows.append({
            "pass_index": int(window["passIndex"]),
            "tooth": int(window["tooth"]),
            "motion_sign": 1 if bool(window["clockwise"]) else -1,
            "time_s": time_s,
            "m0_rad": m0,
            "axis_z_mm": axis,
            "state": state,
            "relative_stroke_mm": stroke,
        })

    m1_rows = []
    for event in events:
        if event.get("e") != "cmd" or int(event.get("m", -1)) != 1:
            continue
        time_s = float(event["t"])
        current = float(timeline.axes[1].pos_at(time_s))
        target = float(event.get("model_target", event["a"]))
        if abs(target - current) <= POSITION_TOLERANCE_RAD:
            continue
        m0 = float(timeline.axes[0].pos_at(time_s))
        axis = float(PARAMS.stator_axis_z(m0))
        state, stroke = _shroud_state(
            axis, wind_axis_shallow, required_stroke)
        leading_radius = radial_start + stroke
        m1_rows.append({
            "time_s": time_s,
            "m0_rad": m0,
            "m1_current_rad": current,
            "m1_target_rad": target,
            "state": state,
            "relative_stroke_mm": stroke,
            "shroud_leading_radius_mm": leading_radius,
            "nominal_clearance_to_completed_coil_envelope_mm": (
                leading_radius - completed_coil_radius
            ),
            "tolerance_reserved_clearance_mm": (
                leading_radius - completed_coil_radius
                - RETRACTION_TOLERANCE_RESERVE_MM
            ),
        })
    if not m1_rows:
        raise RuntimeError("raw capture has no physical M1 moves")

    wraps = [row for row in events
             if row.get("e") == "wind_wire_around_shaft"]
    if len(wraps) != 2:
        raise RuntimeError("raw capture no longer contains two shaft wraps")

    tooth_order = [int(window["tooth"]) for window in windows]
    completed: set[int] = set()
    neighbor_history = []
    for pass_index, tooth in enumerate(tooth_order):
        neighbors = ((tooth - 1) % 24, (tooth + 1) % 24)
        prior = [value for value in neighbors if value in completed]
        neighbor_history.append({
            "pass_index": pass_index,
            "tooth": tooth,
            "completed_neighbor_count": len(prior),
            "completed_neighbors": prior,
        })
        completed.add(tooth)

    minimum_m1_clearance = min(
        row["tolerance_reserved_clearance_mm"] for row in m1_rows)
    return {
        "status": "PASS" if (
            all(row["state"] == "DEPLOYED_TRACKING"
                for row in start_rows)
            and all(row["state"] == "RETRACTED_DWELL"
                    for row in m1_rows)
            and minimum_m1_clearance >= RIGID_CLEARANCE_MM - 1.0e-9
            and dwell > 0.0
        ) else "FAIL",
        "raw_capture": {
            "path": CAPTURE.relative_to(ROOT).as_posix(),
            "sha256": _sha256(CAPTURE),
            "winder_commit": meta["winder_commit"],
            "settings_sha256": meta["settings_sha256"],
            "controller_mode": meta["controller_mode"],
            "adapter_sha256": meta["controller_adapter_sha256"],
            "pass_count": len(windows),
            "motion_sign_counts": dict(sorted(Counter(
                row["motion_sign"] for row in start_rows).items())),
            "shaft_wrap_count": len(wraps),
            "physical_M1_move_count": len(m1_rows),
        },
        "architecture": {
            "halves": 2,
            "half_locations": ["positive stack face", "negative stack face"],
            "carrier_input": "M0 carriage; no new commanded axis",
            "deployed_lock": "positive shoulder, not spring friction",
            "extraction_input": (
                "frame-fixed collinear lost-motion stop; one-for-one radial "
                "slide after the shallow winding endpoint"
            ),
            "return": "two independent return springs on a common yoke",
            "failure_state": (
                "normally extracted outside the winding range; M1 permission "
                "requires the mechanical retracted-dwell position"
            ),
        },
        "wind_range_rad": [wind_deep, wind_shallow],
        "wind_axis_z_range_mm": [wind_axis_deep, wind_axis_shallow],
        "tracking_stroke_mm": wind_axis_shallow - wind_axis_deep,
        "safe_M1_pose_rad": safe_m0,
        "safe_M1_axis_z_mm": safe_axis,
        "radial_support_span_mm": [radial_start, radial_end],
        "bare_stator_radius_mm": float(DEFAULT_STATOR.od) / 2.0,
        "conservative_completed_coil_growth_mm": (
            CONSERVATIVE_COMPLETED_COIL_GROWTH_MM),
        "conservative_completed_coil_radius_mm": completed_coil_radius,
        "required_relative_extraction_mm": required_stroke,
        "available_relative_extraction_mm": available_stroke,
        "extraction_stroke_balance_mm": dwell,
        "extraction_stroke_shortfall_mm": max(0.0, -dwell),
        "M0_slide_speed_mm_s": speed,
        "required_extraction_time_s": required_time,
        "available_extraction_time_s": available_time,
        "extraction_time_balance_s": available_time - required_time,
        "extraction_time_shortfall_s": max(
            0.0, required_time - available_time),
        "minimum_tolerance_reserved_clearance_at_M1_motion_mm": (
            minimum_m1_clearance
        ),
        "all_winding_starts_deployed": all(
            row["state"] == "DEPLOYED_TRACKING" for row in start_rows),
        "all_M1_moves_retracted": all(
            row["state"] == "RETRACTED_DWELL" for row in m1_rows),
        "winding_start_rows": start_rows,
        "M1_motion_rows": m1_rows,
        "neighbor_history": neighbor_history,
        "neighbor_case_counts": dict(sorted(Counter(
            row["completed_neighbor_count"] for row in neighbor_history
        ).items())),
    }


def route_and_contact_study(motion: dict[str, Any]) -> dict[str, Any]:
    slot = coil_growth.slot_geometry(DEFAULT_STATOR)
    scope = _load_json(SCOPE_REPORT)
    phase = _load_json(PHASE_REPORT)
    if (scope.get("schema") != "r3-bend-scope-feasibility/v1"
            or scope.get("status") != "ADVISORY_COMPATIBLE"):
        raise RuntimeError("R3 scope witness is missing or drifted")
    if phase.get("schema") != "phase-aware-progressive-wire-audit/v1":
        raise RuntimeError("phase-aware contact report is missing or drifted")

    wire_r = float(DEFAULT_STATOR.wire_d) / 2.0
    centre_core = LINER_THICKNESS_MM + wire_r
    half_neck = float(slot["tooth_neck_width_mm"]) / 2.0
    side_half_span = half_neck + centre_core

    # The requested physical R3 surface puts the wire centre on at least
    # R(3 + wire_r).  A conventional convex top/bottom crown joins two
    # antiparallel slot-side tangents.  Two quarter turns need a side spacing
    # of at least 2R.  The default lined tooth provides less than that.
    support_wire_centre_radius = PHYSICAL_SUPPORT_RADIUS_MM + wire_r
    available_side_spacing = 2.0 * side_half_span
    required_convex_spacing = 2.0 * support_wire_centre_radius
    convex_shortfall = required_convex_spacing - available_side_spacing

    adjacency = scope["adjacent_tooth_pitch"]
    optimistic_lrl_half_width = float(
        adjacency["maximum_crown_half_width_mm"])
    minimum_sector_half_width = float(
        adjacency["minimum_tooth_sector_half_width_over_witness_mm"])
    sector_intrusion = optimistic_lrl_half_width - minimum_sector_half_width

    neighbor_counts = motion["neighbor_case_counts"]
    passes_with_prior_neighbor = sum(
        count for key, count in neighbor_counts.items() if int(key) >= 1
    )
    passes_with_both_neighbors = int(neighbor_counts.get(2, 0))
    tight_corner = centre_core

    checks = {
        "deployed polished surface gives wire-centre R3": {
            "ok": support_wire_centre_radius >= PARAMS.min_bend_radius,
            "physical_surface_radius_mm": PHYSICAL_SUPPORT_RADIUS_MM,
            "wire_center_radius_mm": support_wire_centre_radius,
            "requirement_mm": PARAMS.min_bend_radius,
        },
        "standard convex crown fits lined tooth side spacing": {
            "ok": convex_shortfall <= 1.0e-12,
            "available_lined_side_spacing_mm": available_side_spacing,
            "required_for_two_R3_quarter_turns_mm": required_convex_spacing,
            "shortfall_mm": convex_shortfall,
            "proof": (
                "a convex cap joining opposed axial slot-side tangents uses "
                "two 90-degree turns; each turn consumes its wire-centre "
                "radius in the tangential direction"
            ),
        },
        "optimistic nonconvex R3 escape remains in tooth sector": {
            "ok": sector_intrusion <= 1.0e-12,
            "minimum_sector_half_width_mm": minimum_sector_half_width,
            "optimistic_LRL_crown_half_width_mm": optimistic_lrl_half_width,
            "sector_intrusion_mm": sector_intrusion,
            "source": "r3_bend_scope_feasibility optimistic wire-centre R3 witness",
        },
        "withdrawn first turn retains physical R3 support": {
            "ok": tight_corner >= PARAMS.min_bend_radius,
            "supported_corner_after_full_extraction_mm": tight_corner,
            "requirement_mm": PARAMS.min_bend_radius,
            "shortfall_mm": PARAMS.min_bend_radius - tight_corner,
            "classification": (
                "forbidden unsupported core/liner conformity; the first turn "
                "has no parent copper and the shroud is physically absent"
            ),
        },
        "previous aggregate can be independently cleared": {
            "ok": sector_intrusion <= 1.0e-12 or passes_with_prior_neighbor == 0,
            "passes_with_at_least_one_completed_neighbor": (
                passes_with_prior_neighbor
            ),
            "passes_with_both_completed_neighbors": passes_with_both_neighbors,
            "finding": (
                "the raw order presents completed-neighbour aggregate while "
                "even the optimistic R3 escape leaves the active sector; "
                "nonpenetrating copper glide cannot be inferred from fill "
                "capacity or from hardware-only passive settling"
            ),
        },
    }
    return {
        "status": "FAIL" if not all(row["ok"] for row in checks.values())
        else "PASS",
        "decision": "FULLY_RETRACTABLE_STANDARD_SHROUD_HAS_NO_AUTHORIZED_R3_ROUTE",
        "model": {
            "active_support": (
                "two polished face paddles form one convex upper and one "
                "convex lower guide surface around the active tooth"
            ),
            "core_contact": (
                "only named shroud contact is allowed while deployed; steel "
                "is offset by liner plus the actual wire radius"
            ),
            "copper_contact": (
                "nonpenetrating contact may be support/glide; centreline or "
                "rigid-tool interpenetration is never waived"
            ),
            "neatness_policy": (
                "no exact layer order, strand centre, or deterministic "
                "settling claim is required"
            ),
        },
        "checks": checks,
        "current_phase_aware_baseline": {
            "status": phase.get("status"),
            "decision": phase.get("decision"),
            "classified_raw_loci": phase.get("capture_contract", {}).get(
                "state_count", 2400),
            "current_core_penetration_loci": phase.get(
                "paths", {}).get("nominal_ellipse_tangent", {}).get(
                    "core_liner", {}).get("penetration_locus_count", 300),
            "use": (
                "confirms the current unshrouded route is not a fallback; "
                "this study does not replace it with an unsupported path"
            ),
        },
        "contact_verdict": {
            "deployed_core_isolation": "GEOMETRICALLY_PLAUSIBLE_BUT_ROUTE_FAMILY_FAILS",
            "prior_active_copper": "ALLOWED_GLIDE_ONLY__NO_INTERPENETRATION_PROOF",
            "completed_neighbor_copper": "FAIL_SECTOR_INTRUSION",
            "completed_other_copper": "SECTOR_SEPARATED_IF_NEIGHBOR_GATE_CLOSED",
            "post_extraction_first_turn": "FAIL_UNSUPPORTED_R0.23876",
        },
    }


def capacity_rigid_load_build_study(motion: dict[str, Any]) -> dict[str, Any]:
    job = coil_growth.analyze_job(DEFAULT_STATOR)
    scope = _load_json(SCOPE_REPORT)
    cap = scope["lrl_crown_witness"]
    radial_start, radial_end = motion["radial_support_span_mm"]
    wind_axis_min, wind_axis_max = motion["wind_axis_z_range_mm"]

    # Conservative deployed AABB/annulus decomposition against the existing
    # flyer source dimensions.  The spoke is entirely behind z=-20; the
    # shroud is no farther rearward than axis_min-radial_end.  The front tip
    # cradle reaches inward to R-11.5, while the full optimistic shroud crown
    # stays inside its measured transverse radius.
    deployed_world_z_min = wind_axis_min - radial_end
    deployed_world_z_max = wind_axis_max - radial_start
    spoke_front_z = float(PARAMS.spoke_z[1])
    spoke_axial_clearance = deployed_world_z_min - spoke_front_z
    crown_half_width = float(
        scope["adjacent_tooth_pitch"]["maximum_crown_half_width_mm"])
    crown_axial = (
        float(DEFAULT_STATOR.stack) / 2.0
        + max(float(row["maximum_axial_rise_mm"])
              for row in cap["layer_bounds"])
    )
    shroud_transverse_radius = math.hypot(crown_half_width, crown_axial)
    tip_structure_inner_radius = (
        float(PARAMS.flyer_tip_r) - 11.5
    )
    tip_radial_clearance = tip_structure_inner_radius - shroud_transverse_radius
    chuck_clearance = radial_start - 9.75  # exact selected ER11 max neck radius
    retracted_clearance = float(
        motion["minimum_tolerance_reserved_clearance_at_M1_motion_mm"])
    rigid_minimum = min(
        spoke_axial_clearance, tip_radial_clearance,
        chuck_clearance, retracted_clearance,
    )

    spring_force = 2.0 * RETURN_SPRING_FORCE_PER_HALF_N
    revised_m0_force = (
        M0_EXISTING_AXIAL_FORCE_N + spring_force
        + CONSERVATIVE_EXTRACTION_DRAG_N
    )
    equivalent_capacity = (
        M0_EXISTING_AXIAL_FORCE_N * M0_EXISTING_FORCE_MARGIN
    )
    revised_force_margin = equivalent_capacity / revised_m0_force
    maximum_contact_resultant = 2.0 * DESIGN_WIRE_TENSION_N

    return {
        "status": "PASS" if (
            job["status"] == "PASS"
            and rigid_minimum >= RIGID_CLEARANCE_MM - 1.0e-9
            and revised_force_margin >= 2.0
        ) else "FAIL",
        "slot_capacity": {
            "status": job["status"],
            "gross_slot_fill": job["packing"]["gross_slot_fill"],
            "hard_fill_limit": job["packing"]["maximum_slot_fill_limit"],
            "hard_limit_turn_capacity": job["packing"][
                "max_turns_at_maximum_fill"],
            "slot_opening_margin_mm": job["slot_opening"]["margin_mm"],
            "shroud_occupancy": (
                "face paddles remain outside the lamination slot section; "
                "no guide post or blade is credited with slot volume"
            ),
        },
        "rigid_clearance": {
            "method": (
                "conservative source-dimension annulus/AABB bounds; seated "
                "active-tooth support is an intentional-contact exception"
            ),
            "deployed_world_z_range_mm": [
                deployed_world_z_min, deployed_world_z_max],
            "spoke_axial_clearance_mm": spoke_axial_clearance,
            "tip_structure_radial_clearance_mm": tip_radial_clearance,
            "selected_ER11_chuck_clearance_mm": chuck_clearance,
            "retracted_M1_tolerance_reserved_clearance_mm": (
                retracted_clearance),
            "minimum_rigid_clearance_mm": rigid_minimum,
            "requirement_mm": RIGID_CLEARANCE_MM,
            "status": "PASS" if rigid_minimum >= RIGID_CLEARANCE_MM else "FAIL",
        },
        "loads_and_timing": {
            "design_wire_tension_N": DESIGN_WIRE_TENSION_N,
            "maximum_180deg_shroud_contact_resultant_N": (
                maximum_contact_resultant),
            "required_positive_stop_proof_load_N": (
                2.0 * maximum_contact_resultant),
            "return_spring_force_per_half_N": RETURN_SPRING_FORCE_PER_HALF_N,
            "conservative_extraction_drag_allowance_N": (
                CONSERVATIVE_EXTRACTION_DRAG_N),
            "existing_M0_axial_force_N": M0_EXISTING_AXIAL_FORCE_N,
            "revised_M0_axial_force_N": revised_m0_force,
            "revised_M0_force_margin": revised_force_margin,
            "maximum_relative_slide_speed_mm_s": motion["M0_slide_speed_mm_s"],
            "maximum_relative_slide_acceleration_mm_s2": (
                M0_ACCELERATION_RAD_S2 * PARAMS.mm_per_rad),
            "extraction_time_balance_s": motion[
                "extraction_time_balance_s"],
            "extraction_time_shortfall_s": motion[
                "extraction_time_shortfall_s"],
            "status": "PASS" if revised_force_margin >= 2.0 else "FAIL",
        },
        "manufacturing_contract": {
            "contact_insert": (
                "two replaceable CNC-machined unfilled PEEK face paddles; "
                "physical support radius >=3.00 mm; Ra <=0.4 um; all wire "
                "entry/exit edges R0.5 minimum"
            ),
            "minimum_contact_wall_mm": 0.8,
            "carrier": "metal common yoke on two preloaded radial slide pairs",
            "each_half_fasteners": (
                "2x ISO 4762 M3 screws plus 2x diameter-3 ground dowels; "
                "positive shoulder carries the 40 N proof load"
            ),
            "cam": (
                "frame-fixed hardened stop and roller-ended common pull yoke; "
                "collinear 1:1 lost motion, not a friction wedge"
            ),
            "springs": "2x independent 1.5 N return springs",
            "interlock": (
                "normally-closed retracted-dwell switch in series with M1 "
                "enable; deployed-seat switch in series with winding enable"
            ),
            "print_policy": (
                "contact paddles and locating yoke are not printed; optional "
                "guards may be printed after a production geometry exists"
            ),
            "fastener_and_material_release": False,
            "reason": "controlling R3/contact gates fail before CAD/BOM release",
        },
    }


def analyze() -> dict[str, Any]:
    motion = capture_and_motion_study()
    route = route_and_contact_study(motion)
    supporting = capacity_rigid_load_build_study(motion)
    gates = {
        "canonical_unmodified_raw_capture_bound": (
            motion["raw_capture"]["sha256"] == EXPECTED_CAPTURE_SHA256),
        "all_24_winding_starts_shroud_deployed": (
            motion["all_winding_starts_deployed"]),
        "all_raw_M1_and_shaft_wrap_motion_shroud_retracted": (
            motion["all_M1_moves_retracted"]
            and motion["raw_capture"]["shaft_wrap_count"] == 2),
        "M0_tracking_extraction_timing_and_clearance": motion["status"] == "PASS",
        "slot_capacity": supporting["slot_capacity"]["status"] == "PASS",
        "two_millimetre_rigid_clearance": (
            supporting["rigid_clearance"]["status"] == "PASS"),
        "cam_force_and_M0_margin": (
            supporting["loads_and_timing"]["status"] == "PASS"),
        "standard_full_shroud_R3_path": route["status"] == "PASS",
        "withdrawn_first_turn_remains_physically_R3_supported": route[
            "checks"]["withdrawn first turn retains physical R3 support"]["ok"],
        "previous_aggregate_noninterpenetration": route[
            "checks"]["previous aggregate can be independently cleared"]["ok"],
    }
    production = all(gates.values())
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "PASS" if production else "DESIGN_NO_GO",
        "decision": (
            "AUTHORIZE_ISOLATED_CAD" if production else
            "REJECT_FULLY_RETRACTABLE_STANDARD_SHROUD_BEFORE_CAD"
        ),
        "production_authorized": production,
        "integration_authorized": production,
        "isolated_CAD_authorized": production,
        "scope": {
            "candidate": (
                "two stack-face polished shroud paddles carried by M0 and "
                "radially extracted by fixed collinear lost motion"
            ),
            "job": (
                "OD46 / 24 slots / stack15 / finished wire 0.22352 mm / "
                "50 turns per tooth"
            ),
            "controller_changes": False,
            "extra_commanded_axes": 0,
            "exact_layer_order_required": False,
            "CAD_generated": False,
        },
        "motion": motion,
        "wire_route_and_contact": route,
        "capacity_rigid_load_build": supporting,
        "gates": gates,
        "controlling_failures": [
            name for name, passed in gates.items() if not passed
        ],
        "bounded_conclusion": (
            "The shroud can follow the M0 winding traverse, but the existing "
            "post-range retract is 1.766 mm short of clearing the conservative "
            "completed-coil envelope with the required rigid clearance and "
            "reserve. Independently, a standard convex R3 crown does not fit "
            "the lined default tooth; the known non-convex R3 escape intrudes "
            "into completed-neighbour sectors; and full extraction would leave "
            "the first turn on only the R0.23876 lined corner. A production "
            "successor needs more extraction authority and must retain a "
            "permanent R3 end support or prove a sector-confined spatial "
            "multi-track basket with complete aggregate clearance."
        ),
        "honest_limits": [
            "No exact passive layer order or winding neatness is claimed.",
            "Tension dynamics, sag, snagging, friction, enamel abrasion, and "
            "springback remain hardware-coupon questions.",
            "Those empirical limits cannot waive an explicit geometric R3 "
            "shortfall or an unproved rigid/copper interpenetration.",
            "The no-go is limited to this fully retractable standard convex "
            "two-face shroud; it is not a theorem against every 3D former."
        ],
        "source_hashes": {
            "GOAL.md": _sha256(GOAL),
            "out/capture/upstream_current_raw.jsonl": _sha256(CAPTURE),
            "cad/params.py": _sha256(CAD / "params.py"),
            "cad/coil_growth.py": _sha256(CAD / "coil_growth.py"),
            "sim/phase_aware_progressive_wire_audit.py": _sha256(
                HERE / "phase_aware_progressive_wire_audit.py"),
            "out/reports/phase_aware_progressive_wire_audit.json": _sha256(
                PHASE_REPORT),
            "sim/r3_bend_scope_feasibility.py": _sha256(
                HERE / "r3_bend_scope_feasibility.py"),
            "out/reports/r3_bend_scope_feasibility.json": _sha256(
                SCOPE_REPORT),
            "sim/m0_following_full_shroud_study.py": _sha256(Path(__file__)),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def render_markdown(report: dict[str, Any]) -> str:
    motion = report["motion"]
    route = report["wire_route_and_contact"]
    support = report["capacity_rigid_load_build"]
    convex = route["checks"][
        "standard convex crown fits lined tooth side spacing"]
    first = route["checks"][
        "withdrawn first turn retains physical R3 support"]
    aggregate = route["checks"][
        "previous aggregate can be independently cleared"]
    lines = [
        "# M0-following full winding-shroud study", "",
        f"**Status: {report['status']} — no CAD or hardware release.**", "",
        "The shroud follows the winding traverse, but the available extraction, "
        "rigid-clearance, and wire/support topology gates do not close. This "
        "result does not require deterministic layer order or neatness.", "",
        "## Raw motion and rigid mechanism", "",
        f"- Full M0 tracking traverse: {motion['tracking_stroke_mm']:.6f} mm.",
        f"- Available post-range extraction: "
        f"{motion['available_relative_extraction_mm']:.6f} mm; required "
        f"{motion['required_relative_extraction_mm']:.6f} mm; stroke shortfall "
        f"{motion['extraction_stroke_shortfall_mm']:.6f} mm; time shortfall "
        f"{motion['extraction_time_shortfall_s']:.6f} s.",
        f"- All 24 winding starts deployed: "
        f"{motion['all_winding_starts_deployed']}; all "
        f"{motion['raw_capture']['physical_M1_move_count']} physical M1 moves "
        f"retracted: {motion['all_M1_moves_retracted']}; shaft wraps: "
        f"{motion['raw_capture']['shaft_wrap_count']}.",
        f"- Conservative completed-coil radius: "
        f"{motion['conservative_completed_coil_radius_mm']:.3f} mm; minimum "
        f"tolerance-reserved M1 clearance: "
        f"{motion['minimum_tolerance_reserved_clearance_at_M1_motion_mm']:.6f} mm.",
        f"- Conservative rigid minimum: "
        f"{support['rigid_clearance']['minimum_rigid_clearance_mm']:.6f} mm; "
        f"revised M0 force margin: "
        f"{support['loads_and_timing']['revised_M0_force_margin']:.2f}x.", "",
        "## Exact controlling wire witnesses", "",
        f"1. A physical R3 surface gives wire-centre radius "
        f"{route['checks']['deployed polished surface gives wire-centre R3']['wire_center_radius_mm']:.5f} mm. "
        f"The lined tooth sides are only {convex['available_lined_side_spacing_mm']:.5f} mm apart, "
        f"while two convex quarter turns require {convex['required_for_two_R3_quarter_turns_mm']:.5f} mm; "
        f"shortfall {convex['shortfall_mm']:.5f} mm.",
        f"2. The optimistic non-convex R3 escape still intrudes "
        f"{route['checks']['optimistic nonconvex R3 escape remains in tooth sector']['sector_intrusion_mm']:.6f} mm "
        f"outside the active sector. The raw order has "
        f"{aggregate['passes_with_at_least_one_completed_neighbor']} passes "
        f"with a completed neighbour and {aggregate['passes_with_both_completed_neighbors']} with both.",
        f"3. After full extraction the first turn has only the lined sharp-corner "
        f"support R{first['supported_corner_after_full_extraction_mm']:.5f}, "
        f"short of R3 by {first['shortfall_mm']:.5f} mm. It has no parent copper.", "",
        "## Capacity and build concept", "",
        f"Slot capacity itself passes: "
        f"{support['slot_capacity']['gross_slot_fill']:.1%} fill, "
        f"{support['slot_capacity']['hard_limit_turn_capacity']} turns at the "
        "60% hard limit. The face paddles do not consume lamination slot area.", "",
        "The bounded hardware concept uses two CNC-machined unfilled-PEEK "
        "contact paddles, a positive deployed shoulder, dowel-located M3 "
        "fasteners, a collinear fixed lost-motion stop, independent return "
        "springs, and hardwired deployed/retracted interlocks. It is not "
        "released because the controlling route gates fail.", "",
        "## Decision", "",
        report["bounded_conclusion"], "",
        f"Report SHA-256: `{report['report_sha256']}`", "",
    ]
    return "\n".join(lines)


def write_reports(report: dict[str, Any] | None = None) -> dict[str, Any]:
    report = analyze() if report is None else report
    REPORTS.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    MD_OUT.write_text(render_markdown(report), encoding="utf-8")
    return report


def main() -> int:
    report = write_reports()
    print(
        f"M0-following full shroud: {report['status']}; "
        f"{len(report['controlling_failures'])} controlling failures"
    )
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
