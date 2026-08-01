"""Fail-closed wire-path audit for the production cap / retained flyer pair.

CAD brief:
- Task type: source-level inspection and validation; no production CAD is
  edited and no new STEP is generated.
- Units/frame: millimetres.  Machine +Z is the M2 axis, machine +Y is the
  stator shaft axis, and the M2 flyer rotates about machine +Z.
- Geometry inputs: the actual natural-unfilled-PEEK production-review caps,
  the retained offset-spoke flyer candidate, and the current named static
  wire guides.
- Motion input: the canonical, unmodified upstream raw capture.  Every one of
  its 24 coil starts and 2,400 physical half-turn loci is reconstructed.
- Authority boundary: raw M0/M1/M2 is controller authority; aggregate copper
  occupancy/contact support is a separate geometry authority; exact strand
  centres, order, settling and neatness are deliberately non-authoritative.
- Required checks: continuous spool-to-work path, R >= 3 mm, only named
  felt/dancer/ceramic/PEEK/active-copper contact, both shaft wraps, and no
  core/prior/neighbor intrusion.  Any absent geometry or coverage is a FAIL.
- Outputs: ``out/reports/integrated_phase_aware_wire_path.{json,md}`` plus a
  regression test.  This module never edits assembly/integrator sources.

The current candidate is expected to fail.  That is a result, not a fixture:
the retained source leaves an 8 mm centreline discontinuity between the
hollow-shaft axis and its visual R3 witness, and the witness occupies the
printed arm's front-face material.  The dynamic torus-to-cap check below also
requires the free span to arrive tangent to the named cap lane; an implicit
kink at the cap mouth is never promoted to an R3 pass.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import asdict
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
from build123d import Align, Cylinder, Pos, Rot


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
MANIFEST = ROOT / "out" / "links" / "manifest.json"
AGGREGATE_REPORT = REPORTS / "permanent_cap_aggregate_authorization.json"
CAP_REPORT = REPORTS / "permanent_cap_production_review.json"
RETAINED_REPORT = REPORTS / "permanent_cap_offset_spoke_retained_review.json"
OUTPUT_JSON = REPORTS / "integrated_phase_aware_wire_path.json"
OUTPUT_MD = REPORTS / "integrated_phase_aware_wire_path.md"
SOURCE = HERE / "integrated_phase_aware_wire_path.py"
CURRENT_UPSTREAM_WINDING = ROOT.parent / "winder" / "src" / "winding.py"
LAST_EXACT_WRAP_WINDING = (
    ROOT.parent / "winder-goal1-contract" / "src" / "winding.py"
)

for search_path in (CAD, HERE):
    if str(search_path) not in sys.path:
        sys.path.insert(0, str(search_path))

import collide  # noqa: E402
from continuous_conductor_route import _raw_shaft_wraps  # noqa: E402
from params import DEFAULT_STATOR, PARAMS  # noqa: E402
import permanent_cap_offset_spoke_retained_review as retained  # noqa: E402
import permanent_cap_offset_spoke_review as offset_flyer  # noqa: E402
from phase_aware_progressive_wire_audit import (  # noqa: E402
    EXPECTED_PASSES,
    EXPECTED_STATE_COUNT,
    STATES_PER_PASS,
    RawLocus,
    core_prism_intersection,
    extract_raw_loci,
)
import stator_insulation_nomex410 as insulation_source  # noqa: E402
from traj import Timeline, load_events  # noqa: E402
import wire_geometry  # noqa: E402
import wirepath  # noqa: E402


SCHEMA = "integrated-phase-aware-wire-path/v1"
EXPECTED_CAPTURE_SCHEMA = 4
MINIMUM_BEND_RADIUS_MM = 3.0
MAX_CAP_ENTRY_TANGENT_ERROR_DEG = 1.0e-5
RAW_M1_WRAP_STEP_DEG = 0.5
ROUND_DIGITS = 9

AUTH_RAW = "AUTHORITATIVE_RAW_CONTROLLER_LOCUS"
AUTH_AGGREGATE = "AUTHORITATIVE_AGGREGATE_COPPER_OCCUPANCY"
AUTH_EXACT_STRANDS = "NON_AUTHORITATIVE_EXACT_STRAND_PACKING"

ALLOWED_CONTACT_CLASSES = (
    "felt_drag_pads",
    "dancer_pulley",
    "ceramic_entry_eyelet",
    "ceramic_flyer_tip_toroid",
    "ceramic_shaft_wrap_sleeve",
    "PEEK_cap_lane",
    "active_aggregate_copper_boundary",
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    body = deepcopy(dict(payload))
    body.pop("report_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} is not a JSON object")
    return value


def _finite(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be finite") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _report_self_hash_valid(report: Mapping[str, Any]) -> bool:
    return report.get("report_sha256") == _canonical_hash(report)


def _source_hash_current(report: Mapping[str, Any], relative_path: str) -> bool:
    expected = report.get("source_hashes", {}).get(relative_path)
    path = ROOT / relative_path
    return bool(expected and path.is_file() and expected == _sha256(path))


def _unit(vector: Sequence[float]) -> np.ndarray:
    value = np.asarray(vector, dtype=float)
    length = float(np.linalg.norm(value))
    if length <= 1.0e-12:
        raise ValueError("zero-length vector")
    return value / length


def _angle_to_axis_deg(vector: Sequence[float], axis: Sequence[float]) -> float:
    # Collinearity, rather than orientation, is the cap-lane C1 requirement.
    dot = abs(float(np.dot(_unit(vector), _unit(axis))))
    return math.degrees(math.acos(float(np.clip(dot, -1.0, 1.0))))


def _stator_local_to_world(point: Sequence[float], locus: RawLocus) \
        -> np.ndarray:
    """Apply the actual assembly stator transform at one raw locus."""

    local = np.asarray(point, dtype=float)
    tooth_angle = (
        locus.tooth_index * 2.0 * math.pi / int(DEFAULT_STATOR.slots)
    )
    rotated = wirepath.rot_z(tooth_angle) @ local
    # assembly.spindle_link maps stator-local (x,y,z) to
    # machine (-y,z,standoff-x), then raw M1 rotates about the shaft axis.
    reference_relative = np.array(
        [-rotated[1], rotated[2], -rotated[0]], dtype=float,
    )
    axis_z = float(PARAMS.stator_axis_z(locus.m0_rad))
    return (
        np.array([0.0, 0.0, axis_z])
        + wirepath.rot_y(locus.m1_rad) @ reference_relative
    )


def _world_to_active_local(points: np.ndarray, locus: RawLocus) -> np.ndarray:
    """Invert actual raw M0/M1 and normalize the active tooth to tooth zero."""

    points = np.asarray(points, dtype=float)
    axis_z = float(PARAMS.stator_axis_z(locus.m0_rad))
    relative = points - np.array([0.0, 0.0, axis_z])
    reference = relative @ wirepath.rot_y(-locus.m1_rad).T
    base = np.column_stack((-reference[:, 2], -reference[:, 0], reference[:, 1]))
    tooth_angle = (
        locus.tooth_index * 2.0 * math.pi / int(DEFAULT_STATOR.slots)
    )
    return base @ wirepath.rot_z(-tooth_angle).T


def _expected_port(locus: RawLocus) -> tuple[str, int]:
    """Return named lane endpoint and end-cap sign for this half turn."""

    even = locus.half_turn_index == 0
    if even:
        side = "right" if locus.clockwise_argument else "left"
        axial_sign = 1
    else:
        side = "left" if locus.clockwise_argument else "right"
        axial_sign = -1
    return side, axial_sign


def _port_local(
    lane: Mapping[str, Any], side: str, axial_sign: int,
) -> np.ndarray:
    key = "outgoing_endpoint_mm" if side == "left" else "incoming_endpoint_mm"
    point = np.asarray(lane["nominal_front_centerline"][key], dtype=float)
    if axial_sign < 0:
        point = point.copy()
        point[2] *= -1.0
    return point


def _static_route_audit() -> dict[str, Any]:
    spec = wire_geometry.static_path_spec()
    landmarks = spec["landmarks"]
    points = np.asarray(spec["points"], dtype=float)
    spool = np.asarray(landmarks["spool_payoff"], dtype=float)
    felt = np.asarray(landmarks["felt_contact"], dtype=float)
    tangent_in = np.asarray(landmarks["dancer_tangent_in"], dtype=float)
    tangent_out = np.asarray(landmarks["dancer_tangent_out"], dtype=float)
    center = np.asarray(landmarks["dancer_center"], dtype=float)
    corner = np.asarray(landmarks["entry_corner"], dtype=float)

    felt_deflection = wirepath._angle_deg(felt - spool, tangent_in - felt)
    incoming = tangent_in[:2] - felt[:2]
    outgoing = corner[:2] - tangent_out[:2]
    rin = tangent_in[:2] - center[:2]
    rout = tangent_out[:2] - center[:2]
    radii = {
        "spool_pack": float(spec["spool_pack_radius"]),
        "felt_inlet_fairlead": 3.5,
        "dancer_pulley": float(spec["dancer"]["path_radius"]),
        "entry_ceramic_elbow": float(spec["entry_bend"]["radius"]),
    }
    bore_rear = np.asarray(landmarks["bore_rear"], dtype=float)
    shaft_z = (
        float(PARAMS.flyer_shaft_rear_z)
        - float(offset_flyer.M2_MODULE_REAR_SHIFT_MM),
        float(offset_flyer.SPOKE_FRONT_Z_MM),
    )
    bore_inside_extended_shaft = (
        abs(float(bore_rear[0])) <= 1.0e-12
        and abs(float(bore_rear[1])) <= 1.0e-12
        and shaft_z[0] <= float(bore_rear[2]) <= shaft_z[1]
    )
    gates = {
        "felt_is_straight_pass": felt_deflection <= 0.1,
        "dancer_incoming_is_tangent": abs(float(np.dot(_unit(rin), _unit(incoming)))) <= 1e-6,
        "dancer_outgoing_is_tangent": abs(float(np.dot(_unit(rout), _unit(outgoing)))) <= 1e-6,
        "dancer_wrap_is_80deg": math.isclose(float(spec["dancer"]["wrap_deg"]), 80.0, abs_tol=1e-9),
        "named_static_guide_radii_ge_3mm": min(radii.values()) >= MINIMUM_BEND_RADIUS_MM,
        "static_route_enters_extended_hollow_shaft": bore_inside_extended_shaft,
        # The checked-in main manifest is not the retained flyer / production
        # cap integration, so old mesh clearances cannot authorize this pair.
        "actual_integrated_static_clearance_mesh_present": False,
    }
    return {
        "status": "FAIL" if not all(gates.values()) else "PASS",
        "centerline_point_count": int(len(points)),
        "source_start_mm": points[0].tolist(),
        "source_end_mm": points[-1].tolist(),
        "felt_straight_pass_deflection_deg": float(felt_deflection),
        "dancer_tangency_dot": {
            "incoming": abs(float(np.dot(_unit(rin), _unit(incoming)))),
            "outgoing": abs(float(np.dot(_unit(rout), _unit(outgoing)))),
        },
        "guide_centerline_radii_mm": radii,
        "extended_shaft_z_span_mm": list(shaft_z),
        "allowed_contact_classes": list(ALLOWED_CONTACT_CLASSES[:3]),
        "gates": gates,
        "authority_note": (
            "source geometry is checked, but old out/links meshes are not an "
            "integrated clearance authority for the retained candidate"
        ),
    }


def _positive_overlap_probe(arm: Any, radius_mm: float) -> dict[str, Any]:
    """Exact OCC witness for the straight visual run inside the arm face."""

    # The actual witness is straight over y=8..58 at z=SPOKE_FRONT_Z.  This
    # bounded y=10..40 sub-cylinder is wholly within that source interval and
    # therefore avoids all R3 transition/toroid ambiguity.
    probe = Pos(0.0, 25.0, offset_flyer.SPOKE_FRONT_Z_MM) * (
        Rot(90.0, 0.0, 0.0)
        * Cylinder(
            float(radius_mm), 30.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
    )
    overlap = arm & probe
    volume = 0.0 if overlap is None else float(overlap.volume)
    point = (0.0, 20.0, offset_flyer.SPOKE_FRONT_Z_MM - radius_mm / 2.0)
    return {
        "probe_centerline_interval_mm": {
            "start": [0.0, 10.0, offset_flyer.SPOKE_FRONT_Z_MM],
            "end": [0.0, 40.0, offset_flyer.SPOKE_FRONT_Z_MM],
        },
        "wire_radius_mm": float(radius_mm),
        "OCC_positive_intersection_volume_mm3": volume,
        "inside_point_mm": list(point),
        "inside_retained_arm": bool(arm.is_inside(point)),
        "positive_overlap": bool(volume > 1.0e-8),
    }


def _flyer_feed_audit() -> tuple[dict[str, Any], dict[str, Any]]:
    arm = retained.retained_arm()
    witness = offset_flyer.flyer_wire_transition_witness()
    wire_radius = float(DEFAULT_STATOR.wire_d) / 2.0
    start = np.array([0.0, 8.0, offset_flyer.SPOKE_FRONT_Z_MM])
    shaft_exit = np.array([0.0, 0.0, offset_flyer.SPOKE_FRONT_Z_MM])
    end = np.array([
        0.0,
        offset_flyer.TIP_GUIDE_CENTER_RADIUS_MM + 3.0,
        offset_flyer.TIP_GUIDE_CENTER_Z_MM,
    ])
    gap = float(np.linalg.norm(start - shaft_exit))
    job_probe = _positive_overlap_probe(arm, wire_radius)
    launch_probe = _positive_overlap_probe(arm, 0.5 / 2.0)
    gates = {
        "extended_hollow_shaft_exists": True,
        "shaft_axis_to_R3_witness_centerline_connected": gap <= 1.0e-9,
        "explicit_shaft_exit_turn_radius_ge_3mm": False,
        "visual_witness_centerline_R3_primitives": True,
        "job_wire_has_no_positive_PETG_arm_overlap": not job_probe["positive_overlap"],
        "launch_max_wire_has_no_positive_PETG_arm_overlap": not launch_probe["positive_overlap"],
        "all_flyer_contacts_are_named_allowed_guides": False,
    }
    return ({
        "status": "FAIL" if not all(gates.values()) else "PASS",
        "hollow_shaft_centerline_exit_mm": shaft_exit.tolist(),
        "visual_R3_witness_start_mm": start.tolist(),
        "visual_R3_witness_end_feed_mm": end.tolist(),
        "unmodeled_centerline_gap_mm": gap,
        "witness_distance_to_retained_arm_mm": float(arm.distance_to(witness)),
        "job_wire_overlap_witness": job_probe,
        "launch_max_wire_overlap_witness": launch_probe,
        "minimum_declared_witness_centerline_radius_mm": 3.0,
        "forbidden_contact": "retained_printed_arm_PETG",
        "gates": gates,
    }, {
        "center_local_mm": [
            0.0,
            float(offset_flyer.TIP_GUIDE_CENTER_RADIUS_MM),
            float(offset_flyer.TIP_GUIDE_CENTER_Z_MM),
        ],
        "axis_local": [0.0, 1.0, 0.0],
        "feed_local_mm": end.tolist(),
        "major_radius_mm": 6.5,
        "tube_radius_mm": 3.0,
        "material": "99.8% alumina ceramic",
    })


def _locus_transfer(
    locus: RawLocus,
    lane: Mapping[str, Any],
    guide: Mapping[str, Any],
    lamination_face: Any,
) -> dict[str, Any]:
    side, axial_sign = _expected_port(locus)
    local_port = _port_local(lane, side, axial_sign)
    target = _stator_local_to_world(local_port, locus)
    flyer_rotation = wirepath.rot_z(locus.m2_rad)
    feed = flyer_rotation @ np.asarray(guide["feed_local_mm"], dtype=float)
    try:
        path, meta = wirepath.tip_guide_path(
            feed,
            target,
            guide,
            float(DEFAULT_STATOR.wire_d) / 2.0,
            flyer_rotation,
        )
        approach = target - path[-2]
        tangent_error = _angle_to_axis_deg(approach, (0.0, 1.0, 0.0))
        local_path = _world_to_active_local(path, locus)
        core = core_prism_intersection(local_path, lamination_face)
        constructed = True
        reason = None
        path_digest = hashlib.sha256(
            np.round(path, decimals=9).tobytes()
        ).hexdigest()
    except Exception as exc:  # fail closed with the exact locus and reason
        meta = {
            "wire_center_bend_radius_mm": None,
            "inside_wire_path_radius_mm": None,
            "arc_turn_deg": None,
        }
        tangent_error = None
        core = {
            "intersects": False,
            "evaluated": False,
            "reason": str(exc),
        }
        constructed = False
        reason = f"{type(exc).__name__}: {exc}"
        path_digest = None
    return {
        "locus": asdict(locus),
        "authority": AUTH_RAW,
        "expected_cap": "front" if axial_sign > 0 else "rear",
        "expected_port": side,
        "port_local_mm": local_port.tolist(),
        "port_world_mm": target.tolist(),
        "path_constructed": constructed,
        "construction_failure": reason,
        "path_sha256": path_digest,
        "tip_guide_arc_turn_deg": meta["arc_turn_deg"],
        "tip_guide_wire_center_radius_mm": meta[
            "wire_center_bend_radius_mm"
        ],
        "tip_guide_inside_radius_mm": meta["inside_wire_path_radius_mm"],
        "cap_lane_tangent_error_deg": tangent_error,
        "implicit_cap_entry_kink": bool(
            tangent_error is None
            or tangent_error > MAX_CAP_ENTRY_TANGENT_ERROR_DEG
        ),
        "core_prism": core,
    }


def _transfer_audit(
    loci: Sequence[RawLocus],
    lane: Mapping[str, Any],
    guide: Mapping[str, Any],
) -> dict[str, Any]:
    lamination_face = insulation_source._main_lamination_face()
    cache: dict[tuple[Any, ...], dict[str, Any]] = {}
    records: list[dict[str, Any]] = []
    for locus in loci:
        key = (
            round(locus.radial_x_mm, 9),
            round(locus.m2_mod_rad, 9),
            locus.motion_sign,
            locus.half_turn_index,
            locus.clockwise_argument,
        )
        if key not in cache:
            cache[key] = _locus_transfer(
                locus, lane, guide, lamination_face,
            )
        # Preserve every raw locus even when exact 24-fold symmetry reuses a
        # geometry result.  Controller evidence is never collapsed.
        row = deepcopy(cache[key])
        row["locus"] = asdict(locus)
        records.append(row)

    constructed = [row for row in records if row["path_constructed"]]
    tangent_rows = [
        row for row in constructed
        if row["cap_lane_tangent_error_deg"] is not None
    ]
    core_hits = [
        row for row in constructed
        if row["core_prism"].get("intersects") is True
    ]
    kink_rows = [row for row in records if row["implicit_cap_entry_kink"]]
    minimum_radius = min(
        (
            float(row["tip_guide_wire_center_radius_mm"])
            for row in constructed
            if row["tip_guide_wire_center_radius_mm"] is not None
        ),
        default=None,
    )
    worst_tangent = max(
        tangent_rows,
        key=lambda row: float(row["cap_lane_tangent_error_deg"]),
        default=None,
    )
    first_core = core_hits[0] if core_hits else None
    gates = {
        "all_2400_raw_half_turn_loci_constructed": len(constructed) == EXPECTED_STATE_COUNT,
        "tip_toroid_centerline_radius_ge_3mm": bool(
            minimum_radius is not None
            and minimum_radius >= MINIMUM_BEND_RADIUS_MM
        ),
        "every_free_span_arrives_tangent_to_named_PEEK_lane": not kink_rows,
        "no_implicit_sub_R3_cap_mouth_kink": not kink_rows,
        "no_raw_free_span_centerline_crosses_lamination_core": not core_hits and len(constructed) == EXPECTED_STATE_COUNT,
        # Aggregate authority covers the named lane/connector boundary only;
        # it does not cover this presently non-tangent terminal free span.
        "terminal_span_does_not_enter_prior_active_aggregate": False,
        "terminal_span_does_not_enter_completed_neighbor_aggregate": False,
        "terminal_span_does_not_enter_completed_other_aggregate": False,
        "only_named_PEEK_or_active_copper_contact": not kink_rows,
    }
    return {
        "status": "FAIL" if not all(gates.values()) else "PASS",
        "raw_locus_count": len(records),
        "unique_geometry_case_count": len(cache),
        "constructed_locus_count": len(constructed),
        "implicit_kink_locus_count": len(kink_rows),
        "core_crossing_locus_count": len(core_hits),
        "minimum_tip_toroid_wire_center_radius_mm": minimum_radius,
        "cap_mouth_contact_edge_radius_mm": 0.1,
        "required_free_running_radius_mm": MINIMUM_BEND_RADIUS_MM,
        "minimum_cap_lane_tangent_error_deg": (
            min(float(row["cap_lane_tangent_error_deg"])
                for row in tangent_rows) if tangent_rows else None
        ),
        "maximum_cap_lane_tangent_error_deg": (
            float(worst_tangent["cap_lane_tangent_error_deg"])
            if worst_tangent else None
        ),
        "worst_cap_entry_witness": worst_tangent,
        "first_core_crossing_witness": first_core,
        "gates": gates,
        "loci": records,
    }


def _shaft_wrap_audit(
    wraps: Sequence[Mapping[str, Any]], timeline: Timeline,
    guide: Mapping[str, Any],
) -> dict[str, Any]:
    contact = wire_geometry.shaft_contact_spec(DEFAULT_STATOR)
    lamination_face = insulation_source._main_lamination_face()
    cases = []
    total_pose_count = 0
    for wrap in wraps:
        start = float(wrap["start"])
        m0, m1, m2 = map(float, timeline.pose_at(start))
        flyer_rotation = wirepath.rot_z(m2)
        feed = flyer_rotation @ np.asarray(guide["feed_local_mm"], dtype=float)
        center = flyer_rotation @ np.asarray(guide["center_local_mm"], dtype=float)
        side = -1 if float(wrap["delta_m1_rad"]) > 0.0 else 1
        target = wirepath.shaft_tangent_point(
            center,
            float(PARAMS.stator_axis_z(m0)),
            contact,
            side,
        )
        path, meta = wirepath.tip_guide_path(
            feed,
            target,
            guide,
            float(DEFAULT_STATOR.wire_d) / 2.0,
            flyer_rotation,
        )
        pose_count = int(math.ceil(
            abs(float(wrap["delta_m1_rad"]))
            / math.radians(RAW_M1_WRAP_STEP_DEG)
        )) + 1
        total_pose_count += pose_count

        # The bare stator/cap substrate is 24-fold periodic.  Thirty exact
        # 0.5-degree residues cover every raw M1 pose for the core class.
        periodic_core_hits = []
        for index in range(30):
            probe_m1 = m1 + math.radians(index * RAW_M1_WRAP_STEP_DEG)
            probe = RawLocus(
                pass_index=-1,
                phase_index=-1,
                tooth_index=0,
                motion_sign=side,
                clockwise_argument=side > 0,
                state_index=index,
                turn_index=0,
                half_turn_index=0,
                time_s=start,
                m0_rad=m0,
                m1_rad=probe_m1,
                m2_rad=m2,
                m2_mod_rad=float(m2 % (2.0 * math.pi)),
                radial_x_mm=float(PARAMS.stator_axis_z(m0)),
                m1_alignment_error_rad=0.0,
            )
            local = _world_to_active_local(path, probe)
            hit = core_prism_intersection(local, lamination_face)
            if hit["intersects"]:
                periodic_core_hits.append({
                    "residue_deg": index * RAW_M1_WRAP_STEP_DEG,
                    "witness": hit,
                })
        cases.append({
            "wrap_number": int(wrap["number"]),
            "raw_start_time_s": start,
            "raw_end_time_s": float(wrap["end"]),
            "raw_delta_m1_rad": float(wrap["delta_m1_rad"]),
            "raw_turns": float(wrap["turns"]),
            "raw_m2_rad": m2,
            "tangent_side": side,
            "raw_pose_count_at_le_0p5deg": pose_count,
            "tip_guide_wire_center_radius_mm": float(
                meta["wire_center_bend_radius_mm"]
            ),
            "shaft_sleeve_wire_center_radius_mm": float(
                contact["radius_to_wire_center_mm"]
            ),
            "periodic_core_residue_count": 30,
            "periodic_core_hits": periodic_core_hits,
            "path_sha256": hashlib.sha256(
                np.round(path, decimals=9).tobytes()
            ).hexdigest(),
        })
    core_hit_count = sum(len(row["periodic_core_hits"]) for row in cases)
    gates = {
        "two_raw_wraps_present": len(cases) == 2,
        "each_raw_wrap_is_two_full_turns": all(
            math.isclose(float(row["raw_turns"]), 2.0, abs_tol=1.0e-9)
            for row in cases
        ),
        "raw_wrap_pose_coverage_at_le_0p5deg": total_pose_count >= 2 * 1441,
        "tip_and_sleeve_contact_radii_ge_3mm": all(
            min(
                float(row["tip_guide_wire_center_radius_mm"]),
                float(row["shaft_sleeve_wire_center_radius_mm"]),
            ) >= MINIMUM_BEND_RADIUS_MM
            for row in cases
        ),
        "bare_core_clear_for_all_periodic_raw_M1_residues": core_hit_count == 0,
        # Completed phase copper is not rotationally symmetric, and no
        # integrated aggregate BREP has been supplied for the terminal span.
        "completed_phase_aggregate_clear_for_both_wraps": False,
        "actual_PEEK_caps_and_retained_flyer_clear_for_both_wraps": False,
    }
    return {
        "status": "FAIL" if not all(gates.values()) else "PASS",
        "case_count": len(cases),
        "raw_pose_count": total_pose_count,
        "periodic_core_hit_count": core_hit_count,
        "contact_model": contact,
        "cases": cases,
        "gates": gates,
    }


def _coil_starts(loci: Sequence[RawLocus]) -> list[dict[str, Any]]:
    starts = [row for row in loci if row.state_index == 0]
    return [
        {
            "pass_index": row.pass_index,
            "phase_index": row.phase_index,
            "tooth_index": row.tooth_index,
            "time_s": row.time_s,
            "m0_rad": row.m0_rad,
            "m1_rad": row.m1_rad,
            "m2_rad": row.m2_rad,
            "authority": AUTH_RAW,
        }
        for row in starts
    ]


@lru_cache(maxsize=1)
def analyze() -> dict[str, Any]:
    aggregate = _load_json(AGGREGATE_REPORT)
    cap = _load_json(CAP_REPORT)
    retained_report = _load_json(RETAINED_REPORT)
    manifest = _load_json(MANIFEST)
    events = load_events(CAPTURE)
    timeline = Timeline(events)
    meta = timeline.meta
    loci, passes = extract_raw_loci(events, timeline)
    wraps = _raw_shaft_wraps(events, timeline)

    if meta.get("controller_mode") != "upstream":
        raise ValueError("integrated wire audit requires upstream controller")
    if meta.get("controller_adapter_sha256") is not None:
        raise ValueError("integrated wire audit rejects controller adapters")
    if int(meta.get("capture_schema", -1)) != EXPECTED_CAPTURE_SCHEMA:
        raise ValueError("canonical raw capture schema drifted")

    contracts = {
        "aggregate": {
            "path": "out/reports/permanent_cap_aggregate_authorization.json",
            "status": aggregate.get("status"),
            "report_self_hash_valid": _report_self_hash_valid(aggregate),
            "source_current": _source_hash_current(
                aggregate, "sim/permanent_cap_aggregate_authorization.py",
            ),
            "aggregate_geometry_authorized": aggregate.get(
                "aggregate_geometry_authorized"
            ),
            "exact_strand_packing_predicted": aggregate.get(
                "aggregate_loft", {}
            ).get("exact_strand_packing_predicted"),
        },
        "production_PEEK_caps": {
            "path": "out/reports/permanent_cap_production_review.json",
            "status": cap.get("status"),
            "report_self_hash_valid": _report_self_hash_valid(cap),
            "source_current": _source_hash_current(
                cap, "cad/permanent_cap_production_review.py",
            ),
            "exact_lane_offset_surface_present": cap.get(
                "release_gates", {}
            ).get("exact_lane_offset_surface_present"),
            "full_raw_cycle_collision_regenerated": cap.get(
                "release_gates", {}
            ).get("full_offset_flyer_raw_cycle_collision_regenerated"),
        },
        "retained_offset_spoke_flyer": {
            "path": "out/reports/permanent_cap_offset_spoke_retained_review.json",
            "status": retained_report.get("status"),
            "report_self_hash_valid": _report_self_hash_valid(retained_report),
            "source_current": _source_hash_current(
                retained_report,
                "cad/permanent_cap_offset_spoke_retained_review.py",
            ),
            "retained_geometry_review": retained_report.get(
                "release_gates", {}
            ).get("isolated_retained_geometry_and_exact_balance"),
            "full_raw_cycle_collision_regenerated": retained_report.get(
                "release_gates", {}
            ).get("full_main_assembly_raw_collision_regenerated"),
        },
    }
    contracts_current = all(
        row["report_self_hash_valid"] and row["source_current"]
        for row in contracts.values()
    )

    static = _static_route_audit()
    flyer, guide = _flyer_feed_audit()
    lane = aggregate["cap_support_lane"]
    transfer = _transfer_audit(loci, lane, guide)
    shaft = _shaft_wrap_audit(wraps, timeline, guide)
    starts = _coil_starts(loci)
    current_upstream_source = CURRENT_UPSTREAM_WINDING.read_text(
        encoding="utf-8"
    )
    last_exact_source = LAST_EXACT_WRAP_WINDING.read_text(encoding="utf-8")
    if "self.move_motor(1, self.m1_zero - motor1_rotation)" not in current_upstream_source:
        raise ValueError("current upstream shaft-wrap formula drifted")
    if "self.move_motor(1, motor1_pos + motor1_rotation)" not in last_exact_source:
        raise ValueError("last exact-turn upstream provenance drifted")

    aggregate_gates = {
        "aggregate_report_current_and_PASS": bool(
            contracts["aggregate"]["report_self_hash_valid"]
            and contracts["aggregate"]["source_current"]
            and aggregate.get("status") == "PASS"
            and aggregate.get("aggregate_geometry_authorized") is True
        ),
        "all_24_aggregate_coils_and_connectors_authorized": all(
            bool(aggregate.get("gates", {}).get(name))
            for name in (
                "all_24_closed_aggregate_topology",
                "continuous_positive_area_slot_to_crown_connectors",
                "connectors_clear_core_cap_and_all_other_aggregates",
                "live_connector_does_not_enter_active_prior_aggregate",
                "complete_lane_meets_R3",
            )
        ),
        "exact_strand_packing_not_promoted_to_authority": (
            aggregate.get("aggregate_loft", {}).get(
                "exact_strand_packing_predicted"
            ) is False
        ),
    }

    raw_gates = {
        "canonical_unmodified_upstream_capture": True,
        "24_passes_present": len(passes) == EXPECTED_PASSES,
        "2400_half_turn_loci_present": len(loci) == EXPECTED_STATE_COUNT,
        "24_coil_starts_present": len(starts) == EXPECTED_PASSES,
        "all_coil_starts_are_state_zero": all(
            row.state_index == 0 for row in loci[::STATES_PER_PASS]
        ),
        "two_raw_shaft_wraps_present": len(wraps) == 2,
    }

    release_gates = {
        "source_contracts_current": contracts_current,
        "raw_control_authority_complete": all(raw_gates.values()),
        "aggregate_occupancy_authority_complete": all(aggregate_gates.values()),
        "static_spool_to_shaft_path_authorized": static["status"] == "PASS",
        "hollow_shaft_to_tip_path_authorized": flyer["status"] == "PASS",
        "tip_to_active_cap_all_2400_authorized": transfer["status"] == "PASS",
        "both_phase_shaft_wraps_authorized": shaft["status"] == "PASS",
        "only_named_contact_classes": (
            flyer["gates"]["all_flyer_contacts_are_named_allowed_guides"]
            and transfer["gates"]["only_named_PEEK_or_active_copper_contact"]
        ),
        "minimum_free_running_bend_radius_ge_3mm": (
            flyer["gates"]["explicit_shaft_exit_turn_radius_ge_3mm"]
            and transfer["gates"]["no_implicit_sub_R3_cap_mouth_kink"]
            and shaft["gates"]["tip_and_sleeve_contact_radii_ge_3mm"]
        ),
        "no_core_prior_or_neighbor_intrusion": (
            transfer["gates"]["no_raw_free_span_centerline_crosses_lamination_core"]
            and transfer["gates"]["terminal_span_does_not_enter_prior_active_aggregate"]
            and transfer["gates"]["terminal_span_does_not_enter_completed_neighbor_aggregate"]
            and transfer["gates"]["terminal_span_does_not_enter_completed_other_aggregate"]
            and shaft["gates"]["bare_core_clear_for_all_periodic_raw_M1_residues"]
            and shaft["gates"]["completed_phase_aggregate_clear_for_both_wraps"]
        ),
    }
    passed = all(release_gates.values())

    worst = transfer.get("worst_cap_entry_witness")
    exact_witnesses = [
        {
            "id": "W1_SHAFT_TO_WITNESS_DISCONTINUITY",
            "measurement_mm": flyer["unmodeled_centerline_gap_mm"],
            "from_mm": flyer["hollow_shaft_centerline_exit_mm"],
            "to_mm": flyer["visual_R3_witness_start_mm"],
            "meaning": "no source curve or >=3 mm elbow joins the hollow shaft to the retained flyer witness",
        },
        {
            "id": "W2_FORBIDDEN_PETG_POSITIVE_OVERLAP",
            **flyer["job_wire_overlap_witness"],
            "meaning": "the job-wire tube occupies the retained printed arm; PETG is not an allowed guide contact",
        },
    ]
    if worst is not None:
        exact_witnesses.append({
            "id": "W3_WORST_CAP_ENTRY_TANGENT_KINK",
            "pass_index": worst["locus"]["pass_index"],
            "state_index": worst["locus"]["state_index"],
            "time_s": worst["locus"]["time_s"],
            "m0_rad": worst["locus"]["m0_rad"],
            "m1_rad": worst["locus"]["m1_rad"],
            "m2_rad": worst["locus"]["m2_rad"],
            "expected_cap": worst["expected_cap"],
            "expected_port": worst["expected_port"],
            "port_world_mm": worst["port_world_mm"],
            "tangent_error_deg": worst["cap_lane_tangent_error_deg"],
            "cap_mouth_edge_radius_mm": transfer[
                "cap_mouth_contact_edge_radius_mm"
            ],
            "required_radius_mm": MINIMUM_BEND_RADIUS_MM,
            "meaning": "the torus free span is not tangent to the named PEEK lane, leaving an unmodeled sub-R3 mouth turn",
        })
    if transfer.get("first_core_crossing_witness") is not None:
        row = transfer["first_core_crossing_witness"]
        exact_witnesses.append({
            "id": "W4_TERMINAL_SPAN_CORE_CROSSING",
            "pass_index": row["locus"]["pass_index"],
            "state_index": row["locus"]["state_index"],
            "time_s": row["locus"]["time_s"],
            "core_prism": row["core_prism"],
            "meaning": "analytic line/prism clipping finds a bare lamination-core crossing",
        })
    if not shaft["gates"]["each_raw_wrap_is_two_full_turns"]:
        exact_witnesses.append({
            "id": "W5_RAW_SHAFT_WRAP_TURN_COUNT_MISMATCH",
            "wraps": [
                {
                    "wrap_number": row["wrap_number"],
                    "raw_delta_m1_rad": row["raw_delta_m1_rad"],
                    "raw_turns": row["raw_turns"],
                }
                for row in shaft["cases"]
            ],
            "required_turns_each": 2.0,
            "meaning": (
                "the unmodified upstream absolute targets do not produce "
                "two physical turns from the actual captured M1 start poses; "
                "geometry cannot change this raw clock"
            ),
        })

    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "PASS" if passed else "FAIL",
        "decision": (
            "INTEGRATED_PHASE_AWARE_WIRE_PATH_AUTHORIZED"
            if passed else
            "NO_GO__RETAINED_FLYER_FEED_AND_CAP_TRANSFER_NOT_GEOMETRICALLY_CONTINUOUS"
        ),
        "production_authorized": bool(passed),
        "assembly_integration_authorized": bool(passed),
        "authority_model": {
            "raw_control_locus": {
                "classification": AUTH_RAW,
                "authoritative": True,
                "meaning": "exact raw M0/M1/M2 clocks and the 24-pass order",
            },
            "aggregate_copper": {
                "classification": AUTH_AGGREGATE,
                "authoritative": True,
                "meaning": "nested occupancy, support and positive-area connector classes only",
            },
            "exact_strand_packing": {
                "classification": AUTH_EXACT_STRANDS,
                "authoritative": False,
                "predicted": False,
                "meaning": "no strand centers, layer order, passive settling or neatness claim",
            },
        },
        "source_contracts": contracts,
        "raw_capture": {
            "path": "out/capture/upstream_current_raw.jsonl",
            "sha256": _sha256(CAPTURE),
            "capture_schema": int(meta["capture_schema"]),
            "controller_mode": meta["controller_mode"],
            "controller_adapter_sha256": meta.get("controller_adapter_sha256"),
            "winder_commit": meta.get("winder_commit"),
            "pass_count": len(passes),
            "half_turn_locus_count": len(loci),
            "coil_starts": starts,
            "shaft_wrap_count": len(wraps),
            "gates": raw_gates,
        },
        "allowed_contact_classes": list(ALLOWED_CONTACT_CLASSES),
        "static_spool_felt_dancer_to_shaft": static,
        "retained_flyer_shaft_to_tip": flyer,
        "tip_to_active_PEEK_cap": transfer,
        "aggregate_copper_occupancy": {
            "status": "PASS" if all(aggregate_gates.values()) else "FAIL",
            "gates": aggregate_gates,
            "lane_id": lane["id"],
            "lane_minimum_wire_center_radius_mm": lane[
                "minimum_lane_wire_center_bend_radius_mm"
            ],
            "active_prior_intrusion_mm3": aggregate[
                "slot_to_crown_connectors"
            ]["progressive_aggregate_contract"][
                "active_prior_aggregate_positive_volume_intrusion_mm3"
            ],
            "completed_neighbor_intrusion_mm3": aggregate[
                "slot_to_crown_connectors"
            ]["progressive_aggregate_contract"][
                "completed_neighbor_aggregate_positive_volume_intrusion_mm3"
            ],
            "scope_limit": (
                "PASS applies to aggregate lane/connectors; it does not "
                "authorize the present non-tangent flyer terminal span"
            ),
        },
        "shaft_wraps": shaft,
        "upstream_regression_provenance": {
            "canonical_current": {
                "commit": "6039b33c8f15a20086c2195c3f2d02b3a833e8ca",
                "source_path": "winder/src/winding.py",
                "source_sha256": _sha256(CURRENT_UPSTREAM_WINDING),
                "target_basis": "self.m1_zero +/- 4*pi absolute target",
                "observed_physical_turns": [
                    row["raw_turns"] for row in shaft["cases"]
                ],
                "status": "FAIL",
            },
            "last_known_exact_turn_formulation": {
                "commit": "8ae82f9e9ebf8cba7afe48e75e5d255d96bdfe3f",
                "source_path": "winder-goal1-contract/src/winding.py",
                "source_sha256": _sha256(LAST_EXACT_WRAP_WINDING),
                "target_basis": "get_motor_position(1) + signed 4*pi relative target",
                "physical_turns_each": 2.0,
            },
            "minimal_upstream_regression_fix_not_applied": (
                "Restore the shaft-wrap target relative to the live M1 "
                "position: motor1_pos = get_motor_position(1); "
                "move_motor(1, motor1_pos + signed_4pi), while maintaining "
                "m1_zero consistently.  This is provenance only: the "
                "canonical current checkout and raw capture remain unmodified."
            ),
            "policy": "DO_NOT_PATCH_OR_FORK_UPSTREAM_IN_THIS_AUDIT",
        },
        "release_gates": release_gates,
        "exact_failure_witnesses": exact_witnesses,
        "smallest_design_changes": [
            {
                "priority": 1,
                "change": (
                    "Replace the visual shaft-to-tip witness with one real, "
                    "continuous polished PEEK or ceramic guide insert.  Join "
                    "the hollow-shaft axis with an explicit R>=3 mm elbow, "
                    "sweep/subtract its seat through the arm, and make the "
                    "insert own every contact now occurring in PETG."
                ),
                "closes": [
                    "W1_SHAFT_TO_WITNESS_DISCONTINUITY",
                    "W2_FORBIDDEN_PETG_POSITIVE_OVERLAP",
                ],
            },
            {
                "priority": 2,
                "change": (
                    "Add explicit R>=3 mm three-dimensional lead-ins at the "
                    "96 PEEK lane mouths (or a passive self-aligning ceramic "
                    "terminal guide) that contain the complete measured raw "
                    "approach cone.  The current 0.1 mm mouth edge cannot be "
                    "used as the missing turn."
                ),
                "closes": ["W3_WORST_CAP_ENTRY_TANGENT_KINK"],
            },
            {
                "priority": 3,
                "change": (
                    "Export the production caps, retained flyer, guide insert "
                    "and progressive aggregate obstacles into one phase-aware "
                    "collision set; then rerun all 2,400 loci and both raw "
                    "shaft wraps.  Do not reuse the old main-link meshes."
                ),
                "closes": [
                    "static integrated-clearance gate",
                    "terminal prior/neighbor aggregate gates",
                    "shaft-wrap cap/flyer/phase-copper gates",
                ],
            },
        ],
        "non_geometric_contract_blocker": (
            "The canonical raw capture produces 1.375 and 2.791666667 "
            "physical M1 turns during the two shaft-wrap calls, not two and "
            "two.  Because the upstream command stream is fixed, this must "
            "be resolved in the accepted upstream/configuration contract or "
            "the goal is unsatisfiable; changing the flyer cannot repair it."
        ),
        "limitations": [
            "No passive settling, exact strand packing, neatness, sag, snagging, friction, springback, enamel abrasion or tension dynamics are claimed.",
            "A raw-locus or aggregate PASS cannot override a missing physical guide or an unmodeled contact turn.",
            "No STEP is generated by this inspection-only validator, so CAD snapshot/viewer handoff does not apply.",
        ],
        "source_hashes": {
            "generator_source_sha256": _sha256(SOURCE),
            "raw_capture_sha256": _sha256(CAPTURE),
            "manifest_sha256": _sha256(MANIFEST),
            "aggregate_report_sha256": _sha256(AGGREGATE_REPORT),
            "cap_report_sha256": _sha256(CAP_REPORT),
            "retained_report_sha256": _sha256(RETAINED_REPORT),
            "wire_geometry_source_sha256": _sha256(CAD / "wire_geometry.py"),
            "retained_source_sha256": _sha256(
                CAD / "permanent_cap_offset_spoke_retained_review.py"
            ),
            "cap_source_sha256": _sha256(
                CAD / "permanent_cap_production_review.py"
            ),
            "phase_audit_source_sha256": _sha256(
                HERE / "phase_aware_progressive_wire_audit.py"
            ),
            "traj_source_sha256": _sha256(HERE / "traj.py"),
            "current_upstream_winding_sha256": _sha256(
                CURRENT_UPSTREAM_WINDING
            ),
            "last_exact_wrap_winding_sha256": _sha256(
                LAST_EXACT_WRAP_WINDING
            ),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    validate_report_integrity(report)
    return report


def validate_report_integrity(
    report: Mapping[str, Any], *, check_sources: bool = True,
) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported integrated wire-path schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("integrated wire-path report hash mismatch")
    if report.get("status") not in {"PASS", "FAIL"}:
        raise ValueError("integrated wire-path status is invalid")
    gates = report.get("release_gates", {})
    if not isinstance(gates, dict) or not gates:
        raise ValueError("integrated wire-path release gates are absent")
    expected_status = "PASS" if all(gates.values()) else "FAIL"
    if report.get("status") != expected_status:
        raise ValueError("integrated wire-path status does not match gates")
    if report.get("production_authorized") is not (expected_status == "PASS"):
        raise ValueError("integrated wire-path production authority drifted")
    authority = report.get("authority_model", {})
    if authority.get("exact_strand_packing", {}).get("authoritative") is not False:
        raise ValueError("exact strand packing was promoted to authority")
    if int(report.get("raw_capture", {}).get("half_turn_locus_count", -1)) != EXPECTED_STATE_COUNT:
        raise ValueError("integrated wire-path raw locus count drifted")
    if len(report.get("raw_capture", {}).get("coil_starts", [])) != EXPECTED_PASSES:
        raise ValueError("integrated wire-path coil starts drifted")
    if check_sources:
        expected = {
            "generator_source_sha256": _sha256(SOURCE),
            "raw_capture_sha256": _sha256(CAPTURE),
            "manifest_sha256": _sha256(MANIFEST),
            "aggregate_report_sha256": _sha256(AGGREGATE_REPORT),
            "cap_report_sha256": _sha256(CAP_REPORT),
            "retained_report_sha256": _sha256(RETAINED_REPORT),
            "wire_geometry_source_sha256": _sha256(CAD / "wire_geometry.py"),
            "retained_source_sha256": _sha256(
                CAD / "permanent_cap_offset_spoke_retained_review.py"
            ),
            "cap_source_sha256": _sha256(
                CAD / "permanent_cap_production_review.py"
            ),
            "phase_audit_source_sha256": _sha256(
                HERE / "phase_aware_progressive_wire_audit.py"
            ),
            "traj_source_sha256": _sha256(HERE / "traj.py"),
            "current_upstream_winding_sha256": _sha256(
                CURRENT_UPSTREAM_WINDING
            ),
            "last_exact_wrap_winding_sha256": _sha256(
                LAST_EXACT_WRAP_WINDING
            ),
        }
        stale = [
            name for name, value in expected.items()
            if report.get("source_hashes", {}).get(name) != value
        ]
        if stale:
            raise ValueError(
                "integrated wire-path report has stale sources: "
                + ", ".join(stale)
            )


def render_markdown(report: Mapping[str, Any]) -> str:
    flyer = report["retained_flyer_shaft_to_tip"]
    transfer = report["tip_to_active_PEEK_cap"]
    shaft = report["shaft_wraps"]
    lines = [
        "# Integrated phase-aware wire-path audit",
        "",
        f"**{report['status']} - {report['decision']}**",
        "",
        "Raw motion, aggregate copper occupancy, and exact strand packing are separate authorities. Exact strand packing remains explicitly non-authoritative.",
        "",
        "## Coverage",
        "",
        f"- Raw passes: {report['raw_capture']['pass_count']}",
        f"- Raw half-turn loci: {report['raw_capture']['half_turn_locus_count']}",
        f"- Coil starts: {len(report['raw_capture']['coil_starts'])}",
        f"- Raw shaft wraps: {report['raw_capture']['shaft_wrap_count']}",
        f"- Tip/cap unique geometry cases: {transfer['unique_geometry_case_count']}",
        f"- Shaft-wrap <=0.5-degree poses represented: {shaft['raw_pose_count']}",
        "",
        "## Exact controlling witnesses",
        "",
        f"- Hollow-shaft axis to retained witness: **{flyer['unmodeled_centerline_gap_mm']:.6f} mm unmodeled gap**.",
        f"- Job wire / PETG arm OCC overlap: **{flyer['job_wire_overlap_witness']['OCC_positive_intersection_volume_mm3']:.9f} mm3**.",
        f"- Cap-entry implicit-kink loci: **{transfer['implicit_kink_locus_count']} / {transfer['raw_locus_count']}**.",
        f"- Worst cap-lane tangent error: **{transfer['maximum_cap_lane_tangent_error_deg']:.6f} deg**; the current mouth edge is 0.1 mm versus the required R3 free-running turn.",
        f"- Terminal-span core-crossing loci: **{transfer['core_crossing_locus_count']}**.",
        "",
        "## Release gates",
        "",
    ]
    lines.extend(
        f"- {'PASS' if value else 'FAIL'} - `{name}`"
        for name, value in report["release_gates"].items()
    )
    lines.extend((
        "",
        "## Smallest design changes",
        "",
    ))
    lines.extend(
        f"{row['priority']}. {row['change']}"
        for row in report["smallest_design_changes"]
    )
    lines.extend((
        "",
        "The validator makes no passive-settling, strand-neatness, tension-dynamics, sag, snagging, or enamel-abrasion claim.",
        "",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ))
    return "\n".join(lines)


def write_outputs(
    report: Mapping[str, Any],
    json_path: Path = OUTPUT_JSON,
    markdown_path: Path = OUTPUT_MD,
) -> None:
    json_path = Path(json_path)
    markdown_path = Path(markdown_path)
    json_path.parent.mkdir(parents=True, exist_ok=True)
    markdown_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    markdown_path.write_text(render_markdown(report), encoding="utf-8")


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", type=Path, default=OUTPUT_JSON)
    parser.add_argument("--markdown", type=Path, default=OUTPUT_MD)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(list(argv) if argv is not None else None)
    if args.validate_only:
        report = _load_json(args.json)
        validate_report_integrity(report)
    else:
        report = analyze()
        write_outputs(report, args.json, args.markdown)
    print(
        f"integrated phase-aware wire path {report['status']}: "
        f"{report['raw_capture']['half_turn_locus_count']} loci, "
        f"{report['tip_to_active_PEEK_cap']['implicit_kink_locus_count']} "
        f"cap-entry kinks, sha256 {report['report_sha256']}"
    )
    # A correct fail-closed audit is a successful generator run.  Callers gate
    # on report status rather than mistaking an intentional NO-GO for a crash.
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
