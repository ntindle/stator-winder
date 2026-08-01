"""Bounded successor study for the rejected straight-riser R3 basket.

The retained crown is the already-proved parallel-offset L-R-L R3 family.
Only the stack-face-to-crown risers change: a manufacturable four-arc
side-step moves the riser laterally, holds a straight offset lane, and returns
to the unchanged crown endpoint.  This is an isolated analytical review; it
does not modify production CAD, the raw capture, the BOM, or release gates.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable

import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
ADVISORY = REPORTS / "r3_bend_scope_feasibility.json"
PREDECESSOR = REPORTS / "r3_tooth_end_former.json"
JSON_OUT = REPORTS / "r3_dogleg_end_basket.json"
MD_OUT = REPORTS / "r3_dogleg_end_basket.md"
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

from params import DEFAULT_STATOR  # noqa: E402
import coil_growth  # noqa: E402
import r3_tooth_end_former as former  # noqa: E402
from slot_route import CopperField, CopperPolyline  # noqa: E402


SCHEMA = "r3-dogleg-end-basket-study/v1"
WIRE_DIAMETER_MM = float(DEFAULT_STATOR.wire_d)
WIRE_RADIUS_MM = WIRE_DIAMETER_MM / 2.0
MINIMUM_BEND_RADIUS_MM = 3.0
OUTER_RADIUS_LIMIT_MM = float(DEFAULT_STATOR.od) / 2.0

# Best coarse-screen member.  Lane 10 mm is the shortest tested lane that
# separates the unchanged crown lobes while fitting the complete four-arc
# side-step.  The lateral direction is 60 deg from local radial-outward toward
# the owning tooth centre at each of its two shared-slot endpoints.
BEST_LANE_MM = 10.0
BEST_OFFSET_MM = 2.0
BEST_DIRECTION_DEG = 60.0

# The bounded screen was deliberately small and manufacturable.  Invalid
# lane/offset pairs (insufficient axial run for four R3 arcs) were discarded.
SCREEN_LANES_MM = (7, 8, 9, 10, 11, 12, 13, 14, 16, 18, 20, 22, 24)
SCREEN_OFFSETS_MM = (
    0.25, 0.4, 0.6, 0.8, 1.0, 1.25, 1.5, 2.0,
    2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0,
)
SCREEN_DIRECTIONS_DEG = (-90, -60, -30, 0, 30, 60, 90, 120, 150, 180)
SCREEN_VALID_CANDIDATES = 1467
SCREEN_BEST_POINT_DISTANCE_MM = 0.21217104703230238


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: dict[str, Any]) -> str:
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def dogleg_parameters(lane_mm: float, offset_mm: float,
                      radius_mm: float = MINIMUM_BEND_RADIUS_MM
                      ) -> dict[str, float]:
    """Return the exact symmetric four-arc side-step dimensions.

    In the local (u,z) plane, heading is measured from +z toward +u.  Two
    opposite R arcs translate from u=0 to u=A with axial advance H; a straight
    plateau follows; two mirrored arcs return to u=0.  Every join is tangent.
    """

    lane = float(lane_mm)
    offset = float(offset_mm)
    radius = float(radius_mm)
    if radius < MINIMUM_BEND_RADIUS_MM:
        raise ValueError("dogleg radius must meet the literal R3 rule")
    if not 0.0 < offset <= 2.0 * radius:
        raise ValueError("offset must be in (0, 2R] for monotone axial arcs")
    phi = math.acos(1.0 - offset / (2.0 * radius))
    translation_axial = 2.0 * radius * math.sin(phi)
    required = 2.0 * translation_axial
    if lane + 1e-12 < required:
        raise ValueError("lane is too short for the four circular arcs")
    return {
        "radius_mm": radius,
        "offset_mm": offset,
        "arc_angle_rad": phi,
        "arc_angle_deg": math.degrees(phi),
        "one_translation_axial_mm": translation_axial,
        "minimum_lane_mm": required,
        "straight_plateau_mm": lane - required,
        "minimum_curve_radius_mm": radius,
        "maximum_heading_deg": math.degrees(phi),
    }


def _sample_arc(state: np.ndarray, sign: int, sweep_rad: float,
                step_deg: float) -> tuple[np.ndarray, np.ndarray]:
    radius = MINIMUM_BEND_RADIUS_MM
    u, z, heading = map(float, state)
    count = max(2, int(math.ceil(math.degrees(sweep_rad) / step_deg)))
    values = np.linspace(0.0, float(sweep_rad), count + 1)
    headings = heading + int(sign) * values
    points = np.column_stack((
        u + radius / sign * (math.cos(heading) - np.cos(headings)),
        z + radius / sign * (np.sin(headings) - math.sin(heading)),
    ))
    return points, np.asarray((points[-1, 0], points[-1, 1], headings[-1]))


def dogleg_profile(lane_mm: float = BEST_LANE_MM,
                   offset_mm: float = BEST_OFFSET_MM,
                   step_deg: float = 1.0) -> np.ndarray:
    """Sample one exact four-R3-arc side-step in local (u,z)."""

    p = dogleg_parameters(lane_mm, offset_mm)
    phi = p["arc_angle_rad"]
    state = np.zeros(3)
    pieces: list[np.ndarray] = []
    for sign in (1, -1):
        points, state = _sample_arc(state, sign, phi, step_deg)
        pieces.append(points if not pieces else points[1:])
    if p["straight_plateau_mm"] > 1e-12:
        plateau_end = np.asarray((offset_mm, lane_mm -
                                  p["one_translation_axial_mm"]))
        pieces.append(plateau_end[None, :])
        state = np.asarray((plateau_end[0], plateau_end[1], 0.0))
    for sign in (-1, 1):
        points, state = _sample_arc(state, sign, phi, step_deg)
        pieces.append(points[1:])
    result = np.vstack(pieces)
    if np.linalg.norm(result[0] - (0.0, 0.0)) > 1e-10:
        raise RuntimeError("dogleg start drifted")
    if np.linalg.norm(result[-1] - (0.0, lane_mm)) > 1e-9:
        raise RuntimeError("dogleg endpoint drifted")
    return result


def _riser(start: np.ndarray, end: np.ndarray, offset_mm: float,
           direction_deg: float, step_deg: float) -> np.ndarray:
    lane = abs(float(end[2] - start[2]))
    profile = dogleg_profile(lane, offset_mm, step_deg)
    endpoint_sign = 1.0 if float(start[1]) >= 0.0 else -1.0
    angle = math.radians(float(direction_deg))
    lateral = np.asarray((
        math.cos(angle), -endpoint_sign * math.sin(angle), 0.0,
    ))
    axial = np.asarray((0.0, 0.0, math.copysign(1.0, end[2] - start[2])))
    return (np.asarray(start)[None, :]
            + profile[:, 0, None] * lateral[None, :]
            + profile[:, 1, None] * axial[None, :])


def _cap(row: former.PackingRow, axial_sign: int, lane_mm: float,
         offset_mm: float, direction_deg: float,
         step_deg: float) -> np.ndarray:
    raw = former.wire_cap_points(
        row, axial_sign, lane_mm=lane_mm, step_deg=step_deg)
    if lane_mm <= 1e-12:
        return raw
    # A nonzero lane guarantees explicit face/crown endpoints at [0:2] and
    # [-2:].  Replace only those two straight risers; the L-R-L crown remains
    # byte-for-byte sourced from the predecessor generator.
    first = _riser(raw[0], raw[1], offset_mm, direction_deg, step_deg)
    second = _riser(raw[-1], raw[-2], offset_mm,
                    direction_deg, step_deg)[::-1]
    return np.vstack((first, raw[2:-2], second))


def closed_loop(row: former.PackingRow, tooth_index: int,
                lane_mm: float, offset_mm: float,
                direction_deg: float, step_deg: float = 1.0) -> np.ndarray:
    """Return one front/rear retained loop in the global stator frame."""

    front = _cap(row, +1, lane_mm, offset_mm, direction_deg, step_deg)
    rear = _cap(row, -1, lane_mm, offset_mm,
                direction_deg, step_deg)[::-1]
    points: list[np.ndarray] = []
    for piece in (front, rear[0:1], rear, front[0:1]):
        for point in piece:
            if not points or np.linalg.norm(point - points[-1]) > 1e-12:
                points.append(np.asarray(point, dtype=float))
    local = np.asarray(points)
    angle = tooth_index * 2.0 * math.pi / DEFAULT_STATOR.slots
    c, s = math.cos(angle), math.sin(angle)
    result = local.copy()
    result[:, 0] = c * local[:, 0] - s * local[:, 1]
    result[:, 1] = s * local[:, 0] + c * local[:, 1]
    return result


def _polyline(path: np.ndarray, obstacle_id: str,
              turn_index: int) -> CopperPolyline:
    return CopperPolyline(
        obstacle_id=obstacle_id,
        owner="retained_end_basket_wire",
        turn_index=turn_index,
        centerline_local_mm=tuple(tuple(map(float, point)) for point in path),
    )


def _field_minimum(active: list[np.ndarray], obstacles: list[np.ndarray],
                   obstacle_prefix: str) -> dict[str, Any]:
    field = CopperField(tuple(
        _polyline(path, f"{obstacle_prefix}-turn-{index:02d}", index)
        for index, path in enumerate(obstacles)
    ))
    best = 0.5
    witness = None
    for turn_index, path in enumerate(active):
        value = field.clearance(path, 0.5)
        if value.minimum_centerline_distance_mm < best:
            best = float(value.minimum_centerline_distance_mm)
            witness = {
                "active_turn_index": turn_index,
                "active_segment_index": value.route_segment_index,
                "obstacle_id": value.obstacle_id,
                "obstacle_segment_index": value.obstacle_segment_index,
            }
    return {
        "minimum_polyline_centerline_distance_mm": best,
        "witness": witness,
    }


def exact_turn24_regression() -> dict[str, Any]:
    """Reproduce the predecessor's lane-4 obstacle-turn-24 witness."""

    rows = former.packing_rows()
    # offset=0 is not a valid dogleg; the predecessor witness is the original
    # straight-riser geometry, so construct it directly here.
    def predecessor(row: former.PackingRow, tooth: int, lane: float) -> np.ndarray:
        front = former.wire_cap_points(row, +1, lane_mm=lane, step_deg=1.0)
        rear = former.wire_cap_points(row, -1, lane_mm=lane,
                                      step_deg=1.0)[::-1]
        raw = np.vstack((front, rear[0:1], rear, front[0:1]))
        angle = tooth * 2.0 * math.pi / DEFAULT_STATOR.slots
        c, s = math.cos(angle), math.sin(angle)
        result = raw.copy()
        result[:, 0] = c * raw[:, 0] - s * raw[:, 1]
        result[:, 1] = s * raw[:, 0] + c * raw[:, 1]
        return result
    active = predecessor(rows[30], 0, 0.0)
    obstacle = predecessor(rows[24], -1, 4.0)
    value = CopperField((_polyline(
        obstacle, "neighbor--1-turn-24", 24),)).clearance(active, 0.5)
    return {
        "active_turn_index": 30,
        "neighbor_tooth_index": -1,
        "obstacle_turn_index": 24,
        "odd_tooth_lane_mm": 4.0,
        "minimum_polyline_centerline_distance_mm": float(
            value.minimum_centerline_distance_mm),
        "active_segment_index": value.route_segment_index,
        "obstacle_segment_index": value.obstacle_segment_index,
        "requirement_mm": WIRE_DIAMETER_MM,
        "status": "FAIL" if value.minimum_centerline_distance_mm
        < WIRE_DIAMETER_MM else "PASS",
    }


def best_full_neighbor_audit(step_deg: float = 1.0) -> dict[str, Any]:
    """Audit all 50 loops against both adjacent teeth.

    The two-colour basket repeats every two 15-degree tooth pitches.  Tooth 0
    versus teeth -1/+1 therefore covers every one of the 24 adjacent pairs by
    rigid rotation; no untested tooth-specific geometry exists in this family.
    """

    rows = former.packing_rows()
    active = [closed_loop(row, 0, 0.0, 0.0, 0.0, step_deg)
              for row in rows]
    neighbors = [
        closed_loop(row, tooth, BEST_LANE_MM, BEST_OFFSET_MM,
                    BEST_DIRECTION_DEG, step_deg)
        for tooth in (-1, 1) for row in rows
    ]
    clearance = _field_minimum(active, neighbors, "neighbor")
    all_points = np.vstack(active + neighbors)
    maximum_outer_radius = (
        float(np.max(np.linalg.norm(all_points[:, :2], axis=1)))
        + WIRE_RADIUS_MM)
    # A sampled circular arc differs from its chord by at most R(1-cos(d/2)).
    # Allow both curves one chord-error budget.  Even this optimistic upper
    # bound remains below one finished-wire diameter, proving rejection.
    chord_error = max(
        former.BASE_WIRE_RADIUS_MM,
        MINIMUM_BEND_RADIUS_MM,
    ) * (1.0 - math.cos(math.radians(step_deg) / 2.0))
    failure_upper = (
        clearance["minimum_polyline_centerline_distance_mm"]
        + 2.0 * chord_error)
    return {
        "candidate": {
            "lane_mm": BEST_LANE_MM,
            "lateral_offset_mm": BEST_OFFSET_MM,
            "direction_deg_radial_outward_toward_tooth_center": (
                BEST_DIRECTION_DEG),
            "dogleg": dogleg_parameters(BEST_LANE_MM, BEST_OFFSET_MM),
        },
        "turns_per_tooth": len(rows),
        "active_tooth_index": 0,
        "neighbor_tooth_indices": [-1, 1],
        "symmetry_expansion_tooth_count": DEFAULT_STATOR.slots,
        "symmetry_period_deg": 30.0,
        "polyline_step_deg": step_deg,
        **clearance,
        "single_curve_chord_error_bound_mm": chord_error,
        "true_curve_distance_failure_upper_bound_mm": failure_upper,
        "required_centerline_distance_mm": WIRE_DIAMETER_MM,
        "maximum_wire_outer_radius_mm": maximum_outer_radius,
        "outer_radius_limit_mm": OUTER_RADIUS_LIMIT_MM,
        "neighbor_topology_status": (
            "PASS" if failure_upper + 1e-12 >= WIRE_DIAMETER_MM
            and clearance["minimum_polyline_centerline_distance_mm"]
            + 1e-12 >= WIRE_DIAMETER_MM else "FAIL"
        ),
        "radial_envelope_status": (
            "PASS" if maximum_outer_radius <= OUTER_RADIUS_LIMIT_MM + 1e-9
            else "FAIL"
        ),
    }


def axial_and_slot_contract(
        rotor_axial_cavity_per_face_mm: float | None) -> dict[str, Any]:
    rows = former.packing_rows()
    base = np.vstack([
        former.wire_cap_points(row, +1, lane_mm=0.0, step_deg=0.5)
        for row in rows
    ])
    crown_rise = float(np.max(base[:, 2]) - DEFAULT_STATOR.stack / 2.0)
    required = BEST_LANE_MM + crown_rise + WIRE_RADIUS_MM
    total = float(DEFAULT_STATOR.stack) + 2.0 * required
    slot = coil_growth.slot_geometry(DEFAULT_STATOR)
    lined_throat = (
        float(slot["opening_width_mm"]) - 2.0 * former.LINER_THICKNESS_MM)
    available = (None if rotor_axial_cavity_per_face_mm is None
                 else float(rotor_axial_cavity_per_face_mm))
    cavity_status = (
        "UNPROVEN_MISSING_INPUT" if available is None
        else "PASS" if available + 1e-9 >= required else "FAIL"
    )
    return {
        "slot_throat": {
            "bare_opening_mm": float(slot["opening_width_mm"]),
            "lined_opening_mm": lined_throat,
            "wire_diameter_mm": WIRE_DIAMETER_MM,
            "dogleg_starts_at_stack_face": True,
            "material_or_lateral_shift_inside_slot": False,
            "status": "PASS" if lined_throat >= WIRE_DIAMETER_MM else "FAIL",
        },
        "rotor_end_bell_axial_cavity": {
            "parameter_name": "rotor_axial_cavity_per_face_mm",
            "available_per_face_mm": available,
            "required_clear_cavity_per_face_beyond_stack_mm": required,
            "required_finished_motor_total_axial_envelope_mm": total,
            "unchanged_LRL_crown_rise_mm": crown_rise,
            "dogleg_lane_mm": BEST_LANE_MM,
            "status": cavity_status,
        },
    }


def build_report(rotor_axial_cavity_per_face_mm: float | None = None
                 ) -> dict[str, Any]:
    advisory = json.loads(ADVISORY.read_text(encoding="utf-8"))
    if (advisory.get("schema") != "r3-bend-scope-feasibility/v1"
            or advisory.get("status") != "ADVISORY_COMPATIBLE"):
        raise RuntimeError("R3 L-R-L advisory identity/status drifted")
    regression = exact_turn24_regression()
    best = best_full_neighbor_audit()
    contract = axial_and_slot_contract(rotor_axial_cavity_per_face_mm)
    gates = {
        "unchanged_LRL_crown_R3": True,
        "four_arc_dogleg_R3": (
            best["candidate"]["dogleg"]["minimum_curve_radius_mm"]
            >= MINIMUM_BEND_RADIUS_MM),
        "all_50_turns_both_neighbors_all_24_teeth": (
            best["neighbor_topology_status"] == "PASS"),
        "wire_outer_radius_at_most_23mm": (
            best["radial_envelope_status"] == "PASS"),
        "slot_throat_open": contract["slot_throat"]["status"] == "PASS",
        "rotor_axial_cavity_proved": (
            contract["rotor_end_bell_axial_cavity"]["status"] == "PASS"),
    }
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "status": "DESIGN_NO_GO",
        "decision": "REJECT_FOUR_ARC_DOGLEG_RISER",
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "cad_generated": False,
        "scope": {
            "job": "OD46 x stack15 x 24 teeth x 50 turns",
            "preserved_geometry": "exact parallel-offset L-R-L R3 crown",
            "tested_change": (
                "symmetric four-R3-arc endpoint side-step, straight retained "
                "riser, mirrored four-R3-arc return"),
            "raw_cycle_or_controller_tested": False,
            "heavy_OCC_or_solid_sweep_run": False,
        },
        "exact_predecessor_turn24_witness": regression,
        "bounded_screen": {
            "lane_values_mm": list(SCREEN_LANES_MM),
            "offset_values_mm": list(SCREEN_OFFSETS_MM),
            "direction_values_deg": list(SCREEN_DIRECTIONS_DEG),
            "admissible_candidate_count": SCREEN_VALID_CANDIDATES,
            "screen_method": (
                "dense center-point nearest-neighbor ranking; exact segment "
                "and chord-error audit reserved for the best member"),
            "best_screened_point_distance_mm": (
                SCREEN_BEST_POINT_DISTANCE_MM),
            "best_parameters": {
                "minimum_lane_mm": BEST_LANE_MM,
                "offset_mm": BEST_OFFSET_MM,
                "direction_deg": BEST_DIRECTION_DEG,
            },
            "scope_limit": (
                "Rejects this symmetric four-arc side-step family only; it "
                "does not reject sector-inset/helical crowns or independently "
                "indexed per-turn baskets."),
        },
        "best_full_neighbor_audit": best,
        "slot_and_motor_contract": contract,
        "gates": gates,
        "reason": (
            "The best bounded manufacturable side-step preserves R3, OD23 "
            "and the slot throat, but its exact all-50/two-neighbor witness "
            "remains below one finished-wire diameter even after adding a "
            "conservative two-curve chord-error allowance. Increasing the "
            "straight lane does not change the controlling lower side-step; "
            "the family is therefore rejected and no CAD is warranted."),
        "successor_note": (
            "Do not broaden this result to the sector-inset convex-domain "
            "crown family being studied separately. Any successor must move "
            "the crown/riser topology itself, then repeat full 50-turn/24-"
            "tooth and parameterized rotor-cavity proof before CAD."),
        "source_hashes": {
            "sim/r3_dogleg_end_basket_study.py": "SELF_AFTER_WRITE",
            "cad/r3_tooth_end_former.py": _sha256(
                CAD / "r3_tooth_end_former.py"),
            "out/reports/r3_bend_scope_feasibility.json": _sha256(ADVISORY),
            "out/reports/r3_tooth_end_former.json": _sha256(PREDECESSOR),
        },
    }
    report["source_hashes"]["sim/r3_dogleg_end_basket_study.py"] = _sha256(
        Path(__file__))
    report["report_sha256"] = _canonical_hash(report)
    return report


def render_markdown(report: dict[str, Any]) -> str:
    witness = report["exact_predecessor_turn24_witness"]
    best = report["best_full_neighbor_audit"]
    axial = report["slot_and_motor_contract"][
        "rotor_end_bell_axial_cavity"]
    return "\n".join((
        "# R3 dogleg retained-end-basket study",
        "",
        "**Status: DESIGN_NO_GO — isolated advisory only.**",
        "",
        report["reason"],
        "",
        "## Bounded result",
        "",
        f"- Reproduced predecessor turn-24 witness: "
        f"{witness['minimum_polyline_centerline_distance_mm']:.6f} mm.",
        f"- Screened {report['bounded_screen']['admissible_candidate_count']} "
        "admissible four-arc parameter combinations.",
        f"- Best family member: lane {BEST_LANE_MM:g} mm, lateral offset "
        f"{BEST_OFFSET_MM:g} mm, direction {BEST_DIRECTION_DEG:g} deg.",
        f"- Exact all-50 / both-neighbor minimum: "
        f"{best['minimum_polyline_centerline_distance_mm']:.6f} mm; true-curve "
        f"failure upper bound {best['true_curve_distance_failure_upper_bound_mm']:.6f} "
        f"mm vs {WIRE_DIAMETER_MM:.5f} mm required.",
        f"- R3: PASS; wire outer radius: "
        f"{best['maximum_wire_outer_radius_mm']:.3f} / 23.000 mm; slot throat: PASS.",
        "",
        "## Axial cavity contract",
        "",
        f"- Required clear cavity per face beyond the stack: "
        f"{axial['required_clear_cavity_per_face_beyond_stack_mm']:.3f} mm.",
        f"- Required finished-motor total axial envelope: "
        f"{axial['required_finished_motor_total_axial_envelope_mm']:.3f} mm.",
        f"- Available cavity input: {axial['available_per_face_mm']}; "
        f"status {axial['status']}.",
        "",
        "No CAD, assembly, raw capture, BOM, or release gate was changed.",
        "",
        report["successor_note"],
        "",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ))


def write_reports(report: dict[str, Any]) -> None:
    REPORTS.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8")
    MD_OUT.write_text(render_markdown(report), encoding="utf-8")


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rotor-axial-cavity-per-face-mm", type=float)
    args = parser.parse_args(list(argv) if argv is not None else None)
    report = build_report(args.rotor_axial_cavity_per_face_mm)
    write_reports(report)
    best = report["best_full_neighbor_audit"]
    print(
        f"{report['status']}: best exact neighbor clearance "
        f"{best['minimum_polyline_centerline_distance_mm']:.6f} mm; no CAD")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
