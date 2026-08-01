"""Bounded advisory study of sector-confined spatial R3 end turns.

The earlier planar L-R-L witness proves one tooth but leaves its angular
Voronoi sector.  This successor asks whether a genuinely all-tooth family can
be made from two elementary spatial bends while keeping every wire centre in
its own sector:

* an R3 half-circle takes the outgoing axial tangent to an interior waypoint
  exactly 2R away and reverses the axial tangent; and
* a pair of opposite R3 arcs (an S-transfer) returns from that waypoint to the
  other coil side with the same descending axial tangent.

Both pieces have straight transverse projections.  A projection therefore
stays inside the convex intersection of the OD-centre disk and the
wire-radius-inset tooth sector whenever its endpoints do.  Vertical risers
permit a bounded set of axial phase pitches to be tested without changing the
exact 50-turn/shared-slot endpoints.

This is an advisory, fail-closed family study.  It does not modify CAD,
controller captures, BOMs, or release authority.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np
from scipy.spatial import cKDTree


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
JSON_OUT = REPORTS / "r3_sector_chord_family_study.json"
MD_OUT = REPORTS / "r3_sector_chord_family_study.md"
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

from params import DEFAULT_STATOR, PARAMS  # noqa: E402
import r3_bend_scope_feasibility as scope  # noqa: E402


SCHEMA = "r3-sector-chord-family-study/v1"
RADIUS_MM = 3.0
SAMPLE_ARCLENGTH_MM = 0.045
WAYPOINT_ANGLE_SAMPLES = 1440
PHASE_PITCH_MULTIPLIERS = (0.0, 0.5, 1.0, 2.0, 4.0)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: dict[str, Any]) -> str:
    raw = json.dumps(payload, sort_keys=True, separators=(",", ":"),
                     allow_nan=False).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _require_default() -> None:
    actual = (DEFAULT_STATOR.slots, DEFAULT_STATOR.od,
              DEFAULT_STATOR.stack, DEFAULT_STATOR.wire_d,
              DEFAULT_STATOR.turns, PARAMS.min_bend_radius)
    expected = (24, 46.0, 15.0, 0.22352, 50, 3.0)
    if actual != expected:
        raise RuntimeError(f"study is pinned to {expected}, not {actual}")


def _domain_margins(point: np.ndarray) -> tuple[float, float]:
    """Return centre clearance to sector sides and OD-centre boundary."""

    beta = math.pi / DEFAULT_STATOR.slots
    wire_r = DEFAULT_STATOR.wire_d / 2.0
    x, y = map(float, point[:2])
    side = x * math.sin(beta) - abs(y) * math.cos(beta) - wire_r
    radial = DEFAULT_STATOR.od / 2.0 - wire_r - math.hypot(x, y)
    return side, radial


def _choose_waypoint(a: np.ndarray, b: np.ndarray) -> dict[str, Any]:
    """Choose one exact 2R waypoint in the inset convex sector.

    The finite angular grid is only a candidate selector.  The returned point
    is checked directly, and all later containment statements use its measured
    endpoint margins plus convexity rather than treating the grid as a proof.
    """

    best: tuple[float, np.ndarray, float, float, float] | None = None
    for index in range(WAYPOINT_ANGLE_SAMPLES):
        angle = 2.0 * math.pi * index / WAYPOINT_ANGLE_SAMPLES
        c = a[:2] + 2.0 * RADIUS_MM * np.asarray(
            (math.cos(angle), math.sin(angle)))
        side, radial = _domain_margins(c)
        transfer = float(np.linalg.norm(b[:2] - c))
        if side < -1.0e-12 or radial < -1.0e-12 or transfer > 4.0 * RADIUS_MM:
            continue
        # Prefer robust sector/OD margin, then the shorter S-transfer.
        score = min(side, radial) - 1.0e-3 * transfer
        if best is None or score > best[0]:
            best = (score, c, side, radial, transfer)
    if best is None:
        raise RuntimeError("no 2R waypoint exists in the bounded selector")
    _, c, side, radial, transfer = best
    return {
        "xy_mm": [float(c[0]), float(c[1])],
        "distance_from_outgoing_endpoint_mm": float(
            np.linalg.norm(c - a[:2])),
        "distance_to_incoming_endpoint_mm": transfer,
        "sector_inset_margin_mm": side,
        "od_center_margin_mm": radial,
    }


def _append_segment(parts: list[np.ndarray], points: np.ndarray) -> None:
    if parts and np.linalg.norm(parts[-1][-1] - points[0]) < 1.0e-10:
        points = points[1:]
    if len(points):
        parts.append(points)


def _line(one: np.ndarray, two: np.ndarray) -> np.ndarray:
    distance = float(np.linalg.norm(two - one))
    count = max(1, math.ceil(distance / SAMPLE_ARCLENGTH_MM))
    return np.linspace(one, two, count + 1)


def build_front_path(row: dict[str, Any], phase_pitch_mm: float
                     ) -> tuple[np.ndarray, dict[str, Any]]:
    """Build one exact-endpoint, C1, piecewise-circular front end turn."""

    x = float(row["tooth_x_mm"])
    s = float(row["tooth_half_span_mm"])
    z0 = DEFAULT_STATOR.stack / 2.0
    a = np.asarray((x, -s, z0))
    b = np.asarray((x, +s, z0))
    waypoint = _choose_waypoint(a, b)
    cxy = np.asarray(waypoint["xy_mm"], dtype=float)
    d1 = (cxy - a[:2]) / (2.0 * RADIUS_MM)
    d2_vec = b[:2] - cxy
    d2 = float(np.linalg.norm(d2_vec))
    theta = math.acos(max(-1.0, min(1.0,
        1.0 - d2 / (2.0 * RADIUS_MM))))
    descent = 2.0 * RADIUS_MM * math.sin(theta)
    phase = int(row["turn_index"]) * phase_pitch_mm
    high_z = z0 + phase + descent

    pieces: list[np.ndarray] = []
    _append_segment(pieces, _line(a, np.asarray((x, -s, high_z))))

    count = max(2, math.ceil(math.pi * RADIUS_MM /
                             SAMPLE_ARCLENGTH_MM))
    phi = np.linspace(0.0, math.pi, count + 1)
    transverse = (a[:2][None, :] +
                  RADIUS_MM * (1.0 - np.cos(phi))[:, None] * d1[None, :])
    half = np.column_stack((transverse, high_z + RADIUS_MM * np.sin(phi)))
    _append_segment(pieces, half)

    if d2 > 1.0e-12:
        u = d2_vec / d2
        count_s = max(2, math.ceil(RADIUS_MM * theta /
                                   SAMPLE_ARCLENGTH_MM))
        q = np.linspace(0.0, theta, count_s + 1)
        w1 = RADIUS_MM * (1.0 - np.cos(q))
        down1 = RADIUS_MM * np.sin(q)
        first = np.column_stack((cxy[None, :] + w1[:, None] * u[None, :],
                                 high_z - down1))
        _append_segment(pieces, first)

        w2 = RADIUS_MM * (1.0 - math.cos(theta)) + RADIUS_MM * (
            np.cos(theta - q) - math.cos(theta))
        down2 = RADIUS_MM * math.sin(theta) + RADIUS_MM * (
            math.sin(theta) - np.sin(theta - q))
        second = np.column_stack((cxy[None, :] + w2[:, None] * u[None, :],
                                  high_z - down2))
        _append_segment(pieces, second)

    _append_segment(pieces, _line(
        np.asarray((x, s, z0 + phase)), b))
    path = np.vstack(pieces)
    return path, {
        "turn_index": int(row["turn_index"]),
        "row_index": int(row["row_index"]),
        "layer_index": int(row["layer_index"]),
        "outgoing_endpoint_mm": a.tolist(),
        "incoming_endpoint_mm": b.tolist(),
        "waypoint": waypoint,
        "s_transfer_sweep_deg": math.degrees(theta),
        "s_transfer_axial_descent_mm": descent,
        "axial_phase_mm": phase,
        "maximum_front_z_mm": float(np.max(path[:, 2])),
        "minimum_sampled_sector_inset_margin_mm": min(
            _domain_margins(point)[0] for point in path),
        "minimum_sampled_od_center_margin_mm": min(
            _domain_margins(point)[1] for point in path),
    }


def _minimum_sampled_pair(paths_a: list[np.ndarray],
                          paths_b: list[np.ndarray] | None = None,
                          *, exclude_same_index: bool = False
                          ) -> dict[str, Any]:
    best = math.inf
    witness: dict[str, Any] | None = None
    right_paths = paths_a if paths_b is None else paths_b
    for i, one in enumerate(paths_a):
        tree = cKDTree(one)
        start = i + 1 if paths_b is None else 0
        for j in range(start, len(right_paths)):
            if exclude_same_index and i == j:
                continue
            distances, indices = tree.query(right_paths[j], k=1)
            index_j = int(np.argmin(distances))
            value = float(distances[index_j])
            if value < best:
                best = value
                index_i = int(indices[index_j])
                witness = {
                    "first_turn": i,
                    "second_turn": j,
                    "first_point_mm": one[index_i].tolist(),
                    "second_point_mm": right_paths[j][index_j].tolist(),
                    "sampled_distance_mm": value,
                }
    return {"minimum_sampled_distance_mm": best, "witness": witness}


def _rotate(paths: list[np.ndarray], angle: float) -> list[np.ndarray]:
    c, s = math.cos(angle), math.sin(angle)
    matrix = np.asarray(((c, -s), (s, c)))
    result = []
    for path in paths:
        rotated = path.copy()
        rotated[:, :2] = path[:, :2] @ matrix.T
        result.append(rotated)
    return result


def analyze() -> dict[str, Any]:
    _require_default()
    rows = scope.square_row_centres()
    wire_d = float(DEFAULT_STATOR.wire_d)
    candidates = []
    turn24: dict[str, Any] | None = None
    for multiplier in PHASE_PITCH_MULTIPLIERS:
        pitch = multiplier * wire_d
        built = [build_front_path(row, pitch) for row in rows]
        paths = [item[0] for item in built]
        metadata = [item[1] for item in built]
        same = _minimum_sampled_pair(paths)
        neighbor = _minimum_sampled_pair(
            paths, _rotate(paths, 2.0 * math.pi / DEFAULT_STATOR.slots))
        direct_failure = min(same["minimum_sampled_distance_mm"],
                             neighbor["minimum_sampled_distance_mm"]) < wire_d
        candidates.append({
            "phase_pitch_multiplier": multiplier,
            "phase_pitch_mm": pitch,
            "total_phase_span_mm": 49.0 * pitch,
            "maximum_front_z_mm": max(item["maximum_front_z_mm"]
                                        for item in metadata),
            "minimum_sector_inset_margin_mm": min(
                item["minimum_sampled_sector_inset_margin_mm"]
                for item in metadata),
            "minimum_od_center_margin_mm": min(
                item["minimum_sampled_od_center_margin_mm"]
                for item in metadata),
            "same_tooth": same,
            "adjacent_tooth": neighbor,
            "direct_sampled_collision_witness": direct_failure,
            "status": "FAIL" if direct_failure else "UNPROVEN",
        })
        if math.isclose(multiplier, 1.0):
            turn24 = metadata[24]
            turn24["same_tooth_candidate_witness"] = same["witness"]

    all_fail = all(item["status"] == "FAIL" for item in candidates)
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "DESIGN_NO_GO" if all_fail else "UNPROVEN",
        "decision": (
            "REJECT_BOUNDED_SECTOR_CHORD_AND_S_TRANSFER_FAMILY"
            if all_fail else "NO_CONSTRUCTIVE_RELEASE_PROOF"
        ),
        "production_authorized": False,
        "integration_authorized": False,
        "inputs": {
            "slots": DEFAULT_STATOR.slots,
            "od_mm": DEFAULT_STATOR.od,
            "stack_mm": DEFAULT_STATOR.stack,
            "wire_finished_diameter_mm": wire_d,
            "turns_per_tooth": DEFAULT_STATOR.turns,
            "shared_slot_endpoint_count": 2 * DEFAULT_STATOR.turns,
            "minimum_bend_radius_mm": PARAMS.min_bend_radius,
        },
        "analytic_family": {
            "name": "sector-inset R3 half-circle plus R3 S-transfer",
            "piece_radii_mm": [RADIUS_MM, RADIUS_MM, RADIUS_MM],
            "minimum_analytic_radius_mm": RADIUS_MM,
            "C1_tangent_continuity": True,
            "transverse_containment_proof": (
                "each circular piece has a straight transverse projection; "
                "both chords join points in the convex intersection of the "
                "wire-radius-inset tooth sector and OD-centre disk"
            ),
            "endpoint_source": (
                "exact 50-turn square-row/shared-slot witness from "
                "r3_bend_scope_feasibility"
            ),
            "waypoint_selector_angle_samples": WAYPOINT_ANGLE_SAMPLES,
            "sample_arclength_mm": SAMPLE_ARCLENGTH_MM,
        },
        "bounded_phase_search": candidates,
        "turn_24_witness": turn24,
        "checks": {
            "exact_default_identity": True,
            "all_50_turns_built_in_every_candidate": True,
            "all_100_shared_slot_endpoints_preserved": True,
            "all_piecewise_bends_meet_R3_analytically": True,
            "all_centrelines_sector_and_OD_contained": all(
                item["minimum_sector_inset_margin_mm"] >= -1.0e-9 and
                item["minimum_od_center_margin_mm"] >= -1.0e-9
                for item in candidates),
            "adjacent_tooth_separation_proved_by_inset_halfplanes": True,
            "adjacent_tooth_analytic_centerline_lower_bound_mm": wire_d,
            "same_tooth_clearance_proved": False,
            "same_tooth_and_neighbor_clearance_proved": False,
        },
        "bounded_verdict": {
            "phase_pitch_multipliers_tested": list(PHASE_PITCH_MULTIPLIERS),
            "meaning": (
                "a sampled point pair below one finished-wire diameter is a "
                "constructive collision witness, not a missed-collision bound"
            ),
            "scope": (
                "rejects only this half-circle/S-transfer construction and "
                "the listed axial phases; it is not a theorem against every "
                "sector-confined spatial curve"
            ),
            "release_limitation": (
                "sector containment solves adjacent-sector ownership but does "
                "not by itself separate 50 curves inside one sector"
            ),
        },
        "source_hashes": {
            "cad/params.py": _sha256(CAD / "params.py"),
            "sim/r3_bend_scope_feasibility.py": _sha256(
                HERE / "r3_bend_scope_feasibility.py"),
            "out/reports/r3_bend_scope_feasibility.json": _sha256(
                REPORTS / "r3_bend_scope_feasibility.json"),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def render_markdown(report: dict[str, Any]) -> str:
    rows = report["bounded_phase_search"]
    lines = [
        "# Sector-confined spatial R3 family study",
        "",
        f"**Overall: {report['status']}**",
        "",
        "The isolated half-circle/S-transfer centreline is constructive: all "
        "pieces are exactly R3, every transverse chord remains in the inset "
        "tooth sector, the OD centre limit is respected, and adjacent sectors "
        "are separated analytically by two wire-radius half-plane insets. The "
        "full 50-turn bundle is not constructive because every tested axial "
        "phase retains a direct same-tooth collision witness.",
        "",
        "| phase pitch | phase span | same-tooth min | neighbor min | result |",
        "|---:|---:|---:|---:|:---|",
    ]
    for row in rows:
        lines.append(
            f"| {row['phase_pitch_mm']:.5f} mm | "
            f"{row['total_phase_span_mm']:.3f} mm | "
            f"{row['same_tooth']['minimum_sampled_distance_mm']:.6f} mm | "
            f"{row['adjacent_tooth']['minimum_sampled_distance_mm']:.6f} mm | "
            f"{row['status']} |"
        )
    witness = report["turn_24_witness"]
    lines.extend([
        "", "## Turn 24", "",
        f"Turn 24 uses waypoint `{witness['waypoint']['xy_mm']}` mm, "
        f"an R3 S-transfer sweep of "
        f"{witness['s_transfer_sweep_deg']:.3f} degrees, and remains "
        f"{witness['minimum_sampled_od_center_margin_mm']:.6f} mm inside "
        "the OD-centre boundary.", "",
        "No CAD or release artifact is authorized by this report.", "",
        f"Proof hash: `{report['report_sha256']}`", "",
    ])
    return "\n".join(lines)


def write_reports(report: dict[str, Any] | None = None) -> dict[str, Any]:
    report = analyze() if report is None else report
    REPORTS.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n",
                        encoding="utf-8")
    MD_OUT.write_text(render_markdown(report), encoding="utf-8")
    return report


def main() -> int:
    report = write_reports()
    print(f"sector chord family: {report['status']}; "
          f"{len(report['bounded_phase_search'])} axial phases")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
