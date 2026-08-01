"""Advisory audit of the literal three-millimetre wire bend rule.

This study is intentionally independent of the release honeycomb and does not
modify CAD, settings, controller captures, or production gates.  It answers a
smaller specification question: is R >= 3 mm intrinsically incompatible with
the exact DEFAULT_STATOR tooth/slot geometry?

The constructive witness uses a square-row packing in the exact two-neck slot
centre domain and an analytic L-R-L bounded-curvature crown.  It proves one
50-turn tooth bundle and the 100 side centres in each shared slot can be
nonpenetrating inside an OD46 radial cylinder.  It does *not* prove that the
current flyer deposits this topology, that all 24 neighbouring crowns can be
routed progressively, or that a real rotor/end bell has the required axial
cavity.
"""

from __future__ import annotations

import argparse
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable

import numpy as np
from scipy.spatial import cKDTree


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
OUTPUT_JSON = REPORTS / "r3_bend_scope_feasibility.json"
OUTPUT_MD = REPORTS / "r3_bend_scope_feasibility.md"
RAW_CYCLE = REPORTS / "upstream_current_raw_cycle.json"
GOAL = ROOT.parent / "GOAL.md"

if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

from params import DEFAULT_STATOR, PARAMS  # noqa: E402
import coil_growth  # noqa: E402


SCHEMA = "r3-bend-scope-feasibility/v1"
LINER_THICKNESS_MM = 0.127
MINIMUM_RADIUS_MM = 3.0
FIRST_ROW = 4
LAST_ROW = 27
MAXIMUM_LAYER_INDEX = 3
SAMPLE_STEP_DEG = 0.25


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: dict[str, Any]) -> str:
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _require_default() -> None:
    actual = (
        DEFAULT_STATOR.slots,
        DEFAULT_STATOR.od,
        DEFAULT_STATOR.stack,
        DEFAULT_STATOR.wire_d,
        DEFAULT_STATOR.turns,
        PARAMS.min_bend_radius,
    )
    expected = (24, 46.0, 15.0, 0.22352, 50, 3.0)
    if actual != expected:
        raise RuntimeError(
            "R3 advisory is pinned to DEFAULT_STATOR "
            f"{expected}, not {actual}"
        )


def geometry_constants() -> dict[str, float]:
    """Return the exact generated-stator constants used by the witness."""

    _require_default()
    slot = coil_growth.slot_geometry(DEFAULT_STATOR)
    beta = math.pi / DEFAULT_STATOR.slots
    wire_d = float(DEFAULT_STATOR.wire_d)
    wire_r = wire_d / 2.0
    centre_core = wire_r + LINER_THICKNESS_MM
    half_neck = float(slot["tooth_neck_width_mm"]) / 2.0
    access = (
        float(slot["tooth_neck_width_mm"])
        + wire_d + 2.0 * LINER_THICKNESS_MM
    ) / (2.0 * math.sin(beta))
    radial_end = (
        float(slot["shoe_inner_radius_mm"])
        - coil_growth.DEFAULT_POLICY.radial_end_clearance_mm
    )
    return {
        "beta_rad": beta,
        "half_pitch_deg": math.degrees(beta),
        "wire_d_mm": wire_d,
        "wire_r_mm": wire_r,
        "liner_mm": LINER_THICKNESS_MM,
        "centre_core_clearance_mm": centre_core,
        "half_neck_mm": half_neck,
        "slot_access_radius_mm": access,
        "slot_radial_end_mm": radial_end,
        "stator_outer_radius_mm": float(DEFAULT_STATOR.od) / 2.0,
        "half_stack_mm": float(DEFAULT_STATOR.stack) / 2.0,
    }


def square_row_centres() -> list[dict[str, float | int]]:
    """Construct 50 centres on one coil side without honeycomb assumptions.

    Slot coordinates use ``u`` along the slot bisector and ``v`` tangential.
    The negative side is the set belonging to the lower-angle tooth.  Its
    mirror belongs to the neighbouring tooth.  Rows are one wire diameter
    apart; centres within a row are also one diameter apart.
    """

    g = geometry_constants()
    beta = g["beta_rad"]
    d = g["wire_d_mm"]
    access = g["slot_access_radius_mm"]
    rows: list[dict[str, float | int]] = []
    turn = 0
    for row_index in range(FIRST_ROW, LAST_ROW + 1):
        capacity = math.floor(
            row_index * math.tan(beta) + 0.5 + 1.0e-12
        )
        u = access + row_index * d
        half_domain = row_index * d * math.tan(beta)
        for layer in range(capacity):
            v = -half_domain + layer * d
            x = u * math.cos(beta) - v * math.sin(beta)
            y = u * math.sin(beta) + v * math.cos(beta)
            rows.append({
                "turn_index": turn,
                "row_index": row_index,
                "layer_index": layer,
                "slot_u_mm": u,
                "slot_v_mm": v,
                "tooth_x_mm": x,
                "tooth_half_span_mm": y,
            })
            turn += 1
    if len(rows) != DEFAULT_STATOR.turns:
        raise RuntimeError(f"square-row witness generated {len(rows)} turns")
    return rows


def shared_slot_centres() -> np.ndarray:
    """Return both mirrored 50-centre coil sides in one exact shared slot."""

    rows = square_row_centres()
    negative = np.asarray([
        (float(row["slot_u_mm"]), float(row["slot_v_mm"]))
        for row in rows
    ])
    positive = negative * np.array((1.0, -1.0))
    return np.vstack((negative, positive))


def minimum_pair_distance(points: np.ndarray) -> float:
    tree = cKDTree(np.asarray(points, dtype=float))
    distances, _ = tree.query(points, k=2)
    return float(np.min(distances[:, 1]))


def lrl_parameters() -> dict[str, float]:
    """Analytic parallel-offset L-R-L crown parameters.

    The base curve starts in the front-face y-z plane at ``(-s0, 0)``
    heading +z.  Arc signs are (+1,-1,+1), with sweep angles
    ``(alpha, 2*alpha+pi, alpha)``.  A left-normal offset ``q`` increases the
    endpoint half-span by q.  Choosing Rb=3+qmax leaves the inward offset of
    the outermost layer at exactly R3.
    """

    g = geometry_constants()
    beta = g["beta_rad"]
    d = g["wire_d_mm"]
    q_step = d * math.cos(beta)
    q_max = MAXIMUM_LAYER_INDEX * q_step
    base_radius = MINIMUM_RADIUS_MM + q_max
    s0 = g["half_neck_mm"] + g["centre_core_clearance_mm"]
    alpha = math.acos((1.0 + s0 / base_radius) / 2.0)
    return {
        "base_radius_mm": base_radius,
        "offset_step_mm": q_step,
        "maximum_offset_mm": q_max,
        "base_half_span_mm": s0,
        "alpha_rad": alpha,
        "alpha_deg": math.degrees(alpha),
        "middle_sweep_rad": 2.0 * alpha + math.pi,
        "middle_sweep_deg": math.degrees(2.0 * alpha + math.pi),
    }


def _advance_arc(state: np.ndarray, sign: int, angle: float,
                 radius: float) -> np.ndarray:
    y, z, heading = map(float, state)
    sigma = int(sign)
    end = heading + sigma * float(angle)
    return np.asarray((
        y + radius / sigma * (math.sin(end) - math.sin(heading)),
        z + radius / sigma * (-math.cos(end) + math.cos(heading)),
        end,
    ))


def sample_base_lrl(step_deg: float = SAMPLE_STEP_DEG
                    ) -> tuple[np.ndarray, np.ndarray]:
    """Sample the base cap and return points plus unit tangents in y-z."""

    p = lrl_parameters()
    radius = p["base_radius_mm"]
    alpha = p["alpha_rad"]
    state = np.asarray((-p["base_half_span_mm"], 0.0, math.pi / 2.0))
    points: list[tuple[float, float]] = []
    tangents: list[tuple[float, float]] = []
    for arc_index, (sign, sweep) in enumerate((
        (1, alpha), (-1, 2.0 * alpha + math.pi), (1, alpha),
    )):
        count = max(2, math.ceil(math.degrees(sweep) / step_deg))
        values = np.linspace(0.0, sweep, count + 1)
        if arc_index:
            values = values[1:]
        for value in values:
            sample = _advance_arc(state, sign, float(value), radius)
            points.append((float(sample[0]), float(sample[1])))
            tangents.append((math.cos(float(sample[2])),
                             math.sin(float(sample[2]))))
        state = _advance_arc(state, sign, sweep, radius)
    expected = np.asarray((p["base_half_span_mm"], 0.0, -math.pi / 2.0))
    if not np.allclose(state, expected, atol=1.0e-10, rtol=0.0):
        raise RuntimeError("analytic L-R-L endpoint identity failed")
    return np.asarray(points), np.asarray(tangents)


def offset_cap(layer_index: int, step_deg: float = SAMPLE_STEP_DEG
               ) -> np.ndarray:
    """Return one layer's front crown as y-z points relative to the face."""

    if not 0 <= int(layer_index) <= MAXIMUM_LAYER_INDEX:
        raise ValueError("layer index must be in 0..3")
    points, tangents = sample_base_lrl(step_deg)
    normal = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    q = int(layer_index) * lrl_parameters()["offset_step_mm"]
    return points + q * normal


def full_loop_points(row: dict[str, float | int],
                     step_deg: float = SAMPLE_STEP_DEG) -> np.ndarray:
    """Sample one closed loop in the tooth-local x-y-z frame."""

    g = geometry_constants()
    x = float(row["tooth_x_mm"])
    s = float(row["tooth_half_span_mm"])
    layer = int(row["layer_index"])
    cap = offset_cap(layer, step_deg)
    top = np.column_stack((
        np.full(len(cap), x), cap[:, 0],
        cap[:, 1] + g["half_stack_mm"],
    ))
    bottom = top * np.array((1.0, 1.0, -1.0))
    side_count = max(3, math.ceil(DEFAULT_STATOR.stack / 0.05))
    z = np.linspace(-g["half_stack_mm"], g["half_stack_mm"], side_count)
    left = np.column_stack((np.full(len(z), x), np.full(len(z), -s), z))
    right = np.column_stack((np.full(len(z), x), np.full(len(z), s), z))
    return np.vstack((top, bottom, left, right))


def _sampled_bundle_clearance(rows: list[dict[str, float | int]],
                              step_deg: float = SAMPLE_STEP_DEG
                              ) -> tuple[float, tuple[int, int]]:
    loops = [full_loop_points(row, step_deg) for row in rows]
    best = math.inf
    pair = (-1, -1)
    for right in range(len(loops)):
        tree = cKDTree(loops[right])
        for left in range(right):
            value = float(np.min(tree.query(loops[left])[0]))
            if value < best:
                best = value
                pair = (left, right)
    return best, pair


@lru_cache(maxsize=1)
def analyze() -> dict[str, Any]:
    _require_default()
    g = geometry_constants()
    rows = square_row_centres()
    p = lrl_parameters()
    shared = shared_slot_centres()
    shared_min = minimum_pair_distance(shared)
    cap_bounds = []
    for layer in range(MAXIMUM_LAYER_INDEX + 1):
        cap = offset_cap(layer)
        q = layer * p["offset_step_mm"]
        cap_bounds.append({
            "layer_index": layer,
            "normal_offset_mm": q,
            "analytic_piece_radii_mm": [
                p["base_radius_mm"] - q,
                p["base_radius_mm"] + q,
                p["base_radius_mm"] - q,
            ],
            "minimum_radius_mm": p["base_radius_mm"] - q,
            "minimum_y_mm": float(np.min(cap[:, 0])),
            "maximum_y_mm": float(np.max(cap[:, 0])),
            "maximum_axial_rise_mm": float(np.max(cap[:, 1])),
        })
    bundle_min, bundle_pair = _sampled_bundle_clearance(rows)
    maximum_centre_radius = 0.0
    for row in rows:
        cap = offset_cap(int(row["layer_index"]))
        radius = np.sqrt(float(row["tooth_x_mm"]) ** 2 + cap[:, 0] ** 2)
        maximum_centre_radius = max(maximum_centre_radius, float(np.max(radius)))
    maximum_wire_radius = maximum_centre_radius + g["wire_r_mm"]
    axial_rise = max(item["maximum_axial_rise_mm"] for item in cap_bounds)
    total_axial_wire_envelope = (
        DEFAULT_STATOR.stack + 2.0 * (axial_rise + g["wire_r_mm"])
    )
    radial_values = [float(row["tooth_x_mm"]) for row in rows]
    layer_counts = [sum(int(row["layer_index"]) == layer for row in rows)
                    for layer in range(MAXIMUM_LAYER_INDEX + 1)]
    sector_half_width = min(radial_values) * math.tan(g["beta_rad"])
    crown_half_width = max(abs(item["minimum_y_mm"])
                           for item in cap_bounds)
    same_plane_sector_margin = sector_half_width - crown_half_width
    conformal_radius = g["centre_core_clearance_mm"]
    raw = json.loads(RAW_CYCLE.read_text(encoding="utf-8"))
    raw_contract = raw["checks"]["physical job contract captured"]["value"]
    raw_span = list(raw_contract["radial_winding_span_mm"])

    checks = {
        "exact shared slot has 100 nonpenetrating side centres": {
            "ok": shared_min + 1.0e-10 >= g["wire_d_mm"],
            "value_mm": shared_min,
            "requirement_mm": g["wire_d_mm"],
        },
        "square-row witness contains exactly 50 turns": {
            "ok": len(rows) == 50,
            "value": len(rows),
            "requirement": 50,
        },
        "witness radial centres fit raw M0 span": {
            "ok": min(radial_values) >= raw_span[0] - 1.0e-9
            and max(radial_values) <= raw_span[1] + 1.0e-9,
            "value_mm": [min(radial_values), max(radial_values)],
            "requirement_mm": raw_span,
        },
        "all analytic crown pieces meet R3": {
            "ok": min(item["minimum_radius_mm"] for item in cap_bounds)
            + 1.0e-12 >= MINIMUM_RADIUS_MM,
            "value_mm": min(item["minimum_radius_mm"]
                            for item in cap_bounds),
            "requirement_mm": MINIMUM_RADIUS_MM,
        },
        "sampled complete one-tooth loops are nonpenetrating": {
            "ok": bundle_min + 2.0e-6 >= g["wire_d_mm"],
            "value_mm": bundle_min,
            "requirement_mm": g["wire_d_mm"],
            "closest_turn_pair": list(bundle_pair),
            "sampling_step_deg": SAMPLE_STEP_DEG,
            "authority": (
                "representative bounded witness; row and corresponding-offset "
                "separations are analytic, nonlocal approach is sampled"
            ),
        },
        "wire outer surface remains inside OD46 radial cylinder": {
            "ok": maximum_wire_radius <= g["stator_outer_radius_mm"] + 1.0e-9,
            "value_mm": maximum_wire_radius,
            "requirement_mm": g["stator_outer_radius_mm"],
        },
    }
    witness_ok = all(item["ok"] for item in checks.values())
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "ADVISORY_COMPATIBLE" if witness_ok else "FAIL",
        "decision": (
            "NO_LITERAL_DEFAULT_TOOTH_GEOMETRIC_CONTRADICTION"
            if witness_ok else "CONSTRUCTIVE_WITNESS_FAILED"
        ),
        "production_authorized": False,
        "scope": {
            "question": (
                "Is a >=3 mm wire-centreline radius intrinsically impossible "
                "for DEFAULT_STATOR, independent of the release honeycomb?"
            ),
            "answer": (
                "No. An exact non-honeycomb shared-slot packing and a "
                "parallel-offset R3 L-R-L crown provide a constructive "
                "one-tooth 50-loop witness inside the OD46 radial cylinder."
            ),
            "proved": [
                "100 nonpenetrating side centres in one exact shared slot",
                "50 closed nonpenetrating R3 loops around one tooth",
                "all witness radial centres inside the captured raw M0 span",
                "wire outer surface inside a diameter-46 radial cylinder",
            ],
            "not_proved": [
                "the raw flyer motion deposits this alternative packing",
                "progressive flyer-to-slot access for the witness",
                "collision-free crowns for all 24 adjacent teeth",
                "a retained rotor/end-bell axial cavity",
                "every OD/wire/turn combination in the launch handling envelope",
            ],
        },
        "inputs": {
            "stator": {
                "slots": DEFAULT_STATOR.slots,
                "od_mm": DEFAULT_STATOR.od,
                "stack_mm": DEFAULT_STATOR.stack,
                "turns_per_tooth": DEFAULT_STATOR.turns,
                "wire_finished_diameter_mm": DEFAULT_STATOR.wire_d,
            },
            "slot_geometry": coil_growth.slot_geometry(DEFAULT_STATOR),
            "raw_m0_radial_span_mm": raw_span,
            "minimum_bend_radius_mm": PARAMS.min_bend_radius,
            "liner_thickness_mm": LINER_THICKNESS_MM,
        },
        "bend_scope_audit": {
            "machine_guides": {
                "rule": "R >= 3 mm remains literal and mandatory",
                "finding": (
                    "This witness does not weaken the spool, dancer, entry "
                    "tube, hollow-shaft, elbow, horn, or eyelet checks."
                ),
            },
            "tight_insulated_workpiece_conformity": {
                "buffered_sharp_corner_centerline_radius_mm": conformal_radius,
                "R3_shortfall_mm": MINIMUM_RADIUS_MM - conformal_radius,
                "finding": (
                    "A wire forced to follow the exact liner-buffered sharp "
                    "lamination corner cannot meet R3."
                ),
            },
            "free_end_turns": {
                "finding": (
                    "Detaching from the corner with the same axial tangent "
                    "allows the analytic L-R-L crown to meet R3."
                ),
                "constructive_witness": True,
            },
            "final_motor_fit": {
                "radial_status": (
                    "PASS" if checks[
                        "wire outer surface remains inside OD46 radial cylinder"
                    ]["ok"] else "FAIL"
                ),
                "maximum_wire_outer_radius_mm": maximum_wire_radius,
                "radial_margin_to_stator_od_cylinder_mm": (
                    g["stator_outer_radius_mm"] - maximum_wire_radius
                ),
                "candidate_total_axial_wire_envelope_mm": (
                    total_axial_wire_envelope
                ),
                "axial_status": "UNPROVEN",
                "reason": (
                    "GOAL.md defines stator stack and radial OD handling but "
                    "does not define the retained motor rotor/end-bell internal "
                    "axial cavity or an allowed finished-motor length."
                ),
            },
        },
        "square_row_witness": {
            "derivation": (
                "u_j=r_access+j*d; h_j=j*d*tan(beta); "
                "k_j=floor(j*tan(beta)+1/2); "
                "v_jn=-h_j+n*d"
            ),
            "row_range_inclusive": [FIRST_ROW, LAST_ROW],
            "layer_counts": layer_counts,
            "turn_count": len(rows),
            "shared_slot_center_count": len(shared),
            "minimum_shared_slot_center_distance_mm": shared_min,
            "tooth_radial_center_range_mm": [
                min(radial_values), max(radial_values),
            ],
            "outer_unused_raw_radial_margin_mm": raw_span[1] - max(radial_values),
            "centres": rows,
        },
        "lrl_crown_witness": {
            "frame": (
                "tooth-local x radial, y tangential, z axial; front cap "
                "starts at (x,-s,+stack/2) tangent +z"
            ),
            "parameterization": {
                **p,
                "arc_signs": [1, -1, 1],
                "arc_sweeps_rad": [
                    p["alpha_rad"], p["middle_sweep_rad"], p["alpha_rad"],
                ],
                "state_update": (
                    "h1=h0+sigma*theta; "
                    "y1=y0+(R/sigma)(sin(h1)-sin(h0)); "
                    "z1=z0+(R/sigma)(-cos(h1)+cos(h0))"
                ),
                "offset_rule": (
                    "gamma_q=gamma_0+q*(-sin(h),cos(h)) in y-z"
                ),
            },
            "layer_bounds": cap_bounds,
            "minimum_sampled_complete_loop_distance_mm": bundle_min,
            "closest_sampled_turn_pair": list(bundle_pair),
            "maximum_wire_center_radius_mm": maximum_centre_radius,
            "maximum_wire_outer_radius_mm": maximum_wire_radius,
            "maximum_axial_rise_beyond_each_stack_face_mm": axial_rise,
            "candidate_total_axial_wire_envelope_mm": total_axial_wire_envelope,
        },
        "adjacent_tooth_pitch": {
            "pitch_deg": 360.0 / DEFAULT_STATOR.slots,
            "minimum_tooth_sector_half_width_over_witness_mm": sector_half_width,
            "maximum_crown_half_width_mm": crown_half_width,
            "same_axial_plane_sector_margin_mm": same_plane_sector_margin,
            "status": "REQUIRES_3D_OR_AXIAL_STAGGERING",
            "interpretation": (
                "The constructive crown crosses a single tooth's angular "
                "sector. Repeating it coplanarly on all 24 teeth is not a "
                "valid full-stator construction. A former must prove axial/3D "
                "staggering and progressive neighbouring-copper clearance."
            ),
        },
        "naive_rounded_end_turn": {
            "family": "single planar semicircle between side tangents",
            "half_span_range_mm": [
                min(float(row["tooth_half_span_mm"]) for row in rows),
                max(float(row["tooth_half_span_mm"]) for row in rows),
            ],
            "radius_range_mm": [
                min(float(row["tooth_half_span_mm"]) for row in rows),
                max(float(row["tooth_half_span_mm"]) for row in rows),
            ],
            "status": "FAIL",
            "reason": "all direct semicircle radii are below 3 mm",
        },
        "checks": checks,
        "requirement_guidance": {
            "default_job": (
                "No radius reduction or stator-size change is justified by "
                "geometry alone; retain R3 while developing a proved former."
            ),
            "minimum_scope_clarification": (
                "State whether R3 applies to free-running guides and detached "
                "end-turn centreline, while ordinary supported conformity to "
                "insulated workpiece surfaces is exempt."
            ),
            "if_tight_corner_conformity_is_mandatory": {
                "maximum_radius_supported_by_exact_buffered_corner_mm": (
                    conformal_radius
                ),
                "required_change": (
                    "reduce the local supported-conformity radius to <= "
                    f"{conformal_radius:.5f} mm, or add an R3 former that "
                    "keeps the wire detached from the corner"
                ),
            },
            "missing_motor_contract": (
                "Add a rotor inner radius, axial cavity, end-bell clearance, "
                "and finished-motor envelope before claiming final motor fit."
            ),
        },
        "source_sha256": {
            "GOAL.md": _sha256(GOAL),
            "cad/params.py": _sha256(CAD / "params.py"),
            "cad/stator_model.py": _sha256(CAD / "stator_model.py"),
            "cad/coil_growth.py": _sha256(CAD / "coil_growth.py"),
            "out/reports/upstream_current_raw_cycle.json": _sha256(RAW_CYCLE),
        },
    }
    payload = dict(report)
    report["report_sha256"] = _canonical_hash(payload)
    return report


def render_markdown(report: dict[str, Any]) -> str:
    slot = report["square_row_witness"]
    lrl = report["lrl_crown_witness"]
    motor = report["bend_scope_audit"]["final_motor_fit"]
    adjacent = report["adjacent_tooth_pitch"]
    conform = report["bend_scope_audit"][
        "tight_insulated_workpiece_conformity"
    ]
    lines = [
        "# R3 bend-scope feasibility audit",
        "",
        f"**Result: {report['status']}**",
        "",
        report["scope"]["answer"],
        "",
        "This is advisory evidence only. It does not authorize production "
        "geometry or replace the raw-motion wire-path gate.",
        "",
        "## Constructive default-tooth witness",
        "",
        f"- Exact non-honeycomb square rows place {slot['turn_count']} centres "
        f"per coil side and {slot['shared_slot_center_count']} centres in a "
        "shared slot.",
        f"- Minimum shared-slot centre distance: "
        f"{slot['minimum_shared_slot_center_distance_mm']:.6f} mm for "
        f"{DEFAULT_STATOR.wire_d:.6f} mm finished wire.",
        f"- Tooth-radial centres: "
        f"{slot['tooth_radial_center_range_mm'][0]:.6f}.."
        f"{slot['tooth_radial_center_range_mm'][1]:.6f} mm, inside the raw "
        f"{report['inputs']['raw_m0_radial_span_mm'][0]:.6f}.."
        f"{report['inputs']['raw_m0_radial_span_mm'][1]:.6f} mm span.",
        f"- Parallel-offset L-R-L cap: base R "
        f"{lrl['parameterization']['base_radius_mm']:.6f} mm; outermost "
        "inward offset is exactly R3.",
        f"- Sampled full one-tooth bundle minimum: "
        f"{lrl['minimum_sampled_complete_loop_distance_mm']:.6f} mm.",
        "",
        "## Why the ordinary semicircle is misleading",
        "",
        "The direct U-turn radius equals the side half-span, only "
        f"{report['naive_rounded_end_turn']['half_span_range_mm'][0]:.6f}.."
        f"{report['naive_rounded_end_turn']['half_span_range_mm'][1]:.6f} mm. "
        "That family fails R3, but the three-arc L-R-L family does not.",
        "",
        "## Scope of the 3 mm rule",
        "",
        "- **Machine guides:** R3 remains literal for the dancer, entry tube, "
        "hollow shaft, elbow, horn, and eyelet.",
        "- **Tight insulated conformity:** an exact buffered sharp corner has "
        f"only R{conform['buffered_sharp_corner_centerline_radius_mm']:.5f}; "
        "forcing contact there contradicts R3.",
        "- **Free end turn:** detachment with the same axial tangent permits "
        "the constructive R3 crown.",
        f"- **Radial motor envelope:** wire outer radius "
        f"{motor['maximum_wire_outer_radius_mm']:.6f} mm leaves "
        f"{motor['radial_margin_to_stator_od_cylinder_mm']:.6f} mm inside "
        "the OD46 cylinder.",
        f"- **Axial motor envelope:** **UNPROVEN**. The candidate occupies "
        f"{motor['candidate_total_axial_wire_envelope_mm']:.6f} mm axially, "
        "but no rotor/end-bell internal cavity is specified.",
        "",
        "## Adjacent teeth",
        "",
        f"The crown's same-plane sector margin is "
        f"{adjacent['same_axial_plane_sector_margin_mm']:.6f} mm. "
        "It crosses the single-tooth sector, so a 24-tooth implementation "
        "requires a proved 3D/axial stagger and progressive-copper audit. "
        "This report deliberately does not infer that proof from the local "
        "witness.",
        "",
        "## Specification action",
        "",
        "Do not reduce R3 for the default job on geometry evidence alone. "
        "Clarify that supported workpiece conformity is either exempt or "
        "formed away from a sharp corner, and add the missing rotor inner "
        "radius/axial-cavity/end-bell envelope before claiming final motor fit.",
        "",
        "## Limits",
        "",
    ]
    for item in report["scope"]["not_proved"]:
        lines.append(f"- {item}")
    lines.append("")
    return "\n".join(lines)


def write_report(json_path: Path = OUTPUT_JSON,
                 markdown_path: Path = OUTPUT_MD) -> dict[str, Any]:
    report = analyze()
    json_path.parent.mkdir(parents=True, exist_ok=True)
    markdown_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    markdown_path.write_text(render_markdown(report), encoding="utf-8")
    return report


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", type=Path, default=OUTPUT_JSON)
    parser.add_argument("--markdown", type=Path, default=OUTPUT_MD)
    args = parser.parse_args(list(argv) if argv is not None else None)
    report = write_report(args.json, args.markdown)
    print(
        "R3 scope feasibility: "
        f"{report['status']}; production_authorized="
        f"{report['production_authorized']}; wrote {args.json} and "
        f"{args.markdown}"
    )
    return 0 if report["status"] == "ADVISORY_COMPATIBLE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
