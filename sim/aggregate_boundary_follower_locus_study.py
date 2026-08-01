"""Exact-locus feasibility study for the floating aggregate follower.

This study deliberately stops before CAD or route authorization.  It asks two
bounded questions against the canonical 2,400 terminal-locus stream:

* does a supporting tangent from the current cap terminal to the exposed
  nested aggregate triangle exist without assigning strand centres; and
* do the proposed 6 mm radial / 1 mm per-identity tangential slides cover the
  resulting contact loci?

The answer is useful but not sufficient.  A straight cap-to-contact span is
also checked against the exact cap-terminal tangent.  Any mismatch requires a
positive-volume R3 follower arc; the study never promotes a kink or a free
mathematical detour to wire authority.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORTS = ROOT / "out" / "reports"
LOCI_PATH = REPORTS / "carriage_active_sector_terminal_guide_loci.json"
AGGREGATE_PATH = REPORTS / "permanent_cap_aggregate_authorization.json"
OUTPUT_JSON = REPORTS / "aggregate_boundary_follower_locus_study.json"
OUTPUT_MD = REPORTS / "aggregate_boundary_follower_locus_study.md"

SCHEMA = "aggregate-boundary-follower-locus-study/v1"
EXPECTED_LOCI = 2400
TURNS_PER_TOOTH = 50
REQUIRED_CENTERLINE_RADIUS_MM = 3.0
PROTOTYPE_RADIAL_STROKE_MM = 6.0
PROTOTYPE_TANGENTIAL_STROKE_PER_IDENTITY_MM = 1.0
PROTOTYPE_GIMBAL_HALF_RANGE_DEG = 65.0
C1_TOLERANCE_DEG = 1.0e-4
SUPPORT_TOLERANCE_MM = 1.0e-9


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(value: Any, field: str | None = None) -> str:
    body = deepcopy(value)
    if field is not None and isinstance(body, dict):
        body.pop(field, None)
    return hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def _side_and_axial_sign(lane_id: str) -> tuple[int, int]:
    if "_left_" in lane_id:
        side = -1
    elif "_right_" in lane_id:
        side = 1
    else:
        raise ValueError(f"unrecognized side in {lane_id!r}")
    if lane_id.endswith("_front"):
        axial = 1
    elif lane_id.endswith("_rear"):
        axial = -1
    else:
        raise ValueError(f"unrecognized axial end in {lane_id!r}")
    return side, axial


def _unit(vector: Sequence[float]) -> tuple[float, ...]:
    length = math.sqrt(sum(float(value) ** 2 for value in vector))
    if length <= 1.0e-15:
        raise ValueError("zero-length vector")
    return tuple(float(value) / length for value in vector)


def _support_candidates(
    point_xy: tuple[float, float],
    triangle: list[tuple[float, float, str]],
) -> list[dict[str, Any]]:
    """Return vertex tangents whose line supports the complete triangle.

    For an external point and a convex polygon, every supporting tangent
    touches a vertex unless the point is collinear with an edge; the edge's
    endpoints remain valid witnesses in that case.  A/B/C all belong to at
    least one exposed edge (AC or BC); the shared AB interior is never used.
    """

    px, py = point_xy
    result: list[dict[str, Any]] = []
    for qx, qy, vertex in triangle:
        dx, dy = px - qx, py - qy
        length = math.hypot(dx, dy)
        if length <= 1.0e-15:
            continue
        for normal_sign in (-1.0, 1.0):
            nx = normal_sign * (-dy / length)
            ny = normal_sign * (dx / length)
            signed = [
                nx * (x - qx) + ny * (y - qy)
                for x, y, _name in triangle
            ]
            if max(signed) <= SUPPORT_TOLERANCE_MM:
                result.append({
                    "vertex": vertex,
                    "contact_xy_mm": [qx, qy],
                    "outward_normal_xy": [nx, ny],
                    "maximum_triangle_support_residual_mm": max(signed),
                })
    return result


def _locus_row(
    locus: Mapping[str, Any], *, u0: float, uc: float, wc: float,
    connector_z: float,
) -> dict[str, Any]:
    binding = locus["terminal_binding"]
    lane_id = str(binding["lane_id"])
    side_sign, axial_sign = _side_and_axial_sign(lane_id)
    turn = int(locus["turn_index"])
    half = int(locus["half_turn_index"])
    g = turn / TURNS_PER_TOOTH
    g_other = (turn + half) / TURNS_PER_TOOTH
    root_g = math.sqrt(g)
    ug = u0 + root_g * (uc - u0)
    wg = root_g * wc
    triangle = [
        (u0, 0.0, "A"),
        (ug, 0.0, "B"),
        (ug, side_sign * wg, "C"),
    ]
    endpoint = list(map(float, binding["cap_endpoint_local_mm"]))
    owner = (
        "nomex_liner_or_permanent_cap"
        if turn == 0 else "exposed_prior_active_aggregate"
    )
    candidates = [] if turn == 0 else _support_candidates(
        (endpoint[0], endpoint[1]), triangle,
    )
    selected = None
    if candidates:
        for row in candidates:
            qx, qy = row["contact_xy_mm"]
            qz = axial_sign * connector_z
            span = (qx - endpoint[0], qy - endpoint[1], qz - endpoint[2])
            tangent = _unit(span)
            guide_tangent = (0.0, 0.0, -float(axial_sign))
            cosine = max(-1.0, min(1.0, sum(
                tangent[index] * guide_tangent[index] for index in range(3)
            )))
            error = math.degrees(math.acos(cosine))
            row.update({
                "contact_local_mm": [qx, qy, qz],
                "straight_span_length_mm": math.sqrt(sum(v * v for v in span)),
                "straight_span_tangent_local": list(tangent),
                "guide_terminal_tangent_local": list(guide_tangent),
                "guide_C1_error_deg": error,
                "minimum_R3_arc_length_to_absorb_error_mm": (
                    REQUIRED_CENTERLINE_RADIUS_MM * math.radians(error)
                ),
                "minimum_R3_lateral_sweep_mm": (
                    REQUIRED_CENTERLINE_RADIUS_MM
                    * (1.0 - math.cos(math.radians(error)))
                ),
            })
        selected = min(candidates, key=lambda row: (
            row["straight_span_length_mm"], row["vertex"],
        ))
    return {
        "locus_index": int(locus["locus_index"]),
        "pass_index": int(locus["pass_index"]),
        "state_index": int(locus["state_index"]),
        "turn_index": turn,
        "half_turn_index": half,
        "tooth_index": int(locus["tooth_index"]),
        "time_s": float(locus["time_s"]),
        "lane_id": lane_id,
        "side_sign": side_sign,
        "axial_sign": axial_sign,
        "g_current": g,
        "g_other": g_other,
        "u_g_mm": ug,
        "w_g_mm": wg,
        "triangle_vertices_uv_mm": [list(row[:2]) for row in triangle],
        "contact_owner": owner,
        "aggregate_normal_available": turn > 0,
        "support_candidate_count": len(candidates),
        "selected_support": selected,
        "direct_straight_span_C1": (
            selected is not None
            and selected["guide_C1_error_deg"] <= C1_TOLERANCE_DEG
        ),
    }


def analyze(
    loci_path: Path = LOCI_PATH,
    aggregate_path: Path = AGGREGATE_PATH,
) -> dict[str, Any]:
    loci_path = Path(loci_path)
    aggregate_path = Path(aggregate_path)
    loci_document = _load(loci_path)
    aggregate = _load(aggregate_path)
    loci = loci_document.get("loci", [])
    partition = aggregate["slot_partition"]
    u0 = float(partition["u_start_mm"])
    uc = float(partition["u_cutoff_mm"])
    wc = float(partition["cutoff_half_width_mm"])
    wire_r = float(partition["wire_radius_mm"])
    liner = float(partition["liner_thickness_mm"])
    stack_half = float(aggregate["inputs"]["stator"]["stack_mm"]) / 2.0
    connector_z = stack_half + wire_r + liner
    rows = [
        _locus_row(
            locus, u0=u0, uc=uc, wc=wc, connector_z=connector_z,
        )
        for locus in loci
    ]
    selected = [row["selected_support"] for row in rows
                if row["selected_support"] is not None]
    radial_positions = [float(row["contact_local_mm"][0]) for row in selected]
    tangential_positions = [
        abs(float(row["contact_local_mm"][1])) for row in selected
    ]
    errors = [float(row["guide_C1_error_deg"]) for row in selected]
    required_radial = (
        max(radial_positions) - min(radial_positions)
        if radial_positions else math.inf
    )
    required_tangential = max(tangential_positions, default=math.inf)
    gates = {
        "aggregate_report_self_hash_and_PASS": (
            aggregate.get("report_sha256")
            == _canonical_hash(aggregate, "report_sha256")
            and aggregate.get("status") == "PASS"
        ),
        "locus_document_self_hash": (
            loci_document.get("locus_payload_sha256")
            == _canonical_hash(loci_document, "locus_payload_sha256")
        ),
        "exact_2400_loci": len(rows) == EXPECTED_LOCI,
        "all_nonzero_growth_loci_have_supporting_tangent": (
            len(selected) == EXPECTED_LOCI - 48
        ),
        "prototype_radial_stroke_covers_selected_contacts": (
            required_radial <= PROTOTYPE_RADIAL_STROKE_MM + 1.0e-12
        ),
        "prototype_tangential_stroke_per_identity_covers_contacts": (
            required_tangential
            <= PROTOTYPE_TANGENTIAL_STROKE_PER_IDENTITY_MM + 1.0e-12
        ),
        "prototype_gimbal_range_covers_direct_turn": (
            max(errors, default=math.inf)
            <= PROTOTYPE_GIMBAL_HALF_RANGE_DEG + 1.0e-12
        ),
        "g0_cap_or_liner_BREP_normal_supplied": False,
        "direct_span_is_C1_at_all_nonzero_growth_loci": (
            bool(selected) and all(
                row["direct_straight_span_C1"]
                for row in rows if row["turn_index"] > 0
            )
        ),
        "positive_volume_R3_arc_placement_proven": False,
        "continuous_intra_half_turn_follower_law_proven": False,
        "swept_rigid_copper_self_clearance_proven": False,
        "dancer_feed_length_and_dynamic_tension_proven": False,
    }
    passed = all(gates.values())
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "PASS" if passed else "FAIL",
        "decision": (
            "FLOATING_FOLLOWER_LOCUS_FAMILY_PROVEN"
            if passed else "SLIDE_ENVELOPE_COVERS_TANGENTS__PHYSICAL_R3_ROUTE_NOT_PROVEN"
        ),
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "scope": {
            "proved": [
                "exact equal-area triangular S_g connector sublevels",
                "support-line tangency at every nonzero-growth locus",
                "selected contact travel bounds against 6 mm / 1 mm slides",
                "direct cap-terminal C1 mismatch and minimum R3 turn demand",
            ],
            "not_proved": [
                "g=0 physical cap/liner normal",
                "positive-volume R3 arc location and clearance",
                "continuous intra-half-turn follower law",
                "swept rigid, copper, neighbor, and self clearance",
                "feed-length history, dancer dynamics, friction, wear, or settling",
            ],
        },
        "geometry": {
            "u_start_mm": u0,
            "u_cutoff_mm": uc,
            "cutoff_half_width_mm": wc,
            "connector_z_abs_mm": connector_z,
            "sublevel_formula": {
                "u_g": "u0 + sqrt(g)*(u_cutoff-u0)",
                "w_g": "sqrt(g)*w_cutoff",
                "triangle": "A=(u0,0), B=(u_g,0), C=(u_g,side*w_g)",
            },
        },
        "prototype_contract": {
            "radial_stroke_mm": PROTOTYPE_RADIAL_STROKE_MM,
            "tangential_stroke_per_identity_mm": (
                PROTOTYPE_TANGENTIAL_STROKE_PER_IDENTITY_MM
            ),
            "gimbal_half_range_deg": PROTOTYPE_GIMBAL_HALF_RANGE_DEG,
            "minimum_wire_centerline_radius_mm": REQUIRED_CENTERLINE_RADIUS_MM,
        },
        "coverage": {
            "required_loci": EXPECTED_LOCI,
            "evaluated_loci": len(rows),
            "zero_growth_loci_missing_physical_normal": sum(
                row["turn_index"] == 0 for row in rows
            ),
            "nonzero_growth_support_tangent_count": len(selected),
            "nonzero_growth_direct_C1_count": sum(
                row["direct_straight_span_C1"] for row in rows
                if row["turn_index"] > 0
            ),
        },
        "travel_and_turn_bounds": {
            "selected_contact_radial_min_mm": min(radial_positions, default=None),
            "selected_contact_radial_max_mm": max(radial_positions, default=None),
            "required_radial_travel_mm": required_radial,
            "required_tangential_travel_per_identity_mm": required_tangential,
            "minimum_direct_span_C1_error_deg": min(errors, default=None),
            "maximum_direct_span_C1_error_deg": max(errors, default=None),
            "maximum_minimum_R3_arc_length_mm": max(
                (row["minimum_R3_arc_length_to_absorb_error_mm"]
                 for row in selected), default=None,
            ),
            "maximum_minimum_R3_lateral_sweep_mm": max(
                (row["minimum_R3_lateral_sweep_mm"]
                 for row in selected), default=None,
            ),
        },
        "gates": gates,
        "loci": rows,
        "input_files": {
            "terminal_loci": str(loci_path.relative_to(ROOT)).replace("\\", "/"),
            "aggregate_authority": str(aggregate_path.relative_to(ROOT)).replace("\\", "/"),
        },
        "input_sha256": {
            "terminal_loci": _sha256(loci_path),
            "aggregate_authority": _sha256(aggregate_path),
        },
        "source_hashes": {
            "sim/aggregate_boundary_follower_locus_study.py": _sha256(
                Path(__file__)
            ),
        },
    }
    report["report_sha256"] = _canonical_hash(report, "report_sha256")
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported floating-follower locus schema")
    if report.get("report_sha256") != _canonical_hash(
            report, "report_sha256"):
        raise ValueError("floating-follower locus report hash mismatch")
    for relative, expected in report.get("source_hashes", {}).items():
        path = ROOT / str(relative).replace("/", "\\")
        if not path.is_file() or _sha256(path) != expected:
            raise ValueError(f"stale floating-follower source {relative}")
    for name, relative in report.get("input_files", {}).items():
        path = ROOT / str(relative).replace("/", "\\")
        if not path.is_file() or _sha256(path) != report["input_sha256"].get(name):
            raise ValueError(f"stale floating-follower input {name}")
    expected_status = "PASS" if all(report.get("gates", {}).values()) else "FAIL"
    if report.get("status") != expected_status:
        raise ValueError("floating-follower status/gate mismatch")
    if report.get("production_authorized") is not False:
        raise ValueError("locus study cannot authorize production")


def render_markdown(report: Mapping[str, Any]) -> str:
    bounds = report["travel_and_turn_bounds"]
    coverage = report["coverage"]
    lines = [
        "# Aggregate-boundary floating follower locus study", "",
        f"**{report['status']} — {report['decision']}**", "",
        "The exact aggregate tangent contacts fit the proposed slide envelope, "
        "but the direct span is never C1. A real R3 nose arc is mandatory.", "",
        "## Coverage", "",
        f"- Loci: {coverage['evaluated_loci']} / {coverage['required_loci']}",
        f"- Nonzero-growth supporting tangents: {coverage['nonzero_growth_support_tangent_count']}",
        f"- Direct C1 continuations: {coverage['nonzero_growth_direct_C1_count']}",
        f"- g=0 loci missing a physical cap/liner normal: {coverage['zero_growth_loci_missing_physical_normal']}",
        "", "## Bounds", "",
        f"- Required selected radial travel: {bounds['required_radial_travel_mm']:.6f} mm",
        f"- Required tangential travel per identity: {bounds['required_tangential_travel_per_identity_mm']:.6f} mm",
        f"- Direct C1 error: {bounds['minimum_direct_span_C1_error_deg']:.6f} to {bounds['maximum_direct_span_C1_error_deg']:.6f} deg",
        f"- Largest minimum R3 arc length: {bounds['maximum_minimum_R3_arc_length_mm']:.6f} mm",
        "", "## Gates", "",
    ]
    lines.extend(
        f"- {'PASS' if value else 'OPEN'} — `{name}`"
        for name, value in report["gates"].items()
    )
    lines.extend(["", "Production and assembly integration remain unauthorized.", ""])
    return "\n".join(lines)


def write_outputs(report: Mapping[str, Any] | None = None) -> dict[str, Any]:
    result = dict(report or analyze())
    validate_report_integrity(result)
    OUTPUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    OUTPUT_MD.write_text(render_markdown(result), encoding="utf-8")
    return result


def main() -> int:
    report = write_outputs()
    print(
        f"aggregate follower locus {report['status']}: "
        f"tangents={report['coverage']['nonzero_growth_support_tangent_count']}; "
        f"C1={report['coverage']['nonzero_growth_direct_C1_count']}"
    )
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
