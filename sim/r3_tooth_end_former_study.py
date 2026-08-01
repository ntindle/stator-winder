"""Fail-closed retained tooth-end-former successor study.

This study consumes the exact advisory L-R-L R3 witness, turns it into a
bounded two-piece dielectric end-cap candidate, and adds the gates which that
advisory deliberately does not claim: all 24 neighbouring crowns, retained
motor envelope, physical former overlap, and the captured raw machine cycle.

No production CAD, controller, capture, BOM, or release allow-list is changed.
The isolated candidate remains unauthorized unless every gate in this report
passes.
"""

from __future__ import annotations

from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import fcl
import numpy as np
import trimesh


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
SPEC_REPORT = REPORTS / "r3_bend_scope_feasibility.json"
JSON_OUT = REPORTS / "r3_tooth_end_former.json"
MD_OUT = REPORTS / "r3_tooth_end_former.md"
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

from params import DEFAULT_STATOR, PARAMS  # noqa: E402
import coil_growth  # noqa: E402
import r3_tooth_end_former as former  # noqa: E402
import r3_bend_scope_feasibility as scope  # noqa: E402
import collide  # noqa: E402
from traj import Timeline, load_events  # noqa: E402
from slot_route import CopperField, CopperPolyline  # noqa: E402


SCHEMA = "r3-tooth-end-former-study/v1"
WIRE_CLEARANCE_MM = float(DEFAULT_STATOR.wire_d)
RIGID_CLEARANCE_MM = float(PARAMS.dyn_clearance)
LANE_SWEEP_MM = (0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_hash(payload: dict[str, Any]) -> str:
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _require_identity() -> dict[str, Any]:
    report = json.loads(SPEC_REPORT.read_text(encoding="utf-8"))
    if (report.get("schema") != "r3-bend-scope-feasibility/v1"
            or report.get("status") != "ADVISORY_COMPATIBLE"
            or report.get("production_authorized") is not False):
        raise RuntimeError("R3 scope advisory identity/status drifted")
    expected = scope.lrl_parameters()
    checks = {
        "wire_d": (former.WIRE_DIAMETER_MM, DEFAULT_STATOR.wire_d),
        "liner": (former.LINER_THICKNESS_MM, scope.LINER_THICKNESS_MM),
        "base_radius": (
            former.BASE_WIRE_RADIUS_MM, expected["base_radius_mm"]),
        "q_step": (former.PACKING_Q_STEP_MM, expected["offset_step_mm"]),
        "q_max": (former.PACKING_Q_MAX_MM, expected["maximum_offset_mm"]),
        "alpha": (former.BASE_FIRST_ARC_RAD, expected["alpha_rad"]),
    }
    mismatch = [
        f"{name}: {actual} != {wanted}"
        for name, (actual, wanted) in checks.items()
        if not math.isclose(float(actual), float(wanted),
                            rel_tol=0.0, abs_tol=1e-12)
    ]
    if mismatch:
        raise RuntimeError("former/advisory drift: " + "; ".join(mismatch))
    return report


def _closed_loop(row: former.PackingRow, tooth_index: int,
                 lane_mm: float, step_deg: float = 1.0) -> np.ndarray:
    """One closed deposited loop with front/rear retained crowns."""

    front = former.wire_cap_points(
        row, +1, lane_mm=lane_mm, step_deg=step_deg)
    rear = former.wire_cap_points(
        row, -1, lane_mm=lane_mm, step_deg=step_deg)[::-1]
    pieces = [front]
    if np.linalg.norm(front[-1] - rear[0]) > 1e-12:
        pieces.append(np.asarray((front[-1], rear[0])))
    pieces.append(rear)
    if np.linalg.norm(rear[-1] - front[0]) > 1e-12:
        pieces.append(np.asarray((rear[-1], front[0])))
    raw: list[np.ndarray] = []
    for piece in pieces:
        for point in piece:
            if not raw or np.linalg.norm(point - raw[-1]) > 1e-12:
                raw.append(np.asarray(point, dtype=float))
    result = np.asarray(raw)
    angle = tooth_index * 2.0 * math.pi / DEFAULT_STATOR.slots
    c, s = math.cos(angle), math.sin(angle)
    rotated = result.copy()
    rotated[:, 0] = c * result[:, 0] - s * result[:, 1]
    rotated[:, 1] = s * result[:, 0] + c * result[:, 1]
    return rotated


def _obstacles(paths: list[np.ndarray], prefix: str
               ) -> tuple[CopperPolyline, ...]:
    return tuple(
        CopperPolyline(
            obstacle_id=f"{prefix}-turn-{index:02d}",
            owner=prefix,
            turn_index=index,
            centerline_local_mm=tuple(tuple(map(float, point))
                                      for point in path),
        )
        for index, path in enumerate(paths)
    )


def _minimum_field(active: list[np.ndarray], obstacles: list[np.ndarray],
                   search_band_mm: float = 0.75) -> dict[str, Any]:
    field = CopperField(_obstacles(obstacles, "obstacle"))
    minimum = float(search_band_mm)
    witness = None
    for turn_index, path in enumerate(active):
        value = field.clearance(path, search_band_mm)
        if value.minimum_centerline_distance_mm < minimum:
            minimum = float(value.minimum_centerline_distance_mm)
            witness = {
                "active_turn_index": turn_index,
                "active_segment_index": value.route_segment_index,
                "obstacle_id": value.obstacle_id,
                "obstacle_segment_index": value.obstacle_segment_index,
            }
    return {
        "minimum_centerline_distance_mm": minimum,
        "witness": witness,
    }


def route_and_neighbor_audit(step_deg: float = 2.0) -> dict[str, Any]:
    """Bind 50 loops, both faces, and both adjacent teeth."""

    rows = former.packing_rows()
    active = [_closed_loop(row, 0, 0.0, step_deg) for row in rows]

    # The exact advisory proves the one-tooth bundle analytically.  Repeat a
    # segment audit here as an independent serialization/topology check.
    same_min = math.inf
    same_witness = None
    for index, path in enumerate(active):
        if not index:
            continue
        value = _minimum_field([path], active[:index])
        if value["minimum_centerline_distance_mm"] < same_min:
            same_min = value["minimum_centerline_distance_mm"]
            same_witness = {
                "active_turn_index": index,
                **(value["witness"] or {}),
            }

    lane_rows = []
    for lane in LANE_SWEEP_MM:
        neighbours = []
        for tooth in (-1, 1):
            neighbours.extend(
                _closed_loop(row, tooth, lane, step_deg) for row in rows
            )
        value = _minimum_field(active, neighbours)
        lane_rows.append({
            "odd_tooth_lane_mm": lane,
            **value,
            "status": (
                "PASS" if value["minimum_centerline_distance_mm"] + 1e-9
                >= WIRE_CLEARANCE_MM else "FAIL"
            ),
        })
    selected = next(
        row for row in lane_rows
        if row["odd_tooth_lane_mm"] == former.NEIGHBOUR_LANE_MM
    )
    best = max(lane_rows,
               key=lambda row: row["minimum_centerline_distance_mm"])
    return {
        "model": (
            "four exact parallel offsets of the analytic L-R-L contact "
            "surface; identical front/rear retained crowns; odd teeth get "
            "a bounded axial lane while even teeth remain at the stack face"
        ),
        "turn_count": len(rows),
        "both_axial_faces": True,
        "same_tooth": {
            "analytic_status": "PASS",
            "analytic_minimum_centerline_mm": WIRE_CLEARANCE_MM,
            "sampled_segment_minimum_mm": same_min,
            "sampled_witness": same_witness,
            "source": "r3_bend_scope_feasibility one-tooth proof",
        },
        "neighbor_lane_sweep": lane_rows,
        "selected_lane": selected,
        "best_tested_lane": best,
        "all_24_neighbor_topology_status": (
            "PASS" if selected["status"] == "PASS" else "FAIL"
        ),
        "scope_limit": (
            "This bounded sweep rules out the modeled straight-riser "
            "two-colour retained cap, not every possible three-dimensional "
            "end-winding basket."
        ),
    }


def slot_fill_rotor_audit(spec_report: dict[str, Any]) -> dict[str, Any]:
    slot = coil_growth.slot_geometry(DEFAULT_STATOR)
    rows = former.packing_rows()
    all_points = []
    for row in rows:
        all_points.append(former.wire_cap_points(row, +1, lane_mm=0.0))
        all_points.append(former.wire_cap_points(row, -1, lane_mm=0.0))
    points = np.vstack(all_points)
    maximum_wire_outer_radius = float(np.max(np.linalg.norm(
        points[:, :2], axis=1))) + former.WIRE_RADIUS_MM
    wire_axial = float(np.max(np.abs(points[:, 2]))) + former.WIRE_RADIUS_MM
    selected_former_axial = float(max(
        abs(value)
        for sign in (-1, 1)
        for value in former.contact_boundary_yz(
            sign, lane_mm=former.NEIGHBOUR_LANE_MM)[:, 1]
    ))
    remaining_throat = (
        float(slot["opening_width_mm"])
        - 2.0 * former.LINER_THICKNESS_MM
    )
    radial_mouth_left_open = (
        float(slot["shoe_inner_radius_mm"])
        - former.RADIAL_SURFACE_MAX_MM
    )
    return {
        "slot_opening": {
            "bare_throat_width_mm": float(slot["opening_width_mm"]),
            "existing_liner_each_edge_mm": former.LINER_THICKNESS_MM,
            "remaining_lined_throat_width_mm": remaining_throat,
            "required_wire_width_mm": former.WIRE_DIAMETER_MM,
            "former_stops_below_shoe_mouth_by_mm": radial_mouth_left_open,
            "status": (
                "PASS" if remaining_throat + 1e-9
                >= former.WIRE_DIAMETER_MM
                and radial_mouth_left_open > 0.0 else "FAIL"
            ),
        },
        "fill_and_50_turn_envelope": {
            "turn_count": len(rows),
            "radial_center_range_mm": [
                min(row.radial_mm for row in rows),
                max(row.radial_mm for row in rows),
            ],
            "tangential_layers": sorted(set(
                row.tangential_layer for row in rows)),
            "shared_slot_100_center_status": spec_report["checks"][
                "exact shared slot has 100 nonpenetrating side centres"
            ]["ok"],
            "status": "PASS",
        },
        "retained_motor_envelope": {
            "maximum_wire_outer_radius_mm": maximum_wire_outer_radius,
            "stator_outer_radius_mm": DEFAULT_STATOR.od / 2.0,
            "radial_status": (
                "PASS" if maximum_wire_outer_radius <= DEFAULT_STATOR.od / 2
                + 1e-9 else "FAIL"
            ),
            "unstaggered_wire_half_length_mm": wire_axial,
            "selected_former_half_length_mm": selected_former_axial,
            "selected_total_former_length_mm": 2.0 * selected_former_axial,
            "rotor_end_bell_axial_cavity_status": "UNPROVEN",
            "reason": (
                "GOAL/default stator defines OD and stack but no finished "
                "motor rotor/end-bell axial cavity; the selected 12 mm lane "
                "would require the reported retained half-length."
            ),
            "status": "FAIL_UNPROVEN_AXIAL_CAVITY",
        },
    }


def _part_mesh(part: Any, linear: float = 0.15,
               angular: float = 0.12) -> trimesh.Trimesh:
    vertices, faces = part.tessellate(linear, angular)
    mesh = trimesh.Trimesh(
        vertices=np.asarray([(v.X, v.Y, v.Z) for v in vertices]),
        faces=np.asarray(faces), process=True,
    )
    if not mesh.is_watertight:
        raise RuntimeError(
            f"{getattr(part, 'label', 'part')} mesh is not watertight")
    return mesh


def _distance(one_bvh: fcl.BVHModel, one_tf: fcl.Transform,
              two_bvh: fcl.BVHModel, two_tf: fcl.Transform) -> float:
    one = fcl.CollisionObject(one_bvh, one_tf)
    two = fcl.CollisionObject(two_bvh, two_tf)
    collision = fcl.CollisionResult()
    fcl.collide(one, two, fcl.CollisionRequest(), collision)
    if collision.is_collision:
        return -1.0
    return float(fcl.distance(
        one, two, fcl.DistanceRequest(), fcl.DistanceResult()))


def physical_former_overlap_audit() -> dict[str, Any]:
    """Check the selected adjacent front/rear paddle solids directly."""

    rows = []
    for face in (-1, 1):
        even = _part_mesh(former.tooth_paddle(face, lane_mm=0.0))
        odd = _part_mesh(former.tooth_paddle(
            face, lane_mm=former.NEIGHBOUR_LANE_MM))
        even_bvh = collide.make_bvh(even)
        odd_bvh = collide.make_bvh(odd)
        for neighbour in (-1, 1):
            angle = neighbour * 2.0 * math.pi / DEFAULT_STATOR.slots
            rotation = collide.rot_z(angle)
            value = _distance(
                even_bvh, fcl.Transform(),
                odd_bvh, fcl.Transform(rotation, np.zeros(3)),
            )
            rows.append({
                "axial_face": face,
                "neighbor_tooth": neighbour,
                "clearance_mm": value,
                "status": "PASS" if value >= 0.0 else "FAIL",
            })
    minimum = min(row["clearance_mm"] for row in rows)
    return {
        "selected_lane_mm": former.NEIGHBOUR_LANE_MM,
        "rows": rows,
        "minimum_solid_clearance_mm": minimum,
        "status": "PASS" if minimum >= 0.0 else "FAIL",
    }


def _merged_existing_mesh(link: str) -> trimesh.Trimesh:
    manifest = collide.load_manifest()
    meshes = [
        trimesh.load(
            collide.LINKS / "parts" / link / f"{label}.stl", force="mesh"
        )
        for label in manifest["parts"][link]
    ]
    return trimesh.util.concatenate(meshes)


def _candidate_mesh() -> trimesh.Trimesh:
    meshes = []
    for face in (-1, 1):
        meshes.extend(_part_mesh(part) for part in former.former_parts(face))
    return trimesh.util.concatenate(meshes)


def raw_rigid_clearance_audit(max_evaluated_buckets: int = 2500
                              ) -> dict[str, Any]:
    """Bounded raw-pose diagnostic for an already-rejected candidate.

    The full raw timeline is enumerated and symmetry-bucketed, but only a
    deterministic stratified subset is sent through expensive FCL.  This is
    deliberately *not* release authority: the neighbour/retained-motor gates
    already reject the architecture, so an exhaustive machine sweep would
    not change its disposition.
    """

    manifest = collide.load_manifest()
    kin = collide.Kinematics(manifest)
    candidate_mesh = _candidate_mesh()
    meshes = {
        "former": candidate_mesh,
        "flyer": _merged_existing_mesh("flyer"),
    }
    bvhs = {name: collide.make_bvh(mesh) for name, mesh in meshes.items()}
    events = load_events(CAPTURE)
    timeline = Timeline(events)
    pitch2 = 4.0 * math.pi / DEFAULT_STATOR.slots  # parity pattern = 30 deg

    raw_count = 0
    flyer_buckets: dict[
        tuple[int, int, int], tuple[float, float, float, float]
    ] = {}
    for pose in timeline.samples():
        raw_count += 1
        t, m0, m1, m2 = pose
        m0_key = round(m0 / 0.25)
        m1_key = round((m1 % pitch2) / math.radians(1.0))
        flyer_key = (
            m0_key,
            m1_key,
            round((m2 % (2.0 * math.pi)) / math.radians(1.0)),
        )
        flyer_buckets.setdefault(flyer_key, (t, m0, m1, m2))

    local_to_machine = np.asarray((
        (0.0, -1.0, 0.0),
        (0.0, 0.0, 1.0),
        (-1.0, 0.0, 0.0),
    ))
    if max_evaluated_buckets < 1:
        raise ValueError("max_evaluated_buckets must be positive")
    ordered = sorted(flyer_buckets.items())
    if len(ordered) <= max_evaluated_buckets:
        selected = ordered
    else:
        indices = np.linspace(
            0, len(ordered) - 1, max_evaluated_buckets, dtype=int)
        selected = [ordered[int(index)] for index in sorted(set(indices))]
        # Always include every observed deepest-insertion bucket; this is the
        # controlling region in the current production spindle sweep.
        minimum_m0_key = min(key[0] for key, _ in ordered)
        selected_by_key = {key: pose for key, pose in selected}
        selected_by_key.update(
            (key, pose) for key, pose in ordered if key[0] == minimum_m0_key)
        selected = sorted(selected_by_key.items())

    minimum = (math.inf, None)
    for _, (t, m0, m1, m2) in selected:
        dz = float(m0) * kin.mm_per_rad
        axis_z = kin.standoff + dz
        former_rotation = collide.rot_y(m1) @ local_to_machine
        former_tf = fcl.Transform(
            former_rotation, np.array((0.0, 0.0, axis_z)))
        flyer_tf = fcl.Transform(collide.rot_z(m2), np.zeros(3))
        value = _distance(
            bvhs["former"], former_tf, bvhs["flyer"], flyer_tf)
        if value < minimum[0]:
            minimum = (value, {
                "t_s": float(t), "m0_rad": float(m0),
                "m1_rad": float(m1), "m2_rad": float(m2),
            })

    former_radius = float(np.max(np.linalg.norm(
        candidate_mesh.vertices[:, :2], axis=1)))
    flyer_radius = float(np.max(np.linalg.norm(
        meshes["flyer"].vertices[:, :2], axis=1)))
    m0_bound = 0.125 * kin.mm_per_rad
    m1_bound = former_radius * math.radians(0.5)
    m2_bound = flyer_radius * math.radians(0.5)
    motion_bound = m0_bound + m1_bound + m2_bound
    sampled, witness = minimum
    bucket_lower = sampled - motion_bound
    diagnostic_pass = (
        sampled >= 0.0 and bucket_lower + 1e-9 >= RIGID_CLEARANCE_MM)
    return {
        "capture": str(CAPTURE.relative_to(ROOT)).replace("\\", "/"),
        "capture_sha256": _sha256(CAPTURE),
        "raw_timeline_samples": raw_count,
        "occupied_former_flyer_buckets": len(flyer_buckets),
        "evaluated_former_flyer_buckets": len(selected),
        "symmetry_period_deg": 30.0,
        "bucket_steps": {
            "m0_rad": 0.25, "m1_deg": 1.0, "m2_deg": 1.0,
        },
        "diagnostic_rule": (
            "deterministic stratified buckets plus every deepest-insertion "
            "bucket; subtract half-bucket rigid motion only from each tested "
            "bucket. Unevaluated occupied buckets remain explicitly unproved"
        ),
        "former_flyer": {
            "minimum_bucket_sample_mm": float(sampled),
            "quantization_motion_bound_mm": float(motion_bound),
            "tested_bucket_lower_bound_mm": float(bucket_lower),
            "witness": witness,
            "status": "PASS" if diagnostic_pass else "FAIL",
        },
        "former_static_and_carriage": "NOT_RUN_CANDIDATE_ALREADY_REJECTED",
        "exhaustive_raw_authority": False,
        "status": (
            "BOUNDED_PASS_NOT_FULL_RAW_AUTHORITY"
            if diagnostic_pass else "FAIL"
        ),
    }


def manufacturing_contract() -> dict[str, Any]:
    return {
        "part_count": 2,
        "concept": (
            "front and rear one-piece tooth-end baskets; hub rings register "
            "concentrically and tooth-only straps prevent angular slip"
        ),
        "candidate_material": (
            "unfilled PEEK, machined or qualified high-temperature molded; "
            "unfilled PPS is a tooling-cost alternative"
        ),
        "guide_finish": "polish wire-contact surfaces to Ra <= 0.4 um; no flash",
        "retention": (
            "thin high-temperature electrical-grade adhesive film on hub "
            "ring/tooth end faces; no adhesive or former material in slot mouth"
        ),
        "installation": (
            "fit rear cap, insert existing Nomex slot cells, fit front cap, "
            "verify every mouth with a wire-diameter go gauge, then wind"
        ),
        "prototype_process": (
            "machined unfilled PEEK or polished PEEK additive prototype; "
            "FDM layer finish is not accepted as an enamel-contact surface"
        ),
        "qualification": [
            "dielectric withstand and thermal class",
            "adhesive compatibility and retention at speed/temperature",
            "Ra/flash inspection on every guide surface",
            "enamel abrasion coupon at production tension and 300 rpm",
            "actual rotor/end-bell axial cavity measurement",
        ],
        "status": "CONCEPT_ONLY_NOT_RELEASED",
    }


def build_report(run_raw: bool = True) -> dict[str, Any]:
    spec_report = _require_identity()
    routes = route_and_neighbor_audit()
    envelope = slot_fill_rotor_audit(spec_report)
    solids = physical_former_overlap_audit()
    raw = raw_rigid_clearance_audit() if run_raw else {
        "status": "NOT_RUN", "reason": "explicit unit-test fast path",
    }
    gates = {
        "literal_R3_one_tooth_witness": (
            spec_report["status"] == "ADVISORY_COMPATIBLE"),
        "slot_opening_preserved": (
            envelope["slot_opening"]["status"] == "PASS"),
        "50_turn_fill_envelope": (
            envelope["fill_and_50_turn_envelope"]["status"] == "PASS"),
        "all_24_neighbor_wire_clearance": (
            routes["all_24_neighbor_topology_status"] == "PASS"),
        "adjacent_former_solids_nonintersecting": solids["status"] == "PASS",
        "retained_rotor_radial_and_axial_envelope": (
            envelope["retained_motor_envelope"]["status"] == "PASS"),
        "every_raw_pose_rigid_clearance": raw["status"] == "PASS",
        "production_material_finish_coupon": False,
    }
    status = "PASS_REVIEW_CANDIDATE" if all(gates.values()) else "DESIGN_NO_GO"
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "status": status,
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "scope": {
            "job": "DEFAULT_STATOR OD46 x stack15 x 24 slots x 50 turns",
            "architecture": (
                "permanent OD-bounded dielectric L-R-L tooth paddles on both "
                "axial faces with bounded even/odd axial staggering"
            ),
            "production_files_modified": False,
            "raw_capture_or_protocol_modified": False,
            "rejected_86p3mm_global_cap_reused": False,
        },
        "advisory_R3_source": {
            "path": str(SPEC_REPORT.relative_to(ROOT)).replace("\\", "/"),
            "sha256": _sha256(SPEC_REPORT),
            "status": spec_report["status"],
            "decision": spec_report["decision"],
        },
        "wire_routes_and_neighbors": routes,
        "slot_fill_and_motor_envelope": envelope,
        "physical_former_overlap": solids,
        "raw_rigid_clearance": raw,
        "manufacturing_and_installation": manufacturing_contract(),
        "gates": gates,
        "decision": (
            "Do not integrate or order this retained former. The literal "
            "R3, OD-bounded one-tooth 50-loop construction and untouched "
            "slot throat are constructive. The selected localized all-tooth "
            "basket is not: the bounded axial-lane search leaves neighboring "
            "wire paths below one finished-wire diameter, and the required "
            "retained rotor/end-bell axial cavity is not defined or proved. "
            "Raw rigid clearance is reported independently and cannot "
            "override those failed workpiece gates."
        ),
        "source_hashes": {
            "cad/r3_tooth_end_former.py": _sha256(
                CAD / "r3_tooth_end_former.py"),
            "sim/r3_tooth_end_former_study.py": "SELF_AFTER_WRITE",
            "sim/r3_bend_scope_feasibility.py": _sha256(
                HERE / "r3_bend_scope_feasibility.py"),
            "out/reports/r3_bend_scope_feasibility.json": _sha256(
                SPEC_REPORT),
            "out/capture/upstream_current_raw.jsonl": _sha256(CAPTURE),
            "cad/params.py": _sha256(CAD / "params.py"),
        },
    }
    report["source_hashes"]["sim/r3_tooth_end_former_study.py"] = _sha256(
        Path(__file__))
    report["report_sha256"] = _canonical_hash(report)
    return report


def _markdown(report: dict[str, Any]) -> str:
    route = report["wire_routes_and_neighbors"]
    motor = report["slot_fill_and_motor_envelope"]["retained_motor_envelope"]
    raw = report["raw_rigid_clearance"]
    selected = route["selected_lane"]
    return "\n".join((
        "# Retained R3 tooth-end-former study",
        "",
        f"**Status: {report['status']} — isolated review only.**",
        "",
        report["decision"],
        "",
        "## Constructive results",
        "",
        "- Exact four-offset L-R-L crown minimum wire-centre radius: 3.000 mm.",
        "- Exact one-tooth witness: 50 turns; shared slot: 100 centres.",
        f"- Retained radial envelope: {motor['maximum_wire_outer_radius_mm']:.3f} / "
        f"{motor['stator_outer_radius_mm']:.3f} mm.",
        "- The former stops below the steel shoe throat and adds no material "
        "to the existing 0.127 mm liner allowance.",
        "",
        "## Controlling failures",
        "",
        f"- Selected 0/{former.NEIGHBOUR_LANE_MM:g} mm two-colour lane minimum "
        f"neighbor centreline: {selected['minimum_centerline_distance_mm']:.6f} "
        f"mm vs {WIRE_CLEARANCE_MM:.5f} mm required.",
        f"- Best bounded lane sweep result: "
        f"{route['best_tested_lane']['minimum_centerline_distance_mm']:.6f} mm.",
        f"- Required retained former total axial length: "
        f"{motor['selected_total_former_length_mm']:.3f} mm; rotor/end-bell cavity "
        "is unproven.",
        f"- Raw rigid clearance status: {raw['status']}.",
        "",
        "No production assembly or procurement line was changed.",
        "",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ))


def write_reports(report: dict[str, Any]) -> None:
    REPORTS.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8")
    MD_OUT.write_text(_markdown(report), encoding="utf-8")


def main() -> int:
    # The all-tooth neighbour and retained-motor gates reject this candidate
    # before machine integration.  Keep the expensive raw sweep explicitly
    # NOT_RUN rather than turning a no-go review into false release evidence.
    report = build_report(run_raw=False)
    write_reports(report)
    selected = report["wire_routes_and_neighbors"]["selected_lane"]
    print(
        f"{report['status']}: selected neighbor clearance "
        f"{selected['minimum_centerline_distance_mm']:.6f} mm; "
        f"raw {report['raw_rigid_clearance']['status']}"
    )
    return 0 if report["status"].startswith("PASS") else 1


if __name__ == "__main__":
    raise SystemExit(main())
