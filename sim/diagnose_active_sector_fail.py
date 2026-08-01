"""Per-occurrence diagnostics for the active-sector audit failure witnesses."""

from __future__ import annotations

import json
import math
from pathlib import Path

import fcl
import numpy as np

import carriage_active_sector_terminal_guide_audit as audit
import carriage_active_sector_terminal_guide as guide
import collide
from continuous_conductor_route import _raw_shaft_wraps
import integrated_release_candidate as candidate
from params import DEFAULT_STATOR, PARAMS
from phase_aware_progressive_wire_audit import RawLocus
from traj import Timeline, load_events
import retained_flyer_peek_guide_successor as flyer
import wire_geometry
import wirepath


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "reports" / "carriage_active_sector_failure_pairs.json"
FIRST_RAW_POSE = (
    -61.918, 0.0, -2.3889194135,
)


def _objects(parts, prefix: str):
    rows = []
    for index, (name, shape) in enumerate(parts):
        label = str(getattr(shape, "label", name)) or name
        safe = "".join(
            character if character.isalnum() else "_"
            for character in label
        )[:80]
        mesh = audit._shape_mesh(shape, f"{prefix}_{index:03d}_{safe}")
        rows.append((name, label, collide.make_bvh(mesh)))
    return rows


def _query(left_bvh, right_bvh, left_tf, right_tf):
    left = fcl.CollisionObject(left_bvh, fcl.Transform(*left_tf))
    right = fcl.CollisionObject(right_bvh, fcl.Transform(*right_tf))
    request = fcl.CollisionRequest(num_max_contacts=100, enable_contact=True)
    result = fcl.CollisionResult()
    fcl.collide(left, right, request, result)
    contacts = list(getattr(result, "contacts", []))
    depths = [float(getattr(contact, "penetration_depth", 0.0))
              for contact in contacts]
    contact_rows = [{
        "position_mm": np.asarray(
            getattr(contact, "pos", np.zeros(3)), dtype=float
        ).tolist(),
        "normal": np.asarray(
            getattr(contact, "normal", np.zeros(3)), dtype=float
        ).tolist(),
        "penetration_depth_mm": float(
            getattr(contact, "penetration_depth", 0.0)
        ),
    } for contact in contacts[:20]]
    if result.is_collision:
        return {
            "collision": True,
            "contact_count": len(contacts),
            "maximum_penetration_depth_mm": max(depths, default=None),
            "contacts": contact_rows,
        }
    distance = float(fcl.distance(
        left, right, fcl.DistanceRequest(), fcl.DistanceResult()
    ))
    return {
        "collision": False,
        "distance_mm": distance,
        "contact_count": 0,
        "maximum_penetration_depth_mm": None,
    }


def _wrap_path(wrap, timeline):
    start = float(wrap["start"])
    m0, m1, m2 = map(float, timeline.pose_at(start))
    rotation = wirepath.rot_z(m2)
    bell_throat = rotation @ np.array([
        0.0, flyer.BELL_THROAT_Y_MM,
        float(flyer.base.TIP_GUIDE_CENTER_Z_MM),
    ])
    side = -1 if float(wrap["delta_m1_rad"]) > 0.0 else 1
    contact = wire_geometry.shaft_contact_spec(DEFAULT_STATOR)
    target = wirepath.shaft_tangent_point(
        bell_throat, float(PARAMS.stator_axis_z(m0)), contact, side,
    )
    locus = RawLocus(
        pass_index=-1, phase_index=-1, tooth_index=0,
        motion_sign=side, clockwise_argument=side > 0,
        state_index=0, turn_index=0, half_turn_index=0,
        time_s=start, m0_rad=m0, m1_rad=m1, m2_rad=m2,
        m2_mod_rad=float(m2 % (2.0 * math.pi)),
        radial_x_mm=float(PARAMS.stator_axis_z(m0)),
        m1_alignment_error_rad=0.0,
    )
    path, meta = audit.bell_fairlead_path(
        target, locus, audit.MAX_WIRE_RADIUS_MM,
    )
    return path, meta, (m0, m1, m2)


def analyze():
    kin = collide.Kinematics(collide.load_manifest())
    fixed_parts = [
        (f"fixed_{index:03d}", shape)
        for index, shape in enumerate(guide.carriage_link_reference_parts())
    ]
    flyer_parts = list(candidate.retained_rotating_parts().items())
    fixed = _objects(
        fixed_parts, "actual_fixed_active_sector_yoke_tower_hardware"
    )
    rotating = _objects(
        flyer_parts,
        "final_integrated_L79_stock_D10_P30_PEEK_bell_six_slug_flyer",
    )
    m0, m1, m2 = FIRST_RAW_POSE
    fixed_tf = kin.link_tf("carriage", m0, m1, m2)
    flyer_tf = kin.link_tf("flyer", m0, m1, m2)
    raw_hits = []
    raw_near = []
    for fixed_name, fixed_label, fixed_bvh in fixed:
        for flyer_name, flyer_label, flyer_bvh in rotating:
            result = _query(fixed_bvh, flyer_bvh, fixed_tf, flyer_tf)
            row = {
                "fixed_name": fixed_name,
                "fixed_label": fixed_label,
                "flyer_name": flyer_name,
                "flyer_label": flyer_label,
                **result,
            }
            if result["collision"]:
                raw_hits.append(row)
            elif result["distance_mm"] < 3.0:
                raw_near.append(row)
    raw_near.sort(key=lambda row: row["distance_mm"])

    events = load_events(audit.CAPTURE)
    timeline = Timeline(events)
    wraps = _raw_shaft_wraps(events, timeline)
    wrap_rows = []
    identity = (np.eye(3), np.zeros(3))
    for wrap in wraps:
        path, meta, pose = _wrap_path(wrap, timeline)
        wire_mesh = audit._polyline_capsule_mesh(
            path, audit.MAX_WIRE_RADIUS_MM
        )
        wire_bvh = collide.make_bvh(wire_mesh)
        carriage_tf = kin.link_tf("carriage", *pose)
        hits = []
        near = []
        for fixed_name, fixed_label, fixed_bvh in fixed:
            result = _query(fixed_bvh, wire_bvh, carriage_tf, identity)
            row = {
                "fixed_name": fixed_name,
                "fixed_label": fixed_label,
                **result,
            }
            if result["collision"]:
                hits.append(row)
            elif result["distance_mm"] < 3.0:
                near.append(row)
        near.sort(key=lambda row: row["distance_mm"])
        wrap_rows.append({
            "wrap_number": int(wrap["number"]),
            "pose": {"m0": pose[0], "m1": pose[1], "m2": pose[2]},
            "path_sha256": audit.hashlib.sha256(
                np.round(path, decimals=9).tobytes()
            ).hexdigest(),
            "path_point_count": len(path),
            "meta": meta,
            "hits": hits,
            "near_under_3mm": near,
        })
    return {
        "schema": "carriage-active-sector-failure-pairs/v1",
        "raw_first_pose": {
            "m0": m0, "m1": m1, "m2": m2,
            "carriage_transform": {
                "rotation": np.asarray(fixed_tf[0]).tolist(),
                "translation": np.asarray(fixed_tf[1]).tolist(),
            },
            "flyer_transform": {
                "rotation": np.asarray(flyer_tf[0]).tolist(),
                "translation": np.asarray(flyer_tf[1]).tolist(),
            },
            "hits": raw_hits,
            "near_under_3mm": raw_near,
        },
        "shaft_wraps": wrap_rows,
    }


if __name__ == "__main__":
    result = analyze()
    OUT.write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({
        "raw_hits": result["raw_first_pose"]["hits"],
        "wrap_hits": [row["hits"] for row in result["shaft_wraps"]],
        "out": str(OUT),
    }, indent=2))
