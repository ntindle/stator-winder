"""Per-occurrence authority for active-guide versus cap collision failures.

This is intentionally a diagnostic, not a release gate.  It decomposes the
aggregate ``fixed`` and ``spindle`` meshes used by
``carriage_active_sector_terminal_guide_audit`` into their physical
occurrences, queries each pair with FCL at a few M1 poses, and confirms every
reference-pose hit with an exact build123d/OpenCascade common-volume check.
No production geometry is modified.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Sequence

import fcl
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
for folder in (CAD, HERE):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

import carriage_active_sector_terminal_guide as guide
import carriage_active_sector_terminal_guide_audit as audit
import collide
import integrated_release_candidate as candidate
from params import PARAMS


OUT = ROOT / "out" / "reports" / "fixed_cap_collision_pairs.json"
ANGLES_DEG = (-1.0, -0.5, 0.0, 0.5, 1.0)


def _label(shape: Any, fallback: str) -> str:
    return str(getattr(shape, "label", fallback)) or fallback


def _safe(label: str) -> str:
    return "".join(
        character if character.isalnum() else "_" for character in label
    )[:80]


def _bbox(shape: Any) -> dict[str, list[float]]:
    box = shape.bounding_box()
    return {
        "minimum_mm": [float(box.min.X), float(box.min.Y), float(box.min.Z)],
        "maximum_mm": [float(box.max.X), float(box.max.Y), float(box.max.Z)],
    }


def _mesh_objects(
    parts: Sequence[Any], prefix: str,
) -> list[dict[str, Any]]:
    rows = []
    for index, shape in enumerate(parts):
        label = _label(shape, f"part_{index:03d}")
        mesh = audit._shape_mesh(
            shape, f"{prefix}_{index:03d}_{_safe(label)}"
        )
        rows.append({
            "index": index,
            "label": label,
            "shape": shape,
            "mesh": mesh,
            "bvh": collide.make_bvh(mesh),
            "bbox": _bbox(shape),
            "mesh_sha256": hashlib.sha256(
                np.round(np.asarray(mesh.vertices), decimals=9).tobytes()
                + np.asarray(mesh.faces, dtype=np.int64).tobytes()
            ).hexdigest(),
        })
    return rows


def _fcl_transform(angle_deg: float) -> tuple[np.ndarray, np.ndarray]:
    angle_rad = math.radians(float(angle_deg))
    rotation = collide.rot_y(angle_rad)
    pivot = np.array([0.0, 0.0, float(PARAMS.m0_home_standoff)])
    return rotation, pivot - rotation @ pivot


def _fcl_query(
    left: fcl.BVHModel,
    right: fcl.BVHModel,
    right_tf: tuple[np.ndarray, np.ndarray],
) -> dict[str, Any]:
    identity = fcl.Transform(np.eye(3), np.zeros(3))
    right_rotation, right_translation = right_tf
    left_object = fcl.CollisionObject(left, identity)
    right_object = fcl.CollisionObject(
        right, fcl.Transform(right_rotation, right_translation)
    )
    request = fcl.CollisionRequest(
        num_max_contacts=500, enable_contact=True
    )
    result = fcl.CollisionResult()
    fcl.collide(left_object, right_object, request, result)
    contacts = list(getattr(result, "contacts", []))
    if result.is_collision:
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
        } for contact in contacts]
        depths = [row["penetration_depth_mm"] for row in contact_rows]
        return {
            "collision": True,
            "contact_count": len(contact_rows),
            "maximum_penetration_depth_mm": max(depths, default=0.0),
            "deepest_contact": max(
                contact_rows,
                key=lambda row: row["penetration_depth_mm"],
                default=None,
            ),
        }
    distance = float(fcl.distance(
        left_object,
        right_object,
        fcl.DistanceRequest(enable_nearest_points=True),
        fcl.DistanceResult(),
    ))
    return {
        "collision": False,
        "distance_mm": distance,
        "contact_count": 0,
        "maximum_penetration_depth_mm": None,
        "deepest_contact": None,
    }


def _brep_pose(shape: Any, angle_deg: float) -> Any:
    # build123d rotations are degrees.  Compose around the same machine-axis
    # pivot as collide.Kinematics.link_tf("spindle", m0=0, m1, m2=0).
    from build123d import Pos, Rot

    pivot_z = float(PARAMS.m0_home_standoff)
    return (
        Pos(0.0, 0.0, pivot_z)
        * Rot(0.0, float(angle_deg), 0.0)
        * Pos(0.0, 0.0, -pivot_z)
        * shape
    )


def _exact_overlap(left: Any, right: Any) -> dict[str, Any]:
    distance = float(left.distance_to(right))
    common = left & right
    volume = float(common.volume)
    solids = list(common.solids())
    return {
        "distance_mm": distance,
        "positive_common_volume_mm3": volume,
        "common_solid_count": len(solids),
        "common_bbox": _bbox(common) if volume > 1.0e-9 else None,
    }


def analyze() -> dict[str, Any]:
    fixed_parts = list(guide.carriage_link_reference_parts())
    cap_parts = list(candidate.cap_module_parts())
    fixed = _mesh_objects(
        fixed_parts, "diagnostic_fixed_active_sector_occurrence"
    )
    caps = _mesh_objects(
        cap_parts, "diagnostic_production_cap_occurrence"
    )

    aggregate_fixed = collide.make_bvh(audit._parts_mesh(
        fixed_parts, "diagnostic_fixed_active_sector_aggregate"
    ))
    aggregate_caps = collide.make_bvh(audit._parts_mesh(
        cap_parts, "diagnostic_production_caps_aggregate"
    ))

    angle_rows = []
    reference_hit_keys: set[tuple[int, int]] = set()
    for angle_deg in ANGLES_DEG:
        transform = _fcl_transform(angle_deg)
        hits = []
        near = []
        for left in fixed:
            for right in caps:
                query = _fcl_query(left["bvh"], right["bvh"], transform)
                row = {
                    "fixed_index": left["index"],
                    "fixed_label": left["label"],
                    "cap_index": right["index"],
                    "cap_label": right["label"],
                    **query,
                }
                if query["collision"]:
                    hits.append(row)
                    if angle_deg == 0.0:
                        reference_hit_keys.add((left["index"], right["index"]))
                elif float(query["distance_mm"]) < 5.0:
                    near.append(row)
        hits.sort(
            key=lambda row: float(row["maximum_penetration_depth_mm"]),
            reverse=True,
        )
        near.sort(key=lambda row: float(row["distance_mm"]))
        angle_rows.append({
            "M1_deg": angle_deg,
            "aggregate": _fcl_query(
                aggregate_fixed, aggregate_caps, transform
            ),
            "per_occurrence_collision_count": len(hits),
            "per_occurrence_hits": hits,
            "per_occurrence_near_under_5mm": near,
        })

    exact_rows = []
    for fixed_index, cap_index in sorted(reference_hit_keys):
        left = fixed[fixed_index]
        right = caps[cap_index]
        exact_rows.append({
            "fixed_index": fixed_index,
            "fixed_label": left["label"],
            "fixed_bbox": left["bbox"],
            "cap_index": cap_index,
            "cap_label": right["label"],
            "cap_bbox": right["bbox"],
            **_exact_overlap(left["shape"], _brep_pose(right["shape"], 0.0)),
        })
    exact_rows.sort(
        key=lambda row: row["positive_common_volume_mm3"], reverse=True
    )

    return {
        "schema": "fixed-cap-collision-pairs/v1",
        "authority": {
            "FCL": (
                "per-occurrence closed collision meshes generated by the "
                "same active-sector audit tessellator"
            ),
            "BREP": (
                "exact OpenCascade distance/common of each reference-pose "
                "FCL hit"
            ),
            "M1_pivot_mm": [
                0.0, 0.0, float(PARAMS.m0_home_standoff)
            ],
        },
        "fixed_occurrences": [{
            key: row[key]
            for key in ("index", "label", "bbox", "mesh_sha256")
        } for row in fixed],
        "cap_occurrences": [{
            key: row[key]
            for key in ("index", "label", "bbox", "mesh_sha256")
        } for row in caps],
        "angle_rows": angle_rows,
        "reference_pose_exact_hits": exact_rows,
        "reference_pose_FCL_hit_count": len(reference_hit_keys),
        "reference_pose_positive_BREP_overlap_count": sum(
            row["positive_common_volume_mm3"] > 1.0e-6
            for row in exact_rows
        ),
    }


if __name__ == "__main__":
    result = analyze()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({
        "out": str(OUT),
        "reference_pose_FCL_hit_count": result[
            "reference_pose_FCL_hit_count"
        ],
        "reference_pose_positive_BREP_overlap_count": result[
            "reference_pose_positive_BREP_overlap_count"
        ],
        "angles": [{
            "M1_deg": row["M1_deg"],
            "aggregate_collision": row["aggregate"]["collision"],
            "per_occurrence_collision_count": row[
                "per_occurrence_collision_count"
            ],
        } for row in result["angle_rows"]],
    }, indent=2))
