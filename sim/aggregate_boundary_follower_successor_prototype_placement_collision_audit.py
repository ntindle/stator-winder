"""Fail-closed placement/collision audit for the isolated successor prototype.

The audit never edits or imports the prototype into the selected assembly.  It
binds the frozen source/STEP/manifest and the 4,704-case placement trade, then
separates three evidence classes:

* exact analytic coverage of every required centre and orientation command;
* direct positive-volume BREP guide/floor checks for every required case; and
* exact BREP endpoint samples of the review mechanism.

Sampled or review-rack results never become continuous-motion, tolerance,
assembly-integration, wire-route, load, production, or release authority.
"""

from __future__ import annotations

from copy import copy, deepcopy
from dataclasses import asdict, dataclass
from functools import lru_cache
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Iterable, Mapping

from build123d import Compound, Pos, Rot, Sphere, import_step


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REVIEW = ROOT / "out" / "review"
REPORTS = ROOT / "out" / "reports"

if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))
import aggregate_boundary_follower_successor_prototype as prototype


SOURCE_PATH = CAD / "aggregate_boundary_follower_successor_prototype.py"
STEP_PATH = REVIEW / "aggregate_boundary_follower_successor_prototype.step"
MANIFEST_PATH = REVIEW / (
    "aggregate_boundary_follower_successor_prototype_manifest.json"
)
PLACEMENT_PATH = REPORTS / "aggregate_boundary_follower_placement_trade.json"
REPORT_JSON = REPORTS / (
    "aggregate_boundary_follower_successor_prototype_placement_collision_audit.json"
)
REPORT_MD = REPORTS / (
    "aggregate_boundary_follower_successor_prototype_placement_collision_audit.md"
)

EXPECTED_INPUT_SHA256 = {
    "prototype_source": (
        "782456ef56019427d2bdf4fa3be8fa2c4e1684f1dd3be9e6cee7b04422c9677b"
    ),
    "prototype_STEP": (
        "6bf20bbca4f166a7c39cee4aec309e8f7765655597a8c8f8e4a335e12a2db183"
    ),
    "prototype_manifest": (
        "0e8ef6bbd0e59a8025d39abd48bf20acc381688ab7b2f63c29540f6c6fc26edb"
    ),
    "placement_trade": (
        "be599cbfed61afdfdaa7fc9c053ee1e20a3ab20cfe723699be1eb5a81e4dbb4c"
    ),
}
EXPECTED_PLACEMENT_INTERNAL_SHA256 = (
    "1800b5f9500f5b0041758991cc8f42f8dc0b62654bec3ce84e402b59dd79dbc3"
)

EXPECTED_CASE_COUNT = 4704
EXPECTED_CASES_PER_IDENTITY = 1176
POSITIVE_VOLUME_TOLERANCE_MM3 = 1.0e-7
POSITION_TOLERANCE_MM = 1.0e-9
ANGLE_TOLERANCE_DEG = 1.0e-9
TANGENT_MATCH_TOLERANCE_DEG = 1.0e-6

AUTHORITY = {
    "guide_placement_authorized": False,
    "gimbal_motion_authorized": False,
    "continuous_motion_collision_authorized": False,
    "assembly_integration_authorized": False,
    "wire_route_authorized": False,
    "clearance_authorized": False,
    "tolerance_authorized": False,
    "load_authorized": False,
    "dynamics_authorized": False,
    "buildability_authorized": False,
    "procurement_authorized": False,
    "BOM_change_authorized": False,
    "production_authorized": False,
    "release_authorized": False,
}


@dataclass(frozen=True)
class EndpointPose:
    name: str
    x_mm: float
    y_mm: float
    z_mm: float
    yaw_deg: float
    elevation_deg: float


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _canonical_hash(value: Mapping[str, Any]) -> str:
    body = deepcopy(dict(value))
    body.pop("report_sha256", None)
    return hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def _input_hashes() -> dict[str, str]:
    return {
        "prototype_source": _sha256(SOURCE_PATH),
        "prototype_STEP": _sha256(STEP_PATH),
        "prototype_manifest": _sha256(MANIFEST_PATH),
        "placement_trade": _sha256(PLACEMENT_PATH),
    }


def _require_frozen_inputs() -> tuple[dict[str, Any], dict[str, Any]]:
    actual = _input_hashes()
    if actual != EXPECTED_INPUT_SHA256:
        raise ValueError(
            f"successor prototype audit input drift: {actual}"
        )
    manifest = _load(MANIFEST_PATH)
    placement = _load(PLACEMENT_PATH)
    if (
        placement.get("report_sha256")
        != EXPECTED_PLACEMENT_INTERNAL_SHA256
        or _canonical_hash(placement)
        != EXPECTED_PLACEMENT_INTERNAL_SHA256
    ):
        raise ValueError("placement-trade internal hash invalid")
    artifacts = manifest.get("artifacts", {})
    if (
        artifacts.get("source_sha256") != actual["prototype_source"]
        or artifacts.get("step_sha256") != actual["prototype_STEP"]
        or artifacts.get("step_exists") is not True
    ):
        raise ValueError("prototype manifest artifact binding invalid")
    return manifest, placement


def _wrap_delta_deg(value: float) -> float:
    return (float(value) + 180.0) % 360.0 - 180.0


def _vector_from_yaw_elevation(
    yaw_deg: float, elevation_deg: float,
) -> tuple[float, float, float]:
    yaw = math.radians(float(yaw_deg))
    elevation = math.radians(float(elevation_deg))
    return (
        math.cos(elevation) * math.cos(yaw),
        math.cos(elevation) * math.sin(yaw),
        math.sin(elevation),
    )


def _prototype_rotated_local_x(
    yaw_deg: float, elevation_deg: float,
) -> tuple[float, float, float]:
    """Actual +X direction from the prototype's Rot(0,-elevation,yaw).

    build123d/OpenCascade applies the supplied Euler rotations in Z then Y
    order here, so this is deliberately not the conventional yaw/elevation
    vector except at special angles.
    """

    yaw = math.radians(float(yaw_deg))
    elevation = math.radians(float(elevation_deg))
    raw = (
        math.cos(elevation) * math.cos(yaw),
        math.sin(yaw),
        math.sin(elevation) * math.cos(yaw),
    )
    norm = math.sqrt(sum(value * value for value in raw))
    return tuple(value / norm for value in raw)


def _angle_between_deg(
    one: Iterable[float], two: Iterable[float],
) -> float:
    a = tuple(float(value) for value in one)
    b = tuple(float(value) for value in two)
    na = math.sqrt(sum(value * value for value in a))
    nb = math.sqrt(sum(value * value for value in b))
    dot = sum(a[i] * b[i] for i in range(3)) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def _case_reference(case: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "locus_index": int(case["locus_index"]),
        "pass_index": int(case["pass_index"]),
        "state_index": int(case["state_index"]),
        "turn_index": int(case["turn_index"]),
        "half_turn_index": int(case["half_turn_index"]),
        "tooth_index": int(case["tooth_index"]),
        "time_s": float(case["time_s"]),
        "physical_id": int(case["identity"]["physical_id"]),
        "lane_id": str(case["lane_id"]),
    }


def analytic_case_coverage(
    placement: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Evaluate every required centre/orientation without BREP sampling."""

    if placement is None:
        _manifest, placement = _require_frozen_inputs()
    trade = placement["successor_trade"]
    cases = placement["case_comparisons"]
    if len(cases) != EXPECTED_CASE_COUNT:
        raise ValueError("placement trade no longer contains 4,704 cases")

    modeled_travel = tuple(float(value) for value in
                           prototype.MODELED_XYZ_TRAVEL_MM)
    half_travel = tuple(value / 2.0 for value in modeled_travel)
    counts = {str(index): 0 for index in range(4)}
    exact_bound_count = 0
    modeled_center_count = 0
    numeric_orientation_count = 0
    realized_tangent_count = 0
    R3_inside_R5_count = 0
    full_two_mm_count = 0
    tangent_error_sum = 0.0
    min_tangent_error = math.inf
    max_tangent_error = -math.inf
    min_tangent_witness = None
    max_tangent_witness = None
    min_R3_relief_margin = math.inf
    min_R3_relief_witness = None
    max_center_offset_norm = -math.inf
    max_center_offset_witness = None
    max_abs_yaw_delta = 0.0
    max_abs_elevation_delta = 0.0

    for case in cases:
        identity = str(int(case["identity"]["physical_id"]))
        counts[identity] += 1
        identity_row = trade["per_identity"][identity]
        center = tuple(float(value) for value in
                       case["required_center_local_mm"])
        datum = tuple(float(value) for value in
                      identity_row["exact_target_datum_local_mm"])
        offset = tuple(center[i] - datum[i] for i in range(3))
        bounds = identity_row["exact_target_center_bounds_local_mm"]
        inside_exact = all(
            float(bounds["min_mm"][i]) - POSITION_TOLERANCE_MM
            <= center[i]
            <= float(bounds["max_mm"][i]) + POSITION_TOLERANCE_MM
            for i in range(3)
        )
        inside_modeled = all(
            abs(offset[i])
            <= half_travel[i] + POSITION_TOLERANCE_MM
            for i in range(3)
        )
        exact_bound_count += int(inside_exact)
        modeled_center_count += int(inside_modeled)

        angles = case["required_guide_tangent_angles"]
        datum_angles = identity_row["polished_guide_tangent_orientation"]
        yaw_delta = _wrap_delta_deg(
            float(angles["yaw_about_positive_Z_deg"])
            - float(datum_angles["yaw"]["datum_deg"])
        )
        elevation_delta = (
            float(angles["elevation_from_XY_deg"])
            - float(datum_angles["elevation"]["datum_deg"])
        )
        max_abs_yaw_delta = max(max_abs_yaw_delta, abs(yaw_delta))
        max_abs_elevation_delta = max(
            max_abs_elevation_delta, abs(elevation_delta))
        numeric_ok = (
            abs(yaw_delta)
            <= prototype.MODELED_YAW_HALF_RANGE_DEG + ANGLE_TOLERANCE_DEG
            and abs(elevation_delta)
            <= prototype.MODELED_ELEVATION_HALF_RANGE_DEG
            + ANGLE_TOLERANCE_DEG
        )
        numeric_orientation_count += int(numeric_ok)

        target_tangent = tuple(float(value) for value in
                               case["required_guide_tangent"])
        angle_vector = _vector_from_yaw_elevation(
            float(angles["yaw_about_positive_Z_deg"]),
            float(angles["elevation_from_XY_deg"]),
        )
        if _angle_between_deg(angle_vector, target_tangent) > 1.0e-6:
            raise ValueError("placement tangent angles/vector disagree")
        realized = _prototype_rotated_local_x(
            float(angles["yaw_about_positive_Z_deg"]),
            float(angles["elevation_from_XY_deg"]),
        )
        tangent_error = _angle_between_deg(realized, target_tangent)
        realized_tangent_count += int(
            tangent_error <= TANGENT_MATCH_TOLERANCE_DEG)
        tangent_error_sum += tangent_error
        if tangent_error < min_tangent_error:
            min_tangent_error = tangent_error
            min_tangent_witness = {
                **_case_reference(case),
                "error_deg": tangent_error,
                "requested_tangent": list(target_tangent),
                "prototype_realized_local_X": list(realized),
            }
        if tangent_error > max_tangent_error:
            max_tangent_error = tangent_error
            max_tangent_witness = {
                **_case_reference(case),
                "error_deg": tangent_error,
                "requested_tangent": list(target_tangent),
                "prototype_realized_local_X": list(realized),
            }

        offset_norm = math.sqrt(sum(value * value for value in offset))
        R3_margin = (
            prototype.FLOOR_RELIEF_RADIUS_MM
            - prototype.CONSERVATIVE_ENVELOPE_RADIUS_MM
            - offset_norm
        )
        R3_inside_R5_count += int(R3_margin >= -POSITION_TOLERANCE_MM)
        full_two_mm_count += int(
            R3_margin
            >= prototype.FLOOR_TARGET_CLEARANCE_MM - POSITION_TOLERANCE_MM
        )
        if R3_margin < min_R3_relief_margin:
            min_R3_relief_margin = R3_margin
            min_R3_relief_witness = {
                **_case_reference(case),
                "center_offset_from_datum_XYZ_mm": list(offset),
                "center_offset_norm_mm": offset_norm,
                "R3_to_R5_remaining_radial_margin_mm": R3_margin,
            }
        if offset_norm > max_center_offset_norm:
            max_center_offset_norm = offset_norm
            max_center_offset_witness = {
                **_case_reference(case),
                "center_offset_from_datum_XYZ_mm": list(offset),
                "center_offset_norm_mm": offset_norm,
            }

    return {
        "case_count": len(cases),
        "cases_per_identity": counts,
        "exact_identity_center_bounds_covered_case_count": exact_bound_count,
        "modeled_1p50x2p40x1p10_center_travel_covered_case_count": (
            modeled_center_count
        ),
        "numeric_yaw_elevation_range_covered_case_count": (
            numeric_orientation_count
        ),
        "max_abs_yaw_delta_from_identity_datum_deg": max_abs_yaw_delta,
        "max_abs_elevation_delta_from_identity_datum_deg": (
            max_abs_elevation_delta
        ),
        "prototype_Rot_realized_tangent_match_case_count": (
            realized_tangent_count
        ),
        "prototype_Rot_tangent_error_deg": {
            "minimum": min_tangent_error,
            "maximum": max_tangent_error,
            "mean": tangent_error_sum / len(cases),
            "minimum_witness": min_tangent_witness,
            "maximum_witness": max_tangent_witness,
            "cause": (
                "Rot(0,-elevation,yaw) applies the Euler frame in an order "
                "that does not map local +X to the requested yaw/elevation "
                "tangent"
            ),
        },
        "conservative_R3_envelope_inside_fixed_R5_relief_case_count": (
            R3_inside_R5_count
        ),
        "full_2mm_R3_to_fixed_R5_relief_margin_case_count": (
            full_two_mm_count
        ),
        "minimum_R3_to_R5_remaining_radial_margin_mm": (
            min_R3_relief_margin
        ),
        "minimum_R3_relief_witness": min_R3_relief_witness,
        "maximum_center_offset_norm_mm": max_center_offset_norm,
        "maximum_center_offset_witness": max_center_offset_witness,
        "proof_scope": {
            "center_and_numeric_range": "exact_all_4704",
            "R3_inside_R5": (
                "exact_spherical_set_containment_for_all_4704"
            ),
            "realized_tangent": (
                "exact_evaluation_of_the_prototype_Euler_transform_for_all_4704"
            ),
        },
    }


def endpoint_poses() -> tuple[EndpointPose, ...]:
    hx, hy, hz = (
        float(value) / 2.0 for value in prototype.MODELED_XYZ_TRAVEL_MM
    )
    yaw = float(prototype.MODELED_YAW_HALF_RANGE_DEG)
    elevation = float(prototype.MODELED_ELEVATION_HALF_RANGE_DEG)
    poses = [EndpointPose("neutral", 0.0, 0.0, 0.0, 0.0, 0.0)]
    axes = (
        ("X", hx, 0), ("Y", hy, 1), ("Z", hz, 2),
        ("yaw", yaw, 3), ("elevation", elevation, 4),
    )
    for label, magnitude, axis in axes:
        for sign, suffix in ((-1.0, "minus"), (1.0, "plus")):
            values = [0.0] * 5
            values[axis] = sign * magnitude
            poses.append(EndpointPose(
                f"axis_{label}_{suffix}", *values,
            ))
    for signs in itertools.product((-1.0, 1.0), repeat=5):
        name = "corner_" + "_".join(
            "m" if sign < 0.0 else "p" for sign in signs
        )
        poses.append(EndpointPose(
            name,
            signs[0] * hx,
            signs[1] * hy,
            signs[2] * hz,
            signs[3] * yaw,
            signs[4] * elevation,
        ))
    if len(poses) != 43 or len({pose.name for pose in poses}) != 43:
        raise RuntimeError("endpoint pose set must contain 43 unique poses")
    return tuple(poses)


def _translated(shape, dx: float, dy: float, dz: float):
    return Pos(float(dx), float(dy), float(dz)) * copy(shape)


def _about_target_yaw(
    shape, original_target: tuple[float, float, float],
    moved_target: tuple[float, float, float], yaw_deg: float,
):
    return Pos(*moved_target) * (
        Rot(0.0, 0.0, float(yaw_deg)) * (
            Pos(*(-value for value in original_target)) * copy(shape)
        )
    )


def _guide_at(
    identity: int,
    center: tuple[float, float, float],
    yaw_deg: float,
    elevation_deg: float,
):
    return Pos(*center) * (
        Rot(0.0, -float(elevation_deg), float(yaw_deg))
        * copy(prototype.c1_guide_local())
    )


def _datum_guide_angles(identity: int) -> tuple[float, float]:
    orientation = prototype.identity_contract(identity)[
        "polished_guide_tangent_orientation"
    ]
    return (
        float(orientation["yaw"]["datum_deg"]),
        float(orientation["elevation"]["datum_deg"]),
    )


def posed_module_shapes(
    identity: int, pose: EndpointPose,
) -> tuple[Any, dict[str, Any]]:
    """Recompose the existing neutral BREP without changing CAD source."""

    stage = prototype.xyz_stage_parts(identity)
    if len(stage) != 11:
        raise RuntimeError("prototype XYZ stage leaf structure drift")
    original_target = prototype._target(identity)
    moved_target = (
        original_target[0] + pose.x_mm,
        original_target[1] + pose.y_mm,
        original_target[2] + pose.z_mm,
    )
    shapes: dict[str, Any] = {}
    for index, part in enumerate(stage):
        dx = pose.x_mm if index >= 3 else 0.0
        dy = pose.y_mm if index >= 6 else 0.0
        dz = pose.z_mm if index >= 9 else 0.0
        shapes[str(part.label)] = _translated(part, dx, dy, dz)

    gimbal = prototype.gimbal_parts(identity)
    if len(gimbal) != 4:
        raise RuntimeError("prototype gimbal leaf structure drift")
    shapes[str(gimbal[0].label)] = _translated(
        gimbal[0], pose.x_mm, pose.y_mm, pose.z_mm,
    )
    for part in gimbal[1:]:
        shapes[str(part.label)] = _about_target_yaw(
            part, original_target, moved_target, pose.yaw_deg,
        )

    datum_yaw, datum_elevation = _datum_guide_angles(identity)
    guide = _guide_at(
        identity,
        moved_target,
        datum_yaw + pose.yaw_deg,
        datum_elevation + pose.elevation_deg,
    )
    guide.label = f"id{identity}_polished_PEEK_C1_guide_cartridge"
    shapes[str(guide.label)] = guide

    for part in prototype.preload_parts(identity):
        shapes[str(part.label)] = _translated(
            part, pose.x_mm, pose.y_mm, pose.z_mm,
        )
    floor = prototype.floor_relief_coupon(identity)
    return floor, shapes


def _bbox_bounds(shape) -> tuple[float, float, float, float, float, float]:
    box = shape.bounding_box()
    return (
        float(box.min.X), float(box.max.X),
        float(box.min.Y), float(box.max.Y),
        float(box.min.Z), float(box.max.Z),
    )


def _positive_aabb_intersection(
    one: tuple[float, float, float, float, float, float],
    two: tuple[float, float, float, float, float, float],
) -> bool:
    tolerance = POSITION_TOLERANCE_MM
    return (
        min(one[1], two[1]) - max(one[0], two[0]) > tolerance
        and min(one[3], two[3]) - max(one[2], two[2]) > tolerance
        and min(one[5], two[5]) - max(one[4], two[4]) > tolerance
    )


def _common_volume(one, two) -> float:
    common = one & two
    return 0.0 if common is None else float(common.volume)


def _new_collision_scope() -> dict[str, Any]:
    return {
        "pair_evaluation_count": 0,
        "positive_AABB_candidate_count": 0,
        "exact_common_boolean_count": 0,
        "positive_collision_evaluation_count": 0,
        "kernel_exception_count": 0,
        "unique_positive_pair_count": 0,
        "maximum_common_volume_mm3": 0.0,
        "maximum_common_volume_witness": None,
        "positive_pairs": {},
        "kernel_exceptions": [],
    }


def _audit_pair(
    scope: dict[str, Any], first_label: str, first,
    second_label: str, second, witness: Mapping[str, Any],
) -> None:
    scope["pair_evaluation_count"] += 1
    try:
        first_bounds = _bbox_bounds(first)
        second_bounds = _bbox_bounds(second)
        if not _positive_aabb_intersection(first_bounds, second_bounds):
            return
        scope["positive_AABB_candidate_count"] += 1
        volume = _common_volume(first, second)
        scope["exact_common_boolean_count"] += 1
    except Exception as exc:
        scope["kernel_exception_count"] += 1
        if len(scope["kernel_exceptions"]) < 20:
            scope["kernel_exceptions"].append({
                **dict(witness),
                "first": first_label,
                "second": second_label,
                "error": f"{type(exc).__name__}: {exc}",
            })
        return
    if volume <= POSITIVE_VOLUME_TOLERANCE_MM3:
        return
    scope["positive_collision_evaluation_count"] += 1
    pair_key = " || ".join(sorted((first_label, second_label)))
    row = scope["positive_pairs"].setdefault(pair_key, {
        "first": first_label,
        "second": second_label,
        "positive_pose_count": 0,
        "maximum_common_volume_mm3": 0.0,
        "maximum_witness": None,
    })
    row["positive_pose_count"] += 1
    if volume > row["maximum_common_volume_mm3"]:
        row["maximum_common_volume_mm3"] = volume
        row["maximum_witness"] = dict(witness)
    if volume > scope["maximum_common_volume_mm3"]:
        scope["maximum_common_volume_mm3"] = volume
        scope["maximum_common_volume_witness"] = {
            **dict(witness),
            "first": first_label,
            "second": second_label,
            "common_volume_mm3": volume,
        }


def _finalize_collision_scope(scope: dict[str, Any]) -> dict[str, Any]:
    pairs = list(scope.pop("positive_pairs").values())
    pairs.sort(key=lambda row: row["maximum_common_volume_mm3"], reverse=True)
    scope["unique_positive_pair_count"] = len(pairs)
    scope["positive_pairs"] = pairs
    scope["status"] = (
        "PASS_ZERO_POSITIVE"
        if not pairs and scope["kernel_exception_count"] == 0
        else "FAIL_POSITIVE_OR_KERNEL_EXCEPTION"
    )
    return scope


def sampled_endpoint_BREP_audit() -> dict[str, Any]:
    poses = endpoint_poses()
    self_scope = _new_collision_scope()
    floor_leaf_scope = _new_collision_scope()
    R3_floor_scope = _new_collision_scope()
    review_sibling_scope = _new_collision_scope()
    active_sibling_scope = _new_collision_scope()

    for pose in poses:
        modules: dict[int, tuple[Any, dict[str, Any]]] = {
            identity: posed_module_shapes(identity, pose)
            for identity in range(4)
        }
        review_shape_sets: dict[int, dict[str, Any]] = {}
        active_shape_sets: dict[int, dict[str, Any]] = {}
        for identity, (floor, shapes) in modules.items():
            items = list(shapes.items())
            bounds = {label: _bbox_bounds(shape) for label, shape in items}
            for index, (first_label, first) in enumerate(items):
                for second_label, second in items[index + 1:]:
                    self_scope["pair_evaluation_count"] += 1
                    if not _positive_aabb_intersection(
                        bounds[first_label], bounds[second_label],
                    ):
                        continue
                    self_scope["pair_evaluation_count"] -= 1
                    _audit_pair(
                        self_scope, first_label, first, second_label, second,
                        {"identity": identity, "pose": pose.name},
                    )

            for label, shape in items:
                _audit_pair(
                    floor_leaf_scope, label, shape, str(floor.label), floor,
                    {"identity": identity, "pose": pose.name},
                )
            center = prototype._target(identity)
            moved_center = (
                center[0] + pose.x_mm,
                center[1] + pose.y_mm,
                center[2] + pose.z_mm,
            )
            R3 = Pos(*moved_center) * Sphere(
                prototype.CONSERVATIVE_ENVELOPE_RADIUS_MM
            )
            R3.label = f"id{identity}_moving_conservative_R3_envelope"
            _audit_pair(
                R3_floor_scope, str(R3.label), R3, str(floor.label), floor,
                {"identity": identity, "pose": pose.name},
            )

            review_shape_sets[identity] = shapes
            datum = tuple(float(value) for value in
                          prototype.identity_contract(identity)[
                              "exact_target_datum_local_mm"
                          ])
            target = prototype._target(identity)
            active_shape_sets[identity] = {
                label: _translated(
                    shape,
                    datum[0] - target[0],
                    datum[1] - target[1],
                    datum[2] - target[2],
                )
                for label, shape in shapes.items()
            }

        for first, second in itertools.combinations(range(4), 2):
            for scope, shape_sets in (
                (review_sibling_scope, review_shape_sets),
                (active_sibling_scope, active_shape_sets),
            ):
                first_items = list(shape_sets[first].items())
                second_items = list(shape_sets[second].items())
                first_bounds = {
                    label: _bbox_bounds(shape)
                    for label, shape in first_items
                }
                second_bounds = {
                    label: _bbox_bounds(shape)
                    for label, shape in second_items
                }
                for first_label, first_shape in first_items:
                    for second_label, second_shape in second_items:
                        scope["pair_evaluation_count"] += 1
                        if not _positive_aabb_intersection(
                            first_bounds[first_label],
                            second_bounds[second_label],
                        ):
                            continue
                        scope["pair_evaluation_count"] -= 1
                        _audit_pair(
                            scope,
                            first_label,
                            first_shape,
                            second_label,
                            second_shape,
                            {"pair": [first, second], "pose": pose.name},
                        )

    return {
        "sampling_contract": {
            "pose_count_per_identity": len(poses),
            "identity_count": 4,
            "total_identity_pose_count": 4 * len(poses),
            "neutral_pose_count": 1,
            "single_axis_endpoint_pose_count": 10,
            "combined_five_DOF_corner_pose_count": 32,
            "poses": [asdict(pose) for pose in poses],
            "motion_semantics": (
                "hierarchical X then Y then Z translation; yaw rotor/fork/pin "
                "about the moved target; guide rebuilt with prototype Euler "
                "law at datum plus yaw/elevation commands; separate preload "
                "translates with XYZ and does not rotate with the guide"
            ),
        },
        "self_collision": _finalize_collision_scope(self_scope),
        "own_floor_leaf_collision": _finalize_collision_scope(
            floor_leaf_scope
        ),
        "conservative_R3_to_own_floor_collision": (
            _finalize_collision_scope(R3_floor_scope)
        ),
        "review_rack_sibling_collision": _finalize_collision_scope(
            review_sibling_scope
        ),
        "exact_active_local_rebased_sibling_collision": (
            _finalize_collision_scope(active_sibling_scope)
        ),
        "scope_limit": (
            "43 endpoint samples are not a continuous motion or tolerance "
            "proof; exact-active-local rebasing exposes integration conflicts "
            "but does not create a shared carrier or valid assembly"
        ),
    }


def direct_all_case_guide_floor_BREP_audit(
    placement: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Directly place the modeled guide law at all 4,704 required cases."""

    if placement is None:
        _manifest, placement = _require_frozen_inputs()
    cases = placement["case_comparisons"]
    floors = {
        identity: prototype.floor_relief_coupon(identity)
        for identity in range(4)
    }
    local_guide = prototype.c1_guide_local()
    local_volume = float(local_guide.volume)
    positive_count = 0
    zero_positive_count = 0
    kernel_exception_count = 0
    exact_distance_query_count = 0
    exact_common_boolean_count = 0
    minimum_exact_distance = math.inf
    minimum_distance_witness = None
    maximum_common_volume = 0.0
    maximum_common_witness = None
    exception_rows = []
    started = time.perf_counter()

    for case in cases:
        identity = int(case["identity"]["physical_id"])
        center = prototype.display_point_from_active_local(
            identity, case["required_center_local_mm"]
        )
        angles = case["required_guide_tangent_angles"]
        guide = _guide_at(
            identity,
            center,
            float(angles["yaw_about_positive_Z_deg"]),
            float(angles["elevation_from_XY_deg"]),
        )
        try:
            distance = float(guide.distance_to(floors[identity]))
            exact_distance_query_count += 1
            if distance < minimum_exact_distance:
                minimum_exact_distance = distance
                minimum_distance_witness = {
                    **_case_reference(case),
                    "exact_distance_mm": distance,
                }
            common_volume = 0.0
            if distance <= POSITION_TOLERANCE_MM:
                common_volume = _common_volume(guide, floors[identity])
                exact_common_boolean_count += 1
        except Exception as exc:
            kernel_exception_count += 1
            if len(exception_rows) < 20:
                exception_rows.append({
                    **_case_reference(case),
                    "error": f"{type(exc).__name__}: {exc}",
                })
            continue
        if common_volume > POSITIVE_VOLUME_TOLERANCE_MM3:
            positive_count += 1
            if common_volume > maximum_common_volume:
                maximum_common_volume = common_volume
                maximum_common_witness = {
                    **_case_reference(case),
                    "common_volume_mm3": common_volume,
                    "prototype_Euler_law_note": (
                        "this is the geometry actually produced by "
                        "Rot(0,-elevation,yaw), not the requested tangent frame"
                    ),
                }
        else:
            zero_positive_count += 1

    return {
        "case_count": len(cases),
        "local_guide_positive_volume_mm3": local_volume,
        "local_guide_single_solid": len(local_guide.solids()) == 1,
        "exact_distance_query_count": exact_distance_query_count,
        "exact_common_boolean_count": exact_common_boolean_count,
        "zero_positive_common_volume_case_count": zero_positive_count,
        "positive_common_volume_case_count": positive_count,
        "kernel_exception_count": kernel_exception_count,
        "minimum_exact_distance_mm": minimum_exact_distance,
        "minimum_exact_distance_witness": minimum_distance_witness,
        "maximum_common_volume_mm3": maximum_common_volume,
        "maximum_common_volume_witness": maximum_common_witness,
        "kernel_exceptions": exception_rows,
        "elapsed_s": time.perf_counter() - started,
        "status": (
            "PASS_ZERO_POSITIVE"
            if positive_count == 0 and kernel_exception_count == 0
            else "FAIL_POSITIVE_OR_KERNEL_EXCEPTION"
        ),
        "scope_limit": (
            "direct BREP uses the prototype's actual Euler placement law; it "
            "does not repair the tangent-frame error and does not include a "
            "wire, tolerance, shared carrier, or continuous gimbal sweep"
        ),
    }


def _artifact_geometry_evidence() -> dict[str, Any]:
    imported = import_step(STEP_PATH)
    source_shape = prototype.gen_step()
    guide = prototype.c1_guide_local()
    guide_box = guide.bounding_box()
    return {
        "STEP_leaf_solid_count": len(imported.solids()),
        "source_leaf_solid_count": len(source_shape.solids()),
        "all_STEP_solids_positive_volume": all(
            float(solid.volume) > 0.0 for solid in imported.solids()
        ),
        "guide_single_solid": len(guide.solids()) == 1,
        "guide_positive_volume_mm3": float(guide.volume),
        "guide_local_bounding_box_mm": {
            "min": [
                float(guide_box.min.X), float(guide_box.min.Y),
                float(guide_box.min.Z),
            ],
            "max": [
                float(guide_box.max.X), float(guide_box.max.Y),
                float(guide_box.max.Z),
            ],
        },
        "C1_construction": {
            "path": "line--R3_quarter_arc--line",
            "bend_radius_mm": prototype.GUIDE_CENTERLINE_BEND_RADIUS_MM,
            "first_join_incoming_tangent": [1.0, 0.0, 0.0],
            "first_join_outgoing_tangent": [1.0, 0.0, 0.0],
            "second_join_incoming_tangent": [0.0, 1.0, 0.0],
            "second_join_outgoing_tangent": [0.0, 1.0, 0.0],
            "C1_tangent_continuity_by_source_construction": True,
        },
    }


@lru_cache(maxsize=1)
def build_report() -> dict[str, Any]:
    started = time.perf_counter()
    manifest, placement = _require_frozen_inputs()
    artifact = _artifact_geometry_evidence()
    analytic = analytic_case_coverage(placement)
    direct = direct_all_case_guide_floor_BREP_audit(placement)
    sampled = sampled_endpoint_BREP_audit()

    evidence_checks = {
        "frozen_input_hash_chain_valid": _input_hashes()
        == EXPECTED_INPUT_SHA256,
        "STEP_and_source_each_have_84_positive_leaf_solids": (
            artifact["STEP_leaf_solid_count"] == 84
            and artifact["source_leaf_solid_count"] == 84
            and artifact["all_STEP_solids_positive_volume"] is True
        ),
        "all_4704_centers_inside_exact_and_modeled_travel": (
            analytic["exact_identity_center_bounds_covered_case_count"]
            == EXPECTED_CASE_COUNT
            and analytic[
                "modeled_1p50x2p40x1p10_center_travel_covered_case_count"
            ] == EXPECTED_CASE_COUNT
        ),
        "all_4704_numeric_orientation_commands_inside_declared_ranges": (
            analytic["numeric_yaw_elevation_range_covered_case_count"]
            == EXPECTED_CASE_COUNT
        ),
        "all_4704_prototype_Euler_transforms_realize_requested_tangent": (
            analytic["prototype_Rot_realized_tangent_match_case_count"]
            == EXPECTED_CASE_COUNT
        ),
        "all_4704_R3_envelopes_contained_by_fixed_R5_relief": (
            analytic[
                "conservative_R3_envelope_inside_fixed_R5_relief_case_count"
            ] == EXPECTED_CASE_COUNT
        ),
        "all_4704_R3_envelopes_retain_full_2mm_relief_margin": (
            analytic[
                "full_2mm_R3_to_fixed_R5_relief_margin_case_count"
            ] == EXPECTED_CASE_COUNT
        ),
        "all_4704_modeled_guides_zero_positive_to_floor": (
            direct["zero_positive_common_volume_case_count"]
            == EXPECTED_CASE_COUNT
            and direct["kernel_exception_count"] == 0
        ),
        "sampled_endpoint_self_collision_zero": (
            sampled["self_collision"]["status"] == "PASS_ZERO_POSITIVE"
        ),
        "sampled_endpoint_floor_collision_zero": (
            sampled["own_floor_leaf_collision"]["status"]
            == "PASS_ZERO_POSITIVE"
            and sampled["conservative_R3_to_own_floor_collision"]["status"]
            == "PASS_ZERO_POSITIVE"
        ),
        "sampled_review_rack_sibling_collision_zero": (
            sampled["review_rack_sibling_collision"]["status"]
            == "PASS_ZERO_POSITIVE"
        ),
        "sampled_exact_active_local_rebased_sibling_collision_zero": (
            sampled["exact_active_local_rebased_sibling_collision"]["status"]
            == "PASS_ZERO_POSITIVE"
        ),
    }
    blocking_findings = [
        key for key, value in evidence_checks.items() if value is not True
    ]
    report = {
        "schema": (
            "aggregate-boundary-follower-successor-prototype-placement-"
            "collision-audit/v1"
        ),
        "status": (
            "PASS_AUDIT__PROTOTYPE_NOT_PLACEMENT_OR_COLLISION_READY"
        ),
        "decision": (
            "NUMERIC_5DOF_COMMAND_ENVELOPE_COVERS_4704__"
            "POSITIVE_VOLUME_PROTOTYPE_FAILS_REALIZED_PLACEMENT_OR_COLLISION_GATES"
        ),
        "input_hashes": _input_hashes(),
        "manifest_status": manifest["status"],
        "artifact_geometry": artifact,
        "analytic_all_4704_case_coverage": analytic,
        "direct_all_4704_guide_to_floor_BREP": direct,
        "sampled_endpoint_BREP": sampled,
        "evidence_checks": evidence_checks,
        "blocking_findings": blocking_findings,
        "blockers": [
            "PROTOTYPE_EULER_ORDER_DOES_NOT_REALIZE_REQUESTED_GUIDE_TANGENT",
            "FIXED_R5_RELIEF_HAS_2MM_MARGIN_ONLY_AT_DATUM_NOT_OVER_TRAVEL",
            "SAMPLED_ONLY_43_ENDPOINTS_PER_IDENTITY_NOT_CONTINUOUS_SWEEP",
            "STATIC_GIMBAL_HAS_NO_PARAMETRIC_ELEVATION_MEMBER_OR_MODELED_STOPS",
            "REVIEW_RACK_IS_NOT_EXACT_ACTIVE_LOCAL_SHARED_CARRIER_INTEGRATION",
            "NO_POSITIVE_VOLUME_SHARED_CARRIER_FASTENER_OR_FLEXURE_STACK",
            "NO_TOLERANCE_LOAD_WEAR_WIRE_ROUTE_OR_DYNAMICS_MODEL",
        ],
        "authority": dict(AUTHORITY),
        "snapshot_review": {
            "new_or_visibly_modified_geometry": False,
            "snapshot_skipped": True,
            "reason": (
                "inspection-only audit; frozen STEP/source/manifest were not "
                "modified, and the prototype's existing reviewed four-view "
                "snapshot packet remains the visual evidence"
            ),
            "existing_snapshot_prefix": (
                "out/review/snapshots/aggregate_boundary_follower_successor_"
            ),
        },
        "elapsed_s": time.perf_counter() - started,
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def validate_report(report: Mapping[str, Any]) -> None:
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("successor placement/collision report hash invalid")
    if report.get("status") != (
        "PASS_AUDIT__PROTOTYPE_NOT_PLACEMENT_OR_COLLISION_READY"
    ):
        raise ValueError("unexpected successor placement/collision status")
    if report.get("input_hashes") != EXPECTED_INPUT_SHA256:
        raise ValueError("successor placement/collision input binding drift")
    analytic = report.get("analytic_all_4704_case_coverage", {})
    if analytic.get("case_count") != EXPECTED_CASE_COUNT:
        raise ValueError("successor placement/collision lost case coverage")
    if any(report.get("authority", {}).values()):
        raise ValueError("successor placement/collision invented authority")
    if not report.get("blocking_findings"):
        raise ValueError("successor prototype unexpectedly cleared every gate")


def _markdown(report: Mapping[str, Any]) -> str:
    analytic = report["analytic_all_4704_case_coverage"]
    direct = report["direct_all_4704_guide_to_floor_BREP"]
    sampled = report["sampled_endpoint_BREP"]
    checks = report["evidence_checks"]
    lines = [
        "# Successor-prototype placement/collision audit", "",
        f"Status: **{report['status']}**", "",
        "The audit completed successfully, but the isolated prototype is not "
        "placement- or collision-ready. All physical and release authority "
        "remains false.", "",
        "## Exact all-4,704 coverage", "",
        f"- Exact identity bounds covered: {analytic['exact_identity_center_bounds_covered_case_count']} / 4704.",
        f"- Modeled 1.50 x 2.40 x 1.10 mm travel covered: {analytic['modeled_1p50x2p40x1p10_center_travel_covered_case_count']} / 4704.",
        f"- Numeric yaw/elevation range covered: {analytic['numeric_yaw_elevation_range_covered_case_count']} / 4704.",
        f"- Prototype Euler transform realizes the requested tangent: {analytic['prototype_Rot_realized_tangent_match_case_count']} / 4704.",
        f"- Tangent error range: {analytic['prototype_Rot_tangent_error_deg']['minimum']:.9f} to {analytic['prototype_Rot_tangent_error_deg']['maximum']:.9f} degrees.",
        f"- Conservative R3 contained in fixed R5 relief: {analytic['conservative_R3_envelope_inside_fixed_R5_relief_case_count']} / 4704.",
        f"- Full 2.00 mm R3-to-relief margin retained: {analytic['full_2mm_R3_to_fixed_R5_relief_margin_case_count']} / 4704.",
        f"- Worst remaining R3-to-R5 margin: {analytic['minimum_R3_to_R5_remaining_radial_margin_mm']:.9f} mm.", "",
        "## Direct BREP", "",
        f"- Modeled guide/floor positive collisions: {direct['positive_common_volume_case_count']} / 4704.",
        f"- Exact distance queries: {direct['exact_distance_query_count']}; exact common booleans: {direct['exact_common_boolean_count']}; kernel exceptions: {direct['kernel_exception_count']}.",
        f"- Endpoint identity poses: {sampled['sampling_contract']['total_identity_pose_count']} ({sampled['sampling_contract']['pose_count_per_identity']} per identity).",
        f"- Self collision status: `{sampled['self_collision']['status']}`; unique positive pairs {sampled['self_collision']['unique_positive_pair_count']}.",
        f"- Own-floor leaf collision status: `{sampled['own_floor_leaf_collision']['status']}`; unique positive pairs {sampled['own_floor_leaf_collision']['unique_positive_pair_count']}.",
        f"- R3/floor endpoint status: `{sampled['conservative_R3_to_own_floor_collision']['status']}`.",
        f"- Review-rack sibling status: `{sampled['review_rack_sibling_collision']['status']}`.",
        f"- Exact-active-local rebased sibling status: `{sampled['exact_active_local_rebased_sibling_collision']['status']}`; unique positive pairs {sampled['exact_active_local_rebased_sibling_collision']['unique_positive_pair_count']}.", "",
        "## Gates", "",
    ]
    lines.extend(
        f"- {'PASS' if value else 'FAIL'}: `{key}`"
        for key, value in checks.items()
    )
    lines.extend(["", "## Blockers", ""])
    lines.extend(f"- `{item}`" for item in report["blockers"])
    lines.extend([
        "", "## Scope", "",
        "The 43 endpoint poses per identity are exact positive-volume BREP "
        "samples, not a continuous sweep. Exact-active-local rebasing is a "
        "collision witness, not a valid integrated carrier design. No new "
        "snapshot was generated because this audit changed no visible geometry.",
        "", f"Report SHA-256: `{report['report_sha256']}`", "",
    ])
    return "\n".join(lines)


def write_reports() -> dict[str, Any]:
    report = build_report()
    validate_report(report)
    REPORT_JSON.parent.mkdir(parents=True, exist_ok=True)
    REPORT_JSON.write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    REPORT_MD.write_text(_markdown(report), encoding="utf-8")
    return report


if __name__ == "__main__":
    result = write_reports()
    print(f"{result['status']} {result['report_sha256']}")
