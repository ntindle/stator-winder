"""Fail-closed first-scope placement/collision audit for successor V2.

This audit binds the exact V2 source dependency set and the frozen 4,704-case
placement trade.  It evaluates only two deliberately narrow evidence classes:

* every exact public ``guide_at_case`` placement against the one public
  ``shared_carrier`` with both distance and positive-volume common booleans;
* the exact-active-local neutral assembly for module self, module/carrier, and
  cross-module sibling positive-volume pairs.

The neutral collision policy is strict.  A positive-volume pair remains a
failure unless both labels match one of the exact, same-identity fitted-stack
rules documented in ``INTENDED_FIT_RULES``.  Names such as "bearing", "screw",
or "insert" never create a blanket waiver.

The audit does not edit V2 CAD and does not grant continuous-motion,
tolerance, load, fatigue, wear, wire-route, buildability, production, or
release authority.
"""

from __future__ import annotations

from copy import deepcopy
from functools import lru_cache
import hashlib
import itertools
import json
import math
from pathlib import Path
import re
import sys
import time
from typing import Any, Iterable, Mapping, Sequence


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"

if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))
import aggregate_boundary_follower_successor_v2 as v2


V2_SOURCE_PATH = CAD / "aggregate_boundary_follower_successor_v2.py"
V1_GUIDE_SOURCE_PATH = CAD / (
    "aggregate_boundary_follower_successor_prototype.py"
)
REPLACEMENT_SOURCE_PATH = CAD / (
    "aggregate_boundary_follower_replacement_carriage.py"
)
ACTIVE_CARRIER_SOURCE_PATH = CAD / (
    "carriage_active_sector_terminal_guide.py"
)
HARDWARE_SOURCE_PATH = CAD / "hardware.py"
PLACEMENT_PATH = REPORTS / "aggregate_boundary_follower_placement_trade.json"
REPORT_JSON = REPORTS / (
    "aggregate_boundary_follower_successor_v2_placement_collision_audit.json"
)
REPORT_MD = REPORTS / (
    "aggregate_boundary_follower_successor_v2_placement_collision_audit.md"
)

EXPECTED_INPUT_SHA256 = {
    "successor_v2_source": (
        "bb3edb19be3cd5bdddb056cd7e474c31caa1566d78432ac64b6500578f2335bc"
    ),
    "successor_v1_guide_source": (
        "782456ef56019427d2bdf4fa3be8fa2c4e1684f1dd3be9e6cee7b04422c9677b"
    ),
    "replacement_carrier_source": (
        "c3b9fa201149a44c771aeb218f9120411ee93b93fc459861c876f3f28bb85136"
    ),
    "active_carrier_source": (
        "ef35437481bed682a2104cf5646f68d9bacd0ab3272acc23af2b67b8cc08526c"
    ),
    "hardware_source": (
        "5a20c9862d0df204865fb206b0f6dc7247621d791b896ba3f2805d6723ad8cc6"
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
EXPECTED_IDENTITY_COUNT = 4
EXPECTED_MANUFACTURED_LEAVES_PER_IDENTITY = 56
POSITION_TOLERANCE_MM = 1.0e-9
FRAME_ANGLE_TOLERANCE_DEG = 1.0e-7
POSITIVE_VOLUME_TOLERANCE_MM3 = 1.0e-7

AUTHORITY = {
    "guide_placement_authorized": False,
    "continuous_motion_collision_authorized": False,
    "assembly_integration_authorized": False,
    "wire_route_authorized": False,
    "clearance_authorized": False,
    "tolerance_authorized": False,
    "load_authorized": False,
    "fatigue_authorized": False,
    "wear_authorized": False,
    "dynamics_authorized": False,
    "buildability_authorized": False,
    "procurement_authorized": False,
    "BOM_change_authorized": False,
    "production_authorized": False,
    "release_authorized": False,
}

# These are the only positive-volume exceptions.  Each rule requires anchored
# full-label matches, equal identity, and (where present) equal stack index or
# axis.  The evidence text points to the corresponding authored V2 stack.
INTENDED_FIT_RULES = {
    "pod_heat_set_insert_in_printed_shoe": (
        "folded_flexure_pod documents the printed shoe as a separate receiving "
        "part; pod_attachment_hardware authors four short M3 inserts"
    ),
    "pod_M3_screw_thread_in_matching_insert": (
        "pod_attachment_hardware authors indexed M3x14/washer/short-insert stacks"
    ),
    "preload_heat_set_insert_in_PEEK_shoe": (
        "preload_shoe_local authors a blind insert pilot in the replaceable shoe"
    ),
    "preload_shoe_screw_thread_in_insert": (
        "preload_parts authors one independent shoe screw/washer/insert stack"
    ),
    "preload_leaf_root_screw_thread_in_cradle": (
        "preload_parts authors two independent leaf-root M2x6 stacks and the "
        "cradle has their tapped-size pilots"
    ),
    "preload_adjuster_screw_thread_in_cradle": (
        "preload_parts authors the M2x8 adjuster through the cradle adjuster pilot"
    ),
    "preload_adjuster_screw_thread_in_jam_nut": (
        "preload_parts authors the adjuster and its same-identity M2 jam nut"
    ),
    "gimbal_shoulder_screw_thread_in_matching_nyloc": (
        "_axis_stack_parts authors one same-axis shoulder-screw/nyloc stack"
    ),
    "gimbal_keeper_screw_thread_in_tapped_barrel": (
        "one_sided_barrel_local identifies a tapped 7075 barrel and "
        "_axis_stack_parts authors its indexed keeper screws"
    ),
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _canonical_hash(value: Mapping[str, Any]) -> str:
    body = deepcopy(dict(value))
    body.pop("report_sha256", None)
    return hashlib.sha256(json.dumps(
        body,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")).hexdigest()


def _input_hashes() -> dict[str, str]:
    return {
        "successor_v2_source": _sha256(V2_SOURCE_PATH),
        "successor_v1_guide_source": _sha256(V1_GUIDE_SOURCE_PATH),
        "replacement_carrier_source": _sha256(REPLACEMENT_SOURCE_PATH),
        "active_carrier_source": _sha256(ACTIVE_CARRIER_SOURCE_PATH),
        "hardware_source": _sha256(HARDWARE_SOURCE_PATH),
        "placement_trade": _sha256(PLACEMENT_PATH),
    }


def _require_frozen_inputs() -> dict[str, Any]:
    actual = _input_hashes()
    if actual != EXPECTED_INPUT_SHA256:
        raise ValueError(f"successor V2 audit input drift: {actual}")

    disk_placement = _load(PLACEMENT_PATH)
    public_placement = v2.placement_report()
    for name, placement in (
        ("disk", disk_placement),
        ("V2 public API", public_placement),
    ):
        if (
            placement.get("report_sha256")
            != EXPECTED_PLACEMENT_INTERNAL_SHA256
            or _canonical_hash(placement)
            != EXPECTED_PLACEMENT_INTERNAL_SHA256
        ):
            raise ValueError(f"{name} placement-trade internal hash invalid")
        if len(placement.get("case_comparisons", [])) != EXPECTED_CASE_COUNT:
            raise ValueError(f"{name} placement trade lost 4,704-case coverage")
    if (
        v2.EXPECTED_PLACEMENT_INTERNAL_SHA256
        != EXPECTED_PLACEMENT_INTERNAL_SHA256
    ):
        raise ValueError("V2 public placement binding drift")

    identity_counts = {identity: 0 for identity in range(4)}
    for case in public_placement["case_comparisons"]:
        identity = int(case["identity"]["physical_id"])
        if identity not in identity_counts:
            raise ValueError(f"unexpected physical identity {identity}")
        identity_counts[identity] += 1
    if any(
        count != EXPECTED_CASES_PER_IDENTITY
        for count in identity_counts.values()
    ):
        raise ValueError(f"placement identity coverage drift: {identity_counts}")
    return public_placement


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
        "wire_diameter_mm": float(case["wire_diameter_mm"]),
    }


def _vector_tuple(value: Any) -> tuple[float, float, float]:
    if all(hasattr(value, axis) for axis in ("X", "Y", "Z")):
        return (float(value.X), float(value.Y), float(value.Z))
    return tuple(float(item) for item in value)  # type: ignore[return-value]


def _vector_norm(value: Sequence[float]) -> float:
    return math.sqrt(sum(float(item) * float(item) for item in value))


def _vector_difference_norm(
    one: Sequence[float], two: Sequence[float],
) -> float:
    return _vector_norm(tuple(float(one[i]) - float(two[i]) for i in range(3)))


def _unit(value: Sequence[float]) -> tuple[float, float, float]:
    length = _vector_norm(value)
    if length <= 1.0e-15:
        raise ValueError("degenerate audit direction")
    return tuple(float(item) / length for item in value)  # type: ignore[return-value]


def _angle_between_deg(
    one: Sequence[float], two: Sequence[float],
) -> float:
    a = _unit(one)
    b = _unit(two)
    dot = max(-1.0, min(1.0, sum(a[i] * b[i] for i in range(3))))
    return math.degrees(math.acos(dot))


def _bbox_bounds(shape: Any) -> tuple[float, float, float, float, float, float]:
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
    return (
        min(one[1], two[1]) - max(one[0], two[0]) > POSITION_TOLERANCE_MM
        and min(one[3], two[3]) - max(one[2], two[2])
        > POSITION_TOLERANCE_MM
        and min(one[5], two[5]) - max(one[4], two[4])
        > POSITION_TOLERANCE_MM
    )


def _common_volume(one: Any, two: Any) -> float:
    common = one & two
    return 0.0 if common is None else float(common.volume)


def _finite_or_none(value: float) -> float | None:
    return None if not math.isfinite(float(value)) else float(value)


def _intended_fitted_stack(
    first_label: str, second_label: str,
) -> dict[str, str] | None:
    """Return an exact fitted-stack rule or ``None``; never infer by keyword."""

    orientations = ((first_label, second_label), (second_label, first_label))
    for fitted_label, receiving_label in orientations:
        match = re.fullmatch(
            r"id([0-3])_pod_McMaster_94459A130_insert_([0-3])",
            fitted_label,
        )
        if match and receiving_label == (
            f"id{match.group(1)}_PA12CF_keyed_pod_mount_shoe"
        ):
            rule = "pod_heat_set_insert_in_printed_shoe"
            return {"rule_id": rule, "evidence": INTENDED_FIT_RULES[rule]}

        match = re.fullmatch(
            r"id([0-3])_pod_ISO4762_M3x14_([0-3])", fitted_label,
        )
        other = re.fullmatch(
            r"id([0-3])_pod_McMaster_94459A130_insert_([0-3])",
            receiving_label,
        )
        if match and other and match.groups() == other.groups():
            rule = "pod_M3_screw_thread_in_matching_insert"
            return {"rule_id": rule, "evidence": INTENDED_FIT_RULES[rule]}

        match = re.fullmatch(
            r"id([0-3])_preload_shoe_McMaster_94459A120_insert",
            fitted_label,
        )
        if match and receiving_label == (
            f"id{match.group(1)}_replaceable_polished_PEEK_preload_shoe"
        ):
            rule = "preload_heat_set_insert_in_PEEK_shoe"
            return {"rule_id": rule, "evidence": INTENDED_FIT_RULES[rule]}

        match = re.fullmatch(
            r"id([0-3])_preload_shoe_ISO4762_M2x6", fitted_label,
        )
        if match and receiving_label == (
            f"id{match.group(1)}_preload_shoe_McMaster_94459A120_insert"
        ):
            rule = "preload_shoe_screw_thread_in_insert"
            return {"rule_id": rule, "evidence": INTENDED_FIT_RULES[rule]}

        match = re.fullmatch(
            r"id([0-3])_preload_leaf_ISO4762_M2x6_([01])",
            fitted_label,
        )
        if match and receiving_label == (
            f"id{match.group(1)}_separate_7075_preload_cradle"
        ):
            rule = "preload_leaf_root_screw_thread_in_cradle"
            return {"rule_id": rule, "evidence": INTENDED_FIT_RULES[rule]}

        match = re.fullmatch(
            r"id([0-3])_preload_adjuster_ISO4762_M2x8", fitted_label,
        )
        if match and receiving_label == (
            f"id{match.group(1)}_separate_7075_preload_cradle"
        ):
            rule = "preload_adjuster_screw_thread_in_cradle"
            return {"rule_id": rule, "evidence": INTENDED_FIT_RULES[rule]}
        if match and receiving_label == (
            f"id{match.group(1)}_preload_adjuster_M2_jam_nut"
        ):
            rule = "preload_adjuster_screw_thread_in_jam_nut"
            return {"rule_id": rule, "evidence": INTENDED_FIT_RULES[rule]}

        match = re.fullmatch(
            r"id([0-3])_(yaw|elevation)_McMaster_90265A115_OD3x10_M2",
            fitted_label,
        )
        other = re.fullmatch(
            r"id([0-3])_(yaw|elevation)_ISO10511_M2_nyloc",
            receiving_label,
        )
        if match and other and match.groups() == other.groups():
            rule = "gimbal_shoulder_screw_thread_in_matching_nyloc"
            return {"rule_id": rule, "evidence": INTENDED_FIT_RULES[rule]}

        match = re.fullmatch(
            r"id([0-3])_(yaw|elevation)_keeper_ISO4762_M2x6_([01])",
            fitted_label,
        )
        if match:
            identity, axis, _index = match.groups()
            expected_body = (
                f"id{identity}_keyed_7075_boom_with_yaw_stator"
                if axis == "yaw"
                else (
                    f"id{identity}_yaw_rotor_with_handed_"
                    "elevation_stator_7075"
                )
            )
            if receiving_label == expected_body:
                rule = "gimbal_keeper_screw_thread_in_tapped_barrel"
                return {
                    "rule_id": rule,
                    "evidence": INTENDED_FIT_RULES[rule],
                }
    return None


def _walk_manufactured_leaves(node: Any) -> Iterable[tuple[str, Any]]:
    label = str(getattr(node, "label", "") or "")
    if label.startswith("CONSTRUCTION_ONLY_"):
        return
    children = tuple(getattr(node, "children", ()) or ())
    if children:
        for child in children:
            yield from _walk_manufactured_leaves(child)
        return
    solids = tuple(node.solids())
    if len(solids) != 1:
        raise ValueError(
            f"manufactured leaf {label!r} must contain exactly one solid; "
            f"got {len(solids)}"
        )
    if not label:
        raise ValueError("manufactured leaf has no exact source label")
    if float(node.volume) <= 0.0:
        raise ValueError(f"manufactured leaf {label!r} has no positive volume")
    yield label, node


def _module_manufactured_leaves(identity: int) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for top_level in v2.module_parts(int(identity)):
        for label, shape in _walk_manufactured_leaves(top_level):
            if label in result:
                raise ValueError(
                    f"identity {identity} has duplicate manufactured label {label!r}"
                )
            result[label] = shape
    if len(result) != EXPECTED_MANUFACTURED_LEAVES_PER_IDENTITY:
        raise ValueError(
            f"identity {identity} manufactured leaf count drift: {len(result)}"
        )
    return result


def direct_all_case_guide_to_shared_carrier_BREP(
    placement: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Evaluate public guide frame, distance, and common at all 4,704 cases."""

    if placement is None:
        placement = _require_frozen_inputs()
    cases = placement["case_comparisons"]
    if len(cases) != EXPECTED_CASE_COUNT:
        raise ValueError("V2 direct audit requires exactly 4,704 cases")

    carrier = v2.shared_carrier()
    if len(tuple(carrier.solids())) != 1 or float(carrier.volume) <= 0.0:
        raise ValueError("V2 shared carrier must be one positive-volume solid")

    exact_distance_query_count = 0
    exact_common_boolean_count = 0
    exact_public_frame_count = 0
    zero_positive_count = 0
    positive_count = 0
    kernel_exception_count = 0
    identity_counts = {str(identity): 0 for identity in range(4)}
    minimum_distance = math.inf
    minimum_distance_witness = None
    maximum_common_volume = 0.0
    maximum_common_witness = None
    maximum_origin_error = 0.0
    maximum_origin_error_witness = None
    maximum_tangent_error = 0.0
    maximum_tangent_error_witness = None
    maximum_normal_error = 0.0
    maximum_normal_error_witness = None
    guide_volume_minimum = math.inf
    guide_volume_maximum = 0.0
    exceptions: list[dict[str, Any]] = []
    started = time.perf_counter()

    for case in cases:
        reference = _case_reference(case)
        identity_counts[str(reference["physical_id"])] += 1
        try:
            frame = v2.guide_frame(
                case["required_center_local_mm"],
                case["required_guide_tangent"],
                case["required_curvature_normal_contact_to_center"],
            )
            origin_error = _vector_difference_norm(
                _vector_tuple(frame.origin),
                tuple(float(value) for value in case["required_center_local_mm"]),
            )
            tangent_error = _angle_between_deg(
                _vector_tuple(frame.x_dir),
                case["required_guide_tangent"],
            )
            normal_error = _angle_between_deg(
                _vector_tuple(frame.y_dir),
                case["required_curvature_normal_contact_to_center"],
            )
            exact_public_frame_count += 1

            guide = v2.guide_at_case(case)
            expected_label = (
                f"id{reference['physical_id']}_polished_PEEK_C1_guide"
            )
            if str(guide.label) != expected_label:
                raise ValueError(
                    f"guide_at_case label {guide.label!r} != {expected_label!r}"
                )
            if len(tuple(guide.solids())) != 1 or float(guide.volume) <= 0.0:
                raise ValueError("guide_at_case must return one positive solid")
            guide_volume = float(guide.volume)
            guide_volume_minimum = min(guide_volume_minimum, guide_volume)
            guide_volume_maximum = max(guide_volume_maximum, guide_volume)

            distance = float(guide.distance_to(carrier))
            exact_distance_query_count += 1
            common_volume = _common_volume(guide, carrier)
            exact_common_boolean_count += 1
        except Exception as exc:
            kernel_exception_count += 1
            if len(exceptions) < 20:
                exceptions.append({
                    **reference,
                    "error": f"{type(exc).__name__}: {exc}",
                })
            continue

        if origin_error > maximum_origin_error:
            maximum_origin_error = origin_error
            maximum_origin_error_witness = {
                **reference, "origin_error_mm": origin_error,
            }
        if tangent_error > maximum_tangent_error:
            maximum_tangent_error = tangent_error
            maximum_tangent_error_witness = {
                **reference, "tangent_error_deg": tangent_error,
            }
        if normal_error > maximum_normal_error:
            maximum_normal_error = normal_error
            maximum_normal_error_witness = {
                **reference, "normal_error_deg": normal_error,
            }
        if distance < minimum_distance:
            minimum_distance = distance
            minimum_distance_witness = {
                **reference, "exact_distance_mm": distance,
            }
        if common_volume > POSITIVE_VOLUME_TOLERANCE_MM3:
            positive_count += 1
            if common_volume > maximum_common_volume:
                maximum_common_volume = common_volume
                maximum_common_witness = {
                    **reference,
                    "common_volume_mm3": common_volume,
                    "exact_distance_mm": distance,
                }
        else:
            zero_positive_count += 1

    frame_pass = (
        maximum_origin_error <= POSITION_TOLERANCE_MM
        and maximum_tangent_error <= FRAME_ANGLE_TOLERANCE_DEG
        and maximum_normal_error <= FRAME_ANGLE_TOLERANCE_DEG
    )
    full_execution = (
        exact_public_frame_count == EXPECTED_CASE_COUNT
        and exact_distance_query_count == EXPECTED_CASE_COUNT
        and exact_common_boolean_count == EXPECTED_CASE_COUNT
        and kernel_exception_count == 0
    )
    passed = (
        full_execution
        and frame_pass
        and zero_positive_count == EXPECTED_CASE_COUNT
        and positive_count == 0
    )
    return {
        "case_count": len(cases),
        "cases_per_identity": identity_counts,
        "carrier_label": str(carrier.label),
        "carrier_single_positive_solid": True,
        "exact_public_frame_count": exact_public_frame_count,
        "exact_distance_query_count": exact_distance_query_count,
        "exact_common_boolean_count": exact_common_boolean_count,
        "zero_positive_common_volume_case_count": zero_positive_count,
        "positive_common_volume_case_count": positive_count,
        "kernel_exception_count": kernel_exception_count,
        "minimum_exact_distance_mm": _finite_or_none(minimum_distance),
        "minimum_exact_distance_witness": minimum_distance_witness,
        "maximum_common_volume_mm3": maximum_common_volume,
        "maximum_common_volume_witness": maximum_common_witness,
        "guide_volume_mm3": {
            "minimum": _finite_or_none(guide_volume_minimum),
            "maximum": guide_volume_maximum,
        },
        "public_frame_error": {
            "maximum_origin_error_mm": maximum_origin_error,
            "maximum_origin_error_witness": maximum_origin_error_witness,
            "maximum_tangent_error_deg": maximum_tangent_error,
            "maximum_tangent_error_witness": maximum_tangent_error_witness,
            "maximum_curvature_normal_error_deg": maximum_normal_error,
            "maximum_curvature_normal_error_witness": (
                maximum_normal_error_witness
            ),
            "status": "PASS" if frame_pass else "FAIL",
        },
        "kernel_exceptions": exceptions,
        "elapsed_s": time.perf_counter() - started,
        "status": (
            "PASS_ALL_4704_ZERO_POSITIVE"
            if passed
            else "FAIL_INCOMPLETE_FRAME_OR_POSITIVE_VOLUME"
        ),
        "proof_scope": (
            "exact public frame placement plus exact BREP distance and common "
            "boolean for every one of the 4,704 frozen cases"
        ),
    }


def _new_collision_scope(name: str) -> dict[str, Any]:
    return {
        "name": name,
        "pair_evaluation_count": 0,
        "positive_AABB_candidate_count": 0,
        "exact_common_boolean_count": 0,
        "positive_volume_evaluation_count": 0,
        "allowed_intended_fit_evaluation_count": 0,
        "forbidden_positive_evaluation_count": 0,
        "kernel_exception_count": 0,
        "maximum_common_volume_mm3": 0.0,
        "maximum_common_volume_witness": None,
        "positive_pairs": {},
        "kernel_exceptions": [],
    }


def _audit_pair(
    scope: dict[str, Any],
    first_label: str,
    first_shape: Any,
    first_bounds: tuple[float, float, float, float, float, float],
    second_label: str,
    second_shape: Any,
    second_bounds: tuple[float, float, float, float, float, float],
    witness: Mapping[str, Any],
) -> None:
    scope["pair_evaluation_count"] += 1
    if not _positive_aabb_intersection(first_bounds, second_bounds):
        return
    scope["positive_AABB_candidate_count"] += 1
    try:
        volume = _common_volume(first_shape, second_shape)
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

    scope["positive_volume_evaluation_count"] += 1
    fitted_rule = _intended_fitted_stack(first_label, second_label)
    disposition = "allowed_intended_fit" if fitted_rule else "forbidden"
    if fitted_rule:
        scope["allowed_intended_fit_evaluation_count"] += 1
    else:
        scope["forbidden_positive_evaluation_count"] += 1

    ordered = tuple(sorted((first_label, second_label)))
    pair_key = " || ".join(ordered)
    row = scope["positive_pairs"].setdefault(pair_key, {
        "first": ordered[0],
        "second": ordered[1],
        "disposition": disposition,
        "intended_fit_rule": fitted_rule,
        "positive_evaluation_count": 0,
        "maximum_common_volume_mm3": 0.0,
        "maximum_witness": None,
    })
    if row["disposition"] != disposition:
        raise ValueError(f"pair disposition instability for {pair_key}")
    row["positive_evaluation_count"] += 1
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
            "disposition": disposition,
            "intended_fit_rule": fitted_rule,
        }


def _finalize_collision_scope(scope: dict[str, Any]) -> dict[str, Any]:
    rows = list(scope.pop("positive_pairs").values())
    rows.sort(key=lambda row: row["maximum_common_volume_mm3"], reverse=True)
    allowed = [
        row for row in rows if row["disposition"] == "allowed_intended_fit"
    ]
    forbidden = [row for row in rows if row["disposition"] == "forbidden"]
    scope["unique_positive_pair_count"] = len(rows)
    scope["unique_allowed_intended_fit_pair_count"] = len(allowed)
    scope["unique_forbidden_positive_pair_count"] = len(forbidden)
    scope["positive_pairs"] = rows
    scope["allowed_intended_fit_pairs"] = allowed
    scope["forbidden_positive_pairs"] = forbidden
    scope["status"] = (
        "PASS_ZERO_FORBIDDEN_POSITIVE"
        if not forbidden and scope["kernel_exception_count"] == 0
        else "FAIL_FORBIDDEN_POSITIVE_OR_KERNEL_EXCEPTION"
    )
    return scope


def neutral_exact_active_local_BREP() -> dict[str, Any]:
    """Audit neutral module self/carrier/sibling positive-volume pairs."""

    carrier = v2.shared_carrier()
    carrier_label = str(carrier.label)
    carrier_bounds = _bbox_bounds(carrier)
    modules = {
        identity: _module_manufactured_leaves(identity)
        for identity in range(EXPECTED_IDENTITY_COUNT)
    }
    bounds = {
        identity: {
            label: _bbox_bounds(shape) for label, shape in leaves.items()
        }
        for identity, leaves in modules.items()
    }

    self_scope = _new_collision_scope("neutral_module_self")
    carrier_scope = _new_collision_scope("neutral_module_to_shared_carrier")
    sibling_scope = _new_collision_scope("neutral_cross_identity_sibling")

    for identity, leaves in modules.items():
        items = list(leaves.items())
        witness = {
            "pose": "neutral_exact_active_local",
            "identity": identity,
            "datum_case": _case_reference(v2.datum_case(identity)),
        }
        for (first_label, first_shape), (second_label, second_shape) in (
            itertools.combinations(items, 2)
        ):
            _audit_pair(
                self_scope,
                first_label,
                first_shape,
                bounds[identity][first_label],
                second_label,
                second_shape,
                bounds[identity][second_label],
                witness,
            )
        for label, shape in items:
            _audit_pair(
                carrier_scope,
                label,
                shape,
                bounds[identity][label],
                carrier_label,
                carrier,
                carrier_bounds,
                witness,
            )

    for first_identity, second_identity in itertools.combinations(
        range(EXPECTED_IDENTITY_COUNT), 2,
    ):
        witness = {
            "pose": "neutral_exact_active_local",
            "identities": [first_identity, second_identity],
            "first_datum_case": _case_reference(v2.datum_case(first_identity)),
            "second_datum_case": _case_reference(v2.datum_case(second_identity)),
        }
        for first_label, first_shape in modules[first_identity].items():
            for second_label, second_shape in modules[second_identity].items():
                _audit_pair(
                    sibling_scope,
                    first_label,
                    first_shape,
                    bounds[first_identity][first_label],
                    second_label,
                    second_shape,
                    bounds[second_identity][second_label],
                    witness,
                )

    self_result = _finalize_collision_scope(self_scope)
    carrier_result = _finalize_collision_scope(carrier_scope)
    sibling_result = _finalize_collision_scope(sibling_scope)
    return {
        "pose": "neutral_exact_active_local",
        "placement_law": (
            "V2 module_parts uses each identity's exact source-keyed datum case; "
            "no review-rack translation or rebasing is applied"
        ),
        "identity_count": len(modules),
        "manufactured_leaf_count_per_identity": {
            str(identity): len(leaves) for identity, leaves in modules.items()
        },
        "total_manufactured_leaf_count": sum(map(len, modules.values())),
        "manufactured_labels_per_identity": {
            str(identity): sorted(leaves) for identity, leaves in modules.items()
        },
        "construction_only_witnesses_excluded": True,
        "shared_carrier_label": carrier_label,
        "shared_carrier_single_positive_solid": (
            len(tuple(carrier.solids())) == 1 and float(carrier.volume) > 0.0
        ),
        "datum_cases": {
            str(identity): _case_reference(v2.datum_case(identity))
            for identity in range(EXPECTED_IDENTITY_COUNT)
        },
        "self_collision": self_result,
        "shared_carrier_collision": carrier_result,
        "sibling_collision": sibling_result,
        "fitted_stack_policy": {
            "default_disposition": "forbidden_positive_volume",
            "generic_substring_exemptions": False,
            "same_identity_and_index_or_axis_required": True,
            "exact_rules": dict(INTENDED_FIT_RULES),
        },
        "scope_limit": (
            "neutral exact-active-local pose only; no commanded motion, hard-stop "
            "sweep, deformation, tolerance, or assembly-sequence proof"
        ),
    }


@lru_cache(maxsize=1)
def build_report() -> dict[str, Any]:
    started = time.perf_counter()
    placement = _require_frozen_inputs()
    all_case = direct_all_case_guide_to_shared_carrier_BREP(placement)
    neutral = neutral_exact_active_local_BREP()

    evidence_checks = {
        "frozen_input_hash_chain_valid": (
            _input_hashes() == EXPECTED_INPUT_SHA256
        ),
        "all_4704_public_frames_exact": (
            all_case["exact_public_frame_count"] == EXPECTED_CASE_COUNT
            and all_case["public_frame_error"]["status"] == "PASS"
        ),
        "all_4704_exact_distance_and_common_queries_complete": (
            all_case["exact_distance_query_count"] == EXPECTED_CASE_COUNT
            and all_case["exact_common_boolean_count"] == EXPECTED_CASE_COUNT
            and all_case["kernel_exception_count"] == 0
        ),
        "all_4704_guides_zero_positive_to_shared_carrier": (
            all_case["zero_positive_common_volume_case_count"]
            == EXPECTED_CASE_COUNT
            and all_case["positive_common_volume_case_count"] == 0
        ),
        "neutral_manufactured_leaf_structure_exact": (
            neutral["identity_count"] == EXPECTED_IDENTITY_COUNT
            and neutral["total_manufactured_leaf_count"]
            == (
                EXPECTED_IDENTITY_COUNT
                * EXPECTED_MANUFACTURED_LEAVES_PER_IDENTITY
            )
            and all(
                count == EXPECTED_MANUFACTURED_LEAVES_PER_IDENTITY
                for count in neutral[
                    "manufactured_leaf_count_per_identity"
                ].values()
            )
        ),
        "neutral_self_zero_forbidden_positive_volume": (
            neutral["self_collision"]["status"]
            == "PASS_ZERO_FORBIDDEN_POSITIVE"
        ),
        "neutral_shared_carrier_zero_forbidden_positive_volume": (
            neutral["shared_carrier_collision"]["status"]
            == "PASS_ZERO_FORBIDDEN_POSITIVE"
        ),
        "neutral_siblings_zero_forbidden_positive_volume": (
            neutral["sibling_collision"]["status"]
            == "PASS_ZERO_FORBIDDEN_POSITIVE"
        ),
    }
    blocking_findings = [
        key for key, passed in evidence_checks.items() if passed is not True
    ]
    first_scope_passed = not blocking_findings
    report = {
        "schema": (
            "aggregate-boundary-follower-successor-v2-placement-collision-"
            "audit/v1"
        ),
        "status": "PASS_FIRST_SCOPE" if first_scope_passed else "FAIL_FIRST_SCOPE",
        "decision": (
            "V2_FIRST_SCOPE_ZERO_FORBIDDEN_POSITIVE_VOLUME"
            if first_scope_passed
            else "V2_FIRST_SCOPE_BLOCKED_BY_FRAME_COLLISION_OR_KERNEL_FAILURE"
        ),
        "V2_geometry_schema": v2.SCHEMA,
        "input_hashes": _input_hashes(),
        "placement_internal_sha256": EXPECTED_PLACEMENT_INTERNAL_SHA256,
        "report_paths": {
            "json": str(REPORT_JSON.relative_to(ROOT)).replace("\\", "/"),
            "markdown": str(REPORT_MD.relative_to(ROOT)).replace("\\", "/"),
        },
        "direct_all_4704_guide_to_shared_carrier_BREP": all_case,
        "neutral_exact_active_local_BREP": neutral,
        "evidence_checks": evidence_checks,
        "blocking_findings": blocking_findings,
        "remaining_scope_blockers": [
            "NO_CONTINUOUS_COMMANDED_MOTION_OR_HARD_STOP_SWEEP",
            "NO_TOLERANCE_OR_ASSEMBLY_SEQUENCE_ANALYSIS",
            "NO_LOAD_FLEXURE_FATIGUE_OR_WEAR_ANALYSIS",
            "NO_WIRE_ROUTE_OR_AGGREGATE_PROCESS_CONTACT_ANALYSIS",
            "NO_BUILDABILITY_PROCUREMENT_PRODUCTION_OR_RELEASE_AUTHORITY",
        ],
        "authority": dict(AUTHORITY),
        "snapshot_review": {
            "new_or_visibly_modified_geometry": False,
            "snapshot_skipped": True,
            "reason": (
                "source-only audit; V2 CAD and its existing review STEP were "
                "not modified"
            ),
        },
        "elapsed_s": time.perf_counter() - started,
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def validate_report(report: Mapping[str, Any]) -> None:
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("successor V2 placement/collision report hash invalid")
    if report.get("input_hashes") != EXPECTED_INPUT_SHA256:
        raise ValueError("successor V2 placement/collision input binding drift")
    if any(report.get("authority", {}).values()):
        raise ValueError("successor V2 placement/collision invented authority")

    all_case = report.get("direct_all_4704_guide_to_shared_carrier_BREP", {})
    if all_case.get("case_count") != EXPECTED_CASE_COUNT:
        raise ValueError("successor V2 audit lost 4,704-case coverage")
    neutral = report.get("neutral_exact_active_local_BREP", {})
    if neutral.get("total_manufactured_leaf_count") != (
        EXPECTED_IDENTITY_COUNT * EXPECTED_MANUFACTURED_LEAVES_PER_IDENTITY
    ):
        raise ValueError("successor V2 neutral manufactured leaf count drift")

    checks = report.get("evidence_checks", {})
    expected_status = (
        "PASS_FIRST_SCOPE"
        if checks and all(value is True for value in checks.values())
        else "FAIL_FIRST_SCOPE"
    )
    if report.get("status") != expected_status:
        raise ValueError("successor V2 audit status disagrees with evidence gates")

    known_rules = set(INTENDED_FIT_RULES)
    for scope_name in (
        "self_collision", "shared_carrier_collision", "sibling_collision",
    ):
        scope = neutral.get(scope_name, {})
        for row in scope.get("allowed_intended_fit_pairs", []):
            fitted = row.get("intended_fit_rule") or {}
            if fitted.get("rule_id") not in known_rules:
                raise ValueError(
                    f"{scope_name} contains an unrecognized fitted-stack waiver"
                )


def _markdown(report: Mapping[str, Any]) -> str:
    all_case = report["direct_all_4704_guide_to_shared_carrier_BREP"]
    neutral = report["neutral_exact_active_local_BREP"]
    checks = report["evidence_checks"]
    lines = [
        "# Successor V2 placement/collision audit", "",
        f"Status: **{report['status']}**", "",
        "This is a first-scope audit only. All physical, production, and "
        "release authority remains false.", "",
        "## Exact all-4,704 guide/carrier BREP", "",
        f"- Public frames evaluated: {all_case['exact_public_frame_count']} / 4704.",
        f"- Exact distance queries: {all_case['exact_distance_query_count']} / 4704.",
        f"- Exact common booleans: {all_case['exact_common_boolean_count']} / 4704.",
        f"- Positive guide/carrier cases: {all_case['positive_common_volume_case_count']} / 4704.",
        f"- Kernel exceptions: {all_case['kernel_exception_count']}.",
        f"- Minimum exact distance: {all_case['minimum_exact_distance_mm']} mm.",
        f"- Maximum common volume: {all_case['maximum_common_volume_mm3']} mm^3.",
        f"- Public-frame status: `{all_case['public_frame_error']['status']}`.", "",
        "## Neutral exact-active-local BREP", "",
        f"- Manufactured leaves: {neutral['total_manufactured_leaf_count']} "
        f"({neutral['manufactured_leaf_count_per_identity']}).",
    ]
    for title, key in (
        ("Self", "self_collision"),
        ("Shared carrier", "shared_carrier_collision"),
        ("Sibling", "sibling_collision"),
    ):
        scope = neutral[key]
        lines.append(
            f"- {title}: `{scope['status']}`; "
            f"{scope['unique_forbidden_positive_pair_count']} forbidden and "
            f"{scope['unique_allowed_intended_fit_pair_count']} exact fitted "
            "positive pairs."
        )
    lines.extend(["", "## Gates", ""])
    lines.extend(
        f"- {'PASS' if value else 'FAIL'}: `{key}`"
        for key, value in checks.items()
    )

    forbidden_rows = []
    allowed_rows = []
    for scope_name in (
        "self_collision", "shared_carrier_collision", "sibling_collision",
    ):
        scope = neutral[scope_name]
        forbidden_rows.extend(
            (scope_name, row) for row in scope["forbidden_positive_pairs"]
        )
        allowed_rows.extend(
            (scope_name, row) for row in scope["allowed_intended_fit_pairs"]
        )
    lines.extend(["", "## Forbidden positive pairs", ""])
    if forbidden_rows:
        lines.extend(
            f"- `{scope}`: `{row['first']}` / `{row['second']}` = "
            f"{row['maximum_common_volume_mm3']:.9f} mm^3"
            for scope, row in forbidden_rows
        )
    else:
        lines.append("- None in this first scope.")

    lines.extend(["", "## Exact intended fitted-stack positives", ""])
    if allowed_rows:
        lines.extend(
            f"- `{scope}`: `{row['first']}` / `{row['second']}` under "
            f"`{row['intended_fit_rule']['rule_id']}` = "
            f"{row['maximum_common_volume_mm3']:.9f} mm^3"
            for scope, row in allowed_rows
        )
    else:
        lines.append("- None observed.")

    lines.extend(["", "## Remaining scope blockers", ""])
    lines.extend(f"- `{item}`" for item in report["remaining_scope_blockers"])
    lines.extend([
        "", "## Scope", "",
        "The all-case guide/carrier result is exact for the frozen placement "
        "cases. The module collision result is neutral-only and is not a "
        "continuous-motion, tolerance, load, wire-route, or release proof.",
        "", f"Report SHA-256: `{report['report_sha256']}`", "",
    ])
    return "\n".join(lines)


def write_reports() -> dict[str, Any]:
    report = build_report()
    validate_report(report)
    REPORT_JSON.parent.mkdir(parents=True, exist_ok=True)
    REPORT_JSON.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    REPORT_MD.write_text(_markdown(report), encoding="utf-8")
    return report


if __name__ == "__main__":
    result = write_reports()
    print(f"{result['status']} {result['report_sha256']}")
