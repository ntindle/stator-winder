"""Fail-closed trade for the aggregate-boundary follower placement contract.

This module compares every nonzero analytic C1 rebound end-arc centre against
the exact source-level nose envelope of the current four-occurrence
replacement carriage.  It also records the smallest analytic successor
envelope supported by those points and orientations.

The result is deliberately not a CAD integration.  In particular, a circular
R3 nose centred on the analytic end arc has zero load projection along the
aggregate support normal in every case.  Translation or gimbal travel cannot
repair that geometric incompatibility; a successor must decouple polished C1
wire guidance from aggregate-normal preload.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable, Mapping, Sequence


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
REVIEW = ROOT / "out" / "review"

if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

import aggregate_boundary_follower_c1_rebound_sweep as rebound
import aggregate_boundary_follower_replacement_carriage as carriage


C1_REPORT_PATH = REPORTS / "aggregate_boundary_follower_c1_rebound_sweep.json"
CARRIAGE_MANIFEST_PATH = REVIEW / (
    "aggregate_boundary_follower_replacement_carriage_manifest.json"
)
CARRIAGE_STEP_PATH = REVIEW / (
    "aggregate_boundary_follower_replacement_carriage.step"
)
OUTPUT_JSON = REPORTS / "aggregate_boundary_follower_placement_trade.json"
OUTPUT_MD = REPORTS / "aggregate_boundary_follower_placement_trade.md"

SCHEMA = "aggregate-boundary-follower-placement-trade/v1"
EXPECTED_NONZERO_CASES = 4704
EXPECTED_CASES_PER_IDENTITY = 1176
EXPECTED_CASES_PER_DIAMETER = 2352
EXPECTED_DIAMETER_PAIRS_PER_IDENTITY = 588
WIRE_DIAMETERS_MM = (0.2, 0.5)
NOSE_SURFACE_RADIUS_MM = 3.0
NOMINAL_CARRIER_CLEARANCE_TARGET_MM = 2.0
GEOMETRY_TOLERANCE_MM = 1.0e-9
ANGLE_TOLERANCE_DEG = 1.0e-8

SOURCE_PATHS = (
    Path("sim/aggregate_boundary_follower_placement_trade.py"),
    Path("sim/aggregate_boundary_follower_c1_rebound_sweep.py"),
    Path("cad/aggregate_boundary_follower_replacement_carriage.py"),
    Path("cad/aggregate_boundary_floating_follower.py"),
)

AUTHORITY_KEYS = (
    "wire_route_authorized",
    "collision_authorized",
    "assembly_integration_authorized",
    "load_authorized",
    "dynamics_authorized",
    "BOM_change_authorized",
    "procurement_authorized",
    "production_authorized",
    "release_authorized",
)


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


def _dot(one: Sequence[float], two: Sequence[float]) -> float:
    return sum(float(one[index]) * float(two[index])
               for index in range(len(one)))


def _norm(vector: Sequence[float]) -> float:
    return math.sqrt(_dot(vector, vector))


def _unit(vector: Sequence[float]) -> tuple[float, ...]:
    length = _norm(vector)
    if length <= 1.0e-15:
        raise ValueError("zero-length vector")
    return tuple(float(value) / length for value in vector)


def _sub(one: Sequence[float], two: Sequence[float]) -> tuple[float, ...]:
    return tuple(float(one[index]) - float(two[index])
                 for index in range(len(one)))


def _distance(one: Sequence[float], two: Sequence[float]) -> float:
    return _norm(_sub(one, two))


def _bounds(points: Sequence[Sequence[float]]) -> dict[str, list[float]]:
    if not points:
        raise ValueError("cannot bound an empty point set")
    axes = tuple(zip(*points))
    minima = [min(map(float, axis)) for axis in axes]
    maxima = [max(map(float, axis)) for axis in axes]
    return {
        "min_mm": minima,
        "max_mm": maxima,
        "span_mm": [
            maxima[index] - minima[index]
            for index in range(len(minima))
        ],
    }


def _midpoint(bounds: Mapping[str, Sequence[float]]) -> list[float]:
    return [
        (float(bounds["min_mm"][index])
         + float(bounds["max_mm"][index])) / 2.0
        for index in range(len(bounds["min_mm"]))
    ]


def _bbox(shape: Any) -> dict[str, list[float]]:
    box = shape.bounding_box()
    minima = [float(box.min.X), float(box.min.Y), float(box.min.Z)]
    maxima = [float(box.max.X), float(box.max.Y), float(box.max.Z)]
    return {
        "min_mm": minima,
        "max_mm": maxima,
        "span_mm": [
            maxima[index] - minima[index] for index in range(3)
        ],
    }


def _corners(bounds: Mapping[str, Sequence[float]]) \
        -> tuple[tuple[float, float, float], ...]:
    return tuple(
        (x, y, z)
        for x in (float(bounds["min_mm"][0]),
                  float(bounds["max_mm"][0]))
        for y in (float(bounds["min_mm"][1]),
                  float(bounds["max_mm"][1]))
        for z in (float(bounds["min_mm"][2]),
                  float(bounds["max_mm"][2]))
    )


def _normalize_angle_deg(angle_deg: float) -> float:
    result = (float(angle_deg) + 180.0) % 360.0 - 180.0
    return 180.0 if math.isclose(result, -180.0, abs_tol=1.0e-12) else result


def _signed_angle_delta_deg(angle_deg: float, datum_deg: float) -> float:
    return _normalize_angle_deg(float(angle_deg) - float(datum_deg))


def _minimum_covering_angle_interval(
    angles_deg: Sequence[float],
) -> dict[str, Any]:
    if not angles_deg:
        raise ValueError("cannot bound an empty angle set")
    normalized = sorted(float(value) % 360.0 for value in angles_deg)
    if len(normalized) == 1:
        center = _normalize_angle_deg(normalized[0])
        return {
            "minimum_cover_start_deg": center,
            "minimum_cover_end_deg": center,
            "datum_deg": center,
            "span_deg": 0.0,
            "half_range_deg": 0.0,
        }
    gaps = [
        (normalized[(index + 1) % len(normalized)]
         + (360.0 if index == len(normalized) - 1 else 0.0)
         - normalized[index])
        for index in range(len(normalized))
    ]
    gap_index = max(range(len(gaps)), key=gaps.__getitem__)
    start = normalized[(gap_index + 1) % len(normalized)]
    span = 360.0 - gaps[gap_index]
    center_unwrapped = start + span / 2.0
    center = _normalize_angle_deg(center_unwrapped)
    return {
        "minimum_cover_start_deg": center - span / 2.0,
        "minimum_cover_end_deg": center + span / 2.0,
        "datum_deg": center,
        "span_deg": span,
        "half_range_deg": span / 2.0,
    }


def _linear_angle_bounds(angles_deg: Sequence[float]) -> dict[str, float]:
    minimum = min(map(float, angles_deg))
    maximum = max(map(float, angles_deg))
    span = maximum - minimum
    return {
        "min_deg": minimum,
        "max_deg": maximum,
        "datum_deg": (minimum + maximum) / 2.0,
        "span_deg": span,
        "half_range_deg": span / 2.0,
    }


def _direction_angles(vector: Sequence[float]) -> dict[str, float]:
    x, y, z = _unit(vector)
    return {
        "yaw_about_positive_Z_deg": math.degrees(math.atan2(y, x)),
        "elevation_from_XY_deg": math.degrees(
            math.atan2(z, math.hypot(x, y))
        ),
    }


def _axis_offset_to_bounds(
    point: Sequence[float], bounds: Mapping[str, Sequence[float]],
) -> tuple[list[float], list[float]]:
    signed: list[float] = []
    absolute: list[float] = []
    for index, value in enumerate(map(float, point)):
        minimum = float(bounds["min_mm"][index])
        maximum = float(bounds["max_mm"][index])
        if value < minimum:
            delta = value - minimum
        elif value > maximum:
            delta = value - maximum
        else:
            delta = 0.0
        signed.append(delta)
        absolute.append(abs(delta))
    return signed, absolute


def _active_nose_envelope(
    identity: carriage.OccurrenceIdentity,
) -> dict[str, Any]:
    points = [
        carriage.occurrence_nose_center(
            identity,
            radial_state=radial_state,
            coarse_base_mm=carriage.COARSE_ACTIVE_BASE_MM,
            passive_tangential_mm=passive_mm,
        )
        for radial_state in ("retracted", "extended")
        for passive_mm in (
            -carriage.PASSIVE_TANGENTIAL_USABLE_MM,
            carriage.PASSIVE_TANGENTIAL_USABLE_MM,
        )
    ]
    local = _bounds(points)
    machine_points = [
        carriage.active_local_to_machine_reference(point)
        for point in _corners(local)
    ]
    return {
        "derivation": (
            "occurrence_nose_center at active coarse base, radial states "
            "retracted/extended, passive tangential endpoints"
        ),
        "active_local_bounds": local,
        "active_local_reference_center_mm": _midpoint(local),
        "M0_home_machine_bounds": _bounds(machine_points),
        "M0_home_machine_reference_center_mm": list(
            carriage.active_local_to_machine_reference(tuple(_midpoint(local)))
        ),
    }


def _validate_source_nose_BREP_centers() -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    tangential_states = (
        ("negative", -carriage.PASSIVE_TANGENTIAL_USABLE_MM),
        ("center", 0.0),
        ("positive", carriage.PASSIVE_TANGENTIAL_USABLE_MM),
    )
    for identity in carriage.OCCURRENCE_IDENTITIES:
        for radial_state in ("retracted", "extended"):
            for tangential_state, passive_mm in tangential_states:
                source_nose = carriage.follower.nose_insert(
                    radial_state, tangential_state,
                )
                placed_nose = carriage._handed_source_part(
                    source_nose,
                    identity,
                    carriage.COARSE_ACTIVE_BASE_MM,
                    "placement_trade_BREP_center_validation",
                )
                measured = _midpoint(_bbox(placed_nose))
                expected = list(carriage.occurrence_nose_center(
                    identity,
                    radial_state=radial_state,
                    coarse_base_mm=carriage.COARSE_ACTIVE_BASE_MM,
                    passive_tangential_mm=passive_mm,
                ))
                residual = _distance(measured, expected)
                rows.append({
                    "physical_id": identity.index,
                    "name": identity.name,
                    "radial_state": radial_state,
                    "tangential_state": tangential_state,
                    "measured_BREP_bbox_center_local_mm": measured,
                    "source_transform_center_local_mm": expected,
                    "residual_mm": residual,
                })
    maximum_residual = max(row["residual_mm"] for row in rows)
    if maximum_residual > GEOMETRY_TOLERANCE_MM:
        raise ValueError("replacement CAD nose BREP/transform mismatch")
    return {
        "checked_center_count": len(rows),
        "radial_states": ["retracted", "extended"],
        "tangential_states": ["negative", "center", "positive"],
        "maximum_BREP_bbox_center_to_source_transform_residual_mm": (
            maximum_residual
        ),
        "all_centers_match": True,
        "rows": rows,
    }


def _validate_upstream_contracts(
    c1: Mapping[str, Any], manifest: Mapping[str, Any],
) -> dict[str, Any]:
    rebound.validate_report_integrity(c1)
    if c1.get("coverage", {}).get(
            "analytic_C1_biarc_pass_case_count") != EXPECTED_NONZERO_CASES:
        raise ValueError("placement trade requires all 4,704 analytic C1 cases")

    source_contract = carriage.geometry_contract()
    manifest_contract = {
        key: manifest.get(key) for key in source_contract
    }
    if manifest_contract != source_contract:
        raise ValueError(
            "stale replacement carriage manifest does not match source contract"
        )
    manifest_step_hash = manifest.get("artifacts", {}).get("step_sha256")
    if not CARRIAGE_STEP_PATH.is_file():
        raise ValueError("replacement carriage STEP is missing")
    if manifest_step_hash != _sha256(CARRIAGE_STEP_PATH):
        raise ValueError("replacement carriage manifest STEP hash mismatch")
    nose_BREP_validation = _validate_source_nose_BREP_centers()
    return {
        "source_geometry_contract_sha256": _canonical_hash(source_contract),
        "manifest_geometry_contract_sha256": _canonical_hash(
            manifest_contract
        ),
        "manifest_matches_source_geometry_contract": True,
        "manifest_STEP_hash_matches_artifact": True,
        "source_nose_BREP_center_validation": nose_BREP_validation,
    }


def _raw_cases(c1: Mapping[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for locus in c1["loci"]:
        for case in locus["diameter_cases"]:
            if case["status"] != "PASS_ANALYTIC_C1_S_BIARC":
                continue
            center = list(map(float, case[
                "follower_center_for_end_arc_mm"
            ]))
            endpoint = list(map(float, case["end_mm"]))
            curvature_normal = _unit(_sub(center, endpoint))
            aggregate_normal = _unit(case["aggregate_outward_normal"])
            tangent = _unit(case["end_tangent"])
            dot = _dot(curvature_normal, aggregate_normal)
            angle = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
            rows.append({
                "locus_index": int(locus["locus_index"]),
                "pass_index": int(locus["pass_index"]),
                "state_index": int(locus["state_index"]),
                "turn_index": int(locus["turn_index"]),
                "half_turn_index": int(locus["half_turn_index"]),
                "tooth_index": int(locus["tooth_index"]),
                "time_s": float(locus["time_s"]),
                "lane_id": str(locus["lane_id"]),
                "identity": dict(locus["identity"]),
                "wire_diameter_mm": float(case["wire_diameter_mm"]),
                "required_center_local_mm": center,
                "required_center_M0_home_machine_mm": list(
                    carriage.active_local_to_machine_reference(tuple(center))
                ),
                "aggregate_contact_local_mm": endpoint,
                "required_curvature_normal_contact_to_center": list(
                    curvature_normal
                ),
                "required_curvature_normal_angles": _direction_angles(
                    curvature_normal
                ),
                "current_reference_nose_orientation": {
                    "convex_arc_plane": "active_local_XY_at_fixed_Z",
                    "nose_cylinder_axis": "+Z_stator_axis",
                    "required_normal_elevation_from_reference_plane_deg": abs(
                        _direction_angles(curvature_normal)[
                            "elevation_from_XY_deg"
                        ]
                    ),
                    "fixed_reference_pose_covers_required_curvature_normal": (
                        abs(curvature_normal[2]) <= GEOMETRY_TOLERANCE_MM
                    ),
                    "positive_volume_gimbal_pose_range_proved": False,
                },
                "required_guide_tangent": list(tangent),
                "required_guide_tangent_angles": _direction_angles(tangent),
                "aggregate_outward_normal": list(aggregate_normal),
                "curvature_normal_dot_aggregate_outward_normal": dot,
                "curvature_to_aggregate_normal_angle_deg": angle,
                "aggregate_compression_projection_fraction": abs(dot),
                "circular_nose_can_supply_aggregate_normal_preload": (
                    abs(dot) >= 1.0 - rebound.NORMAL_ALIGNMENT_TOLERANCE
                ),
            })
    if len(rows) != EXPECTED_NONZERO_CASES:
        raise ValueError("placement trade did not collect exactly 4,704 cases")
    return rows


def _identity_stage_contract(
    identity_rows: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    centers = [row["required_center_local_mm"] for row in identity_rows]
    center_bounds = _bounds(centers)
    datum = _midpoint(center_bounds)
    curvature_yaw = [
        row["required_curvature_normal_angles"]
        ["yaw_about_positive_Z_deg"] for row in identity_rows
    ]
    curvature_elevation = [
        row["required_curvature_normal_angles"]
        ["elevation_from_XY_deg"] for row in identity_rows
    ]
    tangent_yaw = [
        row["required_guide_tangent_angles"]
        ["yaw_about_positive_Z_deg"] for row in identity_rows
    ]
    tangent_elevation = [
        row["required_guide_tangent_angles"]
        ["elevation_from_XY_deg"] for row in identity_rows
    ]
    return {
        "exact_target_center_bounds_local_mm": center_bounds,
        "exact_target_datum_local_mm": datum,
        "exact_target_datum_M0_home_machine_mm": list(
            carriage.active_local_to_machine_reference(tuple(datum))
        ),
        "exact_minimum_center_strokes_XYZ_mm": list(
            center_bounds["span_mm"]
        ),
        "curvature_normal_orientation": {
            "yaw": _minimum_covering_angle_interval(curvature_yaw),
            "elevation": _linear_angle_bounds(curvature_elevation),
        },
        "polished_guide_tangent_orientation": {
            "yaw": _minimum_covering_angle_interval(tangent_yaw),
            "elevation": _linear_angle_bounds(tangent_elevation),
        },
    }


def _diameter_changeover(
    identity_rows: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    by_locus: dict[int, dict[float, Mapping[str, Any]]] = {}
    for row in identity_rows:
        by_locus.setdefault(int(row["locus_index"]), {})[
            float(row["wire_diameter_mm"])
        ] = row
    pairs = []
    for locus_index, diameter_rows in sorted(by_locus.items()):
        if set(diameter_rows) != set(WIRE_DIAMETERS_MM):
            raise ValueError(
                f"locus {locus_index} is missing a diameter changeover pair"
            )
        thin = diameter_rows[WIRE_DIAMETERS_MM[0]]
        thick = diameter_rows[WIRE_DIAMETERS_MM[1]]
        translation = _sub(
            thick["required_center_local_mm"],
            thin["required_center_local_mm"],
        )
        normal_dot = _dot(
            thin["required_curvature_normal_contact_to_center"],
            thick["required_curvature_normal_contact_to_center"],
        )
        pairs.append({
            "locus_index": locus_index,
            "d0p5_minus_d0p2_center_translation_XYZ_mm": list(translation),
            "translation_magnitude_mm": _norm(translation),
            "curvature_normal_rotation_deg": math.degrees(math.acos(
                max(-1.0, min(1.0, normal_dot))
            )),
        })
    if len(pairs) != EXPECTED_DIAMETER_PAIRS_PER_IDENTITY:
        raise ValueError("diameter changeover pair coverage mismatch")
    vectors = [
        row["d0p5_minus_d0p2_center_translation_XYZ_mm"] for row in pairs
    ]
    vector_bounds = _bounds(vectors)
    magnitudes = [row["translation_magnitude_mm"] for row in pairs]
    rotations = [row["curvature_normal_rotation_deg"] for row in pairs]
    constant_vector = all(
        span <= GEOMETRY_TOLERANCE_MM
        for span in vector_bounds["span_mm"]
    )
    maximum_rotation = max(rotations)
    return {
        "paired_locus_count": len(pairs),
        "d0p5_minus_d0p2_translation_vector_bounds_mm": vector_bounds,
        "translation_magnitude_range_mm": [min(magnitudes), max(magnitudes)],
        "curvature_normal_rotation_range_deg": [
            min(rotations), maximum_rotation,
        ],
        "same_translation_vector_at_every_locus": constant_vector,
        "translation_only_changeover_shim_exact": (
            constant_vector and maximum_rotation <= ANGLE_TOLERANCE_DEG
        ),
        "implication": (
            "a single fixed shim cannot reproduce the locus-varying centre "
            "and orientation change"
            if not constant_vector else
            "centre translation is constant, but the keyed guide orientation "
            "must also change; translation alone is not exact"
        ),
        "pairs": pairs,
    }


def _carrier_floor_contract() -> tuple[dict[str, Any], dict[str, Any]]:
    pieces = carriage._selection_bay_pieces()
    expected_piece_count = len(carriage.OCCURRENCE_IDENTITIES) * 3
    if len(pieces) != expected_piece_count:
        raise ValueError(
            "replacement carriage selection-bay piece count changed"
        )
    floor_bounds: dict[str, Any] = {}
    for identity, floor in zip(
            carriage.OCCURRENCE_IDENTITIES, pieces[0::3]):
        bounds = _bbox(floor)
        if not math.isclose(
                bounds["span_mm"][2], 1.0, abs_tol=GEOMETRY_TOLERANCE_MM):
            raise ValueError("selection-bay floor identification failed")
        floor_bounds[str(identity.index)] = {
            "name": identity.name,
            "bounds_local_mm": bounds,
            "axial_outward_face_local_Z_mm": (
                bounds["max_mm"][2]
                if identity.axial_sign > 0 else bounds["min_mm"][2]
            ),
        }
    carrier = carriage.replacement_carrier()
    return floor_bounds, {
        "carrier_BREP_bounds_local_mm": _bbox(carrier),
        "carrier_BREP_volume_mm3": float(carrier.volume),
    }


def analyze() -> dict[str, Any]:
    c1 = _load(C1_REPORT_PATH)
    manifest = _load(CARRIAGE_MANIFEST_PATH)
    upstream = _validate_upstream_contracts(c1, manifest)
    raw_rows = _raw_cases(c1)

    identities = {
        int(identity.index): identity
        for identity in carriage.OCCURRENCE_IDENTITIES
    }
    current_envelopes = {
        str(index): {
            "name": identity.name,
            "owner": "M0_carriage",
            "M1_spatial_transform": False,
            "M2_spatial_transform": False,
            **_active_nose_envelope(identity),
        }
        for index, identity in identities.items()
    }
    floor_contracts, carrier_contract = _carrier_floor_contract()

    stage_contracts: dict[str, Any] = {}
    changeovers: dict[str, Any] = {}
    for physical_id, identity in identities.items():
        selected = [
            row for row in raw_rows
            if int(row["identity"]["physical_id"]) == physical_id
        ]
        if len(selected) != EXPECTED_CASES_PER_IDENTITY:
            raise ValueError("per-identity placement coverage mismatch")
        stage_contracts[str(physical_id)] = {
            "name": identity.name,
            **_identity_stage_contract(selected),
        }
        datum = stage_contracts[str(physical_id)][
            "exact_target_datum_local_mm"
        ]
        current_reference = current_envelopes[str(physical_id)][
            "active_local_reference_center_mm"
        ]
        re_datum = _sub(datum, current_reference)
        stage_contracts[str(physical_id)].update({
            "current_CAD_reference_to_target_datum_translation_XYZ_mm": list(
                re_datum
            ),
            "absolute_current_CAD_reference_to_target_datum_offset_XYZ_mm": [
                abs(value) for value in re_datum
            ],
            "prototype_65deg_two_axis_half_range_covers_reported_intervals": (
                stage_contracts[str(physical_id)]
                ["curvature_normal_orientation"]["yaw"]["half_range_deg"]
                <= rebound.PROTOTYPE_GIMBAL_HALF_RANGE_DEG
                + ANGLE_TOLERANCE_DEG
                and stage_contracts[str(physical_id)]
                ["curvature_normal_orientation"]["elevation"]
                ["half_range_deg"]
                <= rebound.PROTOTYPE_GIMBAL_HALF_RANGE_DEG
                + ANGLE_TOLERANCE_DEG
            ),
            "positive_volume_gimbal_range_proved": False,
        })
        changeovers[str(physical_id)] = {
            "name": identity.name,
            **_diameter_changeover(selected),
        }

    comparisons: list[dict[str, Any]] = []
    for row in raw_rows:
        physical_id = int(row["identity"]["physical_id"])
        key = str(physical_id)
        identity = identities[physical_id]
        center = row["required_center_local_mm"]
        current = current_envelopes[key]
        signed_offset, absolute_offset = _axis_offset_to_bounds(
            center, current["active_local_bounds"],
        )
        reference_delta = _sub(
            center, current["active_local_reference_center_mm"],
        )
        stage = stage_contracts[key]
        stage_delta = _sub(center, stage["exact_target_datum_local_mm"])
        stage_half_strokes = [
            value / 2.0
            for value in stage["exact_minimum_center_strokes_XYZ_mm"]
        ]
        yaw = row["required_curvature_normal_angles"][
            "yaw_about_positive_Z_deg"
        ]
        elevation = row["required_curvature_normal_angles"][
            "elevation_from_XY_deg"
        ]
        yaw_datum = stage["curvature_normal_orientation"]["yaw"][
            "datum_deg"
        ]
        elevation_datum = stage["curvature_normal_orientation"][
            "elevation"
        ]["datum_deg"]
        floor = floor_contracts[key]["bounds_local_mm"]
        projection_inside = all(
            float(floor["min_mm"][index]) - GEOMETRY_TOLERANCE_MM
            <= float(center[index])
            <= float(floor["max_mm"][index]) + GEOMETRY_TOLERANCE_MM
            for index in (0, 1)
        )
        floor_outward_signed = max(
            identity.axial_sign * float(floor["min_mm"][2]),
            identity.axial_sign * float(floor["max_mm"][2]),
        )
        nose_inward_surface_signed = (
            identity.axial_sign * float(center[2])
            - NOSE_SURFACE_RADIUS_MM
        )
        floor_clearance = (
            nose_inward_surface_signed - floor_outward_signed
        )
        comparison = dict(row)
        comparison.update({
            "current_CAD_nose_envelope": {
                "signed_offset_to_reachable_envelope_XYZ_mm": signed_offset,
                "absolute_offset_to_reachable_envelope_XYZ_mm": absolute_offset,
                "minimum_Euclidean_offset_to_reachable_envelope_mm": _norm(
                    absolute_offset
                ),
                "signed_offset_from_reference_center_XYZ_mm": list(
                    reference_delta
                ),
                "absolute_offset_from_reference_center_XYZ_mm": [
                    abs(value) for value in reference_delta
                ],
                "radial_axis_covered": (
                    absolute_offset[0] <= GEOMETRY_TOLERANCE_MM
                ),
                "tangential_axis_covered": (
                    absolute_offset[1] <= GEOMETRY_TOLERANCE_MM
                ),
                "axial_axis_covered": (
                    absolute_offset[2] <= GEOMETRY_TOLERANCE_MM
                ),
                "full_center_covered": all(
                    value <= GEOMETRY_TOLERANCE_MM
                    for value in absolute_offset
                ),
            },
            "successor_stage": {
                "offset_from_target_datum_XYZ_mm": list(stage_delta),
                "inside_exact_center_stroke_envelope": all(
                    abs(stage_delta[index])
                    <= stage_half_strokes[index] + GEOMETRY_TOLERANCE_MM
                    for index in range(3)
                ),
                "curvature_normal_yaw_from_datum_deg": (
                    _signed_angle_delta_deg(yaw, yaw_datum)
                ),
                "curvature_normal_elevation_from_datum_deg": (
                    elevation - elevation_datum
                ),
            },
            "carrier_host_screen": {
                "target_center_XY_inside_matching_floor_footprint": (
                    projection_inside
                ),
                "R3_nose_surface_to_floor_axial_clearance_mm": floor_clearance,
                "meets_nominal_2mm_clearance": (
                    floor_clearance
                    >= NOMINAL_CARRIER_CLEARANCE_TARGET_MM
                    - GEOMETRY_TOLERANCE_MM
                ),
            },
        })
        comparisons.append(comparison)

    per_identity: dict[str, Any] = {}
    for physical_id, identity in identities.items():
        key = str(physical_id)
        identity_rows = [
            row for row in comparisons
            if int(row["identity"]["physical_id"]) == physical_id
        ]
        per_diameter: dict[str, Any] = {}
        for diameter in WIRE_DIAMETERS_MM:
            selected = [
                row for row in identity_rows
                if math.isclose(
                    float(row["wire_diameter_mm"]), diameter,
                    abs_tol=1.0e-12,
                )
            ]
            offset_axes = list(zip(*[
                row["current_CAD_nose_envelope"]
                ["absolute_offset_to_reachable_envelope_XYZ_mm"]
                for row in selected
            ]))
            reference_axes = list(zip(*[
                row["current_CAD_nose_envelope"]
                ["absolute_offset_from_reference_center_XYZ_mm"]
                for row in selected
            ]))
            distances = [
                row["current_CAD_nose_envelope"]
                ["minimum_Euclidean_offset_to_reachable_envelope_mm"]
                for row in selected
            ]
            per_diameter[f"d{diameter:.1f}"] = {
                "case_count": len(selected),
                "required_center_bounds_local_mm": _bounds([
                    row["required_center_local_mm"] for row in selected
                ]),
                "absolute_axis_offset_to_current_envelope_ranges_XYZ_mm": [
                    [min(axis), max(axis)] for axis in offset_axes
                ],
                "absolute_axis_offset_from_current_reference_ranges_XYZ_mm": [
                    [min(axis), max(axis)] for axis in reference_axes
                ],
                "minimum_Euclidean_offset_to_current_envelope_range_mm": [
                    min(distances), max(distances),
                ],
                "radial_axis_covered_case_count": sum(
                    row["current_CAD_nose_envelope"]
                    ["radial_axis_covered"] for row in selected
                ),
                "tangential_axis_covered_case_count": sum(
                    row["current_CAD_nose_envelope"]
                    ["tangential_axis_covered"] for row in selected
                ),
                "axial_axis_covered_case_count": sum(
                    row["current_CAD_nose_envelope"]
                    ["axial_axis_covered"] for row in selected
                ),
                "full_center_covered_case_count": sum(
                    row["current_CAD_nose_envelope"]
                    ["full_center_covered"] for row in selected
                ),
            }
        per_identity[key] = {
            "name": identity.name,
            "case_count": len(identity_rows),
            "current_CAD_nose_envelope": current_envelopes[key],
            "successor_stage_contract": stage_contracts[key],
            "diameter_changeover": changeovers[key],
            "per_diameter": per_diameter,
        }

    current_covered = sum(
        row["current_CAD_nose_envelope"]["full_center_covered"]
        for row in comparisons
    )
    successor_covered = sum(
        row["successor_stage"]["inside_exact_center_stroke_envelope"]
        for row in comparisons
    )
    compression_compatible = sum(
        row["circular_nose_can_supply_aggregate_normal_preload"]
        for row in comparisons
    )
    current_reference_orientation_covered = sum(
        row["current_reference_nose_orientation"]
        ["fixed_reference_pose_covers_required_curvature_normal"]
        for row in comparisons
    )
    projection_covered = sum(
        row["carrier_host_screen"]
        ["target_center_XY_inside_matching_floor_footprint"]
        for row in comparisons
    )
    positive_floor_clearance = sum(
        row["carrier_host_screen"]
        ["R3_nose_surface_to_floor_axial_clearance_mm"]
        >= -GEOMETRY_TOLERANCE_MM
        for row in comparisons
    )
    nominal_floor_clearance = sum(
        row["carrier_host_screen"]["meets_nominal_2mm_clearance"]
        for row in comparisons
    )
    floor_clearances = [
        row["carrier_host_screen"]
        ["R3_nose_surface_to_floor_axial_clearance_mm"]
        for row in comparisons
    ]
    compression_angles = [
        row["curvature_to_aggregate_normal_angle_deg"]
        for row in comparisons
    ]
    compression_projections = [
        row["aggregate_compression_projection_fraction"]
        for row in comparisons
    ]
    common_strokes = [
        max(
            stage_contracts[str(physical_id)]
            ["exact_minimum_center_strokes_XYZ_mm"][axis]
            for physical_id in identities
        )
        for axis in range(3)
    ]

    analytic_gates = {
        "upstream_C1_report_and_carriage_artifacts_hash_bound": True,
        "exactly_4704_nonzero_cases_compared": (
            len(comparisons) == EXPECTED_NONZERO_CASES
        ),
        "all_four_identities_and_both_diameters_covered": (
            set(int(row["identity"]["physical_id"])
                for row in comparisons) == set(range(4))
            and all(
                sum(math.isclose(
                    float(row["wire_diameter_mm"]), diameter,
                    abs_tol=1.0e-12,
                ) for row in comparisons) == EXPECTED_CASES_PER_DIAMETER
                for diameter in WIRE_DIAMETERS_MM
            )
        ),
        "successor_exact_center_envelopes_cover_all_cases": (
            successor_covered == EXPECTED_NONZERO_CASES
        ),
        "successor_orientation_ranges_cover_all_cases": all(
            abs(row["successor_stage"]
                ["curvature_normal_yaw_from_datum_deg"])
            <= stage_contracts[str(row["identity"]["physical_id"])]
            ["curvature_normal_orientation"]["yaw"]["half_range_deg"]
            + ANGLE_TOLERANCE_DEG
            and abs(row["successor_stage"]
                    ["curvature_normal_elevation_from_datum_deg"])
            <= stage_contracts[str(row["identity"]["physical_id"])]
            ["curvature_normal_orientation"]["elevation"]
            ["half_range_deg"] + ANGLE_TOLERANCE_DEG
            for row in comparisons
        ),
        "all_target_center_projections_inside_current_bay_floors": (
            projection_covered == EXPECTED_NONZERO_CASES
        ),
        "all_R3_nose_envelopes_have_nonnegative_floor_separation": (
            positive_floor_clearance == EXPECTED_NONZERO_CASES
        ),
    }
    physical_gates = {
        "current_CAD_nose_envelope_covers_all_required_centers": (
            current_covered == EXPECTED_NONZERO_CASES
        ),
        "current_CAD_fixed_nose_pose_covers_all_required_normals": (
            current_reference_orientation_covered == EXPECTED_NONZERO_CASES
        ),
        "circular_nose_supplies_aggregate_normal_compression": (
            compression_compatible == EXPECTED_NONZERO_CASES
        ),
        "current_carrier_has_nominal_2mm_R3_floor_clearance": (
            nominal_floor_clearance == EXPECTED_NONZERO_CASES
        ),
        "positive_volume_successor_stage_modeled": False,
        "polished_C1_guide_surface_placed": False,
        "independent_aggregate_normal_preload_modeled": False,
        "successor_stage_collision_and_transition_sweeps_passed": False,
        "successor_tolerance_load_wear_and_dynamics_passed": False,
        "assembly_integration_passed": False,
    }
    blockers = [
        name for name, value in physical_gates.items() if not value
    ]

    artifacts = {
        "C1_rebound_report": {
            "path": str(C1_REPORT_PATH.relative_to(ROOT)).replace("\\", "/"),
            "sha256": _sha256(C1_REPORT_PATH),
            "report_sha256": c1.get("report_sha256"),
        },
        "replacement_carriage_manifest": {
            "path": str(CARRIAGE_MANIFEST_PATH.relative_to(ROOT)).replace(
                "\\", "/"
            ),
            "sha256": _sha256(CARRIAGE_MANIFEST_PATH),
        },
        "replacement_carriage_STEP": {
            "path": str(CARRIAGE_STEP_PATH.relative_to(ROOT)).replace(
                "\\", "/"
            ),
            "sha256": _sha256(CARRIAGE_STEP_PATH),
        },
    }
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "FAIL",
        "decision": (
            "CURRENT_NOSE_DATUM_MISSES_ALL_4704__M0_OWNED_REDATUMED_"
            "THREE_AXIS_C1_GUIDE_STAGE_IS_ANALYTIC_ONLY__"
            "AGGREGATE_NORMAL_PRELOAD_MUST_BE_DECOUPLED"
        ),
        **{key: False for key in AUTHORITY_KEYS},
        "scope": {
            "proved": [
                "exact source-level current nose transforms and reachable envelopes",
                "per-case radial, tangential, and axial offsets for 4,704 C1 centres",
                "fixed-reference XY nose-plane mismatch against all required normals",
                "minimum per-identity translation strokes and orientation intervals",
                "paired 0.2-to-0.5 mm diameter changeover translations and rotations",
                "analytic matching-floor footprint and R3 axial-clearance screen",
                "zero aggregate-normal load projection of the circular end-arc nose",
            ],
            "not_proved": [
                "positive-volume successor guide, flexures, slides, springs, or stops",
                "full-carrier collision, transition, tolerance, load, wear, or dynamics",
                "assembly, wire route, dancer coupling, BOM, procurement, or release",
            ],
        },
        "frame_contract": {
            "active_local_axes": {
                "+X": "radial_outward",
                "+Y": "tangential",
                "+Z": "stator_axis",
            },
            "M0_home_transform": "machine=(-local_y,local_z,95-local_x)",
            "owner": "M0_carriage",
            "M1_spatial_transform": False,
            "M2_spatial_transform": False,
        },
        "coverage": {
            "required_nonzero_cases": EXPECTED_NONZERO_CASES,
            "compared_nonzero_cases": len(comparisons),
            "cases_per_identity": {
                str(physical_id): sum(
                    int(row["identity"]["physical_id"]) == physical_id
                    for row in comparisons
                ) for physical_id in identities
            },
            "cases_per_diameter": {
                f"d{diameter:.1f}": sum(
                    math.isclose(
                        float(row["wire_diameter_mm"]), diameter,
                        abs_tol=1.0e-12,
                    ) for row in comparisons
                ) for diameter in WIRE_DIAMETERS_MM
            },
            "current_CAD_full_center_covered_case_count": current_covered,
            "current_CAD_fixed_pose_normal_covered_case_count": (
                current_reference_orientation_covered
            ),
            "successor_analytic_center_covered_case_count": successor_covered,
            "circular_nose_compression_compatible_case_count": (
                compression_compatible
            ),
            "carrier_floor_XY_projection_covered_case_count": projection_covered,
            "nonnegative_R3_floor_clearance_case_count": positive_floor_clearance,
            "nominal_2mm_R3_floor_clearance_case_count": nominal_floor_clearance,
        },
        "current_replacement_CAD": {
            "geometry_contract_binding": upstream,
            "nose_envelopes_by_identity": current_envelopes,
            **carrier_contract,
            "matching_selection_bay_floors": floor_contracts,
        },
        "successor_trade": {
            "selected_topology": (
                "four M0-owned re-datumed XYZ flexure/slide stages, each "
                "carrying a yaw/elevation-compliant polished C1 guide cartridge "
                "plus a separate aggregate-normal preload leaf"
            ),
            "why_this_is_the_smallest_supported_successor": (
                "XYZ travel is required by the measured centre spans; two guide "
                "orientation coordinates are required by the measured curvature-"
                "normal intervals; a separate preload leaf is required because the "
                "circular end-arc radius has zero aggregate-normal projection. A "
                "fixed sculpted guide might remove moving DOFs, but no positive-"
                "volume surface spanning all 4,704 cases exists yet."
            ),
            "owner": "M0_carriage",
            "M1_spatial_transform": False,
            "M2_spatial_transform": False,
            "coarse_selector_remains_separate": True,
            "common_exact_minimum_center_strokes_XYZ_mm": common_strokes,
            "per_identity": stage_contracts,
            "diameter_changeover": changeovers,
            "compression_decoupling": {
                "required": True,
                "curvature_to_aggregate_normal_angle_range_deg": [
                    min(compression_angles), max(compression_angles),
                ],
                "aggregate_normal_projection_fraction_range": [
                    min(compression_projections), max(compression_projections),
                ],
                "translation_can_fix": False,
                "gimbal_rotation_can_fix_while_preserving_circular_arc_center": False,
                "required_mechanical_split": (
                    "polished guide owns C1 route curvature; independent leaf or "
                    "shoe owns force along aggregate support normal"
                ),
            },
        },
        "carrier_host_screen": {
            "mode": "analytic_floor_footprint_and_axial_R3_envelope_only",
            "target_center_projection_all_inside": (
                projection_covered == EXPECTED_NONZERO_CASES
            ),
            "R3_surface_to_floor_clearance_range_mm": [
                min(floor_clearances), max(floor_clearances),
            ],
            "all_have_nonnegative_clearance": (
                positive_floor_clearance == EXPECTED_NONZERO_CASES
            ),
            "all_meet_nominal_2mm_clearance": (
                nominal_floor_clearance == EXPECTED_NONZERO_CASES
            ),
            "current_carrier_could_host_center_kinematics_analytically": (
                projection_covered == EXPECTED_NONZERO_CASES
                and positive_floor_clearance == EXPECTED_NONZERO_CASES
            ),
            "current_carrier_host_authorized": False,
            "limitation": (
                "no positive-volume stage, guide, fastener, spring, stop, or full-"
                "carrier collision envelope was placed"
            ),
        },
        "per_identity": per_identity,
        "case_comparisons": comparisons,
        "analytic_gates": analytic_gates,
        "physical_gates": physical_gates,
        "blockers": blockers,
        "artifacts": artifacts,
        "source_hashes": {
            str(path).replace("\\", "/"): _sha256(ROOT / path)
            for path in SOURCE_PATHS
        },
    }
    report["report_sha256"] = _canonical_hash(report, "report_sha256")
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported follower placement trade schema")
    if report.get("report_sha256") != _canonical_hash(
            report, "report_sha256"):
        raise ValueError("follower placement trade report hash mismatch")
    for relative, expected in report.get("source_hashes", {}).items():
        path = ROOT / str(relative).replace("/", "\\")
        if not path.is_file() or _sha256(path) != expected:
            raise ValueError(f"stale follower placement source {relative}")
    for name, artifact in report.get("artifacts", {}).items():
        path = ROOT / str(artifact["path"]).replace("/", "\\")
        if not path.is_file() or _sha256(path) != artifact.get("sha256"):
            raise ValueError(f"stale follower placement artifact {name}")
    c1_artifact = report.get("artifacts", {}).get("C1_rebound_report", {})
    c1 = _load(C1_REPORT_PATH)
    if c1_artifact.get("report_sha256") != c1.get("report_sha256"):
        raise ValueError("stale follower placement C1 report binding")
    coverage = report.get("coverage", {})
    if coverage.get("compared_nonzero_cases") != EXPECTED_NONZERO_CASES:
        raise ValueError("follower placement case coverage mismatch")
    if coverage.get("current_CAD_full_center_covered_case_count") != 0:
        raise ValueError("current CAD unexpectedly promoted as centre-compatible")
    if coverage.get(
            "successor_analytic_center_covered_case_count") != EXPECTED_NONZERO_CASES:
        raise ValueError("successor analytic centre envelope is incomplete")
    if coverage.get("circular_nose_compression_compatible_case_count") != 0:
        raise ValueError("circular follower incorrectly promoted for compression")
    if not all(report.get("analytic_gates", {}).values()):
        raise ValueError("follower placement analytic gate failure")
    if any(report.get("physical_gates", {}).values()):
        raise ValueError("follower placement physical gates must remain false")
    if report.get("status") != "FAIL":
        raise ValueError("follower placement trade must remain fail-closed")
    for key in AUTHORITY_KEYS:
        if report.get(key) is not False:
            raise ValueError(f"follower placement trade cannot authorize {key}")


def render_markdown(report: Mapping[str, Any]) -> str:
    coverage = report["coverage"]
    host = report["carrier_host_screen"]
    successor = report["successor_trade"]
    compression = successor["compression_decoupling"]
    lines = [
        "# Aggregate-boundary follower placement trade",
        "",
        f"**{report['status']} — `{report['decision']}`**",
        "",
        "## Bottom line",
        "",
        (
            f"All {coverage['compared_nonzero_cases']} nonzero C1 centres were "
            "compared. The current replacement-CAD nose envelope covers "
            f"{coverage['current_CAD_full_center_covered_case_count']}; the "
            "re-datumed analytic XYZ envelopes cover "
            f"{coverage['successor_analytic_center_covered_case_count']}."
        ),
        (
            "A circular R3 nose remains compression-incompatible in every case: "
            f"the curvature-to-aggregate-normal angle is "
            f"{compression['curvature_to_aggregate_normal_angle_range_deg'][0]:.9f} "
            f"to {compression['curvature_to_aggregate_normal_angle_range_deg'][1]:.9f} deg."
        ),
        "",
        "## Absolute miss to the current reachable nose envelope",
        "",
    ]
    for physical_id, identity in report["per_identity"].items():
        for diameter, row in identity["per_diameter"].items():
            axes = row[
                "absolute_axis_offset_to_current_envelope_ranges_XYZ_mm"
            ]
            lines.append(
                f"- `{physical_id} {identity['name']} {diameter}`: radial X "
                f"{axes[0][0]:.6f} to {axes[0][1]:.6f} mm; tangential Y "
                f"{axes[1][0]:.6f} to {axes[1][1]:.6f} mm; axial Z "
                f"{axes[2][0]:.6f} to {axes[2][1]:.6f} mm."
            )
    lines.extend([
        "",
        "## Exact successor centre contract",
        "",
        (
            "Common minimum XYZ stroke (using per-identity datums): "
            f"`{successor['common_exact_minimum_center_strokes_XYZ_mm']}` mm."
        ),
    ])
    for physical_id, row in successor["per_identity"].items():
        orientation = row["curvature_normal_orientation"]
        lines.append(
            f"- `{physical_id} {row['name']}`: datum "
            f"`{row['exact_target_datum_local_mm']}` mm; XYZ stroke "
            f"`{row['exact_minimum_center_strokes_XYZ_mm']}` mm; yaw "
            f"{orientation['yaw']['minimum_cover_start_deg']:.6f} to "
            f"{orientation['yaw']['minimum_cover_end_deg']:.6f} deg; elevation "
            f"{orientation['elevation']['min_deg']:.6f} to "
            f"{orientation['elevation']['max_deg']:.6f} deg; re-datum from "
            f"current reference "
            f"`{row['current_CAD_reference_to_target_datum_translation_XYZ_mm']}` "
            "mm."
        )
    lines.extend([
        "",
        "## Diameter changeover",
        "",
    ])
    for physical_id, row in successor["diameter_changeover"].items():
        lines.append(
            f"- `{physical_id} {row['name']}`: translation magnitude "
            f"{row['translation_magnitude_range_mm'][0]:.6f} to "
            f"{row['translation_magnitude_range_mm'][1]:.6f} mm; normal rotation "
            f"{row['curvature_normal_rotation_range_deg'][0]:.6f} to "
            f"{row['curvature_normal_rotation_range_deg'][1]:.6f} deg; "
            f"translation-only shim exact = "
            f"`{row['translation_only_changeover_shim_exact']}`."
        )
    lines.extend([
        "",
        "## Carrier host screen",
        "",
        (
            "All target-centre XY projections lie over their matching current "
            f"selection-bay floors: `{host['target_center_projection_all_inside']}`."
        ),
        (
            "R3 surface-to-floor clearance range: "
            f"{host['R3_surface_to_floor_clearance_range_mm'][0]:.6f} to "
            f"{host['R3_surface_to_floor_clearance_range_mm'][1]:.6f} mm."
        ),
        (
            "This is positive analytically but does not meet the 2.00 mm nominal "
            f"screen: `{host['all_meet_nominal_2mm_clearance']}`. No positive-"
            "volume host or collision authority is granted."
        ),
        "",
        "## Selected successor topology",
        "",
        successor["selected_topology"] + ".",
        "",
        successor["why_this_is_the_smallest_supported_successor"],
        "",
        "## Open physical gates",
        "",
    ])
    lines.extend(f"- `{name}`" for name in report["blockers"])
    lines.append("")
    return "\n".join(lines)


def write_outputs(report: Mapping[str, Any] | None = None) -> dict[str, Any]:
    value = dict(report) if report is not None else analyze()
    validate_report_integrity(value)
    REPORTS.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(
        json.dumps(value, indent=2, sort_keys=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    OUTPUT_MD.write_text(render_markdown(value), encoding="utf-8")
    return value


def main(argv: Iterable[str] | None = None) -> int:
    del argv
    report = write_outputs()
    print(json.dumps({
        "status": report["status"],
        "decision": report["decision"],
        "coverage": report["coverage"],
        "common_exact_minimum_center_strokes_XYZ_mm": report[
            "successor_trade"
        ]["common_exact_minimum_center_strokes_XYZ_mm"],
        "carrier_host_screen": report["carrier_host_screen"],
        "report_sha256": report["report_sha256"],
    }, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
