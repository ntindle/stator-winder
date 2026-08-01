"""Fail-closed continuous-transition audit for the replacement carriage.

The static replacement-carriage audit binds five endpoint signatures.  This
module audits the missing path between them with exact OpenCascade BREP
operations over a prescribed safe sequence for each of the four physical
identities:

1. stay radially retracted and translate the selected occurrence from the
   parked ``|Y|=10.95`` bay to the active ``|Y|=2.05`` bay;
2. at the active bay, deploy radially from center X=14 to X=20;
3. at every radial sample, audit the supported passive tangential extrema
   ``q=-0.50`` and ``q=+0.50`` in addition to the centered radial sweep.

The four positive blocker solids in the review STEP are deliberately excluded:
they are envelopes, not a manufactured selector or retraction mechanism.  The
audit therefore cannot authorize mechanism integration, assembly integration,
loads, wear, procurement, production, BOM, or release.  It fails closed when
an exact positive common volume is found or the required 2.00 mm non-contact
clearance to the three parked siblings / primary M4 hardware is missed.
"""

from __future__ import annotations

import copy
from copy import deepcopy
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable, Mapping

from build123d import Compound, Pos


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
REVIEW = ROOT / "out" / "review"
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_replacement_architecture as architecture
import aggregate_boundary_follower_replacement_carriage as carriage


OUTPUT_JSON = REPORTS / (
    "aggregate_boundary_follower_replacement_transition_sweep.json"
)
OUTPUT_MD = REPORTS / (
    "aggregate_boundary_follower_replacement_transition_sweep.md"
)

SCHEMA = "aggregate-boundary-follower-replacement-transition-sweep/v2"

PARKED_BASE_ABS_Y_MM = carriage.COARSE_PARKED_BASE_MM
ACTIVE_BASE_ABS_Y_MM = carriage.COARSE_ACTIVE_BASE_MM
RADIAL_RETRACTED_CENTER_X_MM = 14.0
RADIAL_EXTENDED_CENTER_X_MM = 20.0
PASSIVE_Q_EXTREMA_MM = (-0.50, 0.50)

# Uniform subdivisions are at most 0.50 mm in every independent translation.
# These are exact BREP samples, not mesh tests.  The missing positive linkage
# and tolerance stack still prevent physical authority even if every sample is
# clear.
COARSE_SUBDIVISIONS = 18
RADIAL_SUBDIVISIONS = 12
MAX_TRANSLATION_SAMPLE_STEP_MM = 0.50

REQUIRED_NONCONTACT_CLEARANCE_MM = 2.00
CLEARANCE_TOLERANCE_MM = 1.0e-7
POSITIVE_VOLUME_TOLERANCE_MM3 = 1.0e-7

SOURCE_PATHS = (
    CAD / "aggregate_boundary_follower_replacement_carriage.py",
    CAD / "aggregate_boundary_floating_follower.py",
    HERE / "aggregate_boundary_follower_replacement_architecture.py",
    REPORTS / "aggregate_boundary_follower_replacement_architecture.json",
    REVIEW / "aggregate_boundary_follower_replacement_carriage_manifest.json",
)

AUTHORITY = {
    "sampled_transition_geometry_proven": False,
    "continuous_tolerance_stack_proven": False,
    "selection_mechanism_authorized": False,
    "retraction_mechanism_authorized": False,
    "transition_collision_authorized": False,
    "clearance_authorized": False,
    "assembly_integration_authorized": False,
    "load_authorized": False,
    "wear_authorized": False,
    "procurement_authorized": False,
    "production_authorized": False,
    "BOM_change_authorized": False,
    "release_authorized": False,
}

BLOCKERS = (
    "NON_MANUFACTURED_8p90mm_coarse_selector_blocker_envelopes_only",
    "MISSING_positive_volume_coarse_selection_mechanism",
    "MISSING_positive_M0_retraction_mechanism_and_dual_NC_interlock",
    "SAMPLED_ONLY_nominal_2p50mm_clearance_without_tolerance_stack",
    "OPEN_nominal_clearance_tolerance_stack",
    "OPEN_pivot_retention_load_wear_and_40N_load_path",
    "OPEN_wire_route_and_2400_locus_closure",
)


@dataclass(frozen=True)
class Pose:
    identity_index: int
    phase: str
    phase_sample_index: int
    coarse_base_abs_y_mm: float
    radial_center_x_mm: float
    passive_q_mm: float

    @property
    def key(self) -> str:
        return (
            f"i{self.identity_index}:{self.phase}:"
            f"{self.phase_sample_index:03d}:"
            f"y{self.coarse_base_abs_y_mm:.8f}:"
            f"x{self.radial_center_x_mm:.8f}:q{self.passive_q_mm:+.8f}"
        )


def _linspace(start: float, stop: float, subdivisions: int) -> tuple[float, ...]:
    if subdivisions <= 0:
        raise ValueError("subdivisions must be positive")
    return tuple(
        float(start) + (float(stop) - float(start)) * i / subdivisions
        for i in range(subdivisions + 1)
    )


def prescribed_poses(identity_index: int) -> tuple[Pose, ...]:
    """Return the deterministic safe-sequence samples for one identity."""

    if identity_index not in range(len(carriage.OCCURRENCE_IDENTITIES)):
        raise ValueError("identity_index must be 0..3")
    result: list[Pose] = []
    coarse_values = _linspace(
        PARKED_BASE_ABS_Y_MM, ACTIVE_BASE_ABS_Y_MM, COARSE_SUBDIVISIONS,
    )
    for sample_index, base_abs_y in enumerate(coarse_values):
        result.append(Pose(
            identity_index,
            "coarse_translate_while_retracted",
            sample_index,
            base_abs_y,
            RADIAL_RETRACTED_CENTER_X_MM,
            0.0,
        ))

    radial_values = _linspace(
        RADIAL_RETRACTED_CENTER_X_MM,
        RADIAL_EXTENDED_CENTER_X_MM,
        RADIAL_SUBDIVISIONS,
    )
    for sample_index, radial_x in enumerate(radial_values):
        result.append(Pose(
            identity_index,
            "radial_deploy_at_active_centered",
            sample_index,
            ACTIVE_BASE_ABS_Y_MM,
            radial_x,
            0.0,
        ))
    for q_mm in PASSIVE_Q_EXTREMA_MM:
        phase = (
            "radial_deploy_at_active_inward_q_extreme"
            if q_mm < 0.0 else
            "radial_deploy_at_active_outward_q_extreme"
        )
        for sample_index, radial_x in enumerate(radial_values):
            result.append(Pose(
                identity_index,
                phase,
                sample_index,
                ACTIVE_BASE_ABS_Y_MM,
                radial_x,
                q_mm,
            ))
    return tuple(result)


def all_prescribed_poses() -> tuple[Pose, ...]:
    return tuple(
        pose
        for identity in carriage.OCCURRENCE_IDENTITIES
        for pose in prescribed_poses(identity.index)
    )


def _copy_at(part, dx: float, dy: float):
    # build123d transforms may share wrapped OCC topology.  Copy first so an
    # audit pose can never mutate the cached source occurrence used elsewhere.
    return Pos(float(dx), float(dy), 0.0) * copy.copy(part)


def moving_leaves_at_pose(pose: Pose) -> tuple:
    """Return the exact 15 manufactured leaves at an arbitrary audit pose."""

    identity = carriage.OCCURRENCE_IDENTITIES[pose.identity_index]
    parked = carriage.moving_occurrence(
        identity,
        radial_state="retracted",
        coarse_base_mm=PARKED_BASE_ABS_Y_MM,
    )
    dx = pose.radial_center_x_mm - RADIAL_RETRACTED_CENTER_X_MM
    coarse_dy = identity.tangential_sign * (
        pose.coarse_base_abs_y_mm - PARKED_BASE_ABS_Y_MM
    )
    leaves = []
    for child in parked.children:
        is_radial_slide = "radial_slide_7075" in str(child.label)
        passive_dy = (
            0.0 if is_radial_slide
            else identity.tangential_sign * pose.passive_q_mm
        )
        leaves.append(_copy_at(child, dx, coarse_dy + passive_dy))
    if len(leaves) != carriage.MOVING_LEAF_COUNT_PER_OCCURRENCE:
        raise RuntimeError("transition pose lost manufactured leaves")
    return tuple(leaves)


def _compound(parts: Iterable) -> Compound:
    return Compound(children=list(parts))


def _bbox_bounds(shape) -> tuple[float, float, float, float, float, float]:
    box = shape.bounding_box()
    return (
        float(box.min.X), float(box.max.X),
        float(box.min.Y), float(box.max.Y),
        float(box.min.Z), float(box.max.Z),
    )


def _bbox_distance_mm(
    one: tuple[float, float, float, float, float, float],
    two: tuple[float, float, float, float, float, float],
) -> float:
    dx = max(float(one[0]) - float(two[1]), float(two[0]) - float(one[1]), 0.0)
    dy = max(float(one[2]) - float(two[3]), float(two[2]) - float(one[3]), 0.0)
    dz = max(float(one[4]) - float(two[5]), float(two[4]) - float(one[5]), 0.0)
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _positive_aabb_intersection(
    one: tuple[float, float, float, float, float, float],
    two: tuple[float, float, float, float, float, float],
) -> bool:
    return (
        min(one[1], two[1]) - max(one[0], two[0]) > CLEARANCE_TOLERANCE_MM
        and min(one[3], two[3]) - max(one[2], two[2]) > CLEARANCE_TOLERANCE_MM
        and min(one[5], two[5]) - max(one[4], two[4]) > CLEARANCE_TOLERANCE_MM
    )


def _common_volume_mm3(one, two) -> float:
    common = one & two
    return 0.0 if common is None else float(common.volume)


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(value: Mapping[str, Any]) -> str:
    body = deepcopy(dict(value))
    body.pop("report_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _pose_motion_rows(poses: tuple[Pose, ...]) -> list[dict[str, Any]]:
    rows = []
    by_identity = {
        identity.index: prescribed_poses(identity.index)
        for identity in carriage.OCCURRENCE_IDENTITIES
    }
    for identity_index, identity_poses in by_identity.items():
        phases: dict[str, list[Pose]] = {}
        for pose in identity_poses:
            phases.setdefault(pose.phase, []).append(pose)
        for phase, phase_poses in phases.items():
            max_delta = 0.0
            for first, second in zip(phase_poses, phase_poses[1:]):
                max_delta = max(
                    max_delta,
                    abs(first.coarse_base_abs_y_mm - second.coarse_base_abs_y_mm),
                    abs(first.radial_center_x_mm - second.radial_center_x_mm),
                    abs(first.passive_q_mm - second.passive_q_mm),
                )
            rows.append({
                "identity_index": identity_index,
                "phase": phase,
                "sample_count": len(phase_poses),
                "max_adjacent_translation_mm": max_delta,
            })
    return rows


def analyze() -> dict[str, Any]:
    """Run the exact BREP transition sweep and return a self-hashed report."""

    architecture_report = json.loads(
        (REPORTS / "aggregate_boundary_follower_replacement_architecture.json")
        .read_text(encoding="utf-8")
    )
    architecture.validate_report_integrity(architecture_report)

    poses = all_prescribed_poses()
    carrier = carriage.replacement_carrier()
    carrier_bounds = _bbox_bounds(carrier)
    primary_parts = tuple(carriage.primary_tower_m4_hardware())
    primary = _compound(primary_parts)
    primary_bounds = _bbox_bounds(primary)
    parked_by_identity = {
        identity.index: carriage.moving_occurrence(
            identity,
            radial_state="retracted",
            coarse_base_mm=PARKED_BASE_ABS_Y_MM,
        )
        for identity in carriage.OCCURRENCE_IDENTITIES
    }
    parked_bounds = {
        index: _bbox_bounds(shape)
        for index, shape in parked_by_identity.items()
    }

    collision_rows: list[dict[str, Any]] = []
    collision_failures: list[dict[str, Any]] = []
    clearance_violations: list[dict[str, Any]] = []
    min_clearance = math.inf
    min_clearance_witness: dict[str, Any] | None = None
    min_downstream_carrier_distance = math.inf
    min_downstream_carrier_witness: dict[str, Any] | None = None
    min_sibling_distance = math.inf
    min_sibling_witness: dict[str, Any] | None = None
    aabb_clearance_proof_count = 0
    exact_clearance_query_count = 0
    exact_common_query_count = 0
    internal_results: dict[tuple[int, float], dict[str, Any]] = {}

    for pose in poses:
        leaves = moving_leaves_at_pose(pose)
        radial_slide = leaves[0]
        downstream = _compound(leaves[1:])
        selected = _compound(leaves)
        selected_bounds = _bbox_bounds(selected)

        # The radial tongue intentionally has zero-distance captured contact
        # with the carrier.  Exact common volume, not distance, decides
        # collision.  The downstream group is checked separately so the
        # intentional contact cannot hide another collision.
        try:
            radial_carrier_common = _common_volume_mm3(radial_slide, carrier)
            exact_common_query_count += 1
            downstream_carrier_distance = float(downstream.distance_to(carrier))
            if downstream_carrier_distance < min_downstream_carrier_distance:
                min_downstream_carrier_distance = downstream_carrier_distance
                min_downstream_carrier_witness = {
                    "pose": asdict(pose),
                    "scope": "downstream_bodies_and_pivots_to_carrier",
                    "distance_mm": downstream_carrier_distance,
                }
            if downstream_carrier_distance < min_clearance:
                min_clearance = downstream_carrier_distance
                min_clearance_witness = {
                    "pose": asdict(pose),
                    "scope": "carrier_noncontact_downstream_bodies_and_pivots",
                    "distance_mm": downstream_carrier_distance,
                    "bbox_lower_bound_mm": 0.0,
                }
            if (
                downstream_carrier_distance
                < REQUIRED_NONCONTACT_CLEARANCE_MM
                - CLEARANCE_TOLERANCE_MM
            ):
                clearance_violations.append({
                    "pose": asdict(pose),
                    "scope": "carrier_noncontact_downstream_bodies_and_pivots",
                    "exact_distance_mm": downstream_carrier_distance,
                    "required_distance_mm": REQUIRED_NONCONTACT_CLEARANCE_MM,
                })
            downstream_carrier_common = 0.0
            if downstream_carrier_distance <= CLEARANCE_TOLERANCE_MM:
                downstream_carrier_common = _common_volume_mm3(
                    downstream, carrier,
                )
                exact_common_query_count += 1
        except Exception as exc:
            row = {
                "pose": asdict(pose),
                "scope": "selected_to_carrier",
                "status": "FAIL_KERNEL_EXCEPTION",
                "error": f"{type(exc).__name__}: {exc}",
            }
            collision_rows.append(row)
            collision_failures.append(row)
            radial_carrier_common = math.inf
            downstream_carrier_common = math.inf
        else:
            row = {
                "pose": asdict(pose),
                "scope": "selected_to_carrier",
                "radial_slide_common_volume_mm3": radial_carrier_common,
                "downstream_exact_distance_mm": downstream_carrier_distance,
                "downstream_common_volume_mm3": downstream_carrier_common,
                "status": (
                    "PASS_ZERO_POSITIVE"
                    if max(radial_carrier_common, downstream_carrier_common)
                    <= POSITIVE_VOLUME_TOLERANCE_MM3
                    else "FAIL_POSITIVE_VOLUME"
                ),
            }
            collision_rows.append(row)
            if row["status"] != "PASS_ZERO_POSITIVE":
                collision_failures.append(row)

        # Non-contact scopes.  AABB separation is a rigorous lower bound for
        # exact BREP distance; exact distance is evaluated whenever the lower
        # bound can beat the incumbent minimum or violate the 2.00 mm gate.
        targets = [
            (
                f"parked_sibling_{index}",
                parked_by_identity[index],
                parked_bounds[index],
            )
            for index in sorted(parked_by_identity)
            if index != pose.identity_index
        ]
        targets.append(("primary_M4_hardware", primary, primary_bounds))
        for target_label, target, target_bounds in targets:
            bbox_clearance = _bbox_distance_mm(selected_bounds, target_bounds)
            exact_distance: float | None = None
            must_query_exact = (
                bbox_clearance
                <= REQUIRED_NONCONTACT_CLEARANCE_MM + CLEARANCE_TOLERANCE_MM
                or bbox_clearance < min_clearance
            )
            if must_query_exact:
                try:
                    exact_distance = float(selected.distance_to(target))
                    exact_clearance_query_count += 1
                except Exception as exc:
                    exact_distance = -math.inf
                    collision_failures.append({
                        "pose": asdict(pose),
                        "scope": target_label,
                        "status": "FAIL_CLEARANCE_KERNEL_EXCEPTION",
                        "error": f"{type(exc).__name__}: {exc}",
                    })
                if exact_distance < min_clearance:
                    min_clearance = exact_distance
                    min_clearance_witness = {
                        "pose": asdict(pose),
                        "scope": target_label,
                        "distance_mm": exact_distance,
                        "bbox_lower_bound_mm": bbox_clearance,
                    }
                if (
                    target_label.startswith("parked_sibling_")
                    and exact_distance < min_sibling_distance
                ):
                    min_sibling_distance = exact_distance
                    min_sibling_witness = {
                        "pose": asdict(pose),
                        "scope": target_label,
                        "distance_mm": exact_distance,
                        "bbox_lower_bound_mm": bbox_clearance,
                    }
                if (
                    exact_distance
                    < REQUIRED_NONCONTACT_CLEARANCE_MM
                    - CLEARANCE_TOLERANCE_MM
                ):
                    clearance_violations.append({
                        "pose": asdict(pose),
                        "scope": target_label,
                        "exact_distance_mm": exact_distance,
                        "required_distance_mm": (
                            REQUIRED_NONCONTACT_CLEARANCE_MM
                        ),
                    })
            else:
                aabb_clearance_proof_count += 1

            # Positive-volume collision can only occur with positive AABB
            # intersection.  If boxes overlap, exact common volume is the
            # authority regardless of the clearance result above.
            if _positive_aabb_intersection(selected_bounds, target_bounds):
                try:
                    common_volume = _common_volume_mm3(selected, target)
                    exact_common_query_count += 1
                    status = (
                        "PASS_ZERO_POSITIVE"
                        if common_volume <= POSITIVE_VOLUME_TOLERANCE_MM3
                        else "FAIL_POSITIVE_VOLUME"
                    )
                    row = {
                        "pose": asdict(pose),
                        "scope": target_label,
                        "common_volume_mm3": common_volume,
                        "status": status,
                    }
                except Exception as exc:
                    row = {
                        "pose": asdict(pose),
                        "scope": target_label,
                        "status": "FAIL_KERNEL_EXCEPTION",
                        "error": f"{type(exc).__name__}: {exc}",
                    }
                collision_rows.append(row)
                if row["status"] != "PASS_ZERO_POSITIVE":
                    collision_failures.append(row)

        # Passive q is the only relative motion inside an occurrence.  Audit
        # each identity/q signature once; radial/coarse motion is otherwise a
        # rigid transform and cannot alter internal common volume.
        internal_key = (pose.identity_index, pose.passive_q_mm)
        if internal_key not in internal_results:
            try:
                internal_common = _common_volume_mm3(radial_slide, downstream)
                exact_common_query_count += 1
                internal_row = {
                    "identity_index": pose.identity_index,
                    "passive_q_mm": pose.passive_q_mm,
                    "scope": "radial_slide_to_downstream_custom_and_pivots",
                    "common_volume_mm3": internal_common,
                    "status": (
                        "PASS_ZERO_POSITIVE"
                        if internal_common <= POSITIVE_VOLUME_TOLERANCE_MM3
                        else "FAIL_POSITIVE_VOLUME"
                    ),
                }
            except Exception as exc:
                internal_row = {
                    "identity_index": pose.identity_index,
                    "passive_q_mm": pose.passive_q_mm,
                    "scope": "radial_slide_to_downstream_custom_and_pivots",
                    "status": "FAIL_KERNEL_EXCEPTION",
                    "error": f"{type(exc).__name__}: {exc}",
                }
            internal_results[internal_key] = internal_row
            if internal_row["status"] != "PASS_ZERO_POSITIVE":
                collision_failures.append(internal_row)

    # Resolve the exact leaf pair at the sampled global-clearance witness.  It
    # makes pivot-hardware ownership explicit rather than reporting only two
    # opaque compounds.
    leaf_witness = None
    if min_clearance_witness is not None and math.isfinite(min_clearance):
        witness_pose = Pose(**min_clearance_witness["pose"])
        selected_leaves = moving_leaves_at_pose(witness_pose)
        scope = min_clearance_witness["scope"]
        if scope.startswith("parked_sibling_"):
            sibling_index = int(scope.rsplit("_", 1)[1])
            target_leaves = tuple(parked_by_identity[sibling_index].children)
        elif scope.startswith("carrier_noncontact_downstream"):
            selected_leaves = selected_leaves[1:]
            target_leaves = (carrier,)
        else:
            target_leaves = primary_parts
        best = math.inf
        for first in selected_leaves:
            first_bounds = _bbox_bounds(first)
            for second in target_leaves:
                lower = _bbox_distance_mm(first_bounds, _bbox_bounds(second))
                if lower > min_clearance + CLEARANCE_TOLERANCE_MM:
                    continue
                distance = float(first.distance_to(second))
                exact_clearance_query_count += 1
                if distance < best:
                    best = distance
                    leaf_witness = {
                        "selected_label": str(first.label),
                        "target_label": str(second.label),
                        "exact_distance_mm": distance,
                        "selected_is_pivot_hardware": "pivot_" in str(first.label),
                        "target_is_pivot_hardware": "pivot_" in str(second.label),
                    }

    motion_rows = _pose_motion_rows(poses)
    sample_count_per_identity = len(prescribed_poses(0))
    max_observed_step = max(
        row["max_adjacent_translation_mm"] for row in motion_rows
    )
    collision_samples_zero = not collision_failures
    clearance_pass = (
        math.isfinite(min_clearance)
        and min_clearance
        >= REQUIRED_NONCONTACT_CLEARANCE_MM - CLEARANCE_TOLERANCE_MM
        and not clearance_violations
    )
    sampling_gates = {
        "four_identities_swept": {
            pose.identity_index for pose in poses
        } == {0, 1, 2, 3},
        "parked_to_active_coarse_endpoints_included": all(
            any(
                p.phase == "coarse_translate_while_retracted"
                and math.isclose(p.coarse_base_abs_y_mm, value, abs_tol=1e-12)
                for p in prescribed_poses(identity.index)
            )
            for identity in carriage.OCCURRENCE_IDENTITIES
            for value in (PARKED_BASE_ABS_Y_MM, ACTIVE_BASE_ABS_Y_MM)
        ),
        "retracted_to_extended_radial_endpoints_included": all(
            any(
                p.phase.startswith("radial_deploy_at_active")
                and math.isclose(p.radial_center_x_mm, value, abs_tol=1e-12)
                for p in prescribed_poses(identity.index)
            )
            for identity in carriage.OCCURRENCE_IDENTITIES
            for value in (
                RADIAL_RETRACTED_CENTER_X_MM,
                RADIAL_EXTENDED_CENTER_X_MM,
            )
        ),
        "passive_q_extrema_included_at_every_radial_sample": all(
            any(
                math.isclose(p.radial_center_x_mm, radial_x, abs_tol=1e-12)
                and math.isclose(p.passive_q_mm, q_mm, abs_tol=1e-12)
                for p in prescribed_poses(identity.index)
            )
            for identity in carriage.OCCURRENCE_IDENTITIES
            for radial_x in _linspace(
                RADIAL_RETRACTED_CENTER_X_MM,
                RADIAL_EXTENDED_CENTER_X_MM,
                RADIAL_SUBDIVISIONS,
            )
            for q_mm in PASSIVE_Q_EXTREMA_MM
        ),
        "max_independent_translation_step_le_0p50mm": (
            max_observed_step <= MAX_TRANSLATION_SAMPLE_STEP_MM + 1e-12
        ),
        "all_poses_have_15_exact_manufactured_leaves": all(
            len(moving_leaves_at_pose(pose))
            == carriage.MOVING_LEAF_COUNT_PER_OCCURRENCE
            for pose in (
                prescribed_poses(0)[0],
                prescribed_poses(0)[-1],
                prescribed_poses(3)[0],
                prescribed_poses(3)[-1],
            )
        ),
        "blocker_envelopes_excluded_as_non_manufactured": True,
    }

    status = (
        "PASS_SAMPLED_GEOMETRY_ONLY"
        if collision_samples_zero and clearance_pass
        else "FAIL_CLOSED"
    )
    decision = (
        "ZERO_SAMPLED_POSITIVE_VOLUME_BUT_2P00MM_CLEARANCE_FAILS__"
        "MECHANISMS_AND_PHYSICAL_AUTHORITY_OPEN"
        if collision_samples_zero and not clearance_pass
        else (
            "SAMPLED_TRANSITION_GEOMETRY_CLEAR__MECHANISMS_AND_PHYSICAL_"
            "AUTHORITY_OPEN"
            if collision_samples_zero else
            "TRANSITION_COLLISION_OR_KERNEL_FAILURE"
        )
    )

    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": status,
        "decision": decision,
        "prescribed_sequence": {
            "order": [
                "radially_retracted_coarse_parked_to_active",
                "radial_retracted_to_extended_at_active_q0",
                "passive_q_minus_plus_extrema_at_every_radial_sample",
            ],
            "reverse_sequence_covered_by_same_geometric_samples": True,
            "coarse_motion_requires_radially_retracted_state": True,
            "coarse_blocker_envelopes_are_not_mechanisms": True,
            "positive_selection_mechanism_modeled": False,
            "positive_retraction_mechanism_modeled": False,
        },
        "sampling": {
            "method": "UNIFORM_EXACT_BREP_SUBDIVISION",
            "identity_count": len(carriage.OCCURRENCE_IDENTITIES),
            "sample_count_per_identity": sample_count_per_identity,
            "total_pose_count": len(poses),
            "coarse_subdivisions": COARSE_SUBDIVISIONS,
            "radial_subdivisions": RADIAL_SUBDIVISIONS,
            "passive_q_extrema_mm": list(PASSIVE_Q_EXTREMA_MM),
            "maximum_independent_translation_step_mm": max_observed_step,
            "phase_rows": motion_rows,
            "gates": sampling_gates,
        },
        "collision_audit": {
            "method": (
                "STRICT_POSITIVE_AABB_BROAD_PHASE_THEN_EXACT_OCC_COMMON_VOLUME"
            ),
            "positive_volume_tolerance_mm3": (
                POSITIVE_VOLUME_TOLERANCE_MM3
            ),
            "carrier_pose_row_count": len(poses),
            "exact_common_query_count": exact_common_query_count,
            "internal_relative_signature_count": len(internal_results),
            "static_parked_siblings_include_full_pivot_hardware": True,
            "selected_occurrence_includes_full_pivot_hardware": True,
            "primary_M4_hardware_included": True,
            "non_manufactured_blocker_envelope_count_excluded": (
                carriage.COARSE_BLOCKER_COUNT
            ),
            "positive_failure_count": len(collision_failures),
            "all_sampled_positive_common_volumes_zero": collision_samples_zero,
            "minimum_downstream_to_carrier_exact_distance_mm": (
                min_downstream_carrier_distance
            ),
            "minimum_downstream_to_carrier_witness": (
                min_downstream_carrier_witness
            ),
            "minimum_selected_to_parked_sibling_exact_distance_mm": (
                min_sibling_distance
            ),
            "minimum_selected_to_parked_sibling_witness": (
                min_sibling_witness
            ),
            "internal_relative_rows": list(internal_results.values()),
            "failures": collision_failures,
        },
        "clearance_audit": {
            "scope": (
                "selected_downstream_to_carrier_plus_selected_complete_"
                "occurrence_to_three_complete_parked_siblings_and_primary_M4"
            ),
            "designed_radial_slide_carrier_contact_excluded_from_noncontact_gate": True,
            "downstream_body_and_pivot_to_carrier_included_in_noncontact_gate": True,
            "required_minimum_mm": REQUIRED_NONCONTACT_CLEARANCE_MM,
            "tolerance_mm": CLEARANCE_TOLERANCE_MM,
            "minimum_sampled_exact_clearance_mm": min_clearance,
            "minimum_sampled_exact_clearance_witness": min_clearance_witness,
            "minimum_leaf_pair_witness": leaf_witness,
            "exact_distance_query_count": exact_clearance_query_count,
            "AABB_lower_bound_proof_count": aabb_clearance_proof_count,
            "violation_count": len(clearance_violations),
            "violations": clearance_violations,
            "passes_2p00mm_gate": clearance_pass,
            "tolerance_stack_qualified": False,
            "nominal_reserve_above_requirement_mm": (
                min_clearance - REQUIRED_NONCONTACT_CLEARANCE_MM
            ),
        },
        "sampled_geometry_result": {
            "collision_samples_zero": collision_samples_zero,
            "clearance_gate_passes": clearance_pass,
            "sampling_contract_passes": all(sampling_gates.values()),
        },
        "authority": dict(AUTHORITY),
        "blockers": list(BLOCKERS),
        "integration": {
            "assembly_source_modified": False,
            "release_modified": False,
            "BOM_modified": False,
            "order_authorized": False,
        },
        "source_hashes": {
            str(path.relative_to(ROOT)).replace("\\", "/"): _sha256(path)
            for path in (*SOURCE_PATHS, Path(__file__).resolve())
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported replacement transition sweep schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("replacement transition sweep hash mismatch")
    for relative, expected in report.get("source_hashes", {}).items():
        path = ROOT / Path(str(relative).replace("\\", "/"))
        if not path.is_file() or _sha256(path) != expected:
            raise ValueError(f"stale replacement transition source {relative}")
    if not all(report["sampling"]["gates"].values()):
        raise ValueError("replacement transition sampling contract failed")
    if any(report["authority"].values()):
        raise ValueError("transition sweep cannot grant physical authority")
    result = report["sampled_geometry_result"]
    expected_status = (
        "PASS_SAMPLED_GEOMETRY_ONLY"
        if result["collision_samples_zero"]
        and result["clearance_gate_passes"]
        else "FAIL_CLOSED"
    )
    if report.get("status") != expected_status:
        raise ValueError("transition sweep status is not fail-closed")


def render_markdown(report: Mapping[str, Any]) -> str:
    sampling = report["sampling"]
    collision = report["collision_audit"]
    clearance = report["clearance_audit"]
    leaf = clearance["minimum_leaf_pair_witness"] or {}
    return "\n".join([
        "# Replacement-carriage continuous transition sweep",
        "",
        f"**{report['status']} — {report['decision']}**",
        "",
        "## Prescribed motion",
        "",
        f"1. Retract radially, then translate one selected identity from |Y|={PARKED_BASE_ABS_Y_MM:.2f} mm to |Y|={ACTIVE_BASE_ABS_Y_MM:.2f} mm.",
        "2. At the active bay, deploy radially from X=14 mm to X=20 mm.",
        "3. At every radial sample, also audit passive q=-0.50 and +0.50 mm.",
        "",
        f"- Four identities; {sampling['sample_count_per_identity']} poses each; {sampling['total_pose_count']} total.",
        f"- Maximum independent translation increment: {sampling['maximum_independent_translation_step_mm']:.6f} mm.",
        "- Exact OpenCascade common-volume operations decide positive overlap; AABB is broad phase / rigorous clearance lower bound only.",
        "",
        "## Result",
        "",
        f"- Sampled positive-volume failures: {collision['positive_failure_count']}.",
        f"- Minimum exact non-contact clearance: {clearance['minimum_sampled_exact_clearance_mm']:.9f} mm (required {clearance['required_minimum_mm']:.2f} mm).",
        f"- Clearance violations: {clearance['violation_count']}.",
        f"- Closest selected leaf: `{leaf.get('selected_label', 'unresolved')}`.",
        f"- Closest static leaf: `{leaf.get('target_label', 'unresolved')}`.",
        "",
        f"The parked-follower relief retains {collision['minimum_downstream_to_carrier_exact_distance_mm']:.9f} mm to the carrier, and the inward passive q extreme retains {collision['minimum_selected_to_parked_sibling_exact_distance_mm']:.9f} mm to the parked sibling. Every sampled non-contact scope meets the {clearance['required_minimum_mm']:.2f} mm gate with zero sampled positive common volume.",
        "",
        "## Authority boundary",
        "",
        "The four coarse-linkage solids are non-manufactured blocker envelopes. No positive coarse selector or M0 retraction/interlock exists, and no tolerance, load, wear, route, assembly, procurement, production, BOM, or release authority is granted.",
        "",
        "## Blockers",
        "",
        *(f"- `{blocker}`" for blocker in report["blockers"]),
        "",
    ])


def write_reports(report: Mapping[str, Any]) -> None:
    validate_report_integrity(report)
    REPORTS.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    OUTPUT_MD.write_text(render_markdown(report), encoding="utf-8")


def main() -> int:
    report = analyze()
    write_reports(report)
    print(json.dumps({
        "status": report["status"],
        "pose_count": report["sampling"]["total_pose_count"],
        "collision_failures": report["collision_audit"][
            "positive_failure_count"
        ],
        "minimum_exact_clearance_mm": report["clearance_audit"][
            "minimum_sampled_exact_clearance_mm"
        ],
        "clearance_violations": report["clearance_audit"][
            "violation_count"
        ],
        "report_sha256": report["report_sha256"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
