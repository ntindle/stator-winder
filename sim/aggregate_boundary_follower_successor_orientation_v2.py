"""Analytic V2 frame law for the isolated successor C1 guide.

This module does not generate CAD or reports.  It proves the orientation law
against the frozen 4,704-case placement/C1 evidence and documents the exact
build123d frame construction for a later source patch.

Authored guide frame at the R3 entry:

* local +X: guide centerline tangent;
* local +Y: contact-to-R3-center curvature normal;
* local +Z: +X cross +Y, normal to the authored guide plane.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
PLACEMENT_PATH = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_placement_trade.json"
)
C1_PATH = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_c1_rebound_sweep.json"
)
PLACEMENT_COLLISION_AUDIT_PATH = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_successor_prototype_placement_collision_audit.json"
)
PROTOTYPE_SOURCE_PATH = ROOT / "cad" / (
    "aggregate_boundary_follower_successor_prototype.py"
)

EXPECTED_CASE_COUNT = 4704
LOCAL_GUIDE_TANGENT = (1.0, 0.0, 0.0)
LOCAL_CONTACT_TO_CENTER_NORMAL = (0.0, 1.0, 0.0)
LOCAL_GUIDE_PLANE_NORMAL = (0.0, 0.0, 1.0)


Vector3 = tuple[float, float, float]
FrameColumns = tuple[Vector3, Vector3, Vector3]


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(float(a[index]) * float(b[index]) for index in range(3))


def _cross(a: Sequence[float], b: Sequence[float]) -> Vector3:
    ax, ay, az = map(float, a)
    bx, by, bz = map(float, b)
    return (ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx)


def _norm(value: Sequence[float]) -> float:
    return math.sqrt(_dot(value, value))


def _unit(value: Sequence[float]) -> Vector3:
    length = _norm(value)
    if length <= 1.0e-15:
        raise ValueError("cannot normalize a degenerate direction")
    return tuple(float(component) / length for component in value)  # type: ignore[return-value]


def _subtract(a: Sequence[float], b: Sequence[float]) -> Vector3:
    return tuple(float(a[index]) - float(b[index]) for index in range(3))  # type: ignore[return-value]


def _angle_between_deg(a: Sequence[float], b: Sequence[float]) -> float:
    one = _unit(a)
    two = _unit(b)
    return math.degrees(math.atan2(_norm(_cross(one, two)), _dot(one, two)))


def _apply_frame(frame: FrameColumns, local: Sequence[float]) -> Vector3:
    """Apply a column-basis frame to one local direction."""

    return tuple(
        sum(float(local[column]) * frame[column][row] for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def guide_frame_from_tangent_and_normal(
    requested_tangent: Sequence[float],
    contact_to_center_normal: Sequence[float],
) -> FrameColumns:
    """Return the exact right-handed guide frame as world basis columns.

    The C1 construction makes tangent and curvature normal perpendicular.
    Rebuilding Y from ``Z cross X`` removes floating-point non-orthogonality
    while preserving the requested curvature-normal hemisphere.
    """

    world_x = _unit(requested_tangent)
    normal_seed = _unit(contact_to_center_normal)
    world_z = _unit(_cross(world_x, normal_seed))
    world_y = _unit(_cross(world_z, world_x))
    if _dot(world_y, normal_seed) < 0.0:
        world_y = tuple(-value for value in world_y)  # type: ignore[assignment]
        world_z = tuple(-value for value in world_z)  # type: ignore[assignment]
    return world_x, world_y, world_z


def legacy_single_rot_local_x(
    yaw_deg: float, elevation_deg: float,
) -> Vector3:
    """Actual local +X from build123d ``Rot(0, -elevation, yaw)``.

    A single build123d/OCC Euler location applies the Z rotation before the Y
    rotation for this tuple, yielding ``Ry(-elevation) @ Rz(yaw)``.
    """

    yaw = math.radians(float(yaw_deg))
    elevation = math.radians(float(elevation_deg))
    return _unit((
        math.cos(elevation) * math.cos(yaw),
        math.sin(yaw),
        math.sin(elevation) * math.cos(yaw),
    ))


def split_yaw_pitch_local_x(
    yaw_deg: float, elevation_deg: float,
) -> Vector3:
    """Local +X from ``RotZ(yaw) * RotY(-elevation)``.

    In build123d syntax the tangent-only correction is::

        Rot(0, 0, yaw) * Rot(0, -elevation, 0) * guide

    The rightmost pitch acts first, followed by world-Z yaw.
    """

    yaw = math.radians(float(yaw_deg))
    elevation = math.radians(float(elevation_deg))
    return (
        math.cos(elevation) * math.cos(yaw),
        math.cos(elevation) * math.sin(yaw),
        math.sin(elevation),
    )


def build123d_patch_recommendation() -> str:
    """Return the source-level frame construction recommended for V2."""

    return """\
tangent = unit(case[\"required_guide_tangent\"])
normal = unit(case[\"required_curvature_normal_contact_to_center\"])
binormal = unit(cross(tangent, normal))
normal = unit(cross(binormal, tangent))
guide_frame = Plane(origin=required_center, x_dir=tangent, z_dir=binormal)
guide = guide_frame.location * c1_guide_local()

# Plane local axes are X=tangent, Y=binormal cross tangent=normal,
# Z=binormal.  This fixes tangent and roll simultaneously.  If only tangent
# is needed, Rot(0,0,yaw) * Rot(0,-elevation,0) is the exact split-Euler form.
"""


def _c1_case_map(c1: Mapping[str, Any]) -> dict[tuple[int, float], Mapping[str, Any]]:
    result: dict[tuple[int, float], Mapping[str, Any]] = {}
    for locus in c1.get("loci", []):
        locus_index = int(locus["locus_index"])
        for case in locus.get("diameter_cases", []):
            if case.get("status") != "PASS_ANALYTIC_C1_S_BIARC":
                continue
            key = (locus_index, float(case["wire_diameter_mm"]))
            if key in result:
                raise ValueError(f"duplicate C1 case {key}")
            result[key] = case
    return result


def analyze() -> dict[str, Any]:
    placement = _load(PLACEMENT_PATH)
    c1 = _load(C1_PATH)
    collision_audit = _load(PLACEMENT_COLLISION_AUDIT_PATH)
    cases = placement.get("case_comparisons", [])
    if len(cases) != EXPECTED_CASE_COUNT:
        raise ValueError("placement trade must contain exactly 4,704 cases")
    c1_cases = _c1_case_map(c1)
    if len(c1_cases) != EXPECTED_CASE_COUNT:
        raise ValueError("C1 report must contain exactly 4,704 constructed cases")

    authored = collision_audit.get("artifact_geometry", {}).get(
        "C1_construction", {}
    )
    if (
        authored.get("first_join_incoming_tangent") != list(LOCAL_GUIDE_TANGENT)
        or authored.get("first_join_outgoing_tangent")
        != list(LOCAL_GUIDE_TANGENT)
    ):
        raise ValueError("audit no longer binds authored local +X entry tangent")
    if collision_audit.get("input_hashes", {}).get(
        "prototype_source"
    ) != _sha256(PROTOTYPE_SOURCE_PATH):
        raise ValueError("placement/collision audit prototype source is stale")

    legacy_errors: list[float] = []
    split_errors: list[float] = []
    split_vector_residuals: list[float] = []
    frame_tangent_residuals: list[float] = []
    frame_normal_residuals: list[float] = []
    frame_orthogonality_residuals: list[float] = []
    frame_determinant_residuals: list[float] = []
    upstream_tangent_residuals: list[float] = []
    upstream_normal_residuals: list[float] = []
    identity_counts = {str(index): 0 for index in range(4)}

    for case in cases:
        identity_counts[str(int(case["identity"]["physical_id"]))] += 1
        tangent = _unit(case["required_guide_tangent"])
        normal = _unit(case["required_curvature_normal_contact_to_center"])
        angles = case["required_guide_tangent_angles"]
        yaw = float(angles["yaw_about_positive_Z_deg"])
        elevation = float(angles["elevation_from_XY_deg"])

        key = (int(case["locus_index"]), float(case["wire_diameter_mm"]))
        upstream = c1_cases[key]
        upstream_tangent = _unit(upstream["end_tangent"])
        upstream_normal = _unit(_subtract(
            upstream["follower_center_for_end_arc_mm"], upstream["end_mm"]
        ))
        upstream_tangent_residuals.append(_norm(_subtract(tangent, upstream_tangent)))
        upstream_normal_residuals.append(_norm(_subtract(normal, upstream_normal)))

        legacy = legacy_single_rot_local_x(yaw, elevation)
        split = _unit(split_yaw_pitch_local_x(yaw, elevation))
        legacy_errors.append(_angle_between_deg(legacy, tangent))
        split_errors.append(_angle_between_deg(split, tangent))
        split_vector_residuals.append(_norm(_subtract(split, tangent)))

        frame = guide_frame_from_tangent_and_normal(tangent, normal)
        realized_tangent = _apply_frame(frame, LOCAL_GUIDE_TANGENT)
        realized_normal = _apply_frame(
            frame, LOCAL_CONTACT_TO_CENTER_NORMAL
        )
        frame_tangent_residuals.append(
            _norm(_subtract(realized_tangent, tangent))
        )
        frame_normal_residuals.append(_norm(_subtract(realized_normal, normal)))
        x_axis, y_axis, z_axis = frame
        frame_orthogonality_residuals.append(max(
            abs(_dot(x_axis, y_axis)), abs(_dot(x_axis, z_axis)),
            abs(_dot(y_axis, z_axis)), abs(_norm(x_axis) - 1.0),
            abs(_norm(y_axis) - 1.0), abs(_norm(z_axis) - 1.0),
        ))
        determinant = _dot(x_axis, _cross(y_axis, z_axis))
        frame_determinant_residuals.append(abs(determinant - 1.0))

    tolerance_deg = 1.0e-9
    return {
        "case_count": len(cases),
        "identity_counts": identity_counts,
        "authored_local_frame": {
            "tangent": list(LOCAL_GUIDE_TANGENT),
            "contact_to_center_normal": list(
                LOCAL_CONTACT_TO_CENTER_NORMAL
            ),
            "guide_plane_normal": list(LOCAL_GUIDE_PLANE_NORMAL),
        },
        "input_sha256": {
            "prototype_source": _sha256(PROTOTYPE_SOURCE_PATH),
            "placement_trade": _sha256(PLACEMENT_PATH),
            "C1_rebound": _sha256(C1_PATH),
            "placement_collision_audit": _sha256(
                PLACEMENT_COLLISION_AUDIT_PATH
            ),
        },
        "upstream_C1_binding": {
            "matched_case_count": sum(
                residual <= 1.0e-12
                for residual in upstream_tangent_residuals
            ),
            "maximum_tangent_vector_residual": max(
                upstream_tangent_residuals
            ),
            "maximum_normal_vector_residual": max(upstream_normal_residuals),
        },
        "legacy_single_Rot": {
            "matched_case_count": sum(error <= tolerance_deg
                                      for error in legacy_errors),
            "minimum_error_deg": min(legacy_errors),
            "maximum_error_deg": max(legacy_errors),
            "mean_error_deg": sum(legacy_errors) / len(legacy_errors),
            "matrix_order": "Ry(-elevation) @ Rz(yaw)",
        },
        "split_yaw_pitch": {
            "matched_case_count": sum(error <= tolerance_deg
                                      for error in split_errors),
            "maximum_angle_residual_deg": max(split_errors),
            "maximum_vector_residual": max(split_vector_residuals),
            "matrix_order": "Rz(yaw) @ Ry(-elevation)",
            "build123d": (
                "Rot(0,0,yaw) * Rot(0,-elevation,0) * guide"
            ),
        },
        "full_tangent_normal_frame": {
            "matched_tangent_case_count": sum(
                value <= 1.0e-12 for value in frame_tangent_residuals
            ),
            "matched_normal_case_count": sum(
                value <= 1.0e-12 for value in frame_normal_residuals
            ),
            "maximum_tangent_vector_residual": max(
                frame_tangent_residuals
            ),
            "maximum_normal_vector_residual": max(frame_normal_residuals),
            "maximum_orthonormality_residual": max(
                frame_orthogonality_residuals
            ),
            "maximum_determinant_residual": max(
                frame_determinant_residuals
            ),
            "build123d": (
                "Plane(origin=center, x_dir=tangent, "
                "z_dir=cross(tangent, contact_to_center_normal)).location"
            ),
        },
        "patch_recommendation": build123d_patch_recommendation(),
    }


def main() -> int:
    result = analyze()
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
