"""Isolated CAD prototype for a split active-tooth winding shoe.

This file deliberately does *not* add the shoe to ``assembly.py``.  It is a
review model for the smallest machine-fixed concept proposed by
``flyer_slot_guide_feasibility.py`` and is paired with the fail-closed audit in
``sim/active_tooth_shoe_route.py``.

Machine frame (the same frame used by ``assembly.py``):

* +Z is the M0/flyer axis, positive toward the carriage.
* +Y is the stator axial direction (up).
* +X is tangential to the presented tooth.
* the winding/lay radial datum is ``Z = 2.000 mm``.

The two dielectric inserts are mirror parts.  Each uses a <=0.10 mm blade in
one slot flanking the active tooth, a 6.516 mm projected working span, and
front/rear circular horn faces whose physical radius is 2.75 mm.  The
maximum-wire centreline therefore runs at exactly 3.00 mm radius.  Four M3
clearance axes terminate at Datum A, the machine-fixed plane Z=8.0 mm.

The blade centre curve is the midpoint of the common exact slot corridor over
the complete nine-depth M0 range.  Where that common corridor is empty the
same midpoint rule is retained; the resulting interference is intentional
evidence, not an integration-ready design.  The companion audit owns the
release decision.
"""

from __future__ import annotations

from functools import lru_cache
import math
from pathlib import Path

import numpy as np
from build123d import (
    Align,
    Box,
    Compound,
    Cylinder,
    Part,
    Plane,
    Pos,
    Rot,
)

from params import DEFAULT_STATOR
import coil_growth
import stator_model


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

CTR = (Align.CENTER, Align.CENTER, Align.CENTER)

# Controlled prototype dimensions (mm).
WIRE_LAY_DATUM_Z_MM = 2.0
MOUNT_DATUM_A_Z_MM = 8.0
MOUNT_HOLE_DIAMETER_MM = 3.4
MOUNT_HOLE_Y_MM = 13.0
MOUNT_PAD_SIZE_MM = (5.0, 5.0, 5.0)

MAXIMUM_BLADE_THICKNESS_MM = 0.25
BLADE_THICKNESS_MM = 0.10
LINER_RECEIVING_MAX_MM = 0.140
HORN_SURFACE_RADIUS_MM = 2.75
MAX_WIRE_RADIUS_MM = 0.25
HORN_CENTERLINE_RADIUS_MM = HORN_SURFACE_RADIUS_MM + MAX_WIRE_RADIUS_MM
HORN_AXIAL_CENTER_OFFSET_MM = 0.25
MINIMUM_AXIAL_PROJECTION_MM = 3.0

_JOB = coil_growth.analyze_job(DEFAULT_STATOR)
RADIAL_WORKING_START_MM = float(
    _JOB["bundle"]["radial_winding_start_mm"]
)
RADIAL_WORKING_END_MM = float(
    _JOB["bundle"]["radial_winding_end_mm"]
)
RADIAL_WORKING_SPAN_MM = (
    RADIAL_WORKING_END_MM - RADIAL_WORKING_START_MM
)
DEPTH_RADII_MM = tuple(
    float(value)
    for value in np.linspace(
        RADIAL_WORKING_START_MM, RADIAL_WORKING_END_MM, 9
    )
)


def _tooth_boundaries(radius_mm: float) -> tuple[float, float]:
    """Exact +Y slot boundaries for stator_model's tooth0/tooth1 slot.

    The first value is tooth0's positive wall and the second is tooth1's
    negative wall, expressed in stator-local XY.  The neck/shoe Boolean uses
    the more intrusive boundary in the shoe band, matching ``stator_model``.
    """

    spec = DEFAULT_STATOR
    outer = float(spec.od) / 2.0
    if not 0.0 < radius_mm <= outer + 1e-9:
        raise ValueError("radius must lie in the stator radial envelope")
    pitch = 2.0 * math.pi / int(spec.slots)
    shoe_half_angle = 0.36 * pitch
    half_neck = max(2.5, float(spec.od) * 0.07) / 2.0
    shoe_inner = outer - max(1.6, float(spec.od) * 0.045)

    active = half_neck
    neighbor = (
        -half_neck + math.sin(pitch) * float(radius_mm)
    ) / math.cos(pitch)
    if radius_mm >= shoe_inner:
        active = max(active, radius_mm * math.tan(shoe_half_angle))
        neighbor = min(
            neighbor,
            radius_mm * math.tan(pitch - shoe_half_angle),
        )
    return float(active), float(neighbor)


def common_corridor_at_world_z(world_z_mm: float) -> dict[str, float | bool]:
    """Common blade-centre interval at one fixed machine-Z station.

    At the shallowest lay depth, ``world_z_mm`` corresponds to a known local
    radius.  M0 then translates the stator through the complete winding span,
    so the same fixed blade station must fit every local radius up to OD.
    Liner and half-blade offsets are applied to both exact steel boundaries.
    """

    shallow_axis_z = WIRE_LAY_DATUM_Z_MM + RADIAL_WORKING_START_MM
    shallow_radius = shallow_axis_z - float(world_z_mm)
    outer = float(DEFAULT_STATOR.od) / 2.0
    relative_radii = [
        shallow_radius + depth - RADIAL_WORKING_START_MM
        for depth in DEPTH_RADII_MM
    ]
    in_stack = [radius for radius in relative_radii if radius <= outer + 1e-9]
    if not in_stack:
        return {
            "world_z_mm": float(world_z_mm),
            "shallow_radius_mm": float(shallow_radius),
            "lower_mm": -math.inf,
            "upper_mm": math.inf,
            "margin_mm": math.inf,
            "center_mm": 0.0,
            "feasible": True,
        }
    allowance = LINER_RECEIVING_MAX_MM + BLADE_THICKNESS_MM / 2.0
    lower = max(_tooth_boundaries(radius)[0] for radius in in_stack)
    upper = min(_tooth_boundaries(radius)[1] for radius in in_stack)
    lower += allowance
    upper -= allowance
    return {
        "world_z_mm": float(world_z_mm),
        "shallow_radius_mm": float(shallow_radius),
        "lower_mm": float(lower),
        "upper_mm": float(upper),
        "margin_mm": float(upper - lower),
        "center_mm": float((lower + upper) / 2.0),
        "feasible": bool(upper >= lower),
    }


@lru_cache(maxsize=8)
def blade_center_samples(count: int = 33) -> tuple[tuple[float, float], ...]:
    """Return (+slot) machine-frame ``(X,Z)`` centre samples."""

    if count < 2:
        raise ValueError("blade curve needs at least two samples")
    z_outer = WIRE_LAY_DATUM_Z_MM - RADIAL_WORKING_SPAN_MM
    rows = [
        common_corridor_at_world_z(float(z))
        for z in np.linspace(WIRE_LAY_DATUM_Z_MM, z_outer, count)
    ]
    # Stator-local +Y maps to machine -X.
    return tuple((-float(row["center_mm"]), float(row["world_z_mm"]))
                 for row in rows)


def corridor_summary(sample_count: int = 257) -> dict[str, object]:
    z_outer = WIRE_LAY_DATUM_Z_MM - RADIAL_WORKING_SPAN_MM
    rows = [
        common_corridor_at_world_z(float(z))
        for z in np.linspace(WIRE_LAY_DATUM_Z_MM, z_outer, sample_count)
    ]
    worst = min(rows, key=lambda row: float(row["margin_mm"]))
    failing = [row for row in rows if not bool(row["feasible"])]
    return {
        "sample_count": int(sample_count),
        "world_z_range_mm": [float(z_outer), WIRE_LAY_DATUM_Z_MM],
        "depth_radii_mm": list(DEPTH_RADII_MM),
        "allowance_each_wall_mm": (
            LINER_RECEIVING_MAX_MM + BLADE_THICKNESS_MM / 2.0
        ),
        "failing_station_count": len(failing),
        "minimum_common_corridor_margin_mm": float(worst["margin_mm"]),
        "minimum_margin_witness": worst,
        "status": "PASS" if not failing else "FAIL",
    }


def _segment_box(start_xz: tuple[float, float],
                 end_xz: tuple[float, float],
                 *, axial_height_mm: float, thickness_mm: float) -> Part:
    """One overlapping blade ribbon segment in the machine frame."""

    start = np.array((start_xz[0], 0.0, start_xz[1]), dtype=float)
    end = np.array((end_xz[0], 0.0, end_xz[1]), dtype=float)
    vector = end - start
    length = float(np.linalg.norm(vector))
    if length <= 1e-9:
        raise ValueError("zero length blade segment")
    u = vector / length
    world_y = np.array((0.0, 1.0, 0.0))
    normal = np.cross(u, world_y)
    normal /= np.linalg.norm(normal)
    plane = Plane(
        origin=tuple((start + end) / 2.0),
        x_dir=tuple(u),
        z_dir=tuple(normal),
    )
    local = Box(
        length + 0.04,
        axial_height_mm,
        thickness_mm,
        align=CTR,
    )
    return plane * local


def _nose_plane(side: int) -> Plane:
    origin, u, normal = nose_frame_vectors(side)
    return Plane(origin=tuple(origin), x_dir=tuple(u), z_dir=tuple(normal))


@lru_cache(maxsize=2)
def nose_frame_vectors(side: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return machine-frame origin, local +U, and local +N vectors."""

    if side not in (-1, 1):
        raise ValueError("side must be -1 or +1")
    samples = blade_center_samples(33)
    if side < 0:
        samples = tuple((-x, z) for x, z in samples)
    start = np.array((samples[0][0], 0.0, samples[0][1]), dtype=float)
    end = np.array((samples[1][0], 0.0, samples[1][1]), dtype=float)
    u = end - start
    u /= np.linalg.norm(u)
    normal = np.cross(u, np.array((0.0, 1.0, 0.0)))
    normal /= np.linalg.norm(normal)
    return start, u, normal


def horn_center_world(side: int, axial_sign: int) -> np.ndarray:
    """Physical horn-circle centre in the machine frame."""

    if axial_sign not in (-1, 1):
        raise ValueError("axial_sign must be -1 or +1")
    origin, u, _normal = nose_frame_vectors(side)
    return (
        origin
        + HORN_SURFACE_RADIUS_MM * u
        + axial_sign
        * (float(DEFAULT_STATOR.stack) / 2.0
           + HORN_AXIAL_CENTER_OFFSET_MM)
        * np.array((0.0, 1.0, 0.0))
    )


def horn_wire_center_point(side: int, axial_sign: int,
                           angle_rad: float) -> np.ndarray:
    """Maximum-wire centre on the exposed horn, 0 <= angle <= pi/2.

    Angle zero is the radial nose/mouth point.  Increasing angle follows the
    horn toward its axially outboard crown.
    """

    if not -1e-12 <= float(angle_rad) <= math.pi / 2.0 + 1e-12:
        raise ValueError("horn angle must be in [0, pi/2]")
    _origin, u, _normal = nose_frame_vectors(side)
    axial = np.array((0.0, 1.0, 0.0))
    center = horn_center_world(side, axial_sign)
    return (
        center
        - HORN_CENTERLINE_RADIUS_MM * math.cos(float(angle_rad)) * u
        + axial_sign
        * HORN_CENTERLINE_RADIUS_MM * math.sin(float(angle_rad)) * axial
    )


def dielectric_blade(side: int, label: str | None = None) -> Part:
    """Return one best-compromise blade and its front/rear horn faces."""

    if side not in (-1, 1):
        raise ValueError("side must be -1 or +1")
    samples = blade_center_samples(33)
    if side < 0:
        samples = tuple((-x, z) for x, z in samples)
    pieces = [
        _segment_box(
            one,
            two,
            axial_height_mm=float(DEFAULT_STATOR.stack),
            thickness_mm=BLADE_THICKNESS_MM,
        )
        for one, two in zip(samples, samples[1:])
    ]
    blade = pieces[0]
    for piece in pieces[1:]:
        blade += piece

    plane = _nose_plane(side)
    half_stack = float(DEFAULT_STATOR.stack) / 2.0
    for axial_sign in (-1, 1):
        horn = Cylinder(
            HORN_SURFACE_RADIUS_MM,
            BLADE_THICKNESS_MM,
            align=CTR,
        )
        horn = Pos(
            HORN_SURFACE_RADIUS_MM,
            axial_sign * (half_stack + HORN_AXIAL_CENTER_OFFSET_MM),
            0.0,
        ) * horn
        blade += plane * horn
    blade.label = label or f"dielectric_blade_{side:+d}"
    return blade


def _global_bar_y(x: float, y0: float, y1: float,
                  z0: float, z1: float, width_x: float = 2.4) -> Part:
    return Pos(x, (y0 + y1) / 2.0, (z0 + z1) / 2.0) * Box(
        width_x, abs(y1 - y0), abs(z1 - z0), align=CTR
    )


def carrier_half(side: int, label: str | None = None) -> Part:
    """Concept carrier ending at explicit machine-fixed Datum A.

    The carrier is intentionally a separate occurrence from the dielectric
    insert.  No material or clamp qualification is implied.
    """

    if side not in (-1, 1):
        raise ValueError("side must be -1 or +1")
    nose_x = blade_center_samples(33)[0][0]
    if side < 0:
        nose_x = -nose_x
    pad_x, pad_y, pad_z = MOUNT_PAD_SIZE_MM
    parts: list[Part] = []
    for axial_sign in (-1, 1):
        y = axial_sign * MOUNT_HOLE_Y_MM
        pad = Pos(
            nose_x,
            y,
            MOUNT_DATUM_A_Z_MM - pad_z / 2.0,
        ) * Box(pad_x, pad_y, pad_z, align=CTR)
        hole = Pos(nose_x, y, MOUNT_DATUM_A_Z_MM - pad_z / 2.0) * Cylinder(
            MOUNT_HOLE_DIAMETER_MM / 2.0,
            pad_z + 2.0,
            align=CTR,
        )
        pad -= hole
        arm = _global_bar_y(
            nose_x,
            axial_sign * (float(DEFAULT_STATOR.stack) / 2.0 + 1.5),
            axial_sign * (MOUNT_HOLE_Y_MM - pad_y / 2.0),
            1.75,
            MOUNT_DATUM_A_Z_MM - pad_z,
            width_x=2.4,
        )
        parts.extend((pad, arm))
    carrier = parts[0]
    for part in parts[1:]:
        carrier += part
    carrier.label = label or f"carrier_{side:+d}"
    return carrier


def _installed_stator_at_radius(radial_mm: float) -> Part:
    """Reference stator pose with ``radial_mm`` at the Z=2 lay datum."""

    axis_z = WIRE_LAY_DATUM_Z_MM + float(radial_mm)
    local = stator_model.stator(DEFAULT_STATOR, label="stator_reference")
    shape = Pos(0.0, 0.0, axis_z) * (
        Rot(0.0, 90.0, 0.0) * (Rot(-90.0, 0.0, 0.0) * local)
    )
    shape.label = "stator_reference_mid_depth"
    return shape


def shoe_parts() -> tuple[Part, ...]:
    return (
        dielectric_blade(1, "dielectric_blade_left_slot"),
        dielectric_blade(-1, "dielectric_blade_right_slot"),
        carrier_half(1, "carrier_left_slot"),
        carrier_half(-1, "carrier_right_slot"),
    )


def gen_step() -> Compound:
    """Return the isolated review assembly, never the production assembly."""

    mid_radius = (RADIAL_WORKING_START_MM + RADIAL_WORKING_END_MM) / 2.0
    stator = _installed_stator_at_radius(mid_radius)
    stator.label = "stator_reference_mid_depth"
    children = [stator, *shoe_parts()]
    result = Compound(children=children)
    result.label = "active_tooth_shoe_no_go_review"
    return result


if __name__ == "__main__":
    # The CAD skill launcher owns STEP export.  This print is useful for a
    # quick source-only diagnostic and does not write a generated artifact.
    print(corridor_summary())
