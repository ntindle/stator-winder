"""Isolated review CAD for an OD-bounded R3 tooth-end former.

CAD brief:
- Model: two retained stator end-cap baskets plus wire witnesses; review only.
- Task: test a permanent dielectric former for the literal GOAL >=3 mm
  workpiece wire-centre bend on ``DEFAULT_STATOR``.
- Units/frame: millimetres; stator-local +X radial, +Y tangential, +Z axial;
  lamination mid-plane Z=0 and tooth 0 on +X.
- Functional geometry: one L-R-L bounded-curvature paddle per tooth on both
  axial faces, narrow tooth-face straps, and a hub registration ring.
- Manufacturing concept: two one-piece unfilled PEEK/PPS end caps, polished
  guide faces, bonded to the lamination end faces; no material enters the
  steel slot throat beyond the existing 0.127 mm liner allowance.
- Output: ``r3_tooth_end_former.step`` beside this source.
- Validation: isolated study only.  Do not integrate unless its companion
  simulation closes packing, neighbours, rotor envelope, and every raw pose.

The L-R-L convention is shared with ``sim/r3_bend_scope_feasibility.py``.
The base curve is the q=0 *wire centreline*.  A physical contact boundary is
one wire radius inward from it.  Four parallel wire-centre offsets reproduce
the exact four-layer default packing cross-section; the innermost signed arc
radius is exactly 3 mm at layer three.

This file intentionally remains outside ``assembly.py``.  Its current
two-colour 0/12 mm neighbour lane is a bounded review candidate, not released
motor hardware.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from build123d import (
    Align,
    Box,
    BuildLine,
    BuildSketch,
    Circle,
    Compound,
    Cylinder,
    Plane,
    Polygon,
    Polyline,
    Pos,
    Rot,
    Transition,
    extrude,
    sweep,
)

from params import DEFAULT_STATOR
import stator_model


# Exact default-job receiving geometry.
WIRE_DIAMETER_MM = 0.22352
WIRE_RADIUS_MM = WIRE_DIAMETER_MM / 2.0
LINER_THICKNESS_MM = 0.127
HALF_NECK_MM = max(2.5, DEFAULT_STATOR.od * 0.07) / 2.0
SLOT_PITCH_HALF_RAD = math.pi / DEFAULT_STATOR.slots
PACKING_Q_STEP_MM = WIRE_DIAMETER_MM * math.cos(SLOT_PITCH_HALF_RAD)
PACKING_Q_MAX_MM = 3.0 * PACKING_Q_STEP_MM
BASE_WIRE_RADIUS_MM = 3.0 + PACKING_Q_MAX_MM
FIRST_WIRE_HALF_SPAN_MM = (
    HALF_NECK_MM + LINER_THICKNESS_MM + WIRE_RADIUS_MM
)
BASE_FIRST_ARC_RAD = math.acos(
    (1.0 + FIRST_WIRE_HALF_SPAN_MM / BASE_WIRE_RADIUS_MM) / 2.0
)

# Retained part envelope.  The radial span is the exact 50-turn schedule plus
# a small inward/outward surface margin, and remains below the tooth shoe.
ACCESS_RADIUS_MM = (
    2.0 * HALF_NECK_MM + WIRE_DIAMETER_MM + 2.0 * LINER_THICKNESS_MM
) / (2.0 * math.sin(SLOT_PITCH_HALF_RAD))
RADIAL_SURFACE_MIN_MM = ACCESS_RADIUS_MM - 0.10
RADIAL_SURFACE_MAX_MM = 20.605
TOOTH_FACE_STRAP_WIDTH_MM = 2.40
FACE_SHEET_THICKNESS_MM = 0.50
HUB_RING_INNER_RADIUS_MM = 7.20
HUB_RING_OUTER_RADIUS_MM = 11.70
NEIGHBOUR_LANE_MM = 12.0


@dataclass(frozen=True)
class PackingRow:
    turn_index: int
    radial_index: int
    tangential_layer: int
    radial_mm: float
    q_mm: float


def packing_rows() -> tuple[PackingRow, ...]:
    """Return the exact 50-row four-layer default-stator witness."""

    rows: list[PackingRow] = []
    beta = SLOT_PITCH_HALF_RAD
    for radial_index in range(4, 28):
        layer_count = int(math.floor(
            radial_index * math.tan(beta) + 0.5
        ))
        u = ACCESS_RADIUS_MM + radial_index * WIRE_DIAMETER_MM
        h = radial_index * WIRE_DIAMETER_MM * math.tan(beta)
        for layer in range(layer_count):
            v = -h + layer * WIRE_DIAMETER_MM
            x = u * math.cos(beta) - v * math.sin(beta)
            rows.append(PackingRow(
                turn_index=len(rows),
                radial_index=radial_index,
                tangential_layer=layer,
                radial_mm=float(x),
                q_mm=float(layer * PACKING_Q_STEP_MM),
            ))
    if len(rows) != DEFAULT_STATOR.turns:
        raise RuntimeError(f"expected 50 packing rows, got {len(rows)}")
    return tuple(rows)


def _advance_arc(state: np.ndarray, sign: int, angle: float,
                 radius: float = BASE_WIRE_RADIUS_MM) -> np.ndarray:
    y, z, heading = map(float, state)
    end_heading = heading + int(sign) * float(angle)
    return np.array((
        y + radius / sign * (
            math.sin(end_heading) - math.sin(heading)
        ),
        z + radius / sign * (
            -math.cos(end_heading) + math.cos(heading)
        ),
        end_heading,
    ))


def _sample_base_wire_yz(axial_sign: int, lane_mm: float = 0.0,
                         step_deg: float = 2.0
                         ) -> tuple[np.ndarray, np.ndarray]:
    """Return q=0 wire-centre points and continuous left normals."""

    if axial_sign not in (-1, 1):
        raise ValueError("axial_sign must be -1 or +1")
    if lane_mm < 0.0 or step_deg <= 0.0:
        raise ValueError("lane must be non-negative and step positive")
    face = DEFAULT_STATOR.stack / 2.0
    state = np.array((
        -FIRST_WIRE_HALF_SPAN_MM,
        face + float(lane_mm),
        math.pi / 2.0,
    ))
    points: list[tuple[float, float]] = []
    normals: list[tuple[float, float]] = []
    arcs = (
        (+1, BASE_FIRST_ARC_RAD),
        (-1, 2.0 * BASE_FIRST_ARC_RAD + math.pi),
        (+1, BASE_FIRST_ARC_RAD),
    )
    for arc_index, (sign, angle) in enumerate(arcs):
        count = max(2, math.ceil(math.degrees(angle) / step_deg))
        values = np.linspace(0.0, angle, count + 1)
        if arc_index:
            values = values[1:]
        y, z, heading = map(float, state)
        for value in values:
            theta = heading + sign * float(value)
            points.append((
                y + BASE_WIRE_RADIUS_MM / sign * (
                    math.sin(theta) - math.sin(heading)
                ),
                z + BASE_WIRE_RADIUS_MM / sign * (
                    -math.cos(theta) + math.cos(heading)
                ),
            ))
            normals.append((-math.sin(theta), math.cos(theta)))
        state = _advance_arc(state, sign, angle)
    yz = np.asarray(points, dtype=float)
    normal = np.asarray(normals, dtype=float)
    if axial_sign < 0:
        yz[:, 1] *= -1.0
        normal[:, 1] *= -1.0
    return yz, normal


def wire_cap_points(row: PackingRow, axial_sign: int,
                    lane_mm: float = 0.0,
                    step_deg: float = 2.0) -> np.ndarray:
    """One R3-bounded end-turn centreline at a retained cap."""

    yz, normals = _sample_base_wire_yz(
        axial_sign, lane_mm=lane_mm, step_deg=step_deg
    )
    yz = yz + row.q_mm * normals
    face = axial_sign * DEFAULT_STATOR.stack / 2.0
    lower = np.array((row.radial_mm, yz[0, 0], face))
    upper = np.array((row.radial_mm, yz[-1, 0], face))
    cap = np.column_stack((
        np.full(len(yz), row.radial_mm), yz,
    ))
    pieces = [lower]
    if np.linalg.norm(cap[0] - lower) > 1e-10:
        pieces.append(cap[0])
    pieces.extend(cap[1:])
    if np.linalg.norm(cap[-1] - upper) > 1e-10:
        pieces.append(upper)
    return np.asarray(pieces, dtype=float)


def contact_boundary_yz(axial_sign: int, lane_mm: float = 0.0,
                        step_deg: float = 2.0) -> np.ndarray:
    """Physical former contact perimeter, inward of the first wire."""

    wire, normals = _sample_base_wire_yz(
        axial_sign, lane_mm=lane_mm, step_deg=step_deg
    )
    surface = wire - WIRE_RADIUS_MM * normals
    face = axial_sign * DEFAULT_STATOR.stack / 2.0
    lower = np.array((surface[0, 0], face))
    upper = np.array((surface[-1, 0], face))
    boundary = [lower]
    if np.linalg.norm(surface[0] - lower) > 1e-10:
        boundary.append(surface[0])
    boundary.extend(surface[1:])
    if np.linalg.norm(surface[-1] - upper) > 1e-10:
        boundary.append(upper)
    return np.asarray(boundary, dtype=float)


def tooth_paddle(axial_sign: int, lane_mm: float = 0.0,
                 label: str = "r3_tooth_paddle"):
    """Closed radial extrusion under one bounded-curvature contact surface."""

    boundary = contact_boundary_yz(axial_sign, lane_mm=lane_mm)
    # Close across the tooth end face.  All sampled points are in (Y, Z).
    polygon_points = [tuple(map(float, point)) for point in boundary]
    with BuildSketch(Plane.YZ) as sketch:
        Polygon(*polygon_points, align=None)
    length = RADIAL_SURFACE_MAX_MM - RADIAL_SURFACE_MIN_MM
    body = extrude(sketch.sketch, amount=length)
    body = Pos(RADIAL_SURFACE_MIN_MM, 0.0, 0.0) * body
    body.label = label
    return body


def retention_face(axial_sign: int):
    """Hub ring and tooth-only straps for adhesive/anti-rotation retention."""

    face = axial_sign * (
        DEFAULT_STATOR.stack / 2.0 + FACE_SHEET_THICKNESS_MM / 2.0
    )
    ring = (
        Cylinder(
            HUB_RING_OUTER_RADIUS_MM,
            FACE_SHEET_THICKNESS_MM,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
        - Cylinder(
            HUB_RING_INNER_RADIUS_MM,
            FACE_SHEET_THICKNESS_MM + 1.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
    )
    ring = Pos(0.0, 0.0, face) * ring
    ring.label = f"hub_registration_ring_{axial_sign:+d}"
    children = [ring]
    start = HUB_RING_OUTER_RADIUS_MM - 0.20
    length = RADIAL_SURFACE_MIN_MM - start + 0.20
    for tooth in range(DEFAULT_STATOR.slots):
        strap = Box(
            length,
            TOOTH_FACE_STRAP_WIDTH_MM,
            FACE_SHEET_THICKNESS_MM,
            align=(Align.MIN, Align.CENTER, Align.CENTER),
        )
        strap = Pos(start, 0.0, face) * strap
        strap = Rot(0.0, 0.0, tooth * 360.0 / DEFAULT_STATOR.slots) * strap
        strap.label = f"tooth_{tooth:02d}_face_strap_{axial_sign:+d}"
        children.append(strap)
    result = Compound(children=children)
    result.label = f"retention_face_{axial_sign:+d}"
    return result


def former_parts(axial_sign: int) -> tuple:
    """All retained solids for one axial face in the bounded review."""

    parts = [retention_face(axial_sign)]
    for tooth in range(DEFAULT_STATOR.slots):
        lane = NEIGHBOUR_LANE_MM if tooth & 1 else 0.0
        paddle = tooth_paddle(
            axial_sign,
            lane_mm=lane,
            label=f"tooth_{tooth:02d}_r3_paddle_{axial_sign:+d}",
        )
        paddle = Rot(
            0.0, 0.0, tooth * 360.0 / DEFAULT_STATOR.slots
        ) * paddle
        paddle.label = f"tooth_{tooth:02d}_r3_paddle_{axial_sign:+d}"
        parts.append(paddle)
    return tuple(parts)


def _poly_tube(points: np.ndarray, radius_mm: float, label: str):
    clean = [tuple(map(float, point)) for point in np.asarray(points)]
    direction = tuple(clean[1][axis] - clean[0][axis] for axis in range(3))
    with BuildLine() as path:
        Polyline(*clean)
    with BuildSketch(Plane(origin=clean[0], z_dir=direction)) as profile:
        Circle(float(radius_mm))
    body = sweep(
        profile.sketch,
        path.line,
        transition=Transition.TRANSFORMED,
    )
    body.label = label
    return body


def wire_witnesses() -> tuple:
    """Inner/outer layer witnesses on both faces of tooth zero."""

    rows = packing_rows()
    selected = (rows[0], next(row for row in reversed(rows)
                              if row.tangential_layer == 3))
    result = []
    for axial_sign in (-1, 1):
        for row in selected:
            result.append(_poly_tube(
                wire_cap_points(row, axial_sign, lane_mm=0.0),
                WIRE_RADIUS_MM,
                f"wire_turn_{row.turn_index:02d}_layer_"
                f"{row.tangential_layer}_{axial_sign:+d}",
            ))
    return tuple(result)


def gen_step() -> Compound:
    stator = stator_model.stator(DEFAULT_STATOR, label="default_stator")
    children = [stator]
    children.extend(former_parts(+1))
    children.extend(former_parts(-1))
    children.extend(wire_witnesses())
    result = Compound(children=children)
    result.label = "r3_tooth_end_former_isolated_review"
    return result


if __name__ == "__main__":
    rows = packing_rows()
    print({
        "rows": len(rows),
        "base_wire_radius_mm": BASE_WIRE_RADIUS_MM,
        "minimum_layer3_radius_mm": (
            BASE_WIRE_RADIUS_MM - PACKING_Q_MAX_MM
        ),
        "radial_center_range_mm": [
            min(row.radial_mm for row in rows),
            max(row.radial_mm for row in rows),
        ],
        "lane_mm": NEIGHBOUR_LANE_MM,
    })
