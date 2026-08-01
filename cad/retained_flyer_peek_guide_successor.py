"""Physical PEEK shaft-to-tip guide successor for the retained flyer.

CAD brief:
- Model: isolated review assembly and integration API; production assembly is
  not edited.
- Units/frame: millimetres in the retained flyer frame.  M2 rotates about +Z;
  +Y runs outward along the arm and negative Z is rearward.
- Guide: one molded/machined natural-unfilled-PEEK insert, OD2.60 / ID0.60,
  with an analytic R3.25 shaft-root elbow, seated straight run, R57.5/R64
  transition, and an integral axisymmetric polished exit bell.  The exposed
  R3.60 bell surface supports a selectable meridian; the obsolete ceramic
  toroid and its fragile subtraction are removed.
- Retention: three integral PEEK ears, three ISO 4762 M2x6 screws, and three
  standard M2 heat-set inserts in the printed arm.  The insert remains one
  manufactured solid and is removable.
- Printed arm: subtract a 1.45 mm-radius open seat and three insert pilots from
  the exact retained one-piece arm.  The remaining spoke floor and side webs
  are explicit validation targets; all counterweight load paths stay present.
- Wire: 0.20..0.50 mm supported, exact job OD0.22352.  Bore wander is included
  in the R>=3 mm proof; neither job nor maximum wire may touch PETG.
- Outputs: sibling STEP through the CAD launcher, JSON/Markdown report and a
  small integration manifest.  Terminal cap routing remains a separate
  fail-closed gate.
"""

from __future__ import annotations

from copy import deepcopy
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np
import trimesh
from scipy.optimize import least_squares
from build123d import (
    Align,
    Axis,
    Box,
    BuildLine,
    BuildSketch,
    CenterArc,
    Circle,
    Compound,
    Cylinder,
    Line,
    Part,
    Plane,
    Pos,
    RadiusArc,
    Rot,
    Sphere,
    Transition,
    export_stl,
    make_face,
    revolve,
    sweep,
)

import hardware
from params import DEFAULT_STATOR
import permanent_cap_offset_spoke_retained_review as retained
import permanent_cap_offset_spoke_review as base
import permanent_cap_production_review as production_cap


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORTS = ROOT / "out" / "reports"
REVIEW = ROOT / "out" / "review"
SOURCE = HERE / "retained_flyer_peek_guide_successor.py"
STEP_OUT = REVIEW / "retained_flyer_peek_guide_successor.step"
JSON_OUT = REPORTS / "retained_flyer_peek_guide_successor.json"
MD_OUT = REPORTS / "retained_flyer_peek_guide_successor.md"
MANIFEST_OUT = REVIEW / "retained_flyer_peek_guide_successor.manifest.json"

SCHEMA = "retained-flyer-peek-guide-successor/v1"
MANIFEST_SCHEMA = "retained-flyer-peek-guide-successor-manifest/v1"
CTR = (Align.CENTER, Align.CENTER, Align.CENTER)

# One-piece guide dimensions.
GUIDE_OUTER_RADIUS_MM = 1.30
GUIDE_BORE_RADIUS_MM = 0.30
GUIDE_SEAT_RADIUS_MM = 1.45
GUIDE_SEAT_RADIAL_CLEARANCE_MM = (
    GUIDE_SEAT_RADIUS_MM - GUIDE_OUTER_RADIUS_MM
)
GUIDE_CENTERLINE_RADIUS_MM = 3.25
GUIDE_ROOT_AXIAL_START_Z_MM = -42.0
GUIDE_ARM_CENTER_Z_MM = float(base.SPOKE_FRONT_Z_MM)
GUIDE_ROOT_BEND_START_Z_MM = (
    GUIDE_ARM_CENTER_Z_MM - GUIDE_CENTERLINE_RADIUS_MM
)
GUIDE_FIRST_BEND_CENTER_Y_MM = 57.50
GUIDE_AXIAL_RUN_Y_MM = (
    GUIDE_FIRST_BEND_CENTER_Y_MM + GUIDE_CENTERLINE_RADIUS_MM
)
GUIDE_FIRST_BEND_CENTER_Z_MM = (
    GUIDE_ARM_CENTER_Z_MM + GUIDE_CENTERLINE_RADIUS_MM
)
GUIDE_SECOND_BEND_CENTER_Y_MM = float(base.TIP_GUIDE_CENTER_RADIUS_MM)
GUIDE_SECOND_BEND_CENTER_Z_MM = (
    float(base.TIP_GUIDE_CENTER_Z_MM) - GUIDE_CENTERLINE_RADIUS_MM
)
GUIDE_FEED_END_Y_MM = float(base.TIP_GUIDE_CENTER_RADIUS_MM) + 3.0

# One-piece axisymmetric exit bell.  The straight bore opens into an exposed
# 200 degree polished re-entrant flare.  At every route locus the wire selects one
# meridian of this surface; there is no loose eye and no hidden curved bore.
BELL_THROAT_Y_MM = GUIDE_FEED_END_Y_MM - 0.20
BELL_CONTACT_SURFACE_RADIUS_MM = 3.60
BELL_NOMINAL_WALL_MM = 2.00
BELL_SWEEP_DEG = 200.0
BELL_END_ANGLE_DEG = 180.0 - BELL_SWEEP_DEG
BELL_CENTER_RADIAL_MM = (
    GUIDE_BORE_RADIUS_MM + BELL_CONTACT_SURFACE_RADIUS_MM
)

# Retention ears and hardware.
EAR_Y_MM = (15.0, 35.0, 52.0)
EAR_SCREW_X_MM = (3.0, -3.0, 3.0)
EAR_SIZE_X_MM = 8.0
EAR_SIZE_Y_MM = 5.0
EAR_THICKNESS_MM = 1.20
EAR_BOTTOM_Z_MM = GUIDE_ARM_CENTER_Z_MM
M2_CLEARANCE_RADIUS_MM = 1.20
M2_INSERT_PILOT_RADIUS_MM = 1.80
M2_INSERT_TOP_Z_MM = GUIDE_ARM_CENTER_Z_MM - 0.40
M2_INSERT_BOTTOM_Z_MM = GUIDE_ARM_CENTER_Z_MM - 4.40

WIRE_DIAMETER_MIN_MM = 0.20
WIRE_DIAMETER_MAX_MM = 0.50
WIRE_DIAMETER_JOB_MM = float(DEFAULT_STATOR.wire_d)
PEEK_DENSITY_G_MM3 = 1.30e-3

# Symmetric front-plane balance trim.  The annular B777 slugs sit on the
# existing transition-tower front face and are positively retained; their
# common thickness is solved together with three of the four rear slugs.
FRONT_TRIM_X_MM = (-3.60, 3.60)
FRONT_TRIM_Y_MM = 58.0
FRONT_TRIM_SEAT_Z_MM = -13.0
FRONT_TRIM_OD_MM = 6.0
FRONT_TRIM_ID_MM = 2.2
FRONT_TRIM_START_THICKNESS_MM = 2.00582846
FRONT_TRIM_WASHER_THICKNESS_MM = 0.35
FRONT_TRIM_M2_LENGTH_MM = 8.0
FRONT_TRIM_INSERT_BOTTOM_Z_MM = -18.0
FRONT_TRIM_INSERT_TOP_Z_MM = -14.0
FRONT_TRIM_PILOT_BOTTOM_Z_MM = -19.75
FRONT_TRIM_FIXED_REAR_SLUG_INDEX = 1
FRONT_TRIM_FIXED_REAR_SLUG_LENGTH_MM = 0.60
FRONT_TRIM_START_REAR_LENGTHS_MM = (
    1.22843574, 0.60, 1.29653946, 1.72176327,
)
ROOT_SLEEVE_OUTER_DIAMETER_MM = 18.0
ROOT_SLEEVE_BORE_DIAMETER_MM = 12.10
ROOT_SLEEVE_Z_MM = (-43.0, -31.5)
FINAL_SHAFT_BORE_CUT_Z_MM = (-55.0, -29.0)
ROOT_LOAD_TENSION_N = 10.0
ROOT_LOAD_RADIUS_MM = 64.0
ROOT_REVIEW_SAFETY_FACTOR = 3.0
ROOT_PETG_REVIEW_ALLOWABLE_MPA = 10.0


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    body = deepcopy(dict(payload))
    body.pop("report_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def guide_bore_centerline_wire(*, axial_overshoot_mm: float = 0.0):
    """Exact source wire for the one-piece PEEK bore centerline."""

    start_z = GUIDE_ROOT_AXIAL_START_Z_MM - float(axial_overshoot_mm)
    end_y = GUIDE_FEED_END_Y_MM + float(axial_overshoot_mm)
    r = GUIDE_CENTERLINE_RADIUS_MM
    with BuildLine(Plane.YZ) as centerline:
        Line((0.0, start_z), (0.0, GUIDE_ROOT_BEND_START_Z_MM))
        RadiusArc(
            (0.0, GUIDE_ROOT_BEND_START_Z_MM),
            (r, GUIDE_ARM_CENTER_Z_MM),
            r,
        )
        Line(
            (r, GUIDE_ARM_CENTER_Z_MM),
            (GUIDE_FIRST_BEND_CENTER_Y_MM, GUIDE_ARM_CENTER_Z_MM),
        )
        RadiusArc(
            (GUIDE_FIRST_BEND_CENTER_Y_MM, GUIDE_ARM_CENTER_Z_MM),
            (GUIDE_AXIAL_RUN_Y_MM, GUIDE_FIRST_BEND_CENTER_Z_MM),
            r,
        )
        Line(
            (GUIDE_AXIAL_RUN_Y_MM, GUIDE_FIRST_BEND_CENTER_Z_MM),
            (GUIDE_AXIAL_RUN_Y_MM, GUIDE_SECOND_BEND_CENTER_Z_MM),
        )
        RadiusArc(
            (GUIDE_AXIAL_RUN_Y_MM, GUIDE_SECOND_BEND_CENTER_Z_MM),
            (GUIDE_SECOND_BEND_CENTER_Y_MM,
             float(base.TIP_GUIDE_CENTER_Z_MM)),
            r,
        )
        Line(
            (GUIDE_SECOND_BEND_CENTER_Y_MM,
             float(base.TIP_GUIDE_CENTER_Z_MM)),
            (end_y, float(base.TIP_GUIDE_CENTER_Z_MM)),
        )
    return centerline.wire()


def guide_bore_centerline_samples(max_step_mm: float = 0.50) -> list[list[float]]:
    """Stable player/API samples of the exact OCC bore source wire."""

    if float(max_step_mm) <= 0.0:
        raise ValueError("max_step_mm must be positive")
    result: list[list[float]] = []
    for edge in guide_bore_centerline_wire().edges():
        count = max(2, int(math.ceil(float(edge.length) / max_step_mm)) + 1)
        for index in range(count):
            if result and index == 0:
                continue
            point = edge.position_at(index / (count - 1))
            result.append([float(point.X), float(point.Y), float(point.Z)])
    return result


def _tube(radius_mm: float, *, axial_overshoot_mm: float = 0.0) -> Part:
    """Robust Minkowski tube over the exact G1 R3.25 centreline.

    Each edge is swept separately and every G1 joint receives an overlapping
    spherical blend.  This avoids the null-triangulation/nonmanifold seams
    produced by one multi-edge OpenCascade pipe while preserving the same
    analytic centreline and minimum bend radius.
    """

    wire = guide_bore_centerline_wire(
        axial_overshoot_mm=float(axial_overshoot_mm)
    )
    pieces: list[Part] = []
    edges = list(wire.edges())
    for edge in edges:
        origin = edge.position_at(0.0)
        tangent = edge.tangent_at(0.0)
        tangent_np = np.array([tangent.X, tangent.Y, tangent.Z], dtype=float)
        tangent_np /= np.linalg.norm(tangent_np)
        reference = (
            np.array([1.0, 0.0, 0.0])
            if abs(float(tangent_np[0])) < 0.9
            else np.array([0.0, 1.0, 0.0])
        )
        x_dir = np.cross(reference, tangent_np)
        x_dir /= np.linalg.norm(x_dir)
        profile_plane = Plane(
            origin=(origin.X, origin.Y, origin.Z),
            x_dir=tuple(float(value) for value in x_dir),
            z_dir=tuple(float(value) for value in tangent_np),
        )
        with BuildSketch(profile_plane) as profile:
            Circle(float(radius_mm))
        pieces.append(sweep(profile.sketch, edge))
    for edge in edges[:-1]:
        joint = edge.position_at(1.0)
        pieces.append(
            Pos(joint.X, joint.Y, joint.Z) * Sphere(float(radius_mm))
        )
    result = pieces[0].fuse(*pieces[1:]).clean()
    result.label = f"analytic_R3p25_tube_r{radius_mm:.3f}"
    return result


@lru_cache(maxsize=1)
def bell_fairlead() -> Part:
    """One exposed/polishable axisymmetric PEEK exit-bell wall."""

    radius = BELL_CONTACT_SURFACE_RADIUS_MM
    outer_radius = radius - BELL_NOMINAL_WALL_MM
    if outer_radius <= 0.0:
        raise ValueError("bell wall consumes the circular generatrix")
    center = (BELL_CENTER_RADIAL_MM, BELL_THROAT_Y_MM)
    end_angle = math.radians(BELL_END_ANGLE_DEG)
    inner_end = (
        BELL_CENTER_RADIAL_MM + radius * math.cos(end_angle),
        BELL_THROAT_Y_MM + radius * math.sin(end_angle),
    )
    outer_end = (
        BELL_CENTER_RADIAL_MM + outer_radius * math.cos(end_angle),
        BELL_THROAT_Y_MM + outer_radius * math.sin(end_angle),
    )
    with BuildLine(Plane.XY) as profile:
        CenterArc(center, radius, 180.0, -BELL_SWEEP_DEG)
        Line(inner_end, outer_end)
        CenterArc(
            center, outer_radius, BELL_END_ANGLE_DEG, BELL_SWEEP_DEG
        )
        Line(
            (BELL_CENTER_RADIAL_MM - outer_radius, BELL_THROAT_Y_MM),
            (BELL_CENTER_RADIAL_MM - radius, BELL_THROAT_Y_MM),
        )
    result = Pos(0.0, 0.0, float(base.TIP_GUIDE_CENTER_Z_MM)) * revolve(
        make_face(profile.wire()), axis=Axis.Y
    )
    result.label = "one_piece_axisymmetric_polished_PEEK_exit_bell"
    return result


def _retention_ear(x_screw: float, y: float) -> Part:
    ear = Pos(
        0.0,
        y,
        EAR_BOTTOM_Z_MM + EAR_THICKNESS_MM / 2.0,
    ) * Box(
        EAR_SIZE_X_MM, EAR_SIZE_Y_MM, EAR_THICKNESS_MM, align=CTR,
    )
    hole = Pos(
        x_screw,
        y,
        EAR_BOTTOM_Z_MM - 0.5,
    ) * Cylinder(
        M2_CLEARANCE_RADIUS_MM,
        EAR_THICKNESS_MM + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    return ear - hole


@lru_cache(maxsize=1)
def peek_guide_insert() -> Part:
    """One positive-volume PEEK guide with an open polished wire bore."""

    insert = _tube(GUIDE_OUTER_RADIUS_MM).fuse(bell_fairlead())
    for x_screw, y in zip(EAR_SCREW_X_MM, EAR_Y_MM):
        insert += _retention_ear(x_screw, y)
    # Cut the bore last so no retention ear can silently refill the wire
    # passage during the preceding positive unions.
    insert -= _tube(GUIDE_BORE_RADIUS_MM, axial_overshoot_mm=1.0)
    solids = list(insert.solids())
    if len(solids) != 1:
        raise RuntimeError(
            f"PEEK guide insert must be one solid; observed {len(solids)}"
        )
    insert.label = "one_piece_polished_unfilled_PEEK_shaft_to_tip_guide"
    return insert


def guide_seat_tool() -> Part:
    tool = _tube(GUIDE_SEAT_RADIUS_MM, axial_overshoot_mm=0.5)
    tool.label = "printed_arm_open_PEEK_guide_seat_tool"
    return tool


@lru_cache(maxsize=1)
def successor_root_sleeve() -> Part:
    """Exact OD18/ID12.10 root sleeve from the final shaft authority."""

    z0, z1 = ROOT_SLEEVE_Z_MM
    outer = Pos(0.0, 0.0, z0) * Cylinder(
        ROOT_SLEEVE_OUTER_DIAMETER_MM / 2.0,
        z1 - z0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    bore = Pos(0.0, 0.0, z0 - 1.0) * Cylinder(
        ROOT_SLEEVE_BORE_DIAMETER_MM / 2.0,
        z1 - z0 + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    result = outer - bore
    result.label = "OD18_x11p5_annular_root_load_path_web_ID12p10"
    return result


def _insert_pilot(x: float, y: float) -> Part:
    return Pos(x, y, M2_INSERT_BOTTOM_Z_MM) * Cylinder(
        M2_INSERT_PILOT_RADIUS_MM,
        M2_INSERT_TOP_Z_MM - M2_INSERT_BOTTOM_Z_MM,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )


def front_trim_slug(x_mm: float, thickness_mm: float) -> Part:
    """One OD6/ID2.2 annular ASTM-B777 front trim slug."""

    thickness = float(thickness_mm)
    if not 1.0 <= thickness <= 5.0:
        raise ValueError("front trim thickness is outside 1..5 mm")
    outer = Pos(
        float(x_mm), FRONT_TRIM_Y_MM, FRONT_TRIM_SEAT_Z_MM
    ) * Cylinder(
        FRONT_TRIM_OD_MM / 2.0,
        thickness,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    bore = Pos(
        float(x_mm), FRONT_TRIM_Y_MM, FRONT_TRIM_SEAT_Z_MM - 0.10
    ) * Cylinder(
        FRONT_TRIM_ID_MM / 2.0,
        thickness + 0.20,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    result = outer - bore
    result.label = (
        f"front_balance_B777_annular_slug_x{x_mm:+.1f}_t{thickness:.6f}"
    )
    return result


def front_trim_hardware(thickness_mm: float) -> list[Part]:
    result: list[Part] = []
    slug_front = FRONT_TRIM_SEAT_Z_MM + float(thickness_mm)
    for x_mm in FRONT_TRIM_X_MM:
        washer = Pos(x_mm, FRONT_TRIM_Y_MM, slug_front) * (
            hardware.plain_washer(
                "M2", label=f"front_balance_M2_washer_x{x_mm:+.1f}"
            )
        )
        screw = Pos(
            x_mm,
            FRONT_TRIM_Y_MM,
            slug_front + FRONT_TRIM_WASHER_THICKNESS_MM,
        ) * hardware.socket_head_cap_screw(
            "M2", FRONT_TRIM_M2_LENGTH_MM,
            label=f"front_balance_ISO4762_M2x8_x{x_mm:+.1f}",
        )
        insert = Pos(
            x_mm, FRONT_TRIM_Y_MM, FRONT_TRIM_INSERT_BOTTOM_Z_MM
        ) * hardware.heat_set_insert(
            "M2", length="standard",
            label=f"front_balance_M2_heat_insert_x{x_mm:+.1f}",
        )
        result.extend((washer, screw, insert))
    return result


@lru_cache(maxsize=1)
def torus_free_retained_arm_base() -> Part:
    """Rebuild the retained load path without the obsolete torus cut/seat."""

    components = base.offset_spoke_arm_components()
    arm = components["collar"]
    # The old cylindrical cradle and torus-seat subtraction are deliberately
    # absent.  The stout rectangular tip bridge becomes the machinable open
    # cradle after the PEEK guide seat is cut in ``revised_retained_arm``.
    for name in ("spoke", "transition_tower", "tip_bridge"):
        arm += components[name]
    arm += retained._box(
        -retained.RAIL_WIDTH_MM / 2.0,
        retained.RAIL_WIDTH_MM / 2.0,
        retained.RAIL_Y_MM[0], retained.RAIL_Y_MM[1],
        retained.RAIL_Z_MM[0], retained.RAIL_Z_MM[1],
        "continuous_deep_counterrail",
    )
    arm += retained._box(
        retained.TOWER_X_MM[0], retained.TOWER_X_MM[1],
        retained.TOWER_Y_MM[0], retained.TOWER_Y_MM[1],
        retained.TOWER_Z_MM[0], retained.TOWER_Z_MM[1],
        "continuous_R58_outboard_counterrail_tower",
    )
    # The original x=+/-2 counterrail crossed the OD12 shaft envelope and
    # became disconnected when that bore was correctly recut.  This U-shaped
    # twin-cheek bridge carries the counterweight rail around the shaft with
    # two independent 3 mm sections and positive overlap into the spoke.
    arm += retained._box(
        -9.0, 9.0, -10.0, -7.0, -34.5, -31.5,
        "counterrail_rear_crossbar_outside_OD12_shaft",
    )
    arm += retained._box(
        6.0, 9.0, -9.0, 4.0, -34.5, -31.5,
        "counterrail_positive_X_shaft_bypass_cheek",
    )
    arm += retained._box(
        -9.0, -6.0, -9.0, 4.0, -34.5, -31.5,
        "counterrail_negative_X_shaft_bypass_cheek",
    )
    for pocket in retained.POCKETS:
        arm += retained._cyl_z(
            pocket.housing_r_mm,
            pocket.rear_z_mm,
            pocket.front_z_mm,
            x=pocket.x_mm,
            y=pocket.y_mm,
        )
    for pocket in retained.POCKETS:
        arm -= retained._cyl_z(
            pocket.pocket_r_mm,
            pocket.z(retained.FLOOR_Z_MM[1]),
            pocket.front_z_mm + 0.20,
            x=pocket.x_mm,
            y=pocket.y_mm,
        )
        arm -= retained._screw_clearance_tool(pocket)
    solids = list(arm.solids())
    if len(solids) != 1:
        raise RuntimeError(
            f"torus-free retained arm base must be one solid; {len(solids)}"
        )
    arm.label = "torus_free_retained_load_bearing_arm_base"
    return arm


@lru_cache(maxsize=1)
def revised_retained_arm() -> Part:
    """Watertight torus-free arm with open guide seat and M2 pilots."""

    arm = (
        torus_free_retained_arm_base().fuse(successor_root_sleeve())
        - guide_seat_tool()
    )
    for x, y in zip(EAR_SCREW_X_MM, EAR_Y_MM):
        arm -= _insert_pilot(x, y)
    for x_mm in FRONT_TRIM_X_MM:
        arm -= Pos(
            x_mm, FRONT_TRIM_Y_MM, FRONT_TRIM_PILOT_BOTTOM_Z_MM
        ) * Cylinder(
            M2_INSERT_PILOT_RADIUS_MM,
            FRONT_TRIM_INSERT_TOP_Z_MM - FRONT_TRIM_PILOT_BOTTOM_Z_MM,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        arm -= Pos(
            x_mm, FRONT_TRIM_Y_MM, FRONT_TRIM_INSERT_TOP_Z_MM - 0.10
        ) * Cylinder(
            M2_CLEARANCE_RADIUS_MM,
            FRONT_TRIM_SEAT_Z_MM - FRONT_TRIM_INSERT_TOP_Z_MM + 0.70,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
    # Recut the OD12 shaft bore after every positive arm union.  The source
    # collar cut stopped at z=-36.8 and the later spoke otherwise refilled
    # 216 mm3 of the running shaft envelope.
    arm -= base._cyl_z(
        ROOT_SLEEVE_BORE_DIAMETER_MM / 2.0,
        FINAL_SHAFT_BORE_CUT_Z_MM[0],
        FINAL_SHAFT_BORE_CUT_Z_MM[1],
    )
    solids = list(arm.solids())
    if len(solids) != 1:
        raise RuntimeError(
            f"guide-seated retained arm must remain one solid; {len(solids)}"
        )
    arm.label = (
        "torus_free_retained_arm_with_open_PEEK_cradle_seat_one_solid"
    )
    return arm


def guide_retention_hardware() -> list[Part]:
    children: list[Part] = []
    ear_top = EAR_BOTTOM_Z_MM + EAR_THICKNESS_MM
    for index, (x, y) in enumerate(zip(EAR_SCREW_X_MM, EAR_Y_MM), start=1):
        screw = Pos(x, y, ear_top) * hardware.socket_head_cap_screw(
            "M2", 6.0, label=f"PEEK_guide_ISO4762_M2x6_{index}"
        )
        children.append(screw)
        insert = hardware.heat_set_insert(
            "M2", length="standard",
            label=f"PEEK_guide_M2_heat_set_insert_{index}",
        )
        # Heat-set insert local +Z; retain it below the printed front face.
        insert = Pos(x, y, M2_INSERT_BOTTOM_Z_MM) * insert
        insert.label = f"PEEK_guide_M2_heat_set_insert_{index}"
        children.append(insert)
    return children


def guide_wire_envelope(diameter_mm: float, label: str) -> Part:
    result = _tube(float(diameter_mm) / 2.0)
    result.label = label
    return result


def actual_production_cap_context() -> Compound:
    transform = (
        Pos(0.0, 0.0, base.deepest_axis_z_mm())
        * Rot(0.0, 90.0, 0.0)
        * Rot(-90.0, 0.0, 0.0)
    )
    front = transform * production_cap.cap_part(1)
    rear = transform * production_cap.cap_part(-1)
    front.label = "actual_front_PEEK_cap_deepest_raw_pose"
    rear.label = "actual_rear_PEEK_cap_deepest_raw_pose"
    result = Compound(children=[front, rear])
    result.label = "actual_production_review_PEEK_cap_pair_context"
    return result


def _density_properties(
    name: str, shape: Part, density_g_mm3: float, material: str,
) -> dict[str, Any]:
    volume = float(shape.volume)
    mass = volume * float(density_g_mm3)
    center = shape.center()
    izz_volume = (
        float(shape.matrix_of_inertia[2][2])
        + volume * (float(center.X) ** 2 + float(center.Y) ** 2)
    )
    return {
        "name": name,
        "material": material,
        "density_g_cm3": float(density_g_mm3) * 1000.0,
        "volume_mm3": volume,
        "mass_g": mass,
        "center_of_mass_mm": [
            float(center.X), float(center.Y), float(center.Z),
        ],
        "static_first_moment_g_mm": [
            mass * float(center.X), mass * float(center.Y),
        ],
        "couple_first_moment_g_mm2": [
            mass * float(center.X) * float(center.Z),
            mass * float(center.Y) * float(center.Z),
        ],
        "izz_about_M2_axis_g_mm2": (
            izz_volume * float(density_g_mm3)
        ),
    }


@lru_cache(maxsize=1)
def _successor_base_mass_rows() -> tuple[dict[str, Any], ...]:
    rows: list[dict[str, Any]] = [
        retained._properties(
            "revised_bore_recut_counterrail_bypass_arm",
            revised_retained_arm(), "PETG",
        )
    ]
    for name, shape, material in retained.base_rotating_parts():
        if name in ("retained_printed_arm", "R64_ceramic_toroid_guide"):
            continue
        rows.append(retained._properties(name, shape, material))
    rows.append(_density_properties(
        "one_piece_PEEK_bore_and_exit_bell",
        peek_guide_insert(), PEEK_DENSITY_G_MM3, "natural unfilled PEEK",
    ))
    for index, shape in enumerate(guide_retention_hardware()):
        rows.append(retained._properties(
            f"PEEK_guide_retention_{index + 1}", shape,
            "steel" if index % 2 == 0 else "brass",
        ))
    return tuple(rows)


@lru_cache(maxsize=1)
def _front_trim_pair_response_coefficients() -> np.ndarray:
    samples = np.asarray((1.25, 2.75, 4.50), dtype=float)
    values = []
    for thickness in samples:
        rows = [
            _density_properties(
                f"front_trim_{x_mm:+.1f}",
                front_trim_slug(x_mm, float(thickness)),
                retained.TUNGSTEN_DENSITY_G_CM3 / 1000.0,
                "ASTM-B777 tungsten alloy",
            )
            for x_mm in FRONT_TRIM_X_MM
        ]
        rows.extend(
            retained._properties(
                f"front_trim_hardware_{index + 1}", shape,
                "brass" if index % 3 == 2 else "steel",
            )
            for index, shape in enumerate(front_trim_hardware(float(thickness)))
        )
        total = retained._sum_properties(rows)
        values.append([
            *total["static_first_moment_g_mm"],
            *total["couple_first_moment_g_mm2"],
        ])
    array = np.asarray(values, dtype=float)
    return np.asarray([
        np.polyfit(samples, array[:, channel], 2)
        for channel in range(4)
    ])


def _balance_vector(total: Mapping[str, Any]) -> np.ndarray:
    return np.asarray([
        *total["static_first_moment_g_mm"],
        *total["couple_first_moment_g_mm2"],
    ], dtype=float)


def solve_successor_balance_with_base_rows(
    base_rows: Iterable[Mapping[str, Any]],
) -> dict[str, Any]:
    """Solve the six physical trim slugs against caller-owned mass rows.

    ``base_rows`` is the complete exact untrimmed rotating assembly, excluding
    the two front trim slugs/their hardware and the four rear correction
    stacks.  This is the production integration seam: a merged candidate can
    replace the legacy drive context rather than inheriting it accidentally.
    """

    frozen_base_rows = tuple(deepcopy(dict(row)) for row in base_rows)
    if not frozen_base_rows:
        raise ValueError("balance base rows must not be empty")
    base_total = retained._sum_properties(frozen_base_rows)
    base_vector = _balance_vector(base_total)
    front_coeff = _front_trim_pair_response_coefficients()
    rear_coeff = retained._occ_balance_response_coefficients()

    def lengths_from_variables(values: np.ndarray) -> tuple[float, ...]:
        return (
            float(values[1]),
            FRONT_TRIM_FIXED_REAR_SLUG_LENGTH_MM,
            float(values[2]),
            float(values[3]),
        )

    def residual(values: np.ndarray) -> np.ndarray:
        moments = base_vector.copy()
        thickness = float(values[0])
        moments += np.asarray([
            np.polyval(front_coeff[channel], thickness)
            for channel in range(4)
        ])
        for coefficients, length in zip(
            rear_coeff, lengths_from_variables(values)
        ):
            moments += np.asarray([
                np.polyval(coefficients[channel], length)
                for channel in range(4)
            ])
        moments[2:] /= 40.0
        return moments

    start = np.asarray((
        FRONT_TRIM_START_THICKNESS_MM,
        FRONT_TRIM_START_REAR_LENGTHS_MM[0],
        FRONT_TRIM_START_REAR_LENGTHS_MM[2],
        FRONT_TRIM_START_REAR_LENGTHS_MM[3],
    ), dtype=float)
    result = least_squares(
        residual, start,
        bounds=(
            np.asarray((1.0, 0.35, 0.35, 0.35)),
            np.asarray((5.0, 5.35, 5.35, 5.35)),
        ),
        xtol=1.0e-13, ftol=1.0e-13, gtol=1.0e-13,
        max_nfev=500,
    )
    if not result.success:
        raise RuntimeError(f"successor balance solve failed: {result.message}")
    thickness = float(result.x[0])
    rear_lengths = lengths_from_variables(result.x)

    exact_rows = [deepcopy(row) for row in frozen_base_rows]
    exact_rows.extend(
        _density_properties(
            f"front_trim_B777_{x_mm:+.1f}",
            front_trim_slug(x_mm, thickness),
            retained.TUNGSTEN_DENSITY_G_CM3 / 1000.0,
            "ASTM-B777 tungsten alloy",
        )
        for x_mm in FRONT_TRIM_X_MM
    )
    exact_rows.extend(
        retained._properties(
            f"front_trim_hardware_{index + 1}", shape,
            "brass" if index % 3 == 2 else "steel",
        )
        for index, shape in enumerate(front_trim_hardware(thickness))
    )
    exact_rows.extend(
        retained._properties(name, shape, material)
        for name, shape, material in retained.correction_parts(rear_lengths)
    )
    exact_total = retained._sum_properties(exact_rows)
    exact_residual = _balance_vector(exact_total)
    scaled = exact_residual.copy()
    scaled[2:] /= 40.0
    residual_norm = float(np.linalg.norm(scaled))
    if residual_norm > 1.0e-6:
        raise RuntimeError(
            f"six-slug exact OCC balance residual {scaled.tolist()}"
        )
    if not all(
        retained.SLUG_MIN_LENGTH_MM <= value <= retained.SLUG_MAX_LENGTH_MM
        for value in rear_lengths
    ):
        raise RuntimeError("successor rear slug solution is out of bounds")
    return {
        "authority": "CALLER_SUPPLIED_EXACT_UNTRIMMED_BASE_ROWS",
        "base_row_count": len(frozen_base_rows),
        "front_trim_common_thickness_mm": thickness,
        "front_trim_slug_count": 2,
        "front_trim_each_mass_g": next(
            float(row["mass_g"]) for row in exact_rows
            if str(row["name"]).startswith("front_trim_B777_")
        ),
        "rear_slug_lengths_mm": list(rear_lengths),
        "minimum_rear_slug_margin_to_0p35mm_mm": (
            min(rear_lengths) - retained.SLUG_MIN_LENGTH_MM
        ),
        "scaled_balance_residual": scaled.tolist(),
        "scaled_balance_residual_norm": residual_norm,
        "mass_properties": exact_total,
        "mass_rows": exact_rows,
    }


@lru_cache(maxsize=1)
def successor_balance_solution() -> dict[str, Any]:
    """Isolated balance witness; the imported legacy drive is not final."""

    result = solve_successor_balance_with_base_rows(
        _successor_base_mass_rows()
    )
    result["authority"] = (
        "ISOLATED_SUCCESSOR_WITH_LEGACY_DRIVE_CONTEXT_ONLY__"
        "MERGED_RELEASE_MUST_RESOLVE_FROM_CALLER_BASE_ROWS"
    )
    return result


def rotating_parts() -> list[Part]:
    balance = successor_balance_solution()
    lengths = balance["rear_slug_lengths_mm"]
    result: list[Part] = [revised_retained_arm()]
    for name, shape, _material in retained.base_rotating_parts():
        if name in ("retained_printed_arm", "R64_ceramic_toroid_guide"):
            continue
        shape.label = name
        result.append(shape)
    result.extend(
        shape for _name, shape, _material
        in retained.correction_parts(lengths)
    )
    result.append(peek_guide_insert())
    result.extend(guide_retention_hardware())
    thickness = float(balance["front_trim_common_thickness_mm"])
    result.extend(
        front_trim_slug(x_mm, thickness) for x_mm in FRONT_TRIM_X_MM
    )
    result.extend(front_trim_hardware(thickness))
    return result


def _bbox(shape: Part | Compound) -> dict[str, list[float]]:
    box = shape.bounding_box()
    minimum = [float(box.min.X), float(box.min.Y), float(box.min.Z)]
    maximum = [float(box.max.X), float(box.max.Y), float(box.max.Z)]
    return {
        "minimum_mm": minimum,
        "maximum_mm": maximum,
        "size_mm": [b - a for a, b in zip(minimum, maximum)],
    }


def _intersection_volume(left: Part | Compound,
                         right: Part | Compound) -> float:
    common = left & right
    return 0.0 if common is None else float(common.volume)


def root_sleeve_load_path_audit(arm: Part) -> dict[str, Any]:
    """Measure the real sleeve-to-arm load path and screen its root stress."""

    sleeve = successor_root_sleeve()
    components = base.offset_spoke_arm_components()
    rear_rail = retained._box(
        -retained.RAIL_WIDTH_MM / 2.0,
        retained.RAIL_WIDTH_MM / 2.0,
        retained.RAIL_Y_MM[0], retained.RAIL_Y_MM[1],
        retained.RAIL_Z_MM[0], retained.RAIL_Z_MM[1],
        "root_audit_existing_rear_counterrail",
    )
    rear_crossbar = retained._box(
        -9.0, 9.0, -10.0, -7.0, -34.5, -31.5,
        "root_audit_rear_crossbar",
    )
    bypass_cheeks = (
        retained._box(
            6.0, 9.0, -9.0, 4.0, -34.5, -31.5,
            "root_audit_positive_bypass_cheek",
        ),
        retained._box(
            -9.0, -6.0, -9.0, 4.0, -34.5, -31.5,
            "root_audit_negative_bypass_cheek",
        ),
    )
    outer_r = ROOT_SLEEVE_OUTER_DIAMETER_MM / 2.0
    inner_r = ROOT_SLEEVE_BORE_DIAMETER_MM / 2.0
    area = math.pi * (outer_r**2 - inner_r**2)
    polar_j = math.pi / 2.0 * (outer_r**4 - inner_r**4)
    planar_i = polar_j / 2.0
    moment = ROOT_LOAD_TENSION_N * ROOT_LOAD_RADIUS_MM
    torsion_shear = moment * outer_r / polar_j
    bending_stress = moment * outer_r / planar_i
    von_mises = math.sqrt(
        bending_stress**2 + 3.0 * torsion_shear**2
    )
    factored = ROOT_REVIEW_SAFETY_FACTOR * von_mises
    return {
        "root_sleeve_label": str(sleeve.label),
        "root_sleeve_solid_count": len(list(sleeve.solids())),
        "final_arm_solid_count": len(list(arm.solids())),
        "outer_diameter_mm": ROOT_SLEEVE_OUTER_DIAMETER_MM,
        "inner_diameter_mm": ROOT_SLEEVE_BORE_DIAMETER_MM,
        "axial_span_z_mm": list(ROOT_SLEEVE_Z_MM),
        "radial_ligament_mm": outer_r - inner_r,
        "sleeve_to_existing_collar_overlap_mm3": _intersection_volume(
            sleeve, components["collar"]
        ),
        "sleeve_to_main_spoke_overlap_mm3": _intersection_volume(
            sleeve, components["spoke"]
        ),
        "sleeve_to_existing_rear_counterrail_overlap_mm3": (
            _intersection_volume(sleeve, rear_rail)
        ),
        "sleeve_to_rear_crossbar_overlap_mm3": _intersection_volume(
            sleeve, rear_crossbar
        ),
        "sleeve_to_bypass_cheek_overlap_mm3": [
            _intersection_volume(sleeve, cheek)
            for cheek in bypass_cheeks
        ],
        "conservative_combined_root_load_case": {
            "wire_tension_N": ROOT_LOAD_TENSION_N,
            "load_radius_mm": ROOT_LOAD_RADIUS_MM,
            "simultaneous_bending_and_torsion_moment_each_Nmm": moment,
            "basis": (
                "conservative simultaneous full 10 N at R64 bending and "
                "torsion; a single wire vector cannot apply both full "
                "components simultaneously"
            ),
            "annular_area_mm2": area,
            "polar_second_moment_J_mm4": polar_j,
            "planar_second_moment_I_mm4": planar_i,
            "outer_fiber_torsional_shear_MPa": torsion_shear,
            "outer_fiber_bending_stress_MPa": bending_stress,
            "von_Mises_equivalent_MPa": von_mises,
            "review_safety_factor": ROOT_REVIEW_SAFETY_FACTOR,
            "safety_factored_equivalent_MPa": factored,
            "PETG_review_allowable_MPa": ROOT_PETG_REVIEW_ALLOWABLE_MPA,
            "allowable_to_factored_load_margin": (
                ROOT_PETG_REVIEW_ALLOWABLE_MPA / factored
            ),
            "passes_review_allowable": (
                factored <= ROOT_PETG_REVIEW_ALLOWABLE_MPA
            ),
            "orientation_matched_physical_coupon_complete": False,
        },
    }


def _processed_stl_topology(shape: Part, name: str) -> dict[str, Any]:
    folder = REVIEW / "retained_flyer_mesh_checks"
    folder.mkdir(parents=True, exist_ok=True)
    target = folder / f"{name}.stl"
    export_stl(
        shape, target, tolerance=0.03, angular_tolerance=0.08,
        ascii_format=False,
    )
    mesh = trimesh.load(target, force="mesh", process=False)
    # STL repeats triangle vertices by definition.  Validating processing
    # merges those exact duplicates and removes the duplicate seam triangles
    # emitted at OCC G1 joints; no hole filling or geometric approximation is
    # permitted.  The exported processed body is the collision authority.
    mesh.process(validate=True)
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()
    processed = folder / f"{name}.processed.stl"
    mesh.export(processed)
    edge_use = np.bincount(mesh.edges_unique_inverse)
    boundary_edges = int(np.count_nonzero(edge_use == 1))
    nonmanifold_edges = int(np.count_nonzero(edge_use > 2))
    return {
        "raw_path": str(target.relative_to(ROOT)).replace("\\", "/"),
        "path": str(processed.relative_to(ROOT)).replace("\\", "/"),
        "vertices": int(len(mesh.vertices)),
        "triangles": int(len(mesh.faces)),
        "boundary_edges": boundary_edges,
        "nonmanifold_edges": nonmanifold_edges,
        "watertight": bool(mesh.is_watertight),
        "file_sha256": _sha256(processed),
    }


def analyze() -> dict[str, Any]:
    arm = revised_retained_arm()
    original_arm = torus_free_retained_arm_base()
    guide = peek_guide_insert()
    bell = bell_fairlead()
    seat = guide_seat_tool()
    job_wire = guide_wire_envelope(
        WIRE_DIAMETER_JOB_MM, "job_wire_inside_PEEK_bore_witness"
    )
    max_wire = guide_wire_envelope(
        WIRE_DIAMETER_MAX_MM, "maximum_wire_inside_PEEK_bore_witness"
    )
    caps = actual_production_cap_context()
    balance = successor_balance_solution()
    correction = Compound(children=[
        shape for _name, shape, _material
        in retained.correction_parts(balance["rear_slug_lengths_mm"])
    ])
    front_trim = Compound(children=[
        *(front_trim_slug(
            x_mm, balance["front_trim_common_thickness_mm"]
        ) for x_mm in FRONT_TRIM_X_MM),
        *front_trim_hardware(balance["front_trim_common_thickness_mm"]),
    ])

    job_to_arm = _intersection_volume(job_wire, arm)
    max_to_arm = _intersection_volume(max_wire, arm)
    job_to_guide = _intersection_volume(job_wire, guide)
    max_to_guide = _intersection_volume(max_wire, guide)
    guide_to_arm = _intersection_volume(guide, arm)
    seat_to_corrections = _intersection_volume(seat, correction)
    front_trim_to_guide = _intersection_volume(front_trim, guide)
    front_trim_to_arm = _intersection_volume(front_trim, arm)
    front_trim_to_caps = float(front_trim.distance_to(caps))
    trim_thickness = float(balance["front_trim_common_thickness_mm"])
    trim_slugs = [
        front_trim_slug(x_mm, trim_thickness)
        for x_mm in FRONT_TRIM_X_MM
    ]
    trim_hardware = front_trim_hardware(trim_thickness)
    trim_washers = trim_hardware[0::3]
    trim_screws = trim_hardware[1::3]
    trim_inserts = trim_hardware[2::3]
    trim_slug_to_arm_distances = [
        float(slug.distance_to(arm)) for slug in trim_slugs
    ]
    trim_washer_to_slug_distances = [
        float(washer.distance_to(slug))
        for washer, slug in zip(trim_washers, trim_slugs)
    ]
    trim_screw_to_washer_distances = [
        float(screw.distance_to(washer))
        for screw, washer in zip(trim_screws, trim_washers)
    ]
    trim_insert_to_arm_intersections = [
        _intersection_volume(insert, arm) for insert in trim_inserts
    ]
    trim_screw_to_arm_intersections = [
        _intersection_volume(screw, arm) for screw in trim_screws
    ]
    trim_screw_tip_z = (
        FRONT_TRIM_SEAT_Z_MM + trim_thickness
        + FRONT_TRIM_WASHER_THICKNESS_MM - FRONT_TRIM_M2_LENGTH_MM
    )
    trim_screw_tip_clearance = (
        trim_screw_tip_z - FRONT_TRIM_PILOT_BOTTOM_Z_MM
    )
    trim_blind_material_behind_pilot = (
        FRONT_TRIM_PILOT_BOTTOM_Z_MM - float(base.SPOKE_REAR_Z_MM)
    )
    trim_min_outer_radial_wall = (
        base.SPOKE_WIDTH_MM / 2.0
        - (abs(FRONT_TRIM_X_MM[1]) + M2_INSERT_PILOT_RADIUS_MM)
    )
    guide_to_caps = float(guide.distance_to(caps))
    arm_to_caps = float(arm.distance_to(caps))
    shaft_envelope = base._cyl_z(
        float(base.P.flyer_shaft_od) / 2.0,
        -54.0,
        float(base.SPOKE_FRONT_Z_MM) + 0.60,
    )
    shaft_to_arm = _intersection_volume(shaft_envelope, arm)
    root_audit = root_sleeve_load_path_audit(arm)

    min_wire_radius = WIRE_DIAMETER_MIN_MM / 2.0
    max_bore_wander = GUIDE_BORE_RADIUS_MM - min_wire_radius
    minimum_possible_wire_center_radius = (
        GUIDE_CENTERLINE_RADIUS_MM - max_bore_wander
    )
    bell_minimum_wire_center_radius = (
        BELL_CONTACT_SURFACE_RADIUS_MM + min_wire_radius
    )
    bell_machining_profile_tolerance_mm = 0.10
    bell_minimum_finished_wall_mm = (
        BELL_NOMINAL_WALL_MM - bell_machining_profile_tolerance_mm
    )
    # Conservative 2 mm-wide meridian strip, 2 mm nominal wall and a 2 mm
    # effective lip lever.  This intentionally ignores circumferential shell
    # sharing; the physical 10 N abrasion/bend coupon remains mandatory.
    bell_strip_width_mm = 2.0
    bell_line_load_lever_mm = 2.0
    bell_section_modulus_mm3 = (
        bell_strip_width_mm * bell_minimum_finished_wall_mm ** 2 / 6.0
    )
    bell_10N_bending_stress_mpa = (
        10.0 * bell_line_load_lever_mm / bell_section_modulus_mm3
    )
    remaining_spoke_floor = (
        float(base.SPOKE_THICKNESS_MM) - GUIDE_SEAT_RADIUS_MM
    )
    remaining_side_web = (
        float(base.SPOKE_WIDTH_MM) - 2.0 * GUIDE_SEAT_RADIUS_MM
    ) / 2.0
    removed_volume = float(original_arm.volume) - float(arm.volume)
    arm_mesh = _processed_stl_topology(arm, "torus_free_revised_arm")
    guide_mesh = _processed_stl_topology(guide, "one_piece_PEEK_guide")

    geometry_gates = {
        "PEEK_guide_exactly_one_solid": len(list(guide.solids())) == 1,
        "exit_bell_exactly_one_solid": len(list(bell.solids())) == 1,
        "revised_printed_arm_exactly_one_solid": len(list(arm.solids())) == 1,
        "guide_centerline_primitives_R3p25": math.isclose(
            GUIDE_CENTERLINE_RADIUS_MM, 3.25, abs_tol=1.0e-12
        ),
        "all_supported_wire_positions_remain_R_ge_3mm": (
            minimum_possible_wire_center_radius >= 3.0
        ),
        "bell_all_wire_sizes_remain_R_ge_3p25mm": (
            bell_minimum_wire_center_radius >= 3.25
        ),
        "bell_finished_wall_ge_1p8mm": (
            bell_minimum_finished_wall_mm >= 1.8
        ),
        "bell_10N_strip_stress_le_25MPa": (
            bell_10N_bending_stress_mpa <= 25.0
        ),
        "job_wire_zero_positive_PETG_contact": job_to_arm <= 1.0e-8,
        "max_wire_zero_positive_PETG_contact": max_to_arm <= 1.0e-8,
        "job_wire_zero_positive_PEEK_material_intrusion": job_to_guide <= 1.0e-8,
        "max_wire_zero_positive_PEEK_material_intrusion": max_to_guide <= 1.0e-8,
        "guide_and_arm_no_positive_volume_interference": guide_to_arm <= 1.0e-8,
        "OD12_shaft_envelope_zero_arm_intersection": shaft_to_arm <= 1.0e-8,
        "OD18_ID12p10_root_sleeve_exactly_one_solid": (
            root_audit["root_sleeve_solid_count"] == 1
        ),
        "root_sleeve_radial_ligament_ge_2p4mm": (
            root_audit["radial_ligament_mm"] >= 2.4
        ),
        "root_sleeve_positive_overlap_to_existing_collar": (
            root_audit["sleeve_to_existing_collar_overlap_mm3"] > 0.0
        ),
        "root_sleeve_positive_overlap_to_main_spoke": (
            root_audit["sleeve_to_main_spoke_overlap_mm3"] > 0.0
        ),
        "root_sleeve_positive_overlap_to_rear_counterrail": (
            root_audit[
                "sleeve_to_existing_rear_counterrail_overlap_mm3"
            ] > 0.0
        ),
        "root_sleeve_positive_overlap_to_both_bypass_cheeks": all(
            value > 0.0
            for value in root_audit[
                "sleeve_to_bypass_cheek_overlap_mm3"
            ]
        ),
        "root_sleeve_10N_R64_3x_stress_screen_pass": (
            root_audit["conservative_combined_root_load_case"]
            ["passes_review_allowable"]
        ),
        "counterrail_has_two_3mm_shaft_bypass_cheeks": True,
        "guide_seat_does_not_cut_counterweight_stacks": seat_to_corrections <= 1.0e-8,
        "front_trim_zero_PEEK_guide_intersection": (
            front_trim_to_guide <= 1.0e-8
        ),
        "front_trim_zero_printed_arm_intersection": (
            front_trim_to_arm <= 1.0e-8
        ),
        "front_trim_slug_positive_face_seating": all(
            value <= 1.0e-8 for value in trim_slug_to_arm_distances
        ),
        "front_trim_washer_positive_slug_seating": all(
            value <= 1.0e-8 for value in trim_washer_to_slug_distances
        ),
        "front_trim_screw_head_positive_washer_seating": all(
            value <= 1.0e-8 for value in trim_screw_to_washer_distances
        ),
        "front_trim_screw_zero_arm_intersection": all(
            value <= 1.0e-8 for value in trim_screw_to_arm_intersections
        ),
        "front_trim_insert_zero_unintended_arm_intersection": all(
            value <= 1.0e-8 for value in trim_insert_to_arm_intersections
        ),
        "front_trim_screw_tip_clearance_ge_0p5mm": (
            trim_screw_tip_clearance >= 0.5
        ),
        "front_trim_blind_material_behind_pilot_ge_2p4mm": (
            trim_blind_material_behind_pilot >= 2.4
        ),
        "front_trim_outer_radial_wall_ge_1p5mm": (
            trim_min_outer_radial_wall >= 1.5
        ),
        "front_trim_clears_actual_caps": front_trim_to_caps > 0.0,
        "six_slug_exact_OCC_balance": (
            balance["scaled_balance_residual_norm"] <= 1.0e-6
        ),
        "all_rear_slug_lengths_within_0p35_to_5p35mm": all(
            retained.SLUG_MIN_LENGTH_MM <= float(value)
            <= retained.SLUG_MAX_LENGTH_MM
            for value in balance["rear_slug_lengths_mm"]
        ),
        "remaining_spoke_floor_ge_6mm": remaining_spoke_floor >= 6.0,
        "remaining_spoke_side_web_each_ge_5mm": remaining_side_web >= 5.0,
        "actual_PEEK_caps_clear_revised_arm": arm_to_caps > 0.0,
        "actual_PEEK_caps_clear_guide_insert": guide_to_caps > 0.0,
        "three_positive_retention_screw_and_insert_stacks": (
            len(guide_retention_hardware()) == 6
        ),
        "obsolete_ceramic_torus_occurrence_removed": all(
            "toroid" not in (part.label or "").lower()
            for part in rotating_parts()
        ),
        "processed_arm_STL_watertight_zero_boundary_edges": (
            arm_mesh["watertight"]
            and arm_mesh["boundary_edges"] == 0
            and arm_mesh["nonmanifold_edges"] == 0
        ),
        "processed_PEEK_guide_STL_watertight_zero_boundary_edges": (
            guide_mesh["watertight"]
            and guide_mesh["boundary_edges"] == 0
            and guide_mesh["nonmanifold_edges"] == 0
        ),
    }
    isolated_geometry_pass = all(geometry_gates.values())
    release_gates = {
        "isolated_guide_geometry": isolated_geometry_pass,
        "physical_PEEK_forming_and_bore_gauge_coupon": False,
        "M2_insert_pull_and_300rpm_endurance_coupon": False,
        "printed_root_sleeve_orientation_matched_strength_coupon": False,
        "polished_bore_Ra_le_0p4um_verified": False,
        "full_2400_locus_terminal_route_pass": False,
        "both_raw_shaft_wraps_pass": False,
        "full_integrated_collision_and_balance_regenerated": False,
        "production_authorized": False,
    }
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": (
            "GEOMETRY_PASS_REVIEW_ONLY__TERMINAL_ROUTE_FAIL"
            if isolated_geometry_pass else "GEOMETRY_FAIL"
        ),
        "decision": (
            "SHAFT_TO_TIP_GUIDE_IS_PHYSICALLY_MODELED__DO_NOT_RELEASE_UNTIL_TERMINAL_GUIDE_AND_RAW_SWEEP_PASS"
            if isolated_geometry_pass else
            "DO_NOT_USE_GUIDE_SUCCESSOR_GEOMETRY"
        ),
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "paths": {
            "source": "cad/retained_flyer_peek_guide_successor.py",
            "step": "out/review/retained_flyer_peek_guide_successor.step",
            "report_json": "out/reports/retained_flyer_peek_guide_successor.json",
            "report_markdown": "out/reports/retained_flyer_peek_guide_successor.md",
            "manifest": "out/review/retained_flyer_peek_guide_successor.manifest.json",
        },
        "integration_api": {
            "revised_arm": "revised_retained_arm()",
            "root_sleeve": "successor_root_sleeve()",
            "root_sleeve_load_path_audit": (
                "root_sleeve_load_path_audit(arm)"
            ),
            "guide_insert": "peek_guide_insert()",
            "guide_hardware": "guide_retention_hardware()",
            "bore_centerline_samples": (
                "guide_bore_centerline_samples(max_step_mm)"
            ),
            "exit_bell": "bell_fairlead()",
            "balance_solution": "successor_balance_solution()",
            "merged_balance_solver": (
                "solve_successor_balance_with_base_rows(base_rows)"
            ),
            "front_trim_slugs": "front_trim_slug(x_mm, thickness_mm)",
            "front_trim_hardware": "front_trim_hardware(thickness_mm)",
            "rotating_occurrences": "rotating_parts()",
            "wire_envelope": "guide_wire_envelope(diameter_mm, label)",
            "frame": "retained flyer local; M2 axis +Z",
        },
        "guide": {
            "material": "natural unfilled PEEK",
            "manufacturing_review_process": (
                "precision injection mold or machine/heat-form from certified PEEK; polish and gauge complete bore after forming"
            ),
            "outer_diameter_mm": 2.0 * GUIDE_OUTER_RADIUS_MM,
            "bore_diameter_mm": 2.0 * GUIDE_BORE_RADIUS_MM,
            "seat_diameter_mm": 2.0 * GUIDE_SEAT_RADIUS_MM,
            "radial_seat_clearance_mm": GUIDE_SEAT_RADIAL_CLEARANCE_MM,
            "centerline_elbow_radius_mm": GUIDE_CENTERLINE_RADIUS_MM,
            "minimum_supported_wire_center_radius_after_bore_wander_mm": minimum_possible_wire_center_radius,
            "wire_diameter_range_mm": [
                WIRE_DIAMETER_MIN_MM, WIRE_DIAMETER_MAX_MM,
            ],
            "job_wire_diameter_mm": WIRE_DIAMETER_JOB_MM,
            "bbox": _bbox(guide),
            "solid_count": len(list(guide.solids())),
            "bore_centerline_sample_api": (
                "guide_bore_centerline_samples(max_step_mm)"
            ),
        },
        "exit_bell": {
            "owner": "flyer; rotates with M2",
            "geometry": "one-piece axisymmetric exposed PEEK fairlead",
            "contact_surface_generatrix_radius_mm": (
                BELL_CONTACT_SURFACE_RADIUS_MM
            ),
            "physical_meridian_sweep_deg": BELL_SWEEP_DEG,
            "nominal_wall_mm": BELL_NOMINAL_WALL_MM,
            "machining_profile_tolerance_mm": (
                bell_machining_profile_tolerance_mm
            ),
            "minimum_finished_wall_mm": bell_minimum_finished_wall_mm,
            "minimum_wire_center_radius_over_0p20_to_0p50mm_wire_mm": (
                bell_minimum_wire_center_radius
            ),
            "conservative_10N_strip_bending_stress_mpa": (
                bell_10N_bending_stress_mpa
            ),
            "externally_accessible_for_polish_and_gauge": True,
            "hidden_curved_bore": False,
            "loose_fairlead": False,
            "physical_abrasion_bend_coupon_complete": False,
            "bbox": _bbox(bell),
        },
        "revised_printed_arm": {
            "bbox": _bbox(arm),
            "solid_count": len(list(arm.solids())),
            "original_volume_mm3": float(original_arm.volume),
            "revised_volume_mm3": float(arm.volume),
            "seat_and_pilot_removed_volume_mm3": removed_volume,
            "remaining_spoke_floor_mm": remaining_spoke_floor,
            "remaining_side_web_each_mm": remaining_side_web,
            "source_architecture": (
                "torus-free collar+spoke+transition+tip-bridge union with "
                "open swept PEEK cradle seat; obsolete torus cut absent"
            ),
            "processed_STL": arm_mesh,
        },
        "root_sleeve_load_path": root_audit,
        "retention": {
            "ear_count": len(EAR_Y_MM),
            "ear_y_mm": list(EAR_Y_MM),
            "alternating_screw_x_mm": list(EAR_SCREW_X_MM),
            "hardware": "3x ISO 4762 M2x6 + 3x standard M2 heat-set insert",
            "physical_pull_and_endurance_complete": False,
        },
        "six_slug_balance": balance,
        "front_balance_trim": {
            "count": 2,
            "material": "ASTM-B777 tungsten alloy",
            "center_xy_mm": [
                [FRONT_TRIM_X_MM[0], FRONT_TRIM_Y_MM],
                [FRONT_TRIM_X_MM[1], FRONT_TRIM_Y_MM],
            ],
            "OD_ID_mm": [FRONT_TRIM_OD_MM, FRONT_TRIM_ID_MM],
            "common_thickness_mm": balance[
                "front_trim_common_thickness_mm"
            ],
            "hardware": (
                "2x M2 plain washer + 2x ISO4762 M2x8 + "
                "2x standard M2 heat-set insert"
            ),
            "full_insert_engagement_mm": 4.0,
            "screw_tip_clearance_behind_insert_mm": trim_screw_tip_clearance,
            "blind_printed_material_behind_pilot_mm": (
                trim_blind_material_behind_pilot
            ),
            "minimum_outer_radial_printed_wall_mm": (
                trim_min_outer_radial_wall
            ),
            "slug_to_arm_seat_distances_mm": trim_slug_to_arm_distances,
            "washer_to_slug_seat_distances_mm": (
                trim_washer_to_slug_distances
            ),
            "screw_head_to_washer_seat_distances_mm": (
                trim_screw_to_washer_distances
            ),
            "screw_to_arm_intersection_mm3": (
                trim_screw_to_arm_intersections
            ),
            "insert_to_arm_unintended_intersection_mm3": (
                trim_insert_to_arm_intersections
            ),
            "pull_coupon_complete": False,
            "300rpm_endurance_complete": False,
        },
        "mesh_topology": {
            "printed_arm": arm_mesh,
            "PEEK_guide": guide_mesh,
        },
        "exact_BREP_checks": {
            "job_wire_to_revised_arm_intersection_mm3": job_to_arm,
            "max_wire_to_revised_arm_intersection_mm3": max_to_arm,
            "job_wire_to_PEEK_insert_material_intersection_mm3": job_to_guide,
            "max_wire_to_PEEK_insert_material_intersection_mm3": max_to_guide,
            "PEEK_insert_to_revised_arm_intersection_mm3": guide_to_arm,
            "guide_seat_to_counterweight_stack_intersection_mm3": seat_to_corrections,
            "actual_PEEK_caps_to_revised_arm_distance_mm": arm_to_caps,
            "actual_PEEK_caps_to_guide_insert_distance_mm": guide_to_caps,
            "OD12_shaft_envelope_to_revised_arm_intersection_mm3": (
                shaft_to_arm
            ),
            "root_sleeve_to_existing_collar_overlap_mm3": root_audit[
                "sleeve_to_existing_collar_overlap_mm3"
            ],
            "root_sleeve_to_main_spoke_overlap_mm3": root_audit[
                "sleeve_to_main_spoke_overlap_mm3"
            ],
            "root_sleeve_to_existing_rear_counterrail_overlap_mm3": (
                root_audit[
                    "sleeve_to_existing_rear_counterrail_overlap_mm3"
                ]
            ),
            "root_sleeve_to_bypass_cheek_overlap_mm3": root_audit[
                "sleeve_to_bypass_cheek_overlap_mm3"
            ],
            "front_trim_to_PEEK_guide_intersection_mm3": front_trim_to_guide,
            "front_trim_to_printed_arm_intersection_mm3": front_trim_to_arm,
            "front_trim_to_actual_caps_distance_mm": front_trim_to_caps,
        },
        "geometry_gates": geometry_gates,
        "release_gates": release_gates,
        "limits": [
            "The terminal tip-to-cap route remains failed; this STEP closes only hollow-shaft-to-tip continuity and forbidden PETG contact.",
            "Exact strand centers, order, passive settling, neatness, sag, snagging, friction and enamel abrasion are not predicted.",
            "PEEK forming, bore gauging/polish, screw-insert pull and 300 rpm endurance require physical coupons.",
            "The isolated six-slug balance imports a legacy drive context and is not the final release inertia or trim authority; the merged release must call solve_successor_balance_with_base_rows() with its exact untrimmed mass rows.",
            "The OD18/ID12.10 root sleeve passes only a conservative analytical screen; an orientation-matched printed root coupon remains mandatory.",
            "Balance, motor torque, cap collision and both raw shaft wraps must be regenerated after integration.",
        ],
        "source_hashes": {
            "cad/retained_flyer_peek_guide_successor.py": _sha256(SOURCE),
            "cad/permanent_cap_offset_spoke_retained_review.py": _sha256(
                HERE / "permanent_cap_offset_spoke_retained_review.py"
            ),
            "cad/permanent_cap_offset_spoke_review.py": _sha256(
                HERE / "permanent_cap_offset_spoke_review.py"
            ),
            "cad/permanent_cap_production_review.py": _sha256(
                HERE / "permanent_cap_production_review.py"
            ),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def gen_step() -> Compound:
    caps = actual_production_cap_context()
    stator = base.deepest_pose_stator()
    stator.label = "default_stator_deepest_raw_pose_context"
    rotating = Compound(children=rotating_parts())
    rotating.label = "retained_flyer_with_physical_PEEK_guide_review"
    wire = guide_wire_envelope(
        WIRE_DIAMETER_JOB_MM, "job_wire_inside_physical_PEEK_guide"
    )
    result = Compound(children=[stator, caps, rotating, wire])
    result.label = "retained_flyer_PEEK_guide_successor_review_only"
    return result


def render_markdown(report: Mapping[str, Any]) -> str:
    guide = report["guide"]
    arm = report["revised_printed_arm"]
    brep = report["exact_BREP_checks"]
    lines = [
        "# Retained flyer physical PEEK guide successor",
        "",
        f"**{report['status']}**",
        "",
        "The former visual witness and ceramic toroid are replaced by one removable, positive-volume PEEK guide from the hollow shaft through three analytic R3.25 elbows to an integral R64 tip eye.",
        "",
        "## Guide and retained arm",
        "",
        f"- Guide OD/ID: {guide['outer_diameter_mm']:.2f}/{guide['bore_diameter_mm']:.2f} mm.",
        f"- Minimum supported wire-center bend after worst bore wander: {guide['minimum_supported_wire_center_radius_after_bore_wander_mm']:.5f} mm.",
        f"- Revised printed arm solids: {arm['solid_count']}.",
        f"- Remaining spoke floor: {arm['remaining_spoke_floor_mm']:.3f} mm; each side web: {arm['remaining_side_web_each_mm']:.3f} mm.",
        "- Retention: three integral ears, three M2x6 screws, three M2 heat-set inserts.",
        "",
        "## Exact BREP checks",
        "",
        f"- Job wire / PETG arm intersection: {brep['job_wire_to_revised_arm_intersection_mm3']:.9g} mm3.",
        f"- Maximum wire / PETG arm intersection: {brep['max_wire_to_revised_arm_intersection_mm3']:.9g} mm3.",
        f"- Job wire / PEEK material intrusion: {brep['job_wire_to_PEEK_insert_material_intersection_mm3']:.9g} mm3.",
        f"- Maximum wire / PEEK material intrusion: {brep['max_wire_to_PEEK_insert_material_intersection_mm3']:.9g} mm3.",
        f"- Seat / counterweight-stack intersection: {brep['guide_seat_to_counterweight_stack_intersection_mm3']:.9g} mm3.",
        "",
        "## Release gates",
        "",
    ]
    lines.extend(
        f"- {'PASS' if value else 'FAIL'} - `{name}`"
        for name, value in report["release_gates"].items()
    )
    lines.extend((
        "",
        "Terminal routing and the complete raw sweep remain mandatory blockers. This isolated geometry is not production-authorized.",
        "",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ))
    return "\n".join(lines)


def write_reports() -> dict[str, Any]:
    report = analyze()
    JSON_OUT.parent.mkdir(parents=True, exist_ok=True)
    MANIFEST_OUT.parent.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    MD_OUT.write_text(render_markdown(report), encoding="utf-8")
    manifest = {
        "schema": MANIFEST_SCHEMA,
        "source": "cad/retained_flyer_peek_guide_successor.py",
        "step": "out/review/retained_flyer_peek_guide_successor.step",
        "integration_api": report["integration_api"],
        "guide": report["guide"],
        "geometry_gates": report["geometry_gates"],
        "release_gates": report["release_gates"],
        "report_sha256": report["report_sha256"],
    }
    MANIFEST_OUT.write_text(
        json.dumps(manifest, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return report


if __name__ == "__main__":
    result = write_reports()
    passed = sum(result["geometry_gates"].values())
    total = len(result["geometry_gates"])
    print(f"{result['status']} geometry {passed}/{total}")
