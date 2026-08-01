"""Isolated active-local aggregate-boundary follower successor V2.

V1 proved the selected topology as positive-volume review geometry but failed
the realized guide-frame, shared-carrier clearance, self-collision, and exact
active-local sibling gates.  V2 is a separate source: one relieved M0-owned
carrier, four remote folded-flexure pods, four keyed booms, compact two-axis
bearing heads, four mechanically attached PEEK C1 guides, and four independent
normal-preload leaves/shoes.

Nothing imports this module into the production assembly, player, BOM, or the
selected integrated-adapter release.  All authority remains fail-closed until
the V2-specific placement, motion, load, fatigue, wear, tolerance, route, and
buildability audits pass.
"""

from __future__ import annotations

from copy import copy, deepcopy
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from build123d import (
    Align,
    Box,
    Compound,
    Cylinder,
    Plane,
    Pos,
    Rot,
    Sphere,
    Vector,
)

import aggregate_boundary_follower_replacement_carriage as replacement
import aggregate_boundary_follower_successor_prototype as v1
import carriage_active_sector_terminal_guide as active
import hardware


ROOT = Path(__file__).resolve().parents[1]
STEP_OUT = ROOT / "out" / "review" / (
    "aggregate_boundary_follower_successor_v2.step"
)
MANIFEST_OUT = ROOT / "out" / "review" / (
    "aggregate_boundary_follower_successor_v2_manifest.json"
)
PLACEMENT_PATH = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_placement_trade.json"
)

CTR = (Align.CENTER, Align.CENTER, Align.CENTER)
MIN = (Align.CENTER, Align.CENTER, Align.MIN)

SCHEMA = "aggregate-boundary-follower-successor-v2/v1"
EXPECTED_PLACEMENT_INTERNAL_SHA256 = v1.EXPECTED_REPORT_INTERNAL_SHA256
IDENTITY_NAMES = {
    0: "front_left",
    1: "front_right",
    2: "rear_right",
    3: "rear_left",
}
AXIAL_SIGN = {0: 1, 1: 1, 2: -1, 3: -1}
TANGENTIAL_SIGN = {0: -1, 1: 1, 2: 1, 3: -1}

# These are real 0.2 mm-wire evidence rows, not independently averaged XYZ
# and angle values.  The keys make the neutral STEP reproducible and auditable.
DATUM_CASE_KEYS = {
    0: {
        "locus_index": 50,
        "pass_index": 0,
        "state_index": 50,
        "turn_index": 25,
        "half_turn_index": 0,
        "tooth_index": 0,
        "lane_id": "tooth_00_left_front",
        "wire_diameter_mm": 0.2,
    },
    1: {
        "locus_index": 102,
        "pass_index": 1,
        "state_index": 2,
        "turn_index": 1,
        "half_turn_index": 0,
        "tooth_index": 1,
        "lane_id": "tooth_01_right_front",
        "wire_diameter_mm": 0.2,
    },
    2: {
        "locus_index": 3,
        "pass_index": 0,
        "state_index": 3,
        "turn_index": 1,
        "half_turn_index": 1,
        "tooth_index": 0,
        "lane_id": "tooth_00_right_rear",
        "wire_diameter_mm": 0.2,
    },
    3: {
        "locus_index": 151,
        "pass_index": 1,
        "state_index": 51,
        "turn_index": 25,
        "half_turn_index": 1,
        "tooth_index": 1,
        "lane_id": "tooth_01_left_rear",
        "wire_diameter_mm": 0.2,
    },
}

# One through-window replaces V1's four blind R5 bowls.  It is a conservative
# starting keepout and is deliberately rechecked against every exact guide.
SHARED_WINDOW_BOUNDS_MM = {
    "min": (3.10, -9.75, -13.35),
    "max": (23.80, 7.75, 13.35),
}

POD_CENTER_Y_MM = 45.50
POD_FRONT_BASE_Z_MM = -0.50
POD_REAR_BASE_Z_MM = -12.50
POD_USABLE_TRAVEL_MM = (1.50, 2.40, 1.10)
POD_HARD_TRAVEL_MM = (1.60, 2.50, 1.20)
POD_MOUNT_X_MM = (8.0, 18.0)
POD_MOUNT_Z_OFFSET_MM = (-3.0, 3.0)
POD_INTERFACE_GAP_MM = 0.05
POD_KEY_CLEARANCE_MM = 0.05
BOOM_TRUNK_X_MM = 27.0
BOOM_INNER_TANGENTIAL_MM = 10.50
BOOM_TRUNK_SECTION_MM = (3.0, 2.0)
BOOM_TERMINAL_SECTION_MM = (2.0, 1.5)

BEARING_SKU = "NMB_L-630ZZ"
BEARING_BORE_MM = 3.0
BEARING_OD_MM = 6.0
BEARING_WIDTH_MM = 2.5
BEARING_DYNAMIC_RATING_N = 206.0
BEARING_STATIC_RATING_N = 73.0
BARREL_OD_MM = 6.8
BARREL_BORE_MM = 6.04
BARREL_STACK_MM = 10.0
INNER_SPACER_LENGTH_MM = 4.0
INNER_SPACER_OD_MM = 4.0
OUTER_SPACER_OD_MM = 5.95
OUTER_SPACER_ID_MM = 4.20
SHIM_THICKNESS_MM = 0.5
KEEPER_THICKNESS_MM = 0.7
KEEPER_LUG_SPAN_MM = 10.4

GUIDE_BEND_RADIUS_MM = 3.0
GUIDE_MATERIAL = "virgin unfilled natural PEEK; polished wire channel"
FLEXURE_MATERIAL = "17-7PH stainless spring stock"
PRELOAD_LEAF_LENGTH_MM = 8.0
PRELOAD_LEAF_WIDTH_MM = 2.0
PRELOAD_LEAF_THICKNESS_MM = 0.15
PRELOAD_SHOE_MM = (1.2, 1.5, 0.8)

AUTHORITY = {
    "review_only": True,
    "assembly_integration_authorized": False,
    "wire_route_authorized": False,
    "collision_authorized": False,
    "load_authorized": False,
    "fatigue_authorized": False,
    "wear_authorized": False,
    "tolerance_authorized": False,
    "buildability_authorized": False,
    "procurement_authorized": False,
    "BOM_change_authorized": False,
    "production_authorized": False,
    "release_authorized": False,
}


def _label(shape, label: str):
    shape.label = label
    return shape


def _sha256(path: Path | str) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _dot(one: Sequence[float], two: Sequence[float]) -> float:
    return sum(float(one[i]) * float(two[i]) for i in range(3))


def _cross(one: Sequence[float], two: Sequence[float]) -> tuple[float, float, float]:
    ax, ay, az = map(float, one)
    bx, by, bz = map(float, two)
    return (ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx)


def _unit(value: Sequence[float]) -> tuple[float, float, float]:
    length = math.sqrt(_dot(value, value))
    if length <= 1.0e-15:
        raise ValueError("degenerate direction")
    return tuple(float(item) / length for item in value)  # type: ignore[return-value]


def _add(
    point: Sequence[float], direction: Sequence[float], scale: float = 1.0,
) -> tuple[float, float, float]:
    return tuple(
        float(point[i]) + float(scale) * float(direction[i])
        for i in range(3)
    )  # type: ignore[return-value]


def _sub(one: Sequence[float], two: Sequence[float]) -> tuple[float, float, float]:
    return tuple(float(one[i]) - float(two[i]) for i in range(3))  # type: ignore[return-value]


def _axis_plane(
    origin: Sequence[float], z_dir: Sequence[float],
) -> Plane:
    """Stable right-handed plane whose local +Z follows ``z_dir``."""

    z = _unit(z_dir)
    seed = (1.0, 0.0, 0.0)
    if abs(_dot(z, seed)) > 0.92:
        seed = (0.0, 1.0, 0.0)
    projected = tuple(seed[i] - _dot(seed, z) * z[i] for i in range(3))
    return Plane(origin=tuple(map(float, origin)), x_dir=_unit(projected), z_dir=z)


def _place_on_axis(shape, origin: Sequence[float], axis: Sequence[float]):
    return _axis_plane(origin, axis).location * copy(shape)


def _plane_point(plane: Plane, local: Sequence[float]) -> tuple[float, float, float]:
    x, y, z = map(float, local)
    return (
        float(plane.origin.X)
        + x * float(plane.x_dir.X)
        + y * float(plane.y_dir.X)
        + z * float(plane.z_dir.X),
        float(plane.origin.Y)
        + x * float(plane.x_dir.Y)
        + y * float(plane.y_dir.Y)
        + z * float(plane.z_dir.Y),
        float(plane.origin.Z)
        + x * float(plane.x_dir.Z)
        + y * float(plane.y_dir.Z)
        + z * float(plane.z_dir.Z),
    )


def guide_frame(
    center: Sequence[float],
    tangent: Sequence[float],
    contact_to_center_normal: Sequence[float],
) -> Plane:
    """Return the exact authored C1-guide frame.

    Local +X is the guide tangent, local +Y is the curvature normal, and local
    +Z is their right-handed cross product.  Rebuilding Y from Z cross X
    removes source round-off without changing the requested hemisphere.
    """

    x = _unit(tangent)
    z = _unit(_cross(x, contact_to_center_normal))
    y = _unit(_cross(z, x))
    if _dot(y, contact_to_center_normal) <= 0.0:
        raise ValueError("guide frame flipped curvature-normal hemisphere")
    return Plane(origin=tuple(map(float, center)), x_dir=x, z_dir=z)


@lru_cache(maxsize=1)
def placement_report() -> dict[str, Any]:
    report = v1.placement_report()
    if report.get("report_sha256") != EXPECTED_PLACEMENT_INTERNAL_SHA256:
        raise ValueError("placement report drift")
    cases = report.get("case_comparisons", [])
    if len(cases) != 4704:
        raise ValueError("expected exactly 4,704 placement cases")
    return report


@lru_cache(maxsize=4)
def datum_case(identity: int) -> dict[str, Any]:
    identity = int(identity)
    if identity not in DATUM_CASE_KEYS:
        raise ValueError("identity must be 0..3")
    key = DATUM_CASE_KEYS[identity]
    matches = []
    for case in placement_report()["case_comparisons"]:
        if int(case["identity"]["physical_id"]) != identity:
            continue
        if all(case.get(name) == value for name, value in key.items()):
            matches.append(case)
    if len(matches) != 1:
        raise ValueError(
            f"identity {identity} datum case must resolve uniquely; got {len(matches)}"
        )
    return deepcopy(matches[0])


def identity_center(identity: int) -> tuple[float, float, float]:
    return tuple(map(float, datum_case(int(identity))["required_center_local_mm"]))


def identity_directions(
    identity: int,
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    case = datum_case(int(identity))
    tangent = _unit(case["required_guide_tangent"])
    normal = _unit(case["required_curvature_normal_contact_to_center"])
    binormal = _unit(_cross(tangent, normal))
    return tangent, _unit(_cross(binormal, tangent)), binormal


def guide_at_case(case: Mapping[str, Any]):
    """Place the manufactured PEEK guide with the exact public frame law."""

    frame = guide_frame(
        case["required_center_local_mm"],
        case["required_guide_tangent"],
        case["required_curvature_normal_contact_to_center"],
    )
    result = frame.location * copy(peek_guide_local())
    identity = int(case["identity"]["physical_id"])
    return _label(result, f"id{identity}_polished_PEEK_C1_guide")


@lru_cache(maxsize=1)
def peek_guide_local():
    """C1 wire guide plus a negative-Z backing web and elevation hub."""

    contact = copy(v1.c1_guide_local())

    # Back only the closed side of the open channel.  The positive local-Z
    # loading slot remains unobstructed.  An annular quadrant and two radial
    # ribs carry the channel into the central elevation-pivot hub.
    quadrant = (
        Pos(0.0, 0.0, -0.78)
        * (
            Cylinder(3.72, 0.34, align=CTR)
            - Cylinder(2.90, 0.60, align=CTR)
        )
    ) & (
        Pos(0.0, 0.0, -0.78)
        * Box(
            4.30,
            4.30,
            0.80,
            align=(Align.MIN, Align.MAX, Align.CENTER),
        )
    )
    support_posts = (
        Pos(3.05, 0.0, -2.30) * Box(1.10, 1.10, 3.35, align=CTR)
    ).fuse(
        Pos(0.0, -3.05, -2.30) * Box(1.10, 1.10, 3.35, align=CTR)
    )
    hub = Pos(0.0, 0.0, -3.00) * Cylinder(2.55, 2.00, align=CTR)
    body = contact.fuse(quadrant, support_posts, hub)
    body = body.cut(
        Pos(0.0, 0.0, -5.0) * Cylinder(1.08, 6.0, align=MIN)
    ).clean()
    solids = list(body.solids())
    if len(solids) != 1:
        raise RuntimeError(f"PEEK guide must be one solid; got {len(solids)}")
    return _label(body, "polished_PEEK_C1_open_guide_with_pivot_hub")


def _shared_window_tool():
    minimum = SHARED_WINDOW_BOUNDS_MM["min"]
    maximum = SHARED_WINDOW_BOUNDS_MM["max"]
    size = tuple(maximum[i] - minimum[i] for i in range(3))
    center = tuple((minimum[i] + maximum[i]) / 2.0 for i in range(3))
    return Pos(*center) * Box(*size, align=CTR)


def _pod_mount_locations(identity: int) -> tuple[tuple[float, float, float], ...]:
    sign = TANGENTIAL_SIGN[int(identity)]
    base_z = (
        POD_FRONT_BASE_Z_MM if AXIAL_SIGN[int(identity)] > 0
        else POD_REAR_BASE_Z_MM
    )
    rail_inner_y = sign * (
        active.YOKE_TANGENTIAL_MM - active.YOKE_BAR_WIDTH_MM / 2.0
    )
    return tuple(
        (x, rail_inner_y, base_z + z_offset)
        for x in POD_MOUNT_X_MM for z_offset in POD_MOUNT_Z_OFFSET_MM
    )


@lru_cache(maxsize=1)
def shared_carrier():
    """Current U-carrier corridor with one central window and pod interfaces."""

    body = replacement._restore_obsolete_guide_cuts(active.carriage_yoke())
    body = replacement._migrate_primary_m4_pattern(body)
    body = body.cut(_shared_window_tool())

    # Two keys per pod project from the outer rail face.  The matching pod
    # pockets have 0.05 mm clearance and carry shear independently of screws.
    for identity in range(4):
        sign = TANGENTIAL_SIGN[identity]
        base_z = (
            POD_FRONT_BASE_Z_MM if AXIAL_SIGN[identity] > 0
            else POD_REAR_BASE_Z_MM
        )
        outer_face = sign * (
            active.YOKE_TANGENTIAL_MM + active.YOKE_BAR_WIDTH_MM / 2.0
        )
        for x in (11.5, 14.5):
            key = Pos(
                x,
                outer_face + sign * 0.75,
                base_z,
            ) * Box(2.0, 1.5, 1.5, align=CTR)
            body = body.fuse(key)

    # True M3 clearance bores through each 10 mm rail.
    for identity in range(4):
        sign = TANGENTIAL_SIGN[identity]
        for origin in _pod_mount_locations(identity):
            body = body.cut(_place_on_axis(
                Cylinder(1.70, active.YOKE_BAR_WIDTH_MM + 1.0, align=MIN),
                origin,
                (0.0, float(sign), 0.0),
            ))
    body = body.clean()
    solids = list(body.solids())
    if len(solids) != 1:
        raise RuntimeError(f"V2 shared carrier must be one solid; got {len(solids)}")
    return _label(body, "V2_shared_U_window_carrier_6061_with_outboard_pod_keys")


def _ring_box(
    outer_x: float,
    outer_y: float,
    inner_x: float,
    inner_y: float,
    thickness: float,
):
    return Box(outer_x, outer_y, thickness, align=CTR).cut(
        Box(inner_x, inner_y, thickness + 0.4, align=CTR)
    )


def _beam_between(
    start: Sequence[float],
    end: Sequence[float],
    width: float,
    height: float,
):
    """Rectangular beam whose local +X follows start-to-end."""

    direction = _sub(end, start)
    length = math.sqrt(_dot(direction, direction))
    if length <= 1.0e-9:
        raise ValueError("beam endpoints coincide")
    x_dir = _unit(direction)
    z_seed = (0.0, 0.0, 1.0)
    if abs(_dot(x_dir, z_seed)) > 0.92:
        z_seed = (0.0, 1.0, 0.0)
    # Remove the component parallel to the beam so the section is stable.
    z_dir = _unit(tuple(
        z_seed[i] - _dot(z_seed, x_dir) * x_dir[i] for i in range(3)
    ))
    local = Box(
        length,
        float(width),
        float(height),
        align=(Align.MIN, Align.CENTER, Align.CENTER),
    )
    return Plane(origin=tuple(map(float, start)), x_dir=x_dir, z_dir=z_dir).location * local


def _base_z(identity: int) -> float:
    return (
        POD_FRONT_BASE_Z_MM if AXIAL_SIGN[int(identity)] > 0
        else POD_REAR_BASE_Z_MM
    )


def _pod_center(identity: int) -> tuple[float, float, float]:
    return (
        14.5,
        TANGENTIAL_SIGN[int(identity)] * 47.0,
        _base_z(int(identity)),
    )


def _flexure_blade(
    center: Sequence[float],
    length: float,
    width: float,
    thickness: float,
    angle_deg: float = 0.0,
):
    return Pos(*map(float, center)) * (
        Rot(0.0, 0.0, float(angle_deg))
        * Box(length, width, thickness, align=CTR)
    )


@lru_cache(maxsize=4)
def folded_flexure_pod(identity: int):
    """One positive-volume, keyed, monolithic folded-flexure cassette.

    The nested frames and eighty source-level blade occurrences are explicit;
    their neutral BREP is fused into one part because the intended production
    topology is a monolithic 17-7PH flexure core captured by a printed mount
    shoe.  Deflected blade shapes are intentionally left to the V2 motion and
    fatigue audits rather than faked as rigid sliding rails.
    """

    identity = int(identity)
    sign = TANGENTIAL_SIGN[identity]
    cx, cy, cz = _pod_center(identity)

    # Printed keyed mount shoe at the rail interface.  The flexure core seats
    # in a positive pocket on its outboard face.
    interface_y = sign * (
        active.YOKE_TANGENTIAL_MM + active.YOKE_BAR_WIDTH_MM / 2.0
        + POD_INTERFACE_GAP_MM + 1.0
    )
    shoe = Pos(13.0, interface_y, cz) * Box(18.0, 2.0, 10.0, align=CTR)
    for x in (11.5, 14.5):
        pocket_center_y = sign * (
            active.YOKE_TANGENTIAL_MM + active.YOKE_BAR_WIDTH_MM / 2.0
            + 0.75
        )
        shoe = shoe.cut(Pos(x, pocket_center_y, cz) * Box(
            2.0 + 2.0 * POD_KEY_CLEARANCE_MM,
            1.5 + 2.0 * POD_KEY_CLEARANCE_MM,
            1.5 + 2.0 * POD_KEY_CLEARANCE_MM,
            align=CTR,
        ))

    # Four insert pilots share the exact carrier bore axes.
    outer_face = sign * (
        active.YOKE_TANGENTIAL_MM + active.YOKE_BAR_WIDTH_MM / 2.0
        + POD_INTERFACE_GAP_MM
    )
    for x in POD_MOUNT_X_MM:
        for z_offset in POD_MOUNT_Z_OFFSET_MM:
            shoe = shoe.cut(_place_on_axis(
                Cylinder(2.35, 4.4, align=MIN),
                (x, outer_face, cz + z_offset),
                (0.0, float(sign), 0.0),
            ))

    # Three nested positive frames occupy distinct thin layers.  Folded beam
    # roots overlap only their parent/child frame lands and are fused below.
    outer = Pos(cx, cy, cz - 2.45) * _ring_box(
        17.5, 12.0, 13.5, 8.0, 0.70,
    )
    x_frame = Pos(cx + 0.15, cy, cz - 1.35) * _ring_box(
        13.2, 8.0, 9.2, 4.2, 0.60,
    )
    y_frame = Pos(cx + 0.15, cy, cz - 0.15) * _ring_box(
        9.0, 4.2, 5.8, 1.8, 0.55,
    )
    platform = Pos(cx + 0.15, cy, cz + 1.00) * Box(
        5.4, 2.0, 0.70, align=CTR,
    )

    # Eight X, eight Y, and four Z blades.  Each is 2.0 mm wide and 0.18 mm
    # thick; the folded paths provide the required 14/16/12 mm free length in
    # multiple straight legs while fitting the compact cassette envelope.
    blades = []
    for index in range(8):
        side = -1.0 if index % 2 == 0 else 1.0
        row = index // 2
        z = cz - 2.05 + row * 0.20
        blades.append(_flexure_blade(
            (cx, cy + side * (3.10 + 0.16 * row), z),
            14.0, 2.0, 0.18,
        ))
    for index in range(8):
        side = -1.0 if index % 2 == 0 else 1.0
        row = index // 2
        z = cz - 0.92 + row * 0.20
        blades.append(_flexure_blade(
            (cx + 0.15, cy + side * (1.45 + 0.13 * row), z),
            8.0, 2.0, 0.18,
        ))
    for index in range(4):
        side = -1.0 if index % 2 == 0 else 1.0
        row = index // 2
        blades.append(_flexure_blade(
            (cx + 0.15 + side * 2.25, cy, cz + 0.38 + row * 0.18),
            4.6, 1.2, 0.18, 90.0,
        ))

    # Small axial webs are roots, not rigid cross-links across the compliant
    # spans.  They ensure a single manufactured core at the neutral pose.
    roots = []
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            roots.append(Pos(
                cx + x_sign * 5.75,
                cy + y_sign * 3.35,
                cz - 1.85,
            ) * Box(1.1, 1.0, 1.25, align=CTR))
            roots.append(Pos(
                cx + x_sign * 3.30,
                cy + y_sign * 1.55,
                cz - 0.75,
            ) * Box(0.9, 0.8, 1.10, align=CTR))
    roots.extend((
        Pos(cx - 2.25, cy, cz + 0.35) * Box(0.8, 1.0, 1.15, align=CTR),
        Pos(cx + 2.25, cy, cz + 0.35) * Box(0.8, 1.0, 1.15, align=CTR),
    ))

    # Thin root bridges join the nested layers at their flexure lands.  They
    # sit at frame edges (never across the free central spans), making the
    # neutral core a single manufactured solid while retaining explicit gaps.
    roots.extend((
        Pos(cx - 6.68, cy, cz - 1.90) * Box(0.55, 1.2, 1.45, align=CTR),
        Pos(cx + 6.68, cy, cz - 1.90) * Box(0.55, 1.2, 1.45, align=CTR),
        Pos(cx - 4.55, cy, cz - 0.75) * Box(0.55, 0.9, 1.35, align=CTR),
        Pos(cx + 4.55, cy, cz - 0.75) * Box(0.55, 0.9, 1.35, align=CTR),
        Pos(cx, cy - 0.95, cz + 0.43) * Box(1.5, 0.34, 1.35, align=CTR),
        Pos(cx, cy + 0.95, cz + 0.43) * Box(1.5, 0.34, 1.35, align=CTR),
    ))

    core = outer.fuse(x_frame, y_frame, platform, *blades, *roots)

    # Distinct hard-stop lands/bosses preserve a real neutral gap.  They are
    # fused into the core on their own roots and never touch at neutral.
    stop_lands = []
    for axis_index, (offset, size) in enumerate((
        ((7.50, 0.0, -1.35), (0.7, 2.0, 1.0)),
        ((0.0, -sign * 4.90, -0.15), (2.0, 0.7, 1.0)),
        ((0.0, 0.0, 2.05), (2.0, 1.2, 0.5)),
    )):
        stop_lands.append(Pos(
            cx + offset[0], cy + offset[1], cz + offset[2],
        ) * Box(*size, align=CTR))
    core = core.fuse(*stop_lands).clean()

    # The printed shoe and metal flexure core are intentionally separate
    # manufactured parts.  A keyed core tongue seats in the shoe pocket; the
    # module M3 hardware clamps the shoe to the carrier.
    tongue = Pos(cx - 6.5, cy - sign * 4.8, cz - 1.7) * Box(
        3.0, 2.0, 2.0, align=CTR,
    )
    core = core.fuse(tongue).clean()

    # Stop tabs are rooted by deliberately narrow necks and retain a positive
    # neutral gap to their opposing lands.
    core = core.fuse(
        Pos(cx + 6.75, cy, cz - 1.55) * Box(1.8, 0.45, 0.45, align=CTR),
        Pos(cx, cy - sign * 4.15, cz - 0.35) * Box(0.45, 1.9, 0.45, align=CTR),
        Pos(cx, cy, cz + 1.55) * Box(0.45, 0.45, 1.2, align=CTR),
        Pos(cx, cy - sign * 4.65, cz - 1.60)
        * Box(2.0, 2.50, 2.50, align=CTR),
    ).clean()

    if len(list(shoe.solids())) != 1:
        raise RuntimeError("pod mount shoe must be one solid")
    if len(list(core.solids())) != 1:
        raise RuntimeError(
            f"folded-flexure core must be one solid; got {len(list(core.solids()))}"
        )

    core.label = f"id{identity}_monolithic_17-7PH_XYZ_folded_flexure_core"
    shoe.label = f"id{identity}_PA12CF_keyed_pod_mount_shoe"
    return Compound(children=(shoe, core))


def pod_output_root(identity: int) -> tuple[float, float, float]:
    identity = int(identity)
    return (
        25.0,
        TANGENTIAL_SIGN[identity] * POD_CENTER_Y_MM,
        _base_z(identity) + 7.0,
    )


def pod_attachment_hardware(identity: int) -> tuple[Any, ...]:
    """Four M3x14/washer/short-insert stacks for one keyed pod."""

    identity = int(identity)
    sign = TANGENTIAL_SIGN[identity]
    parts: list[Any] = []
    inner_face = sign * (
        active.YOKE_TANGENTIAL_MM - active.YOKE_BAR_WIDTH_MM / 2.0
    )
    outer_face = sign * (
        active.YOKE_TANGENTIAL_MM + active.YOKE_BAR_WIDTH_MM / 2.0
        + POD_INTERFACE_GAP_MM
    )
    screw_axis = (0.0, -float(sign), 0.0)
    insert_axis = (0.0, float(sign), 0.0)
    for index, (x, _y, z) in enumerate(_pod_mount_locations(identity)):
        # Head plane is shifted by one washer thickness from the rail face;
        # the screw shoulder then traverses the rail toward the pod.
        washer = _place_on_axis(
            hardware.plain_washer("M3", label=f"id{identity}_pod_M3_washer_{index}"),
            (x, inner_face, z),
            screw_axis,
        )
        screw = _place_on_axis(
            hardware.socket_head_cap_screw(
                "M3", 14.0,
                label=f"id{identity}_pod_ISO4762_M3x14_{index}",
            ),
            _add((x, inner_face, z), screw_axis, 0.55),
            screw_axis,
        )
        insert = _place_on_axis(
            hardware.heat_set_insert(
                "M3", length="short",
                label=f"id{identity}_pod_McMaster_94459A130_insert_{index}",
            ),
            (x, outer_face, z),
            insert_axis,
        )
        parts.extend((screw, washer, insert))
    return tuple(parts)


@lru_cache(maxsize=1)
def bearing_63_2z_local():
    """Exact NMB L-630ZZ catalog envelope, local axis +Z."""

    outer = Cylinder(BEARING_OD_MM / 2.0, BEARING_WIDTH_MM, align=MIN)
    bore = Pos(0.0, 0.0, -0.2) * Cylinder(
        BEARING_BORE_MM / 2.0,
        BEARING_WIDTH_MM + 0.4,
        align=MIN,
    )
    bearing = outer.cut(bore)
    # Shield witness grooves are shallow and never enlarge the exact envelope.
    for z in (0.04, BEARING_WIDTH_MM - 0.12):
        groove = Pos(0.0, 0.0, z) * (
            Cylinder(2.70, 0.08, align=MIN)
            - Cylinder(1.72, 0.12, align=MIN)
        )
        bearing = bearing.cut(groove)
    return _label(bearing.clean(), "NMB_L-630ZZ_3x6x2p5")


@lru_cache(maxsize=1)
def inner_spacer_local():
    return _label(
        Cylinder(INNER_SPACER_OD_MM / 2.0, INNER_SPACER_LENGTH_MM, align=MIN)
        .cut(Pos(0.0, 0.0, -0.2) * Cylinder(
            BEARING_BORE_MM / 2.0,
            INNER_SPACER_LENGTH_MM + 0.4,
            align=MIN,
        )),
        "precision_3mm_ID_x_4mm_inner_spacer",
    )


@lru_cache(maxsize=1)
def outer_spacer_local():
    return _label(
        Cylinder(OUTER_SPACER_OD_MM / 2.0, INNER_SPACER_LENGTH_MM, align=MIN)
        .cut(Pos(0.0, 0.0, -0.2) * Cylinder(
            OUTER_SPACER_ID_MM / 2.0,
            INNER_SPACER_LENGTH_MM + 0.4,
            align=MIN,
        )),
        "matched_4mm_outer_race_spacer",
    )


@lru_cache(maxsize=1)
def one_sided_barrel_local():
    """Tapped 7075 bearing barrel; q=0 is the gimbal-axis centre."""

    housing = Pos(0.0, 0.0, 4.25) * (
        Cylinder(BARREL_OD_MM / 2.0, 5.50, align=MIN)
        - Cylinder(BARREL_BORE_MM / 2.0, 5.70, align=MIN)
    )
    # Inner outer-race shoulder and remote keeper lugs.
    shoulder = Pos(0.0, 0.0, 4.25) * (
        Cylinder(BARREL_OD_MM / 2.0, 0.35, align=MIN)
        - Cylinder(2.82, 0.55, align=MIN)
    )
    lugs = (
        Pos(-4.20, 0.0, 8.95) * Box(2.0, 3.0, 2.2, align=CTR)
    ).fuse(
        Pos(4.20, 0.0, 8.95) * Box(2.0, 3.0, 2.2, align=CTR)
    )
    body = housing.fuse(shoulder, lugs)
    for x in (-4.20, 4.20):
        body = body.cut(Pos(x, 0.0, 7.55) * Cylinder(0.80, 3.0, align=MIN))
    body = body.clean()
    if len(list(body.solids())) != 1:
        raise RuntimeError("bearing barrel must be one solid")
    return _label(body, "one_sided_7075_L630ZZ_pair_barrel")


@lru_cache(maxsize=1)
def keeper_cap_local():
    cap = Pos(0.0, 0.0, 9.75) * Box(
        KEEPER_LUG_SPAN_MM, 7.0, KEEPER_THICKNESS_MM, align=CTR,
    )
    cap = cap.cut(Pos(0.0, 0.0, 9.25) * Cylinder(2.75, 2.0, align=MIN))
    for x in (-4.20, 4.20):
        cap = cap.cut(Pos(x, 0.0, 9.20) * Cylinder(1.10, 2.0, align=MIN))
    return _label(cap.clean(), "two_screw_remote_bearing_keeper_cap")


def _axis_stack_parts(
    identity: int,
    axis_name: str,
    center: Sequence[float],
    outward_axis: Sequence[float],
    *,
    include_housing: bool,
) -> tuple[Any, ...]:
    """Complete 10 mm shoulder/bearing/spacer/keeper stack."""

    identity = int(identity)
    axis = _unit(outward_axis)
    prefix = f"id{identity}_{axis_name}"
    parts: list[Any] = []
    if include_housing:
        housing = _place_on_axis(one_sided_barrel_local(), center, axis)
        housing.label = f"{prefix}_one_sided_7075_barrel"
        parts.append(housing)

    # Exact shoulder occupation keeps the large OD6 bearings remote from the
    # universal-joint centre:
    # shim .5 + inner/outer spacer 4 + bearing 2.5 + bearing 2.5 + shim .5.
    placements = (
        ("inner_shim", 0.0, hardware.thrust_washer(
            3.0, 6.0, SHIM_THICKNESS_MM,
            label=f"{prefix}_DIN988_3x6x0p5_inner",
        )),
        ("inner_spacer", 0.5, inner_spacer_local()),
        ("outer_spacer", 0.5, outer_spacer_local()),
        ("bearing_A", 4.5, bearing_63_2z_local()),
        ("bearing_B", 7.0, bearing_63_2z_local()),
        ("outer_shim", 9.5, hardware.thrust_washer(
            3.0, 6.0, SHIM_THICKNESS_MM,
            label=f"{prefix}_DIN988_3x6x0p5_outer",
        )),
    )
    for role, q, local in placements:
        part = _place_on_axis(local, _add(center, axis, q), axis)
        part.label = f"{prefix}_{role}"
        parts.append(part)

    screw = _place_on_axis(
        hardware.shoulder_screw_90265a115(
            label=f"{prefix}_McMaster_90265A115_OD3x10_M2"
        ),
        _add(center, axis, BARREL_STACK_MM),
        axis,
    )
    screw.label = f"{prefix}_McMaster_90265A115_OD3x10_M2"
    parts.append(screw)

    nut = _place_on_axis(
        hardware.nyloc_nut("M2", label=f"{prefix}_ISO10511_M2_nyloc"),
        _add(center, axis, -3.6),
        axis,
    )
    nut.label = f"{prefix}_ISO10511_M2_nyloc"
    parts.append(nut)

    cap = _place_on_axis(keeper_cap_local(), center, axis)
    cap.label = f"{prefix}_two_screw_keeper_cap"
    parts.append(cap)

    local_plane = _axis_plane(center, axis)
    for index, x in enumerate((-4.20, 4.20)):
        local_origin = _plane_point(local_plane, (x, 0.0, 10.10))
        washer = _place_on_axis(
            hardware.plain_washer(
                "M2", label=f"{prefix}_keeper_M2_washer_{index}",
            ),
            local_origin,
            axis,
        )
        fastener = _place_on_axis(
            hardware.socket_head_cap_screw(
                "M2", 6.0,
                label=f"{prefix}_keeper_ISO4762_M2x6_{index}",
            ),
            _add(local_origin, axis, 0.50),
            axis,
        )
        parts.extend((fastener, washer))
    return tuple(parts)


def _elevation_axis(identity: int) -> tuple[float, float, float]:
    # The guide's negative-Z side is the loading-slot-free backing side.  The
    # bearing barrel points +Z/binormal; the guide hub and preload remain on
    # the opposite side of the pivot centre.
    return identity_directions(int(identity))[2]


def _yaw_axis(identity: int) -> tuple[float, float, float]:
    return (0.0, 0.0, float(AXIAL_SIGN[int(identity)]))


def _head_anchor(identity: int) -> tuple[float, float, float]:
    center = identity_center(int(identity))
    return _add(center, _yaw_axis(int(identity)), 5.0)


def boom_and_yaw_stator(identity: int):
    """One keyed boom fused to the fixed yaw bearing barrel."""

    identity = int(identity)
    sign = TANGENTIAL_SIGN[identity]
    root = pod_output_root(identity)
    anchor = _head_anchor(identity)
    raised_root = (
        root[0],
        root[1],
        anchor[2],
    )
    waypoint_outer = (
        BOOM_TRUNK_X_MM,
        sign * BOOM_INNER_TANGENTIAL_MM,
        anchor[2],
    )
    pre_anchor = (
        BOOM_TRUNK_X_MM,
        sign * BOOM_INNER_TANGENTIAL_MM,
        anchor[2],
    )
    parts = [
        _beam_between(root, raised_root,
                      BOOM_TRUNK_SECTION_MM[0], BOOM_TRUNK_SECTION_MM[1]),
        _beam_between(raised_root, waypoint_outer,
                      BOOM_TRUNK_SECTION_MM[0], BOOM_TRUNK_SECTION_MM[1]),
        _beam_between(pre_anchor, anchor,
                      BOOM_TERMINAL_SECTION_MM[0],
                      BOOM_TERMINAL_SECTION_MM[1]),
        _place_on_axis(one_sided_barrel_local(), identity_center(identity),
                       _yaw_axis(identity)),
    ]
    body = parts[0].fuse(*parts[1:]).clean()
    if len(list(body.solids())) != 1:
        # A small keyed saddle at the remote barrel end is the actual boom to
        # barrel joint and closes any sub-tolerance diagonal-beam seam.
        saddle = Pos(*anchor) * Sphere(2.20)
        body = body.fuse(saddle).clean()
    if len(list(body.solids())) != 1:
        raise RuntimeError(
            f"id{identity} boom/yaw stator must be one solid; "
            f"got {len(list(body.solids()))}"
        )
    return _label(body, f"id{identity}_keyed_7075_boom_with_yaw_stator")


def yaw_rotor_with_elevation_stator(identity: int):
    """Yaw rotor hub fused to the handed elevation bearing barrel."""

    identity = int(identity)
    center = identity_center(identity)
    yaw_axis = _yaw_axis(identity)
    elevation_axis = _elevation_axis(identity)
    yaw_hub_local = Pos(0.0, 0.0, -1.55) * (
        Cylinder(2.60, 1.55, align=MIN)
        - Cylinder(1.10, 1.90, align=MIN)
    )
    yaw_hub = _place_on_axis(yaw_hub_local, center, yaw_axis)
    elevation_barrel = _place_on_axis(
        one_sided_barrel_local(), center, elevation_axis,
    )
    # A compact spherical root fuses the intersecting orthogonal members but
    # stays within the OD5.5 target-neck envelope.
    root = Pos(*center) * Sphere(2.65)
    root = root.cut(_place_on_axis(
        Cylinder(1.10, 7.0, align=CTR), center, yaw_axis,
    ))
    body = yaw_hub.fuse(elevation_barrel, root).clean()
    if len(list(body.solids())) != 1:
        raise RuntimeError("yaw rotor/elevation stator must be one solid")
    return _label(
        body,
        f"id{identity}_yaw_rotor_with_handed_elevation_stator_7075",
    )


def gimbal_hardware(identity: int) -> tuple[Any, ...]:
    identity = int(identity)
    center = identity_center(identity)
    yaw = _axis_stack_parts(
        identity, "yaw", center, _yaw_axis(identity), include_housing=False,
    )
    elevation = _axis_stack_parts(
        identity, "elevation", center, _elevation_axis(identity),
        include_housing=False,
    )
    return tuple((*yaw, *elevation))


@lru_cache(maxsize=1)
def preload_cradle_local():
    """Separate aluminum cradle behind the guide loading opening."""

    collar = Pos(0.0, 0.0, -3.00) * (
        Cylinder(3.65, 0.70, align=CTR)
        - Cylinder(2.75, 1.0, align=CTR)
    )
    spine = Pos(0.0, 4.45, -2.55) * Box(3.4, 3.1, 0.75, align=CTR)
    clamp_land = Pos(0.0, 5.00, -2.55) * Box(6.2, 2.4, 0.75, align=CTR)
    adjuster_land = Pos(0.0, 3.30, -2.55) * Box(4.5, 2.4, 0.75, align=CTR)
    body = collar.fuse(spine, clamp_land, adjuster_land)
    for x in (-2.20, 2.20):
        body = body.cut(Pos(x, 5.00, -3.2) * Cylinder(0.85, 2.0, align=MIN))
    body = body.cut(Pos(0.0, 3.30, -3.2) * Cylinder(0.85, 2.0, align=MIN))
    body = body.clean()
    if len(list(body.solids())) != 1:
        raise RuntimeError("preload cradle must be one solid")
    return _label(body, "separate_7075_normal_preload_cradle")


@lru_cache(maxsize=1)
def preload_leaf_local():
    leaf = Pos(0.0, 1.0, -2.04) * Box(
        PRELOAD_LEAF_WIDTH_MM,
        PRELOAD_LEAF_LENGTH_MM,
        PRELOAD_LEAF_THICKNESS_MM,
        align=CTR,
    )
    # Hardened dimple witness under the adjuster; it is part of the leaf.
    dimple = Pos(0.0, 3.30, -1.94) * Sphere(0.30)
    return _label(
        leaf.fuse(dimple).clean(),
        "independent_17-7PH_aggregate_normal_preload_leaf",
    )


@lru_cache(maxsize=1)
def preload_shoe_local():
    x_size, y_size, z_size = PRELOAD_SHOE_MM
    contact_pad = Pos(0.0, -3.0, -1.38) * Box(
        x_size, y_size, z_size, align=CTR,
    )
    # Shallow convex polished contact crown; overall envelope remains compact.
    crown = Pos(0.0, -3.0, -1.10) * Sphere(0.70)
    crown = crown & Pos(0.0, -3.0, -1.18) * Box(
        x_size, y_size, 0.35, align=CTR,
    )
    keyed_stem = Pos(0.0, -1.85, -1.80) * Box(
        1.0, 2.6, 0.85, align=CTR,
    )
    shoe = contact_pad.fuse(crown, keyed_stem)
    if len(list(shoe.solids())) != 1:
        raise RuntimeError("preload shoe must be one solid")
    return _label(shoe.clean(), "replaceable_polished_PEEK_normal_preload_shoe")


def preload_parts(identity: int) -> tuple[Any, ...]:
    identity = int(identity)
    case = datum_case(identity)
    frame = guide_frame(
        case["required_center_local_mm"],
        case["required_guide_tangent"],
        case["required_curvature_normal_contact_to_center"],
    )
    parts: list[Any] = []
    for role, local in (
        ("separate_7075_preload_cradle", preload_cradle_local()),
        ("independent_17-7PH_preload_leaf", preload_leaf_local()),
        ("replaceable_polished_PEEK_preload_shoe", preload_shoe_local()),
    ):
        part = frame.location * copy(local)
        part.label = f"id{identity}_{role}"
        parts.append(part)

    # Leaf-root clamp bar and two independent M2x6 stacks.
    clamp = frame.location * (
        Pos(0.0, 5.00, -1.76) * Box(6.2, 1.2, 0.55, align=CTR)
    )
    clamp.label = f"id{identity}_preload_leaf_root_clamp_bar"
    parts.append(clamp)
    for index, x in enumerate((-2.20, 2.20)):
        local_screw = Pos(x, 5.00, -1.43) * hardware.socket_head_cap_screw(
            "M2", 6.0,
            label=f"id{identity}_preload_leaf_ISO4762_M2x6_{index}",
        )
        local_washer = Pos(x, 5.00, -1.98) * hardware.plain_washer(
            "M2", label=f"id{identity}_preload_leaf_M2_washer_{index}",
        )
        screw = frame.location * local_screw
        washer = frame.location * local_washer
        parts.extend((screw, washer))

    # M2x8 adjuster and jam nut.  The shank points toward negative local Z and
    # terminates on the hardened dimple without penetrating the leaf.
    adjuster = frame.location * (
        Pos(0.0, 3.30, -1.18) * hardware.socket_head_cap_screw(
            "M2", 8.0,
            label=f"id{identity}_preload_adjuster_ISO4762_M2x8",
        )
    )
    adjuster_nut = frame.location * (
        Pos(0.0, 3.30, -2.00) * hardware.nyloc_nut(
            "M2", label=f"id{identity}_preload_adjuster_M2_jam_nut",
        )
    )
    parts.extend((adjuster, adjuster_nut))

    # The shoe fastener clamps a keyed stem; it never threads into PEEK.
    shoe_clamp = frame.location * (
        Pos(0.0, -1.15, -1.30) * Box(4.2, 1.6, 0.55, align=CTR)
    )
    shoe_clamp.label = f"id{identity}_preload_shoe_key_clamp_bar"
    parts.append(shoe_clamp)
    shoe_screw = frame.location * (
        Pos(0.0, -1.15, -0.98) * hardware.socket_head_cap_screw(
            "M2", 6.0,
            label=f"id{identity}_preload_shoe_ISO4762_M2x6",
        )
    )
    shoe_washer = frame.location * (
        Pos(0.0, -1.15, -1.53) * hardware.plain_washer(
            "M2", label=f"id{identity}_preload_shoe_M2_washer",
        )
    )
    parts.extend((shoe_screw, shoe_washer))
    return tuple(parts)


def center_bound_witnesses(identity: int) -> tuple[Any, ...]:
    """Construction witnesses excluded from manufactured leaf counts."""

    identity = int(identity)
    bounds = v1.identity_contract(identity)["exact_target_center_bounds_local_mm"]
    result = []
    for name in ("min_mm", "max_mm"):
        point = tuple(map(float, bounds[name]))
        witness = Pos(*point) * Sphere(0.12)
        witness.label = f"CONSTRUCTION_ONLY_id{identity}_center_{name}_witness"
        result.append(witness)
    return tuple(result)


def module_parts(identity: int) -> tuple[Any, ...]:
    identity = int(identity)
    case = datum_case(identity)
    guide = guide_at_case(case)
    result: list[Any] = [
        folded_flexure_pod(identity),
        boom_and_yaw_stator(identity),
        yaw_rotor_with_elevation_stator(identity),
        guide,
    ]
    result.extend(pod_attachment_hardware(identity))
    result.extend(gimbal_hardware(identity))
    result.extend(preload_parts(identity))
    result.extend(center_bound_witnesses(identity))
    return tuple(result)


def hardware_contract() -> dict[str, Any]:
    """Exact modeled assembly demand; no procurement authority is implied."""

    per_module = {
        "ISO4762_M3x14_pod_mount": 4,
        "ISO7089_M3_pod_mount_washer": 4,
        "McMaster_94459A130_short_M3_insert": 4,
        "McMaster_90265A115_OD3x10_M2_shoulder_screw": 2,
        "NMB_L-630ZZ_3x6x2p5": 4,
        "DIN988_3x6x0p5_shim": 4,
        "precision_3mm_ID_x_4mm_inner_spacer": 2,
        "matched_4mm_outer_race_spacer": 2,
        "ISO10511_M2_pivot_nyloc": 2,
        "bearing_keeper_cap": 2,
        "ISO4762_M2x6_bearing_keeper": 4,
        "ISO7089_M2_bearing_keeper_washer": 4,
        "ISO4762_M2x6_leaf_root": 2,
        "ISO7089_M2_leaf_root_washer": 2,
        "ISO4762_M2x8_preload_adjuster": 1,
        "ISO10511_M2_adjuster_jam_nut": 1,
        "ISO4762_M2x6_shoe": 1,
        "ISO7089_M2_shoe_washer": 1,
    }
    total = {name: quantity * 4 for name, quantity in per_module.items()}
    total.update({
        "NBK_SSHS_M4x10_SD_ALK_primary_tower_screw": 4,
        "short_M4_heat_set_primary_tower_insert": 4,
    })
    return {
        "per_module": per_module,
        "four_module_plus_existing_primary_mount": total,
        "bearing_catalog": {
            "sku": BEARING_SKU,
            "envelope_mm": [BEARING_BORE_MM, BEARING_OD_MM, BEARING_WIDTH_MM],
            "dynamic_rating_N": BEARING_DYNAMIC_RATING_N,
            "static_rating_N": BEARING_STATIC_RATING_N,
            "step_parts_search_result": "no exact L-630ZZ/MR63 record found",
            "local_623ZZ_rejected": True,
        },
    }


def geometry_contract() -> dict[str, Any]:
    cases = {}
    for identity in range(4):
        case = datum_case(identity)
        tangent, normal, binormal = identity_directions(identity)
        cases[str(identity)] = {
            "name": IDENTITY_NAMES[identity],
            "datum_case_key": deepcopy(DATUM_CASE_KEYS[identity]),
            "center_local_mm": list(map(float, case["required_center_local_mm"])),
            "guide_tangent": list(tangent),
            "curvature_normal_contact_to_center": list(normal),
            "guide_binormal": list(binormal),
            "pod_center_local_mm": list(_pod_center(identity)),
            "pod_output_root_local_mm": list(pod_output_root(identity)),
        }
    return {
        "schema": SCHEMA,
        "status": "REVIEW_ONLY_V2_GEOMETRY_NOT_YET_AUDITED",
        "frame": {
            "axes": "+X radial outward; +Y tangential; +Z stator axis",
            "M0_home_transform": "machine=(-local_y,local_z,95-local_x)",
            "guide_frame": (
                "Plane(origin=center,x_dir=tangent,"
                "z_dir=unit(tangent_x_curvature_normal))"
            ),
            "STEP_pose": "four real active-local 0p2mm-wire datum cases",
        },
        "placement_evidence": {
            "path": str(PLACEMENT_PATH.relative_to(ROOT)).replace("\\", "/"),
            "file_sha256": _sha256(PLACEMENT_PATH),
            "internal_report_sha256": placement_report()["report_sha256"],
            "case_count": len(placement_report()["case_comparisons"]),
        },
        "carrier": {
            "source_corridor": "carriage_yoke_with_migrated_primary_mount",
            "selection_bay_floors_omitted": True,
            "shared_through_window_bounds_local_mm": deepcopy(
                SHARED_WINDOW_BOUNDS_MM
            ),
            "pod_owner": "M0_carriage",
        },
        "pod": {
            "count": 4,
            "usable_XYZ_travel_mm": list(POD_USABLE_TRAVEL_MM),
            "hard_stop_XYZ_travel_mm": list(POD_HARD_TRAVEL_MM),
            "folded_flexure_blades_per_module": {"X": 8, "Y": 8, "Z": 4},
            "material": FLEXURE_MATERIAL,
            "deflected_BREP_proved": False,
        },
        "guide": {
            "count": 4,
            "material": GUIDE_MATERIAL,
            "centerline": "line--R3_quarter_arc--line",
            "centerline_bend_radius_mm": GUIDE_BEND_RADIUS_MM,
            "C1_by_construction": True,
            "positive_Z_loading_opening": True,
            "mechanically_attached_by_elevation_pivot_hub": True,
        },
        "preload": {
            "leaf_count": 4,
            "shoe_count": 4,
            "guide_shared_body": False,
            "guide_shared_leaf_fastener": False,
            "leaf_mm": [
                PRELOAD_LEAF_LENGTH_MM,
                PRELOAD_LEAF_WIDTH_MM,
                PRELOAD_LEAF_THICKNESS_MM,
            ],
            "shoe_envelope_mm": list(PRELOAD_SHOE_MM),
        },
        "identities": cases,
        "hardware": hardware_contract(),
        "open_gates": [
            "all_4704_complete_guide_to_shared_carrier_BREP_clearance",
            "adaptive_five_DOF_self_carrier_sibling_collision_sweep",
            "folded_flexure_nonlinear_stress_fatigue_and_stop_impact",
            "boom_static_modal_and_300RPM_response",
            "bearing_reaction_false_brinelling_fit_and_life",
            "preload_force_shoe_wear_and_aggregate_marking",
            "PEEK_wire_abrasion_dielectric_temperature_and_creep",
            "M3_insert_pullout_and_carrier_web_load_path",
            "full_tolerance_stack_and_manufacturing_drawings",
            "continuous_wire_route_sag_snag_and_dynamics",
        ],
        "authority": dict(AUTHORITY),
    }


def manifest(step_path: Path | str = STEP_OUT) -> dict[str, Any]:
    result = geometry_contract()
    step = Path(step_path)
    source = Path(__file__).resolve()
    result["artifacts"] = {
        "source": str(source),
        "source_sha256": _sha256(source),
        "brief": str((source.parent / "aggregate_boundary_follower_successor_v2_brief.md").resolve()),
        "step": str(step.resolve()),
        "step_exists": step.exists(),
        "step_size_bytes": step.stat().st_size if step.exists() else None,
        "step_sha256": _sha256(step) if step.exists() else None,
    }
    return result


def write_manifest(
    path: Path | str = MANIFEST_OUT,
    step_path: Path | str = STEP_OUT,
) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(manifest(step_path), indent=2) + "\n", encoding="utf-8"
    )
    return target


def gen_step() -> Compound:
    carrier_children: list[Any] = [shared_carrier()]
    carrier_children.extend(replacement.primary_tower_m4_hardware())
    carrier = Compound(children=carrier_children)
    carrier.label = "V2_shared_carrier_and_existing_primary_tower_hardware"

    modules = []
    for identity in range(4):
        module = Compound(children=module_parts(identity))
        module.label = f"V2_identity_{identity}_{IDENTITY_NAMES[identity]}"
        modules.append(module)
    assembly = Compound(children=(carrier, *modules))
    assembly.label = "aggregate_boundary_follower_successor_V2_REVIEW_ONLY"
    solids = list(assembly.solids())
    if not solids or any(float(solid.volume) <= 0.0 for solid in solids):
        raise RuntimeError("every V2 STEP leaf must have positive volume")
    return assembly
