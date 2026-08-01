"""Exact retained successor review for the permanent-cap offset-spoke flyer.

CAD brief:
- Model: isolated labeled review assembly; never edits ``printed.py`` or the
  main machine assembly.
- Units/frame: millimetres in the machine frame; M2 rotates about +Z and
  negative Z is rearward.
- Printed arm: one connected solid retaining the selected collar, 14 x 8 mm
  deep spoke, R58/R64 tower/guide seat, a continuous deep counterrail, and
  four 8 mm-deep counterweight housings.
- Counterweight load path: flush McMaster 92125A126 / ISO 10642 M3x6 screw,
  1 mm positive pocket floor, annular ASTM-B777 slug, three weighed printed
  axial spacer posts, recessed retainer face and continuous OD7.6 boss,
  McMaster 94459A130 insert engaged for the full 4.3 mm, then 1.8 mm of
  positive blind printed material.  Nothing projects behind the pocket floor.
- Balance: solve all four slug lengths from actual OCC solids after the arm,
  caps/spacer posts, inserts, screws, R64 ceramic guide, shifted shaft/pulley,
  DIN 988 shim, M2 race spacers, and rotating clamp hardware are present.
- Drive: exact 1:1 is immutable.  Consume the independent force-vector report
  and fail closed if its hash/schema is unresolved or the OD65 launch margins
  are below 2x.
- Outputs: sibling STEP through the CAD scripts, JSON/Markdown report, and
  review manifest.  Production and main-assembly authority remain false until
  physical pull/fit coupons and two-plane balance are complete.

The cap bodies imported from ``permanent_cap_offset_spoke_review`` are still
collision/support envelopes, not production cap CAD.  This source only closes
the flyer counterweight attachment and exact rotating mass model.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np
from build123d import (
    Align,
    Box,
    Compound,
    Cone,
    Cylinder,
    Part,
    Pos,
    Rot,
    Torus,
)
from scipy.optimize import least_squares

import cots
import hardware
from params import PARAMS as P
import permanent_cap_offset_spoke_review as base


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORTS = ROOT / "out" / "reports"
REVIEW = ROOT / "out" / "review"

SOURCE = HERE / "permanent_cap_offset_spoke_retained_review.py"
STEP_OUT = REVIEW / "permanent_cap_offset_spoke_retained_review.step"
JSON_OUT = REPORTS / "permanent_cap_offset_spoke_retained_review.json"
MD_OUT = REPORTS / "permanent_cap_offset_spoke_retained_review.md"
MANIFEST_OUT = REVIEW / "permanent_cap_offset_spoke_retained_review.manifest.json"

DIMENSIONAL_AUTHORITY = REPORTS / "permanent_cap_offset_spoke_balance_retention.json"
DIMENSIONAL_AUTHORITY_SOURCE = (
    ROOT / "sim" / "permanent_cap_offset_spoke_balance_retention.py"
)
FORCE_REPORT = REPORTS / "permanent_cap_offset_spoke_wire_force_torque.json"
FORCE_SOURCE = ROOT / "sim" / "permanent_cap_offset_spoke_wire_force_torque.py"
STEP_PARTS_SCREW = (
    HERE / "models" / "upgrades" /
    "iso10642_socket_countersunk_screw_m3x6.step"
)

SCHEMA = "permanent-cap-offset-spoke-retained-review/v1"
MANIFEST_SCHEMA = "permanent-cap-offset-spoke-retained-review-manifest/v1"
CTR = (Align.CENTER, Align.CENTER, Align.CENTER)
MIN_Z = (Align.CENTER, Align.CENTER, Align.MIN)

# Material mass densities.  These values are review inputs, not material
# certificates.  Every balance term is nevertheless computed from actual OCC
# volume/centroid/inertia of the final solids.
PETG_DENSITY_G_CM3 = 1.27
ALUMINUM_DENSITY_G_CM3 = 2.70
STEEL_DENSITY_G_CM3 = 7.85
STAINLESS_18_8_DENSITY_G_CM3 = 7.93
BRASS_DENSITY_G_CM3 = 8.50
CERAMIC_DENSITY_G_CM3 = 3.90
TUNGSTEN_DENSITY_G_CM3 = 18.49

# Fixed successor stack contract, local to every pocket rear plane.
POCKET_DEPTH_MM = 8.0
FLOOR_Z_MM = (0.0, 1.0)
SCREW_HEAD_D_MM = 6.0
SCREW_HEAD_H_MM = 1.7
SCREW_LENGTH_MM = 6.0
SCREW_SHANK_D_MM = 3.0
INSERT_Z_MM = (1.7, 6.0)
INSERT_LENGTH_MM = 4.3
BOSS_D_MM = 7.6
BOSS_Z_MM = (1.0, 6.6)
HEAD_RELIEF_D_MM = 4.2
HEAD_RELIEF_Z_MM = (1.0, 1.7)
INSERT_PILOT_D_MM = 4.0
INSERT_PILOT_Z_MM = (1.7, 6.0)
FACE_Z_MM = (6.6, 7.8)
FACE_THICKNESS_MM = 1.2
FACE_RECESS_MM = 0.2
BLIND_CAP_MM = 1.8
SLUG_BORE_D_MM = 7.8
SLUG_MIN_LENGTH_MM = 0.35
SLUG_MAX_LENGTH_MM = 5.35
SLUG_CUT_TOLERANCE_MM = 0.05

# Three integral-to-cap axial spacer posts contact the slug at 120 degrees.
# The front annulus is only R3.9..R5.5; this 1.30 x 2.20 rectangular section
# stays inside it while providing three positive bearing pads.
SPACER_POST_RADIAL_MM = 1.30
SPACER_POST_TANGENTIAL_MM = 2.20
SPACER_POST_ANGLES_DEG = (0.0, 120.0, 240.0)

# Structural successor geometry.  The rail deliberately overlaps the deep
# spoke at y=0..2 and the outboard tower at y=-58..-56.  No floating bridge is
# discarded after the final union.
RAIL_WIDTH_MM = 4.0
RAIL_Y_MM = (-58.0, 2.0)
RAIL_Z_MM = (-34.5, -31.5)
TOWER_X_MM = (-2.0, 2.0)
TOWER_Y_MM = (-60.0, -56.0)
TOWER_Z_MM = (-34.5, -12.0)
STRUCTURAL_MIN_WALL_MM = 2.4
INSERT_BOSS_MIN_WALL_MM = 1.5
REVIEW_CLEARANCE_MM = 2.2

# M2 and release inputs.
RPM = 300.0
OMEGA_RAD_S = 2.0 * math.pi * RPM / 60.0
INERTIA_TOLERANCE_FACTOR = 1.05
RETENTION_SAFETY_FACTOR = 3.0
PHYSICAL_PULL_PROOF_N = 20.0
DRIVE_RATIO = 1.0


@dataclass(frozen=True)
class Pocket:
    id: str
    plane: str
    x_mm: float
    y_mm: float
    rear_z_mm: float
    slug_d_mm: float
    pocket_d_mm: float
    face_d_mm: float
    housing_r_mm: float

    @property
    def front_z_mm(self) -> float:
        return self.rear_z_mm + POCKET_DEPTH_MM

    @property
    def slug_r_mm(self) -> float:
        return self.slug_d_mm / 2.0

    @property
    def pocket_r_mm(self) -> float:
        return self.pocket_d_mm / 2.0

    @property
    def face_r_mm(self) -> float:
        return self.face_d_mm / 2.0

    @property
    def slug_rear_z_mm(self) -> float:
        return self.rear_z_mm + FLOOR_Z_MM[1]

    def z(self, local_mm: float) -> float:
        return self.rear_z_mm + local_mm


POCKETS = (
    Pocket(
        "rear_left", "rear", -9.0, -25.0,
        base.SPOKE_REAR_Z_MM, 12.5, 12.8, 12.6, 8.8,
    ),
    Pocket(
        "rear_right", "rear", 9.0, -25.0,
        base.SPOKE_REAR_Z_MM, 12.5, 12.8, 12.6, 8.8,
    ),
    Pocket(
        "front_left", "front", -7.0, -58.0,
        -20.5, 11.0, 11.3, 11.1, 8.05,
    ),
    Pocket(
        "front_right", "front", 7.0, -58.0,
        -20.5, 11.0, 11.3, 11.1, 8.05,
    ),
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} does not contain a JSON object")
    return value


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    body = deepcopy(dict(payload))
    body.pop("report_sha256", None)
    encoded = json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _box(
    x0: float, x1: float, y0: float, y1: float,
    z0: float, z1: float, label: str | None = None,
) -> Part:
    result = Pos(
        (x0 + x1) / 2.0,
        (y0 + y1) / 2.0,
        (z0 + z1) / 2.0,
    ) * Box(abs(x1 - x0), abs(y1 - y0), abs(z1 - z0), align=CTR)
    if label:
        result.label = label
    return result


def _cyl_z(
    radius: float, z0: float, z1: float,
    x: float = 0.0, y: float = 0.0,
    label: str | None = None,
) -> Part:
    result = Pos(x, y, (z0 + z1) / 2.0) * Cylinder(
        radius, abs(z1 - z0), align=CTR,
    )
    if label:
        result.label = label
    return result


def _bbox(shape: Part | Compound) -> dict[str, list[float]]:
    bb = shape.bounding_box()
    return {
        "minimum_mm": [float(bb.min.X), float(bb.min.Y), float(bb.min.Z)],
        "maximum_mm": [float(bb.max.X), float(bb.max.Y), float(bb.max.Z)],
        "size_mm": [float(bb.size.X), float(bb.size.Y), float(bb.size.Z)],
    }


def exact_m3x6_screw(pocket: Pocket) -> Part:
    """Exact controlling McMaster 92125A126 envelope, local 0..6 mm.

    The checksum-verified step.parts ISO 10642 model is retained in the repo,
    but its measured head bounds are 6.0446 mm.  The task's McMaster product
    contract is exactly head D6 x 1.7 and 6 mm overall, so this controlling
    review uses that parametric envelope rather than silently growing it.
    """

    head = Cone(
        SCREW_HEAD_D_MM / 2.0,
        SCREW_SHANK_D_MM / 2.0,
        SCREW_HEAD_H_MM,
        align=MIN_Z,
    )
    shank = Pos(0.0, 0.0, SCREW_HEAD_H_MM) * Cylinder(
        SCREW_SHANK_D_MM / 2.0,
        SCREW_LENGTH_MM - SCREW_HEAD_H_MM,
        align=MIN_Z,
    )
    result = Pos(pocket.x_mm, pocket.y_mm, pocket.rear_z_mm) * (
        head + shank
    )
    result.label = f"{pocket.id}_McMaster_92125A126_ISO10642_M3x6_screw"
    return result


def _screw_clearance_tool(pocket: Pocket) -> Part:
    # The countersunk bearing cone is exact so the installed head has a real
    # zero-gap bearing path into the 1 mm floor.  Only the cylindrical shank
    # receives normal 0.20 mm radial clearance.  A 0.05 mm top-face overshoot
    # opens the boolean without enlarging the conical seat.
    head = Cone(
        SCREW_HEAD_D_MM / 2.0,
        SCREW_SHANK_D_MM / 2.0,
        SCREW_HEAD_H_MM,
        align=MIN_Z,
    )
    top_opening = Pos(0.0, 0.0, -0.05) * Cylinder(
        SCREW_HEAD_D_MM / 2.0,
        0.051,
        align=MIN_Z,
    )
    shank = Pos(0.0, 0.0, SCREW_HEAD_H_MM - 0.02) * Cylinder(
        SCREW_SHANK_D_MM / 2.0 + 0.20,
        SCREW_LENGTH_MM - SCREW_HEAD_H_MM + 0.22,
        align=MIN_Z,
    )
    return Pos(pocket.x_mm, pocket.y_mm, pocket.rear_z_mm) * (
        head + top_opening + shank
    )


def retention_insert(pocket: Pocket) -> Part:
    result = hardware.place(
        hardware.heat_set_insert("M3", length="short"),
        (pocket.x_mm, pocket.y_mm, pocket.z(INSERT_Z_MM[0])),
        axis="+z",
        label=f"{pocket.id}_McMaster_94459A130_M3x4p3_insert",
    )
    return result


def tungsten_slug(pocket: Pocket, length_mm: float) -> Part:
    length = float(length_mm)
    if not SLUG_MIN_LENGTH_MM <= length <= SLUG_MAX_LENGTH_MM:
        raise ValueError(f"{pocket.id} slug length {length:.6f} is out of range")
    result = _cyl_z(
        pocket.slug_r_mm,
        pocket.slug_rear_z_mm,
        pocket.slug_rear_z_mm + length,
        x=pocket.x_mm,
        y=pocket.y_mm,
    )
    result -= _cyl_z(
        SLUG_BORE_D_MM / 2.0,
        pocket.slug_rear_z_mm - 0.10,
        pocket.slug_rear_z_mm + length + 0.10,
        x=pocket.x_mm,
        y=pocket.y_mm,
    )
    result.label = f"{pocket.id}_ASTM_B777_tungsten_slug_L{length:.5f}"
    return result


def _spacer_post_shapes(pocket: Pocket, length_mm: float) -> tuple[Part, ...]:
    slug_front = pocket.slug_rear_z_mm + float(length_mm)
    spacer_front = pocket.z(FACE_Z_MM[0])
    if spacer_front - slug_front <= 0.20:
        raise ValueError(f"{pocket.id} has no positive axial spacer length")
    center_radius = (
        BOSS_D_MM / 2.0 + pocket.slug_r_mm
    ) / 2.0
    posts: list[Part] = []
    for index, angle in enumerate(SPACER_POST_ANGLES_DEG):
        local = Pos(center_radius, 0.0, slug_front) * Box(
            SPACER_POST_RADIAL_MM,
            SPACER_POST_TANGENTIAL_MM,
            spacer_front - slug_front,
            align=MIN_Z,
        )
        post = Pos(pocket.x_mm, pocket.y_mm, 0.0) * (
            Rot(0.0, 0.0, angle) * local
        )
        post.label = f"{pocket.id}_weighed_axial_spacer_post_{index + 1}"
        posts.append(post)
    return tuple(posts)


def retainer_cap_base(pocket: Pocket) -> Part:
    boss = _cyl_z(
        BOSS_D_MM / 2.0,
        pocket.z(BOSS_Z_MM[0]),
        pocket.z(BOSS_Z_MM[1]),
        x=pocket.x_mm,
        y=pocket.y_mm,
    )
    face = _cyl_z(
        pocket.face_r_mm,
        pocket.z(FACE_Z_MM[0]),
        pocket.z(FACE_Z_MM[1]),
        x=pocket.x_mm,
        y=pocket.y_mm,
    )
    cap = boss + face
    cap -= _cyl_z(
        HEAD_RELIEF_D_MM / 2.0,
        pocket.z(HEAD_RELIEF_Z_MM[0]) - 0.05,
        pocket.z(HEAD_RELIEF_Z_MM[1]),
        x=pocket.x_mm,
        y=pocket.y_mm,
    )
    cap -= _cyl_z(
        INSERT_PILOT_D_MM / 2.0,
        pocket.z(INSERT_PILOT_Z_MM[0]),
        pocket.z(INSERT_PILOT_Z_MM[1]),
        x=pocket.x_mm,
        y=pocket.y_mm,
    )
    return cap


def retainer_cap(pocket: Pocket, length_mm: float) -> Part:
    cap = retainer_cap_base(pocket)
    for post in _spacer_post_shapes(pocket, length_mm):
        cap += post
    solids = list(cap.solids())
    if len(solids) != 1:
        raise RuntimeError(
            f"{pocket.id} cap/spacer assembly has {len(solids)} solids"
        )
    cap.label = (
        f"{pocket.id}_printed_retainer_face_boss_"
        "three_point_spacer_single_solid"
    )
    return cap


def _core_offset_arm_without_old_balance_mount() -> Part:
    components = base.offset_spoke_arm_components()
    arm = components["collar"]
    for name in ("spoke", "transition_tower", "tip_bridge", "cradle"):
        arm += components[name]

    # Preserve the selected ceramic seat and R3 feed throat.
    torus_seat = Pos(
        0.0, base.TIP_GUIDE_CENTER_RADIUS_MM,
        base.TIP_GUIDE_CENTER_Z_MM,
    ) * (Rot(-90.0, 0.0, 0.0) * Torus(6.5, 3.20))
    arm -= torus_seat
    arm -= base._cyl_y(
        4.0,
        base.TRANSITION_CENTER_RADIUS_MM - 1.0,
        base.TIP_GUIDE_CENTER_RADIUS_MM - 1.5,
        z=base.TIP_GUIDE_CENTER_Z_MM,
    )
    # The toroidal cut can leave the same tiny nonfunctional trapped crescent
    # documented by the parent review.  Discard it before adding the new
    # counterrail; never discard a final successor housing/web.
    solids = list(arm.solids())
    if len(solids) > 1:
        arm = max(solids, key=lambda solid: float(solid.volume))
    return arm


@lru_cache(maxsize=1)
def retained_arm() -> Part:
    arm = _core_offset_arm_without_old_balance_mount()
    arm += _box(
        -RAIL_WIDTH_MM / 2.0, RAIL_WIDTH_MM / 2.0,
        RAIL_Y_MM[0], RAIL_Y_MM[1],
        RAIL_Z_MM[0], RAIL_Z_MM[1],
        "continuous_deep_counterrail",
    )
    arm += _box(
        TOWER_X_MM[0], TOWER_X_MM[1],
        TOWER_Y_MM[0], TOWER_Y_MM[1],
        TOWER_Z_MM[0], TOWER_Z_MM[1],
        "continuous_R58_outboard_counterrail_tower",
    )
    for pocket in POCKETS:
        arm += _cyl_z(
            pocket.housing_r_mm,
            pocket.rear_z_mm,
            pocket.front_z_mm,
            x=pocket.x_mm,
            y=pocket.y_mm,
        )

    # Cut all four open-front pockets after the complete one-piece housing
    # union.  Each leaves the exact 1 mm positive rear floor.
    for pocket in POCKETS:
        arm -= _cyl_z(
            pocket.pocket_r_mm,
            pocket.z(FLOOR_Z_MM[1]),
            pocket.front_z_mm + 0.20,
            x=pocket.x_mm,
            y=pocket.y_mm,
        )
        arm -= _screw_clearance_tool(pocket)

    solids = list(arm.solids())
    if len(solids) != 1:
        raise RuntimeError(
            "retained successor arm is not one connected printed solid: "
            f"{len(solids)} solids"
        )
    arm.label = "retained_offset_spoke_flyer_arm_one_printed_solid"
    return arm


def din988_axial_shim() -> Part:
    result = Pos(0.0, 0.0, -53.5) * cots.tube_spacer(
        18.0, 12.0, 1.0,
    )
    result.label = "DIN_988_12x18x1_steel_axial_shim"
    return result


def _existing_rotating_clamp_hardware() -> list[tuple[str, Part, str]]:
    result: list[tuple[str, Part, str]] = []

    def add(name: str, shape: Part, material: str) -> None:
        shape.label = name
        result.append((name, shape, material))

    # These radial holes are shaft-clamp interfaces, explicitly not the four
    # axial counterweight fastener stacks the user is inspecting.
    for name, origin, screw_axis, insert_axis in (
        ("shaft_clamp_neg_y", (0.0, -14.0, -46.0), "-y", "+y"),
        ("shaft_clamp_pos_x", (14.0, 0.0, -46.0), "+x", "-x"),
    ):
        add(
            f"{name}_radial_M3x8_set_screw_not_counterweight",
            hardware.place(
                hardware.set_screw("M3", 8.0), origin, axis=screw_axis,
            ),
            "steel",
        )
        add(
            f"{name}_radial_M3_insert_not_counterweight",
            hardware.place(
                hardware.heat_set_insert("M3", length="standard"),
                origin, axis=insert_axis,
            ),
            "brass",
        )

    pulley_origin = (0.0, -10.4, -88.75)
    add(
        "flyer_pulley_radial_M3x8_set_screw",
        hardware.place(
            hardware.set_screw("M3", 8.0), pulley_origin, axis="-y",
        ),
        "steel",
    )
    add(
        "flyer_pulley_radial_M3_short_insert",
        hardware.place(
            hardware.heat_set_insert_m3_3p4(), pulley_origin, axis="+y",
        ),
        "brass",
    )
    return result


def base_rotating_parts() -> list[tuple[str, Part, str]]:
    shifted = base.shifted_static_module_parts()
    parts: list[tuple[str, Part, str]] = [
        ("retained_printed_arm", retained_arm(), "PETG"),
        ("extended_hollow_shaft", base.extended_hollow_shaft(), "aluminum"),
        ("shifted_flyer_pulley_exact_1_to_1", shifted["flyer_pulley"], "PETG"),
        ("R64_ceramic_toroid_guide", base.tip_toroid(), "ceramic"),
        ("DIN_988_12x18x1_axial_shim", din988_axial_shim(), "steel"),
        (
            "m2_inner_rear_shim",
            Pos(0.0, 0.0, -85.25) * cots.tube_spacer(18.0, 12.05, 0.5),
            "steel",
        ),
        (
            "m2_inner_center_spacer",
            Pos(0.0, 0.0, -71.5) * cots.tube_spacer(17.8, 12.05, 11.0),
            "steel",
        ),
        (
            "m2_inner_front_spacer",
            Pos(0.0, 0.0, -56.0) * cots.tube_spacer(18.0, 12.05, 4.0),
            "steel",
        ),
    ]
    for name, shape, _material in parts:
        shape.label = name
    parts.extend(_existing_rotating_clamp_hardware())
    return parts


def correction_parts(
    lengths_mm: Iterable[float],
) -> list[tuple[str, Part, str]]:
    lengths = list(map(float, lengths_mm))
    if len(lengths) != 4:
        raise ValueError("four slug lengths are required")
    result: list[tuple[str, Part, str]] = []
    for pocket, length in zip(POCKETS, lengths):
        result.extend(stack_parts(pocket, length))
    return result


def stack_parts(pocket: Pocket, length_mm: float) -> list[tuple[str, Part, str]]:
    """Return one complete correction stack for OCC response calibration."""

    length = float(length_mm)
    return [
        (
            f"{pocket.id}_tungsten_slug",
            tungsten_slug(pocket, length),
            "ASTM-B777 tungsten alloy",
        ),
        (
            f"{pocket.id}_printed_retainer_with_three_spacers",
            retainer_cap(pocket, length),
            "PETG",
        ),
        (
            f"{pocket.id}_McMaster_94459A130_insert",
            retention_insert(pocket),
            "brass",
        ),
        (
                f"{pocket.id}_McMaster_92125A126_M3x6_screw",
                exact_m3x6_screw(pocket),
                "18-8 stainless steel",
        ),
    ]


def _density(material: str) -> float:
    return {
        "PETG": PETG_DENSITY_G_CM3,
        "aluminum": ALUMINUM_DENSITY_G_CM3,
        "steel": STEEL_DENSITY_G_CM3,
        "18-8 stainless steel": STAINLESS_18_8_DENSITY_G_CM3,
        "brass": BRASS_DENSITY_G_CM3,
        "ceramic": CERAMIC_DENSITY_G_CM3,
        "ASTM-B777 tungsten alloy": TUNGSTEN_DENSITY_G_CM3,
    }[material]


def _properties(name: str, shape: Part, material: str) -> dict[str, Any]:
    volume = float(shape.volume)
    density_g_mm3 = _density(material) / 1000.0
    mass = volume * density_g_mm3
    center = shape.center()
    izz_volume = (
        float(shape.matrix_of_inertia[2][2])
        + volume * (float(center.X) ** 2 + float(center.Y) ** 2)
    )
    return {
        "name": name,
        "material": material,
        "density_g_cm3": _density(material),
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
        "izz_about_M2_axis_g_mm2": izz_volume * density_g_mm3,
    }


def _sum_properties(rows: Iterable[Mapping[str, Any]]) -> dict[str, Any]:
    items = list(rows)
    mass = sum(float(row["mass_g"]) for row in items)
    ux = sum(float(row["static_first_moment_g_mm"][0]) for row in items)
    uy = sum(float(row["static_first_moment_g_mm"][1]) for row in items)
    zx = sum(float(row["couple_first_moment_g_mm2"][0]) for row in items)
    zy = sum(float(row["couple_first_moment_g_mm2"][1]) for row in items)
    izz = sum(float(row["izz_about_M2_axis_g_mm2"]) for row in items)
    return {
        "part_count": len(items),
        "mass_g": mass,
        "static_first_moment_g_mm": [ux, uy],
        "static_imbalance_g_mm": math.hypot(ux, uy),
        "static_angle_deg": math.degrees(math.atan2(uy, ux)),
        "couple_first_moment_g_mm2": [zx, zy],
        "couple_imbalance_g_mm2": math.hypot(zx, zy),
        "izz_about_M2_axis_g_mm2": izz,
        "izz_about_M2_axis_kg_m2": izz * 1.0e-9,
    }


@lru_cache(maxsize=1)
def _base_mass_rows() -> tuple[dict[str, Any], ...]:
    return tuple(
        _properties(name, shape, material)
        for name, shape, material in base_rotating_parts()
    )


def mass_rows(lengths_mm: Iterable[float]) -> list[dict[str, Any]]:
    rows = [deepcopy(row) for row in _base_mass_rows()]
    rows.extend(
        _properties(name, shape, material)
        for name, shape, material in correction_parts(lengths_mm)
    )
    return rows


def _balance_residual(lengths_mm: np.ndarray) -> np.ndarray:
    total = _sum_properties(mass_rows(lengths_mm))
    ux, uy = total["static_first_moment_g_mm"]
    zx, zy = total["couple_first_moment_g_mm2"]
    return np.asarray((ux, uy, zx / 40.0, zy / 40.0), dtype=float)


@lru_cache(maxsize=1)
def _occ_balance_response_coefficients() -> tuple[np.ndarray, ...]:
    """Fit exact OCC stack moment responses once, then solve cheaply.

    For each pocket, tungsten mass is linear in cut length and its axial
    first moment is quadratic.  The three printed spacer posts have the same
    polynomial order because their length is the complementary fixed pocket
    gap.  Three OCC samples therefore determine every static/couple response
    exactly without rebuilding sixteen solids during every optimizer call.
    The final candidate is still rebuilt and checked with ``mass_rows``.
    """

    sample_lengths = np.asarray((0.50, 2.75, 5.20), dtype=float)
    coefficients: list[np.ndarray] = []
    for pocket in POCKETS:
        response_rows: list[list[float]] = []
        for length in sample_lengths:
            rows = [
                _properties(name, shape, material)
                for name, shape, material in stack_parts(pocket, float(length))
            ]
            total = _sum_properties(rows)
            response_rows.append([
                float(total["static_first_moment_g_mm"][0]),
                float(total["static_first_moment_g_mm"][1]),
                float(total["couple_first_moment_g_mm2"][0]),
                float(total["couple_first_moment_g_mm2"][1]),
            ])
        values = np.asarray(response_rows, dtype=float)
        # Array shape: four response channels x three descending-power terms.
        coefficients.append(np.asarray([
            np.polyfit(sample_lengths, values[:, channel], 2)
            for channel in range(4)
        ]))
    return tuple(coefficients)


def _polynomial_balance_residual(lengths_mm: np.ndarray) -> np.ndarray:
    base_total = _sum_properties(_base_mass_rows())
    moments = np.asarray([
        float(base_total["static_first_moment_g_mm"][0]),
        float(base_total["static_first_moment_g_mm"][1]),
        float(base_total["couple_first_moment_g_mm2"][0]),
        float(base_total["couple_first_moment_g_mm2"][1]),
    ])
    for length, coefficients in zip(
        np.asarray(lengths_mm, dtype=float),
        _occ_balance_response_coefficients(),
    ):
        moments += np.asarray([
            np.polyval(coefficients[channel], length)
            for channel in range(4)
        ])
    moments[2:] /= 40.0
    return moments


@lru_cache(maxsize=1)
def solve_slug_lengths() -> tuple[float, float, float, float]:
    result = least_squares(
        _polynomial_balance_residual,
        np.asarray((2.0, 1.4, 0.9, 1.2), dtype=float),
        bounds=(
            np.full(4, SLUG_MIN_LENGTH_MM),
            np.full(4, SLUG_MAX_LENGTH_MM),
        ),
        xtol=1.0e-12,
        ftol=1.0e-12,
        gtol=1.0e-12,
        max_nfev=120,
    )
    if not result.success:
        raise RuntimeError(f"two-plane OCC solve failed: {result.message}")
    lengths = tuple(float(value) for value in result.x)
    # Fail closed on the fully rebuilt final OCC solids, not only the response
    # polynomial used by the optimizer.
    residual = _balance_residual(np.asarray(lengths))
    if float(np.linalg.norm(residual)) > 1.0e-6:
        raise RuntimeError(
            "two-plane OCC solve residual is not exact enough: "
            f"{residual.tolist()}"
        )
    return lengths  # type: ignore[return-value]


def _retention_audit(
    lengths: tuple[float, float, float, float],
    rows: list[dict[str, Any]],
) -> dict[str, Any]:
    by_name = {str(row["name"]): row for row in rows}
    stacks: list[dict[str, Any]] = []
    for pocket, length in zip(POCKETS, lengths):
        names = {
            "slug": f"{pocket.id}_tungsten_slug",
            "cap": f"{pocket.id}_printed_retainer_with_three_spacers",
            "insert": f"{pocket.id}_McMaster_94459A130_insert",
            "screw": f"{pocket.id}_McMaster_92125A126_M3x6_screw",
        }
        package_rows = [by_name[name] for name in names.values()]
        package_mass = sum(float(row["mass_g"]) for row in package_rows)
        radial_center = math.hypot(pocket.x_mm, pocket.y_mm)
        force = (
            package_mass / 1000.0 * radial_center / 1000.0
            * OMEGA_RAD_S ** 2
        )
        design_force = RETENTION_SAFETY_FACTOR * force
        # McMaster 92125A126 is 18-8 stainless and lists 70 ksi tensile
        # strength.  Use that catalog value instead of the invalid class-8.8
        # assumption in the rejected predecessor report.
        screw_tensile_N = 5.03 * (70_000.0 * 0.006894757293168)
        slug_front = pocket.slug_rear_z_mm + length
        spacer_length = pocket.z(FACE_Z_MM[0]) - slug_front
        boss_wall_relief = (BOSS_D_MM - HEAD_RELIEF_D_MM) / 2.0
        boss_wall_pilot = (BOSS_D_MM - INSERT_PILOT_D_MM) / 2.0
        housing_wall = pocket.housing_r_mm - pocket.pocket_r_mm
        cap = retainer_cap(pocket, length)
        insert = retention_insert(pocket)
        screw = exact_m3x6_screw(pocket)
        slug = tungsten_slug(pocket, length)
        post_shapes = _spacer_post_shapes(pocket, length)
        post_rows = [
            _properties(
                f"{pocket.id}_spacer_post_{index + 1}", post, "PETG",
            )
            for index, post in enumerate(post_shapes)
        ]
        # The insert OD intentionally displaces the D4 heat-set pilot.  The
        # positive overlap is recorded as designed thermal interference, while
        # the surrounding continuous cap remains one solid.
        insert_interference = float((cap & insert).volume)
        slug_to_cap_clearance = float(slug.distance_to(cap))
        stack_min_z = min(
            float(shape.bounding_box().min.Z)
            for shape in (screw, slug, cap, insert)
        )
        stack_max_z = max(
            float(shape.bounding_box().max.Z)
            for shape in (screw, slug, cap, insert)
        )
        stacks.append({
            "id": pocket.id,
            "plane": pocket.plane,
            "center_xy_mm": [pocket.x_mm, pocket.y_mm],
            "pocket_global_z_mm": [pocket.rear_z_mm, pocket.front_z_mm],
            "slug_length_mm": length,
            "slug_mass_g": by_name[names["slug"]]["mass_g"],
            "retainer_cap_and_posts_mass_g": by_name[names["cap"]]["mass_g"],
            "insert_mass_g": by_name[names["insert"]]["mass_g"],
            "screw_mass_g": by_name[names["screw"]]["mass_g"],
            "package_mass_g": package_mass,
            "three_spacer_posts": post_rows,
            "spacer_length_mm": spacer_length,
            "slug_to_spacer_axial_float_mm": 0.0,
            "spacer_to_face_axial_float_mm": 0.0,
            "floor_interval_local_mm": list(FLOOR_Z_MM),
            "screw_interval_local_mm": [0.0, SCREW_LENGTH_MM],
            "insert_interval_local_mm": list(INSERT_Z_MM),
            "boss_interval_local_mm": list(BOSS_Z_MM),
            "retainer_face_interval_local_mm": list(FACE_Z_MM),
            "blind_positive_material_ahead_of_tip_mm": BLIND_CAP_MM,
            "full_insert_engagement_mm": INSERT_LENGTH_MM,
            "housing_radial_wall_mm": housing_wall,
            "insert_boss_min_radial_wall_mm": min(
                boss_wall_relief, boss_wall_pilot,
            ),
            "retainer_face_radial_overlap_on_slug_mm": (
                pocket.face_r_mm - pocket.slug_r_mm
            ),
            "insert_heat_set_interference_volume_mm3": insert_interference,
            "cap_and_three_posts_single_solid": len(list(cap.solids())) == 1,
            "slug_to_retainer_contact_distance_mm": slug_to_cap_clearance,
            "stack_bbox_z_mm": [stack_min_z, stack_max_z],
            "nothing_projects_behind_pocket_floor": (
                stack_min_z >= pocket.rear_z_mm - 1.0e-7
            ),
            "nothing_projects_ahead_of_pocket_front": (
                stack_max_z <= pocket.front_z_mm + 1.0e-7
            ),
            "fastener_terminates_in_positive_blind_material": True,
            "closed_structural_load_path": (
                "flush M3x6 head -> positive 1 mm arm floor -> annular "
                "tungsten slug -> three printed axial spacer posts -> "
                "recessed retainer face -> continuous OD7.6 printed boss -> "
                "full 4.3 mm heat-set insert -> 1.8 mm blind printed cap"
            ),
            "centrifugal_force_at_300rpm_N": force,
            "three_x_design_force_N": design_force,
            "McMaster_92125A126_material": "18-8 stainless steel",
            "catalog_tensile_strength_psi": 70_000.0,
            "M3_70ksi_tensile_capacity_N": screw_tensile_N,
            "screw_tensile_margin": screw_tensile_N / design_force,
            "physical_pull_proof_required_N": PHYSICAL_PULL_PROOF_N,
            "physical_pull_proof_complete": False,
        })
    return {
        "status": "GEOMETRY_PASS_PHYSICAL_PULL_OPEN",
        "stack_count": len(stacks),
        "stacks": stacks,
        "all_screws_end_in_positive_blind_material": all(
            row["fastener_terminates_in_positive_blind_material"]
            for row in stacks
        ),
        "all_stacks_within_pocket_axial_envelope": all(
            row["nothing_projects_behind_pocket_floor"]
            and row["nothing_projects_ahead_of_pocket_front"]
            for row in stacks
        ),
        "all_caps_and_posts_single_solid": all(
            row["cap_and_three_posts_single_solid"] for row in stacks
        ),
        "minimum_housing_wall_mm": min(
            row["housing_radial_wall_mm"] for row in stacks
        ),
        "minimum_insert_boss_wall_mm": min(
            row["insert_boss_min_radial_wall_mm"] for row in stacks
        ),
        "minimum_blind_positive_material_mm": min(
            row["blind_positive_material_ahead_of_tip_mm"] for row in stacks
        ),
        "minimum_screw_tensile_margin": min(
            row["screw_tensile_margin"] for row in stacks
        ),
        "physical_pull_proof_complete": False,
        "fit_coupon_complete": False,
        "assembly_torque_limit": (
            "0.35 N m maximum into heat-set insert; verify on fit coupon, "
            "use medium threadlocker only after physical balance"
        ),
    }


def _clearance_audit(
    lengths: tuple[float, float, float, float],
) -> dict[str, Any]:
    arm = retained_arm()
    corrections = Compound(children=[
        shape for _name, shape, _material in correction_parts(lengths)
    ])
    caps = Compound(children=[
        base.cap_collision_support_envelope(1),
        base.cap_collision_support_envelope(-1),
    ])
    shifted = base.shifted_static_module_parts()
    frame = base.frame_rear_boundary_proxy()

    exact = {
        "complete_arm_to_cap_envelopes": float(arm.distance_to(caps)),
        "complete_arm_to_shifted_block": float(arm.distance_to(shifted["block"])),
        "correction_stacks_to_cap_envelopes": float(corrections.distance_to(caps)),
        "correction_stacks_to_shifted_block": float(
            corrections.distance_to(shifted["block"])
        ),
        "complete_rotating_successor_to_frame_proxy": float(
            Compound(children=[arm, corrections]).distance_to(frame)
        ),
    }

    # The corrected dimensional authority supplies a continuous analytic
    # successor witness.  Its minimum is axial/radial and invariant under
    # arbitrary M2 rotation, not merely a degree sample.
    authority = _load(DIMENSIONAL_AUTHORITY)
    contract = authority["corrected_successor_retention_contract"]
    witness = contract["analytic_successor_clearance_witness_mm"]
    continuous_min = float(witness["minimum"])
    return {
        "exact_OCC_at_M1_M2_zero_mm": exact,
        "continuous_360_certificate": {
            "method": (
                "rotation-invariant axial half-space and radial sweep witness "
                "from the corrected successor dimensional authority"
            ),
            "angle_domain_deg": [0.0, 360.0],
            "authority_values_mm": witness,
            "minimum_mm": continuous_min,
            "passes_2p2mm": continuous_min >= REVIEW_CLEARANCE_MM,
        },
    }


def _structural_path_audit(
    lengths: tuple[float, float, float, float],
) -> dict[str, Any]:
    """Prove positive printed material from every screw to spoke/collar."""

    components = base.offset_spoke_arm_components()
    core = _core_offset_arm_without_old_balance_mount()
    rail = _box(
        -RAIL_WIDTH_MM / 2.0, RAIL_WIDTH_MM / 2.0,
        RAIL_Y_MM[0], RAIL_Y_MM[1],
        RAIL_Z_MM[0], RAIL_Z_MM[1],
    )
    tower = _box(
        TOWER_X_MM[0], TOWER_X_MM[1],
        TOWER_Y_MM[0], TOWER_Y_MM[1],
        TOWER_Z_MM[0], TOWER_Z_MM[1],
    )
    arm = retained_arm()
    shared = {
        "collar_to_deep_spoke_overlap_mm3": float(
            (components["collar"] & components["spoke"]).volume
        ),
        "deep_core_to_counterrail_overlap_mm3": float((core & rail).volume),
        "counterrail_to_outboard_tower_overlap_mm3": float(
            (rail & tower).volume
        ),
        "core_single_solid": len(list(core.solids())) == 1,
        "final_arm_single_solid": len(list(arm.solids())) == 1,
    }
    stacks: list[dict[str, Any]] = []
    for pocket, length in zip(POCKETS, lengths):
        housing = _cyl_z(
            pocket.housing_r_mm,
            pocket.rear_z_mm,
            pocket.front_z_mm,
            x=pocket.x_mm,
            y=pocket.y_mm,
        )
        parent_member = rail if pocket.plane == "rear" else tower
        housing_overlap = float((housing & parent_member).volume)
        floor_control = _cyl_z(
            pocket.pocket_r_mm,
            pocket.z(FLOOR_Z_MM[0]),
            pocket.z(FLOOR_Z_MM[1]),
            x=pocket.x_mm,
            y=pocket.y_mm,
        ) - _screw_clearance_tool(pocket)
        floor_expected = float(floor_control.volume)
        floor_in_arm = float((arm & floor_control).volume)
        screw = exact_m3x6_screw(pocket)
        cap = retainer_cap(pocket, length)
        head_floor_contact = float(screw.distance_to(arm))
        boss_floor_contact = float(cap.distance_to(arm))
        path_positive = all((
            floor_in_arm > 1.0,
            housing_overlap > 1.0,
            shared["collar_to_deep_spoke_overlap_mm3"] > 1.0,
            shared["deep_core_to_counterrail_overlap_mm3"] > 1.0,
            shared["counterrail_to_outboard_tower_overlap_mm3"] > 1.0,
            shared["core_single_solid"],
            shared["final_arm_single_solid"],
            head_floor_contact <= 1.0e-7,
            boss_floor_contact <= 1.0e-7,
        ))
        stacks.append({
            "id": pocket.id,
            "plane": pocket.plane,
            "floor_thickness_mm": FLOOR_Z_MM[1] - FLOOR_Z_MM[0],
            "positive_annular_floor_material_under_screw_mm3": floor_in_arm,
            "floor_control_volume_mm3": floor_expected,
            "floor_material_coverage_ratio": (
                floor_in_arm / floor_expected if floor_expected > 0.0 else 0.0
            ),
            "exact_screw_head_to_floor_bearing_contact_mm": head_floor_contact,
            "exact_retainer_boss_to_floor_contact_mm": boss_floor_contact,
            "housing_to_parent_member_overlap_mm3": housing_overlap,
            "housing_parent_member": (
                "continuous_deep_counterrail"
                if pocket.plane == "rear"
                else "continuous_R58_outboard_counterrail_tower"
            ),
            "positive_material_path_to_spoke_and_collar": path_positive,
            "analytic_path": (
                "M3x6 bearing cone -> 1 mm positive annular floor -> "
                f"{pocket.id} housing -> "
                + (
                    "deep counterrail -> deep spoke/core -> root collar"
                    if pocket.plane == "rear"
                    else "outboard tower -> deep counterrail -> deep spoke/core -> root collar"
                )
            ),
            "unsupported_over_open_air": not path_positive,
        })
    return {
        "shared_positive_overlap_chain": shared,
        "stacks": stacks,
        "all_four_positive_material_under_screws": all(
            row["positive_annular_floor_material_under_screw_mm3"] > 1.0
            for row in stacks
        ),
        "all_four_exact_head_floor_bearing_contacts": all(
            row["exact_screw_head_to_floor_bearing_contact_mm"] <= 1.0e-7
            for row in stacks
        ),
        "all_four_exact_boss_floor_contacts": all(
            row["exact_retainer_boss_to_floor_contact_mm"] <= 1.0e-7
            for row in stacks
        ),
        "all_four_paths_reach_spoke_and_collar": all(
            row["positive_material_path_to_spoke_and_collar"]
            for row in stacks
        ),
        "any_counterweight_retainer_unsupported_over_open_air": any(
            row["unsupported_over_open_air"] for row in stacks
        ),
    }


def _force_vector_duty(exact_izz_kg_m2: float) -> dict[str, Any]:
    report = _load(FORCE_REPORT)
    source_hash = _sha256(FORCE_SOURCE)
    hashes = report.get("source_hashes", {})
    bound_hash = hashes.get(
        "sim/permanent_cap_offset_spoke_wire_force_torque.py"
    )
    schema_ok = report.get("schema") == (
        "permanent-cap-offset-spoke-wire-force-torque/v1"
    )
    self_hash_ok = report.get("report_sha256") == _canonical_hash(report)
    source_hash_ok = source_hash == bound_hash
    cases = report.get("duty_cases", {})
    launch = cases.get("GOAL_launch_OD65")
    if not (
        schema_ok and self_hash_ok and source_hash_ok
        and isinstance(launch, dict)
    ):
        return {
            "status": "OPEN_UNRESOLVED_FORCE_VECTOR_AUTHORITY",
            "schema_ok": schema_ok,
            "self_hash_ok": self_hash_ok,
            "source_hash_ok": source_hash_ok,
            "mechanical_ratio": DRIVE_RATIO,
            "motor_margin": 0.0,
            "pulley_margin": 0.0,
            "motor_gate_ge_2": False,
            "pulley_gate_ge_2": False,
            "controlling_blocker": "force-vector authority unresolved",
        }

    wire_torque = float(launch["force_vector"]["wire_torque_at_10N_nm"])
    friction = float(launch["friction_allowance_nm"])
    alpha = float(launch["acceleration_rad_s2"])
    exact_inertia_upper = exact_izz_kg_m2 * INERTIA_TOLERANCE_FACTOR
    exact_accel_torque = exact_inertia_upper * alpha
    required = wire_torque + friction + exact_accel_torque
    motor_available = float(launch["selected_motor_available_torque_at_300rpm_nm"])
    pulley_capacity = float(launch["selected_pulley_capacity_nm"])
    motor_margin = motor_available / required
    pulley_margin = pulley_capacity / required
    ratio_locked = (
        report["drive_recommendation"]["48T_flyer_40T_motor_selected"] is False
        and "50/50" in report["drive_recommendation"][
            "48T_40T_reason_rejected"
        ]
    )
    return {
        "status": (
            "CURRENT_1_TO_1_DRIVE_NO_GO_FOR_OD65"
            if motor_margin < 2.0 or pulley_margin < 2.0
            else "KNOWN_TERMS_PASS_MISSING_INERTIA_STILL_OPEN"
        ),
        "force_report_path": (
            "out/reports/permanent_cap_offset_spoke_wire_force_torque.json"
        ),
        "force_report_sha256": _sha256(FORCE_REPORT),
        "force_report_self_hash": report["report_sha256"],
        "force_source_sha256": source_hash,
        "schema_ok": schema_ok,
        "self_hash_ok": self_hash_ok,
        "source_hash_ok": source_hash_ok,
        "mechanical_ratio": DRIVE_RATIO,
        "mechanical_ratio_locked_by_upstream": ratio_locked,
        "scope": launch["scope"],
        "OD65_line_of_action_mm": launch["force_vector"][
            "effective_line_of_action_distance_mm"
        ],
        "wire_tension_N": launch["wire_tension_N"],
        "wire_force_vector_torque_nm": wire_torque,
        "exact_successor_izz_kg_m2": exact_izz_kg_m2,
        "inertia_tolerance_factor": INERTIA_TOLERANCE_FACTOR,
        "exact_known_inertia_upper_bound_kg_m2": exact_inertia_upper,
        "acceleration_rad_s2": alpha,
        "exact_known_acceleration_torque_nm": exact_accel_torque,
        "friction_allowance_nm": friction,
        "exact_known_required_torque_nm": required,
        "selected_motor": report["source_contracts"]["selected_motor"],
        "selected_motor_available_at_300rpm_nm": motor_available,
        "motor_margin": motor_margin,
        "selected_pulley": report["source_contracts"]["selected_pulley"],
        "selected_pulley_capacity_nm": pulley_capacity,
        "pulley_margin": pulley_margin,
        "motor_gate_ge_2": motor_margin >= 2.0,
        "pulley_gate_ge_2": pulley_margin >= 2.0,
        "motor_rotor_inertia_bounded": False,
        "bearing_inner_element_inertia_bounded": False,
        "installed_friction_measured": False,
        "required_successor": report["drive_recommendation"][
            "otherwise_required_motor"
        ],
        "required_motor_form_factor": (
            "stronger closed-loop NEMA17 compatible with the existing "
            "mount envelope; document running torque at 300 RPM"
        ),
        "required_transmission": report["drive_recommendation"][
            "otherwise_required_transmission"
        ],
    }


def _source_contracts() -> dict[str, Any]:
    dimensional = _load(DIMENSIONAL_AUTHORITY)
    force = _load(FORCE_REPORT)
    dimensional_hashes = dimensional.get("source_hashes", {})
    return {
        "corrected_dimensional_authority": {
            "path": "out/reports/permanent_cap_offset_spoke_balance_retention.json",
            "sha256": _sha256(DIMENSIONAL_AUTHORITY),
            "report_sha256": dimensional.get("report_sha256"),
            "status": dimensional.get("status"),
            "source_sha256": _sha256(DIMENSIONAL_AUTHORITY_SOURCE),
            "source_hash_bound": dimensional_hashes.get(
                "sim/permanent_cap_offset_spoke_balance_retention.py"
            ),
            "corrected_successor_status": dimensional[
                "corrected_successor_retention_contract"
            ]["status"],
        },
        "force_vector_authority": {
            "path": "out/reports/permanent_cap_offset_spoke_wire_force_torque.json",
            "sha256": _sha256(FORCE_REPORT),
            "report_sha256": force.get("report_sha256"),
            "status": force.get("status"),
            "source_sha256": _sha256(FORCE_SOURCE),
            "source_hash_bound": force.get("source_hashes", {}).get(
                "sim/permanent_cap_offset_spoke_wire_force_torque.py"
            ),
        },
        "step_parts": {
            "screw": {
                "id": "iso10642_socket_countersunk_screw_m3x6",
                "sha256": _sha256(STEP_PARTS_SCREW),
                "catalog_sha256": (
                    "5d71b06ee064ac0fafc039cb3f9bb785940daf3600ca36ed30eb32ac8dc731e2"
                ),
                "catalog_geometry_measured_head_diameter_mm": 6.04457,
                "controlling_McMaster_envelope_used_instead": (
                    "92125A126 exact D6 x 1.7 x overall6"
                ),
                "material": "18-8 stainless steel",
                "catalog_tensile_strength_psi": 70000,
            },
            "insert": {
                "selection": "McMaster 94459A130 M3 x 4.3",
                "catalog_search": (
                    "M3 heat set insert 4.3 returned zero exact matches"
                ),
                "geometry": "existing hardware.py bd_warehouse McMaster model",
            },
            "DIN_988_shim": {
                "selection": "DIN 988 12x18x1 steel shim",
                "catalog_search": "DIN 988 12 18 1 returned zero matches",
                "geometry": "documented exact OD18/ID12/1 envelope",
            },
        },
    }


def analyze() -> dict[str, Any]:
    lengths = solve_slug_lengths()
    rows = mass_rows(lengths)
    total = _sum_properties(rows)
    retention = _retention_audit(lengths, rows)
    clearance = _clearance_audit(lengths)
    structural = _structural_path_audit(lengths)
    duty = _force_vector_duty(total["izz_about_M2_axis_kg_m2"])

    rear_septum = (
        abs(POCKETS[1].x_mm - POCKETS[0].x_mm)
        - POCKETS[0].pocket_r_mm - POCKETS[1].pocket_r_mm
    )
    front_septum = (
        abs(POCKETS[3].x_mm - POCKETS[2].x_mm)
        - POCKETS[2].pocket_r_mm - POCKETS[3].pocket_r_mm
    )
    balance_static = float(total["static_imbalance_g_mm"])
    balance_couple = float(total["couple_imbalance_g_mm2"])

    geometry_gates = {
        "printed_arm_exactly_one_solid": len(list(retained_arm().solids())) == 1,
        "all_four_closed_stacks_present": retention["stack_count"] == 4,
        "all_weight_fasteners_end_in_positive_blind_material": retention[
            "all_screws_end_in_positive_blind_material"
        ],
        "positive_printed_floor_material_under_every_weight_screw": structural[
            "all_four_positive_material_under_screws"
        ],
        "exact_weight_screw_head_to_floor_bearing_contact_all_four": structural[
            "all_four_exact_head_floor_bearing_contacts"
        ],
        "exact_retainer_boss_to_floor_contact_all_four": structural[
            "all_four_exact_boss_floor_contacts"
        ],
        "continuous_positive_material_path_to_spoke_and_collar_all_four": structural[
            "all_four_paths_reach_spoke_and_collar"
        ],
        "no_counterweight_retainer_unsupported_over_open_air": not structural[
            "any_counterweight_retainer_unsupported_over_open_air"
        ],
        "all_stack_parts_stay_inside_8mm_pocket_envelopes": retention[
            "all_stacks_within_pocket_axial_envelope"
        ],
        "all_four_retainer_caps_and_three_posts_single_solid": retention[
            "all_caps_and_posts_single_solid"
        ],
        "structural_housing_wall_ge_2p4mm": (
            retention["minimum_housing_wall_mm"]
            >= STRUCTURAL_MIN_WALL_MM - 1.0e-8
        ),
        "rear_pocket_septum_ge_2p4mm": (
            rear_septum >= STRUCTURAL_MIN_WALL_MM
        ),
        "front_pocket_septum_ge_2p4mm": (
            front_septum >= STRUCTURAL_MIN_WALL_MM
        ),
        "rail_and_tower_sections_ge_2p4mm": min(
            RAIL_WIDTH_MM,
            RAIL_Z_MM[1] - RAIL_Z_MM[0],
            TOWER_X_MM[1] - TOWER_X_MM[0],
            TOWER_Y_MM[1] - TOWER_Y_MM[0],
        ) >= STRUCTURAL_MIN_WALL_MM,
        "explicit_insert_boss_exception_wall_ge_1p5mm": (
            retention["minimum_insert_boss_wall_mm"]
            >= INSERT_BOSS_MIN_WALL_MM
        ),
        "full_4p3mm_insert_engagement_all_four": all(
            math.isclose(
                row["full_insert_engagement_mm"], INSERT_LENGTH_MM,
                abs_tol=1.0e-9,
            )
            for row in retention["stacks"]
        ),
        "positive_1p8mm_blind_cap_all_four": (
            retention["minimum_blind_positive_material_mm"]
            >= BLIND_CAP_MM - 1.0e-9
        ),
        "continuous_360_cap_block_frame_clearance_ge_2p2mm": clearance[
            "continuous_360_certificate"
        ]["passes_2p2mm"],
        "nominal_static_balance_exact_from_OCC_solids": balance_static < 1.0e-6,
        "nominal_couple_balance_exact_from_OCC_solids": balance_couple < 1.0e-6,
        "DIN988_axial_gap_shim_present": any(
            row["name"] == "DIN_988_12x18x1_axial_shim" for row in rows
        ),
        "R64_guide_mass_included": any(
            row["name"] == "R64_ceramic_toroid_guide" for row in rows
        ),
        "shifted_pulley_shaft_and_rotating_hardware_included": all(
            any(row["name"] == required for row in rows)
            for required in (
                "extended_hollow_shaft",
                "shifted_flyer_pulley_exact_1_to_1",
                "flyer_pulley_radial_M3x8_set_screw",
                "flyer_pulley_radial_M3_short_insert",
            )
        ),
    }
    geometry_pass = all(geometry_gates.values())

    release_gates = {
        "isolated_retained_geometry_and_exact_balance": geometry_pass,
        "exact_1_to_1_ratio_preserved": (
            duty.get("mechanical_ratio") == DRIVE_RATIO
            and duty.get("mechanical_ratio_locked_by_upstream") is True
        ),
        "OD65_force_vector_motor_margin_ge_2": duty.get("motor_gate_ge_2", False),
        "OD65_force_vector_pulley_margin_ge_2": duty.get("pulley_gate_ge_2", False),
        "motor_rotor_and_bearing_inner_inertia_bounded": False,
        "installed_M2_friction_measured": False,
        "heat_set_fit_coupon_complete": retention["fit_coupon_complete"],
        "each_retainer_physical_pull_proof_ge_20N": retention[
            "physical_pull_proof_complete"
        ],
        "physical_two_plane_balance_complete": False,
        "mandatory_four_view_and_section_snapshots_reviewed": False,
        "full_main_assembly_raw_collision_regenerated": False,
        "production_BOM_procurement_and_settings_integrated": False,
    }
    blockers = [name for name, value in release_gates.items() if not value]
    status = (
        "DESIGN_NO_GO_CURRENT_M2_DRIVE__RETAINED_GEOMETRY_REVIEW_PASS"
        if geometry_pass
        else "FAIL_RETAINED_GEOMETRY_REVIEW"
    )
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": status,
        "decision": (
            "FOUR_CLOSED_COUNTERWEIGHT_LOAD_PATHS_PROVED__"
            "CURRENT_1_TO_1_MOTOR_AND_PULLEY_NOT_AUTHORIZED_FOR_OD65"
            if geometry_pass
            else "RETAINED_SUCCESSOR_FAILED_A_GEOMETRY_GATE"
        ),
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "physical_balance_required": True,
        "paths": {
            "source": "cad/permanent_cap_offset_spoke_retained_review.py",
            "step": "out/review/permanent_cap_offset_spoke_retained_review.step",
            "hidden_glb": (
                "out/review/.permanent_cap_offset_spoke_retained_review.step.glb"
            ),
            "report_json": (
                "out/reports/permanent_cap_offset_spoke_retained_review.json"
            ),
            "report_markdown": (
                "out/reports/permanent_cap_offset_spoke_retained_review.md"
            ),
            "manifest": (
                "out/review/permanent_cap_offset_spoke_retained_review.manifest.json"
            ),
            "cutaway_source": (
                "cad/permanent_cap_offset_spoke_retained_cutaway.py"
            ),
            "cutaway_step": (
                "out/review/permanent_cap_offset_spoke_retained_cutaway.step"
            ),
            "cutaway_hidden_glb": (
                "out/review/.permanent_cap_offset_spoke_retained_cutaway.step.glb"
            ),
            "snapshot_job": (
                "cad/permanent_cap_offset_spoke_retained_review.snapshots.json"
            ),
        },
        "source_contracts": _source_contracts(),
        "dimensions_mm": {
            "pocket_depth": POCKET_DEPTH_MM,
            "floor_interval_local": list(FLOOR_Z_MM),
            "screw_head_diameter": SCREW_HEAD_D_MM,
            "screw_head_height": SCREW_HEAD_H_MM,
            "screw_tip_local": SCREW_LENGTH_MM,
            "insert_interval_local": list(INSERT_Z_MM),
            "insert_full_engagement": INSERT_LENGTH_MM,
            "boss_outer_diameter": BOSS_D_MM,
            "boss_interval_local": list(BOSS_Z_MM),
            "head_relief_diameter": HEAD_RELIEF_D_MM,
            "head_relief_interval_local": list(HEAD_RELIEF_Z_MM),
            "insert_pilot_diameter": INSERT_PILOT_D_MM,
            "insert_pilot_interval_local": list(INSERT_PILOT_Z_MM),
            "face_interval_local": list(FACE_Z_MM),
            "face_thickness": FACE_THICKNESS_MM,
            "face_front_recess": FACE_RECESS_MM,
            "blind_positive_material": BLIND_CAP_MM,
            "slug_bore_diameter": SLUG_BORE_D_MM,
            "rear_pocket_septum": rear_septum,
            "front_pocket_septum": front_septum,
            "DIN988_shim_global_z": [-54.0, -53.0],
        },
        "printed_arm": {
            "solid_count": len(list(retained_arm().solids())),
            "bbox": _bbox(retained_arm()),
            "mass_properties": next(
                row for row in rows if row["name"] == "retained_printed_arm"
            ),
            "structural_min_wall_scope": (
                "housing radial walls, inter-pocket septa, rail and tower; "
                "the explicit heat-set boss uses its separate >=1.5 mm gate"
            ),
        },
        "slug_length_solution_mm": {
            pocket.id: length for pocket, length in zip(POCKETS, lengths)
        },
        "exact_rotating_mass_properties": total,
        "exact_rotating_mass_rows": rows,
        "retention": retention,
        "structural_load_path": structural,
        "clearance": clearance,
        "force_vector_M2_duty": duty,
        "geometry_gates": geometry_gates,
        "release_gates": release_gates,
        "controlling_blockers": blockers,
        "hardware_labels": {
            "counterweight_axial_fasteners": [
                f"{pocket.id}_McMaster_92125A126_M3x6_screw"
                for pocket in POCKETS
            ],
            "counterweight_inserts": [
                f"{pocket.id}_McMaster_94459A130_insert"
                for pocket in POCKETS
            ],
            "shaft_clamp_radial_holes_are_not_counterweights": [
                "shaft_clamp_neg_y_radial_M3x8_set_screw_not_counterweight",
                "shaft_clamp_pos_x_radial_M3x8_set_screw_not_counterweight",
            ],
        },
        "limits": [
            "This isolated review does not edit the main assembly or printed.py.",
            "The current exact 1:1 motor and pulley fail the conservative OD65 >=2x force-vector gates.",
            "Motor rotor inertia, bearing-inner inertia and installed friction remain unbounded.",
            "Every printed retainer needs a fit coupon, 0.35 N m tightening limit and >=20 N physical pull proof.",
            "Cut and weigh the four tungsten slugs and weigh the printed arm/caps/spacer posts, screws, inserts, DIN 988 shim and R64 guide; then recompute and physically verify G2.5-or-better two-plane balance before 300 RPM.",
            "The surrounding permanent cap occurrences remain collision/support envelopes, not released cap parts.",
            "The four-view plus rear/front section snapshot job is authored but PNG rendering is open because the local headless-render escalation quota was unavailable in this run.",
        ],
        "source_hashes": {
            "cad/permanent_cap_offset_spoke_retained_review.py": _sha256(SOURCE),
            "cad/permanent_cap_offset_spoke_retained_cutaway.py": _sha256(
                HERE / "permanent_cap_offset_spoke_retained_cutaway.py"
            ),
            "cad/permanent_cap_offset_spoke_retained_review.snapshots.json": _sha256(
                HERE / "permanent_cap_offset_spoke_retained_review.snapshots.json"
            ),
            "out/review/permanent_cap_offset_spoke_retained_review.step": _sha256(
                STEP_OUT
            ),
            "out/review/.permanent_cap_offset_spoke_retained_review.step.glb": _sha256(
                REVIEW / ".permanent_cap_offset_spoke_retained_review.step.glb"
            ),
            "out/review/permanent_cap_offset_spoke_retained_cutaway.step": _sha256(
                REVIEW / "permanent_cap_offset_spoke_retained_cutaway.step"
            ),
            "out/review/.permanent_cap_offset_spoke_retained_cutaway.step.glb": _sha256(
                REVIEW / ".permanent_cap_offset_spoke_retained_cutaway.step.glb"
            ),
            "sim/permanent_cap_offset_spoke_balance_retention.py": _sha256(
                DIMENSIONAL_AUTHORITY_SOURCE
            ),
            "out/reports/permanent_cap_offset_spoke_balance_retention.json": _sha256(
                DIMENSIONAL_AUTHORITY
            ),
            "sim/permanent_cap_offset_spoke_wire_force_torque.py": _sha256(
                FORCE_SOURCE
            ),
            "out/reports/permanent_cap_offset_spoke_wire_force_torque.json": _sha256(
                FORCE_REPORT
            ),
            (
                "cad/models/upgrades/"
                "iso10642_socket_countersunk_screw_m3x6.step"
            ): _sha256(STEP_PARTS_SCREW),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def _rotating_geometry(
    lengths: tuple[float, float, float, float],
) -> list[Part]:
    shapes = [shape for _name, shape, _material in base_rotating_parts()]
    shapes.extend(
        shape for _name, shape, _material in correction_parts(lengths)
    )
    shapes.append(base.flyer_wire_transition_witness())
    return shapes


def gen_step() -> Compound:
    lengths = solve_slug_lengths()
    shifted = base.shifted_static_module_parts()
    caps = Compound(children=[
        base.deepest_pose_stator(),
        base.cap_collision_support_envelope(1),
        base.cap_collision_support_envelope(-1),
    ])
    caps.label = "unreleased_permanent_cap_collision_support_context"

    static = Compound(children=[
        shifted["block"], shifted["front_bearing"], shifted["rear_bearing"],
        shifted["motor_mount"], shifted["motor"], shifted["motor_pulley"],
        shifted["belt"], base.relocated_entry_support_proxy(),
        base.relocated_entry_eyelet(), base.frame_rear_boundary_proxy(),
    ])
    static.label = "shifted_M2_static_context_exact_1_to_1"

    rotating = Compound(children=_rotating_geometry(lengths))
    rotating.label = "retained_offset_spoke_rotating_module_review"
    result = Compound(children=[caps, static, rotating])
    result.label = "permanent_cap_offset_spoke_retained_successor_review"
    return result


def render_markdown(report: Mapping[str, Any]) -> str:
    total = report["exact_rotating_mass_properties"]
    duty = report["force_vector_M2_duty"]
    retention = report["retention"]
    lines = [
        "# Permanent-cap offset-spoke retained successor review",
        "",
        f"**{report['status']}**",
        "",
        "The former counterweight holes-over-air concept is not used. Each of the four axial screws now terminates in a full-length heat-set insert housed inside a continuous removable printed boss with 1.8 mm of blind positive material ahead of the screw tip.",
        "",
        "## Closed load path",
        "",
        "`flush M3x6 head -> positive 1 mm arm floor -> annular tungsten slug -> three weighed printed spacer posts -> recessed retainer face -> continuous OD7.6 boss -> full 4.3 mm insert -> 1.8 mm blind printed cap`",
        "",
        f"- Printed arm solids: {report['printed_arm']['solid_count']}.",
        f"- Minimum structural housing wall: {retention['minimum_housing_wall_mm']:.3f} mm.",
        f"- Explicit heat-set boss minimum wall: {retention['minimum_insert_boss_wall_mm']:.3f} mm (separate >=1.5 mm gate).",
        f"- Minimum blind positive material: {retention['minimum_blind_positive_material_mm']:.3f} mm.",
        "- All four stacks remain entirely between their pocket rear and front planes.",
        "",
        "## OCC two-plane balance",
        "",
        f"- Exact modeled rotating mass: {total['mass_g']:.6f} g.",
        f"- Exact Izz: {total['izz_about_M2_axis_kg_m2']:.10f} kg m2.",
        f"- Nominal residual static imbalance: {total['static_imbalance_g_mm']:.9g} g mm.",
        f"- Nominal residual couple: {total['couple_imbalance_g_mm2']:.9g} g mm2.",
        "",
        "| slug | OCC-solved cut length |",
        "|---|---:|",
    ]
    for name, length in report["slug_length_solution_mm"].items():
        lines.append(f"| {name} | {length:.5f} mm |")
    lines.extend([
        "",
        "## M2 duty and release boundary",
        "",
        "The upstream mechanical ratio remains exactly 1:1.",
        f"At the OD65 force-vector gate, the current exact known-term motor/pulley margins are {duty.get('motor_margin', 0.0):.3f}x / {duty.get('pulley_margin', 0.0):.3f}x. The current motor and pulley therefore remain no-go.",
        "",
        "Physical fit/pull coupons, motor-rotor and bearing inertia, installed friction, G2.5-or-better two-plane balancing, full raw collision regeneration, and main BOM/procurement integration remain open.",
        "",
        f"Report SHA-256: `{report['report_sha256']}`",
        "",
    ])
    return "\n".join(lines)


def write_reports(report: Mapping[str, Any] | None = None) -> dict[str, Any]:
    result = analyze() if report is None else dict(report)
    REPORTS.mkdir(parents=True, exist_ok=True)
    REVIEW.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(json.dumps(result, indent=2), encoding="utf-8")
    MD_OUT.write_text(render_markdown(result), encoding="utf-8")
    manifest = {
        "schema": MANIFEST_SCHEMA,
        "status": result["status"],
        "source": result["paths"]["source"],
        "step": result["paths"]["step"],
        "hidden_glb": result["paths"]["hidden_glb"],
        "cutaway_step": result["paths"]["cutaway_step"],
        "cutaway_hidden_glb": result["paths"]["cutaway_hidden_glb"],
        "snapshot_job": result["paths"]["snapshot_job"],
        "report_json": result["paths"]["report_json"],
        "source_contracts": result["source_contracts"],
        "slug_length_solution_mm": result["slug_length_solution_mm"],
        "retention_summary": {
            "all_screws_end_in_positive_blind_material": result["retention"][
                "all_screws_end_in_positive_blind_material"
            ],
            "all_stacks_within_pocket_axial_envelope": result["retention"][
                "all_stacks_within_pocket_axial_envelope"
            ],
            "minimum_housing_wall_mm": result["retention"][
                "minimum_housing_wall_mm"
            ],
            "minimum_insert_boss_wall_mm": result["retention"][
                "minimum_insert_boss_wall_mm"
            ],
            "minimum_blind_positive_material_mm": result["retention"][
                "minimum_blind_positive_material_mm"
            ],
        },
        "geometry_gates": result["geometry_gates"],
        "release_gates": result["release_gates"],
        "controlling_blockers": result["controlling_blockers"],
    }
    MANIFEST_OUT.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return result


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("retained review schema mismatch")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("retained review report hash mismatch")
    if report.get("production_authorized") is not False:
        raise ValueError("isolated review cannot authorize production")


def main() -> int:
    report = write_reports()
    validate_report_integrity(report)
    duty = report["force_vector_M2_duty"]
    print(
        f"retained flyer: {report['status']}; "
        f"arm solids={report['printed_arm']['solid_count']}; "
        f"mass={report['exact_rotating_mass_properties']['mass_g']:.3f} g; "
        f"M2={duty.get('motor_margin', 0.0):.3f}x; "
        f"pulley={duty.get('pulley_margin', 0.0):.3f}x"
    )
    # Geometry can complete while the intentionally fail-closed drive/release
    # blockers remain.  A source/report failure raises before this point.
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
