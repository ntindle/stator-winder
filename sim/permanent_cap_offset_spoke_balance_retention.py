"""Fail-closed mass, balance, and retained-weight reconciliation.

This study consumes the frozen isolated offset-spoke review without editing
``cad/permanent_cap_offset_spoke_review.py``.  The current review's tungsten
cylinders are intentionally *not* retained.  An earlier version of this study
modeled a lightweight counterrail and four independently machined ASTM-B777
correction slugs, but incorrectly passed that proposal.  Exact review found
that its retainers protrude behind their floors, leave only 0.706947 mm to the
shifted block, and violate the 2.4 mm printed-wall rule at the front pockets.
That geometry and its nominal balance solution are retained only as rejected
witnesses.

The report now fails closed and publishes a corrected M3x6 in-envelope stack
as a successor dimensional contract.  It does not authorize focused CAD until
that contract has been modeled, rebalanced with weighed spacers/hardware, and
checked against the actual wire-force vectors and complete rotating inertia.

The nominal slug lengths solve both complex static imbalance and complex
couple imbalance from exact OCC mass properties.  Production integration and
the main BOM remain outside this bounded review.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from functools import lru_cache
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable, Mapping

import numpy as np
from build123d import Compound, Part, Pos, Rot
from scipy.optimize import root


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORTS = ROOT / "out" / "reports"
REVIEW = ROOT / "out" / "review"
SOURCE_REVIEW = CAD / "permanent_cap_offset_spoke_review.py"
SOURCE_STEP = REVIEW / "permanent_cap_offset_spoke_review.step"
SOURCE_REPORT = REPORTS / "permanent_cap_offset_spoke_review.json"
LOADS_SOURCE = CAD / "loads.py"
LOADS_REPORT = REPORTS / "loads.json"
HARDWARE_SOURCE = CAD / "hardware.py"
HARDWARE_AUDIT = REPORTS / "m2_m3_hardware_audit.json"
WIRE_FORCE_SOURCE = HERE / "permanent_cap_offset_spoke_wire_force_torque.py"
WIRE_FORCE_REPORT = (
    REPORTS / "permanent_cap_offset_spoke_wire_force_torque.json"
)
JSON_OUT = REPORTS / "permanent_cap_offset_spoke_balance_retention.json"
MD_OUT = REPORTS / "permanent_cap_offset_spoke_balance_retention.md"

for path in (CAD, HERE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import cots  # noqa: E402
import hardware  # noqa: E402
import loads  # noqa: E402
from params import PARAMS as P  # noqa: E402
import permanent_cap_offset_spoke_review as review  # noqa: E402
import permanent_cap_offset_spoke_wire_force_torque as wire_force  # noqa: E402


SCHEMA = "permanent-cap-offset-spoke-balance-retention/v1"
PETG_DENSITY_G_CM3 = 1.27
ALUMINUM_DENSITY_G_CM3 = 2.70
STEEL_DENSITY_G_CM3 = 7.85
BRASS_DENSITY_G_CM3 = 8.50
CERAMIC_DENSITY_G_CM3 = 3.90
# McMaster ultra-dense ASTM-B777 rod is listed at 0.668 lb/in^3.
TUNGSTEN_DENSITY_G_CM3 = 18.49

RAIL_WIDTH_MM = 4.0
RAIL_THICKNESS_MM = 3.0
RAIL_Z0_MM = -34.5
RAIL_Z1_MM = -31.5
RAIL_Y0_MM = -58.0
RAIL_Y1_MM = -7.0
TOWER_X_HALF_MM = 2.0
TOWER_Y0_MM = -60.0
TOWER_Y1_MM = -56.0
TOWER_Z1_MM = -12.0
FRONT_BOSS_RADIUS_MM = 7.15
FRONT_POCKET_RADIUS_MM = 6.50
FRONT_BOSS_Z0_MM = -21.0
FRONT_BOSS_Z1_MM = -12.0
FRONT_X_MM = 7.0
FRONT_Y_MM = -58.0
REAR_X_MM = 9.0
REAR_Y_MM = -25.0

SLUG_RADIUS_MM = 12.7 / 2.0
SLUG_BORE_RADIUS_MM = 7.8 / 2.0
SLUG_RADIAL_CLEARANCE_MM = FRONT_POCKET_RADIUS_MM - SLUG_RADIUS_MM
POCKET_FLOOR_MM = 1.0
SLUG_REAR_GAP_MM = 0.10
RETAINER_FACE_RADIUS_MM = 6.90
RETAINER_FACE_THICKNESS_MM = 1.20
RETAINER_BOSS_RADIUS_MM = 3.80
RETAINER_BOSS_REAR_REACH_MM = 4.80
INSERT_POCKET_RADIUS_MM = 2.40
SCREW_LENGTH_MM = 8.0
INSERT_LENGTH_MM = 4.30
SCREW_TIP_BLIND_CAP_MM = 0.60
LENGTH_TOLERANCE_MM = 0.05
RPM = 300.0
OMEGA_RAD_S = 2.0 * math.pi * RPM / 60.0
ACCEL_RAD_S2 = 200.0
WIRE_TENSION_N = 10.0
RETENTION_SAFETY_FACTOR = 3.0
MOTOR_NAME = "NEMA17 McMaster 6627T421 encoder motor @24V (M2)"


@dataclass(frozen=True)
class Pocket:
    id: str
    plane: str
    x_mm: float
    y_mm: float
    rear_z_mm: float

    @property
    def slug_rear_z_mm(self) -> float:
        return self.rear_z_mm + POCKET_FLOOR_MM + SLUG_REAR_GAP_MM


POCKETS = (
    Pocket("rear_left", "rear", -REAR_X_MM, REAR_Y_MM,
           review.SPOKE_REAR_Z_MM),
    Pocket("rear_right", "rear", REAR_X_MM, REAR_Y_MM,
           review.SPOKE_REAR_Z_MM),
    Pocket("front_left", "front", -FRONT_X_MM, FRONT_Y_MM,
           FRONT_BOSS_Z0_MM),
    Pocket("front_right", "front", FRONT_X_MM, FRONT_Y_MM,
           FRONT_BOSS_Z0_MM),
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} is not a JSON object")
    return value


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    body = deepcopy(dict(payload))
    body.pop("report_sha256", None)
    return hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def _density(material: str) -> float:
    return {
        "PETG": PETG_DENSITY_G_CM3,
        "aluminum": ALUMINUM_DENSITY_G_CM3,
        "steel": STEEL_DENSITY_G_CM3,
        "brass": BRASS_DENSITY_G_CM3,
        "ceramic": CERAMIC_DENSITY_G_CM3,
        "ASTM-B777 tungsten alloy": TUNGSTEN_DENSITY_G_CM3,
    }[material]


def _properties(name: str, shape: Part, material: str) -> dict[str, Any]:
    volume = float(shape.volume)
    rho = _density(material) / 1000.0
    mass = volume * rho
    center = shape.center()
    x, y, z = float(center.X), float(center.Y), float(center.Z)
    izz = (
        float(shape.matrix_of_inertia[2][2]) + volume * (x * x + y * y)
    ) * rho
    return {
        "name": name,
        "material": material,
        "density_g_cm3": _density(material),
        "volume_mm3": volume,
        "mass_g": mass,
        "center_of_mass_mm": [x, y, z],
        "static_first_moment_g_mm": [mass * x, mass * y],
        "couple_first_moment_g_mm2": [mass * x * z, mass * y * z],
        "izz_about_M2_axis_g_mm2": izz,
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
        "mass_g": mass,
        "static_first_moment_g_mm": [ux, uy],
        "static_imbalance_g_mm": math.hypot(ux, uy),
        "static_angle_deg": math.degrees(math.atan2(uy, ux)),
        "couple_first_moment_g_mm2": [zx, zy],
        "couple_imbalance_g_mm2": math.hypot(zx, zy),
        "izz_about_M2_axis_g_mm2": izz,
        "izz_about_M2_axis_kg_m2": izz * 1.0e-9,
    }


def counterrail_attachment() -> Part:
    """Light integral successor overlay; never mutates the frozen source."""

    parts = [
        review._box(
            -RAIL_WIDTH_MM / 2.0, RAIL_WIDTH_MM / 2.0,
            RAIL_Y0_MM, RAIL_Y1_MM,
            RAIL_Z0_MM, RAIL_Z1_MM,
            "counterbalance_deep_4x3_rail",
        ),
        review._box(
            -TOWER_X_HALF_MM, TOWER_X_HALF_MM,
            TOWER_Y0_MM, TOWER_Y1_MM,
            RAIL_Z0_MM, TOWER_Z1_MM,
            "counterbalance_R58_outboard_tower",
        ),
    ]
    for sign in (-1.0, 1.0):
        x = sign * FRONT_X_MM
        boss = review._cyl_z(
            FRONT_BOSS_RADIUS_MM,
            FRONT_BOSS_Z0_MM,
            FRONT_BOSS_Z1_MM,
            x=x, y=FRONT_Y_MM,
        )
        boss -= review._cyl_z(
            FRONT_POCKET_RADIUS_MM,
            FRONT_BOSS_Z0_MM + POCKET_FLOOR_MM,
            FRONT_BOSS_Z1_MM + 0.5,
            x=x, y=FRONT_Y_MM,
        )
        web = review._bar_xy(
            (sign * 1.5, -57.0), (x, FRONT_Y_MM), 3.0,
            FRONT_BOSS_Z0_MM, FRONT_BOSS_Z1_MM,
        )
        parts.extend((boss, web))
    result = parts[0]
    for part in parts[1:]:
        result += part
    result.label = "light_integral_two_plane_counterbalance_attachment"
    return result


def retention_screw(pocket: Pocket) -> Part:
    result = hardware.place(
        hardware.countersunk_screw("M3", SCREW_LENGTH_MM),
        (pocket.x_mm, pocket.y_mm, pocket.rear_z_mm),
        axis="-z",
        label=f"{pocket.id}_iso10642_m3x8_flush_screw",
    )
    return result


def balanced_arm_shell() -> Part:
    """Copied review arm plus overlay and exact four flush screw seats."""

    result = review.offset_spoke_arm() + counterrail_attachment()
    for pocket in POCKETS:
        # The exact screw envelope is the minimum machining cut.  The focused
        # CAD report declares +0.10 mm radial process clearance separately.
        result -= retention_screw(pocket)
    solids = list(result.solids())
    if len(solids) > 1:
        # Exact fastener-envelope subtraction can leave the four conical
        # countersink cores as isolated zero-function islands.  They are not
        # connected printable structure; retain the unique load-bearing body.
        result = max(solids, key=lambda solid: float(solid.volume))
    if len(list(result.solids())) != 1:
        raise RuntimeError("balanced arm shell is not one load-bearing solid")
    result.label = "offset_spoke_arm_with_integral_two_plane_counterrail"
    return result


def tungsten_slug(pocket: Pocket, length_mm: float) -> Part:
    if not 0.5 <= length_mm <= 6.5:
        raise ValueError(f"{pocket.id} slug length {length_mm} is not buildable")
    z0 = pocket.slug_rear_z_mm
    result = review._cyl_z(
        SLUG_RADIUS_MM, z0, z0 + length_mm,
        x=pocket.x_mm, y=pocket.y_mm,
    )
    result -= review._cyl_z(
        SLUG_BORE_RADIUS_MM, z0 - 0.1, z0 + length_mm + 0.1,
        x=pocket.x_mm, y=pocket.y_mm,
    )
    result.label = f"{pocket.id}_machined_tungsten_slug_L{length_mm:.4f}"
    return result


def _insert_z0(pocket: Pocket) -> float:
    screw_tip = pocket.rear_z_mm + SCREW_LENGTH_MM
    return screw_tip - SCREW_TIP_BLIND_CAP_MM - INSERT_LENGTH_MM


def retention_insert(pocket: Pocket) -> Part:
    result = hardware.place(
        hardware.heat_set_insert("M3", length="short"),
        (pocket.x_mm, pocket.y_mm, _insert_z0(pocket)),
        axis="+z",
        label=f"{pocket.id}_mcmaster_94459A130_insert",
    )
    return result


def retainer_cap(pocket: Pocket, length_mm: float) -> Part:
    slug_front = pocket.slug_rear_z_mm + length_mm
    face_z0 = slug_front + 0.10
    screw_tip = pocket.rear_z_mm + SCREW_LENGTH_MM
    boss_z0 = min(face_z0 - RETAINER_BOSS_REAR_REACH_MM,
                  _insert_z0(pocket) - 0.20)
    boss_z1 = max(
        face_z0 + RETAINER_FACE_THICKNESS_MM,
        screw_tip + SCREW_TIP_BLIND_CAP_MM,
    )
    face = review._cyl_z(
        RETAINER_FACE_RADIUS_MM,
        face_z0,
        face_z0 + RETAINER_FACE_THICKNESS_MM,
        x=pocket.x_mm, y=pocket.y_mm,
    )
    boss = review._cyl_z(
        RETAINER_BOSS_RADIUS_MM,
        boss_z0,
        boss_z1,
        x=pocket.x_mm, y=pocket.y_mm,
    )
    cap = face + boss
    # Exact insert pocket plus blind M3 tip pocket.  The pocket ends before
    # boss_z1, leaving the explicit blind cap thickness.
    cap -= review._cyl_z(
        INSERT_POCKET_RADIUS_MM,
        _insert_z0(pocket) - 0.10,
        screw_tip + 0.05,
        x=pocket.x_mm, y=pocket.y_mm,
    )
    cap.label = f"{pocket.id}_positive_volume_blind_retainer_cap"
    return cap


def correction_parts(lengths_mm: Iterable[float]) -> list[tuple[str, Part, str]]:
    lengths = list(map(float, lengths_mm))
    if len(lengths) != len(POCKETS):
        raise ValueError("four correction lengths are required")
    result: list[tuple[str, Part, str]] = []
    for pocket, length in zip(POCKETS, lengths):
        result.extend((
            (f"{pocket.id}_tungsten_slug", tungsten_slug(pocket, length),
             "ASTM-B777 tungsten alloy"),
            (f"{pocket.id}_retainer_cap", retainer_cap(pocket, length),
             "PETG"),
            (f"{pocket.id}_insert", retention_insert(pocket), "brass"),
            (f"{pocket.id}_screw", retention_screw(pocket), "steel"),
        ))
    return result


def _existing_rotating_hardware() -> list[tuple[str, Part, str]]:
    result: list[tuple[str, Part, str]] = []

    def add(name: str, shape: Part, material: str) -> None:
        shape.label = name
        result.append((name, shape, material))

    for name, origin, screw_axis, insert_axis in (
        ("arm_neg_y", (0.0, -14.0, -46.0), "-y", "+y"),
        ("arm_pos_x", (14.0, 0.0, -46.0), "+x", "-x"),
    ):
        add(
            f"{name}_m3x8_set_screw",
            hardware.place(
                hardware.set_screw("M3", 8.0), origin, axis=screw_axis,
            ),
            "steel",
        )
        add(
            f"{name}_m3_standard_insert",
            hardware.place(
                hardware.heat_set_insert("M3", length="standard"),
                origin, axis=insert_axis,
            ),
            "brass",
        )
    pulley_origin = (0.0, -10.4, -88.75)
    add(
        "flyer_pulley_m3x8_set_screw",
        hardware.place(
            hardware.set_screw("M3", 8.0), pulley_origin, axis="-y",
        ),
        "steel",
    )
    add(
        "flyer_pulley_m3_short_insert",
        hardware.place(
            hardware.heat_set_insert_m3_3p4(), pulley_origin, axis="+y",
        ),
        "brass",
    )
    return result


def base_rotating_parts() -> list[tuple[str, Part, str]]:
    shifted = review.shifted_static_module_parts()
    parts: list[tuple[str, Part, str]] = [
        ("balanced_arm_shell", balanced_arm_shell(), "PETG"),
        ("extended_hollow_shaft", review.extended_hollow_shaft(), "aluminum"),
        ("shifted_flyer_pulley", shifted["flyer_pulley"], "PETG"),
        ("R64_ceramic_toroid", review.tip_toroid(), "ceramic"),
        ("m2_inner_rear_shim", Pos(0.0, 0.0, -85.25) *
         cots.tube_spacer(18.0, 12.05, 0.5), "steel"),
        ("m2_inner_center_spacer", Pos(0.0, 0.0, -71.5) *
         cots.tube_spacer(17.8, 12.05, 11.0), "steel"),
        ("m2_inner_front_spacer", Pos(0.0, 0.0, -56.0) *
         cots.tube_spacer(18.0, 12.05, 4.0), "steel"),
    ]
    parts.extend(_existing_rotating_hardware())
    return parts


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


def _residual(lengths_mm: np.ndarray) -> np.ndarray:
    total = _sum_properties(mass_rows(lengths_mm))
    ux, uy = total["static_first_moment_g_mm"]
    zx, zy = total["couple_first_moment_g_mm2"]
    # Scale couple terms so scipy sees similarly sized residuals.
    return np.asarray((ux, uy, zx / 40.0, zy / 40.0), dtype=float)


def solve_slug_lengths() -> list[float]:
    solution = root(_residual, np.asarray((3.8, 2.2, 1.2, 2.2)))
    if not solution.success:
        raise RuntimeError(f"two-plane balance solve failed: {solution.message}")
    values = [float(value) for value in solution.x]
    if any(not 0.5 <= value <= 6.5 for value in values):
        raise RuntimeError(f"two-plane balance produced unbuildable lengths {values}")
    if float(np.linalg.norm(_residual(np.asarray(values)))) > 1.0e-5:
        raise RuntimeError("two-plane balance residual did not converge")
    return values


def _plane_summary(rows: Iterable[Mapping[str, Any]], plane: str) -> dict[str, Any]:
    names = {pocket.id for pocket in POCKETS if pocket.plane == plane}
    selected = [
        row for row in rows
        if any(str(row["name"]).startswith(name) for name in names)
    ]
    return {"plane": plane, "parts": len(selected), **_sum_properties(selected)}


def _retention_audit(lengths: list[float], rows: list[dict[str, Any]]) -> dict[str, Any]:
    by_name = {str(row["name"]): row for row in rows}
    stacks = []
    for pocket, length in zip(POCKETS, lengths):
        slug = by_name[f"{pocket.id}_tungsten_slug"]
        package_mass = sum(
            float(by_name[f"{pocket.id}_{suffix}"]["mass_g"])
            for suffix in ("tungsten_slug", "retainer_cap", "insert", "screw")
        )
        radius = math.hypot(pocket.x_mm, pocket.y_mm)
        force = package_mass / 1000.0 * radius / 1000.0 * OMEGA_RAD_S ** 2
        design_force = RETENTION_SAFETY_FACTOR * force
        screw_proof = 5.03 * 580.0  # M3 tensile stress area x class-8.8 proof
        pocket_bearing_area = max(1.0, 2.0 * SLUG_RADIUS_MM * length)
        bearing_stress = design_force / pocket_bearing_area
        screw_tip = pocket.rear_z_mm + SCREW_LENGTH_MM
        insert_z0 = _insert_z0(pocket)
        face_z0 = pocket.slug_rear_z_mm + length + 0.10
        boss_front = max(
            face_z0 + RETAINER_FACE_THICKNESS_MM,
            screw_tip + SCREW_TIP_BLIND_CAP_MM,
        )
        if pocket.plane == "rear":
            cap_rear = float(
                review.cap_collision_support_envelope(1).bounding_box().min.Z
            )
            forward_clearance = cap_rear - boss_front
        else:
            forward_clearance = None
        cap_shape = retainer_cap(pocket, length)
        retainer_rear_z = float(cap_shape.bounding_box().min.Z)
        rear_projection = max(0.0, pocket.rear_z_mm - retainer_rear_z)
        stacks.append({
            "id": pocket.id,
            "plane": pocket.plane,
            "center_xy_mm": [pocket.x_mm, pocket.y_mm],
            "slug_length_mm": length,
            "slug_mass_g": slug["mass_g"],
            "package_mass_g": package_mass,
            "radial_center_mm": radius,
            "centrifugal_force_at_300rpm_N": force,
            "three_x_design_force_N": design_force,
            "M3_class_8p8_proof_capacity_N": screw_proof,
            "screw_capacity_margin": screw_proof / design_force,
            "pocket_sidewall_bearing_area_mm2": pocket_bearing_area,
            "three_x_sidewall_bearing_stress_MPa": bearing_stress,
            "full_insert_engagement_mm": INSERT_LENGTH_MM,
            "screw_shank_interval_z_mm": [pocket.rear_z_mm, screw_tip],
            "insert_interval_z_mm": [insert_z0, insert_z0 + INSERT_LENGTH_MM],
            "blind_cap_ahead_of_screw_tip_mm": boss_front - screw_tip,
            "rear_head_flush": True,
            "slug_radially_captured_by_pocket": True,
            "front_retainer_face_overlaps_slug_radially_mm": (
                RETAINER_FACE_RADIUS_MM - SLUG_BORE_RADIUS_MM
            ),
            "rear_stack_forward_clearance_to_cap_mm": forward_clearance,
            "pocket_rear_floor_z_mm": pocket.rear_z_mm,
            "retainer_minimum_z_mm": retainer_rear_z,
            "retainer_projection_behind_floor_mm": rear_projection,
            "closed_load_path": (
                "flush screw head -> positive pocket floor -> annular tungsten "
                "slug -> retainer face/boss -> full insert -> blind cap"
            ),
            "fastener_terminates_in_air": False,
            "stack_is_geometrically_acceptable": rear_projection <= 1.0e-9,
        })
    front_exterior_wall = FRONT_BOSS_RADIUS_MM - FRONT_POCKET_RADIUS_MM
    front_center_septum = 2.0 * FRONT_X_MM - 2.0 * FRONT_POCKET_RADIUS_MM
    return {
        "status": "REJECTED_PROPOSED_STACK",
        "finding": (
            "The modeled positive-material cap does not terminate in air, but "
            "it projects behind the pocket floor into the shifted-block "
            "clearance. The front bosses also violate wall and septum rules."
        ),
        "stacks": stacks,
        "minimum_screw_capacity_margin": min(
            row["screw_capacity_margin"] for row in stacks
        ),
        "maximum_three_x_sidewall_bearing_stress_MPa": max(
            row["three_x_sidewall_bearing_stress_MPa"] for row in stacks
        ),
        "minimum_blind_cap_mm": min(
            row["blind_cap_ahead_of_screw_tip_mm"] for row in stacks
        ),
        "minimum_rear_stack_forward_cap_clearance_mm": min(
            row["rear_stack_forward_clearance_to_cap_mm"]
            for row in stacks if row["plane"] == "rear"
        ),
        "all_four_fasteners_end_in_positive_material": all(
            not row["fastener_terminates_in_air"] for row in stacks
        ),
        "all_four_retainers_stay_ahead_of_their_floors": all(
            row["retainer_projection_behind_floor_mm"] <= 1.0e-9
            for row in stacks
        ),
        "maximum_retainer_projection_behind_floor_mm": max(
            row["retainer_projection_behind_floor_mm"] for row in stacks
        ),
        "front_boss_exterior_wall_mm": front_exterior_wall,
        "front_pair_center_septum_mm": front_center_septum,
        "required_minimum_printed_wall_mm": float(P.min_wall),
        "assembly_note": (
            "The current review cylinders have no retention. This rejected "
            "four-stack proposal must not be regenerated as CAD; consume the "
            "corrected successor retention contract instead."
        ),
    }


def _collision_audit(lengths: list[float]) -> dict[str, Any]:
    arm = balanced_arm_shell()
    correction_shapes = [shape for _, shape, _ in correction_parts(lengths)]
    correction = Compound(children=correction_shapes)
    caps = Compound(children=[
        review.cap_collision_support_envelope(1),
        review.cap_collision_support_envelope(-1),
    ])
    static = review.shifted_static_module_parts()
    block = static["block"]
    frame = review.frame_rear_boundary_proxy()
    exact = {
        "counterrail_and_arm_to_cap_mm": float(arm.distance_to(caps)),
        "counterrail_and_arm_to_block_mm": float(arm.distance_to(block)),
        "correction_packages_to_cap_mm": float(correction.distance_to(caps)),
        "correction_packages_to_block_mm": float(correction.distance_to(block)),
        "correction_packages_to_frame_proxy_mm": float(
            correction.distance_to(frame)
        ),
    }
    cap_bb = caps.bounding_box()
    cap_radial_bound = math.hypot(
        max(abs(float(cap_bb.min.X)), abs(float(cap_bb.max.X))),
        max(abs(float(cap_bb.min.Y)), abs(float(cap_bb.max.Y))),
    )
    front_inner_radius = math.hypot(FRONT_X_MM, FRONT_Y_MM) - max(
        FRONT_BOSS_RADIUS_MM, RETAINER_FACE_RADIUS_MM,
    )
    cap_front_radial_clearance = front_inner_radius - cap_radial_bound
    cap_rear_z = float(cap_bb.min.Z)
    deep_front_z = max(RAIL_Z1_MM, max(
        float(shape.bounding_box().max.Z)
        for name, shape, _ in correction_parts(lengths)
        if name.startswith("rear_")
    ))
    deep_axial_clearance = cap_rear_z - deep_front_z
    block_front_z = float(block.bounding_box().max.Z)
    deep_rear_z = min(
        RAIL_Z0_MM,
        min(pocket.rear_z_mm for pocket in POCKETS[:2]),
    )
    block_axial_clearance = deep_rear_z - block_front_z
    rotating_outer_radius = max(
        67.0,
        math.hypot(FRONT_X_MM, FRONT_Y_MM) + FRONT_BOSS_RADIUS_MM,
    )
    frame_radial_clearance = P.frame_w / 2.0 - rotating_outer_radius
    continuous = {
        "method": (
            "REJECTED prior rotation-invariant bound: it considered nominal "
            "rail/floor planes but omitted retainer volume protruding behind "
            "those planes. Exact package-to-block distance controls instead."
        ),
        "certificate_valid": False,
        "angle_domain_deg": [0.0, 360.0],
        "cap_deep_axial_clearance_mm": deep_axial_clearance,
        "cap_front_radial_clearance_mm": cap_front_radial_clearance,
        "block_deep_axial_clearance_mm": block_axial_clearance,
        "frame_radial_clearance_mm": frame_radial_clearance,
        "prior_incomplete_minimum_continuous_clearance_mm": min(
            deep_axial_clearance,
            cap_front_radial_clearance,
            block_axial_clearance,
            frame_radial_clearance,
        ),
        "exact_correction_packages_to_block_at_controlling_pose_mm": exact[
            "correction_packages_to_block_mm"
        ],
        "minimum_certified_clearance_mm": min(
            exact["correction_packages_to_block_mm"],
            deep_axial_clearance,
            cap_front_radial_clearance,
            block_axial_clearance,
            frame_radial_clearance,
        ),
    }
    return {"exact_controlling_pose_mm": exact,
            "continuous_360_certificate": continuous}


def _tolerance_and_loads(lengths: list[float], nominal_rows: list[dict[str, Any]],
                         force_report: Mapping[str, Any]) -> dict[str, Any]:
    nominal = _sum_properties(nominal_rows)
    corner_rows = []
    for signs in itertools.product((-1.0, 1.0), repeat=4):
        candidate = [
            length + sign * LENGTH_TOLERANCE_MM
            for length, sign in zip(lengths, signs)
        ]
        total = _sum_properties(mass_rows(candidate))
        corner_rows.append({
            "signs": list(signs),
            "lengths_mm": candidate,
            "static_imbalance_g_mm": total["static_imbalance_g_mm"],
            "couple_imbalance_g_mm2": total["couple_imbalance_g_mm2"],
        })
    worst_static = max(row["static_imbalance_g_mm"] for row in corner_rows)
    worst_couple = max(row["couple_imbalance_g_mm2"] for row in corner_rows)
    residual_force = worst_static * 1.0e-6 * OMEGA_RAD_S ** 2
    bearing_span_m = abs(P.flyer_brg_front_z - P.flyer_brg_rear_z) / 1000.0
    couple_force = worst_couple * 1.0e-9 * OMEGA_RAD_S ** 2 / bearing_span_m

    # This is retained only to quantify sensitivity of the rejected M3x8
    # proposal.  It is not a release inertia because the actual transition
    # guide, adhesive, motor rotor, motor-pulley set screws, corrected M3x6
    # retainers, and locating spacers are not in this model.
    worst_izz = nominal["izz_about_M2_axis_kg_m2"] * 1.05
    t_acc = worst_izz * ACCEL_RAD_S2
    t_energy = 2.0 * (1.5 * WIRE_TENSION_N * 0.060 / (2.0 * math.pi))
    rejected_energy_required = t_acc + t_energy
    launch = force_report["duty_cases"]["GOAL_launch_OD65"]
    return {
        "status": "REJECTED_NOMINAL_STACK_AND_FAIL_CLOSED_FORCE_DUTY",
        "slug_cut_length_tolerance_mm": LENGTH_TOLERANCE_MM,
        "corner_count": len(corner_rows),
        "worst_static_imbalance_g_mm": worst_static,
        "worst_static_force_at_300rpm_N": residual_force,
        "worst_couple_imbalance_g_mm2": worst_couple,
        "worst_couple_bearing_reaction_at_300rpm_N": couple_force,
        "post_assembly_trim_required": True,
        "trim_method": (
            "weigh the printed arm and four cut slugs, recompute the four "
            "lengths, then verify on a two-plane balancer; shorten the named "
            "slug or add weighed M3 DIN-988 shim stock before threadlocking"
        ),
        "inertia_tolerance_factor": 1.05,
        "rejected_stack_worst_case_izz_kg_m2": worst_izz,
        "acceleration_rad_s2": ACCEL_RAD_S2,
        "rejected_stack_acceleration_torque_nm": t_acc,
        "rejected_energy_only_wire_torque_nm": t_energy,
        "rejected_energy_only_required_torque_nm": rejected_energy_required,
        "force_vector_scope": launch["scope"],
        "known_rotating_inertia_upper_bound_kg_m2": launch[
            "known_rotating_inertia_upper_bound_kg_m2"
        ],
        "known_acceleration_torque_nm": launch["known_acceleration_torque_nm"],
        "friction_allowance_nm": launch["friction_allowance_nm"],
        "wire_line_of_action_distance_mm": launch["force_vector"]
        ["effective_line_of_action_distance_mm"],
        "wire_force_vector_torque_nm": launch["force_vector"]
        ["wire_torque_at_10N_nm"],
        "required_torque_at_300rpm_10N_nm": launch[
            "known_load_required_torque_nm"
        ],
        "selected_motor": MOTOR_NAME,
        "selected_motor_available_torque_at_300rpm_nm": launch[
            "selected_motor_available_torque_at_300rpm_nm"
        ],
        "selected_motor_margin": launch["known_load_motor_margin"],
        "selected_pulley": "NBK P40-2GT-BLP-6C-5",
        "selected_pulley_capacity_nm": launch["selected_pulley_capacity_nm"],
        "selected_pulley_margin": launch["known_load_pulley_margin"],
        "motor_rotor_inertia_missing": True,
        "reported_margins_are_optimistic": True,
        "wire_force_report_sha256": force_report["report_sha256"],
    }


def analyze() -> dict[str, Any]:
    source_report = _load(SOURCE_REPORT)
    loads_report = _load(LOADS_REPORT)
    hardware_audit = _load(HARDWARE_AUDIT)
    lengths = solve_slug_lengths()
    rows = mass_rows(lengths)
    total = _sum_properties(rows)
    rear = _plane_summary(rows, "rear")
    front = _plane_summary(rows, "front")
    retention = _retention_audit(lengths, rows)
    collision = _collision_audit(lengths)
    force_report = wire_force.analyze()
    duty = _tolerance_and_loads(lengths, rows, force_report)

    ux, uy = total["static_first_moment_g_mm"]
    zx, zy = total["couple_first_moment_g_mm2"]
    exact_package_block = collision["exact_controlling_pose_mm"][
        "correction_packages_to_block_mm"
    ]
    gates = {
        "frozen_offset_review_PASS": source_report["status"] == "PASS_REVIEW_ONLY",
        "current_review_slugs_correctly_classified_unretained": (
            source_report["provisional_balance_envelope"]
            ["retention_hardware_complete"] is False
        ),
        "current_hardware_schedule_and_audit_bound": hardware_audit["passed"] is True,
        "rejected_solve_has_four_positive_tungsten_lengths": all(
            0.5 <= value <= 6.5 for value in lengths
        ),
        "rejected_nominal_static_balance_math_closes": math.hypot(ux, uy) <= 1.0e-5,
        "rejected_nominal_couple_balance_math_closes": math.hypot(zx, zy) <= 1.0e-4,
        "centrifugal_load_is_not_the_controlling_retention_risk": (
            retention["minimum_screw_capacity_margin"] >= 3.0
            and retention["maximum_three_x_sidewall_bearing_stress_MPa"] <= 5.0
        ),
        "proposed_retainer_exact_package_to_block_clearance_ge_2p2mm": (
            exact_package_block >= 2.2
        ),
        "all_proposed_retainers_stay_ahead_of_pocket_floors": (
            retention["all_four_retainers_stay_ahead_of_their_floors"]
        ),
        "front_boss_minimum_exterior_wall_ge_2p4mm": (
            retention["front_boss_exterior_wall_mm"] >= float(P.min_wall)
        ),
        "front_pocket_center_septum_ge_2p4mm": (
            retention["front_pair_center_septum_mm"] >= float(P.min_wall)
        ),
        "corrected_M3x6_successor_geometry_modeled_in_CAD": False,
        "corrected_successor_hardware_spacers_and_adhesive_mass_bound": False,
        "corrected_successor_two_plane_balance_recomputed": False,
        "actual_transition_guide_and_adhesive_mass_bound": False,
        "bearing_inner_elements_and_selected_motor_rotor_inertia_bound": False,
        "motor_pulley_set_screws_modeled": False,
        "physical_two_plane_balance_to_G2p5_or_better_complete": False,
        "GOAL_OD65_force_vector_motor_margin_ge_2": (
            duty["selected_motor_margin"] >= 2.0
        ),
        "GOAL_OD65_force_vector_pulley_margin_ge_2": (
            duty["selected_pulley_margin"] >= 2.0
        ),
        "installed_M2_friction_measured_within_allowance": False,
        "NEMA23_candidate_dynamic_curve_at_300rpm_verified": False,
        "NEMA23_candidate_installed_geometry_and_inertia_verified": False,
        "standard_dense_stock_and_corrected_successor_hardware_identified": True,
    }
    blockers = [name for name, value in gates.items() if not value]
    status = "DESIGN_NO_GO"
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": status,
        "decision": "REJECT_M3X8_STACK__RETAINERS_OVER_OPEN_CLEARANCE__RECOMPUTE_M3X6_SUCCESSOR",
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "focused_counterweight_CAD_authorized": False,
        "current_review_truth": {
            "tungsten_slugs_attached": False,
            "finding": (
                "The stable source places two solid tungsten envelopes in "
                "open-front pockets over a 1 mm rear floor. It defines no "
                "screw, insert, nut, cover, lip, or press fit."
            ),
        },
        "rejected_proposed_geometry_mm": {
            "integral_rail_cross_section": [RAIL_WIDTH_MM, RAIL_THICKNESS_MM],
            "rail_y_span": [RAIL_Y0_MM, RAIL_Y1_MM],
            "rail_z_span": [RAIL_Z0_MM, RAIL_Z1_MM],
            "front_boss_radius": FRONT_BOSS_RADIUS_MM,
            "front_pocket_radius": FRONT_POCKET_RADIUS_MM,
            "tungsten_stock_diameter": 2.0 * SLUG_RADIUS_MM,
            "tungsten_bore_diameter": 2.0 * SLUG_BORE_RADIUS_MM,
            "slug_radial_clearance": SLUG_RADIAL_CLEARANCE_MM,
            "slug_lengths": {
                pocket.id: length for pocket, length in zip(POCKETS, lengths)
            },
        },
        "exact_rotating_mass_properties": {
            "status": "INCOMPLETE_AND_NOT_RELEASE_USABLE",
            "parts": rows,
            "part_count": len(rows),
            **total,
        },
        "two_plane_balance": {
            "status": "REJECTED_NOMINAL_SOLUTION__RECOMPUTE_REQUIRED",
            "equations": [
                "sum(m*(x+i*y)) = 0",
                "sum(m*(x+i*y)*z) = 0",
            ],
            "rear_plane": rear,
            "front_plane": front,
            "nominal_residual_static_g_mm": math.hypot(ux, uy),
            "nominal_residual_couple_g_mm2": math.hypot(zx, zy),
            "rejected_slug_length_solution_mm": {
                pocket.id: length for pocket, length in zip(POCKETS, lengths)
            },
            "centered_package_targets_for_corrected_recompute_g": {
                "rear_left": 5.776218,
                "rear_right": 4.929977,
                "front_left": 2.018484,
                "front_right": 2.516098,
            },
            "target_planes_z_mm": {"rear": -34.12, "front": -16.50},
            "package_target_warning": (
                "These are centered total-package targets, not slug cut "
                "lengths. Recompute after exact M3x6 screws, inserts, "
                "retainers, and weighed locating spacers are modeled."
            ),
            "physical_balance_requirement": {
                "maximum_G2p5_residual_g_mm": 6.465,
                "preferred_G1_residual_g_mm": 2.586,
                "weighing_resolution_g": 0.01,
                "radial_center_measurement_mm": 0.1,
                "axial_plane_measurement_mm": 0.05,
            },
        },
        "retention": retention,
        "collision": collision,
        "tolerance_and_M2_duty": duty,
        "wire_force_vector_torque": {
            "status": force_report["status"],
            "decision": force_report["decision"],
            "report_sha256": force_report["report_sha256"],
            "canonical_default_OD46": force_report["duty_cases"]
            ["canonical_default_OD46"],
            "GOAL_launch_OD65": force_report["duty_cases"]
            ["GOAL_launch_OD65"],
            "parametric_OD90_advisory": force_report["duty_cases"]
            ["parametric_OD90_advisory"],
            "unconstrained_direct_R64": force_report["duty_cases"]
            ["unconstrained_direct_R64"],
            "current_1_to_1_line_of_action_limits": force_report[
                "line_of_action_limits_for_current_1_to_1_drive"
            ],
            "drive_recommendation": force_report["drive_recommendation"],
        },
        "corrected_successor_retention_contract": {
            "status": "DIMENSIONAL_CONTRACT_ONLY__NOT_MODELED_OR_BALANCED",
            "pocket_body_axial_depth_mm": 8.0,
            "rear_floor_interval_from_pocket_rear_mm": [0.0, 1.0],
            "screw": {
                "selection": "McMaster 92125A126 / ISO 10642 M3x6",
                "head_diameter_mm": 6.0,
                "head_height_mm": 1.7,
                "head_interval_from_pocket_rear_mm": [0.0, 1.7],
                "tip_from_pocket_rear_mm": 6.0,
            },
            "insert": {
                "selection": "McMaster 94459A130 M3 x 4.3 heat-set insert",
                "interval_from_pocket_rear_mm": [1.7, 6.0],
                "full_engagement_mm": 4.3,
            },
            "printed_retainer_boss": {
                "outer_diameter_mm": 7.6,
                "interval_from_pocket_rear_mm": [1.0, 6.6],
                "head_relief_diameter_mm": 4.2,
                "head_relief_interval_mm": [1.0, 1.7],
                "insert_pilot_diameter_mm": 4.0,
                "insert_pilot_interval_mm": [1.7, 6.0],
                "rear_annulus_bottoms_on_floor": True,
                "rear_projection_allowed_mm": 0.0,
            },
            "retainer_face": {
                "interval_from_pocket_rear_mm": [6.6, 7.8],
                "thickness_mm": 1.2,
                "front_recess_mm": 0.2,
            },
            "positive_blind_cap_ahead_of_screw_tip_mm": 1.8,
            "slug_bore_diameter_mm": 7.8,
            "axial_location": (
                "weighed printed three-point spacers; include each exact mass "
                "and center in the balance solve"
            ),
            "rear_pair": {
                "centers_xy_mm": [[-9.0, -25.0], [9.0, -25.0]],
                "turned_slug_diameter_mm": 12.5,
                "pocket_diameter_mm": 12.8,
                "retainer_face_diameter_mm": 12.6,
                "housing_outer_radius_mm": 8.8,
                "inter_pocket_ligament_mm": 5.2,
            },
            "front_pair": {
                "centers_xy_mm": [[-7.0, -58.0], [7.0, -58.0]],
                "turned_slug_diameter_mm": 11.0,
                "pocket_diameter_mm": 11.3,
                "retainer_face_diameter_mm": 11.1,
                "housing_outer_radius_mm": 8.05,
                "center_septum_mm": 2.7,
            },
            "analytic_successor_clearance_witness_mm": {
                "rear_housing_to_cap": 4.8105,
                "rear_housing_to_block": 2.88,
                "rear_housing_to_frame": 34.7455,
                "front_housing_to_cap": 13.183,
                "front_housing_to_block": 20.5,
                "front_housing_to_frame": 19.9944,
                "minimum": 2.88,
            },
            "successor_balance_recompute_required": True,
            "focused_CAD_authorized": False,
        },
        "sourcing": {
            "dense_stock": {
                "selection": (
                    "McMaster 5995N71 (8 in) or 5995N72 (12 in), 1/2 in "
                    "ultra-dense corrosion-resistant ASTM-B777 tungsten rod"
                ),
                "density_g_cm3": TUNGSTEN_DENSITY_G_CM3,
                "machine": (
                    "cut four named lengths, drill/ream ID7.8, deburr and "
                    "0.2 mm max edge chamfer; weigh each finished slug"
                ),
                "url": "https://www.mcmaster.com/products/tungsten-alloy-rods/",
            },
            "rejected_retention_screw": {
                "selection": "ISO 10642 M3x8 countersunk socket screw, 4",
                "step_parts_id": "countersunk_socket_screw_m3_l0008_simple",
                "step_parts_sha256": (
                    "0d2403c11f30d1195509ba7b999949f7d26a4ea6563e96928f293ffd75f02ddb"
                ),
                "page": (
                    "https://www.step.parts/parts/"
                    "countersunk_socket_screw_m3_l0008_simple"
                ),
            },
            "corrected_successor_retention_screw": {
                "selection": "McMaster 92125A126 / ISO 10642 M3x6, 4",
                "reason": (
                    "the 6 mm screw and recessed in-envelope retainer remove "
                    "the rejected rear projection"
                ),
            },
            "insert": {
                "selection": "McMaster 94459A130 M3 x 4.3 heat-set insert, 4",
                "model": "existing cad/hardware.py bd_warehouse envelope",
                "step_parts_search": (
                    "canonical query 'M3 heat set insert 4.3' returned zero; "
                    "existing McMaster-backed parametric envelope retained"
                ),
            },
            "retainer_caps": (
                "4 corrected in-envelope PETG printed retainers with weighed "
                "three-point slug spacers; exact geometry not yet modeled"
            ),
            "thread_retention": (
                "medium-strength threadlocker after final two-plane trim; "
                "torque stripe and pre-run/10-minute/1-hour inspection"
            ),
        },
        "source_contracts": {
            "frozen_review_source_sha256": _sha256(SOURCE_REVIEW),
            "frozen_review_STEP_sha256": _sha256(SOURCE_STEP),
            "frozen_review_report_sha256": _sha256(SOURCE_REPORT),
            "loads_source_sha256": _sha256(LOADS_SOURCE),
            "loads_report_sha256": _sha256(LOADS_REPORT),
            "hardware_source_sha256": _sha256(HARDWARE_SOURCE),
            "hardware_audit_sha256": _sha256(HARDWARE_AUDIT),
            "wire_force_source_sha256": _sha256(WIRE_FORCE_SOURCE),
            "wire_force_report_sha256": force_report["report_sha256"],
            "selected_motor": loads_report["motors"]["m2"],
            "selected_pulley": loads_report["m2"]["pulley"]["selection"],
        },
        "gates": gates,
        "controlling_blockers": blockers,
        "limits": [
            "This is an isolated successor review and does not edit the frozen offset-spoke source.",
            "The M3x8 retainer geometry and its nominal slug-length solution are rejected witnesses, not build data.",
            "The corrected M3x6 contract must be regenerated and its balance recomputed from weighed printed and machined parts.",
            "A physical two-plane balance run remains mandatory before 300 RPM operation.",
            "Actual transition-guide/adhesive/set-screw mass, bearing inner-element inertia, motor rotor inertia, and installed friction remain unbounded.",
            "The current 1:1 drive fails the conservative GOAL OD65 >=2x force-vector motor and pulley gates.",
            "No main assembly, BOM, procurement schedule, settings, or release authority is changed.",
        ],
        "source_hashes": {
            "sim/permanent_cap_offset_spoke_balance_retention.py": _sha256(Path(__file__)),
            "cad/permanent_cap_offset_spoke_review.py": _sha256(SOURCE_REVIEW),
            "out/review/permanent_cap_offset_spoke_review.step": _sha256(SOURCE_STEP),
            "out/reports/permanent_cap_offset_spoke_review.json": _sha256(SOURCE_REPORT),
            "cad/loads.py": _sha256(LOADS_SOURCE),
            "cad/hardware.py": _sha256(HARDWARE_SOURCE),
            "out/reports/m2_m3_hardware_audit.json": _sha256(HARDWARE_AUDIT),
            "sim/permanent_cap_offset_spoke_wire_force_torque.py": _sha256(
                WIRE_FORCE_SOURCE
            ),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def render_markdown(report: Mapping[str, Any]) -> str:
    balance = report["two_plane_balance"]
    duty = report["tolerance_and_M2_duty"]
    collision = report["collision"]["continuous_360_certificate"]
    retention = report["retention"]
    corrected = report["corrected_successor_retention_contract"]
    lines = [
        "# Offset-spoke balance and counterweight retention reconciliation",
        "",
        f"Status: **{report['status']}** — `{report['decision']}`.",
        "",
        "## Current review truth",
        "",
        "The two tungsten cylinders in the stable offset-spoke review are unattached provisional envelopes. They sit in open-front pockets over a 1 mm rear floor; no fastener, insert, nut, cover, lip, or press fit exists.",
        "",
        "## Rejected M3x8 proposal",
        "",
        "The modeled M3x8 cap contains positive material, but that does not make the stack acceptable: its retainers project behind their pocket floors into the shifted-block clearance.",
        "",
        f"Exact package-to-block clearance is only {collision['exact_correction_packages_to_block_at_controlling_pose_mm']:.6f} mm. Maximum rear projection is {retention['maximum_retainer_projection_behind_floor_mm']:.6f} mm. The front exterior wall is {retention['front_boss_exterior_wall_mm']:.3f} mm and the front center septum is {retention['front_pair_center_septum_mm']:.3f} mm, both below the {retention['required_minimum_printed_wall_mm']:.1f} mm rule.",
        "",
        "Centrifugal load is not controlling; the load path, wall, insert pullout, loosening, and collision are.",
        "",
        "## Balance status",
        "",
        f"The old mathematical solve closes to {balance['nominal_residual_static_g_mm']:.8f} g mm static and {balance['nominal_residual_couple_g_mm2']:.8f} g mm2 couple, but its geometry is rejected. Its cut lengths are not build data.",
        "",
        "| corrected package target | total package mass |",
        "|---|---:|",
    ]
    for name, mass in balance[
            "centered_package_targets_for_corrected_recompute_g"].items():
        lines.append(f"| {name} | {mass:.6f} g |")
    lines.extend([
        "",
        "These are centered package targets, not slug lengths. Recompute after the exact M3x6 hardware, retainers, and weighed spacers exist, then physically balance to G2.5 or better.",
        "",
        "## Corrected dimensional contract",
        "",
        f"Use {corrected['screw']['selection']} with the 4.3 mm insert fully engaged from 1.70 to 6.00 mm, a retainer wholly inside the 8.00 mm pocket, and a 1.80 mm positive blind cap. Rear slugs are turned to OD12.50 in OD12.80 pockets; front slugs are turned to OD11.00 in OD11.30 pockets. The analytic successor envelope has {corrected['analytic_successor_clearance_witness_mm']['minimum']:.3f} mm minimum clearance, but no CAD or balance is authorized from that number alone.",
        "",
        "## Whole-flyer M2 duty",
        "",
        f"At the conservative GOAL OD65 line-of-action bound, known required torque is {duty['required_torque_at_300rpm_10N_nm']:.6f} Nm. The current exact-1:1 motor/pulley margins are only {duty['selected_motor_margin']:.3f}x / {duty['selected_pulley_margin']:.3f}x, before unknown motor rotor inertia. Both miss 2x.",
        "",
        "Keep 1:1 because upstream command radians are flyer radians. Either prove the production OD65 outgoing line stays inside the reported 27.746868 mm motor limit with all inertia/friction terms included, or select a stronger closed-loop motor and matched higher-capacity 1:1 pulleys/belt. The existing 23HS22-4004D-E1000/30T-3GT package remains a candidate until its 300 RPM running curve and larger installed envelope pass.",
        "",
        "This report does not modify the arm source, assembly, controller, settings, BOM, or procurement schedule.",
        "",
    ])
    if report["controlling_blockers"]:
        lines.extend(["## Controlling blockers", ""])
        lines.extend(f"- `{name}`" for name in report["controlling_blockers"])
        lines.append("")
    lines.extend([
        f"Report SHA-256: `{report['report_sha256']}`", "",
    ])
    return "\n".join(lines)


def write_reports(report: Mapping[str, Any] | None = None) -> dict[str, Any]:
    result = analyze() if report is None else dict(report)
    REPORTS.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n",
                        encoding="utf-8")
    MD_OUT.write_text(render_markdown(result), encoding="utf-8")
    return result


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unexpected balance-retention schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("balance-retention proof hash mismatch")
    if report.get("status") != "DESIGN_NO_GO":
        raise ValueError("reconciled balance-retention report must fail closed")
    if report.get("focused_counterweight_CAD_authorized") is not False:
        raise ValueError("rejected stack cannot authorize focused CAD")
    if not report.get("controlling_blockers"):
        raise ValueError("no-go report has no controlling blockers")


def main() -> int:
    report = write_reports()
    validate_report_integrity(report)
    duty = report["tolerance_and_M2_duty"]
    print(
        f"offset balance retention: {report['status']}; "
        f"mass={report['exact_rotating_mass_properties']['mass_g']:.3f} g; "
        f"M2={duty['selected_motor_margin']:.3f}x; "
        f"pulley={duty['selected_pulley_margin']:.3f}x"
    )
    return 0 if report["status"] == "PASS_REVIEW_ONLY" else 1


if __name__ == "__main__":
    raise SystemExit(main())
