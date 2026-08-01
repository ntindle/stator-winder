"""Isolated positive-volume successor-follower prototype.

Four review-rack modules implement the minimum topology selected by
``aggregate_boundary_follower_placement_trade.json``.  Nothing imports this
module into the machine assembly, player, BOM, or release path.
"""

from __future__ import annotations

from copy import deepcopy
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

from build123d import (
    Align, Box, BuildLine, BuildSketch, Circle, Compound, Cylinder, Line,
    Locations, Plane, Pos, Rectangle, Rot, Sphere, ThreePointArc, export_step,
    sweep,
)


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
PLACEMENT_REPORT = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_placement_trade.json"
)
STEP_OUT = ROOT / "out" / "review" / (
    "aggregate_boundary_follower_successor_prototype.step"
)
MANIFEST_OUT = ROOT / "out" / "review" / (
    "aggregate_boundary_follower_successor_prototype_manifest.json"
)

CTR = (Align.CENTER, Align.CENTER, Align.CENTER)
EXPECTED_REPORT_INTERNAL_SHA256 = (
    "1800b5f9500f5b0041758991cc8f42f8dc0b62654bec3ce84e402b59dd79dbc3"
)
EXPECTED_TOPOLOGY = (
    "four M0-owned re-datumed XYZ flexure/slide stages, each carrying a "
    "yaw/elevation-compliant polished C1 guide cartridge plus a separate "
    "aggregate-normal preload leaf"
)

MODELED_XYZ_TRAVEL_MM = (1.50, 2.40, 1.10)
MODELED_YAW_HALF_RANGE_DEG = 55.0
MODELED_ELEVATION_HALF_RANGE_DEG = 10.0
GUIDE_CENTERLINE_BEND_RADIUS_MM = 3.0
CONSERVATIVE_ENVELOPE_RADIUS_MM = 3.0
FLOOR_TARGET_CLEARANCE_MM = 2.0
FLOOR_RELIEF_RADIUS_MM = 5.0
TARGET_IN_MODULE_MM = (0.0, 0.0, 2.0)
DISPLAY_OFFSETS_MM = {
    0: (0.0, 0.0, 0.0),
    1: (38.0, 0.0, 0.0),
    2: (38.0, 30.0, 0.0),
    3: (0.0, 30.0, 0.0),
}

AUTHORITY = {
    "isolated_review_only": True,
    "assembly_integration_authorized": False,
    "wire_route_authorized": False,
    "collision_authorized": False,
    "load_authorized": False,
    "dynamics_authorized": False,
    "buildability_authorized": False,
    "procurement_authorized": False,
    "BOM_change_authorized": False,
    "production_authorized": False,
    "release_authorized": False,
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(value: Mapping[str, Any]) -> str:
    body = deepcopy(dict(value))
    body.pop("report_sha256", None)
    return hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


@lru_cache(maxsize=1)
def placement_report() -> dict[str, Any]:
    report = json.loads(PLACEMENT_REPORT.read_text(encoding="utf-8"))
    if report.get("report_sha256") != EXPECTED_REPORT_INTERNAL_SHA256:
        raise ValueError("placement-trade internal hash drift")
    if _canonical_hash(report) != EXPECTED_REPORT_INTERNAL_SHA256:
        raise ValueError("placement-trade canonical hash invalid")
    trade = report.get("successor_trade", {})
    if trade.get("selected_topology") != EXPECTED_TOPOLOGY:
        raise ValueError("placement-trade selected topology drift")
    if set(trade.get("per_identity", {})) != {"0", "1", "2", "3"}:
        raise ValueError("placement-trade identity set drift")
    forbidden = (
        "wire_route_authorized", "collision_authorized",
        "assembly_integration_authorized", "load_authorized",
        "dynamics_authorized", "BOM_change_authorized",
        "procurement_authorized", "production_authorized",
        "release_authorized",
    )
    if any(bool(report.get(key)) for key in forbidden):
        raise ValueError("placement trade unexpectedly grants authority")
    return report


def identity_contract(identity: int) -> dict[str, Any]:
    if int(identity) not in DISPLAY_OFFSETS_MM:
        raise ValueError("identity must be 0..3")
    return placement_report()["successor_trade"]["per_identity"][str(identity)]


def _label(part, label: str):
    part.label = label
    return part


def _target(identity: int) -> tuple[float, float, float]:
    ox, oy, oz = DISPLAY_OFFSETS_MM[int(identity)]
    tx, ty, tz = TARGET_IN_MODULE_MM
    return ox + tx, oy + ty, oz + tz


def display_point_from_active_local(
    identity: int, point: list[float] | tuple[float, float, float],
) -> tuple[float, float, float]:
    datum = identity_contract(identity)["exact_target_datum_local_mm"]
    target = _target(identity)
    return tuple(
        float(target[i]) + float(point[i]) - float(datum[i])
        for i in range(3)
    )


@lru_cache(maxsize=1)
def c1_guide_local():
    """Open guide with line--R3 quarter arc--line tangent continuity."""

    start = (-1.5, -3.0, 0.0)
    arc_start = (0.0, -3.0, 0.0)
    arc_mid = (3.0 / math.sqrt(2.0), -3.0 / math.sqrt(2.0), 0.0)
    arc_end = (3.0, 0.0, 0.0)
    end = (3.0, 1.5, 0.0)
    with BuildLine() as path:
        Line(start, arc_start)
        ThreePointArc(arc_start, arc_mid, arc_end)
        Line(arc_end, end)
    profile_plane = Plane(origin=start, x_dir=(0.0, 0.0, 1.0),
                          z_dir=(1.0, 0.0, 0.0))
    with BuildSketch(profile_plane) as outer_profile:
        Circle(0.85)
    with BuildSketch(profile_plane) as negative_profile:
        Circle(0.36)
        with Locations((0.72, 0.0)):
            Rectangle(1.44, 0.68)
    result = sweep(outer_profile.sketch, path.wire()).cut(
        sweep(negative_profile.sketch, path.wire())
    ).clean()
    result.label = "polished_PEEK_C1_open_guide_cartridge"
    return result


def guide_cartridge(identity: int):
    orient = identity_contract(identity)["polished_guide_tangent_orientation"]
    yaw = float(orient["yaw"]["datum_deg"])
    elevation = float(orient["elevation"]["datum_deg"])
    result = Pos(*_target(identity)) * (
        Rot(0.0, -elevation, yaw) * c1_guide_local()
    )
    return _label(result, f"id{identity}_polished_PEEK_C1_guide_cartridge")


def floor_relief_coupon(identity: int):
    tx, ty, tz = _target(identity)
    stock = Pos(tx, ty, tz - 4.0) * Box(18.0, 16.0, 2.0, align=CTR)
    result = stock.cut(Pos(tx, ty, tz) * Sphere(FLOOR_RELIEF_RADIUS_MM)).clean()
    return _label(result, f"id{identity}_carrier_floor_R5_relief_coupon")


def xyz_stage_parts(identity: int) -> list[Any]:
    tx, ty, tz = _target(identity)
    x0, y0 = tx - 12.0, ty
    parts = [
        _label(Pos(x0, y0, tz - 7.0) * Box(12.0, 12.0, 1.4, align=CTR),
               f"id{identity}_fixed_stage_base"),
        _label(Pos(x0, y0 - 4.0, tz - 5.8) * Box(13.5, 0.7, 0.7, align=CTR),
               f"id{identity}_X_rail_A_1p50_travel"),
        _label(Pos(x0, y0 + 4.0, tz - 5.8) * Box(13.5, 0.7, 0.7, align=CTR),
               f"id{identity}_X_rail_B_1p50_travel"),
        _label(Pos(x0, y0, tz - 4.8) * Box(3.0, 10.0, 0.8, align=CTR),
               f"id{identity}_X_moving_bridge"),
        _label(Pos(x0 - 1.0, y0, tz - 3.8) * Box(0.7, 12.0, 0.7, align=CTR),
               f"id{identity}_Y_rail_A_2p40_travel"),
        _label(Pos(x0 + 1.0, y0, tz - 3.8) * Box(0.7, 12.0, 0.7, align=CTR),
               f"id{identity}_Y_rail_B_2p40_travel"),
        _label(Pos(x0, y0, tz - 2.8) * Box(5.0, 3.0, 0.8, align=CTR),
               f"id{identity}_Y_moving_bridge"),
        _label(Pos(x0 - 1.5, y0, tz - 0.5) * Box(0.7, 0.7, 4.0, align=CTR),
               f"id{identity}_Z_rail_A_1p10_travel"),
        _label(Pos(x0 + 1.5, y0, tz - 0.5) * Box(0.7, 0.7, 4.0, align=CTR),
               f"id{identity}_Z_rail_B_1p10_travel"),
    ]
    z_bridge = Pos(x0, y0, tz - 0.5) * Box(5.0, 2.0, 1.1, align=CTR)
    for x in (x0 - 1.5, x0 + 1.5):
        z_bridge = z_bridge.cut(Pos(x, y0, tz - 0.5) * Box(
            0.95, 0.95, 1.8, align=CTR))
    parts.append(_label(z_bridge.clean(), f"id{identity}_Z_moving_bridge"))
    parts.append(_label(
        Pos(tx - 6.25, ty, tz + 0.25) * Box(8.5, 1.0, 0.6, align=CTR),
        f"id{identity}_moving_cantilever_to_gimbal",
    ))
    return parts


def gimbal_parts(identity: int) -> list[Any]:
    tx, ty, tz = _target(identity)
    ring = Cylinder(1.65, 0.55, align=CTR).cut(
        Cylinder(1.28, 0.8, align=CTR))
    rotor = Cylinder(1.10, 0.45, align=CTR)
    fork = (
        Pos(0.0, -1.10, 0.85) * Box(2.4, 0.45, 1.7, align=CTR)
    ).fuse(
        Pos(0.0, 1.10, 0.85) * Box(2.4, 0.45, 1.7, align=CTR),
        Pos(0.0, 0.0, 0.18) * Box(2.4, 2.65, 0.40, align=CTR),
    ).clean()
    return [
        _label(Pos(tx, ty, tz) * ring, f"id{identity}_yaw_stator_plus_minus_55deg"),
        _label(Pos(tx, ty, tz) * rotor, f"id{identity}_yaw_rotor"),
        _label(Pos(tx, ty, tz + 0.35) * fork,
               f"id{identity}_elevation_fork_plus_minus_10deg"),
        _label(Pos(tx, ty, tz + 1.20) * (
            Rot(90.0, 0.0, 0.0) * Cylinder(0.30, 3.0, align=CTR)),
            f"id{identity}_elevation_pivot_pin"),
    ]


def preload_parts(identity: int) -> list[Any]:
    tx, ty, tz = _target(identity)
    datum = identity_contract(identity)["exact_target_datum_local_mm"]
    length = math.hypot(float(datum[0]), float(datum[1]))
    nx, ny = float(datum[0]) / length, float(datum[1]) / length
    angle = math.degrees(math.atan2(ny, nx))
    leaf = Pos(tx - 3.25 * nx, ty - 3.25 * ny, tz + 3.0) * (
        Rot(0.0, 0.0, angle) * Box(5.0, 0.45, 0.25, align=CTR)
    )
    shoe = Pos(tx - 0.55 * nx, ty - 0.55 * ny, tz + 2.72) * (
        Rot(0.0, 0.0, angle) * Box(0.90, 0.85, 0.40, align=CTR)
    )
    return [
        _label(leaf, f"id{identity}_separate_aggregate_normal_preload_leaf"),
        _label(shoe, f"id{identity}_separate_polished_PEEK_preload_shoe"),
    ]


def bound_witnesses(identity: int) -> list[Any]:
    bounds = identity_contract(identity)["exact_target_center_bounds_local_mm"]
    result = []
    for name in ("min_mm", "max_mm"):
        result.append(_label(
            Pos(*display_point_from_active_local(identity, bounds[name]))
            * Sphere(0.12),
            f"id{identity}_exact_center_{name}_datum_witness",
        ))
    return result


def module_parts(identity: int) -> list[Any]:
    result = [floor_relief_coupon(identity)]
    result.extend(xyz_stage_parts(identity))
    result.extend(gimbal_parts(identity))
    result.append(guide_cartridge(identity))
    result.extend(preload_parts(identity))
    result.extend(bound_witnesses(identity))
    return result


def geometry_contract() -> dict[str, Any]:
    report = placement_report()
    trade = report["successor_trade"]
    required = [float(v) for v in
                trade["common_exact_minimum_center_strokes_XYZ_mm"]]
    identities = {}
    for identity in range(4):
        row = identity_contract(identity)
        identities[str(identity)] = {
            "name": row["name"],
            "exact_target_center_bounds_local_mm": deepcopy(
                row["exact_target_center_bounds_local_mm"]),
            "exact_target_datum_local_mm": list(
                row["exact_target_datum_local_mm"]),
            "polished_guide_tangent_orientation": deepcopy(
                row["polished_guide_tangent_orientation"]),
            "display_offset_mm": list(DISPLAY_OFFSETS_MM[identity]),
            "display_target_mm": list(_target(identity)),
            "aggregate_normal_datum_witness": "normalized(datum_X,datum_Y,0)",
        }
    return {
        "schema": "aggregate_boundary_follower_successor_prototype_v1",
        "status": "PASS_ISOLATED_POSITIVE_VOLUME_TOPOLOGY_PROTOTYPE",
        "placement_trade": {
            "path": str(PLACEMENT_REPORT.relative_to(ROOT)).replace("\\", "/"),
            "file_sha256": _sha256(PLACEMENT_REPORT),
            "internal_report_sha256": report["report_sha256"],
            "selected_topology": trade["selected_topology"],
        },
        "frame": {
            "active_local_axes": "+X radial outward; +Y tangential; +Z stator axis",
            "M0_home_transform": "machine=(-local_y,local_z,95-local_x)",
            "owner": "M0_carriage",
            "STEP_pose": "2x2_isolated_review_rack_not_assembly_placement",
        },
        "stage": {
            "count": 4,
            "required_common_XYZ_travel_mm": required,
            "modeled_XYZ_travel_mm": list(MODELED_XYZ_TRAVEL_MM),
            "all_modeled_travel_meets_required": all(
                modeled >= need for modeled, need
                in zip(MODELED_XYZ_TRAVEL_MM, required)
            ),
            "required_yaw_half_range_deg": 53.23669873274605,
            "modeled_yaw_half_range_deg": MODELED_YAW_HALF_RANGE_DEG,
            "required_elevation_half_range_deg": 9.086049191773341,
            "modeled_elevation_half_range_deg": MODELED_ELEVATION_HALF_RANGE_DEG,
        },
        "guide": {
            "count": 4,
            "material": "virgin unfilled PEEK; polished contact channel",
            "centerline": "line--R3_quarter_arc--line",
            "centerline_bend_radius_mm": GUIDE_CENTERLINE_BEND_RADIUS_MM,
            "join_continuity": "C1_tangent_continuous_by_construction",
            "loading_opening_complete": True,
            "all_4704_case_surface_proved": False,
        },
        "preload": {
            "leaf_count": 4, "shoe_count": 4,
            "mechanically_separate_from_guide": True,
            "force_or_fatigue_proved": False,
        },
        "carrier_floor_relief": {
            "coupon_count": 4,
            "conservative_envelope_radius_mm": CONSERVATIVE_ENVELOPE_RADIUS_MM,
            "target_clearance_mm": FLOOR_TARGET_CLEARANCE_MM,
            "relief_radius_mm": FLOOR_RELIEF_RADIUS_MM,
            "radial_clearance_mm": (
                FLOOR_RELIEF_RADIUS_MM - CONSERVATIVE_ENVELOPE_RADIUS_MM
            ),
            "selected_carrier_modified": False,
        },
        "identities": identities,
        "blockers": [
            "UNPROVED_positive_volume_surface_over_all_4704_cases",
            "UNRUN_full_XYZ_yaw_elevation_collision_sweep",
            "UNSIZED_flexures_and_preload_leaf",
            "UNQUALIFIED_finish_wear_retention_tolerances_fasteners",
            "UNPROVED_wire_route_dynamics_buildability",
            "UNINTEGRATED_selected_carrier_and_machine_assembly",
        ],
        "authority": dict(AUTHORITY),
    }


def manifest(step_path: Path | str = STEP_OUT) -> dict[str, Any]:
    result = geometry_contract()
    step = Path(step_path)
    result["artifacts"] = {
        "source": str(Path(__file__).resolve()),
        "source_sha256": _sha256(Path(__file__)),
        "step": str(step.resolve()),
        "step_exists": step.exists(),
        "step_size_bytes": step.stat().st_size if step.exists() else None,
        "step_sha256": _sha256(step) if step.exists() else None,
    }
    return result


def write_manifest(path: Path | str = MANIFEST_OUT,
                   step_path: Path | str = STEP_OUT) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(manifest(step_path), indent=2) + "\n",
                      encoding="utf-8")
    return target


def gen_step() -> Compound:
    modules = []
    for identity in range(4):
        module = Compound(children=module_parts(identity))
        module.label = f"identity_{identity}_{identity_contract(identity)['name']}"
        modules.append(module)
    result = Compound(children=modules)
    result.label = "aggregate_boundary_follower_successor_REVIEW_ONLY"
    solids = list(result.solids())
    if not solids or any(float(solid.volume) <= 0.0 for solid in solids):
        raise RuntimeError("all prototype leaves must have positive volume")
    return result


if __name__ == "__main__":
    STEP_OUT.parent.mkdir(parents=True, exist_ok=True)
    export_step(gen_step(), str(STEP_OUT))
    write_manifest()
    print(STEP_OUT)
    print(MANIFEST_OUT)

