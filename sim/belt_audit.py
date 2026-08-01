"""Fail-closed full-revolution audit of the selected M2 belt lane.

This audit consumes the source-level selected integrated candidate, not the
legacy ``out/links`` P40/200-2GT export.  It checks the exact selected
NBK P30 30T:30T pulley occurrences and the 210-3GT-6 belt envelope.  The belt
is fixed in the machine frame while every non-pulley flyer occurrence is
sampled through a complete M2 revolution at one-degree increments.

Only two positive-volume contacts are permitted: belt-to-motor-P30 and
belt-to-flyer-P30 tooth engagement.  The flyer pulley contact must persist at
all sampled angles; every other rotating flyer component and every selected
local static component must remain collision-free with at least the declared
clearance target.

The belt remains a supplier-dimensioned, toothless physical envelope.  This
report proves rigid packaging only.  Installed pretension, tooth stress,
tracking, resonance, fatigue, wear, and hub slip remain physical gates.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable, Mapping

import fcl
import numpy as np
import trimesh


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
REPORT = ROOT / "out" / "reports" / "belt_audit.json"
CANDIDATE_REPORT = ROOT / "out" / "reports" / "integrated_release_candidate.json"

if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

import integrated_release_candidate as candidate  # noqa: E402
import m2_drive_successor_review as drive  # noqa: E402


SCHEMA = "selected-m2-belt-audit/v2"
CLEARANCE_TARGET_MM = 2.2
ANGLE_STEP_DEG = 1.0
TESSELLATION_LINEAR_MM = 0.08
TESSELLATION_ANGULAR_RAD = 0.08
BOOLEAN_TOLERANCE_MM3 = 1.0e-5

MOTOR_ENGAGEMENT_PAIR = "belt_to_motor_P30_D5_tooth_band"
FLYER_ENGAGEMENT_PAIR = "belt_to_flyer_P30_D10_tooth_band"
INTENDED_CONTACT_EXEMPTIONS = (
    MOTOR_ENGAGEMENT_PAIR,
    FLYER_ENGAGEMENT_PAIR,
)

# These are the direct geometry inputs used by this audit.  The integrated
# candidate report is included as the selected-identity and geometry-check
# contract, while the source files remain the actual shape authority.  This
# intentionally excludes procurement-only inputs such as BOM prices.
SOURCE_PATHS = (
    "sim/belt_audit.py",
    "cad/integrated_release_candidate.py",
    "cad/assembly.py",
    "cad/params.py",
    "cad/printed.py",
    "cad/cots.py",
    "cad/hardware.py",
    "cad/hardware_placements.py",
    "cad/wire_geometry.py",
    "cad/m2_drive_successor_review.py",
    "cad/permanent_cap_offset_spoke_retained_review.py",
    "cad/retained_flyer_peek_guide_successor.py",
    "cad/flyer_shaft_d10.py",
    "cad/nbk_p30_official_occurrence.py",
    "cad/models/upgrades/NBK_P30-3GT-BLP-6C-5_AP214.step",
    "cad/nbk_p30_d10_official_occurrence.py",
    "cad/models/upgrades/NBK_P30_D10_download/P30-3GT-BLP-6C-10.stp",
    "cad/leadshine_cs_m21708_cableless.py",
    "cad/models/upgrades/CS-M21708.STEP",
    "cad/models/upgrades/CS-M21708_cableless.step",
    "out/reports/integrated_release_candidate.json",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_hash(value: Mapping[str, Any]) -> str:
    payload = dict(value)
    payload.pop("report_sha256", None)
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _source_hashes() -> dict[str, str]:
    missing = [relative for relative in SOURCE_PATHS if not (ROOT / relative).is_file()]
    if missing:
        raise FileNotFoundError("missing belt-audit source inputs: " + ", ".join(missing))
    return {relative: _sha256(ROOT / relative) for relative in SOURCE_PATHS}


def _candidate_contract() -> tuple[dict[str, Any], list[str]]:
    blockers: list[str] = []
    try:
        report = json.loads(CANDIDATE_REPORT.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        return {}, [f"cannot read selected integrated-candidate report: {exc}"]

    if report.get("schema") != "integrated-release-candidate/v1":
        blockers.append("selected integrated-candidate schema mismatch")
    if report.get("status") != "REFERENCE_GEOMETRY_PASS_RELEASE_GATES_OPEN":
        blockers.append("selected integrated-candidate geometry status is not PASS")
    if report.get("report_sha256") != _canonical_hash(report):
        blockers.append("selected integrated-candidate report self-hash mismatch")

    geometry_checks = report.get("geometry", {}).get("checks", {})
    required_geometry_checks = (
        "official_NBK_P30_D5_motor_vendor_STEP_hash_pinned_and_unmodified",
        "official_NBK_P30_D10_flyer_vendor_STEP_hash_pinned_and_unmodified",
        "exact_1_to_1_ratio_retained",
        "coupled_exact_live_line_P30_210_3GT_capacity_ge_2x",
    )
    for name in required_geometry_checks:
        if geometry_checks.get(name) is not True:
            blockers.append(f"selected integrated-candidate check is not PASS: {name}")

    assumptions = report.get("P30_NBK_interface_assumptions", {})
    if assumptions.get("ratio") != "30T motor / 30T flyer = exact 1:1":
        blockers.append("selected drive ratio is not the exact P30 30T:30T contract")
    if "210-3GT-6" not in str(assumptions.get("belt", "")):
        blockers.append("selected drive report does not identify belt 210-3GT-6")
    return report, blockers


def _part_mesh(part: Any) -> trimesh.Trimesh:
    meshes: list[trimesh.Trimesh] = []
    solids = list(part.solids()) or [part]
    for solid_index, solid in enumerate(solids):
        try:
            vertices, faces = solid.tessellate(
                TESSELLATION_LINEAR_MM,
                TESSELLATION_ANGULAR_RAD,
            )
        except Exception as exc:
            raise RuntimeError(
                "tessellation failed for "
                f"{getattr(part, 'label', 'unlabelled part')} "
                f"solid {solid_index}: {type(exc).__name__}: {exc}"
            ) from exc
        mesh = trimesh.Trimesh(
            vertices=np.asarray([(v.X, v.Y, v.Z) for v in vertices], dtype=float),
            faces=np.asarray(faces, dtype=np.int64),
            process=False,
        )
        if mesh.is_empty or len(mesh.faces) == 0:
            raise RuntimeError(
                f"empty tessellation for {getattr(part, 'label', 'unlabelled part')}"
            )
        meshes.append(mesh)
    combined = trimesh.util.concatenate(meshes)
    combined.remove_unreferenced_vertices()
    return combined


def _bvh(mesh: trimesh.Trimesh) -> fcl.BVHModel:
    model = fcl.BVHModel()
    model.beginModel(len(mesh.vertices), len(mesh.faces))
    model.addSubModel(
        np.asarray(mesh.vertices, dtype=np.float64),
        np.asarray(mesh.faces, dtype=np.int64),
    )
    model.endModel()
    return model


def _rot_z(angle_rad: float) -> np.ndarray:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.asarray(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))


def _angles(step_deg: float) -> list[float]:
    if not math.isfinite(step_deg) or step_deg <= 0.0:
        raise ValueError("angle step must be finite and positive")
    count = round(360.0 / step_deg)
    if count < 1 or not math.isclose(count * step_deg, 360.0, abs_tol=1.0e-9):
        raise ValueError("angle step must divide a complete 360 degree revolution")
    return [index * step_deg for index in range(count)]


def _query(
    part_bvh: fcl.BVHModel,
    belt_object: fcl.CollisionObject,
    transform: fcl.Transform,
) -> tuple[bool, float]:
    part_object = fcl.CollisionObject(part_bvh, transform)
    collision_result = fcl.CollisionResult()
    fcl.collide(
        part_object,
        belt_object,
        fcl.CollisionRequest(num_max_contacts=1),
        collision_result,
    )
    collided = bool(collision_result.is_collision)
    if collided:
        return True, 0.0
    distance = fcl.distance(
        part_object,
        belt_object,
        fcl.DistanceRequest(),
        fcl.DistanceResult(),
    )
    return False, float(distance)


def _sweep_clearance(
    key: str,
    part: Any,
    belt_object: fcl.CollisionObject,
    angles_deg: Iterable[float],
) -> dict[str, Any]:
    mesh = _part_mesh(part)
    part_bvh = _bvh(mesh)
    minimum = math.inf
    minimum_angle: float | None = None
    collision_angles: list[float] = []
    sample_count = 0
    for angle_deg in angles_deg:
        sample_count += 1
        collided, distance = _query(
            part_bvh,
            belt_object,
            fcl.Transform(_rot_z(math.radians(angle_deg)), np.zeros(3)),
        )
        if collided:
            collision_angles.append(angle_deg)
        elif distance < minimum:
            minimum = distance
            minimum_angle = angle_deg
    minimum_value = None if not math.isfinite(minimum) else float(minimum)
    ok = (
        not collision_angles
        and minimum_value is not None
        and minimum_value >= CLEARANCE_TARGET_MM - 1.0e-6
    )
    return {
        "part_key": key,
        "label": str(getattr(part, "label", key)),
        "ok": ok,
        "sample_count": sample_count,
        "collision_count": len(collision_angles),
        "collision_angles_deg": collision_angles,
        "minimum_clearance_mm": minimum_value,
        "minimum_angle_deg": minimum_angle,
        "mesh": {
            "solid_count": len(list(part.solids())),
            "vertex_count": int(len(mesh.vertices)),
            "triangle_count": int(len(mesh.faces)),
            "watertight": bool(mesh.is_watertight),
        },
    }


def _static_clearance(
    group: str,
    key: str,
    part: Any,
    belt: Any,
) -> dict[str, Any]:
    # Static hardware needs no sampled rigid transform, so preserve the exact
    # OCC BREP instead of weakening imported multi-solid STEP occurrences to a
    # triangle proxy.  This is also robust to supplier STEP faces which OCC can
    # measure but which do not carry a cached triangulation.
    distance = float(part.distance_to(belt))
    common = part & belt
    overlap = 0.0 if common is None else float(common.volume)
    collided = overlap > BOOLEAN_TOLERANCE_MM3
    return {
        "group": group,
        "part_key": key,
        "label": str(getattr(part, "label", key)),
        "ok": (
            not collided
            and distance >= CLEARANCE_TARGET_MM - 1.0e-6
        ),
        "positive_overlap": collided,
        "overlap_mm3": overlap,
        "clearance_mm": distance,
        "BREP": {
            "solid_count": len(list(part.solids())),
            "valid": bool(part.is_valid),
            "method": "exact_OCC_distance_to_and_common_volume",
        },
    }


def _intersection_facts(
    belt: Any,
    pulley: Any,
    *,
    axis_xy_mm: tuple[float, float],
) -> dict[str, Any]:
    common = belt & pulley
    volume = 0.0 if common is None else float(common.volume)
    facts: dict[str, Any] = {"exact_overlap_mm3": volume}
    if common is None or volume <= BOOLEAN_TOLERANCE_MM3:
        facts.update({
            "minimum_radius_from_pulley_axis_mm": None,
            "maximum_radius_from_pulley_axis_mm": None,
            "bbox_mm": None,
            "tooth_band_only": False,
        })
        return facts

    mesh = _part_mesh(common)
    x = np.asarray(mesh.vertices[:, 0]) - axis_xy_mm[0]
    y = np.asarray(mesh.vertices[:, 1]) - axis_xy_mm[1]
    radii = np.hypot(x, y)
    radial_min = float(radii.min())
    radial_max = float(radii.max())
    expected_min = (
        drive.PITCH_DIAMETER_MM / 2.0
        - drive.BELT_INWARD_FROM_PITCH_MM
        - TESSELLATION_LINEAR_MM
    )
    expected_max = (
        drive.PITCH_DIAMETER_MM / 2.0
        + drive.BELT_OUTWARD_FROM_PITCH_MM
        + TESSELLATION_LINEAR_MM
    )
    facts.update({
        "minimum_radius_from_pulley_axis_mm": radial_min,
        "maximum_radius_from_pulley_axis_mm": radial_max,
        "expected_tooth_engagement_radial_band_mm": [expected_min, expected_max],
        "bbox_mm": [mesh.bounds[0].tolist(), mesh.bounds[1].tolist()],
        "tooth_band_only": radial_min >= expected_min and radial_max <= expected_max,
    })
    return facts


def _flyer_engagement(
    pulley: Any,
    belt: Any,
    belt_object: fcl.CollisionObject,
    angles_deg: list[float],
) -> dict[str, Any]:
    mesh = _part_mesh(pulley)
    pulley_bvh = _bvh(mesh)
    contact_angles: list[float] = []
    for angle_deg in angles_deg:
        collided, _ = _query(
            pulley_bvh,
            belt_object,
            fcl.Transform(_rot_z(math.radians(angle_deg)), np.zeros(3)),
        )
        if collided:
            contact_angles.append(angle_deg)
    facts = _intersection_facts(belt, pulley, axis_xy_mm=(0.0, 0.0))
    contact_angle_set = set(contact_angles)
    facts.update({
        "pair": FLYER_ENGAGEMENT_PAIR,
        "exemption": "positive contact required only in the P30 tooth band",
        "sample_count": len(angles_deg),
        "contact_count": len(contact_angles),
        "contact_at_every_sample": len(contact_angles) == len(angles_deg),
        "missing_contact_angles_deg": [
            angle for angle in angles_deg if angle not in contact_angle_set
        ],
    })
    return facts


def _relevant_static_parts(
    drive_parts: Mapping[str, Any],
    static_groups: Mapping[str, list[Any]],
    static_wire: Any,
) -> list[tuple[str, str, Any]]:
    selected: list[tuple[str, str, Any]] = []
    excluded_drive_keys = {"belt", "motor_pulley", "flyer_pulley"}
    for key, part in drive_parts.items():
        if key not in excluded_drive_keys:
            selected.append(("successor_drive", key, part))
    for group in ("shifted_support", "shifted_entry"):
        for index, part in enumerate(static_groups[group]):
            key = str(getattr(part, "label", f"{group}_{index}"))
            selected.append((group, key, part))
    selected.append(("configured_wire", "configured_static_supply_wire", static_wire))
    return selected


def audit(angle_step_deg: float = ANGLE_STEP_DEG) -> dict[str, Any]:
    angles_deg = _angles(angle_step_deg)
    candidate_report, contract_blockers = _candidate_contract()
    source_hashes = _source_hashes()

    drive_parts = candidate.successor_drive_parts()
    rotating = candidate.retained_rotating_parts()
    static_groups = candidate.main_static_groups()
    static_wire = candidate.configured_static_supply_wire()

    belt = drive_parts["belt"]
    motor_pulley = drive_parts["motor_pulley"]
    flyer_pulley = rotating["flyer_pulley"]
    belt_mesh = _part_mesh(belt)
    belt_object = fcl.CollisionObject(
        _bvh(belt_mesh), fcl.Transform(np.eye(3), np.zeros(3)),
    )

    rotating_rows = [
        _sweep_clearance(key, part, belt_object, angles_deg)
        for key, part in rotating.items()
        if key != "flyer_pulley"
    ]
    static_rows = [
        _static_clearance(group, key, part, belt)
        for group, key, part in _relevant_static_parts(
            drive_parts, static_groups, static_wire,
        )
    ]

    motor_engagement = _intersection_facts(
        belt, motor_pulley, axis_xy_mm=(0.0, drive.MOTOR_AXIS_Y),
    )
    motor_engagement.update({
        "pair": MOTOR_ENGAGEMENT_PAIR,
        "exemption": "positive contact required only in the P30 tooth band",
        "contact_required": True,
    })
    flyer_engagement = _flyer_engagement(
        flyer_pulley, belt, belt_object, angles_deg,
    )

    rotating_failures = [row for row in rotating_rows if not row["ok"]]
    static_failures = [row for row in static_rows if not row["ok"]]
    intended_contacts_ok = (
        motor_engagement["exact_overlap_mm3"] > BOOLEAN_TOLERANCE_MM3
        and motor_engagement["tooth_band_only"] is True
        and flyer_engagement["exact_overlap_mm3"] > BOOLEAN_TOLERANCE_MM3
        and flyer_engagement["tooth_band_only"] is True
        and flyer_engagement["contact_at_every_sample"] is True
    )
    complete_rotation = (
        len(angles_deg) * angle_step_deg == 360.0
        and angles_deg[0] == 0.0
        and math.isclose(angles_deg[-1] + angle_step_deg, 360.0)
    )
    lane_identity_ok = (
        drive.MOTOR_TEETH == 30
        and drive.FLYER_TEETH == 30
        and drive.BELT_MODEL == "210-3GT-6"
        and math.isclose(drive.CENTER_DISTANCE_MM, 60.0)
        and str(getattr(belt, "label", "")) == "m2_successor_210_3gt_6_belt"
        and str(getattr(motor_pulley, "label", ""))
        == "NBK_P30_3GT_BLP_6C_5_stock_split_clamp_vendor_occurrence"
        and str(getattr(flyer_pulley, "label", ""))
        == "NBK_P30_3GT_BLP_6C_10_stock_hub_rear_vendor_occurrence"
    )
    checks = {
        "selected_integrated_candidate_contract_valid": not contract_blockers,
        "selected_lane_is_NBK_P30_30T_to_30T_and_210_3GT_6": lane_identity_ok,
        "complete_360_degree_flyer_revolution_sampled": complete_rotation,
        "only_two_intended_P30_tooth_engagement_exemptions": (
            set(INTENDED_CONTACT_EXEMPTIONS)
            == {MOTOR_ENGAGEMENT_PAIR, FLYER_ENGAGEMENT_PAIR}
        ),
        "every_non_pulley_rotating_flyer_component_checked": (
            len(rotating_rows) == len(rotating) - 1
            and {row["part_key"] for row in rotating_rows}
            == set(rotating) - {"flyer_pulley"}
        ),
        "all_non_pulley_rotating_flyer_components_clear": not rotating_failures,
        "all_relevant_static_motor_mount_BNW_wire_entry_parts_clear": (
            bool(static_rows) and not static_failures
        ),
        "both_P30_tooth_engagement_contacts_established": intended_contacts_ok,
    }
    passed = all(checks.values())
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "PASS" if passed else "FAIL",
        "passed": passed,
        "geometry_authorized": passed,
        "production_authorized": False,
        "authority": {
            "selected_candidate": "cad/integrated_release_candidate.py",
            "candidate_report": "out/reports/integrated_release_candidate.json",
            "belt": "210-3GT-6 physical envelope",
            "motor_pulley": "NBK P30-3GT-BLP-6C-5 official vendor occurrence",
            "flyer_pulley": "NBK P30-3GT-BLP-6C-10 official vendor occurrence",
            "ratio": "30T:30T exact 1:1",
            "scope": "rigid belt-lane packaging only",
        },
        "artifact_contract": {
            "candidate_schema": candidate_report.get("schema"),
            "candidate_status": candidate_report.get("status"),
            "candidate_report_sha256": _sha256(CANDIDATE_REPORT),
            "candidate_report_self_hash_valid": (
                candidate_report.get("report_sha256") == _canonical_hash(candidate_report)
                if candidate_report else False
            ),
            "blockers": contract_blockers,
        },
        "sampling": {
            "axis": "machine +Z / M2 flyer axis",
            "start_deg_inclusive": 0.0,
            "stop_deg_exclusive": 360.0,
            "step_deg": angle_step_deg,
            "sample_count": len(angles_deg),
            "complete_revolution": complete_rotation,
        },
        "method": {
            "rotating_collision": "python-fcl BVH triangle collision/distance",
            "static_collision": "exact OCC BREP distance and common-volume",
            "tessellation_linear_mm": TESSELLATION_LINEAR_MM,
            "tessellation_angular_rad": TESSELLATION_ANGULAR_RAD,
            "clearance_target_mm": CLEARANCE_TARGET_MM,
            "intended_overlap": "exact OCC common-volume at M2=0 plus FCL contact coverage",
        },
        "lane": {
            "motor_teeth": drive.MOTOR_TEETH,
            "flyer_teeth": drive.FLYER_TEETH,
            "pitch_mm": drive.PITCH_MM,
            "belt_model": drive.BELT_MODEL,
            "belt_pitch_length_mm": drive.BELT_PITCH_LENGTH_MM,
            "belt_width_mm": drive.BELT_WIDTH_MM,
            "center_distance_mm": drive.CENTER_DISTANCE_MM,
            "motor_pulley_label": str(getattr(motor_pulley, "label", "")),
            "flyer_pulley_label": str(getattr(flyer_pulley, "label", "")),
            "belt_label": str(getattr(belt, "label", "")),
        },
        "exemption_policy": {
            "allowed_positive_contact_pairs": list(INTENDED_CONTACT_EXEMPTIONS),
            "all_other_belt_contacts_forbidden": True,
            "generic_collision_gate_modified": False,
        },
        "intended_engagements": [motor_engagement, flyer_engagement],
        "rotating_non_engagement_parts": rotating_rows,
        "static_non_engagement_parts": static_rows,
        "summary": {
            "rotating_part_count_total": len(rotating),
            "rotating_non_engagement_part_count": len(rotating_rows),
            "rotating_query_count": len(rotating_rows) * len(angles_deg),
            "static_part_count": len(static_rows),
            "rotating_failure_count": len(rotating_failures),
            "static_failure_count": len(static_failures),
            "minimum_rotating_clearance_mm": min(
                row["minimum_clearance_mm"] for row in rotating_rows
                if row["minimum_clearance_mm"] is not None
            ),
            "minimum_static_clearance_mm": min(
                row["clearance_mm"] for row in static_rows
            ),
        },
        "checks": checks,
        "unexpected": [
            *({"kind": "artifact_contract", "detail": item}
              for item in contract_blockers),
            *({"kind": "rotating_clearance", **row} for row in rotating_failures),
            *({"kind": "static_clearance", **row} for row in static_failures),
            *([] if intended_contacts_ok else [{
                "kind": "intended_engagement",
                "detail": "one or both P30 tooth-band contacts are absent or escape the declared band",
            }]),
        ],
        "source_hashes": source_hashes,
        "limits": [
            "The belt is a supplier-dimensioned toothless physical envelope; individual belt teeth are not modeled.",
            "Installed pretension, tracking, tooth stress, resonance, fatigue, wear, and manufacturing pitch error require hardware tests.",
            "Motor-side BNW hole and screw solids are conservative unreleased witnesses until the configured supplier drawing arrives.",
            "This geometry PASS does not authorize either pulley retention interface or production operation.",
        ],
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def failure_report(reason: str) -> dict[str, Any]:
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "FAIL",
        "passed": False,
        "geometry_authorized": False,
        "production_authorized": False,
        "checks": {"audit_completed_without_exception": False},
        "unexpected": [{"kind": "audit_exception", "detail": reason}],
        "source_hashes": {
            relative: _sha256(ROOT / relative)
            for relative in SOURCE_PATHS
            if (ROOT / relative).is_file()
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def write_report(report: Mapping[str, Any], path: Path = REPORT) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return path


def main() -> int:
    try:
        report = audit()
    except Exception as exc:  # fail closed and preserve diagnostic evidence
        report = failure_report(f"{type(exc).__name__}: {exc}")
    target = write_report(report)
    summary = report.get("summary", {})
    print(
        f"selected M2 belt audit: {report['status']}; "
        f"rotating queries={summary.get('rotating_query_count', 0)}; "
        f"unexpected={len(report.get('unexpected', []))}"
    )
    print(target)
    return 0 if report.get("passed") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
