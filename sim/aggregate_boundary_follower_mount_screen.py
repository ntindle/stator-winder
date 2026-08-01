"""Eccentric mount and custom-body mass screen for the R3 follower prototype."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping

from build123d import CenterOf


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

import aggregate_boundary_floating_follower as follower


CAD_SOURCE = CAD / "aggregate_boundary_floating_follower.py"
CAD_AUDIT = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_cad_audit.json"
)
OUTPUT_JSON = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_mount_screen.json"
)
OUTPUT_MD = ROOT / "out" / "reports" / (
    "aggregate_boundary_follower_mount_screen.md"
)
SCHEMA = "aggregate-boundary-follower-mount-screen/v1"

PROOF_LOAD_N = 40.0
DENSITY_G_PER_MM3 = {
    "6061-T6": 2.70e-3,
    "7075-T6": 2.81e-3,
    "PEEK": 1.30e-3,
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_hash(value: Mapping[str, Any]) -> str:
    body = deepcopy(dict(value))
    body.pop("report_sha256", None)
    return hashlib.sha256(json.dumps(
        body, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


def _part_row(label: str, part, material: str) -> dict[str, Any]:
    volume = float(part.volume)
    center = part.center(CenterOf.MASS)
    mass = volume * DENSITY_G_PER_MM3[material]
    return {
        "label": label,
        "material": material,
        "volume_mm3": volume,
        "density_g_per_mm3": DENSITY_G_PER_MM3[material],
        "mass_g": mass,
        "center_of_mass_local_mm": [float(center.X), float(center.Y), float(center.Z)],
    }


def analyze() -> dict[str, Any]:
    cad_audit = json.loads(CAD_AUDIT.read_text(encoding="utf-8"))
    parts = [
        _part_row("carrier", follower.carrier(), "6061-T6"),
        _part_row("radial_slide", follower.radial_slide("mid"), "7075-T6"),
        _part_row(
            "monolithic_tangential_slide_outer_yoke_cartridge",
            follower.tangential_slide_outer_gimbal_cartridge(
                "mid", "center"
            ),
            "7075-T6",
        ),
        _part_row(
            "inner_gimbal_yoke",
            follower.inner_gimbal_yoke("mid", "center"), "6061-T6",
        ),
        _part_row(
            "PEEK_R3_nose", follower.nose_insert("mid", "center"), "PEEK",
        ),
        _part_row(
            "radial_bellcrank",
            follower.radial_bellcrank("mid", "center"), "6061-T6",
        ),
    ]
    custom_mass = sum(row["mass_g"] for row in parts)
    weighted = [
        sum(row["mass_g"] * row["center_of_mass_local_mm"][axis]
            for row in parts) / custom_mass
        for axis in range(3)
    ]

    hole_points = follower._tower_m4_local_locations()
    mount_x = sum(point[0] for point in hole_points) / len(hole_points)
    mount_y = sum(point[1] for point in hole_points) / len(hole_points)
    mount_z = follower.TOWER_FRONT_FACE_MACHINE_Y_MM
    mount = [mount_x, mount_y, mount_z]
    gimbal = follower._gimbal_center("mid", "center")
    nose = [float(gimbal[0] + 8.0), float(gimbal[1]), float(gimbal[2])]
    lever = [nose[index] - mount[index] for index in range(3)]

    x_values = sorted({float(point[0]) for point in hole_points})
    y_values = sorted({float(point[1]) for point in hole_points})
    span_x = x_values[-1] - x_values[0]
    span_y = y_values[-1] - y_values[0]
    axial_offset = abs(lever[2])
    radial_moment = PROOF_LOAD_N * axial_offset
    tangential_moment = PROOF_LOAD_N * axial_offset
    radial_row_couple = radial_moment / span_x
    tangential_row_couple = tangential_moment / span_y

    key_points = [
        follower._machine_reference_to_active_local((
            machine_x,
            follower.TOWER_FRONT_FACE_MACHINE_Y_MM,
            follower.TOWER_KEY_MACHINE_Z_MM,
        ))
        for machine_x in follower.TOWER_KEY_MACHINE_X_MM
    ]
    key_span_x = max(p[0] for p in key_points) - min(p[0] for p in key_points)
    key_span_y = max(p[1] for p in key_points) - min(p[1] for p in key_points)

    definition_gates = {
        "positive_volume_CAD_audit_bound": (
            cad_audit.get("positive_volume_R3_prototype_geometry_proven") is True
        ),
        "custom_body_mass_and_COM_computed": custom_mass > 0.0,
        "40N_eccentric_moments_computed": (
            radial_moment > 0.0 and tangential_moment > 0.0
        ),
    }
    release_gates = {
        "equal_10N_per_M4_is_sufficient_mount_proof": False,
        "radial_5p52Nm_joint_separation_and_fastener_reaction_proven": False,
        "adapter_face_pressure_and_key_bearing_proven": False,
        "hardware_spring_and_linkage_mass_included": False,
        "M0_acceleration_and_shock_load_updated": False,
        "40N_multidirectional_physical_proof_completed": False,
    }
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "FAIL",
        "decision": "ECCENTRIC_40N_MOUNT_LOAD_INVALIDATES_EQUAL_SHARE_ONLY_SCREEN",
        "production_authorized": False,
        "assembly_integration_authorized": False,
        "proof_load_N": PROOF_LOAD_N,
        "custom_body_mass": {
            "hardware_springs_and_unattached_M0_gate_excluded": True,
            "total_g": custom_mass,
            "center_of_mass_local_mm": weighted,
            "parts": parts,
        },
        "mount_geometry": {
            "tower_interface_centroid_local_mm": mount,
            "nose_axis_center_local_mm": nose,
            "lever_vector_local_mm": lever,
            "lever_magnitude_mm": math.sqrt(sum(value * value for value in lever)),
            "M4_x_rows_mm": x_values,
            "M4_y_rows_mm": y_values,
            "M4_x_span_mm": span_x,
            "M4_y_span_mm": span_y,
            "key_points_local_mm": [list(map(float, p)) for p in key_points],
            "key_x_span_mm": key_span_x,
            "key_y_span_mm": key_span_y,
        },
        "load_cases": {
            "radial_X_40N": {
                "moment_about_Y_Nmm": radial_moment,
                "ideal_M4_row_couple_N": radial_row_couple,
                "ideal_differential_reaction_per_screw_N": radial_row_couple / 2.0,
                "direct_shear_per_screw_N_if_equal": PROOF_LOAD_N / 4.0,
                "warning": "preload, prying, joint separation, face bearing and key sharing not included",
            },
            "tangential_Y_40N": {
                "moment_about_X_Nmm": tangential_moment,
                "torsional_moment_about_Z_Nmm": abs(lever[0]) * PROOF_LOAD_N,
                "ideal_M4_row_couple_N": tangential_row_couple,
                "ideal_differential_reaction_per_screw_N": tangential_row_couple / 2.0,
                "direct_shear_per_screw_N_if_equal": PROOF_LOAD_N / 4.0,
                "warning": "preload, prying, joint separation, face bearing and key sharing not included",
            },
        },
        "definition_gates": definition_gates,
        "release_gates": release_gates,
        "blockers": [name for name, value in release_gates.items() if not value],
        "source_evidence": {
            "cad_source": "cad/aggregate_boundary_floating_follower.py",
            "cad_source_sha256": _sha256(CAD_SOURCE),
            "cad_audit": "out/reports/aggregate_boundary_follower_cad_audit.json",
            "cad_audit_sha256": _sha256(CAD_AUDIT),
        },
    }
    report["report_sha256"] = _canonical_hash(report)
    validate_report_integrity(report)
    return report


def validate_report_integrity(report: Mapping[str, Any]) -> None:
    if report.get("schema") != SCHEMA:
        raise ValueError("unsupported follower mount screen schema")
    if report.get("report_sha256") != _canonical_hash(report):
        raise ValueError("follower mount screen hash mismatch")
    if report.get("status") != "FAIL":
        raise ValueError("unqualified eccentric mount was promoted")
    if report.get("production_authorized") is not False:
        raise ValueError("mount screen invented production authority")
    source = report.get("source_evidence", {})
    if source.get("cad_source_sha256") != _sha256(CAD_SOURCE):
        raise ValueError("stale follower CAD source")
    if source.get("cad_audit_sha256") != _sha256(CAD_AUDIT):
        raise ValueError("stale follower CAD audit")


def render_markdown(report: Mapping[str, Any]) -> str:
    mass = report["custom_body_mass"]
    mount = report["mount_geometry"]
    radial = report["load_cases"]["radial_X_40N"]
    tangential = report["load_cases"]["tangential_Y_40N"]
    lines = [
        "# Aggregate-boundary follower mount screen", "",
        f"**{report['status']} — {report['decision']}**", "",
        f"- Custom-body mass before hardware/springs: {mass['total_g']:.3f} g.",
        f"- Mount-to-nose lever: {mount['lever_magnitude_mm']:.3f} mm.",
        f"- M4 row spans: {mount['M4_x_span_mm']:.1f} mm X and "
        f"{mount['M4_y_span_mm']:.1f} mm Y.",
        f"- 40 N radial case: {radial['moment_about_Y_Nmm'] / 1000:.3f} N m, "
        f"{radial['ideal_M4_row_couple_N']:.1f} N ideal row couple, "
        f"{radial['ideal_differential_reaction_per_screw_N']:.1f} N per screw.",
        f"- 40 N tangential case: {tangential['moment_about_X_Nmm'] / 1000:.3f} N m, "
        f"{tangential['ideal_M4_row_couple_N']:.1f} N ideal row couple, "
        f"{tangential['ideal_differential_reaction_per_screw_N']:.1f} N per screw.",
        "", "The earlier 10 N-per-screw equal direct-share figure is not a mount proof; "
        "it omits the dominant eccentric moment, preload, prying, joint separation, "
        "face pressure, and key bearing.", "", "## Open gates", "",
    ]
    lines.extend(f"- `{name}`" for name in report["blockers"])
    lines.extend(["", f"Report SHA-256: `{report['report_sha256']}`", ""])
    return "\n".join(lines)


def write_outputs(report: Mapping[str, Any] | None = None) -> dict[str, Any]:
    value = dict(report or analyze())
    validate_report_integrity(value)
    OUTPUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
    OUTPUT_MD.write_text(render_markdown(value), encoding="utf-8")
    return value


def main() -> int:
    report = write_outputs()
    radial = report["load_cases"]["radial_X_40N"]
    print(
        "follower mount screen FAIL: "
        f"moment={radial['moment_about_Y_Nmm'] / 1000:.3f}Nm; "
        f"row_couple={radial['ideal_M4_row_couple_N']:.1f}N"
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
