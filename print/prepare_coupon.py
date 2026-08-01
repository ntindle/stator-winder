"""Generate the explicit A1-plate STL and evidence manifest for the coupon."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys

from build123d import export_stl
import trimesh


ROOT = Path(__file__).resolve().parents[1]
CAD_DIR = ROOT / "cad"
OUT_DIR = ROOT / "out" / "print" / "coupon"
PLATE_STL = OUT_DIR / "fit_bridge_coupon_a1_plate.stl"
MANIFEST = OUT_DIR / "coupon-manifest.json"
PROFILE_SOURCE = ROOT / "print" / "profiles" / "winder_a1_0p4_petg_strength.json"

sys.path.insert(0, str(CAD_DIR))
from fit_bridge_coupon import coupon_spec, gen_a1_plate_part  # noqa: E402


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _record(path: Path, role: str) -> dict:
    return {
        "role": role,
        "path": path.relative_to(ROOT).as_posix(),
        "bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def verify_gcode_profile(path: Path) -> dict:
    """Fail closed if the sliced artifact's embedded settings drift."""
    lock = json.loads(PROFILE_SOURCE.read_text(encoding="utf-8"))
    expected = {
        "curr_bed_type": lock["machine"]["build_plate"],
        "layer_height": str(lock["process"]["layer_height_mm"]),
        "wall_loops": str(lock["process"]["walls"]),
        "top_shell_layers": str(lock["process"]["top_layers"]),
        "bottom_shell_layers": str(lock["process"]["bottom_layers"]),
        "sparse_infill_density": f'{lock["process"]["infill_percent"]}%',
        "sparse_infill_pattern": lock["process"]["infill_pattern"],
        "enable_support": "0",
        "filament_density": str(lock["filament"]["density_g_cm3"]),
        "filament_flow_ratio": str(lock["filament"]["flow_ratio"]),
        "filament_max_volumetric_speed": str(
            lock["filament"]["max_volumetric_speed_mm3_s"]
        ),
        "nozzle_temperature": str(lock["filament"]["nozzle_temp_c"]),
        "textured_plate_temp": str(
            lock["filament"]["textured_plate_temp_c"]
        ),
        "fan_min_speed": str(lock["filament"]["fan_min_percent"]),
        "fan_max_speed": str(lock["filament"]["fan_max_percent"]),
    }
    actual: dict[str, str] = {}
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for index, line in enumerate(stream):
            if index > 5000:
                break
            if not line.startswith("; ") or " = " not in line:
                continue
            key, value = line[2:].rstrip("\r\n").split(" = ", 1)
            if key in expected:
                actual[key] = value
    mismatch = {
        key: {"expected": expected[key], "actual": actual.get(key)}
        for key in expected
        if actual.get(key) != expected[key]
    }
    if mismatch:
        raise RuntimeError(f"sliced G-code profile metadata drift: {mismatch}")
    return {"profile_metadata_verified": True, "values": actual}


def generate(*, verify_existing_gcode: bool = True) -> dict:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    export_stl(
        gen_a1_plate_part(),
        str(PLATE_STL),
        tolerance=0.02,
        angular_tolerance=0.05,
    )

    mesh = trimesh.load_mesh(PLATE_STL)
    components = len(mesh.split(only_watertight=False))
    bounds = mesh.bounds.tolist()
    valid = (
        mesh.is_watertight
        and mesh.is_winding_consistent
        and components == 1
        and bounds[0][0] >= 0.0
        and bounds[0][1] >= 0.0
        and bounds[0][2] >= 0.0
        and bounds[1][0] <= 256.0
        and bounds[1][1] <= 256.0
        and bounds[1][2] <= 256.0
    )
    if not valid:
        raise RuntimeError("A1 coupon slicer mesh failed closed validation")

    artifacts = []
    for role, relative in (
        ("canonical_step", "fit_bridge_coupon.step"),
        ("canonical_stl", "fit_bridge_coupon.stl"),
        ("canonical_3mf_inspection_only", "fit_bridge_coupon.3mf"),
        ("a1_plate_stl", PLATE_STL.name),
        ("sliced_gcode", "fit_bridge_coupon.gcode"),
    ):
        path = OUT_DIR / relative
        if path.is_file():
            artifacts.append(_record(path, role))

    gcode_path = OUT_DIR / "fit_bridge_coupon.gcode"
    report = {
        "status": "physical_coupon_test_required",
        "production_release_allowed": False,
        "coupon": coupon_spec(),
        "slicer_input": {
            "source_geometry_unchanged": True,
            "translation_mm": [128.0, 128.0, 0.0],
            "mesh_watertight": bool(mesh.is_watertight),
            "mesh_winding_consistent": bool(mesh.is_winding_consistent),
            "mesh_components": components,
            "bounds_mm": bounds,
        },
        "sliced_gcode": (
            verify_gcode_profile(gcode_path)
            if verify_existing_gcode and gcode_path.is_file()
            else None
        ),
        "artifacts": artifacts,
    }
    MANIFEST.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


if __name__ == "__main__":
    print(json.dumps(generate(), indent=2))
