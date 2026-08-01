"""Fail-closed materialization of the project-owned OrcaSlicer wrapper."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PROFILE_DIR = ROOT / "print" / "profiles"
LOCK_SOURCE = PROFILE_DIR / "winder_a1_0p4_petg_strength.json"
PROCESS_SOURCE = PROFILE_DIR / "orca_winder_a1_0p4_petg_strength_process.json"
FILAMENT_SOURCE = PROFILE_DIR / "orca_inland_petg_plus_a1_0p4.json"
DEFAULT_ORCA_HOME = ROOT / ".tools" / "orcaslicer-2.4.2"


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _require_equal(actual, expected, label: str) -> None:
    if actual != expected:
        raise RuntimeError(f"{label}: expected {expected!r}, got {actual!r}")


def prepare(output_dir: Path | None = None,
            orca_home: Path | None = None) -> dict:
    lock = _load(LOCK_SOURCE)
    process = _load(PROCESS_SOURCE)
    filament = _load(FILAMENT_SOURCE)

    configured_home = os.environ.get("ORCASLICER_HOME")
    home = Path(configured_home) if configured_home else (
        orca_home if orca_home is not None else DEFAULT_ORCA_HOME)
    home = home.resolve()
    binary = home / "orca-slicer.exe"
    machine_profile = (
        home / "resources" / "profiles" / "BBL" / "machine"
        / "Bambu Lab A1 0.4 nozzle.json"
    )
    for required in (binary, machine_profile, LOCK_SOURCE,
                     PROCESS_SOURCE, FILAMENT_SOURCE):
        if not required.is_file():
            raise RuntimeError(
                f"required slicer/profile artifact missing: {required}")

    machine = _load(machine_profile)
    _require_equal(machine.get("name"), lock["machine"]["native_profile"],
                   "native machine profile")
    _require_equal(machine.get("nozzle_diameter"),
                   [str(lock["machine"]["nozzle_mm"])], "native nozzle")
    _require_equal(float(machine.get("printable_height")),
                   float(lock["machine"]["z_height_mm"]), "native Z height")
    _require_equal(process.get("wall_loops"),
                   str(lock["process"]["walls"]), "wall loops")
    _require_equal(process.get("sparse_infill_pattern"),
                   lock["process"]["infill_pattern"], "infill pattern")
    _require_equal(filament.get("nozzle_temperature"),
                   [str(lock["filament"]["nozzle_temp_c"])],
                   "filament nozzle temperature")

    out = (output_dir if output_dir is not None
           else ROOT / "out" / "print" / "profile").resolve()
    out.mkdir(parents=True, exist_ok=True)
    wrapper_path = out / "a1_0p4_petg_strength.wrapper.json"
    wrapper = {
        "backend": "orcaslicer",
        "native_config": str(machine_profile.resolve()),
        "native_settings": [
            str(machine_profile.resolve()),
            str(PROCESS_SOURCE.resolve()),
        ],
        "native_filaments": [str(FILAMENT_SOURCE.resolve())],
        "machine": {
            "name": "Bambu Lab A1 0.4 nozzle",
            "bed_size_mm": lock["machine"]["bed_size_mm"],
            "z_height_mm": lock["machine"]["z_height_mm"],
            "motion_bounds_mm": lock["machine"]["motion_bounds_mm"],
        },
        "filament": {
            "type": lock["filament"]["type"],
            "nozzle_temp_c": lock["filament"]["nozzle_temp_c"],
            "bed_temp_c": lock["filament"]["textured_plate_temp_c"],
        },
    }
    wrapper_path.write_text(json.dumps(wrapper, indent=2) + "\n",
                            encoding="utf-8")

    records = []
    for role, path in (
        ("profile_lock", LOCK_SOURCE),
        ("native_machine", machine_profile),
        ("project_process", PROCESS_SOURCE),
        ("project_filament", FILAMENT_SOURCE),
        ("slicer_binary", binary),
        ("wrapper", wrapper_path),
    ):
        records.append({
            "role": role,
            "path": str(path.resolve()),
            "bytes": path.stat().st_size,
            "sha256": _sha256(path),
        })
    report = {
        "status": "ready_for_local_slicing",
        "profile_id": lock["id"],
        "qualification_status": lock["status"],
        "wrapper": str(wrapper_path),
        "orcaslicer_bin": str(binary.resolve()),
        "records": records,
    }
    report_path = out / "profile-lock.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n",
                           encoding="utf-8")
    return {**report, "report": str(report_path)}


if __name__ == "__main__":
    print(json.dumps(prepare(), indent=2))
