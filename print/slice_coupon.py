"""Reproducibly slice the PETG qualification coupon without printer contact."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import subprocess

import prepare_coupon
import prepare_profile


ROOT = Path(__file__).resolve().parents[1]
PROFILE_OUT = ROOT / "out" / "print" / "profile"
COUPON_OUT = ROOT / "out" / "print" / "coupon"
SLICER_INPUT = COUPON_OUT / "fit_bridge_coupon_a1_plate.stl"
FINAL_GCODE = COUPON_OUT / "fit_bridge_coupon.gcode"
TEMP_OUT = ROOT / "out" / "print" / ".coupon-slice"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def build_command(binary: Path, wrapper: dict, output_dir: Path,
                  input_path: Path) -> list[str]:
    settings = wrapper.get("native_settings")
    filaments = wrapper.get("native_filaments")
    if not isinstance(settings, list) or not settings:
        raise RuntimeError("Orca wrapper has no native_settings")
    if not isinstance(filaments, list) or not filaments:
        raise RuntimeError("Orca wrapper has no native_filaments")
    return [
        str(binary),
        "--load-settings", ";".join(str(row) for row in settings),
        "--load-filaments", ";".join(str(row) for row in filaments),
        "--outputdir", str(output_dir),
        "--slice", "0",
        str(input_path),
    ]


def slice_coupon() -> dict:
    profile = prepare_profile.prepare(PROFILE_OUT)
    wrapper_path = Path(profile["wrapper"])
    wrapper = json.loads(wrapper_path.read_text(encoding="utf-8"))
    binary = Path(profile["orcaslicer_bin"])

    # This only regenerates the source-equivalent, build-plate-translated STL.
    # It does not upload, connect to, or discover a printer.
    prepare_coupon.generate(verify_existing_gcode=False)

    resolved_root = (ROOT / "out" / "print").resolve()
    resolved_temp = TEMP_OUT.resolve()
    if resolved_temp.parent != resolved_root:
        raise RuntimeError("refusing slicer temporary directory outside out/print")
    if TEMP_OUT.exists():
        shutil.rmtree(TEMP_OUT)
    TEMP_OUT.mkdir(parents=True)

    command = build_command(binary, wrapper, TEMP_OUT, SLICER_INPUT)
    completed = subprocess.run(
        command,
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=600,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "OrcaSlicer failed closed with return code "
            f"{completed.returncode}: "
            f"{(completed.stderr or completed.stdout)[-2000:]}"
        )

    # Orca's Bambu CLI names a single-plate job plate_1.gcode regardless of
    # filename_format. Normalize that deterministic output into our artifact.
    generated = TEMP_OUT / "plate_1.gcode"
    if not generated.is_file() or generated.stat().st_size < 1024:
        raise RuntimeError("OrcaSlicer returned success without usable G-code")
    generated.replace(FINAL_GCODE)
    shutil.rmtree(TEMP_OUT)

    metadata = prepare_coupon.verify_gcode_profile(FINAL_GCODE)
    manifest = prepare_coupon.generate()
    result = {
        "status": "sliced_locally_not_sent_to_printer",
        "gcode": FINAL_GCODE.relative_to(ROOT).as_posix(),
        "bytes": FINAL_GCODE.stat().st_size,
        "sha256": _sha256(FINAL_GCODE),
        "profile_metadata": metadata,
        "manifest": prepare_coupon.MANIFEST.relative_to(ROOT).as_posix(),
        "production_release_allowed": manifest["production_release_allowed"],
    }
    return result


if __name__ == "__main__":
    print(json.dumps(slice_coupon(), indent=2))
