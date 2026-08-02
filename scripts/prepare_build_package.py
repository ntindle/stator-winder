"""Stage the project-owned alpha hardware package and write its manifest.

The allowlist is intentionally explicit. Supplier CAD, downloaded datasheets,
converted vendor previews, simulation reports, and the exact local assembly
must never enter ``hardware/`` through this script.
"""

from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
PACKAGE = ROOT / "hardware"
ASSEMBLY = PACKAGE / "assembly" / "stator_winder_reference_envelope.step"
MANIFEST = PACKAGE / "manifests" / "build-package.json"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _copy(source: Path, destination: Path, category: str,
          records: list[dict]) -> None:
    source = source.resolve()
    destination = destination.resolve()
    if ROOT.resolve() not in source.parents:
        raise RuntimeError(f"source escapes project root: {source}")
    if PACKAGE.resolve() not in destination.parents:
        raise RuntimeError(f"destination escapes hardware package: {destination}")
    if not source.is_file() or source.stat().st_size == 0:
        raise FileNotFoundError(source)
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    records.append({
        "path": destination.relative_to(ROOT).as_posix(),
        "source": source.relative_to(ROOT).as_posix(),
        "category": category,
        "bytes": destination.stat().st_size,
        "sha256": _sha256(destination),
    })


def _copy_glob(source_dir: str, pattern: str, destination_dir: str,
               category: str, records: list[dict]) -> None:
    paths = sorted((ROOT / source_dir).glob(pattern))
    if not paths:
        raise FileNotFoundError(f"no {pattern} files under {source_dir}")
    for source in paths:
        _copy(source, ROOT / destination_dir / source.name, category, records)


def _generate_public_assembly() -> None:
    ASSEMBLY.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run([
        sys.executable,
        str(ROOT / "cad" / "assembly.py"),
        "--reference-mode", "envelope",
        "--output", str(ASSEMBLY),
    ], cwd=ROOT, check=True)


def stage() -> dict:
    records: list[dict] = []
    _generate_public_assembly()
    records.append({
        "path": ASSEMBLY.relative_to(ROOT).as_posix(),
        "source": "cad/assembly.py --reference-mode envelope",
        "category": "reference_assembly",
        "bytes": ASSEMBLY.stat().st_size,
        "sha256": _sha256(ASSEMBLY),
    })

    _copy_glob("out/stl", "*.stl", "hardware/printables/stl",
               "printable_stl", records)
    _copy_glob("out/step", "*.step", "hardware/printables/step",
               "printable_step", records)
    _copy_glob("out/custom/step", "*.step", "hardware/machining/legacy",
               "custom_machining_step", records)
    _copy_glob("out/custom/successor/step", "*.step",
               "hardware/machining/successor",
               "successor_machining_step", records)
    _copy_glob("out/order", "*.csv", "hardware/orders", "order_csv", records)

    for name in ("manifest.json",):
        _copy(ROOT / "out" / "custom" / name,
              PACKAGE / "machining" / "legacy" / name,
              "custom_manifest", records)
        _copy(ROOT / "out" / "custom" / "successor" / name,
              PACKAGE / "machining" / "successor" / name,
              "successor_manifest", records)
    _copy(ROOT / "out" / "custom" / "successor" / "successor_rfq.csv",
          PACKAGE / "machining" / "successor" / "successor_rfq.csv",
          "successor_rfq", records)

    _copy(ROOT / "cad" / "fabricated_carriage.dxf",
          PACKAGE / "fabrication" / "fabricated_carriage.dxf",
          "fabrication_dxf", records)

    drawing_names = (
        "bearing_retention_spacers.pdf",
        "dancer_standoff_sleeves.pdf",
        "extrusion_cut_list.pdf",
        "felt_tensioner_consumables.pdf",
        "fixed_eyelet_id4_od9_t3.pdf",
        "flyer_shaft_d10_id6_to_id9_l79.pdf",
        "shaft_wrap_sleeve_family.pdf",
        "shaft8_socket_holder.pdf",
        "successor_custom_parts_rfq.pdf",
        "t8x8_leadscrew_188_journal30.pdf",
    )
    for name in drawing_names:
        _copy(ROOT / "output" / "pdf" / name,
              PACKAGE / "drawings" / name, "shop_drawing", records)

    for name in ("fit_bridge_coupon.step", "fit_bridge_coupon.stl",
                 "fit_bridge_coupon.3mf", "coupon-manifest.json"):
        _copy(ROOT / "out" / "print" / "coupon" / name,
              PACKAGE / "coupon" / name, "proof_coupon", records)

    records.sort(key=lambda row: row["path"])
    payload = {
        "schema": "stator-winder-build-package/v1",
        "release_status": "alpha_research_not_production_authorized",
        "third_party_cad_included": False,
        "files": records,
        "summary": {
            "files": len(records),
            "bytes": sum(row["bytes"] for row in records),
            "printable_stl": sum(row["category"] == "printable_stl"
                                  for row in records),
            "printable_step": sum(row["category"] == "printable_step"
                                   for row in records),
            "custom_machining_step": sum(
                row["category"] == "custom_machining_step" for row in records),
            "successor_machining_step": sum(
                row["category"] == "successor_machining_step"
                for row in records),
        },
    }
    MANIFEST.parent.mkdir(parents=True, exist_ok=True)
    MANIFEST.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return payload


def verify() -> dict:
    payload = json.loads(MANIFEST.read_text(encoding="utf-8"))
    for row in payload["files"]:
        path = ROOT / row["path"]
        if not path.is_file():
            raise FileNotFoundError(path)
        if path.stat().st_size != row["bytes"]:
            raise RuntimeError(f"size mismatch: {path}")
        if _sha256(path) != row["sha256"]:
            raise RuntimeError(f"checksum mismatch: {path}")
    if payload.get("third_party_cad_included") is not False:
        raise RuntimeError("build package must exclude third-party CAD")
    return payload


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    payload = verify() if args == ["--verify"] else stage()
    print(json.dumps(payload["summary"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
