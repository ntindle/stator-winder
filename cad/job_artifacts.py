"""Regenerate every measured-input winding job artifact from one command.

The supplier nominal wire and liner dimensions are useful defaults for the
digital twin, but they are not a physical receiving certificate.  A hardware
job supplies the conservative measured finished-wire OD and installed liner
thickness here.  Packing geometry, controller waypoints and settings are then
generated from the same in-memory report and hash-bound in a small manifest.

This module deliberately stops before capture/player generation until the
packed moving-wire route gate is PASS.  A settings file must never imply that
an unproved route is ready for hardware.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SIM = ROOT / "sim"
OUT = ROOT / "out"
REPORTS = OUT / "reports"
if str(SIM) not in sys.path:
    sys.path.insert(0, str(SIM))

from params import DEFAULT_STATOR  # noqa: E402
import settings_gen  # noqa: E402
import slot_packing_audit  # noqa: E402
import slot_packing  # noqa: E402


MANIFEST_SCHEMA = "measured-winding-job/v1"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def generate(
    wire_measured_mm: float,
    liner_measured_mm: float,
    *,
    output_root: Path = OUT,
) -> dict:
    output_root = Path(output_root)
    reports = output_root / "reports"
    reports.mkdir(parents=True, exist_ok=True)

    job = slot_packing_audit.PackingInput(
        float(wire_measured_mm), float(liner_measured_mm))
    packing = slot_packing_audit.analyze(job)
    if packing["status"] != "PASS":
        raise RuntimeError("measured slot packing is not PASS")
    packing_json = reports / "slot_packing.json"
    packing_md = reports / "slot_packing.md"
    slot_packing_audit.write_outputs(packing, packing_json, packing_md)

    plan_json = reports / "slot_winding_plan.json"
    plan_md = reports / "slot_winding_plan.md"
    plan = slot_packing.write_reports(
        plan_json, plan_md, report=packing)
    if plan["selected_case"]["status"] != "PASS":
        raise RuntimeError("measured controller winding plan is not PASS")

    spec = replace(DEFAULT_STATOR, wire_d=job.wire_d_mm)
    settings = settings_gen.derive(
        spec,
        liner_t_mm=job.liner_t_mm,
        winding_plan_path="reports/slot_winding_plan.json",
        winding_plan_proof_sha256=plan["proof_sha256"],
    )
    settings_path = output_root / "settings.yml"
    settings_path.write_text(settings_gen.to_yaml(settings), encoding="utf-8")

    manifest = {
        "schema": MANIFEST_SCHEMA,
        "status": "PACKING_PASS_ROUTE_PENDING",
        "inputs": {
            "wire_finished_diameter_measured_mm": job.wire_d_mm,
            "liner_installed_thickness_measured_mm": job.liner_t_mm,
            "measurement_rule": (
                "use the conservative received maximum including calibrated "
                "instrument uncertainty; never substitute catalog nominal"
            ),
        },
        "artifacts": {
            "slot_packing": {
                "path": "reports/slot_packing.json",
                "report_sha256": packing["report_sha256"],
                "file_sha256": _sha256(packing_json),
            },
            "slot_winding_plan": {
                "path": "reports/slot_winding_plan.json",
                "proof_sha256": plan["proof_sha256"],
                "file_sha256": _sha256(plan_json),
            },
            "settings": {
                "path": "settings.yml",
                "file_sha256": _sha256(settings_path),
            },
        },
        "next_required_gate": (
            "regenerate packed moving-wire routes and require every crossing "
            "and continuous capture interval PASS before hardware motion"
        ),
    }
    encoded = json.dumps(
        manifest, sort_keys=True, separators=(",", ":"), allow_nan=False)
    manifest["manifest_sha256"] = hashlib.sha256(encoded.encode()).hexdigest()
    manifest_path = reports / "measured_winding_job.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--wire-measured", type=float,
        default=slot_packing_audit.SUPPLIER_NOMINAL_WIRE_MM,
        help="conservative measured finished magnet-wire OD, mm",
    )
    parser.add_argument(
        "--liner-measured", type=float,
        default=slot_packing_audit.SUPPLIER_NOMINAL_LINER_MM,
        help="conservative measured installed liner thickness, mm",
    )
    parser.add_argument("--output-root", type=Path, default=OUT)
    args = parser.parse_args()
    manifest = generate(
        args.wire_measured, args.liner_measured,
        output_root=args.output_root,
    )
    print(
        "measured winding job packing PASS; moving-wire route still required; "
        f"manifest={manifest['manifest_sha256']}"
    )


if __name__ == "__main__":
    main()
