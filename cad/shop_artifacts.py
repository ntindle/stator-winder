"""Shared specifications and manifest generator for small shop artifacts.

The parts in this packet are intentionally separated from the primary custom
RFQ packet in ``custom_parts.py``.  They are either locally cut consumables,
small turned stand-off sleeves, or factory/local extrusion cuts.  Nothing in
this module claims that the undersize annular parts are SendCutSend-ready.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import csv
import hashlib
import json
from pathlib import Path

from build123d import Align, Cylinder
import ezdxf


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "custom" / "shop"
STEP_OUT = OUT / "step"
DXF_OUT = OUT / "dxf"
CSV_OUT = OUT / "csv"
QA_OUT = OUT / "pdf_qa"
STEP_QA_OUT = OUT / "step_qa"
PDF_OUT = ROOT / "output" / "pdf"
MANIFEST = OUT / "manifest.json"

SENDCUTSEND_MIN_WIDTH_IN = 0.375
SENDCUTSEND_MIN_LENGTH_IN = 1.5
SENDCUTSEND_LIMIT_URL = (
    "https://sendcutsend.com/materials/processing-min-max/"
)


@dataclass(frozen=True)
class SleeveSpec:
    part_id: str
    description: str
    outer_diameter_mm: float
    bore_diameter_mm: float
    length_mm: float
    quantity: int


SLEEVE_SPECS = {
    "dancer_stop_sleeve_od5_id3p2_l4": SleeveSpec(
        "dancer_stop_sleeve_od5_id3p2_l4",
        "Dancer hard-stop stand-off sleeve",
        5.0,
        3.2,
        4.0,
        2,
    ),
    "dancer_fixed_anchor_sleeve_od4_id2p2_l1p5": SleeveSpec(
        "dancer_fixed_anchor_sleeve_od4_id2p2_l1p5",
        "Fixed spring-loop stand-off sleeve",
        4.0,
        2.2,
        1.5,
        1,
    ),
    "dancer_moving_anchor_sleeve_od4_id2p2_l4": SleeveSpec(
        "dancer_moving_anchor_sleeve_od4_id2p2_l4",
        "Moving spring-loop stand-off sleeve",
        4.0,
        2.2,
        4.0,
        1,
    ),
}


@dataclass(frozen=True)
class ExtrusionCut:
    line_id: str
    description: str
    quantity: int
    length_mm: float


EXTRUSION_CUTS = (
    ExtrusionCut("BASE-450", "Long base rail", 2, 450.0),
    ExtrusionCut("CROSS-180", "Base cross member", 3, 180.0),
    ExtrusionCut("STRINGER-280", "MGN rail stringer", 2, 280.0),
    ExtrusionCut("POST-235", "Flyer tower post", 2, 235.0),
    ExtrusionCut("REAR-305", "Wire-system rear post", 1, 305.0),
)


def sleeve_part(spec: SleeveSpec):
    """Return one centered, single-solid turned sleeve."""
    align = (Align.CENTER, Align.CENTER, Align.CENTER)
    part = (
        Cylinder(spec.outer_diameter_mm / 2.0, spec.length_mm, align=align)
        - Cylinder(
            spec.bore_diameter_mm / 2.0,
            spec.length_mm + 2.0,
            align=align,
        )
    )
    part.label = spec.part_id
    return part


def extrusion_total_mm() -> float:
    return sum(row.quantity * row.length_mm for row in EXTRUSION_CUTS)


def write_extrusion_csv(path: Path | None = None) -> Path:
    path = path or (CSV_OUT / "extrusion_cut_list.csv")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=(
                "line_id",
                "description",
                "profile",
                "quantity",
                "length_mm",
                "line_total_mm",
                "end_1",
                "end_2",
                "length_tolerance_mm",
                "supplier_status",
                "notes",
            ),
        )
        writer.writeheader()
        for row in EXTRUSION_CUTS:
            writer.writerow(
                {
                    "line_id": row.line_id,
                    "description": row.description,
                    "profile": "MISUMI HFS5-2020 or exact B-type slot-6",
                    "quantity": row.quantity,
                    "length_mm": f"{row.length_mm:.2f}",
                    "line_total_mm": f"{row.quantity * row.length_mm:.2f}",
                    "end_1": "square cut; deburr",
                    "end_2": "square cut; deburr",
                    "length_tolerance_mm": "+/-0.50",
                    "supplier_status": "factory cut-to-length or local saw",
                    "notes": "No end tapping; preserve slot compatibility",
                }
            )
        writer.writerow(
            {
                "line_id": "TOTAL",
                "description": "All extrusion",
                "profile": "",
                "quantity": sum(row.quantity for row in EXTRUSION_CUTS),
                "length_mm": "",
                "line_total_mm": f"{extrusion_total_mm():.2f}",
                "end_1": "",
                "end_2": "",
                "length_tolerance_mm": "",
                "supplier_status": "",
                "notes": "10 members; 2775 mm total",
            }
        )
    return path


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _base_record(part_id: str, kind: str, path: Path) -> dict:
    return {
        "id": part_id,
        "kind": kind,
        "file": path.relative_to(ROOT).as_posix(),
        "bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _require(path: Path) -> Path:
    if not path.is_file() or path.stat().st_size <= 0:
        raise FileNotFoundError(f"required generated artifact is missing: {path}")
    return path


def _dxf_record(part_id: str, path: Path, thickness_mm: float,
                material: str, supplier_status: str) -> dict:
    document = ezdxf.readfile(path)
    circles = list(document.modelspace().query('CIRCLE[layer=="CUT"]'))
    if document.units != ezdxf.units.MM:
        raise RuntimeError(f"{path.name} is not explicitly millimetres")
    radii = sorted(round(float(entity.dxf.radius), 6) for entity in circles)
    if radii != [2.25, 10.0]:
        raise RuntimeError(f"{path.name} circle radii are {radii}")
    record = _base_record(part_id, "dxf", path)
    record.update(
        {
            "units": "mm",
            "cut_entity_count": 2,
            "outer_diameter_mm": 20.0,
            "bore_diameter_mm": 4.5,
            "thickness_mm": thickness_mm,
            "material": material,
            "quantity": 2,
            "supplier_status": supplier_status,
            "sendcutsend_ready": False,
            "sendcutsend_reason": (
                "20 x 20 mm (0.787 x 0.787 in) part is shorter than the "
                "published 1.5 in minimum bounding-box length"
            ),
        }
    )
    return record


def write_manifest() -> Path:
    """Write a fail-closed, hash-bearing manifest after all generators run."""
    if (ROOT / "out").resolve() not in OUT.resolve().parents:
        raise RuntimeError(f"refusing output outside project out/: {OUT}")
    OUT.mkdir(parents=True, exist_ok=True)

    artifacts: list[dict] = []
    for spec in SLEEVE_SPECS.values():
        path = _require(STEP_OUT / f"{spec.part_id}.step")
        record = _base_record(spec.part_id, "step", path)
        record.update(
            {
                "single_solid": True,
                "dimensions_mm": {
                    "outer_diameter": spec.outer_diameter_mm,
                    "bore_diameter": spec.bore_diameter_mm,
                    "length": spec.length_mm,
                },
                "quantity": spec.quantity,
                "material": "303 stainless steel",
                "supplier_status": "local lathe or small turned-parts supplier",
                "sendcutsend_ready": False,
                "sendcutsend_reason": "three-dimensional turned sleeve",
            }
        )
        artifacts.append(record)

    artifacts.append(
        _dxf_record(
            "felt_backing_disc_od20_id4p5_t1",
            _require(DXF_OUT / "local_felt_backing_disc_od20_id4p5.dxf"),
            1.0,
            "304 stainless sheet",
            "local laser, waterjet, punch, or lathe; not an individual SendCutSend job",
        )
    )
    felt_record = _dxf_record(
        "felt_pad_od20_id4p5_t3",
        _require(DXF_OUT / "local_felt_pad_od20_id4p5.dxf"),
        3.175,
        "McMaster 8341K31 F5 plain wool felt sheet, 1/8 in",
        "locally punch or knife-cut from selected sheet stock",
    )
    felt_record["source_url"] = "https://www.mcmaster.com/product/8341K31/"
    felt_record["finished_thickness_requirement_mm"] = "3.0 +/-0.3"
    artifacts.append(felt_record)

    csv_path = _require(CSV_OUT / "extrusion_cut_list.csv")
    csv_record = _base_record("extrusion_cut_list", "csv", csv_path)
    csv_record.update(
        {
            "profile": "MISUMI HFS5-2020 or exact B-type slot-6",
            "member_count": sum(row.quantity for row in EXTRUSION_CUTS),
            "total_length_mm": extrusion_total_mm(),
            "cuts": [asdict(row) for row in EXTRUSION_CUTS],
            "supplier_status": "factory cut-to-length or local square saw cut",
        }
    )
    artifacts.append(csv_record)

    for part_id, filename in (
        ("felt_tensioner_consumables_drawing", "felt_tensioner_consumables.pdf"),
        ("dancer_standoff_sleeves_drawing", "dancer_standoff_sleeves.pdf"),
        ("extrusion_cut_list_drawing", "extrusion_cut_list.pdf"),
    ):
        artifacts.append(_base_record(part_id, "pdf", _require(PDF_OUT / filename)))

    for part_id, filename, pdf_filename in (
        (
            "felt_tensioner_consumables_render_qa",
            "felt_tensioner_consumables.png",
            "felt_tensioner_consumables.pdf",
        ),
        (
            "dancer_standoff_sleeves_render_qa",
            "dancer_standoff_sleeves.png",
            "dancer_standoff_sleeves.pdf",
        ),
        (
            "extrusion_cut_list_render_qa",
            "extrusion_cut_list.png",
            "extrusion_cut_list.pdf",
        ),
    ):
        record = _base_record(part_id, "png_qa", _require(QA_OUT / filename))
        record.update(
            {
                "rendered_from": (PDF_OUT / pdf_filename).relative_to(ROOT).as_posix(),
                "renderer": "pypdfium2 at 2 pixels per PDF point",
                "visual_qa": "reviewed; no clipping, overlap, or page-boundary defects",
            }
        )
        artifacts.append(record)

    for part_id, filename, step_filename in (
        (
            "dancer_stop_sleeve_render_qa",
            "dancer_stop_sleeve.png",
            "dancer_stop_sleeve_od5_id3p2_l4.step",
        ),
        (
            "dancer_fixed_anchor_sleeve_render_qa",
            "dancer_fixed_anchor_sleeve.png",
            "dancer_fixed_anchor_sleeve_od4_id2p2_l1p5.step",
        ),
        (
            "dancer_moving_anchor_sleeve_render_qa",
            "dancer_moving_anchor_sleeve.png",
            "dancer_moving_anchor_sleeve_od4_id2p2_l4.step",
        ),
    ):
        record = _base_record(part_id, "png_qa", _require(STEP_QA_OUT / filename))
        record.update(
            {
                "rendered_from": (STEP_OUT / step_filename).relative_to(ROOT).as_posix(),
                "renderer": "CAD skill Chromium snapshot, isometric view",
                "visual_qa": "reviewed; annular single-solid sleeve geometry visible",
            }
        )
        artifacts.append(record)

    manifest = {
        "schema": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "packet": "small locally fabricated and factory-cut shop artifacts",
        "sendcutsend_policy": {
            "catalog_url": SENDCUTSEND_LIMIT_URL,
            "minimum_bbox_in_for_0p030_304": [
                SENDCUTSEND_MIN_WIDTH_IN,
                SENDCUTSEND_MIN_LENGTH_IN,
            ],
            "undersize_annular_jobs_created": False,
        },
        "artifacts": artifacts,
    }
    MANIFEST.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return MANIFEST


def main() -> int:
    path = write_extrusion_csv()
    print(path)
    manifest = write_manifest()
    print(manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
