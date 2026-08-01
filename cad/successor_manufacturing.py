"""Standalone manufacturing packet for the frozen integrated successor.

The normal-GOAL release candidate owns assembly placement and balance.  This
module exports the corresponding supplier-facing solids individually without
editing or re-exporting the candidate assembly itself.  All supplier, price,
material-lot, finish and coupon unknowns remain explicitly blocked in the
manifest and RFQ CSV.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from build123d import Part, Pos, export_step

import carriage_active_sector_terminal_guide as active
import integrated_release_candidate as candidate
import permanent_cap_offset_spoke_retained_review as retained
import retained_flyer_peek_guide_successor as flyer


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "custom" / "successor"
STEP_OUT = OUT / "step"
MANIFEST = OUT / "manifest.json"
RFQ_CSV = OUT / "successor_rfq.csv"
DRAWING = ROOT / "output" / "pdf" / "successor_custom_parts_rfq.pdf"


@dataclass(frozen=True)
class RfqContract:
    """Supplier-return and qualification contract for a quote-ready solid."""

    quote_requirements: tuple[str, ...]
    receiving_inspection: tuple[str, ...]
    qualification_before_use: tuple[str, ...]
    material_design_basis: str


@dataclass(frozen=True)
class ManufacturingPart:
    part_id: str
    part: Part
    material: str
    process: str
    group: str
    rfq: RfqContract
    quantity: int = 1
    geometry_authority: str = "step_exact"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _source_sha256(path: Path) -> str:
    return _sha256(path)


def _artifact_record(path: Path) -> dict[str, object]:
    """Return a root-relative, byte- and SHA-bound artifact record."""

    exists = path.is_file()
    return {
        "path": path.relative_to(ROOT).as_posix(),
        "exists": exists,
        "bytes": path.stat().st_size if exists else None,
        "sha256": _sha256(path) if exists else None,
    }


def _expected_source_paths() -> dict[str, Path]:
    """Direct source closure for the supplier packet and its drawing."""

    return {
        "cad/successor_manufacturing.py": Path(__file__),
        "cad/custom_drawings.py": ROOT / "cad" / "custom_drawings.py",
        "cad/integrated_release_candidate.py": Path(candidate.__file__),
        "cad/retained_flyer_peek_guide_successor.py": Path(flyer.__file__),
        "cad/permanent_cap_offset_spoke_retained_review.py": Path(
            retained.__file__
        ),
        # The short-leadin caps are sourced from this module even when the
        # optional active guide/yoke export is deferred.
        "cad/carriage_active_sector_terminal_guide.py": Path(active.__file__),
    }


def _root_artifact(value: object) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError("successor artifact path is missing")
    relative = Path(value)
    if relative.is_absolute():
        raise ValueError(f"successor artifact path must be relative: {value}")
    candidate_path = (ROOT / relative).resolve(strict=False)
    try:
        candidate_path.relative_to(ROOT.resolve())
    except ValueError as exc:
        raise ValueError(
            f"successor artifact escapes the machine root: {value}"
        ) from exc
    return candidate_path


def validate_manifest(manifest: dict) -> None:
    """Fail closed if any packet source or emitted artifact has drifted."""

    if manifest.get("schema") != "successor-manufacturing-packet/v1":
        raise ValueError("successor manifest schema drift")
    if manifest.get("production_authorized") is not False:
        raise ValueError("successor packet cannot authorize production")
    if manifest.get("order_authorized") is not False:
        raise ValueError("successor packet cannot authorize ordering")
    if not isinstance(manifest.get("rfq_submission_authorized"), bool):
        raise ValueError("successor packet RFQ authorization is malformed")

    expected_sources = {
        relative: _source_sha256(path)
        for relative, path in _expected_source_paths().items()
    }
    if manifest.get("source_hashes") != expected_sources:
        raise ValueError("successor manifest source hash drift")

    source_paths = _expected_source_paths()
    newest_packet_source_mtime_ns = max(
        path.stat().st_mtime_ns for path in source_paths.values()
    )
    drawing_source_mtime_ns = max(
        path.stat().st_mtime_ns
        for relative, path in source_paths.items()
        if relative != "cad/successor_manufacturing.py"
    )

    records = manifest.get("parts")
    if not isinstance(records, list) or not records:
        raise ValueError("successor manifest has no STEP part records")
    part_ids: set[str] = set()
    part_paths: set[str] = set()
    for record in records:
        if not isinstance(record, dict):
            raise ValueError("successor STEP record is malformed")
        part_id = record.get("id")
        relative = record.get("file")
        if not isinstance(part_id, str) or not part_id:
            raise ValueError("successor STEP record has no part id")
        if part_id in part_ids:
            raise ValueError(f"duplicate successor part id: {part_id}")
        if not isinstance(relative, str) or relative in part_paths:
            raise ValueError(f"duplicate or invalid successor STEP path: {relative}")
        part_ids.add(part_id)
        part_paths.add(relative)
        path = _root_artifact(relative)
        if not path.is_file():
            raise ValueError(f"successor STEP is missing: {relative}")
        if record.get("bytes") != path.stat().st_size:
            raise ValueError(f"successor STEP byte count drift: {relative}")
        if record.get("sha256") != _sha256(path):
            raise ValueError(f"successor STEP hash drift: {relative}")
        if record.get("single_solid") is not True:
            raise ValueError(f"successor STEP is not one solid: {relative}")
        if record.get("candidate_purchase_status") != "rfq_ready":
            raise ValueError(f"successor STEP is not RFQ-ready: {relative}")
        if record.get("purchase_status") != "rfq_ready":
            raise ValueError(f"successor STEP handoff is not RFQ-ready: {relative}")
        if record.get("authorization_status") != "blocked":
            raise ValueError(
                f"successor STEP production authorization drift: {relative}"
            )
        if record.get("rfq_submission_authorized") is not True:
            raise ValueError(
                f"successor STEP quote-submission authorization drift: {relative}"
            )
        contract = record.get("rfq_contract")
        if not isinstance(contract, dict) or set(contract) != {
            "material_design_basis",
            "quote_requirements",
            "receiving_inspection",
            "qualification_before_use",
        }:
            raise ValueError(f"successor RFQ contract drift: {relative}")
        if not isinstance(contract["material_design_basis"], str) or not (
            contract["material_design_basis"].strip()
        ):
            raise ValueError(f"successor material basis is missing: {relative}")
        for field in (
            "quote_requirements",
            "receiving_inspection",
            "qualification_before_use",
        ):
            values = contract[field]
            if (
                not isinstance(values, list)
                or not values
                or not all(isinstance(value, str) and value.strip()
                           for value in values)
            ):
                raise ValueError(
                    f"successor RFQ contract {field} is incomplete: {relative}"
                )
        if path.stat().st_mtime_ns < newest_packet_source_mtime_ns:
            raise ValueError(f"successor STEP predates its source closure: {relative}")

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict) or set(artifacts) != {
        "rfq_csv", "drawing_pdf",
    }:
        raise ValueError("successor packet artifact contract drift")
    expected_artifacts = {
        "rfq_csv": RFQ_CSV,
        "drawing_pdf": DRAWING,
    }
    for name, path in expected_artifacts.items():
        record = artifacts.get(name)
        if not isinstance(record, dict):
            raise ValueError(f"successor {name} record is malformed")
        if record != _artifact_record(path):
            raise ValueError(f"successor {name} hash or byte count drift")
    if not RFQ_CSV.is_file():
        raise ValueError("successor RFQ CSV is missing")
    if RFQ_CSV.stat().st_mtime_ns < newest_packet_source_mtime_ns:
        raise ValueError("successor RFQ CSV predates its source closure")
    packet_complete = not manifest.get("pending_rev2_packaging_parts") and not (
        manifest.get("pending_supplier_authority_parts")
    )
    expected_rfq_authorized = bool(records) and packet_complete and all(
        record.get("rfq_submission_authorized") is True for record in records
    )
    if manifest.get("rfq_submission_authorized") is not expected_rfq_authorized:
        raise ValueError("successor packet RFQ authorization drift")
    readiness = manifest.get("rfq_readiness")
    expected_ready_ids = sorted(record["id"] for record in records)
    if not isinstance(readiness, dict) or readiness.get(
        "rfq_ready_part_ids"
    ) != expected_ready_ids:
        raise ValueError("successor packet RFQ-ready part index drift")
    if readiness.get("order_ready_part_ids") != []:
        raise ValueError("successor packet cannot contain order-ready custom parts")
    if readiness.get("production_authorized_part_ids") != []:
        raise ValueError(
            "successor packet cannot contain production-authorized custom parts"
        )
    if packet_complete and not DRAWING.is_file():
        raise ValueError("complete successor packet is missing its drawing PDF")
    if DRAWING.is_file() and DRAWING.stat().st_mtime_ns < drawing_source_mtime_ns:
        raise ValueError("successor drawing PDF predates its source closure")


def _centered(part: Part, label: str) -> Part:
    box = part.bounding_box()
    centered = Pos(
        -(float(box.min.X) + float(box.max.X)) / 2.0,
        -(float(box.min.Y) + float(box.max.Y)) / 2.0,
        -(float(box.min.Z) + float(box.max.Z)) / 2.0,
    ) * part
    centered.label = label
    return centered


PEEK_MATERIAL_BASIS = (
    "Natural, virgin, unfilled PEEK stock shape. Supplier must identify the "
    "stock-shape manufacturer, product/grade, filler declaration and traceable "
    "lot; no recycled, glass-, carbon-, PTFE- or bearing-filled substitution."
)
PEEK_COMMON_QUOTE = (
    "Confirm the supplied STEP is the exact shape authority and identify any "
    "feature that cannot be machined without changing it.",
    "Return the stock-shape manufacturer, product/grade, filler declaration, "
    "lot-certificate availability and proposed machining process.",
    "Propose an achievable wire-contact surface finish and the inspection "
    "method used to verify burr-free polished channels.",
    "Quote a first-article dimensional inspection report against the STEP.",
)
PEEK_COMMON_RECEIVING = (
    "Verify the certificate and traceable lot identify natural virgin unfilled "
    "PEEK with no filler substitution.",
    "Review the supplier first-article report and independently inspect all "
    "retention interfaces and wire-contact passages against the STEP.",
    "Inspect every wire-contact edge and passage for burrs, chips, closure, "
    "bridging and polish damage before assembly.",
)


def _peek_rfq(
    *qualification: str,
    extra_quote: tuple[str, ...] = (),
    extra_receiving: tuple[str, ...] = (),
) -> RfqContract:
    return RfqContract(
        quote_requirements=PEEK_COMMON_QUOTE + extra_quote,
        receiving_inspection=PEEK_COMMON_RECEIVING + extra_receiving,
        qualification_before_use=qualification,
        material_design_basis=PEEK_MATERIAL_BASIS,
    )


FLYER_GUIDE_RFQ = _peek_rfq(
    "Pass the M2 insert pull coupon, guide-to-arm fit check, wire abrasion, "
    "dielectric, varnish compatibility and 60 N hot-load coupons.",
    "Pass the installed flexible-wire route and hot 300 RPM endurance gates.",
    extra_quote=(
        "State how the continuous ID0.60 passage, R3.25 root elbow and integral "
        "exit bell will be produced and inspected without closing the bore.",
    ),
    extra_receiving=(
        "Verify the continuous ID0.60 passage is open end-to-end and inspect "
        "its root elbow and exit bell with the supplier's stated method.",
    ),
)
CAP_RFQ = _peek_rfq(
    "Pass paired-cap fit, M2 retention, wire abrasion, dielectric and "
    "hot-varnish coupons.",
    "Pass the flexible-conductor launch matrix before winding.",
    extra_quote=(
        "Quote the front and rear caps as a matched pair from one material lot; "
        "preserve all 24 two-branch short-leadin sectors (48 open branch "
        "grooves) on each cap.",
    ),
    extra_receiving=(
        "Verify all 24 two-branch short-leadin sectors (48 open branch grooves) "
        "per cap, matched front/rear identity and the shared through-fastener "
        "interfaces.",
    ),
)
ACTIVE_GUIDE_RFQ = _peek_rfq(
    "Pass guide-key fit, all four M3 insert pull tests, wire abrasion and "
    "flexible-conductor capture coupons.",
    "Pass the full launch matrix and hot 300 RPM endurance gates.",
    extra_quote=(
        "Quote the front and rear guides as a matched pair from one material "
        "lot; preserve the open bowls, datum keys and heat-insert pilots.",
    ),
    extra_receiving=(
        "Verify front/rear identity, all datum keys, open bowls and M3 insert "
        "pilots against the STEP before installing inserts.",
    ),
)
YOKE_RFQ = RfqContract(
    quote_requirements=(
        "Quote one-piece CNC machining from certified 6061-T6 aluminum; no "
        "alloy or temper substitution without written approval.",
        "Confirm all keyed guide/tower datums and M3/M4 clearance-hole axes are "
        "included and return a proposed dimensional inspection plan.",
        "State achievable tolerance, datum-face flatness and surface-treatment "
        "proposal; keep locating and mating faces free of dimensional buildup.",
        "Quote a first-article dimensional inspection report against the STEP.",
    ),
    receiving_inspection=(
        "Verify the material certificate states 6061-T6 and matches the "
        "serialized first article.",
        "Review the supplier first-article report and independently inspect the "
        "tower keys, guide seats, datum faces and every M3/M4 clearance hole.",
        "Confirm the yoke is one solid with no plugged, bonded or separately "
        "fastened structural substitutions.",
    ),
    qualification_before_use=(
        "Re-run the exact rigid sweep with the received-part measurements.",
        "Pass guide fit, flexible-conductor, launch-matrix and physical-load "
        "qualification before production use.",
    ),
    material_design_basis="6061-T6 aluminum; no equivalent is pre-authorized.",
)
B777_RFQ = RfqContract(
    quote_requirements=(
        "Quote each serialized STEP separately in ASTM B777 tungsten heavy "
        "alloy at the 18.49 g/cm3 balance-design density.",
        "State ASTM B777 class, certified density, chemistry, traceable lot, "
        "machining process and achievable dimensional tolerance.",
        "Return the predicted finished mass for each serial and quote an actual "
        "mass record with the finished parts.",
        "Flag any proposed density other than 18.49 g/cm3; it requires a new "
        "balance solve and six new STEP files before order approval.",
    ),
    receiving_inspection=(
        "Verify the ASTM B777 certificate, class, chemistry, traceable lot and "
        "certified density before accepting the parts.",
        "Measure and record each serial's OD, ID, axial length and finished mass.",
        "Reject mixed-up serials, rounded bearing faces, burrs or density drift "
        "until the balance solution is regenerated and reviewed.",
    ),
    qualification_before_use=(
        "Install all six measured serials and pass the final G2.5 balance check.",
        "Pass all rear/front retention pull tests and hot 300 RPM endurance.",
    ),
    material_design_basis=(
        "ASTM B777 tungsten heavy alloy at 18.49 g/cm3 (the immutable digital "
        "balance basis); grade/class and lot certificate must be returned by "
        "the supplier."
    ),
)


def manufacturing_parts() -> dict[str, ManufacturingPart]:
    """Return every non-FDM custom solid in its standalone supplier frame."""

    solution = candidate.integrated_balance_solution()
    rear_lengths = tuple(map(float, solution["rear_slug_lengths_mm"]))
    front_thickness = float(solution["front_trim_common_thickness_mm"])

    rows: list[ManufacturingPart] = [
        ManufacturingPart(
            "flyer_peek_guide_bell",
            _centered(flyer.peek_guide_insert(), "flyer_peek_guide_bell"),
            "natural unfilled PEEK; exact resin grade and lot certification TBD",
            "5-axis CNC or supplier-approved molded-and-finish-machined process",
            "wire_path",
            FLYER_GUIDE_RFQ,
        ),
        ManufacturingPart(
            "stator_short_leadin_cap_front",
            _centered(active.cap_with_short_leadins(1),
                      "stator_short_leadin_cap_front"),
            "natural unfilled PEEK; exact resin grade and lot certification TBD",
            "CNC machine and polish all open wire-contact channels",
            "wire_path",
            CAP_RFQ,
        ),
        ManufacturingPart(
            "stator_short_leadin_cap_rear",
            _centered(active.cap_with_short_leadins(-1),
                      "stator_short_leadin_cap_rear"),
            "natural unfilled PEEK; exact resin grade and lot certification TBD",
            "CNC machine and polish all open wire-contact channels",
            "wire_path",
            CAP_RFQ,
        ),
        ManufacturingPart(
            "active_sector_peek_guide_front",
            _centered(active.active_sector_guide(1),
                      "active_sector_peek_guide_front"),
            "natural unfilled PEEK; exact resin grade and lot certification TBD",
            "CNC machine open bowls, keys and heat-insert pilots; polish wire surfaces",
            "wire_path",
            ACTIVE_GUIDE_RFQ,
        ),
        ManufacturingPart(
            "active_sector_peek_guide_rear",
            _centered(active.active_sector_guide(-1),
                      "active_sector_peek_guide_rear"),
            "natural unfilled PEEK; exact resin grade and lot certification TBD",
            "CNC machine open bowls, keys and heat-insert pilots; polish wire surfaces",
            "wire_path",
            ACTIVE_GUIDE_RFQ,
        ),
        ManufacturingPart(
            "active_sector_aluminum_yoke",
            _centered(active.carriage_yoke(), "active_sector_aluminum_yoke"),
            "6061-T6 aluminum; no equivalent without written approval",
            "CNC mill as one solid; preserve keyed tower and guide datums",
            "structure",
            YOKE_RFQ,
        ),
    ]

    for index, (pocket, length) in enumerate(
        zip(retained.POCKETS, rear_lengths), start=1
    ):
        part_id = f"balance_b777_rear_{index}_{pocket.id}"
        rows.append(ManufacturingPart(
            part_id,
            _centered(retained.tungsten_slug(pocket, length), part_id),
            "ASTM B777 tungsten heavy alloy at 18.49 g/cm3 design density; "
            "class and certified lot TBD",
            "precision turn/mill annular trim to solved length; weigh and serialize",
            "balance",
            B777_RFQ,
        ))

    for side, x_mm in zip(("left", "right"), flyer.FRONT_TRIM_X_MM):
        part_id = f"balance_b777_front_{side}"
        rows.append(ManufacturingPart(
            part_id,
            _centered(flyer.front_trim_slug(x_mm, front_thickness), part_id),
            "ASTM B777 tungsten heavy alloy at 18.49 g/cm3 design density; "
            "class and certified lot TBD",
            "precision turn annular trim to common solved thickness; weigh and serialize",
            "balance",
            B777_RFQ,
        ))

    result = {row.part_id: row for row in rows}
    if len(result) != 12:
        raise RuntimeError(f"expected 12 custom successor parts, got {len(result)}")
    return result


ACTIVE_SECTOR_PENDING_IDS = {
    "active_sector_peek_guide_front",
    "active_sector_peek_guide_rear",
    "active_sector_aluminum_yoke",
}
SUPPLIER_AUTHORITY_PENDING_IDS: set[str] = set()


def _write_rfq_csv(
    parts: dict[str, ManufacturingPart], exported_ids: set[str]
) -> None:
    with RFQ_CSV.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=(
            "part_id", "group", "quantity", "material", "process",
            "geometry_authority", "step_file", "drawing_file",
            "candidate_purchase_status", "purchase_status",
            "order_authorized", "cost_status", "material_design_basis",
            "supplier_return_requirements", "receiving_inspection",
            "qualification_before_use", "release_blocker",
        ))
        writer.writeheader()
        for part_id, row in parts.items():
            writer.writerow({
                "part_id": part_id,
                "group": row.group,
                "quantity": row.quantity,
                "material": row.material,
                "process": row.process,
                "geometry_authority": row.geometry_authority,
                "step_file": (
                    f"out/custom/successor/step/{part_id}.step"
                    if part_id in exported_ids
                    else (
                        "PENDING_TOOTH_COMPLETE_SUPPLIER_CAD"
                        if part_id in SUPPLIER_AUTHORITY_PENDING_IDS
                        else "PENDING_REV2_PACKAGING"
                    )
                ),
                "drawing_file": "output/pdf/successor_custom_parts_rfq.pdf",
                "candidate_purchase_status": (
                    "RFQ_READY"
                    if part_id in exported_ids
                    else (
                        "BLOCKED_PendingSupplierAuthority"
                        if part_id in SUPPLIER_AUTHORITY_PENDING_IDS
                        else "BLOCKED_PendingActiveSectorPackaging"
                    )
                ),
                "purchase_status": (
                    "RFQ_READY" if part_id in exported_ids else "BLOCKED"
                ),
                "order_authorized": "FALSE",
                "cost_status": "TBD",
                "material_design_basis": row.rfq.material_design_basis,
                "supplier_return_requirements": "; ".join(
                    row.rfq.quote_requirements
                ),
                "receiving_inspection": "; ".join(
                    row.rfq.receiving_inspection
                ),
                "qualification_before_use": "; ".join(
                    row.rfq.qualification_before_use
                ),
                "release_blocker": (
                    "RFQ submission only: supplier quote/returned DFM, receiving "
                    "inspection and all physical qualification remain open; no "
                    "order or production use is authorized"
                ),
            })


def generate(*, include_active_sector: bool = False) -> dict:
    STEP_OUT.mkdir(parents=True, exist_ok=True)
    if OUT.resolve().parents[1] != (ROOT / "out").resolve():
        raise RuntimeError(f"refusing unexpected successor output path {OUT}")
    for old in STEP_OUT.glob("*.step"):
        old.unlink()

    parts = manufacturing_parts()
    exported = {
        part_id: row for part_id, row in parts.items()
        if include_active_sector or part_id not in ACTIVE_SECTOR_PENDING_IDS
        if part_id not in SUPPLIER_AUTHORITY_PENDING_IDS
    }
    records = []
    for part_id, row in exported.items():
        solids = list(row.part.solids())
        if len(solids) != 1 or not row.part.is_valid or row.part.volume <= 0.0:
            raise RuntimeError(f"{part_id} is not one valid positive-volume solid")
        path = STEP_OUT / f"{part_id}.step"
        export_step(row.part, str(path))
        box = row.part.bounding_box()
        records.append({
            "id": part_id,
            "group": row.group,
            "quantity": row.quantity,
            "material": row.material,
            "process": row.process,
            "geometry_authority": row.geometry_authority,
            "file": path.relative_to(ROOT).as_posix(),
            "sha256": _sha256(path),
            "bytes": path.stat().st_size,
            "single_solid": True,
            "bbox_mm": [
                round(float(box.size.X), 6),
                round(float(box.size.Y), 6),
                round(float(box.size.Z), 6),
            ],
            "candidate_purchase_status": "rfq_ready",
            "purchase_status": "rfq_ready",
            "authorization_status": "blocked",
            "rfq_submission_authorized": True,
            "cost_status": "tbd",
            "rfq_contract": {
                "material_design_basis": row.rfq.material_design_basis,
                "quote_requirements": list(row.rfq.quote_requirements),
                "receiving_inspection": list(row.rfq.receiving_inspection),
                "qualification_before_use": list(
                    row.rfq.qualification_before_use
                ),
            },
        })

    _write_rfq_csv(parts, set(exported))
    solution = candidate.integrated_balance_solution()
    pending_rev2 = sorted(
        set(parts) & ACTIVE_SECTOR_PENDING_IDS - set(exported)
    )
    pending_supplier = sorted(set(parts) & SUPPLIER_AUTHORITY_PENDING_IDS)
    packet_complete = not pending_rev2 and not pending_supplier
    manifest = {
        "schema": "successor-manufacturing-packet/v1",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "production_authorized": False,
        "order_authorized": False,
        "rfq_submission_authorized": packet_complete,
        "drawing": DRAWING.relative_to(ROOT).as_posix(),
        "rfq_csv": RFQ_CSV.relative_to(ROOT).as_posix(),
        "artifacts": {
            "rfq_csv": _artifact_record(RFQ_CSV),
            "drawing_pdf": _artifact_record(DRAWING),
        },
        "parts": records,
        "pending_rev2_packaging_parts": pending_rev2,
        "pending_supplier_authority_parts": pending_supplier,
        "rfq_readiness": {
            "rfq_ready_part_ids": sorted(exported),
            "order_ready_part_ids": [],
            "production_authorized_part_ids": [],
            "meaning": (
                "RFQ_READY authorizes sending the hash-bound STEP/drawing/CSV "
                "packet for supplier quotation only. It does not authorize an "
                "order, installation or production use."
            ),
        },
        "balance_contract": {
            "rear_slug_lengths_mm": list(map(float, solution["rear_slug_lengths_mm"])),
            "front_trim_common_thickness_mm": float(
                solution["front_trim_common_thickness_mm"]
            ),
            "authority": solution["authority"],
        },
        "source_hashes": {
            relative: _source_sha256(path)
            for relative, path in _expected_source_paths().items()
        },
        "blockers": [
            "Supplier-returned PEEK grade/lot, DFM, finish and dimensional-inspection commitments are absent.",
            "Supplier-returned ASTM B777 class/18.49 g/cm3 density certificate, serialized weights and machining tolerances are absent.",
            "Prices, receiving inspection and every physical qualification gate remain open; RFQ readiness is not order authorization.",
        ],
    }
    validate_manifest(manifest)
    MANIFEST.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--include-active-sector",
        action="store_true",
        help="export the selected rev6 active guides and outboard yoke",
    )
    args = parser.parse_args(argv)
    manifest = generate(include_active_sector=args.include_active_sector)
    print(f"successor custom parts: {len(manifest['parts'])}")
    if manifest["pending_rev2_packaging_parts"]:
        print("pending active-sector packaging:", ", ".join(
            manifest["pending_rev2_packaging_parts"]
        ))
    print(MANIFEST)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
