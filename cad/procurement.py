"""Generate and gate the order/print handoff from machine-readable sources.

This reconciles the human-facing BOM, canonical release catalog, exact
hardware schedule, placed CAD occurrences, and generated print files.  It
deliberately returns a non-zero exit status while a supplier line,
manufacturing artifact, fastener interface, print profile, or release file is
unresolved.  ``--stage`` never regenerates evidence: it copies only an
already-green allowlist into an atomically named, hashed release directory.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
from pathlib import Path

import buildability
import hardware
import hardware_placements
import release_catalog


ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "out"
ORDER = OUT / "order"
REPORT = OUT / "reports" / "procurement.json"
CATALOG_PROCUREMENT_REPORT = (
    OUT / "reports" / "release_catalog_procurement.json"
)
BOM = ROOT / "bom.csv"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _bom_rows() -> list[dict[str, str]]:
    with BOM.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def _bom_pending(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    pattern = re.compile(
        r"(?:exact (?:part|pn|length).{0,12}pending|selection pending|"
        r"pending final|spring-tbd)", re.IGNORECASE
    )
    pending = []
    for row in rows:
        text = " ".join(str(value or "") for value in row.values())
        if pattern.search(text):
            pending.append({
                "item": row.get("item", ""),
                "reason": "BOM still contains an unresolved selection marker",
            })
    return pending


def _print_manifest() -> tuple[list[dict], list[str]]:
    expected = [name for name, *_ in buildability.PARTS]
    stl_dir = OUT / "stl"
    rows = []
    missing = []
    for name in expected:
        path = stl_dir / f"{name}.stl"
        if not path.is_file() or path.stat().st_size == 0:
            missing.append(name)
            continue
        rows.append({
            "part": name,
            "file": str(path.relative_to(ROOT)).replace("\\", "/"),
            "bytes": path.stat().st_size,
            "sha256": _sha256(path),
        })
    return rows, missing


def audit() -> dict:
    bom = _bom_rows()
    schedule = hardware.procurement_schedule()
    schedule_pending = [
        {"sku": row["sku"], "status": row["status"]}
        for row in schedule if row["status"] != "selected"
    ]
    unmodeled = hardware_placements.unmodeled_schedule_items()
    placement = [issue.__dict__ for issue in hardware_placements.placement_issues()]
    prints, missing_prints = _print_manifest()
    catalog = release_catalog.audit(ROOT, hardware_order=schedule)
    normalized_hardware = {
        row["id"].removeprefix("hardware:"): row
        for row in catalog["hardware"]
    }
    enriched_schedule = []
    for row in schedule:
        normalized = normalized_hardware.get(row["sku"], {})
        quantity = normalized.get("quantity") or {}
        enriched_schedule.append({
            **row,
            "required_qty": quantity.get("required_qty", row.get("required_qty")),
            "spare_qty": quantity.get("spare_qty", row.get("spare_qty")),
            "pack_qty": quantity.get("pack_qty", 1),
            "packages_to_order": quantity.get("packages_to_order", 0),
            "order_qty": quantity.get("order_qty", row.get("order_qty")),
            "design_status": normalized.get("design_status", "pending"),
            "purchase_status": normalized.get("purchase_status", "blocked"),
            "selection": normalized.get("selection"),
            "checkout_condition": normalized.get("checkout_condition", ""),
            "mapping_blocker": normalized.get("mapping_blocker"),
            "note": normalized.get("note", ""),
        })

    blockers = []
    blockers.extend({"kind": "hardware_selection", **item}
                    for item in schedule_pending)
    blockers.extend({"kind": "unmodeled_schedule", "id": key, "qty": value}
                    for key, value in sorted(unmodeled.items()) if value)
    blockers.extend({"kind": "placement", **item} for item in placement)
    blockers.extend({"kind": "bom_selection", **item}
                    for item in _bom_pending(bom))
    blockers.extend({"kind": "missing_stl", "part": name}
                    for name in missing_prints)
    blockers.extend({"kind": "release_catalog", **item}
                    for item in catalog["blockers"])

    expected_prints = {name for name, *_ in buildability.PARTS}
    catalog_prints = {
        row["part"] for row in catalog.get("print_plan", {}).get("items", [])
    }
    if catalog_prints != expected_prints:
        blockers.append({
            "kind": "print_plan_mismatch",
            "missing": sorted(expected_prints - catalog_prints),
            "extra": sorted(catalog_prints - expected_prints),
        })

    totals = [row for row in bom if (row.get("item") or "").strip() == ""]
    total_text = next((row.get("ext_usd", "") for row in totals
                       if (row.get("unit_usd") or "").strip() == "TOTAL:"), "")
    return {
        "ready_to_order_and_print": not blockers,
        "bom": {
            "path": str(BOM.relative_to(ROOT)),
            "sha256": _sha256(BOM),
            "declared_total_usd": float(total_text) if total_text else None,
            "line_items": sum(1 for row in bom if row.get("item")),
        },
        "hardware_order": enriched_schedule,
        "release_catalog": catalog,
        "hardware_occurrences": {
            link: len(items) for link, items in
            hardware_placements.hardware_occurrences_by_link().items()
        },
        "print_manifest": prints,
        "blockers": blockers,
    }


def _full_order_rows(catalog: dict) -> list[dict]:
    """Return one fulfillment row per physical purchasing obligation.

    Hardware schedule rows remain independently visible in
    ``hardware_order.csv``.  When a hardware SKU maps to a catalog item, that
    catalog item already owns the RFQ/cart quantity and may aggregate several
    hardware SKUs.  Emitting both rows would double-order controlled parts.
    """

    items = list(catalog.get("items") or [])
    item_ids = {str(row.get("id") or "") for row in items}
    hardware = []
    for row in catalog.get("hardware") or []:
        catalog_item_id = row.get("fulfilled_by_catalog_item")
        if catalog_item_id is None:
            catalog_item_id = row.get("catalog_item_id")
        if catalog_item_id:
            if str(catalog_item_id) not in item_ids:
                raise ValueError(
                    "hardware fulfillment mapping references absent catalog "
                    f"item {catalog_item_id!r}"
                )
            continue
        hardware.append(row)
    return [*items, *hardware]


def write_outputs(result: dict) -> None:
    ORDER.mkdir(parents=True, exist_ok=True)
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    # Keep the source-backed purchasing decisions small enough to review
    # without mining the complete procurement report.  Only catalog rows with
    # explicit evidence or an interface blocker are included; the canonical
    # readiness verdict remains the full report above.
    evidence_rows = []
    for row in result["release_catalog"]["items"]:
        if (not row.get("evidence") and not row.get("blocker")
                and not row.get("commissioning_gate")
                and not row.get("receiving_contract")):
            continue
        evidence_rows.append({
            key: row.get(key)
            for key in (
                "id", "class", "scope", "category", "description",
                "design_status", "purchase_status",
                "candidate_purchase_status", "authorization_status",
                "quantity", "selection", "manufacturing",
                "checkout_condition",
                "evidence", "blocker", "commissioning_gate",
                "receiving_contract", "note",
            )
        })
    CATALOG_PROCUREMENT_REPORT.write_text(
        json.dumps({
            "schema": 1,
            "catalog_id": result["release_catalog"].get("catalog_id"),
            "source": "cad/release_catalog.json",
            "ready": result["release_catalog"].get("ready"),
            "items": evidence_rows,
        }, indent=2) + "\n",
        encoding="utf-8",
    )

    normalized_hardware = {
        row["id"].removeprefix("hardware:"): row
        for row in result["release_catalog"]["hardware"]
    }
    fields = ["sku", "description", "standard", "required_qty",
              "spare_qty", "pack_qty", "packages_to_order", "order_qty",
              "status", "design_status",
              "purchase_status", "supplier", "supplier_sku",
              "checkout_condition",
              "mapping_blocker",
              "schedule_ids"]
    with (ORDER / "hardware_order.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in result["hardware_order"]:
            output = {key: row.get(key, "") for key in fields}
            normalized = normalized_hardware.get(row["sku"], {})
            selection = normalized.get("selection") or {}
            output["design_status"] = normalized.get("design_status", "")
            output["purchase_status"] = normalized.get("purchase_status", "")
            output["supplier"] = selection.get("supplier", "")
            output["supplier_sku"] = selection.get("supplier_sku", "")
            output["checkout_condition"] = normalized.get(
                "checkout_condition", ""
            )
            output["mapping_blocker"] = json.dumps(
                normalized.get("mapping_blocker") or "",
                sort_keys=True,
            )
            output["schedule_ids"] = ";".join(row.get("schedule_ids", []))
            writer.writerow(output)

    with (ORDER / "print_manifest.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream,
                                fieldnames=["part", "file", "bytes", "sha256"])
        writer.writeheader()
        writer.writerows(result["print_manifest"])

    full_fields = [
        "id", "class", "scope", "category", "description",
        "design_status", "purchase_status", "authorization_status",
        "rfq_submission_status", "order_authorized",
        "production_authorized", "required_qty", "spare_qty",
        "pack_qty", "packages_to_order", "order_qty", "uom", "supplier",
        "supplier_sku", "receiving_contract_json",
        "checkout_condition",
    ]
    with (ORDER / "full_order.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=full_fields)
        writer.writeheader()
        rows = _full_order_rows(result["release_catalog"])
        for row in rows:
            quantity = row.get("quantity") or {}
            selection = row.get("selection") or {}
            manufacturing = row.get("manufacturing") or {}
            rfq_contract = manufacturing.get("rfq_contract") or {}
            writer.writerow({
                "id": row.get("id", ""),
                "class": row.get("class", ""),
                "scope": row.get("scope", ""),
                "category": row.get("category", ""),
                "description": row.get("description", ""),
                "design_status": row.get("design_status", ""),
                "purchase_status": row.get("purchase_status", ""),
                "authorization_status": row.get("authorization_status", ""),
                "rfq_submission_status": rfq_contract.get(
                    "rfq_submission_status", ""
                ),
                "order_authorized": rfq_contract.get(
                    "order_authorized", ""
                ),
                "production_authorized": rfq_contract.get(
                    "production_authorized", ""
                ),
                **{key: quantity.get(key, "") for key in (
                    "required_qty", "spare_qty", "pack_qty",
                    "packages_to_order", "order_qty", "uom",
                )},
                "supplier": selection.get("supplier", ""),
                "supplier_sku": selection.get("supplier_sku", ""),
                "checkout_condition": row.get("checkout_condition", ""),
                "receiving_contract_json": json.dumps(
                    row.get("receiving_contract") or {}, sort_keys=True,
                    separators=(",", ":"),
                ),
            })

    profile = result["release_catalog"].get("print_plan", {}).get("profile") or {}
    print_fields = [
        "part", "quantity", "revision", "printer", "material", "nozzle_mm",
        "layer_height_mm", "walls", "top_layers", "bottom_layers",
        "infill_percent", "infill_pattern", "overrides_json",
    ]
    with (ORDER / "print_jobs.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=print_fields)
        writer.writeheader()
        for row in result["release_catalog"].get("print_plan", {}).get("items", []):
            writer.writerow({
                "part": row.get("part", ""),
                "quantity": row.get("quantity", ""),
                "revision": row.get("revision", ""),
                **{key: profile.get(key, "") for key in (
                    "printer", "material", "nozzle_mm", "layer_height_mm",
                    "walls", "top_layers", "bottom_layers", "infill_percent",
                    "infill_pattern",
                )},
                "overrides_json": json.dumps(row.get("overrides") or {},
                                             sort_keys=True),
            })


def _stage_gate_blockers() -> list[dict]:
    """Read-only final gates used only by ``--stage``.

    Procurement is itself an input to supplemental release readiness, so the
    normal procurement report cannot depend on that downstream verdict.  The
    staging command closes the loop without rewriting either report.
    """
    blockers: list[dict] = []
    validation_path = OUT / "reports" / "validation.json"
    try:
        validation = json.loads(validation_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        blockers.append({
            "kind": "validation_gate",
            "path": str(validation_path.relative_to(ROOT)).replace("\\", "/"),
            "detail": f"cannot read machine-readable DoD verdict: {exc}",
        })
    else:
        if not isinstance(validation, dict) or validation.get("passed") is not True:
            blockers.append({
                "kind": "validation_gate",
                "path": str(validation_path.relative_to(ROOT)).replace("\\", "/"),
                "detail": f"validation passed={validation.get('passed')!r}",
            })

    # Lazy import avoids a module cycle and, importantly, evaluates freshness
    # against the current procurement report rather than trusting an old JSON
    # boolean copied into the release allowlist.
    import release_readiness
    supplemental = release_readiness.evaluate(ROOT)
    if supplemental.get("passed") is not True:
        blockers.append({
            "kind": "supplemental_release_gate",
            "detail": (
                f"{supplemental.get('passed_gate_count')}/"
                f"{supplemental.get('required_gate_count')} required gates pass"
            ),
        })
    return blockers


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", action="store_true",
                        help="atomically stage the existing green handoff")
    parser.add_argument("--verify", metavar="RELEASE_DIR",
                        help="verify a previously staged release directory")
    args = parser.parse_args(argv)
    if args.verify:
        verification = release_catalog.verify_stage(args.verify)
        print(json.dumps(verification, indent=2))
        return 0 if verification["passed"] else 1

    result = audit()
    if not args.stage:
        write_outputs(result)
    print(f"hardware SKUs: {len(result['hardware_order'])}")
    print(f"placed occurrences: {sum(result['hardware_occurrences'].values())}")
    print(f"print files: {len(result['print_manifest'])}")
    print("release handoff:",
          "PASS" if result["ready_to_order_and_print"] else "BLOCKED")
    for blocker in result["blockers"]:
        print(" -", json.dumps(blocker, sort_keys=True))
    stage_blockers = _stage_gate_blockers() if args.stage else []
    for blocker in stage_blockers:
        print(" -", json.dumps(blocker, sort_keys=True))
    if args.stage and result["ready_to_order_and_print"] and not stage_blockers:
        stage_report = dict(result["release_catalog"])
        stage_report["ready"] = True
        stage_report["blockers"] = []
        release_path = release_catalog.stage_release(stage_report, ROOT)
        print("staged release:", release_path)
    return 0 if result["ready_to_order_and_print"] and not stage_blockers else 1


if __name__ == "__main__":
    raise SystemExit(main())
