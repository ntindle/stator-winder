"""Canonical order catalog and deterministic release-bundle staging.

The assembly hardware schedule answers a design question: which standard
parts and how many occurrences are present in CAD.  It does *not* prove that
there is an orderable supplier line.  This module deliberately keeps those
two states separate and fails closed until every required line has an exact
purchasing route or a complete manufacturing handoff.

``release_catalog.json`` is the human-editable authority for non-fastener
items, purchasing mappings for the hardware schedule, print quantities, and
the exact release allowlist.  Generated reports embed hashes; release staging
copies only allowlisted files into a new temporary directory and atomically
renames it after a complete integrity check.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import uuid
from typing import Any, Iterable, Mapping, Sequence


ROOT = Path(__file__).resolve().parent.parent
CATALOG_PATH = Path(__file__).with_suffix(".json")
SCHEMA = 1

ITEM_CLASSES = {
    "cots", "fastener", "raw_stock", "custom", "sendcutsend", "consumable",
    "electronics", "tooling",
}
SCOPES = {"machine", "job", "optional", "excluded"}
DESIGN_STATES = {"selected", "pending", "not_applicable"}
PURCHASE_STATES = {
    "cart_ready", "rfq_ready", "upload_ready", "print_ready", "blocked",
    "excluded",
}
AUTHORIZATION_STATES = {"order_ready", "conditional", "blocked", "excluded"}
COST_STATES = {"known_current", "planning_allowance", "tbd", "excluded"}
READY_PURCHASE_STATES = {
    "cart_ready", "rfq_ready", "upload_ready", "print_ready", "excluded",
}
READY_BY_CLASS = {
    "cots": {"cart_ready"},
    "fastener": {"cart_ready"},
    "raw_stock": {"cart_ready"},
    "custom": {"rfq_ready"},
    "sendcutsend": {"upload_ready"},
    "consumable": {"cart_ready"},
    "electronics": {"cart_ready"},
    "tooling": {"cart_ready", "rfq_ready", "print_ready"},
}


class CatalogError(ValueError):
    """The catalog or a staged release is malformed."""


class ReleaseBlocked(RuntimeError):
    """A release stage was requested while fail-closed blockers remain."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"),
                       ensure_ascii=True) + "\n").encode("utf-8")


def _safe_path(root: Path, relative: Any) -> Path | None:
    if not isinstance(relative, str) or not relative.strip():
        return None
    path = Path(relative)
    if path.is_absolute():
        return None
    try:
        resolved = (root / path).resolve(strict=False)
        resolved.relative_to(root.resolve())
    except (OSError, ValueError):
        return None
    return resolved


def _file_record(root: Path, relative: Any) -> dict[str, Any]:
    path = _safe_path(root, relative)
    record: dict[str, Any] = {
        "path": relative if isinstance(relative, str) else None,
        "exists": False,
    }
    if path is None:
        record["error"] = "path must be a non-empty root-relative path"
        return record
    try:
        stat = path.stat()
        if not path.is_file():
            record["error"] = "not a regular file"
            return record
        record.update({
            "exists": True,
            "bytes": stat.st_size,
            "sha256": _sha256(path),
        })
    except OSError as exc:
        record["error"] = str(exc)
    return record


def load(path: str | Path = CATALOG_PATH) -> dict[str, Any]:
    source = Path(path)
    try:
        value = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CatalogError(f"cannot read release catalog {source}: {exc}") from exc
    if not isinstance(value, dict):
        raise CatalogError("release catalog root must be a JSON object")
    return value


def _block(blockers: list[dict[str, Any]], kind: str, item_id: str,
           detail: str) -> None:
    blockers.append({"kind": kind, "id": item_id, "detail": detail})


def _quantity(item: Mapping[str, Any], blockers: list[dict[str, Any]],
              item_id: str) -> dict[str, Any]:
    raw = item.get("quantity")
    if not isinstance(raw, Mapping):
        _block(blockers, "catalog_schema", item_id,
               "quantity must be an object")
        raw = {}
    values: dict[str, int] = {}
    for key, default in (("required_qty", 0), ("spare_qty", 0),
                         ("pack_qty", 1)):
        value = raw.get(key, default)
        if type(value) is not int or value < 0 or (key == "pack_qty" and value < 1):
            _block(blockers, "catalog_schema", item_id,
                   f"quantity.{key} must be a non-negative integer"
                   if key != "pack_qty" else
                   "quantity.pack_qty must be a positive integer")
            value = default
        values[key] = value
    required = values["required_qty"] + values["spare_qty"]
    packages = math.ceil(required / values["pack_qty"]) if required else 0
    declared_packages = raw.get("packages_to_order")
    declared_order = raw.get("order_qty")
    if declared_packages is not None and declared_packages != packages:
        _block(blockers, "quantity_mismatch", item_id,
               f"packages_to_order={declared_packages!r}; computed {packages}")
    order_qty = packages * values["pack_qty"]
    if declared_order is not None and declared_order != order_qty:
        _block(blockers, "quantity_mismatch", item_id,
               f"order_qty={declared_order!r}; computed {order_qty}")
    return {
        **values,
        "packages_to_order": packages,
        "order_qty": order_qty,
        "uom": str(raw.get("uom") or "each"),
    }


def _cost(item: Mapping[str, Any], quantity: Mapping[str, Any],
          blockers: list[dict[str, Any]], item_id: str) -> dict[str, Any] | None:
    """Normalize optional price evidence without inventing a complete total.

    A current catalog price is distinct from an order authorization.  A
    conditional driver may have a verified current price while its exact
    configuration remains blocked, and an RFQ planning allowance is never
    promoted to a supplier quote.
    """

    raw = item.get("cost")
    if raw is None:
        return None
    if not isinstance(raw, Mapping):
        _block(blockers, "catalog_schema", item_id, "cost must be an object")
        return None
    status = raw.get("status")
    if status not in COST_STATES:
        _block(
            blockers, "catalog_schema", item_id,
            f"invalid cost.status {status!r}",
        )
        status = "tbd"
    unit = raw.get("unit_usd")
    priced = status in {"known_current", "planning_allowance"}
    if priced and (isinstance(unit, bool) or not isinstance(unit, (int, float))
                   or not math.isfinite(float(unit)) or float(unit) < 0.0):
        _block(
            blockers, "catalog_schema", item_id,
            f"cost.unit_usd must be a finite non-negative number for {status}",
        )
        unit = None
    if not priced and unit is not None:
        _block(
            blockers, "catalog_schema", item_id,
            f"cost.unit_usd must be omitted when cost.status is {status}",
        )
        unit = None
    extended = (
        round(float(unit) * int(quantity["order_qty"]), 2)
        if unit is not None else None
    )
    declared = raw.get("extended_usd")
    if declared is not None and (
        extended is None
        or isinstance(declared, bool)
        or not isinstance(declared, (int, float))
        or not math.isclose(float(declared), extended, abs_tol=0.005)
    ):
        _block(
            blockers, "cost_mismatch", item_id,
            f"cost.extended_usd={declared!r}; computed {extended!r}",
        )
    return {
        "status": status,
        "unit_usd": float(unit) if unit is not None else None,
        "extended_usd": extended,
        "basis": str(raw.get("basis") or ""),
        "verified_on": raw.get("verified_on"),
    }


def _selection_ready(item: Mapping[str, Any], blockers: list[dict[str, Any]],
                     item_id: str) -> None:
    selection = item.get("selection")
    if not isinstance(selection, Mapping):
        _block(blockers, "supplier_selection", item_id,
               "cart-ready line has no selection object")
        return
    missing = [key for key in (
        "manufacturer", "mpn", "supplier", "supplier_sku", "url",
    ) if not isinstance(selection.get(key), str) or not selection.get(key).strip()]
    if missing:
        _block(blockers, "supplier_selection", item_id,
               "cart-ready line is missing " + ", ".join(missing))


def _artifact_rows(root: Path, owner_id: str, raw: Any,
                   blockers: list[dict[str, Any]], *,
                   require_nonempty: bool = False) -> list[dict[str, Any]]:
    if not isinstance(raw, list):
        if require_nonempty:
            _block(blockers, "manufacturing_artifact", owner_id,
                   "required artifacts must be a non-empty list")
        return []
    if require_nonempty and not raw:
        _block(blockers, "manufacturing_artifact", owner_id,
               "required artifacts must be a non-empty list")
    result: list[dict[str, Any]] = []
    seen: set[str] = set()
    for index, entry in enumerate(raw):
        if not isinstance(entry, Mapping):
            _block(blockers, "catalog_schema", owner_id,
                   f"artifact {index} must be an object")
            continue
        artifact_id = entry.get("id")
        if not isinstance(artifact_id, str) or not artifact_id:
            _block(blockers, "catalog_schema", owner_id,
                   f"artifact {index} has no stable id")
            continue
        if artifact_id in seen:
            _block(blockers, "catalog_schema", owner_id,
                   f"duplicate artifact id {artifact_id}")
            continue
        seen.add(artifact_id)
        required = entry.get("required", True) is not False
        generated = entry.get("generated", False) is True
        record = _file_record(root, entry.get("path"))
        json_requirements = entry.get("json_requirements")
        output = {
            "id": artifact_id,
            "role": str(entry.get("role") or "unspecified"),
            "required": required,
            "generated": generated,
            **record,
        }
        if json_requirements is not None:
            if not isinstance(json_requirements, Mapping) or not json_requirements:
                _block(blockers, "catalog_schema", owner_id,
                       f"{artifact_id}.json_requirements must be a non-empty object")
            else:
                output["json_requirements"] = deepcopy(
                    dict(json_requirements))
            if isinstance(json_requirements, Mapping) and json_requirements and record["exists"]:
                path = root / str(record["path"])
                try:
                    payload = json.loads(path.read_text(encoding="utf-8"))
                except (OSError, UnicodeError, json.JSONDecodeError) as exc:
                    _block(blockers, "evidence_verdict", owner_id,
                           f"{artifact_id}: cannot read JSON verdict: {exc}")
                else:
                    if not isinstance(payload, Mapping):
                        _block(blockers, "evidence_verdict", owner_id,
                               f"{artifact_id}: JSON root must be an object")
                    else:
                        observed = {
                            str(key): payload.get(key)
                            for key in json_requirements
                        }
                        output["json_observed"] = observed
                        mismatches = [
                            f"{key}={observed[str(key)]!r} (expected {value!r})"
                            for key, value in json_requirements.items()
                            if observed[str(key)] != value
                        ]
                        if required and mismatches:
                            _block(blockers, "evidence_verdict", owner_id,
                                   f"{artifact_id}: " + "; ".join(mismatches))
        result.append(output)
        if required and not generated and not record["exists"]:
            _block(blockers, "missing_artifact", owner_id,
                   f"{artifact_id}: {record.get('error') or 'file missing'} "
                   f"({record.get('path')})")
    return result


def _normalize_item(root: Path, item: Any, index: int,
                    blockers: list[dict[str, Any]]) -> dict[str, Any] | None:
    if not isinstance(item, Mapping):
        _block(blockers, "catalog_schema", str(index),
               "item must be an object")
        return None
    item_id = item.get("id")
    if not isinstance(item_id, str) or not item_id.strip():
        _block(blockers, "catalog_schema", str(index),
               "item has no stable id")
        return None
    item_class = item.get("class")
    scope = item.get("scope")
    design = item.get("design_status")
    purchase = item.get("purchase_status")
    if item_class not in ITEM_CLASSES:
        _block(blockers, "catalog_schema", item_id,
               f"invalid class {item_class!r}")
    if scope not in SCOPES:
        _block(blockers, "catalog_schema", item_id,
               f"invalid scope {scope!r}")
    if design not in DESIGN_STATES:
        _block(blockers, "catalog_schema", item_id,
               f"invalid design_status {design!r}")
    if purchase not in PURCHASE_STATES:
        _block(blockers, "catalog_schema", item_id,
               f"invalid purchase_status {purchase!r}")
    quantity = _quantity(item, blockers, item_id)
    required = scope not in {"optional", "excluded"} and quantity["required_qty"] > 0
    authorization = item.get("authorization_status")
    if authorization is None:
        if scope in {"optional", "excluded"}:
            authorization = "excluded"
        elif design == "selected" and purchase in READY_PURCHASE_STATES:
            authorization = "order_ready"
        else:
            authorization = "blocked"
    if authorization not in AUTHORIZATION_STATES:
        _block(
            blockers, "catalog_schema", item_id,
            f"invalid authorization_status {authorization!r}",
        )
        authorization = "blocked"
    candidate_purchase = item.get("candidate_purchase_status")
    if candidate_purchase is not None and candidate_purchase not in PURCHASE_STATES:
        _block(
            blockers, "catalog_schema", item_id,
            f"invalid candidate_purchase_status {candidate_purchase!r}",
        )
        candidate_purchase = None
    if required and design != "selected":
        _block(blockers, "design_selection", item_id,
               f"required line design_status is {design!r}")
    allowed = READY_BY_CLASS.get(str(item_class), set())
    if required and purchase not in allowed:
        _block(blockers, "purchase_selection", item_id,
               f"required {item_class} line is {purchase!r}; expected one of "
               f"{sorted(allowed)}")
    if purchase == "cart_ready":
        _selection_ready(item, blockers, item_id)
    if required and authorization != "order_ready":
        _block(
            blockers, "production_authorization", item_id,
            f"required line authorization_status is {authorization!r}",
        )
    cost = _cost(item, quantity, blockers, item_id)

    manufacturing = item.get("manufacturing")
    manufacture_artifacts: list[dict[str, Any]] = []
    if purchase in {"rfq_ready", "upload_ready", "print_ready"}:
        if not isinstance(manufacturing, Mapping):
            _block(blockers, "manufacturing_contract", item_id,
                   f"{purchase} line has no manufacturing object")
        else:
            for key in ("process", "revision"):
                if not isinstance(manufacturing.get(key), str) or not manufacturing.get(key):
                    _block(blockers, "manufacturing_contract", item_id,
                           f"manufacturing.{key} is required for {purchase}")
            manufacture_artifacts = _artifact_rows(
                root, item_id, manufacturing.get("artifacts"), blockers,
                require_nonempty=True,
            )
    else:
        manufacture_artifacts = _artifact_rows(
            root, item_id,
            manufacturing.get("artifacts") if isinstance(manufacturing, Mapping) else [],
            blockers,
        )

    model = item.get("model")
    normalized_model: dict[str, Any] | None = None
    if isinstance(model, Mapping):
        normalized_model = deepcopy(dict(model))
        if "path" in model:
            model_record = _file_record(root, model.get("path"))
            normalized_model["artifact"] = {
                "id": f"model:{item_id}",
                "role": "cad_model",
                "required": True,
                "generated": False,
                **model_record,
            }
            if not model_record["exists"]:
                _block(blockers, "missing_model", item_id,
                       f"CAD model is missing: {model_record.get('path')}")

    return {
        "id": item_id,
        "class": item_class,
        "scope": scope,
        "category": str(item.get("category") or item_class or "unknown"),
        "description": str(item.get("description") or ""),
        "design_status": design,
        "purchase_status": purchase,
        "candidate_purchase_status": candidate_purchase,
        "authorization_status": authorization,
        "quantity": quantity,
        "cost": cost,
        "selection": deepcopy(item.get("selection")),
        "checkout_condition": str(item.get("checkout_condition") or ""),
        "model": normalized_model,
        "manufacturing": {
            **(deepcopy(dict(manufacturing)) if isinstance(manufacturing, Mapping) else {}),
            "artifacts": manufacture_artifacts,
        } if manufacturing is not None else None,
        # Procurement evidence is intentionally carried into the generated
        # report and staged manifest.  It is descriptive source material, not
        # a substitute for the state machine above: an item with excellent
        # candidate evidence remains blocked until the exact interface is
        # proved and purchase_status is explicitly advanced.
        "evidence": deepcopy(item.get("evidence") or []),
        "blocker": deepcopy(item.get("blocker")),
        "commissioning_gate": deepcopy(item.get("commissioning_gate")),
        "receiving_contract": deepcopy(item.get("receiving_contract")),
        "note": str(item.get("note") or ""),
    }


def _cost_summary(items: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    order_ready_known = 0.0
    conditional_known = 0.0
    planning_allowances = 0.0
    tbd_required: list[str] = []
    unannotated_required: list[str] = []
    for item in items:
        quantity = item.get("quantity")
        required = (
            item.get("scope") not in {"optional", "excluded"}
            and isinstance(quantity, Mapping)
            and int(quantity.get("required_qty", 0)) > 0
        )
        cost = item.get("cost")
        if not isinstance(cost, Mapping):
            if required:
                unannotated_required.append(str(item.get("id")))
            continue
        status = cost.get("status")
        extended = cost.get("extended_usd")
        if status == "known_current" and isinstance(extended, (int, float)):
            if (
                item.get("authorization_status") == "order_ready"
                and item.get("purchase_status") in READY_PURCHASE_STATES
            ):
                order_ready_known += float(extended)
            else:
                conditional_known += float(extended)
        elif status == "planning_allowance" and isinstance(extended, (int, float)):
            planning_allowances += float(extended)
        elif status == "tbd" and required:
            tbd_required.append(str(item.get("id")))
    return {
        "coverage": "partial_catalog_annotations",
        "complete_machine_total_available": False,
        "annotated_order_ready_known_current_usd": round(order_ready_known, 2),
        "annotated_conditional_known_current_usd": round(conditional_known, 2),
        "annotated_planning_allowance_usd": round(planning_allowances, 2),
        "tbd_required_item_ids": sorted(tbd_required),
        "unannotated_required_item_ids": sorted(unannotated_required),
        "note": (
            "These subtotals cover only items with explicit cost metadata; "
            "they are not a complete machine total or an order authorization."
        ),
    }


def _hardware_rows(catalog: Mapping[str, Any], hardware_order: Sequence[Mapping[str, Any]],
                   blockers: list[dict[str, Any]],
                   catalog_item_states: Mapping[str, Any]) -> list[dict[str, Any]]:
    mappings = catalog.get("hardware_purchase_map")
    if not isinstance(mappings, Mapping):
        mappings = {}
        _block(blockers, "catalog_schema", "hardware_purchase_map",
               "hardware_purchase_map must be an object")
    result: list[dict[str, Any]] = []
    for raw in hardware_order:
        sku = str(raw.get("sku") or "")
        mapping = mappings.get(sku)
        design = "selected" if raw.get("status") == "selected" else "pending"
        item_class = (mapping.get("class", "fastener")
                      if isinstance(mapping, Mapping) else "fastener")
        purchase = (mapping.get("purchase_status")
                    if isinstance(mapping, Mapping) else "blocked")
        selection = (deepcopy(mapping.get("selection"))
                     if isinstance(mapping, Mapping) else None)
        catalog_item_id = (mapping.get("catalog_item_id")
                           if isinstance(mapping, Mapping) else None)
        if design != "selected":
            _block(blockers, "hardware_design", sku,
                   f"hardware schedule status is {raw.get('status')!r}")
        allowed = READY_BY_CLASS.get(str(item_class), set())
        if purchase not in allowed:
            _block(blockers, "hardware_purchase", sku,
                   f"no ready purchasing mapping; {purchase!r} is not one of "
                   f"{sorted(allowed)}")
        elif purchase == "cart_ready":
            _selection_ready({"selection": selection}, blockers, sku)
        elif (
            not isinstance(catalog_item_id, str)
            or catalog_item_states.get(catalog_item_id) != purchase
        ):
            _block(blockers, "hardware_purchase", sku,
                   f"{purchase} mapping must reference a catalog item in the "
                   f"same state; got {catalog_item_id!r}")
        required = int(raw.get("required_qty", 0))
        spare = int(raw.get("spare_qty", 0))
        # Hardware schedule quantities are pieces. Pack sizes must eventually
        # come from the exact purchasing mapping; default one is conservative
        # and never makes an unmapped line order-ready.
        pack = (mapping.get("pack_qty", 1)
                if isinstance(mapping, Mapping) else 1)
        if type(pack) is not int or pack < 1:
            _block(blockers, "quantity_mismatch", sku,
                   "hardware pack_qty must be a positive integer")
            pack = 1
        packages = math.ceil((required + spare) / pack) if required + spare else 0
        result.append({
            "id": f"hardware:{sku}",
            "class": item_class,
            "scope": "machine",
            "category": "hardware",
            "description": str(raw.get("description") or ""),
            "standard": str(raw.get("standard") or ""),
            "design_status": design,
            "purchase_status": purchase,
            "quantity": {
                "required_qty": required,
                "spare_qty": spare,
                "pack_qty": pack,
                "packages_to_order": packages,
                "order_qty": packages * pack,
                "uom": "each",
            },
            "selection": selection,
            "checkout_condition": (
                str(mapping.get("checkout_condition") or "")
                if isinstance(mapping, Mapping) else ""
            ),
            # Preserve supplier-table evidence and commercial caveats.  A
            # cart-ready mapping identifies the exact supplier SKU and pack;
            # it does not assert live price, fulfillment, checkout success,
            # installation approval, or motion authorization.
            "evidence": (
                deepcopy(mapping.get("evidence") or [])
                if isinstance(mapping, Mapping) else []
            ),
            "mapping_blocker": (
                deepcopy(mapping.get("blocker"))
                if isinstance(mapping, Mapping) else None
            ),
            "note": (
                str(mapping.get("note") or "")
                if isinstance(mapping, Mapping) else ""
            ),
            "catalog_item_id": catalog_item_id,
            # A mapped catalog row is the sole fulfillment authority in the
            # combined order sheet.  Keep this hardware row for schedule and
            # placement reconciliation, but never emit it as a second thing
            # to buy alongside the referenced catalog item.
            "fulfilled_by_catalog_item": catalog_item_id,
            "schedule_ids": list(raw.get("schedule_ids") or []),
        })
    return result


def _print_plan(root: Path, catalog: Mapping[str, Any],
                blockers: list[dict[str, Any]]) -> dict[str, Any]:
    plan = catalog.get("print_plan")
    if not isinstance(plan, Mapping):
        _block(blockers, "print_plan", "print_plan",
               "print_plan must be an object")
        return {"status": "blocked", "profile": None, "items": []}
    status = plan.get("purchase_status")
    profile = plan.get("profile")
    items = plan.get("items")
    qualification = plan.get("qualification")
    if status != "print_ready":
        _block(blockers, "print_plan", "print_plan",
               f"purchase_status is {status!r}; expected 'print_ready'")
    required_profile = (
        "printer", "material", "nozzle_mm", "layer_height_mm", "walls",
        "top_layers", "bottom_layers", "infill_percent", "infill_pattern",
    )
    if not isinstance(profile, Mapping):
        _block(blockers, "print_plan", "print_plan", "profile is missing")
        profile = {}
    missing_profile = [key for key in required_profile if profile.get(key) in (None, "")]
    if missing_profile:
        _block(blockers, "print_plan", "print_plan",
               "profile missing " + ", ".join(missing_profile))
    normalized_qualification: dict[str, Any] = {}
    if not isinstance(qualification, Mapping):
        _block(blockers, "print_qualification", "fit_bridge_coupon",
               "physical coupon qualification is missing")
    else:
        qualification_status = qualification.get("status")
        release_allowed = qualification.get("production_release_allowed")
        if qualification_status != "passed" or release_allowed is not True:
            _block(
                blockers,
                "print_qualification",
                "fit_bridge_coupon",
                "physical coupon has not passed; production printing remains blocked",
            )
        physical_record = qualification.get("physical_test_record")
        if qualification_status == "passed" and not isinstance(physical_record, Mapping):
            _block(blockers, "print_qualification", "fit_bridge_coupon",
                   "passed qualification requires a physical_test_record")
        raw_coupon = qualification.get("coupon")
        coupon: dict[str, Any] = {}
        if not isinstance(raw_coupon, Mapping):
            _block(blockers, "print_qualification", "fit_bridge_coupon",
                   "coupon definition is missing")
        else:
            coupon = deepcopy(dict(raw_coupon))
            coupon["artifacts"] = _artifact_rows(
                root,
                "print_qualification:fit_bridge_coupon",
                raw_coupon.get("artifacts"),
                blockers,
                require_nonempty=True,
            )
            roles = {row["role"] for row in coupon["artifacts"]
                     if row["required"]}
            missing_roles = {"step", "stl", "gcode", "evidence"} - roles
            if missing_roles:
                _block(
                    blockers,
                    "print_qualification",
                    "fit_bridge_coupon",
                    "required coupon artifacts missing roles "
                    + ", ".join(sorted(missing_roles)),
                )
        normalized_qualification = {
            "status": qualification_status,
            "production_release_allowed": release_allowed,
            "blocker": qualification.get("blocker"),
            "physical_test_record": deepcopy(physical_record),
            "coupon": coupon,
        }
    normalized: list[dict[str, Any]] = []
    if not isinstance(items, list) or not items:
        _block(blockers, "print_plan", "print_plan",
               "items must be a non-empty list")
        items = []
    seen: set[str] = set()
    for index, raw in enumerate(items):
        if not isinstance(raw, Mapping):
            _block(blockers, "print_plan", str(index), "item is not an object")
            continue
        part = raw.get("part")
        if not isinstance(part, str) or not part or part in seen:
            _block(blockers, "print_plan", str(part or index),
                   "part must be a unique non-empty name")
            continue
        seen.add(part)
        qty = raw.get("quantity")
        if type(qty) is not int or qty < 1:
            _block(blockers, "print_plan", part,
                   "quantity must be a positive integer")
        artifacts = _artifact_rows(root, f"print:{part}", raw.get("artifacts"),
                                   blockers, require_nonempty=True)
        roles = {row["role"] for row in artifacts if row["required"]}
        if not {"stl", "step"}.issubset(roles):
            _block(blockers, "print_plan", part,
                   "required artifacts must include STL and STEP")
        normalized.append({
            "part": part,
            "quantity": qty,
            "revision": raw.get("revision"),
            "overrides": deepcopy(raw.get("overrides") or {}),
            "artifacts": artifacts,
        })
    return {
        "purchase_status": status,
        "profile": deepcopy(dict(profile)),
        "qualification": normalized_qualification,
        "items": normalized,
    }


def audit(root: str | Path = ROOT, *, path: str | Path | None = None,
          hardware_order: Sequence[Mapping[str, Any]] = ()) -> dict[str, Any]:
    root = Path(root).resolve()
    source = Path(path) if path is not None else root / "cad" / "release_catalog.json"
    blockers: list[dict[str, Any]] = []
    try:
        catalog = load(source)
    except CatalogError as exc:
        return {
            "schema": SCHEMA,
            "ready": False,
            "catalog": _file_record(root, str(source)),
            "items": [], "hardware": [], "print_plan": {},
            "cost_summary": _cost_summary([]),
            "release_artifacts": [],
            "blockers": [{"kind": "catalog_read", "id": "catalog",
                          "detail": str(exc)}],
        }
    if catalog.get("schema") != SCHEMA:
        _block(blockers, "catalog_schema", "catalog",
               f"schema={catalog.get('schema')!r}; expected {SCHEMA}")
    raw_items = catalog.get("items")
    if not isinstance(raw_items, list) or not raw_items:
        _block(blockers, "catalog_schema", "items",
               "items must be a non-empty list")
        raw_items = []
    items: list[dict[str, Any]] = []
    seen: set[str] = set()
    for index, raw in enumerate(raw_items):
        normalized = _normalize_item(root, raw, index, blockers)
        if normalized is None:
            continue
        item_id = normalized["id"]
        if item_id in seen:
            _block(blockers, "catalog_schema", item_id, "duplicate item id")
        else:
            seen.add(item_id)
            items.append(normalized)
    hardware = _hardware_rows(
        catalog, hardware_order, blockers,
        {row["id"]: row["purchase_status"] for row in items},
    )
    print_plan = _print_plan(root, catalog, blockers)
    release_artifacts = _artifact_rows(
        root, "release", catalog.get("release_artifacts"), blockers,
        require_nonempty=True,
    )
    catalog_record = _file_record(root, str(source.relative_to(root))
                                  if source.is_relative_to(root) else str(source))
    return {
        "schema": SCHEMA,
        "catalog_id": catalog.get("catalog_id"),
        "ready": not blockers,
        "catalog": catalog_record,
        "items": items,
        "cost_summary": _cost_summary(items),
        "hardware": hardware,
        "print_plan": print_plan,
        "release_artifacts": release_artifacts,
        "blockers": blockers,
    }


def _all_artifacts(report: Mapping[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    rows.extend(report.get("release_artifacts") or [])
    for item in report.get("items") or []:
        model = item.get("model")
        if isinstance(model, Mapping) and isinstance(model.get("artifact"), Mapping):
            rows.append(model["artifact"])
        manufacturing = item.get("manufacturing")
        if isinstance(manufacturing, Mapping):
            rows.extend(manufacturing.get("artifacts") or [])
    plan = report.get("print_plan")
    if isinstance(plan, Mapping):
        qualification = plan.get("qualification")
        if isinstance(qualification, Mapping):
            coupon = qualification.get("coupon")
            if isinstance(coupon, Mapping):
                rows.extend(coupon.get("artifacts") or [])
        for item in plan.get("items") or []:
            rows.extend(item.get("artifacts") or [])
    unique: dict[str, dict[str, Any]] = {}
    for row in rows:
        if row.get("required") is False:
            continue
        path = row.get("path")
        if isinstance(path, str):
            unique.setdefault(path, row)
    return [unique[key] for key in sorted(unique)]


def verify_stage(path: str | Path) -> dict[str, Any]:
    stage = Path(path).resolve()
    manifest_path = stage / "manifest.json"
    errors: list[str] = []
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        return {"passed": False, "errors": [f"cannot read manifest: {exc}"]}
    payload = manifest.get("payload")
    expected_payload_hash = manifest.get("payload_sha256")
    actual_payload_hash = hashlib.sha256(_canonical_bytes(payload)).hexdigest()
    if expected_payload_hash != actual_payload_hash:
        errors.append("manifest payload SHA-256 mismatch")
    expected_files = {"manifest.json"}
    artifacts = payload.get("artifacts") if isinstance(payload, Mapping) else None
    if not isinstance(artifacts, list):
        errors.append("manifest payload artifacts must be a list")
        artifacts = []
    for row in artifacts:
        if not isinstance(row, Mapping):
            errors.append("malformed artifact record")
            continue
        staged_path = row.get("staged_path")
        if not isinstance(staged_path, str):
            errors.append("artifact staged_path missing")
            continue
        expected_files.add(staged_path)
        file_path = _safe_path(stage, staged_path)
        if file_path is None or not file_path.is_file():
            errors.append(f"missing staged artifact {staged_path}")
            continue
        if file_path.stat().st_size != row.get("bytes"):
            errors.append(f"byte-count mismatch for {staged_path}")
        if _sha256(file_path) != row.get("sha256"):
            errors.append(f"SHA-256 mismatch for {staged_path}")
    actual_files = {
        item.relative_to(stage).as_posix()
        for item in stage.rglob("*") if item.is_file()
    }
    unexpected = sorted(actual_files - expected_files)
    missing = sorted(expected_files - actual_files)
    if unexpected:
        errors.append("unmanifested files: " + ", ".join(unexpected))
    if missing:
        errors.append("manifested files missing: " + ", ".join(missing))
    return {
        "passed": not errors,
        "release_id": manifest.get("release_id"),
        "file_count": len(actual_files),
        "errors": errors,
    }


def stage_release(report: Mapping[str, Any], root: str | Path = ROOT,
                  *, destination: str | Path | None = None) -> Path:
    """Create one immutable, allowlisted release directory atomically."""
    if report.get("ready") is not True or report.get("blockers"):
        raise ReleaseBlocked(
            f"release catalog has {len(report.get('blockers') or [])} blocker(s)"
        )
    root = Path(root).resolve()
    release_parent = (Path(destination).resolve() if destination is not None
                      else root / "out" / "release")
    release_parent.mkdir(parents=True, exist_ok=True)
    temporary = release_parent.parent / f".release-tmp-{uuid.uuid4().hex}"
    temporary.mkdir(parents=False, exist_ok=False)
    try:
        staged_artifacts: list[dict[str, Any]] = []
        for row in _all_artifacts(report):
            source = _safe_path(root, row.get("path"))
            if source is None or not source.is_file():
                raise ReleaseBlocked(f"allowlisted artifact missing: {row.get('path')}")
            staged_relative = (Path("files") / Path(str(row["path"]))).as_posix()
            target = temporary / staged_relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            staged_artifacts.append({
                "id": row.get("id"),
                "role": row.get("role"),
                "source_path": row.get("path"),
                "staged_path": staged_relative,
                "bytes": target.stat().st_size,
                "sha256": _sha256(target),
            })
        payload = {
            "schema": SCHEMA,
            "catalog_id": report.get("catalog_id"),
            "catalog_sha256": (report.get("catalog") or {}).get("sha256"),
            "items": report.get("items"),
            "hardware": report.get("hardware"),
            "print_plan": report.get("print_plan"),
            "artifacts": staged_artifacts,
        }
        payload_hash = hashlib.sha256(_canonical_bytes(payload)).hexdigest()
        release_id = payload_hash[:16]
        manifest = {
            "schema": SCHEMA,
            "release_id": release_id,
            "payload_sha256": payload_hash,
            "payload": payload,
        }
        (temporary / "manifest.json").write_bytes(_canonical_bytes(manifest))
        verified = verify_stage(temporary)
        if not verified["passed"]:
            raise CatalogError("staged release failed integrity check: " +
                               "; ".join(verified["errors"]))
        final = release_parent / release_id
        if final.exists():
            existing = verify_stage(final)
            if not existing["passed"] or existing.get("release_id") != release_id:
                raise CatalogError(f"existing release directory is invalid: {final}")
            shutil.rmtree(temporary)
            return final
        os.replace(temporary, final)
        return final
    except Exception:
        if temporary.exists():
            shutil.rmtree(temporary)
        raise
