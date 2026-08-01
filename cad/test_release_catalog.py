"""Focused tests for canonical purchasing state and release staging."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parent))
import hardware  # noqa: E402
import release_catalog  # noqa: E402


def _write(path: Path, value: str = "artifact\n") -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value, encoding="utf-8")
    return path


def _passing_catalog(root: Path) -> Path:
    _write(root / "artifact.txt")
    _write(root / "out/stl/fixture.stl", "solid fixture\nendsolid\n")
    _write(root / "out/step/fixture.step", "STEP\n")
    payload = {
        "schema": 1,
        "catalog_id": "fixture-v1",
        "items": [{
            "id": "exact-bearing",
            "class": "cots",
            "scope": "machine",
            "category": "bearing",
            "description": "Exact selected bearing",
            "design_status": "selected",
            "purchase_status": "cart_ready",
            "quantity": {
                "required_qty": 2,
                "spare_qty": 1,
                "pack_qty": 2,
                "packages_to_order": 2,
                "order_qty": 4,
                "uom": "each",
            },
            "selection": {
                "manufacturer": "Example Bearing",
                "mpn": "EB-1",
                "supplier": "Example Supplier",
                "supplier_sku": "SKU-1",
                "url": "https://example.invalid/SKU-1",
            },
        }],
        "hardware_purchase_map": {},
        "print_plan": {
            "purchase_status": "print_ready",
            "profile": {
                "printer": "fixture-printer",
                "material": "PETG",
                "nozzle_mm": 0.4,
                "layer_height_mm": 0.2,
                "walls": 5,
                "top_layers": 6,
                "bottom_layers": 6,
                "infill_percent": 40,
                "infill_pattern": "gyroid",
            },
            "qualification": {
                "status": "passed",
                "production_release_allowed": True,
                "physical_test_record": {
                    "result": "passed",
                    "operator": "fixture",
                    "date": "2026-01-01"
                },
                "coupon": {
                    "part": "fit_bridge_coupon",
                    "revision": "A",
                    "artifacts": [
                        {"id": "coupon-step", "role": "step",
                         "path": "out/step/fixture.step"},
                        {"id": "coupon-stl", "role": "stl",
                         "path": "out/stl/fixture.stl"},
                        {"id": "coupon-gcode", "role": "gcode",
                         "path": "artifact.txt"},
                        {"id": "coupon-evidence", "role": "evidence",
                         "path": "artifact.txt"}
                    ]
                }
            },
            "items": [{
                "part": "fixture",
                "quantity": 1,
                "revision": "A",
                "artifacts": [
                    {"id": "fixture-stl", "role": "stl",
                     "path": "out/stl/fixture.stl"},
                    {"id": "fixture-step", "role": "step",
                     "path": "out/step/fixture.step"},
                ],
            }],
        },
        "release_artifacts": [{
            "id": "fixture-artifact", "role": "evidence",
            "path": "artifact.txt",
        }],
    }
    path = root / "cad/release_catalog.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return path


class ReleaseCatalogTests(unittest.TestCase):
    def test_valid_catalog_separates_statuses_and_rounds_packages(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = _passing_catalog(root)
            result = release_catalog.audit(root, path=path)

        self.assertTrue(result["ready"], result["blockers"])
        item = result["items"][0]
        self.assertEqual(item["design_status"], "selected")
        self.assertEqual(item["purchase_status"], "cart_ready")
        self.assertEqual(item["quantity"]["packages_to_order"], 2)
        self.assertEqual(item["quantity"]["order_qty"], 4)

    def test_design_selection_does_not_imply_purchase_readiness(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = _passing_catalog(root)
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["items"][0]["purchase_status"] = "blocked"
            payload["items"][0].pop("selection")
            path.write_text(json.dumps(payload), encoding="utf-8")
            result = release_catalog.audit(root, path=path)

        self.assertFalse(result["ready"])
        self.assertTrue(any(
            row["kind"] == "purchase_selection" and row["id"] == "exact-bearing"
            for row in result["blockers"]
        ))

    def test_candidate_evidence_is_preserved_without_bypassing_blocker(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = _passing_catalog(root)
            payload = json.loads(path.read_text(encoding="utf-8"))
            item = payload["items"][0]
            item["purchase_status"] = "blocked"
            item.pop("selection")
            item["evidence"] = [{
                "source_type": "manufacturer_catalog",
                "url": "https://example.invalid/candidate",
                "observed": "body matches but mounting interface does not",
            }]
            item["blocker"] = {
                "kind": "interface_mismatch",
                "required": "M5",
                "observed": "M6",
            }
            path.write_text(json.dumps(payload), encoding="utf-8")
            result = release_catalog.audit(root, path=path)

        self.assertFalse(result["ready"])
        normalized = result["items"][0]
        self.assertEqual(normalized["evidence"], item["evidence"])
        self.assertEqual(normalized["blocker"], item["blocker"])
        self.assertTrue(any(
            row["kind"] == "purchase_selection"
            and row["id"] == "exact-bearing"
            for row in result["blockers"]
        ))

    def test_canonical_small_part_procurement_decisions_are_fail_closed(self):
        root = Path(__file__).resolve().parent.parent
        result = release_catalog.audit(root)
        items = {row["id"]: row for row in result["items"]}

        for item_id in (
            "felt-sheet-8341k31",
            "structural-petg-filament",
            "ceramic-bonding-epoxy",
            "foot-stud-threadlocker-loctite-243",
        ):
            row = items[item_id]
            self.assertEqual(row["purchase_status"], "cart_ready")
            self.assertTrue(row["selection"])
            self.assertTrue(row["evidence"])

        foot_record = items["foot-stack-elesa-432001-wurth-970180581"]
        self.assertEqual(foot_record["scope"], "excluded")
        self.assertEqual(foot_record["purchase_status"], "excluded")
        self.assertEqual(foot_record["quantity"]["order_qty"], 0)
        self.assertTrue(foot_record["evidence"])
        self.assertTrue(foot_record["model"]["artifact"]["exists"])
        self.assertFalse(foot_record["blocker"])

        eyelet = items["fixed-ceramic-eyelet-id4-od9"]
        self.assertEqual(eyelet["class"], "custom")
        self.assertEqual(eyelet["purchase_status"], "rfq_ready")
        self.assertEqual(eyelet["quantity"]["required_qty"], 1)
        self.assertEqual(eyelet["quantity"]["spare_qty"], 1)
        self.assertTrue(eyelet["evidence"])
        self.assertEqual(len(eyelet["manufacturing"]["artifacts"]), 2)
        self.assertFalse(eyelet["blocker"])

        endstop = items["endstop-omron-d2f-01l2-d3"]
        self.assertEqual(endstop["purchase_status"], "cart_ready")
        self.assertEqual(endstop["selection"]["supplier_sku"], "Z4701-ND")
        self.assertTrue(endstop["model"]["artifact"]["exists"])
        self.assertFalse(endstop["blocker"])

        spindle_bearings = items["bearing-608zz"]
        self.assertEqual(spindle_bearings["quantity"]["required_qty"], 2)
        self.assertEqual(spindle_bearings["quantity"]["spare_qty"], 1)
        self.assertEqual(spindle_bearings["quantity"]["order_qty"], 3)

        carriage = items["carriage-plate-mic6-0p250"]
        self.assertEqual(carriage["purchase_status"], "upload_ready")
        self.assertEqual(carriage["selection"]["supplier_sku"], "ALUMIC6-250")
        self.assertEqual(carriage["quantity"]["order_qty"], 1)
        self.assertEqual(
            {row["role"] for row in carriage["manufacturing"]["artifacts"]},
            {"dxf", "step", "preflight_json"},
        )

    def test_successor_custom_parts_and_hardware_are_fail_closed(self):
        root = Path(__file__).resolve().parent.parent
        raw = release_catalog.load(root / "cad" / "release_catalog.json")
        items = {row["id"]: row for row in raw["items"]}
        self.assertNotIn("tip-toroid-guide-alumina", items)
        self.assertNotIn("spindle-er8-c8-100l", items)
        expected_custom = {
            "flyer-guide-peek-one-piece": 1,
            "stator-short-leadin-peek-cap-pair": 2,
            "active-sector-peek-guide-pair": 2,
            "active-sector-aluminum-yoke": 1,
            "flyer-balance-b777-six-trim-set": 6,
        }
        for item_id, quantity in expected_custom.items():
            with self.subTest(item=item_id):
                row = items[item_id]
                self.assertEqual(row["design_status"], "selected")
                self.assertEqual(row["purchase_status"], "rfq_ready")
                self.assertEqual(row["authorization_status"], "blocked")
                self.assertEqual(row["candidate_purchase_status"], "rfq_ready")
                self.assertEqual(row["quantity"]["required_qty"], quantity)
                self.assertEqual(row["cost"]["status"], "tbd")
                self.assertTrue(row["manufacturing"]["artifacts"])
                contract = row["manufacturing"]["rfq_contract"]
                self.assertEqual(
                    contract["rfq_submission_status"], "ready"
                )
                self.assertFalse(contract["order_authorized"])
                self.assertFalse(contract["production_authorized"])
                self.assertTrue(contract["material_design_basis"])
                self.assertTrue(contract["supplier_return_requirements"])
                self.assertTrue(contract["receiving_inspection"])
                self.assertTrue(contract["qualification_before_use"])
                self.assertTrue(row["blocker"])
                roles = {
                    artifact["role"]
                    for artifact in row["manufacturing"]["artifacts"]
                }
                self.assertIn("rfq_csv", roles)
                self.assertIn("manifest_json", roles)
                manifest = next(
                    artifact
                    for artifact in row["manufacturing"]["artifacts"]
                    if artifact["role"] == "manifest_json"
                )
                self.assertEqual(
                    manifest["json_requirements"],
                    {
                        "schema": "successor-manufacturing-packet/v1",
                        "rfq_submission_authorized": True,
                        "order_authorized": False,
                        "production_authorized": False,
                    },
                )

        yoke = items["active-sector-aluminum-yoke"]
        self.assertIn(
            "rev6-front-plane-outboard-coil-bypass-yoke",
            yoke["blocker"]["observed"],
        )
        self.assertIn(
            "active-yoke-rigid-audit",
            {
                artifact["id"]
                for artifact in yoke["manufacturing"]["artifacts"]
            },
        )

        rfq_ready_scope = raw["policy"]["rfq_ready_scope"]
        self.assertIn("may be sent to a supplier", rfq_ready_scope)
        self.assertIn("does not assert", rfq_ready_scope)
        self.assertIn("order authorization", rfq_ready_scope)
        self.assertIn("production use", rfq_ready_scope)

        mappings = raw["hardware_purchase_map"]
        cart_ready_scope = raw["policy"]["cart_ready_scope"]
        self.assertIn("exact supplier SKU and pack", cart_ready_scope)
        self.assertIn("does not assert", cart_ready_scope)
        self.assertIn("fulfillment", cart_ready_scope)
        self.assertIn("order authorization", cart_ready_scope)
        self.assertIn("rechecked at checkout", cart_ready_scope)
        for stale in ("ISO10511-M3", "ISO4762-M3x18", "ISO4026-M3x6"):
            self.assertNotIn(stale, mappings)
        exact_ready_mappings = {
            "ISO10642-M3x6": ("92125A126", 100),
            "ISO4762-M2x6": ("91290A013", 100),
            "ISO4762-M2x8": ("91290A015", 100),
            "ISO4762-M2x20": ("91290A049", 100),
            "ISO4762-M3x14": ("91502A106", 100),
            "ISO4762-M4x10": ("90128A212", 100),
            "MCMASTER-94459A120": ("94459A120", 50),
            "MCMASTER-94459A150": ("94459A150", 50),
        }
        for sku, (supplier_sku, pack_qty) in exact_ready_mappings.items():
            with self.subTest(sku=sku):
                self.assertIn(sku, mappings)
                self.assertEqual(mappings[sku]["purchase_status"], "cart_ready")
                self.assertEqual(mappings[sku]["pack_qty"], pack_qty)
                self.assertEqual(
                    mappings[sku]["selection"]["supplier_sku"],
                    supplier_sku,
                )
                self.assertEqual(
                    mappings[sku]["selection"]["url"],
                    f"https://www.mcmaster.com/{supplier_sku}",
                )
                self.assertTrue(mappings[sku]["evidence"])
                self.assertNotIn("blocker", mappings[sku])
                self.assertIn("not asserted", mappings[sku]["note"])
                self.assertIn(
                    "Confirm current pack price",
                    mappings[sku]["checkout_condition"],
                )
                self.assertIn(
                    "not order or motion authorization",
                    mappings[sku]["checkout_condition"],
                )

        catalog_audit = release_catalog.audit(
            root,
            hardware_order=hardware.procurement_schedule(),
        )
        normalized_hardware = {
            row["id"].removeprefix("hardware:"): row
            for row in catalog_audit["hardware"]
        }
        custom_ids = set(expected_custom)
        self.assertFalse(any(
            blocker["kind"] == "purchase_selection"
            and blocker["id"] in custom_ids
            for blocker in catalog_audit["blockers"]
        ))
        self.assertEqual(
            {
                blocker["id"]
                for blocker in catalog_audit["blockers"]
                if blocker["kind"] == "production_authorization"
                and blocker["id"] in custom_ids
            },
            custom_ids,
        )
        normalized_items = {
            row["id"]: row for row in catalog_audit["items"]
        }
        for item_id in custom_ids:
            artifacts = normalized_items[item_id]["manufacturing"]["artifacts"]
            manifest = next(
                artifact
                for artifact in artifacts
                if artifact["role"] == "manifest_json"
            )
            self.assertEqual(
                manifest["json_observed"],
                {
                    "schema": "successor-manufacturing-packet/v1",
                    "rfq_submission_authorized": True,
                    "order_authorized": False,
                    "production_authorized": False,
                },
            )
        required_qty = {
            "ISO10642-M3x6": 4,
            "ISO4762-M2x6": 3,
            "ISO4762-M2x8": 2,
            "ISO4762-M2x20": 3,
            "ISO4762-M3x14": 4,
            "ISO4762-M4x10": 4,
            "MCMASTER-94459A120": 5,
            "MCMASTER-94459A150": 4,
        }
        for sku, expected_required in required_qty.items():
            with self.subTest(normalized_sku=sku):
                row = normalized_hardware[sku]
                self.assertEqual(row["purchase_status"], "cart_ready")
                self.assertEqual(
                    row["quantity"]["required_qty"], expected_required
                )
                self.assertEqual(row["quantity"]["packages_to_order"], 1)
                self.assertEqual(
                    row["quantity"]["order_qty"],
                    exact_ready_mappings[sku][1],
                )
                self.assertTrue(row["evidence"])
                self.assertIsNone(row["mapping_blocker"])
                self.assertIn("not asserted", row["note"])
                self.assertEqual(
                    row["checkout_condition"],
                    mappings[sku]["checkout_condition"],
                )

        flyer_pulley = items["flyer-pulley-nbk-p30-3gt-blp-6c-10"]
        self.assertEqual(flyer_pulley["selection"]["mpn"],
                         "P30-3GT-BLP-6C-10")
        self.assertEqual(flyer_pulley["purchase_status"], "cart_ready")
        self.assertEqual(flyer_pulley["candidate_purchase_status"],
                         "cart_ready")
        self.assertEqual(flyer_pulley["authorization_status"], "blocked")
        self.assertIn(
            "not order or motion authorization",
            flyer_pulley["checkout_condition"],
        )
        self.assertEqual(
            flyer_pulley["selection"]["cart_state"],
            "official_manufacturer_cart_in_stock",
        )
        self.assertEqual(flyer_pulley["selection"]["verified_on"],
                          "2026-07-12")
        self.assertIn("JPY 5,060", flyer_pulley["cost"]["basis"])
        self.assertIn("in stock", flyer_pulley["evidence"][0]["observed"])
        self.assertEqual(
            flyer_pulley["model"]["sha256"],
            "780110e1d59a988661f5ae80e9ebbe5d2eb324b9037d33a481809c939fa4c9f1",
        )

        balance = items["flyer-balance-b777-six-trim-set"]
        self.assertEqual(
            balance["receiving_contract"]["design_density_g_cm3"], 18.49
        )
        self.assertTrue(
            balance["receiving_contract"]
            ["density_substitution_requires_new_balance_solve"]
        )
        self.assertEqual(
            balance["evidence"][0]["supplier_skus"],
            ["5995N71", "5995N72"],
        )
        combined = " ".join(
            balance["manufacturing"]["rfq_contract"]
            ["supplier_return_requirements"]
        )
        self.assertIn("18.49 g/cm3", combined)
        self.assertIn("regenerated", combined)

        print_parts = {row["part"] for row in raw["print_plan"]["items"]}
        self.assertNotIn("flyer_pulley", print_parts)
        self.assertNotIn("wire_elbow", print_parts)
        self.assertTrue({
            "balance_retainer_rear_left", "balance_retainer_rear_right",
            "balance_retainer_front_left", "balance_retainer_front_right",
        }.issubset(print_parts))

    def test_canonical_m2_delta_rejects_stale_parts_and_separates_cost_state(self):
        root = Path(__file__).resolve().parent.parent
        raw = release_catalog.load(root / "cad" / "release_catalog.json")
        raw_ids = {row["id"] for row in raw["items"]}
        for stale in (
            "motor-m2-6627t421",
            "driver-m2-cl42t-v4p1",
            "gt2-pulley-40t-bore5-w6",
            "gt2-belt-200-2gt-w6",
            "power-supply-24v-10a",
        ):
            self.assertNotIn(stale, raw_ids)

        result = release_catalog.audit(root)
        items = {row["id"]: row for row in result["items"]}

        motor = items["motor-m2-leadshine-cs-m21708"]
        self.assertEqual(motor["selection"]["mpn"], "CS-M21708")
        self.assertEqual(motor["purchase_status"], "cart_ready")
        self.assertEqual(motor["authorization_status"], "order_ready")
        self.assertEqual(motor["cost"]["status"], "known_current")
        self.assertEqual(motor["cost"]["extended_usd"], 199.0)
        self.assertEqual(
            motor["model"]["sha256"],
            "7e995e724fc7e019278e0a919ba1db8c8abb3333f156c64eb6e62485e0f6662b",
        )

        driver = items["driver-m2-leadshine-cs-d508-conditional"]
        self.assertEqual(driver["selection"]["mpn"], "CS-D508")
        self.assertEqual(driver["design_status"], "selected")
        self.assertEqual(driver["purchase_status"], "cart_ready")
        self.assertEqual(driver["candidate_purchase_status"], "cart_ready")
        self.assertEqual(driver["authorization_status"], "order_ready")
        self.assertEqual(driver["cost"]["extended_usd"], 199.0)
        self.assertEqual(
            driver["evidence"][0]["sha256"],
            "0faaf40eebe24203511b50b3e3658bec9fc298b13221031759b84d3eb9bdba60",
        )
        self.assertNotEqual(driver["selection"]["mpn"], "CL42T-V41")
        self.assertIn("rejected", driver["note"])
        self.assertNotEqual(driver["selection"]["mpn"], "CS1-D503S")
        self.assertEqual(
            driver["receiving_contract"]["software_peak_current_A"],
            {"candidate_lower": 3.5, "candidate_upper": 3.6, "released": None},
        )
        self.assertEqual(
            driver["commissioning_gate"]["kind"],
            "curve_configuration_or_hot_dyno_unreleased_before_motion",
        )
        self.assertIsNone(driver["blocker"])
        self.assertIn("post-purchase commissioning", driver["note"])

        pulley = items[
            "motor-pulley-nbk-p30-3gt-blp-6c-5-bnw-conditional"
        ]
        self.assertEqual(pulley["purchase_status"], "rfq_ready")
        self.assertEqual(pulley["candidate_purchase_status"], "rfq_ready")
        self.assertEqual(pulley["authorization_status"], "conditional")
        self.assertEqual(pulley["cost"]["status"], "planning_allowance")
        self.assertIn("not a BNW quote", pulley["cost"]["basis"])
        self.assertEqual(
            pulley["model"]["sha256"],
            "996449b7d9ec7703e7b38c6f75eff00a1174e3e1f088c05f0f1460b205169df9",
        )
        rfq = pulley["manufacturing"]["rfq_configuration"]
        self.assertEqual(rfq["base_part"], "P30-3GT-BLP-6C-5")
        self.assertEqual(rfq["additional_machining_code"], "BNW")
        self.assertEqual(rfq["bore_mm"], 5.0)
        self.assertIn("D-shaft", rfq["application_shaft"])
        self.assertIn("configured drawing", rfq["required_supplier_return"])
        self.assertIn("M3x12", rfq["do_not_assume"])
        artifacts = pulley["manufacturing"]["artifacts"]
        self.assertEqual(len(artifacts), 1)
        self.assertTrue(artifacts[0]["exists"])
        self.assertEqual(artifacts[0]["role"], "rfq_request")

        belt = items["timing-belt-210-3gt-6"]
        self.assertEqual(belt["selection"]["supplier_sku"], "210-3GT-6")
        self.assertEqual(belt["quantity"]["order_qty"], 2)
        self.assertEqual(belt["cost"]["unit_usd"], 11.14)
        self.assertEqual(belt["cost"]["extended_usd"], 22.28)

        supply = items["power-supply-m2-regulated-36v-condition"]
        self.assertEqual(supply["scope"], "machine")
        self.assertEqual(supply["design_status"], "selected")
        self.assertEqual(supply["purchase_status"], "blocked")
        self.assertEqual(supply["candidate_purchase_status"], "cart_ready")
        self.assertEqual(supply["authorization_status"], "conditional")
        self.assertEqual(supply["selection"]["mpn"], "LSP-360-36")
        self.assertEqual(supply["cost"]["status"], "known_current")
        self.assertEqual(supply["cost"]["unit_usd"], 199.0)
        self.assertEqual(supply["cost"]["extended_usd"], 199.0)
        self.assertEqual(
            supply["receiving_contract"]["input_voltage_windows_vac"],
            [[92.0, 138.0], [184.0, 276.0]],
        )
        self.assertEqual(
            supply["blocker"]["kind"], "mains_safety_integration_unreleased"
        )
        self.assertIn("36 V", supply["description"])
        self.assertNotIn("24 V", supply["description"])

        costs = result["cost_summary"]
        self.assertFalse(costs["complete_machine_total_available"])
        self.assertEqual(
            costs["annotated_order_ready_known_current_usd"], 420.28
        )
        self.assertEqual(
            costs["annotated_conditional_known_current_usd"], 199.0
        )
        self.assertEqual(costs["annotated_planning_allowance_usd"], 50.0)
        self.assertNotIn(
            "power-supply-m2-regulated-36v-condition",
            costs["tbd_required_item_ids"],
        )

        blockers = {(row["kind"], row["id"]) for row in result["blockers"]}
        self.assertNotIn(
            ("production_authorization",
             "driver-m2-leadshine-cs-d508-conditional"),
            blockers,
        )
        for item_id in (
            "motor-pulley-nbk-p30-3gt-blp-6c-5-bnw-conditional",
            "power-supply-m2-regulated-36v-condition",
        ):
            self.assertIn(("production_authorization", item_id), blockers)

        adhesive = items["ceramic-bonding-epoxy"]
        self.assertIn("fixed ceramic eyelets", adhesive["description"])
        self.assertIn("shaft-wrap sleeve", adhesive["description"])
        self.assertNotIn("torus", adhesive["description"].lower())

    def test_cost_metadata_does_not_bypass_authorization(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = _passing_catalog(root)
            payload = json.loads(path.read_text(encoding="utf-8"))
            item = payload["items"][0]
            item["purchase_status"] = "blocked"
            item["authorization_status"] = "conditional"
            item["candidate_purchase_status"] = "cart_ready"
            item["cost"] = {
                "status": "known_current",
                "unit_usd": 12.5,
                "extended_usd": 50.0,
                "basis": "current candidate price",
                "verified_on": "2026-07-11",
            }
            path.write_text(json.dumps(payload), encoding="utf-8")
            result = release_catalog.audit(root, path=path)

        self.assertFalse(result["ready"])
        normalized = result["items"][0]
        self.assertEqual(normalized["authorization_status"], "conditional")
        self.assertEqual(normalized["cost"]["extended_usd"], 50.0)
        self.assertEqual(
            result["cost_summary"]["annotated_conditional_known_current_usd"],
            50.0,
        )
        self.assertTrue(any(
            row["kind"] == "production_authorization"
            and row["id"] == "exact-bearing"
            for row in result["blockers"]
        ))

    def test_canonical_winding_consumables_keep_nominal_and_measured_inputs_distinct(self):
        root = Path(__file__).resolve().parent.parent
        result = release_catalog.audit(root)
        items = {row["id"]: row for row in result["items"]}

        wire = items["default-magnet-wire-32snsp125"]["receiving_contract"]
        self.assertEqual(wire["supplier_nominal_finished_diameter_mm"], 0.22352)
        self.assertEqual(
            wire["accepted_conservative_measured_finished_diameter_mm"],
            {"minimum": 0.22, "maximum": 0.235},
        )
        liner = items["default-slot-liner-nomex410-5mil"]["receiving_contract"]
        self.assertEqual(liner["supplier_nominal_thickness_mm"], 0.127)
        self.assertEqual(
            liner["accepted_conservative_measured_installed_thickness_mm"],
            {"minimum": 0.12, "maximum": 0.14},
        )
        self.assertIn("job_artifacts.py", wire["regeneration_command"])
        self.assertIn("job_artifacts.py", liner["regeneration_command"])

        release_ids = {row["id"] for row in result["release_artifacts"]}
        # ContractWind-only packing outputs remain outside the allowlist. The
        # selected integrated rigid, conductor, wrap and launch authorities
        # are separate hash-bound release artifacts below.
        self.assertNotIn("release-slot-wire-routes", release_ids)
        self.assertNotIn("release-continuous-wire-audit", release_ids)
        self.assertIn("release-upstream-raw-wirepath", release_ids)
        self.assertIn("release-winding-tooling-authority", release_ids)
        for artifact_id in (
            "release-integrated-candidate-step",
            "release-integrated-candidate-report",
            "release-active-sector-audit",
            "release-active-sector-loci",
            "release-integrated-adapter-manifest",
            "release-continuous-conductor-presentation",
            "release-full-cycle-conductor-authority",
            "release-shaft-wrap-regression",
            "release-launch-envelope-authority",
            "release-integrated-animation-glb",
            "release-integrated-player",
        ):
            self.assertIn(artifact_id, release_ids)
        self.assertFalse(result["ready"])
        self.assertTrue(any(
            row["kind"] in {"evidence_verdict", "missing_artifact"}
            and row["id"] == "release"
            for row in result["blockers"]
        ))

    def test_release_allowlist_separates_legacy_and_selected_authorities(self):
        root = Path(__file__).resolve().parent.parent
        catalog = release_catalog.load(root / "cad" / "release_catalog.json")
        artifacts = {row["id"]: row for row in catalog["release_artifacts"]}
        expected = {
            "release-upstream-raw-capture":
                "out/capture/upstream_current_raw.jsonl",
            "release-upstream-raw-animation-glb":
                "out/winding_cycle_upstream_raw.glb",
            "release-upstream-raw-player":
                "out/play_animation_upstream_raw.html",
            "release-upstream-raw-cycle":
                "out/reports/upstream_current_raw_cycle.json",
            "release-upstream-raw-clearance":
                "out/reports/clearance_upstream_raw.json",
            "release-upstream-raw-wirepath":
                "out/reports/wirepath_upstream_raw.json",
            "release-winding-tooling-authority":
                "out/reports/winding_tooling_authority.json",
            "release-integrated-candidate-step":
                "out/review/integrated_release_candidate.step",
            "release-integrated-candidate-report":
                "out/reports/integrated_release_candidate.json",
            "release-active-sector-audit":
                "out/reports/carriage_active_sector_terminal_guide_audit.json",
            "release-active-sector-loci":
                "out/reports/carriage_active_sector_terminal_guide_loci.json",
            "release-integrated-adapter-manifest":
                "out/review/integrated_adapter/links/manifest.json",
            "release-continuous-conductor-presentation":
                "out/reports/continuous_conductor_route.json",
            "release-full-cycle-conductor-authority":
                "out/reports/full_cycle_continuous_conductor_authority_audit.json",
            "release-shaft-wrap-regression":
                "out/reports/shaft_wrap_regression_evidence.json",
            "release-launch-envelope-authority":
                "out/reports/launch_envelope_authority.json",
            "release-integrated-animation-glb":
                "out/review/integrated_adapter/winding_cycle_integrated_candidate_raw.glb",
            "release-integrated-player":
                "out/review/integrated_adapter/play_integrated_candidate_raw.html",
        }
        for artifact_id, path in expected.items():
            self.assertIn(artifact_id, artifacts)
            self.assertEqual(artifacts[artifact_id]["path"], path)

        all_paths = {row["path"] for row in catalog["release_artifacts"]}
        for legacy in (
            "out/capture/commands.jsonl",
            "out/winding_cycle.glb",
            "out/play_animation.html",
            "out/reports/cycle.json",
            "out/reports/clearance.json",
            "out/reports/wirepath.json",
            "out/reports/slot_wire_routes.json",
            "out/reports/continuous_wire_audit.json",
        ):
            self.assertNotIn(legacy, all_paths)

        self.assertEqual(
            artifacts["release-upstream-raw-cycle"]["json_requirements"],
            {
                "schema": "captured-cycle-verification/v2",
                "status": "PASS",
                "passed": True,
            },
        )
        self.assertFalse(
            artifacts["release-winding-tooling-authority"]["required"]
        )
        self.assertEqual(
            artifacts["release-winding-tooling-authority"]["role"],
            "historical_predecessor_evidence",
        )
        self.assertEqual(
            artifacts["release-upstream-raw-clearance"]["role"],
            "legacy_baseline_evidence",
        )
        self.assertEqual(
            artifacts["release-upstream-raw-wirepath"]["role"],
            "legacy_baseline_evidence",
        )
        self.assertEqual(
            artifacts["release-full-cycle-conductor-authority"]
            ["json_requirements"]["production_authorized"],
            True,
        )
        self.assertEqual(
            artifacts["release-launch-envelope-authority"]
            ["json_requirements"],
            {
                "schema": "launch-envelope-authority/v1",
                "status": "PASS",
                "production_authorized": True,
            },
        )
        self.assertTrue(artifacts["release-readiness"]["generated"])
        self.assertEqual(
            artifacts["release-elastic-wire-contact"]["json_requirements"],
            {"schema": "elastic-wire-contact-study/v1"},
        )

    def test_optional_electronics_do_not_gate_mechanical_release(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = _passing_catalog(root)
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["items"].append({
                "id": "optional-controller",
                "class": "electronics",
                "scope": "optional",
                "category": "electronics",
                "description": "Future controller integration",
                "design_status": "pending",
                "purchase_status": "blocked",
                "quantity": {
                    "required_qty": 1,
                    "spare_qty": 0,
                    "pack_qty": 1,
                    "uom": "each",
                },
            })
            path.write_text(json.dumps(payload), encoding="utf-8")
            result = release_catalog.audit(root, path=path)

        self.assertTrue(result["ready"], result["blockers"])
        optional = next(
            row for row in result["items"]
            if row["id"] == "optional-controller"
        )
        self.assertEqual(optional["scope"], "optional")
        self.assertEqual(optional["purchase_status"], "blocked")

    def test_unmapped_hardware_is_not_order_ready(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = _passing_catalog(root)
            result = release_catalog.audit(root, path=path, hardware_order=[{
                "sku": "ISO4762-M3x10",
                "description": "M3x10 screw",
                "standard": "ISO 4762",
                "required_qty": 4,
                "spare_qty": 2,
                "status": "selected",
                "schedule_ids": ["motor_screws"],
            }])

        self.assertFalse(result["ready"])
        self.assertTrue(any(
            row["kind"] == "hardware_purchase"
            and row["id"] == "ISO4762-M3x10"
            for row in result["blockers"]
        ))

    def test_physical_coupon_gate_cannot_be_bypassed(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = _passing_catalog(root)
            payload = json.loads(path.read_text(encoding="utf-8"))
            qualification = payload["print_plan"]["qualification"]
            qualification["status"] = "physical_test_required"
            qualification["production_release_allowed"] = False
            qualification["physical_test_record"] = None
            path.write_text(json.dumps(payload), encoding="utf-8")
            result = release_catalog.audit(root, path=path)

        self.assertFalse(result["ready"])
        self.assertTrue(any(
            row["kind"] == "print_qualification"
            and row["id"] == "fit_bridge_coupon"
            for row in result["blockers"]
        ))

    def test_required_json_evidence_must_have_the_declared_pass_verdict(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = _passing_catalog(root)
            evidence = root / "route.json"
            evidence.write_text(
                json.dumps({"schema": "route/v1", "status": "FAIL"}),
                encoding="utf-8",
            )
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["release_artifacts"].append({
                "id": "route-proof",
                "role": "evidence",
                "path": "route.json",
                "json_requirements": {
                    "schema": "route/v1",
                    "status": "PASS",
                },
            })
            path.write_text(json.dumps(payload), encoding="utf-8")
            failed = release_catalog.audit(root, path=path)
            evidence.write_text(
                json.dumps({"schema": "route/v1", "status": "PASS"}),
                encoding="utf-8",
            )
            passed = release_catalog.audit(root, path=path)

        self.assertFalse(failed["ready"])
        self.assertTrue(any(
            row["kind"] == "evidence_verdict" and row["id"] == "release"
            for row in failed["blockers"]
        ))
        self.assertTrue(passed["ready"], passed["blockers"])
        route = next(
            row for row in passed["release_artifacts"]
            if row["id"] == "route-proof"
        )
        self.assertEqual(route["json_observed"]["status"], "PASS")

    def test_receiving_contract_is_preserved_in_normalized_report(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = _passing_catalog(root)
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["items"][0]["receiving_contract"] = {
                "accepted_measured_mm": {"minimum": 0.220, "maximum": 0.235},
                "regenerate_before_use": True,
            }
            path.write_text(json.dumps(payload), encoding="utf-8")
            result = release_catalog.audit(root, path=path)

        self.assertTrue(result["ready"], result["blockers"])
        self.assertEqual(
            result["items"][0]["receiving_contract"],
            payload["items"][0]["receiving_contract"],
        )

    def test_stage_is_allowlisted_hashed_and_detects_tampering(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = _passing_catalog(root)
            report = release_catalog.audit(root, path=path)
            staged = release_catalog.stage_release(
                report, root, destination=root / "release",
            )
            first = release_catalog.verify_stage(staged)
            (staged / "files/artifact.txt").write_text(
                "tampered\n", encoding="utf-8",
            )
            tampered = release_catalog.verify_stage(staged)

        self.assertTrue(first["passed"], first["errors"])
        self.assertFalse(tampered["passed"])
        self.assertTrue(any("SHA-256 mismatch" in row for row in tampered["errors"]))

    def test_stage_rejects_unmanifested_file(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = _passing_catalog(root)
            report = release_catalog.audit(root, path=path)
            staged = release_catalog.stage_release(
                report, root, destination=root / "release",
            )
            _write(staged / "stale-output.stl")
            result = release_catalog.verify_stage(staged)

        self.assertFalse(result["passed"])
        self.assertTrue(any("unmanifested files" in row for row in result["errors"]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
