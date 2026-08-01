"""Regression checks for the standalone frozen-successor RFQ packet."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
import unittest

import successor_manufacturing as packet


class SuccessorManufacturingTests(unittest.TestCase):
    def test_source_parts_are_twelve_single_solids_without_retired_guides(self):
        parts = packet.manufacturing_parts()
        self.assertEqual(len(parts), 12)
        self.assertNotIn("tip_toroid_guide", parts)
        self.assertNotIn("wire_elbow", parts)
        self.assertNotIn("custom_flyer_p30_3gt", parts)
        self.assertEqual(
            sum(name.startswith("balance_b777_") for name in parts), 6
        )
        for name, row in parts.items():
            with self.subTest(part=name):
                self.assertTrue(row.part.is_valid)
                self.assertEqual(len(list(row.part.solids())), 1)
                self.assertGreater(row.part.volume, 0.0)

    def test_checked_in_manifest_is_hash_bound_and_fail_closed(self):
        self.assertTrue(packet.MANIFEST.is_file())
        data = json.loads(packet.MANIFEST.read_text(encoding="utf-8"))
        packet.validate_manifest(data)
        self.assertEqual(data["schema"], "successor-manufacturing-packet/v1")
        self.assertFalse(data["production_authorized"])
        self.assertFalse(data["order_authorized"])
        self.assertTrue(data["rfq_submission_authorized"])
        exported_ids = {row["id"] for row in data["parts"]}
        pending_ids = set(data["pending_rev2_packaging_parts"])
        pending_supplier_ids = set(data["pending_supplier_authority_parts"])
        self.assertEqual(exported_ids | pending_ids | pending_supplier_ids,
                         set(packet.manufacturing_parts()))
        self.assertFalse(exported_ids & pending_ids)
        self.assertFalse(exported_ids & pending_supplier_ids)
        self.assertEqual(len(data["balance_contract"]["rear_slug_lengths_mm"]), 4)
        self.assertGreater(
            data["balance_contract"]["front_trim_common_thickness_mm"], 0.0
        )
        root = Path(__file__).resolve().parent.parent
        for row in data["parts"]:
            path = root / row["file"]
            with self.subTest(part=row["id"]):
                self.assertTrue(path.is_file())
                self.assertEqual(
                    hashlib.sha256(path.read_bytes()).hexdigest(), row["sha256"]
                )
                self.assertTrue(row["single_solid"])
                self.assertEqual(row["candidate_purchase_status"], "rfq_ready")
                self.assertEqual(row["purchase_status"], "rfq_ready")
                self.assertEqual(row["authorization_status"], "blocked")
                self.assertTrue(row["rfq_submission_authorized"])
                self.assertEqual(row["cost_status"], "tbd")
                contract = row["rfq_contract"]
                self.assertTrue(contract["material_design_basis"])
                self.assertTrue(contract["quote_requirements"])
                self.assertTrue(contract["receiving_inspection"])
                self.assertTrue(contract["qualification_before_use"])
        self.assertFalse(pending_supplier_ids)
        readiness = data["rfq_readiness"]
        self.assertEqual(
            readiness["rfq_ready_part_ids"], sorted(exported_ids)
        )
        self.assertEqual(readiness["order_ready_part_ids"], [])
        self.assertEqual(readiness["production_authorized_part_ids"], [])

        expected_sources = {
            relative: hashlib.sha256(path.read_bytes()).hexdigest()
            for relative, path in packet._expected_source_paths().items()
        }
        self.assertEqual(data["source_hashes"], expected_sources)

        artifacts = data["artifacts"]
        self.assertEqual(set(artifacts), {"rfq_csv", "drawing_pdf"})
        for name, path in (
            ("rfq_csv", packet.RFQ_CSV),
            ("drawing_pdf", packet.DRAWING),
        ):
            with self.subTest(artifact=name):
                record = artifacts[name]
                self.assertEqual(record["path"], path.relative_to(root).as_posix())
                self.assertEqual(record["exists"], path.is_file())
                self.assertEqual(
                    record["bytes"], path.stat().st_size if path.is_file() else None,
                )
                self.assertEqual(
                    record["sha256"],
                    hashlib.sha256(path.read_bytes()).hexdigest()
                    if path.is_file() else None,
                )

    def test_manifest_validator_rejects_source_step_and_packet_hash_drift(self):
        original = json.loads(packet.MANIFEST.read_text(encoding="utf-8"))
        mutations = (
            ("source", lambda value: value["source_hashes"].__setitem__(
                "cad/integrated_release_candidate.py", "0" * 64
            )),
            ("step", lambda value: value["parts"][0].__setitem__(
                "sha256", "0" * 64
            )),
            ("rfq", lambda value: value["artifacts"]["rfq_csv"].__setitem__(
                "sha256", "0" * 64
            )),
            ("drawing", lambda value: value["artifacts"]["drawing_pdf"].__setitem__(
                "sha256", "0" * 64
            )),
            ("rfq_status", lambda value: value["parts"][0].__setitem__(
                "candidate_purchase_status", "blocked"
            )),
            ("rfq_contract", lambda value: value["parts"][0][
                "rfq_contract"
            ].__setitem__("receiving_inspection", [])),
            ("purchase_status", lambda value: value["parts"][0].__setitem__(
                "purchase_status", "cart_ready"
            )),
        )
        for name, mutate in mutations:
            with self.subTest(drift=name):
                value = json.loads(json.dumps(original))
                mutate(value)
                with self.assertRaises(ValueError):
                    packet.validate_manifest(value)

    def test_rfq_csv_and_renderable_drawing_are_present(self):
        self.assertTrue(packet.RFQ_CSV.is_file())
        self.assertGreater(packet.RFQ_CSV.stat().st_size, 100)
        data = json.loads(packet.MANIFEST.read_text(encoding="utf-8"))
        if (data["pending_rev2_packaging_parts"]
                or data["pending_supplier_authority_parts"]):
            self.assertFalse(
                packet.DRAWING.is_file(),
                "successor RFQ PDF must not be frozen before rev2 packaging closes",
            )
        else:
            self.assertTrue(packet.DRAWING.is_file())
            self.assertGreater(packet.DRAWING.stat().st_size, 1000)

        with packet.RFQ_CSV.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), len(data["parts"]))
        for row in rows:
            with self.subTest(rfq_row=row["part_id"]):
                self.assertEqual(row["candidate_purchase_status"], "RFQ_READY")
                self.assertEqual(row["purchase_status"], "RFQ_READY")
                self.assertEqual(row["order_authorized"], "FALSE")
                self.assertTrue(row["material_design_basis"])
                self.assertTrue(row["supplier_return_requirements"])
                self.assertTrue(row["receiving_inspection"])
                self.assertTrue(row["qualification_before_use"])

    def test_balance_rfq_is_bound_to_the_digital_density_and_revalidation(self):
        data = json.loads(packet.MANIFEST.read_text(encoding="utf-8"))
        balance = [
            row for row in data["parts"]
            if row["id"].startswith("balance_b777_")
        ]
        self.assertEqual(len(balance), 6)
        for row in balance:
            contract = row["rfq_contract"]
            combined = " ".join(
                [contract["material_design_basis"]]
                + contract["quote_requirements"]
                + contract["receiving_inspection"]
            )
            self.assertIn("18.49 g/cm3", combined)
            self.assertIn("new balance solve", combined)

    def test_custom_packet_never_promotes_quote_readiness_to_order_readiness(self):
        data = json.loads(packet.MANIFEST.read_text(encoding="utf-8"))
        self.assertTrue(data["rfq_submission_authorized"])
        self.assertFalse(data["order_authorized"])
        self.assertFalse(data["production_authorized"])
        self.assertTrue(all(
            row["candidate_purchase_status"] == "rfq_ready"
            and row["purchase_status"] == "rfq_ready"
            and row["authorization_status"] == "blocked"
            for row in data["parts"]
        ))


if __name__ == "__main__":
    unittest.main(verbosity=2)
