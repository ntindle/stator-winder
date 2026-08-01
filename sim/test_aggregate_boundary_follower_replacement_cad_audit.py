"""Focused tests for the replacement-carriage CAD evidence binder."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_replacement_cad_audit as audit


class AggregateBoundaryFollowerReplacementCadAuditTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.report = audit.analyze()

    def test_all_final_artifacts_are_exactly_sha_bound(self):
        bindings = self.report["artifact_binding"]
        self.assertEqual(
            set(bindings),
            {"cad_source", "manifest", "step", "architecture_report"},
        )
        for evidence in bindings.values():
            self.assertTrue(evidence["exists"])
            self.assertTrue(evidence["matches_inspected_sha256"])
            self.assertEqual(evidence["sha256"], evidence["expected_sha256"])
            self.assertEqual(len(evidence["sha256"]), 64)
            self.assertGreater(evidence["byte_count"], 0)
        self.assertEqual(
            bindings["step"]["sha256"], audit.INSPECTED_STEP_SHA256,
        )
        self.assertEqual(
            self.report["architecture_report_internal_sha256"],
            "8d7a08946edb8ea69f0ac12c66a384451f2db5b6bc8ba8b167654f1aeb95af17",
        )

    def test_step_tree_is_69_manufactured_plus_4_blocker_only(self):
        leaves = self.report["leaf_accounting"]
        self.assertEqual(leaves["STEP_review_leaf_count"], 73)
        self.assertEqual(leaves["manufactured_leaf_count"], 69)
        self.assertEqual(leaves["blocker_only_envelope_count"], 4)
        self.assertEqual(
            leaves["manufactured_leaf_count"]
            + leaves["blocker_only_envelope_count"],
            leaves["STEP_review_leaf_count"],
        )
        self.assertEqual(leaves["carrier_leaf_count"], 1)
        self.assertEqual(leaves["moving_occurrence_count"], 4)
        self.assertEqual(leaves["moving_leaf_count"], 60)
        self.assertEqual(leaves["primary_mount_leaf_count"], 8)
        self.assertEqual(
            self.report["step_binding"]["leaf_count_method"],
            "ROOT_OCC_INSPECTION_BOUND_BY_SHA256",
        )
        self.assertEqual(
            self.report["step_binding"]["inspection_warning_count"], 0,
        )

    def test_diagonal_mount_and_catalog_outer_pivot_stacks_are_exact(self):
        witness = self.report["hardware_witness"]
        self.assertEqual(
            witness["diagonal_M4_axes_local_XY_mm"],
            [[29.0, -24.5], [35.0, -17.5],
             [29.0, 24.5], [35.0, 17.5]],
        )
        source = witness["source"]
        self.assertEqual(source["occurrence_leaf_counts"], [15, 15, 15, 15])
        self.assertEqual(source["NBK_M4_screw_count"], 4)
        self.assertEqual(source["M4_washer_count"], 0)
        self.assertEqual(source["M4_insert_count"], 4)
        self.assertEqual(source["outer_SCCG5_10_pin_count"], 4)
        self.assertEqual(source["outer_NETWS4_ring_count"], 8)
        self.assertEqual(source["outer_DIN988_shim_count"], 8)
        self.assertTrue(source["all_leaf_labels_unique"])
        self.assertEqual(witness["M4_screw_sku"], "NBK_SSHS-M4-10-SD-ALK")
        self.assertEqual(witness["outer_pin_sku"], "MISUMI_SCCG5-10")

    def test_all_36_states_reduce_to_5_zero_positive_signatures(self):
        pairs = self.report["state_pair_audit"]
        self.assertEqual(pairs["state_count"], 36)
        self.assertEqual(pairs["engaged_state_count"], 12)
        self.assertEqual(pairs["all_parked_state_count"], 24)
        self.assertEqual(pairs["unique_geometry_signature_count"], 5)
        self.assertEqual(
            set(pairs["geometry_signatures"]),
            audit.EXPECTED_GEOMETRY_SIGNATURES,
        )
        self.assertEqual(pairs["follower_carrier_failure_state_count"], 0)
        self.assertEqual(pairs["complete_installed_failure_state_count"], 0)
        self.assertTrue(pairs["all_scopes_zero_positive"])
        self.assertFalse(pairs["clearance_authority"])

    def test_static_geometry_closes_but_every_physical_authority_is_false(self):
        self.assertEqual(self.report["status"], "FAIL")
        self.assertTrue(self.report["static_CAD_geometry_proven"])
        self.assertFalse(self.report["mechanism_complete"])
        self.assertTrue(all(self.report["proof_gates"].values()))
        self.assertEqual(
            set(self.report["authority"]), set(audit.AUTHORITY_KEYS),
        )
        self.assertFalse(any(self.report["authority"].values()))
        source = audit.Path(audit.__file__).read_text(encoding="utf-8")
        self.assertNotIn("import aggregate_boundary_follower_acceptance", source)

    def test_report_hash_tamper_detection_and_outputs(self):
        audit.validate_report_integrity(self.report)
        tampered = deepcopy(self.report)
        tampered["leaf_accounting"]["STEP_review_leaf_count"] = 72
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            audit.validate_report_integrity(tampered)

        generated = audit.write_outputs(self.report)
        written = json.loads(audit.OUTPUT_JSON.read_text(encoding="utf-8"))
        self.assertEqual(generated["report_sha256"], written["report_sha256"])
        audit.validate_report_integrity(written)
        markdown = audit.OUTPUT_MD.read_text(encoding="utf-8")
        self.assertIn("69 manufactured + 4 blocker-only", markdown)
        self.assertIn("zero positive common volume", markdown)


if __name__ == "__main__":
    unittest.main(verbosity=2)
