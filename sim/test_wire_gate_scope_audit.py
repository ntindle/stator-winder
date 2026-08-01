from __future__ import annotations

import unittest

import wire_gate_scope_audit as audit


class WireGateScopeAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = audit.analyze()

    def test_goal_clauses_are_bound(self):
        self.assertTrue(all(self.report["goal_contract"].values()))

    def test_no_production_gate_is_modified(self):
        self.assertTrue(self.report["advisory_only"])
        self.assertFalse(self.report["production_gate_modified"])

    def test_real_open_items_are_contact_semantics_and_R3(self):
        ids = {
            row["id"]
            for row in self.report["required_and_currently_unproven"]
        }
        self.assertEqual(
            ids,
            {"moving_path_contact_semantics", "R3_workpiece_turning_path"},
        )

    def test_exact_packing_and_tooling_selection_are_not_goal_DOD3(self):
        ids = {
            row["id"]
            for row in self.report["stricter_internal_policy_not_DOD3"]
        }
        self.assertIn(
            "match_one_preselected_hash_bound_packing_schedule", ids
        )
        self.assertIn(
            "global_noncrossing_repacking_branch_certificate", ids
        )
        self.assertIn(
            "exactly_one_architecture_study_self_declares_production_authority",
            ids,
        )

    def test_current_required_positive_evidence_passes(self):
        failed = [
            row["id"]
            for row in self.report["required_and_currently_proven"]
            if not row["passed"]
        ]
        self.assertEqual(failed, [])

    def test_report_self_hash(self):
        self.assertEqual(
            self.report["report_sha256"], audit._canonical_hash(self.report)
        )


if __name__ == "__main__":
    unittest.main()
