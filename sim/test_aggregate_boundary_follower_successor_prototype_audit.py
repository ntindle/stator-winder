"""Focused fail-closed audit tests for successor-follower prototype."""

from __future__ import annotations

import unittest
from pathlib import Path
import sys


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_successor_prototype_audit as audit


class SuccessorFollowerPrototypeAuditTests(unittest.TestCase):

    def test_audit_passes_all_checks_without_authority(self):
        report = audit.run_audit()
        audit.validate_report(report)
        self.assertEqual(report["status"], "PASS")
        self.assertTrue(all(report["checks"].values()))
        self.assertFalse(any(report["authority"].values()))

    def test_frozen_input_hash_chain_is_exact(self):
        report = audit.run_audit()
        self.assertEqual(report["input_hashes"], audit.EXPECTED_HASHES)


if __name__ == "__main__":
    unittest.main()
