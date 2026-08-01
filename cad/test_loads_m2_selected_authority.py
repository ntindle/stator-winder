"""Regression gates for the selected M2 authority consumed by loads.py."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import loads  # noqa: E402


class SelectedM2LoadsAuthorityTests(unittest.TestCase):
    def test_selected_report_is_hash_bound_and_production_stays_blocked(self):
        authority = loads.selected_m2_drive_authority()
        self.assertEqual(
            authority["motor"], "Leadshine CS-M21708 closed-loop NEMA17"
        )
        self.assertEqual(authority["driver"], "Leadshine CS-D508")
        self.assertEqual(authority["supply"], "Leadshine LSP-360-36")
        self.assertEqual(authority["curve_condition"], "36 VDC, RMS 2.5 A")
        self.assertAlmostEqual(
            authority["available_300rpm_lower_edge_nm"], 0.735
        )
        self.assertGreaterEqual(
            authority["available_to_required_multiple"], 2.0
        )
        self.assertTrue(authority["static_curve_margin_gate_ge_2x"])
        self.assertFalse(authority["driver_current_configuration_verified"])
        self.assertFalse(authority["installed_hot_dyno_verified"])
        self.assertFalse(authority["production_authorized"])
        source = ROOT / authority["source_path"]
        self.assertEqual(
            authority["source_sha256"],
            hashlib.sha256(source.read_bytes()).hexdigest(),
        )

    def test_stale_selection_report_is_rejected(self):
        original = json.loads(
            loads.M2_SELECTION_REPORT.read_text(encoding="utf-8")
        )
        stale = deepcopy(original)
        stale["analysis_source_sha256"] = "0" * 64
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "stale.json"
            path.write_text(json.dumps(stale), encoding="utf-8")
            with mock.patch.object(loads, "M2_SELECTION_REPORT", path):
                with self.assertRaisesRegex(RuntimeError, "stale"):
                    loads.selected_m2_drive_authority()

    def test_generated_load_report_cannot_promote_legacy_curve(self):
        report = json.loads(
            (ROOT / "out" / "reports" / "loads.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(report["schema"], "machine-loads/v2")
        self.assertTrue(report["static_sizing_pass"])
        self.assertFalse(report["production_authorized"])
        self.assertEqual(
            report["motors"]["m2"],
            "Leadshine CS-M21708 closed-loop NEMA17",
        )
        selected = report["m2"]["governing_selected_authority"]
        self.assertEqual(selected["role"], "governing_selected_M2_authority")
        legacy = report["m2"]["legacy_baseline"]
        self.assertTrue(legacy["non_governing"])
        self.assertEqual(legacy["role"], "historical_non_governing_baseline")
        self.assertIn("McMaster 6627T421", legacy["motor"])
        self.assertNotIn("McMaster 6627T421", report["motors"]["m2"])
        gates = report["m2"]["release_gates"]
        self.assertTrue(gates["selected_static_curve_margin_ge_2x"])
        self.assertFalse(gates["driver_current_configuration_verified"])
        self.assertFalse(gates["installed_hot_dyno_verified"])
        self.assertFalse(gates["production_authorized"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
