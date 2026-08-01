"""DoD #5 uses analytical sizing; post-purchase motion stays fail-closed."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import report  # noqa: E402


def by_label(checks, label):
    return next(row for row in checks if row["label"] == label)


class SelectedM2DefinitionOfDoneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.loads = json.loads(
            (ROOT / "out" / "reports" / "loads.json").read_text(
                encoding="utf-8"
            )
        )

    def test_static_selected_sizing_passes_and_motion_proof_stays_separate(self):
        checks = report._loads_gate(self.loads, None)
        self.assertTrue(by_label(
            checks, "loads report uses selected-authority schema"
        )["ok"])
        self.assertTrue(by_label(
            checks,
            "selected motor and transmission load margins are at least 2x",
        )["ok"])
        self.assertTrue(by_label(
            checks, "DoD #5 uses the selected Leadshine M2 stack"
        )["ok"])
        self.assertTrue(by_label(
            checks, "retired McMaster M2 curve is non-governing"
        )["ok"])
        self.assertTrue(by_label(
            checks,
            "DoD #5 analytical loads authority passes",
        )["ok"])
        self.assertTrue(by_label(
            checks,
            "post-purchase M2 production qualification is separate and fail-closed",
        )["ok"])
        self.assertFalse(self.loads["production_authorized"])
        self.assertEqual(
            self.loads["post_purchase_motion_qualification"]["status"],
            "BLOCKED",
        )

    def test_legacy_motor_cannot_substitute_for_selected_stack(self):
        tampered = deepcopy(self.loads)
        tampered["motors"]["m2"] = (
            "NEMA17 McMaster 6627T421 encoder motor @24V (M2)"
        )
        tampered["m2"]["governing_selected_authority"] = deepcopy(
            tampered["m2"]["legacy_baseline"]
        )
        checks = report._loads_gate(tampered, None)
        self.assertFalse(by_label(
            checks, "DoD #5 uses the selected Leadshine M2 stack"
        )["ok"])
        self.assertFalse(by_label(
            checks, "M2 selection satisfies GOAL NEMA17 constraint"
        )["ok"])
        self.assertFalse(by_label(
            checks, "retired McMaster M2 curve is non-governing"
        )["ok"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
