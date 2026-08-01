"""Regression checks for the unmodified-upstream full-cycle authority."""

import hashlib
import json
import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import verify_cycle as verify  # noqa: E402
from traj import Timeline, load_events  # noqa: E402


CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
SERIAL_CAPTURE = (
    ROOT / "out" / "capture" / "upstream_serial_twin_raw.jsonl"
)
REPORT = ROOT / "out" / "reports" / "upstream_current_raw_cycle.json"
SETTINGS = ROOT / "out" / "settings.yml"
UPSTREAM_SOURCE = ROOT.parent / "winder" / "src" / "winding.py"


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class RawCycleAuthorityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = json.loads(REPORT.read_text(encoding="utf-8"))

    def test_report_is_bound_to_canonical_raw_capture(self):
        self.assertEqual(self.report["schema"],
                         "captured-cycle-verification/v2")
        self.assertFalse(self.report["passed"])
        self.assertEqual(self.report["status"], "FAIL")
        self.assertEqual(self.report["capture"]["controller_mode"],
                         "upstream")
        self.assertEqual(self.report["capture"]["sha256"], sha256(CAPTURE))
        self.assertEqual(
            self.report["source_hashes"]["out/settings.yml"],
            sha256(SETTINGS),
        )
        self.assertEqual(
            self.report["source_hashes"]["sim/verify_cycle.py"],
            sha256(HERE / "verify_cycle.py"),
        )
        self.assertEqual(
            self.report["upstream_source"]["source_sha256"],
            sha256(UPSTREAM_SOURCE),
        )
        self.assertEqual(
            self.report["report_sha256"], verify._canonical_hash(self.report),
        )
        self.assertTrue(
            self.report["upstream_source"]["same_clean_commit_as_capture"]
        )

    def test_all_24_passes_have_50_physical_turns_and_full_m0_span(self):
        rows = self.report["raw_winding_progression"]
        self.assertEqual(len(rows), 24)
        self.assertEqual({row["completed_turns"] for row in rows}, {50})
        self.assertTrue(all(row["ok"] for row in rows))
        self.assertAlmostEqual(min(row["m0_min_rad"] for row in rows),
                               -61.918, places=3)
        self.assertAlmostEqual(max(row["m0_max_rad"] for row in rows),
                               -56.8, places=3)

    def test_exact_raw_shaft_wraps_complete_and_discrepancy_is_explicit(self):
        wraps = self.report["shaft_wraps"]
        self.assertEqual([row["index"] for row in wraps], [1, 2])
        self.assertTrue(all(row["raw_target_motion_complete"]
                            for row in wraps))
        self.assertFalse(any(row["exactly_two_physical_turns"]
                             for row in wraps))
        self.assertFalse(any(row["ok"] for row in wraps))
        self.assertAlmostEqual(
            wraps[0]["source_model_turns"], 1.375, places=9,
        )
        self.assertAlmostEqual(
            wraps[1]["source_model_turns"], 2.791666667, places=9,
        )
        self.assertAlmostEqual(
            wraps[0]["controller_effective_turns"],
            1.3749395533708841,
            places=12,
        )
        self.assertAlmostEqual(
            wraps[1]["controller_effective_turns"],
            2.791736856774936,
            places=12,
        )
        self.assertEqual(
            [row["previous_phase_last_tooth_index"] for row in wraps],
            [15, 19],
        )
        self.assertEqual(
            [row["next_phase_branch"] for row in wraps],
            ["clockwise", "counterclockwise"],
        )
        self.assertEqual(
            [row["serial_command"] for row in wraps],
            ["M1A-12.566", "M1A0.0"],
        )
        self.assertTrue(all(
            row["source_formula"]["matches_capture"]
            and row["absolute_target_formula_matches_capture"]
            and row["arrival_margin_s"] > 0.0
            for row in wraps
        ))
        self.assertAlmostEqual(
            wraps[0]["source_formula"]["bookkeeping_zero_rad"],
            0.0,
            places=8,
        )
        self.assertAlmostEqual(
            wraps[1]["source_formula"]["bookkeeping_zero_rad"],
            -4.0 * math.pi,
            places=8,
        )
        ids = {row["id"] for row in
               self.report["requirements_discrepancies"]}
        self.assertIn("shaft-wrap-relative-two-turn-claim", ids)
        self.assertFalse(self.report["checks"][
            "both raw shaft-wrap intervals execute exactly two M1 turns"]["ok"])
        self.assertTrue(self.report["checks"][
            "raw shaft-wrap absolute-target formula matches command stream"
        ]["ok"])
        self.assertTrue(self.report["checks"][
            "controller-effective wrap travel differs only by serial quantization"
        ]["ok"])

        diagnostic = self.report["shaft_wrap_diagnostic"]
        self.assertEqual(
            diagnostic["classification"],
            "GENUINE_UPSTREAM_ABSOLUTE_TARGET_CONTRADICTION",
        )
        self.assertFalse(diagnostic["absolute_angle_unwrap_defect"])
        self.assertFalse(
            diagnostic["upstream_motion_logic_modified_by_verifier"]
        )

    def test_serial_twin_independently_matches_controller_effective_turns(self):
        events = load_events(SERIAL_CAPTURE)
        physical_events = verify._controller_effective_events(events)
        requested_events = []
        for event in events:
            row = dict(event)
            if row.get("e") == "cmd":
                row["model_target"] = row.get(
                    "requested_model_target", row["model_target"]
                )
            requested_events.append(row)
        wraps = verify._raw_shaft_wraps(
            physical_events,
            Timeline(physical_events),
            source_timeline=Timeline(requested_events),
        )
        self.assertEqual(len(wraps), 2)
        self.assertTrue(all(
            row["interpretation_verdict"]
            == "GENUINE_UPSTREAM_ABSOLUTE_TARGET_CONTRADICTION"
            for row in wraps
        ))
        self.assertEqual(
            [row["controller_effective_turns"] for row in wraps],
            [
                row["controller_effective_turns"]
                for row in self.report["shaft_wraps"]
            ],
        )

    def test_controller_inverse_mapping_applies_direction_gear_and_rounding(self):
        meta = {
            "directions": [False, True, True, False],
            "m2_gear_ratio": 2.0,
        }
        self.assertEqual(
            verify._controller_effective_model_target(
                {"m": 0, "controller_target": 4.25}, meta,
            ),
            -4.25,
        )
        self.assertEqual(
            verify._controller_effective_model_target(
                {"m": 1, "controller_target": -12.566}, meta,
            ),
            -12.566,
        )
        self.assertEqual(
            verify._controller_effective_model_target(
                {"m": 2, "controller_target": 7.0}, meta,
            ),
            3.5,
        )

    def test_adapter_only_packing_markers_are_not_used_as_raw_evidence(self):
        proof = self.report["packing_phase_proof"]
        self.assertFalse(proof["applicable"])
        self.assertTrue(self.report["checks"][
            "all raw passes complete 50 turns across the full M0 span"]["ok"])


if __name__ == "__main__":
    unittest.main()
