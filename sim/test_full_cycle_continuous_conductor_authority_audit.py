from __future__ import annotations

import json
import math
from pathlib import Path
import tempfile
import unittest

import full_cycle_continuous_conductor_authority_audit as audit
from traj import Timeline


class FullCycleContinuousConductorAuthorityAuditTests(unittest.TestCase):
    def test_current_normal_goal_cycle_is_completely_classified_but_fail_closed(
        self,
    ) -> None:
        report = audit.analyze()

        self.assertEqual(report["schema"], audit.SCHEMA)
        self.assertEqual(report["status"], "FAIL")
        self.assertIs(report["production_authorized"], False)
        self.assertEqual(
            report["decision"],
            "FULL_CYCLE_CONDUCTOR_NOT_PROVEN_FAIL_CLOSED",
        )
        self.assertIs(
            report["coverage_result"][
                "presentation_timeline_fully_classified"
            ],
            True,
        )
        self.assertEqual(
            report["coverage_result"][
                "physically_authorized_continuous_interval_count"
            ],
            48,
        )
        self.assertAlmostEqual(
            report["coverage_result"][
                "physically_authorized_continuous_interval_duration_s"
            ],
            1.1433629280001538,
            places=7,
        )
        self.assertGreater(
            report["coverage_result"][
                "physically_authorized_timeline_fraction"
            ],
            0.0,
        )

        capture = report["capture_evidence"]
        self.assertEqual(capture["winding_pass_count"], 24)
        self.assertEqual(len(capture["shaft_wraps"]), 2)
        self.assertTrue(all(
            math.isclose(observed, expected, abs_tol=1.0e-9)
            for observed, expected in zip(
                [row["turns"] for row in capture["shaft_wraps"]],
                [1.375, 2.7916666666666665],
            )
        ))
        self.assertIs(
            capture["gates"]["both_shaft_wraps_exactly_two_turns"],
            False,
        )

        loci = report["deposition_locus_evidence"]
        self.assertEqual(loci["locus_count"], 2400)
        self.assertIs(loci["gates"]["all_2400_pass_state_keys"], True)
        self.assertIs(loci["gates"]["axes_match_raw_timeline"], True)
        self.assertIs(loci["gates"]["segment_order_and_seams"], True)
        self.assertIs(loci["continuous_interpolation_authorized"], False)

        cap_entry = report["current_cap_entry_evidence"]
        self.assertEqual(cap_entry["status"], "PASS")
        self.assertIs(
            cap_entry[
                "retired_direct_chord_cap_endpoint_check_applicable"
            ],
            False,
        )
        self.assertEqual(cap_entry["current_route_cap_entry_kink_count"], 0)
        self.assertGreaterEqual(
            cap_entry["minimum_named_guide_wire_center_radius_mm"], 3.0
        )

        held = report["stationary_locus_interval_evidence"]
        self.assertEqual(held["authorized_interval_count"], 48)
        self.assertAlmostEqual(
            held["authorized_duration_s"], 1.1433629280001538, places=7
        )
        self.assertTrue(
            held["gates"]
            ["every_admitted_interval_matches_exact_same_pass_locus"]
        )
        self.assertFalse(
            held["gates"]["moving_between_locus_route_family_proven"]
        )

        presentation = report["presentation_timeline_evidence"]
        self.assertEqual(presentation["kind_counts"], {
            "final_hold": 1,
            "from_shaft_wrap": 2,
            "initial_hold": 1,
            "shaft_wrap": 2,
            "to_shaft_wrap": 2,
            "tooth_transition": 21,
            "winding_half_turn": 2400,
        })
        self.assertIs(
            presentation["gates"]["presentation_partition_valid"],
            True,
        )
        self.assertIs(
            presentation["gates"][
                "static_supply_to_flyer_bore_seam_exact"
            ],
            True,
        )
        self.assertEqual(
            presentation["wire_handoff_contract"][
                "static_to_flyer_seam_local_mm"
            ],
            [0.0, 0.0, -42.0],
        )
        self.assertIs(
            presentation["gates"][
                "presentation_is_physical_interval_authority"
            ],
            False,
        )

        matrix = report["required_state_matrix"]
        self.assertIs(
            matrix["load_and_initial_lead_capture"][
                "fixed_supply_to_flyer_bore_bound"
            ],
            True,
        )
        self.assertIs(matrix["between_locus_and_m0_motion"]["proven"], False)
        self.assertEqual(
            matrix["between_locus_and_m0_motion"]
            ["authorized_constant_route_interval_count"],
            48,
        )
        self.assertIs(matrix["park_for_shaft_wrap"]["proven"], False)
        self.assertIs(matrix["tooth_and_phase_index"]["proven"], False)
        self.assertIs(matrix["shaft_wrap"]["proven"], False)
        self.assertIs(
            matrix["unload_and_final_lead_state"]["proven"],
            False,
        )
        self.assertEqual(
            report["report_sha256"],
            audit._canonical_hash(report, "report_sha256"),
        )

    def test_wrap_inference_uses_actual_feedback_position_and_accepts_two_turns(
        self,
    ) -> None:
        events = [
            {
                "e": "meta",
                "t": 0.0,
                "velocities": [20.0, 20.0, 20.0, 5.0],
            },
            {"e": "wind_wire_around_shaft", "t": 1.0, "args": [1]},
            {
                "e": "cmd",
                "t": 1.0,
                "m": 1,
                "a": 4.0 * math.pi,
                "model_target": 4.0 * math.pi,
            },
            {"e": "wind_wire_around_shaft_done", "t": 2.0},
            {"e": "wind_wire_around_shaft", "t": 3.0, "args": [2]},
            {
                "e": "cmd",
                "t": 3.0,
                "m": 1,
                "a": 0.0,
                "model_target": 0.0,
            },
            {"e": "wind_wire_around_shaft_done", "t": 4.0},
        ]
        timeline = Timeline(events)

        rows = audit._infer_shaft_wraps(events, timeline)

        self.assertEqual(len(rows), 2)
        self.assertTrue(all(
            row["valid_marker_and_command_contract"] for row in rows
        ))
        self.assertTrue(all(row["arrives_before_done_marker"] for row in rows))
        self.assertTrue(all(
            math.isclose(row["turns"], 2.0, abs_tol=1.0e-12)
            for row in rows
        ))

    def test_missing_parameterized_capture_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            tmp_path = Path(temp_dir)
            report = audit.analyze(
                capture_path=tmp_path / "future_capture.jsonl"
            )

        self.assertEqual(report["status"], "FAIL")
        self.assertEqual(report["audit_integrity_status"], "FAIL")
        self.assertIs(report["production_authorized"], False)
        self.assertTrue(any(
            issue["code"] == "capture_missing"
            for issue in report["issues"]
        ))
        self.assertEqual(
            report["report_sha256"],
            audit._canonical_hash(report, "report_sha256"),
        )

    def test_report_writer_preserves_self_hash(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            tmp_path = Path(temp_dir)
            report = audit._empty_report(
                {"capture": tmp_path / "missing.jsonl"},
                [{
                    "severity": "INTEGRITY_FAIL",
                    "code": "fixture",
                    "message": "x",
                }],
            )
            output = tmp_path / "audit.json"
            markdown = tmp_path / "audit.md"

            audit.write_report(report, output)
            audit.write_markdown(report, markdown)
            loaded = json.loads(output.read_text(encoding="utf-8"))
            rendered = markdown.read_text(encoding="utf-8")

        self.assertEqual(
            loaded["report_sha256"],
            audit._canonical_hash(loaded, "report_sha256"),
        )
        self.assertIn("**Release truth: FAIL**", rendered)
        self.assertIn(report["report_sha256"], rendered)


if __name__ == "__main__":
    unittest.main(verbosity=2)
