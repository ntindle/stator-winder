"""Regression gates for the production-cap / retained-flyer wire audit."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import integrated_phase_aware_wire_path as audit  # noqa: E402


class IntegratedPhaseAwareWirePathTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = json.loads(
            audit.OUTPUT_JSON.read_text(encoding="utf-8")
        )
        audit.validate_report_integrity(cls.report)

    def test_authorities_are_explicit_and_never_conflated(self) -> None:
        authority = self.report["authority_model"]
        self.assertTrue(authority["raw_control_locus"]["authoritative"])
        self.assertTrue(authority["aggregate_copper"]["authoritative"])
        self.assertFalse(authority["exact_strand_packing"]["authoritative"])
        self.assertFalse(authority["exact_strand_packing"]["predicted"])
        self.assertEqual(
            authority["exact_strand_packing"]["classification"],
            audit.AUTH_EXACT_STRANDS,
        )
        self.assertTrue(
            self.report["aggregate_copper_occupancy"]["gates"]
            ["exact_strand_packing_not_promoted_to_authority"]
        )

    def test_raw_capture_covers_all_half_turns_starts_and_wrap_calls(self) -> None:
        raw = self.report["raw_capture"]
        self.assertEqual(raw["controller_mode"], "upstream")
        self.assertIsNone(raw["controller_adapter_sha256"])
        self.assertEqual(raw["pass_count"], 24)
        self.assertEqual(raw["half_turn_locus_count"], 2400)
        self.assertEqual(len(raw["coil_starts"]), 24)
        self.assertEqual(
            [row["pass_index"] for row in raw["coil_starts"]],
            list(range(24)),
        )
        self.assertEqual(raw["shaft_wrap_count"], 2)
        self.assertTrue(all(raw["gates"].values()))

    def test_retained_flyer_feed_is_physically_discontinuous(self) -> None:
        flyer = self.report["retained_flyer_shaft_to_tip"]
        self.assertEqual(flyer["status"], "FAIL")
        self.assertAlmostEqual(flyer["unmodeled_centerline_gap_mm"], 8.0)
        self.assertEqual(
            flyer["hollow_shaft_centerline_exit_mm"],
            [0.0, 0.0, -30.12],
        )
        self.assertEqual(
            flyer["visual_R3_witness_start_mm"],
            [0.0, 8.0, -30.12],
        )
        self.assertFalse(
            flyer["gates"]["shaft_axis_to_R3_witness_centerline_connected"]
        )
        self.assertFalse(
            flyer["gates"]["explicit_shaft_exit_turn_radius_ge_3mm"]
        )

    def test_visual_wire_has_exact_positive_overlap_with_forbidden_petg(self) -> None:
        flyer = self.report["retained_flyer_shaft_to_tip"]
        job = flyer["job_wire_overlap_witness"]
        launch = flyer["launch_max_wire_overlap_witness"]
        self.assertEqual(flyer["witness_distance_to_retained_arm_mm"], 0.0)
        self.assertTrue(job["inside_retained_arm"])
        self.assertTrue(job["positive_overlap"])
        self.assertGreater(job["OCC_positive_intersection_volume_mm3"], 0.5)
        self.assertTrue(launch["positive_overlap"])
        self.assertGreater(
            launch["OCC_positive_intersection_volume_mm3"],
            job["OCC_positive_intersection_volume_mm3"],
        )
        self.assertEqual(flyer["forbidden_contact"],
                         "retained_printed_arm_PETG")
        self.assertNotIn(
            "retained_printed_arm_PETG",
            self.report["allowed_contact_classes"],
        )

    def test_every_raw_tip_path_constructs_but_none_enters_cap_tangent(self) -> None:
        transfer = self.report["tip_to_active_PEEK_cap"]
        self.assertEqual(transfer["raw_locus_count"], 2400)
        self.assertEqual(transfer["constructed_locus_count"], 2400)
        self.assertEqual(transfer["unique_geometry_case_count"], 520)
        self.assertEqual(transfer["implicit_kink_locus_count"], 2400)
        self.assertGreater(transfer["minimum_cap_lane_tangent_error_deg"], 5.0)
        self.assertGreater(transfer["maximum_cap_lane_tangent_error_deg"], 60.0)
        self.assertAlmostEqual(
            transfer["minimum_tip_toroid_wire_center_radius_mm"],
            3.11176,
            places=6,
        )
        self.assertFalse(
            transfer["gates"]
            ["every_free_span_arrives_tangent_to_named_PEEK_lane"]
        )
        self.assertFalse(
            transfer["gates"]["no_implicit_sub_R3_cap_mouth_kink"]
        )
        worst = transfer["worst_cap_entry_witness"]
        self.assertEqual(worst["locus"]["pass_index"], 1)
        self.assertEqual(worst["locus"]["state_index"], 51)
        self.assertAlmostEqual(
            worst["cap_lane_tangent_error_deg"],
            transfer["maximum_cap_lane_tangent_error_deg"],
            places=9,
        )

    def test_terminal_span_core_class_is_independent_and_fails(self) -> None:
        transfer = self.report["tip_to_active_PEEK_cap"]
        self.assertEqual(transfer["core_crossing_locus_count"], 1000)
        self.assertFalse(
            transfer["gates"]
            ["no_raw_free_span_centerline_crosses_lamination_core"]
        )
        witness = transfer["first_core_crossing_witness"]
        self.assertEqual(witness["locus"]["pass_index"], 2)
        self.assertEqual(witness["locus"]["state_index"], 0)
        self.assertTrue(witness["core_prism"]["intersects"])
        self.assertGreater(
            witness["core_prism"]["projected_intersection_length_mm"],
            9.0,
        )
        # Aggregate lane support remains valid; it does not mask the terminal
        # free-span core witness.
        self.assertEqual(self.report["aggregate_copper_occupancy"]["status"],
                         "PASS")

    def test_raw_shaft_wrap_turn_counts_are_reported_not_normalized(self) -> None:
        shaft = self.report["shaft_wraps"]
        self.assertEqual(shaft["case_count"], 2)
        turns = [row["raw_turns"] for row in shaft["cases"]]
        self.assertAlmostEqual(turns[0], 1.375, places=8)
        self.assertAlmostEqual(turns[1], 2.7916666667, places=8)
        self.assertFalse(shaft["gates"]["each_raw_wrap_is_two_full_turns"])
        self.assertTrue(
            shaft["gates"]["tip_and_sleeve_contact_radii_ge_3mm"]
        )
        self.assertTrue(
            shaft["gates"]
            ["bare_core_clear_for_all_periodic_raw_M1_residues"]
        )
        self.assertFalse(
            shaft["gates"]["completed_phase_aggregate_clear_for_both_wraps"]
        )
        self.assertIn("changing the flyer cannot repair it",
                      self.report["non_geometric_contract_blocker"])
        provenance = self.report["upstream_regression_provenance"]
        self.assertEqual(
            provenance["canonical_current"]["commit"],
            "6039b33c8f15a20086c2195c3f2d02b3a833e8ca",
        )
        self.assertEqual(
            provenance["last_known_exact_turn_formulation"]["commit"],
            "8ae82f9e9ebf8cba7afe48e75e5d255d96bdfe3f",
        )
        self.assertEqual(
            provenance["policy"],
            "DO_NOT_PATCH_OR_FORK_UPSTREAM_IN_THIS_AUDIT",
        )

    def test_report_is_fail_closed_and_bound_to_current_sources(self) -> None:
        self.assertEqual(self.report["status"], "FAIL")
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["assembly_integration_authorized"])
        self.assertTrue(self.report["exact_failure_witnesses"])
        self.assertEqual(
            {row["id"] for row in self.report["exact_failure_witnesses"]},
            {
                "W1_SHAFT_TO_WITNESS_DISCONTINUITY",
                "W2_FORBIDDEN_PETG_POSITIVE_OVERLAP",
                "W3_WORST_CAP_ENTRY_TANGENT_KINK",
                "W4_TERMINAL_SPAN_CORE_CROSSING",
                "W5_RAW_SHAFT_WRAP_TURN_COUNT_MISMATCH",
            },
        )
        self.assertTrue(
            self.report["release_gates"]["raw_control_authority_complete"]
        )
        self.assertTrue(
            self.report["release_gates"]
            ["aggregate_occupancy_authority_complete"]
        )
        self.assertFalse(
            self.report["release_gates"]
            ["hollow_shaft_to_tip_path_authorized"]
        )
        audit.validate_report_integrity(self.report)

    def test_self_hash_rejects_tampering(self) -> None:
        tampered = deepcopy(self.report)
        tampered["status"] = "PASS"
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            audit.validate_report_integrity(tampered, check_sources=False)


if __name__ == "__main__":
    unittest.main(verbosity=2)
