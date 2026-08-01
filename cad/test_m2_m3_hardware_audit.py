"""Regression tests for the isolated exact M2/M3 hardware audit."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import m2_m3_hardware_audit as audit


class M2M3HardwareAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = audit.run_audit()
        cls.evidence = cls.result["evidence"]

    def test_all_candidate_repairs_pass_exact_checks(self):
        failed = [c for c in self.result["checks"] if not c["passed"]]
        self.assertEqual(failed, [])
        self.assertTrue(self.result["passed"])

    def test_m2_low_heads_are_clear_in_integrated_production_geometry(self):
        rows = {r["label"]: r for r in self.evidence["m2_mount_screws"]}
        for side in ("L", "R"):
            row = rows[f"m2_mount_{side}_low_m5x12"]
            self.assertAlmostEqual(row["before_mm3"], 0.0, places=6)
            self.assertAlmostEqual(row["after_mm3"], 0.0, places=6)
        self.assertEqual(self.evidence["m2_mount_candidate"]["solids"], 1)

    def test_flush_base_screws_clear_full_dancer_stop_sweep(self):
        rows = self.evidence["dancer_arm_vs_base_screws"]
        for row in rows:
            self.assertAlmostEqual(row["current_max_overlap_mm3"], 0.0,
                                   places=6)
            self.assertAlmostEqual(row["candidate_max_overlap_mm3"], 0.0,
                                   places=6)
            self.assertGreaterEqual(row["candidate_min_distance_mm"], 0.999)
        candidate = self.evidence["entry_bracket_candidate"]
        self.assertEqual(candidate["stop_angles_unchanged_deg"], [-3.0, 5.5])

    def test_entry_notch_and_flush_anchor_clear_moving_hardware(self):
        rows = self.evidence["entry_bracket_vs_moving_hardware"]
        current = {r["label"]: r["current_max_overlap_mm3"] for r in rows}
        self.assertAlmostEqual(current["dancer_pulley_nyloc_m2p5"], 0.0,
                               places=6)
        self.assertAlmostEqual(
            current["dancer_spring_moving_m2x16_flush"], 0.0, places=6)
        for row in rows:
            self.assertAlmostEqual(row["candidate_max_overlap_mm3"], 0.0,
                                   places=6)
        anchor = self.evidence["dancer_moving_anchor_candidate"]
        self.assertEqual(anchor["arm_solids"], 1)
        self.assertAlmostEqual(anchor["screw_arm_overlap_mm3"], 0.0,
                               places=6)

    def test_felt_and_counterweight_stack_repairs(self):
        felt = self.evidence["felt_stack"][-1]
        self.assertGreater(felt["thread_proud_mm"], 5.0)
        self.assertGreater(felt["candidate_m4x55_thread_proud_mm"], 5.0)
        self.assertGreaterEqual(felt["candidate_m4x55_wire_gap_mm"], 3.39)
        self.assertGreaterEqual(felt["candidate_m4x55_belt_gap_mm"], 37.7)
        counterweight = self.evidence["counterweight_stack"]
        self.assertEqual(len(counterweight), 4)
        self.assertTrue(all(
            row["stack_id"] == "rear_right" for row in counterweight
        ))
        self.assertTrue(all(
            row["distance_mm"] <= 1.0e-6 for row in counterweight
        ))

    def test_counterweight_is_volumetrically_attached_and_clear(self):
        attachment = self.evidence["counterweight_attachment"]
        self.assertEqual(attachment["flyer_arm_solids"], 1)
        self.assertEqual(attachment["serialized_occurrence_count"], 24)
        self.assertEqual(attachment["rear_M3_stack_count"], 4)
        self.assertEqual(attachment["front_M2_stack_count"], 2)
        self.assertEqual(len(attachment["rear_M3_occurrences"]), 16)
        self.assertEqual(len(attachment["front_M2_occurrences"]), 8)
        self.assertTrue(
            attachment[
                "all_six_screws_terminate_in_positive_printed_material"
            ]
        )
        self.assertFalse(attachment["any_balance_fastener_over_open_air"])
        self.assertFalse(attachment["physical_pull_proof_complete"])
        self.assertAlmostEqual(
            attachment["minimum_rear_insert_engagement_mm"], 4.3,
            places=6,
        )
        self.assertGreaterEqual(
            attachment["minimum_rear_blind_positive_material_mm"], 1.8
        )
        self.assertGreaterEqual(
            attachment["minimum_front_insert_engagement_mm"], 4.0
        )
        self.assertGreaterEqual(
            attachment["minimum_front_screw_tip_clearance_mm"], 0.5
        )
        self.assertGreaterEqual(
            attachment["minimum_front_blind_positive_material_mm"], 2.4
        )
        self.assertTrue(
            attachment["rear_M3_retained_stacks"][
                "all_caps_and_posts_single_solid"
            ]
        )

    def test_heat_set_embeds_are_the_only_intended_positive_volume_fits(self):
        rows = self.evidence["flyer_heat_set_inserts"]
        self.assertEqual(len(rows), 4)
        self.assertTrue(all(r["interference_embed_mm3"] > 4.0 for r in rows))
        running = {row["pair"]: row for row in self.result["intended_contacts"]}
        self.assertEqual(running[
            "M2 inner-race spacer / outer-race spacer"]["class"],
            "running clearance")
        self.assertEqual(running[
            "flyer pulley clamp hardware / static flyer block"]["class"],
            "running clearance")


if __name__ == "__main__":
    unittest.main()
