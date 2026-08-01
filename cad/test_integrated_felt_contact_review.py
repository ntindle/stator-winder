"""Regression tests for actual and maximum-wire wool-felt contact panels."""

from __future__ import annotations

import unittest

import integrated_felt_contact_review as felt


class IntegratedFeltContactReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = felt.analyze()

    def test_all_review_geometry_checks_pass(self) -> None:
        self.assertEqual(self.report["status"], "PASS_REVIEW_ONLY")
        self.assertTrue(all(self.report["checks"].values()))
        self.assertFalse(self.report["production_authorized"])

    def test_snapshot_review_cannot_reuse_pre_step_renders(self) -> None:
        reviewed = self.report["visual_review"]["reviewed"]
        packet = self.report["visual_review"]["snapshot_packet"]
        self.assertEqual(len(packet), 3 if reviewed else 0)
        self.assertEqual(
            self.report["release_gates"][
                "mandatory_tight_snapshot_packet_reviewed"
            ],
            reviewed,
        )
        for relative in packet:
            self.assertGreaterEqual(
                (felt.ROOT / relative).stat().st_mtime_ns,
                felt.STEP_OUT.stat().st_mtime_ns,
            )

    def test_actual_job_wire_is_tangent_without_felt_intersection(self) -> None:
        exact = self.report["exact_BREP"]
        self.assertLessEqual(exact["actual_wire_to_fixed_felt_distance_mm"], 1e-7)
        self.assertLessEqual(exact["actual_wire_to_moving_felt_distance_mm"], 1e-7)
        self.assertLessEqual(exact["actual_wire_to_fixed_felt_overlap_mm3"], 1e-8)
        self.assertLessEqual(exact["actual_wire_to_moving_felt_overlap_mm3"], 1e-8)
        self.assertAlmostEqual(self.report["actual_pad_gap_mm"], 0.22352, places=8)

    def test_maximum_wire_changeover_is_separate_and_tangent(self) -> None:
        exact = self.report["exact_BREP"]
        self.assertLessEqual(exact["max_wire_to_fixed_felt_distance_mm"], 1e-7)
        self.assertLessEqual(exact["max_wire_to_moving_felt_distance_mm"], 1e-7)
        self.assertLessEqual(exact["max_wire_to_fixed_felt_overlap_mm3"], 1e-8)
        self.assertLessEqual(exact["max_wire_to_moving_felt_overlap_mm3"], 1e-8)
        self.assertAlmostEqual(
            self.report["maximum_changeover_pad_gap_mm"], 0.5, places=8
        )

    def test_wool_felt_labels_and_cross_section_are_explicit(self) -> None:
        parts = felt.contact_parts()
        for key in ("actual_fixed", "actual_moving", "max_fixed", "max_moving"):
            self.assertIn("wool_felt", str(parts[key].label))
        sections = felt._section_parts(parts)
        self.assertEqual(len(sections), 3)
        self.assertTrue(all(float(shape.volume) > 0.0 for shape in sections))

    def test_static_sizing_passes_and_pull_gauge_calibration_remains_open(self) -> None:
        gates = self.report["release_gates"]
        self.assertTrue(gates["review_geometry"])
        self.assertTrue(gates["preload_spring_and_drag_sizing_PASS"])
        self.assertTrue(
            gates["actual_and_0p5mm_changeover_contact_geometry_PASS"]
        )
        self.assertFalse(gates["operating_drag_pull_gauge_calibrated"])
        self.assertEqual(
            self.report["felt_preload_sizing"]["selected_spring"],
            "McMaster 94125K614",
        )
        self.assertFalse(gates["production_authorized"])

    def test_report_hash_is_self_consistent(self) -> None:
        felt.validate_report_integrity(self.report)


if __name__ == "__main__":
    unittest.main()
