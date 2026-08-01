"""Regression tests for the focused counterweight review sources."""

from __future__ import annotations

import unittest

import counterweight_attachment_review as review


class CounterweightAttachmentReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.full = review.gen_step()
        cls.section = review.gen_section_step()

    def test_focused_review_uses_current_six_stack_occurrences(self):
        labels = {child.label for child in self.full.children}
        self.assertIn(review.ARM_CONTEXT_LABEL, labels)
        self.assertIn(review.SHAFT_CONTEXT_LABEL, labels)
        self.assertTrue(
            set(review.COUNTERWEIGHT_OCCURRENCE_LABELS).issubset(labels)
        )
        self.assertTrue(set(review.COLLAR_CONTEXT_LABELS).issubset(labels))
        self.assertNotIn("counterweight_m3_insert", labels)
        self.assertEqual(len(review.REAR_M3_OCCURRENCE_LABELS), 16)
        self.assertEqual(len(review.FRONT_M2_OCCURRENCE_LABELS), 8)

    def test_section_is_real_rear_stack_positive_x_axis_half(self):
        bounds = self.section.bounding_box()
        self.assertAlmostEqual(
            float(bounds.min.X), review.SECTION_AXIS_X_MM, places=6
        )
        self.assertGreater(float(bounds.max.X), review.SECTION_AXIS_X_MM)
        self.assertEqual(len(self.section.children), 5)
        labels = {child.label for child in self.section.children}
        expected = {
            f"{review.ARM_CONTEXT_LABEL}_positive_x_axis_half_section",
            *(
                f"{label}_positive_x_axis_half_section"
                for label in review.SECTION_OCCURRENCE_LABELS
            ),
        }
        self.assertEqual(labels, expected)
        self.assertTrue(
            all(float(child.volume) > 0.0 for child in self.section.children)
        )

    def test_section_stack_has_closed_positive_material_contract(self):
        audit = review.attachment_audit()
        self.assertEqual(audit["stack_count"], 6)
        self.assertTrue(
            audit[
                "all_six_screws_terminate_in_positive_printed_material"
            ]
        )
        self.assertFalse(audit["any_balance_fastener_over_open_air"])
        selected = next(
            row
            for row in audit["rear_M3_retained_stacks"]["stacks"]
            if row["id"] == review.SECTION_STACK_ID
        )
        self.assertTrue(
            selected["fastener_terminates_in_positive_blind_material"]
        )
        self.assertGreaterEqual(
            selected["blind_positive_material_ahead_of_tip_mm"], 1.8
        )
        self.assertIn(
            "positive 1 mm arm floor",
            selected["closed_structural_load_path"],
        )
        self.assertIn(
            "blind printed cap", selected["closed_structural_load_path"]
        )


if __name__ == "__main__":
    unittest.main()
