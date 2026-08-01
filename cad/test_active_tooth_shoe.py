"""Regression checks for the isolated active-tooth shoe review model."""

from pathlib import Path
import unittest

import active_tooth_shoe as shoe


class ActiveToothShoeCadTests(unittest.TestCase):
    def test_controlled_dimensions_meet_concept_inputs(self):
        self.assertLessEqual(
            shoe.BLADE_THICKNESS_MM, shoe.MAXIMUM_BLADE_THICKNESS_MM
        )
        self.assertGreaterEqual(shoe.HORN_CENTERLINE_RADIUS_MM, 3.0)
        self.assertGreaterEqual(shoe.MINIMUM_AXIAL_PROJECTION_MM, 3.0)
        self.assertGreaterEqual(
            shoe.RADIAL_WORKING_SPAN_MM, 6.516099494243948 - 1e-12
        )
        self.assertEqual(shoe.MOUNT_DATUM_A_Z_MM, 8.0)
        self.assertEqual(shoe.MOUNT_HOLE_DIAMETER_MM, 3.4)

    def test_common_rigid_corridor_fails_closed(self):
        report = shoe.corridor_summary()
        self.assertEqual(report["status"], "FAIL")
        self.assertEqual(report["failing_station_count"], 37)
        self.assertLess(report["minimum_common_corridor_margin_mm"], -0.16)

    def test_review_model_is_split_and_not_integrated(self):
        labels = {part.label for part in shoe.shoe_parts()}
        self.assertEqual(labels, {
            "dielectric_blade_left_slot",
            "dielectric_blade_right_slot",
            "carrier_left_slot",
            "carrier_right_slot",
        })
        assembly_source = (
            Path(__file__).resolve().parent / "assembly.py"
        ).read_text()
        self.assertNotIn("active_tooth_shoe", assembly_source)

    def test_review_assembly_has_stator_plus_four_shoe_occurrences(self):
        model = shoe.gen_step()
        self.assertEqual(len(model.children), 5)
        self.assertEqual(model.label, "active_tooth_shoe_no_go_review")


if __name__ == "__main__":
    unittest.main()

