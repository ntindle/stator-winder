"""Regression gates for the fail-closed M2 drive successor review."""

from __future__ import annotations

import math
import unittest

import m2_drive_successor_review as review


class M2DriveSuccessorReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = review.run_audit()

    def test_frozen_exact_one_to_one_contract(self):
        drive = self.report["candidate"]["drive"]
        self.assertEqual(drive["teeth_each"], 30)
        self.assertEqual(drive["motor_teeth"], 30)
        self.assertEqual(drive["flyer_teeth"], 30)
        self.assertEqual(drive["ratio"], 1.0)
        self.assertTrue(self.report["geometry"]["checks"]["exact_1_to_1_ratio"])

    def test_standard_210mm_loop_is_exact_at_60mm_centres(self):
        drive = self.report["candidate"]["drive"]
        self.assertTrue(math.isclose(
            drive["calculated_pitch_length_mm"], 210.0, abs_tol=1.0e-9
        ))
        self.assertTrue(
            self.report["geometry"]["checks"]["exact_210mm_pitch_length"]
        )

    def test_corrected_nbk_capacity_applies_length_factor(self):
        drive = self.report["candidate"]["drive"]
        self.assertTrue(math.isclose(
            drive["allowable_torque_300rpm_nm"], 2.06 * 0.9,
            abs_tol=1.0e-9,
        ))
        self.assertTrue(
            self.report["geometry"]["checks"][
                "pulley_capacity_ge_2x_requirement"
            ]
        )

    def test_every_review_body_is_valid_and_core_parts_are_one_solid(self):
        parts = review.review_parts()
        for name, part in parts.items():
            with self.subTest(name=name):
                self.assertGreater(part.volume, 0.0)
                self.assertTrue(part.is_valid)
        for name in ("mount", "belt", "motor_pulley", "flyer_pulley"):
            with self.subTest(name=name):
                self.assertEqual(len(parts[name].solids()), 1)

    def test_no_unintended_wire_or_drive_overlap(self):
        overlaps = self.report["geometry"]["unintended_overlaps_mm3"]
        for pair, volume in overlaps.items():
            with self.subTest(pair=pair):
                self.assertLessEqual(volume, review.BOOLEAN_TOL_MM3)

    def test_all_reported_running_clearances_are_at_least_two_mm(self):
        clearances = self.report["geometry"]["clearances_mm"]
        for pair, clearance in clearances.items():
            with self.subTest(pair=pair):
                self.assertGreaterEqual(
                    clearance, review.MIN_RUNNING_CLEARANCE_MM - 1.0e-6
                )
        self.assertGreaterEqual(
            self.report["geometry"]["minimum_running_clearance_mm"], 2.0
        )

    def test_belt_has_positive_contact_with_both_pulleys(self):
        contacts = self.report["geometry"]["intended_contacts_mm3"]
        self.assertGreater(contacts["belt_vs_motor_pulley_mm3"], 0.0)
        self.assertGreater(contacts["belt_vs_flyer_pulley_mm3"], 0.0)

    def test_asymmetric_supplier_belt_envelope_is_bound(self):
        drive = self.report["candidate"]["drive"]
        self.assertTrue(math.isclose(
            drive["belt_inward_from_pitch_line_mm"], 1.520,
            abs_tol=1.0e-9,
        ))
        self.assertTrue(math.isclose(
            drive["belt_outward_from_pitch_line_mm"], 0.890,
            abs_tol=1.0e-9,
        ))
        self.assertTrue(
            self.report["geometry"]["checks"][
                "supplier_belt_envelope_is_asymmetric_about_pitch_line"
            ]
        )

    def test_flyer_clamp_screws_are_accessible_behind_belt_channel(self):
        geometry = self.report["geometry"]
        self.assertTrue(
            geometry["checks"][
                "custom_flyer_pulley_has_two_positive_clamp_screws"
            ]
        )
        self.assertLessEqual(
            geometry["unintended_overlaps_mm3"][
                "flyer_set_screws_vs_pulley_max_mm3"
            ],
            review.BOOLEAN_TOL_MM3,
        )
        self.assertGreaterEqual(
            geometry["clearances_mm"][
                "belt_to_closest_flyer_set_screw_mm"
            ],
            review.MIN_RUNNING_CLEARANCE_MM,
        )

    def test_geometry_summary_matches_positive_geometry_checks(self):
        self.assertEqual(
            self.report["geometry_passed"],
            all(self.report["geometry"]["checks"].values()),
        )
        self.assertTrue(self.report["geometry_passed"])

    def test_motor_gate_remains_fail_closed(self):
        self.assertFalse(self.report["motor_running_torque_gate_passed"])
        self.assertFalse(
            self.report["flyer_pulley_retention_torque_gate_passed"]
        )
        self.assertFalse(self.report["production_authorized"])
        self.assertIsNone(
            self.report["candidate"]["motor"][
                "available_torque_at_300rpm_nm"
            ]
        )

    def test_stock_clamp_pulley_interface_remains_fail_closed(self):
        self.assertFalse(
            self.report["motor_pulley_vendor_cad_gate_passed"]
        )
        self.assertFalse(
            self.report["motor_pulley_to_motor_shaft_gate_passed"]
        )
        clamp = self.report["candidate"]["drive"][
            "stock_motor_pulley_clamp"
        ]
        self.assertEqual(clamp["bolt_size"], "M2")
        self.assertTrue(math.isclose(
            clamp["tightening_torque_nm"], 0.5, abs_tol=1.0e-9
        ))
        self.assertTrue(math.isclose(
            clamp["published_inertia_kgm2"], 3.0e-6,
            abs_tol=1.0e-12,
        ))
        self.assertFalse(clamp["exact_vendor_CAD_gate"])
        self.assertFalse(clamp["selected_motor_D_cut_interface_gate"])


if __name__ == "__main__":
    unittest.main()
