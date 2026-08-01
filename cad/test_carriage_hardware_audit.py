"""Deterministic source-level tests for carriage_hardware_audit."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import carriage_endstop_flag  # noqa: E402
import carriage_hardware_audit as audit  # noqa: E402
import fabricated_carriage  # noqa: E402
from params import PARAMS as P  # noqa: E402


class CarriageHardwareBooleanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.results = audit.audit_results()
        cls.by_name = {result.check: result for result in cls.results}

    def test_all_current_faults_are_reproduced_and_corrected(self):
        self.assertEqual(len(self.results), 5)
        for result in self.results:
            with self.subTest(check=result.check):
                self.assertTrue(result.passed)
        baseline = {result.check: result.baseline_mm3 for result in self.results}
        self.assertGreater(baseline["four inner MGN screw heads vs tower"], 0)
        self.assertGreater(baseline["rear flag washer/nyloc vs flag"], 0)
        self.assertGreater(
            baseline["complete T8 set vs plate/front-side screws"], 0)
        self.assertGreater(baseline["endstop flag vs M0 fixed-end mount"], 0)

    def test_four_block_reliefs_have_print_allowance_and_minimum_roof(self):
        self.assertEqual(len(audit.BLOCK_HEAD_RELIEF_CENTERS_XZ), 4)
        self.assertAlmostEqual(audit.BLOCK_HEAD_RELIEF_D - 5.68, 0.72)
        self.assertAlmostEqual(6.0 - audit.BLOCK_HEAD_RELIEF_DEPTH, 2.75)
        self.assertGreaterEqual(6.0 - audit.BLOCK_HEAD_RELIEF_DEPTH,
                                P.min_wall)

    def test_corrected_candidate_parts_are_single_valid_solids(self):
        for factory in (audit.proposed_spindle_tower,
                        audit.proposed_nut_bracket,
                        audit.proposed_fixed_end_mount):
            with self.subTest(factory=factory.__name__):
                part = factory()
                self.assertTrue(part.is_valid)
                self.assertEqual(len(part.solids()), 1)
                self.assertGreater(part.volume, 0.0)

    def test_rear_stack_bears_below_flag_and_has_full_nyloc(self):
        hw = audit.proposed_hardware()
        for x in (-31.0, 31.0):
            washer = hw[f"tower_washer_m4_{x:+g}_+31"].bounding_box()
            nut = hw[f"tower_nyloc_m4_{x:+g}_+31"].bounding_box()
            screw = hw[f"tower_m4x25_{x:+g}_+31"].bounding_box()
            self.assertAlmostEqual(washer.max.Y,
                                   carriage_endstop_flag.FLAG_BOTTOM_Y)
            self.assertAlmostEqual(nut.max.Y,
                                   carriage_endstop_flag.FLAG_BOTTOM_Y - 0.9)
            self.assertAlmostEqual(
                carriage_endstop_flag.FLAG_BOTTOM_Y - screw.min.Y, 6.65,
            )

    def test_t8_m3x12_threaded_flange_stack_and_spring_clearance(self):
        engagement = 12.0 - 8.0 - audit.M3_WASHER_T
        self.assertAlmostEqual(engagement, 3.45)
        screw_tip_z = P.m0_home_standoff - 10.0 + audit.M3_WASHER_T - 12.0
        self.assertAlmostEqual(screw_tip_z, 73.55)
        self.assertGreater(screw_tip_z, P.m0_home_standoff - 22.0)

        bracket = audit.proposed_nut_bracket()
        hw = audit.proposed_hardware()
        for index in (1, 2, 3, 4):
            for label in (
                    f"t8_flange_m3x12_{index}",
                    f"t8_flange_washer_m3_{index}"):
                with self.subTest(label=label):
                    self.assertLess(audit.common_volume(bracket, hw[label]),
                                    1e-6)

    def test_existing_m1_roof_preserves_motor_and_spindle_datums(self):
        self.assertAlmostEqual(audit.M1_MOTOR_HEAD_Y, -179.65)
        roof = audit.M1_MOTOR_HEAD_Y - P.m1_motor_top_y
        self.assertAlmostEqual(roof, 4.0)
        self.assertAlmostEqual(10.0 - roof, 6.0)
        report = audit.report_dict()
        self.assertFalse(report["stator_datum"]["changed"])
        self.assertEqual(report["stator_datum"]["axis_z_home_mm"], 95.0)

    def test_sendcutsend_plate_has_only_the_required_through_cut_change(self):
        before = fabricated_carriage.carriage_plate()
        report = audit.report_dict()["sendcutsend"]
        self.assertTrue(report["plate_geometry_changed"])
        self.assertEqual(
            fabricated_carriage.T8_RELIEF[2],
            fabricated_carriage.PLATE_Z_MIN,
        )
        self.assertTrue(report["through_cut_profile_only"])
        self.assertAlmostEqual(report["plate_volume_mm3"], before.volume, 5)

    def test_exact_patch_and_intended_contact_tables_are_complete(self):
        self.assertEqual(len(audit.PATCH_TABLE), 6)
        self.assertEqual(len(audit.INTENDED_CONTACTS), 9)
        sources = {row["source"] for row in audit.PATCH_TABLE}
        self.assertIn("cad/printed.py:spindle_tower", sources)
        self.assertIn("cad/fabricated_carriage.py:T8_RELIEF", sources)
        self.assertTrue(any("hardware.py" in source for source in sources))
        exemptions = [row for row in audit.INTENDED_CONTACTS
                      if "explicit dynamic fit exemption" in row["intent"]]
        self.assertEqual(len(exemptions), 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
