"""Focused tests for the analytic C1 follower rebound sweep."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_c1_rebound_sweep as sweep


class AggregateBoundaryFollowerC1ReboundSweepTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.report = sweep.analyze()
        cls.cases = [
            case for row in cls.report["loci"]
            for case in row["diameter_cases"]
        ]
        cls.constructed = [
            case for case in cls.cases
            if case["status"] == "PASS_ANALYTIC_C1_S_BIARC"
        ]

    def test_all_cases_attempted_and_g0_is_precisely_not_applicable(self):
        coverage = self.report["coverage"]
        self.assertEqual(coverage["evaluated_loci"], 2400)
        self.assertEqual(coverage["route_case_count"], 4800)
        self.assertEqual(coverage["g0_no_endpoint_pair_case_count"], 96)
        self.assertEqual(coverage["nonzero_attempted_case_count"], 4704)
        self.assertEqual(coverage["analytic_C1_biarc_pass_case_count"], 4704)
        self.assertEqual(coverage["mathematical_failure_case_count"], 0)
        g0 = [case for case in self.cases
              if case["status"].startswith("NOT_APPLICABLE_G0")]
        self.assertEqual(len(g0), 96)
        self.assertTrue(all("no nondegenerate aggregate" in case["reason"]
                            for case in g0))

    def test_every_constructed_biarc_closes_C0_and_C1(self):
        self.assertEqual(len(self.constructed), 4704)
        for case in self.constructed:
            closure = case["closure"]
            self.assertLessEqual(
                closure["end_position_residual_mm"],
                sweep.GEOMETRY_TOLERANCE_MM,
            )
            self.assertLessEqual(
                closure["end_tangent_residual"], sweep.TANGENT_TOLERANCE
            )
            self.assertLessEqual(
                closure["join_tangent_residual"], sweep.TANGENT_TOLERANCE
            )
            self.assertGreater(case["total_length_mm"], case["chord_length_mm"])
            self.assertGreater(
                case["maximum_lateral_sweep_from_chord_mm"], 0.0
            )

    def test_diameter_specific_end_radius_and_first_arc_floor(self):
        for case in self.constructed:
            expected = 3.0 + case["wire_radius_mm"]
            self.assertAlmostEqual(
                case["second_arc"]["absolute_radius_mm"], expected, places=9
            )
            self.assertGreaterEqual(
                case["first_arc"]["absolute_radius_mm"], expected - 1.0e-8
            )
            self.assertGreaterEqual(case["minimum_absolute_radius_mm"], 3.0)
            self.assertGreater(case["first_arc"]["signed_sweep_deg"], 0.0)
            self.assertLess(case["second_arc"]["signed_sweep_deg"], 0.0)
        bounds = self.report["bounds"]["second_arc_absolute_radius_mm"]
        self.assertAlmostEqual(bounds[0], 3.10, places=9)
        self.assertAlmostEqual(bounds[1], 3.25, places=9)

    def test_contact_normal_and_follower_travel_remain_fail_closed(self):
        coverage = self.report["coverage"]
        self.assertEqual(coverage["compression_normal_compatible_case_count"], 0)
        dots = self.report["bounds"][
            "end_center_direction_dot_aggregate_normal"
        ]
        self.assertLess(max(abs(dots[0]), abs(dots[1])), 1.0e-8)
        self.assertFalse(self.report["physical_gates"]
                         ["end_arc_center_direction_matches_aggregate_compression_normal"])
        travel = self.report["follower_center_travel"]
        self.assertTrue(all(value["radial_stroke_analytic_pass"]
                            for value in travel.values()))
        self.assertFalse(all(value["tangential_stroke_analytic_pass"]
                             for value in travel.values()))
        self.assertFalse(self.report["physical_gates"]
                         ["required_axial_center_shift_has_physical_DOF"])

    def test_no_analytic_curve_is_promoted_to_physical_authority(self):
        self.assertEqual(
            self.report["coverage"]["positive_volume_placed_case_count"], 0
        )
        self.assertEqual(
            self.report["coverage"]["physically_authorized_case_count"], 0
        )
        self.assertEqual(self.report["status"], "FAIL")
        for key in (
            "wire_route_authorized", "collision_authorized",
            "assembly_integration_authorized", "dancer_coupling_authorized",
            "production_authorized",
        ):
            self.assertFalse(self.report[key])
        self.assertTrue(self.report["analytic_gates"]
                        ["maximum_tangent_excursion_within_65deg_analytic_gimbal_range"])

    def test_hash_binding_tamper_rejection_and_written_outputs(self):
        sweep.validate_report_integrity(self.report)
        bad = deepcopy(self.report)
        bad["wire_route_authorized"] = True
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            sweep.validate_report_integrity(bad)
        generated = sweep.write_outputs(self.report)
        written = json.loads(sweep.OUTPUT_JSON.read_text(encoding="utf-8"))
        self.assertEqual(generated["report_sha256"], written["report_sha256"])
        sweep.validate_report_integrity(written)


if __name__ == "__main__":
    unittest.main(verbosity=2)
