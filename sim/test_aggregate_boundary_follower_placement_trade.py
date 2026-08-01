"""Focused tests for the fail-closed follower placement trade."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_placement_trade as trade


class AggregateBoundaryFollowerPlacementTradeTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.report = trade.analyze()
        cls.cases = cls.report["case_comparisons"]

    def test_all_4704_centres_are_compared_without_current_CAD_promotion(self):
        coverage = self.report["coverage"]
        self.assertEqual(coverage["compared_nonzero_cases"], 4704)
        self.assertEqual(
            coverage["cases_per_identity"],
            {"0": 1176, "1": 1176, "2": 1176, "3": 1176},
        )
        self.assertEqual(
            coverage["cases_per_diameter"],
            {"d0.2": 2352, "d0.5": 2352},
        )
        self.assertEqual(
            coverage["current_CAD_full_center_covered_case_count"], 0
        )
        self.assertEqual(
            coverage["current_CAD_fixed_pose_normal_covered_case_count"], 0
        )
        self.assertTrue(all(
            not row["current_CAD_nose_envelope"]["full_center_covered"]
            for row in self.cases
        ))
        self.assertTrue(all(
            not row["current_CAD_nose_envelope"]["radial_axis_covered"]
            and not row["current_CAD_nose_envelope"]["axial_axis_covered"]
            for row in self.cases
        ))

    def test_exact_current_source_transform_and_axis_offsets_are_bound(self):
        cad = self.report["current_replacement_CAD"]
        binding = cad["geometry_contract_binding"]
        self.assertTrue(binding[
            "manifest_matches_source_geometry_contract"
        ])
        self.assertTrue(binding["manifest_STEP_hash_matches_artifact"])
        BREP = binding["source_nose_BREP_center_validation"]
        self.assertEqual(BREP["checked_center_count"], 24)
        self.assertTrue(BREP["all_centers_match"])
        self.assertLessEqual(
            BREP[
                "maximum_BREP_bbox_center_to_source_transform_residual_mm"
            ],
            trade.GEOMETRY_TOLERANCE_MM,
        )
        for physical_id, axial_sign, tangential_sign in (
            (0, 1, -1), (1, 1, 1), (2, -1, 1), (3, -1, -1),
        ):
            envelope = cad["nose_envelopes_by_identity"][str(physical_id)]
            bounds = envelope["active_local_bounds"]
            self.assertAlmostEqual(bounds["min_mm"][0], 29.7, places=9)
            self.assertAlmostEqual(bounds["max_mm"][0], 35.7, places=9)
            self.assertAlmostEqual(
                bounds["min_mm"][1],
                -2.55 if tangential_sign < 0 else 1.55,
                places=9,
            )
            self.assertAlmostEqual(
                bounds["max_mm"][1],
                -1.55 if tangential_sign < 0 else 2.55,
                places=9,
            )
            self.assertAlmostEqual(
                bounds["min_mm"][2], axial_sign * 21.35, places=9
            )
            self.assertAlmostEqual(
                bounds["max_mm"][2], axial_sign * 21.35, places=9
            )
            self.assertEqual(envelope["owner"], "M0_carriage")
            self.assertFalse(envelope["M1_spatial_transform"])
            self.assertFalse(envelope["M2_spatial_transform"])

        left_thick = self.report["per_identity"]["0"]["per_diameter"][
            "d0.5"
        ]
        self.assertAlmostEqual(
            left_thick[
                "absolute_axis_offset_to_current_envelope_ranges_XYZ_mm"
            ][0][0],
            11.160593278918984,
            places=9,
        )
        self.assertAlmostEqual(
            left_thick[
                "absolute_axis_offset_to_current_envelope_ranges_XYZ_mm"
            ][2][1],
            15.322573710083802,
            places=9,
        )
        self.assertEqual(left_thick["tangential_axis_covered_case_count"], 132)
        right_thin = self.report["per_identity"]["1"]["per_diameter"][
            "d0.2"
        ]
        self.assertAlmostEqual(
            right_thin[
                "absolute_axis_offset_to_current_envelope_ranges_XYZ_mm"
            ][1][0],
            0.03472058773586584,
            places=9,
        )
        self.assertEqual(right_thin["tangential_axis_covered_case_count"], 0)

    def test_successor_datums_strokes_and_angle_ranges_cover_every_case(self):
        successor = self.report["successor_trade"]
        self.assertEqual(successor["owner"], "M0_carriage")
        self.assertFalse(successor["M1_spatial_transform"])
        self.assertFalse(successor["M2_spatial_transform"])
        expected_common = (
            1.3822561230042538,
            2.233484956719163,
            0.9733701456993078,
        )
        for actual, expected in zip(
                successor["common_exact_minimum_center_strokes_XYZ_mm"],
                expected_common):
            self.assertAlmostEqual(actual, expected, places=9)
        front_left = successor["per_identity"]["0"]
        for actual, expected in zip(
                front_left["exact_target_datum_local_mm"],
                (17.848278659578888, -2.57896814776774,
                 6.514111362765854)):
            self.assertAlmostEqual(actual, expected, places=9)
        self.assertAlmostEqual(
            front_left["curvature_normal_orientation"]["yaw"]
            ["half_range_deg"],
            53.23669873274603,
            places=9,
        )
        for actual, expected in zip(
                front_left[
                    "current_CAD_reference_to_target_datum_translation_XYZ_mm"
                ],
                (-14.851721340421114, -0.5289681477677399,
                 -14.835888637234148)):
            self.assertAlmostEqual(actual, expected, places=9)
        self.assertTrue(front_left[
            "prototype_65deg_two_axis_half_range_covers_reported_intervals"
        ])
        self.assertFalse(front_left["positive_volume_gimbal_range_proved"])
        self.assertLess(
            front_left["curvature_normal_orientation"]["yaw"]
            ["half_range_deg"],
            65.0,
        )
        self.assertEqual(
            self.report["coverage"]
            ["successor_analytic_center_covered_case_count"],
            4704,
        )
        self.assertTrue(all(
            row["successor_stage"]["inside_exact_center_stroke_envelope"]
            for row in self.cases
        ))

    def test_diameter_changeover_cannot_be_a_translation_only_shim(self):
        changeovers = self.report["successor_trade"]["diameter_changeover"]
        for row in changeovers.values():
            self.assertEqual(row["paired_locus_count"], 588)
            self.assertFalse(row["translation_only_changeover_shim_exact"])
            self.assertGreater(
                row["curvature_normal_rotation_range_deg"][1], 1.0
            )
        self.assertFalse(
            changeovers["0"]["same_translation_vector_at_every_locus"]
        )
        self.assertTrue(
            changeovers["1"]["same_translation_vector_at_every_locus"]
        )
        self.assertAlmostEqual(
            changeovers["0"]["translation_magnitude_range_mm"][1],
            0.5947156252822966,
            places=9,
        )
        self.assertAlmostEqual(
            changeovers["1"]["translation_magnitude_range_mm"][0],
            0.26923946743358046,
            places=9,
        )

    def test_compression_normal_and_carrier_host_remain_fail_closed(self):
        coverage = self.report["coverage"]
        self.assertEqual(
            coverage["circular_nose_compression_compatible_case_count"], 0
        )
        self.assertTrue(all(
            abs(row["curvature_to_aggregate_normal_angle_deg"] - 90.0)
            <= 1.0e-8
            for row in self.cases
        ))
        compression = self.report["successor_trade"][
            "compression_decoupling"
        ]
        self.assertTrue(compression["required"])
        self.assertFalse(compression["translation_can_fix"])
        self.assertFalse(compression[
            "gimbal_rotation_can_fix_while_preserving_circular_arc_center"
        ])

        host = self.report["carrier_host_screen"]
        self.assertTrue(host["target_center_projection_all_inside"])
        self.assertTrue(host["all_have_nonnegative_clearance"])
        self.assertAlmostEqual(
            host["R3_surface_to_floor_clearance_range_mm"][0],
            0.17742628991619958,
            places=9,
        )
        self.assertFalse(host["all_meet_nominal_2mm_clearance"])
        self.assertFalse(host["current_carrier_host_authorized"])

    def test_hash_binding_tamper_rejection_and_written_outputs(self):
        trade.validate_report_integrity(self.report)
        bad = deepcopy(self.report)
        bad["release_authorized"] = True
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            trade.validate_report_integrity(bad)
        generated = trade.write_outputs(self.report)
        written = json.loads(trade.OUTPUT_JSON.read_text(encoding="utf-8"))
        self.assertEqual(generated["report_sha256"], written["report_sha256"])
        trade.validate_report_integrity(written)
        self.assertIn(
            "AGGREGATE_NORMAL_PRELOAD_MUST_BE_DECOUPLED",
            written["decision"],
        )
        for key in trade.AUTHORITY_KEYS:
            self.assertFalse(written[key])


if __name__ == "__main__":
    unittest.main(verbosity=2)
