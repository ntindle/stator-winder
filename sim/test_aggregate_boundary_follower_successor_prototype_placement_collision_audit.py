"""Focused report tests for the frozen successor placement/collision audit."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_successor_prototype_placement_collision_audit as audit


class SuccessorPrototypePlacementCollisionAuditTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.report = json.loads(
            audit.REPORT_JSON.read_text(encoding="utf-8")
        )

    def test_report_hash_inputs_status_and_authority_fail_closed(self):
        audit.validate_report(self.report)
        self.assertEqual(self.report["input_hashes"], audit._input_hashes())
        self.assertEqual(
            self.report["status"],
            "PASS_AUDIT__PROTOTYPE_NOT_PLACEMENT_OR_COLLISION_READY",
        )
        self.assertFalse(any(self.report["authority"].values()))
        self.assertGreater(len(self.report["blocking_findings"]), 0)

    def test_exact_all_4704_coverage_and_exact_failures(self):
        coverage = self.report["analytic_all_4704_case_coverage"]
        self.assertEqual(coverage["case_count"], 4704)
        self.assertEqual(
            coverage["cases_per_identity"],
            {"0": 1176, "1": 1176, "2": 1176, "3": 1176},
        )
        self.assertEqual(
            coverage["exact_identity_center_bounds_covered_case_count"],
            4704,
        )
        self.assertEqual(
            coverage[
                "modeled_1p50x2p40x1p10_center_travel_covered_case_count"
            ],
            4704,
        )
        self.assertEqual(
            coverage["numeric_yaw_elevation_range_covered_case_count"],
            4704,
        )
        self.assertEqual(
            coverage["prototype_Rot_realized_tangent_match_case_count"],
            0,
        )
        self.assertEqual(
            coverage[
                "conservative_R3_envelope_inside_fixed_R5_relief_case_count"
            ],
            4704,
        )
        self.assertEqual(
            coverage[
                "full_2mm_R3_to_fixed_R5_relief_margin_case_count"
            ],
            0,
        )
        self.assertAlmostEqual(
            coverage["minimum_R3_to_R5_remaining_radial_margin_mm"],
            0.635960420053225,
            places=12,
        )

    def test_direct_all_case_and_endpoint_BREP_counts_are_complete(self):
        direct = self.report["direct_all_4704_guide_to_floor_BREP"]
        self.assertEqual(direct["case_count"], 4704)
        self.assertEqual(direct["exact_distance_query_count"], 4704)
        self.assertEqual(direct["kernel_exception_count"], 0)
        self.assertEqual(direct["zero_positive_common_volume_case_count"], 4692)
        self.assertEqual(direct["positive_common_volume_case_count"], 12)

        sampled = self.report["sampled_endpoint_BREP"]
        contract = sampled["sampling_contract"]
        self.assertEqual(contract["pose_count_per_identity"], 43)
        self.assertEqual(contract["total_identity_pose_count"], 172)
        self.assertEqual(contract["single_axis_endpoint_pose_count"], 10)
        self.assertEqual(contract["combined_five_DOF_corner_pose_count"], 32)
        self.assertEqual(
            sampled["self_collision"]["unique_positive_pair_count"], 34,
        )
        self.assertEqual(
            sampled["own_floor_leaf_collision"]["unique_positive_pair_count"],
            8,
        )
        self.assertEqual(
            sampled["conservative_R3_to_own_floor_collision"][
                "unique_positive_pair_count"
            ],
            0,
        )
        self.assertEqual(
            sampled["review_rack_sibling_collision"][
                "unique_positive_pair_count"
            ],
            0,
        )
        self.assertEqual(
            sampled["exact_active_local_rebased_sibling_collision"][
                "unique_positive_pair_count"
            ],
            8,
        )
        for name in (
            "self_collision", "own_floor_leaf_collision",
            "conservative_R3_to_own_floor_collision",
            "review_rack_sibling_collision",
            "exact_active_local_rebased_sibling_collision",
        ):
            self.assertEqual(sampled[name]["kernel_exception_count"], 0)


if __name__ == "__main__":
    unittest.main()
