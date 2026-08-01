"""Focused tests for the fail-closed replacement transition sweep."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_replacement_transition_sweep as sweep


class AggregateBoundaryFollowerReplacementTransitionSweepTests(
    unittest.TestCase,
):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = json.loads(sweep.OUTPUT_JSON.read_text(encoding="utf-8"))

    def test_prescribed_sequence_covers_four_identities_and_all_endpoints(self):
        poses = sweep.all_prescribed_poses()
        self.assertEqual({pose.identity_index for pose in poses}, {0, 1, 2, 3})
        self.assertEqual(len(poses), 232)
        self.assertEqual(len(sweep.prescribed_poses(0)), 58)
        for identity_index in range(4):
            identity_poses = sweep.prescribed_poses(identity_index)
            self.assertTrue(any(
                pose.phase == "coarse_translate_while_retracted"
                and math.isclose(pose.coarse_base_abs_y_mm, 10.95)
                and math.isclose(pose.radial_center_x_mm, 14.0)
                for pose in identity_poses
            ))
            self.assertTrue(any(
                pose.phase == "coarse_translate_while_retracted"
                and math.isclose(pose.coarse_base_abs_y_mm, 2.05)
                and math.isclose(pose.radial_center_x_mm, 14.0)
                for pose in identity_poses
            ))
            for radial_x in sweep._linspace(14.0, 20.0, 12):
                for q_mm in (-0.5, 0.0, 0.5):
                    self.assertTrue(any(
                        math.isclose(pose.coarse_base_abs_y_mm, 2.05)
                        and math.isclose(pose.radial_center_x_mm, radial_x)
                        and math.isclose(pose.passive_q_mm, q_mm)
                        for pose in identity_poses
                    ))

    def test_pose_transform_keeps_exact_15_manufactured_leaves(self):
        probes = (
            sweep.prescribed_poses(0)[0],
            sweep.prescribed_poses(0)[-1],
            sweep.prescribed_poses(3)[0],
            sweep.prescribed_poses(3)[-1],
        )
        for pose in probes:
            with self.subTest(pose=pose.key):
                leaves = sweep.moving_leaves_at_pose(pose)
                self.assertEqual(len(leaves), 15)
                self.assertTrue(all(float(part.volume) > 0.0 for part in leaves))
                self.assertTrue(any(
                    "outer_pivot_MISUMI_SCCG5-10" in str(part.label)
                    for part in leaves
                ))
                self.assertEqual(sum(
                    "outer_pivot_MISUMI_NETWS4" in str(part.label)
                    for part in leaves
                ), 2)

    def test_report_is_self_hashed_current_and_sampling_contract_passes(self):
        sweep.validate_report_integrity(self.report)
        sampling = self.report["sampling"]
        self.assertEqual(sampling["identity_count"], 4)
        self.assertEqual(sampling["sample_count_per_identity"], 58)
        self.assertEqual(sampling["total_pose_count"], 232)
        self.assertLessEqual(
            sampling["maximum_independent_translation_step_mm"], 0.5,
        )
        self.assertTrue(all(sampling["gates"].values()))

    def test_exact_common_volumes_are_zero_and_clearance_gate_passes(self):
        collision = self.report["collision_audit"]
        clearance = self.report["clearance_audit"]
        self.assertTrue(collision["all_sampled_positive_common_volumes_zero"])
        self.assertEqual(collision["positive_failure_count"], 0)
        self.assertTrue(collision[
            "static_parked_siblings_include_full_pivot_hardware"
        ])
        self.assertTrue(collision[
            "selected_occurrence_includes_full_pivot_hardware"
        ])
        self.assertTrue(collision["primary_M4_hardware_included"])
        self.assertEqual(
            collision["non_manufactured_blocker_envelope_count_excluded"], 4,
        )
        self.assertAlmostEqual(
            clearance["minimum_sampled_exact_clearance_mm"],
            2.5,
            places=7,
        )
        self.assertAlmostEqual(
            collision["minimum_downstream_to_carrier_exact_distance_mm"],
            2.5,
            places=7,
        )
        self.assertAlmostEqual(
            collision[
                "minimum_selected_to_parked_sibling_exact_distance_mm"
            ],
            2.5,
            places=7,
        )
        self.assertEqual(clearance["required_minimum_mm"], 2.0)
        self.assertAlmostEqual(
            clearance["nominal_reserve_above_requirement_mm"],
            0.5,
            places=7,
        )
        self.assertEqual(clearance["violation_count"], 0)
        self.assertTrue(clearance["passes_2p00mm_gate"])
        self.assertTrue(clearance[
            "downstream_body_and_pivot_to_carrier_included_in_noncontact_gate"
        ])
        self.assertIn(
            clearance["minimum_leaf_pair_witness"]["selected_label"],
            {
                "front_left:monolithic_7075_tangential_slide_outer_yoke:retracted",
                "front_left:inner_gimbal_yoke_6061",
                "front_left:inner_pivot_M2_nyloc",
                "front_left:outer_pivot_MISUMI_SCCG5-10_D5_grooved_pin",
            },
        )
        self.assertEqual(
            self.report["status"], "PASS_SAMPLED_GEOMETRY_ONLY",
        )

    def test_mechanism_integration_and_release_authority_remain_false(self):
        self.assertTrue(all(not value for value in self.report["authority"].values()))
        sequence = self.report["prescribed_sequence"]
        self.assertFalse(sequence["positive_selection_mechanism_modeled"])
        self.assertFalse(sequence["positive_retraction_mechanism_modeled"])
        self.assertIn(
            "NON_MANUFACTURED_8p90mm_coarse_selector_blocker_envelopes_only",
            self.report["blockers"],
        )
        self.assertFalse(self.report["integration"]["assembly_source_modified"])
        self.assertFalse(self.report["integration"]["release_modified"])
        self.assertFalse(self.report["integration"]["BOM_modified"])

    def test_invalid_identity_is_rejected(self):
        with self.assertRaises(ValueError):
            sweep.prescribed_poses(-1)
        with self.assertRaises(ValueError):
            sweep.prescribed_poses(4)


if __name__ == "__main__":
    unittest.main(verbosity=2)
