"""Focused tests for the isolated aggregate-follower acceptance gate."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_acceptance as acceptance


class AggregateBoundaryFollowerAcceptanceTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.report = acceptance.analyze()

    def test_exact_endpoint_binding_and_24_remaining_g0_blockers(self):
        report = self.report
        coverage = report["coverage"]
        self.assertEqual(coverage["bound_endpoint_loci"], 2400)
        self.assertEqual(coverage["endpoint_binding_mismatch_count"], 0)
        self.assertEqual(coverage["g0_blocker_count"], 24)
        expected = [
            1, 100, 201, 300, 400, 501, 600, 701, 800, 901,
            1000, 1101, 1201, 1300, 1401, 1500, 1601, 1700,
            1801, 1900, 2000, 2101, 2200, 2301,
        ]
        self.assertEqual(report["g0_blocker_locus_indices"], expected)
        self.assertTrue(all(
            row["code"] == "g0_right_seam_positive_BREP_normal_missing"
            and row["required_owner"].startswith("Nomex")
            for row in report["g0_blockers"]
        ))

    def test_R3_CAD_is_bound_but_mechanism_and_motion_fail_closed(self):
        report = self.report
        gates = report["gates"]
        self.assertEqual(
            report["schema"], "aggregate-boundary-follower-acceptance/v14"
        )
        self.assertEqual(report["status"], "FAIL")
        self.assertFalse(report["production_authorized"])
        self.assertTrue(gates["positive_volume_R3_follower_CAD_provenance"])
        self.assertFalse(gates["R3_follower_mechanism_complete"])
        self.assertTrue(gates["retraction_topology_analysis_closed"])
        self.assertFalse(gates[
            "positive_retraction_and_actual_position_interlock_integrated"
        ])
        self.assertFalse(gates[
            "replacement_carriage_integration_and_collision_authorized"
        ])
        self.assertTrue(gates[
            "replacement_architecture_identity_and_counts_bound"
        ])
        self.assertTrue(gates[
            "replacement_carriage_static_CAD_and_zero_overlap_bound"
        ])
        self.assertTrue(gates[
            "replacement_carriage_sampled_transition_geometry_bound"
        ])
        self.assertFalse(gates[
            "replacement_carriage_transition_physical_authority"
        ])
        self.assertTrue(gates[
            "replacement_load_wear_analytical_screens_bound"
        ])
        self.assertFalse(gates[
            "replacement_load_wear_physical_qualification_complete"
        ])
        self.assertTrue(gates["retraction_procurement_evidence_current"])
        self.assertTrue(gates[
            "g0_normal_audit_current_and_exactly_classified"
        ])
        self.assertTrue(gates["g0_robust_landing_trade_current"])
        self.assertTrue(gates[
            "g0_PEEK_shelf_isolated_CAD_geometry_bound"
        ])
        self.assertFalse(gates[
            "g0_PEEK_shelf_and_0p65mm_cap_lane_integrated"
        ])
        self.assertTrue(gates["custom_return_concepts_screened"])
        self.assertTrue(gates[
            "custom_return_isolated_CAD_geometry_bound"
        ])
        self.assertTrue(gates[
            "all_2400_loci_x_two_diameters_analytic_route_classified"
        ])
        self.assertTrue(gates[
            "all_4704_nonzero_routes_have_exact_analytic_C1_biarcs"
        ])
        self.assertFalse(gates[
            "positive_volume_C1_route_and_normal_preload_authorized"
        ])
        self.assertTrue(gates["successor_placement_trade_exactly_bound"])
        self.assertTrue(gates[
            "successor_prototype_source_STEP_manifest_audit_current"
        ])
        self.assertTrue(gates[
            "successor_isolated_positive_volume_prototype_geometry_bound"
        ])
        self.assertTrue(gates[
            "successor_placement_collision_audit_source_and_paths_current"
        ])
        self.assertTrue(gates[
            "successor_analytic_center_and_numeric_range_coverage_bound"
        ])
        for name in (
            "successor_realized_tangent_matches_all_4704_cases",
            "successor_full_2mm_R3_relief_margin_all_4704_cases",
            "successor_modeled_guides_zero_positive_to_floor_all_4704",
            "successor_sampled_endpoint_self_collision_zero",
            "successor_sampled_endpoint_floor_collision_zero",
            "successor_exact_active_local_sibling_collision_zero",
        ):
            self.assertFalse(gates[name])
        self.assertFalse(gates[
            "successor_prototype_all_4704_routes_and_full_motion_bound"
        ])
        self.assertFalse(gates[
            "successor_prototype_collision_sweep_authorized"
        ])
        self.assertFalse(gates[
            "successor_prototype_load_tolerance_buildability_qualified"
        ])
        self.assertFalse(gates[
            "successor_redatumed_stage_positive_volume_and_integrated"
        ])
        self.assertFalse(gates[
            "custom_return_hardware_CAD_and_endurance_qualified"
        ])
        self.assertFalse(gates[
            "retraction_hardware_stack_selected_and_releasable"
        ])
        self.assertFalse(gates["eccentric_40N_mount_load_path_qualified"])
        self.assertFalse(gates["R3_route_closes_every_direct_C1_mismatch"])
        self.assertFalse(gates["exact_continuous_intra_half_turn_follower_law"])
        self.assertFalse(gates[
            "adaptive_transition_swept_rigid_core_cap_prior_self_clearance"
        ])
        self.assertEqual(
            report["coverage"]["direct_C1_mismatch_locus_count"], 2352,
        )
        self.assertEqual(
            report["continuous_motion_evidence"]["paired_C0_interval_count"],
            2376,
        )
        self.assertEqual(
            report["continuous_motion_evidence"]["unpaired_closing_interval_count"],
            24,
        )

    def test_dancer_static_model_is_not_dynamic_or_length_coupling(self):
        dancer = self.report["dancer_coupling"]
        self.assertTrue(dancer["static_model_available"])
        self.assertFalse(dancer["downstream_length_history_bound"])
        self.assertFalse(dancer["static_coupling_proven"])
        self.assertFalse(dancer["dynamic_authority"])
        joined = " ".join(dancer["limitations"]).lower()
        self.assertIn("transient damping", joined)
        self.assertIn("flyer acceleration", joined)
        self.assertEqual(self.report["dynamic_authority"], "FAIL")

    def test_no_strand_order_or_interior_linear_interpolation(self):
        scope = self.report["scope"]
        self.assertFalse(scope["deterministic_strand_order_used"])
        self.assertFalse(
            scope["linear_interpolation_through_aggregate_interior_allowed"]
        )
        self.assertTrue(scope[
            "straight_taut_free_spans_allowed_when_endpoint_reacted_C0_C1_clear_and_tensioned"
        ])
        self.assertTrue(self.report["gates"][
            "aggregate_authority_PASS_without_strand_order"
        ])
        self.assertTrue(self.report["gates"]["predecessor_self_hashes_valid"])
        self.assertTrue(self.report["gates"]["predecessor_source_hashes_current"])

    def test_exact_global_blockers_cover_all_intervals(self):
        blockers = {row["code"]: row for row in self.report["blockers"]}
        self.assertIn(
            "positive_volume_R3_follower_mechanism_incomplete", blockers)
        self.assertIn("eccentric_40N_mount_load_path_unqualified", blockers)
        self.assertIn(
            "positive_retraction_and_actual_position_interlock_unintegrated",
            blockers,
        )
        self.assertIn("replacement_carriage_integration_missing", blockers)
        self.assertIn("retraction_hardware_procurement_incomplete", blockers)
        self.assertIn(
            "g0_robust_PEEK_shelf_and_wire_range_route_unintegrated", blockers
        )
        self.assertIn(
            "custom_return_hardware_unintegrated_and_unqualified", blockers
        )
        self.assertTrue(blockers[
            "g0_robust_PEEK_shelf_and_wire_range_route_unintegrated"
        ]["isolated_cap_shelf_CAD_bound"])
        self.assertEqual(
            blockers[
                "g0_robust_PEEK_shelf_and_wire_range_route_unintegrated"
            ]["isolated_cap_shelf_STEP_sha256"],
            "9ee0306f5c71e6f46bc791b6d5b017c3ed864d939a2926346c2a8505d5a97443",
        )
        self.assertTrue(blockers[
            "custom_return_hardware_unintegrated_and_unqualified"
        ]["isolated_return_package_CAD_bound"])
        self.assertEqual(
            blockers["replacement_carriage_integration_missing"][
                "reference_positive_pair_count"
            ],
            21,
        )
        replacement = blockers["replacement_carriage_integration_missing"]
        self.assertTrue(replacement["replacement_static_CAD_bound"])
        self.assertEqual(
            replacement["replacement_STEP_sha256"],
            "3c1a8299ade7bb2487a528b0b39f03e00cfd0eeb702734c6f7a2d898bcb55468",
        )
        self.assertTrue(
            replacement["static_state_pair_audit"][
                "all_scopes_zero_positive"
            ]
        )
        self.assertNotIn(
            "positive_volume_R3_follower_CAD_provenance_missing", blockers)
        for code in (
            "exact_continuous_intra_half_turn_follower_law_missing",
            "adaptive_transition_swept_clearance_missing",
            "downstream_length_and_dancer_static_coupling_missing",
        ):
            row = blockers[code]
            self.assertEqual(row["affected_interval_count"], 2400)
            self.assertEqual(row["affected_interval_indices"], list(range(2400)))
        route_blocker = blockers[
            "exact_continuous_intra_half_turn_follower_law_missing"
        ]
        self.assertTrue(
            route_blocker["route_sweep_analytic_classification_bound"]
        )
        self.assertEqual(route_blocker["diameter_route_case_count"], 4800)
        self.assertEqual(route_blocker["direct_C1_case_count"], 0)
        self.assertFalse(blockers[
            "adaptive_transition_swept_clearance_missing"
        ]["exact_FCL_or_equivalent_narrow_phase_run"])

    def test_final_transition_load_C1_and_placement_evidence_is_bound(self):
        report = self.report

        transition = report["replacement_transition_evidence"]
        self.assertTrue(transition["evidence_current"])
        self.assertTrue(transition["sampled_geometry_bound"])
        self.assertFalse(transition["physical_authority"])
        self.assertEqual(transition["sampling"]["total_pose_count"], 232)
        self.assertAlmostEqual(
            transition["clearance_audit"][
                "minimum_sampled_exact_clearance_mm"
            ],
            2.5,
            places=7,
        )

        load_wear = report["replacement_load_wear_evidence"]
        self.assertTrue(load_wear["evidence_current"])
        self.assertTrue(load_wear["analytical_screens_bound"])
        self.assertFalse(load_wear["physical_qualification_complete"])
        self.assertAlmostEqual(
            load_wear["load_envelope"][
                "candidate_high_side_margin_to_2N_N"
            ],
            0.017290749,
        )
        self.assertTrue(all(load_wear["analytical_gates"].values()))
        self.assertFalse(any(load_wear["qualification_gates"].values()))

        C1 = report["C1_rebound_evidence"]
        self.assertTrue(C1["evidence_current"])
        self.assertTrue(C1["exact_analytic_C1_biarcs_bound"])
        self.assertFalse(C1["positive_volume_route_authorized"])
        self.assertEqual(
            C1["coverage"]["analytic_C1_biarc_pass_case_count"], 4704
        )
        self.assertEqual(C1["coverage"]["positive_volume_placed_case_count"], 0)
        self.assertEqual(
            C1["coverage"]["compression_normal_compatible_case_count"], 0
        )

        placement = report["placement_trade_evidence"]
        self.assertTrue(placement["evidence_current"])
        self.assertTrue(placement["analytic_trade_bound"])
        self.assertFalse(placement["successor_physical_authority"])
        self.assertEqual(
            placement["coverage"][
                "current_CAD_full_center_covered_case_count"
            ],
            0,
        )
        self.assertEqual(
            placement["coverage"][
                "successor_analytic_center_covered_case_count"
            ],
            4704,
        )
        self.assertEqual(
            placement["successor_trade"][
                "common_exact_minimum_center_strokes_XYZ_mm"
            ],
            [
                1.3822561230042538,
                2.233484956719163,
                0.9733701456993078,
            ],
        )
        self.assertFalse(
            placement["carrier_host_screen"]["all_meet_nominal_2mm_clearance"]
        )

        prototype = report["successor_prototype_evidence"]
        self.assertTrue(prototype["evidence_current"])
        self.assertTrue(prototype["isolated_positive_volume_geometry_bound"])
        self.assertFalse(prototype["all_4704_routes_and_full_motion_bound"])
        self.assertFalse(prototype["collision_authorized"])
        self.assertFalse(prototype["load_tolerance_buildability_qualified"])
        self.assertFalse(prototype["physical_authority"])
        self.assertEqual(prototype["evidence"]["stage"]["count"], 4)
        self.assertEqual(prototype["evidence"]["guide"]["count"], 4)
        self.assertFalse(prototype["evidence"]["guide"]
                         ["all_4704_case_surface_proved"])
        self.assertFalse(any(prototype["authority"].values()))
        expected_hashes = {
            "source": "782456ef56019427d2bdf4fa3be8fa2c4e1684f1dd3be9e6cee7b04422c9677b",
            "step": "6bf20bbca4f166a7c39cee4aec309e8f7765655597a8c8f8e4a335e12a2db183",
            "manifest": "0e8ef6bbd0e59a8025d39abd48bf20acc381688ab7b2f63c29540f6c6fc26edb",
            "audit": "1d5939e6a6d380ae97982675fff033fa919cdf5f47bc3e78be1e7de8265c38d7",
        }
        self.assertEqual(
            {
                name: row["sha256"]
                for name, row in prototype["artifact_binding"].items()
            },
            expected_hashes,
        )
        self.assertEqual(
            prototype["report_sha256"],
            "5ff1180308c043d0933fadcfbe8fd8cc0c3a1bdc71bc104400b8d7e2d13e921f",
        )

        realized = report["successor_placement_collision_evidence"]
        self.assertTrue(realized["evidence_current"])
        self.assertTrue(realized["analytic_center_and_range_bound"])
        self.assertFalse(realized["realized_tangent_all_cases"])
        self.assertFalse(realized["full_2mm_relief_all_cases"])
        self.assertFalse(realized["guide_floor_collision_zero"])
        self.assertFalse(realized["sampled_self_collision_zero"])
        self.assertFalse(realized["sampled_floor_collision_zero"])
        self.assertFalse(realized["exact_local_sibling_collision_zero"])
        coverage = realized["analytic_all_4704_case_coverage"]
        self.assertEqual(coverage["case_count"], 4704)
        self.assertEqual(
            coverage["prototype_Rot_realized_tangent_match_case_count"], 0
        )
        self.assertEqual(
            coverage["full_2mm_R3_to_fixed_R5_relief_margin_case_count"], 0
        )
        direct = realized["direct_all_4704_guide_to_floor_counts"]
        self.assertEqual(direct["zero_positive_common_volume_case_count"], 4692)
        self.assertEqual(direct["positive_common_volume_case_count"], 12)
        sampled = realized["sampled_endpoint_collision_counts"]
        self.assertEqual(sampled["self_collision"]
                         ["positive_collision_evaluation_count"], 1213)
        self.assertEqual(sampled["self_collision"]
                         ["unique_positive_pair_count"], 34)
        self.assertEqual(sampled["own_floor_leaf_collision"]
                         ["positive_collision_evaluation_count"], 94)
        self.assertEqual(sampled["own_floor_leaf_collision"]
                         ["unique_positive_pair_count"], 8)
        self.assertEqual(sampled["exact_active_local_rebased_sibling_collision"]
                         ["positive_collision_evaluation_count"], 344)
        self.assertEqual(sampled["exact_active_local_rebased_sibling_collision"]
                         ["unique_positive_pair_count"], 8)
        self.assertFalse(any(realized["authority"].values()))
        self.assertEqual(
            realized["report_sha256"],
            "bbbbf2e228edeacd74e4c83b3bec1ff30fafa7f97ccfb1fc97b0eb13aee13eea",
        )
        realized_hashes = {
            name: row["sha256"]
            for name, row in realized["artifact_binding"].items()
        }
        self.assertEqual(
            realized_hashes["audit_source"],
            "1484275a2cbaf16a4163714a90bee90dde66c470ebb601facadc75ba2ecf8b2c",
        )
        self.assertEqual(
            realized_hashes["audit"],
            "329b2cb06801cfd90d78a890983f2225788bd7d8ff6cabe817f3e8324ed02e6e",
        )

        blockers = {row["code"]: row for row in report["blockers"]}
        self.assertIn(
            "replacement_carriage_transition_physical_authority_open",
            blockers,
        )
        self.assertIn(
            "replacement_load_wear_physical_qualification_incomplete",
            blockers,
        )
        self.assertIn(
            "analytic_C1_biarcs_not_positive_volume_or_compression_compatible",
            blockers,
        )
        self.assertIn(
            "successor_prototype_realized_placement_and_collision_failures",
            blockers,
        )
        successor_blocker = blockers[
            "successor_prototype_realized_placement_and_collision_failures"
        ]
        self.assertEqual(successor_blocker["realized_tangent_match_case_count"], 0)
        self.assertEqual(successor_blocker["full_2mm_relief_margin_case_count"], 0)
        self.assertEqual(
            successor_blocker["guide_floor_all_case_counts"]
            ["positive_common_volume_case_count"],
            12,
        )

    def test_integrity_rejects_tampering_and_written_report_is_current(self):
        audit = acceptance._load(acceptance.SUCCESSOR_PROTOTYPE_AUDIT_PATH)
        self.assertTrue(acceptance._successor_prototype_sources_current(audit))
        stale_audit = deepcopy(audit)
        stale_audit["input_hashes"]["source"] = "0" * 64
        self.assertFalse(
            acceptance._successor_prototype_sources_current(stale_audit)
        )
        realized_audit = acceptance._load(
            acceptance.SUCCESSOR_PLACEMENT_COLLISION_AUDIT_PATH
        )
        self.assertTrue(
            acceptance._successor_placement_collision_sources_current(
                realized_audit
            )
        )
        stale_realized = deepcopy(realized_audit)
        stale_realized["input_hashes"]["prototype_STEP"] = "0" * 64
        self.assertFalse(
            acceptance._successor_placement_collision_sources_current(
                stale_realized
            )
        )

        current = deepcopy(self.report)
        acceptance.validate_report_integrity(current)
        current["gates"]["positive_volume_R3_follower_CAD_provenance"] = False
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            acceptance.validate_report_integrity(current)

        generated = acceptance.write_outputs(self.report)
        written = json.loads(acceptance.OUTPUT_JSON.read_text(encoding="utf-8"))
        self.assertEqual(generated["report_sha256"], written["report_sha256"])
        acceptance.validate_report_integrity(written)


if __name__ == "__main__":
    unittest.main(verbosity=2)
