"""Focused contracts for the manufactured cap-to-live-tail support trade."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import cap_live_tail_manufactured_support_trade as trade


class ManufacturedSupportTradeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = trade.analyze()
        cls.by_id = {
            row["id"]: row for row in cls.report["candidates"]
        }

    def test_trade_is_bounded_fail_closed_and_does_not_change_motion(self):
        self.assertEqual(
            self.report["schema"],
            "cap-live-tail-manufactured-support-trade/v1",
        )
        self.assertEqual(self.report["status"], "DESIGN_NO_GO")
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["assembly_integration_authorized"])
        self.assertFalse(self.report["upstream_motion_modified"])
        self.assertFalse(
            self.report["free_mathematical_curve_is_support_authority"]
        )
        self.assertEqual(self.report["surviving_candidate_count"], 0)
        self.assertEqual(self.report["surviving_candidate_ids"], [])
        self.assertFalse(self.report["prototype"]["created"])
        self.assertIsNone(self.report["prototype"]["path"])

    def test_wire_range_sets_literal_surface_and_groove_bounds(self):
        scope = self.report["scope"]
        self.assertEqual(scope["wire_diameter_range_mm"], [0.2, 0.5])
        self.assertEqual(scope["required_growth_states_per_tooth"], 50)
        self.assertEqual(scope["required_raw_loci"], 2400)
        self.assertAlmostEqual(
            scope["minimum_convex_support_surface_radius_for_range_mm"],
            2.75,
        )
        self.assertAlmostEqual(
            scope["minimum_polished_groove_clear_width_for_range_mm"],
            0.65,
        )
        self.assertTrue(scope["not_a_universal_impossibility_proof"])

    def test_fixed_and_mouth_only_features_fail_exact_route_geometry(self):
        fixed = self.by_id["integral_fixed_PEEK_cap_horn"]
        self.assertTrue(
            fixed["positive_volume_manufactured_contact_defined"]
        )
        self.assertEqual(fixed["evidence"]["raw_locus_count"], 2400)
        self.assertEqual(
            fixed["evidence"]["core_crossing_locus_count"], 1000
        )
        self.assertGreater(
            fixed["evidence"]["required_R3_lateral_sweep_mm"],
            fixed["evidence"]["authorized_sector_margin_mm"],
        )
        self.assertGreater(
            fixed["evidence"]["required_capture_width_mm"],
            fixed["evidence"]["available_mouth_width_mm"],
        )
        self.assertGreater(
            fixed["evidence"]["independent_R3_overlap_mm"], 0.0
        )

        mouth = self.by_id["M0_cammed_mouth_only_PEEK_fingers"]
        self.assertTrue(mouth["exact_gates"]["M0_engage_extract_retract_law"])
        self.assertEqual(mouth["evidence"]["passing_route_cases"], 1332)
        self.assertEqual(mouth["evidence"]["required_route_cases"], 7200)
        self.assertLess(
            mouth["evidence"]["minimum_core_center_clearance_mm"],
            mouth["evidence"]["required_core_center_clearance_mm"],
        )
        self.assertLess(
            mouth["evidence"]["minimum_prior_copper_clearance_mm"],
            mouth["evidence"]["required_copper_clearance_mm"],
        )

    def test_existing_positive_volume_formers_remain_exact_no_go(self):
        shoe = self.by_id["machine_fixed_full_depth_split_PEEK_shoe"]
        self.assertEqual(shoe["evidence"]["passing_route_cases"], 0)
        self.assertEqual(shoe["evidence"]["required_route_cases"], 6480)
        self.assertLess(
            shoe["evidence"]["minimum_common_corridor_margin_mm"], 0.0
        )
        self.assertLess(shoe["evidence"]["minimum_chuck_clearance_mm"], 0.0)

        retained = self.by_id["retained_stator_R3_end_former"]
        self.assertEqual(retained["evidence"]["turn_count"], 50)
        self.assertEqual(retained["evidence"]["selected_lane_status"], "FAIL")
        self.assertLess(
            retained["evidence"]["minimum_solid_clearance_mm"], 0.0
        )
        self.assertEqual(
            retained["evidence"]["motor_axial_cavity_status"], "UNPROVEN"
        )

    def test_smallest_moving_shoe_closes_mechanics_but_not_wire_route(self):
        selected = self.by_id[
            "M1_selected_M2_phased_single_gimbal_polished_shoe"
        ]
        for name in (
            "three_laws_selected",
            "M0_fail_safe_retraction",
            "rigid_clearance",
            "300rpm_load_balance_interlock",
        ):
            self.assertTrue(selected["exact_gates"][name], name)
        self.assertFalse(
            selected["exact_gates"]["all_turn_growth_R3_tail_routes"]
        )
        self.assertFalse(selected["exact_gates"]["elastic_R3_contact"])
        self.assertEqual(
            selected["evidence"]["copper_clear_R3_tail_candidates"], 0
        )
        self.assertEqual(selected["evidence"]["bounded_tail_candidate_count"], 6240)
        self.assertEqual(selected["evidence"]["turn_growth_state_count"], 50)
        self.assertGreater(
            selected["evidence"]["minimum_deployed_flyer_clearance_mm"], 2.0
        )
        self.assertGreater(
            selected["evidence"]["forced_retracted_M1_margin_mm"], 0.0
        )

    def test_successor_rides_aggregate_boundary_without_packed_tangent(self):
        successor = self.report["recommended_successor"]
        self.assertEqual(
            successor["status"],
            "PROMISING_AGGREGATE_AUTHORITY_ONLY_NOT_CAD_PROVED",
        )
        self.assertFalse(successor["new_commanded_axis"])
        self.assertFalse(successor["upstream_motion_change"])
        dofs = successor["smallest_additional_physical_DOF"]
        self.assertEqual(len(dofs), 2)
        self.assertAlmostEqual(
            dofs[0]["minimum_current_aggregate_tracking_span_mm"],
            5.472581832289956,
        )
        self.assertAlmostEqual(
            dofs[1]["minimum_current_half_slot_tracking_span_mm"],
            0.7187467281538217,
        )
        surface = successor["manufactured_support_surface"]
        self.assertEqual(surface["wire_center_radius_range_mm"], [3.1, 3.25])
        self.assertAlmostEqual(surface["polished_groove_clear_width_mm"], 0.65)
        aggregate = successor["aggregate_tangent_evaluation"]
        self.assertTrue(aggregate["aggregate_geometry_authorized"])
        self.assertFalse(aggregate["exact_strand_packing_predicted"])
        self.assertTrue(aggregate["convex_supporting_tangent_exists"])
        self.assertTrue(
            aggregate["all_50_growth_states_classified_without_strand_order"]
        )
        self.assertEqual(
            aggregate["active_prior_positive_volume_intrusion_mm3"], 0.0
        )

    def test_obsolete_terminal_tangent_detour_is_not_promoted(self):
        comparison = self.report["recommended_successor"][
            "obsolete_packed_tangent_comparison"
        ]
        self.assertAlmostEqual(comparison["source_target_chord_mm"], 0.5)
        self.assertAlmostEqual(
            comparison["five_arc_detour_length_mm"], 36.90399156124403
        )
        self.assertFalse(comparison["detour_is_required_by_aggregate_authority"])
        self.assertTrue(
            comparison["shallow_R3_bow_passes_when_terminal_tangent_not_enforced"]
        )
        gates = self.report["recommended_successor"]["gates"]
        self.assertTrue(gates["aggregate_contact_authority_current_job"])
        self.assertTrue(gates["taut_span_meets_R3_away_from_contact"])
        self.assertFalse(gates["positive_volume_slide_gimbal_shoe_CAD_complete"])
        self.assertFalse(gates["all_2400_raw_pose_route_and_rigid_clearance"])

    def test_payload_and_source_hash_tampering_are_rejected(self):
        payload_tamper = deepcopy(self.report)
        payload_tamper["prototype"]["created"] = True
        with self.assertRaisesRegex(ValueError, "payload hash mismatch"):
            trade.validate_report(payload_tamper)

        source_tamper = deepcopy(self.report)
        key = "cad/permanent_cap_production_review.py"
        source_tamper["source_hashes"][key] = "0" * 64
        source_tamper["report_sha256"] = trade._canonical_hash(source_tamper)
        with self.assertRaisesRegex(ValueError, "source hash mismatch"):
            trade.validate_report(source_tamper)

    def test_generated_report_is_current_and_self_validating(self):
        stored = json.loads(trade.JSON_OUT.read_text(encoding="utf-8"))
        trade.validate_report(stored)
        self.assertEqual(stored["report_sha256"], self.report["report_sha256"])
        self.assertTrue(trade.MD_OUT.is_file())


if __name__ == "__main__":
    unittest.main(verbosity=2)
