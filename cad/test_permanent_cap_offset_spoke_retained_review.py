"""Regression gates for the exact retained offset-spoke successor."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
import unittest

import permanent_cap_offset_spoke_retained_review as retained


class RetainedOffsetSpokeReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = retained.analyze()

    def test_arm_is_one_structurally_connected_solid(self) -> None:
        gates = self.report["geometry_gates"]
        self.assertEqual(self.report["printed_arm"]["solid_count"], 1)
        self.assertTrue(gates["printed_arm_exactly_one_solid"])
        self.assertTrue(gates["structural_housing_wall_ge_2p4mm"])
        self.assertTrue(gates["rear_pocket_septum_ge_2p4mm"])
        self.assertTrue(gates["front_pocket_septum_ge_2p4mm"])
        self.assertTrue(gates["rail_and_tower_sections_ge_2p4mm"])
        self.assertAlmostEqual(
            self.report["retention"]["minimum_housing_wall_mm"],
            2.4,
            places=7,
        )

    def test_all_four_screws_terminate_in_supported_blind_bosses(self) -> None:
        retention = self.report["retention"]
        self.assertEqual(retention["stack_count"], 4)
        self.assertTrue(retention["all_screws_end_in_positive_blind_material"])
        self.assertTrue(retention["all_stacks_within_pocket_axial_envelope"])
        self.assertTrue(retention["all_caps_and_posts_single_solid"])
        for stack in retention["stacks"]:
            self.assertEqual(stack["screw_interval_local_mm"], [0.0, 6.0])
            self.assertEqual(stack["insert_interval_local_mm"], [1.7, 6.0])
            self.assertEqual(stack["boss_interval_local_mm"], [1.0, 6.6])
            self.assertEqual(stack["retainer_face_interval_local_mm"], [6.6, 7.8])
            self.assertAlmostEqual(stack["full_insert_engagement_mm"], 4.3)
            self.assertAlmostEqual(
                stack["blind_positive_material_ahead_of_tip_mm"], 1.8,
            )
            self.assertTrue(stack["nothing_projects_behind_pocket_floor"])
            self.assertTrue(stack["nothing_projects_ahead_of_pocket_front"])
            self.assertTrue(stack["fastener_terminates_in_positive_blind_material"])
            self.assertGreater(stack["insert_heat_set_interference_volume_mm3"], 0.0)
            self.assertEqual(
                stack["McMaster_92125A126_material"], "18-8 stainless steel",
            )
            self.assertEqual(stack["catalog_tensile_strength_psi"], 70000.0)
            self.assertGreater(stack["screw_tensile_margin"], 3.0)
            self.assertIn("continuous OD7.6 printed boss", stack["closed_structural_load_path"])

    def test_every_screw_has_positive_material_path_to_spoke_and_collar(self) -> None:
        audit = self.report["structural_load_path"]
        shared = audit["shared_positive_overlap_chain"]
        self.assertTrue(audit["all_four_positive_material_under_screws"])
        self.assertTrue(audit["all_four_exact_head_floor_bearing_contacts"])
        self.assertTrue(audit["all_four_exact_boss_floor_contacts"])
        self.assertTrue(audit["all_four_paths_reach_spoke_and_collar"])
        self.assertFalse(audit["any_counterweight_retainer_unsupported_over_open_air"])
        self.assertGreater(shared["collar_to_deep_spoke_overlap_mm3"], 1.0)
        self.assertGreater(shared["deep_core_to_counterrail_overlap_mm3"], 1.0)
        self.assertGreater(shared["counterrail_to_outboard_tower_overlap_mm3"], 1.0)
        self.assertTrue(shared["core_single_solid"])
        self.assertTrue(shared["final_arm_single_solid"])
        for stack in audit["stacks"]:
            self.assertAlmostEqual(stack["floor_thickness_mm"], 1.0)
            self.assertGreater(
                stack["positive_annular_floor_material_under_screw_mm3"], 1.0,
            )
            self.assertGreater(stack["floor_material_coverage_ratio"], 0.999)
            self.assertLessEqual(
                stack["exact_screw_head_to_floor_bearing_contact_mm"], 1.0e-7,
            )
            self.assertLessEqual(
                stack["exact_retainer_boss_to_floor_contact_mm"], 1.0e-7,
            )
            self.assertGreater(stack["housing_to_parent_member_overlap_mm3"], 1.0)
            self.assertTrue(stack["positive_material_path_to_spoke_and_collar"])
            self.assertFalse(stack["unsupported_over_open_air"])

    def test_three_weighed_spacer_posts_close_every_axial_stack(self) -> None:
        for stack in self.report["retention"]["stacks"]:
            posts = stack["three_spacer_posts"]
            self.assertEqual(len(posts), 3)
            self.assertGreater(stack["spacer_length_mm"], 0.2)
            self.assertAlmostEqual(stack["slug_to_spacer_axial_float_mm"], 0.0)
            self.assertAlmostEqual(stack["spacer_to_face_axial_float_mm"], 0.0)
            for post in posts:
                self.assertGreater(post["volume_mm3"], 0.0)
                self.assertGreater(post["mass_g"], 0.0)

    def test_occ_two_plane_solution_includes_all_named_rotating_parts(self) -> None:
        total = self.report["exact_rotating_mass_properties"]
        lengths = self.report["slug_length_solution_mm"]
        self.assertEqual(set(lengths), {pocket.id for pocket in retained.POCKETS})
        for value in lengths.values():
            self.assertGreaterEqual(value, retained.SLUG_MIN_LENGTH_MM)
            self.assertLessEqual(value, retained.SLUG_MAX_LENGTH_MM)
        self.assertLess(total["static_imbalance_g_mm"], 1.0e-6)
        self.assertLess(total["couple_imbalance_g_mm2"], 1.0e-6)
        names = {row["name"] for row in self.report["exact_rotating_mass_rows"]}
        self.assertIn("DIN_988_12x18x1_axial_shim", names)
        self.assertIn("R64_ceramic_toroid_guide", names)
        self.assertIn("extended_hollow_shaft", names)
        self.assertIn("shifted_flyer_pulley_exact_1_to_1", names)
        self.assertIn("flyer_pulley_radial_M3x8_set_screw", names)
        self.assertIn("flyer_pulley_radial_M3_short_insert", names)

    def test_continuous_clearance_certificate_and_exact_pose_pass(self) -> None:
        clearance = self.report["clearance"]
        continuous = clearance["continuous_360_certificate"]
        self.assertEqual(continuous["angle_domain_deg"], [0.0, 360.0])
        self.assertTrue(continuous["passes_2p2mm"])
        self.assertGreaterEqual(continuous["minimum_mm"], 2.2)
        for value in clearance["exact_OCC_at_M1_M2_zero_mm"].values():
            self.assertGreaterEqual(value, 2.2)

    def test_insert_boss_exception_is_explicit_and_physically_gated(self) -> None:
        gates = self.report["geometry_gates"]
        retention = self.report["retention"]
        self.assertTrue(gates["explicit_insert_boss_exception_wall_ge_1p5mm"])
        self.assertAlmostEqual(retention["minimum_insert_boss_wall_mm"], 1.7)
        self.assertFalse(retention["physical_pull_proof_complete"])
        self.assertFalse(retention["fit_coupon_complete"])
        self.assertIn("0.35 N m", retention["assembly_torque_limit"])
        for stack in retention["stacks"]:
            self.assertEqual(stack["physical_pull_proof_required_N"], 20.0)
            self.assertFalse(stack["physical_pull_proof_complete"])

    def test_force_vector_is_hash_bound_and_current_1_to_1_drive_fails_closed(self) -> None:
        duty = self.report["force_vector_M2_duty"]
        self.assertTrue(duty["schema_ok"])
        self.assertTrue(duty["self_hash_ok"])
        self.assertTrue(duty["source_hash_ok"])
        self.assertEqual(duty["mechanical_ratio"], 1.0)
        self.assertTrue(duty["mechanical_ratio_locked_by_upstream"])
        self.assertFalse(duty["motor_gate_ge_2"])
        self.assertFalse(duty["pulley_gate_ge_2"])
        self.assertLess(duty["motor_margin"], 2.0)
        self.assertLess(duty["pulley_margin"], 2.0)
        self.assertFalse(duty["motor_rotor_inertia_bounded"])
        self.assertIn("stronger closed-loop", duty["required_successor"])
        self.assertIn("closed-loop NEMA17", duty["required_motor_form_factor"])

    def test_catalog_provenance_labels_and_release_boundary(self) -> None:
        contracts = self.report["source_contracts"]["step_parts"]
        screw = contracts["screw"]
        self.assertEqual(
            screw["id"], "iso10642_socket_countersunk_screw_m3x6",
        )
        self.assertEqual(screw["sha256"], screw["catalog_sha256"])
        self.assertIn("zero exact matches", contracts["insert"]["catalog_search"])
        self.assertIn("zero matches", contracts["DIN_988_shim"]["catalog_search"])
        labels = self.report["hardware_labels"]
        self.assertEqual(len(labels["counterweight_axial_fasteners"]), 4)
        self.assertEqual(len(labels["counterweight_inserts"]), 4)
        self.assertEqual(
            len(labels["shaft_clamp_radial_holes_are_not_counterweights"]), 2,
        )
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["assembly_integration_authorized"])
        self.assertTrue(self.report["physical_balance_required"])
        self.assertIn("CURRENT_M2_DRIVE", self.report["status"])
        retained.validate_report_integrity(self.report)


if __name__ == "__main__":
    unittest.main()
