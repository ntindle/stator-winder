"""Regression gates for the fail-closed normal-GOAL integration candidate."""

from __future__ import annotations

import math
import unittest

import integrated_release_candidate as rc


class IntegratedReleaseCandidateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = rc.analyze()

    def test_targeted_reference_geometry_passes_but_release_stays_closed(self) -> None:
        self.assertEqual(
            self.report["status"],
            "REFERENCE_GEOMETRY_PASS_RELEASE_GATES_OPEN",
        )
        self.assertTrue(all(self.report["geometry"]["checks"].values()))
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["main_assembly_replacement_authorized"])
        self.assertFalse(self.report["release_gates"]["production_authorized"])
        self.assertTrue(
            self.report["release_gates"]["full_raw_cycle_collision_regenerated"]
        )
        packet = self.report["visual_review"]["snapshot_packet"]
        reviewed = self.report["visual_review"]["reviewed"]
        self.assertEqual(len(packet), 4 if reviewed else 0)
        self.assertEqual(len(set(packet)), len(packet))
        self.assertEqual(
            self.report["release_gates"][
                "mandatory_integrated_snapshot_packet_reviewed"
            ],
            reviewed,
        )
        for relative in packet:
            self.assertGreaterEqual(
                (rc.ROOT / relative).stat().st_mtime_ns,
                rc.STEP_OUT.stat().st_mtime_ns,
            )

    def test_exact_source_level_relocations_are_explicit(self) -> None:
        transforms = self.report["transforms"]
        self.assertEqual(
            transforms["M2_static_bearing_drive_and_pulley_module_mm"],
            [0.0, 0.0, -10.0],
        )
        self.assertEqual(
            transforms["entry_bracket_eyelet_and_mounting_hardware_mm"],
            [0.0, 0.0, -4.25],
        )
        self.assertEqual(
            transforms["base_rail_frame_window_mm"], [0.0, 0.0, -7.5]
        )
        self.assertAlmostEqual(
            transforms[
                "felt_moving_pad_backing_spring_thrust_and_wingnut_mm"
            ][2],
            -(0.5 - 0.22352),
            places=9,
        )

    def test_frame_window_closes_candidate_motor_rear_clearance(self) -> None:
        frame = self.report["geometry"]["frame_window_interface"]
        self.assertAlmostEqual(frame["candidate_motor_rear_z_mm"], -195.2, places=5)
        self.assertAlmostEqual(frame["selected_integrated_boundary_z_mm"], -197.5)
        self.assertAlmostEqual(frame["selected_clearance_mm"], 2.3, places=5)
        self.assertGreaterEqual(frame["selected_clearance_mm"], 2.2)
        self.assertAlmostEqual(
            frame["additional_rearward_boundary_shift_required_mm"], 0.0
        )

    def test_shifted_base_rails_leave_no_attachment_over_open_air(self) -> None:
        audit = self.report["geometry"]["frame_relocation_attachment_audit"]
        self.assertGreater(audit["attachment_occurrence_count"], 0)
        self.assertTrue(audit["all_fully_inside_shifted_rail_longitudinal_span"])
        self.assertTrue(
            audit["all_occurrences_project_over_real_rail_longitudinal_material"]
        )
        self.assertTrue(audit["all_groups_have_supported_attachment_chain"])
        self.assertFalse(audit["any_attachment_over_open_air"])
        for row in audit["attachments"]:
            self.assertGreater(row["longitudinal_projection_overlap_mm"], 0.0)
            self.assertTrue(row["fully_inside_shifted_rail_longitudinal_span"])
            self.assertTrue(row["projects_over_real_rail_longitudinal_material"])
        for group in audit["attachment_groups"]:
            self.assertTrue(
                group["all_members_supported_by_direct_or_chained_contact"]
            )

    def test_raw_entry_failure_is_consumed_and_redesign_has_reserve(self) -> None:
        geom = self.report["geometry"]
        clearance = geom["clearances_mm"]
        self.assertAlmostEqual(
            clearance["extended_hollow_shaft_to_entry_bracket_mm"],
            2.5,
            places=5,
        )
        self.assertAlmostEqual(
            clearance["flyer_P30_to_entry_bracket_mm"],
            2.5,
            places=5,
        )
        self.assertGreaterEqual(
            clearance["extended_hollow_shaft_to_entry_eyelet_mm"], 2.2
        )
        self.assertGreaterEqual(
            clearance["flyer_P30_to_entry_eyelet_mm"], 2.2
        )
        bridge = geom["entry_wire_bridge"]
        self.assertAlmostEqual(bridge["straight_axial_bridge_length_mm"], 5.5)
        self.assertAlmostEqual(bridge["entry_additional_rear_shift_mm"], 0.75)
        self.assertAlmostEqual(bridge["shaft_additional_rear_extension_mm"], 0.75)
        self.assertAlmostEqual(bridge["net_bridge_length_change_mm"], 0.0)
        self.assertEqual(bridge["wire_to_entry_wall_overlap_mm3"], 0.0)
        self.assertGreaterEqual(
            bridge["radial_wall_clearance_exact_BREP_mm"], 1.4
        )
        attachment = geom["entry_module_attachment_audit"]
        self.assertTrue(attachment["entry_bracket_one_valid_solid"])
        self.assertTrue(
            attachment["all_mounting_hardware_contacts_bracket_and_post"]
        )
        self.assertEqual(
            attachment["preserved_dancer_fixed_screw_to_bracket_distance_mm"],
            0.0,
        )
        raw = geom["frozen_raw_clearance_diagnostic_reconciliation"]
        self.assertEqual(raw["old_shaft_to_entry_bracket_mm"], 1.0)
        self.assertEqual(raw["old_flyer_P30_to_entry_bracket_mm"], 1.75)
        self.assertEqual(raw["candidate_required_target_mm"], 2.2)
        self.assertTrue(
            raw["full_raw_rerun_was_required_by_frozen_diagnostic"]
        )
        self.assertTrue(raw["current_exact_full_raw_rerun_consumed"])
        self.assertTrue(
            self.report["release_gates"][
                "entry_reference_BREP_shaft_and_flyer_P30_clear_ge_2p2mm"
            ]
        )
        self.assertTrue(
            self.report["release_gates"]["full_raw_cycle_collision_regenerated"]
        )

    def test_integrated_p30_mass_set_is_rebalanced_with_new_slug_cuts(self) -> None:
        geom = self.report["geometry"]
        solution = geom["integrated_slug_length_solution_mm"]
        old = geom["superseded_retained_review_slug_lengths_mm"]
        self.assertEqual(set(solution), {pocket.id for pocket in rc.retained.POCKETS})
        self.assertNotEqual(solution, old)
        for value in solution.values():
            self.assertGreaterEqual(value, rc.retained.SLUG_MIN_LENGTH_MM)
            self.assertLessEqual(value, rc.retained.SLUG_MAX_LENGTH_MM)
        mass = geom["integrated_rotating_mass_properties"]
        self.assertLess(mass["static_imbalance_g_mm"], 1.0e-6)
        self.assertLess(mass["couple_imbalance_g_mm2"], 1.0e-6)
        names = {row["name"] for row in geom["integrated_rotating_mass_rows"]}
        self.assertIn("flyer_pulley", names)
        self.assertEqual(sum(name == "flyer_pulley" for name in names), 1)
        self.assertNotIn("shifted_flyer_pulley_exact_1_to_1", names)
        pulley = next(
            row for row in geom["integrated_rotating_mass_rows"]
            if row["name"] == "flyer_pulley"
        )
        self.assertEqual(pulley["mass_g"], 28.0)
        self.assertEqual(pulley["izz_about_M2_axis_g_mm2"], 3000.0)
        self.assertTrue(pulley["supplied_clamp_bolt_included"])

    def test_configured_wire_physically_contacts_felt_and_dancer(self) -> None:
        contact = self.report["geometry"]["intended_contacts"]
        self.assertLessEqual(contact["wire_to_fixed_felt_distance_mm"], 1.0e-7)
        self.assertLessEqual(contact["wire_to_moving_felt_distance_mm"], 1.0e-7)
        self.assertLessEqual(contact["wire_to_dancer_pulley_distance_mm"], 1.0e-7)
        self.assertTrue(
            self.report["release_gates"][
                "configured_0p22352mm_wire_contacts_both_felt_pads_and_dancer"
            ]
        )
        gates = self.report["release_gates"]
        self.assertTrue(gates["felt_preload_spring_and_drag_sizing_PASS"])
        self.assertTrue(
            gates["felt_actual_and_0p5mm_changeover_contact_geometry_PASS"]
        )
        self.assertFalse(gates["felt_operating_drag_pull_gauge_calibrated"])
        sizing = self.report["source_contracts"]["felt_preload_and_drag_sizing"]
        self.assertEqual(sizing["selected_spring"], "McMaster 94125K614")
        self.assertAlmostEqual(sizing["wingnut_travel_turns"], 5.17417408721556)

    def test_actual_caps_and_retained_arm_have_exact_cross_module_clearance(self) -> None:
        geom = self.report["geometry"]
        self.assertGreaterEqual(
            geom["clearances_mm"]["actual_deep_cap_pair_to_retained_arm_mm"],
            2.2,
        )
        self.assertAlmostEqual(
            geom["clearances_mm"]["retained_arm_to_shifted_flyer_block_mm"],
            2.88,
            places=6,
        )
        self.assertLessEqual(
            max(geom["unintended_overlaps_mm3"].values()), 1.0e-5
        )

    def test_all_new_hardware_occurrence_contracts_are_present(self) -> None:
        hardware = self.report["hardware_occurrence_contract"]
        self.assertEqual(hardware["counterweight_stacks"], 6)
        self.assertEqual(hardware["rear_counterweight_stacks"], 4)
        self.assertEqual(hardware["front_balance_trim_stacks"]["count"], 2)
        self.assertEqual(
            hardware["front_balance_trim_stacks"]["M2x8_screws"], 2
        )
        self.assertEqual(
            hardware["front_balance_trim_stacks"][
                "M2_standard_heat_set_inserts"
            ],
            2,
        )
        self.assertEqual(
            hardware["one_piece_PEEK_flyer_guide"]["M2x6_screws"], 3
        )
        self.assertEqual(
            hardware["one_piece_PEEK_flyer_guide"][
                "M2_standard_heat_set_inserts"
            ],
            3,
        )
        self.assertEqual(
            hardware["one_piece_PEEK_flyer_guide"][
                "obsolete_ceramic_torus"
            ],
            0,
        )
        self.assertEqual(hardware["cap_retention"]["M2x20_screws"], 3)
        self.assertEqual(hardware["cap_retention"]["M2_front_washers"], 3)
        self.assertEqual(hardware["cap_retention"]["M2_rear_washers"], 3)
        self.assertEqual(hardware["cap_retention"]["M2_nyloc_nuts"], 3)
        self.assertEqual(
            hardware["successor_drive"]["flyer_stock_supplied_M2_clamp_bolts"],
            1,
        )
        self.assertEqual(
            hardware["successor_drive"]["Leadshine_CS_M21708_NEMA17"], 1
        )
        self.assertEqual(hardware["entry_module"]["rear_shift_mm"], 4.25)
        self.assertEqual(hardware["entry_module"]["M5x12_base_screws"], 2)
        self.assertEqual(
            hardware["successor_drive"][
                "NBK_BNW_M3_upper_bound_hole_path_witnesses"
            ],
            2,
        )
        self.assertEqual(
            hardware["successor_drive"][
                "NBK_BNW_M3x12_set_screw_upper_bound_witnesses"
            ],
            2,
        )
        self.assertEqual(
            hardware["successor_drive"][
                "official_NBK_P30_stock_vendor_occurrences"
            ],
            2,
        )
        self.assertTrue(
            hardware["successor_drive"][
                "official_stock_split_clamp_and_bolt_in_each_vendor_occurrence"
            ]
        )
        rotating = rc.retained_rotating_parts()
        self.assertEqual(sum("M3x6_screw" in name for name in rotating), 4)
        self.assertEqual(sum("94459A130_insert" in name for name in rotating), 4)
        self.assertEqual(
            sum(name.startswith("front_trim_B777_") for name in rotating), 2
        )
        self.assertEqual(
            sum(name.startswith("front_trim_hardware_") for name in rotating),
            6,
        )
        self.assertEqual(
            sum(
                name.startswith("flyer_PEEK_guide_retention_screw_")
                for name in rotating
            ),
            3,
        )

    def test_all_six_balance_fasteners_terminate_in_real_material(self) -> None:
        attachment = self.report["geometry"][
            "integrated_six_stack_attachment_audit"
        ]
        self.assertEqual(attachment["stack_count"], 6)
        self.assertEqual(
            attachment["status"], "GEOMETRY_PASS_PHYSICAL_PULL_OPEN"
        )
        self.assertTrue(
            attachment[
                "all_four_rear_stacks_have_closed_printed_boss_load_paths"
            ]
        )
        self.assertTrue(
            attachment["both_front_stacks_thread_into_blind_spoke_inserts"]
        )
        self.assertTrue(
            attachment[
                "all_six_screws_terminate_in_positive_printed_material"
            ]
        )
        self.assertFalse(attachment["any_balance_fastener_over_open_air"])
        self.assertFalse(attachment["physical_pull_proof_complete"])
        rear = attachment["rear_M3_retained_stacks"]
        self.assertEqual(rear["stack_count"], 4)
        for row in rear["stacks"]:
            self.assertTrue(row["fastener_terminates_in_positive_blind_material"])
            self.assertIn("heat-set insert", row["closed_structural_load_path"])
        front = attachment["front_M2_blind_spoke_stacks"]
        self.assertEqual(len(front), 2)
        for row in front:
            self.assertGreaterEqual(
                row["screw_tip_clearance_behind_insert_mm"], 0.5
            )
            self.assertGreaterEqual(
                row["blind_printed_material_behind_pilot_mm"], 2.4
            )
            self.assertTrue(
                row[
                    "fastener_terminates_in_positive_blind_printed_material"
                ]
            )
        self.assertTrue(
            self.report["release_gates"][
                "all_six_counterweight_fasteners_have_closed_material_load_paths"
            ]
        )

    def test_released_L79_stock_D10_shaft_geometry_and_artifact_gate(self) -> None:
        shaft = rc.shaft_with_integrated_p30_flats()
        bbox = shaft.bounding_box()
        self.assertEqual(len(list(shaft.solids())), 1)
        self.assertTrue(shaft.is_valid)
        self.assertAlmostEqual(float(bbox.min.Z), -110.75, places=6)
        self.assertAlmostEqual(float(bbox.max.Z), -31.75, places=6)
        self.assertAlmostEqual(float(bbox.size.Z), 79.0, places=5)
        self.assertEqual(
            shaft.label,
            "released_M2_001_Rev_D_flyer_shaft_D10_ID6_ID9_L79",
        )
        brep = self.report["geometry"]["released_M2_001_Rev_D_shaft_BREP"]
        self.assertEqual(brep["solid_count"], 1)
        self.assertEqual(
            brep["cylindrical_surface_radii_mm"], [3.0, 4.5, 5.0, 6.0]
        )
        self.assertEqual(brep["wire_mouth_toroidal_face_count"], 2)
        self.assertAlmostEqual(brep["wire_mouth_fillet_radius_mm"], 0.5)
        self.assertEqual(brep["D10_seat_length_mm"], 18.5)
        self.assertEqual(brep["ID6_to_ID9_transition_length_mm"], 3.0)
        self.assertEqual(
            [
                (
                    row["normal"],
                    round(row["station_from_rear_datum_mm"], 6),
                    round(row["axial_length_mm"], 6),
                )
                for row in brep["indexed_flats"]
            ],
            [
                ("minus_y", 64.75, 5.0),
                ("plus_x", 64.75, 5.0),
            ],
        )
        self.assertTrue(
            self.report["release_gates"][
                "new_L79_stock_D10_shaft_STEP_drawing_RFQ_and_arm_flats_released"
            ]
        )
        contract = self.report["source_contracts"][
            "released_M2_001_Rev_D_shaft"
        ]
        self.assertEqual(contract["revision"], "D")
        self.assertEqual(
            contract["source_sha256"], rc._sha256(rc.Path(rc.flyer_shaft_d10.__file__))
        )
        self.assertEqual(contract["STEP_sha256"], rc._sha256(rc.RELEASED_SHAFT_STEP))
        self.assertEqual(
            contract["drawing_PDF_sha256"], rc._sha256(rc.RELEASED_SHAFT_PDF)
        )
        self.assertEqual(
            contract["custom_manifest_sha256"],
            rc._sha256(rc.CUSTOM_PARTS_MANIFEST),
        )
        self.assertEqual(
            contract["release_catalog_sha256"], rc._sha256(rc.RELEASE_CATALOG)
        )
        self.assertFalse(contract["old_70mm_three_flat_artifact_governing"])
        self.assertFalse(
            contract["retired_Rev_C_L80p75_artifacts"]["governing"]
        )
        interface = self.report["geometry"][
            "released_Rev_D_shaft_front_interface"
        ]
        self.assertGreaterEqual(
            interface["shaft_front_setback_from_root_sleeve_front_mm"], 0.25
        )
        self.assertEqual(
            interface["shaft_vs_PEEK_guide_outer_overlap_mm3"], 0.0
        )
        self.assertEqual(interface["shaft_vs_flyer_wire_overlap_mm3"], 0.0)
        self.assertTrue(interface["source_gate"])
        self.assertFalse(
            any(
                "no released STEP/drawing/RFQ" in blocker
                for blocker in self.report["open_blockers"]
            )
        )

    def test_corrected_arm_bore_root_sleeve_and_stock_clamp_are_physical(self) -> None:
        geom = self.report["geometry"]
        arm = rc.flyer_successor.revised_retained_arm()
        shaft = rc.shaft_with_integrated_p30_flats()
        self.assertEqual(len(list(arm.solids())), 1)
        self.assertTrue(arm.is_valid)
        self.assertEqual(rc._overlap(arm, shaft), 0.0)
        self.assertAlmostEqual(rc._distance(arm, shaft), 0.05, places=6)
        root = geom["corrected_printed_arm_root_sleeve_load_path"]
        failure = root["failure_found_before_correction"]
        self.assertAlmostEqual(
            failure["retained_arm_vs_released_shaft_overlap_mm3"],
            173.533039504,
            places=5,
        )
        corrected = root["corrected_interface"]
        self.assertEqual(corrected["arm_solid_count"], 1)
        self.assertTrue(corrected["arm_valid"])
        self.assertEqual(corrected["arm_vs_shaft_overlap_mm3"], 0.0)
        self.assertAlmostEqual(
            corrected["arm_to_shaft_radial_clearance_mm"], 0.05, places=6
        )
        web = root["root_sleeve_web"]
        self.assertAlmostEqual(web["radial_ligament_mm"], 2.95)
        self.assertAlmostEqual(
            web["web_to_existing_collar_overlap_mm3"], 725.290071, places=5
        )
        self.assertAlmostEqual(
            web["web_to_main_spoke_overlap_mm3"], 359.358957, places=5
        )
        self.assertAlmostEqual(
            web["web_to_rear_counterrail_overlap_mm3"], 35.849292, places=5
        )
        intended = geom["intended_contacts"]
        self.assertEqual(len(intended["arm_M3x8_to_shaft_flat_distances_mm"]), 2)
        self.assertLessEqual(
            max(intended["arm_M3x8_to_shaft_flat_distances_mm"]), 1.0e-5
        )
        overlap = geom["unintended_overlaps_mm3"]
        self.assertEqual(
            overlap["arm_M3x8_set_screws_vs_released_shaft_max_mm3"], 0.0
        )
        packaging = geom["released_shaft_screw_packaging"]["arm_M3x8"]
        self.assertAlmostEqual(packaging["screw_inward_adjustment_mm"], 0.3)
        for row in packaging["rows"]:
            self.assertAlmostEqual(row["M3x8_length_preserved_mm"], 8.0)
            self.assertAlmostEqual(row["outer_socket_end_radius_mm"], 13.7)
            self.assertAlmostEqual(
                row["projected_screw_insert_engagement_mm"], 5.4, places=6
            )

    def test_released_shaft_fits_wire_handoffs_and_root_strength(self) -> None:
        geom = self.report["geometry"]
        fits = geom["released_shaft_bearing_spacer_collar_fits"]
        self.assertLessEqual(max(fits["6001_bearings"]["distances_mm"]), 1.0e-5)
        self.assertEqual(max(fits["6001_bearings"]["overlaps_mm3"]), 0.0)
        for row in fits["inner_race_spacers"].values():
            self.assertAlmostEqual(row["radial_clearance_mm"], 0.025, places=5)
            self.assertEqual(row["overlap_mm3"], 0.0)
        stock = fits["flyer_P30_stock_D10_clamp"]
        self.assertEqual(stock["official_part_number"], "P30-3GT-BLP-6C-10")
        self.assertEqual(stock["nominal_bore_diameter_mm"], 10.0)
        self.assertEqual(stock["shaft_seat_outer_diameter_mm"], 10.0)
        self.assertLessEqual(stock["contact_distance_mm"], 1.0e-5)
        self.assertEqual(stock["overlap_mm3"], 0.0)
        self.assertEqual(stock["shaft_D10_seat_axial_span_z_mm"], [-110.75, -92.25])
        self.assertLessEqual(fits["DIN988_shim"]["distance_mm"], 1.0e-5)
        self.assertEqual(fits["DIN988_shim"]["overlap_mm3"], 0.0)
        handoffs = geom["released_shaft_wire_handoffs"]
        self.assertEqual(handoffs["rear_entry"]["wire_vs_shaft_overlap_mm3"], 0.0)
        self.assertGreaterEqual(
            handoffs["rear_entry"]["wire_to_shaft_wall_clearance_mm"], 2.75
        )
        self.assertEqual(handoffs["front_root"]["wire_vs_shaft_overlap_mm3"], 0.0)
        self.assertEqual(
            handoffs["front_root"]["wire_vs_corrected_arm_overlap_mm3"], 0.0
        )
        self.assertAlmostEqual(
            handoffs["front_root"]["root_web_to_job_wire_clearance_mm"],
            1.26824,
            places=6,
        )
        self.assertAlmostEqual(
            handoffs["front_root"]["root_web_to_0p5mm_wire_clearance_mm"],
            1.13,
            places=6,
        )
        seam = handoffs["axis_ownership_seam"]
        self.assertEqual(seam["static_axis_run_centerline_end_z_mm"], -42.0)
        self.assertEqual(seam["flyer_guide_centerline_start_z_mm"], -42.0)
        self.assertLessEqual(seam["static_to_flyer_wire_distance_mm"], 1.0e-5)
        self.assertLessEqual(seam["static_to_flyer_wire_overlap_mm3"], 1.0e-5)
        self.assertEqual(seam["static_axis_wire_vs_shaft_wall_overlap_mm3"], 0.0)
        self.assertEqual(seam["static_axis_wire_vs_corrected_arm_overlap_mm3"], 0.0)
        load = geom["corrected_printed_arm_root_sleeve_load_path"][
            "conservative_combined_root_load_case"
        ]
        self.assertAlmostEqual(load["annular_area_mm2"], 139.47886, places=5)
        self.assertAlmostEqual(load["polar_second_moment_J_mm4"], 8201.53131, places=5)
        self.assertAlmostEqual(load["planar_second_moment_I_mm4"], 4100.76565, places=5)
        self.assertAlmostEqual(load["von_Mises_equivalent_MPa"], 1.858132, places=5)
        self.assertAlmostEqual(
            load["safety_factored_equivalent_MPa"], 5.574396, places=5
        )
        self.assertTrue(load["passes_review_allowable"])
        self.assertFalse(load["orientation_matched_physical_coupon_complete"])
        ream = geom["corrected_printed_arm_root_sleeve_load_path"][
            "post_print_manufacturing_contract"
        ]
        self.assertFalse(ream["as_printed_FDM_bore_is_accepted_without_reaming"])
        self.assertEqual(ream["finished_bore_tolerance_mm"], [12.1, 12.13])
        self.assertEqual(ream["measured_shaft_OD_acceptance_mm"], [11.98, 12.0])
        self.assertFalse(ream["measured_fit_and_assembly_check_complete"])
        self.assertFalse(
            self.report["release_gates"][
                "printed_arm_ID12p10_post_ream_measured_fit_before_physical_balance"
            ]
        )

    def test_hardware_audit_snapshot_is_reconciled_not_silently_overridden(self) -> None:
        contract = self.report["source_contracts"][
            "hardware_release_audit_snapshot"
        ]
        self.assertEqual(
            contract["schema"], "hardware-manufacturing-release-audit/v1"
        )
        self.assertEqual(contract["status"], "FAIL_CLOSED")
        self.assertTrue(contract["audit_predates_this_candidate"])
        reconciliation = contract["reconciled_blocker_ids"]
        self.assertTrue(
            reconciliation["release.extended_shaft_artifact_wrong_length"].startswith(
                "RESOLVED"
            )
        )
        self.assertEqual(
            reconciliation["release.candidate_fasteners_missing_or_stale"],
            "OPEN",
        )

    def test_exact_leadshine_and_official_NBK_occurrences_replace_envelopes(self) -> None:
        contract = self.report["source_contracts"]["selected_M2_motor"]
        self.assertEqual(contract["model"], "Leadshine CS-M21708")
        self.assertEqual(contract["shaft_profile"], "D")
        self.assertAlmostEqual(contract["shaft_across_flat_mm"], 4.5)
        self.assertFalse(contract["stock_round_bore_split_clamp_authorized"])
        self.assertFalse(contract["old_17HS24_motor_gate_carried_forward"])
        nbk_contract = self.report["source_contracts"][
            "official_NBK_P30_D5_motor_stock_occurrence"
        ]
        self.assertEqual(
            nbk_contract["vendor_STEP_sha256"],
            rc.nbk_p30.SOURCE_STEP_SHA256,
        )
        self.assertEqual(
            nbk_contract["current_vendor_STEP_sha256"],
            rc.nbk_p30.SOURCE_STEP_SHA256,
        )
        self.assertTrue(
            nbk_contract["stock_occurrence_is_byte_identical_transform_only"]
        )
        self.assertFalse(nbk_contract["vendor_STEP_reexported_alone"])
        self.assertFalse(hasattr(rc.nbk_p30, "gen_step"))
        self.assertEqual(nbk_contract["stock_mass_g"], 28.0)
        self.assertEqual(
            nbk_contract["stock_axial_moment_of_inertia_kgm2"], 3.0e-6
        )
        self.assertFalse(
            nbk_contract["configured_BNW_boundary"][
                "retention_release_authorized"
            ]
        )
        flyer_contract = self.report["source_contracts"][
            "official_NBK_P30_D10_flyer_stock_occurrence"
        ]
        self.assertEqual(
            flyer_contract["vendor_STEP_sha256"],
            rc.nbk_p30_d10.SOURCE_STEP_SHA256,
        )
        self.assertEqual(
            flyer_contract["official_part_number"], "P30-3GT-BLP-6C-10"
        )
        self.assertEqual(flyer_contract["stock_D10_bore_mm"], 10.0)
        self.assertEqual(flyer_contract["stock_mass_g_consumed_in_balance"], 28.0)
        self.assertEqual(
            flyer_contract["stock_axial_moment_of_inertia_kgm2_consumed"],
            3.0e-6,
        )
        self.assertFalse(
            flyer_contract["physical_receiving_and_retention_release_authorized"]
        )
        drive_parts = rc.successor_drive_parts()
        official = rc.official_motor_pulley_review()
        stock = drive_parts["motor_pulley"]
        self.assertEqual(stock.label, rc.nbk_p30.STOCK_LABEL)
        self.assertEqual(len(stock.solids()), 1)
        self.assertAlmostEqual(stock.volume, official.stock_occurrence.volume, 5)
        self.assertAlmostEqual(stock.bounding_box().min.Z, -103.25, places=5)
        self.assertAlmostEqual(stock.bounding_box().max.Z, -84.75, places=5)
        flyer_stock = drive_parts["flyer_pulley"]
        self.assertEqual(flyer_stock.label, rc.nbk_p30_d10.STOCK_LABEL)
        self.assertEqual(len(flyer_stock.solids()), 1)
        self.assertAlmostEqual(flyer_stock.bounding_box().min.Z, -110.75, places=5)
        self.assertAlmostEqual(flyer_stock.bounding_box().max.Z, -92.25, places=5)
        assumptions = self.report["P30_NBK_interface_assumptions"]
        self.assertFalse(assumptions["motor_interface_authorized"])
        self.assertIn("official NBK", assumptions["motor_pulley_geometry"])
        self.assertIn("M3x12", assumptions["motor_pulley_geometry"])
        self.assertIn("BNW", assumptions["selection_interface_refinement"])
        self.assertFalse(assumptions["flyer_hub_torque_capacity_authorized"])
        self.assertLessEqual(
            self.report["geometry"]["intended_contacts"][
                "motor_pulley_to_exact_Leadshine_D_shaft_distance_mm"
            ],
            1.0e-5,
        )
        self.assertTrue(
            self.report["release_gates"][
                "coupled_exact_live_line_Leadshine_36V_margin_ge_2x"
            ]
        )
        torque = self.report["geometry"]["final_integrated_M2_torque"]
        self.assertTrue(torque["Leadshine_36V_gate_ge_2x"])
        self.assertTrue(torque["Leadshine_24V_gate_ge_2x"])
        self.assertFalse(torque["Leadshine_24V_release_authorized"])
        self.assertTrue(torque["P30_210_3GT_gate_ge_2x"])
        self.assertFalse(torque["old_m2_drive_successor_review_is_governing"])
        self.assertTrue(
            torque[
                "terminal_guide_2400_locus_max_perpendicular_moment_arm_consumed"
            ]
        )
        self.assertTrue(torque["coupled_final_motor_gate_ge_2x"])
        self.assertAlmostEqual(
            torque["maximum_perpendicular_live_line_lever_mm"],
            19.861544713,
            places=8,
        )
        self.assertGreater(
            torque["Leadshine_36V_available_to_required_multiple"], 3.11
        )
        self.assertGreater(
            torque["P30_210_3GT_available_to_required_multiple"], 7.85
        )
        self.assertTrue(torque["coupled_axis_loads"]["M1"]["gate_ge_2x"])
        self.assertTrue(torque["coupled_axis_loads"]["M0"]["gate_ge_2x"])
        self.assertFalse(
            torque[
                "driver_36V_current_microstep_limits_configured_and_verified"
            ]
        )
        self.assertFalse(torque["installed_hot_dyno_verified"])
        authority = torque["motor_pulley_mass_and_J_authority"]
        self.assertEqual(authority["official_stock_mass_g"], 28.0)
        self.assertEqual(authority["official_stock_axial_J_kgm2"], 3.0e-6)
        self.assertFalse(authority["separate_stock_M2_bolt_witness_J_added"])
        self.assertEqual(
            torque["added_output_referred_components_kgm2"][
                "official_NBK_P30_stock_complete_assembly"
            ],
            3.0e-6,
        )
        self.assertTrue(
            self.report["release_gates"][
                "coupled_exact_live_line_Leadshine_36V_margin_ge_2x"
            ]
        )
        self.assertEqual(
            len(self.report["geometry"]["intended_contacts"][
                "BNW_set_screw_to_exact_Leadshine_shaft_distances_mm"
            ]),
            2,
        )
        intended = self.report["geometry"]["intended_contacts"]
        self.assertLessEqual(
            max(intended["BNW_set_screw_to_exact_Leadshine_shaft_distances_mm"]),
            1.0e-5,
        )
        self.assertEqual(
            self.report["geometry"]["unintended_overlaps_mm3"][
                "BNW_set_screw_witnesses_vs_exact_Leadshine_shaft_max_mm3"
            ],
            0.0,
        )
        packaging = self.report["geometry"][
            "BNW_set_screw_socket_end_and_packaging"
        ]
        self.assertEqual(packaging["screw_inward_adjustments_mm"], [0.0, 0.5])
        self.assertEqual(packaging["M3x12_length_preserved_mm"], [12.0, 12.0])
        self.assertGreaterEqual(packaging["belt_clearance_min_mm"], 2.2)
        self.assertGreater(
            min(intended["BNW_hole_path_to_official_stock_overlap_mm3"]),
            0.0,
        )
        self.assertGreater(
            min(intended["BNW_set_screw_to_official_stock_overlap_mm3"]),
            0.0,
        )
        self.assertGreater(
            min(intended["BNW_hole_path_to_matching_screw_overlap_mm3"]),
            0.0,
        )
        self.assertTrue(
            self.report["release_gates"][
                "official_stock_NBK_P30_D5_and_D10_STEP_placement_and_mass_authority"
            ]
        )
        self.assertFalse(
            self.report["release_gates"][
                "exact_configured_NBK_P30_BNW_CAD_and_retention_rating"
            ]
        )

    def test_active_sector_route_and_full_raw_contract_is_consumed(self) -> None:
        contract = self.report["source_contracts"][
            "active_sector_terminal_route_and_rigid_sweep"
        ]
        self.assertEqual(
            contract["schema"],
            "carriage-active-sector-terminal-guide-audit/v1",
        )
        self.assertTrue(contract["assembly_geometry_integration_authorized"])
        self.assertFalse(contract["production_authorized"])
        self.assertEqual(contract["collision_geometry_revision"], rc.COLLISION_GEOMETRY_REVISION)
        self.assertEqual(contract["locus_count"], 2400)
        self.assertEqual(contract["locus_file_sha256"], rc._sha256(rc.ACTIVE_SECTOR_LOCI))
        self.assertEqual(contract["step_sha256"], rc._sha256(rc.ACTIVE_SECTOR_STEP))
        self.assertFalse(contract["continuous_park_index_load_unload_proven"])
        self.assertFalse(contract["raw_wraps_exactly_two_turns"])
        gates = self.report["release_gates"]
        self.assertTrue(gates["full_raw_cycle_collision_regenerated"])
        self.assertTrue(gates["all_2400_deposition_terminal_routes_exact_and_clear"])
        self.assertTrue(gates["both_raw_wrap_wire_paths_bypass_fixed_guide_yoke"])
        self.assertFalse(gates["both_raw_shaft_wraps_exactly_two_turns"])
        self.assertFalse(gates["park_index_load_unload_continuous_conductor_proven"])
        self.assertFalse(gates["continuous_conductor_from_spool_through_every_deposited_turn"])

    def test_public_build_links_contract_is_ready_for_raw_cycle_runner(self) -> None:
        links = rc.build_links()
        self.assertEqual(set(links), {"static", "carriage", "spindle", "flyer"})
        labels = {
            name: {str(getattr(shape, "label", "")) for shape in parts}
            for name, parts in links.items()
        }
        self.assertIn("m2_Leadshine_CS-M21708_exact_cableless", labels["static"])
        self.assertIn(rc.nbk_p30.STOCK_LABEL, labels["static"])
        self.assertEqual(
            sum("M3x12_set_screw_envelope_witness" in label
                for label in labels["static"]),
            2,
        )
        self.assertFalse(
            any("hole_path_witness" in label for label in labels["static"])
        )
        self.assertNotIn("m2_motor", labels["static"])
        self.assertNotIn("m2_motor_pulley", labels["static"])
        self.assertNotIn("gt2_belt", labels["static"])
        self.assertIn(
            "torus_free_retained_arm_with_open_PEEK_cradle_seat_one_solid",
            labels["flyer"],
        )
        self.assertIn(
            "released_M2_001_Rev_D_flyer_shaft_D10_ID6_ID9_L79",
            labels["flyer"],
        )
        self.assertIn(rc.nbk_p30_d10.STOCK_LABEL, labels["flyer"])
        self.assertNotIn("flyer_arm", labels["flyer"])
        self.assertFalse(
            any(
                "torus" in label.lower()
                and "torus_free" not in label.lower()
                for label in labels["flyer"]
            )
        )
        self.assertIn(
            "front_one_solid_PEEK_cap_with_short_open_leadins",
            labels["spindle"],
        )
        self.assertIn(
            "rear_one_solid_PEEK_cap_with_short_open_leadins",
            labels["spindle"],
        )
        self.assertIn(
            "front_M0_following_M1_static_PEEK_active_sector",
            labels["carriage"],
        )
        self.assertIn(
            "rear_M0_following_M1_static_PEEK_active_sector",
            labels["carriage"],
        )
        self.assertIn(
            "M0_carriage_owned_aluminum_active_sector_split_yoke",
            labels["carriage"],
        )
        self.assertIn(
            "spindle_tower_with_active_sector_M4_insert_pilots",
            labels["carriage"],
        )
        self.assertNotIn("spindle_tower", labels["carriage"])
        visuals = rc.wire_visuals()
        self.assertEqual(set(visuals), {"static", "flyer", "spindle"})

    def test_report_hash_is_self_consistent(self) -> None:
        rc.validate_report_integrity(self.report)


if __name__ == "__main__":
    unittest.main()
