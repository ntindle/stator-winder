"""Release-contract tests for the physical active-sector terminal guide."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "out" / "reports" / (
    "carriage_active_sector_terminal_guide_audit.json"
)
LOCI = ROOT / "out" / "reports" / (
    "carriage_active_sector_terminal_guide_loci.json"
)
MANIFEST = ROOT / "out" / "review" / (
    "carriage_active_sector_terminal_guide.manifest.json"
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical(value: object) -> str:
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")).hexdigest()


class CarriageActiveSectorTerminalGuideAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = json.loads(REPORT.read_text(encoding="utf-8"))
        cls.loci = json.loads(LOCI.read_text(encoding="utf-8"))
        cls.manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))

    def test_canonical_2400_locus_payload_and_file_hashes(self) -> None:
        self.assertEqual(len(self.loci["loci"]), 2400)
        body = deepcopy(self.loci)
        expected = body.pop("locus_payload_sha256")
        self.assertEqual(_canonical(body), expected)
        api = self.report["player_route_api"]
        self.assertEqual(api["canonical_payload_sha256"], expected)
        self.assertEqual(api["compact_file_sha256"], _sha256(LOCI))
        self.assertEqual(api["compact_size_bytes"], LOCI.stat().st_size)
        self.assertEqual(
            self.loci["flyer_reference_validation"]["status"], "PASS"
        )

    def test_no_obsolete_torus_route_metadata(self) -> None:
        self.assertNotIn("torus", LOCI.read_text(encoding="utf-8").lower())
        self.assertFalse(any(
            segment["name"] == "flyer_geometric_bore"
            for locus in self.loci["loci"]
            for segment in locus["segments"]
        ))
        reference = self.loci["flyer_reference"]
        self.assertEqual(reference["full_geometric_bore_point_count"], 181)
        self.assertEqual(reference["conductor_prefix_point_count"], 175)

    def test_all_wire_extremes_and_2400_terminal_routes_pass(self) -> None:
        route = self.report["terminal_deposition_route"]
        self.assertEqual(route["pass_count"], 2400)
        self.assertEqual(route["failure_count"], 0)
        self.assertGreaterEqual(route["minimum_bell_wire_center_radius_mm"], 3.25)
        self.assertGreaterEqual(
            route[
                "minimum_straight_bore_handoff_radius_over_0p20_to_0p50mm_wire_mm"
            ],
            3.0,
        )
        self.assertLessEqual(route["maximum_bell_turn_deg"], 200.0)

    def test_all_2400_short_leadins_bind_actual_side_specific_cap_BREP(self) -> None:
        binding = self.report["physical_cap_lane_binding"]
        self.assertEqual(binding["status"], "PASS")
        self.assertTrue(binding[
            "all_2400_loci_bind_actual_cap_lane_BREP"
        ])
        self.assertEqual(binding["case_count"], 4)
        self.assertEqual(binding["locus_count"], 2400)
        self.assertLessEqual(
            binding["maximum_locus_BREP_seam_gap_mm"], 1.0e-8
        )
        endpoint_names = {
            row["side"]: row["cap_endpoint_name"]
            for row in binding["cases"] if row["axial_sign"] == 1
        }
        self.assertEqual(
            endpoint_names,
            {
                "left": "_lane_points()['riser_top']",
                "right": "_lane_points()['waypoint']",
            },
        )
        for row in binding["cases"]:
            self.assertEqual(row["status"], "PASS")
            self.assertEqual(row["incident_cap_edge_count"], 2)
            self.assertGreaterEqual(
                row["minimum_named_centerline_radius_mm"], 3.5
            )
            self.assertEqual(
                len(row["actual_BREP_circle_radii_mm"]),
                1 if row["side"] == "left" else 3,
            )
            self.assertGreaterEqual(
                min(row["actual_BREP_circle_radii_mm"]), 3.5 - 1.0e-8
            )
            self.assertLessEqual(
                row["maximum_internal_edge_position_error_mm"], 1.0e-8
            )
            self.assertLessEqual(
                row["maximum_internal_edge_tangent_error_deg"], 1.0e-7
            )
        self.assertTrue(
            self.report["release_gates"]
            ["all_2400_short_leadin_endpoints_join_actual_cap_lane_BREP"]
        )

    def test_exact_rigid_and_tolerance_clearances_pass(self) -> None:
        self.assertEqual(
            self.report["deposition_rigid_collision"]["status"], "PASS"
        )
        self.assertEqual(
            self.report["arbitrary_M1_and_progressive_copper_clearance"]
            ["status"],
            "PASS",
        )
        tolerance = self.report["tolerance_budget"]
        self.assertEqual(tolerance["status"], "PASS")
        for row in tolerance["controls"].values():
            self.assertGreaterEqual(row["worst_case_clearance_mm"], 2.0)

    def test_adjacent_global_groove_cuts_isolate_max_wire(self) -> None:
        isolation = self.report["adjacent_short_leadin_isolation"]
        self.assertEqual(isolation["status"], "PASS")
        self.assertEqual(isolation["representative_case_count"], 2)
        self.assertEqual(isolation["physical_adjacent_pair_count"], 48)
        for row in isolation["cases"]:
            self.assertEqual(row["status"], "PASS")
            self.assertEqual(row["negative_tool_pair_count_checked"], 70)
            self.assertGreater(
                row["minimum_distinct_negative_tool_gap_mm"], 0.5
            )
            self.assertEqual(
                row["globally_cut_representative_pair_solid_count"], 1
            )
            self.assertGreaterEqual(
                row["intended_wire_reserve_after_profile_tolerance_mm"],
                0.05,
            )
            self.assertGreaterEqual(
                row["separator_web_after_two_sided_profile_tolerance_mm"],
                0.05,
            )
            self.assertLessEqual(
                row["right_max_wire_to_globally_cut_pair_intrusion_mm3"],
                1.0e-8,
            )
            self.assertLessEqual(
                row["left_max_wire_to_globally_cut_pair_intrusion_mm3"],
                1.0e-8,
            )
        self.assertTrue(
            self.report["release_gates"]
            ["all_48_adjacent_short_leadin_pairs_isolate_R0p25_wire"]
        )

    def test_all_finished_right_seams_accept_tolerance_gauge(self) -> None:
        access = self.report["right_seam_final_BREP_accessibility"]
        self.assertEqual(access["status"], "PASS")
        self.assertEqual(access["seam_count"], 48)
        self.assertEqual(access["maximum_gauge_positive_overlap_mm3"], 0.0)
        self.assertEqual(access["maximum_open_ray_inside_sample_count"], 0)
        self.assertEqual(len(access["cap_rows"]), 2)
        for cap_row in access["cap_rows"]:
            self.assertEqual(cap_row["status"], "PASS")
            self.assertEqual(
                cap_row["solid_count_after_all_24_right_mouth_cuts"], 1,
            )
            self.assertGreater(
                cap_row["removed_volume_per_right_seam_mm3"], 0.15,
            )
            self.assertLess(
                cap_row["removed_volume_per_right_seam_mm3"], 0.16,
            )
        for seam in access["seams"]:
            self.assertEqual(seam["status"], "PASS")
            self.assertEqual(
                seam["gauge_to_final_cap_positive_overlap_mm3"], 0.0,
            )
            self.assertEqual(seam["open_ray_inside_sample_count"], 0)
        self.assertTrue(
            self.report["release_gates"]
            ["all_48_right_seams_accept_R0p36_radial_insertion_gauge"]
        )

    def test_front_plane_yoke_full_m2_and_frame_packaging_pass(self) -> None:
        full_m2 = self.report["front_plane_yoke_full_M2_clearance"]
        self.assertEqual(full_m2["status"], "PASS")
        self.assertEqual(full_m2["M2_step_deg"], 0.25)
        self.assertEqual(full_m2["M2_sample_count"], 1440)
        self.assertEqual(full_m2["collision_count"], 0)
        self.assertGreaterEqual(full_m2["minimum_clearance_mm"], 2.0)
        packaging = self.report["outboard_yoke_packaging"]
        self.assertEqual(packaging["status"], "PASS")
        self.assertEqual(
            packaging["same_link_carriage"]
            ["unintended_positive_overlap_count"],
            0,
        )
        self.assertEqual(
            packaging["static_frame_full_M0"]["collision_count"], 0
        )
        self.assertGreaterEqual(
            packaging["static_frame_full_M0"]["minimum_clearance_mm"], 2.0
        )

    def test_yoke_structure_and_attachment_chain_pass(self) -> None:
        structure = self.report["guide_structure_DFM_and_attachments"]
        load = structure["mass_and_yoke_load"]
        self.assertTrue(load["stress_gate"])
        self.assertLessEqual(
            load["maximum_bending_stress_MPa"],
            load["6061_T6_screen_allowable_MPa"],
        )
        self.assertTrue(
            structure["attachments"]["no_floating_screw_chain_gate"]
        )

    def test_m0_m1_m2_numeric_margins_and_open_control_gates(self) -> None:
        loads = self.report["coupled_live_line_loads"]
        self.assertGreaterEqual(
            loads["M2"]["Leadshine_36V_available_to_required_multiple"], 2.0
        )
        self.assertGreaterEqual(
            loads["M2"]["Leadshine_24V_available_to_required_multiple"], 2.0
        )
        self.assertFalse(loads["M2"]["Leadshine_24V_release_authorized"])
        self.assertFalse(
            loads["M2"]
            ["driver_36V_current_microstep_limits_configured_and_verified"]
        )
        self.assertFalse(loads["M2"]["installed_hot_dyno_verified"])
        self.assertGreaterEqual(loads["M1"]["available_to_required_multiple"], 2.0)
        self.assertFalse(loads["M1"]["drive_fault_safe_behavior_verified"])
        self.assertGreaterEqual(loads["M0"]["available_to_required_multiple"], 2.0)

    def test_raw_rigid_pass_but_continuous_conductor_stays_fail_closed(self) -> None:
        self.assertEqual(self.report["full_raw_rigid_motion"]["status"], "PASS")
        wraps = self.report["shaft_wrap_guide_bypass"]
        self.assertTrue(
            wraps["gates"]["wire_bypasses_fixed_guide_yoke_for_both_wraps"]
        )
        self.assertFalse(wraps["gates"]["each_raw_wrap_is_two_full_turns"])
        self.assertFalse(
            wraps["gates"]
            ["park_index_load_unload_continuous_conductor_proven"]
        )

    def test_step_and_manifest_bind_exact_artifacts(self) -> None:
        step = self.report["artifacts"]["step"]
        path = ROOT / step["path"]
        self.assertTrue(step["exists"])
        self.assertEqual(step["file_sha256"], _sha256(path))
        self.assertEqual(step["size_bytes"], path.stat().st_size)
        self.assertEqual(self.manifest["artifacts"]["step"], step)
        self.assertEqual(
            self.manifest["locus_api"]["compact_file_sha256"],
            _sha256(LOCI),
        )

    def test_release_remains_physically_fail_closed(self) -> None:
        gates = self.report["release_gates"]
        self.assertFalse(gates["M2_36V_driver_configuration_verified"])
        self.assertFalse(gates["M2_installed_hot_dyno_verified"])
        self.assertFalse(gates["M1_closed_loop_drive_fault_safe_behavior_verified"])
        self.assertFalse(gates["both_raw_shaft_wraps_exactly_two_turns"])
        self.assertFalse(gates["PEEK_forming_gauge_polish_abrasion_coupon"])
        self.assertFalse(gates["production_authorized"])
        self.assertFalse(self.report["production_authorized"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
