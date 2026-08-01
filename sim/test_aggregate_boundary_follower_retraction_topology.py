"""Focused tests for the isolated follower retraction topology."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_retraction_topology as topology


class AggregateBoundaryFollowerRetractionTopologyTests(unittest.TestCase):

    def test_attached_bellcrank_equations_close_force_and_spring_limits(self):
        radial = topology.radial_topology()
        rows = radial["LEM050AB01"]["force_rows"]

        self.assertAlmostEqual(rows[0]["spring_length_mm"], 9.5, places=8)
        self.assertAlmostEqual(
            topology.bellcrank_long_arm_length_mm(17.0), 20.0, places=9
        )
        self.assertAlmostEqual(
            radial["bellcrank"]["nominal_motion_ratio"], 0.29, places=9
        )
        self.assertGreaterEqual(
            radial["LEM050AB01"]["motion_ratio_range"][0], 0.279
        )
        self.assertLessEqual(
            radial["LEM050AB01"]["motion_ratio_range"][1], 0.291
        )
        self.assertTrue(all(radial["analytical_gates"].values()))
        self.assertLess(
            radial["LEM050AB01"]["maximum_combined_inward_contact_force_N"],
            2.0,
        )
        self.assertFalse(
            radial["independent_radial_return"]["exact_spring_or_flexure_selected"]
        )
        self.assertEqual(
            radial["independent_radial_return"][
                "minimum_return_to_breakaway_ratio"
            ],
            2.0,
        )

    def test_opposed_tangential_springs_stay_captured_and_center(self):
        tangential = topology.tangential_topology()
        springs = tangential["opposed_centering_springs"]
        rows = springs["force_rows"]

        self.assertEqual(springs["center_preload_each_N"], 0.15)
        self.assertEqual(springs["net_centering_stiffness_N_per_mm"], 0.3)
        self.assertEqual(springs["restoring_force_at_usable_limit_N"], 0.15)
        self.assertTrue(all(tangential["analytical_gates"].values()))
        self.assertGreater(rows[0]["positive_side_spring_force_N"], 0.0)
        self.assertGreater(rows[-1]["negative_side_spring_force_N"], 0.0)
        self.assertFalse(tangential["bearing"]["exact_shaft_and_bushing_SKUs_selected"])
        self.assertFalse(springs["exact_spring_SKU_selected"])

    def test_positive_m0_cam_completes_and_dwells_through_home(self):
        m0 = topology.m0_retraction_topology()
        radial = m0["radial_positive_retraction"]

        self.assertAlmostEqual(
            radial["retraction_complete_axis_z_mm"], 28.633333333, places=8
        )
        self.assertGreater(radial["positive_dwell_before_API_boundary_mm"], 0.36)
        self.assertEqual(radial["closed_dwell_machine_z_range_mm"][-1], 77.0)
        self.assertAlmostEqual(
            radial["static_rail_surface_angle_deg"], 30.963756532, places=8
        )
        self.assertEqual(topology.cam_radial_center_mm(95.0), 14.0)
        self.assertTrue(all(m0["analytical_gates"].values()))
        self.assertFalse(m0["gimbal_positive_neutral_dock"]["topology_selected"])

    def test_dual_nc_truth_table_and_system_modes_fail_closed(self):
        interlock = topology.interlock_topology()
        pair_rows = interlock["dual_NC_pair_truth_table"]
        proved = [row for row in pair_rows if row["safe_position_proved"]]
        disagreements = [
            row for row in pair_rows if row["disagreement_fault_latched"]
        ]

        self.assertEqual(len(proved), 1)
        self.assertTrue(proved[0]["channel_A_closed"])
        self.assertTrue(proved[0]["channel_B_closed"])
        self.assertEqual(len(disagreements), 2)

        rows = {row["name"]: row for row in interlock["system_truth_table"]}
        self.assertTrue(rows["home_all_actual_positions_proved"]["M1_enable"])
        self.assertTrue(rows["home_all_actual_positions_proved"]["M2_enable"])
        self.assertFalse(rows["winding_selector_seated"]["M1_enable"])
        self.assertTrue(rows["winding_selector_seated"]["M2_enable"])
        for name, row in rows.items():
            if name not in {
                "home_all_actual_positions_proved",
                "winding_selector_seated",
            }:
                self.assertFalse(row["M1_enable"], name)
                self.assertFalse(row["M2_enable"], name)

    def test_report_keeps_all_physical_authority_false(self):
        report = topology.build_report()
        topology.validate_report_integrity(report)
        self.assertEqual(report["report_sha256"], topology._canonical_hash(report))

        self.assertEqual(report["status"], "DESIGN_ANALYSIS_ONLY_FAIL_CLOSED")
        self.assertTrue(all(report["analysis_gates"].values()))
        self.assertTrue(
            all(value is False for value in report["physical_authority_gates"].values())
        )
        for key in (
            "physical_authority",
            "CAD_integration_authorized",
            "assembly_integration_authorized",
            "player_integration_authorized",
            "BOM_change_authorized",
            "procurement_authorized",
            "release_authorized",
        ):
            self.assertFalse(report[key], key)

        markdown = topology._markdown(report)
        self.assertIn("Closed dwell continues through machine Z=77.000 mm", markdown)
        self.assertIn("Physical authority: **false**", markdown)
        self.assertIn("Do not integrate, order, wind, or release", markdown)


if __name__ == "__main__":
    unittest.main()
