"""Focused checks for the offset-spoke permanent-cap recovery study."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
CAD = HERE.parent / "cad"
for path in (HERE, CAD):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import permanent_cap_offset_spoke_flyer_study as study


class PermanentCapOffsetSpokeFlyerStudyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = study.analyze()

    def test_canonical_raw_pose_population_and_deepest_witness(self):
        raw = self.report["raw_pose_evidence"]
        self.assertEqual(raw["status"], "PASS")
        self.assertTrue(all(raw["checks"].values()))
        self.assertEqual(raw["raw_sample_count"], 225775)
        self.assertEqual(raw["unique_quantized_pose_count"], 15765)
        self.assertEqual(raw["integer_flyer_angle_count"], 360)
        self.assertEqual(raw["controller_mode"], "upstream")
        self.assertTrue(math.isclose(
            raw["minimum_stator_axis_z_mm"], 16.16355386908819,
            rel_tol=0.0, abs_tol=1.0e-10,
        ))
        self.assertEqual(
            raw["capture_sha256"],
            "350210c36550ae2c22e5352675aee6bd2eebb18325f3b5f45c7cb35d6a314958",
        )

    def test_current_block_corridor_and_shift_bounds_are_exact(self):
        by_wall = {row["wall_mm"]: row for row in self.report["shift_bounds"]}
        self.assertTrue(math.isclose(
            by_wall[0.5]["current_block_max_spoke_thickness_mm"],
            0.010258176691557708, rel_tol=0.0, abs_tol=1.0e-12,
        ))
        self.assertTrue(math.isclose(
            by_wall[1.0]["minimum_shift_for_retained_8mm_spoke_mm"],
            8.239741823308442, rel_tol=0.0, abs_tol=1.0e-12,
        ))
        self.assertGreater(
            by_wall[1.0]["minimum_shift_for_2p4mm_spoke_mm"], 2.6
        )

    def test_minimum_wall_is_not_a_structural_substitute(self):
        thin = self.report["minimum_wall_structural_screen"]
        self.assertEqual(thin["thickness_mm"], 2.4)
        self.assertFalse(thin["passes"])
        self.assertGreater(thin["tip_deflection_mm"], 9.0)
        self.assertLess(thin["fatigue_screening_margin"], 1.0)

    def test_selected_successor_retains_8mm_and_all_candidate_gates(self):
        selected = self.report["bounded_sweep"]["selected"]
        self.assertIsNotNone(selected)
        self.assertEqual(selected["status"], "PASS")
        self.assertEqual(selected["wall_mm"], 1.0)
        self.assertEqual(selected["module_shift_mm"], 10.0)
        self.assertEqual(selected["spoke_front_z_mm"], -30.12)
        self.assertEqual(selected["spoke_thickness_mm"], 8.0)
        self.assertEqual(selected["transition_radius_mm"], 58.0)
        self.assertEqual(selected["tip_radius_mm"], 64.0)
        self.assertTrue(all(selected["gates"].values()))
        self.assertGreaterEqual(
            selected["clearances_mm"]["raw_cap_to_spoke"], 2.2
        )
        self.assertGreaterEqual(
            selected["clearances_mm"]["shifted_block_to_spoke"], 2.2
        )
        self.assertGreaterEqual(
            selected["clearances_mm"]["extended_shaft_to_entry_eyelet"], 2.2
        )

    def test_retained_spoke_and_m2_drive_keep_two_x_margins(self):
        loads = study._load(study.LOADS_REPORT)
        part_names = {row["part"] for row in loads["flyer"]["parts"]}
        self.assertIn("retained_arm", part_names)
        self.assertIn("flyer_PEEK_guide", part_names)
        self.assertNotIn("flyer_arm", part_names)
        self.assertNotIn("tip_toroid_guide", part_names)
        selected = self.report["bounded_sweep"]["selected"]
        structure = selected["structure"]
        motor = selected["motor_and_balance"]
        self.assertTrue(structure["passes"])
        self.assertLess(structure["tip_deflection_mm"], 0.5)
        self.assertGreaterEqual(structure["fatigue_screening_margin"], 2.0)
        self.assertGreaterEqual(motor["selected_motor_margin"], 2.0)
        self.assertGreaterEqual(motor["selected_pulley_margin"], 2.0)
        self.assertFalse(motor["current_three_washer_stack_sufficient"])
        self.assertTrue(motor["successor_weight_is_geometrically_sizeable"])

    def test_selected_p30_belt_authority_replaces_legacy_p40_schema(self):
        basis = self.report["belt_and_static_basis"]
        self.assertEqual(
            basis["selected_belt_audit_schema"],
            "selected-m2-belt-audit/v2",
        )
        self.assertTrue(basis["selected_belt_audit_pass"])
        self.assertEqual(
            basis["selected_belt_motor_label"],
            "m2_Leadshine_CS-M21708_exact_cableless",
        )
        self.assertGreaterEqual(
            basis["selected_belt_motor_clearance_mm"], 2.2
        )
        self.assertGreaterEqual(
            basis["selected_belt_minimum_static_clearance_mm"], 2.2
        )

    def test_aggregate_is_core_and_cross_tooth_nonpenetrating(self):
        aggregate = self.report["aggregate_nonpenetration"]
        self.assertEqual(aggregate["status"], "PARTIAL_BOUNDARY_PASS")
        self.assertFalse(all(aggregate["gates"].values()))
        self.assertFalse(
            aggregate["gates"]["continuous_slot_to_crown_connector_nonpenetrating"]
        )
        self.assertEqual(aggregate["support_endpoint_count"], 2400)
        self.assertGreaterEqual(
            aggregate["minimum_cross_tooth_center_distance_mm"],
            aggregate["required_wire_center_distance_mm"],
        )
        self.assertGreaterEqual(
            aggregate["outboard_adjacent_crown_gap_mm"],
            aggregate["required_wire_center_distance_mm"],
        )
        self.assertGreaterEqual(
            aggregate["outboard_crown_to_core_gap_mm"], 2.0
        )
        self.assertFalse(aggregate["stored_deterministic_route_family_reused"])

    def test_continuous_aggregate_authority_is_hash_and_capture_bound(self):
        authority = self.report["continuous_aggregate_authority"]
        self.assertEqual(authority["status"], "PASS")
        self.assertTrue(all(authority["checks"].values()))
        self.assertEqual(authority["lane_id"], "cap-r3-sector-lane-v1")
        self.assertEqual(authority["connector_count"], 96)
        self.assertEqual(
            authority["capture_sha256"],
            self.report["raw_pose_evidence"]["capture_sha256"],
        )
        self.assertEqual(len(authority["support_contract_sha256"]), 64)
        self.assertTrue(
            self.report["release_gates"]
            ["continuous_slot_to_crown_aggregate_nonpenetrating"]
        )

    def test_launch_envelope_and_full_flyer_sweep_remain_in_scope(self):
        gates = self.report["architecture_gates"]
        self.assertTrue(all(gates.values()))
        launch = self.report["launch_envelope"]
        self.assertTrue(launch["gates"]["OD65_stack20_nominal_50_turn_job"])
        self.assertTrue(
            launch["gates"]["maximum_0p5_wire_has_open_slot_access"]
        )
        self.assertFalse(launch["gates"]["maximum_0p5_wire_50_turn_fill"])

    def test_result_is_advisory_pass_but_release_fails_closed(self):
        self.assertEqual(
            self.report["status"], "REVIEW_CANDIDATE_NOT_PRODUCTION"
        )
        self.assertEqual(
            self.report["decision"],
            "OFFSET_SPOKE_AND_CONTINUOUS_AGGREGATE_REVIEW_CANDIDATE__EXACT_LOADS_UNPROVEN",
        )
        self.assertTrue(self.report["architecture_feasible"])
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["assembly_integration_authorized"])
        self.assertTrue(self.report["review_CAD_authorized"])
        self.assertFalse(all(self.report["release_gates"].values()))

    def test_report_has_authority_compatible_path_hashes(self):
        hashes = self.report["source_hashes"]
        self.assertIn("out/capture/upstream_current_raw.jsonl", hashes)
        self.assertIn("sim/permanent_cap_offset_spoke_flyer_study.py", hashes)
        self.assertIn(
            "out/reports/permanent_cap_aggregate_authorization.json", hashes
        )
        self.assertIn("sim/permanent_cap_aggregate_authorization.py", hashes)
        self.assertTrue(all("/" in name or name == "GOAL.md"
                            for name in hashes))
        self.assertEqual(len(self.report["report_sha256"]), 64)

    def test_stored_report_is_not_stale_after_generation(self):
        stored = json.loads(study.JSON_OUT.read_text(encoding="utf-8"))
        self.assertEqual(stored["schema"], study.SCHEMA)
        self.assertEqual(
            stored["status"], "REVIEW_CANDIDATE_NOT_PRODUCTION"
        )
        self.assertFalse(stored["production_authorized"])


if __name__ == "__main__":
    unittest.main()
