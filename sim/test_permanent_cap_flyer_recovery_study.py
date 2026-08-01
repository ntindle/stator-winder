"""Focused fail-closed checks for the permanent-cap flyer recovery audit."""

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

import permanent_cap_flyer_recovery_study as study


class PermanentCapFlyerRecoveryStudyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = study.analyze()

    def test_raw_contract_remains_unmodified_upstream_cycle(self):
        raw = self.report["raw_contract"]
        self.assertEqual(raw["status"], "PASS")
        self.assertFalse(raw["candidate_changes_command_stream"])
        self.assertTrue(all(raw["checks"].values()))
        self.assertTrue(
            raw["checks"][
                "cycle_report_fail_is_only_exact_two_turn_blocker"
            ]
        )
        self.assertEqual(
            raw["capture_sha256"],
            "350210c36550ae2c22e5352675aee6bd2eebb18325f3b5f45c7cb35d6a314958",
        )

    def test_exact_common_spoke_witness_has_positive_volume(self):
        witness = self.report["exact_common_spoke_witness"]
        self.assertEqual(witness["status"], "FAIL")
        self.assertTrue(math.isclose(
            witness["intersection_volume_mm3"],
            4.008684165320739,
            rel_tol=0.0,
            abs_tol=1e-8,
        ))
        self.assertEqual(witness["intersection_solid_count"], 2)
        self.assertLess(
            witness["minimum_clearance_upper_bound_mm"],
            witness["required_dynamic_clearance_mm"],
        )

    def test_bounded_radius_and_guide_z_sweep_has_no_pass(self):
        sweep = self.report["bounded_sweep"]
        self.assertEqual(sweep["candidate_count"], 35)
        self.assertEqual(sweep["pass_count"], 0)
        self.assertFalse(sweep["larger_flyer_clears_cap"])
        self.assertTrue(all(
            not row["gates"]["cap_to_common_spoke_2mm"]
            for row in sweep["candidates"]
        ))

    def test_zero_distance_routes_are_crossings_not_adjacent_contact(self):
        semantics = self.report["stored_route_contact_semantics"]
        self.assertEqual(semantics["status"], "FAIL")
        self.assertFalse(semantics["controlling_architecture_rejection"])
        self.assertTrue(all(
            row["classification"] == "ACTUAL_CENTERLINE_CROSSING"
            for row in semantics["cases"]
        ))
        self.assertTrue(all(
            row["raw_centerline_distance_mm"] < 1e-12
            for row in semantics["cases"]
        ))

    def test_launch_od65_stack20_access_and_chuck_boundaries(self):
        launch = self.report["launch_envelope"]
        self.assertTrue(launch["gates"]["OD65_stack20_nominal_50_turn_job"])
        self.assertTrue(
            launch["gates"]["maximum_0p5_wire_has_open_slot_access"]
        )
        self.assertFalse(launch["gates"]["maximum_0p5_wire_50_turn_fill"])
        self.assertGreaterEqual(
            launch["cap_to_ER11_chuck_radial_clearance_mm"], 2.0
        )
        self.assertLess(launch["required_flyer_tip_radius_mm"], 45.0)
        self.assertEqual(
            launch["maximum_wire_job"]["maximum_turns_at_hard_fill"], 26
        )

    def test_wall_options_do_not_create_a_released_part(self):
        material = self.report["material_and_finish"]
        self.assertEqual(material["status"], "UNQUALIFIED")
        self.assertFalse(material["release_gate"])
        self.assertEqual(
            [row["wall_mm"] for row in material["wall_options"]],
            [0.5, 1.0],
        )
        self.assertTrue(all(
            not row["ready_for_project_printer"]
            and not row["ready_to_order"]
            for row in material["wall_options"]
        ))

    def test_r45_to_r60_remains_inside_reasonable_motor_envelope(self):
        loads = json.loads(study.LOADS_REPORT.read_text())
        part_names = {row["part"] for row in loads["flyer"]["parts"]}
        self.assertIn("retained_arm", part_names)
        self.assertIn("flyer_PEEK_guide", part_names)
        self.assertNotIn("flyer_arm", part_names)
        self.assertNotIn("tip_toroid_guide", part_names)
        motor = self.report["motor_envelope"]
        self.assertEqual(motor["status"], "PASS")
        self.assertTrue(all(
            row["selected_M2_margin"] >= 2.0
            and row["selected_pulley_margin"] >= 2.0
            and row["status"] == "PASS"
            for row in motor["candidates"]
        ))
        self.assertTrue(self.report["gates"]["reasonable_motor_envelope"])

    def test_report_is_fail_closed_and_forbids_integration(self):
        self.assertEqual(self.report["status"], "DESIGN_NO_GO")
        self.assertFalse(self.report["release_authorized"])
        self.assertFalse(self.report["assembly_integration_authorized"])
        stored = json.loads(study.JSON_OUT.read_text())
        self.assertEqual(stored["status"], "DESIGN_NO_GO")
        self.assertEqual(
            stored["exact_common_spoke_witness"]["intersection_solid_count"],
            2,
        )


if __name__ == "__main__":
    unittest.main()
