"""Regression checks for the bounded retained-former no-go report."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import unittest


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPORT = ROOT / "out" / "reports" / "r3_tooth_end_former.json"


class R3ToothEndFormerStudyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = json.loads(REPORT.read_text(encoding="utf-8"))

    def test_report_is_isolated_design_no_go(self):
        self.assertEqual(self.report["schema"],
                         "r3-tooth-end-former-study/v1")
        self.assertEqual(self.report["status"], "DESIGN_NO_GO")
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["assembly_integration_authorized"])
        self.assertFalse(self.report["scope"]["production_files_modified"])
        self.assertFalse(self.report["scope"]["raw_capture_or_protocol_modified"])
        self.assertFalse(self.report["scope"]["rejected_86p3mm_global_cap_reused"])

    def test_local_r3_slot_and_fill_gates_pass(self):
        gates = self.report["gates"]
        self.assertTrue(gates["literal_R3_one_tooth_witness"])
        self.assertTrue(gates["slot_opening_preserved"])
        self.assertTrue(gates["50_turn_fill_envelope"])
        motor = self.report["slot_fill_and_motor_envelope"]
        self.assertEqual(motor["slot_opening"]["status"], "PASS")
        self.assertEqual(motor["fill_and_50_turn_envelope"]["turn_count"], 50)
        self.assertEqual(motor["retained_motor_envelope"]["radial_status"],
                         "PASS")

    def test_full_stator_neighbor_and_solid_gates_fail(self):
        route = self.report["wire_routes_and_neighbors"]
        selected = route["selected_lane"]
        self.assertEqual(selected["odd_tooth_lane_mm"], 12.0)
        self.assertEqual(selected["status"], "FAIL")
        self.assertLess(selected["minimum_centerline_distance_mm"], 0.22352)
        self.assertEqual(route["all_24_neighbor_topology_status"], "FAIL")
        self.assertEqual(self.report["physical_former_overlap"]["status"],
                         "FAIL")
        self.assertFalse(
            self.report["gates"]["all_24_neighbor_wire_clearance"])
        self.assertFalse(
            self.report["gates"]["adjacent_former_solids_nonintersecting"])

    def test_rotor_axial_and_raw_authority_remain_fail_closed(self):
        motor = self.report["slot_fill_and_motor_envelope"][
            "retained_motor_envelope"]
        self.assertEqual(motor["status"], "FAIL_UNPROVEN_AXIAL_CAVITY")
        self.assertEqual(motor["rotor_end_bell_axial_cavity_status"],
                         "UNPROVEN")
        self.assertEqual(self.report["raw_rigid_clearance"]["status"],
                         "NOT_RUN")
        self.assertFalse(
            self.report["gates"]["retained_rotor_radial_and_axial_envelope"])
        self.assertFalse(
            self.report["gates"]["every_raw_pose_rigid_clearance"])

    def test_report_and_source_hashes_are_current(self):
        payload = dict(self.report)
        claimed = payload.pop("report_sha256")
        encoded = json.dumps(
            payload, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
        self.assertEqual(hashlib.sha256(encoded).hexdigest(), claimed)
        for relative, expected in self.report["source_hashes"].items():
            path = ROOT / relative
            self.assertTrue(path.is_file(), relative)
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),
                             expected, relative)


if __name__ == "__main__":
    unittest.main()
