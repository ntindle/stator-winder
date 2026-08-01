"""Regression checks for the bounded R3 dogleg retained-basket no-go."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
import unittest

import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import r3_dogleg_end_basket_study as study  # noqa: E402


REPORT = ROOT / "out" / "reports" / "r3_dogleg_end_basket.json"


class R3DoglegEndBasketStudyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = json.loads(REPORT.read_text(encoding="utf-8"))

    def test_four_arc_profile_is_tangent_and_exact_r3(self):
        profile = study.dogleg_profile()
        self.assertTrue(np.allclose(profile[0], (0.0, 0.0), atol=1e-10))
        self.assertTrue(np.allclose(
            profile[-1], (0.0, study.BEST_LANE_MM), atol=1e-9))
        parameters = study.dogleg_parameters(
            study.BEST_LANE_MM, study.BEST_OFFSET_MM)
        self.assertEqual(parameters["minimum_curve_radius_mm"], 3.0)
        self.assertGreaterEqual(parameters["straight_plateau_mm"], 0.0)

    def test_turn24_witness_is_explicit_and_fails(self):
        witness = self.report["exact_predecessor_turn24_witness"]
        self.assertEqual(witness["obstacle_turn_index"], 24)
        self.assertEqual(witness["active_turn_index"], 30)
        self.assertEqual(witness["status"], "FAIL")
        self.assertLess(
            witness["minimum_polyline_centerline_distance_mm"],
            study.WIRE_DIAMETER_MM)

    def test_best_member_covers_full_neighbor_symmetry_and_fails(self):
        best = self.report["best_full_neighbor_audit"]
        self.assertEqual(best["turns_per_tooth"], 50)
        self.assertEqual(best["neighbor_tooth_indices"], [-1, 1])
        self.assertEqual(best["symmetry_expansion_tooth_count"], 24)
        self.assertEqual(best["neighbor_topology_status"], "FAIL")
        self.assertLess(
            best["true_curve_distance_failure_upper_bound_mm"],
            study.WIRE_DIAMETER_MM)

    def test_r3_od_slot_pass_but_cavity_and_release_fail_closed(self):
        self.assertEqual(self.report["status"], "DESIGN_NO_GO")
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["assembly_integration_authorized"])
        self.assertFalse(self.report["cad_generated"])
        best = self.report["best_full_neighbor_audit"]
        self.assertEqual(best["radial_envelope_status"], "PASS")
        self.assertLessEqual(best["maximum_wire_outer_radius_mm"], 23.0)
        contract = self.report["slot_and_motor_contract"]
        self.assertEqual(contract["slot_throat"]["status"], "PASS")
        self.assertEqual(
            contract["rotor_end_bell_axial_cavity"]["status"],
            "UNPROVEN_MISSING_INPUT")

    def test_axial_cavity_parameter_is_live(self):
        required = self.report["slot_and_motor_contract"][
            "rotor_end_bell_axial_cavity"][
                "required_clear_cavity_per_face_beyond_stack_mm"]
        fail = study.axial_and_slot_contract(required - 0.01)
        passed = study.axial_and_slot_contract(required + 0.01)
        self.assertEqual(
            fail["rotor_end_bell_axial_cavity"]["status"], "FAIL")
        self.assertEqual(
            passed["rotor_end_bell_axial_cavity"]["status"], "PASS")

    def test_report_hash_and_sources_are_current(self):
        payload = dict(self.report)
        claimed = payload.pop("report_sha256")
        encoded = json.dumps(
            payload, sort_keys=True, separators=(",", ":"), allow_nan=False,
        ).encode("utf-8")
        self.assertEqual(hashlib.sha256(encoded).hexdigest(), claimed)
        for relative, expected in self.report["source_hashes"].items():
            path = ROOT / relative
            self.assertTrue(path.is_file(), relative)
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),
                             expected, relative)


if __name__ == "__main__":
    unittest.main()
