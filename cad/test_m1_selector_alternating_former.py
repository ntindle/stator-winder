"""Regression tests for the isolated M1 selector/former review CAD."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import m1_selector_alternating_former as mechanism
from params import PARAMS


class M1SelectorAlternatingFormerCadTests(unittest.TestCase):

    def test_all_24_m1_sectors_have_exactly_three_positive_codes(self):
        self.assertEqual(
            sorted(mechanism.M1_ANGLE_TO_LAW), list(range(0, 360, 15)))
        counts = {
            law: sum(value == law
                     for value in mechanism.M1_ANGLE_TO_LAW.values())
            for law in mechanism.LAW_CODES
        }
        self.assertEqual(counts, {
            mechanism.LAW_DIRECT: 12,
            mechanism.LAW_REVERSE_ZERO: 2,
            mechanism.LAW_REVERSE_180: 10,
        })
        self.assertEqual(len(set(mechanism.CODE_RADII_MM.values())), 3)

    def test_m0_gate_is_positive_over_wind_index_wrap_and_load_poses(self):
        self.assertEqual(mechanism.gate_state_for_axis_z(
            PARAMS.stator_axis_z(-61.918)), "ENGAGED_LOCKED")
        self.assertEqual(mechanism.gate_state_for_axis_z(
            PARAMS.stator_axis_z(-56.8)), "ENGAGED_LOCKED")
        self.assertEqual(mechanism.gate_state_for_axis_z(
            PARAMS.stator_axis_z(-47.124)), "ALL_RETRACTED_DISCONNECTED")
        self.assertEqual(mechanism.gate_state_for_axis_z(
            PARAMS.stator_axis_z(0.0)), "ALL_RETRACTED_DISCONNECTED")

    def test_docking_tongue_fully_overlaps_wind_and_clears_index_pose(self):
        receiver = mechanism.selector_receiver().bounding_box()

        def overlap(axis_z: float) -> float:
            tongue = mechanism.docking_tongue(axis_z).bounding_box()
            return max(0.0, min(tongue.max.Z, receiver.max.Z)
                       - max(tongue.min.Z, receiver.min.Z))

        self.assertTrue(math.isclose(
            overlap(PARAMS.stator_axis_z(-61.918)), 8.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(
            overlap(PARAMS.stator_axis_z(-56.8)), 8.0, abs_tol=1e-9))
        safe = mechanism.docking_tongue(
            PARAMS.stator_axis_z(-47.124)).bounding_box()
        self.assertGreaterEqual(safe.min.Z - receiver.max.Z, 2.999)

    def test_code_collar_and_R3_fingers_are_valid_review_solids(self):
        collar = mechanism.selector_code_collar()
        self.assertGreater(collar.volume, 0.0)
        self.assertEqual(len(collar.solids()), 1)
        self.assertGreaterEqual(mechanism.GUIDE_SURFACE_RADIUS_MM, 3.0)
        for finger_index in range(4):
            finger = mechanism.guide_finger(finger_index, deployed=True)
            self.assertGreater(finger.volume, 0.0)
            self.assertEqual(len(finger.solids()), 1)

    def test_review_contract_is_bound_to_deep_raw_pose(self):
        contract = mechanism.geometry_contract()
        self.assertEqual(contract["m1_sector_count"], 24)
        self.assertEqual(contract["cam_track_count"], 4)
        self.assertEqual(contract["review_law"], mechanism.LAW_REVERSE_ZERO)
        self.assertEqual(contract["guide_surface_radius_mm"], 3.0)
        self.assertEqual(contract["gate_at_review"], "ENGAGED_LOCKED")
        self.assertGreater(
            contract["maximum_code_collar_front_extent_z_mm"], 0.75)


if __name__ == "__main__":
    unittest.main()
