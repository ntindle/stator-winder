"""Focused geometry regressions for the isolated passive-cam review CAD."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import passive_cam_slot_guide as guide
from params import DEFAULT_STATOR
import stator_model


class PassiveCamSlotGuideCadTests(unittest.TestCase):

    def test_horn_contract_is_mouth_only_and_R3(self):
        contract = guide.horn_contract()
        self.assertIs(contract["mouth_only"], True)
        self.assertIs(contract["wire_center_R3"], True)
        self.assertTrue(math.isclose(
            contract["horn_wire_center_radius_mm"], 3.11176,
            rel_tol=0.0, abs_tol=1e-9,
        ))
        self.assertGreater(
            contract["minimum_horn_material_radius_mm"],
            contract["packed_inner_neck_max_radius_mm"],
        )

    def test_four_quarter_horns_never_enter_inner_neck(self):
        for slot_side in (-1, 1):
            for axial_sign in (-1, 1):
                horn = guide.quarter_horn(slot_side, axial_sign)
                bounds = horn.bounding_box()
                self.assertEqual(len(horn.solids()), 1)
                self.assertGreaterEqual(
                    bounds.min.X, guide.MOUTH_EXIT_RADIUS_MM - 1e-7)
                self.assertLessEqual(
                    bounds.max.X,
                    guide.MOUTH_EXIT_RADIUS_MM
                    + guide.HORN_SURFACE_RADIUS_MM + 1e-7,
                )
                self.assertTrue(math.isclose(
                    bounds.size.Y,
                    guide.GUIDE_TANGENTIAL_THICKNESS_MM,
                    rel_tol=0.0,
                    abs_tol=1e-6,
                ))

    def test_engaged_fingers_do_not_intersect_exact_default_steel(self):
        stator = stator_model.stator(DEFAULT_STATOR)
        for finger in guide.guide_parts():
            common = finger & stator
            self.assertEqual(len(common.solids()), 0)
            self.assertLessEqual(common.volume, 1e-9)

    def test_review_assembly_is_labeled_and_bounded(self):
        review = guide.gen_step()
        bounds = review.bounding_box()
        self.assertEqual(
            review.label, "passive_cam_slot_guide_no_go_review")
        self.assertEqual(len(review.children), 3)
        self.assertEqual(len(review.solids()), 8)
        self.assertTrue(math.isclose(bounds.size.X, 59.8, abs_tol=1e-6))
        self.assertTrue(math.isclose(bounds.size.Y, 46.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(bounds.size.Z, 38.0, abs_tol=1e-6))


if __name__ == "__main__":
    unittest.main()
