"""Focused proofs for the isolated fixed-M0 topology study."""

from __future__ import annotations

import json
import math
import unittest

import numpy as np

import helical_loop_topology as study
from crown_routes import CrownPolicy, adjacent_self_clearance
from slot_route import PackingSupportGraph
import slot_wire_routes


class HelicalLoopTopologyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        packing = json.loads(
            study.PACKING_PATH.read_text(encoding="utf-8"))
        cls.spec = slot_wire_routes._validate_packing_contract(packing)
        cls.graph = PackingSupportGraph.from_report(
            packing, spec=cls.spec)
        cls.planner = slot_wire_routes.build_planner(cls.graph, cls.spec)

    def test_captured_plan_forbids_intra_turn_radial_hairpin(self):
        plan = json.loads(study.PLAN_PATH.read_text(encoding="utf-8"))
        audit = study._validate_captured_m0_contract(plan)
        self.assertEqual(audit["status"], "PASS")
        self.assertEqual(audit["checked_turns"], 50)
        self.assertFalse(audit["radial_motion_within_turn_allowed"])
        self.assertFalse(audit["failed_turn_indices"])

    def test_parallel_offset_loop_is_closed_simple_fixed_plane_and_radius_safe(self):
        maximum_profile = max(
            turn.profile_radius_mm for turn in self.graph.turns)
        study.DEFAULT_POLICY.validate(maximum_profile)
        loop = study.loop_components(
            self.graph.turn(49), self.spec, tooth_index=0,
            policy=study.DEFAULT_POLICY)
        points = loop.points_local_mm
        self.assertLessEqual(np.linalg.norm(points[0] - points[-1]), 1e-8)
        self.assertLessEqual(np.ptp(points[:, 0]), 1e-10)
        self.assertTrue(study._self_simple(loop))
        self.assertTrue(math.isclose(
            study.DEFAULT_POLICY.base_bend_radius_mm - maximum_profile,
            3.1193852813294294,
            rel_tol=0.0,
            abs_tol=1e-12,
        ))

    def test_exact_segments_reject_the_straight_parity_lane(self):
        active = study.loop_components(
            self.graph.turn(42), self.spec, tooth_index=0,
            policy=study.DEFAULT_POLICY)
        neighbor = study.loop_components(
            self.graph.turn(47), self.spec, tooth_index=1,
            policy=study.DEFAULT_POLICY)
        transformed = study._rotate_tooth(
            neighbor.points_local_mm, 1, int(self.spec.slots))
        distance, _, _ = study._polyline_pair_clearance(
            active.points_local_mm, transformed, 0.5)
        self.assertLess(distance, 0.01)
        self.assertLess(distance, self.graph.wire_diameter_mm)

    def test_sign_aware_free_span_clears_both_arriving_halves_for_seed_turn(self):
        loop = study.loop_components(
            self.graph.turn(0), self.spec, tooth_index=0,
            policy=study.DEFAULT_POLICY)
        for half in (0, 1):
            for sign in (-1, 1):
                route = study._build_sign_aware_route(
                    self.planner, self.graph, self.spec, 0, half, sign,
                    study.DEFAULT_POLICY)
                current = study._current_half(
                    loop, self.graph, self.spec, 0, half, sign)
                clearance = adjacent_self_clearance(
                    route.points_local_mm, current,
                    self.graph.wire_diameter_mm,
                    CrownPolicy(), search_band_mm=0.6)
                self.assertGreater(
                    clearance.minimum_centerline_distance_mm,
                    self.graph.wire_diameter_mm
                    + study.known_physical_lower_bound_mm())


if __name__ == "__main__":
    unittest.main()
