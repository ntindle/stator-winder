"""Regression tests for the fail-closed layer-staggered crown study."""

from __future__ import annotations

from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import unittest

import numpy as np


HERE = Path(__file__).resolve().parent
CAD = HERE.parent / "cad"
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

import slot_wire_routes
from crown_route_study import (
    HALF_TWIST_OUTPUT_PATH,
    OUTPUT_PATH,
    RADIAL_AXIAL_OUTPUT_PATH,
    SCHEMA,
    _canonical_hash,
)
from crown_routes import (
    CurrentHalfObstacle,
    DEFAULT_CROWN_POLICY,
    adjacent_self_clearance,
    build_c1_crown_route,
    build_current_half_obstacle,
    crowned_loop_centerline,
    crowned_loop_point,
    half_twist_bridge_midpoint_local_z_mm,
    half_twist_curvature_proof,
    packing_frame_half_twist_policy,
    radial_axial_curvature_study,
    radial_axial_dubins_policy,
)
from slot_route import PackingSupportGraph, _rounded_loop_yz


class CrownRouteGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        packing = json.loads(slot_wire_routes.PACKING_PATH.read_text())
        cls.spec = slot_wire_routes._validate_packing_contract(packing)
        cls.graph = PackingSupportGraph.from_report(
            packing, spec=cls.spec)
        cls.planner = slot_wire_routes.build_planner(cls.graph, cls.spec)

    def test_layer_stagger_preserves_packed_side_points(self):
        for turn_index in (0, 42, 45, 49):
            turn = self.graph.turn(turn_index)
            for half_turn_index in (0, 1):
                phase = half_turn_index * math.pi
                expected = np.array((
                    turn.radial_mm,
                    *_rounded_loop_yz(
                        turn.profile_radius_mm, phase, self.spec),
                ))
                self.assertTrue(np.allclose(
                    crowned_loop_point(turn, phase, self.spec),
                    expected,
                    rtol=0.0,
                    atol=1e-8,
                ))

    def test_turn42_turn45_positive_crown_seed_exceeds_1_25_mm(self):
        turn42 = self.graph.turn(42)
        turn45 = self.graph.turn(45)
        crown42 = crowned_loop_centerline(turn42, self.spec)
        crown45 = crowned_loop_centerline(turn45, self.spec)
        positive_apex_separation = abs(
            float(np.max(crown42[:, 2]))
            - float(np.max(crown45[:, 2])))
        self.assertGreaterEqual(positive_apex_separation, 1.25)

    def test_c1_route_is_radius_controlled_exact_and_mirrored(self):
        routes = [build_c1_crown_route(
            self.planner, self.graph, self.spec, 45, half)
            for half in (0, 1)]
        for route in routes:
            points = np.asarray(route.points_local_mm)
            self.assertTrue(np.allclose(
                points[-1], route.target_local_mm,
                rtol=0.0, atol=1e-8))
            self.assertGreaterEqual(route.minimum_bend_radius_mm, 3.0)
            self.assertLessEqual(
                route.join_tangent_error_deg,
                DEFAULT_CROWN_POLICY.route_arc_step_deg / 2.0 + 1e-6)
            self.assertLessEqual(
                route.terminal_tangent_error_deg,
                DEFAULT_CROWN_POLICY.route_arc_step_deg / 2.0 + 1e-6)
            self.assertLessEqual(route.tip_exit_tangent_error_deg, 1e-6)
        positive = np.asarray(routes[0].points_local_mm)
        negative = np.asarray(routes[1].points_local_mm)
        self.assertTrue(np.allclose(
            positive,
            negative * np.array((1.0, -1.0, -1.0)),
            rtol=0.0,
            atol=1e-8,
        ))

    def test_short_bridge_fails_instead_of_reversing_the_fillet(self):
        short = replace(DEFAULT_CROWN_POLICY, bridge_length_mm=0.5)
        with self.assertRaisesRegex(RuntimeError, "bridge is too short"):
            build_c1_crown_route(
                self.planner, self.graph, self.spec, 45, 0, short)

    def test_both_motion_signs_build_complementary_full_halves(self):
        obstacles = [build_current_half_obstacle(
            self.graph, self.spec, 45, 0, sign)
            for sign in (-1, 1)]
        target = crowned_loop_point(self.graph.turn(45), 0.0, self.spec)
        opposite = crowned_loop_point(
            self.graph.turn(45), math.pi, self.spec)
        for obstacle in obstacles:
            points = np.asarray(obstacle.points_local_mm)
            self.assertTrue(np.allclose(
                points[0], opposite, rtol=0.0, atol=1e-8))
            self.assertTrue(np.allclose(
                points[-1], target, rtol=0.0, atol=1e-8))
            self.assertGreater(obstacle.length_mm, 0.0)
        self.assertNotEqual(obstacles[0].sha256, obstacles[1].sha256)
        self.assertFalse(np.allclose(
            obstacles[0].points_local_mm[len(obstacles[0].points_local_mm)//2],
            obstacles[1].points_local_mm[len(obstacles[1].points_local_mm)//2],
            rtol=0.0,
            atol=1e-3,
        ))

    def test_half_twist_keeps_full_3d_phase_and_has_curvature_proof(self):
        policy = packing_frame_half_twist_policy(
            self.graph, base_radius_mm=21.0, bridge_step_mm=0.05)
        turn = self.graph.turn(45)
        loop = crowned_loop_centerline(turn, self.spec, policy)
        self.assertGreater(float(np.ptp(loop[:, 0])), 1.0)
        expected_zero = np.array((
            turn.radial_mm,
            -max(2.5, float(self.spec.od) * 0.07) / 2.0
            - turn.profile_radius_mm,
            float(self.spec.stack) / 2.0,
        ))
        expected_pi = expected_zero * np.array((1.0, -1.0, -1.0))
        self.assertTrue(np.allclose(
            crowned_loop_point(turn, 0.0, self.spec, policy),
            expected_zero, rtol=0.0, atol=1e-8))
        self.assertTrue(np.allclose(
            crowned_loop_point(turn, math.pi, self.spec, policy),
            expected_pi, rtol=0.0, atol=1e-8))
        for sign in (-1, 1):
            current = build_current_half_obstacle(
                self.graph, self.spec, 45, 0, sign, policy)
            points = np.asarray(current.points_local_mm)
            self.assertGreater(float(np.ptp(points[:, 0])), 1.0)
            self.assertTrue(np.allclose(
                points[0], expected_pi, rtol=0.0, atol=1e-8))
            self.assertTrue(np.allclose(
                points[-1], expected_zero, rtol=0.0, atol=1e-8))
        proof = half_twist_curvature_proof(
            self.graph, self.spec, policy)
        self.assertEqual(proof["status"], "PASS")
        self.assertGreaterEqual(
            proof["overall_bend_radius_lower_bound_mm"], 3.0)
        seed = (
            half_twist_bridge_midpoint_local_z_mm(
                self.graph.turn(45), self.spec, policy)
            - half_twist_bridge_midpoint_local_z_mm(
                self.graph.turn(42), self.spec, policy))
        self.assertGreaterEqual(seed, 1.25)

    def test_radial_axial_candidate_stays_tangentially_bounded(self):
        policy = radial_axial_dubins_policy()
        loops = [crowned_loop_centerline(
            self.graph.turn(index), self.spec, policy)
            for index in (0, 42, 45, 49)]
        for index, loop in zip((0, 42, 45, 49), loops):
            turn = self.graph.turn(index)
            tangential_limit = (
                max(2.5, float(self.spec.od) * 0.07) / 2.0
                + turn.profile_radius_mm)
            self.assertLessEqual(
                float(np.max(np.abs(loop[:, 1]))),
                tangential_limit + 1e-9)
            self.assertTrue(np.allclose(
                crowned_loop_point(turn, math.pi, self.spec, policy),
                loop[0] * np.array((1.0, -1.0, -1.0)),
                rtol=0.0, atol=1e-8))
        study = radial_axial_curvature_study(
            self.graph, self.spec, policy,
            samples_per_arc_degree=10)
        self.assertGreater(
            study["sampled_minimum_bend_radius_mm"], 3.0)
        self.assertEqual(study["status"], "NOT_PROVEN")


class AdjacentSelfRuleTests(unittest.TestCase):
    @staticmethod
    def obstacle(points: tuple[tuple[float, float, float], ...]
                 ) -> CurrentHalfObstacle:
        return CurrentHalfObstacle(
            turn_index=0,
            physical_half_index=0,
            motion_sign=1,
            start_phase_rad=-math.pi,
            end_phase_rad=0.0,
            points_local_mm=points,
            length_mm=sum(
                float(np.linalg.norm(np.asarray(b) - np.asarray(a)))
                for a, b in zip(points, points[1:])),
            sha256="synthetic",
        )

    def test_straight_shared_endpoint_exempts_only_two_diameters(self):
        route = ((-1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        current = self.obstacle(((1.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
        result = adjacent_self_clearance(route, current, 0.22352)
        self.assertAlmostEqual(
            result.combined_geodesic_to_endpoint_mm,
            2.0 * 0.22352,
            places=12,
        )
        self.assertAlmostEqual(
            result.minimum_centerline_distance_mm,
            2.0 * 0.22352,
            places=12,
        )

    def test_nonlocal_overlap_is_not_hidden_by_endpoint_adjacency(self):
        route = ((-1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        current = self.obstacle((
            (-0.5, -1.0, 0.0),
            (-0.5, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        ))
        result = adjacent_self_clearance(route, current, 0.22352)
        self.assertAlmostEqual(result.minimum_centerline_distance_mm, 0.0)
        self.assertGreater(
            result.combined_geodesic_to_endpoint_mm,
            result.adjacency_limit_mm,
        )


class CrownRouteReportTests(unittest.TestCase):
    def test_report_is_hash_bound_complete_and_fail_closed(self):
        report = json.loads(Path(OUTPUT_PATH).read_text())
        self.assertEqual(report["schema"], SCHEMA)
        claimed = report.pop("report_sha256")
        self.assertEqual(claimed, _canonical_hash(report))
        validation = report["validation"]
        self.assertEqual(len(report["routes"]), 100)
        self.assertEqual(
            sum(len(row["motion_sign_cases"])
                for row in report["routes"]),
            200,
        )
        self.assertEqual(report["status"], "FAIL")
        self.assertFalse(validation["release_flags"][
            "physical_error_budget_complete_measured_evidence"])
        self.assertLess(validation["motion_sign_passed"], 200)

    def test_half_twist_report_covers_all_cases_and_fails_neighbors(self):
        report = json.loads(Path(HALF_TWIST_OUTPUT_PATH).read_text())
        self.assertEqual(report["schema"], SCHEMA)
        claimed = report.pop("report_sha256")
        self.assertEqual(claimed, _canonical_hash(report))
        self.assertEqual(len(report["routes"]), 100)
        self.assertEqual(sum(
            len(row["motion_sign_cases"]) for row in report["routes"]), 200)
        self.assertEqual(report["status"], "FAIL")
        flags = report["validation"]["release_flags"]
        self.assertTrue(flags["turn42_turn45_seed_separation_at_least_1_25mm"])
        self.assertTrue(flags["deposited_crown_curvature_proven"])
        self.assertFalse(flags["deposited_crowns_clear"])
        self.assertFalse(flags["machine_crown_envelope_validated"])
        audit = report["deposited_crown_audit"]
        self.assertLess(audit[
            "minimum_neighbor_centerline_lower_bound_mm"], 0.22352)
        self.assertTrue(any(
            row["pair_kind"] == "active_to_neighbor_tooth"
            for row in audit["failed_pairs"]))

    def test_radial_axial_report_is_complete_and_fails_self_crossing(self):
        report = json.loads(Path(RADIAL_AXIAL_OUTPUT_PATH).read_text())
        claimed = report.pop("report_sha256")
        self.assertEqual(claimed, _canonical_hash(report))
        self.assertEqual(len(report["routes"]), 100)
        self.assertEqual(sum(
            len(row["motion_sign_cases"]) for row in report["routes"]), 200)
        self.assertEqual(report["status"], "FAIL")
        self.assertEqual(
            report["policy"]["geometry_family"], "radial_axial_dubins")
        self.assertGreaterEqual(
            report["turn42_turn45_seed_separation_mm"], 1.25)
        self.assertLess(
            report["deposited_crown_audit"][
                "minimum_noncontact_centerline_lower_bound_mm"],
            0.22352)
        self.assertFalse(report["validation"]["release_flags"][
            "deposited_crowns_clear"])


if __name__ == "__main__":
    unittest.main()
