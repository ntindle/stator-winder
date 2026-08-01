"""Regression tests for the dancer quasi-static audit."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dancer_loads as D


class DancerGeometryTests(unittest.TestCase):
    def test_nominal_geometry_matches_authoritative_wire(self):
        pose = D.wire_pose(D.NOMINAL_ANGLE_DEG)
        self.assertAlmostEqual(pose.center[0], D.NOMINAL_CENTER[0], places=9)
        self.assertAlmostEqual(pose.center[1], D.NOMINAL_CENTER[1], places=9)
        self.assertAlmostEqual(pose.wrap_deg, 80.0, places=8)
        self.assertAlmostEqual(pose.moment_per_tension_mm,
                               46.1305168807, places=7)

    def test_tangencies_are_exact_and_clockwise(self):
        for offset in (-12.0, -6.0, 0.0, 6.0, 12.0):
            pose = D.wire_pose(D.NOMINAL_ANGLE_DEG + offset)
            for tangent in (pose.tangent_in, pose.tangent_out):
                self.assertAlmostEqual(math.dist(tangent, pose.center),
                                       D.PATH_RADIUS, places=9)
            radial_in = D._sub(pose.tangent_in, pose.center)
            radial_out = D._sub(pose.tangent_out, pose.center)
            self.assertAlmostEqual(D._dot(radial_in,
                D._sub(pose.tangent_in, D.UPSTREAM)), 0.0, places=8)
            self.assertAlmostEqual(D._dot(radial_out,
                D._sub(D.DOWNSTREAM, pose.tangent_out)), 0.0, places=8)


class DancerLoadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.design, cls.result, _ = D.search_design()

    def test_search_finds_fully_feasible_catalog_design(self):
        self.assertTrue(self.result["ok"], self.result["failures"])
        self.assertEqual(len(self.result["sweep"]), 10)

    def test_every_equilibrium_is_balanced_and_restoring(self):
        for row in self.result["sweep"]:
            self.assertLess(abs(row["moment_residual_n_mm"]), 1e-5)
            self.assertLess(row["stability_n_mm_per_deg"], 0.0)

    def test_rated_spring_and_wire_clearances_hold(self):
        metrics = self.result["metrics"]
        spring = self.design.spring
        self.assertGreaterEqual(metrics["spring_length_range_mm"][0],
                                spring.free_length_mm - 1e-6)
        self.assertLessEqual(metrics["spring_length_range_mm"][1],
                             spring.max_length_mm + 1e-6)
        self.assertGreaterEqual(metrics["minimum_entry_channel_clearance_mm"],
                                D.MIN_ENTRY_CHANNEL_CLEARANCE)
        self.assertGreaterEqual(metrics["minimum_felt_stud_clearance_mm"],
                                D.MIN_FELT_STUD_CLEARANCE)
        self.assertGreaterEqual(metrics["minimum_spring_wire_clearance_mm"],
                                D.MIN_SPRING_WIRE_CLEARANCE)

    def test_stop_to_stop_angle_sweep_brackets_tension_range(self):
        rows = self.result["angle_sweep"]
        self.assertGreater(len(rows), 20)
        self.assertLess(rows[0]["equivalent_equilibrium_tension_n"], 1.0)
        self.assertGreater(rows[-1]["equivalent_equilibrium_tension_n"], 10.0)
        self.assertLessEqual(max(row["felt_deflection_deg"] for row in rows),
                             D.FELT_DEFLECTION_LIMIT_DEG)
        self.assertGreaterEqual(min(row["entry_channel_clearance_mm"]
                                    for row in rows),
                                D.MIN_ENTRY_CHANNEL_CLEARANCE)

    def test_recommended_stop_pin_centers_touch_exact_arm_edges(self):
        offsets = self.result["metrics"]["hard_stop_offsets_deg"]
        pins = D.hard_stop_pin_centers(offsets)
        for row, side in zip(pins, (-1.0, 1.0)):
            angle = math.radians(D.NOMINAL_ANGLE_DEG + row["offset_deg"])
            arm = (math.cos(angle), math.sin(angle))
            normal = (-math.sin(angle), math.cos(angle))
            delta = D._sub(tuple(row["center_xy"]), D.PIVOT)
            self.assertAlmostEqual(D._dot(delta, arm),
                                   D.STOP_CONTACT_RADIUS, places=8)
            self.assertAlmostEqual(D._dot(delta, normal),
                                   side * (D.ARM_HALF_WIDTH
                                           + D.STOP_PIN_RADIUS), places=8)

    def test_stop_boss_is_axially_retracted_and_only_pin_contacts(self):
        offsets = self.result["metrics"]["hard_stop_offsets_deg"]
        pins = D.hard_stop_pin_centers(offsets)
        self.assertAlmostEqual(D._z_gap(D.FIXED_BOSS_Z, D.ARM_Z), 1.0)
        self.assertAlmostEqual(D._z_gap(D.STOP_PIN_Z, D.ARM_Z), -2.5)
        self.assertAlmostEqual(D._z_gap(D.STOP_WASHER_Z, D.ARM_Z), 0.5)
        for row in pins:
            center = tuple(row["center_xy"])
            # This is the caught regression: a radius-4.5 boss in the arm
            # slab penetrates even at nominal, while the Ø5 pin is clear.
            self.assertLess(D._circle_arm_clearance(
                center, 4.5, D.NOMINAL_ANGLE_DEG), 0.0)
            self.assertGreater(D._circle_arm_clearance(
                center, D.STOP_PIN_RADIUS, D.NOMINAL_ANGLE_DEG), 0.0)
            self.assertAlmostEqual(D._circle_arm_clearance(
                center, D.STOP_PIN_RADIUS,
                D.NOMINAL_ANGLE_DEG + row["offset_deg"]), 0.0, places=8)

    def test_spring_is_in_front_of_arm_and_still_clears_wire(self):
        radius = self.design.spring.od_mm / 2.0
        spring_body = (D.SPRING_PLANE_Z - radius,
                       D.SPRING_PLANE_Z + radius)
        self.assertAlmostEqual(D._z_gap(spring_body, D.ARM_Z), 4.5)
        self.assertAlmostEqual(D._z_gap(D.SPRING_BRIDGE_Z, D.ARM_Z), 1.0)
        self.assertAlmostEqual(D._z_gap(spring_body, D.SPRING_BRIDGE_Z), 0.5)
        self.assertGreater(self.result["metrics"]
                           ["minimum_spring_wire_clearance_mm"], 5.0)
        self.assertLess(D._circle_arm_clearance(
            self.design.fixed_anchor, 4.5, D.NOMINAL_ANGLE_DEG), 0.0)
        riser_clearances = []
        bridge_clearances = []
        moving_pin_clearances = []
        for row in self.result["angle_sweep"]:
            angle = D.NOMINAL_ANGLE_DEG + row["offset_deg"]
            wire = D.wire_pose(angle)
            riser_clearances.append(D._circle_arm_clearance(
                D.SPRING_RISER_ROOT, D.SPRING_RISER_RADIUS, angle))
            bridge_clearances.append(min(
                D._segment_distance(D.SPRING_RISER_ROOT,
                                    self.design.fixed_anchor,
                                    D.UPSTREAM, wire.tangent_in),
                D._segment_distance(D.SPRING_RISER_ROOT,
                                    self.design.fixed_anchor,
                                    wire.tangent_out, D.DOWNSTREAM),
            ) - D.SPRING_BRIDGE_HALF_WIDTH - D.WIRE_RADIUS)
            angle_rad = math.radians(angle)
            moving = D._add(D.PIVOT, D._mul(
                (math.cos(angle_rad), math.sin(angle_rad)),
                self.design.moving_anchor_radius_mm))
            moving_pin_clearances.append(D._point_segment_distance(
                moving, D.SPRING_RISER_ROOT, self.design.fixed_anchor)
                - 2.0 - D.SPRING_BRIDGE_HALF_WIDTH)
        self.assertGreaterEqual(min(riser_clearances), 0.5)
        self.assertGreater(min(bridge_clearances), 11.0)
        self.assertGreater(min(moving_pin_clearances), 2.9)


if __name__ == "__main__":
    unittest.main()
