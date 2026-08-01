"""Analytical regression tests for the moving wire contact construction."""

import math
from pathlib import Path
import unittest

import numpy as np
import trimesh

from traj import Timeline, load_events
from wirepath import (STATIC_WIRE_INTENTIONAL_CONTACTS,
                      _captured_wrap_intervals,
                      _case_clearances, _sample_polyline, _surface_clearance,
                      shaft_tangent_point,
                      tip_guide_path, tooth_contact_point,
                      tooth_support_tangents)


class ToothContactTests(unittest.TestCase):
    contact = {
        "physical_tangential_radius_mm": 4.61,
        "physical_axial_radius_mm": 10.5,
        "wire_offset_radius_mm": 0.25,
        "tangential_half_extent_mm": 4.86,
        "axial_half_extent_mm": 10.75,
        "z_mm": 2.0,
    }

    def test_contacts_are_support_tangents_on_offset_ellipse(self):
        a = self.contact["physical_tangential_radius_mm"]
        b = self.contact["physical_axial_radius_mm"]
        offset = self.contact["wire_offset_radius_mm"]
        boundary = []
        for theta in np.radians(np.arange(0.0, 360.0, 0.25)):
            c, s = math.cos(theta), math.sin(theta)
            body = np.array([a * c, b * s])
            normal = np.array([c / a, s / b])
            normal /= np.linalg.norm(normal)
            boundary.append(body + offset * normal)
        boundary = np.asarray(boundary)
        for angle in np.radians(np.arange(0.0, 360.0, 1.0)):
            tip = np.array([-45.0 * math.sin(angle),
                            45.0 * math.cos(angle), -1.5])
            tangents = tooth_support_tangents(tip, self.contact)
            self.assertEqual(set(tangents), {-1, 1})
            for side, point in tangents.items():
                line = point[:2] - tip[:2]
                crosses = (line[0] * (boundary[:, 1] - tip[1])
                           - line[1] * (boundary[:, 0] - tip[0]))
                self.assertTrue(np.all(crosses >= -2e-4) if side == 1
                                else np.all(crosses <= 2e-4))
                self.assertEqual(point[2], 2.0)

    def test_motion_sign_selects_trailing_support(self):
        tip = np.array([0.0, 45.0, -1.5])
        positive = tooth_contact_point(tip, self.contact, 1)
        negative = tooth_contact_point(tip, self.contact, -1)
        self.assertGreater(positive[0], 0.0)
        self.assertLess(negative[0], 0.0)
        self.assertAlmostEqual(positive[0], -negative[0], places=8)
        self.assertAlmostEqual(positive[1], negative[1], places=8)

    def test_rounded_contact_is_continuous_for_full_revolution(self):
        for motion_sign in (-1, 1):
            points = []
            for angle in np.radians(np.arange(0.0, 360.5, 0.5)):
                tip = np.array([-45.0 * math.sin(angle),
                                45.0 * math.cos(angle), -1.5])
                points.append(tooth_contact_point(
                    tip, self.contact, motion_sign,
                ))
            jumps = np.linalg.norm(np.diff(points, axis=0), axis=1)
            self.assertLess(float(jumps.max()), 1.0)


class ShaftContactTests(unittest.TestCase):
    contact = {"radius_to_wire_center_mm": 4.25, "axial_y_mm": 12.0}

    def test_both_constructed_points_are_true_circle_tangencies(self):
        tip = np.array([8.0, 44.0, -1.5])
        axis_z = 35.0
        for side in (-1, 1):
            target = shaft_tangent_point(tip, axis_z, self.contact, side)
            radius = np.array([target[0], target[2] - axis_z])
            line = np.array([tip[0] - target[0], tip[2] - target[2]])
            self.assertAlmostEqual(np.linalg.norm(radius), 4.25, places=8)
            self.assertAlmostEqual(np.dot(radius, line), 0.0, places=8)
            self.assertEqual(target[1], 12.0)

    def test_inside_projection_fails_closed(self):
        with self.assertRaises(ValueError):
            shaft_tangent_point(np.array([0.0, 45.0, 35.0]), 35.0,
                                self.contact, 1)

    def test_raw_upstream_wrap_intervals_are_inferred_from_commands(self):
        capture = (
            Path(__file__).resolve().parent.parent
            / "out" / "capture" / "upstream_current_raw.jsonl")
        events = load_events(capture)
        wraps, contract = _captured_wrap_intervals(
            events, Timeline(events))
        self.assertEqual(contract["source"], "raw_upstream_commands")
        self.assertTrue(contract["ok"])
        self.assertEqual([row["number"] for row in wraps], [1, 2])
        self.assertTrue(all(
            abs(row["end_m0"] - row["start_m0"]) <= 1e-9
            and abs(row["end_m2"] - row["start_m2"]) <= 1e-9
            for row in wraps
        ))


class TipGuideTests(unittest.TestCase):
    guide = {
        "center_local_mm": [0.0, 45.0, -1.5],
        "axis_local": [0.0, 1.0, 0.0],
        "feed_local_mm": [0.0, 12.0, -1.5],
        "major_radius_mm": 6.5,
        "tube_radius_mm": 3.0,
    }

    def test_torus_path_is_continuous_tangent_and_above_three_mm(self):
        feed = np.array(self.guide["feed_local_mm"])
        for target in (np.array([4.0, 10.0, 2.0]),
                       np.array([-4.0, 10.0, 2.0]),
                       np.array([4.25, 12.0, 36.0])):
            points, meta = tip_guide_path(
                feed, target, self.guide, 0.25,
            )
            self.assertGreater(len(points), 20)
            self.assertGreaterEqual(meta["wire_center_bend_radius_mm"], 3.0)
            self.assertGreaterEqual(meta["inside_wire_path_radius_mm"], 3.0)
            self.assertLessEqual(meta["entry_tangent_error"], 1.01)
            self.assertLessEqual(meta["exit_tangent_error"], 1.01)

    def test_segment_sampler_honors_declared_spacing(self):
        points = _sample_polyline(
            [[0.0, 0.0, 0.0], [0.0, 1.1, 0.0], [2.0, 1.1, 0.0]],
            spacing=0.25,
        )
        self.assertLessEqual(
            float(np.linalg.norm(np.diff(points, axis=0), axis=1).max()),
            0.25 + 1e-12,
        )


class ClearanceKernelTests(unittest.TestCase):
    def test_felt_base_and_hardware_are_not_blanket_excluded(self):
        self.assertNotIn("felt_tensioner", STATIC_WIRE_INTENTIONAL_CONTACTS)
        self.assertNotIn("felt_m4x55_stud", STATIC_WIRE_INTENTIONAL_CONTACTS)
        self.assertIn("felt_pad_fixed", STATIC_WIRE_INTENTIONAL_CONTACTS)
        self.assertIn("felt_pad_moving", STATIC_WIRE_INTENTIONAL_CONTACTS)

    def test_unsigned_surface_kernel_detects_a_continuous_crossing(self):
        mesh = trimesh.creation.box(extents=[2.0, 2.0, 2.0])
        points = _sample_polyline(
            [[-2.0, 0.0, 0.0], [2.0, 0.0, 0.0]], spacing=0.25,
        )
        value = _surface_clearance(
            trimesh.proximity.ProximityQuery(mesh), points, 0.25,
        )
        self.assertLess(value, 0.0)

    def test_part_aabb_branch_and_bound_matches_exact_distance(self):
        mesh = trimesh.creation.box(extents=[2.0, 2.0, 2.0])
        near = _sample_polyline(
            [[-2.0, 1.5, 0.0], [2.0, 1.5, 0.0]], spacing=0.25,
        )
        far = _sample_polyline(
            [[-2.0, 8.0, 0.0], [2.0, 8.0, 0.0]], spacing=0.25,
        )
        cases = [
            {"points": near, "meta": {"case": "near"}},
            {"points": far, "meta": {"case": "far"}},
        ]
        ranked = _case_clearances(
            {"static": {"box": mesh}}, cases, 0.25,
            initial_band=1.0, rank_count=1,
        )
        expected = _surface_clearance(
            trimesh.proximity.ProximityQuery(mesh), near, 0.25,
        )
        self.assertEqual(ranked[0][2]["meta"]["case"], "near")
        self.assertAlmostEqual(ranked[0][0], expected, places=9)


if __name__ == "__main__":
    unittest.main(verbosity=2)
