"""Regression tests for the coupled, slot-aware moving-wire planner."""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

import numpy as np
import trimesh
from build123d import Box

HERE = Path(__file__).resolve().parent
CAD = HERE.parent / "cad"
if str(CAD) not in sys.path:
    sys.path.insert(0, str(CAD))

import coil_growth
from params import DEFAULT_STATOR
import stator_model
import wire_geometry

from slot_route import (
    PackingSupportGraph,
    SlotRoutePlanner,
    build_deposited_profiles,
    classify_active_loop_contacts,
    dependency_versions,
    exact_polyline_mesh_clearance,
    exact_polyline_part_clearance,
    segment_triangle_distance,
)

import slot_packing_audit
import slot_packing


class ExactDistanceKernelTests(unittest.TestCase):
    def test_segment_triangle_crossing_and_parallel_offset(self):
        triangle = np.array((
            (0.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (0.0, 2.0, 0.0),
        ))
        self.assertAlmostEqual(
            segment_triangle_distance(
                np.array((0.5, 0.5, -1.0)),
                np.array((0.5, 0.5, 1.0)),
                triangle,
            ),
            0.0,
            places=12,
        )
        self.assertAlmostEqual(
            segment_triangle_distance(
                np.array((-0.5, 0.5, 1.0)),
                np.array((1.5, 0.5, 1.0)),
                triangle,
            ),
            1.0,
            places=12,
        )

    def test_polyline_mesh_query_reports_segment_and_triangle(self):
        mesh = trimesh.creation.box(extents=(2.0, 2.0, 2.0))
        distance, segment, triangle = exact_polyline_mesh_clearance(
            np.array(((-2.0, 1.25, 0.0), (2.0, 1.25, 0.0))),
            mesh,
        )
        self.assertAlmostEqual(distance, 0.25, places=12)
        self.assertEqual(segment, 0)
        self.assertIsInstance(triangle, int)

    def test_polyline_occ_query_uses_source_part(self):
        distance = exact_polyline_part_clearance(
            np.array(((-1.0, 3.0, 1.0), (3.0, 3.0, 1.0))),
            Box(2.0, 2.0, 2.0),
        )
        self.assertAlmostEqual(distance, 2.0, places=12)

    def test_dependency_versions_pin_shapely(self):
        versions = dependency_versions()
        self.assertEqual(versions["shapely"], "2.1.2")
        for package in ("build123d", "numpy", "scipy", "trimesh"):
            self.assertNotEqual(versions[package], "missing")


class ProjectPlannerTests(unittest.TestCase):
    """Small real-CAD sentinel set; the exhaustive table is capture-driven."""

    @classmethod
    def setUpClass(cls):
        cls.spec = DEFAULT_STATOR
        policy = replace(
            coil_growth.DEFAULT_POLICY,
            opening_edge_clearance_mm=0.20,
        )
        cls.coil = coil_growth.analyze_job(cls.spec, policy)
        cls.part = stator_model.stator(cls.spec)
        vertices, faces = cls.part.tessellate(0.01, 0.03)
        cls.mesh = trimesh.Trimesh(
            vertices=np.array([(v.X, v.Y, v.Z) for v in vertices]),
            faces=np.asarray(faces),
            process=True,
        )
        if not cls.mesh.is_watertight:
            raise AssertionError("real stator sentinel mesh is not watertight")
        cls.planner = SlotRoutePlanner(
            spec=cls.spec,
            stator_part=cls.part,
            stator_mesh_local=cls.mesh,
            guide=wire_geometry.tip_guide_spec(),
            contact=wire_geometry.tooth_contact_spec(cls.spec, cls.coil),
            guide_wire_radius_mm=0.25,
            access_radius_mm=0.320,
            # OCCT cannot offset this fused stator reliably at 0.320; 0.328
            # is a conservative planning shell.  The exact postcheck and
            # physical support target remain at 0.320.
            planner_offset_mm=0.328,
            clamp_goal_to_stack=False,
            visibility_chord_mm=0.01,
        )

    def assert_valid_route(self, result):
        self.assertTrue(result.ok, result.reason)
        self.assertGreaterEqual(result.center_core_min_mm, 0.320 - 1e-9)
        self.assertLess(result.torus_continuity_error_deg, 1e-9)
        self.assertIsNotNone(result.torus_exit_point_index)
        self.assertEqual(len(result.segment_tags), len(result.points_local_mm) - 1)
        self.assertIn("tip_guide_contact", result.segment_tags)
        self.assertIn("free", result.segment_tags)
        self.assertFalse(result.progressive_support_validated)
        self.assertEqual(result.boundary_source, "bare_core_offset_profile")

    def test_shallow_shoe_and_rounded_end_sentinels(self):
        for angle_deg, motion_sign in ((0.0, -1), (270.0, -1)):
            with self.subTest(angle_deg=angle_deg, motion_sign=motion_sign):
                result = self.planner.route(
                    20.68,
                    math.radians(angle_deg),
                    motion_sign,
                    endpoint_family="liner_outbound",
                    support_profile_radius_mm=0.320,
                )
                self.assert_valid_route(result)
                self.assertEqual(result.endpoint_support, "slot_liner_glide")

    def test_prior_deep_failures_clear_with_revised_access_radius(self):
        radial = 14.786304320792953
        for angle_deg, motion_sign in (
            (86.0, -1), (94.0, 1), (266.0, -1), (274.0, 1)
        ):
            with self.subTest(angle_deg=angle_deg, motion_sign=motion_sign):
                result = self.planner.route(
                    radial,
                    math.radians(angle_deg),
                    motion_sign,
                    endpoint_family="liner_outbound",
                    support_profile_radius_mm=0.320,
                )
                self.assert_valid_route(result)

    def test_boundary_portal_is_nudged_outward_before_visibility_query(self):
        # GEOS formerly classified the exact nearest boundary coordinate for
        # this pose as polygon interior and returned zero goal edges.
        result = self.planner.route(
            14.786304320792953,
            math.radians(25.0),
            1,
            endpoint_family="liner_outbound",
            support_profile_radius_mm=0.320,
        )
        self.assert_valid_route(result)
        nudge = result.metadata["planner_portal_nudge_mm"]
        self.assertGreater(nudge, 0.0)
        self.assertLess(nudge, 1e-6)
        self.assertNotEqual(
            result.metadata["raw_goal_local_yz_mm"],
            result.metadata["goal_local_yz_mm"],
        )

    def test_unmodeled_deposited_profile_fails_closed(self):
        result = self.planner.route(
            14.786304320792953,
            math.radians(90.0),
            1,
            endpoint_family="deposited_layer",
            support_profile_radius_mm=0.560,
        )
        self.assertFalse(result.ok)
        self.assertIn("support profile", result.reason)
        self.assertFalse(result.progressive_support_validated)


class PackingSupportGraphTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.spec = DEFAULT_STATOR
        cls.report = slot_packing_audit.analyze()
        cls.graph = PackingSupportGraph.from_report(
            cls.report, spec=cls.spec)

    def test_hash_bound_graph_and_v2_profile_normalization(self):
        self.assertEqual(len(self.graph.turns), 50)
        rows = self.report["selected_schedule"]["side_positive"]
        half_neck = max(2.5, self.spec.od * 0.07) / 2.0
        for turn_index in (0, 19, 32, 44, 49):
            turn = self.graph.turn(turn_index)
            row = rows[turn_index]
            self.assertEqual(turn.layer_index, row["layer_index"])
            self.assertAlmostEqual(
                turn.profile_radius_mm,
                row["normal_profile_radius_mm"] - half_neck,
                places=12,
            )
            self.assertAlmostEqual(
                turn.radial_mm, row["radial_parameter_mm"], places=12)
        self.assertEqual(
            [sum(turn.layer_index == layer for turn in self.graph.turns)
             for layer in range(4)],
            self.report["selected_schedule"]["layer_counts"],
        )

    def test_tampered_or_mismatched_report_fails_closed(self):
        tampered = dict(self.report)
        tampered["status"] = "FAIL"
        with self.assertRaises(ValueError):
            PackingSupportGraph.from_report(tampered, spec=self.spec)
        with self.assertRaises(ValueError):
            PackingSupportGraph.from_report(
                self.report,
                spec=replace(self.spec, wire_d=self.spec.wire_d + 0.001),
            )

    def test_profiles_are_closed_and_stator_local(self):
        profiles = build_deposited_profiles(
            self.graph, 19, self.spec, arc_step_deg=5.0)
        self.assertEqual(len(profiles), 20)
        profile = profiles[-1]
        points = np.asarray(profile.centerline_local_mm)
        self.assertTrue(np.allclose(points[:, 0], profile.radial_mm))
        self.assertTrue(np.allclose(points[0], points[-1]))
        self.assertGreater(len(points), 70)

    def test_every_selected_transition_classifies_without_overlap(self):
        for turn_index in (0, 1, 19, 32, 44, 49):
            with self.subTest(turn_index=turn_index):
                audit = classify_active_loop_contacts(
                    self.graph, turn_index)
                self.assertTrue(audit.ok, audit.reason)
                self.assertTrue(audit.progressive_support_validated)
                self.assertNotIn(
                    "overlap",
                    {contact.classification for contact in audit.contacts},
                )
        parent_audit = classify_active_loop_contacts(self.graph, 19)
        parent_hits = {
            contact.prior_turn_index
            for contact in parent_audit.contacts
            if contact.classification == "intended_parent_tangent"
        }
        self.assertEqual(
            parent_hits, set(self.graph.turn(19).parent_turn_indices))

    def test_authoritative_winding_plan_schema_is_consumed_fail_closed(self):
        plan = slot_packing.build_plan()
        graph = PackingSupportGraph.from_report(plan, spec=DEFAULT_STATOR)
        self.assertEqual(graph.schema, "slot-winding-plan/v1")
        self.assertEqual(len(graph.turns), 50)
        self.assertAlmostEqual(
            graph.wire_diameter_mm,
            plan["job"]["model_wire_envelope_mm"], places=12)
        self.assertAlmostEqual(
            graph.center_core_access_mm,
            plan["selected_case"][
                "required_center_core_clearance_mm"], places=12)
        for turn_index in (0, 1, 10, 25, 49):
            audit = classify_active_loop_contacts(graph, turn_index)
            self.assertTrue(audit.ok, (turn_index, audit.reason))
            self.assertTrue(audit.progressive_support_validated)
        tampered = dict(plan)
        tampered["proof_sha256"] = "0" * 64
        with self.assertRaises(ValueError):
            PackingSupportGraph.from_report(tampered, spec=DEFAULT_STATOR)


if __name__ == "__main__":
    unittest.main()
