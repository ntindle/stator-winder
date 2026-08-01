"""Tests for the isolated elastic wire/contact feasibility study."""

from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import unittest

import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

from elastic_wire_contact_study import (  # noqa: E402
    AxisTimeline,
    CAPTURE,
    CONTACT_ROUTE_STEPS_DEG,
    OUTPUT_JSON,
    PACKING,
    ROUTES,
    SCHEMA,
    contact_arc_convergence,
    contact_detour,
    validate_capture_contract,
    validate_release,
    validate_report_integrity,
)
from params import DEFAULT_STATOR, PARAMS  # noqa: E402
from slot_route import PackingSupportGraph  # noqa: E402


class ElasticWireContactStudyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = json.loads(OUTPUT_JSON.read_text(encoding="utf-8"))
        cls.packing = json.loads(PACKING.read_text(encoding="utf-8"))
        cls.routes = json.loads(ROUTES.read_text(encoding="utf-8"))
        cls.graph = PackingSupportGraph.from_report(
            cls.packing, spec=DEFAULT_STATOR)

    def test_report_is_current_hash_bound_and_fails_closed(self):
        validate_report_integrity(self.report)
        self.assertEqual(self.report["schema"], SCHEMA)
        self.assertEqual(self.report["status"], "FAIL")
        self.assertEqual(
            self.report["decision"],
            "FIXED_FLYER_NOT_PROVEN_WITHOUT_ACTIVE_TOOLING",
        )
        with self.assertRaisesRegex(ValueError, "not PASS"):
            validate_release(self.report)

    def test_authoritative_raw_capture_covers_both_signs_and_every_state(self):
        raw = self.report["raw_motion_replay"]
        self.assertEqual(raw["pass_count"], 24)
        self.assertEqual(raw["state_count"], 2400)
        self.assertEqual(
            raw["motion_sign_counts"], {"negative": 12, "positive": 12})
        self.assertTrue(raw["both_motion_signs_covered"])
        self.assertEqual(raw["states_inside_winding_span"], 2400)
        self.assertEqual(len(raw["passes"]), 24)
        self.assertEqual(len(raw["states"]), 2400)
        self.assertEqual(
            {(row["pass_index"], row["state_index"])
             for row in raw["states"]},
            {(p, s) for p in range(24) for s in range(100)},
        )

    def test_raw_motion_is_not_misrepresented_as_the_packed_route_schedule(self):
        raw = self.report["raw_motion_replay"]
        self.assertEqual(raw["packed_schedule_binding_status"], "FAIL")
        self.assertLess(raw["states_matching_packed_route_schedule"], 2400)
        self.assertGreater(raw["maximum_m0_schedule_error_rad"], 4.0)
        self.assertGreater(raw["maximum_radial_schedule_error_mm"], 5.0)
        self.assertFalse(self.report["release_flags"][
            "raw_motion_matches_hash_bound_packing_schedule"])

    def test_zero_pitch_requires_lateral_repacking_not_elastic_compression(self):
        demand = self.report["raw_motion_replay"]["raw_repacking_demand"]
        self.assertEqual(
            demand[
                "minimum_nominal_intervals_below_wire_diameter_primary_phase_lane"],
            23,
        )
        self.assertEqual(
            demand[
                "maximum_nominal_intervals_below_wire_diameter_primary_phase_lane"],
            25,
        )
        self.assertAlmostEqual(demand["minimum_raw_radial_pitch_mm"], 0.0)
        self.assertGreaterEqual(demand["maximum_raw_radial_pitch_mm"], 0.7077)
        self.assertTrue(demand[
            "every_primary_phase_pass_has_nominal_zero_pitch"])
        self.assertAlmostEqual(
            demand["maximum_same_track_finished_diameter_compression_fraction"],
            1.0,
        )
        self.assertAlmostEqual(
            demand["maximum_minimum_orthogonal_repacking_mm"], 0.22352)
        self.assertTrue(demand[
            "local_orthogonal_relief_fits_certified_envelope"])
        self.assertFalse(demand["global_noncrossing_repacking_certificate"])
        self.assertFalse(self.report["release_flags"][
            "raw_low_pitch_repacking_has_global_noncrossing_proof"])

    def test_current_turn45_routes_keep_exact_contact_but_not_3mm_bends(self):
        result = self.report["elastic_contact_reanalysis"]
        self.assertEqual(result["rigid_failure_case_count"], 0)
        self.assertEqual(result["contact_case_count"], 2)
        self.assertEqual(result["current_rigid_geometry_pass_count"], 2)
        self.assertEqual(
            result["failed_route_contact_geometric_pass_count"], 0)
        self.assertEqual(result["contact_geometric_pass_count"], 2)
        self.assertEqual(result["elastic_curvature_pass_count"], 0)
        self.assertEqual(result["status"], "FAIL")
        for row in result["cases"]:
            self.assertEqual(row["stored_route_status"], "PASS")
            self.assertFalse(row["was_rigid_failure"])
            self.assertEqual(row["geometric_contact_status"], "PASS")
            self.assertEqual(row["status"], "FAIL")
            self.assertTrue(row["checks"][
                "exact_parent_contact_no_penetration"])
            self.assertTrue(row["checks"]["steel_core_clearance"])
            self.assertTrue(row["checks"]["nonparent_copper_clearance"])
            self.assertTrue(row["checks"]["simple_local_topology"])
            self.assertFalse(row["checks"][
                "goal_minimum_bend_radius_3mm"])
            self.assertAlmostEqual(
                row["analytic_local_bend_radius_mm"],
                self.graph.wire_diameter_mm, places=12)
            self.assertLess(
                row["analytic_local_bend_radius_mm"],
                PARAMS.min_bend_radius)

    def test_contact_construction_is_mirrored_and_numerically_convergent(self):
        turn45 = [row for row in self.routes["routes"]
                  if row["turn_index"] == 45]
        metadata = []
        for row in sorted(turn45, key=lambda value: value["half_turn_index"]):
            original = np.asarray(row["route"]["points_local_mm"], dtype=float)
            points, meta = contact_detour(row, self.graph)
            self.assertTrue(np.all(np.isfinite(points)))
            source_index = meta["end_plane_source_point_index"]
            expected_z = (DEFAULT_STATOR.stack / 2.0
                          if row["half_turn_index"] == 0
                          else -DEFAULT_STATOR.stack / 2.0)
            self.assertEqual(source_index, len(original) - 2)
            np.testing.assert_allclose(
                meta["source_local_mm"],
                row["planner_metadata"]["support_normal_approach"]
                ["approach_target_local_mm"],
                atol=1e-12, rtol=0.0)
            np.testing.assert_allclose(
                points[:source_index + 1], original[:source_index + 1],
                atol=0.0, rtol=0.0)
            np.testing.assert_allclose(
                points[-1], row["target_local_mm"], atol=1e-12, rtol=0.0)
            np.testing.assert_allclose(
                points[source_index:, 2], expected_z, atol=1e-12, rtol=0.0)
            self.assertAlmostEqual(meta["end_plane_z_mm"], expected_z)
            self.assertEqual(meta["replaced_end_plane_point_count"], 1)
            self.assertTrue(meta["local_contact_path_simple"])
            self.assertLessEqual(meta["tangent_orthogonality_error"], 1e-12)
            convergence = contact_arc_convergence(row, self.graph)
            self.assertEqual(
                [item["requested_step_deg"] for item in convergence],
                list(CONTACT_ROUTE_STEPS_DEG))
            errors = [item["arc_sag_error_bound_mm"]
                      for item in convergence]
            # Requests coarser than the whole short contact arc legitimately
            # use the same single interval.  Refinement must never worsen the
            # bound and must improve it once the requested step is smaller
            # than the arc span.
            self.assertTrue(all(a >= b for a, b in zip(errors, errors[1:])))
            self.assertLess(errors[-1], errors[0])
            self.assertLess(errors[-1], 2e-7)
            self.assertAlmostEqual(
                meta["analytic_local_bend_radius_mm"],
                self.graph.wire_diameter_mm, places=12)
            self.assertLess(
                meta["analytic_local_bend_radius_mm"],
                PARAMS.min_bend_radius)
            metadata.append(meta)
        self.assertAlmostEqual(
            metadata[0]["contact_arc_angle_deg"],
            metadata[1]["contact_arc_angle_deg"], places=12)
        self.assertAlmostEqual(
            metadata[0]["parent_contact_center_xy_mm"][0],
            metadata[1]["parent_contact_center_xy_mm"][0], places=12)
        self.assertAlmostEqual(
            metadata[0]["parent_contact_center_xy_mm"][1],
            -metadata[1]["parent_contact_center_xy_mm"][1], places=12)

    def test_contact_detour_rejects_a_terminal_off_the_end_plane(self):
        row = deepcopy(next(
            value for value in self.routes["routes"]
            if value["turn_index"] == 45
            and value["half_turn_index"] == 0
        ))
        row["route"]["points_local_mm"][-1][2] += 0.01
        row["target_local_mm"][2] += 0.01
        with self.assertRaisesRegex(
                ValueError, "target is not on the winding end plane"):
            contact_detour(row, self.graph)

    def test_axis_replay_matches_velocity_limit_and_target_clamp(self):
        axis = AxisTimeline(2.0)
        axis.command(0.0, 10.0)
        axis.command(2.0, -1.0)
        axis.finish(8.0)
        self.assertAlmostEqual(axis.position(1.0), 2.0)
        self.assertAlmostEqual(axis.position(2.0), 4.0)
        self.assertAlmostEqual(axis.position(4.0), 0.0)
        self.assertAlmostEqual(axis.position(5.0), -1.0)
        self.assertAlmostEqual(axis.position(7.5), -1.0)

    def test_capture_and_report_tampering_fail_closed(self):
        events = [json.loads(line) for line in CAPTURE.read_text(
            encoding="utf-8").splitlines()]
        broken = deepcopy(events)
        broken[0]["controller_mode"] = "contract"
        with self.assertRaisesRegex(ValueError, "raw capture contract failed"):
            validate_capture_contract(broken)

        tampered = deepcopy(self.report)
        tampered["raw_motion_replay"]["state_count"] -= 1
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            validate_report_integrity(tampered)


if __name__ == "__main__":
    unittest.main()
