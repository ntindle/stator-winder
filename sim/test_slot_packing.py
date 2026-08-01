"""Regression tests for the controller-facing release packing plan."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import unittest

import slot_packing
import slot_packing_audit


ROOT = Path(__file__).resolve().parents[1]
PLAN_PATH = ROOT / "out" / "reports" / "slot_winding_plan.json"


class ReleasePackingPlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan = slot_packing.build_plan()

    def test_plan_is_exact_purchase_matched_default(self):
        plan = self.plan
        self.assertEqual(plan["schema"], "slot-winding-plan/v1")
        self.assertEqual(plan["selected_case"]["status"], "PASS")
        self.assertEqual(plan["job"]["wire_finished_d_mm"], 0.22352)
        self.assertEqual(
            plan["job"]["supplier_nominal_wire_finished_d_mm"], 0.22352)
        self.assertEqual(plan["job"]["liner_max_thickness_mm"], 0.127)
        self.assertEqual(plan["job"]["turns_per_tooth"], 50)
        self.assertEqual(plan["packing_report"]["schema"], "slot-packing/v2")

    def test_every_schedule_transition_is_one_modeled_wire_diameter(self):
        points = self.plan["placements"]
        self.assertEqual(len(points), 50)
        distances = [math.hypot(
            right["active_tooth_radial_mm"]
            - left["active_tooth_radial_mm"],
            right["active_tooth_tangential_mm"]
            - left["active_tooth_tangential_mm"],
        ) for left, right in zip(points, points[1:])]
        self.assertEqual(len(distances), 49)
        for value in distances:
            self.assertAlmostEqual(value, 0.22352, places=9)
        self.assertEqual(
            [sum(point["layer"] == layer for point in points)
             for layer in range(4)],
            [23, 16, 9, 2],
        )

    def test_every_later_layer_names_tangent_prior_support(self):
        points = self.plan["placements"]
        support = self.plan["selected_case"]["transition_proof"][
            "first_side_insertion"
        ]
        for index, row in enumerate(support):
            parents = row["support_predecessor_indices"]
            if index == 0:
                self.assertEqual(row["support"], "slot_liner")
                self.assertEqual(parents, [])
            else:
                self.assertEqual(row["support"], "deposited_wire")
                self.assertTrue(parents)
                for parent in parents:
                    self.assertLess(parent, index)
                    self.assertAlmostEqual(math.hypot(
                        points[index]["active_tooth_radial_mm"]
                        - points[parent]["active_tooth_radial_mm"],
                        points[index]["active_tooth_tangential_mm"]
                        - points[parent]["active_tooth_tangential_mm"],
                    ), 0.22352, places=9)

    def test_half_turns_use_active_tooth_radial_and_explicit_m0(self):
        halves = self.plan["half_turn_centers"]
        points = self.plan["placements"]
        self.assertEqual(len(halves), 100)
        for index, half in enumerate(halves):
            placement = points[index // 2]
            self.assertEqual(half["half_turn_index"], index)
            self.assertEqual(half["placement_index"], index // 2)
            self.assertAlmostEqual(
                half["radial_mm"],
                placement["active_tooth_radial_mm"], places=12)
            self.assertAlmostEqual(
                half["m0_target_rad"], placement["m0_target_rad"], places=12)
            # Slot-bisector radial is deliberately a different presentation
            # coordinate; using it for M0 would reproduce the prior bug.
            self.assertGreater(
                abs(placement["radial_mm"]
                    - placement["active_tooth_radial_mm"]), 1e-3)

    def test_exact_pair_and_core_proof_is_retained(self):
        proof = self.plan["selected_case"]["final_slot_proof"]
        self.assertEqual(proof["status"], "PASS")
        self.assertEqual(proof["center_count"], 100)
        self.assertEqual(proof["pair_count_checked"], 4950)
        self.assertGreaterEqual(
            proof["minimum_pairwise_center_distance_mm"], 0.22352 - 1e-9)
        self.assertGreaterEqual(
            proof["minimum_center_core_clearance_mm"], 0.23876 - 1e-9)

    def test_measured_job_propagates_into_plan_geometry(self):
        job = slot_packing_audit.PackingInput(0.231, 0.134)
        measured = slot_packing.build_plan(job)
        self.assertEqual(measured["job"]["wire_finished_d_mm"], 0.231)
        self.assertEqual(
            measured["job"]["liner_measured_thickness_mm"], 0.134)
        self.assertEqual(
            measured["job"]["liner_receiving_max_thickness_mm"], 0.140)
        for distance in measured["selected_case"]["transition_proof"][
                "all_consecutive_center_distances_mm"]:
            self.assertAlmostEqual(distance, 0.231, places=9)
        self.assertNotEqual(
            measured["placements"][0]["m0_target_rad"],
            self.plan["placements"][0]["m0_target_rad"],
        )

    def test_proof_hash_and_generated_artifact_are_current(self):
        payload = dict(self.plan)
        expected = payload.pop("proof_sha256")
        canonical = json.dumps(
            payload, sort_keys=True, separators=(",", ":"), allow_nan=False,
        ).encode("utf-8")
        self.assertEqual(hashlib.sha256(canonical).hexdigest(), expected)
        checked = json.loads(PLAN_PATH.read_text(encoding="utf-8"))
        self.assertEqual(checked, self.plan)


if __name__ == "__main__":
    unittest.main(verbosity=2)
