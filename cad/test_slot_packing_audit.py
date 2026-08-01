"""Tests for the measured-input release slot-packing certificate."""

from __future__ import annotations

import hashlib
import json
import math
import unittest

import slot_packing_audit as packing


class SlotPackingAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = packing.analyze()

    def test_nominal_identity_and_fixed_topology(self):
        report = self.report
        selected = report["selected_schedule"]
        self.assertEqual(report["schema"], "slot-packing/v2")
        self.assertEqual(report["status"], "PASS")
        self.assertEqual(report["role"], "authoritative_release_default")
        self.assertEqual(report["config"]["wire_finished_diameter_mm"],
                         0.22352)
        self.assertEqual(report["config"]["liner_thickness_mm"], 0.127)
        self.assertEqual(selected["turns_per_tooth"], 50)
        self.assertEqual(selected["centers_per_slot"], 100)
        self.assertEqual(selected["layer_counts"], [23, 16, 9, 2])
        self.assertEqual(
            selected["deposition_order_lattice_indices"],
            list(packing.DEPOSITION_ORDER),
        )

    def test_every_step_is_tangent_and_every_post_seed_has_support(self):
        rows = self.report["selected_schedule"]["side_positive"]
        d = self.report["config"]["wire_finished_diameter_mm"]
        self.assertEqual(rows[0]["support_kind"], "slot_liner")
        self.assertEqual(rows[0]["parent_turn_indices"], [])
        for index, row in enumerate(rows[1:], 1):
            self.assertEqual(row["support_kind"], "deposited_wire")
            self.assertTrue(row["parent_turn_indices"])
            for parent, distance in zip(
                    row["parent_turn_indices"],
                    row["parent_center_distances_mm"]):
                self.assertLess(parent, index)
                self.assertAlmostEqual(distance, d, places=9)
        for distance in self.report["validation"][
                "all_consecutive_schedule_distances_mm"]:
            self.assertAlmostEqual(distance, d, places=9)

    def test_exact_nominal_core_pair_and_radial_constraints(self):
        validation = self.report["validation"]
        config = self.report["config"]
        self.assertGreaterEqual(
            validation["minimum_pair_center_distance_mm"],
            config["wire_finished_diameter_mm"] - 1e-9,
        )
        self.assertGreaterEqual(
            validation["minimum_center_core_distance_mm"],
            config["center_core_access_mm"] - 1e-9,
        )
        self.assertGreater(validation["radial_outer_margin_mm"], 0.5)
        self.assertTrue(validation["pair_clearance_ok"])
        self.assertTrue(validation["core_access_ok"])
        self.assertTrue(validation["radial_cap_ok"])

    def test_mouth_access_and_receiving_window_pass(self):
        selected = self.report["selected_schedule"]
        mouth = selected["sequential_mouth_access"]
        self.assertEqual(mouth["status"], "PASS")
        self.assertEqual(len(
            mouth["prefilled_neighbor_side_mouth_connected"]), 50)
        self.assertTrue(mouth["all_empty_neighbor_side_connected"])
        self.assertTrue(mouth["all_prefilled_neighbor_side_connected"])
        receiving = self.report["receiving_contract"]
        self.assertEqual(receiving["wire_finished_diameter_range_mm"],
                         [0.22, 0.235])
        self.assertEqual(receiving["liner_thickness_range_mm"],
                         [0.12, 0.14])
        self.assertEqual(receiving["topology_sensitivity_status"], "PASS")
        self.assertEqual(receiving["sensitivity_grid_shape"], [7, 7])
        self.assertEqual(len(receiving["sensitivity_grid_cases"]), 49)
        self.assertEqual(len(receiving["corner_cases"]), 4)
        self.assertTrue(all(
            case["status"] == "PASS"
            for case in receiving["sensitivity_grid_cases"]
        ))

    def test_measured_inputs_regenerate_centers_and_m0_targets(self):
        job = packing.PackingInput(0.231, 0.134)
        report = packing.analyze(job)
        self.assertEqual(report["status"], "PASS")
        self.assertEqual(report["role"],
                         "authoritative_measured_release_job")
        self.assertEqual(report["config"]["wire_finished_diameter_mm"],
                         job.wire_d_mm)
        self.assertEqual(report["config"]["liner_thickness_mm"],
                         job.liner_t_mm)
        distances = report["validation"][
            "all_consecutive_schedule_distances_mm"]
        self.assertTrue(all(
            abs(value - job.wire_d_mm) <= 1e-9 for value in distances
        ))
        self.assertNotEqual(
            report["selected_schedule"]["side_positive"][0][
                "m0_target_rad"],
            self.report["selected_schedule"]["side_positive"][0][
                "m0_target_rad"],
        )

    def test_explicit_nominal_dimensions_remain_a_measured_job(self):
        report = packing.analyze(packing.PackingInput(0.22352, 0.127))
        self.assertEqual(
            report["role"], "authoritative_measured_release_job")
        self.assertEqual(
            report["config"]["input_provenance"],
            "measured_receiving_input",
        )

    def test_receiving_values_outside_certificate_fail_closed(self):
        for job in (
            packing.PackingInput(0.219, 0.127),
            packing.PackingInput(0.236, 0.127),
            packing.PackingInput(0.22352, 0.119),
            packing.PackingInput(0.22352, 0.141),
        ):
            with self.subTest(job=job):
                with self.assertRaisesRegex(ValueError, "outside receiving"):
                    packing.analyze(job)

    def test_controller_radial_is_active_tooth_projection(self):
        rows = self.report["selected_schedule"]["side_positive"]
        for row in rows:
            active_radial = row["active_tooth_frame_uv_mm"][0]
            self.assertAlmostEqual(
                row["radial_parameter_mm"], active_radial, places=12)
            expected = packing.PARAMS.m0_rad_for_axis_z(
                active_radial + packing.TOOTH_CONTACT_Z)
            self.assertAlmostEqual(row["m0_target_rad"], expected, places=12)
            # It is intentionally not the slot-bisector coordinate.
            self.assertGreater(abs(
                row["radial_parameter_mm"]
                - row["slot_frame_uv_mm"][0]), 0.05)

    def test_report_hash_covers_complete_payload(self):
        payload = dict(self.report)
        expected = payload.pop("report_sha256")
        canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        self.assertEqual(
            hashlib.sha256(canonical.encode()).hexdigest(), expected)


if __name__ == "__main__":
    unittest.main(verbosity=2)
