"""Regression gates for the isolated permanent-cap offset-spoke review."""

from __future__ import annotations

import math
import unittest

import permanent_cap_offset_spoke_review as review


class PermanentCapOffsetSpokeReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = review.analyze()

    def test_authoritative_contracts_are_bound(self) -> None:
        contracts = self.report["source_contracts"]
        self.assertEqual(contracts["aggregate_report"]["status"], "PASS")
        self.assertEqual(
            contracts["aggregate_report"]["lane_id"],
            "cap-r3-sector-lane-v1",
        )
        self.assertEqual(len(contracts["aggregate_report"]["sha256"]), 64)
        self.assertEqual(len(contracts["offset_report"]["sha256"]), 64)

    def test_printed_arm_is_one_solid_with_exact_spoke_section(self) -> None:
        arm = review.offset_spoke_arm()
        self.assertEqual(len(list(arm.solids())), 1)
        components = review.offset_spoke_arm_components()
        bbox = components["spoke"].bounding_box()
        self.assertAlmostEqual(float(bbox.size.X), 14.0, places=7)
        self.assertAlmostEqual(float(bbox.size.Z), 8.0, places=7)
        self.assertAlmostEqual(float(bbox.min.Z), -38.12, places=7)
        self.assertAlmostEqual(float(bbox.max.Z), -30.12, places=7)

    def test_exact_controlling_pose_clearances(self) -> None:
        clear = self.report["controlling_clearances_mm"][
            "exact_OCC_at_M1_M2_zero"
        ]
        for name, distance in clear.items():
            with self.subTest(name=name):
                self.assertGreaterEqual(distance, review.REVIEW_CLEARANCE_MM)

    def test_continuous_360_certificate(self) -> None:
        certificate = self.report["controlling_clearances_mm"][
            "continuous_rotation_certificate"
        ]
        self.assertEqual(certificate["integer_angle_count"], 360)
        self.assertEqual(len(certificate["per_degree"]), 360)
        self.assertGreaterEqual(
            certificate["minimum_cap_arm_lower_bound_mm"], 2.2,
        )
        self.assertGreaterEqual(
            certificate["minimum_block_arm_lower_bound_mm"], 2.2,
        )

    def test_shaft_and_static_relocations_are_exact(self) -> None:
        dims = self.report["selected_dimensions_mm"]
        self.assertEqual(dims["module_rear_shift"], 10.0)
        self.assertEqual(dims["entry_guide_rear_shift"], 2.0)
        self.assertEqual(dims["frame_window_rear_shift"], 2.5)
        shaft = self.report["geometry"]["extended_hollow_shaft"]
        self.assertEqual(shaft["outer_diameter_mm"], 12.0)
        self.assertEqual(shaft["inner_diameter_mm"], 9.0)

    def test_mass_properties_and_balance_capacity_are_finite(self) -> None:
        props = self.report["geometry"]["arm"][
            "mass_properties_100pct_PETG"
        ]
        self.assertGreater(props["volume_mm3"], 0.0)
        self.assertGreater(props["mass_g"], 0.0)
        self.assertTrue(math.isfinite(props["izz_about_M2_axis_g_mm2"]))
        balance = self.report["provisional_balance_envelope"]
        self.assertGreaterEqual(
            balance["combined_radial_moment_g_mm"],
            balance["study_target_radial_moment_g_mm"],
        )
        self.assertFalse(balance["retention_hardware_complete"])
        self.assertFalse(balance["two_plane_balance_complete"])

    def test_review_pass_does_not_authorize_production(self) -> None:
        self.assertEqual(self.report["status"], "PASS_REVIEW_ONLY")
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["assembly_integration_authorized"])
        gates = self.report["release_gates"]
        self.assertFalse(gates["cap_collision_support_envelope_is_production_cap"])
        self.assertFalse(gates["exact_two_plane_balance"])
        self.assertFalse(gates["full_raw_assembly_collision_regenerated"])


if __name__ == "__main__":
    unittest.main()
