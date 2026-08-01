"""Focused tests for the isolated successor-follower prototype."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_successor_prototype as prototype


class SuccessorFollowerPrototypeTests(unittest.TestCase):

    def test_fail_closed_placement_contract_and_exact_identity_bounds(self):
        report = prototype.placement_report()
        self.assertEqual(
            report["report_sha256"],
            prototype.EXPECTED_REPORT_INTERNAL_SHA256,
        )
        contract = prototype.geometry_contract()
        for identity in range(4):
            source = report["successor_trade"]["per_identity"][str(identity)]
            copied = contract["identities"][str(identity)]
            self.assertEqual(
                copied["exact_target_center_bounds_local_mm"],
                source["exact_target_center_bounds_local_mm"],
            )
            self.assertEqual(
                copied["exact_target_datum_local_mm"],
                source["exact_target_datum_local_mm"],
            )
            bounds = source["exact_target_center_bounds_local_mm"]
            displayed_min = prototype.display_point_from_active_local(
                identity, bounds["min_mm"])
            displayed_max = prototype.display_point_from_active_local(
                identity, bounds["max_mm"])
            for axis in range(3):
                self.assertAlmostEqual(
                    displayed_max[axis] - displayed_min[axis],
                    bounds["span_mm"][axis],
                    places=12,
                )

    def test_modeled_travel_and_two_axis_ranges_cover_exact_minimum(self):
        stage = prototype.geometry_contract()["stage"]
        self.assertTrue(stage["all_modeled_travel_meets_required"])
        self.assertGreaterEqual(stage["modeled_yaw_half_range_deg"],
                                stage["required_yaw_half_range_deg"])
        self.assertGreaterEqual(stage["modeled_elevation_half_range_deg"],
                                stage["required_elevation_half_range_deg"])

    def test_each_module_is_positive_volume_and_mechanically_split(self):
        for identity in range(4):
            with self.subTest(identity=identity):
                parts = prototype.module_parts(identity)
                self.assertEqual(len(parts), 21)
                self.assertTrue(all(float(part.volume) > 0.0 for part in parts))
                self.assertTrue(all(len(part.solids()) == 1 for part in parts))
                labels = {part.label for part in parts}
                self.assertIn(
                    f"id{identity}_polished_PEEK_C1_guide_cartridge", labels)
                self.assertIn(
                    f"id{identity}_separate_aggregate_normal_preload_leaf",
                    labels,
                )
                self.assertIn(
                    f"id{identity}_separate_polished_PEEK_preload_shoe",
                    labels,
                )

    def test_C1_guide_and_R3_plus_2mm_floor_relief(self):
        guide = prototype.c1_guide_local()
        self.assertEqual(len(guide.solids()), 1)
        self.assertGreater(float(guide.volume), 0.0)
        relief = prototype.geometry_contract()["carrier_floor_relief"]
        self.assertEqual(relief["conservative_envelope_radius_mm"], 3.0)
        self.assertEqual(relief["relief_radius_mm"], 5.0)
        self.assertEqual(relief["radial_clearance_mm"], 2.0)
        self.assertTrue(all(
            len(prototype.floor_relief_coupon(identity).solids()) == 1
            for identity in range(4)
        ))

    def test_export_is_positive_and_authority_stays_false(self):
        result = prototype.gen_step()
        self.assertEqual(len(result.solids()), 84)
        self.assertTrue(all(float(solid.volume) > 0.0
                            for solid in result.solids()))
        authority = prototype.geometry_contract()["authority"]
        self.assertTrue(authority.pop("isolated_review_only"))
        self.assertFalse(any(authority.values()))


if __name__ == "__main__":
    unittest.main()
