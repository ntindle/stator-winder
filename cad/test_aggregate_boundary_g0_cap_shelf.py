"""Regression tests for the isolated robust g=0 cap-shelf prototype."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_g0_cap_shelf as shelf


class AggregateBoundaryG0CapShelfCadTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.validation = shelf.validation_report()
        cls.contract = shelf.geometry_contract()

    def test_shelf_mouth_lane_and_endpoint_contract(self):
        self.assertEqual(
            self.contract["shelf_dimensions_mm"], [1.5, 0.75, 0.3])
        self.assertEqual(
            self.contract["mouth_dimensions_mm"], [2.4, 1.0, 0.9])
        self.assertAlmostEqual(
            self.contract["mouth_center_y_mm"], 0.9436710365709817)
        self.assertGreaterEqual(self.contract["lane_clear_width_mm"], 0.65)
        self.assertEqual(self.contract["lane_max_wire_radius_mm"], 0.25)
        self.assertEqual(self.contract["shelf_count_front_rear"], 48)
        self.assertEqual(
            shelf.endpoint_for_diameter(0.2, 1),
            (12.687039228440508, 0.8686710365709818, 13.961655295981739),
        )
        self.assertEqual(
            shelf.endpoint_for_diameter(0.5, -1),
            (12.687039228440508, 1.0186710365709817, -13.961655295981739),
        )
        with self.assertRaisesRegex(ValueError, "0.2 or 0.5"):
            shelf.endpoint_for_diameter(0.3)

    def test_front_and_rear_caps_are_positive_single_solids(self):
        self.assertEqual(self.validation["status"], "PASS_GEOMETRY_ONLY")
        self.assertEqual(
            self.validation["cap_solid_counts"], {"front": 1, "rear": 1})
        front = shelf.finished_cap(1)
        rear = shelf.finished_cap(-1)
        self.assertGreater(front.volume, 0.0)
        self.assertGreater(rear.volume, 0.0)
        self.assertEqual(len(front.solids()), 1)
        self.assertEqual(len(rear.solids()), 1)
        self.assertTrue(math.isclose(
            front.volume, rear.volume, rel_tol=0.0, abs_tol=0.05,
        ))

    def test_both_diameters_are_tangent_and_wire_gauge_clear(self):
        rows = self.validation["diameter_cases"]
        self.assertEqual(len(rows), 4)
        self.assertEqual(
            {(row["axial_end"], row["wire_diameter_mm"]) for row in rows},
            {("front", 0.2), ("front", 0.5),
             ("rear", 0.2), ("rear", 0.5)},
        )
        for row in rows:
            self.assertTrue(row["distance_equals_wire_radius"])
            self.assertTrue(row["wire_zero_positive_overlap"])
            self.assertTrue(row["gauge_zero_positive_overlap"])
            self.assertAlmostEqual(
                row["endpoint_to_cap_distance_mm"],
                row["expected_wire_radius_mm"], places=8,
            )
            self.assertEqual(row["cap_to_wire_positive_overlap_mm3"], 0.0)
            self.assertEqual(
                row["cap_to_R0p36_gauge_positive_overlap_mm3"], 0.0)

    def test_symmetry_mass_facts_and_authority_fail_closed(self):
        self.assertEqual(self.validation["physical_shelf_count"], 48)
        self.assertTrue(self.validation["24fold_front_rear_symmetry_authored"])
        self.assertTrue(self.validation["first_moment_expected_to_cancel"])
        self.assertFalse(
            self.validation["exact_mass_inertia_release_update_complete"])
        rows = self.validation["volume_mass_balance"]
        self.assertEqual(len(rows), 2)
        self.assertAlmostEqual(rows[0]["prototype_mass_g"],
                               rows[1]["prototype_mass_g"], places=4)
        self.assertLess(abs(rows[0]["center_of_mass_mm"][0]), 1.0e-4)
        self.assertLess(abs(rows[0]["center_of_mass_mm"][1]), 1.0e-4)
        authority = self.contract["authority"]
        self.assertTrue(authority["isolated_review_only"])
        for key, value in authority.items():
            if key != "isolated_review_only":
                self.assertFalse(value, key)

    def test_labeled_compound_and_hash_bound_manifest_are_current(self):
        compound = shelf.gen_step()
        self.assertEqual(
            compound.label, "isolated_robust_g0_cap_shelf_review_only")
        self.assertEqual(len(compound.children), 10)
        self.assertTrue(shelf.STEP_OUT.is_file())
        self.assertTrue(shelf.MANIFEST_OUT.is_file())
        manifest = json.loads(shelf.MANIFEST_OUT.read_text(encoding="utf-8"))
        self.assertEqual(
            manifest["manifest_sha256"], shelf._canonical_hash(manifest))
        self.assertEqual(
            manifest["artifact_sha256"], shelf._sha256(shelf.STEP_OUT))
        self.assertEqual(
            manifest["artifact_byte_count"], shelf.STEP_OUT.stat().st_size)
        self.assertEqual(manifest["validation"]["status"],
                         "PASS_GEOMETRY_ONLY")
        self.assertFalse(manifest["selected_cap_or_release_modified"])
        for relative, expected in manifest["source_hashes"].items():
            self.assertEqual(shelf._sha256(shelf.ROOT / relative), expected)


if __name__ == "__main__":
    unittest.main(verbosity=2)
