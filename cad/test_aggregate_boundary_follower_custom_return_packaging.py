"""Focused tests for the isolated custom-return CAD packaging prototype."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_custom_return_packaging as packaging


class AggregateBoundaryFollowerCustomReturnPackagingTests(unittest.TestCase):

    def test_nominal_shaft_and_igus_envelope_close_dimensions(self):
        shaft = packaging.nominal_shaft()
        shaft_bounds = packaging._bounds(shaft)
        bushing = packaging.igus_bushing_envelope("center")
        contract = packaging.geometry_contract()["igus_bushing_and_pocket"]

        self.assertAlmostEqual(
            shaft_bounds["y"][1] - shaft_bounds["y"][0], 16.0
        )
        self.assertAlmostEqual(
            shaft_bounds["x"][1] - shaft_bounds["x"][0], 3.0
        )
        self.assertEqual(contract["catalog_number"], "WPFFM-0304-05")
        self.assertEqual(contract["body_OD_mm"], 4.5)
        self.assertEqual(contract["flange_OD_mm"], 7.5)
        self.assertEqual(contract["body_length_mm"], 5.0)
        self.assertEqual(contract["flange_thickness_mm"], 0.75)
        self.assertEqual(len(bushing.solids()), 1)
        self.assertFalse(contract["selected"])

    def test_torsion_pair_and_indexed_anchors_are_positive_single_solids(self):
        contract = packaging.geometry_contract()["torsion_pair"]
        self.assertEqual(contract["wire_diameter_mm"], 0.30)
        self.assertEqual(contract["mean_coil_diameter_mm"], 4.00)
        self.assertAlmostEqual(contract["active_coils_analytical"], 2.63671875)
        self.assertEqual(contract["indexed_holes_per_fixed_anchor"], 6)
        self.assertFalse(contract["indexed_anchor_selected"])
        for side in (-1, 1):
            with self.subTest(side=side):
                spring = packaging.torsion_spring_envelope(side)
                anchor = packaging.indexed_fixed_anchor(side)
                self.assertEqual(len(spring.solids()), 1)
                self.assertEqual(len(anchor.solids()), 1)
                self.assertGreater(float(spring.volume), 0.0)
                self.assertGreater(float(anchor.volume), 0.0)

    def test_same_state_custom_and_catalog_overlaps_are_zero_at_all_states(self):
        for state in packaging.TANGENTIAL_OFFSETS_MM:
            with self.subTest(state=state):
                custom = packaging.same_state_overlap_audit(state)
                all_parts = packaging.same_state_overlap_audit(
                    state, include_catalog_envelopes=True
                )
                self.assertEqual(custom["status"], "PASS", custom)
                self.assertEqual(custom["positive_overlap_count"], 0)
                self.assertEqual(all_parts["status"], "PASS", all_parts)
                self.assertEqual(all_parts["positive_overlap_count"], 0)

    def test_custom_bodies_are_single_solids(self):
        for state in packaging.TANGENTIAL_OFFSETS_MM:
            with self.subTest(state=state):
                for part in packaging.custom_bodies(state):
                    self.assertGreater(float(part.volume), 0.0, part.label)
                    self.assertEqual(len(part.solids()), 1, part.label)

    def test_cartridge_containment_and_package_bounds_close(self):
        for state in packaging.TANGENTIAL_OFFSETS_MM:
            with self.subTest(state=state):
                audit = packaging.bounds_audit(state)
                self.assertTrue(audit["containment_matches_declared_bounds"])
                self.assertTrue(audit["cartridge_inside_service_pocket"])
                self.assertTrue(audit["containment_inside_service_pocket"])
                self.assertTrue(audit["assembly_inside_global_review_bounds"])
        contract = packaging.geometry_contract()["radial_cartridge"]
        self.assertEqual(contract["catalog_number"], "9293K122")
        self.assertEqual(contract["coil_OD_mm"], 15.75)
        self.assertEqual(contract["coil_width_mm"], 6.35)
        self.assertEqual(contract["ratio_hole_targets"], [0.235, 0.27, 0.315])
        self.assertFalse(contract["fragment_containment_qualified"])

    def test_manifest_contract_is_complete_and_fail_closed(self):
        contract = packaging.geometry_contract()
        body = contract["body_contract"]
        self.assertEqual(
            contract["status"],
            "REVIEW_ONLY_CUSTOM_RETURN_PACKAGING_NO_AUTHORITY",
        )
        self.assertTrue(body["all_custom_center_bodies_single_solid"])
        self.assertTrue(body["all_same_state_custom_overlap_checks_pass"])
        self.assertTrue(body["all_same_state_all_part_overlap_checks_pass"])
        self.assertTrue(body["all_bounds_checks_pass"])
        authority = contract["authority"]
        self.assertTrue(authority["review_only"])
        self.assertTrue(
            all(value is False for key, value in authority.items()
                if key != "review_only")
        )
        self.assertFalse(
            contract["source_evidence"]["main_sources_edited_by_this_prototype"]
        )
        self.assertEqual(contract["inspection_refs"]["shaft_occurrence"], "#o1.13")
        self.assertTrue(
            contract["artifacts"]["step"].endswith(
                "aggregate_boundary_follower_custom_return_packaging.step"
            )
        )
        self.assertTrue(
            all(url.startswith("https://")
                for url in contract["catalog_sources"].values())
        )


if __name__ == "__main__":
    unittest.main()
