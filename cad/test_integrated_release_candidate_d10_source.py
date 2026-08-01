"""Source-only regression gates for the Rev-D stock-D10 integration.

These tests intentionally avoid ``analyze()`` and all checked-in aggregate
reports.  They are the focused pre-regeneration gate for the candidate source.
"""

from __future__ import annotations

import math
import unittest

import integrated_release_candidate as rc


class IntegratedD10SourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.drive = rc.successor_drive_parts()
        cls.rotating = rc.integrated_base_rotating_parts()
        cls.shaft = cls.rotating["shaft"]
        cls.pulley = cls.rotating["flyer_pulley"]
        cls.shaft_brep = rc.released_shaft_brep_audit(cls.shaft)
        cls.balance = rc.integrated_balance_solution()
        cls.static_groups = rc.main_static_groups()
        cls.entry = rc._find(cls.static_groups["shifted_entry"], "entry_bracket")
        cls.eyelet = rc._find(cls.static_groups["shifted_entry"], "entry_eyelet")
        cls.block = rc._find(cls.static_groups["shifted_support"], "flyer_block")
        cls.rear_bearing = rc._find(
            cls.static_groups["shifted_support"], "flyer_6001_rear"
        )
        cls.static_wire = rc.configured_static_supply_wire()
        cls.flyer_wire = rc.flyer_successor.guide_wire_envelope(
            float(rc.DEFAULT_STATOR.wire_d),
            "source_test_flyer_guide_wire",
        )
        cls.arm = cls.rotating["retained_arm"]

    def test_exact_revision_and_selected_relocations(self) -> None:
        self.assertEqual(
            rc.COLLISION_GEOMETRY_REVISION,
            "active-sector-r39p2__physical-bell-root-sleeve-six-slug__"
            "L79-stock-D10-P30__short-cap-leadins__"
            "rev6-front-plane-outboard-coil-bypass-yoke",
        )
        self.assertEqual(rc.ENTRY_REAR_SHIFT_MM, 4.25)
        self.assertEqual(rc.ENTRY_ADDITIONAL_REAR_SHIFT_MM, 0.75)

    def test_official_D10_occurrence_and_Rev_D_shaft_match_full_seat(self) -> None:
        self.assertEqual(
            rc.nbk_p30_d10.source_sha256(),
            rc.nbk_p30_d10.SOURCE_STEP_SHA256,
        )
        self.assertEqual(self.pulley.label, rc.nbk_p30_d10.STOCK_LABEL)
        self.assertEqual(len(self.pulley.solids()), 1)
        self.assertTrue(self.pulley.is_valid)
        self.assertAlmostEqual(self.pulley.bounding_box().min.Z, -110.75, places=5)
        self.assertAlmostEqual(self.pulley.bounding_box().max.Z, -92.25, places=5)
        self.assertEqual(len(self.shaft.solids()), 1)
        self.assertTrue(self.shaft.is_valid)
        self.assertEqual(
            self.shaft_brep["bbox_mm"]["size_mm"], [12.0, 12.0, 79.0]
        )
        self.assertEqual(self.shaft_brep["rear_datum_z_mm"], -110.75)
        self.assertEqual(self.shaft_brep["front_datum_z_mm"], -31.75)
        self.assertEqual(self.shaft_brep["D10_seat_length_mm"], 18.5)
        self.assertEqual(self.shaft_brep["neck_outer_diameter_mm"], 10.0)
        self.assertEqual(self.shaft_brep["neck_inner_diameter_mm"], 6.0)
        self.assertEqual(
            self.shaft_brep["neck_inner_diameter_limits_mm"], [6.0, 6.03]
        )
        self.assertAlmostEqual(
            self.shaft_brep["minimum_neck_radial_wall_at_limits_mm"], 1.9805
        )
        self.assertEqual(
            self.shaft_brep["cylindrical_surface_radii_mm"],
            [3.0, 4.5, 5.0, 6.0],
        )
        self.assertEqual(
            [
                (row["normal"], row["station_from_rear_datum_mm"])
                for row in self.shaft_brep["indexed_flats"]
            ],
            [("minus_y", 64.75), ("plus_x", 64.75)],
        )
        self.assertLessEqual(rc._distance(self.shaft, self.pulley), 1.0e-5)
        self.assertEqual(rc._overlap(self.shaft, self.pulley), 0.0)

    def test_rev_D_front_clears_root_sleeve_PEEK_guide_and_wire(self) -> None:
        interface = rc.released_shaft_front_interface_audit(
            self.shaft,
            self.rotating["flyer_PEEK_guide"],
            self.flyer_wire,
        )
        self.assertEqual(interface["shaft_front_z_mm"], -31.75)
        self.assertEqual(interface["root_sleeve_front_z_mm"], -31.5)
        self.assertGreaterEqual(
            interface["shaft_front_setback_from_root_sleeve_front_mm"],
            0.25,
        )
        self.assertAlmostEqual(
            interface["shaft_to_PEEK_guide_outer_distance_mm"], 0.33,
            places=9,
        )
        self.assertEqual(
            interface["shaft_vs_PEEK_guide_outer_overlap_mm3"], 0.0
        )
        self.assertAlmostEqual(
            interface["shaft_to_flyer_wire_distance_mm"], 1.51824,
            places=9,
        )
        self.assertEqual(interface["shaft_vs_flyer_wire_overlap_mm3"], 0.0)
        self.assertTrue(interface["source_gate"])

    def test_entry_clearance_and_fixed_anchor_keeper_survive_4p25_shift(self) -> None:
        self.assertEqual(len(self.entry.solids()), 1)
        self.assertTrue(self.entry.is_valid)
        self.assertAlmostEqual(rc._distance(self.shaft, self.entry), 2.5, places=5)
        self.assertAlmostEqual(rc._distance(self.pulley, self.entry), 2.5, places=5)
        self.assertAlmostEqual(rc._distance(self.pulley, self.block), 2.25, places=5)
        self.assertAlmostEqual(
            rc._distance(self.pulley, self.rear_bearing), 7.25, places=5
        )
        self.assertGreaterEqual(rc._distance(self.shaft, self.eyelet), 2.2)
        attachment = rc.entry_module_attachment_audit(self.static_groups)
        self.assertEqual(attachment["additional_shift_beyond_prior_candidate_mm"], 0.75)
        self.assertTrue(attachment["entry_bracket_one_valid_solid"])
        self.assertTrue(attachment["all_mounting_hardware_contacts_bracket_and_post"])
        self.assertLessEqual(
            attachment["preserved_dancer_fixed_screw_to_bracket_distance_mm"],
            1.0e-5,
        )

    def test_wire_is_continuous_through_shaft_to_flyer_owned_guide(self) -> None:
        self.assertAlmostEqual(
            rc.flyer_successor.GUIDE_ROOT_AXIAL_START_Z_MM, -42.0
        )
        self.assertLessEqual(
            rc._distance(self.static_wire, self.flyer_wire), rc.CONTACT_TOL_MM
        )
        self.assertLessEqual(
            rc._overlap(self.static_wire, self.flyer_wire), rc.BOOLEAN_TOL_MM3
        )
        self.assertEqual(rc._overlap(self.static_wire, self.shaft), 0.0)
        self.assertEqual(rc._overlap(self.static_wire, self.arm), 0.0)
        self.assertGreaterEqual(
            rc._distance(self.static_wire, self.shaft) + 1.0e-9,
            rc.flyer_shaft_d10.NECK_ID_MM / 2.0 - rc.wire_vis.R_VIS,
        )
        self.assertGreater(
            self.static_wire.bounding_box().max.Z,
            rc.flyer_successor.GUIDE_ROOT_AXIAL_START_Z_MM,
        )

    def test_official_mass_and_J_drive_the_six_trim_resolve(self) -> None:
        pulley_row = next(
            row for row in rc._integrated_base_mass_rows()
            if row["name"] == "flyer_pulley"
        )
        self.assertEqual(pulley_row["mass_g"], 28.0)
        self.assertEqual(pulley_row["izz_about_M2_axis_g_mm2"], 3000.0)
        self.assertTrue(pulley_row["supplied_clamp_bolt_included"])
        self.assertEqual(
            self.balance["rear_slug_lengths_mm"],
            [
                1.5375341624852528,
                0.6,
                0.6308324065833766,
                1.5680396641162582,
            ],
        )
        self.assertAlmostEqual(
            self.balance["front_trim_common_thickness_mm"],
            1.5021684970204356,
            places=12,
        )
        mass = self.balance["mass_properties"]
        self.assertLess(mass["static_imbalance_g_mm"], 1.0e-6)
        self.assertLess(mass["couple_imbalance_g_mm2"], 1.0e-6)
        self.assertTrue(math.isclose(
            mass["izz_about_M2_axis_kg_m2"],
            6.568916558612206e-5,
            rel_tol=0.0,
            abs_tol=1.0e-15,
        ))

    def test_deeper_front_trim_pilots_preserve_full_insert_and_reserve(self) -> None:
        self.assertEqual(
            rc.flyer_successor.FRONT_TRIM_PILOT_BOTTOM_Z_MM, -19.75
        )
        attachment = rc.integrated_six_stack_attachment_audit()
        self.assertTrue(
            attachment["both_front_stacks_thread_into_blind_spoke_inserts"]
        )
        for row in attachment["front_M2_blind_spoke_stacks"]:
            self.assertAlmostEqual(row["full_insert_engagement_mm"], 4.0)
            self.assertGreaterEqual(
                row["screw_tip_clearance_behind_insert_mm"], 0.5
            )
            self.assertAlmostEqual(
                row["screw_tip_clearance_behind_insert_mm"],
                0.6021684970204362,
                places=12,
            )
            self.assertAlmostEqual(
                row["blind_printed_material_behind_pilot_mm"], 18.37,
                places=9,
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
