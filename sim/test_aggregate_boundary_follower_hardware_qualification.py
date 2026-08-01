"""Focused tests for the aggregate-follower hardware/load qualification."""

from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import aggregate_boundary_follower_hardware_qualification as qualification


class AggregateBoundaryFollowerHardwareQualificationTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.report = qualification.analyze()

    def test_report_is_fail_closed_on_physical_unknowns(self):
        report = self.report
        self.assertEqual(report["status"], "FAIL")
        self.assertEqual(report["qualification_status"], "FAIL_CLOSED")
        self.assertFalse(report["production_authorized"])
        self.assertFalse(report["assembly_integration_authorized"])
        self.assertTrue(all(report["definition_gates"].values()))
        self.assertFalse(any(report["release_gates"].values()))
        for required in (
            "exact maximum follower wrap angle across required routes",
            "follower and selected moving mass/inertia",
            "nose-to-datum force moment arm and key bearing distribution",
            "tangential centering spring and fatigue life",
            "tangential bushing/flexure fit, friction, and hysteresis",
            "0.2/0.5 mm production-wire enamel/contact/wear coupons",
            "300 rpm reversal endurance, return-to-dock, and spring life",
        ):
            self.assertIn(required, report["fail_closed_unknowns"])

    def test_unknown_wrap_requires_40N_proof_and_four_M4_at_10N(self):
        load = self.report["load_contract"]
        primary = self.report["primary_tower_mount"]
        self.assertFalse(load["exact_successor_wrap_known"])
        self.assertTrue(math.isclose(
            load["unbound_static_reaction_N"], 20.0, abs_tol=1.0e-12,
        ))
        self.assertTrue(math.isclose(
            load["required_proof_load_N"], 40.0, abs_tol=1.0e-12,
        ))
        self.assertEqual(primary["fastener_count"], 4)
        self.assertTrue(math.isclose(
            primary["equal_share_per_fastener_N"], 10.0,
            abs_tol=1.0e-12,
        ))
        expected_90_proof = 20.0 * math.sqrt(2.0)
        self.assertTrue(math.isclose(
            load["proof_load_if_all_exact_routes_are_le_90deg_N"],
            expected_90_proof,
            abs_tol=1.0e-12,
        ))
        self.assertIn("every exact required route", load["reduction_condition"])

    def test_primary_M4_stack_is_full_and_secondary_M3_is_non_proof(self):
        primary = self.report["primary_tower_mount"]
        primary_stack = primary["stack"]
        self.assertEqual(
            primary["hardware"]["screw"]["supplier_sku"], "90128A212"
        )
        self.assertEqual(
            primary["hardware"]["insert"]["supplier_sku"], "94459A150"
        )
        self.assertTrue(math.isclose(
            primary_stack["screw_penetration_mm"], 5.1, abs_tol=1.0e-12,
        ))
        self.assertTrue(primary_stack["full_insert_engagement_analytical"])
        self.assertFalse(primary_stack["pilot_bottoming_analytical"])

        secondary = self.report["secondary_axial_cassette_mount"]
        current = secondary["current_stack"]
        self.assertEqual(secondary["classification"],
                         "NON_PROOF_LOCATOR_AND_CLAMP_ONLY")
        self.assertFalse(secondary["proof_load_path_authorized"])
        self.assertTrue(math.isclose(
            current["calculated_screw_insert_overlap_mm"], 3.45,
            abs_tol=1.0e-12,
        ))
        self.assertTrue(math.isclose(
            current["full_engagement_shortfall_mm"], 0.85,
            abs_tol=1.0e-12,
        ))
        self.assertTrue(math.isclose(
            secondary["equal_share_if_misapplied_per_fastener_N"], 20.0,
            abs_tol=1.0e-12,
        ))
        self.assertTrue(
            secondary["equal_share_exceeds_existing_analytical_screen"]
        )

    def test_LEM050AB01_ratio_and_contact_cap(self):
        spring = self.report["radial_spring_contract"]
        usable = spring["at_usable_travel"]
        maximum_hard = spring["at_hard_travel_bounds"][-1]
        self.assertEqual(spring["spring"]["sku"], "LEM050AB 01")
        self.assertTrue(math.isclose(
            spring["motion_ratio_spring_extension_per_follower_travel"],
            0.29, abs_tol=1.0e-12,
        ))
        self.assertTrue(math.isclose(
            spring["initial_follower_output_force_N"], 0.5133,
            abs_tol=1.0e-12,
        ))
        self.assertTrue(math.isclose(
            usable["follower_output_force_N"], 1.69911,
            abs_tol=1.0e-12,
        ))
        self.assertTrue(math.isclose(
            usable["spring_length_mm"], 11.24, abs_tol=1.0e-12,
        ))
        self.assertLessEqual(
            maximum_hard["follower_output_force_N"],
            spring["contact_force_hard_cap_N"],
        )
        self.assertLessEqual(
            maximum_hard["spring_length_mm"],
            spring["spring"]["maximum_length_mm"],
        )
        rejected = spring["direct_one_to_one_6mm_extension_rejected"]
        self.assertTrue(rejected["length_exceeds_catalog_max"])
        self.assertTrue(rejected["load_exceeds_catalog_max"])
        self.assertFalse(spring["topology_released"])

    def test_radial_and_tangential_travel_contract(self):
        radial = self.report["radial_spring_contract"]
        tangential = self.report["tangential_contract"]
        self.assertEqual(radial["usable_follower_travel_mm"], 6.0)
        self.assertEqual(radial["hard_travel_range_mm"], [6.4, 6.6])
        self.assertEqual(tangential["usable_half_travel_mm"], 0.5)
        self.assertEqual(tangential["usable_total_travel_mm"], 1.0)
        self.assertEqual(tangential["hard_stop_half_travel_mm"], 0.6)
        self.assertEqual(tangential["hard_stop_total_travel_mm"], 1.2)
        self.assertFalse(tangential["centering_spring_selected"])
        self.assertFalse(tangential["bushing_or_flexure_selected"])

    def test_integrity_rejects_tampering(self):
        current = deepcopy(self.report)
        qualification.validate_report_integrity(current)
        current["load_contract"]["required_proof_load_N"] = 28.3
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            qualification.validate_report_integrity(current)

    def test_written_report_is_current(self):
        generated = qualification.write_outputs(self.report)
        checked = json.loads(qualification.OUTPUT_JSON.read_text(
            encoding="utf-8"
        ))
        self.assertEqual(generated["report_sha256"], checked["report_sha256"])
        qualification.validate_report_integrity(checked)
        markdown = qualification.OUTPUT_MD.read_text(encoding="utf-8")
        self.assertIn("Required proof: 40.0 N", markdown)
        self.assertIn("NON_PROOF_LOCATOR_AND_CLAMP_ONLY", markdown)
        self.assertIn("Production and assembly integration remain unauthorized", markdown)


if __name__ == "__main__":
    unittest.main(verbosity=2)
