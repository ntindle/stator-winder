from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import permanent_cap_production_review as cap


class PermanentCapProductionReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = cap.analyze()

    def test_exact_authority_and_material_are_bound(self) -> None:
        source = self.report["source_contracts"]
        self.assertEqual(source["aggregate"]["status"], "PASS")
        self.assertEqual(source["aggregate"]["lane_id"], cap.LANE_ID)
        self.assertEqual(
            source["material_dfm"]["status"],
            "CONDITIONAL_PEEK_ROUTE_SELECTED_NOT_PRODUCTION",
        )
        self.assertEqual(source["offset_spoke_review"]["status"],
                         "PASS_REVIEW_ONLY")

    def test_caps_are_physical_one_solid_parts_not_fan_envelopes(self) -> None:
        geometry = self.report["geometry"]
        self.assertEqual(geometry["front_cap"]["solid_count"], 1)
        self.assertEqual(geometry["rear_cap"]["solid_count"], 1)
        self.assertFalse(geometry["fan_like_collision_envelopes_used"])
        self.assertGreater(geometry["front_cap"]["volume_mm3"], 0.0)
        self.assertGreater(geometry["rear_cap"]["volume_mm3"], 0.0)

    def test_all_sectors_and_connectors_are_present(self) -> None:
        geometry = self.report["geometry"]
        self.assertEqual(geometry["sectors_per_cap"], 24)
        self.assertEqual(geometry["continuous_channel_count"], 48)
        self.assertEqual(geometry["connector_mouth_count"], 96)

    def test_contact_contract_is_exact(self) -> None:
        contact = self.report["geometry"]["wire_contact"]
        self.assertAlmostEqual(
            contact["primitive_radius_mm"]
            - contact["lane_half_width_mm"]
            - contact["finished_wire_radius_mm"],
            contact["minimum_manufactured_contact_radius_mm"],
            places=12,
        )
        self.assertGreaterEqual(
            contact["minimum_manufactured_contact_radius_mm"], 2.88824,
        )
        self.assertGreaterEqual(
            contact["minimum_clear_polished_groove_width_mm"], 0.47752,
        )
        self.assertGreaterEqual(contact["open_access_mouth_mm"], 0.5)
        self.assertAlmostEqual(
            contact["finished_wire_axial_envelope_mm"],
            34.65478063661919,
            places=9,
        )

    def test_retention_is_positive_and_clears_modeled_forbidden_bodies(self) -> None:
        retention = self.report["retention"]
        brep = self.report["exact_BREP_checks"]
        self.assertEqual(retention["fastener_count"], 3)
        self.assertEqual(retention["key_count_per_cap"], 24)
        self.assertFalse(retention["friction_or_adhesive_is_sole_retention"])
        self.assertGreaterEqual(retention["shaft_surface_clearance_mm"], 1.0)
        self.assertGreaterEqual(retention["stator_bore_surface_clearance_mm"], 0.5)
        self.assertGreaterEqual(retention["minimum_thread_engagement_mm"], 2.0)
        self.assertGreaterEqual(
            retention["key_to_finished_wire_radial_clearance_mm"], 0.1,
        )
        self.assertLessEqual(
            brep["front_key_to_stator_intersection_volume_mm3"], 1e-8,
        )
        self.assertLessEqual(
            brep["retention_screw_to_stator_intersection_volume_mm3"], 1e-8,
        )

    def test_nominal_wire_is_tangent_not_positive_volume_intruding(self) -> None:
        brep = self.report["exact_BREP_checks"]
        self.assertLessEqual(
            brep["tooth0_wire_to_own_channel_intersection_volume_mm3"], 1e-7,
        )
        self.assertLessEqual(
            brep["tooth0_wire_to_own_channel_distance_mm"], 1e-5,
        )
        self.assertLessEqual(
            brep[
                "all_24_front_wires_to_complete_cap_intersection_volume_mm3"
            ],
            1e-7,
        )
        self.assertLessEqual(
            brep[
                "all_24_rear_wires_to_complete_cap_intersection_volume_mm3"
            ],
            1e-7,
        )

    def test_release_fails_closed_on_real_motor_cavity_and_coupons(self) -> None:
        gates = self.report["release_gates"]
        self.assertTrue(gates["physical_cap_geometry"])
        self.assertTrue(gates["positive_retention_and_antirotation_present"])
        self.assertFalse(gates["actual_motor_rotor_endbell_cavity_proven"])
        self.assertFalse(gates["full_offset_flyer_raw_cycle_collision_regenerated"])
        self.assertFalse(gates["production_authorized"])
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["assembly_integration_authorized"])

    def test_mandatory_snapshot_packet_was_reviewed(self) -> None:
        visual = self.report["visual_review"]
        self.assertTrue(visual["reviewed"])
        self.assertEqual(len(visual["primary_step_snapshot_packet"]), 5)
        for relative in visual["primary_step_snapshot_packet"]:
            self.assertTrue((cap.ROOT / relative).exists(), relative)
        self.assertIn("SECTION", visual["section_actual"].upper())

    def test_every_geometry_gate_passes_and_report_round_trips(self) -> None:
        self.assertEqual(self.report["status"], "PASS_REVIEW_ONLY")
        self.assertTrue(all(self.report["checks"].values()))
        self.assertEqual(self.report["review_checks_passed"],
                         self.report["review_checks_total"])
        cap.validate_report_integrity(self.report)
        written = cap.write_reports()
        cap.validate_report_integrity(written)
        self.assertTrue(cap.JSON_OUT.exists())
        self.assertTrue(cap.MD_OUT.exists())
        self.assertTrue(cap.MANIFEST_OUT.exists())
        self.assertEqual(written["report_sha256"], self.report["report_sha256"])


if __name__ == "__main__":
    unittest.main()
