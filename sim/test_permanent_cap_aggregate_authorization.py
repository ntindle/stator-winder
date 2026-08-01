from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = ROOT / "cad"
for path in (HERE, CAD):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import permanent_cap_aggregate_authorization as study


class PermanentCapAggregateAuthorizationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = study.analyze()

    def test_canonical_unmodified_raw_contract_is_exact(self) -> None:
        raw = self.report["canonical_raw_capture"]
        self.assertEqual(raw["status"], "PASS")
        self.assertEqual(raw["sha256"],
                         "350210c36550ae2c22e5352675aee6bd2eebb18325f3b5f45c7cb35d6a314958")
        self.assertEqual(raw["pass_count"], 24)
        self.assertEqual(raw["locus_count"], 2400)
        self.assertEqual(raw["controller_mode"], "upstream")
        self.assertIsNone(raw["controller_adapter_sha256"])

    def test_each_half_slot_has_fifty_wire_areas_and_fill_is_bounded(self) -> None:
        slot = self.report["slot_partition"]
        expected = 50.0 * math.pi * (0.22352 / 2.0) ** 2
        self.assertAlmostEqual(
            slot["required_50_turn_copper_area_mm2"], expected, places=12)
        self.assertAlmostEqual(
            slot["selected_aggregate_half_slot_area_mm2"], expected, places=10)
        self.assertGreater(
            slot["available_center_safe_half_slot_area_mm2"], expected)
        self.assertLessEqual(slot["gross_slot_fill"], 0.60)
        self.assertEqual(
            slot["all_24_partition_topology"]["adjacent_interior_overlap_area_mm2"],
            0.0,
        )

    def test_liner_core_and_all_cross_coil_interiors_are_disjoint(self) -> None:
        loft = self.report["aggregate_loft"]
        self.assertEqual(
            loft["core_intrusion"]["positive_volume_intersection_mm3"], 0.0)
        self.assertAlmostEqual(
            loft["core_intrusion"]["minimum_wire_outer_surface_to_core_mm"],
            0.127,
            places=12,
        )
        self.assertEqual(
            loft["nonoverlap_proof"]["adjacent_positive_volume_overlap_mm3"],
            0.0,
        )
        self.assertEqual(
            loft["nonoverlap_proof"]["nonadjacent_positive_volume_overlap_mm3"],
            0.0,
        )
        self.assertEqual(loft["closed_aggregate_count"], 24)
        self.assertGreaterEqual(
            loft["minimum_declared_loft_section_area_mm2"],
            loft["required_copper_cross_section_mm2"] - study.AREA_ABS_TOL_MM2,
        )

    def test_complete_cap_lane_meets_R3_and_ports_fit(self) -> None:
        lane = self.report["cap_support_lane"]
        self.assertTrue(all(
            row["inside_selected_aggregate_cutoff"]
            and row["full_lane_half_width_fits_port"]
            for row in lane["endpoint_ports"]
        ))
        self.assertTrue(
            lane["nominal_front_centerline"]["C1_tangent_continuity"])
        self.assertGreaterEqual(
            lane["minimum_lane_wire_center_bend_radius_mm"], 3.0)
        self.assertAlmostEqual(
            lane["support_surface_contract"]["minimum_contact_surface_radius_mm"]
            + lane["finished_wire_radius_mm"],
            3.0,
            places=12,
        )
        for value in lane["nominal_front_centerline"][
                "minimum_sampled_domain_margins_mm"].values():
            self.assertGreaterEqual(value, -1.0e-9)

    def test_connectors_join_by_full_area_and_clear_every_forbidden_body(self) -> None:
        connectors = self.report["slot_to_crown_connectors"]
        loft = self.report["aggregate_loft"]
        self.assertEqual(connectors["status"], "PASS")
        self.assertEqual(connectors["connector_count"], 96)
        self.assertAlmostEqual(
            connectors["positive_area_join_to_crown_mm2"],
            loft["required_copper_cross_section_mm2"],
            places=10,
        )
        clearance = connectors["clearance_audit"]
        for name in (
            "core_positive_volume_intrusion_mm3",
            "cap_positive_volume_intrusion_mm3",
            "adjacent_aggregate_positive_volume_intrusion_mm3",
            "nonadjacent_aggregate_positive_volume_intrusion_mm3",
        ):
            self.assertEqual(clearance[name], 0.0)
        self.assertGreaterEqual(
            clearance["minimum_lane_margin_after_lane_and_wire_inset_mm"],
            -1.0e-9,
        )
        progressive = connectors["progressive_aggregate_contract"]
        self.assertEqual(
            progressive["active_prior_aggregate_positive_volume_intrusion_mm3"],
            0.0,
        )
        self.assertIn("zero-distance crossings", progressive["note"])

    def test_OD_raw_span_and_explicit_axial_envelope(self) -> None:
        slot = self.report["slot_partition"]
        lane = self.report["cap_support_lane"]
        self.assertTrue(slot["raw_M0_span_contains_complete_aggregate"])
        self.assertLessEqual(
            slot["aggregate_outer_center_radius_mm"],
            slot["raw_radial_center_span_mm"][1] + 1.0e-9,
        )
        self.assertGreater(lane["finished_wire_total_axial_envelope_mm"], 0.0)
        bounds = lane["front_rear_finished_wire_envelope_mm"]
        self.assertAlmostEqual(bounds[0], -bounds[1], places=12)
        self.assertAlmostEqual(
            bounds[1] - bounds[0],
            lane["finished_wire_total_axial_envelope_mm"],
            places=12,
        )

    def test_PASS_is_aggregate_only_and_emits_path_based_hashes(self) -> None:
        self.assertEqual(self.report["status"], "PASS")
        self.assertTrue(self.report["aggregate_geometry_authorized"])
        self.assertTrue(self.report["offset_flyer_input_authorized"])
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["assembly_integration_authorized"])
        self.assertEqual(self.report["controlling_blockers"], [])
        hashes = self.report["source_hashes"]
        for path in (
            "GOAL.md",
            "cad/coil_growth.py",
            "sim/aggregate_progressive_wire_corridor.py",
            "sim/phase_aware_progressive_wire_audit.py",
            "sim/r3_sector_chord_family_study.py",
            "sim/permanent_cap_flyer_recovery_study.py",
            "sim/permanent_cap_aggregate_authorization.py",
            "out/capture/upstream_current_raw.jsonl",
        ):
            self.assertIn(path, hashes)
            self.assertEqual(len(hashes[path]), 64)

    def test_report_hash_and_written_outputs_round_trip(self) -> None:
        study.validate_report_integrity(self.report)
        written = study.write_reports(self.report)
        study.validate_report_integrity(written)
        self.assertTrue(study.JSON_OUT.exists())
        self.assertTrue(study.MD_OUT.exists())
        self.assertEqual(written["report_sha256"], self.report["report_sha256"])


if __name__ == "__main__":
    unittest.main()
