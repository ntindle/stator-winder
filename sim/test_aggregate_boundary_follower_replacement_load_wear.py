"""Focused tests for the replacement follower load/wear audit."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_replacement_load_wear as audit


class AggregateBoundaryFollowerReplacementLoadWearTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.report = audit.analyze()

    def test_tighter_custom_return_bias_is_carried_fail_closed(self):
        load = self.report["load_envelope"]
        self.assertAlmostEqual(load["LEM_max_follower_force_N"], 1.696709251)
        self.assertEqual(load["candidate_independent_return_range_N"], [0.266, 0.286])
        self.assertEqual(load["candidate_combined_bias_range_N"], [1.962709251, 1.982709251])
        self.assertAlmostEqual(load["candidate_high_side_margin_to_2N_N"], 0.017290749)
        self.assertAlmostEqual(load["conservative_local_structural_superposition_N"], 41.982709251)
        self.assertFalse(load["positive_retraction_drive_load_bound"])

    def test_diagonal_M4_group_uses_40N_5520Nmm_governing_case(self):
        row = next(
            row for row in self.report["component_pass_fail_matrix"]
            if row["id"] == "MOUNT-01"
        )
        result = row["analytical_result"]
        self.assertEqual(result["axes_local_XY_mm"], [[29.0, -24.5], [35.0, -17.5], [29.0, 24.5], [35.0, 17.5]])
        self.assertEqual(result["sum_x_squared_mm2"], 36.0)
        self.assertEqual(result["sum_y_squared_mm2"], 1813.0)
        self.assertEqual(result["polar_sum_r_squared_mm2"], 1849.0)
        self.assertAlmostEqual(result["pure_radial_max_ideal_axial_reaction_per_screw_N"], 460.0)
        self.assertAlmostEqual(result["pure_tangential_max_ideal_axial_reaction_per_screw_N"], 74.594594595)
        self.assertAlmostEqual(result["arbitrary_in_plane_direction_max_ideal_axial_reaction_per_screw_N"], 466.008962942)
        self.assertAlmostEqual(result["M4_nominal_tensile_stress_area_mm2"], 8.78)
        self.assertAlmostEqual(result["M4_nominal_external_von_Mises_proxy_MPa"], 53.094343929)
        self.assertAlmostEqual(
            result["nominal_external_von_Mises_to_reference_proof_factor"],
            3.95522356,
        )
        self.assertAlmostEqual(result["NBK_reference_0p2_proof_load_N"], 1843.8)
        self.assertAlmostEqual(
            result["external_axial_reaction_to_reference_proof_load_margin_N"],
            1377.791037058,
        )
        self.assertEqual(result["NBK_maximum_tightening_torque_Nm"], 1.0)
        self.assertIsNone(result["selected_assembly_torque_Nm"])
        self.assertIsNone(result["bearing_pressure_from_tightening_MPa"])
        self.assertIsNone(result["joint_preload_and_separation_margin"])
        self.assertEqual(row["qualification_status"], "FAIL")

    def test_inner_outer_pins_have_nominal_stress_but_no_invented_margin(self):
        rows = {row["id"]: row for row in self.report["component_pass_fail_matrix"]}
        outer = rows["STRUCT-01"]["analytical_result"]
        inner = rows["STRUCT-02"]["analytical_result"]
        self.assertAlmostEqual(outer["gross_von_Mises_proxy_MPa"], 6.266674727, places=6)
        self.assertAlmostEqual(inner["gross_von_Mises_proxy_MPa"], 28.190140941, places=6)
        self.assertIsNone(outer["material_allowable_MPa"])
        self.assertIsNone(inner["margin_to_allowable"])
        self.assertIn("NETWS4 ring axial rating", rows["STRUCT-01"]["missing"])
        self.assertEqual(outer["NETWS4_hardness_HRC_range"], [37.0, 46.0])
        self.assertIsNone(outer["NETWS4_supplier_axial_thrust_rating_N"])
        self.assertIn("M2 thread/nut retention", rows["STRUCT-02"]["missing"])

    def test_PEEK_shaft_and_bushing_screen_values_remain_non_authoritative(self):
        rows = {row["id"]: row for row in self.report["component_pass_fail_matrix"]}
        peek = rows["WEAR-01"]["analytical_result"]
        shaft = rows["STRUCT-04"]["analytical_result"]
        self.assertAlmostEqual(peek["minimum_radial_ligament_mm"], 1.4)
        self.assertAlmostEqual(peek["nominal_pin_bore_bearing_stress_MPa"], 3.498559104, places=6)
        self.assertAlmostEqual(peek["wire_line_load_on_0p65mm_band_N_per_mm"], 64.588783463, places=6)
        self.assertIsNone(peek["contact_pressure_or_Hertz_solution"])
        self.assertAlmostEqual(shaft["shaft_gross_bending_stress_MPa"], 63.352942949, places=6)
        self.assertAlmostEqual(shaft["igus_nominal_projected_pressure_MPa"], 2.798847283, places=6)
        self.assertAlmostEqual(
            shaft["igus_W300_supplier_static_limit_MPa_at_20C"],
            59.998177965,
        )
        self.assertAlmostEqual(shaft["igus_static_pressure_screen_factor"], 21.436745878)
        self.assertAlmostEqual(
            shaft["igus_static_pressure_screen_margin_MPa"],
            57.199330682,
        )
        self.assertAlmostEqual(
            shaft["igus_supplier_static_projected_load_screen_N"],
            899.972669471,
        )
        self.assertTrue(shaft["igus_static_pressure_screen_pass"])
        self.assertAlmostEqual(
            shaft["PV_limited_speed_ceiling_at_screen_pressure_m_per_s"],
            0.082593796,
        )
        self.assertEqual(shaft["current_bushing_wall_thickness_mm"], 0.75)
        self.assertEqual(shaft["supplier_PV_table_wall_thickness_mm"], 1.0)
        self.assertIsNone(shaft["actual_PV_MPa_m_per_s"])
        self.assertIsNone(shaft["igus_dynamic_PV_margin"])
        self.assertFalse(shaft["igus_pressure_PV_and_wear_authority"])

    def test_official_supplier_records_are_hash_bound_but_not_authority(self):
        sources = self.report["official_supplier_sources"]
        self.assertEqual(
            set(sources),
            {
                "NBK_SSHS_M4_10_SD_ALK",
                "NBK_small_head_engineering_table",
                "NBK_A2_50_reference_properties",
                "igus_W300_material",
                "igus_WPFFM_0304_05",
                "MISUMI_NETWS",
            },
        )
        for source in sources.values():
            self.assertTrue(source["url"].startswith("https://"))
            self.assertEqual(len(source["source_record_sha256"]), 64)
            self.assertEqual(
                source["source_record_sha256"],
                audit._source_record_hash(source),
            )
        self.assertFalse(
            sources["NBK_A2_50_reference_properties"]["facts"]
            ["values_guaranteed_for_this_received_lot"]
        )
        self.assertFalse(
            sources["MISUMI_NETWS"]["facts"]
            ["per_part_axial_thrust_rating_available"]
        )
        self.assertFalse(any(self.report["authority"].values()))

    def test_screening_margin_is_not_endurance_authority(self):
        spring = next(
            row for row in self.report["component_pass_fail_matrix"]
            if row["id"] == "SPRING-01"
        )
        result = spring["analytical_result"]
        self.assertGreater(result["modified_Goodman_screening_factor"], 2.8)
        self.assertGreater(result["static_yield_screening_factor"], 4.6)
        self.assertTrue(result["screening_endurance_is_assumed"])
        self.assertEqual(
            spring["qualification_status"],
            "FAIL_NOT_MANUFACTURED_OR_ENDURANCE_TESTED",
        )

    def test_test_matrix_is_precise_about_known_loads_and_missing_duty(self):
        tests = self.report["physical_test_requirements"]
        self.assertAlmostEqual(
            tests["single_insert_and_torque_coupons"]["derived_external_axial_load_per_insert_N"],
            466.008962942,
        )
        self.assertEqual(
            tests["shaft_bushing_breakaway_and_PV"]["breakaway_requirement_N"],
            0.125,
        )
        self.assertIn(
            "required cycle count is not yet bound",
            tests["spring_and_retraction_endurance"]["acceptance"],
        )
        self.assertIn(
            "axial side-thrust test load remains undefined",
            tests["pivot_and_retention_fixture"]["acceptance"],
        )

    def test_all_sources_are_current_and_all_authority_is_false(self):
        audit.validate_report_integrity(self.report)
        self.assertTrue(all(self.report["analytical_gates"].values()))
        self.assertFalse(any(self.report["qualification_gates"].values()))
        self.assertEqual(set(self.report["authority"]), set(audit.AUTHORITY_KEYS))
        self.assertFalse(any(self.report["authority"].values()))
        self.assertEqual(
            len(self.report["open_blockers"]),
            len(self.report["qualification_gates"]),
        )
        for binding in self.report["source_bindings"].values():
            self.assertEqual(len(binding["sha256"]), 64)
            self.assertGreater(binding["byte_count"], 0)

    def test_geometry_chain_guard_rejects_mixed_old_and_new_artifacts(self):
        replacement = audit._load(audit.SOURCE_PATHS["replacement_CAD_audit"])
        audit._require_current_geometry_chain(replacement)
        stale = deepcopy(replacement)
        stale["artifact_binding"]["cad_source"]["sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "geometry chain is not current"):
            audit._require_current_geometry_chain(stale)

    def test_hash_tamper_detection_and_written_outputs(self):
        tampered = deepcopy(self.report)
        tampered["load_envelope"]["governing_structural_proof_force_N"] = 39.0
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            audit.validate_report_integrity(tampered)

        written_report = audit.write_outputs(self.report)
        written = json.loads(audit.OUTPUT_JSON.read_text(encoding="utf-8"))
        self.assertEqual(written_report["report_sha256"], written["report_sha256"])
        audit.validate_report_integrity(written)
        markdown = audit.OUTPUT_MD.read_text(encoding="utf-8")
        self.assertIn("1.982709251 N", markdown)
        self.assertIn("466.009 N", markdown)
        self.assertIn("Required physical evidence", markdown)


if __name__ == "__main__":
    unittest.main(verbosity=2)
