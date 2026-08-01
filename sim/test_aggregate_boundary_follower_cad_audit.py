"""Focused tests for the isolated aggregate follower CAD audit."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_cad_audit as audit


class AggregateBoundaryFollowerCadAuditTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.report = audit.analyze()

    def test_all_nine_endpoint_states_are_positive_single_and_disjoint(self):
        coverage = self.report["state_coverage"]
        self.assertEqual(coverage["audited_state_count"], 9)
        self.assertEqual(coverage["passing_state_count"], 9)
        self.assertEqual(len(coverage["states"]), 9)
        self.assertTrue(all(
            row["all_custom_bodies_positive_single_solid"]
            and row["positive_overlap_count"] == 0
            and row["status"] == "PASS"
            for row in coverage["states"]
        ))

    def test_strokes_capture_and_positive_volume_R3_contract_are_proven(self):
        stroke = self.report["stroke_and_capture"]
        self.assertEqual(stroke["usable_radial_mm"], 6.0)
        self.assertEqual(stroke["usable_tangential_mm"], 1.0)
        self.assertEqual(stroke["hard_radial_center_travel_mm"], 6.4)
        self.assertEqual(
            stroke["hard_tangential_center_stops_mm"], [-0.6, 0.6])
        self.assertTrue(stroke["all_endpoint_tongues_captured"])
        self.assertTrue(
            self.report["positive_volume_R3_prototype_geometry_proven"])
        nose = self.report["R3_nose_witness"]
        self.assertEqual(nose["source_contact_radius_mm"], 3.0)
        self.assertEqual(nose["source_cylinder_axis"], "+Z_stator_axis")
        self.assertTrue(nose["R2p99_floor_point_is_solid"])
        self.assertTrue(nose["R3p10_open_groove_point_is_clear"])

    def test_primary_M4_is_complete_and_secondary_M3_is_nonproof(self):
        hardware = self.report["hardware_witness"]
        primary = hardware["primary_M4"]
        self.assertTrue(primary["complete"])
        self.assertEqual(primary["part_count"], 12)
        self.assertEqual(primary["complete_stack_count"], 4)
        self.assertEqual(primary["load_case_N"], 40.0)
        self.assertEqual(primary["load_per_fastener_N"], 10.0)
        secondary = hardware["secondary_M3"]
        self.assertEqual(secondary["part_count"], 6)
        self.assertEqual(secondary["complete_stack_count"], 2)
        self.assertFalse(secondary["structural_proof_claimed"])
        self.assertTrue(secondary["nonproof_contract_pass"])

    def test_geometry_proof_does_not_promote_incomplete_mechanism(self):
        report = self.report
        self.assertEqual(report["status"], "FAIL")
        self.assertFalse(report["mechanism_complete"])
        self.assertFalse(report["production_authorized"])
        self.assertFalse(report["assembly_integration_authorized"])
        self.assertFalse(report["collision_authorized"])
        self.assertFalse(report["wire_route_authorized"])
        gates = report["mechanism_gates"]
        self.assertTrue(gates["inner_pivot_selected_and_retained"])
        pivots = report["hardware_witness"]["gimbal_pivots"]
        self.assertTrue(pivots["inner_pivot_selected_and_retained"])
        self.assertTrue(pivots["inner_pivot_sku"].startswith("McMaster "))
        self.assertFalse(gates[
            "radial_spring_anchors_and_bellcrank_linkage_modeled"])
        self.assertFalse(gates["tangential_bearing_selected_and_modeled"])
        self.assertFalse(gates[
            "tangential_return_spring_selected_and_anchored"])
        self.assertTrue(gates[
            "monolithic_tangential_slide_outer_yoke_complete"
        ])
        self.assertFalse(gates["M0_positive_retraction_linkage_attached"])

    def test_hash_binding_tamper_rejection_and_report_outputs(self):
        report = deepcopy(self.report)
        audit.validate_report_integrity(report)
        report["mechanism_complete"] = True
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            audit.validate_report_integrity(report)

        generated = audit.write_outputs(self.report)
        written = json.loads(audit.OUTPUT_JSON.read_text(encoding="utf-8"))
        self.assertEqual(generated["report_sha256"], written["report_sha256"])
        audit.validate_report_integrity(written)
        source = written["source_evidence"]
        self.assertEqual(len(source["cad_source_sha256"]), 64)
        self.assertEqual(len(source["cad_brief_sha256"]), 64)
        if source["step"]["exists"]:
            self.assertEqual(len(source["step"]["sha256"]), 64)
            self.assertGreater(source["step"]["byte_count"], 0)
            self.assertTrue(source["step"][
                "matches_inspected_authoritative_sha256"])
            self.assertEqual(source["step"]["leaf_count"], 40)
            self.assertEqual(source["step"]["inspection_warning_count"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
