"""Regression tests for the fail-closed active-tooth guide study."""

import hashlib
import json
import unittest

import flyer_slot_guide_feasibility as guide


class FlyerSlotGuideFeasibilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fast = guide.analyze(include_occ_witness=False)

    def test_max_wire_fits_but_closed_nozzle_does_not_have_wall_budget(self):
        budget = self.fast["mouth_and_guide_budget"]
        slot = budget["slot"]
        corridor = budget["wire_center_corridor"]
        nozzle = budget["enclosed_nozzle_wall_budget"]
        self.assertAlmostEqual(slot["bare_mouth_mm"],
                               1.5339056464564453, places=12)
        self.assertAlmostEqual(slot["lined_mouth_mm"],
                               1.2539056464564453, places=12)
        self.assertAlmostEqual(slot["existing_cap_mouth_mm"],
                               0.8339056464564454, places=12)
        self.assertGreater(corridor["with_current_cap_mm"], 0.0)
        self.assertAlmostEqual(
            nozzle["maximum_symmetric_wall_each_with_current_cap_mm"],
            0.1169528232282227, places=12)
        self.assertEqual(nozzle["current_cap_release_status"],
                         "NO_GO_UNQUALIFIED_ULTRATHIN_WALL")

    def test_three_mm_bend_feature_and_pitch_constraints_are_explicit(self):
        bend = self.fast["mouth_and_guide_budget"][
            "three_mm_bend_geometry"]
        self.assertAlmostEqual(
            bend["minimum_external_guide_surface_radius_mm"], 2.75)
        self.assertAlmostEqual(bend["minimum_external_pin_diameter_mm"], 5.5)
        self.assertAlmostEqual(
            bend["minimum_concave_groove_root_radius_mm"], 3.25)
        self.assertLess(bend["pin_gap_at_shoe_inner_radius_mm"], 0.0)
        self.assertGreater(bend["pin_gap_at_stator_od_mm"], 0.0)
        self.assertAlmostEqual(
            bend["existing_flare_projection_shortfall_mm"], 1.5)

    def test_split_shoe_has_only_a_tight_geometric_error_budget(self):
        candidate = self.fast["candidate"]
        self.assertEqual(candidate["status"], "CONCEPT_ONLY_NOT_PROVEN")
        self.assertAlmostEqual(
            candidate["one_blade_per_adjacent_slot_target_thickness_mm"],
            0.25)
        self.assertAlmostEqual(
            candidate["liner_only_residual_lateral_budget_mm"],
            0.4039056464564453, places=12)
        self.assertAlmostEqual(
            candidate["maximum_combined_lateral_error_for_centered_budget_mm"],
            0.20195282322822265, places=12)
        self.assertGreater(
            candidate["minimum_polished_radial_working_span_mm"], 6.5)

    def test_report_can_never_authorize_current_geometry(self):
        report = self.fast
        self.assertEqual(report["status"], "DESIGN_CHANGE_REQUIRED")
        self.assertFalse(report["release_authorized"])
        self.assertGreaterEqual(len(report["failures"]), 6)
        self.assertTrue(report["proofs_required_before_cad_release"])

    def test_report_hash_is_canonical(self):
        report = dict(self.fast)
        expected = report.pop("report_sha256")
        actual = hashlib.sha256(json.dumps(
            report, sort_keys=True, separators=(",", ":"),
            allow_nan=False).encode()).hexdigest()
        self.assertEqual(expected, actual)

    def test_occ_witness_proves_bare_core_and_endpoint_penetration(self):
        report = guide.analyze(include_occ_witness=True)
        witness = report["exact_penetration_witness"]
        self.assertEqual(witness["classification"], "TRUE_CORE_PENETRATION")
        self.assertEqual(witness["core_inside_sample_count"], 36)
        self.assertEqual(
            witness["neighbor_tooth_2_envelope_inside_sample_count"], 62)
        self.assertTrue(
            witness["constructed_lay_endpoint_inside_bare_core"])
        self.assertEqual(witness["core_inside_index_span"], [356, 413])
        self.assertEqual(
            witness["neighbor_tooth_2_inside_index_span"], [352, 413])


if __name__ == "__main__":
    unittest.main()
