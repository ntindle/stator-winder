"""Regression gates for fixed-mouth rejection and passive successor."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import passive_terminal_guide_successor as study


class PassiveTerminalGuideSuccessorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = json.loads(study.OUTPUT_JSON.read_text(encoding="utf-8"))
        study.validate_report(cls.report)

    def test_physical_shaft_to_tip_guide_is_consumed(self) -> None:
        source = self.report["source_guide"]
        self.assertEqual(
            source["status"],
            "GEOMETRY_PASS_REVIEW_ONLY__TERMINAL_ROUTE_FAIL",
        )
        self.assertTrue(source["geometry_gates_pass"])
        self.assertTrue(
            self.report["authority_gates"]
            ["physical_shaft_to_tip_PEEK_guide_geometry_pass"]
        )

    def test_all_2400_raw_terminal_loci_are_replayed(self) -> None:
        sweep = self.report["raw_terminal_sweep"]
        self.assertEqual(sweep["pass_count"], 24)
        self.assertEqual(sweep["locus_count"], 2400)
        self.assertEqual(sweep["constructed_loci"], 2400)
        self.assertEqual(sweep["unique_geometry_cases"], 520)
        self.assertEqual(sweep["implicit_kink_loci"], 2400)
        self.assertEqual(sweep["core_crossing_loci"], 1000)
        self.assertTrue(sweep["predecessor_counts_match"])

    def test_fixed_R3_lead_in_exceeds_authorized_cap_domain(self) -> None:
        fixed = self.report["fixed_cap_lead_in_impossibility"]
        self.assertTrue(fixed["proved"])
        self.assertGreater(fixed["worst_approach_turn_deg"], 60.0)
        self.assertGreater(
            fixed["R3_minimum_lateral_turn_sweep_mm"],
            fixed["authorized_lane_sector_margin_mm"],
        )
        self.assertGreater(fixed["lateral_sweep_deficit_mm"], 1.4)
        self.assertGreater(
            fixed["required_full_capture_width_mm"],
            fixed["current_open_mouth_width_mm"],
        )
        self.assertGreater(fixed["mouth_width_deficit_mm"], 4.7)

    def test_48_independent_fixed_R3_mouth_guides_cannot_fit(self) -> None:
        spacing = self.report["fixed_cap_lead_in_impossibility"][
            "port_spacing"
        ]
        self.assertEqual(spacing["port_count_per_cap"], 48)
        self.assertLess(spacing["minimum_center_spacing_mm"], 0.7)
        self.assertEqual(spacing["two_independent_R3_diameter_mm"], 6.0)
        self.assertGreater(spacing["independent_R3_envelope_overlap_mm"], 5.3)
        self.assertFalse(
            self.report["fixed_cap_gates"]
            ["independent_R3_mouth_guides_do_not_overlap"]
        )

    def test_smallest_passive_architecture_uses_existing_axes_only(self) -> None:
        architecture = self.report["smallest_passive_architecture"]
        self.assertFalse(architecture["new_commanded_axis"])
        self.assertFalse(architecture["upstream_protocol_change"])
        self.assertEqual(architecture["law_count"], 3)
        self.assertEqual(architecture["shoe_count"], 4)
        self.assertEqual(architecture["simultaneously_deployed_shoes"], 1)
        self.assertEqual(
            len(set(architecture["required_shoe_identities"])), 4
        )
        self.assertEqual(
            architecture["ceramic_wire_center_radius_mm"], 3.25
        )
        self.assertGreater(architecture["angular_allowance_deg"], 4.0)

    def test_gimbal_concept_remains_fail_closed_until_rerouted(self) -> None:
        gates = self.report["passive_successor_gates"]
        self.assertTrue(gates["three_raw_cam_laws_positively_selected_by_M1"])
        self.assertTrue(gates["four_mutually_exclusive_R3_shoes_defined"])
        self.assertTrue(gates["gimbal_contains_measured_approach_cone"])
        self.assertFalse(
            gates["all_2400_gimballed_shoe_routes_clear_core_and_aggregate"]
        )
        self.assertFalse(gates["complete_passive_shoe_collision_sweep"])
        self.assertEqual(self.report["status"], "FAIL")
        self.assertFalse(self.report["production_authorized"])

    def test_exact_strand_packing_is_not_promoted(self) -> None:
        self.assertEqual(
            self.report["authority_boundary"]
            ["exact_strand_centers_order_settling_neatness"],
            "non-authoritative",
        )
        self.assertTrue(
            self.report["authority_gates"]
            ["exact_strand_packing_not_authority"]
        )

    def test_report_hash_rejects_tampering(self) -> None:
        changed = deepcopy(self.report)
        changed["production_authorized"] = True
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            study.validate_report(changed)


if __name__ == "__main__":
    unittest.main(verbosity=2)
