"""Focused tests for the four-shoe replacement-carriage contract."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_replacement_architecture as architecture


class AggregateBoundaryFollowerReplacementArchitectureTests(unittest.TestCase):

    def test_three_laws_four_tracks_and_gate_states_fail_closed(self):
        report = architecture.build_report()
        rows = report["selection_contract"]["rows"]
        self.assertEqual(len(rows), 36)
        for row in rows:
            selected = row["selected_physical_ids"]
            if row["M0_gate_state"] == "ENGAGED_LOCKED":
                self.assertEqual(len(selected), 1)
            else:
                self.assertEqual(selected, [])

    def test_reverse_180_permutation_and_carriage_ownership(self):
        self.assertEqual(
            [architecture.selected_physical_id(architecture.LAW_REVERSE_180, i) for i in range(4)],
            [2, 3, 0, 1],
        )
        self.assertEqual(
            [architecture.selected_physical_id(architecture.LAW_DIRECT, i) for i in range(4)],
            [0, 1, 2, 3],
        )
        rows = architecture.selected_occurrences(
            architecture.LAW_REVERSE_ZERO, 2, "ENGAGED_LOCKED"
        )
        self.assertTrue(all(row["owner"] == "carriage" for row in rows))
        self.assertTrue(all(not row["M1_spatial_transform"] for row in rows))
        self.assertTrue(all(not row["M2_spatial_transform"] for row in rows))

    def test_handed_transform_and_exact_nose_centers(self):
        expected = {
            0: (2.05, 21.35, 65.30),
            1: (-2.05, 21.35, 65.30),
            2: (-2.05, -21.35, 65.30),
            3: (2.05, -21.35, 65.30),
        }
        for physical_id, center in expected.items():
            got = architecture.nose_machine_center(
                physical_id, "retracted", selected=True
            )
            for actual, target in zip(got, center):
                self.assertAlmostEqual(actual, target, places=9)
        self.assertEqual(
            architecture.nose_machine_center(0, "mid", selected=True)[2],
            62.3,
        )
        self.assertEqual(
            architecture.nose_machine_center(0, "extended", selected=True)[2],
            59.3,
        )

    def test_exact_install_counts_and_coarse_selector_blocker(self):
        report = architecture.build_report()
        counts = report["exact_install_counts"]
        self.assertEqual(counts["shared_U_windowed_replacement_carrier"], 1)
        self.assertEqual(counts["physical_follower_occurrences"], 4)
        self.assertEqual(counts["tower_M4_screws"], 4)
        self.assertEqual(counts["tower_M4_washers"], 0)
        self.assertEqual(counts["tower_M4_inserts"], 4)
        self.assertEqual(counts["outer_pivot_SCCG5_10_pins"], 4)
        self.assertEqual(counts["outer_pivot_DIN988_shims"], 8)
        self.assertEqual(counts["outer_pivot_NETWS4_rings"], 8)
        for name in (
            "old_active_sector_yoke", "old_PEEK_active_sector_guides",
            "old_secondary_M3_stacks", "mounting_backer_context",
            "follower_local_tower_M4_stacks", "central_spine",
        ):
            self.assertEqual(counts[name], 0)
        self.assertAlmostEqual(
            report["travel"]["coarse_selection_stroke_mm"], 8.90
        )
        self.assertFalse(report["physical_gates"][
            "positive_volume_8p90mm_selector_linkage_integrated"
        ])

    def test_finalized_diagonal_M4_pattern_and_exact_hardware(self):
        report = architecture.build_report()
        self.assertEqual(
            report["shared_adapter"]["M4_axes_local_xy_mm"],
            [[29.0, -24.5], [35.0, -17.5],
             [29.0, 24.5], [35.0, 17.5]],
        )
        self.assertEqual(
            report["shared_adapter"]["M4_same_side_diagonal_delta_xy_mm"],
            [6.0, 7.0],
        )
        self.assertTrue(
            report["shared_adapter"]["M4_proof_basis_x_row_span_preserved"]
        )
        mount = report["primary_mount_hardware"]
        self.assertEqual(mount["screw_sku"], "NBK SSHS-M4-10-SD-ALK")
        self.assertEqual(mount["screw_count"], 4)
        self.assertEqual(mount["washer_count"], 0)
        self.assertEqual(mount["insert_count"], 4)
        self.assertEqual(mount["leaf_count"], 8)

    def test_each_occurrence_has_finalized_15_leaf_pivot_stack(self):
        report = architecture.build_report()
        contract = report["occurrence_leaf_contract"]
        self.assertEqual(contract["physical_occurrence_count"], 4)
        self.assertEqual(contract["leaf_count_per_occurrence"], 15)
        self.assertEqual(contract["custom_body_count_per_occurrence"], 4)
        outer = contract["outer_pivot"]
        self.assertEqual(outer["pin_sku"], "MISUMI SCCG5-10")
        self.assertEqual(outer["pin_count_per_occurrence"], 1)
        self.assertEqual(outer["DIN988_shim_count_per_occurrence"], 2)
        self.assertEqual(outer["NETWS4_ring_count_per_occurrence"], 2)
        self.assertEqual(outer["inward_shoulder_screw_or_nyloc_count"], 0)
        self.assertEqual(contract["inner_pivot_leaf_count_per_occurrence"], 6)

    def test_review_leaf_accounting_and_fail_closed_blockers(self):
        report = architecture.build_report()
        leaves = report["review_leaf_counts"]
        self.assertEqual(leaves["moving_occurrence_manufactured_leaves"], 60)
        self.assertEqual(leaves["primary_mount_manufactured_leaves"], 8)
        self.assertEqual(leaves["manufactured_leaves"], 69)
        self.assertEqual(leaves["coarse_linkage_blocker_envelopes"], 4)
        self.assertEqual(leaves["total_review_leaves"], 73)
        self.assertEqual(
            report["nominal_clearance_not_authority"]
            ["complete_outer_pivot_envelope_clearance_mm"],
            3.0,
        )
        self.assertEqual(
            report["nominal_clearance_not_authority"]
            ["inward_q_complete_outer_pivot_envelope_clearance_mm"],
            2.5,
        )
        self.assertEqual(
            report["nominal_clearance_not_authority"]
            ["nominal_reserve_above_2mm_requirement_mm"],
            0.5,
        )
        self.assertFalse(
            report["nominal_clearance_not_authority"]
            ["tolerance_stack_qualified"]
        )
        self.assertIn(
            "SCCG5_10_pin_retention_load_and_wear_qualification",
            report["blockers"],
        )
        self.assertNotIn("0p10", json.dumps(report))

    def test_carrier_relief_and_trim_are_bound_without_count_drift(self):
        shared = architecture.build_report()["shared_adapter"]
        self.assertEqual(
            shared["parked_follower_relief_bounds_local_mm"],
            {
                "x": [25.0, 36.2],
                "abs_y": [5.45, 16.5],
                "abs_z": [9.85, 27.85],
            },
        )
        self.assertEqual(
            shared["selection_wall_abs_z_bounds_mm"], [2.85, 12.85],
        )
        self.assertEqual(
            shared["selection_bay_tangential_clearance_mm"], 0.50,
        )
        self.assertEqual(
            shared["outboard_dogleg_web_min_radial_thickness_mm"], 2.80,
        )
        self.assertTrue(shared["carrier_one_solid_required"])

    def test_hash_binding_tamper_rejection_and_written_output(self):
        report = architecture.build_report()
        architecture.validate_report_integrity(report)
        bad = deepcopy(report)
        bad["assembly_integration_authorized"] = True
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            architecture.validate_report_integrity(bad)
        generated = architecture.write_outputs(report)
        written = json.loads(architecture.OUTPUT_JSON.read_text(encoding="utf-8"))
        self.assertEqual(generated["report_sha256"], written["report_sha256"])
        architecture.validate_report_integrity(written)


if __name__ == "__main__":
    unittest.main(verbosity=2)
