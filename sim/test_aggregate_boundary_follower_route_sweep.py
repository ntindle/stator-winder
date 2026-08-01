"""Focused tests for the fail-closed 2,400-locus follower route sweep."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_route_sweep as sweep


class AggregateBoundaryFollowerRouteSweepTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.report = sweep.analyze()
        cls.cases = [
            case for row in cls.report["loci"]
            for case in row["diameter_cases"]
        ]

    def test_all_loci_diameters_identities_and_laws_are_bound(self):
        report = self.report
        self.assertEqual(report["coverage"]["evaluated_loci"], 2400)
        self.assertEqual(report["coverage"]["diameter_route_case_count"], 4800)
        self.assertEqual(len(report["loci"]), 2400)
        self.assertEqual({
            row["identity"]["physical_id"] for row in report["loci"]
        }, {0, 1, 2, 3})
        self.assertTrue(all(
            len(row["diameter_cases"]) == 2
            and set(row["law_track_bindings"]) == set(sweep.LAWS)
            for row in report["loci"]
        ))
        example = next(row for row in report["loci"]
                       if row["identity"]["physical_id"] == 0)
        self.assertEqual(example["law_track_bindings"], {
            sweep.LAWS[0]: 0,
            sweep.LAWS[1]: 0,
            sweep.LAWS[2]: 2,
        })

    def test_both_diameters_fit_lane_and_R3_radius_contract(self):
        self.assertTrue(self.report["analytic_gates"]
                        ["all_cases_fit_0p65_cap_lane_and_follower_groove"])
        margins = {
            case["wire_diameter_mm"]: case["cap_lane_diametral_margin_mm"]
            for case in self.cases[:2]
        }
        self.assertAlmostEqual(margins[0.2], 0.45)
        self.assertAlmostEqual(margins[0.5], 0.15)
        radii = self.report["diameter_and_contact_contract"][
            "wire_centerline_radius_by_diameter_mm"
        ]
        self.assertEqual(radii, {"d0.2": 3.1, "d0.5": 3.25})
        self.assertTrue(all(
            case["minimum_centerline_radius_analytic_pass"]
            for case in self.cases
        ))

    def test_right_shelf_is_exact_but_left_rebound_is_analytic_only(self):
        right = [
            case for row in self.report["loci"] if row["side_sign"] > 0
            for case in row["diameter_cases"]
        ]
        left = [
            case for row in self.report["loci"] if row["side_sign"] < 0
            for case in row["diameter_cases"]
        ]
        self.assertEqual(len(right), 2400)
        self.assertTrue(all(case["terminal_C0_exact"] for case in right))
        self.assertTrue(all(case["R0p36_insertion_gauge_exact_clear"]
                            for case in right))
        self.assertTrue(all(not case["terminal_C0_exact"] for case in left))
        self.assertTrue(all(case["terminal_C0_analytic"] for case in left))
        left_d02 = next(case for case in left
                        if case["wire_diameter_mm"] == 0.2)
        left_d05 = next(case for case in left
                        if case["wire_diameter_mm"] == 0.5)
        self.assertAlmostEqual(left_d02["endpoint_rebind_magnitude_mm"], 0.15)
        self.assertFalse(left_d02["upstream_predecessor_centerline_reused"])
        self.assertAlmostEqual(left_d05["endpoint_rebind_magnitude_mm"], 0.0)
        self.assertTrue(left_d05["upstream_predecessor_centerline_reused"])

    def test_nonzero_C0_and_normal_sign_are_complete_but_C1_is_zero(self):
        coverage = self.report["coverage"]
        self.assertEqual(coverage["g0_case_count"], 96)
        self.assertEqual(coverage["nonzero_growth_case_count"], 4704)
        self.assertEqual(
            coverage["nonzero_analytic_aggregate_C0_case_count"], 4704
        )
        self.assertEqual(coverage["nonzero_direct_C1_case_count"], 0)
        self.assertEqual(coverage["positive_volume_locus_arc_case_count"], 0)
        self.assertTrue(self.report["analytic_gates"]
                        ["all_selected_aggregate_normals_are_outward_supports"])
        self.assertGreater(
            self.report["turn_and_length_bounds"]["minimum_direct_C1_error_deg"],
            10.0,
        )

    def test_length_is_explicitly_a_proxy_and_dancer_stays_unauthorized(self):
        bounds = self.report["turn_and_length_bounds"]
        self.assertFalse(bounds["exact_route_length_available"])
        self.assertFalse(bounds["dancer_coupling_available"])
        for key in ("d0.2", "d0.5"):
            row = bounds["length_proxy"][key]
            self.assertEqual(row["case_count"], 2400)
            self.assertGreater(row["maximum_proxy_length_mm"],
                               row["minimum_proxy_length_mm"])
            self.assertGreater(row["maximum_consecutive_locus_proxy_delta_mm"],
                               0.0)
            self.assertGreater(
                row["maximum_consecutive_locus_proxy_rate_mm_s"], 0.0
            )
            self.assertEqual(set(row["per_identity"]), {"0", "1", "2", "3"})
        self.assertFalse(self.report["dancer_coupling_authorized"])
        self.assertFalse(self.report["wire_route_authorized"])
        self.assertEqual(
            self.report["coverage"]["physically_authorized_route_case_count"],
            0,
        )

    def test_hash_binding_tamper_rejection_and_written_outputs(self):
        sweep.validate_report_integrity(self.report)
        bad = deepcopy(self.report)
        bad["wire_route_authorized"] = True
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            sweep.validate_report_integrity(bad)
        generated = sweep.write_outputs(self.report)
        written = json.loads(sweep.OUTPUT_JSON.read_text(encoding="utf-8"))
        self.assertEqual(generated["report_sha256"], written["report_sha256"])
        sweep.validate_report_integrity(written)


if __name__ == "__main__":
    unittest.main(verbosity=2)
