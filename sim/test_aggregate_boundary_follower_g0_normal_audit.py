"""Focused tests for the exact g=0 physical-normal audit."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import aggregate_boundary_follower_g0_normal_audit as audit


class AggregateBoundaryFollowerG0NormalAuditTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.report = audit.analyze()

    def test_exact_48_locus_mapping_is_24_owned_and_24_unsupported(self):
        coverage = self.report["coverage"]
        self.assertEqual(coverage["classified_g0_locus_count"], 48)
        self.assertEqual(coverage["existing_positive_BREP_owner_count"], 24)
        self.assertEqual(coverage["unsupported_count"], 24)
        rows = coverage["loci"]
        owned = [row for row in rows if row["existing_positive_BREP_owner"]]
        unsupported = [
            row for row in rows if not row["existing_positive_BREP_owner"]
        ]
        self.assertTrue(all(row["side"] == "left" for row in owned))
        self.assertTrue(all(row["side"] == "right" for row in unsupported))
        self.assertEqual(
            [row["locus_index"] for row in owned],
            [0, 101, 200, 301, 401, 500, 601, 700, 801, 900,
             1001, 1100, 1200, 1301, 1400, 1501, 1600, 1701,
             1800, 1901, 2001, 2100, 2201, 2300],
        )
        self.assertEqual(
            [row["locus_index"] for row in unsupported],
            [1, 100, 201, 300, 400, 501, 600, 701, 800, 901,
             1000, 1101, 1201, 1300, 1401, 1500, 1601, 1700,
             1801, 1900, 2000, 2101, 2200, 2301],
        )

    def test_left_floor_distance_and_right_gap_are_exact(self):
        coverage = self.report["coverage"]
        for value in coverage["left_surface_distance_range_mm"]:
            self.assertAlmostEqual(value, 0.11176, places=10)
        for value in coverage["right_surface_distance_range_mm"]:
            self.assertAlmostEqual(value, 0.13425624516932, places=10)
        self.assertAlmostEqual(
            coverage["right_unsupported_gap_mm"],
            0.02249624516932,
            places=10,
        )
        contract = self.report["existing_owner_contract"]
        self.assertEqual(
            contract["canonical_surface_to_wire_normal_active_local"],
            [1.0, 0.0, 0.0],
        )
        self.assertTrue(all(self.report["geometric_gates"].values()))

    def test_constructive_PEEK_landing_is_positive_tangent_and_gauge_clear(self):
        landing = self.report["constructive_PEEK_landing_witness"]
        self.assertEqual(landing["status"], "PASS")
        self.assertTrue(landing["not_integrated_into_cap_CAD"])
        self.assertAlmostEqual(
            landing["required_normal_protrusion_mm"],
            0.02249624516932,
            places=10,
        )
        self.assertEqual(landing["contact_normal_active_local"], [0.0, 1.0, 0.0])
        self.assertEqual(len(landing["cases"]), 2)
        for case in landing["cases"]:
            self.assertGreater(
                case["landing_to_selected_cap_positive_overlap_mm3"], 0.0)
            self.assertEqual(
                case["landing_to_nominal_wire_positive_overlap_mm3"], 0.0)
            self.assertAlmostEqual(
                case["landing_to_nominal_wire_distance_mm"], 0.0, places=8)
            self.assertEqual(
                case["landing_to_R0p36_insertion_gauge_positive_overlap_mm3"],
                0.0,
            )
            self.assertEqual(case["status"], "PASS")

    def test_nomex_and_release_authority_remain_fail_closed(self):
        nomex = self.report["nomex_assessment"]
        self.assertFalse(nomex["current_3D_BREP_owner_available"])
        self.assertEqual(nomex["selected_stock_nominal_thickness_mm"], 0.127)
        self.assertGreater(nomex["nominal_stock_to_gap_ratio"], 5.6)
        self.assertFalse(nomex["drop_in_flat_insert_valid"])
        self.assertEqual(self.report["status"], "FAIL")
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["wire_route_authorized"])
        self.assertFalse(self.report["collision_authorized"])
        self.assertFalse(self.report["assembly_integration_authorized"])
        self.assertFalse(self.report["selected_release_modified"])
        self.assertFalse(any(self.report["release_gates"].values()))

    def test_hash_binding_tamper_rejection_and_written_outputs(self):
        current = deepcopy(self.report)
        audit.validate_report_integrity(current)
        current["production_authorized"] = True
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            audit.validate_report_integrity(current)

        generated = audit.write_outputs(self.report)
        written = json.loads(audit.OUTPUT_JSON.read_text(encoding="utf-8"))
        self.assertEqual(generated["report_sha256"], written["report_sha256"])
        audit.validate_report_integrity(written)
        self.assertEqual(len(
            written["artifacts"]["terminal_loci"]["sha256"]
        ), 64)
        self.assertEqual(len(
            written["artifacts"]["permanent_cap_STEP"]["sha256"]
        ), 64)


if __name__ == "__main__":
    unittest.main(verbosity=2)
