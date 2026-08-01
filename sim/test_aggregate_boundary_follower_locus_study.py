"""Focused tests for the exact-locus aggregate follower study."""

from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import aggregate_boundary_follower_locus_study as study


class AggregateBoundaryFollowerLocusStudyTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.report = study.analyze()

    def test_current_report_is_exactly_2400_and_fail_closed(self):
        report = self.report
        self.assertEqual(report["status"], "FAIL")
        self.assertFalse(report["production_authorized"])
        self.assertEqual(report["coverage"]["evaluated_loci"], 2400)
        self.assertEqual(
            report["coverage"]["zero_growth_loci_missing_physical_normal"],
            48,
        )
        self.assertEqual(
            report["coverage"]["nonzero_growth_support_tangent_count"],
            2352,
        )

    def test_slide_envelope_covers_selected_nonzero_contacts(self):
        bounds = self.report["travel_and_turn_bounds"]
        self.assertLessEqual(bounds["required_radial_travel_mm"], 6.0)
        self.assertLessEqual(
            bounds["required_tangential_travel_per_identity_mm"], 1.0,
        )
        self.assertTrue(self.report["gates"][
            "prototype_radial_stroke_covers_selected_contacts"
        ])
        self.assertTrue(self.report["gates"][
            "prototype_tangential_stroke_per_identity_covers_contacts"
        ])

    def test_direct_span_never_hides_required_R3_turn(self):
        coverage = self.report["coverage"]
        bounds = self.report["travel_and_turn_bounds"]
        self.assertEqual(coverage["nonzero_growth_direct_C1_count"], 0)
        self.assertGreater(bounds["minimum_direct_span_C1_error_deg"], 10.0)
        self.assertLess(bounds["maximum_direct_span_C1_error_deg"], 65.0)
        self.assertFalse(self.report["gates"][
            "positive_volume_R3_arc_placement_proven"
        ])

    def test_triangle_sublevel_is_exact_equal_area(self):
        geometry = self.report["geometry"]
        u0 = geometry["u_start_mm"]
        uc = geometry["u_cutoff_mm"]
        wc = geometry["cutoff_half_width_mm"]
        aggregate = json.loads(study.AGGREGATE_PATH.read_text(encoding="utf-8"))
        area50 = aggregate["slot_partition"][
            "required_50_turn_copper_area_mm2"
        ]
        for turn in (1, 7, 25, 49):
            g = turn / 50.0
            ug = u0 + math.sqrt(g) * (uc - u0)
            wg = math.sqrt(g) * wc
            self.assertTrue(math.isclose(
                0.5 * (ug - u0) * wg,
                g * area50,
                abs_tol=2.0e-12,
            ))

    def test_integrity_rejects_tampering(self):
        current = deepcopy(self.report)
        study.validate_report_integrity(current)
        current["gates"]["positive_volume_R3_arc_placement_proven"] = True
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            study.validate_report_integrity(current)

    def test_written_report_is_current(self):
        generated = study.write_outputs(self.report)
        checked = json.loads(study.OUTPUT_JSON.read_text(encoding="utf-8"))
        self.assertEqual(generated["report_sha256"], checked["report_sha256"])
        study.validate_report_integrity(checked)


if __name__ == "__main__":
    unittest.main(verbosity=2)
