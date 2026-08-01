"""Tests for the advisory R3 bend-scope witness."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

import numpy as np


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import r3_bend_scope_feasibility as study


class R3BendScopeFeasibilityTests(unittest.TestCase):
    def test_square_rows_have_exact_count_and_shared_slot_clearance(self):
        rows = study.square_row_centres()
        self.assertEqual(len(rows), 50)
        self.assertEqual([sum(row["layer_index"] == layer for row in rows)
                          for layer in range(4)], [24, 16, 9, 1])
        self.assertGreaterEqual(
            study.minimum_pair_distance(study.shared_slot_centres()) + 1e-10,
            study.DEFAULT_STATOR.wire_d,
        )

    def test_lrl_endpoint_identity_and_exact_outer_R3(self):
        points, tangents = study.sample_base_lrl()
        p = study.lrl_parameters()
        self.assertTrue(np.allclose(
            points[0], (-p["base_half_span_mm"], 0.0), atol=1e-10,
        ))
        self.assertTrue(np.allclose(
            points[-1], (p["base_half_span_mm"], 0.0), atol=1e-10,
        ))
        self.assertTrue(np.allclose(tangents[0], (0.0, 1.0), atol=1e-10))
        self.assertTrue(np.allclose(tangents[-1], (0.0, -1.0), atol=1e-10))
        self.assertAlmostEqual(
            p["base_radius_mm"] - p["maximum_offset_mm"], 3.0, places=12,
        )

    def test_parallel_offset_endpoints_match_square_row_layers(self):
        rows = study.square_row_centres()
        p = study.lrl_parameters()
        for layer in range(4):
            cap = study.offset_cap(layer)
            expected = p["base_half_span_mm"] + layer * p["offset_step_mm"]
            self.assertAlmostEqual(float(cap[0, 0]), -expected, places=9)
            self.assertAlmostEqual(float(cap[-1, 0]), expected, places=9)
            row_values = {
                round(float(row["tooth_half_span_mm"]), 9)
                for row in rows if row["layer_index"] == layer
            }
            self.assertEqual(row_values, {round(expected, 9)})

    def test_report_is_advisory_and_fails_closed_on_missing_global_proof(self):
        report = study.analyze()
        self.assertEqual(report["status"], "ADVISORY_COMPATIBLE")
        self.assertFalse(report["production_authorized"])
        self.assertEqual(
            report["adjacent_tooth_pitch"]["status"],
            "REQUIRES_3D_OR_AXIAL_STAGGERING",
        )
        self.assertEqual(
            report["bend_scope_audit"]["final_motor_fit"]["axial_status"],
            "UNPROVEN",
        )
        self.assertLess(
            report["bend_scope_audit"]["tight_insulated_workpiece_conformity"]
            ["buffered_sharp_corner_centerline_radius_mm"],
            3.0,
        )
        self.assertTrue(all(item["ok"] for item in report["checks"].values()))

    def test_naive_semicircle_is_not_mistaken_for_global_contradiction(self):
        report = study.analyze()
        self.assertEqual(report["naive_rounded_end_turn"]["status"], "FAIL")
        self.assertLess(
            max(report["naive_rounded_end_turn"]["radius_range_mm"]), 3.0,
        )
        self.assertEqual(
            report["decision"],
            "NO_LITERAL_DEFAULT_TOOTH_GEOMETRIC_CONTRADICTION",
        )


if __name__ == "__main__":
    unittest.main()
