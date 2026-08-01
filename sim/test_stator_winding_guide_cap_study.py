"""Fail-closed contract checks for the permanent guide-cap study."""

import json
import math
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
CAD = HERE.parent / "cad"
for path in (HERE, CAD):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import stator_winding_guide_cap_study as study


class StatorWindingGuideCapStudyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _packing, cls.graph = study._load_graph()

    def test_exact_packing_forces_outboard_recovery_and_large_cap(self):
        bounds = study.geometry_boundaries(self.graph)
        planar = bounds["planar_horn_boundary"]
        outboard = bounds["outboard_recovery"]
        self.assertEqual(planar["failing_turn_count"], 50)
        self.assertTrue(math.isclose(
            planar["actual_minimum_span_mm"], 3.69752, abs_tol=1e-9))
        self.assertLess(planar["actual_maximum_span_mm"], 6.0)
        self.assertGreater(outboard["selected_base_margin_mm"], 0.0)
        self.assertGreater(
            outboard["minimum_required_cap_outer_diameter_mm"], 86.0)
        self.assertEqual(bounds["open_mouth"]["status"], "PASS")

    def test_all_turn_paths_are_R3_and_nonself_but_report_fails_other_copper(self):
        radial_min = min(turn.radial_mm for turn in self.graph.turns)
        profile_min = min(turn.profile_radius_mm for turn in self.graph.turns)
        routes = [study.route_for_turn(
            turn, radial_min_mm=radial_min, profile_min_mm=profile_min
        ) for turn in self.graph.turns]
        self.assertEqual(len(routes), 50)
        self.assertTrue(all(
            route.minimum_analytic_bend_radius_mm >= 3.0
            for route in routes))
        self.assertTrue(all(route.simple_non_self_looping for route in routes))

        report = json.loads(study.JSON_OUT.read_text())
        self.assertEqual(report["status"], "DESIGN_NO_GO")
        self.assertEqual(report["route_audit"]["status"], "FAIL")
        gates = report["route_audit"]["gates"]
        self.assertFalse(gates["prior_nonparent_copper"])
        self.assertFalse(gates["both_neighbor_teeth"])
        self.assertEqual(
            report["rigid_envelope"]["flyer"]["status"], "FAIL")
        self.assertEqual(
            report["rigid_envelope"]["flyer"]["sample_count"], 50 * 360)


if __name__ == "__main__":
    unittest.main()
