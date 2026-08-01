from __future__ import annotations

from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import r3_sector_chord_family_study as study


class SectorChordFamilyStudyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = study.analyze()

    def test_exact_default_and_endpoints(self) -> None:
        self.assertEqual(self.report["inputs"]["turns_per_tooth"], 50)
        self.assertEqual(self.report["inputs"]["shared_slot_endpoint_count"],
                         100)
        self.assertTrue(self.report["checks"]
                        ["all_100_shared_slot_endpoints_preserved"])

    def test_piecewise_family_is_literal_r3_and_contained(self) -> None:
        family = self.report["analytic_family"]
        self.assertEqual(family["minimum_analytic_radius_mm"], 3.0)
        self.assertTrue(family["C1_tangent_continuity"])
        self.assertTrue(self.report["checks"]
                        ["all_centrelines_sector_and_OD_contained"])
        self.assertTrue(self.report["checks"]
                        ["adjacent_tooth_separation_proved_by_inset_halfplanes"])
        self.assertAlmostEqual(
            self.report["checks"]
            ["adjacent_tooth_analytic_centerline_lower_bound_mm"],
            self.report["inputs"]["wire_finished_diameter_mm"], places=12)

    def test_turn_24_witness_is_explicit(self) -> None:
        witness = self.report["turn_24_witness"]
        self.assertEqual(witness["turn_index"], 24)
        self.assertAlmostEqual(
            witness["waypoint"]["distance_from_outgoing_endpoint_mm"],
            6.0, places=10)
        self.assertGreaterEqual(witness["minimum_sampled_od_center_margin_mm"],
                                -1.0e-9)

    def test_every_bounded_candidate_has_direct_collision_witness(self) -> None:
        required = self.report["inputs"]["wire_finished_diameter_mm"]
        self.assertEqual(self.report["status"], "DESIGN_NO_GO")
        for candidate in self.report["bounded_phase_search"]:
            self.assertTrue(candidate["direct_sampled_collision_witness"])
            self.assertLess(
                candidate["same_tooth"]["minimum_sampled_distance_mm"],
                required)
            self.assertGreaterEqual(
                candidate["adjacent_tooth"]["minimum_sampled_distance_mm"],
                required)
            self.assertIsNotNone(candidate["same_tooth"]["witness"])
            self.assertIsNotNone(candidate["adjacent_tooth"]["witness"])

    def test_report_is_fail_closed(self) -> None:
        self.assertFalse(self.report["production_authorized"])
        self.assertFalse(self.report["integration_authorized"])
        self.assertFalse(self.report["checks"]
                         ["same_tooth_and_neighbor_clearance_proved"])
        self.assertGreater(int(self.report["report_sha256"], 16), 0)


if __name__ == "__main__":
    unittest.main()
