"""Source-level tests for the slot-fill and final-coil envelope model."""

from __future__ import annotations

import math
import unittest

from build123d import Align, Cylinder, Plane, Polygon, extrude

from params import StatorSpec
import coil_growth
import stator_model


class SlotGeometryTests(unittest.TestCase):
    def test_slot_area_matches_generated_geometry_regression(self):
        expected = {
            28.0: 2.522617575145,
            36.0: 7.984477318872,
            46.0: 13.036384264454,
            65.0: 26.029642494006,
            90.0: 49.902983242948,
        }
        for od, area in expected.items():
            with self.subTest(od=od):
                actual = coil_growth.slot_geometry(StatorSpec(od=od))[
                    "geometric_slot_area_mm2"
                ]
                self.assertAlmostEqual(actual, area, places=9)

    def test_analytic_slot_area_matches_independent_brep_boolean(self):
        spec = StatorSpec(od=46)
        radius = spec.od / 2.0
        hub_radius = spec.od * spec.hub_od_ratio / 2.0
        pitch = 2.0 * math.pi / spec.slots
        align = (Align.CENTER, Align.CENTER, Align.CENTER)
        annulus = Cylinder(radius, spec.stack, align=align) - Cylinder(
            hub_radius, spec.stack + 2.0, align=align
        )
        big = spec.od * 2.0
        wedge_2d = Polygon(
            (0, 0),
            (big, 0),
            (big * math.cos(pitch), big * math.sin(pitch)),
            align=None,
        )
        wedge = extrude(Plane.XY * wedge_2d, amount=spec.stack / 2.0, both=True)
        sector = annulus & wedge
        lamination = max(
            stator_model.stator(spec).solids(), key=lambda solid: solid.volume
        )
        brep_slot_area = (sector - lamination).volume / spec.stack
        analytic = coil_growth.slot_geometry(spec)["geometric_slot_area_mm2"]
        self.assertAlmostEqual(analytic, brep_slot_area, places=8)

    def test_slot_area_is_independent_of_stack_and_shaft(self):
        areas = {
            round(
                coil_growth.slot_geometry(
                    StatorSpec(od=46, stack=stack, shaft_d=shaft)
                )["geometric_slot_area_mm2"],
                12,
            )
            for stack in (5.0, 20.0)
            for shaft in (3.0, 8.0)
        }
        self.assertEqual(len(areas), 1)

    def test_smallest_slot_opening_does_not_imply_deep_slot_access(self):
        result = coil_growth.analyze_job(
            StatorSpec(od=28, wire_d=0.5, turns=1)
        )
        self.assertTrue(result["slot_opening"]["ok"])
        self.assertGreater(result["slot_opening"]["margin_mm"], 0.10)
        self.assertFalse(result["slot_access"]["ok"])
        self.assertEqual(result["status"], "FAIL")

    def test_wire_access_excludes_closed_root_and_shoe_throat(self):
        result = coil_growth.analyze_job(StatorSpec())
        access = result["slot_access"]
        self.assertTrue(access["ok"])
        self.assertAlmostEqual(
            access["wire_accessible_start_radius_mm"],
            14.163900505756,
            places=9,
        )
        self.assertAlmostEqual(
            access["wire_accessible_end_radius_mm"], 20.68, places=9)
        self.assertAlmostEqual(
            access["accessible_slot_area_mm2"], 8.707377358219, places=9)
        self.assertLess(
            access["accessible_slot_area_mm2"],
            result["slot"]["geometric_slot_area_mm2"],
        )
        self.assertEqual(
            result["bundle"]["radial_winding_start_mm"],
            access["wire_accessible_start_radius_mm"],
        )


class CapacityTests(unittest.TestCase):
    def test_legacy_default_is_rejected_and_corrected_default_passes(self):
        result = coil_growth.analyze_job(StatorSpec(wire_d=0.30))
        self.assertEqual(result["status"], "FAIL")
        self.assertAlmostEqual(
            result["packing"]["gross_slot_fill"], 0.826238, places=6)
        self.assertEqual(result["packing"]["max_turns_at_maximum_fill"], 36)
        with self.assertRaisesRegex(ValueError, "infeasible winding job"):
            coil_growth.require_feasible(StatorSpec(wire_d=0.30))
        corrected = coil_growth.analyze_job(StatorSpec())
        self.assertEqual(corrected["status"], "PASS")
        self.assertLessEqual(corrected["packing"]["gross_slot_fill"], 0.55)

    def test_od46_wire_candidates_are_classified(self):
        pass_job = coil_growth.analyze_job(StatorSpec(wire_d=0.24))
        marginal = coil_growth.analyze_job(StatorSpec(wire_d=0.25))
        self.assertEqual(pass_job["status"], "PASS")
        self.assertEqual(marginal["status"], "MARGINAL")
        self.assertLessEqual(pass_job["packing"]["gross_slot_fill"], 0.55)
        self.assertGreater(marginal["packing"]["gross_slot_fill"], 0.55)
        self.assertLessEqual(marginal["packing"]["gross_slot_fill"], 0.60)

    def test_launch_envelope_is_not_universal_75_turn_capacity(self):
        self.assertEqual(
            coil_growth.analyze_job(
                StatorSpec(od=28, wire_d=0.20, turns=75)
            )["status"],
            "FAIL",
        )
        self.assertEqual(
            coil_growth.analyze_job(
                StatorSpec(od=65, wire_d=0.30, turns=75)
            )["status"],
            "MARGINAL",
        )
        self.assertEqual(
            coil_growth.analyze_job(
                StatorSpec(od=65, wire_d=0.40, turns=75)
            )["status"],
            "FAIL",
        )

    def test_old_default_75_turn_job_is_rejected_by_accessible_area(self):
        result = coil_growth.analyze_job(StatorSpec(turns=75))
        self.assertEqual(result["status"], "FAIL")
        self.assertGreater(result["packing"]["gross_slot_fill"], 0.60)
        self.assertEqual(result["packing"]["max_turns_at_design_fill"], 61)
        self.assertEqual(result["packing"]["max_turns_at_maximum_fill"], 66)

    def test_capacity_scales_monotonically(self):
        last_area = 0.0
        last_capacity = 0
        for od in range(28, 66):
            result = coil_growth.analyze_job(
                StatorSpec(od=float(od), wire_d=0.25)
            )
            area = result["slot"]["geometric_slot_area_mm2"]
            capacity = result["packing"]["max_turns_at_maximum_fill"]
            self.assertGreater(area, last_area)
            self.assertGreaterEqual(capacity, last_capacity)
            last_area = area
            last_capacity = capacity


class EnvelopeTests(unittest.TestCase):
    def setUp(self):
        self.spec = StatorSpec(wire_d=0.24)

    def test_collision_envelopes_are_closed_positive_solids(self):
        result = coil_growth.analyze_job(self.spec)
        growth = result["bundle"]["collision_growth_mm"]
        bodies = coil_growth.coil_collision_envelopes(self.spec)
        self.assertEqual(len(bodies), self.spec.slots)
        self.assertTrue(all(body.volume > 0 for body in bodies))
        first_box = bodies[0].bounding_box().size
        self.assertAlmostEqual(first_box.Z, self.spec.stack + 2 * growth, places=8)

    def test_stack_changes_envelope_height_not_fill(self):
        low = StatorSpec(stack=5, wire_d=0.24)
        high = StatorSpec(stack=20, wire_d=0.24)
        low_result = coil_growth.analyze_job(low)
        high_result = coil_growth.analyze_job(high)
        self.assertAlmostEqual(
            low_result["packing"]["gross_slot_fill"],
            high_result["packing"]["gross_slot_fill"],
            places=12,
        )
        low_z = coil_growth.coil_collision_envelopes(low)[0].bounding_box().size.Z
        high_z = coil_growth.coil_collision_envelopes(high)[0].bounding_box().size.Z
        self.assertAlmostEqual(high_z - low_z, 15.0, places=8)

    def test_visual_shells_are_coil_only_and_positive(self):
        shells = coil_growth.coil_bundle_shells(self.spec)
        self.assertEqual(len(shells), self.spec.slots)
        self.assertTrue(all(shell.volume > 0 for shell in shells))

    def test_infeasible_envelope_requires_explicit_override(self):
        with self.assertRaisesRegex(ValueError, "refusing envelope"):
            coil_growth.coil_collision_envelopes(StatorSpec(wire_d=0.30))
        bodies = coil_growth.coil_collision_envelopes(
            StatorSpec(wire_d=0.30), allow_infeasible=True
        )
        self.assertEqual(len(bodies), 24)


class ReportTests(unittest.TestCase):
    def test_report_records_recommendations_and_endpoint_evidence(self):
        report = coil_growth.generate_report()
        self.assertFalse(report["launch_envelope"]["all_75_turn_combinations_supported"])
        self.assertEqual(report["current_default"]["status"], "PASS")
        self.assertEqual(report["legacy_default_0_30_mm"]["status"], "FAIL")
        self.assertEqual(
            report["legacy_default_0_24_mm_75_turns"]["status"], "FAIL"
        )
        self.assertEqual(report["candidate_default_0_24_mm"]["status"], "PASS")
        self.assertEqual(report["candidate_default_0_25_mm"]["status"], "MARGINAL")
        self.assertEqual(len(report["stack_shaft_endpoint_invariance"]), 4)
        self.assertGreaterEqual(len(report["recommendations"]), 4)


if __name__ == "__main__":
    unittest.main(verbosity=2)
