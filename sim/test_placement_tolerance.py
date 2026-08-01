"""Fail-closed tests for the physical placement tolerance gate."""

from __future__ import annotations

from copy import deepcopy
import unittest

from placement_tolerance import (
    PACKING_M0_SETTLE_TOLERANCE_RAD,
    PhysicalEvidence,
    RouteMargins,
    _canonical_hash,
    evaluate_budget,
    extract_noncontact_route_margins,
    validate_report,
)


def _complete_evidence(value: float = 0.0) -> PhysicalEvidence:
    return PhysicalEvidence(
        m0_observed_max_error_rad=value,
        m0_observation_source="hardware capture fixture A",
        m0_carriage_physical_error_mm=value,
        m0_carriage_source="dial indicator sweep fixture A",
        spindle_tir_mm=value,
        spindle_tir_source="spindle indicator sweep fixture A",
        mounted_stator_tir_mm=value,
        mounted_stator_tir_source="mounted stator indicator sweep fixture A",
        wire_diameter_instrument_uncertainty_mm=value,
        wire_measurement_source="micrometer calibration record A",
        liner_thickness_instrument_uncertainty_mm=value,
        liner_measurement_source="micrometer calibration record A",
        contact_position_uncertainty_mm=value,
        contact_uncertainty_source="instrumented winding coupon A",
    )


class PlacementToleranceTests(unittest.TestCase):
    def test_unknown_physical_inputs_are_not_proven(self):
        result = evaluate_budget(
            RouteMargins(1.0, 1.0, "synthetic copper", "synthetic core"),
            PhysicalEvidence(),
            packing_is_measured=True,
            upstream_statuses={
                "packing": "PASS", "plan": "PASS", "routes": "PASS"},
        )
        self.assertEqual(result["status"], "NOT_PROVEN")
        self.assertTrue(result["unknowns"])
        self.assertFalse(result["budget"]["copper_complete"])

    def test_tiny_nonparent_margin_fails_on_known_axis_terms_alone(self):
        # This is the current route margin order of magnitude.  No optimistic
        # zero for unknown hardware can save it: controller acceptance plus
        # half-count quantization already exceed the nominal space.
        result = evaluate_budget(
            RouteMargins(
                0.000278846, 1.0,
                "synthetic current-route copper", "synthetic core"),
            PhysicalEvidence(),
            packing_is_measured=True,
            upstream_statuses={
                "packing": "PASS", "plan": "PASS", "routes": "PASS"},
        )
        self.assertEqual(result["status"], "FAIL")
        self.assertGreater(
            result["budget"]["copper_known_lower_bound_mm"],
            0.000278846,
        )
        self.assertIn(
            "nonparent copper margin does not exceed the known physical "
            "uncertainty lower bound",
            result["hard_failures"],
        )

    def test_complete_zero_uncertainty_fixture_can_pass_generous_margins(self):
        result = evaluate_budget(
            RouteMargins(0.1, 0.1, "synthetic copper", "synthetic core"),
            _complete_evidence(),
            packing_is_measured=True,
            upstream_statuses={
                "packing": "PASS", "plan": "PASS", "routes": "PASS"},
        )
        self.assertEqual(result["status"], "PASS")
        self.assertFalse(result["unknowns"])
        self.assertFalse(result["hard_failures"])

    def test_observed_error_over_controller_gate_is_fail(self):
        evidence = _complete_evidence()
        evidence = PhysicalEvidence(
            **{
                **evidence.__dict__,
                "m0_observed_max_error_rad": (
                    PACKING_M0_SETTLE_TOLERANCE_RAD + 0.001),
            }
        )
        result = evaluate_budget(
            RouteMargins(1.0, 1.0, "synthetic copper", "synthetic core"),
            evidence,
            packing_is_measured=True,
            upstream_statuses={
                "packing": "PASS", "plan": "PASS", "routes": "PASS"},
        )
        self.assertEqual(result["status"], "FAIL")
        self.assertIn(
            "observed M0 error exceeds controller settle tolerance",
            result["hard_failures"],
        )

    def test_only_explicit_noncontact_margins_are_consumed(self):
        report = {
            "input_contract": {
                "required_copper_center_distance_mm": 0.22352,
            },
            "validation": {
                # These generic fields include intended zero-gap contacts and
                # must never become release margin.
                "minimum_copper_margin_mm": 0.0,
                "minimum_core_margin_mm": 0.0,
            },
            "routes": [{
                "planner_metadata": {"exact_release_postcheck": {
                    "nonparent_raw_chordal_distance_mm": 0.22452,
                }},
            }],
        }
        margins = extract_noncontact_route_margins(report)
        self.assertAlmostEqual(
            margins.copper_nonparent_margin_mm or -1.0, 0.001, places=12)
        self.assertIsNone(margins.core_noncontact_margin_mm)

    def test_explicit_negative_margin_is_a_failure_not_unknown(self):
        report = {
            "validation": {
                "minimum_nonparent_copper_margin_mm": -0.001,
                "minimum_noncontact_core_margin_mm": 0.1,
            },
        }
        margins = extract_noncontact_route_margins(report)
        self.assertEqual(margins.copper_nonparent_margin_mm, -0.001)
        result = evaluate_budget(
            margins,
            _complete_evidence(),
            packing_is_measured=True,
            upstream_statuses={
                "packing": "PASS", "plan": "PASS", "routes": "PASS"},
        )
        self.assertEqual(result["status"], "FAIL")

    def test_report_hash_tampering_fails_closed(self):
        report = {
            "schema": "placement-route-tolerance/v1",
            "status": "NOT_PROVEN",
            "hard_failures": [],
        }
        report["report_sha256"] = _canonical_hash(report)
        validate_report(report)
        tampered = deepcopy(report)
        tampered["status"] = "PASS"
        with self.assertRaisesRegex(ValueError, "report_sha256 mismatch"):
            validate_report(tampered)


if __name__ == "__main__":
    unittest.main()
