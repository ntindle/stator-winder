"""Regression tests for the isolated 3D turn-45 elastic route study."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest

import numpy as np


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from elastic_3d_turn45_route_study import (  # noqa: E402
    ELASTIC_CONTACT,
    MINIMUM_BEND_RADIUS_MM,
    OUTPUT_JSON,
    ROUTES,
    SCHEMA,
    _canonical_hash,
    _parent_endpoint_centers,
    _sha256,
    _shallow_bow_bounds,
    _source_target_tangent,
    _turn45_cases,
    elastic_contact,
    r3_multiarc_route,
    validate_report_integrity,
)
from params import DEFAULT_STATOR  # noqa: E402
from slot_route import PackingSupportGraph  # noqa: E402


class Elastic3DTurn45StudyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = json.loads(OUTPUT_JSON.read_text(encoding="utf-8"))
        cls.route_report = json.loads(ROUTES.read_text(encoding="utf-8"))

    def test_report_is_hash_bound_and_fail_closed(self):
        self.assertEqual(self.report["schema"], SCHEMA)
        payload = dict(self.report)
        claimed = payload.pop("report_sha256")
        self.assertEqual(claimed, _canonical_hash(payload))
        validate_report_integrity(self.report)
        self.assertEqual(self.report["status"], "FAIL")
        self.assertIs(self.report["production_authorized"], False)
        self.assertIs(self.report["scope"][
            "alternate_raw_compatible_wire_distribution_rejected"], False)
        contact = json.loads(ELASTIC_CONTACT.read_text(encoding="utf-8"))
        source_hashes = self.report["source_hashes"]
        self.assertEqual(
            source_hashes["elastic_contact_file_sha256"],
            _sha256(ELASTIC_CONTACT),
        )
        self.assertEqual(
            source_hashes["elastic_contact_report_sha256"],
            contact["report_sha256"],
        )
        self.assertEqual(
            source_hashes["elastic_contact_source_sha256"],
            _sha256(Path(elastic_contact.__file__)),
        )

    def test_turn45_selection_is_independent_of_stored_status(self):
        route_report = json.loads(ROUTES.read_text(encoding="utf-8"))
        turn45 = [
            row for row in route_report["routes"]
            if row["turn_index"] == 45
        ]
        self.assertEqual(len(turn45), 2)
        turn45[0]["status"] = "PASS"
        turn45[1]["status"] = "FAIL"
        selected = _turn45_cases(route_report)
        self.assertEqual(
            [row["half_turn_index"] for row in selected], [0, 1],
        )
        self.assertEqual(
            {row["status"] for row in selected}, {"PASS", "FAIL"})

    def test_terminal_source_tracks_trailing_end_plane_run(self):
        row = _turn45_cases(self.route_report)[0]
        points = np.asarray(row["route"]["points_local_mm"], dtype=float)
        source, target, incoming = _source_target_tangent(row)
        end_plane_z = float(DEFAULT_STATOR.stack) / 2.0
        source_index = len(points) - 1
        while (source_index > 0
               and abs(points[source_index - 1, 2] - end_plane_z) <= 1e-9):
            source_index -= 1
        self.assertGreater(source_index, 0)
        self.assertLess(source_index, len(points) - 1)
        self.assertTrue(np.allclose(source, points[source_index]))
        self.assertTrue(np.allclose(target, points[-1]))
        self.assertTrue(np.allclose(
            incoming,
            (source - points[source_index - 1])
            / np.linalg.norm(source - points[source_index - 1]),
        ))

    def test_current_turn45_r3_chain_has_positive_bridge(self):
        row = _turn45_cases(self.route_report)[0]
        packing = json.loads(
            (OUTPUT_JSON.parent / "slot_packing.json").read_text(
                encoding="utf-8"))
        graph = PackingSupportGraph.from_report(
            packing, spec=DEFAULT_STATOR)
        route, metadata = r3_multiarc_route(row, graph)
        self.assertGreater(metadata["vertical_bridge_length_mm"], 0.0)
        self.assertAlmostEqual(
            metadata["source_local_mm"][2],
            float(DEFAULT_STATOR.stack) / 2.0,
        )
        self.assertGreaterEqual(
            metadata["minimum_bend_radius_mm"], MINIMUM_BEND_RADIUS_MM)
        self.assertLessEqual(metadata["endpoint_error_mm"], 1e-8)
        self.assertTrue(np.allclose(
            route[0], metadata["source_local_mm"], atol=1e-12, rtol=0.0))

    def test_zero_amplitude_shallow_bound_is_strict_json_diagnostic(self):
        row = _turn45_cases(self.route_report)[0]
        packing = json.loads(
            (OUTPUT_JSON.parent / "slot_packing.json").read_text(
                encoding="utf-8"))
        graph = PackingSupportGraph.from_report(
            packing, spec=DEFAULT_STATOR)
        source, target, _ = _source_target_tangent(row)
        parent_center = _parent_endpoint_centers(
            graph, row["half_turn_index"])[44]
        bow = _shallow_bow_bounds(source, target, parent_center)
        self.assertEqual(bow["terminal_safe_minimum_amplitude_mm"], 0.0)
        self.assertIsNone(bow[
            "radius_at_terminal_safe_amplitude_mm"])
        self.assertTrue(bow[
            "radius_at_terminal_safe_amplitude_unbounded"])
        self.assertFalse(bow["qualifies_as_full_C1_route"])
        json.dumps(bow, allow_nan=False)

    def test_both_turn45_loci_are_exact_mirrors(self):
        self.assertEqual(len(self.report["cases"]), 2)
        first, second = self.report["cases"]
        mirror = np.array((1.0, -1.0, -1.0))
        a = np.asarray(first["analytic_R3_multiarc"][
            "route_points_local_mm"])
        b = np.asarray(second["analytic_R3_multiarc"][
            "route_points_local_mm"])
        self.assertTrue(np.allclose(a * mirror, b, atol=1e-10, rtol=0.0))
        self.assertLessEqual(
            self.report["geometry_contract"]["mirror_max_abs_error_mm"],
            1e-10)

    def test_biarcs_reject_R3_and_shallow_bow_stays_diagnostic(self):
        for case in self.report["cases"]:
            self.assertTrue(all(
                branch["minimum_radius_mm"] < MINIMUM_BEND_RADIUS_MM
                for branch in case["standard_biarc"]["branches"]))
            bow = case["shallow_normal_bow"]
            self.assertEqual(bow["status"], "DIAGNOSTIC_ONLY")
            self.assertFalse(bow["endpoint_tangent_contract_evaluated"])
            self.assertFalse(bow["qualifies_as_full_C1_route"])

    def test_explicit_R3_curve_clears_steel_but_not_progressive_copper(self):
        for case in self.report["cases"]:
            route = case["analytic_R3_multiarc"]
            audit = route["audit"]
            self.assertGreaterEqual(
                route["minimum_bend_radius_mm"], MINIMUM_BEND_RADIUS_MM)
            self.assertTrue(audit["checks"]["exact_OCC_core_lower_bound"])
            self.assertFalse(audit["checks"][
                "all_prior_nonparent_copper_lower_bound"])
            self.assertFalse(audit["checks"][
                "declared_parent_prefix_no_penetration"])

    def test_each_locus_has_both_motion_signs(self):
        for case in self.report["cases"]:
            signs = {row["motion_sign"] for row in case[
                "analytic_R3_multiarc"]["audit"]["motion_sign_cases"]}
            self.assertEqual(signs, {-1, 1})

    def test_convergence_is_monotone_in_error_bound(self):
        for case in self.report["cases"]:
            rows = case["convergence"]
            errors = [row["route_chord_error_bound_mm"] for row in rows]
            points = [row["point_count"] for row in rows]
            self.assertEqual(len(rows), 4)
            self.assertTrue(all(a > b for a, b in zip(errors, errors[1:])))
            self.assertTrue(all(a < b for a, b in zip(points, points[1:])))
            self.assertTrue(all(row["endpoint_error_mm"] <= 1e-8
                                for row in rows))

    def test_free_equilibrium_requires_a_guide(self):
        equilibrium = self.report["free_equilibrium_assessment"]
        self.assertEqual(equilibrium["status"], "FAIL")
        self.assertFalse(equilibrium["plausible_free_elastic_equilibrium"])
        self.assertTrue(equilibrium["guide_or_former_required"])
        self.assertFalse(self.report["release_flags"][
            "physical_former_geometry_bound"])


if __name__ == "__main__":
    unittest.main()
