"""Tests for the capture-bound continuous moving-wire release gate."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import continuous_wire_audit as audit


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical(payload: dict) -> str:
    return hashlib.sha256(json.dumps(
        payload, sort_keys=True, separators=(",", ":"),
        allow_nan=False).encode()).hexdigest()


def _write_hashed(path: Path, payload: dict, field: str) -> dict:
    payload = dict(payload)
    payload[field] = _canonical(payload)
    path.write_text(json.dumps(payload, indent=2) + "\n")
    return payload


def _completed_hash(turn: int) -> str:
    return hashlib.sha256(json.dumps(
        list(range(turn)), separators=(",", ":")).encode()).hexdigest()


class Fixture:
    def __init__(self, root: Path, provenance: str = "UNIVERSALLY_BOUNDED"):
        self.root = root
        (root / "sim").mkdir()
        self.source = root / "source.txt"
        self.source.write_text("fixture source\n")
        self.adapter = root / "sim" / "controller_adapter.py"
        self.adapter.write_text("# fixture adapter\n")
        self.proof = root / "proof.txt"
        self.proof.write_text("universal swept-contact proof\n")
        source_hashes = {"source.txt": _sha(self.source)}

        self.packing_path = root / "packing.json"
        self.packing = _write_hashed(self.packing_path, {
            "schema": "slot-packing/v2",
            "status": "PASS",
            "source_hashes": source_hashes,
        }, "report_sha256")

        self.plan_path = root / "plan.json"
        self.plan = _write_hashed(self.plan_path, {
            "schema": "slot-winding-plan/v1",
            "source_hashes": source_hashes,
            "packing_report": {
                "report_sha256": self.packing["report_sha256"],
            },
            "job": {
                "slots": 1,
                "turns_per_tooth": 1,
                "wire_finished_d_mm": 0.2,
                "od_mm": 2.0,
                "stack_mm": 1.0,
            },
            "selected_case": {"status": "PASS"},
        }, "proof_sha256")

        rows = []
        for half in (0, 1):
            rows.append({
                "turn_index": 0,
                "half_turn_index": half,
                "logical_phase_rad": half * 3.141592653589793,
                "validated_motion_signs": [-1, 1],
                "status": "PASS",
                "progressive_support_validated": True,
                "route": {
                    "points_local_mm": [[0.0, 0.0, 0.0],
                                        [1.0, 0.0, 0.0]],
                    "segment_tags": ["free"],
                },
                "postcheck": {
                    "minimum_core_center_distance_mm": 1.0,
                    "required_core_center_distance_mm": 0.1,
                    "minimum_copper_center_distance_mm": 1.0,
                    "required_copper_center_distance_mm": 0.2,
                },
            })
        self.routes_path = root / "routes.json"
        self.routes = _write_hashed(self.routes_path, {
            "schema": "slot-wire-routes/v1",
            "status": "PASS",
            "source_hashes": source_hashes,
            "scope": {
                "prior_copper_rule": (
                    "completed active turns and neighbor prefills; crossing "
                    "table alone does not prove current conductor"),
            },
            "input_contract": {
                "packing_report_sha256": self.packing["report_sha256"],
                "packing_file_sha256": _sha(self.packing_path),
            },
            "routes": rows,
        }, "report_sha256")

        self.capture_path = root / "commands.jsonl"
        self.events = self._events(provenance)
        self.write_capture()
        self.contact_path = root / "contact.json"
        self.contact = self._contact(provenance)
        self.write_contact()

    def _events(self, provenance: str) -> list[dict]:
        events = [{
            "t": 0.0,
            "e": "meta",
            "capture_schema": 3,
            "controller_mode": "contract",
            "controller_adapter_sha256": _sha(self.adapter),
            "velocities": [1.0, 1.0, 3.141592653589793, 1.0],
            "turns": 1,
            "teeth_count": 1,
            "job": {
                "slots": 1,
                "turns_per_tooth": 1,
                "wire_finished_d_mm": 0.2,
                "od_mm": 2.0,
                "stack_mm": 1.0,
            },
            "winding_plan": {
                "sha256": _sha(self.plan_path),
                "proof_sha256": self.plan["proof_sha256"],
            },
        }, {
            "t": 0.0, "e": "wind_wire", "args": [0, False, 0],
        }, {
            "t": 0.0, "e": "cmd", "m": 0, "model_target": 0.0,
            "a": 0.0,
        }, {
            "t": 0.0, "e": "cmd", "m": 1, "model_target": 0.0,
            "a": 0.0,
        }, {
            "t": 0.0, "e": "packing_pass_origin", "pass_index": 0,
            "start_phase_rad": 0.0,
            "first_crossing_phase_rad": 0.0,
            "phase_origin_rad": 0.0,
            "actual_travel_rad": 2.0 * 3.141592653589793,
        }, self._waypoint(0, 0.0, "placement_center", 0.0), {
            "t": 0.0, "e": "cmd", "m": 2,
            "model_target": 3.141592653589793,
            "a": 3.141592653589793,
        }, self._waypoint(1, 1.0, "placement_center",
                          3.141592653589793), {
            "t": 1.0, "e": "cmd", "m": 2,
            "model_target": 2.0 * 3.141592653589793,
            "a": 2.0 * 3.141592653589793,
        }, self._waypoint(2, 2.0, "final_hold",
                          2.0 * 3.141592653589793), {
            "t": 2.0, "e": "wind_wire_done", "m2state": "BOTTOM",
        }, {
            "t": 2.0, "e": "cycle_complete",
        }]
        if provenance == "OBSERVED":
            observations = [
                {"t": 0.0, "e": "wire_contact_observation",
                 "observation_id": "o0", "homotopy_tag": "slot-loop",
                 "minimum_noncontact_clearance_mm": 10.0},
                {"t": 1.0, "e": "wire_contact_observation",
                 "observation_id": "o1", "homotopy_tag": "slot-loop",
                 "minimum_noncontact_clearance_mm": 10.0},
                {"t": 2.0, "e": "wire_contact_observation",
                 "observation_id": "o2", "homotopy_tag": "slot-loop",
                 "minimum_noncontact_clearance_mm": 10.0},
            ]
            events.extend(observations)
        return events

    @staticmethod
    def _waypoint(index: int, t: float, kind: str, phase: float) -> dict:
        return {
            "t": t,
            "e": "packing_waypoint",
            "pass_index": 0,
            "waypoint_index": index,
            "m2_phase_rad": phase,
            "observed_m2_phase_rad": phase,
            "m0_target_rad": 0.0,
            "observed_m0_rad": 0.0,
            "m0_error_rad": 0.0,
            "m0_settled_before_crossing": True,
            "placement_index": min(index // 2, 0),
            "kind": kind,
        }

    def write_capture(self) -> None:
        self.capture_path.write_text("".join(
            json.dumps(event) + "\n" for event in self.events))

    def _contact(self, provenance: str) -> dict:
        cases = []
        for half in (0, 1):
            progressive = {
                "completed_turn_count": 0,
                "completed_turn_indices_sha256": _completed_hash(0),
                "active_turn_index": 0,
                "already_laid_current_half_turns": half,
                "all_prior_completed_turns_included": True,
                "current_conductor_exclusion": "adjacent_segment_only",
                "adjacent_self_exclusion_length_mm": 0.2,
                "neighbor_prefill_sides": [-1, 1],
            }
            case = {
                "case_id": f"packing-0-{half}",
                "kind": "packing",
                "pass_index": 0,
                "half_turn_index": half,
                "start_t_s": float(half),
                "end_t_s": float(half + 1),
                "provenance": provenance,
                "homotopy_tag": "slot-loop",
                "minimum_noncontact_clearance_mm": 10.0,
                "contact_uncertainty_mm": 0.01,
                "maximum_flyer_radius_mm": 1.0,
                "maximum_stator_radius_mm": 1.1,
                "progressive_copper": progressive,
            }
            if provenance == "UNIVERSALLY_BOUNDED":
                case.update({
                    "proof_id": "fixture",
                    "proof_method": "analytic fixture clearance envelope",
                })
            else:
                case["observation_ids"] = [f"o{half}", f"o{half + 1}"]
            cases.append(case)
        if provenance == "UNIVERSALLY_BOUNDED":
            _write_hashed(self.proof, {
                "schema": audit.UNIVERSAL_PROOF_SCHEMA,
                "status": "PASS",
                "source_hashes": {"source.txt": _sha(self.source)},
                "case_sha256": {
                    case["case_id"]: audit._universal_case_hash(case)
                    for case in cases
                },
            }, "report_sha256")
        contact = {
            "schema": audit.CONTACT_SCHEMA,
            "status": "PASS",
            "provenance": provenance,
            "source_hashes": {"source.txt": _sha(self.source)},
            "capture_file_sha256": _sha(self.capture_path),
            "plan_file_sha256": _sha(self.plan_path),
            "plan_proof_sha256": self.plan["proof_sha256"],
            "packing_report_sha256": self.packing["report_sha256"],
            "route_file_sha256": _sha(self.routes_path),
            "route_report_sha256": self.routes["report_sha256"],
            "wire_live_window_s": [0.0, 2.0],
            "cases": cases,
        }
        if provenance == "UNIVERSALLY_BOUNDED":
            contact["proof_artifacts"] = {
                "fixture": {"path": "proof.txt", "sha256": _sha(self.proof)},
            }
        return contact

    def write_contact(self) -> None:
        self.contact = dict(self.contact)
        self.contact.pop("report_sha256", None)
        self.contact["report_sha256"] = _canonical(self.contact)
        self.contact_path.write_text(json.dumps(self.contact, indent=2) + "\n")

    def rewrite_universal_proof(self) -> None:
        _write_hashed(self.proof, {
            "schema": audit.UNIVERSAL_PROOF_SCHEMA,
            "status": "PASS",
            "source_hashes": {"source.txt": _sha(self.source)},
            "case_sha256": {
                case["case_id"]: audit._universal_case_hash(case)
                for case in self.contact["cases"]
            },
        }, "report_sha256")
        self.contact["proof_artifacts"]["fixture"]["sha256"] = _sha(
            self.proof)

    def analyze(self, *, contact: bool = True) -> dict:
        return audit.analyze(
            self.capture_path, self.plan_path, self.packing_path,
            self.routes_path, self.contact_path if contact else None,
            source_root=self.root,
            config=audit.AuditConfig(max_element_motion_mm=100.0,
                                     default_flyer_radius_mm=1.0))


class ContinuousWireAuditTests(unittest.TestCase):
    def fixture(self, provenance: str = "UNIVERSALLY_BOUNDED"):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        return Fixture(Path(temporary.name), provenance)

    def test_universal_contact_proof_passes_every_interval(self):
        fixture = self.fixture()
        report = fixture.analyze()
        self.assertEqual(report["status"], "PASS", report["issues"])
        self.assertEqual(report["coverage"][
            "expected_packing_halfturn_cases"], 2)
        self.assertEqual(report["coverage"][
            "partitioned_packing_halfturn_cases"], 2)
        self.assertEqual(report["contact_evidence"]["passed_cases"], 2)
        self.assertEqual(report["contact_evidence"][
            "provenance"], "UNIVERSALLY_BOUNDED")
        self.assertGreaterEqual(report["partition"][
            "boundary_reason_counts"]["command"], 2)
        self.assertGreaterEqual(report["partition"][
            "boundary_reason_counts"]["arrival"], 1)
        self.assertGreaterEqual(report["partition"][
            "boundary_reason_counts"]["packing_halfturn_root"], 3)

    def test_observed_contact_samples_are_consumed_from_capture(self):
        fixture = self.fixture("OBSERVED")
        report = fixture.analyze()
        self.assertEqual(report["status"], "PASS", report["issues"])
        self.assertEqual(report["contact_evidence"][
            "observed_capture_sample_count"], 3)
        self.assertEqual(report["contact_evidence"][
            "provenance"], "OBSERVED")

    def test_commands_without_contact_profile_are_not_proven(self):
        fixture = self.fixture()
        report = fixture.analyze(contact=False)
        self.assertEqual(report["status"], "NOT_PROVEN")
        self.assertEqual(report["contact_evidence"][
            "not_proven_cases"], 2)
        self.assertIn("contact_provenance_missing",
                      {issue["code"] for issue in report["issues"]})
        self.assertTrue(all(case["status"] == "NOT_PROVEN"
                            for case in report["interval_cases"]))

    def test_offset_first_crossing_and_leadout_closure_are_partitioned(self):
        fixture = self.fixture()
        pi = 3.141592653589793
        first_t = pi - 1.0
        second_t = first_t + pi
        closure_t = second_t + pi
        meta = fixture.events[0]
        meta["velocities"] = [1.0, 1.0, 1.0, 1.0]
        fixture.events = [meta, {
            "t": 0.0, "e": "wind_wire", "args": [0, False, 0],
        }, {
            "t": 0.0, "e": "cmd", "m": 0,
            "model_target": 0.0, "a": 0.0,
        }, {
            "t": 0.0, "e": "cmd", "m": 1,
            "model_target": 0.0, "a": 0.0,
        }, {
            "t": 0.0, "e": "packing_pass_origin", "pass_index": 0,
            "start_phase_rad": 1.0,
            "first_crossing_phase_rad": pi,
            "phase_origin_rad": pi,
            "actual_travel_rad": 3.0 * pi,
        }, {
            "t": 0.0, "e": "cmd", "m": 2,
            "model_target": pi - 1.0, "a": pi - 1.0,
        }, Fixture._waypoint(0, first_t, "placement_center", pi), {
            "t": first_t, "e": "cmd", "m": 2,
            "model_target": 2.0 * pi - 1.0, "a": 2.0 * pi - 1.0,
        }, Fixture._waypoint(1, second_t, "placement_center", 2.0 * pi), {
            "t": second_t, "e": "cmd", "m": 2,
            "model_target": 3.0 * pi - 1.0, "a": 3.0 * pi - 1.0,
        }, Fixture._waypoint(2, closure_t, "final_hold", 3.0 * pi), {
            "t": closure_t, "e": "wind_wire_done", "m2state": "BOTTOM",
        }, {
            "t": closure_t, "e": "cycle_complete",
        }]
        fixture.write_capture()
        report = fixture.analyze(contact=False)
        self.assertEqual(report["status"], "NOT_PROVEN", report["issues"])
        self.assertEqual(report["coverage"][
            "partitioned_packing_halfturn_cases"], 2)
        self.assertEqual(report["packing_passes"][0]["start_phase_rad"], 1.0)
        self.assertAlmostEqual(report["packing_passes"][0][
            "phase_origin_rad"], pi)
        self.assertFalse(any(issue["severity"] == "FAIL"
                             for issue in report["issues"]), report["issues"])

    def test_stale_capture_plan_binding_fails_closed(self):
        fixture = self.fixture()
        fixture.events[0]["winding_plan"]["sha256"] = "0" * 64
        fixture.write_capture()
        report = fixture.analyze(contact=False)
        self.assertEqual(report["status"], "FAIL")
        self.assertIn("capture_plan_file_binding_mismatch",
                      {issue["code"] for issue in report["issues"]})

    def test_waypoint_cannot_be_accepted_before_actual_m2_phase(self):
        fixture = self.fixture()
        waypoint = next(event for event in fixture.events
                        if event.get("e") == "packing_waypoint"
                        and event["waypoint_index"] == 1)
        waypoint["t"] = 0.5
        fixture.write_capture()
        report = fixture.analyze(contact=False)
        self.assertEqual(report["status"], "FAIL")
        self.assertIn("packing_early_phase_acceptance",
                      {issue["code"] for issue in report["issues"]})

    def test_last_center_without_closing_halfturn_is_not_a_complete_coil(self):
        fixture = self.fixture()
        origin = next(event for event in fixture.events
                      if event.get("e") == "packing_pass_origin")
        origin["actual_travel_rad"] = 3.141592653589793
        fixture.events = [event for event in fixture.events
                          if not (event.get("e") == "packing_waypoint"
                                  and event.get("waypoint_index") == 2)
                          and not (event.get("e") == "cmd"
                                   and event.get("m") == 2
                                   and event.get("t") == 1.0)]
        for event in fixture.events:
            if event.get("e") in ("wind_wire_done", "cycle_complete"):
                event["t"] = 1.0
        fixture.write_capture()
        report = fixture.analyze(contact=False)
        self.assertEqual(report["status"], "FAIL")
        self.assertIn("packing_closure_crossing_missing",
                      {issue["code"] for issue in report["issues"]})

    def test_already_laid_current_half_must_be_in_progressive_state(self):
        fixture = self.fixture()
        fixture.contact["cases"][1]["progressive_copper"][
            "already_laid_current_half_turns"] = 0
        fixture.write_contact()
        report = fixture.analyze()
        self.assertEqual(report["status"], "FAIL")
        failed = [case for case in report["interval_cases"]
                  if case["status"] == "FAIL"]
        self.assertTrue(any("already_laid_current_half_turns"
                            in case["reason"] for case in failed))

    def test_motion_bound_consumes_clearance_and_fails(self):
        fixture = self.fixture()
        for case in fixture.contact["cases"]:
            case["minimum_noncontact_clearance_mm"] = 1.0
        fixture.rewrite_universal_proof()
        fixture.write_contact()
        report = fixture.analyze()
        self.assertEqual(report["status"], "FAIL")
        self.assertEqual(report["contact_evidence"]["failed_cases"], 2)
        self.assertTrue(all(
            case.get("minimum_swept_margin_mm", 0.0) < 0.0
            for case in report["interval_cases"]))

    def test_stale_universal_proof_artifact_fails(self):
        fixture = self.fixture()
        fixture.proof.write_text("changed after contact certificate\n")
        report = fixture.analyze()
        self.assertEqual(report["status"], "FAIL")
        self.assertIn("contact_proof_artifact_stale",
                      {issue["code"] for issue in report["issues"]})

    def test_report_hash_binds_complete_result(self):
        fixture = self.fixture()
        report = fixture.analyze()
        expected = report["report_sha256"]
        payload = dict(report)
        payload.pop("report_sha256")
        self.assertEqual(expected, _canonical(payload))


if __name__ == "__main__":
    unittest.main()
