"""Fail-closed contracts for the winding-tooling authority aggregator."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import winding_tooling_authority as authority


class Fixture:
    def __init__(self, base: Path):
        self.root = base / "robotics" / "machine"
        self.reports = self.root / "out" / "reports"
        self.capture = self.root / "out" / "capture" / "upstream_current_raw.jsonl"
        self.reports.mkdir(parents=True)
        self.capture.parent.mkdir(parents=True)
        self.capture.write_text(json.dumps({
            "t": 0.0,
            "e": "meta",
            "capture_schema": 4,
            "controller_mode": "upstream",
            "controller_adapter_sha256": None,
            "winding_plan": None,
            "winder_commit": "a" * 40,
        }) + "\n", encoding="utf-8")
        self._source("out/reports/slot_packing.json")
        self._source("out/reports/slot_wire_routes.json")
        self._source("out/settings.yml")
        self._source("sim/traj.py")
        self._source("sim/elastic_wire_contact_study.py")
        goal = self.root.parent / "GOAL.md"
        goal.parent.mkdir(parents=True, exist_ok=True)
        goal.write_text("goal\n", encoding="utf-8")

    def _source(self, relative: str, value: str = "source\n") -> Path:
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(value, encoding="utf-8")
        return path

    def _write_report(self, name: str, payload: dict) -> Path:
        body = dict(payload)
        body["report_sha256"] = authority._canonical_hash(body)
        path = self.reports / name
        path.write_text(json.dumps(body, indent=2) + "\n", encoding="utf-8")
        return path

    def elastic(self, status: str, *, production_authorized: bool = False) -> Path:
        sources = {
            "raw_capture_sha256": authority._sha256(self.capture),
            "packing_file_sha256": authority._sha256(
                self.root / "out/reports/slot_packing.json"
            ),
            "slot_wire_routes_file_sha256": authority._sha256(
                self.root / "out/reports/slot_wire_routes.json"
            ),
            "settings_sha256": authority._sha256(self.root / "out/settings.yml"),
            "goal_sha256": authority._sha256(self.root.parent / "GOAL.md"),
            "traj_source_sha256": authority._sha256(self.root / "sim/traj.py"),
            "study_source_sha256": authority._sha256(
                self.root / "sim/elastic_wire_contact_study.py"
            ),
        }
        return self._write_report("elastic_wire_contact_study.json", {
            "schema": "elastic-wire-contact-study/v1",
            "status": status,
            "decision": "fixture",
            "production_authorized": production_authorized,
            "source_hashes": sources,
        })

    def study(
        self,
        name: str,
        *,
        status: str,
        production_authorized: bool,
    ) -> Path:
        source = self._source(f"sim/{Path(name).stem}.py")
        bindings = {
            "out/capture/upstream_current_raw.jsonl": authority._sha256(
                self.capture
            ),
            source.relative_to(self.root).as_posix(): authority._sha256(source),
        }
        return self._write_report(name, {
            "schema": f"{Path(name).stem.replace('_', '-')}-study/v1",
            "status": status,
            "production_authorized": production_authorized,
            "release_authorized": production_authorized,
            "assembly_integration_authorized": production_authorized,
            "decision": "fixture",
            "source_hashes": bindings,
        })


class WindingToolingAuthorityTests(unittest.TestCase):
    def test_r3_route_family_nouns_are_inventoried(self):
        for name, schema in (
            ("r3_dogleg_end_basket.json", "r3-dogleg-end-basket-study/v1"),
            ("r3_sector_chord_family.json", "r3-sector-chord-family-study/v1"),
            ("m0_follower_shroud.json", "m0-follower-shroud-study/v1"),
            (
                "aggregate_progressive_wire_corridor.json",
                "aggregate-progressive-wire-corridor/v1",
            ),
            (
                "permanent_cap_aggregate_authorization.json",
                "permanent-cap-aggregate-authorization/v1",
            ),
            (
                "permanent_cap_offset_spoke_flyer.json",
                "permanent-cap-offset-spoke-flyer/v1",
            ),
        ):
            with self.subTest(name=name):
                self.assertTrue(authority._is_architecture_study(
                    Path(name), {"schema": schema},
                ))

    def test_permanent_support_recovery_supersedes_older_dogleg_context(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = Fixture(Path(temporary))
            fixture.elastic("FAIL")
            fixture.study(
                "r3_dogleg_end_basket.json",
                status="DESIGN_NO_GO",
                production_authorized=False,
            )
            newest = fixture.study(
                "permanent_cap_flyer_recovery.json",
                status="DESIGN_NO_GO",
                production_authorized=False,
            )
            result = authority.evaluate(fixture.root)

        self.assertEqual(
            result["surviving_lane"]["report"],
            newest.relative_to(fixture.root).as_posix(),
        )
        self.assertIn("not a survival", result["surviving_lane"]["statement"])

    def test_goal_binding_resolves_only_to_parent_goal(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = Fixture(Path(temporary))
            goal = fixture.root.parent / "GOAL.md"
            ok, mismatches = authority._verify_path_bindings(
                fixture.root, {"GOAL.md": authority._sha256(goal)}
            )
            escaped, escaped_mismatches = authority._verify_path_bindings(
                fixture.root, {"../not-goal.md": "0" * 64}
            )

        self.assertTrue(ok)
        self.assertEqual(mismatches, [])
        self.assertFalse(escaped)
        self.assertTrue(escaped_mismatches)

    def test_no_go_studies_fail_with_advisory_development_lane(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = Fixture(Path(temporary))
            fixture.elastic("FAIL")
            selector = fixture.study(
                "m1_indexed_selector_former.json",
                status="DESIGN_CHANGE_REQUIRED",
                production_authorized=False,
            )
            result = authority.evaluate(fixture.root)

        self.assertEqual(result["schema"], authority.SCHEMA)
        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(result["production_authorized"])
        self.assertIsNone(result["selected_production_candidate"])
        self.assertEqual(
            result["surviving_lane"]["report"],
            selector.relative_to(fixture.root).as_posix(),
        )
        self.assertEqual(
            result["surviving_lane"]["id"],
            "goal-bound-r3-contact-corridor",
        )
        self.assertIn("not a survival", result["surviving_lane"]["statement"])
        self.assertEqual(len(result["architecture_studies"]), 1)

    def test_active_candidate_can_supersede_current_fixed_flyer_no_go(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = Fixture(Path(temporary))
            fixture.elastic("FAIL")
            selected = fixture.study(
                "m1_indexed_selector_former.json",
                status="PASS",
                production_authorized=True,
            )
            result = authority.evaluate(fixture.root)

        self.assertEqual(result["status"], "PASS")
        self.assertTrue(result["production_authorized"])
        self.assertEqual(
            result["selected_production_candidate"],
            selected.relative_to(fixture.root).as_posix(),
        )
        self.assertEqual(result["release_blockers"], [])

    def test_explicitly_authorized_fixed_flyer_can_be_the_candidate(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = Fixture(Path(temporary))
            selected = fixture.elastic("PASS", production_authorized=True)
            result = authority.evaluate(fixture.root)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(
            result["selected_production_candidate"],
            selected.relative_to(fixture.root).as_posix(),
        )

    def test_pass_without_production_authority_or_current_sources_fails(self):
        for production_authorized, tamper in ((False, False), (True, True)):
            with self.subTest(
                production_authorized=production_authorized, tamper=tamper,
            ), tempfile.TemporaryDirectory() as temporary:
                fixture = Fixture(Path(temporary))
                fixture.elastic("PASS")
                fixture.study(
                    "candidate_former.json",
                    status="PASS",
                    production_authorized=production_authorized,
                )
                if tamper:
                    fixture._source("sim/candidate_former.py", "changed\n")
                result = authority.evaluate(fixture.root)

            self.assertEqual(result["status"], "FAIL")
            self.assertFalse(result["production_authorized"])
            self.assertIsNone(result["selected_production_candidate"])


if __name__ == "__main__":
    unittest.main()
