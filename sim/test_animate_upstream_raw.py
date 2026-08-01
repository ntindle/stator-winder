"""Focused contracts for the watchable canonical raw-upstream player."""

from __future__ import annotations

import base64
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import unittest
from unittest import mock

from pygltflib import GLTF2


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from animate import (  # noqa: E402
    CONTINUOUS_CONDUCTOR_ROUTE,
    DEFAULT_CAPTURE,
    INTEGRATED_ADAPTER_MANIFEST_SCHEMA,
    INTEGRATED_CONDUCTOR_MODE,
    LEGACY_CONDUCTOR_MODE,
    LEGACY_RAW_GLB,
    LEGACY_RAW_HTML,
    PLAYER_SCHEMA,
    _load_player_slot_plan,
    _player_data,
    _raw_player_manifest_role,
    _raw_shaft_wraps,
    _raw_upstream_progress,
)
from traj import Timeline, load_events, winding_windows  # noqa: E402
from integrated_candidate_player_adapter import configured_animate  # noqa: E402


CAPTURE = ROOT / "out" / "capture" / "upstream_current_raw.jsonl"
PLAN = ROOT / "out" / "reports" / "slot_winding_plan.json"
PLAYER_ROOT = ROOT / "out" / "review" / "integrated_adapter"
MANIFEST = PLAYER_ROOT / "links" / "manifest.json"
LEGACY_MANIFEST = ROOT / "out" / "links" / "manifest.json"
CONTINUOUS_CONDUCTOR_ROUTE = (
    PLAYER_ROOT / "reports" / "continuous_conductor_route.json"
)
GLB = PLAYER_ROOT / "winding_cycle_integrated_candidate_raw.glb"
HTML = PLAYER_ROOT / "play_integrated_candidate_raw.html"
CAPTURE_SHA256 = (
    "350210c36550ae2c22e5352675aee6bd2eebb18325f3b5f45c7cb35d6a314958"
)


class RawUpstreamPlayerDataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.events = load_events(CAPTURE)
        cls.timeline = Timeline(cls.events)
        cls.manifest = json.loads(MANIFEST.read_text())
        cls.plan = _load_player_slot_plan(PLAN)
        with configured_animate(PLAYER_ROOT):
            cls.data = _player_data(
                cls.events,
                cls.timeline,
                cls.manifest,
                10.0,
                10.0,
                slot_plan=cls.plan,
                capture_path=CAPTURE,
                conductor_route_path=CONTINUOUS_CONDUCTOR_ROUTE,
            )

    def test_capture_is_canonical_unmodified_upstream_authority(self):
        self.assertEqual(DEFAULT_CAPTURE.resolve(), CAPTURE.resolve())
        self.assertEqual(hashlib.sha256(CAPTURE.read_bytes()).hexdigest(),
                         CAPTURE_SHA256)
        meta = self.timeline.meta
        self.assertEqual(meta["controller_mode"], "upstream")
        self.assertIsNone(meta["controller_adapter_sha256"])
        self.assertIsNone(meta["winding_plan"])
        self.assertEqual(self.data["captureMode"], "upstream_raw")
        self.assertEqual(self.data["captureSha256"], CAPTURE_SHA256)
        self.assertIn("unmodified upstream", self.data["captureAuthority"])

    def test_player_consumes_the_hash_bound_fail_closed_route(self):
        route = json.loads(
            CONTINUOUS_CONDUCTOR_ROUTE.read_text(encoding="utf-8")
        )
        self.assertEqual(self.data["conductorRoute"], route)
        self.assertEqual(
            self.data["conductorRouteSha256"], route["report_sha256"],
        )
        self.assertEqual(route["structural_status"], "PASS")
        self.assertEqual(route["status"], "FAIL")
        self.assertFalse(route["production_authorized"])
        self.assertTrue(route["timeline"]["no_hidden_live_interval"])
        self.assertEqual(
            self.data["conductorEvidence"]["mode"],
            INTEGRATED_CONDUCTOR_MODE,
        )
        self.assertTrue(self.data["conductorEvidence"]["playerGoverning"])
        self.assertFalse(
            self.data["conductorEvidence"]["continuousConductorAuthorized"]
        )

    def test_explicit_route_path_is_forwarded_without_weakening_binding(self):
        with configured_animate(PLAYER_ROOT):
            with mock.patch(
                "animate._load_continuous_conductor_route",
                wraps=sys.modules["animate"]._load_continuous_conductor_route,
            ) as loader:
                data = _player_data(
                    self.events,
                    self.timeline,
                    self.manifest,
                    10.0,
                    10.0,
                    slot_plan=self.plan,
                    capture_path=CAPTURE,
                    conductor_route_path=CONTINUOUS_CONDUCTOR_ROUTE,
                )
        loader.assert_called_once_with(
            CONTINUOUS_CONDUCTOR_ROUTE.resolve(),
            capture_path=CAPTURE,
            manifest_path=MANIFEST.resolve(),
        )
        self.assertEqual(
            data["conductorRoutePath"],
            str(CONTINUOUS_CONDUCTOR_ROUTE.resolve()),
        )
        self.assertEqual(
            data["conductorRouteArtifactSha256"],
            hashlib.sha256(CONTINUOUS_CONDUCTOR_ROUTE.read_bytes()).hexdigest(),
        )
        self.assertEqual(data["captureSha256"], CAPTURE_SHA256)

    def test_every_raw_command_for_all_four_axes_is_preserved(self):
        expected = [
            [event["t"], event["m"],
             event.get("model_target", event["a"]),
             event.get("controller_target"),
             event.get("command", "").strip()]
            for event in self.events if event["e"] == "cmd"
        ]
        self.assertEqual(self.data["commands"], expected)
        counts = Counter(command[1] for command in expected)
        self.assertEqual(counts, Counter({0: 2110, 2: 358, 3: 50, 1: 31}))
        self.assertEqual(
            self.data["commandCountsByAxis"],
            {str(axis): counts[axis] for axis in range(4)},
        )
        self.assertEqual(self.data["commandCount"], 2549)

    def test_each_pass_has_fifty_physical_timeline_turns(self):
        self.assertEqual(len(self.data["windings"]), 24)
        self.assertEqual(len(self.data["halfTurns"]), 24 * 100)
        self.assertEqual(len(self.data["depositions"]), 24 * 50)
        for pass_index, winding in enumerate(self.data["windings"]):
            halves = [row for row in self.data["halfTurns"]
                      if row["passIndex"] == pass_index]
            turns = [row for row in self.data["depositions"]
                     if row["passIndex"] == pass_index]
            self.assertEqual(len(halves), 100)
            self.assertEqual(len(turns), 50)
            self.assertEqual(winding["physicalTurnCount"], 50)
            self.assertEqual(winding["physicalHalfTurnCount"], 100)
            direction = 1 if winding["clockwise"] else -1
            for half in halves:
                start = self.timeline.axes[2].pos_at(half["start"])
                end = self.timeline.axes[2].pos_at(half["end"])
                self.assertAlmostEqual(
                    direction * (end - start), math.pi, places=4,
                )
                self.assertIn("Timeline", half["clockAuthority"])

    def test_raw_m0_radial_progression_is_distinct_from_visual_placement(self):
        summary = self.data["depositionSummary"]
        self.assertEqual(
            summary["rawRadialRangeMm"],
            [14.163553869, 20.678831162],
        )
        self.assertIn("raw M0 radial progression", summary["slotLegAuthority"])
        self.assertIn("not raw controller authority",
                      summary["slotLegAuthority"])
        self.assertEqual(
            self.data["slotPlan"]["presentationRole"],
            "diagnostic approximate elastic slot placement; not raw controller authority",
        )
        self.assertTrue(any(
            abs(row["rawActiveRadialMeanMm"]
                - self.plan["placements"][row["placementIndex"]]["activeRadial"])
            > 0.1
            for row in self.data["depositions"]
        ))
        for row in self.data["depositions"]:
            self.assertIn("raw M0 radial progression", row["geometryAuthority"])
            self.assertIn("approximate elastic", row["geometryAuthority"])

    def test_two_raw_shaft_intervals_come_from_actual_m1_commands(self):
        wraps = self.data["wraps"]
        self.assertEqual(len(wraps), 2)
        self.assertEqual([row["sourceCommand"] for row in wraps],
                         ["M1A-12.566", "M1A0.0"])
        self.assertEqual([row["turns"] for row in wraps],
                         [1.375, 2.791667])
        for wrap in wraps:
            self.assertLess(wrap["start"], wrap["end"])
            self.assertLessEqual(wrap["end"], wrap["markerDone"])
            self.assertAlmostEqual(
                self.timeline.axes[1].pos_at(wrap["start"]),
                wrap["startM1"], places=9,
            )
            self.assertAlmostEqual(
                self.timeline.axes[1].pos_at(wrap["end"]),
                wrap["endM1"], places=8,
            )
            self.assertIn("exact raw M1 command", wrap["intervalAuthority"])

    def test_twenty_four_coil_starts_are_explicit_physical_annotations(self):
        starts = self.data["coilStarts"]
        self.assertEqual(len(starts), 24)
        self.assertEqual([row["passIndex"] for row in starts], list(range(24)))
        for start in starts:
            self.assertEqual(start["turnClockAuthority"],
                             "Timeline physical M2 crossing")
            self.assertIn("first physical raw M2 turn",
                          start["markerAuthority"])
            self.assertTrue(start["continuousConductor"])
            self.assertFalse(start["cutOrJoin"])


class LegacyRawBaselinePlayerDataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.events = load_events(CAPTURE)
        cls.timeline = Timeline(cls.events)
        cls.manifest = json.loads(LEGACY_MANIFEST.read_text())
        cls.plan = _load_player_slot_plan(PLAN)
        with mock.patch(
            "animate._load_continuous_conductor_route"
        ) as route_loader, mock.patch(
            "animate.conductor_route._wire_handoff_contract"
        ) as handoff_loader:
            cls.data = _player_data(
                cls.events,
                cls.timeline,
                cls.manifest,
                10.0,
                10.0,
                slot_plan=cls.plan,
                capture_path=CAPTURE,
            )
            cls.route_loader_calls = route_loader.call_count
            cls.handoff_loader_calls = handoff_loader.call_count

    def test_schema_less_manifest_is_explicitly_legacy_not_integrated(self):
        self.assertNotIn("schema", self.manifest)
        self.assertEqual(
            _raw_player_manifest_role(self.manifest), LEGACY_CONDUCTOR_MODE
        )
        self.assertEqual(
            _raw_player_manifest_role({
                "schema": INTEGRATED_ADAPTER_MANIFEST_SCHEMA,
            }),
            INTEGRATED_CONDUCTOR_MODE,
        )
        with self.assertRaisesRegex(RuntimeError, "unsupported raw-player"):
            _raw_player_manifest_role({"schema": "unknown-player/v1"})

    def test_legacy_player_skips_integrated_only_route_loaders(self):
        self.assertEqual(self.route_loader_calls, 0)
        self.assertEqual(self.handoff_loader_calls, 0)
        self.assertIsNone(self.data["conductorRoute"])
        self.assertIsNone(self.data["conductorRoutePath"])
        self.assertIsNone(self.data["conductorRouteSha256"])
        self.assertIsNone(self.data["activeTerminalLoci"])
        self.assertIsNone(self.data["activeTerminalLociSha256"])
        self.assertIsNone(self.data["wireHandoff"])

    def test_legacy_conductor_evidence_and_coil_starts_fail_closed(self):
        evidence = self.data["conductorEvidence"]
        self.assertEqual(evidence["mode"], LEGACY_CONDUCTOR_MODE)
        self.assertFalse(evidence["playerGoverning"])
        self.assertFalse(evidence["continuousRouteAvailable"])
        self.assertFalse(evidence["activeTerminalLociAvailable"])
        self.assertFalse(evidence["continuousConductorAuthorized"])
        self.assertFalse(evidence["productionAuthorized"])
        self.assertIn("no continuous route", evidence["reason"])
        self.assertEqual(len(self.data["coilStarts"]), 24)
        for start in self.data["coilStarts"]:
            self.assertFalse(start["continuousConductor"])
            self.assertFalse(start["cutOrJoin"])
            self.assertIsNone(start["continuityFromPassIndex"])
            self.assertEqual(
                start["continuityStatus"], "UNAVAILABLE_UNPROVED"
            )
        self.assertIn(
            "no active-terminal-locus or continuous-route artifact",
            self.data["depositionSummary"]["endTurnAuthority"],
        )
        self.assertTrue(any(
            "disconnected approximate elastic review geometry" in limitation
            for limitation in self.data["limitations"]
        ))

    def test_legacy_output_names_and_browser_copy_are_unambiguous(self):
        self.assertEqual(
            LEGACY_RAW_GLB.resolve(),
            (ROOT / "out" / "winding_cycle_upstream_raw.glb").resolve(),
        )
        self.assertEqual(
            LEGACY_RAW_HTML.resolve(),
            (ROOT / "out" / "play_animation_upstream_raw.html").resolve(),
        )
        template = (HERE / "player_template.html").read_text(encoding="utf-8")
        self.assertIn("legacyRawBaseline", template)
        self.assertIn(
            "legacy_baseline_continuous_route_unavailable_unproved", template
        )
        self.assertIn("Disconnected approximate turns", template)
        self.assertIn("No flexible live-wire span", template)
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        self.assertIn(
            "sim/animate.py --capture out/capture/upstream_current_raw.jsonl "
            "--speed 10 --output out/winding_cycle_upstream_raw.glb "
            "--html out/play_animation_upstream_raw.html",
            readme,
        )
        self.assertIn("non-governing legacy baseline", readme)
        self.assertRegex(readme, r"makes no conductor-\s+continuity claim")


class GeneratedRawUpstreamPlayerTests(unittest.TestCase):
    def test_separate_player_embeds_raw_state_and_exact_glb(self):
        self.assertTrue(GLB.is_file())
        self.assertTrue(HTML.is_file())
        html = HTML.read_text(encoding="utf-8")
        state_match = re.search(
            r'const stateB64 = "([A-Za-z0-9+/=]+)";', html,
        )
        glb_match = re.search(
            r'const glbB64 = "([A-Za-z0-9+/=]+)";', html,
        )
        self.assertIsNotNone(state_match)
        self.assertIsNotNone(glb_match)
        state = json.loads(base64.b64decode(state_match.group(1)))
        self.assertEqual(state["schema"], PLAYER_SCHEMA)
        self.assertEqual(state["captureMode"], "upstream_raw")
        self.assertEqual(state["captureSha256"], CAPTURE_SHA256)
        self.assertEqual(len(state["commands"]), 2549)
        self.assertEqual(len(state["coilStarts"]), 24)
        self.assertEqual(len(state["depositions"]), 1200)
        self.assertEqual(len(state["wraps"]), 2)
        self.assertEqual(
            state["conductorRouteSha256"],
            state["conductorRoute"]["report_sha256"],
        )
        self.assertEqual(state["conductorRoute"]["structural_status"],
                         "PASS")
        self.assertEqual(state["conductorRoute"]["status"], "FAIL")
        self.assertEqual(base64.b64decode(glb_match.group(1)), GLB.read_bytes())
        self.assertEqual(state["glbSha256"],
                         hashlib.sha256(GLB.read_bytes()).hexdigest())

    def test_player_exposes_command_and_authority_controls(self):
        html = HTML.read_text(encoding="utf-8")
        for element_id in (
            "command-position", "command-summary", "coil-build-label",
            "prev-command", "next-command", "m0-controller",
            "m1-controller", "m2-controller", "m3-controller",
            "prev-coil", "next-coil", "hold-coil-starts",
            "focus-coil-starts", "coil-navigation-label",
        ):
            self.assertIn(f'id="{element_id}"', html)
        self.assertIn("raw upstream serial", html)
        self.assertIn("approximate elastic slot placement", html)
        self.assertIn("not raw controller authority", html)
        self.assertIn("physical raw M2 turns", html)
        self.assertIn("function conductorTailAt", html)
        self.assertIn("function createContinuousRoute", html)
        self.assertIn("phase_aware_ordered_continuous_conductor", html)
        self.assertIn("dashed_red", html)
        self.assertIn("function stepCoil", html)
        self.assertIn("function crossedUnheldCoilStart", html)
        self.assertIn("COIL_START_HOLD_MS = 1000", html)
        self.assertIn("COIL_START_SLOW_FACTOR = 0.25", html)
        self.assertIn("new URLSearchParams(location.search)", html)
        self.assertIn("reviewParams.get('coil')", html)
        self.assertIn("reviewParams.get('autoplay') !== '0'", html)
        self.assertIn("seek(reviewStartTime)", html)
        self.assertIn("GOAL contract FAIL", html)
        self.assertIn("required 2.000, actual", html)
        self.assertIn("shaftWrapToleranceTurns", html)
        legacy_branch = html.index(
            "if (legacyRawBaseline) {", html.index("function updateWire")
        )
        legacy_hide = html.index("if ((!winding && !wrap) || (winding && !halfTurn))")
        self.assertLess(legacy_branch, legacy_hide)

    def test_raw_glb_retains_three_physical_motion_channels(self):
        model = GLTF2().load_binary(GLB)
        self.assertEqual(len(model.animations), 1)
        self.assertEqual(len(model.animations[0].channels), 3)
        self.assertTrue(
            all(
                primitive.attributes.NORMAL is not None
                for mesh in model.meshes
                for primitive in mesh.primitives
            ),
            "every lit PBR primitive must carry vertex normals",
        )
        self.assertEqual(
            {node.name for node in model.nodes},
            {"static", "carriage", "spindle_pivot", "spindle", "flyer",
             "wire_static", "wire_flyer"}
            | set(json.loads(MANIFEST.read_text())["visual_groups"]),
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
