"""Focused contracts for the plan-bound progressive browser player."""

from __future__ import annotations

import base64
from copy import deepcopy
import json
import math
import re
import tempfile
import unittest
from pathlib import Path

from pygltflib import GLTF2

from animate import (
    PLAYER_SCHEMA,
    _coil_start_events,
    _load_player_slot_plan,
    _m0_target_for_radial,
    _packing_progress,
    _player_data,
    _validate_player_artifact_identity,
    _visual_group_asset_records,
)
from traj import Timeline, load_events


ROOT = Path(__file__).resolve().parent.parent
PLAN_PATH = ROOT / "out" / "reports" / "slot_winding_plan.json"
INTEGRATED_PLAYER_ROOT = ROOT / "out" / "review" / "integrated_adapter"


def identity_meta(plan):
    return {
        "turns": plan["job"]["turnsPerTooth"],
        "teeth_count": plan["job"]["slots"],
        "job": {
            "slots": plan["job"]["slots"],
            "od_mm": plan["job"]["od"],
            "stack_mm": plan["job"]["stack"],
            "wire_finished_d_mm": plan["job"]["wireFinishedDiameter"],
            "liner_max_thickness_mm": plan["job"]["linerMaximumThickness"],
            "radial_winding_span_mm": [14.32662646626053, 20.68],
        },
        "m0_wind_range": [-61.79, -56.8],
        "winding_plan": {
            "schema": plan["schema"],
            "path": plan["path"],
            "sha256": plan["artifactSha256"],
            "proof_sha256": plan["proofSha256"],
            "transition_status": plan["transitionStatus"],
            "nominal_wire_mm": plan["job"]["wireFinishedDiameter"],
            "model_wire_envelope_mm": plan["job"]["modelWireEnvelope"],
            "receiving_sensitivity_wire_envelope_mm": (
                plan["receivingSensitivity"]["wireEnvelope"]),
            "receiving_sensitivity_status": (
                plan["receivingSensitivity"]["status"]),
            "turns_per_tooth": plan["job"]["turnsPerTooth"],
            "placement_count": plan["placementCount"],
            "half_turn_center_count": plan["halfTurnCenterCount"],
        },
    }


def identity_manifest(plan):
    return {
        "stator": {
            "slots": plan["job"]["slots"],
            "od": plan["job"]["od"],
            "stack": plan["job"]["stack"],
            "wire_d": plan["job"]["wireFinishedDiameter"],
            "turns": plan["job"]["turnsPerTooth"],
        },
    }


class SlotPlanPresentationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan = _load_player_slot_plan(PLAN_PATH)

    def test_plan_retains_all_exact_two_sided_centers(self):
        plan = self.plan
        self.assertEqual(plan["schema"], "slot-winding-plan/v1")
        self.assertEqual(plan["transitionStatus"], "PASS")
        self.assertEqual(plan["placementCount"], 50)
        self.assertEqual(plan["halfTurnCenterCount"], 100)
        self.assertEqual(plan["job"]["wireFinishedDiameter"], 0.22352)
        self.assertEqual(plan["job"]["modelWireEnvelope"], 0.22352)
        self.assertEqual(plan["receivingSensitivity"]["status"], "PASS")
        for index, placement in enumerate(plan["placements"]):
            with self.subTest(turn=index):
                self.assertEqual(placement["turn"], index)
                self.assertLess(placement["leftActiveTangential"], 0.0)
                self.assertAlmostEqual(
                    placement["leftActiveTangential"],
                    -placement["rightActiveTangential"], places=12,
                )
                self.assertAlmostEqual(
                    placement["leftSlotCenter"][0], placement["radial"],
                    places=12,
                )
                self.assertAlmostEqual(
                    placement["rightSlotCenter"][0], placement["radial"],
                    places=12,
                )
                self.assertTrue(placement["laterNeighborMouthConnected"])
        self.assertEqual(plan["placements"][0]["contact"], "slot_liner")
        self.assertEqual(plan["placements"][0]["support"], "slot_liner")
        self.assertEqual(plan["placements"][0]["supportPredecessors"], [])
        self.assertTrue(plan["placements"][-1]["supportPredecessors"])

    def test_plan_proof_hash_tampering_fails_closed(self):
        raw = json.loads(PLAN_PATH.read_text(encoding="utf-8"))
        raw["placements"][0]["active_tooth_tangential_mm"] -= 0.001
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tampered.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "proof_sha256"):
                _load_player_slot_plan(path)

    def test_capture_and_manifest_identity_mismatch_fails_closed(self):
        meta = identity_meta(self.plan)
        manifest = identity_manifest(self.plan)
        _validate_player_artifact_identity(meta, manifest, self.plan)

        bad_meta = deepcopy(meta)
        bad_meta["winding_plan"]["sha256"] = "0" * 64
        with self.assertRaisesRegex(RuntimeError, "sha256"):
            _validate_player_artifact_identity(bad_meta, manifest, self.plan)

        bad_manifest = deepcopy(manifest)
        bad_manifest["stator"]["turns"] = 75
        with self.assertRaisesRegex(RuntimeError, "stator.turns"):
            _validate_player_artifact_identity(meta, bad_manifest, self.plan)

    def test_special_visual_groups_are_exact_and_base_mesh_excluded(self):
        manifest = json.loads(
            (ROOT / "out" / "links" / "manifest.json").read_text()
        )
        records = _visual_group_asset_records(manifest)
        self.assertEqual(
            [record["name"] for record in records], ["felt_pads"]
        )
        self.assertTrue(all(record["path"].is_file() for record in records))
        self.assertIn("flyer_arm", manifest["parts"]["flyer"])
        bad = deepcopy(manifest)
        bad["visual_groups"]["felt_pads"][
            "excluded_from_base_link_mesh"
        ] = False
        with self.assertRaisesRegex(RuntimeError, "not excluded"):
            _visual_group_asset_records(bad)

    def test_waypoint_clock_exposes_every_turn_side_contact_and_support(self):
        meta = identity_meta(self.plan)
        winding = {
            "start": 0.0,
            "motionStart": 0.0,
            "positionedAt": 0.0,
            "end": 11.0,
            "tooth": 0,
            "clockwise": False,
            "passIndex": 0,
            "phase": 0,
        }
        turns = self.plan["job"]["turnsPerTooth"]
        events = [{
            "t": 0.0,
            "e": "packing_pass_origin",
            "pass_index": 0,
            "start_phase_rad": 0.0,
            "first_crossing_phase_rad": 0.0,
            "phase_origin_rad": 0.0,
            "actual_travel_rad": 2 * turns * math.pi,
            "final_hold_phase_rad": 2 * turns * math.pi,
            "expected_deposition_center_count": 2 * turns,
            "placement_zero_settled_before_first_crossing": True,
            "pre_crossing_deposition_count": 0,
        }]
        for index in range(2 * turns):
            placement = self.plan["placements"][min(index // 2, turns - 1)]
            target = placement["m0Target"]
            events.append({
                "t": index * 0.1,
                "e": "packing_waypoint",
                "pass_index": 0,
                "waypoint_index": index,
                "m2_phase_rad": index * math.pi,
                "observed_m2_phase_rad": index * math.pi,
                "m0_target_rad": target,
                "observed_m0_rad": target,
                "m0_error_rad": 0.0,
                "placement_index": index // 2,
                "kind": "placement_center",
                "m0_ready_phase_rad": index * math.pi,
                "m0_settled_before_crossing": True,
            })
        placement = self.plan["placements"][-1]
        target = placement["m0Target"]
        events.append({
            "t": 10.0,
            "e": "packing_waypoint",
            "pass_index": 0,
            "waypoint_index": 2 * turns,
            "m2_phase_rad": 2 * turns * math.pi,
            "observed_m2_phase_rad": 2 * turns * math.pi,
            "m0_target_rad": target,
            "observed_m0_rad": target,
            "m0_error_rad": 0.0,
            "placement_index": turns - 1,
            "kind": "final_hold",
            "m0_ready_phase_rad": (2 * turns - 1) * math.pi,
            "m0_settled_before_crossing": True,
        })
        half_turns, depositions = _packing_progress(
            events, [winding], meta, self.plan)
        self.assertEqual(len(half_turns), 100)
        self.assertEqual(len(depositions), 50)
        self.assertEqual([row["side"] for row in half_turns[:4]],
                         ["left", "right", "left", "right"])
        self.assertEqual(half_turns[0]["contact"], "slot_liner")
        self.assertEqual(half_turns[0]["support"], "slot_liner")
        self.assertEqual(half_turns[-1]["placementIndex"], 49)
        self.assertEqual(depositions[0]["midpoint"], 0.1)
        self.assertEqual(depositions[-1]["end"], 10.0)
        coil_starts = _coil_start_events([winding], depositions)
        self.assertEqual(len(coil_starts), 1)
        start = coil_starts[0]
        self.assertEqual(start["event"], "coil_start")
        self.assertEqual(start["positioningStart"], 0.0)
        self.assertEqual(start["layStart"], depositions[0]["start"])
        self.assertEqual(start["firstTurnEnd"], depositions[0]["end"])
        self.assertEqual(start["firstSide"], "left")
        self.assertEqual(start["secondSide"], "right")
        self.assertTrue(start["continuousConductor"])
        self.assertFalse(start["cutOrJoin"])
        self.assertIsNone(start["continuityFromPassIndex"])
        self.assertIn("no synthetic lead", start["markerAuthority"])


class CurrentCapturePlayerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.events = load_events(ROOT / "out" / "capture" / "commands.jsonl")
        cls.timeline = Timeline(cls.events)
        cls.manifest = json.loads(
            (ROOT / "out" / "links" / "manifest.json").read_text()
        )
        cls.plan = _load_player_slot_plan(PLAN_PATH)
        cls.data = _player_data(
            cls.events, cls.timeline, cls.manifest, 10.0, 10.0,
            slot_plan=cls.plan,
        )

    def test_player_contains_complete_plan_scheduled_capture(self):
        turns = self.timeline.meta["turns"]
        expected_commands = sum(
            event["e"] == "cmd" for event in self.events
        )
        self.assertEqual(self.data["schema"], PLAYER_SCHEMA)
        self.assertEqual(len(self.data["commands"]), expected_commands)
        self.assertEqual(len(self.data["windings"]), 24)
        self.assertEqual(len(self.data["depositions"]), 24 * turns)
        self.assertEqual(len(self.data["halfTurns"]), 48 * turns)
        self.assertEqual(len(self.data["coilStarts"]), 24)
        self.assertEqual(self.data["depositionSummary"]["count"], 24 * turns)
        self.assertEqual(
            self.data["slotPlan"]["artifactSha256"],
            self.timeline.meta["winding_plan"]["sha256"],
        )

    def test_each_pass_builds_plan_placements_zero_through_forty_nine(self):
        for winding in self.data["windings"]:
            rows = [row for row in self.data["depositions"]
                    if row["passIndex"] == winding["passIndex"]]
            halves = [row for row in self.data["halfTurns"]
                      if row["passIndex"] == winding["passIndex"]]
            self.assertEqual([row["placementIndex"] for row in rows],
                             list(range(50)))
            self.assertEqual([row["placementIndex"] for row in halves],
                             [index // 2 for index in range(100)])
            expected_first = "right" if winding["clockwise"] else "left"
            self.assertEqual(halves[0]["side"], expected_first)
            self.assertEqual(len(winding["turnTimes"]), 50)
            self.assertEqual(len(winding["halfTurnTimes"]), 100)

    def test_every_pass_has_an_unambiguous_continuous_coil_start(self):
        starts = self.data["coilStarts"]
        self.assertEqual([start["passIndex"] for start in starts],
                         list(range(24)))
        self.assertEqual([start["phase"] for start in starts],
                         [index // 8 for index in range(24)])
        for index, (start, winding) in enumerate(zip(
                starts, self.data["windings"])):
            with self.subTest(pass_index=index):
                self.assertEqual(start["event"], "coil_start")
                self.assertEqual(start["tooth"], winding["tooth"])
                self.assertEqual(
                    start["direction"],
                    "clockwise" if winding["clockwise"]
                    else "counter-clockwise",
                )
                self.assertLessEqual(start["positioningStart"],
                                     start["toothPresentedAt"])
                self.assertLessEqual(start["toothPresentedAt"],
                                     start["motionStart"])
                self.assertLessEqual(start["motionStart"], start["layStart"])
                self.assertLessEqual(start["layStart"],
                                     start["firstHalfEnd"])
                self.assertLessEqual(start["firstHalfEnd"],
                                     start["firstTurnEnd"])
                self.assertTrue(start["continuousConductor"])
                self.assertFalse(start["cutOrJoin"])
                self.assertEqual(
                    start["continuityFromPassIndex"],
                    None if index == 0 else index - 1,
                )
        synthetic_markers = [marker for marker in self.data["markers"]
                             if marker[1] == "coil_start"]
        self.assertEqual(len(synthetic_markers), 24)

    def test_visual_groups_are_exact_and_not_duplicated_in_base_links(self):
        records = _visual_group_asset_records(self.manifest)
        self.assertEqual(
            [record["name"] for record in records], ["felt_pads"]
        )
        for record in records:
            self.assertTrue(record["path"].is_file())
        self.assertAlmostEqual(
            self.data["wireRenderRadius"],
            self.data["slotPlan"]["job"]["wireFinishedDiameter"] / 2.0,
            places=12,
        )

    def test_active_lay_starts_with_exactly_presented_tooth(self):
        pitch = 2.0 * math.pi / self.data["teethCount"]
        for winding in self.data["windings"]:
            first_half = next(row for row in self.data["halfTurns"]
                              if row["passIndex"] == winding["passIndex"])
            m1 = self.timeline.axes[1].pos_at(first_half["start"])
            residual = math.atan2(
                math.sin(m1 + winding["tooth"] * pitch),
                math.cos(m1 + winding["tooth"] * pitch),
            )
            self.assertLessEqual(abs(residual), math.radians(0.001))

    def test_no_fake_final_ellipse_survives_in_player_state(self):
        for deposition in self.data["depositions"]:
            self.assertNotIn("radialSamples", deposition)
            self.assertIn("placementIndex", deposition)
        authority = self.data["slotPlan"]["geometryAuthority"]
        self.assertIn("exact", authority["slotLegCenters"])
        self.assertIn("illustrative", authority["endTurnConnectors"])
        self.assertIn("not a sequential route proof", authority["liveFeedSpan"])

    def test_generated_player_embeds_current_state_and_three_axis_glb(self):
        html = (
            INTEGRATED_PLAYER_ROOT / "play_integrated_candidate_raw.html"
        ).read_text()
        state_match = re.search(r'const stateB64 = "([A-Za-z0-9+/=]+)";', html)
        glb_match = re.search(r'const glbB64 = "([A-Za-z0-9+/=]+)";', html)
        self.assertIsNotNone(state_match)
        self.assertIsNotNone(glb_match)
        embedded_state = json.loads(base64.b64decode(state_match.group(1)))
        self.assertEqual(embedded_state["schema"], PLAYER_SCHEMA)
        self.assertEqual(
            embedded_state["slotPlan"]["artifactSha256"],
            self.plan["artifactSha256"],
        )
        self.assertEqual(len(embedded_state["halfTurns"]), 24 * 100)
        self.assertEqual(len(embedded_state["coilStarts"]), 24)
        self.assertEqual(
            base64.b64decode(glb_match.group(1)),
            (
                INTEGRATED_PLAYER_ROOT
                / "winding_cycle_integrated_candidate_raw.glb"
            ).read_bytes(),
        )
        html_ids = set(re.findall(r'id="([A-Za-z0-9-]+)"', html))
        queried_ids = set(re.findall(r"\$\('([A-Za-z0-9-]+)'\)", html))
        self.assertFalse(queried_ids - html_ids)
        self.assertNotIn("__GLB_B64__", html)
        self.assertNotIn("__STATE_B64__", html)
        self.assertNotIn("offsetEllipse", html)
        self.assertNotIn("slotRouteForPose", html)
        self.assertIn("function coilTurnPoints", html)
        self.assertIn("function createCoilStartMarker", html)
        self.assertIn("function updateCoilStart", html)
        self.assertIn("No cut or join between passes", html)
        self.assertIn("two real wool-felt drag pads", html)
        self.assertIn(
            "four retained rear stacks plus two front B777 trim stacks", html
        )
        self.assertIn("Rendering is not the sequential route proof", html)
        for start_id in (
            "coil-start-banner", "coil-start-stage", "coil-start-title",
            "coil-start-detail",
        ):
            self.assertIn(f'id="{start_id}"', html)
        for axis in range(4):
            self.assertIn(f'id="m{axis}-controller"', html)
            self.assertIn(f'id="m{axis}-progress"', html)
        for control_id in (
            "play", "scrub", "prev-command", "next-command", "speed",
            "camera", "show-wire", "show-coils", "follow-wire",
        ):
            self.assertIn(f'id="{control_id}"', html)

        model = GLTF2().load_binary(
            INTEGRATED_PLAYER_ROOT
            / "winding_cycle_integrated_candidate_raw.glb"
        )
        self.assertEqual(len(model.animations), 1)
        self.assertEqual(len(model.animations[0].channels), 3)
        adapter_manifest = json.loads(
            (INTEGRATED_PLAYER_ROOT / "links" / "manifest.json").read_text()
        )
        self.assertEqual(
            {node.name for node in model.nodes},
            {"static", "carriage", "spindle_pivot", "spindle", "flyer",
             "wire_static", "wire_flyer"}
            | set(adapter_manifest["visual_groups"]),
        )
        materials = {material.name: material for material in model.materials}
        self.assertIn("felt_pads", materials)
        felt = materials["felt_pads"].pbrMetallicRoughness
        self.assertEqual(felt.baseColorFactor, [0.22, 0.075, 0.025, 1.0])
        self.assertEqual(felt.metallicFactor, 0.0)
        self.assertEqual(felt.roughnessFactor, 1.0)
        node_indices = {node.name: index
                        for index, node in enumerate(model.nodes)}
        self.assertIn(node_indices["felt_pads"], model.scenes[0].nodes)


if __name__ == "__main__":
    unittest.main(verbosity=2)
