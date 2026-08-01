"""Focused contracts for the exact active-terminal locus player seam."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import animate


def _fixture_payload(capture: Path) -> dict:
    contract = {
        "flyer_geometric_bore": {
            "surface_owner": "flyer",
            "local_frame": "flyer_reference_M2_axis_plus_Z",
            "authority": "fixture shared flyer reference",
        },
        "exact_segment": {
            "surface_owner": "flyer",
            "local_frame": "fixture",
            "authority": "fixture exact sampled route",
        },
    }
    loci = []
    for index in range(animate.EXPECTED_ACTIVE_TERMINAL_LOCI):
        pass_index, state_index = divmod(index, 100)
        time_s = index + 0.25
        flyer_angle = time_s * 0.003
        reference_end_world = [
            -math.sin(flyer_angle), math.cos(flyer_angle), 0.0,
        ]
        loci.append({
            "locus_index": index,
            "time_s": time_s,
            "pass_index": pass_index,
            "phase_index": pass_index // 8,
            "state_index": state_index,
            "turn_index": state_index // 2,
            "half_turn_index": state_index & 1,
            "tooth_index": pass_index,
            "motion_sign": 1 if pass_index % 2 == 0 else -1,
            "axes": {
                "M0_raw_rad": time_s * 0.001,
                "M1_spindle_rad": time_s * 0.002,
                "M2_flyer_rad": time_s * 0.003,
            },
            "path_sha256": f"{index:064x}"[-64:],
            "segments": [{
                "name": "exact_segment",
                **contract["exact_segment"],
                "local_samples_mm": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
                "machine_world_samples_mm": [
                    reference_end_world,
                    [reference_end_world[0], reference_end_world[1], 1.0],
                    [reference_end_world[0], reference_end_world[1], 2.0],
                ],
            }],
            "terminal_binding": {
                "exact_strand_settling_and_neatness_authorized": False,
            },
        })
    payload = {
        "schema": animate.ACTIVE_TERMINAL_LOCI_SCHEMA,
        "run": {
            "capture": capture.name,
            "capture_sha256": animate._file_sha256(capture),
            "goal_contract": "fixture",
            "tags": ["2400_deposition_loci"],
            "locus_count": len(loci),
        },
        "axes_mapping": {"M0": "fixture", "M1": "fixture", "M2": "fixture"},
        "segment_contract": contract,
        "flyer_reference": {
            "frame": "flyer_reference_M2_axis_plus_Z",
            "full_geometric_bore_local_samples_mm": [
                [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 2.0, 0.0]
            ],
            "full_geometric_bore_point_count": 3,
            "conductor_prefix_point_count": 2,
            "geometric_bore_to_tensioned_handoff_local_samples_mm": [
                [0.0, 0.0, 0.0], [0.0, 1.0, 0.0]
            ],
            "source_api": "fixture",
        },
        "loci": loci,
    }
    payload["locus_payload_sha256"] = animate._active_terminal_payload_hash(
        payload
    )
    return payload


class _FixtureTimeline:
    @staticmethod
    def pose_at(time_s: float) -> tuple[float, float, float]:
        return time_s * 0.001, time_s * 0.002, time_s * 0.003


class ActiveTerminalLocusPlayerTests(unittest.TestCase):
    @staticmethod
    def _wire_fixture() -> dict:
        bore = [
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 2.0, 0.0],
        ]
        return {
            "static": {
                "points": [[0.0, 0.0, -1.0], [0.0, 0.0, 0.0]],
                "landmarks": {"guide_root": [0.0, 0.0, 0.0]},
            },
            "flyer": {"points": bore},
            "active_terminal_guide": {
                "bore_centerline_local_mm": bore,
                "unproved_transition_origin_local_mm": bore[-1],
            },
        }

    def test_loader_accepts_exact_2400_hash_bound_loci_and_sizes_wire_pool(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            capture = root / "capture.jsonl"
            capture.write_text("fixture raw capture\n", encoding="utf-8")
            payload = _fixture_payload(capture)
            path = root / animate.ACTIVE_TERMINAL_LOCI_NAME
            path.write_text(json.dumps(payload), encoding="utf-8")

            loaded = animate._load_active_terminal_loci(
                path, capture_path=capture
            )

            self.assertEqual(loaded, payload)
            self.assertEqual(len(loaded["loci"]), 2400)
            self.assertEqual(animate._active_terminal_max_edges(loaded), 3)

    def test_locus_order_maps_exactly_to_raw_pass_and_half_turn_start(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture = Path(tmp) / "capture.jsonl"
            capture.write_bytes(b"fixture")
            payload = _fixture_payload(capture)
            half_turns = [{
                "passIndex": locus["pass_index"],
                "halfTurnIndex": locus["state_index"],
                "start": locus["time_s"],
            } for locus in payload["loci"]]

            animate._validate_active_terminal_timeline(
                payload, half_turns, _FixtureTimeline()
            )
            half_turns[101]["start"] += 0.01
            with self.assertRaisesRegex(RuntimeError, "half-turn start"):
                animate._validate_active_terminal_timeline(
                    payload, half_turns, _FixtureTimeline()
                )

    def test_static_guide_root_locus_prefix_and_coil_starts_share_one_clocked_chain(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture = Path(tmp) / "capture.jsonl"
            capture.write_bytes(b"fixture")
            payload = _fixture_payload(capture)
            handoff = animate._validate_active_terminal_wire_chain(
                self._wire_fixture(), payload
            )
            self.assertEqual(
                handoff["static_to_flyer_seam_local_mm"], [0.0, 0.0, 0.0]
            )
            self.assertEqual(handoff["maximum_gap_mm"], 0.0)

            starts = [{
                "passIndex": pass_index,
                "layStart": payload["loci"][pass_index * 100]["time_s"],
            } for pass_index in range(24)]
            animate._bind_coil_starts_to_terminal_loci(starts, payload)
            self.assertEqual(starts[7]["firstTerminalLocusIndex"], 700)
            self.assertEqual(
                starts[7]["firstTerminalLocusTime"], starts[7]["layStart"]
            )
            self.assertTrue(starts[7]["firstTerminalLocusExact"])

            starts[8]["layStart"] += 0.01
            with self.assertRaisesRegex(RuntimeError, "lay clock"):
                animate._bind_coil_starts_to_terminal_loci(starts, payload)

    def test_loader_rejects_capture_hash_drift_and_obsolete_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            capture = root / "capture.jsonl"
            capture.write_bytes(b"fixture")
            payload = _fixture_payload(capture)
            path = root / animate.ACTIVE_TERMINAL_LOCI_NAME
            path.write_text(json.dumps(payload), encoding="utf-8")
            capture.write_bytes(b"different capture")
            with self.assertRaisesRegex(RuntimeError, "another raw capture"):
                animate._load_active_terminal_loci(path, capture_path=capture)

            capture.write_bytes(b"fixture")
            payload = _fixture_payload(capture)
            payload["segment_contract"]["obsolete_torus"] = {}
            payload["locus_payload_sha256"] = (
                animate._active_terminal_payload_hash(payload)
            )
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "obsolete torus"):
                animate._load_active_terminal_loci(path, capture_path=capture)

    def test_template_uses_exact_world_polylines_and_no_guide_reconstruction(self):
        template = (Path(animate.__file__).parent / "player_template.html").read_text(
            encoding="utf-8"
        )
        self.assertIn("function activeTerminalLocusAt", template)
        self.assertIn("function setExactTerminalLocus", template)
        self.assertIn("machine_world_samples_mm", template)
        self.assertIn("geometric_bore_to_tensioned_handoff_local_samples_mm", template)
        self.assertIn("applyAxisAngle(zAxis, flyerAngle)", template)
        self.assertIn("state.activeTerminalLoci.segment_contract[name]", template)
        self.assertIn(
            "exact_sampled_machine_prefix_plus_unproved_cap_to_live_tail_witness",
            template,
        )
        self.assertIn("setExactTerminalLocus(exactLocus, target)", template)
        self.assertIn(
            "unproved_cap_endpoint_to_live_deposited_tail_witness", template
        )
        self.assertIn("UNPROVED_FREE_SPAN_NO_SUPPORT_OWNER", template)
        self.assertIn("unprovedConnectorEdgeCount", template)
        self.assertIn("unproved_straight_transition_witness", template)
        self.assertIn("park/index/load/unload", template)
        self.assertIn("settling, and neatness", template)
        self.assertIn("overlap is intended tooth engagement", template)
        self.assertIn("orange/copper wire is separate", template)
        self.assertIn("Belt/pulley overlap is intended engagement", template)
        self.assertIn('id="prev-coil"', template)
        self.assertIn('id="next-coil"', template)
        self.assertIn('id="hold-coil-starts"', template)
        self.assertIn('id="focus-coil-starts"', template)
        self.assertIn("function updateCoilStart", template)
        self.assertIn("static_owner_continues_through_shaft_to_guide_root", template)
        self.assertIn("misses the static guide-root seam", template)
        self.assertIn("firstTerminalLocusExact", template)
        self.assertNotIn("function tipGuidePath", template)
        self.assertNotIn("No physical tip-torus tangent", template)
        self.assertNotIn("function circleTangents2D", template)


if __name__ == "__main__":
    unittest.main(verbosity=2)
