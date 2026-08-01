"""Structural contracts for the canonical continuous conductor artifact."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import continuous_conductor_route as route  # noqa: E402
from traj import Timeline, load_events  # noqa: E402


class ContinuousConductorRouteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = json.loads(
            route.OUTPUT_JSON.read_text(encoding="utf-8")
        )
        route.validate_route_artifact(cls.report)
        cls.items = cls.report["items"]

    def test_full_timeline_has_one_visible_live_endpoint(self) -> None:
        self.assertEqual(self.items[0]["start_time_s"], 0.0)
        self.assertAlmostEqual(
            self.items[-1]["end_time_s"],
            self.report["timeline"]["end_time_s"],
            places=8,
        )
        probes = {0.0, self.report["timeline"]["end_time_s"]}
        for item in self.items:
            probes.update((
                item["start_time_s"],
                (item["start_time_s"] + item["end_time_s"]) / 2.0,
                item["end_time_s"],
            ))
        for time_s in sorted(probes):
            with self.subTest(time_s=time_s):
                endpoint = route.live_endpoint_at(self.report, time_s)
                self.assertEqual(len(endpoint), 3)
                self.assertTrue(all(math.isfinite(value)
                                    for value in endpoint))
        self.assertTrue(self.report["timeline"]["no_hidden_live_interval"])

    def test_ordered_conductor_graph_is_one_connected_chain(self) -> None:
        self.assertEqual(
            [item["index"] for item in self.items],
            list(range(len(self.items))),
        )
        for previous, current in zip(self.items, self.items[1:]):
            self.assertLessEqual(
                route._distance(
                    previous["end_point_mm"], current["start_point_mm"],
                ),
                route.POINT_TOL_MM,
            )
            self.assertAlmostEqual(
                previous["end_time_s"], current["start_time_s"], places=6,
            )

    def test_live_endpoint_fields_equal_the_deposited_tail(self) -> None:
        for item in self.items:
            self.assertEqual(
                item["live_endpoint_start_mm"], item["points_mm"][0],
            )
            self.assertEqual(
                item["live_endpoint_end_mm"], item["points_mm"][-1],
            )
            self.assertEqual(
                route._item_tail_at(item, item["start_time_s"]),
                item["live_endpoint_start_mm"],
            )
            self.assertEqual(
                route._item_tail_at(item, item["end_time_s"]),
                item["live_endpoint_end_mm"],
            )

    def test_every_point_jump_is_bounded(self) -> None:
        maximum = 0.0
        for item in self.items:
            for left, right in zip(
                    item["points_mm"], item["points_mm"][1:]):
                maximum = max(maximum, route._distance(left, right))
        self.assertLessEqual(maximum, route.MAX_POINT_JUMP_MM + 1.0e-8)
        self.assertAlmostEqual(
            maximum,
            self.report["metrics"]["maximum_point_jump_mm"],
            places=9,
        )

    def test_inter_turn_cap_and_tooth_transitions_are_explicit_fail_closed(self) -> None:
        observed = {
            run["kind"]: run
            for item in self.items for run in item["runs"]
        }
        for kind in route.TRANSITION_KINDS:
            with self.subTest(kind=kind):
                self.assertIn(kind, observed)
                self.assertEqual(observed[kind]["authorization"],
                                 route.UNPROVEN)
                self.assertEqual(observed[kind]["visual_style"],
                                 route.DASHED_RED)
        self.assertGreaterEqual(
            self.report["edge_kind_counts"]["inter_turn_advance"],
            24 * 49,
        )
        self.assertEqual(self.report["status"], "FAIL")
        self.assertFalse(self.report["production_authorized"])

    def test_both_raw_shaft_wraps_persist_to_cycle_end(self) -> None:
        wraps = [item for item in self.items
                 if item["kind"] == "shaft_wrap"]
        self.assertEqual([item["wrap_number"] for item in wraps], [1, 2])
        self.assertEqual(
            [item["source_command"] for item in wraps],
            ["M1A-12.566", "M1A0.0"],
        )
        self.assertTrue(all(item["persistent_after_end"] for item in wraps))
        final_index = self.items[-1]["index"]
        self.assertTrue(all(item["index"] < final_index for item in wraps))
        self.assertEqual(
            self.report["shaft_wrap_item_indices"],
            [item["index"] for item in wraps],
        )

    def test_route_hash_status_and_sources_are_bound(self) -> None:
        self.assertEqual(
            self.report["report_sha256"], route._canonical_hash(self.report),
        )
        self.assertEqual(
            self.report["source_hashes"]["raw_capture_sha256"],
            route._sha256(route.CAPTURE),
        )
        self.assertEqual(self.report["structural_status"], "PASS")
        self.assertEqual(
            self.report["decision"],
            "CONNECTED_PRESENTATION_ROUTE_GEOMETRY_UNPROVEN",
        )
        self.assertTrue(self.report["release_blockers"])

    def test_phase_and_pass_order_comes_from_raw_capture(self) -> None:
        order = self.report["pass_order"]
        self.assertEqual([row["pass_index"] for row in order], list(range(24)))
        self.assertEqual([row["phase"] for row in order],
                         [0] * 8 + [1] * 8 + [2] * 8)
        half_items = [item for item in self.items
                      if item["kind"] == "winding_half_turn"]
        self.assertEqual(len(half_items), 2400)
        self.assertEqual(
            [(item["phase"], item["pass_index"], item["half_turn_index"])
             for item in half_items[::100]],
            [(index // 8, index, 0) for index in range(24)],
        )

    def test_every_route_clock_is_constructible_from_the_exit_bell(self) -> None:
        """Every live target admits a tangent from the physical PEEK bell."""

        manifest = json.loads(route.MANIFEST.read_text(encoding="utf-8"))
        timeline = Timeline(load_events(route.CAPTURE))
        standoff = float(manifest["m0_home_standoff"])
        mm_per_rad = float(manifest["mm_per_rad_m0"])
        guide = manifest["wire"]["active_terminal_guide"]
        feed = guide["unproved_transition_origin_local_mm"]
        self.assertEqual(feed, manifest["wire"]["flyer"]["points"][-1])
        path_radius = (
            float(guide["exit_bell_contact_surface_radius_mm"])
            + float(manifest["wire"]["radius_max"])
        )
        minimum_margin = math.inf
        for item in self.items:
            clocks = item["point_times_s"]
            if len(clocks) != len(item["points_mm"]):
                self.fail(f"item {item['index']} point clock length drifted")
            for point, time_s in zip(item["points_mm"], clocks):
                m0, m1, m2 = timeline.pose_at(float(time_s))
                # spindle local -> world: carriage translation, stator-axis
                # pivot, then spindle's -standoff child translation.
                qx, qy, qz = point[0], point[1], point[2] - standoff
                c1, s1 = math.cos(m1), math.sin(m1)
                target = (
                    c1 * qx + s1 * qz,
                    qy,
                    standoff + m0 * mm_per_rad - s1 * qx + c1 * qz,
                )
                c2, s2 = math.cos(m2), math.sin(m2)
                center = (
                    c2 * float(feed[0]) - s2 * float(feed[1]),
                    s2 * float(feed[0]) + c2 * float(feed[1]),
                    float(feed[2]),
                )
                relative = tuple(target[i] - center[i] for i in range(3))
                center_distance = math.sqrt(
                    sum(value * value for value in relative)
                )
                minimum_margin = min(
                    minimum_margin, center_distance - path_radius,
                )
        self.assertGreater(minimum_margin, 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
