"""Focused geometry contract for the active-sector guide and fixed yoke."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

from build123d import Vertex

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import carriage_active_sector_terminal_guide as guide


class CarriageActiveSectorTerminalGuideTests(unittest.TestCase):
    def test_short_leadins_bind_actual_cap_lane_and_fixed_handoff(self) -> None:
        self.assertEqual(
            guide.cap_lane_endpoint_name(-1), "riser_top"
        )
        self.assertEqual(
            guide.cap_lane_endpoint_name(1), "waypoint"
        )
        for sign in (-1, 1):
            lane = guide.cap.lane_wire(sign)
            for side in (-1, 1):
                endpoint = guide.cap_lane_endpoint(side, sign)
                handoff = guide.leadin_handoff(side, sign)
                centerline = guide._leadin_centerline(side, sign)
                self.assertLessEqual(
                    lane.distance_to(Vertex(endpoint)), 1.0e-9
                )
                self.assertLessEqual(
                    centerline.position_at(0.0).sub(endpoint).length,
                    1.0e-9,
                )
                self.assertLessEqual(
                    centerline.position_at(1.0).sub(handoff).length,
                    1.0e-9,
                )
                start_tangent = centerline.tangent_at(0.0)
                end_tangent = centerline.tangent_at(1.0)
                self.assertAlmostEqual(start_tangent.X, 0.0, places=9)
                self.assertAlmostEqual(start_tangent.Y, 0.0, places=9)
                self.assertAlmostEqual(start_tangent.Z, sign, places=9)
                self.assertAlmostEqual(end_tangent.X, 1.0, places=9)
                self.assertAlmostEqual(end_tangent.Y, 0.0, places=9)
                self.assertAlmostEqual(end_tangent.Z, 0.0, places=9)

    def test_right_s_bend_is_symmetric_and_above_r3p5(self) -> None:
        self.assertAlmostEqual(
            guide.RIGHT_S_BEND_RADIUS_MM,
            7.031443572259498,
            places=12,
        )
        self.assertAlmostEqual(
            guide.RIGHT_S_BEND_SWEEP_DEG,
            23.30199182941579,
            places=12,
        )
        self.assertGreaterEqual(
            guide.RIGHT_S_BEND_RADIUS_MM,
            guide.LEADIN_CENTERLINE_RADIUS_MM,
        )
        for sign in (-1, 1):
            for side, expected_edge_count, expected_circle_count in (
                (-1, 3, 1), (1, 4, 3),
            ):
                edges = list(guide._leadin_centerline(side, sign).edges())
                self.assertEqual(len(edges), expected_edge_count)
                circles = [
                    edge for edge in edges
                    if edge.geom_type.name == "CIRCLE"
                ]
                self.assertEqual(len(circles), expected_circle_count)
                for edge in circles:
                    self.assertGreaterEqual(float(edge.radius), 3.5 - 1.0e-9)
                for first, second in zip(edges, edges[1:]):
                    first_end = first.position_at(1.0)
                    second_start = second.position_at(0.0)
                    self.assertLessEqual(
                        first_end.sub(second_start).length, 1.0e-9
                    )
                    tangent_dot = first.tangent_at(1.0).dot(
                        second.tangent_at(0.0)
                    )
                    tangent_dot = max(-1.0, min(1.0, tangent_dot))
                    self.assertLessEqual(
                        math.degrees(math.acos(tangent_dot)), 1.0e-7
                    )

    def test_global_groove_cut_leaves_adjacent_separator_web(self) -> None:
        for sign in (-1, 1):
            right_outer = guide._outer_channel_for_tooth(0, 1, sign)
            left_outer = guide._outer_channel_for_tooth(1, -1, sign)
            right_negative = guide._channel_negative_for_tooth(0, 1, sign)
            left_negative = guide._channel_negative_for_tooth(1, -1, sign)
            self.assertGreater(
                float(right_negative.distance_to(left_negative)), 0.5
            )
            self.assertGreater(float((right_outer & left_outer).volume), 0.0)
            pair = right_outer.fuse(left_outer).cut(
                right_negative, left_negative,
            )
            self.assertEqual(len(list(pair.solids())), 1)

    def test_right_seam_breakouts_are_final_global_cuts(self) -> None:
        self.assertAlmostEqual(
            guide.RIGHT_SEAM_MOUTH_RADIAL_LENGTH_MM, 2.40, places=12,
        )
        self.assertAlmostEqual(
            guide.RIGHT_SEAM_MOUTH_TANGENTIAL_WIDTH_MM, 0.75, places=12,
        )
        self.assertAlmostEqual(
            guide.RIGHT_SEAM_MOUTH_AXIAL_SPAN_MM, 0.90, places=12,
        )
        for sign in (-1, 1):
            before = guide._cap_with_short_leadins_before_right_seam_mouth(
                sign
            )
            final = guide.cap_with_short_leadins(sign)
            removed_per_seam = float(before.volume - final.volume) / guide.SLOTS
            self.assertEqual(len(list(final.solids())), 1)
            self.assertGreater(removed_per_seam, 0.15)
            self.assertLess(removed_per_seam, 0.16)

    def test_front_plane_wrap_bypass_constants(self) -> None:
        self.assertEqual(guide.YOKE_RADIAL_X_MM, 13.0)
        self.assertEqual(guide.YOKE_BAR_RADIAL_LENGTH_MM, 14.0)
        self.assertEqual(guide.YOKE_FRONT_PLANE_X_MIN_MM, 6.0)
        self.assertEqual(guide.YOKE_FRONT_DOGLEG_AXIAL_MM, 5.0)
        self.assertEqual(guide.YOKE_AXIAL_MAX_MM, 8.7)
        self.assertEqual(guide.YOKE_GUIDE_CONNECTOR_X_MAX_MM, 39.0)
        self.assertEqual(guide.YOKE_GUIDE_SEAT_X_MAX_MM, 39.85)
        self.assertEqual(guide.YOKE_TANGENTIAL_MM, 34.5)

    def test_yoke_and_each_peek_guide_are_single_solids(self) -> None:
        self.assertEqual(len(list(guide.carriage_yoke().solids())), 1)
        for sign in (-1, 1):
            self.assertEqual(
                len(list(guide.active_sector_guide(sign).solids())), 1
            )

    def test_guides_face_mate_without_positive_overlap(self) -> None:
        yoke = guide.carriage_yoke()
        for sign in (-1, 1):
            peek = guide.active_sector_guide(sign)
            self.assertAlmostEqual(peek.distance_to(yoke), 0.0, places=9)
            common = peek & yoke
            overlap = 0.0 if common is None else float(common.volume)
            self.assertAlmostEqual(overlap, 0.0, places=9)

    def test_yoke_face_mates_revised_tower(self) -> None:
        yoke = guide.to_machine_reference(guide.carriage_yoke())
        tower = guide.revised_spindle_tower()
        self.assertAlmostEqual(yoke.distance_to(tower), 0.0, places=9)
        common = yoke & tower
        overlap = 0.0 if common is None else float(common.volume)
        self.assertAlmostEqual(overlap, 0.0, places=9)

    def test_attachment_stack_counts(self) -> None:
        self.assertEqual(len(guide.guide_retention_hardware()), 12)
        self.assertEqual(len(guide.tower_adapter_hardware_reference()), 12)


if __name__ == "__main__":
    unittest.main(verbosity=2)
