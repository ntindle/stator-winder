"""Deterministic count, axis, and mating checks for hardware placements."""

from __future__ import annotations

from collections import Counter, defaultdict
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import hardware  # noqa: E402
import hardware_placements as placements  # noqa: E402
from params import PARAMS  # noqa: E402


def _sub(a, b):
    return tuple(float(x - y) for x, y in zip(a, b))


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _norm(a):
    return math.sqrt(_dot(a, a))


class HardwarePlacementContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = placements.current_geometry(PARAMS)
        cls.links = placements.hardware_occurrences_by_link(
            PARAMS, cls.geometry,
        )
        cls.all_occurrences = [
            occurrence
            for occurrences in cls.links.values()
            for occurrence in occurrences
        ]

    def test_all_four_links_exist_and_labels_are_unique(self):
        self.assertEqual(
            set(self.links), {"static", "carriage", "spindle", "flyer"},
        )
        self.assertEqual(self.links["spindle"], [])
        labels = [occurrence.label for occurrence in self.all_occurrences]
        self.assertEqual(len(labels), len(set(labels)))

    def test_audit_schedule_quantity_is_fully_placed(self):
        counts = Counter(
            occurrence.schedule_id for occurrence in self.all_occurrences
        )
        local_items = [
            item for item in hardware.HARDWARE_SCHEDULE
            if item.get("placement_authority") == "hardware_placements"
        ]
        for item in local_items:
            with self.subTest(schedule_id=item["id"]):
                self.assertEqual(counts[item["id"]], item["qty"])
        self.assertEqual(
            placements.unmodeled_schedule_items(PARAMS, self.geometry),
            {},
        )

    def test_frame_has_fifteen_hbkts_plus_one_complete_printed_shoe(self):
        brackets = [
            occurrence for occurrence in self.links["static"]
            if occurrence.schedule_id == "frame_brackets"
        ]
        screws = [
            occurrence for occurrence in self.links["static"]
            if occurrence.schedule_id == "frame_bracket_screws"
        ]
        nuts = [
            occurrence for occurrence in self.links["static"]
            if occurrence.schedule_id == "frame_bracket_tnuts"
        ]
        self.assertEqual((len(brackets), len(screws), len(nuts)), (15, 30, 30))

        by_joint = defaultdict(list)
        for occurrence in screws + nuts:
            by_joint[occurrence.mate_id.rsplit(":", 1)[0]].append(occurrence)
        self.assertEqual(len(by_joint), 15)
        for joint, occurrences in by_joint.items():
            with self.subTest(joint=joint):
                self.assertEqual(len(occurrences), 4)
                self.assertEqual(
                    Counter(o.schedule_id for o in occurrences),
                    Counter({"frame_bracket_screws": 2,
                             "frame_bracket_tnuts": 2}),
                )
        shoe = [o for o in self.links["static"]
                if o.schedule_id in {"rear_post_shoe_screws",
                                     "rear_post_shoe_tnuts"}]
        self.assertEqual(
            Counter(o.schedule_id for o in shoe),
            Counter({"rear_post_shoe_screws": 2,
                     "rear_post_shoe_tnuts": 2}),
        )
        self.assertEqual(len({o.mate_id for o in shoe}), 2)

    def test_bracket_frames_are_right_handed_and_holes_match_catalog_axes(self):
        for frame in self.geometry.frame_brackets:
            with self.subTest(frame=frame.label):
                self.assertAlmostEqual(_norm(frame.x_dir), 1.0)
                self.assertAlmostEqual(_norm(frame.y_dir), 1.0)
                self.assertAlmostEqual(_norm(frame.z_dir), 1.0)
                self.assertEqual(_cross(frame.z_dir, frame.x_dir), frame.y_dir)

                floor_mate = f"{frame.joint}:floor"
                upright_mate = f"{frame.joint}:upright"
                floor = [o for o in self.all_occurrences
                         if o.mate_id == floor_mate]
                upright = [o for o in self.all_occurrences
                           if o.mate_id == upright_mate]
                self.assertEqual({o.axis for o in floor}, {
                    placements._axis_name(frame.z_dir),
                    placements._opposite(placements._axis_name(frame.z_dir)),
                })
                self.assertEqual({o.axis for o in upright}, {
                    placements._axis_name(frame.y_dir),
                    placements._opposite(placements._axis_name(frame.y_dir)),
                })
                self.assertEqual({o.mate_center for o in floor}, {
                    frame.point(0.0, 12.0, 0.0),
                })
                self.assertEqual({o.mate_center for o in upright}, {
                    frame.point(0.0, 0.0, 12.0),
                })

    def test_bracket_floor_origins_sit_on_the_actual_supporting_top_faces(self):
        for frame in self.geometry.frame_brackets:
            with self.subTest(frame=frame.label):
                if frame.label.startswith("frame_bracket_post_"):
                    expected_y = PARAMS.stringer_top_y
                else:
                    # Cross/base, cross/stringer and rear-post braces all sit
                    # on a lower cross-member top face.
                    expected_y = PARAMS.base_top_y
                self.assertAlmostEqual(frame.origin[1], expected_y)

    def test_every_occurrence_axis_is_cardinal_and_hits_its_mate_center(self):
        for occurrence in self.all_occurrences:
            if occurrence.bracket_frame is not None:
                self.assertIsNone(occurrence.axis)
                continue
            with self.subTest(label=occurrence.label):
                self.assertIn(occurrence.axis, placements.AXIS_VECTOR)
                axis = occurrence.axis_vector
                delta = _sub(occurrence.origin, occurrence.mate_center)
                self.assertLess(_norm(_cross(delta, axis)), 1e-8)

        # Every multi-part stack has one common line and all member axes are
        # parallel or anti-parallel, never skew.
        by_mate = defaultdict(list)
        for occurrence in self.all_occurrences:
            if occurrence.axis is not None:
                by_mate[occurrence.mate_id].append(occurrence)
        for mate_id, occurrences in by_mate.items():
            if len(occurrences) < 2:
                continue
            reference = occurrences[0].axis_vector
            with self.subTest(mate_id=mate_id):
                for occurrence in occurrences[1:]:
                    self.assertAlmostEqual(
                        abs(_dot(reference, occurrence.axis_vector)), 1.0,
                    )

    def test_hole_centers_match_corrected_integration_contract(self):
        # MGN rail pattern: 6 x 25 mm, selected HIWIN E1/E2 = 10/15 mm.
        self.assertEqual(
            self.geometry.rail_hole_z,
            tuple(PARAMS.rail_z0 + 10.0 + 25.0 * i for i in range(6)),
        )
        rail_screws = [o for o in self.links["static"]
                       if o.schedule_id == "rail_screws"]
        self.assertEqual(
            {(o.mate_center[0], o.mate_center[2]) for o in rail_screws},
            {(x, z) for x in (-PARAMS.rail_x, PARAMS.rail_x)
             for z in self.geometry.rail_hole_z},
        )

        # MGN12H blocks align to the plate's 20 x 20 grids at HOME.
        block_screws = [o for o in self.links["carriage"]
                        if o.schedule_id == "block_screws"]
        self.assertEqual(
            {(o.origin[0], o.origin[2]) for o in block_screws},
            {(sx * PARAMS.rail_x + dx, PARAMS.m0_home_standoff + dz)
             for sx in (-1, 1) for dx in (-10.0, 10.0)
             for dz in (-10.0, 10.0)},
        )

        # Flyer block and M2 mount use the current post slot centerlines.
        flyer_block = [o for o in self.links["static"]
                       if o.schedule_id == "flyer_block_screws"]
        self.assertEqual(
            {(o.mate_center[0], o.mate_center[1], o.mate_center[2])
             for o in flyer_block},
            {(sx * PARAMS.post_x, y, PARAMS.post_z[1])
             for sx in (-1, 1) for y in (-12.0, 12.0)},
        )

    def test_every_placement_issue_is_eliminated(self):
        issues = placements.placement_issues(PARAMS, self.geometry)
        self.assertEqual(issues, [])
        self.assertTrue(all(o.plausible for o in self.all_occurrences))
        self.assertTrue(all(not o.issue for o in self.all_occurrences))

    def test_shared_geometry_patch_table_is_exact_and_complete(self):
        patches = placements.required_geometry_patches()
        self.assertEqual(
            {patch.feature for patch in patches},
            {
                "endstop_pedestal_side_ears",
                "endstop_switch_rear_nut_pockets",
                "felt_captive_jam_nut",
            },
        )
        self.assertTrue(all(patch.source.startswith("cad/") for patch in patches))
        self.assertTrue(all(patch.current and patch.required and
                            patch.selected_hardware for patch in patches))

        by_feature = {patch.feature: patch for patch in patches}
        self.assertIn("x=-19,+19", by_feature["endstop_pedestal_side_ears"].required)
        self.assertIn("y=-226..-207.0",
                      by_feature["endstop_switch_rear_nut_pockets"].required)
        self.assertIn("x=-3.25,+3.25",
                      by_feature["endstop_switch_rear_nut_pockets"].required)
        self.assertIn("AF7.2 x 3.4", by_feature["felt_captive_jam_nut"].required)

    def test_replacement_hardware_lengths_and_catalog_variants(self):
        schedule = {item["id"]: item for item in hardware.HARDWARE_SCHEDULE}
        self.assertEqual(schedule["carriage_m4_screws"]["sku"],
                         "ISO4762-M4x20")
        self.assertEqual(schedule["carriage_flag_m4_screws"]["sku"],
                         "ISO4762-M4x25")
        self.assertEqual(schedule["nut_bracket_m4_screws"]["sku"],
                         "ISO4762-M4x25")
        self.assertEqual(schedule["endstop_pedestal_screws"]["sku"],
                         "ISO4762-M5x12")
        self.assertEqual(schedule["endstop_switch_screws"]["sku"],
                         "ISO4762-M2x16")
        self.assertEqual(schedule["t8_nut_screws"]["sku"],
                         "ISO4762-M3x12")
        self.assertNotIn("t8_nut_nylocs", schedule)
        self.assertEqual(schedule["felt_stud"]["sku"], "DIN976-M4x55")
        self.assertEqual(schedule["m3_base_screws"]["sku"],
                         "ISO10642-M5x12")
        self.assertEqual(schedule["dancer_moving_anchor_screw"]["sku"],
                         "ISO14581-M2x16")
        self.assertEqual(schedule["felt_compression_spring"]["sku"],
                         "94125K614")
        self.assertEqual(schedule["felt_spring_thrust_washer"]["sku"],
                         "91116A130")
        self.assertEqual(schedule["counterweight_inserts"]["sku"],
                         "MCMASTER-94459A130")
        self.assertEqual(schedule["front_trim_screws"]["sku"],
                         "ISO4762-M2x8")
        self.assertEqual(schedule["flyer_guide_screws"]["sku"],
                         "ISO4762-M2x6")
        self.assertEqual(schedule["cap_retention_screws"]["sku"],
                         "ISO4762-M2x20")
        self.assertEqual(schedule["active_sector_m3_screws"]["sku"],
                         "ISO4762-M3x14")
        self.assertEqual(schedule["active_sector_m4_screws"]["sku"],
                         "ISO4762-M4x10")
        self.assertEqual(schedule["dancer_pulley_shims"]["qty"], 7)
        self.assertEqual(schedule["m0_support_tnuts"]["sku"],
                         "MISUMI-HNTAJ5-5")
        self.assertEqual(schedule["dancer_stop_screws"]["sku"],
                         "ISO4762-M3x10")
        self.assertEqual(schedule["dancer_stop_inserts"]["sku"],
                         "MCMASTER-94459A769")
        self.assertEqual(schedule["dancer_extension_spring"]["sku"],
                         "LEM050AB01")

    def test_dancer_stop_occurrence_labels_match_m3x10_schedule(self):
        labels = {
            occurrence.label
            for occurrence in self.links["static"]
            if occurrence.schedule_id == "dancer_stop_screws"
        }
        self.assertEqual(
            labels,
            {"dancer_stop_1_m3x10", "dancer_stop_2_m3x10"},
        )

    def test_geometry_override_is_accepted_without_importing_assembly(self):
        moved = placements.current_geometry(PARAMS)
        moved = placements._resolve_geometry(
            PARAMS, {"rail_counterbore_y": moved.rail_counterbore_y + 0.25},
        )
        screws = [o for o in placements.static_occurrences(PARAMS, moved)
                  if o.schedule_id == "rail_screws"]
        self.assertTrue(screws)
        self.assertTrue(all(
            math.isclose(o.origin[1], self.geometry.rail_counterbore_y + 0.25)
            for o in screws
        ))

    def test_one_occurrence_per_catalog_item_builds_and_keeps_label(self):
        # Building every repeated screw adds no coverage; one transformed
        # occurrence for every used schedule model validates the catalog,
        # cardinal-axis transforms and bracket-frame transform.
        first_by_schedule = {}
        for occurrence in self.all_occurrences:
            first_by_schedule.setdefault(occurrence.schedule_id, occurrence)
        for schedule_id, occurrence in first_by_schedule.items():
            with self.subTest(schedule_id=schedule_id):
                part = occurrence.build()
                self.assertEqual(part.label, occurrence.label)
                self.assertTrue(part.is_valid)
                self.assertGreater(part.volume, 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
