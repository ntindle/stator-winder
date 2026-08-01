"""Regression checks for absolute-placement same-link BREP booleans."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "cad"))
sys.path.insert(0, str(HERE))

import assembly  # noqa: E402
import frame_hardware_audit  # noqa: E402
import hardware_placements  # noqa: E402
from params import PARAMS as P  # noqa: E402
import static_audit  # noqa: E402


class StaticAuditPlacementTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parts = {part.label: part for part in assembly.static_link()}

    def test_m2_pulley_overlaps_only_the_five_mm_motor_shaft(self):
        volume = static_audit._source_volume(
            self.parts["m2_motor"], self.parts["m2_motor_pulley"]
        )
        self.assertGreater(volume, 30.0)
        self.assertLess(volume, 45.0)

    def test_belt_does_not_interpenetrate_motor_body(self):
        volume = static_audit._source_volume(
            self.parts["m2_motor"], self.parts["gt2_belt"]
        )
        self.assertAlmostEqual(volume, 0.0, places=6)


class StaticAuditRepairTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.links = {
            link: {part.label: part for part in parts}
            for link, parts in assembly.build_links(
                final_wound_collision=True).items()
        }

    def assertNoPositiveOverlap(self, link, a, b):
        self.assertAlmostEqual(
            static_audit._source_volume(
                self.links[link][a], self.links[link][b]),
            0.0,
            places=6,
            msg=f"[{link}] {a} vs {b}",
        )

    def test_kw12_screws_follow_verified_vendor_hole_centers(self):
        for index in (1, 2):
            self.assertNoPositiveOverlap(
                "static", "endstop", f"endstop_switch_m2x16_{index}")
        occurrences = {
            item.label: item
            for item in hardware_placements.static_occurrences(P)
        }
        self.assertEqual(
            tuple(occurrences[
                f"endstop_switch_m2x16_{index}"].origin[0]
                for index in (1, 2)),
            P.endstop_switch_hole_x,
        )

    def test_complete_t8_set_clears_plate_and_front_side_screws(self):
        for label in ("t8_nut_spring", "t8_nut_secondary"):
            self.assertNoPositiveOverlap(
                "carriage", "fabricated_carriage_0p250in_mic6", label)
        for index in range(1, 5):
            self.assertNoPositiveOverlap(
                "carriage", "t8_nut_spring",
                f"t8_flange_m3x12_{index}")
        labels = set(self.links["carriage"])
        self.assertFalse(any("t8_flange_nyloc" in label for label in labels))
        self.assertFalse(any("t8_flange_m3x18" in label for label in labels))

    def test_t8_interlock_is_the_only_new_named_positive_fit(self):
        volume = static_audit._source_volume(
            self.links["carriage"]["t8_nut_main"],
            self.links["carriage"]["t8_nut_secondary"],
        )
        self.assertGreater(volume, 100.0)
        occurrences = static_audit._occurrences()
        frame_hosts = frame_hardware_audit.bracket_host_pairs(
            hardware_placements.current_geometry(P))
        self.assertEqual(
            static_audit._classify_positive(
                "carriage", "t8_nut_main", "t8_nut_secondary",
                occurrences, frame_hosts),
            "Zyltech anti-backlash complementary interlock envelope",
        )

    def test_legacy_link_does_not_duplicate_candidate_counterweights(self):
        # The six serialized balance stacks are owned by the integrated
        # release candidate.  This legacy static-audit link must not recreate
        # the removed generic M3/three-washer occurrence.
        labels = set(self.links["flyer"])
        self.assertNotIn("counterweight_m3_insert", labels)
        self.assertNotIn("counterweight_m3x12", labels)
        self.assertFalse(any(
            label.startswith("counterweight_washer_m3_")
            for label in labels
        ))


if __name__ == "__main__":
    unittest.main(verbosity=2)
