"""Regression tests for the selected P30/210-3GT-6 belt evidence."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import belt_audit as audit  # noqa: E402


class SelectedM2BeltAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = json.loads(audit.REPORT.read_text(encoding="utf-8"))

    def test_report_is_current_self_hashed_pass_evidence(self) -> None:
        self.assertEqual(self.report["schema"], audit.SCHEMA)
        self.assertEqual(self.report["status"], "PASS")
        self.assertTrue(self.report["passed"])
        self.assertTrue(self.report["geometry_authorized"])
        self.assertFalse(self.report["production_authorized"])
        self.assertEqual(
            self.report["report_sha256"], audit._canonical_hash(self.report),
        )
        self.assertEqual(
            set(self.report["source_hashes"]), set(audit.SOURCE_PATHS),
        )
        for relative, expected in self.report["source_hashes"].items():
            self.assertEqual(
                expected,
                audit._sha256(ROOT / relative),
                msg=f"stale belt-audit input: {relative}",
            )
        self.assertEqual(self.report["unexpected"], [])
        self.assertTrue(all(self.report["checks"].values()))

    def test_selected_lane_and_complete_revolution_are_explicit(self) -> None:
        lane = self.report["lane"]
        self.assertEqual(lane["motor_teeth"], 30)
        self.assertEqual(lane["flyer_teeth"], 30)
        self.assertEqual(lane["belt_model"], "210-3GT-6")
        self.assertEqual(lane["belt_pitch_length_mm"], 210.0)
        self.assertEqual(lane["belt_width_mm"], 6.0)
        self.assertEqual(lane["center_distance_mm"], 60.0)
        self.assertEqual(lane["belt_label"], "m2_successor_210_3gt_6_belt")

        sampling = self.report["sampling"]
        self.assertEqual(sampling["start_deg_inclusive"], 0.0)
        self.assertEqual(sampling["stop_deg_exclusive"], 360.0)
        self.assertEqual(sampling["step_deg"], 1.0)
        self.assertEqual(sampling["sample_count"], 360)
        self.assertTrue(sampling["complete_revolution"])
        self.assertEqual(audit._angles(1.0), [float(i) for i in range(360)])
        with self.assertRaises(ValueError):
            audit._angles(7.0)

    def test_only_the_two_tooth_engagements_are_exempt(self) -> None:
        policy = self.report["exemption_policy"]
        self.assertEqual(
            policy["allowed_positive_contact_pairs"],
            [audit.MOTOR_ENGAGEMENT_PAIR, audit.FLYER_ENGAGEMENT_PAIR],
        )
        self.assertTrue(policy["all_other_belt_contacts_forbidden"])
        self.assertFalse(policy["generic_collision_gate_modified"])

        engagements = {
            row["pair"]: row for row in self.report["intended_engagements"]
        }
        self.assertEqual(
            set(engagements),
            {audit.MOTOR_ENGAGEMENT_PAIR, audit.FLYER_ENGAGEMENT_PAIR},
        )
        for row in engagements.values():
            self.assertGreater(row["exact_overlap_mm3"], audit.BOOLEAN_TOLERANCE_MM3)
            self.assertTrue(row["tooth_band_only"])
            expected_min, expected_max = row[
                "expected_tooth_engagement_radial_band_mm"
            ]
            self.assertGreaterEqual(
                row["minimum_radius_from_pulley_axis_mm"], expected_min,
            )
            self.assertLessEqual(
                row["maximum_radius_from_pulley_axis_mm"], expected_max,
            )
        flyer = engagements[audit.FLYER_ENGAGEMENT_PAIR]
        self.assertEqual(flyer["sample_count"], 360)
        self.assertEqual(flyer["contact_count"], 360)
        self.assertTrue(flyer["contact_at_every_sample"])
        self.assertEqual(flyer["missing_contact_angles_deg"], [])

    def test_every_other_rotating_flyer_part_clears_the_belt(self) -> None:
        rows = self.report["rotating_non_engagement_parts"]
        summary = self.report["summary"]
        self.assertEqual(summary["rotating_part_count_total"], 42)
        self.assertEqual(summary["rotating_non_engagement_part_count"], 41)
        self.assertEqual(summary["rotating_query_count"], 41 * 360)
        self.assertEqual(len(rows), 41)
        self.assertEqual(len({row["part_key"] for row in rows}), 41)
        self.assertNotIn("flyer_pulley", {row["part_key"] for row in rows})
        for row in rows:
            self.assertTrue(row["ok"], msg=row["part_key"])
            self.assertEqual(row["sample_count"], 360)
            self.assertEqual(row["collision_count"], 0)
            self.assertEqual(row["collision_angles_deg"], [])
            self.assertGreaterEqual(
                row["minimum_clearance_mm"], audit.CLEARANCE_TARGET_MM,
                msg=row["part_key"],
            )

    def test_relevant_static_hardware_uses_exact_brep_and_clears(self) -> None:
        rows = self.report["static_non_engagement_parts"]
        by_group: dict[str, list[dict]] = {}
        for row in rows:
            by_group.setdefault(row["group"], []).append(row)
        self.assertEqual(
            {group: len(group_rows) for group, group_rows in by_group.items()},
            {
                "successor_drive": 10,
                "shifted_support": 21,
                "shifted_entry": 6,
                "configured_wire": 1,
            },
        )
        drive_keys = {
            row["part_key"] for row in by_group["successor_drive"]
        }
        self.assertTrue({
            "mount",
            "motor",
            "motor_pulley_BNW_hole_path_0",
            "motor_pulley_BNW_hole_path_1",
            "motor_pulley_BNW_set_screw_0",
            "motor_pulley_BNW_set_screw_1",
            "motor_screw_0",
            "motor_screw_1",
            "motor_screw_2",
            "motor_screw_3",
        }.issubset(drive_keys))
        self.assertEqual(
            by_group["configured_wire"][0]["part_key"],
            "configured_static_supply_wire",
        )
        for row in rows:
            self.assertTrue(row["ok"], msg=f'{row["group"]}:{row["part_key"]}')
            self.assertFalse(row["positive_overlap"])
            self.assertLessEqual(row["overlap_mm3"], audit.BOOLEAN_TOLERANCE_MM3)
            self.assertGreaterEqual(
                row["clearance_mm"], audit.CLEARANCE_TARGET_MM,
                msg=f'{row["group"]}:{row["part_key"]}',
            )
            self.assertTrue(row["BREP"]["valid"])
            self.assertEqual(
                row["BREP"]["method"],
                "exact_OCC_distance_to_and_common_volume",
            )

    def test_exception_evidence_is_fail_closed(self) -> None:
        failure = audit.failure_report("synthetic failure")
        self.assertEqual(failure["status"], "FAIL")
        self.assertFalse(failure["passed"])
        self.assertFalse(failure["geometry_authorized"])
        self.assertFalse(failure["production_authorized"])
        self.assertFalse(failure["checks"]["audit_completed_without_exception"])
        self.assertEqual(failure["unexpected"][0]["kind"], "audit_exception")
        self.assertEqual(failure["report_sha256"], audit._canonical_hash(failure))


if __name__ == "__main__":
    unittest.main(verbosity=2)
