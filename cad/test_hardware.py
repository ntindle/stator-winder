"""Deterministic geometry and schedule checks for cad/hardware.py."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hardware  # noqa: E402


def bbox_size(part):
    bb = part.bounding_box()
    return (bb.size.X, bb.size.Y, bb.size.Z)


class HardwareGeometryTests(unittest.TestCase):
    def assert_valid(self, part):
        self.assertTrue(part.is_valid)
        self.assertGreater(part.volume, 0.0)
        self.assertGreaterEqual(len(part.solids()), 1)

    def assert_dims(self, part, expected, places=5):
        for actual, wanted in zip(bbox_size(part), expected):
            self.assertAlmostEqual(actual, wanted, places=places)

    def test_iso_fasteners_are_valid(self):
        parts = [
            hardware.socket_head_cap_screw("M3", 10),
            hardware.set_screw("M3", 8),
            hardware.hex_nut("M4"),
            hardware.plain_washer("M3"),
            hardware.heat_set_insert("M3"),
        ]
        for part in parts:
            with self.subTest(part=part.label):
                self.assert_valid(part)

        screw_bb = parts[0].bounding_box()
        self.assertAlmostEqual(screw_bb.min.Z, -10.0, places=5)
        self.assertAlmostEqual(screw_bb.max.Z, 3.0, places=5)
        # bd_warehouse uses the ISO 7089 maximum nominal sheet thickness.
        self.assert_dims(parts[3], (7.0, 7.0, 0.55))
        self.assert_dims(parts[4], (4.7, 4.7, 5.7))

    def test_catalog_interfaces(self):
        bracket = hardware.angle_bracket_2020()
        tnut = hardware.tnut_slot6("M5")
        short_tnut = hardware.tnut_slot6_short_m5()
        pulley_axle = hardware.shoulder_screw_90265a420()
        pivot = hardware.dancer_pivot_shoulder_screw()
        collar = hardware.shaft_collar(8)
        ring = hardware.retaining_ring_external(8)
        for part in (bracket, tnut, short_tnut, pulley_axle, pivot, collar, ring):
            with self.subTest(part=part.label):
                self.assert_valid(part)

        self.assert_dims(bracket, (20.0, 25.0, 25.0))
        self.assert_dims(tnut, (8.0, 15.0, 3.2))
        self.assert_dims(short_tnut, (8.0, 10.0, 3.2))
        self.assert_dims(pulley_axle, (5.0, 5.0, 22.0))
        self.assert_dims(pivot, (9.0, 9.0, 19.0))
        self.assert_dims(collar, (16.0, 16.0, 8.0))
        self.assertAlmostEqual(bbox_size(ring)[2], 0.8, places=5)

    def test_placement_maps_local_axis(self):
        screw = hardware.socket_head_cap_screw("M3", 10)
        placed = hardware.place(screw, (100, 20, -5), axis="+x")
        bb = placed.bounding_box()
        # Head is +axis, shank is -axis, and the head-bearing plane is x=100.
        self.assertAlmostEqual(bb.min.X, 90.0, places=5)
        self.assertAlmostEqual(bb.max.X, 103.0, places=5)
        self.assertAlmostEqual((bb.min.Y + bb.max.Y) / 2, 20.0, places=5)
        self.assertAlmostEqual((bb.min.Z + bb.max.Z) / 2, -5.0, places=5)


class HardwareScheduleTests(unittest.TestCase):
    def test_schedule_ids_are_unique_and_models_build(self):
        ids = [item["id"] for item in hardware.HARDWARE_SCHEDULE]
        self.assertEqual(len(ids), len(set(ids)))
        for item in hardware.HARDWARE_SCHEDULE:
            self.assertGreater(item["qty"], 0)
            if item["model"] is None:
                self.assertEqual(item["status"], "selection_pending")
                continue
            with self.subTest(item=item["id"]):
                self.assertTrue(hardware.make_scheduled_part(item["id"]).is_valid)

    def test_audit_quantities(self):
        by_id = {item["id"]: item["qty"] for item in hardware.HARDWARE_SCHEDULE}
        expected = {
            "frame_brackets": 15,
            "frame_bracket_screws": 30,
            "frame_bracket_tnuts": 30,
            "rear_post_shoe_screws": 2,
            "rear_post_shoe_tnuts": 2,
            "machine_feet": 4,
            "machine_foot_standoffs": 4,
            "machine_foot_set_screws": 4,
            "rail_screws": 12,
            "rail_tnuts": 12,
            "block_screws": 8,
            "t8_nut_screws": 4,
            "carriage_m4_screws": 2,
            "carriage_flag_m4_screws": 2,
            "nut_bracket_m4_screws": 2,
            "motor_screws": 12,
            "m0_mount_screws": 2,
            "m0_support_screws": 2,
            "endstop_pedestal_screws": 2,
            "endstop_switch_screws": 2,
            "flyer_block_screws": 4,
            "m2_mount_screws": 4,
            "m3_base_screws": 8,
            "dancer_base_tnuts": 2,
            "flyer_set_screws": 2,
            "flyer_set_inserts": 2,
            "counterweight_screws": 4,
            "counterweight_inserts": 4,
            "front_trim_screws": 2,
            "front_trim_washers": 2,
            "front_trim_inserts": 2,
            "flyer_guide_screws": 3,
            "flyer_guide_inserts": 3,
            "cap_retention_screws": 3,
            "cap_retention_washers": 6,
            "cap_retention_nylocs": 3,
            "active_sector_m3_screws": 4,
            "active_sector_m3_washers": 4,
            "active_sector_m3_inserts": 4,
            "active_sector_m4_screws": 4,
            "active_sector_m4_washers": 4,
            "active_sector_m4_inserts": 4,
            "dancer_pulley_shoulder": 1,
            "dancer_pivot_shoulder": 1,
            "dancer_stop_screws": 2,
            "dancer_stop_inserts": 2,
            "dancer_extension_spring": 1,
            "spool_axle": 1,
            "felt_stud": 1,
        }
        for item_id, qty in expected.items():
            self.assertEqual(by_id[item_id], qty, item_id)

        procurement = {row["sku"]: row for row in
                       hardware.procurement_schedule()}
        self.assertEqual(procurement["MISUMI-HNTA5-5"]["required_qty"], 54)
        self.assertEqual(procurement["MISUMI-HNTA5-5"]["order_qty"], 60)
        self.assertEqual(procurement["MISUMI-HNTAJ5-5"]["required_qty"], 4)
        self.assertEqual(procurement["MISUMI-HNTAJ5-5"]["order_qty"], 5)
        self.assertEqual(procurement["MISUMI-CBSA5-10"]["required_qty"], 30)
        self.assertEqual(procurement["MISUMI-CBSA5-10"]["order_qty"], 34)
        self.assertEqual(procurement["ISO4762-M5x12"]["required_qty"], 10)
        self.assertEqual(procurement["ISO10642-M5x12"]["required_qty"], 10)
        self.assertEqual(procurement["ISO4762-M3x10"]["required_qty"], 22)
        self.assertEqual(procurement["MCMASTER-94459A769"]["required_qty"], 2)
        self.assertEqual(procurement["MCMASTER-94459A130"]["required_qty"], 8)
        self.assertEqual(procurement["MCMASTER-94459A120"]["required_qty"], 5)
        self.assertEqual(procurement["ISO7089-M2"]["required_qty"], 14)
        self.assertEqual(procurement["ISO7089-M3"]["required_qty"], 10)
        self.assertEqual(procurement["ISO7089-M4"]["required_qty"], 10)
        self.assertEqual(procurement["ISO10511-M2"]["required_qty"], 7)
        self.assertEqual(procurement["ISO4762-M3x12"]["required_qty"], 4)
        self.assertEqual(procurement["ISO4026-M3x8"]["required_qty"], 2)
        self.assertEqual(procurement["ISO10642-M3x6"]["required_qty"], 4)
        self.assertEqual(procurement["ISO4762-M2x6"]["required_qty"], 3)
        self.assertEqual(procurement["ISO4762-M2x8"]["required_qty"], 2)
        self.assertEqual(procurement["ISO4762-M2x20"]["required_qty"], 3)
        self.assertEqual(procurement["ISO4762-M3x14"]["required_qty"], 4)
        self.assertEqual(procurement["ISO4762-M4x10"]["required_qty"], 4)
        self.assertEqual(procurement["ELESA-432001"]["required_qty"], 4)
        self.assertEqual(procurement["WURTH-970180581"]["required_qty"], 4)
        self.assertEqual(procurement["ISO4026-M5x12"]["required_qty"], 4)

    def test_schedule_serializes(self):
        assembly_json = hardware.schedule_json()
        procurement_json = hardware.schedule_json(procurement=True)
        self.assertIn('"frame_brackets"', assembly_json)
        self.assertIn('"order_qty"', procurement_json)


if __name__ == "__main__":
    unittest.main(verbosity=2)
