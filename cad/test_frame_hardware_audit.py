"""Deterministic source-BREP tests for frame_hardware_audit.py."""

from __future__ import annotations

import unittest

import frame_hardware_audit as audit
import hardware
import hardware_placements as placements
from params import PARAMS as P


class FrameHardwareAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.current_geometry = placements.current_geometry(P)
        cls.corrected_geometry = audit.proposed_layout(cls.current_geometry)
        cls.current = audit.audit_bracket_bodies(
            "current", cls.current_geometry)
        cls.corrected = audit.audit_bracket_bodies(
            "corrected", cls.corrected_geometry)
        cls.shoe = audit.audit_rear_post_left_shoe(cls.corrected_geometry)

    def test_integrated_production_hbkt_layout_is_green(self):
        self.assertEqual(self.current.bracket_count, 15)
        self.assertEqual(self.current.positive_volume_pairs, 0)
        self.assertEqual(self.current.forbidden_positive_volume_pairs, 0)

    def test_corrected_hbkt_bodies_have_no_positive_volume_collision(self):
        self.assertEqual(self.corrected.bracket_count, 15)
        self.assertEqual(self.corrected.positive_volume_pairs, 0)
        self.assertEqual(self.corrected.forbidden_positive_volume_pairs, 0)

    def test_exact_reposition_and_replacement_contract(self):
        frames = {frame.label: frame for frame in self.corrected_geometry.frame_brackets}
        self.assertNotIn("frame_bracket_rear_post_left", frames)
        expected_origins = {
            "frame_bracket_rear_base_L": (-80.0, -225.0, -170.0),
            "frame_bracket_mid_base_L": (-80.0, -225.0, -40.0),
            "frame_bracket_mid_base_R": (80.0, -225.0, -40.0),
            "frame_bracket_front_base_L": (-80.0, -225.0, 170.0),
            "frame_bracket_front_base_R": (80.0, -225.0, 170.0),
            "frame_bracket_front_stringer_L": (-45.0, -225.0, 170.0),
            "frame_bracket_front_stringer_R": (45.0, -225.0, 170.0),
        }
        for label, origin in expected_origins.items():
            frame = frames[label]
            self.assertEqual(frame.origin, origin)
            self.assertEqual(frame.x_dir, (1.0, 0.0, 0.0))
            self.assertEqual(frame.y_dir, (0.0, 0.0, 1.0))
            self.assertEqual(frame.z_dir, (0.0, -1.0, 0.0))

    def test_repositioned_host_roles_are_swapped_correctly(self):
        hosts = audit.bracket_host_pairs(self.corrected_geometry)
        self.assertEqual(hosts["frame_bracket_front_stringer_L"],
                         ("stringer_L", "cross_front"))
        self.assertEqual(hosts["frame_bracket_mid_base_R"],
                         ("base_rail_R", "cross_mid"))
        self.assertEqual(hosts["frame_bracket_mid_stringer_L"],
                         ("cross_mid", "stringer_L"))

    def test_t8_post_bracket_has_real_production_tolerance_clearance(self):
        hosts = audit.source_static_hosts()
        frame = next(frame for frame in self.current_geometry.frame_brackets
                     if frame.label == "frame_bracket_post_L_front")
        bracket = frame.location * hardware.angle_bracket_2020(frame.label)
        self.assertAlmostEqual(
            audit.common_volume_mm3(bracket, hosts["t8_screw"].shape),
            0.0, places=6)
        self.assertGreaterEqual(float(bracket.distance_to(
            hosts["t8_screw"].shape)), 1.999)

    def test_front_z_hbkt_variant_is_rejected_by_exact_common(self):
        self.assertAlmostEqual(
            audit.rejected_front_z_rear_post_volume_mm3(),
            1810.303324, places=3)

    def test_original_custom_shoe_fastener_centers_fail(self):
        shoe = audit.rear_post_left_shoe(corrected=False)
        fasteners = audit.rear_post_left_shoe_hardware(corrected=False)
        self.assertAlmostEqual(audit.common_volume_mm3(
            shoe, fasteners["rear_post_left_shoe_floor_m5x12"]),
            62.827142, places=3)
        self.assertAlmostEqual(audit.common_volume_mm3(
            fasteners["rear_post_left_shoe_floor_m5x12"],
            fasteners["rear_post_left_shoe_upright_m5x12"]),
            12.713730, places=3)

    def test_corrected_custom_shoe_and_hardware_pass(self):
        result = self.shoe
        self.assertEqual(result.shoe_cross_common_volume_mm3, 0.0)
        self.assertEqual(result.shoe_post_common_volume_mm3, 0.0)
        self.assertEqual(result.base_rail_gap_mm, 1.0)
        self.assertEqual(result.nearest_repositioned_bracket_gap_mm, 1.0)
        self.assertEqual(result.forbidden_positive_volume_pairs, ())
        self.assertEqual(
            {row[3] for row in result.allowed_positive_volume_pairs},
            {"tslot_capture", "tslot_passage_envelope"},
        )
        self.assertGreaterEqual(result.floor_bore_min_ligament_mm, P.min_wall)
        self.assertGreaterEqual(result.upright_bore_min_ligament_mm, P.min_wall)
        self.assertGreaterEqual(result.scallop_back_wall_mm, P.min_wall)
        self.assertGreaterEqual(
            result.scallop_to_upright_bore_ligament_mm, P.min_wall)
        self.assertGreater(result.floor_head_min_edge_margin_mm, 0.0)
        self.assertGreater(result.upright_head_min_edge_margin_mm, 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
