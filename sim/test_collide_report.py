"""Fast fail-closed verdict tests for the full collision sweep report."""

import math
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import collide
from collide import proof_passed


class CollisionReportVerdictTests(unittest.TestCase):
    def test_clearance_at_target_passes(self):
        worst = {("flyer", "static"): (2.0, (0.0, 0.0, 0.0, 0.0))}
        self.assertTrue(proof_passed([], worst, 2.0))

    def test_collision_always_fails(self):
        worst = {("flyer", "static"): (3.0, (0.0, 0.0, 0.0, 0.0))}
        self.assertFalse(proof_passed([{"pair": ("flyer", "static")}],
                                     worst, 2.0))

    def test_subtarget_or_nonfinite_clearance_fails(self):
        pose = (0.0, 0.0, 0.0, 0.0)
        self.assertFalse(proof_passed(
            [], {("flyer", "static"): (1.999, pose)}, 2.0))
        self.assertFalse(proof_passed(
            [], {("flyer", "static"): (math.nan, pose)}, 2.0))

    def test_empty_or_malformed_evidence_fails(self):
        self.assertFalse(proof_passed([], {}, 2.0))
        self.assertFalse(proof_passed(
            [], {("flyer", "static"): (None, None)}, 2.0))

    def test_manifest_loader_honors_explicit_and_runtime_links_roots(self):
        payload = {"schema": "isolated-integrated-links/test"}
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "manifest.json").write_text(json.dumps(payload))
            self.assertEqual(collide.load_manifest(root), payload)

            previous = collide.LINKS
            try:
                collide.LINKS = root
                self.assertEqual(collide.load_manifest(), payload)
            finally:
                collide.LINKS = previous

    def test_successor_belt_mesh_is_the_only_new_flyer_static_exemption(self):
        exempt = collide.EXEMPT[("flyer", "static")]["static"]
        self.assertIn("m2_successor_210_3gt_6_belt", exempt)
        self.assertNotIn("m2_Leadshine_NBK_P30_3GT_split_clamp_BNW_motor_pulley", exempt)

    def test_collision_assets_support_legacy_and_explicit_filenames(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = {
                "parts": {
                    "static": ["legacy_label"],
                    "flyer": {
                        "display label": "001_display_label.stl",
                        "nested": {"file": "002_nested.stl"},
                    },
                }
            }
            assets = collide.resolve_part_assets(manifest, root)
            self.assertEqual(
                assets["static"]["legacy_label"],
                (root / "parts" / "static" / "legacy_label.stl").resolve(),
            )
            self.assertEqual(
                assets["flyer"]["display label"].name,
                "001_display_label.stl",
            )
            self.assertEqual(assets["flyer"]["nested"].name, "002_nested.stl")

            manifest["parts"]["flyer"]["escape"] = "..\\escape.stl"
            with self.assertRaises(ValueError):
                collide.resolve_part_assets(manifest, root)


if __name__ == "__main__":
    unittest.main()
