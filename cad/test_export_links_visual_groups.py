"""Source-level special-material export contract for legacy links."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import assembly  # noqa: E402
import export_links  # noqa: E402


class ExportLinkVisualGroupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.links = assembly.build_links()

    def test_legacy_export_only_separates_the_two_felt_pads(self):
        self.assertEqual(set(export_links.VISUAL_EXPORT_GROUPS), {"felt_pads"})
        base, groups = export_links._split_visual_parts(
            "static", self.links["static"]
        )
        self.assertEqual(set(groups), {"felt_pads"})
        self.assertEqual(len(groups["felt_pads"]), 2)
        self.assertEqual(
            {part.label for part in groups["felt_pads"]},
            {"felt_pad_fixed", "felt_pad_moving"},
        )
        self.assertEqual(
            len(base) + len(groups["felt_pads"]), len(self.links["static"])
        )

    def test_flyer_does_not_require_removed_generic_counterweights(self):
        base, groups = export_links._split_visual_parts(
            "flyer", self.links["flyer"]
        )
        self.assertEqual(groups, {})
        self.assertEqual(len(base), len(self.links["flyer"]))
        labels = {part.label for part in self.links["flyer"]}
        self.assertFalse(any(
            label == "counterweight_m3_insert"
            or label == "counterweight_m3x12"
            or label.startswith("counterweight_washer_m3_")
            for label in labels
        ))


if __name__ == "__main__":
    unittest.main(verbosity=2)
