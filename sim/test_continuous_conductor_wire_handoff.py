"""Focused fail-closed tests for the supply-to-flyer route seam."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import continuous_conductor_route as route  # noqa: E402


def _manifest() -> dict:
    bore = [
        [0.0, 0.0, -42.0],
        [0.0, 12.0, -30.12],
        [0.0, 67.0, -17.0],
    ]
    return {
        "wire": {
            "static": {
                "points": [
                    [-39.0, -135.0, -157.0],
                    [0.0, 0.0, -110.75],
                    [0.0, 0.0, -42.0],
                ],
                "landmarks": {
                    "shaft_bore_rear": [0.0, 0.0, -110.75],
                    "guide_root": [0.0, 0.0, -42.0],
                },
            },
            "flyer": {"points": bore},
            "active_terminal_guide": {
                "bore_centerline_local_mm": bore,
                "unproved_transition_origin_local_mm": bore[-1],
            },
        }
    }


class ContinuousConductorWireHandoffTests(unittest.TestCase):
    def test_on_axis_through_shaft_handoff_is_exact_and_stays_unproved_flexible(self):
        contract = route._wire_handoff_contract(_manifest())
        self.assertEqual(contract["status"], "PASS")
        self.assertEqual(
            contract["static_to_flyer_seam_local_mm"], [0.0, 0.0, -42.0]
        )
        self.assertEqual(contract["maximum_gap_mm"], 0.0)
        self.assertTrue(
            contract["static_owner_continues_through_shaft_to_guide_root"]
        )
        self.assertTrue(
            contract["static_to_flyer_handoff_is_M2_axis_invariant"]
        )
        self.assertFalse(contract["unsupported_flexible_intervals_authorized"])

    def test_old_rear_mouth_endpoint_is_rejected_as_a_visible_gap(self):
        manifest = deepcopy(_manifest())
        manifest["wire"]["static"]["points"][-1] = [0.0, 0.0, -100.0]
        with self.assertRaisesRegex(RuntimeError, "continuous wire has a gap"):
            route._wire_handoff_contract(manifest)

    def test_off_axis_static_handoff_is_rejected_even_if_both_sides_match(self):
        manifest = deepcopy(_manifest())
        off_axis = [0.1, 0.0, -42.0]
        manifest["wire"]["static"]["points"][-1] = off_axis
        manifest["wire"]["static"]["landmarks"]["guide_root"] = off_axis
        manifest["wire"]["flyer"]["points"][0] = off_axis
        manifest["wire"]["active_terminal_guide"][
            "bore_centerline_local_mm"
        ][0] = off_axis
        with self.assertRaisesRegex(RuntimeError, "not invariant"):
            route._wire_handoff_contract(manifest)


if __name__ == "__main__":
    unittest.main(verbosity=2)
