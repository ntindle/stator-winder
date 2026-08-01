"""Fail-closed report contract for the active-tooth shoe study."""

import json
from pathlib import Path
import unittest

import active_tooth_shoe_route as audit


REPORT = (
    Path(__file__).resolve().parent.parent
    / "out" / "reports" / "active_tooth_shoe.json"
)


class ActiveToothShoeRouteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = json.loads(REPORT.read_text())

    def test_full_required_sweep_is_present_and_rejected(self):
        sweep = self.report["route_sweep"]
        self.assertEqual(sweep["required_case_count"], 360 * 9 * 2)
        self.assertEqual(
            sweep["evaluated_case_count"], sweep["required_case_count"]
        )
        self.assertEqual(sweep["passing_case_count"], 0)
        self.assertEqual(sweep["status"], "FAIL")

    def test_release_and_integration_are_hard_false(self):
        self.assertEqual(self.report["status"], "DESIGN_NO_GO")
        self.assertFalse(self.report["release_authorized"])
        self.assertFalse(self.report["assembly_integration_authorized"])
        self.assertFalse(
            self.report["gates"]["common_rigid_blade_corridor"]
        )
        self.assertFalse(self.report["gates"]["exact_brep_insertion"])

    def test_exact_brep_witnesses_are_recorded(self):
        insertion = self.report["insertion"]
        self.assertEqual(
            insertion["method"], "exact OpenCascade BREP common volume"
        )
        self.assertGreater(
            insertion["rows"][7]["left_common_volume_mm3"], 0.02
        )
        self.assertGreater(
            insertion["rows"][8]["left_common_volume_mm3"], 0.83
        )
        self.assertEqual(insertion["status"], "FAIL")

    def test_rigid_and_extraction_results_are_not_overclaimed(self):
        self.assertEqual(self.report["extraction"]["status"], "PASS")
        rigid = self.report["rigid_motion"]
        self.assertEqual(rigid["flyer_360deg"]["status"], "PASS")
        self.assertEqual(rigid["chuck_9_depths"]["status"], "FAIL")
        self.assertEqual(
            rigid["m1_index_at_full_retraction"]["status"], "PASS"
        )
        self.assertEqual(self.report["cap_relief"]["status"], "NOT_RELEASED")
        self.assertEqual(
            self.report["material_finish_rfq"]["status"],
            "BLOCKED_UNQUALIFIED",
        )

    def test_representative_route_still_fails_without_report_shortcut(self):
        case = audit._evaluate_case(0, 90, 1)
        self.assertFalse(case.route_ok)
        self.assertIn("mouth_to_lay_not_C1", case.failures)


if __name__ == "__main__":
    unittest.main()

