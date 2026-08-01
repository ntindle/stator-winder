"""Regression tests for the isolated split-former no-go study."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import collapsible_former_study as study


REPORT = ROOT / "out" / "reports" / "collapsible_former.json"


def _report():
    return json.loads(REPORT.read_text(encoding="utf-8"))


class CollapsibleFormerStudyTests(unittest.TestCase):

    def test_report_hash_and_fail_closed_decision(self):
        report = _report()
        expected = report.pop("report_sha256")
        canonical = json.dumps(
            report, sort_keys=True, separators=(",", ":"), allow_nan=False,
        )
        self.assertEqual(hashlib.sha256(canonical.encode()).hexdigest(), expected)
        self.assertEqual(report["schema"], study.SCHEMA)
        self.assertEqual(report["status"], "DESIGN_NO_GO")
        self.assertIs(report["release_authorized"], False)
        self.assertIs(report["assembly_integration_authorized"], False)

    def test_binds_only_canonical_unmodified_upstream_capture(self):
        report = _report()
        capture = report["capture_contract"]
        self.assertEqual(
            capture["capture_path"], "out/capture/upstream_current_raw.jsonl",
        )
        self.assertEqual(
            capture["capture_sha256"],
            "350210c36550ae2c22e5352675aee6bd2eebb18325f3b5f45c7cb35d6a314958",
        )
        self.assertEqual(capture["controller_mode"], "upstream")
        self.assertIsNone(capture["adapter_sha256"])
        self.assertEqual(capture["velocities_rad_s"][:3], [20.0, 20.0, 20.0])
        self.assertEqual(
            hashlib.sha256(study.RAW_CAPTURE.read_bytes()).hexdigest(),
            capture["capture_sha256"],
        )

    def test_exact_job_signs_and_neighbor_histories_are_covered(self):
        capture = _report()["capture_contract"]
        self.assertEqual(capture["job"]["slots"], 24)
        self.assertAlmostEqual(capture["job"]["od_mm"], 46.0, places=12)
        self.assertAlmostEqual(capture["job"]["stack_mm"], 15.0, places=12)
        self.assertAlmostEqual(
            capture["job"]["wire_finished_d_mm"], 0.22352, places=12,
        )
        self.assertEqual(capture["motion_sign_counts"], {"-1": 12, "1": 12})
        self.assertEqual(
            capture["neighbor_case_counts"], {"0": 2, "1": 20, "2": 2},
        )

    def test_settings_only_speed_fix_leaves_enough_m0_stroke_and_time(self):
        report = _report()
        retract = report["capture_contract"]["post_pass_retract"]
        transfer = report["transfer_study"]["radial_transfer"]
        self.assertIs(retract["all_arrive_before_index"], True)
        self.assertGreater(retract["minimum_settling_margin_s"], 0.77)
        self.assertGreater(
            transfer["minimum_available_post_pass_M0_stroke_mm"], 18.2,
        )
        self.assertLess(
            transfer["minimum_shift_to_clear_shoe_and_liner_mm"], 8.22,
        )
        self.assertGreater(transfer["stroke_margin_mm"], 10.0)

    def test_raw_ease_law_cannot_define_the_exact_finished_pack(self):
        report = _report()
        ease = report["capture_contract"]["ease_law"]
        self.assertIs(ease["all_first_flyer_motion_inside_raw_wind_range"], True)
        self.assertAlmostEqual(ease["minimum_full_turn_pitch_mm"], 0.0, places=12)
        self.assertGreater(ease["maximum_full_turn_pitch_mm"], 0.707)
        self.assertGreaterEqual(ease["minimum_intervals_below_wire_per_pass"], 23)
        self.assertIs(ease["every_pass_has_zero_pitch_interval"], True)
        self.assertIs(
            report["gates"][
                "raw_ease_law_constructs_deterministic_50_turn_pack"
            ],
            False,
        )

    def test_two_half_transfer_has_negative_global_clearance_bound(self):
        witness = _report()["transfer_study"]["two_half_transfer_witness"]
        self.assertEqual(witness["status"], "FAIL")
        self.assertLess(witness["global_upper_bound_joint_margin_mm"], -0.068)
        self.assertAlmostEqual(
            witness["exact_OCC_core_margin_at_best_mm"],
            witness["polygon_core_margin_at_best_mm"],
            places=12,
        )
        self.assertLess(witness["neighbor_margin_at_best_mm"], 0.0)

    def test_tangent_lattice_jams_every_slot_fitting_affine_taper(self):
        taper = _report()["transfer_study"]["affine_taper_bound"]
        branches = taper["affine_shear_noncompression_branches"]
        wedge = taper["slot_wedge_feasible_shear_range_at_witness"]
        self.assertIs(taper["opposed_diagonal_bonds_present"], True)
        self.assertAlmostEqual(
            branches["large_branch_abs_k_min"], 2.0 * math.sqrt(3.0),
            places=12,
        )
        self.assertLess(
            abs(wedge["minimum_feasible_k"]),
            branches["large_branch_abs_k_min"],
        )
        self.assertLess(
            abs(wedge["maximum_feasible_k"]),
            branches["large_branch_abs_k_min"],
        )
        self.assertEqual(taper["status"], "FAIL")

    def test_no_production_files_were_authorized(self):
        report = _report()
        self.assertEqual(
            report["decision"]["production_CAD_controller_changes"], "NONE",
        )
        self.assertIs(report["scope"]["production_files_modified"], False)


if __name__ == "__main__":
    unittest.main()

