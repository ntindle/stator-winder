"""Regression tests for finite-coil M0 winding travel generation."""

import unittest

import coil_growth
import settings_gen
import wire_geometry
from params import DEFAULT_STATOR, PARAMS, StatorSpec


class SettingsTravelTests(unittest.TestCase):
    def test_default_job_is_the_reachable_fifty_turn_baseline(self):
        self.assertEqual(DEFAULT_STATOR.turns, 50)
        self.assertEqual(DEFAULT_STATOR.wire_d, 0.22352)
        self.assertEqual(
            coil_growth.DEFAULT_POLICY.opening_edge_clearance_mm, 0.127)
        cfg = settings_gen.derive(DEFAULT_STATOR)
        self.assertEqual(cfg["winding"]["turns"], 50)
        self.assertEqual(cfg["job"]["slots"], 24)
        self.assertEqual(cfg["job"]["liner_max_thickness_mm"], 0.127)
        self.assertEqual(cfg["job"]["liner_measured_thickness_mm"], 0.127)
        self.assertFalse(cfg["job"]["hardware_motion_authorized"])
        self.assertEqual(
            cfg["job"]["winding_plan"],
            "reports/slot_winding_plan.json")
        self.assertIn(
            'winding_plan: "reports/slot_winding_plan.json"',
            settings_gen.to_yaml(cfg))
        self.assertIn(
            "hardware_motion_authorized: false",
            settings_gen.to_yaml(cfg))

    def test_custom_job_never_reuses_default_constructive_plan(self):
        custom = StatorSpec(wire_d=0.20, turns=1)
        cfg = settings_gen.derive(custom)
        self.assertIsNone(cfg["job"]["winding_plan"])
        self.assertNotIn("winding_plan:", settings_gen.to_yaml(cfg))

    def test_measured_wire_and_liner_are_bound_to_regenerated_plan(self):
        measured = StatorSpec(wire_d=0.231)
        proof = "ab" * 32
        cfg = settings_gen.derive(
            measured,
            liner_t_mm=0.134,
            winding_plan_path="reports/slot_winding_plan.json",
            winding_plan_proof_sha256=proof,
        )
        self.assertEqual(cfg["job"]["wire_finished_d_mm"], 0.231)
        self.assertEqual(cfg["job"]["liner_measured_thickness_mm"], 0.134)
        self.assertEqual(cfg["job"]["winding_plan_proof_sha256"], proof)
        rendered = settings_gen.to_yaml(cfg)
        self.assertIn("liner_measured_thickness_mm: 0.134", rendered)
        self.assertIn(proof, rendered)

    def test_liner_outside_receiving_contract_fails_closed(self):
        for value in (0.119, 0.141):
            with self.subTest(value=value):
                with self.assertRaisesRegex(SystemExit, "outside receiving"):
                    settings_gen.derive(DEFAULT_STATOR, liner_t_mm=value)

    def test_default_range_matches_finite_coil_prism(self):
        cfg = settings_gen.derive(DEFAULT_STATOR)
        self.assertEqual(cfg["job"]["spindle_id"], "er11")
        coil = coil_growth.require_feasible(DEFAULT_STATOR)
        contact = wire_geometry.tooth_contact_spec(DEFAULT_STATOR, coil)
        shallow, deep = contact["insertion_depth_range_mm"]
        m0 = cfg["motor"]["M0"]
        self.assertAlmostEqual(
            PARAMS.stator_axis_z(m0["wind_range_start"]),
            DEFAULT_STATOR.od / 2.0 - deep,
            delta=0.001,
        )
        self.assertAlmostEqual(
            PARAMS.stator_axis_z(m0["wind_range_end"]),
            DEFAULT_STATOR.od / 2.0 - shallow,
            delta=0.001,
        )
        self.assertAlmostEqual(
            abs(m0["wind_range_end"] - m0["wind_range_start"])
            * PARAMS.mm_per_rad,
            contact["radial_winding_span_mm"][1]
            - contact["radial_winding_span_mm"][0],
            delta=0.002,
        )
        self.assertLessEqual(deep, PARAMS.max_insertion(DEFAULT_STATOR))

    def test_default_values_reproduce_reviewed_geometry(self):
        cfg = settings_gen.derive(DEFAULT_STATOR)
        m0 = cfg["motor"]["M0"]
        self.assertAlmostEqual(m0["wind_range_start"], -61.918, places=3)
        self.assertAlmostEqual(m0["wind_range_end"], -56.800, places=3)

    def test_launch_od_endpoints_reach_full_finite_span(self):
        # One turn keeps capacity from masking the mechanical reach check.
        for od in (28.0, 36.0, 46.0, 65.0):
            spec = StatorSpec(
                od=od, stack=max(5.0, min(20.0, od * 0.3)),
                wire_d=0.20, turns=1,
            )
            cfg = settings_gen.derive(spec, spindle="er11")
            contact = wire_geometry.tooth_contact_spec(
                spec, coil_growth.require_feasible(spec),
            )
            self.assertLessEqual(
                contact["insertion_depth_range_mm"][1],
                PARAMS.max_insertion(spec, spindle="er11"),
                msg=f"OD{od:g}",
            )
            self.assertLess(
                cfg["motor"]["M0"]["wind_range_start"],
                cfg["motor"]["M0"]["wind_range_end"],
            )

    def test_od28_and_od36_use_actual_finite_span_with_er11(self):
        # These use each geometry's wire-accessible capacity, not the former
        # inaccessible slot-root area. Neither needs an OD-threshold spindle
        # swap; the selected turns simply have to fit the actual slot.
        jobs = (
            StatorSpec(od=28.0, stack=8.4, wire_d=0.20, turns=3),
            StatorSpec(od=36.0, stack=10.8, wire_d=0.20, turns=45),
        )
        for spec in jobs:
            cfg = settings_gen.derive(spec, spindle="er11")
            contact = wire_geometry.tooth_contact_spec(
                spec, coil_growth.require_feasible(spec))
            self.assertLessEqual(
                contact["insertion_depth_range_mm"][1],
                PARAMS.max_insertion(spec, spindle="er11"),
                msg=f"OD{spec.od:g}",
            )
            self.assertEqual(cfg["job"]["spindle_id"], "er11")

    def test_workholding_options_fail_closed_by_shaft_diameter(self):
        shaft8 = StatorSpec(
            od=28.0, stack=8.4, shaft_d=8.0, wire_d=0.20, turns=1)
        cfg = settings_gen.derive(shaft8, spindle="shaft8")
        self.assertEqual(cfg["job"]["spindle_id"], "shaft8")
        with self.assertRaisesRegex(SystemExit, "supports shaft diameter"):
            settings_gen.derive(shaft8, spindle="er11")

        shaft7 = StatorSpec(shaft_d=7.0)
        settings_gen.derive(shaft7, spindle="er11")
        with self.assertRaisesRegex(SystemExit, "supports shaft diameter"):
            settings_gen.derive(shaft7, spindle="shaft8")

    def test_flyer_elbow_clears_longest_launch_stack_shaft_tip(self):
        for stack in (PARAMS.stack_min, PARAMS.stack_max):
            with self.subTest(stack=stack):
                spec = StatorSpec(stack=stack)
                cfg = settings_gen.derive(spec, spindle="er11")
                m0 = cfg["motor"]["M0"]
                deepest_axis = min(
                    PARAMS.stator_axis_z(m0[key])
                    for key in ("wind_range_start", "wind_range_end")
                )
                # Add 0.5 mm to the modeled shaft extent as the collision
                # mesh/tessellation allowance used by the exported envelope.
                shaft_tip_z = (
                    deepest_axis - stack / 2.0 - spec.shaft_below - 0.5
                )
                elbow_front_z = (
                    wire_geometry.TIP_GUIDE_CENTER_Z
                    + wire_geometry.FLYER_ELBOW_BODY_RADIUS
                )
                self.assertGreaterEqual(
                    shaft_tip_z - elbow_front_z,
                    PARAMS.dyn_clearance,
                )


if __name__ == "__main__":
    unittest.main(verbosity=2)
