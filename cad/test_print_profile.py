"""Fail-closed tests for the project-owned A1/PETG slicer profile lock."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "print"))
import prepare_profile  # noqa: E402
import prepare_coupon  # noqa: E402
import slice_coupon  # noqa: E402


class PrintProfileTests(unittest.TestCase):
    def test_frozen_profile_matches_release_decisions(self):
        lock = json.loads(prepare_profile.LOCK_SOURCE.read_text(encoding="utf-8"))
        self.assertEqual(lock["machine"]["model"], "A1")
        self.assertEqual(lock["machine"]["nozzle_mm"], 0.4)
        self.assertEqual(lock["machine"]["build_plate"], "Textured PEI Plate")
        self.assertEqual(lock["process"]["layer_height_mm"], 0.2)
        self.assertEqual(lock["process"]["walls"], 6)
        self.assertEqual(lock["process"]["top_layers"], 5)
        self.assertEqual(lock["process"]["bottom_layers"], 3)
        self.assertEqual(lock["process"]["infill_percent"], 25)
        self.assertEqual(lock["process"]["infill_pattern"], "cubic")
        self.assertEqual(
            lock["process"]["successor_flyer_arm_infill_percent"], 100
        )
        self.assertEqual(
            lock["process"]["balance_retainer_infill_percent"], 100
        )
        self.assertFalse(lock["process"]["coupon_supports"])
        self.assertEqual(lock["filament"]["nozzle_temp_c"], 260)
        self.assertEqual(lock["filament"]["textured_plate_temp_c"], 70)

    def test_orca_transport_profiles_match_lock(self):
        lock = json.loads(prepare_profile.LOCK_SOURCE.read_text(encoding="utf-8"))
        process = json.loads(prepare_profile.PROCESS_SOURCE.read_text(encoding="utf-8"))
        filament = json.loads(prepare_profile.FILAMENT_SOURCE.read_text(encoding="utf-8"))

        self.assertEqual(process["curr_bed_type"], "Textured PEI Plate")
        self.assertEqual(float(process["layer_height"]),
                         lock["process"]["layer_height_mm"])
        self.assertEqual(int(process["wall_loops"]), lock["process"]["walls"])
        self.assertEqual(int(process["top_shell_layers"]),
                         lock["process"]["top_layers"])
        self.assertEqual(int(process["bottom_shell_layers"]),
                         lock["process"]["bottom_layers"])
        self.assertEqual(process["sparse_infill_density"], "25%")
        self.assertEqual(process["sparse_infill_pattern"], "cubic")
        self.assertEqual(float(filament["filament_flow_ratio"][0]),
                         lock["filament"]["flow_ratio"])
        self.assertEqual(float(filament["filament_max_volumetric_speed"][0]),
                         lock["filament"]["max_volumetric_speed_mm3_s"])
        self.assertEqual(int(filament["nozzle_temperature"][0]), 260)
        self.assertEqual(int(filament["textured_plate_temp"][0]), 70)
        self.assertEqual(int(filament["fan_min_speed"][0]), 10)
        self.assertEqual(int(filament["fan_max_speed"][0]), 40)

    def test_wrapper_materialization_uses_absolute_native_paths(self):
        with tempfile.TemporaryDirectory() as temporary:
            home = Path(temporary) / "orca"
            machine = (home / "resources" / "profiles" / "BBL"
                       / "machine" / "Bambu Lab A1 0.4 nozzle.json")
            machine.parent.mkdir(parents=True)
            machine.write_text(json.dumps({
                "name": "Bambu Lab A1 0.4 nozzle",
                "nozzle_diameter": ["0.4"],
                "printable_height": "256",
            }), encoding="utf-8")
            (home / "orca-slicer.exe").write_bytes(b"fixture")
            out = Path(temporary) / "out"
            result = prepare_profile.prepare(out, home)
            wrapper = json.loads(Path(result["wrapper"]).read_text(
                encoding="utf-8"))

        self.assertEqual(wrapper["backend"], "orcaslicer")
        self.assertTrue(Path(wrapper["native_config"]).is_absolute())
        self.assertEqual(wrapper["machine"]["motion_bounds_mm"]["x"],
                         [-48.2, 267.0])
        self.assertEqual(wrapper["filament"]["type"], "PETG")

    def test_missing_native_slicer_fails_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(RuntimeError, "required slicer"):
                prepare_profile.prepare(Path(temporary) / "out",
                                        Path(temporary) / "missing")

    def test_coupon_gcode_metadata_verification_fails_closed(self):
        lock = json.loads(prepare_profile.LOCK_SOURCE.read_text(encoding="utf-8"))
        rows = {
            "curr_bed_type": lock["machine"]["build_plate"],
            "layer_height": lock["process"]["layer_height_mm"],
            "wall_loops": lock["process"]["walls"],
            "top_shell_layers": lock["process"]["top_layers"],
            "bottom_shell_layers": lock["process"]["bottom_layers"],
            "sparse_infill_density": f'{lock["process"]["infill_percent"]}%',
            "sparse_infill_pattern": lock["process"]["infill_pattern"],
            "enable_support": 0,
            "filament_density": lock["filament"]["density_g_cm3"],
            "filament_flow_ratio": lock["filament"]["flow_ratio"],
            "filament_max_volumetric_speed": lock["filament"]["max_volumetric_speed_mm3_s"],
            "nozzle_temperature": lock["filament"]["nozzle_temp_c"],
            "textured_plate_temp": lock["filament"]["textured_plate_temp_c"],
            "fan_min_speed": lock["filament"]["fan_min_percent"],
            "fan_max_speed": lock["filament"]["fan_max_percent"],
        }
        with tempfile.TemporaryDirectory() as temporary:
            fixture = Path(temporary) / "fixture.gcode"
            fixture.write_text("".join(
                f"; {key} = {value}\n" for key, value in rows.items()
            ), encoding="utf-8")
            self.assertTrue(
                prepare_coupon.verify_gcode_profile(fixture)[
                    "profile_metadata_verified"
                ]
            )
            fixture.write_text(
                fixture.read_text(encoding="utf-8").replace(
                    "; wall_loops = 6", "; wall_loops = 2"
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(RuntimeError, "metadata drift"):
                prepare_coupon.verify_gcode_profile(fixture)

    def test_project_slicer_command_has_no_printer_transport(self):
        command = slice_coupon.build_command(
            Path("orca-slicer.exe"),
            {
                "native_settings": ["machine.json", "process.json"],
                "native_filaments": ["filament.json"],
            },
            Path("local-output"),
            Path("coupon.stl"),
        )
        self.assertEqual(command[1:3], ["--load-settings",
                                        "machine.json;process.json"])
        self.assertIn("--slice", command)
        self.assertNotIn("--send-to-printer", command)
        self.assertNotIn("--upload", command)


if __name__ == "__main__":
    unittest.main(verbosity=2)
