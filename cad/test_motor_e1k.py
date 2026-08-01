"""Regression checks for the live M0/M1 closed-loop motor selection."""

from __future__ import annotations

import hashlib
from pathlib import Path
import sys
import unittest

from build123d import import_step

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cots
from loads import MOTORS, torque_at_rpm


UPGRADES = Path(__file__).resolve().parent / "models" / "upgrades"
STEP = UPGRADES / "17HS19-2004D-E1K.step"
CURVE = UPGRADES / "17HS19-2004D-E1K_Torque_Curve.pdf"
MOTOR = "17HS19-2004D-E1K + CL42T-V41 @24V (M0/M1)"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class E1KMotorTests(unittest.TestCase):
    def test_vendor_files_are_the_verified_revisions(self):
        self.assertEqual(
            _sha256(STEP),
            "2cf71823dc9b09f255397e6b8a3e771d043945e8a131cce5f62a2bf06f788bad",
        )
        self.assertEqual(
            _sha256(CURVE),
            "b5c736bb8116f51052f16952e55bdac9695977b6c1df2e329947d4f599462cbb",
        )

    def test_vendor_step_separates_rigid_motor_from_loose_cable(self):
        solids = import_step(str(STEP)).solids()
        self.assertEqual(len(solids), 24)

        # Four body/end-cap solids exceed 10 cm^3.  The only additional
        # rigid installed solid is the central Ø5 shaft.  Everything else
        # is the flexible lead, connector, or connector-pin detail.
        rigid = []
        for solid in solids:
            box = solid.bounding_box()
            is_shaft = (
                box.size.X <= 5.01
                and box.size.Z <= 5.01
                and box.max.Y >= 23.9
            )
            if solid.volume > 10_000 or is_shaft:
                rigid.append(solid)
        self.assertEqual(len(rigid), 5)

        self.assertAlmostEqual(min(s.bounding_box().min.X for s in rigid), -21.15, places=2)
        self.assertAlmostEqual(max(s.bounding_box().max.X for s in rigid), 21.15, places=2)
        self.assertAlmostEqual(min(s.bounding_box().min.Y for s in rigid), -68.0, places=2)
        self.assertAlmostEqual(max(s.bounding_box().max.Y for s in rigid), 24.0, places=2)
        self.assertAlmostEqual(min(s.bounding_box().min.Z for s in rigid), -21.15, places=2)
        self.assertAlmostEqual(max(s.bounding_box().max.Z for s in rigid), 21.15, places=2)

    def test_collision_envelope_matches_verified_interface(self):
        box = cots.nema17().bounding_box()
        self.assertAlmostEqual(box.size.X, 42.3, places=2)
        self.assertAlmostEqual(box.size.Y, 42.3, places=2)
        self.assertAlmostEqual(box.min.Z, -68.0, places=2)
        self.assertAlmostEqual(box.max.Z, 24.0, places=2)

    def test_load_model_uses_dynamic_curve_not_holding_torque(self):
        self.assertAlmostEqual(MOTORS[MOTOR]["holding_nm"], 0.52, places=3)
        self.assertAlmostEqual(torque_at_rpm(MOTOR, 50), 0.350, places=3)
        self.assertLess(torque_at_rpm(MOTOR, 100), 0.52)
        self.assertGreater(torque_at_rpm(MOTOR, 100), 0.34)


if __name__ == "__main__":
    unittest.main(verbosity=2)
