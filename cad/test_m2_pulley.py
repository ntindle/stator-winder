"""Fit and load contracts for the selected M2 motor pulley."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parent))
import assembly  # noqa: E402
import cots  # noqa: E402
from params import PARAMS  # noqa: E402


class M2PulleyTests(unittest.TestCase):
    def test_selected_pulley_envelope_and_belt_lane(self):
        box = cots.gt2_pulley_40t_b5().bounding_box()
        self.assertAlmostEqual(box.size.Z, 10.3, places=3)
        self.assertGreaterEqual(box.size.X, 30.0)
        self.assertAlmostEqual(PARAMS.m2_motor_pulley_channel, 7.0)
        self.assertAlmostEqual(
            (PARAMS.m2_motor_pulley_channel - 6.0) / 2.0, 0.5
        )

    def test_belt_has_no_motor_body_interpenetration(self):
        parts = {part.label: part for part in assembly.static_link()}
        motor_solids = list(parts["m2_motor"].solids())
        belt_solids = list(parts["gt2_belt"].solids())
        common = sum(
            0.0 if (a & b) is None else (a & b).volume
            for a in motor_solids for b in belt_solids
        )
        self.assertAlmostEqual(common, 0.0, places=6)

    def test_pulley_capacity_is_two_x_simulated_demand(self):
        # Current loads report derives 0.292 Nm including acceleration and
        # doubled capstan energy. Keep the exact comparison here so changing
        # either the pulley rating or duty forces deliberate revalidation.
        self.assertGreaterEqual(PARAMS.m2_motor_pulley_capacity_nm / 0.292, 2.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
