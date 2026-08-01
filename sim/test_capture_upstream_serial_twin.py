"""Regression tests for the untouched-upstream serial transport capture."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import capture_upstream_serial_twin as capture  # noqa: E402
from continuous_conductor_route import _raw_shaft_wraps  # noqa: E402
from traj import Timeline, load_events, winding_windows  # noqa: E402


CAPTURE = ROOT / "out" / "capture" / "upstream_serial_twin_raw.jsonl"


class SerialTwinProtocolTests(unittest.TestCase):
    def test_retarget_starts_at_instantaneous_position(self) -> None:
        clock = capture.VirtualClock()
        twin = capture.SerialDigitalTwin([2.0, 2.0, 2.0, 2.0], clock)
        twin.write(b"M1A10\n")
        clock.sleep(1.5)
        twin.write(b"M1A-2\n")
        self.assertAlmostEqual(twin.axes[1].start_position, 3.0, places=12)
        clock.sleep(1.0)
        twin.write(b"M1P\n")
        self.assertEqual(twin.readline(), b"M1P1.000000000000\n")

    def test_estop_freezes_all_axes(self) -> None:
        clock = capture.VirtualClock()
        twin = capture.SerialDigitalTwin([4.0] * 4, clock)
        twin.write(b"M0A8\nM2A-8\n")
        clock.sleep(0.5)
        twin.write(b"ESTOP\n")
        before = [axis.position_at() for axis in twin.axes]
        clock.sleep(3.0)
        self.assertEqual(before, [axis.position_at() for axis in twin.axes])


class GeneratedSerialCaptureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.events = load_events(CAPTURE)
        cls.timeline = Timeline(cls.events)
        cls.meta = next(row for row in cls.events if row["e"] == "meta")

    def test_capture_binds_clean_unmodified_upstream_and_harness(self) -> None:
        self.assertEqual(
            self.meta["winder_commit"],
            "6039b33c8f15a20086c2195c3f2d02b3a833e8ca",
        )
        self.assertFalse(self.meta["winder_dirty"])
        self.assertEqual(self.meta["controller_mode"], "upstream")
        self.assertEqual(
            self.meta["upstream_transport"],
            "serial_position_digital_twin",
        )
        self.assertFalse(self.meta["upstream_source_subclassed"])
        self.assertFalse(self.meta["upstream_source_modified_by_harness"])
        self.assertEqual(
            self.meta["serial_twin_source_sha256"],
            hashlib.sha256(Path(capture.__file__).read_bytes()).hexdigest(),
        )

    def test_complete_cycle_and_all_passes_are_present(self) -> None:
        self.assertEqual(
            sum(row["e"] == "cycle_complete" for row in self.events), 1,
        )
        self.assertEqual(len(winding_windows(self.events)), 24)

    def test_serial_feedback_does_not_fix_pinned_source_wrap_targets(self) \
            -> None:
        wraps = _raw_shaft_wraps(self.events, self.timeline)
        self.assertEqual(len(wraps), 2)
        observed = [row["turns"] for row in wraps]
        self.assertAlmostEqual(observed[0], 1.3749395533708841, places=12)
        self.assertAlmostEqual(observed[1], 2.791736856774936, places=12)
        self.assertTrue(any(
            not math.isclose(value, 2.0, abs_tol=0.0001)
            for value in observed
        ))


if __name__ == "__main__":
    unittest.main()
