"""Regression and parity tests for the captured-command trajectory."""

import math
import unittest
from pathlib import Path

from traj import AxisTrack, Timeline, load_events, winding_windows


class ReferenceAxis:
    """Direct replay of upstream calculate_motor_position_in_simulation."""

    def __init__(self, velocity):
        self.velocity = velocity
        self.commands = []

    def command(self, time, target):
        self.commands.append((time, target))

    def pos_at(self, query):
        position = 0.0
        target = 0.0
        last = 0.0
        for time, new_target in self.commands:
            if time > query:
                break
            dt = time - last
            movement = self.velocity * dt
            delta = target - position
            position += math.copysign(min(abs(delta), movement), delta) \
                if abs(delta) > 1e-12 else 0.0
            target = new_target
            last = time
        dt = query - last
        movement = self.velocity * dt
        delta = target - position
        return position + (math.copysign(min(abs(delta), movement), delta)
                           if abs(delta) > 1e-12 else 0.0)


class AxisTrackTests(unittest.TestCase):
    def test_winding_window_ignores_pre_positioning_flyer_move(self):
        events = [
            {"t": 0.0, "e": "wind", "args": [0]},
            {"t": 1.0, "e": "wind_wire", "args": [3, True, 0]},
            {"t": 1.2, "e": "cmd", "m": 2, "model_target": 1.0},
            {"t": 1.2, "e": "set_motor2_wire_position_done"},
            {"t": 2.0, "e": "cmd", "m": 0, "model_target": -10.0},
            {"t": 2.5, "e": "cmd", "m": 2, "model_target": 7.0},
            {"t": 4.0, "e": "wind_wire_done"},
        ]
        windows = winding_windows(events)
        self.assertEqual(len(windows), 1)
        self.assertEqual(windows[0]["motionStart"], 2.5)
        self.assertEqual(windows[0]["positionedAt"], 1.2)
        self.assertEqual(windows[0]["tooth"], 3)

    def test_retarget_discards_obsolete_arrival(self):
        axis = AxisTrack(10.0)
        axis.command(0.0, 100.0)   # old arrival t=10
        axis.command(2.0, 0.0)     # retarget at position 20; new arrival t=4
        self.assertEqual(axis.knots, [(0.0, 0.0), (2.0, 20.0), (4.0, 0.0)])
        self.assertAlmostEqual(axis.pos_at(3.0), 10.0)
        self.assertAlmostEqual(axis.pos_at(6.0), 0.0)

    def test_same_timestamp_last_target_wins(self):
        axis = AxisTrack(5.0)
        axis.command(1.0, 10.0)
        axis.command(1.0, -5.0)
        self.assertAlmostEqual(axis.pos_at(1.5), -2.5)
        self.assertFalse(any(t > 1.0 and p == 10.0 for t, p in axis.knots))

    def test_full_capture_matches_independent_replay(self):
        path = Path(__file__).resolve().parent.parent / "out" / "capture" / \
            "commands.jsonl"
        events = load_events(path)
        timeline = Timeline(events)
        velocities = timeline.meta["velocities"]
        refs = {axis: ReferenceAxis(velocities[axis]) for axis in range(4)}
        command_times = {0.0}
        for event in events:
            if event["e"] == "cmd":
                refs[event["m"]].command(
                    event["t"], event.get("model_target", event["a"]))
                command_times.add(event["t"])
        times = sorted(command_times)
        dense = set(times)
        dense.update((a + b) / 2.0 for a, b in zip(times, times[1:]))
        for time in sorted(dense):
            for axis in range(4):
                self.assertAlmostEqual(
                    timeline.axes[axis].pos_at(time), refs[axis].pos_at(time),
                    places=8, msg=f"axis {axis} at t={time}")

    def test_full_capture_starts_every_winding_inside_m0_span(self):
        path = Path(__file__).resolve().parent.parent / "out" / "capture" / \
            "commands.jsonl"
        events = load_events(path)
        timeline = Timeline(events)
        lo, hi = sorted(timeline.meta["m0_wind_range"])
        windows = winding_windows(events)
        self.assertEqual(len(windows), 24)
        for window in windows:
            m0 = timeline.axes[0].pos_at(window["motionStart"])
            self.assertGreaterEqual(m0, lo - 1e-6, window)
            self.assertLessEqual(m0, hi + 1e-6, window)


if __name__ == "__main__":
    unittest.main()
