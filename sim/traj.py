"""Reconstruct the machine pose timeline from the captured command stream.

Motion model = the software's own simulation semantics (constant-velocity
slew toward the last commanded absolute target, per-axis velocity caps from
settings) — identical math to src/winding.calculate_motor_position_in_
simulation and scripts/ws.py.

Produces per-axis piecewise-linear knot lists and a pose sampler.
"""

import json
import math
from pathlib import Path


def load_events(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines()]


def winding_windows(events):
    """Extract each tooth pass and its true captured M2 winding start.

    ``wind_wire`` includes stator indexing, tensioning, M0 positioning, and an
    optional M2 side-change.  Deposited turns begin only at the first M2
    command *after* ``set_motor2_wire_position_done``; counting from the
    high-level call marker incorrectly treats setup motion as coil winding.
    """
    windows = []
    active = None
    phase = -1
    for event in events:
        name = event["e"]
        if name == "wind":
            args = event.get("args", [])
            phase = int(args[0]) if args else phase + 1
        elif name == "wind_wire":
            if active is not None:
                raise ValueError("nested wind_wire markers")
            args = event.get("args", [])
            active = {
                "start": float(event["t"]),
                "tooth": int(args[0]) if args else -1,
                "clockwise": bool(args[1]) if len(args) > 1 else False,
                "passIndex": int(args[2]) if len(args) > 2 else len(windows),
                "phase": phase,
            }
        elif name == "set_motor2_wire_position_done" and active is not None:
            active["positionedAt"] = float(event["t"])
        elif (name == "cmd" and active is not None
              and event.get("m") == 2
              and "positionedAt" in active
              and "motionStart" not in active):
            active["motionStart"] = float(event["t"])
        elif name == "wind_wire_done":
            if active is None:
                raise ValueError("unpaired wind_wire_done marker")
            if "positionedAt" not in active:
                raise ValueError("wind pass has no flyer-positioned marker")
            if "motionStart" not in active:
                raise ValueError("wind pass has no post-positioning M2 command")
            active["end"] = float(event["t"])
            windows.append(active)
            active = None
    if active is not None:
        raise ValueError("unpaired wind_wire marker")
    return windows


class AxisTrack:
    """Piecewise-linear position track built from timed absolute targets."""

    def __init__(self, velocity):
        self.v = velocity
        self.knots = [(0.0, 0.0)]        # (t, pos)
        self._target = 0.0

    def command(self, t, target):
        pos = self.pos_at(t)
        # A prior command may have scheduled an arrival after this new
        # command time.  Retargeting cancels that obsolete future motion.
        # Keeping it made both the GLB and collision sweep overshoot the real
        # upstream constant-velocity simulation by as much as 13.39 mm.
        self.knots = [knot for knot in self.knots if knot[0] <= t]
        if self.knots[-1][0] < t:
            self.knots.append((t, pos))
        else:
            self.knots[-1] = (t, pos)
        self._target = target
        dist = abs(target - pos)
        if dist > 1e-9:
            self.knots.append((t + dist / self.v, target))

    def pos_at(self, t):
        ks = self.knots
        if t >= ks[-1][0]:
            return ks[-1][1]
        # binary search
        lo, hi = 0, len(ks) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if ks[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, p0 = ks[lo]
        t1, p1 = ks[hi]
        if t1 <= t0:
            return p1
        return p0 + (p1 - p0) * (t - t0) / (t1 - t0)


class Timeline:
    def __init__(self, events):
        meta = next(e for e in events if e["e"] == "meta")
        self.meta = meta
        v = meta["velocities"]
        self.axes = {i: AxisTrack(v[i]) for i in range(4)}
        for e in events:
            if e["e"] == "cmd":
                target = e.get("model_target", e["a"])
                self.axes[e["m"]].command(e["t"], target)
        self.t_end = max(a.knots[-1][0] for a in self.axes.values())
        self.events = events

    def pose_at(self, t):
        return (self.axes[0].pos_at(t), self.axes[1].pos_at(t),
                self.axes[2].pos_at(t))

    def knot_times(self):
        ts = set()
        for i in (0, 1, 2):
            ts.update(t for t, _ in self.axes[i].knots)
        return sorted(ts)

    def phase_at(self, t):
        """Most recent high-level event name at time t (for reporting)."""
        name = "init"
        for e in self.events:
            if e["t"] > t:
                break
            if e["e"] not in ("cmd", "meta"):
                name = e["e"]
        return name

    def samples(self, max_dm2=math.radians(2.0), max_dm0=0.25,
                max_dm1=math.radians(2.0)):
        """Yield (t, m0, m1, m2) finely enough that no axis moves more than
        the given deltas between consecutive samples."""
        ts = self.knot_times()
        prev = None
        for i in range(len(ts) - 1):
            t0, t1 = ts[i], ts[i + 1]
            p0 = self.pose_at(t0)
            p1 = self.pose_at(t1)
            n = max(1, int(math.ceil(max(
                abs(p1[0] - p0[0]) / max_dm0,
                abs(p1[1] - p0[1]) / max_dm1,
                abs(p1[2] - p0[2]) / max_dm2,
            ))))
            for k in range(n):
                t = t0 + (t1 - t0) * k / n
                pose = (t,
                        p0[0] + (p1[0] - p0[0]) * k / n,
                        p0[1] + (p1[1] - p0[1]) * k / n,
                        p0[2] + (p1[2] - p0[2]) * k / n)
                if prev != pose[1:]:
                    yield pose
                    prev = pose[1:]
        t = ts[-1]
        yield (t, *self.pose_at(t))
