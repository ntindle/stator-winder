"""Capture untouched upstream winder code against a serial position twin.

``capture.py --controller upstream`` intentionally exercises upstream's own
WebSocket/SQLite simulation path.  This independent transport check answers
position queries as a real serial controller would.  At the pinned 6039b33
checkout it confirms that the shaft-wrap regression is source-level: the
current ``wind_wire_around_shaft`` does not query M1 and instead commands an
absolute target from its bookkeeping zero.  A real ``M1P`` response therefore
cannot repair the two non-two-turn physical moves.

This harness runs the exact same imported ``Wind`` class with
``simulation=False``.  It replaces only ``serial.Serial`` and wall-clock sleep
at runtime, just as a hardware-in-the-loop test replaces a COM port.  The fake
serial device implements the public protocol from the project contract:

* ``M{id}A{absolute}\n`` retargets a constant-velocity axis;
* ``M{id}P\n`` returns its current controller-space position; and
* ``ESTOP\n`` freezes all axes at their instantaneous positions.

No upstream source file is edited or subclassed.  Both the source-requested
model target and the target actually representable after the upstream
three-decimal serial quantization are recorded.  ``model_target`` is the
effective physical target so ``traj.Timeline`` reconstructs the same motion as
this serial twin; ``requested_model_target`` preserves the untouched source
request for the regression audit.
"""

from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timedelta
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Iterable


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DEFAULT_SETTINGS = ROOT / "out" / "settings.yml"
DEFAULT_WINDER = ROOT.parent / "winder"
DEFAULT_OUTPUT = ROOT / "out" / "capture" / "upstream_serial_twin_raw.jsonl"
CAPTURE_SCHEMA = 5


class VirtualClock:
    """Monotone virtual clock used by both upstream and the serial twin."""

    def __init__(self) -> None:
        self.t = 0.0
        self.base = datetime(2026, 1, 1)

    def sleep(self, seconds: float) -> None:
        value = float(seconds)
        if value < 0.0 or not math.isfinite(value):
            raise ValueError("virtual sleep must be finite and nonnegative")
        self.t += value

    def now(self) -> datetime:
        return self.base + timedelta(seconds=self.t)


class SerialAxis:
    """One controller-space, constant-velocity absolute-position axis."""

    def __init__(self, velocity: float, clock: VirtualClock) -> None:
        self.velocity = float(velocity)
        if self.velocity <= 0.0 or not math.isfinite(self.velocity):
            raise ValueError("serial-twin velocity must be finite and positive")
        self.clock = clock
        self.start_t = float(clock.t)
        self.start_position = 0.0
        self.target = 0.0

    def position_at(self, time_s: float | None = None) -> float:
        time_value = self.clock.t if time_s is None else float(time_s)
        elapsed = max(0.0, time_value - self.start_t)
        delta = self.target - self.start_position
        travel = self.velocity * elapsed
        if abs(delta) <= travel:
            return self.target
        return self.start_position + math.copysign(travel, delta)

    def command(self, target: float) -> None:
        position = self.position_at()
        self.start_t = float(self.clock.t)
        self.start_position = position
        self.target = float(target)

    def freeze(self) -> None:
        self.command(self.position_at())


class SerialDigitalTwin:
    """Minimal deterministic implementation of the winder serial protocol."""

    _ABSOLUTE = re.compile(
        r"^M(?P<axis>[0-3])A(?P<value>[+-]?(?:\d+(?:\.\d*)?|\.\d+))$"
    )
    _POSITION = re.compile(r"^M(?P<axis>[0-3])P$")

    def __init__(self, velocities: Iterable[float], clock: VirtualClock) -> None:
        values = [float(value) for value in velocities]
        if len(values) != 4:
            raise ValueError("serial twin requires four axis velocities")
        self.clock = clock
        self.axes = [SerialAxis(value, clock) for value in values]
        self._responses: deque[bytes] = deque()
        self.closed = False
        self.estopped = False
        self.protocol_log: list[dict[str, Any]] = []

    @property
    def in_waiting(self) -> int:
        return sum(len(row) for row in self._responses)

    def write(self, raw: bytes) -> int:
        if self.closed:
            raise OSError("serial twin is closed")
        text = bytes(raw).decode("utf-8")
        commands = [row.strip() for row in text.splitlines() if row.strip()]
        for command in commands:
            absolute = self._ABSOLUTE.fullmatch(command)
            position = self._POSITION.fullmatch(command)
            record: dict[str, Any] = {
                "t": round(float(self.clock.t), 9),
                "command": command,
            }
            if absolute is not None:
                if self.estopped:
                    raise RuntimeError("absolute command received after ESTOP")
                axis = int(absolute.group("axis"))
                target = float(absolute.group("value"))
                start = self.axes[axis].position_at()
                self.axes[axis].command(target)
                record.update({
                    "kind": "absolute",
                    "axis": axis,
                    "start_controller_position": start,
                    "target_controller_position": target,
                })
            elif position is not None:
                axis = int(position.group("axis"))
                value = self.axes[axis].position_at()
                # More precision than the three-decimal command avoids adding
                # an artificial feedback quantizer which upstream does not
                # specify.  The command target itself remains exactly rounded.
                self._responses.append(
                    f"M{axis}P{value:.12f}\n".encode("utf-8")
                )
                record.update({
                    "kind": "position",
                    "axis": axis,
                    "controller_position": value,
                })
            elif command == "ESTOP":
                for axis in self.axes:
                    axis.freeze()
                self.estopped = True
                record["kind"] = "estop"
            else:
                raise ValueError(f"unsupported serial command: {command!r}")
            self.protocol_log.append(record)
        return len(raw)

    def readline(self) -> bytes:
        if not self._responses:
            return b""
        return self._responses.popleft()

    def close(self) -> None:
        self.closed = True


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _git_text(repo: Path, *args: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(repo), *args], text=True,
    ).strip()


def _effective_model_target(
    wind: Any, winding_module: Any, motor_id: int, requested: float,
    round_to: int,
) -> tuple[float, float]:
    controller = wind.check_motor_direction(motor_id, requested)
    if motor_id == 2:
        controller = wind.adjust_motor_position_from_gear_ratio(
            controller, winding_module.m2_gear_ratio,
        )
    controller = round(controller, round_to)
    effective = controller
    if motor_id == 2:
        effective = wind.adjust_motor_position_from_gear_ratio(
            effective, winding_module.m2_gear_ratio, True,
        )
    effective = wind.check_motor_direction(motor_id, effective)
    return float(controller), float(effective)


def capture(
    *, settings_path: Path = DEFAULT_SETTINGS,
    winder_path: Path = DEFAULT_WINDER,
    output_path: Path = DEFAULT_OUTPUT,
    turns: int | None = None,
) -> dict[str, Any]:
    """Run one full untouched upstream cycle and write its JSONL capture."""

    settings_path = Path(settings_path).resolve()
    winder_path = Path(winder_path).resolve()
    output_path = Path(output_path).resolve()
    if not settings_path.is_file():
        raise FileNotFoundError(settings_path)
    if not (winder_path / "src" / "winding.py").is_file():
        raise FileNotFoundError(winder_path / "src" / "winding.py")

    sys.path.insert(0, str(winder_path))
    winding_module = importlib.import_module("src.winding")
    Wind = winding_module.Wind
    config = winding_module.load_config(str(settings_path))
    model_velocities = [
        float(config["motor"][f"M{axis}"]["velocity"])
        for axis in range(4)
    ]
    controller_velocities = list(model_velocities)
    controller_velocities[2] *= abs(float(winding_module.m2_gear_ratio))

    clock = VirtualClock()
    serial_twin = SerialDigitalTwin(controller_velocities, clock)
    winding_module.serial.Serial = lambda *_args, **_kwargs: serial_twin
    winding_module.sleep = clock.sleep

    class FakeDateTime:
        @staticmethod
        def now() -> datetime:
            return clock.now()

    winding_module.datetime = FakeDateTime
    events: list[dict[str, Any]] = []

    real_move = Wind.move_motor

    def move_motor(self: Any, motor_id: int, target: float,
                   round_to: int = 3) -> Any:
        controller, effective = _effective_model_target(
            self, winding_module, motor_id, float(target), int(round_to),
        )
        events.append({
            "t": round(float(clock.t), 9),
            "e": "cmd",
            "m": int(motor_id),
            "a": round(effective, 9),
            "model_target": round(effective, 12),
            "requested_model_target": round(float(target), 12),
            "controller_target": controller,
            "serial_round_digits": int(round_to),
            "command": f"M{motor_id}A{controller}\n",
        })
        return real_move(self, motor_id, target, round_to)

    Wind.move_motor = move_motor

    for name in (
        "wind", "wind_wire", "wind_wire_around_shaft", "move_to_teeth",
        "prevent_collision", "set_motor2_wire_position",
        "move_wire_to_right_position",
    ):
        real = getattr(Wind, name)

        def wrapper(self: Any, *args: Any, _real: Any = real,
                    _name: str = name, **kwargs: Any) -> Any:
            events.append({
                "t": round(float(clock.t), 9),
                "e": _name,
                "args": [
                    value for value in args
                    if isinstance(value, (int, float, bool))
                ],
            })
            result = _real(self, *args, **kwargs)
            events.append({
                "t": round(float(clock.t), 9),
                "e": _name + "_done",
                "m2state": str(self.motor2_pos),
            })
            return result

        setattr(Wind, name, wrapper)

    source_commit = _git_text(winder_path, "rev-parse", "HEAD")
    source_status = _git_text(winder_path, "status", "--short")
    source_winding = winder_path / "src" / "winding.py"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    workdir = output_path.parent
    workdir.mkdir(parents=True, exist_ok=True)
    prior_cwd = Path.cwd()
    os.chdir(workdir)
    try:
        wind = Wind(str(settings_path), simulation=False, turns=turns)
        events.append({
            "t": 0.0,
            "e": "meta",
            "capture_schema": CAPTURE_SCHEMA,
            "winder_commit": source_commit,
            "winder_dirty": bool(source_status),
            "winder_status_short": source_status,
            "winder_path": str(winder_path),
            "winding_source_sha256": _sha256(source_winding),
            "controller_mode": "upstream",
            "upstream_transport": "serial_position_digital_twin",
            "upstream_source_subclassed": False,
            "upstream_source_modified_by_harness": False,
            "serial_twin_source_sha256": _sha256(Path(__file__)),
            "settings_sha256": _sha256(settings_path),
            "teeth_count": int(wind.teeth_count),
            "turns": int(wind.turns),
            "settings_turns": int(wind.config["winding"]["turns"]),
            "job": wind.config.get("job"),
            "winding_plan": None,
            "m0_wind_range": list(wind.m0_wind_range),
            "m0_zero": float(wind.m0_zero),
            "shaft_wrap_contract": {
                "source_request": (
                    "absolute m1_zero +/-4*pi; current pinned source does "
                    "not query live M1 before either wrap"
                ),
                "requested_turns": 2.0,
                "controller_quantization_rad_max": 0.0005,
            },
            "m1_rotating_position": float(wind.m1_rotating_position),
            "angle_to_prevent_collision": float(
                wind.m2_angle_to_prevent_collision
            ),
            "velocities": list(model_velocities),
            "controller_velocities": list(controller_velocities),
            "directions": list(wind.rotating_directions),
            "m2_gear_ratio": float(winding_module.m2_gear_ratio),
        })
        wind.continuous_winding()
        events.append({
            "t": round(float(clock.t), 9),
            "e": "cycle_complete",
            "m1_zero": float(wind.m1_zero),
            "m2_zero": float(wind.m2_zero),
        })
        wind.close()
    finally:
        os.chdir(prior_cwd)

    with output_path.open("w", encoding="utf-8", newline="\n") as stream:
        for event in events:
            stream.write(json.dumps(event, separators=(",", ":")))
            stream.write("\n")

    result = {
        "path": str(output_path),
        "sha256": _sha256(output_path),
        "event_count": len(events),
        "command_count": sum(event["e"] == "cmd" for event in events),
        "serial_write_count": len(serial_twin.protocol_log),
        "virtual_duration_s": float(clock.t),
        "winder_commit": source_commit,
        "winder_dirty": bool(source_status),
    }
    return result


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--settings", type=Path, default=DEFAULT_SETTINGS)
    parser.add_argument("--winder", type=Path, default=DEFAULT_WINDER)
    parser.add_argument("--turns", type=int, default=None)
    parser.add_argument("-o", "--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(list(argv) if argv is not None else None)
    result = capture(
        settings_path=args.settings,
        winder_path=args.winder,
        output_path=args.output,
        turns=args.turns,
    )
    print(json.dumps(result, indent=2))
    return 0 if not result["winder_dirty"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
