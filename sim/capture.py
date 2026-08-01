"""Run the UNMODIFIED winder software through a full continuous_winding()
cycle and capture its complete motion command stream.

The software is imported from the cloned repo (../../winder) and executed in
its own simulation mode (Wind(cfg, simulation=True)).  ``--controller
upstream`` captures that checkout byte-for-byte as a negative regression
test.  The default ``--controller contract`` selects the project-owned
subclass in controller_adapter.py, which fixes upstream's proven shaft-wrap
regression and waits for the initial positioning axes to arrive before the
first winding pass.  It preserves the serial protocol and all other winding
behavior.  No upstream source file is modified.

A virtual clock is injected at runtime (the repo's own test suite uses the
same monkeypatch seam for `sleep`): `winding.sleep` advances virtual time
instantly and `winding.datetime.now()` reads it, so the software's internal
constant-velocity motion model produces exactly the trajectory a real-time
run would, in seconds of wall clock instead of ~40 minutes.

Every move_motor() call records both the model-space target used by upstream's
simulation and the exact rounded/direction-mapped serial command a real
controller would receive.
State markers (wind pass, tooth index, shaft wrap, park state) are recorded
by wrapping the corresponding methods.

Output: ../out/capture/commands.jsonl
Usage:  python capture.py [--settings ../out/settings.yml] [--turns N]
                          [--controller contract|upstream]
"""

import argparse
import hashlib
import importlib
import json
import subprocess
import sys
import types
from datetime import datetime, timedelta
from pathlib import Path

HERE = Path(__file__).resolve().parent


class VirtualClock:
    def __init__(self):
        self.t = 0.0
        self.base = datetime(2026, 1, 1)

    def sleep(self, seconds):
        self.t += seconds

    def now(self):
        return self.base + timedelta(seconds=self.t)

    def perf_counter(self):
        return self.t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--settings",
                    default=str(HERE.parent / "out" / "settings.yml"))
    ap.add_argument("--turns", type=int, default=None,
                    help="override winding turns (default: settings value)")
    ap.add_argument("--winder", type=Path,
                    default=HERE.parent.parent / "winder",
                    help="path to an untouched upstream winder checkout")
    ap.add_argument("--controller", choices=("contract", "upstream"),
                    default="contract",
                    help="project corrected controller or raw upstream regression")
    ap.add_argument("-o", "--output",
                    default=str(HERE.parent / "out" / "capture" /
                                "commands.jsonl"))
    args = ap.parse_args()

    winder_path = args.winder.resolve()
    settings_path = Path(args.settings).resolve()
    output_path = Path(args.output).resolve()
    sys.path.insert(0, str(winder_path))
    winding_module = importlib.import_module("src.winding")
    UpstreamWind = winding_module.Wind
    shaft_wrap_contract = None
    if args.controller == "contract":
        from controller_adapter import (
            SHAFT_WRAP_M0_PARK_RAD,
            SHAFT_WRAP_M2_PARK_PHASE_RAD,
            make_contract_wind,
        )
        Wind = make_contract_wind(
            UpstreamWind, winding_module, require_packing_plan=True)
        refine_path = HERE.parent / "out" / "reports" / \
            "shaft_wrap_refine.json"
        refine = (json.loads(refine_path.read_text(encoding="utf-8"))
                  if refine_path.is_file() else {})
        shaft_wrap_contract = {
            "m0_park_rad": SHAFT_WRAP_M0_PARK_RAD,
            "m2_park_phase_rad": SHAFT_WRAP_M2_PARK_PHASE_RAD,
            "refinement_report": str(refine_path),
            "refinement_report_sha256": (
                hashlib.sha256(refine_path.read_bytes()).hexdigest()
                if refine_path.is_file() else None
            ),
            "refinement_status": refine.get("status"),
            "residual_clearance_after_budget_mm": refine.get(
                "residual_clearance_after_budget_mm"),
        }
    else:
        Wind = UpstreamWind

    clock = VirtualClock()
    events = []

    # The upstream simulation mirrors every target/query into SQLite solely
    # for its websocket dashboard.  Capture records the authoritative in-
    # memory model and exact serial commands directly, so those thousands of
    # disk transactions add minutes without changing a single trajectory
    # value.  Use the same callable seams with a close-compatible null handle.
    class NullConnection:
        @staticmethod
        def close():
            return None

    winding_module.init_db = lambda: NullConnection()
    winding_module.update_motor_position = lambda *_args, **_kwargs: None
    winding_module.update_motor_target = lambda *_args, **_kwargs: None

    # ---- runtime injection (test-harness seam; software untouched) ------
    winding_module.sleep = clock.sleep

    class FakeDateTime:
        @staticmethod
        def now():
            return clock.now()

    winding_module.datetime = FakeDateTime
    fake_time = types.SimpleNamespace(perf_counter=clock.perf_counter,
                                      sleep=clock.sleep)
    winding_module.time = fake_time

    real_move = Wind.move_motor

    def move_motor(self, motor_id, target, round_to=3):
        controller_target = self.check_motor_direction(motor_id, target)
        if motor_id == 2:
            controller_target = self.adjust_motor_position_from_gear_ratio(
                controller_target, winding_module.m2_gear_ratio)
        controller_target = round(controller_target, round_to)
        command = f"M{motor_id}A{controller_target}\n"
        events.append({"t": round(clock.t, 4), "e": "cmd",
                       "m": motor_id,
                       "a": round(target, 6),  # legacy model-space key
                       "model_target": round(target, 9),
                       "controller_target": controller_target,
                       "command": command})
        return real_move(self, motor_id, target, round_to)

    Wind.move_motor = move_motor

    if hasattr(Wind, "packing_waypoint_hit"):
        real_waypoint_hit = Wind.packing_waypoint_hit

        def packing_waypoint_hit(self, *args, **kwargs):
            result = real_waypoint_hit(self, *args, **kwargs)
            record = dict(self.packing_waypoint_events[-1])
            record.update({"t": round(clock.t, 4),
                           "e": "packing_waypoint"})
            events.append(record)
            return result

        Wind.packing_waypoint_hit = packing_waypoint_hit

    if hasattr(Wind, "packing_pass_origin"):
        real_pass_origin = Wind.packing_pass_origin

        def packing_pass_origin(self, *args, **kwargs):
            result = real_pass_origin(self, *args, **kwargs)
            record = dict(self.packing_pass_origin_events[-1])
            record.update({"t": round(clock.t, 4),
                           "e": "packing_pass_origin"})
            events.append(record)
            return result

        Wind.packing_pass_origin = packing_pass_origin

    if hasattr(Wind, "shaft_wrap_phase_event"):
        real_shaft_wrap_phase = Wind.shaft_wrap_phase_event

        def shaft_wrap_phase_event(self, *args, **kwargs):
            result = real_shaft_wrap_phase(self, *args, **kwargs)
            record = dict(self.shaft_wrap_phase_events[-1])
            record.update({"t": round(clock.t, 4),
                           "e": "shaft_wrap_phase"})
            events.append(record)
            return result

        Wind.shaft_wrap_phase_event = shaft_wrap_phase_event

    for name in ("wind", "wind_wire", "wind_wire_around_shaft",
                 "move_to_teeth", "prevent_collision",
                 "set_motor2_wire_position", "move_wire_to_right_position"):
        real = getattr(Wind, name)

        def wrapper(self, *a, _real=real, _name=name, **kw):
            events.append({"t": round(clock.t, 4), "e": _name,
                           "args": [x for x in a if isinstance(x, (int,
                                                                   float,
                                                                   bool))]})
            r = _real(self, *a, **kw)
            events.append({"t": round(clock.t, 4), "e": _name + "_done",
                           "m2state": str(self.motor2_pos)})
            return r

        setattr(Wind, name, wrapper)

    # sqlite sidecar (unused by us) needs its data/ dir relative to CWD
    import os
    workdir = HERE.parent / "out" / "capture"
    (workdir / "data").mkdir(parents=True, exist_ok=True)
    os.chdir(workdir)

    wind = Wind(str(settings_path), simulation=True, turns=args.turns)
    source_commit = subprocess.check_output(
        ["git", "-C", str(winder_path), "rev-parse", "HEAD"], text=True).strip()
    adapter_path = HERE / "controller_adapter.py"
    plan = getattr(wind, "_slot_winding_plan", None)
    plan_meta = None
    if plan is not None:
        plan_meta = {
            "schema": plan.raw["schema"],
            "path": str(plan.path),
            "sha256": plan.sha256,
            "proof_sha256": plan.raw.get("proof_sha256"),
            "transition_status": plan.transition_status,
            "nominal_wire_mm": plan.wire_finished_d_mm,
            "model_wire_envelope_mm": plan.model_wire_envelope_mm,
            "receiving_sensitivity_wire_envelope_mm": (
                plan.receiving_sensitivity_wire_envelope_mm),
            "receiving_sensitivity_status": (
                plan.receiving_sensitivity_status),
            "turns_per_tooth": plan.turns_per_tooth,
            "placement_count": len(plan.placements),
            "half_turn_center_count": len(plan.half_turn_centers),
        }
    events.append({"t": 0.0, "e": "meta",
                   "capture_schema": 4,
                   "winder_commit": source_commit,
                   "winder_path": str(winder_path),
                   "controller_mode": args.controller,
                   "controller_adapter_sha256": (
                       hashlib.sha256(adapter_path.read_bytes()).hexdigest()
                       if args.controller == "contract" else None),
                   "settings_sha256": hashlib.sha256(
                       settings_path.read_bytes()).hexdigest(),
                   "teeth_count": wind.teeth_count,
                   "turns": wind.turns,
                   "settings_turns": wind.config["winding"]["turns"],
                   "job": wind.config.get("job"),
                   "winding_plan": plan_meta,
                   "m0_wind_range": list(wind.m0_wind_range),
                   "m0_zero": wind.m0_zero,
                   "shaft_wrap_contract": (
                       {**shaft_wrap_contract,
                        "machine_m2_reference_rad": getattr(
                            wind, "_machine_m2_reference", None)}
                       if shaft_wrap_contract is not None else None),
                   "m1_rotating_position": wind.m1_rotating_position,
                   "angle_to_prevent_collision":
                       wind.m2_angle_to_prevent_collision,
                   "velocities": wind.motor_velocities,
                   "directions": getattr(
                       wind, "rotating_directions",
                       getattr(winding_module, "rotating_directions", None)),
                   "m2_gear_ratio": winding_module.m2_gear_ratio})
    try:
        wind.continuous_winding()
        events.append({"t": round(clock.t, 4), "e": "cycle_complete",
                       "m1_zero": wind.m1_zero, "m2_zero": wind.m2_zero})
    finally:
        wind.close()

    out = output_path
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w") as f:
        for ev in events:
            f.write(json.dumps(ev) + "\n")
    n_cmd = sum(1 for e in events if e["e"] == "cmd")
    print(f"cycle complete: {n_cmd} motor commands, "
          f"virtual duration {clock.t/60:.1f} min -> {out}")
    print(f"final m1_zero drift: {wind.m1_zero:.3f} rad; "
          f"m2_zero: {wind.m2_zero:.3f} rad")


if __name__ == "__main__":
    main()
