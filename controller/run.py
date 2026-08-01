"""Run the project-owned winding controller against an upstream checkout.

The upstream repository remains untouched.  This entry point loads its user
interface and serial implementation, then substitutes the narrowly scoped
``ContractWind`` class from ``sim/controller_adapter.py``.  The substitution
fixes the current upstream shaft-wrap regression and waits for M1 to arrive;
all serial commands and the rest of the winding sequence remain upstream's.

Examples (run from ``machine``):

    .venv/Scripts/python controller/run.py --check
    .venv/Scripts/python controller/run.py --simulation
    .venv/Scripts/python controller/run.py --settings out/settings.yml

The final command opens the real upstream menu and can command hardware.
Review the serial port in the generated settings file before using it.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
DEFAULT_WINDER = ROOT.parent / "winder"
DEFAULT_SETTINGS = ROOT / "out" / "settings.yml"
ADAPTER = ROOT / "sim" / "controller_adapter.py"


def _upstream_commit(winder: Path) -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=winder, text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unavailable"


def _load(winder: Path):
    if not (winder / "src" / "winding.py").is_file():
        raise FileNotFoundError(
            f"not a winder checkout (missing src/winding.py): {winder}"
        )
    sys.path.insert(0, str(winder))
    sys.path.insert(0, str(ROOT / "sim"))
    winding = importlib.import_module("src.winding")
    ui = importlib.import_module("scripts.main")
    adapter = importlib.import_module("controller_adapter")
    contract_wind = adapter.make_contract_wind(
        winding.Wind, winding, require_packing_plan=True)
    return winding, ui, contract_wind, adapter


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run the audited project controller over upstream winder"
    )
    parser.add_argument("--winder", type=Path, default=DEFAULT_WINDER)
    parser.add_argument("--settings", type=Path, default=DEFAULT_SETTINGS)
    parser.add_argument(
        "--simulation", action="store_true",
        help="use upstream's local simulation backend; never open the serial port",
    )
    parser.add_argument("--turns", type=int, default=None)
    parser.add_argument(
        "--check", action="store_true",
        help="verify imports and provenance without constructing a controller",
    )
    args = parser.parse_args(argv)

    winder = args.winder.resolve()
    settings = args.settings.resolve()
    winding, ui, contract_wind, adapter = _load(winder)
    adapter_hash = hashlib.sha256(ADAPTER.read_bytes()).hexdigest()
    print(f"upstream: {winder} @ {_upstream_commit(winder)}")
    print(f"adapter:  {ADAPTER} sha256={adapter_hash}")
    print(f"settings: {settings}")
    if not settings.is_file():
        raise FileNotFoundError(f"settings file does not exist: {settings}")
    if args.check:
        config = winding.load_config(str(settings))
        plan_ref = config.get("job", {}).get("winding_plan")
        if not plan_ref:
            raise RuntimeError(
                "settings has no job.winding_plan; production controller "
                "refuses the upstream ease-out schedule")
        plan = adapter.load_slot_winding_plan(settings.parent / plan_ref)
        plan.validate_settings(config)
        if not plan.controller_ready:
            raise RuntimeError("winding plan transition proof is not PASS")
        if config.get("job", {}).get(
                "hardware_motion_authorized") is not True:
            raise RuntimeError(
                "hardware motion is not authorized: packed-route and "
                "continuous captured-interval audits are not both "
                "hash-bound PASS"
            )
        print(
            "winding plan: "
            f"{plan.path} sha256={plan.sha256} "
            f"placements={len(plan.placements)} "
            f"half_turn_centers={len(plan.half_turn_centers)} "
            f"transition={plan.transition_status}"
        )
        print("contract controller import: PASS")
        return 0

    wind = contract_wind(str(settings), args.simulation, turns=args.turns)
    try:
        ui.main(wind)
    except KeyboardInterrupt:
        if not args.simulation:
            wind.estop()
        print("Keyboard interrupt detected. Exiting...")
    finally:
        wind.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
