"""Generate a draft settings.yml for the unmodified winder software from
the CAD parameters (GOAL DoD #4).

All M0 values are radians of the T8x8 lead screw (mm_per_rad documented in
params.py). Semantics per docs/requirements.md §3, including the upstream
quirk that end_to_rotating_position lives under M1.

Usage: python settings_gen.py [--od 46 ... --spindle er11|shaft8]
                              [-o ../out/settings.yml]
"""

import argparse
from dataclasses import replace
import math
from pathlib import Path

from params import (
    DEFAULT_STATOR,
    DEFAULT_SPINDLE_ID,
    PARAMS as P,
    SPINDLE_OPTIONS,
    StatorSpec,
    spindle_option,
)
import coil_growth
import wire_geometry


def derive(
    spec: StatorSpec,
    spindle: str = DEFAULT_SPINDLE_ID,
    *,
    liner_t_mm: float | None = None,
    winding_plan_path: str | None = None,
    winding_plan_proof_sha256: str | None = None,
    hardware_motion_authorized: bool = False,
) -> dict:
    option = spindle_option(spindle)
    liner_t = (
        coil_growth.DEFAULT_POLICY.opening_edge_clearance_mm
        if liner_t_mm is None else float(liner_t_mm)
    )
    if not math.isfinite(liner_t):
        raise SystemExit("liner thickness must be finite")
    if not (coil_growth.DEFAULT_POLICY.liner_receiving_min_mm
            <= liner_t
            <= coil_growth.DEFAULT_POLICY.liner_receiving_max_mm):
        raise SystemExit(
            f"liner thickness {liner_t:.6f} mm is outside receiving range "
            f"[{coil_growth.DEFAULT_POLICY.liner_receiving_min_mm:.3f}, "
            f"{coil_growth.DEFAULT_POLICY.liner_receiving_max_mm:.3f}] mm"
        )
    policy = replace(
        coil_growth.DEFAULT_POLICY,
        opening_edge_clearance_mm=liner_t,
    )
    errs = P.validate(spec, option)
    if errs:
        raise SystemExit("stator fails machine envelope: " + "; ".join(errs))
    try:
        coil = coil_growth.require_feasible(spec, policy)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    contact = wire_geometry.tooth_contact_spec(spec, coil)
    # Traverse exactly the finite radial coil prism at the physical wire lay
    # plane. The old 1 mm..d_max shortcut was unrelated to the finite pack
    # span and could command the wire beyond either end of the modeled coil.
    d_end, d_start = contact["insertion_depth_range_mm"]
    d_max = P.max_insertion(spec, option)
    if d_start > d_max + 1e-9:
        raise SystemExit(
            "stator fails machine envelope: finite coil needs "
            f"{d_start:.3f} mm insertion but spindle {option.id} permits "
            f"{d_max:.3f} mm"
        )
    half_chord = math.sqrt(max(
        spec.od**2 / 4.0 - (spec.od / 2.0 - d_start) ** 2,
        0.0,
    ))
    flyer_radius_needed = (
        math.hypot(half_chord, spec.stack / 2.0)
        + P.wire_bundle_allow + P.dyn_clearance
    )
    if flyer_radius_needed > P.flyer_tip_r + 1e-9:
        raise SystemExit(
            "stator fails machine envelope: flyer tip radius "
            f"{P.flyer_tip_r:.3f} mm is below the finite-coil requirement "
            f"{flyer_radius_needed:.3f} mm"
        )
    r = spec.od / 2.0

    def m0(axis_z):
        return round(P.m0_rad_for_axis_z(axis_z), 3)

    wind_range_start = m0(r - d_start)
    wind_range_end = m0(r - d_end)
    m0_rot = m0(r + 12.0)                # tip 12 mm clear behind plane
    m0_zero = m0(r + 20.0)               # init/zero pose, 20 mm clear
    end_to_rotating = round(m0_rot - wind_range_end, 3)
    end_to_zero = round(m0_zero - wind_range_end, 3)

    # travel checks (model M0 <= 0; deepest must stay above hard stop)
    deepest_z = r - d_start
    assert deepest_z >= P.m0_axis_z_min - 1e-9, \
        f"deepest axis z {deepest_z} below hard stop {P.m0_axis_z_min}"
    assert d_start <= d_max + 1e-9, \
        f"winding depth {d_start} exceeds mechanical limit {d_max}"
    assert wind_range_start < wind_range_end, "range ordering"
    assert abs(wind_range_start) * P.mm_per_rad <= P.m0_travel, \
        "exceeds usable travel"

    return {
        "serial": {"port": "/dev/ttyACM0", "baudrate": 115200},
        "logging": {"level": "INFO"},
        "motor": {
            "M0": {
                "direction": False,
                "wind_range_end": wind_range_end,
                "wind_range_start": wind_range_start,
                "end_to_zero": end_to_zero,
                "velocity": P.m0_velocity_max_rad,
            },
            "M1": {
                "direction": True,
                "zero": 0.0,
                "end_to_rotating_position": end_to_rotating,
                "velocity": P.m1_velocity_max_rad,
            },
            "M2": {
                "direction": True,
                "zero": 0.0,
                "angle_to_prevent_collision": 1.0,
                "velocity": P.m2_velocity_max_rad,
            },
            "M3": {
                "direction": False,
                "pull_wire_torque": 0.15,
                "wind_torque": 0.04,
                "velocity": 5.0,
            },
        },
        "winding": {
            "turns": spec.turns,
            "starts_at": 0,
            "winding_config": spec.winding_config,
            "dont_move_m3": True,
        },
        "job": {
            "spindle_id": option.id,
            "spindle_artifact_id": option.artifact_id,
            "od_mm": spec.od,
            "stack_mm": spec.stack,
            "shaft_d_mm": spec.shaft_d,
            "wire_finished_d_mm": spec.wire_d,
            "slots": spec.slots,
            # Kept under the upstream-adapter's established key for schema
            # compatibility; this is the measured job input, not a supplier
            # nominal or an unbounded worst case.
            "liner_max_thickness_mm": liner_t,
            "liner_measured_thickness_mm": liner_t,
            "liner_receiving_range_mm": [
                coil_growth.DEFAULT_POLICY.liner_receiving_min_mm,
                coil_growth.DEFAULT_POLICY.liner_receiving_max_mm,
            ],
            # The checked-in constructive plan is job-specific.  Never apply
            # it to an arbitrary CLI override merely because dimensions look
            # similar; custom jobs need their own generated proof artifact.
            "winding_plan": (
                winding_plan_path
                if winding_plan_path is not None else
                "reports/slot_winding_plan.json"
                if (spec == DEFAULT_STATOR
                    and option.id == DEFAULT_SPINDLE_ID
                    and liner_t == coil_growth.DEFAULT_POLICY
                    .opening_edge_clearance_mm)
                else None
            ),
            "winding_plan_proof_sha256": winding_plan_proof_sha256,
            # This remains false until both packed crossing routes and the
            # capture-bound continuous interval audit are hash-bound PASS.
            # The production controller rejects hardware construction when
            # it is false; simulation remains available for diagnostics.
            "hardware_motion_authorized": bool(
                hardware_motion_authorized),
            "winding_insertion_depth_mm": [d_end, d_start],
            "radial_winding_span_mm": contact["radial_winding_span_mm"],
            "wire_contact_z_mm": contact["z_mm"],
        },
    }


def to_yaml(cfg: dict) -> str:
    # hand-rolled to keep upstream's exact key order/format
    m = cfg["motor"]
    winding_plan_lines = []
    if cfg["job"].get("winding_plan"):
        winding_plan_lines.append(
            f'  winding_plan: "{cfg["job"]["winding_plan"]}"')
    if cfg["job"].get("winding_plan_proof_sha256"):
        winding_plan_lines.append(
            "  winding_plan_proof_sha256: "
            f'"{cfg["job"]["winding_plan_proof_sha256"]}"')
    winding_plan_block = "\n".join(winding_plan_lines)
    return f"""# Generated by settings_gen.py from machine/cad/params.py
# Machine: 4-axis flyer winder | M0 transmission: T8x8 lead screw,
# {P.mm_per_rad:.5f} mm/rad | flyer tip radius {P.flyer_tip_r} mm
# Spindle: {cfg['job']['spindle_id']} | {cfg['job']['spindle_artifact_id']}
# Job: OD {cfg['job']['od_mm']} mm | stack {cfg['job']['stack_mm']} mm |
# finished wire OD {cfg['job']['wire_finished_d_mm']} mm |
# {cfg['winding']['turns']} turns/tooth
serial:
  port: "{cfg['serial']['port']}"
  baudrate: {cfg['serial']['baudrate']}

logging:
  level: "{cfg['logging']['level']}"

motor:
  M0:
    direction: {str(m['M0']['direction']).lower()}
    wind_range_end: {m['M0']['wind_range_end']}
    wind_range_start: {m['M0']['wind_range_start']}
    end_to_zero: {m['M0']['end_to_zero']}
    velocity: {m['M0']['velocity']}
  M1:
    direction: {str(m['M1']['direction']).lower()}
    zero: {m['M1']['zero']}
    end_to_rotating_position: {m['M1']['end_to_rotating_position']}
    velocity: {m['M1']['velocity']}
  M2:
    direction: {str(m['M2']['direction']).lower()}
    zero: {m['M2']['zero']}
    angle_to_prevent_collision: {m['M2']['angle_to_prevent_collision']}
    velocity: {m['M2']['velocity']}
  M3:
    direction: {str(m['M3']['direction']).lower()}
    pull_wire_torque: {m['M3']['pull_wire_torque']}
    wind_torque: {m['M3']['wind_torque']}
    velocity: {m['M3']['velocity']}

winding:
  turns: {cfg['winding']['turns']}
  starts_at: {cfg['winding']['starts_at']}
  winding_config: "{cfg['winding']['winding_config']}"
  dont_move_m3: {str(cfg['winding']['dont_move_m3']).lower()}

# Project-owned physical job contract.  The upstream controller ignores
# unknown top-level keys; capture/validation retain these values so the
# observed motor stream cannot silently drift from its CAD wire/stator job.
job:
  spindle_id: "{cfg['job']['spindle_id']}"
  spindle_artifact_id: "{cfg['job']['spindle_artifact_id']}"
  od_mm: {cfg['job']['od_mm']}
  stack_mm: {cfg['job']['stack_mm']}
  slots: {cfg['job']['slots']}
  shaft_d_mm: {cfg['job']['shaft_d_mm']}
  wire_finished_d_mm: {cfg['job']['wire_finished_d_mm']}
  wire_radius_job_mm: {cfg['job']['wire_finished_d_mm'] / 2.0}
  liner_max_thickness_mm: {cfg['job']['liner_max_thickness_mm']}
  liner_measured_thickness_mm: {cfg['job']['liner_measured_thickness_mm']}
  liner_receiving_range_mm: {cfg['job']['liner_receiving_range_mm']}
  hardware_motion_authorized: {str(cfg['job']['hardware_motion_authorized']).lower()}
{winding_plan_block}
  winding_insertion_depth_mm: {cfg['job']['winding_insertion_depth_mm']}
  radial_winding_span_mm: {cfg['job']['radial_winding_span_mm']}
  wire_contact_z_mm: {cfg['job']['wire_contact_z_mm']}
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--od", type=float, default=46.0)
    ap.add_argument("--stack", type=float, default=15.0)
    ap.add_argument("--slots", type=int, default=24)
    ap.add_argument("--shaft", type=float, default=4.0)
    ap.add_argument("--wire", type=float, default=DEFAULT_STATOR.wire_d,
                    help="finished enamelled wire outside diameter, mm")
    ap.add_argument(
        "--liner", type=float,
        default=coil_growth.DEFAULT_POLICY.opening_edge_clearance_mm,
        help="measured installed slot-liner thickness, mm",
    )
    ap.add_argument("--turns", type=int, default=DEFAULT_STATOR.turns)
    ap.add_argument("--config", default="AaAabBbBCcCcaAaABbBbcCcC")
    ap.add_argument(
        "--spindle",
        choices=sorted(SPINDLE_OPTIONS),
        default=DEFAULT_SPINDLE_ID,
        help=("physical M1 workholding option; the former --slim flag was "
              "removed because it had no corresponding ER8 CAD"),
    )
    ap.add_argument("-o", "--output",
                    default=str(Path(__file__).parent.parent / "out" /
                                "settings.yml"))
    ap.add_argument(
        "--winding-plan",
        help="project-relative controller plan path regenerated for this job",
    )
    ap.add_argument(
        "--winding-plan-proof-sha256",
        help="hash from the regenerated slot-winding-plan artifact",
    )
    args = ap.parse_args()
    spec = StatorSpec(od=args.od, stack=args.stack, slots=args.slots,
                      shaft_d=args.shaft, wire_d=args.wire, turns=args.turns,
                      winding_config=args.config)
    cfg = derive(
        spec, args.spindle,
        liner_t_mm=args.liner,
        winding_plan_path=args.winding_plan,
        winding_plan_proof_sha256=args.winding_plan_proof_sha256,
    )
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(to_yaml(cfg))
    print(f"wrote {out}")
    mm = P.mm_per_rad
    m = cfg["motor"]
    print(f"  wind range: {m['M0']['wind_range_start']} .. "
          f"{m['M0']['wind_range_end']} rad "
          f"({abs(m['M0']['wind_range_end']-m['M0']['wind_range_start'])*mm:.1f} mm traverse)")
    print(f"  deepest axis z: "
          f"{P.stator_axis_z(m['M0']['wind_range_start']):.1f} mm; "
          f"rotating z: {P.stator_axis_z(m['M0']['wind_range_end']+m['M1']['end_to_rotating_position']):.1f} mm")


if __name__ == "__main__":
    main()
