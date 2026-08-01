"""Loads sanity check + motor sizing (GOAL DoD #5).

Mass properties from the CAD solids (OCC volume integrals via build123d),
densities per material. Printed parts assumed solid (conservative upper
bound for inertia; the flyer arm is specified 100% infill for balance
predictability anyway).

Sizing requirement (GOAL): selected motors >= 2x the simulation-derived
requirement. M2 is governed by the hash-bound normal-GOAL Leadshine selection
at 36 V and 300 RPM. The older McMaster 6627T421 / 24 V curve is retained only
as a labeled historical baseline and cannot satisfy this report.

Output: ../out/reports/loads.json + console.
"""

import hashlib
import json
import math
from collections import Counter
from pathlib import Path
import sys

from params import PARAMS as P, DEFAULT_STATOR
import assembly

OUT = Path(__file__).parent.parent / "out"
ROOT = Path(__file__).resolve().parent.parent
RAW_UPSTREAM_CAPTURE = OUT / "capture" / "upstream_current_raw.jsonl"
M2_SELECTION_REPORT = OUT / "reports" / "m2_normal_goal_drive_selection.json"
M2_SELECTION_SOURCE = ROOT / "sim" / "m2_normal_goal_drive_selection.py"
LEGACY_M2_MOTOR = "NEMA17 McMaster 6627T421 encoder motor @24V (M2)"

G_CM3 = {  # densities
    "petg": 1.27, "alu": 2.70, "steel": 7.85, "ceramic": 3.9,
    "ptfe": 2.20, "brass": 8.5,
}

REAR_COUNTERWEIGHT_SUFFIX_MATERIALS = {
    "_tungsten_slug": "ASTM-B777 tungsten alloy",
    "_printed_retainer_with_three_spacers": "PETG",
    "_McMaster_94459A130_insert": "brass",
    "_McMaster_92125A126_M3x6_screw": "18-8 stainless steel",
}


def _is_counterweight_mass_row(name: str) -> bool:
    """Return whether an exact rotating row belongs to one of six stacks."""

    return (
        name.startswith("front_trim_B777_")
        or name.startswith("front_trim_hardware_")
        or any(name.endswith(suffix)
               for suffix in REAR_COUNTERWEIGHT_SUFFIX_MATERIALS)
    )


def _counterweight_occurrence_contract(rows, official_pulley) -> dict:
    """Fail closed if the serialized six-stack mass rows drift.

    Counterweight placement authority moved out of ``hardware_placements``.
    The four retained M3 stacks and two front M2 trim stacks now exist as
    individually named release-candidate occurrences.  Keeping the checks
    here prevents a superficially successful loads report that silently drops
    them or assigns the former generic washer-stack material map.
    """

    by_name = {str(row["name"]): row for row in rows}
    if len(by_name) != len(rows):
        raise RuntimeError("current flyer mass rows have duplicate names")

    rear_counts = {}
    for suffix, expected_material in REAR_COUNTERWEIGHT_SUFFIX_MATERIALS.items():
        selected = [
            row for row in rows if str(row["name"]).endswith(suffix)
        ]
        if len(selected) != 4:
            raise RuntimeError(
                f"serialized rear counterweight occurrence drift for {suffix}: "
                f"expected 4, got {len(selected)}"
            )
        materials = Counter(str(row["material"]) for row in selected)
        if materials != Counter({expected_material: 4}):
            raise RuntimeError(
                f"serialized rear counterweight material drift for {suffix}: "
                f"{dict(materials)}"
            )
        rear_counts[suffix] = len(selected)

    front_slugs = [
        row for row in rows
        if str(row["name"]).startswith("front_trim_B777_")
    ]
    if len(front_slugs) != 2 or {
        str(row["material"]) for row in front_slugs
    } != {"ASTM-B777 tungsten alloy"}:
        raise RuntimeError("serialized two-slug front trim contract drift")

    front_hardware = [
        row for row in rows
        if str(row["name"]).startswith("front_trim_hardware_")
    ]
    front_materials = Counter(
        str(row["material"]) for row in front_hardware
    )
    if len(front_hardware) != 6 or front_materials != Counter({
        "steel": 4, "brass": 2,
    }):
        raise RuntimeError(
            "serialized two-stack front trim hardware/material drift: "
            f"count={len(front_hardware)}, materials={dict(front_materials)}"
        )

    pulley = by_name.get("flyer_pulley")
    if pulley is None:
        raise RuntimeError("exact rotating rows omit the flyer D10 pulley")
    if not math.isclose(
        float(pulley["mass_g"]), float(official_pulley.OFFICIAL_MASS_G),
        rel_tol=0.0, abs_tol=1.0e-12,
    ):
        raise RuntimeError("flyer D10 pulley row does not use official mass")
    if not math.isclose(
        float(pulley["izz_about_M2_axis_g_mm2"]),
        float(official_pulley.OFFICIAL_AXIAL_MOMENT_OF_INERTIA_KG_M2) * 1.0e9,
        rel_tol=0.0, abs_tol=1.0e-12,
    ):
        raise RuntimeError("flyer D10 pulley row does not use official axial J")
    if pulley.get("source_step_sha256") != official_pulley.SOURCE_STEP_SHA256:
        raise RuntimeError("flyer D10 pulley mass row source STEP drift")

    counterweight_rows = [
        row for row in rows if _is_counterweight_mass_row(str(row["name"]))
    ]
    if len(counterweight_rows) != 24:
        raise RuntimeError(
            "six counterweight stacks must contain exactly 24 serialized "
            f"mass occurrences, got {len(counterweight_rows)}"
        )
    return {
        "rear_stack_count": 4,
        "front_stack_count": 2,
        "stack_count": 6,
        "serialized_mass_occurrence_count": len(counterweight_rows),
        "rear_occurrence_counts_by_suffix": rear_counts,
        "front_hardware_material_counts": dict(front_materials),
        "official_D10_pulley": {
            "part_number": official_pulley.OFFICIAL_PART_NUMBER,
            "mass_g": float(pulley["mass_g"]),
            "izz_about_M2_axis_g_mm2": float(
                pulley["izz_about_M2_axis_g_mm2"]
            ),
            "source_step_sha256": pulley["source_step_sha256"],
        },
    }


def current_flyer_mass_model() -> dict:
    """Return the exact merged rotating rows without an import cycle.

    The lazy import keeps the inexpensive timing helpers in this module
    usable without building release CAD.  ``integrated_release_candidate``
    does not import ``loads``; it owns the merged D10/PEEK/six-stack balance
    solve and exposes its exact rows through ``rotating_mass_rows``.
    """

    import integrated_release_candidate as candidate
    import nbk_p30_d10_official_occurrence as official_pulley

    source_rows, source_total = candidate.rotating_mass_rows()
    rows = [dict(row) for row in source_rows]
    total = dict(source_total)
    contract = _counterweight_occurrence_contract(rows, official_pulley)

    mass_sum = sum(float(row["mass_g"]) for row in rows)
    izz_sum = sum(float(row["izz_about_M2_axis_g_mm2"]) for row in rows)
    if not math.isclose(
        mass_sum, float(total["mass_g"]), rel_tol=0.0, abs_tol=1.0e-9,
    ):
        raise RuntimeError("exact rotating mass-row sum drift")
    if not math.isclose(
        izz_sum, float(total["izz_about_M2_axis_g_mm2"]),
        rel_tol=0.0, abs_tol=1.0e-6,
    ):
        raise RuntimeError("exact rotating inertia-row sum drift")
    if int(total["part_count"]) != len(rows):
        raise RuntimeError("exact rotating part-count drift")

    return {
        "rows": rows,
        "total": total,
        "counterweight_contract": contract,
        "balance_authority": candidate.integrated_balance_solution()[
            "authority"
        ],
    }

# Selected M0/M1 curves plus the explicitly non-governing historical M2 curve.
MOTORS = {
    "17HS19-2004D-E1K + CL42T-V41 @24V (M0/M1)": {
        "holding_nm": 0.52,
        # Conservative lower-edge digitization of the official 24 V, 2 A,
        # 2000-microstep pull-out curve saved alongside the manufacturer
        # STEP.  The plotted dynamic curve is intentionally used instead of
        # the 0.52 Nm holding-torque headline.
        "curve": [(0, 0.350), (50, 0.350), (100, 0.345),
                  (200, 0.330), (300, 0.330), (400, 0.310),
                  (500, 0.310), (600, 0.310), (700, 0.285),
                  (800, 0.265), (900, 0.245), (1000, 0.245),
                  (1100, 0.220), (1200, 0.220), (1300, 0.195),
                  (1400, 0.175), (1500, 0.155)],
    },
    LEGACY_M2_MOTOR: {
        "holding_nm": 0.880,
        # Conservative lower-edge digitization of the McMaster 24 V, 2 A,
        # half-step torque-speed curve saved as
        # models/upgrades/6627T421_torque_curve.png.  Catalog holding torque
        # is 124.6 in-oz (0.880 Nm); the plotted running curve plateaus near
        # 98-100 in-oz, so the dynamic values below do not use holding torque.
        "curve": [(0, 0.69), (100, 0.70), (200, 0.69), (300, 0.63),
                  (400, 0.55), (600, 0.38), (800, 0.24), (1000, 0.20),
                  (1200, 0.17), (1400, 0.145)],
    },
}


def torque_at_rpm(motor, rpm):
    c = MOTORS[motor]["curve"]
    for (r0, t0), (r1, t1) in zip(c, c[1:]):
        if r0 <= rpm <= r1:
            return t0 + (t1 - t0) * (rpm - r0) / (r1 - r0)
    return c[-1][1]


def selected_m2_drive_authority() -> dict:
    """Load and validate the sole governing M2 motor/drive selection.

    The curve is not copied into this module.  Its exact 36 V / 300 RPM lower
    edge, full bounded demand, P30/210-3GT capacity, and open production gates
    come from ``m2_normal_goal_drive_selection``.  Binding the generated report
    back to its source hash prevents an old report from silently authorizing a
    superseded motor curve.
    """

    if not M2_SELECTION_REPORT.is_file():
        raise RuntimeError(f"missing selected M2 report: {M2_SELECTION_REPORT}")
    if not M2_SELECTION_SOURCE.is_file():
        raise RuntimeError(f"missing selected M2 source: {M2_SELECTION_SOURCE}")
    selection = json.loads(M2_SELECTION_REPORT.read_text(encoding="utf-8"))
    if selection.get("schema") != "m2-normal-goal-drive-selection/v1":
        raise RuntimeError("unsupported selected M2 report schema")
    source_sha = hashlib.sha256(M2_SELECTION_SOURCE.read_bytes()).hexdigest()
    if selection.get("analysis_source_sha256") != source_sha:
        raise RuntimeError("selected M2 report is stale against its source")
    if selection.get("analysis_source") != "sim/m2_normal_goal_drive_selection.py":
        raise RuntimeError("selected M2 report source path drift")

    motor = selection.get("motor", {})
    evidence = selection.get("manufacturer_evidence", {})
    driver = evidence.get("driver_binding", {})
    supply = evidence.get("supply_binding", {})
    duty = selection.get("OD65_10N_full_inertia_torque", {})
    transmission = selection.get("transmission", {})
    gates = selection.get("release_gates", {})
    expected_stack = (
        motor.get("model") == "CS-M21708"
        and driver.get("model") == "CS-D508"
        and supply.get("model") == "LSP-360-36"
        and float(driver.get("selected_supply_vdc", -1.0)) == 36.0
        and float(supply.get("output_voltage_vdc", -1.0)) == 36.0
        and float(selection.get("selection", {}).get("ratio", -1.0)) == 1.0
    )
    if not expected_stack:
        raise RuntimeError("selected M2 motor/driver/supply/ratio identity drift")

    required = float(duty["required_output_torque_nm"])
    threshold = float(duty["required_2x_running_torque_nm"])
    available = float(duty["available_36V_lower_edge_nm"])
    margin = float(duty["available_to_required_multiple"])
    if not math.isclose(threshold, 2.0 * required, rel_tol=0.0, abs_tol=1.0e-12):
        raise RuntimeError("selected M2 2x threshold arithmetic drift")
    if not math.isclose(margin, available / required,
                        rel_tol=0.0, abs_tol=1.0e-12):
        raise RuntimeError("selected M2 curve-margin arithmetic drift")
    static_gate = bool(duty["manufacturer_curve_motor_gate_ge_2x"])
    if static_gate != (margin >= 2.0):
        raise RuntimeError("selected M2 static curve gate drift")

    driver_configured = bool(
        gates.get("driver_configuration_reproduces_RMS_2p5A_curve")
    )
    hot_dyno = bool(gates.get("hot_36V_300rpm_dyno_ge_required_2x"))
    production = bool(selection.get("production_authorized"))
    if driver_configured or hot_dyno or production:
        raise RuntimeError(
            "selected M2 report unexpectedly promotes an open production gate"
        )

    transmission_capacity = float(transmission["allowable_transmission_torque_nm"])
    transmission_margin = float(transmission["allowable_to_required_multiple"])
    if not math.isclose(
        transmission_margin, transmission_capacity / required,
        rel_tol=0.0, abs_tol=1.0e-12,
    ):
        raise RuntimeError("selected M2 transmission-margin arithmetic drift")

    return {
        "role": "governing_selected_M2_authority",
        "report_path": str(M2_SELECTION_REPORT.relative_to(ROOT)).replace("\\", "/"),
        "report_sha256": hashlib.sha256(
            M2_SELECTION_REPORT.read_bytes()
        ).hexdigest(),
        "source_path": str(M2_SELECTION_SOURCE.relative_to(ROOT)).replace("\\", "/"),
        "source_sha256": source_sha,
        "motor": "Leadshine CS-M21708 closed-loop NEMA17",
        "driver": "Leadshine CS-D508",
        "supply": "Leadshine LSP-360-36",
        "ratio": 1.0,
        "curve_condition": motor["curve_condition"],
        "available_300rpm_lower_edge_nm": available,
        "governing_required_output_torque_nm": required,
        "governing_required_2x_torque_nm": threshold,
        "available_to_required_multiple": margin,
        "static_curve_margin_gate_ge_2x": static_gate,
        "driver_current_configuration_verified": driver_configured,
        "installed_hot_dyno_verified": hot_dyno,
        "production_authorized": production,
        "transmission": {
            "selection": "NBK P30 30T:30T / 210-3GT-6 exact 1:1",
            "allowable_torque_nm": transmission_capacity,
            "allowable_to_required_multiple": transmission_margin,
            "capacity_gate_ge_2x": bool(
                transmission["transmission_capacity_gate_ge_2x"]
            ),
        },
    }


def minimum_trapezoid_time(distance_rad, velocity_rad_s, accel_rad_s2):
    """Minimum symmetric accel/cruise/decel time for one rotary move."""

    distance = abs(float(distance_rad))
    velocity = float(velocity_rad_s)
    accel = float(accel_rad_s2)
    if velocity <= 0.0 or accel <= 0.0:
        raise ValueError("velocity and acceleration must be positive")
    accel_and_decel_distance = velocity * velocity / accel
    if distance <= accel_and_decel_distance:
        return 2.0 * math.sqrt(distance / accel)
    return (2.0 * velocity / accel
            + (distance - accel_and_decel_distance) / velocity)


def raw_upstream_timing_evidence():
    """Bind fixed upstream sleeps to the selected M0/M1 speed settings.

    The upstream program does not wait for either the first M0 insertion or
    the two between-phase M1 moves.  A raw capture is therefore load/timing
    evidence, not just animation input.  This check deliberately refuses the
    project ContractWind capture because that adapter adds arrival waits.
    """

    if not RAW_UPSTREAM_CAPTURE.is_file():
        return {
            "status": "FAIL",
            "reason": f"missing raw capture: {RAW_UPSTREAM_CAPTURE}",
        }
    sim_dir = Path(__file__).parent.parent / "sim"
    if str(sim_dir) not in sys.path:
        sys.path.insert(0, str(sim_dir))
    from traj import Timeline, load_events

    events = load_events(RAW_UPSTREAM_CAPTURE)
    meta = next((event for event in events if event.get("e") == "meta"), {})
    timeline = Timeline(events)
    settings_path = OUT / "settings.yml"
    settings_hash = (
        hashlib.sha256(settings_path.read_bytes()).hexdigest()
        if settings_path.is_file() else None)
    first_flyer = next(
        event for event in events
        if (event.get("e") == "cmd" and event.get("m") == 2
            and abs(float(event.get("a", 0.0))) > 1.0e-9))
    first_pose = timeline.pose_at(float(first_flyer["t"]))
    wind_low, wind_high = sorted(map(float, meta["m0_wind_range"]))
    m0_tolerance = 0.01
    first_m0_in_span = bool(
        wind_low - m0_tolerance <= first_pose[0]
        <= wind_high + m0_tolerance)
    first_m0_at_deep_target = bool(
        abs(first_pose[0] - wind_low) <= m0_tolerance)
    m0_accel = 50.0
    conservative_m0_time = minimum_trapezoid_time(
        wind_low, P.m0_velocity_max_rad, m0_accel)

    wraps = []
    m1_accel = 50.0
    wrap_calls = [
        event for event in events
        if event.get("e") == "wind_wire_around_shaft"]
    for call in wrap_calls:
        start_t = float(call["t"])
        start_m1 = float(timeline.pose_at(start_t)[1])
        command = next(
            event for event in events
            if (event.get("e") == "cmd" and event.get("m") == 1
                and float(event["t"]) >= start_t - 1.0e-12))
        target = float(command["a"])
        distance = abs(target - start_m1)
        available = 1.5
        calculated = minimum_trapezoid_time(
            distance, P.m1_velocity_max_rad, m1_accel)
        observed = float(timeline.pose_at(start_t + available)[1])
        wraps.append({
            "next_wire_idx": int(call["args"][0]),
            "start_m1_rad": start_m1,
            "target_m1_rad": target,
            "distance_rad": distance,
            "available_s": available,
            "minimum_trapezoid_time_s": calculated,
            "time_margin_s": available - calculated,
            "observed_after_sleep_rad": observed,
            "observed_error_rad": observed - target,
            "arrived_in_raw_timeline": abs(observed - target) <= 0.01,
            "timing_bound_pass": calculated <= available,
        })

    checks = {
        "controller_mode_is_raw_upstream": (
            meta.get("controller_mode") == "upstream"),
        "settings_hash_matches_capture": (
            settings_hash is not None
            and meta.get("settings_sha256") == settings_hash),
        "configured_m0_velocity_matches_capture": math.isclose(
            float(meta["velocities"][0]), P.m0_velocity_max_rad,
            rel_tol=0.0, abs_tol=1.0e-12),
        "configured_m1_velocity_matches_capture": math.isclose(
            float(meta["velocities"][1]), P.m1_velocity_max_rad,
            rel_tol=0.0, abs_tol=1.0e-12),
        "first_flyer_move_starts_inside_wind_span": first_m0_in_span,
        "first_flyer_move_starts_at_deep_target": first_m0_at_deep_target,
        "conservative_m0_move_fits_raw_setup_window": (
            conservative_m0_time <= float(first_flyer["t"])),
        "two_raw_shaft_wrap_calls_present": len(wraps) == 2,
        "all_raw_shaft_wrap_moves_arrive_before_sleep_ends": (
            len(wraps) == 2
            and all(row["arrived_in_raw_timeline"] for row in wraps)),
        "all_raw_shaft_wrap_moves_pass_acceleration_bound": (
            len(wraps) == 2
            and all(row["timing_bound_pass"] for row in wraps)),
    }
    return {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "capture": str(RAW_UPSTREAM_CAPTURE),
        "capture_sha256": hashlib.sha256(
            RAW_UPSTREAM_CAPTURE.read_bytes()).hexdigest(),
        "capture_schema": meta.get("capture_schema"),
        "winder_commit": meta.get("winder_commit"),
        "settings_sha256": settings_hash,
        "first_flyer_move": {
            "time_s": float(first_flyer["t"]),
            "observed_m0_rad": float(first_pose[0]),
            "required_wind_span_rad": [wind_low, wind_high],
            "conservative_zero_to_deep_time_s": conservative_m0_time,
            "time_margin_s": float(first_flyer["t"]) - conservative_m0_time,
            "acceleration_rad_s2": m0_accel,
        },
        "shaft_wrap_moves": wraps,
        "checks": checks,
    }


def main():
    # The legacy assembly link intentionally no longer duplicates the
    # candidate-owned counterweights.  Consume the exact merged balance rows
    # so the four retained M3 stacks, two front M2 trim stacks, official stock
    # D10 pulley mass/J and every other rotating occurrence are counted once.
    flyer_model = current_flyer_mass_model()
    exact_rows = flyer_model["rows"]
    exact_total = flyer_model["total"]
    counterweight_rows = [
        row for row in exact_rows
        if _is_counterweight_mass_row(str(row["name"]))
    ]
    total_m = float(exact_total["mass_g"])
    total_izz = float(exact_total["izz_about_M2_axis_g_mm2"])
    com_y_moment = float(exact_total["static_first_moment_g_mm"][1])
    cw_mass = sum(float(row["mass_g"]) for row in counterweight_rows)
    cw_moment = sum(
        float(row["static_first_moment_g_mm"][1])
        for row in counterweight_rows
    )
    rows = [
        {
            "part": str(row["name"]),
            "material": str(row["material"]),
            "mass_g": round(float(row["mass_g"]), 3),
            "izz_gmm2": round(float(row["izz_about_M2_axis_g_mm2"]), 3),
            "com_y": round(float(row["center_of_mass_mm"][1]), 3),
        }
        for row in exact_rows
    ]

    izz_kgm2 = total_izz * 1e-9
    imbalance_gmm = float(exact_total["static_imbalance_g_mm"])
    resid_force_300 = abs(imbalance_gmm) * 1e-6 * (2 * math.pi * 300 / 60) ** 2

    # --- M2 ---------------------------------------------------------------
    selected_m2 = selected_m2_drive_authority()
    alpha = 200.0                      # rad/s^2 (reach 20 rad/s in 0.1 s)
    t_acc = izz_kgm2 * alpha
    # Two tension-torque models:
    #  lever (ultra-conservative): full tension acting at tip radius
    #  energy (physical): work/rev = k_capstan * F * wire consumed/rev
    #    (worst tooth perimeter ~60 mm at OD90), x2 dynamic peak factor
    t_lever = 10.0 * P.flyer_tip_r / 1000.0            # 0.45 Nm
    t_energy_peak = 2.0 * (1.5 * 10.0 * 0.060 / (2 * math.pi))  # 0.29 Nm
    friction_allowance = 0.020
    current_screen_required = t_acc + t_energy_peak + friction_allowance
    current_lever_required = t_acc + t_lever + friction_allowance
    selected_available = selected_m2["available_300rpm_lower_edge_nm"]
    current_screen_margin = selected_available / current_screen_required
    current_lever_margin = selected_available / current_lever_required

    # Preserve the retired curve arithmetic only as named historical context.
    # It is not consulted by any selected sizing or Definition-of-Done gate.
    legacy_required = t_acc + t_energy_peak
    legacy_margin_191 = torque_at_rpm(LEGACY_M2_MOTOR, 191) / legacy_required
    legacy_margin_lever = torque_at_rpm(LEGACY_M2_MOTOR, 191) / (
        t_acc + t_lever
    )
    legacy_margin_300 = torque_at_rpm(LEGACY_M2_MOTOR, 300) / legacy_required

    # --- M0 ---------------------------------------------------------------
    carriage_mass_g = sum(
        pp.volume * (G_CM3["petg"] if pp.label in
                     ("carriage_plate", "spindle_tower", "nut_bracket")
                     else G_CM3["steel"]) / 1000.0
        for pp in assembly.carriage_link()) + 350  # + motor 350 g
    f_axial = 10.0 + 0.02 * carriage_mass_g * 9.81e-3  # tension + rail fric
    eta = 0.5                                     # POM anti-backlash nut
    t_m0 = f_axial * (P.m0_lead / 1000.0) / (2 * math.pi * eta)
    motor17 = "17HS19-2004D-E1K + CL42T-V41 @24V (M0/M1)"
    rpm_m0 = P.m0_velocity_max_rad * 60 / (2 * math.pi)
    m0_margin = torque_at_rpm(motor17, rpm_m0) / t_m0

    # --- M1 ---------------------------------------------------------------
    # wrap torque: 10 N at 4 mm shaft radius; indexing accel of stator+chuck
    def iyy_about_spindle(pp):
        com = pp.center()
        return (pp.matrix_of_inertia[1][1] +
                pp.volume * (com.X**2 + (com.Z - P.m0_home_standoff)**2))
    spindle_izz = sum(
        iyy_about_spindle(pp) * (G_CM3["steel"] / 1000.0)
        for pp in assembly.spindle_link()) * 1e-9
    t_m1 = 10.0 * 0.004 + spindle_izz * 50.0 + 0.02
    rpm_m1 = P.m1_velocity_max_rad * 60 / (2 * math.pi)
    m1_margin = torque_at_rpm(motor17, rpm_m1) / t_m1
    coupling_m0_margin = P.coupling_5x8_dynamic_reversing_nm / t_m0
    coupling_m1_margin = P.coupling_5x8_dynamic_reversing_nm / t_m1
    raw_timing = raw_upstream_timing_evidence()
    static_sizing_pass = all((
        selected_m2["static_curve_margin_gate_ge_2x"],
        selected_m2["transmission"]["capacity_gate_ge_2x"],
        m0_margin >= 2.0,
        m1_margin >= 2.0,
        coupling_m0_margin >= 2.0,
        coupling_m1_margin >= 2.0,
        raw_timing.get("status") == "PASS",
    ))
    production_load_authorized = all((
        static_sizing_pass,
        selected_m2["driver_current_configuration_verified"],
        selected_m2["installed_hot_dyno_verified"],
        selected_m2["production_authorized"],
    ))
    post_purchase_motion_qualification = {
        "status": "PASS" if production_load_authorized else "BLOCKED",
        "required_for_definition_of_done_5": False,
        "required_before_energized_motion": True,
        "driver_current_configuration_verified": selected_m2[
            "driver_current_configuration_verified"
        ],
        "installed_hot_dyno_verified": selected_m2[
            "installed_hot_dyno_verified"
        ],
        "selected_stack_production_authorized": selected_m2[
            "production_authorized"
        ],
        "note": (
            "These commissioning gates require delivered hardware. They are "
            "not part of GOAL Definition of Done #5, which is the CAD-derived "
            "mass/inertia and >=2x motor/transmission sizing check. They remain "
            "mandatory before energized production motion."
        ),
    }

    report = {
        "schema": "machine-loads/v2",
        "status": (
            "DOD5_PASS_PRODUCTION_QUALIFICATION_PASS"
            if production_load_authorized
            else "DOD5_PASS_PRODUCTION_QUALIFICATION_OPEN"
            if static_sizing_pass
            else "DOD5_FAIL_PRODUCTION_QUALIFICATION_BLOCKED"
        ),
        "static_sizing_pass": static_sizing_pass,
        "analytical_order_input_ready": static_sizing_pass,
        "definition_of_done_5": {
            "status": "PASS" if static_sizing_pass else "FAIL",
            "pass": static_sizing_pass,
            "requires_post_purchase_hardware": False,
            "scope": (
                "CAD mass properties, flyer inertia and drive torque at "
                "300 RPM with tensioned wire, M0 traverse load, M1 indexing "
                "torque, and >=2x selected motor/transmission margins"
            ),
        },
        "post_purchase_motion_qualification": (
            post_purchase_motion_qualification
        ),
        "production_authorized": production_load_authorized,
        "source_hashes": {
            "cad/loads.py": hashlib.sha256(
                Path(__file__).resolve().read_bytes()
            ).hexdigest(),
            selected_m2["source_path"]: selected_m2["source_sha256"],
            selected_m2["report_path"]: selected_m2["report_sha256"],
        },
        "flyer": {"parts": rows, "mass_g": round(total_m, 3),
                  "izz_kgm2": izz_kgm2,
                  "mass_model_authority": flyer_model["balance_authority"],
                  "counterweight_occurrence_contract": flyer_model[
                      "counterweight_contract"
                  ],
                  "counterweight_mass_g": round(cw_mass, 3),
                  "counterweight_moment_g_mm": round(abs(cw_moment), 2),
                  "required_counterweight_moment_g_mm":
                      round(com_y_moment - cw_moment, 2),
                  "imbalance_g_mm": round(imbalance_gmm, 1),
                  "residual_force_at_300rpm_N": round(resid_force_300, 2)},
        "motors": {
            "m2": selected_m2["motor"],
            "m2_driver": selected_m2["driver"],
            "m2_supply": selected_m2["supply"],
            "m0_m1": motor17,
        },
        "m2": {
            "governing_selected_authority": selected_m2,
            "current_geometry_supporting_screen": {
                "role": (
                    "supporting conservative current-flyer screen; the "
                    "hash-bound selected authority remains governing"
                ),
                "t_accel_nm": round(t_acc, 6),
                "t_energy_peak_nm": round(t_energy_peak, 6),
                "t_lever_worstcase_nm": round(t_lever, 6),
                "friction_allowance_nm": friction_allowance,
                "energy_model_required_nm": round(
                    current_screen_required, 6
                ),
                "energy_model_selected_curve_margin": round(
                    current_screen_margin, 6
                ),
                "lever_model_required_nm": round(current_lever_required, 6),
                "lever_model_selected_curve_margin": round(
                    current_lever_margin, 6
                ),
                "selected_curve_value_is_300rpm_lower_edge_nm": (
                    selected_available
                ),
            },
            "pulley": {
                **selected_m2["transmission"],
                "requirement": ">= 2.0",
            },
            "release_gates": {
                "definition_of_done_5_pass": static_sizing_pass,
                "selected_static_curve_margin_ge_2x": selected_m2[
                    "static_curve_margin_gate_ge_2x"
                ],
                "selected_transmission_margin_ge_2x": selected_m2[
                    "transmission"
                ]["capacity_gate_ge_2x"],
                "driver_current_configuration_verified": selected_m2[
                    "driver_current_configuration_verified"
                ],
                "installed_hot_dyno_verified": selected_m2[
                    "installed_hot_dyno_verified"
                ],
                "production_authorized": production_load_authorized,
            },
            "legacy_baseline": {
                "role": "historical_non_governing_baseline",
                "non_governing": True,
                "motor": LEGACY_M2_MOTOR,
                "condition": "24 V, 2 A, half-step",
                "curve_source": (
                    "cad/models/upgrades/6627T421_torque_curve.png"
                ),
                "required_without_selected_friction_nm": round(
                    legacy_required, 6
                ),
                "margin_at_191rpm": round(legacy_margin_191, 6),
                "margin_lever_model": round(legacy_margin_lever, 6),
                "margin_at_300rpm_10N": round(legacy_margin_300, 6),
                "reason_non_governing": (
                    "superseded by Leadshine CS-M21708 / CS-D508 / "
                    "LSP-360-36 and prohibited as DoD #5 authority"
                ),
            },
        },
        "m0": {"axial_force_n": round(f_axial, 1),
               "t_required_nm": round(t_m0, 4),
               "margin": round(m0_margin, 1),
               "carriage_mass_g": round(carriage_mass_g, 0),
               "curve_source":
                   "cad/models/upgrades/17HS19-2004D-E1K_Torque_Curve.pdf",
               "curve_condition": "24 V, 2.0 A, 2000 microstep"},
        "m1": {"t_required_nm": round(t_m1, 4),
               "margin": round(m1_margin, 1),
               "curve_source":
                   "cad/models/upgrades/17HS19-2004D-E1K_Torque_Curve.pdf",
               "curve_condition": "24 V, 2.0 A, 2000 microstep"},
        "m0_m1_couplings": {
            "selection": "Ruland PCMR22-8-5-A",
            "dynamic_reversing_capacity_nm":
                P.coupling_5x8_dynamic_reversing_nm,
            "m0_margin_vs_sim_duty": round(coupling_m0_margin, 2),
            "m1_margin_vs_sim_duty": round(coupling_m1_margin, 2),
            "shaft_engagement_mm_each": 12.5,
            "manufacturer_max_penetration_mm":
                P.coupling_5x8_shaft_penetration,
            "requirement": ">= 2.0",
        },
        "raw_upstream_timing": raw_timing,
        "requirement": (
            "GOAL DoD #5 requires CAD-derived loads and selected motor and "
            "transmission margins >=2.0. Post-purchase motion authorization "
            "separately requires verified CS-D508 current configuration and "
            "an installed hot dyno."
        ),
    }
    (OUT / "reports").mkdir(parents=True, exist_ok=True)
    (OUT / "reports" / "loads.json").write_text(json.dumps(report, indent=2))

    print(f"flyer link: {total_m:.0f} g, Izz {izz_kgm2*1e6:.1f} g*m^2 "
          f"= {izz_kgm2:.2e} kg*m^2")
    for r in rows:
        print(f"  {r['part']:54s} {r['mass_g']:8.3f} g  "
              f"com_y {r['com_y']}")
    print(f"imbalance {imbalance_gmm:+.0f} g*mm -> residual force "
          f"{resid_force_300:.2f} N @300rpm")
    print(
        f"\nM2 [{selected_m2['motor']} + {selected_m2['driver']} @36 V]: "
        f"governing need {selected_m2['governing_required_output_torque_nm']:.6f} "
        f"Nm at 300 RPM -> static curve margin "
        f"{selected_m2['available_to_required_multiple']:.3f}x"
    )
    print(
        f"  legacy baseline [{LEGACY_M2_MOTOR}]: "
        f"{legacy_margin_191:.2f}x at 191 RPM; NON-GOVERNING"
    )
    print(f"M0: need {t_m0:.3f} Nm -> margin {m0_margin:.1f}x")
    print(f"M1: need {t_m1:.3f} Nm -> margin {m1_margin:.1f}x")
    print("Raw upstream fixed-delay timing:", raw_timing["status"])
    if "first_flyer_move" in raw_timing:
        first = raw_timing["first_flyer_move"]
        print(f"  first flyer M0={first['observed_m0_rad']:.3f} rad; "
              f"conservative timing margin {first['time_margin_s']:.3f} s")
        for wrap in raw_timing["shaft_wrap_moves"]:
            print(f"  wrap {wrap['next_wire_idx']}: "
                  f"{wrap['distance_rad']:.3f} rad, "
                  f"margin {wrap['time_margin_s']:.3f} s, "
                  f"arrival error {wrap['observed_error_rad']:.6f} rad")
    print("STATIC SIZING RESULT:", "PASS" if static_sizing_pass else "FAIL")
    print(
        "PRODUCTION LOAD AUTHORITY:",
        "PASS" if production_load_authorized else "BLOCKED",
        "(driver current configuration and hot dyno remain open)"
        if not production_load_authorized else "",
    )
    return report


if __name__ == "__main__":
    main()
