"""Project-owned compatibility controller for the winder serial contract.

The upstream project is MIT licensed and remains an untouched checkout.  Its
current ``wind_wire_around_shaft`` implementation regressed from a relative
4*pi move to an absolute ``m1_zero +/- 4*pi`` target, so the physical travel
depends on whichever tooth ended the phase.  It also waits a fixed 1.5 s
instead of confirming arrival.

The same upstream sequence begins winding after a fixed 0.5 second initial
positioning delay.  At the configured M0 velocity, the launch move from zero
to ``m0_zero`` takes more than four seconds, so the first flyer revolution can
begin before the carriage reaches the finite winding span.  This adapter also
arrival-checks that initial pose before allowing ``continuous_winding`` to
continue.

Both corrections inherit the rest of upstream's winding sequence and serial
protocol verbatim.  An optional versioned packing schedule can additionally
replace only upstream's ease-out-sine M0 law with explicit M2-phase/M0-target
waypoints.  It includes the pass-dependent lead-out travel, checks actual M0
arrival at every target phase with one-waypoint lookahead, and then resumes the
unchanged collision-prevention and parking sequence.  With no schedule the
upstream ``wind_wire`` implementation is called directly.

The adapter is selected explicitly by ``capture.py --controller contract``
and recorded in capture metadata; the raw upstream behavior remains capturable
as a negative regression test with ``--controller upstream``.
"""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping

try:  # Package import in tests, direct import in controller/run.py.
    from .winding_plan import load_slot_winding_plan
except ImportError:  # pragma: no cover - direct sim-path import
    from winding_plan import load_slot_winding_plan


# Two encoder quadrature counts on the selected 1000 PPR M0 feedback path are
# about 0.00314 rad (0.0040 mm on the T8x8 screw).  The former 0.02 rad gate
# could certify a center 0.0255 mm away from its regenerated packing target.
PACKING_M0_SETTLE_TOLERANCE_RAD = 0.0035
PACKING_M2_TARGET_TOLERANCE_RAD = 0.005
PACKING_PHASE_EVENT_EPS_RAD = 1.0e-9

# Collision-checked shaft-wrap pose.  M0 model-space zero puts the spindle
# axis at z=95 mm in the released CAD, and M2=+45 degrees (relative to the
# immutable machine reference) keeps the complete two-turn M1 sweep clear.
# ``shaft_wrap_refine.py`` samples the surrounding tolerance box and binds the
# result to the exported link manifest.
SHAFT_WRAP_M0_PARK_RAD = 0.0
SHAFT_WRAP_M2_PARK_PHASE_RAD = math.pi / 4.0


@dataclass(frozen=True)
class PackingWaypoint:
    """One M0 target at a non-negative flyer-travel phase."""

    m2_phase_rad: float
    m0_target_rad: float
    placement_index: int | None = None
    kind: str = "schedule"


@dataclass(frozen=True)
class PackingPass:
    """Identity and complete actual M2 travel for one upstream tooth pass."""

    teeth_idx: int
    wind_idx: int
    clockwise: bool
    m2_travel_rad: float
    waypoints: tuple[PackingWaypoint, ...]


@dataclass(frozen=True)
class PackingSchedule:
    """Sequential, fail-closed schedule consumed across ``wind_wire`` calls."""

    version: int
    passes: tuple[PackingPass, ...]


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{field} must be a finite number, not bool")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


def _integer(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{field} must be an integer")
    if value < 0:
        raise ValueError(f"{field} must be non-negative")
    return value


def normalize_packing_schedule(
    schedule: PackingSchedule | Mapping[str, Any] | None,
) -> PackingSchedule | None:
    """Validate the public version-1 radians schema.

    The final waypoint phase must equal the *actual* upstream M2 travel,
    including any half-turn lead-out selected by ``get_target_motor2_pos``.
    Holding M0 after the last packed loop is represented by additional
    waypoints through that lead-out phase.
    """

    if schedule is None:
        return None
    if isinstance(schedule, PackingSchedule):
        raw: Mapping[str, Any] = {
            "version": schedule.version,
            "passes": [
                {
                    "teeth_idx": item.teeth_idx,
                    "wind_idx": item.wind_idx,
                    "clockwise": item.clockwise,
                    "m2_travel_rad": item.m2_travel_rad,
                    "waypoints": [
                        {"m2_phase_rad": point.m2_phase_rad,
                         "m0_target_rad": point.m0_target_rad}
                        for point in item.waypoints
                    ],
                }
                for item in schedule.passes
            ],
        }
    elif isinstance(schedule, Mapping):
        raw = schedule
    else:
        raise ValueError("packing schedule must be a mapping or PackingSchedule")
    if set(raw) != {"version", "passes"}:
        raise ValueError("packing schedule keys must be exactly version, passes")
    if raw["version"] != 1:
        raise ValueError("packing schedule version must be 1")
    raw_passes = raw["passes"]
    if not isinstance(raw_passes, list) or not raw_passes:
        raise ValueError("packing schedule passes must be a non-empty list")

    passes: list[PackingPass] = []
    for pass_index, item in enumerate(raw_passes):
        prefix = f"passes[{pass_index}]"
        if not isinstance(item, Mapping) or set(item) != {
            "teeth_idx", "wind_idx", "clockwise", "m2_travel_rad",
            "waypoints",
        }:
            raise ValueError(
                f"{prefix} keys must be exactly teeth_idx, wind_idx, "
                "clockwise, m2_travel_rad, waypoints"
            )
        teeth_idx = _integer(item["teeth_idx"], f"{prefix}.teeth_idx")
        wind_idx = _integer(item["wind_idx"], f"{prefix}.wind_idx")
        if not isinstance(item["clockwise"], bool):
            raise ValueError(f"{prefix}.clockwise must be bool")
        travel = _finite_number(
            item["m2_travel_rad"], f"{prefix}.m2_travel_rad")
        if travel <= 0.0:
            raise ValueError(f"{prefix}.m2_travel_rad must be positive")
        raw_waypoints = item["waypoints"]
        if not isinstance(raw_waypoints, list) or len(raw_waypoints) < 2:
            raise ValueError(f"{prefix}.waypoints must contain at least two points")
        waypoints: list[PackingWaypoint] = []
        previous = -math.inf
        for waypoint_index, point in enumerate(raw_waypoints):
            point_prefix = f"{prefix}.waypoints[{waypoint_index}]"
            if not isinstance(point, Mapping) or set(point) != {
                "m2_phase_rad", "m0_target_rad",
            }:
                raise ValueError(
                    f"{point_prefix} keys must be exactly "
                    "m2_phase_rad, m0_target_rad"
                )
            phase = _finite_number(
                point["m2_phase_rad"], f"{point_prefix}.m2_phase_rad")
            target = _finite_number(
                point["m0_target_rad"], f"{point_prefix}.m0_target_rad")
            if phase <= previous:
                raise ValueError(
                    f"{prefix} waypoint phases must be strictly increasing")
            if phase < 0.0 or phase > travel + 1e-9:
                raise ValueError(f"{point_prefix}.m2_phase_rad outside pass travel")
            waypoints.append(PackingWaypoint(phase, target))
            previous = phase
        if abs(waypoints[0].m2_phase_rad) > 1e-9:
            raise ValueError(f"{prefix} first waypoint phase must be zero")
        if abs(waypoints[-1].m2_phase_rad - travel) > 1e-9:
            raise ValueError(
                f"{prefix} final waypoint phase must equal m2_travel_rad")
        passes.append(PackingPass(
            teeth_idx=teeth_idx,
            wind_idx=wind_idx,
            clockwise=item["clockwise"],
            m2_travel_rad=travel,
            waypoints=tuple(waypoints),
        ))
    return PackingSchedule(version=1, passes=tuple(passes))


def make_contract_wind(upstream_cls, winding_module, packing_schedule=None,
                       require_packing_plan=False):
    """Return the contract subclass, optionally with a waypoint schedule."""

    default_schedule = normalize_packing_schedule(packing_schedule)
    load_schedule_from_config = packing_schedule is None

    class ContractWind(upstream_cls):
        def __init__(self, *args, **kwargs):
            config_arg = args[0] if args else kwargs.get("config_path")
            super().__init__(*args, **kwargs)
            # Upstream deliberately redefines ``m2_zero`` after each phase
            # handoff.  Preserve the assembly's immutable CAD/reference zero
            # so every shaft wrap returns to the same proven physical pose.
            self._machine_m2_reference = float(
                getattr(self, "m2_zero", 0.0)
            )
            configured_schedule = None
            job_config = None
            if load_schedule_from_config:
                config = getattr(self, "config", None)
                if isinstance(config, Mapping):
                    job = config.get("job")
                    if isinstance(job, Mapping):
                        job_config = job
                        configured_schedule = job.get("packing_schedule")
            self._packing_schedule = (
                normalize_packing_schedule(configured_schedule)
                if load_schedule_from_config else default_schedule
            )
            self._slot_winding_plan = None
            plan_ref = None
            if isinstance(getattr(self, "config", None), Mapping):
                job = self.config.get("job")
                if isinstance(job, Mapping):
                    job_config = job
                    plan_ref = job.get("winding_plan")
            if (require_packing_plan and not self.simulation
                    and (not isinstance(job_config, Mapping)
                         or job_config.get("hardware_motion_authorized")
                         is not True)):
                raise RuntimeError(
                    "hardware motion is not authorized: regenerate and "
                    "hash-bind PASS packed-route and continuous captured-"
                    "interval audits before setting "
                    "job.hardware_motion_authorized=true"
                )
            if self._packing_schedule is not None and plan_ref is not None:
                raise ValueError(
                    "settings cannot combine packing_schedule and winding_plan")
            if plan_ref is not None:
                if not isinstance(plan_ref, str) or not plan_ref.strip():
                    raise ValueError("settings job.winding_plan must be a path")
                if config_arg is None:
                    raise ValueError(
                        "cannot resolve winding plan without a settings path")
                plan_path = Path(config_arg).resolve().parent / plan_ref
                plan = load_slot_winding_plan(plan_path)
                plan.validate_settings(self.config)
                if not self.simulation and not plan.controller_ready:
                    raise RuntimeError(
                        "winding plan transition proof is not PASS; hardware "
                        "motion is refused")
                self._slot_winding_plan = plan
            elif require_packing_plan and self._packing_schedule is None:
                raise RuntimeError(
                    "contract controller requires a validated winding plan "
                    "for this job; generate job.winding_plan or explicitly "
                    "capture --controller upstream as a negative baseline")
            self._packing_pass_cursor = 0
            self.packing_waypoint_events = []
            self.packing_pass_origin_events = []
            self.shaft_wrap_phase_events = []

        def set_packing_schedule(self, schedule):
            """Install/reset an optional schedule before winding starts."""
            self._packing_schedule = normalize_packing_schedule(schedule)
            self._slot_winding_plan = None
            self._packing_pass_cursor = 0
            self.packing_waypoint_events = []
            self.packing_pass_origin_events = []

        def shaft_wrap_phase_event(self, phase, next_wire_idx):
            """Record an observed shaft-wrap phase at the physical axes."""
            event = {
                "phase": str(phase),
                "next_wire_idx": int(next_wire_idx),
                "m0_rad": float(self.get_motor_position(0)),
                "m1_rad": float(self.get_motor_position(1)),
                "m2_rad": float(self.get_motor_position(2)),
            }
            self.shaft_wrap_phase_events.append(event)
            logger = getattr(self, "logger", None)
            if logger is not None:
                logger.info(
                    "Shaft wrap phase=%s wire=%d M0=%.6f M1=%.6f M2=%.6f",
                    event["phase"], event["next_wire_idx"], event["m0_rad"],
                    event["m1_rad"], event["m2_rad"],
                )

        @staticmethod
        def _nearest_equivalent_angle(reference, phase, current):
            """Return ``reference + phase + 2*pi*k`` nearest ``current``."""
            base = float(reference) + float(phase)
            turns = math.floor(
                (float(current) - base) / (2.0 * math.pi) + 0.5
            )
            return base + turns * 2.0 * math.pi

        def _packing_pass(self, teeth_idx, clockwise, wind_idx):
            schedule = self._packing_schedule
            if schedule is None:
                return None
            cursor = self._packing_pass_cursor
            if cursor >= len(schedule.passes):
                raise RuntimeError(
                    f"packing schedule exhausted before pass {cursor}: "
                    f"tooth={teeth_idx}, wind_idx={wind_idx}"
                )
            item = schedule.passes[cursor]
            actual = (teeth_idx, wind_idx, bool(clockwise))
            expected = (item.teeth_idx, item.wind_idx, item.clockwise)
            if actual != expected:
                raise RuntimeError(
                    f"packing pass {cursor} identity mismatch: "
                    f"expected {expected}, got {actual}"
                )
            low, high = sorted(map(float, self.m0_wind_range))
            for index, waypoint in enumerate(item.waypoints):
                if not low - 1e-9 <= waypoint.m0_target_rad <= high + 1e-9:
                    raise RuntimeError(
                        f"packing pass {cursor} waypoint {index} M0 target "
                        f"{waypoint.m0_target_rad:.6f} outside winding range "
                        f"[{low:.6f}, {high:.6f}]"
                    )
            return item

        def packing_waypoint_hit(self, pass_index, waypoint_index,
                                 waypoint, observed_phase_rad,
                                 observed_m0_rad,
                                 m0_ready_phase_rad=None):
            """Record a capture-friendly, protocol-neutral waypoint event."""
            event = {
                "pass_index": int(pass_index),
                "waypoint_index": int(waypoint_index),
                "m2_phase_rad": float(waypoint.m2_phase_rad),
                "observed_m2_phase_rad": float(observed_phase_rad),
                "m0_target_rad": float(waypoint.m0_target_rad),
                "observed_m0_rad": float(observed_m0_rad),
                "m0_error_rad": float(
                    observed_m0_rad - waypoint.m0_target_rad),
                "placement_index": waypoint.placement_index,
                "kind": waypoint.kind,
                "m0_ready_phase_rad": float(
                    observed_phase_rad if m0_ready_phase_rad is None
                    else m0_ready_phase_rad),
                "m0_settled_before_crossing": bool(
                    (observed_phase_rad if m0_ready_phase_rad is None
                     else m0_ready_phase_rad)
                    <= waypoint.m2_phase_rad + 1e-9),
            }
            self.packing_waypoint_events.append(event)
            self.logger.info(
                "Packing waypoint pass=%d point=%d phase=%.6f/%.6f "
                "M0=%.6f/%.6f",
                event["pass_index"], event["waypoint_index"],
                event["observed_m2_phase_rad"], event["m2_phase_rad"],
                event["observed_m0_rad"], event["m0_target_rad"],
            )

        def packing_pass_origin(self, pass_index, start_phase_rad,
                                first_crossing_phase_rad, actual_travel_rad):
            event = {
                "pass_index": int(pass_index),
                "start_phase_rad": float(start_phase_rad),
                "first_crossing_phase_rad": float(first_crossing_phase_rad),
                "phase_origin_rad": float(first_crossing_phase_rad),
                "actual_travel_rad": float(actual_travel_rad),
                "final_hold_phase_rad": float(actual_travel_rad),
                "expected_deposition_center_count": (
                    2 * self._slot_winding_plan.turns_per_tooth),
                "placement_zero_settled_before_first_crossing": True,
                "pre_crossing_deposition_count": 0,
            }
            self.packing_pass_origin_events.append(event)
            self.logger.info(
                "Packing origin pass=%d start=%.6f first=%.6f target=%.6f",
                event["pass_index"], event["start_phase_rad"],
                event["first_crossing_phase_rad"],
                event["actual_travel_rad"],
            )

        def _validate_packing_timing(self, packing_pass, poll_dt=0.01):
            """Prove M0 can settle between every two flyer crossings.

            A waypoint is the required M0 position *at* its M2 phase.  The
            next M0 target is issued immediately after that crossing, leaving
            one complete waypoint interval for the carriage to arrive.  The
            velocity calculation reserves two polling periods for command and
            observation latency; a plan that cannot meet that bound is
            rejected before M2 moves.
            """
            m0_velocity = max(float(self.motor_velocities[0]), 1e-9)
            m2_velocity = max(float(self.motor_velocities[2]), 1e-9)
            rows = []
            for index, (left, right) in enumerate(zip(
                    packing_pass.waypoints,
                    packing_pass.waypoints[1:])):
                available_s = (
                    right.m2_phase_rad - left.m2_phase_rad
                ) / m2_velocity
                required_s = abs(
                    right.m0_target_rad - left.m0_target_rad
                ) / m0_velocity
                reserve_s = 2.0 * poll_dt
                margin_s = available_s - required_s - reserve_s
                if margin_s < -1e-12:
                    raise RuntimeError(
                        f"packing pass {self._packing_pass_cursor} interval "
                        f"{index}->{index + 1} cannot settle M0 before the "
                        f"next crossing: available={available_s:.6f}s, "
                        f"required={required_s:.6f}s, "
                        f"poll reserve={reserve_s:.6f}s"
                    )
                rows.append({
                    "from_waypoint": index,
                    "to_waypoint": index + 1,
                    "available_s": available_s,
                    "required_s": required_s,
                    "poll_reserve_s": reserve_s,
                    "margin_s": margin_s,
                })
            return rows

        def _wait_for_motor(self, motor_id, target, tolerance=0.01, dt=0.02):
            """Poll one commanded axis until its physical position arrives."""
            start = self.get_motor_position(motor_id)
            velocity = max(float(self.motor_velocities[motor_id]), 1e-6)
            max_polls = int(abs(target - start) / velocity / dt) + 250
            for _ in range(max_polls):
                position = self.get_motor_position(motor_id)
                if abs(position - target) <= tolerance:
                    return position
                winding_module.sleep(dt)
            raise TimeoutError(
                f"M{motor_id} did not reach {target:.6f} rad; "
                f"last position {position:.6f} rad"
            )

        def init_position(self, pull_wire=False):
            """Preserve upstream setup, then prove every positioning axis ready."""
            super().init_position(pull_wire)
            for motor_id, target in (
                (0, self.m0_zero),
                (1, self.m1_zero),
                (2, self.m2_zero),
            ):
                self._wait_for_motor(motor_id, target)

        def _packing_pass_from_slot_plan(self, teeth_idx, clockwise,
                                         wind_idx, actual_travel,
                                         first_crossing_phase=0.0):
            plan = self._slot_winding_plan
            if plan is None:
                return None
            points = plan.controller_waypoints(
                self.config["job"]["radial_winding_span_mm"],
                self.m0_wind_range,
                actual_travel,
                first_crossing_phase,
            )
            return PackingPass(
                teeth_idx=int(teeth_idx),
                wind_idx=int(wind_idx),
                clockwise=bool(clockwise),
                m2_travel_rad=float(actual_travel),
                waypoints=tuple(PackingWaypoint(
                    float(point["m2_phase_rad"]),
                    float(point["m0_target_rad"]),
                    point.get("placement_index"),
                    point.get("kind", "schedule"),
                ) for point in points),
            )

        def wind_wire(self, teeth_idx: int, clockwise, wind_idx):
            packing_pass = self._packing_pass(teeth_idx, clockwise, wind_idx)
            slot_plan = self._slot_winding_plan
            if packing_pass is None and slot_plan is None:
                return super().wind_wire(teeth_idx, clockwise, wind_idx)

            if slot_plan is not None:
                packing_pass = self._packing_pass_from_slot_plan(
                    teeth_idx, clockwise, wind_idx,
                    slot_plan.turns_per_tooth * 2.0 * math.pi,
                )

            poll_dt = 0.01
            self.packing_timing = self._validate_packing_timing(
                packing_pass, poll_dt=poll_dt)

            # Preserve upstream staging, tension, flyer-state, and settling
            # semantics.  Only the M0 packing law is replaced.
            self.move_to_teeth(teeth_idx)
            self.set_wire_tension(1)
            self.move_motor(0, self.m0_wind_range[1])
            winding_module.sleep(0.8)
            self.set_motor2_wire_position()
            winding_module.sleep(0.2)

            initial = packing_pass.waypoints[0]
            self.move_motor(0, initial.m0_target_rad)
            winding_module.sleep(1.2)
            observed_m0 = self._wait_for_motor(0, initial.m0_target_rad)
            pass_index = self._packing_pass_cursor
            if slot_plan is None:
                self.packing_waypoint_hit(
                    pass_index, 0, initial, 0.0, observed_m0)
                # Point 1 is issued immediately after point 0 is physically
                # proven. Every later command follows the same lookahead.
                waypoints = packing_pass.waypoints
                next_crossing = 1
                self.move_motor(0, waypoints[next_crossing].m0_target_rad)
            else:
                # Placement zero is physically settled, but is not counted as
                # deposited until the flyer reaches the first actual slot-side
                # crossing. Collision-offset passes begin between crossings.
                waypoints = packing_pass.waypoints
                next_crossing = 0

            init_motor2_pos = self.get_init_motor2_pos()
            target_motor2_pos = self.get_target_motor2_pos(clockwise, wind_idx)
            signed_travel = target_motor2_pos - init_motor2_pos
            expected_direction = 1.0 if clockwise else -1.0
            if signed_travel * expected_direction <= 0.0:
                raise RuntimeError(
                    f"packing pass {pass_index} M2 direction mismatch: "
                    f"clockwise={clockwise}, signed travel={signed_travel:.9f}"
                )
            actual_travel = abs(signed_travel)
            if slot_plan is not None:
                current_motor2_pos = self.get_motor_position(2)
                start_phase = expected_direction * (
                    current_motor2_pos - init_motor2_pos)
                if start_phase < -1e-6:
                    # Collision parking can leave the flyer just before the
                    # nearest slot-side crossing in the next winding
                    # direction.  Express that same physical pose from the
                    # preceding half-turn crossing instead of rejecting it or
                    # inventing an early deposit.
                    if start_phase < -math.pi - 1e-6:
                        raise RuntimeError(
                            f"packing pass {pass_index} starts more than one "
                            f"half-turn before its origin: {start_phase:.9f} "
                            "rad")
                    init_motor2_pos -= expected_direction * math.pi
                    signed_travel = target_motor2_pos - init_motor2_pos
                    actual_travel = abs(signed_travel)
                    start_phase += math.pi
                if start_phase > math.pi + 1e-6:
                    raise RuntimeError(
                        f"packing pass {pass_index} starts outside one "
                        f"half-turn origin window: {start_phase:.9f} rad")
                start_phase = max(0.0, start_phase)
                first_crossing_phase = (
                    0.0 if start_phase <= 1e-6
                    else math.ceil((start_phase - 1e-9) / math.pi) * math.pi
                )
                # A pass that begins inside its first half-turn needs one
                # additional half-turn beyond the nominal upstream target:
                # 100 placement crossings alone leave the conductor on the
                # far side of its last turn.  Extend only short targets to the
                # next physical crossing so capture can prove that turn's
                # closure.  This is an actual M2 move, not a synthetic event.
                closure_travel = (
                    first_crossing_phase
                    + slot_plan.turns_per_tooth * 2.0 * math.pi
                )
                if actual_travel < closure_travel:
                    actual_travel = closure_travel
                    signed_travel = expected_direction * actual_travel
                    target_motor2_pos = init_motor2_pos + signed_travel
                    self.logger.info(
                        "Extending packing pass %d M2 target to physical "
                        "closure phase %.9f rad",
                        pass_index, closure_travel,
                    )
                packing_pass = self._packing_pass_from_slot_plan(
                    teeth_idx, clockwise, wind_idx, actual_travel,
                    first_crossing_phase)
                # The nominal and actual plans share every deposition point;
                # only a pass-dependent final lead-out hold may be appended.
                self.packing_timing = self._validate_packing_timing(
                    packing_pass, poll_dt=poll_dt)
                waypoints = packing_pass.waypoints
                if abs(initial.m0_target_rad
                       - waypoints[0].m0_target_rad) > 1e-9:
                    raise RuntimeError(
                        "slot plan placement-zero target changed after phase "
                        "origin resolution")
                self.packing_pass_origin(
                    pass_index, start_phase, first_crossing_phase,
                    actual_travel)
                if first_crossing_phase <= start_phase + 1e-6:
                    # Zero-offset pass: phase zero is an actual crossing and
                    # placement zero was already proven before motion.
                    self.packing_waypoint_hit(
                        pass_index, 0, waypoints[0], start_phase,
                        observed_m0)
                    next_crossing = 1
                    self.move_motor(
                        0, waypoints[next_crossing].m0_target_rad)
                else:
                    # Offset pass: keep placement zero through the remaining
                    # partial half-turn; do not invent a phase-zero deposit.
                    next_crossing = 0
            elif abs(actual_travel - packing_pass.m2_travel_rad) > 1e-6:
                raise RuntimeError(
                    f"packing pass {pass_index} M2 travel mismatch: "
                    f"schedule={packing_pass.m2_travel_rad:.9f}, "
                    f"upstream={actual_travel:.9f}"
                )

            # M0 is always commanded one waypoint ahead: immediately after
            # crossing point i, target i+1 is issued and has the entire phase
            # interval to settle.  The first revolution retains upstream's
            # cautious twelve 30-degree targets, but is implemented here so
            # those phases are observed instead of hidden inside
            # ``fast_winding``.  The remainder is one continuous M2 target.
            # Together these cover the former blind first and final turns.
            last_phase = start_phase if slot_plan is not None else 0.0
            m0_tolerance = PACKING_M0_SETTLE_TOLERANCE_RAD
            m2_tolerance = PACKING_M2_TARGET_TOLERANCE_RAD
            velocity = max(float(self.motor_velocities[2]), 1e-9)
            ready_m0 = observed_m0
            ready_phase = last_phase
            next_target_ready = (
                next_crossing < len(waypoints)
                and abs(ready_m0
                        - waypoints[next_crossing].m0_target_rad)
                <= m0_tolerance
            )

            def fail_closed(message, motor2_pos):
                # The long M2 command is asynchronous.  Hold the observed
                # position before raising so a failed pass cannot keep winding
                # open-loop while the caller handles the exception.
                self.move_motor(2, motor2_pos)
                raise RuntimeError(message)

            def observe_motor2(motor2_pos):
                nonlocal next_crossing, last_phase
                nonlocal next_target_ready, ready_m0, ready_phase
                directed_phase = expected_direction * (
                    motor2_pos - init_motor2_pos)
                observed_phase = min(
                    actual_travel, max(last_phase, directed_phase))
                last_phase = observed_phase

                if next_crossing >= len(waypoints):
                    return
                waypoint = waypoints[next_crossing]
                # Never claim a deposition crossing early.  The former
                # ``observed + 0.01 >= target`` condition could issue the next
                # M0 command while the flyer was still 0.01 rad before the
                # physical crossing.  Encoder/poll quantization may make this
                # observation late, which is safe because readiness was
                # already latched before the crossing.
                crossed = (
                    observed_phase + PACKING_PHASE_EVENT_EPS_RAD
                    >= waypoint.m2_phase_rad)
                if not crossed:
                    if not next_target_ready:
                        candidate_m0 = self.get_motor_position(0)
                        if abs(candidate_m0
                               - waypoint.m0_target_rad) <= m0_tolerance:
                            next_target_ready = True
                            ready_m0 = candidate_m0
                            ready_phase = observed_phase
                    return

                # More than one crossed waypoint in one observation means its
                # required M0 pose was never verified at the crossing.  Never
                # catch up and claim a late hit.
                if (next_crossing + 1 < len(waypoints)
                        and observed_phase + PACKING_PHASE_EVENT_EPS_RAD >=
                        waypoints[next_crossing + 1].m2_phase_rad):
                    fail_closed(
                        f"packing pass {pass_index} skipped waypoint "
                        f"{next_crossing}: observed phase "
                        f"{observed_phase:.6f} rad",
                        motor2_pos,
                    )

                if not next_target_ready:
                    fail_closed(
                        f"packing pass {pass_index} waypoint "
                        f"{next_crossing} reached phase before M0 was "
                        "observed settled",
                        motor2_pos,
                    )
                self.packing_waypoint_hit(
                    pass_index, next_crossing, waypoint,
                    observed_phase, ready_m0, ready_phase)
                next_crossing += 1
                if next_crossing < len(waypoints):
                    self.move_motor(
                        0, waypoints[next_crossing].m0_target_rad)
                    # A repeated radial center is already settled at the new
                    # target; otherwise a pre-crossing poll must prove arrival.
                    next_target_ready = (
                        abs(ready_m0
                            - waypoints[next_crossing].m0_target_rad)
                        <= m0_tolerance)
                    if next_target_ready:
                        ready_phase = observed_phase

            def move_m2_and_observe(segment_target, settle_s=0.0):
                """Drive one upstream M2 segment while enforcing crossings."""
                segment_start = self.get_motor_position(2)
                self.move_motor(2, segment_target)
                if self.simulation:
                    # Retain upstream fast_winding's simulation update seam.
                    self.get_motor_position(2)
                if settle_s:
                    winding_module.sleep(settle_s)
                segment_travel = abs(segment_target - segment_start)
                max_polls = int(
                    segment_travel / velocity / poll_dt
                ) + 250
                for _ in range(max_polls):
                    motor2_pos = self.get_motor_position(2)
                    observe_motor2(motor2_pos)
                    if abs(motor2_pos - segment_target) <= m2_tolerance:
                        return
                    winding_module.sleep(poll_dt)

                motor2_pos = self.get_motor_position(2)
                self.move_motor(2, motor2_pos)
                raise TimeoutError(
                    f"packing pass {pass_index} M2 segment did not reach "
                    f"{segment_target:.6f} rad"
                )

            # Upstream fast_winding is one revolution in twelve incremental
            # commands based on the current *commanded* M2 pose.  Keeping that
            # basis matters when get_init_motor2_pos removes a collision
            # offset: absolute targets derived from the logical init would
            # otherwise make the first command reverse direction.
            fast_step = expected_direction * math.pi * 2.0 / 12.0
            for _ in range(12):
                move_m2_and_observe(
                    self.motor_positions[2] + fast_step,
                    settle_s=0.05,
                )

            # Preserve upstream's exact final absolute M2 target after the
            # cautious first revolution.
            move_m2_and_observe(target_motor2_pos)

            # Retain upstream's final flyer-settle interval before collision
            # prevention and parking, then confirm the same exact target.
            winding_module.sleep(0.5)
            settled_motor2 = self.get_motor_position(2)
            if abs(settled_motor2 - target_motor2_pos) >= 0.1:
                self.move_motor(2, settled_motor2)
                raise RuntimeError(
                    f"packing pass {pass_index} M2 final tracking error: "
                    f"target={target_motor2_pos:.9f}, "
                    f"actual={settled_motor2:.9f}"
                )

            if next_crossing != len(waypoints):
                raise RuntimeError(
                    f"packing pass {pass_index} reached exact M2 target with "
                    f"{len(waypoints) - next_crossing} unverified waypoint(s)"
                )

            if slot_plan is not None:
                centers = [
                    event for event in self.packing_waypoint_events
                    if event["pass_index"] == pass_index
                    and event["kind"] == "placement_center"
                ]
                counts = {
                    placement: sum(
                        event["placement_index"] == placement
                        for event in centers)
                    for placement in range(slot_plan.turns_per_tooth)
                }
                if (len(centers) != 2 * slot_plan.turns_per_tooth
                        or any(value != 2 for value in counts.values())):
                    raise RuntimeError(
                        f"packing pass {pass_index} did not deposit exactly "
                        "two half-turn centers per placement")
                if any(event["m2_phase_rad"] + 1e-9
                       < first_crossing_phase for event in centers):
                    raise RuntimeError(
                        f"packing pass {pass_index} counted a deposition "
                        "before its first physical crossing")
                closure_phase = (
                    first_crossing_phase
                    + 2.0 * slot_plan.turns_per_tooth * math.pi
                )
                closures = [
                    event for event in self.packing_waypoint_events
                    if event["pass_index"] == pass_index
                    and event["kind"] == "final_hold"
                    and abs(event["m2_phase_rad"] - closure_phase)
                    <= PACKING_PHASE_EVENT_EPS_RAD
                ]
                if (len(closures) != 1
                        or closures[0]["observed_m2_phase_rad"]
                        + PACKING_PHASE_EVENT_EPS_RAD < closure_phase):
                    raise RuntimeError(
                        f"packing pass {pass_index} did not record one "
                        "physical post-deposition closure crossing")

            final_target = waypoints[-1].m0_target_rad
            self._wait_for_motor(0, final_target, tolerance=m0_tolerance)

            self.logger.info(f"Winding teeth {teeth_idx} done")
            skip_prevent_collision_teeth_idx = [self.teeth_count - 1]
            if teeth_idx not in skip_prevent_collision_teeth_idx:
                self.prevent_collision(clockwise)
            winding_module.sleep(0.7)
            self.move_motor(0, self.m1_rotating_position)
            winding_module.sleep(1.5)
            self._packing_pass_cursor += 1

        def continuous_winding(self, *args, **kwargs):
            result = super().continuous_winding(*args, **kwargs)
            schedule = self._packing_schedule
            if (schedule is not None
                    and self._packing_pass_cursor != len(schedule.passes)):
                raise RuntimeError(
                    "packing schedule was not fully consumed: "
                    f"{self._packing_pass_cursor}/{len(schedule.passes)} passes"
                )
            if (self._slot_winding_plan is not None
                    and self._packing_pass_cursor != self.teeth_count):
                raise RuntimeError(
                    "slot winding plan did not cover every tooth pass: "
                    f"{self._packing_pass_cursor}/{self.teeth_count}")
            return result

        def wind_wire_around_shaft(self, next_wire_idx: int):
            self.starts_at = 0
            signed_turn = (
                -1.0 if self.is_starting_from_cw(next_wire_idx) else 1.0
            )
            delta = signed_turn * 4.0 * math.pi

            state = winding_module.Motor2State
            valid_pre_wrap_states = {
                state.TOP_LEFT, state.TOP_RIGHT,
                state.BOTTOM_LEFT, state.BOTTOM_RIGHT,
            }
            if self.motor2_pos not in valid_pre_wrap_states:
                raise RuntimeError(
                    "motor2_pos must be a parked left/right state before "
                    "shaft wrap; "
                    f"got {self.motor2_pos}"
                )

            # Parking is deliberately outside the sleeve-contact interval.
            # First retract M0 to the CAD proof plane, then put M2 at the one
            # physical flyer angle whose complete M1 sweep passed the refined
            # collision study.  Every axis is observed at target before the
            # contact marker or any M1 winding motion is issued.
            self.shaft_wrap_phase_event("prepark_start", next_wire_idx)
            self.move_motor(0, SHAFT_WRAP_M0_PARK_RAD)
            self._wait_for_motor(
                0, SHAFT_WRAP_M0_PARK_RAD,
                tolerance=PACKING_M0_SETTLE_TOLERANCE_RAD,
            )
            self.shaft_wrap_phase_event("m0_parked", next_wire_idx)

            current_m2 = self.get_motor_position(2)
            m2_target = self._nearest_equivalent_angle(
                self._machine_m2_reference,
                SHAFT_WRAP_M2_PARK_PHASE_RAD,
                current_m2,
            )
            self.move_motor(2, m2_target)
            self._wait_for_motor(
                2, m2_target, tolerance=PACKING_M2_TARGET_TOLERANCE_RAD,
            )
            # Re-prove M0 after the M2 park move; a drifting carriage must not
            # silently enter the shaft-contact interval.
            self._wait_for_motor(
                0, SHAFT_WRAP_M0_PARK_RAD,
                tolerance=PACKING_M0_SETTLE_TOLERANCE_RAD,
            )
            self.shaft_wrap_phase_event("contact_start", next_wire_idx)

            # The physical contract is exactly two turns from the observed
            # current spindle position while M0 and M2 remain fixed at their
            # proven park pose.
            start = self.get_motor_position(1)
            target = start + delta
            self.move_motor(1, target)
            self._wait_for_motor(1, target)
            self.m1_zero += delta
            self.shaft_wrap_phase_event("contact_done", next_wire_idx)

            self.motor2_pos = state.TOP
            self.m2_zero = m2_target

    ContractWind.__name__ = "ContractWind"
    return ContractWind
