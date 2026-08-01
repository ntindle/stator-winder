"""Unit tests for the project-owned shaft-wrap controller primitive."""

import math
from pathlib import Path
import unittest
from enum import Enum

from controller_adapter import (PACKING_M0_SETTLE_TOLERANCE_RAD,
                                 SHAFT_WRAP_M0_PARK_RAD,
                                 SHAFT_WRAP_M2_PARK_PHASE_RAD,
                                 make_contract_wind,
                                 normalize_packing_schedule)
from winding_plan import load_slot_winding_plan


ROOT = Path(__file__).resolve().parents[1]
PLAN_PATH = ROOT / "out" / "reports" / "slot_winding_plan.json"


class Motor2State(Enum):
    TOP = 0
    BOTTOM = 1
    TOP_LEFT = 2
    TOP_RIGHT = 3
    BOTTOM_LEFT = 4
    BOTTOM_RIGHT = 5


class FakeModule:
    Motor2State = Motor2State

    @staticmethod
    def sleep(_seconds):
        return None


class FakeWind:
    def __init__(self, clockwise, m1_start, flyer_state):
        self.clockwise = clockwise
        self.m0_zero = -40.0
        self.positions = [self.m0_zero, m1_start, 1.25, 0.0]
        self.motor_positions = list(self.positions)
        self.motor_velocities = [10.0, 5.0, 20.0, 5.0]
        self.m1_zero = 0.375
        self.m2_zero = 0.0
        self.m2_angle_to_prevent_collision = 1.0
        self.motor2_pos = flyer_state
        self.moves = []
        self.starts_at = 9
        self.upstream_wind_calls = []

    def is_starting_from_cw(self, _wire_idx):
        return self.clockwise

    def get_motor_position(self, motor_id):
        return self.positions[motor_id]

    def move_motor(self, motor_id, target):
        self.moves.append((motor_id, target))
        self.positions[motor_id] = target
        self.motor_positions[motor_id] = target

    def init_position(self, pull_wire=False):
        self.move_motor(1, self.m1_zero)
        self.move_motor(0, getattr(self, "m0_zero", 0.0))
        self.move_motor(2, self.m2_zero)

    def wind_wire(self, teeth_idx, clockwise, wind_idx):
        self.upstream_wind_calls.append((teeth_idx, clockwise, wind_idx))
        return "upstream"


class DelayedStartupWind:
    """Minimal upstream stand-in whose M0 arrives over several polls."""

    def __init__(self):
        self.motor_velocities = [10.0, 5.0, 20.0, 5.0]
        self.motor_positions = [0.0, 0.0, 0.0, 0.0]
        self.actual = [0.0, 0.0, 0.0, 0.0]
        self.m0_zero = -40.0
        self.m1_zero = 0.0
        self.m2_zero = 0.0
        self.polls = [0, 0, 0, 0]

    def init_position(self, pull_wire=False):
        self.motor_positions = [self.m0_zero, self.m1_zero,
                                self.m2_zero, 0.0]

    def get_motor_position(self, motor_id):
        self.polls[motor_id] += 1
        target = self.motor_positions[motor_id]
        delta = target - self.actual[motor_id]
        if abs(delta) <= 5.0:
            self.actual[motor_id] = target
        else:
            self.actual[motor_id] += math.copysign(5.0, delta)
        return self.actual[motor_id]


class DummyLogger:
    def __init__(self):
        self.messages = []

    def info(self, *args):
        self.messages.append(args)


class ScheduledFakeWind:
    """Upstream-shaped fake with asynchronous M0/M2 arrival."""

    def __init__(self):
        self.positions = [0.0, 0.0, 0.0, 0.0]
        self.motor_positions = list(self.positions)
        self.motor_velocities = [10.0, 5.0, 20.0, 5.0]
        self.m0_wind_range = [-20.0, -5.0]
        self.m1_rotating_position = -2.0
        self.teeth_count = 24
        self.simulation = True
        self.logger = DummyLogger()
        self.moves = []
        self.calls = []
        self.m0_polls = 0
        self._delay_initial_target = True
        self._long_m2_target = None
        self._m2_samples = [8.0, 10.0]

    def move_motor(self, motor_id, target):
        target = float(target)
        self.moves.append((motor_id, target))
        self.motor_positions[motor_id] = target
        if motor_id == 2 and abs(target - 10.0) < 1e-9:
            self._long_m2_target = target
        elif (motor_id == 0 and abs(target + 10.0) < 1e-9
              and self._delay_initial_target):
            self._delay_initial_target = False
        else:
            self.positions[motor_id] = target

    def get_motor_position(self, motor_id):
        if motor_id == 0:
            self.m0_polls += 1
            delta = self.motor_positions[0] - self.positions[0]
            if abs(delta) <= 2.0:
                self.positions[0] = self.motor_positions[0]
            else:
                self.positions[0] += math.copysign(2.0, delta)
        elif motor_id == 2 and self._long_m2_target is not None:
            self.positions[2] = (self._m2_samples.pop(0)
                                 if self._m2_samples
                                 else self._long_m2_target)
        return self.positions[motor_id]

    def move_to_teeth(self, teeth_idx):
        self.calls.append(("move_to_teeth", teeth_idx))

    def set_wire_tension(self, value):
        self.calls.append(("set_wire_tension", value))

    def set_motor2_wire_position(self):
        self.calls.append(("set_motor2_wire_position",))

    def get_init_motor2_pos(self):
        return self.get_motor_position(2)

    def get_target_motor2_pos(self, clockwise, wind_idx):
        self.calls.append(("get_target_motor2_pos", clockwise, wind_idx))
        return 10.0 if clockwise else -10.0

    def fast_winding(self, clockwise):
        self.calls.append(("fast_winding", clockwise))
        self.positions[2] = 1.0 if clockwise else -1.0
        self.motor_positions[2] = self.positions[2]

    def prevent_collision(self, clockwise):
        self.calls.append(("prevent_collision", clockwise))


class ReversedScheduledFakeWind(ScheduledFakeWind):
    def get_target_motor2_pos(self, clockwise, wind_idx):
        self.calls.append(("get_target_motor2_pos", clockwise, wind_idx))
        return -10.0


class DelayedWaypointWind(ScheduledFakeWind):
    """M0 never reaches waypoint 1, despite accepting its command."""

    def __init__(self):
        super().__init__()
        self._hold_waypoint_one = False

    def move_motor(self, motor_id, target):
        if motor_id == 0 and abs(float(target) + 11.0) < 1e-9:
            target = float(target)
            self.moves.append((motor_id, target))
            self.motor_positions[motor_id] = target
            self._hold_waypoint_one = True
            return
        super().move_motor(motor_id, target)

    def get_motor_position(self, motor_id):
        if motor_id == 0 and self._hold_waypoint_one:
            self.m0_polls += 1
            return self.positions[motor_id]
        return super().get_motor_position(motor_id)


class OffsetScheduledFakeWind(ScheduledFakeWind):
    """Logical M2 init excludes a one-radian collision offset."""

    def __init__(self):
        super().__init__()
        self.positions[2] = 1.0
        self.motor_positions[2] = 1.0

    def get_init_motor2_pos(self):
        return 0.0


class OffsetSlotPlanFakeWind(ScheduledFakeWind):
    """Real-plan fake whose upstream target stops before closure."""

    def __init__(self):
        super().__init__()
        self.positions[2] = 1.0
        self.motor_positions[2] = 1.0
        self.m0_wind_range = [-61.918, -56.8]
        self.m1_rotating_position = -47.124
        self.config = {
            "job": {
                "radial_winding_span_mm": [
                    14.163900505756052, 20.68,
                ],
            },
        }

    def get_init_motor2_pos(self):
        return 0.0

    def get_target_motor2_pos(self, clockwise, wind_idx):
        self.calls.append(("get_target_motor2_pos", clockwise, wind_idx))
        nominal = 50.0 * 2.0 * math.pi
        return nominal if clockwise else -nominal

    def move_motor(self, motor_id, target):
        target = float(target)
        if motor_id == 2 and abs(target) > 20.0:
            self.moves.append((motor_id, target))
            self.motor_positions[motor_id] = target
            self._long_m2_target = target
            return
        super().move_motor(motor_id, target)

    def get_motor_position(self, motor_id):
        if motor_id == 2 and self._long_m2_target is not None:
            delta = self._long_m2_target - self.positions[2]
            if abs(delta) <= 1.0:
                self.positions[2] = self._long_m2_target
                self._long_m2_target = None
            else:
                self.positions[2] += math.copysign(1.0, delta)
            return self.positions[2]
        return super().get_motor_position(motor_id)


class NegativeOffsetSlotPlanFakeWind(OffsetSlotPlanFakeWind):
    """Flyer is one radian before the next crossing in winding direction."""

    def __init__(self):
        super().__init__()
        self.positions[2] = -1.0
        self.motor_positions[2] = -1.0


class EarlyThresholdWind(ScheduledFakeWind):
    """Expose one sample 0.005 rad before the pi crossing."""

    def __init__(self):
        super().__init__()
        self._pi_target_pending = False
        self._pi_early_returned = False
        self.next_m0_command_phase = None

    def move_motor(self, motor_id, target):
        if (motor_id == 0 and abs(float(target) + 12.0) < 1e-9
                and self.next_m0_command_phase is None):
            self.next_m0_command_phase = self.positions[2]
        super().move_motor(motor_id, target)
        if motor_id == 2 and abs(float(target) - math.pi) < 1e-9:
            self._pi_target_pending = True

    def get_motor_position(self, motor_id):
        if motor_id == 2 and self._pi_target_pending:
            if not self._pi_early_returned:
                self._pi_early_returned = True
                self.positions[2] = math.pi - 0.005
                return self.positions[2]
            self.positions[2] = math.pi
            self._pi_target_pending = False
            return self.positions[2]
        return super().get_motor_position(motor_id)


PACKING_SCHEDULE = {
    "version": 1,
    "passes": [{
        "teeth_idx": 5,
        "wind_idx": 2,
        "clockwise": True,
        "m2_travel_rad": 10.0,
        "waypoints": [
            {"m2_phase_rad": 0.0, "m0_target_rad": -10.0},
            {"m2_phase_rad": math.pi, "m0_target_rad": -11.0},
            {"m2_phase_rad": 2.0 * math.pi, "m0_target_rad": -12.0},
            # Explicit lead-out hold through the actual upstream target.
            {"m2_phase_rad": 10.0, "m0_target_rad": -12.0},
        ],
    }],
}


class ConfiguredScheduledFakeWind(ScheduledFakeWind):
    def __init__(self):
        super().__init__()
        self.config = {"job": {"packing_schedule": PACKING_SCHEDULE}}


class UnauthorizedHardwareWind(ScheduledFakeWind):
    def __init__(self):
        super().__init__()
        self.simulation = False
        self.config = {"job": {"hardware_motion_authorized": False}}


ContractWind = make_contract_wind(FakeWind, FakeModule)
StartupContractWind = make_contract_wind(DelayedStartupWind, FakeModule)
ScheduledContractWind = make_contract_wind(
    ScheduledFakeWind, FakeModule, PACKING_SCHEDULE)
ConfiguredContractWind = make_contract_wind(
    ConfiguredScheduledFakeWind, FakeModule)


class ContractAdapterTests(unittest.TestCase):
    def test_production_constructor_fails_closed_without_route_authorization(self):
        Wind = make_contract_wind(
            UnauthorizedHardwareWind, FakeModule,
            require_packing_plan=True,
        )
        with self.assertRaisesRegex(
                RuntimeError, "hardware motion is not authorized"):
            Wind()

    def test_initial_position_waits_for_slow_m0_arrival(self):
        wind = StartupContractWind()
        wind.init_position(pull_wire=True)
        self.assertEqual(wind.actual[:3], [wind.m0_zero, wind.m1_zero,
                                          wind.m2_zero])
        self.assertGreater(wind.polls[0], 2)

    def test_clockwise_wrap_is_two_negative_turns_from_actual_position(self):
        wind = ContractWind(True, 2.25, Motor2State.TOP_LEFT)
        wind.wind_wire_around_shaft(1)
        self.assertEqual(wind.moves[0], (0, SHAFT_WRAP_M0_PARK_RAD))
        self.assertAlmostEqual(wind.moves[1][1],
                               SHAFT_WRAP_M2_PARK_PHASE_RAD)
        self.assertAlmostEqual(wind.moves[2][1], 2.25 - 4 * math.pi)
        self.assertAlmostEqual(wind.m1_zero, 0.375 - 4 * math.pi)
        self.assertEqual(wind.starts_at, 0)
        self.assertEqual(wind.motor2_pos, Motor2State.TOP)
        self.assertAlmostEqual(wind.m2_zero,
                               SHAFT_WRAP_M2_PARK_PHASE_RAD)
        self.assertEqual(
            [event["phase"] for event in wind.shaft_wrap_phase_events],
            ["prepark_start", "m0_parked", "contact_start", "contact_done"],
        )

    def test_counterclockwise_wrap_is_two_positive_turns_from_actual_position(self):
        wind = ContractWind(False, -1.75, Motor2State.TOP_RIGHT)
        wind.wind_wire_around_shaft(2)
        self.assertEqual(wind.moves[0], (0, SHAFT_WRAP_M0_PARK_RAD))
        self.assertAlmostEqual(wind.moves[1][1],
                               SHAFT_WRAP_M2_PARK_PHASE_RAD)
        self.assertAlmostEqual(wind.moves[2][1], -1.75 + 4 * math.pi)
        self.assertAlmostEqual(wind.m1_zero, 0.375 + 4 * math.pi)
        self.assertAlmostEqual(wind.m2_zero,
                               SHAFT_WRAP_M2_PARK_PHASE_RAD)

    def test_bottom_left_wrap_recenters_to_top_after_complete_closure(self):
        wind = ContractWind(False, -1.75, Motor2State.BOTTOM_LEFT)
        wind.wind_wire_around_shaft(2)
        self.assertAlmostEqual(wind.moves[1][1],
                               SHAFT_WRAP_M2_PARK_PHASE_RAD)
        self.assertEqual(wind.motor2_pos, Motor2State.TOP)
        self.assertAlmostEqual(wind.m2_zero, wind.moves[1][1])

    def test_bottom_right_wrap_recenters_to_top_after_complete_closure(self):
        wind = ContractWind(True, 2.25, Motor2State.BOTTOM_RIGHT)
        wind.wind_wire_around_shaft(1)
        self.assertAlmostEqual(wind.moves[1][1],
                               SHAFT_WRAP_M2_PARK_PHASE_RAD)
        self.assertEqual(wind.motor2_pos, Motor2State.TOP)
        self.assertAlmostEqual(wind.m2_zero, wind.moves[1][1])

    def test_invalid_flyer_state_fails_closed(self):
        wind = ContractWind(False, 0.0, Motor2State.TOP)
        with self.assertRaises(RuntimeError):
            wind.wind_wire_around_shaft(1)
        self.assertEqual(wind.moves, [])

    def test_shaft_wrap_uses_nearest_equivalent_machine_reference_pose(self):
        wind = ContractWind(False, 0.0, Motor2State.TOP_RIGHT)
        wind.positions[2] = 7.3
        wind.motor_positions[2] = 7.3
        wind.wind_wire_around_shaft(2)
        self.assertAlmostEqual(
            wind.moves[1][1],
            SHAFT_WRAP_M2_PARK_PHASE_RAD + 2.0 * math.pi,
        )

    def test_no_schedule_delegates_to_upstream_wind_wire(self):
        wind = ContractWind(False, 0.0, Motor2State.TOP_LEFT)
        result = wind.wind_wire(7, True, 3)
        self.assertEqual(result, "upstream")
        self.assertEqual(wind.upstream_wind_calls, [(7, True, 3)])

    def test_schedule_commands_waypoints_in_order_then_exact_parking(self):
        wind = ScheduledContractWind()
        wind.wind_wire(5, True, 2)

        m0_targets = [target for motor, target in wind.moves if motor == 0]
        self.assertEqual(
            m0_targets,
            [-5.0, -10.0, -11.0, -12.0, -12.0, -2.0],
        )
        self.assertGreater(wind.m0_polls, 2)
        self.assertEqual(wind.positions[2], 10.0)
        self.assertIn(("set_wire_tension", 1), wind.calls)
        self.assertIn(("prevent_collision", True), wind.calls)
        self.assertEqual(wind._packing_pass_cursor, 1)

        self.assertEqual(
            [event["waypoint_index"]
             for event in wind.packing_waypoint_events],
            [0, 1, 2, 3],
        )
        self.assertEqual(
            [event["m0_target_rad"]
             for event in wind.packing_waypoint_events],
            [-10.0, -11.0, -12.0, -12.0],
        )
        observed_phases = [
            event["observed_m2_phase_rad"]
            for event in wind.packing_waypoint_events
        ]
        for observed, expected in zip(
                observed_phases, [0.0, math.pi, 2.0 * math.pi, 10.0]):
            self.assertAlmostEqual(observed, expected)
        for event in wind.packing_waypoint_events:
            self.assertAlmostEqual(
                event["observed_m2_phase_rad"], event["m2_phase_rad"])
            self.assertAlmostEqual(
                event["observed_m0_rad"], event["m0_target_rad"])
        self.assertNotIn(("fast_winding", True), wind.calls)
        self.assertTrue(all(
            abs(event["m0_error_rad"])
            <= PACKING_M0_SETTLE_TOLERANCE_RAD
            for event in wind.packing_waypoint_events
        ))

    def test_crossing_is_never_accepted_early(self):
        Wind = make_contract_wind(
            EarlyThresholdWind, FakeModule, PACKING_SCHEDULE)
        wind = Wind()
        wind.wind_wire(5, True, 2)
        self.assertIsNotNone(wind.next_m0_command_phase)
        self.assertGreaterEqual(wind.next_m0_command_phase, math.pi - 1e-9)
        crossing = wind.packing_waypoint_events[1]
        self.assertGreaterEqual(
            crossing["observed_m2_phase_rad"],
            crossing["m2_phase_rad"] - 1e-9,
        )

        # Each M0 target is issued before the M2 command that reaches its
        # required phase, including both first-turn crossings and lead-out.
        move_pairs = wind.moves
        for m0_target, m2_phase in (
                (-11.0, math.pi),
                (-12.0, 2.0 * math.pi)):
            m0_command = next(
                index for index, move in enumerate(move_pairs)
                if move == (0, m0_target))
            m2_crossing = next(
                index for index, (motor, target) in enumerate(move_pairs)
                if motor == 2 and target >= m2_phase - 1e-9)
            self.assertLess(m0_command, m2_crossing)

    def test_inline_job_schedule_loads_without_factory_argument(self):
        wind = ConfiguredContractWind()
        self.assertIsNotNone(wind._packing_schedule)
        wind.wind_wire(5, True, 2)
        self.assertEqual(wind._packing_pass_cursor, 1)
        self.assertEqual(
            [event["waypoint_index"]
             for event in wind.packing_waypoint_events],
            [0, 1, 2, 3],
        )

    def test_delayed_m0_fails_closed_at_first_turn_crossing(self):
        Wind = make_contract_wind(
            DelayedWaypointWind, FakeModule, PACKING_SCHEDULE)
        wind = Wind()
        with self.assertRaisesRegex(
                RuntimeError, "before M0 was observed settled"):
            wind.wind_wire(5, True, 2)

        self.assertEqual(
            [event["waypoint_index"]
             for event in wind.packing_waypoint_events],
            [0],
        )
        self.assertNotIn((0, -12.0), wind.moves)
        self.assertFalse(any(
            motor == 2 and abs(target - 10.0) < 1e-9
            for motor, target in wind.moves
        ))
        m2_moves = [target for motor, target in wind.moves if motor == 2]
        self.assertAlmostEqual(m2_moves[-1], math.pi)

    def test_first_fast_step_uses_actual_commanded_pose_with_offset(self):
        Wind = make_contract_wind(
            OffsetScheduledFakeWind, FakeModule, PACKING_SCHEDULE)
        wind = Wind()
        wind.wind_wire(5, True, 2)

        first_m2_target = next(
            target for motor, target in wind.moves if motor == 2)
        self.assertAlmostEqual(first_m2_target, 1.0 + math.pi / 6.0)
        self.assertGreater(first_m2_target, 1.0)

    def test_offset_slot_pass_physically_reaches_one_unique_closure(self):
        Wind = make_contract_wind(OffsetSlotPlanFakeWind, FakeModule)
        wind = Wind()
        wind._slot_winding_plan = load_slot_winding_plan(PLAN_PATH)
        wind.wind_wire(5, True, 2)

        closure_phase = math.pi + 50.0 * 2.0 * math.pi
        pass_events = [
            event for event in wind.packing_waypoint_events
            if event["pass_index"] == 0
        ]
        centers = [event for event in pass_events
                   if event["kind"] == "placement_center"]
        closures = [
            event for event in pass_events
            if event["kind"] == "final_hold"
            and abs(event["m2_phase_rad"] - closure_phase) <= 1e-9
        ]
        self.assertEqual(len(centers), 100)
        self.assertEqual(len(closures), 1)
        self.assertGreaterEqual(
            closures[0]["observed_m2_phase_rad"], closure_phase - 1e-9)
        self.assertTrue(all(
            event["observed_m2_phase_rad"] + 1e-9
            >= event["m2_phase_rad"]
            for event in pass_events
        ))
        self.assertAlmostEqual(wind.positions[2], closure_phase)
        self.assertAlmostEqual(
            wind.packing_pass_origin_events[0]["actual_travel_rad"],
            closure_phase)

    def test_negative_collision_offset_uses_preceding_crossing_origin(self):
        Wind = make_contract_wind(NegativeOffsetSlotPlanFakeWind, FakeModule)
        wind = Wind()
        wind._slot_winding_plan = load_slot_winding_plan(PLAN_PATH)
        wind.wind_wire(5, True, 2)

        nominal = 50.0 * 2.0 * math.pi
        closure_phase = math.pi + nominal
        origin = wind.packing_pass_origin_events[0]
        closures = [
            event for event in wind.packing_waypoint_events
            if event["kind"] == "final_hold"
            and abs(event["m2_phase_rad"] - closure_phase) <= 1e-9
        ]
        self.assertAlmostEqual(origin["start_phase_rad"], math.pi - 1.0)
        self.assertAlmostEqual(origin["first_crossing_phase_rad"], math.pi)
        self.assertAlmostEqual(origin["actual_travel_rad"], closure_phase)
        self.assertEqual(len(closures), 1)
        self.assertGreaterEqual(
            closures[0]["observed_m2_phase_rad"], closure_phase - 1e-9)
        self.assertTrue(all(
            event["observed_m2_phase_rad"] + 1e-9
            >= event["m2_phase_rad"]
            for event in wind.packing_waypoint_events
        ))
        self.assertAlmostEqual(wind.positions[2], nominal)

    def test_skipped_waypoint_fails_instead_of_late_drain(self):
        skipped = {
            "version": 1,
            "passes": [{
                "teeth_idx": 5, "wind_idx": 2, "clockwise": True,
                "m2_travel_rad": 10.0,
                "waypoints": [
                    {"m2_phase_rad": 0.0, "m0_target_rad": -10.0},
                    {"m2_phase_rad": math.pi, "m0_target_rad": -11.0},
                    {"m2_phase_rad": 2.0 * math.pi,
                     "m0_target_rad": -12.0},
                    {"m2_phase_rad": 7.0, "m0_target_rad": -12.0},
                    {"m2_phase_rad": 10.0, "m0_target_rad": -12.0},
                ],
            }],
        }
        Wind = make_contract_wind(ScheduledFakeWind, FakeModule, skipped)
        wind = Wind()
        wind._m2_samples = [10.0]
        with self.assertRaisesRegex(RuntimeError, "skipped waypoint 3"):
            wind.wind_wire(5, True, 2)
        self.assertEqual(
            [event["waypoint_index"]
             for event in wind.packing_waypoint_events],
            [0, 1, 2],
        )

    def test_impossible_m0_settling_time_fails_before_m2_motion(self):
        too_fast = {
            "version": 1,
            "passes": [{
                "teeth_idx": 5, "wind_idx": 2, "clockwise": True,
                "m2_travel_rad": 10.0,
                "waypoints": [
                    {"m2_phase_rad": 0.0, "m0_target_rad": -5.0},
                    {"m2_phase_rad": 0.1, "m0_target_rad": -20.0},
                    {"m2_phase_rad": 10.0, "m0_target_rad": -20.0},
                ],
            }],
        }
        Wind = make_contract_wind(ScheduledFakeWind, FakeModule, too_fast)
        wind = Wind()
        with self.assertRaisesRegex(RuntimeError, "cannot settle M0"):
            wind.wind_wire(5, True, 2)
        self.assertNotIn(("fast_winding", True), wind.calls)
        self.assertFalse(any(motor == 2 for motor, _ in wind.moves))

    def test_schedule_identity_mismatch_fails_before_any_motion(self):
        wind = ScheduledContractWind()
        with self.assertRaisesRegex(RuntimeError, "identity mismatch"):
            wind.wind_wire(6, True, 2)
        self.assertEqual(wind.moves, [])
        self.assertEqual(wind.packing_waypoint_events, [])

    def test_invalid_schedule_rejects_missing_lead_out_endpoint(self):
        invalid = {
            "version": 1,
            "passes": [{
                "teeth_idx": 0, "wind_idx": 0, "clockwise": True,
                "m2_travel_rad": 10.0,
                "waypoints": [
                    {"m2_phase_rad": 0.0, "m0_target_rad": -10.0},
                    {"m2_phase_rad": 9.0, "m0_target_rad": -11.0},
                ],
            }],
        }
        with self.assertRaisesRegex(ValueError, "final waypoint phase"):
            normalize_packing_schedule(invalid)

    def test_out_of_range_m0_target_fails_before_motion(self):
        invalid_for_machine = {
            "version": 1,
            "passes": [{
                "teeth_idx": 5, "wind_idx": 2, "clockwise": True,
                "m2_travel_rad": 10.0,
                "waypoints": [
                    {"m2_phase_rad": 0.0, "m0_target_rad": -21.0},
                    {"m2_phase_rad": 10.0, "m0_target_rad": -12.0},
                ],
            }],
        }
        Wind = make_contract_wind(
            ScheduledFakeWind, FakeModule, invalid_for_machine)
        wind = Wind()
        with self.assertRaisesRegex(RuntimeError, "outside winding range"):
            wind.wind_wire(5, True, 2)
        self.assertEqual(wind.moves, [])

    def test_actual_upstream_travel_mismatch_blocks_flyer_motion(self):
        wrong_travel = {
            "version": 1,
            "passes": [{
                "teeth_idx": 5, "wind_idx": 2, "clockwise": True,
                "m2_travel_rad": 11.0,
                "waypoints": [
                    {"m2_phase_rad": 0.0, "m0_target_rad": -10.0},
                    {"m2_phase_rad": 11.0, "m0_target_rad": -12.0},
                ],
            }],
        }
        Wind = make_contract_wind(
            ScheduledFakeWind, FakeModule, wrong_travel)
        wind = Wind()
        with self.assertRaisesRegex(RuntimeError, "M2 travel mismatch"):
            wind.wind_wire(5, True, 2)
        self.assertNotIn(("fast_winding", True), wind.calls)
        self.assertFalse(any(motor == 2 for motor, _ in wind.moves))

    def test_actual_upstream_direction_mismatch_blocks_flyer_motion(self):
        Wind = make_contract_wind(
            ReversedScheduledFakeWind, FakeModule, PACKING_SCHEDULE)
        wind = Wind()
        with self.assertRaisesRegex(RuntimeError, "M2 direction mismatch"):
            wind.wind_wire(5, True, 2)
        self.assertNotIn(("fast_winding", True), wind.calls)
        self.assertFalse(any(motor == 2 for motor, _ in wind.moves))


if __name__ == "__main__":
    unittest.main(verbosity=2)
