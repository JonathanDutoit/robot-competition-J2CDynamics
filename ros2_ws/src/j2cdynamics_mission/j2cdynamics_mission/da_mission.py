"""
Duplo-Aspiration mission — zone-3 focused.

Objective: collect all 6 duplos in zone 3. Robot carries max 5, so expect at
least one mid-mission dropoff and re-entry through the (already open) door.
"""

import time
from enum import IntEnum

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from j2cdynamics_mission.mission_base import (
    MissionBase, MissionAbortException, make_pose,
)
from j2cdynamics_mission.mission_duplo import DuploMixin
from j2cdynamics_mission.mission_button import ButtonMixin, DOOR_PROBE_POSE


class SweeperMode(IntEnum):
    IDLE    = 0
    COLLECT = 1
    DROPOFF = 2
    FAULT   = 3


_SWEEPER_MODE_JOINT  = 'sweeper_mode'
_DUPLO_COUNTER_JOINT = 'duplo_counter'


# ──────────────────────────────────────────────────────────────────────────────
#  POSES & MISSION CONSTANTS
# ──────────────────────────────────────────────────────────────────────────────

BASE_POSE             = (0.45, 0.45, 1.57)
DROPFF_FIRST_WAYPOINT = (1.0, 0.45, 1.57)
START_POSE            = (0.557, 0.626, 1.57)
BUTTON_APPROACH       = (4.45, 7.40, 1.57)

WAYPOINTS_ZONE_3 = '/maps/arena/waypoints_zone3.yaml'
WAYPOINTS_ZONE_1 = '/maps/arena/waypoints_zone1_da.yaml'


TIMEOUT_ZONE_3   = 240.0
TIMEOUT_ZONE_1   = 200.0

ZONE_3_TOTAL_DUPLOS = 6
MAX_CAPACITY        = 5     # interrupt collection when tank hits this

DOOR_MAX_ATTEMPTS  = 5       # full (approach + press) cycles before giving up
DOOR_TIME_BUDGET_S = 150.0   # cap on total time in _step_open_door


# ──────────────────────────────────────────────────────────────────────────────
#  MISSION RUNNER
# ──────────────────────────────────────────────────────────────────────────────

class DaMissionRunner(DuploMixin, ButtonMixin, MissionBase):
    BASE_POSE             = BASE_POSE
    DROPFF_FIRST_WAYPOINT = DROPFF_FIRST_WAYPOINT
    START_POSE            = START_POSE
    DROPOFF_SPEED         = -0.3
    DROPOFF_TIME          = 3.0

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._sweeper_mode = None
        self._duplo_count  = 0

        # Persistent mission state — survives RESET (instance lives across resets).
        self._door_opened     = False
        self._zone3_remaining = ZONE_3_TOTAL_DUPLOS

        self._sweeper_pub = self.create_publisher(
            Float64MultiArray, '/sweeper_controller/commands', 10)
        self._duplo_count_pub = self.create_publisher(
            Float64MultiArray, '/duplo_counter_controller/commands', 10)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10)

    # ── Sweeper FSM + counter (unchanged) ────────────────────────────────────

    def _on_joint_states(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name == _SWEEPER_MODE_JOINT:
                try:
                    self._sweeper_mode = SweeperMode(int(round(pos)))
                except ValueError:
                    self._sweeper_mode = None
            elif name == _DUPLO_COUNTER_JOINT:
                self._duplo_count = int(round(pos))

    def set_sweeper_mode(self, mode: SweeperMode) -> None:
        self._sweeper_pub.publish(Float64MultiArray(data=[float(int(mode))]))
        self.get_logger().info(f'sweeper → {mode.name}')

    def request_duplo_count_refresh(self) -> None:
        self._duplo_count_pub.publish(Float64MultiArray(data=[1.0]))

    def get_sweeper_mode(self):
        return self._sweeper_mode

    def get_duplo_count(self) -> int:
        return self._duplo_count

    @property
    def zone3_remaining(self) -> int:
        return self._zone3_remaining

    # ── Step sequence ────────────────────────────────────────────────────────

    def _build_steps(self):
        return [
            ('SETUP',          self._step_setup),
            ('OPEN_DOOR',      self._step_open_door),
            ('COLLECT_ZONE_3', self._step_collect_zone_3),
            ('DROPOFF',  self._step_dropoff),
            ('COLLECT_ZONE_1', self._step_collect_zone_1),
            ('DROPOFF', self._step_dropoff)
        ]

    # ── Step implementations ─────────────────────────────────────────────────

    def _step_setup(self) -> None:
        self.setInitialPose(make_pose(*self.START_POSE))
        self.waitUntilNav2Active(localizer='amcl')
        self.get_logger().info('Nav2 active — waiting for AMCL pose')
        t_deadline = time.time() + 5.0
        while time.time() < t_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if getattr(self, '_amcl_pose', None) is not None:
                break
        time.sleep(0.5)
        self._enable_collection(False)
        self.set_sweeper_mode(SweeperMode.IDLE)

    def _step_open_door(self) -> None:
        """Critical: open the door.Retries within a time budget, escalating
        recovery between attempts.
        Never raises except when abort_event fires (supervisor timeout / RESET).
        On exhaustion, logs loud and returns; _step_collect_zone_3 will no-op and
        the mission completes the FINAL_DROPOFF gracefully instead of crashing."""
        if self._door_opened:
            self.get_logger().info('door already open — skipping button press')
            return

        t_start = time.time()

        for attempt in range(1, DOOR_MAX_ATTEMPTS + 1):
            # ONLY abort_event short-circuits us — supervisor (time-up) or RESET.
            if self.abort_event.is_set():
                raise MissionAbortException()
            if time.time() - t_start > DOOR_TIME_BUDGET_S:
                self.get_logger().error(
                    f'door budget exceeded ({DOOR_TIME_BUDGET_S:.0f}s) after '
                    f'{attempt - 1} attempts; continuing without door')
                return

            self.get_logger().info(
                f'═══ Open door, attempt {attempt}/{DOOR_MAX_ATTEMPTS} ═══')

            # Phase A — reach button (escalating strategies inside).
            if not self._reach_button_for_press():
                self.get_logger().warn(
                    f'door attempt {attempt}: could not reach button — recovery + retry')
                self._run_recovery(f'open-door-reach-r{attempt}')
                continue

            # press cycle (internal 3-retry probe is in ButtonMixin).
            # Sweeper IDLE so it doesn't drive into the button while we back into it.
            self.set_sweeper_mode(SweeperMode.IDLE)
            if self.push_button_and_wait_for_door():
                self._door_opened = True
                self.get_logger().info(f'door opened   (attempt {attempt}, 'f'{time.time() - t_start:.0f}s elapsed)')
                return

            
            self.get_logger().warn(f'door attempt {attempt}: press cycle failed — recovery + re-approach')
            self._run_recovery(f'open-door-press-r{attempt}')

        self.get_logger().error(
            f'COULD NOT OPEN DOOR after {DOOR_MAX_ATTEMPTS} attempts / '
            f'{time.time() - t_start:.0f}s — zone 3 will be skipped. '
            f'Operator: RESET to retry from this step.')

    def _reach_button_for_press(self) -> bool:
        """Drive to BUTTON_APPROACH with an escalating ladder of strategies.

        Rung 1: precise heading, full 4-attempt recovery cascade.
        Rung 2: if hopper near full, drop off then re-approach.
        Rung 3: looser tolerance (general_goal_checker). The button press is
                forgiving — we don't NEED 5cm precision to reach it.
        Returns True on first success."""

        self.set_sweeper_mode(SweeperMode.COLLECT)
        if self._go_to_with_recovery(BUTTON_APPROACH, label='BUTTON_APPROACH', precise=True):
            return True

        if self.get_duplo_count() >= MAX_CAPACITY - 1:
            self.get_logger().warn(
                f'full tank ({self.get_duplo_count()}) — dumping before re-approach')
            self._step_dropoff()
            if self._go_to_with_recovery(
                    BUTTON_APPROACH, label='BUTTON_APPROACH-postdump', precise=True):
                return True

        # Rung 3 — looser tolerance. The button-press dance has its own backoff +
        # alignment; we don't need a precise nav goal here.
        self.get_logger().warn('precise reach failed — trying with loose tolerance')
        if self._go_to_with_recovery(
                BUTTON_APPROACH, label='BUTTON_APPROACH-loose', precise=False):
            return True

        return False

    def _step_collect_zone_3(self) -> None:
        """Main objective loop:
          (a) traverse the door into zone 3
          (b) collect until tank hits MAX_CAPACITY or zone is exhausted
          (c) if zone exhausted → done
          (d) else → return to button approach, dropoff, go back to (a)

        Note that (a) does NOT re-push the button — _door_opened gated that.
        """
        if not self._door_opened:
            self.get_logger().warn('door never opened — skipping zone 3 collection')
            return
        while self._zone3_remaining > 0:
            # (a) Door traverse — door is already open, this is just a Nav2 goal.
            if not self._go_to_with_recovery(
                    DOOR_PROBE_POSE, label='DOOR_TRAVERSE', precise=True):
                self.get_logger().error('could not traverse door')
                raise MissionAbortException()

            # (b) Collect with mid-zone tank-full interrupt.
            stopped_for_full = self._collect_zone_3_pass()

            # (c) Zone exhausted (  finished its waypoint list).
            if not stopped_for_full:
                self.get_logger().info(
                    f'zone 3 exploration complete; assuming clear '
                    f'(belief: {self._zone3_remaining} remaining)')
                self._zone3_remaining = 0
                break

            if self._zone3_remaining == 0:
                self.get_logger().info('zone 3 cleared!')
                break

            # (d) Tank full but duplos remain — return and dump.
            self.get_logger().info(
                f'tank full ({self.get_duplo_count()}), '
                f'{self._zone3_remaining} duplos still in zone 3 — returning to dropoff')
            self.set_sweeper_mode(SweeperMode.COLLECT)  # opportunistic en route
            self.go_to(BUTTON_APPROACH, precise=True)
            self._step_dropoff()
            # loop continues; OPEN_DOOR is NOT re-run since we're inside one step

    def _collect_zone_3_pass(self) -> bool:
        """One pass through zone 3 waypoints. Returns True if we stopped because
        the tank hit MAX_CAPACITY, False if explore_zone completed its waypoint
        list (i.e. zone is effectively exhausted)."""
        self.set_sweeper_mode(SweeperMode.COLLECT)
        count_before = self.get_duplo_count()

        stopped_for_full = [False]
        stop_condition = self._make_full_stop(stopped_for_full)

        self.explore_zone(WAYPOINTS_ZONE_3, TIMEOUT_ZONE_3, label='ZONE_3',
                          stop_condition=stop_condition)

        picked_up = max(0, self.get_duplo_count() - count_before)
        picked_up = min(picked_up, self._zone3_remaining)
        self._zone3_remaining -= picked_up

        self.get_logger().info(
            f'zone 3 pass: picked up {picked_up}, '
            f'{self._zone3_remaining}/{ZONE_3_TOTAL_DUPLOS} remaining '
            f'(tank: {self.get_duplo_count()}/{MAX_CAPACITY}, '
            f'stopped_for_full={stopped_for_full[0]})')
        return stopped_for_full[0]

    def _make_full_stop(self, stopped_box):
        """Build a stop_condition closure for explore_zone that:
          • forces a firmware count refresh before reading (the joint_state value
            may lag — without a refresh we can trip on a stale read),
          • logs LOUD when it fires so the trigger is never 'random' in the logs,
          • sets stopped_box[0]=True so the caller can branch on the outcome.
        stopped_box is a mutable list so the closure can mutate from outside scope."""
        def _stop():
            self.request_duplo_count_refresh()
            # Let the JointState callback receive the refreshed value before reading.
            rclpy.spin_once(self, timeout_sec=0.05)
            n = self.get_duplo_count()
            if n >= MAX_CAPACITY:
                self.get_logger().warn(
                    f'TANK FULL ({n}/{MAX_CAPACITY}) — interrupting zone for dropoff')
                stopped_box[0] = True
                return True
            return False
        return _stop

    def _step_collect_zone_1(self) -> None:
        self.set_sweeper_mode(SweeperMode.COLLECT)
        count_before = self.get_duplo_count()

        stopped_for_full = [False]
        condition = self._make_full_stop(stopped_for_full)

        self.explore_zone(WAYPOINTS_ZONE_1, TIMEOUT_ZONE_1, label='ZONE_3',
                          stop_condition=condition)

        picked_up = max(0, self.get_duplo_count() - count_before)
        # Clamp in case opportunistic pickups en route inflated the count.
        picked_up = min(picked_up, self._zone3_remaining)
        self._zone3_remaining -= picked_up

        self.get_logger().info(
            f'zone 3 pass: picked up {picked_up}, '
            f'{self._zone3_remaining}/{ZONE_3_TOTAL_DUPLOS} remaining '
            f'(tank: {self.get_duplo_count()}/{MAX_CAPACITY}, '
            f'stopped_for_full={stopped_for_full[0]})')
        return stopped_for_full[0]

    def _step_dropoff(self) -> None:
        n_before = self.get_duplo_count()
        self.set_sweeper_mode(SweeperMode.DROPOFF)
        self.dropoff()
        self.set_sweeper_mode(SweeperMode.IDLE)
        n_after = self.get_duplo_count()
        msg = f'dropoff complete: count {n_before} → {n_after}'
        if n_before > 0 and n_after == n_before:
            msg += ' (no change — check hardware!)'
        self.get_logger().info(msg)


def main() -> None:
    rclpy.init()
    runner = DaMissionRunner()
    try:
        runner.main_loop()
    except KeyboardInterrupt:
        runner.get_logger().info('Interrupted')
    finally:
        runner._ramp_pub.publish(Twist())
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()