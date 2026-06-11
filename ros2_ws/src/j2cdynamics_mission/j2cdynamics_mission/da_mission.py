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
TIMEOUT_ZONE_3   = 200.0

ZONE_3_TOTAL_DUPLOS = 6
MAX_CAPACITY        = 5     # interrupt collection when tank hits this


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
        # Collapsed from 9 → 4. The fill/dropoff loop lives inside
        # COLLECT_ZONE_3 so it can iterate naturally.
        return [
            ('SETUP',          self._step_setup),
            ('OPEN_DOOR',      self._step_open_door),
            ('COLLECT_ZONE_3', self._step_collect_zone_3),
            ('FINAL_DROPOFF',  self._step_dropoff),
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

        # If we started carrying duplos (RESET mid-mission), dump them first
        # so we don't press the button with a full hopper.
        if self.get_duplo_count() > 0:
            self.get_logger().info(
                f'starting with {self.get_duplo_count()} duplos — dumping first')
            self._step_dropoff()

    def _step_open_door(self) -> None:
        """Press the button — but only the first time. Subsequent entries to
        zone 3 (after a fill-up dropoff) skip this entirely."""
        if self._door_opened:
            self.get_logger().info('door already open — skipping button press')
            return

        self.set_sweeper_mode(SweeperMode.COLLECT)
        # Drive to the button. If we somehow already have duplos, dump them.
        while not self._go_to_with_recovery(
                BUTTON_APPROACH, label='BUTTON_APPROACH', precise=True):
            if self.get_duplo_count() > 0:
                self._step_dropoff()
            else:
                self.go_to(self.BASE_POSE, precise=False, timeout_s=20)

        self.set_sweeper_mode(SweeperMode.IDLE)
        if not self.push_button_and_wait_for_door():
            self.get_logger().error('Aborting — could not open door')
            raise MissionAbortException()

        self._door_opened = True
        self.get_logger().info('door opened — flag set for the rest of the mission')

    def _step_collect_zone_3(self) -> None:
        """Main objective loop:
          (a) traverse the door into zone 3
          (b) collect until tank hits MAX_CAPACITY or zone is exhausted
          (c) if zone exhausted → done
          (d) else → return to button approach, dropoff, go back to (a)

        Note that (a) does NOT re-push the button — _door_opened gated that.
        """
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

        # Stash hit in a mutable so the closure can flip it.
        stopped_for_full = [False]
        def stop_condition() -> bool:
            if self.get_duplo_count() >= MAX_CAPACITY:
                stopped_for_full[0] = True
                return True
            return False

        # NOTE: this requires explore_zone() in mission_duplo.py to accept a
        # `stop_condition` callable that's polled between waypoints (and ideally
        # during visual-servo handoffs). See the note below the file.
        self.explore_zone(WAYPOINTS_ZONE_3, TIMEOUT_ZONE_3, label='ZONE_3')

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