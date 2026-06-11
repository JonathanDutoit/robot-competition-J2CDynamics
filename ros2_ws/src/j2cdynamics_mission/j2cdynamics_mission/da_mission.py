"""
Duplo-Aspiration mission — step sequence + DA-only hardware (sweeper).

Shared infrastructure lives in mixin/base files:
  mission_base.py    — nav primitives, recovery, dropoff, /mission_command lifecycle
  mission_duplo.py   — visual-servo handoff, sweep, opportunistic collect, explore_zone
  mission_button.py  — button press + door probe

DA-specific (kept inline below since the sweeper is on this robot only):
  • SweeperMode enum
  • _on_joint_states / set_sweeper_mode / get_duplo_count / etc.
  • _step_dropoff wrapper that toggles sweeper mode around MissionBase.dropoff()

Sequence:
    1.  SETUP                 — initial pose + Nav2, sweeper to IDLE
    2.  DROPOFF_IF_CARRYING   — if duplo_counter > 0 (e.g. RESET mid-mission), dump first
    3.  BUTTON_APPROACH       — Nav2 to button approach pose (critical, precise)
    4.  PUSH_BUTTON           — open-loop press, retry-and-probe for door open
    5.  DOOR_TRAVERSE         — Nav2 through the door (critical, precise)
    6.  ZONE_3                — visual-servo collection behind the door
    7.  RETURN_TO_BTN         — Nav2 back to button approach pose
    8.  DROPOFF_1             — sweeper DROPOFF + reverse + IDLE
    9.  ZONE_1                — visual-servo collection in the starting arena
    10. DROPOFF_2             — final dropoff
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


# ── Sweeper FSM (mirrors enum in j2cdynamics_driver/src/joy_mode_mapper.cpp) ──
class SweeperMode(IntEnum):
    IDLE    = 0
    COLLECT = 1
    DROPOFF = 2
    FAULT   = 3


# Joint names exposed by the Arduino-bridged ros2_control hardware interface
# (see da_description/urdf/da/gazebo_ros2_control.xacro).
_SWEEPER_MODE_JOINT  = 'sweeper_mode'
_DUPLO_COUNTER_JOINT = 'duplo_counter'


# ──────────────────────────────────────────────────────────────────────────────
#  POSES & PATHS  (Duplo-Aspiration specific)
# ──────────────────────────────────────────────────────────────────────────────

BASE_POSE             = (0.45, 0.45, 1.57)
DROPFF_FIRST_WAYPOINT = (1.0, 0.45, 1.57)   # rough alignment waypoint
START_POSE            = (0.557, 0.626, 1.57)
# START_POSE          = (4.255, 5.228, 1.50)  # alt: start mid-arena for debugging

BUTTON_APPROACH = (4.45, 7.40, 1.57)   # Nav2 stops here, precise heading

WAYPOINTS_ZONE_3 = '/maps/arena/waypoints_zone3.yaml'
WAYPOINTS_ZONE_1 = '/maps/arena/waypoints_zone1_da.yaml'

TIMEOUT_ZONE_3 = 200.0
TIMEOUT_ZONE_1 = 160.0


# ──────────────────────────────────────────────────────────────────────────────
#  MISSION RUNNER
# ──────────────────────────────────────────────────────────────────────────────

class DaMissionRunner(DuploMixin, ButtonMixin, MissionBase):
    """DA mission = visual-servo collection behind a button-controlled door."""

    # Class attributes consumed by MissionBase.dropoff() and main_loop():
    BASE_POSE             = BASE_POSE
    DROPFF_FIRST_WAYPOINT = DROPFF_FIRST_WAYPOINT
    START_POSE            = START_POSE
    DROPOFF_SPEED         = -0.3
    DROPOFF_TIME          = 3.0

    # ── Init: cooperative chain + DA-specific sweeper setup ─────────────────

    def __init__(self, *args, **kwargs):
        # MUST forward via super() so DuploMixin / ButtonMixin / MissionBase
        # __init__ chain runs and creates their pubs/subs.
        super().__init__(*args, **kwargs)

        # Sweeper hardware state (None until first /joint_states arrives).
        self._sweeper_mode = None    # SweeperMode | None
        self._duplo_count  = 0       # int

        self._sweeper_pub = self.create_publisher(
            Float64MultiArray, '/sweeper_controller/commands', 10)
        self._duplo_count_pub = self.create_publisher(
            Float64MultiArray, '/duplo_counter_controller/commands', 10)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10)

    # ── Sweeper FSM + counter (DA hardware only) ─────────────────────────────

    def _on_joint_states(self, msg: JointState) -> None:
        """Pull our two joints by name. /joint_states arrives at ~50 Hz from
        ros2_control; mutating self.* here is safe (single executor)."""
        for name, pos in zip(msg.name, msg.position):
            if name == _SWEEPER_MODE_JOINT:
                try:
                    self._sweeper_mode = SweeperMode(int(round(pos)))
                except ValueError:
                    self._sweeper_mode = None
            elif name == _DUPLO_COUNTER_JOINT:
                self._duplo_count = int(round(pos))

    def set_sweeper_mode(self, mode: SweeperMode) -> None:
        """Publish a new FSM target to the Arduino-bridged controller.
        Idempotent — duplicate commands are echoed back via /joint_states either way."""
        self._sweeper_pub.publish(Float64MultiArray(data=[float(int(mode))]))
        self.get_logger().info(f'sweeper → {mode.name}')

    def request_duplo_count_refresh(self) -> None:
        """Trigger the firmware to refresh the duplo counter (see joy_mode_mapper.cpp)."""
        self._duplo_count_pub.publish(Float64MultiArray(data=[1.0]))

    def get_sweeper_mode(self):
        return self._sweeper_mode

    def get_duplo_count(self) -> int:
        return self._duplo_count

    # ── Mission step sequence ────────────────────────────────────────────────

    def _build_steps(self):
        """Step list driven by MissionBase.run(). On RESET, the same step is
        re-tried; completed steps keep their _next_step advance."""
        return [
            ('SETUP',               self._step_setup),
            ('DROPOFF_IF_CARRYING', self._step_dropoff_if_carrying),
            ('BUTTON_APPROACH',     self._step_button_approach),
            ('PUSH_BUTTON',         self._step_push_button),
            ('DOOR_TRAVERSE',       self._step_door_traverse),
            ('ZONE_3',              self._step_zone_3),
            ('RETURN_TO_BTN',       self._step_return_to_button),
            ('DROPOFF_1',           self._step_dropoff),
            ('ZONE_1',              self._step_zone_1),
            ('DROPOFF_2',           self._step_dropoff),
        ]

    # ── Per-step helpers ─────────────────────────────────────────────────────

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

    def _step_dropoff_if_carrying(self) -> None:
        """No-op if empty. If RESET fired mid-mission (or the robot was started
        with duplos already loaded), dump them before the button-press dance so
        we don't try to press the button with a full hopper."""
        n = self.get_duplo_count()
        if n > 0:
            self.get_logger().info(f'carrying {n} duplos at startup — dropping off first')
            self._step_dropoff()
        else:
            self.get_logger().info('starting empty — proceeding')

    def _step_button_approach(self) -> None:
        # COLLECT en route so the sweeper grabs anything spotted on the way.
        self.set_sweeper_mode(SweeperMode.COLLECT)
        if not self._go_to_with_recovery(
                BUTTON_APPROACH, label='BUTTON_APPROACH', precise=True):
            self.get_logger().error('Aborting — could not reach button approach')
            raise MissionAbortException()

    def _step_push_button(self) -> None:
        # IDLE during the button press — don't want the sweeper running
        # backwards into the button.
        self.set_sweeper_mode(SweeperMode.IDLE)
        if not self.push_button_and_wait_for_door():
            self.get_logger().error('Aborting — could not open door')
            raise MissionAbortException()

    def _step_door_traverse(self) -> None:
        if not self._go_to_with_recovery(
                DOOR_PROBE_POSE, label='DOOR_TRAVERSE', precise=True):
            self.get_logger().error('Aborting — could not traverse door')
            raise MissionAbortException()

    def _step_zone_3(self) -> None:
        self.set_sweeper_mode(SweeperMode.COLLECT)
        self.explore_zone(WAYPOINTS_ZONE_3, TIMEOUT_ZONE_3, label='ZONE_3')
        self.get_logger().info(f'count after ZONE_3: {self.get_duplo_count()}')

    def _step_return_to_button(self) -> None:
        # Stay in COLLECT during the return — opportunistic pickups en route.
        self.set_sweeper_mode(SweeperMode.COLLECT)
        self.go_to(BUTTON_APPROACH, precise=True)

    def _step_zone_1(self) -> None:
        self.set_sweeper_mode(SweeperMode.COLLECT)
        self.explore_zone(WAYPOINTS_ZONE_1, TIMEOUT_ZONE_1, label='ZONE_1')
        self.get_logger().info(f'count after ZONE_1: {self.get_duplo_count()}')

    def _step_dropoff(self) -> None:
        """The canonical dropoff for this mission: nav to base, toggle sweeper
        into DROPOFF for the reverse motion, then back to IDLE. Use this
        everywhere instead of bare self.dropoff() so the firmware always knows
        when to dump."""
        n_before = self.get_duplo_count()
        self.set_sweeper_mode(SweeperMode.DROPOFF)
        self.dropoff()                       # nav to base + open-loop reverse
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
