"""
Duplo-Aspiration mission — step sequence only.

All shared infrastructure lives in mixin/base files:
  mission_base.py    — nav primitives, recovery, dropoff, /mission_command lifecycle
  mission_duplo.py   — visual-servo handoff, sweep, opportunistic collect, explore_zone
  mission_button.py  — button press + door probe

Sequence:
    1. Nav2 to button approach pose (CRITICAL, precise heading)
    2. Open-loop push the button, retry-and-probe for door to open
    3. Traverse the door (CRITICAL)
    4. Explore Zone 3 — visual-servo collection
    5. Return to button area, then base — dropoff
    6. Explore Zone 1
    7. Dropoff again
"""

import time

import rclpy
from geometry_msgs.msg import Twist

from j2cdynamics_mission.mission_base import (
    MissionBase, MissionAbortException, make_pose,
)
from j2cdynamics_mission.mission_duplo import DuploMixin
from j2cdynamics_mission.mission_button import ButtonMixin, DOOR_PROBE_POSE


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
#  MISSION RUNNER  —  compose mixins, define run()
# ──────────────────────────────────────────────────────────────────────────────

class DaMissionRunner(DuploMixin, ButtonMixin, MissionBase):
    """DA mission = Duplo collection behind a button-controlled door. Mix in
    DuploMixin (visual-servo + explore_zone) and ButtonMixin (push button +
    probe door). The MRO ensures cooperative __init__ via super() chain."""

    # Class attributes consumed by MissionBase.dropoff() and main_loop():
    BASE_POSE             = BASE_POSE
    DROPFF_FIRST_WAYPOINT = DROPFF_FIRST_WAYPOINT
    START_POSE            = START_POSE
    DROPOFF_SPEED         = -0.3
    DROPOFF_TIME          = 3.0

    # ── Mission step sequence ────────────────────────────────────────────────

    def _build_steps(self):
        """Step list driven by MissionBase.run(). On RESET, the same step is
        re-tried — completed steps keep their _next_step advance."""
        return [
            ('SETUP',           self._step_setup),
            ('BUTTON_APPROACH', self._step_button_approach),
            ('PUSH_BUTTON',     self._step_push_button),
            ('DOOR_TRAVERSE',   self._step_door_traverse),
            ('ZONE_3',          self._step_zone_3),
            ('RETURN_TO_BTN',   self._step_return_to_button),
            ('DROPOFF_1',       self.dropoff),
            ('ZONE_1',          self._step_zone_1),
            ('DROPOFF_2',       self.dropoff),
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

    def _step_button_approach(self) -> None:
        if not self._go_to_with_recovery(
                BUTTON_APPROACH, label='BUTTON_APPROACH', precise=True):
            self.get_logger().error('Aborting — could not reach button approach')
            raise MissionAbortException()

    def _step_push_button(self) -> None:
        if not self.push_button_and_wait_for_door():
            self.get_logger().error('Aborting — could not open door')
            # Door failure is unrecoverable; raise so we don't try the subsequent
            # door-traverse step (which would just fail).
            raise MissionAbortException()

    def _step_door_traverse(self) -> None:
        if not self._go_to_with_recovery(
                DOOR_PROBE_POSE, label='DOOR_TRAVERSE', precise=True):
            self.get_logger().error('Aborting — could not traverse door')
            raise MissionAbortException()

    def _step_zone_3(self) -> None:
        self.explore_zone(WAYPOINTS_ZONE_3, TIMEOUT_ZONE_3, label='ZONE_3')

    def _step_return_to_button(self) -> None:
        self.go_to(BUTTON_APPROACH, precise=True)

    def _step_zone_1(self) -> None:
        self.explore_zone(WAYPOINTS_ZONE_1, TIMEOUT_ZONE_1, label='ZONE_1')


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
