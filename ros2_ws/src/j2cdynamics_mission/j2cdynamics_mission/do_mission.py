"""
Duplo-Obliterator mission — step sequence only.

All shared infrastructure lives in mixin/base files:
  mission_base.py    — nav primitives, recovery, dropoff, /mission_command lifecycle
  mission_duplo.py   — visual-servo handoff, sweep, opportunistic collect, explore_zone
  mission_ramp.py    — ramp climb sequence

Sequence:
    1. Explore Zone 1 (waypoints_zone1_do.yaml) — visual-servo collection
    2. Return to base — dropoff
    3. Nav2 to low ramp pose (CRITICAL, multi-attempt recovery)
    4. Open-loop climb the ramp
    5. setInitialPose for upper-zone AMCL seed
    6. Explore Zone 4 (waypoints_zone4.yaml)
    7. Nav2 to ramp exit
    8-10. (Descent + second dropoff currently commented; future work)
"""

import time

import rclpy
from geometry_msgs.msg import Twist

from j2cdynamics_mission.mission_base import (
    MissionBase, MissionAbortException, make_pose,
)
from j2cdynamics_mission.mission_duplo import DuploMixin
from j2cdynamics_mission.mission_ramp import RampMixin


# ──────────────────────────────────────────────────────────────────────────────
#  POSES & PATHS  (Duplo-Obliterator specific)
# ──────────────────────────────────────────────────────────────────────────────

BASE_POSE             = (0.05, 0.25, 3.14)
DROPFF_FIRST_WAYPOINT = (1.5, 0.4, 3.14)
START_POSE            = (1.25, 0.4, 0.02)
# START_POSE          = (8.38, 5.88, 1.50)  # alt: start at ramp top for debugging

RAMP_APPROACH     = (7.95, 4.0, 0.02)
RAMP_TOP          = (8.17, 5.88, 1.50)
RAMP_EXIT_FIRST   = (8.30, 6.25, 0.0)
RAMP_EXIT_SECOND  = (8.30, 6.25, 3.25)

WAYPOINTS_ZONE_1 = '/maps/arena/waypoints_zone1_do.yaml'
WAYPOINTS_ZONE_4 = '/maps/arena/waypoints_zone4.yaml'

TIMEOUT_ZONE_1 = 200.0
TIMEOUT_ZONE_4 = 240.0


# ──────────────────────────────────────────────────────────────────────────────
#  MISSION RUNNER  —  compose mixins, define run()
# ──────────────────────────────────────────────────────────────────────────────

class DoMissionRunner(DuploMixin, RampMixin, MissionBase):
    """DO mission = Duplo collection + ramp traversal. Mix in DuploMixin
    (visual-servo + explore_zone) and RampMixin (go_up_ramp / go_down_ramp).
    The MRO ensures cooperative __init__ via super() chain."""

    # Class attributes consumed by MissionBase.dropoff() and main_loop():
    BASE_POSE             = BASE_POSE
    DROPFF_FIRST_WAYPOINT = DROPFF_FIRST_WAYPOINT
    START_POSE            = START_POSE
    DROPOFF_SPEED         = -0.3
    DROPOFF_TIME          = 3.0

    # ── Mission step sequence ────────────────────────────────────────────────

    def _build_steps(self):
        """Step list driven by MissionBase.run(). On RESET, the same step is
        re-tried — completed steps keep their _next_step advance. Operator is
        responsible for placing the robot in a sane state before resetting
        (e.g. back at START_POSE if SETUP needs to re-seed AMCL)."""
        return [
            ('SETUP',          self._step_setup),
            #('ZONE_1',         self._step_zone_1),
            #('DROPOFF_1',      self.dropoff),
            ('RAMP_APPROACH',  self._step_ramp_approach),
            ('RAMP_CLIMB',     self.go_up_ramp),
            ('RAMP_TOP_RESEED', self._step_ramp_top_reseed),
            ('ZONE_4',         self._step_zone_4),
            ('RAMP_EXIT',      self._step_ramp_exit),
            ('DOWN_RAMP',      self.go_down_ramp),
            ('SAFE_POINT',     self.go_to((7.95, 4.0, 0.02), precise=False)),
            #('ESTIMATE_POSITION', self._ramp_down_pose_estimate),
            ('DROPOFF_2',       self.dropoff),
            ('ZONE_1_SECOND', self._step_zone_1),
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
        # Visual servo defaults to enabled on startup; disable here so
        # mission-step transit isn't hijacked. explore_zone re-enables it
        # for sweeps and opportunistic collect.
        self._enable_collection(False)

    def _step_zone_1(self) -> None:
        self.explore_zone(WAYPOINTS_ZONE_1, TIMEOUT_ZONE_1, label='ZONE_1')

    def _step_ramp_approach(self) -> None:
        if not self._go_to_with_recovery(RAMP_APPROACH, label='RAMP_APPROACH', max_attempts=10, timeout_s=90, precise=True):
            self.get_logger().error(f'RAMP_APPROACH {RAMP_APPROACH} failed after all attempts')
            raise MissionAbortException()

    def _step_ramp_top_reseed(self) -> None:
        self.setInitialPose(make_pose(*RAMP_TOP))

    def _step_zone_4(self) -> None:
        self.explore_zone(WAYPOINTS_ZONE_4, TIMEOUT_ZONE_4, label='ZONE_4')

    def _step_ramp_exit(self) -> None:
        self._go_to_with_recovery(RAMP_EXIT_FIRST, label="RAMP FIRST WAYPOINT", precise=False)
        if not self._go_to_with_recovery(RAMP_EXIT_SECOND, label="RAMP EXIT", precise=True):
            self.get_logger().error(f'RAMP DOWN {RAMP_EXIT_SECOND} failed after all attempts')
            raise MissionAbortException()

    def _ramp_down_pose_estimate(self) -> None:
        self.setInitialPose(make_pose(*RAMP_APPROACH))

def main() -> None:
    rclpy.init()
    runner = DoMissionRunner()
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
