"""
J2C-Dynamics mission strategy orchestrator.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String

DUPLO_DURING_PATROL = False   # Activate for duplo detection during patrol


# ──────────────────────────────────────────────────────────────────────────────
#  MISSION POSES  —  TODO: fill in from RViz "Publish Point" after mapping
#  Format: (x_metres, y_metres, yaw_radians) in the map frame
# ──────────────────────────────────────────────────────────────────────────────

BASE_POSE  = (0.35, 0.3, -1.57)      
START_POSE = (1.25, 0.4, 0.02)        

# The ramp approach heading is the high-variance item — tune this yaw carefully.
RAMP_APPROACH = (8.20, 4.30, 0.0)   
RAMP_TOP      = (8.20, -6, 0.0)   

MAIN_PATROL = [                    
    (4, 1.06,  0.0),
    (7, 2.3,  1.57),
    (4.75, 3.66,  3.33),
    (0.35, 0.3, -1.57),
]

UPPER_PATROL = [                   # TODO: waypoints covering the upper platform
    (4.5, 1.5, 0.0),
    (5.0, 2.0, 1.57),
]

#  TIMING  (seconds)
PATROL_MAIN_S  = 240.0   # 4 min in lower arena
PATROL_UPPER_S = 120.0   # 2 min on upper platform — set to remaining time budget


#  RAMP OPEN-LOOP DRIVE  —  TUNE ON HARDWARE  (distance ≈ speed × time)
RAMP_VEL_TOPIC     = 'ramp_vel'   # must match twist_mux.yaml at priority 150

RAMP_CLIMB_SPEED   =  0.15        # m/s  (positive = forward / up)
RAMP_CLIMB_TIME    =  6.0         # s
RAMP_DESCEND_SPEED =  0.12        # m/s  (sign is set in run(); always positive here)
RAMP_DESCEND_TIME  =  6.0         # s


#  HELPERS
def make_pose(x: float, y: float, yaw: float, frame: str = 'map') -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


#  MISSION RUNNER
class MissionRunner(BasicNavigator):
    """
    Extends BasicNavigator (nav2_simple_commander) with three primitives:

        go_to(pose_tuple)          — navigate to a single pose
        patrol(waypoints, seconds) — cycle waypoints until a deadline
        drive_ramp(speed, seconds) — open-loop straight drive (for the ramp)

    BasicNavigator already owns the Nav2 action clients, lifecycle waiter,
    and the isTaskComplete / getResult / cancelTask loop.
    """

    def __init__(self) -> None:
        super().__init__('mission_runner')
        self._ramp_pub  = self.create_publisher(Twist, RAMP_VEL_TOPIC, 10)
        self._duplo_pub = self.create_publisher(Bool, '/enable_duplo_collection', 10)

        # Goal-checker selection. TRANSIENT_LOCAL so the BT's GoalCheckerSelector
        # (which subscribes latched) reliably sees the latest choice even if it
        # subscribed after we published.
        gc_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
        self._gc_pub = self.create_publisher(String, 'goal_checker_selector', gc_qos)

    def _select_goal_checker(self, name: str) -> None:
        """Tell the BT which goal checker to use for the NEXT goal."""
        self._gc_pub.publish(String(data=name))
        time.sleep(0.1)   # let the latched selection propagate before the goal starts

    # ── single-goal navigation ─────────────────────────────────────────────────
    def go_to(self, pose_tuple: tuple, timeout_s: float = 120.0,
              precise: bool = False) -> bool:
        """
        Navigate to (x, y, yaw). Blocks until Nav2 reports SUCCEEDED or
        timeout_s is exceeded. Returns True on success.

        precise=False (default): position-only goal (patrol) — stops on arrival,
                                 no in-place heading rotation.
        precise=True:            enforce the goal heading (ramp approach).
        """
        self._select_goal_checker(
            'precise_goal_checker' if precise else 'general_goal_checker')
        self.goToPose(make_pose(*pose_tuple))
        t0 = time.time()

        while not self.isTaskComplete():
            if time.time() - t0 > timeout_s:
                self.cancelTask()
                self.get_logger().warn(
                    f'go_to timeout after {timeout_s:.0f}s — {pose_tuple}')
                return False

        ok = self.getResult() == TaskResult.SUCCEEDED
        if not ok:
            self.get_logger().warn(
                f'go_to failed ({self.getResult().name}) — {pose_tuple}')
        return ok

    # ── timed patrol ──────────────────────────────────────────────────────────
    def patrol(self, waypoints: list, duration_s: float) -> None:
        """
        Cycle through waypoints until duration_s wall-clock seconds have passed.

        A single waypoint failure is logged and skipped; the patrol continues
        to the next waypoint. This prevents one bad goal from killing the run.
        stop_on_failure is False in nav2_params.yaml, which matches this intent.
        """
        deadline = time.time() + duration_s
        n = len(waypoints)
        idx = 0

        # Position-only for the whole patrol (resets any 'precise' left latched
        # from a prior ramp goal). Selected once — the choice is latched.
        self._select_goal_checker('general_goal_checker')

        self.get_logger().info(
            f'PATROL  {n} waypoints  {duration_s:.0f}s budget')

        while rclpy.ok() and time.time() < deadline:
            self.goToPose(make_pose(*waypoints[idx % n]))

            # spin until done or deadline
            while not self.isTaskComplete():
                if time.time() >= deadline:
                    self.cancelTask()
                    self.get_logger().info(
                        f'PATROL deadline reached — {idx} waypoints visited')
                    return

            result = self.getResult()
            if result != TaskResult.SUCCEEDED:
                self.get_logger().warn(
                    f'PATROL wp {idx % n} {result.name} — skipping')

            idx += 1

        self.get_logger().info(f'PATROL done — {idx} waypoints visited')

    # ── ramp open-loop drive ───────────────────────────────────────────────────
    def drive_ramp(self, speed_m_s: float, duration_s: float, hz: float = 20.0) -> None:
        """
        Publish a constant Twist on ramp_vel for duration_s seconds.

        twist_mux gives ramp_vel priority 150, which is above duplo_vel (100)
        and nav_vel (10). Nav2 keeps its plan alive but cannot move the robot
        while ramp_vel is publishing. duplo_approach cannot override the climb.
        """
        self.get_logger().info(
            f'RAMP  {speed_m_s:+.3f} m/s  {duration_s:.1f}s')

        cmd = Twist()
        cmd.linear.x = float(speed_m_s)
        period = 1.0 / hz

        for _ in range(round(duration_s * hz)):
            self._ramp_pub.publish(cmd)
            time.sleep(period)

        self._ramp_pub.publish(Twist())    # explicit stop
        self.get_logger().info('RAMP done')

    # ── duplo collection gate ──────────────────────────────────────────────────
    def _set_duplo(self, enabled: bool) -> None:
        """
        Enable or disable duplo_approach velocity output.
        When disabled, duplo_approach still runs its FSM but publishes Twist(0,0),
        so twist_mux sees silence on duplo_vel and falls through to nav_vel.
        """
        msg = Bool()
        msg.data = enabled
        self._duplo_pub.publish(msg)

    # ── mission ───────────────────────────────────────────────────────────────
    def run(self) -> None:
        # Seed AMCL with the dock pose (so autonomous runs don't need an RViz
        # 2D Pose Estimate), then block until AMCL + Nav2 are actually active.
        # localizer='amcl' (NOT 'slam_toolbox') — that mismatch hangs the wait.
        self.setInitialPose(make_pose(*START_POSE))
        self.waitUntilNav2Active(localizer='amcl')
        self.get_logger().info('Nav2 active — mission start')

        # ── Phase 1: lower-arena patrol ───────────────────────────────────────
        self._set_duplo(DUPLO_DURING_PATROL)
        self.patrol(MAIN_PATROL, PATROL_MAIN_S)

        # ── Phase 2: return to base before the ramp ───────────────────────────
        self._set_duplo(False)
        self.go_to(BASE_POSE)

        # ── Phase 3: ramp approach + open-loop climb ──────────────────────────
        # Nav2 navigates to the base of the ramp; then open-loop takes over.
        # The yaw of RAMP_APPROACH is what determines the 2cm alignment margin,
        # so enforce the heading here (precise goal checker).
        self.go_to(RAMP_APPROACH, precise=True)
        self.drive_ramp(+RAMP_CLIMB_SPEED, RAMP_CLIMB_TIME)

        # ── Phase 4: upper-platform patrol ────────────────────────────────────
        self._set_duplo(DUPLO_DURING_PATROL)
        self.patrol(UPPER_PATROL, PATROL_UPPER_S)

        # ── Phase 5: re-square at ramp top, then open-loop descend ────────────
        self._set_duplo(False)
        self.go_to(RAMP_TOP, precise=True)   # re-square before the open-loop descent
        self.drive_ramp(-RAMP_DESCEND_SPEED, RAMP_DESCEND_TIME)

        # ── Phase 6: home ─────────────────────────────────────────────────────
        self.go_to(BASE_POSE)

        self.get_logger().info('MISSION COMPLETE')


def main() -> None:
    rclpy.init()
    runner = MissionRunner()
    try:
        runner.run()
    except KeyboardInterrupt:
        runner.get_logger().info('Mission interrupted by user')
    finally:
        runner._ramp_pub.publish(Twist())   # leave robot stopped
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
