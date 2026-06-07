"""
Duplo-aspiration mission:
  1. Nav2 to button pose
  2. Open-loop push to press button, wait for door
  3. Explore Zone A via reachability-checked waypoint grid
  4. Return to base
  5. Explore Zone B via reachability-checked waypoint grid
  6. Return to base

Duplos are caught passively (robot rolls over them); no detection FSM.
"""

import math
import time
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import (QoSProfile, DurabilityPolicy,
                       ReliabilityPolicy, HistoryPolicy)

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from nav2_msgs.action import ComputePathToPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# ──────────────────────────────────────────────────────────────────────────────
#  POSES & PATHS  (TODO: tune to your arena)
# ──────────────────────────────────────────────────────────────────────────────

BASE_POSE   = (0.45, 0.45, -1.57)
#START_POSE  = (0.557, 0.626, 1.50)
START_POSE=(4.255, 5.228, 1.50)

BUTTON_APPROACH = (4.45, 7.40, 1.57) # Nav2 stops here (precise heading)
BUTTON_PUSH_SPEED = 0.10               # m/s, open-loop forward
BUTTON_PUSH_TIME  = 1.5                # s
BUTTON_BACKOFF_SPEED = -0.10           # m/s, reverse after push
BUTTON_BACKOFF_TIME  = 3            # s
BUTTON_BACKOFF_STEP = 0.2

BUTTON_ROTATE_SPEED    = 1.0           # rad/s during the 180° spin
BUTTON_ROTATE_TIME     = math.pi / BUTTON_ROTATE_SPEED   # = π s for 180°

DOOR_DWELL_S           = 1           # wait this long after backing off
DOOR_PROBE_POSE        = (2.21, 7.40, 3.14)   # a point on the OTHER side of the door 
MAX_BUTTON_RETRIES     = 3

DOOR_WAIT_S        = 3.0               # fallback dwell dif no /door_open topic
DOOR_TOPIC         = '/door_open'      # optional std_msgs/Bool, latched
DOOR_USE_TOPIC     = False             # set True if you publish /door_open

WAYPOINTS_ZONE_A   = '/maps/arena/waypoints_left.yaml'
WAYPOINTS_ZONE_B   = '/maps/arena/waypoints_left.yaml'

ZONE_A_TIMEOUT_S   = 180.0
ZONE_B_TIMEOUT_S   = 120.0

NAV_GOAL_TIMEOUT_S = 20.0
PLAN_TIMEOUT_S     = 8.0
MAX_NODE_RETRIES   = 2

RAMP_VEL_TOPIC     = 'ramp_vel'        # high-priority twist_mux input


# ──────────────────────────────────────────────────────────────────────────────
#  HELPERS
# ──────────────────────────────────────────────────────────────────────────────

def make_pose(x: float, y: float, yaw: float, frame: str = 'map') -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def node_key(wp):
    return (round(float(wp[0]), 3), round(float(wp[1]), 3))


# ──────────────────────────────────────────────────────────────────────────────
#  MISSION RUNNER
# ──────────────────────────────────────────────────────────────────────────────

class MissionRunner(BasicNavigator):

    def __init__(self) -> None:
        super().__init__('mission_runner')

        self._ramp_pub = self.create_publisher(Twist, RAMP_VEL_TOPIC, 10)

        # Goal-checker selector (latched, transient_local — survives late subscribers)
        gc_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
        self._gc_pub = self.create_publisher(String, 'goal_checker_selector', gc_qos)

        # ComputePathToPose action client for reachability checks
        self._plan_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        # Optional door signal
        self._door_open = False
        if DOOR_USE_TOPIC:
            self.create_subscription(Bool, DOOR_TOPIC,
                                     lambda m: setattr(self, '_door_open', bool(m.data)),
                                     10)

    # ── primitives ────────────────────────────────────────────────────────────

    def _select_goal_checker(self, name: str) -> None:
        self._gc_pub.publish(String(data=name))
        time.sleep(0.1)

    def go_to(self, pose_tuple, timeout_s: float = NAV_GOAL_TIMEOUT_S,
              precise: bool = False) -> bool:
        self._select_goal_checker(
            'precise_goal_checker' if precise else 'general_goal_checker')
        self.goToPose(make_pose(*pose_tuple))
        t0 = time.time()
        while not self.isTaskComplete():
            if time.time() - t0 > timeout_s:
                self.cancelTask()
                self.get_logger().warn(f'go_to timeout — {pose_tuple}')
                return False
        ok = self.getResult() == TaskResult.SUCCEEDED
        if not ok:
            self.get_logger().warn(
                f'go_to {self.getResult().name} — {pose_tuple}')
        return ok

    def _open_loop_drive(self, speed_m_s: float, duration_s: float,
                         hz: float = 20.0) -> None:
        """Constant-velocity twist on ramp_vel for duration_s."""
        cmd = Twist()
        cmd.linear.x = float(speed_m_s)
        period = 1.0 / hz
        for _ in range(round(duration_s * hz)):
            self._ramp_pub.publish(cmd)
            time.sleep(period)
        self._ramp_pub.publish(Twist())

    def _is_reachable(self, pose_tuple, timeout_s: float = PLAN_TIMEOUT_S) -> bool:
        """
        Ask Nav2's planner if there's a path to pose_tuple from the current pose.
        Returns True iff a non-empty path comes back within timeout_s.
        """
        if not self._plan_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('compute_path_to_pose unavailable — assuming reachable')
            return True

        goal = ComputePathToPose.Goal()
        goal.goal = make_pose(*pose_tuple)
        goal.use_start = False

        send_future = self._plan_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_s)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_s)
        if not result_future.done():
            gh.cancel_goal_async()
            return False

        result = result_future.result().result
        return bool(result and result.path.poses)

    # ── button + door ─────────────────────────────────────────────────────────

    def _open_loop_rotate(self, yaw_rate: float, duration_s: float,
                      hz: float = 20.0) -> None:
        """Constant angular twist on ramp_vel for duration_s."""
        cmd = Twist()
        cmd.angular.z = float(yaw_rate)
        period = 1.0 / hz
        for _ in range(round(duration_s * hz)):
            self._ramp_pub.publish(cmd)
            time.sleep(period)
        self._ramp_pub.publish(Twist())


    def push_button_and_wait_for_door(self) -> bool:
        """
        Approach pose is assumed reached. Sequence:
        1. Rotate 180° in place    (so the back of the robot faces the button)
        2. Back up a few cm         (depresses the button with the rear bumper)
        3. Dwell while the door opens
        4. Probe with ComputePathToPose against DOOR_PROBE_POSE
            - if reachable: door's open, we're done
            - if not: retry from step 1, up to MAX_BUTTON_RETRIES
        Returns True if the door eventually opened, False if we ran out of retries.
        """
        for attempt in range(1, MAX_BUTTON_RETRIES + 1):
            backtrack_time = BUTTON_BACKOFF_TIME + (attempt - 1) * BUTTON_BACKOFF_STEP
            
            if(attempt == 1):
                self.get_logger().info(f'BUTTON: attempt {attempt}/{MAX_BUTTON_RETRIES} — rotate 180°')
                self._open_loop_rotate(BUTTON_ROTATE_SPEED, BUTTON_ROTATE_TIME)
            else:
                self.get_logger().info(f'BUTTON: attempt {attempt}/{MAX_BUTTON_RETRIES} — rotate 90°')
                self._open_loop_rotate(BUTTON_ROTATE_SPEED / 2, BUTTON_ROTATE_TIME)

            self.get_logger().info('BUTTON: backing into button')
            self._open_loop_drive(BUTTON_BACKOFF_SPEED, backtrack_time)

            self.get_logger().info('BUTTON: moving back from button')
            self._open_loop_drive(BUTTON_PUSH_SPEED, backtrack_time)

            self.get_logger().info(f'BUTTON: pressing the button during {DOOR_DWELL_S}')
            time.sleep(DOOR_DWELL_S)

            self.get_logger().info('BUTTON: rotate towards door')
            self._open_loop_rotate(- BUTTON_ROTATE_SPEED / 2, BUTTON_ROTATE_TIME)

            self.get_logger().info(f'BUTTON: waiting {DOOR_WAIT_S:.1f}s for door to open')
            time.sleep(DOOR_WAIT_S)

            self.get_logger().info(
                f'BUTTON: probing reachability to {DOOR_PROBE_POSE}')
            if self._is_reachable(DOOR_PROBE_POSE):
                self.get_logger().info('BUTTON: door open (probe succeeded)')
                return True

            self.get_logger().warn(
                f'BUTTON: probe failed — door still closed, retrying')

        self.get_logger().error(f'BUTTON: door never opened after {MAX_BUTTON_RETRIES} attempts')
        return False

    # ── waypoint-grid exploration ─────────────────────────────────────────────

    def explore_zone(self, waypoints_file: str, duration_s: float,
                     label: str = 'ZONE') -> None:
        """
        Drive every waypoint in `waypoints_file` once, skipping unreachable ones.
        Failures are retried up to MAX_NODE_RETRIES times (later in the pass).
        Stops when grid is exhausted OR duration_s expires.
        """
        with open(waypoints_file) as f:
            grid = list(yaml.safe_load(f)['waypoints'])

        done = set()
        failed = {}
        last_xy = None
        deadline = time.time() + duration_s

        self.get_logger().info(
            f'{label}: {len(grid)} waypoints, budget {duration_s:.0f}s')

        self._select_goal_checker('general_goal_checker')

        while rclpy.ok() and time.time() < deadline:
            # Build candidate list (unvisited, not exhausted)
            candidates = []
            for wp in grid:
                k = node_key(wp)
                if k in done:
                    continue
                if failed.get(k, 0) >= MAX_NODE_RETRIES:
                    continue
                candidates.append(wp)

            if not candidates:
                self.get_logger().info(f'{label}: all waypoints visited')
                return

            # Motion-coherent pick: closest, with a penalty for reversing direction
            rx, ry = self._current_xy()

            def score(w):
                x, y = w[0], w[1]
                dist = math.hypot(x - rx, y - ry)
                if last_xy is None:
                    return dist
                lx, ly = last_xy
                v1 = (lx - rx, ly - ry)
                v2 = (x - rx,  y - ry)
                n1 = math.hypot(*v1) + 1e-6
                n2 = math.hypot(*v2) + 1e-6
                cos_t = (v1[0]*v2[0] + v1[1]*v2[1]) / (n1 * n2)
                return dist + 2.0 * (1.0 - cos_t)

            wp = min(candidates, key=score)
            k = node_key(wp)

            # Reachability gate — skip if no plan
            if not self._is_reachable(wp):
                failed[k] = failed.get(k, 0) + 1
                self.get_logger().warn(
                    f'{label}: {k} unreachable ({failed[k]}/{MAX_NODE_RETRIES})')
                if failed[k] >= MAX_NODE_RETRIES:
                    done.add(k)
                continue

            # Navigate
            time_left = deadline - time.time()
            if time_left <= 0:
                break
            ok = self.go_to(wp, timeout_s=min(NAV_GOAL_TIMEOUT_S, time_left))

            if ok:
                done.add(k)
                last_xy = (wp[0], wp[1])
                self.get_logger().info(f'{label}: {k} done')
            else:
                failed[k] = failed.get(k, 0) + 1
                self.get_logger().warn(
                    f'{label}: {k} nav failed ({failed[k]}/{MAX_NODE_RETRIES})')
                if failed[k] >= MAX_NODE_RETRIES:
                    done.add(k)

        self.get_logger().info(
            f'{label}: ended  visited={len(done)} / {len(grid)}  '
            f'time_left={max(0.0, deadline - time.time()):.0f}s')

    def _current_xy(self):
        """
        Best-effort current robot XY from AMCL via BasicNavigator's _amcl_pose
        cache. Falls back to (0,0) if unset.
        """
        pose = getattr(self, '_amcl_pose', None)
        if pose is None:
            return (0.0, 0.0)
        return (pose.pose.pose.position.x, pose.pose.pose.position.y)

    # ── mission ───────────────────────────────────────────────────────────────

    def run(self) -> None:
        self.setInitialPose(make_pose(*START_POSE))
        self.waitUntilNav2Active(localizer='amcl')
        self.get_logger().info('Nav2 active — mission start')

        # 1. Go to the button
        self.go_to(BUTTON_APPROACH, precise=False)

        # 2. Push it, wait for the door
        if not self.push_button_and_wait_for_door():
            self.get_logger().error('Aborting mission — could not open door')
            self.go_to(BASE_POSE)
            return
        
        # 3. Traverse the door
        self.go_to(DOOR_PROBE_POSE, precise=False)

        # 3. Explore the zone behind the door
        self.explore_zone(WAYPOINTS_ZONE_A, ZONE_A_TIMEOUT_S, label='ZONE_A')

        # 4. Back to base
        self.go_to(BASE_POSE)

        # 5. Explore the second zone
        self.explore_zone(WAYPOINTS_ZONE_B, ZONE_B_TIMEOUT_S, label='ZONE_B')

        # 6. Home
        self.go_to(BASE_POSE)

        self.get_logger().info('MISSION COMPLETE')


def main() -> None:
    rclpy.init()
    runner = MissionRunner()
    try:
        runner.run()
    except KeyboardInterrupt:
        runner.get_logger().info('Interrupted')
    finally:
        runner._ramp_pub.publish(Twist())
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()