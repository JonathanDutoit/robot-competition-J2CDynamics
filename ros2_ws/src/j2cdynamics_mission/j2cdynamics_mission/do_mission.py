"""
Duplo-obliterator mission:
    1. Explore Zone 1 via reachability-checked waypoint grid 
    2. Return to base (drop off)
    3. Nav2 to low ramp pose
    4. Open-loop to go up the ramp 
    5. Send Nav2 Pose Estimate to init pose of Zone 4
    6. Explore Zone 4 via reachability-checked waypoint grid 
    7. Go to high ramp pose 
    8. Open-loop to go down the ramp 
    9. Send Nav2 Pose Estimate down ramp pose 
    10. Return to base (drop off)
    11. If time allows, go into carpet 
    12. Return to base (drop off)
"""

import math
import time
import yaml
import threading 

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import (QoSProfile, DurabilityPolicy,
                       ReliabilityPolicy, HistoryPolicy)

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav2_msgs.action import ComputePathToPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ──────────────────────────────────────────────────────────────────────────────
#  POSES & PATHS  
# ──────────────────────────────────────────────────────────────────────────────

BASE_POSE  = (0.35, 0.3, -1.57)      
START_POSE = (1.25, 0.4, 0.02)  

RAMP_APPROACH = (8.20, 4.30, 0.0)   
RAMP_TOP      = (8.20, -6, 0.0) 

WAYPOINTS_ZONE_4  = '/maps/arena/waypoints_zone4.yaml'
WAYPOINTS_ZONE_1  = '/maps/arena/waypoints_zone1_do.yaml'

TIMEOUT_ZONE_4 = 240.0
TIMEOUT_ZONE_1 = 200.0

MISSION_TIMEOUT = 600.0
MISSION_CLOSING_TIME = 60.0

NAV_GOAL_TIMEOUT_S = 45.0
PLAN_TIMEOUT_S     = 8.0
MAX_NODE_RETRIES   = 2

DUPLO_COUNT_ZONE_4 = 6

SCAN_ROT_STEP = 0.8

RAMP_VEL_TOPIC     = 'ramp_vel'   # must match twist_mux.yaml at priority 150


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

# Helper exception for the mission time handling 
class MissionAbortException(Exception):
    pass


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

        # Watchdog for end of mission time handling 
        self.abort_event = threading.Event()
        self.mission_start = 0.0

    def _supervisor(self):
        while rclpy.ok():
            elapsed = time.time() - self.mission_start
            if MISSION_TIMEOUT - elapsed <= MISSION_CLOSING_TIME:
                self.abort_event.set()
                self.get_logger().warn(f'MISSION ABORT – only ≤{MISSION_CLOSING_TIME}s left')
                self.cancelTask()                 # kill Nav2 goal
                self._ramp_pub.publish(Twist())   # stop open‑loop motion
                break
            time.sleep(0.2)


     # ── primitives ────────────────────────────────────────────────────────────

    def _select_goal_checker(self, name: str) -> None:
        self._gc_pub.publish(String(data=name))
        time.sleep(0.1)

    def go_to(self, pose_tuple, timeout_s: float = NAV_GOAL_TIMEOUT_S, precise: bool = False) -> bool:
        if self.abort_event.is_set():
            raise MissionAbortException()

        self._select_goal_checker('precise_goal_checker' if precise else 'general_goal_checker')
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
        cmd = Twist()
        cmd.linear.x = float(speed_m_s)
        period = 1.0 / hz
        for _ in range(round(duration_s * hz)):
            self._ramp_pub.publish(cmd)
            time.sleep(period)
        self._ramp_pub.publish(Twist())

    def _open_loop_rotate(self, yaw_rate: float, duration_s: float,
                      hz: float = 20.0) -> None:
        cmd = Twist()
        cmd.angular.z = float(yaw_rate)
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

    # ── ramp ─────────────────────────────────────────────────────────

    def go_up_ramp(self) -> bool:
        pass

    def go_down_ramp(self) -> bool: 
        pass


     # ── waypoint-grid exploration ─────────────────────────────────────────────

    def explore_zone(self, waypoints_file: str, duration_s: float,
                     label: str = 'ZONE') -> None:
        """
        Sparse waypoint exploration + active scanning + reactivate duplo handling. 

        Behavior: 
        - Visit sparse waypoints
        - At each waypoints
            - rotate 360 (scan)
            - check duplo_map 
            - if duplos exist -> interrupt exploration and collect 
        - Continue until:
            - tine expires OR
            - all anchors visited AND no duplos remain (best-effort)
        """
        with open(waypoints_file) as f:
            grid = list(yaml.safe_load(f)['waypoints'])

        done = set()
        failed = {}
        last_xy = None
        deadline = time.time() + duration_s

        self.get_logger().info(f'{label}: {len(grid)} waypoints, budget {duration_s:.0f}s')

        self._select_goal_checker('general_goal_checker')

        while rclpy.ok() and time.time() < deadline:
            # ─────────────────────────────────────────────
            # 1. Filter candidates
            # ─────────────────────────────────────────────
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
            
            # ─────────────────────────────────────────────
            # 2. Pick next waypoint with motion-coherence: 
            # closest, with a penalty for reversing direction
            # ─────────────────────────────────────────────
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

            # ─────────────────────────────────────────────
            # 3. Reachability gate
            # ─────────────────────────────────────────────
            if not self._is_reachable(wp):
                failed[k] = failed.get(k, 0) + 1
                self.get_logger().warn(
                    f'{label}: {k} unreachable ({failed[k]}/{MAX_NODE_RETRIES})')
                if failed[k] >= MAX_NODE_RETRIES:
                    done.add(k)
                continue

            # ─────────────────────────────────────────────
            # 4. Navigate to waypoint
            # ─────────────────────────────────────────────
            time_left = deadline - time.time()
            if time_left <= 0:
                break

            ok = self.go_to(wp, timeout_s=min(NAV_GOAL_TIMEOUT_S, time_left), precise=False)

            if not ok:
                failed[k] = failed.get(k, 0) + 1
                self.get_logger().warn(
                    f'{label}: {k} nav failed ({failed[k]}/{MAX_NODE_RETRIES})')
                continue

            done.add(k)
            last_xy = (wp[0], wp[1])

            # ─────────────────────────────────────────────
            # 5. SCAN PHASE 
            # ─────────────────────────────────────────────
            self.get_logger().info(f"{label}: scanning at {k}")

            self._open_loop_rotate(SCAN_ROT_STEP, 2 * math.pi / SCAN_ROT_STEP)  # full 360 scan

            duplos = self._read_duplos()

            if duplos:
                self.get_logger().info(
                    f"{label}: detected {len(duplos)} duplos → switching to collection"
                )

                for d in duplos:
                    self._handle_detected_duplo(d)

                # optional: re-check after collection cycle
                continue
            

        self.get_logger().info(
            f'{label}: ended  visited={len(done)} / {len(grid)}  '
            f'time_left={max(0.0, deadline - time.time()):.0f}s')
        
    def _read_duplos(self):
        """
        Converts current duplo_map into usable list.
        Centralizes perception logic.
        """
        msg = getattr(self, '_duplo_map_cache', None)
        if msg is None:
            return []

        return [(p.position.x, p.position.y) for p in msg.poses]

    def on_duplo_map(msg, self):
        self._duplo_map_cache = msg

    def _handle_detected_duplo(self, duplo_xy):
        """
        Minimal hook: override later with full FSM if needed.
        """
        self.get_logger().info(f"Handling duplo at {duplo_xy}")

        # reuse your existing pipeline if you want:
        # 1. set target
        # 2. approach
        # 3. collect

        # placeholder:
        pass

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
        
        # Watchdogs for mission time handlong 
        self.mission_start = time.time()
        self.abort_event.clear()
        threading.Thread(target=self._supervisor, daemon=True).start()

        self.get_logger().info('Nav2 active — mission start')

        try:
            #1. Explore Zone 1 via reachability-checked waypoint grid 
            self.explore_zone(WAYPOINTS_ZONE_1, TIMEOUT_ZONE_1, label='ZONE_1')

            #2. Return to base (drop off)
            self.go_to(BASE_POSE, precise=True)

            #3. Nav2 to low ramp pose
            self.go_to(RAMP_APPROACH, precise=True)

            #4. Open-loop to go up the ramp 
            if not self.go_up_ramp():
                self.get_logger().error('Aborting mission — could not go up the ramp')
                self.go_to(BASE_POSE)
                return
            
            #5. Send Nav2 Pose Estimate to init pose of Zone 4
            # self.setInitialPose(make_pose(*START_POSE))

            #6. Explore Zone 4 via reachability-checked waypoint grid 
            self.explore_zone(WAYPOINTS_ZONE_4, TIMEOUT_ZONE_4, label='ZONE_4')

            #7. Go to high ramp pose 
            self.go_to(RAMP_TOP)

            #8. Open-loop to go down the ramp 
            self.go_down_ramp()

            #9. Send Nav2 Pose Estimate down ramp pose 
            # self.setInitialPose(make_pose(*START_POSE))

            #10. Return to base (drop off)
            self.go_to(BASE_POSE, precise=True)

            #11. If time allows, go into carpet 
            #12. Return to base (drop off)

            self.get_logger().info('MISSION COMPLETE')

        except MissionAbortException:
            self.get_logger().info('Abort triggered – returning to base')
            self.go_to(BASE_POSE)


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
