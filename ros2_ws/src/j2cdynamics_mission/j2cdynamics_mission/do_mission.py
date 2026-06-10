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

from geometry_msgs.msg import PoseStamped, Twist, PoseArray
from std_msgs.msg import String
from nav2_msgs.action import ComputePathToPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ──────────────────────────────────────────────────────────────────────────────
#  POSES & PATHS  
# ──────────────────────────────────────────────────────────────────────────────

BASE_POSE  = (0.35, 0.3, -1.57)  

DROPFF_FIRST_WAYPOINT = (1.5, 0.3, 3.14)
DROPOFF_POSE = (0.2, 0.2, 3.14)

START_POSE = (1.25, 0.4, 0.02)  
#START_POSE = (8.38, 5.88, 1.50)

RAMP_APPROACH = (8.0, 4, 0.02)   
RAMP_TOP      = (8.22, 5.88, 1.50) 
RAMP_EXIT     = (8.48, 5.47, -1.50)

RAMP_BACKOFF_TIME    = 1.5            
RAMP_ROTATE_SPEED    = 1.0           # rad/s during the 180° spin
RAMP_ROTATE_TIME     = math.pi / RAMP_ROTATE_SPEED   # = π s for 180°
RAMP_SPEED = 0.35 # Speed to go up the ramp
RAMP_TIME = 6.5   # s - Time to go up the ramp

RAMP_DOWN_SPEED = 0.1 # Speed for going down the ramp
RAMP_DOWN_TIME = 10 # s - Time to go down the ramp

WAYPOINTS_ZONE_4  = '/maps/arena/waypoints_zone4.yaml'
WAYPOINTS_ZONE_1  = '/maps/arena/waypoints_zone1_do.yaml'

TIMEOUT_ZONE_4 = 240.0
TIMEOUT_ZONE_1 = 200.0

MISSION_TIMEOUT = 600.0
MISSION_CLOSING_TIME = 60.0

DROPOFF_SPEED = -0.3
DROPOFF_TIME = 2 # s

NAV_GOAL_TIMEOUT_S = 60.0
PLAN_TIMEOUT_S     = 8.0
MAX_NODE_RETRIES   = 2

DUPLO_COUNT_ZONE_4 = 6

# ── scan-and-stop parameters ──
SCAN_STEP_RAD       = 0.4        # ~23° per step
SCAN_STEP_RATE      = 0.4        # rad/s while moving
SCAN_DWELL_S        = 0.6        # stand still and look between steps
SCAN_FRESH_FRAMES   = 2          # require N consecutive detections to commit

# ── duplo collection parameters ──
DUPLO_OVERSHOOT_M           = 0.40   # how far past the duplo to aim (trailer reach)
DUPLO_NAV_TIMEOUT_S         = 20.0
DUPLO_MAX_PER_SCAN          = 4      # safety cap to avoid infinite loops
DUPLO_COARSE_STANDOFF_M     = 0.8    # land this far short, then re-read for refinement
DUPLO_REFINE_RADIUS_M       = 0.4    # accept refinement within this of original
DUPLO_REFINE_DWELL_S        = 0.7    # let detector get clean frames after coarse approach
COLLECTED_BLACKLIST_RADIUS_M = 0.4   # ignore future detections within this of collected

COARSE_STANDOFF_M   = 0.6    # land this far short on stage 1 (generic, unused for now)
FINE_TIMEOUT_S      = 20.0
COARSE_TIMEOUT_S    = 25.0

RAMP_VEL_TOPIC = 'ramp_vel'   # must match twist_mux.yaml at priority 150


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

        # duplo perception
        self._duplo_map_cache = None
        self.create_subscription(
            PoseArray, '/duplo_map', self._on_duplo_map, 10
        )

        # persistent blacklist of collected (or attempted) duplos — survives across waypoints
        self._collected_blacklist = []   # [(x, y), ...]

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
        """
        Approach pose is assumed reached. Sequence: 
        1. Rotate 180°
        2. Reverse from button
        3. Rotate toward ramp
        4. Drive up
        """
        self.get_logger().info('Ramp: rotate 180°')
        self._open_loop_rotate(RAMP_ROTATE_SPEED, RAMP_ROTATE_TIME)

        self.get_logger().info('Ramp: moving back from button')
        self._open_loop_drive(-RAMP_SPEED, RAMP_BACKOFF_TIME)

        self.get_logger().info('Ramp: rotate towards ramp')
        self._open_loop_rotate(-RAMP_ROTATE_SPEED / 1.5, RAMP_ROTATE_TIME)

        self.get_logger().info('Ramp: attempt to climb the ramp')
        self._open_loop_drive(RAMP_SPEED, RAMP_TIME)

        return True

    def go_down_ramp(self) -> bool: 
        pass


    # ── waypoint-grid exploration ─────────────────────────────────────────────

    def explore_zone(self, waypoints_file: str, duration_s: float,
                     label: str = 'ZONE') -> None:
        """
        Sparse waypoint exploration + scan-and-stop + duplo collection.

        Visits waypoints strictly in YAML order. At each waypoint, performs a
        stepped scan that pauses briefly between steps to let the detector get
        clean frames. As soon as a duplo is detected, stops scanning and goes
        to collect.
        """
        with open(waypoints_file) as f:
            grid = list(yaml.safe_load(f)['waypoints'])

        failed = {}
        deadline = time.time() + duration_s
        idx = 0

        self.get_logger().info(f'{label}: {len(grid)} waypoints, budget {duration_s:.0f}s')

        self._select_goal_checker('general_goal_checker')

        while rclpy.ok() and time.time() < deadline and idx < len(grid):
            wp = grid[idx]
            k = node_key(wp)

            # ── Reachability gate ──
            if not self._is_reachable(wp):
                failed[k] = failed.get(k, 0) + 1
                self.get_logger().warn(
                    f'{label}: {k} unreachable ({failed[k]}/{MAX_NODE_RETRIES})')
                if failed[k] >= MAX_NODE_RETRIES:
                    self.get_logger().warn(
                        f'{label}: skipping {k} after {MAX_NODE_RETRIES} attempts')
                    idx += 1
                continue

            # ── Navigate to waypoint ──
            time_left = deadline - time.time()
            if time_left <= 0:
                break

            ok = self.go_to(wp, timeout_s=min(NAV_GOAL_TIMEOUT_S, time_left), precise=False)

            if not ok:
                failed[k] = failed.get(k, 0) + 1
                self.get_logger().warn(
                    f'{label}: {k} nav failed ({failed[k]}/{MAX_NODE_RETRIES})')
                if failed[k] >= MAX_NODE_RETRIES:
                    self.get_logger().warn(
                        f'{label}: skipping {k} after {MAX_NODE_RETRIES} attempts')
                    idx += 1
                continue

            idx += 1

            # ── Scan phase: scan-and-stop until duplo or full revolution ──
            self.get_logger().info(f"{label}: scanning at {k}")
            found = self._scan_rotate_until_duplo()

            if found:
                duplos = self._read_duplos()
                self.get_logger().info(
                    f"{label}: detected {len(duplos)} duplos mid-scan → collecting")
                n = self._collect_visible_duplos(label)
                self.get_logger().info(f'{label}: collected {n} duplos this scan')
                # don't advance idx — re-scan from current pose next iteration?
                # current behavior: move to next waypoint. Change if you want re-scan.

        self.get_logger().info(
            f'{label}: ended  visited={idx} / {len(grid)}  '
            f'time_left={max(0.0, deadline - time.time()):.0f}s')

    # ── scanning ──────────────────────────────────────────────────────────────

    def _scan_rotate_until_duplo(self, total_yaw: float = 2 * math.pi) -> bool:
        """
        Rotate in small steps with dwell periods. Polls /duplo_map during dwell.
        Returns True as soon as a duplo is seen on >= SCAN_FRESH_FRAMES consecutive
        polls (debounced to reject flicker). Returns False if a full revolution
        completes without detection.
        """
        step_time = SCAN_STEP_RAD / SCAN_STEP_RATE
        n_steps = max(1, int(math.ceil(total_yaw / SCAN_STEP_RAD)))

        # require fresh data — drop any cache from before the scan started
        self._duplo_map_cache = None

        consecutive_hits = 0
        for i in range(n_steps):
            if self.abort_event.is_set():
                return False

            # rotate one step
            self._open_loop_rotate(SCAN_STEP_RATE, step_time)

            # dwell: stand still and let the detector get clean frames
            t_end = time.time() + SCAN_DWELL_S
            while time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self._read_duplos():
                    consecutive_hits += 1
                    if consecutive_hits >= SCAN_FRESH_FRAMES:
                        self.get_logger().info(
                            f'scan: duplo detected after step {i+1}/{n_steps}')
                        return True
                else:
                    consecutive_hits = 0

        return False

    # ── Duplos ────────────────────────────────────────────────────────────────

    def _on_duplo_map(self, msg: PoseArray) -> None:
        self._duplo_map_cache = msg

    def _is_blacklisted(self, xy) -> bool:
        x, y = xy
        return any(math.hypot(x - bx, y - by) < COLLECTED_BLACKLIST_RADIUS_M
                   for bx, by in self._collected_blacklist)

    def _read_duplos(self):
        """
        Returns currently-tracked duplos in map frame, with the persistent
        collected-blacklist filtered out.
        """
        msg = getattr(self, '_duplo_map_cache', None)
        if msg is None:
            return []
        raw = [(p.position.x, p.position.y) for p in msg.poses]
        return [d for d in raw if not self._is_blacklisted(d)]

    def _drive_through_duplo(self, duplo_xy) -> bool:
        """
        Single-shot Nav2: goal placed OVERSHOOT_M past the duplo along the
        approach vector from the current pose, yawed toward the duplo. The
        trailer drags through the duplo as the wheelbase passes over it.
        """
        dx, dy = duplo_xy
        rx, ry = self._current_xy()

        vx, vy = dx - rx, dy - ry
        dist = math.hypot(vx, vy)
        if dist < 1e-3:
            self.get_logger().warn(f'duplo at current pose? {duplo_xy} — skipping')
            return False

        ux, uy = vx / dist, vy / dist
        yaw = math.atan2(vy, vx)
        goal = (dx + DUPLO_OVERSHOOT_M * ux,
                dy + DUPLO_OVERSHOOT_M * uy,
                yaw)

        if not self._is_reachable(goal):
            self.get_logger().warn(
                f'duplo at {duplo_xy} unreachable (goal {goal})')
            return False

        self.get_logger().info(f'driving through duplo at {duplo_xy} via goal {goal}')
        return self.go_to(goal, timeout_s=DUPLO_NAV_TIMEOUT_S, precise=False)

    def _go_collect_duplo(self, duplo_xy) -> bool:
        """
        Two-stage collection to mitigate parallax error on far targets:
          1. Coarse approach to DUPLO_COARSE_STANDOFF_M short of the duplo.
          2. Re-read /duplo_map for a sharper position estimate.
          3. Drive through the (possibly refined) duplo position.
        Short-range targets skip stage 1 and go direct.
        """
        dx, dy = duplo_xy
        rx, ry = self._current_xy()
        dist = math.hypot(dx - rx, dy - ry)

        # Short-range: just go
        if dist < DUPLO_COARSE_STANDOFF_M + 0.2:
            return self._drive_through_duplo(duplo_xy)

        # Stage 1: coarse approach to standoff
        ux, uy = (dx - rx) / dist, (dy - ry) / dist
        yaw = math.atan2(dy - ry, dx - rx)
        coarse = (dx - DUPLO_COARSE_STANDOFF_M * ux,
                  dy - DUPLO_COARSE_STANDOFF_M * uy,
                  yaw)

        if not self._is_reachable(coarse):
            self.get_logger().warn(
                f'duplo {duplo_xy}: coarse pose unreachable, going direct')
            return self._drive_through_duplo(duplo_xy)

        self.get_logger().info(f'duplo {duplo_xy}: coarse stage to {coarse}')
        if not self.go_to(coarse, timeout_s=DUPLO_NAV_TIMEOUT_S, precise=False):
            return False

        # Refinement window: let detector produce clean frames from the new vantage
        self._duplo_map_cache = None
        t_end = time.time() + DUPLO_REFINE_DWELL_S
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)

        # Snap to nearest fresh detection within radius of the original target
        fresh = self._read_duplos()
        refined = duplo_xy
        if fresh:
            candidates = [(d, math.hypot(d[0] - dx, d[1] - dy)) for d in fresh]
            candidates = [(d, e) for d, e in candidates if e < DUPLO_REFINE_RADIUS_M]
            if candidates:
                refined, err = min(candidates, key=lambda x: x[1])
                self.get_logger().info(
                    f'duplo refined {duplo_xy} -> {refined} (Δ {err:.2f}m)')
            else:
                self.get_logger().info(
                    f'duplo {duplo_xy}: no refinement within {DUPLO_REFINE_RADIUS_M}m, '
                    f'using original')

        # Stage 2: drive through the (possibly refined) position
        return self._drive_through_duplo(refined)

    def _collect_visible_duplos(self, label: str) -> int:
        """
        Greedy nearest-first collection loop. Returns count attempted (succeeded
        or otherwise marked off). Re-reads duplo_map after each pickup so the
        next pick reflects reality, and uses the persistent blacklist to skip
        already-collected positions.
        """
        collected = 0

        for _ in range(DUPLO_MAX_PER_SCAN):
            if self.abort_event.is_set():
                break

            duplos = self._read_duplos()   # blacklist filtering happens inside
            if not duplos:
                break

            rx, ry = self._current_xy()
            target = min(duplos, key=lambda d: math.hypot(d[0] - rx, d[1] - ry))

            if self._go_collect_duplo(target):
                collected += 1
                self._collected_blacklist.append(target)
                self.get_logger().info(
                    f'{label}: collected duplo #{collected} at {target} '
                    f'(blacklist size {len(self._collected_blacklist)})')
            else:
                # Blacklist failed attempts too, otherwise we'd loop forever
                # on an unreachable/missed duplo.
                self._collected_blacklist.append(target)
                self.get_logger().warn(
                    f'{label}: failed at {target} — blacklisted')

        return collected

    def _current_xy(self):
        """
        Best-effort current robot XY from AMCL via BasicNavigator's _amcl_pose
        cache. Falls back to (0,0) if unset.
        """
        pose = getattr(self, '_amcl_pose', None)
        if pose is None:
            return (0.0, 0.0)
        return (pose.pose.pose.position.x, pose.pose.pose.position.y)


    def dropoff(self): 
        # Go to safe waypoint to align correctly
        self.go_to(DROPFF_FIRST_WAYPOINT, precise=True)

        # Go to base position
        self.go_to(BASE_POSE, precise=True)
        
        # Go backwards for a bit
        self._open_loop_drive(DROPOFF_SPEED, DROPOFF_TIME)
    
    # ── mission ───────────────────────────────────────────────────────────────

    def run(self) -> None:
        self.setInitialPose(make_pose(*START_POSE))
        self.waitUntilNav2Active(localizer='amcl')
        
        # Watchdog for end-of-mission time handling
        self.mission_start = time.time()
        self.abort_event.clear()
        threading.Thread(target=self._supervisor, daemon=True).start()

        self.get_logger().info('Nav2 active — waiting for AMCL pose')
        t_deadline = time.time() + 5.0
        while time.time() < t_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if getattr(self, '_amcl_pose', None) is not None:
                break

        # extra half-second for costmap settle
        time.sleep(0.5)

        try:
            #1. Explore Zone 1 via reachability-checked waypoint grid 
            self.explore_zone(WAYPOINTS_ZONE_1, TIMEOUT_ZONE_1, label='ZONE_1')

            #2. Return to base (drop off)
            self.dropoff()

            #3. Nav2 to low ramp pose
            if not self.go_to(RAMP_APPROACH, precise=False):
                self.get_logger().error(f'RAMP_APPROACH {RAMP_APPROACH} failed')
                raise MissionAbortException()
            else:
                #4. Open-loop to go up the ramp
                self.go_up_ramp()

                #5. Send Nav2 Pose Estimate to init pose of Zone 4
                self.setInitialPose(make_pose(*RAMP_TOP))

                #6. Explore Zone 4 via reachability-checked waypoint grid 
                self.explore_zone(WAYPOINTS_ZONE_4, TIMEOUT_ZONE_4, label='ZONE_4')

                #7. Go to high ramp pose 
                self.go_to(RAMP_EXIT, precise=True)

            #8. Open-loop to go down the ramp 
            #self.go_down_ramp()

            #9. Send Nav2 Pose Estimate down ramp pose 
            # self.setInitialPose(make_pose(*START_POSE))

            #10. Return to base (drop off)
            #self.dropoff()

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