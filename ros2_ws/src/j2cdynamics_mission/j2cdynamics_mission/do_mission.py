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

import json
import math
import time
import yaml
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import (QoSProfile, DurabilityPolicy,
                       ReliabilityPolicy, HistoryPolicy)

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
from nav2_msgs.action import ComputePathToPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ──────────────────────────────────────────────────────────────────────────────
#  POSES & PATHS  
# ──────────────────────────────────────────────────────────────────────────────

BASE_POSE  = (0.35, 0.3, -1.57)  

DROPFF_FIRST_WAYPOINT = (1.5, 0.3, 3.14)
DROPOFF_POSE = (0.05, 0.4, 3.14)

START_POSE = (1.25, 0.4, 0.02)  
#START_POSE = (8.38, 5.88, 1.50)

RAMP_APPROACH = (8.0, 4, 0.02)   
RAMP_TOP      = (8.22, 5.88, 1.50) 
RAMP_EXIT     = (8.0, 6.0, -1.50)
RAMP_DOWN_SPEED = 0.1
RAMP_DOWN_TIME = 10.0

RAMP_BACKOFF_TIME    = 1.5            
RAMP_ROTATE_SPEED    = 1.0           # rad/s during the 180° spin
RAMP_ROTATE_TIME     = math.pi / RAMP_ROTATE_SPEED   # = π s for 180°
RAMP_SPEED = 0.35 # Speed to go up the ramp
RAMP_TIME = 6.5   # s - Time to go up the ramp

RAMP_DOWN_SPEED = 0.1 # Speed for going down the ramp
RAMP_DOWN_TIME = 12 # s - Time to go down the ramp

WAYPOINTS_ZONE_4  = '/maps/arena/waypoints_zone4.yaml'
WAYPOINTS_ZONE_1  = '/maps/arena/waypoints_zone1_do.yaml'

TIMEOUT_ZONE_4 = 240.0
TIMEOUT_ZONE_1 = 200.0

MISSION_TIMEOUT = 600.0
MISSION_CLOSING_TIME = 60.0

DROPOFF_SPEED = -0.3
DROPOFF_TIME = 3 # s

NAV_GOAL_TIMEOUT_S = 60.0
PLAN_TIMEOUT_S     = 8.0
MAX_NODE_RETRIES   = 1   # single attempt per waypoint — retries rarely succeed and double the wasted time

# ── Nav2 recovery (clear costmaps + back up) ──
# When Nav2 returns FAILED instantly, the robot is usually parked in inflation
# and the planner refuses to start. Clearing costmaps + a small backup often
# unsticks it. We trigger after a streak in regular nav, and explicitly between
# attempts on critical operations (dropoff, ramp approach).
RECOVERY_FAILURE_STREAK   = 3      # consecutive go_to failures before auto recovery
RECOVERY_BACKUP_DIST_M    = 0.15
RECOVERY_BACKUP_SPEED_M_S = 0.10
RECOVERY_TIMEOUT_S        = 10.0

DUPLO_COUNT_ZONE_4 = 6

# ── scan-and-collect parameters ──
# At each waypoint we sweep in small rotation steps via ramp_vel. After each
# step we dwell long enough for ramp_vel to time out (twist_mux: ramp=150 >
# duplo=100) so duplo_approach can take over via duplo_vel if it sees a duplo.
SCAN_STEP_RAD       = 0.4        # ~23° per step
SCAN_STEP_RATE      = 0.4        # rad/s while moving
SCAN_DWELL_S        = 1.0        # > ramp_vel timeout (0.5s) so visual servo can grab control
SCAN_MAX_FSM_CYCLES = 3          # cap approach cycles per waypoint to avoid stubborn-target loops
FSM_WAIT_TIMEOUT_S  = 12.0       # max time per cycle: successful pickup ~13s (8s approach + 5s collect);
                                 # stuck approaches now self-abort at APPROACH_HARD_TIMEOUT_S (~8s) in duplo_approach.py

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

        # Visual-servo collection handoff.
        # duplo_approach.py runs the search/approach/collect FSM purely in image
        # space (no map/TF/AMCL) and drives via duplo_vel (twist_mux priority 100).
        # We toggle it on for the sweep at each waypoint and watch /duplo_state
        # to know when the FSM is busy vs idle.
        self._enable_collection_pub = self.create_publisher(
            Bool, '/enable_duplo_collection', 10)
        self._fsm_state = 'search'   # last seen state from /duplo_state
        self.create_subscription(String, '/duplo_state', self._on_duplo_state, 10)

        # ComputePathToPose action client for reachability checks
        self._plan_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        # Watchdog for end of mission time handling
        self.abort_event = threading.Event()
        self.mission_start = 0.0

        # Auto-recovery: count consecutive go_to failures so we can run a Nav2
        # recovery (clear costmaps + back up) when a streak suggests the robot
        # is stuck in inflation rather than just failing for transient reasons.
        self._consecutive_nav_failures = 0

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
        ok = False
        while not self.isTaskComplete():
            if time.time() - t0 > timeout_s:
                self.cancelTask()
                self.get_logger().warn(f'go_to timeout — {pose_tuple}')
                break
        else:
            ok = self.getResult() == TaskResult.SUCCEEDED
            if not ok:
                self.get_logger().warn(
                    f'go_to {self.getResult().name} — {pose_tuple}')

        # Track streak and auto-run recovery when it looks like the robot is
        # stuck in inflation (planner returns FAILED instantly for everything).
        if ok:
            self._consecutive_nav_failures = 0
        else:
            self._consecutive_nav_failures += 1
            if self._consecutive_nav_failures >= RECOVERY_FAILURE_STREAK:
                self.get_logger().warn(
                    f'{self._consecutive_nav_failures} consecutive nav failures '
                    f'— running Nav2 recovery')
                self._run_recovery('auto-recovery')
                self._consecutive_nav_failures = 0
        return ok

    def _run_recovery(self, label: str = 'recovery') -> None:
        """Best-effort Nav2 recovery: clear both costmaps + small backup via the
        behavior server. Each step is wrapped in try/except so a single failure
        doesn't poison the whole sequence."""
        if self.abort_event.is_set():
            return
        try:
            self.get_logger().info(f'{label}: clearAllCostmaps')
            self.clearAllCostmaps()
        except Exception as e:
            self.get_logger().warn(f'{label}: clearAllCostmaps failed ({e})')

        if self.abort_event.is_set():
            return
        try:
            self.get_logger().info(
                f'{label}: backup {RECOVERY_BACKUP_DIST_M}m '
                f'@ {RECOVERY_BACKUP_SPEED_M_S}m/s')
            self.backup(backup_dist=RECOVERY_BACKUP_DIST_M,
                        backup_speed=RECOVERY_BACKUP_SPEED_M_S,
                        time_allowance=int(RECOVERY_TIMEOUT_S))
            t0 = time.time()
            while not self.isTaskComplete():
                if time.time() - t0 > RECOVERY_TIMEOUT_S:
                    self.cancelTask()
                    self.get_logger().warn(f'{label}: backup timed out')
                    break
        except Exception as e:
            self.get_logger().warn(f'{label}: backup failed ({e})')

        # Brief settle so cleared costmaps repopulate from /scan before next nav.
        time.sleep(0.5)

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
        self.get_logger().info('Ramp: rotate 180°')
        self._open_loop_rotate(RAMP_ROTATE_SPEED, RAMP_ROTATE_TIME)

        self.get_logger().info('Ramp: moving back from button')
        self._open_loop_drive(-RAMP_SPEED, RAMP_BACKOFF_TIME)

        self.get_logger().info('Ramp: rotate towards ramp')
        self._open_loop_rotate(+RAMP_ROTATE_SPEED / 1.5, RAMP_ROTATE_TIME)

        self.get_logger().info('Ramp: attempt to climb the ramp')
        self._open_loop_drive(RAMP_DOWN_SPEED, RAMP_DOWN_TIME)


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

            # ── Sweep + collect: visual-servo FSM does the actual fetching ──
            # Enable only for the sweep so Nav2 transit between waypoints is not
            # hijacked by an opportunistic detection.
            self.get_logger().info(f"{label}: sweeping at {k}")
            self._enable_collection(True)
            try:
                n_cycles = self._sweep_and_collect(label)
            finally:
                self._enable_collection(False)
            self.get_logger().info(
                f'{label}: {n_cycles} FSM cycle(s) at {k}')

        self.get_logger().info(
            f'{label}: ended  visited={idx} / {len(grid)}  '
            f'time_left={max(0.0, deadline - time.time()):.0f}s')

    # ── Visual-servo collection handoff ───────────────────────────────────────

    def _on_duplo_state(self, msg: String) -> None:
        try:
            self._fsm_state = json.loads(msg.data).get('state', self._fsm_state)
        except Exception:
            pass

    def _enable_collection(self, on: bool) -> None:
        self._enable_collection_pub.publish(Bool(data=bool(on)))

    def _wait_until_fsm_idle(self, timeout_s: float) -> bool:
        """Spin until /duplo_state reports 'search' (FSM idle) or timeout.
        Returns True if FSM returned to idle, False on timeout/abort."""
        t_end = time.time() + timeout_s
        while time.time() < t_end:
            if self.abort_event.is_set():
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._fsm_state == 'search':
                return True
        self.get_logger().warn(
            f'FSM stuck in "{self._fsm_state}" for {timeout_s:.0f}s — moving on')
        return False

    def _sweep_and_collect(self, label: str,
                           total_yaw: float = 2 * math.pi) -> int:
        """
        Rotate in small steps via ramp_vel. After each step we dwell long enough
        for ramp_vel to time out so duplo_approach (twist_mux priority 100) can
        take over via duplo_vel. If /duplo_state shows the FSM left 'search'
        during the dwell, we wait for it to come back before rotating again.

        Returns the number of approach cycles observed at this waypoint.
        Capped by SCAN_MAX_FSM_CYCLES to avoid stubborn-target loops.
        """
        step_time = SCAN_STEP_RAD / SCAN_STEP_RATE
        n_steps = max(1, int(math.ceil(total_yaw / SCAN_STEP_RAD)))
        cycles = 0

        for i in range(n_steps):
            if self.abort_event.is_set():
                return cycles
            if cycles >= SCAN_MAX_FSM_CYCLES:
                self.get_logger().info(
                    f'{label}: hit FSM-cycle cap ({SCAN_MAX_FSM_CYCLES}) — moving on')
                return cycles

            self._open_loop_rotate(SCAN_STEP_RATE, step_time)

            # Dwell: ramp_vel times out (~0.5s) → duplo_vel can drive.
            t_end = time.time() + SCAN_DWELL_S
            while time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=0.05)

            # If the visual servo grabbed control, wait for it to release.
            if self._fsm_state != 'search':
                self.get_logger().info(
                    f'{label}: FSM entered "{self._fsm_state}" — waiting for cycle')
                self._wait_until_fsm_idle(FSM_WAIT_TIMEOUT_S)
                cycles += 1

        return cycles

    def dropoff(self):
        """Critical: this is the deposit. Cascade for robustness:
          1. Align waypoint (best-effort) and base pose use general_goal_checker
             so tight tolerances don't cause Nav2 to refuse/abort.
          2. On BASE_POSE failure, run a Nav2 recovery (clear costmaps + backup)
             before retry — the usual cause of fast FAILEDs here is the robot
             ending up in inflation after the last leg.
          3. The final reverse is the actual deposit motion — perform it whether
             nav succeeded or not, so we get a partial deposit at worst.
        """
        # Align waypoint — recovery+retry too: failing here forces BASE_POSE to
        # approach from the wrong direction, which often cascades into a fail.
        if not self.go_to(DROPFF_FIRST_WAYPOINT, precise=False, timeout_s=20):
            self.get_logger().warn('dropoff: DROPFF_FIRST_WAYPOINT failed — running recovery')
            self._run_recovery('dropoff-align-recovery')
            self.go_to(DROPFF_FIRST_WAYPOINT, precise=False, timeout_s=20)

        # Base pose with Nav2-recovery retry
        base_ok = self.go_to(BASE_POSE, precise=False, timeout_s=20)
        if not base_ok:
            self.get_logger().warn('dropoff: BASE_POSE attempt 1 failed — running recovery')
            self._run_recovery('dropoff-recovery')
            base_ok = self.go_to(BASE_POSE, precise=False, timeout_s=20)

        if not base_ok:
            self.get_logger().error(
                'dropoff: Nav2 could not reach BASE_POSE after recovery — '
                'performing reverse from current pose (deposit may be incomplete)')

        # Always reverse — this is the actual deposit motion
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

        # duplo_approach defaults to enabled — explicitly disable so the visual
        # servo can't fire during nav-between-waypoints or ramp/dropoff. Each
        # waypoint sweep re-enables briefly via _sweep_and_collect.
        self._enable_collection(False)

        try:
            #1. Explore Zone 1 via reachability-checked waypoint grid 
            self.explore_zone(WAYPOINTS_ZONE_1, TIMEOUT_ZONE_1, label='ZONE_1')

            #2. Return to base (drop off)
            self.dropoff()

            #3. Nav2 to low ramp pose — critical, retry once via recovery
            ramp_ok = self.go_to(RAMP_APPROACH, precise=False)
            if not ramp_ok:
                self.get_logger().warn(
                    f'RAMP_APPROACH {RAMP_APPROACH} failed — running recovery and retrying')
                self._run_recovery('ramp-recovery')
                ramp_ok = self.go_to(RAMP_APPROACH, precise=False)
            if not ramp_ok:
                self.get_logger().error(f'RAMP_APPROACH {RAMP_APPROACH} failed after recovery')
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