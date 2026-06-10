"""
mission_base — core mission infrastructure shared by all mission runners.

This is the BasicNavigator-derived base class. Subclasses define their pose
constants, mix in feature modules (DuploMixin, RampMixin, ButtonMixin), and
override run() with their step sequence.

Layout of the mission package:
    mission_base.py    — this file: nav primitives, recovery, dropoff, lifecycle
    mission_duplo.py   — DuploMixin: visual-servo handoff, sweep, explore_zone
    mission_ramp.py    — RampMixin: ramp climb sequence (DO uses)
    mission_button.py  — ButtonMixin: button-press + door probe (DA uses)
    do_mission.py      — DoMissionRunner(DuploMixin, RampMixin, MissionBase)
    da_mission.py      — DaMissionRunner(DuploMixin, ButtonMixin, MissionBase)

What MissionBase provides:
  • Primitives
      _select_goal_checker, go_to (with auto-recovery streak detection),
      _open_loop_drive, _open_loop_rotate, _is_reachable
  • Nav2 recovery
      _run_recovery (clear costmaps + backup via behavior_server)
      _clear_costmaps (light: clear only)
      _go_to_with_recovery (critical-leg cascade, multi-attempt with recovery)
  • Common dropoff (uses subclass BASE_POSE / DROPFF_FIRST_WAYPOINT)
  • Mission lifecycle
      main_loop (waits for /mission_command 'start', restarts on 'reset',
                  preserves mission_start across resets)
      _supervisor (aborts when ≤ MISSION_CLOSING_TIME remains)
      _on_mission_command

Anything mission-specific (ramp climb, button press) lives in its own mixin
file. Subclasses pull in only the mixins they need.
"""

import math
import time
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
#  CONSTANTS (apply to all missions; override in subclass class-attrs if needed)
# ──────────────────────────────────────────────────────────────────────────────

# Per-goal nav
NAV_GOAL_TIMEOUT_S = 60.0
PLAN_TIMEOUT_S     = 8.0

# Nav2 recovery (clear costmaps + back up via behavior_server)
RECOVERY_FAILURE_STREAK   = 3      # consecutive go_to failures → auto recovery
RECOVERY_BACKUP_DIST_M    = 0.15
RECOVERY_BACKUP_SPEED_M_S = 0.10
RECOVERY_TIMEOUT_S        = 10.0
CRITICAL_NAV_ATTEMPTS     = 4      # max retries for go_to_with_recovery

# Stuck-detection: while a Nav2 goal is supposedly active, track AMCL pose.
# If the robot hasn't moved more than STUCK_MOTION_M in STUCK_TIMEOUT_S, the
# robot is stuck (BT churn, costmap blocked by duplos visible in scan, etc.).
# Cancel the goal + run a stronger recovery (clear costmaps + backup + spin)
# and return failure so the caller can move on.
STUCK_TIMEOUT_S    = 8.0
STUCK_MOTION_M     = 0.05      # m; below this counts as "didn't move"
STUCK_SPIN_RAD     = 1.0       # rad spin during deblock to rebuild local costmap

# Mission lifecycle
MISSION_TIMEOUT       = 600.0   # total mission budget (seconds)
MISSION_CLOSING_TIME  =  60.0   # supervisor aborts when ≤ this remains
MISSION_COMMAND_TOPIC = '/mission_command'

# twist_mux topics (see config/twist_mux.yaml)
RAMP_VEL_TOPIC = 'ramp_vel'   # priority 150 (above duplo=100, nav=10)


# ──────────────────────────────────────────────────────────────────────────────
#  HELPERS (module-level)
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


class MissionAbortException(Exception):
    """Raised when abort_event is set mid-mission (supervisor timeout or
    /mission_command reset). main_loop catches; subclasses can also catch in
    run() for cleanup before re-raise."""
    pass


# ──────────────────────────────────────────────────────────────────────────────
#  MISSION BASE
# ──────────────────────────────────────────────────────────────────────────────

class MissionBase(BasicNavigator):
    """Base class for mission runners. Subclasses set these class attributes
    and override run()."""

    # ── Subclass MUST override ──
    BASE_POSE: tuple              = None   # (x, y, yaw) — dropoff reverses from here
    START_POSE: tuple             = None   # (x, y, yaw) — setInitialPose target
    DROPFF_FIRST_WAYPOINT: tuple  = None   # alignment waypoint before BASE_POSE
    DROPOFF_SPEED                 = -0.3   # m/s, negative = reverse
    DROPOFF_TIME                  = 3.0    # s

    def __init__(self, node_name: str = 'mission_runner') -> None:
        super().__init__(node_name)

        # Open-loop velocity publisher (ramp_vel, priority 150 in twist_mux).
        self._ramp_pub = self.create_publisher(Twist, RAMP_VEL_TOPIC, 10)

        # Goal-checker selector — latched, transient_local so the BT picks up
        # the latest choice even if it subscribed late.
        gc_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
        self._gc_pub = self.create_publisher(String, 'goal_checker_selector', gc_qos)

        # Reachability checks via Nav2's planner.
        self._plan_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        # Watchdog state.
        self.abort_event = threading.Event()
        self.mission_start = 0.0
        self._consecutive_nav_failures = 0

        # External control: /mission_command.
        # mission_status: 'waiting' → 'running' → 'complete'.
        self._mission_status = 'waiting'
        self._start_event = threading.Event()
        self._restart_event = threading.Event()
        self.create_subscription(
            String, MISSION_COMMAND_TOPIC, self._on_mission_command, 10)

        # Step tracker for resume-on-RESET. Subclasses define their step list
        # via _build_steps(); run() iterates from _next_step. A step that
        # completes increments _next_step. A step that raises MissionAbort
        # leaves _next_step unchanged so the next call resumes from there.
        self._next_step = 0
        self._current_step_name = None

    # ── External control / lifecycle ────────────────────────────────────────

    def _supervisor(self):
        """Fires abort_event when close to MISSION_TIMEOUT. Runs forever so it
        keeps watching across resets — mission_start is preserved on reset."""
        fired = False
        while rclpy.ok():
            elapsed = time.time() - self.mission_start
            if MISSION_TIMEOUT - elapsed <= MISSION_CLOSING_TIME:
                if not fired:
                    self.get_logger().warn(
                        f'MISSION ABORT – only ≤{MISSION_CLOSING_TIME}s left')
                    fired = True
                self.abort_event.set()
                try:
                    self.cancelTask()
                except Exception:
                    pass
                self._ramp_pub.publish(Twist())
            time.sleep(0.2)

    def _on_mission_command(self, msg: String):
        """Handle /mission_command. 'start' kicks off in 'waiting'; 'reset'
        aborts current iteration and re-runs from the beginning in 'running'."""
        cmd = (msg.data or '').strip().lower()
        if cmd == 'start':
            if self._mission_status == 'waiting':
                self._start_event.set()
                self.get_logger().info("mission_command: START accepted")
            else:
                self.get_logger().warn(
                    f"mission_command: START ignored (status={self._mission_status})")
        elif cmd == 'reset':
            if self._mission_status == 'running':
                self._restart_event.set()
                self.abort_event.set()
                try:
                    self.cancelTask()
                except Exception:
                    pass
                self._ramp_pub.publish(Twist())
                self.get_logger().warn("mission_command: RESET accepted")
            else:
                self.get_logger().warn(
                    f"mission_command: RESET ignored (status={self._mission_status})")
        else:
            self.get_logger().warn(f"mission_command: unknown '{cmd}'")

    # ── Primitives ──────────────────────────────────────────────────────────

    def _select_goal_checker(self, name: str) -> None:
        self._gc_pub.publish(String(data=name))
        time.sleep(0.1)

    def go_to(self, pose_tuple, timeout_s: float = NAV_GOAL_TIMEOUT_S,
              precise: bool = False) -> bool:
        """Navigate to (x, y, yaw). Returns True on success. Tracks consecutive
        failures and triggers auto-recovery after RECOVERY_FAILURE_STREAK."""
        if self.abort_event.is_set():
            raise MissionAbortException()

        self._select_goal_checker('precise_goal_checker' if precise else 'general_goal_checker')
        self.goToPose(make_pose(*pose_tuple))

        t0 = time.time()
        # Stuck-detection state: track the last position we saw motion at.
        last_pos = self._current_xy()
        last_motion_t = t0
        stuck = False
        ok = False
        while not self.isTaskComplete():
            now = time.time()
            if now - t0 > timeout_s:
                self.cancelTask()
                self.get_logger().warn(f'go_to timeout — {pose_tuple}')
                break
            # Stuck check: did the robot move since last sample?
            cur_pos = self._current_xy()
            if cur_pos is not None:
                if last_pos is None:
                    last_pos = cur_pos
                    last_motion_t = now
                else:
                    dx = cur_pos[0] - last_pos[0]
                    dy = cur_pos[1] - last_pos[1]
                    if (dx * dx + dy * dy) ** 0.5 > STUCK_MOTION_M:
                        last_pos = cur_pos
                        last_motion_t = now
                    elif now - last_motion_t > STUCK_TIMEOUT_S:
                        self.get_logger().warn(
                            f'go_to: robot stuck (<{STUCK_MOTION_M}m) for '
                            f'{STUCK_TIMEOUT_S}s — deblocking')
                        stuck = True
                        break
        else:
            ok = self.getResult() == TaskResult.SUCCEEDED
            if not ok:
                self.get_logger().warn(f'go_to {self.getResult().name} — {pose_tuple}')

        if stuck:
            self._deblock('go_to-deblock')
            ok = False

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
        """Ask Nav2's planner if there's a path to pose_tuple from the current pose.
        Returns True iff a non-empty path comes back within timeout_s."""
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

    # ── Nav2 recovery ───────────────────────────────────────────────────────

    def _run_recovery(self, label: str = 'recovery') -> None:
        """Best-effort Nav2 recovery: clear both costmaps + small backup via the
        behavior server. Each step is wrapped in try/except."""
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

        time.sleep(0.5)

    def _clear_costmaps(self, label: str = 'cleanup') -> None:
        """Light recovery: clear costmaps only. ~0.2s overhead, no backup."""
        if self.abort_event.is_set():
            return
        try:
            self.clearAllCostmaps()
        except Exception as e:
            self.get_logger().warn(f'{label}: clearAllCostmaps failed ({e})')

    def _current_xy(self):
        """Best-effort current robot XY from BasicNavigator's _amcl_pose cache.
        Returns None if AMCL hasn't published yet."""
        pose = getattr(self, '_amcl_pose', None)
        if pose is None:
            return None
        return (pose.pose.pose.position.x, pose.pose.pose.position.y)

    def _deblock(self, label: str = 'deblock') -> None:
        """Heavier recovery used when stuck-detection fires: cancel active task,
        clear costmaps, backup, then spin to rebuild the local costmap from a
        new angle. Best-effort, all wrapped in try/except."""
        try:
            self.cancelTask()
        except Exception:
            pass
        self._run_recovery(label)
        if self.abort_event.is_set():
            return
        try:
            self.get_logger().info(f'{label}: spin {STUCK_SPIN_RAD:.2f} rad')
            self.spin(spin_dist=STUCK_SPIN_RAD,
                      time_allowance=int(RECOVERY_TIMEOUT_S))
            t0 = time.time()
            while not self.isTaskComplete():
                if time.time() - t0 > RECOVERY_TIMEOUT_S:
                    self.cancelTask()
                    break
        except Exception as e:
            self.get_logger().warn(f'{label}: spin failed ({e})')

    def _go_to_with_recovery(self, pose_tuple, label: str,
                             max_attempts: int = CRITICAL_NAV_ATTEMPTS,
                             timeout_s: float = NAV_GOAL_TIMEOUT_S,
                             precise: bool = False) -> bool:
        """Critical-leg nav: try up to max_attempts times, full recovery between
        each. For places we MUST succeed (ramp approach, dropoff base pose)."""
        for attempt in range(1, max_attempts + 1):
            if self.abort_event.is_set():
                return False
            if self.go_to(pose_tuple, timeout_s=timeout_s, precise=precise):
                return True
            if attempt < max_attempts:
                self.get_logger().warn(
                    f'{label}: attempt {attempt}/{max_attempts} failed — recovery + retry')
                self._run_recovery(f'{label}-r{attempt}')
            else:
                self.get_logger().error(
                    f'{label}: all {max_attempts} attempts failed')
        return False

    # ── Dropoff (uses subclass BASE_POSE / DROPFF_FIRST_WAYPOINT) ───────────

    def dropoff(self) -> None:
        """Critical: this is the deposit. Cascade for robustness:
          1. Alignment waypoint with recovery+retry.
          2. BASE_POSE with recovery+retry.
          3. Always perform the open-loop reverse — the deposit motion itself.
        """
        if self.DROPFF_FIRST_WAYPOINT is None or self.BASE_POSE is None:
            raise RuntimeError('subclass must set DROPFF_FIRST_WAYPOINT and BASE_POSE')

        # Align waypoint — non-critical, but a clean approach helps BASE_POSE succeed.
        if not self.go_to(self.DROPFF_FIRST_WAYPOINT, precise=False, timeout_s=20):
            self.get_logger().warn('dropoff: DROPFF_FIRST_WAYPOINT failed — running recovery')
            self._run_recovery('dropoff-align-recovery')
            self.go_to(self.DROPFF_FIRST_WAYPOINT, precise=False, timeout_s=20)

        # Base pose with recovery+retry.
        base_ok = self.go_to(self.BASE_POSE, precise=False, timeout_s=20)
        if not base_ok:
            self.get_logger().warn('dropoff: BASE_POSE attempt 1 failed — running recovery')
            self._run_recovery('dropoff-recovery')
            base_ok = self.go_to(self.BASE_POSE, precise=False, timeout_s=20)

        if not base_ok:
            self.get_logger().error(
                'dropoff: Nav2 could not reach BASE_POSE after recovery — '
                'performing reverse from current pose (deposit may be incomplete)')

        # ALWAYS reverse — this is the actual deposit motion.
        self._open_loop_drive(self.DROPOFF_SPEED, self.DROPOFF_TIME)

    # ── Mission lifecycle ───────────────────────────────────────────────────

    def main_loop(self) -> None:
        """Outer mission entry. Blocks until /mission_command 'start' is received,
        then runs the mission. On 'reset', re-runs from the beginning while
        preserving mission_start so MISSION_TIMEOUT keeps counting. Exits on
        natural completion, on supervisor abort (out of time), or when not
        enough budget remains to start another attempt."""
        self.get_logger().info(
            f"mission waiting for '{MISSION_COMMAND_TOPIC}' = 'start'")
        while rclpy.ok() and not self._start_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not rclpy.ok():
            return

        # Start the mission clock — only here, never reset.
        self.mission_start = time.time()
        threading.Thread(target=self._supervisor, daemon=True).start()
        self.get_logger().info(
            f'mission clock started, budget {MISSION_TIMEOUT:.0f}s')

        while rclpy.ok():
            elapsed = time.time() - self.mission_start
            remaining = MISSION_TIMEOUT - elapsed
            if remaining <= MISSION_CLOSING_TIME:
                self.get_logger().warn(
                    f'only {remaining:.0f}s left ≤ {MISSION_CLOSING_TIME}s '
                    f'closing window — exiting main_loop')
                break

            self._restart_event.clear()
            self.abort_event.clear()
            self._mission_status = 'running'

            try:
                self.run()
            except MissionAbortException:
                pass

            if not self._restart_event.is_set():
                # Natural completion OR supervisor abort — done either way.
                break
            self.get_logger().info(
                f'RESET → resuming at step {self._next_step + 1} '
                f'({elapsed:.0f}s elapsed, {remaining:.0f}s left)')

        self._mission_status = 'complete'
        self._ramp_pub.publish(Twist())  # final stop

    # ── Step-list mission driver ────────────────────────────────────────────

    def _build_steps(self):
        """Subclass returns a list of (step_name: str, callable) tuples.

        A step is just a thunk — it does whatever (nav, open-loop, exploration).
        A step that completes normally → _next_step advances → on RESET we
        won't redo it. A step that raises MissionAbortException → _next_step
        stays put → on RESET we re-try it.

        Step callables MAY catch exceptions internally if they want to be
        marked complete despite a failure (e.g. dropoff always returns and the
        reverse is always performed). They MAY raise MissionAbortException to
        signal a hard abort that should resume on RESET.
        """
        raise NotImplementedError('Subclass must override _build_steps()')

    def run(self) -> None:
        """Drive the mission by iterating over _build_steps() from _next_step.

        Resumable: if RESET fires mid-step, MissionAbortException propagates up,
        main_loop calls us again, and we pick up at the same step. Operator is
        responsible for putting the robot in a sane state before resetting
        (e.g. physically returning it to START_POSE if SETUP step needs to re-
        seed AMCL)."""
        steps = self._build_steps()
        n = len(steps)
        try:
            while self._next_step < n:
                if self.abort_event.is_set():
                    raise MissionAbortException()
                name, fn = steps[self._next_step]
                self._current_step_name = name
                self.get_logger().info(
                    f'═══ STEP {self._next_step + 1}/{n}: {name} ═══')
                fn()
                self._next_step += 1
                self._current_step_name = None
            self.get_logger().info('═══ MISSION COMPLETE ═══')
        except MissionAbortException:
            # Step did NOT complete. _next_step is unchanged → resume on RESET.
            if self._restart_event.is_set():
                # Operator-initiated reset; main_loop will re-call run().
                self.get_logger().warn(
                    f'step {self._next_step + 1}/{n} '
                    f'({self._current_step_name!r}) interrupted by RESET — '
                    f'will resume here')
            else:
                # Supervisor time-up; best-effort return to base.
                self.get_logger().info(
                    f'step {self._next_step + 1}/{n} '
                    f'({self._current_step_name!r}) interrupted by time abort — '
                    f'attempting return to base')
                if self.BASE_POSE is not None:
                    try:
                        self.go_to(self.BASE_POSE)
                    except MissionAbortException:
                        pass
            raise
