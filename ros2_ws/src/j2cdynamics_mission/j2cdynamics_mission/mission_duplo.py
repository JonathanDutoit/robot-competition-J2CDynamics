"""
mission_duplo — visual-servo handoff, waypoint sweep, and opportunistic
mid-transit collection. Mixed into any MissionBase subclass that wants the
duplo-collection pipeline.

The hand-off is one-way: the mixin tells `duplo_approach` when it's allowed to
drive (via /enable_duplo_collection) and watches /duplo_state to know when the
FSM is idle vs busy. duplo_approach owns velocity output on `duplo_vel`
(twist_mux priority 100); this mixin never publishes velocities directly.

Usage:
    class MyMission(DuploMixin, MissionBase):
        def run(self):
            self._enable_collection(False)         # defensive at start
            self.explore_zone('/maps/.../wp.yaml', 200.0, label='ZONE_X')

What this mixin provides:
  • Constants for sweep + opportunistic timing
  • State: _fsm_state, _enable_collection_pub, _duplo_state_sub
  • Methods:
      _on_duplo_state, _enable_collection, _wait_until_fsm_idle
      _sweep_and_collect, _go_to_with_opportunistic_collect, explore_zone

Setup runs via cooperative __init__ (super() chain).
"""

import json
import math
import time
import yaml

import rclpy
from std_msgs.msg import Bool, String
from nav2_simple_commander.robot_navigator import TaskResult

from j2cdynamics_mission.mission_base import (
    NAV_GOAL_TIMEOUT_S, make_pose, node_key, MissionAbortException,
    STUCK_TIMEOUT_S, STUCK_MOTION_M,
)


# ── Sweep + visual-servo handoff ──────────────────────────────────────────────
SCAN_STEP_RAD       = 0.4     # ~23° per rotation step
SCAN_STEP_RATE      = 0.4     # rad/s while rotating
SCAN_DWELL_S        = 1.0     # > ramp_vel timeout (0.5s) so duplo_vel can take over
SCAN_MAX_FSM_CYCLES = 3       # max approach cycles per waypoint
FSM_WAIT_TIMEOUT_S  = 12.0    # max wait for a single FSM approach→collect→search cycle

# ── Opportunistic mid-nav collection ─────────────────────────────────────────
# If FSM enters 'approach' mid-transit, we cancel Nav2, let the cycle finish,
# then re-issue the same goal. Cap interruptions to avoid stuck loops on
# unreachable duplos.
OPPORTUNISTIC_MAX_INTERRUPTIONS = 2
OPPORTUNISTIC_NAV_TIMEOUT_S     = 60.0


class DuploMixin:
    """Visual-servo handoff. Mix in BEFORE MissionBase in the bases list:
        class DoMissionRunner(DuploMixin, RampMixin, MissionBase): ...
    """

    def __init__(self, *args, **kwargs):
        # Forward to MissionBase (and onward to BasicNavigator/Node) so self is
        # a fully-initialized Node before we create our pubs/subs.
        super().__init__(*args, **kwargs)

        # Tell duplo_approach when it's allowed to drive.
        self._enable_collection_pub = self.create_publisher(
            Bool, '/enable_duplo_collection', 10)
        # Last seen FSM state from /duplo_state (search | approach | collect).
        self._fsm_state = 'search'
        self.create_subscription(
            String, '/duplo_state', self._on_duplo_state, 10)

    # ── FSM state tracking ──────────────────────────────────────────────────

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

    # ── Sweep + collect at one waypoint ─────────────────────────────────────

    def _sweep_and_collect(self, label: str,
                           total_yaw: float = 2 * math.pi) -> int:
        """Rotate in small steps via ramp_vel. Between steps, dwell long enough
        for ramp_vel to time out so duplo_vel can take over. If /duplo_state
        shows the FSM left 'search', wait for it to come back before rotating
        again. Returns the number of approach cycles observed at this waypoint.
        Capped by SCAN_MAX_FSM_CYCLES."""
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

            t_end = time.time() + SCAN_DWELL_S
            while time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=0.05)

            if self._fsm_state != 'search':
                self.get_logger().info(
                    f'{label}: FSM entered "{self._fsm_state}" — waiting for cycle')
                self._wait_until_fsm_idle(FSM_WAIT_TIMEOUT_S)
                cycles += 1

        return cycles

    # ── Opportunistic transit pickup ────────────────────────────────────────

    def _go_to_with_opportunistic_collect(
            self, pose_tuple, label: str = 'transit',
            timeout_s: float = OPPORTUNISTIC_NAV_TIMEOUT_S,
            max_interruptions: int = OPPORTUNISTIC_MAX_INTERRUPTIONS,
            precise: bool = False) -> bool:
        """Like go_to but enables visual-servo en route. If FSM enters
        'approach' mid-transit (and we're under the interruption cap), cancel
        Nav2, let the FSM run a collect cycle, then re-issue the same goal.

        Caller does NOT need to wrap with _enable_collection — this method
        owns the toggle for its lifetime. Disabled in finally."""
        if self.abort_event.is_set():
            raise MissionAbortException()

        self._enable_collection(True)
        interruptions = 0
        try:
            t_overall = time.time()
            while True:
                if self.abort_event.is_set():
                    raise MissionAbortException()
                if time.time() - t_overall > timeout_s:
                    self.cancelTask()
                    self.get_logger().warn(f'{label}: overall timeout — giving up')
                    return False

                self._select_goal_checker(
                    'precise_goal_checker' if precise else 'general_goal_checker')
                self.goToPose(make_pose(*pose_tuple))

                t_attempt = time.time()
                # Stuck-detection state — reset for each (re-)issued goal.
                last_pos = self._current_xy()
                last_motion_t = t_attempt
                stuck = False
                interrupted = False
                while not self.isTaskComplete():
                    now = time.time()
                    if now - t_attempt > timeout_s:
                        self.cancelTask()
                        self.get_logger().warn(f'{label}: per-attempt timeout')
                        return False
                    # Mid-transit pickup: if visual servo grabbed control, pause nav.
                    if interruptions < max_interruptions and self._fsm_state != 'search':
                        self.get_logger().info(
                            f'{label}: FSM in "{self._fsm_state}" — pausing nav for collect')
                        self.cancelTask()
                        interrupted = True
                        break
                    # Stuck-detection
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
                                    f'{label}: robot stuck (<{STUCK_MOTION_M}m) for '
                                    f'{STUCK_TIMEOUT_S}s — deblocking')
                                stuck = True
                                break
                    rclpy.spin_once(self, timeout_sec=0.05)

                if stuck:
                    self._deblock(f'{label}-deblock')
                    return False

                if interrupted:
                    self._wait_until_fsm_idle(FSM_WAIT_TIMEOUT_S)
                    interruptions += 1
                    self.get_logger().info(
                        f'{label}: cycle done ({interruptions}/{max_interruptions}) — resuming nav')
                    # Clear costmaps so replan doesn't inherit stale marks
                    # from the duplo-approach motion.
                    self._clear_costmaps(f'{label}-resume')
                    continue

                # Natural completion.
                ok = self.getResult() == TaskResult.SUCCEEDED
                if not ok:
                    self.get_logger().warn(
                        f'{label}: go_to {self.getResult().name} — {pose_tuple}')
                if ok:
                    self._consecutive_nav_failures = 0
                else:
                    self._consecutive_nav_failures += 1
                return ok
        finally:
            self._enable_collection(False)

    # ── Generic waypoint exploration ────────────────────────────────────────

    def _waypoint_wakeup(self, label: str = 'wakeup') -> None:
        """Lighter than _deblock: clear costmaps + 90° spin to rebuild local
        view. Used at waypoint boundaries in explore_zone — Nav2 often gives
        up 'between obstacles' because its local view is stale, and a single
        spin makes the next waypoint attempt succeed."""
        self._clear_costmaps(label)
        if self.abort_event.is_set():
            return
        try:
            self.get_logger().info(f'{label}: spin 90°')
            self.spin(spin_dist=math.pi / 2, time_allowance=6)
            t0 = time.time()
            while not self.isTaskComplete():
                if time.time() - t0 > 6:
                    self.cancelTask()
                    break
        except Exception as e:
            self.get_logger().warn(f'{label}: spin failed ({e})')

    def explore_zone(self, waypoints_file: str, duration_s: float,
                 label: str = 'ZONE',
                 opportunistic_collect: bool = False,
                 max_node_retries: int = 1,
                 stop_condition = None
                 ) -> bool:
        """Sparse waypoint exploration with per-waypoint sweep + collect.

        Args:
        stop_condition: optional callable() -> bool. If provided and returns True,
            exploration ends early. Checked at the top of each waypoint iteration
            AND immediately after each sweep, so a condition that becomes true
            mid-sweep is caught at the next boundary.

        Returns:
        True  — stopped early because stop_condition fired.
        False — natural completion (deadline elapsed, all waypoints visited, or
                all remaining unreachable).
        """
        with open(waypoints_file) as f:
            grid = list(yaml.safe_load(f)['waypoints'])

        failed = {}
        deadline = time.time() + duration_s
        idx = 0

        def _stop():
            """Wrap so a bad lambda can't kill the explorer."""
            if stop_condition is None:
                return False
            try:
                return bool(stop_condition())
            except Exception as e:
                self.get_logger().warn(f'{label}: stop_condition raised ({e}); ignoring')
                return False

        self.get_logger().info(f'{label}: {len(grid)} waypoints, budget {duration_s:.0f}s')
        self._select_goal_checker('general_goal_checker')

        while rclpy.ok() and time.time() < deadline and idx < len(grid):
            # Early-stop check — before reachability / nav / anything.
            if _stop():
                self.get_logger().info(
                    f'{label}: stop_condition met before waypoint {idx + 1}/{len(grid)} — '
                    f'ending early')
                return True

            wp = grid[idx]
            k = node_key(wp)

            if not self._is_reachable(wp):
                failed[k] = failed.get(k, 0) + 1
                self.get_logger().warn(
                    f'{label}: {k} unreachable ({failed[k]}/{max_node_retries})')
                # Wake up before deciding to retry or skip — stale local costmap
                # is a common cause of "planner can't find a path from here".
                self._waypoint_wakeup(label=f'{label}-wakeup-unreach')
                if failed[k] >= max_node_retries:
                    self.get_logger().warn(
                        f'{label}: skipping {k} after {max_node_retries} attempts')
                    idx += 1
                continue

            time_left = deadline - time.time()
            if time_left <= 0:
                break

            if opportunistic_collect:
                ok = self._go_to_with_opportunistic_collect(
                    wp, label=f'{label}-transit',
                    timeout_s=min(OPPORTUNISTIC_NAV_TIMEOUT_S, time_left))
            else:
                ok = self.go_to(wp, timeout_s=min(NAV_GOAL_TIMEOUT_S, time_left),
                                precise=False)

            if not ok:
                failed[k] = failed.get(k, 0) + 1
                self.get_logger().warn(
                    f'{label}: {k} nav failed ({failed[k]}/{max_node_retries})')
                # Wake up so the NEXT waypoint (whether retry or skip) sees a
                # fresh local costmap and the planner has a clean view.
                self._waypoint_wakeup(label=f'{label}-wakeup-nav')
                if failed[k] >= max_node_retries:
                    self.get_logger().warn(
                        f'{label}: skipping {k} after {max_node_retries} attempts')
                    idx += 1
                continue

            idx += 1

            self.get_logger().info(f"{label}: sweeping at {k}")
            self._enable_collection(True)
            try:
                n_cycles = self._sweep_and_collect(label)
            finally:
                self._enable_collection(False)
            self.get_logger().info(f'{label}: {n_cycles} FSM cycle(s) at {k}')

            # After-sweep check — catches "tank just filled this cycle".
            if _stop():
                self.get_logger().info(
                    f'{label}: stop_condition met after sweep at {k} — ending early')
                return True

            self._clear_costmaps(f'{label}-post-sweep')

        self.get_logger().info(
            f'{label}: ended  visited={idx} / {len(grid)}  '
            f'time_left={max(0.0, deadline - time.time()):.0f}s')
        return False
