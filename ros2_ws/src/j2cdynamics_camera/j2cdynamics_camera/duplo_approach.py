import math
import rclpy
import json
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import tf2_ros

from j2cdynamics_camera.config import MAIN_SIZE, LORES_SIZE, CONF_THRESH, CLASS_NAMES
from j2cdynamics_camera.collection_fsm import DuploCollectionMachine


# ── Parameters ────────────────────────────────────────────────────────────────
IMAGE_WIDTH    = MAIN_SIZE[0]
IMAGE_HEIGHT   = MAIN_SIZE[1]

TARGET_CLASS   = CLASS_NAMES[0]
MIN_CONFIDENCE = CONF_THRESH

KP_ANG         = 0.2
MAX_ANGULAR    = 0.4
ALIGN_TOL      = 0.1

MAX_LINEAR     = 0.195
COLLECT_SPEED  = MAX_LINEAR
CLOSE_ROW_FRAC = 0.75       # lower → commit to "close" earlier; with a narrow FOV the
                            # duplo leaves the bottom of the frame before reaching 0.9

MAX_LIN_ACC    = 0.5
MAX_ANG_ACC    = 1.5

# ── Commit-and-persist tuning (poor image / narrow FOV) ───────────────────────
# Once a duplo is seen we keep pursuing it through detection dropouts instead of
# abandoning on the first lost frame. LOST_TIMEOUT = "fresh"; REACQUIRE_TIME =
# how long we dead-reckon toward the last-known target before giving up.
LOST_TIMEOUT        = 1.0    # s; detection counts as "fresh" within this window
REACQUIRE_TIME      = 2.5    # s; keep pursuing the last-seen duplo before quitting
REACQUIRE_VX        = 0.10   # m/s; slow creep forward while reacquiring a lost duplo
COLLECT_DURATION    = 5.0    # s; open-loop scoop duration
COLLECT_LOST_TIME   = 0.3    # s continuously lost (went under the robot) before scooping
RECENT_CLOSE_WINDOW = 0.7    # s; "was close very recently" memory for the scoop trigger
APPROACH_HARD_TIMEOUT_S = 10.0   # max time in 'approach' before giving up via 'lost'.
                                # Catches the case where a duplo is visible but unreachable
                                # (e.g. blocked by costmap safety near a wall) — without this,
                                # the FSM has no transition out since the target stays visible.
CONTROL_HZ          = 10.0

# ── Forward safety (global costmap) ───────────────────────────────────────────
# We use the GLOBAL costmap, not local:
#   - global has static_layer (walls) + keepout_filter (carpet/ramp/danger zones)
#   - global is in MAP frame → keepout positions don't drift when AMCL is stale
#     (which it is during open-loop visual-servo rotation)
#   - local is in odom frame; keepout is re-projected via map→odom every cycle, and
#     stale TF caused the carpet entry we saw in field tests
# Threshold 99: in OccupancyGrid encoding, 100=lethal/keepout and 99=inscribed-
# inflated (robot footprint would touch). Lower thresholds catch soft inflation
# and stop the robot well before the wall, which was preventing pickups of
# wall-near duplos. 99 = "stop only when about to actually intersect something".
SAFETY_LOOKAHEAD_M_GLOBAL = [0.05, 0.50]
SAFETY_LOOKAHEAD_M_LOCAL  = [0.05, 0.25]        # m ahead of base_link; shorter = less cautious near walls
SAFETY_THRESHOLD    = 99                  # 100=lethal/keepout, 99=inscribed (footprint touches)
SAFETY_BASE_FRAME   = 'base_link'
# Blocked-while-in-approach timer: if the gate has been firing continuously for
# this long while we're in approach, the target is unreachable (e.g. on carpet).
# Abandon faster than the 10s hard timeout so we don't burn 40s/waypoint.
BLOCKED_APPROACH_TIMEOUT_S = 3.0

# ── Anti-wander tuning ────────────────────────────────────────────────────────
# When the target is lost mid-approach, distinguish "tracking cleanly, momentary
# dropout" (creep forward) from "lost at edge or far" (don't commit forward,
# slow-rotate to re-acquire). Without this, blind forward + stale steering curves
# the robot away from the duplo when it was last seen near the FOV edge.
WANDER_OFFCENTER_THRESH = 0.4   # |err_x| above this → was at FOV edge, don't dead-reckon forward
REACQUIRE_ROTATE_RATE   = 0.4   # rad/s; in-place sweep toward last bearing while re-acquiring

# ── by → ground distance mapping ──────────────────────────────────────────────
# Bottom-of-bbox in normalized image coordinates is a monotonic proxy for
# distance from the bumper to the duplo. The mapping is camera-specific (depends
# on pitch + height + intrinsics); these are rough numbers to calibrate later by
# placing a duplo at known distances and reading `by` off the dashboard.
#   by = 1.00 → ~0 m   (at the trailer, about to disappear under us)
#   by = 0.75 → ~0.12 m (entering scoop zone — matches CLOSE_ROW_FRAC)
#   by = 0.50 → ~0.35 m
#   by = 0.30 → ~0.70 m
#   by = 0.10 → ~1.20 m
_BY_DIST_REF = [
    (1.00, 0.00),
    (0.75, 0.12),
    (0.50, 0.35),
    (0.30, 0.70),
    (0.10, 1.20),
]

class DuploApproach(Node):

    def __init__(self):
        super().__init__('duplo_approach')

        self.sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.on_detections,
            10
        )

        self.pub = self.create_publisher(Twist, 'duplo_vel', 10)

        self.enabled = True
        self.create_subscription(
            Bool,
            '/enable_duplo_collection',
            self.enable_duplo_collection,
            10
        )

        self.state_pub = self.create_publisher(String, 'duplo_state', 10)

        # Global costmap (static walls + inflation + keepout, per nav2_params.yaml).
        # None means "not received yet" → treated as safe so startup doesn't
        # deadlock before Nav2 is fully up. The gate fires only on a real grid.
        # In map frame → unaffected by AMCL/odom drift during open-loop rotation.
        self._global_map = None
        self._global_data = None
        self._global_frame = None

        self._local_map = None
        self._local_data = None
        self._local_frame = None
        
        # Global costmap is published transient_local; need matching QoS for late join.
        costmap_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.on_global_map, costmap_qos)
        
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.on_local_map, costmap_qos)

        # TF buffer (non-blocking lookups; safe in single-threaded executor).
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._last_blocked = False   # for dashboard

        self.dt = 1.0 / CONTROL_HZ
        self.timer = self.create_timer(self.dt, self.on_timer)

        # FSM
        self.machine = DuploCollectionMachine()

        # Perception
        self.best_target = None
        self.last_seen_time = None
        self.duplo_visible = False     # fresh detection (< LOST_TIMEOUT)
        self.target_active = False     # still worth pursuing (< REACQUIRE_TIME)
        self.err_x_filt = 0.0

        # Collect state memory
        self.collect_start_time = None
        self.last_close_time = None
        self.lost_start_time = None
        self.approach_start_time = None   # for APPROACH_HARD_TIMEOUT_S
        self.blocked_start_time = None    # for BLOCKED_APPROACH_TIMEOUT_S

        # Accel-limited output state
        self.cur_vx = 0.0
        self.cur_wz = 0.0

        self.get_logger().info("duplo_approach (FSM) started -> duplo_vel")

    # Detection callback
    def on_detections(self, msg: Detection2DArray):
        best = self.find_best_duplo(msg)

        if best is not None:
            self.best_target = best
            self.last_seen_time = self.get_clock().now()

    # Enable duplo collection callback
    def enable_duplo_collection(self, msg: Bool):
        self.enabled = msg.data

    # Global costmap callback (latched, republished at ~1 Hz).
    def on_global_map(self, msg: OccupancyGrid):
        self._global_map = msg
        self._global_data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        new_frame = msg.header.frame_id or 'map'
        if new_frame != self._global_frame:
            self.get_logger().info(
                f'global_costmap: {msg.info.width}x{msg.info.height} '
                f'@ {msg.info.resolution:.3f}m/cell, frame={new_frame}')
            self._global_frame = new_frame

    def on_local_map(self, msg: OccupancyGrid):
        self._local_map = msg
        self._local_data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        new_frame = msg.header.frame_id or 'map'
        if new_frame != self._local_frame:
            self.get_logger().info(
                f'local_costmap: {msg.info.width}x{msg.info.height} '
                f'@ {msg.info.resolution:.3f}m/cell, frame={new_frame}')
            self._local_frame = new_frame


    def _costmap_blocked(self, map, data, frame, lookahead, threshold) -> bool:
        """True if any forward look-ahead point lies in a costly cell of the costmap"""
        if map is None or frame is None:
            return False  # not received yet — don't gate
        try:
            tf = self.tf_buffer.lookup_transform(
                frame, SAFETY_BASE_FRAME, rclpy.time.Time())
        except Exception:
            return False
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cx, cy = math.cos(yaw), math.sin(yaw)
        info = map.info
        for d in lookahead:
            x = rx + d * cx
            y = ry + d * cy
            col = int((x - info.origin.position.x) / info.resolution)
            row = int((y - info.origin.position.y) / info.resolution)
            if 0 <= col < info.width and 0 <= row < info.height:
                if int(data[row, col]) >= threshold:
                    return True
        return False
    
    def _forward_blocked(self) -> bool:
        return (self._costmap_blocked(self._global_map, self._global_data,
                                   self._global_frame, SAFETY_LOOKAHEAD_M_GLOBAL, threshold=99)
         or self._costmap_blocked(self._local_map,  self._local_data,
                                   self._local_frame, SAFETY_LOOKAHEAD_M_LOCAL, threshold=60))
    # Detection selection 
    def find_best_duplo(self, msg):
        best_score = -1.0
        best = None

        for det in msg.detections:
            for res in det.results:
                if res.hypothesis.class_id != TARGET_CLASS:
                    continue
                if res.hypothesis.score < MIN_CONFIDENCE:
                    continue

                if res.hypothesis.score > best_score:
                    best_score = res.hypothesis.score

                    cx = det.bbox.center.position.x
                    by = det.bbox.center.position.y + det.bbox.size_y / 2.0

                    best = (
                        (cx - IMAGE_WIDTH / 2.0) / (IMAGE_WIDTH / 2.0),
                        by / IMAGE_HEIGHT
                    )

        return best

    # Main loop
    def on_timer(self):
        if not self.enabled:
            return
        self.update_perception()
        self.update_fsm()
        self.publish_control()
        self.publish_state()

    # Perception filtering
    def update_perception(self):
        if self.last_seen_time is None:
            self.duplo_visible = False
            self.target_active = False
            return

        dt = (self.get_clock().now() - self.last_seen_time).nanoseconds * 1e-9
        self.duplo_visible = dt < LOST_TIMEOUT      # fresh detection → full visual servo
        self.target_active = dt < REACQUIRE_TIME    # recently lost → keep pursuing (dead-reckon)

        # Keep best_target through the whole reacquire window so the controller can
        # dead-reckon toward where the duplo was. Only forget it once we truly give up.
        if not self.target_active:
            self.best_target = None

    def _fire(self, event):
        """Trigger a transition by name; never let an illegal one crash the loop."""
        try:
            getattr(self.machine, event)()
        except Exception as e:
            self.get_logger().warn(f"transition '{event}' rejected: {e}")

    def _reset_approach_timers(self):
        """Clear all approach-state timers on exit (lost or to collect)."""
        self.approach_start_time = None
        self.blocked_start_time = None
        self.last_close_time = None
        self.lost_start_time = None


    def estimate_distance(self, by: float) -> float:
        """Piecewise-linear interpolation from bbox-bottom to ground distance (m)."""
        by = max(0.0, min(1.0, by))
        for (by_hi, d_hi), (by_lo, d_lo) in zip(_BY_DIST_REF[:-1], _BY_DIST_REF[1:]):
            if by >= by_lo:
                t = (by_hi - by) / (by_hi - by_lo)
                return d_hi + t * (d_lo - d_hi)
        return _BY_DIST_REF[-1][1]
    
    # FSM transitions
    def update_fsm(self):
        state = self.machine.current_state
        now = self.get_clock().now()
        
        # Search -> Approach
        if state == self.machine.search:
            if self.duplo_visible:
                self._fire('search_to_approach')
                self.approach_start_time = now

        # Approach -> Collected
        elif state == self.machine.approach:

            # Hard timeout: if a duplo stays visible but unreachable (e.g.
            # blocked by costmap safety near a wall), the recently_close /
            # target_active conditions never fire and we'd loop forever.
            if self.approach_start_time is not None:
                approach_age = (now - self.approach_start_time).nanoseconds * 1e-9
                if approach_age > APPROACH_HARD_TIMEOUT_S:
                    self.get_logger().warn(
                        f'approach timeout after {approach_age:.1f}s — giving up target')
                    self._reset_approach_timers()
                    self._fire('lost')   # back to search
                    return

            # Blocked-while-in-approach: faster abandon when the safety gate is
            # firing continuously. Catches duplos that project onto carpet /
            # keepout, where the FSM would otherwise burn the full hard timeout.
            if self._last_blocked:
                if self.blocked_start_time is None:
                    self.blocked_start_time = now
                else:
                    blocked_age = (now - self.blocked_start_time).nanoseconds * 1e-9
                    if blocked_age > BLOCKED_APPROACH_TIMEOUT_S:
                        self.get_logger().warn(
                            f'approach blocked for {blocked_age:.1f}s '
                            f'(duplo likely in keepout) — giving up target')
                        self._reset_approach_timers()
                        self._fire('lost')
                        return
            else:
                self.blocked_start_time = None

            # Track proximity when visible
            if self.duplo_visible and self.best_target is not None:
                if self.best_target[1] > CLOSE_ROW_FRAC:
                    self.last_close_time = now

            # Track continuous loss (NEW robustness layer)
            if not self.duplo_visible:
                if self.lost_start_time is None:
                    self.lost_start_time = now
            else:
                self.lost_start_time = None

            # Robust transition condition
            recently_close = (
                self.last_close_time is not None and
                (now - self.last_close_time).nanoseconds * 1e-9 < RECENT_CLOSE_WINDOW
            )

            lost_long_enough = (
                self.lost_start_time is not None and
                (now - self.lost_start_time).nanoseconds * 1e-9 > COLLECT_LOST_TIME
            )

            # 1) Was close, then dropped out of the (narrow) frame → it went under
            #    the robot → commit to the scoop.
            if recently_close and lost_long_enough:
                self._reset_approach_timers()
                self._fire('approach_to_collect')
                self.collect_start_time = now

            # 2) Give up ONLY after the full REACQUIRE window with no detection and it
            #    wasn't close — i.e. target_active has finally gone False. Until then we
            #    keep pursuing (dead-reckon in publish_control). "If we see it, we fetch it."
            elif not self.target_active and not recently_close:
                self._reset_approach_timers()
                self._fire('lost')  # Approach -> search

        # Collect -> Search
        elif state == self.machine.collect:
            # open-loop scoop: ignore "lost", just time out
            if self.collect_start_time is None:
                self.collect_start_time = now

            dt = (now - self.collect_start_time).nanoseconds * 1e-9
            if dt > COLLECT_DURATION:
                self.collect_start_time = None
                self._fire('collect_to_search')

    # Controller
    def publish_control(self):
        state = self.machine.current_state
        des_vx = 0.0
        des_wz = 0.0

        # Search
        if state == self.machine.search:
            if abs(self.cur_vx) > 1e-3 or abs(self.cur_wz) > 1e-3:
                self._publish(0.0, 0.0)
            return

        # Approach — fresh detection: full visual servo
        if state == self.machine.approach and self.duplo_visible and self.best_target is not None:
            err_x, by = self.best_target

            ALPHA = 0.4   # 0..1; lower = smoother (more lag)
            self.err_x_filt = (1 - ALPHA) * self.err_x_filt + ALPHA * err_x
            des_wz = -KP_ANG * self.err_x_filt

            # gate forward motion on actual alignment (boolean, not abs(err_x))
            tol = ALIGN_TOL * (2.0 - min(1.0, by / CLOSE_ROW_FRAC))   # ~2x looser far, ALIGN_TOL near
            if abs(self.err_x_filt) < tol:
                # Speed scales with estimated remaining distance, clamped to MAX_LINEAR.
                # Far away → full speed; getting close → ease in so we don't blow past
                # the scoop window when the duplo leaves the FOV.
                dist = self.estimate_distance(by)
                des_vx = max(0.6 * MAX_LINEAR, min(MAX_LINEAR, 0.4 + 0.6 * dist))
            else:
                des_vx = 0.0                 # turn in place until centered

            # bottom override: duplo at the trailer -> commit to the scoop
            if by > CLOSE_ROW_FRAC:
                des_vx = COLLECT_SPEED
                des_wz = -(KP_ANG * 0.6) * err_x

        # Approach — momentarily lost but still active. Two regimes:
        #   (a) was tracking cleanly (centered + close-ish) → likely a real dropout,
        #       creep forward briefly to re-find / scoop.
        #   (b) was at FOV edge or far → blind forward would curve us into a wall;
        #       stop forward, slow-rotate toward last bearing to re-acquire.
        # In both, steering decays with time-since-last-seen (stale bearing → less input).
        elif state == self.machine.approach and self.target_active and self.best_target is not None:
            err_x, last_by = self.best_target
            time_lost = (self.get_clock().now() - self.last_seen_time).nanoseconds * 1e-9
            decay = max(0.0, 1.0 - time_lost / REACQUIRE_TIME)
            was_offcenter = abs(err_x) > WANDER_OFFCENTER_THRESH

            if was_offcenter:
                des_vx = 0.0
                des_wz = -REACQUIRE_ROTATE_RATE * math.copysign(1.0, err_x) * decay
            else:
                last_dist = self.estimate_distance(last_by)
                # Speed: slow creep when we estimate we're basically on top of it,
                # otherwise commit at a reasonable fraction of approach speed.
                if last_dist < 0.15:
                    blind_vx = REACQUIRE_VX
                else:
                    blind_vx = 0.75 * COLLECT_SPEED
                # Stop dead-reckoning once we've covered roughly the last estimated
                # distance (+0.3 s margin for the camera lag).
                max_blind_t = min(REACQUIRE_TIME, last_dist / max(blind_vx, 1e-3) + 0.3)
                des_vx = blind_vx if time_lost < max_blind_t else 0.0
                des_wz = -(KP_ANG * 0.6) * err_x * decay

        # COLLECT
        elif state == self.machine.collect:
            des_vx = COLLECT_SPEED

        # ── Forward safety (global + local costmap) ───────────────────────────────────
        # Two-layer gate:
        #   - global costmap @ threshold=99: static walls + keepout (carpet/ramp).
        #     In map frame, immune to AMCL drift during open-loop rotation.
        #   - local  costmap @ threshold=60: live lidar obstacle_layer. In odom frame,
        #     so the obstacle_layer is unaffected by map→odom drift (unlike the keepout
        #     projection that caused the carpet-entry bug — we don't read keepout here).
        #     Lower threshold catches inflation around dynamic obstacles before contact.
        blocked = des_vx > 0.0 and self._forward_blocked()
        if blocked:
            self.get_logger().warn(
                f'forward look-ahead in costly costmap cell (state={state.id})',
                throttle_duration_sec=1.0)
            des_vx = 0.0
        self._last_blocked = blocked

        self._publish(des_vx, des_wz)


    def _ramp(self, cur, target, max_acc):
        step = max_acc * self.dt
        if target > cur + step:
            return cur + step
        if target < cur - step:
            return cur - step
        return target

    def _publish(self, des_vx, des_wz):
        des_vx = max(-MAX_LINEAR, min(MAX_LINEAR, des_vx))
        des_wz = max(-MAX_ANGULAR, min(MAX_ANGULAR, des_wz))
        self.cur_vx = self._ramp(self.cur_vx, des_vx, MAX_LIN_ACC)
        self.cur_wz = self._ramp(self.cur_wz, des_wz, MAX_ANG_ACC)
        twist = Twist()
        twist.linear.x = self.cur_vx
        twist.angular.z = self.cur_wz
        self.pub.publish(twist)

    # FSM state -> dashboard
    def publish_state(self):
        s = {
            "state": self.machine.current_state.id,
            "visible": self.duplo_visible,
            "err_x": None,
            "by": None,
            "dist_est": None,        
            "align_tol": ALIGN_TOL,
            "close_frac": CLOSE_ROW_FRAC,
            "vx": round(self.cur_vx, 3),
            "wz": round(self.cur_wz, 3),
            "global_ready": self._global_map is not None,
            "local_ready":  self._local_map is not None,
            "forward_blocked": self._last_blocked,
        }
        if self.best_target is not None:
            s["err_x"] = round(self.best_target[0], 3)
            s["by"] = round(self.best_target[1], 3)
            s["dist_est"] = round(self.estimate_distance(self.best_target[1]), 2)
        msg = String()
        msg.data = json.dumps(s)
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = DuploApproach()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()