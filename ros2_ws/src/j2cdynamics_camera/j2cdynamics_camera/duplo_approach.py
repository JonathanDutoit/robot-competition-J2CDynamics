import math
import rclpy
import json
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

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
CONTROL_HZ          = 10.0

# ── Forward safety (lidar) ────────────────────────────────────────────────────
# The visual servo has no obstacle awareness — it would happily drive into a
# wall behind a duplo. We sample /scan in a forward cone and zero out forward
# velocity when something is closer than SAFE_DIST. Rotation is unaffected so
# the robot can still re-acquire / turn away while stopped.
# NOTE: if the lidar sits below ~5cm it may see the duplo itself as an obstacle
# and stop short of every pickup. Verify on the bench; narrow CONE_HALF_RAD or
# raise SAFE_DIST if false-positives, the opposite if it bumps walls.
SAFE_DIST_M     = 0.30   # forward stop distance (lidar frame)
CONE_HALF_RAD   = 0.35   # ~20° each side of straight-ahead

# ── Anti-wander tuning ────────────────────────────────────────────────────────
# When the target is lost mid-approach, distinguish "tracking cleanly, momentary
# dropout" (creep forward) from "lost at edge or far" (don't commit forward,
# slow-rotate to re-acquire). Without this, blind forward + stale steering curves
# the robot away from the duplo when it was last seen near the FOV edge.
BLIND_FORWARD_BUDGET   = 1.0    # s; cap on continuous blind forward motion
WANDER_OFFCENTER_THRESH = 0.4   # |err_x| above this → was at FOV edge, don't dead-reckon forward
WANDER_FAR_THRESH       = 0.4   # last by below this → was far, don't dead-reckon forward
REACQUIRE_ROTATE_RATE   = 0.3   # rad/s; in-place sweep toward last bearing while re-acquiring


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

        # Forward clearance (m) from the latest /scan, in the ±CONE_HALF_RAD cone.
        # inf means "no scan yet" — we treat that as safe so we don't deadlock at
        # startup if /scan is briefly late. The gate fires only on a real reading.
        self.front_clearance = float('inf')
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

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
        self.lost_start_time = None   # NEW: continuous loss tracking

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

    # Lidar callback — min range in forward cone, cached for the control loop.
    def on_scan(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        valid = (np.abs(angles) <= CONE_HALF_RAD) & \
                (ranges > msg.range_min) & (ranges < msg.range_max) & \
                np.isfinite(ranges)
        self.front_clearance = float(ranges[valid].min()) if valid.any() else float('inf')

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

    # FSM transitions
    def update_fsm(self):
        state = self.machine.current_state
        now = self.get_clock().now()
        
        # Search -> Approach
        if state == self.machine.search:
            if self.duplo_visible:
                self._fire('search_to_approach')
            
        # Approach -> Collected
        elif state == self.machine.approach:

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
                self.last_close_time = None
                self.lost_start_time = None
                self._fire('approach_to_collect')
                self.collect_start_time = now

            # 2) Give up ONLY after the full REACQUIRE window with no detection and it
            #    wasn't close — i.e. target_active has finally gone False. Until then we
            #    keep pursuing (dead-reckon in publish_control). "If we see it, we fetch it."
            elif not self.target_active and not recently_close:
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
                des_vx = MAX_LINEAR
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
            was_far       = last_by < WANDER_FAR_THRESH

            if was_offcenter or was_far:
                # (b) Re-acquire by sweeping toward last bearing — no forward commit.
                des_vx = 0.0
                des_wz = -REACQUIRE_ROTATE_RATE * math.copysign(1.0, err_x) * decay
            else:
                # (a) Clean dropout, creep forward, but cap blind distance.
                des_vx = REACQUIRE_VX if time_lost < BLIND_FORWARD_BUDGET else 0.0
                des_wz = -(KP_ANG * 0.6) * err_x * decay

        # COLLECT
        elif state == self.machine.collect:
            des_vx = COLLECT_SPEED

        # Forward safety gate: lidar says something's in our path → stop forward.
        # Rotation is left intact so we can still steer away or re-acquire.
        if des_vx > 0.0 and self.front_clearance < SAFE_DIST_M:
            self.get_logger().warn(
                f'front clearance {self.front_clearance:.2f}m < {SAFE_DIST_M}m '
                f'(state={state.id}) — blocking forward',
                throttle_duration_sec=1.0)
            des_vx = 0.0

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
            "align_tol": ALIGN_TOL,
            "close_frac": CLOSE_ROW_FRAC,
            "vx": round(self.cur_vx, 3),
            "wz": round(self.cur_wz, 3),
            "clearance": (round(self.front_clearance, 2)
                          if math.isfinite(self.front_clearance) else None),
            "safe_dist": SAFE_DIST_M,
        }
        if self.best_target is not None:
            s["err_x"] = round(self.best_target[0], 3)
            s["by"] = round(self.best_target[1], 3)
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