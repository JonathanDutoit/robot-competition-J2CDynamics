"""
Subscribes : /detections     vision_msgs/Detection2DArray
Publishes  : duplo_vel       geometry_msgs/Twist   (NOT /cmd_vel — twist_mux arbitrates)

Behavior:
  - When a duplo is visible: turn to center it, drive forward over it
    (passive trailer pickup — no grasp, no stopping at a distance).
  - When no duplo is visible: publish nothing. twist_mux times out the
    duplo_vel lane after 0.5s and Nav2's nav_vel resumes driving the robot.

This means the patrol just runs, and any time a duplo enters the frame the
patrol gets overridden, the robot steers over it, and then the patrol resumes
automatically. No coordination with the mission runner needed.
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
from j2cdynamics_camera.config import MODEL_PATH, LORES_SIZE, MAIN_SIZE, CONF_THRESH, CLASS_NAMES


# ── Parameters ──────────────────────────────────────────────────────────────────
IMAGE_WIDTH    = LORES_SIZE[0]        
IMAGE_HEIGHT   = LORES_SIZE[1]        
TARGET_CLASS   = "duplo"
MIN_CONFIDENCE = CONF_THRESH        # ignore weak detections; tune on hardware

# Steering (rotation)
KP_ANG         = 1.5        # rad/s per unit of normalized error
MAX_ANGULAR    = 0.8        # rad/s — well under the 0.94 rad/s kinematic ceiling
ALIGN_TOL      = 0.10       # normalized; |err| below this = "aligned"

# Forward drive (over the duplo)
MAX_LINEAR     = 0.195       # m/s — slow approach so we don't bowl past it
MIN_LINEAR     = 0.05       # m/s — keep nudging even when aligned + close

# Box-bottom row at which we consider the duplo "right at the trailer" and stop
# pushing forward (it'll get scooped by the trailer as we drive past).
# 0.95 = box bottom must be in the bottom 5% of the frame.
CLOSE_ROW_FRAC = 0.95

LOST_TIMEOUT   = 0.5        # s — must match twist_mux duplo lane timeout
CONTROL_HZ     = 10.0


class DuploApproach(Node):
    def __init__(self):
        super().__init__('duplo_approach')
        self.sub = self.create_subscription(
            Detection2DArray, '/detections', self.on_detections, 10)
        self.pub = self.create_publisher(Twist, 'duplo_vel', 10)
        self.timer = self.create_timer(1.0 / CONTROL_HZ, self.on_timer)

        self.target = None      # (cx_norm, by_norm) or None — set per detection msg
        self.last_seen = self.get_clock().now()
        self.get_logger().info('duplo_approach started, publishing to duplo_vel')

    def on_detections(self, msg: Detection2DArray):
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
                    # Normalize: cx_norm in [-1, +1], by_norm in [0, 1] (0 top, 1 bottom)
                    best = (
                        (cx - IMAGE_WIDTH / 2.0) / (IMAGE_WIDTH / 2.0),
                        by / IMAGE_HEIGHT,
                    )

        if best is not None:
            self.target = best
            self.last_seen = self.get_clock().now()

    def on_timer(self):
        dt = (self.get_clock().now() - self.last_seen).nanoseconds * 1e-9

        # No target / target lost — publish NOTHING. twist_mux will time out
        # the duplo_vel lane and Nav2's nav_vel takes over for patrol.
        if self.target is None or dt > LOST_TIMEOUT:
            self.target = None
            return

        err_x, by_norm = self.target
        twist = Twist()

        # Angular: proportional to horizontal error.
        # err_x > 0 (target on the right) -> turn clockwise -> negative z (REP-103).
        if abs(err_x) > ALIGN_TOL / 2:   # smaller deadband than the linear gate
            w = -KP_ANG * err_x
            twist.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, w))

        # Linear: drive forward, scaled DOWN when off-axis (turn first, then push).
        # When the box bottom reaches CLOSE_ROW_FRAC, the duplo is at our feet —
        # the trailer will scoop it as we continue past, so we just stop pushing.
        if by_norm < CLOSE_ROW_FRAC:
            alignment = max(0.0, 1.0 - abs(err_x) / ALIGN_TOL)  # 1 aligned, 0 off-axis
            twist.linear.x = max(MIN_LINEAR, MAX_LINEAR * alignment)
        # else: leave linear.x = 0; the angular keeps tracking if it drifts

        self.pub.publish(twist)


def main():
    rclpy.init()
    node = DuploApproach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())   # explicit stop on shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()