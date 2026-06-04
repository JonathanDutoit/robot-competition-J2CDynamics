import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from j2cdynamics_camera.config import LORES_SIZE, CONF_THRESH, CLASS_NAMES
from j2cdynamics_camera.collection_fsm import DuploCollectionMachine


# ── Parameters ────────────────────────────────────────────────────────────────
IMAGE_WIDTH    = LORES_SIZE[0]
IMAGE_HEIGHT   = LORES_SIZE[1]

TARGET_CLASS   = CLASS_NAMES[0]
MIN_CONFIDENCE = CONF_THRESH

KP_ANG         = 0.3
MAX_ANGULAR    = 0.4
ALIGN_TOL      = 0.10

MAX_LINEAR     = 0.195
COLLECT_SPEED  = MAX_LINEAR * 0.8
CLOSE_ROW_FRAC = 0.95

MAX_LIN_ACC    = 0.5   
MAX_ANG_ACC    = 1.5   

LOST_TIMEOUT        = 0.5    # s; hold the last target through brief dropouts
COLLECT_DURATION    = 0.8    # s; open-loop push duration
CONTROL_HZ          = 10.0


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

        self.dt = 1.0 / CONTROL_HZ
        self.timer = self.create_timer(self.dt, self.on_timer)

        # FSM
        self.machine = DuploCollectionMachine()

        # Perception
        self.best_target = None
        self.last_seen_time = None
        self.duplo_visible = False

        # Collect state memory
        self.collect_start_time = None

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
        self.update_perception()
        self.update_fsm()
        self.publish_control()
        self.publish_state()

    # Perception filtering
    def update_perception(self):
        if self.last_seen_time is None:
            self.duplo_visible = False
            return

        dt = (self.get_clock().now() - self.last_seen_time).nanoseconds * 1e-9
        self.duplo_visible = dt < LOST_TIMEOUT

        if not self.duplo_visible:
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
        
        # Search -> Align 
        if state == self.machine.search:
            if self.duplo_visible:
                self._fire('search_to_align')

        # Align -> Approach
        elif state == self.machine.align:
            if not self.duplo_visible:
                self._fire('lost') # align -> search              
            elif self.best_target is not None and abs(self.best_target[0]) < ALIGN_TOL:
                self._fire('align_to_approach')
    
        # Approach -> Collected
        elif state == self.machine.approach:
            if not self.duplo_visible:
                self._fire('lost')  # approach -> search
            elif self.best_target is not None and self.best_target[1] > CLOSE_ROW_FRAC:
                self._fire('approach_to_collect')
                self.collect_start_time = now

        # Collect -> Search
        elif state == self.machine.collect:
            # open-loop scoop: ignore "lost" (we're driving over it), just time out
            if self.collect_start_time is None:
                self.collect_start_time = now
            dt = (now - self.collect_start_time).nanoseconds * 1e-9
            if dt > COLLECT_DURATION:
                self.collect_start_time = None
                self._fire('collect_to_search')

    # Controller
    def publish_control(self):
        state = self.machine.current_state
 
        # SEARCH: ramp down to zero, then go SILENT so twist_mux times out the
        # duplo lane and Nav2's patrol resumes. Publishing zeros here would pin
        # the high-priority lane and freeze the robot instead.
        if state == self.machine.search:
            if abs(self.cur_vx) > 1e-3 or abs(self.cur_wz) > 1e-3:
                self._publish(0.0, 0.0)
            return
 
        des_vx = 0.0
        des_wz = 0.0
 
        if state == self.machine.align and self.best_target is not None:
            des_wz = -KP_ANG * self.best_target[0]
        elif state == self.machine.approach and self.best_target is not None:
            des_wz = -KP_ANG * self.best_target[0]
            des_vx = MAX_LINEAR
        elif state == self.machine.collect:
            des_vx = COLLECT_SPEED          # open-loop push; no target needed
 
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