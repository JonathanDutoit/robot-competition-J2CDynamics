#!/usr/bin/env python3
"""
ROS 2 robot dashboard — run on your laptop inside the sourced ros2_dev env.

    source setup_dev.sh
    python3 dashboard.py
    # then open http://localhost:8080

Layout:
  - KEY SIGNALS  : curated, grouped status (localization / velocity / perception)
  - Resources    : robot CPU/mem/temp (via /robot_stats) + this host's (psutil)
  - Nodes/Topics : collapsed drawers, infra noise filtered out
  - Camera       : MJPEG stream
  - Map view     : static map + global & local costmaps overlaid + scan + pose
                   INTERACTIVE: 2D Pose Estimate (-> /initialpose) and
                   2D Goal (-> /goal_pose), click + drag like RViz

Adjust the CONFIG block for your topic names (camera especially).
"""

import json
import math
import threading
import time
from collections import defaultdict

import numpy as np

try:
    import cv2
except ImportError:
    raise SystemExit("Missing cv2. Install conda 'opencv' (see environment.yml)")
try:
    from flask import Flask, Response, jsonify, request
except ImportError:
    raise SystemExit("Missing flask. conda install -c conda-forge flask")
try:
    import psutil
except ImportError:
    raise SystemExit("Missing psutil. conda install -c conda-forge psutil")

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy, qos_profile_sensor_data)

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import LaserScan, Image, CompressedImage
from nav_msgs.msg import Odometry, OccupancyGrid, Path

import tf2_ros

try:
    from vision_msgs.msg import Detection2DArray
    _HAVE_VISION = True
except ImportError:
    _HAVE_VISION = False

# --------------------------- CONFIG ----------------------------------------
HTTP_PORT        = 8080
CAMERA_TOPIC     = "/camera/image_raw"   # <-- set to your real camera topic
MAP_TOPIC        = "/map"
GLOBAL_COSTMAP   = "/global_costmap/costmap"
LOCAL_COSTMAP    = "/local_costmap/costmap"
SCAN_TOPIC       = "/scan"
ODOM_TOPIC       = "/odom"
CMD_OUT_TOPIC    = "/cmd_vel_muxed"      # twist_mux output
ESTOP_TOPIC      = "/e_stop"
DETECTIONS_TOPIC = "/detections"
COLLISION_POLY   = "/collision_approach"
# detections arrive in LORES pixel space; set to your detector's frame size
# so boxes scale correctly onto whatever resolution the camera topic streams.
DETECTION_REF_W  = 640      # <-- LORES_SIZE[0]
DETECTION_REF_H  = 480      # <-- LORES_SIZE[1]
GLOBAL_PLAN_TOPIC = "/plan"              # Nav2 global path
LOCAL_PLAN_TOPIC  = "/local_plan"        # controller local trajectory
INITIALPOSE_TOPIC = "/initialpose"       # SLAM Toolbox re-localization
GOAL_TOPIC        = "/goal_pose"         # Nav2 bt_navigator goal
DUPLO_STATE_TOPIC = "/duplo_state"       # duplo_approach FSM state (JSON String)
MAP_FRAME        = "map"
ROBOT_FRAME      = "base_link"
# velocity lanes in priority order (highest first) for "active source"
VEL_LANES        = [("/teleop_vel", "teleop"), ("/duplo_vel", "duplo"), ("/nav_vel", "nav")]

MAP_VIEW_FPS     = 5
CAMERA_FPS       = 15
MAP_VIEW_MAXPX   = 700
COSTMAP_THRESH   = 50    # show costmap cells at/above this cost


def map_scale(W, H):
    return min(MAP_VIEW_MAXPX / max(W, H), 6.0)


_RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)
_LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST)

WATCH = [
    (SCAN_TOPIC,      LaserScan,     qos_profile_sensor_data),
    (ODOM_TOPIC,      Odometry,      _RELIABLE),
    (MAP_TOPIC,       OccupancyGrid, _LATCHED),
    (GLOBAL_COSTMAP,  OccupancyGrid, _LATCHED),
    (LOCAL_COSTMAP,   OccupancyGrid, _LATCHED),
    (CMD_OUT_TOPIC,   Twist,         _RELIABLE),
    (ESTOP_TOPIC,     Bool,          _RELIABLE),
    ("/nav_vel",      Twist,         _RELIABLE),
    ("/duplo_vel",    Twist,         _RELIABLE),
    ("/teleop_vel",   Twist,         _RELIABLE),
    ("/robot_stats",  String,        _RELIABLE),
    (DUPLO_STATE_TOPIC, String,      _RELIABLE),
    (GLOBAL_PLAN_TOPIC, Path,        _RELIABLE),
    (LOCAL_PLAN_TOPIC,  Path,        _RELIABLE),
]
if _HAVE_VISION:
    WATCH.append((DETECTIONS_TOPIC, Detection2DArray, _RELIABLE))

INFRA_SUBSTR = ("/_action/", "/bond", "/parameter_events", "/rosout",
                "/diagnostics", "set_feedback")


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def is_infra(name):
    if any(p in name for p in INFRA_SUBSTR):
        return True
    return name.endswith("transition_event")


class Monitor(Node):
    def __init__(self):
        super().__init__("dashboard_monitor")
        self.lock = threading.Lock()
        self._counts = defaultdict(int)
        self.rates = {}
        self._last_rate_t = time.monotonic()

        self.scan = None
        self.gplan = None
        self.lplan = None
        self.map = None
        self.gcost = None
        self.lcost = None
        self.odom = None
        self.cmd = None
        self.estop = None
        self.robot_stats = None
        self.det_n = 0
        self.det_t = 0.0
        self.det_msg = None
        self.duplo = None
        self.duplo_t = 0.0
        self.tf_ok = False
        self.nodes = []
        self.topics = []
        self.camera_jpeg = None
        self._camera_sub = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        for name, mtype, qos in WATCH:
            self.create_subscription(mtype, name, self._make_cb(name, mtype), qos)

        # interactive tool publishers
        self.pub_initpose = self.create_publisher(
            PoseWithCovarianceStamped, INITIALPOSE_TOPIC, 10)
        self.pub_goal = self.create_publisher(PoseStamped, GOAL_TOPIC, 10)

        self.create_timer(1.0, self._tick)
        self.create_timer(2.0, self._ensure_camera_sub)
        self.get_logger().info("dashboard_monitor started")

    def _make_cb(self, name, mtype):
        def cb(msg):
            now = time.monotonic()
            with self.lock:
                self._counts[name] += 1
                if mtype is LaserScan:
                    self.scan = msg
                elif mtype is OccupancyGrid:
                    if name == MAP_TOPIC:
                        self.map = msg
                    elif name == GLOBAL_COSTMAP:
                        self.gcost = msg
                    elif name == LOCAL_COSTMAP:
                        self.lcost = msg
                elif mtype is Path:
                    if name == GLOBAL_PLAN_TOPIC:
                        self.gplan = msg
                    elif name == LOCAL_PLAN_TOPIC:
                        self.lplan = msg
                elif mtype is Odometry:
                    self.odom = msg
                elif mtype is Twist and name == CMD_OUT_TOPIC:
                    self.cmd = msg
                elif mtype is Bool and name == ESTOP_TOPIC:
                    self.estop = msg.data
                elif mtype is String and name == "/robot_stats":
                    try:
                        self.robot_stats = json.loads(msg.data)
                    except Exception:
                        pass
                elif mtype is String and name == DUPLO_STATE_TOPIC:
                    try:
                        self.duplo = json.loads(msg.data)
                        self.duplo_t = now
                    except Exception:
                        pass
                elif _HAVE_VISION and mtype is Detection2DArray:
                    self.det_n = len(msg.detections)
                    self.det_t = now
                    self.det_msg = msg
        return cb

    # ---- interactive tools ----
    def publish_initialpose(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = MAP_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = float(x)
        m.pose.pose.position.y = float(y)
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = 0.25      # x
        cov[7] = 0.25      # y
        cov[35] = 0.0685   # yaw
        m.pose.covariance = cov
        self.pub_initpose.publish(m)
        self.get_logger().info(f"published /initialpose ({x:.2f},{y:.2f},{yaw:.2f})")

    def publish_goal(self, x, y, yaw):
        m = PoseStamped()
        m.header.frame_id = MAP_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_goal.publish(m)
        self.get_logger().info(f"published /goal_pose ({x:.2f},{y:.2f},{yaw:.2f})")

    # ---- camera ----
    def _camera_cb(self, msg):
        with self.lock:
            self._counts[CAMERA_TOPIC] += 1
            det = self.det_msg
            det_fresh = (time.monotonic() - self.det_t) < 1.5 if self.det_t else False
        img = self._decode_image(msg)
        if img is None:
            return
        if det is not None and det_fresh:
            self._draw_detections(img, det)
        ok, out = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self.lock:
                self.camera_jpeg = out.tobytes()

    def _draw_detections(self, img, det):
        # detections are in DETECTION_REF_W/H pixel space; scale to this frame
        h, w = img.shape[:2]
        sx, sy = w / float(DETECTION_REF_W), h / float(DETECTION_REF_H)
        for d in det.detections:
            cx = d.bbox.center.position.x * sx
            cy = d.bbox.center.position.y * sy
            bw = d.bbox.size_x * sx
            bh = d.bbox.size_y * sy
            x1, y1 = int(cx - bw / 2), int(cy - bh / 2)
            x2, y2 = int(cx + bw / 2), int(cy + bh / 2)
            label, score = "?", 0.0
            if d.results:
                label = d.results[0].hypothesis.class_id
                score = d.results[0].hypothesis.score
            cv2.rectangle(img, (x1, y1), (x2, y2), (40, 200, 40), 2)
            txt = f"{label} {score:.2f}"
            cv2.rectangle(img, (x1, y1 - 16), (x1 + 8 * len(txt), y1), (40, 200, 40), -1)
            cv2.putText(img, txt, (x1 + 2, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (0, 0, 0), 1, cv2.LINE_AA)

    def _decode_image(self, msg):
        try:
            if isinstance(msg, CompressedImage):
                arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                return cv2.imdecode(arr, cv2.IMREAD_COLOR)
            h, w, enc, step = msg.height, msg.width, msg.encoding, msg.step
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            if enc in ("rgb8", "bgr8"):
                img = buf.reshape(h, step)[:, : w * 3].reshape(h, w, 3)
                if enc == "rgb8":
                    img = img[:, :, ::-1]
                img = np.ascontiguousarray(img)
            elif enc in ("rgba8", "bgra8"):
                img = buf.reshape(h, step)[:, : w * 4].reshape(h, w, 4)
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR if enc == "rgba8"
                                   else cv2.COLOR_BGRA2BGR)
            elif enc == "mono8":
                img = buf.reshape(h, step)[:, :w]
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            elif enc in ("16UC1", "mono16"):
                d = buf.view(np.uint16).reshape(h, step // 2)[:, :w]
                img = cv2.convertScaleAbs(d, alpha=255.0 / max(1, int(d.max())))
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            else:
                return None
            return img
        except Exception as e:
            self.get_logger().warn(f"image decode failed: {e}")
            return None

    def _ensure_camera_sub(self):
        if self._camera_sub is not None:
            return
        for name, types in self.get_topic_names_and_types():
            if name != CAMERA_TOPIC:
                continue
            t = types[0] if types else ""
            mtype = CompressedImage if "CompressedImage" in t else Image
            self._camera_sub = self.create_subscription(
                mtype, CAMERA_TOPIC, self._camera_cb, qos_profile_sensor_data)
            self.get_logger().info(f"subscribed camera {CAMERA_TOPIC} as {t}")
            return

    def _tick(self):
        now = time.monotonic()
        dt = now - self._last_rate_t
        self._last_rate_t = now
        with self.lock:
            counts = dict(self._counts)
            self._counts.clear()
        rates = {k: round(v / dt, 1) for k, v in counts.items()}

        nodes = []
        for n, ns in self.get_node_names_and_namespaces():
            if "transform_listener_impl" in n:
                continue
            full = (ns.rstrip("/") + "/" + n) if ns != "/" else "/" + n
            nodes.append(full)
        nodes.sort()

        topics = []
        for name, types in sorted(self.get_topic_names_and_types()):
            topics.append({
                "name": name,
                "type": types[0].split("/")[-1] if types else "?",
                "pubs": self.count_publishers(name),
                "subs": self.count_subscribers(name),
                "hz": rates.get(name),
                "infra": is_infra(name),
            })

        try:
            self.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, rclpy.time.Time())
            tf_ok = True
        except Exception:
            tf_ok = False

        with self.lock:
            self.rates = rates
            self.nodes = nodes
            self.topics = topics
            self.tf_ok = tf_ok

    def _signals(self, rates):
        def hz(t):
            return rates.get(t)

        def row(label, state, value=None, hzval=None):
            return {"label": label, "state": state, "value": value, "hz": hzval}

        odom_v = None
        if self.odom is not None:
            tw = self.odom.twist.twist
            odom_v = f"{tw.linear.x:+.2f} m/s  {tw.angular.z:+.2f} rad/s"
        map_v = f"{self.map.info.width}x{self.map.info.height}" if self.map else None
        loc = [
            row("odometry", "ok" if hz(ODOM_TOPIC) else "bad", odom_v, hz(ODOM_TOPIC)),
            row("laser scan", "ok" if hz(SCAN_TOPIC) else "bad", None, hz(SCAN_TOPIC)),
            row("map", "ok" if self.map else "bad", map_v),
            row("tf map->base", "ok" if self.tf_ok else "bad",
                "locked" if self.tf_ok else "no transform"),
        ]

        active = "idle"
        for t, n in VEL_LANES:
            if hz(t):
                active = n
                break
        if self.estop:
            active = "E-STOP"
        cmd_v = None
        if self.cmd is not None:
            cmd_v = f"{self.cmd.linear.x:+.2f}  {self.cmd.angular.z:+.2f}"
        estop_state = "bad" if self.estop else ("ok" if self.estop is not None else "idle")
        estop_val = ("ENGAGED" if self.estop else
                     ("clear" if self.estop is not None else "n/a"))
        vel = [
            row("active source", "bad" if active == "E-STOP" else
                ("ok" if active != "idle" else "idle"), active),
            row("cmd out (muxed)", "ok" if hz(CMD_OUT_TOPIC) else "idle", cmd_v, hz(CMD_OUT_TOPIC)),
            row("nav lane", "ok" if hz("/nav_vel") else "idle", None, hz("/nav_vel")),
            row("duplo lane", "ok" if hz("/duplo_vel") else "idle", None, hz("/duplo_vel")),
            row("teleop lane", "ok" if hz("/teleop_vel") else "idle", None, hz("/teleop_vel")),
            row("e-stop", estop_state, estop_val),
        ]

        det_recent = (time.monotonic() - self.det_t) < 1.5 if self.det_t else False
        per = [
            row("detections", "ok" if det_recent else "idle",
                f"{self.det_n} obj" if det_recent else "—", hz(DETECTIONS_TOPIC)),
            row("collision monitor", "ok" if hz(COLLISION_POLY) else "idle",
                "active" if hz(COLLISION_POLY) else "—", hz(COLLISION_POLY)),
        ]
        return [
            {"name": "localization", "rows": loc},
            {"name": "velocity pipeline", "rows": vel},
            {"name": "perception / safety", "rows": per},
        ]

    def status(self):
        with self.lock:
            rates = dict(self.rates)
            mg = None
            if self.map is not None:
                info = self.map.info
                sc = map_scale(info.width, info.height)
                mg = {"ox": info.origin.position.x, "oy": info.origin.position.y,
                      "res": info.resolution, "W": info.width, "H": info.height,
                      "scale": sc, "w_px": int(info.width * sc),
                      "h_px": int(info.height * sc)}
            duplo = None
            if self.duplo is not None and (time.monotonic() - self.duplo_t) < 1.5:
                duplo = self.duplo
            return {
                "signals": self._signals(rates),
                "map_geom": mg,
                "duplo": duplo,
                "nodes": self.nodes,
                "topics": self.topics,
                "robot": self.robot_stats,
            }

    def latest_camera(self):
        with self.lock:
            return self.camera_jpeg

    def render_map_view(self):
        with self.lock:
            grid, scan = self.map, self.scan
            gcost, lcost = self.gcost, self.lcost
            gplan, lplan = self.gplan, self.lplan
        if grid is None:
            return self._render_scan_only(scan)

        info = grid.info
        W, H, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        data = np.array(grid.data, dtype=np.int8).reshape(H, W)

        img = np.full((H, W), 200, dtype=np.uint8)
        img[data == 0] = 245
        occ = data > 0
        img[occ] = (255 - data.clip(0, 100) * 2.4).astype(np.uint8)[occ]
        img[data < 0] = 130
        img = np.flipud(img)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        scale = map_scale(W, H)
        img = cv2.resize(img, (int(W * scale), int(H * scale)),
                         interpolation=cv2.INTER_NEAREST)

        def to_px(wx, wy):
            return int((wx - ox) / res * scale), int((H - 1 - (wy - oy) / res) * scale)

        if gcost is not None:
            self._blend_costmap(img, ox, oy, res, scale, gcost, (255, 150, 30), 0.30)
        if lcost is not None:
            self._blend_costmap(img, ox, oy, res, scale, lcost, (30, 120, 255), 0.40)

        if gplan is not None:
            self._draw_path(img, gplan, to_px, (255, 180, 40), 2)   # global plan, blue-ish
        if lplan is not None:
            self._draw_path(img, lplan, to_px, (70, 70, 255), 2)    # local plan, red

        if scan is not None:
            self._draw_scan(img, scan, to_px)

        try:
            tf = self.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, rclpy.time.Time())
            rx, ry = tf.transform.translation.x, tf.transform.translation.y
            yaw = quat_to_yaw(tf.transform.rotation)
            px, py = to_px(rx, ry)
            cv2.circle(img, (px, py), 7, (40, 80, 255), -1)
            cv2.line(img, (px, py),
                     (int(px + 22 * math.cos(yaw)), int(py - 22 * math.sin(yaw))),
                     (40, 80, 255), 2)
        except Exception:
            pass

        self._legend(img)
        return self._jpeg(img)

    def _blend_costmap(self, canvas, ox_m, oy_m, res_m, scale, cm, color, alpha):
        try:
            Hc, Wc = cm.info.height, cm.info.width
            res_c = cm.info.resolution
            ox_c, oy_c = cm.info.origin.position.x, cm.info.origin.position.y
            data = np.array(cm.data, dtype=np.int8).reshape(Hc, Wc)
            mask = data >= COSTMAP_THRESH
            if not mask.any():
                return
            ov = np.zeros((Hc, Wc, 3), dtype=np.uint8)
            ov[mask] = color
            ov = np.flipud(ov)
            m = np.flipud(mask).astype(np.uint8)

            s_cell = (res_c / res_m) * scale
            nw, nh = max(1, int(Wc * s_cell)), max(1, int(Hc * s_cell))
            ov = cv2.resize(ov, (nw, nh), interpolation=cv2.INTER_NEAREST)
            m = cv2.resize(m, (nw, nh), interpolation=cv2.INTER_NEAREST).astype(bool)

            x0 = int((ox_c - ox_m) / res_m * scale)
            top_world_y = oy_c + Hc * res_c
            H_canvas = canvas.shape[0]
            y0 = int(H_canvas - (top_world_y - oy_m) / res_m * scale)

            Hh, Ww = canvas.shape[:2]
            xs, ys = max(0, x0), max(0, y0)
            xe, ye = min(Ww, x0 + nw), min(Hh, y0 + nh)
            if xe <= xs or ye <= ys:
                return
            sx, sy = xs - x0, ys - y0
            sub_ov = ov[sy:sy + (ye - ys), sx:sx + (xe - xs)]
            sub_m = m[sy:sy + (ye - ys), sx:sx + (xe - xs)]
            region = canvas[ys:ye, xs:xe]
            region[sub_m] = ((1 - alpha) * region[sub_m]
                             + alpha * sub_ov[sub_m]).astype(np.uint8)
        except Exception:
            pass

    def _draw_path(self, img, path, to_px, color, thick):
        try:
            pts = []
            for ps in path.poses:
                pts.append(to_px(ps.pose.position.x, ps.pose.position.y))
            if len(pts) >= 2:
                cv2.polylines(img, [np.array(pts, dtype=np.int32)], False,
                              color, thick, cv2.LINE_AA)
        except Exception:
            pass

    def _draw_scan(self, img, scan, to_px):
        try:
            tf = self.tf_buffer.lookup_transform(MAP_FRAME, scan.header.frame_id,
                                                 rclpy.time.Time())
            tx, ty = tf.transform.translation.x, tf.transform.translation.y
            yaw = quat_to_yaw(tf.transform.rotation)
            c, s = math.cos(yaw), math.sin(yaw)
        except Exception:
            return
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        good = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
        lx, ly = ranges[good] * np.cos(angles[good]), ranges[good] * np.sin(angles[good])
        mx, my = c * lx - s * ly + tx, s * lx + c * ly + ty
        for wx, wy in zip(mx, my):
            px, py = to_px(wx, wy)
            cv2.circle(img, (px, py), 1, (80, 220, 80), -1)

    def _render_scan_only(self, scan):
        size = 500
        img = np.full((size, size, 3), 24, dtype=np.uint8)
        cx = cy = size // 2
        cv2.circle(img, (cx, cy), 4, (40, 80, 255), -1)
        if scan is None:
            cv2.putText(img, "waiting for /map or /scan ...", (40, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)
            return self._jpeg(img)
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        good = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
        rmax = max(1e-3, float(np.nanmax(ranges[good])) if good.any() else 1.0)
        ppm = (size * 0.45) / rmax
        for r, a in zip(ranges[good], angles[good]):
            cv2.circle(img, (int(cx + r * ppm * math.cos(a)),
                             int(cy - r * ppm * math.sin(a))), 1, (80, 220, 80), -1)
        cv2.putText(img, "scan (no map / TF)", (12, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
        return self._jpeg(img)

    @staticmethod
    def _legend(img):
        items = [("scan", (80, 220, 80)), ("costmap", (30, 120, 255)),
                 ("plan", (255, 180, 40)), ("local", (70, 70, 255)),
                 ("robot", (40, 80, 255))]
        x = 10
        for label, color in items:
            cv2.circle(img, (x + 4, 14), 4, color, -1)
            cv2.putText(img, label, (x + 12, 18), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (210, 210, 210), 1)
            x += 22 + 8 * len(label)

    @staticmethod
    def _jpeg(img):
        ok, out = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
        return out.tobytes() if ok else None


app = Flask(__name__)
monitor = None


def mjpeg(get_frame, fps):
    while True:
        frame = get_frame()
        if frame:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        time.sleep(1.0 / fps)


@app.route("/api/status")
def api_status():
    return jsonify(monitor.status())


@app.route("/api/set_pose", methods=["POST"])
def api_set_pose():
    d = request.get_json(force=True)
    monitor.publish_initialpose(d["x"], d["y"], d["yaw"])
    return jsonify({"ok": True})


@app.route("/api/set_goal", methods=["POST"])
def api_set_goal():
    d = request.get_json(force=True)
    monitor.publish_goal(d["x"], d["y"], d["yaw"])
    return jsonify({"ok": True})


@app.route("/camera.mjpg")
def camera():
    return Response(mjpeg(monitor.latest_camera, CAMERA_FPS),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/map.mjpg")
def map_view():
    return Response(mjpeg(monitor.render_map_view, MAP_VIEW_FPS),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/")
def index():
    return PAGE


PAGE = """<!doctype html><html><head><meta charset="utf-8">
<title>robot dashboard</title>
<style>
  :root{--bg:#0d1117;--panel:#161b22;--line:#272d36;--fg:#d7dde5;--mut:#7d8694;
        --ok:#3fb950;--bad:#f85149;--idle:#56606e;--acc:#2f81f7;--tool:#f0a020;
        --mono:ui-monospace,"SF Mono","JetBrains Mono",Menlo,Consolas,monospace;}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--fg);font-family:var(--mono);font-size:13px}
  header{padding:9px 16px;border-bottom:1px solid var(--line);display:flex;
         align-items:center;gap:12px;background:var(--panel)}
  header b{font-size:14px;letter-spacing:.6px}
  .dot{width:9px;height:9px;border-radius:50%;background:var(--idle);flex:none}
  .dot.ok{background:var(--ok)} .dot.bad{background:var(--bad)} .dot.idle{background:var(--idle)}
  #conn{background:var(--bad)} #conn.live{background:var(--ok)}
  .wrap{display:flex;flex-direction:column;gap:12px;padding:12px}
  .row{display:grid;grid-template-columns:1fr 1fr;gap:12px;align-items:start}
  .col{display:flex;flex-direction:column;gap:12px;min-width:0}
  .panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;overflow:hidden}
  .panel>h2{margin:0;padding:8px 12px;font-size:11px;letter-spacing:1px;text-transform:uppercase;
            color:var(--mut);border-bottom:1px solid var(--line);display:flex;align-items:center;gap:8px}
  .grp{padding:6px 12px}
  .grp+.grp{border-top:1px solid var(--line)}
  .grp .gh{font-size:10px;letter-spacing:1px;text-transform:uppercase;color:var(--mut);margin:2px 0 6px}
  .sig{display:flex;align-items:center;gap:8px;padding:3px 0}
  .sig .lbl{flex:1;color:var(--fg)}
  .sig .val{color:var(--mut);text-align:right;white-space:nowrap}
  .sig .hz{color:var(--ok);margin-left:8px;min-width:46px;text-align:right}
  .sig .hz.zero{color:var(--idle)}
  details{border-top:1px solid var(--line)}
  details>summary{padding:7px 12px;cursor:pointer;color:var(--mut);font-size:11px;
                  letter-spacing:.6px;text-transform:uppercase;list-style:none}
  details>summary::-webkit-details-marker{display:none}
  details>summary:before{content:"\\25B8 ";color:var(--idle)}
  details[open]>summary:before{content:"\\25BE "}
  .drawer{max-height:200px;overflow:auto;padding:4px 12px 10px}
  .filt{display:flex;gap:8px;align-items:center;padding:4px 12px;border-top:1px solid var(--line)}
  .filt input[type=text]{flex:1;background:#0a0d12;border:1px solid var(--line);color:var(--fg);
        border-radius:5px;padding:3px 6px;font:inherit}
  table{width:100%;border-collapse:collapse}
  td,th{text-align:left;padding:2px 6px;white-space:nowrap}
  th{color:var(--mut);font-weight:400;font-size:10px}
  tr:hover td{background:#1c232c}
  .num{text-align:right;color:var(--mut)}
  .stream{width:100%;display:block;background:#000;border-radius:6px}
  .res{display:flex;gap:18px;flex-wrap:wrap;padding:8px 12px}
  .res .k{color:var(--mut)} .big{font-size:19px}
  .cores{display:flex;gap:3px;margin:0 12px 10px}
  .cores>i{flex:1;height:22px;background:#0a0d12;border-radius:2px;position:relative}
  .cores>i>b{position:absolute;bottom:0;left:0;right:0;background:var(--acc);display:block}
  .mut{color:var(--mut)}
  .tool{background:#0a0d12;border:1px solid var(--line);color:var(--mut);border-radius:5px;
        padding:3px 9px;font:inherit;cursor:pointer;text-transform:none;letter-spacing:0}
  .tool:hover{border-color:var(--mut)}
  .tool.on{border-color:var(--tool);color:var(--tool)}
  #mapwrap{position:relative;line-height:0}
  #mapwrap.armed #mapimg{cursor:crosshair}
  #mapov{position:absolute;inset:0;width:100%;height:100%;pointer-events:none}
  #maptip{padding:4px 12px;min-height:20px}
  .fsm{display:flex;align-items:center;padding:12px 12px 4px}
  .fsm .st{flex:1;text-align:center;padding:7px 4px;border:1px solid var(--line);
           border-radius:6px;color:var(--mut);background:#0a0d12;font-size:10px;
           letter-spacing:1px;text-transform:uppercase}
  .fsm .st.on{border-color:var(--acc);color:var(--acc);background:#0d1b2e}
  .fsm .st.on.collect{border-color:var(--ok);color:var(--ok);background:#0d1f12}
  .fsm .arr{color:var(--idle);padding:0 6px;flex:none}
  .bars{padding:4px 12px 12px}
  .bar{margin:9px 0}
  .bar .bl{display:flex;justify-content:space-between;color:var(--mut);font-size:11px;margin-bottom:4px}
  .track{position:relative;height:14px;background:#0a0d12;border:1px solid var(--line);border-radius:7px}
  .track .tol{position:absolute;top:0;bottom:0;background:rgba(63,185,80,.18)}
  .track .thr{position:absolute;top:-2px;bottom:-2px;width:2px;background:var(--ok)}
  .track .ctr{position:absolute;top:-2px;bottom:-2px;width:1px;left:50%;background:var(--line)}
  .track .mk{position:absolute;top:50%;width:11px;height:11px;border-radius:50%;
             background:var(--acc);transform:translate(-50%,-50%);border:1px solid #0a0d12}
  .track .mk.bad{background:var(--bad)}
</style></head><body>
<header><span class="dot" id="conn"></span><b>ROBOT DASHBOARD</b>
  <span class="mut" id="counts"></span></header>
<div class="wrap">
  <div class="row">
    <div class="panel"><h2>Key signals</h2><div id="signals"></div></div>
    <div class="panel"><h2>Resources</h2><div id="res"></div></div>
  </div>
  <div class="row">
    <div class="panel"><h2>Camera</h2><img class="stream" src="/camera.mjpg" alt="camera"
         onerror="this.style.opacity=.3"></div>
    <div class="panel">
      <h2>Map / costmaps / scan / plan
        <span style="flex:1"></span>
        <button class="tool" id="btnpose">2D pose</button>
        <button class="tool" id="btngoal">2D goal</button>
      </h2>
      <div id="mapwrap">
        <img class="stream" id="mapimg" src="/map.mjpg" alt="map" draggable="false">
        <svg id="mapov"></svg>
      </div>
      <div class="mut" id="maptip"></div>
    </div>
  </div>
  <div class="panel">
    <h2>Duplo collection FSM</h2>
    <div id="duplo"></div>
  </div>
  <div class="panel">
    <details><summary>Nodes (<span id="nnodes">0</span>)</summary>
      <div class="drawer" id="nodes"></div></details>
    <details><summary>Topics (<span id="ntopics">0</span>)</summary>
      <div class="filt"><input type="text" id="tfilter" placeholder="filter...">
        <label class="mut"><input type="checkbox" id="showinfra"> infra</label></div>
      <div class="drawer" id="topics"></div></details>
  </div>
</div>
<script>
let allTopics = [], mapGeom = null, mode = null, drag = null;
const mapimg = document.getElementById('mapimg');
const mapwrap = document.getElementById('mapwrap');
const mapov = document.getElementById('mapov');
const maptip = document.getElementById('maptip');
const btnpose = document.getElementById('btnpose');
const btngoal = document.getElementById('btngoal');

function setMode(m){
  mode = (mode === m) ? null : m;
  btnpose.classList.toggle('on', mode === 'pose');
  btngoal.classList.toggle('on', mode === 'goal');
  mapwrap.classList.toggle('armed', !!mode);
  maptip.textContent = mode === 'pose' ? 'click + drag on the map to set the robot pose (drag = heading)'
                     : mode === 'goal' ? 'click + drag on the map to send a Nav2 goal (drag = heading)'
                     : '';
}
btnpose.onclick = () => setMode('pose');
btngoal.onclick = () => setMode('goal');

function evFrame(ev){
  const r = mapimg.getBoundingClientRect();
  return [ (ev.clientX - r.left) / r.width  * mapGeom.w_px,
           (ev.clientY - r.top ) / r.height * mapGeom.h_px,
           ev.clientX - r.left, ev.clientY - r.top ];   // also return css px for drawing
}
function frameToWorld(fx, fy){
  const g = mapGeom;
  return [ g.ox + (fx / g.scale) * g.res,
           g.oy + (g.H - 1 - fy / g.scale) * g.res ];
}
mapimg.addEventListener('mousedown', ev => {
  if(!mode || !mapGeom) return;
  ev.preventDefault();
  const [fx, fy, cx, cy] = evFrame(ev);
  drag = {fx, fy, cx, cy};
});
window.addEventListener('mousemove', ev => {
  if(!drag) return;
  const r = mapimg.getBoundingClientRect();
  const cx = ev.clientX - r.left, cy = ev.clientY - r.top;
  const col = mode === 'goal' ? '#2f81f7' : '#f0a020';
  mapov.innerHTML =
    '<line x1="'+drag.cx+'" y1="'+drag.cy+'" x2="'+cx+'" y2="'+cy+'" stroke="'+col+'" stroke-width="2.5"/>'
    +'<circle cx="'+drag.cx+'" cy="'+drag.cy+'" r="5" fill="'+col+'"/>';
});
window.addEventListener('mouseup', ev => {
  if(!drag) return;
  const [efx, efy] = evFrame(ev);
  const [wx, wy]   = frameToWorld(drag.fx, drag.fy);
  const [wex, wey] = frameToWorld(efx, efy);
  let yaw = Math.atan2(wey - wy, wex - wx);
  if(Math.hypot(efx - drag.fx, efy - drag.fy) < 6) yaw = 0;
  const ep = mode === 'pose' ? '/api/set_pose' : '/api/set_goal';
  fetch(ep, {method:'POST', headers:{'Content-Type':'application/json'},
             body: JSON.stringify({x:wx, y:wy, yaw:yaw})})
    .then(()=> maptip.textContent = (mode==='pose'?'pose set':'goal sent')
        +' @ ('+wx.toFixed(2)+', '+wy.toFixed(2)+', '+(yaw*180/Math.PI).toFixed(0)+'\\u00B0)')
    .catch(()=> maptip.textContent = 'failed to publish');
  mapov.innerHTML = ''; drag = null;
});

function sigRow(r){
  let hz = r.hz==null ? '' : '<span class="hz'+(r.hz?'':' zero')+'">'+r.hz+' Hz</span>';
  let val = r.value!=null ? '<span class="val">'+r.value+'</span>' : '';
  return '<div class="sig"><span class="dot '+r.state+'"></span>'
        +'<span class="lbl">'+r.label+'</span>'+val+hz+'</div>';
}
function resBlock(title, s){
  if(!s) return '<div class="grp"><div class="gh">'+title+'</div><div class="mut">no data</div></div>';
  let cores=(s.per_cpu||[]).map(c=>'<i><b style="height:'+c+'%"></b></i>').join('');
  let temp = s.temp_c!=null ? '<div><span class="k">temp</span><div class="big">'+s.temp_c+'&deg;</div></div>':'';
  return '<div class="gh" style="padding:6px 12px 0">'+title+' &mdash; '+(s.host||'')+'</div>'
    +'<div class="res">'
    +'<div><span class="k">cpu</span><div class="big">'+(s.cpu_percent??'-')+'%</div></div>'
    +'<div><span class="k">mem</span><div class="big">'+(s.mem_percent??'-')+'%</div>'
    +'<div class="mut">'+(s.mem_used_mb||0)+'/'+(s.mem_total_mb||0)+'MB</div></div>'+temp+'</div>'
    +(cores?'<div class="cores">'+cores+'</div>':'');
}
function renderTopics(){
  let q=document.getElementById('tfilter').value.toLowerCase();
  let infra=document.getElementById('showinfra').checked;
  let rows=allTopics.filter(t=>(infra||!t.infra)&&t.name.toLowerCase().includes(q)).map(t=>{
    let hz=t.hz==null?'--':(t.hz?t.hz:'0');
    return '<tr><td>'+t.name+'</td><td class="mut">'+t.type+'</td>'
      +'<td class="num">'+t.pubs+'p/'+t.subs+'s</td><td class="num">'+hz+'</td></tr>';
  }).join('');
  document.getElementById('topics').innerHTML=
    '<table><tr><th>topic</th><th>type</th><th class="num">p/s</th><th class="num">Hz</th></tr>'+rows+'</table>';
}
function renderDuplo(d){
  const el = document.getElementById('duplo');
  if(!d){ el.innerHTML = '<div class="bars"><div class="mut">duplo_approach not running</div></div>'; return; }
  const clamp = v => Math.max(0, Math.min(100, v));
  const states = ['search','align','approach','collect'];
  let chain = '<div class="fsm">';
  states.forEach((s,i)=>{
    chain += '<div class="st'+(d.state===s?' on '+s:'')+'">'+s+'</div>';
    if(i<states.length-1) chain += '<span class="arr">\u2192</span>';
  });
  chain += '</div>';

  let bars = '<div class="bars">';
  // err_x: range [-1,1], 0 centered, green tolerance zone +/- align_tol
  const tol = d.align_tol ?? 0.10;
  if(d.err_x!=null){
    const pos = clamp((d.err_x+1)/2*100);
    const tolL = (1-tol)/2*100, tolW = tol*100;
    const inTol = Math.abs(d.err_x) < tol;
    bars += '<div class="bar"><div class="bl"><span>err_x &middot; centering (tol \u00B1'+tol.toFixed(2)+')</span><span>'+d.err_x.toFixed(3)+'</span></div>'
      +'<div class="track"><div class="ctr"></div><div class="tol" style="left:'+tolL+'%;width:'+tolW+'%"></div>'
      +'<div class="mk'+(inTol?'':' bad')+'" style="left:'+pos+'%"></div></div></div>';
  } else {
    bars += '<div class="bar"><div class="bl"><span>err_x &middot; centering</span><span class="mut">no target</span></div>'
      +'<div class="track"><div class="ctr"></div></div></div>';
  }
  // by_norm: range [0,1], green threshold line at close_frac
  const cf = d.close_frac ?? 0.95;
  if(d.by!=null){
    const pos = clamp(d.by*100);
    const close = d.by > cf;
    bars += '<div class="bar"><div class="bl"><span>by_norm &middot; proximity (close &gt;'+cf.toFixed(2)+')</span><span>'+d.by.toFixed(3)+'</span></div>'
      +'<div class="track"><div class="thr" style="left:'+(cf*100)+'%"></div>'
      +'<div class="mk" style="left:'+pos+'%;background:'+(close?'var(--ok)':'var(--acc)')+'"></div></div></div>';
  } else {
    bars += '<div class="bar"><div class="bl"><span>by_norm &middot; proximity</span><span class="mut">no target</span></div>'
      +'<div class="track"><div class="thr" style="left:'+(cf*100)+'%"></div></div></div>';
  }
  bars += '<div class="sig"><span class="dot '+(d.visible?'ok':'idle')+'"></span>'
        +'<span class="lbl">duplo visible</span>'
        +'<span class="val">'+(d.visible?'yes':'no')
        + (d.vx!=null ? '  &middot;  vx '+d.vx.toFixed(2)+'  wz '+d.wz.toFixed(2) : '')
        +'</span></div>';
  bars += '</div>';
  el.innerHTML = chain + bars;
}
async function tick(){
  try{
    let s = await (await fetch('/api/status')).json();
    document.getElementById('conn').classList.add('live');
    mapGeom = s.map_geom;
    document.getElementById('signals').innerHTML = s.signals.map(g=>
      '<div class="grp"><div class="gh">'+g.name+'</div>'+g.rows.map(sigRow).join('')+'</div>').join('');
    document.getElementById('res').innerHTML = resBlock('ROBOT', s.robot);
    renderDuplo(s.duplo);
    document.getElementById('nnodes').textContent = s.nodes.length;
    document.getElementById('ntopics').textContent = s.topics.length;
    document.getElementById('counts').textContent = s.nodes.length+' nodes  '+s.topics.length+' topics';
    document.getElementById('nodes').innerHTML = s.nodes.map(n=>'<div>'+n+'</div>').join('');
    allTopics = s.topics; renderTopics();
  }catch(e){ document.getElementById('conn').classList.remove('live'); }
}
document.getElementById('tfilter').addEventListener('input', renderTopics);
document.getElementById('showinfra').addEventListener('change', renderTopics);
tick(); setInterval(tick, 1500);
</script></body></html>"""


def main():
    global monitor
    rclpy.init()
    monitor = Monitor()
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    threading.Thread(target=executor.spin, daemon=True).start()
    print(f"dashboard at http://localhost:{HTTP_PORT}")
    try:
        app.run(host="0.0.0.0", port=HTTP_PORT, threaded=True)
    finally:
        executor.shutdown()
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()