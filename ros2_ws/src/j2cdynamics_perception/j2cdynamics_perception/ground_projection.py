#!/usr/bin/env python3
"""
ground_projection — turn image-space duplo detections into metric (x, y) points
in the map frame, by projecting the bbox's ground-contact pixel onto the ground
plane (pinhole model + known camera mount), then transforming through tf.

Pipeline:
    /detections (Detection2DArray, 1640x1232 px — MAIN_SIZE)
        → bottom-centre pixel of each bbox  (the duplo touches the ground there)
        → undistort + back-project to a ray in the camera frame
        → intersect the ground plane (z = 0 in base_link)  → (x, y) in base_link
        → tf base_link → map                                → (x, y) in map
        → /duplo_points (PoseArray, map frame)  [+ RViz markers]

This is what lets the mission NAVIGATE to a duplo (robust, localized) instead of
only image-servoing onto one. Validated by a prior team on the same Pi Cam v2.1.

Both intrinsics and the camera mount come from config/camera_calibration.yaml
(see calibrate_camera.py). Until you calibrate, rough IMX219 defaults are used.
"""
import math

import numpy as np
import yaml

try:
    import cv2
except ImportError:
    raise SystemExit("ground_projection needs OpenCV (cv2)")

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

import json

from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Pose, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

import tf2_ros
from tf2_geometry_msgs import do_transform_point


# ── Pure projection (no ROS — unit-testable) ──────────────────────────────────
class GroundProjector:
    """Pixel (u, v) → (x, y) on the ground in base_link.

    Camera optical centre sits at height `height` above the ground, pitched down
    by `pitch` rad from horizontal, offset `x_offset` forward / `y_offset` left of
    base_link. Assumes the projected point lies on the ground plane (z = 0).
    """

    def __init__(self, K, dist, height, pitch, x_offset=0.0, y_offset=0.0):
        self.K = np.asarray(K, np.float64).reshape(3, 3)
        self.dist = np.asarray(dist, np.float64).reshape(-1)
        # optical frame (x-right, y-down, z-fwd) → base frame (x-fwd, y-left, z-up)
        R0 = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
        c, s = math.cos(pitch), math.sin(pitch)
        Ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)  # tilt down about base-y
        self.R = Ry @ R0
        self.t = np.array([float(x_offset), float(y_offset), float(height)], dtype=float)

    def project(self, u, v):
        # undistort the pixel → normalised camera-frame ray [x_n, y_n, 1]
        pt = cv2.undistortPoints(
            np.array([[[float(u), float(v)]]], dtype=np.float32), self.K, self.dist)[0, 0]
        ray = self.R @ np.array([pt[0], pt[1], 1.0])
        if ray[2] >= -1e-6:                       # ray must point toward the ground
            return None
        s = -self.t[2] / ray[2]                   # intersect z = 0
        p = self.t + s * ray
        return float(p[0]), float(p[1])
    
# ── ROS node ──────────────────────────────────────────────────────────────────
class GroundProjectionNode(Node):
    def __init__(self):
        super().__init__('ground_projection')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('min_score', 0.30)
        self.declare_parameter('max_range', 3.0)     # m; drop far projections (perspective error grows)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        # Resolution the INCOMING bbox coords are expressed in. Must match the
        # detector's MAIN_SIZE. K is auto-scaled from its calibration resolution
        # (set in the yaml) to this resolution at load time.
        self.declare_parameter('bbox_image_size', [1640, 1232])

        self.min_score = float(self.get_parameter('min_score').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.bbox_image_size = list(self.get_parameter('bbox_image_size').value)

        cal = self._load_calibration(self.get_parameter('calibration_file').value)
        self.proj = GroundProjector(
            cal['camera_matrix'], cal['dist_coeffs'], cal['height'], cal['pitch'],
            cal.get('x_offset', 0.0), cal.get('y_offset', 0.0))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_points = self.create_publisher(PoseArray, 'duplo_points', 10)
        self.pub_markers = self.create_publisher(MarkerArray, 'duplo_points_markers', 10)
        # Per-detection debug info (JSON) for the dashboard: bbox center pixel,
        # robot-frame (x, y), map-frame (x, y), and distance from base_link.
        self.pub_debug = self.create_publisher(String, 'duplo_debug', 10)
        self.create_subscription(Detection2DArray, '/detections',
                                 self.on_detections, qos_profile_sensor_data)
        self.get_logger().info('ground_projection started -> /duplo_points (map frame)')

    def _load_calibration(self, path):
        cal = dict(camera_matrix=[530.0, 0.0, 320.0, 0.0, 530.0, 240.0, 0.0, 0.0, 1.0],
                   dist_coeffs=[0.0, 0.0, 0.0, 0.0, 0.0],
                   height=0.20, pitch=0.52, x_offset=0.12, y_offset=0.0)
        if path:
            try:
                with open(path) as f:
                    loaded = yaml.safe_load(f) or {}
                cal.update({k: v for k, v in loaded.items() if v is not None})
                self.get_logger().info(f'loaded calibration: {path}')
            except Exception as e:
                self.get_logger().warn(f'calibration load failed ({e}); using defaults')
        else:
            self.get_logger().warn('no calibration_file; using rough IMX219 defaults — CALIBRATE!')

        # ── Scale K from its calibration resolution to the bbox resolution ────
        # K is resolution-dependent: fx, fy, cx, cy all scale linearly with size.
        # Distortion coefficients are in normalised image coords so they DON'T scale.
        calib_size = list(cal.get('image_size', [640, 480]))
        target_size = list(self.bbox_image_size)
        if calib_size[0] != target_size[0] or calib_size[1] != target_size[1]:
            K = np.asarray(cal['camera_matrix'], dtype=np.float64).reshape(3, 3)
            sx = float(target_size[0]) / float(calib_size[0])
            sy = float(target_size[1]) / float(calib_size[1])
            K[0, 0] *= sx   # fx
            K[1, 1] *= sy   # fy
            K[0, 2] *= sx   # cx
            K[1, 2] *= sy   # cy
            cal['camera_matrix'] = K.flatten().tolist()
            self.get_logger().info(
                f'K scaled: calib {calib_size} → bbox {target_size}  '
                f'(sx={sx:.3f}, sy={sy:.3f})')
        else:
            self.get_logger().info(
                f'K already at bbox resolution {target_size} — no scaling')
        return cal

    def on_detections(self, msg: Detection2DArray):
        out = PoseArray()
        out.header.frame_id = self.map_frame
        out.header.stamp = msg.header.stamp
        markers = MarkerArray()

        debug_items = []
        for i, det in enumerate(msg.detections):
            score = max((r.hypothesis.score for r in det.results), default=0.0)
            if score < self.min_score:
                continue
            # bottom-centre of the bbox = where the duplo meets the ground
            u = det.bbox.center.position.x
            v = det.bbox.center.position.y + det.bbox.size_y / 2.0
            g = self.proj.project(u, v)
            if g is None:
                continue
            dist = math.hypot(g[0], g[1])
            if dist > self.max_range:
                continue
            mp = self._to_map(g, msg.header.stamp)
            if mp is None:
                continue
            pose = Pose()
            pose.position.x, pose.position.y = mp
            pose.orientation.w = 1.0
            out.poses.append(pose)
            markers.markers.append(self._marker(i, mp, msg.header.stamp))
            debug_items.append({
                "cx": float(det.bbox.center.position.x),
                "cy": float(det.bbox.center.position.y),
                "bw": float(det.bbox.size_x),
                "bh": float(det.bbox.size_y),
                "x_base": float(g[0]),
                "y_base": float(g[1]),
                "x_map": float(mp[0]),
                "y_map": float(mp[1]),
                "dist": float(dist),
                "score": float(score),
            })

        if out.poses:
            self.pub_points.publish(out)
            self.pub_markers.publish(markers)
        # Always publish debug (empty list is meaningful — "no projections this tick")
        self.pub_debug.publish(String(data=json.dumps({"items": debug_items})))
            

    # If lookup-at-stamp misses by less than this, accept latest TF instead.
    # At 0.4 rad/s rotation, 100ms = ~2.3° = ~4cm error at 1m range — far smaller
    # than the rotation drift the original "always latest" lookup was eating.
    _TF_FALLBACK_MAX_GAP_S = 0.10

    def _to_map(self, g, stamp):
        ps = PointStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = stamp
        ps.point.x, ps.point.y = float(g[0]), float(g[1])

        requested = Time.from_msg(stamp)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, requested)
        except Exception:
            # Stamp typically races ~15-30ms ahead of latest TF (camera publish
            # vs AMCL TF rate). Fall back to latest only if the gap is small —
            # large gaps mean a real time-sync problem and the detection isn't
            # trustworthy anyway.
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, Time())
            except Exception as e:
                self.get_logger().warn(
                    f'tf {self.base_frame}->{self.map_frame} failed entirely ({e}); dropping',
                    throttle_duration_sec=2.0)
                return None
            latest = Time.from_msg(tf.header.stamp)
            gap_s = abs((requested - latest).nanoseconds) / 1e9
            if gap_s > self._TF_FALLBACK_MAX_GAP_S:
                self.get_logger().warn(
                    f'tf stamp/latest gap {gap_s*1000:.0f}ms exceeds '
                    f'{self._TF_FALLBACK_MAX_GAP_S*1000:.0f}ms — dropping detection',
                    throttle_duration_sec=2.0)
                return None

        try:
            mp = do_transform_point(ps, tf)
            return mp.point.x, mp.point.y
        except Exception as e:
            self.get_logger().warn(f'do_transform_point failed ({e}); dropping',
                                   throttle_duration_sec=2.0)
            return None

    def _marker(self, i, mp, stamp):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = stamp
        m.ns = 'duplo_points'
        m.id = i
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y = mp
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.08
        m.scale.z = 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.4, 0.0, 0.8
        m.lifetime.sec = 2
        return m


def main():
    rclpy.init()
    node = GroundProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
