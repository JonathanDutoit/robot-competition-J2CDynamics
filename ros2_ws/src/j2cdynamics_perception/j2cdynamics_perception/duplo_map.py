#!/usr/bin/env python3
"""
duplo_map — fuse the stream of projected duplo points into a stable, de-duplicated
map of duplo locations, and track which ones have been collected.

    /duplo_points (PoseArray, map)  ─▶  online single-link clustering
                                         (merge points within MERGE_DIST)
                                    ─▶  /duplo_map (PoseArray of CONFIRMED,
                                         uncollected duplo centres, map frame)
                                    ─▶  /duplo_map_markers (RViz/dashboard)

    /duplo_collected (PointStamped) ─▶  mark the nearest cluster collected

Why online single-link instead of batch DBSCAN: detections arrive over time as the
robot scans, so an incremental running-average cluster is the natural fit and needs
no scikit-learn on the Pi. A cluster only becomes a "confirmed" duplo after
MIN_OBS supporting detections, which filters the false positives a poor camera throws.

The mission picks the nearest CONFIRMED, uncollected centre from /duplo_map.
"""
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose, PointStamped
from visualization_msgs.msg import Marker, MarkerArray


class Cluster:
    __slots__ = ('x', 'y', 'n', 'collected')

    def __init__(self, x, y):
        self.x, self.y, self.n, self.collected = x, y, 1, False

    def add(self, x, y):
        # running mean — pulls the centre toward the accumulating evidence
        self.n += 1
        self.x += (x - self.x) / self.n
        self.y += (y - self.y) / self.n


class DuploMap(Node):
    def __init__(self):
        super().__init__('duplo_map')
        self.declare_parameter('merge_dist', 0.35)      # m; points closer than this fuse
        self.declare_parameter('min_obs', 5)            # detections before a cluster is "confirmed"
        self.declare_parameter('collect_radius', 0.30)  # m; /duplo_collected → nearest within this
        self.declare_parameter('map_frame', 'map')

        self.merge_dist = float(self.get_parameter('merge_dist').value)
        self.min_obs = int(self.get_parameter('min_obs').value)
        self.collect_radius = float(self.get_parameter('collect_radius').value)
        self.map_frame = self.get_parameter('map_frame').value

        self.clusters = []

        self.create_subscription(PoseArray, 'duplo_points', self.on_points, 10)
        self.create_subscription(PointStamped, 'duplo_collected', self.on_collected, 10)
        self.pub_map = self.create_publisher(PoseArray, 'duplo_map', 10)
        self.pub_markers = self.create_publisher(MarkerArray, 'duplo_map_markers', 10)
        self.create_timer(0.5, self.publish_map)
        self.get_logger().info('duplo_map started -> /duplo_map')

    def _nearest(self, x, y):
        best, best_d = None, float('inf')
        for c in self.clusters:
            d = math.hypot(c.x - x, c.y - y)
            if d < best_d:
                best, best_d = c, d
        return best, best_d

    def on_points(self, msg: PoseArray):
        for p in msg.poses:
            x, y = p.position.x, p.position.y
            c, d = self._nearest(x, y)
            if c is not None and d <= self.merge_dist:
                c.add(x, y)
            else:
                self.clusters.append(Cluster(x, y))

    def on_collected(self, msg: PointStamped):
        c, d = self._nearest(msg.point.x, msg.point.y)
        if c is not None and d <= self.collect_radius:
            c.collected = True
            self.get_logger().info(f'marked duplo collected @ ({c.x:.2f}, {c.y:.2f})')

    def confirmed_uncollected(self):
        return [c for c in self.clusters if c.n >= self.min_obs and not c.collected]

    def publish_map(self):
        pa = PoseArray()
        pa.header.frame_id = self.map_frame
        pa.header.stamp = self.get_clock().now().to_msg()
        for c in self.confirmed_uncollected():
            pose = Pose()
            pose.position.x, pose.position.y = c.x, c.y
            pose.orientation.w = 1.0
            pa.poses.append(pose)
        self.pub_map.publish(pa)
        self.pub_markers.publish(self._markers())

    def _markers(self):
        ma = MarkerArray()
        for i, c in enumerate(self.clusters):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'duplo_map'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y = c.x, c.y
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.10
            m.scale.z = 0.06
            if c.collected:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.4, 0.4, 0.4, 0.6   # grey
            elif c.n >= self.min_obs:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.9, 0.2, 0.9   # green confirmed
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.9, 0.2, 0.5   # yellow tentative
            ma.markers.append(m)
        return ma


def main():
    rclpy.init()
    node = DuploMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
