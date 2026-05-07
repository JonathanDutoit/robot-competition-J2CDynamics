"""
fake_sim.py
-----------
Publishes fake /scan, /odom, and /tf so that the navigation stack
can be tested without any real hardware or Gazebo.

The robot stays stationary — this is enough to verify that Nav2 and
slam_toolbox boot correctly, receive topics, and activate their
lifecycle nodes.

Run with:
  python3 fake_sim.py
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class FakeSimulator(Node):
    def __init__(self):
        super().__init__('fake_simulator')

        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Publish static transforms once
        self._publish_static_tfs()

        self.create_timer(0.1, self._publish_odom)   # 10 Hz
        self.create_timer(0.1, self._publish_scan)   # 10 Hz

        self.get_logger().info('FakeSimulator running — publishing /scan, /odom, /tf')

    def _publish_static_tfs(self):
        """Publish odom→base_link and base_link→laser as static transforms."""
        now = self.get_clock().now().to_msg()

        # odom → base_link (robot at origin, not moving)
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'odom'
        t1.child_frame_id = 'base_link'
        t1.transform.rotation.w = 1.0

        # base_link → laser (lidar 10 cm above base, centred)
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'laser'
        t2.transform.translation.z = 0.1
        t2.transform.rotation.w = 1.0

        self._static_broadcaster.sendTransform([t1, t2])

    def _publish_odom(self):
        now = self.get_clock().now().to_msg()

        # Dynamic odom → base_link TF (robot stationary at origin)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(t)

        # /odom message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w = 1.0
        # Small non-zero covariance so Nav2 doesn't complain
        odom.pose.covariance[0]  = 1e-3
        odom.pose.covariance[7]  = 1e-3
        odom.pose.covariance[35] = 1e-3
        odom.twist.covariance[0]  = 1e-3
        odom.twist.covariance[35] = 1e-3
        self._odom_pub.publish(odom)

    def _publish_scan(self):
        """
        Publish a fake laser scan — an open circular ring at 2 m distance.
        Gives Nav2 something plausible to build a costmap from.
        """
        now = self.get_clock().now().to_msg()
        num_readings = 360

        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = 'laser'
        scan.angle_min = -math.pi
        scan.angle_max =  math.pi
        scan.angle_increment = 2 * math.pi / num_readings
        scan.time_increment = 0.0
        scan.range_min = 0.1
        scan.range_max = 12.0
        # Uniform circle of obstacles 2 m away — open space around robot
        scan.ranges = [2.0] * num_readings
        scan.intensities = [100.0] * num_readings

        self._scan_pub.publish(scan)


def main():
    rclpy.init()
    node = FakeSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()