import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
import tf2_ros


class OdometryPublisher:
    """
    Integrates wheel velocities (rad/s) received from the Arduino ODOMETRY
    response into a 2-D pose and publishes:
      - /odom  (nav_msgs/Odometry)
      - /tf    (odom → base_link transform via tf2_ros.TransformBroadcaster)

    Velocity-based integration is used instead of encoder ticks because the
    Arduino firmware (ESCON driver) exposes measured wheel velocities, not
    raw encoder counts.

    Parameters (declare in parent node or launch file):
        wheel_radius        [m]    default 0.033
        wheel_separation    [m]    default 0.16
        odom_frame_id              default "odom"
        base_frame_id              default "base_link"

    Public attributes (read by ArduinoBridge for Twist conversion):
        wheel_r    – wheel radius [m]
        wheel_sep  – wheel separation [m]
    """

    def __init__(self, node: Node):
        self._node = node
        self._declare_params()

        self.wheel_r   = node.get_parameter('wheel_radius').value
        self.wheel_sep = node.get_parameter('wheel_separation').value
        self._odom_fid = node.get_parameter('odom_frame_id').value
        self._base_fid = node.get_parameter('base_frame_id').value

        # 2-D pose
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0

        self._prev_stamp = None   # rclpy.time.Time, set on first update

        # ROS interface
        self._odom_pub      = node.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(node)

        node.get_logger().info('OdometryPublisher ready')

    # ── public API ────────────────────────────────────────────────────────────

    def update_from_velocities(
        self, left_rad_s: float, right_rad_s: float
    ) -> None:
        """
        Integrate wheel velocities into the pose and publish /odom + /tf.

        Args:
            left_rad_s:  left  wheel angular velocity [rad/s]  (from STATUS)
            right_rad_s: right wheel angular velocity [rad/s]  (from STATUS)
        """
        now = self._node.get_clock().now()

        if self._prev_stamp is None:
            self._prev_stamp = now
            return

        dt = (now - self._prev_stamp).nanoseconds * 1e-9
        self._prev_stamp = now

        if dt <= 0.0:
            return

        # wheel velocities → linear velocities at the rim [m/s]
        v_left  = left_rad_s  * self.wheel_r
        v_right = right_rad_s * self.wheel_r

        # differential-drive kinematics
        v_center = (v_left + v_right) / 2.0          # forward velocity [m/s]
        omega    = (v_right - v_left) / self.wheel_sep  # yaw rate        [rad/s]

        d_theta = omega    * dt
        d_x     = v_center * math.cos(self._theta + d_theta / 2.0) * dt
        d_y     = v_center * math.sin(self._theta + d_theta / 2.0) * dt

        self._x     += d_x
        self._y     += d_y
        self._theta  = self._normalise_angle(self._theta + d_theta)

        stamp = now.to_msg()
        self._publish_tf(stamp)
        self._publish_odom(stamp, v_center, omega)

    def _declare_params(self) -> None:
        n = self._node
        n.declare_parameter('wheel_radius',     0.033)
        n.declare_parameter('wheel_separation', 0.16)
        n.declare_parameter('odom_frame_id',    'odom')
        n.declare_parameter('base_frame_id',    'base_link')

    @staticmethod
    def _normalise_angle(angle: float) -> float:
        while angle >  math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def _quat_from_yaw(self) -> tuple:
        """Return (x, y, z, w) quaternion for a pure yaw rotation."""
        return (
            0.0,
            0.0,
            math.sin(self._theta / 2.0),
            math.cos(self._theta / 2.0),
        )

    def _publish_tf(self, stamp) -> None:
        qx, qy, qz, qw = self._quat_from_yaw()

        t = TransformStamped()
        t.header.stamp      = stamp
        t.header.frame_id   = self._odom_fid
        t.child_frame_id    = self._base_fid

        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        t.transform.rotation.x    = qx
        t.transform.rotation.y    = qy
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw

        self._tf_broadcaster.sendTransform(t)

    def _publish_odom(self, stamp, v_linear: float, v_angular: float) -> None:
        qx, qy, qz, qw = self._quat_from_yaw()

        odom = Odometry()
        odom.header.stamp      = stamp
        odom.header.frame_id   = self._odom_fid
        odom.child_frame_id    = self._base_fid

        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x  = v_linear
        odom.twist.twist.angular.z = v_angular

        # Diagonal covariance – tune with your hardware characterisation
        POSE_COV  = 1e-3
        TWIST_COV = 1e-3
        odom.pose.covariance[0]   = POSE_COV   # x
        odom.pose.covariance[7]   = POSE_COV   # y
        odom.pose.covariance[35]  = POSE_COV   # yaw
        odom.twist.covariance[0]  = TWIST_COV  # vx
        odom.twist.covariance[35] = TWIST_COV  # vyaw

        self._odom_pub.publish(odom)