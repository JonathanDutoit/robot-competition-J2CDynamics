import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

from .odometry_publisher import OdometryPublisher

""" 
Adapted from Samshoni GitHub project
https://github.com/samshoni/ros2-arduino-bridge/tree/main

Arduino Protocol (as implemented in the Arduino):
    Send:    SPEED <left_rad_s> <right_rad_s>\n
    Request: ODOMETRY\n
    Receive: ODOMETRY: L_vel=<f> rad/s R_vel=<f> rad/s 
    Receive: OK: L=<f> R=<f>             (confirmation after SPEED)
    Receive: ERR: <message>              (parse / unknown command errors)
"""
class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.declare_parameter('port', '/dev/arduino')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('serial_rate', 0.1)  # seconds - cmd send rate (10 Hz)
        self.declare_parameter('odom_rate', 0.1)    # seconds - odometry poll rate (10 Hz)
        self.declare_parameter('max_rad_s', 7.0)    # rad/s - wheel velocity limit
        
        # Ramp: max variation of wheel velocity per serial tick
        self.declare_parameter('ramp_step', 0.5)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        rate = self.get_parameter('serial_rate').value
        odom_rate = self.get_parameter('odom_rate').value

        self.max_rad_s = self.get_parameter('max_rad_s').value
        self.ramp_step = self.get_paramter('ramp_step').value

        self.ser = serial.Serial(port, baud, timeout=1)

        self.target_left   = 0.0
        self.target_right  = 0.0
        self.current_left  = 0.0
        self.current_right = 0.0

        self.odom = OdometryPublisher(self)   # /odom + /tf

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmdvel_callback, 10
        )
        self.create_timer(rate, self.send_serial)   
        self.crate_timer(odom_rate, self._poll_odometry)
        self.create_timer(0.2, self.read_serial)    

        self.get_logger().info("Arduino Bridge node started")

    
    def cmdvel_callback(self, msg: Twist):
        """
        Convert a Twist (m/s, rad/s) into left/right wheel velocities (rad/s).
 
        v  = msg.linear.x     [m/s]
        ω  = msg.angular.z    [rad/s]
        wheel_separation L declared in OdometryPublisher params.
 
        v_left  = (v - ω·L/2) / r
        v_right = (v + ω·L/2) / r
        """
        v = msg.linear.x
        w = msg.angular.z
        L = self.odom.wheel_sep    # shared from OdometryPublisher
        r = self.odom.wheel_r
 
        self.target_left  = self._clamp((v - w * L / 2.0) / r)
        self.target_right = self._clamp((v + w * L / 2.0) / r)
        

    def send_serial(self):
        left  = self._ramp(self.current_left,  self.target_left)
        right = self._ramp(self.current_right, self.target_right)
 
        if left != self.current_left or right != self.current_right:
            self.current_left  = left
            self.current_right = right
            cmd = f'SPEED {left:.4f} {right:.4f}\n'
            self.ser.write(cmd.encode('utf-8'))
            self.get_logger().debug(f'Sent: {cmd.strip()}')

    def _poll_odometry(self):
        self.ser.write(b'ODOMETRY\n')

    def _read_serial(self):
        while self.ser.in_waiting:
            raw = self.ser.readline().decode('utf-8', errors='replace').strip()
 
            if raw.startswith('ODOMETRY:'):
                self._handle_odometry(raw)
            elif raw.startswith('OK:'):
                self.get_logger().debug(f'Arduino: {raw}')
            elif raw.startswith('ERR:'):
                self.get_logger().warn(f'Arduino error: {raw}')
            else:
                self.get_logger().info(f'Arduino: {raw}')


    def _handle_odometry(self, line: str):
        """
        Expected format:
          ODOMETRY: L_vel=<f> rad/s R_vel=<f> rad/s
        """
        try:
            # strip "ODOMETRY: " prefix, then tokenise key=value pairs
            tokens = line[len('ODOMETRY:'):].split()
            # tokens: ['L_vel=1.23', 'rad/s', 'R_vel=1.23', 'rad/s']
            l_vel = float(tokens[0].split('=')[1])
            r_vel = float(tokens[2].split('=')[1])
        except (IndexError, ValueError) as e:
            self.get_logger().warn(f'Malformed ODOMETRY line: "{line}" ({e})')
            return
 
        self.odom.update_from_velocities(l_vel, r_vel)


    def _ramp(self, current: float, target: float) -> float:
        delta = target - current
        if abs(delta) <= self.ramp_step:
            return target
        return current + self.ramp_step * (1.0 if delta > 0 else -1.0)
 
    def _clamp(self, value: float) -> float:
        return max(-self.max_rad_s, min(self.max_rad_s, value))

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
