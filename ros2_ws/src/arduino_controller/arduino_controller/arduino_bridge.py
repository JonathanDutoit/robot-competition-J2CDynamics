import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # Parameters (tune these!)
        self.declare_parameter('port', '/dev/arduino')
        self.declare_parameter('baudrate', 115200)
        #self.declare_parameter('max_pwm', 255)
        #self.declare_parameter('wheel_base', 0.3)  # distance between wheels (meters)
        #self.declare_parameter('max_speed', 1.0)   # m/s scaling

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.ser = serial.Serial(port, baud, timeout=1)

        # ROS2 subscriber
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmdvel_callback,
            10
        )

        self.get_logger().info("CmdVel to Arduino node started")

    def cmdvel_callback(self, msg: Twist):
        """ linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive kinematics
        wheel_base = self.get_parameter('wheel_base').value
        left_speed = linear - (angular * wheel_base / 2.0)
        right_speed = linear + (angular * wheel_base / 2.0)

        # Normalize to PWM
        max_speed = self.get_parameter('max_speed').value
        max_pwm = self.get_parameter('max_pwm').value

        left_pwm = int(max(-1.0, min(1.0, left_speed / max_speed)) * max_pwm)
        right_pwm = int(max(-1.0, min(1.0, right_speed / max_speed)) * max_pwm) """
        left_pwm = 120
        right_pwm = 100

        # Format command
        cmd = f"SPEED {left_pwm} {right_pwm}\n"

        # Send over serial
        self.ser.write(cmd.encode('utf-8'))

        # Debug
        self.get_logger().info(f"Sent: {cmd.strip()}")

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
