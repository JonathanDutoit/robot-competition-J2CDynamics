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

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        
        self.ser = serial.Serial(port, baud, timeout=1)
        
        # ROS2 subscriber
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmdvel_callback,
            10
        )

        self.get_logger().info("Arduino Bridge node started")

        self.timer = self.create_timer(0.2, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').strip()
            self.get_logger().info(f"Received: {line}")
            

    def cmdvel_callback(self, msg: Twist):
        left_pwm = 6
        right_pwm = 6

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
