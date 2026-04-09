import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

""" 
Adapted from Samshoni GitHub project
https://github.com/samshoni/ros2-arduino-bridge/tree/main
"""
class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.declare_parameter('port', '/dev/arduino')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('serial_rate', 0.1)  # seconds, 10Hz default

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        rate = self.get_parameter('serial_rate').value

        self.ser = serial.Serial(port, baud, timeout=1)

        # Store latest command instead of sending immediately
        self.latest_cmd = "SPEED 0.00 0.00\n"
        self.last_sent_cmd = None  # ← add this

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmdvel_callback, 10
        )

        self.create_timer(rate, self.send_serial)   # serial write timer
        self.create_timer(0.2, self.read_serial)    # serial read timer

        self.get_logger().info("Arduino Bridge node started")

    def cmdvel_callback(self, msg: Twist):
        linear  = msg.linear.x
        angular = msg.angular.z
        MAX_PWM = 100.0
        left_pwm  = max(-MAX_PWM, min(MAX_PWM, (linear - angular) * MAX_PWM))
        right_pwm = max(-MAX_PWM, min(MAX_PWM, (linear + angular) * MAX_PWM))

        
        self.latest_cmd = f"SPEED {left_pwm:.2f} {right_pwm:.2f}\n"

    def send_serial(self):
        if self.latest_cmd != self.last_sent_cmd:  
            self.ser.write(self.latest_cmd.encode('utf-8'))
            self.get_logger().info(f"Sent: {self.latest_cmd.strip()}")
            self.last_sent_cmd = self.latest_cmd

    def read_serial(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').strip()
            self.get_logger().info(f"Received: {line}")

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
