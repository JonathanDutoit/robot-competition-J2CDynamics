#!/usr/bin/env python3
"""
ROS2 Keyboard Teleop — cmd_vel publisher
Controls:
    w / ↑   : forward
    s / ↓   : backward
    a / ←   : turn left
    d / →   : turn right
    space   : stop (immediate)
    q       : quit
"""

import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# --- Tunable speeds ---
LINEAR_SPEED  = 1.0   # m/s
ANGULAR_SPEED = 0.5   # rad/s

BANNER = """
╔══════════════════════════════════╗
║     ROS2 Keyboard Teleop         ║
╠══════════════════════════════════╣
║  w / ↑   : forward               ║
║  s / ↓   : backward              ║
║  a / ←   : turn left             ║
║  d / →   : turn right            ║
║  SPACE   : stop                  ║
║  q       : quit                  ║
╚══════════════════════════════════╝
Linear  speed : {lin} m/s
Angular speed : {ang} rad/s
""".format(lin=LINEAR_SPEED, ang=ANGULAR_SPEED)


def get_key(settings):
    """Read a single keypress (handles arrow escape sequences)."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    if key == '\x1b':                  # escape sequence → arrow key
        key += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def publish(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x  = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)
        print(f"\r  linear.x={linear_x:+.2f}  angular.z={angular_z:+.2f}    ", end='', flush=True)

    def stop(self):
        self.publish(0.0, 0.0)


def main():
    rclpy.init()
    node = TeleopNode()
    settings = termios.tcgetattr(sys.stdin)

    print(BANNER)

    KEY_BINDINGS = {
        'w':       ( LINEAR_SPEED,  0.0),
        '\x1b[A':  ( LINEAR_SPEED,  0.0),   # ↑
        's':       (-LINEAR_SPEED,  0.0),
        '\x1b[B':  (-LINEAR_SPEED,  0.0),   # ↓
        'a':       ( 0.0,  ANGULAR_SPEED),
        '\x1b[D':  ( 0.0,  ANGULAR_SPEED),  # ←
        'd':       ( 0.0, -ANGULAR_SPEED),
        '\x1b[C':  ( 0.0, -ANGULAR_SPEED),  # →
        ' ':       ( 0.0,  0.0),            # stop
    }

    try:
        while True:
            key = get_key(settings)

            if key == 'q' or key == '\x03':   # q or Ctrl-C
                break

            if key in KEY_BINDINGS:
                lin, ang = KEY_BINDINGS[key]
                node.publish(lin, ang)
            else:
                # Unknown key → stop for safety
                node.stop()

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        node.stop()
        print("\nStopped. Bye!")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
