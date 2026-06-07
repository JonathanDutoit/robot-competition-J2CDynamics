
"""
Publishes system resource stats as a JSON string on /robot_stats so the
dashboard (running on your laptop) can display the *robot's* CPU/mem/temp.
Without this, the dashboard only sees its own host's resources, since psutil
is local-only and ROS topics are the only thing that crosses the network.

Dependencies: rclpy (ROS), psutil (pip).
"""

import json
import os
import socket
import time

import psutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

PUBLISH_HZ = 1.0

def read_pi_temp_c():
    """Best-effort CPU temperature in Celsius. Returns None if unavailable."""
    # Raspberry Pi / most Linux SBCs expose this:
    path = "/sys/class/thermal/thermal_zone0/temp"
    try:
        with open(path) as f:
            return round(int(f.read().strip()) / 1000.0, 1)
    except Exception:
        pass
    # Fallback to psutil sensors (x86 boards, some SBCs)
    try:
        temps = psutil.sensors_temperatures()
        for entries in temps.values():
            if entries:
                return round(entries[0].current, 1)
    except Exception:
        pass
    return None


class RobotStatsPublisher(Node):
    def __init__(self):
        super().__init__("robot_stats_publisher")
        self.pub = self.create_publisher(String, "/robot_stats", 10)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.on_timer)
        self.host = socket.gethostname()
        self.boot = psutil.boot_time()
        # Prime cpu_percent so the first reading isn't 0.0
        psutil.cpu_percent(interval=None)
        psutil.cpu_percent(interval=None, percpu=True)
        self.get_logger().info("robot_stats_publisher started -> /robot_stats")

    def on_timer(self):
        vm = psutil.virtual_memory()
        stats = {
            "host": self.host,
            "cpu_percent": psutil.cpu_percent(interval=None),
            "per_cpu": psutil.cpu_percent(interval=None, percpu=True),
            "mem_percent": vm.percent,
            "mem_used_mb": round(vm.used / 1e6),
            "mem_total_mb": round(vm.total / 1e6),
            "temp_c": read_pi_temp_c(),
            "load_avg": list(os.getloadavg()) if hasattr(os, "getloadavg") else None,
            "uptime_s": round(time.time() - self.boot),
            "stamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(stats)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = RobotStatsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()