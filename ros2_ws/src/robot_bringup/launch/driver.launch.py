from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arduino_controller',
            executable='arduino_bridge',
            name='arduino_bridge',
            parameters=[{'serial_rate': 0.2}]
        )
    ])
