from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='j2cdynamics_driver',
            executable='arduino_bridge',
            name='arduino_bridge',
            parameters=[{'serial_rate': 0.2}]
        )
    ])
