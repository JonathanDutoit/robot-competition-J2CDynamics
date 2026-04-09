from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[
                '/ros2_ws/src/robot_bringup/config/teleop_joy.yaml'
            ]
        ),
        Node(
            package='arduino_controller',
            executable='arduino_bridge',
            name='arduino_bridge',
            parameters=[{'serial_rate': 0.2}]
        ),
        Node(
            package='camera_stream',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
    ])
