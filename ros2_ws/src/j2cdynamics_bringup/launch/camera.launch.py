from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='j2cdynamics_camera', executable='camera_node', output='screen', emulate_tty=True),
        Node(package='j2cdynamics_camera', executable='duplo_approach', output='screen', emulate_tty=True),
    ])