# camera.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='j2cdynamics_camera', executable='camera_node'),
        Node(package='j2cdynamics_camera', executable='duplo_approach'),
    ])