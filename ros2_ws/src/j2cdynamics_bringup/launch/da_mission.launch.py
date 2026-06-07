from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='j2cdynamics_mission', executable='da_mission'),
    ])