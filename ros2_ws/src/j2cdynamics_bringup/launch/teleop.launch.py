import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    teleop_config = os.path.join(
        get_package_share_directory('j2cdynamics_bringup'),
        'config',
        'teleop_joy.yaml'
    )

    use_keyboard = LaunchConfiguration('use_keyboard')
    use_joy      = LaunchConfiguration('use_joy')

    joy_node = Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                condition=IfCondition(use_joy),
            )

    teleop_joy_node = Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                parameters=[teleop_config],
                condition=IfCondition(use_joy),
                remappings=[('/cmd_vel', '/cmd_vel')]
            )
    
    teleop_keyboard_node = Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_twist_keyboard',
                prefix='xterm -e',   
                output='screen',
                condition=IfCondition(use_keyboard)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_keyboard', 
            default_value='false',
            description='Enable keyboard teleop'
        ),

        joy_node,
        teleop_joy_node,
        teleop_keyboard_node
    ])