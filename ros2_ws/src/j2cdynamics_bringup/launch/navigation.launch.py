import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_dir = get_package_share_directory('j2cdynamics_bringup')

    collision_monitor_params_file = os.path.join(bringup_dir, 'config', 'collision_monitor.yaml')

    # ── Nav2 stack (controller / planner / bt_navigator / smoother) ─────────────
    nav2 = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel_smoothed', dst='/nav_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': params_file,
                }.items(),
            )
        ]
    )

    # ── Safety layer — runs in BOTH modes (brakes the joystick during mapping) ──
    collision_monitor = Node(
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='screen',
        parameters=[collision_monitor_params_file],
    )

    collision_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_safety', output='screen',
        parameters=[{'autostart': True,
                     'node_names': ['collision_monitor']}],
    )

    return LaunchDescription([
        LogInfo(msg=['nav2 params file: ', params_file]),
        DeclareLaunchArgument('params_file', default_value='nav2_params.yaml',
                              description="nav2 parameters file"),
        nav2,
        #collision_monitor,
        collision_lifecycle,
    ])
