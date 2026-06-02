import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch.conditions import IfCondition
from launch.actions import LogInfo

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    j2cdynamics_bringup_dir = get_package_share_directory('j2cdynamics_bringup')

    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map')

    nav2_params_file = os.path.join(j2cdynamics_bringup_dir, 'config', 'nav2_params.yaml')
    collision_monitor_params_file = os.path.join(j2cdynamics_bringup_dir, 'config', 'collision_monitor.yaml')

    executable = PythonExpression([
        "'localization_slam_toolbox_node' if '", mode,
        "' == 'localization' else 'async_slam_toolbox_node'"
    ])

    slam_params = PathJoinSubstitution([
        j2cdynamics_bringup_dir, 'config',
        PythonExpression([
            "'slam_localization.yaml' if '", mode, "' == 'localization' else 'slam_mapping.yaml'"
        ])
    ])

    slam_node = Node(
        package='slam_toolbox',
        executable=executable,
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params, 
            {'map_file_name': map_file},   # ignored in mapping mode
        ],
    )

    nav2 = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'localization'"])
        ),
        actions=[
            SetRemap(src='/cmd_vel_smoothed', dst='/nav_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': nav2_params_file,
                }.items(),
            )
        ]
    )

    # Safety layer — runs in both modes (also brakes the joystick during mapping).
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
        LogInfo(msg=['mode resolved to: ', mode, '  |  map: ', map_file]),
        DeclareLaunchArgument('mode', default_value='mapping', 
                              description="'mapping' (teleop + SLAM, no Nav2) or 'localization' (SLAM-loc + Nav2)"),

        DeclareLaunchArgument('map', default_value='', 
                              description="Path to saved map. Required in localization mode"),
        slam_node, 
        nav2,
        collision_monitor,
        collision_lifecycle,
    ])