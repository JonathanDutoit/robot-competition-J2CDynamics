import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_teleop      = LaunchConfiguration('use_teleop')
    mode            = LaunchConfiguration('mode')
    map_file        = LaunchConfiguration('map')

    bringup_share = get_package_share_directory('j2cdynamics_bringup')
    teleop_launch = os.path.join(bringup_share, 'launch', 'teleop.launch.py')
    localization_launch = os.path.join(bringup_share, 'launch', 'localization.launch.py')
    navigation_launch = os.path.join(bringup_share, 'launch', 'navigation.launch.py')

    nav2_params_file = PathJoinSubstitution([
        bringup_share,
        'config',
        LaunchConfiguration('nav2_yaml')
    ])
    
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch),
        launch_arguments={
            'use_keyboard': 'false',
            'use_joy': 'true',
        }.items(),
        condition=IfCondition(use_teleop)
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            'mode': mode,
            'map': map_file,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            'mode': mode,
            'map': map_file,
            'params_file': nav2_params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_teleop', default_value='false',
            description='Launch joystick teleop alongside robot'),

        DeclareLaunchArgument('mode', default_value='mapping',
            description="'mapping' (SLAM only) or 'localization' (AMCL + Nav2)"),

        DeclareLaunchArgument('map', default_value='',
            description='Path to saved map (no extension). Required in localization mode.'),

        DeclareLaunchArgument('nav2_yaml', default_value='nav2_params.yaml', 
            description="Path to the nav2 parameters yaml"),

        teleop,
        localization,
        navigation
    ])

    