"""
Localization layer — provides the map -> odom transform.

  mode:=mapping       slam_toolbox builds a new map (run once, then save).
  mode:=localization  map_server + AMCL against a saved map, plus the keepout
                      filter servers (filter_mask_server + costmap_filter_info_server).

This file owns ONLY "where am I" — the nav2 planning/control stack lives in
navigation.launch.py. do_robot.launch.py includes both.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    bringup_dir = get_package_share_directory('j2cdynamics_bringup')

    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map')
    amcl_file = LaunchConfiguration('amcl_yaml')

    amcl_params_file = PathJoinSubstitution([bringup_dir, 'config', amcl_file])

    is_localization = IfCondition(PythonExpression(["'", mode, "' == 'localization'"]))
    is_mapping      = IfCondition(PythonExpression(["'", mode, "' == 'mapping'"]))

    # ── Mapping mode: slam_toolbox builds the map (run once, then save) ──────────
    slam_mapping = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(bringup_dir, 'config', 'slam_mapping.yaml')],
        condition=is_mapping,
    )

    # ── Localization mode: map_server + AMCL ────────────────────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': [map_file, '.yaml'],   # `map` arg has no extension
        }],
        condition=is_localization,
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params_file],
        condition=is_localization,
    )

    # ── Keepout filter servers (danger zones) ───────────────────────────────────
    # filter_mask_server serves <map>_keepout.yaml; costmap_filter_info_server
    # publishes the CostmapFilterInfo the global costmap's KeepoutFilter consumes.
    filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'frame_id': 'map',
            'topic_name': '/keepout_filter_mask',
            'yaml_filename': [map_file, '_keepout.yaml'],   # convention: next to the map
        }],
        condition=is_localization,
    )

    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'type': 0,                              # 0 = keepout filter
            'filter_info_topic': '/costmap_filter_info',
            'mask_topic': '/keepout_filter_mask',
            'base': 0.0,
            'multiplier': 1.0,
        }],
        condition=is_localization,
    )

    localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl',
                           'filter_mask_server', 'costmap_filter_info_server'],
        }],
        condition=is_localization,
    )

    return LaunchDescription([
        LogInfo(msg=['localization mode: ', mode, '  |  map: ', map_file]),
        DeclareLaunchArgument('mode', default_value='localization',
                              description="'mapping' (slam_toolbox) or "
                                          "'localization' (map_server + AMCL)"),
        DeclareLaunchArgument('map', default_value='',
                              description="Path to the saved map WITHOUT extension "
                                          "(map_server reads <map>.yaml, keepout reads "
                                          "<map>_keepout.yaml). Required in localization mode."),
        DeclareLaunchArgument('amcl_yaml', default_value='amcl.yaml',
                              description="Path to the amcl yaml"),
        slam_mapping,
        map_server,
        amcl,
        filter_mask_server,
        costmap_filter_info_server,
        localization_lifecycle,
    ])
