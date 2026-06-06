import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_dir = get_package_share_directory('j2cdynamics_bringup')

    nav2_params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    collision_monitor_params_file = os.path.join(bringup_dir, 'config', 'collision_monitor.yaml')

    no_recovery_bt = os.path.join(bringup_dir, 'behavior_trees', 'navigate_to_pose_no_recovery.xml')
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={'default_nav_to_pose_bt_xml': no_recovery_bt},
        convert_types=True,
    )


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
                    'params_file': configured_nav2_params,
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
        nav2,
        collision_monitor,
        collision_lifecycle,
    ])
