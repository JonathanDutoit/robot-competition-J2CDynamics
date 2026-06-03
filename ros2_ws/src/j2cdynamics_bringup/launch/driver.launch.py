import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('j2cdynamics_bringup')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('do_description'),
            'urdf', 'do', 'do.urdf.xacro',
        ]),
        ' use_sim:=false use_ros2_control:=true',
    ])
    robot_description = {'robot_description': robot_description_content}

    controller_params = os.path.join(
        bringup_dir, 'config', 'do_diff_drive_controller.yaml'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_params],
        output='screen',
        remappings=[
            ('/diff_drive_controller/cmd_vel_unstamped', '/cmd_vel'),
            ('/diff_drive_controller/odom', '/odom'),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    delayed_joint_state_spawner = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_diff_drive_spawner = TimerAction(
        period=4.0,
        actions=[diff_drive_controller_spawner],
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[os.path.join(bringup_dir, 'config', 'twist_mux.yaml')],
        remappings=[
            ('cmd_vel_out', '/cmd_vel_muxed'),
        ],
    )

    diagnostics = Node(
        package='j2cdynamics_diagnostics', 
        executable='robot_stats_publisher',
        name='robot_stats_publisher', 
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        delayed_joint_state_spawner,
        delayed_diff_drive_spawner,
        twist_mux, 
        diagnostics
    ])