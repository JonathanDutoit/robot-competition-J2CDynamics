import os

from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, LogInfo
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    FindExecutable,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bringup_dir = get_package_share_directory('j2cdynamics_bringup')

    # -------------------------
    # Robot name (da / do)
    # -------------------------
    robot_name = LaunchConfiguration('robot_name', default='do')

    # -------------------------
    # Dynamic package name: da_description / do_description
    # -------------------------
    description_pkg = PythonExpression([
        "str('", robot_name, "') + '_description'"
    ])

    # -------------------------
    # Dynamic xacro filename: da.urdf.xacro / do.urdf.xacro
    # -------------------------
    xacro_file = PythonExpression([
        "str('", robot_name, "') + '.urdf.xacro'"
    ])

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare(description_pkg),
            'urdf',
            robot_name,
            xacro_file
        ]),
        ' use_sim:=false use_ros2_control:=true'
    ])

    robot_description = {'robot_description': robot_description_content}

    # -------------------------
    # Controller YAML: da_diff_drive_controller.yaml / do_...
    # -------------------------
    controller_params = PathJoinSubstitution([
        bringup_dir,
        'config',
        PythonExpression([
            "str('", robot_name, "') + '_diff_drive_controller.yaml'"
        ])
    ])

    # -------------------------
    # Robot state publisher
    # -------------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # -------------------------
    # ros2_control node
    # -------------------------
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

    # -------------------------
    # Spawners
    # -------------------------
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

    # -------------------------
    # Twist mux
    # -------------------------
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[
            PathJoinSubstitution([
                bringup_dir,
                'config',
                'twist_mux.yaml'
            ])
        ],
        remappings=[
            ('cmd_vel_out', '/cmd_vel'),
        ],
    )

    # -------------------------
    # Diagnostics
    # -------------------------
    diagnostics = Node(
        package='j2cdynamics_diagnostics',
        executable='robot_stats_publisher',
        name='robot_stats_publisher',
        output='screen'
    )

    # -------------------------
    # Launch description
    # -------------------------
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='do',
            description='Name of the robot (da or do)',
        ),

        robot_state_publisher_node,
        controller_manager_node,

        delayed_joint_state_spawner,
        delayed_diff_drive_spawner,

        twist_mux,
        diagnostics,
    ])