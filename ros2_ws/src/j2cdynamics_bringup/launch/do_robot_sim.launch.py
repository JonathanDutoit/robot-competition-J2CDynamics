import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

import ament_index_python.packages


def generate_launch_description():
    use_sim          = LaunchConfiguration('use_sim')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_sim_time     = LaunchConfiguration('use_sim_time')

    pkg_share   = FindPackageShare(package='do_description').find('do_description')
    urdf        = os.path.join(pkg_share, 'urdf/do', 'do.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'rviz_config.rviz')
    world_file  = os.path.join(pkg_share, 'worlds', 'empty.world')

    gazebo_model_path = os.path.dirname(
        ament_index_python.packages.get_package_share_directory('do_description')
    )

    robot_description = {
        'robot_description': Command([
            'xacro ', LaunchConfiguration('model'),
            ' use_sim:=', use_sim,
            ' use_ros2_control:=', use_ros2_control
        ])
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]  
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                FindPackageShare('gazebo_ros').find('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ]),
        launch_arguments={
            'verbose': 'true',
            'world': LaunchConfiguration('world')
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'do',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        remappings=[
            ('/diff_drive_controller/odom', '/odom'),
            ('/diff_drive_controller/cmd_vel', '/cmd_vel'),
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=gazebo_model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
        ),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_DATABASE_URI',
            value=''
        ),

        DeclareLaunchArgument('model',            default_value=urdf,
            description='Absolute path to robot model file'),

        DeclareLaunchArgument(name='world',       default_value=world_file,
            description='Absolute path to Gazebo world file'
        ),

        DeclareLaunchArgument('rvizconfig',       default_value=rviz_config,
            description='Absolute path to rviz config file'),

        DeclareLaunchArgument('use_sim_time',     default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument('use_sim',          default_value='true',
            description='Use simulation hardware interface if true'),
            
        DeclareLaunchArgument('use_ros2_control', default_value='true',
            description='Use ros2_control if true, legacy Gazebo plugin if false'),

        robot_state_publisher_node,
        gazebo,
        spawn_entity,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner]
            )
        ),

        # rviz_node,
    ])