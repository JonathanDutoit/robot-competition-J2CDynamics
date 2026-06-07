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
from launch.conditions import IfCondition

import ament_index_python.packages


def generate_launch_description():
    use_sim          = LaunchConfiguration('use_sim')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_sim_time     = LaunchConfiguration('use_sim_time')
    use_teleop       = LaunchConfiguration('use_teleop')

    pkg_share   = FindPackageShare(package='da_description').find('da_description')
    urdf        = os.path.join(pkg_share, 'urdf/da', 'da.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'rviz_config.rviz')
    world_file  = os.path.join(pkg_share, 'worlds', 'world_with_objects.world')

    bringup_share = FindPackageShare('j2cdynamics_bringup').find('j2cdynamics_bringup')
    teleop_launch = os.path.join(bringup_share, 'launch', 'teleop.launch.py')

    gazebo_model_path = os.path.dirname(
        ament_index_python.packages.get_package_share_directory('da_description')
    )

    robot_description = {
        'robot_description': Command([
            'xacro ', LaunchConfiguration('model'),
            ' use_sim:=', use_sim,
            ' use_ros2_control:=', use_ros2_control
        ])
    }

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[robot_description]
        )

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
            '-entity', 'da',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        condition=IfCondition(use_ros2_control), 
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        remappings=[
            ('/diff_drive_controller/odom', '/odom'),
            ('/diff_drive_controller/cmd_vel', '/cmd_vel'),
        ],
        condition=IfCondition(use_ros2_control),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch),
        launch_arguments={
            'use_keyboard': 'true',
            'use_joy': 'false',
        }.items(),
        condition=IfCondition(use_teleop)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', LaunchConfiguration('rvizconfig')]
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

        DeclareLaunchArgument('use_teleop',       default_value='true',
            description='Launch keyboard teleop alongside sim'),

        joint_state_publisher_node,
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

        teleop,
        rviz_node
    ])