import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler, 
    SetEnvironmentVariable
)

from launch.event_handlers import (
    OnProcessExit,
)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

import ament_index_python.packages 


def generate_launch_description():

    use_sim = LaunchConfiguration('use_sim', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_share = FindPackageShare(package='turtlebot3_description').find('turtlebot3_description')
    urdf = os.path.join(pkg_share, 'urdf', 'turtlebot3_waffle.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'model.rviz')

    gazebo_model_path = os.path.dirname(
        ament_index_python.packages.get_package_share_directory('turtlebot3_description')
    )

    robot_description = {
        'robot_description': Command([
            'xacro ', LaunchConfiguration('model'),
            ' use_sim:=', use_sim        
        ])
    }

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                FindPackageShare('gazebo_ros').find('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ]),
        launch_arguments={
            'verbose': 'true'   
        }.items()
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'turtlebot3', 
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '1',   
                    '-R', '0.0',
                    '-P', '0.0',
                    '-Y', '0.0',],
        output='screen'
    )

    # Controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ]
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager'
        ], 
        remappings=[
        ('/diff_drive_controller/odom', '/odom'),
        ('/diff_drive_controller/cmd_vel', '/cmd_vel'),
    ]
    )

    # RViz
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

        DeclareLaunchArgument(
            name='model', 
            default_value=urdf, 
            description='Absolute path to robot model file'),

        DeclareLaunchArgument(
            name='rvizconfig', 
            default_value=rviz_config, 
            description='Absolute path to rviz config file'),

        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value=use_sim_time, 
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            name='use_sim',
            default_value='true',
            description='Use simulation hardware interface if true'
        ),

        robot_state_publisher_node,
        gazebo, 
        spawn_entity,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ), 

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner]
            )
        ),

        #rviz_node
    ])