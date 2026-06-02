import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    laser_filter_config = os.path.join(
        get_package_share_directory('j2cdynamics_bringup'),  
        'config',
        'laser_filter.yaml'
    )
    return LaunchDescription([
         Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/lidar',
                'frame_id': 'laser',
                'serial_baudrate': 115200,
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        ), 
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            output='screen',
            parameters=[laser_filter_config],
            remappings=[
                ('scan', '/scan'),                   # input from rplidar
                ('scan_filtered', '/scan_filtered')  # output for SLAM
            ]
        )
    ])
