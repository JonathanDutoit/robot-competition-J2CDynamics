from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
    

def generate_launch_description():
    calibration_file = LaunchConfiguration('calibration_file')

    perception_pkg = get_package_share_directory('j2cdynamics_perception')

    camera_node = Node(
        package='j2cdynamics_camera', 
        executable='camera_node', 
        output='screen'
    )

    ground_projection = Node(
        package='j2cdynamics_perception', 
        executable='ground_projection',
        name='ground_projection', 
        output='screen',
        parameters=[{
            'calibration_file': calibration_file
        }]
    )

    duplo_map = diagnostics = Node(
        package='j2cdynamics_perception', 
        executable='duplo_map',
        name='duplo_map', 
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value='', description='Camera calibration file'),
        camera_node,
        duplo_map,
        ground_projection
    ])