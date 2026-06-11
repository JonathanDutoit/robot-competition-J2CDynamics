from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
    

def generate_launch_description():
    calibration_file = LaunchConfiguration('calibration_file')

    perception_pkg = get_package_share_directory('j2cdynamics_perception')
    calibration_file_path = PathJoinSubstitution([perception_pkg, 'config', calibration_file])

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
            'calibration_file': calibration_file_path
        }]
    )

    duplo_map = Node(
        package='j2cdynamics_perception',
        executable='duplo_map',
        name='duplo_map',
        output='screen'
    )

    duplo_approach = Node(
        package='j2cdynamics_camera',
        executable='duplo_approach',
        name='duplo_approach',
        output='screen',
        # Same calibration as ground_projection — duplo_approach uses it to
        # ground-project bbox-bottom pixels and reject detections in keepout /
        # behind walls before the FSM even enters approach.
        parameters=[{
            'calibration_file': calibration_file_path
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value='camera_calibration_do.yaml', description='Camera calibration file'),
        camera_node,
        #duplo_map,
        #ground_projection,
        duplo_approach,
    ])