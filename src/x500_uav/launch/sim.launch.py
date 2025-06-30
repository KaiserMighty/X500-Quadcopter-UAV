from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('x500_uav')
    world_path = os.path.join(pkg_path, 'worlds', 'terrain.sdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', world_path, '--verbose'],
            output='screen'
        ),

        Node(
            package='x500_controller',
            executable='flight_controller',
            name='flight_controller',
            output='screen'
        )
    ])
