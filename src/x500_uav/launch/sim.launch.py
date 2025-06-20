from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('x500_uav')
    world_path = os.path.join(pkg_path, 'worlds', 'terrain.sdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', world_path, '--verbose'],
            output='screen'
        )
    ])
