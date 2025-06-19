from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    model_path = os.path.join(
        os.getenv('HOME'), 'X500-Quadcopter-UAV', 'src', 'x500', 'models', 'model.sdf'
    )

    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', model_path, '-entity', 'my_drone'],
            output='screen'
        )
    ])
