import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    global_config = os.path.join(get_package_share_directory('local_pathfinding'), 'globals.yaml')

    return LaunchDescription(
        [
            Node(
                package='local_pathfinding',
                executable='sailbot',
                name='sailbot_launch',
                output='screen',
                emulate_tty=True,
                parameters=[global_config],
            )
        ]
    )
