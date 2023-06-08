import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    global_config = os.path.join(get_package_share_directory('local_pathfinding'), 'globals.yaml')

    log_level = LaunchConfiguration('log_level')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='log_level',
                default_value=['info'],
                description='Logging level',
            ),
            Node(
                package='local_pathfinding',
                executable='navigate',
                name='navigate_main',
                output='screen',
                emulate_tty=True,
                parameters=[global_config],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    ["navigate_main:=", log_level],
                ],
            ),
        ]
    )
