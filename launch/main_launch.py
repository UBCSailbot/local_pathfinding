import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    global_config = os.path.join(get_package_share_directory('local_pathfinding'), 'globals.yaml')

    log_level = LaunchConfiguration('log_level')

    return LaunchDescription(
        [
            # put node log files in same directory as launch log files
            # ref: https://github.com/ros2/launch/issues/551#issuecomment-982146452
            SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
            # ref: https://answers.ros.org/question/311471/selecting-log-level-in-ros2-launch-file/
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
