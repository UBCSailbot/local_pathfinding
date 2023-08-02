"""Launch file that runs all nodes for the local pathfinding ROS package."""

import importlib
import os
from typing import List

from launch_ros.actions import Node

from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration

# Local launch arguments and constants
PACKAGE_NAME = "local_pathfinding"
NAVIGATE_NODE_NAME = "navigate_main"

# Add args with DeclareLaunchArguments object(s) and utilize in setup_launch()
LOCAL_LAUNCH_ARGUMENTS = []


def generate_launch_description() -> LaunchDescription:
    """The launch file entry point. Generates the launch description for the `network_systems`
    package.

    Returns:
        LaunchDescription: The launch description.
    """
    global_launch_arguments = get_global_launch_arguments()
    local_launch_arguments = LOCAL_LAUNCH_ARGUMENTS
    return LaunchDescription(
        [*global_launch_arguments, *local_launch_arguments, OpaqueFunction(function=setup_launch)]
    )


def get_global_launch_arguments() -> List[LaunchDescriptionEntity]:
    """Gets the global launch arguments defined in the global launch file.

    Returns:
        List[LaunchDescriptionEntity]: List of global launch argument objects.
    """
    global_main_launch = os.path.join(
        os.getenv("ROS_WORKSPACE"), "src", "global_launch", "main_launch.py"
    )
    spec = importlib.util.spec_from_file_location("global_launch", global_main_launch)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    global_launch_arguments = module.GLOBAL_LAUNCH_ARGUMENTS
    return global_launch_arguments


def setup_launch(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    """Collects launch descriptions that describe the system behavior in the `local_pathfinding`
    package.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        List[LaunchDescriptionEntity]: Launch descriptions.
    """
    launch_description_entities = list()
    launch_description_entities.append(get_navigate_node_description(context))
    return launch_description_entities


def get_navigate_node_description(context: LaunchContext) -> Node:
    """Gets the launch description for the navigate_main node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the navigate_main node.
    """
    ros_parameters = [LaunchConfiguration("config").perform(context)]
    ros_arguments = [
        "--log-level",
        [f"{NAVIGATE_NODE_NAME}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable="navigate",
        name=NAVIGATE_NODE_NAME,
        output="screen",
        emulate_tty=True,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node
