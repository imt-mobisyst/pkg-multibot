import os, socket

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    robot_id = int(socket.gethostname()[-2:])

    # Create turtle with its own namespace 'robotX' based on the machine name
    turtle = GroupAction([
        PushRosNamespace(f'robot{robot_id}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('communication_test'),
                    'launch/includeturtlesimsturtlesim_namespace_robot_launch.py')),

            # Launch turtles with the correct DDS configuration
            launch_arguments={
                'robot_id': str(robot_id)
            }.items()
        )
    ])
    return LaunchDescription([
        # Turtle
        turtle
    ])