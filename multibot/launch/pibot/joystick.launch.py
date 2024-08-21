import os,re,socket

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import PushRosNamespace
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace'
    )

    return LaunchDescription([
        namespace_launch_arg,

        # Joystick with namespace
        GroupAction([
            PushRosNamespace(LaunchConfiguration('namespace')),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(os.path.join(get_package_share_directory('basic_node'), 'launch', 'joystick.yaml'))
            )
        ])
    ])