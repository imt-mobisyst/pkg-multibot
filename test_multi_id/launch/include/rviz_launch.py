import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        # Rviz with specific config
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('test_multi_id'), 'config', 'turtlesim.rviz')]
        )
    ])