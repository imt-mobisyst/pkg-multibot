import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import TextSubstitution, LaunchConfiguration

def rviz(context):
    config_file  =  LaunchConfiguration('config').perform(context)
    return [Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d' + config_file]
    )]

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=TextSubstitution(text=os.path.join(get_package_share_directory('communication_test'), 'config', 'turtlesim.rviz'))
        ),
        # Rviz with specific config
        OpaqueFunction(function=rviz)
    ])