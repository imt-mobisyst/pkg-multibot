import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import TextSubstitution, LaunchConfiguration

def rviz(context):
    config_file  =  LaunchConfiguration('config').perform(context)

    path = config_file if config_file.startswith('/') else os.path.join(get_package_share_directory('communication_test'), config_file)

    return [Node(
        package='rviz2',
        executable='rviz2',
        namespace= LaunchConfiguration('namespace'),
        arguments=['-d' + path],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    )]

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=TextSubstitution(text=os.path.join('config', 'turtlesim.rviz'))
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value=TextSubstitution(text='')
        ),
        # Rviz with specific config
        OpaqueFunction(function=rviz)
    ])