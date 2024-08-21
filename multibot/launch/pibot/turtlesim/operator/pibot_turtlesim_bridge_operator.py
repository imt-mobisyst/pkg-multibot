import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Get nb robots param from CLI
    nb_robots_launch_arg = DeclareLaunchArgument(
        "nb_robots", default_value=TextSubstitution(text="3")
    )

    # Launch operator nodes only in the operator domain ID (1)
    operator_nodes = GroupAction([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value="1"),

        # Operator node
        Node(
            package='multibot',
            executable='static_operator.py',
            name='operator',
            parameters=[
                {'nb_robots': LaunchConfiguration('nb_robots')}
            ]
        ),

        # Rviz node with specific configuration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('multibot'),
                    'launch/include/rviz_launch.py')),
            ),
    ])



    return LaunchDescription([
        nb_robots_launch_arg,
        
        # Run operator nodes (operator & rviz)
        operator_nodes
    ])