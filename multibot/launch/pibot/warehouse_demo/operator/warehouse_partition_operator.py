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


    # Launch operator nodes only listening and publishing to the "shared" partition
    operator_nodes = GroupAction([
        SetEnvironmentVariable(name='FASTRTPS_DEFAULT_PROFILES_FILE',
                               value=os.path.join(get_package_share_directory('multibot'),'config','dds_partitions','operator_config.xml')),

        # Operator node
        Node(
            package='multibot',
            executable='static_operator.py',
            name='operator',
            parameters=[
                {'nb_robots': LaunchConfiguration('nb_robots')}
            ]
        ),

        # Package dispenser
        Node(
            package='multibot',
            executable='package_dispenser.py',
            parameters=[
                {'is_simulation': False}
            ]
        ),

        # Rviz node with specific configuration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('multibot'),'launch','include','rviz_launch.py')),
                launch_arguments={
                    "config": os.path.join(get_package_share_directory('multibot'),'config','real.rviz')
                }.items()
            ),
    ])



    return LaunchDescription([
        nb_robots_launch_arg,

        # Run operator nodes (operator, package dispenser & rviz)
        operator_nodes
    ])