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

    # CLI arguments
    nb_robots_launch_arg = DeclareLaunchArgument(
        "nb_robots", default_value=TextSubstitution(text="3")
    )
    operator_domain_id_launch_arg = DeclareLaunchArgument(
        "operator_domain_id", default_value=TextSubstitution(text="99")
    )

    # Launch operator nodes only in the operator domain ID (1)
    operator_nodes = GroupAction([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=LaunchConfiguration("operator_domain_od")),

        # Operator node
        Node(
            package='communication_test',
            executable='static_operator.py',
            name='operator',
            parameters=[
                {'nb_robots': LaunchConfiguration('nb_robots')}
            ]
        ),

        # Package dispenser
        Node(
            package='communication_test',
            executable='package_dispenser.py'
        ),

        # Rviz node with specific configuration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('communication_test'),'launch','include','rviz_launch.py')),
                launch_arguments={
                    "config": os.path.join(get_package_share_directory('communication_test'),'config','real.rviz'),
                }.items()
            ),
    ])



    return LaunchDescription([
        nb_robots_launch_arg,
        operator_domain_id_launch_arg,
        
        # Run operator nodes (operator, package dispenser & rviz)
        operator_nodes
    ])