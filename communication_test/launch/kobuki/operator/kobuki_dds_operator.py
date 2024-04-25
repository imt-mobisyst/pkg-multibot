import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import FindExecutable
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Get nb robots param from CLI
    nb_robots_launch_arg = DeclareLaunchArgument(
        "nb_robots", default_value=TextSubstitution(text="3")
    )


    discoveryServer = ";127.0.0.1:11811"

    # Start FastDDS servers
    DDSserver = ExecuteProcess(
        cmd=[[
            FindExecutable(name='fastdds'),
            ' discovery -i 1 -l 127.0.0.1 -p 11811'
        ]],
        shell=True
    )

    # Launch operator nodes only in the common network
    operator_nodes = GroupAction([
        SetEnvironmentVariable(name='ROS_DISCOVERY_SERVER', value=discoveryServer),

        # Operator node
        Node(
            package='communication_test',
            executable='operator.py',
            name='operator',
            parameters=[
                {'nb_robots': LaunchConfiguration('nb_robots')}
            ]
        ),

        # Rviz node with specific configuration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('communication_test'),
                    'launch/include/rviz_launch.py')),
            ),
    ])



    return LaunchDescription([
        nb_robots_launch_arg,

        # Start common DDS server
        DDSserver,
        
        # Run operator nodes (operator & rviz)
        operator_nodes
    ])