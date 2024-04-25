import os, socket

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import FindExecutable
from launch.substitutions import TextSubstitution
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    # Get nb robots param from CLI
    nb_robots_launch_arg = DeclareLaunchArgument(
        "nb_robots", default_value=TextSubstitution(text="3")
    )

    operator_server_launch_arg = DeclareLaunchArgument("operator_server")

    # Start FastDDS servers
    localDDSserver = ExecuteProcess(
        cmd=[[
            FindExecutable(name='fastdds'),
            ' discovery -i 0 -l 127.0.0.1 -p 11811' #Local server port is 11811
        ]],
        shell=True
    )

    robot_id = int(socket.gethostname()[-2:])

    # Create turtle with its own namespace based on the machine name, and connect to the correct DDS servers
    turtle = GroupAction([
        PushRosNamespace(f'robot{robot_id}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('communication_test'),
                    'launch/include/robots/turtlesim_dds_robot_launch.py')),

            # Launch turtles with the correct DDS configuration
            launch_arguments={
                "local_dds_server": "127.0.0.1:11811",
                "subnet_dds_server": LaunchConfiguration('operator_server'),
                "robot_id": str(robot_id),
                'nb_robots': LaunchConfiguration('nb_robots'),
                "is_local": False
            }.items()
        )
    ])

    return LaunchDescription([
        nb_robots_launch_arg,
        operator_server_launch_arg,

        # Start DDS server
        localDDSserver,

        # Turtle
        turtle,
    ])