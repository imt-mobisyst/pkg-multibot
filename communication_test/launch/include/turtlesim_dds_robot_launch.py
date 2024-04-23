
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    # args that can be set from the command line or a default will be used
    local_dds_server_launch_arg = DeclareLaunchArgument(
        "local_dds_server", default_value=TextSubstitution(text="127.0.0.1:11811")
    )
    subnet_dds_server_launch_arg = DeclareLaunchArgument(
        "subnet_dds_server", default_value=TextSubstitution(text="127.0.0.1:11812")
    )
    robot_id_launch_arg = DeclareLaunchArgument(
        "robot_id", default_value=TextSubstitution(text="1")
    )
    


    # Start a turtlesim_node in the local network
    turtlesim_node = GroupAction([
        SetEnvironmentVariable(name='ROS_DISCOVERY_SERVER', value=LaunchConfiguration('local_dds_server')),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            namespace='',
            name='turtle'
        )
    ])

    # Start a controller node in the common network (both DDS servers)
    controller_servers = PythonExpression(["'", LaunchConfiguration('local_dds_server'), ";", LaunchConfiguration('subnet_dds_server'), "'"])
    controller_node = GroupAction([
        SetEnvironmentVariable(name='ROS_DISCOVERY_SERVER', value=controller_servers),
        Node(
            package='communication_test',
            executable='turtlesim_controller.py',
            name='turtlesim_controller',
            parameters=[
                {'robot_id': LaunchConfiguration('robot_id')}
            ]
        )
    ])

  

    return LaunchDescription([
        local_dds_server_launch_arg,
        subnet_dds_server_launch_arg,
        robot_id_launch_arg,

        turtlesim_node,
        controller_node,
    ])
