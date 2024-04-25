
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import SetEnvironmentVariable
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def create_turtle_node(context, *args, **kwargs):
    local_dds_server  =  LaunchConfiguration('local_dds_server').perform(context)
    robot_id          =  int(LaunchConfiguration('robot_id').perform(context))
    is_local          = LaunchConfiguration('is_local').perform(context) == "True"

    # The ROS_DISCOVERY_SERVER variable must list the servers as a list with their ID as index
    if is_local:
        # When running in local, we need to add multiple ";" to match the many IDs
        turtle_servers = (";"*(robot_id-1)) + local_dds_server
    else:
        # When running on a kobuki, the local DDS server has ID 0
        turtle_servers = local_dds_server

    
    return [    # Start a turtlesim_node in the local network
        GroupAction([
            SetEnvironmentVariable(name='ROS_DISCOVERY_SERVER', value=turtle_servers),
            Node(
                package='turtlesim',
                executable='turtlesim_node',
                namespace='',
                name='turtle'
            )
        ])
    ]

def create_controller_node(context, *args, **kwargs):
    local_dds_server  =  LaunchConfiguration('local_dds_server').perform(context)
    subnet_dds_server =  LaunchConfiguration('subnet_dds_server').perform(context)
    nb_robots         =  int(LaunchConfiguration('nb_robots').perform(context))
    robot_id          =  int(LaunchConfiguration('robot_id').perform(context))
    is_local          = LaunchConfiguration('is_local').perform(context) == "True"

    # The ROS_DISCOVERY_SERVER variable must list the servers as a list with their ID as index
    if is_local:
        # When running in local, we need to add multiple ";" to match the many IDs
        controller_servers = (";"*(robot_id-1)) + local_dds_server + (";"*(nb_robots+1-robot_id)) + subnet_dds_server
    else:
        # When running on a Kobuki, the local DDS server has ID 0 and the common one has ID 1 
        controller_servers = local_dds_server + ";" + subnet_dds_server

    return [GroupAction([
        SetEnvironmentVariable(name='ROS_DISCOVERY_SERVER', value=controller_servers),
        Node(
            package='communication_test',
            executable='turtlesim_controller.py',
            name='turtlesim_controller',
            parameters=[
                {'robot_id': robot_id}
            ]
        )
    ])]

def generate_launch_description():

    # Get nb robots param from CLI
    nb_robots_launch_arg = DeclareLaunchArgument(
        "nb_robots", default_value=TextSubstitution(text="3")
    )

    is_local_launch_arg = DeclareLaunchArgument(
        "is_local", default_value="True"
    )
    
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
    turtlesim_node = OpaqueFunction(function=create_turtle_node)

    # Start a controller node in the common network (both DDS servers)
    controller_node = OpaqueFunction(function=create_controller_node)
    
  

    return LaunchDescription([
        local_dds_server_launch_arg,
        subnet_dds_server_launch_arg,
        robot_id_launch_arg,
        nb_robots_launch_arg,
        is_local_launch_arg,

        turtlesim_node,
        controller_node,
    ])
