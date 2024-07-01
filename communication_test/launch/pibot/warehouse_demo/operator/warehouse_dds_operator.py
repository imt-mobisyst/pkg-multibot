import os, socket

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
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

    # Automatic DDS server
    common_dds_ip = socket.gethostbyname(socket.gethostname())
    common_dds_port = 11811
    
    def setDiscoveryServerEnv(_):

        discoveryServer = f"{common_dds_ip}:{common_dds_port}"

        print(f'-> Starting shared DDS Discovery server : "{discoveryServer}"')

        return [SetEnvironmentVariable(name='ROS_DISCOVERY_SERVER', value=discoveryServer)]




    # Start FastDDS Discovery server
    DDSserver = ExecuteProcess(
        cmd=[[
            FindExecutable(name='fastdds'),
            ' discovery -i 0 -l ',
            common_dds_ip,
            ' -p ',
            str(common_dds_port)
        ]],
        shell=True
    )

    # Launch operator nodes only in the common network
    operator_nodes = GroupAction([
        OpaqueFunction(function=setDiscoveryServerEnv),

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
            executable='package_dispenser.py',
            parameters=[
                {'is_simulation': False}
            ]
        ),

        # Rviz node with specific configuration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('communication_test'),'launch','include','rviz_launch.py')),
                launch_arguments={
                    "config": os.path.join(get_package_share_directory('communication_test'),'config','real.rviz')
                }.items()
            ),
    ])



    return LaunchDescription([
        nb_robots_launch_arg,

        # Start common DDS server
        DDSserver,
        
        # Run operator nodes (operator, package dispenser & rviz)
        operator_nodes
    ])