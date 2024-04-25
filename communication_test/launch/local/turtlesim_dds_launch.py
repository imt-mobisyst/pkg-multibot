import os

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
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource

def createDDSservers(context):
    # Get nb robots
    nb_robots = int(LaunchConfiguration('nb_robots').perform(context))

    # Create all nodes
    DDSservers = []
    for i in range(nb_robots+1):
        DDSservers.append(ExecuteProcess(
            cmd=[[
                FindExecutable(name='fastdds'),
                ' discovery -i ',
                str(i),
                ' -l 127.0.0.1 -p ',
                str(11811 + i) # Servers start at port 11811
            ]],
            shell=True
        ))

    return DDSservers

def createTurtleNodes(context):
    # Get nb robots
    nb_robots = int(LaunchConfiguration('nb_robots').perform(context))

    # Create "nb_robots" turtle nodes
    turtles = []
    for i in range(nb_robots):
        # Get ports of corresponding DDS servers
        localDDSport =  str(11811 + i)
        subnetDDSport =  str(11811 + nb_robots)

        # Create turtles with correct DDS servers
        turtles.append(
            GroupAction([
                PushRosNamespace(f'robot{i}'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('communication_test'),
                            'launch/include/robots/turtlesim_dds_robot_launch.py')),

                    # Launch turtles with the correct DDS configuration
                    launch_arguments={
                        "local_dds_server": "127.0.0.1:" + localDDSport,
                        "subnet_dds_server": "127.0.0.1:" + subnetDDSport,
                        "robot_id": str(i+1),
                        'nb_robots': str(nb_robots),
                        "is_same_machine": str(True)
                    }.items()
                )
            ])
        )

    return turtles


def launchOperator(context):
    # Get values
    nb_robots = int(LaunchConfiguration('nb_robots').perform(context))
    subnetDDSport = str(11811 + nb_robots)

    discoveryServer = ";"*nb_robots + "127.0.0.1:" + subnetDDSport

    # Start nodes on the operator with the subnet DDS server
    nodes = [GroupAction([
        SetEnvironmentVariable(name='ROS_DISCOVERY_SERVER', value=discoveryServer),

        # Operator node
        Node(
            package='communication_test',
            executable='operator.py',
            name='operator',
            parameters=[
                {'nb_robots': nb_robots}
            ]
        ),

        # Rviz node with specific configuration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('communication_test'),
                    'launch/include/rviz_launch.py')),
            ),
    ])]

    return nodes




def generate_launch_description():

    # Get nb robots param from CLI
    nb_robots_launch_arg = DeclareLaunchArgument(
        "nb_robots", default_value=TextSubstitution(text="3")
    )

    # Start FastDDS servers
    DDSservers = OpaqueFunction(function=createDDSservers)

    # Create turtles with their own namespace 'robotX'
    turtles = OpaqueFunction(function=createTurtleNodes)

    # Launch operator nodes only in the common network
    operator_nodes = OpaqueFunction(function=launchOperator)



    return LaunchDescription([
        nb_robots_launch_arg,

        # Start DDS servers
        DDSservers,

        # Turtles
        turtles,
        
        # Run operator nodes (operator & rviz)
        operator_nodes
    ])