import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource


def createTurtleNodes(context):
    # Get nb robots
    nb_robots = int(LaunchConfiguration('nb_robots').perform(context))

    # Create "nb_robots" turtle nodes
    turtles = []
    for i in range(nb_robots):

        # Create turtles with correct DDS servers
        turtles.append(
            GroupAction([
                PushRosNamespace(f'robot{i+1}'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('communication_test'),
                            'launch/include/turtlesims/turtlesim_partition_robot_launch.py')),

                    # Launch turtles with the correct DDS configuration
                    launch_arguments={
                        "robot_id": str(i+1),
                        'nb_robots': str(nb_robots),
                    }.items()
                )
            ])
        )

    return turtles


def launchOperator(context):
    # Get values
    nb_robots = int(LaunchConfiguration('nb_robots').perform(context))

    operator_config = os.path.join(get_package_share_directory('communication_test'), 'config', 'dds_partitions', 'operator_config.xml')

    # Start nodes on the operator with the subnet DDS server
    nodes = [GroupAction([
        SetEnvironmentVariable(name='RMW_FASTRTPS_USE_QOS_FROM_XML', value="1"),
        SetEnvironmentVariable(name='FASTRTPS_DEFAULT_PROFILES_FILE', value=operator_config),

        # Operator node
        Node(
            package='communication_test',
            executable='static_operator.py',
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

    # Create turtles with their own namespace 'robotX'
    turtles = OpaqueFunction(function=createTurtleNodes)

    # Launch operator nodes only in the common network
    operator_nodes = OpaqueFunction(function=launchOperator)



    return LaunchDescription([
        nb_robots_launch_arg,

        # Turtles
        turtles,
        
        # Run operator nodes (operator & rviz)
        operator_nodes
    ])