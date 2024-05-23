import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource

def createTurtleNodes(context):

    # Get nb robots
    nb_robots = int(LaunchConfiguration('nb_robots').perform(context))

    # Create turtles with their own namespace 'robotX'
    turtles = []
    for i in range(nb_robots):
        turtles.append(
            GroupAction([
                PushRosNamespace(f'robot{i}'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('communication_test'),
                            'launch/include/turtlesims/turtlesim_namespace_robot_launch.py')),

                    # Launch turtles with the correct DDS configuration
                    launch_arguments={
                        'robot_id': str(i)
                    }.items()
                )
            ])
        )

    return turtles

def generate_launch_description():
    # Get nb robots param from CLI
    nb_robots_launch_arg = DeclareLaunchArgument(
        "nb_robots", default_value=TextSubstitution(text="3")
    )

    # Create turtles with their own namespace 'robotX'
    turtles = OpaqueFunction(function=createTurtleNodes)

    # Launch operator node only in the common network
    operator_node = Node(
        package='communication_test',
        executable='static_operator.py',
        name='operator',
        parameters=[
            {'nb_robots': LaunchConfiguration('nb_robots')}
        ]
    )

    # Rviz with specific config
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('communication_test'),
                'launch/include/rviz_launch.py')),
    )

    return LaunchDescription([
        nb_robots_launch_arg,

        # Turtles
        turtles,
        
        # Run operator node
        operator_node,
        
        # Rviz with specific config
        rviz_node
    ])