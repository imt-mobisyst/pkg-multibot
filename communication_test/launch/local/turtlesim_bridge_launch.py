import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def createTurtleNodes(context):
    # Get values
    nb_robots = int(LaunchConfiguration('nb_robots').perform(context))

    turtles = []
    for i in range(nb_robots):
        turtles.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('communication_test'),
                        'launch/include/turtlesims/turtlesim_bridge_robot_launch.py')),

                # Launch turtles in domain ID 10, 11, 12...
                launch_arguments={
                    "bot_domain_id": str(10+i),
                    "operator_domain_id": "1"
                }.items()
            )
        )

    return turtles

def generate_launch_description():
    
    # Get nb robots param from CLI
    nb_robots_launch_arg = DeclareLaunchArgument(
        "nb_robots", default_value=TextSubstitution(text="3")
    )

    # Create turtles with their own domain ID and bridge
    turtles = OpaqueFunction(function=createTurtleNodes)

    # Launch operator node in domain ID 1
    operator_node = Node(
        package='communication_test',
        executable='operator.py',
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

        # Turtles with corresponding bridges
        turtles,
        
        # Run operator in domain ID 1
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value="1"),
        operator_node,
        
        # Rviz with specific config
        rviz_node
    ])