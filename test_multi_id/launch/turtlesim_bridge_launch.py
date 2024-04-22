import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    turtles = []

    for i in range(3):
        turtles.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('test_multi_id'),
                        'launch/include/turtlesim_robot_launch.py')),

                # Launch turtles in domain ID 10, 11 and 12
                launch_arguments={
                    "bot_domain_id": str(10+i),
                    "operator_domain_id": "1"
                }.items()
            )
        )

    # Launch operator node in domain ID 1
    operator_node = Node(
        package='test_multi_id',
        executable='operator.py',
        name='operator',
        parameters=[
            {'nb_robots': 3}
        ]
    )

    # Rviz with specific config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('test_multi_id'), 'config', 'turtlesim.rviz')]
    )

    return LaunchDescription([
        # Turtles with corresponding bridges
        *turtles,
        
        # Run operator in domain ID 1
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value="1"),
        operator_node,
        
        # Rviz with specific config
        rviz_node
    ])