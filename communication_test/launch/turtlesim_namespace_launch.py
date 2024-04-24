import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Create turtles with their own namespace 'robotX'
    turtles = []
    for i in range(3):
        turtles.append(
            GroupAction([
                PushRosNamespace(f'robot{i}'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('communication_test'),
                            'launch/include/turtlesim_namespace_robot_launch.py')),

                    # Launch turtles with the correct DDS configuration
                    launch_arguments={
                        'robot_id': str(i)
                    }.items()
                )
            ])
        )

    # Launch operator node only in the common network
    operator_node = Node(
        package='communication_test',
        executable='operator.py',
        name='operator',
        parameters=[
            {'nb_robots': 3}
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
        # Turtles
        *turtles,
        
        # Run operator node
        operator_node,
        
        # Rviz with specific config
        rviz_node
    ])