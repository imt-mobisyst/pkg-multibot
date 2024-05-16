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


def generate_launch_description():

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('stage_ros2'),
                'launch/stage.launch.py')),
        launch_arguments={
            'world':'cave_three_robots',
            'one_tf_tree':'false'
        }.items()
    )

    # Create turtles with their own namespace 'robotX'
    turtles = []
    for i in range(3):
        turtles.append(
            GroupAction([
                PushRosNamespace(f'robot_{i}'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('communication_test'),
                            'launch/include/stage/stage_namespace_robot_launch.py')),

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
        executable='static_operator.py',
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
        launch_arguments={
            "config": os.path.join(get_package_share_directory('communication_test'), 'config', 'stage.rviz')
        }.items()
    )

    load_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('communication_test'),
                'launch/include/load_map.py')),
        launch_arguments={
            "map_file": os.path.join(get_package_share_directory('communication_test'), 'config', 'maps', 'cave', 'map.yaml')
        }.items()
    )

    return LaunchDescription([
        # Rviz with specific config
        # rviz_node,

        # Launch stage simulator with 3 robots
        simulator,

        # Turtles
        *turtles,

        # Run operator node
        operator_node,

        # Load map
        load_map,
        
    ])