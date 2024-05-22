import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    simulator = GroupAction([
        SetRemap(src='/robot_0/base_scan',dst='/robot_0/scan'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('stage_ros2'),
                    'launch/stage.launch.py')),
            launch_arguments={
                'world':'cave',
                'enforce_prefixes':'true',
                'one_tf_tree':'false'
            }.items()
        )
    ])
    
    # Start 1 controller
    load_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('communication_test'),
                'launch/include/stage/stage_namespace_robot_launch.py')),

        # Launch turtles with the correct DDS configuration
        launch_arguments={
            'namespace': "robot_0",
            'robot_id': "0"
        }.items()
    )


    # Launch operator node only in the common network
    operator_node = Node(
        package='communication_test',
        executable='static_operator.py',
        name='operator',
        parameters=[
            {'nb_robots': 1}
        ]
    )
    
  

    return LaunchDescription([
        simulator,

        load_controller,

        operator_node,
    ])
