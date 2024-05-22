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
    

    # Start a controller node
    controller_node = GroupAction([
        PushRosNamespace('robot_0'),
        Node(
            package='communication_test',
            executable='stage_controller.py',
            name='stage_controller',
            parameters=[
                {'robot_id': 1}
            ]
        )
    ])

    load_map = GroupAction([
        PushRosNamespace('robot_0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('nav2_bringup'),
                            'launch/localization_launch.py')),
            launch_arguments={
                "namespace": "robot_0",
                'map': os.path.join(get_package_share_directory('communication_test'), 'config', 'maps', 'cave', 'map.yaml'),
                'params_file': os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params.yaml'),
                'autostart': 'True',
                # "use_composition": "False"
            }.items()
        ),
    ])
    # Launch operator node only in the common network
    operator_node = Node(
        package='communication_test',
        executable='static_operator.py',
        name='operator',
        parameters=[
            {'nb_robots': 1}
        ]
    )



    nav2 = GroupAction([
        # PushRosNamespace('robot_0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('communication_test'),
                    'launch/nav/navigation_launch.py')),
            launch_arguments={
                "namespace": "robot_0",
                "params_file": os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params.yaml'),
                # "use_composition": "False"
            }.items()
        )
    ])
    
  

    return LaunchDescription([
        simulator,
        load_map,

        controller_node,
        operator_node,

        nav2
    ])
