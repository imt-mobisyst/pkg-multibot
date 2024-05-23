import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # args that can be set from the command line or a default will be used
    robot_id_launch_arg = DeclareLaunchArgument(
        "robot_id", default_value=TextSubstitution(text="1")
    )

    namespace_launch_arg = DeclareLaunchArgument(
        "namespace", default_value=''
    )

    # Start a controller node
    controller_node = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(
            package='communication_test',
            executable='stage_controller.py',
            name='stage_controller',
            parameters=[
                {'robot_id': LaunchConfiguration('robot_id')}
            ]
        )
    ])
    

    localization = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('communication_test'),
                            'launch/nav/localization_launch.py')),
            launch_arguments={
                "namespace": LaunchConfiguration('namespace'),
                'map': os.path.join(get_package_share_directory('communication_test'), 'config', 'maps', 'cave', 'map.yaml'),
                'params_file': os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params.yaml'),
                'autostart': 'True',
                'use_sim_time': 'True'
            }.items()
        )
    ])



    nav2 = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('communication_test'),
                    'launch/nav/navigation_launch.py')),
            launch_arguments={
                "namespace": LaunchConfiguration('namespace'),
                "params_file": os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params.yaml'),
            }.items()
        )
    ])
    
  

    return LaunchDescription([
        robot_id_launch_arg,
        namespace_launch_arg,

        controller_node,

        localization,

        nav2
    ])
