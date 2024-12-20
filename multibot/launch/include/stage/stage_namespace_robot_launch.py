import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    log_level_launch_arg = DeclareLaunchArgument(
        'nav_log_level', default_value='info', description='log level'
    )

    # CLI arguments
    robot_id_launch_arg = DeclareLaunchArgument(
        "robot_id", default_value=TextSubstitution(text="1")
    )

    namespace_launch_arg = DeclareLaunchArgument(
        "namespace", default_value=''
    )

    map_launch_arg = DeclareLaunchArgument(
        'map', default_value='warehouse'
    )

    # Start a controller node
    controller_node = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(
            package='multibot',
            executable='stage_controller.py',
            name='stage_controller',
            parameters=[
                {
                    'robot_id': LaunchConfiguration('robot_id'),
                }
            ]
        )
    ])

    def map_path(context):
        map = LaunchConfiguration('map').perform(context)

        file = [os.path.join(get_package_share_directory('multibot'), 'world', 'maps', map, 'map.yaml')]

        return [SetLaunchConfiguration('map_file', file)]
    
    map_path_arg = OpaqueFunction(function=map_path)

    localization = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('multibot'),
                            'launch/nav/localization_launch.py')),
            launch_arguments={
                "namespace": LaunchConfiguration('namespace'),
                'map': LaunchConfiguration('map_file'),
                'params_file': os.path.join(get_package_share_directory('multibot'), 'config', 'nav2', 'nav2_params.yaml'),
                'autostart': 'True',
                'use_sim_time': 'True',
                'log_level': LaunchConfiguration('nav_log_level')
            }.items()
        )
    ])



    nav2 = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('multibot'),
                    'launch/nav/navigation_launch.py')),
            launch_arguments={
                "namespace": LaunchConfiguration('namespace'),
                "params_file": os.path.join(get_package_share_directory('multibot'), 'config', 'nav2', 'nav2_params.yaml'),
                'log_level': LaunchConfiguration('nav_log_level')
            }.items()
        )
    ])


    costmapPublisher = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(
            package='multibot',
            executable='costmap_publisher.py',
            name='costmap_publisher',
            parameters=[{
                'robot_id': LaunchConfiguration('robot_id')
            }]
        )
    ])

    
  

    return LaunchDescription([
        log_level_launch_arg,

        robot_id_launch_arg,
        namespace_launch_arg,
        map_launch_arg,
        map_path_arg,

        controller_node,

        localization,

        nav2,

        costmapPublisher
    ])
