import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import SetEnvironmentVariable
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
    nb_robots_launch_arg = DeclareLaunchArgument(
        "nb_robots", default_value=TextSubstitution(text="3")
    )
    namespace_launch_arg = DeclareLaunchArgument(
        "namespace", default_value=''
    )

    map_launch_arg = DeclareLaunchArgument(
        'map', default_value='warehouse'
    )

    local_dds_server_launch_arg = DeclareLaunchArgument(
        "local_dds_server", default_value=TextSubstitution(text="127.0.0.1:11813")
    )
    subnet_dds_server_launch_arg = DeclareLaunchArgument(
        "subnet_dds_server", default_value=TextSubstitution(text="127.0.0.1:11812")
    )
    sim_dds_server_launch_arg = DeclareLaunchArgument(
        "sim_dds_server", default_value=TextSubstitution(text="127.0.0.1:11811")
    )

    # Create ROS_DISCOVERY_SERVER variables

    def create_common_servers(context):
        local_dds_server  =  LaunchConfiguration('local_dds_server').perform(context)
        subnet_dds_server =  LaunchConfiguration('subnet_dds_server').perform(context)
        sim_dds_server    =  LaunchConfiguration('sim_dds_server').perform(context)
        robot_id          =  int(LaunchConfiguration('robot_id').perform(context))

        # The ROS_DISCOVERY_SERVER variable must list the servers as a list with their ID as index
        # When running on the same machine, we need to add multiple ";" to match the many IDs
        controller_servers = sim_dds_server + ";" + subnet_dds_server + (";"*(robot_id + 1)) + local_dds_server

        return [SetLaunchConfiguration('common_servers', controller_servers)]

    common_servers_arg = OpaqueFunction(function=create_common_servers)


    def create_local_servers(context):
        local_dds_server  =  LaunchConfiguration('local_dds_server').perform(context)
        sim_dds_server    =  LaunchConfiguration('sim_dds_server').perform(context)
        robot_id          =  int(LaunchConfiguration('robot_id').perform(context))

        # The ROS_DISCOVERY_SERVER variable must list the servers as a list with their ID as index
        # When running on the same machine, we need to add multiple ";" to match the many IDs
        controller_servers = sim_dds_server + (";"*(robot_id + 2)) + local_dds_server

        return [SetLaunchConfiguration('local_servers', controller_servers)]

    local_servers_arg = OpaqueFunction(function=create_local_servers)



    # START NODES

    # Start a controller node
    controller_node = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(
            package='communication_test',
            executable='stage_controller.py',
            name='stage_controller',
            parameters=[
                {'robot_id': LaunchConfiguration('robot_id')},
            ]
        )
    ])
    

    def map_path(context):
        map = LaunchConfiguration('map').perform(context)

        file = [os.path.join(get_package_share_directory('communication_test'), 'world', 'maps', map, 'map.yaml')]

        return [SetLaunchConfiguration('map_file', file)]
    
    map_path_arg = OpaqueFunction(function=map_path)

    localization = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('communication_test'),
                            'launch/nav/localization_launch.py')),
            launch_arguments={
                "namespace": LaunchConfiguration('namespace'),
                'map': LaunchConfiguration('map_file'),
                'params_file': os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params.yaml'),
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
                    get_package_share_directory('communication_test'),
                    'launch/nav/navigation_launch.py')),
            launch_arguments={
                "namespace": LaunchConfiguration('namespace'),
                "params_file": os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params.yaml'),
                'log_level': LaunchConfiguration('nav_log_level')
            }.items()
        )
    ])
    
  
    costmapPublisher = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(
            package='communication_test',
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
        nb_robots_launch_arg,
        namespace_launch_arg,
        local_dds_server_launch_arg,
        subnet_dds_server_launch_arg,
        sim_dds_server_launch_arg,

        common_servers_arg,
        local_servers_arg,
        map_launch_arg,
        map_path_arg,

        GroupAction([
            SetEnvironmentVariable(name='ROS_DISCOVERY_SERVER', value=LaunchConfiguration('common_servers')),
            controller_node,
            
            costmapPublisher
        ]),

        GroupAction([
            SetEnvironmentVariable('ROS_DISCOVERY_SERVER', LaunchConfiguration('local_servers')),
            localization,
            nav2,
        ])
    ])
