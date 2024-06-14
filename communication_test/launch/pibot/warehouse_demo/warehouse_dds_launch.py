import os, socket
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    log_level_launch_arg = DeclareLaunchArgument(
        'nav_log_level', default_value='info', description='log level'
    )

    this_ip = socket.gethostbyname(socket.gethostname())

    # CLI arguments
    subnet_dds_server_launch_arg = DeclareLaunchArgument("subnet_dds_server")
    robot_ip_launch_arg = DeclareLaunchArgument('robot_ip',default_value=this_ip)

    # Robot ID
    robot_id = int(socket.gethostname()[-2:])
    namespace = f"robot_{robot_id}"


    # Create ROS_DISCOVERY_SERVER variables

    def create_common_servers(context):
        subnet_dds_server =  LaunchConfiguration('subnet_dds_server').perform(context)
        robot_ip = LaunchConfiguration('robot_ip_launch_arg').perform(context)

        robot_dds_server = robot_ip + ":11811"

        # The ROS_DISCOVERY_SERVER variable must list the servers as a list with their ID as index
        # When running on the same machine, we need to add multiple ";" to match the many IDs
        controller_servers = subnet_dds_server + (";"*robot_id) + (robot_dds_server)

        return [SetLaunchConfiguration('common_servers', controller_servers)]

    common_servers_arg = OpaqueFunction(function=create_common_servers)


    def create_local_servers(context):
        robot_ip = LaunchConfiguration('robot_ip_launch_arg').perform(context)

        robot_dds_server = robot_ip + ":11811"

        # The ROS_DISCOVERY_SERVER variable must list the servers as a list with their ID as index
        # When running on the same machine, we need to add multiple ";" to match the many IDs
        controller_servers = (";"*(robot_id + 1)) + robot_dds_server

        return [SetLaunchConfiguration('local_servers', controller_servers)]

    local_servers_arg = OpaqueFunction(function=create_local_servers)


    # START NODES

    # Start a controller node
    controller_node = GroupAction([
        PushRosNamespace(namespace),
        Node(
            package='communication_test',
            executable='stage_controller.py',
            name='stage_controller',
            parameters=[
                {'robot_id': robot_id},
            ]
        )
    ])
    

    localization = GroupAction([
        PushRosNamespace(namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('nav2_bringup'),
                            'launch/localization_launch.py')),
            launch_arguments={
                "namespace": namespace,
                'map': os.path.join(get_package_share_directory('communication_test'), 'map', 'map.yaml'),
                'params_file': os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params_kobuki.yaml'),
                'autostart': 'True',
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
                "namespace": namespace,
                "params_file": os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params_kobuki.yaml'),
                'log_level': LaunchConfiguration('nav_log_level')
            }.items()
        )
    ])
    
  

    return LaunchDescription([
        log_level_launch_arg,
        
        subnet_dds_server_launch_arg,

        common_servers_arg,
        local_servers_arg,

        GroupAction([
            SetEnvironmentVariable('ROS_DISCOVERY_SERVER', LaunchConfiguration('common_servers')),
            controller_node,
        ]),

        GroupAction([
            SetEnvironmentVariable('ROS_DISCOVERY_SERVER', LaunchConfiguration('local_servers')),
            localization,
            nav2
        ])
    ])
