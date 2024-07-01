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
    robot_ip_launch_arg = DeclareLaunchArgument('robot_ip', default_value=this_ip)
    robot_port_launch_arg = DeclareLaunchArgument('robot_port', default_value='11811')

    # Robot ID

    # Robot ID
    def setRobotId(context):
        id = LaunchConfiguration('robot_id').perform(context)

        if id == '':
            namespace = ""
            id = int(socket.gethostname()[-2:]) # Default value = Last number of the kobuki RPI hostname
        else:
            namespace = f"robot_{id}"


        robot_config = os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', f'nav2_localization_kobuki_{id}.yaml')

        return [
            SetLaunchConfiguration('id', id),
            SetLaunchConfiguration('namespace', namespace),
            SetLaunchConfiguration('robot_config', robot_config),
        ]
    
    robot_id_setup = OpaqueFunction(function=setRobotId)

    # Create ROS_DISCOVERY_SERVER variables

    def create_common_servers(context):
        subnet_dds_server =  LaunchConfiguration('subnet_dds_server').perform(context)
        robot_ip = LaunchConfiguration('robot_ip').perform(context)
        robot_port = LaunchConfiguration('robot_port').perform(context)
        robot_id = int(LaunchConfiguration('id').perform(context))

        robot_dds_server = robot_ip + ":" + robot_port

        # The ROS_DISCOVERY_SERVER variable must list the servers as a list with their ID as index
        # When running on the same machine, we need to add multiple ";" to match the many IDs
        controller_servers = subnet_dds_server + (";"*robot_id) + (robot_dds_server)

        return [SetLaunchConfiguration('common_servers', controller_servers)]

    common_servers_arg = OpaqueFunction(function=create_common_servers)


    def create_local_servers(context):
        robot_ip = LaunchConfiguration('robot_ip').perform(context)
        robot_port = LaunchConfiguration('robot_port').perform(context)
        robot_id = int(LaunchConfiguration('id').perform(context))

        robot_dds_server = robot_ip + ":" + robot_port

        # The ROS_DISCOVERY_SERVER variable must list the servers as a list with their ID as index
        # When running on the same machine, we need to add multiple ";" to match the many IDs
        controller_servers = (";"*robot_id) + robot_dds_server

        return [SetLaunchConfiguration('local_servers', controller_servers)]

    local_servers_arg = OpaqueFunction(function=create_local_servers)


    # START NODES

    # Start a controller node
    controller_node = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(
            package='communication_test',
            executable='kobuki_warehouse_controller.py',
            name='warehouse_controller',
            parameters=[
                {'robot_id': LaunchConfiguration('id')},
            ]
        )
    ])
    

    localization = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('nav2_bringup'),
                            'launch/localization_launch.py')),
            launch_arguments={
                "namespace": LaunchConfiguration('namespace'),
                'map': os.path.join(get_package_share_directory('communication_test'), 'map', 'map.yaml'),
                'params_file': LaunchConfiguration('robot_config'),
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
                "namespace": LaunchConfiguration('namespace'),
                "params_file": os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params_kobuki.yaml'),
                'log_level': LaunchConfiguration('nav_log_level')
            }.items()
        )
    ])
    
  

    return LaunchDescription([
        log_level_launch_arg,
        
        subnet_dds_server_launch_arg,
        
        robot_ip_launch_arg,
        robot_port_launch_arg,
        robot_id_setup,

        common_servers_arg,
        local_servers_arg,

        DDSserver,


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
