import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import FindExecutable
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource

IP = "127.0.0.1"
SIM_DDS_SERVER_PORT = 11811
SUBNET_DDS_SERVER_PORT = 11812
NB_ROBOTS = 3

def createDDSservers():
    """Create all DDS servers"""

    DDSservers = []
    for i in range(NB_ROBOTS + 2):
        DDSservers.append(ExecuteProcess(
            cmd=[[
                FindExecutable(name='fastdds'),
                ' discovery -i ',
                str(i),
                ' -l ',
                IP,
                ' -p ',
                str(SIM_DDS_SERVER_PORT + i) # Servers start at port 11811
            ]],
            shell=True
        ))

    return DDSservers

def createRobotNodes():
    """Create 3 robot nodes"""

    robots = []
    for i in range(NB_ROBOTS):
        # Get ports of corresponding DDS servers
        localDDSport =  str(SUBNET_DDS_SERVER_PORT + i + 1)

        # Create robots with correct DDS servers
        robots.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('communication_test'),
                        'launch/include/stage/stage_dds_robot_launch.py')),
                # Launch robots with the correct DDS configuration
                launch_arguments={
                    "local_dds_server": f"{IP}:{localDDSport}",
                    "subnet_dds_server": f"{IP}:{SUBNET_DDS_SERVER_PORT}",
                    "sim_dds_server": f"{IP}:{SIM_DDS_SERVER_PORT}",
                    "robot_id": str(i),
                    "namespace": f"robot_{i}",
                    "map": LaunchConfiguration('map'),
                    'nav_log_level': LaunchConfiguration('nav_log_level')
                }.items()
            )
        )

    return robots


def createDebugBridges():
    bridges = []
    for i in range(NB_ROBOTS):

        discovery_servers = f";{IP}:{SUBNET_DDS_SERVER_PORT};" + ";"*i + f"{IP}:" + str(11813+i)

        bridges.append(
            GroupAction([
                SetEnvironmentVariable('ROS_DISCOVERY_SERVER', discovery_servers),
                Node(
                    package="communication_test",
                    executable="stage_dds_bridge.py",
                    name=f"sim_bridge_{i}",
                    parameters=[{'robot_id': i}]
                )
            ])
        )

    return bridges

def generate_launch_description():
    log_level_launch_arg = DeclareLaunchArgument(
        'nav_log_level', default_value='info', description='log level'
    )
        
    map_launch_arg = DeclareLaunchArgument(
        'map', default_value='warehouse'
    )

    # Start FastDDS servers
    DDSservers = createDDSservers()

    # Create robot nodes
    robots = createRobotNodes()


    simulator = GroupAction([
        SetEnvironmentVariable(name='ROS_DISCOVERY_SERVER', value=f"{IP}:{SIM_DDS_SERVER_PORT}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('communication_test'),
                    'launch/include/stage/stage.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('map'),
                'enforce_prefixes':'true',
                'one_tf_tree':'false'
            }.items()
        )
    ])

    debug_bridges = createDebugBridges()

    package_dispenser = Node(
        package='communication_test',
        executable='package_dispenser.py'
    )


    # Launch operator node only in the common network
    operator_node = Node(
        package='communication_test',
        executable='stage_operator.py',
        name='operator',
        parameters=[
            {'nb_robots': NB_ROBOTS}
        ]
    )


    return LaunchDescription([
        log_level_launch_arg,
        map_launch_arg,

        # Start DDS servers
        *DDSservers,

        # Launch stage simulator with 3 robots
        simulator,

        # Start the individual robots
        *robots,

        # Debug bridges
        *debug_bridges,

        GroupAction([
            SetEnvironmentVariable('ROS_DISCOVERY_SERVER', f";{IP}:{SUBNET_DDS_SERVER_PORT}"),
            # Run operator node
            operator_node,

            # Package dispenser
            package_dispenser
        ])
        
    ])