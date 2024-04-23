import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    localDDSports = ["11811","11812","11813"]
    subnetDDSport = "11814"
    
    # Start FastDDS servers
    DDSservers = []
    for i, port in enumerate(localDDSports + [subnetDDSport]):
        DDSservers.append(ExecuteProcess(
            cmd=[[
                FindExecutable(name='fastdds'),
                ' discovery -i ',
                str(i),
                ' -l 127.0.0.1 -p ',
                port
            ]],
            shell=True
        ))



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
                            'launch/include/turtlesim_dds_robot_launch.py')),

                    # Launch turtles with the correct DDS configuration
                    launch_arguments={
                        "local_dds_server": "127.0.0.1:" + localDDSports[i],
                        "subnet_dds_server": "127.0.0.1:" + subnetDDSport,
                        "robot_id": str(i+1),
                        'nb_robots': "3"
                    }.items()
                )
            ])
        )

    # Launch operator node only in the common network
    operator_node = GroupAction([
        SetEnvironmentVariable(name='ROS_DISCOVERY_SERVER', value=f";;;127.0.0.1:{subnetDDSport}"),
        Node(
            package='communication_test',
            executable='operator.py',
            name='operator',
            parameters=[
                {'nb_robots': 3}
            ]
        )
    ])

    # Rviz with specific config
    rviz_node = GroupAction([
        SetEnvironmentVariable(name='ROS_DISCOVERY_SERVER', value=f";;;127.0.0.1:{subnetDDSport}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('communication_test'),
                    'launch/include/rviz_launch.py')),
            ),
    ])

    return LaunchDescription([
        # Start DDS servers
        *DDSservers,

        # Turtles
        *turtles,
        
        # Run operator node
        operator_node,
        
        # Rviz with specific config
        rviz_node
    ])