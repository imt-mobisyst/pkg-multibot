import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.substitutions import PythonExpression
from launch.substitutions import FindExecutable
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    local_dds_server_launch_arg = DeclareLaunchArgument(
        "local_dds_server", default_value=TextSubstitution(text="127.0.0.1:11811")
    )
    subnet_dds_server_launch_arg = DeclareLaunchArgument(
        "subnet_dds_server", default_value=TextSubstitution(text="127.0.0.1:11812")
    )
    
    # Start FastDDS servers
    # localDDSserver = ExecuteProcess(
    #     cmd=[[
    #         FindExecutable(name='fastdds'),
    #         ' discovery -i 0 -l 127.0.0.1 -p ',
    #         LaunchConfiguration('local_dds_server')
    #     ]],
    #     shell=True
    # )
    # subnetDDSserver = ExecuteProcess(
    #     cmd=[[
    #         FindExecutable(name='fastdds'),
    #         ' discovery -i 1 -l 127.0.0.1 -p ',
    #         LaunchConfiguration('subnet_dds_server')
    #     ]],
    #     shell=True
    # )



    turtles = []

    # Create turtles with their own namespace 'robotX'
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
                        "local_dds_server": LaunchConfiguration('local_dds_server'),
                        "subnet_dds_server": LaunchConfiguration('subnet_dds_server'),
                        "robot_id": str(i+1)
                    }.items()
                )
            ])
        )

    # Launch operator node only in the common network
    operator_node = GroupAction([
        SetEnvironmentVariable(name='ROS_DISCOVERY_SERVER', value=PythonExpression(["';", LaunchConfiguration('subnet_dds_server'), "'"])),
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
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('communication_test'),
                'launch/include/rviz_launch.py')),
    )

    return LaunchDescription([
        local_dds_server_launch_arg,
        subnet_dds_server_launch_arg,

        # Start DDS servers
        # localDDSserver,
        # subnetDDSserver,

        # Turtles
        *turtles,
        
        # Run operator node
        operator_node,
        
        # Rviz with specific config
        rviz_node
    ])