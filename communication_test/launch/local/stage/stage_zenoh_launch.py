import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    log_level_launch_arg = DeclareLaunchArgument(
        'nav_log_level', default_value='info', description='log level'
    )

    map_launch_arg = DeclareLaunchArgument(
        'map', default_value='warehouse'
    )
    

    simulator = GroupAction([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value="100"),

        # Stage simulator
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
        ),

        # Zenoh bridge
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='zenoh-bridge-ros2dds'),
                ' -c ',
                os.path.join(get_package_share_directory('communication_test'), 'config', 'zenoh', 'stage_bridge_config_sim.json5')
            ]],
            shell=True
        )
    ])

    package_dispenser = Node(
        package='communication_test',
        executable='package_dispenser.py'
    )

    # Launch operator node only in the common network
    operator_node = Node(
        package='communication_test',
        executable='warehouse_operator.py',
        name='operator',
        parameters=[
            {'nb_robots': 3}
        ]
    )

    # Create 3 robots with their own namespace 'robotX'
    robots = []
    for i in range(3):
        robots.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('communication_test'),
                        'launch/include/stage/stage_zenoh_robot_launch.py')),

                # Launch turtles with the correct configuration
                launch_arguments={
                    'robot_id': str(i),
                    'map': LaunchConfiguration('map'),

                    "bot_domain_id": str(i),
                    "namespace": f"robot_{i}",

                    "operator_domain_id": "99",
                    "sim_domain_id": "100",

                    'nav_log_level': LaunchConfiguration('nav_log_level')
                }.items()
            )
        )



    return LaunchDescription([
        log_level_launch_arg,
        map_launch_arg,

        # Launch stage simulator with 3 robots in the Domain ID 100
        simulator,

        # Robots
        *robots,

        GroupAction([
            SetEnvironmentVariable(name='ROS_DOMAIN_ID', value="99"),

            # Run operator node
            operator_node,

            # Package dispenser
            package_dispenser,

            # Zenoh bridge
            ExecuteProcess(
                cmd=[[
                    FindExecutable(name='zenoh-bridge-ros2dds'),
                    ' -c ',
                    os.path.join(get_package_share_directory('communication_test'), 'config', 'zenoh', 'stage_bridge_config_operator.json5')
                ]],
                shell=True
            )
        ])
        
    ])