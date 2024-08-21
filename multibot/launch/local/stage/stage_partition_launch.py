import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    log_level_launch_arg = DeclareLaunchArgument(
        'nav_log_level', default_value='info', description='log level'
    )

    map_launch_arg = DeclareLaunchArgument(
        'map', default_value='warehouse'
    )

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('multibot'),
                'launch/include/stage/stage.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('map'),
            'enforce_prefixes':'true',
            'one_tf_tree':'false'
        }.items()
    )

    package_dispenser = Node(
        package='multibot',
        executable='package_dispenser.py'
    )

    # Create 3 robots with their own namespace 'robot_X'
    robots = []
    for i in range(3):
        robots.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('multibot'),
                        'launch/include/stage/stage_partition_robot_launch.py')),

                # Launch robots with the correct DDS configuration
                launch_arguments={
                    'namespace': f"robot_{i}",
                    'robot_id': str(i),
                    'map': LaunchConfiguration('map'),
                    'nav_log_level': LaunchConfiguration('nav_log_level')
                }.items()
            )
        )

    # Launch operator node only in the common network
    operator_node = Node(
        package='multibot',
        executable='warehouse_operator.py',
        name='operator',
        parameters=[
            {'nb_robots': 3}
        ]
    )


    return LaunchDescription([
        log_level_launch_arg,
        map_launch_arg,

        # Launch stage simulator with 3 robots
        GroupAction([
            SetEnvironmentVariable(name='RMW_FASTRTPS_USE_QOS_FROM_XML', value="1"),
            SetEnvironmentVariable(name='FASTRTPS_DEFAULT_PROFILES_FILE', value=os.path.join(get_package_share_directory('multibot'), 'config', 'dds_partitions', 'stage_config.xml')),
            
            simulator
        ]),

        # Robots
        *robots,

        GroupAction([
            SetEnvironmentVariable(name='RMW_FASTRTPS_USE_QOS_FROM_XML', value="1"),
            SetEnvironmentVariable(name='FASTRTPS_DEFAULT_PROFILES_FILE', value=os.path.join(get_package_share_directory('multibot'), 'config', 'dds_partitions', 'operator_config.xml')),
            
            # Run operator node
            operator_node,

            # Package dispenser
            package_dispenser
        ])
        
    ])