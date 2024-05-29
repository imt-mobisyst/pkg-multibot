import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    map_launch_arg = DeclareLaunchArgument(
        'map', default_value='warehouse'
    )

    simulator = GroupAction([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value="100"),
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

    package_dispenser = Node(
        package='communication_test',
        executable='package_dispenser.py'
    )

    # Launch operator node only in the common network
    operator_node = Node(
        package='communication_test',
        executable='static_operator.py',
        name='operator',
        parameters=[
            {'nb_robots': 3}
        ]
    )

    # Create 3 turtles with their own namespace 'robotX'
    turtles = []
    for i in range(3):
        turtles.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('communication_test'),
                        'launch/include/stage/stage_bridge_robot_launch.py')),

                # Launch turtles with the correct DDS configuration
                launch_arguments={
                    'robot_id': str(i),
                    'map': LaunchConfiguration('map'),

                    "bot_domain_id": str(i),
                    "operator_domain_id": "99",
                    "sim_domain_id": "100"
                }.items()
            )
        )


    # Rviz with specific config
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('communication_test'),
                'launch/include/rviz_launch.py')),
        launch_arguments={
            "config": os.path.join('config', 'stage.rviz')
        }.items()
    )



    return LaunchDescription([
        map_launch_arg,

        # Rviz with specific config
        # rviz_node,

        # Launch stage simulator with 3 robots in the Domain ID 100
        simulator,

        # Turtles
        *turtles,

        GroupAction([
            SetEnvironmentVariable(name='ROS_DOMAIN_ID', value="99"),

            # Run operator node
            operator_node,

            # Package dispenser
            package_dispenser
        ])
        
    ])