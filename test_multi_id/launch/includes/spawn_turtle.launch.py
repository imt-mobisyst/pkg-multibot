
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    # Args that can be set from the command line or a default will be used
    bot_domain_id_launch_arg = DeclareLaunchArgument(
        "bot_domain_id", default_value=TextSubstitution(text="2")
    )
    operator_domain_id_launch_arg = DeclareLaunchArgument(
        "operator_domain_id", default_value=TextSubstitution(text="1")
    )

    x_pose = DeclareLaunchArgument('x_pose', default_value=TextSubstitution(text='-2.0'))
    y_pose = DeclareLaunchArgument('y_pose', default_value=TextSubstitution(text='-0.5'))

    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value=TextSubstitution(text='true'))



    # Include the launch file for the domain_bridge with given domain ids
    # bridge_launch = GroupAction(
    #     actions=[
    #         IncludeLaunchDescription(
    #             XMLLaunchDescriptionSource(
    #                 os.path.join(
    #                     get_package_share_directory('domain_bridge'),
    #                     'launch/domain_bridge.launch.xml')),
    #             launch_arguments={
    #                 'config':  os.path.join(get_package_share_directory('test_multi_id'), 'config/turtle_gazebo_bridge_config.yaml'),
    #                 'to_domain': LaunchConfiguration('bot_domain_id'),
    #                 'from_domain': LaunchConfiguration('operator_domain_id')
    #             }.items()
    #         )
    #     ]
    # )



    # Start a turtle node 
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose')
        }.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    controller_node = Node(
        package='test_multi_id',
        executable='turtle_gazebo_controller.py',
        name='turtle_gazebo_controller',
        # arguments=['--ros-args', '--log-level', 'DEBUG']
    )

  

    return LaunchDescription([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=LaunchConfiguration('bot_domain_id')),
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle'),
        bot_domain_id_launch_arg,
        operator_domain_id_launch_arg,
        x_pose,
        y_pose,
        use_sim_time,

        # bridge_launch,
        
        spawn_turtlebot_cmd,
        controller_node,
        robot_state_publisher_cmd
    ])
