
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

def generate_launch_description():

    # args that can be set from the command line or a default will be used
    bot_domain_id_launch_arg = DeclareLaunchArgument(
        "bot_domain_id", default_value=TextSubstitution(text="2")
    )
    operator_domain_id_launch_arg = DeclareLaunchArgument(
        "operator_domain_id", default_value=TextSubstitution(text="1")
    )
    # include the launch file for the domain_bridge with given domain ids
    bridge_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('domain_bridge'),
                        'launch/domain_bridge.launch.xml')),
                launch_arguments={
                    'config':  os.path.join(get_package_share_directory('test_multi_id'), 'config/turtlesim_bridge_config.yaml'),
                    'to_domain': LaunchConfiguration('bot_domain_id'),
                    'from_domain': LaunchConfiguration('operator_domain_id')
                }.items()
            )
        ]
    )

    # start a turtlesim_node in the turtlesim1 namespace
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtle'
        # parameters=[{
        #     "background_r": LaunchConfiguration('background_r'),
        # }]
    )

    controller_node = Node(
        package='test_multi_id',
        executable='turtlesim_controller.py',
        name='turtlesim_controller',
        # arguments=['--ros-args', '--log-level', 'DEBUG']
    )

  

    return LaunchDescription([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=LaunchConfiguration('bot_domain_id')),
        bot_domain_id_launch_arg,
        operator_domain_id_launch_arg,

        bridge_launch,
        
        turtlesim_node,
        controller_node,
    ])
