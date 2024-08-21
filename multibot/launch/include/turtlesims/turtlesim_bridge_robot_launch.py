
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
    is_same_machine_launch_arg = DeclareLaunchArgument(
        "is_same_machine", default_value="True"
    )

    # CLI arguments
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
                    'config':  os.path.join(get_package_share_directory('multibot'), 'config/domain_bridge/turtlesim_bridge_config.yaml'),
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
    )

    turtle_mvt_node = Node(        
        package='multibot',
        executable='turtle_mvt.py',
        name='turtle_mvt',
    )

    controller_node = Node(
        package='multibot',
        executable='turtlesim_controller.py',
        name='turtlesim_controller',
        parameters=[
            {'robot_id': LaunchConfiguration('bot_domain_id')}
        ]
    )

  

    return LaunchDescription([
        is_same_machine_launch_arg,
        bot_domain_id_launch_arg,
        operator_domain_id_launch_arg,

        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=LaunchConfiguration('bot_domain_id')),

        bridge_launch,
        
        turtlesim_node,
        turtle_mvt_node,
        controller_node,
    ])
