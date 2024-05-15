from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def generate_launch_description():

    # args that can be set from the command line or a default will be used
    robot_id_launch_arg = DeclareLaunchArgument(
        "robot_id", default_value=TextSubstitution(text="1")
    )

    # Start a controller node
    controller_node = Node(
        package='communication_test',
        executable='stage_controller.py',
        name='stage_controller',
        parameters=[
            {'robot_id': LaunchConfiguration('robot_id')}
        ]
    )

    movement_node = Node(
        package='communication_test',
        executable='stage_mvt.py',
        name='stage_mvt',
    )
    

  

    return LaunchDescription([
        robot_id_launch_arg,

        controller_node,
        movement_node
    ])
