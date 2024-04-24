from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    # args that can be set from the command line or a default will be used
    robot_id_launch_arg = DeclareLaunchArgument(
        "robot_id", default_value=TextSubstitution(text="1")
    )

    # Start a turtlesim_node in the local network
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        namespace='',
        name='turtle'
    )

    # Start a controller node in the common network (both DDS servers)
    controller_node = Node(
        package='communication_test',
        executable='turtlesim_controller.py',
        name='turtlesim_controller',
        parameters=[
            {'robot_id': LaunchConfiguration('robot_id')}
        ]
    )
    

  

    return LaunchDescription([
        robot_id_launch_arg,

        turtlesim_node,
        controller_node,
    ])
