from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

def createTurtleNode(context):
    is_same_machine = LaunchConfiguration('is_same_machine').perform(context) == "True"

    if is_same_machine :
        return Node(
            package='turtlesim',
            executable='turtlesim_node',
            namespace='',
            name='turtle',
        )
    else:
        return Node(
            package='turtlesim',
            executable='turtlesim_node',
            namespace='',
            name='turtle',
            arguments=['-platform', 'offscreen']
        )

def generate_launch_description():

    is_same_machine_launch_arg = DeclareLaunchArgument(
        "is_same_machine", default_value="True"
    )

    # args that can be set from the command line or a default will be used
    robot_id_launch_arg = DeclareLaunchArgument(
        "robot_id", default_value=TextSubstitution(text="1")
    )

    # Start a turtlesim_node in the local network
    turtlesim_node = OpaqueFunction(function=createTurtleNode)


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
        is_same_machine_launch_arg,
        robot_id_launch_arg,

        turtlesim_node,
        controller_node,
    ])
