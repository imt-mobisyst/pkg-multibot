import os, socket

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration


def generate_launch_description():

    # Get nb robots param from CLI
    nb_robots_launch_arg = DeclareLaunchArgument(
        "nb_robots", default_value=TextSubstitution(text="3")
    )

    robot_id_launch_arg = DeclareLaunchArgument(
        'robot_id', default_value=None
    )


    # Robot ID
    def getRobotId(context):
        id = LaunchConfiguration('robot_id').perform(context)

        if id is None:
            id = int(socket.gethostname()[-2:]) # Default value = Last number of the kobuki RPI hostname


        namespace = f"robot_{id}"

        print(f'Using robot {id} on namespace "{namespace}"')

        return [
            SetLaunchConfiguration('id', id),
            SetLaunchConfiguration('namespace', namespace)
        ]
    
    robot_id = OpaqueFunction(function=getRobotId)



    # Launch operator nodes only in the common network
    operator_nodes = GroupAction([
        # Operator node
        Node(
            package='communication_test',
            executable='static_operator.py',
            name='operator',
            parameters=[
                {'nb_robots': LaunchConfiguration('nb_robots')}
            ]
        ),

        # Package dispenser
        Node(
            package='communication_test',
            executable='package_dispenser.py',
            parameters=[
                {'is_simulation': False}
            ]
        ),

        

        # Rviz node with specific configuration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('communication_test'), 'launch','include','rviz_launch.py')),
                launch_arguments={
                    "config": os.path.join(get_package_share_directory('communication_test'),'config','real.rviz'),
                    "namespace": LaunchConfiguration('namespace')
                }.items()
        )

    ])



    return LaunchDescription([
        nb_robots_launch_arg,
        robot_id_launch_arg,

        robot_id,
        
        # Run operator nodes (operator & rviz)
        operator_nodes
    ])