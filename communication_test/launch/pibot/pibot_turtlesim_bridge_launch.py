import os, socket

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    robot_id = int(socket.gethostname()[-2:])
    

    # Create turtle with its own domain ID and bridge
    turtle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('communication_test'),
                'launch/include/robots/turtlesim_bridge_robot_launch.py')),

        # Launch turtles in domain ID relative to their name
        launch_arguments={
            "bot_domain_id": str(robot_id),
            "operator_domain_id": "1"
        }.items()
    )


    return LaunchDescription([
        # Turtles with corresponding bridges
        turtle,
    ])