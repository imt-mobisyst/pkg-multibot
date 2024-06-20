import os, socket
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # CLI arguments
    log_level_launch_arg = DeclareLaunchArgument(
        'nav_log_level', default_value='info', description='log level'
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


    # Start a controller node
    controller_node = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(
            package='communication_test',
            executable='stage_controller.py',
            name='stage_controller',
            parameters=[
                {
                    'robot_id': LaunchConfiguration('id'),
                }
            ]
        )
    ])
    

    localization = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('nav2_bringup'),
                            'launch/localization_launch.py')),
            launch_arguments={
                "namespace": LaunchConfiguration('namespace'),
                'map': os.path.join(get_package_share_directory('communication_test'), 'map', 'map.yaml'),
                'params_file': os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params_kobuki.yaml'),
                'autostart': 'True',
                'log_level': LaunchConfiguration('nav_log_level')
            }.items()
        )
    ])



    nav2 = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('communication_test'),
                    'launch/nav/navigation_launch.py')),
            launch_arguments={
                "namespace": LaunchConfiguration('namespace'),
                "params_file": os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params_kobuki.yaml'),
                "log_level": LaunchConfiguration('nav_log_level'),
                # "use_composition": "True"
            }.items()
        )
    ])
    
  

    return LaunchDescription([
        log_level_launch_arg,
        robot_id_launch_arg,

        robot_id,

        controller_node,

        localization,

        nav2,
        
    ])
