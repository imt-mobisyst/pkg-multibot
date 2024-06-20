import os, re, socket
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def createConfigFile(originalPath, robotId):
    targetPath = originalPath.replace('/robot_', f'/build/robot_{robotId}_')
    os.makedirs(os.path.dirname(targetPath), exist_ok=True) #Create folder if doesn't exist

    # Make substitutions to the original file text
    xmlstring = open(originalPath, 'r').read()
    substitutions = {'robotX': f'robot_{robotId}'} 
    pattern = re.compile(r'%([^%]+)%')
    xmlstring = re.sub(pattern, lambda m: substitutions[m.group(1)], xmlstring) # Replace %robotX% by robot_9 (if id=9)

    # Save edited file
    f = open(targetPath, "w")
    f.write(xmlstring)
    f.close()

    return targetPath

def generate_launch_description():
    log_level_launch_arg = DeclareLaunchArgument(
        'nav_log_level', default_value='info', description='log level'
    )


    # Robot ID
    robot_id = int(socket.gethostname()[-2:])
    namespace = f"robot_{robot_id}"



    def local_config_path(context):
        # Get configuration file
        config_file_path = os.path.join(get_package_share_directory('communication_test'), 'config', 'dds_partitions','robot_local_debug_config.xml')
        target_path = createConfigFile(config_file_path, robot_id)

        return [SetLaunchConfiguration('local_config_path', target_path)]

    local_config_path_arg = OpaqueFunction(function=local_config_path)

    def shared_config_path(context):
        # Get configuration file
        config_file_path = os.path.join(get_package_share_directory('communication_test'), 'config', 'dds_partitions','robot_shared_config.xml')
        target_path = createConfigFile(config_file_path, robot_id)

        return [SetLaunchConfiguration('shared_config_path', target_path)]

    shared_config_path_arg = OpaqueFunction(function=shared_config_path)




    # Start a controller node
    controller_node = GroupAction([
        SetEnvironmentVariable(name='RMW_FASTRTPS_USE_QOS_FROM_XML', value="1"),
        SetEnvironmentVariable(name='FASTRTPS_DEFAULT_PROFILES_FILE', value=LaunchConfiguration('shared_config_path')),
        PushRosNamespace(namespace),
        Node(
            package='communication_test',
            executable='kobuki_warehouse_controller.py',
            name='warehouse_controller',
            parameters=[
                {
                    'robot_id': robot_id,
                }
            ]
        )
    ])


    localization = GroupAction([
        PushRosNamespace(namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('nav2_bringup'),
                            'launch/localization_launch.py')),
            launch_arguments={
                "namespace": namespace,
                'map': os.path.join(get_package_share_directory('communication_test'), 'map', 'map.yaml'),
                'params_file': os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params_kobuki.yaml'),
                'autostart': 'True',
                'log_level': LaunchConfiguration('nav_log_level')
            }.items()
        )
    ])



    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('communication_test'),
                'launch/nav/navigation_launch.py')),
        launch_arguments={
            "namespace": namespace,
            "params_file": os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params_kobuki.yaml'),
            'log_level': LaunchConfiguration('nav_log_level')
        }.items()
    )
  


    return LaunchDescription([
        log_level_launch_arg,

        local_config_path_arg,
        shared_config_path_arg,

        controller_node,

        GroupAction([
            SetEnvironmentVariable(name='RMW_FASTRTPS_USE_QOS_FROM_XML', value="1"),
            SetEnvironmentVariable(name='FASTRTPS_DEFAULT_PROFILES_FILE', value=LaunchConfiguration('local_config_path')),
            
            localization,
            nav2
        ])

    ])
