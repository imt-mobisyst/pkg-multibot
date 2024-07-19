import os, re
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.substitutions import FindExecutable
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    log_level_launch_arg = DeclareLaunchArgument(
        'nav_log_level', default_value='info', description='log level'
    )

    # CLI arguments
    robot_id_launch_arg = DeclareLaunchArgument(
        "robot_id", default_value=TextSubstitution(text="1")
    )

    bot_domain_id_launch_arg = DeclareLaunchArgument(
        "bot_domain_id", default_value=TextSubstitution(text="1")
    )

    namespace_launch_arg = DeclareLaunchArgument(
        "namespace", default_value=''
    )

    map_launch_arg = DeclareLaunchArgument(
        'map', default_value='warehouse'
    )

    # Start a controller node
    controller_node = GroupAction([        
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(
            package='communication_test',
            executable='stage_controller.py',
            name='stage_controller',
            parameters=[
                {
                    'robot_id': LaunchConfiguration('robot_id'),
                    'world': LaunchConfiguration('map')
                }
            ]
        )
    ])

    def map_path(context):
        map = LaunchConfiguration('map').perform(context)

        file = [os.path.join(get_package_share_directory('communication_test'), 'world', 'maps', map, 'map.yaml')]

        return [SetLaunchConfiguration('map_file', file)]
    
    map_path_arg = OpaqueFunction(function=map_path)

    localization = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('communication_test'),
                            'launch/nav/localization_launch.py')),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'map': LaunchConfiguration('map_file'),
                'params_file': os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params.yaml'),
                'autostart': 'True',
                'use_sim_time': 'True',
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
            'params_file': os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params.yaml'),
            'namespace': LaunchConfiguration('namespace'),
            'log_level': LaunchConfiguration('nav_log_level')
        }.items()
    )

    costmapPublisher = Node(
        package='communication_test',
        executable='costmap_publisher.py',
        name='costmap_publisher',
        parameters=[{
            'robot_id': LaunchConfiguration('robot_id')
        }]
    )


    # ------------------------------ Zenoh bridge ------------------------------ #

    def createConfigFile(originalPath, robotId):
        targetPath = originalPath.replace('bridge_config_robot', f'/build/bridge_config_robot{robotId}')
        os.makedirs(os.path.dirname(targetPath), exist_ok=True) #Create folder if doesn't exist

        # Make substitutions to the original file text
        xmlstring = open(originalPath, 'r').read()
        substitutions = {
            'ID': str(robotId),
            'PORT_SIM': str(7448+robotId),
            'PORT_OPE': str(7458+robotId),
        } 
        pattern = re.compile(r'%([^%]+)%')
        xmlstring = re.sub(pattern, lambda m: substitutions[m.group(1)], xmlstring) # Replace %ID% by 9 (if id=9)

        # Save edited file
        f = open(targetPath, "w")
        f.write(xmlstring)
        f.close()

        return targetPath
    
    def createSimBridge(context):
        robot_id = int(LaunchConfiguration('robot_id').perform(context))

        configFilePath = createConfigFile(os.path.join(get_package_share_directory('communication_test'), 'config', 'zenoh', 'stage', 'bridge_config_robot_sim.json5'), robot_id)


        return [GroupAction([
            SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=LaunchConfiguration('bot_domain_id')),
            ExecuteProcess(
                cmd=[[
                    FindExecutable(name='zenoh-bridge-ros2dds'),
                    ' -c ',
                    configFilePath
                ]],
                shell=True
            ),
        
        ])]

    # Include the launch file to bridge with the simulation 
    sim_bridge_launch = OpaqueFunction(function=createSimBridge)

    def createOperatorBridge(context):
        robot_id = int(LaunchConfiguration('robot_id').perform(context))

        configFilePath = createConfigFile(os.path.join(get_package_share_directory('communication_test'), 'config', 'zenoh', 'stage', 'bridge_config_robot_operator.json5'), robot_id)

        return [GroupAction([
            SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=LaunchConfiguration('bot_domain_id')),
            ExecuteProcess(
                cmd=[[
                    FindExecutable(name='zenoh-bridge-ros2dds'),
                    ' -c ',
                    configFilePath
                ]],
                shell=True
            ),
        
        ])]

    # Include the launch file to bridge with the operator 
    operator_bridge_launch = OpaqueFunction(function=createOperatorBridge)
    
  

    return LaunchDescription([
        log_level_launch_arg,

        robot_id_launch_arg,
        bot_domain_id_launch_arg,
        namespace_launch_arg,
        map_launch_arg,
        map_path_arg,

        sim_bridge_launch,
        operator_bridge_launch,

        GroupAction([
            SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=LaunchConfiguration('bot_domain_id')),
            controller_node,

            localization,

            nav2,

            costmapPublisher
        ])

    ])
