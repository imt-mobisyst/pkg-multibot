import os, re, socket
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import FindExecutable
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    log_level_launch_arg = DeclareLaunchArgument(
        'nav_log_level', default_value='info', description='log level'
    )

    # CLI arguments
    operator_ip_launch_arg = DeclareLaunchArgument("operator_ip")

    robot_id_launch_arg = DeclareLaunchArgument(
        'robot_id', default_value=''
    )

    # Robot ID
    def setRobotId(context):
        id = LaunchConfiguration('robot_id').perform(context)

        if id == '':
            id = int(socket.gethostname()[-2:]) # Default value = Last number of the kobuki RPI hostname

        
        print(f'Using robot {id} on domain ID "{id}"')

        robot_config = os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', f'nav2_localization_kobuki_{id}.yaml')

        return [
            SetLaunchConfiguration('id', id),
            SetLaunchConfiguration('namespace', f"robot_{id}"),
            SetLaunchConfiguration('robot_config', robot_config)
        ]
    
    robot_id_setup = OpaqueFunction(function=setRobotId)

    # Start a controller node
    controller_node = GroupAction([        
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(
            package='communication_test',
            executable='kobuki_warehouse_controller.py',
            name='warehouse_controller',
            parameters=[
                {
                    'robot_id': LaunchConfiguration('id')
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
                'namespace': LaunchConfiguration('namespace'),
                'map': os.path.join(get_package_share_directory('communication_test'), 'map', 'map.yaml'),
                'params_file': LaunchConfiguration('robot_config'),
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
            'params_file': os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params_kobuki.yaml'),
            'namespace': LaunchConfiguration('namespace'),
            'log_level': LaunchConfiguration('nav_log_level')
        }.items()
    )


    # ------------------------------ Zenoh bridge ------------------------------- #

    def createConfigFile(originalPath, robotId, operator_ip):
        targetPath = originalPath.replace('bridge_config', f'/build/bridge_config_robot{robotId}')
        os.makedirs(os.path.dirname(targetPath), exist_ok=True) #Create folder if doesn't exist

        # Make substitutions to the original file text
        xmlstring = open(originalPath, 'r').read()
        substitutions = {
            'ID': str(robotId),
            'OPERATOR_IP': operator_ip,
        } 
        pattern = re.compile(r'%([^%]+)%')
        xmlstring = re.sub(pattern, lambda m: substitutions[m.group(1)], xmlstring) # Replace %robot_X% by robot_9 (if id=9)

        # Save edited file
        f = open(targetPath, "w")
        f.write(xmlstring)
        f.close()

        return targetPath



    def createOperatorBridge(context):
        robot_id = int(LaunchConfiguration('robot_id').perform(context))
        operator_ip = LaunchConfiguration('operator_ip').perform(context)

        configFilePath = createConfigFile(os.path.join(get_package_share_directory('communication_test'), 'config', 'zenoh', 'kobuki', 'bridge_config_robot_operator.json5'), robot_id, operator_ip)

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
        operator_ip_launch_arg,
        robot_id_launch_arg,

        robot_id_setup,

        operator_bridge_launch,

        GroupAction([
            SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=LaunchConfiguration('id')),
            controller_node,

            localization,

            nav2,
        ])

    ])
