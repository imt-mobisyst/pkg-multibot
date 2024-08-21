import os, re, socket
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import FindExecutable
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch.actions import IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    log_level_launch_arg = DeclareLaunchArgument(
        'nav_log_level', default_value='info', description='log level'
    )

    # CLI arguments
    operator_ip_launch_arg = DeclareLaunchArgument("operator_ip")
    robot_ip_launch_arg = DeclareLaunchArgument("robot_ip", default_value="192.168.1.1")

    robot_id_launch_arg = DeclareLaunchArgument(
        'robot_id', default_value=''
    )

    # Robot ID
    def setRobotId(context):
        id = LaunchConfiguration('robot_id').perform(context)

        if id == '':
            id = int(socket.gethostname()[-2:]) # Default value = Last number of the kobuki RPI hostname


        robot_config = os.path.join(get_package_share_directory('multibot'), 'config', 'nav2', f'nav2_localization_kobuki_{id}.yaml')

        return [
            SetLaunchConfiguration('id', id),
            SetLaunchConfiguration('namespace', f"robot_{id}"),
            SetLaunchConfiguration('robot_config', robot_config)
        ]
    
    robot_id_setup = OpaqueFunction(function=setRobotId)

    # Start a controller node
    controller_node = GroupAction([        
        SetRemap(src='/tf',dst='tf'),
        SetRemap(src='/tf_static',dst='tf_static'),
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(
            package='multibot',
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
                'map': os.path.join(get_package_share_directory('multibot'), 'map', 'map.yaml'),
                'params_file': LaunchConfiguration('robot_config'),
                'autostart': 'True',
                'log_level': LaunchConfiguration('nav_log_level')
            }.items()
        )
    ])



    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('multibot'),
                'launch/nav/navigation_launch.py')),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory('multibot'), 'config', 'nav2', 'nav2_params_kobuki.yaml'),
            'namespace': LaunchConfiguration('namespace'),
            'log_level': LaunchConfiguration('nav_log_level')
        }.items()
    )


    # ------------------------------ Zenoh bridge ------------------------------- #

    def createConfigFile(originalPath, robotId, operator_ip, robot_ip):
        targetPath = originalPath.replace('bridge_config', f'/build/bridge_config_robot{robotId}')
        os.makedirs(os.path.dirname(targetPath), exist_ok=True) #Create folder if doesn't exist

        # Make substitutions to the original file text
        xmlstring = open(originalPath, 'r').read()
        substitutions = {
            'ID': str(robotId),
            'OPERATOR_IP': operator_ip,
            'PIBOT_IP': robot_ip,
        } 
        pattern = re.compile(r'%([^%]+)%')
        xmlstring = re.sub(pattern, lambda m: substitutions[m.group(1)], xmlstring) # Replace values

        # Save edited file
        f = open(targetPath, "w")
        f.write(xmlstring)
        f.close()

        return targetPath



    def createOperatorBridge(context):
        robot_id = int(LaunchConfiguration('robot_id').perform(context))
        operator_ip = LaunchConfiguration('operator_ip').perform(context)
        robot_ip = LaunchConfiguration('robot_ip').perform(context)

        configFilePath = createConfigFile(
            os.path.join(get_package_share_directory('multibot'), 'config', 'zenoh', 'kobuki', 'bridge_config_robot.json5'),
            robot_id, operator_ip, robot_ip
        )

        return [GroupAction([
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
        robot_ip_launch_arg,
        robot_id_launch_arg,

        robot_id_setup,


        GroupAction([
            # Launch all nodes using only the local network interface
            SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
            SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET'),
            SetEnvironmentVariable('CYCLONEDDS_URI', 
                os.path.join(get_package_share_directory('multibot'), 'config', 'zenoh', 'kobuki', 'local_cyclonedds.xml')),
            
            operator_bridge_launch,

            controller_node,

            localization,

            nav2,
        ])

    ])
