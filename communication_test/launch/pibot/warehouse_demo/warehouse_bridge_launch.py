import os, re, socket
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    log_level_launch_arg = DeclareLaunchArgument(
        'nav_log_level', default_value='info', description='log level'
    )

    # CLI arguments
    operator_domain_id_launch_arg = DeclareLaunchArgument(
        "operator_domain_id", default_value=TextSubstitution(text="99")
    )

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
            SetLaunchConfiguration('robot_config', robot_config)
        ]
    
    robot_id_setup = OpaqueFunction(function=setRobotId)

    # Start a controller node
    controller_node = Node(
        package='communication_test',
        executable='kobuki_warehouse_controller.py',
        name='warehouse_controller',
        parameters=[
            {
                'robot_id': LaunchConfiguration('id')
            }
        ]
    )


    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch/localization_launch.py')),
        launch_arguments={
            'map': os.path.join(get_package_share_directory('communication_test'), 'map', 'map.yaml'),
            'params_file': LaunchConfiguration('robot_config'),
            'autostart': 'True',
            'log_level': LaunchConfiguration('nav_log_level')
        }.items()
    )



    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('communication_test'),
                'launch/nav/navigation_launch.py')),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params_kobuki.yaml'),
            'namespace': '',
            'log_level': LaunchConfiguration('nav_log_level')
        }.items()
    )


    # ------------------------------ Domain bridge ------------------------------ #

    def createConfigFile(originalPath, robotId):
        targetPath = originalPath.replace('stage_bridge_config', f'/build/stage_bridge_config_robot{robotId}')
        os.makedirs(os.path.dirname(targetPath), exist_ok=True) #Create folder if doesn't exist

        # Make substitutions to the original file text
        xmlstring = open(originalPath, 'r').read()
        substitutions = {
            'robot_X': f'robot_{robotId}'
        } 
        pattern = re.compile(r'%([^%]+)%')
        xmlstring = re.sub(pattern, lambda m: substitutions[m.group(1)], xmlstring) # Replace %robot_X% by robot_9 (if id=9)

        # Save edited file
        f = open(targetPath, "w")
        f.write(xmlstring)
        f.close()

        return targetPath



    def createOperatorBridge(context):
        robot_id = LaunchConfiguration('id').perform(context)
        configFilePath = createConfigFile(os.path.join(get_package_share_directory('communication_test'), 'config','domain_bridge','stage_bridge_config_operator.yaml'), robot_id)

        return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('communication_test'),
                'launch/include/domain_bridge.launch.py')),
        launch_arguments={
            'config':  configFilePath,
            'to_domain': robot_id,
            'from_domain': LaunchConfiguration('operator_domain_id'),
            # 'wait_for_subscription': 'false',
            # 'wait_for_publisher': 'false',
        }.items()
    )]


    # Include the launch file to bridge with the operator 
    operator_bridge_launch = OpaqueFunction(function=createOperatorBridge)
    
  

    return LaunchDescription([
        log_level_launch_arg,
        operator_domain_id_launch_arg,
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
