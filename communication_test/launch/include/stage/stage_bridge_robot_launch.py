import os, re
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # args that can be set from the command line or a default will be used
    robot_id_launch_arg = DeclareLaunchArgument(
        "robot_id", default_value=TextSubstitution(text="1")
    )

    bot_domain_id_launch_arg = DeclareLaunchArgument(
        "bot_domain_id", default_value=TextSubstitution(text="1")
    )
    operator_domain_id_launch_arg = DeclareLaunchArgument(
        "operator_domain_id", default_value=TextSubstitution(text="99")
    )
    sim_domain_id_launch_arg = DeclareLaunchArgument(
        "sim_domain_id", default_value=TextSubstitution(text="100")
    )

    map_launch_arg = DeclareLaunchArgument(
        'map', default_value='warehouse'
    )

    # Start a controller node
    controller_node = Node(
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

    def map_path(context):
        map = LaunchConfiguration('map').perform(context)

        file = [os.path.join(get_package_share_directory('communication_test'), 'world', 'maps', map, 'map.yaml')]

        return [SetLaunchConfiguration('map_file', file)]
    
    map_path_arg = OpaqueFunction(function=map_path)

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('communication_test'),
                        'launch/nav/localization_launch.py')),
        launch_arguments={
            "namespace": '',
            'map': LaunchConfiguration('map_file'),
            'params_file': os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params.yaml'),
            'autostart': 'True',
            'use_sim_time': 'True'
        }.items()
    )



    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('communication_test'),
                'launch/nav/navigation_launch.py')),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory('communication_test'), 'config', 'nav2', 'nav2_params.yaml'),
            'namespace': ''
        }.items()
    )


    # ------------------------------ Domain bridge ------------------------------ #

    def createConfigFile(originalPath, robotId):
        targetPath = originalPath.replace('stage_bridge_config', f'/build/stage_bridge_config_robot{robotId}_')
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
    
    def createSimBridge(context):
        robot_id = LaunchConfiguration('robot_id').perform(context)

        configFilePath = createConfigFile(os.path.join(get_package_share_directory('communication_test'), 'config/domain_bridge/stage_bridge_config_sim.yaml'), robot_id)

        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('communication_test'),
                    'launch/include/domain_bridge.launch.py')),
            launch_arguments={
                'config': configFilePath,
                'to_domain': LaunchConfiguration('bot_domain_id'),
                'from_domain': LaunchConfiguration('sim_domain_id'),
                'wait_for_subscription': 'false',
                'wait_for_publisher': 'false',
            }.items()
        )]

    # Include the launch file to bridge with the simulation 
    sim_bridge_launch = OpaqueFunction(function=createSimBridge)

    def createOperatorBridge(context):
        robot_id = LaunchConfiguration('robot_id').perform(context)

        configFilePath = createConfigFile(os.path.join(get_package_share_directory('communication_test'), 'config/domain_bridge/stage_bridge_config_operator.yaml'), robot_id)

        return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('communication_test'),
                'launch/include/domain_bridge.launch.py')),
        launch_arguments={
            'config':  configFilePath,
            'to_domain': LaunchConfiguration('bot_domain_id'),
            'from_domain': LaunchConfiguration('operator_domain_id'),
            # 'wait_for_subscription': 'false',
            # 'wait_for_publisher': 'false',
        }.items()
    )]


    # Include the launch file to bridge with the operator 
    operator_bridge_launch = OpaqueFunction(function=createOperatorBridge)
    
  

    return LaunchDescription([
        robot_id_launch_arg,
        bot_domain_id_launch_arg,
        operator_domain_id_launch_arg,
        sim_domain_id_launch_arg,
        map_launch_arg,
        map_path_arg,

        sim_bridge_launch,
        operator_bridge_launch,

        GroupAction([
            SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=LaunchConfiguration('bot_domain_id')),
            controller_node,

            localization,

            nav2,
        ])

    ])
