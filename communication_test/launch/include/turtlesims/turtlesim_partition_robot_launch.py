
import os, re

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import SetEnvironmentVariable
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def createConfigFile(originalPath, robotId):
    targetPath = originalPath.replace('/robot_', f'/build/robot{robotId}_')
    os.makedirs(os.path.dirname(targetPath), exist_ok=True) #Create folder if doesn't exist

    # Make substitutions to the original file text
    xmlstring = open(originalPath, 'r').read()
    substitutions = {'robotX': f'robot{robotId}'} 
    pattern = re.compile(r'%([^%]+)%')
    xmlstring = re.sub(pattern, lambda m: substitutions[m.group(1)], xmlstring) # Replace %robotX% by robot9 (if id=9)

    # Save edited file
    f = open(targetPath, "w")
    f.write(xmlstring)
    f.close()

    return targetPath

def create_turtle_nodes(context, *args, **kwargs):
    robot_id =  int(LaunchConfiguration('robot_id').perform(context))

    # Get configuration file
    config_file_path = os.path.join(get_package_share_directory('communication_test'), 'config', 'dds_partitions','robot_local_config.xml')
    target_path = createConfigFile(config_file_path, robot_id)
    
    return [    # Start a turtlesim_node in the local partition
        GroupAction([
            SetEnvironmentVariable(name='RMW_FASTRTPS_USE_QOS_FROM_XML', value="1"),
            SetEnvironmentVariable(name='FASTRTPS_DEFAULT_PROFILES_FILE', value=target_path),
            Node(
                package='turtlesim',
                executable='turtlesim_node',
                namespace='',
                name='turtle',
            ),
            Node(
                package='communication_test',
                executable='turtle_mvt.py',
                name='turtle_mvt'
            )
        ])
    ]

def create_controller_node(context, *args, **kwargs):
    robot_id =  int(LaunchConfiguration('robot_id').perform(context))

    # Get configuration file
    config_file_path = os.path.join(get_package_share_directory('communication_test'), 'config', 'dds_partitions','robot_shared_config.xml')
    target_path = createConfigFile(config_file_path, robot_id)
    
    return [    # Start a turtlesim_node in the local partition
        GroupAction([         
            SetEnvironmentVariable(name='RMW_FASTRTPS_USE_QOS_FROM_XML', value="1"),
            SetEnvironmentVariable(name='FASTRTPS_DEFAULT_PROFILES_FILE', value=target_path),
            Node(
                package='communication_test',
                executable='turtlesim_controller.py',
                name='turtlesim_controller',
                parameters=[
                    {'robot_id': robot_id}
                ]
            )
        ])
    ]

def generate_launch_description():

    # Get nb robots param from CLI
    nb_robots_launch_arg = DeclareLaunchArgument(
        "nb_robots", default_value=TextSubstitution(text="3")
    )

    robot_id_launch_arg = DeclareLaunchArgument(
        "robot_id", default_value=TextSubstitution(text="1")
    )    


    # Start a turtlesim_node in the local network
    turtlesim_node = OpaqueFunction(function=create_turtle_nodes)

    # Start a controller node in the common network (both DDS servers)
    controller_node = OpaqueFunction(function=create_controller_node)
    
  

    return LaunchDescription([
        robot_id_launch_arg,
        nb_robots_launch_arg,

        turtlesim_node,
        controller_node,
    ])
