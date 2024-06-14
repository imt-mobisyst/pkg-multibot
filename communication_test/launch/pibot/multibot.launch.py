import os,re,socket

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch.actions import IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource

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




def tbot(context):
    type = LaunchConfiguration('type').perform(context)

    # Get robot informations
    robot_id = int(socket.gethostname()[-2:])
    robot_ip = socket.gethostbyname(socket.gethostname())


    base_launchfile = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(get_package_share_directory('tbot_node'), 'launch', 'minimal_launch.yaml'))
    )

    match(type):
        case "namespace":
            # Add namespace to base launchfile
            return [GroupAction([
                SetRemap(src='/tf',dst='tf'),
                SetRemap(src='/tf_static',dst='tf_static'),
                PushRosNamespace(f"robot_{robot_id}"),
                base_launchfile
            ])]
        


        case "domain_id":
            # Set the correct domain ID to base launchfile
            return [GroupAction([
                SetEnvironmentVariable('ROS_DOMAIN_ID', robot_id),
                base_launchfile
            ])]
        


        case "discovery":
            # Start and set the correct discovery server to base launchfile
            return [GroupAction([
                # Start DDS server
                ExecuteProcess(
                    cmd=[[
                        FindExecutable(name='fastdds'),
                        ' discovery -i ',
                        '1',
                        ' -l ',
                        robot_ip,
                        ' -p ',
                        '11811'
                    ]],
                    shell=True
                ),

                SetEnvironmentVariable('ROS_DISCOVERY_SERVER', robot_id),
                base_launchfile
            ])]
        


        case "partitions":
            # Get configuration file
            config_file_path = os.path.join(get_package_share_directory('communication_test'), 'config', 'dds_partitions','robot_local_config.xml')
            target_path = createConfigFile(config_file_path, robot_id)
    
            # Add partition config to base launchfile
            return [GroupAction([
                SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', "1"),
                SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', target_path),
                base_launchfile
            ])]


    print("Wrong multibot type, must be one of the following : 'namespace', 'domain_id', 'discovery' 'partitions'")
    return []



def generate_launch_description():
    multibot_type_launch_arg = DeclareLaunchArgument(
        'type', description="Must be one of the following : 'namespace', 'domain_id', 'discovery' 'partitions'", default_value=''
    )

    return LaunchDescription([
        multibot_type_launch_arg,

        # Rviz with specific config
        OpaqueFunction(function=tbot)
    ])