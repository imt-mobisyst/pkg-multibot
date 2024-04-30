import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  # args that can be set from the command line or a default will be used
  map_file_launch_arg = DeclareLaunchArgument(
    "map_file",
    default_value=TextSubstitution(text=os.path.join(get_package_share_directory('communication_test'),'config/map/map.yaml'))
  )

  load_map_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
                  os.path.join(
                      get_package_share_directory('nav2_bringup'),
                      'launch/localization_launch.py')),
    launch_arguments={
      'map': LaunchConfiguration('map_file'),
      'autostart': 'True'
      }.items()
    )

  return LaunchDescription([
    map_file_launch_arg,

    load_map_launch
  ])