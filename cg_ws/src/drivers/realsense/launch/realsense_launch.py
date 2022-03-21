from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  # Get path to the parameters config file
  realsense_params_path = os.path.join(get_package_share_directory('realsense'), 'config/d435i_params.yaml')

  # Pass parameters config file to the realsense2_camera launch file
  included_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('realsense2_camera'), 'launch/rs_launch.py')
    ), 
    launch_arguments={'config_file': f"'{realsense_params_path}'"}.items()
  )  

  # Return the launch file
  return LaunchDescription([
      included_launch
  ])

