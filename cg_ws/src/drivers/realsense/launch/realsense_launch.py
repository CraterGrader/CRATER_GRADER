from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
  realsense_params_path =  os.path.join(
    get_package_share_directory('realsense'),
    'config',
    'd435i_params.yaml'
  )

  with open(realsense_params_path, 'r') as stream:
    realsense_params = yaml.load(
    stream,
    Loader=yaml.SafeLoader
  )

  # TODO: launch the realsense launch file 

  return LaunchDescription(nodes)