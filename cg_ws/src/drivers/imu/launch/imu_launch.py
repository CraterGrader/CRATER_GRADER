from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
  imu_params_path =  os.path.join(
    get_package_share_directory('imu'),
    'config',
    'imu_node_params.yaml'
  )

  with open(imu_params_path, 'r') as stream:
    imu_params = yaml.load(
    stream,
    Loader=yaml.SafeLoader
  )

  nodes = [
    Node(
      package='imu',
      executable='imu_node',
      name='imu_node',
      parameters=[imu_params_path]
    )
  ]

  if imu_params['imu_node']['ros__parameters']['publish_viz']:
    nodes.append(Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2'
    ))

  return LaunchDescription(nodes)
