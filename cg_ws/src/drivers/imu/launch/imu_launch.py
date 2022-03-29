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
  imu_static_tf_params_path = os.path.join(
    get_package_share_directory('imu'),
    'config',
    'imu_static_tf_params.yaml'
  )
  with open(imu_static_tf_params_path, 'r') as stream:
    imu_static_tf_params = yaml.load(
      stream,
      Loader=yaml.SafeLoader
    )['imu_static_transform_publisher']['ros__parameters']

  nodes = [
    Node(
      package='imu',
      executable='imu_node',
      name='imu_node',
      parameters=[imu_params_path]
    ),
    # Static tf publisher
    # Order of args is [x, y, z, yaw, pitch, roll, parent_frame, child_frame]
    # http://docs.ros.org/en/foxy/Tutorials/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html#the-proper-way-to-publish-static-transforms
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=[
        imu_static_tf_params['x'], imu_static_tf_params['y'], imu_static_tf_params['z'],
        imu_static_tf_params['yaw'], imu_static_tf_params['pitch'], imu_static_tf_params['roll'],
        imu_static_tf_params['parent_frame_id'], imu_static_tf_params['child_frame_id']
      ]
    )
  ]

  if imu_params['imu_node']['ros__parameters']['publish_viz']:
    nodes.append(Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2'
    ))

  return LaunchDescription(nodes)
