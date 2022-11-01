from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  ekf_odom_params = os.path.join(
    get_package_share_directory('localization'),
    'config',
    'ekf_odom_node_params.yaml'
  )
  ekf_global_params = os.path.join(
    get_package_share_directory('localization'),
    'config',
    'ekf_global_node_params.yaml'
  )
  ekf_slip_params = os.path.join(
    get_package_share_directory('localization'),
    'config',
    'ekf_slip_node_params.yaml'
  )

  return LaunchDescription([
    Node(
      package='robot_localization',
      executable='ekf_node',
      name='ekf_odom_node',
      parameters=[ekf_odom_params],
      remappings=[
        ('odometry/filtered', 'odometry/filtered/ekf_odom_node')
      ]
    ),
    Node(
      package='robot_localization',
      executable='ekf_node',
      name='ekf_global_node',
      parameters=[ekf_global_params],
      remappings=[
        ('odometry/filtered', 'odometry/filtered/ekf_global_node')
      ]
    ),
    Node(
      package='robot_localization',
      executable='ekf_node',
      name='ekf_slip_node',
      parameters=[ekf_slip_params],
      remappings=[
        ('odometry/filtered', 'odometry/filtered/ekf_slip_node')
      ]
    ),
    Node(
            package='localization',
            executable='ts_prism_transformer',
            name='ts_prism_transformer',
            output='screen')
  ])
