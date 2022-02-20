from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
  imu_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('imu'), 'launch/imu_launch.py')
    )
  )

  return LaunchDescription([
    imu_launch,
    Node(
      package='imu',
      executable='imu_viz_node',
      name='imu_viz_node'
    ),
    Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2'
    )
  ])
