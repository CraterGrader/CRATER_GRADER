from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  bearing_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('bearing'), 'launch/bearing_launch.py')
    )
  )
  imu_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('imu'), 'launch/imu_launch.py')
    )
  )
  realsense_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('realsense'), 'launch/realsense_launch.py')
    )
  )
  uwb_beacon_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('uwb_beacon_rtls'), 'launch/uwb_beacon_rtls.launch.py')
    )
  )
  return LaunchDescription([
    bearing_launch,
    imu_launch,
    realsense_launch,
    uwb_beacon_launch
  ])
