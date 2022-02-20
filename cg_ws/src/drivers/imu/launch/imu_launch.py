from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    imu_params =  os.path.join(
        get_package_share_directory('imu'),
        'config',
        'imu_node_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='imu',
            executable='imu_node',
            name='imu_node',
            parameters=[imu_params]
        )
    ])
