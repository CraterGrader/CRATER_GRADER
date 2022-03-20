from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ekf_odom_params =  os.path.join(
        get_package_share_directory('localization'),
        'config',
        'ekf_odom_node_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_odom_node',
            parameters=[ekf_odom_params]
        )
    ])
