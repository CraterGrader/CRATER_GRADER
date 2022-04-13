from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    slip_estimate_params =  os.path.join(
        get_package_share_directory('localization'),
        'config',
        'slip_estimate_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='localization',
            executable='slip_estimate_node',
            name='slip_estimate_node',
            parameters=[slip_estimate_params]
        )
    ])
