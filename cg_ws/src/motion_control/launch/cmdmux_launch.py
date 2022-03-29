from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cmdmux_params =  os.path.join(
        get_package_share_directory('motion_control'),
        'config',
        'cmdmux_node_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='motion_control',
            executable='cmd_mux',
            name='cmd_mux',
            parameters=[cmdmux_params]
        )
    ])