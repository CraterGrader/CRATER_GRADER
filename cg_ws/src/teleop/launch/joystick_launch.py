from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    teleop_params =  os.path.join(
        get_package_share_directory('teleop'),
        'config',
        'teleop_node_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            package='teleop',
            executable='teleop_node',
            name='teleop_node',
            parameters=[teleop_params]
        ),
        Node(
            package='arduino',
            executable='serial_interface_node',
            name='serial_interface_node'
        )
    ])
