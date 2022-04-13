from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    autograder_node_params =  os.path.join(
        get_package_share_directory('planning'),
        'config',
        'autograder_node_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='planning',
            executable='autograder_node',
            name='autograder_node',
            parameters=[autograder_node_params]
        )
    ])
