from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Load site map params so planning subsystem knows map dimensions
    site_map_params = os.path.join(
        get_package_share_directory('mapping'),
        'config',
        'site_map_params.yaml'
    )

    behavior_executive_params = os.path.join(
        get_package_share_directory('planning'),
        'config',
        'behavior_executive_params.yaml'
    ) 

    return LaunchDescription([
        Node(
            package='planning',
            executable='behavior_executive_node',
            name='behavior_executive_node',
            output='screen',
            parameters=[site_map_params, behavior_executive_params]
        )
    ])
