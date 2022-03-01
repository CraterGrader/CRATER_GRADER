from http.server import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    uwb_beacon_rtls_params =  os.path.join(
        get_package_share_directory('uwb_rtls'),
        'config',
        'uwb_beacon_rtls.yaml'
    )

    return LaunchDescription([
        Node(
            package='uwb_rtls',
            executable='uwb',
            name='uwb',
            parameters=[uwb_beacon_rtls_params]
        )
    ])
