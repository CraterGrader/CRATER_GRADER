from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    odom_params =  os.path.join(
        get_package_share_directory('motion_control'),
        'config',
        'odom_node_params.yaml'
    )
    cmdmux_params =  os.path.join(
        get_package_share_directory('motion_control'),
        'config',
        'cmdmux_node_params.yaml'
    )

    # Also bring up the teleop launch file, expect that teleop launch file to handle parameters
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'teleop'), 'launch/joystick_launch.py')
        )
    )

    return LaunchDescription([
        Node(
            package='motion_control',
            executable='odom_node',
            name='odom_node',
            parameters=[odom_params]
        ),
         Node(
            package='motion_control',
            executable='cmd_mux',
            name='cmd_mux',
            parameters=[cmdmux_params]
        ),
        included_launch
    ])
