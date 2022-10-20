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

    diagnostic_params = os.path.join(
        get_package_share_directory('motion_control'),
        'config',
        'diagnostic_params.yaml'
    )

    worksystem_params = os.path.join(
        get_package_share_directory('motion_control'),
        'config',
        'worksystem_control_params.yaml'
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
            parameters=[diagnostic_params]
        ),
        Node(
            package='motion_control',
            executable='worksystem_control_node',
            name='worksystem_control_node',
            output='screen',
            parameters=[worksystem_params]
        ),
        included_launch
    ])
