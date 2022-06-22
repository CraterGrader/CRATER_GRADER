from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    autograder_node_params =  os.path.join(
        get_package_share_directory('planning'),
        'config',
        'autograder_node_params.yaml'
    )

    # Also bring up the teleop launch file, expect that teleop launch file to handle parameters
    included_launch_slip_estimate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'localization'), 'launch/slip_estimate_launch.py')
        )
    )

    return LaunchDescription([
        Node(
            package='planning',
            executable='autograder_node',
            name='autograder_node',
            parameters=[autograder_node_params]
        ),
        # included_launch_slip_estimate
    ])
