from http.server import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    teleop_params =  os.path.join(
        get_package_share_directory('teleop'),
        'config',
        'teleop_node_params.yaml'
    )
    scale_offset_params =  os.path.join(
        get_package_share_directory('arduino'),
        'config',
        'scale_offset_params.yaml'
    )

    diagnostic_params = os.path.join(
        get_package_share_directory('arduino'),
        'config',
        'diagnostic_params.yaml'
    )

    with open(
        os.path.join(get_package_share_directory('arduino'),
        'config',
        'device_params.yaml'), "r") as stream:
        device_params = yaml.load(
        stream,
        Loader=yaml.SafeLoader
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
            name='serial_interface_node',
            parameters=[scale_offset_params, diagnostic_params]
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', device_params['arduino_node']['ros__parameters']['arduino_port'], '-v6']
        )
    ])
