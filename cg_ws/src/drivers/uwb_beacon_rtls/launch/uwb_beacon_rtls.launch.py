from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():

    # Load rtls parameters
    uwb_beacon_rtls_params =  os.path.join(
        get_package_share_directory('uwb_beacon_rtls'),
        'config',
        'uwb_beacon_rtls.yaml'
    )
    beacon_static_tf_params_path = os.path.join(
        get_package_share_directory('uwb_beacon_rtls'),
        'config',
        'beacon_static_tf_params.yaml'
    )
    with open(beacon_static_tf_params_path, 'r') as stream:
        beacon_static_tf_params = yaml.load(
            stream,
            Loader=yaml.SafeLoader
        )['beacon_static_transform_publisher']['ros__parameters']

    return LaunchDescription([
        Node(
            package='uwb_beacon_rtls',
            executable='uwb_beacon_rtls_node',
            name='uwb_beacon_rtls_node',
            parameters=[uwb_beacon_rtls_params]
        ),
        # Static tf publisher
        # Order of args is [x, y, z, yaw, pitch, roll, parent_frame, child_frame]
        # http://docs.ros.org/en/foxy/Tutorials/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html#the-proper-way-to-publish-static-transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                beacon_static_tf_params['beacon_back']['x'], beacon_static_tf_params['beacon_back']['y'], beacon_static_tf_params['beacon_back']['z'],
                beacon_static_tf_params['beacon_back']['yaw'], beacon_static_tf_params['beacon_back']['pitch'], beacon_static_tf_params['beacon_back']['roll'],
                beacon_static_tf_params['beacon_back']['parent_frame_id'], beacon_static_tf_params['beacon_back']['child_frame_id']
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                beacon_static_tf_params['beacon_front']['x'], beacon_static_tf_params['beacon_front']['y'], beacon_static_tf_params['beacon_front']['z'],
                beacon_static_tf_params['beacon_front']['yaw'], beacon_static_tf_params['beacon_front']['pitch'], beacon_static_tf_params['beacon_front']['roll'],
                beacon_static_tf_params['beacon_front']['parent_frame_id'], beacon_static_tf_params['beacon_front']['child_frame_id']
            ]
        )
    ])
