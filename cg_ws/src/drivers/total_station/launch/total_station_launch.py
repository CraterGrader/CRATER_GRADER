from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():

    # Load rtls parameters
    total_station_params =  os.path.join(
        get_package_share_directory('total_station'),
        'config',
        'total_station.yaml'
    )
    total_station_beacon_static_tf_params_path = os.path.join(
        get_package_share_directory('total_station'),
        'config',
        'total_station_beacon_static_tf_params.yaml'
    )
    with open(total_station_beacon_static_tf_params_path, 'r') as stream:
        total_station_beacon_static_tf_params = yaml.load(
            stream,
            Loader=yaml.SafeLoader
        )['total_station_beacon_static_transform_publisher']['ros__parameters']

    return LaunchDescription([
        Node(
            package='total_station',
            executable='total_station_node',
            name='total_station_node',
            parameters=[total_station_params]
        ),
        # Static tf publisher
        # Order of args is [x, y, z, yaw, pitch, roll, parent_frame, child_frame]
        # http://docs.ros.org/en/foxy/Tutorials/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html#the-proper-way-to-publish-static-transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                total_station_beacon_static_tf_params['total_station_prism']['x'], total_station_beacon_static_tf_params['total_station_prism']['y'], total_station_beacon_static_tf_params['total_station_prism']['z'],
                total_station_beacon_static_tf_params['total_station_prism']['yaw'], total_station_beacon_static_tf_params['total_station_prism']['pitch'], total_station_beacon_static_tf_params['total_station_prism']['roll'],
                total_station_beacon_static_tf_params['total_station_prism']['parent_frame_id'], total_station_beacon_static_tf_params['total_station_prism']['child_frame_id']
            ]
        ),
    ])
