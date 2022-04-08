from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    point_cloud_registration_params =  os.path.join(
        get_package_share_directory('mapping'),
        'config',
        'point_cloud_registration_params.yaml'
    )


    return LaunchDescription([
        Node(
            package='mapping',
            executable='terrain_filtering',
            name='terrain_filtering',
            output='screen'
        ),
        Node(
            package='mapping',
            executable='point_cloud_registration',
            name='point_cloud_registration',
            output='screen',
            parameters=[point_cloud_registration_params]
        )
    ])
