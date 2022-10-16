from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    terrain_filtering_params =  os.path.join(
        get_package_share_directory('mapping'),
        'config',
        'terrain_filtering_params.yaml'
    )
    # point_cloud_registration_params =  os.path.join(
    #     get_package_share_directory('mapping'),
    #     'config',
    #     'point_cloud_registration_params.yaml'
    # )
    site_map_params = os.path.join(
        get_package_share_directory('mapping'),
        'config',
        'site_map_params.yaml'
    ) 


    return LaunchDescription([
        Node(
            package='mapping',
            executable='terrain_filtering',
            name='terrain_filtering',
            output='screen',
            parameters=[terrain_filtering_params]
        ),
        Node(
            package='mapping',
            executable='site_map',
            name='site_map',
            output='screen',
            parameters=[site_map_params]
        )
    ])
