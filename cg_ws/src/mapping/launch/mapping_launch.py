from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='mapping',
            executable='terrain_filtering',
            name='terrain_filtering',
            output='screen'
        )
    ])
