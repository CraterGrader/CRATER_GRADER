import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# detect all Standard41h12 tags
cfg_Standard41h12 = {
    "image_transport": "raw",
    "family": "Standard41h12",
    "size": 0.1,
    "max_hamming": 0,
    "z_up": True
}

def generate_launch_description():
    composable_node = ComposableNode(
        name='apriltag',
        package='apriltag_ros', plugin='AprilTagNode',
        remappings=[("/apriltag/image", "/image_raw"), ("/apriltag/camera_info", "/camera_info")],
        parameters=[cfg_Standard41h12])
    container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return launch.LaunchDescription([container])
