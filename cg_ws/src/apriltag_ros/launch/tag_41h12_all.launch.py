import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# detect all Standard41h12 tags
cfg_Standard41h12 = {
    #"image_transport": "raw",
    #"family": "Standard41h12",
    #"size": 0.1,
    #"max_hamming": 0,
    #"z_up": True

    "image_transport": "raw",    # image format
    "family": "Standard41h12",   # tag family name
    "size": 0.1,             # tag edge size in meter
    "max_hamming": 0,          # maximum allowed hamming distance (corrected bits)
    "z_up": False,              # rotate about x-axis to have Z pointing upwards

    # see "apriltag.h" for more documentation on these optional parameters
    "decimate": 1.0,           # decimate resolution for quad detection
    "blur": 0.0,               # sigma of Gaussian blur for quad detection
    "refine-edges": 1,         # snap to strong gradients
    "threads": 1,              # number of threads
    "refine-decode": 0,        # increase the number of detected tags
    "refine-pose": 0,          # increase the accuracy of the extracted pose
    "debug": 1,                # write additional debugging images to current working directory

    "tag_ids": [0]            # tag IDs for which to publish transform
    #"tag_frames": ['test']   # optional frame names
    #tag_sizes": [0.1]      # optional tag-specific edge size
}

def generate_launch_description():
    composable_node = ComposableNode(
        name='apriltag',
        package='apriltag_ros', plugin='AprilTagNode',
        remappings=[("/apriltag/image", "/image_rect"), ("/apriltag/camera_info", "/camera_info")],
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
