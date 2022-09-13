from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory

import os
import yaml

# Detect Standard41h12 tags of index 0-3
cfg_Standard41h12 = {
    "image_transport": "raw",    # image format
    "family": "Standard41h12",   # tag family name
    "size": 0.282,             # tag edge size in meter
    "max_hamming": 0,          # maximum allowed hamming distance (corrected bits)
    "z_up": True,              # rotate about x-axis to have Z pointing upwards

    # see "apriltag.h" for more documentation on these optional parameters
    "decimate": 1.0,           # decimate resolution for quad detection
    "blur": 0.0,               # sigma of Gaussian blur for quad detection
    "refine-edges": 1,         # snap to strong gradients
    "threads": 1,              # number of threads
    "refine-decode": 0,        # increase the number of detected tags
    "refine-pose": 1,          # increase the accuracy of the extracted pose
    "debug": 0,                # write additional debugging images to current working directory

    "tag_ids": [0,1,2,3],           # tag IDs for which to publish transform
    "tag_frames": ['april_tag0', 'april_tag1', 'april_tag2', 'april_tag3']   # optional frame names
    #"tag_sizes": [0.1]      # optional tag-specific edge size
}

def generate_launch_description():
    # Tag positions, camera-base link transform, and visualization
    bearing_params_path = os.path.join(
        get_package_share_directory('bearing'),
        'config',
        'bearing_node_params.yaml'
    )

    # Get bearing parameters to determine if rviz2 needs to be launched
    with open(bearing_params_path, "r") as stream:
        bearing_params = yaml.load(
        stream,
        Loader = yaml.SafeLoader
    )  

    # Camera input parameters
    usb_cam_path = os.path.join(
        get_package_share_directory('usb_cam'),
        'config',
        'params.yaml'
    )

    # Bearing and usb_cam nodes
    nodes = [
        Node(
            package='bearing',
            executable='bearing_node',
            name='bearing_node',
            parameters=[bearing_params_path]
        ),
        # Base link to camera transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = [bearing_params['bearing_node']['ros__parameters']['base_cam_tf']['position']['x'],
                        bearing_params['bearing_node']['ros__parameters']['base_cam_tf']['position']['y'],
                        bearing_params['bearing_node']['ros__parameters']['base_cam_tf']['position']['z'],
                        bearing_params['bearing_node']['ros__parameters']['base_cam_tf']['orientation']['z'],
                        bearing_params['bearing_node']['ros__parameters']['base_cam_tf']['orientation']['y'],
                        bearing_params['bearing_node']['ros__parameters']['base_cam_tf']['orientation']['x'],
                        'rob_base_link', 
                        'camera']
        ),
        Node(
            package='usb_cam', 
            executable='usb_cam_node_exe', 
            output='screen',
            name='usb_cam',
            parameters=[usb_cam_path]
        )
    ]

    # Transforms for all tags declared in parameters above - 'map' to 'tagX'
    for i in range(len(bearing_params['bearing_node']['ros__parameters']['tags']['position']['x'])):
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[bearing_params['bearing_node']['ros__parameters']['tags']['position']['x'][i],
                    bearing_params['bearing_node']['ros__parameters']['tags']['position']['y'][i],
                    bearing_params['bearing_node']['ros__parameters']['tags']['position']['z'][i],
                    bearing_params['bearing_node']['ros__parameters']['tags']['orientation']['z'][i],
                    bearing_params['bearing_node']['ros__parameters']['tags']['orientation']['y'][i],
                    bearing_params['bearing_node']['ros__parameters']['tags']['orientation']['x'][i],
                    'tag_map',
                    'tag' + str(i)]
        ))
    print(bearing_params['bearing_node']['ros__parameters']['base_cam_tf'])

    # AprilTag node with remapping - adjust here if changes to rectified image name
    composable_node = ComposableNode(
        name='apriltag',
        package='apriltag_ros',
        plugin='AprilTagNode',
        remappings=[("/apriltag/camera_info", "/camera_info"), ("/apriltag/image", "/image_raw")],
        parameters=[cfg_Standard41h12]
    )

    # Corresponding AprilTag Container
    nodes.append(ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen'
    ))

    return LaunchDescription(nodes)
