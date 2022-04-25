from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
  urdf = os.path.join(
    get_package_share_directory('cg_visualization'),
    'urdf', '600220.urdf'
  )

  marker_viz_params =  os.path.join(
        get_package_share_directory('cg_visualization'),
        'config',
        'marker_viz_params.yaml'
  )
  with open(urdf, 'r') as f:
    robot_desc = f.read()
    
  return LaunchDescription([
    # Node(
    #   package='robot_state_publisher',
    #   executable='robot_state_publisher',
    #   name='robot_state_publisher',
    #   output='screen',
    #   parameters=[{'robot_description': robot_desc}],
    #   arguments=[urdf]
    # ),
    Node(
      package='rviz2',
      namespace='',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', [os.path.join(get_package_share_directory('cg_visualization'), 'config', 'crater_loc_test.rviz')]]
    ),
    # Node(
    #   package='rviz2',
    #   namespace='',
    #   executable='rviz2',
    #   name='rviz2',
    #   arguments=['-d', [os.path.join(get_package_share_directory('cg_visualization'), 'config', 'svd_suncam.rviz')]]
    # ),
    # Node(
    #   package='rviz2',
    #   namespace='',
    #   executable='rviz2',
    #   name='rviz2',
    #   arguments=['-d', [os.path.join(get_package_share_directory('cg_visualization'), 'config', 'svd_depthcam.rviz')]]
    # ),
     Node(
      package='cg_visualization',
      executable='marker_viz_node',
      name='marker_viz_node',
      output='screen'
    )
    # Plotjuggler cannot be placed in a launch file
    # https://githubhot.com/repo/facontidavide/PlotJuggler/issues/561
    # Node(
    #   package='plotjuggler',
    #   executable='plotjuggler',
    #   name='plotjuggler'
    # )
  ])