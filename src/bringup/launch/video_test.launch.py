import os
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    workspace_dir = os.path.expanduser('~/vtol_ws')
    
    # Read camera config
    camera_config_path = os.path.join(workspace_dir, 'config', 'camera.yaml')
    with open(camera_config_path, 'r') as f:
        camera_config = yaml.safe_load(f)
        
    # Read markers config
    markers_config_path = os.path.join(workspace_dir, 'config', 'markers.yaml')
    with open(markers_config_path, 'r') as f:
        markers_config = yaml.safe_load(f)
        
    # Extract marker IDs and sizes for the detector from the ROS config.
    marker_ids = []
    marker_sizes = []
    for marker in markers_config.get('markers', []):
        marker_ids.append(int(marker['id']))
        marker_sizes.append(float(marker['size']))
        
    # Parameters for detector
    detector_params = {
        'camera_matrix_data': camera_config['camera_matrix']['data'],
        'distortion_coefficients_data': camera_config['distortion_coefficients']['data'],
        'default_marker_size': markers_config.get('default_marker_size', 0.2),
        'marker_ids': marker_ids,
        'marker_sizes': marker_sizes
    }
    
    # Parameters for localizer
    camera_offset = camera_config.get('camera_offset', {})
    localizer_params = {
        'cam_trans': camera_offset.get('translation', [0.0, 0.0, -0.18]),
        'cam_rot': camera_offset.get('rotation', [180.0, 0.0, -90.0])
    }
    
    return LaunchDescription([
        # 1. Video Player Node
        Node(
            package='bringup',
            executable='video_player',
            name='video_player',
            parameters=[{
                'video_path': os.path.join(workspace_dir, 'data', 'C0026', 'C0026.MP4'),
                'fps': 30.0
            }],
            output='screen'
        ),
        
        # 2. Tag Detector Node (C++)
        Node(
            package='tag_detector',
            executable='tag_detector_node',
            name='tag_detector_node',
            parameters=[detector_params],
            output='screen'
        ),
        
        # 3. Tag Localizer Node (Python)
        Node(
            package='tag_localizer',
            executable='localizer_node',
            name='tag_localizer',
            parameters=[localizer_params],
            output='screen'
        )
    ])
