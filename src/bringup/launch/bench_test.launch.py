from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    tag_localizer_dir = get_package_share_directory('tag_localizer')
    map_config = os.path.join(tag_localizer_dir, 'config', 'bench_test_map.yaml')

    return LaunchDescription([
        Node(
            package='camera',
            executable='camera_node',
            name='camera',
            parameters=[{'video_source': 'rtsp://192.168.1.110:8554/live'}]
        ),
        Node(
            package='tag_detector',
            executable='tag_detector_node',
            name='tag_detector'
        ),
        Node(
            package='tag_localizer',
            executable='localizer_node',
            name='tag_localizer',
            parameters=[map_config]
        ),
        Node(
            package='ardupilot_mavlink_bridge',
            executable='mavlink_bridge_node',
            name='mavlink_bridge'
        )
    ])
