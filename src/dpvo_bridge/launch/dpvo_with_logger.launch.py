from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('gps_topic', default_value='/gps/fix'),
        DeclareLaunchArgument('weights_path', default_value=''),
        DeclareLaunchArgument('config_path', default_value=''),
        DeclareLaunchArgument('output_csv', default_value='/tmp/dpvo_vs_gps.csv'),
        Node(
            package='dpvo_bridge',
            executable='dpvo_node',
            name='dpvo_node',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'weights_path': LaunchConfiguration('weights_path'),
                'config_path': LaunchConfiguration('config_path'),
                'pose_topic': '/dpvo/pose',
                'path_topic': '/dpvo/path',
                'frame_id': 'dpvo_map',
                'undistort': True,
                'publish_path': True,
                'process_every_n': 1,
                'enable_viz': False,
            }],
        ),
        Node(
            package='dpvo_bridge',
            executable='trajectory_logger_node',
            name='trajectory_logger_node',
            output='screen',
            parameters=[{
                'gps_topic': LaunchConfiguration('gps_topic'),
                'pose_topic': '/dpvo/pose',
                'output_csv': LaunchConfiguration('output_csv'),
                'max_time_delta_sec': 0.25,
            }],
        ),
    ])
