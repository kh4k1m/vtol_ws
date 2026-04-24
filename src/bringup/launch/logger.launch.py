import os
import time
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    workspace_dir = os.path.expanduser('~/vtol_ws')
    config_path = os.path.join(workspace_dir, 'config', 'logger.yaml')
    
    # Читаем конфиг
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)['logger']
        
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    bag_path = os.path.join(workspace_dir, 'logs', f'sim_bag_{timestamp}')
    video_path = os.path.join(workspace_dir, 'logs', f'flight_video_{timestamp}.mp4')
    
    launch_entities = []
    
    # 1. Запись Rosbag (телеметрия)
    if config.get('rosbag', {}).get('record', True):
        rosbag_cfg = config['rosbag']
        topics = " ".join(rosbag_cfg.get('topics', []))
        
        cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', bag_path]
        
        # Добавляем сжатие, если указано
        comp_format = rosbag_cfg.get('compression_format', '')
        comp_mode = rosbag_cfg.get('compression_mode', '')
        if comp_format and comp_mode:
            cmd.extend(['--compression-format', comp_format, '--compression-mode', comp_mode])
            
        cmd.append(topics)
        
        launch_entities.append(
            ExecuteProcess(
                cmd=[" ".join(cmd)],
                output='screen',
                shell=True
            )
        )
        
    # 2. Запись видео (сжатие картинок в MP4)
    if config.get('video', {}).get('record', True):
        video_cfg = config['video']
        
        launch_entities.append(
            Node(
                package='camera',
                executable='video_recorder_node',
                name='video_recorder_node',
                output='screen',
                parameters=[
                    {'output_path': video_path},
                    {'fps': float(video_cfg.get('fps', 30.0))},
                    {'chunk_duration_sec': float(video_cfg.get('chunk_duration_sec', 60.0))}
                ],
                remappings=[
                    ('/camera/image_raw', video_cfg.get('topic', '/camera/image_raw'))
                ]
            )
        )
        
    return LaunchDescription(launch_entities)
