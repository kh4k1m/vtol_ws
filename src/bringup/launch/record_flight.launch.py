import os
import time
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    video_path = f'/home/nvidia/vtol_ws/flight_video_{timestamp}.mp4'

    return LaunchDescription([
        # Запуск камеры
        Node(
            package='camera',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                # Укажите тут правильный источник видео, если нужно
                {'video_source': 'rtsp://192.168.1.110:8554/live'}
            ]
        ),
        
        # Запуск логгера телеметрии (только чтение, не мешает полету)
        Node(
            package='ardupilot_mavlink_bridge',
            executable='telemetry_logger_node',
            name='telemetry_logger_node',
            output='screen',
            parameters=[
                {'connection_string': '/dev/ttyACM0', 'baudrate': 115200},
                {'log_dir': '/home/nvidia/vtol_ws/'}
            ]
        ),

        # Запись видео в MP4 и сохранение таймстемпов кадров
        Node(
            package='camera',
            executable='video_recorder_node',
            name='video_recorder_node',
            output='screen',
            parameters=[
                {'output_path': video_path},
                {'fps': 30.0}
            ]
        )
    ])
