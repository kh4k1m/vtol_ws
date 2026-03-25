import os
import time
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    bag_path = f'/home/nvidia/vtol_ws/flight_video_bag_{timestamp}'

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

        # Запись видео в rosbag
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_path, '/camera/image_raw'],
            output='screen'
        )
    ])
