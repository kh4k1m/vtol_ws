import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                {'video_source': 'rtsp://192.168.1.110:8554/live'}
            ]
        ),
        Node(
            package='tag_detector',
            executable='tag_detector_node',
            name='tag_detector_node',
            output='screen'
        ),
        Node(
            package='pose_bridge',
            executable='pose_bridge_node',
            name='pose_bridge_node',
            output='screen'
        ),
        Node(
            package='ardupilot_mavlink_bridge',
            executable='mavlink_bridge_node',
            name='mavlink_bridge_node',
            output='screen',
            parameters=[
                # Для реального полета раскомментируйте строку ниже:
                # {'connection_string': '/dev/ttyACM0', 'baudrate': 115200}
                
                # Для симуляции (SITL на вашем ПК) раскомментируйте строку ниже и укажите IP вашего ПК (с которого вы подключаетесь по SSH):
                # {'connection_string': 'udp:192.168.0.56:14551', 'baudrate': 115200}
                {'connection_string': 'tcp:127.0.0.1:5762', 'baudrate': 115200}
            ]
        ),
    ])
