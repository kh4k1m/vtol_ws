"""Replay сессии (или одиночного MP4) для прогона CV-pipeline.

Usage:
    # Старый режим: одиночный MP4 + детектор/локализатор
    ros2 launch bringup video_test.launch.py

    # Replay записанной сессии (синхронно с bag)
    ros2 launch bringup video_test.launch.py \
        session_dir:=/home/dragon/vtol_ws/logs/session_20260505-203000 \
        use_sim_time:=true
    # параллельно в другом терминале:
    #   ros2 bag play <session>/bag --clock 100

Параметры:
    session_dir   - путь до session_*/ (если задан, играет
                    <session>/video/*.mp4 + sidecar timestamps).
    video_path    - одиночный MP4 (используется если session_dir пуст).
    use_sim_time  - true для синхронного replay'я с ros2 bag play --clock.
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def generate_launch_description():
    workspace_dir = os.environ.get('VTOL_WS_ROOT', os.path.expanduser('~/vtol_ws'))

    camera_config_full = _load_yaml(os.path.join(workspace_dir, 'config', 'camera.yaml'))
    if 'active_profile' in camera_config_full:
        active_profile = camera_config_full['active_profile']
        camera_config = camera_config_full['profiles'][active_profile]
    else:
        camera_config = camera_config_full

    markers_config = _load_yaml(os.path.join(workspace_dir, 'config', 'markers.yaml'))

    marker_ids = [int(m['id']) for m in markers_config.get('markers', [])]
    marker_sizes = [float(m['size']) for m in markers_config.get('markers', [])]

    detector_params = {
        'camera_matrix_data': camera_config['camera_matrix']['data'],
        'distortion_coefficients_data': camera_config['distortion_coefficients']['data'],
        'default_marker_size': markers_config.get('default_marker_size', 0.2),
        'marker_ids': marker_ids,
        'marker_sizes': marker_sizes,
    }

    camera_offset = camera_config.get('camera_offset', {})
    localizer_params = {
        'cam_trans': camera_offset.get('translation', [0.0, 0.0, -0.18]),
        'cam_rot': camera_offset.get('rotation', [180.0, 0.0, -90.0]),
    }

    session_arg = DeclareLaunchArgument('session_dir', default_value='')
    video_arg = DeclareLaunchArgument(
        'video_path',
        default_value=os.path.join(workspace_dir, 'data', 'C0026', 'C0026.MP4'),
    )
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    session_dir = LaunchConfiguration('session_dir')
    video_path = LaunchConfiguration('video_path')
    use_sim_time = LaunchConfiguration('use_sim_time')

    has_session = PythonExpression(["'", session_dir, "' != ''"])
    no_session = PythonExpression(["'", session_dir, "' == ''"])

    session_player = Node(
        package='bringup',
        executable='video_player',
        name='video_player',
        condition=IfCondition(has_session),
        parameters=[{
            'session_dir': session_dir,
            'mode': 'auto',
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    legacy_player = Node(
        package='bringup',
        executable='video_player',
        name='video_player',
        condition=IfCondition(no_session),
        parameters=[{
            'video_path': video_path,
            'mode': 'loop',
            'fps': 30.0,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    return LaunchDescription([
        session_arg,
        video_arg,
        sim_time_arg,
        session_player,
        legacy_player,
        Node(
            package='tag_detector',
            executable='tag_detector_node',
            name='tag_detector_node',
            parameters=[detector_params, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='tag_localizer',
            executable='localizer_node',
            name='tag_localizer',
            parameters=[localizer_params, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
