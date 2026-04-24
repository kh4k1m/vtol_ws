import os
import time
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    workspace_dir = os.path.expanduser('~/vtol_ws')
    
    # 1. Читаем основной конфиг полета
    flight_config_path = os.path.join(workspace_dir, 'config', 'flight.yaml')
    with open(flight_config_path, 'r') as f:
        flight_config = yaml.safe_load(f)['flight_config']
        
    environment = flight_config.get('environment', 'real')
    navigation = flight_config.get('navigation', 'tags')
    
    # 2. Читаем конфиги камеры и маркеров
    camera_config_path = os.path.join(workspace_dir, 'config', 'camera.yaml')
    with open(camera_config_path, 'r') as f:
        camera_config_full = yaml.safe_load(f)
        
    # Поддержка профилей для разных разрешений
    if 'active_profile' in camera_config_full:
        active_profile = camera_config_full['active_profile']
        camera_config = camera_config_full['profiles'][active_profile]
        camera_offset = camera_config_full.get('camera_offset', {})
    else:
        camera_config = camera_config_full
        camera_offset = camera_config_full.get('camera_offset', {})
        
    markers_config_path = os.path.join(workspace_dir, 'config', 'markers.yaml')
    with open(markers_config_path, 'r') as f:
        markers_config = yaml.safe_load(f)
        
    # 3. Читаем конфиг миссии
    mission_config_path = os.path.join(workspace_dir, 'config', 'mission.yaml')
    with open(mission_config_path, 'r') as f:
        mission_config = yaml.safe_load(f)['mission']
        
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
        'height_log_dir': os.path.join(workspace_dir, 'logs'),
    }
    
    tag_localizer_dir = get_package_share_directory('tag_localizer')
    map_config = os.path.join(tag_localizer_dir, 'config', 'alti_transition_runway_pad_map.yaml')
    
    launch_entities = []
    connection_string = ""
    mavros_connection_string = ""
    
    # === ИСТОЧНИК ДАННЫХ (ENVIRONMENT) ===
    if environment == 'gazebo':
        gz_cam_topic = flight_config.get('gazebo_camera_topic')
        gz_info_topic = flight_config.get('gazebo_camera_info_topic')
        connection_string = flight_config.get('gazebo_connection')
        mavros_connection_string = flight_config.get('mavros_gazebo_connection')
        
        # Мост из Gazebo в ROS 2. 
        launch_entities.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gazebo_ros_bridge',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    f'{gz_cam_topic}@sensor_msgs/msg/Image[gz.msgs.Image',
                    f'{gz_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                ],
                remappings=[
                    (gz_cam_topic, '/camera/image_raw'),
                    (gz_info_topic, '/camera/camera_info'),
                ],
                output='screen',
            )
        )
    elif environment == 'real':
        connection_string = flight_config.get('real_connection')
        mavros_connection_string = flight_config.get('mavros_real_connection')
        launch_entities.append(
            Node(
                package='camera',
                executable='camera_node',
                name='camera_node',
                output='screen',
                parameters=[{
                    'video_source': flight_config.get('real_video_source'),
                    'image_width': camera_config.get('image_width', 1920),
                    'image_height': camera_config.get('image_height', 1080),
                    'fps': 30.0,
                    'camera_matrix_data': camera_config['camera_matrix']['data'],
                    'distortion_coefficients_data': camera_config['distortion_coefficients']['data']
                }]
            )
        )
    elif environment == 'from_log':
        connection_string = flight_config.get('log_connection')
        # В режиме from_log мы не запускаем драйвер камеры и мост Gazebo, 
        # так как картинки и телеметрия будут публиковаться из rosbag и video_player.py
        launch_entities.append(
            Node(
                package='bringup',
                executable='video_player',
                name='video_player_node',
                output='screen',
                parameters=[{
                    'video_dir': os.path.join(workspace_dir, 'logs'),
                    'fps': 30.0,
                }]
            )
        )

    # === АЛГОРИТМ НАВИГАЦИИ (NAVIGATION) ===
    if navigation == 'tags':
        # Полет по маркерам (AprilTag) + Взлет/Посадка
        launch_entities.extend([
            Node(
                package='tag_detector',
                executable='tag_detector_node',
                name='tag_detector_node',
                parameters=[detector_params],
                output='screen',
            ),
            Node(
                package='tag_localizer',
                executable='localizer_node',
                name='tag_localizer',
                parameters=[map_config, localizer_params],
                output='screen',
            ),
            Node(
                package='flight_manager',
                executable='mission_manager_node',
                name='mission_manager',
                output='screen',
                parameters=[{
                    'target_altitude_m': mission_config.get('target_altitude_m', 7.0),
                    'hover_duration_sec': mission_config.get('hover_duration_sec', 3.0),
                }],
            )
        ])
    elif navigation == 'dpvo':
        # Полет по DPVO (публикует позу в /vision_pose_enu и пишет CSV DPVO vs GPS)
        _dpvo_csv = os.path.join(
            workspace_dir, 'logs', 'dpvo_vs_gps_{}.csv'.format(
                time.strftime('%Y%m%d-%H%M%S')
            )
        )
        launch_entities.extend([
            Node(
                package='dpvo_bridge',
                executable='dpvo_node',
                name='dpvo_node',
                output='screen',
                parameters=[{
                    'image_topic': '/camera/image_raw',
                    'camera_info_topic': '/camera/camera_info',
                    'weights_path': os.path.expanduser('~/vtol_ws/dpvo.pth'),
                    # ВАЖНО: Мост MAVLink ждет позу в /vision_pose_enu
                    'pose_topic': '/vision_pose_enu',
                    'path_topic': '/dpvo/path',
                    'frame_id': 'map',
                    'undistort': True,
                    'publish_path': True,
                    'process_every_n': 1,
                    'enable_viz': False,
                }],
            ),
            # GPS с борта идёт из mavlink_bridge: /mavros/global_position/raw/fix (НЕ /gps/fix)
            Node(
                package='dpvo_bridge',
                executable='trajectory_logger_node',
                name='trajectory_logger_node',
                output='screen',
                parameters=[{
                    'gps_topic': '/mavros/global_position/raw/fix',
                    'pose_topic': '/vision_pose_enu',
                    'output_csv': _dpvo_csv,
                    'max_time_delta_sec': 0.5,
                }],
            )
        ])
    elif navigation == 'none':
        # Режим чистого сбора данных. Никакие алгоритмы навигации не запускаются.
        pass
        
    # === MAVLINK BRIDGE ===
    launch_entities.append(
        Node(
            package='ardupilot_mavlink_bridge',
            executable='mavlink_bridge_node',
            name='mavlink_bridge_node',
            parameters=[{
                'connection_string': connection_string,
                'baudrate': 115200,
                'vision_timestamp_mode': 'message_stamp',
            }],
            output='screen',
        )
    )

    # === MAVROS (Для чтения телеметрии IMU/GPS) ===
    # MAVROS удален, так как его нет в репозиториях Humble.
    # Теперь ardupilot_mavlink_bridge сам читает и публикует IMU и GPS.
    
    return LaunchDescription(launch_entities)
