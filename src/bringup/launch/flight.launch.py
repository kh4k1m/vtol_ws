"""Top-level launch file for the visual navigation stack.

Reads ``$VTOL_WS_ROOT/config/flight.yaml`` (defaulting to ``~/vtol_ws``
if the env var is unset) and starts the right combination of camera
source, navigation algorithm, ArduPilot bridge, TF broadcaster and
logging entities for the requested scenario.

Scenarios are selected by three top-level fields in flight.yaml:

  environment: real | gazebo | from_log
  navigation:  tags | dpvo | orb_slam3 | hybrid | none
  mode:        log_only | fly_and_log | fly_only | log_subset

For ``hybrid`` navigation, ``vo_backend`` (``dpvo`` or ``orb_slam3``)
selects which VO source the fusion node consumes alongside the tag
localizer.

The ``mode`` selector controls two things:

* whether ``vision_pose_enu`` is forwarded into ArduPilot ODOMETRY
  (off in ``log_only``);
* whether ``mission_manager`` is started (off in ``log_only``);
* what is logged to disk (controlled by the ``logging`` block).

Logging entities are built by :func:`_build_logging_entities` and live
in a single timestamped session directory under ``logs/``.
"""

import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node


# Modes that should run mission_manager (i.e. attempt autonomous flight).
_MODES_WITH_MISSION_MANAGER = {'fly_and_log', 'fly_only', 'log_subset'}

# Modes that should forward /vision_pose_enu into MAVLink ODOMETRY.
_MODES_WITH_VISION_FORWARD = {'fly_and_log', 'fly_only', 'log_subset'}

# Modes that should write something to disk.
_MODES_WITH_LOGGING = {'log_only', 'fly_and_log', 'log_subset'}

_VALID_MODES = {'log_only', 'fly_and_log', 'fly_only', 'log_subset'}


def _workspace_dir() -> str:
    return os.environ.get('VTOL_WS_ROOT', os.path.expanduser('~/vtol_ws'))


def _load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def _resolve_orb_slam3_paths(flight_config: dict, workspace_dir: str):
    """Return (vocabulary_path, settings_path, sensor_type, viewer)."""
    orb_cfg = flight_config.get('orb_slam3', {}) or {}
    sensor_type = orb_cfg.get('sensor_type', 'MONOCULAR')
    enable_viewer = bool(orb_cfg.get('enable_viewer', False))

    orb_root_env = os.environ.get('ORB_SLAM3_ROOT', '')
    default_vocab = os.path.join(orb_root_env, 'Vocabulary', 'ORBvoc.txt') \
        if orb_root_env else ''
    vocab = orb_cfg.get('vocabulary_path', default_vocab) or default_vocab

    pkg_share = get_package_share_directory('orb_slam3_bridge')
    if sensor_type == 'IMU_MONOCULAR':
        default_settings = os.path.join(
            pkg_share, 'config', 'orb_slam3_imu_monocular_template.yaml')
    else:
        default_settings = os.path.join(
            pkg_share, 'config', 'orb_slam3_monocular_template.yaml')

    settings_rel = orb_cfg.get('settings_path', '')
    if settings_rel:
        if os.path.isabs(settings_rel):
            settings = settings_rel
        else:
            settings = os.path.join(workspace_dir, settings_rel)
    else:
        settings = default_settings

    return vocab, settings, sensor_type, enable_viewer


def _orb_slam3_node(*, pose_topic: str, nav_status_topic: str,
                    frame_id: str, vocab: str, settings: str,
                    sensor_type: str, viewer: bool) -> Node:
    return Node(
        package='orb_slam3_bridge',
        executable='orb_slam3_node',
        name='orb_slam3_node',
        output='screen',
        parameters=[
            {
                'vocabulary_path': vocab,
                'settings_path': settings,
                'sensor_type': sensor_type,
                'image_topic': '/camera/image_raw',
                'imu_topic': '/ap/imu/raw',
                'pose_topic': pose_topic,
                'nav_status_topic': nav_status_topic,
                'frame_id': frame_id,
                'enable_viewer': viewer,
            }
        ],
    )


def _dpvo_node(*, pose_topic: str, nav_status_topic: str,
               frame_id: str) -> Node:
    return Node(
        package='dpvo_bridge',
        executable='dpvo_node',
        name='dpvo_node',
        output='screen',
        parameters=[
            {
                'image_topic': '/camera/image_raw',
                'camera_info_topic': '/camera/camera_info',
                'weights_path': os.path.expanduser('~/vtol_ws/dpvo.pth'),
                'pose_topic': pose_topic,
                'nav_status_topic': nav_status_topic,
                'path_topic': '/dpvo/path',
                'frame_id': frame_id,
                'undistort': True,
                'publish_path': True,
                'process_every_n': 1,
                'enable_viz': False,
            }
        ],
    )


def _single_source_router(*, source_pose_topic: str,
                          source_nav_status_topic: str,
                          source_label: str) -> Node:
    """Forward a single source's pose / nav status to /vision_pose_enu + /nav/status.

    The downstream stack (mavlink_bridge, tf_broadcaster, mission_manager)
    is written against the unified topics. Single-source navigation modes
    publish to source-named topics for honest logging / replay; this
    router translates those into the unified contract. Hybrid mode does
    the same multiplexing inside vision_fusion_node and therefore does
    not start this router.
    """
    return Node(
        package='flight_manager',
        executable='single_source_router_node',
        name='single_source_router_node',
        output='screen',
        parameters=[
            {
                'source_pose_topic': source_pose_topic,
                'source_nav_status_topic': source_nav_status_topic,
                'out_pose_topic': '/vision_pose_enu',
                'out_nav_status_topic': '/nav/status',
                'source_label': source_label,
            }
        ],
    )


def _tf_broadcaster(cam_trans, cam_rot) -> Node:
    return Node(
        package='flight_manager',
        executable='tf_broadcaster_node',
        name='vision_tf_broadcaster',
        parameters=[
            {
                'cam_trans': cam_trans,
                'cam_rot': cam_rot,
            }
        ],
        output='screen',
    )


def _mission_manager(mission_config: dict) -> Node:
    return Node(
        package='flight_manager',
        executable='mission_manager_node',
        name='mission_manager',
        output='screen',
        parameters=[
            {
                'target_altitude_m': mission_config.get('target_altitude_m', 7.0),
                'hover_duration_sec': mission_config.get('hover_duration_sec', 3.0),
            }
        ],
    )


def _resolve_mode(flight_config: dict) -> str:
    mode = str(flight_config.get('mode', 'fly_and_log')).strip().lower()
    if mode not in _VALID_MODES:
        raise ValueError(
            f'flight_config.mode={mode!r} is not one of {sorted(_VALID_MODES)}'
        )
    return mode


def _resolve_session_dir(log_cfg: dict, workspace_dir: str) -> str:
    log_root = log_cfg.get('log_root', 'logs')
    if not os.path.isabs(log_root):
        log_root = os.path.join(workspace_dir, log_root)
    prefix = log_cfg.get('session_prefix', 'session')
    timestamp = time.strftime('%Y%m%d-%H%M%S')
    return os.path.join(log_root, f'{prefix}_{timestamp}')


def _expand_topic_groups(rosbag_cfg: dict, mode: str,
                         video_enabled: bool) -> list:
    """Resolve the active topic list for ros2 bag based on mode + profile.

    If ``video_enabled`` is true we drop ``/camera/image_raw`` from the bag
    even when the user explicitly added the ``image`` group: video frames
    already go to MP4 + sidecar timestamps CSV, duplicating them in the
    bag wastes ~180 MB/s of disk.
    """
    groups = rosbag_cfg.get('groups', {}) or {}
    profiles = rosbag_cfg.get('profiles', {}) or {}
    profile = str(rosbag_cfg.get('profile', 'auto')).lower()

    if profile == 'auto':
        active_groups = list(profiles.get(mode, []))
    elif profile == 'custom':
        active_groups = list(rosbag_cfg.get('subset_groups', []))
    elif profile in profiles:
        active_groups = list(profiles[profile])
    else:
        raise ValueError(
            f"logging.rosbag.profile={profile!r} is unknown. "
            f"Use 'auto', 'custom' or one of {list(profiles.keys())}."
        )

    if mode == 'log_subset' and profile == 'auto':
        active_groups = list(rosbag_cfg.get('subset_groups', []))

    seen = set()
    topics: list = []
    for group_name in active_groups:
        for topic in groups.get(group_name, []):
            if topic in seen:
                continue
            if video_enabled and topic == '/camera/image_raw':
                continue
            seen.add(topic)
            topics.append(topic)
    return topics


def _bag_record_process(rosbag_cfg: dict, topics: list, bag_dir: str):
    storage_id = str(rosbag_cfg.get('storage_id', 'mcap'))
    cmd = ['ros2', 'bag', 'record', '-s', storage_id, '-o', bag_dir]

    compression = str(rosbag_cfg.get('compression', '') or '').strip().lower()
    if compression and storage_id != 'mcap':
        cmd += ['--compression-format', compression, '--compression-mode', 'message']

    cmd += topics

    return ExecuteProcess(
        cmd=cmd,
        output='screen',
        shell=False,
    )


def _video_recorder(video_cfg: dict, session_dir: str) -> Node:
    video_dir = os.path.join(session_dir, 'video')
    os.makedirs(video_dir, exist_ok=True)
    output_path = os.path.join(video_dir, 'flight_video.mp4')
    return Node(
        package='camera',
        executable='video_recorder_node',
        name='video_recorder_node',
        output='screen',
        parameters=[
            {'output_path': output_path},
            {'fps': float(video_cfg.get('fps', 30.0))},
            {'chunk_duration_sec': float(video_cfg.get('chunk_duration_sec', 60.0))},
        ],
        remappings=[
            ('/camera/image_raw', video_cfg.get('topic', '/camera/image_raw')),
        ],
    )


def _telemetry_csv_logger(telemetry_cfg: dict, session_dir: str) -> Node:
    return Node(
        package='ardupilot_mavlink_bridge',
        executable='telemetry_logger_node',
        name='telemetry_logger_node',
        output='screen',
        parameters=[
            {
                'log_dir': session_dir,
                'log_rate_hz': float(telemetry_cfg.get('rate_hz', 10.0)),
            }
        ],
    )


def _write_session_manifest(session_dir, mode, navigation, environment,
                            topics, video_enabled, csv_enabled):
    manifest = {
        'created_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'mode': mode,
        'navigation': navigation,
        'environment': environment,
        'rosbag_topics': topics,
        'video_enabled': video_enabled,
        'telemetry_csv_enabled': csv_enabled,
    }
    manifest_path = os.path.join(session_dir, 'manifest.yaml')
    with open(manifest_path, 'w') as f:
        yaml.safe_dump(manifest, f, sort_keys=False, allow_unicode=True)


def _build_logging_entities(flight_config: dict, workspace_dir: str,
                            mode: str, navigation: str, environment: str):
    """Materialize bag/video/csv recorders for the active mode.

    Wrapped in an OpaqueFunction so the session directory and manifest
    are only created when launch is actually executed (not on
    ``ros2 launch --print`` or other dry-run inspections).
    """
    if mode not in _MODES_WITH_LOGGING:
        return []

    def _materialize(_context):
        log_cfg = flight_config.get('logging', {}) or {}
        session_dir = _resolve_session_dir(log_cfg, workspace_dir)
        os.makedirs(session_dir, exist_ok=True)

        entities = []

        # Resolve video first so the bag profile can drop /camera/image_raw.
        video_cfg = log_cfg.get('video', {}) or {}
        video_enabled = bool(video_cfg.get('enabled', False))

        rosbag_cfg = log_cfg.get('rosbag', {}) or {}
        topics = []
        if rosbag_cfg.get('enabled', True):
            topics = _expand_topic_groups(rosbag_cfg, mode, video_enabled)
            if topics:
                bag_dir = os.path.join(session_dir, 'bag')
                entities.append(_bag_record_process(rosbag_cfg, topics, bag_dir))

        if video_enabled:
            entities.append(_video_recorder(video_cfg, session_dir))

        telemetry_cfg = log_cfg.get('telemetry_csv', {}) or {}
        csv_enabled = bool(telemetry_cfg.get('enabled', False))
        if csv_enabled:
            entities.append(_telemetry_csv_logger(telemetry_cfg, session_dir))

        _write_session_manifest(
            session_dir, mode, navigation, environment, topics,
            video_enabled, csv_enabled,
        )

        return entities

    return [OpaqueFunction(function=_materialize)]


def generate_launch_description():
    workspace_dir = _workspace_dir()

    flight_config_path = os.path.join(workspace_dir, 'config', 'flight.yaml')
    flight_config = _load_yaml(flight_config_path).get('flight_config', {})

    environment = flight_config.get('environment', 'real')
    navigation = flight_config.get('navigation', 'tags')
    vo_backend = flight_config.get('vo_backend', 'dpvo')
    mode = _resolve_mode(flight_config)

    forward_vision = mode in _MODES_WITH_VISION_FORWARD
    run_mission_manager = mode in _MODES_WITH_MISSION_MANAGER

    camera_config_path = os.path.join(workspace_dir, 'config', 'camera.yaml')
    camera_config_full = _load_yaml(camera_config_path)
    if 'active_profile' in camera_config_full:
        active_profile = camera_config_full['active_profile']
        camera_config = camera_config_full['profiles'][active_profile]
    else:
        camera_config = camera_config_full

    markers_config = _load_yaml(os.path.join(workspace_dir, 'config', 'markers.yaml'))
    mission_config = _load_yaml(
        os.path.join(workspace_dir, 'config', 'mission.yaml')
    ).get('mission', {})

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
    cam_trans = camera_offset.get('translation', [0.0, 0.0, -0.18])
    cam_rot = camera_offset.get('rotation', [180.0, 0.0, -90.0])

    # tag_localizer / anchor / discovery tunables come from
    # config/tags_detector.yaml. The file is OPTIONAL: when absent or
    # empty, localizer_node falls back to in-code defaults. Layout:
    #
    #   tags_detector:
    #     anchor:     {strategy, anchor_marker_id, gps_topic, ...}
    #     discovery:  {learn_unknown_tags, persist_map_path, ...}
    #     localizer:  {pose_solver, sliding_window, ...}
    #
    # Each subsection's keys are flattened into the localizer_node's
    # ROS parameter list. The node owns the full whitelist of accepted
    # keys (declare_parameter calls in localizer_node.__init__).
    tags_detector_yaml = os.path.join(workspace_dir, 'config', 'tags_detector.yaml')
    tags_detector_cfg = (
        _load_yaml(tags_detector_yaml).get('tags_detector', {})
        if os.path.exists(tags_detector_yaml) else {}
    )
    anchor_cfg = tags_detector_cfg.get('anchor', {}) or {}
    discovery_cfg = tags_detector_cfg.get('discovery', {}) or {}
    localizer_cfg = tags_detector_cfg.get('localizer', {}) or {}

    localizer_params = {
        'cam_trans': cam_trans,
        'cam_rot': cam_rot,
        'height_log_dir': os.path.join(workspace_dir, 'logs'),
        'anchor_log_dir': os.path.join(workspace_dir, 'logs', 'maps'),
    }
    _LOCALIZER_TUNING_KEYS = (
        'pose_solver', 'absolute_max_per_tag_spread_m',
        'cost_threshold_per_tag_m', 'detection_gap_reset_sec',
        'stable_window_size', 'stable_xy_stddev_m', 'stable_z_stddev_m',
        'sliding_window', 'kalman_R_pos_m', 'kalman_Q_pos',
        'kalman_Q_vel', 'kalman_Q_accel', 'z_score_threshold',
    )
    _ANCHOR_KEYS = (
        'anchor_strategy', 'anchor_marker_id', 'gps_topic',
        'attitude_topic', 'gps_wait_timeout_sec',
    )
    # The discovery keys we forward to localizer_node. Note: the YAML
    # uses the natural name "strategy" inside `anchor:`, but the node
    # parameter is `anchor_strategy` - we remap that one explicitly
    # below.
    _DISCOVERY_KEYS = ('learn_unknown_tags', 'persist_map_path')
    for k in _LOCALIZER_TUNING_KEYS:
        if k in localizer_cfg:
            localizer_params[k] = localizer_cfg[k]
    if 'strategy' in anchor_cfg:
        localizer_params['anchor_strategy'] = anchor_cfg['strategy']
    for k in _ANCHOR_KEYS:
        if k in anchor_cfg:
            localizer_params[k] = anchor_cfg[k]
    for k in _DISCOVERY_KEYS:
        if k in discovery_cfg:
            localizer_params[k] = discovery_cfg[k]

    launch_entities = []
    connection_string = ''

    # === DATA SOURCE ===
    if environment == 'gazebo':
        gz_cam_topic = flight_config.get('gazebo_camera_topic')
        gz_info_topic = flight_config.get('gazebo_camera_info_topic')
        connection_string = flight_config.get('gazebo_connection')
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
        launch_entities.append(
            Node(
                package='camera',
                executable='camera_node',
                name='camera_node',
                output='screen',
                parameters=[
                    {
                        'video_source': flight_config.get('real_video_source'),
                        'image_width': camera_config.get('image_width', 1920),
                        'image_height': camera_config.get('image_height', 1080),
                        'fps': 30.0,
                        'camera_matrix_data': camera_config['camera_matrix']['data'],
                        'distortion_coefficients_data': camera_config['distortion_coefficients']['data'],
                    }
                ],
            )
        )
    elif environment == 'from_log':
        connection_string = flight_config.get('log_connection')
        launch_entities.append(
            Node(
                package='bringup',
                executable='video_player',
                name='video_player_node',
                output='screen',
                parameters=[
                    {
                        'video_dir': os.path.join(workspace_dir, 'logs'),
                        'fps': 30.0,
                    }
                ],
            )
        )

    # === NAVIGATION ALGORITHM ===
    #
    # Topology contract (same in single-source and hybrid):
    #   * Each navigation source ALWAYS publishes to its own native topic
    #     (/tag_localizer/pose, /dpvo/pose, /orb_slam3/pose) and to its
    #     own /<src>/nav_status. This keeps source-named channels honest
    #     in bag/viz/replay regardless of which mode is active.
    #   * The unified /vision_pose_enu + /nav/status (consumed by
    #     mavlink_bridge, tf_broadcaster, mission_manager) is produced
    #     by:
    #       - single_source_router_node in single-source modes (1:1 pass-through)
    #       - vision_fusion_node          in hybrid           (multiplexer)
    #   * Whether /vision_pose_enu actually reaches ArduPilot ODOMETRY is
    #     decided by mode (vision_forward_enabled in mavlink_bridge).
    if navigation == 'tags':
        nav_entities = [
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
                parameters=[localizer_params],
                output='screen',
            ),
            _single_source_router(
                source_pose_topic='/tag_localizer/pose',
                source_nav_status_topic='/tag_localizer/nav_status',
                source_label='tags',
            ),
            _tf_broadcaster(cam_trans, cam_rot),
        ]
        if run_mission_manager:
            nav_entities.append(_mission_manager(mission_config))
        launch_entities.extend(nav_entities)
    elif navigation == 'dpvo':
        dpvo_csv = os.path.join(
            workspace_dir,
            'logs',
            f'dpvo_vs_gps_{time.strftime("%Y%m%d-%H%M%S")}.csv',
        )
        launch_entities.extend(
            [
                _dpvo_node(
                    pose_topic='/dpvo/pose',
                    nav_status_topic='/dpvo/nav_status',
                    frame_id='map',
                ),
                Node(
                    package='dpvo_bridge',
                    executable='trajectory_logger_node',
                    name='trajectory_logger_node',
                    output='screen',
                    parameters=[
                        {
                            'gps_topic': '/ap/gps/fix',
                            'pose_topic': '/dpvo/pose',
                            'output_csv': dpvo_csv,
                            'max_time_delta_sec': 0.5,
                        }
                    ],
                ),
                _single_source_router(
                    source_pose_topic='/dpvo/pose',
                    source_nav_status_topic='/dpvo/nav_status',
                    source_label='dpvo',
                ),
                _tf_broadcaster(cam_trans, cam_rot),
            ]
        )
    elif navigation == 'orb_slam3':
        vocab, settings, sensor_type, viewer = _resolve_orb_slam3_paths(
            flight_config, workspace_dir
        )
        launch_entities.extend(
            [
                _orb_slam3_node(
                    pose_topic='/orb_slam3/pose',
                    nav_status_topic='/orb_slam3/nav_status',
                    frame_id='map',
                    vocab=vocab,
                    settings=settings,
                    sensor_type=sensor_type,
                    viewer=viewer,
                ),
                _single_source_router(
                    source_pose_topic='/orb_slam3/pose',
                    source_nav_status_topic='/orb_slam3/nav_status',
                    source_label='orb_slam3',
                ),
                _tf_broadcaster(cam_trans, cam_rot),
            ]
        )
    elif navigation == 'hybrid':
        backend = vo_backend.lower()
        if backend == 'orb_slam3':
            vocab, settings, sensor_type, viewer = _resolve_orb_slam3_paths(
                flight_config, workspace_dir
            )
            vo_node = _orb_slam3_node(
                pose_topic='/orb_slam3/pose',
                nav_status_topic='/orb_slam3/nav_status',
                frame_id='orb_slam3_map',
                vocab=vocab,
                settings=settings,
                sensor_type=sensor_type,
                viewer=viewer,
            )
            vo_pose_topic = '/orb_slam3/pose'
            vo_nav_status_topic = '/orb_slam3/nav_status'
            vo_source_label = 'orb_slam3'
        else:
            vo_node = _dpvo_node(
                pose_topic='/dpvo/pose',
                nav_status_topic='/dpvo/nav_status',
                frame_id='dpvo_map',
            )
            vo_pose_topic = '/dpvo/pose'
            vo_nav_status_topic = '/dpvo/nav_status'
            vo_source_label = 'dpvo'

        hybrid_entities = [
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
                parameters=[localizer_params],
                output='screen',
            ),
            vo_node,
            Node(
                package='flight_manager',
                executable='vision_fusion_node',
                name='vision_fusion_node',
                output='screen',
                parameters=[
                    {
                        'fusion_mode': 'auto',
                        'vo_pose_topic': vo_pose_topic,
                        'vo_nav_status_topic': vo_nav_status_topic,
                        'vo_source_label': vo_source_label,
                    }
                ],
            ),
            # NB: discovery of new markers is now done inside
            # tag_localizer (learn_unknown_tags=True). The dedicated
            # landmark_mapper_node has been removed.
            _tf_broadcaster(cam_trans, cam_rot),
        ]
        if run_mission_manager:
            hybrid_entities.append(_mission_manager(mission_config))
        launch_entities.extend(hybrid_entities)
    elif navigation == 'none':
        # Pure data-collection / manual-test mode. The bridge still runs so
        # bench tests of GPS waypoints / GPS takeoff / etc. work.
        pass

    # === MAVLINK BRIDGE ===
    launch_entities.append(
        Node(
            package='ardupilot_mavlink_bridge',
            executable='mavlink_bridge_node',
            name='mavlink_bridge_node',
            parameters=[
                {
                    'connection_string': connection_string,
                    'baudrate': 115200,
                    'vision_forward_enabled': forward_vision,
                }
            ],
            output='screen',
        )
    )

    # === LOGGING ===
    launch_entities.extend(
        _build_logging_entities(
            flight_config, workspace_dir, mode, navigation, environment
        )
    )

    return LaunchDescription(launch_entities)
