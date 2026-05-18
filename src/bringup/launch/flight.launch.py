"""Top-level launch file for the visual navigation stack.

Reads ``$VTOL_WS_ROOT/config/flight.yaml`` (defaulting to ``~/vtol_ws``
if the env var is unset) and starts the right combination of camera
source, navigation algorithm, ArduPilot bridge, TF broadcaster and
logging entities for the requested scenario.

Scenarios are selected by three top-level fields in flight.yaml:

  environment: real | gazebo | from_log
  navigation:  tags | dpvo | orb_slam3 | hybrid | none
  mode:        log_only | fly_and_log | fly_and_viz | fly_only | log_subset

For ``hybrid`` navigation, ``vo_backend`` (``dpvo`` or ``orb_slam3``)
selects which VO source the fusion node consumes alongside the tag
localizer.

The ``mode`` selector controls two things:

* whether ``vision_pose_enu`` is forwarded into ArduPilot ODOMETRY
  (off in ``log_only``);
* whether ``mission_manager`` is started (off in ``log_only``);
* what is logged to disk (controlled by the ``logging`` block);
* whether ``flight_viz`` renders trajectory + altitude + ATE plots
  for every nav source after the launch shuts down
  (``fly_and_viz`` only).

Logging entities are built by :func:`_build_logging_entities` and live
in a single timestamped session directory under ``logs/``.
"""

import os
import sys
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node


# Modes that should run mission_manager (i.e. attempt autonomous flight).
_MODES_WITH_MISSION_MANAGER = {'fly_and_log', 'fly_and_viz', 'fly_only', 'log_subset'}

# Modes that should forward /vision_pose_enu into MAVLink ODOMETRY.
_MODES_WITH_VISION_FORWARD = {'fly_and_log', 'fly_and_viz', 'fly_only', 'log_subset'}

# Modes that should write something to disk.
_MODES_WITH_LOGGING = {'log_only', 'fly_and_log', 'fly_and_viz', 'log_subset'}

# Modes that should auto-render flight_viz plots on shutdown.
_MODES_WITH_VIZ_RENDER = {'fly_and_viz'}

_VALID_MODES = {
    'log_only', 'fly_and_log', 'fly_and_viz', 'fly_only', 'log_subset',
}


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


def _mission_manager(mission_config: dict, *, enable_waypoints: bool) -> Node:
    """Spawn the mission state machine.

    ``enable_waypoints`` is decided by the caller (typically
    ``navigation != 'tags'``) and forwarded as a ROS parameter so the
    node itself stays YAML-driven for every other knob. Tags-only
    missions intentionally skip the cruise leg: that flow is "takeoff +
    hover above pad + AprilTag-guided land" and a long horizontal
    cruise would push the marker outside the camera's FOV mid-flight.
    """
    waypoints_cfg = mission_config.get('waypoints', {}) or {}
    return Node(
        package='flight_manager',
        executable='mission_manager_node',
        name='mission_manager',
        output='screen',
        parameters=[
            {
                'target_altitude_m': mission_config.get('target_altitude_m', 7.0),
                'hover_duration_sec': mission_config.get('hover_duration_sec', 3.0),
                'enable_waypoints': bool(enable_waypoints),
                # Path is taken verbatim - mission_manager expands ~
                # itself, so a relative or ~-prefixed path in
                # mission.yaml works on any machine without launch-time
                # surgery.
                'waypoints_path': str(
                    waypoints_cfg.get('path', '~/coords.txt')
                ),
                # Defaults sized for VTOL quadplane forward-flight
                # cruise (see config/mission.yaml comments and
                # mission_manager_node declare_parameter rationale):
                # 50 m gives the VTOL->plane transition altitude
                # headroom, 80 m matches Plane GUIDED's WP_LOITER_RAD,
                # 200 s covers the full 600 m route + transitions.
                'waypoint_cruise_altitude_m': float(
                    waypoints_cfg.get('cruise_altitude_m', 50.0)
                ),
                'waypoint_arrival_radius_m': float(
                    waypoints_cfg.get('arrival_radius_m', 80.0)
                ),
                'waypoint_timeout_sec': float(
                    waypoints_cfg.get('timeout_sec', 200.0)
                ),
                'waypoint_flight_mode': str(
                    waypoints_cfg.get('flight_mode', 'GUIDED')
                ),
                'waypoint_post_mode': str(
                    waypoints_cfg.get('post_mode', 'QLOITER')
                ),
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


def _resolve_session_persist_map_path(
    discovery_cfg: dict,
    session_dir: str,
    workspace_dir: str,
) -> str:
    """Decide where (if anywhere) the localizer should persist its map.

    Default policy: the map is session-scoped. The launch creates a
    fresh ``<session_dir>/discovered_map.yaml`` for every flight, the
    localizer starts with an empty map (anchor re-bootstraps from the
    first detection), and the file is rewritten on every new confirmed
    marker. After the flight the map ships alongside the bag/video/CSV
    so you can post-analyze how accurately each marker position was
    estimated.

    Overrides (in priority order):

    1. ``tags_detector.yaml -> discovery.persist_map_path`` is the
       explicit knob. Set it to a non-empty path to keep cross-session
       persistence (e.g. you have a pre-surveyed pad). Set it to ``""``
       to force in-memory only (no disk artifacts at all). Leave the
       key absent to get the default session-scoped behavior.
    2. Modes without a session directory (``fly_only``) fall back to
       in-memory regardless, since there is no session log to attach
       the map to.
    """
    if 'persist_map_path' in discovery_cfg:
        explicit = str(discovery_cfg.get('persist_map_path') or '')
        if not explicit:
            return ''
        if os.path.isabs(explicit):
            return explicit
        return os.path.join(workspace_dir, explicit)

    if not session_dir:
        return ''
    return os.path.join(session_dir, 'discovered_map.yaml')


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
                            topics, video_enabled, csv_enabled,
                            marker_detector_profile=None):
    manifest = {
        'created_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'mode': mode,
        'navigation': navigation,
        'environment': environment,
        'rosbag_topics': topics,
        'video_enabled': video_enabled,
        'telemetry_csv_enabled': csv_enabled,
    }
    if marker_detector_profile:
        manifest['marker_detector_profile'] = marker_detector_profile
    manifest_path = os.path.join(session_dir, 'manifest.yaml')
    with open(manifest_path, 'w') as f:
        yaml.safe_dump(manifest, f, sort_keys=False, allow_unicode=True)


def _build_viz_render_entity(workspace_dir: str, session_dir: str):
    """Auto-run ``flight_viz`` over the just-recorded session on launch shutdown.

    Used by ``mode: fly_and_viz`` to turn every flight into a fixed
    set of PNG plots (one trajectory + altitude pair per nav source,
    each annotated with ATE vs GPS truth) without forcing the operator
    to remember the post-flight command. Plots land in
    ``<session_dir>/viz/`` so they ship alongside the bag/video/csv.

    Lifecycle gotcha (the reason for the ``setsid ... &`` trick below):
    when the operator hits Ctrl+C, ros2 launch starts to tear itself
    down and waits for child processes to exit, but its wait window is
    short - any subprocess that takes longer than ~1-2 s to finish
    after the OnShutdown event fires gets cut off by SIGTERM from the
    launch process. ``flight_viz`` typically takes 5-15 s (Kaleido
    spawns a headless Chromium per PNG), so a naive ExecuteProcess
    inside OnShutdown silently dies mid-render and the operator finds
    an empty ``viz/`` directory ("Я не нашёл картинки в logs").

    Workaround: we shell out to ``setsid`` which detaches the renderer
    into its own process group, then immediately backgrounds it with
    ``&``. The outer ExecuteProcess exits in <100 ms (launch is
    happy and shuts down cleanly), while the detached renderer keeps
    running independently and writes PNGs + a log file at
    ``<session>/viz/flight_viz.log``. Output is redirected to that log
    because by the time ``setsid`` runs the parent terminal is being
    torn down and ``output='screen'`` would lose lines anyway.

    A small ``sleep 2`` inside the detached child gives the rosbag
    recorder time to flush the final messages to disk after its own
    SIGINT (sqlite3 storage can drop the last second of samples
    otherwise, which makes ATE numbers wobble run-to-run).

    Only enabled for ``mode: fly_and_viz`` - other modes that produce
    a session dir still get the bag/video/csv stack but skip the
    viz step. To re-render any session manually:

        python3 -m flight_viz --session <logs/session_*> --format png

    Returns an empty list when no session_dir is configured (e.g.
    ``fly_only``), so callers can ``launch_entities.extend(...)``
    unconditionally.
    """
    if not session_dir:
        return []

    viz_out = os.path.join(session_dir, 'viz')
    viz_log = os.path.join(viz_out, 'flight_viz.log')
    # ``flight_viz`` lives in <workspace>/tools/flight_viz/ - it is not
    # an ament package on AMENT_PREFIX_PATH, so we prepend its parent
    # to PYTHONPATH explicitly. Otherwise ``-m flight_viz`` fails with
    # ModuleNotFoundError even though the directory exists on disk.
    tools_dir = os.path.join(workspace_dir, 'tools')
    proc_env = os.environ.copy()
    existing_py_path = proc_env.get('PYTHONPATH', '')
    proc_env['PYTHONPATH'] = (
        f'{tools_dir}:{existing_py_path}' if existing_py_path else tools_dir
    )

    # NOTE on quoting: session_dir and friends come from
    # _resolve_session_dir() which produces strftime-formatted paths
    # without spaces or shell metacharacters, so a bare interpolation
    # is safe here. If you ever change session_prefix to something
    # exotic, switch to shlex.quote().
    #
    # ``--format both`` so HTML is always produced (zero external
    # deps) and PNGs are written when Kaleido finds a Chrome binary.
    # The earlier ``--format png`` looked great when it worked but
    # left the operator with an empty viz dir on fresh Jetson/SITL
    # images where Chrome had never been installed - Kaleido throws
    # "Kaleido requires Google Chrome to be installed" and the only
    # evidence is buried in flight_viz.log.
    inner_cmd = (
        f'sleep 2; '
        f'echo "[flight_viz] rendering plots for {session_dir}"; '
        f'{sys.executable} -m flight_viz '
        f'--session {session_dir} '
        f'--output-dir {viz_out} '
        f'--format both; '
        f'echo "[flight_viz] done (HTML always written; PNGs only if '
        f'Kaleido finds Chrome. To enable PNG: run plotly_get_chrome)"'
    )
    viz_cmd = (
        f'mkdir -p {viz_out} && '
        f'echo "[flight_viz] launch shutdown - spawning detached renderer '
        f'(log: {viz_log})" && '
        f'setsid bash -c \'{inner_cmd}\' '
        f'> {viz_log} 2>&1 < /dev/null &'
    )
    return [
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    ExecuteProcess(
                        cmd=['bash', '-lc', viz_cmd],
                        output='screen',
                        shell=False,
                        cwd=workspace_dir,
                        env=proc_env,
                    ),
                ]
            )
        )
    ]


def _build_logging_entities(flight_config: dict, workspace_dir: str,
                            mode: str, navigation: str, environment: str,
                            session_dir: str):
    """Materialize bag/video/csv recorders for the active mode.

    Wrapped in an OpaqueFunction so the session directory and manifest
    are only created when launch is actually executed (not on
    ``ros2 launch --print`` or other dry-run inspections). The session
    directory is precomputed by ``generate_launch_description`` so the
    same string is shared with the tag_localizer's per-session map.
    """
    if mode not in _MODES_WITH_LOGGING:
        return []

    def _materialize(_context):
        log_cfg = flight_config.get('logging', {}) or {}
        safety_cfg = log_cfg.get('safety_gated_sessions', {}) or {}

        video_cfg = log_cfg.get('video', {}) or {}
        video_enabled = bool(video_cfg.get('enabled', False))

        rosbag_cfg = log_cfg.get('rosbag', {}) or {}
        topics = []
        if rosbag_cfg.get('enabled', True):
            topics = _expand_topic_groups(rosbag_cfg, mode, video_enabled)

        if safety_cfg.get('enabled', False):
            telemetry_cfg = log_cfg.get('telemetry_csv', {}) or {}
            return [
                Node(
                    package='flight_manager',
                    executable='safety_log_supervisor_node',
                    name='safety_log_supervisor',
                    output='screen',
                    parameters=[
                        {
                            'workspace_dir': workspace_dir,
                            'session_prefix': log_cfg.get('session_prefix', 'session'),
                            'log_root': log_cfg.get('log_root', 'logs'),
                            'rosbag_topics': topics,
                            'rosbag_storage_id': str(rosbag_cfg.get('storage_id', 'sqlite3')),
                            'rosbag_compression': str(
                                rosbag_cfg.get('compression', '') or ''
                            ),
                            'video_enabled': video_enabled,
                            'video_fps': float(video_cfg.get('fps', 30.0)),
                            'video_chunk_duration_sec': float(
                                video_cfg.get('chunk_duration_sec', 60.0)
                            ),
                            'video_topic': str(video_cfg.get('topic', '/camera/image_raw')),
                            'telemetry_csv_enabled': bool(
                                telemetry_cfg.get('enabled', False)
                            ),
                            'telemetry_csv_rate_hz': float(
                                telemetry_cfg.get('rate_hz', 10.0)
                            ),
                            'stack_mode': mode,
                            'stack_navigation': navigation,
                            'stack_environment': environment,
                            # Where the tag_localizer streams its per-flight
                            # discovered_map.yaml + anchor_*.yaml. The
                            # supervisor snapshots both into every cycle dir
                            # at safety-engaged so each cycle folder ships
                            # self-contained for post-flight analysis.
                            'localizer_runtime_dir': session_dir,
                            # Path to markers config. Loaded by the supervisor
                            # so each cycle's manifest.yaml includes the same
                            # marker_detector_profile block the
                            # non-safety-gated path already writes.
                            'markers_yaml_path': os.path.join(
                                workspace_dir, 'config', 'markers.yaml'
                            ),
                        }
                    ],
                )
            ]

        os.makedirs(session_dir, exist_ok=True)

        entities = []

        if rosbag_cfg.get('enabled', True):
            if topics:
                bag_dir = os.path.join(session_dir, 'bag')
                entities.append(_bag_record_process(rosbag_cfg, topics, bag_dir))

        if video_enabled:
            entities.append(_video_recorder(video_cfg, session_dir))

        telemetry_cfg = log_cfg.get('telemetry_csv', {}) or {}
        csv_enabled = bool(telemetry_cfg.get('enabled', False))
        if csv_enabled:
            entities.append(_telemetry_csv_logger(telemetry_cfg, session_dir))

        markers_path = os.path.join(workspace_dir, 'config', 'markers.yaml')
        markers_cfg = _load_yaml(markers_path)
        marker_profile = {
            'dictionary': markers_cfg.get('dictionary'),
            'default_marker_size': float(markers_cfg.get('default_marker_size', 0.2)),
            'markers': markers_cfg.get('markers', []),
        }
        _write_session_manifest(
            session_dir, mode, navigation, environment, topics,
            video_enabled, csv_enabled,
            marker_detector_profile=marker_profile,
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

    # Resolve the session directory ONCE up here so the same path is
    # shared by:
    #   * the tag_localizer's persist_map_path (so each session gets a
    #     fresh map written into its own log dir, see below);
    #   * _build_logging_entities (bag/video/csv all under the same dir).
    # Previously the session dir was rebuilt inside the logging OpaqueFunction
    # which produced a slightly different timestamp string and made it
    # impossible to colocate the localizer map with the rest of the session.
    log_cfg = flight_config.get('logging', {}) or {}
    session_dir = (
        _resolve_session_dir(log_cfg, workspace_dir)
        if mode in _MODES_WITH_LOGGING else ''
    )

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
        # By default anchor_<timestamp>.yaml lives alongside the rest of
        # the session log; only fall back to logs/maps when no session
        # dir is available (e.g. fly_only).
        'anchor_log_dir': (
            session_dir if session_dir
            else os.path.join(workspace_dir, 'logs', 'maps')
        ),
        # Session-scoped marker map. See `_resolve_session_persist_map_path`
        # for the override semantics.
        'persist_map_path': _resolve_session_persist_map_path(
            discovery_cfg, session_dir, workspace_dir
        ),
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
    # The discovery keys we forward to localizer_node. Note:
    #   * `strategy` -> `anchor_strategy` is remapped explicitly below.
    #   * `persist_map_path` is intentionally NOT in this list - its
    #     final value is already computed above via
    #     `_resolve_session_persist_map_path` and must not be overwritten
    #     here, otherwise an absent key in the YAML would clobber the
    #     session-scoped default we just set.
    _DISCOVERY_KEYS = ('learn_unknown_tags',)
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
            # Tags-only flow is "takeoff + AprilTag-guided land": the
            # waypoint cruise would fly the pad out of camera FOV, so we
            # explicitly disable it here regardless of mission.yaml.
            nav_entities.append(
                _mission_manager(mission_config, enable_waypoints=False)
            )
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
            # Hybrid uses tags for takeoff/landing reference and DPVO
            # (or ORB-SLAM3) for cruise, so flying the waypoint route
            # is the whole point. Enabled by default; per-file knobs
            # live in mission.yaml -> mission.waypoints.
            hybrid_entities.append(
                _mission_manager(mission_config, enable_waypoints=True)
            )
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
            flight_config, workspace_dir, mode, navigation, environment,
            session_dir,
        )
    )

    # === VIZ RENDER ON SHUTDOWN (fly_and_viz only) ===
    if mode in _MODES_WITH_VIZ_RENDER:
        launch_entities.extend(
            _build_viz_render_entity(workspace_dir, session_dir)
        )

    return LaunchDescription(launch_entities)
