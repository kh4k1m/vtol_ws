# Source-agnostic health/uncertainty status published by every visual
# navigation backend on /vision/<backend>/status.
#
# This is the SUPERSET of marker_interfaces/NavStatus and is meant to
# replace it for VO/VIO/SLAM/global-map backends. The mission layer
# subscribes to a single unified /nav/status and only cares about the
# state code + std bounds; the backend-specific fields below let viz
# and post-mortem tools inspect what each pipeline is actually doing.
#
# Backends MUST set: state, source, position_std_m, rotation_std_rad.
# All other fields are optional (use defaults / zero).

std_msgs/Header header

# ----- Common state -----

# Lifecycle code. Same numbers as marker_interfaces/NavStatus to keep
# downstream gating logic (mission_manager) trivially compatible.
uint8 state
uint8 STATE_UNKNOWN           = 0
uint8 STATE_NO_DATA           = 1
uint8 STATE_INITIALIZING      = 2   # backend started, not tracking yet
uint8 STATE_SETTLING          = 3   # tracking but quality below threshold
uint8 STATE_TRACKING_DEGRADED = 4   # tracking with reduced confidence
uint8 STATE_TRACKING          = 5   # nominal
uint8 STATE_LOST              = 6   # tracking failure, attempting recovery
uint8 STATE_REJECTED          = 7   # consumer-side rejection (outlier, big jump, ...)

# Backend identifier (free-form). Examples: "dpvo", "orb_slam3",
# "vins_mono", "tag_localizer", "fusion:dpvo+tag", "globalmap:reloc_v1".
string source

# Optional human-readable detail, e.g. "loop_closure_pending",
# "scale_uncertain", "imu_init_in_progress".
string detail

# Backend-defined health score in [0, 1]. 1 = ideal.
# Convention suggestions:
#   * tag_localizer  - normalized inverse of reprojection_error
#   * VO / SLAM      - tracked features / target_features
#   * fusion         - max(child quality_scores) * temporal continuity
float32 quality_score

# ----- Uncertainty (forwarded to ArduPilot ODOMETRY std fields) -----

float32 position_std_m
float32 rotation_std_rad
float32 velocity_std_m_s

# True when a velocity sample is published on the matching /vision/<bk>/odom
# topic for the same stamp as the last pose.
bool has_velocity
geometry_msgs/Vector3 velocity_enu

# ----- Optional SLAM / map-aware fields -----

# Number of active map points / features the backend is currently using.
# 0 if not applicable (pure pose-graph VO).
int32 n_map_points

# Number of keyframes in the active session. 0 for non-keyframe systems.
int32 n_keyframes

# Estimated metric scale factor.
# 1.0 = output is already in meters (default for VIO, RGB-D, stereo).
# != 1.0 = pure monocular VO that has been externally aligned, or that
# self-reports its scale uncertainty (use rotation_std_rad / position_std_m
# to express the uncertainty itself).
float32 scale_factor

# True when the backend has anchored its local frame to a global frame
# (lat/lon/alt). The current anchor is published as GlobalAnchor on
# /vision/<bk>/global_anchor.
bool has_global_anchor

# ----- Tag / feature provenance -----

# Marker / feature IDs in view. Empty for non-marker backends.
int32[] visible_ids
