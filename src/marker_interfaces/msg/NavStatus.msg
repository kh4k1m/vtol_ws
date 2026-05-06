# Unified navigation status published by every pose source (tag localizer,
# DPVO, ORB-SLAM, vision fusion, ...). Consumers like the mission state
# machine and the MAVLink bridge subscribe to this topic to learn the
# current source, its health, and the uncertainty bounds to apply to
# downstream filters.

std_msgs/Header header

# State enum. Use the constants below.
uint8 state

uint8 STATE_UNKNOWN          = 0
uint8 STATE_NO_DATA          = 1
uint8 STATE_INITIALIZING     = 2
uint8 STATE_SETTLING         = 3
uint8 STATE_TRACKING_DEGRADED = 4
uint8 STATE_TRACKING         = 5
uint8 STATE_LOST             = 6
uint8 STATE_REJECTED         = 7

# Active source identifier, e.g. "tags", "dpvo", "orb_slam3", "fusion".
string source

# Free-form descriptor of the active sub-source, e.g. "markers_[3,4]"
# for the tag localizer or "fusion:dpvo+tag_offset" for the fuser.
string detail

# Health score in [0, 1] (1 = ideal). Source-specific definition.
float32 quality_score

# Standard deviations to apply to position/orientation/velocity when
# forwarding this pose to ArduPilot's external-nav fusion.
float32 position_std_m
float32 rotation_std_rad
float32 velocity_std_m_s

# True when this status carries a velocity sample valid for the same
# stamp as the last published pose.
bool has_velocity
geometry_msgs/Vector3 velocity_enu

# Optional: list of marker IDs (or feature IDs) currently used by the
# source. Empty for non-marker pipelines.
int32[] visible_ids
