# Diagnostic status published exclusively by the tag localizer.
# For mission-level gating, subscribe to NavStatus instead - this
# message remains for tag-specific diagnostics (which markers are
# currently in view, etc.).

std_msgs/Header header
bool pose_valid
string active_marker_source
int32[] visible_ids
float32 quality_score

# Numeric state. Mirrors NavStatus state codes.
uint8 state_code

uint8 STATE_UNKNOWN          = 0
uint8 STATE_NO_DATA          = 1
uint8 STATE_INITIALIZING     = 2
uint8 STATE_SETTLING         = 3
uint8 STATE_TRACKING_DEGRADED = 4
uint8 STATE_TRACKING         = 5
uint8 STATE_LOST             = 6
uint8 STATE_REJECTED         = 7

# Free-form state name, kept for backwards compatibility with logs and
# rqt tools. Mission gating must use state_code, not this field.
string state
