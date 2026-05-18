# Optional SLAM-only event topic on /vision/<backend>/keyframe.
# Used by SLAM backends to notify consumers (logger, viz, re-localizer)
# about discrete events that affect the global map. Non-SLAM backends
# simply do not publish on this topic.

std_msgs/Header header

uint8 event_type
uint8 EVENT_KEYFRAME_INSERTED   = 0
uint8 EVENT_KEYFRAME_CULLED     = 1
uint8 EVENT_LOOP_CLOSURE        = 2
uint8 EVENT_RELOCALIZATION      = 3
uint8 EVENT_MAP_RESET           = 4
uint8 EVENT_GLOBAL_BA_FINISHED  = 5

# Index of the keyframe this event refers to. Backend-defined ID space.
int64 keyframe_id

# Pose of the keyframe in the local backend frame at the time of the
# event. (For LOOP_CLOSURE / RELOCALIZATION this is the pose AFTER
# correction.)
geometry_msgs/PoseStamped keyframe_pose

# For LOOP_CLOSURE: the matched older keyframe ID. -1 otherwise.
int64 matched_keyframe_id

# Free-form payload (e.g. "covisibility=42", "scale_correction=0.97").
string detail
