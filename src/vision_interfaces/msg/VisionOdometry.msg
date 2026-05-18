# Combined pose + velocity + status sample published on
# /vision/<backend>/odom. Designed for VIO / metric-scale SLAM pipelines
# that have a meaningful velocity estimate; pure pose VOs may omit this
# topic and rely on PoseStamped on /vision/<backend>/pose alone.
#
# The pose and twist are in the backend's local frame (typically defined
# by the first keyframe). To convert into the global frame, combine
# with GlobalAnchor.

std_msgs/Header header

# Pose + twist in the local backend frame.
nav_msgs/Odometry odometry

# Inline health snapshot for the same stamp. Saves a join with the
# /vision/<backend>/status topic in tight control loops.
VisionStatus status
