# Anchors a backend's local frame to a global reference (typically WGS84
# ENU at a known lat/lon). Published on /vision/<backend>/global_anchor
# whenever the anchor changes (first lock, drift correction, manual reset).
#
# Convention:
#   p_global_enu = q_local_to_enu * p_local + t_local_to_enu
# where t_local_to_enu is the ENU offset, in meters, of the local frame
# origin from the anchor lat/lon/alt, and q_local_to_enu is the rotation
# (as a unit quaternion) from the local frame to ENU.

std_msgs/Header header

# Anchor in WGS84. alt is meters above ellipsoid (or whatever frame your
# autopilot is using - keep one convention per session).
float64 anchor_lat
float64 anchor_lon
float64 anchor_alt_m

# Rotation that takes a vector expressed in the backend's local frame
# into the ENU frame attached to (anchor_lat, anchor_lon).
geometry_msgs/Quaternion q_local_to_enu

# Translation (ENU, meters): backend local-frame origin expressed in
# ENU at the anchor.
geometry_msgs/Vector3 t_local_to_enu

# Estimated standard deviations of the anchor itself (separate from
# per-pose uncertainty, which lives in VisionStatus).
float32 anchor_position_std_m
float32 anchor_rotation_std_rad

# Who set this anchor. Examples:
#   "manual"             - operator placed via /vision/<bk>/set_global_anchor
#   "tag:anchor_id=3"    - tag_localizer locked onto a known-pose marker
#   "gps_init"           - initial GPS fix snapped to the first VO pose
#   "fusion:hybrid"      - vision_fusion_node derived the offset
string anchor_source
