"""Replay tag_localizer pipeline against a recorded session offline.

Reads `/tag_detections` and ground-truth altitude from a session bag and
runs them through `tag_localizer.geo.tagDB` directly (no ROS process,
no rclpy spin). Lets us grid-search localizer parameters over a
recorded flight in seconds instead of doing a new SITL run for every
combination.

Entry point: ``python3 -m tools.localizer_replay <session_dir> ...``.
"""
