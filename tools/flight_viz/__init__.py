"""flight_viz: offline visualization of vtol_ws flight sessions.

A session is a directory produced by ``flight.launch.py`` (typically
``logs/session_<timestamp>/``) and contains:

* ``bag/`` - rosbag2 (mcap or sqlite3)
* ``video/`` - chunked MP4 recordings
* ``flight_data_<ts>.csv`` - flat telemetry CSV
* ``manifest.yaml`` - what was recorded and in what mode

This package reads the bag + auxiliary CSVs and renders one or more
plots described by ``config/visualize.yaml``. See ``README.md`` for the
plot catalog.
"""

__all__ = []
