# bringup/launch

One launch file lives here:

**`flight.launch.py`** - omnibus launch driven by `config/flight.yaml`.
It supports every supported combination of `environment` x `navigation`
x `mode`. This is the entrypoint used by the autostart service and all
existing tooling. **Do not delete or break it.**

The old `mission_*.launch.py` family (apriltag-only takeoff/land,
DPVO-only logging, AprilTag-takeoff + DPVO-cruise hybrid) has been
folded back into `flight.launch.py`: every scenario it used to offer is
expressible as a pair of `navigation` + `mode` values in
`config/flight.yaml`. Keeping a single launch fixes the systemic problem
where the side launches diverged from `flight.launch.py` over time
(different tag tuning, different anchor strategy, missing logging, ...)
and surprised the operator at the worst possible moment.

---

## How the old scenarios map to `flight.yaml`

| Old launch                                         | `navigation` | `mode`        | Notes                                                                                                        |
|----------------------------------------------------|--------------|---------------|--------------------------------------------------------------------------------------------------------------|
| `mission_apriltag_takeoff_land.launch.py`          | `tags`       | `fly_and_log` | mission_manager arms once `/nav/status == TRACKING`, takes off to `mission.target_altitude_m`, hovers, lands. |
| `mission_dpvo_only_log.launch.py`                  | `dpvo`       | `log_only`    | mission_manager is disabled; vision NOT forwarded to ArduPilot; bag + telemetry CSV record everything.       |
| `mission_apriltag_takeoff_dpvo_cruise.launch.py`   | `hybrid`     | `fly_and_log` | `vision_fusion_node` multiplexes `/tag_localizer/pose` + `/dpvo/pose` onto the unified `/vision_pose_enu`.   |

For SITL smoke tests where you want the same flight plus an
automatically rendered set of trajectory / altitude / ATE plots, pick
`mode: fly_and_viz` instead of `fly_and_log` - everything else stays
identical.

---

## Waypoint cruise between hover and land

The vehicle is a VTOL quadplane, and the navigation backends
deliberately exercise different airframe regimes:

* `navigation: tags` is **quadcopter only**. The flow is
  AprilTag-guided VTOL takeoff -> QLOITER hover above the pad ->
  AprilTag-guided QLAND. No GUIDED, no forward-flight transition; if
  you ever needed a 600 m arc here, the pad would leave the downward
  camera's FOV and the localizer would lose its anchor.
* `navigation: dpvo` and `navigation: hybrid` exercise the **plane**
  half of the airframe. `mission_manager` inserts a forward-flight
  cruise leg between the VTOL takeoff hover and the final landing:
  read waypoints from `~/coords.txt`
  (`mission.waypoints.path` in `config/mission.yaml`, one
  `lat lon [alt_rel_m]` per line, `#` comments OK), switch to
  `GUIDED`, send each point via `/ap/cmd/goto_global`. ArduPlane
  initiates a VTOL->forward-flight transition on the GUIDED entry,
  flies to the target as a plane, and enters a loiter circle of
  radius `WP_LOITER_RAD` (~60 m). We advance to the next waypoint
  once the horizontal haversine distance drops below
  `mission.waypoints.arrival_radius_m` (default 80 m =
  `WP_LOITER_RAD` + slack), then switch to `QLOITER` to trigger the
  back-transition to VTOL before QLAND.

SITL note: keep `Q_GUIDED_MODE=0` (ArduPlane default; pinned in
`config/ardupilot/visual_only_sitl.parm`). With `Q_GUIDED_MODE=1` the
GUIDED mode would behave as a multirotor position-hold and the
forward-flight transition would never fire - the quadplane would
crawl between waypoints as a copter, defeating the point of
DPVO-under-forward-flight testing.

---

## Mode cheat-sheet

* `log_only`     - record everything, do **not** forward visual pose into ArduPilot, no mission_manager. Use for ground-truth replays.
* `fly_and_log`  - record everything **and** fly autonomously. Default for real-vehicle flights.
* `fly_and_viz`  - same as `fly_and_log` plus an automatic `flight_viz`
                   render pass on shutdown. Drops PNG plots into
                   `<session>/viz/`: per-source horizontal trajectory
                   (truth vs predicted, with ATE annotated), altitude
                   over time, XYZ per axis, and tag visibility. Use for
                   SITL iteration.
* `fly_only`     - autonomous flight, no logging on disk. Use for rapid SITL iteration where bag bytes are wasteful.
* `log_subset`   - autonomous flight with only the `logging.subset_groups` profile recorded.

---

## How a new VO / VIO / SLAM backend gets added

1. Bridge it into ROS 2 publishing on `/vision/<bk>/pose` +
   `/vision/<bk>/status` + (optional) `/vision/<bk>/odom`. See
   `src/vision_interfaces/README.md` for the full contract.
2. Wire the backend's startup into a new branch under
   `if navigation == 'your_backend':` in `flight.launch.py` so it
   coexists with `single_source_router_node` (the 1:1 forwarder that
   feeds `/vision_pose_enu`).
3. If you want it usable for AprilTag-anchored cruise, add it as a
   `vo_backend` choice next to `dpvo` / `orb_slam3` in the `hybrid`
   branch.

No changes to `mission_manager_node`, `single_source_router_node`,
`vision_fusion_node` or `ardupilot_mavlink_bridge` are needed - they all
already consume the unified `/vision_pose_enu` + `/nav/status` pair.
