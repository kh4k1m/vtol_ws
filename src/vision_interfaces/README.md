# vision_interfaces

Source-agnostic ROS 2 interfaces for visual navigation backends.

The contract behind this package: **any visual navigation system
(VO / VIO / SLAM / global-map binder) is interchangeable from the
flight stack's point of view as long as it publishes the topics below.**

The flight stack lives in `~/vtol_ws`. Concrete backends can live
anywhere on disk (separate repos, separate Python / CUDA environments)
and are wired into the flight stack via a thin ROS 2 bridge that talks
these topics.

---

## Topic contract

For a backend named `<bk>` (e.g. `dpvo`, `orb_slam3`, `vins_mono`,
`globalmap_reloc`):

| Topic                              | Type                                     | Required for | Description                                                                 |
|------------------------------------|------------------------------------------|--------------|-----------------------------------------------------------------------------|
| `/vision/<bk>/pose`                | `geometry_msgs/PoseStamped`              | all          | Latest pose in the backend's local frame.                                   |
| `/vision/<bk>/status`              | `vision_interfaces/VisionStatus`         | all          | Health, uncertainty, scale info. Published at >=5 Hz even if pose stalls.   |
| `/vision/<bk>/odom`                | `vision_interfaces/VisionOdometry`       | VIO / SLAM   | Pose + twist with covariance + inline status. Skip for pure pose VO.        |
| `/vision/<bk>/path`                | `nav_msgs/Path`                          | optional     | Rolling-window trajectory, for rviz.                                        |
| `/vision/<bk>/keyframe`            | `vision_interfaces/KeyFrameEvent`        | SLAM only    | Keyframe insertions, loop closures, relocs.                                 |
| `/vision/<bk>/global_anchor`       | `vision_interfaces/GlobalAnchor`         | global-map   | Anchor of the local frame to WGS84. Publish on change.                      |
| `/vision/<bk>/reset` (service)     | `vision_interfaces/ResetVision`          | all          | Drop state, optionally seed with a pose. Used by mission_manager.           |
| `/vision/<bk>/set_global_anchor`   | `vision_interfaces/SetGlobalAnchor`      | optional     | Force the local frame to align to a given global anchor.                    |

### What the flight stack subscribes to

The flight stack itself **never** subscribes to a specific backend.
Instead it consumes the unified pair:

* `/vision_pose_enu`  (`geometry_msgs/PoseStamped`)
* `/nav/status`       (`marker_interfaces/NavStatus`, the legacy
  predecessor of `VisionStatus` - field layout is identical)

These are produced by either:

* `single_source_router_node` - in single-backend modes, simply forwards
  one source onto the unified topics.
* `vision_fusion_node`        - in hybrid modes, multiplexes a tag
  localizer with a VO source (TAG_MODE / VO_MODE state machine).

**To add a new backend** you:

1. Bridge it into ROS 2 publishing on `/vision/<bk>/*` as above.
2. Pick a launch file (or write a new one in `bringup/launch/`) that
   plugs `<bk>` into `single_source_router` or `vision_fusion_node`.
3. Nothing else changes in the flight code.

---

## Backwards compatibility with `marker_interfaces/NavStatus`

`VisionStatus` is a strict superset of `marker_interfaces/NavStatus`:
the first nine fields and the eight state constants are identical, only
SLAM-specific fields (`n_map_points`, `n_keyframes`, `scale_factor`,
`has_global_anchor`) are appended. The router / fusion / mission
manager nodes already subscribe to `NavStatus` and stay working;
new backends that want the richer fields publish `VisionStatus` and
let a tiny adapter (or the bridge itself) emit both legacy and new
topics during the transition.

---

## State machine (`VisionStatus.state`)

```
UNKNOWN -> NO_DATA -> INITIALIZING -> SETTLING -> TRACKING_DEGRADED <-> TRACKING
                                                            |
                                                            v
                                                          LOST -> INITIALIZING
```

`REJECTED` is set by *consumers* (e.g. the fusion node when a pose is
discarded as an outlier), not by the backend itself.

Mission gating in `mission_manager_node`:

* `TRACKING` and `TRACKING_DEGRADED` count as "ready to fly".
* `SETTLING` is acceptable for log-only modes but blocks ARM.
* Anything else blocks takeoff and triggers an abort-to-LAND mid-flight.
