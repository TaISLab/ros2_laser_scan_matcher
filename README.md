# Laser Scan Matcher for ROS2

Ported to ros2 version of laser-scan-matcher by [scan_tools](https://github.com/ccny-ros-pkg/scan_tools).
Scan-to-scan matching via CSM (Canonical Scan Matcher / PL-ICP, [Censi 2008](https://purl.org/censi/2006/icpcov)).

## Installation
* Install modified version of [csmlib](https://github.com/AlexKaravaev/csm)

## Topics

### Subscribed topics
- `laser_scan_topic` ([sensor_msgs/LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html)) — param, default `scan`.

### Published topics
- `/tf` ([tf2_msgs/TFMessage](http://docs.ros.org/melodic/api/tf2_msgs/html/msg/TFMessage.html)), transform `odom_frame -> base_frame` (or the reverse if `invert_tf` is set). Only if `publish_tf` is true.
- `publish_odom` ([nav_msgs/Odometry](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Odometry.msg)). Optional: only published if this parameter is set to a non-empty topic name.

## Will be released features:
- [x] Support of pure laserscan
- [ ] Support of IMU
- [ ] Support of odometry
- [ ] Support of PointCloud msgs

## Odometry message: covariance and twist frame

Historically this node published `nav_msgs/Odometry` with:
- `pose.covariance` and `twist.covariance` **always zero** (the fields were simply never written).
- `twist.twist.angular.x` carrying the yaw rate, instead of `angular.z`.
- `twist.twist.linear.{x,y}` computed as the raw **world-frame** (`odom_frame`) position delta between scans, divided by `dt` — but `nav_msgs/Odometry.twist` is defined in `child_frame_id` (REP 103: a body-fixed frame), not in the world frame.

All three are fixed now. Why they mattered, for anyone fusing this node's output with `robot_localization` or similar:

- **Zero covariance** tells a consumer to trust the measurement with *infinite* confidence. `robot_localization`'s Mahalanobis-distance rejection gate divides by this covariance, so a value of exactly zero is a landmine — depending on gate settings it can cause spurious rejection of every subsequent update once the filter's own belief drifts even slightly, or numerically undefined behavior. Use the new `pose_covariance_diagonal` / `twist_covariance_diagonal` parameters (below) instead of leaving this at the old implicit zero.
- **Yaw rate on the wrong axis** silently breaks any consumer that fuses `vyaw` from this topic (e.g. `robot_localization`'s `odom0_config` with `vyaw: true`) — it would read a constant-zero roll-rate instead of the actual heading rate.
- **World-frame twist mislabeled as body-frame** double-rotates the velocity in a consumer that (correctly, per REP 103) re-projects `twist.linear` into the world frame using its own current yaw estimate before integrating it (this is exactly what `robot_localization`'s internal state transition does). Validated live against a mocap-labeled bag: with this bug, an EKF fusing this node's position *and* twist could report a plausible-looking straight-line path while its internal yaw state was left with no valid correction signal — see the frame gotcha below, which turned out to be the dominant failure mode in that same test but this bug independently corrupts velocity-only consumers (e.g. a filter fusing only `vx`/`vy` from this topic, without also fusing absolute pose).

### New parameters

```yaml
laser_scan_matcher_node:
  ros__parameters:
    # Diagonal of the published pose covariance: x, y, z, roll, pitch, yaw
    # (m^2, rad^2). CSM does not estimate this per-scan unless
    # do_compute_covariance is enabled (expensive), so this is a fixed,
    # tunable estimate.
    pose_covariance_diagonal:  [0.0025, 0.0025, 1.0e-6, 1.0e-6, 1.0e-6, 0.0012]
    # Diagonal of the published twist covariance: vx, vy, vz, vroll, vpitch,
    # vyaw ((m/s)^2, (rad/s)^2).
    twist_covariance_diagonal: [0.01,   0.01,   1.0e-6, 1.0e-6, 1.0e-6, 0.02]
```

These are starting points, not calibrated defaults — tune them against your own ground truth (see [Validation](#validation-against-mocap-ground-truth) below for the tooling used to do that for this fork).

## Critical: `odom_frame` must be usable by TF, or `robot_localization` will silently drop every pose update

This is the single most important integration gotcha, and it does **not** raise any error or warning that points at the real cause — it took reading `robot_localization`'s own source (`ros_filter.cpp::preparePose`) to track down.

The published `nav_msgs/Odometry` message carries `header.frame_id = odom_frame` (the `odom_frame` parameter). If you feed this into `robot_localization` (`ekf_node`/`ukf_node`) as an `odomN` input, `robot_localization` needs a TF path from this message's `header.frame_id` to its own `world_frame` before it will fuse **any** of the position or orientation in that message — position and orientation both, together, all-or-nothing (see `RosFilter<T>::preparePose`, specifically the `lookupTransformSafe(target_frame=world_frame, source_frame=msg.header.frame_id, ...)` call and its `if (can_transform) {...} else { retVal = false; }` branch).

`lookupTransformSafe` only has one fallback when it can't find a real TF path: if `target_frame == source_frame` it uses an identity transform. There is **no fallback for two different, unconnected frame names** — the lookup just fails, and the entire pose measurement (x, y, z, roll, pitch, **and yaw**) is dropped for that message, silently, forever, on every single update.

Meanwhile, this node's `twist` half of the same `Odometry` message is expressed in `child_frame_id` (`base_frame`), which typically *does* match `robot_localization`'s `base_link_frame` — so twist fusion (`vx`, `vy`, `vyaw` if configured) keeps working fine even while pose fusion is completely broken. **This makes the failure very easy to miss**: a filter fusing both this node's absolute pose and its twist can look like it's tracking (position keeps moving, driven by the still-working twist integration) while its yaw state is frozen at whatever it started at, because the only source of *absolute* heading correction (this node's pose) never actually reaches the filter.

Concretely, in one project built on this fork, `odom_frame` was set to a made-up name (`scan_odom`) that was never published anywhere in the TF tree, while the EKF's `world_frame` was `odom`. Result, verified against mocap ground truth: yaw output frozen at exactly `0.0 rad` for the entire run (position still advanced, since it happened to be driven by the still-working twist path — see above), regardless of which `odom0_config` / `differential` / `relative` combination was tried on the EKF side, because none of those settings matter if `preparePose` never gets past its TF lookup.

**Fix:** set `odom_frame` to the exact same frame name as the consuming filter's `world_frame` (commonly `odom`). Since this node does not itself broadcast that frame's TF unless `publish_tf: true`, and `robot_localization` is the one that will publish `odom -> base_frame`, using the same name here means `lookupTransformSafe`'s identity fallback (`target_frame == source_frame`) always succeeds immediately — no dependency on TF broadcast order at startup, no orphan frame name floating around that nothing ever connects.

If you genuinely need a distinct frame name here (e.g. running this node's output through something other than `robot_localization`, or wanting a clearly-labeled "raw scan-matcher-only" frame for debugging in `rviz`), you must also publish a static (or dynamic) transform bridging that name to whatever `world_frame` your EKF uses, or configure that consumer's own frame-remapping if it offers one — otherwise expect this exact silent-drop failure mode.

## Validation against mocap ground truth

This fork's fixes (covariance, twist frame/axis) and the `odom_frame` gotcha above were found and verified using rosbags with motion-capture ground truth (`mocap4r2_msgs/RigidBodies`) recorded alongside a robot using this node, comparing the fused `nav_msgs/Odometry` trajectory against the mocap-tracked rigid body pose (absolute trajectory error after 2D rigid alignment, plus yaw error). That evaluation tooling — `test_laser_imu_odom.launch.py` (bag replay) and `eval_odom_against_mocap.py` (ATE/yaw-error scoring) — lives in the consuming project (`WalKit/walker_bringup` and `WalKit/walker_step_detector/scripts`), not in this package, since it's specific to that robot's mocap setup and calibration; this section exists here so the failure modes above are traceable back to how they were actually diagnosed and confirmed fixed.
