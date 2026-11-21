# Changelog

## Unreleased

- Per-robot baselines: `--learn SECS` records each stream's normal message rate, `--baseline` warns with `RATE_DROP` / `RATE_SPIKE` below half or above double that. Plain statistics, no model. Run live against the fake camera (learn at 15 Hz, quiet at 15 Hz, `RATE_DROP` at 6 Hz).
- Fix: `IMAGE_FROZEN` is an error again for identical frames with advancing stamps on a **real** clock (a ZED X freeze looks exactly like that). `IMAGE_STATIC` (warn) is now only for simulators: `/clock` live or sim-time stamps. 14 unit tests; fake camera verified in both modes; real webcam stays quiet. `test/fake_camera.py --sim` drives sim time.
- `test/webcam_bridge.py` publishes a local webcam to ROS. Run on a real USB webcam: healthy over 581 frames (about 26 Hz), no false static or frozen findings.

## 0.1.0
- First version: camera, depth and point cloud health for a running ROS 2 robot. Read-only.
- Checks: `NO_SENSORS`, `STREAM_NONE`, `STREAM_STALE`, `LOW_RATE`, `STREAM_LAG`, `IMAGE_DARK`, `IMAGE_FLAT`, `IMAGE_SATURATED`, `IMAGE_FROZEN`, `DEPTH_POOR`, `DEPTH_DEAD`, `CLOUD_EMPTY`, `NO_CAMERA_INFO`.
- `IMAGE_STATIC` (warn) vs `IMAGE_FROZEN` (error): identical frames with advancing stamps are a still scene on a noiseless simulated camera, not a stuck driver. Found on a UR3 Gazebo sim where 81 frames had one distinct image.
- 11 unit tests of the checks (no ROS needed). Run end to end against a fake camera (healthy, dark, frozen, no camera info, silent) and a UR3 Gazebo sim with head and wrist cameras (healthy; pausing the sim gives `STREAM_STALE` on all 7 streams, recovering afterwards).
