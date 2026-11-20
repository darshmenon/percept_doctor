# Changelog

## 0.1.0
- First version: camera, depth and point cloud health for a running ROS 2 robot. Read-only.
- Checks: `NO_SENSORS`, `STREAM_NONE`, `STREAM_STALE`, `LOW_RATE`, `STREAM_LAG`, `IMAGE_DARK`, `IMAGE_FLAT`, `IMAGE_SATURATED`, `IMAGE_FROZEN`, `DEPTH_POOR`, `DEPTH_DEAD`, `CLOUD_EMPTY`, `NO_CAMERA_INFO`.
- `IMAGE_STATIC` (warn) vs `IMAGE_FROZEN` (error): identical frames with advancing stamps are a still scene on a noiseless simulated camera, not a stuck driver. Found on a UR3 Gazebo sim where 81 frames had one distinct image.
- 11 unit tests of the checks (no ROS needed). Run end to end against a fake camera (healthy, dark, frozen, no camera info, silent) and a UR3 Gazebo sim with head and wrist cameras (healthy; pausing the sim gives `STREAM_STALE` on all 7 streams, recovering afterwards).
