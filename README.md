# percept_doctor

[![CI](https://github.com/darshmenon/percept_doctor/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/darshmenon/percept_doctor/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/license-Apache--2.0-green)](LICENSE)

**Why can't my robot see?** Attach to a running ROS 2 robot and get a one-line cause for a broken camera, depth sensor or point cloud. Read-only: it subscribes to sensor topics and never publishes or changes anything.

Companion to [nav2_doctor](https://github.com/darshmenon/nav2_doctor) (why isn't my robot moving?), [armdoc](https://github.com/darshmenon/armdoc) (why isn't my arm moving?) and [slam_doctor](https://github.com/darshmenon/slam_doctor) (is my SLAM healthy?).

## Quick start

```bash
ros2 run percept_doctor percept_doctor                  # live, a report every 2 s
ros2 run percept_doctor percept_doctor --hints          # with a suggested next step per finding
ros2 run percept_doctor percept_doctor --once 10        # watch 10 s, print one report, exit 1 if any ERROR (for CI)
ros2 run percept_doctor percept_doctor --ns /camera     # only topics under one namespace
```

Sensor topics are found by message type (`sensor_msgs/Image`, `PointCloud2`), up to 8 at a time, so it needs no configuration. Compressed and theora variants are skipped.

```
ERROR [IMAGE_FROZEN] /cam/image_raw delivers identical frames (10+ in a row): the camera driver is stuck
      -> restart the camera driver; a stuck pipeline keeps republishing the last frame
```

## What it checks

| Code | Level | Means |
|---|---|---|
| `NO_SENSORS` | warn | no image, depth or point cloud topic on the graph (wrong domain or namespace, driver not running) |
| `STREAM_NONE` | error | a sensor topic is advertised but no message ever arrived (driver up but silent, or a QoS mismatch) |
| `STREAM_STALE` | error | a stream went silent for more than 2 s |
| `LOW_RATE` | warn | a live stream runs below 5 Hz |
| `STREAM_LAG` | warn | messages arrive more than 0.5 s after their header stamp (ignored under sim time) |
| `IMAGE_DARK` / `IMAGE_FLAT` / `IMAGE_SATURATED` | warn | very dark, no contrast (covered lens or blank), or blown out |
| `IMAGE_FROZEN` | error | ten or more identical frames with an identical header stamp: the driver is stuck |
| `IMAGE_STATIC` | warn | identical frames while the stamps advance: a still scene on a noiseless simulated camera, or a driver re-stamping a stale frame |
| `DEPTH_POOR` / `DEPTH_DEAD` | warn / error | under 20% or under 1% of depth pixels are valid |
| `CLOUD_EMPTY` | error | the point cloud has zero points |
| `NO_CAMERA_INFO` | warn | image streams are live but no `camera_info` exists |

Thresholds are the defaults in `Config` (`core.py`); `--grace`, `--stale` and `--min-rate` change the common ones.

## Limits

- Image statistics come from a small subsample (about 48 x 64 pixels), so it is cheap, but it cannot see a small defect.
- Supported encodings: `rgb8`, `bgr8`, `rgba8`, `bgra8`, `mono8`, `mono16`, `16UC1`, `32FC1`. Others (Bayer, YUV) are counted for rate and staleness only.
- It does not check calibration quality, focus, or whether a detector finds anything.
- `PointCloud2` is checked for rate, lag and emptiness, not for point quality.

## Status

Version 0.1.0. 11 unit tests of the checks, plus end-to-end runs:

- **Fake camera:** healthy, dark, frozen, no `camera_info`, silent each gave the expected finding.
- **UR3 Gazebo sim** (head and wrist cameras, depth, point clouds, 7 streams): reads healthy. Pausing the sim made all 7 streams `STREAM_STALE` with a growing silence time, and they returned to healthy after the resume.
- That run found a false positive that is now fixed: noiseless simulated cameras on a still scene deliver bit-identical frames (81 frames, 1 distinct image) with advancing stamps. That is `IMAGE_STATIC`, a warning; only a repeating stamp is `IMAGE_FROZEN`.

Not tested: a real camera or depth sensor on hardware, and ROS 2 Jazzy locally (CI builds it). Run it on your own sensors and report what it gets wrong.

Build: `colcon build --packages-select percept_doctor`. Tests: `python3 -m pytest test` (no ROS needed). Try it without a robot: `python3 test/fake_camera.py [--dark|--frozen|--no-info|--stop-after N]`.
