#!/usr/bin/env python3
"""Fake camera for trying percept_doctor: healthy by default, or --dark, --frozen, --stop-after SECONDS, --no-info."""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dark', action='store_true')
    ap.add_argument('--frozen', action='store_true')
    ap.add_argument('--no-info', action='store_true')
    ap.add_argument('--sim', action='store_true', help='drive time like a simulator: publish /clock and sim-time stamps')
    ap.add_argument('--stop-after', type=float, default=0.0)
    ap.add_argument('--hz', type=float, default=15.0)
    a = ap.parse_args()
    rclpy.init()
    n = Node('fake_camera')
    pub = n.create_publisher(Image, '/cam/image_raw', qos_profile_sensor_data)
    info = None if a.no_info else n.create_publisher(CameraInfo, '/cam/camera_info', qos_profile_sensor_data)
    clock = n.create_publisher(Clock, '/clock', 10) if a.sim else None
    rng = np.random.default_rng(0)
    t0 = time.time()
    frame = rng.normal(8 if a.dark else 120, 3 if a.dark else 40, (96, 128, 3)).clip(0, 255).astype(np.uint8)
    while rclpy.ok():
        if a.stop_after and time.time() - t0 > a.stop_after:
            time.sleep(0.2)
            continue
        if not a.frozen:
            frame = rng.normal(8 if a.dark else 120, 3 if a.dark else 40, (96, 128, 3)).clip(0, 255).astype(np.uint8)
        m = Image(height=96, width=128, encoding='rgb8', step=128 * 3, data=frame.tobytes())
        if a.sim:
            sim_t = time.time() - t0 + 5.0
            m.header.stamp.sec, m.header.stamp.nanosec = int(sim_t), int((sim_t % 1) * 1e9)
            clock.publish(Clock(clock=m.header.stamp))
        else:
            m.header.stamp = n.get_clock().now().to_msg()
        m.header.frame_id = 'cam'
        pub.publish(m)
        if info is not None:
            ci = CameraInfo(height=96, width=128)
            ci.header = m.header
            info.publish(ci)
        time.sleep(1.0 / a.hz)


if __name__ == '__main__':
    main()
