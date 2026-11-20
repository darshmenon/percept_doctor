#!/usr/bin/env python3
"""Publish a local webcam (OpenCV) as /cam/image_raw so percept_doctor can be tried on a real sensor.
Frames are only published to ROS, never written to disk. Usage: webcam_bridge.py [--device 0] [--seconds 30]"""
import argparse
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', type=int, default=0)
    ap.add_argument('--seconds', type=float, default=30.0)
    a = ap.parse_args()
    cap = cv2.VideoCapture(a.device)
    if not cap.isOpened():
        raise SystemExit(f'cannot open /dev/video{a.device}')
    rclpy.init()
    n = Node('webcam_bridge')
    pub = n.create_publisher(Image, '/cam/image_raw', qos_profile_sensor_data)
    info = n.create_publisher(CameraInfo, '/cam/camera_info', qos_profile_sensor_data)
    t0, frames = time.time(), 0
    while rclpy.ok() and time.time() - t0 < a.seconds:
        ok, bgr = cap.read()
        if not ok:
            continue
        m = Image(height=bgr.shape[0], width=bgr.shape[1], encoding='bgr8', step=bgr.shape[1] * 3, data=bgr.tobytes())
        m.header.stamp = n.get_clock().now().to_msg()
        m.header.frame_id = 'webcam'
        pub.publish(m)
        ci = CameraInfo(height=m.height, width=m.width)
        ci.header = m.header
        info.publish(ci)
        frames += 1
    cap.release()
    print(f'published {frames} frames in {time.time() - t0:.0f}s')


if __name__ == '__main__':
    main()
