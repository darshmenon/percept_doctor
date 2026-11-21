"""Run the real CLI against a fake camera process (no sim). Needs a sourced ROS 2 environment."""
import json
import os
import subprocess
import sys
import time

import pytest

pytest.importorskip('rclpy')
ROOT = os.path.join(os.path.dirname(__file__), '..')
CAM = os.path.join(os.path.dirname(__file__), 'fake_camera.py')
_domain = [160]


def run(cam_args, watch=7.0, extra=()):
    """Start the fake camera on a private domain, run the doctor with --once, return (exit code, {code: level})."""
    _domain[0] += 1
    env = dict(os.environ, ROS_DOMAIN_ID=str(_domain[0]), ROS_LOCALHOST_ONLY='1')
    cam = subprocess.Popen([sys.executable, CAM, *cam_args], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(1.5)
        p = subprocess.run([sys.executable, '-m', 'percept_doctor.doctor', '--once', str(watch), '--json', *extra],
                           env=env, cwd=ROOT, capture_output=True, text=True, timeout=60)
    finally:
        cam.terminate()
        cam.wait(timeout=10)
    lines = [json.loads(x) for x in p.stdout.splitlines() if x.startswith('{')]
    assert lines, p.stdout + p.stderr
    return p.returncode, {f['code']: f['level'] for f in lines[-1]['findings']}


def test_healthy_camera_exits_zero():
    rc, found = run([])
    assert rc == 0 and not [c for c, lv in found.items() if lv == 'ERROR'], found


def test_dark_camera_is_named():
    rc, found = run(['--dark'])
    assert 'IMAGE_DARK' in found


def test_frozen_pixels_on_a_real_clock_is_an_error():
    rc, found = run(['--frozen'], watch=9.0)
    assert found.get('IMAGE_FROZEN') == 'ERROR' and rc == 1


def test_frozen_pixels_under_sim_time_is_only_a_warning():
    rc, found = run(['--frozen', '--sim'], watch=9.0)
    assert found.get('IMAGE_STATIC') == 'WARN' and 'IMAGE_FROZEN' not in found


def test_camera_that_stops_is_reported_stale():
    rc, found = run(['--stop-after', '5'], watch=10.0)
    assert 'STREAM_STALE' in found and rc == 1


def test_missing_camera_info_is_a_warning():
    rc, found = run(['--no-info'], extra=['--grace', '2'])
    assert found.get('NO_CAMERA_INFO') == 'WARN'


def test_mostly_invalid_depth_is_flagged():
    rc, found = run(['--depth'])
    assert 'DEPTH_POOR' in found or 'DEPTH_DEAD' in found, found


def test_namespace_filter_ignores_other_topics():
    rc, found = run([], extra=['--ns', 'nothing_here', '--grace', '2'])
    assert 'NO_SENSORS' in found
