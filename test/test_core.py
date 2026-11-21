import numpy as np

from percept_doctor.core import Config, HINTS, State, evaluate, feed, image_stats


def codes(f):
    return {x.code for x in f}


def rgb(h=96, w=128, value=None, noise=40.0, seed=0):
    rng = np.random.default_rng(seed)
    a = rng.normal(value if value is not None else 120, noise, (h, w, 3)).clip(0, 255).astype(np.uint8)
    return a.tobytes(), h, w, w * 3, 'rgb8'


def live_image(s, t_end, hz=10, topic='/cam/image_raw', make=rgb, vary=True, fixed_stamp=None):
    t = 0.0
    i = 0
    while t <= t_end:
        data, h, w, step, enc = make(seed=i if vary else 0)
        feed(s, topic, 'image', t, stamp=t, stats=image_stats(data, h, w, step, enc),
             header_stamp=fixed_stamp if fixed_stamp is not None else t)
        t += 1.0 / hz
        i += 1
    return t - 1.0 / hz


def test_healthy_camera_with_info_is_healthy():
    s = State(t0=0.0)
    s.info_topics.add('/cam/camera_info')
    now = live_image(s, 6.0)
    out = evaluate(s, now, Config())
    assert codes(out) == {'HEALTHY'}


def test_no_sensors_after_grace_only():
    s = State(t0=0.0)
    assert 'NO_SENSORS' not in codes(evaluate(s, 1.0))
    assert 'NO_SENSORS' in codes(evaluate(s, 30.0))


def test_advertised_but_silent_stream_is_an_error_after_grace():
    s = State(t0=0.0)
    s.advertised['/cam/image_raw'] = 'image'
    assert 'STREAM_NONE' not in codes(evaluate(s, 2.0))
    assert 'STREAM_NONE' in codes(evaluate(s, 30.0))


def test_stale_stream_and_low_rate():
    s = State(t0=0.0)
    s.info_topics.add('i')
    now = live_image(s, 4.0)
    assert 'STREAM_STALE' in codes(evaluate(s, now + 5.0))
    slow = State(t0=0.0)
    slow.info_topics.add('i')
    now = live_image(slow, 8.0, hz=2)
    assert 'LOW_RATE' in codes(evaluate(slow, now))


def test_dark_flat_and_saturated_images():
    for make, code in ((lambda seed: rgb(value=5, noise=3, seed=seed), 'IMAGE_DARK'),
                       (lambda seed: rgb(value=128, noise=0.0, seed=seed), 'IMAGE_FLAT'),
                       (lambda seed: rgb(value=254, noise=2.0, seed=seed), 'IMAGE_SATURATED')):
        s = State(t0=0.0)
        s.info_topics.add('i')
        now = live_image(s, 5.0, make=make)
        assert code in codes(evaluate(s, now)), code


def test_frozen_camera_repeats_identical_frames_with_a_repeating_stamp():
    s = State(t0=0.0)
    s.info_topics.add('i')
    now = live_image(s, 4.0, vary=False, fixed_stamp=3.0)     # same frame, same stamp every time
    got = codes(evaluate(s, now))
    assert 'IMAGE_FROZEN' in got and 'IMAGE_STATIC' not in got
    ok = State(t0=0.0)
    ok.info_topics.add('i')
    now = live_image(ok, 4.0, vary=True)
    assert 'IMAGE_FROZEN' not in codes(evaluate(ok, now))


def depth_msg(valid_frac, h=60, w=80, seed=0):
    rng = np.random.default_rng(seed)
    a = np.where(rng.random((h, w)) < valid_frac, 1500, 0).astype(np.uint16)
    return a.tobytes(), h, w, w * 2, '16UC1'


def test_depth_validity_levels():
    for frac, expect in ((0.9, None), (0.1, 'DEPTH_POOR'), (0.0, 'DEPTH_DEAD')):
        s = State(t0=0.0)
        s.info_topics.add('i')
        now = live_image(s, 5.0, topic='/cam/depth', make=lambda seed, f=frac: depth_msg(f, seed=seed))
        got = codes(evaluate(s, now))
        if expect:
            assert expect in got
        else:
            assert not ({'DEPTH_POOR', 'DEPTH_DEAD'} & got)


def test_empty_cloud_and_missing_camera_info_and_lag():
    s = State(t0=0.0)
    t = 0.0
    while t <= 5.0:
        feed(s, '/pc', 'cloud', t, stamp=t, points=0)
        t += 0.1
    assert 'CLOUD_EMPTY' in codes(evaluate(s, 5.0))
    s = State(t0=0.0)
    now = live_image(s, 12.0)
    assert 'NO_CAMERA_INFO' in codes(evaluate(s, now))
    s = State(t0=0.0)
    s.info_topics.add('i')
    t = 0.0
    while t <= 5.0:
        data, h, w, step, enc = rgb(seed=int(t * 10))
        feed(s, '/cam/image_raw', 'image', t, stamp=t - 1.2, stats=image_stats(data, h, w, step, enc))
        t += 0.1
    assert 'STREAM_LAG' in codes(evaluate(s, 5.0))


def test_sim_time_stamps_are_not_lag():
    s = State(t0=0.0)
    s.info_topics.add('i')
    t = 1.8e9
    for i in range(60):
        data, h, w, step, enc = rgb(seed=i)
        feed(s, '/cam/image_raw', 'image', t, stamp=12.0 + i * 0.1, stats=image_stats(data, h, w, step, enc))
        t += 0.1
    assert 'STREAM_LAG' not in codes(evaluate(s, t))


def test_unsupported_encoding_is_ignored_and_every_code_has_a_hint():
    assert image_stats(b'\x00' * 10, 2, 2, 4, 'bayer_rggb8') is None
    s = State(t0=0.0)
    s.info_topics.add('i')
    now = live_image(s, 3.0)
    produced = {'NO_SENSORS', 'STREAM_NONE', 'STREAM_STALE', 'LOW_RATE', 'STREAM_LAG', 'IMAGE_FLAT', 'IMAGE_DARK',
                'IMAGE_SATURATED', 'IMAGE_FROZEN', 'DEPTH_POOR', 'DEPTH_DEAD', 'CLOUD_EMPTY', 'NO_CAMERA_INFO'}
    assert produced <= set(HINTS)


def test_identical_frames_with_advancing_stamps_in_sim_are_static_not_frozen():
    """Found on a UR3 Gazebo sim: noiseless cameras on a still scene deliver bit-identical frames (81 frames, 1 hash)
    with advancing stamps. In a simulator that is a warning."""
    s = State(t0=0.0)
    s.info_topics.add('i')
    s.sim_time = True
    now = live_image(s, 4.0, vary=False)
    got = codes(evaluate(s, now))
    assert 'IMAGE_STATIC' in got and 'IMAGE_FROZEN' not in got
    assert 'IMAGE_STATIC' in HINTS


def test_identical_frames_with_advancing_stamps_on_a_real_clock_are_frozen():
    """A real ZED X freeze keeps advancing header stamps and camera_info while the pixels repeat. Real sensors have
    noise, so identical frames on a real clock are a stuck pipeline."""
    s = State(t0=0.0)
    s.info_topics.add('i')
    now = live_image(s, 4.0, vary=False)
    got = codes(evaluate(s, now))
    assert 'IMAGE_FROZEN' in got and 'IMAGE_STATIC' not in got


def test_noisy_frames_are_never_frozen_or_static_in_either_clock_mode():
    for sim in (False, True):
        s = State(t0=0.0)
        s.info_topics.add('i')
        s.sim_time = sim
        now = live_image(s, 4.0, vary=True)
        assert not ({'IMAGE_FROZEN', 'IMAGE_STATIC'} & codes(evaluate(s, now)))


def test_feed_marks_sim_time():
    s = State(t0=0.0)
    feed(s, '/a', 'image', 1.0, sim=True)
    assert s.sim_time is True


# ---- image_stats: encodings, padding, bad buffers -------------------------------------------------

def test_image_stats_supported_encodings():
    for enc, ch, dtype in [('rgb8', 3, np.uint8), ('bgr8', 3, np.uint8), ('rgba8', 4, np.uint8), ('bgra8', 4, np.uint8),
                           ('mono8', 1, np.uint8), ('8UC1', 1, np.uint8)]:
        a = np.full((20, 30, ch), 100, dtype)
        st = image_stats(a.tobytes(), 20, 30, 30 * ch, enc)
        assert st and abs(st['mean'] - 100) < 1e-3 and st['std'] < 1e-3, enc


def test_image_stats_ignores_row_padding():
    a = np.full((10, 16), 50, np.uint8)
    padded = np.pad(a, ((0, 0), (0, 8)), constant_values=255)          # 8 junk bytes per row
    st = image_stats(padded.tobytes(), 10, 16, 24, 'mono8')
    assert abs(st['mean'] - 50) < 1e-3 and st['sat'] == 0.0


def test_image_stats_depth_validity_for_16bit_and_float():
    d16 = np.zeros((10, 10), np.uint16); d16[:, :5] = 1000
    assert abs(image_stats(d16.tobytes(), 10, 10, 20, '16UC1')['valid'] - 0.5) < 1e-6
    d32 = np.full((10, 10), np.nan, np.float32); d32[:3] = 1.5
    assert abs(image_stats(d32.tobytes(), 10, 10, 40, '32FC1')['valid'] - 0.3) < 1e-6   # NaN counts as invalid
    inf = np.full((4, 4), np.inf, np.float32)
    assert image_stats(inf.tobytes(), 4, 4, 16, '32FC1')['valid'] == 0.0


def test_image_stats_rejects_what_it_cannot_read():
    assert image_stats(b'\x00' * 10, 10, 10, 10, 'bayer_rggb8') is None        # unsupported encoding
    assert image_stats(b'', 0, 10, 10, 'mono8') is None                          # zero height
    assert image_stats(b'', 10, 0, 10, 'mono8') is None                          # zero width
    assert image_stats(b'\x00' * 50, 10, 10, 10, 'mono8') is None                # buffer shorter than height * step
    assert image_stats(b'\x00' * 100, 10, 10, 5, 'mono8') is None                # step narrower than the row


def test_image_stats_same_pixels_same_signature_and_one_pixel_changes_it():
    a = np.random.default_rng(1).integers(0, 255, (48, 64), dtype=np.uint8)
    b = a.copy(); b[0, 0] ^= 1
    s1 = image_stats(a.tobytes(), 48, 64, 64, 'mono8')['sig']
    assert s1 == image_stats(a.tobytes(), 48, 64, 64, 'mono8')['sig']
    assert s1 != image_stats(b.tobytes(), 48, 64, 64, 'mono8')['sig']


def test_sensor_kind_and_depth_like():
    from percept_doctor.core import depth_like, sensor_kind
    assert sensor_kind('sensor_msgs/msg/Image') == 'image'
    assert sensor_kind('sensor_msgs/msg/PointCloud2') == 'cloud'
    assert sensor_kind('sensor_msgs/msg/LaserScan') is None
    assert depth_like('16UC1') and depth_like('32FC1') and depth_like('mono16') and not depth_like('rgb8')


# ---- output formatting (no ROS needed) -----------------------------------------------------------

def test_fmt_and_json_output_shape():
    import json
    from percept_doctor.core import Finding
    from percept_doctor.doctor import fmt, to_json
    f = [Finding('ERROR', 'STREAM_STALE', '/cam silent'), Finding('WARN', 'NO_CAMERA_INFO', 'no info')]
    text = fmt(f, hints=True)
    assert 'ERROR [STREAM_STALE] /cam silent' in text and '->' in text
    assert '->' not in fmt(f, hints=False)
    j = json.loads(to_json(f, '/cam', 123.0))
    assert j['ok'] is False and j['ns'] == '/cam' and {x['code'] for x in j['findings']} == {'STREAM_STALE', 'NO_CAMERA_INFO'}
    assert all(x['hint'] for x in j['findings'])
    assert json.loads(to_json([Finding('WARN', 'NO_CAMERA_INFO', 'x')], '', 1.0))['ok'] is True


# ---- timing edge cases ----------------------------------------------------------------------------

def test_stream_that_stops_mid_run_goes_stale_then_recovers():
    s = State(t0=0.0)
    end = live_image(s, 5.0)
    assert 'STREAM_STALE' not in codes(evaluate(s, end + 0.5))
    assert 'STREAM_STALE' in codes(evaluate(s, end + 10.0))
    for i in range(20):
        t = end + 10.0 + i * 0.1
        feed(s, '/cam/image_raw', 'image', t, stamp=t, stats=image_stats(*rgb(seed=100 + i)), header_stamp=t)
    assert 'STREAM_STALE' not in codes(evaluate(s, end + 12.0))


def test_every_finding_code_has_a_hint_and_a_known_level():
    from percept_doctor.core import HINTS
    src = open(__import__('percept_doctor.core', fromlist=['x']).__file__).read()
    import re
    for code in set(re.findall(r"'([A-Z]+(?:_[A-Z]+)+)'", src)):
        if code in ('CLOUD_EMPTY', 'DEPTH_DEAD', 'DEPTH_POOR', 'IMAGE_DARK', 'IMAGE_FLAT', 'IMAGE_FROZEN', 'IMAGE_SATURATED',
                    'IMAGE_STATIC', 'LOW_RATE', 'NO_CAMERA_INFO', 'NO_SENSORS', 'STREAM_LAG', 'STREAM_NONE', 'STREAM_STALE'):
            assert HINTS.get(code), code
