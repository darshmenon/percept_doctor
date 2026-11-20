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
