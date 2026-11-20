"""Perception health checks that need no ROS: feed it stream events, ask for findings."""
from collections import deque
from dataclasses import dataclass, field

import numpy as np

ERROR, WARN, OK = 'ERROR', 'WARN', 'OK'
SIM_STAMP_GAP_S = 1e4    # a stamp-vs-clock gap this large means sim time vs wall time, not lag


@dataclass
class Config:
    grace: float = 10.0          # s after start before a never-seen stream is reported
    stale: float = 2.0           # s of silence before a stream is stale
    min_rate: float = 5.0        # Hz below which a live stream is slow
    dark_mean: float = 20.0      # mean intensity (0-255) below which an image is dark
    saturated_frac: float = 0.5  # fraction of pixels >= 250 above which an image is blown out
    flat_std: float = 2.0        # intensity std below which an image has no content
    frozen_frames: int = 10      # identical consecutive frames before the camera counts as frozen
    depth_valid_min: float = 0.2  # valid-depth fraction below which depth is poor
    depth_dead: float = 0.01     # valid-depth fraction below which depth is dead
    lag_max: float = 0.5         # s of header-stamp lag before STREAM_LAG
    window: float = 5.0          # s of history the checks look at


@dataclass
class Stream:
    name: str
    kind: str                                   # image | depth | cloud
    first: float = None
    last: float = None
    times: deque = field(default_factory=lambda: deque(maxlen=300))
    lags: deque = field(default_factory=lambda: deque(maxlen=300))     # (t, lag s)
    stats: deque = field(default_factory=lambda: deque(maxlen=300))    # (t, dict)
    sigs: deque = field(default_factory=lambda: deque(maxlen=64))      # frame signatures
    hstamps: deque = field(default_factory=lambda: deque(maxlen=64))   # header stamps (s), as published


@dataclass
class State:
    t0: float = 0.0
    streams: dict = field(default_factory=dict)       # topic -> Stream
    advertised: dict = field(default_factory=dict)    # topic -> kind, seen on the graph (may never deliver)
    info_topics: set = field(default_factory=set)     # camera_info topics seen (published or received)
    sim_time: bool = False                            # a simulator drives time (/clock is live, or stamps are sim time)


@dataclass
class Finding:
    level: str
    code: str
    msg: str


def sensor_kind(msg_type):
    return {'sensor_msgs/msg/Image': 'image', 'sensor_msgs/msg/PointCloud2': 'cloud'}.get(msg_type)


def depth_like(encoding):
    return encoding in ('16UC1', '32FC1', 'mono16')


def image_stats(data, height, width, step, encoding):
    """Cheap statistics from a raw image buffer (a ~48 x 64 subsample). None when the encoding is unsupported."""
    enc = encoding.lower()
    ch = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1, '8uc1': 1, 'mono16': 1, '16uc1': 1, '32fc1': 1}.get(enc)
    if ch is None or height == 0 or width == 0:
        return None
    dtype = {'mono16': np.uint16, '16uc1': np.uint16, '32fc1': np.float32}.get(enc, np.uint8)
    arr = np.frombuffer(bytes(data), dtype=dtype)
    row = step // arr.itemsize
    if arr.size < row * height or row < width * ch:
        return None
    a = arr[:row * height].reshape(height, row)[:, :width * ch].reshape(height, width, ch)
    a = a[::max(1, height // 48), ::max(1, width // 64), :]
    sig = hash(a.tobytes())
    if dtype is np.uint8:
        a = a.astype(np.float32)
        return {'mean': float(a.mean()), 'std': float(a.std()), 'sat': float((a >= 250).mean()), 'sig': sig}
    a = a.astype(np.float32)
    valid = np.isfinite(a) & (a > 0)
    return {'valid': float(valid.mean()), 'sig': sig}


def feed(s, topic, kind, now, stamp=None, stats=None, points=None, header_stamp=None, sim=False):
    st = s.streams.setdefault(topic, Stream(topic, kind))
    if sim:
        s.sim_time = True
    st.first = now if st.first is None else st.first
    st.last = now
    st.times.append(now)
    if header_stamp is not None:
        st.hstamps.append(header_stamp)
    if stamp is not None and stamp > 0:
        lag = now - stamp
        if abs(lag) < SIM_STAMP_GAP_S:
            st.lags.append((now, lag))
    if stats is not None:
        st.stats.append((now, stats))
        if 'sig' in stats:
            st.sigs.append(stats['sig'])
    if points is not None:
        st.stats.append((now, {'points': points}))
    return st


def rate(st, now, window):
    ts = [t for t in st.times if t >= now - window]
    return (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) >= 3 and ts[-1] > ts[0] else None


def evaluate(s, now, c=Config()):
    f = []
    late = now - s.t0 > c.grace
    live = [x for x in s.streams.values() if x.last is not None]
    if not s.streams and not s.advertised:
        if late:
            f.append(Finding(WARN, 'NO_SENSORS', 'no camera, depth or point cloud topics on the graph '
                             '(wrong domain or namespace, or the driver is not running)'))
        return f or [Finding(OK, 'HEALTHY', 'waiting for sensor streams')]
    for topic, kind in s.advertised.items():
        if topic not in s.streams and late:
            f.append(Finding(ERROR, 'STREAM_NONE', f'{kind} topic {topic} is advertised but no message ever arrived '
                             '(driver up but not publishing, or a QoS mismatch)'))
    for st in live:
        age = now - st.last
        if age > c.stale:
            f.append(Finding(ERROR, 'STREAM_STALE', f'{st.kind} stream {st.name} silent for {age:.1f}s'))
            continue
        r = rate(st, now, c.window)
        if r is not None and r < c.min_rate:
            f.append(Finding(WARN, 'LOW_RATE', f'{st.kind} stream {st.name} runs at {r:.1f} Hz (below {c.min_rate:g})'))
        lags = sorted(l for t, l in st.lags if t >= now - c.window)
        if len(lags) >= 3 and lags[len(lags) // 2] > c.lag_max:
            f.append(Finding(WARN, 'STREAM_LAG', f'{st.name} arrives {lags[len(lags) // 2]:.2f}s after its header stamp'))
        recent = [d for t, d in st.stats if t >= now - c.window]
        if st.kind == 'image':
            m = [d for d in recent if 'mean' in d]
            if m:
                mean = float(np.mean([d['mean'] for d in m]))
                std = float(np.mean([d['std'] for d in m]))
                sat = float(np.mean([d['sat'] for d in m]))
                if std < c.flat_std:
                    f.append(Finding(WARN, 'IMAGE_FLAT', f'{st.name} has almost no contrast (std {std:.1f}): covered lens or a blank image'))
                elif mean < c.dark_mean:
                    f.append(Finding(WARN, 'IMAGE_DARK', f'{st.name} is very dark (mean {mean:.0f}/255)'))
                if sat > c.saturated_frac:
                    f.append(Finding(WARN, 'IMAGE_SATURATED', f'{st.name} is blown out ({sat:.0%} of pixels saturated)'))
            d16 = [d for d in recent if 'valid' in d]
            if d16:
                v = float(np.mean([d['valid'] for d in d16]))
                if v < c.depth_dead:
                    f.append(Finding(ERROR, 'DEPTH_DEAD', f'{st.name} has almost no valid depth ({v:.1%}): sensor blocked, too close, or a driver fault'))
                elif v < c.depth_valid_min:
                    f.append(Finding(WARN, 'DEPTH_POOR', f'{st.name} has only {v:.0%} valid depth pixels'))
            if len(st.sigs) >= c.frozen_frames and len(set(list(st.sigs)[-c.frozen_frames:])) == 1:
                stamps = list(st.hstamps)[-c.frozen_frames:]
                if len(stamps) >= c.frozen_frames and len(set(stamps)) == 1:
                    f.append(Finding(ERROR, 'IMAGE_FROZEN', f'{st.name} delivers identical frames with an identical stamp '
                                     f'({c.frozen_frames}+ in a row): the camera driver is stuck'))
                elif s.sim_time:
                    f.append(Finding(WARN, 'IMAGE_STATIC', f'{st.name} delivers identical frames while its stamps advance: '
                                     'normal for a still scene on a noiseless simulated camera'))
                else:
                    f.append(Finding(ERROR, 'IMAGE_FROZEN', f'{st.name} delivers identical frames ({c.frozen_frames}+ in a row) '
                                     'while its stamps keep advancing: the pixels are frozen but the driver keeps stamping '
                                     '(a real sensor always has noise)'))
        elif st.kind == 'cloud':
            pts = [d['points'] for d in recent if 'points' in d]
            if pts and max(pts) == 0:
                f.append(Finding(ERROR, 'CLOUD_EMPTY', f'{st.name} publishes empty point clouds'))
    imgs = [x.name for x in s.streams.values() if x.kind == 'image']
    if imgs and not s.info_topics and late:
        f.append(Finding(WARN, 'NO_CAMERA_INFO', 'image streams are live but no camera_info was seen: projection, '
                         'depth-to-cloud and detectors need the intrinsics'))
    if not any(x.level == ERROR for x in f):
        f.append(Finding(OK, 'HEALTHY', f'{len(live)} sensor stream(s) look healthy'))
    return f


HINTS = {
    'NO_SENSORS': 'check ROS_DOMAIN_ID and the namespace; `ros2 topic list -t | grep -E "Image|PointCloud2"`',
    'STREAM_NONE': 'the topic exists but nothing arrives: check the driver log and that your QoS is best_effort for sensor data',
    'STREAM_STALE': 'driver or bridge stopped: restart it, check USB/bandwidth, and in sim check the bridge is not paused',
    'LOW_RATE': 'reduce resolution or fps load, check USB bandwidth and CPU, and use best_effort QoS',
    'STREAM_LAG': 'stamps trail the clock: check time sync and the driver\'s timestamping (use_sim_time on every node in sim)',
    'IMAGE_FLAT': 'remove the lens cap, check exposure and that the camera is not facing a blank surface',
    'IMAGE_DARK': 'raise exposure/gain or add light; check auto-exposure is on',
    'IMAGE_SATURATED': 'lower exposure/gain or avoid direct light into the lens',
    'IMAGE_STATIC': 'normal in simulation for a still scene; if this is not a simulator, check that /clock is not being published by mistake',
    'IMAGE_FROZEN': 'restart the camera driver; a stuck pipeline keeps republishing the last frame, often with fresh stamps (seen on ZED X)',
    'DEPTH_POOR': 'check the working range, reflective or transparent surfaces and the IR projector',
    'DEPTH_DEAD': 'check the sensor is not blocked or too close, and the depth stream is enabled in the driver',
    'CLOUD_EMPTY': 'check the depth source and the cloud filter limits (range, voxel size, crop box)',
    'NO_CAMERA_INFO': 'publish camera_info next to the image (same namespace); detectors and depth projection need it',
}
