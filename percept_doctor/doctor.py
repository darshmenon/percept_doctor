#!/usr/bin/env python3
"""percept_doctor - attach to a running ROS 2 robot and report camera, depth and point cloud health. Read-only."""
import argparse
import json
import time

from percept_doctor import __version__
from percept_doctor.core import Config, HINTS, State, evaluate, feed, image_stats, sensor_kind

MAX_TOPICS = 8
SKIP = ('compressed', 'theora', 'compressedDepth', 'zstd')


def fmt(findings, hints=False):
    out = []
    for x in findings:
        out.append(f'{x.level:<5} [{x.code}] {x.msg}')
        if hints and HINTS.get(x.code):
            out.append(f'      -> {HINTS[x.code]}')
    return '\n'.join(out)


def to_json(findings, ns, now_wall):
    return json.dumps({'time': now_wall, 'ns': ns, 'ok': not any(x.level == 'ERROR' for x in findings),
                       'findings': [dict(x.__dict__, hint=HINTS.get(x.code)) for x in findings]})


def main(argv=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, PointCloud2

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--version', action='version', version=f'percept_doctor {__version__}')
    ap.add_argument('--ns', default='', help='only sensor topics under this namespace, e.g. robot1 or /camera')
    ap.add_argument('--period', type=float, default=2.0, help='report period (s)')
    ap.add_argument('--grace', type=float, default=Config.grace, help='seconds before a silent stream is an error')
    ap.add_argument('--stale', type=float, default=Config.stale, help='seconds of silence before a stream is stale')
    ap.add_argument('--min-rate', type=float, default=Config.min_rate, help='Hz below which a stream is slow')
    ap.add_argument('--json', action='store_true', help='print one JSON object per report')
    ap.add_argument('--hints', action='store_true', help='print a suggested next step under each finding')
    ap.add_argument('--learn', type=float, default=0.0, metavar='SECS',
                    help='watch healthy sensors for SECS, write each stream\'s normal rate to --baseline-file, exit')
    ap.add_argument('--baseline-file', default='percept_baseline.json', help='where --learn writes, and --baseline reads')
    ap.add_argument('--expect', action='append', default=[], metavar='STREAM:HZ',
                    help='expected rate, e.g. --expect color:30 (repeatable); no --learn needed, overrides a learned value')
    ap.add_argument('--baseline', action='store_true', help='also flag streams far slower or faster than the learned baseline')
    ap.add_argument('--once', type=float, default=0.0, help='watch N seconds, print one report, exit 1 on any ERROR')
    args, ros_args = ap.parse_known_args(argv)
    if args.learn:
        args.once = args.learn

    rclpy.init(args=ros_args)
    node = Node('percept_doctor')
    cfg = Config(grace=args.grace, stale=args.stale, min_rate=args.min_rate)
    now = time.monotonic
    s = State(t0=now())
    if args.baseline:
        from percept_doctor import baseline as bl
        try:
            s.baseline = bl.load(args.baseline_file)
        except ValueError as e:
            ap.error(str(e))
    if args.expect:
        from percept_doctor import baseline as bl
        try:
            expected = bl.parse_expect(args.expect)
        except ValueError as e:
            ap.error(str(e))
        s.baseline = {**(s.baseline or {}), **expected}
        print('[percept_doctor] expecting rates: ' + ', '.join(f'{k}={v["hz"]:g} Hz' for k, v in sorted(expected.items())))
    prefix = ('/' + args.ns.strip('/')) if args.ns else ''
    subscribed = {}
    clock_seen = {'t': None}

    def clock_live():
        return clock_seen['t'] is not None and now() - clock_seen['t'] < 2.0

    node.create_subscription(Clock, '/clock', lambda m: clock_seen.update(t=now()), qos_profile_sensor_data)

    def stamp_of(header):
        """(stamp for the lag check or None, True when the stamp is sim time)."""
        st = header.stamp.sec + header.stamp.nanosec / 1e9
        if st <= 0:
            return None, False
        lag = node.get_clock().now().nanoseconds / 1e9 - st
        return (now() - lag, False) if abs(lag) < 1e4 else (None, True)    # sim-time stamps: no usable lag

    def on_image(topic):
        def cb(m):
            stats = image_stats(m.data, m.height, m.width, m.step, m.encoding)
            stamp, sim = stamp_of(m.header)
            feed(s, topic, 'image', now(), stamp=stamp, stats=stats, sim=sim or clock_live(),
                 header_stamp=m.header.stamp.sec + m.header.stamp.nanosec / 1e9)
        return cb

    def on_cloud(topic):
        def cb(m):
            stamp, sim = stamp_of(m.header)
            feed(s, topic, 'cloud', now(), stamp=stamp, points=m.width * m.height, sim=sim or clock_live())
        return cb

    def discover():
        for topic, types in node.get_topic_names_and_types():
            if prefix and not topic.startswith(prefix):
                continue
            if any(topic.endswith(k) or f'/{k}' in topic for k in SKIP):
                continue
            if 'sensor_msgs/msg/CameraInfo' in types and node.get_publishers_info_by_topic(topic):
                s.info_topics.add(topic)
            for t in types:
                kind = sensor_kind(t)
                if kind is None or not node.get_publishers_info_by_topic(topic):
                    continue
                s.advertised.setdefault(topic, kind)
                if topic not in subscribed and len(subscribed) < MAX_TOPICS:
                    if kind == 'image':
                        subscribed[topic] = node.create_subscription(Image, topic, on_image(topic), qos_profile_sensor_data)
                    else:
                        subscribed[topic] = node.create_subscription(PointCloud2, topic, on_cloud(topic), qos_profile_sensor_data)

    node.create_timer(1.0, discover)
    exit_code = {'v': 0}

    def report():
        found = evaluate(s, now(), cfg)
        print(to_json(found, prefix, time.time()) if args.json else
              f'\n--- percept_doctor {time.strftime("%H:%M:%S")} ns="{prefix}" ---\n{fmt(found, args.hints)}', flush=True)
        return found

    def tick():
        if args.learn:
            from percept_doctor import baseline as bl
            learned = bl.learn({k: v.times for k, v in s.streams.items()})
            bl.save(learned, args.baseline_file)
            print(f'learned {len(learned)} streams -> {args.baseline_file}: ' +
                  ', '.join(f'{k} {v["hz"]:g} Hz' for k, v in sorted(learned.items())), flush=True)
            raise SystemExit(0)
        found = report()
        if args.once:
            exit_code['v'] = 1 if any(x.level == 'ERROR' for x in found) else 0
            raise SystemExit(exit_code['v'])

    node.create_timer(args.once if args.once else args.period, tick)
    node.get_logger().info(f'watching {prefix or "/"} for image, depth and point cloud topics - Ctrl+C to stop')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code['v']


if __name__ == '__main__':
    raise SystemExit(main())
