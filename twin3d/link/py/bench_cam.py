"""bench_cam.py: camera stand-in for BENCH robots (bare ESP32 boards running the firmware bench build, -DBENCH_SIM=1:
real tasks / WiFi / MQTT / control, motors and encoders simulated on the chip). Used by run_real('bench').
No OpenCV, no camera, no corner markers. Contract: link/TOPICS.md ("Bench mode").

A bench robot has no floor under it: its own estimate IS its truth. So the "camera" is the robot's own telemetry
pose, republished as zadp/cam/pose in exactly the format aruco_node.py publishes (same pose_message(): bot, x, y,
heading, t, age), per robot, at most P.cam.rateHz, only when a NEW telemetry message arrived (a repeat would carry
the same image time and be dropped by the robot as a duplicate; telemetry is 20 Hz, the camera 30 Hz).
  t   = Unix time of the telemetry on this PC minus P.link.netDelay (the assumed robot -> PC transit)
  age = publish time - t, measured here (no clock sync), the robot adds P.link.netDelay itself: lag = age + netDelay
A robot whose telemetry is older than P.cam.maxAge, or whose state is "offline", is "not seen": nothing is published.

Start pose (one rule): a bench board keeps its odometry pose from earlier commands, and an echo can never move it.
run_real requires every robot on its parking slot, so before the echo starts this node plays the person who puts
the robot on its slot: it publishes the SLOT pose (--slot N=x,y,heading, from the brain's layout) as camera fixes
until the robot's own telemetry reports that pose. The firmware needs no change: a fix further than 50 mm / 20 deg
from the estimate is rejected until 10 in a row agree (the "robot moved by hand" path, TOPICS.md Fusion), then the
estimate is replaced; a nearer fix is blended in with the fusion gain. Only after every robot reports the slot (within
a tenth of P.link.parkTol / parkTolDeg, 5 telemetry messages in a row) the log says "bench ready" (run_real waits for
that word) and the echo starts. Not reached within P.link.readyTimeout: "ERROR ..." in the log, exit code 2.

Safety gate: the retained zadp/botN/status (last status the board published, NOT proof that it is alive: telemetry is
the only liveness evidence) must say "bench":true, else this node logs "ERROR robot N: status does not confirm a bench
build" and exits 2 BEFORE it publishes any pose. A robot on the floor would otherwise get a fake camera. Boards flashed
before the flag existed need --allow-unflagged (run_real: opts.benchUnflagged = true). Another camera node already
publishing zadp/cam/pose for one of the robots is an ERROR too (exit 2, nothing published).
Echo faults (robot restarted: seq back to 0 after a command, or a position step larger than P.link.parkTol
between two messages) stop the echo of that robot with "ERROR robot N restarted": a real camera would have
corrected the boot pose, the echo cannot. Any uncaught exception is logged as ERROR (exit 2).
Known bias (TOPICS.md Bench mode): the echo goes through the firmware latency compensation (lag = age + netDelay), so a
wrong real round trip scales the estimate's speed slightly. Not corrected here (netDelay is shared with the real camera).

    python bench_cam.py --broker 127.0.0.1:1883 --robots 2,3 --slot 2=0.75,-0.159,3.1416 --slot 3=0.9,-0.159,3.1416
"""
import argparse
import json
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from aruco_node import pose_message          # the one camera message format (numpy / cv2 optional there)
from transport import PahoClient

DEFAULT_CONFIG = os.path.join(os.path.dirname(HERE), 'link_config.json')
SETTLE = 5            # telemetry messages in a row on the slot before the echo starts
RESET_TOL_FRAC = 0.1  # reset tolerance = this fraction of P.link.parkTol / parkTolDeg (run_real then passes by 10x)
STATUS_WAIT = 1.0     # s to collect the retained status messages after connecting


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Bench:
    """Latest telemetry of the bench robots (filled by the MQTT thread, read by the main loop)."""

    def __init__(self, ids, cam_topic=''):
        self.cam_topic = cam_topic
        self.foreign = set()                 # robots for which a camera pose was seen (checked before our first publish)
        self.faults = {}                     # id -> text, set by the echo loop
        self.lock = threading.Lock()
        self.tel = {i: None for i in ids}    # id -> dict(x, y, heading, state, rx = time.time() at arrival, n = count)
        self.status = {}
        self.n = {i: 0 for i in ids}

    def on_message(self, topic, p):
        if not isinstance(p, dict):
            return
        if topic == self.cam_topic:
            with self.lock:
                if p.get('bot') in self.tel:
                    self.foreign.add(p['bot'])
            return
        parts = topic.split('/')
        if len(parts) != 3 or not parts[1].startswith('bot'):
            return
        try:
            i = int(parts[1][3:])
        except ValueError:
            return
        if i not in self.tel:
            return
        with self.lock:
            if parts[2] == 'status':
                self.status[i] = p
            elif parts[2] == 'telemetry':
                if p.get('state') == 'offline' or not all(k in p for k in ('x', 'y', 'heading')):
                    self.tel[i] = None                       # last will / partial message: robot not seen
                    return
                self.n[i] += 1
                self.tel[i] = {'x': float(p['x']), 'y': float(p['y']), 'heading': float(p['heading']),
                               'state': p.get('state', ''), 'seq': int(p.get('seq', 0) or 0),
                               'rx': time.time(), 'n': self.n[i]}

    def snapshot(self):
        with self.lock:
            return {i: (dict(t) if t else None) for i, t in self.tel.items()}, dict(self.status)

    def pre_publish_foreign(self):
        with self.lock:
            return sorted(self.foreign)


def slot_error(t, slot):
    """(distance m, heading error deg) of a telemetry sample from a slot pose."""
    return (math.hypot(t['x'] - slot[0], t['y'] - slot[1]),
            abs(math.degrees(wrap(t['heading'] - slot[2]))))


def reset_to_slots(link, topic, bench, slots, cfg, say, timeout):
    """Publish the slot pose of every robot as camera fixes (30 Hz) until its telemetry shows it there. Returns the
    worst (mm, deg) error at the end, or None on timeout (after naming what is missing)."""
    cam, lk = cfg['cam'], cfg['link']
    tol, tol_deg = RESET_TOL_FRAC * lk['parkTol'], RESET_TOL_FRAC * lk['parkTolDeg']
    period = 1.0 / cam['rateHz']
    ok = {i: 0 for i in slots}
    last_n = {i: 0 for i in slots}
    t0 = time.monotonic()
    nxt = t0
    worst = {i: (0.0, 0.0) for i in slots}
    while time.monotonic() - t0 < timeout:
        now = time.monotonic()
        if now >= nxt:
            nxt = max(nxt + period, now - period)
            tp = time.time()
            for i, s in slots.items():
                # the slot is a fresh "image": time = now - capture delay, age = that delay (like the real camera)
                link.publish(topic, pose_message(i, s, tp - cam['captureDelay'], tp), 0)
        tel, _ = bench.snapshot()
        for i, s in slots.items():
            t = tel[i]
            if t is None or t['n'] == last_n[i]:
                continue
            last_n[i] = t['n']
            d, dh = slot_error(t, s)
            worst[i] = (d, dh)
            ok[i] = ok[i] + 1 if (d <= tol and dh <= tol_deg) else 0
        if all(v >= SETTLE for v in ok.values()):
            return max(w[0] for w in worst.values()) * 1000.0, max(w[1] for w in worst.values())
        time.sleep(0.002)
    tel, _ = bench.snapshot()
    for i, s in slots.items():
        if ok[i] >= SETTLE:
            continue
        t = tel[i]
        if t is None:
            say('ERROR robot %d: no telemetry (board off, WiFi, broker, wrong robot id?)' % i)
        else:
            d, dh = slot_error(t, s)
            say('ERROR robot %d does not reach its slot within %g s: reports x %.3f y %.3f heading %.1f deg, slot x %.3f '
                'y %.3f heading %.1f deg (%.1f mm, %.1f deg off; state %s)'
                % (i, timeout, t['x'], t['y'], math.degrees(t['heading']), s[0], s[1], math.degrees(s[2]),
                   1000 * d, dh, t['state']))
    return None


def parse_slots(items):
    slots = {}
    for s in items:
        n, v = s.split('=')
        x, y, h = [float(t) for t in v.split(',')]
        slots[int(n)] = (x, y, h)
    return slots


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', default=DEFAULT_CONFIG, help='link_config.json written by MATLAB (link_write_config)')
    ap.add_argument('--broker', default='', help='host[:port] of the MQTT broker (default P.mqtt.broker)')
    ap.add_argument('--robots', required=True, help='bench robot ids (comma list)')
    ap.add_argument('--slot', action='append', default=[], help='N=x,y,heading parking slot of robot N (repeatable)')
    ap.add_argument('--no-reset', action='store_true', help='skip the start-pose phase: echo the telemetry as it is')
    ap.add_argument('--allow-unflagged', action='store_true',
                    help='accept boards whose status has no "bench":true (flashed before the flag existed)')
    ap.add_argument('--log', default='', help='write output to this file (used when started in the background)')
    ap.add_argument('--ttl', type=float, default=0, help='exit by itself after this many seconds (0 = never)')
    args = ap.parse_args()
    if args.log:
        sys.stdout = sys.stderr = open(args.log, 'a', buffering=1)
    say = lambda s: print('bench_cam: ' + s, flush=True)
    if not os.path.isfile(args.config):
        say('ERROR %s not found: run link_write_config(twin_params()) in MATLAB first' % args.config)
        sys.exit(2)
    with open(args.config) as f:
        cfg = json.load(f)
    cam, lk = cfg['cam'], cfg['link']
    topic = cfg['mqtt']['topicCamPose']
    ids = sorted(int(s) for s in args.robots.split(',') if s.strip())
    slots = parse_slots(args.slot)
    if not args.no_reset and sorted(slots) != ids:
        say('ERROR --slot needed for exactly the robots %s (got %s), or pass --no-reset' % (ids, sorted(slots)))
        sys.exit(2)
    t_end = time.monotonic() + args.ttl if args.ttl > 0 else None

    host, _, port = (args.broker or cfg['mqtt']['broker']).partition(':')
    port = int(port or cfg['mqtt']['port'])
    bench = Bench(ids, topic)
    # rule: one client id per process: a stale bench_cam must not kick this one off the broker (broker takeover)
    link = PahoClient(host, port, client_id='zadp-benchcam-%d' % os.getpid())
    tel_topic = cfg['mqtt']['topicTelemetry']                   # status topic: same robot prefix (TOPICS.md)
    link.subscribe([tel_topic % i for i in ids] + [(tel_topic % i).rsplit('/', 1)[0] + '/status' for i in ids] + [topic],
                   bench.on_message)
    try:
        link.connect(10.0)
    except ConnectionError as e:
        say('ERROR ' + str(e))
        sys.exit(2)
    say('robots %s on broker %s:%d, publishing %s, at most %g Hz per robot, pid=%d'
        % (ids, host, port, topic, cam['rateHz'], os.getpid()))
    time.sleep(STATUS_WAIT)
    _, st = bench.snapshot()
    # rule: no pose is published before every robot's last retained status says bench:true (or --allow-unflagged)
    bad = False
    soft = 'WARNING (allowed by --allow-unflagged) ' if args.allow_unflagged else 'ERROR '
    for i in ids:
        if i not in st:
            say('%srobot %d: no retained status, so a bench build is not confirmed (board off, or firmware without '
                'status)' % (soft, i))
            bad = bad or not args.allow_unflagged
        elif st[i].get('bench') is True:
            say('robot %d: last status says bench build (bench:true, fw %s)' % (i, st[i].get('fw', '?')))
        else:
            say('%srobot %d: status does not confirm a bench build (no "bench":true; flashed before the flag, or a '
                'robot on the floor: this node would feed it a fake camera)' % (soft, i))
            bad = bad or not args.allow_unflagged
    # rule: exactly one camera node per robot
    other = bench.pre_publish_foreign()
    if other:
        say('ERROR another camera node is already publishing %s for robot(s) %s (stale bench_cam or aruco_node?)'
            % (topic, other))
        bad = True
    if bad:
        link.close()
        sys.exit(2)

    if not args.no_reset:
        res = reset_to_slots(link, topic, bench, slots, cfg, say, float(lk['readyTimeout']))
        if res is None:
            link.close()
            sys.exit(2)
        say('robots %s reset to their parking slots (worst %.2f mm, %.2f deg)' % (ids, res[0], res[1]))
    say('bench ready: echoing the robots\' telemetry pose as %s' % topic)

    period = 1.0 / cam['rateHz']
    nxt = {i: 0.0 for i in ids}
    sent_n = {i: 0 for i in ids}
    n_pub, t_stat, seen = 0, time.monotonic(), set()
    prev = {}                                            # id -> last telemetry sample checked
    while t_end is None or time.monotonic() < t_end:
        now = time.monotonic()
        tel, _ = bench.snapshot()
        for i in ids:
            t = tel[i]
            if t is None or i in bench.faults:
                continue
            if t['n'] != prev.get(i, {}).get('n'):
                q = prev.get(i)
                if q is not None:
                    step = math.hypot(t['x'] - q['x'], t['y'] - q['y'])
                    dh = abs(math.degrees(wrap(t['heading'] - q['heading'])))
                    # rule: a restarted board (seq back to 0 after a command, or a position step no motion explains)
                    # starts at its boot pose and the echo would confirm it; a real camera would correct it. Stop, tell
                    # run_real. The heading step is only reported: a robot turning on the spot steps more than
                    # parkTolDeg between two polled messages (seen on the boards: 16 deg), so it is no criterion.
                    if (q['seq'] != 0 and t['seq'] == 0) or step > lk['parkTol']:
                        bench.faults[i] = 'restarted'
                        say('ERROR robot %d restarted (seq %d -> %d, pose step %.1f mm / %.1f deg between two telemetry '
                            'messages): echo stopped' % (i, q['seq'], t['seq'], 1000 * step, dh))
                        continue
                prev[i] = t
            if t['n'] == sent_n[i] or now < nxt[i]:
                continue
            tp = time.time()
            t_img = t['rx'] - lk['netDelay']
            if tp - t_img > cam['maxAge']:                   # stale telemetry = robot not seen
                continue
            link.publish(topic, pose_message(i, (t['x'], t['y'], t['heading']), t_img, tp), 0)
            sent_n[i] = t['n']
            nxt[i] = max(nxt[i] + period, now - period) if nxt[i] else now + period
            n_pub += 1
            seen.add(i)
        if time.monotonic() - t_stat >= 5.0:
            dt = time.monotonic() - t_stat
            say('%.1f poses/s, robots seen %s' % (n_pub / dt, sorted(seen)))
            n_pub, t_stat, seen = 0, time.monotonic(), set()
        time.sleep(0.002)
    link.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
    except Exception as e:                       # rule: any failure is logged with the word ERROR (run_real stops on it)
        import traceback
        print('bench_cam: ERROR unexpected %s: %s\n%s' % (type(e).__name__, e, traceback.format_exc()), flush=True)
        sys.exit(2)
