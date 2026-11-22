"""fake_bot.py: stand-in for the ESP32 robots (and the overhead camera node), for testing the link without hardware.

One process simulates one or more differential-drive robots. Every number comes from link_config.json, which
MATLAB writes from params/twin_params.m (link_write_config): real robot geometry and motor model (bot_step.m),
the go-to-point / turn controller and the command semantics of the firmware (firmware/zadp_bot/include/
motion_core.h, the same laws as bot/bot_controller.m and brain/brain_step.m). Contract: link/TOPICS.md.

Per robot:  subscribes zadp/bot<N>/cmd and zadp/cam/pose (fuses its own fix: latency-compensated with the
            message's age, gain P.cam.fusionGain, like the firmware), publishes zadp/bot<N>/telemetry at
            P.link.telemetryHz and once at once when a new command arrives.
As the camera (unless --no-cam): publishes zadp/cam/pose per robot at P.cam.rateHz: the true pose of
            P.cam.latency ago + noise (3 mm, 1 deg), with t (image time) and age.
--truth:    publishes zadp/sim/truth {bot, x, y, heading, t} (true pose now) at P.cam.rateHz, the input of
            aruco_node.py --sim (the simulated camera of run_real('dry')).

    python fake_bot.py --bots 1,2,3 --init 1=0.44,-0.159,3.1416 --init 2=0.75,-0.159,3.1416 --init 3=0.90,-0.159,3.1416
    python fake_bot.py --bots 1 --broker localhost:1883
"""
import argparse
import json
import math
import os
import queue
import random
import sys
import time

from transport import BridgeClient

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(os.path.dirname(HERE), 'link_config.json')
TOPIC_TRUTH = 'zadp/sim/truth'


def load_config(path):
    if not os.path.isfile(path):
        sys.exit('fake_bot: %s not found: run link_write_config(twin_params()) in MATLAB first' % path)
    with open(path) as f:
        return json.load(f)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def arc(pose, ds, dth):
    x, y, th = pose
    if abs(dth) < 1e-9:
        return [x + ds * math.cos(th), y + ds * math.sin(th), th + dth]
    r = ds / dth
    return [x + r * (math.sin(th + dth) - math.sin(th)), y - r * (math.cos(th + dth) - math.cos(th)), th + dth]


class Num:
    """The numbers of link_config.json as attributes (one place)."""
    def __init__(self, C):
        b, c, br, cam, lk = C['bot'], C['ctrl'], C['brain'], C['cam'], C['link']
        self.WHEEL_D = b['wheelDiameter']
        self.TRACK = b['trackEff']                      # effective track (P.bot.trackScale)
        self.CPR = b['encoderCPR']
        self.VMAX = b['maxWheelSpeed']
        self.CRUISE = b['cruiseSpeed']
        self.ACCEL = b['maxAccel']
        self.TAU = b['motorTimeConstant']
        self.SLIP_CORR = b['slipCorrLength']
        self.KP_HEAD = c['kpHeading']
        self.KP_DIST = c['kpDistance']
        self.ROT_THRESH = math.radians(c['rotateThreshDeg'])
        self.ROT_EXIT = math.radians(c['rotateExitDeg'])
        self.CAPTURE_R = c['captureRadius']
        self.DECEL_FRAC = c['decelFrac']
        self.ARRIVE_TOL = c['arriveTol']
        self.HEAD_TOL = math.radians(br['turnTolDeg'])
        self.MIN_TURN_RATE = br['minTurnRate']
        self.SLIDE = br['slideDist']
        self.LOOKAHEAD = br['lookahead']
        self.DUMP_TIME = C['truck']['dumpTime']
        self.CAM_POS_STD = cam['posStd']
        self.CAM_HEAD_STD = math.radians(cam['headingStdDeg'])
        self.CAM_LATENCY = cam['latency']
        self.CAM_MAX_AGE = cam['maxAge']
        self.FUSION_GAIN = cam['fusionGain']
        self.CAM_HZ = cam['rateHz']
        self.TEL_HZ = lk['telemetryHz']
        self.WATCHDOG = lk['watchdog']
        self.NET_DELAY = lk['netDelay']
        self.DT = C['sim']['dt']
        self.MIN_GOTO = 0.02                            # firmware MIN_GOTO_SPEED (not a twin number)
        self.TURN_SLACK = 0.3                           # brain_step.m turnCmd wrap slack
        self.STOPPED = 2e-3                             # brain_step.m "stopped" wheel speed


class Bot:
    def __init__(self, bid, pose, slip_std, seed, N):
        self.N = N
        self.id = bid
        self.rng = random.Random(seed + bid)
        self.pose = list(pose)               # TRUE pose (what the camera sees)
        self.est = list(pose)                # the robot's own estimate: encoders + camera fixes
        self.odom = list(pose)               # encoders only (latency compensation)
        self.v = [0.0, 0.0]
        self.wheel = [0.0, 0.0]
        self.enc = [0, 0]
        self.slip_std = slip_std
        self.slip = [1 + slip_std * self.rng.gauss(0, 1) for _ in range(2)]
        self.seq = 0                         # seq of the accepted command (0 = none yet), uint32
        self.act = None                      # active command: dict(kind, phase, ...)
        self.state = 'idle'
        self.last_rx = time.monotonic()      # wall time of the last command line (also a repeat)
        self.stalled = False
        self.rot = False
        self.t = 0.0                         # sim time
        self.hist = [(0.0, list(pose), list(pose))]   # (t, true pose, odom) for the camera latency
        self.last_cam_t = None

    # ---------------------------------------------------------------- commands (motion_core.h onCommand)
    def on_cmd(self, c):
        """Return True when the command is new (a telemetry ack should go out at once)."""
        N = self.N
        self.last_rx = time.monotonic()
        seq = int(c.get('seq', 0) or 0) & 0xFFFFFFFF
        if seq == self.seq and seq != 0:
            return False                     # repeat of the active (or finished) command = keep-alive
        self.seq = seq
        self.stalled = False                 # a new command clears a watchdog stop
        self.rot = False
        action = c.get('action')
        speed = min(float(c.get('speed') or N.CRUISE), N.CRUISE)
        if action == 'goto' and ('x' not in c or 'y' not in c):
            action = None
        if action == 'turn' and 'heading' not in c:
            action = None
        if action == 'goto':
            tgt = (float(c['x']), float(c['y']))
            h = c.get('heading')
            if h is None:                    # no heading: the line from here to the target
                h = math.atan2(tgt[1] - self.est[1], tgt[0] - self.est[0])
            self.act = dict(kind='goto', phase='drive', tgt=tgt, h=float(h), speed=max(speed, N.MIN_GOTO))
            self.state = 'moving'
        elif action == 'turn':
            tol = float(c.get('tol') or 0)
            self.act = dict(kind='turn', phase='turn', h=float(c['heading']), dir=int(c.get('dir', 0) or 0),
                            tol=math.radians(tol) if tol > 0 else N.HEAD_TOL)
            self.state = 'turning'
        elif action == 'dump':
            self.act = dict(kind='dump', phase='dump', t=0.0)
            self.state = 'dumping'
        elif action == 'stop':
            self.act = dict(kind='stop', phase='stop')
            self.state = 'idle'              # idle while the wheels run down, arrived once stopped
        else:
            self.act = None                  # unknown: cancel, wheels to zero, idle
            self.state = 'idle'
        return True

    # ---------------------------------------------------------------- laws (bot_controller.m, turnCmd)
    def goto_cmd(self, pe, tgt, cruise):
        N = self.N
        dx, dy = tgt[0] - pe[0], tgt[1] - pe[1]
        d = math.hypot(dx, dy)
        if d < N.ARRIVE_TOL:
            self.rot = False
            return (0.0, 0.0), True
        e = wrap(math.atan2(dy, dx) - pe[2])
        if d < N.CAPTURE_R:
            self.rot = False                 # inside the capture radius: never rotate in place
        elif self.rot:
            self.rot = abs(e) > N.ROT_EXIT   # hysteresis
        else:
            self.rot = abs(e) > N.ROT_THRESH
        if self.rot:
            wmax = cruise / (N.TRACK / 2)
            w = min(max(N.KP_HEAD * e, -wmax), wmax)
            v = 0.0
        else:
            v = min(cruise, N.KP_DIST * d, math.sqrt(2 * N.DECEL_FRAC * N.ACCEL * d)) * math.cos(e)
            es = e if math.cos(e) >= 0 else wrap(e + math.pi)   # backing up: steer the tail toward the goal
            w = N.KP_HEAD * es * min(1.0, d / N.CAPTURE_R)
        return (v - w * N.TRACK / 2, v + w * N.TRACK / 2), False

    def turn_cmd(self, th, h, d, tol):
        N = self.N
        if d == 0:
            d = 1 if wrap(h - th) >= 0 else -1
        e = (d * (h - th) + N.TURN_SLACK) % (2 * math.pi) - N.TURN_SLACK
        if abs(e) < tol:
            return (0.0, 0.0), True
        wmax = N.CRUISE / (N.TRACK / 2)
        w = d * (1 if e > 0 else -1) * min(wmax, max(N.MIN_TURN_RATE, N.KP_HEAD * abs(e)))
        return (-w * N.TRACK / 2, w * N.TRACK / 2), False

    def coast(self, v):
        N = self.N
        v = max(v, 0.0)
        a, tau = N.ACCEL, N.TAU
        return v * tau if v <= a * tau else v * v / (2 * a) + a * tau * tau / 2

    def control(self, dt):
        """Wheel speed command [vL, vR] from the active command; updates state (motion_core.h control)."""
        N = self.N
        a = self.act
        if a is None:
            return (0.0, 0.0)
        stopped = max(abs(self.v[0]), abs(self.v[1])) < N.STOPPED
        k = a['kind']
        if k == 'goto':
            if a['phase'] == 'drive':
                pe = self.est
                c, s = math.cos(a['h']), math.sin(a['h'])
                tx, ty = a['tgt']
                along = (tx - pe[0]) * c + (ty - pe[1]) * s
                aim = along + N.ARRIVE_TOL
                if along < N.SLIDE:          # last slideDist: aim along the leg, brake straight
                    tgt = (pe[0] + max(aim, 0) * c, pe[1] + max(aim, 0) * s)
                else:                        # carrot lookahead ahead of the foot point on the leg line
                    fx, fy = tx - along * c, ty - along * s
                    la = min(along, N.LOOKAHEAD)
                    cvx, cvy = fx + la * c - pe[0], fy + la * s - pe[1]
                    n = math.hypot(cvx, cvy) or 1.0
                    tgt = (pe[0] + aim * cvx / n, pe[1] + aim * cvy / n)
                cmd, arr = self.goto_cmd(pe, tgt, a['speed'])
                if along <= self.coast((self.v[0] + self.v[1]) / 2) or (arr and along <= N.ARRIVE_TOL):
                    a['phase'] = 'brake'
                    self.rot = False
                    return (0.0, 0.0)
                return cmd
            if a['phase'] == 'brake' and stopped:
                self.finish()                # no final turn: the brain plans every turn itself
            return (0.0, 0.0)
        if k == 'turn':
            if a['phase'] == 'turn':
                cmd, arr = self.turn_cmd(self.est[2], a['h'], a['dir'], a['tol'])
                if arr:
                    a['phase'] = 'brake2'
                    return (0.0, 0.0)
                return cmd
            if a['phase'] == 'brake2' and stopped:
                self.finish()
            return (0.0, 0.0)
        if k == 'dump':
            a['t'] += dt
            if a['t'] >= N.DUMP_TIME:
                self.finish()
            return (0.0, 0.0)
        if k == 'stop':
            if stopped:
                self.finish()                # the stop's seq reports arrived once the wheels stopped
            return (0.0, 0.0)
        return (0.0, 0.0)

    def finish(self):
        self.act = None
        self.state = 'arrived'

    def watchdog_active(self):
        a = self.act
        return a is not None and ((a['kind'] == 'goto' and a['phase'] == 'drive') or
                                  (a['kind'] == 'turn' and a['phase'] == 'turn'))

    # ---------------------------------------------------------------- physics (bot_step.m, bot_odometry.m)
    def step(self, dt, silent):
        N = self.N
        if silent:
            cmd = (0.0, 0.0)
            self.stalled = True              # watchdog stop: the goal is kept, the command resumes when lines return
            self.state = 'idle'
            self.rot = False
        else:
            if self.stalled:
                self.stalled = False
                if self.act is not None:
                    self.state = 'moving' if self.act['kind'] == 'goto' else 'turning'
            cmd = self.control(dt)
        cmd = [min(max(c, -N.VMAX), N.VMAX) for c in cmd]
        new_v = []
        for i in range(2):
            v = self.v[i] + (cmd[i] - self.v[i]) * (1 - math.exp(-dt / N.TAU))
            dv = min(max(v - self.v[i], -N.ACCEL * dt), N.ACCEL * dt)
            new_v.append(self.v[i] + dv)
        d_rot = [(self.v[i] + new_v[i]) / 2 * dt for i in range(2)]   # trapezoidal, as bot_step
        self.v = new_v
        for i in range(2):
            self.wheel[i] += d_rot[i] / (N.WHEEL_D / 2)
        enc_new = [math.floor(w / (2 * math.pi) * N.CPR) for w in self.wheel]
        d_enc = [enc_new[i] - self.enc[i] for i in range(2)]
        for i in range(2):
            a = math.exp(-abs(d_rot[i]) / N.SLIP_CORR)
            self.slip[i] = 1 + (self.slip[i] - 1) * a + self.slip_std * math.sqrt(1 - a * a) * self.rng.gauss(0, 1)
        dg = [d_rot[i] * self.slip[i] for i in range(2)]
        self.pose = arc(self.pose, (dg[0] + dg[1]) / 2, (dg[1] - dg[0]) / N.TRACK)
        self.enc = enc_new
        per_count = math.pi * N.WHEEL_D / N.CPR
        dl, dr = d_enc[0] * per_count, d_enc[1] * per_count
        self.est = arc(self.est, (dl + dr) / 2, (dr - dl) / N.TRACK)
        self.odom = arc(self.odom, (dl + dr) / 2, (dr - dl) / N.TRACK)
        self.t += dt
        self.hist.append((self.t, list(self.pose), list(self.odom)))
        keep = int(math.ceil(max(N.CAM_MAX_AGE, N.CAM_LATENCY) / dt)) + 5
        if len(self.hist) > keep:
            del self.hist[:len(self.hist) - keep]

    def hist_at(self, t, col):
        """Pose (col 1 = true, 2 = odom) at sim time t, linearly interpolated (hist_at.m)."""
        H = self.hist
        if t <= H[0][0]:
            return list(H[0][col])
        for k in range(len(H) - 1, -1, -1):
            if H[k][0] <= t:
                if k == len(H) - 1:
                    return list(H[k][col])
                a, b = H[k][col], H[k + 1][col]
                f = (t - H[k][0]) / (H[k + 1][0] - H[k][0])
                return [a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2] + f * wrap(b[2] - a[2])]
        return list(H[0][col])

    def fuse(self, p):
        """Camera fix (motion_core.h onCamera without the firmware's jump gate): order check, lag = age + netDelay,
        moved forward by the odometry since the image, est += gain * (z - est)."""
        N = self.N
        t_img = float(p.get('t') or 0)
        if t_img > 0:
            if self.last_cam_t is not None and t_img <= self.last_cam_t:
                return
            self.last_cam_t = t_img
        age = p.get('age')
        lag = float(age) + N.NET_DELAY if age is not None and float(age) >= 0 else N.CAM_LATENCY
        if lag > N.CAM_MAX_AGE:
            return
        o0, o1 = self.hist_at(self.t - lag, 2), self.odom
        c0, s0 = math.cos(o0[2]), math.sin(o0[2])
        dx, dy = o1[0] - o0[0], o1[1] - o0[1]
        rx, ry = c0 * dx + s0 * dy, -s0 * dx + c0 * dy
        dth = wrap(o1[2] - o0[2])
        hz = float(p['heading'])
        cz, sz = math.cos(hz), math.sin(hz)
        z = (float(p['x']) + cz * rx - sz * ry, float(p['y']) + sz * rx + cz * ry, hz + dth)
        e = [z[0] - self.est[0], z[1] - self.est[1], wrap(z[2] - self.est[2])]
        g = N.FUSION_GAIN
        self.est = [self.est[0] + g * e[0], self.est[1] + g * e[1], wrap(self.est[2] + g * e[2])]

    def telemetry(self):
        return {'id': self.id, 'seq': self.seq, 'encL': self.enc[0], 'encR': self.enc[1],
                'x': round(self.est[0], 5), 'y': round(self.est[1], 5), 'heading': round(wrap(self.est[2]), 5),
                'state': self.state}

    def cam(self):
        """Camera message: the true pose of P.cam.latency ago; its age field = latency - netDelay (the rest is
        the simulated transit), so a robot's age + netDelay is the true lag."""
        N = self.N
        p = self.hist_at(self.t - N.CAM_LATENCY, 1)
        now = time.time()
        return {'bot': self.id,
                'x': round(p[0] + N.CAM_POS_STD * self.rng.gauss(0, 1), 5),
                'y': round(p[1] + N.CAM_POS_STD * self.rng.gauss(0, 1), 5),
                'heading': round(wrap(p[2] + N.CAM_HEAD_STD * self.rng.gauss(0, 1)), 5),
                't': round(now - N.CAM_LATENCY, 4), 'age': round(N.CAM_LATENCY - N.NET_DELAY, 4)}

    def truth(self):
        return {'bot': self.id, 'x': round(self.pose[0], 6), 'y': round(self.pose[1], 6),
                'heading': round(wrap(self.pose[2]), 6), 't': round(time.time(), 4)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bots', default='1,2,3', help='robot numbers simulated by this process')
    ap.add_argument('--init', action='append', default=[], help='N=x,y,heading start pose (repeatable)')
    ap.add_argument('--config', default=DEFAULT_CONFIG, help='link_config.json written by MATLAB (link_write_config)')
    ap.add_argument('--connect', default='127.0.0.1:18830', help='bridge host:port (default)')
    ap.add_argument('--broker', default='', help='host[:port] of an MQTT broker: talk to it directly (needs paho-mqtt)')
    ap.add_argument('--slip', type=float, default=None, help='wheel slip std (fraction), default P.bot.slipStd')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--no-cam', action='store_true', help='do not publish zadp/cam/pose (a camera node does)')
    ap.add_argument('--no-fuse', action='store_true', help='ignore camera poses (encoders only)')
    ap.add_argument('--truth', action='store_true', help='publish zadp/sim/truth (input of aruco_node.py --sim)')
    ap.add_argument('--log', default='', help='write output to this file (used when started in the background)')
    ap.add_argument('--ttl', type=float, default=0, help='exit by itself after this many seconds (safety net)')
    a = ap.parse_args()
    if a.log:
        sys.stdout = sys.stderr = open(a.log, 'a', buffering=1)
    C = load_config(a.config)
    N = Num(C)
    slip = C['bot']['slipStd'] if a.slip is None else a.slip
    tcmd, ttel, tcam = C['mqtt']['topicCmd'], C['mqtt']['topicTelemetry'], C['mqtt']['topicCamPose']

    ids = [int(s) for s in a.bots.split(',') if s]
    starts = {}
    for s in a.init:
        n, v = s.split('=')
        starts[int(n)] = [float(t) for t in v.split(',')]
    bots = {i: Bot(i, starts.get(i, [0.5, 0.5, 0.0]), slip, a.seed, N) for i in ids}

    if a.broker:
        from transport import PahoClient
        host, _, port = a.broker.partition(':')
        link = PahoClient(host, int(port or 1883), client_id='fake-bots-%d' % os.getpid())
    else:
        host, _, port = a.connect.partition(':')
        link = BridgeClient(host, int(port))
    inbox = queue.SimpleQueue()
    link.subscribe([tcmd % i for i in ids] + [tcam], lambda t, p: inbox.put((t, p)))
    link.connect(10.0)
    print('fake_bot: bots %s attached (%s), pid=%d' % (ids, a.broker or a.connect, os.getpid()), flush=True)

    t0 = time.monotonic()
    sim_t = 0.0
    next_tel = {i: 0.0 for i in ids}
    next_cam = {i: 0.0 for i in ids}
    end = (t0 + a.ttl) if a.ttl > 0 else None
    cmd_topics = {tcmd % i: i for i in ids}
    while end is None or time.monotonic() < end:
        now = time.monotonic()
        while True:
            try:
                topic, p = inbox.get_nowait()
            except queue.Empty:
                break
            if topic in cmd_topics:
                b = bots[cmd_topics[topic]]
                if isinstance(p, dict) and p.get('id', b.id) != b.id:
                    continue                 # command for another robot: ignored
                if b.on_cmd(p):
                    link.publish(ttel % b.id, b.telemetry(), 0)      # ack at once
            elif topic == tcam and not a.no_fuse:
                b = bots.get(int(p.get('bot', 0)))
                if b is not None:
                    b.fuse(p)
        target = now - t0
        if target - sim_t > 0.25:
            sim_t = target - 0.25
        while sim_t + N.DT <= target:
            for b in bots.values():
                b.step(N.DT, now - b.last_rx > N.WATCHDOG and b.watchdog_active())
            sim_t += N.DT
        for i, b in bots.items():
            if sim_t >= next_tel[i]:
                link.publish(ttel % i, b.telemetry(), 0)
                next_tel[i] = max(next_tel[i] + 1 / N.TEL_HZ, sim_t - 1 / N.TEL_HZ)
            if sim_t >= next_cam[i]:
                if not a.no_cam:
                    link.publish(tcam, b.cam(), 0)
                if a.truth:
                    link.publish(TOPIC_TRUTH, b.truth(), 0)
                next_cam[i] = max(next_cam[i] + 1 / N.CAM_HZ, sim_t - 1 / N.CAM_HZ)
        time.sleep(0.002)
    link.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
