"""fake_bot.py: stand-in for the ESP32 robots (and the overhead camera node), for testing the link without hardware.

One process simulates one or more differential-drive robots with the REAL numbers (34 mm wheels, 75.3 mm track,
2772.2 counts/rev, 300 rpm limit, 0.08 s motor lag, 0.5 m/s^2) and the same go-to-point / turn controller as the
twin (bot/bot_controller.m, brain/brain_step.m turnCmd), so a real robot and its twin should agree to a few mm.

Per robot:  subscribes zadp/bot<N>/cmd and zadp/cam/pose (fuses its own camera fix, gain 0.5, like the twin),
            publishes zadp/bot<N>/telemetry at 20 Hz (and once at once when a new command arrives).
As the camera node: publishes zadp/cam/pose for each robot at 30 Hz, true pose + noise (3 mm, 1 deg).
Command rules are the firmware checklist in link/README.md (seq dedupe, repeated commands are keep-alives,
stop when no command line arrived for 300 ms and continue when it comes back).

Attach to the bridge (default, works in loopback and broker mode) or go straight to a broker:
    python fake_bot.py --bots 1,2,3 --init 1=0.44,-0.157,3.1416 --init 2=0.75,-0.157,3.1416 --init 3=0.90,-0.157,3.1416
    python fake_bot.py --bots 1 --broker localhost:1883
"""
import argparse
import math
import os
import queue
import random
import sys
import time

from transport import BridgeClient

# ---- numbers: copied from params/twin_params.m (P.bot, P.ctrl, P.cam, P.truck.dumpTime) --------------------
WHEEL_D = 0.034
TRACK = 0.0753
CPR = 2772.2
VMAX = 300.0 / 60 * math.pi * WHEEL_D      # wheel surface speed at the 300 rpm limit, m/s
CRUISE = 0.20
ACCEL = 0.5
TAU = 0.08
SLIP_CORR = 0.20
KP_HEAD = 3.0
KP_DIST = 2.5
ROT_THRESH = math.radians(25)
DECEL_FRAC = 0.5
ARRIVE_TOL = 0.005
HEAD_TOL = math.radians(3)
MIN_TURN_RATE = 0.3
DUMP_TIME = 4.0
CAM_POS_STD = 0.003
CAM_HEAD_STD = math.radians(1)
FUSION_GAIN = 0.5
TEL_HZ = 20.0
CAM_HZ = 30.0
DT = 0.01                                    # physics step, s
WATCHDOG = 0.3                               # s without a command line: stop


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Bot:
    def __init__(self, bid, pose, slip_std, seed):
        self.id = bid
        self.rng = random.Random(seed + bid)
        self.pose = list(pose)               # TRUE pose (what the camera sees)
        self.est = list(pose)                # the robot's own estimate: encoders + camera fixes
        self.v = [0.0, 0.0]
        self.wheel = [0.0, 0.0]
        self.enc = [0, 0]
        self.slip_std = slip_std
        self.slip = [1 + slip_std * self.rng.gauss(0, 1) for _ in range(2)]
        self.seq = 0                         # seq of the accepted command (0 = none yet)
        self.act = None                      # active command: dict(kind, phase, ...)
        self.state = 'idle'
        self.last_rx = time.monotonic()      # wall time of the last command line (also a repeat)
        self.stalled = False

    # ---------------------------------------------------------------- commands
    def on_cmd(self, c):
        """Return True when the command is new (a telemetry ack should go out at once)."""
        self.last_rx = time.monotonic()
        seq = int(c.get('seq', 0))
        if seq == self.seq and seq != 0:
            return False                     # repeat of the active (or finished) command = keep-alive
        action = c.get('action')
        self.seq = seq
        speed = min(float(c.get('speed', CRUISE) or CRUISE), CRUISE)
        if action == 'goto':
            tgt = (float(c['x']), float(c['y']))
            h = c.get('heading')
            if h is None:
                h = math.atan2(tgt[1] - self.est[1], tgt[0] - self.est[0])
            self.act = dict(kind='goto', phase='drive', tgt=tgt, h=float(h), speed=max(speed, 0.02))
            self.state = 'moving'
        elif action == 'turn':
            self.act = dict(kind='turn', phase='turn', h=float(c['heading']), dir=int(c.get('dir', 0)))
            self.state = 'turning'
        elif action == 'dump':
            self.act = dict(kind='dump', phase='dump', t=0.0)
            self.state = 'dumping'
        elif action == 'stop':
            self.act = dict(kind='stop', phase='stop')
            self.state = 'idle'
        else:
            self.act = None
            self.state = 'idle'
        return True

    # ---------------------------------------------------------------- controller (bot_controller.m)
    def goto_cmd(self, pe, tgt, cruise):
        dx, dy = tgt[0] - pe[0], tgt[1] - pe[1]
        d = math.hypot(dx, dy)
        if d < ARRIVE_TOL:
            return (0.0, 0.0), True
        e = wrap(math.atan2(dy, dx) - pe[2])
        if abs(e) > ROT_THRESH:
            wmax = cruise / (TRACK / 2)
            w = min(max(KP_HEAD * e, -wmax), wmax)
            v = 0.0
        else:
            v = min(cruise, KP_DIST * d, math.sqrt(2 * DECEL_FRAC * ACCEL * d)) * math.cos(e)
            w = KP_HEAD * e * min(1.0, d / 0.03)
        return (v - w * TRACK / 2, v + w * TRACK / 2), False

    def turn_cmd(self, th, h, d, tol):
        """In-place turn to heading h in direction d (+1 CCW, -1 CW, 0 = shortest); brain_step.m turnCmd."""
        if d == 0:
            d = 1 if wrap(h - th) >= 0 else -1
        e = (d * (h - th) + 0.3) % (2 * math.pi) - 0.3
        if abs(e) < tol:
            return (0.0, 0.0), True
        wmax = CRUISE / (TRACK / 2)
        w = d * (1 if e > 0 else -1) * min(wmax, max(MIN_TURN_RATE, KP_HEAD * abs(e)))
        return (-w * TRACK / 2, w * TRACK / 2), False

    def control(self, dt):
        """Wheel speed command [vL, vR] from the active command; updates state."""
        a = self.act
        if a is None:
            return (0.0, 0.0)
        stopped = max(abs(self.v[0]), abs(self.v[1])) < 2e-3
        k = a['kind']
        if k == 'goto':
            if a['phase'] == 'drive':
                pe = self.est
                dv = (math.cos(a['h']), math.sin(a['h']))
                along = (a['tgt'][0] - pe[0]) * dv[0] + (a['tgt'][1] - pe[1]) * dv[1]
                tgt = a['tgt']
                if along < 0.05:             # last 5 cm: aim along the leg, brake straight (brain_step.m)
                    tgt = (pe[0] + max(along, 0) * dv[0], pe[1] + max(along, 0) * dv[1])
                cmd, arr = self.goto_cmd(pe, tgt, a['speed'])
                if arr or along < ARRIVE_TOL:
                    a['phase'] = 'brake'
                    return (0.0, 0.0)
                return cmd
            if a['phase'] == 'brake':
                if stopped:
                    if abs(wrap(a['h'] - self.est[2])) > HEAD_TOL:
                        a['phase'] = 'turn'
                        a['kind'] = 'turn'; a['dir'] = 0
                        self.state = 'turning'
                    else:
                        self.finish()
                return (0.0, 0.0)
        if k == 'turn':
            if a['phase'] == 'turn':
                cmd, arr = self.turn_cmd(self.est[2], a['h'], a['dir'], HEAD_TOL)
                if arr:
                    a['phase'] = 'brake2'
                    return (0.0, 0.0)
                return cmd
            if a['phase'] == 'brake2':
                if stopped:
                    self.finish()
                return (0.0, 0.0)
        if k == 'dump':
            a['t'] += dt
            if a['t'] >= DUMP_TIME:
                self.finish()
            return (0.0, 0.0)
        if k == 'stop':
            if stopped:
                self.state = 'idle'
            return (0.0, 0.0)
        return (0.0, 0.0)

    def finish(self):
        self.act = None
        self.state = 'arrived'

    # ---------------------------------------------------------------- physics (bot_step.m, bot_odometry.m)
    def step(self, dt, silent):
        cmd = (0.0, 0.0) if silent else self.control(dt)
        if silent and self.act is not None and self.act['kind'] in ('goto', 'turn') \
                and self.act['phase'] in ('drive', 'turn'):
            self.stalled = True              # watchdog stop: the goal is kept, the command resumes when lines return
            self.state = 'idle'
        elif self.stalled and not silent and self.act is not None:
            self.stalled = False
            self.state = 'moving' if self.act['kind'] == 'goto' else 'turning'
        cmd = [min(max(c, -VMAX), VMAX) for c in cmd]
        new_v = []
        for i in range(2):
            v = self.v[i] + (cmd[i] - self.v[i]) * (1 - math.exp(-dt / TAU))
            dv = min(max(v - self.v[i], -ACCEL * dt), ACCEL * dt)
            new_v.append(self.v[i] + dv)
        self.v = new_v
        d_rot = [v * dt for v in self.v]
        for i in range(2):
            self.wheel[i] += d_rot[i] / (WHEEL_D / 2)
        enc_new = [math.floor(w / (2 * math.pi) * CPR) for w in self.wheel]
        d_enc = [enc_new[i] - self.enc[i] for i in range(2)]
        for i in range(2):
            a = math.exp(-abs(d_rot[i]) / SLIP_CORR)
            self.slip[i] = 1 + (self.slip[i] - 1) * a + self.slip_std * math.sqrt(1 - a * a) * self.rng.gauss(0, 1)
        dg = [d_rot[i] * self.slip[i] for i in range(2)]
        self.pose = arc(self.pose, (dg[0] + dg[1]) / 2, (dg[1] - dg[0]) / TRACK)
        self.enc = enc_new
        per_count = math.pi * WHEEL_D / CPR
        dl, dr = d_enc[0] * per_count, d_enc[1] * per_count
        self.est = arc(self.est, (dl + dr) / 2, (dr - dl) / TRACK)

    def fuse(self, z):
        e = [z[0] - self.est[0], z[1] - self.est[1], wrap(z[2] - self.est[2])]
        self.est = [self.est[0] + FUSION_GAIN * e[0], self.est[1] + FUSION_GAIN * e[1],
                    wrap(self.est[2] + FUSION_GAIN * e[2])]

    def telemetry(self):
        return {'id': self.id, 'seq': self.seq, 'encL': self.enc[0], 'encR': self.enc[1],
                'x': round(self.est[0], 5), 'y': round(self.est[1], 5), 'heading': round(wrap(self.est[2]), 5),
                'state': self.state}

    def cam(self):
        return {'bot': self.id,
                'x': round(self.pose[0] + CAM_POS_STD * self.rng.gauss(0, 1), 5),
                'y': round(self.pose[1] + CAM_POS_STD * self.rng.gauss(0, 1), 5),
                'heading': round(wrap(self.pose[2] + CAM_HEAD_STD * self.rng.gauss(0, 1)), 5),
                't': round(time.time(), 4)}


def arc(pose, ds, dth):
    x, y, th = pose
    if abs(dth) < 1e-9:
        return [x + ds * math.cos(th), y + ds * math.sin(th), th + dth]
    r = ds / dth
    return [x + r * (math.sin(th + dth) - math.sin(th)), y - r * (math.cos(th + dth) - math.cos(th)), th + dth]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bots', default='1,2,3', help='robot numbers simulated by this process')
    ap.add_argument('--init', action='append', default=[], help='N=x,y,heading start pose (repeatable)')
    ap.add_argument('--connect', default='127.0.0.1:18830', help='bridge host:port (default)')
    ap.add_argument('--broker', default='', help='host[:port] of an MQTT broker: talk to it directly (needs paho-mqtt)')
    ap.add_argument('--topic-cmd', default='zadp/bot%d/cmd')
    ap.add_argument('--topic-tel', default='zadp/bot%d/telemetry')
    ap.add_argument('--topic-cam', default='zadp/cam/pose')
    ap.add_argument('--slip', type=float, default=0.01, help='wheel slip std (fraction), as P.bot.slipStd')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--no-cam', action='store_true', help='do not publish zadp/cam/pose (a real camera node does)')
    ap.add_argument('--no-fuse', action='store_true', help='ignore camera poses (encoders only)')
    ap.add_argument('--log', default='', help='write output to this file (used when started in the background)')
    ap.add_argument('--ttl', type=float, default=0, help='exit by itself after this many seconds (safety net)')
    a = ap.parse_args()
    if a.log:
        sys.stdout = sys.stderr = open(a.log, 'a', buffering=1)

    ids = [int(s) for s in a.bots.split(',') if s]
    starts = {}
    for s in a.init:
        n, v = s.split('=')
        starts[int(n)] = [float(t) for t in v.split(',')]
    bots = {i: Bot(i, starts.get(i, [0.5, 0.5, 0.0]), a.slip, a.seed) for i in ids}

    if a.broker:
        from transport import PahoClient
        host, _, port = a.broker.partition(':')
        link = PahoClient(host, int(port or 1883), client_id='fake-bots-%d' % os.getpid())
    else:
        host, _, port = a.connect.partition(':')
        link = BridgeClient(host, int(port))
    inbox = queue.SimpleQueue()
    link.subscribe([a.topic_cmd % i for i in ids] + [a.topic_cam], lambda t, p: inbox.put((t, p)))
    link.connect(10.0)
    print('fake_bot: bots %s attached (%s), pid=%d' % (ids, a.broker or a.connect, os.getpid()), flush=True)

    t0 = time.monotonic()
    sim_t = 0.0
    next_tel = {i: 0.0 for i in ids}
    next_cam = {i: 0.0 for i in ids}
    end = (t0 + a.ttl) if a.ttl > 0 else None
    cmd_topics = {a.topic_cmd % i: i for i in ids}
    while end is None or time.monotonic() < end:
        now = time.monotonic()
        # incoming: commands and camera fixes
        while True:
            try:
                topic, p = inbox.get_nowait()
            except queue.Empty:
                break
            if topic in cmd_topics:
                b = bots[cmd_topics[topic]]
                if b.on_cmd(p):
                    link.publish(a.topic_tel % b.id, b.telemetry(), 0)      # ack at once
            elif topic == a.topic_cam and not a.no_fuse:
                b = bots.get(int(p.get('bot', 0)))
                if b is not None:
                    b.fuse((p['x'], p['y'], p['heading']))
        # physics, in fixed steps up to the wall clock (never more than 0.25 s behind)
        target = now - t0
        if target - sim_t > 0.25:
            sim_t = target - 0.25
        while sim_t + DT <= target:
            for b in bots.values():
                b.step(DT, now - b.last_rx > WATCHDOG and b.act is not None and b.act['kind'] in ('goto', 'turn')
                       and b.act['phase'] in ('drive', 'turn'))
            sim_t += DT
        # outgoing
        for i, b in bots.items():
            if sim_t >= next_tel[i]:
                link.publish(a.topic_tel % i, b.telemetry(), 0)
                next_tel[i] = max(next_tel[i] + 1 / TEL_HZ, sim_t - 1 / TEL_HZ)
            if not a.no_cam and sim_t >= next_cam[i]:
                link.publish(a.topic_cam, b.cam(), 0)
                next_cam[i] = max(next_cam[i] + 1 / CAM_HZ, sim_t - 1 / CAM_HZ)
        time.sleep(0.002)
    link.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
