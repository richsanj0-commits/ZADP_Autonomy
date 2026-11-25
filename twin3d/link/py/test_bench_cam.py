"""test_bench_cam.py: bench_cam.py against a private mini_broker.py and two fake_bot.py robots that play bench boards
(python only, no MATLAB, never touches P.mqtt.port). The stand-ins get the firmware's camera gate (first fix initialises;
a fix > 50 mm / 20 deg off is rejected until 10 in a row agree, then the estimate is replaced; nearer fixes are blended),
so the start-pose rule is tested the way the firmware behaves. Needs link/link_config.json and paho-mqtt.
Checks: both robots start off their slots (one far: relock, one near: blend) and end on them, "bench ready" is logged,
the camera messages have exactly the aruco_node.py fields with strictly increasing t and age >= 0, a goto of robot 2 is
followed by the echo (cam == telemetry), the rate is at most P.cam.rateHz, a missing robot gives an ERROR and exit 2, an unflagged status (no bench:true) is
refused before any pose is published (and accepted with --allow-unflagged), a second camera node is refused, a restarted
board (seq back to 0, pose jump) stops its echo.
    python test_bench_cam.py
"""
import json
import math
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from transport import PahoClient          # noqa: E402

# fake_bot.py with the firmware's camera gate (motion_core.h onCamera), written to a temp file and run as a process
GATED = """
import math, sys
sys.path.insert(0, HERE_PLACEHOLDER)
import fake_bot
Bot = fake_bot.Bot
orig_init = Bot.__init__
def init(self, *a, **k):
    orig_init(self, *a, **k)
    self.rej = 0                                       # a board that already had a camera fix before (camInit true)
def fuse(self, p):
    N = self.N
    t_img = float(p.get('t') or 0)
    if self.last_cam_t is not None and t_img <= self.last_cam_t:
        return
    self.last_cam_t = t_img
    lag = float(p['age']) + N.NET_DELAY
    if lag > N.CAM_MAX_AGE:
        return
    o0, o1 = self.hist_at(self.t - lag, 2), self.odom
    c0, s0 = math.cos(o0[2]), math.sin(o0[2])
    dx, dy = o1[0] - o0[0], o1[1] - o0[1]
    rx, ry = c0 * dx + s0 * dy, -s0 * dx + c0 * dy
    hz = float(p['heading']); cz, sz = math.cos(hz), math.sin(hz)
    z = [float(p['x']) + cz * rx - sz * ry, float(p['y']) + sz * rx + cz * ry, hz + fake_bot.wrap(o1[2] - o0[2])]
    e = [z[0] - self.est[0], z[1] - self.est[1], fake_bot.wrap(z[2] - self.est[2])]
    if math.hypot(e[0], e[1]) > 0.05 or abs(e[2]) > math.radians(20):
        self.rej += 1
        if self.rej >= 10:
            self.est, self.rej = z, 0
            print('gated: bot %d relock' % self.id, flush=True)
        return
    self.rej = 0
    g = N.FUSION_GAIN
    self.est = [self.est[0] + g * e[0], self.est[1] + g * e[1], fake_bot.wrap(self.est[2] + g * e[2])]
Bot.__init__, Bot.fuse = init, fuse
fake_bot.main()
"""


def free_port():
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    p = s.getsockname()[1]
    s.close()
    return p


def main():
    cfg = json.load(open(os.path.join(os.path.dirname(HERE), 'link_config.json')))
    tmp = tempfile.mkdtemp(prefix='zadp_bench_test_')
    port = free_port()
    procs = []

    def start(args, log):
        f = open(os.path.join(tmp, log), 'w')
        p = subprocess.Popen([sys.executable, '-u'] + args, stdout=f, stderr=subprocess.STDOUT)
        procs.append(p)
        return p

    cam_msgs, tel = [], {}
    lock = threading.Lock()
    try:
        start([os.path.join(HERE, 'mini_broker.py'), '--port', str(port), '--ttl', '120'], 'broker.log')
        time.sleep(1.0)
        gated = os.path.join(tmp, 'gated_bot.py')
        with open(gated, 'w') as f:
            f.write(GATED.replace('HERE_PLACEHOLDER', repr(HERE)))
        start([gated, '--bots', '2,3', '--broker', '127.0.0.1:%d' % port, '--no-cam', '--ttl', '100',
               '--init', '2=0.3,0.3,1.0', '--init', '3=0.93,-0.159,2.97'], 'bots.log')
        time.sleep(3.5)
        probe = PahoClient('127.0.0.1', port, client_id='test-probe')

        def cb(t, p):
            with lock:
                if t == 'zadp/cam/pose':
                    cam_msgs.append((time.time(), p))
                elif t.endswith('/telemetry') and 'x' in p:
                    tel[p['id']] = p
        probe.subscribe(['zadp/#'], cb)
        probe.connect(5)
        for i in (1, 2, 3):                      # retained status like the bench firmware (robot 1 has no telemetry)
            probe.c.publish('zadp/bot%d/status' % i, json.dumps({'id': i, 'bench': True, 'fw': 'test'}), retain=True)
        time.sleep(0.3)
        # unflagged status (robot 4 is not a bench robot here): refused before any pose, accepted with the switch
        probe.c.publish('zadp/bot4/status', json.dumps({'id': 4, 'fw': 'old'}), retain=True)
        time.sleep(0.3)
        with lock:
            n_pre = len(cam_msgs)
        r = subprocess.run([sys.executable, os.path.join(HERE, 'bench_cam.py'), '--broker', '127.0.0.1:%d' % port,
                            '--robots', '4', '--slot', '4=0.5,0.5,0.0'], capture_output=True, text=True, timeout=60)
        assert r.returncode == 2 and 'ERROR robot 4: status does not confirm' in r.stdout, (r.returncode, r.stdout)
        time.sleep(0.3)
        with lock:
            assert len(cam_msgs) == n_pre, 'a pose was published for an unflagged robot'
        r = subprocess.run([sys.executable, os.path.join(HERE, 'bench_cam.py'), '--broker', '127.0.0.1:%d' % port,
                            '--robots', '4', '--slot', '4=0.5,0.5,0.0', '--allow-unflagged'],
                           capture_output=True, text=True, timeout=60)
        assert r.returncode == 2 and 'WARNING (allowed' in r.stdout and 'no telemetry' in r.stdout, (r.returncode, r.stdout)
        print('ok: unflagged status refused before any pose; --allow-unflagged only warns')
        slots = {2: (0.75, -0.159, 3.1416), 3: (0.90, -0.159, 3.1416)}
        camlog = os.path.join(tmp, 'cam.log')
        start([os.path.join(HERE, 'bench_cam.py'), '--broker', '127.0.0.1:%d' % port, '--robots', '2,3',
               '--slot', '2=0.75,-0.159,3.1416', '--slot', '3=0.9,-0.159,3.1416', '--ttl', '60', '--log', camlog],
              'cam.out')
        t0 = time.time()
        while time.time() - t0 < 20 and 'bench ready' not in (open(camlog).read() if os.path.exists(camlog) else ''):
            time.sleep(0.1)
        log = open(camlog).read()
        assert 'bench ready' in log, 'no "bench ready": ' + log
        assert 'gated: bot 2 relock' in open(os.path.join(tmp, 'bots.log')).read(), 'robot 2 was not relocked'
        time.sleep(0.5)
        with lock:
            for i, s in slots.items():
                d = math.hypot(tel[i]['x'] - s[0], tel[i]['y'] - s[1])
                assert d < 0.003, 'robot %d %.1f mm from its slot' % (i, 1000 * d)
            n0 = len(cam_msgs)
        # a goto of robot 2 (bridge-style repeats): the echo follows (cam pose == telemetry pose)
        t1 = time.time()
        while time.time() - t1 < 8:
            probe.publish('zadp/bot2/cmd', {'id': 2, 'truck': 1, 'seq': 77, 'action': 'goto', 'x': 0.6, 'y': -0.10,
                                            'heading': math.pi, 'speed': 0.2}, 0)
            time.sleep(0.1)
        with lock:
            assert tel[2]['state'] == 'arrived' and tel[2]['seq'] == 77, 'goto not arrived: %s' % tel[2]
            m2 = [p for _, p in cam_msgs[n0:] if p['bot'] == 2]
            assert abs(m2[-1]['x'] - tel[2]['x']) < 0.002 and abs(m2[-1]['y'] - tel[2]['y']) < 0.002, 'cam != telemetry'
            allm = [p for _, p in cam_msgs[n0:]]
            span = cam_msgs[-1][0] - cam_msgs[n0][0]
        assert all(sorted(p) == ['age', 'bot', 'heading', 't', 'x', 'y'] for p in allm), 'fields'
        for i in (2, 3):
            ts = [p['t'] for p in allm if p['bot'] == i]
            assert all(b > a for a, b in zip(ts, ts[1:])), 'robot %d: t not strictly increasing' % i
        assert all(p['age'] >= 0 for p in allm), 'negative age'
        rate = len(m2) / span
        assert rate <= cfg['cam']['rateHz'] + 1, 'rate %.1f Hz' % rate
        print('ok: relock + blend to the slots, echo follows a goto (%d msgs, robot 2 %.1f Hz, age %.3f..%.3f s)'
              % (len(allm), rate, min(p['age'] for p in allm), max(p['age'] for p in allm)))
        # a robot that never reports: ERROR, exit code 2
        # second camera node while the first one still publishes: refused
        r = subprocess.run([sys.executable, os.path.join(HERE, 'bench_cam.py'), '--broker', '127.0.0.1:%d' % port,
                            '--robots', '2', '--slot', '2=0.75,-0.159,3.1416'], capture_output=True, text=True, timeout=60)
        assert r.returncode == 2 and 'ERROR another camera node' in r.stdout, (r.returncode, r.stdout)
        print('ok: second camera node refused')
        # a restarted board: seq back to 0 and a pose jump in the telemetry of robot 3 stops its echo
        for _ in range(3):
            probe.publish('zadp/bot3/telemetry', {'id': 3, 'x': 0.5, 'y': 0.5, 'heading': 0.0, 'state': 'idle', 'seq': 0}, 0)
            time.sleep(0.05)
        time.sleep(0.5)
        assert 'ERROR robot 3 restarted' in open(camlog).read(), open(camlog).read()[-600:]
        print('ok: restarted board detected')
        r = subprocess.run([sys.executable, os.path.join(HERE, 'bench_cam.py'), '--broker', '127.0.0.1:%d' % port,
                            '--robots', '1', '--slot', '1=0.44,-0.159,3.1416'], capture_output=True, text=True, timeout=60)
        assert r.returncode == 2 and 'ERROR robot 1: no telemetry' in r.stdout, (r.returncode, r.stdout)
        print('ok: missing robot -> exit 2, "%s"' % r.stdout.strip().splitlines()[-1][:80])
        probe.close()
        print('test_bench_cam PASSED')
    finally:
        for p in procs:
            p.kill()


if __name__ == '__main__':
    main()
