"""Tests for mini_broker.py (python test_mini_broker.py; no MATLAB). Starts the broker on a free localhost port and
talks to it with paho-mqtt clients and raw sockets. Prints one line per check and 'PASS n/n' at the end (exit 1 on
any failure)."""
import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

import paho.mqtt.client as mqtt

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
results = []


def check(name, ok, detail=''):
    results.append(bool(ok))
    print('%s  %s%s' % ('ok  ' if ok else 'FAIL', name, ('  (' + detail + ')') if detail else ''), flush=True)


def free_port():
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    p = s.getsockname()[1]
    s.close()
    return p


class Sub:
    """paho client that collects messages."""

    def __init__(self, port, cid, topics=(), keepalive=30):
        self.msgs = []
        self.lock = threading.Lock()
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=cid)
        self.c.on_message = self._msg
        self.up = threading.Event()
        self.down = threading.Event()
        self.c.on_connect = lambda c, u, f, rc, p=None: self.up.set()
        self.c.on_disconnect = lambda c, u, f, rc, p=None: self.down.set()
        self.c.connect('127.0.0.1', port, keepalive=keepalive)
        self.c.loop_start()
        assert self.up.wait(5), 'connect'
        for t, q in topics:
            self.subscribe(t, q)

    def subscribe(self, t, q):
        ev = threading.Event()
        self.c.on_subscribe = lambda *a: ev.set()
        self.c.subscribe(t, q)
        assert ev.wait(5), 'suback'

    def _msg(self, c, u, m):
        with self.lock:
            self.msgs.append((m.topic, bytes(m.payload), m.qos, m.retain))

    def wait(self, n, timeout=3.0):
        t = time.time() + timeout
        while time.time() < t:
            with self.lock:
                if len(self.msgs) >= n:
                    return True
            time.sleep(0.01)
        return False

    def close(self):
        self.c.disconnect()
        self.c.loop_stop()


def raw_connect(port, cid, keepalive=30, will=None):
    """Raw MQTT CONNECT; will = (topic, payload). Returns the socket after CONNACK."""
    def s16(b):
        return struct.pack('!H', len(b)) + b
    flags = 0x02
    payload = s16(cid.encode())
    if will:
        flags |= 0x04
        payload += s16(will[0].encode()) + s16(will[1])
    var = s16(b'MQTT') + bytes([4, flags]) + struct.pack('!H', keepalive)
    body = var + payload
    s = socket.create_connection(('127.0.0.1', port), timeout=5)
    s.sendall(bytes([0x10, len(body)]) + body)
    ack = s.recv(4)
    assert ack == b'\x20\x02\x00\x00', ack
    return s


def socket_closed(s, timeout):
    s.settimeout(timeout)
    try:
        while True:
            d = s.recv(1024)
            if not d:
                return True
    except socket.timeout:
        return False
    except OSError:
        return True


def main():
    port = free_port()
    logf = os.path.join(tempfile.gettempdir(), 'mini_broker_test_%d.log' % port)
    br = subprocess.Popen([PY, '-u', os.path.join(HERE, 'mini_broker.py'), '--host', '127.0.0.1', '--port', str(port),
                           '--log', logf, '--ttl', '300', '--queue', '200'])
    procs = [br]
    try:
        t0 = time.time()
        while time.time() - t0 < 5:
            if os.path.isfile(logf) and 'listening' in open(logf).read():
                break
            time.sleep(0.05)
        check('broker starts', 'listening' in open(logf).read())

        # second broker on the same port must refuse with exit code 3
        r = subprocess.run([PY, os.path.join(HERE, 'mini_broker.py'), '--host', '127.0.0.1', '--port', str(port)],
                           capture_output=True, text=True, timeout=10)
        check('refuses a port in use (exit 3, clear message)', r.returncode == 3 and 'already in use' in r.stdout,
              'rc %d' % r.returncode)

        # QoS 0 and QoS 1 round trips
        s = Sub(port, 'sub-a', [('t/q0', 0), ('t/q1', 1)])
        p = Sub(port, 'pub-a')
        p.c.publish('t/q0', b'zero', 0)
        info = p.c.publish('t/q1', b'one', 1)
        info.wait_for_publish(3)
        check('QoS 1 PUBACK to publisher', info.is_published())
        ok = s.wait(2)
        got = {m[0]: m for m in s.msgs}
        check('QoS 0 round trip', 't/q0' in got and got['t/q0'][1] == b'zero' and got['t/q0'][2] == 0)
        check('QoS 1 round trip (delivered at QoS 1)', 't/q1' in got and got['t/q1'][1] == b'one' and got['t/q1'][2] == 1)
        # delivery QoS = min(pub, sub)
        s.msgs.clear()
        p.c.publish('t/q0', b'x', 1).wait_for_publish(3)
        s.wait(1)
        check('delivery QoS = min(pub 1, sub 0) = 0', s.msgs and s.msgs[0][2] == 0)

        # wildcards
        w = Sub(port, 'sub-w', [('zadp/+/telemetry', 0), ('deep/#', 0)])
        for t in ('zadp/bot1/telemetry', 'zadp/bot2/telemetry', 'zadp/bot1/cmd', 'deep/a/b/c', 'deep', 'other/x'):
            p.c.publish(t, t.encode(), 0)
        w.wait(4, 2)
        time.sleep(0.2)
        topics = sorted(m[0] for m in w.msgs)
        check('wildcards + and #', topics == sorted(['zadp/bot1/telemetry', 'zadp/bot2/telemetry', 'deep/a/b/c', 'deep']),
              str(topics))
        # unsubscribe
        w.c.unsubscribe('deep/#')
        time.sleep(0.2)
        w.msgs.clear()
        p.c.publish('deep/z', b'z', 0)
        p.c.publish('zadp/bot3/telemetry', b'k', 0)
        w.wait(1, 1)
        time.sleep(0.2)
        check('UNSUBSCRIBE', [m[0] for m in w.msgs] == ['zadp/bot3/telemetry'], str([m[0] for m in w.msgs]))

        # retained
        p.c.publish('ret/a', b'kept', 1, retain=True).wait_for_publish(3)
        time.sleep(0.1)
        r1 = Sub(port, 'sub-r', [('ret/#', 1)])
        r1.wait(1, 2)
        check('retained delivered on subscribe (retain flag set)', r1.msgs and r1.msgs[0][:2] == ('ret/a', b'kept')
              and r1.msgs[0][3])
        p.c.publish('ret/a', b'', 1, retain=True).wait_for_publish(3)
        time.sleep(0.1)
        r2 = Sub(port, 'sub-r2', [('ret/#', 0)])
        time.sleep(0.4)
        check('empty retained payload deletes it', not r2.msgs, str(r2.msgs))

        # 512-byte payload
        big = bytes((i * 7) % 256 for i in range(512))
        s.msgs.clear()
        p.c.publish('t/q1', big, 1).wait_for_publish(3)
        s.wait(1)
        check('512-byte payload intact', s.msgs and s.msgs[0][1] == big)

        # last will on a killed socket
        lw = Sub(port, 'sub-will', [('will/#', 0)])
        k = raw_connect(port, 'dying', 30, ('will/dying', b'{"state":"offline"}'))
        time.sleep(0.2)
        k.close()                                   # no DISCONNECT
        lw.wait(1, 3)
        check('last will on a killed socket', lw.msgs and lw.msgs[0][:2] == ('will/dying', b'{"state":"offline"}'))
        # no will after a clean DISCONNECT
        lw.msgs.clear()
        k = raw_connect(port, 'polite', 30, ('will/polite', b'x'))
        k.sendall(b'\xe0\x00')
        time.sleep(0.5)
        k.close()
        time.sleep(0.3)
        check('no will after DISCONNECT', not lw.msgs)

        # keep-alive timeout: keep-alive 1 s, silent -> closed after about 1.5 s, will published
        lw.msgs.clear()
        k = raw_connect(port, 'silent', 1, ('will/silent', b'gone'))
        ts = time.time()
        closed = socket_closed(k, 4)
        dt = time.time() - ts
        lw.wait(1, 1)
        check('keep-alive timeout at 1.5 x (closed, will sent)', closed and 1.3 < dt < 2.5 and lw.msgs
              and lw.msgs[0][0] == 'will/silent', 'closed after %.2f s' % dt)
        # PINGREQ keeps it alive
        k = raw_connect(port, 'pinger', 1)
        alive = True
        for _ in range(5):
            k.sendall(b'\xc0\x00')
            k.settimeout(1)
            try:
                alive = alive and k.recv(2) == b'\xd0\x00'
            except OSError:
                alive = False
            time.sleep(0.6)
        check('PINGREQ/PINGRESP keeps a 1 s keep-alive client for 3 s', alive)
        k.close()

        # client id takeover: old socket closed, new one works, no will from the old one
        lw.msgs.clear()
        old = raw_connect(port, 'zadp-botX', 30, ('will/takeover', b'offline'))
        new = Sub(port, 'zadp-botX', [('t/take', 0)])
        closed = socket_closed(old, 2)
        p.c.publish('t/take', b'hi', 0)
        new.wait(1, 2)
        check('client id takeover closes the old connection', closed)
        check('new connection works after takeover, no will', new.msgs and not lw.msgs, str(lw.msgs))
        new.close()

        # malformed packet closes that client only
        bad = raw_connect(port, 'bad', 30)
        bad.sendall(b'\x30\x04\x00\x09ab')          # PUBLISH whose topic length runs past the packet
        closed = socket_closed(bad, 2)
        s.msgs.clear()
        p.c.publish('t/q0', b'still', 0)
        s.wait(1)
        check('malformed packet closes only that client', closed and s.msgs and s.msgs[0][1] == b'still')

        # slow client: subscribes to everything and never reads; others keep getting messages in time
        slow = raw_connect(port, 'slow', 0)
        slow.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
        sub_body = struct.pack('!H', 1) + struct.pack('!H', 1) + b'#' + b'\x00'
        slow.sendall(bytes([0x82, len(sub_body)]) + sub_body)
        time.sleep(0.2)
        s.msgs.clear()
        pay = b'y' * 4000
        t1 = time.time()
        for i in range(2000):                       # 8 MB at the slow client, in bursts of 50 (a normal
            p.c.publish('t/q0', pay, 0)             # client's own queue cap is not the subject here)
            if i % 50 == 49:
                time.sleep(0.02)
        ok = s.wait(2000, 15)
        dt = time.time() - t1
        dropped_slow = 'slow: send queue full' in open(logf).read()
        check('slow client does not block others (2000 x 4 kB delivered, slow client drops its own)',
              ok and dropped_slow and dt < 10, '%d in %.1f s, slow dropped %d' % (len(s.msgs), dt, dropped_slow))
        slow.close()

        # 3 fake robots through the broker
        cfg = os.path.join(os.path.dirname(HERE), 'link_config.json')
        blog = os.path.join(tempfile.gettempdir(), 'mini_broker_test_bots_%d.log' % port)
        fb = subprocess.Popen([PY, '-u', os.path.join(HERE, 'fake_bot.py'), '--bots', '1,2,3', '--broker',
                               '127.0.0.1:%d' % port, '--config', cfg, '--log', blog, '--ttl', '60',
                               '--init', '1=0.44,-0.159,3.1416', '--init', '2=0.75,-0.159,3.1416',
                               '--init', '3=0.90,-0.159,3.1416'])
        procs.append(fb)
        tel = Sub(port, 'zadp-test-brain', [('zadp/+/telemetry', 0), ('zadp/cam/pose', 0)])
        t0 = time.time()
        while time.time() - t0 < 10 and not (os.path.isfile(blog) and 'attached' in open(blog).read()):
            time.sleep(0.1)
        time.sleep(1.0)
        seqs = {1: 1001, 2: 1002, 3: 1003}
        for i in (1, 2, 3):                          # a turn of 90 deg in place for each robot
            cmd = {'id': i, 'truck': 0, 'seq': seqs[i], 'action': 'turn', 'x': 0.5, 'y': 0.5, 'heading': 1.5708,
                   'speed': 0.2, 'dir': 0, 'tol': 3}
            tel.c.publish('zadp/bot%d/cmd' % i, json.dumps(cmd), 1)
        t1 = time.time()
        last = {}
        while time.time() - t1 < 8:
            for i in (1, 2, 3):
                tel.c.publish('zadp/bot%d/cmd' % i, json.dumps({'id': i, 'seq': seqs[i], 'action': 'turn',
                              'x': 0.5, 'y': 0.5, 'heading': 1.5708, 'speed': 0.2, 'dir': 0, 'tol': 3}), 0)
            with tel.lock:
                for m in tel.msgs:
                    if m[0].endswith('/telemetry'):
                        d = json.loads(m[1])
                        last[d['id']] = d
            if all(i in last and last[i]['seq'] == seqs[i] and last[i]['state'] == 'arrived' for i in (1, 2, 3)):
                break
            time.sleep(0.1)                          # 10 Hz repeats, like the bridge
        dt = time.time() - t1
        ntel = sum(1 for m in tel.msgs if m[0].endswith('/telemetry'))
        ncam = sum(1 for m in tel.msgs if m[0] == 'zadp/cam/pose')
        check('3 fake robots: telemetry from all', all(i in last for i in (1, 2, 3)), '%d telemetry, %d cam msgs' % (ntel, ncam))
        check('3 fake robots: cmd QoS 1 delivered, turn done (arrived, heading ~ pi/2)',
              all(i in last and last[i]['seq'] == seqs[i] and last[i]['state'] == 'arrived'
                  and abs(last[i]['heading'] - 1.5708) < 0.1 for i in (1, 2, 3)),
              '%.1f s; ' % dt + ', '.join('bot%d %s %.2f' % (i, last[i]['state'], last[i]['heading']) for i in sorted(last)))
        fb.kill()
        fb.wait(5)
        tel.wait(len(tel.msgs) + 1, 2)
        tel.close()
        for c in (s, p, w, r1, r2, lw):
            c.close()
        txt = open(logf).read()
        check('log has one connect line per client with id and address', 'connect sub-a from 127.0.0.1:' in txt
              and 'disconnect dying 127.0.0.1:' in txt)
    finally:
        for pr in procs:
            try:
                pr.kill()
            except OSError:
                pass
    n = sum(results)
    print('PASS %d/%d' % (n, len(results)) if n == len(results) else 'FAILED %d of %d' % (len(results) - n, len(results)))
    print('broker log: %s' % logf)
    sys.exit(0 if n == len(results) else 1)


if __name__ == '__main__':
    main()
