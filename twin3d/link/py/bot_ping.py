"""bot_ping.py: check robots without the twin. Connects to the MQTT broker (starts nothing itself), prints every
telemetry message from zadp/bot1..3/telemetry for N seconds, then a rate summary per robot.

    python bot_ping.py [--broker 127.0.0.1] [--port 1883] [--seconds 5] [--bots 1,2,3]

One line per message: time since start, id, seq, x, y, heading (deg), state, rate (Hz, over the last second).
Needs paho-mqtt. Subscribes only (never publishes, so it never commands a robot).
"""
import argparse
import json
import os
import sys
import threading
import time
from collections import deque

import paho.mqtt.client as mqtt


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--broker', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=1883)
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--bots', default='1,2,3')
    a = ap.parse_args()
    ids = [int(s) for s in a.bots.split(',') if s]

    t0 = time.monotonic()
    lock = threading.Lock()
    hist = {i: deque() for i in ids}
    count = {i: 0 for i in ids}
    up = threading.Event()

    def on_connect(c, u, f, rc, p=None):
        for i in ids:
            c.subscribe('zadp/bot%d/telemetry' % i, 0)
        up.set()

    def on_message(c, u, m):
        now = time.monotonic()
        try:
            i = int(m.topic.split('/')[1][3:])
        except ValueError:
            return
        try:
            d = json.loads(m.payload.decode('utf-8'))
        except (ValueError, UnicodeDecodeError):
            print('%7.3f bot%d unparsable payload: %r' % (now - t0, i, m.payload[:80]), flush=True)
            return
        with lock:
            h = hist.setdefault(i, deque())
            h.append(now)
            while h and now - h[0] > 1.0:
                h.popleft()
            count[i] = count.get(i, 0) + 1
            rate = len(h) / min(1.0, max(now - t0, 1e-3))
        def f(k, fmt):
            v = d.get(k)
            return fmt % v if isinstance(v, (int, float)) else '-'
        hd = d.get('heading')
        hds = '%7.1f' % (hd * 180.0 / 3.141592653589793) if isinstance(hd, (int, float)) else '      -'
        print('%7.3f id %s seq %s x %s y %s heading %s deg state %s rate %.1f Hz' % (
            now - t0, d.get('id', i), d.get('seq', '-'), f('x', '%.3f'), f('y', '%.3f'), hds,
            d.get('state', '-'), rate), flush=True)

    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id='zadp-bot-ping-%d' % os.getpid())
    c.on_connect = on_connect
    c.on_message = on_message
    try:
        c.connect(a.broker, a.port, keepalive=15)
    except OSError as e:
        sys.exit('bot_ping: broker %s:%d not reachable: %s' % (a.broker, a.port, e))
    c.loop_start()
    if not up.wait(5):
        sys.exit('bot_ping: broker %s:%d did not accept the connection' % (a.broker, a.port))
    print('bot_ping: listening on zadp/bot{%s}/telemetry at %s:%d for %g s' % (a.bots, a.broker, a.port, a.seconds), flush=True)
    time.sleep(max(0.0, a.seconds - (time.monotonic() - t0)))
    c.disconnect()                       # clean DISCONNECT first (sent by the loop thread), then stop the loop
    time.sleep(0.1)
    c.loop_stop()
    T = time.monotonic() - t0
    for i in sorted(count):
        print('bot_ping: bot%d %d message(s), mean %.1f Hz' % (i, count[i], count[i] / T) if count[i]
              else 'bot_ping: bot%d no telemetry' % i, flush=True)


if __name__ == '__main__':
    main()
