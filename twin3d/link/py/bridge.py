"""bridge.py: connects MATLAB (and other local programs) to MQTT.

MATLAB (link_open) and the fake robots speak newline-delimited JSON over a localhost TCP socket:
    {"topic": "zadp/bot1/cmd", "payload": {...}}      publish
    {"subscribe": ["zadp/+/telemetry", ...]}          choose which topics this client receives
and receive {"topic": ..., "payload": ...} lines for every matching message.
On connect the bridge sends {"topic": "zadp/bridge/status", "payload": {"mode": "loopback"|"broker", ...}}.

Modes
  broker    paho-mqtt connects to a real MQTT broker (--broker HOST --broker-port 1883); every message
            published by a TCP client goes to the broker, everything on zadp/# coming back from the broker
            is forwarded to the TCP clients that subscribed to a matching topic.
  loopback  no broker: the bridge itself is the topic router (same MQTT wildcard rules). Fake robots attach
            to it over the same TCP port, so the rest of the system runs unchanged.
Default is auto: broker if paho-mqtt is importable AND the broker answers within 1 s, otherwise loopback
(--loopback forces it, --no-fallback refuses it, e.g. when real robots are connected).

    python bridge.py [--port 18830] [--loopback] [--broker localhost] [--broker-port 1883] [--ttl 900]
"""
import argparse
import json
import os
import socket
import sys
import threading

from transport import topic_match, PahoClient


class Hub:
    def __init__(self, paho=None):
        self.paho = paho                      # PahoClient or None (loopback)
        self.clients = []                     # each client is [sock, patterns, send_lock]
        self.lock = threading.Lock()
        self.counts = {}
        self.mode = 'broker' if paho else 'loopback'

    def deliver(self, topic, payload):
        line = (json.dumps({'topic': topic, 'payload': payload}, separators=(',', ':')) + '\n').encode()
        with self.lock:
            targets = [c for c in self.clients if any(topic_match(p, topic) for p in c[1])]
        for c in targets:
            self.send_line(c, line)

    def send_line(self, c, line):
        try:
            with c[2]:
                c[0].sendall(line)
        except OSError:
            self.drop(c)

    def publish(self, topic, payload):
        self.counts[topic] = self.counts.get(topic, 0) + 1
        if self.paho:
            # the broker echoes the message back; it reaches the subscribed TCP clients through on_message
            self.paho.publish(topic, payload, qos=1 if topic.endswith('/cmd') else 0)
        else:
            self.deliver(topic, payload)

    def drop(self, c):
        with self.lock:
            if c in self.clients:
                self.clients.remove(c)
        try:
            c[0].close()
        except OSError:
            pass

    def serve_client(self, sock):
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        c = [sock, [], threading.Lock()]
        with self.lock:
            self.clients.append(c)
        status = {'mode': self.mode, 'broker': self.paho.host if self.paho else None}
        self.send_line(c, (json.dumps({'topic': 'zadp/bridge/status', 'payload': status}) + '\n').encode())
        buf = b''
        try:
            while True:
                chunk = sock.recv(65536)
                if not chunk:
                    break
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    if not line.strip():
                        continue
                    try:
                        m = json.loads(line)
                    except ValueError:
                        continue
                    if 'subscribe' in m:
                        c[1] = list(m['subscribe'])
                    elif 'topic' in m:
                        self.publish(m['topic'], m.get('payload', {}))
        except OSError:
            pass
        self.drop(c)


def broker_up(host, port):
    try:
        socket.create_connection((host, port), timeout=1.0).close()
        return True
    except OSError:
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=18830, help='localhost TCP port for MATLAB / fake robots')
    ap.add_argument('--loopback', action='store_true', help='force the in-process router, never use a broker')
    ap.add_argument('--no-fallback', action='store_true', help='exit with an error instead of falling back to loopback')
    ap.add_argument('--broker', default='localhost')
    ap.add_argument('--broker-port', type=int, default=1883)
    ap.add_argument('--log', default='', help='write output to this file (used when started in the background)')
    ap.add_argument('--ttl', type=float, default=0, help='exit by itself after this many seconds (safety net, 0 = never)')
    a = ap.parse_args()
    if a.log:
        sys.stdout = sys.stderr = open(a.log, 'a', buffering=1)

    paho = None
    why = 'forced'
    if not a.loopback:
        have = True
        try:
            import importlib; importlib.import_module("paho.mqtt.client")
        except ImportError:
            have, why = False, 'paho-mqtt not installed'
        if have and not broker_up(a.broker, a.broker_port):
            have, why = False, 'no broker at %s:%d' % (a.broker, a.broker_port)
        if have:
            paho = PahoClient(a.broker, a.broker_port, client_id='zadp-bridge')
        elif a.no_fallback:
            print('bridge: %s and --no-fallback given' % why, file=sys.stderr, flush=True)
            sys.exit(2)
    hub = Hub(paho)
    if paho:
        paho.subscribe(['zadp/#'], hub.deliver)
        paho.connect(5.0)
        why = 'broker %s:%d' % (a.broker, a.broker_port)

    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', a.port))
    srv.listen(8)
    print('bridge: listening on 127.0.0.1:%d, mode=%s (%s), pid=%d' % (a.port, hub.mode, why, os.getpid()), flush=True)
    if a.ttl > 0:
        t = threading.Timer(a.ttl, lambda: os._exit(0))
        t.daemon = True
        t.start()
    while True:
        s, _ = srv.accept()
        threading.Thread(target=hub.serve_client, args=(s,), daemon=True).start()


if __name__ == '__main__':
    main()
