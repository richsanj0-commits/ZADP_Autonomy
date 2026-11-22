"""bridge.py: connects MATLAB (and other local programs) to MQTT.

MATLAB (link_open) and the fake robots speak newline-delimited JSON over a localhost TCP socket:
    {"topic": "zadp/bot1/cmd", "payload": {...}}      publish
    {"subscribe": ["zadp/+/telemetry", ...]}          choose which topics this client receives
and receive {"topic": ..., "payload": ...} lines for every matching message.
    {"heartbeat": t}                                  "I am alive" (MATLAB, link_keepalive)
On connect the bridge sends {"topic": "zadp/bridge/status", "payload": {"mode": "loopback"|"broker", ...}}.

Keep-alive repeats (TOPICS.md "Timing"): the bridge repeats each robot's LAST command (same seq, QoS 0) at
P.link.repeatHz while the client that published it is connected and was heard (any line) within
P.link.hostTimeout. A brain step that stalls (planning, 0.4-0.6 s) therefore never trips the robots' 300 ms
watchdog; a MATLAB that hangs stops the repeats after hostTimeout, one that closes its socket stops them at
once, and the robots' watchdog then stops the wheels. Numbers: link_config.json (--config), from twin_params.

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
import queue
import socket
import sys
import threading
import time

from transport import topic_match, PahoClient


HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(os.path.dirname(HERE), 'link_config.json')


class Hub:
    def __init__(self, paho=None, repeat_hz=10.0, host_timeout=1.0):
        self.paho = paho                      # PahoClient or None (loopback)
        self.clients = []                     # each client is [sock, patterns, send_queue, last_rx (monotonic)]
        self.last_seq = {}                    # cmd topic -> seq of the last cmd published (repeats go out at QoS 0)
        self.last_cmd = {}                    # cmd topic -> (payload, client) of the last command (keep-alive repeats)
        self.lock = threading.Lock()
        self.counts = {}
        self.mode = 'broker' if paho else 'loopback'
        self.repeat_hz = repeat_hz
        self.host_timeout = host_timeout
        self.n_repeats = 0

    def repeater(self):
        # rule: repeat each robot's last command (same seq) at repeat_hz while its sender is connected and was
        # heard within host_timeout; otherwise stay silent so the robot's watchdog stops it
        period = 1.0 / self.repeat_hz
        nxt = time.monotonic()
        while True:
            nxt += period
            time.sleep(max(0.0, nxt - time.monotonic()))
            now = time.monotonic()
            with self.lock:
                items = [(t, pc[0]) for t, pc in self.last_cmd.items()
                         if pc[1] in self.clients and now - pc[1][3] <= self.host_timeout]
            for topic, payload in items:
                self.publish(topic, payload, repeat=True)
                self.n_repeats += 1
            if now - nxt > 1.0:
                nxt = now                     # the machine was suspended: do not burst

    def deliver(self, topic, payload):
        line = (json.dumps({'topic': topic, 'payload': payload}, separators=(',', ':')) + '\n').encode()
        with self.lock:
            targets = [c for c in self.clients if any(topic_match(p, topic) for p in c[1])]
        for c in targets:
            self.send_line(c, line)

    QUEUE_MAX = 2000                          # lines waiting for one client (about 10 s of all traffic)

    def send_line(self, c, line):
        # never blocks the publisher: the line goes to the client's own queue, sent by its sender thread.
        # A client that does not read (MATLAB at a breakpoint, crashed without close) fills its queue and is
        # dropped instead of freezing every publisher (the fake robots, the camera node)
        try:
            c[2].put_nowait(line)
        except queue.Full:
            print('bridge: client too slow, dropped', file=sys.stderr, flush=True)
            self.drop(c)

    def sender(self, c):
        while True:
            line = c[2].get()
            if line is None:
                return
            try:
                c[0].sendall(line)
            except OSError:
                self.drop(c)
                return

    def publish(self, topic, payload, repeat=False, client=None):
        self.counts[topic] = self.counts.get(topic, 0) + 1
        if not repeat and client is not None and topic.endswith('/cmd'):
            with self.lock:
                self.last_cmd[topic] = (payload, client)
        if self.paho:
            # the broker echoes the message back; it reaches the subscribed TCP clients through on_message
            # QoS 1 only for a NEW command (new seq); its 10 Hz keep-alive repeats go at QoS 0, so a broker never
            # queues stale repeats for an offline robot and replays them later as old seqs
            qos = 0
            if topic.endswith('/cmd'):
                seq = payload.get('seq') if isinstance(payload, dict) else None
                if self.last_seq.get(topic) != seq:
                    qos = 1
                self.last_seq[topic] = seq
            self.paho.publish(topic, payload, qos=qos)
        else:
            self.deliver(topic, payload)

    def drop(self, c):
        with self.lock:
            if c in self.clients:
                self.clients.remove(c)
            for t in [t for t, pc in self.last_cmd.items() if pc[1] is c]:
                del self.last_cmd[t]          # its sender is gone: no more repeats
        try:
            c[2].put_nowait(None)             # ends the sender thread
        except queue.Full:
            pass
        try:
            c[0].close()
        except OSError:
            pass

    def serve_client(self, sock):
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        c = [sock, [], queue.Queue(self.QUEUE_MAX), time.monotonic()]
        with self.lock:
            self.clients.append(c)
        threading.Thread(target=self.sender, args=(c,), daemon=True).start()
        status = {'mode': self.mode, 'broker': self.paho.host if self.paho else None,
                  'repeatHz': self.repeat_hz, 'hostTimeout': self.host_timeout}
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
                    c[3] = time.monotonic()      # any line = the client is alive (heartbeat included)
                    try:
                        m = json.loads(line)
                    except ValueError:
                        continue
                    if not isinstance(m, dict):
                        continue
                    if 'subscribe' in m:
                        c[1] = list(m['subscribe'])
                    elif 'topic' in m:
                        self.publish(m['topic'], m.get('payload', {}), client=c)
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
    ap.add_argument('--broker', default=None, help='default P.mqtt.broker from the config')
    ap.add_argument('--broker-port', type=int, default=None, help='default P.mqtt.port from the config')
    ap.add_argument('--config', default=DEFAULT_CONFIG, help='link_config.json written by MATLAB (link_write_config)')
    ap.add_argument('--log', default='', help='write output to this file (used when started in the background)')
    ap.add_argument('--ttl', type=float, default=0, help='exit by itself after this many seconds (safety net, 0 = never)')
    a = ap.parse_args()
    if a.log:
        sys.stdout = sys.stderr = open(a.log, 'a', buffering=1)
    if not os.path.isfile(a.config):
        print('bridge: %s not found: run link_write_config(twin_params()) in MATLAB first' % a.config, file=sys.stderr, flush=True)
        sys.exit(2)
    with open(a.config) as f:
        C = json.load(f)
    if a.broker is None:
        a.broker = C['mqtt']['broker']
    if a.broker_port is None:
        a.broker_port = int(C['mqtt']['port'])

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
    hub = Hub(paho, float(C['link']['repeatHz']), float(C['link']['hostTimeout']))
    threading.Thread(target=hub.repeater, daemon=True).start()
    if paho:
        paho.subscribe(['zadp/#'], hub.deliver)
        paho.connect(5.0)
        why = 'broker %s:%d' % (a.broker, a.broker_port)

    srv = socket.socket()
    if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
        # Windows: SO_REUSEADDR would let this bridge bind a port a stale bridge still listens on (both then
        # get half the clients); exclusive use makes the bind fail instead
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
    else:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', a.port))
    srv.listen(8)
    print('bridge: listening on 127.0.0.1:%d, mode=%s (%s), repeat %g Hz while the sender was heard within %g s, pid=%d'
          % (a.port, hub.mode, why, hub.repeat_hz, hub.host_timeout, os.getpid()), flush=True)
    if a.ttl > 0:
        t = threading.Timer(a.ttl, lambda: os._exit(0))
        t.daemon = True
        t.start()
    while True:
        s, _ = srv.accept()
        threading.Thread(target=hub.serve_client, args=(s,), daemon=True).start()


if __name__ == '__main__':
    main()
