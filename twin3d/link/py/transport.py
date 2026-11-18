"""Topic transports shared by bridge.py and fake_bot.py (standard library only; paho-mqtt only for PahoClient).

Both clients have the same small API, so a program does not care what is behind it:
    c.connect(timeout)              open the connection (raises ConnectionError)
    c.subscribe(patterns, cb)       cb(topic, payload_dict) is called from a background thread
    c.publish(topic, payload, qos)  payload is a dict, sent as JSON
    c.close()
Topic patterns follow MQTT: '+' is one level, '#' is the rest (last level only).
"""
import json
import socket
import threading
import time


def topic_match(pattern, topic):
    """True when an MQTT subscription pattern matches a topic."""
    p = pattern.split('/')
    t = topic.split('/')
    for i, seg in enumerate(p):
        if seg == '#':
            return True
        if i >= len(t):
            return False
        if seg != '+' and seg != t[i]:
            return False
    return len(p) == len(t)


class BridgeClient:
    """Newline-delimited JSON over a localhost TCP socket to bridge.py (the protocol MATLAB speaks too).
    Lines: {"topic": "...", "payload": {...}} both ways, plus {"subscribe": [patterns]} client -> bridge."""

    def __init__(self, host='127.0.0.1', port=18830):
        self.host, self.port = host, port
        self.sock = None
        self.cb = None
        self.lock = threading.Lock()
        self.alive = False
        self.patterns = []

    def connect(self, timeout=10.0):
        t_end = time.monotonic() + timeout
        last = None
        while time.monotonic() < t_end:
            try:
                self.sock = socket.create_connection((self.host, self.port), timeout=2.0)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock.settimeout(None)
                self.alive = True
                threading.Thread(target=self._reader, daemon=True).start()
                if self.patterns:
                    self._send({'subscribe': self.patterns})
                return
            except OSError as e:
                last = e
                time.sleep(0.1)
        raise ConnectionError('bridge %s:%d not reachable: %s' % (self.host, self.port, last))

    def subscribe(self, patterns, cb):
        self.cb = cb
        self.patterns = list(patterns)
        if self.alive:
            self._send({'subscribe': self.patterns})

    def publish(self, topic, payload, qos=0):
        self._send({'topic': topic, 'payload': payload})

    def _send(self, obj):
        data = (json.dumps(obj, separators=(',', ':')) + '\n').encode()
        with self.lock:
            self.sock.sendall(data)

    def _reader(self):
        buf = b''
        try:
            while True:
                chunk = self.sock.recv(65536)
                if not chunk:
                    break
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    if not line.strip():
                        continue
                    try:
                        m = json.loads(line)
                        if self.cb and 'topic' in m:
                            self.cb(m['topic'], m.get('payload', {}))
                    except ValueError:
                        pass
        except OSError:
            pass
        self.alive = False

    def close(self):
        try:
            self.sock.close()
        except (OSError, AttributeError):
            pass


class PahoClient:
    """paho-mqtt client talking straight to a broker (payloads are JSON)."""

    def __init__(self, host='localhost', port=1883, client_id=''):
        import paho.mqtt.client as mqtt          # imported here so loopback needs no paho
        try:                                      # paho 2.x
            self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        except AttributeError:                    # paho 1.x
            self.c = mqtt.Client(client_id=client_id)
        self.host, self.port = host, int(port)
        self.patterns, self.cb = [], None
        self.up = threading.Event()
        self.c.on_connect = self._on_connect
        self.c.on_message = self._on_message

    def _on_connect(self, client, userdata, flags, rc, *args):
        for p in self.patterns:
            client.subscribe(p, qos=0)
        self.up.set()

    def _on_message(self, client, userdata, msg):
        if self.cb is None:
            return
        try:
            self.cb(msg.topic, json.loads(msg.payload.decode('utf-8')))
        except ValueError:
            pass

    def connect(self, timeout=5.0):
        try:
            self.c.connect(self.host, self.port, keepalive=30)
        except OSError as e:
            raise ConnectionError('broker %s:%d not reachable: %s' % (self.host, self.port, e))
        self.c.loop_start()
        if not self.up.wait(timeout):
            raise ConnectionError('broker %s:%d did not accept the connection' % (self.host, self.port))

    def subscribe(self, patterns, cb):
        self.cb = cb
        self.patterns = list(patterns)
        if self.up.is_set():
            for p in self.patterns:
                self.c.subscribe(p, qos=0)

    def publish(self, topic, payload, qos=0):
        self.c.publish(topic, json.dumps(payload, separators=(',', ':')), qos=qos)

    def close(self):
        try:
            self.c.loop_stop()
            self.c.disconnect()
        except Exception:
            pass
