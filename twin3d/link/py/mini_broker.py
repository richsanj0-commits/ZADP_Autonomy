"""mini_broker.py: a small MQTT 3.1.1 broker, Python standard library only (threads), enough for the ZADP link.

Used when no real broker (Mosquitto) runs on this PC: link_launch broker mode and run_real start it by themselves
when nothing answers on P.mqtt.broker:P.mqtt.port and that host is this PC. A real broker, if one runs, is always
preferred: this program refuses to start when the port is already taken.

Supported (rules are stated as comments where they are enforced):
  CONNECT / CONNACK (protocol level 4 "MQTT"; level 3 "MQIsdp" accepted too), clean session only (a session-present
  request is accepted as clean), username / password accepted and ignored, keep-alive timeout at 1.5 x the client's
  keep-alive, client id takeover (the old connection is closed), last will published on abnormal disconnect.
  SUBSCRIBE / SUBACK with + and # wildcards (topics starting with $ are not matched by a leading wildcard),
  UNSUBSCRIBE / UNSUBACK, PUBLISH QoS 0 and 1 both ways (PUBACK; delivery QoS = min(publish, subscription); QoS 2 from
  a client is acknowledged with PUBREC / PUBCOMP and delivered at QoS <= 1), retained messages (stored, delivered on
  subscribe with the retain flag, an empty retained payload deletes), PINGREQ / PINGRESP, DISCONNECT.
  No offline queues, no retries: an unacknowledged outgoing QoS 1 message is not resent (clean sessions only).
  A malformed packet closes that client only. A slow client never blocks others: each client has its own send queue
  (cap --queue packets) drained by its own thread; a full queue drops new PUBLISH packets for that client only.

    python mini_broker.py [--host 0.0.0.0] [--port 1883] [--verbose] [--log file] [--ttl s]

Prints one line per connect / disconnect (client id, address, reason). Exit code 3 = port already in use.
"""
import argparse
import os
import queue
import socket
import struct
import sys
import threading
import time

MAX_PACKET = 256 * 1024          # bytes; larger = malformed (the robots' buffer is 512 B)
CONNECT_TIMEOUT = 10.0           # s from TCP accept to CONNECT
KEEPALIVE_FACTOR = 1.5           # MQTT 3.1.1 section 3.1.2.10

VERBOSE = False


def log(msg):
    print('%s mini_broker: %s' % (time.strftime('%H:%M:%S'), msg), flush=True)


def vlog(msg):
    if VERBOSE:
        log(msg)


class Malformed(Exception):
    pass


# ------------------------------------------------------------------ encoding helpers
def enc_len(n):
    out = bytearray()
    while True:
        b = n % 128
        n //= 128
        if n:
            b |= 0x80
        out.append(b)
        if not n:
            return bytes(out)


def enc_str(s):
    b = s.encode('utf-8') if isinstance(s, str) else s
    return struct.pack('!H', len(b)) + b


def packet(first, body=b''):
    return bytes([first]) + enc_len(len(body)) + body


class Reader:
    """Cursor over one packet body; any overrun = malformed."""

    def __init__(self, data):
        self.d, self.i = data, 0

    def u8(self):
        if self.i + 1 > len(self.d):
            raise Malformed('truncated')
        v = self.d[self.i]
        self.i += 1
        return v

    def u16(self):
        if self.i + 2 > len(self.d):
            raise Malformed('truncated')
        v = struct.unpack_from('!H', self.d, self.i)[0]
        self.i += 2
        return v

    def raw(self):
        n = self.u16()
        if self.i + n > len(self.d):
            raise Malformed('truncated string')
        v = self.d[self.i:self.i + n]
        self.i += n
        return v

    def string(self):
        try:
            s = self.raw().decode('utf-8')
        except UnicodeDecodeError:
            raise Malformed('bad UTF-8')
        if '\x00' in s:
            raise Malformed('NUL in string')     # 3.1.1 section 1.5.3
        return s

    def rest(self):
        v = self.d[self.i:]
        self.i = len(self.d)
        return v

    def left(self):
        return len(self.d) - self.i


# ------------------------------------------------------------------ topics
def valid_filter(f):
    # rule: non-empty; '#' only as the whole last level; '+' only as a whole level
    if not f:
        return False
    lv = f.split('/')
    for i, s in enumerate(lv):
        if '#' in s and (s != '#' or i != len(lv) - 1):
            return False
        if '+' in s and s != '+':
            return False
    return True


def valid_topic(t):
    # rule: a PUBLISH topic is non-empty and has no wildcard
    return bool(t) and '+' not in t and '#' not in t


def match(f, t):
    # rule: a filter starting with a wildcard does not match a topic starting with '$' (3.1.1 section 4.7.2)
    if t.startswith('$') and f[:1] in ('+', '#'):
        return False
    fl, tl = f.split('/'), t.split('/')
    for i, s in enumerate(fl):
        if s == '#':
            return True
        if i >= len(tl):
            return False
        if s != '+' and s != tl[i]:
            return False
    return len(fl) == len(tl)


# ------------------------------------------------------------------ broker
class Broker:
    def __init__(self, queue_cap):
        self.lock = threading.Lock()
        self.clients = {}          # client id -> Client (connected only)
        self.retained = {}         # topic -> (payload, qos)
        self.queue_cap = queue_cap
        self.n_auto = 0

    def route(self, topic, payload, qos, retain, sender=None):
        if retain:
            with self.lock:
                if payload:
                    self.retained[topic] = (payload, qos)
                else:
                    self.retained.pop(topic, None)      # rule: an empty retained payload deletes the retained message
        with self.lock:
            targets = []
            for c in self.clients.values():
                best = -1
                for f, sq in c.subs.items():
                    if sq > best and match(f, topic):
                        best = sq
                if best >= 0:
                    targets.append((c, min(qos, best)))  # rule: delivery QoS = min(publish QoS, subscription QoS)
        for c, q in targets:
            c.send_publish(topic, payload, q, False)     # rule: retain flag 0 when forwarding to live subscriptions
        vlog('route %s (%d B, qos %d, retain %d) -> %d client(s)' % (topic, len(payload), qos, retain, len(targets)))

    def register(self, c):
        with self.lock:
            old = self.clients.get(c.cid)
            self.clients[c.cid] = c
        if old is not None:
            # rule: a second CONNECT with the same client id takes over; the old connection is closed WITHOUT its
            # will (the client is evidently alive: e.g. a robot that reconnected before its old socket timed out)
            old.close('taken over by %s:%d' % c.addr, will=False)

    def unregister(self, c):
        with self.lock:
            if self.clients.get(c.cid) is c:
                del self.clients[c.cid]

    def retained_for(self, f):
        with self.lock:
            return [(t, p, q) for t, (p, q) in self.retained.items() if match(f, t)]

    def new_id(self):
        with self.lock:
            self.n_auto += 1
            return 'auto-%d-%d' % (os.getpid(), self.n_auto)


class Client:
    def __init__(self, broker, sock, addr):
        self.b, self.sock, self.addr = broker, sock, addr
        self.cid = None
        self.subs = {}             # filter -> granted QoS
        self.will = None           # (topic, payload, qos, retain)
        self.keepalive = 0
        self.q = queue.Queue(broker.queue_cap)
        self.closed = False
        self.close_lock = threading.Lock()
        self.pid_lock = threading.Lock()
        self.next_pid = 0
        self.dropped = 0
        self.registered = False

    # ---------------------------------------------------------- sending
    def enqueue(self, data, droppable):
        if self.closed:
            return
        try:
            self.q.put_nowait(data)
        except queue.Full:
            if droppable:
                # rule: a slow client loses its own PUBLISH packets; nobody else waits for it
                self.dropped += 1
                if self.dropped == 1 or self.dropped % 1000 == 0:
                    log('%s: send queue full (%d packets), %d message(s) dropped' % (self.cid, self.b.queue_cap, self.dropped))
            else:
                self.close('send queue full (control packet)', will=True)

    def send_publish(self, topic, payload, qos, retain):
        first = 0x30 | (qos << 1) | (1 if retain else 0)
        body = enc_str(topic)
        if qos:
            with self.pid_lock:
                self.next_pid = self.next_pid % 65535 + 1      # packet id 1..65535, never 0
                body += struct.pack('!H', self.next_pid)
        self.enqueue(packet(first, body + payload), droppable=True)

    def writer(self):
        while True:
            data = self.q.get()
            if data is None:
                return
            try:
                self.sock.sendall(data)
            except OSError as e:
                self.close('send failed: %s' % e, will=True)
                return

    # ---------------------------------------------------------- receiving
    def recv_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError('socket closed by peer')
            buf += chunk
        return bytes(buf)

    def read_packet(self):
        first = self.recv_exact(1)[0]
        mult, n = 1, 0
        for i in range(4):
            b = self.recv_exact(1)[0]
            n += (b & 0x7F) * mult
            mult *= 128
            if not b & 0x80:
                break
        else:
            raise Malformed('remaining length longer than 4 bytes')
        if n > MAX_PACKET:
            raise Malformed('packet of %d bytes > %d' % (n, MAX_PACKET))
        return first, self.recv_exact(n)

    def run(self):
        reason, will = 'socket closed', True
        try:
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock.settimeout(CONNECT_TIMEOUT)
            first, body = self.read_packet()
            if first >> 4 != 1:
                raise Malformed('first packet is not CONNECT')
            if not self.on_connect(first, body):
                reason, will = 'connect refused', False
                return
            threading.Thread(target=self.writer, daemon=True).start()
            # rule: keep-alive timeout = 1.5 x the client's keep-alive (any received byte resets it); 0 = none
            self.sock.settimeout(KEEPALIVE_FACTOR * self.keepalive if self.keepalive else None)
            while not self.closed:
                first, body = self.read_packet()
                if self.handle(first, body) == 'disconnect':
                    reason, will = 'client DISCONNECT', False   # rule: a clean DISCONNECT discards the will
                    break
        except socket.timeout:
            reason = 'keep-alive timeout (%g s)' % (KEEPALIVE_FACTOR * self.keepalive) if self.registered else 'no CONNECT'
        except Malformed as e:
            reason = 'malformed packet: %s' % e
        except (ConnectionError, OSError) as e:
            reason = 'socket closed (%s)' % e.__class__.__name__ if not isinstance(e, ConnectionError) else str(e)
        finally:
            self.close(reason, will)

    def on_connect(self, first, body):
        if first & 0x0F:
            raise Malformed('CONNECT flags')
        r = Reader(body)
        proto = r.string()
        level = r.u8()
        flags = r.u8()
        self.keepalive = r.u16()
        if (proto, level) not in (('MQTT', 4), ('MQIsdp', 3)):
            self.sock.sendall(packet(0x20, b'\x00\x01'))        # 0x01 unacceptable protocol version
            log('refused %s:%d: protocol %r level %d' % (self.addr + (proto, level)))
            return False
        if flags & 0x01:
            raise Malformed('reserved CONNECT flag set')
        clean = bool(flags & 0x02)
        cid = r.string()
        if flags & 0x04:
            wt = r.string()
            wp = r.raw()
            wq = (flags >> 3) & 3
            if wq == 3 or not valid_topic(wt):
                raise Malformed('bad will')
            self.will = (wt, wp, min(wq, 1), bool(flags & 0x20))
        elif flags & 0x38:
            raise Malformed('will QoS/retain without will flag')
        if flags & 0x80:
            r.string()                                         # username: accepted, ignored
        if flags & 0x40:
            if not flags & 0x80:
                raise Malformed('password without username')
            r.raw()                                            # password: accepted, ignored
        if not cid:
            if not clean:
                self.sock.sendall(packet(0x20, b'\x00\x02'))    # 0x02 identifier rejected
                log('refused %s:%d: empty client id without clean session' % self.addr)
                return False
            cid = self.b.new_id()
        self.cid = cid
        # rule: clean sessions only; a request for a stored session is served as a clean one (session present 0)
        self.sock.sendall(packet(0x20, b'\x00\x00'))
        self.registered = True
        self.b.register(self)
        log('connect %s from %s:%d (keep-alive %d s, clean %d%s)' % (
            cid, self.addr[0], self.addr[1], self.keepalive, clean,
            ', will on %s' % self.will[0] if self.will else ''))
        return True

    def handle(self, first, body):
        t, fl = first >> 4, first & 0x0F
        if t == 3:                                             # PUBLISH
            qos = (fl >> 1) & 3
            if qos == 3:
                raise Malformed('PUBLISH QoS 3')
            r = Reader(body)
            topic = r.string()
            if not valid_topic(topic):
                raise Malformed('PUBLISH topic %r' % topic)
            pid = r.u16() if qos else 0
            if qos and pid == 0:
                raise Malformed('packet id 0')
            payload = r.rest()
            vlog('%s PUBLISH %s qos %d retain %d %d B' % (self.cid, topic, qos, fl & 1, len(payload)))
            if qos == 1:
                self.enqueue(packet(0x40, struct.pack('!H', pid)), False)       # PUBACK
            elif qos == 2:
                self.enqueue(packet(0x50, struct.pack('!H', pid)), False)       # PUBREC (QoS 2: delivered at once)
            self.b.route(topic, payload, min(qos, 1), bool(fl & 1), self)
        elif t == 4:                                           # PUBACK of our QoS 1 delivery: nothing stored
            if len(body) != 2:
                raise Malformed('PUBACK length')
        elif t == 6:                                           # PUBREL (client QoS 2) -> PUBCOMP
            if fl != 2 or len(body) != 2:
                raise Malformed('PUBREL')
            self.enqueue(packet(0x70, body), False)
        elif t in (5, 7):                                      # PUBREC / PUBCOMP: we never send QoS 2
            if len(body) != 2:
                raise Malformed('PUBREC/PUBCOMP length')
        elif t == 8:                                           # SUBSCRIBE
            if fl != 2:
                raise Malformed('SUBSCRIBE flags')
            r = Reader(body)
            pid = r.u16()
            req = []
            while r.left():
                f = r.string()
                rq = r.u8()
                if rq & 0xFC:
                    raise Malformed('SUBSCRIBE options')
                req.append((f, rq))
            if not req:
                raise Malformed('SUBSCRIBE without filter')
            codes, new = bytearray(), []
            for f, rq in req:
                if rq == 3 or not valid_filter(f):
                    codes.append(0x80)                         # failure for this filter only
                    continue
                g = min(rq, 1)                                 # rule: granted QoS at most 1
                with self.b.lock:
                    self.subs[f] = g
                codes.append(g)
                new.append((f, g))
            self.enqueue(packet(0x90, struct.pack('!H', pid) + bytes(codes)), False)
            vlog('%s SUBSCRIBE %s' % (self.cid, ', '.join('%s q%d' % x for x in req)))
            for f, g in new:                                   # rule: retained messages follow the SUBACK
                for topic, payload, q in self.b.retained_for(f):
                    self.send_publish(topic, payload, min(q, g), True)
        elif t == 10:                                          # UNSUBSCRIBE
            if fl != 2:
                raise Malformed('UNSUBSCRIBE flags')
            r = Reader(body)
            pid = r.u16()
            fs = []
            while r.left():
                fs.append(r.string())
            if not fs:
                raise Malformed('UNSUBSCRIBE without filter')
            with self.b.lock:
                for f in fs:
                    self.subs.pop(f, None)
            self.enqueue(packet(0xB0, struct.pack('!H', pid)), False)
            vlog('%s UNSUBSCRIBE %s' % (self.cid, ', '.join(fs)))
        elif t == 12:                                          # PINGREQ
            if body:
                raise Malformed('PINGREQ length')
            self.enqueue(packet(0xD0), False)
        elif t == 14:                                          # DISCONNECT
            return 'disconnect'
        else:
            raise Malformed('packet type %d from a client' % t)
        return None

    # ---------------------------------------------------------- closing
    def close(self, reason, will):
        with self.close_lock:
            if self.closed:
                return
            self.closed = True
        if self.registered:
            self.b.unregister(self)
        try:
            self.sock.shutdown(socket.SHUT_RDWR)               # wakes the reader and writer threads
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass
        try:
            self.q.put_nowait(None)
        except queue.Full:
            pass
        if self.registered:
            log('disconnect %s %s:%d (%s)' % (self.cid, self.addr[0], self.addr[1], reason))
            # rule: the last will is published on an abnormal disconnect (not after DISCONNECT, not on takeover)
            if will and self.will:
                wt, wp, wq, wr = self.will
                log('will of %s -> %s' % (self.cid, wt))
                self.b.route(wt, wp, wq, wr)
        else:
            vlog('closed %s:%d before CONNECT (%s)' % (self.addr[0], self.addr[1], reason))


def port_in_use(host, port):
    probe = '127.0.0.1' if host in ('0.0.0.0', '') else host
    try:
        socket.create_connection((probe, port), timeout=1.0).close()
        return True
    except OSError:
        return False


def main():
    global VERBOSE
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='0.0.0.0', help='address to listen on (default all interfaces)')
    ap.add_argument('--port', type=int, default=1883)
    ap.add_argument('--queue', type=int, default=1000, help='per-client send queue cap, packets')
    ap.add_argument('--verbose', action='store_true', help='one line per packet')
    ap.add_argument('--log', default='', help='append output to this file (background use)')
    ap.add_argument('--ttl', type=float, default=0, help='exit by itself after this many seconds (0 = never)')
    a = ap.parse_args()
    VERBOSE = a.verbose
    if a.log:
        sys.stdout = sys.stderr = open(a.log, 'a', buffering=1)

    msg = ('port %d is already in use (a broker such as Mosquitto probably runs: use it, '
           'or stop it before starting mini_broker)' % a.port)
    if port_in_use(a.host, a.port):
        log(msg)
        sys.exit(3)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)   # Windows: never share the port
    else:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        srv.bind((a.host, a.port))
    except OSError as e:
        log('%s [%s]' % (msg, e))
        sys.exit(3)
    srv.listen(32)
    servers = [srv]
    extra = ''
    if a.host in ('0.0.0.0', '') and socket.has_ipv6:
        # rule: on all interfaces also listen on [::] (IPv6 only), so 'localhost' (resolved to ::1 first on Windows)
        # connects at once instead of after a refused-connection retry of about 1 s; best effort
        try:
            s6 = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
            s6.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
            if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
                s6.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            s6.bind(('::', a.port))
            s6.listen(32)
            servers.append(s6)
            extra = ' and [::]:%d' % a.port
        except OSError as e:
            extra = ' (IPv6 listener not available: %s)' % e
    b = Broker(a.queue)
    log('listening on %s:%d%s, MQTT 3.1.1, pid=%d' % (a.host, a.port, extra, os.getpid()))
    if a.ttl > 0:
        t = threading.Timer(a.ttl, lambda: os._exit(0))
        t.daemon = True
        t.start()

    def accept_loop(srv):
        while True:
            try:
                s, addr = srv.accept()
            except OSError as e:
                log('accept failed: %s' % e)
                time.sleep(0.1)
                continue
            c = Client(b, s, addr[:2])
            threading.Thread(target=c.run, daemon=True).start()

    for sv in servers[1:]:
        threading.Thread(target=accept_loop, args=(sv,), daemon=True).start()
    accept_loop(servers[0])


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
