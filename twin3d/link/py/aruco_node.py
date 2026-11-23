"""aruco_node.py: overhead ArUco camera node, publishes zadp/cam/pose (contract: link/TOPICS.md).

Numbers come from link_config.json (MATLAB writes it from params/twin_params.m, P.cam / P.link / P.mqtt).

Markers (DICT_4X4_50, P.cam.dictionary; printable sheets: link/markers/make_markers.py):
  robot ids 1-3   (P.cam.robotIds) flat on the robot top, the marker's printed UP edge toward the robot's NOSE.
                  Marker centre P.cam.markerOffset AHEAD of the axle (0.026 m: centred on the 100 mm top), plane
                  P.cam.markerHeight above the floor.
  corner ids 10-13 (P.cam.cornerIds) fixed, centres at P.cam.cornerXY (arena m), plane P.cam.cornerHeight above the
                  floor. Orientation of the corner markers does not matter (only their centres are used).
Pipeline per frame: detect -> homography image -> arena plane from the 4 corner centres (kept from the last frame
that showed all 4: a robot hiding a corner does not lose it) -> for each robot marker: 4 corners to the arena,
parallax correction if the corner plane is not the marker plane, centre, heading (bottom-edge middle -> top-edge
middle), axle = centre - markerOffset * (cos h, sin h) -> publish
    {"bot": id, "x": m, "y": m, "heading": rad, "t": image Unix time (s), "age": publish time - t (s)}
t = grab time - P.cam.captureDelay (the webcam's exposure-to-grab delay, assumed); robots add P.link.netDelay to age.

Sources:
  --camera N  a webcam (needs opencv-contrib-python or opencv-python >= 4.7 with cv2.aruco)
  --video F   a recorded clip
  --sim       no camera: true poses from fake_bot.py --truth (topic zadp/sim/truth) are projected through a
              synthetic pinhole camera into marker corner pixels (+ noise) and run through the SAME geometry
              (run_real('dry')). Needs numpy only.
Geometry is in plain functions (numpy only), tested by test_aruco_geometry.py.

    python aruco_node.py --camera 0 --broker 192.168.1.10          # straight to the broker
    python aruco_node.py --camera 0 --bridge 127.0.0.1:18830       # through bridge.py
    python aruco_node.py --camera 0 --show                         # + preview window
"""
import argparse
import json
import math
import os
import sys
import threading
import time

try:
    import numpy as np
except ImportError:                      # lets the file be imported for a syntax check without numpy
    np = None
try:
    import cv2
    if not hasattr(cv2, 'aruco'):
        cv2 = None
except ImportError:
    cv2 = None

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(os.path.dirname(HERE), 'link_config.json')
TOPIC_TRUTH = 'zadp/sim/truth'


def load_config(path):
    with open(path) as f:
        return json.load(f)


# =============================================================================================== geometry (numpy)
def homography(src, dst):
    """3x3 H with dst ~ H * src (both N x 2, N >= 4), normalised DLT (Hartley)."""
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)

    def norm(p):
        c = p.mean(axis=0)
        s = math.sqrt(2) / max(np.sqrt(((p - c) ** 2).sum(axis=1)).mean(), 1e-12)
        return np.array([[s, 0, -s * c[0]], [0, s, -s * c[1]], [0, 0, 1]])
    Ts, Td = norm(src), norm(dst)
    a = (Ts @ np.c_[src, np.ones(len(src))].T).T
    b = (Td @ np.c_[dst, np.ones(len(dst))].T).T
    A = []
    for (x, y, w), (u, v, z) in zip(a, b):
        A.append([0, 0, 0, -z * x, -z * y, -z * w, v * x, v * y, v * w])
        A.append([z * x, z * y, z * w, 0, 0, 0, -u * x, -u * y, -u * w])
    _, _, Vt = np.linalg.svd(np.asarray(A))
    H = np.linalg.inv(Td) @ Vt[-1].reshape(3, 3) @ Ts
    return H / H[2, 2]


def apply_h(H, pts):
    """Map N x 2 points through H."""
    p = np.c_[np.asarray(pts, float).reshape(-1, 2), np.ones(len(np.asarray(pts).reshape(-1, 2)))]
    q = (H @ p.T).T
    return q[:, :2] / q[:, 2:3]


def marker_centre_px(c4):
    """Image centre of a marker from its 4 corners (TL, TR, BR, BL): intersection of the diagonals (exact under
    perspective, unlike the corner mean)."""
    c = np.asarray(c4, float).reshape(4, 2)
    p, r = c[0], c[2] - c[0]
    q, s = c[1], c[3] - c[1]
    den = r[0] * s[1] - r[1] * s[0]
    if abs(den) < 1e-12:
        return c.mean(axis=0)
    t = ((q[0] - p[0]) * s[1] - (q[1] - p[1]) * s[0]) / den
    return p + t * r


def parallax(pts, nadir, cam_h, plane_h, point_h):
    """Points mapped onto the plane at height plane_h (where the homography lives) that really lie at height
    point_h: the ray from the camera (above `nadir`, at cam_h) through the true point meets plane_h at p, so the
    true point is nadir + (p - nadir) * (cam_h - point_h) / (cam_h - plane_h)."""
    if abs(plane_h - point_h) < 1e-9:
        return np.asarray(pts, float)
    k = (cam_h - point_h) / (cam_h - plane_h)
    n = np.asarray(nadir, float).reshape(1, 2)
    return n + (np.asarray(pts, float) - n) * k


def robot_pose(H, c4, cam, nadir=None):
    """Axle pose (x, y, heading) of a robot marker. c4 = image corners TL, TR, BR, BL (ArUco order, printed up edge
    toward the nose); cam = config['cam']; nadir = arena point under the camera (only needed when the corner and
    marker planes differ)."""
    a = apply_h(H, c4)
    if abs(cam['cornerHeight'] - cam['markerHeight']) > 1e-9:
        if nadir is None:
            raise ValueError('nadir needed for the parallax correction')
        a = parallax(a, nadir, cam['cameraHeight'], cam['cornerHeight'], cam['markerHeight'])
    centre = a.mean(axis=0)                          # a square maps to a square on its own plane: mean = centre
    top, bottom = (a[0] + a[1]) / 2, (a[3] + a[2]) / 2
    h = math.atan2(top[1] - bottom[1], top[0] - bottom[0])
    off = cam['markerOffset']
    return centre[0] - off * math.cos(h), centre[1] - off * math.sin(h), h


def marker_side(H, c4):
    """Mean side length of a marker on the homography plane (m): a sanity check against P.cam.markerSize."""
    a = apply_h(H, c4)
    return float(np.mean([np.linalg.norm(a[i] - a[(i + 1) % 4]) for i in range(4)]))


class HomographyKeeper:
    """Image -> arena homography from the corner marker centres. Updated whenever all corners are visible; when
    one is hidden the last homography is kept (and the change is reported once)."""

    def __init__(self, cam, image_size=None):
        self.ids = [int(i) for i in cam['cornerIds']]
        self.world = {i: tuple(xy) for i, xy in zip(self.ids, cam['cornerXY'])}
        self.H = None
        self.seen = set()
        self.image_size = image_size
        self.n_updates = 0

    def update(self, det):
        """det: {id: 4x2 corners}. Returns a message string when the set of visible corners changed, else None."""
        seen = {i for i in self.ids if i in det}
        msg = None
        if seen != self.seen:
            missing = sorted(set(self.ids) - seen)
            if not missing:
                msg = 'corners %d/%d seen (ids %s)' % (len(seen), len(self.ids), self.ids)
            elif self.H is None:
                msg = 'corners %d/%d seen, missing ids %s: waiting for all of them' % (len(seen), len(self.ids), missing)
            else:
                msg = 'corners %d/%d seen, missing ids %s: keeping the last homography' % (len(seen), len(self.ids), missing)
            self.seen = seen
        if len(seen) == len(self.ids):
            src = [marker_centre_px(det[i]) for i in self.ids]
            dst = [self.world[i] for i in self.ids]
            self.H = homography(src, dst)
            self.n_updates += 1
        return msg

    def nadir(self):
        """Arena point under the camera, taken as the image of the image centre (camera roughly straight down)."""
        if self.H is None or self.image_size is None:
            return None
        w, h = self.image_size
        return apply_h(self.H, [[w / 2.0, h / 2.0]])[0]


def pose_message(bot, pose, t_img, t_pub):
    x, y, h = pose
    return {'bot': int(bot), 'x': round(float(x), 5), 'y': round(float(y), 5), 'heading': round(float(h), 5),
            't': round(float(t_img), 4), 'age': round(max(0.0, t_pub - t_img), 4)}


# =============================================================================================== synthetic camera
class PinholeCam:
    """Ideal pinhole camera above the arena (sim mode, tests). Position (cx, cy, height) in arena m, looking down
    with a small tilt; image x along arena +x, image y along arena -y (an upright view of the arena)."""

    def __init__(self, centre, height, f=700.0, size=(1280, 1024), tilt_deg=(0.0, 0.0), yaw_deg=0.0):
        self.size = size
        self.K = np.array([[f, 0, size[0] / 2.0], [0, f, size[1] / 2.0], [0, 0, 1]])
        self.C = np.array([centre[0], centre[1], height], float)
        # camera axes in the arena frame (looking down): x_c = +x, y_c = -y, z_c = -z, then small rotations
        R0 = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)
        ax, ay = np.radians(tilt_deg)
        az = math.radians(yaw_deg)
        Rx = np.array([[1, 0, 0], [0, math.cos(ax), -math.sin(ax)], [0, math.sin(ax), math.cos(ax)]])
        Ry = np.array([[math.cos(ay), 0, math.sin(ay)], [0, 1, 0], [-math.sin(ay), 0, math.cos(ay)]])
        Rz = np.array([[math.cos(az), -math.sin(az), 0], [math.sin(az), math.cos(az), 0], [0, 0, 1]])
        self.R = Rz @ Ry @ Rx @ R0                # world -> camera rotation

    def project(self, pts3):
        p = (self.R @ (np.asarray(pts3, float) - self.C).T)
        uv = self.K @ p
        return (uv[:2] / uv[2]).T


def marker_corners_3d(x, y, heading, size, height):
    """3D corners TL, TR, BR, BL of a face-up marker centred at (x, y) whose printed up edge points along heading."""
    u = np.array([math.cos(heading), math.sin(heading)])
    left = np.array([-math.sin(heading), math.cos(heading)])
    c = np.array([x, y])
    s = size / 2.0
    pts = [c + s * u + s * left, c + s * u - s * left, c - s * u - s * left, c - s * u + s * left]
    return np.array([[p[0], p[1], height] for p in pts])


def synth_detections(camera, cam, robots, noise_px=0.0, rng=None, hide=()):
    """Marker corner pixels as ArUco would report them. robots: {id: (x_axle, y_axle, heading)}."""
    det = {}
    for i, xy in zip(cam['cornerIds'], cam['cornerXY']):
        if int(i) in hide:
            continue
        det[int(i)] = camera.project(marker_corners_3d(xy[0], xy[1], 0.0, cam['cornerMarkerSize'], cam['cornerHeight']))
    off = cam['markerOffset']
    for i, (x, y, h) in robots.items():
        mx, my = x + off * math.cos(h), y + off * math.sin(h)
        det[int(i)] = camera.project(marker_corners_3d(mx, my, h, cam['markerSize'], cam['markerHeight']))
    if noise_px > 0:
        rng = rng or np.random.default_rng(0)
        det = {k: v + rng.normal(0, noise_px, v.shape) for k, v in det.items()}
    return det


def default_sim_camera(cam):
    """The synthetic camera of --sim: above the middle of the corner square, cam['cameraHeight'] up, tilted 1.5 deg."""
    xy = np.asarray(cam['cornerXY'], float)
    return PinholeCam(xy.mean(axis=0), cam['cameraHeight'], f=700.0, size=(1280, 1024), tilt_deg=(1.5, -1.0), yaw_deg=0.5)


# =============================================================================================== sources
class CameraSource:
    def __init__(self, args, cam):
        if cv2 is None:
            raise RuntimeError('OpenCV with cv2.aruco is not installed (pip install opencv-contrib-python)')
        src = args.video if args.video else args.camera
        backend = cv2.CAP_DSHOW if (os.name == 'nt' and not args.video) else cv2.CAP_ANY
        self.cap = cv2.VideoCapture(src, backend)
        if not args.video:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        if not self.cap.isOpened():
            raise RuntimeError('cannot open camera %s' % src)
        self.size = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cam['dictionary']))
        if hasattr(cv2.aruco, 'ArucoDetector'):
            prm = cv2.aruco.DetectorParameters()
            prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            det = cv2.aruco.ArucoDetector(dic, prm)
            self._detect = lambda g: det.detectMarkers(g)[:2]
        else:
            prm = cv2.aruco.DetectorParameters_create()
            prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self._detect = lambda g: cv2.aruco.detectMarkers(g, dic, parameters=prm)[:2]
        self.calib = None
        if args.calib:
            with open(args.calib) as f:
                cj = json.load(f)
            self.calib = (np.array(cj['camera_matrix'], float), np.array(cj['dist_coeffs'], float))
        self.frame = None
        self.capture_delay = cam['captureDelay']

    def read(self):
        ok, frame = self.cap.read()
        t = time.time() - self.capture_delay        # image time: grab time minus the assumed capture delay
        if not ok:
            return None, None
        self.frame = frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self._detect(gray)
        det = {}
        if ids is not None:
            for i, c in zip(ids.flatten(), corners):
                c = c.reshape(4, 2).astype(float)
                if self.calib is not None:            # lens undistortion (optional calibration file)
                    K, D = self.calib
                    c = cv2.undistortPoints(c.reshape(-1, 1, 2), K, D, P=K).reshape(4, 2)
                det[int(i)] = c
        return t, det

    def close(self):
        self.cap.release()


class SimSource:
    """True poses from fake_bot.py --truth -> synthetic marker pixels (same geometry downstream)."""

    def __init__(self, cam, args):
        self.cam = cam
        self.camera = default_sim_camera(cam)
        self.size = self.camera.size
        self.truth = {}
        self.lock = threading.Lock()
        self.rng = np.random.default_rng(args.sim_seed)
        self.noise = args.sim_noise_px
        self.hide = set(int(i) for i in args.sim_hide_corner.split(',') if i)
        self.period = 1.0 / cam['rateHz']
        self.next = time.monotonic()
        self.frame = None

    def on_truth(self, topic, p):
        if topic == TOPIC_TRUTH and isinstance(p, dict):
            with self.lock:
                self.truth[int(p['bot'])] = (float(p['x']), float(p['y']), float(p['heading']), float(p['t']))

    def read(self):
        self.next += self.period
        time.sleep(max(0.0, self.next - time.monotonic()))
        with self.lock:
            tr = dict(self.truth)
        robots = {k: v[:3] for k, v in tr.items()}
        t = max([v[3] for v in tr.values()], default=time.time())   # image time = time of the truth sample
        return t, synth_detections(self.camera, self.cam, robots, self.noise, self.rng, self.hide)

    def close(self):
        pass


# =============================================================================================== main
def make_link(args, cfg):
    sys.path.insert(0, HERE)
    if args.bridge:
        from transport import BridgeClient
        host, _, port = args.bridge.partition(':')
        return BridgeClient(host, int(port or cfg['link']['bridgePort']))
    from transport import PahoClient
    host, _, port = (args.broker or cfg['mqtt']['broker']).partition(':')
    return PahoClient(host, int(port or cfg['mqtt']['port']), client_id='zadp-camera')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', default=DEFAULT_CONFIG, help='link_config.json written by MATLAB (link_write_config)')
    ap.add_argument('--camera', type=int, default=0, help='camera index')
    ap.add_argument('--video', default='', help='video file instead of a camera')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--calib', default='', help='JSON {"camera_matrix": 3x3, "dist_coeffs": [...]} for lens undistortion')
    ap.add_argument('--broker', default='', help='host[:port] of the MQTT broker (default P.mqtt.broker)')
    ap.add_argument('--bridge', default='', help='host:port of bridge.py (instead of a broker)')
    ap.add_argument('--sim', action='store_true', help='simulated camera from zadp/sim/truth (no OpenCV needed)')
    ap.add_argument('--sim-noise-px', type=float, default=0.3, help='sim: corner pixel noise (std)')
    ap.add_argument('--sim-seed', type=int, default=1)
    ap.add_argument('--sim-hide-corner', default='', help='sim: corner ids to hide (comma list), to test corner loss')
    ap.add_argument('--robots', default=None, help='robot ids to publish (comma list, default P.cam.robotIds; '
                    'run_real gives the real robots only, so a simulated robot id is never published)')
    ap.add_argument('--show', action='store_true', help='preview window (camera only)')
    ap.add_argument('--log', default='', help='write output to this file (used when started in the background)')
    ap.add_argument('--ttl', type=float, default=0, help='exit by itself after this many seconds (0 = never)')
    args = ap.parse_args()
    if args.log:
        sys.stdout = sys.stderr = open(args.log, 'a', buffering=1)
    say = lambda s: print('aruco_node: ' + s, flush=True)
    if np is None:
        say('numpy is not installed (pip install numpy)')
        sys.exit(2)
    if not os.path.isfile(args.config):
        say('%s not found: run link_write_config(twin_params()) in MATLAB first' % args.config)
        sys.exit(2)
    cfg = load_config(args.config)
    cam = cfg['cam']
    topic = cfg['mqtt']['topicCamPose']
    robot_ids = set(int(i) for i in cam['robotIds'])
    if args.robots is not None:
        robot_ids &= set(int(i) for i in args.robots.split(',') if i.strip())
    t_end = time.monotonic() + args.ttl if args.ttl > 0 else None

    link = make_link(args, cfg)
    if args.sim:
        src = SimSource(cam, args)
        link.subscribe([TOPIC_TRUTH], src.on_truth)
    else:
        try:
            src = CameraSource(args, cam)
        except RuntimeError as e:
            say('ERROR ' + str(e))
            sys.exit(2)
    link.connect(10.0)
    say('source %s, image %dx%d, dictionary %s, robots %s, corners %s at %s, publishing %s via %s, pid=%d'
        % ('sim' if args.sim else ('video ' + args.video if args.video else 'camera %d' % args.camera), src.size[0],
           src.size[1], cam['dictionary'], sorted(robot_ids), cam['cornerIds'], cam['cornerXY'], topic,
           args.bridge or 'broker', os.getpid()))
    if abs(cam['cornerHeight'] - cam['markerHeight']) > 1e-9:
        say('corner plane %.3f m, marker plane %.3f m: parallax corrected with camera height %.2f m'
            % (cam['cornerHeight'], cam['markerHeight'], cam['cameraHeight']))
    keeper = HomographyKeeper(cam, src.size)
    n_frames, n_pub, t_stat, seen_bots, ready = 0, 0, time.monotonic(), set(), False
    while t_end is None or time.monotonic() < t_end:
        t_img, det = src.read()
        if det is None:
            say('camera / video ended')
            break
        n_frames += 1
        msg = keeper.update(det)
        if msg:
            say(msg)
        if keeper.H is not None and not ready:
            ready = True
            say('homography ready (4 corner markers); robot poses are published from now on')
        if keeper.H is not None:
            nadir = keeper.nadir()
            for i, c in det.items():
                if i not in robot_ids:
                    continue
                pose = robot_pose(keeper.H, c, cam, nadir)
                link.publish(topic, pose_message(i, pose, t_img, time.time()), 0)
                n_pub += 1
                seen_bots.add(i)
        if args.show and cv2 is not None and getattr(src, 'frame', None) is not None:
            f = src.frame
            for i, c in det.items():
                cv2.polylines(f, [c.astype(int).reshape(-1, 1, 2)], True, (0, 255, 0), 2)
                cv2.putText(f, str(i), tuple(int(v) for v in c[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow('aruco_node', f)
            if cv2.waitKey(1) & 0xFF == 27:
                break
        if time.monotonic() - t_stat >= 5.0:
            dt = time.monotonic() - t_stat
            say('%.1f frames/s, %.1f poses/s, robots seen %s, corners seen %s'
                % (n_frames / dt, n_pub / dt, sorted(seen_bots), sorted(keeper.seen)))
            n_frames, n_pub, t_stat, seen_bots = 0, 0, time.monotonic(), set()
    src.close()
    link.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
