"""aruco_node.py: overhead ArUco camera node, publishes zadp/cam/pose.

*** NOT RUN AGAINST A CAMERA. *** Written for OpenCV 4.x (cv2.aruco) and checked only for syntax
(python -m py_compile aruco_node.py). Expect to adjust camera index, exposure, marker size and mounting.

What it does
  1. Grabs frames from a webcam looking straight down on the 1 m x 1 m arena.
  2. Detects ArUco markers, dictionary DICT_4X4_50:
       ids 1, 2, 3     the robots (marker flat on top, centred over the axle, the marker's "up" edge pointing
                       to the robot's NOSE, i.e. the direction of travel)
       ids 10, 11, 12, 13   four fixed markers at the arena corners, centre positions in arena metres:
                       10 = (0, 0) lower-left, 11 = (1, 0) lower-right, 12 = (1, 1) upper-right, 13 = (0, 1) upper-left
                       (arena frame: origin lower-left, x right, y up; --corner-inset moves them in from the corners)
  3. Computes the image -> arena homography from the corner markers (cv2.findHomography, kept from the last
     frame in which at least 4 corner markers were visible, so a robot hiding a corner does not lose it).
  4. For every robot marker found: x, y = homography(marker centre); heading = direction of the homography images of
     (centre -> middle of the marker's top edge), counter-clockwise from +x, radians.
  5. Publishes {"bot": id, "x": m, "y": m, "heading": rad, "t": unix seconds} on zadp/cam/pose (QoS 0).

Run (after installing paho-mqtt and opencv-python yourself, see README.md):
    python aruco_node.py --camera 0 --broker localhost            # straight to the broker
    python aruco_node.py --camera 0 --bridge 127.0.0.1:18830      # through bridge.py
    python aruco_node.py --camera 0 --show                         # preview window with the detected markers
    python aruco_node.py --video test.mp4 --show                   # a recorded clip instead of a camera
"""
import argparse
import math
import sys
import time

try:
    import cv2
    import numpy as np
except ImportError:                      # lets the file be syntax-checked / imported without OpenCV
    cv2 = None
    np = None

ROBOT_IDS = (1, 2, 3)
CORNER_IDS = (10, 11, 12, 13)
TOPIC = 'zadp/cam/pose'


def make_detector():
    """cv2.aruco.ArucoDetector (OpenCV >= 4.7) or the old functional API; returns detect(gray) -> (corners, ids)."""
    aruco = cv2.aruco
    dic = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    if hasattr(aruco, 'ArucoDetector'):
        det = aruco.ArucoDetector(dic, aruco.DetectorParameters())

        def detect(gray):
            corners, ids, _ = det.detectMarkers(gray)
            return corners, ids
    else:
        params = aruco.DetectorParameters_create()

        def detect(gray):
            corners, ids, _ = aruco.detectMarkers(gray, dic, parameters=params)
            return corners, ids
    return detect


def corner_points(inset):
    """Arena positions (m) of the corner markers' centres, by id."""
    a, b = inset, 1.0 - inset
    return {10: (a, a), 11: (b, a), 12: (b, b), 13: (a, b)}


def marker_centre_and_top(c):
    """c: 4x2 image corners of one marker (order: top-left, top-right, bottom-right, bottom-left of the marker).
    Returns the centre and the middle of the top edge, image pixels."""
    c = c.reshape(4, 2)
    return c.mean(axis=0), (c[0] + c[1]) / 2.0


def to_arena(H, pt):
    v = H @ np.array([pt[0], pt[1], 1.0])
    return v[0] / v[2], v[1] / v[2]


def robot_pose(H, c, axle_offset=0.0):
    """Arena pose (x, y, heading) of a robot marker from its image corners and the homography."""
    centre, top = marker_centre_and_top(c)
    x, y = to_arena(H, centre)
    tx, ty = to_arena(H, top)
    heading = math.atan2(ty - y, tx - x)
    if axle_offset:                        # marker not over the axle: move the reference point back along the heading
        x -= axle_offset * math.cos(heading)
        y -= axle_offset * math.sin(heading)
    return x, y, heading


def update_homography(H, corners, ids, inset):
    """New homography when all four corner markers are visible, else the old one."""
    if ids is None:
        return H
    seen = {int(i): c for i, c in zip(ids.flatten(), corners)}
    if not all(k in seen for k in CORNER_IDS):
        return H
    world = corner_points(inset)
    src = np.array([marker_centre_and_top(seen[k])[0] for k in CORNER_IDS], dtype=np.float32)
    dst = np.array([world[k] for k in CORNER_IDS], dtype=np.float32)
    Hn, _ = cv2.findHomography(src, dst)
    return Hn if Hn is not None else H


def make_publisher(args):
    sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
    if args.bridge:
        from transport import BridgeClient
        host, _, port = args.bridge.partition(':')
        link = BridgeClient(host, int(port or 18830))
    else:
        from transport import PahoClient
        host, _, port = args.broker.partition(':')
        link = PahoClient(host, int(port or 1883), client_id='zadp-camera')
    link.connect(10.0)
    return link


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--camera', type=int, default=0, help='camera index')
    ap.add_argument('--video', default='', help='video file instead of a camera')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--broker', default='localhost', help='host[:port] of the MQTT broker')
    ap.add_argument('--bridge', default='', help='host:port of bridge.py (instead of a broker)')
    ap.add_argument('--corner-inset', type=float, default=0.0, help='m, corner marker centres this far inside the arena corners')
    ap.add_argument('--axle-offset', type=float, default=0.0, help='m, robot marker centre ahead of the axle')
    ap.add_argument('--rate', type=float, default=30.0, help='max messages per second per robot')
    ap.add_argument('--show', action='store_true', help='preview window')
    args = ap.parse_args()
    if cv2 is None:
        print('aruco_node: OpenCV (pip install opencv-python numpy) is not installed', file=sys.stderr)
        sys.exit(2)

    cap = cv2.VideoCapture(args.video if args.video else args.camera)
    if not args.video:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print('aruco_node: cannot open the camera / video', file=sys.stderr)
        sys.exit(2)
    detect = make_detector()
    link = make_publisher(args)
    H = None
    last_pub = {}
    warned = False
    while True:
        ok, frame = cap.read()
        t = time.time()                    # the camera latency (about 50 ms) is not subtracted
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = detect(gray)
        H = update_homography(H, corners, ids, args.corner_inset)
        if H is None:
            if not warned:
                print('aruco_node: waiting for the 4 corner markers (ids 10-13)', file=sys.stderr)
                warned = True
        elif ids is not None:
            for i, c in zip(ids.flatten(), corners):
                i = int(i)
                if i not in ROBOT_IDS or t - last_pub.get(i, 0) < 1.0 / args.rate:
                    continue
                x, y, h = robot_pose(H, c, args.axle_offset)
                link.publish(TOPIC, {'bot': i, 'x': round(x, 4), 'y': round(y, 4), 'heading': round(h, 4), 't': round(t, 3)}, 0)
                last_pub[i] = t
        if args.show:
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.imshow('aruco_node', frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
    cap.release()
    link.close()


if __name__ == '__main__':
    main()
