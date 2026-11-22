"""test_aruco_geometry.py: the camera node's geometry without a camera (numpy only; OpenCV is not needed).

Synthetic pinhole camera (1280 x 1024, f 700 px, 1.2 m up, tilted 1.5 / -1.0 deg, yawed 0.5 deg) over the arena;
marker corners projected exactly as ArUco reports them (TL, TR, BR, BL) + 0.3 px noise; aruco_node.robot_pose must
recover the AXLE pose. Cases: corner plane = marker plane (the recommended set-up), corners on the floor (parallax
corrected with the camera height and the image-centre nadir), a hidden corner (last homography kept), and a
marker printed the other way round would give heading + pi (checked so a wrong orientation is visible).
Numbers from link_config.json (P.cam). Prints PASS/FAIL lines, exit code 0 = all passed.

    python test_aruco_geometry.py [--config ../link_config.json]
"""
import argparse
import copy
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import aruco_node as an  # noqa: E402

n_pass = n_fail = 0


def check(ok, what):
    global n_pass, n_fail
    n_pass += bool(ok)
    n_fail += not ok
    print('  [%s] %s' % ('PASS' if ok else 'FAIL', what))


def cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def poses(rng, n):
    """Random axle poses over the site and the apron (x 0.05..0.95, y -0.2..0.95)."""
    return [(rng.uniform(0.05, 0.95), rng.uniform(-0.2, 0.95), rng.uniform(-math.pi, math.pi)) for _ in range(n)]


def run_case(cam, camera, rng, n=300, noise=0.3, hide=()):
    keeper = an.HomographyKeeper(cam, camera.size)
    keeper.update(an.synth_detections(camera, cam, {}, noise, rng))     # all corners once
    ep, eh = [], []
    for k, p in enumerate(poses(rng, n)):
        det = an.synth_detections(camera, cam, {1: p}, noise, rng, hide)
        keeper.update(det)
        x, y, h = an.robot_pose(keeper.H, det[1], cam, keeper.nadir())
        ep.append(math.hypot(x - p[0], y - p[1]))
        eh.append(abs(math.degrees(wrap(h - p[2]))))
    return np.array(ep), np.array(eh), keeper


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=an.DEFAULT_CONFIG)
    a = ap.parse_args()
    cfg = an.load_config(a.config)
    cam = cfg['cam']
    camera = an.default_sim_camera(cam)
    rng = np.random.default_rng(7)
    print('aruco geometry test: camera %.2f m up, marker %.0f mm at %.3f m, corners at %.3f m, offset %.3f m'
          % (cam['cameraHeight'], 1000 * cam['markerSize'], cam['markerHeight'], cam['cornerHeight'], cam['markerOffset']))

    # 0. the homography itself: exact data, exact answer
    src = rng.uniform(0, 1000, (6, 2))
    Ht = np.array([[0.001, 0.0002, -0.1], [-0.0001, -0.0011, 1.0], [1e-6, 2e-6, 1.0]])
    Hs = an.homography(src, an.apply_h(Ht, src))
    check(np.allclose(an.apply_h(Hs, src), an.apply_h(Ht, src), atol=1e-9), 'DLT homography reproduces an exact map')
    c4 = np.array([[0, 0], [10, 1], [11, 12], [-1, 9]], float)
    m = an.marker_centre_px(c4)
    check(abs(cross2(c4[2] - c4[0], m - c4[0])) < 1e-9 and abs(cross2(c4[3] - c4[1], m - c4[1])) < 1e-9,
          'marker centre = intersection of the diagonals')

    # 1. recommended set-up: corner plane = marker plane, 0.3 px noise
    ep, eh, keeper = run_case(cam, camera, rng)
    rp, rh = math.sqrt((ep ** 2).mean()), math.sqrt((eh ** 2).mean())
    print('  same plane, 0.3 px noise, 300 poses: position error rms %.2f mm, max %.2f mm; heading rms %.2f deg, max %.2f deg'
          % (1000 * rp, 1000 * ep.max(), rh, eh.max()))
    # P.cam.posStd / headingStdDeg are the noise STD the twin assumes (3 mm, 1 deg): the node must not be noisier
    check(rp <= cam['posStd'] and rh <= cam['headingStdDeg'],
          'pose noise within the twin camera model (rms <= P.cam.posStd %.0f mm, P.cam.headingStdDeg %.0f deg)'
          % (1000 * cam['posStd'], cam['headingStdDeg']))
    ep0, eh0, keeper = run_case(cam, camera, rng, n=50, noise=0.0)
    check(ep0.max() < 1e-6 and eh0.max() < 1e-4, 'noise-free: exact (%.1e m, %.1e deg)' % (ep0.max(), eh0.max()))
    det = an.synth_detections(camera, cam, {2: (0.5, 0.5, 0.3)}, 0.0)
    side = an.marker_side(keeper.H, det[2])
    keeper_exact = keeper
    check(abs(side - cam['markerSize']) < 1e-6, 'marker side on the plane = P.cam.markerSize (%.4f m)' % side)

    # 2. corners on the floor: parallax correction (camera height + image-centre nadir; the tilt makes the nadir
    #    estimate approximate)
    camF = copy.deepcopy(cam)
    camF['cornerHeight'] = 0.0
    camera_floor = an.default_sim_camera(camF)
    ep, eh, _ = run_case(camF, camera_floor, rng)
    camN = copy.deepcopy(camF)
    camN['cameraHeight'] = 1e9                         # disables the correction (k -> 1)
    keeperN = an.HomographyKeeper(camF, camera_floor.size)
    keeperN.update(an.synth_detections(camera_floor, camF, {}, 0.0))
    epn = []
    for p in poses(rng, 100):
        d = an.synth_detections(camera_floor, camF, {1: p}, 0.0)
        x, y, _ = an.robot_pose(keeperN.H, d[1], camN, keeperN.nadir())
        epn.append(math.hypot(x - p[0], y - p[1]))
    print('  corners on the floor, markers %.3f m up: corrected error max %.2f mm (uncorrected %.1f mm)'
          % (cam['markerHeight'], 1000 * ep.max(), 1000 * max(epn)))
    check(ep.max() < 0.004 and max(epn) > 3 * ep.max(), 'parallax correction: within 4 mm (tilted camera)')

    # 3. a corner hidden after the start: the last homography is kept and the poses stay right
    keeper = an.HomographyKeeper(cam, camera.size)
    keeper.update(an.synth_detections(camera, cam, {}, 0.3, rng))
    H0 = keeper.H.copy()
    msg = keeper.update(an.synth_detections(camera, cam, {1: (0.5, 0.5, 0)}, 0.3, rng, hide={11}))
    p = (0.3, 0.7, 1.0)
    d = an.synth_detections(camera, cam, {1: p}, 0.3, rng, hide={11})
    keeper.update(d)
    x, y, h = an.robot_pose(keeper.H, d[1], cam)
    print('  corner 11 hidden: "%s"; pose error %.2f mm' % (msg, 1000 * math.hypot(x - p[0], y - p[1])))
    check(np.array_equal(H0, keeper.H) and 'keeping the last homography' in (msg or '') and math.hypot(x - p[0], y - p[1]) < 0.003,
          'a hidden corner keeps the last homography')
    k2 = an.HomographyKeeper(cam, camera.size)
    m2 = k2.update(an.synth_detections(camera, cam, {}, 0.0, hide={12}))
    check(k2.H is None and 'waiting' in m2, 'no homography before all corners were seen once ("%s")' % m2)

    # 4. message format
    msg = an.pose_message(2, (0.123456, -0.1, 3.0), 1795000000.0, 1795000000.045)
    check(set(msg) == {'bot', 'x', 'y', 'heading', 't', 'age'} and abs(msg['age'] - 0.045) < 1e-9, 'message fields {bot,x,y,heading,t,age}')

    # 5. a marker stuck on backwards shows up as heading + pi (not silently right)
    det = an.synth_detections(camera, cam, {1: (0.5, 0.4, 0.2)}, 0.0)
    rev = det[1][[2, 3, 0, 1]]
    _, _, hr = an.robot_pose(keeper_exact.H, rev, cam)
    check(abs(abs(wrap(hr - 0.2)) - math.pi) < 1e-6, 'marker rotated 180 deg gives heading + 180 deg')
    print('\n%d passed, %d failed' % (n_pass, n_fail))
    return 0 if n_fail == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
