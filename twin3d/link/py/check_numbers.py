"""check_numbers.py: the firmware's numbers must be the twin's numbers.

Compares firmware/zadp_bot/include/config.h (#define values) with link_config.json, which MATLAB writes from
params/twin_params.m (link_write_config). Prints one table row per number (config.h name, value, twin field,
value) and exits 1 on any mismatch. Also checks the firmware-only constants that fake_bot.py copies.

    python check_numbers.py [--config ../link_config.json] [--header ../../../firmware/zadp_bot/include/config.h]
"""
import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
DEFAULT_CONFIG = os.path.join(os.path.dirname(HERE), 'link_config.json')
DEFAULT_HEADER = os.path.join(ROOT, 'firmware', 'zadp_bot', 'include', 'config.h')

# config.h name -> (path in link_config.json, relative tolerance)
TABLE = [
    ('WHEEL_DIAMETER_M', 'bot.wheelDiameter', 1e-6),
    ('TRACK_M', 'bot.track', 1e-6),
    ('TRACK_SCALE', 'bot.trackScale', 1e-6),
    ('MAX_WHEEL_SPEED', 'bot.maxWheelSpeed', 1e-4),
    ('CRUISE_SPEED', 'bot.cruiseSpeed', 1e-6),
    ('MAX_ACCEL', 'bot.maxAccel', 1e-6),
    ('MOTOR_TAU_S', 'bot.motorTimeConstant', 1e-6),
    ('KP_HEADING', 'ctrl.kpHeading', 1e-6),
    ('KP_DISTANCE', 'ctrl.kpDistance', 1e-6),
    ('ROTATE_THRESH_DEG', 'ctrl.rotateThreshDeg', 1e-6),
    ('ROTATE_EXIT_DEG', 'ctrl.rotateExitDeg', 1e-6),
    ('CAPTURE_RADIUS_M', 'ctrl.captureRadius', 1e-6),
    ('DECEL_FRAC', 'ctrl.decelFrac', 1e-6),
    ('ARRIVE_TOL_M', 'ctrl.arriveTol', 1e-6),
    ('HEADING_TOL_DEG', 'brain.turnTolDeg', 1e-6),
    ('MIN_TURN_RATE', 'brain.minTurnRate', 1e-6),
    ('SLIDE_DIST_M', 'brain.slideDist', 1e-6),
    ('LOOKAHEAD_M', 'brain.lookahead', 1e-6),
    ('DUMP_TIME_S', 'truck.dumpTime', 1e-6),
    ('WATCHDOG_S', 'link.watchdog', 1e-6),
    ('TELEMETRY_HZ', 'link.telemetryHz', 1e-6),
    ('CAM_FUSION_GAIN', 'cam.fusionGain', 1e-6),
    ('CAM_NET_DELAY_S', 'link.netDelay', 1e-6),
    ('CAM_DEFAULT_LAG_S', 'cam.latency', 1e-6),
    ('CAM_MAX_AGE_S', 'cam.maxAge', 1e-6),
    ('MQTT_PORT', 'mqtt.port', 0),
    ('INIT_Y', 'brain.parking[*][1]', 1e-6),
    ('INIT_HEADING', 'brain.parking[*][2]', 1e-6),
]
# firmware-only constants that fake_bot.py (Num) copies: config.h name -> fake_bot value
FW_ONLY = [('MIN_GOTO_SPEED', 0.02), ('TURN_WRAP_SLACK', 0.3), ('STOPPED_SPEED', 2e-3)]


def parse_header(path):
    """#define NAME value -> {NAME: [values in file order]} (numeric values only; f suffix and parentheses dropped)."""
    out = {}
    for line in open(path, encoding='utf-8'):
        m = re.match(r'\s*#define\s+(\w+)\s+\(?\s*(-?[0-9.]+(?:[eE][-+]?\d+)?)f?\s*\)?\s*(//.*)?$', line)
        if m:
            out.setdefault(m.group(1), []).append(float(m.group(2)))
    return out


def get(C, path):
    m = re.match(r'(\w+)\.(\w+)(\[\*\]\[(\d)\])?$', path)
    v = C[m.group(1)][m.group(2)]
    if m.group(3):
        return [row[int(m.group(4))] for row in v]
    return v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=DEFAULT_CONFIG)
    ap.add_argument('--header', default=DEFAULT_HEADER)
    a = ap.parse_args()
    C = json.load(open(a.config))
    D = parse_header(a.header)
    bad = 0
    print('%-18s %-12s %-24s %-12s %s' % ('config.h', 'value', 'twin_params', 'value', 'ok'))

    def row(name, fv, tname, tv, tol):
        nonlocal bad
        ok = abs(fv - tv) <= tol * max(1.0, abs(tv)) if tol else fv == tv
        bad += not ok
        print('%-18s %-12.6g %-24s %-12.6g %s' % (name, fv, tname, tv, 'ok' if ok else 'MISMATCH'))

    for name, path, tol in TABLE:
        if name not in D:
            print('%-18s missing in config.h' % name)
            bad += 1
            continue
        tv = get(C, path)
        if isinstance(tv, list):                       # same value for every robot (INIT_Y, INIT_HEADING)
            for k, t in enumerate(tv):
                row(name, D[name][0], path.replace('*', str(k)), t, tol)
        else:
            row(name, D[name][0], path, tv, tol)
    # INIT_X: one #define per BOT_ID branch, in order 1, 2, 3
    for k, (fv, slot) in enumerate(zip(D.get('INIT_X', []), C['brain']['parking'])):
        row('INIT_X (bot %d)' % (k + 1), fv, 'brain.parking[%d][0]' % k, slot[0], 1e-6)
    # per-wheel CPR (calibrated per robot): the mean must stay within 0.5 % of the twin's encoderCPR
    cpr = (D['CPR_LEFT'][0] + D['CPR_RIGHT'][0]) / 2
    row('CPR mean', cpr, 'bot.encoderCPR', C['bot']['encoderCPR'], 0.005)
    for name, v in FW_ONLY:
        row(name, D[name][0], 'fake_bot.py', v, 1e-9)
    # topics
    h = open(a.header, encoding='utf-8').read()
    for fmt, key in (('"zadp/bot" ZADP_STR(BOT_ID) "/cmd"', 'topicCmd'), ('"zadp/bot" ZADP_STR(BOT_ID) "/telemetry"', 'topicTelemetry')):
        ok = fmt in h and C['mqtt'][key] == fmt.replace('" ZADP_STR(BOT_ID) "', '%d').strip('"')
        bad += not ok
        print('%-18s %-12s %-24s %-12s %s' % (key, '', 'mqtt.' + key, C['mqtt'][key], 'ok' if ok else 'MISMATCH'))
    ok = '"%s"' % C['mqtt']['topicCamPose'] in h
    bad += not ok
    print('%-18s %-12s %-24s %-12s %s' % ('topicCamPose', '', 'mqtt.topicCamPose', C['mqtt']['topicCamPose'], 'ok' if ok else 'MISMATCH'))
    print('\ncheck_numbers: %s (%d mismatches)' % ('ALL MATCH' if bad == 0 else 'MISMATCH', bad))
    return 0 if bad == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
