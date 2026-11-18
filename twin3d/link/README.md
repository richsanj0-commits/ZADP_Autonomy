# link: MATLAB <-> MQTT <-> robots (phase 7)

The twin leads. The brain keeps running the simulated robots (proven collision-free). Whenever a robot's current
route step changes, the link publishes that step to the real robot as a command (`goto`, `turn`, `dump`, `stop`). The
real robot executes it with its own encoder controller and reports telemetry; the overhead camera reports its true pose.
The link records the position and heading difference between every real robot and its twin, and with
`opts.syncToReal = true` it holds the twin at the end of a step until the real robot reports `arrived`.

Everything up to (not including) real hardware is built and tested against `fake_bot.py`, a stand-in for the ESP32s.
Message contract and the ESP32 firmware checklist: `TOPICS.md`.

```
 MATLAB brain --TCP/JSON--> bridge.py --MQTT--> broker --MQTT--> ESP32 x3 (real)   /  fake_bot.py (test)
 (twin3d)     <-------------            <--------        <--------  aruco_node.py (camera)  /  fake_bot.py (test)
```
In `--loopback` mode the bridge is its own broker and `fake_bot.py` attaches to the bridge; nothing else changes.

## Run without hardware (what the tests do)
```matlab
run('E:\catrover_dumpautonomy\twin3d\startup.m')
demo_link                 % 120 s of sim time = 120 s of wall time, saves link\demo_link.png
test_link                 % or run_all_tests
```
Needs only Python 3 (standard library) on the PATH. `paho-mqtt` and a broker are used automatically if both are present,
otherwise the bridge falls back to loopback (`link_launch` reports the mode).

## Files
| File | What |
|---|---|
| `py/bridge.py` | localhost TCP (newline-delimited JSON `{"topic":..,"payload":{..}}`, `{"subscribe":[..]}`) <-> MQTT broker (paho-mqtt) or in-process loopback router |
| `py/fake_bot.py` | stand-in for 1..3 ESP32 robots with the real numbers (wheel 34 mm, track 75.3 mm, 2772.2 counts/rev, 300 rpm, motor lag, go-to-point + turn controller, 20 Hz telemetry) and the camera node (30 Hz pose, 3 mm / 1 deg noise) |
| `py/transport.py` | the two clients both use (TCP to the bridge, paho to a broker) |
| `py/aruco_node.py` | OpenCV camera node, **not run against a camera** (below) |
| `link_open`, `link_send_cmd`, `link_poll`, `link_close` | MATLAB side, base MATLAB only (`tcpclient`, `jsonencode`, `jsondecode`) |
| `link_keepalive`, `link_stop_all` | repeat the last command every 100 ms (watchdog), stop everybody |
| `link_from_brain(W, Wprev, L)` | route-step changes -> commands (drive pieces of one drive = one `goto`) |
| `brain_run_linked(plan, P, opts)` | `brain_run` with the link attached, paced to real time; logs twin pose, real pose, error, latency; honours `opts.syncToReal` |
| `link_launch`, `link_kill`, `link_python` | start / stop the background processes (PID checked), find Python |
| `demo_link.m`, `../tests/test_link.m` | demo and test |

Hooks added to the brain (no change when unused): `brain_run` options `beforeStep` / `afterStep`, `W.hold` in
`brain_step` (`brain_init` sets it to false).

### syncToReal and stalls
* `syncToReal = false`: the real robots simply follow; the log shows how far they drift from the twin.
* `syncToReal = true`: at the end of every drive/turn step and at the end of the dump the twin robot waits until the real one
  reports `arrived` for that command's `seq`. Release after `holdTimeout` (20 s, counted in `lk.holdTimeouts`).
* The run is paced to the wall clock. A slow brain step (the planning at the start takes about 1 s) is absorbed by moving
  the clock, never caught up faster than real time. If the link is silent for 300 ms the real robots stop (watchdog)
  and continue when the commands come back.

## Real broker and real robots (later)
You run these yourself; the repository never installs anything.
```
pip install paho-mqtt                       # MQTT client for bridge.py and aruco_node.py
pip install opencv-python numpy             # only for the camera node
```
Broker: install Mosquitto (https://mosquitto.org/download/), allow LAN clients in `mosquitto.conf`
(`listener 1883` and `allow_anonymous true` for a closed test network), start `mosquitto -v`.

1. Edit `params/twin_params.m`: `P.mqtt.broker = '192.168.x.y'` (the PC running the broker; `'localhost'` if the broker is
   on the same PC as MATLAB), `P.mqtt.port = 1883`. The ESP32 firmware needs the same address.
2. Start the bridge (no fallback, so a missing broker is an error instead of silent loopback):
   `python link\py\bridge.py --broker 192.168.x.y --broker-port 1883 --no-fallback --port 18830`
3. Start the camera node (see below), mount marker ids 1, 2, 3 on the robots.
4. Power the robots (firmware per the checklist in `TOPICS.md`; the current firmware speaks TCP port 23 and needs the MQTT client first).
5. In MATLAB: `L = link_open(P, struct('port', 18830)); [W, log, lk] = brain_run_linked(plan, P, struct('link', L, 'maxTime', 120, 'syncToReal', true)); link_close(L);`
   Place the robots at the parking slots `P.brain.parking` first (the twin starts there).
   Try without `syncToReal` first, with one robot, at low speed.
6. Test the bridge/broker side with the fake robots first: `python link\py\fake_bot.py --bots 1,2,3 --broker localhost`
   (with `--no-cam` once the real camera node runs).

## ArUco camera node: NOT TESTED
`py/aruco_node.py` is complete but has **never been run against a camera** (no camera, OpenCV not installed here). It passes
`python -m py_compile` and imports without OpenCV; the rest is written from the OpenCV 4.x documentation. Expect to tune it.

* Dictionary `DICT_4X4_50`. Robot markers ids 1-3 flat on top, centred over the axle, the marker's up edge toward the nose.
* Corner markers ids 10, 11, 12, 13 at the arena corners (10 = (0,0), 11 = (1,0), 12 = (1,1), 13 = (0,1); `--corner-inset`
  if their centres are inside the arena). All four must be visible once; the homography is then kept.
* Run: `python link\py\aruco_node.py --camera 0 --broker localhost --show` (or `--bridge 127.0.0.1:18830`). Camera straight above
  the arena, whole arena + apron in view, fixed exposure, markers at least 25 mm printed with white border.
* Publishes `zadp/cam/pose` `{bot, x, y, heading, t}`. The 50 ms camera latency is not compensated.
* Not done: lens undistortion, apron region (y < 0) is outside the 4 corner markers but the homography extrapolates; add
  markers if its accuracy is poor.
