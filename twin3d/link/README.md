# link: MATLAB <-> MQTT <-> robots + camera (phase 7)

The twin leads. The brain keeps running the simulated robots (proven collision-free). Whenever a robot's route step
changes, the link publishes that step to the real robot as one command (`goto`, `turn`, `dump`, `stop`). The robot
drives it with its own controller (the same laws as the twin) and reports telemetry; the overhead ArUco camera reports
its true pose. The link logs twin-vs-real error; with `syncToReal` the twin waits at the end of every step until the
real robot reports `arrived`. Contract (single source for topics, fields, semantics, timing): `TOPICS.md`.

```
 MATLAB brain --TCP/JSON--> bridge.py --MQTT--> broker --MQTT--> ESP32 x3 (firmware/zadp_bot)  | fake_bot.py (dry/test)
 (twin3d)     <-- heartbeat ->  (repeats cmds 10 Hz)  <--------  aruco_node.py (webcam)         | aruco_node.py --sim
```
Numbers: `params/twin_params.m` -> `link_write_config` -> `link_config.json`, read by bridge.py, fake_bot.py,
aruco_node.py; `py/check_numbers.py` compares the firmware's `config.h` with it.

## Real robots: one command
```matlab
run('E:\catrover_dumpautonomy\twin3d\startup.m')
R = run_real();                               % or run_real('real', struct('maxTime', 120, 'camArgs', '--camera 1'))
```
`run_real` writes `link_config.json`, checks Python / numpy / paho-mqtt / OpenCV (cv2.aruco) / the broker, starts
`bridge.py` (broker mode, no fallback) and `aruco_node.py`, waits until the camera sees the 4 corner markers, waits for
telemetry and a camera pose from robots 1-3, checks robot i stands on parking slot P_i (30 mm, 15 deg; the message
names any robot that does not), then runs `brain_run_linked` with `syncToReal` and a live 3D view drawn at the camera
poses. An error, Ctrl+C or closing the view sends stop to all robots (twice) and kills every helper process.
Everything it prints is one line per check. Launch order and broker set-up: `../HANDOVER.md` section 14.

`run_real('dry')` runs the same code path without hardware: loopback bridge, `fake_bot.py` robots (no camera of their
own, `--truth`), `aruco_node.py --sim` (true poses projected through a synthetic pinhole camera into marker corner pixels
+ 0.3 px noise, then the real homography / pose code). Tested by `tests/test_run_real`.

## Tests and demo (no hardware)
```matlab
test_link          % loopback bridge + fake robots: contract fields, watchdog, keep-alive across a stall, linked runs
test_run_real      % run_real('dry'), plus a robot off its slot and a hidden corner (both must fail with a clear message)
test_firmware      % config.h vs twin_params, motion_core JS mirror (node), camera geometry (numpy)
demo_link
```
Tests ALWAYS use loopback (in broker mode they would publish on the real robot topics).

## Files
| File | What |
|---|---|
| `TOPICS.md` | the contract |
| `run_real.m` | one-command launcher (real / dry) |
| `link_write_config.m` | twin_params -> `link_config.json` |
| `py/bridge.py` | localhost TCP <-> broker (paho-mqtt) or in-process loopback router; repeats each robot's last command at `P.link.repeatHz` while MATLAB was heard within `P.link.hostTimeout` |
| `py/fake_bot.py` | reference robot: the firmware's command semantics and controller on the twin's motor model; optional camera / truth publisher |
| `py/aruco_node.py` | camera node: DICT_4X4_50, robots 1-3, corners 10-13, homography, parallax, axle offset, `t` + `age`; `--sim` mode |
| `py/test_aruco_geometry.py` | camera geometry on synthetic pixels (numpy only) |
| `py/check_numbers.py` | firmware `config.h` vs `link_config.json` |
| `py/transport.py` | the two clients (TCP to the bridge, paho to a broker) |
| `markers/make_markers.py` | printable markers at true size (needs OpenCV); output `robots_A4.pdf`, `corners_A4.pdf`, `placement.txt` |
| `link_open`, `link_send_cmd`, `link_poll`, `link_close` | MATLAB side, base MATLAB only (`tcpclient`, `jsonencode`, `jsondecode`) |
| `link_keepalive`, `link_stop_all` | heartbeat to the bridge, stop everybody |
| `link_from_brain(W, Wprev, L)` | route-step changes -> commands (drive pieces of one drive = one `goto`; turns carry `dir` and `tol`) |
| `brain_run_linked(plan, P, opts)` | `brain_run` with the link, paced to real time; `syncToReal`, `onStep` (live view), `abortFcn`; ends on a robot `offline` / silent |
| `link_launch`, `link_kill`, `link_python` | start / stop the helper processes (PID checked), find Python |

## Timing in one line
Bridge repeats at 10 Hz while MATLAB was heard within 1.0 s; robot watchdog 0.3 s. A 0.4-0.6 s brain stall never
stops a robot; a closed MATLAB stops them in 0.3 s, a hung one in 1.3 s (`TOPICS.md` "Timing").

## Camera node
Markers: robot id = robot number, 60 mm, centred on the robot top (26 mm ahead of the axle), printed arrow (marker
up edge) to the nose. Corner ids 10-13 at `P.cam.cornerXY`, outside the driven area, on blocks so their plane equals
the robot marker plane (`P.cam.cornerHeight = P.cam.markerHeight`, 50 mm: measure your robot and set both); if the
corners lie on the floor set `cornerHeight = 0` and `cameraHeight` (parallax is then corrected about the image-centre
nadir). Optional lens calibration: `--calib file.json` (`camera_matrix`, `dist_coeffs`). Run by hand:
`python link\py\aruco_node.py --camera 0 --broker <ip> --show`.
Not run against a real camera here (OpenCV not installed); the geometry is tested on synthetic pixels.
