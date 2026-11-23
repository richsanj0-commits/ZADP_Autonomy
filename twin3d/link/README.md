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
R = run_real();                                   % all 3 robots real
R = run_real('real', struct('realBots', 1));      % robot 1 real, robots 2 and 3 simulated (also [1 2])
```
`run_real` writes `link_config.json`, checks Python / numpy / paho-mqtt / OpenCV (cv2.aruco) / the broker, starts
the broker if needed (see "Broker"), `bridge.py` (broker mode, no fallback), `aruco_node.py --robots <realBots>` and,
for the robots not in `realBots`, `fake_bot.py` on the same broker (own camera pose, refuses real ids: `--real`).
It waits until the camera sees the 4 corner markers, waits for telemetry and a camera pose from every robot, checks
each REAL robot i stands on parking slot P_i (`P.link.parkTol`, `P.link.parkTolDeg`; the message names any that does
not), then runs `brain_run_linked` with `syncToReal` and a live 3D view at the camera poses. An error, Ctrl+C or
closing the view sends stop to all robots (twice) and kills every helper it started. One printed line per check.
Launch order: `../HANDOVER.md` section 14.

`run_real('dry', opts)` is the same code path without hardware: loopback bridge, `fake_bot.py` stand-ins for the real
robots (no camera of their own, `--truth`), `aruco_node.py --sim` (true poses through a synthetic pinhole camera into
marker corner pixels + 0.3 px noise, then the real homography / pose code); `realBots` works the same.

## Broker (Mosquitto optional)
`py/mini_broker.py`: MQTT 3.1.1 broker, Python standard library only (clean sessions, QoS 0/1, + and # wildcards,
retained messages, last will, keep-alive 1.5 x, client id takeover, per-client send queue). `link_launch` in broker
mode (so `run_real()`) uses a broker that answers on `P.mqtt.broker:P.mqtt.port` as it is (`procs.broker` =
`'external'`, never stopped); when none answers and that host is this PC (`link_broker_status`) it starts
`mini_broker.py` on all interfaces, port `P.mqtt.port` (`procs.broker` = `'mini'`, log `procs.logs.broker`), and
`link_kill` stops it. `mini_broker.py` exits with code 3 when the port is taken. By hand:
`python link\py\mini_broker.py [--port N] [--verbose] [--log file]`.
**Firewall**: the first time Python listens on the port, Windows asks; allow Python on **private** networks (the PC's
hotspot is one), or the robots cannot connect. Nothing in the repository changes firewall settings.
One robot without the twin: `python link\py\bot_ping.py --seconds 5` (every `zadp/bot1..3/telemetry` message and the
rate; subscribes only).

## Tests and demo (no hardware)
```matlab
test_link          % loopback bridge + fake robots: contract fields, watchdog, keep-alive across a stall, linked runs
test_run_real      % run_real('dry'); robot off its slot; hidden corner; view closed; mixed realBots 1; real+fake id
                   % refused; broker mode with mini_broker.py on a private free port
test_firmware      % config.h vs twin_params, motion_core JS mirror (node), camera geometry (numpy)
demo_link
```
`python link\py\test_mini_broker.py` (broker, free port). Tests never use `P.mqtt.port`: loopback, or broker mode on a
private free port (`test_run_real` g), so they never publish where a real robot listens.

## Files
| File | What |
|---|---|
| `TOPICS.md` | the contract |
| `run_real.m` | one-command launcher (real / dry) |
| `link_write_config.m` | twin_params -> `link_config.json` |
| `py/bridge.py` | localhost TCP <-> broker (paho-mqtt) or in-process loopback router; repeats each robot's last command at `P.link.repeatHz` while MATLAB was heard within `P.link.hostTimeout` |
| `py/fake_bot.py` | reference robot: the firmware's command semantics and controller on the twin's motor model; optional camera / truth publisher; `--real` ids refused |
| `py/aruco_node.py` | camera node: DICT_4X4_50, robots 1-3, corners 10-13, homography, parallax, axle offset, `t` + `age`; `--sim` mode; `--robots` limits the published ids |
| `py/test_aruco_geometry.py` | camera geometry on synthetic pixels (numpy only) |
| `py/check_numbers.py` | firmware `config.h` vs `link_config.json` |
| `py/transport.py` | the two clients (TCP to the bridge, paho to a broker) |
| `py/mini_broker.py` | dependency-free MQTT 3.1.1 broker, started by `link_launch` when no broker runs on this PC |
| `py/test_mini_broker.py` | broker tests with paho clients, raw sockets and 3 `fake_bot.py` robots (python only) |
| `py/bot_ping.py` | prints robot telemetry for N seconds (one robot check without the twin) |
| `link_broker_status` | does a broker answer on `P.mqtt.broker:P.mqtt.port`, is that host this PC |
| `markers/make_markers.py` | printable markers at true size (needs OpenCV); output `robots_A4.pdf`, `corners_A4.pdf`, `placement.txt` |
| `link_open`, `link_send_cmd`, `link_poll`, `link_close` | MATLAB side, base MATLAB only (`tcpclient`, `jsonencode`, `jsondecode`) |
| `link_keepalive`, `link_stop_all` | heartbeat to the bridge, stop everybody |
| `link_from_brain(W, Wprev, L)` | route-step changes -> commands (drive pieces of one drive = one `goto`; turns carry `dir` and `tol`) |
| `brain_run_linked(plan, P, opts)` | `brain_run` with the link, paced to real time; `syncToReal`, `onStep` (live view), `abortFcn`; ends on a robot `offline` / silent |
| `link_launch`, `link_kill`, `link_python` | start / stop the helper processes (PID checked; `bots`, `simBots`, `realBots`, `camBots`), find Python |

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
