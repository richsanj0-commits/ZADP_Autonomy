# MQTT contract (twin3d <-> brain <-> robots <-> camera)

The single contract. Implemented by `link/*.m` (MATLAB), `link/py/bridge.py`, `link/py/fake_bot.py` (reference
robot), `link/py/aruco_node.py` (camera) and `firmware/zadp_bot` (ESP32). Every number below is a field of
`params/twin_params.m`; MATLAB writes them to `link/link_config.json` (`link_write_config`) for the Python side, and
`link/py/check_numbers.py` checks `firmware/zadp_bot/include/config.h` against them (`tests/test_firmware`).

Units: metres, radians, seconds. Frame: arena frame, origin = lower-left corner of the 1 m arena, x right (east), y up (north),
heading counter-clockwise from +x, wrapped to (-pi, pi]. The apron is y in [-0.25, 0]. Payloads are JSON, UTF-8.
`N` = robot number 1..3. Broker: `P.mqtt.broker`:`P.mqtt.port` (1883).

| Topic | Direction | Rate | QoS | Payload |
|---|---|---|---|---|
| `zadp/botN/cmd` | brain -> robot | new command + bridge repeats at `P.link.repeatHz` (10 Hz) | 1 new seq, 0 repeats | `{id, truck, seq, action, x, y, heading, speed[, dir, tol]}` |
| `zadp/botN/telemetry` | robot -> brain | `P.link.telemetryHz` (20 Hz) + at once on a new seq | 0, not retained | `{id, seq, encL, encR, x, y, heading, state}` |
| `zadp/cam/pose` | camera -> all | `P.cam.rateHz` (30 Hz) per visible robot | 0 | `{bot, x, y, heading, t, age}` |
| `zadp/botN/status` | robot -> anyone | once per MQTT connect | 0, **retained** | `{id, fw, reset, gyro, ip, rssi}` |

`status` (firmware only, diagnosis): build date `fw`, `reset` = reason of the last reboot (e.g. `poweron`, `brownout`,
`task watchdog`, `panic`), `gyro` true/false, WiFi `ip` and `rssi` (dBm). Retained: `mosquitto_sub -t zadp/+/status -v`.

Robot subscriptions: `zadp/botN/cmd` at QoS 1, `zadp/cam/pose` at QoS 0, clean session (no queued old commands).
MQTT client id `zadp-botN`, keep-alive 15 s, automatic reconnect. Last will: `{"id":N,"state":"offline"}` on
`zadp/botN/telemetry`, QoS 0, not retained.

Design: THE TWIN LEADS. The brain runs the simulated robots; whenever a robot's route step changes the link publishes
that step as one command; the real robot drives it with its own controller (the same laws as the twin); the camera
reports the true pose. With `syncToReal` the twin waits at the end of each step until the robot reports `arrived`.

## cmd
`{"id":1,"truck":7,"seq":1795023945,"action":"turn","x":0.80,"y":0.50,"heading":1.57,"speed":0.2,"dir":1,"tol":1}`

| Field | Meaning |
|---|---|
| `id` | robot number. A message whose `id` is present and differs from the robot is ignored |
| `truck` | running truck number (0 = none). Informative only |
| `seq` | per-robot command id, **unsigned 32 bit**. MATLAB starts each session at a time base (0.1 s ticks since 1970, mod 2^30, `link_open`). **Opaque, not an order**: same `seq` as the robot's current one = repeat (refreshes the watchdog only); any other `seq` = new command, replaces the active one at once. `seq` 0 or missing (first-draft messages) = always new |
| `action` | `goto`, `turn`, `dump`, `stop`; anything else = unknown |
| `x`, `y` | goto: the end point. turn / dump: where the robot stands (informative). stop: ignored |
| `heading` | goto: direction of the leg (the line through x, y with this direction is followed). turn: target heading. dump / stop: ignored |
| `speed` | m/s cruise cap. Missing or <= 0 = `P.bot.cruiseSpeed` (0.20); capped at 0.20; a goto never below 0.02 |
| `dir` | turn only: +1 CCW, -1 CW, 0 or missing = shortest. The brain swept-checked that direction, so it is obeyed |
| `tol` | turn only: heading tolerance in DEGREES. Missing = `P.brain.turnTolDeg` (3). The brain sends `P.brain.dumpTolDeg` (1) for the final turn at a dump stop |

Actions (robot side = `motion_core.h` = `fake_bot.py` = the twin's `bot_controller.m` / `brain_step.m`):
* `goto`: drive along the leg line to (x, y), brake, stop; `arrived` once the wheels have stopped. **No final turn in
  place** (the brain plans every turn as its own `turn` command). Law:
  `along` = distance to go along the leg; aim distance `along + P.ctrl.arriveTol`; when `along >= P.brain.slideDist`
  (0.05) the aim point lies toward a carrot `P.brain.lookahead` (0.15) ahead of the robot's foot point on the leg
  line, else straight along the robot's own track. Go-to-point (`bot_controller.m`): farther than
  `P.ctrl.captureRadius` (0.03) rotate in place when the heading error exceeds `rotateThreshDeg` (25) and keep
  rotating until below `rotateExitDeg` (15); otherwise `v = min(cruise, kpDistance*d, sqrt(2*decelFrac*maxAccel*d))*cos(e)`,
  `w = kpHeading*e'*min(1, d/captureRadius)` (e' = error of the nose, or of the tail when backing up). Inside the
  capture radius never rotate in place. Brake (wheel command 0) when `along <= coast(v)`, `coast = v*tau` for
  `v <= a*tau`, else `v^2/(2a) + a*tau^2/2` (a = `P.bot.maxAccel`, tau = `P.bot.motorTimeConstant`, v = mean
  measured wheel speed), or when within arriveTol. Missing `heading` = the line from the robot to (x, y).
  A goto without `x`/`y` is an unknown action.
* `turn`: in place toward `heading` in direction `dir`; remaining angle `mod(dir*(h - th) + 0.3, 2pi) - 0.3` (an
  overshoot up to 0.3 rad is corrected backwards); yaw rate `min(cruise/(track/2), max(P.brain.minTurnRate, kpHeading*|e|))`;
  stop within `tol`; `arrived` once the wheels have stopped. A turn without `heading` is an unknown action.
* `dump`: wheels still for `P.truck.dumpTime` (4 s) in state `dumping`, then `arrived`. A repeat does not restart it.
  The real robot has no bed; the twin tips and grows the pile.
* `stop`: cancel, wheel command 0 (the speed ramps down at `maxAccel`, 0.4 s from cruise), state `idle`, then
  `arrived` (with the stop's seq) once the wheels have stopped.
* unknown action: the `seq` is taken (and acknowledged), the active command is cancelled (wheels ramp to 0), `idle`.
* Wheel speed limit `P.bot.maxWheelSpeed` (0.534 m/s): the firmware scales both wheels together (keeps the curvature;
  the laws never ask more than 0.23 m/s). Effective track = `P.bot.track * P.bot.trackScale` (= `TRACK_M*TRACK_SCALE`).

## telemetry
`{"id":1,"seq":1795023945,"encL":12034,"encR":12110,"x":0.52,"y":0.40,"heading":0.31,"state":"moving"}`

`seq` = the robot's current seq (0 before the first command). `encL`, `encR` = cumulative 4x quadrature counts since
power-on (reset on reboot; about `P.bot.encoderCPR` = 2772.2 per wheel revolution, per-wheel calibrated in the
firmware). `x, y, heading` = the robot's own fused estimate (heading wrapped). `state`:

| `state` | when |
|---|---|
| `idle` | no command yet; after an unknown action; a `stop` while the wheels run down; a watchdog stop |
| `moving` | executing a `goto` (driving or braking) |
| `turning` | executing a `turn` |
| `dumping` | executing a `dump` |
| `arrived` | the command with this `seq` is finished AND both wheels are below 2 mm/s with zero command. Stays until the next new seq |
| `offline` | MQTT last will only (`{"id":N,"state":"offline"}`, no other fields) |

The brain waits for `state == "arrived"` AND `seq ==` its seq. `brain_run_linked` ends the run (stop to everyone) on
`offline` or when a robot's telemetry stops for 2 s.

## cam/pose
`{"bot":2,"x":1.10,"y":0.75,"heading":-0.80,"t":1795023945.561,"age":0.042}`

Pose of the robot's AXLE centre (the marker sits `P.cam.markerOffset` ahead of it), heading = the robot's nose.
`t` = Unix time of the image on the camera PC (`grab time - P.cam.captureDelay`); used only to drop duplicate /
out-of-order messages (`t <= last t`). `age` = publish time - `t`, measured on the camera PC, so no clock sync is
needed. A robot takes the image to show it `lag = age + P.link.netDelay` (0.01 s, assumed transit) ago; a message
without `age` uses `lag = P.cam.latency` (0.05). `lag > P.cam.maxAge` (0.25) = dropped.

Fusion (`pose_fusion.m`, `motion_core.h onCamera`, `fake_bot.py fuse`): take the encoder-only odometry pose at
`now - lag` from a history (>= 0.3 s), move the fix forward by the odometry motion since then (rigid motion in the
robot frame), then `est += P.cam.fusionGain * (z - est)` (gain 0.3, heading wrapped). Firmware only (not in the twin):
the first fix replaces the estimate; later fixes more than 50 mm / 20 deg off are rejected unless 10 in a row agree
(robot moved by hand). Before any fix robot N assumes it stands on parking slot P_N (`P.brain.parking`), which is
where `run_real` requires it to be.

## Timing (one place: `P.link`)
| Number | Value | Where it acts |
|---|---|---|
| `P.link.watchdog` | 0.3 s | robot: no cmd message (new or repeat) for this long while driving (goto drive phase) or turning: wheel command 0, state `idle`, goal kept; the command resumes when messages return. A dump keeps timing, braking completes |
| `P.link.repeatHz` | 10 Hz | bridge.py repeats each robot's last command (same seq, QoS 0) ... |
| `P.link.hostTimeout` | 1.0 s | ... while the MATLAB client that sent it is connected and sent any line within this time. Brain steps stall 0.4-0.6 s (planning): the repeats bridge the stall, the robots keep driving |
| `P.link.heartbeat` | 0.1 s | MATLAB sends `{"heartbeat": t}` to the bridge at least this often (`link_keepalive`, called by `brain_run_linked`) |
| `P.link.telemetryHz` | 20 Hz | robot telemetry |
| `P.link.netDelay` | 0.01 s | added to the camera `age` |

Stop latency: MATLAB closed or crashed (socket closed) -> repeats stop at once -> robots idle after 0.3 s. MATLAB
hung (socket open, silent) -> repeats stop after 1.0 s -> robots idle after 1.3 s. Bridge or broker gone -> 0.3 s.

## Bridge (local only)
MATLAB <-> `bridge.py`: localhost TCP (`P.link.bridgePort` 18830), newline-delimited JSON:
`{"topic":...,"payload":{...}}` (publish / receive), `{"subscribe":[patterns]}`, `{"heartbeat":t}`. On connect the
bridge sends `zadp/bridge/status` = `{"mode":"loopback"|"broker","broker":...,"repeatHz":...,"hostTimeout":...}`.
`zadp/sim/truth` (`fake_bot.py --truth` -> `aruco_node.py --sim`) exists only in dry runs.

## Decisions (firmware/README.md section 6 had 18 open points; all are resolved above)
1 watchdog scope: drive / turn phases only. 2 last will `offline`. 3 `stop` ends in `arrived` once stopped.
4 unknown action: seq taken, command cancelled, `idle`. 5 missing fields: goto without x/y or turn without heading =
unknown; goto without heading = line from the robot. 6 `speed` default and cap 0.20, goto floor 0.02. 7 turn
tolerance: `tol` field (deg). 8 goto: no final turn. 9 seq 0 always new. 10 foreign `id` ignored. 11 heading wrapped,
counts since power-on. 12 "stopped" = both measured wheel speeds < 2 mm/s and zero command. 13 stop ramps at
maxAccel. 14 camera time: `age` field (no clock sync). 15 camera gating: firmware-only safety, documented.
16 start pose: slot P_N. 17 fake_bot stall flag cleared on a new command (fixed). 18 wheel limit: both scaled.
