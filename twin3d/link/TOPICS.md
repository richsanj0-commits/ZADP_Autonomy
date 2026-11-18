# MQTT contract (twin3d <-> brain <-> robots)

Units: metres and radians. Frame: arena frame, origin lower-left, x right, y up,
heading measured counter-clockwise from +x. All payloads are JSON, UTF-8.
`{1-3}` is the robot number. Broker default: `localhost:1883`.

| Topic | Direction | Rate | QoS | Payload |
|---|---|---|---|---|
| `zadp/bot{1-3}/cmd` | brain -> bot | on demand + repeat at 10 Hz | 1 | `{id, truck, seq, action, x, y, heading, speed}` |
| `zadp/bot{1-3}/telemetry` | bot -> brain | 20 Hz (+ once at once on a new cmd) | 0 | `{id, seq, encL, encR, x, y, heading, state}` |
| `zadp/cam/pose` | camera -> all | 30 Hz per robot | 0 | `{bot, x, y, heading, t}` |

Design: THE TWIN LEADS. The brain runs the simulated robots (the digital twin). Whenever a robot's current route
step changes, the link publishes that step as a command; the real robot executes it with its own encoder
controller and reports telemetry; the camera reports the true pose. The link logs twin-vs-real error and can
hold the twin until the real robot reports `arrived` (`opts.syncToReal`, see `README.md`).

## cmd
`{"id":1,"truck":7,"seq":12,"action":"goto","x":0.80,"y":0.50,"heading":1.57,"speed":0.2}`

| Field | Meaning |
|---|---|
| `id` | robot number 1..3 |
| `truck` | running truck number carried by the robot (0 = none). Informative only |
| `seq` | per-robot command counter, starts at 1. **An opaque id, not an order**: a message with the same `seq` as the active command is a repeat (keep-alive), any other `seq` replaces the active command |
| `action` | `"goto"`, `"turn"`, `"dump"` or `"stop"` |
| `x`, `y` | m. goto: the point to drive to. turn / dump: where the robot stands (informative). stop: ignored (0) |
| `heading` | rad. goto: direction of travel (the robot drives to x, y with its nose first; if it ends more than 3 deg off this heading it turns in place to it). turn: target heading. dump: heading at the dump stop. stop: ignored |
| `speed` | m/s cruise speed cap (default 0.20) |
| `dir` | OPTIONAL, `turn` only: +1 counter-clockwise, -1 clockwise, 0 or absent = shortest. The brain checked the swept body for that direction, so a robot must obey it |

Actions:
* `goto` drive forward to (x, y), brake, stop within 5 mm, then (only if needed) turn in place to `heading` (3 deg).
* `turn` rotate in place to `heading` in direction `dir`, stop within 3 deg.
* `dump` stand still `dumpTime` (4 s) in state `dumping`, then `arrived`. The real robot has no bed; the twin animates the
  tip and grows the pile.
* `stop` cancel the active command, stop the wheels, state `idle`.

Backwards compatible with the first draft `{id, x, y, heading, action}`: a message with only those fields still works
(`seq` 0 counts as a new command, `speed` defaults to 0.20, `truck` is ignored).

## telemetry
`{"id":1,"seq":12,"encL":12034,"encR":12110,"x":0.52,"y":0.40,"heading":0.31,"state":"moving"}`

`seq` is the `seq` of the active or last accepted command (0 before the first). `encL`, `encR` are cumulative encoder counts
(4x quadrature, 2772.2 counts per wheel revolution). `x, y, heading` are the robot's own estimate (encoders, plus the
camera fix when it uses `zadp/cam/pose`). `state`:

| `state` | when |
|---|---|
| `idle` | no command, after `stop`, or while stopped by the 300 ms watchdog |
| `moving` | executing a `goto` (driving or braking) |
| `turning` | executing a `turn` (or the final turn of a `goto`) |
| `dumping` | executing a `dump` |
| `arrived` | the command with this `seq` is finished. Stays `arrived` until the next command |

The brain waits for `state == "arrived"` AND `seq ==` the seq it sent.

## cam/pose
`{"bot":2,"x":1.10,"y":0.75,"heading":-0.80,"t":1795023945.56}`

Absolute pose from the overhead ArUco webcam (nominal 3 mm, 1 deg, 50 ms latency). `t` = Unix time of the image in
seconds. One message per visible robot per frame.

## Bridge status (local only)
Between MATLAB and `link/py/bridge.py` (localhost TCP, newline-delimited JSON, `{"topic":...,"payload":{...}}`) the bridge
also sends `zadp/bridge/status` = `{"mode":"loopback"|"broker","broker":...}` on connect. Not an MQTT topic on the broker.

## Firmware checklist (ESP32 side)
The real firmware currently uses TCP port 23, NOT MQTT. To join this contract:

1. **Connect** to the broker (`P.mqtt.broker`, port 1883), client id `zadp-bot<N>`, keep-alive 15 s, automatic reconnect.
   Set an MQTT last-will on `zadp/bot<N>/telemetry` if you like (`state: "idle"`).
2. **Subscribe** `zadp/bot<N>/cmd` at QoS 1 and `zadp/cam/pose` at QoS 0 (optional: fuse the pose of your own `bot`
   into the dead-reckoned estimate with gain 0.5 on each fix, as the twin does; without it the estimate drifts about
   1 % of the distance).
3. **Publish** `zadp/bot<N>/telemetry` at 20 Hz (QoS 0, retain off) with the fields above, and once immediately whenever
   a new command is accepted (this is the acknowledgement the brain measures the round trip with).
4. **Commands**: same `seq` as the active/last command = repeat, only refresh the watchdog. A different `seq` = new command, replace
   the old one at once (no need to finish it). Unknown `action` = ignore, report `idle`.
5. **goto**: forward-only go-to-point on the estimate (rotate in place above 25 deg heading error, otherwise
   `v = min(cruise, 2.5*d, sqrt(2*0.25*d))*cos(e)`, `w = 3*e`), arrive within 5 mm, brake to a stop, then turn to `heading` if
   more than 3 deg off. **turn**: in-place, direction `dir`, yaw rate `min(5.3, max(0.3, 3*|e|))` rad/s, stop within 3 deg.
   **dump**: wheels still, state `dumping` for 4 s, then `arrived`. **stop**: wheels to zero now, `idle`.
6. **Limits**: wheel speed 0.534 m/s (300 rpm), acceleration 0.5 m/s^2, cruise 0.20 m/s.
7. **Watchdog**: if no `cmd` message (new or repeat) arrived for **300 ms**, stop the wheels (report `idle`), keep the goal, and
   resume when messages come back. The brain repeats the last command every 100 ms.
8. State `arrived` only when the wheels have stopped.
