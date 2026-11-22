# ZADP truck robot firmware (ESP32)

One small ESP32 differential-drive robot = one haul truck of the ZADP digital twin (`twin3d/`). The twin leads:
the MATLAB brain sends route steps over MQTT, the robot drives them with its own encoder controller, reports
telemetry, and fuses the overhead ArUco camera pose. Contract: `twin3d/link/TOPICS.md`. Reference behaviour:
`twin3d/link/py/fake_bot.py` (same semantics and laws). Every number in `include/config.h` equals
`twin3d/params/twin_params.m`; `twin3d/link/py/check_numbers.py` (run by MATLAB `test_firmware`) proves it.

```
firmware/
  .gitignore                     secrets.h, .pio/, test binaries
  zadp_bot/                      PlatformIO project
    platformio.ini               envs bot1, bot2, bot3 (+ bot1_pcb.. for BIN2 on GPIO 5)
    include/config.h             EVERY tunable (pins, geometry, gains, tolerances, topics); portable
    include/motion_core.h        all maths, portable C++11 (no Arduino): odometry, wheel PI, laws, state machine, fusion
    include/secrets.example.h    copy to include/secrets.h (WIFI_SSID, WIFI_PASS, MQTT_HOST)
    src/main.cpp                 setup(): motors off, queues, task watchdog, starts the FreeRTOS tasks (section 1a)
    src/app.h                    shared messages, snapshot, queue / event-group / task handles
    src/control_task.cpp         200 Hz control task (core 1): encoders, motion_core, motors, watchdogs
    src/comms_task.cpp           WiFi + MQTT state machine (core 0), JSON -> queues, thread-safe publish
    src/telemetry_task.cpp       20 Hz telemetry + ack on a new seq
    src/console_task.cpp         serial bench self-test, boot stack report
    src/health_task.cpp          status LED
    src/encoders.cpp, motors.cpp, hw.h   PCNT quadrature counters (or GPIO ISR), TB6612 via LEDC
    test/host/test_motion_core.cpp   host test of motion_core.h against a simulated robot (plain C++17)
    test/host/js_mirror/mirror_test.js  line-by-line JS copy of the above (see "Host test")
```

## 1. Flashing

1. Install PlatformIO (VS Code extension or `pip install platformio`). Versions are pinned in `platformio.ini`:
   `espressif32 @ 7.0.1` (arduino-esp32 core 2.0.17, ESP-IDF 4.4.7; already in the development PC's PlatformIO
   cache), `knolleary/PubSubClient @ 2.8`, `bblanchon/ArduinoJson @ 7.2.1`. The first build downloads the two
   libraries (internet needed once).
2. `copy zadp_bot\include\secrets.example.h zadp_bot\include\secrets.h` and fill in `WIFI_SSID`, `WIFI_PASS` and
   `MQTT_HOST` (the IP of the PC running Mosquitto; the same address as `P.mqtt.broker`). Without it the build stops
   with `include/secrets.h is missing: ...`. It is git-ignored.
3. The robot number is the PlatformIO env: robot 1 = `bot1`, etc. (it sets `BOT_ID`, which sets the topics
   `zadp/botN/cmd`, `zadp/botN/telemetry`, client id `zadp-botN` and the start pose = parking slot PN).
   PCB robots with BIN2 on GPIO 5: use `bot1_pcb`, `bot2_pcb`, `bot3_pcb`.
4. Plug in robot N and run, in `firmware/zadp_bot`:
   ```
   pio run -e bot1 -t upload
   pio device monitor -e bot1
   ```
   The monitor prints `ZADP truck bot 1, arduino-esp32 core 2.x, ESP-IDF 4.4, encoders pcnt, cmd zadp/bot1/cmd, ...`,
   the FreeRTOS line (rates, priorities, watchdog), the help, and after 3 s the task stack report.
5. Label each robot with its number and put ArUco marker id N on it (`twin3d/link/README.md`).

## 1a. FreeRTOS design (no polling loop)

`setup()` forces the motors off, starts the encoders, creates the RTOS objects, sets the task watchdog and starts
five pinned tasks; the Arduino `loop()` task deletes itself. All numbers below are named constants in
`include/config.h` (section "FreeRTOS tasks"). Stacks are bytes (ESP-IDF); serial command `w` prints the unused
stack of every task (also printed once 3 s after boot; under 512 bytes is flagged `LOW`).

| Task | Core | Prio | Rate | Stack | Owns |
|---|---|---|---|---|---|
| `zadp_ctrl` | 1 | 20 | 200 Hz, `vTaskDelayUntil` | 4096 | encoders, `zadp::Truck` (odometry, fusion, state machine, laws, wheel PI), PWM, 300 ms command watchdog, link-loss stop; feeds the task watchdog; never blocks |
| `zadp_tel` | 0 | 6 | 20 Hz + at once on a new seq | 4096 | telemetry JSON; publishes through the comms lock |
| `zadp_comms` | 0 | 5 | 5 ms (50 ms while WiFi is down) | 6144 | WiFi, the PubSubClient object (mutex), connect / backoff state machine, last will, JSON parsing; reads up to 1 + 8 packets per pass; heartbeat for the control task |
| `zadp_console` | 1 | 2 | 10 ms poll | 4096 | serial bench self-test, stack report once 3 s after boot |
| `zadp_health` | 1 | 1 | 20 ms | 2048 | status LED (no printf) |

Data flow (no shared globals without protection):

* `ctrlQ` (8 deep, FIFO): comms puts each parsed `cmd`, console puts bench commands / raw PWM / pose; the control
  task drains it at the start of every tick. Senders never wait (full = dropped and counted, `drops` in `i`).
* `camQ` (1 deep, `xQueueOverwrite`): comms puts this robot's camera fix with its arrival time; the control task takes
  the latest one per tick (an older unread fix is replaced: latest wins) and adds the queue wait to the fix's `age`.
* `snapQ` (1 deep, `xQueueOverwrite` / `xQueuePeek`): the control task writes one consistent snapshot per tick
  (seq, counts, pose, state, flags, counters); telemetry, console and LED only read it.
* task notification control -> telemetry: a new seq was accepted this tick, so the acknowledgement telemetry goes
  out at once (contract), then the 20 Hz period restarts.
* event group `linkEvents` (WiFi up, MQTT up): written by comms, read by control (link-loss stop), LED and console.
  MQTT counts as up only after both subscriptions succeeded; on WiFi loss the bits are cleared before anything else.
* `commsAliveMs` (atomic heartbeat): comms stores `millis()` each pass; the control task treats a heartbeat older than
  0.3 s as link loss (comms stuck on the MQTT lock or a socket).
* `mqttMutex`: PubSubClient is not thread safe; comms (loop / connect) and telemetry (publish) take it.

Safety: motors are driven to zero (short brake) first thing in `setup()`; the 300 ms command watchdog of the
contract stops a drive or turn; MQTT down (or the comms heartbeat stale) for more than `LINK_LOSS_STOP_S`
(= `WATCHDOG_S`, 0.3 s) forces both motors to zero whatever the state (bench commands and raw tests exempt); any new
command, from the broker or the bench, ends a raw PWM test, which is also capped at `RAW_MAX_S` (10 s) and +-255; the
task watchdog (`TWDT_TIMEOUT_MS` 1 s, panic) resets the chip if the control task stops running, and a reset leaves the
driver inputs low (stopped). The watchdog watches only the control task: the Arduino default also watches the core-0
idle task, which PubSubClient can starve for up to the 2 s socket timeout (bad broker, WiFi drop mid-packet), so it is
removed (`disableCore0WDT()`, IDF 5: `idle_core_mask = 0`).
Odometry uses the chord form of the exact arc, which stays exact in float32 at every heading (the old
difference-of-sines form lost whole steps at headings near +-pi in float32; `mirror_test.js` checks both).
Encoders: ESP32 PCNT units (x4 quadrature, 1 us glitch filter, no interrupt per edge), unwrapped to 32 bit every
tick; `ENC_USE_PCNT 0` (env `bot1_isr`) brings back the GPIO-interrupt decoder with identical counts and signs.
Note: a TCP drop is seen by the MQTT client only when the socket errors or the 15 s keep-alive fails; a lost WiFi is
seen at once. In both cases the 300 ms command watchdog has already stopped the robot.

## 2. Wiring (matches `include/config.h`)

| Function | ESP32 GPIO | TB6612 / encoder pin | Note |
|---|---|---|---|
| Left motor PWM | 27 | PWMA | LEDC 5 kHz, 8 bit |
| Left motor IN1 | 14 | AIN1 | |
| Left motor IN2 | 26 | AIN2 | `MOTOR_L_INVERT 1`: forward = AIN1 LOW, AIN2 HIGH |
| Right motor PWM | 32 | PWMB | |
| Right motor IN1 | 25 | BIN1 | |
| Right motor IN2 | 33 (PCB: **5**) | BIN2 | `PIN_BIN2`; GPIO 5 is a strapping pin (motor may twitch at reset) |
| TB6612 STBY | - | STBY | tied high on the board, not driven |
| Left encoder A / B | 34 / 35 | | input-only pins, need external pull-ups; `ENC_L_DIR +1` |
| Right encoder A / B | 36 / 39 | | input-only pins, need external pull-ups; `ENC_R_DIR -1` |
| Status LED | 2 | on-board blue LED | |

Pins, directions and encoder signs come from the working micromouse firmware (same ESP32 + TB6612 + encoders).
The new robots have encoders only (no ToF, no IMU). Recheck every robot with the bench test below.

## 3. Bench self-test (serial monitor, 115200 baud)

Do this before the first MQTT test, with the brain NOT running (a broker command replaces any bench command).
Bench commands are exempt from the 300 ms watchdog. Type a command and Enter. `h` prints the list.

**Wheels off the ground first** (robot on a box):

1. **Motor direction.** `l` spins the LEFT wheel open loop at PWM 120 for 2 s. It must turn so that the robot would
   drive FORWARD. Wrong way: flip `MOTOR_L_INVERT` in `config.h`. Same with `r` and `MOTOR_R_INVERT`. A different
   wheel turns: the motor plugs are swapped.
2. **Encoder sign.** `e`, then `l`, then `e` again: the LEFT count must have gone UP by a few thousand, the right
   count barely changed. Went down: flip `ENC_L_DIR`. Same with `r` and `ENC_R_DIR`. Count unchanged: check the
   encoder supply and pull-ups on GPIO 34-39.
3. **Counts per revolution.** Put a tape mark on the tyre. `m`, turn the left wheel by hand exactly 10 turns
   forward, `e`. It prints the counts since the mark and the revolutions; 10 turns should read about 27721 counts
   (2772.1 per turn). Set `CPR_LEFT` / `CPR_RIGHT` to counts / 10 if they differ by more than 0.5 %.
   Once per robot also turn one wheel 20 turns (about 55400 counts): this crosses the PCNT counter wrap at 30000;
   the count must keep rising smoothly (about 2772 per turn). If it jumps, build env `bot1_isr` and report it.
4. **Dead band.** `l 15 1`, `l 20 1`, `l 25 1`, ... : the lowest PWM at which the wheel starts turning every time.
   Repeat on the floor later (it is higher under load) and put that value in `PWM_DEADBAND`.
   Raw tests (`l`, `r`, `b`) are capped at 10 s and PWM +-255; `s` or any new command ends them.
5. **Top speed (feed-forward).** `v` (status every second), then `b 255 2`: read `v` (m/s) while both wheels run
   at full PWM. Off the ground it is near 0.534 m/s; if it is much lower, set
   `PWM_PER_MPS = (255 - PWM_DEADBAND) / v_full`. `v` again to stop the stream.

**Then on the floor:**

6. `p 0 0 0` (pose = origin, heading 0), `d`: the robot drives 0.2 m straight (closed loop, accelerating at
   0.5 m/s^2, max 0.2 m/s) and stops. Measure with a ruler: 200 mm +- 3 mm. Long: the real wheel is bigger than
   34 mm (lower `WHEEL_DIAMETER_M`), short: raise it.
7. `t`: turns +90 deg (counter-clockwise seen from above) on the spot. Four `t` in a row should end where the
   robot started. If it under-turns (the encoders see the full turn but the body turns less, tyre scrub) raise
   `TRACK_SCALE`: the micromouse needed 1.14. Set the SAME value in `twin_params.m` (`P.bot.trackScale`), or
   `test_firmware` / `check_numbers.py` reports a mismatch. `t -90` turns clockwise.
8. `d 0.5`, `t 180`, `d 0.5`: back at the start within about 1 cm (encoders only; the camera removes the rest).
9. `s` stops anything at once.

## 4. First MQTT test (one robot, no MATLAB)

Start Mosquitto on the PC (`listener 1883` and `allow_anonymous true` in `mosquitto.conf` for a closed test
network, then `mosquitto -v`). Power the robot: the LED goes from 5 Hz blink (no WiFi) to a double blink (no
broker) to a short flash every 2 s (connected, idle). Watch everything:

```
mosquitto_sub -h 192.168.1.10 -t "zadp/#" -v
```
Telemetry arrives 20 times a second:
`zadp/bot1/telemetry {"id":1,"seq":0,"encL":0,"encR":0,"x":0.44000,"y":-0.15900,"heading":3.14159,"state":"idle"}`.

Tell the robot where it is (as the camera would; `t` 0 = no order check, `age` 0 = no latency). The first fix
replaces the start pose:
```
mosquitto_pub -h 192.168.1.10 -t zadp/cam/pose -m "{\"bot\":1,\"x\":0.2,\"y\":0.2,\"heading\":0,\"t\":0,\"age\":0}"
```

Commands must be REPEATED at least every 300 ms while the robot drives or turns, or it stops (watchdog, state
`idle`). In the real system `bridge.py` repeats them at 10 Hz; by hand use Mosquitto 2.x `--repeat N --repeat-delay 0.1`
(or a shell loop). Keep `seq` the same while repeating; use a new `seq` for the next command.

```
# goto (0.5, 0.2) heading east at 0.1 m/s: 60 repeats = 6 s
mosquitto_pub -h 192.168.1.10 -q 1 -t zadp/bot1/cmd --repeat 60 --repeat-delay 0.1 -m "{\"id\":1,\"truck\":0,\"seq\":1,\"action\":\"goto\",\"x\":0.5,\"y\":0.2,\"heading\":0,\"speed\":0.1}"
# turn in place to heading 1.57 rad, counter-clockwise, within 1 deg
mosquitto_pub -h 192.168.1.10 -q 1 -t zadp/bot1/cmd --repeat 40 --repeat-delay 0.1 -m "{\"id\":1,\"truck\":0,\"seq\":2,\"action\":\"turn\",\"x\":0.5,\"y\":0.2,\"heading\":1.57,\"speed\":0.2,\"dir\":1,\"tol\":1}"
# dump: 4 s in state dumping, then arrived (needs no repeats)
mosquitto_pub -h 192.168.1.10 -q 1 -t zadp/bot1/cmd -m "{\"id\":1,\"truck\":0,\"seq\":3,\"action\":\"dump\",\"x\":0.5,\"y\":0.2,\"heading\":1.57,\"speed\":0.2}"
# stop
mosquitto_pub -h 192.168.1.10 -q 1 -t zadp/bot1/cmd -m "{\"id\":1,\"truck\":0,\"seq\":4,\"action\":\"stop\",\"x\":0,\"y\":0,\"heading\":0,\"speed\":0.2}"
```
Bash loop instead of `--repeat`: `for i in $(seq 60); do mosquitto_pub ... ; sleep 0.1; done`.
PowerShell: `1..60 | % { mosquitto_pub ... ; Start-Sleep -Milliseconds 100 }`.

Expected telemetry: on each new `seq` one message at once (the acknowledgement) with that `seq`; `state` goes
`moving` (or `turning` / `dumping`; `idle` for a stop), then `arrived` once the wheels have stopped, and stays
`arrived` until the next `seq`. A goto never turns in place at its end. Stopping the repeats mid-drive gives `idle` within 0.3 s; resuming them continues the same goto. Without the
camera node the pose is the encoder estimate only. Next: `run_real` (`twin3d/HANDOVER.md` section 14).

Status LED: no WiFi 5 Hz blink | WiFi, no broker: double blink | watchdog stop: 10 Hz flicker | moving / turning /
bench raw test: on | dumping: 2 Hz | idle / arrived: short flash every 2 s.

## 5. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Robot spins on the spot instead of driving | one motor runs backwards, or one encoder counts backwards (the wheel loop then drives that wheel harder and harder) | bench steps 1-2: `MOTOR_x_INVERT`, `ENC_x_DIR` |
| Drives backwards on `d` | both motors inverted, or both encoders inverted (then it also runs away) | flip both `MOTOR_x_INVERT` (or both `ENC_x_DIR`); recheck steps 1-2 |
| Runs off at full speed, never stops | encoder sign wrong or encoder dead (speed reads 0, the PI winds up to full PWM) | `e` while turning a wheel by hand: both counts must move; step 2 |
| Drifts to one side on straights | CPR or wheel diameter differ between wheels, low battery on one side, dead band too high | step 3 per wheel; `PWM_DEADBAND`; the camera fix corrects position, not a bent path |
| Under-turns / over-turns | tyre scrub, wrong track | `TRACK_SCALE` (step 7) |
| Stops short / hunts at the goal, `arrived` late | dead band too low: the wheel cannot creep at the final low speeds | raise `PWM_DEADBAND` to the on-floor value |
| Resets (brown-out) when the motors start | motors share the ESP32 supply; battery sag | separate motor supply or a big capacitor (470-1000 uF) at the TB6612 VM, fresh battery, thick ground wire; the serial log then shows `Brownout detector was triggered` |
| No telemetry at all | wrong WiFi / broker IP in `secrets.h`, broker only listens on localhost, firewall on port 1883 | serial log `[wifi] ...` / `[mqtt] failed, state N` (N = -2 network, 5 not authorised); `listener 1883` + `allow_anonymous true`; allow 1883 in the PC firewall |
| Telemetry but the robot ignores commands | wrong topic (`zadp/bot1/cmd` for robot 1), JSON error, `id` field of another robot | `i` in the serial monitor: `rx cmd` counter, `badjson` counter |
| Moves a bit then stops, state `idle` | no repeats: watchdog (300 ms) | repeat the command every 100 ms (the brain does) |
| Pose jumps or stays wrong with the camera running | wrong marker id, camera frame differs from the arena frame, camera node lagging | `i`: `cam acc / rej` counters; fixes more than 50 mm / 20 deg off are rejected until 10 agree in a row |
| `overruns` counter rising | the control task missed ticks | report it; should stay 0; `exec max` (us per tick) in `i` shows the load |
| State `(link lost: motors off)` in `i` | MQTT down longer than 0.3 s, or the comms task stuck that long | check WiFi / broker; bench commands still work |
| Serial `[mqtt] ... subscribe failed` | broker accepted the connect but a SUBSCRIBE write failed | it disconnects and retries with backoff by itself; persistent: check the broker log |
| Reset with `Task watchdog got triggered ... zadp_ctrl` | control task blocked > 1 s | report the decoded backtrace (monitor filter `esp32_exception_decoder`) |
| `w` shows a task `LOW` | stack nearly full | raise that `*_TASK_STACK` in `config.h` |

## 6. Contract decisions (formerly 18 open points)

All 18 points this section used to list are decided in `twin3d/link/TOPICS.md` (section "Decisions") and implemented
identically in `motion_core.h`, `src/*.cpp`, `fake_bot.py`, `bridge.py` and the MATLAB link. Changes against the first
firmware: goto has NO final turn and follows the leg line (carrot `LOOKAHEAD_M`, `SLIDE_DIST_M`) with coast-predicted
braking (`MOTOR_TAU_S`); go-to-point has the 3 cm capture radius and the 25/15 deg rotate hysteresis; turns take the
optional `tol` (deg; the brain sends 1 deg at dump stops); `stop` ends in `arrived` once the wheels stopped; seq is
uint32; camera fixes carry `age` (no clock sync; lag = age + `CAM_NET_DELAY_S`) and are moved forward by the odometry
since the image (latency compensation, as `pose_fusion.m`); fusion gain 0.3; start pose y = -0.159 (was -0.157, now
checked against `P.brain.parking`). The firmware still keeps its own camera gate (first fix snaps, 50 mm / 20 deg
jumps rejected unless 10 in a row agree).

## 7. Numbers and where they come from

All in `include/config.h`, each tagged with its source: `[twin]` = `twin3d/params/twin_params.m` (P.bot, P.ctrl,
P.brain, P.cam, P.link, P.truck, P.mqtt), `[ref]` = the micromouse firmware (pins, directions, CPR, PWM), `[fw]` =
firmware-only (control rate, wheel PI, dead band, camera gate). `python twin3d/link/py/check_numbers.py` prints the
table config.h vs twin_params (43 rows, all must say `ok`; needs `twin3d/link/link_config.json`, which MATLAB writes:
`link_write_config(twin_params())`). After calibrating `TRACK_SCALE`, put the same value in `P.bot.trackScale`.

## 8. Host test

`test/host/test_motion_core.cpp` runs `motion_core.h` against a simulated robot that is deliberately different from
the firmware's assumptions (dead band 28 vs 20, loaded top speed 0.48 vs 0.534 m/s, motor lag 0.08 s, short-brake lag
0.03 s, encoder quantisation, 1 % wheel slip, 1 kHz physics, camera 30 Hz, 3 mm / 1 deg noise, age 40 ms + 5-15 ms):
```
cd firmware/zadp_bot
g++ -std=c++17 -O2 -Wall -Wextra -Iinclude test/host/test_motion_core.cpp -o test_motion_core && ./test_motion_core
```
No host C++ compiler on the development PC: the C++ test was only compiled (`-fsyntax-only`, xtensa g++ 8.4, clean).
`test/host/js_mirror/mirror_test.js` is a line-by-line JavaScript copy of `motion_core.h` and of the test; it is the
one that ran (`node test/host/js_mirror/mirror_test.js`, also inside MATLAB `test_firmware`): 47 passed, 0 failed
(the last 2 checks run the odometry in float32 via `Math.fround`, as on the ESP32).

| Check | Result (JS mirror, 2026-11-19) |
|---|---|
| goto, 40 random starts, camera on | 40/40 arrived, worst estimate 7.3 mm, worst TRUE 8.9 mm from the target |
| goto, 40 random starts, encoders only | 40/40 arrived, worst estimate 8.0 mm, worst TRUE 62.1 mm (1 % slip, no camera) |
| line following, 0.6 m leg, 20 runs | worst TRUE cross-track 5.5 mm; the twin's own model on the same leg: 8.4 mm (check: firmware <= twin) |
| turn, 60 runs, dir -1/0/+1 | 60/60 within 3 deg (estimate, no camera); camera on: worst TRUE 3.20 deg; direction obeyed 120/120 |
| turn with tol 1 deg | 0.06 deg |
| semantics | ack on new seq only; repeats keep `arrived`; no final goto turn; stop -> idle -> arrived 0.46 s later; dump 4.005 s; seq above 2^31; new dump after a watchdog stop reports `dumping` |
| watchdog | idle 0.21 s after the last repeat stopped, wheels < 2 mm/s 0.66 s after, resumes and arrives |
| camera fusion | latency compensation: along-track estimate error at cruise 1.40 mm (uncompensated 10.25 mm); gain 0.3 exact; wrap through 180; jump rejected, relock after 10; out-of-order and age 0.30 s rejected |
| odometry in float32 | equal counts, headings pi / 2.5 / -3 / 1.5708 / 0.5 / 0, 1-26 counts per tick: distance error 2.8e-6 (old form: up to 100 %) |

## 9. Not verified (no hardware, no network here)

* Nothing ran on an ESP32. Not tested: motor and encoder directions, CPR, dead band, PI gains on the real motors,
  the PCNT decoder on real encoders (sign table checked against the old ISR table in a script, 8/8 transitions; the
  counter reset at +-30000 that the unwrap relies on is documented ESP32 behaviour, check it with the 20-turn test),
  the 200 Hz timing under WiFi load, task stack margins (use `w`), brown-out behaviour, LED patterns, the serial
  self-test, the task-watchdog reset path, removal of the core-0 idle task from the watchdog, the comms heartbeat
  link-loss stop, the socket drain loop.
* Do tonight with internet (after creating `secrets.h`): `pio run -e bot1` once, so both libraries are fetched and
  `comms_task.cpp` is compiled against the real headers before the robots are built.
* Not linked with PlatformIO: PubSubClient and ArduinoJson are not in the local PlatformIO cache and downloading was
  not allowed. Every `src/*.cpp` was syntax-checked (`-fsyntax-only -Wall -Wextra`) with the installed xtensa g++
  8.4.0 against the installed arduino-esp32 2.0.17 / ESP-IDF 4.4.7 headers of `espressif32 @ 7.0.1`: gnu++17 and
  gnu++11, BOT_ID 1-3, `ENC_USE_PCNT` 1 and 0, and `PIN_BIN2=5`: 104 compiles, 0 errors, 0 warnings. PubSubClient and
  ArduinoJson were replaced by minimal stand-in headers with the expected signatures, so calls into those two
  libraries are checked only against those signatures.
* The arduino-esp32 3.x / ESP-IDF 5.x branches: the PCNT `pulse_cnt` branch of `encoders.cpp` was syntax-checked
  against the installed ESP-IDF 6.0.1 driver headers (clean); the TWDT `esp_task_wdt_config_t` /
  `esp_task_wdt_reconfigure` calls were only compared with the IDF 6.0.1 header; the 3.x LEDC branch
  (`ledcAttach` / `ledcWrite(pin)`) was not compiled (core 3 not installed).
* No real broker: the MQTT side of the firmware was never connected.
* The C++ host test itself was not run (no host compiler; it was syntax-checked with xtensa g++); the numbers in
  section 8 come from the JS mirror (47 passed, 0 failed after this change).
* IDF 5 branch of the watchdog setup: that `esp_task_wdt_reconfigure` with `idle_core_mask = 0` really unsubscribes
  an already subscribed idle task was not checked against the IDF source (only matters on core 3.x).
