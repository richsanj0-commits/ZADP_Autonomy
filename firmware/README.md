# ZADP truck robot firmware (ESP32)

One small ESP32 differential-drive robot = one haul truck of the ZADP digital twin (`twin3d/`). The twin leads:
the MATLAB brain sends route steps over MQTT, the robot drives them with its own controller, reports telemetry and
fuses the overhead camera pose. Contract: `twin3d/link/TOPICS.md`. Reference behaviour: `twin3d/link/py/fake_bot.py`.
Every twin number in `include/config.h` equals `twin3d/params/twin_params.m`; `twin3d/link/py/check_numbers.py`
(run by MATLAB `test_firmware`) checks it.

Status: robot 1 verified on the floor over WiFi / MQTT (built-in broker): straight 0.2 m, left 90 deg, right 90 deg
all arrive. Robots 2 and 3 with motors: not yet flashed or verified. Two bare ESP32 boards ran the bench build as robots 2 and 3 under `run_real('bench')` on 2026-11-24 (300 s, 236 commands, every monitor 0).

```
firmware/zadp_bot/                PlatformIO project
  platformio.ini                  envs botN, botN_pcb, botN_b, botN_bench, bot1_isr (table in section 1)
  include/config.h                every tunable (pins, geometry, gains, tolerances, topics, tasks); portable
  include/motion_core.h           all maths, portable C++ (odometry, wheel PI, laws, state machine, camera fusion)
  include/secrets.example.h       copy to include/secrets.h (WIFI_SSID, WIFI_PASS, MQTT_HOST); git-ignored
  src/main.cpp                    setup(): motors off, encoders, gyro probe, queues, task watchdog, tasks; reset reason
  src/app.h                       messages, snapshot, queue / event-group / task handles
  src/control_task.cpp            200 Hz control (core 1)
  src/comms_task.cpp              WiFi + MQTT state machine (core 0), JSON -> queues, status on connect
  src/telemetry_task.cpp          20 Hz telemetry + ack on a new seq
  src/console_task.cpp            serial bench self-test, boot stack report
  src/health_task.cpp             status LED
  src/gyro.cpp                    MPU6050 probe, bias, gyro task, sign check against the encoders
  src/encoders.cpp, motors.cpp, hw.h   PCNT quadrature counters (or GPIO ISR), TB6612 via LEDC
  test/host/test_motion_core.cpp  host test of motion_core.h against a simulated robot (plain C++17)
  test/host/js_mirror/mirror_test.js   line-by-line JS copy of motion_core.h and that test
```

## 1. Build and flash

Toolchain is pinned in `platformio.ini`: `espressif32 @ 7.0.1` (arduino-esp32 2.0.17, ESP-IDF 4.4.7),
`PubSubClient @ 2.8`, `ArduinoJson @ 7.2.1`. Run in `firmware/zadp_bot`:

```
pio run -e bot2 -t upload        # robot 2 (bot1, bot3 likewise)
pio device monitor -e bot2
```

The env sets `BOT_ID`: topics `zadp/botN/...`, client id `zadp-botN`, start pose = parking slot PN. All robots share
`include/secrets.h` (same WiFi, same broker IP). PCB robots with BIN2 on GPIO 5: `bot2_pcb`, `bot3_pcb`.
Keep the robot still for the first second after power-up (gyro bias). Boot banner:

```
gyro: on (heading rate from the gyro)                       or  off (no MPU6050 at 0x68: heading from the encoders)
ZADP truck bot 2, arduino-esp32 core 2.x, ESP-IDF 4.4, encoders pcnt, cmd zadp/bot2/cmd, telemetry zadp/bot2/telemetry
build Nov 23 2026 12:00:00, reset reason: poweron, gyro: off (...)
FreeRTOS: control 200 Hz (core 1 prio 20), telemetry 20 Hz, task watchdog 1000 ms, link-loss stop 0.30 s
```

Then the help, and after 3 s the task stack report. Put ArUco marker id N on robot N (`twin3d/link/README.md`).

Build variants (`platformio.ini`; N = 1..3, all set `BOT_ID`):

| Env | Variant |
|---|---|
| `botN` | standard wiring (section 2), PCNT encoders; the gyro is detected at boot |
| `botN_pcb` | PCB robots: BIN2 on GPIO 5 instead of 33 |
| `botN_b` | wiring B: PWMA 13, AIN2 12, AIN1 14, BIN1 26, BIN2 25, PWMB 33 (`include/config.h`) |
| `bot1_isr` | GPIO-interrupt encoder decoder instead of PCNT (fallback; `-DENC_USE_PCNT=0`) |
| `botN_bench` | bench build `-DBENCH_SIM=1`: a bare ESP32 (no driver, encoders, gyro) acts as a moving robot; motors, encoders and gyro are simulated at the lowest layer, everything above is the real code. Its status message carries `"bench":true`. Used by `run_real('bench')` (`twin3d/link/README.md`) |

Built by the lead (2026-11-23/24): `bot1`, `bot2_b`, `bot2_bench`, `bot3_bench`. The other envs were not built in this check. On the floor: `bot1` (robot 1). On the bench: `bot2_bench`, `bot3_bench`.

## 2. Robot 1 hardware (the micromouse build; robots 2 and 3 use wiring B, env `botN_b`)

* ESP32 DevKit (esp32dev), TB6612FNG, two encoder motors (34 mm wheels, track 75.3 mm, CPR 2772.1 / 2772.3).
* Feed-forward measured on the micromouse (wheels in the air): `PWM = 15 + 432.6 x speed (m/s)` (`PWM_DEADBAND`,
  `PWM_PER_MPS`); the wheel PI adds the floor load.
* MPU6050 gyro at 0x68 on I2C 21 / 22. Yaw axis found from gravity at boot; the sign is then checked against the
  encoders while turning (wheels slip but never turn the wrong way) and flipped if it disagrees. Encoder-only turns
  slipped (79-95 deg for the same counts); with the gyro the 90 deg turns arrive.
* Five ToF sensors (XSHUT on 23, 19, 18, 17, 16) are unused and held in shutdown (open drain low).
* Wiring: PWMA 27, AIN1 14, AIN2 26, PWMB 32, BIN1 25, BIN2 33 (PCB: 5, strapping pin), STBY tied high; encoders
  L 34 / 35, R 36 / 39 (input only, external pull-ups); LED 2. Left motor inverted, right encoder sign -1.

Gyro detection: every robot probes the MPU6050 at boot. Found = heading rate from the gyro; not found = heading from
the encoders. The XSHUT pins are pulled low on every robot: open drain only sinks, none is a strapping pin, so a board
without ToF sensors is not affected. A gyro that stops answering (100 failed reads) is switched off for good and the
heading falls back to the encoders; `i` and the status topic then show `gyro off`.

## 3. Tasks (FreeRTOS, no polling loop)

`setup()` forces the motors off, starts the encoders, probes the gyro, creates the RTOS objects, sets the task
watchdog and starts the tasks; `loop()` deletes itself. Numbers: `include/config.h`, section "FreeRTOS tasks".

| Task | Core | Prio | Rate | Stack (B) | Owns |
|---|---|---|---|---|---|
| `zadp_ctrl` | 1 | 20 | 200 Hz, `vTaskDelayUntil` | 4096 | encoders, `zadp::Truck`, PWM, 300 ms command watchdog, 1 s camera watchdog, link-loss stop, gyro sign check; feeds the task watchdog and the motor guard; never blocks |
| (esp_timer task) | 0 | 22 | 10 ms timer | system | motor guard callback (`motors.cpp`): zeroes both motor outputs when `zadp_ctrl` has not ticked for 50 ms |
| `zadp_gyro` | 1 | 10 | 5 ms | 3072 | only if a gyro was found: I2C read, rate -> atomics (the control task never touches I2C; readings older than 20 ms are ignored) |
| `zadp_tel` | 0 | 6 | 20 Hz + at once on a new seq | 4096 | telemetry JSON through the comms lock |
| `zadp_comms` | 0 | 5 | 5 ms (50 ms while WiFi is down) | 6144 | WiFi, PubSubClient (mutex), connect / backoff, last will, JSON parsing, status on connect, heartbeat |
| `zadp_console` | 1 | 2 | 10 ms | 4096 | serial bench self-test, stack report 3 s after boot |
| `zadp_health` | 1 | 1 | 20 ms | 2048 | status LED |

Data flow: `ctrlQ` (8, FIFO, senders never wait) comms / console -> control; `camQ` (1, overwrite: latest fix wins)
comms -> control; `snapQ` (1, overwrite / peek) control -> everyone else; task notification control -> telemetry
(ack); event group `linkEvents` (WiFi up, MQTT up); atomic `commsAliveMs` heartbeat; `mqttMutex` around PubSubClient.

Safety: motors short-braked first thing in `setup()`; 300 ms command watchdog stops a drive or turn; camera watchdog
(`CAM_STALE_S` 1 s): once a camera fix was accepted, a drive or turn stops the same way when no fix was accepted for
1 s and resumes with the next accepted fix (a robot that never saw a camera and serial bench commands are not
watched; telemetry field `cam` = ms since the last accepted fix, -1 = never); the camera order check is bounded (10
fixes in a row with a time stamp not after the last one = camera clock stepped back: re-base and accept); motor guard
(`MOTOR_GUARD_MS` 50 ms, checked every 10 ms by an `esp_timer` callback outside the control task, real and bench
builds): control task silent = both motor outputs forced to zero until it ticks again, `i` shows `guard trips`; MQTT down or the
comms heartbeat stale for more than 0.3 s forces both motors to zero (bench commands exempt); raw PWM tests capped
at 10 s and +-255; task watchdog (1 s, panic -> reset, driver inputs low) watches only the control task (the core-0
idle task is removed because PubSubClient can block core 0 for the 2 s socket timeout).

## 4. MQTT

Topics (`twin3d/link/TOPICS.md`): subscribes `zadp/botN/cmd` (QoS 1) and `zadp/cam/pose`; publishes
`zadp/botN/telemetry` (20 Hz, last will `{"id":N,"state":"offline"}`) and, once per connect, the retained
`zadp/botN/status`:

```
{"id":2,"fw":"Nov 23 2026 12:00:00","reset":"brownout","gyro":false,"ip":"192.168.1.42","rssi":-58}
```

A bench build adds `"bench":true` to this message.

`reset` is `esp_reset_reason()`: `poweron`, `external pin`, `software`, `panic`, `interrupt watchdog`,
`task watchdog`, `other watchdog`, `brownout`, ... Being retained, it explains the last reboot without USB:
`mosquitto_sub -h <broker> -t "zadp/+/status" -v`.

Broker: Mosquitto if one runs, otherwise `twin3d/link/py/mini_broker.py` (started by `run_real` / `link_launch`).
Check robots without the twin: `python twin3d/link/py/bot_ping.py --bots 1,2,3` (subscribes only).
Commands must be repeated at least every 300 ms while driving or turning (the bridge repeats at 10 Hz); same `seq` =
repeat, new `seq` = new command. Full run: `run_real` (`twin3d/HANDOVER.md` section 14).

## 5. Bench self-test (serial 115200)

Brain not running (a broker command replaces any bench command). `h` lists the commands. For a new robot:

1. Wheels in the air. `l` / `r`: that wheel turns FORWARD (else flip `MOTOR_x_INVERT`).
2. `e`, `l`, `e`: left count goes UP by a few thousand (else flip `ENC_L_DIR`); same with `r`.
3. `m`, turn a wheel by hand 10 turns, `e`: about 27721 counts. Once also 20 turns (crosses the PCNT wrap at 30000):
   the count must rise smoothly; if it jumps, use env `bot1_isr` (GPIO-interrupt decoder).
4. On the floor: `p 0 0 0`, `d` = 0.2 m straight (ruler: 200 +- 3 mm); `t` = +90 deg, `t -90` = clockwise.
5. `i` = status once (pose, counts, speeds, PWM, link, counters, gyro on/off, reset reason); `v` = every second;
   `w` = task stacks and heap; `s` = stop.

Status LED: no WiFi 5 Hz | no broker double blink | watchdog stop 10 Hz flicker | moving / turning / raw on |
dumping 2 Hz | idle / arrived short flash every 2 s.

## 6. Troubleshooting

| Symptom | Fix |
|---|---|
| Spins instead of driving, or runs away | motor or encoder sign wrong: bench steps 1-2 |
| Rebooted unexpectedly | `reset` in `zadp/botN/status` (or `i`): `brownout` = battery / motor supply sag (capacitor at TB6612 VM, fresh battery); `task watchdog` / `panic` = report the serial backtrace (`esp32_exception_decoder`) |
| `gyro off` on a robot that has one | boot line gives the reason: `no MPU6050 at 0x68` = check SDA 21 / SCL 22 and power; `lost` = bus failed while running |
| Under-turns without a gyro | `TRACK_SCALE` (same value in `P.bot.trackScale`) |
| No telemetry | `secrets.h` WiFi / broker IP, firewall on 1883; serial `[wifi]` / `[mqtt] failed, state N` |
| Ignores commands | wrong topic or `id`; `i`: `rx cmd`, `badjson` |
| Stops after a moment, state `idle` | no command repeats (300 ms watchdog) |
| Stops 1 s after the camera node ended, `i` shows `(camera lost)` | camera watchdog: restart the camera node (the command resumes with the first fix) or reboot the robot for a run without camera |
| `(link lost: motors off)` in `i` | MQTT down > 0.3 s; bench commands still work |
| `overruns` rising, or `w` shows `LOW` | report it / raise that `*_TASK_STACK` |

## 7. Tests

```
node test/host/js_mirror/mirror_test.js          # 68 checks of motion_core.h (JS mirror, float32 odometry)
python ../../twin3d/link/py/check_numbers.py     # config.h vs twin_params: must print ALL MATCH
g++ -std=c++17 -O2 -Wall -Wextra -Iinclude test/host/test_motion_core.cpp -o test_motion_core   # needs a host g++
```

The gyro is not modelled in the twin or the host test (it only makes the real heading better). The arduino-esp32
3.x / ESP-IDF 5 branches (LEDC by pin, `pulse_cnt`, TWDT reconfigure) are kept but were never compiled here.
