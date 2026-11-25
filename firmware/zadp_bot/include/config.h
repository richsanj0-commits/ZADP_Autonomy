// config.h : every tunable of the ZADP truck firmware in one place.
//
// PORTABLE: no Arduino headers. The host test (test/host) includes this file too, so the numbers the
// robot runs with are the numbers the test checks.
//
// Source of each number:
//   [twin]  twin3d/params/twin_params.m (P.bot, P.ctrl, P.brain, P.cam, P.truck), the single source of numbers
//           of the digital twin; also copied in twin3d/link/py/fake_bot.py
//   [topics] twin3d/link/TOPICS.md (MQTT contract and firmware checklist)
//   [ref]   the user's working micromouse firmware (include/robot_config.h, src/main.cpp), same ESP32 + TB6612
//           + encoder hardware
//   [fw]    chosen here for the firmware; not in the twin.
#pragma once

// ---------------------------------------------------------------- identity
// Set per robot: platformio.ini has envs bot1, bot2, bot3 that pass -DBOT_ID=N. The default is only used
// when building without an env flag.
#ifndef BOT_ID
#define BOT_ID 1
#endif
#if (BOT_ID < 1) || (BOT_ID > 3)
#error "BOT_ID must be 1, 2 or 3 (TOPICS.md: robots 1..3)"
#endif

#define ZADP_STR_(x) #x
#define ZADP_STR(x) ZADP_STR_(x)

// ---------------------------------------------------------------- MQTT [topics]
#define TOPIC_CMD        "zadp/bot" ZADP_STR(BOT_ID) "/cmd"        // subscribe, QoS 1
#define TOPIC_TELEMETRY  "zadp/bot" ZADP_STR(BOT_ID) "/telemetry"  // publish, QoS 0, retain off
#define TOPIC_STATUS     "zadp/bot" ZADP_STR(BOT_ID) "/status"     // publish once per connect, QoS 0, RETAINED
#define TOPIC_CAM_POSE   "zadp/cam/pose"                           // subscribe, QoS 0 (all robots)
#define MQTT_CLIENT_ID   "zadp-bot" ZADP_STR(BOT_ID)
#define MQTT_PORT        1883        // [twin] P.mqtt.port
#define MQTT_KEEPALIVE_S 15          // [topics] keep-alive 15 s
#define MQTT_BUFFER_SIZE 512         // [fw] bytes; a cmd is ~130 B, PubSubClient default 256 is tight
#define MQTT_RETRY_MS    2000        // [fw] first wait between broker connect attempts (doubles per failure ...)
#define MQTT_RETRY_MAX_MS 30000      // [fw] ... up to this (exponential backoff, reset on success)
#define MQTT_SOCKET_TIMEOUT_S 2      // [fw] PubSubClient read timeout inside one packet
#define WIFI_RETRY_MS    5000        // [fw] WiFi.reconnect() if still down after this long (doubles per failure ...)
#define WIFI_RETRY_MAX_MS 60000      // [fw] ... up to this (exponential backoff, reset when WiFi is up)

// ---------------------------------------------------------------- pins [ref]
// TB6612FNG. STBY is assumed tied high on the board (not driven by firmware).
// Wiring B (envs botN_b, -DWIRING_B=1): motor pins in one row of the 30-pin DevKit: D13 D12 D14 . D26 D25 D33.
// rule: GPIO 12 is a strapping pin (must be low at reset): only the TB6612 input (pull-down) may be on it.
#if defined(WIRING_B) && WIRING_B
#define PIN_PWMA 13   // left motor PWM
#define PIN_AIN1 14
#define PIN_AIN2 12
#define PIN_PWMB 33   // right motor PWM
#define PIN_BIN1 26
#define PIN_BIN2 25
#else
#define PIN_PWMA 27   // left motor PWM
#define PIN_AIN1 14
#define PIN_AIN2 26
#define PIN_PWMB 32   // right motor PWM
#define PIN_BIN1 25
#endif
// FLAG: the breadboard robot uses GPIO 33. The PCB design moves BIN2 to GPIO 5. Set PIN_BIN2 to 5 for PCB
// robots (e.g. build_flags -DPIN_BIN2=5). GPIO 5 is a strapping pin and toggles at reset, so the right motor
// may twitch while the ESP32 boots.
#ifndef PIN_BIN2
#define PIN_BIN2 33
#endif
// Encoders: input-only pins 34..39 (no internal pull-ups: the encoder board must pull up).
#define PIN_ENC_L_A 34
#define PIN_ENC_L_B 35
#define PIN_ENC_R_A 36
#define PIN_ENC_R_B 39
#define PIN_BUTTON  4    // push button to GND (internal pull-up: pressed = LOW). Reserved, not read yet
#define PIN_LED     2   // on-board blue LED [ref]

// Directions [ref]: forward PWM on the left motor is AIN1 LOW / AIN2 HIGH (inverted), on the right motor
// BIN1 HIGH / BIN2 LOW. Encoder sign: left +1, right -1 makes forward counts positive (checked on the
// reference robot 2026-10-27). Re-check on every new robot with the bench self-test (README).
#define MOTOR_L_INVERT 1
#define MOTOR_R_INVERT 0
#define ENC_L_DIR  1
#define ENC_R_DIR -1

// ---------------------------------------------------------------- robot geometry
#define WHEEL_DIAMETER_M 0.034f      // [twin] P.bot.wheelDiameter, [ref] 34 mm
#define TRACK_M          0.0753f     // [twin] P.bot.track, [ref] 75.3 mm wheel centre to centre
#define CPR_LEFT         2772.1f     // [ref] counts per wheel rev, 4x quadrature (twin uses the mean 2772.2)
#define CPR_RIGHT        2772.3f     // [ref]
// Track scale [twin] P.bot.trackScale (effective track = TRACK_M * TRACK_SCALE = P.bot.trackEff). With the gyro it
// only converts yaw rate to wheel speeds; without it, it also sets the encoder heading (the micromouse firmware
// needed 1.14 for tyre scrub). Change it HERE AND in twin_params.m (P.bot.trackScale); check_numbers.py flags a mismatch.
#define TRACK_SCALE      1.0f

// ---------------------------------------------------------------- limits [twin]/[topics]
#define MAX_WHEEL_SPEED  0.53407f    // m/s, P.bot.maxWheelSpeed = 300 rpm / 60 * pi * 0.034
#define CRUISE_SPEED     0.20f       // m/s, P.bot.cruiseSpeed (also the default and the cap of cmd.speed)
#define MIN_GOTO_SPEED   0.02f       // m/s, fake_bot.py: speed = max(speed, 0.02) for goto
#define MAX_ACCEL        0.5f        // m/s^2, P.bot.maxAccel, applied to each wheel speed reference
#define MOTOR_TAU_S      0.08f       // s, P.bot.motorTimeConstant (coast prediction of the goto braking point)

// ---------------------------------------------------------------- go-to-point and turn laws [twin]
#define KP_HEADING        3.0f       // 1/s, P.ctrl.kpHeading
#define KP_DISTANCE       2.5f       // 1/s, P.ctrl.kpDistance
#define ROTATE_THRESH_DEG 25.0f      // P.ctrl.rotateThreshDeg: rotate in place above this heading error ...
#define ROTATE_EXIT_DEG   15.0f      // P.ctrl.rotateExitDeg: ... until below this one (hysteresis)
#define DECEL_FRAC        0.5f       // P.ctrl.decelFrac: approach braking = 0.5 * MAX_ACCEL
#define ARRIVE_TOL_M      0.005f     // P.ctrl.arriveTol: 5 mm on the estimate
#define HEADING_TOL_DEG   3.0f       // P.brain.turnTolDeg: default turn tolerance (a turn's `tol` field overrides it)
#define MIN_TURN_RATE     0.3f       // rad/s, P.brain.minTurnRate
#define CAPTURE_RADIUS_M  0.03f      // P.ctrl.captureRadius: never rotate in place closer; yaw faded out to 0 at the target
#define SLIDE_DIST_M      0.05f      // P.brain.slideDist: last 5 cm aim along the leg, brake straight
#define LOOKAHEAD_M       0.15f      // P.brain.lookahead: before that steer to a carrot 0.15 m ahead on the leg line
#define TURN_WRAP_SLACK   0.3f       // rad, brain_step.m turnCmd: overshoot up to 0.3 rad is corrected backwards
#define STOPPED_SPEED     0.002f     // m/s, brain_step.m / fake_bot.py: "stopped" = both wheels below 2 mm/s
#define DUMP_TIME_S       4.0f       // s, P.truck.dumpTime
#define WATCHDOG_S        0.3f       // s, P.link.watchdog: no cmd for 300 ms: stop the wheels, keep the goal

// ---------------------------------------------------------------- camera fusion
#define CAM_FUSION_GAIN   0.3f       // [twin] P.cam.fusionGain, weight of one fix (heading wrapped)
#define CAM_NET_DELAY_S   0.01f      // [twin] P.link.netDelay: the fix shows the robot (age + this) ago
#define CAM_DEFAULT_LAG_S 0.05f      // [twin] P.cam.latency: lag used when a message has no `age` field
#define CAM_MAX_AGE_S     0.25f      // [twin] P.cam.maxAge: drop a fix older than this (odometry history is 0.32 s)
#define CAM_MAX_JUMP_M    0.05f      // [fw] reject a fix this far from the estimate ...
#define CAM_MAX_JUMP_DEG  20.0f      // [fw] ... or this far off in heading
#define CAM_RELOCK_COUNT  10         // [fw] ... unless this many fixes in a row disagree: then jump to the camera.
                                     // Also: this many fixes in a row with a time stamp not after the last accepted
                                     // one = the camera clock stepped back: re-base the order check and go on
// Camera watchdog [fw] (copied in fake_bot.py, checked by check_numbers.py). Once a fix was accepted, a drive / turn
// stops when no fix was accepted for this long, and resumes with the next accepted fix. Why 1 s: the camera sends
// 30 fixes per second (P.cam.rateHz), so 1 s is 30 lost in a row: a dead camera node, not WiFi jitter, and 3 times
// the relock time (CAM_RELOCK_COUNT fixes = 0.33 s), so a relock never trips it. Until the stop the robot drives at
// most CRUISE_SPEED * 1 s = 0.2 m on encoders alone.
#define CAM_STALE_S       1.0f
#define CAM_AGE_REPORT_MAX_MS 99999  // [fw] cap of the telemetry field `cam` (ms since the last accepted fix): 5 digits

// ---------------------------------------------------------------- wheel speed loop [fw]
#define CONTROL_HZ       200         // control loop rate (FreeRTOS tick is 1 ms on arduino-esp32)
#define TELEMETRY_HZ     20          // [twin] P.link.telemetryHz
#define VEL_WINDOW       4           // ticks: wheel speed = counts over the last 4 ticks (20 ms)
#define PWM_FREQ_HZ      5000        // [ref] 5 kHz
#define PWM_BITS         8           // [ref] 8-bit
#define PWM_MAX          255         // full scale of PWM_BITS
// Measured on this robot (micromouse motor characterization, wheels in the air, 2026-10-28):
//   PWM = 4 + counts_per_s / 60, linear up to PWM 150, both motors within 2 %; PWM 15 always starts a wheel.
//   60 counts/s per PWM = 432.6 PWM per m/s (25954 counts per m). On the floor the wheel PI adds the load.
#define PWM_DEADBAND     15.0f       // [ref] smallest PWM that always starts a wheel
#define PWM_PER_MPS      432.6f      // [ref] feed-forward slope, PWM per m/s
#define WHEEL_KP         600.0f      // PWM per m/s of speed error
#define WHEEL_KI         7500.0f     // PWM per m of accumulated error (Kp/Ki = 0.08 s = motor time constant)
#define WHEEL_KD         0.0f        // PWM per m/s^2; off (encoder speed is too coarse to differentiate)
#define WHEEL_I_LIMIT    120.0f      // PWM, integral clamp (anti-windup)
#define REF_ZERO         1e-4f       // m/s, |reference| below this counts as zero (no feed-forward deadband kick)
#define ZERO_PWM_BRAKE   1           // 1 = TB6612 short brake at zero output, 0 = coast

// ---------------------------------------------------------------- start pose [twin]
// P.brain.parking: robot N starts on slot PN (fake_bot.py --init). The first camera fix replaces it.
#if BOT_ID == 1
#define INIT_X 0.44f
#elif BOT_ID == 2
#define INIT_X 0.75f
#else
#define INIT_X 0.90f
#endif
#define INIT_Y       (-0.159f)
#define INIT_HEADING 3.14159265f

// ---------------------------------------------------------------- serial
#define SERIAL_BAUD 115200           // [ref] monitor_speed

// ---------------------------------------------------------------- FreeRTOS tasks [fw]
// arduino-esp32: configMAX_PRIORITIES 25, tick 1 kHz. System tasks for reference: WiFi 23 (core 0), esp_timer 22
// (core 0), lwIP tcpip 18 (core 0), Arduino loopTask 1 (core 1, deleted after setup). Core 0 = network, core 1 =
// control. Stack sizes are in BYTES (ESP-IDF FreeRTOS); the serial command 'w' prints the unused stack of each task.
#define CTRL_TASK_CORE      1        // control: the only time-critical task, alone on core 1 with low-priority helpers
#define CTRL_TASK_PRIO      20       // above everything on core 1; below the core-0 system tasks
#define CTRL_TASK_STACK     4096     // Truck object is static, not on the stack
#define COMMS_TASK_CORE     0        // WiFi + MQTT client, JSON parsing (next to the WiFi / lwIP tasks)
#define COMMS_TASK_PRIO     5
#define COMMS_TASK_STACK    6144     // ArduinoJson JsonDocument + PubSubClient + WiFi printf
#define COMMS_PERIOD_MS     5        // mqtt.loop() period while connected; each pass drains up to COMMS_DRAIN_MAX packets
#define COMMS_DRAIN_MAX     8        // extra packets read per pass (PubSubClient reads one per loop(); a camera burst has 3)
#define COMMS_IDLE_MS       50       // loop period while WiFi is down
#define TEL_TASK_CORE       0        // telemetry publisher
#define TEL_TASK_PRIO       6        // above comms so the 20 Hz rate holds while comms parses
#define TEL_TASK_STACK      4096     // snprintf with %f
#define TEL_MUTEX_WAIT_MS   10       // max wait for the MQTT client lock; on timeout this telemetry slot is skipped
#define TEL_BUF_SIZE        200      // bytes, one telemetry JSON (about 125 used; longest possible 140)
#define STATUS_BUF_SIZE     200      // bytes, the retained status JSON (about 130 used)
#define CONSOLE_TASK_CORE   1        // serial bench self-test
#define CONSOLE_TASK_PRIO   2
#define CONSOLE_TASK_STACK  4096     // sscanf / printf with %f
#define CONSOLE_PERIOD_MS   10       // serial poll period
#define CONSOLE_LINE_MAX    64       // characters per serial command line
#define HEALTH_TASK_CORE    1        // status LED only (no printf: the boot stack report runs in the console task)
#define HEALTH_TASK_PRIO    1
#define HEALTH_TASK_STACK   2048
#define HEALTH_PERIOD_MS    20       // LED update period (the fastest pattern toggles every 50 ms)
#define CMD_QUEUE_LEN       8        // comms/console -> control: cmd, bench raw, bench pose (in order, never blocks)
#define CAM_QUEUE_LEN       1        // comms -> control: camera fix, latest wins (xQueueOverwrite needs length 1)
#define TWDT_TIMEOUT_MS     1000     // task watchdog: control task must run at least this often or the chip resets.
                                     // Only the control task is watched (the core-0 idle task is removed: PubSubClient
                                     // busy-waits up to MQTT_SOCKET_TIMEOUT_S on core 0 and must not reset the chip)
// Motor guard: an esp_timer callback (esp_timer task, core 0; NOT the control task) forces both motor outputs to zero
// when the control task has not ticked for MOTOR_GUARD_MS, and keeps forcing zero every MOTOR_GUARD_PERIOD_MS until
// it ticks again. It covers the time before the task watchdog resets the chip. 50 ms = 10 missed control ticks
// (the control task normally runs every 5 ms and takes well under 1 ms); worst case cut after 50 + 10 ms.
#define MOTOR_GUARD_MS        50     // control task silent this long = motors forced to zero
#define MOTOR_GUARD_PERIOD_MS 10     // guard timer period
#define LINK_LOSS_STOP_S    WATCHDOG_S  // [twin] P.link.watchdog: MQTT down (or comms task stuck) this long = motors off
                                        // (bench exempt)
#define BOOT_STACK_REPORT_MS 3000    // print the stack high-water marks once this long after boot (console task)
#define RAW_MAX_S           10.0f    // bench raw PWM test ('l', 'r', 'b'): longest allowed duration, s

// ---------------------------------------------------------------- gyro [fw]
// MPU6050 yaw gyro, probed at boot on every robot (found = used, not found = encoders only). The heading step
// between camera fixes comes from the gyro instead of the wheel difference (encoder-only turns slip on robot 1:
// 79-95 deg for the same counts). Not modelled in the twin: it only makes the real heading better.
#define PIN_I2C_SDA         21       // [ref]
#define PIN_I2C_SCL         22       // [ref]
#define GYRO_ADDR           0x68     // [ref] MPU6050
#define GYRO_LSB_PER_DPS    65.5f    // [ref] +-500 deg/s range
#define GYRO_BIAS_SAMPLES   400      // [ref] readings averaged at boot, 2 ms apart, robot still
#define GYRO_I2C_TIMEOUT_MS 20       // I2C timeout of one read (gyro task only; the control task never waits on it)
#define GYRO_MAX_FAILS      100      // failed reads in a row before the gyro is switched off (0.5 s)
#define GYRO_SIGN_MIN_RATE  1.0f     // rad/s from the encoders: above this the turn direction is certain
#define GYRO_SIGN_SAMPLES   20       // ticks of such turning before the gyro sign is judged (0.1 s)
#define GYRO_DEAD_RATE      0.2f     // rad/s: mean |gyro rate| below this while the encoders turn = the chip does not follow, gyro off
#define GYRO_PERIOD_MS      5        // gyro task period (= control period)
#define GYRO_STALE_MS       20       // a reading older than this is not used (heading falls back to the encoders)
#define GYRO_TASK_CORE      1
#define GYRO_TASK_PRIO      10       // below control (20), above console
#define GYRO_TASK_STACK     3072
#define PIN_TOF_XSHUT_LIST  23, 19, 18, 17, 16   // [ref] XSHUT of the 5 unused ToF sensors: held low = shutdown

// ---------------------------------------------------------------- encoders [fw]
// 1 = ESP32 PCNT hardware counters (x4 quadrature, glitch filter, no CPU load per edge); 0 = GPIO interrupts
// (the previous firmware). Both give the same counts and signs (ENC_x_DIR).
#ifndef ENC_USE_PCNT
#define ENC_USE_PCNT        1
#endif
#define PCNT_FILTER_NS      1000     // ignore pulses shorter than 1 us (also the 80 ns GPIO 36/39 dips with WiFi on);
                                     // shortest real edge spacing at top speed is about 70 us
#define PCNT_LIMIT          30000    // hardware counter range +-PCNT_LIMIT; it is unwrapped to 32 bit every tick

// ---------------------------------------------------------------- bench build [fw]
// -DBENCH_SIM=1 (envs botN_bench): a BARE ESP32 (no motor driver, no encoders, no gyro) behaves like a moving robot.
// Only the lowest layer changes (motors.cpp, encoders.cpp, gyro.cpp); tasks, queues, MQTT, control laws, wheel PI and
// watchdogs are the real code. motors: no GPIO touched, the signed PWM is only recorded. encoders: counts come from a
// wheel model fed by that PWM (speed = sign * max(0, |pwm| - PWM_DEADBAND) / PWM_PER_MPS, first-order lag
// BENCH_MOTOR_TAU_S, CPR_x / WHEEL_DIAMETER_M for counts). gyro: off, no I2C, no ToF pins.
#ifndef BENCH_SIM
#define BENCH_SIM 0
#endif
#define BENCH_MOTOR_TAU_S   0.05f    // s, first-order lag of the simulated wheel speed (real: MOTOR_TAU_S 0.08 incl. PI loop)
#define BENCH_MAX_STEP_S    0.25f    // s, longest elapsed time integrated in one encodersRead() (a stalled task is not a jump)
