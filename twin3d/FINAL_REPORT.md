# Final report: twin audit, maps, hardware integration, RTOS firmware

Date: 2026-11-22. Numbers below were measured in the runs named; nothing is estimated.

## 1. Test status

Full `run_all_tests` (10 tests, 493 s): 9 passed, `test_link` failed once (36 mm twin-vs-real against its 30 mm limit, while another job loaded the PC). Re-run alone three times on an idle PC: 3 of 3 passed.

| Test | Result |
|---|---|
| `test_brain` | arena seeds 1/2/3: 125 / 125 / 125 of 125 piles; min robot gap 12.1 / 11.5 / 9.9 mm; every monitor 0 |
| `test_maps` | 8 maps x 400 s sim, 17 to 19 piles each, every monitor 0 |
| `test_rules` | section ownership 0 violations; forced wait cycle and forced stall both detected |
| `test_bot` | 40 random go-tos: true error mean 3.51 mm, max 5.81 mm |
| `test_link` (3 repeats) | worst twin-vs-real 22.5 to 26.6 mm (limit 30 mm), twin violations 0 |
| `test_run_real` (dry) | 20 s: error mean 3.8 mm, worst 13.9 mm; park, corner and view-close failure paths stop cleanly |
| `test_firmware` | 43 firmware numbers match `twin_params`; motion_core mirror 47 passed; camera geometry 10 passed |
| `test_world3d` | Simulink 2 x 30 s identical and equal to `brain_run` (2 piles, 0.9184 %) |
| `test_params`, `test_piles`, `test_plan` | passed |

`test_link` compares wall-clock timing and has a thin margin (about 4 mm): run it with the PC idle.

## 2. Full fills on other shapes (seed 1, `demo_maps`)

| Map | Spots | Piles | Coverage % | Violations |
|---|---|---|---|---|
| arena (pentagon) | 125 | 125 | 58.9 | 0 |
| rectangle | 86 | 86 | 54.5 | 0 |
| L shape | 82 | 82 | 54.0 | 0 |
| hexagon | 98 | 97 | 55.8 | 0 |

Hexagon: lane spot 36 cannot be reached because a zone pile next to the spine blocks the dump stop. Not fixed (needs lane spots in the Rule 1b cut-off test, or more spacing in the planner).

Supported map family: any simple polygon, bottom entry, 2 zones (one per side), 1 branch each. Others are refused by `brain/assert_plan_supported.m` with a clear message.

## 3. What was fixed in the twin

- Lane sections were released at the start of each drive piece instead of the end (512 ownership violations in 40 s before, 0 after). New ownership monitor.
- Camera latency (50 ms) is modelled and compensated: along-track bias -10.4 mm before, -0.30 mm after.
- Controller: 3 cm capture radius and rotate hysteresis (no spinning at the target), line following, predicted braking.
- Deadlock detector false negatives; runs now end at a deadlock.
- Simulink model: reproducible runs, stop block, plan and seed parameters.
- Numbers moved into `twin_params.m`; monitors made faster (about 85 s per full fill).
- Link: loopback is the default so tests cannot reach real robots; time-based seq; stale camera poses ignored.

## 4. Hardware integration

- One contract: `link/TOPICS.md`. All 18 ambiguities found by the firmware author are decided there.
- `run_real()` is the one command for hardware; `run_real('dry')` runs the same path with fake robots and a simulated camera.
- Bridge repeats the last command at 10 Hz while MATLAB is alive, so a brain stall does not trip the 300 ms robot watchdog. MATLAB closed: robots stop in 0.3 s; MATLAB hung: 1.3 s.
- Camera node `link/py/aruco_node.py` and marker generator `link/markers/make_markers.py`.
- Guide: `docs/ZADP_Hardware_Twin_Guide.pdf` (8 pages).

## 5. Firmware (`firmware/zadp_bot`), FreeRTOS

| Task | Core | Priority | Rate | Stack (bytes) |
|---|---|---|---|---|
| control | 1 | 20 | 200 Hz | 4096 |
| telemetry | 0 | 6 | 20 Hz + at once on a new seq | 4096 |
| comms (WiFi, MQTT) | 0 | 5 | 5 ms | 6144 |
| console (serial bench test) | 1 | 2 | 10 ms | 4096 |
| health (LED) | 1 | 1 | 20 ms | 2048 |

Queues carry commands and camera poses to the control task; encoders use the hardware pulse counters. Three reviewers (concurrency, contract, build) found 11 issues; all are fixed, including an odometry step that lost precision in 32-bit floats. Pinned: `espressif32 @ 7.0.1`, `PubSubClient @ 2.8`, `ArduinoJson @ 7.2.1`.

## 6. Web app (separate commit)

Fixed: input guards (pile radius, truck count, absurd fleets), name escaping, damaged saved session, stale banner, export button now writes the 1 x 1 m 1:200 plan, favicon, per-frame time budget in play. Reference sites unchanged: valley 75.93 %, crescent 67.44 %, two lobes 76.02 %, waste 76.14 %, 0 collisions, 0 deadlocks; arena plan export byte-identical.

Open, not changed: the collision check and the route grid treat a truck as a disc sized by its width (bodies can pass over piles by up to about 7 m); L-paddock arm (fix still in `stash@{0}`); layout at 1366 x 768.

## 7. Not verified

- The firmware has never been built by PlatformIO or run on an ESP32. It was syntax-checked with the ESP32 compiler (104 compiles, 0 errors) against stand-in headers for the two MQTT/JSON libraries. The first `pio run -e bot1` needs internet once.
- No real MQTT broker and no camera were used. OpenCV is not installed on this PC, so ArUco detection was never run and the marker sheets are not generated.
- New maps were filled with seed 1 only.
- Pages 5 and 7 of the guide PDF were not checked visually.
