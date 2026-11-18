# twin3d handover

For a new session (any model) or the user, without the people who built it. Read this first, then the folder READMEs it points to.
Only numbers that were checked against the code or by running it are quoted.

## 1. What it is
Three small ESP32 differential-drive robots (encoders only, overhead ArUco camera) act as haul trucks on a 1 m x 1 m arena that
stands for a 200 m x 200 m dump site (1:200). A central MATLAB "brain" fills the pile spots of a plan exported from the ZADP web
app (`zadp_game.html`), with no collisions and no deadlock. The twin leads: it simulates the robots and shows them in 3D (trucks
tip a bed, cone piles form). Real robots can follow its commands over MQTT. On hardware the robot only drives to the spot and back.
Needs only MATLAB R2026a, Simulink, Simulink 3D Animation, Stateflow (the Unreal viewer is NOT used).

## 2. Architecture and data flow
```
 zadp_game.html (ZADP web app, Rounds 1-4)
      | tools\export_plan.py --arena plan\arena_site.json   (Playwright + Chrome, page served on :8777)
      v
 plan\plan_arena.json --load_plan--> plan struct
      v
 brain  (brain_init / brain_step / brain_run)  <--- params\twin_params.m (P, single source of numbers)
   |  per robot: bot_step (physics) + bot_controller + cam_measure + pose_fusion        [bot\]
   |  dispatch, sections (locks), routes, dumping, piles, safety monitors                [brain\]
   +--> log / world W --> world_play / world_live / BrainSystem+View3DSystem --> 3D     [world3d\, simulink\]
   +--> link_from_brain (route-step changes -> commands)                                [link\]
            MATLAB --TCP/JSON--> bridge.py --MQTT--> broker --> ESP32 x3   (or fake_bot.py in tests)
            MATLAB <-- telemetry, cam pose <-- bridge.py <-- broker <-- ESP32 x3, aruco_node.py (camera)
```
The brain always runs the simulated robots; the real robots only mirror each route step (`goto`, `turn`, `dump`, `stop`).

## 3. Folders and important functions
Start every session: `run('E:\catrover_dumpautonomy\twin3d\startup.m')` (puts the subfolders on the path).

**params\\**
- `twin_params()` returns `P`: every number (see section 6).
- `pile_geometry(V, reposeDeg)` cone radius and height from volume.

**plan\\**
- `plan_arena.json` is the exported plan; `arena_site.json` is the input of the exporter (site metres).
- `load_plan(file)` reads a plan and normalises the arrays jsondecode returns. `plot_plan(plan, 'off')` 2D overview.

**bot\\** (one robot, phase 3)
- `bot_init(P, pose0, id)` state; `bot_step(s, cmd, P, dt)` motor lag, accel limit, encoders, slip, true pose.
- `bot_odometry` dead reckoning from counts. `bot_controller(pose, target, P)` go-to-point (mirror of the firmware).
- `cam_measure(truePose, P)` noisy camera fix. `pose_fusion(s, z, P)` pulls the estimate to the fix (gain 0.5).
- `bot_goto` runs controller + physics to arrival. `demo_one_bot` one robot entry to exit.

**brain\\** (dispatcher, phases 4-5; details in `brain\README.md`)
- `brain_init(plan, P, seed)` world `W`. `brain_step(W, P, dt)` one step. `brain_run(plan, P, opts)` whole run + safety monitors + log.
- `brain_layout`, `brain_sections` junction points, apron loop, lane sections. `brain_rank` fill order (Rules 1, 1b, 4).
- `brain_spot_plan`, `brain_piles_changed`, `brain_obstacles` plan cache, invalidation when a pile lands near a planned path.
- `route_to_spot` full trip P1 -> dump stop -> P3; `route_exit`, `route_exit_points`, `route_exit_tail` the way out;
  `route_apron` queue advance; `route_steps` builds drive/turn/dump steps; `route_new`, `route_check`, `route_needs` swept-body checks and the sections a move needs.
- `body_outline`, `rect_circle_clear`, `rect_pair`, `in_allowed` geometry. `brain_coverage` union area / site area, tonnes.
- `pile_from_payload(t, P)` payload to pile. `pile_growth(frac, r, h)` pile during the dump. `brain_plot`, `demo_brain` figure and demo.

**world3d\\** (3D in base MATLAB graphics; `world3d\README.md`)
- `world_build` scene. `world_view` camera (`top|orbit|gate|follow`). `truck_build`, `truck_pose` haul truck and bed tilt.
- `pile_draw`, `pile_update` piles. `dump_profile` bed angle and pile fraction in a dump. `world_sync`, `world_hud` state and HUD.
- `world_play(log, plan, P, opts)` replay (+ MP4). `world_live` live view while the brain computes. `coverage_curve`, `brain_dump_state`, `brain_view_state`.
- `solid_box`, `solid_prism`, `solid_wheel` primitives. `demo_3d` writes `demo_3d.png` and `demo_3d.mp4`.

**simulink\\** (`simulink\README.md`)
- `zadp_twin.slx`: `BrainSystem` (source) -> `View3DSystem` (sink), fixed step 0.02 s. `build_twin_model` rebuilds it, `run_twin_model(60)` runs it.

**link\\** (MQTT, phase 7; `link\README.md`, `link\TOPICS.md`)
- `link_open`, `link_send_cmd`, `link_poll`, `link_close` MATLAB side (base MATLAB `tcpclient`). `link_keepalive` repeats the last command every 100 ms, `link_stop_all`.
- `link_from_brain(W, Wprev, L)` route-step changes to commands. `brain_run_linked` brain with the link, paced to real time, `opts.syncToReal`.
- `link_launch`, `link_kill`, `link_python` start/stop and find the Python processes (PID checked).
- `py\bridge.py` TCP <-> MQTT (or loopback router). `py\fake_bot.py` stand-in for the ESP32s and camera. `py\transport.py` clients. `py\aruco_node.py` camera node (never run on a camera).

**tests\\** `test_*.m` scripts and `run_all_tests.m`. **Repo root `tools\`**: `export_plan.py` (plan export), `make_screenshots.py` (web app screenshots).

## 4. Contracts in brief
- **plan.json** (`plan\load_plan.m`, written by `exportPlan` in `zadp_game.html`): `version`, `site`, `scale`, `arena{w,h}`, `boundary`, `safe`, `entry{pt,n,t}`,
  `spine{poly, inLine, outLine, width}`, `zones[{n, poly, branch, branchDepth, depth}]`, `trucks[{id, name, payload, count, pileR, L, W, turnR}]`,
  `spots` keyed by truck id, each `{x, y, zone, kind (zone|branch|lane), order}`. Arena metres. Current plan: 125 spots (72 zone, 14 branch, 39 lane), pile radius 0.0353 m, one truck type.
- **MQTT** (full text and the ESP32 checklist in `link\TOPICS.md`): `zadp/bot{1-3}/cmd` `{id, truck, seq, action, x, y, heading, speed[, dir]}`;
  `zadp/bot{1-3}/telemetry` `{id, seq, encL, encR, x, y, heading, state}` at 20 Hz; `zadp/cam/pose` `{bot, x, y, heading, t}` at 30 Hz.
  Same `seq` = keep-alive, new `seq` replaces the command. Robot stops after 300 ms without a command line. `arrived` only when the wheels have stopped.
  Frame: arena, origin lower left, x right, y up, heading CCW from +x, metres and radians.

## 5. Brain rules and lane sections (plain words; each is also a comment in the code)
1. Fill order: in a zone take the lowest plan order that is reachable now; use the other zone when the first is locked. 1b: skip a spot whose pile would cut off the approach of another reachable spot until that one is filled. Branch spots after their zone.
2. Zone lock: a zone with its branch road has one holder at a time.
3. Lane mutex: the drivable area is cut into sections; a robot may start a move only while it holds every section its swept body (+8 mm) touches.
4. Spine (lane) spots last: only when no zone or branch spot is reachable and no zone trip is running; deepest first; one truck, everybody else parked.
5. Dumping: rear to the spot, stop = spot + u*D with D = pile radius + axle-to-back + 8 mm (longer only if blocked), wait `P.truck.dumpTime`; the pile appears where the truck really stopped and stays an obstacle forever.
6. Never start what you cannot finish: the whole inbound route and zone lock are granted at once before a truck leaves the queue head; the whole exit route before it leaves the dump stop. Waiting happens only in apron slots or at the dump stop.
7. Deadlock watch: a wait cycle of 10 s, or nobody moving for 60 s while trips are open, is reported.
8. End: when nothing can be served all robots park in P1..P3; each unfilled spot keeps the reason its last plan failed.

Sections: `Z1`, `Z2` (zone + branch), `IN_LOW`, `OUT_LOW` (each lane below the junction plus the apron strip), `J` (spine width around the branch roads), `UP` (spine above), apron `APR_R`, `QW`, `QM`, `Q2S`, `Q3S`.
Apron loop (one way): queue row heading west, P1 under the IN lane -> IN lane -> site -> OUT lane -> return row east -> east column down -> P3 -> P2 -> P1. Robots are forward-only plus in-place turns.
Parking is a rolling queue: a returning robot always parks at P3 and moves up to P1. Every entry from P1 is a new truck. Why there is no deadlock: `brain\README.md`.

## 6. Parameters (`twin_params.m`)
`P.bot` real robot (wheel 34 mm, track 75.3 mm, 2772.2 counts/rev, 300 rpm, 0.20 m/s cruise, 0.5 m/s^2, slip 1 %, chassis 0.100 x 0.084 m, axle to nose 0.076 / back 0.024) | `P.arena`, `P.scale` (1 m, apron 0.25 m, 0.005 arena m per site m) |
`P.cam` (30 Hz, 3 mm, 1 deg, gain 0.5) | `P.sim` (dt 0.01 s, used by bot-only tests) | `P.truck` visual only (size, bed tilt 50 deg, dumpTime 4 s, colours) |
`P.pile` (repose 37 deg, density 1.8 t/m^3, load factor 0.85..1.00) | `P.ctrl` onboard controller gains and tolerances (arrive 5 mm, heading 3 deg) |
`P.brain` (dt 0.02 s, clearances 7 / 4 / 8 mm, dump margin, approach fan, parking slots P1..P3, deadlock times) | `P.mqtt` broker, port, topic formats.
`tests\test_params.m` checks the groups exist; add a group there when you add one.

## 7. Pile maths
`V = payload / rho`, `r = cbrt(3 V / (pi tan(repose)))`, `h = r tan(repose)`, so `V = pi r^2 h / 3`; no shape factor; same as `calc(t)` in `zadp_game.html`.
Computed at site scale, drawn at 1:200. Payload = plan payload x a load factor uniform in [0.85, 1.00] (own seeded stream `1000 + seed`).
500 t: site r 7.06 m, h 5.32 m, V 277.8 m^3; arena r 35.3 mm, h 26.6 mm. 425 t: site r 6.69 m, h 5.04 m; arena r 33.4 mm, h 25.2 mm.
The real radius is capped at the planned one, so planned clearances still hold. During a dump `pile_growth` scales r, h with frac^(1/3).

## 8. Scaling (`SCALING.md`)
| Quantity | Arena | Site |
|---|---|---|
| Arena / site | 1 m x 1 m | 200 m x 200 m |
| Truck | 0.100 x 0.084 m | 20.0 x 16.8 m |
| Pile radius | 0.0353 m | 7.06 m |
| Spine (two lanes) | 0.240 m | 48 m |
| Turning radius | 0.120 m | 24 m |
| Boundary clearance to arena edge | 0.05 m | 10 m |
| Cruise speed | 0.20 m/s | 40 m/s (hardware runs about 4.8x faster than a 30 km/h hauler; site time = sim time x 4.8) |

The scale is fixed by the robot size. A bigger site needs a bigger arena or a smaller robot.

## 9. Make a new arena plan
1. Edit `plan\arena_site.json`: `poly` (site metres, here 10..190), `entryEdge` (index of the polygon edge that is the gate), `laneW` (one lane, site metres), `fleet` (payload, count, L, W in site metres), `scale` 0.005.
2. In the repo root: `python -m http.server 8777` (leave running), then `python tools\export_plan.py --arena twin3d\plan\arena_site.json` (optional `--rows N` = pile rows each side of a branch). Needs Playwright and Chrome. Writes `twin3d\plan\plan_arena.json`.
3. In MATLAB: `plan = load_plan('E:\catrover_dumpautonomy\twin3d\plan\plan_arena.json'); plot_plan(plan,'off')` to look at it, then `run_all_tests`.
4. `test_plan` pins the current design (1 x 1 m, one truck type, count 3, 0.100 x 0.084 m, pile radius 0.02..0.06 m, spine at least 0.24 m, zones at least 0.084 m wide, boundary at least 0.04 m from the edge). A different design needs those asserts reviewed on purpose.
5. The brain layout (`brain_layout`, `P.brain.parking`, `P.brain.apronRowY`) is built for this plan's two-zone, one-spine shape with P1 under the IN lane; a different shape needs layout and section work, not only a new plan. Spots the brain cannot reach are reported, not hidden.

## 10. Run each demo (MATLAB MCP, after `startup`)
| Command | What | Time |
|---|---|---|
| `demo_brain` | full fill, seed 1, prints piles, coverage, monitors, saves `brain\demo_brain.png` | about 2 min wall |
| `demo_one_bot` | one robot entry to exit, plot | short (not timed) |
| `demo_3d` | brain 240 s, writes `world3d\demo_3d.png` and `.mp4` (30 s, 30 fps) | about 3 min (the 901-frame render alone about 110 s) |
| `[W,log]=brain_run(plan,P); world_play(log,plan,P,struct('speed',40))` | replay a run in 3D | visible figure |
| `world_live(plan,P,struct('maxTime',300,'speed',10))` | watch the brain compute | visible figure |
| `build_twin_model; r = run_twin_model(60)` | Simulink model, 60 s | about 15 s wall |
| `demo_link` | 120 s linked run against `fake_bot.py`, saves `link\demo_link.png` | 120 s wall |

Calls longer than about 60 s need a timer or slices (`opts.maxTime`), see `CLAUDE.md`.

## 11. Tests (`tests\run_all_tests`)
| Test | Proves |
|---|---|
| `test_params` | all parameter groups exist, robot and scale numbers, truck visual = robot footprint, `pile_geometry` volume law |
| `test_plan` | `plan_arena.json` loads, zones/orders consistent, spots inside the boundary, 1:200, three robot-size trucks, lane and zone widths |
| `test_bot` | one wheel turn = 2772 counts, straight and turn-around accuracy, speed and accel limits, odometry drifts without camera, camera keeps arrival error under 10 mm |
| `test_piles` | pile formula vs plan radius, volume law, growth, truck numbers 1,2,3.. with no repeats, pile records complete, coverage maths, single parking source, monitors zero over 300 s |
| `test_world3d` | 3D world handles, truck footprint and bed tilt, pile size, replay and live frames, Simulink model runs 5 s (hidden figures) |
| `test_link` | command reaches fake robot, payloads match `TOPICS.md` field for field, 30 s linked run error and latency, syncToReal, 300 ms watchdog, no processes left; SKIPs without Python |
| `test_brain` | 3 seeds: zero overlaps, pile hits, off-site, deadlocks; piles within 15 mm of spot; at least 85 % filled; zone lock; two robots inside at once |

Last run: ALL TESTS PASSED in 404.9 s; `test_brain` 379 s, everything else about 26 s together. `test_brain` is the slow one: run it in slices or via a timer.

## 12. Measured results (verified runs)
- Seed 1 full run: 123 of 125 piles (spots 41 and 100 unfilled), coverage 57.9 %, 57148 t dumped, 123 trucks numbered, sim time 2851 s. Seeds 2 and 3: 124 of 125. Piles span r 6.70-7.06 m, payload 426-500 t.
- Safety monitors all zero. Min robot gap 12.7 mm, min pile gap 4.9 mm (seed 1). The 300 s test run numbered 14 trucks.
- 3D: replay 8.2 frames/s (1280 x 720, hidden); `demo_3d.mp4` 30.0 s, 901 frames, 11.4 MB (limit 15 MB). Simulink 60 s: 2 piles, coverage 0.919 %, collisions 0 (`brain_run` 60 s: 2 piles, 0.918 %).
- Link (loopback, fake robots): demo 120 s, 115 commands, 115 of 115 acknowledged, twin-vs-real position error mean 5.2 mm, worst 24.8 mm, worst heading error 11.6 deg, command round trip mean 12.9 ms (worst 24.7), twin violations 0. 20 s syncToReal run: worst error 23.7 mm.
- No broker was present (paho-mqtt 2.1.0 installed, no mosquitto): the MQTT broker path was never exercised here.

## 13. Known limits
- 1-2 spots stay unreachable (41 in zone 1, 100 in zone 2, depending on noise): inner-row branch spots sit in a pocket between the branch outer row and the zone's last row; the channel left (about 0.057 m) is narrower than the truck (0.084 m), and the forward-only robot needs a 180 degree in-place turn at the dump stop (nose sweep radius 0.087 m).
- Odometry only drifts (about 1 % of distance with slip 1 %); the camera fix (gain 0.5) is what keeps arrival errors small. Camera latency (50 ms) is not modelled or compensated.
- `aruco_node.py` is written but NEVER run against a camera; expect to tune it (no undistortion; the apron outside the four corner markers is extrapolated).
- The real firmware still speaks TCP port 23, not MQTT. The MQTT broker path of `bridge.py` is untested here.
- A fixed parking bay per robot is NOT implemented: a rolling queue P3 -> P2 -> P1 is used (a fixed bay would need a wider apron and breaks the all-or-nothing section grants).
- Lane (spine) spots are filled last, one truck at a time, deepest first; the others wait parked.
- Three robots stand for an endless stream of trucks; truck numbers are labels, not separate robots.
- Piles exist only in the twin. The 3D view is base MATLAB graphics (software rendering, about 8 frames/s at 1280 x 720).
- Brain and link are tuned for the current plan shape (section 9, point 5). The web app's truck order setting (random, big first, ...) does not matter here: one truck type.
- Memory: do not start a second MATLAB or the Unreal viewer on the 16 GB PC.

## 14. What the user does next (real robots)
1. Firmware (ESP32): join MQTT per the checklist in `link\TOPICS.md` (client id `zadp-bot<N>`, subscribe `cmd`, publish telemetry at 20 Hz and once on each new command, same controller numbers, 300 ms watchdog, `arrived` only when stopped).
2. Broker: install Mosquitto, `listener 1883` and `allow_anonymous true` on a closed test network, run `mosquitto -v`. `pip install paho-mqtt` (and `opencv-python numpy` for the camera). Set `P.mqtt.broker` to the broker PC address; the firmware needs the same address.
3. Test the PC side with fake robots against the real broker: `python link\py\fake_bot.py --bots 1,2,3 --broker localhost`, and `python link\py\bridge.py --broker <ip> --broker-port 1883 --no-fallback --port 18830`.
4. Camera: print ArUco DICT_4X4_50 markers (ids 1-3 on the robots, centred over the axle, up edge to the nose; ids 10-13 at the arena corners, at least 25 mm). Mount the camera straight above, fixed exposure, whole arena plus apron in view. Run `python link\py\aruco_node.py --camera 0 --broker localhost --show`, check poses against a ruler, tune (undistortion, apron markers).
5. First single-robot test: put one robot on parking slot P1 (`P.brain.parking`). The link has no one-robot mode: the twin runs all three robots and sends commands to ids 1-3, so commands for robots 2 and 3 get no answer. Therefore run WITHOUT `syncToReal` first (with it the twin would wait `holdTimeout` = 20 s for each missing robot), and read only robot 1 in `lk`. In MATLAB: `L = link_open(P, struct('port',18830)); [W,log,lk] = brain_run_linked(plan, P, struct('link',L,'maxTime',60)); link_close(L);` Keep a hand near the power switch. `link_close` sends stop to all robots.
6. Raise `maxTime` step by step, compare twin and real error in `lk`. Use `syncToReal = true` only when every real robot present answers.
7. Then three robots on P1..P3, short runs first, full fill last. Hardware runs at 0.20 m/s, so a full fill (2851 s sim time) takes as long in wall time.
