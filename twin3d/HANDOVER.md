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
- Three more arena shapes, same fleet and scale: `arena_rect.json` (180 x 130 m rectangle), `arena_lshape.json` (L, arm up
  the left side, non-convex), `arena_hex.json` (irregular hexagon) -> `plan_<name>.json` / `.png`. `brain\demo_maps` runs the
  full fill on all four and writes `maps_results.md`, `fill_<name>.png`, `fill3d_<name>.png`.

**maps\\** four more map shapes (`<name>_site.json` -> `plan_<name>.json`: trapezoid, chamfered rectangle, mirrored pentagon,
octagon), tested by `test_maps`; `maps\README.md`.
- `load_plan(file)` reads a plan and normalises the arrays jsondecode returns. `plot_plan(plan, 'off')` 2D overview.

**bot\\** (one robot, phase 3)
- `bot_init(P, pose0, id, rs)` state (rs = random stream, the brain passes its own); `bot_step(s, cmd, P, dt)` motor lag,
  accel limit, encoders, slip, true pose, trapezoidal wheel travel, short pose history `s.hist`.
- `bot_odometry` dead reckoning from counts. `[cmd, arrived, rot] = bot_controller(pose, target, P, rot)` go-to-point
  (mirror of the firmware): rotate-in-place with hysteresis (enter 25 deg, leave 15 deg), never inside the 3 cm capture
  radius (there it drives, backing up if the goal is behind, with the heading correction fading to 0).
- `[z, tz] = cam_measure(s, P)` noisy camera fix of the pose `P.cam.latency` (50 ms) ago. `pose_fusion(s, z, P, tz)`
  moves the fix forward by the odometry since `tz` (latency compensation) and pulls the estimate to it (gain 0.3).
  `hist_at` interpolates the pose history.
- `bot_goto` runs controller + physics to arrival. `demo_one_bot` one robot entry to exit.

**brain\\** (dispatcher, phases 4-5; details in `brain\README.md`)
- `brain_init(plan, P, seed)` world `W` (checks the map with `assert_plan_supported`; own random stream `W.rs`). `brain_step(W, P, dt)` one step.
  `brain_run(plan, P, opts)` whole run + safety monitors + log (ends at a deadlock). `brain_monitor` the monitors (robot-robot,
  robot-pile, off-site, section ownership) for every viewer; `brain_monitor_collisions` the one "collisions" number;
  `brain_owner_check` the Rule 3 invariant.
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
- `zadp_twin.slx`: `BrainSystem` (source) -> `View3DSystem` (sink), fixed step 0.02 s, stop time 600 s, Stop block on
  `done`, annotation on the canvas, path callbacks (opens in a fresh MATLAB). `build_twin_model` rebuilds it,
  `run_twin_model(60)` runs it (hidden view, model left unchanged).

**link\\** (MQTT, phase 7; `link\README.md`, `link\TOPICS.md`)
- `link_open`, `link_send_cmd`, `link_poll`, `link_close` MATLAB side (base MATLAB `tcpclient`). `link_keepalive` = heartbeat to bridge.py (the bridge repeats each robot's last command at 10 Hz while MATLAB was heard within 1 s), `link_stop_all`. `run_real` = one-command launcher (real / dry), `link_write_config` = numbers for the Python side and the firmware check.
- `link_from_brain(W, Wprev, L)` route-step changes to commands. `brain_run_linked` brain with the link, paced to real time, `opts.syncToReal`.
- `link_launch`, `link_kill`, `link_python` start/stop and find the Python processes (PID checked).
- `py\bridge.py` TCP <-> MQTT (or loopback router). `py\fake_bot.py` stand-in for the ESP32s and camera. `py\transport.py` clients. `py\aruco_node.py` camera node (never run on a camera).

**Repo root `firmware\zadp_bot\`** (ESP32 robot firmware, PlatformIO + arduino-esp32 core 2.0.17, FreeRTOS; details in
`firmware\README.md`). Five pinned FreeRTOS tasks, no polling loop: `control` (core 1, prio 20, 200 Hz `vTaskDelayUntil`:
PCNT encoders, odometry, camera fusion, go-to / turn laws, wheel PI, PWM, 300 ms command watchdog, 0.3 s link-loss stop),
`telemetry` (core 0, prio 6, 20 Hz + at once on a new seq), `comms` (core 0, prio 5, WiFi + MQTT state machine with
backoff, JSON parsing), `console` (core 1, prio 2, serial bench self-test, boot stack report), `health` (core 1, prio 1,
LED). Queues: `ctrlQ` (commands, FIFO), `camQ` and `snapQ` (length 1, overwrite). The task watchdog (1 s, reset) watches
only the control task. The maths is `include\motion_core.h`, mirrored by `test\host\js_mirror\mirror_test.js`; its
numbers are checked against `twin_params.m` by `test_firmware`.

**tests\\** `test_*.m` scripts and `run_all_tests.m`; `check_brain_run` (one full fill with every assert, used by
`test_brain` and `test_maps`). **Repo root `tools\`**: `export_plan.py` (plan export, `--out` for other maps),
`make_screenshots.py` (web app screenshots).

## 4. Contracts in brief
- **plan.json** (`plan\load_plan.m`, written by `exportPlan` in `zadp_game.html`): `version`, `site`, `scale`, `arena{w,h}`, `boundary`, `safe`, `entry{pt,n,t}`,
  `spine{poly, inLine, outLine, width}`, `zones[{n, poly, branch, branchDepth, depth}]`, `trucks[{id, name, payload, count, pileR, L, W, turnR}]`,
  `spots` keyed by truck id, each `{x, y, zone, kind (zone|branch|lane), order}`. Arena metres. Current plan: 125 spots (72 zone, 14 branch, 39 lane), pile radius 0.0353 m, one truck type.
- **MQTT** (the single contract: `link\TOPICS.md`): `zadp/bot{1-3}/cmd` `{id, truck, seq, action, x, y, heading, speed[, dir, tol]}`;
  `zadp/bot{1-3}/telemetry` `{id, seq, encL, encR, x, y, heading, state}` at 20 Hz; `zadp/cam/pose` `{bot, x, y, heading, t, age}` at 30 Hz.
  Same `seq` (uint32) = repeat, new `seq` replaces the command. bridge.py repeats the last command at 10 Hz while MATLAB was heard within 1 s;
  the robot stops after 300 ms without a cmd message. `arrived` only when the wheels have stopped (also for `stop`). goto: no final turn.
  Numbers for the Python side and the firmware check: `link_write_config` -> `link\link_config.json`.
  Frame: arena, origin lower left, x right, y up, heading CCW from +x, metres and radians.

## 5. Brain rules and lane sections (plain words; each is also a comment in the code)
1. Fill order: in a zone take the lowest plan order that is reachable now; use the other zone when the first is locked. 1b: skip a spot whose pile would cut off the approach of another reachable spot until that one is filled. Branch spots after their zone, unless every zone spot left would cut a spot off (then a branch spot that cuts nothing off goes first: the inner-row pocket).
2. Zone lock: a zone with its branch road has one holder at a time.
3. Lane mutex: the drivable area is cut into sections; a robot may start a move only while it holds every section its swept body (+8 mm) touches. Long drives are cut into 5 cm pass-through pieces; a section is released only when the robot has passed the end of the last piece that needs it. Invariant, checked every step (monitor e): every section a true body touches is held by that robot.
4. Spine (lane) spots last: only when no zone or branch spot is reachable and no zone trip is running; deepest first; one truck, everybody else parked.
5. Dumping: rear to the spot, stop = spot + u*D with D = pile radius + axle-to-back + 8 mm (longer only if blocked), wait `P.truck.dumpTime`; the pile appears where the truck really stopped and stays an obstacle forever.
6. Never start what you cannot finish: the whole inbound route and zone lock are granted at once before a truck leaves the queue head; the whole exit route before it leaves the dump stop. Waiting happens only in apron slots or at the dump stop.
7. Deadlock watch: wait-for graph (sections and queue slots); a cycle for 10 s, or nobody moving or dumping for 60 s before the run has ended, is a deadlock and ends `brain_run`.
8. End: when nothing can be served all robots park in P1..P3; each unfilled spot keeps the reason its last plan failed.

Sections: `Z1`, `Z2` (zone + branch), `IN_LOW`, `OUT_LOW` (each lane below the junction plus the apron strip), `J` (spine width around the branch roads), `UP` (spine above), apron `APR_R`, `QW`, `QM`, `Q2S`, `Q3S`.
Apron loop (one way): queue row heading west, P1 under the IN lane -> IN lane -> site -> OUT lane -> return row east -> east column down -> P3 -> P2 -> P1. Robots are forward-only plus in-place turns.
Parking is a rolling queue: a returning robot always parks at P3 and moves up to P1. Every entry from P1 is a new truck. Why there is no deadlock: `brain\README.md`.

## 6. Parameters (`twin_params.m`)
`P.bot` real robot (wheel 34 mm, track 75.3 mm, 2772.2 counts/rev, 300 rpm, 0.20 m/s cruise, 0.5 m/s^2, slip 1 %, chassis 0.100 x 0.084 m, axle to nose 0.076 / back 0.024) | `P.arena`, `P.scale` (1 m, apron 0.25 m, 0.005 arena m per site m, site time factor 4.8) |
`P.cam` (30 Hz, 3 mm, 1 deg, latency 50 ms compensated, gain 0.3) | `P.sim` (dt 0.01 s, used by bot-only tests) | `P.truck` visual only (size, bed tilt 50 deg, dumpTime 4 s, colours) |
`P.pile` (repose 37 deg, density 1.8 t/m^3, load factor 0.85..1.00) | `P.ctrl` onboard controller gains and tolerances (arrive 5 mm, heading 3 deg, rotate 25 / 15 deg hysteresis, capture radius 3 cm) |
`P.brain` (dt 0.02 s, clearances 7 / 4 / 8 mm, dump margin, approach fan, apron return row y -0.0555, parking slots P1..P3 at y -0.159, drive pieces 5 cm, line-following lookahead 0.15 m, junction and exit margins, deadlock times) | `P.mqtt` broker, port, topic formats |
`P.bot.trackScale` (1.0; effective track `P.bot.trackEff`, must equal firmware `TRACK_SCALE`) | `P.cam` marker ids, sizes, corner positions, heights, offset, capture delay | `P.link` watchdog 0.3 s, bridge repeats 10 Hz, hostTimeout 1.0 s, heartbeat, net delay, run_real tolerances.
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
5. Another map next to the reference one: put `<name>_site.json` in `maps` and run `python tools/export_plan.py --arena twin3d/maps/<name>_site.json --out ../maps/plan_<name>.json` (`--out` is relative to `twin3d/plan`; without it the reference `plan_arena.json` is overwritten). Add it to the list in `tests/test_maps.m`.
6. Supported map family (`brain/assert_plan_supported.m`, called by `brain_init`, refuses anything else with a clear message): simple site polygon, convex or not (an L works: `in_allowed` does exact point-in-polygon plus edge distance, `route_check` and the site monitor keep the body clear of inner (reflex) corners, `lane_turn_y` moves the lane-to-branch turn down when a branch runs close to a site edge across the spine); entry on the horizontal bottom edge with the IN lane over parking P1 (x 0.44, i.e. entry centred at x = 0.5 with the 24 m lane); exactly two zones, one each side of the spine, at most one branch each; inside the arena; one truck type. Why each limit exists, i.e. what a wider family needs:
   - non-convex sites: supported (see above); polygons with holes are not;
   - entry facing +y: `brain_layout`, `brain_sections` (sections are x/y bands), the mouth and apron rectangles, `world_build` `berm_build` (finds the entry edge by y);
   - parking: `P.brain.parking` is fixed (x 0.44 / 0.75 / 0.90, y -0.159) and P1 must sit under the IN lane; another spine x needs parking derived from the spine;
   - zones: sections `Z1`/`Z2` are hard-wired left/right (`brain_layout` `secZone`, `brain_sections`); three zones, or two on one side, need a section list built from `plan.zones`; only `z.branch{1}` is used and `route_to_spot` assumes one horizontal branch band per zone; the junction box spans all branch rows (`junctionMargin`), so branch rows far apart make it very large;
   - exits: `route_exit_points` picks the exit side by `pose(1) < xMid`;
   - fleet: `brain_init` uses `plan.trucks(1)` only.
   Spots the brain cannot reach are reported, not hidden.

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
| `test_bot` | one wheel turn = 2772 counts, straight and turn-around accuracy, speed and accel limits, odometry drifts without camera, camera keeps arrival error under 10 mm, camera latency modelled and compensated (bias under 2 mm), wheel travel independent of dt, controller never rotates in place inside the capture radius |
| `test_piles` | pile formula vs plan radius, volume law, growth, truck numbers 1,2,3.. with no repeats, pile records complete, coverage maths, single parking source, monitors zero over 300 s |
| `test_world3d` | 3D world handles, truck footprint and bed tilt, pile size, replay and live frames; Simulink model (built in a temp folder): stop time 600, annotation, Stop block, two 30 s runs in a row equal to each other and to `brain_run`, view block survives its figure being closed |
| `test_link` | loopback mode only, time-based seq base, command reaches fake robot, payloads match `TOPICS.md` field for field, 30 s linked run error and latency, syncToReal, 300 ms watchdog, no processes left; SKIPs without Python |
| `test_brain` | 3 seeds (`check_brain_run`): zero overlaps, pile hits, off-site, section-ownership violations, deadlocks, failed exit re-plans; normal end; piles within 15 mm of spot; at least 98 % filled; zone lock; lane spots last and alone; two robots inside at once |
| `test_maps` | 400 s sim (seed 1) on the 4 arena shapes in `plan\` (arena, rect, lshape, hex) and the 4 in `maps\`: all monitors 0, zone lock, two robots inside at once, a pile in every zone; a self-intersecting site is refused with a clear message. Full fills of the 4 arena shapes: `demo_maps` -> `plan\maps_results.md` |
| `test_rules` | 60 s section-ownership check; forced wait cycle and forced no-progress stall are each flagged once and end the run; `W.hold` holds and releases a robot |

Last run: ALL TESTS PASSED in 445.1 s; `test_brain` 249 s, `test_maps` 76 s (8 maps x 400 s sim). Run the suite via a timer (a blocked MATLAB MCP call times out and interrupts it).

## 12. Measured results (verified runs)
- Reference plan, seeds 1 / 2 / 3: 125 / 125 / 125 of 125 piles, sim time 2873 / 2881 / 2888 s, min robot gap 12.1 / 11.5 / 9.9 mm, min pile gap 5.3 / 3.4 / 4.4 mm, worst pile-to-spot error 6.3 / 5.4 / 6.4 mm, 0 exit re-plans.
- Arena shapes in `plan\` (`demo_maps`, seed 1, all monitors 0): arena 125 / 125, rect 86 / 86, lshape 82 / 82, hex 97 / 98
  (lane spot order 36 next to the narrowing top: its dump stop overlaps a zone pile that spills 3 mm into the spine). Table: `plan\maps_results.md`.
- Other maps (seed 1, before the non-convex change): trapezoid 109 / 109, chamfered 121 / 121, mirrored pentagon 126 / 126, octagon 115 / 115.
- Every monitor zero in all of these runs: robot-robot, robot-pile, off-site, section ownership, deadlocks. The 300 s test run numbered 15 trucks.
- 3D: replay 8.2 frames/s (1280 x 720, hidden); `demo_3d.mp4` 30.0 s, 901 frames, 11.4 MB (limit 15 MB). Simulink 30 s: 2 piles, coverage 0.9184 %, identical to `brain_run` 30 s (own random stream).
- Link (loopback, fake robots): demo 120 s, 115 commands, 115 of 115 acknowledged, twin-vs-real position error mean 5.2 mm, worst 24.8 mm, worst heading error 11.6 deg, command round trip mean 12.9 ms (worst 24.7), twin violations 0. 20 s syncToReal run: worst error 23.7 mm.
- Broker path (2026-11-23): robot 1 was verified on the floor over WiFi/MQTT through `mini_broker.py` (see section 13). The twin runs above used loopback.

## 13. Known limits
- Inner-row branch spots (41, 100) sit in a pocket that the zone's last row closes (channel 0.057 m < truck 0.084 m). They are now filled first (Rule 1b over branch-after-zone); a plan with a different pocket could still lose a spot, reported, not hidden.
- The apron is cramped: margins at the return-row corner (7.5 mm to the apron top), the queue band (9.75 mm) and the P3 turn (4 mm planned to the apron bottom) are only a few mm more than the tracking error. Stops are centred on their end point (predicted coast) and drives follow the leg line; keep that in the firmware.
- Odometry alone (camera off) drifts much faster than the 1 % slip suggests, because the left/right scale difference integrates into heading: about 1.7 % / 1.7 deg over 0.2 m, 6 % / 6 deg over 1 m, 13 % / 13 deg over 4 m (150 runs). The camera fix (gain 0.3) is what keeps arrival errors small.
- Camera latency (50 ms) is modelled (`cam_measure` returns the pose of 50 ms ago) and compensated in `pose_fusion` with the odometry since the image time; uncompensated, the estimate would lag about 10 mm at cruise, more than the 7 mm pile clearance. The firmware must do the same (`link/TOPICS.md` checklist). `fake_bot.py` does the same.
- `aruco_node.py` is written but has not been run against a camera (OpenCV is not installed); expect to tune it (no undistortion; the apron outside the four corner markers is extrapolated).
- Firmware (`firmware/zadp_bot`), as of 2026-11-23: all 5 envs build with PlatformIO. Robot 1 is flashed and was verified on the floor over WiFi/MQTT through `mini_broker.py`: straight 0.2 m (+5161 / +5171 counts, arrived); left and right 90 degree turns (gyro heading +87.4 / -87.4 deg, arrived in 1.5 s); telemetry 19.3 Hz. Not yet verified: robots 2 and 3, a real camera and `aruco_node.py` (OpenCV not installed), `run_real('real')` end to end. `run_real('dry')` tests the same code path with loopback, fake robots and a simulated camera.
- A fixed parking bay per robot is NOT implemented: a rolling queue P3 -> P2 -> P1 is used (a fixed bay would need a wider apron and breaks the all-or-nothing section grants).
- Lane (spine) spots are filled last, one truck at a time, deepest first; the others wait parked.
- Three robots stand for an endless stream of trucks; truck numbers are labels, not separate robots.
- Piles exist only in the twin. The 3D view is base MATLAB graphics (software rendering, about 8 frames/s at 1280 x 720).
- Brain and link work for the map family of section 9, point 6 (`maps\` has four more shapes). The web app's truck order setting (random, big first, ...) does not matter here: one truck type.
- Memory: do not start a second MATLAB or the Unreal viewer on the 16 GB PC.

## 14. What the user does next (real robots)
Everything below runs on the PC that runs MATLAB. Nothing is installed by the repository. No Mosquitto is needed.

1. **Python** (yourself): `pip install paho-mqtt numpy opencv-contrib-python pillow`.
2. **Network**: switch on the PC's WiFi hotspot (Windows Mobile hotspot). `secrets.h` of every robot: hotspot SSID and
   password, `MQTT_HOST` = the PC's hotspot address (usually 192.168.137.1). Keep `P.mqtt.broker = 'localhost'`.
3. **Flash** (per robot N = 1..3, `firmware\README.md`): `pio run -e botN -t upload`, then `pio device monitor -e botN`
   (boot banner, task stack report, bench self-test). A calibrated `TRACK_SCALE` goes into `P.bot.trackScale` too
   (`test_firmware` checks it).
4. **Broker**: nothing to start by hand for the run: when nothing answers on `P.mqtt.broker:P.mqtt.port` and that host
   is this PC, `run_real` (`link_launch`) starts `link\py\mini_broker.py` on all interfaces, port `P.mqtt.port`, and
   stops it at the end; a broker that already answers (Mosquitto, or a `mini_broker.py` you started) is used and left
   running. **Firewall**: the first time Python listens on that port Windows asks; allow Python on **private** networks
   (the hotspot is one), or the robots cannot connect.
5. **Check each robot alone**: `python link\py\mini_broker.py` in one shell, `python link\py\bot_ping.py --seconds 5`
   in another: one line per telemetry message (id, seq, x, y, heading, state) and the rate per robot (about
   `P.link.telemetryHz`). Stop the hand-started broker (Ctrl+C) before `run_real`, or leave it: it is then used.
6. **Markers**: in MATLAB `link_write_config(twin_params())`, then `python link\markers\make_markers.py`; print
   `robots_A4.pdf` and `corners_A4.pdf` at 100 %, check the 50 mm bar. Marker id N on robot N, centred, arrow to the
   nose. Corners 10-13 at `P.cam.cornerXY` (`placement.txt`), on blocks at the robot marker height
   (`P.cam.markerHeight` = `P.cam.cornerHeight`).
7. **Camera**: straight above the arena, whole arena + apron + corners in view, fixed focus and exposure. Alone:
   `python link\py\aruco_node.py --camera 0 --show` (prints `homography ready`, then poses).
8. **Robots** on parking slots P1, P2, P3 (`P.brain.parking`: x 0.44 / 0.75 / 0.90, y -0.159, heading west), powered.
   Only the real ones need to be there.
9. **Run**: `run('E:\catrover_dumpautonomy\twin3d\startup.m')`, then
   - one real robot: `R = run_real('real', struct('realBots', 1, 'maxTime', 60));` (robots 2 and 3 are simulated by
     `fake_bot.py` on the same broker, with their own camera pose; the camera node publishes robot 1 only)
   - two: `struct('realBots', [1 2], ...)`; all three: `R = run_real();`
   It checks Python modules, broker, corners, telemetry + camera pose of every robot, the parking slots of the real
   robots, then runs with `syncToReal`; close the view or Ctrl+C to stop all robots. `run_real('dry', struct('realBots',
   1))` is the same path without hardware (tested by `test_run_real` e).
10. Raise `maxTime` step by step; `R.lk` holds twin-vs-real error, latency and hold times per robot. A full fill
   (about 2880 s sim time) takes as long in wall time.

Optional Mosquitto instead of `mini_broker.py`: config `listener <P.mqtt.port> 0.0.0.0` + `allow_anonymous true`
(closed network only), allow that TCP port in the firewall, start `mosquitto.exe -v -c <config>`; `run_real` then uses it.
