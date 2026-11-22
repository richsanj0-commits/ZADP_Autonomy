# twin3d: digital twin of the 3-robot dump-truck arena

Three ESP32 differential-drive robots act as scaled haul trucks in a 1.0 x 1.0 m (1:200, see SCALING.md)
arena. A central MATLAB "brain" sends each robot a target. The twin simulates
the robots and shows them in 3D as low-poly haul trucks that tip a bed and form
a conical pile. Robots have wheel encoders only; an overhead ArUco webcam gives
absolute pose. Needs only MATLAB R2026a, Simulink, Simulink 3D Animation, Stateflow.

## Folder map
- `params/`  `twin_params.m` (all numbers), `pile_geometry.m` (cone from volume)
- `plan/`    the reference dump plan `plan_arena.json` and its loader (phase 2)
- `maps/`    four more map shapes (trapezoid, chamfered rectangle, mirrored pentagon, octagon) for `test_maps` (see `maps/README.md`)
- `brain/`   central dispatcher (phase 4): 3 robots fill the plan, zero collisions, no deadlock (see `brain/README.md`)
- `bot/`     single-robot physics, encoders, go-to-point controller (phase 3)
- `world3d/` 3D scene, haul trucks, piles, replay and live view (phase 6, see `world3d/README.md`)
- `simulink/` `zadp_twin.slx`: Brain + 3D View as MATLAB System blocks (phase 6, see `simulink/README.md`)
- `link/`    MQTT link; `TOPICS.md` is the message contract (phase 7)
- `tests/`   `test_*.m` scripts and `run_all_tests.m`

## Start
```matlab
run('E:\catrover_dumpautonomy\twin3d\startup.m')
run_all_tests          % about 20 min (test_brain + test_maps are full fills); one test: test_bot, test_rules, ...
```

## Physics vs visuals
Physics uses the REAL robot numbers (`P.bot`: 34 mm wheels, 75.3 mm track,
300 rpm motors). The visuals show a haul truck (`P.truck`) drawn over the robot.
In the sim the truck tips and piles form; on real hardware the robot just drives
there and comes back.

## Phases
Handover for a new session or the user: **`HANDOVER.md`** (architecture, functions, contracts, how to make a plan, demos, tests, limits, next steps for the real robots).

| # | Phase | Status |
|---|---|---|
| 1 | Setup (params, tests) | done |
| 2 | Plan (export from the web app, loader) | done |
| 3 | One bot (physics, encoders, camera fusion, go-to-point) | done |
| 4 | Brain (3 robots, zero collisions, no deadlock) | done |
| 5 | Piles (parking, truck numbers, pile maths, coverage) | done |
| 6 | 3D (scene, trucks, replay, live view, Simulink model, video) | done |
| 7 | MQTT link | done up to real hardware, tested against `fake_bot.py` (no broker, no camera, no firmware yet) |
| 8 | Real robots (firmware MQTT, broker, camera calibration, first hardware test) | open: `HANDOVER.md` section 14 |
