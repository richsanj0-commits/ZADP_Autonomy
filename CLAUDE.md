# CLAUDE.md (repo guide, loaded every session)

## What this is
- `zadp_game.html` = ZADP dump planner web app, the MAIN solution (5 rounds: area, fleet, safe boundary, zones and piles, dispatch). Details in `README.md`, `docs/ZADP_Project_Documentation.md`, problem statement in `docs/`.
- `twin3d/` = SUPPORTING hardware digital twin (MATLAB R2026a + Simulink + 3D Animation + Stateflow only): 3 small ESP32 differential-drive robots act as haul trucks on a 1 m x 1 m arena = 200 m x 200 m site (1:200).
- Start in `twin3d/HANDOVER.md` (architecture, functions, contracts, how to extend, next steps for the real robots). Then `twin3d/README.md`, `SCALING.md`, `brain/README.md`, `link/TOPICS.md`.

## Where things live
- Web app: `zadp_game.html`, `vendor/`, `tools/` (`export_plan.py` makes plan JSON for the twin, `make_screenshots.py`, `pipeline_test.js`), `docs/`, `start.bat`.
- Twin: `twin3d/` folders `params plan brain bot world3d simulink link tests`. All numbers: `twin3d/params/twin_params.m`.

## Run and test
- Web app: `start.bat` (Windows) or `python -m http.server 8765` in the repo root, open `http://localhost:8765/zadp_game.html` (never `file://`).
- New arena plan: `python -m http.server 8777`, then `python tools/export_plan.py --arena twin3d/plan/arena_site.json` (writes `twin3d/plan/plan_arena.json`; needs Playwright + Chrome).
- Twin (MATLAB MCP tool, `project_path` `E:\catrover_dumpautonomy\twin3d`): first `run('E:\catrover_dumpautonomy\twin3d\startup.m')`.
  Demos: `demo_brain`, `demo_3d`, `demo_link`, `demo_one_bot`, `run_twin_model(60)`. Tests: `run_all_tests` (all) or one `test_*`.
- Real robots: `run_real()` (one command; launch order in `twin3d/HANDOVER.md` section 14); `run_real('dry')` = same path with fake robots + simulated camera (`test_run_real`, about 95 s). Contract: `twin3d/link/TOPICS.md`. Firmware numbers vs twin: `test_firmware` (`link/py/check_numbers.py`, JS mirror of `motion_core.h`, camera geometry).
- Other map shapes: `twin3d/maps/` (`<name>_site.json` -> `python tools/export_plan.py --arena twin3d/maps/<name>_site.json --out ../maps/plan_<name>.json`), tested by `test_maps`. Supported family: `brain/assert_plan_supported.m`.
- A full `run_all_tests` (10 tests) takes about 8 to 9 minutes (`test_brain` about 4.5 min: 3 full fills; `test_maps` 8 maps x 400 s sim; `test_link` and `test_run_real` about 80 s each). One MATLAB tool call times out near 60 s: start long jobs with a MATLAB timer or slice them (`opts.maxTime`), save results to a .mat file and poll. A timeout does not mean the run failed. `test_link` compares wall-clock timing: run it with the PC otherwise idle.
- Last full run (2026-11-22): 9 of 10 passed in 493 s; `test_link` failed once under load and passed 3 of 3 alone. Arena seeds 1/2/3: 125/125/125 piles, every monitor 0. Details: `twin3d/FINAL_REPORT.md`.

## Hard rules
- Physics uses the REAL robot numbers in `twin3d/params/twin_params.m` (`P.bot`), the single source of numbers. Do not hard-code a number elsewhere. `P.truck` is visual only.
- Visuals are a generic haul truck: no brand names, logos or marks.
- Scale 1:200 is set by the robot (0.100 x 0.084 m = 20.0 x 16.8 m truck). Arena 1 x 1 m plus a 0.25 m apron (parking and return loop).
- Robots have encoders only; the overhead camera gives absolute pose (3 mm, 1 deg).
- Every entry from parking is a NEW truck (`W.truckNo` counts them). The rolling parking queue P3 -> P2 -> P1 is the parking.
- On real hardware the robot only drives to the spot and back; piles exist only in the twin. The twin leads, the robots follow (`link/`).
- Safety monitors must stay at zero: robot-robot overlaps, pile hits, off-site, deadlocks. Never relax them to make a test pass.
- Code style: plain functions on structs, each rule stated as a comment where it is enforced.
- Machine has 16 GB RAM and is nearly full: do not start a second MATLAB (no `matlab -batch`), do not start the Unreal viewer of Simulink 3D Animation, keep figures hidden (`Visible off`) and close them.

## User preferences
- Terse replies. Do not ask between steps; just do the work.
- The lead model delegates bulk work to cheaper models and tests the result itself. Quote only numbers you verified in code or by running.
- No git commands unless asked. No emojis.

## Open items
- The L-paddock connector-road fix for the web app is parked in a git stash (`stash@{0}`, "L-paddock connector fix (untested)"). Untested.
- Twin, firmware (`firmware/zadp_bot`, FreeRTOS tasks) and the hardware guide (`docs/ZADP_Hardware_Twin_Guide.pdf`) are committed. Web app fixes are a separate commit.
- Web app open findings (disc collision model, route grid ignores truck length): `twin3d/FINAL_REPORT.md` section 6.
- Firmware was never built by PlatformIO or run on an ESP32; no broker or camera was used here. First hardware steps: `twin3d/HANDOVER.md` section 14.
