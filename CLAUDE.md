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
- A full `run_all_tests` takes about 7 minutes (`test_brain` alone about 6.3 min: 3 full fills). One MATLAB tool call times out near 60 s: start long jobs with a MATLAB timer or slice them (`opts.maxTime`), save results to a .mat file and poll. A timeout does not mean the run failed.
- Last full run: ALL TESTS PASSED, 404.9 s (seed 1/2/3: 123/124/124 of 125 piles).

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
- `docs/Problem_Statement_Optimal_Dump_Packing.pdf` is marked confidential while the GitHub repo is public. Decide before pushing.
- Nothing from the twin may be committed: at last check `twin3d/` and `tools/export_plan.py` were untracked and `zadp_game.html` modified. Run `git status` to check.
- Real-robot steps (firmware MQTT, broker, camera calibration, first hardware test) are not done: see "What the user does next" in `twin3d/HANDOVER.md`.
