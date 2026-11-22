# simulink: the twin as a Simulink model (phase 6)

`zadp_twin.slx` has two MATLAB System blocks, both "Simulate using: Interpreted execution", fixed-step discrete solver,
step `P.brain.dt` = 0.02 s, stop time 600 s (about 25 piles; a full fill needs about 2900 s: set the stop time to `inf`,
the Stop block ends the run when Brain's `done` output goes to 1). An annotation on the canvas explains the blocks.

Open it and press Run: the 3D view opens (View3D `Visible` = 'on' in the saved model) and the displays show piles placed,
trucks dispatched, coverage, robot modes and collisions. The model's PreLoadFcn / InitFcn run `startup.m` when the twin
is not on the path, so it also works in a fresh MATLAB. Running it twice in a row is fine, and so is closing the 3D
window during a run (the view stops drawing, the simulation goes on).

```matlab
run('E:\catrover_dumpautonomy\twin3d\startup.m')
build_twin_model                 % (re)creates simulink/zadp_twin.slx, opens no window
r = run_twin_model(60)           % 60 s with a hidden view; reports piles placed, wall time, picture updates
r = run_twin_model(60, struct('seed', 2, 'planFile', 'plan_trap.json'))   % another seed / map (maps/)
```
`run_twin_model` never changes the saved model (parameters are set in memory, the model is closed without saving)
and writes its last picture to a temp file (`r.png`). Tests build the model into a temp folder, so a test run leaves
the repository unchanged.

| File | What |
|---|---|
| `BrainSystem.m` | `matlab.System` source block (parameters `Seed`, `PlanFile`, `MaxPiles`). Holds the brain world `W` (`brain_init`), calls `brain_step` once per sample time and outputs the robot poses (nb x 3, nb = `P.bot.count`), robot modes, piles placed, coverage %, sim time, bed angles, truck numbers, the pile being dumped by each robot, a padded table of placed piles, the collision count (`brain_monitor`, the same monitors as `brain_run`), trucks dispatched and `done`. The outputs are the state at the start of the sample, so the time output equals Simulink time. Setup refuses a plan with more spots than `MaxPiles` |
| `View3DSystem.m` | `matlab.System` sink block. Builds the 3D world (`world_build`) in `setup` (tag `zadp_twin_view`) and every `RenderPeriod` (0.25 s of sim time) moves the trucks, tips the bed, grows the pile being dumped, adds finished piles and writes the HUD (`world_sync`, `world_hud`). A closed figure is skipped; at the end a hidden figure is closed, a visible one stays |
| `build_twin_model.m` | builds and saves the model: Brain -> View3D, Display blocks (piles placed, trucks dispatched, coverage %, robot modes, collisions), Scope (coverage, not opened at start), Stop Simulation on `done`, To Workspace (`piles_placed`, `coverage_pct`, `sim_time`), the annotation and the path callbacks |
| `run_twin_model.m` | runs the model (hidden view by default) and prints / returns the result |

## How the 3D view gets its data
Through the signal lines only: Brain's outputs `pose`, `bed`, `truck`, `dump`, `pileTable`, `piles`, `coverage`, `time`,
`collisions` are wired to View3D's inputs of the same names. This was chosen over a global or an appdata handle because it
needs no hidden coupling (the view works with any source of the same signals, e.g. a replay) and is robust to Simulink
creating the two System objects in either order. Pile records travel as a fixed-size table (`MaxPiles` x 5: x, y, r,
h, spot number; unused rows are zero) and the pile being dumped as a 3 x 6 row block (x, y, r, h, spot, fraction), which is
how a growing pile reaches the view before its record exists.

## Notes
- The model name `zadp_twin` must not exist as another file on the path; `build_twin_model` deletes the old `.slx` first.
- `matlab.System` blocks in interpreted mode run the plain MATLAB functions. The brain draws all its noise from its
  own random stream (`W.rs`, seeded with `Seed`), so the model, a re-run of it and `brain_run` with the same seed are
  the same run (`test_world3d` checks the piles and coverage after 30 s against `brain_run`).
- Display blocks do not open a window during `sim`; the Scope is set to stay closed. View3D `Visible` = 'off' hides the
  3D view (what `run_twin_model` and the tests use).
