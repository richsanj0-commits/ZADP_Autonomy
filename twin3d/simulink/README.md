# simulink: the twin as a Simulink model (phase 6)

`zadp_twin.slx` has two MATLAB System blocks, both "Simulate using: Interpreted execution", fixed-step discrete solver,
step `P.brain.dt` = 0.02 s, stop time 60 s.

```matlab
run('E:\catrover_dumpautonomy\twin3d\startup.m')
build_twin_model                 % (re)creates zadp_twin.slx, opens no window
r = run_twin_model(60)           % sim('zadp_twin', 'StopTime', '60'), reports piles placed, wall time, picture updates
```

| File | What |
|---|---|
| `BrainSystem.m` | `matlab.System` source block. Holds the brain world `W` (`brain_init`), calls `brain_step` once per sample time and outputs the 3 robot poses (3x3), robot modes, piles placed, coverage %, sim time, bed angles, truck numbers, the pile being dumped by each robot, a padded table of placed piles and the collision count. The outputs are the state at the start of the sample, so the time output equals Simulink time |
| `View3DSystem.m` | `matlab.System` sink block. Builds the 3D world (`world_build`) in `setup` (hidden figure, tag `zadp_twin_view`) and every `RenderPeriod` (0.25 s of sim time) moves the trucks, tips the bed, grows the pile being dumped and adds finished piles (`world_sync`) |
| `build_twin_model.m` | builds and saves the model: Brain -> View3D, Display blocks (piles placed, coverage %, robot modes, collisions), Scope (coverage, not opened at start), To Workspace (`piles_placed`, `coverage_pct`, `sim_time`) |
| `run_twin_model.m` | runs the model and prints / returns the result; writes the last picture to `zadp_twin_last.png` |

## How the 3D view gets its data
Through the signal lines only: Brain's outputs `pose`, `bed`, `truck`, `dump`, `pileTable`, `piles`, `coverage`, `time`,
`collisions` are wired to View3D's inputs of the same names. This was chosen over a global or an appdata handle because it
needs no hidden coupling (the view works with any source of the same signals, e.g. a replay) and is robust to Simulink
creating the two System objects in either order. Pile records travel as a fixed-size table (`MaxPiles` x 5: x, y, r,
h, spot number; unused rows are zero) and the pile being dumped as a 3 x 6 row block (x, y, r, h, spot, fraction), which is
how a growing pile reaches the view before its record exists.

## Notes
- The model name `zadp_twin` must not exist as another file on the path; `build_twin_model` deletes the old `.slx` first.
- `matlab.System` blocks in interpreted mode run the plain MATLAB functions: after 60 s the model and `brain_run`
  (seed 1) both have 2 piles; coverage is 0.919 % in the model and 0.918 % in `brain_run`, because the global random
  stream of the camera noise is shared with everything else running in the same MATLAB session, so the noise
  sequence is not bit-identical.
- Display blocks do not open a window during `sim`; the Scope is set to stay closed. Set `Visible` of View3D to `'on'`
  to watch the run.
