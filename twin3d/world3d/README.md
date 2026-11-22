# world3d: the 3D simulation (phase 6)

A 3D view of the twin in base MATLAB graphics (`patch`, `surface`, `hgtransform`, lights, perspective camera). The
Unreal-based viewer of Simulink 3D Animation is NOT used: it needs several GB of RAM and a second process. The
Simulink model in `../simulink/` drives this renderer, so the simulation still runs from Simulink.

```matlab
run('E:\catrover_dumpautonomy\twin3d\startup.m')
demo_3d                                   % writes demo_3d.png and demo_3d.mp4 (about 3 minutes)
[W, log] = brain_run(plan, P);            % a full run ...
world_play(log, plan, P, struct('speed', 40))   % ... replayed in 3D (visible figure, real-time pacing)
world_live(plan, P, struct('maxTime', 300, 'speed', 10))   % watch the brain while it computes
```

Arena metres: x east, y north, z up. The site is 1 x 1 m = 200 x 200 m (1:200); the apron lies at y < 0.

## Files
| File | What |
|---|---|
| `world_build(plan, P, opts)` | scene: dirt ground, apron slab, parking slots P1..P3 (from `P.brain.parking`), site on a raised pad with a berm cut open at the gate, zones Z1 / Z2 (tinted), branch roads, spine with IN (blue) / OUT (red) centre lines and arrows, gate ramp and posts, 3 trucks, HUD axes, sun + head light. Returns handle struct `h` |
| `world_view(h, 'top'|'orbit'|'gate'|'follow', k)` | camera (perspective); `k` = extra azimuth (orbit), robot number (follow) |
| `truck_build(ax, P, color)`, `truck_pose(ht, pose, bedDeg, truckNo)` | the haul truck, move it and tip the bed |
| `pile_draw(ax, c, r, h, P, key)`, `pile_update(hp, r, h)` | dirt pile (cone surface) and its growth |
| `dump_profile(f, P)` | bed angle and pile fraction during a dump (f = share of `P.truck.dumpTime`) |
| `world_sync(h, s)`, `world_hud(...)` | put trucks and piles into a state / write the HUD |
| `world_play(log, plan, P, opts)` | replay a brain log, optional MP4 (GIF if the MPEG-4 profile is missing) |
| `world_live(plan, P, opts)` | brain step loop with live drawing |
| `brain_dump_state`, `brain_view_state` | view state of a running brain world (used by `world_live` and `BrainSystem`) |
| `coverage_curve(centres, radii, plan)` | coverage after each pile (same definition as `brain_coverage`), for the replay HUD |
| `solid_box`, `solid_prism`, `solid_wheel` | flat-shaded patch primitives |
| `demo_3d` | produces `demo_3d.png` and `demo_3d.mp4` |

## The truck (`truck_build`)
Rigid-frame mining dump truck from boxes, prisms and wheels, yellow body, dark tyres, no logos. Footprint exactly the
robot's 0.100 x 0.084 m (`P.truck`), origin at the REAR-AXLE midpoint like the robot pose (x forward, y left): large
rear wheels, small front wheels, cab on the front left under the canopy of the dump body, engine deck with grille on the
front right, tipping dump body hinged at the rear (child `hgtransform`, tilts up to `P.truck.bedTiltMaxDeg`), label
`T<truck number>` above it and a robot colour stripe (robot 1 orange, 2 blue, 3 green) on the canopy and cab. The label is
hidden while the robot is parked (it carries no truck then).

## The pile
`pile_draw`: cone surface, colour `P.pile.color`, flat shaded, every vertex a little brighter or darker. The base ring is
exactly the pile radius (the radius the planner and the safety monitor use) and the apex exactly the pile height; the
slopes between are slumped a little, fixed per spot number so a pile looks the same while it grows and when it is done.
During the dump the radius and height come from `pile_growth(frac, r, h)` (volume proportional to frac).

## What a replay needs from the log (`brain_run`)
`log.pose` (N-by-3-by-nb true poses, every `logEvery` = 0.1 s, interpolated between samples), `log.truck` (N-by-nb truck
number, 0 = none; added in phase 6), `log.events` (assigned), `log.pileRec` / `log.piles` (centre, size and time of each
pile). The dump of a pile ran for `P.truck.dumpTime` before its record time, so bed angle and pile growth are
derived from `dump_profile`; nothing else had to be logged.

## Dump sequence (`dump_profile`)
Bed rises during the first 25 % of the dump, stays up, lowers in the last 25 %; material runs out between 20 % and 70 %.

## HUD
Sim time, site time (sim time x 4.8, SCALING.md), trucks dispatched, piles placed / spots, coverage %, collisions
(robot-robot, robot-pile and off-site events from the safety monitors, `brain_monitor_collisions`; 0). The same
count everywhere: `world_play` shows the events up to the frame time (each event is logged with its time),
`world_live` and the Simulink view count them at every brain step. `world_play` and `world_live` stop cleanly when
the figure is closed; `world_play` closes its video file and hidden figure even when a frame fails.

## Replay speed
Rendering is software-limited (hidden figure, 1280 x 720 `getframe`); see the numbers in the phase report. Use a larger
`opts.speed` for a fast review, smaller for detail.
