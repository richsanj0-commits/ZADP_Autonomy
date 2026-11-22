# maps: other site shapes for the twin

Four more 1 x 1 m arena maps at the same 1:200 scale as `plan/plan_arena.json`, made with the web app's planner
(`zadp_game.html`, rounds 1-4) through `tools/export_plan.py`. `test_maps` runs a full fill on each.

| Map | Site polygon (site metres, `<name>_site.json`) | Spots |
|---|---|---|
| `trap` | trapezoid, bottom 180 m wide, top 130 m | 109 |
| `chamfer` | rectangle with both top corners cut at 45 deg | 121 |
| `mirror` | the reference pentagon mirrored (cut corner top left) | 126 |
| `octa` | octagon (all four corners cut) | 115 |

Make or remake one (repo root, `python -m http.server 8777` running; needs Playwright and Chrome):
```
python tools/export_plan.py --arena twin3d/maps/trap_site.json --out ../maps/plan_trap.json
```
`--out` is relative to `twin3d/plan/`; without it the exporter writes `plan/plan_arena.json`, the reference plan that
`test_plan` and `test_brain` pin.

## What shapes work
The brain supports one map family, checked with a clear message by `brain/assert_plan_supported.m` (called by
`brain_init`): simple site polygon, convex or not (the L in `plan/arena_lshape.json` runs); entry on the horizontal bottom edge, centred so the IN lane lies over parking P1
(x = 0.44 with the 24 m lane); exactly two zones, one each side of the spine, at most one branch each; inside the
1 x 1 m arena; one truck type. All four maps here are in it. Anything else (paddocks with holes, an entry on another
edge, three zones) needs layout work first: `HANDOVER.md` section 9.
`test_maps` now runs 400 s of sim time on these four and on the four arena shapes in `plan/`; the table below is
from the earlier full fills (before the non-convex change; not re-run).

## Results
`test_maps`, seed 1, every monitor 0 (robot-robot, pile, off-site, section ownership, deadlock):

| Map | Piles | Sim time | Min robot gap | Min pile gap | Worst pile error |
|---|---|---|---|---|---|
| trap | 109 / 109 | 2556 s | 12.0 mm | 5.7 mm | 5.0 mm |
| chamfer | 121 / 121 | 2777 s | 12.0 mm | 6.2 mm | 6.0 mm |
| mirror | 126 / 126 | 2892 s | 11.4 mm | 5.8 mm | 6.8 mm |
| octa | 115 / 115 | 2675 s | 11.0 mm | 6.2 mm | 5.6 mm |
