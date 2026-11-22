# brain: central dispatcher (phase 4, parking / truck numbers / pile maths added in phase 5)

Runs the 3 robots as haul trucks on the exported dump plan, with the same rules as the
ZADP dispatcher (`zadp_game.html`: checkSpot / findSpot / sortKeys / stepSim), written as small
MATLAB functions on structs. Each robot is the phase 3 model (`bot_step`, `bot_controller`,
`cam_measure` + `pose_fusion`, camera on).

```matlab
run('E:\catrover_dumpautonomy\twin3d\startup.m')
demo_brain            % full fill, seed 1, saves brain/demo_brain.png
```

## Files
| File | What |
|---|---|
| `brain_init(plan, P)` | world `W`: robots parked in the apron queue, spots with status, piles, section owners, events |
| `brain_step(W, P, dt)` | one step: exits, queue moves, dispatch, route following, dumping, lock release, deadlock watch, end |
| `brain_run(plan, P, opts)` | runs to the end (or a deadlock); safety monitors on the true poses every step; sparse log |
| `brain_monitor`, `brain_monitor_collisions`, `brain_owner_check` | monitors (robot-robot, robot-pile, off-site, section ownership) shared by `brain_run`, `world_live` and Simulink; the one "collisions" number; the Rule 3 invariant |
| `assert_plan_supported` | the map family the brain supports, checked with clear messages (`maps/README.md`) |
| `route_to_spot` | trip plan for one spot: queue head -> IN lane -> branch -> stop -> dump -> OUT lane -> apron -> queue tail |
| `route_apron` | queue advance P3 -> P2 -> P1 |
| `route_steps`, `route_new`, `route_check`, `route_needs` | route builder, swept-body check (piles, site edge), sections a move needs |
| `brain_rank` | fill order (Rules 1, 1b, 4) |
| `brain_spot_plan`, `brain_piles_changed`, `brain_obstacles` | plan cache, invalidation when a pile lands near a planned path |
| `brain_layout`, `brain_sections` | junction points, apron loop, sections |
| `body_outline`, `rect_circle_clear`, `rect_pair`, `in_allowed` | geometry: body outline, body-to-pile gap, separating-axis test, site test (any simple polygon; convex sites use the fast half-plane form; `L.reflex` inner corners are kept clear in `route_check` and the site monitor) |
| `lane_turn_y` | height of the in-place turn between a spine lane and a branch road: the branch height, or lower when the turn disc would leave the site (branch close to a site edge, e.g. the L) |
| `demo_maps` | full fill on the four arena shapes in `plan\` (timer, one map per call), writes `plan\maps_results.md` and the fill images |
| `pile_from_payload(t, P)` | payload -> pile r, h, V at site scale (+ arena values), same maths as the planner's `calc(t)` |
| `pile_growth(frac, r, h)` | visible pile radius/height while the dump runs (constant repose angle, volume proportional to frac) |
| `brain_coverage(W, plan)` | union area of the placed piles inside the site / site polygon area, and tonnes dumped |
| `brain_plot`, `demo_brain` | figure and demo |

## Rules (each is stated again as a comment where it is enforced)
1. **Fill order** (`brain_rank`): inside each zone the lowest plan order that is reachable now; a truck
   takes a spot in another zone when the first zone is locked.
   **1b Isolation** (as `cutsOff` in the html): a spot whose pile would cut off the approach of another
   reachable spot (tested with the pile shifted 8 mm in four directions) waits until that spot is filled.
   Branch spots come after their zone, except when every zone spot left would cut a spot off: then a
   branch spot that cuts nothing off goes first (this fills the inner-row branch pocket, spots 41 and 100,
   before the zone's last row closes it).
2. **Zone lock** (`brain_step>dispatch`): the zone section (zone + its branch) has one holder.
3. **Lane mutex** (`brain_layout`, `brain_sections`): the whole drivable area is cut into sections;
   a robot may start a move only while it holds every section its swept body (+8 mm) touches.
   A long drive is cut into pass-through pieces of at most 5 cm; a piece is done once the robot's projection
   on the leg passes the piece's end point, and only then are sections no later piece needs released.
   Invariant (monitor e, `brain_owner_check`, every step): every section a true body touches is held by that
   robot.
4. **Spine last** (`brain_rank`, `dispatch`): lane spots only when no zone/branch spot is reachable and no
   zone trip is under way; deepest first; one truck at a time, everybody else parked.
5. **Dumping** (`route_to_spot`, `brain_step>placePile`): rear to the spot, stop = spot + u*D,
   D = pileR + axleToBack + 8 mm (longer only when the short stop is blocked), heading u, wait
   `P.truck.dumpTime`; the pile appears where the truck really stopped and is an obstacle forever.
6. **Never start what you cannot finish** (`dispatch`, `atS`): the whole inbound route and the zone
   lock are granted at once before a truck leaves the queue head; the whole exit route is granted at
   once before it leaves its dump stop. Waiting happens only in the apron slots or at the dump stop
   (inside the truck's own locked zone). Exits go before new entries.
7. **Deadlock** (`brain_step`): wait-for graph every step (edges: a robot waits for the holder of a section
   it needs, or for the robot holding the queue slot ahead); a cycle for 10 s, or nobody moving or dumping
   for 60 s before the run has ended (Rule 8), is a deadlock. `brain_run` ends the run there
   (`opts.stopOnDeadlock`, default true); `test_rules` forces both cases.
8. **End**: when nothing can be served, all robots are parked in P1..P3 and the run ends; every unfilled
   spot keeps the reason its last plan search failed.

## Lane sections and apron loop
Sections: `Z1`, `Z2` (zone + branch = zone lock), `IN_LOW` / `OUT_LOW` (each lane below the junction
plus the apron strip under it), `J` (whole spine width around the branch roads: the lane crossing), `UP` (spine above the junction), apron `APR_R` (return row),
`QW`, `QM`, `Q2S`, `Q3S` (queue row: head slot, middle, slot 2, tail slot). The junction takes
3 cm of each branch mouth; a robot whose axle is inside a zone polygon therefore always holds that
zone (its body reaches at least 0.024 m past the axle, +8 mm inflation). A left-zone truck leaves
by turning down 1.2 cm short of the OUT lane centre and slanting onto it, so no zone trip ever
needs the other zone.

Apron loop (one way): queue row y = -0.159 heading west, head P1 under the IN lane -> up the IN lane
-> site -> down the OUT lane -> return row y = -0.0555 heading east -> down the east column x = 0.90
-> tail slot P3 -> P2 (x 0.75) -> P1. Turn directions in the apron are fixed so every in-place turn
sweep stays inside the apron.

Why there is no deadlock: inside the lanes every grant is all-or-nothing, so no robot waits while
holding a lane section. The only holders that wait are parked queue robots (`QW`, `Q2S`, `Q3S`) and
trucks at their dump stop (their zone). An exit route never needs `QW` or `Q2S`, an inbound route never
needs `Q2S` or `Q3S`, and with 3 robots and 3 slots a chain stop -> P3 -> P2 -> P1 -> stop needs 4
robots, so every wait chain ends at a robot that is moving.

## Geometry limit
The forward-only robot needs an in-place 180 degree turn at the dump stop (nose sweep radius 0.087 m).
A branch spot of the inner row (y = 0.422) sits in a pocket between the branch's outer row and the
zone's last row; once the zone's last row is filled the channel left (about 0.057 m) is narrower than the
truck (0.084 m). Rule 1b now lets that branch spot go before the zone spot that would close it, so
spots 41 and 100 are filled (they were lost in earlier versions, depending on the noise).

## Results (tests/test_brain.m, camera on, dt 0.02 s)
| Seed | Piles | Sim time | Min robot gap | Min pile gap | Worst pile error | Overlaps / pile hits / off-site / ownership / deadlocks | Unfilled |
|---|---|---|---|---|---|---|---|
| 1 | 125 / 125 | 2873 s | 12.1 mm | 5.3 mm | 6.3 mm | 0 / 0 / 0 / 0 / 0 | none |
| 2 | 125 / 125 | 2881 s | 11.5 mm | 3.4 mm | 5.4 mm | 0 / 0 / 0 / 0 / 0 | none |
| 3 | 125 / 125 | 2888 s | 9.9 mm | 4.4 mm | 6.4 mm | 0 / 0 / 0 / 0 / 0 | none |

Other maps (`test_maps`, seed 1): trapezoid 109 / 109, chamfered 121 / 121, mirrored pentagon 126 / 126, octagon 115 / 115, all monitors 0.

One full fill takes about 80 s of wall time; `test_brain` (3 seeds) about 4 minutes.

Note on the fill order: the exported plan orders each zone's lower block nearest-row-first
(row y = 0.356 before 0.111). Followed literally, the first pile in that row seals the rows behind
it from the branch road. Rule 1b (isolation) re-orders those rows far-first automatically; the plan
order still breaks ties.


## Parking, trucks and piles (phase 5)
**Parking.** The three apron queue slots are the robots' parking: `P.brain.parking` (3-by-3, rows
`[x y heading]` for P1, P2, P3; the only source, `brain_layout` reads it and checks that P1 sits under the plan's
IN lane). Each robot starts in a slot, drives in, dumps, leaves the site, comes back to parking and enters again.
A robot that returns always parks at P3 and advances P3 -> P2 -> P1 while it waits (a rolling parking queue); the
robot in P1 is the next to be dispatched. The slot is therefore not tied to a robot: a fixed bay per robot would
need a wider apron (each robot would have to reach its own bay across the others' lanes), and the proven
all-or-nothing section grants rely on the one-way loop.

**Trucks.** Three robots stand in for an endless stream of trucks. Every time a robot leaves P1 for the gate it
is a NEW truck: `W.truckNo` counts them (1, 2, 3, ...), the robot stores `b.truck` and `b.payload` for the trip,
and every event (`assigned`, `entered`, `dumped`, `exited`, `returned`) and every pile record carries the truck number.

**Payload and pile maths.** Payload = plan payload (500 t) x load factor, uniform in
`[P.pile.loadMin, P.pile.loadMax] = [0.85, 1.00]` from its own seeded stream (`1000 + seed`, so the robot noise
is the same as in phase 4). `pile_from_payload` is `calc(t)` of `zadp_game.html`: V = payload / rho (rho 1.8 t/m^3,
repose 37 deg, waste rock), r = cbrt(3 V / (pi tan repose)), h = r tan(repose), no shape factor. 500 t gives
r = 7.06 m, h = 5.32 m (arena 35.3 mm, 26.6 mm); 425 t gives r = 6.69 m, h = 5.04 m (arena 33.4 mm, 25.2 mm).
The actual radius is capped at the planned one, so every clearance planned with the largest pile still holds; the
planning keeps the planned radius, while the safety monitor, `brain_coverage` and the plot use each pile's own
radius (`W.pileR`). `W.pileRec` has one record per pile: truck, bot, spot, order, zone, payload, load, V, r, h
(site and arena), t.

**Coverage.** `brain_coverage` = union area of the placed circles inside the site polygon / polygon area, on a 1 mm
grid (no toolbox); reported by `demo_brain` and stored in the run log (`log.coverage`, `log.tonnes`, `log.trucks`).
`pile_growth(frac, r, h)` gives the pile during the dump: r, h scale with frac^(1/3) (volume proportional to frac).
