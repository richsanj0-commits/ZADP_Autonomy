# ZADP Dump Planner — test results

Fleet: Big hauler (363 t, pile r 6.35 m, 15.1 × 9.75 m, 2 trucks) + Small hauler (227 t, pile r 4.00 m, 12.9 × 7.6 m, 3 trucks).
Defaults: 20 km/h, 60 s manoeuvre, 60 s dump, a truck every 60 s, max trucks inside = number of zones.
Each shape goes through all 5 rounds from the uploaded sketch image. Coverage = average over 3 truck orders (seeds 7, 11, 23).

| Shape | Area error | Zones @ sweep | Baseline (fixed, 7.38 m) | Grid + remove | Touching | Collisions / deadlocks |
|---|---|---|---|---|---|---|
| L-shape | 0.0 % | 2 @ 95° | 18.1 % | 36.2 % | **58.2 %** | 0 / 0 |
| CAD curved (your sketch) | — | 2 @ 65° | 13.8 % | 44.7 % | **64.9 %** | 0 / 0 |
| Circle Ø80 | 0.2 % | 2 @ 100° | 12.1 % | 34.2 % | **55.6 %** | 0 / 0 |
| Rounded rect 120×80 | −0.1 % | 2 @ 100° | 13.9 % | 37.9 % | **61.7 %** | 0 / 0 |
| Stadium 150×60 | 0.1 % | 2 @ 155° | 14.3 % | 35.8 % | **58.9 %** | 0 / 0 |
| D-shape 130×100 | 1.0 % | 2 @ 125° | 16.7 % | 45.0 % | **64.1 %** | 0 / 0 |
| Ellipse 140×80 | 0.4 % | 2 @ 170° | 13.3 % | 39.3 % | **59.5 %** | 0 / 0 |
| Kidney | −0.4 % | 2 @ 5° | 13.4 % | 42.6 % | **66.0 %** | 0 / 0 |
| Star | 2.4 % | 4 @ 40° | 12.5 % | 40.2 % | **58.3 %** | 0 / 0 |
| Cross | −0.5 % | 4 @ 60° | 11.2 % | 37.6 % | **62.8 %** | 0 / 0 |
| Long strip 320×45 | −1.9 % | 2 @ 150° | 16.4 % | 21.1 % | **47.0 %** | 0 / 0 |
| Crescent | 0.5 % | 2 @ 60° | 13.8 % | 45.1 % | **67.4 %** | 0 / 0 |
| Dumbbell (narrow neck) | −0.1 % | 2 @ 160° | 15.5 % | 35.2 % | **47.2 %** | 0 / 0 |
| Comb (4 teeth) | −1.0 % | 2 @ 170° | 15.4 % | 41.7 % | **62.6 %** | 0 / 0 |
| Triangle | 0.0 % | 2 @ 115° | 16.1 % | 44.0 % | **65.3 %** | 0 / 0 |
| T-shape | −0.5 % | 2 @ 0° | 15.5 % | 40.6 % | **62.0 %** | 0 / 0 |
| Notched blob | 1.0 % | 2 @ 75° | 14.9 % | 49.9 % | **73.5 %** | 0 / 0 |
| Pentagon 12 m (too small) | — | 0 | 0 % | 0 % | 0 % | 0 / 0 — no truck fits; Round 3 warns |

Average over the 17 usable shapes: **Baseline 14.5 % · Grid + remove 39.5 % · Touching 61.5 %** — touching placement packs about 4.2× more than fixed spot points.

Note: coverage is of the whole dump area (including the safety margins and the lane). The baseline here is lower than the 52 % in the problem statement because this baseline uses the largest pile size for every truck and 7.38 m gaps; adjust in Round 5 if you have the real site figures.

## Safety rules enforced in Round 5
1. One-way lanes (IN / OUT), driven only on the lane centre line; other lane cells are crossed sideways only.
2. Zone locking + route locking: a truck's route outside the lanes (in and out) is reserved; no other route or pile may use it.
3. Wait gate: at most "max trucks inside" in the area.
4. Lane fills last, deepest part first, one truck at a time.
5. Lane mutex: the lanes (plus a safety strip) hold one moving truck at a time — the holder never waits, so no circular wait.
6. Isolation check: a pile is only given if the truck can still drive out after dumping it, and it never lands on another truck's reserved route.

Collisions (two truck bodies overlapping) and deadlocks (no truck can move for 10 minutes) are measured from the simulated truck positions, not assumed.

## How to re-run
1. Serve the folder: `python -m http.server 8765` and open `http://localhost:8765/zadp_game.html`
2. In the browser console: `await import('/tools/pipeline_test.js')` then `await pipe('weird_cross', [7, 11, 23])`
3. Test sketches are made by `python tools/make_test_sketches.py`
