# ZADP Dump Planner

Browser simulation of **ZADP (Zoned Adaptive Dump Plan)** for autonomous haul-truck dump packing, in five rounds:

1. **Dump area** — upload a sketch with dimensions; the outline is traced (lines and arcs), the numbers are read and matched to what they measure, and the real shape is built in metres.
2. **Fleet** — per truck type: payload, number of trucks, pile radius (entered or calculated from payload, material density and angle of repose), length × width; lane width.
3. **Safe boundary** — Minkowski erosion of the area by the largest truck margins.
4. **Zones & piles** — Boustrophedon zones (sweep slider, best angle) and a hexagonal pile grid per truck type, with a two-lane corridor that fills last.
5. **Dispatch** — live multi-truck simulation comparing Baseline (fixed spots), Grid + remove and Touching placement, with collision and deadlock counts.

## Run

```
python -m http.server 8765
```

Then open `http://localhost:8765/zadp_game.html`. Reading dimensions from a sketch needs an internet connection the first time (the text reader loads from a CDN).

## Files

- `zadp_game.html` — the whole app
- `test_sketches/` — test sketches (made by `tools/make_test_sketches.py`)
- `tools/pipeline_test.js` — runs a sketch through all rounds in the browser console
- `test_results.md` — results on 18 shapes
- `zadp_sim.py`, `zadp_run.gif` — first Python prototype
