# ZADP Dump Planner

Browser simulation of **ZADP (Zoned Adaptive Dump Plan)** for autonomous haul-truck dump packing, in five rounds:

1. **Dump area** — upload a sketch with dimensions; the outline is traced (lines and arcs), the numbers are read and matched to what they measure, and the real shape is built in metres.
2. **Fleet** — per truck type: payload, number of trucks, pile radius (entered or calculated from payload, material density and angle of repose), length × width; lane width.
3. **Safe boundary** — Minkowski erosion of the area by the largest truck margins.
4. **Zones & piles** — Boustrophedon zones (sweep slider, best angle) and a hexagonal pile grid per truck type, with a two-lane corridor that fills last.
5. **Dispatch** — live multi-truck simulation comparing Baseline (fixed spots), Grid + remove and Touching placement, with collision and deadlock counts.

## Run

Windows: double-click `start.bat` (needs Python). It starts a local server and opens the page.

Any system:

```
python -m http.server 8765
```

then open `http://localhost:8765/zadp_game.html`. Works offline: the text reader and geometry library are bundled in `vendor/`. Do not open the HTML file directly (the text reader cannot start from a `file://` page).

Round 1 also offers 5 sample sketches with known dimensions.

## Files

- `zadp_game.html` — the whole app
- `test_sketches/` — test sketches (made by `tools/make_test_sketches.py`)
- `tools/pipeline_test.js` — runs a sketch through all rounds in the browser console
- `test_results.md` — results on 18 shapes
- `zadp_sim.py`, `zadp_run.gif` — first Python prototype

## 3D digital twin (supporting hardware)

`twin3d/` is a MATLAB digital twin of the plan: three small ESP32 robots act as haul trucks on a 1 m x 1 m arena (1:200 of a 200 m x 200 m site), a central dispatcher fills the pile spots exported from this app, and a 3D view shows trucks tipping and piles forming. It supports the hardware demonstration; the planner above is the main solution. See [`twin3d/README.md`](twin3d/README.md) and the handover guide [`twin3d/HANDOVER.md`](twin3d/HANDOVER.md). Plans are exported with `tools/export_plan.py`.
