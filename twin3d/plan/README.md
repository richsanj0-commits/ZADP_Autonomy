# plan: the exported dump plan (phase 2)

`plan_arena.json` is written by `tools/export_plan.py` from `arena_site.json` and `zadp_game.html`; `load_plan.m` reads it, `plot_plan.m` draws it. How to make a new plan and the field list: `../HANDOVER.md` sections 4 and 9. Test: `tests/test_plan.m`.

Three more arena shapes (same fleet, scale, laneW 24, bottom entry): `arena_rect.json`, `arena_lshape.json` (non-convex L),
`arena_hex.json` -> `plan_<name>.json` (`python tools/export_plan.py --arena twin3d/plan/arena_<name>.json --out plan_<name>.json`)
and `plan_<name>.png` (plot_plan). `brain/demo_maps` fills all four and writes `maps_results.md`, `fill_<name>.png`,
`fill3d_<name>.png`; `tests/test_maps.m` runs 400 s on each.
