# tools

Run from the repo root unless noted. Playwright scripts use the system Chrome and need the page served: `python -m http.server 8777` in the repo root.

| Script | Purpose | Needs | Output |
|---|---|---|---|
| `export_plan.py` | export a Round 4 plan of the web app for the MATLAB twin (valley site, or `--arena <site.json>`, `--rows N`, `--out NAME`) | Playwright, Chrome, server on :8777 | `twin3d/plan/plan_valley.json` or `twin3d/plan/plan_arena.json` (`--out` is relative to `twin3d/plan/`) |
| `make_screenshots.py` | presentation screenshots of the five rounds (1920x1080) | Playwright, Chrome, server on :8777 | `docs/screenshots/` |
| `make_test_sketches.py` | dimensioned test sketches (shapes) for Round 1 | matplotlib | `test_sketches/*.png` |
| `make_site_sketches.py` | five practical mine-scale site sketches | matplotlib | `test_sketches/site_*.png` |
| `make_theorems_pdf.py` | theorems and references PDF (run from the repo root, the path is relative) | reportlab | `docs/ZADP_Theorems_and_References.pdf` |
| `pipeline_test.js` | browser helper: runs a sketch through Rounds 1-5; load in the page console with `await import('/tools/pipeline_test.js')`, then `await pipe('weird_cross')` | the web app served over http | results in the console |
