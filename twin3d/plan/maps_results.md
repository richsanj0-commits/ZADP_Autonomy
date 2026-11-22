# Four arena shapes: full 3-robot fill, seed 1 (demo_maps)

Same brain code and parameters on every map. Violations = robot-robot / pile / off-site / section ownership / deadlock / zone lock events.
Images: `fill_<map>.png` (2D, brain_plot) and `fill3d_<map>.png` (3D still) in this folder.

| Map | Site area (m^2) | Zones | Spots | Piles placed | Coverage % | Tonnes | Sim time (s) | Min robot gap (mm) | Violations | Unfilled spots and reason |
|---|---|---|---|---|---|---|---|---|---|---|
| arena | 31600 | 2 | 125 | 125 | 58.9 | 58087 | 2873 | 12.1 | 0 / 0 / 0 / 0 / 0 / 0 | none |
| rect | 23400 | 2 | 86 | 86 | 54.5 | 39840 | 1898 | 12.0 | 0 / 0 / 0 / 0 / 0 / 0 | none |
| lshape | 22500 | 2 | 82 | 82 | 54.0 | 37863 | 1792 | 13.9 | 0 / 0 / 0 / 0 / 0 / 0 | none |
| hex | 25850 | 2 | 98 | 97 | 55.8 | 45037 | 2235 | 12.1 | 0 / 0 / 0 / 0 / 0 / 0 | 1: order 36 lane: no approach clears the piles (all 19 directions x 8 stop distances); straight one: stop: pile at (0.348, 0.753) -18.4 mm from the body at (0.406, 0.724) |
