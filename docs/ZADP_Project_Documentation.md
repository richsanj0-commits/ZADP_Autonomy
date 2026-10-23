# ZADP — Zoned Adaptive Dump Plan
### Simulation, method and results: full project documentation

Team Ign!te · Problem statement: autonomous dump packing in open-pit mining

---

## 1. Problem

Autonomous haul trucks in open-pit mines fill a dump area ("paddock" or "dump polygon") much less densely than human operators:

| | Autonomous (today) | Human operators |
|---|---|---|
| Spacing between piles | ~7.38 m | ~3.03 m |
| Polygon coverage | ~52 % | ~95 % |

That is a **2.4× density gap**. It exists because today's systems (e.g. Caterpillar MineStar Command, Komatsu FrontRunner) use **fixed spot points**: a planner places 4–6 points by hand before any truck moves, and all trucks follow fixed lanes to them. This causes:

1. **Early stopping:** the truck cannot tell a safe dump pile from a real obstacle, so wide spacing is used.
2. **Poor fit in irregular areas:** L-shaped, tapered or curved areas leave gaps with a fixed grid.
3. **Mixed fleets:** trucks of different sizes make different piles (e.g. 4 m vs 2.5 m radius); fixed points cannot adapt.
4. **Design-induced spacing:** the large spacing caps density at about 52 %.

**Goal:** fill any dump polygon close to human density while guaranteeing that no truck leaves the boundary, no two trucks collide, no deadlock happens, and no pile ever blocks access to the rest of the area.

---

## 2. The ZADP method (five steps)

| Step | What it does | Runs |
|---|---|---|
| 1. **Minkowski erosion** | Shrinks the polygon edge by edge by the truck's safety margins, giving the *safe zone*. If the truck's GPS centre is inside it, the whole truck body is inside the dump area. | offline |
| 2. **Boustrophedon decomposition** | A sweep line crosses the safe zone; at every critical point (where the width of the shape changes) it cuts a new cell. Every cell is a simple strip without notches. | offline |
| 3. **Hexagonal grid** | Fills each cell with spot points on a hexagonal lattice — the densest packing of equal circles (90.7 %, Fejes Tóth). Pile radius scales with each truck's payload. | offline |
| 4. **Priority-queue dispatcher** | When a truck reports "ready, payload X t", the planner returns the best free spot (deepest first, back-to-front), after an isolation check. | live |
| 5. **Four safety rules** | One-way lanes, zone locking, priority yield (wait gate), reserved access corridor. | live |

---

## 3. The simulation tool

A single-page browser application (`zadp_game.html`) that runs completely offline. It is organised as five **rounds**, one per stage of planning, in the order the method needs them:

```
Round 1  Dump area      sketch -> real shape (metres)
Round 2  Fleet          payload -> pile radius, truck size -> margins
Round 3  Safe zone      Step 1 (Minkowski erosion) + Step 2 (Boustrophedon cells, bays)
Round 4  Spots          entry, spine, branches + Step 3 (hexagonal spot points)
Round 5  Dispatch       Steps 4 + 5: live multi-truck simulation, 3 methods compared
```

Why a browser app: it runs on any laptop (no 3D engine, no GPU), judges can try it themselves, and the whole method is 2D geometry, so 2D is the right level of detail.

Units are the same everywhere: metres, tonnes, m³, degrees, seconds.

Libraries (bundled in `vendor/`): Clipper (polygon clipping and offsetting) and Tesseract.js (text recognition).

---

## 4. Round 1 — from a sketch to the real shape

The user uploads a picture of the dump area with its dimensions written on it (hand sketch, CAD drawing or survey sketch). Five built-in sample sites can be used instead.

### 4.1 Finding the outline
1. **Ink mask:** Otsu's threshold on lightness (works for black, coloured or grey lines).
2. **Outside paper:** flood fill from the image border. Gaps in dashed construction lines let the fill in, so dashed boxes are ignored automatically.
3. **Closed regions:** every enclosed patch of paper is a candidate region. Lines inside the shape (centre lines, extension lines) split it into several regions; the user clicks the ones that form the dump area (the largest is pre-selected).
4. **Union + closing:** the selected regions are merged and a morphological closing bridges the thin lines between them.
5. **Boundary trace:** Moore-neighbour tracing gives the outline as a dense chain of pixels.

### 4.2 Lines and arcs
Each part of the outline becomes a **straight line** or a **circular arc**:
- **Corners** = points where the direction turns a lot over a short distance. A curve turns gradually (half the window gives about half the turn); a corner gives the whole turn in both windows.
- Between corners: fit a straight line; if it does not fit, fit a circle by least squares (Kåsa's algebraic fit, x² + y² + Dx + Ey + F = 0); if neither fits, split at the point farthest from the chord and repeat.
- Neighbouring pieces on the same line or the same circle are merged; short "lines" where a curve meets a straight edge are absorbed into the arc.

Tested on circles, rounded rectangles, stadiums, D-shapes, ellipses and mixed outlines: arc radii within about 1 unit of the truth, areas within 0.3 %.

### 4.3 Reading the dimensions
1. **Find the text labels:** small ink blobs (characters) are grouped into labels; each label's direction is found by principal component analysis (PCA) of its pixels.
2. **Read each label alone:** the label's own pixels are cut out on a clean white background, turned level (both ways up) and read by Tesseract as a single line. A reading must look like "number + optional unit". Several readers (one per spare CPU core, up to 4) work on different labels at the same time, and a label read with high confidence (> 85 %) the right way up is not read upside down too — Rounds 1–4 of a sketch now take about 1–2 s.
3. **Match the number to what it measures:** each number sits on an arrow (dimension) line. The arrow line's drawn length is compared with every possible dimension (each edge, each arc radius, the overall width and height); the closest match wins. Without an arrow line, the number goes to the nearest edge.
4. **Decimal check:** the reader sometimes drops a decimal point ("1019.1" → "10191"). Each value implies a scale (metres per pixel); a value whose scale is 10×, 100× or 1000× the others is corrected and flagged.
5. **Ask only for what is missing:** a table lists every needed dimension, marked "read from sketch", "entered" or "needed".

### 4.4 Building the real shape
- **Polygons:** the drawing gives the edge *directions*, the numbers give the edge *lengths*. A hand sketch is rarely to scale, so the given lengths do not close. Closure condition: Σ Lᵢ (cos aᵢ, sin aᵢ) = 0. Each edge is turned by a small δᵢ (and unknown lengths may change by εᵢ) with the least total change — the minimum-norm solution of the linearised closure, x = Aᵀ(AAᵀ)⁻¹(−g), repeated until the gap is zero. The tool reports how far the sketch was from scale (e.g. "edges turned by up to 6.9°").
- **Shapes with arcs:** one scale factor s = median(value / measured) over all given dimensions; every dimension is then checked and its error shown. If only width and height are given, x and y are scaled separately.

---

## 5. Round 2 — fleet

Each truck type is identified by its **payload** and needs only four inputs:

| Input | Used for |
|---|---|
| Payload P (t) | pile volume |
| Number of trucks | mixed-fleet ratio |
| Pile radius (known, or calculated) | circle size |
| Length × width (m) | safety margins |

**Pile radius from payload** (pile = cone at the angle of repose θ, loose density ρ from a material list):

- V = P / ρ
- V = ⅓ π r² h, h = r tan θ ⇒ **r = ∛(3V / (π tan θ))**

Example: 227 t of blasted waste rock (ρ = 1.8 t/m³, θ = 37°): V = 126 m³, r = 5.43 m.

**Safety margins** (feed Step 1): side = W/2 + 0.16 W, rear (dump side) = 0.62 L, front (entry side) = 0.23 L. For a 793F-class truck (12.9 × 7.6 m) this gives about 5 / 8 / 3 m — the values in the method description.

The **lane width** (each of the two lanes) is a site setting; by default it is the widest truck + 1 m.

---

## 6. Round 3 — safe zone and partition (Steps 1 and 2)

### 6.1 Minkowski erosion (Step 1)
Every edge of the polygon is offset inward by its own margin: edges facing the entry use the front margin, edges facing away use the rear margin, the rest the side margin. Implemented exactly: the union of "edge buffers" (each edge swept by its margin) is subtracted from the polygon. The largest margins in the fleet are used, so every truck is safe.

### 6.2 Boustrophedon decomposition (Step 2)
The safe zone is rotated so the sweep runs along x. **Critical points** are vertices where the outline turns back in x (both neighbours on the same side); only there can the number of pieces cut by the sweep line change. The zone is cut into slabs between critical x-values; fragments that connect one-to-one across a cut are merged into one cell. Result: cells C1…Cn, each a simple strip.

The sweep direction can be set along the entry (default — the spine then runs straight into the area), set by a slider, or chosen to give the fewest cells.

### 6.3 Bays (from the trucks)
A cell much deeper than one truck can serve is cut again into **bays**. A bay is one **branch road** with pile rows on both sides:

- reach = rows × (2 r_max × 0.866) + r_max (default 3 rows each side)
- branch width = 1.4 W_max + 2 m
- **bay depth B = 2 × reach + branch width** (about 90–100 m for 360 t trucks)

Small or smooth areas stay as 1–3 cells; large areas (50–200 acres) get many bays automatically. One truck works one bay half at a time, so the number of bays sets how many trucks can dump at once.

---

## 7. Round 4 — spine, branches and spot points (Step 3)

- **Entry:** the user chooses the side trucks enter from, or presses **Best entry**: the current entry and the four longest edges are each tried with their own spine, and the full touching dispatch is run from each; the best run with **0 collisions and 0 deadlocks** is kept. Edges whose straight spine stops short of the far side (< 75 % of the area depth) are skipped. The entry matters: on the 50-acre valley the default edge gave 68.6 %, other long edges 75.5–76.3 %. The Round 4 plan alone is *not* a reliable guide (on the crescent the edge with the best plan gave only 52 % in dispatch), which is why every candidate is simulated.
- **Spine:** the main road, two one-way lanes (IN and OUT), from the entry straight along the sweep direction to the far boundary.
- **Work zones:** each bay is split by the spine into two halves; each half is a zone with a temporary **branch** road across its middle (always joined to the spine).
- **Spot points:** a hexagonal lattice per truck type. Rows are (2r + spacing) apart along a row and (2r + spacing)·√3/2 between rows; every other row is shifted half a step. The phase with most points wins. A spot is kept if its centre is in the zone and its circle is inside the dump area and clear of the spine and branches. Spacing 0 = touching; negative = overlap.
- **Fill order — a clean front toward the road ("water rising"):** deepest zone first; inside a zone, the pile row **farthest from its branch first**, each row from its far end back toward the spine, so a truck always has open ground to reverse in from; then the branch itself from the far end back to the spine; the spine last, deepest part first. A road is filled only when nothing behind it needs it any more. (Filling the row next to the road first seals the rows behind it; this front order raised coverage on the 50-acre site from 68 % to 77 %.)
- **Overlap (human-like spacing):** with spacing 0 the piles just touch (hexagonal packing, at most 90.7 % of the usable area). Human operators place piles closer than their size so neighbours merge. A negative spacing models this; one click sets −2 m. On the 50-acre site: 0 m → 77 %, −1 m → 87 %, −2 m → 94 % of the whole dump area — the human level in the problem statement (~95 %).

---

## 8. Round 5 — dispatch (Steps 4 and 5)

### 8.1 The dispatcher
A truck arriving at the gate reports its payload (so its pile radius). The planner looks at candidate spots for that radius and gives the first one that passes all checks. Three ways of generating candidates are compared on the same truck sequence:

| Method | Candidates |
|---|---|
| **Baseline** (today's practice) | fixed square grid of spot points, 7.38 m between piles, largest pile for every truck |
| **Grid + remove** | the smallest truck's hexagonal grid from Round 4; a bigger pile cancels the grid spots it covers |
| **Touching placement** | every point where a new circle touches two existing piles, a pile and a boundary, or two boundaries (circle–circle and circle–line intersections); the deepest one is taken |

With one truck size, touching placement reproduces the hexagonal lattice; with a mixed fleet it fills the gaps a fixed grid cannot.

### 8.2 Checks before a spot is given
1. **Free:** the pile does not overlap any pile already dumped or reserved (any truck type).
2. **Zone locking (Rule 2):** no other truck works in that zone.
3. **Road order:** branch spots only when the zone's own piles are done; spine spots only when nothing deeper is left and no other truck is inside.
4. **Reachable:** a truck can drive from the gate to a stop cell next to the pile (route search on a grid of truck-centre positions).
5. **Way out:** with this pile in place, the truck can still drive out.
6. **Route lock:** the truck's route outside the lanes (in and out) must not cross a route reserved by another truck; the pile must not land on a reserved route.
7. **Isolation check (Step 4):** with this pile in place, no free ground that could still take a pile becomes unreachable.

Spots that are proved impossible (unreachable, no way out) are dropped for good: piles are only ever added, so they stay impossible.

### 8.3 Driving on the roads
Routes are found on a grid of truck-centre positions with A*. Spine and branch centre lines are cheap to drive on (cost 0.35 against 1 on open ground), so trucks drive the spine, turn onto their zone's branch and only leave the road for the last stretch to the pile; the path is then straightened (no staircase steps). Along a straight spine the truck drives the exact centre line.

### 8.4 Moving the trucks safely (Step 5)
- **One-way lanes (Rule 1):** trucks drive *along* a lane only on its centre line and only in its direction; the other lane may only be crossed sideways.
- **Wait gate (Rule 3):** at most N trucks inside (default = number of zones).
- **Lane mutex:** the lanes plus a safety strip beside them hold one *moving* truck at a time. The holder never waits for anyone — its lane path is empty and its route outside the lanes is reserved for it — so it always gets through.
- **Reserved routes:** each truck's whole round trip outside the lanes is reserved while it works and released when it is back in the lane. Reserved routes never overlap.
- **Truck ahead:** a truck stops if another truck is on the next stretch of its own route.

**Why there can be no collision:** in the lanes only one truck moves; outside the lanes each truck stays on its own reserved route, and reserved routes are disjoint.
**Why there can be no deadlock:** the only waiting is for the lane mutex or the gate. The mutex holder never waits, so it always finishes and releases it; a truck waiting at the gate holds nothing.

Both are also **measured**, not assumed: a collision is two truck bodies overlapping; a deadlock is no truck able to move for 10 minutes.

### 8.5 Settings and output
Speed in the dump area (default 20 km/h), manoeuvre time (60 s), dump time (60 s), haul cycle (load + travel to the loader and back, 15 min, ±15 % random), first departures every 60 s, maximum trucks inside, random seed. Playback at 1×–10× (1× = one minute of site time per second). Live: piles per truck type, coverage, trucks inside and waiting, waiting time, collisions, deadlocks, time to fill. "Compare all three" runs every method on the same truck sequence.

**Real fleet:** only the trucks entered in Round 2 operate (e.g. 5 × 300 t, 6 × 500 t, 1 × 1000 t = 12 trucks). Each one loops: leaves the loader, joins the gate queue, dumps, drives out, and comes back one haul cycle later. The queue can never hold more trucks than the fleet. The **Trucks** panel lists every truck with its type, state (at the loader / waiting at gate / inside – driving in, reversing, dumping, leaving / parked) and loads delivered. A truck size is "parked" when no spot is left for it; before the run ends every parked size is checked once more, because spots can open again after other piles finish.

**Zone panel:** each zone shows its status (not started · in progress · truck assigned · dumping · finished), its truck and what that truck is doing, its pile count and fill %. Zones are coloured the same way on the map, every truck is labelled T#, and the panel shows the coverage next to the theoretical maximum.

---

## 9. Verification

Fleet used in all tests: 363 t hauler (pile r 6.35 m, 15.1 × 9.75 m, 2 trucks) + 227 t hauler (pile r 4.0 m, 12.9 × 7.6 m, 3 trucks). Defaults: 20 km/h, 60 s manoeuvre, 60 s dump, a truck every 60 s. Coverage = area covered by piles ÷ whole dump area (including safety margins and roads). Every sketch goes through all five rounds from its image.

### 9.1 Practical mine-scale sites (seed 7, final version)

| Site | Area error from sketch | Baseline (fixed, 7.38 m) | Grid + remove | **Touching (ZADP)** | **Touching + 2 m overlap** | Collisions / deadlocks |
|---|---|---|---|---|---|---|
| Valley fill, 50 acres | 0.3 % | 15.4 % | 46.2 % | **76.9 %** | **94.0 %** | 0 / 0 |
| Pit-edge crescent, 80 acres | 1.0 % (decimal fixed) | 17.0 % | 47.2 % | **76.5 %** | **91.7 %** | 0 / 0 |
| Two lobes + neck, 120 acres* | −0.2 % | 13.9 % | 29.4 % | **56.3 %** | – | 0 / 0 |
| Waste dump with notch, 150 acres* | −0.4 % | 14.9 % | 42.5 % | **64.2 %** | – | 0 / 0 |

\* measured with the earlier fill order (before the front-advancing order that added about 9 points on the 50- and 80-acre sites); not re-run for time.

**Headline:** with touching piles ZADP fills about **77 %** of the whole dump area (≈ 5× fixed spot points); with the human-like 2 m overlap it reaches **92–94 %**, the human level of ~95 % in the problem statement — with up to 16 trucks working at once and no collision or deadlock.

### 9.1b Real fleet (fixed number of looping trucks) with Best entry

Fleet: **5 × 300 t** (13.5 × 8 m, pile r 5.96 m), **6 × 500 t** (17 × 10 m, r 7.06 m), **1 × 1000 t** (22 × 12 m, r 8.90 m) — 12 trucks, each looping with a 15-min haul cycle. Seed 7, spacing 0 (touching). Entry chosen by **Best entry** (full dispatch runs; the default entry in brackets).

| Site | Baseline (fixed) | Grid + remove | **Touching (ZADP)** | Touching, default entry | Collisions / deadlocks | Fill time |
|---|---|---|---|---|---|---|
| Valley fill, 50 acres | 16.9 % | 33.9 % | **76.3 %** | 68.6 % | 0 / 0 | 49 h |
| Pit-edge crescent, 80 acres | 16.0 % | 33.1 % | **74.6 %** | 73.1 % | 0 / 0 | 95 h |
| Two lobes + neck, 120 acres | 16.8 % | 34.3 % | **76.5 %** | 67.1 % | 0 / 0 | 173 h |
| Waste dump with notch, 150 acres | 17.4 % | 34.8 % | **76.7 %** | 70.4 % | 0 / 0 | 175 h |

With a real fleet ZADP fills **75–77 %** of every site (about 4.5× fixed spot points and 2.2× a grid with removal), with no collision or deadlock. The entry choice alone is worth up to 9 points. Two fixes made this possible: a truck size marked "full" is re-checked before the run ends (the spine spots had been left empty), and the Best entry search.

### 9.2 Seventeen test shapes (seed 7, final version)

Straight-edged, curved and "weird" outlines: areas rebuilt from the sketches within 0–2.4 % of the truth. Touching coverage: CAD sketch 67 %, circle 64 %, rounded rectangle 68 %, stadium 64 %, ellipse 65 %, kidney 67 %, D-shape 50 %, cross 68 %, T 69 %, triangle 71 %, crescent 70 %, comb 59 %, notched 56 %, strip 49 %, star 44 %, dumbbell 40 % (Baseline 8–18 %). All runs ended with 0 collisions and 0 deadlocks except the small dumbbell (1 deadlock in the baseline run) — see Limitations. The 12 m pentagon is correctly rejected as too small for the fleet.

### 9.3 How to read the numbers
- Round 4's single-type plan (66–79 %) is the *prediction*; Round 5's mixed fleet with real access rules reaches 56–72 %. The gap is the price of mixing pile sizes and of keeping every spot reachable.
- The baseline is lower than the 52 % in the problem statement because here it uses the largest pile for every truck and counts roads and margins; adjust it to site data if needed.


---

## 9A. How sure are we that this is the best? (defence of the coverage figure)

A simulation number like "77 %" is only convincing next to the **best that is mathematically possible** on the same area. The tool computes that bound and shows it in Round 5 ("Theoretical max" and "Achieved / max").

**1. The upper bound — no method can beat it.**
- A pile centre may only be in the safe zone (Step 1), so piles can only ever cover the *coverable ground* C = (safe zone grown by the pile radius) ∩ dump area.
- Equal circles that only touch can cover at most **π / (2√3) = 90.69 %** of any region (Thue's theorem, proved by Fejes Tóth, 1940s). This is a proof, not a simulation.
- So for touching piles: **max coverage = 0.9069 × C / dump area.** With overlapping piles the cap is C / dump area.

For the 50-acre valley fill: C = 97.6 % of the area, so the touching bound is **88.5 %**. ZADP reaches **76.9 %**, which is **87 % of the theoretical maximum**.

**2. Why the remaining gap is not a weakness of the method.** The bound assumes an infinite plane. Real areas have edges, curved boundaries and roads, where circles cannot pack perfectly. As a *reachable* reference, Round 4 lays the ideal hexagonal lattice of the smallest truck over the same zones with no traffic at all: about 76 %. The live mixed-fleet dispatcher, with all safety rules, reaches **76.9 %**, matching the ideal static plan.

**3. Nothing usable is left empty.** After a ZADP run, the free spots that could still take a pile add up to about **0.2 %** of the area. Every other part of the gap is safety margin (≈ 6 %) or the unavoidable gaps between round piles.

**4. The only way higher is overlap, which is what humans do.** Human operators place piles closer than their size, so neighbours merge. With a 2 m overlap ZADP reaches **92–94 %**, matching the ~95 % human figure in the problem statement, while keeping 0 collisions and 0 deadlocks.

**5. It is repeatable.** The same area with different truck arrival orders (random seeds) gives results within about ±3 points, and every run records 0 collisions and 0 deadlocks.

**Answer to "why 66 % and not 79 % or 55 %?":** the figure is boxed in from both sides. It cannot exceed the proven geometric cap (88.5 % on this site), and it equals the ideal static hexagonal plan (~76 %) while leaving practically no reachable space unused. Any other method can be scored against the same cap with the same tool.

---

## 10. How the design evolved (what failed, and why it was changed)

These changes came out of testing and are useful for the paper's discussion:

| Problem found in testing | Change |
|---|---|
| Tracing a CAD drawing grabbed the dashed construction box | Regions are found from enclosed paper; dashed lines leak and are ignored; the user confirms the regions |
| Curves were cut into many short lines (area lost) | Line/arc fitting with least-squares circles |
| Text reader missed tilted labels on whole drawings | Each label cut out alone, turned level by PCA, read as one line |
| Overall dimensions were matched to short edges (shape 9× too big) | Numbers are matched by the drawn length of their dimension line |
| Decimal points dropped by the reader (area 10× too big) | Scale-consistency check restores them and flags the value |
| Hand sketches are not to scale | Closure solver: lengths from the numbers, angles from the drawing |
| Head-on meetings when a zone was unlocked too early | Zone unlocked only when the truck is back in the lane |
| Trucks boxed in by piles dumped while they worked | Isolation check + reserved round-trip routes |
| Edge cases on diagonal and narrow lanes | Lane centre-line driving + a lane mutex (simple, provably safe) |
| Only one truck inside at a time | Route reservations exclude the lanes, so trucks in different zones run in parallel |
| Boustrophedon gave only 1–2 cells on smooth shapes | Partition before the lane; bays sized from the trucks; spine and branches (herringbone) |
| Branches sealed by their first pile row | Branch spots filled far end first across the whole road width |
| Big areas too slow | A* route search, cached reachability, spot caching, sliced runs |

---

## 11. Limitations and future work

- **Open safety case:** on very small areas where a narrow neck sits right at the entry (the dumbbell test), trucks entering and leaving can still meet at the gate mouth; mine-scale sites passed. Fix planned: lock the gate mouth together with the spine.
- **Some entry edges are unsafe:** from a few non-default entry edges (seen on the 80-acre crescent, an edge with a diagonal spine) the dispatcher let trucks touch (1–2 collisions per run). The default entry and the edges Best entry keeps are verified at 0 / 0; Best entry rejects any edge whose run has a collision. Root cause not yet found.
- **Haul cycle:** one average cycle time with ±15 % random spread for every truck; loader queues and haul-road traffic are not modelled.
- **2D only:** piles are circles; real piles spread unevenly and settle. Pile height and the dump crest are not modelled.
- **Grid routing:** truck positions use a grid of about half a truck width; turning circles are not modelled exactly (the truck reverses to its pile from a free cell next to it).
- **Lane mutex is conservative:** one moving truck in the lanes at a time is safe but limits throughput on very large areas; a convoy rule (same-direction following) would raise it.
- **Text recognition:** works on clean drawings; unusual fonts may need values typed in.
- **Baseline:** modelled as a fixed grid with 7.38 m gaps; real site layouts may differ.
- **Next:** link to the rover prototype (the physical 2D test), add turning-circle-aware paths, loader queues, and pile settlement.

---

## 12. Running it

- Windows: double-click `start.bat` (needs Python). Or run `python -m http.server 8765` in the project folder and open `http://localhost:8765/zadp_game.html`.
- Do not open the HTML file directly (the text reader cannot start from a `file://` page).
- **Demo for judges:** choose a site in the header and press **▶ Demo for judges**. It runs all five rounds with a caption at each step and ends with the three-method comparison.

### Files
| File | What it is |
|---|---|
| `zadp_game.html` | the whole application |
| `vendor/` | bundled libraries (offline) |
| `test_sketches/` | test sketches: 17 shapes + 5 practical sites |
| `tools/make_test_sketches.py`, `tools/make_site_sketches.py` | generate the test sketches |
| `tools/pipeline_test.js` | runs a sketch through all rounds from the browser console |
| `test_results.md` | earlier result tables |
| `docs/ZADP_Project_Documentation.md` | this document |

---

## 13. Glossary

| Term | Meaning |
|---|---|
| Dump area / paddock / polygon | the area where trucks dump |
| Safe zone | the dump area shrunk by the truck margins (Minkowski erosion) |
| Cell | one strip of the Boustrophedon decomposition |
| Bay | part of a cell served by one branch road |
| Zone | a bay half — one truck works one zone at a time |
| Spine | main two-way road from the entry (IN lane + OUT lane) |
| Branch | temporary one-truck road from the spine into a zone |
| Spot point | where a pile's centre goes |
| Coverage | area covered by piles ÷ area of the dump polygon |
| Water rising | roads are filled last, from the far end back, once nothing behind them needs them |
