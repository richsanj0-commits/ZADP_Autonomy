# Export a Round 4 plan for the MATLAB twin, using the system Chrome.
# Needs the page served: `python -m http.server 8777` in the project folder.  Run:
#   python tools/export_plan.py                                            valley site -> twin3d/plan/plan_valley.json
#   python tools/export_plan.py --arena twin3d/plan/arena_site.json        1 m arena at fixed scale -> twin3d/plan/plan_arena.json
#   (add --rows N to set "Pile rows each side of a branch" in arena mode)
#   (add --out NAME to write twin3d/plan/NAME instead, e.g. --arena twin3d/maps/trap_site.json --out ../maps/plan_trap.json;
#    without --out the arena plan goes to plan_arena.json, the reference plan the tests pin)
import os, sys, json
from playwright.sync_api import sync_playwright

URL = "http://localhost:8777/zadp_game.html?shots=1"
HERE = os.path.dirname(os.path.abspath(__file__))
PLAN = os.path.join(HERE, "..", "twin3d", "plan")
SITE, EDGE = "site_valley_fill_50ac", 12
args = sys.argv[1:]
arg = lambda k: args[args.index(k) + 1] if k in args else None

def valley(pg):
    pg.evaluate("""() => { S.fleet = []; [[300,5,13.5,8],[500,6,17,10],[1000,1,22,12]].forEach(([pl,c,L,W],i) => { $('addTruck').click();
        Object.assign(S.fleet[i], { name: pl + ' t hauler', payload: pl, count: c, L, W }); }); S.gap = {}; saveSession(); }""")
    pg.evaluate(f"""async () => {{ setRound(1); const i = new Image(); await new Promise(r => {{ i.onload = r; i.src = '/test_sketches/{SITE}.png'; }});
        analyseImage(i, '{SITE}.png'); await new Promise(r => setTimeout(r, 300)); if (S.mode === 'pick') useSelection();
        for (let k = 0; k < 200 && !/^(Read \d|No numbers)/.test($('progMsg').textContent); k++) await new Promise(r => setTimeout(r, 250)); }}""")
    pg.evaluate("() => { $('buildBtn').click(); $('areaName').value = 'Valley fill 50 ac'; $('saveArea').click(); setRound(3); }")
    pg.evaluate(f"""() => {{ setRound(4); const p = S.poly, e = {EDGE}, a = p[e], b2 = p[(e + 1) % p.length];
        S.entry = {{ i: e, pt: [(a[0] + b2[0]) / 2, (a[1] + b2[1]) / 2] }}; S.sweep = sweepAlongEntry(); run4(); }}""")
    return pg.evaluate("() => exportPlan(2.0, 1.5)"), "plan_valley.json"

def arena(pg, cfg, rows):
    # fleet from the file (Round 2 state), then the finished Round 1 shape set directly: S.poly + S.entry + S.areaSaved (what Save stores)
    pg.evaluate("""(cfg) => { S.fleet = []; cfg.fleet.forEach((t, i) => { $('addTruck').click();
        Object.assign(S.fleet[i], { name: t.name, payload: t.payload, count: t.count, L: t.L, W: t.W }); }); S.gap = {}; }""", cfg)
    if cfg.get("laneW"): pg.evaluate("(w) => { S.laneW = w; $('laneW').value = w; }", cfg["laneW"])  # one lane, site metres
    if rows is not None: pg.evaluate("(n) => { S.rows = n; $('rowsIn').value = n; }", rows)
    pg.evaluate("""(cfg) => { const p = cfg.poly, e = cfg.entryEdge, a = p[e], b = p[(e + 1) % p.length];
        loadRecord({ name: cfg.name, units: 'm', polygon: p, entry: { i: e, pt: [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2] }, dimensions: [],
            area_m2: Math.abs(area(p)), perimeter_m: perim(p), source: 'arena_site.json', saved: new Date().toISOString() });
        S.areaSaved = S.poly && areaRecord(); saveSession(); }""", cfg)
    pg.evaluate("() => { setRound(2); setRound(3); setRound(4); S.sweep = sweepAlongEntry(); run4(); }")
    info = pg.evaluate("""() => ({ round: S.round, banner: $('banner').textContent, safe: S.safe.reduce((s, p) => s + Math.abs(area(p)), 0), bay: S.part && S.part.B,
        zones: S.r4 ? S.r4.zones.length : 0, laneW: S.r4 ? S.r4.lane.w : null, rows: S.rows })""")
    print("run4:", info)
    ok = info["round"] == 4 and info["zones"] > 0
    plan = pg.evaluate("(c) => exportPlan(c.arena.w, c.arena.h, c.scale)", cfg) if ok else None
    return plan, "plan_arena.json"

with sync_playwright() as p:
    b = p.chromium.launch(channel="chrome", headless=True)
    pg = b.new_page(viewport={"width": 1920, "height": 1080}, device_scale_factor=1)
    pg.goto(URL); pg.wait_for_timeout(1500)
    pg.evaluate("localStorage.clear()"); pg.reload(); pg.wait_for_timeout(1500)
    pg.evaluate("async () => { await import('/tools/pipeline_test.js'); }")
    if arg("--arena"):
        cfg = json.load(open(arg("--arena")))
        plan, name = arena(pg, cfg, int(arg("--rows")) if arg("--rows") else None)
    else:
        plan, name = valley(pg)
    b.close()

if plan is None: sys.exit("no plan produced (see run4 line above)")
if arg("--out"): name = arg("--out")                 # other maps must not overwrite the reference plan
OUT = os.path.join(PLAN, name)
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w") as f: json.dump(plan, f, separators=(",", ":"))
print("wrote", os.path.normpath(OUT), os.path.getsize(OUT), "bytes")
