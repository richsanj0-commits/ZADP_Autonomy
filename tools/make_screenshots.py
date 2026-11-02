# Presentation screenshots, step by step (1920x1080), using the system Chrome.
# Needs the page served: `python -m http.server 8777` in the project folder.  Run:  python tools/make_screenshots.py
import os
from playwright.sync_api import sync_playwright

URL = "http://localhost:8777/zadp_game.html?shots=1"
OUT = os.path.join(os.path.dirname(__file__), "..", "docs", "screenshots")
os.makedirs(OUT, exist_ok=True)
SITE, EDGE = "site_valley_fill_50ac", 12

with sync_playwright() as p:
    b = p.chromium.launch(channel="chrome", headless=True)
    pg = b.new_page(viewport={"width": 1920, "height": 1080}, device_scale_factor=1)
    pg.goto(URL); pg.wait_for_timeout(1500)
    pg.evaluate("localStorage.clear()"); pg.reload(); pg.wait_for_timeout(1500)
    pg.evaluate("async () => { await import('/tools/pipeline_test.js'); }")
    shot = lambda name: (pg.wait_for_timeout(700), pg.screenshot(path=os.path.join(OUT, name)), print("saved", name))

    # fleet: 5 x 300 t, 6 x 500 t, 1 x 1000 t
    pg.evaluate("""() => { S.fleet = []; [[300,5,13.5,8],[500,6,17,10],[1000,1,22,12]].forEach(([pl,c,L,W],i) => { $('addTruck').click();
        Object.assign(S.fleet[i], { name: pl + ' t hauler', payload: pl, count: c, L, W }); }); S.gap = {}; saveSession(); }""")

    # Round 1: sketch analysed, then real shape
    pg.evaluate(f"""async () => {{ setRound(1); const i = new Image(); await new Promise(r => {{ i.onload = r; i.src = '/test_sketches/{SITE}.png'; }});
        analyseImage(i, '{SITE}.png'); await new Promise(r => setTimeout(r, 300)); if (S.mode === 'pick') useSelection();
        for (let k = 0; k < 200 && !/^(Read \\d|No numbers)/.test($('progMsg').textContent); k++) await new Promise(r => setTimeout(r, 250));
        S.view1 = 'sketch'; setViewTab(); fitView(); draw(); }}""")
    shot("01_round1_sketch_read.png")
    pg.evaluate("() => { $('buildBtn').click(); S.view1 = 'real'; setViewTab(); fitView(); draw(); }")
    shot("02_round1_real_shape.png")
    pg.evaluate("() => { $('areaName').value = 'Valley fill 50 ac'; $('saveArea').click(); setRound(2); }")
    shot("03_round2_fleet.png")

    # Round 3: safe zone + Boustrophedon partition
    pg.evaluate("() => { S.sweep = null; setRound(3); }")
    shot("04_round3_safe_zone_partition.png")

    # Round 4: entry, spine, branches, hex spots
    pg.evaluate(f"""() => {{ setRound(4); const p = S.poly, e = {EDGE}, a = p[e], b2 = p[(e + 1) % p.length];
        S.entry = {{ i: e, pt: [(a[0] + b2[0]) / 2, (a[1] + b2[1]) / 2] }}; S.sweep = sweepAlongEntry(); run4(); }}""")
    shot("05_round4_spine_zones_spots.png")

    # Round 5: dispatch in progress, then finished
    pg.evaluate("() => { S.simCfg.method = 'touch'; setRound(5); resetSim(); while (!SIM.done && SIM.piles.filter(q => q.state === 'placed').length < 420) stepSim(SIM, 1); render5(); draw(); }")
    shot("06_round5_dispatch_running.png")
    pg.evaluate("() => { while (!SIM.done && SIM.t < 400 * 3600) stepSim(SIM, 1); render5(); draw(); }")
    shot("07_round5_finished_zadp.png")

    # baseline for contrast
    pg.evaluate("() => { S.simCfg.method = 'base'; resetSim(); while (!SIM.done && SIM.t < 400 * 3600) stepSim(SIM, 1); render5(); draw(); }")
    shot("08_round5_baseline_fixed_spots.png")

    # compare all methods
    pg.evaluate("() => { S.simCfg.method = 'touch'; resetSim(); $('cmpBtn').click(); }")
    pg.wait_for_function("() => /Same area|stopped/.test(document.getElementById('cmpBox').textContent)", timeout=600000)
    pg.evaluate("() => document.getElementById('cmpBox').scrollIntoView({ block: 'center' })")
    shot("09_round5_compare_all_methods.png")

    # human-like overlap (-2 m)
    pg.evaluate("() => { setRound(4); $('gapHuman').click(); setRound(5); S.simCfg.method = 'touch'; resetSim(); while (!SIM.done && SIM.t < 400 * 3600) stepSim(SIM, 1); render5(); draw(); }")
    shot("10_round5_overlap_2m.png")
    b.close()
