// Browser test helper: runs one sketch through Rounds 1–5 and reports the results.
// Load in the page console:  await import('/tools/pipeline_test.js')   then   await pipe('weird_cross')
window.truth = {
  weird_star: [209.23, 16164], weird_cross: [210, 24500], weird_strip: [320, 14400], weird_crescent: [187.51, 21666],
  weird_dumbbell: [360, 34873], weird_comb: [260, 31600], weird_triangle: [240, 24000], weird_T: [240, 25600], weird_notched: [266.22, 35085],
  circle_d80: [80, 5017], roundrect_120x80_r20: [120, 9254], stadium_150x60: [150, 8222], dshape_130x100: [130, 11923],
  ellipse_140x80: [140, 8787], kidney: [175, 14463], cad_curved: [176.3607, null], pentagon: [null, null],
  site_waste_dump_150ac: [907.6, 607028], site_valley_fill_50ac: [715.7, 202343], site_L_paddock_200ac: [1157.3, 809371],
  site_pit_crescent_80ac: [1019.1, 323748], site_two_lobes_120ac: [1177.9, 485623],
};
// test fleet: two truck types
window.testFleet = () => {
  S.fleet = [];
  $("addTruck").click(); Object.assign(S.fleet[0], { name: "Big hauler", payload: 363, count: 2, L: 15.1, W: 9.75 });
  $("addTruck").click(); Object.assign(S.fleet[1], { name: "Small hauler", payload: 227, count: 3, L: 12.9, W: 7.6, rMode: "in", rIn: 4 });
  saveSession();
};
window.pipe = async (fn, seeds = [7, 11], methods = ["base", "grid", "touch"]) => {
  const load = async u => { const i = new Image(); await new Promise(r => { i.onload = r; i.src = u; }); return i; };
  const r = {}, T0 = performance.now();
  if (!S.fleet.length) testFleet();
  setRound(1); analyseImage(await load("/test_sketches/" + fn + ".png"), fn + ".png");
  await new Promise(z => setTimeout(z, 150)); if (S.mode === "pick") { r.pick = S.cands.length; useSelection(); }
  for (let k = 0; k < 400 && !/^(Read \d|No numbers|Could not|The text reader)/.test($("progMsg").textContent); k++) await new Promise(z => setTimeout(z, 250));
  r.segs = S.segs.map(s => s.t === "arc" ? "A" : "L").join("");
  r.read = S.dims.filter(d => d.src === "sketch").map(d => dimName(d).split(" ")[0] + "=" + d.value).join(" ");
  if (!S.dims.some(d => d.value > 0)) { const w = S.dims.find(d => d.kind === "width"); w.value = truth[fn][0]; w.src = "user"; r.note = "width typed"; }
  $("buildBtn").click();
  if (truth[fn] && truth[fn][1]) r.areaErr = ((Math.abs(area(S.poly)) - truth[fn][1]) / truth[fn][1] * 100).toFixed(1) + "%";
  $("areaName").value = fn; $("saveArea").click(); setRound(3);
  r.usable = ($("stats3").innerText.match(/Usable\n([\d.]+)/) || [])[1] + "%";
  S.sweep = null; setRound(4);
  if (S.r4 && window.useBestEntry) r.best = await bestEntry();
  if (!S.r4 || S.round !== 4) { r.note = "no safe zone (area too small)"; r.r5 = {}; r.totalMs = Math.round(performance.now() - T0); return r; }
  r.zones = S.r4.zones.length + "@" + S.sweep + "°";
  r.r4 = Object.values(S.r4.grids).map(g => g.nZone + "+" + (g.nBranch || 0) + "+" + g.nLane + " (" + g.cov.toFixed(0) + "%)").join(" | ");
  r.bays = S.part ? S.part.cells.length + " cells / " + S.part.bays.length + " bays / " + S.r4.zones.length + " zones" : "-";
  r.t4 = Math.round(performance.now() - T0);
  r.r5 = {};
  for (const seed of seeds) for (const m of methods) {
    const t0 = performance.now(), W = runToEnd(newSim(m, Object.assign({}, S.simCfg, { seed }))), s = simSummary(W);
    r.r5[m + seed] = `${s.cov.toFixed(1)}% c${s.collisions} d${s.deadlocks}${W.done ? "" : " NOTDONE"} ${Math.round(performance.now() - t0)}ms`;
  }
  r.totalMs = Math.round(performance.now() - T0);
  return r;
};
// background runner (keeps each browser call short): start with runBg(name), poll window.__r
window.runBg = (fn, seeds, methods) => { window.__r = null; pipe(fn, seeds, methods).then(x => window.__r = x).catch(e => window.__r = { error: String(e) }); return "started"; };
// big sites: rounds 1-4 via pipe (no round 5), then each method run in slices (page stays responsive)
window.siteRun = async (fn, seeds = [7], methods = ["base", "grid", "touch"]) => {
  const r = await pipe(fn, [], []);
  if (r.note) return r;
  r.r5 = {};
  for (const seed of seeds) for (const m of methods) {
    const t0 = performance.now(), W = await runChunked(newSim(m, Object.assign({}, S.simCfg, { seed })));
    const s = simSummary(W);
    r.r5[m + seed] = `${s.cov.toFixed(1)}% ${s.piles}p ${s.hours.toFixed(0)}h c${s.collisions} d${s.deadlocks}${W.done ? "" : " NOTDONE"} ${Math.round((performance.now() - t0) / 1000)}s`;
    window.__siteP = { fn, ...r };
  }
  return r;
};
window.siteBg = (list, seeds, methods) => { window.__site = {}; window.__siteBusy = true;
  (async () => { for (const fn of list) { try { window.__site[fn] = await siteRun(fn, seeds, methods); } catch (e) { window.__site[fn] = { error: String(e) }; } } window.__siteBusy = false; })(); return "started"; };
// final table: methods at spacing 0, plus touching at -2 m overlap
window.finalBg = (list) => { window.__fin = {}; window.__finBusy = true;
  (async () => { for (const fn of list) { try {
    S.fleet.forEach(t => S.gap[t.id] = 0);
    const r = await siteRun(fn, [7]);
    S.fleet.forEach(t => S.gap[t.id] = -2); run4();
    const W = await runChunked(newSim("touch", Object.assign({}, S.simCfg, { seed: 7 }))), s = simSummary(W);
    r.r5.overlap2 = `${s.cov.toFixed(1)}% ${s.piles}p ${s.hours.toFixed(0)}h c${s.collisions} d${s.deadlocks}${W.done ? "" : " NOTDONE"}`;
    S.fleet.forEach(t => S.gap[t.id] = 0);
    window.__fin[fn] = r; } catch (e) { window.__fin[fn] = { error: String(e) }; } } window.__finBusy = false; })(); return "started"; };
// one site from a chosen entry edge (null = default), one method; returns the summary + timing
window.runEdge = async (fn, edge = null, method = "touch", seed = 7) => {
  const r = await pipe(fn, [], []); if (r.note) return r;
  if (edge != null) { const p = S.poly, a = p[edge], b = p[(edge + 1) % p.length]; S.entry = { i: edge, pt: [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2] }; S.sweep = sweepAlongEntry(); run4(); }
  const t0 = performance.now(), W = await runChunked(newSim(method, Object.assign({}, S.simCfg, { seed }))), s = simSummary(W); window.__W = W;
  return { fn, edge: S.entry.i, sweep: S.sweep, zones: S.r4.zones.length, cov: +s.cov.toFixed(1), piles: s.piles, h: +s.hours.toFixed(0), c: s.collisions, d: s.deadlocks, forced: s.forcedDumps || 0, done: W.done, sec: Math.round((performance.now() - t0) / 1000) };
};
window.runList = (list) => { window.__rl = []; window.__rlBusy = true; (async () => { for (const [fn, e, m] of list) { try { window.__rl.push(await runEdge(fn, e, m)); } catch (err) { window.__rl.push({ fn, e, error: String(err && err.stack || err) }); } } window.__rlBusy = false; })(); return "started"; };

// ---------------------------------------------------------------- evidence for the coverage defence
// coverable ground C = (safe zone grown by r) ∩ dump area, as % of the area, for a pile radius r
window.coverable = (r) => sumArea(clip(offsetPaths(S.safe.filter(p => area(p) > 0), r), [S.poly], CT().ctIntersection)) / Math.abs(area(S.poly)) * 100;
// offline oracle: the same geometry and tangency candidates, but NO trucks, routes, locks or fill-order rules.
// Greedy: place a pile on a free candidate until none is left (a saturated packing). order: "fill" (dispatcher's order),
// "front" (shallow first), "random" (seeded random pick = random sequential packing on tangency points)
window.oraclePack = (tyIdx = null, order = "fill", seed = 1) => {
  const W = newSim("touch", Object.assign({}, S.simCfg, { seed })), rnd = rng(seed * 7 + 3);
  const ty = tyIdx == null ? W.smallest : W.types[tyIdx];
  const ovl = o => W.hash.near(o.c, ty.r + W.rmax + W.gmax).some(q => Math.hypot(q.p[0] - o.c[0], q.p[1] - o.c[1]) < ty.r + q.r + gapOf(W, ty, q.ty) - 1e-3);
  for (let guard = 0; guard < 100000; guard++) {
    const list = candidates(W, ty, {}); let pick = null;
    if (order === "random") { const ok = list.filter(o => !o.dead && !(ovl(o) && (o.dead = true))); if (ok.length) pick = ok[Math.floor(rnd() * ok.length)]; }
    else { const L = order === "front" ? [...list].sort((a, b) => Math.abs(a.depth) - Math.abs(b.depth)) : list;
      for (const o of L) { if (o.dead) continue; if (ovl(o)) { o.dead = true; continue; } pick = o; break; } }
    if (!pick) break;
    const q = { id: W.piles.length, p: pick.c, r: ty.r, ty, kind: pick.kind, depth: pick.depth, state: "placed", zone: pick.z };
    W.piles.push(q); W.hash.add(q); pick.dead = true;
  }
  return { cov: simCoverage(W), piles: W.piles.length, r: ty.r };
};
// after a live run: free spots still left for each truck size (no overlap), and how many of them a truck could reach
window.residual = (W) => W.types.map(ty => {
  const list = candidates(W, ty, {}); let free = 0, reach = 0;
  for (const o of list) { if (o.dead) continue;
    if (W.hash.near(o.c, ty.r + W.rmax + W.gmax).some(q => Math.hypot(q.p[0] - o.c[0], q.p[1] - o.c[1]) < ty.r + q.r + gapOf(W, ty, q.ty) - 1e-3)) continue;
    free++; if (stopCell(W, o.c, ty.r, W.Wmax, o.kind === "zone", true) >= 0) reach++; }
  return { type: ty.name, r: +ty.r.toFixed(2), free, reach, freeAreaPct: +(free * Math.PI * ty.r * ty.r / Math.abs(area(S.poly)) * 100).toFixed(2) }; });
// after a live run (mutates W): how many MORE piles of the smallest size fit on top of the finished layout?
// reachOnly = a truck must still be able to reach the spot (as in the live run); otherwise geometry only
window.extraFill = (W, reachOnly = false) => {
  const ty = W.smallest, ovl = o => W.hash.near(o.c, ty.r + W.rmax + W.gmax).some(q => Math.hypot(q.p[0] - o.c[0], q.p[1] - o.c[1]) < ty.r + q.r + gapOf(W, ty, q.ty) - 1e-3);
  const before = simCoverage(W); let n = 0;
  for (let guard = 0; guard < 20000; guard++) {
    let pick = null;
    for (const o of candidates(W, ty, {})) { if (o.dead) continue; if (ovl(o)) { o.dead = true; continue; }
      if (reachOnly && (W.minLaneDepth <= o.depth + ty.r || stopCell(W, o.c, ty.r, W.Wmax, o.kind === "zone", true) < 0)) continue; pick = o; break; }
    if (!pick) break;
    const q = { id: W.piles.length, p: pick.c, r: ty.r, ty, kind: pick.kind, depth: pick.depth, state: "placed", zone: pick.z };
    W.piles.push(q); W.hash.add(q); blockAround(W, q); W.version++; refreshReach(W); pick.dead = true; n++;
  }
  return { extraPiles: n, extraPct: +(simCoverage(W) - before).toFixed(2) };
};
// experiment driver: call `await expStep(25000)` repeatedly; results in window.EXP.out
// sites: [[file, entryEdge]], seeds: live runs per site, oracle runs, bounds, saturation of the seed-7 run
window.expPlan = (sites, seeds = [1, 2, 3, 4, 5]) => {
  const q = [];
  for (const [fn, e] of sites) {
    q.push({ fn, e, kind: "bound" });
    for (const s of seeds) q.push({ fn, e, kind: "live", seed: s });
    q.push({ fn, e, kind: "oracle", order: "fill" }, { fn, e, kind: "oracle", order: "front" }, { fn, e, kind: "oracle", order: "random", seed: 1 }, { fn, e, kind: "oracle", order: "random", seed: 2 });
  }
  window.EXP = { q, out: [], cur: null, loaded: null }; return q.length + " tasks";
};
async function expLoad(fn, e) {
  if (EXP.loaded === fn + "|" + e) return;
  await pipe(fn, [], []);
  if (e != null) { const p = S.poly, a = p[e], b = p[(e + 1) % p.length]; S.entry = { i: e, pt: [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2] }; S.sweep = sweepAlongEntry(); run4(); }
  EXP.loaded = fn + "|" + e;
}
window.expStep = async (ms = 25000) => {
  const t0 = performance.now();
  while (performance.now() - t0 < ms) {
    if (!EXP.cur) { EXP.cur = EXP.q.shift(); if (!EXP.cur) return "ALL DONE " + EXP.out.length; EXP.cur.W = null; }
    const T = EXP.cur, site = T.fn.replace("site_", "");
    await expLoad(T.fn, T.e);
    if (T.kind === "bound") {
      const W = newSim("touch", Object.assign({}, S.simCfg, { seed: 7 })), rmin = W.smallest.r, rmax = W.rmax;
      EXP.out.push({ site, kind: "bound", areaHa: +(Math.abs(area(S.poly)) / 1e4).toFixed(1), safePct: +(sumArea(S.safe) / Math.abs(area(S.poly)) * 100).toFixed(1),
        C_rmin: +coverable(rmin).toFixed(1), C_rmax: +coverable(rmax).toFixed(1), rmin: +rmin.toFixed(2), rmax: +rmax.toFixed(2),
        r4plan: Object.values(S.r4.grids).map(g => +g.cov.toFixed(1)) });
      EXP.cur = null; continue;
    }
    if (T.kind === "oracle") { const o = oraclePack(null, T.order, T.seed || 1); EXP.out.push({ site, kind: "oracle", order: T.order + (T.seed ? T.seed : ""), cov: +o.cov.toFixed(2), piles: o.piles }); EXP.cur = null; continue; }
    // live run, sliced
    if (!T.W) T.W = newSim("touch", Object.assign({}, S.simCfg, { seed: T.seed }));
    const W = T.W; while (!W.done && W.t < 400 * 3600 && performance.now() - t0 < ms) stepSim(W, 1);
    if (W.done || W.t >= 400 * 3600) {
      const s = simSummary(W), rec = { site, kind: "live", seed: T.seed, cov: +s.cov.toFixed(2), piles: s.piles, h: +s.hours.toFixed(1), c: s.collisions, d: s.deadlocks, forced: s.forcedDumps || 0, done: W.done };
      if (T.seed === EXP.satSeed || (EXP.satSeed == null && T.seed === 1)) { rec.residual = residual(W); rec.extraReach = extraFill(W, true); rec.extraGeom = extraFill(W, false); }
      EXP.out.push(rec); EXP.cur = null;
    }
  }
  return "running " + (EXP.cur ? EXP.cur.fn + " " + EXP.cur.kind : "") + " | done " + EXP.out.length + " | left " + EXP.q.length;
};
// turning-radius audit: wraps release(); every vertex of every dispatched route must satisfy
// turn angle <= (adjacent distance) / Rt (discrete curvature), or be a recorded three-point turn.
// The corner into the dump spot (in) and out of it (out) are the reverse-in manoeuvre and are exempt.
window.auditTurns = () => {
  const A = window.__turnAudit = { routes: 0, corners: 0, smooth: 0, sharp: 0, kturns: 0 };
  if (!window.__relOrig) window.__relOrig = release;
  window.release = function (W, ty) { const tr = __relOrig(W, ty); if (!tr) return tr; A.routes++; A.kturns += tr.kturns || 0;
    for (const [pts, end] of [[tr.path, "in"], [tr.ro.path, "out"]]) { const P = simplifyPath(pts);
      for (let i = 1; i < P.length - 1; i++) { if ((end === "in" && i === P.length - 2) || (end === "out" && i === 1)) continue;
        const a = P[i - 1], b = P[i], c = P[i + 1]; let d = Math.abs(Math.atan2(c[1] - b[1], c[0] - b[0]) - Math.atan2(b[1] - a[1], b[0] - a[0])) % (2 * Math.PI); if (d > Math.PI) d = 2 * Math.PI - d;
        if (d < 0.02) continue; A.corners++;
        const room = (Math.hypot(b[0] - a[0], b[1] - a[1]) + Math.hypot(c[0] - b[0], c[1] - b[1])) / 2;
        if (d <= room / ty.Rt * 1.05) A.smooth++; else A.sharp++; } } return tr; };
  return A;
};
