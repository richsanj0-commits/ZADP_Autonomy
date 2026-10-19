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
  for (let k = 0; k < 400 && /…/.test($("progMsg").textContent); k++) await new Promise(z => setTimeout(z, 250));
  r.segs = S.segs.map(s => s.t === "arc" ? "A" : "L").join("");
  r.read = S.dims.filter(d => d.src === "sketch").map(d => dimName(d).split(" ")[0] + "=" + d.value).join(" ");
  if (!S.dims.some(d => d.value > 0)) { const w = S.dims.find(d => d.kind === "width"); w.value = truth[fn][0]; w.src = "user"; r.note = "width typed"; }
  $("buildBtn").click();
  if (truth[fn] && truth[fn][1]) r.areaErr = ((Math.abs(area(S.poly)) - truth[fn][1]) / truth[fn][1] * 100).toFixed(1) + "%";
  $("areaName").value = fn; $("saveArea").click(); setRound(3);
  r.usable = ($("stats3").innerText.match(/Usable\n([\d.]+)/) || [])[1] + "%";
  S.sweep = null; setRound(4);
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
