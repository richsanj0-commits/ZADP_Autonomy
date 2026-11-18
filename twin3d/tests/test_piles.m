% TEST_PILES  Phase 5: pile maths, truck numbering, pile records, coverage.
%   Formula against the plan radius (500 t), monotonic in payload, V = pi r^2 h / 3, growth helper keeps
%   the repose angle and ends at the full pile, load factors in range, truck numbers strictly increasing
%   with no repeats over a 300 s run (each one a new truck, 3 robots), every pile record complete,
%   coverage between 0 and 1 and equal to a hand-computed union for two circles, parking single source.
%   Prints "test_piles PASSED".
here = fileparts(mfilename('fullpath'));
P = twin_params();
plan = load_plan(fullfile(here, '..', 'plan', 'plan_arena.json'));
sc = P.scale.arenaPerSite;

% formula against the plan
[r, h, V, A] = pile_from_payload(plan.trucks(1).payload, P);
assert(abs(r*sc/plan.trucks(1).pileR - 1) < 0.01, 'pile_from_payload(500) %.4f vs plan %.4f', r*sc, plan.trucks(1).pileR);
assert(abs(V - 500/1.8) < 1e-9 && abs(A.r - r*sc) < 1e-12 && abs(A.h - h*sc) < 1e-12 && abs(A.V - V*sc^3) < 1e-12, 'arena scale');
% monotonic and volume law
pl = 100:25:700;
[rr, hh, VV] = pile_from_payload(pl, P);
assert(all(diff(rr) > 0) && all(diff(hh) > 0) && all(diff(VV) > 0), 'not monotonic in payload');
assert(max(abs(VV - pi*rr.^2.*hh/3)./VV) < 1e-12, 'V ~= pi r^2 h / 3');
assert(max(abs(hh./rr - tand(P.pile.reposeDeg))) < 1e-12, 'repose angle');
% growth helper
[r0, h0] = pile_growth(1, r, h);
assert(abs(r0 - r) < 1e-12 && abs(h0 - h) < 1e-12, 'growth must end at the full pile');
fr = linspace(0.01, 1, 50); [rg, hg] = pile_growth(fr, r, h);
assert(max(abs(hg./rg - h/r)) < 1e-12, 'growth must keep the repose angle');
assert(max(abs(pi*rg.^2.*hg/3 - fr*V)) < 1e-9*V, 'growth volume must be proportional to frac');
assert(all(diff(rg) > 0) && all(diff(hg) > 0), 'growth must be monotonic');
[r00, h00] = pile_growth(-0.5, r, h); assert(r00 == 0 && h00 == 0, 'growth at 0');
[r11, ~] = pile_growth(2, r, h); assert(r11 == r, 'growth clamps at 1');

% parking: one source, P1 under the IN lane
assert(isequal(size(P.brain.parking), [3 3]), 'P.brain.parking must be 3-by-3');
L = brain_layout(plan, P); assert(isequal(L.slot, P.brain.parking), 'layout parking');

% a 300 s run
[W, log] = brain_run(plan, P, struct('seed', 1, 'maxTime', 300));
m = log.mon;
assert(m.rrOverlapSteps == 0 && m.pileHitSteps == 0 && m.boundSteps == 0 && m.deadlocks == 0, 'safety monitors not zero');
ev = W.events; assigned = ev(strcmp({ev.type}, 'assigned'));
tn = [assigned.truck];
assert(numel(tn) >= 3 && isequal(tn, 1:numel(tn)), 'truck numbers must be 1,2,3,... strictly increasing, no repeats: %s', mat2str(tn));
assert(W.truckNo == numel(tn), 'truck counter');
assert(all([ev.truck] >= 1), 'every event carries a truck number');
% a robot that left and came back is a new truck at its next trip
ret = ev(strcmp({ev.type}, 'returned'));
assert(~isempty(ret), 'no robot returned to parking in 300 s');
for q = 1:numel(ret)
    nxt = assigned([assigned.bot] == ret(q).bot & [assigned.t] > ret(q).t);
    if ~isempty(nxt), assert(nxt(1).truck > ret(q).truck, 'returning robot must become a new truck'); end
end
% pile records
pr = W.pileRec; assert(numel(pr) == size(W.piles, 1) && numel(pr) >= 1, 'one record per pile');
need = {'truck','bot','spot','order','zone','payload','V','r','h','Varena','rArena','hArena','t'};
for q = 1:numel(need)
    v = [pr.(need{q})]; assert(numel(v) == numel(pr) && all(isfinite(v)) && all(v >= 0), 'record field %s incomplete', need{q});
end
lf = [pr.payload]/plan.trucks(1).payload;
assert(all(lf >= P.pile.loadMin - 1e-12 & lf <= P.pile.loadMax + 1e-12), 'load factor out of range');
assert(all([pr.rArena] <= plan.trucks(1).pileR + 1e-12), 'pile larger than planned');
assert(max(abs([pr.V] - pi*[pr.r].^2.*[pr.h]/3)./[pr.V]) < 1e-9, 'record V');
assert(max(abs([pr.rArena] - sc*[pr.r])) < 1e-12 && max(abs([pr.rArena] - W.pileR')) < 1e-12, 'record radius vs world');
assert(numel(unique([pr.truck])) == numel(pr), 'a truck made two piles');
assert(all(ismember([pr.bot], 1:3)), 'record bot');
% coverage
[c, tons] = brain_coverage(W, plan);
assert(c > 0 && c < 1, 'coverage %.4f', c);
assert(abs(tons - sum([pr.payload])) < 1e-9, 'tonnes');
% coverage of two disjoint circles equals the sum of their areas / polygon area; overlapping ones less than the sum
Wt.piles = [0.3 0.3; 0.6 0.3]; Wt.pileR = [0.03; 0.04]; Wt.pileRec = [];
A2 = (pi*0.03^2 + pi*0.04^2)/polyarea(plan.boundary(:,1), plan.boundary(:,2));
assert(abs(brain_coverage(Wt, plan)/A2 - 1) < 0.01, 'coverage of two circles');
Wt.piles = [0.3 0.3; 0.31 0.3];
assert(brain_coverage(Wt, plan) < 0.99*A2, 'overlap must count once');
fprintf('  300 s: %d trucks numbered, %d piles, coverage %.2f %%, %.0f t\n', W.truckNo, numel(pr), 100*c, tons);
disp('test_piles PASSED');
