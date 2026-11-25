% TEST_MAPS  The same brain code and parameters on seven more site shapes, plus the reference arena (8 maps).
%   Part 1, the four arena shapes in plan\ (arena = square with a cut corner, rect = wide rectangle,
%   lshape = L with an arm up the left side, a NON-CONVEX site, hex = irregular hexagon): a shortened fill
%   (opts.maxTime = 400 s of sim time, seed 1) on each, asserting zero robot-robot overlaps, pile hits,
%   off-site, section-ownership violations and deadlocks, the zone lock (never two robots in one zone),
%   at least two robots inside at once, and at least one pile in every zone (all zones have spots that
%   are reachable within 400 s). Full fills: demo_maps (plan\maps_results.md).
%   Part 2, the four extra convex shapes in maps\ (trapezoid, chamfered, mirrored pentagon, octagon): the
%   plan is supported and the first 400 s are clean too.
%   Part 3: a self-intersecting site polygon is refused at init with a clear message.
%   Prints "test_maps PASSED". About 20 s per map.
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
P = twin_params();
files = [fullfile(root, 'plan', strcat('plan_', {'arena', 'rect', 'lshape', 'hex'}, '.json')), ...
         fullfile(root, 'maps', strcat('plan_', {'trap', 'chamfer', 'mirror', 'octa'}, '.json'))];
tAll = tic;
for q = 1:numel(files)
    [~, name] = fileparts(files{q});
    plan = load_plan(files{q});
    assert_plan_supported(plan, P);
    [W, log] = brain_run(plan, P, struct('seed', 1, 'maxTime', 400));
    m = log.mon;
    assert(m.rrOverlapSteps == 0, '%s: robot-robot overlap', name);
    assert(m.pileHitSteps == 0, '%s: robot-pile overlap', name);
    assert(m.boundSteps == 0, '%s: body outside the site/apron', name);
    assert(m.ownerSteps == 0, '%s: section-ownership violation', name);
    assert(m.deadlocks == 0 && ~m.deadlock, '%s: deadlock', name);
    nb = size(log.pose, 3);
    for z = 1:numel(plan.zones)
        zp = plan.zones(z).poly; c = zeros(numel(log.t), 1);
        for i = 1:nb, c = c + inpolygon(log.pose(:,1,i), log.pose(:,2,i), zp(:,1), zp(:,2)); end
        assert(all(c <= 1), '%s: two robots in zone %d', name, plan.zones(z).n);
    end
    B = plan.boundary; inside = zeros(numel(log.t), 1);
    for i = 1:nb, inside = inside + inpolygon(log.pose(:,1,i), log.pose(:,2,i), B(:,1), B(:,2)); end
    assert(m.maxInside >= 2 && any(inside >= 2), '%s: never two robots inside at once', name);
    placed = strcmp({W.spots.status}, 'placed');
    zs = [W.spots.zone]; kinds = {W.spots.kind};
    for z = [plan.zones.n]
        has = zs == z & ~strcmp(kinds, 'lane');
        if any(has), assert(any(placed & has), '%s: no pile in zone %d after 400 s', name, z); end
    end
    fprintf('  %-12s %3d / %3d piles in 400 s, min robot gap %.1f mm, min pile gap %.1f mm, all monitors 0, %.0f s wall\n', ...
        name, nnz(placed), numel(W.spots), 1000*m.minRR, 1000*m.minPile, log.wall);
end
% a self-intersecting site polygon is refused at init with a clear message
plan = load_plan(files{1});
plan.boundary = [0.05 0.05; 0.95 0.05; 0.05 0.95; 0.95 0.95];
try
    brain_init(plan, P, 1); ok = false;
catch err
    ok = contains(err.message, 'simple');
end
assert(ok, 'a self-intersecting site must be refused with a "simple" message');
fprintf('  self-intersecting site refused: %s\n', err.message);
fprintf('  test_maps runtime %.0f s\n', toc(tAll));
disp('test_maps PASSED');
