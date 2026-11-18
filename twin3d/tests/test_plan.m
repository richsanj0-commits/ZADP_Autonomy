% test_plan  Checks plan_arena.json (1 x 1 m, 1:200): loads, zones numbered 1..N, boundary in the arena, spots inside the site, sane trucks and spine.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'plan'));
plan = load_plan(fullfile(here, '..', 'plan', 'plan_arena.json'));
assert(plan.version == 1, 'version');
assert(numel(plan.zones) >= 1, 'no zones');
assert(isequal(sort([plan.zones.n]), 1:numel(plan.zones)), 'zone numbers must be 1..N');
B = plan.boundary; tol = 1e-6;
assert(size(B, 2) == 2 && all(B(:,1) >= -tol & B(:,1) <= plan.arena.w + tol & B(:,2) >= -tol & B(:,2) <= plan.arena.h + tol), 'boundary outside arena');
for i = 1:numel(plan.trucks)
    t = plan.trucks(i); assert(t.pileR > 0, 'pileR must be > 0');
    s = t.spots; assert(~isempty(s), 'truck %d has no spots', i);
    [in, on] = inpolygon([s.x], [s.y], B(:,1), B(:,2));
    assert(all(in | on), 'truck %d: %d spots outside the boundary', i, sum(~(in | on)));
    assert(isequal(sort([s.order]), 1:numel(s)), 'truck %d: orders must be 1..n', i);
end
assert(isequal(size(plan.spine.inLine), [2 2]) && isequal(size(plan.spine.outLine), [2 2]), 'spine lines must be 2-by-2');
% fixed 1:200 arena, three robots, robot-sized lanes and zones
assert(abs(plan.scale - 0.005) < 1e-9, 'scale must be 0.005');
assert(plan.arena.w == 1 && plan.arena.h == 1, 'arena must be 1 x 1');
assert(numel(plan.trucks) == 1, 'exactly one truck type');
t = plan.trucks(1);
assert(t.count == 3 && abs(t.L - 0.100) < 1e-6 && abs(t.W - 0.084) < 1e-6, 'truck must be count 3, 0.100 x 0.084 m');
assert(t.pileR >= 0.02 && t.pileR <= 0.06, 'pileR %.4f outside 0.02..0.06 m', t.pileR);
assert(plan.spine.width >= 0.24 - 1e-6, 'spine width %.4f < 0.24 m (two 0.12 m lanes)', plan.spine.width);
for i = 1:numel(plan.zones)
    zp = plan.zones(i).poly; side = min(max(zp(:,1)) - min(zp(:,1)), max(zp(:,2)) - min(zp(:,2)));
    assert(side >= 0.084, 'zone %d shorter side %.4f < 0.084 m', i, side);
end
m = min([B(:,1); B(:,2); plan.arena.w - B(:,1); plan.arena.h - B(:,2)]);
assert(m >= 0.04 - tol, 'boundary only %.4f m from the arena edge (need 0.04)', m);
disp('test_plan PASSED');
