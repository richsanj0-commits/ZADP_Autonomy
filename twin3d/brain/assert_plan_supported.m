function assert_plan_supported(plan, P)
%ASSERT_PLAN_SUPPORTED  Check, with a clear message, every map assumption the brain makes (called by brain_init).
%   The brain (brain_layout, brain_sections, route_*) supports this map family:
%     1. site polygon simple (convex or not: in_allowed and the monitors handle inner corners)
%     2. entry on the bottom edge: entry normal +y, that edge exactly horizontal (y = entry y), so the spine is
%        vertical and the apron sits below it
%     3. the plan's IN lane x equals parking slot P1 x (P.brain.parking(1,1)): P1 is the queue head under the IN
%        lane, and the apron loop (return row, east column at P3) lies east of the OUT lane
%     4. exactly 2 zones, one on each side of the spine (sections Z1 left, Z2 right), each with at most one
%        branch road
%     5. every spot inside the arena and the site polygon fits the arena (0..P.arena.w, 0..P.arena.h)
%     6. one truck type (plan.trucks(1) is the fleet)
%   Other shapes need layout work first (HANDOVER.md section 9.5). tools/export_plan.py --out writes a new map.
B = plan.boundary; n = size(B, 1);
% 1. simple polygon (convex or not, e.g. an L): no two non-adjacent edges cross
for i = 1:n
    for j = i+2:n
        if i == 1 && j == n, continue; end                       % adjacent through the closing edge
        assert(~segCross(B(i,:), B(mod(i,n)+1,:), B(j,:), B(mod(j,n)+1,:)), ...
            'map not supported: the site polygon must be simple (edges %d and %d cross)', i, j);
    end
end
% 2. entry on a horizontal bottom edge
assert(abs(plan.entry.n(1)) < 1e-9 && plan.entry.n(2) > 0, 'map not supported: the entry must face +y (entry on the bottom edge)');
ye = plan.entry.pt(2);
onEdge = abs(B(:,2) - ye) < 1e-9;
assert(nnz(onEdge) >= 2 && abs(min(B(:,2)) - ye) < 1e-9, ...
    'map not supported: the entry edge must be the horizontal bottom edge of the site (y = %.4f)', ye);
% 3. IN lane above parking P1, apron loop east of the OUT lane
xIn = mean(plan.spine.inLine(:,1)); xOut = mean(plan.spine.outLine(:,1));
assert(abs(xIn - P.brain.parking(1,1)) < 1e-6, ...
    'map not supported: the IN lane x (%.4f) must equal parking P1 x (%.4f): centre the entry like plan_arena or move P.brain.parking', ...
    xIn, P.brain.parking(1,1));
assert(xOut < P.brain.parking(2,1) && xOut > xIn, 'map not supported: the OUT lane must lie between P1 and P2');
% 4. two zones, one each side, at most one branch each
assert(numel(plan.zones) == 2, 'map not supported: exactly 2 zones expected, the plan has %d', numel(plan.zones));
xMid = (min(plan.spine.poly(:,1)) + max(plan.spine.poly(:,1)))/2;
side = arrayfun(@(z) sign(mean(z.poly(:,1)) - xMid), plan.zones);
assert(isequal(sort(side(:))', [-1 1]), 'map not supported: one zone must lie on each side of the spine');
assert(all(arrayfun(@(z) numel(z.branch) <= 1, plan.zones)), 'map not supported: at most one branch road per zone');
% 5. inside the arena
assert(all(B(:,1) >= -1e-9 & B(:,1) <= P.arena.w + 1e-9 & B(:,2) >= -1e-9 & B(:,2) <= P.arena.h + 1e-9), ...
    'map not supported: the site polygon must lie inside the %g x %g m arena', P.arena.w, P.arena.h);
% 6. one truck type
assert(numel(plan.trucks) >= 1 && ~isempty(plan.trucks(1).spots), 'map not supported: the plan has no truck spots');
end

function c = segCross(a, b, p, q)
% proper crossing of segments a-b and p-q
o = @(u, v, w) sign((v(1)-u(1))*(w(2)-u(2)) - (v(2)-u(2))*(w(1)-u(1)));
c = o(a,b,p)*o(a,b,q) < 0 && o(p,q,a)*o(p,q,b) < 0;
end
