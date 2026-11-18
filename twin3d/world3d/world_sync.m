function world_sync(h, s)
%WORLD_SYNC  Put the world into a state: trucks and piles. Used by world_play, world_live and View3DSystem.
%   s.pose (nb-by-3 true poses), s.bed (nb bed angles, deg), s.truck (nb truck numbers, 0 = none),
%   s.piles: struct array with c (1-by-2), r, h (arena m, FULL size), frac (0..1: share of the pile that exists
%   now, 0 = not drawn yet, 1 = finished), key (spot number: the same key is the same pile while it grows).
%   A pile is created the first time it has frac > 0 and only resized afterwards; finished piles are never
%   touched again (they stay).
for i = 1:min(h.nTrucks, size(s.pose, 1))
    truck_pose(h.truck(i), s.pose(i,:), s.bed(i), s.truck(i));
end
if isempty(s.piles), return; end
obj = getappdata(h.fig, 'pileObj');
have = [obj.key];
for q = 1:numel(s.piles)
    p = s.piles(q);
    if p.frac <= 0.005, continue; end
    [r, hh] = pile_growth(p.frac, p.r, p.h);
    j = find(have == p.key, 1);
    if isempty(j)
        hp = pile_draw(h.ax, p.c, r, hh, h.P, p.key);
        obj(end+1) = struct('key', p.key, 'hp', hp, 'frac', p.frac); %#ok<AGROW>
        have(end+1) = p.key; %#ok<AGROW>
    elseif obj(j).frac < 1 && abs(obj(j).frac - p.frac) > 1e-9
        obj(j).hp.c = p.c(:)';
        pile_update(obj(j).hp, r, hh);
        obj(j).frac = p.frac;
    end
end
setappdata(h.fig, 'pileObj', obj);
end
