function s = brain_view_state(W, P)
%BRAIN_VIEW_STATE  What the 3D world needs from a running brain world W (for world_live).
%   s = brain_view_state(W, P) in the form world_sync takes: s.pose (nb-by-3), s.bed (1-by-nb deg), s.truck (1-by-nb),
%   s.piles = the finished piles (frac 1, from the pile records) followed by the piles being dumped right now
%   (frac from dump_profile): fields c, r, h (arena m, full size), frac, key (spot number). See brain_dump_state.
d = brain_dump_state(W, P);
pr = W.pileRec; n = numel(pr);
piles = struct('c', cell(1, n), 'r', 0, 'h', 0, 'frac', 1, 'key', 0);
for k = 1:n
    piles(k) = struct('c', W.piles(k,:), 'r', pr(k).rArena, 'h', pr(k).hArena, 'frac', 1, 'key', pr(k).spot);
end
for i = find(d.dump(:,6) > 0)'
    piles(end+1) = struct('c', d.dump(i,1:2), 'r', d.dump(i,3), 'h', d.dump(i,4), 'frac', d.dump(i,6), ...
        'key', d.dump(i,5)); %#ok<AGROW>
end
s = struct('pose', d.pose, 'bed', d.bed', 'truck', d.truck', 'piles', piles);
end
