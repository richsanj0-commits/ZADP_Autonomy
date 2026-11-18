function hp = pile_draw(ax, c, r, h, P, key)
%PILE_DRAW  A dirt pile as a flat-shaded cone surface at c = [x y] (arena m), base radius r, height h.
%   hp = pile_draw(ax, c, r, h, P, key). The base ring is an exact circle of radius r and the apex is exactly h
%   above the base, so the pile fills the radius the planner and the safety monitor use; the slopes in between are
%   slumped a little (rings pulled in by up to 12 %, heights lowered by up to 6 %) and every vertex has its own
%   brightness, so it reads as a heap of dirt. The irregularity is fixed per key (e.g. the spot number): the same
%   pile looks the same while it grows (pile_update) and when it is finished.
%   hp.surf = the surface handle, hp.base = base height (m, top of the pad), hp.key = key.
if nargin < 6, key = 0; end
nth = 24; nr = 5;                                  % angular facets, rings from the base (j = 0) to the apex (j = nr-1)
s = RandStream('mt19937ar', 'Seed', 7000 + round(key));
th = linspace(0, 2*pi, nth + 1);
u = smooth_ring(rand(s, 1, nth), 2); v = smooth_ring(rand(s, 1, nth), 2);
rk = ones(nr, nth); zk = ones(nr, nth);
for j = 2:nr-1
    rk(j,:) = 1 - 0.12*u*(j/nr)*2;                 % pull inner rings in (never out: base radius stays r)
    zk(j,:) = 1 - 0.06*v;
end
rk(:, nth + 1) = rk(:, 1); zk(:, nth + 1) = zk(:, 1);
shade = 0.86 + 0.26*rand(s, nr, nth); shade(:, nth + 1) = shade(:, 1);
col = repmat(reshape(P.pile.color, 1, 1, 3), nr, nth + 1) .* shade;
col(nr, :, :) = reshape(repmat(P.pile.color*1.12, nth + 1, 1), 1, nth + 1, 3);      % lighter, freshly tipped top
col = min(col, 1);
hp.surf = surface('Parent', ax, 'XData', zeros(nr, nth + 1), 'YData', zeros(nr, nth + 1), 'ZData', zeros(nr, nth + 1), 'CData', reshape(col, nr, nth + 1, 3), ...
    'FaceColor', 'flat', 'EdgeColor', 'none', 'FaceLighting', 'flat', 'AmbientStrength', 0.55, ...
    'DiffuseStrength', 0.8, 'SpecularStrength', 0, 'Clipping', 'off');
hp.th = th; hp.rk = rk; hp.zk = zk; hp.nr = nr; hp.c = c(:)'; hp.key = key; hp.base = 0.0060;
pile_update(hp, r, h);
end

function y = smooth_ring(x, k)
% circular moving average, so neighbouring facets are correlated
y = x;
for i = 1:k, y = (circshift(y, 1) + 2*y + circshift(y, -1))/4; end
y = (y - min(y))/max(max(y) - min(y), eps);
end
