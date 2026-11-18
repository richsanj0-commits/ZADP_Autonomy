function [r, h] = pile_growth(frac, r0, h0)
%PILE_GROWTH  Visible pile while it is being dumped (for the 3D phase).
%   [r, h] = pile_growth(frac, r0, h0): frac = fraction of the dump time elapsed (0..1, clamped);
%   r0, h0 = full pile radius and height (any one scale). The repose angle stays constant (h/r = h0/r0)
%   and the volume grows in proportion to frac: V(frac) = frac * V0, so r = r0*frac^(1/3), h = h0*frac^(1/3).
f = min(max(frac, 0), 1);
k = f .^ (1/3);
r = r0 .* k;
h = h0 .* k;
end
