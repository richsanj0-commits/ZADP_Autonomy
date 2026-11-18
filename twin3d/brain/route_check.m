function [ok, why] = route_check(poses, piles, L, P)
%ROUTE_CHECK  Is a swept set of body poses clear of every pile (by P.brain.clearPile) and inside the
%   site/apron (by P.brain.clearBound)? why = short reason when not. piles = N-by-2 centres.
ok = true; why = '';
r = P.brain.pileR;
if ~isempty(piles)
    m = 0.1 + r;                                           % body reach from the axle is < 0.09 m
    sel = piles(:,1) > min(poses(:,1))-m & piles(:,1) < max(poses(:,1))+m & ...
          piles(:,2) > min(poses(:,2))-m & piles(:,2) < max(poses(:,2))+m;
    if any(sel)
        C = piles(sel,:);
        [d, ip, ic] = rect_circle_clear(poses, C, r, P);
        if d < P.brain.clearPile
            ok = false; why = sprintf('pile at (%.3f, %.3f) %.1f mm from the body at (%.3f, %.3f)', ...
                C(ic,1), C(ic,2), 1000*d, poses(ip,1), poses(ip,2));
            return
        end
    end
end
% site edges: skip the outline test when every axle point is deep inside (body reach < 0.09 m)
if all(in_allowed(L, poses(:,1), poses(:,2), 0.09 + P.brain.clearBound)), return; end
[X, Y] = body_outline(poses, P, 0, 0.012);
in = in_allowed(L, X, Y, P.brain.clearBound);
if ~all(in(:))
    [i, ~] = find(~in, 1);
    ok = false; why = sprintf('body leaves the site at (%.3f, %.3f)', poses(i,1), poses(i,2));
end
end
