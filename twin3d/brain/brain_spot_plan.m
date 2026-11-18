function [W, R] = brain_spot_plan(W, k)
%BRAIN_SPOT_PLAN  Cached trip plan for spot k against the current piles and promised piles.
%   A plan stays valid until a new pile (or promise) lands near its swept path (brain_piles_changed).
%   A spot whose plan search fails with no promised pile nearby is marked dead for good: piles only
%   ever get added, so a blocked approach never opens again (the reason is kept for the report).
s = W.spots(k);
if s.planOK
    R = s.plan; return
end
if s.failVer == W.ver
    R = s.plan; return
end
[C, promised] = brain_obstacles(W, k);
R = route_to_spot(W.L, W.P, s, C);
W.nPlans = W.nPlans + 1;
W.spots(k).plan = R;
if R.ok
    W.spots(k).planOK = true; W.spots(k).why = '';
else
    W.spots(k).planOK = false;
    W.spots(k).why = sprintf('no approach clears the piles (all %d directions x %d stop distances); straight one: %s', ...
        numel(W.P.brain.fanDeg), numel(W.P.brain.dumpExtra), R.why);
    near = ~isempty(promised) && any(hypot(promised(:,1)-s.x, promised(:,2)-s.y) < 0.35);
    if near
        W.spots(k).failVer = W.ver;                % a promised pile is involved: try again later
    else
        W.spots(k).status = 'dead';
    end
end
end
