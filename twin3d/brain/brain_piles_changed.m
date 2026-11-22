function W = brain_piles_changed(W, p)
%BRAIN_PILES_CHANGED  A pile (or promise) appeared at p = [x y]: invalidate cached plans whose swept
%   path passes within reach of it, and bump the world version (re-ranks the fill order).
reach = W.L.reach + W.P.brain.pileR + W.P.brain.clearPile + 0.01;
for k = 1:numel(W.spots)
    s = W.spots(k);
    if ~isempty(s.plan) && ~isempty(s.plan.axle)
        if min(hypot(s.plan.axle(:,1)-p(1), s.plan.axle(:,2)-p(2))) < reach
            W.spots(k).planOK = false; W.spots(k).epoch = W.spots(k).epoch + 1;
        end
    end
end
W.ver = W.ver + 1;
end
