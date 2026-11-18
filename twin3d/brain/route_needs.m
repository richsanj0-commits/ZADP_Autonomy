function steps = route_needs(steps, L, P)
%ROUTE_NEEDS  For every step, the sections (logical 1-by-nSec) its swept body touches, inflated by
%   P.brain.clearSec. A robot must hold all of them before it starts that step (Rule 3).
for i = 1:numel(steps)
    po = steps(i).poses;
    if size(po,1) > 60, po = po(unique(round(linspace(1, size(po,1), 60))), :); end
    [X, Y] = body_outline(po, P, P.brain.clearSec, 0.01);
    id = brain_sections(L, X(:), Y(:));
    nd = false(1, L.nSec); nd(id(id > 0)) = true;
    steps(i).need = nd;
end
end
