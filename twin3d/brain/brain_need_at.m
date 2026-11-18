function nd = brain_need_at(L, P, pose)
%BRAIN_NEED_AT  Logical 1-by-nSec: sections the (inflated) body at one pose touches.
[X, Y] = body_outline(pose, P, P.brain.clearSec, 0.01);
id = brain_sections(L, X(:), Y(:));
nd = false(1, L.nSec); nd(id(id > 0)) = true;
end
