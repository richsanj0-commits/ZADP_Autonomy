function bad = brain_owner_check(W, P, poses)
%BRAIN_OWNER_CHECK  Rule 3 invariant (monitor e): every section a robot's TRUE body touches is held
%   by that robot. bad(i) = true when robot i's body (no inflation) reaches a section W.owner does not
%   give to i. poses (nb-by-3) = true poses (default: from W). Sections are exclusive, so this holding
%   at every step is what makes robot-robot overlaps impossible by construction.
if nargin < 3, poses = reshape([W.bots.s], 1, []); poses = vertcat(poses.pose); end
nb = size(poses, 1);
[X, Y] = body_outline(poses, P, 0, 0.01);
id = brain_sections(W.L, X, Y);
bad = false(1, nb);
for i = 1:nb
    s = id(i, :); s = s(s > 0);
    bad(i) = any(W.owner(s) ~= i);
end
end
