function [d, iPose, iCirc] = rect_circle_clear(poses, C, r, P)
%RECT_CIRCLE_CLEAR  Smallest gap between the robot body at each pose and circles (centres C N-by-2, radius r).
%   d < 0 means overlap. Returns the overall minimum and where it occurs. Inf if no circles.
d = Inf; iPose = 0; iCirc = 0;
if isempty(C) || isempty(poses), return; end
dx = C(:,1)' - poses(:,1); dy = C(:,2)' - poses(:,2);     % K-by-N
c = cos(poses(:,3)); s = sin(poses(:,3));
lx = dx.*c + dy.*s; ly = -dx.*s + dy.*c;
ex = max(max(-P.bot.axleToBack - lx, lx - P.bot.axleToNose), 0);
ey = max(abs(ly) - P.bot.chassisWidth/2, 0);
D = hypot(ex, ey) - r;
[d, k] = min(D(:));
if nargout > 1, [iPose, iCirc] = ind2sub(size(D), k); end
end
