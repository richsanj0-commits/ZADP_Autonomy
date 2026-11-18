function [overlap, gap] = rect_pair(pa, pb, P)
%RECT_PAIR  Two robot bodies (axle poses [x y th]): overlap by the separating-axis test, and the gap
%   between them (0 when they overlap). Body = P.bot.axleToNose ahead, axleToBack behind, chassisWidth wide.
A = corners(pa, P); B = corners(pb, P);
overlap = true;
for Q = {A, B}
    C = Q{1};
    for e = 1:2
        ax = C(e+1,:) - C(e,:); ax = [-ax(2), ax(1)];
        pA = A*ax'; pB = B*ax';
        if max(pA) < min(pB) || max(pB) < min(pA), overlap = false; break; end
    end
    if ~overlap, break; end
end
if overlap, gap = 0; return; end
gap = min(ptSeg(A, B), ptSeg(B, A));
end

function C = corners(p, P)
a = P.bot.axleToNose; b = P.bot.axleToBack; w = P.bot.chassisWidth/2;
l = [a -w; a w; -b w; -b -w];
c = cos(p(3)); s = sin(p(3));
C = [p(1) + c*l(:,1) - s*l(:,2), p(2) + s*l(:,1) + c*l(:,2)];
end

function d = ptSeg(Pts, C)
% smallest distance from the points Pts to the edges of polygon C
d = Inf;
for i = 1:4
    a = C(i,:); b = C(mod(i,4)+1,:); ab = b - a;
    t = min(max(((Pts(:,1)-a(1))*ab(1) + (Pts(:,2)-a(2))*ab(2))/(ab*ab'), 0), 1);
    d = min(d, min(hypot(Pts(:,1) - a(1) - t*ab(1), Pts(:,2) - a(2) - t*ab(2))));
end
end
