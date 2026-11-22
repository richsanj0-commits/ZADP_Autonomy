function ok = in_allowed(L, x, y, m, which)
%IN_ALLOWED  True where point (x,y) lies inside the site polygon, the apron or the gate mouth with
%   margin m >= 0 (inside one of these pieces and at least m from its boundary).
%   which = 'all' (default) or 'site' (site polygon only).
%   The site polygon may be non-convex (an L): then the test is exact point-in-polygon (crossing
%   number) plus the distance to every edge segment. A convex site uses the equivalent, faster
%   inward-normal test (signed distance to every edge line >= m).
if nargin < 4, m = 0; end
if nargin < 5, which = 'all'; end
if L.convex
    ok = inConvex(L.bndN, L.bndD, x, y, m);
else
    ok = inPoly(L.bndA, L.bndB, x, y, m);
end
if strcmp(which, 'site'), return; end
ok = ok | inRect(L.apronRect, x, y, m) | inRect(L.mouthRect, x, y, m);
end

function in = inRect(R, x, y, m)
in = x >= R(1)+m & x <= R(2)-m & y >= R(3)+m & y <= R(4)-m;
end

function in = inConvex(N, D, x, y, m)
% convex polygon as inward unit normals N (n-by-2) and offsets D: inside where N*[x;y] - D >= m
in = true(size(x));
for i = 1:size(N,1)
    in = in & (x*N(i,1) + y*N(i,2) - D(i) >= m);
end
end

function in = inPoly(A, B, x, y, m)
% simple polygon, edges A(i,:) -> B(i,:): crossing number for inside, segment distance for the margin
in = false(size(x)); dmin = Inf(size(x));
for i = 1:size(A,1)
    ax = A(i,1); ay = A(i,2); bx = B(i,1); by = B(i,2);
    c = (ay > y) ~= (by > y);
    xc = ax + (y - ay).*(bx - ax)./(by - ay + (by == ay));
    in = xor(in, c & x < xc);
    if m > 0
        ex = bx - ax; ey = by - ay;
        t = min(max(((x - ax)*ex + (y - ay)*ey)/(ex^2 + ey^2), 0), 1);
        dmin = min(dmin, hypot(x - ax - t*ex, y - ay - t*ey));
    end
end
if m > 0, in = in & dmin >= m; end
end
