function ok = in_allowed(L, x, y, m, which)
%IN_ALLOWED  True where point (x,y) lies inside the site polygon, the apron or the gate mouth with
%   margin m (signed distance to every edge of one of these convex pieces >= m).
%   which = 'all' (default) or 'site' (site polygon only).
if nargin < 4, m = 0; end
if nargin < 5, which = 'all'; end
ok = inConvex(L.bndN, L.bndD, x, y, m);
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
