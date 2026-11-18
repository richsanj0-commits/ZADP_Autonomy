function [cov, tonnes] = brain_coverage(W, plan)
%BRAIN_COVERAGE  Share of the site covered by piles: area of the UNION of the placed pile circles
%   (each with its own radius W.pileR), clipped to the site polygon, divided by the polygon area.
%   [cov, tonnes] = brain_coverage(W, plan). tonnes = total payload dumped (site scale).
%   Numerical, no toolboxes: a grid of 1 mm cells (1000 x 1000 over the arena); a cell counts when its
%   centre is inside the polygon (inpolygon) and inside at least one circle; the polygon area is the
%   shoelace formula. Error is about the cell size times the perimeter, well under 0.5 %.
B = plan.boundary;
areaPoly = abs(polyarea(B(:,1), B(:,2)));
tonnes = 0;
if isfield(W, 'pileRec') && ~isempty(W.pileRec), tonnes = sum([W.pileRec.payload]); end
if isempty(W.piles), cov = 0; return; end
dx = 0.001;
xs = (min(B(:,1)) + dx/2):dx:max(B(:,1)); ys = (min(B(:,2)) + dx/2):dx:max(B(:,2));
[X, Y] = meshgrid(xs, ys);
hit = false(size(X));
for k = 1:size(W.piles, 1)
    r = W.pileR(k);
    ix = find(xs >= W.piles(k,1) - r & xs <= W.piles(k,1) + r);
    iy = find(ys >= W.piles(k,2) - r & ys <= W.piles(k,2) + r);
    if isempty(ix) || isempty(iy), continue; end
    sub = (X(iy,ix) - W.piles(k,1)).^2 + (Y(iy,ix) - W.piles(k,2)).^2 <= r^2;
    hit(iy,ix) = hit(iy,ix) | sub;
end
hit = hit & inpolygon(X, Y, B(:,1), B(:,2));
cov = sum(hit(:)) * dx^2 / areaPoly;
end
