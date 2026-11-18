function cov = coverage_curve(centres, radii, plan)
%COVERAGE_CURVE  Coverage after each pile: cov(k+1) = union area of piles 1..k inside the site polygon / polygon
%   area, cov(1) = 0. Same definition and 1 mm grid as brain_coverage, built incrementally (one pass), so a replay
%   can show the coverage at any moment; cov(end) equals brain_coverage of the whole run.
B = plan.boundary;
areaPoly = abs(polyarea(B(:,1), B(:,2)));
dx = 0.001;
xs = (min(B(:,1)) + dx/2):dx:max(B(:,1)); ys = (min(B(:,2)) + dx/2):dx:max(B(:,2));
[X, Y] = meshgrid(xs, ys);
inSite = inpolygon(X, Y, B(:,1), B(:,2));
hit = false(size(X));
n = size(centres, 1);
cov = zeros(n + 1, 1);
for k = 1:n
    r = radii(k);
    ix = find(xs >= centres(k,1) - r & xs <= centres(k,1) + r);
    iy = find(ys >= centres(k,2) - r & ys <= centres(k,2) + r);
    if ~isempty(ix) && ~isempty(iy)
        sub = (X(iy,ix) - centres(k,1)).^2 + (Y(iy,ix) - centres(k,2)).^2 <= r^2;
        hit(iy,ix) = hit(iy,ix) | (sub & inSite(iy,ix));
    end
    cov(k + 1) = sum(hit(:)) * dx^2 / areaPoly;
end
end
