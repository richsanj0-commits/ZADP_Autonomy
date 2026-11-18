function p = solid_prism(par, xz, y, col)
%SOLID_PRISM  Convex profile in the x-z plane (N-by-2, counter-clockwise) extruded between y(1) and y(2).
n = size(xz, 1);
V = [xz(:,1), y(1)*ones(n,1), xz(:,2); xz(:,1), y(2)*ones(n,1), xz(:,2)];
F = NaN(n + 2, max(n, 4));
F(1, 1:n) = 1:n; F(2, 1:n) = n + (1:n);
for i = 1:n
    j = mod(i, n) + 1;
    F(2 + i, 1:4) = [i j n+j n+i];
end
p = patch('Parent', par, 'Vertices', V, 'Faces', F, 'FaceColor', col, 'EdgeColor', 'none', ...
    'FaceLighting', 'flat', 'AmbientStrength', 0.55, 'DiffuseStrength', 0.75, 'SpecularStrength', 0.05);
end
